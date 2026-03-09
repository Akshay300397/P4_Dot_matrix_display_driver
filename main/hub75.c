/*
 * hub75.c
 * HUB75 LED Matrix Driver — ESP32-S3 / ESP-IDF v5.x
 *
 * Hardware:  2× P4 64×32 ICN6124EJ panels chained = 128×32 total
 * Interface: SPI2 (HSPI) for data clocking, GPIO for control signals
 * Scan rate: 1:16 (ABCD binary row addressing)
 *
 * Refresh flow per row:
 *   1. OE HIGH       — blank output (prevent ghosting during data load)
 *   2. SPI transmit  — clock 128 bytes into shift registers via MOSI+CLK
 *   3. set_row()     — set ABCD address lines + 1µs settling delay
 *   4. LAT pulse     — latch shift register → ICN6124 output registers
 *   5. OE LOW        — enable output (row lights up)
 *   6. 1µs delay     — row visible time
 *   7. next row      — increment and repeat
 *
 * Bit packing per SPI byte (1 byte = 1 column position):
 *   Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0
 *    0     0    B2    B1    G2    G1    R2    R1
 *
 *   R1/G1/B1 = top half pixel (rows 0–15)
 *   R2/G2/B2 = bottom half pixel (rows 16–31)
 *   Order matches shift register chain after PI→PO jumpers
 */

#include "hub75.h"
#include "framebuffer.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"       // GPIO_OUT_W1TS_REG, GPIO_OUT_W1TC_REG

#include <string.h>

static const char *TAG = "HUB75";

// ─────────────────────────────────────────────────────────────
//  SPI DEVICE HANDLE
// ─────────────────────────────────────────────────────────────

static spi_device_handle_t spi_dev;

// ─────────────────────────────────────────────────────────────
//  SPI DMA TRANSMIT BUFFER
//  128 bytes per row (1 byte per column, 128 columns)
//  Must be 4-byte aligned and in DRAM for DMA access
// ─────────────────────────────────────────────────────────────

// 6 channels × 64 columns = 384 bits = 48 bytes
// Chain order (SPI sends first byte to END of chain):
//   bytes  0– 7 = B2  (end of chain)
//   bytes  8–15 = G2
//   bytes 16–23 = R2
//   bytes 24–31 = B1
//   bytes 32–39 = G1
//   bytes 40–47 = R1  (start of chain)
#define SPI_BUF_BYTES  (PANEL_W * 6 / 8)
static DRAM_ATTR uint8_t spi_tx_buf[SPI_BUF_BYTES] __attribute__((aligned(4)));

// ─────────────────────────────────────────────────────────────
//  FAST GPIO MACROS
//
//  ESP32-S3 ESP-IDF v5.x changed the GPIO register struct layout.
//  The old GPIO.out_w1ts.val syntax no longer compiles on ESP32-S3.
//
//  The correct approach for all ESP32 variants in ESP-IDF v5.x is to
//  write directly to the hardware register addresses using REG_WRITE().
//  These registers are defined in soc/gpio_reg.h:
//
//    GPIO_OUT_W1TS_REG  — Write 1 To Set:   any bit written 1 → that GPIO goes HIGH
//    GPIO_OUT_W1TC_REG  — Write 1 To Clear: any bit written 1 → that GPIO goes LOW
//
//  This is a single 32-bit peripheral register write — one instruction,
//  same ~4ns speed as the old .val approach, fully portable across
//  ESP32, ESP32-S2, ESP32-S3, ESP32-C3 variants.
//
//  Valid for GPIO pins 0–31 only.
//  For pins 32+: use GPIO_OUT1_W1TS_REG / GPIO_OUT1_W1TC_REG
//  All our HUB75 pins are assigned below GPIO 32 so this is safe.
// ─────────────────────────────────────────────────────────────

#define GPIO_FAST_SET(pin)   REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << (pin)))
#define GPIO_FAST_CLR(pin)   REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << (pin)))

// ─────────────────────────────────────────────────────────────
//  SET ROW ADDRESS (ABCD)
//  Writes all 4 address bits atomically to minimize glitch time
//  Then waits HUB75_MUX_DELAY_US for address lines to settle
//  (confirmed needed by PxMatrix test: setMuxDelay(1,1,1,1,1))
// ─────────────────────────────────────────────────────────────

static inline void set_row_address(uint8_t row)
{
    // Build mask of all 4 address pins to clear them first
    uint32_t clear_mask = (1UL << HUB75_PIN_A) |
                          (1UL << HUB75_PIN_B) |
                          (1UL << HUB75_PIN_C) |
                          (1UL << HUB75_PIN_D);

    // Build set mask for only the bits that should be HIGH
    uint32_t set_mask = ((row & 0x01) ? (1UL << HUB75_PIN_A) : 0) |
                        ((row & 0x02) ? (1UL << HUB75_PIN_B) : 0) |
                        ((row & 0x04) ? (1UL << HUB75_PIN_C) : 0) |
                        ((row & 0x08) ? (1UL << HUB75_PIN_D) : 0);

    // Atomic clear then set — all 4 pins updated within 2 register writes
    REG_WRITE(GPIO_OUT_W1TC_REG, clear_mask);
    REG_WRITE(GPIO_OUT_W1TS_REG, set_mask);

    // Wait for address lines to settle before latching
    // ICN6124 requires this — confirmed by PxMatrix setMuxDelay(1,1,1,1,1)
    esp_rom_delay_us(HUB75_MUX_DELAY_US);
}

// ─────────────────────────────────────────────────────────────
//  BUILD SPI BUFFER FOR ONE ROW
//
//  For each column x (0 to PANEL_W-1):
//    - Read top pixel:    framebuffer[row * PANEL_W + x]
//    - Read bottom pixel: framebuffer[(row + SCAN_ROWS) * PANEL_W + x]
//    - Extract R,G,B bits from each (stored as 0b00000RGB)
//    - Pack 6 bits into 1 byte matching shift register chain order
//
//  PI→PO jumper chain order on FM6124 panel:
//    MOSI → [R1] → [G1] → [B1] → [R2] → [G2] → [B2] → end
//
//  SPI sends MSB first. First bit clocked goes to END of chain (B2).
//  Last bit clocked stays at START of chain (R1).
//
//  Therefore byte bit layout:
//    bit5 = B2  (sent first  → travels to end of chain)
//    bit4 = G2
//    bit3 = R2
//    bit2 = B1
//    bit1 = G1
//    bit0 = R1  (sent last   → stays at start of chain)
//
//  Framebuffer pixel format: 0b00000RGB
//    bit2 = R, bit1 = G, bit0 = B
// ─────────────────────────────────────────────────────────────

static inline void build_spi_buffer(const uint8_t *fb, uint8_t row)
{
    // Each of 6 channels gets its own 8-byte (64-bit) block.
    // One bit per column: col 0 = byte 0 MSB, col 63 = byte 7 LSB.
    // Buffer order = chain end first (B2) to chain start last (R1).
    // PI→PO chain: MOSI → R1 → R2 → G1 → G2 → B1 → B2 (end)
    // SPI sends first byte to chain END, last byte to chain START.
    // Buffer order = chain end first → chain start last:
    uint8_t *b2_plane = spi_tx_buf + 0 * (PANEL_W / 8);   // sent first → chain end
    uint8_t *b1_plane = spi_tx_buf + 1 * (PANEL_W / 8);
    uint8_t *g2_plane = spi_tx_buf + 2 * (PANEL_W / 8);
    uint8_t *g1_plane = spi_tx_buf + 3 * (PANEL_W / 8);
    uint8_t *r2_plane = spi_tx_buf + 4 * (PANEL_W / 8);
    uint8_t *r1_plane = spi_tx_buf + 5 * (PANEL_W / 8);   // sent last → chain start (R1)

    memset(spi_tx_buf, 0, SPI_BUF_BYTES);

    const uint8_t *top_row = fb + (row              * PANEL_W);
    const uint8_t *bot_row = fb + ((row + SCAN_ROWS) * PANEL_W);

    for (int x = 0; x < PANEL_W; x++) {
        uint8_t top = top_row[x];  // 0b00000RGB — top half
        uint8_t bot = bot_row[x];  // 0b00000RGB — bottom half

        int byte_idx = x / 8;
        int bit_pos  = 7 - (x % 8);  // MSB first: col 0 → bit 7

        if ((top >> 2) & 1) r1_plane[byte_idx] |= (1 << bit_pos);
        if ((top >> 1) & 1) g1_plane[byte_idx] |= (1 << bit_pos);
        if ((top >> 0) & 1) b1_plane[byte_idx] |= (1 << bit_pos);
        if ((bot >> 2) & 1) r2_plane[byte_idx] |= (1 << bit_pos);
        if ((bot >> 1) & 1) g2_plane[byte_idx] |= (1 << bit_pos);
        if ((bot >> 0) & 1) b2_plane[byte_idx] |= (1 << bit_pos);
    }
}

// ─────────────────────────────────────────────────────────────
//  HUB75 INIT
//  Configures GPIO outputs and SPI2 peripheral
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  FM6124 / FM6126A INITIALIZATION SEQUENCE
//
//  The FM6124 is NOT a simple shift register like ICN2038S.
//  It uses a LAT-pulse-count command protocol:
//
//    CLK pulses while LAT is HIGH → selects the command:
//      0–1  pulses = RESET_OEN   (soft reset)
//        3  pulses = DATA_LATCH  (normal pixel latch — used in refresh)
//     4–10  pulses = Reserved
//       11  pulses = WR_REG1    (write brightness register)
//       12  pulses = WR_REG2    (write enable/output register)
//
//  Without WR_REG1 + WR_REG2, the chip's internal registers
//  contain random power-on values → garbage / dots on display.
//
//  Register values used (same as PxMatrix library and ESP32-HUB75-DMA):
//    REG1 = 0x7FFF  → maximum brightness, all channels enabled
//    REG2 = 0x0040  → enable output drivers (bit 6 = 1)
//
//  CRITICAL: This must run BEFORE SPI is initialized.
//  MOSI and CLK are temporarily configured as plain GPIO for
//  bit-banging, then SPI takes them over after this function returns.
//
//  We shift PANEL_W * 3 bits total (3 chips per row × PANEL_W/16 bits)
//  The standard approach: shift 64 bits per register write,
//  with LAT raised for the last N clocks of the 64-clock sequence.
// ─────────────────────────────────────────────────────────────

static void fm6124_init(void)
{
    ESP_LOGI(TAG, "FM6124: Running init sequence (REG1=0x7FFF, REG2=0x0040)");

    // ── Step 1: Configure MOSI and CLK as plain GPIO outputs ──
    // SPI has NOT been initialized yet — these pins are free to use
    gpio_config_t data_pins = {
        .pin_bit_mask = (1ULL << HUB75_PIN_MOSI) | (1ULL << HUB75_PIN_CLK),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&data_pins));

    // Start with CLK and MOSI LOW, LAT LOW
    REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << HUB75_PIN_CLK));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << HUB75_PIN_MOSI));
    GPIO_FAST_CLR(HUB75_PIN_LAT);
    GPIO_FAST_SET(HUB75_PIN_OE);   // blank display during init
    esp_rom_delay_us(10);

    // ── Helper macro: one bit-bang CLK pulse ──────────────────
    // FM6124 latches data on the RISING edge of CLK
    #define FM_CLK_PULSE() do {                                         \
        REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << HUB75_PIN_CLK));          \
        esp_rom_delay_us(1);                                            \
        REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << HUB75_PIN_CLK));          \
        esp_rom_delay_us(1);                                            \
    } while(0)

    // ── Helper macro: set MOSI HIGH or LOW ────────────────────
    #define FM_MOSI_HIGH() REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << HUB75_PIN_MOSI))
    #define FM_MOSI_LOW()  REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << HUB75_PIN_MOSI))

    // ── WR_REG1: Write brightness register (11 LAT clocks) ────
    //
    // Sequence: shift 64 bits total.
    // LAT goes HIGH at clock (64 - 11) = clock 53 (0-indexed).
    // LAT stays HIGH for the remaining 11 clocks.
    // Data for REG1: all bits HIGH (0xFFFF pattern) = max brightness.
    //
    // The FM6124 has 16-bit registers. We are driving a panel with
    // multiple chips. Setting MOSI HIGH for all 64 clocks puts 0xFF
    // into every chip's shift register — safe and correct for REG1.

    GPIO_FAST_CLR(HUB75_PIN_LAT);
    FM_MOSI_HIGH();   // REG1 data = all 1s = max brightness

    for (int i = 0; i < 64; i++) {
        // Raise LAT 11 clocks before the end
        if (i == (64 - 11)) {
            GPIO_FAST_SET(HUB75_PIN_LAT);
        }
        FM_CLK_PULSE();
    }
    GPIO_FAST_CLR(HUB75_PIN_LAT);
    esp_rom_delay_us(10);

    // ── WR_REG2: Write enable register (12 LAT clocks) ────────
    //
    // Sequence: shift 64 bits total.
    // LAT goes HIGH at clock (64 - 12) = clock 52 (0-indexed).
    // LAT stays HIGH for the remaining 12 clocks.
    //
    // REG2 data = 0x0040: bit 6 HIGH = enable output drivers.
    // Pattern: only bit 6 of the 16-bit register should be 1.
    //
    // We shift MSB first. For a 16-bit value 0x0040 = 0000 0000 0100 0000:
    // bit positions 15..0: only bit 6 is HIGH.
    // We repeat this 4 times for 64 total clocks (4 chips × 16 bits).
    //
    // Bit 6 within each 16-bit group → clock positions 9, 25, 41, 57
    // (counting from 0, MSB first: clock 0 = bit15, clock 9 = bit6)

    GPIO_FAST_CLR(HUB75_PIN_LAT);

    for (int i = 0; i < 64; i++) {
        // Position within the current 16-bit word (0=MSB, 15=LSB)
        int bit_pos = i % 16;

        // Bit 6 of 0x0040: in MSB-first order, bit6 is at position 9
        // (bit15=pos0, bit14=pos1, ..., bit6=pos9)
        if (bit_pos == 9) {
            FM_MOSI_HIGH();   // this is bit 6 — set HIGH
        } else {
            FM_MOSI_LOW();
        }

        // Raise LAT 12 clocks before the end
        if (i == (64 - 12)) {
            GPIO_FAST_SET(HUB75_PIN_LAT);
        }
        FM_CLK_PULSE();
    }
    GPIO_FAST_CLR(HUB75_PIN_LAT);
    esp_rom_delay_us(10);

    // ── Cleanup: return pins to safe state ────────────────────
    FM_MOSI_LOW();
    REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << HUB75_PIN_CLK));

    // Un-define local macros — they should not leak outside this function
    #undef FM_CLK_PULSE
    #undef FM_MOSI_HIGH
    #undef FM_MOSI_LOW

    ESP_LOGI(TAG, "FM6124: Init complete");
}

void hub75_init(void)
{
    ESP_LOGI(TAG, "Initializing HUB75 driver (FM6124)...");
    ESP_LOGI(TAG, "Panel: %dx%d, %d panel, 1:%d scan",
             PANEL_W, PANEL_H, PANELS_COUNT, SCAN_ROWS);

    // ── Step 1: Configure control GPIO outputs ────────────────
    // LAT, OE, A, B, C, D — plain GPIO outputs
    // MOSI and CLK are NOT configured here — fm6124_init() does
    // them first as GPIO, then SPI takes them over below
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << HUB75_PIN_LAT) |
                        (1ULL << HUB75_PIN_OE)  |
                        (1ULL << HUB75_PIN_A)   |
                        (1ULL << HUB75_PIN_B)   |
                        (1ULL << HUB75_PIN_C)   |
                        (1ULL << HUB75_PIN_D),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    // Safe initial state
    GPIO_FAST_SET(HUB75_PIN_OE);    // OE HIGH  = display blanked
    GPIO_FAST_CLR(HUB75_PIN_LAT);   // LAT LOW  = no latch
    GPIO_FAST_CLR(HUB75_PIN_A);     // Row address = row 0
    GPIO_FAST_CLR(HUB75_PIN_B);
    GPIO_FAST_CLR(HUB75_PIN_C);
    GPIO_FAST_CLR(HUB75_PIN_D);

    // ── Step 2: FM6124 register initialization ────────────────
    // Must happen BEFORE SPI is initialized.
    // fm6124_init() temporarily owns MOSI and CLK as plain GPIO,
    // writes REG1 (brightness) and REG2 (enable), then releases them.
    fm6124_init();

    // ── Step 3: Initialize SPI2 ───────────────────────────────
    // SPI now takes ownership of MOSI (GPIO11) and CLK (GPIO12)
    ESP_LOGI(TAG, "Initializing SPI2: MOSI=GPIO%d CLK=GPIO%d @ %dMHz",
             HUB75_PIN_MOSI, HUB75_PIN_CLK, HUB75_SPI_SPEED_HZ / 1000000);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = HUB75_PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = HUB75_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = SPI_BUF_BYTES,
        .flags           = 0,
    };

    spi_device_interface_config_t dev_cfg = {
        .mode           = 0,                   // CPOL=0 CPHA=0 — CLK idle LOW
        .clock_speed_hz = HUB75_SPI_SPEED_HZ,
        .spics_io_num   = -1,                  // no CS — we use LAT
        .queue_size     = 1,
        .flags          = SPI_DEVICE_NO_DUMMY,
        .pre_cb         = NULL,
        .post_cb        = NULL,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev));

    ESP_LOGI(TAG, "HUB75 + FM6124 initialized OK");
}

// ─────────────────────────────────────────────────────────────
//  HUB75 REFRESH TASK
//  Runs forever on Core 1 at maximum priority
//  Continuously cycles through all 16 scan rows
//
//  At 20MHz SPI, one row takes:
//    SPI: 128 bytes × 8 bits / 20MHz = 51.2µs
//    Control GPIO:                     ~2µs
//    Row on time:                       1µs
//    Total per row:                   ~54µs
//    Total for 16 rows:              ~864µs
//    Refresh rate:                   ~1157Hz  (well above 100Hz minimum)
// ─────────────────────────────────────────────────────────────

void hub75_refresh_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Refresh task started on core %d", xPortGetCoreID());

    uint8_t current_row = 0;

    // Pre-configure SPI transaction struct — reused every row
    // Filling it once here avoids repeated struct initialization overhead
    spi_transaction_t tx = {
        .length    = SPI_BUF_BYTES * 8,  // 48 bytes × 8 = 384 bits
        .tx_buffer = spi_tx_buf,    // DMA reads from this buffer
        .rx_buffer = NULL,          // No receive
        .flags     = 0,
    };

    while (1) {
        // ── Get display buffer pointer ─────────────────────────
        // This is read-only — Core 0 never writes display_buf
        // No lock needed for the read itself
        const uint8_t *fb = framebuffer_get_display();

        // ── Step 1: Blank output ───────────────────────────────
        // Prevent ghosting — LEDs off while shift registers are being loaded
        GPIO_FAST_SET(HUB75_PIN_OE);

        // ── Step 2: Build SPI buffer ───────────────────────────
        // Pack framebuffer pixels into SPI byte format for this row
        build_spi_buffer(fb, current_row);

        // ── Step 3: Transmit via SPI DMA ──────────────────────
        // DMA clocks out 128 bytes → 1024 CLK pulses → fills all shift registers
        // This call blocks until DMA transfer is complete
        spi_device_transmit(spi_dev, &tx);

        // ── Step 4: Set row address ────────────────────────────
        // Set ABCD pins to select which row pair will light up
        // Includes HUB75_MUX_DELAY_US settling time (1µs confirmed needed)
        set_row_address(current_row);

        // ── Step 5: FM6124 DATA_LATCH — exactly 3 CLK pulses while LAT HIGH
        //
        // FM6124 command protocol: the number of CLK rising edges
        // while LAT is HIGH determines the command:
        //   3 pulses = DATA_LATCH (transfer shift reg → output reg)
        //  11 pulses = WR_REG1   (write brightness register)
        //  12 pulses = WR_REG2   (write enable register)
        //
        // We must give EXACTLY 3 pulses here — no more, no less.
        // SPI clock is paused (SPI peripheral holds CLK LOW after
        // transfer completes), so we bit-bang 3 pulses manually.
        //
        // CLK is currently LOW (SPI left it LOW after transmission).
        GPIO_FAST_SET(HUB75_PIN_LAT);                                   // LAT HIGH
        REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << HUB75_PIN_CLK)); esp_rom_delay_us(1); // pulse 1
        REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << HUB75_PIN_CLK)); esp_rom_delay_us(1);
        REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << HUB75_PIN_CLK)); esp_rom_delay_us(1); // pulse 2
        REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << HUB75_PIN_CLK)); esp_rom_delay_us(1);
        REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << HUB75_PIN_CLK)); esp_rom_delay_us(1); // pulse 3
        REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << HUB75_PIN_CLK)); esp_rom_delay_us(1);
        GPIO_FAST_CLR(HUB75_PIN_LAT);                                   // LAT LOW

        // ── Step 6: Enable output ──────────────────────────────
        // OE LOW → LEDs turn on for this row pair
        GPIO_FAST_CLR(HUB75_PIN_OE);

        // ── Step 7: Row on time ────────────────────────────────
        // Keep this row lit for a short time before moving to next row
        // Too long  = lower refresh rate (flickering)
        // Too short = dim display
        esp_rom_delay_us(HUB75_ROW_ON_US);

        // ── Step 8: Advance to next row ────────────────────────
        current_row = (current_row + 1) % SCAN_ROWS;

        // No vTaskDelay here — this task runs as fast as possible
        // At 240MHz CPU, FreeRTOS task switching overhead is negligible
        // The SPI transmit time (~51µs) is the natural pacing mechanism
    }
}