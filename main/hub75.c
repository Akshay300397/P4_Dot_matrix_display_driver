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
#include "hal/gpio_ll.h"

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

static DRAM_ATTR uint8_t spi_tx_buf[PANEL_W] __attribute__((aligned(4)));

// ─────────────────────────────────────────────────────────────
//  FAST GPIO MACROS
//  Direct register write — single instruction, ~4ns per toggle
//  All assigned pins are GPIO 0–31, so out_w1ts/out_w1tc are valid
//
//  NOTE: If you move any pin above GPIO31, switch to:
//        GPIO.out1_w1ts.val and GPIO.out1_w1tc.val
// ─────────────────────────────────────────────────────────────

#define GPIO_FAST_SET(pin)   (GPIO.out_w1ts.val = (1UL << (pin)))
#define GPIO_FAST_CLR(pin)   (GPIO.out_w1tc.val = (1UL << (pin)))

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
    GPIO.out_w1tc.val = clear_mask;
    GPIO.out_w1ts.val = set_mask;

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
//  Shift register chain after PI→PO jumpers:
//    MOSI → [R1 reg] → [R2 reg] → [G1 reg] → [G2 reg] → [B1 reg] → [B2 reg]
//
//  Since SPI sends MSB first, the first bit clocked travels to the end of
//  the chain (B2), and the last bit stays at the start (R1). So:
//    Byte bit5 = B2 (sent first  → end of chain)
//    Byte bit4 = B1
//    Byte bit3 = G2
//    Byte bit2 = G1
//    Byte bit1 = R2
//    Byte bit0 = R1 (sent last   → start of chain)
// ─────────────────────────────────────────────────────────────

static inline void build_spi_buffer(const uint8_t *fb, uint8_t row)
{
    // Pointers to top and bottom half rows in framebuffer
    const uint8_t *top_row = fb + (row             * PANEL_W);
    const uint8_t *bot_row = fb + ((row + SCAN_ROWS) * PANEL_W);

    for (int x = 0; x < PANEL_W; x++) {
        uint8_t top = top_row[x];
        uint8_t bot = bot_row[x];

        // Extract individual bits from 3-bit color (0b00000RGB)
        uint8_t r1 = (top >> 2) & 1;   // Red   — top half
        uint8_t g1 = (top >> 1) & 1;   // Green — top half
        uint8_t b1 = (top >> 0) & 1;   // Blue  — top half
        uint8_t r2 = (bot >> 2) & 1;   // Red   — bottom half
        uint8_t g2 = (bot >> 1) & 1;   // Green — bottom half
        uint8_t b2 = (bot >> 0) & 1;   // Blue  — bottom half

        // Pack into one byte — order must match shift register chain
        spi_tx_buf[x] = (b2 << 5) |
                        (b1 << 4) |
                        (g2 << 3) |
                        (g1 << 2) |
                        (r2 << 1) |
                        (r1 << 0);

        // NOTE: If colors appear wrong on your panel, the shift register
        // chain order may differ. Try swapping these assignments.
        // See hub75.h comments for alternative bit orders.
    }
}

// ─────────────────────────────────────────────────────────────
//  HUB75 INIT
//  Configures GPIO outputs and SPI2 peripheral
// ─────────────────────────────────────────────────────────────

void hub75_init(void)
{
    ESP_LOGI(TAG, "Initializing HUB75 driver...");
    ESP_LOGI(TAG, "Panel: %dx%d, %d panels chained, 1:%d scan",
             PANEL_W, PANEL_H, PANELS_COUNT, SCAN_ROWS);
    ESP_LOGI(TAG, "SPI: MOSI=GPIO%d CLK=GPIO%d @ %dMHz",
             HUB75_PIN_MOSI, HUB75_PIN_CLK, HUB75_SPI_SPEED_HZ / 1000000);

    // ── Configure control GPIO outputs ────────────────────────
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

    // Start in safe state: output blanked, latch inactive
    GPIO_FAST_SET(HUB75_PIN_OE);    // OE HIGH = output disabled
    GPIO_FAST_CLR(HUB75_PIN_LAT);   // LAT LOW = no latch
    GPIO_FAST_CLR(HUB75_PIN_A);     // Row address = 0
    GPIO_FAST_CLR(HUB75_PIN_B);
    GPIO_FAST_CLR(HUB75_PIN_C);
    GPIO_FAST_CLR(HUB75_PIN_D);

    // ── Configure SPI2 (HSPI) ─────────────────────────────────
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = HUB75_PIN_MOSI,
        .miso_io_num     = -1,          // Not used — output only
        .sclk_io_num     = HUB75_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = PANEL_W,     // 128 bytes per transaction
        .flags           = 0,
    };

    spi_device_interface_config_t dev_cfg = {
        .mode             = 0,                      // SPI MODE0: CPOL=0, CPHA=0
                                                    // ICN6124 samples on rising CLK edge
                                                    // CLK idle LOW — matches MODE0 exactly
        .clock_speed_hz   = HUB75_SPI_SPEED_HZ,    // 20MHz default
        .spics_io_num     = -1,                     // No chip select — we use LAT instead
        .queue_size       = 1,                      // Single transaction queue
        .flags            = SPI_DEVICE_NO_DUMMY,    // No dummy bits between bytes
        .pre_cb           = NULL,
        .post_cb          = NULL,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev));

    ESP_LOGI(TAG, "HUB75 initialized OK");
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
        .length    = PANEL_W * 8,   // Total bits = 128 bytes × 8 bits = 1024 bits
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

        // ── Step 5: Latch pulse ────────────────────────────────
        // Rising edge of LAT transfers shift register contents
        // to ICN6124 output registers — pixels are now ready to display
        GPIO_FAST_SET(HUB75_PIN_LAT);
        GPIO_FAST_CLR(HUB75_PIN_LAT);

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