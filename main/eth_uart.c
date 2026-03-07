/*
 * eth_uart.c
 * Ethernet-to-Serial UART receiver and packet parser
 * Runs on Core 0 — receives display commands, updates framebuffer
 *
 * Packet state machine:
 *   WAIT_START  → look for 0xAA start byte
 *   WAIT_CMD    → read command byte, start CRC
 *   WAIT_LEN    → read payload length
 *   WAIT_PAYLOAD → accumulate payload bytes, update CRC
 *   WAIT_CRC    → verify CRC, process or discard packet
 */

#include "eth_uart.h"
#include "framebuffer.h"
#include "hub75.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "ETH_UART";

// ─────────────────────────────────────────────────────────────
//  PACKET RECEIVE STATE MACHINE
// ─────────────────────────────────────────────────────────────

typedef enum {
    WAIT_START,
    WAIT_CMD,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CRC
} rx_state_t;

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[PKT_MAX_PAYLOAD];
} packet_t;

// ─────────────────────────────────────────────────────────────
//  FULL FRAME REASSEMBLY BUFFER
//  CMD_FULLFRAME sends 64 chunks × 64 bytes = 4096 bytes total
//  Reassemble here before swapping to display
// ─────────────────────────────────────────────────────────────

static uint8_t fullframe_buf[PANEL_W * PANEL_H];
static bool    fullframe_receiving = false;

// ─────────────────────────────────────────────────────────────
//  rgb_to_3bit()
//  Converts 8-bit R,G,B values to 3-bit color
//  Threshold at 128 — above = ON, below = OFF
//  This is the same logic as the ATmega PxMatrix test code
// ─────────────────────────────────────────────────────────────

static inline uint8_t rgb_to_3bit(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r > 128) ? COLOR_RED   : 0) |
           ((g > 128) ? COLOR_GREEN : 0) |
           ((b > 128) ? COLOR_BLUE  : 0);
}

// ─────────────────────────────────────────────────────────────
//  PACKET PROCESSOR
//  Called when a complete valid packet is received
// ─────────────────────────────────────────────────────────────

static void process_packet(const packet_t *pkt)
{
    switch (pkt->cmd) {

        // ── CMD_CLEAR (0x01) ──────────────────────────────────
        // Payload: none
        // Action:  Clear entire display to black
        case CMD_CLEAR: {
            framebuffer_clear_back();
            framebuffer_swap();
            ESP_LOGI(TAG, "CMD_CLEAR");
            break;
        }

        // ── CMD_PIXEL (0x02) ──────────────────────────────────
        // Payload: x(1) y(1) r(1) g(1) b(1) = 5 bytes
        // Action:  Set single pixel
        case CMD_PIXEL: {
            if (pkt->len < 5) {
                ESP_LOGW(TAG, "CMD_PIXEL: short payload %d", pkt->len);
                break;
            }
            uint8_t x = pkt->payload[0];
            uint8_t y = pkt->payload[1];
            uint8_t r = pkt->payload[2];
            uint8_t g = pkt->payload[3];
            uint8_t b = pkt->payload[4];
            framebuffer_set_pixel(x, y, rgb_to_3bit(r, g, b));
            framebuffer_swap();
            break;
        }

        // ── CMD_TEXT (0x03) ───────────────────────────────────
        // Payload: x(1) y(1) r(1) g(1) b(1) string(null-terminated)
        // Action:  Draw text string at position with color
        case CMD_TEXT: {
            if (pkt->len < 6) {
                ESP_LOGW(TAG, "CMD_TEXT: short payload %d", pkt->len);
                break;
            }
            uint8_t     x     = pkt->payload[0];
            uint8_t     y     = pkt->payload[1];
            uint8_t     r     = pkt->payload[2];
            uint8_t     g     = pkt->payload[3];
            uint8_t     b     = pkt->payload[4];
            const char *str   = (const char *)&pkt->payload[5];
            uint8_t     color = rgb_to_3bit(r, g, b);

            framebuffer_draw_string(x, y, str, color);
            framebuffer_swap();
            ESP_LOGI(TAG, "CMD_TEXT (%d,%d) \"%s\"", x, y, str);
            break;
        }

        // ── CMD_FILL_RECT (0x04) ──────────────────────────────
        // Payload: x(1) y(1) w(1) h(1) r(1) g(1) b(1) = 7 bytes
        // Action:  Fill rectangle with solid color
        case CMD_FILL_RECT: {
            if (pkt->len < 7) {
                ESP_LOGW(TAG, "CMD_FILL_RECT: short payload %d", pkt->len);
                break;
            }
            uint8_t x = pkt->payload[0];
            uint8_t y = pkt->payload[1];
            uint8_t w = pkt->payload[2];
            uint8_t h = pkt->payload[3];
            uint8_t r = pkt->payload[4];
            uint8_t g = pkt->payload[5];
            uint8_t b = pkt->payload[6];
            framebuffer_fill_rect(x, y, w, h, rgb_to_3bit(r, g, b));
            framebuffer_swap();
            break;
        }

        // ── CMD_FULLFRAME (0x06) ──────────────────────────────
        // Payload: chunk_index(1) + 64 bytes pixel data = 65 bytes
        // Chunks:  0–63 (64 chunks × 64 bytes = 4096 bytes total)
        // Action:  Reassemble full frame, display when last chunk arrives
        //
        // Pixel format in payload: each byte = 3-bit color (0bRGB)
        // Same format as framebuffer — direct memcpy, no conversion needed
        case CMD_FULLFRAME: {
            if (pkt->len < 65) {
                ESP_LOGW(TAG, "CMD_FULLFRAME: short payload %d", pkt->len);
                break;
            }
            uint8_t chunk_idx = pkt->payload[0];
            if (chunk_idx >= 64) {
                ESP_LOGW(TAG, "CMD_FULLFRAME: invalid chunk %d", chunk_idx);
                break;
            }

            // Copy 64 bytes into reassembly buffer at correct offset
            memcpy(fullframe_buf + (chunk_idx * 64), &pkt->payload[1], 64);
            fullframe_receiving = true;

            // When last chunk arrives, copy to back buffer and swap
            if (chunk_idx == 63) {
                memcpy(framebuffer_get_back(), fullframe_buf, PANEL_W * PANEL_H);
                framebuffer_swap();
                fullframe_receiving = false;
                ESP_LOGI(TAG, "CMD_FULLFRAME complete — frame displayed");
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown CMD: 0x%02X", pkt->cmd);
            break;
    }
}

// ─────────────────────────────────────────────────────────────
//  ETH UART TASK
//  Runs forever on Core 0
//  Reads UART bytes → state machine → processes valid packets
// ─────────────────────────────────────────────────────────────

void eth_uart_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UART task started on core %d", xPortGetCoreID());

    // ── Configure UART1 ───────────────────────────────────────
    uart_config_t uart_cfg = {
        .baud_rate  = ETH_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        ETH_UART_PORT,
        ETH_UART_BUF_SIZE * 2,  // RX buffer
        0,                       // TX buffer (0 = use driver directly)
        0,                       // event queue size
        NULL,                    // event queue handle
        0                        // interrupt flags
    ));
    ESP_ERROR_CHECK(uart_param_config(ETH_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        ETH_UART_PORT,
        ETH_UART_PIN_TX,
        ETH_UART_PIN_RX,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_LOGI(TAG, "UART%d ready: RX=GPIO%d TX=GPIO%d @ %d baud",
             ETH_UART_PORT, ETH_UART_PIN_RX, ETH_UART_PIN_TX, ETH_UART_BAUD);

    // ── Packet receive state machine ──────────────────────────
    rx_state_t rx_state = WAIT_START;
    packet_t   pkt;
    uint8_t    rx_byte;
    uint8_t    rx_idx  = 0;
    uint8_t    rx_crc  = 0;

    // Statistics (optional, useful for debugging)
    uint32_t packets_received = 0;
    uint32_t packets_dropped  = 0;

    while (1) {
        // Block until a byte arrives (10ms timeout)
        int len = uart_read_bytes(
            ETH_UART_PORT,
            &rx_byte,
            1,
            pdMS_TO_TICKS(10)
        );

        if (len <= 0) {
            // Timeout — no data. Yield to other tasks briefly.
            taskYIELD();
            continue;
        }

        // ── State machine ──────────────────────────────────────
        switch (rx_state) {

            case WAIT_START:
                // Wait for magic start byte 0xAA
                if (rx_byte == PKT_START_BYTE) {
                    rx_state = WAIT_CMD;
                }
                break;

            case WAIT_CMD:
                pkt.cmd  = rx_byte;
                rx_crc   = rx_byte;     // CRC starts from CMD byte
                rx_state = WAIT_LEN;
                break;

            case WAIT_LEN:
                pkt.len = rx_byte;
                rx_crc ^= rx_byte;
                rx_idx  = 0;
                // If no payload, go straight to CRC
                rx_state = (pkt.len > 0) ? WAIT_PAYLOAD : WAIT_CRC;
                break;

            case WAIT_PAYLOAD:
                if (rx_idx < PKT_MAX_PAYLOAD) {
                    pkt.payload[rx_idx] = rx_byte;
                    rx_crc ^= rx_byte;
                    rx_idx++;
                    if (rx_idx >= pkt.len) {
                        rx_state = WAIT_CRC;
                    }
                } else {
                    // Payload overflow — discard packet
                    ESP_LOGW(TAG, "Payload overflow — packet discarded");
                    packets_dropped++;
                    rx_state = WAIT_START;
                }
                break;

            case WAIT_CRC:
                if (rx_byte == rx_crc) {
                    // Valid packet — process it
                    process_packet(&pkt);
                    packets_received++;
                } else {
                    // CRC mismatch — discard
                    ESP_LOGW(TAG,
                        "CRC mismatch: expected 0x%02X got 0x%02X — dropped",
                        rx_crc, rx_byte);
                    packets_dropped++;
                }
                rx_state = WAIT_START;

                // Log stats every 100 packets
                if ((packets_received + packets_dropped) % 100 == 0) {
                    ESP_LOGI(TAG, "Stats: rx=%lu dropped=%lu",
                             packets_received, packets_dropped);
                }
                break;
        }
    }
}