/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "esp_intr_alloc.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "display_uart.h"       // DISP_FIELD_CHARS, DISP_X_OFFSET, display_update_from_uart()
#include "display_content.h"    // run_hardware_check()
/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD GPIO_NUM_18
#define ECHO_TEST_RXD GPIO_NUM_17
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)



#define ECHO_UART_PORT_NUM      UART_NUM_1
#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (4096)

static const char *TAG1 = "UART TEST";

#define BUF_SIZE (1024)

// ─────────────────────────────────────────────────────────────
//  PROTOCOL CONSTANTS
// ─────────────────────────────────────────────────────────────

#define FRAME_START         '*'
#define FRAME_END           '#'
#define FRAME_CD_LEN        2       // C/D field bytes
#define FRAME_LEN_LEN       1       // LEN field bytes
#define FRAME_PAYLOAD_LEN   4      // PAYLOAD bytes (4 chars × 3 fields)
#define FRAME_CRC_LEN       2       // CRC bytes (lo, hi)
#define FRAME_TOTAL         11      // Total frame bytes

// C/D mode identifiers
#define MODE_COMMAND        "C0"
#define MODE_DATA_F1        "D1"
#define MODE_DATA_F2        "D2"
#define MODE_DATA_F3        "D3"
#define MODE_DATA_F4        "D4"
#define MODE_DATA_F5        "D5"
#define MODE_DATA_F6        "D6"

// Command IDs (payload[0] when C/D = "C0")
#define CMD_CLEAR           0x01
#define CMD_BRIGHTNESS      0x02
#define CMD_SCROLL          0x03
#define CMD_QUERY_IP        0x04
#define CMD_DISPLAY_TEST    0x05


// ─────────────────────────────────────────────────────────────
//  CRC: simple sum of C/D + LEN + PAYLOAD bytes
//  Returns 16-bit sum (lo byte = sum & 0xFF, hi = sum >> 8)
// ─────────────────────────────────────────────────────────────
 
static uint16_t calc_crc(const uint8_t *frame)
{
    // frame layout (0-indexed, after stripping '*'):
    //   [0][1] = C/D
    //   [2]    = LEN
    //   [3..7]= PAYLOAD (4 bytes)
    // Sum bytes 0–14 (C/D + LEN + PAYLOAD)
    uint16_t sum = 0;
    for (int i = 1; i <= 7; i++) {   // indices 1-7 in full frame (skip '*' at 0)
        sum += frame[i];
    }
    return sum;
}
 
// ─────────────────────────────────────────────────────────────
//  FRAME PARSER
//  Validates and dispatches a complete 19-byte frame
// ─────────────────────────────────────────────────────────────
 
static void parse_frame(const uint8_t *frame)
{
    // ── Validate delimiters ───────────────────────────────────
    if (frame[0] != FRAME_START || frame[10] != FRAME_END) {
        ESP_LOGW(TAG1, "Bad delimiters: 0x%02X ... 0x%02X", frame[0], frame[10]);
        return;
    }
 
    // ── Extract fields ────────────────────────────────────────
    char   cd[3]      = { frame[1], frame[2], '\0' };   // C/D as string
    uint8_t len       = frame[3];                        // payload length
    const uint8_t *payload = &frame[4];                  // 4 payload bytes
    uint16_t rx_crc   = (uint16_t)frame[8] |
                        ((uint16_t)frame[9] << 8);      // lo, hi
    
    // ── Validate LEN ─────────────────────────────────────────
    if (len > FRAME_PAYLOAD_LEN) {
        ESP_LOGW(TAG1, "LEN %d exceeds max %d", len, FRAME_PAYLOAD_LEN);
        return;
    }
 
    // ── Validate CRC ─────────────────────────────────────────
    uint16_t calc = calc_crc(frame);
    if (calc != rx_crc) {
        ESP_LOGW(TAG1, "CRC fail: calc=0x%04X rx=0x%04X", calc, rx_crc);
        return;
    }
 
    //ESP_LOGI(TAG1, "Frame OK  — C/D=[%s] LEN=%d CRC=0x%04X", cd, len, rx_crc);
    ESP_LOGI(TAG1, "Received frame: C/D=[%s] LEN=%d PAYLOAD=[%.*s] CRC=0x%04X", cd, len, len, payload, rx_crc);

    // Extract payload as null-terminated string (max 4 chars)
    char text[DISP_FIELD_CHARS + 1];
    int  copy_len = (len < DISP_FIELD_CHARS) ? len : DISP_FIELD_CHARS;
    memcpy(text, payload, copy_len);
    text[copy_len] = '\0';
 
    // Map C/D string to field number 1–6
    int field = 0;
    if      (strcmp(cd, MODE_DATA_F1) == 0) field = 1;
    else if (strcmp(cd, MODE_DATA_F2) == 0) field = 2;
    else if (strcmp(cd, MODE_DATA_F3) == 0) field = 3;
    else if (strcmp(cd, MODE_DATA_F4) == 0) field = 4;
    else if (strcmp(cd, MODE_DATA_F5) == 0) field = 5;
    else if (strcmp(cd, MODE_DATA_F6) == 0) field = 6;
    else {
        ESP_LOGW(TAG1, "Unknown C/D: %s", cd);
        return;
    }
    //parse text and update display
    
    // Calculate display position for log
    int col = (field - 1) % 3;
    int row = (field - 1) / 3;
    int x   = DISP_X_OFFSET + col * DISP_FIELD_SPACING;
    int y   = (row == 0)  ? DISP_ROW1_Y : DISP_ROW2_Y;
 
    ESP_LOGI(TAG1, "Field %d → Row%d Col%d  x=%d y=%d  text=\"%s\"",
             field, row + 1, col + 1, x, y, text);
 
    // Update display — mirrors draw_default_content() pattern
    display_update_from_uart(field, text);
}
void eth_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = ECHO_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM,
                                 ECHO_TEST_TXD, ECHO_TEST_RXD,
                                 ECHO_TEST_RTS, ECHO_TEST_CTS));

    // NULL for event queue — we don't need it
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM,
                                        BUF_SIZE * 2, 0, 0, NULL, 0));

    
}

void uart_parse_frame(void *pvParameters)
{
    ESP_LOGI(TAG1, "UART task started on core %d", xPortGetCoreID());
 
    uint8_t frame[FRAME_TOTAL];
    uint8_t byte;
 
    while (1) {
        // ── Step 1: Wait for start delimiter '*' ──────────────
        int n = uart_read_bytes(ECHO_UART_PORT_NUM, &byte, 1, portMAX_DELAY);
        if (n <= 0) continue;
        if (byte != FRAME_START) continue;
 
        frame[0] = byte;
 
        // ── Step 2: Read remaining 18 bytes ───────────────────
        int remaining = uart_read_bytes(ECHO_UART_PORT_NUM,
                                        &frame[1],
                                        FRAME_TOTAL - 1,
                                        pdMS_TO_TICKS(100));
 
        if (remaining != FRAME_TOTAL - 1) {
            ESP_LOGW(TAG1, "Incomplete frame: got %d of %d bytes — discarding",
                     remaining + 1, FRAME_TOTAL);
            uart_flush_input(ECHO_UART_PORT_NUM);
            continue;
        }
 
        // ── Step 3: Log raw frame ──────────────────────────────
        ESP_LOGI(TAG1, "Frame received (%d bytes): "
                 "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X "
                 "%02X ",
                 FRAME_TOTAL,
                 frame[0],  frame[1],  frame[2],  frame[3],  frame[4],
                 frame[5],  frame[6],  frame[7],  frame[8],  frame[9],
                 frame[10]);
 
        // ── Step 4: Parse and dispatch ─────────────────────────
        parse_frame(frame);
    }
}
 