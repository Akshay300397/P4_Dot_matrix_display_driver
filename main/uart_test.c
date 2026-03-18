/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "esp_intr_alloc.h"
#include "sdkconfig.h"
#include "esp_log.h"

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
#define FRAME_PAYLOAD_LEN   12      // PAYLOAD bytes (4 chars × 3 fields)
#define FRAME_CRC_LEN       2       // CRC bytes (lo, hi)
#define FRAME_TOTAL         19      // Total frame bytes

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


typedef struct {
    uint8_t frame[FRAME_TOTAL];
    size_t  len;
} uart_msg_t;

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

void uart_parse_frame(void *arg)
{
    //uint8_t frame[FRAME_TOTAL];
    static uart_msg_t msg;
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, msg.frame, FRAME_TOTAL, portMAX_DELAY);
        // Write data back to the UART
        //uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
        if (len) {
            data[len] = '\0';
            ESP_LOGI(TAG1, "Recv %d str: %s", len, msg.frame);
        }
    }
}

