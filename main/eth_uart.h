#pragma once

/*
 * eth_uart.h
 * Ethernet-to-Serial UART receiver and packet parser
 * Runs on Core 0, receives display commands from ethernet board
 *
 * Packet Protocol:
 * ┌────────┬────────┬────────┬──────────────────┬────────┐
 * │  0xAA  │  CMD   │  LEN   │    PAYLOAD       │  CRC   │
 * │ 1 byte │ 1 byte │ 1 byte │   LEN bytes      │ 1 byte │
 * └────────┴────────┴────────┴──────────────────┴────────┘
 *
 * CRC = XOR of all bytes from CMD through end of PAYLOAD
 *
 * Commands:
 *   0x01  CMD_CLEAR       No payload — clear entire display
 *   0x02  CMD_PIXEL       5 bytes: x(1) y(1) r(1) g(1) b(1)
 *   0x03  CMD_TEXT        variable: x(1) y(1) color(1) string+\0
 *   0x04  CMD_FILL_RECT   7 bytes: x(1) y(1) w(1) h(1) r(1) g(1) b(1)
 *   0x05  CMD_BRIGHTNESS  1 byte:  not used (display always full brightness)
 *   0x06  CMD_FULLFRAME   65 bytes: chunk_index(1) + 64 bytes pixel data
 *                         Send 64 chunks (0–63) to transfer full 4096-byte frame
 */

#include <stdint.h>

// ─────────────────────────────────────────────────────────────
//  UART CONFIGURATION
// ─────────────────────────────────────────────────────────────

#define ETH_UART_PORT       UART_NUM_1      // UART1 — leave UART0 for USB debug
#define ETH_UART_PIN_RX     16              // Connect to ethernet board TX
#define ETH_UART_PIN_TX     17              // Connect to ethernet board RX
#define ETH_UART_BAUD       115200          // 115200 baud — change to 460800 for faster updates
#define ETH_UART_BUF_SIZE   512             // UART ring buffer size

// ─────────────────────────────────────────────────────────────
//  PACKET PROTOCOL CONSTANTS
// ─────────────────────────────────────────────────────────────

#define PKT_START_BYTE      0xAA

#define CMD_CLEAR           0x01
#define CMD_PIXEL           0x02
#define CMD_TEXT            0x03
#define CMD_FILL_RECT       0x04
#define CMD_BRIGHTNESS      0x05
#define CMD_FULLFRAME       0x06

#define PKT_MAX_PAYLOAD     128             // Maximum payload bytes per packet

// ─────────────────────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────────────────────

/**
 * @brief FreeRTOS task — receives UART bytes, parses packets,
 *        updates framebuffer, triggers buffer swap.
 *        Pin to Core 0.
 *        Never returns.
 *
 * @param pvParameters  unused, pass NULL
 */
void eth_uart_task(void *pvParameters);