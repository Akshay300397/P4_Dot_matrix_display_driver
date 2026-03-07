#pragma once

/*
 * hub75.h
 * HUB75 LED Matrix Driver — ESP32-S3 / ESP-IDF v5.x
 * Panel:  2× P4 64×32 ICN6124EJ chained = 128×32
 * Scan:   1:16 (ABCD addressing)
 * Author: Generated for ESP32-S3 bare-metal project
 */

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────
//  PANEL CONFIGURATION
//  Match these to your physical panel spec
// ─────────────────────────────────────────────────────────────

#define PANEL_W         128     // Total width  = 2 panels × 64px
#define PANEL_H          32     // Total height = 32px
#define SCAN_ROWS        16     // 1:16 scan → 16 row address lines (ABCD)
#define PANELS_COUNT      2     // Number of chained panels

// ─────────────────────────────────────────────────────────────
//  ESP32-S3 PIN ASSIGNMENTS
//  Safe pins — avoids strapping pins (0,3,45,46) and flash (26-32)
//  Change these to match your PCB layout
// ─────────────────────────────────────────────────────────────

// SPI data pins (SPI2 / HSPI)
#define HUB75_PIN_MOSI      11      // Data → R1 (after PI→PO jumpers)
#define HUB75_PIN_CLK       12      // SPI clock → CLK

// Control pins (plain GPIO)
#define HUB75_PIN_LAT       13      // Latch (active HIGH pulse)
#define HUB75_PIN_OE        14      // Output Enable (active LOW)

// Row address pins (binary encoded)
#define HUB75_PIN_A          1      // Row address bit 0 (LSB)
#define HUB75_PIN_B          2      // Row address bit 1
#define HUB75_PIN_C          4      // Row address bit 2
#define HUB75_PIN_D          5      // Row address bit 3 (MSB)

// ─────────────────────────────────────────────────────────────
//  TIMING CONFIGURATION
//  Tuned from your working PxMatrix test (setMuxDelay confirmed)
// ─────────────────────────────────────────────────────────────

#define HUB75_SPI_SPEED_HZ  20000000    // 20 MHz — safe for ICN6124 (max ~25MHz)
#define HUB75_MUX_DELAY_US       1      // Row address settling delay (confirmed needed)
#define HUB75_ROW_ON_US          1      // Row display time per refresh cycle

// ─────────────────────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────────────────────

/**
 * @brief Initialize SPI2 and GPIO pins for HUB75 driving.
 *        Call once from app_main() before starting tasks.
 */
void hub75_init(void);

/**
 * @brief FreeRTOS task — continuously refreshes the display.
 *        Pin to Core 1 at highest priority.
 *        Never returns.
 *
 * @param pvParameters  unused, pass NULL
 */
void hub75_refresh_task(void *pvParameters);