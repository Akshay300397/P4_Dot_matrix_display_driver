#pragma once

/*
 * hub75.h
 * HUB75 LED Matrix Driver — ESP32-S3 / ESP-IDF v5.x
 * Panel:  1× P4 64×32 FM6124EJ
 * Scan:   1:16 (ABCD addressing)
 * Author: Generated for ESP32-S3 bare-metal project
 */

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────
//  PANEL CONFIGURATION
//  Match these to your physical panel spec
// ─────────────────────────────────────────────────────────────

#define PANEL_W          64     // Total width  = 1 panel × 64px
#define PANEL_H          32     // Total height = 32px
#define SCAN_ROWS        16     // 1:16 scan → 16 row address lines (ABCD)
#define PANELS_COUNT      1     // Number of chained panels

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
// ─────────────────────────────────────────────────────────────

#define HUB75_SPI_SPEED_HZ  20000000    // 20 MHz SPI clock
#define HUB75_MUX_DELAY_US       8      // Row address settling delay (µs)
#define HUB75_ROW_ON_US          4      // Minimum row dwell time (µs) — brightness via LEDC PWM

// ─────────────────────────────────────────────────────────────
//  OE PWM BRIGHTNESS CONTROL
//  ──────────────────────────
//  FM6124 has NO software brightness register (unlike FM6126A).
//  The datasheet defines only one brightness method:
//    Iout = (1.191V / Rext) × 15   ← hardware resistor on panel PCB
//
//  Software brightness is achieved via PWM on the OE pin:
//    OE LOW  = LEDs on
//    OE HIGH = LEDs off
//  By driving OE with a PWM signal, the duty cycle controls brightness.
//    duty 100% (OE always LOW)  → maximum brightness
//    duty  50%                  → half brightness
//    duty  10%                  → dim
//
//  Implementation: ESP32-S3 LEDC timer drives OE pin in hardware.
//  The refresh task sets OE LOW/HIGH as usual for blanking during
//  data load. Between rows the LEDC PWM controls the actual on-time.
//
//  OE_PWM_FREQ_HZ — PWM carrier frequency
//    Must be >> refresh rate to avoid beating/flicker
//    20kHz is above human hearing and well above any refresh rate
//
//  OE_DUTY_MAX — LEDC duty cycle range (2^OE_DUTY_RES - 1)
//    10-bit resolution = 0..1023
//    Set hub75_set_brightness(percent) maps 0–100 → 0–1023
// ─────────────────────────────────────────────────────────────

#define HUB75_OE_PWM_FREQ_HZ    20000   // 20 kHz OE PWM carrier
#define HUB75_OE_DUTY_RES           10  // 10-bit = 1024 steps
#define HUB75_OE_DUTY_MAX         1023  // (2^10 - 1)
#define HUB75_OE_LEDC_CHANNEL        0  // LEDC channel 0 for OE pin
#define HUB75_OE_LEDC_TIMER          0  // LEDC timer 0

// ─────────────────────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────────────────────

/**
 * @brief Initialize SPI2, GPIO pins, and OE PWM for HUB75 driving.
 *        Call once from app_main() before starting tasks.
 *        Sets brightness to 80% by default.
 */
void hub75_init(void);

/**
 * @brief Set display brightness via OE PWM duty cycle.
 *
 *        FM6124 has no software brightness register.
 *        Brightness is controlled by PWM on the OE pin (Output Enable).
 *        OE LOW = panel on, OE HIGH = panel off.
 *        Duty cycle = fraction of each PWM period where OE is LOW = LEDs lit.
 *
 * @param percent  Brightness 0–100.
 *                 0   = display off (OE always HIGH)
 *                 100 = maximum brightness (OE always LOW)
 *                 80  = recommended default (reduces heat, extends LED life)
 */
void hub75_set_brightness(uint8_t percent);

/**
 * @brief FreeRTOS task — continuously refreshes the display.
 *        Pin to Core 1 at highest priority.
 *        Never returns.
 *
 * @param pvParameters  unused, pass NULL
 */
void hub75_refresh_task(void *pvParameters);