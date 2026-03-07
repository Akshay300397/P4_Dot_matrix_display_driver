#pragma once

/*
 * framebuffer.h
 * Double-buffered framebuffer for 128×32 HUB75 display
 * Color format: 3-bit per pixel (0bRGB) — 8 colors
 *               fits in 1 byte, uses only lower 3 bits
 *
 * Memory: 2 × (128 × 32) = 8,192 bytes — trivial on ESP32-S3
 *
 * Thread safety:
 *   Core 1 (hub75_refresh_task) reads display_buf — read only, no lock needed
 *   Core 0 (eth_uart_task)      writes back_buf   — then calls framebuffer_swap()
 *   Swap is mutex-protected atomic pointer exchange — zero tearing
 */

#include <stdint.h>
#include <stdbool.h>
#include "hub75.h"

// ─────────────────────────────────────────────────────────────
//  3-BIT COLOR HELPERS
//  Each pixel stored as 0b00000RGB in one byte
// ─────────────────────────────────────────────────────────────

#define COLOR_BLACK     0b00000000
#define COLOR_RED       0b00000100
#define COLOR_GREEN     0b00000010
#define COLOR_BLUE      0b00000001
#define COLOR_YELLOW    0b00000110
#define COLOR_CYAN      0b00000011
#define COLOR_MAGENTA   0b00000101
#define COLOR_WHITE     0b00000111

// ─────────────────────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────────────────────

/**
 * @brief Initialize both framebuffers to black and create swap mutex.
 *        Call once from app_main() before starting tasks.
 */
void framebuffer_init(void);

/**
 * @brief Get pointer to the back buffer (write target).
 *        Only Core 0 / eth_uart_task should write here.
 *
 * @return Pointer to back buffer array [PANEL_H][PANEL_W]
 */
uint8_t *framebuffer_get_back(void);

/**
 * @brief Get pointer to the display buffer (read-only for refresh task).
 *        Core 1 reads this continuously — never write to it directly.
 *
 * @return Const pointer to display buffer array
 */
const uint8_t *framebuffer_get_display(void);

/**
 * @brief Atomically swap back buffer and display buffer.
 *        Call from Core 0 after finishing a complete frame update.
 *        Thread-safe via mutex.
 */
void framebuffer_swap(void);

/**
 * @brief Clear the back buffer to black (all pixels off).
 */
void framebuffer_clear_back(void);

/**
 * @brief Set a single pixel in the back buffer.
 *        Color is 3-bit: use COLOR_* defines above.
 *
 * @param x      Column 0–127
 * @param y      Row    0–31
 * @param color  3-bit color value (0bRGB)
 */
void framebuffer_set_pixel(int x, int y, uint8_t color);

/**
 * @brief Fill a rectangle in the back buffer with a solid color.
 *
 * @param x      Start column
 * @param y      Start row
 * @param w      Width  in pixels
 * @param h      Height in pixels
 * @param color  3-bit color value
 */
void framebuffer_fill_rect(int x, int y, int w, int h, uint8_t color);

/**
 * @brief Draw a single character using the built-in 5×7 font.
 *
 * @param x      Top-left column of character
 * @param y      Top-left row of character
 * @param c      ASCII character (32–126)
 * @param color  3-bit color value
 */
void framebuffer_draw_char(int x, int y, char c, uint8_t color);

/**
 * @brief Draw a null-terminated string using the built-in 5×7 font.
 *        Characters are 6px wide (5px glyph + 1px spacing).
 *
 * @param x      Start column
 * @param y      Top row
 * @param str    Null-terminated ASCII string
 * @param color  3-bit color value
 */
void framebuffer_draw_string(int x, int y, const char *str, uint8_t color);

/**
 * @brief Draw a horizontal line in the back buffer.
 */
void framebuffer_draw_hline(int x, int y, int len, uint8_t color);

/**
 * @brief Draw a vertical line in the back buffer.
 */
void framebuffer_draw_vline(int x, int y, int len, uint8_t color);

/**
 * @brief Draw a rectangle outline in the back buffer.
 */
void framebuffer_draw_rect(int x, int y, int w, int h, uint8_t color);