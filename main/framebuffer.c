/*
 * framebuffer.c
 * Double-buffered framebuffer with drawing primitives
 *
 * Buffer layout:
 *   framebuffer[y * PANEL_W + x]
 *   y: 0–31, x: 0–127
 *   Each byte: 0b00000RGB (3-bit color)
 *
 * Double buffer:
 *   display_buf → read by Core 1 refresh task (never written during use)
 *   back_buf    → written by Core 0 application / UART task
 *   swap()      → atomic pointer exchange protected by mutex
 *
 * Font:
 *   Font data is NOT embedded here.
 *   framebuffer_draw_char() and framebuffer_draw_string() call
 *   font_get_active() from font.h to get the current font.
 *   To change font: edit font.c only — no changes needed here.
 */

#include "framebuffer.h"
#include "hub75.h"
#include "font.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <string.h>
#include <stdint.h>

static const char *TAG = "FRAMEBUF";

// ─────────────────────────────────────────────────────────────
//  FRAMEBUFFER STORAGE
//  Two full buffers in DRAM — 4096 bytes each = 8192 bytes total
//  Trivial for ESP32-S3 with 512KB SRAM
// ─────────────────────────────────────────────────────────────

static DRAM_ATTR uint8_t buf_A[PANEL_W * PANEL_H];
static DRAM_ATTR uint8_t buf_B[PANEL_W * PANEL_H];

// Pointers — swapped atomically on framebuffer_swap()
static volatile uint8_t *display_buf = buf_A;
static          uint8_t *back_buf    = buf_B;

// Mutex protecting the pointer swap operation
static SemaphoreHandle_t swap_mutex = NULL;

// ─────────────────────────────────────────────────────────────
//  INIT
// ─────────────────────────────────────────────────────────────

void framebuffer_init(void)
{
    memset(buf_A, 0, sizeof(buf_A));
    memset(buf_B, 0, sizeof(buf_B));
    display_buf = buf_A;
    back_buf    = buf_B;
    swap_mutex  = xSemaphoreCreateMutex();
    configASSERT(swap_mutex != NULL);
    ESP_LOGI(TAG, "Framebuffer initialized — %d×%d × 2 buffers = %d bytes",
             PANEL_W, PANEL_H, (PANEL_W * PANEL_H) * 2);
}

// ─────────────────────────────────────────────────────────────
//  BUFFER ACCESS
// ─────────────────────────────────────────────────────────────

uint8_t *framebuffer_get_back(void)
{
    return back_buf;
}

const uint8_t *framebuffer_get_display(void)
{
    return (const uint8_t *)display_buf;
}

// ─────────────────────────────────────────────────────────────
//  ATOMIC BUFFER SWAP
//  Exchanges display_buf and back_buf pointers under mutex
//  Called from Core 0 after a complete frame is ready in back_buf
//  Core 1 will pick up the new display_buf on next refresh cycle
// ─────────────────────────────────────────────────────────────

void framebuffer_swap(void)
{
    xSemaphoreTake(swap_mutex, portMAX_DELAY);

    uint8_t *tmp    = (uint8_t *)display_buf;
    display_buf     = back_buf;
    back_buf        = tmp;

    xSemaphoreGive(swap_mutex);
}

// ─────────────────────────────────────────────────────────────
//  DRAWING PRIMITIVES
//  All write to back_buf only
//  Call framebuffer_swap() after all drawing is done
// ─────────────────────────────────────────────────────────────

void framebuffer_clear_back(void)
{
    memset(back_buf, 0, PANEL_W * PANEL_H);
}

void framebuffer_set_pixel(int x, int y, uint8_t color)
{
    if (x < 0 || x >= PANEL_W || y < 0 || y >= PANEL_H) return;
    back_buf[y * PANEL_W + x] = color & 0x07;
}

void framebuffer_fill_rect(int x, int y, int w, int h, uint8_t color)
{
    for (int row = y; row < y + h; row++) {
        for (int col = x; col < x + w; col++) {
            framebuffer_set_pixel(col, row, color);
        }
    }
}

void framebuffer_draw_hline(int x, int y, int len, uint8_t color)
{
    for (int i = x; i < x + len; i++) {
        framebuffer_set_pixel(i, y, color);
    }
}

void framebuffer_draw_vline(int x, int y, int len, uint8_t color)
{
    for (int i = y; i < y + len; i++) {
        framebuffer_set_pixel(x, i, color);
    }
}

void framebuffer_draw_rect(int x, int y, int w, int h, uint8_t color)
{
    framebuffer_draw_hline(x,         y,         w, color);  // top
    framebuffer_draw_hline(x,         y + h - 1, w, color);  // bottom
    framebuffer_draw_vline(x,         y,         h, color);  // left
    framebuffer_draw_vline(x + w - 1, y,         h, color);  // right
}

// ─────────────────────────────────────────────────────────────
//  FONT RENDERING
//  Uses font_get_active() from font.h — font data lives in font_*.c
//  To change the font: edit font.c only, no changes needed here
// ─────────────────────────────────────────────────────────────

void framebuffer_draw_char(int x, int y, char c, uint8_t color)
{
    const font_t *font = font_get_active();

    // Check character is within this font's supported range
    if ((uint8_t)c < font->first_char || (uint8_t)c > font->last_char) return;

    // Get pointer to glyph columns for this character
    const uint8_t *glyph = font->glyphs[(uint8_t)c - font->first_char];

    // Render each column of the glyph
    for (int col = 0; col < font->char_w; col++) {
        uint8_t col_bits = glyph[col];
        // Render each row bit in this column
        for (int row = 0; row < font->char_h; row++) {
            if (col_bits & (1 << row)) {
                framebuffer_set_pixel(x + col, y + row, color);
            }
        }
    }
}

void framebuffer_draw_string(int x, int y, const char *str, uint8_t color)
{
    const font_t *font = font_get_active();
    int cursor_x = x;

    while (*str) {
        framebuffer_draw_char(cursor_x, y, *str, color);
        cursor_x += font->char_step;
        str++;
        // Stop if next character would start past the right edge
        if (cursor_x >= PANEL_W) break;
    }
}