/*
 * display_uart.c
 * UART-driven display update
 *
 * Follows the exact same pattern as draw_default_content():
 *
 *   void draw_default_content(void) {
 *       framebuffer_clear_back();
 *       framebuffer_draw_string(20,  6, "...", COLOR_RED);
 *       framebuffer_draw_string(20, 19, "...", COLOR_RED);
 *       framebuffer_swap();
 *   }
 *
 * Here we store 6 field strings, build two row strings from them,
 * and redraw the same way every time a field is updated via UART.
 *
 * Field → display position mapping:
 *   Field 1 (D1): x=26,  y=6
 *   Field 2 (D2): x=60,  y=6
 *   Field 3 (D3): x=94,  y=6
 *   Field 4 (D4): x=26,  y=19
 *   Field 5 (D5): x=60,  y=19
 *   Field 6 (D6): x=94,  y=19
 */

#include "display_uart.h"
#include "framebuffer.h"
#include "esp_log.h"

#include <string.h>
#include <stdint.h>

static const char *TAG = "DISP_UART";

// ─────────────────────────────────────────────────────────────
//  FIELD BUFFERS  —  index 1–6, [0] unused
// ─────────────────────────────────────────────────────────────

static char field_text[DISP_FIELD_COUNT + 1][DISP_FIELD_CHARS + 1];

// ─────────────────────────────────────────────────────────────
//  REDRAW
//  Mirrors draw_default_content() exactly:
//    clear → draw each field string at its fixed position → swap
// ─────────────────────────────────────────────────────────────

static void redraw_all(void)
{
    framebuffer_clear_back();

    // Row 1: Fields 1, 2, 3
    framebuffer_draw_string(DISP_X_OFFSET + 0 * DISP_FIELD_SPACING,
                            DISP_ROW1_Y,
                            field_text[1], COLOR_RED);

    framebuffer_draw_string(DISP_X_OFFSET + 1 * DISP_FIELD_SPACING,
                            DISP_ROW1_Y,
                            field_text[2], COLOR_RED);

    framebuffer_draw_string(DISP_X_OFFSET + 2 * DISP_FIELD_SPACING,
                            DISP_ROW1_Y,
                            field_text[3], COLOR_RED);

    // Row 2: Fields 4, 5, 6
    framebuffer_draw_string(DISP_X_OFFSET + 0 * DISP_FIELD_SPACING,
                            DISP_ROW2_Y,
                            field_text[4], COLOR_RED);

    framebuffer_draw_string(DISP_X_OFFSET + 1 * DISP_FIELD_SPACING,
                            DISP_ROW2_Y,
                            field_text[5], COLOR_RED);

    framebuffer_draw_string(DISP_X_OFFSET + 2 * DISP_FIELD_SPACING,
                            DISP_ROW2_Y,
                            field_text[6], COLOR_RED);

    framebuffer_swap();
}

// ─────────────────────────────────────────────────────────────
//  PUBLIC: INIT
// ─────────────────────────────────────────────────────────────

void display_uart_init(void)
{
    memset(field_text, 0, sizeof(field_text));
    ESP_LOGI(TAG, "Display UART init OK");
}

// ─────────────────────────────────────────────────────────────
//  PUBLIC: UPDATE FROM UART
// ─────────────────────────────────────────────────────────────

void display_update_from_uart(int field, const char *text)
{
    if (field < 1 || field > DISP_FIELD_COUNT) {
        ESP_LOGW(TAG, "Invalid field %d", field);
        return;
    }

    // Store text into field buffer
    strncpy(field_text[field], text, DISP_FIELD_CHARS);
    field_text[field][DISP_FIELD_CHARS] = '\0';

    // Calculate display position for log
    int col = (field - 1) % 3;
    int row = (field - 1) / 3;
    int x   = DISP_X_OFFSET + col * DISP_FIELD_SPACING;
    int y   = (row == 0) ? DISP_ROW1_Y : DISP_ROW2_Y;

    ESP_LOGI(TAG, "Field %d = \"%s\"  →  x=%d y=%d", field, field_text[field], x, y);

    // Redraw entire display — same as draw_default_content()
    redraw_all();
}

// ─────────────────────────────────────────────────────────────
//  PUBLIC: CLEAR ALL
// ─────────────────────────────────────────────────────────────

void display_uart_clear(void)
{
    memset(field_text, 0, sizeof(field_text));
    framebuffer_clear_back();
    framebuffer_swap();
    ESP_LOGI(TAG, "Display cleared");
}