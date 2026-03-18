#pragma once

/*
 * display_uart.h
 * UART-driven display update
 *
 * Follows exact same pattern as draw_default_content() in display_content1.c:
 *   framebuffer_clear_back() → framebuffer_draw_string() → framebuffer_swap()
 *
 * Layout (font 5×7, 6px advance, 4 chars per field):
 *
 *   Row 1 (y=6):   Field1@x=26  Field2@x=60  Field3@x=94
 *   Row 2 (y=19):  Field4@x=26  Field5@x=60  Field6@x=94
 */

#include <stdint.h>

// ── Layout constants (match display_content1.c) ───────────────
#define DISP_X_OFFSET        26
#define DISP_FIELD_SPACING   34
#define DISP_ROW1_Y           6
#define DISP_ROW2_Y          19
#define DISP_FIELD_CHARS      4
#define DISP_FIELD_COUNT      6

/**
 * @brief Init field buffers to empty. Call from app_main() before tasks.
 */
void display_uart_init(void);

/**
 * @brief Receive one field update from UART and redraw display.
 *        field=1–6, text=max 4 chars null-terminated.
 */
void display_update_from_uart(int field, const char *text);

/**
 * @brief Clear all fields and blank the display.
 */
void display_uart_clear(void);