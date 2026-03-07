#pragma once

/*
 * font.h
 * Font interface for HUB75 LED Matrix framebuffer renderer
 *
 * HOW TO CHANGE THE FONT:
 *   1. Create a new font_<name>.c file (copy font_5x7.c as template)
 *   2. Fill in the glyph data array with your new font bitmaps
 *   3. Update the #include in font.c to point to your new file
 *   4. Update FONT_CHAR_W, FONT_CHAR_H, FONT_CHAR_STEP to match
 *   5. Rebuild — no changes needed in framebuffer.c or anywhere else
 *
 * FONT DATA FORMAT:
 *   Column-major bitmap — each byte represents one vertical column
 *   Bit 0 = top pixel of column, Bit (H-1) = bottom pixel
 *
 *   Example: Letter 'A' in 5×7 font
 *   Column:   0     1     2     3     4
 *             0x7E  0x11  0x11  0x11  0x7E
 *
 *   Bit layout of column 0 (0x7E = 0b01111110):
 *   bit0 (top)    = 0  →  pixel OFF
 *   bit1          = 1  →  pixel ON
 *   bit2          = 1  →  pixel ON
 *   bit3          = 1  →  pixel ON
 *   bit4          = 1  →  pixel ON
 *   bit5          = 1  →  pixel ON
 *   bit6          = 1  →  pixel ON
 *   bit7 (unused) = 0
 *
 * AVAILABLE FONTS (swap by editing font.c):
 *   font_5x7.c   — standard 5×7 pixel font, full ASCII 32–126
 *   font_3x5.c   — compact 3×5 pixel font, numbers + basic chars only
 *                  useful for displaying more text on small displays
 */

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────
//  FONT DESCRIPTOR
//  One instance of this struct describes the active font.
//  framebuffer.c reads this struct — never hardcodes font details.
// ─────────────────────────────────────────────────────────────

typedef struct {
    const uint8_t (*glyphs)[8];   // Pointer to glyph data array
                                  // Each glyph: up to 8 columns × 1 byte
                                  // Unused columns at end are 0x00
    uint8_t  char_w;              // Glyph width in pixels (e.g. 5)
    uint8_t  char_h;              // Glyph height in pixels (e.g. 7)
    uint8_t  char_step;           // Advance per character including spacing
                                  // (typically char_w + 1)
    uint8_t  first_char;          // First ASCII code in the glyph table
    uint8_t  last_char;           // Last  ASCII code in the glyph table
} font_t;

// ─────────────────────────────────────────────────────────────
//  ACTIVE FONT
//  Defined in font.c — points to whichever font is currently selected.
//  framebuffer.c includes this header and calls font_get_active().
// ─────────────────────────────────────────────────────────────

/**
 * @brief  Get pointer to the currently active font descriptor.
 *         Called by framebuffer_draw_char() and framebuffer_draw_string().
 *
 * @return Const pointer to active font_t struct.
 *         Never returns NULL.
 */
const font_t *font_get_active(void);