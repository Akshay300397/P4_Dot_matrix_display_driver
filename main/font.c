/*
 * font.c
 * Active font selector
 *
 * ┌─────────────────────────────────────────────────────┐
 * │  TO CHANGE FONT: edit only the #include line below  │
 * │  and change ACTIVE_FONT to match.                   │
 * │  Nothing else in the project needs to change.       │
 * └─────────────────────────────────────────────────────┘
 *
 * Step 1: Choose your font file:
 *           font_5x7.c  — Standard 5×7, full ASCII (default)
 *           font_3x5.c  — Compact 3×5, numbers + basic symbols
 *
 * Step 2: Change the include and extern below to match:
 *           #include "font_5x7.c"   ← for 5×7
 *           #include "font_3x5.c"   ← for 3×5
 *
 * Step 3: Change ACTIVE_FONT to match:
 *           #define ACTIVE_FONT   font_5x7   ← for 5×7
 *           #define ACTIVE_FONT   font_3x5   ← for 3×5
 *
 * Step 4: Rebuild. Done.
 */

#include "font.h"

// ─────────────────────────────────────────────────────────────
//  FONT SELECTION — EDIT ONLY THESE TWO LINES TO CHANGE FONT
// ─────────────────────────────────────────────────────────────

#include "font_5x7.c"           // ← swap this line to change font
#define ACTIVE_FONT  font_5x7   // ← swap this name to match above

// ─────────────────────────────────────────────────────────────
//  PUBLIC API — do not edit below this line
// ─────────────────────────────────────────────────────────────

// Forward declaration of the selected font descriptor
// (defined inside the included font_*.c file above)
extern const font_t ACTIVE_FONT;

const font_t *font_get_active(void)
{
    return &ACTIVE_FONT;
}