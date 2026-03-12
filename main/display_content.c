/*
 * display_content.c  —  STEP 1: Hardware Check + Static Content
 * ══════════════════════════════════════════════════════════════
 *
 * Single panel: 64 × 32 pixels (PANEL_W=64, PANEL_H=32)
 *
 * Contains:
 *   run_hardware_check()   — 5 test patterns at boot (~6 seconds)
 *   draw_default_content() — static screen, edit to customise
 *
 * ══════════════════════════════════════════════════════════════
 * C CONCEPT: STATIC vs NON-STATIC FUNCTIONS
 * ══════════════════════════════════════════════════════════════
 * static void pattern_xxx()  → private, only callable in THIS file
 * void run_hardware_check()  → public, callable from any file
 *                               that includes display_content.h
 *
 * Use static on every helper that is only used internally.
 * This prevents accidental naming conflicts across files.
 */

#include "display_content.h"
#include "framebuffer.h"          // framebuffer_* functions, COLOR_* defines
#include "hub75.h"                // PANEL_W, PANEL_H constants

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"        // vTaskDelay, pdMS_TO_TICKS
#include "esp_log.h"

static const char *TAG = "DISPLAY";

/*
 * ══════════════════════════════════════════════════════════════
 * C CONCEPT: #define CONSTANTS — avoid magic numbers
 * ══════════════════════════════════════════════════════════════
 * PATTERN_HOLD_MS and FINAL_HOLD_MS are defined once here.
 * Change 1000 to 500 → all patterns shorten automatically.
 * No need to hunt through the code changing individual numbers.
 */
#define PATTERN_HOLD_MS   1000    // each test pattern visible for 1 second
#define FINAL_HOLD_MS     2000    // text pattern holds for 2 seconds

/* ══════════════════════════════════════════════════════════════
 * PRIVATE PATTERN FUNCTIONS
 * Each pattern is a separate static function.
 *
 * C CONCEPT: SMALL FOCUSED FUNCTIONS
 * ════════════════════════════════════
 * Each function does exactly ONE thing.
 * run_hardware_check() reads like a checklist — clean and clear.
 * If a pattern fails on hardware, you know exactly which function
 * to look at without reading hundreds of lines.
 * ══════════════════════════════════════════════════════════════
 */

/*
 * Pattern 1: Left half RED, Right half BLUE
 * ─────────────────────────────────────────
 * Splits the 64px panel into two 32px halves.
 * Confirms: SPI data reaches the full width of the panel.
 *
 * PANEL_W   = 64  (from hub75.h)
 * PANEL_W/2 = 32  (each half)
 * PANEL_H   = 32
 *
 * C CONCEPT: USING #defined CONSTANTS IN EXPRESSIONS
 * ────────────────────────────────────────────────────
 * PANEL_W/2 is computed by the compiler at compile time.
 * It is NOT a runtime division. The compiler substitutes
 * 64/2 = 32 directly into the machine code.
 * This is called a "constant expression".
 */
static void pattern_color_bars(void)
{
    ESP_LOGI(TAG, "Pattern 1: RED left | BLUE right  (64px panel split)");

    framebuffer_clear_back();
    framebuffer_fill_rect(0,         0, PANEL_W/2, PANEL_H, COLOR_RED);   // left 32px
    framebuffer_fill_rect(PANEL_W/2, 0, PANEL_W/2, PANEL_H, COLOR_BLUE);  // right 32px
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(PATTERN_HOLD_MS));
}

/*
 * Pattern 2: Top half GREEN, Bottom half YELLOW
 * ──────────────────────────────────────────────
 * Splits the 32px height into two 16px bands.
 * Confirms: ABCD row addressing is correct.
 * If rows are scrambled, this pattern will look broken/mixed.
 *
 * PANEL_H   = 32
 * PANEL_H/2 = 16  (each band)
 */
static void pattern_horizontal_bands(void)
{
    ESP_LOGI(TAG, "Pattern 2: GREEN top | YELLOW bottom  (row addressing check)");

    framebuffer_clear_back();
    framebuffer_fill_rect(0, 0,         PANEL_W, PANEL_H/2, COLOR_GREEN);   // rows 0-15
    framebuffer_fill_rect(0, PANEL_H/2, PANEL_W, PANEL_H/2, COLOR_YELLOW);  // rows 16-31
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(PATTERN_HOLD_MS));
}

/*
 * Pattern 3: All 7 colors in vertical bands
 * ──────────────────────────────────────────
 * 7 color bands + 1 black band = 8 bands × 8px each = 64px total.
 * Confirms: R, G, B channels all work independently.
 * If one channel is dead, you will see wrong colors here.
 *
 * C CONCEPT: STRUCT + ARRAY FOR TABLE-DRIVEN CODE
 * ─────────────────────────────────────────────────
 * struct { int x; uint8_t color; } defines a custom type inline.
 * bands[] is an ARRAY of that struct — each element is one band.
 *
 * sizeof(bands) / sizeof(bands[0]) safely gives the array length.
 * Add/remove a band: the loop count updates automatically.
 * No hardcoded '8' that would go stale if you edit the table.
 */
static void pattern_all_colors(void)
{
    ESP_LOGI(TAG, "Pattern 3: All 7 colors (8px bands x 8 = 64px)");

    /*
     * 64px panel / 8 bands = 8px per band exactly
     * band_w is computed at compile time from PANEL_W
     */
    const int band_w = PANEL_W / 8;   // = 64/8 = 8px per band

    struct { int x; uint8_t color; } bands[] = {
        { 0 * band_w, COLOR_RED     },
        { 1 * band_w, COLOR_GREEN   },
        { 2 * band_w, COLOR_BLUE    },
        { 3 * band_w, COLOR_YELLOW  },
        { 4 * band_w, COLOR_CYAN    },
        { 5 * band_w, COLOR_MAGENTA },
        { 6 * band_w, COLOR_WHITE   },
        { 7 * band_w, COLOR_BLACK   },   // LEDs off — confirms pixels turn off
    };

    int num_bands = sizeof(bands) / sizeof(bands[0]);

    framebuffer_clear_back();
    for (int i = 0; i < num_bands; i++) {
        framebuffer_fill_rect(bands[i].x, 0, band_w, PANEL_H, bands[i].color);
    }
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(PATTERN_HOLD_MS));
}

/*
 * Pattern 4: Single pixel at each corner
 * ──────────────────────────────────────
 * Most important test — checks exact pixel addressing.
 *
 * Expected result:
 *   RED    at top-left     (0, 0)
 *   GREEN  at top-right    (63, 0)
 *   BLUE   at bottom-left  (0, 31)
 *   WHITE  at bottom-right (63, 31)
 *   YELLOW at center       (31, 15)
 *
 * If corners are swapped: X or Y axis is inverted in build_spi_buffer().
 * If wrong color: bit assignment in build_spi_buffer() needs adjustment.
 *
 * C CONCEPT: PANEL_W-1 and PANEL_H-1
 * ────────────────────────────────────
 * Pixel columns are numbered 0 to PANEL_W-1 (0 to 63).
 * Pixel rows    are numbered 0 to PANEL_H-1 (0 to 31).
 * The last valid pixel is at (PANEL_W-1, PANEL_H-1) = (63, 31).
 * Writing to (64, 32) would be out of bounds → ignored by bounds check.
 */
static void pattern_corner_pixels(void)
{
    ESP_LOGI(TAG, "Pattern 4: Corner pixels");
    ESP_LOGI(TAG, "  Expected: TL=RED  TR=GREEN  BL=BLUE  BR=WHITE  C=YELLOW");

    framebuffer_clear_back();
    framebuffer_set_pixel(0,          0,          COLOR_RED);    // top-left
    framebuffer_set_pixel(PANEL_W-1,  0,          COLOR_GREEN);  // top-right
    framebuffer_set_pixel(0,          PANEL_H-1,  COLOR_BLUE);   // bottom-left
    framebuffer_set_pixel(PANEL_W-1,  PANEL_H-1,  COLOR_WHITE);  // bottom-right
    framebuffer_set_pixel(PANEL_W/2,  PANEL_H/2,  COLOR_YELLOW); // center
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(PATTERN_HOLD_MS));
}

/*
 * Pattern 5: Text on single panel
 * ─────────────────────────────────
 * Confirms: font rendering, character spacing, string drawing.
 *
 * 64px panel / 6px per char = 10 characters max per row.
 * Keep strings ≤ 10 characters or they will be clipped.
 *
 * C CONCEPT: STRING LITERALS
 * ───────────────────────────
 * "64X32" is stored in flash as: {'6','4','X','3','2','\0'}
 * framebuffer_draw_string() reads until the '\0' terminator.
 * The '\0' is added automatically by the C compiler.
 */
static void pattern_text(void)
{
    ESP_LOGI(TAG, "Pattern 5: Text (10 chars max on 64px panel)");

    framebuffer_clear_back();
    framebuffer_draw_string(1, 1,  "64 X 32",   COLOR_RED);     // panel size
    framebuffer_draw_string(1, 12, "ICN6124",   COLOR_GREEN);   // driver IC
    framebuffer_draw_string(1, 23, "ESP32-S3",  COLOR_CYAN);    // MCU
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(FINAL_HOLD_MS));
}

/* ══════════════════════════════════════════════════════════════
 * PUBLIC FUNCTIONS
 * ══════════════════════════════════════════════════════════════
 */

/*
 * run_hardware_check()
 * ─────────────────────
 * Calls each private pattern function in sequence.
 * Total duration: 4 × 1000ms + 1 × 2000ms = 6 seconds.
 *
 * app_main() calls this as a DIRECT FUNCTION CALL — it blocks
 * here until all 5 patterns finish, then returns.
 *
 * C CONCEPT: CALL STACK DURING THIS FUNCTION
 * ────────────────────────────────────────────
 * app_main()
 *   └─ run_hardware_check()       ← you are here
 *         └─ pattern_color_bars() ← called, runs, returns
 *         └─ pattern_horizontal_bands()
 *         └─ pattern_all_colors()
 *         └─ pattern_corner_pixels()
 *         └─ pattern_text()
 *   └─ draw_default_content()     ← runs after this returns
 */
// ══════════════════════════════════════════════════════════════
// DIAGNOSTIC: CHAIN ORDER TEST
// ══════════════════════════════════════════════════════════════
//
// Lights ONE pixel at top-left (0,0) with a specific 6-bit raw
// value written directly into the framebuffer byte — bypassing
// color names entirely. This tells us which physical LED channel
// maps to which bit position.
//
// How to use:
//   Call chain_order_test() instead of run_hardware_check().
//   Look at what lights up on the panel.
//   Report: color and position of the lit pixel(s).
//
// Test sequence (2 seconds each):
//   Step 0: bit0 only = 0x01  → should light R1 (top-left, RED if chain is R1 first)
//   Step 1: bit1 only = 0x02  → next channel in chain
//   Step 2: bit2 only = 0x04
//   Step 3: bit3 only = 0x08
//   Step 4: bit4 only = 0x10
//   Step 5: bit5 only = 0x20
//
// For each step note: which LED color lights up, top or bottom half.
// That gives us the exact chain order for build_spi_buffer().
// ══════════════════════════════════════════════════════════════
void chain_order_test(void)
{
    ESP_LOGI(TAG, "=== CHAIN ORDER DIAGNOSTIC ===");

    /*
     * 6 steps, 2 seconds each. Fill entire top or bottom half
     * with a single color, other half black.
     * Report: which color appeared on which rows for each step.
     */
    typedef struct { uint8_t top; uint8_t bot; const char *label; } step_t;
    const step_t steps[] = {
        { COLOR_RED,   COLOR_BLACK, "top=RED,   bot=BLACK" },
        { COLOR_GREEN, COLOR_BLACK, "top=GREEN, bot=BLACK" },
        { COLOR_BLUE,  COLOR_BLACK, "top=BLUE,  bot=BLACK" },
        { COLOR_BLACK, COLOR_RED,   "top=BLACK, bot=RED"   },
        { COLOR_BLACK, COLOR_GREEN, "top=BLACK, bot=GREEN" },
        { COLOR_BLACK, COLOR_BLUE,  "top=BLACK, bot=BLUE"  },
    };

    for (int i = 0; i < 6; i++) {
        ESP_LOGI(TAG, "Step %d: %s", i, steps[i].label);
        framebuffer_clear_back();
        framebuffer_fill_rect(0, 0,         PANEL_W, SCAN_ROWS, steps[i].top);
        framebuffer_fill_rect(0, SCAN_ROWS, PANEL_W, SCAN_ROWS, steps[i].bot);
        framebuffer_swap();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGI(TAG, "=== CHAIN ORDER DIAGNOSTIC COMPLETE ===");
    ESP_LOGI(TAG, "Report: which color lit up on which rows for each step.");
}

// ══════════════════════════════════════════════════════════════
// DIAGNOSTIC: TWO PANEL RGB SEQUENCE TEST
//
// Lights each panel half (P1 top, P1 bottom, P2 top, P2 bottom)
// with RED, GREEN, BLUE in sequence. 12 steps total, 2s each.
//
// Expected output if chain order is correct:
//   Step  0: P1 top  = RED,   all else dark
//   Step  1: P1 top  = GREEN, all else dark
//   Step  2: P1 top  = BLUE,  all else dark
//   Step  3: P1 bot  = RED,   all else dark
//   Step  4: P1 bot  = GREEN, all else dark
//   Step  5: P1 bot  = BLUE,  all else dark
//   Step  6: P2 top  = RED,   all else dark
//   Step  7: P2 top  = GREEN, all else dark
//   Step  8: P2 top  = BLUE,  all else dark
//   Step  9: P2 bot  = RED,   all else dark
//   Step 10: P2 bot  = GREEN, all else dark
//   Step 11: P2 bot  = BLUE,  all else dark
//
// Report: for each step, what color lights up and where.
// ══════════════════════════════════════════════════════════════
void two_panel_rgb_test(void)
{
    ESP_LOGI(TAG, "=== TWO PANEL RGB SEQUENCE TEST ===");

    typedef struct {
        int      x;           // start column
        int      y;           // start row
        int      w;           // width
        int      h;           // height
        uint8_t  color;
        const char *label;
    } step_t;

    // P1 occupies columns   0–63
    // P2 occupies columns  64–127
    // Top half  = rows  0–15  (SCAN_ROWS = 16)
    // Bottom half = rows 16–31
    const step_t steps[] = {
        {  0,  0, 64, SCAN_ROWS, COLOR_RED,    "P1 top  RED"   },
        {  0,  0, 64, SCAN_ROWS, COLOR_GREEN,  "P1 top  GREEN" },
        {  0,  0, 64, SCAN_ROWS, COLOR_BLUE,   "P1 top  BLUE"  },
        {  0, SCAN_ROWS, 64, SCAN_ROWS, COLOR_RED,    "P1 bot  RED"   },
        {  0, SCAN_ROWS, 64, SCAN_ROWS, COLOR_GREEN,  "P1 bot  GREEN" },
        {  0, SCAN_ROWS, 64, SCAN_ROWS, COLOR_BLUE,   "P1 bot  BLUE"  },
        { 64,  0, 64, SCAN_ROWS, COLOR_RED,    "P2 top  RED"   },
        { 64,  0, 64, SCAN_ROWS, COLOR_GREEN,  "P2 top  GREEN" },
        { 64,  0, 64, SCAN_ROWS, COLOR_BLUE,   "P2 top  BLUE"  },
        { 64, SCAN_ROWS, 64, SCAN_ROWS, COLOR_RED,    "P2 bot  RED"   },
        { 64, SCAN_ROWS, 64, SCAN_ROWS, COLOR_GREEN,  "P2 bot  GREEN" },
        { 64, SCAN_ROWS, 64, SCAN_ROWS, COLOR_BLUE,   "P2 bot  BLUE"  },
    };

    int num_steps = sizeof(steps) / sizeof(steps[0]);

    for (int i = 0; i < num_steps; i++) {
        ESP_LOGI(TAG, "Step %2d: %s", i, steps[i].label);
        framebuffer_clear_back();
        framebuffer_fill_rect(steps[i].x, steps[i].y,
                              steps[i].w, steps[i].h,
                              steps[i].color);
        framebuffer_swap();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGI(TAG, "=== TWO PANEL RGB TEST COMPLETE ===");
    ESP_LOGI(TAG, "Report: what color and location appeared for each step.");
}

void run_hardware_check(void)
{
    ESP_LOGI(TAG, "=== Hardware Check Start (single 64x32 panel) ===");

    pattern_color_bars();
    pattern_horizontal_bands();
    pattern_all_colors();
    pattern_corner_pixels();
    pattern_text();

    ESP_LOGI(TAG, "=== Hardware Check Complete ===");
}

/*
 * draw_default_content()
 * ──────────────────────
 * *** EDIT THIS FUNCTION to show your desired static content ***
 *
 * This is the screen shown after the hardware check completes.
 * It stays on display until reset (Step 1).
 * In Step 2+, UART commands will overwrite it.
 *
 * COORDINATE REFERENCE for 64×32 panel:
 * ──────────────────────────────────────
 *  Origin (0,0) = top-left corner
 *  X: 0 (left) → 63 (right)
 *  Y: 0 (top)  → 31 (bottom)
 *
 *  Font: 5×7 pixels, 6px advance (5px glyph + 1px gap)
 *  Max chars per row: 64 / 6 = 10 characters
 *  Safe Y positions for 3 text rows: y=2, y=12, y=22
 *  (7px font height + some spacing)
 *
 *  framebuffer_draw_rect(x, y, w, h, color)
 *    → draws outline rectangle, NOT filled
 *  framebuffer_fill_rect(x, y, w, h, color)
 *    → draws FILLED rectangle
 */
void draw_default_content(void)
{
    ESP_LOGI(TAG, "Drawing default static content (64x32)");

    framebuffer_clear_back();

    // No border — border's top/bottom rows merged visually with text
    // and caused ghost confusion at row 0.
    //
    // 5×7 font layout on 32px tall panel:
    //   y= 1 : row 1  pixels 1–7   (font height = 7px)
    //   y=12 : row 2  pixels 12–18 (gap rows 8–11 = 4px breathing room)
    //   y=23 : row 3  pixels 23–29 (gap rows 19–22 = 4px breathing room)
    //                               rows 30–31 = 2px bottom margin
    framebuffer_draw_string(1,  1, "HUB75 OK", COLOR_RED);
    framebuffer_draw_string(1, 12, "Panel  1",  COLOR_GREEN);
    framebuffer_draw_string(1, 23, "P4 128x32", COLOR_CYAN);
    framebuffer_draw_string(65, 12, "Panel  2",  COLOR_RED);
    framebuffer_draw_string(65, 23, "P4 128x32", COLOR_CYAN);

    framebuffer_swap();

    ESP_LOGI(TAG, "Default content live — display ready");
}

// ══════════════════════════════════════════════════════════════
// DIAGNOSTIC: ROW SCAN ORDER TEST
// Lights one framebuffer row at a time (RED), holds 1 second.
// Watch which PHYSICAL row lights up for each scan address 0–15.
// Report: for scan address N, which physical row on panel lights?
// ══════════════════════════════════════════════════════════════
void row_scan_test(void)
{
    ESP_LOGI(TAG, "=== ROW SCAN ORDER TEST ===");
    ESP_LOGI(TAG, "Watch which PHYSICAL row lights for each step.");

    for (int r = 0; r < SCAN_ROWS; r++) {
        ESP_LOGI(TAG, "Step %2d: lighting framebuffer row %d (top half)", r, r);
        framebuffer_clear_back();
        // Light only row r in the top half — solid RED
        framebuffer_fill_rect(0, r, PANEL_W, 1, COLOR_RED);
        framebuffer_swap();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "=== ROW SCAN TEST COMPLETE ===");
}