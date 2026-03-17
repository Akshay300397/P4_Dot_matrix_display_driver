/*
 * display_content1.c  —  STEP 2:Static Content(rearranged the columns and rows 
    to start the cursor from panel-2 and continue to panel-1)
 
 * Double panel: 128 × 32 pixels (PANEL_W=128, PANEL_H=32)
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
#define PATTERN_HOLD_MS   500    // each test pattern visible for 1 second

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
        vTaskDelay(pdMS_TO_TICKS(PATTERN_HOLD_MS));
    }

    ESP_LOGI(TAG, "=== TWO PANEL RGB TEST COMPLETE ===");
    ESP_LOGI(TAG, "Report: what color and location appeared for each step.");
}

void draw_default_content(void)
{
    ESP_LOGI(TAG, "Drawing default static content (64x32)");

    framebuffer_clear_back();

    framebuffer_draw_string(20,  6, " 0090 R3K5 IND", COLOR_RED);
    framebuffer_draw_string(20, 19, " 0091 R3K5 IND",  COLOR_RED);
    

    framebuffer_swap();

    ESP_LOGI(TAG, "Default content live — display ready");
}
