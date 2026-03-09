/*
 * main.c  —  STEP 1: Static Display + Hardware Check
 * ═══════════════════════════════════════════════════
 *
 * Single panel: 64 × 32  (set by PANEL_W=64, PANEL_H=32 in hub75.h)
 *
 * Boot sequence:
 *   A. framebuffer_init()           — zero buffers, create mutex
 *   B. hub75_init()                 — configure SPI2 + GPIO
 *   C. xTaskCreatePinnedToCore()    — start refresh task on Core 1
 *   D. run_hardware_check()         — 5 test patterns, ~6 seconds
 *   E. draw_default_content()       — static screen, stays forever
 *
 * Step 2 adds:  UART task on Core 0
 * Step 3 adds:  UART protocol + CRC
 */

#include "freertos/FreeRTOS.h"   // FreeRTOS core — scheduler, types
#include "freertos/task.h"        // xTaskCreatePinnedToCore, vTaskDelay
#include "esp_log.h"              // ESP_LOGI

#include "hub75.h"                // hub75_init(), hub75_refresh_task(), PANEL_W/H
#include "framebuffer.h"          // framebuffer_init(), COLOR_* defines
#include "display_content.h"      // run_hardware_check(), draw_default_content()

/*
 * ═══════════════════════════════════════════════════════════
 * C CONCEPT: static const char *TAG
 * ═══════════════════════════════════════════════════════════
 * Every .c file in the project has its own TAG.
 * Log output looks like:  I (1234) MAIN: message
 *                                   ^^^^
 *                                   TAG value
 *
 * static → visible only in this .c file (prevents name clash)
 * const  → the string content is read-only
 */
static const char *TAG = "MAIN";

/*
 * ═══════════════════════════════════════════════════════════
 * app_main — ENTRY POINT
 * ═══════════════════════════════════════════════════════════
 * ESP-IDF calls this after boot. FreeRTOS is already running.
 * Runs on Core 0. Returns after setup — FreeRTOS takes over.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, " HUB75 STEP 1  — Single Panel 64x32");
    ESP_LOGI(TAG, " Panel : P4 64x32 ICN6124EJ");
    ESP_LOGI(TAG, " Scan  : 1:16 ABCD binary");
    ESP_LOGI(TAG, " No UART yet — display only");
    ESP_LOGI(TAG, "=========================================");

    /* ── A: Initialize framebuffer ──────────────────────────────
     * Must be called first — zeroes buf_A and buf_B,
     * sets up display_buf / back_buf pointers, creates swap mutex.
     */
    framebuffer_init();
    ESP_LOGI(TAG, "Framebuffer ready");

    /* ── B: Initialize HUB75 hardware ──────────────────────────
     * Configures SPI2 (MOSI GPIO11, CLK GPIO12 at 20MHz)
     * and GPIO outputs (LAT, OE, A, B, C, D).
     * Sets OE HIGH — display blanked until refresh task runs.
     */
    hub75_init();
    ESP_LOGI(TAG, "HUB75 hardware ready");

    /* ── C: Start display refresh task on Core 1 ────────────────
     *
     * ═══════════════════════════════════════════════════════════
     * C CONCEPT: FUNCTION POINTER
     * ═══════════════════════════════════════════════════════════
     * A function pointer is a variable that stores the MEMORY
     * ADDRESS of a function — just like a data pointer stores
     * the address of a variable.
     *
     *   int x = 5;
     *   int *p = &x;      // p = address of x
     *   *p = 10;          // use via dereference
     *
     *   void foo(void) { }
     *   void (*fp)(void) = foo;  // fp = address of foo
     *   fp();                    // call via pointer
     *
     * FreeRTOS TaskFunction_t is defined as:
     *   typedef void (*TaskFunction_t)(void *);
     *   — "pointer to function taking void*, returning void"
     *
     * hub75_refresh_task   → its ADDRESS  (no call, just a pointer)
     * hub75_refresh_task() → CALLS it now (wrong — we want later)
     *
     * ═══════════════════════════════════════════════════════════
     * C CONCEPT: CALLBACK
     * ═══════════════════════════════════════════════════════════
     * A callback = function YOU write, but SOMEONE ELSE calls.
     *
     * Registration here:
     *   xTaskCreatePinnedToCore(hub75_refresh_task, ...)
     *   → you give FreeRTOS the function address
     *   → FreeRTOS calls it later on Core 1 via stored pointer
     *   → your code never calls hub75_refresh_task() directly
     *
     * Same pattern used everywhere in embedded C:
     *   gpio_isr_register(my_handler, ...)      GPIO interrupt
     *   esp_timer_create(&cfg)   cfg.callback=fn Timer callback
     *   spi dev_cfg.pre_cb = fn                 SPI transaction
     *
     * ═══════════════════════════════════════════════════════════
     * DIRECT CALL vs CALLBACK — KEY DIFFERENCE
     * ═══════════════════════════════════════════════════════════
     *
     * Direct call (used for run_hardware_check below):
     *   run_hardware_check();
     *   → CPU jumps to function RIGHT NOW
     *   → this line blocks until the function returns
     *
     * Callback registration (used here):
     *   xTaskCreatePinnedToCore(hub75_refresh_task, ...)
     *   → CPU does NOT jump to hub75_refresh_task now
     *   → FreeRTOS stores the address
     *   → this line returns immediately
     *   → FreeRTOS calls hub75_refresh_task later on Core 1
     */
    xTaskCreatePinnedToCore(
        hub75_refresh_task,        // FUNCTION POINTER — address of task
                                   // no parentheses = address only, not a call
                                   // FreeRTOS calls this as a CALLBACK on Core 1

        "hub75_refresh",           // Name string — debug only, visible in monitor

        4096,                      // Stack bytes for this task
                                   // local vars + nested call frames live here
                                   // too small → stack overflow → crash

        NULL,                      // pvParameters → void* passed to task function
                                   // hub75_refresh_task ignores it → NULL

        configMAX_PRIORITIES - 1, // Highest FreeRTOS priority
                                   // never preempted → steady refresh rate

        NULL,                      // Task handle — not needed (no suspend/delete)

        1                          // Core 1 — pinned, never migrated
                                   // Core 0 reserved for app_main + future UART
    );
    ESP_LOGI(TAG, "hub75_refresh_task running on Core 1 (max priority)");

    /* ── D: Let the refresh task actually start ─────────────────
     * 100ms pause so FreeRTOS schedules Core 1 before we swap.
     */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* ── E: Hardware check patterns ─────────────────────────────
     * DIRECT CALL — blocks ~6 seconds while patterns cycle.
     * Defined in display_content.c, declared in display_content.h
     */
    //row_scan_test();
    //ESP_LOGI(TAG, "Hardware check done");

    run_hardware_check();
    ESP_LOGI(TAG, "Hardware check patterns complete");
    /* ── F: Static default content ──────────────────────────────
     * DIRECT CALL — draws and swaps, returns immediately.
     * This frame stays on display until reset (Step 1).
     * Step 2 onwards: UART commands will update it.
     */
    draw_default_content();
    ESP_LOGI(TAG, "Step 1 complete — display is live");

    /*
     * app_main returns here.
     * FreeRTOS keeps running. Core 1 refresh task loops forever.
     *
     * Step 2 will add before this return:
     *   xTaskCreatePinnedToCore(eth_uart_task, "uart", 4096,
     *                           NULL, 5, NULL, 0);
     */
}