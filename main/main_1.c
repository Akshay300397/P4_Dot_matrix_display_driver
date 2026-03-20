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
#include "uart_test2.c"          // eth_uart_init(), uart_parse_frame()
#include "display_uart.h"

#define WIZ_STATUS_PIN      GPIO_NUM_7      // WIZ550S2E STATUS2 pin
                                            // LOW  = TCP connected
                                            // HIGH = TCP disconnected
 
#define FAULT_LED_PIN       GPIO_NUM_15     // Fault indicator LED
                                            // HIGH = fault (disconnected)

static const char *TAG = "MAIN";
static const char *TAG2 = "Connection_Status";

// ─────────────────────────────────────────────────────────────
//  STATUS GPIO INIT
//  WIZ_STATUS_PIN : input, pull-up (active LOW — WIZ drives it LOW
//                   when connected, floats/HIGH when disconnected)
//  FAULT_LED_PIN  : output (HIGH = LED on = fault)
// ─────────────────────────────────────────────────────────────
 
void wiz_status_gpio_init(void)
{
    gpio_config_t status_cfg = {
        .pin_bit_mask = (1ULL << WIZ_STATUS_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,     // HIGH when WIZ unpowered
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&status_cfg));
 
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << FAULT_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_cfg));
    gpio_set_level(FAULT_LED_PIN, 0);   // LED off at startup
 
    ESP_LOGI(TAG2, "GPIO ready: STATUS2=GPIO%d FAULT_LED=GPIO%d",
             WIZ_STATUS_PIN, FAULT_LED_PIN);
}
 
// ─────────────────────────────────────────────────────────────
//  WIZ CONNECTION STATUS TASK
//
//  Polls WIZ_STATUS_PIN every 500ms.
//  Only acts on state CHANGE to avoid redrawing every poll.
//
//  Connected   (LOW)  → LED off, clear "NO NET", wait for fresh data
//  Disconnected (HIGH) → LED on, show "NO NET" on display
// ─────────────────────────────────────────────────────────────
 
void wiz_status_task(void *pvParameters)
{
    ESP_LOGI(TAG2, "Status monitor started on core %d", xPortGetCoreID());
 
    // Read initial state so we don't trigger a false change at boot
    bool prev_connected = (gpio_get_level(WIZ_STATUS_PIN) == 0);
 
    // Apply initial state at startup
    if (!prev_connected) {
        gpio_set_level(FAULT_LED_PIN, 1);
        //framebuffer_clear_back();
        //framebuffer_draw_string(20,  6, "NO NET", COLOR_RED);
        //framebuffer_draw_string(16, 19, "CHECK",  COLOR_YELLOW);
        //framebuffer_swap();
        ESP_LOGW(TAG2, "Boot state: DISCONNECTED");
        uart_send("CHECK_CONNECTION..\r\n");
    } else {
        gpio_set_level(FAULT_LED_PIN, 0);
        ESP_LOGI(TAG2, "Boot state: CONNECTED");
    }
 
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
 
        bool connected = (gpio_get_level(WIZ_STATUS_PIN) == 0);
 
        if (connected == prev_connected) continue;  // no change — skip
 
        if (connected) {
            // ── Reconnected ───────────────────────────────────
            ESP_LOGI(TAG2, "WIZ550S2E: TCP CONNECTED");
            gpio_set_level(FAULT_LED_PIN, 0);       // LED off
            
        } else {
            // ── Disconnected ──────────────────────────────────
            ESP_LOGW(TAG2, "WIZ550S2E: TCP DISCONNECTED");
            gpio_set_level(FAULT_LED_PIN, 1);       // LED on
            uart_send("CHECK_CONNECTION..\r\n");
        }
 
        prev_connected = connected;
    }
}

/*
 * ═══════════════════════════════════════════════════════════
 * app_main — ENTRY POINT
 * ═══════════════════════════════════════════════════════════
 * ESP-IDF calls this after boot. FreeRTOS is already running.
 * Runs on Core 0. Returns after setup — FreeRTOS takes over.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "System booted, app_main running on core %d", xPortGetCoreID());

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

    /* ── C: Start HUB75 refresh task on Core 1 ─────────────────
     * This continuously reads display_buf and sends rows to the panel.
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
    two_panel_rgb_test();
    ESP_LOGI(TAG, "Two panel RGB test done");

    //run_hardware_check();
    //ESP_LOGI(TAG, "Hardware check patterns complete");
    /* ── F: Static default content ──────────────────────────────
     * DIRECT CALL — draws and swaps, returns immediately.
     * This frame stays on display until reset (Step 1).
     * Step 2 onwards: UART commands will update it.
     */
    draw_default_content();
    ESP_LOGI(TAG, "Step 1 complete — display is live");

   eth_uart_init();

    // uart_parse_frame — highest, must never be starved
xTaskCreatePinnedToCore(uart_parse_frame, "uart",    4096, NULL,
                        configMAX_PRIORITIES - 1, NULL, 0);

// wiz_status_task — lower, preemptible, only polls GPIO
xTaskCreatePinnedToCore(wiz_status_task,  "wiz_mon", 2048, NULL,
                        configMAX_PRIORITIES - 2, NULL, 0);

    ESP_LOGI(TAG, "eth_uart_task started");
    ESP_LOGI(TAG, "System ready — waiting for UART commands");

    // app_main() returns here — FreeRTOS scheduler takes over

}