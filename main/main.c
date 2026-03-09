/*
 * main.c
 * Application entry point — ESP32-S3 HUB75 LED Matrix Display
 *
 * Hardware:
 *   MCU:     ESP32-S3
 *   Display: 2× P4 64×32 ICN6124EJ HUB75 panels chained = 128×32
 *   Comms:   Ethernet-to-Serial board on UART1 (GPIO16/17)
 *
 * Task layout:
 *   Core 0: eth_uart_task   — receives display commands via UART
 *   Core 1: hub75_refresh_task — continuously refreshes LED panels
 *
 * On boot:
 *   1. Draws a startup pattern to confirm display is working
 *   
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "hub75.h"
#include "framebuffer.h"
#include "eth_uart.h"

static const char *TAG = "MAIN";

// ─────────────────────────────────────────────────────────────
//  STARTUP PATTERN
//  Draws a test image to confirm hardware is working before
//  the UART listener starts. Mirrors what your PxMatrix test did.
// ─────────────────────────────────────────────────────────────

static void draw_startup_pattern(void)
{
    ESP_LOGI(TAG, "Running hardware check patterns...");

    // ── Test 1: Color bars ─────────────────────────────────────
    framebuffer_clear_back();
    framebuffer_fill_rect(0,   0, 64, 32, COLOR_RED);
    framebuffer_fill_rect(64,  0, 64, 32, COLOR_BLUE);
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ── Test 2: Horizontal bands ───────────────────────────────
    framebuffer_clear_back();
    framebuffer_fill_rect(0, 0,  128, 16, COLOR_GREEN);
    framebuffer_fill_rect(0, 16, 128, 16, COLOR_YELLOW);
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ── Test 3: All 7 colors ───────────────────────────────────
    framebuffer_clear_back();
    framebuffer_fill_rect(0,   0, 16, 32, COLOR_RED);
    framebuffer_fill_rect(16,  0, 16, 32, COLOR_GREEN);
    framebuffer_fill_rect(32,  0, 16, 32, COLOR_BLUE);
    framebuffer_fill_rect(48,  0, 16, 32, COLOR_YELLOW);
    framebuffer_fill_rect(64,  0, 16, 32, COLOR_CYAN);
    framebuffer_fill_rect(80,  0, 16, 32, COLOR_MAGENTA);
    framebuffer_fill_rect(96,  0, 16, 32, COLOR_WHITE);
    framebuffer_fill_rect(112, 0, 16, 32, COLOR_BLACK);
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ── Test 4: Corner pixels ──────────────────────────────────
    framebuffer_clear_back();
    framebuffer_set_pixel(0,   0,  COLOR_RED);
    framebuffer_set_pixel(127, 0,  COLOR_GREEN);
    framebuffer_set_pixel(0,   31, COLOR_BLUE);
    framebuffer_set_pixel(127, 31, COLOR_WHITE);
    framebuffer_set_pixel(63,  15, COLOR_YELLOW);
    framebuffer_set_pixel(64,  15, COLOR_CYAN);
    framebuffer_swap();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Hardware check complete");
}

// ─────────────────────────────────────────────────────────────
//  FUNCTION 2: Default static content
//  Displayed after hardware check — stays on screen
//  until UART sends a new command to update it
//  *** EDIT THIS FUNCTION to show your desired default content ***
// ─────────────────────────────────────────────────────────────
static void draw_default_content(void)
{
    framebuffer_clear_back();

    // ── Put your default static content here ──────────────────
    // Example: white border + ready message
    framebuffer_draw_rect(0, 0, 128, 32, COLOR_WHITE);
    framebuffer_draw_string(2,  2,  "SYSTEM READY", COLOR_GREEN);
    framebuffer_draw_string(2,  12, "WAITING...",   COLOR_CYAN);
    framebuffer_draw_string(2,  22, "115200 BAUD",  COLOR_YELLOW);
    // ──────────────────────────────────────────────────────────

    framebuffer_swap();
    ESP_LOGI(TAG, "Default content displayed");
}

// ─────────────────────────────────────────────────────────────
//  APP MAIN
// ─────────────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " HUB75 LED Matrix — ESP32-S3 / ESP-IDF");
    ESP_LOGI(TAG, " Panel: 2x P4 64x32 = 128x32 total");
    ESP_LOGI(TAG, " Driver IC: ICN6124EJ");
    ESP_LOGI(TAG, " Scan: 1:16 (ABCD binary)");
    ESP_LOGI(TAG, "========================================");

    // ── Step 1: Initialize framebuffer ────────────────────────
    // Must happen before any drawing or task creation
    framebuffer_init();

    // ── Step 2: Initialize HUB75 hardware ─────────────────────
    // Configures SPI2 and GPIO pins
    hub75_init();

    // ── Step 3: Start display refresh task on Core 1 ──────────
    // This task runs at maximum priority — never blocked
    // It reads display_buf continuously, panel refresh starts immediately
    xTaskCreatePinnedToCore(
        hub75_refresh_task,             // Task function
        "hub75_refresh",                // Task name (for debug)
        4096,                           // Stack size in bytes
        NULL,                           // Parameters
        configMAX_PRIORITIES - 1,       // Highest priority
        NULL,                           // Task handle (not needed)
        1                               // Core 1 — dedicated to display
    );
    ESP_LOGI(TAG, "hub75_refresh_task started on Core 1");

    // ── Step 4: Small delay to let refresh task start ─────────
    // Ensures display is actively refreshing before we draw on it
    vTaskDelay(pdMS_TO_TICKS(100));

    // ── Step 5: Draw startup test pattern ─────────────────────
    // Confirms hardware wiring, SPI, row addressing all work
    // If display looks correct here, proceed — if not, check wiring
    draw_startup_pattern();

    // Step 6: Draw default static content  ← stays on screen
    draw_default_content();

    // ── Step 7: Start UART receiver task on Core 0 ────────────
    // Listens for commands from ethernet board
    // Updates back_buf and triggers framebuffer_swap()
    xTaskCreatePinnedToCore(
        eth_uart_task,                  // Task function
        "eth_uart",                     // Task name
        4096,                           // Stack size in bytes
        NULL,                           // Parameters
        5,                              // Medium priority
        NULL,                           // Task handle
        0                               // Core 0 — shared with WiFi/BT if used
    );
    ESP_LOGI(TAG, "eth_uart_task started on Core 0");
    ESP_LOGI(TAG, "System ready — waiting for UART commands");

    // app_main() returns here — FreeRTOS scheduler takes over
    // Both tasks run independently on their respective cores
}