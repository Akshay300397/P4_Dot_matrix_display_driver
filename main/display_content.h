/*
 * display_content.h  —  STEP 1: Hardware Check + Static Content
 * ══════════════════════════════════════════════════════════════
 *
 * Declares two functions:
 *   run_hardware_check()   — cycling test patterns at boot
 *   draw_default_content() — static screen shown after check
 *
 * Definitions live in display_content.c
 *
 * ══════════════════════════════════════════════════════════════
 * C CONCEPT: HEADER GUARDS (#pragma once)
 * ══════════════════════════════════════════════════════════════
 * If two .c files both include this header, without protection
 * the declarations appear twice → compiler error.
 * #pragma once tells the compiler: include this file only once
 * per compilation unit, no matter how many times it is #included.
 */
#pragma once

/**
 * @brief  Run visual hardware check patterns at boot.
 *
 *         Cycles through these patterns (1 second each):
 *           1. Red left half, Blue right half  (vertical split)
 *           2. Green top half, Yellow bottom   (horizontal split)
 *           3. All 7 colors in vertical bands
 *           4. Single corner pixels
 *           5. Text labels
 *
 *         Call AFTER hub75_refresh_task is running.
 *         Blocks for ~6 seconds total then returns.
 *
 *         Purpose: confirm wiring, SPI, row addressing, all colors.
 */
void run_hardware_check(void);

/**
 * @brief  Draw the static default content on the display.
 *
 *         Shown after hardware check completes.
 *         Stays on screen until power-off (Step 1).
 *         In Step 2+, UART commands will update it.
 *
 *         *** EDIT draw_default_content() in display_content.c
 *             to show your own text/graphics. ***
 *
 *         Returns immediately after framebuffer_swap().
 */
void draw_default_content(void);

void chain_order_test(void);
void row_scan_test(void);