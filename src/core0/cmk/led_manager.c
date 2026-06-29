/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 *
 * CMK is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation, version 2.
 *
 * CMK is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program. If not, see
 * <https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

//
// led_manager.c — Core 0 FreeRTOS task: GP2 power LED + GP3 BLE status LED
//
// GP2 — Power LED: always ON as soon as led_manager_init() is called.
//
// GP3 — BLE status LED:
//
//   State                               GP3 behaviour
//   ─────────────────────────────────── ──────────────────────────────────────
//   Downstream connected                Solid ON
//   Directed ADV (targeting saved addr) Slow blink  500 ms ON / 500 ms OFF
//   General ADV OR upstream scanning    Fast blink  200 ms ON / 200 ms OFF
//   Anything else (idle, reconnecting)  OFF
//
// The task polls connection_manager state every LED_PERIOD_MS and drives GP3
// accordingly. No IPC needed — the accessors are cheap volatile reads.
//

#include "led_manager.h"
#include "connection_manager.h"
#include "config/cmk_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "hardware/gpio.h"
#include "config/debug_config.h"
#include "diag.h"

#include <stdio.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#define LED_TASK_STACK     256
#define LED_TASK_PRIORITY  1       // lowest — pure cosmetic, never preempts real work
#define LED_PERIOD_MS      50      // poll / blink-tick period

// R5: shared half-periods (cmk_config.h), phase-locked to the absolute tick so
// GP3 and the FN dashboard blink in unison (they used different periods AND a
// mode-change-relative phase before — visibly inconsistent).
#define LED_SLOW_HALF_MS   CMK_BLINK_SLOW_HALF_MS
#define LED_FAST_HALF_MS   CMK_BLINK_FAST_HALF_MS

// ---------------------------------------------------------------------------
// Internal blink mode
// ---------------------------------------------------------------------------

typedef enum {
    MODE_OFF,
    MODE_SLOW,    // 500 ms on / 500 ms off — directed ADV (targeting a known host)
    MODE_FAST,    // 200 ms on / 200 ms off — general ADV or upstream scanning
    MODE_SOLID,   // solid ON — downstream connected
} led_mode_t;

static led_mode_t compute_mode(void) {
    cm_upstream_state_t up = connection_manager_upstream_state();
    cm_ds_ble_state_t   ds = connection_manager_ds_ble_state();

    if (ds == CM_DS_BLE_CONNECTED)      return MODE_SOLID;
    if (ds == CM_DS_BLE_ADV_DIRECTED)   return MODE_SLOW;
    if (ds == CM_DS_BLE_ADVERTISING ||
        up == CM_UPSTREAM_SCANNING)     return MODE_FAST;
    return MODE_OFF;
}

// ---------------------------------------------------------------------------
// LED task
// ---------------------------------------------------------------------------

static void led_task(void *arg) {
    (void)arg;

    bool s_hwm_logged = false;

    for (;;) {
        if (!s_hwm_logged) {
            s_hwm_logged = true;
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
            DBG_STATUS("[LED] Stack high-watermark: %lu words free (stack=%u words)\n",
                       (unsigned long)hwm, LED_TASK_STACK);
        }
        led_mode_t mode = compute_mode();
        TickType_t now  = xTaskGetTickCount();

        // R5: phase derived from the ABSOLUTE tick (same formula as fn_overlay's
        // blink_fast/slow) — no per-mode-change phase reset, all indicators in sync.
        bool led_on;
        switch (mode) {
        case MODE_SOLID: led_on = true;  break;
        case MODE_SLOW:  led_on = ((now / LED_SLOW_HALF_MS) & 1u) != 0u; break;
        case MODE_FAST:  led_on = ((now / LED_FAST_HALF_MS) & 1u) != 0u; break;
        case MODE_OFF:
        default:         led_on = false; break;
        }
#ifdef PIN_LED_BLE
        gpio_put(PIN_LED_BLE, led_on ? 1 : 0);
#else
        (void)led_on;
#endif

        DIAG_TASK_ALIVE(DIAG_TASK_LED);
        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void led_manager_init(void) {
#ifdef PIN_LED_POWER
    // Power LED — always ON
    gpio_init(PIN_LED_POWER);
    gpio_set_dir(PIN_LED_POWER, GPIO_OUT);
    gpio_put(PIN_LED_POWER, 1);
#endif

#ifdef PIN_LED_BLE
    // BLE status LED — start OFF, driven by the task
    gpio_init(PIN_LED_BLE);
    gpio_set_dir(PIN_LED_BLE, GPIO_OUT);
    gpio_put(PIN_LED_BLE, 0);
#endif
}

void led_manager_create_task(void) {
    xTaskCreate(led_task, "led_mgr", LED_TASK_STACK, NULL, LED_TASK_PRIORITY, NULL);
}
