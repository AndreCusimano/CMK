/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 * Copyright (C) 2016-2026 QMK Firmware contributors
 * Copyright (C) Jun Wako and TMK Keyboard Firmware contributors
 *
 * This file contains code derived from the QMK Firmware project
 * (https://github.com/qmk/qmk_firmware) and, indirectly, from the
 * TMK Keyboard Firmware by Jun Wako, both distributed under the
 * GNU General Public License version 2.
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
// keyscan.c — Key scanner task (Phase 7), Core 0 FreeRTOS
//
// Thin scanner: matrix_scan → debounce → action_exec for each changed key.
// All keycode processing, layer lookup, tap/hold, combo, and report building
// is delegated to the action subsystem (action.h).
//
// Replaces the Phase 6 keyscan.c which had inline combo/report logic.
//

#include "keyscan.h"
#include "matrix.h"
#include "debounce.h"
#include "action.h"
#include "action_util.h"
#include "action_layer.h"        // layer_lookup() — resolved keycode for [SPLIT] logs
#include "keymap.h"               // keymap_local_col_base(), KEYMAP_COLS
#include "board_side.h"           // CMK_SIDE_RIGHT
#include "diag.h"
#include "connection_manager.h"   // split role + remote side (Phase 18)
#include "ipc/ipc_core.h"         // split remote-event ring + CMK_SPLIT_EVT
#include "low_power.h"            // idle tracking + dynamic scan rate
#include "config/cmk_config.h"
#include "config/board.h"         // PIN_I2C1_SDA (cross-half wake)
#include "config/debug_config.h"
#include "hardware/gpio.h"        // GPIO IRQ wake (DEEP sleep)
#include "hardware/watchdog.h"    // watchdog_reboot — wake from DEEP = clean reboot
#ifdef INTER_HALF_LINK_ENABLE
#include "i2c1_link.h"            // i2c1_link_wake_pulse (cross-half wake)
#endif

#if defined(RGB_MATRIX_ENABLE)
#include "rgb_matrix.h"
#elif defined(LED_MATRIX_ENABLE)
#include "led_matrix.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Scanner task
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Split runtime (Phase 18) — MASTER injects the remote half's matrix events into
// the SAME action pipeline (unified 4×12 keymap, single timeline).
// ---------------------------------------------------------------------------

// Bitmap of the SLAVE's currently-held keys (bit row*MATRIX_COLS+col), so the
// MASTER can release them on disconnect and diff against a re-sync snapshot.
static uint32_t        s_remote_pressed = 0;
static cm_split_role_t s_prev_role      = CM_SPLIT_STANDALONE;

// Per-half settings (18n): while the MASTER's own LAYER_SETTINGS is held, the
// remote half must NOT be able to click it (device config is per-half — mirror
// of the SLAVE's local FN routing). Remote PRESSES arriving with SETTINGS on are
// suppressed and tracked here (bit row*MATRIX_COLS+col, slave-physical coords),
// so the matching RELEASE is swallowed too — even if FN was released in between
// (key identity decided at press, same philosophy as R2). Keys pressed BEFORE
// FN keep the normal release path (no stuck keys). The re-sync snapshot heals
// lost releases of suppressed keys and masks them out of the XOR diff, so a
// held suppressed key is not re-injected every 500 ms.
static uint32_t        s_remote_suppressed = 0;

// SLAVE: FN + LAYER_SETTINGS are handled LOCALLY (device config runs on this half),
// everything else is shipped raw to the master. This bitmap (bit row*MATRIX_COLS+col)
// tracks keys whose PRESS was handled locally, so the matching RELEASE follows the same
// path even if FN was engaged/released between the two (no key-stuck on mixed routing).
static uint32_t        s_slave_local_keys = 0;

// Logical-column base of the REMOTE half's slice (peer side: 0 = left, 6 = right).
static inline uint8_t remote_col_base(void) {
    return (connection_manager_remote_side() == CMK_SIDE_RIGHT) ? MATRIX_COLS : 0;
}

// MASTER: inject one remote (slave) key change at its logical column. Single
// choke point for BOTH the event path and the snapshot diff — the per-half
// settings gate (18n) lives here so no path can bypass it.
static void master_apply_remote(uint8_t r, uint8_t c, bool pressed, uint32_t now) {
    if (r >= MATRIX_ROWS || c >= MATRIX_COLS) return;
    uint8_t bit = (uint8_t)(r * MATRIX_COLS + c);
    if (pressed) {
        if (layer_is_on(LAYER_SETTINGS)) {
            // Our settings layer is held → the remote half can't click it.
            s_remote_suppressed |= (1u << bit);
            DBG_SPLIT("[SPLIT-RX] remote r=%u c=%u DN soppresso (SETTINGS per-half)\n", r, c);
            return;
        }
    } else if (s_remote_suppressed & (1u << bit)) {
        // Release of a suppressed press → swallow (follows the press's path).
        s_remote_suppressed &= ~(1u << bit);
        DBG_SPLIT("[SPLIT-RX] remote r=%u c=%u UP soppresso (SETTINGS per-half)\n", r, c);
        return;
    }
    low_power_note_remote_activity(now);   // remote (slave) keys keep the MASTER awake
    uint8_t logcol = (uint8_t)(c + remote_col_base());
    DBG_SPLIT("[SPLIT-RX] remote r=%u c=%u -> logcol=%u %s kc=0x%04x\n",
              r, c, logcol, pressed ? "DN" : "UP", layer_lookup(r, logcol));
    action_exec(r, logcol, pressed, now);
#if defined(RGB_MATRIX_ENABLE)
    // Phase 24b: the slave key has no LED on this half — place a reactive hit at
    // its UNIFIED position so splash/multinexus radiate across the split.
    if (pressed) {
        cmk_side_t rs = (board_side_get() == CMK_SIDE_LEFT) ? CMK_SIDE_RIGHT : CMK_SIDE_LEFT;
        uint16_t ux, uy;
        keymap_unified_xy(rs, r, c, &ux, &uy);
        rgb_matrix_record_hit_xy(ux, uy);
    }
#endif
    if (pressed) s_remote_pressed |=  (1u << bit);
    else         s_remote_pressed &= ~(1u << bit);
}

// ---------------------------------------------------------------------------
// Low-power DEEP wake (sub-fase 23d). While in DEEP the keyscan task suspends
// itself and blocks on a notification; the core sleeps in WFI (tickless) until
// a key press pulls a sense line LOW (falling-edge GPIO IRQ) or the sibling
// pulses the I2C1 wake line (GP42). The held key is registered by the normal
// scan that runs right after the wake (R2-safe).
// ---------------------------------------------------------------------------
#ifdef LOW_POWER_ENABLE
static TaskHandle_t  s_keyscan_task;          // notified from the wake ISR
static volatile bool s_wake_local;            // last wake was a local key (not the sibling)

static void keyscan_wake_isr(uint gpio, uint32_t events) {
    (void)events;
#ifdef INTER_HALF_LINK_ENABLE
    s_wake_local = (gpio != PIN_I2C1_SDA);    // GP42 edge = sibling's wake pulse
#else
    (void)gpio;
    s_wake_local = true;
#endif
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(s_keyscan_task, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static bool keyscan_wake_arm(bool enable) {
    uint8_t        n    = 0;
    const uint8_t *pins = matrix_get_input_pins(&n);
    if (enable) {
        matrix_wake_irq_drive(true);          // drive selects active → any key → sense LOW
        // One shared GPIO callback for all pins (pico-sdk has a single bank cb).
        gpio_set_irq_enabled_with_callback(pins[0], GPIO_IRQ_EDGE_FALL, true, keyscan_wake_isr);
        for (uint8_t i = 1; i < n; i++)
            gpio_set_irq_enabled(pins[i], GPIO_IRQ_EDGE_FALL, true);
#ifdef INTER_HALF_LINK_ENABLE
        gpio_set_irq_enabled(PIN_I2C1_SDA, GPIO_IRQ_EDGE_FALL, true);   // cross-half wake
#endif
    } else {
        for (uint8_t i = 0; i < n; i++)
            gpio_set_irq_enabled(pins[i], GPIO_IRQ_EDGE_FALL, false);
#ifdef INTER_HALF_LINK_ENABLE
        gpio_set_irq_enabled(PIN_I2C1_SDA, GPIO_IRQ_EDGE_FALL, false);
#endif
        matrix_wake_irq_drive(false);         // restore normal scan idle state
    }
    // An edge before IRQ arming must not be lost: a held key is already LOW.
    if (enable) for (uint8_t i=0; i<n; ++i) if (!gpio_get(pins[i])) return true;
    return false;
}
#endif // LOW_POWER_ENABLE

static void keyscan_task(void *arg) {
    (void)arg;
    DBG_STATUS("[KS] Keyscan task started (%dx%d matrix, debounce=%dms)\n",
               MATRIX_ROWS, MATRIX_COLS, DEBOUNCE);

    // Take initial snapshot to avoid spurious events at boot
    matrix_scan();
    memcpy(matrix_get_debounced(), matrix_get_raw(),
           sizeof(matrix_row_t) * MATRIX_ROWS);

    matrix_row_t prev_matrix[MATRIX_ROWS];
    memcpy(prev_matrix, matrix_get_debounced(), sizeof(prev_matrix));

    bool hwm_logged = false;

    for (;;) {
        if (!hwm_logged) {
            hwm_logged = true;
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
            DBG_STATUS("[KS] Stack high-watermark: %lu words free (stack=%u words)\n",
                       (unsigned long)hwm, KS_TASK_STACK);
        }

        TickType_t now = xTaskGetTickCount();

        cm_split_role_t role       = connection_manager_split_role();
        uint8_t         local_base = keymap_local_col_base();

        // --- Role transition cleanup (edge-triggered) ---
        if (role != s_prev_role) {
            if (s_prev_role == CM_SPLIT_MASTER) {
                if (s_remote_pressed) {
                    // Lost the slave → release every remote-held key (no stuck keys at PC).
                    for (uint8_t b = 0; b < MATRIX_ROWS * MATRIX_COLS; b++) {
                        if (s_remote_pressed & (1u << b)) {
                            action_exec((uint8_t)(b / MATRIX_COLS),
                                        (uint8_t)((b % MATRIX_COLS) + remote_col_base()),
                                        false, (uint32_t)now);
                        }
                    }
                    s_remote_pressed = 0;
                }
                // Drain-and-discard any in-flight remote events/snapshot: with the
                // hot-switch the link stays up, so stale events must not survive a
                // later re-engage (they would re-press keys from the old session).
                uint8_t  stale_evt;
                uint32_t stale_snap;
                while (ipc_split_pop_remote_event(&stale_evt)) { }
                (void)ipc_split_take_remote_snapshot(&stale_snap);
                s_remote_suppressed = 0;   // 18n: suppressed-keys map dies with the role
            }
            if (s_prev_role == CM_SPLIT_SLAVE) {
                // Release any locally-held FN/settings keys first (turns LAYER_SETTINGS
                // back off via the FN release), then clear the HID report.
                if (s_slave_local_keys) {
                    for (uint8_t b = 0; b < MATRIX_ROWS * MATRIX_COLS; b++) {
                        if (s_slave_local_keys & (1u << b)) {
                            action_exec((uint8_t)(b / MATRIX_COLS),
                                        (uint8_t)((b % MATRIX_COLS) + local_base),
                                        false, (uint32_t)now);
                        }
                    }
                    s_slave_local_keys = 0;
                }
                clear_keyboard();   // back to local typing → clean slate
            }
            if (role == CM_SPLIT_SLAVE) {
                // Entering SLAVE: any key held across the transition was pressed
                // LOCALLY (in STANDALONE) — its release must follow the local path
                // too. Seeding the routing map from the current debounced matrix
                // keeps the "release follows the press" invariant across the role
                // change; with an empty map the release was shipped raw to the
                // master and the local release never ran (an FN held through the
                // engage left LAYER_SETTINGS stuck on until FN was re-pressed).
                // The R2 press-cache resolves each release to the keycode of its
                // own local press, whatever layer was active back then.
                s_slave_local_keys = 0;
                for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
                    matrix_row_t row = matrix_get_debounced()[r];
                    for (uint8_t c = 0; c < MATRIX_COLS; c++) {
                        if ((row >> c) & 1u) {
                            s_slave_local_keys |= (1u << (r * MATRIX_COLS + c));
                        }
                    }
                }
            }
            s_prev_role = role;
        }

        // 1. Scan matrix hardware
        bool raw_changed = matrix_scan();

        // 2. Debounce
        bool debounced_changed = debounce(
            matrix_get_raw(), matrix_get_debounced(),
            MATRIX_ROWS, raw_changed
        );

        // 3a. MASTER: inject the remote half's events FIRST (snapshot, then deltas),
        //     so local and remote keys share one timeline within the tick.
        if (role == CM_SPLIT_MASTER) {
            uint32_t snap;
            if (ipc_split_take_remote_snapshot(&snap)) {
                // 18n: heal suppressed keys whose RELEASE got lost (no longer held
                // on the slave → nothing to apply, the press was never injected),
                // then mask the still-held ones out of the diff so they are not
                // re-injected as presses every resync period.
                s_remote_suppressed &= snap;
                uint32_t changed = (snap ^ s_remote_pressed) & ~s_remote_suppressed;
                for (uint8_t b = 0; b < MATRIX_ROWS * MATRIX_COLS; b++) {
                    if (changed & (1u << b)) {
                        master_apply_remote((uint8_t)(b / MATRIX_COLS),
                                            (uint8_t)(b % MATRIX_COLS),
                                            ((snap >> b) & 1u) != 0u, (uint32_t)now);
                    }
                }
            }
            uint8_t evt;
            while (ipc_split_pop_remote_event(&evt)) {
                master_apply_remote(CMK_SPLIT_EVT_ROW(evt), CMK_SPLIT_EVT_COL(evt),
                                    CMK_SPLIT_EVT_PRESSED(evt), (uint32_t)now);
                // Emit a report PER remote event. Multiple events drained in one tick
                // (a press+release of the same key arriving together) would otherwise
                // collapse to nothing — only one report is built at end-of-tick. The
                // kbd FIFO carries each distinct state out at the BLE interval.
                send_keyboard_report();
            }
        }

        // 3b. Local matrix changes
        if (debounced_changed) {
            low_power_note_activity((uint32_t)now);   // local keys keep this half awake
            for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
                matrix_row_t diff = prev_matrix[r] ^ matrix_get_debounced()[r];
                if (diff == 0) continue;

                matrix_row_t curr_row = matrix_get_debounced()[r];
                for (uint8_t c = 0; c < MATRIX_COLS; c++) {
                    if (!((diff >> c) & 1u)) continue;

                    bool pressed = (curr_row >> c) & 1u;
                    if (role == CM_SPLIT_SLAVE) {
                        // SLAVE: keep local reactive LED responsive regardless of routing.
#if defined(RGB_MATRIX_ENABLE)
                        if (pressed) rgb_matrix_record_hit(r, c);
#elif defined(LED_MATRIX_ENABLE)
                        if (pressed) led_matrix_record_hit(r, c);
#endif
                        // FN + LAYER_SETTINGS are handled LOCALLY (device config runs on
                        // this half); everything else is shipped raw to the master. The
                        // release follows the press's path via s_slave_local_keys.
                        uint8_t  lc  = (uint8_t)(c + local_base);
                        uint8_t  bit = (uint8_t)(r * MATRIX_COLS + c);
                        if (pressed) {
                            uint16_t kc    = layer_lookup(r, lc);  // base, or base+SETTINGS
                            bool     local = layer_is_on(LAYER_SETTINGS) || (kc == KC_FN);
                            if (local) {
                                s_slave_local_keys |= (1u << bit);
                                DBG_SPLIT("[SPLIT] slave-local DN r=%u c=%u kc=0x%04x\n", r, c, kc);
                                action_exec(r, lc, true, (uint32_t)now);
                            } else {
                                DBG_SPLIT("[SPLIT-TX] local r=%u c=%u DN -> master\n", r, c);
                                ipc_push_to_core1(IPC_ENCODE(IPC_SPLIT_TX_EVENT,
                                                             CMK_SPLIT_EVT(true, r, c)));
                            }
                        } else {
                            if (s_slave_local_keys & (1u << bit)) {
                                s_slave_local_keys &= ~(1u << bit);
                                DBG_SPLIT("[SPLIT] slave-local UP r=%u c=%u\n", r, c);
                                action_exec(r, lc, false, (uint32_t)now);
                            } else {
                                DBG_SPLIT("[SPLIT-TX] local r=%u c=%u UP -> master\n", r, c);
                                ipc_push_to_core1(IPC_ENCODE(IPC_SPLIT_TX_EVENT,
                                                             CMK_SPLIT_EVT(false, r, c)));
                            }
                        }
                    } else {
                        // STANDALONE / MASTER local keys → logical column of this slice.
                        uint8_t lc = (uint8_t)(c + local_base);
#if defined(RGB_MATRIX_ENABLE)
                        // Bug #11 — per-half config keys carry NO keypress to the other
                        // half. Mirror of the two existing gates: the SLAVE keeps FN and
                        // every key pressed with LAYER_SETTINGS on for itself (see below),
                        // and master_apply_remote() suppresses remote presses while our
                        // own SETTINGS layer is held (18n). Without this the MASTER was
                        // the only half shipping FN across, so FN splashed on the sibling
                        // in one direction only. Resolved BEFORE action_exec: pressing FN
                        // turns the layer on, so the post-exec test would be too late
                        // (key identity decided at press — same philosophy as R2).
                        bool cfg_key = pressed && (layer_is_on(LAYER_SETTINGS) ||
                                                   layer_lookup(r, lc) == KC_FN);
#endif
                        action_exec(r, lc, pressed, (uint32_t)now);
#if defined(RGB_MATRIX_ENABLE)
                        // Phase 24b: as MASTER, forward this local hit to the slave
                        // so its reactive effect mirrors our keypress (Core 0 ring →
                        // CM → IPC; never an IPC push straight from the keyscan task).
                        if (role == CM_SPLIT_MASTER && pressed && !cfg_key) {
                            ipc_rgb_hit_tx_push(CMK_RGB_HIT(board_side_get(), r, c));
                        }
#endif
                    }
                }
            }
            memcpy(prev_matrix, matrix_get_debounced(), sizeof(prev_matrix));
        }

#if defined(RGB_MATRIX_ENABLE)
        // Phase 24b — SLAVE: apply reactive hits forwarded by the master before the
        // RGB tick runs, so a master keypress splashes onto this half the same frame.
        if (role == CM_SPLIT_SLAVE) {
            uint8_t ph;
            while (ipc_rgb_hit_rx_pop(&ph)) {
                uint16_t ux, uy;
                keymap_unified_xy((cmk_side_t)CMK_RGB_HIT_SIDE(ph),
                                  CMK_RGB_HIT_ROW(ph), CMK_RGB_HIT_COL(ph), &ux, &uy);
                rgb_matrix_record_hit_xy(ux, uy);
            }
        }
#endif

        bool any_held=false;
        for (uint8_t row=0; row<MATRIX_ROWS; ++row)
            any_held |= matrix_get_debounced()[row] != 0;
        low_power_update_held(any_held);

        // 4. Tick timers (tapping, RGB, etc.) — runs in every role.
        action_tick((uint32_t)now);

        // 5. Send HID report (not as a SLAVE — the master owns the unified report).
        if (role != CM_SPLIT_SLAVE) {
            send_keyboard_report();
        }

        DIAG_TASK_ALIVE(DIAG_TASK_KS);

#ifdef LOW_POWER_ENABLE
        if (low_power_reboot_needed()) {
            watchdog_reboot(0,0,0);
            for (;;) { __asm volatile("wfi"); }
        }
        if (low_power_state() == LP_DEEP) {
            // Suspend the scan and sleep until a key edge. With tickless idle and
            // every other task blocked/throttled, the core stays in WFI here.
            s_wake_local=false;
            if (keyscan_wake_arm(true)) s_wake_local=true;
            else ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            keyscan_wake_arm(false);
#ifdef INTER_HALF_LINK_ENABLE
            // Local key → pulse the sibling so it wakes (reboots) too. A wake from
            // the sibling's own GP42 edge must NOT pulse back (no reboot ping-pong).
            if (s_wake_local) i2c1_link_wake_pulse();
#endif
            // Wake from DEEP = full reboot. The CYW43 was powered down on DEEP
            // entry and cannot be resumed reliably in-place on this transport; a
            // clean reset re-runs the boot path (cyw43 + BTstack + CM reconnect
            // from the bonding store). The waking keypress is lost — acceptable
            // after ≥5 min idle (like a BT keyboard waking on the first touch).
            watchdog_reboot(0, 0, 0);
            for (;;) { __asm volatile("wfi"); }   // wait for the reset
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(low_power_scan_interval_ms()));
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void keyscan_init(void) {
    matrix_init();
    debounce_init();
    // action_init() is intentionally NOT called here: rgb_matrix_init()
    // (inside action_init) reads eeconfig, which main() loads after
    // matrix bring-up. main.c calls action_init() explicitly at the
    // right point, between eeconfig_init() and bootmagic_execute() /
    // task creation.
}

void keyscan_create_task(void) {
#ifdef LOW_POWER_ENABLE
    xTaskCreate(keyscan_task, "keyscan", KS_TASK_STACK, NULL, KS_TASK_PRIORITY, &s_keyscan_task);
#else
    xTaskCreate(keyscan_task, "keyscan", KS_TASK_STACK, NULL, KS_TASK_PRIORITY, NULL);
#endif
}
