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
// process_auto_shift.c — Auto Shift: hold brief → auto-shift (Phase 10)
//
// When enabled, holding any eligible basic key longer than AUTO_SHIFT_TIMEOUT
// sends the shifted version. Quick tap sends the unshifted version.
//
// Has its own event buffer (separate from action_tapping) since it applies
// to basic keycodes, not MT/LT-encoded keycodes.
//

#include "process_auto_shift.h"
#include "keycode.h"
#include "action_util.h"
#include "eeconfig.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// Forward declaration — replay goes through the normal pipeline
extern void action_exec_inner(const keyevent_t *event);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool     s_as_enabled;
static bool     s_as_active;       // tracking a key
static uint8_t  s_as_kc;           // basic keycode being tracked
static uint8_t  s_as_row, s_as_col;
static uint32_t s_as_press_time;
static bool     s_as_resolved;     // timeout fired → shift applied

static keyevent_t s_as_buffer[AS_BUFFER_SIZE];
static uint8_t    s_as_buf_count;

// ---------------------------------------------------------------------------
// Weak default: eligible key check
// ---------------------------------------------------------------------------

__attribute__((weak))
bool auto_shift_is_eligible(uint16_t kc) {
    // Letters, numbers, and symbols — not modifiers (0xE0+) or KC_NO/KC_TRNS
    return IS_BASIC(kc) && kc >= KC_A && kc < 0xE0;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void auto_shift_init(void) {
    s_as_enabled   = eeconfig_read_auto_shift() != 0;  // restore from eeconfig
    s_as_active    = false;
    s_as_buf_count = 0;
    s_as_resolved  = false;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void as_replay_buffer(void) {
    for (uint8_t i = 0; i < s_as_buf_count; i++) {
        action_exec_inner(&s_as_buffer[i]);
    }
    s_as_buf_count = 0;
}

static void as_resolve_tap(void) {
    // Quick release → unshifted tap. Queue it (press+release on separate ticks) so
    // the report is actually emitted; a same-tick register+unregister nets to nothing.
    tap_code(s_as_kc);
    DBG_KEY("[AS] tap → kc=0x%02x queued\n", s_as_kc);

    s_as_active = false;
    as_replay_buffer();
}

static void as_resolve_hold(void) {
    // Timeout → send shifted key (stays pressed until release)
    add_weak_mods(MOD_BIT(KC_LSFT));
    register_code(s_as_kc);
    s_as_resolved = true;
    DBG_KEY("[AS] hold → kc=0x%02x (shifted)\n", s_as_kc);

    as_replay_buffer();
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_auto_shift(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // Toggle on/off
    if (kc == CMK_AUTO_SHIFT_TOG) {
        if (event->pressed) {
            s_as_enabled = !s_as_enabled;
            eeconfig_update_auto_shift(s_as_enabled ? 1 : 0);
            DBG_KEY("[AS] %s\n", s_as_enabled ? "enabled" : "disabled");
            // If disabling while tracking, resolve as tap
            if (!s_as_enabled && s_as_active) {
                as_resolve_tap();
            }
        }
        return false;  // consumed
    }

    // Not enabled → pass through
    if (!s_as_enabled) return true;

    // Currently tracking a key
    if (s_as_active) {
        // Release of the tracked key
        if (!event->pressed &&
            event->row == s_as_row && event->col == s_as_col) {
            if (s_as_resolved) {
                // Was already resolved as hold → just unregister
                unregister_code(s_as_kc);
                del_weak_mods(MOD_BIT(KC_LSFT));   // only our shift bit
                s_as_active   = false;
                s_as_resolved = false;
            } else {
                // Released before timeout → tap
                as_resolve_tap();
            }
            return false;  // consumed
        }

        // Any other event during tracking → buffer it. On overflow NEVER drop
        // the event (a dropped release = stuck key): resolve as hold now and
        // let this event continue through the chain normally.
        if (s_as_buf_count >= AS_BUFFER_SIZE) {
            DBG_KEY("[AS] buffer full → force hold\n");
            as_resolve_hold();
            return true;  // pass through (tracking ended)
        }
        s_as_buffer[s_as_buf_count++] = *event;
        return false;  // consumed into buffer
    }

    // Not tracking — check if this is an eligible press
    if (event->pressed && auto_shift_is_eligible(kc)) {
        s_as_active     = true;
        s_as_kc         = (uint8_t)kc;
        s_as_row        = event->row;
        s_as_col        = event->col;
        s_as_press_time = event->timestamp;
        s_as_resolved   = false;
        s_as_buf_count  = 0;
        return false;  // consumed, waiting for tap/hold
    }

    return true;  // pass through
}

// ---------------------------------------------------------------------------
// Tick — timeout check
// ---------------------------------------------------------------------------

void auto_shift_tick(uint32_t now_ticks) {
    if (!s_as_active || s_as_resolved) return;

    TickType_t elapsed = (TickType_t)(now_ticks - s_as_press_time);
    if (elapsed >= pdMS_TO_TICKS(AUTO_SHIFT_TIMEOUT)) {
        as_resolve_hold();
    }
}
