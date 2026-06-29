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
// process_tap_dance.c — Tap Dance: multi-tap counting (Phase 9)
//
// Counts rapid taps of the same TD(n) key, resolving after TAPPING_TERM
// or when a different key is pressed.
//
// Data-driven: td_actions[] maps (single_tap, single_hold, double_tap, double_hold).
// User defines td_actions[] as weak symbol override.
//

#include "process_tap_dance.h"
#include "keycode.h"
#include "action_util.h"
#include "action_layer.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// Weak default: user defines tap dance actions (empty by default)
// ---------------------------------------------------------------------------

__attribute__((weak))
const td_action_t td_actions[TAP_DANCE_COUNT] = { {0} };

// ---------------------------------------------------------------------------
// Per-dance runtime state
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t    count;       // tap count (0 = idle)
    bool       pressed;     // currently held
    bool       finished;    // resolved
    uint32_t   timer;       // timestamp of last press
    uint16_t   active_kc;   // keycode currently registered (for release)
} td_entry_t;

static td_entry_t s_td[TAP_DANCE_COUNT];
static int8_t     s_td_active;   // index of TD being tracked (-1 = none)

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void tap_dance_init(void) {
    for (int i = 0; i < TAP_DANCE_COUNT; i++) {
        s_td[i].count     = 0;
        s_td[i].pressed   = false;
        s_td[i].finished  = false;
        s_td[i].timer     = 0;
        s_td[i].active_kc = KC_NO;
    }
    s_td_active = -1;
}

// ---------------------------------------------------------------------------
// Resolve — fire the appropriate keycode based on count and pressed state
// ---------------------------------------------------------------------------

static void td_resolve(uint8_t idx) {
    td_entry_t *td = &s_td[idx];
    const td_action_t *act = &td_actions[idx];

    td_state_t state = TD_NONE;
    uint16_t kc = KC_NO;

    switch (td->count) {
    case 1:
        if (td->pressed) {
            state = TD_SINGLE_HOLD;
            kc = act->single_hold;
        } else {
            state = TD_SINGLE_TAP;
            kc = act->single_tap;
        }
        break;
    case 2:
        if (td->pressed) {
            state = TD_DOUBLE_HOLD;
            kc = act->double_hold;
        } else {
            state = TD_DOUBLE_TAP;
            kc = act->double_tap;
        }
        break;
    default:  // 3+: treat as single tap (simplified)
        state = TD_TRIPLE_TAP;
        kc = act->single_tap;
        break;
    }

    td->finished = true;

    if (kc == KC_NO) {
        // Fallback: if slot not defined, try single_tap
        kc = act->single_tap;
    }
    if (kc == KC_NO) return;

    DBG_KEY("[TD] #%d resolved state=%d → kc=0x%04x\n", idx, state, kc);

    // Handle layer keys
    if (IS_LAYER_KEY(kc)) {
        uint8_t layer = LAYER_VAL(kc);
        if (IS_MO(kc)) {
            if (td->pressed) layer_on(layer);
        } else if (IS_TG(kc)) {
            layer_toggle(layer);
        } else if (IS_TO(kc)) {
            layer_set(0);
            if (layer != default_layer_get()) layer_on(layer);
        }
        td->active_kc = kc;
        return;
    }

    // Handle basic keycode
    if (IS_BASIC(kc)) {
        if (td->pressed) {
            register_code((uint8_t)kc);   // held → stays down until release
            td->active_kc = kc;
        } else {
            tap_code((uint8_t)kc);        // tap → queued press+release (separate ticks)
            td->active_kc = KC_NO;
            td->count = 0;
            s_td_active = -1;
        }
        return;
    }

    // Handle Mods+Key
    if (IS_MOD_KEY(kc)) {
        if (td->pressed) {
            add_weak_mods(mod5_to_hid(MODKEY_MOD(kc)));
            register_code(MODKEY_KC(kc));
            td->active_kc = kc;
        } else {
            tap_code_mods(MODKEY_KC(kc), mod5_to_hid(MODKEY_MOD(kc)));
            td->active_kc = KC_NO;
            td->count = 0;
            s_td_active = -1;
        }
        return;
    }

    // Other keycodes — just register
    td->active_kc = kc;
}

// ---------------------------------------------------------------------------
// Reset — cleanup after TD key fully released
// ---------------------------------------------------------------------------

static void td_reset(uint8_t idx) {
    td_entry_t *td = &s_td[idx];

    if (td->active_kc != KC_NO) {
        uint16_t kc = td->active_kc;
        if (IS_LAYER_KEY(kc) && IS_MO(kc)) {
            layer_off(LAYER_VAL(kc));
        } else if (IS_BASIC(kc)) {
            unregister_code((uint8_t)kc);
        } else if (IS_MOD_KEY(kc)) {
            unregister_code(MODKEY_KC(kc));
            del_weak_mods(mod5_to_hid(MODKEY_MOD(kc)));   // only our bits
        }
    }

    td->count     = 0;
    td->pressed   = false;
    td->finished  = false;
    td->timer     = 0;
    td->active_kc = KC_NO;

    if (s_td_active == (int8_t)idx) {
        s_td_active = -1;
    }
}

// ---------------------------------------------------------------------------
// Process record
// ---------------------------------------------------------------------------

bool process_tap_dance(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // --- Non-TD key press while a TD is pending → resolve immediately ---
    if (!IS_TAP_DANCE(kc) && event->pressed && s_td_active >= 0) {
        td_entry_t *td = &s_td[s_td_active];
        if (!td->finished) {
            td_resolve((uint8_t)s_td_active);
        }
        return true;  // let the interrupting key proceed normally
    }

    if (!IS_TAP_DANCE(kc)) return true;  // not ours

    uint8_t idx = TD_INDEX(kc);
    if (idx >= TAP_DANCE_COUNT) return true;  // out of range

    td_entry_t *td = &s_td[idx];

    if (event->pressed) {
        // If a different TD is active, resolve it first
        if (s_td_active >= 0 && s_td_active != (int8_t)idx) {
            td_resolve((uint8_t)s_td_active);
        }

        td->count++;
        td->pressed = true;
        td->timer   = event->timestamp;
        s_td_active = (int8_t)idx;

        DBG_KEY("[TD] #%d press count=%d\n", idx, td->count);
    } else {
        td->pressed = false;

        // If already resolved (hold resolved by timeout), do cleanup
        if (td->finished) {
            td_reset(idx);
            DBG_KEY("[TD] #%d release (was hold) → reset\n", idx);
        } else {
            // Released before timeout — wait for next tap or timeout
            td->timer = event->timestamp;
            DBG_KEY("[TD] #%d release count=%d (pending)\n", idx, td->count);
        }
    }

    return false;  // consumed
}

// ---------------------------------------------------------------------------
// Tick — check TAPPING_TERM timeout
// ---------------------------------------------------------------------------

void tap_dance_tick(uint32_t now_ticks) {
    if (s_td_active < 0) return;

    td_entry_t *td = &s_td[s_td_active];
    if (td->finished) return;
    if (td->count == 0) return;

    TickType_t elapsed = (TickType_t)(now_ticks - td->timer);
    if (elapsed < pdMS_TO_TICKS(TAPPING_TERM)) return;

    // Timeout → resolve
    td_resolve((uint8_t)s_td_active);
}
