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
// action_tapping.c — Tap/hold state machine for LT() and MT() (Phase 7)
//
// State machine:
//   IDLE → (MT/LT key pressed) → PRESSED
//     PRESSED → (same key released before TAPPING_TERM) → TAP → replay buffer → IDLE
//     PRESSED → (TAPPING_TERM expires) → HOLD → replay buffer → IDLE
//
// While in PRESSED state, other key events are buffered and replayed
// after resolution (so they get the correct layer/modifier context).
//

#include "action_tapping.h"
#include "keycode.h"
#include "action.h"        // action_pressed_cache_update (R2)
#include "action_layer.h"
#include "action_util.h"
#include "process_record.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdio.h>

// Forward declaration — action_exec_inner is in action.c
extern void action_exec_inner(const keyevent_t *event);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

typedef enum {
    TAP_IDLE,
    TAP_PRESSED,    // waiting for tap/hold resolution
} tap_state_t;

static tap_state_t s_state;
static keyevent_t  s_tapping_key;     // the MT/LT key being evaluated
static keyevent_t  s_buffer[TAPPING_BUFFER_SIZE];
static uint8_t     s_buffer_count;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void action_tapping_init(void) {
    s_state = TAP_IDLE;
    s_buffer_count = 0;
    memset(&s_tapping_key, 0, sizeof(s_tapping_key));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline bool is_tapping_key(uint16_t kc) {
    return IS_MOD_TAP(kc) || IS_LAYER_TAP(kc);
}

// Replay the buffered events after a tap/hold resolution (R2 rework).
//   - State goes IDLE *before* the replay and the buffer is moved to a local
//     copy: a buffered MT/LT press re-enters action_tapping_process and starts
//     its own tracking session (before R2 it fell into the "unrecognized —
//     ignore" branch of exec_inner: its press was lost and its later release
//     did spurious unregister_mods/layer_off, wiping REAL held modifiers).
//   - relookup=true (hold path): buffered PRESSES are re-resolved on the new
//     layer; their keycode is propagated to the pressed-key cache AND to any
//     later buffered RELEASE of the same position. Buffered releases are never
//     re-resolved (they carry the press identity).
//   - A report is flushed after EVERY replayed event: a DN+UP of the same key
//     replayed in one tick otherwise nets to nothing (the lost-tap bug class).
static void replay_buffer(bool relookup) {
    keyevent_t local[TAPPING_BUFFER_SIZE];
    uint8_t    n = s_buffer_count;
    memcpy(local, s_buffer, sizeof(keyevent_t) * n);
    s_buffer_count = 0;
    s_state = TAP_IDLE;

    for (uint8_t i = 0; i < n; i++) {
        if (relookup && local[i].pressed) {
            uint16_t nk = layer_lookup(local[i].row, local[i].col);
            if (nk != local[i].keycode) {
                local[i].keycode = nk;
                action_pressed_cache_update(local[i].row, local[i].col, nk);
                for (uint8_t j = (uint8_t)(i + 1); j < n; j++) {
                    if (!local[j].pressed &&
                        local[j].row == local[i].row && local[j].col == local[i].col) {
                        local[j].keycode = nk;
                    }
                }
            }
        }
        if (!action_tapping_process(&local[i])) {
            action_exec_inner(&local[i]);
        }
        send_keyboard_report();
    }
}

static void resolve_as_hold(void) {
    uint16_t kc = s_tapping_key.keycode;

    if (IS_MOD_TAP(kc)) {
        uint8_t mods = mod5_to_hid(MT_MOD(kc));
        register_mods(mods);
        DBG_KEY("[TAP] MT hold → mods=0x%02x\n", mods);
    } else if (IS_LAYER_TAP(kc)) {
        uint8_t layer = LT_LAYER(kc);
        layer_on(layer);
        DBG_KEY("[TAP] LT hold → layer %d ON\n", layer);
    }

    // Replay buffered events with the hold context active (presses re-resolved)
    replay_buffer(true);
}

static void resolve_as_tap(void) {
    uint16_t kc = s_tapping_key.keycode;
    uint8_t basic_kc = 0;

    if (IS_MOD_TAP(kc)) {
        basic_kc = MT_KC(kc);
    } else if (IS_LAYER_TAP(kc)) {
        basic_kc = LT_KC(kc);
    }

    // Emit the tap via the non-blocking queue: press and release land on separate
    // keyscan ticks so the report is actually sent (a same-tick press+release nets
    // to nothing) and a discrete down→up reaches both USB and the last-wins BLE slot.
    if (basic_kc) {
        tap_code(basic_kc);
        DBG_KEY("[TAP] tap → kc=0x%02x queued\n", basic_kc);
    }

    // Replay buffered events (no layer/mod context — tap resolved, no relookup)
    replay_buffer(false);
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool action_tapping_process(const keyevent_t *event) {
    switch (s_state) {
    case TAP_IDLE:
        if (event->pressed && is_tapping_key(event->keycode)) {
            // Start tracking this key
            s_tapping_key = *event;
            s_buffer_count = 0;
            s_state = TAP_PRESSED;
            DBG_KEY("[TAP] start tracking kc=0x%04x\n", event->keycode);
            return true;  // consumed
        }
        return false;  // not a tapping key — handle normally

    case TAP_PRESSED:
        // Check if this is the release of the tapping key
        if (!event->pressed &&
            event->row == s_tapping_key.row &&
            event->col == s_tapping_key.col) {
            // Released before timeout → TAP
            resolve_as_tap();
            return true;
        }

        // Buffer other events. NEVER consume an event we can't store: a dropped
        // RELEASE is a guaranteed stuck key. On overflow resolve as HOLD now
        // (the user is clearly rolling, the tapping key has been down a while)
        // and re-process this event in the fresh IDLE state.
        if (s_buffer_count >= TAPPING_BUFFER_SIZE) {
            DBG_KEY("[TAP] buffer full → force HOLD\n");
            resolve_as_hold();
            return action_tapping_process(event);
        }
        s_buffer[s_buffer_count++] = *event;
        return true;  // consumed into buffer
    }

    return false;
}

// ---------------------------------------------------------------------------
// Tick — timeout check
// ---------------------------------------------------------------------------

void action_tapping_tick(uint32_t now_ticks) {
    if (s_state != TAP_PRESSED) return;

    TickType_t elapsed = (TickType_t)(now_ticks - s_tapping_key.timestamp);
    if (elapsed >= pdMS_TO_TICKS(TAPPING_TERM)) {
        // Timeout → HOLD
        DBG_KEY("[TAP] timeout → HOLD\n");
        resolve_as_hold();
    }
}
