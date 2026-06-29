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
// process_combo.c — Combo: simultaneous key press detection (Phase 9)
//
// Hooks BEFORE action_tapping_process() in action_exec() to intercept
// raw matrix events. Buffers keys during COMBO_TERM window, fires combo
// if all keys match, or replays buffer through the normal pipeline.
//
// Combo definitions are a weak extern array — user overrides to define combos.
//

#include "process_combo.h"
#include "keycode.h"
#include "action.h"        // action_pressed_cache_update (R2)
#include "action_util.h"
#include "action_layer.h"
#include "action_tapping.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// Forward declaration — replay goes through the normal pipeline
extern void action_exec_inner(const keyevent_t *event);

// ---------------------------------------------------------------------------
// Weak default: user defines combos (empty by default)
// ---------------------------------------------------------------------------

__attribute__((weak))
const combo_def_t combo_defs[COMBO_COUNT] = { {{COMBO_KEY_END}, KC_NO} };

// ---------------------------------------------------------------------------
// Per-combo runtime state
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t matched;    // bitmask of which keys in combo are pressed
    bool    fired;      // combo was activated
} combo_state_t;

static combo_state_t s_combo_state[COMBO_COUNT];

// Event buffer during combo detection window
#define COMBO_BUFFER_SIZE  8
static keyevent_t s_buffer[COMBO_BUFFER_SIZE];
static uint8_t    s_buf_count;

// Timer and state
static uint32_t s_combo_timer;     // timestamp when combo detection started
static bool     s_combo_active;    // currently in combo detection window

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void combo_init(void) {
    for (int i = 0; i < COMBO_COUNT; i++) {
        s_combo_state[i].matched = 0;
        s_combo_state[i].fired   = false;
    }
    s_buf_count    = 0;
    s_combo_timer  = 0;
    s_combo_active = false;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Encode matrix position as a single byte for matching
static inline uint8_t pos_encode(uint8_t row, uint8_t col) {
    return (row << 4) | (col & 0x0F);
}

// Count number of keys in a combo definition
static uint8_t combo_key_count(const combo_def_t *def) {
    uint8_t n = 0;
    for (int i = 0; i < COMBO_MAX_KEYS; i++) {
        if (def->keys[i] == COMBO_KEY_END) break;
        n++;
    }
    return n;
}

// Check if a position is part of a combo, return the key index (0-based) or -1
static int8_t combo_key_index(const combo_def_t *def, uint8_t pos) {
    for (int i = 0; i < COMBO_MAX_KEYS; i++) {
        if (def->keys[i] == COMBO_KEY_END) break;
        if (def->keys[i] == pos) return (int8_t)i;
    }
    return -1;
}

// Check if a position is part of ANY combo (for buffering decision)
static bool pos_in_any_combo(uint8_t pos) {
    for (int i = 0; i < COMBO_COUNT; i++) {
        if (combo_defs[i].result == KC_NO) continue;
        if (combo_key_index(&combo_defs[i], pos) >= 0) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Replay buffered events through normal pipeline
// ---------------------------------------------------------------------------

static void replay_buffer(void) {
    // R2: re-resolve ONLY presses (a buffered release carries its press identity);
    // sync the pressed-key cache + later buffered releases of the same position;
    // flush a report after every event so a replayed DN+UP doesn't net to nothing.
    uint8_t n = s_buf_count;
    s_buf_count = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (s_buffer[i].pressed) {
            uint16_t nk = layer_lookup(s_buffer[i].row, s_buffer[i].col);
            if (nk != s_buffer[i].keycode) {
                s_buffer[i].keycode = nk;
                action_pressed_cache_update(s_buffer[i].row, s_buffer[i].col, nk);
                for (uint8_t j = (uint8_t)(i + 1); j < n; j++) {
                    if (!s_buffer[j].pressed &&
                        s_buffer[j].row == s_buffer[i].row &&
                        s_buffer[j].col == s_buffer[i].col) {
                        s_buffer[j].keycode = nk;
                    }
                }
            }
        }

        // Try tapping first, then inner dispatch
        if (!action_tapping_process(&s_buffer[i])) {
            action_exec_inner(&s_buffer[i]);
        }
        send_keyboard_report();
    }
}

// Reset combo detection state
static void combo_reset(void) {
    for (int i = 0; i < COMBO_COUNT; i++) {
        s_combo_state[i].matched = 0;
        // Don't reset .fired — active combos stay until all keys released
    }
    s_combo_active = false;
    // Don't clear buffer here — caller decides to replay or not
}

// ---------------------------------------------------------------------------
// combo_process — called from action_exec() BEFORE tapping
// ---------------------------------------------------------------------------

bool combo_process(const keyevent_t *event) {
    uint8_t pos = pos_encode(event->row, event->col);

    // --- Release handling for active combos ---
    if (!event->pressed) {
        for (int i = 0; i < COMBO_COUNT; i++) {
            if (!s_combo_state[i].fired) continue;

            int8_t kidx = combo_key_index(&combo_defs[i], pos);
            if (kidx < 0) continue;

            // Clear matched bit
            s_combo_state[i].matched &= ~(1u << kidx);

            // All keys released → unregister result
            if (s_combo_state[i].matched == 0) {
                uint16_t result = combo_defs[i].result;
                if (IS_BASIC(result)) {
                    unregister_code((uint8_t)result);
                } else if (IS_MOD_KEY(result)) {
                    unregister_code(MODKEY_KC(result));
                    del_weak_mods(mod5_to_hid(MODKEY_MOD(result)));   // only our bits
                }
                s_combo_state[i].fired = false;
                DBG_KEY("[COMBO] #%d released\n", i);
            }
            return true;  // consumed
        }

        // Release during combo detection window — buffer it. On a full buffer
        // NEVER process this release ahead of its still-buffered press (that
        // guarantees a stuck key): abort detection, replay in order, then let
        // the release flow through the normal pipeline after its press.
        if (s_combo_active) {
            if (s_buf_count < COMBO_BUFFER_SIZE) {
                s_buffer[s_buf_count++] = *event;
                return true;
            }
            DBG_KEY("[COMBO] buffer full → abort + replay\n");
            combo_reset();
            replay_buffer();
        }
        return false;  // not consumed
    }

    // --- Press handling ---

    // Check if this key is part of any combo
    if (!pos_in_any_combo(pos)) {
        if (s_combo_active) {
            // Non-combo key pressed during detection → no combo possible
            combo_reset();
            replay_buffer();
        }
        return false;  // not consumed
    }

    // Start or continue combo detection window
    if (!s_combo_active) {
        s_combo_active = true;
        s_combo_timer  = event->timestamp;
    }

    // Buffer the event. A press we can't store must not silently vanish while
    // still driving the match state below: abort detection and process normally.
    if (s_buf_count >= COMBO_BUFFER_SIZE) {
        DBG_KEY("[COMBO] buffer full on press → abort + replay\n");
        combo_reset();
        replay_buffer();
        return false;  // this press flows through the normal pipeline
    }
    s_buffer[s_buf_count++] = *event;

    // Update matched state for each combo
    for (int i = 0; i < COMBO_COUNT; i++) {
        if (combo_defs[i].result == KC_NO) continue;
        if (s_combo_state[i].fired) continue;

        int8_t kidx = combo_key_index(&combo_defs[i], pos);
        if (kidx < 0) continue;

        s_combo_state[i].matched |= (1u << kidx);

        // Check if all keys matched
        uint8_t needed = combo_key_count(&combo_defs[i]);
        uint8_t full_mask = (1u << needed) - 1;

        if (s_combo_state[i].matched == full_mask) {
            // Combo complete — fire!
            s_combo_state[i].fired = true;
            uint16_t result = combo_defs[i].result;

            DBG_KEY("[COMBO] #%d fired → kc=0x%04x\n", i, result);

            if (IS_BASIC(result)) {
                register_code((uint8_t)result);
            } else if (IS_MOD_KEY(result)) {
                add_weak_mods(mod5_to_hid(MODKEY_MOD(result)));
                register_code(MODKEY_KC(result));
            }

            // Discard buffer (combo keys consumed)
            s_buf_count = 0;
            s_combo_active = false;

            // Reset other combo states (only one combo can fire)
            for (int j = 0; j < COMBO_COUNT; j++) {
                if (j != i) {
                    s_combo_state[j].matched = 0;
                }
            }
            return true;
        }
    }

    return true;  // buffered, waiting for more keys or timeout
}

// ---------------------------------------------------------------------------
// Tick — check COMBO_TERM timeout
// ---------------------------------------------------------------------------

void combo_tick(uint32_t now_ticks) {
    if (!s_combo_active) return;

    TickType_t elapsed = (TickType_t)(now_ticks - s_combo_timer);
    if (elapsed < pdMS_TO_TICKS(COMBO_TERM)) return;

    // Timeout — no complete combo within window
    DBG_KEY("[COMBO] timeout → replay %d events\n", s_buf_count);
    combo_reset();
    replay_buffer();
}
