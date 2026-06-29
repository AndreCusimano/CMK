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
// process_dynamic_macro.c — Dynamic Macro: record and replay (Phase 10)
//
// Two macros share one buffer (grow from opposite ends, QMK-style).
// Macro 1: grows from index 0 upward.
// Macro 2: grows from index DYNAMIC_MACRO_SIZE-1 downward.
// Recording stops if pointers collide.
//
// Playback is non-blocking: one event per DYNAMIC_MACRO_DELAY ms via tick.
//

#include "process_dynamic_macro.h"
#include "keycode.h"
#include "action_util.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// Event type (4 bytes per event)
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t keycode;
    uint8_t  pressed;   // 1 = press, 0 = release
    uint8_t  _pad;      // alignment
} dm_event_t;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static dm_event_t s_dm_buffer[DYNAMIC_MACRO_SIZE];

// Macro 1: [0 .. s_dm_end1)
static uint8_t s_dm_end1;

// Macro 2: [s_dm_start2 .. DYNAMIC_MACRO_SIZE)
static uint8_t s_dm_start2;

// Recording
static int8_t  s_dm_recording;   // -1 = idle, 0 = macro 1, 1 = macro 2
static uint8_t s_dm_rec_ptr;

// Playback (non-blocking)
static int8_t   s_dm_playing;    // -1 = idle, 0 = macro 1, 1 = macro 2
static uint8_t  s_dm_play_ptr;
static uint8_t  s_dm_play_end;
static int8_t   s_dm_play_dir;   // +1 or -1
static uint32_t s_dm_play_timer;

// ---------------------------------------------------------------------------
// Weak user hooks
// ---------------------------------------------------------------------------

__attribute__((weak))
void dynamic_macro_record_start_user(int8_t macro_id) { (void)macro_id; }

__attribute__((weak))
void dynamic_macro_record_end_user(int8_t macro_id) { (void)macro_id; }

__attribute__((weak))
void dynamic_macro_play_user(int8_t macro_id) { (void)macro_id; }

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void dynamic_macro_init(void) {
    s_dm_end1      = 0;
    s_dm_start2    = DYNAMIC_MACRO_SIZE;
    s_dm_recording = -1;
    s_dm_playing   = -1;
}

// ---------------------------------------------------------------------------
// Recording helpers
// ---------------------------------------------------------------------------

static void dm_record_event(uint16_t kc, bool pressed) {
    if (s_dm_recording == 0) {
        // Macro 1: grow upward
        if (s_dm_rec_ptr >= s_dm_start2) {
            // Collision with macro 2 — stop
            DBG_KEY("[DM] rec1 buffer full\n");
            s_dm_end1 = s_dm_rec_ptr;
            s_dm_recording = -1;
            dynamic_macro_record_end_user(0);
            return;
        }
        s_dm_buffer[s_dm_rec_ptr].keycode = kc;
        s_dm_buffer[s_dm_rec_ptr].pressed = pressed ? 1 : 0;
        s_dm_rec_ptr++;
    } else if (s_dm_recording == 1) {
        // Macro 2: grow downward
        if (s_dm_rec_ptr <= s_dm_end1) {
            // Collision with macro 1 — stop
            DBG_KEY("[DM] rec2 buffer full\n");
            s_dm_start2 = s_dm_rec_ptr;
            s_dm_recording = -1;
            dynamic_macro_record_end_user(1);
            return;
        }
        s_dm_rec_ptr--;
        s_dm_buffer[s_dm_rec_ptr].keycode = kc;
        s_dm_buffer[s_dm_rec_ptr].pressed = pressed ? 1 : 0;
    }
}

static void dm_stop_recording(void) {
    if (s_dm_recording == 0) {
        s_dm_end1 = s_dm_rec_ptr;
        DBG_KEY("[DM] rec1 stop → %d events\n", s_dm_end1);
        dynamic_macro_record_end_user(0);
    } else if (s_dm_recording == 1) {
        s_dm_start2 = s_dm_rec_ptr;
        DBG_KEY("[DM] rec2 stop → %d events\n",
                (int)(DYNAMIC_MACRO_SIZE - s_dm_start2));
        dynamic_macro_record_end_user(1);
    }
    s_dm_recording = -1;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_dynamic_macro(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    if (!IS_DYN_MACRO(kc)) {
        // Not a DM keycode — if recording, capture the event
        if (s_dm_recording >= 0) {
            dm_record_event(kc, event->pressed);
        }
        return true;  // pass through (key is both recorded AND processed)
    }

    // DM keycodes — only act on press
    if (!event->pressed) return false;  // consume release

    switch (kc) {
        case DM_REC1:
            if (s_dm_recording >= 0) {
                dm_stop_recording();
            } else if (s_dm_playing < 0) {
                s_dm_recording = 0;
                s_dm_end1      = 0;
                s_dm_rec_ptr   = 0;
                DBG_KEY("[DM] rec1 start\n");
                dynamic_macro_record_start_user(0);
            }
            break;

        case DM_REC2:
            if (s_dm_recording >= 0) {
                dm_stop_recording();
            } else if (s_dm_playing < 0) {
                s_dm_recording = 1;
                s_dm_start2    = DYNAMIC_MACRO_SIZE;
                s_dm_rec_ptr   = DYNAMIC_MACRO_SIZE;
                DBG_KEY("[DM] rec2 start\n");
                dynamic_macro_record_start_user(1);
            }
            break;

        case DM_RSTP:
            if (s_dm_recording >= 0) {
                dm_stop_recording();
            }
            break;

        case DM_PLY1:
            if (s_dm_recording >= 0 || s_dm_playing >= 0) break;
            if (s_dm_end1 == 0) break;  // empty macro

            s_dm_playing    = 0;
            s_dm_play_ptr   = 0;
            s_dm_play_end   = s_dm_end1;
            s_dm_play_dir   = 1;
            s_dm_play_timer = 0;  // start immediately
            DBG_KEY("[DM] play1 → %d events\n", s_dm_end1);
            dynamic_macro_play_user(0);
            break;

        case DM_PLY2:
            if (s_dm_recording >= 0 || s_dm_playing >= 0) break;
            if (s_dm_start2 >= DYNAMIC_MACRO_SIZE) break;  // empty macro

            s_dm_playing    = 1;
            s_dm_play_ptr   = s_dm_start2;
            s_dm_play_end   = DYNAMIC_MACRO_SIZE;
            s_dm_play_dir   = 1;
            s_dm_play_timer = 0;
            DBG_KEY("[DM] play2 → %d events\n",
                    (int)(DYNAMIC_MACRO_SIZE - s_dm_start2));
            dynamic_macro_play_user(1);
            break;

        default:
            break;
    }

    return false;  // consumed
}

// ---------------------------------------------------------------------------
// Tick — non-blocking playback
// ---------------------------------------------------------------------------

void dynamic_macro_tick(uint32_t now_ticks) {
    if (s_dm_playing < 0) return;

    // Wait for delay
    if (s_dm_play_timer != 0) {
        TickType_t elapsed = (TickType_t)(now_ticks - s_dm_play_timer);
        if (elapsed < pdMS_TO_TICKS(DYNAMIC_MACRO_DELAY)) return;
    }

    // Play one event
    dm_event_t *ev = &s_dm_buffer[s_dm_play_ptr];

    if (ev->pressed) {
        if (IS_MOD_KEY(ev->keycode)) {
            add_weak_mods(mod5_to_hid(MODKEY_MOD(ev->keycode)));
            register_code(MODKEY_KC(ev->keycode));
        } else if (IS_BASIC(ev->keycode)) {
            register_code((uint8_t)ev->keycode);
        }
    } else {
        if (IS_MOD_KEY(ev->keycode)) {
            unregister_code(MODKEY_KC(ev->keycode));
            del_weak_mods(mod5_to_hid(MODKEY_MOD(ev->keycode)));   // only our bits
        } else if (IS_BASIC(ev->keycode)) {
            unregister_code((uint8_t)ev->keycode);
        }
    }

    s_dm_play_ptr += s_dm_play_dir;
    s_dm_play_timer = now_ticks;

    // Check if done
    if (s_dm_play_ptr >= s_dm_play_end) {
        DBG_KEY("[DM] play%d done\n", s_dm_playing + 1);
        s_dm_playing = -1;
    }
}
