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
// process_macro.c — see process_macro.h for the design.
//
// FSM modeled on send_string_tick (press → delay → release → delay) reading
// raw bytes from the macro_storage RAM cache (stable lifetime). Emission via
// register_code/unregister_code + weak shift, like dynamic_macro playback.
// Robustness: truncated escape / kc==0 in an escape / buffer end without NUL
// → stop playback; control bytes outside escapes are skipped.
//

#include "process_macro.h"
#include "macro_storage.h"
#include "keycode.h"
#include "action_util.h"
#include "send_string.h"      // shared ASCII LUT + busy check
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

#ifdef MACRO_STORAGE_ENABLE

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

#define MP_ESCAPE     0x01
#define MP_ESC_TAP    0x01
#define MP_ESC_DOWN   0x02
#define MP_ESC_UP     0x03
#define MP_ESC_DELAY  0x04

static const uint8_t *s_mp_ptr;        // next byte to interpret (NULL = idle)
static const uint8_t *s_mp_end;        // buffer end guard
static uint32_t s_mp_timer;            // tick timestamp of the last step
static uint32_t s_mp_wait_ms;          // delay before the next step (step pacing or DELAY escape)
static bool     s_mp_key_down;         // in the release phase of a TAP/char
static uint8_t  s_mp_cur_kc;           // keycode currently pressed
static bool     s_mp_shifted;          // we added weak LSFT for this char
// Set by the config protocol (USB / ih_link task), consumed by keyscan.
static volatile bool s_mp_abort_req;

// ---------------------------------------------------------------------------
// Init / API
// ---------------------------------------------------------------------------

void macro_play_init(void) {
    s_mp_ptr      = NULL;
    s_mp_end      = NULL;
    s_mp_timer    = 0;
    s_mp_wait_ms  = 0;
    s_mp_key_down = false;
    s_mp_cur_kc   = 0;
    s_mp_shifted  = false;
}

bool macro_play_is_active(void) {
    return s_mp_ptr != NULL;
}

// Keyscan context only.
static void macro_play_abort(void) {
    if (s_mp_key_down) {
        unregister_code(s_mp_cur_kc);
        if (s_mp_shifted) del_weak_mods(MOD_BIT(KC_LSFT));
    }
    macro_play_init();
}

void macro_play_request_abort(void) {
    s_mp_abort_req = true;
}

// Serve a pending abort request before any buffer access (keyscan context).
static void mp_service_abort(void) {
    if (!s_mp_abort_req) return;
    s_mp_abort_req = false;
    macro_play_abort();
}

// ---------------------------------------------------------------------------
// Process — MACRO(n) press starts playback
// ---------------------------------------------------------------------------

bool process_macro(const keyevent_t *event) {
    uint16_t kc = event->keycode;
    if (!IS_MACRO_KC(kc)) return true;

    if (!event->pressed) return false;   // consume release
    mp_service_abort();

    // No queue, no nesting: busy (macro or send_string) → ignore the press.
    if (macro_play_is_active() || send_string_is_active()) {
        DBG_KEY("[MACRO] busy — MACRO(%u) ignorata\n", (unsigned)MACRO_INDEX(kc));
        return false;
    }

    const uint8_t *start = macro_storage_find((uint8_t)MACRO_INDEX(kc));
    if (start == NULL || *start == 0x00) {
        DBG_KEY("[MACRO] MACRO(%u) vuota/assente\n", (unsigned)MACRO_INDEX(kc));
        return false;
    }

    s_mp_ptr     = start;
    s_mp_end     = macro_storage_data() + macro_storage_size();
    s_mp_timer   = 0;      // first step on the next tick
    s_mp_wait_ms = 0;
    DBG_KEY("[MACRO] play MACRO(%u)\n", (unsigned)MACRO_INDEX(kc));
    return false;
}

// ---------------------------------------------------------------------------
// Tick — one FSM step per MACRO_PLAY_DELAY (or a pending DELAY escape)
// ---------------------------------------------------------------------------

// Press a resolved 8-bit keycode (with optional weak shift) and enter the
// release phase.
static void mp_press(uint8_t kc, bool shifted) {
    if (shifted) add_weak_mods(MOD_BIT(KC_LSFT));
    register_code(kc);
    s_mp_cur_kc   = kc;
    s_mp_shifted  = shifted;
    s_mp_key_down = true;
}

void macro_play_tick(uint32_t now_ticks) {
    mp_service_abort();
    if (s_mp_ptr == NULL) return;

    if (s_mp_timer != 0) {
        TickType_t elapsed = (TickType_t)(now_ticks - s_mp_timer);
        uint32_t   wait_ms = s_mp_wait_ms ? s_mp_wait_ms : MACRO_PLAY_DELAY;
        if (elapsed < pdMS_TO_TICKS(wait_ms)) return;
    }
    s_mp_wait_ms = 0;

    // Release phase of the previous TAP/char.
    if (s_mp_key_down) {
        unregister_code(s_mp_cur_kc);
        if (s_mp_shifted) del_weak_mods(MOD_BIT(KC_LSFT));
        s_mp_key_down = false;
        s_mp_shifted  = false;
        s_mp_timer    = now_ticks;
        return;
    }

    // Decode the next byte.
    if (s_mp_ptr >= s_mp_end) {              // buffer end without NUL → stop
        macro_play_init();
        return;
    }
    uint8_t b = *s_mp_ptr++;

    if (b == 0x00) {                         // end of macro
        DBG_KEY("[MACRO] done\n");
        macro_play_init();
        return;
    }

    if (b == MP_ESCAPE) {
        // Escape: need at least the opcode byte.
        if (s_mp_ptr >= s_mp_end) { macro_play_init(); return; }
        uint8_t op = *s_mp_ptr++;
        switch (op) {
        case MP_ESC_TAP:
        case MP_ESC_DOWN:
        case MP_ESC_UP: {
            if (s_mp_ptr >= s_mp_end) { macro_play_init(); return; }
            uint8_t kc = *s_mp_ptr++;
            if (kc == 0x00) { macro_play_init(); return; }   // malformed
            if (op == MP_ESC_TAP) {
                mp_press(kc, false);          // release on the next step
            } else if (op == MP_ESC_DOWN) {
                register_code(kc);            // stays held (documented)
            } else {
                unregister_code(kc);
            }
            s_mp_timer = now_ticks;
            return;
        }
        case MP_ESC_DELAY: {
            if (s_mp_ptr + 1 >= s_mp_end) { macro_play_init(); return; }
            uint8_t b1 = *s_mp_ptr++;
            uint8_t b2 = *s_mp_ptr++;
            if (b1 == 0 || b2 == 0) { macro_play_init(); return; }   // malformed
            s_mp_wait_ms = (uint32_t)(b1 - 1) + (uint32_t)(b2 - 1) * 255u;
            s_mp_timer = now_ticks;   // 0 ms decade nel pacing standard al prossimo step
            return;
        }
        default:                              // unknown opcode → stop (safety)
            macro_play_init();
            return;
        }
    }

    // Plain byte: printable ASCII via the send_string LUT; anything the LUT
    // doesn't map (control chars 0x02..0x1F, DEL) is skipped without pacing.
    uint16_t entry = send_string_ascii_entry((char)b);
    if (entry == 0) return;                   // skip, re-enter on next tick
    mp_press((uint8_t)(entry & 0xFF), (entry & 0x8000) != 0);
    s_mp_timer = now_ticks;
}

#else  // !MACRO_STORAGE_ENABLE — inert stubs

void macro_play_init(void)                      { }
bool process_macro(const keyevent_t *event)     { (void)event; return true; }
void macro_play_tick(uint32_t now_ticks)        { (void)now_ticks; }
bool macro_play_is_active(void)                 { return false; }
void macro_play_request_abort(void)             { }

#endif // MACRO_STORAGE_ENABLE
