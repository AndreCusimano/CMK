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
// action.c — Central key action dispatcher (Phase 7)
//

#include "action.h"
#include "keycode.h"
#include "keymap.h"
#include "action_layer.h"
#include "action_util.h"
#include "action_tapping.h"
#include "process_record.h"
#include "cmk_keycodes.h"
#include "output_select.h"
#include "inter_half_sync.h"
#include "process_combo.h"
#include "process_tap_dance.h"
#include "process_caps_word.h"
#include "process_key_override.h"
#include "process_mouse_keys.h"
#include "process_repeat_key.h"
#include "process_auto_shift.h"
#include "process_leader.h"
#include "process_dynamic_macro.h"
#include "process_space_cadet.h"
#include "process_grave_esc.h"
#include "process_magic.h"
#include "send_string.h"
#include "process_layer_lock.h"
#include "process_key_lock.h"
#include "process_secure.h"
#include "swap_hands.h"
#include "process_tri_layer.h"
#include "wpm.h"
#include "velocikey.h"
#include "eeconfig.h"
#include "unicode.h"
#include "process_unicode.h"
#include "process_unicodemap.h"
#include "process_ucis.h"
#include "process_programmable_button.h"
#include "rgb_driver.h"
#include "config/cmk_config.h"
#if defined(RGB_MATRIX_ENABLE)
#include "rgb_matrix.h"
#elif defined(LED_MATRIX_ENABLE)
#include "led_matrix.h"
#elif defined(RGBLIGHT_ENABLE)
#include "rgblight.h"
#endif
#ifdef BACKLIGHT_ENABLE
#include "backlight.h"
#endif
#include "low_power.h"
#include "config/debug_config.h"

#include <stdio.h>

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void action_init(void) {
    DBG_BOOT("[ACT]  action_layer_init\n");           action_layer_init();
    DBG_BOOT("[ACT]  action_util_init\n");            action_util_init();
    DBG_BOOT("[ACT]  action_tapping_init\n");         action_tapping_init();
    DBG_BOOT("[ACT]  cmk_keycodes_init\n");           cmk_keycodes_init();
    DBG_BOOT("[ACT]  output_select_init\n");          output_select_init();
    DBG_BOOT("[ACT]  inter_half_sync_init\n");        inter_half_sync_init();
    DBG_BOOT("[ACT]  combo_init\n");                  combo_init();
    DBG_BOOT("[ACT]  tap_dance_init\n");              tap_dance_init();
    DBG_BOOT("[ACT]  caps_word_init\n");              caps_word_init();
    DBG_BOOT("[ACT]  key_override_init\n");           key_override_init();
    DBG_BOOT("[ACT]  mouse_keys_init\n");             mouse_keys_init();
    DBG_BOOT("[ACT]  repeat_key_init\n");             repeat_key_init();
    DBG_BOOT("[ACT]  auto_shift_init\n");             auto_shift_init();
    DBG_BOOT("[ACT]  leader_init\n");                 leader_init();
    DBG_BOOT("[ACT]  dynamic_macro_init\n");          dynamic_macro_init();
    DBG_BOOT("[ACT]  space_cadet_init\n");            space_cadet_init();
    DBG_BOOT("[ACT]  grave_esc_init\n");              grave_esc_init();
    DBG_BOOT("[ACT]  magic_init\n");                  magic_init();
    DBG_BOOT("[ACT]  send_string_init\n");            send_string_init();
    DBG_BOOT("[ACT]  layer_lock_init\n");             layer_lock_init();
    DBG_BOOT("[ACT]  key_lock_init\n");               key_lock_init();
    DBG_BOOT("[ACT]  secure_init\n");                 secure_init();
    DBG_BOOT("[ACT]  swap_hands_init\n");             swap_hands_init();
    DBG_BOOT("[ACT]  wpm_init\n");                    wpm_init();
    DBG_BOOT("[ACT]  velocikey_init\n");              velocikey_init();
    DBG_BOOT("[ACT]  process_unicode_init\n");        process_unicode_init();
    DBG_BOOT("[ACT]  ucis_init\n");                   ucis_init();
    DBG_BOOT("[ACT]  programmable_button_init\n");    programmable_button_init();

#if defined(RGB_MATRIX_ENABLE) || defined(LED_MATRIX_ENABLE) || defined(RGBLIGHT_ENABLE)
    DBG_BOOT("[ACT]  rgb_driver_init (PIO + VLED_EN)\n");
    rgb_driver_init();
#endif
#if defined(RGB_MATRIX_ENABLE)
    DBG_BOOT("[ACT]  rgb_matrix_init\n");
    rgb_matrix_init();
#elif defined(LED_MATRIX_ENABLE)
    DBG_BOOT("[ACT]  led_matrix_init\n");
    led_matrix_init();
#elif defined(RGBLIGHT_ENABLE)
    DBG_BOOT("[ACT]  rgblight_init\n");
    rgblight_init();
#endif
#ifdef BACKLIGHT_ENABLE
    DBG_BOOT("[ACT]  backlight_init\n");
    backlight_init();
#endif
    DBG_BOOT("[ACT]  low_power_init\n");              low_power_init();
    DBG_BOOT("[ACT]  action_init DONE\n");
}

// ---------------------------------------------------------------------------
// action_exec_inner — dispatch without tapping (called directly or from replay)
// ---------------------------------------------------------------------------

void action_exec_inner(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // KC_NO / KC_TRNS — nothing to do
    if (kc == KC_NO || kc == KC_TRNS) return;

    // --- Magic keycode transform (Ctrl↔Caps, ESC↔Caps, GUI disable, etc.) ---
    kc = magic_transform_keycode(kc);

    // If keycode was transformed, create a modified event for the chain
    keyevent_t transformed_event;
    const keyevent_t *ev = event;
    if (kc != event->keycode) {
        transformed_event = *event;
        transformed_event.keycode = kc;
        ev = &transformed_event;
    }

    // --- Process record chain (CMK custom + user) ---
    if (!process_record(ev)) return;  // consumed

    // --- Basic HID keycode (0x00-0xFF) ---
    if (IS_BASIC(kc)) {
        if (event->pressed) {
            register_code((uint8_t)kc);
        } else {
            unregister_code((uint8_t)kc);
        }
        return;
    }

    // --- Mods+Key (e.g. LSFT(KC_1) = '!') ---
    if (IS_MOD_KEY(kc)) {
        uint8_t mods = mod5_to_hid(MODKEY_MOD(kc));
        uint8_t basic = MODKEY_KC(kc);
        if (event->pressed) {
            add_weak_mods(mods);
            register_code(basic);
        } else {
            unregister_code(basic);
            del_weak_mods(mods);   // only OUR bits — never wipe other features' weak mods
        }
        return;
    }

    // --- Mod-Tap release (after tapping resolved as hold) ---
    if (IS_MOD_TAP(kc) && !event->pressed) {
        uint8_t mods = mod5_to_hid(MT_MOD(kc));
        unregister_mods(mods);
        return;
    }

    // --- Layer-Tap release (after tapping resolved as hold) ---
    if (IS_LAYER_TAP(kc) && !event->pressed) {
        uint8_t layer = LT_LAYER(kc);
        layer_off(layer);
        return;
    }

    // --- Layer keys ---
    if (IS_LAYER_KEY(kc)) {
        uint8_t layer = LAYER_VAL(kc);

        if (IS_MO(kc)) {
            if (event->pressed) layer_on(layer);
            else                layer_off(layer);
        } else if (IS_TG(kc)) {
            if (event->pressed) layer_toggle(layer);
        } else if (IS_TO(kc)) {
            if (event->pressed) {
                // Turn off all layers except default, then turn on target
                layer_set(0);
                if (layer != default_layer_get()) layer_on(layer);
            }
        } else if (IS_DF(kc)) {
            if (event->pressed) default_layer_set(layer);
        } else if (IS_OSL(kc)) {
            if (event->pressed) layer_osl_set(layer);
        }
        return;
    }

    // --- CMK custom keycodes are handled by process_record above ---
    // If we reach here, the keycode is unrecognized — ignore silently.
}

// ---------------------------------------------------------------------------
// Pressed-key cache (R2)
// ---------------------------------------------------------------------------
// Identity of a held key = the keycode resolved at PRESS time, NOT a re-lookup
// at release. Keyed by the RAW (pre swap-hands) position so a swap-state change
// between press and release can't move the release to a different entry. Also
// stores the post-remap position used in the press event, so the release event
// matches the press event exactly (tapping/auto-shift match on row/col).
typedef struct {
    uint16_t keycode;
    uint8_t  row;      // post-remap row used at press
    uint8_t  col;      // post-remap logical col used at press
    bool     valid;
} pressed_entry_t;

static pressed_entry_t s_pressed[MATRIX_ROWS][KEYMAP_COLS];

void action_pressed_cache_update(uint8_t row, uint8_t col, uint16_t keycode) {
    // Replay paths re-resolve a buffered press on a new layer: keep the cache in
    // sync so the eventual release unregisters the keycode actually registered.
    // Lookup by the POST-remap position (replay buffers carry that one); for the
    // common no-swap case it equals the raw position. On a swap mismatch, scan.
    if (row < MATRIX_ROWS && col < KEYMAP_COLS && s_pressed[row][col].valid &&
        s_pressed[row][col].row == row && s_pressed[row][col].col == col) {
        s_pressed[row][col].keycode = keycode;
        return;
    }
    for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
        for (uint8_t c = 0; c < KEYMAP_COLS; c++) {
            if (s_pressed[r][c].valid && s_pressed[r][c].row == row && s_pressed[r][c].col == col) {
                s_pressed[r][c].keycode = keycode;
                return;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// action_exec — main entry point from keyscan
// ---------------------------------------------------------------------------

void action_exec(uint8_t row, uint8_t col, bool pressed, uint32_t timestamp) {
    uint8_t raw_row = row, raw_col = col;
    uint16_t kc;

    if (pressed) {
        // Phase 18: `col` is a LOGICAL column 0..KEYMAP_COLS-1 of the unified 4×12
        // keymap. Keys in THIS half's slice also drive Swap-Hands + reactive LED on
        // PHYSICAL coordinates; remote keys (split MASTER resolving the peer half)
        // are outside the local slice → skip both (their LED lives on the other half).
        uint8_t base = keymap_local_col_base();
        if (col >= base && col < (uint8_t)(base + MATRIX_COLS)) {
            uint8_t phys = (uint8_t)(col - base);
            swap_hands_remap(&row, &phys);        // local physical 0..MATRIX_COLS-1
            col = (uint8_t)(phys + base);         // re-logicalize after a swap
#if defined(RGB_MATRIX_ENABLE)
            rgb_matrix_record_hit(row, phys);
#elif defined(LED_MATRIX_ENABLE)
            led_matrix_record_hit(row, phys);
#endif
        }

        kc = layer_lookup(row, col);              // logical walk over the 4×12 keymap

        // Cache press identity for the matching release (overwrite a stale entry).
        if (raw_row < MATRIX_ROWS && raw_col < KEYMAP_COLS) {
            pressed_entry_t *pe = &s_pressed[raw_row][raw_col];
            pe->keycode = kc;
            pe->row     = row;
            pe->col     = col;
            pe->valid   = true;
        }
    } else {
        pressed_entry_t *pe = (raw_row < MATRIX_ROWS && raw_col < KEYMAP_COLS)
                                  ? &s_pressed[raw_row][raw_col] : NULL;
        if (pe && pe->valid) {
            // Release uses the press identity — immune to layer/swap changes.
            kc  = pe->keycode;
            row = pe->row;
            col = pe->col;
            pe->valid = false;
        } else {
            // No cached press (boot transient, split snapshot diff): legacy path.
            uint8_t base = keymap_local_col_base();
            if (col >= base && col < (uint8_t)(base + MATRIX_COLS)) {
                uint8_t phys = (uint8_t)(col - base);
                swap_hands_remap(&row, &phys);
                col = (uint8_t)(phys + base);
            }
            kc = layer_lookup(row, col);
        }
    }

    // Key Lock: suppress release of locked key
    if (key_lock_should_suppress(kc, pressed)) return;

    // WPM: count keypresses
    if (pressed) wpm_keypress();

    keyevent_t event = {
        .keycode   = kc,
        .row       = row,
        .col       = col,
        .pressed   = pressed,
        .timestamp = timestamp,
    };

    DBG_KEY("[ACT] [%d][%d] %s → kc=0x%04x\n",
            row, col, pressed ? "press" : "release", kc);

    // Combo pre-process (buffers events during COMBO_TERM window)
    if (combo_process(&event)) return;

    // Try tapping system first (for MT/LT keys)
    if (action_tapping_process(&event)) return;

    // Not a tapping key (or tapping is idle) — dispatch directly
    action_exec_inner(&event);

    // One-shot layer: clear after any non-OSL key release
    if (!pressed && layer_osl_active() && !IS_OSL(kc)) {
        layer_osl_clear();
    }
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void action_tick(uint32_t now_ticks) {
    combo_tick(now_ticks);
    tap_dance_tick(now_ticks);
    mouse_keys_tick(now_ticks);
    send_string_tick(now_ticks);
    action_tapping_tick(now_ticks);
    tap_code_tick(now_ticks);
    action_util_tick(now_ticks);   // R2: USB confirmed-delivery retry / CM merge flush
    cmk_keycodes_tick(now_ticks);
    // inter_half_sync_tick: R6 — moved to its own ih_link task (the slave-leg
    // EEPROM flushes froze this 1 kHz tick during configuration).
    caps_word_tick(now_ticks);
    auto_shift_tick(now_ticks);
    leader_tick(now_ticks);
    space_cadet_tick(now_ticks);
    dynamic_macro_tick(now_ticks);
    wpm_tick(now_ticks);
    velocikey_tick(now_ticks);
    unicode_tick(now_ticks);
    low_power_tick(now_ticks);     // idle state machine (LED suspend / deep-sleep arming)
#if defined(RGB_MATRIX_ENABLE)
    rgb_matrix_tick(now_ticks);
#elif defined(LED_MATRIX_ENABLE)
    led_matrix_tick(now_ticks);
#elif defined(RGBLIGHT_ENABLE)
    rgblight_tick(now_ticks);
#endif
}
