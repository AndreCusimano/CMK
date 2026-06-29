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
// process_unicodemap.c — Unicode Map: UM()/UP() lookup table handler (Phase 12)
//
// UM(index): look up unicode_map[index] → send codepoint
// UP(i, j):  unshifted = unicode_map[i], shifted = unicode_map[j]
//            (check current real_mods for Shift)
//

#include "process_unicodemap.h"
#include "unicode.h"
#include "keycode.h"
#include "action_util.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// Weak default: empty unicode map (user provides in keymap)
// ---------------------------------------------------------------------------

__attribute__((weak))
const uint32_t unicode_map[] = { 0 };

__attribute__((weak))
const uint16_t unicode_map_size = 0;

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_unicodemap(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // --- UM(index) ---
    if (IS_UNICODEMAP(kc)) {
        if (event->pressed && !unicode_is_active()) {
            uint8_t idx = kc & 0xFF;
            if (idx < unicode_map_size) {
                uint32_t cp = unicode_map[idx];
                DBG_KEY("[UMAP] UM(%d) → U+%04lX\n", idx, (unsigned long)cp);
                unicode_send_codepoint(cp);
            }
        }
        return false;  // consumed
    }

    // --- UP(i, j) ---
    if (IS_UNICODEMAP_PAIR(kc)) {
        if (event->pressed && !unicode_is_active()) {
            uint8_t i = (kc >> 4) & 0xF;
            uint8_t j = kc & 0xF;

            // Check if Shift is active
            bool shifted = (get_real_mods() & (MOD_BIT(KC_LSFT) | MOD_BIT(KC_RSFT))) != 0;
            uint8_t idx = shifted ? j : i;

            if (idx < unicode_map_size) {
                uint32_t cp = unicode_map[idx];
                // Temporarily clear Shift so it doesn't interfere with the sequence
                if (shifted) {
                    unregister_mods(MOD_BIT(KC_LSFT) | MOD_BIT(KC_RSFT));
                }
                DBG_KEY("[UMAP] UP(%d,%d) %s → U+%04lX\n",
                        i, j, shifted ? "shifted" : "plain", (unsigned long)cp);
                unicode_send_codepoint(cp);
            }
        }
        return false;  // consumed
    }

    return true;  // pass through
}
