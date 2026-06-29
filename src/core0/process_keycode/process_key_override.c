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
// process_key_override.c — Key Override: remap output based on active modifiers (Phase 9)
//
// Scans a user-defined table of (trigger + mods → replacement).
// On match: suppress specified mods, register replacement instead.
// Example: Shift+Backspace → Delete
//

#include "process_key_override.h"
#include "keycode.h"
#include "action_util.h"
#include "action_layer.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// Weak default: user defines overrides (empty by default)
// ---------------------------------------------------------------------------

__attribute__((weak))
const key_override_t key_overrides[KEY_OVERRIDE_COUNT] = { {0} };

// ---------------------------------------------------------------------------
// State — track active override for proper release handling
// ---------------------------------------------------------------------------

typedef struct {
    int8_t   index;        // override table index (-1 = none)
    uint16_t replacement;  // replacement keycode being held
    uint8_t  suppressed;   // mods that were suppressed
} active_override_t;

static active_override_t s_active;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void key_override_init(void) {
    s_active.index       = -1;
    s_active.replacement = KC_NO;
    s_active.suppressed  = 0;
}

// ---------------------------------------------------------------------------
// Process record
// ---------------------------------------------------------------------------

bool process_key_override(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // --- Physical release of a SUPPRESSED modifier while the override is held ---
    // The user let go of (e.g.) Shift before releasing the trigger: drop it from
    // the restore set, or the release below would re-register a modifier that is
    // no longer physically held (stuck Shift until the next tap). R2 fix.
    if (!event->pressed && s_active.index >= 0 &&
        IS_BASIC(kc) && IS_HID_MOD((uint8_t)kc)) {
        s_active.suppressed &= (uint8_t)~MOD_BIT((uint8_t)kc);
        return true;  // let the (no-op) unregister flow through
    }

    // --- Release of overridden key ---
    if (!event->pressed && s_active.index >= 0) {
        if (kc == key_overrides[s_active.index].trigger) {
            unregister_code((uint8_t)(s_active.replacement & 0xFF));
            // Restore suppressed mods (only the ones still physically held)
            if (s_active.suppressed) {
                register_mods(s_active.suppressed);
            }
            DBG_KEY("[KO] release override #%d → unreg 0x%04x\n",
                    s_active.index, s_active.replacement);
            s_active.index       = -1;
            s_active.replacement = KC_NO;
            s_active.suppressed  = 0;
            return false;  // consumed
        }
    }

    // --- Press: scan override table ---
    if (!event->pressed) return true;
    if (!IS_BASIC(kc)) return true;  // only override basic keycodes

    uint8_t mods = get_real_mods();
    uint32_t layer_state = layer_state_get();

    for (int i = 0; i < KEY_OVERRIDE_COUNT; i++) {
        const key_override_t *ko = &key_overrides[i];
        if (ko->trigger == KC_NO) continue;  // empty slot
        if (ko->trigger != kc) continue;
        if ((mods & ko->trigger_mods) != ko->trigger_mods) continue;
        if (ko->layers != 0xFF && !(ko->layers & layer_state)) continue;

        // Match found — activate override
        if (ko->suppressed_mods) {
            unregister_mods(ko->suppressed_mods);
        }
        register_code((uint8_t)(ko->replacement & 0xFF));

        s_active.index       = (int8_t)i;
        s_active.replacement = ko->replacement;
        s_active.suppressed  = ko->suppressed_mods & mods;  // only what was actually active

        DBG_KEY("[KO] override #%d: 0x%04x+mods=0x%02x → 0x%04x\n",
                i, kc, mods, ko->replacement);
        return false;  // consumed
    }

    return true;  // no override matched
}
