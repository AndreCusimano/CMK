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
// process_magic.c — Magic: runtime keycode swaps (Phase 10)
//
// RAM-only settings (lost on reboot). Phase 11h (eeconfig) adds persistence.
//

#include "process_magic.h"
#include "keycode.h"
#include "eeconfig.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static magic_config_t s_magic;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

// Keymap flags bitmask layout (must match eeconfig)
#define MG_FLAG_SWAP_CTL_CAPS   (1u << 0)
#define MG_FLAG_SWAP_ESC_CAPS   (1u << 1)
#define MG_FLAG_CAPS_AS_CTL     (1u << 2)
#define MG_FLAG_SWAP_LALT_LGUI  (1u << 3)
#define MG_FLAG_SWAP_RALT_RGUI  (1u << 4)
#define MG_FLAG_GUI_DISABLED    (1u << 5)

static void magic_persist(void) {
    uint16_t flags = 0;
    if (s_magic.swap_ctl_caps)  flags |= MG_FLAG_SWAP_CTL_CAPS;
    if (s_magic.swap_esc_caps)  flags |= MG_FLAG_SWAP_ESC_CAPS;
    if (s_magic.caps_as_ctl)    flags |= MG_FLAG_CAPS_AS_CTL;
    if (s_magic.swap_lalt_lgui) flags |= MG_FLAG_SWAP_LALT_LGUI;
    if (s_magic.swap_ralt_rgui) flags |= MG_FLAG_SWAP_RALT_RGUI;
    if (s_magic.gui_disabled)   flags |= MG_FLAG_GUI_DISABLED;
    eeconfig_update_keymap_flags(flags);
}

void magic_init(void) {
    // Restore from eeconfig (Phase 11)
    uint16_t flags = eeconfig_read_keymap_flags();
    s_magic.swap_ctl_caps  = (flags & MG_FLAG_SWAP_CTL_CAPS)  != 0;
    s_magic.swap_esc_caps  = (flags & MG_FLAG_SWAP_ESC_CAPS)  != 0;
    s_magic.caps_as_ctl    = (flags & MG_FLAG_CAPS_AS_CTL)    != 0;
    s_magic.swap_lalt_lgui = (flags & MG_FLAG_SWAP_LALT_LGUI) != 0;
    s_magic.swap_ralt_rgui = (flags & MG_FLAG_SWAP_RALT_RGUI) != 0;
    s_magic.gui_disabled   = (flags & MG_FLAG_GUI_DISABLED)   != 0;
    DBG_KEY("[MAGIC] restored from eeconfig: flags=0x%04x\n", flags);
}

// ---------------------------------------------------------------------------
// Process — toggle settings on press
// ---------------------------------------------------------------------------

bool process_magic(const keyevent_t *event) {
    uint16_t kc = event->keycode;
    if (!IS_MAGIC(kc)) return true;  // pass through

    if (!event->pressed) return false;  // consume release silently

    switch (kc) {
        case MG_SWAP_CTL_CAPS:
            s_magic.swap_ctl_caps = !s_magic.swap_ctl_caps;
            DBG_KEY("[MAGIC] swap_ctl_caps = %d\n", s_magic.swap_ctl_caps);
            break;
        case MG_SWAP_ESC_CAPS:
            s_magic.swap_esc_caps = !s_magic.swap_esc_caps;
            DBG_KEY("[MAGIC] swap_esc_caps = %d\n", s_magic.swap_esc_caps);
            break;
        case MG_CAPS_AS_CTL:
            s_magic.caps_as_ctl = !s_magic.caps_as_ctl;
            DBG_KEY("[MAGIC] caps_as_ctl = %d\n", s_magic.caps_as_ctl);
            break;
        case MG_SWAP_LALT_LGUI:
            s_magic.swap_lalt_lgui = !s_magic.swap_lalt_lgui;
            DBG_KEY("[MAGIC] swap_lalt_lgui = %d\n", s_magic.swap_lalt_lgui);
            break;
        case MG_SWAP_RALT_RGUI:
            s_magic.swap_ralt_rgui = !s_magic.swap_ralt_rgui;
            DBG_KEY("[MAGIC] swap_ralt_rgui = %d\n", s_magic.swap_ralt_rgui);
            break;
        case MG_GUI_OFF:
            s_magic.gui_disabled = true;
            DBG_KEY("[MAGIC] GUI disabled\n");
            break;
        case MG_GUI_ON:
            s_magic.gui_disabled = false;
            DBG_KEY("[MAGIC] GUI enabled\n");
            break;
        default:
            break;
    }

    // Persist to EEPROM (Phase 11)
    magic_persist();

    return false;  // consumed
}

// ---------------------------------------------------------------------------
// Transform — apply swaps to a keycode before dispatch
// ---------------------------------------------------------------------------

uint16_t magic_transform_keycode(uint16_t kc) {
    if (!IS_BASIC(kc)) return kc;

    // Ctrl ↔ Caps Lock
    if (s_magic.swap_ctl_caps) {
        if (kc == KC_LCTL) return KC_CAPS;
        if (kc == KC_CAPS) return KC_LCTL;
    }

    // ESC ↔ Caps Lock
    if (s_magic.swap_esc_caps) {
        if (kc == KC_ESC)  return KC_CAPS;
        if (kc == KC_CAPS) return KC_ESC;
    }

    // Caps Lock → Left Ctrl (one-way)
    if (s_magic.caps_as_ctl) {
        if (kc == KC_CAPS) return KC_LCTL;
    }

    // Left Alt ↔ Left GUI
    if (s_magic.swap_lalt_lgui) {
        if (kc == KC_LALT) return KC_LGUI;
        if (kc == KC_LGUI) return KC_LALT;
    }

    // Right Alt ↔ Right GUI
    if (s_magic.swap_ralt_rgui) {
        if (kc == KC_RALT) return KC_RGUI;
        if (kc == KC_RGUI) return KC_RALT;
    }

    // GUI disabled
    if (s_magic.gui_disabled) {
        if (kc == KC_LGUI || kc == KC_RGUI) return KC_NO;
    }

    return kc;
}
