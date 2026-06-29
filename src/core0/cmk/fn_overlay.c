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
// fn_overlay.c — see fn_overlay.h for the design rationale.
//

#include "fn_overlay.h"
#include "config/cmk_config.h"

#ifdef FN_OVERLAY_ENABLE

#include "keymap.h"               // keymaps[LAYER_SETTINGS]
#include "keycode.h"              // CMK_TD0/1/2, CMK_TU, CMK_OUT_USB, KC_FN, IS_DF, LAYER_VAL
#include "action_layer.h"         // layer_is_on, default_layer_get
#include "eeconfig.h"             // eeconfig_read_split_mode (split toggle state)
#include "connection_manager.h"   // cm_*_state getters
#include "bonding_store.h"        // active slot + per-slot bonded
#include "usb/usb_hid.h"          // usb_hid_is_active
#include "power_manager.h"        // pm_get_battery_status
#include "output_select.h"        // output_select_get (USB selected vs merely connected)

// Brightness for "on/selected" vs "saved-in-memory / connected-not-selected" states.
#define OV_FULL  255
#define OV_DIM    31

static const rgb_t OV_BLACK = { 0, 0, 0 };

// Blink phases driven by the millisecond tick — shared periods (CMK_BLINK_*)
// so GP3 (led_manager) and the dashboard blink in unison (R5).
static inline bool blink_fast(uint32_t now) { return ((now / CMK_BLINK_FAST_HALF_MS) & 1u) != 0u; }
static inline bool blink_slow(uint32_t now) { return ((now / CMK_BLINK_SLOW_HALF_MS) & 1u) != 0u; }

bool fn_overlay_active(void) {
    return layer_is_on(LAYER_SETTINGS);
}

rgb_t fn_overlay_color(uint8_t row, uint8_t col, uint32_t now) {
    if (row >= MATRIX_ROWS || col >= MATRIX_COLS) return OV_BLACK;

    const rgb_t blue    = { 0, 0, OV_FULL };
    const rgb_t blued   = { 0, 0, OV_DIM  };
    const rgb_t cyan    = { 0, OV_FULL, OV_FULL };
    const rgb_t purple  = { 160, 0, OV_FULL };
    const rgb_t purpled = { 160 * OV_DIM / OV_FULL, 0, OV_DIM };  // dim purple, same hue
    const rgb_t red     = { OV_FULL, 0, 0 };
    const rgb_t redd    = { OV_DIM,  0, 0 };           // dim red — DF present, not active
    const rgb_t yellow  = { OV_FULL, OV_FULL, 0 };     // split enabled
    const rgb_t yellowd = { OV_DIM,  OV_DIM,  0 };     // split disabled

    // row/col are PHYSICAL (LED-driven). Map the column into this half's slice of
    // the unified 4×12 keymap (Phase 18).
    uint16_t kc = keymaps[LAYER_SETTINGS][row][col + keymap_local_col_base()];

    // --- Downstream BLE slots — true PER-SLOT state (R5). The slot being
    // advertised is the PENDING one (connection_manager tracks it), no longer
    // the active slot: during a slot-switch or a new pairing the blink now
    // appears on the slot actually being worked on.
    if (kc == CMK_TD0 || kc == CMK_TD1 || kc == CMK_TD2) {
        uint8_t slot = (uint8_t)(kc - CMK_TD0);
        switch (connection_manager_ds_slot_state(slot)) {
            case CM_SLOT_CONNECTED:    return blue;
            case CM_SLOT_ADV_GENERAL:  return blink_fast(now) ? blue : OV_BLACK;
            case CM_SLOT_ADV_DIRECTED: return blink_slow(now) ? blue : OV_BLACK;
            case CM_SLOT_SAVED:        return blued;
            case CM_SLOT_EMPTY:
            default:                   return OV_BLACK;
        }
    }

    // --- Upstream BLE (the CMK_TU control key) ---
    if (kc == CMK_TU) {
        switch (connection_manager_upstream_state()) {
            case CM_UPSTREAM_CONNECTED:  return cyan;
            case CM_UPSTREAM_CONNECTING: return blink_slow(now) ? cyan : OV_BLACK;
            case CM_UPSTREAM_SCANNING:   return blink_fast(now) ? cyan : OV_BLACK;
            case CM_UPSTREAM_IDLE:
            default:                     return OV_BLACK;
        }
    }

    // --- USB: off if not connected; dim if connected but not the selected output;
    //     full if USB is the selected HID output. ---
    if (kc == CMK_OUT_USB) {
        if (!usb_hid_is_active()) return OV_BLACK;
        return (output_select_get() == OUTPUT_USB) ? purple : purpled;
    }

    // --- Battery charge (on the FN key). R5: gate on the gauge validity and
    // use the charger flag for the blink — `current == 0` is NOT "no cell"
    // (a full battery in termination idles at 0 mA and the LED went dark with
    // a healthy 100% cell). SoC-hue: 0=red … 85=green.
    if (kc == KC_FN) {
        pm_battery_status_t bat;
        pm_get_battery_status(&bat);
        if (!bat.valid) return OV_BLACK;                          // gauge unreadable → off
        uint8_t soc = bat.soc_pct > 100 ? 100 : bat.soc_pct;
        hsv_t h = { .h = (uint8_t)((uint16_t)soc * 85u / 100u), .s = 255, .v = OV_FULL };
        rgb_t color = hsv_to_rgb(h);
        if (pm_is_charging()) {                                   // charging → blink
            return blink_slow(now) ? color : OV_BLACK;
        }
        return color;                                             // on battery → steady
    }

    // --- Split config toggle (CMK_SPLIT_TOGG): yellow full = enabled, dim =
    // disabled, BLINKING = strict-matching mismatch (the sibling's split_mode
    // disagrees with ours → the halves are not cooperating; toggle one of them). ---
    if (kc == CMK_SPLIT_TOGG) {
        if (connection_manager_split_role() == CM_SPLIT_MISMATCH) {
            return blink_slow(now) ? yellow : OV_BLACK;
        }
        return eeconfig_read_split_mode() ? yellow : yellowd;
    }

    // --- Default layer (DF): full red = active default, dim red = present option ---
    if (IS_DF(kc)) {
        return (LAYER_VAL(kc) == default_layer_get()) ? red : redd;
    }

    return OV_BLACK;
}

#else  // !FN_OVERLAY_ENABLE — inert stubs

bool fn_overlay_active(void) { return false; }

rgb_t fn_overlay_color(uint8_t row, uint8_t col, uint32_t now) {
    (void)row; (void)col; (void)now;
    rgb_t black = { 0, 0, 0 };
    return black;
}

#endif // FN_OVERLAY_ENABLE
