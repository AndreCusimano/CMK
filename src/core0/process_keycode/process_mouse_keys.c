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
// process_mouse_keys.c — Mouse Keys: keyboard-emulated mouse (Phase 9)
//
// Movement keys (KC_MS_U/D/L/R) set direction flags. mouse_keys_tick()
// sends periodic mouse reports via IPC at MK_INTERVAL rate.
// Button keys (KC_BTN1-5) send immediate reports.
// Acceleration presets (KC_ACL0-2) change the speed multiplier.
//
// Mouse HID report format (7 bytes, Report ID 2):
//   [0] buttons  (bit0=L, bit1=R, bit2=M, bit3=Back, bit4=Fwd)
//   [1] X low    (int16_t little-endian)
//   [2] X high
//   [3] Y low    (int16_t little-endian)
//   [4] Y high
//   [5] wheel    (int8_t, vertical scroll)
//   [6] pan      (int8_t, horizontal scroll)
//

#include "process_mouse_keys.h"
#include "keycode.h"
#include "ipc/ipc_core.h"
#include "config/cmk_config.h"

#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif
#include "output_select.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t  s_mk_buttons;      // button bitmask
static int8_t   s_mk_move_x;       // -1, 0, +1
static int8_t   s_mk_move_y;
static int8_t   s_mk_wheel_v;      // vertical scroll direction
static int8_t   s_mk_wheel_h;      // horizontal scroll direction
static uint32_t s_mk_last_report;   // timestamp of last report
static bool     s_mk_active;        // any movement/wheel key held
static uint8_t  s_mk_accel;         // acceleration preset (0=slow, 1=default, 2=fast)

static uint8_t  s_mk_prev_buttons;  // previous button state for dedup

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void mouse_keys_init(void) {
    s_mk_buttons      = 0;
    s_mk_move_x       = 0;
    s_mk_move_y       = 0;
    s_mk_wheel_v      = 0;
    s_mk_wheel_h      = 0;
    s_mk_last_report  = 0;
    s_mk_active       = false;
    s_mk_accel        = 1;  // default = medium
    s_mk_prev_buttons = 0;
}

// ---------------------------------------------------------------------------
// Send mouse report via IPC
// ---------------------------------------------------------------------------

static void mk_send_report(int16_t dx, int16_t dy, int8_t wheel, int8_t pan) {
    uint8_t report[7];
    report[0] = s_mk_buttons;
    report[1] = (uint8_t)(dx & 0xFF);
    report[2] = (uint8_t)((dx >> 8) & 0xFF);
    report[3] = (uint8_t)(dy & 0xFF);
    report[4] = (uint8_t)((dy >> 8) & 0xFF);
    report[5] = (uint8_t)wheel;
    report[6] = (uint8_t)pan;

    ipc_put_mouse_report(report);
#ifdef USB_ENABLE
    // Local mouse goes to USB only when USB is the selected output.
    if (output_usb_enabled()) {
        usb_hid_send_mouse_report(report);
    }
#endif
}

// ---------------------------------------------------------------------------
// Get delta based on acceleration preset
// ---------------------------------------------------------------------------

static int16_t mk_get_move_delta(void) {
    switch (s_mk_accel) {
    case 0:  return MK_MOVE_DELTA / 2 > 0 ? MK_MOVE_DELTA / 2 : 1;
    case 2:  return MK_MOVE_DELTA * 3;
    default: return MK_MOVE_DELTA;
    }
}

// ---------------------------------------------------------------------------
// Process record
// ---------------------------------------------------------------------------

bool process_mouse_keys(const keyevent_t *event) {
    uint16_t kc = event->keycode;
    if (!IS_MOUSEKEY(kc)) return true;  // not ours

    bool pressed = event->pressed;

    // --- Movement keys ---
    if (IS_MOUSEKEY_MOVE(kc)) {
        switch (kc) {
        case KC_MS_U: s_mk_move_y = pressed ? -1 : 0; break;
        case KC_MS_D: s_mk_move_y = pressed ?  1 : 0; break;
        case KC_MS_L: s_mk_move_x = pressed ? -1 : 0; break;
        case KC_MS_R: s_mk_move_x = pressed ?  1 : 0; break;
        }
        s_mk_active = (s_mk_move_x != 0 || s_mk_move_y != 0 ||
                        s_mk_wheel_v != 0 || s_mk_wheel_h != 0);
        if (!s_mk_active && s_mk_buttons == 0) {
            // All released — send zero report
            mk_send_report(0, 0, 0, 0);
        }
        return false;
    }

    // --- Button keys ---
    if (IS_MOUSEKEY_BUTTON(kc)) {
        uint8_t btn = 1u << (kc - KC_BTN1);
        if (pressed) {
            s_mk_buttons |= btn;
        } else {
            s_mk_buttons &= ~btn;
        }
        // Send immediate report on button change
        if (s_mk_buttons != s_mk_prev_buttons) {
            mk_send_report(0, 0, 0, 0);
            s_mk_prev_buttons = s_mk_buttons;
        }
        return false;
    }

    // --- Wheel keys ---
    if (IS_MOUSEKEY_WHEEL(kc)) {
        switch (kc) {
        case KC_WH_U: s_mk_wheel_v = pressed ?  MK_WHEEL_DELTA : 0; break;
        case KC_WH_D: s_mk_wheel_v = pressed ? -MK_WHEEL_DELTA : 0; break;
        case KC_WH_L: s_mk_wheel_h = pressed ? -MK_WHEEL_DELTA : 0; break;
        case KC_WH_R: s_mk_wheel_h = pressed ?  MK_WHEEL_DELTA : 0; break;
        }
        s_mk_active = (s_mk_move_x != 0 || s_mk_move_y != 0 ||
                        s_mk_wheel_v != 0 || s_mk_wheel_h != 0);
        return false;
    }

    // --- Acceleration presets ---
    if (IS_MOUSEKEY_ACCEL(kc)) {
        if (pressed) {
            s_mk_accel = (uint8_t)(kc - KC_ACL0);
            DBG_KEY("[MK] accel preset %d\n", s_mk_accel);
        }
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Tick — send periodic movement reports
// ---------------------------------------------------------------------------

void mouse_keys_tick(uint32_t now_ticks) {
    if (!s_mk_active) return;

    TickType_t elapsed = (TickType_t)(now_ticks - s_mk_last_report);
    if (elapsed < pdMS_TO_TICKS(MK_INTERVAL)) return;

    s_mk_last_report = now_ticks;

    int16_t delta = mk_get_move_delta();
    int16_t dx = s_mk_move_x * delta;
    int16_t dy = s_mk_move_y * delta;

    mk_send_report(dx, dy, s_mk_wheel_v, s_mk_wheel_h);
}
