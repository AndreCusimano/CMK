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
// process_programmable_button.c — 32 Programmable HID Buttons (Phase 12)
//
// Each PB_N keycode maps to bit (N-1) in a 32-bit bitmask.
// Press sets the bit, release clears it. Report sent via IPC as Report ID 3.
//

#include "process_programmable_button.h"
#include "keycode.h"
#include "ipc/ipc_core.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"
#include "output_select.h"

#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint32_t s_pb_state;  // bitmask: bit N = button N+1 pressed

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void pb_send_report(void) {
    // 4-byte report: little-endian 32-bit bitmask
    uint8_t report[4];
    report[0] = (uint8_t)(s_pb_state & 0xFF);
    report[1] = (uint8_t)((s_pb_state >> 8) & 0xFF);
    report[2] = (uint8_t)((s_pb_state >> 16) & 0xFF);
    report[3] = (uint8_t)((s_pb_state >> 24) & 0xFF);
    ipc_put_pb_report(report);
#ifdef USB_ENABLE
    // Programmable buttons are local-only; route to USB when USB is selected.
    if (output_usb_enabled()) {
        usb_hid_send_pb_report(report);
    }
#endif
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void programmable_button_init(void) {
    s_pb_state = 0;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_programmable_button(const keyevent_t *event) {
    uint16_t kc = event->keycode;
    if (!IS_PROG_BUTTON(kc)) return true;  // pass through

    uint8_t btn = (uint8_t)(kc - PB_1);  // 0-31
    uint32_t mask = 1u << btn;

    if (event->pressed) {
        s_pb_state |= mask;
    } else {
        s_pb_state &= ~mask;
    }

    pb_send_report();
    DBG_KEY("[PB] btn=%d %s state=0x%08lX\n",
            btn + 1, event->pressed ? "press" : "release",
            (unsigned long)s_pb_state);

    return false;  // consumed
}
