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
// process_consumer.c — see process_consumer.h for the design.
//
// The Report ID 4 field is a single 16-bit Consumer usage (Array): the host
// treats the reported usage as "held" until a report with 0x0000 arrives.
// Only one usage can be active at a time (last-wins): pressing a second
// consumer key replaces the first; releasing a key that is no longer the
// active one is ignored (the newer key stays held).
//

#include "process_consumer.h"
#include "keycode.h"
#include "ipc/ipc_core.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"
#include "output_select.h"

#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif

// ---------------------------------------------------------------------------
// Keycode index → 16-bit Consumer usage (HID Usage Tables, page 0x0C)
// ---------------------------------------------------------------------------

static const uint16_t consumer_usage_map[CONSUMER_KC_COUNT] = {
    0x00E2,   // KC_MUTE — Mute
    0x00E9,   // KC_VOLU — Volume Increment
    0x00EA,   // KC_VOLD — Volume Decrement
    0x00CD,   // KC_MPLY — Play/Pause
    0x00B5,   // KC_MNXT — Scan Next Track
    0x00B6,   // KC_MPRV — Scan Previous Track
    0x00B7,   // KC_MSTP — Stop
    0x006F,   // KC_BRIU — Display Brightness Increment
    0x0070,   // KC_BRID — Display Brightness Decrement
    0x029F,   // KC_MCTL — AC Desktop Show All Windows (macOS Mission Control)
    0x02A0,   // KC_LPAD — AC Soft Key Left (macOS Launchpad)
};

// ---------------------------------------------------------------------------
// State + send
// ---------------------------------------------------------------------------

static uint16_t s_current_kc;   // consumer keycode currently held (0 = none)

static void consumer_send_usage(uint16_t usage) {
    uint8_t report[2];
    report[0] = (uint8_t)(usage & 0xFF);
    report[1] = (uint8_t)(usage >> 8);
    ipc_put_consumer_report(report);
#ifdef USB_ENABLE
    if (output_usb_enabled()) {
        usb_hid_send_consumer_report(report);
    }
#endif
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_consumer(const keyevent_t *event) {
    uint16_t kc = event->keycode;
    if (!IS_CONSUMER(kc)) return true;   // pass through

    uint8_t idx = (uint8_t)CONSUMER_INDEX(kc);
    if (idx >= CONSUMER_KC_COUNT) return false;   // unmapped index → consume, no-op

    if (event->pressed) {
        s_current_kc = kc;
        consumer_send_usage(consumer_usage_map[idx]);
        DBG_KEY("[CONS] press 0x%04X → usage 0x%04X\n", kc, consumer_usage_map[idx]);
    } else if (kc == s_current_kc) {
        s_current_kc = 0;
        consumer_send_usage(0x0000);
        DBG_KEY("[CONS] release 0x%04X\n", kc);
    }
    // Release of a superseded key: ignored — the newer usage stays held.

    return false;   // consumed
}
