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
// raw_hid.c — Bidirectional Raw HID channel (Phase 14c)
//

#include "config/cmk_config.h"

#if defined(USB_ENABLE) && defined(RAW_HID_ENABLE)

#include "raw_hid.h"
#include "usb_descriptors.h"
#include "tusb.h"

// ---------------------------------------------------------------------------
// Receive handler (called from tud_hid_set_report_cb, Core 0 task context)
// ---------------------------------------------------------------------------
// R7: the single delivery path is the synchronous callback. The old code BOTH
// invoked the callback AND queued the packet into a ring that nothing drained
// (raw_hid_receive() had no caller) — and any future consumer of that ring
// would have re-executed every command a second time.

void raw_hid_receive_handler(const uint8_t *data, uint16_t len) {
    raw_hid_receive_cb(data, len);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool raw_hid_send(const uint8_t data[RAW_HID_BUFFER_SIZE]) {
    if (!tud_hid_n_ready(ITF_NUM_HID_RAW)) return false;
    return tud_hid_n_report(ITF_NUM_HID_RAW, 0, data, RAW_HID_BUFFER_SIZE);
}

// Weak user callback — default no-op
__attribute__((weak))
void raw_hid_receive_cb(const uint8_t *data, uint16_t length) {
    (void)data;
    (void)length;
}

#endif // USB_ENABLE && RAW_HID_ENABLE
