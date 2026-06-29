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
// raw_hid.h — Bidirectional Raw HID channel (Phase 14c)
//
// 32-byte packets via USB HID Interface 1.
// Used by WebHID for host↔keyboard communication (Phase 15).
//

#ifndef CMK_RAW_HID_H
#define CMK_RAW_HID_H

#include "config/cmk_config.h"

#include <stdint.h>
#include <stdbool.h>

#if defined(USB_ENABLE) && defined(RAW_HID_ENABLE)

// Send 32 bytes to host via Raw HID. Returns true if sent.
bool raw_hid_send(const uint8_t data[RAW_HID_BUFFER_SIZE]);

// Internal: called from tud_hid_set_report_cb() when instance == ITF_NUM_HID_RAW.
void raw_hid_receive_handler(const uint8_t *data, uint16_t len);

// Weak callback — called from ISR/task context when a packet arrives.
void raw_hid_receive_cb(const uint8_t *data, uint16_t length);

#else

static inline bool raw_hid_send(const uint8_t *data) { (void)data; return false; }
static inline void raw_hid_receive_handler(const uint8_t *data, uint16_t len) {
    (void)data; (void)len;
}

#endif // USB_ENABLE && RAW_HID_ENABLE

#endif // CMK_RAW_HID_H
