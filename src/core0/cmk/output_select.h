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
// output_select.h — HID output destination selector (Phase 14 ext)
//
// Explicit choice of where locally-generated (and forwarded upstream) HID
// reports are sent: USB or BLE. Replaces the old automatic USB-priority rule.
//
//   - CMK_OUT_USB              -> OUTPUT_USB
//   - CMK_OUT_BLE / CMK_TD0-2  -> OUTPUT_BLE
//
// The choice is persisted in eeconfig (factory default = USB). The current
// target is held in a volatile byte in shared SRAM so Core 1 (ble_manager)
// can read it directly — same cross-core pattern as usb_hid_is_active().
//

#ifndef CMK_OUTPUT_SELECT_H
#define CMK_OUTPUT_SELECT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OUTPUT_USB = 0,   // factory default (matches eeconfig zero-init)
    OUTPUT_BLE = 1,
} output_target_t;

// Load persisted target from eeconfig. Call from action_init().
void output_select_init(void);

// Set and persist the active output destination.
void output_select_set(output_target_t target);

// Current destination.
output_target_t output_select_get(void);

// Routing predicates — read by both Core 0 (send gating) and Core 1 (BLE suppression).
bool output_usb_enabled(void);   // true when target == OUTPUT_USB
bool output_ble_enabled(void);   // true when target == OUTPUT_BLE

#endif // CMK_OUTPUT_SELECT_H
