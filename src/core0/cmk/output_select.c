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
// output_select.c — HID output destination selector (Phase 14 ext)
//

#include "output_select.h"
#include "eeconfig.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

// Volatile: written by Core 0 (keycode handlers), read by Core 1 (ble_manager).
static volatile uint8_t s_target = OUTPUT_USB;

void output_select_init(void) {
    uint8_t t = eeconfig_read_output_target();
    s_target = (t == OUTPUT_BLE) ? OUTPUT_BLE : OUTPUT_USB;
    DBG_STATUS("[OUT] init target=%s\n", s_target == OUTPUT_USB ? "USB" : "BLE");
}

void output_select_set(output_target_t target) {
    uint8_t t = (target == OUTPUT_BLE) ? OUTPUT_BLE : OUTPUT_USB;
    if (s_target == t) return;
    s_target = t;
    eeconfig_update_output_target(t);
    DBG_STATUS("[OUT] target -> %s\n", t == OUTPUT_USB ? "USB" : "BLE");
}

output_target_t output_select_get(void) {
    return (output_target_t)s_target;
}

bool output_usb_enabled(void) {
#ifdef USB_ENABLE
    return s_target == OUTPUT_USB;
#else
    return false;   // no USB interface compiled in
#endif
}

bool output_ble_enabled(void) {
#ifdef USB_ENABLE
    return s_target == OUTPUT_BLE;
#else
    return true;    // no USB → BLE is the only possible output
#endif
}
