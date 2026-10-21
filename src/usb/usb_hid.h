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
// usb_hid.h — TinyUSB HID device API (Phase 14a)
//
// Manages the USB device task and provides report send functions.
// USB has priority over BLE — when active, BLE report sending is suppressed.
//

#ifndef CMK_USB_HID_H
#define CMK_USB_HID_H

#include "config/cmk_config.h"

#ifdef USB_ENABLE

#include <stdint.h>
#include <stdbool.h>

// Create the FreeRTOS task for tud_task(). Call from main() before vTaskStartScheduler().
void usb_hid_task_create(void);

// Allow the USB task to call tusb_init() and start. MUST be called only after
// Core 1 has finished CYW43/BTstack bringup (IPC_BLE_WORKING), otherwise the
// concurrent USB init hangs Core 1 during BT firmware download.
void usb_hid_start(void);

// Send HID reports via USB (no-op if USB not ready).
// Keyboard returns true only when TinyUSB accepted the report — callers use it
// to commit their dedup state only on confirmed delivery (R2).
bool usb_hid_send_kbd_report(const uint8_t report[8]);
void usb_hid_send_mouse_report(const uint8_t report[7]);
void usb_hid_send_pb_report(const uint8_t report[4]);
void usb_hid_send_consumer_report(const uint8_t report[2]);

// Returns true if USB is mounted and the HID interface is ready.
// Used by ble_manager to suppress BLE report sending.
bool usb_hid_is_active(void);

#else

// Stubs when USB_ENABLE is not defined
static inline void usb_hid_task_create(void) {}
static inline void usb_hid_start(void) {}
static inline bool usb_hid_send_kbd_report(const uint8_t report[8]) { (void)report; return false; }
static inline void usb_hid_send_mouse_report(const uint8_t report[7]) { (void)report; }
static inline void usb_hid_send_pb_report(const uint8_t report[4]) { (void)report; }
static inline void usb_hid_send_consumer_report(const uint8_t report[2]) { (void)report; }
static inline bool usb_hid_is_active(void) { return false; }

#endif // USB_ENABLE

#endif // CMK_USB_HID_H
