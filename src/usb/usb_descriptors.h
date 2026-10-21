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
// usb_descriptors.h — USB descriptor enums and externs (Phase 14)
//

#ifndef CMK_USB_DESCRIPTORS_H
#define CMK_USB_DESCRIPTORS_H

#include "config/cmk_config.h"
#include "config/debug_config.h"

#ifdef USB_ENABLE

#include <stdint.h>

// ---------------------------------------------------------------------------
// Interface & endpoint numbering
// ---------------------------------------------------------------------------

enum {
    ITF_NUM_HID_KBD = 0,   // HID: Keyboard + Mouse + Programmable Button
#ifdef RAW_HID_ENABLE
    ITF_NUM_HID_RAW,        // HID: Raw HID (IN+OUT, 32B, WebHID)
#endif
#if DEBUG_USB_SERIAL
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
#endif
    ITF_NUM_TOTAL
};

// Endpoint addresses (IN endpoints have bit 7 set, OUT do not)
#define EP_HID_KBD_IN    0x81
#define EP_HID_RAW_IN    0x82
#define EP_HID_RAW_OUT   0x02
#define EP_CDC_NOTIFY    0x83
#define EP_CDC_OUT       0x04
#define EP_CDC_IN        0x84

// ---------------------------------------------------------------------------
// Descriptor externs (defined in usb_descriptors.c)
// ---------------------------------------------------------------------------

extern const uint8_t desc_hid_report_kbd[];
extern const uint16_t desc_hid_report_kbd_len;

extern const uint8_t desc_hid_report_raw[];
extern const uint16_t desc_hid_report_raw_len;

#endif // USB_ENABLE

#endif // CMK_USB_DESCRIPTORS_H
