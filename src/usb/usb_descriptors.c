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
// usb_descriptors.c — TinyUSB descriptor callbacks (Phase 14)
//
// USB HID composite: Keyboard+Mouse+PB (Report IDs 1/2/3) + Raw HID (32B).
//

#include "config/cmk_config.h"

#ifdef USB_ENABLE

#include "usb_descriptors.h"
#include "board_side.h"
#include "tusb.h"
#if DEBUG_USB_SERIAL
#include "pico/unique_id.h"
#endif

#ifdef OS_DETECTION_ENABLE
#include "os_detection.h"
#include "pico/time.h"
#endif

#include <string.h>

// ---------------------------------------------------------------------------
// HID Report Descriptor — Interface 0 (Keyboard + Mouse + PB + Consumer)
// ---------------------------------------------------------------------------
// MUST stay byte-for-byte identical to the BLE hid_descriptor[] in
// ble_manager.c — if one changes, update BOTH.

const uint8_t desc_hid_report_kbd[] = {
    // --- Keyboard — Report ID 1 (8 bytes: modifier, reserved, 6 keycodes) ---
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xa1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    // Modifier keys: 8 bits
    0x75, 0x01, 0x95, 0x08, 0x05, 0x07, 0x19, 0xe0,
    0x29, 0xe7, 0x15, 0x00, 0x25, 0x01, 0x81, 0x02,
    // Reserved byte
    0x75, 0x08, 0x95, 0x01, 0x81, 0x03,
    // LED output (5 bits + 3 pad)
    0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01,
    0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x03,
    // Keycodes: 6 bytes
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0xff,
    0x05, 0x07, 0x19, 0x00, 0x29, 0xff, 0x81, 0x00,
    0xc0,              // End Collection

    // --- Mouse — Report ID 2 (7 bytes) ---
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xa1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xa1, 0x00,        //   Collection (Physical)
    0x85, 0x02,        //   Report ID (2)
    // Buttons: 5 bits
    0x05, 0x09, 0x19, 0x01, 0x29, 0x05,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x05, 0x81, 0x02,
    // Padding: 3 bits
    0x75, 0x03, 0x95, 0x01, 0x81, 0x03,
    // X, Y: 16-bit signed relative
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
    0x16, 0x01, 0x80, 0x26, 0xff, 0x7f,
    0x75, 0x10, 0x95, 0x02, 0x81, 0x06,
    // Wheel: 8-bit signed
    0x09, 0x38, 0x15, 0x81, 0x25, 0x7f,
    0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
    // Pan: 8-bit signed (Consumer AC Pan)
    0x05, 0x0c, 0x0a, 0x38, 0x02,
    0x15, 0x81, 0x25, 0x7f,
    0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
    0xc0,              //   End Collection (Physical)
    0xc0,              // End Collection (Application)

    // --- Programmable Buttons — Report ID 3 (4 bytes: 32-bit bitmask) ---
    // Vendor page 0xFF00, aligned to the BLE descriptor (bug #6): the old USB
    // version used Consumer+Button pages, diverging from the macOS-safe BLE one
    // (a second Button-page range breaks the mouse clicks on macOS).
    0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01,
    0x85, 0x03,
    0x19, 0x01, 0x29, 0x20,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x20, 0x75, 0x01, 0x81, 0x02,
    0xC0,

    // --- Consumer Control — Report ID 4 (2 bytes: one 16-bit consumer usage) ---
    // Media/volume/brightness/screen keys (bug #6). Array field: the host holds
    // the reported usage until a 0x0000 report. Max 0x2FF covers AC Desktop
    // Show All Windows (0x29F) and AL Launchpad (0x2A0) for macOS.
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x04,        //   Report ID (4)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x02,  //   Logical Maximum (0x2FF)
    0x19, 0x00,        //   Usage Minimum (0)
    0x2A, 0xFF, 0x02,  //   Usage Maximum (0x2FF)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data, Array, Abs)
    0xC0,              // End Collection
};

const uint16_t desc_hid_report_kbd_len = sizeof(desc_hid_report_kbd);

// ---------------------------------------------------------------------------
// HID Report Descriptor — Interface 1 (Raw HID, 32B bidirectional)
// ---------------------------------------------------------------------------

#ifdef RAW_HID_ENABLE
// Raw HID report descriptor — Usage Page 0xFF60 / Usage 0x61 (QMK/VIA convention,
// matches CMK_Protocol.md and the WebHID filter). NOT the TinyUSB
// TUD_HID_REPORT_DESC_GENERIC_INOUT macro, which emits Usage Page 0xFF00 — that
// would make the WebHID `usagePage: 0xFF60` filter find nothing.
// No report ID (report id 0). 32-byte IN (usage 0x62) + 32-byte OUT (usage 0x63).
const uint8_t desc_hid_report_raw[] = {
    0x06, 0x60, 0xFF,             // Usage Page (Vendor Defined 0xFF60)
    0x09, 0x61,                   // Usage (0x61)
    0xA1, 0x01,                   // Collection (Application)
    0x09, 0x62,                   //   Usage (0x62) — data IN (device → host)
    0x15, 0x00,                   //   Logical Minimum (0)
    0x26, 0xFF, 0x00,             //   Logical Maximum (255)
    0x95, RAW_HID_BUFFER_SIZE,    //   Report Count (32)
    0x75, 0x08,                   //   Report Size (8)
    0x81, 0x02,                   //   Input (Data, Variable, Absolute)
    0x09, 0x63,                   //   Usage (0x63) — data OUT (host → device)
    0x15, 0x00,                   //   Logical Minimum (0)
    0x26, 0xFF, 0x00,             //   Logical Maximum (255)
    0x95, RAW_HID_BUFFER_SIZE,    //   Report Count (32)
    0x75, 0x08,                   //   Report Size (8)
    0x91, 0x02,                   //   Output (Data, Variable, Absolute)
    0xC0,                         // End Collection
};

const uint16_t desc_hid_report_raw_len = sizeof(desc_hid_report_raw);
#endif

// ---------------------------------------------------------------------------
// Device Descriptor
// ---------------------------------------------------------------------------

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
#if DEBUG_USB_SERIAL
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
#else
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
#endif
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

// ---------------------------------------------------------------------------
// Configuration Descriptor
// ---------------------------------------------------------------------------

#if DEBUG_USB_SERIAL
#define DEBUG_CDC_DESC_LEN TUD_CDC_DESC_LEN
#else
#define DEBUG_CDC_DESC_LEN 0
#endif
#ifdef RAW_HID_ENABLE
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_INOUT_DESC_LEN + DEBUG_CDC_DESC_LEN)
#else
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + DEBUG_CDC_DESC_LEN)
#endif

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface 0: HID Keyboard+Mouse+PB — composite, multiplexed by Report ID.
    // Declared NON-boot (bInterfaceSubClass=0, bInterfaceProtocol=0). A USB boot
    // keyboard MUST send fixed 8-byte reports with NO Report ID, but this interface
    // tags every report with its Report ID (1=kbd, 2=mouse, 3=PB). If we declared it
    // as a boot keyboard, macOS keeps the interface in Boot Protocol and then ignores
    // our Report-ID-prefixed reports — the device enumerates but never types. (BLE
    // HOGP has no boot-protocol concept, which is why the same report map works there.)
    // HID_ITF_PROTOCOL_NONE forces the host to use Report Protocol and parse the
    // report descriptor. See docs/fixes/USB.md.
    TUD_HID_DESCRIPTOR(ITF_NUM_HID_KBD, 0, HID_ITF_PROTOCOL_NONE,
                       sizeof(desc_hid_report_kbd), EP_HID_KBD_IN, CFG_TUD_HID_EP_BUFSIZE, 1),

#ifdef RAW_HID_ENABLE
    // Interface 1: HID Raw (IN+OUT, 32B)
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID_RAW, 0, HID_ITF_PROTOCOL_NONE,
                             sizeof(desc_hid_report_raw), EP_HID_RAW_OUT, EP_HID_RAW_IN,
                             CFG_TUD_HID_EP_BUFSIZE, 1),
#endif
#if DEBUG_USB_SERIAL
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EP_CDC_NOTIFY, 8, EP_CDC_OUT, EP_CDC_IN, 64),
#endif
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// ---------------------------------------------------------------------------
// HID Report Descriptor Callback — per instance
// ---------------------------------------------------------------------------

uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    if (instance == ITF_NUM_HID_KBD) return desc_hid_report_kbd;
#ifdef RAW_HID_ENABLE
    if (instance == ITF_NUM_HID_RAW) return desc_hid_report_raw;
#endif
    return NULL;
}

// ---------------------------------------------------------------------------
// String Descriptors
// ---------------------------------------------------------------------------

static const char* const string_desc_arr[] = {
    [0] = (const char[]){0x09, 0x04},  // English (US)
    [1] = USB_MANUFACTURER,
    [2] = CMK_NAME_LEFT,               // fallback only — index 2 (product) is
                                       // overridden at runtime by board_side_name()
    [3] = "000001",
#if DEBUG_USB_SERIAL
    [4] = "CMK Debug Serial",
#endif
};

static uint16_t s_desc_str[32 + 1];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&s_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        // Index 2 = product string: per-side name decided at boot.
        const char* str = (index == 2) ? board_side_name() : string_desc_arr[index];
#if DEBUG_USB_SERIAL
        char serial[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
        if (index == 3) {
            pico_get_unique_board_id_string(serial, sizeof(serial));
            str = serial;
        }
#endif
        for (chr_count = 0; chr_count < 31 && str[chr_count]; chr_count++) {
            s_desc_str[1 + chr_count] = str[chr_count];
        }
    }

    s_desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

#ifdef OS_DETECTION_ENABLE
    // Record this string descriptor request for OS detection.
    // wLength is not directly available here — we pass 0xFF as placeholder.
    // Full wLength tracking requires a lower-level TinyUSB hook (future).
    uint32_t now = to_ms_since_boot(get_absolute_time());
    os_detection_record_wlength(2 * chr_count + 2, now);
#endif

    return s_desc_str;
}

#endif // USB_ENABLE
