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
// hid_parser.h — HID report descriptor parser + mouse report decoder
//
// Parses a raw HID descriptor into a flat field map, then decodes any
// upstream mouse report into a normalised hid_mouse_report_t, regardless
// of the upstream's internal byte layout.
//
// Standard downstream mouse output (7 bytes, no Report ID byte):
//   Byte 0  Buttons  (bit0=L, bit1=R, bit2=M, bit3=Back, bit4=Fwd)
//   Byte 1  X_L      (int16 LE, low byte)
//   Byte 2  X_H
//   Byte 3  Y_L      (int16 LE, low byte)
//   Byte 4  Y_H
//   Byte 5  Wheel    (int8, vertical scroll)
//   Byte 6  Pan      (int8, horizontal scroll / AC Pan)
//

#ifndef CMK_HID_PARSER_H
#define CMK_HID_PARSER_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Usage helpers:  combined = (page << 16) | id
// ---------------------------------------------------------------------------
#define HID_USAGE(page, id)    (((uint32_t)(page) << 16) | ((uint32_t)(id) & 0xFFFFu))
#define HID_USAGE_PAGE(u)      ((uint16_t)((u) >> 16))
#define HID_USAGE_ID(u)        ((uint16_t)((u) & 0xFFFFu))

// Mouse usages we decode
#define HID_USAGE_X            HID_USAGE(0x01u, 0x30u)   // Generic Desktop: X
#define HID_USAGE_Y            HID_USAGE(0x01u, 0x31u)   // Generic Desktop: Y
#define HID_USAGE_WHEEL        HID_USAGE(0x01u, 0x38u)   // Generic Desktop: Wheel (vertical)
#define HID_USAGE_DIAL         HID_USAGE(0x01u, 0x37u)   // Generic Desktop: Dial (H-scroll on some mice)
#define HID_USAGE_AC_PAN       HID_USAGE(0x0Cu, 0x0238u) // Consumer: AC Pan (horizontal scroll)
#define HID_USAGE_BTN_PAGE     0x09u                      // Button usage page

// HID Input item flags (bit positions in Input data byte)
#define HID_INPUT_CONSTANT     (1u << 0)  // 0=Data,     1=Constant
#define HID_INPUT_VARIABLE     (1u << 1)  // 0=Array,    1=Variable
#define HID_INPUT_RELATIVE     (1u << 2)  // 0=Absolute, 1=Relative

// ---------------------------------------------------------------------------
// Parsed field map
// ---------------------------------------------------------------------------
#define HID_FIELD_MAX 48

typedef struct {
    uint8_t  report_id;
    uint32_t bit_offset;    // bit position within report Input stream (not counting report_id byte in BLE)
    uint8_t  bit_count;     // total bits for this entry
    uint32_t usage_first;   // first (or only) usage in HID_USAGE format
    uint32_t usage_last;    // last usage for aggregate fields; == usage_first for single fields
    int32_t  logical_min;
    int32_t  logical_max;
    uint8_t  input_flags;
} hid_field_t;

typedef struct {
    hid_field_t fields[HID_FIELD_MAX];
    uint8_t     count;
} hid_report_map_t;

// ---------------------------------------------------------------------------
// Normalised mouse report
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t buttons;  // bitmask: bit0=L, bit1=R, bit2=M, bit3=Back, bit4=Fwd
    int16_t x;
    int16_t y;
    int8_t  wheel;    // vertical scroll
    int8_t  pan;      // horizontal scroll
} hid_mouse_report_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Parse a raw HID report descriptor into a field map.
// Returns number of Input fields parsed (≥0), or -1 if HID_FIELD_MAX exceeded.
int hid_parse_descriptor(const uint8_t *desc, uint16_t len, hid_report_map_t *out);

// Extract bit_count bits starting at bit_offset from data (LSB-first, LE bytes).
// Sign-extends the result when logical_min < 0.
int32_t hid_extract_bits(const uint8_t *data, uint16_t data_len,
                          uint32_t bit_offset, uint8_t bit_count,
                          int32_t logical_min);

// Decode a raw upstream mouse notification into a normalised hid_mouse_report_t.
//   map       — field map produced by hid_parse_descriptor() for the upstream device
//   report_id — as returned by BTstack gattservice_subevent_hid_report_get_report_id()
//   data      — raw GATT notification bytes (no report_id byte prepended)
//   data_len  — number of bytes in data
// Returns true if at least one mouse field was decoded successfully.
bool hid_decode_mouse(const hid_report_map_t *map, uint8_t report_id,
                      const uint8_t *data, uint16_t data_len,
                      hid_mouse_report_t *out);

// Pack hid_mouse_report_t into the standard 7-byte downstream format:
//   [buttons][X_L][X_H][Y_L][Y_H][wheel][pan]
void hid_pack_mouse(const hid_mouse_report_t *in, uint8_t out[7]);

// True if the given report carries mouse motion (X or Y usage) — classifies
// upstream reports by content instead of assuming our own Report ID numbering.
bool hid_report_is_mouse(const hid_report_map_t *map, uint8_t report_id);

#endif // CMK_HID_PARSER_H
