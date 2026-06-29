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
// cmk_config_protocol.h — CMK Config Protocol over Raw HID (Phase 15b).
//
// Implements the host↔device command set described in CMK_Config_Protocol.md
// (request/response, 32-byte reports). The handler overrides the weak
// raw_hid_receive_cb() in raw_hid.c; it runs synchronously in the Core 0 USB
// task and drives dynamic_keymap, eeconfig and the RGB engine.
//
// Compiled only when RAW_HID_ENABLE is defined.
//

#ifndef CMK_CONFIG_PROTOCOL_H
#define CMK_CONFIG_PROTOCOL_H

#include <stdint.h>

// Command ids (CMK_Config_Protocol.md §11).
enum {
    CMD_GET_PROTOCOL_VERSION = 0x01,
    CMD_GET_KEYBOARD_INFO    = 0x02,
    CMD_GET_FW_VERSION       = 0x03,   // returns firmware semver: major, minor, patch
    CMD_GET_DIAG             = 0x04,   // R1 telemetry: drop/error counters (paged)
    CMD_EEPROM_RESET         = 0x0D,
    CMD_COMMIT               = 0x0E,
    CMD_BOOTLOADER           = 0x0F,
    CMD_GET_KEYMAP_KEYCODE   = 0x11,
    CMD_SET_KEYMAP_KEYCODE   = 0x12,
    CMD_GET_KEYMAP_BUFFER    = 0x13,
    CMD_SET_KEYMAP_BUFFER    = 0x14,
    CMD_PUSH_KEYMAP_TO_PEER  = 0x15,   // force-copy the full local keymap → peer (I2C1)
    CMD_RESET_KEYMAP         = 0x1F,
    CMD_GET_RGB_CONFIG       = 0x21,
    CMD_SET_RGB_CONFIG       = 0x22,
    CMD_SET_RGB_FIELD        = 0x23,
    CMD_GET_SETTINGS         = 0x31,
    CMD_SET_SETTING          = 0x32,
};

// Status codes (response byte 1).
enum {
    ST_OK              = 0x00,
    ST_UNKNOWN_CMD     = 0x01,
    ST_BAD_PARAMS      = 0x02,
    ST_OUT_OF_RANGE    = 0x03,
    ST_NOT_SUPPORTED   = 0x04,
    ST_LOCKED          = 0x05,
    ST_EEPROM          = 0x06,
    ST_PEER_UNAVAIL    = 0x07,   // split proxy: peer half not reachable over I2C1
    ST_OK_LOCAL_ONLY   = 0x08,   // R6: BOTH-targeted write applied LOCALLY only
                                 // (peer not joined) — clients must not assume
                                 // the two halves are in sync after this status.
};

// Command target — absolute board side (protocol v3: uniform on EVERY command at
// req[1]). Matches GET_KEYBOARD_INFO `side` (0=Left, 1=Right). BOTH = local + peer
// (writes apply peer-first-then-local); BOTH is invalid for reads (one response).
enum {
    CFG_TGT_LEFT  = 0,
    CFG_TGT_RIGHT = 1,
    CFG_TGT_BOTH  = 2,
};
// Back-compat aliases (the v2 RGB target used these names).
#define RGB_TGT_LEFT  CFG_TGT_LEFT
#define RGB_TGT_RIGHT CFG_TGT_RIGHT
#define RGB_TGT_BOTH  CFG_TGT_BOTH

// v3 framing: req[0]=command, req[1]=target (CFG_TGT_*), req[2..]=args.
//             resp[0]=echoed command, resp[1]=status, resp[2..]=data (no target).
//
// USB-origin entry: parses the target and routes to the local half, the peer half
// (via the I2C1 config proxy), or both (peer-first-then-local), then returns the
// 32-byte response. System commands (version/info/commit/bootloader/push) ignore
// the target and run locally.
void cmk_config_protocol_handle(const uint8_t *req, uint8_t *resp);

// Peer-side entry point: handle a 32-byte request proxied from the other half over
// I2C1. Runs the LOCAL handler directly (never re-routes), so a BOTH/peer-targeted
// request from the USB half resolves to a single local apply and cannot loop.
void cmk_config_protocol_handle_peer(const uint8_t *req, uint8_t *resp);

#endif // CMK_CONFIG_PROTOCOL_H
