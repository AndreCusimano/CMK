/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 *
 * CMK is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation, version 2.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

//
// cmk_split_uuids.h — 128-bit UUIDs for the CMK-Split GATT service (Phase 18).
//
// The split keyboard's raw-matrix transport between the two wireless halves. The
// MASTER (Central) discovers this service on the SLAVE (Peripheral) and subscribes
// to matrix_event; the SLAVE notifies key changes; matrix_snapshot re-syncs full
// state; side_id tells the master which physical half the slave is.
//
// Keep these in sync with the PRIMARY_SERVICE / CHARACTERISTIC lines in
// hid_keyboard.gatt (same UUID strings, dashes → the generated handle macros).
//
//   service          c3a50001-7a4e-4c1b-9b21-000000000001
//   matrix_event     c3a50002-7a4e-4c1b-9b21-000000000001   NOTIFY,        1 B
//   matrix_snapshot  c3a50003-7a4e-4c1b-9b21-000000000001   READ + NOTIFY, 3 B
//   side_id          c3a50004-7a4e-4c1b-9b21-000000000001   READ,          1 B
//   split_mode       c3a50005-7a4e-4c1b-9b21-000000000001   READ + NOTIFY, 1 B
//   rgb_sync         c3a50006-7a4e-4c1b-9b21-000000000001   WRITE(_NR),  ≤16 B
//
// rgb_sync is the ONLY MASTER→SLAVE channel (Phase 24): the central writes the
// RGB effect state (and, Phase 24b, remote key hits) so both halves render the
// same animation in phase. Write Without Response (no ACK round-trip) on the
// opposite direction of matrix_event → it never contends the HID report path.
//
// matrix_event byte: bit7 = pressed, bits5:3 = row (0..3), bits2:0 = col (0..5).
// split_mode byte: this half's eeconfig split_mode (strict ON/ON || OFF/OFF matching).
//

#ifndef CMK_SPLIT_UUIDS_H
#define CMK_SPLIT_UUIDS_H

#include <stdint.h>

// Big-endian 16-byte forms for the gatt_client discovery / match API.
static const uint8_t cmk_split_service_uuid[16] = {
    0xc3,0xa5,0x00,0x01, 0x7a,0x4e, 0x4c,0x1b, 0x9b,0x21, 0x00,0x00,0x00,0x00,0x00,0x01 };
static const uint8_t cmk_split_matrix_event_uuid[16] = {
    0xc3,0xa5,0x00,0x02, 0x7a,0x4e, 0x4c,0x1b, 0x9b,0x21, 0x00,0x00,0x00,0x00,0x00,0x01 };
static const uint8_t cmk_split_matrix_snapshot_uuid[16] = {
    0xc3,0xa5,0x00,0x03, 0x7a,0x4e, 0x4c,0x1b, 0x9b,0x21, 0x00,0x00,0x00,0x00,0x00,0x01 };
static const uint8_t cmk_split_side_id_uuid[16] = {
    0xc3,0xa5,0x00,0x04, 0x7a,0x4e, 0x4c,0x1b, 0x9b,0x21, 0x00,0x00,0x00,0x00,0x00,0x01 };
static const uint8_t cmk_split_mode_uuid[16] = {
    0xc3,0xa5,0x00,0x05, 0x7a,0x4e, 0x4c,0x1b, 0x9b,0x21, 0x00,0x00,0x00,0x00,0x00,0x01 };
static const uint8_t cmk_split_rgb_sync_uuid[16] = {
    0xc3,0xa5,0x00,0x06, 0x7a,0x4e, 0x4c,0x1b, 0x9b,0x21, 0x00,0x00,0x00,0x00,0x00,0x01 };

// The matrix_event byte encoding (CMK_SPLIT_EVT*) lives in ipc/ipc_core.h so both
// cores (keyscan on Core 0, BLE on Core 1) share one definition.

#endif // CMK_SPLIT_UUIDS_H
