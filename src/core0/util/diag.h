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
// diag.h — Lightweight drop/error telemetry (Refactor R1)
//
// Free-running counters for every silent-drop path in the firmware. Each field
// has a SINGLE writer core (no cross-core RMW); increments are plain ++ on a
// volatile (multiple tasks on the same core may rarely lose one count — fine
// for diagnostics). Read via diag_dump() (UART, DEBUG_DIAG) or the GET_DIAG
// config command.
//

#pragma once

#include <stdint.h>

typedef struct {
    // --- Core 0 writers ---
    uint32_t fifo_c0c1_drop;     // ipc_push_to_core1 failed (HW FIFO full) — command lost
    uint32_t kbd_ring_overflow;  // kbd ring full: oldest report dropped
    uint32_t kbd_notify_fail;    // kbd ring empty→nonempty notify lost (B3)
    uint32_t split_rx_drop;      // master: remote-event ring full (event lost)
    uint32_t usb_kbd_drop;       // USB send retry (!active/!ready), or keyboard queue full
    uint32_t usb_mouse_drop;     // usb_hid_send_mouse skipped
    uint32_t usb_pb_drop;        // usb_hid_send_pb skipped
    uint32_t ih_crc_fail;        // I2C1 slave: frame rejected (CRC/len/type)
    uint32_t ih_heal;            // I2C1 slave peripheral re-armed
    uint32_t ih_proxy_retry;     // I2C1 master proxy: per-transaction retries
    uint32_t ih_proxy_fail;      // I2C1 master proxy: failed after all retries
    uint32_t eeprom_err;         // EEPROM I2C transfer errors
    // --- Core 1 writers ---
    uint32_t fifo_c1c0_drop;     // ipc_push_to_core0 failed (HW FIFO full) — event lost
    uint32_t hid_ring_drop;      // upstream HID report ring full (report lost)
    uint32_t split_tx_drop;      // slave: split TX ring full (event lost)
    uint32_t ipc_stale_skip;     // FIFO mouse/PB notify already consumed; excludes ordinary empty polling
    uint32_t relay_mismatch_drop; // sibling HID dropped: split_mode mismatch (strict matching)
    uint32_t relay_mouse_fold;   // mouse relay queue full: oldest entry folded into successor
} diag_counters_t;

extern volatile diag_counters_t g_diag;

#define DIAG_INC(field) do { g_diag.field++; } while (0)

// Number of uint32 counters exported by diag_snapshot() / GET_DIAG.
#define DIAG_COUNTER_COUNT  (sizeof(diag_counters_t) / sizeof(uint32_t))

// Copy all counters (declaration order) into out[]. Core 0 task context.
void diag_snapshot(uint32_t out[DIAG_COUNTER_COUNT]);

// printf every non-zero counter (Core 0 only — UART). No-op when all zero.
void diag_dump(void);

// --- Task liveness (GET_DIAG page 0xFF) --------------------------------------
// One free-running byte per Core 0 task, bumped once per task loop. Two reads
// of the live-state page tell which tasks the scheduler is actually running.
enum { DIAG_TASK_CM, DIAG_TASK_LED, DIAG_TASK_PM, DIAG_TASK_USB, DIAG_TASK_KS,
       DIAG_TASK_COUNT };
extern volatile uint8_t g_diag_task_alive[DIAG_TASK_COUNT];
#define DIAG_TASK_ALIVE(t) do { g_diag_task_alive[t]++; } while (0)
