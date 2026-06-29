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
// diag.c — Lightweight drop/error telemetry (Refactor R1)
//

#include "diag.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

volatile diag_counters_t g_diag;

volatile uint8_t g_diag_task_alive[DIAG_TASK_COUNT];

// Name table in declaration order (kept in sync with diag_counters_t).
static const char *const s_names[DIAG_COUNTER_COUNT] = {
    "fifo_c0c1_drop",
    "kbd_ring_overflow",
    "kbd_notify_fail",
    "split_rx_drop",
    "usb_kbd_drop",
    "usb_mouse_drop",
    "usb_pb_drop",
    "ih_crc_fail",
    "ih_heal",
    "ih_proxy_retry",
    "ih_proxy_fail",
    "eeprom_err",
    "fifo_c1c0_drop",
    "hid_ring_drop",
    "split_tx_drop",
    "ipc_stale_skip",
    "relay_mismatch_drop",
    "relay_mouse_fold",
};

void diag_snapshot(uint32_t out[DIAG_COUNTER_COUNT]) {
    const volatile uint32_t *src = (const volatile uint32_t *)&g_diag;
    for (uint32_t i = 0; i < DIAG_COUNTER_COUNT; i++) out[i] = src[i];
}

void diag_dump(void) {
    uint32_t snap[DIAG_COUNTER_COUNT];
    diag_snapshot(snap);
    bool any = false;
    for (uint32_t i = 0; i < DIAG_COUNTER_COUNT; i++) {
        if (snap[i] == 0) continue;
        if (!any) { printf("[DIAG]"); any = true; }
        printf(" %s=%lu", s_names[i], (unsigned long)snap[i]);
    }
    if (any) printf("\n");
}
