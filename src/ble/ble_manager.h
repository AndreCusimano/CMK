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
// ble_manager.h — BLE dual-capability manager
//
// Each device can advertise (be found) and/or scan (find others) independently,
// triggered by FN+GP12 (ADV) / FN+GP13 (SCAN) combos via IPC from Core 0.
//
// All functions must be called from Core 1 ONLY (BTstack async_context).
//

#ifndef CMK_BLE_MANAGER_H
#define CMK_BLE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Connection state
// ---------------------------------------------------------------------------

typedef enum {
    PERIPH_IDLE,
    PERIPH_ADVERTISING,         // general undirected ADV
    PERIPH_ADV_DIRECTED,        // directed ADV high duty (1.28s hardware timeout)
    PERIPH_CONNECTED,
} periph_state_t;

typedef enum {
    CENTRAL_IDLE,
    CENTRAL_SCANNING,
    CENTRAL_CONNECTING,
    CENTRAL_DISCOVERING,
    CENTRAL_CONNECTED,
} central_state_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Initialise BTstack layers and register profiles.
// Call ONCE before hci_power_control(HCI_POWER_ON).
void ble_manager_init(void);

// Handle an IPC command from Core 0 (called with async_context lock held).
void ble_manager_handle_ipc(uint32_t msg);

// R3: called every IPC poll tick (5 ms) from core1_entry, independent of the
// FIFO. Drains the kbd ring + mouse/PB slots (a lost coalesced notify becomes
// ≤5 ms of latency instead of a permanent stall) and publishes the level-based
// BLE status snapshot for the CM task's reconciliation.
void ble_manager_ipc_slots_poll(void);

periph_state_t  ble_manager_periph_state(void);
central_state_t ble_manager_central_state(void);

#endif // CMK_BLE_MANAGER_H
