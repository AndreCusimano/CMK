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
// low_power.h — automatic idle low-power state machine (Core 0).
//
// Two tiers gated on keyboard inactivity:
//   ACTIVE → LIGHT (LP_LIGHT_IDLE_MS): LEDs + 5V boost rail off, keyscan
//            throttled; BLE stays connected (instant wake).
//   LIGHT  → DEEP  (LP_DEEP_IDLE_MS):  BLE radio powered off, core sleeps until
//            a column GPIO edge (any key). Reconnects on wake.
//
// Any key event (local or remote/split) calls low_power_note_activity() and
// snaps the keyboard back to ACTIVE. The __wfi idle hook (vApplicationIdleHook
// in main.c) is the always-on baseline below all tiers.
//

#ifndef CMK_LOW_POWER_H
#define CMK_LOW_POWER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LP_ACTIVE = 0,   // full speed, LEDs on, BLE connected
    LP_LIGHT,        // LEDs + boost off, keyscan throttled, BLE connected
    LP_DEEP,         // BLE radio off, GPIO-IRQ wake
} lp_state_t;

// One-shot init (call from action_init).
void low_power_init(void);

// Register keyboard activity; resets the idle timer and wakes to ACTIVE.
// Called from keyscan on every local/remote key event. now = tick count (ms).
void low_power_note_activity(uint32_t now);

// Advance the idle state machine. Called from action_tick (keyscan loop).
void low_power_tick(uint32_t now);

// Current tier.
lp_state_t low_power_state(void);

// Keyscan delay for the current tier: KS_SCAN_INTERVAL_MS in ACTIVE,
// LP_LIGHT_SCAN_MS in LIGHT/DEEP.
uint32_t low_power_scan_interval_ms(void);

// True while in DEEP — the keyscan task uses this to drive the GPIO-IRQ sleep.
bool low_power_deep_requested(void);

#endif // CMK_LOW_POWER_H
