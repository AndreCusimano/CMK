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
// inter_half_sync.h — Inter-half link over I2C1. Two uses today:
//
//   1. Config PUSH (Phase 20): clone this half's eeconfig (settings + RGB) to
//      the other half. Arm the receiver with CMK_CFG_RECV, push with
//      CMK_CFG_PUSH. Bulk + CRC-16, applied in task context then reboot. The
//      keymap is NOT pushed — each half owns its own per-side slice, edited via
//      the proxy below.
//
//   2. Config PROXY (Phase 20b): when split config mode is ON
//      (eeconfig split_mode) and the two halves are joined over I2C1, the half
//      attached to USB serves its own 6 columns and PROXIES the peer's 6
//      columns, so the configurator sees a unified logical 4×12 board from a
//      single cable. The half without USB sits as a persistent I2C1 slave
//      "config-serve" and answers proxied CMK Config Protocol requests by
//      running cmk_config_protocol_handle() in task context.
//
// Role arbitration is automatic by USB presence: the half with a USB host is
// master/proxy on-demand; the half without USB is the config-serve slave.
//
// Compiled as no-op stubs unless INTER_HALF_LINK_ENABLE is defined.
//

#ifndef CMK_INTER_HALF_SYNC_H
#define CMK_INTER_HALF_SYNC_H

#include <stdint.h>
#include <stdbool.h>

// LED-overlay state for fn_overlay to surface link activity.
typedef enum {
    IH_LED_OFF = 0,   // idle, nothing to show
    IH_LED_ARMED,     // slave armed, waiting for a push
    IH_LED_ACTIVE,    // transfer in progress (master or slave)
    IH_LED_OK,        // last transfer succeeded
    IH_LED_ERR,       // last transfer failed / aborted
} ih_led_state_t;

// Set up I2C1 pins. Call once from action_init().
void inter_half_sync_init(void);

// Drive the non-blocking state machine. R6: runs in the dedicated ih_link task
// (created by inter_half_sync_create_task), NOT in the keyscan tick — the slave
// leg's EEPROM flushes froze the 1 kHz scan.
void inter_half_sync_tick(uint32_t now_ticks);

// Create the ih_link FreeRTOS task (R6). Call from main() with the other tasks.
void inter_half_sync_create_task(void);

// CMK_CFG_PUSH: become master and push this half's eeconfig to the peer.
void inter_half_sync_request_push(void);

// CMK_CFG_RECV: become slave and wait (with timeout) for an incoming push.
void inter_half_sync_arm_receive(void);

// Current LED-overlay state (for fn_overlay).
ih_led_state_t inter_half_sync_led_state(void);

// --- Config proxy (Phase 20b) — used by the CMK Config Protocol -------------

// Cached "is the peer half joined over I2C1?" (refreshed by *_refresh_peer()).
bool inter_half_sync_peer_joined(void);

// Ping the peer over I2C1 and update the cached joined state; returns it.
// Call from the GET_KEYBOARD_INFO handler before reporting cols/side.
bool inter_half_sync_refresh_peer(void);

// Proxy a 32-byte CMK Config Protocol request to the peer half over I2C1 and
// return its 32-byte response. Master one-shot. Returns false if the peer
// didn't answer (clears the cached joined state on failure).
bool inter_half_sync_proxy_config(const uint8_t req[32], uint8_t resp[32]);

// --- Live-state debug (GET_DIAG page 0xFF) ----------------------------------

// Raw internal state for remote debugging: [0]=ih state machine (ih_state_t),
// [1]=loop-exit liveness counter (advances while ih_task completes loops),
// [2]=slave-persistent flag, [3]=want_config_serve() as seen by the tick,
// [4]=tick-entry counter (entry vs exit distinguishes blocked-in-tick from
// never-scheduled), [5]=last heal phase reached (1=pre-idle, 2=pre-arm, 3=done).
void inter_half_sync_debug_state(uint8_t out[6]);

#endif // CMK_INTER_HALF_SYNC_H
