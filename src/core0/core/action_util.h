/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 * Copyright (C) 2016-2026 QMK Firmware contributors
 * Copyright (C) Jun Wako and TMK Keyboard Firmware contributors
 *
 * This file contains code derived from the QMK Firmware project
 * (https://github.com/qmk/qmk_firmware) and, indirectly, from the
 * TMK Keyboard Firmware by Jun Wako, both distributed under the
 * GNU General Public License version 2.
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
// action_util.h — HID report builder and modifier state (Phase 7)
//
// Tracks modifier state (real + weak), keycode slots (6KRO),
// builds the 8-byte HID Keyboard report, and sends via IPC.
//

#ifndef CMK_ACTION_UTIL_H
#define CMK_ACTION_UTIL_H

#include <stdint.h>
#include <stdbool.h>

// Initialise modifier/keycode state.
void action_util_init(void);

// Register/unregister a basic keycode (0x00-0xFF).
// Modifier keycodes (0xE0-0xE7) go into s_real_mods;
// normal keycodes go into s_keys[] (6KRO).
void register_code(uint8_t kc);
void unregister_code(uint8_t kc);

// Modifier manipulation (8-bit HID bitmap)
void register_mods(uint8_t mods);
void unregister_mods(uint8_t mods);

// Weak modifiers — applied once with the next report, cleared by clear_weak_mods().
// Used by Mods+Key keycodes (e.g. LSFT(KC_1) = '!').
void add_weak_mods(uint8_t mods);
void clear_weak_mods(void);
// Remove ONLY the given weak-mod bits (R2). Features that added weak mods must
// release their own bits with this, not clear_weak_mods(): a global clear wipes
// the weak mods of every OTHER active feature (Auto Shift hold, Caps Word, a
// tap_code in flight, ...) — the cross-feature stuck/lost-shift class of bugs.
void del_weak_mods(uint8_t mods);

// Release everything and send an empty report.
void clear_keyboard(void);

// ---------------------------------------------------------------------------
// Non-blocking tap emitter (tap_code)
// ---------------------------------------------------------------------------
// A tap is a press immediately followed by a release. Doing register_code() +
// unregister_code() in the SAME keyscan tick is wrong: the report is flushed
// once per tick (keyscan.c), so the net 0→kc→0 collapses to "released" and
// nothing is typed; on BLE the single-slot IPC report is also last-wins. These
// helpers QUEUE taps and emit press/release on SEPARATE ticks (TAP_CODE_DELAY_MS
// apart, ≥ one BLE connection interval), pumped by tap_code_tick() from
// action_tick(). Used by Mod-Tap/Layer-Tap, Auto Shift and Tap Dance.
void tap_code_init(void);
void tap_code(uint8_t kc);                    // queue a plain tap
void tap_code_mods(uint8_t kc, uint8_t mods); // queue a tap with weak mods held during the press
void tap_code_tick(uint32_t now_ticks);       // state machine — call once per keyscan tick
bool tap_code_busy(void);                     // a tap is in flight or queued

// Build the 8-byte HID report and call ipc_put_kbd_report() if changed.
// Returns true if a report was sent.
bool send_keyboard_report(void);

// Per-tick housekeeping (R2): retries a USB keyboard send that was dropped by a
// busy endpoint, and flushes a merged-send requested by the CM task (the merge
// is built and sent ONLY here in keyscan context — single sender, no reordering).
// Call once per keyscan tick from action_tick().
void action_util_tick(uint32_t now_ticks);

// Set the upstream keyboard contribution for the USB output merge (Phase 14 ext).
// Called from the CM task when an upstream keyboard report is forwarded by Core 1
// while output == USB. Core 0 merges it (upstream-first, 6KRO) with the current
// local report and pushes the result to USB. Declaration left unguarded on purpose
// (a missing USB_ENABLE in the caller must not silently drop the call).
void action_util_set_usb_upstream_kbd(const uint8_t up[8]);

// Returns true if state is dirty (register/unregister happened since last send).
bool action_util_is_dirty(void);

// Get current real modifier state (8-bit HID bitmap).
uint8_t get_real_mods(void);

#endif // CMK_ACTION_UTIL_H
