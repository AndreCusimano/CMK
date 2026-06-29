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
// action_util.c — HID report builder and modifier state (Phase 7)
//

#include "config/cmk_config.h"   // MUST come before any #ifdef USB_ENABLE — defines USB_ENABLE.
                                 // Without it USB_ENABLE is invisible here and the USB send below
                                 // is silently preprocessed out (compiles clean, never sends).
#include "action_util.h"
#include "keycode.h"
#include "process_secure.h"
#include "output_select.h"
#include "ipc/ipc_core.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"   // pdMS_TO_TICKS (tap_code timing); taskENTER_CRITICAL (USB merge)
#include "task.h"

#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif

#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

#define KEY_SLOTS 6   // 6KRO

static uint8_t s_real_mods;            // persistent modifier bits (HID byte)
static uint8_t s_weak_mods;            // one-shot modifier bits (cleared after send)
static uint8_t s_keys[KEY_SLOTS];      // active keycodes
static uint8_t s_prev_report[8];       // last sent report (for dedup)
static bool    s_dirty;                // true if state changed since last send

#ifdef USB_ENABLE
// USB output merge (Phase 14 ext): on USB the local keyboard goes Core 0 → USB
// directly (zero latency), but an upstream CMK keyboard must be merged in. Core 1
// forwards only the upstream contribution; Core 0 keeps the last local report and
// combines the two (upstream-first, 6KRO — same rule as the BLE merge on Core 1).
// Touched by both the keyscan task (local) and the CM task (upstream) → guarded by
// a critical section. The BLE merge path on Core 1 is untouched.
static uint8_t s_local_report[8];      // last local-only report (snapshot for merge)
static uint8_t s_usb_upstream_kbd[8];  // upstream kbd contribution (set by CM task)
static uint8_t s_usb_prev[8];          // dedup of last merged report pushed to USB
#endif

// tap_code queue (non-blocking tap emitter — see header). Press and release are
// emitted on separate ticks so both USB and the last-wins BLE IPC slot observe a
// discrete down→up; consecutive taps serialise through the queue.
typedef struct { uint8_t kc; uint8_t mods; } tap_entry_t;
static tap_entry_t s_tapq[TAP_CODE_QUEUE_SIZE];
static uint8_t     s_tapq_head;        // next slot to pop
static uint8_t     s_tapq_count;       // queued taps
static bool        s_tap_down;         // current tap's press is registered
static uint8_t     s_tap_cur_kc;
static uint8_t     s_tap_cur_mods;
static uint32_t    s_tap_timer;        // tick of the last press/release (0 = idle, start now)

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void action_util_init(void) {
    s_real_mods = 0;
    s_weak_mods = 0;
    memset(s_keys, 0, sizeof(s_keys));
    memset(s_prev_report, 0, sizeof(s_prev_report));
    s_dirty = false;
#ifdef USB_ENABLE
    memset(s_local_report, 0, sizeof(s_local_report));
    memset(s_usb_upstream_kbd, 0, sizeof(s_usb_upstream_kbd));
    memset(s_usb_prev, 0, sizeof(s_usb_prev));
#endif
    tap_code_init();
}

// ---------------------------------------------------------------------------
// Register / unregister
// ---------------------------------------------------------------------------

void register_code(uint8_t kc) {
    if (IS_HID_MOD(kc)) {
        s_real_mods |= MOD_BIT(kc);
        s_dirty = true;
        return;
    }

    if (kc == 0) return;

    // Check if already registered
    for (int i = 0; i < KEY_SLOTS; i++) {
        if (s_keys[i] == kc) return;
    }

    // Find empty slot
    for (int i = 0; i < KEY_SLOTS; i++) {
        if (s_keys[i] == 0) {
            s_keys[i] = kc;
            s_dirty = true;
            return;
        }
    }
    // No slot available — rollover (silently drop)
}

void unregister_code(uint8_t kc) {
    if (IS_HID_MOD(kc)) {
        s_real_mods &= ~MOD_BIT(kc);
        s_dirty = true;
        return;
    }

    for (int i = 0; i < KEY_SLOTS; i++) {
        if (s_keys[i] == kc) {
            s_keys[i] = 0;
            s_dirty = true;
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// Modifier manipulation
// ---------------------------------------------------------------------------

void register_mods(uint8_t mods) {
    s_real_mods |= mods;
    s_dirty = true;
}

void unregister_mods(uint8_t mods) {
    s_real_mods &= ~mods;
    s_dirty = true;
}

void add_weak_mods(uint8_t mods) {
    s_weak_mods |= mods;
    s_dirty = true;
}

void clear_weak_mods(void) {
    if (s_weak_mods) {
        s_weak_mods = 0;
        s_dirty = true;
    }
}

void del_weak_mods(uint8_t mods) {
    if (s_weak_mods & mods) {
        s_weak_mods &= (uint8_t)~mods;
        s_dirty = true;
    }
}

// ---------------------------------------------------------------------------
// Clear all
// ---------------------------------------------------------------------------

void clear_keyboard(void) {
    s_real_mods = 0;
    s_weak_mods = 0;
    memset(s_keys, 0, sizeof(s_keys));
    s_dirty = true;
    send_keyboard_report();
}

// ---------------------------------------------------------------------------
// Non-blocking tap emitter
// ---------------------------------------------------------------------------

void tap_code_init(void) {
    s_tapq_head   = 0;
    s_tapq_count  = 0;
    s_tap_down    = false;
    s_tap_cur_kc  = 0;
    s_tap_cur_mods = 0;
    s_tap_timer   = 0;   // first tap starts immediately
}

static void tap_enqueue(uint8_t kc, uint8_t mods) {
    if (kc == 0) return;
    if (s_tapq_count >= TAP_CODE_QUEUE_SIZE) return;   // full → drop (rare)
    uint8_t tail = (uint8_t)((s_tapq_head + s_tapq_count) % TAP_CODE_QUEUE_SIZE);
    s_tapq[tail].kc   = kc;
    s_tapq[tail].mods = mods;
    s_tapq_count++;
}

void tap_code(uint8_t kc)                    { tap_enqueue(kc, 0); }
void tap_code_mods(uint8_t kc, uint8_t mods) { tap_enqueue(kc, mods); }

bool tap_code_busy(void) {
    return s_tap_down || s_tapq_count > 0;
}

void tap_code_tick(uint32_t now_ticks) {
    const TickType_t delay = pdMS_TO_TICKS(TAP_CODE_DELAY_MS);

    if (s_tap_down) {
        // Hold the press for the delay, then release it.
        if ((TickType_t)(now_ticks - s_tap_timer) < delay) return;
        unregister_code(s_tap_cur_kc);
        if (s_tap_cur_mods) del_weak_mods(s_tap_cur_mods);
        s_tap_down  = false;
        s_tap_timer = now_ticks;   // start the inter-tap gap
        DBG_KEY("[TAPQ] release kc=0x%02x\n", s_tap_cur_kc);
        return;
    }

    if (s_tapq_count == 0) return;
    // Gap between a release and the next press (skipped for the very first tap,
    // when s_tap_timer == 0) so consecutive same-key taps stay distinct.
    if (s_tap_timer != 0 && (TickType_t)(now_ticks - s_tap_timer) < delay) return;

    tap_entry_t e = s_tapq[s_tapq_head];
    s_tapq_head = (uint8_t)((s_tapq_head + 1) % TAP_CODE_QUEUE_SIZE);
    s_tapq_count--;

    if (e.mods) add_weak_mods(e.mods);
    register_code(e.kc);
    s_tap_cur_kc   = e.kc;
    s_tap_cur_mods = e.mods;
    s_tap_down     = true;
    s_tap_timer    = now_ticks;
    DBG_KEY("[TAPQ] press kc=0x%02x mods=0x%02x\n", e.kc, e.mods);
}

// ---------------------------------------------------------------------------
// USB output merge (upstream + local) — Core 0 side
// ---------------------------------------------------------------------------

#ifdef USB_ENABLE
// R2: USB keyboard delivery is CONFIRMED, single-sender, retried.
//   - The dedup snapshot (s_usb_prev) is committed only when TinyUSB actually
//     accepted the report. Before R2 a busy endpoint dropped the report while the
//     dedup advanced anyway → a lost RELEASE was never re-sent (stuck key).
//   - The merge is built and sent ONLY from the keyscan task (send path + the
//     action_util_tick retry). The CM task no longer sends directly: it updates
//     the upstream contribution and raises s_usb_send_pending. One sender → no
//     out-of-order sends between tasks.
static volatile bool s_usb_send_pending = false;

// Build merged = local OR upstream (upstream keycodes first, 6KRO — same rule as
// the BLE merge on Core 1) and push to USB. Source arrays are snapshotted under a
// critical section; the USB send happens OUTSIDE it (never call TinyUSB with
// interrupts disabled). Keyscan-task context ONLY.
static void usb_send_merged(void) {
    uint8_t merged[8];
    bool    changed;

    taskENTER_CRITICAL();
    memset(merged, 0, sizeof(merged));
    merged[0] = (uint8_t)(s_local_report[0] | s_usb_upstream_kbd[0]);
    uint8_t slot = 2;
    for (int i = 2; i < 8 && slot < 8; i++) {
        if (s_usb_upstream_kbd[i]) merged[slot++] = s_usb_upstream_kbd[i];
    }
    for (int i = 2; i < 8 && slot < 8; i++) {
        if (s_local_report[i]) merged[slot++] = s_local_report[i];
    }
    changed = (memcmp(merged, s_usb_prev, 8) != 0);
    taskEXIT_CRITICAL();

    if (!changed) {
        s_usb_send_pending = false;
        return;
    }
    if (usb_hid_send_kbd_report(merged)) {
        // Delivered to TinyUSB — commit the dedup snapshot.
        taskENTER_CRITICAL();
        memcpy(s_usb_prev, merged, 8);
        taskEXIT_CRITICAL();
        s_usb_send_pending = false;
    } else {
        // Endpoint busy / unmounted — retry from action_util_tick (next tick).
        s_usb_send_pending = true;
    }
}

void action_util_set_usb_upstream_kbd(const uint8_t up[8]) {
    taskENTER_CRITICAL();
    memcpy(s_usb_upstream_kbd, up, 8);
    taskEXIT_CRITICAL();
    // CM-task context: request the send; the keyscan tick flushes it (≤1 ms).
    s_usb_send_pending = true;
}
#endif // USB_ENABLE

void action_util_tick(uint32_t now_ticks) {
    (void)now_ticks;
#ifdef USB_ENABLE
    if (s_usb_send_pending && output_usb_enabled()) {
        usb_send_merged();
    }
#endif
}

// ---------------------------------------------------------------------------
// Build and send report
// ---------------------------------------------------------------------------

bool send_keyboard_report(void) {
    // Secure: suppress HID output while locked
    if (secure_is_locked()) {
        s_dirty = false;
        return false;
    }

    uint8_t report[8];
    memset(report, 0, 8);

    // byte 0: modifiers (real OR weak)
    report[0] = s_real_mods | s_weak_mods;
    // byte 1: reserved
    // bytes 2-7: keycodes (compact — skip zeros)
    int slot = 2;
    for (int i = 0; i < KEY_SLOTS && slot < 8; i++) {
        if (s_keys[i] != 0) {
            report[slot++] = s_keys[i];
        }
    }

    // Dedup: only send if changed
    if (memcmp(report, s_prev_report, 8) == 0) {
        s_dirty = false;
        return false;
    }

    memcpy(s_prev_report, report, 8);
    // Always feed Core 1 the local report — it merges with the upstream
    // contribution for the BLE downstream path (suppressed there when output=USB).
    ipc_put_kbd_report(report);
#ifdef USB_ENABLE
    // Keep the local-only snapshot current (read by the CM task when an upstream
    // keyboard report arrives), then push the merged report to USB when selected.
    // On USB the merge (upstream + local) is done here on Core 0; on BLE it stays
    // on Core 1. Local typing keeps its zero-latency Core 0 → USB fast path.
    taskENTER_CRITICAL();
    memcpy(s_local_report, report, 8);
    taskEXIT_CRITICAL();
    if (output_usb_enabled()) {
        usb_send_merged();
    }
#endif
    s_dirty = false;

    DBG_KEY("[UTIL] report mod=%02x keys=%02x %02x %02x %02x %02x %02x\n",
            report[0], report[2], report[3], report[4],
            report[5], report[6], report[7]);
    return true;
}

bool action_util_is_dirty(void) {
    return s_dirty;
}

uint8_t get_real_mods(void) {
    return s_real_mods;
}
