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
// process_leader.c — Leader Key: sequence → action (Phase 10)
//
// Press CMK_LEAD, then type a sequence of keys within LEADER_TIMEOUT.
// On timeout (or buffer full), leader_end_user() is called where the user
// matches sequences using helper functions and triggers actions.
//

#include "process_leader.h"
#include "keycode.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool     s_leader_active;
static uint16_t s_leader_seq[LEADER_SEQ_MAX];
static uint8_t  s_leader_count;
static uint32_t s_leader_timer;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void leader_init(void) {
    s_leader_active = false;
    s_leader_count  = 0;
}

// ---------------------------------------------------------------------------
// Weak user hooks
// ---------------------------------------------------------------------------

__attribute__((weak))
void leader_start_user(void) { }

__attribute__((weak))
void leader_end_user(void) { }

// ---------------------------------------------------------------------------
// Sequence matching helpers
// ---------------------------------------------------------------------------

bool leader_sequence_one_key(uint16_t kc1) {
    return s_leader_count == 1 && s_leader_seq[0] == kc1;
}

bool leader_sequence_two_keys(uint16_t kc1, uint16_t kc2) {
    return s_leader_count == 2 &&
           s_leader_seq[0] == kc1 && s_leader_seq[1] == kc2;
}

bool leader_sequence_three_keys(uint16_t kc1, uint16_t kc2, uint16_t kc3) {
    return s_leader_count == 3 &&
           s_leader_seq[0] == kc1 && s_leader_seq[1] == kc2 &&
           s_leader_seq[2] == kc3;
}

bool leader_sequence_four_keys(uint16_t kc1, uint16_t kc2, uint16_t kc3, uint16_t kc4) {
    return s_leader_count == 4 &&
           s_leader_seq[0] == kc1 && s_leader_seq[1] == kc2 &&
           s_leader_seq[2] == kc3 && s_leader_seq[3] == kc4;
}

bool leader_sequence_five_keys(uint16_t kc1, uint16_t kc2, uint16_t kc3, uint16_t kc4, uint16_t kc5) {
    return s_leader_count == 5 &&
           s_leader_seq[0] == kc1 && s_leader_seq[1] == kc2 &&
           s_leader_seq[2] == kc3 && s_leader_seq[3] == kc4 &&
           s_leader_seq[4] == kc5;
}

// ---------------------------------------------------------------------------
// End leader sequence — call user hook for matching
// ---------------------------------------------------------------------------

static void leader_end(void) {
    DBG_KEY("[LEAD] end → %d keys\n", s_leader_count);
    leader_end_user();
    s_leader_active = false;
    s_leader_count  = 0;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_leader(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // Activate leader mode
    if (kc == CMK_LEAD) {
        if (event->pressed && !s_leader_active) {
            s_leader_active = true;
            s_leader_count  = 0;
            s_leader_timer  = event->timestamp;
            DBG_KEY("[LEAD] start\n");
            leader_start_user();
        }
        return false;  // consumed
    }

    // Not in leader mode → pass through
    if (!s_leader_active) return true;

    // In leader mode: only consume key presses (ignore releases)
    if (!event->pressed) return false;  // consume releases silently

    // Buffer the keycode
    if (s_leader_count < LEADER_SEQ_MAX) {
        s_leader_seq[s_leader_count++] = kc;
        s_leader_timer = event->timestamp;  // reset timer (per-key timing)
        DBG_KEY("[LEAD] key %d → kc=0x%04x\n", s_leader_count, kc);
    }

    // Buffer full → end immediately
    if (s_leader_count >= LEADER_SEQ_MAX) {
        leader_end();
    }

    return false;  // consumed
}

// ---------------------------------------------------------------------------
// Tick — timeout check
// ---------------------------------------------------------------------------

void leader_tick(uint32_t now_ticks) {
    if (!s_leader_active) return;

    TickType_t elapsed = (TickType_t)(now_ticks - s_leader_timer);
    if (elapsed >= pdMS_TO_TICKS(LEADER_TIMEOUT)) {
        leader_end();
    }
}
