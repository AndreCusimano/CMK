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
// velocikey.c — Typing speed → animation speed (Phase 11)
//
// EMA (exponential moving average) smoothing of WPM → speed 0-255.
// Phase 11: stub — computes speed, no RGB consumer yet.
//

#include "velocikey.h"
#include "wpm.h"
#include "keycode.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool    s_velocikey_enabled;
static uint8_t s_velocikey_speed;  // 0-255, smoothed

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void velocikey_init(void) {
    s_velocikey_enabled = false;
    s_velocikey_speed   = 0;
}

// ---------------------------------------------------------------------------
// Process — toggle keycode
// ---------------------------------------------------------------------------

bool process_velocikey(const keyevent_t *event) {
    if (event->keycode != CMK_VELOCIKEY_TOG) return true;

    if (event->pressed) {
        s_velocikey_enabled = !s_velocikey_enabled;
        if (!s_velocikey_enabled) s_velocikey_speed = 0;
        DBG_KEY("[VKEY] %s\n", s_velocikey_enabled ? "enabled" : "disabled");
    }
    return false;  // consumed
}

// ---------------------------------------------------------------------------
// Tick — EMA smoothing
// ---------------------------------------------------------------------------

void velocikey_tick(uint32_t now_ticks) {
    (void)now_ticks;
    if (!s_velocikey_enabled) return;

    uint8_t wpm = get_current_wpm();
    // EMA: speed = speed * 7/8 + wpm * 1/8
    s_velocikey_speed = (uint8_t)(((uint16_t)s_velocikey_speed * 7 + (uint16_t)wpm) / 8);
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------

uint8_t velocikey_get_speed(void) {
    return s_velocikey_speed;
}

bool velocikey_is_enabled(void) {
    return s_velocikey_enabled;
}
