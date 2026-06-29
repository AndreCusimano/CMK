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
// wpm.c — Words Per Minute calculation (Phase 11)
//
// Sliding window: count keypresses in periods, compute WPM.
// Standard: 1 word = 5 characters.
//

#include "wpm.h"
#include "config/cmk_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

#define WPM_SAMPLE_SECONDS  10
#define WPM_SAMPLE_PERIODS   4   // 4 × 10s = 40s window

static uint8_t  s_wpm_current;
static uint16_t s_wpm_periods[WPM_SAMPLE_PERIODS];
static uint8_t  s_wpm_period_idx;
static uint32_t s_wpm_period_timer;
static uint16_t s_wpm_current_count;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void wpm_init(void) {
    s_wpm_current = 0;
    for (int i = 0; i < WPM_SAMPLE_PERIODS; i++) {
        s_wpm_periods[i] = 0;
    }
    s_wpm_period_idx   = 0;
    s_wpm_period_timer = 0;
    s_wpm_current_count = 0;
}

// ---------------------------------------------------------------------------
// Keypress — called from action_exec on every press
// ---------------------------------------------------------------------------

void wpm_keypress(void) {
    s_wpm_current_count++;
}

// ---------------------------------------------------------------------------
// Tick — rotate periods
// ---------------------------------------------------------------------------

void wpm_tick(uint32_t now_ticks) {
    if (s_wpm_period_timer == 0) {
        s_wpm_period_timer = now_ticks;
        return;
    }

    TickType_t elapsed = (TickType_t)(now_ticks - s_wpm_period_timer);
    if (elapsed < pdMS_TO_TICKS(WPM_SAMPLE_SECONDS * 1000)) return;

    // Period complete — save and rotate
    s_wpm_periods[s_wpm_period_idx] = s_wpm_current_count;
    s_wpm_current_count = 0;
    s_wpm_period_idx = (s_wpm_period_idx + 1) % WPM_SAMPLE_PERIODS;
    s_wpm_period_timer = now_ticks;

    // Compute WPM
    uint32_t total = 0;
    for (int i = 0; i < WPM_SAMPLE_PERIODS; i++) {
        total += s_wpm_periods[i];
    }
    // WPM = (total chars / 5 chars_per_word) / (window_sec / 60)
    // window_sec = WPM_SAMPLE_SECONDS * WPM_SAMPLE_PERIODS
    // WPM = total * 60 / (5 * WPM_SAMPLE_SECONDS * WPM_SAMPLE_PERIODS)
    uint32_t wpm = (total * 60) / (5 * WPM_SAMPLE_SECONDS * WPM_SAMPLE_PERIODS);
    s_wpm_current = (wpm > 255) ? 255 : (uint8_t)wpm;
}

// ---------------------------------------------------------------------------
// Getter
// ---------------------------------------------------------------------------

uint8_t get_current_wpm(void) {
    return s_wpm_current;
}
