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
// led_indicators.c — Host LED indicator state (Phase 13)
//

#include "led_indicators.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static volatile uint8_t s_led_state;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void led_indicators_set_state(uint8_t state) {
    uint8_t old = s_led_state;
    s_led_state = state;
    if (old != state) {
        led_indicators_update(state);
    }
}

uint8_t led_indicators_get_state(void) {
    return s_led_state;
}

// ---------------------------------------------------------------------------
// Weak user hook
// ---------------------------------------------------------------------------

__attribute__((weak))
void led_indicators_update(uint8_t state) {
    (void)state;
    // Default: no-op. Override to drive GPIO or RGB indicators.
}
