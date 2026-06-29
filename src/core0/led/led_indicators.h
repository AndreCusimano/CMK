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
// led_indicators.h — Host LED indicator state (Phase 13)
//
// Receives Caps/Num/Scroll Lock state from BLE host via IPC.
//

#ifndef CMK_LED_INDICATORS_H
#define CMK_LED_INDICATORS_H

#include <stdint.h>
#include <stdbool.h>

// HID Output Report LED bit definitions (USB HID spec)
#define LED_NUM_LOCK    (1u << 0)
#define LED_CAPS_LOCK   (1u << 1)
#define LED_SCROLL_LOCK (1u << 2)
#define LED_COMPOSE     (1u << 3)
#define LED_KANA        (1u << 4)

// Called from connection_manager when IPC_LED_STATE arrives.
void led_indicators_set_state(uint8_t state);

// Returns current host LED state bitmask.
uint8_t led_indicators_get_state(void);

// Convenience checks.
static inline bool led_is_num_lock(void)    { return led_indicators_get_state() & LED_NUM_LOCK; }
static inline bool led_is_caps_lock(void)   { return led_indicators_get_state() & LED_CAPS_LOCK; }
static inline bool led_is_scroll_lock(void) { return led_indicators_get_state() & LED_SCROLL_LOCK; }

// User hook (weak) — called when state changes. Override for custom LED behavior.
void led_indicators_update(uint8_t state);

#endif // CMK_LED_INDICATORS_H
