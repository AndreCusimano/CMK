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
// backlight.h — PWM single-channel backlight (Phase 13d)
//
// Simple brightness control via hardware PWM.
// Independent of RGB/LED Matrix — can be enabled alongside or alone.
//

#ifndef CMK_BACKLIGHT_H
#define CMK_BACKLIGHT_H

#include "config/cmk_config.h"

#ifdef BACKLIGHT_ENABLE

#include <stdint.h>
#include <stdbool.h>

void     backlight_init(void);
void     backlight_toggle(void);
void     backlight_step(void);        // cycle 0 → 1 → ... → LEVELS → 0
void     backlight_increase(void);
void     backlight_decrease(void);
void     backlight_set(uint8_t level);
uint8_t  backlight_get(void);
bool     backlight_is_on(void);

#endif // BACKLIGHT_ENABLE

#endif // CMK_BACKLIGHT_H
