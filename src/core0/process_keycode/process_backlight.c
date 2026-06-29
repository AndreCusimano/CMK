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
// process_backlight.c — Backlight keycode handler (Phase 13d)
//

#include "config/cmk_config.h"

#ifdef BACKLIGHT_ENABLE

#include "process_backlight.h"
#include "backlight.h"
#include "keycode.h"

bool process_backlight(const keyevent_t *event) {
    uint16_t kc = event->keycode;
    if (!IS_BACKLIGHT(kc)) return true;

    if (!event->pressed) return false;

    switch (kc) {
        case CMK_BL_TOGG:  backlight_toggle();    break;
        case CMK_BL_STEP:  backlight_step();       break;
        case CMK_BL_UP:    backlight_increase();   break;
        case CMK_BL_DOWN:  backlight_decrease();   break;
        default: return true;
    }

    return false;  // consumed
}

#endif // BACKLIGHT_ENABLE
