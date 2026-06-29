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
// process_rgb.c — RGB/LED keycode handler (Phase 13)
//
// Dispatches CMK_RGB_* keycodes to the active lighting module.
//

#include "process_rgb.h"
#include "keycode.h"
#include "config/cmk_config.h"

#if defined(RGB_MATRIX_ENABLE)
#include "rgb_matrix.h"
#elif defined(LED_MATRIX_ENABLE)
#include "led_matrix.h"
#elif defined(RGBLIGHT_ENABLE)
#include "rgblight.h"
#endif

bool process_rgb(const keyevent_t *event) {
    uint16_t kc = event->keycode;
    if (!IS_RGB(kc)) return true;

    // Only process on press
    if (!event->pressed) return false;

#if defined(RGB_MATRIX_ENABLE)
    switch (kc) {
        case CMK_RGB_TOG:    rgb_matrix_toggle();           break;
        case CMK_RGB_MOD:    rgb_matrix_step();             break;
        case CMK_RGB_RMOD:   rgb_matrix_step_reverse();     break;
        case CMK_RGB_HUI:    rgb_matrix_increase_hue();     break;
        case CMK_RGB_HUD:    rgb_matrix_decrease_hue();     break;
        case CMK_RGB_SAI:    rgb_matrix_increase_sat();     break;
        case CMK_RGB_SAD:    rgb_matrix_decrease_sat();     break;
        case CMK_RGB_INC:    rgb_matrix_increase_val();     break;
        case CMK_RGB_DEC:    rgb_matrix_decrease_val();     break;
        case CMK_RGB_SPI:    rgb_matrix_increase_speed();   break;
        case CMK_RGB_SPD:    rgb_matrix_decrease_speed();   break;
        // 8-step rings
        case CMK_RGB_VAL_R:  rgb_matrix_cycle_val();        break;
        case CMK_RGB_HUE_R:  rgb_matrix_cycle_hue();        break;
        case CMK_RGB_SAT_R:  rgb_matrix_cycle_sat();        break;
        case CMK_RGB_SPD_R:  rgb_matrix_cycle_speed();      break;
        case CMK_RGB_WSPD_R: rgb_matrix_cycle_wave_speed(); break;
        case CMK_RGB_WWID_R: rgb_matrix_cycle_wave_width(); break;
        case CMK_RGB_FADE_R: rgb_matrix_cycle_fade();       break;
        case CMK_RGB_HSFT_R: rgb_matrix_cycle_hue_shift();  break;
        default: return true;
    }

#elif defined(LED_MATRIX_ENABLE)
    switch (kc) {
        case CMK_RGB_TOG:  led_matrix_toggle();           break;
        case CMK_RGB_MOD:  led_matrix_step();             break;
        case CMK_RGB_RMOD: led_matrix_step_reverse();     break;
        case CMK_RGB_INC:  led_matrix_increase_val();     break;
        case CMK_RGB_DEC:  led_matrix_decrease_val();     break;
        case CMK_RGB_SPI:  led_matrix_increase_speed();   break;
        case CMK_RGB_SPD:  led_matrix_decrease_speed();   break;
        default: return true;  // HUI/HUD/SAI/SAD ignored for mono
    }

#elif defined(RGBLIGHT_ENABLE)
    switch (kc) {
        case CMK_RGB_TOG:  rgblight_toggle();             break;
        case CMK_RGB_MOD:  rgblight_step();               break;
        case CMK_RGB_RMOD: rgblight_step_reverse();       break;
        case CMK_RGB_HUI:  rgblight_increase_hue();       break;
        case CMK_RGB_HUD:  rgblight_decrease_hue();       break;
        case CMK_RGB_SAI:  rgblight_increase_sat();       break;
        case CMK_RGB_SAD:  rgblight_decrease_sat();       break;
        case CMK_RGB_INC:  rgblight_increase_val();       break;
        case CMK_RGB_DEC:  rgblight_decrease_val();       break;
        case CMK_RGB_SPI:  rgblight_increase_speed();     break;
        case CMK_RGB_SPD:  rgblight_decrease_speed();     break;
        default: return true;
    }

#else
    // No lighting module enabled — ignore
    (void)kc;
    return true;
#endif

    return false;  // consumed
}
