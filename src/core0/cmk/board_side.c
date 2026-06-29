/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
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
// board_side.c — see board_side.h for the design rationale.
//

#include "board_side.h"

#include "config/board.h"
#include "config/cmk_config.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"

// The level is static (board identity), so a small average is plenty to reject
// noise on the boot read.
#define SIDE_SENSE_SAMPLES   16

// 12-bit ADC, 3.3V reference (same as power_manager.c).
#define SIDE_ADC_VREF_MV     3300u
#define SIDE_ADC_FULL_SCALE  4095u

static volatile cmk_side_t s_side     = CMK_SIDE_LEFT;
static volatile uint16_t   s_sense_mv = 0;

void board_side_init(void) {
#if defined(CMK_FORCE_SIDE_LEFT)
    s_side     = CMK_SIDE_LEFT;
    s_sense_mv = 0;
#elif defined(CMK_FORCE_SIDE_RIGHT)
    s_side     = CMK_SIDE_RIGHT;
    s_sense_mv = 0;
#else
    // The ADC peripheral may already be up (power_manager_init() also calls
    // adc_init()); adc_init() is idempotent, so calling it here keeps board_side
    // self-contained and order-independent.
    adc_init();

    // adc_gpio_init() puts the pad in analog mode: high-Z output, digital input
    // buffer OFF, pulls cleared. We then RE-enable the internal pull-down so the
    // left half (pin floating) reads ~0V. The pull bit (PDE) is independent of the
    // digital input-enable, so it stays active on the analog input. With the
    // digital receiver off, RP2350 erratum E9 (pull-down latch on digital inputs)
    // does not apply.
    adc_gpio_init(PIN_SIDE_SENSE);
    gpio_pull_down(PIN_SIDE_SENSE);

    adc_select_input(ADC_CHAN_SIDE_SENSE);

    uint32_t acc = 0;
    for (uint32_t i = 0; i < SIDE_SENSE_SAMPLES; i++) {
        acc += adc_read();
    }
    uint16_t raw = (uint16_t)(acc / SIDE_SENSE_SAMPLES);
    s_sense_mv = (uint16_t)((uint32_t)raw * SIDE_ADC_VREF_MV / SIDE_ADC_FULL_SCALE);

    s_side = (s_sense_mv > SIDE_SENSE_THRESHOLD_MV) ? CMK_SIDE_RIGHT : CMK_SIDE_LEFT;
#endif
}

cmk_side_t board_side_get(void) {
    return s_side;
}

uint16_t board_side_sense_mv(void) {
    return s_sense_mv;
}

const char *board_side_name(void) {
    return (s_side == CMK_SIDE_RIGHT) ? CMK_NAME_RIGHT : CMK_NAME_LEFT;
}

const char *board_side_short(void) {
    return (s_side == CMK_SIDE_RIGHT) ? "R" : "L";
}
