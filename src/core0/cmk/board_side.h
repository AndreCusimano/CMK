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
// board_side.h — Runtime board-side (Left/Right) identity for the split halves.
//
// One firmware runs on both physical halves. Which half this is, is sensed ONCE
// at boot from an analog level on PIN_SIDE_SENSE (GP41 / ADC1): the right half
// ties the pin to an external ~1V source, the left half leaves it floating. An
// internal pull-down holds the floating (left) case near 0V. We read the voltage
// — not the digital level: ~1V sits in the RP2350 indeterminate input band — and
// compare it to SIDE_SENSE_THRESHOLD_MV.
//
// board_side_init() runs on Core 0, BEFORE Core 1 is launched. The result is
// cached and never changes afterwards, so both cores may read it (board_side_get
// / _name) as an immutable shared constant — no IPC/FIFO needed.
//

#ifndef CMK_BOARD_SIDE_H
#define CMK_BOARD_SIDE_H

#include <stdint.h>

typedef enum {
    CMK_SIDE_LEFT  = 0,
    CMK_SIDE_RIGHT = 1,
} cmk_side_t;

// One-shot detection. Call on Core 0 early in main(), before keymap_init(),
// action_init() and multicore_launch_core1(). Honors the compile-time overrides
// CMK_FORCE_SIDE_LEFT / CMK_FORCE_SIDE_RIGHT (skips the ADC entirely).
void board_side_init(void);

// Cached side. Valid after board_side_init(). Safe from either core.
cmk_side_t board_side_get(void);

// Raw averaged sense voltage in millivolts (0 when a compile-time override is
// active). Useful for the boot banner and threshold calibration.
uint16_t board_side_sense_mv(void);

// Full device name for this side: "CMK-Keeeyboard-Split-L" / "-R".
// Points to flash-resident const storage — valid for the program lifetime.
const char *board_side_name(void);

// Short label for logs: "L" / "R".
const char *board_side_short(void);

#endif // CMK_BOARD_SIDE_H
