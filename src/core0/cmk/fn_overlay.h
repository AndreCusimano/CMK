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
// fn_overlay.h — FN status overlay (Phase 19).
//
// While FN is held (LAYER_SETTINGS active), the RGB matrix shows a status
// dashboard instead of the normal effect: per-key colours derived from the
// keycode that sits at that position in the Settings layer (data-driven, so the
// mapping follows the left/right keymap automatically):
//   CMK_TD0/1/2  → DS BLE slot (blue): active slot live (solid=connected,
//                  fast blink=advertising, slow blink=directed, dim=idle);
//                  saved-but-inactive slot = dim; empty = off.
//   CMK_TU       → US BLE (cyan): solid=connected, slow blink=connecting,
//                  fast blink=scanning, off=idle.
//   CMK_OUT_USB  → USB (purple): on=mounted, off otherwise.
//   KC_FN        → battery charge gradient (green→red by SoC).
//   DF(n)        → red if n is the active default layer, else off.
//   anything else → off.
//
// rgb_matrix_tick() calls these to overwrite the frame while FN is held.
// Compiled only when FN_OVERLAY_ENABLE is defined.
//

#ifndef CMK_FN_OVERLAY_H
#define CMK_FN_OVERLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "rgb_driver.h"   // rgb_t

// True while the FN status overlay should take over the RGB frame.
bool fn_overlay_active(void);

// Status colour for the key at (row,col); black (0,0,0) if that key carries no
// status indicator. `now_ticks` (ms) drives the blink phases.
rgb_t fn_overlay_color(uint8_t row, uint8_t col, uint32_t now_ticks);

#endif // CMK_FN_OVERLAY_H
