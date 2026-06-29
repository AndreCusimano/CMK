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
// process_unicodemap.h — Unicode Map: UM()/UP() lookup table handler (Phase 12)
//

#ifndef PROCESS_UNICODEMAP_H
#define PROCESS_UNICODEMAP_H

#include "process_record.h"
#include <stdbool.h>
#include <stdint.h>

bool process_unicodemap(const keyevent_t *event);

// User-defined lookup table (weak — provide in keymap)
// Index with UM(i) or UP(i,j).
extern const uint32_t unicode_map[];
extern const uint16_t unicode_map_size;

#endif // PROCESS_UNICODEMAP_H
