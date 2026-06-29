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
// process_ucis.h — UCIS: Unicode Character Input System (Phase 12)
//
// Press CMK_UCIS → type abbreviation → Enter/Space → backspace + Unicode output.
// Similar pattern to Leader Key but matches abbreviation strings.
//

#ifndef PROCESS_UCIS_H
#define PROCESS_UCIS_H

#include "process_record.h"
#include <stdbool.h>
#include <stdint.h>

// UCIS entry: abbreviation string → Unicode codepoint
typedef struct {
    const char *abbreviation;  // e.g. "alpha"
    uint32_t    codepoint;     // e.g. 0x03B1 (α)
} ucis_entry_t;

void ucis_init(void);
bool process_ucis(const keyevent_t *event);

// User-defined UCIS table (weak — provide in keymap)
// Terminated by entry with abbreviation == NULL.
extern const ucis_entry_t ucis_table[];

#endif // PROCESS_UCIS_H
