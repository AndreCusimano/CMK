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
// process_magic.h — Magic: runtime keycode swaps (Phase 10)
//

#ifndef PROCESS_MAGIC_H
#define PROCESS_MAGIC_H

#include "process_record.h"
#include <stdint.h>

typedef struct {
    bool swap_ctl_caps;
    bool swap_esc_caps;
    bool caps_as_ctl;
    bool swap_lalt_lgui;
    bool swap_ralt_rgui;
    bool gui_disabled;
} magic_config_t;

void     magic_init(void);
bool     process_magic(const keyevent_t *event);
uint16_t magic_transform_keycode(uint16_t kc);

#endif // PROCESS_MAGIC_H
