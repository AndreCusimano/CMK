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
// process_consumer.h — Consumer Page media/volume/screen keys (bug #6).
//
// KC_MUTE/KC_VOLU/…/KC_LPAD (0x7100-0x710A) map to 16-bit Consumer usages
// sent in the dedicated Report ID 4 (2 bytes, single-usage Array field):
// press → usage, release → 0x0000 (last-wins, mirror of the PB pipeline).
//

#ifndef CMK_PROCESS_CONSUMER_H
#define CMK_PROCESS_CONSUMER_H

#include <stdint.h>
#include <stdbool.h>

#include "process_record.h"   // keyevent_t

// false = consumed (consumer keycode), true = pass through.
bool process_consumer(const keyevent_t *event);

#endif // CMK_PROCESS_CONSUMER_H
