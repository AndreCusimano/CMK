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
// unicode.h — Unicode input common infrastructure (Phase 12)
//
// Provides OS-specific Unicode codepoint entry via tick-based state machine.
// Supports 4 modes: Windows (Alt+numpad hex), macOS (Option+hex),
// Linux (Ctrl+Shift+U), WinCompose (Compose+U+hex).
//

#ifndef CMK_UNICODE_H
#define CMK_UNICODE_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Unicode input mode
// ---------------------------------------------------------------------------

typedef enum {
    UC_MODE_WINDOWS,     // Alt + KP_PLUS + hex digits (needs registry EnableHexNumpad)
    UC_MODE_MACOS,       // hold Option + hex digits
    UC_MODE_LINUX,       // Ctrl+Shift+U + hex digits + Enter
    UC_MODE_WINCOMPOSE,  // tap RAlt + U + hex digits + Enter
    UC_MODE_COUNT,
} unicode_mode_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void unicode_init(void);

// Get/set current mode (persisted in eeconfig)
unicode_mode_t unicode_get_mode(void);
void unicode_set_mode(unicode_mode_t mode);
void unicode_cycle_mode(bool forward);

// Send a Unicode codepoint (BMP: 0x0000-0x7FFF).
// Non-blocking — queues the sequence, unicode_tick() drives it.
void unicode_send_codepoint(uint32_t codepoint);

// Tick — drives the OS-specific key sequence (press/release cycle).
// Called from action_tick().
void unicode_tick(uint32_t now_ticks);

// Returns true if a unicode send is in progress.
bool unicode_is_active(void);

#endif // CMK_UNICODE_H
