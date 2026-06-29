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
// process_unicode.c — Unicode basic: UC() keycode + mode switch handler (Phase 12)
//
// Handles:
//   - UC(codepoint) keycodes → unicode_send_codepoint()
//   - CMK_UC_WIN/MAC/LINX/WINC → set mode directly
//   - CMK_UC_NEXT/PREV → cycle mode
//

#include "process_unicode.h"
#include "unicode.h"
#include "keycode.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void process_unicode_init(void) {
    unicode_init();
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_unicode(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // --- Mode switch keycodes ---
    if (IS_UC_MODE(kc)) {
        if (event->pressed) {
            switch (kc) {
                case CMK_UC_WIN:  unicode_set_mode(UC_MODE_WINDOWS);    break;
                case CMK_UC_MAC:  unicode_set_mode(UC_MODE_MACOS);      break;
                case CMK_UC_LINX: unicode_set_mode(UC_MODE_LINUX);      break;
                case CMK_UC_WINC: unicode_set_mode(UC_MODE_WINCOMPOSE); break;
                case CMK_UC_NEXT: unicode_cycle_mode(true);             break;
                case CMK_UC_PREV: unicode_cycle_mode(false);            break;
                default: break;
            }
        }
        return false;  // consumed
    }

    // --- UC(codepoint) keycodes ---
    if (IS_UNICODE(kc)) {
        if (event->pressed && !unicode_is_active()) {
            uint32_t codepoint = kc & 0x7FFF;
            unicode_send_codepoint(codepoint);
        }
        return false;  // consumed
    }

    return true;  // pass through
}
