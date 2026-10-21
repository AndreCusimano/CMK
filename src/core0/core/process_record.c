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
// process_record.c — Key event processing chain (Phase 7)
//

#include "process_record.h"
#include "process_tap_dance.h"
#include "process_caps_word.h"
#include "process_key_override.h"
#include "process_mouse_keys.h"
#include "process_repeat_key.h"
#include "process_auto_shift.h"
#include "process_leader.h"
#include "process_space_cadet.h"
#include "process_grave_esc.h"
#include "process_magic.h"
#include "process_dynamic_macro.h"
#include "process_macro.h"
#include "process_layer_lock.h"
#include "process_key_lock.h"
#include "process_secure.h"
#include "swap_hands.h"
#include "velocikey.h"
#include "process_unicode.h"
#include "process_unicodemap.h"
#include "process_ucis.h"
#include "process_programmable_button.h"
#include "process_consumer.h"
#include "process_rgb.h"
#include "process_backlight.h"
#include "cmk_keycodes.h"

// ---------------------------------------------------------------------------
// Chain dispatch
// ---------------------------------------------------------------------------

bool process_record(const keyevent_t *event) {
    // 1. Tap Dance (multi-tap counting)
    if (!process_tap_dance(event)) return false;

    // 2. Caps Word (auto-shift)
    if (!process_caps_word(event)) return false;

    // 3. Key Override (modifier-based remap)
    if (!process_key_override(event)) return false;

    // 4. Mouse Keys (emulated mouse)
    if (!process_mouse_keys(event)) return false;

    // 5. Repeat Key (replay last keycode)
    if (!process_repeat_key(event)) return false;

    // 6. Auto Shift (hold brief → auto-shift)
    if (!process_auto_shift(event)) return false;

    // 7. Leader Key (sequence → action)
    if (!process_leader(event)) return false;

    // 8. Dynamic Macro (record/play — records non-DM keys passing through)
    if (!process_dynamic_macro(event)) return false;

    // 9. Space Cadet (Shift tap → parenthesis)
    if (!process_space_cadet(event)) return false;

    // 10. Grave Escape (ESC or ` based on mods)
    if (!process_grave_esc(event)) return false;

    // 11. Magic (toggle runtime settings)
    if (!process_magic(event)) return false;

    // 12. Layer Lock (Phase 11)
    if (!process_layer_lock(event)) return false;

    // 13. Key Lock (Phase 11)
    if (!process_key_lock(event)) return false;

    // 14. Secure (Phase 11)
    if (!process_secure(event)) return false;

    // 15. Swap Hands (Phase 11)
    if (!process_swap_hands(event)) return false;

    // 16. Velocikey toggle (Phase 11)
    if (!process_velocikey(event)) return false;

    // 17. Unicode basic (UC() keycodes + mode switch)
    if (!process_unicode(event)) return false;

    // 18. Unicode Map (UM()/UP() lookup table)
    if (!process_unicodemap(event)) return false;

    // 19. UCIS (abbreviation → Unicode)
    if (!process_ucis(event)) return false;

    // 20. Programmable Buttons (PB_1-32)
    if (!process_programmable_button(event)) return false;

    // 21. Consumer Page (media/volume/brightness — bug #6)
    if (!process_consumer(event)) return false;

    // 22. RGB (toggle, mode, HSV, speed)
    if (!process_rgb(event)) return false;

    // 23. Backlight (toggle, step, brightness)
    if (!process_backlight(event)) return false;

    // 24. Macro storage playback (Phase 15e — MACRO(n), bug #5)
    if (!process_macro(event)) return false;

    // 25. CMK custom keycodes (BLE slots, reset, etc.)
    if (!process_record_cmk(event)) return false;

    // 26. User hook (weak — default pass-through)
    if (!process_record_user(event)) return false;

    return true;  // not consumed — caller should handle as standard key
}

// ---------------------------------------------------------------------------
// Weak default: user hook
// ---------------------------------------------------------------------------

__attribute__((weak))
bool process_record_user(const keyevent_t *event) {
    (void)event;
    return true;  // pass-through
}
