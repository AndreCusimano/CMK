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
// bootmagic.c — Boot-time actions via held key (Phase 11)
//
// Bootmagic Lite: single key check at boot.
// If BOOTMAGIC key (row/col) is held → factory reset eeconfig + bonding_store,
// then reboot via watchdog.
//

#include "bootmagic.h"
#include "matrix.h"
#include "eeconfig.h"
#include "bonding_store.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#ifdef DYNAMIC_KEYMAP_ENABLE
#include "board_side.h"
#include "dynamic_keymap.h"
#endif

#include "eeprom.h"
#include "tlv/btstack_tlv_eeprom.h"

#include "hardware/watchdog.h"
#include <stdio.h>

void bootmagic_execute(void) {
    // Scan matrix once to read current key state
    matrix_scan();

    // Check if bootmagic key is held
    matrix_row_t row_state = matrix_get_row(BOOTMAGIC_ROW);
    if (!(row_state & (1u << BOOTMAGIC_COL))) {
        return;  // key not held — normal boot
    }

    printf("[BOOTMAGIC] key [%d][%d] held at boot — factory reset!\n",
           BOOTMAGIC_ROW, BOOTMAGIC_COL);

    // Factory reset all EEPROM regions
    eeconfig_factory_reset();

    // Clear all bonding slots
    for (uint8_t i = 0; i < BONDING_DS_SLOTS; i++) {
        bonding_store_clear_ds(i);
    }
    bonding_store_clear_us();

#ifdef DYNAMIC_KEYMAP_ENABLE
    // Restore the editable keymap to the compiled default for this side.
    dynamic_keymap_reset(board_side_get());
#endif

    // Purge the SM keys too (LTK/IRK in the BTstack TLV region). Clearing only
    // the bond addresses leaves stale LTKs behind → the next connect attempts
    // re-encryption with a key the peer no longer has ("re-encryption failed
    // 0x13") instead of a fresh pairing. Invalidating the TLV magic makes
    // btstack_tlv_eeprom_init (which runs later in main()) start empty. Safe
    // here: pre-scheduler, single thread, before Core 1 launch.
    uint8_t tlv_kill[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    eeprom_write(TLV_EEPROM_BASE, tlv_kill, sizeof(tlv_kill));

    printf("[BOOTMAGIC] reset complete — rebooting...\n");

    // Reboot via watchdog
    watchdog_reboot(0, 0, 100);  // 100ms delay
    while (1) { __asm volatile("nop"); }
}
