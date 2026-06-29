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
// btstack_tlv_eeprom.h — BTstack TLV persistence on external I2C EEPROM
//
// Architecture:
//   - RAM mirror (~2.3 KB) in shared SRAM, protected by hardware spinlock 16
//   - Core 1 (BTstack SM) calls get_tag/store_tag/delete_tag on RAM → instant
//   - Core 0 (CM task) periodically flushes dirty pages to EEPROM
//   - Boot: Core 0 loads EEPROM → RAM before launching Core 1
//
// EEPROM layout (M24M01-R, 128 KiB total):
//   0x0000–0x00FF  bonding_store (addresses)
//   0x0100–0x011F  eeconfig (32 B)
//   0x1000–0x1A03  TLV store (this module, ~2.5 KiB)
//   0x2000+        free / reserved for dynamic_keymap (Phase 15)
//

#ifndef CMK_BTSTACK_TLV_EEPROM_H
#define CMK_BTSTACK_TLV_EEPROM_H

#include "btstack_tlv.h"

// ---------------------------------------------------------------------------
// EEPROM layout constants
// ---------------------------------------------------------------------------

// Start address in EEPROM. MUST NOT collide with the eeconfig region at
// 0x0100 (32 B). Aligned to a 4 KiB boundary so the TLV region has room to
// grow without needing to re-shuffle other regions on disk.
#define TLV_EEPROM_BASE       0x1000u
#define TLV_MAX_ENTRIES       32        // max number of tags
#define TLV_DATA_POOL_SIZE    2048      // bytes for tag values

// ---------------------------------------------------------------------------
// API — Core 0 only (except instance/context which are read-only after init)
// ---------------------------------------------------------------------------

// Load TLV region from EEPROM into RAM mirror.
// Call from main() AFTER eeprom_init()/bonding_store_init(), BEFORE multicore_launch_core1().
void btstack_tlv_eeprom_init(void);

// Return the btstack_tlv_t vtable for Core 1 to register with BTstack.
const btstack_tlv_t *btstack_tlv_eeprom_instance(void);

// Return the context pointer for Core 1 to pass to btstack_tlv_set_instance().
void *btstack_tlv_eeprom_context(void);

// Flush one dirty EEPROM page per call (non-blocking, max ~10ms for one page).
// Call from CM task loop (every 10ms). Multiple calls will flush all dirty pages.
// Safe to call even if nothing is dirty (returns immediately).
void btstack_tlv_eeprom_flush_if_dirty(void);

#endif // CMK_BTSTACK_TLV_EEPROM_H
