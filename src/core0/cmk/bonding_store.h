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
// bonding_store.h — Persistent multi-slot BLE address storage
//
// Backend: EEPROM esterna M24M01-R (I2C0, GPIO4/GPIO5).
// Se la EEPROM non viene rilevata all'avvio, funziona in RAM-only
// (i dati vengono persi al reboot, ma nessun crash).
//
// Record layout (34 byte, indirizzo EEPROM 0x0000):
//   magic[4] | flags[1] | ds_addr[3][6] | ds_addr_type[3] | ds_slot[1]
//             | us_addr[6] | us_addr_type[1]
//
// flags: bit 0-2 = DS slot 0/1/2 valid;  bit 3 = US valid
//
// Chiamare bonding_store_init() da main() prima di vTaskStartScheduler().
// Tutte le altre funzioni sono Core 0 only.
//

#ifndef CMK_BONDING_STORE_H
#define CMK_BONDING_STORE_H

#include <stdint.h>
#include <stdbool.h>

// Number of downstream slots
#define BONDING_DS_SLOTS  3

// ---------------------------------------------------------------------------
// Init — chiamare da main() prima di vTaskStartScheduler()
// ---------------------------------------------------------------------------

// Inizializza il backend EEPROM.  Se la EEPROM non risponde, cade in RAM-only.
void bonding_store_init(void);

// ---------------------------------------------------------------------------
// Downstream (Peripheral) slots
// ---------------------------------------------------------------------------

// Returns true if a bonded downstream address is stored in slot.
bool bonding_store_has_ds(uint8_t slot);

// Fill addr_out[6] and *type_out with the stored downstream address for slot.
// Only valid if bonding_store_has_ds(slot) returns true.
void bonding_store_get_ds(uint8_t slot, uint8_t addr_out[6], uint8_t *type_out);

// Persist a downstream address to flash for the given slot.
// Core 0 only.
void bonding_store_save_ds(uint8_t slot, const uint8_t addr[6], uint8_t addr_type);

// Erase the downstream address for the given slot from flash.
// Core 0 only.
void bonding_store_clear_ds(uint8_t slot);

// ---------------------------------------------------------------------------
// Active downstream slot
// ---------------------------------------------------------------------------

// Returns the currently active downstream slot (0-2, default 0).
uint8_t bonding_store_get_active_slot(void);

// Persist the active downstream slot to flash.
// Core 0 only.
void bonding_store_set_active_slot(uint8_t slot);

// ---------------------------------------------------------------------------
// Upstream (Central)
// ---------------------------------------------------------------------------

// Returns true if a bonded upstream address is stored in flash.
bool bonding_store_has_us(void);

// Fill addr_out[6] and *type_out with the stored upstream address.
// Only valid if bonding_store_has_us() returns true.
void bonding_store_get_us(uint8_t addr_out[6], uint8_t *type_out);

// Persist an upstream address to flash.
// Core 0 only.
void bonding_store_save_us(const uint8_t addr[6], uint8_t addr_type);

// Erase the upstream address from flash.
// Core 0 only.
void bonding_store_clear_us(void);

#endif // CMK_BONDING_STORE_H
