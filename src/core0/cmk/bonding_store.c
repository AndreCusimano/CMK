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
// bonding_store.c — BLE address storage su EEPROM esterna (M24M01-R)
//
// Init: eeprom_init() → se la EEPROM risponde, legge il record in RAM.
//       Se assente, funziona in RAM-only (dati persi al reboot).
//
// Tutte le letture avvengono dalla cache RAM (nessun I2C in lettura).
// Le scritture aggiornano la cache e, se la EEPROM è presente, persistono su di essa.
//
// Thread safety: le funzioni sono chiamate da task FreeRTOS Core 0 (CM task e
// keyscan task).  Non viene usato un mutex perché le scritture sono rare e
// brevi; le due operazioni concorrenti worst-case (clear_ds da keyscan +
// read da CM) non condividono la stessa sezione critica in pratica.
//
// Nessuna dipendenza da flash, IPC, o lock multicore.
//

#include "bonding_store.h"
#include "eeprom.h"
#include "config/debug_config.h"

#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Record layout — identico alla versione flash per compatibilità futura
// ---------------------------------------------------------------------------

#define BONDING_MAGIC_VAL   0x324B4D43u   // "CMK2" little-endian

#define FLAG_DS(slot)   ((uint8_t)(1u << (slot)))   // bit 0-2 = DS slot 0/1/2
#define FLAG_US         0x08u                        // bit 3   = upstream valid

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  flags;
    uint8_t  ds_addr[BONDING_DS_SLOTS][6];
    uint8_t  ds_addr_type[BONDING_DS_SLOTS];
    uint8_t  ds_slot;
    uint8_t  us_addr[6];
    uint8_t  us_addr_type;
} bonding_record_t;                // 4+1+18+3+1+6+1 = 34 byte

// Indirizzo EEPROM dove viene salvato il record (sempre byte 0)
#define EEPROM_RECORD_ADDR  0x0000u

// ---------------------------------------------------------------------------
// Stato interno
// ---------------------------------------------------------------------------

static bonding_record_t s_cache;       // copia in RAM, sempre valida dopo init
static bool             s_eeprom_ok;   // true se EEPROM rilevata e funzionante

// ---------------------------------------------------------------------------
// Helpers interni
// ---------------------------------------------------------------------------

static void cache_reset(void) {
    memset(&s_cache, 0, sizeof(s_cache));
    s_cache.magic   = BONDING_MAGIC_VAL;
    s_cache.ds_slot = 0;
}

// Persiste s_cache su EEPROM (se disponibile)
static void store_flush(void) {
    if (!s_eeprom_ok) return;
    eeprom_write(EEPROM_RECORD_ADDR, (const uint8_t *)&s_cache, sizeof(s_cache));
    DBG_MEM("[EEPROM] flush OK (%u byte)\n", (unsigned)sizeof(s_cache));
}

// ---------------------------------------------------------------------------
// Init — chiamare da main() prima di vTaskStartScheduler()
// ---------------------------------------------------------------------------

void bonding_store_init(void) {
    s_eeprom_ok = eeprom_init();

    if (s_eeprom_ok) {
        // R4: retried read + explicit error path. A bus glitch must NOT look
        // like a magic mismatch (which would wipe valid bonds from EEPROM).
        if (!eeprom_read_retry(EEPROM_RECORD_ADDR, (uint8_t *)&s_cache, sizeof(s_cache), 3)) {
            printf("[EEPROM] ERRORE lettura bonding record — modalità RAM-only\n");
            s_eeprom_ok = false;
            cache_reset();
            return;
        }
        if (s_cache.magic != BONDING_MAGIC_VAL) {
            DBG_MEM("[EEPROM] prima accensione — inizializzo record\n");
            cache_reset();
            store_flush();
        } else {
            DBG_MEM("[EEPROM] record valido letto dalla EEPROM\n");
        }
    } else {
        printf("[EEPROM] ATTENZIONE: EEPROM non rilevata — modalità RAM-only\n");
        cache_reset();
    }
}

// ---------------------------------------------------------------------------
// Downstream slots
// ---------------------------------------------------------------------------

bool bonding_store_has_ds(uint8_t slot) {
    if (slot >= BONDING_DS_SLOTS) return false;
    bool valid = (s_cache.flags & FLAG_DS(slot)) != 0;
    DBG_MEM("[EEPROM] has_ds(%u): %s\n", slot, valid ? "YES" : "NO");
    return valid;
}

void bonding_store_get_ds(uint8_t slot, uint8_t addr_out[6], uint8_t *type_out) {
    if (slot >= BONDING_DS_SLOTS) return;
    if (addr_out)  memcpy(addr_out, s_cache.ds_addr[slot], 6);
    if (type_out)  *type_out = s_cache.ds_addr_type[slot];
    DBG_MEM("[EEPROM] get_ds(%u): %02x:%02x:%02x:%02x:%02x:%02x type=%u\n", slot,
              s_cache.ds_addr[slot][5], s_cache.ds_addr[slot][4], s_cache.ds_addr[slot][3],
              s_cache.ds_addr[slot][2], s_cache.ds_addr[slot][1], s_cache.ds_addr[slot][0],
              s_cache.ds_addr_type[slot]);
}

void bonding_store_save_ds(uint8_t slot, const uint8_t addr[6], uint8_t addr_type) {
    if (slot >= BONDING_DS_SLOTS) return;
    DBG_MEM("[EEPROM] save_ds(%u): %02x:%02x:%02x:%02x:%02x:%02x type=%u\n", slot,
              addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], addr_type);
    memcpy(s_cache.ds_addr[slot], addr, 6);
    s_cache.ds_addr_type[slot] = addr_type;
    s_cache.flags |= FLAG_DS(slot);
    store_flush();
}

void bonding_store_clear_ds(uint8_t slot) {
    if (slot >= BONDING_DS_SLOTS) return;
    DBG_MEM("[EEPROM] clear_ds(%u)\n", slot);
    memset(s_cache.ds_addr[slot], 0, 6);
    s_cache.ds_addr_type[slot] = 0;
    s_cache.flags &= (uint8_t)~FLAG_DS(slot);
    store_flush();
}

// ---------------------------------------------------------------------------
// Active slot
// ---------------------------------------------------------------------------

uint8_t bonding_store_get_active_slot(void) {
    uint8_t s = s_cache.ds_slot;
    return (s < BONDING_DS_SLOTS) ? s : 0;
}

void bonding_store_set_active_slot(uint8_t slot) {
    if (slot >= BONDING_DS_SLOTS) return;
    DBG_MEM("[EEPROM] set_active_slot(%u)\n", slot);
    s_cache.ds_slot = slot;
    store_flush();
}

// ---------------------------------------------------------------------------
// Upstream
// ---------------------------------------------------------------------------

bool bonding_store_has_us(void) {
    bool valid = (s_cache.flags & FLAG_US) != 0;
    DBG_MEM("[EEPROM] has_us: %s\n", valid ? "YES" : "NO");
    return valid;
}

void bonding_store_get_us(uint8_t addr_out[6], uint8_t *type_out) {
    if (addr_out)  memcpy(addr_out, s_cache.us_addr, 6);
    if (type_out)  *type_out = s_cache.us_addr_type;
    DBG_MEM("[EEPROM] get_us: %02x:%02x:%02x:%02x:%02x:%02x type=%u\n",
              s_cache.us_addr[5], s_cache.us_addr[4], s_cache.us_addr[3],
              s_cache.us_addr[2], s_cache.us_addr[1], s_cache.us_addr[0],
              s_cache.us_addr_type);
}

void bonding_store_save_us(const uint8_t addr[6], uint8_t addr_type) {
    DBG_MEM("[EEPROM] save_us: %02x:%02x:%02x:%02x:%02x:%02x type=%u\n",
              addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], addr_type);
    memcpy(s_cache.us_addr, addr, 6);
    s_cache.us_addr_type = addr_type;
    s_cache.flags |= FLAG_US;
    store_flush();
}

void bonding_store_clear_us(void) {
    DBG_MEM("[EEPROM] clear_us\n");
    memset(s_cache.us_addr, 0, 6);
    s_cache.us_addr_type = 0;
    s_cache.flags &= (uint8_t)~FLAG_US;
    store_flush();
}
