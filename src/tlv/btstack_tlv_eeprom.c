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
// btstack_tlv_eeprom.c — BTstack TLV persistence on external I2C EEPROM
//
// RAM mirror architecture:
//   - s_store lives in shared SRAM, accessible by both cores
//   - Hardware spinlock 16 protects concurrent access
//   - Core 1 reads/writes RAM instantly; sets dirty bits for EEPROM flush
//   - Core 0 CM task calls flush_if_dirty() every 10ms, writing one page per call
//

#include "btstack_tlv_eeprom.h"
#include "core0/drivers/eeprom/eeprom.h"
#include "ipc/ipc_core.h"
#include "config/debug_config.h"

#include "hardware/sync.h"

#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define TLV_MAGIC             0x544C5631u  // "TLV1"
// Claim-free spinlock range 24..31 (R3). The old ID 16 sat in the SDK "striped"
// range, shared round-robin with every SDK mutex/semaphore → hidden contention.
#define TLV_SPINLOCK_NUM      26

// Derived layout
#define TLV_HEADER_SIZE       4                                         // magic
#define TLV_INDEX_SIZE        (TLV_MAX_ENTRIES * sizeof(tlv_index_entry_t))  // 32 × 8 = 256
#define TLV_STORE_SIZE        (TLV_HEADER_SIZE + TLV_INDEX_SIZE + TLV_DATA_POOL_SIZE)  // 2308

// Number of EEPROM pages the TLV region spans
#define TLV_NUM_PAGES         ((TLV_STORE_SIZE + EEPROM_PAGE_SIZE - 1) / EEPROM_PAGE_SIZE)

// ---------------------------------------------------------------------------
// On-disk / RAM index entry
// ---------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint32_t tag;       // BTstack tag (e.g. 'BTD0', 'SMIR'), 0 = empty slot
    uint16_t offset;    // byte offset into data_pool[]
    uint16_t len;       // data length (0 = deleted)
} tlv_index_entry_t;

// ---------------------------------------------------------------------------
// RAM mirror (shared SRAM, both cores)
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t           magic;
    tlv_index_entry_t  index[TLV_MAX_ENTRIES];
    uint8_t            data_pool[TLV_DATA_POOL_SIZE];
    uint16_t           data_used;       // next free offset in data_pool
} tlv_ram_store_t;

static tlv_ram_store_t s_store;

// Dirty page tracking — one bit per EEPROM page
static volatile uint32_t s_dirty_pages = 0;   // bitmask, bit N = page N dirty
static spin_lock_t *s_lock;

// Flag: true if EEPROM was present at boot (false = RAM-only mode)
static bool s_eeprom_present = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Mark all EEPROM pages as dirty (full flush needed)
static void mark_all_dirty(void) {
    s_dirty_pages = (1u << TLV_NUM_PAGES) - 1u;
}

// Mark the page(s) that cover a byte range [offset, offset+len) within s_store
static void mark_dirty_range(uint16_t store_offset, uint16_t len) {
    if (len == 0) return;
    uint16_t first_page = store_offset / EEPROM_PAGE_SIZE;
    uint16_t last_page  = (store_offset + len - 1) / EEPROM_PAGE_SIZE;
    for (uint16_t p = first_page; p <= last_page && p < TLV_NUM_PAGES; p++) {
        s_dirty_pages |= (1u << p);
    }
}

// Offset of the index array within s_store (for dirty tracking)
#define INDEX_STORE_OFFSET   offsetof(tlv_ram_store_t, index)
// Offset of data_pool within s_store
#define DATA_STORE_OFFSET    offsetof(tlv_ram_store_t, data_pool)

// Find an index entry by tag. Returns pointer or NULL.
static tlv_index_entry_t *find_entry(uint32_t tag) {
    for (int i = 0; i < TLV_MAX_ENTRIES; i++) {
        if (s_store.index[i].tag == tag && s_store.index[i].len > 0) {
            return &s_store.index[i];
        }
    }
    return NULL;
}

// Find an empty index slot. Returns pointer or NULL.
static tlv_index_entry_t *find_empty_slot(void) {
    for (int i = 0; i < TLV_MAX_ENTRIES; i++) {
        if (s_store.index[i].tag == 0 || s_store.index[i].len == 0) {
            return &s_store.index[i];
        }
    }
    return NULL;
}

// Compact the data pool: pack all valid entries contiguously.
// Must be called under spinlock.
static void compact(void) {
    uint16_t write_pos = 0;
    uint8_t temp[TLV_DATA_POOL_SIZE];

    for (int i = 0; i < TLV_MAX_ENTRIES; i++) {
        tlv_index_entry_t *e = &s_store.index[i];
        if (e->tag == 0 || e->len == 0) {
            // Clear deleted entries fully
            e->tag = 0;
            e->offset = 0;
            e->len = 0;
            continue;
        }
        if (e->offset + e->len > TLV_DATA_POOL_SIZE) {
            // Corrupted entry — discard
            e->tag = 0;
            e->offset = 0;
            e->len = 0;
            continue;
        }
        memcpy(&temp[write_pos], &s_store.data_pool[e->offset], e->len);
        e->offset = write_pos;
        write_pos += e->len;
    }

    memcpy(s_store.data_pool, temp, write_pos);
    s_store.data_used = write_pos;
    mark_all_dirty();
}

// ---------------------------------------------------------------------------
// btstack_tlv_t implementation (callable from Core 1)
// ---------------------------------------------------------------------------

static int tlv_get_tag(void *context, uint32_t tag, uint8_t *buffer, uint32_t buffer_size) {
    (void)context;
    uint32_t save = spin_lock_blocking(s_lock);

    tlv_index_entry_t *e = find_entry(tag);
    if (!e) {
        spin_unlock(s_lock, save);
        return 0;
    }

    uint32_t copy = (e->len < buffer_size) ? e->len : buffer_size;
    memcpy(buffer, &s_store.data_pool[e->offset], copy);
    uint16_t actual_len = e->len;

    spin_unlock(s_lock, save);
    return (int)actual_len;
}

static int tlv_store_tag(void *context, uint32_t tag, const uint8_t *data, uint32_t data_size) {
    (void)context;
    if (data_size > TLV_DATA_POOL_SIZE) return -1;
    // Core 1 log — safe from BTstack callbacks
    core1_log("[TLV] store_tag '%c%c%c%c' %lu B\n",
              (char)(tag >> 24), (char)(tag >> 16), (char)(tag >> 8), (char)tag,
              (unsigned long)data_size);

    uint32_t save = spin_lock_blocking(s_lock);

    tlv_index_entry_t *e = find_entry(tag);

    if (e && e->len == (uint16_t)data_size) {
        // Same size — overwrite in place
        memcpy(&s_store.data_pool[e->offset], data, data_size);
        mark_dirty_range((uint16_t)(DATA_STORE_OFFSET + e->offset), (uint16_t)data_size);
        spin_unlock(s_lock, save);
        return 0;
    }

    if (e) {
        // Different size — logically delete old entry
        e->len = 0;
    }

    // Allocate new space
    if (s_store.data_used + (uint16_t)data_size > TLV_DATA_POOL_SIZE) {
        compact();
        if (s_store.data_used + (uint16_t)data_size > TLV_DATA_POOL_SIZE) {
            // Still no space after compaction
            spin_unlock(s_lock, save);
            return -1;
        }
    }

    // Find or reuse index slot
    tlv_index_entry_t *slot = (e && e->len == 0) ? e : find_empty_slot();
    if (!slot) {
        spin_unlock(s_lock, save);
        return -1;
    }

    slot->tag    = tag;
    slot->offset = s_store.data_used;
    slot->len    = (uint16_t)data_size;
    memcpy(&s_store.data_pool[s_store.data_used], data, data_size);
    s_store.data_used += (uint16_t)data_size;

    // Mark index entry page + data page as dirty
    uint16_t idx_off = (uint16_t)(INDEX_STORE_OFFSET +
                       (uint16_t)((slot - s_store.index) * sizeof(tlv_index_entry_t)));
    mark_dirty_range(idx_off, sizeof(tlv_index_entry_t));
    mark_dirty_range((uint16_t)(DATA_STORE_OFFSET + slot->offset), (uint16_t)data_size);

    spin_unlock(s_lock, save);
    return 0;
}

static void tlv_delete_tag(void *context, uint32_t tag) {
    (void)context;
    uint32_t save = spin_lock_blocking(s_lock);

    tlv_index_entry_t *e = find_entry(tag);
    if (e) {
        e->len = 0;  // logical delete
        // Mark index entry page dirty
        uint16_t idx_off = (uint16_t)(INDEX_STORE_OFFSET +
                           (uint16_t)((e - s_store.index) * sizeof(tlv_index_entry_t)));
        mark_dirty_range(idx_off, sizeof(tlv_index_entry_t));
    }

    spin_unlock(s_lock, save);
}

// ---------------------------------------------------------------------------
// vtable
// ---------------------------------------------------------------------------

static const btstack_tlv_t s_tlv_impl = {
    .get_tag    = &tlv_get_tag,
    .store_tag  = &tlv_store_tag,
    .delete_tag = &tlv_delete_tag,
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void btstack_tlv_eeprom_init(void) {
    spin_lock_claim(TLV_SPINLOCK_NUM);
    s_lock = spin_lock_init(TLV_SPINLOCK_NUM);
    s_dirty_pages = 0;

    // Try to read the TLV region from EEPROM
    // (eeprom_init() must have been called already by bonding_store_init)
    // R4: retried + explicit error path. Before, a bus glitch returned garbage
    // → "no valid store" → empty store flushed over the REAL one → all SM keys
    // (LTK/IRK) lost → re-encryption failure loop with every bonded host.
    uint8_t raw[TLV_STORE_SIZE];
    if (!eeprom_read_retry(TLV_EEPROM_BASE, raw, TLV_STORE_SIZE, 3)) {
        printf("[TLV] ERRORE lettura EEPROM — store vuoto in RAM, flush disabilitato\n");
        memset(&s_store, 0, sizeof(s_store));
        s_store.magic     = TLV_MAGIC;
        s_store.data_used = 0;
        s_eeprom_present  = false;   // RAM-only: never flush over possibly-valid keys
        return;
    }

    // Check magic
    uint32_t magic;
    memcpy(&magic, raw, 4);

    if (magic == TLV_MAGIC) {
        // Valid store — load into RAM
        memcpy(&s_store.magic, raw, TLV_HEADER_SIZE);
        memcpy(s_store.index, raw + TLV_HEADER_SIZE, TLV_INDEX_SIZE);
        memcpy(s_store.data_pool, raw + TLV_HEADER_SIZE + TLV_INDEX_SIZE, TLV_DATA_POOL_SIZE);

        // Reconstruct data_used from index entries
        s_store.data_used = 0;
        int entry_count = 0;
        for (int i = 0; i < TLV_MAX_ENTRIES; i++) {
            if (s_store.index[i].tag != 0 && s_store.index[i].len > 0) {
                uint16_t end = s_store.index[i].offset + s_store.index[i].len;
                if (end > s_store.data_used) {
                    s_store.data_used = end;
                }
                entry_count++;
            }
        }
        s_eeprom_present = true;
        printf("[TLV] Loaded from EEPROM: %u bytes used, %d entries\n",
               s_store.data_used, entry_count);
        for (int i = 0; i < TLV_MAX_ENTRIES; i++) {
            if (s_store.index[i].tag != 0 && s_store.index[i].len > 0) {
                uint32_t t = s_store.index[i].tag;
                printf("[TLV]   [%d] tag='%c%c%c%c' off=%u len=%u\n", i,
                       (char)(t >> 24), (char)(t >> 16), (char)(t >> 8), (char)t,
                       s_store.index[i].offset, s_store.index[i].len);
            }
        }
    } else {
        // No valid store — initialise empty
        memset(&s_store, 0, sizeof(s_store));
        s_store.magic = TLV_MAGIC;
        s_store.data_used = 0;
        s_eeprom_present = true;
        // Write the empty store to EEPROM
        mark_all_dirty();
        printf("[TLV] No valid store found (magic=0x%08lx) — initialised empty\n",
               (unsigned long)magic);
    }
}

const btstack_tlv_t *btstack_tlv_eeprom_instance(void) {
    return &s_tlv_impl;
}

void *btstack_tlv_eeprom_context(void) {
    return NULL;  // context not used — state is in static s_store
}

void btstack_tlv_eeprom_flush_if_dirty(void) {
    if (!s_eeprom_present) return;
    if (s_dirty_pages == 0) return;

    // Find the first dirty page and flush it
    for (uint8_t p = 0; p < TLV_NUM_PAGES; p++) {
        if (!(s_dirty_pages & (1u << p))) continue;

        // Compute the byte range for this page within s_store
        uint16_t store_off = (uint16_t)(p * EEPROM_PAGE_SIZE);
        uint16_t page_len  = EEPROM_PAGE_SIZE;
        if (store_off + page_len > TLV_STORE_SIZE) {
            page_len = (uint16_t)(TLV_STORE_SIZE - store_off);
        }

        // Read current RAM content under spinlock
        uint8_t page_buf[EEPROM_PAGE_SIZE];
        uint32_t save = spin_lock_blocking(s_lock);
        memcpy(page_buf, (const uint8_t *)&s_store + store_off, page_len);
        s_dirty_pages &= ~(1u << p);
        spin_unlock(s_lock, save);

        // Write to EEPROM (outside spinlock — I2C takes ~10ms)
        eeprom_write(TLV_EEPROM_BASE + store_off, page_buf, page_len);
        printf("[TLV] Flushed page %u (%u B @ 0x%04x)\n",
               p, page_len, TLV_EEPROM_BASE + store_off);

        // One page per call — keeps CM task responsive
        return;
    }
}
