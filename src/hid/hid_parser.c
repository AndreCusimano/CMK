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
// hid_parser.c — HID report descriptor parser + mouse report decoder
//

#include "hid_parser.h"
#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Short-item decoding helpers
// ---------------------------------------------------------------------------

#define ITEM_SIZE(b)   ((b) & 0x03u)           // 0=0B 1=1B 2=2B 3=4B
#define ITEM_TYPE(b)   (((b) >> 2u) & 0x03u)   // 0=Main 1=Global 2=Local
#define ITEM_TAG(b)    (((b) >> 4u) & 0x0Fu)
#define ITEM_NBYTES(s) ((uint8_t)((s) == 3u ? 4u : (s)))

// Item type constants
#define TYPE_MAIN    0u
#define TYPE_GLOBAL  1u
#define TYPE_LOCAL   2u

// Main tags
#define TAG_INPUT       8u
#define TAG_OUTPUT      9u
#define TAG_FEATURE    11u
#define TAG_COLLECTION 10u
#define TAG_END_COLL   12u

// Global tags
#define TAG_USAGE_PAGE  0u
#define TAG_LOG_MIN     1u
#define TAG_LOG_MAX     2u
#define TAG_RPT_SIZE    7u
#define TAG_RPT_ID      8u
#define TAG_RPT_COUNT   9u
#define TAG_PUSH       10u
#define TAG_POP        11u

// Local tags
#define TAG_USAGE       0u
#define TAG_USAGE_MIN   1u
#define TAG_USAGE_MAX   2u

static uint32_t item_uint(const uint8_t *p, uint8_t sc) {
    uint8_t n = ITEM_NBYTES(sc);
    uint32_t v = 0;
    for (uint8_t i = 0; i < n; i++) v |= ((uint32_t)p[i]) << (8u * i);
    return v;
}

static int32_t item_int(const uint8_t *p, uint8_t sc) {
    uint8_t n = ITEM_NBYTES(sc);
    uint32_t raw = item_uint(p, sc);
    if (n > 0u && n < 4u) {
        uint32_t msb = 1u << (n * 8u - 1u);
        if (raw & msb) raw |= ~((msb << 1u) - 1u);
    }
    return (int32_t)raw;
}

// ---------------------------------------------------------------------------
// Per-report Input bit-offset tracking
// ---------------------------------------------------------------------------

#define MAX_RPT_TRACK 8

typedef struct { uint8_t id; uint32_t bits; } rpt_bits_t;

static uint32_t *get_input_bits(rpt_bits_t *tbl, uint8_t *cnt, uint8_t id) {
    for (uint8_t i = 0; i < *cnt; i++)
        if (tbl[i].id == id) return &tbl[i].bits;
    if (*cnt < MAX_RPT_TRACK) {
        tbl[*cnt].id   = id;
        tbl[*cnt].bits = 0;
        return &tbl[(*cnt)++].bits;
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// Parser local state (reset after each Main item)
// ---------------------------------------------------------------------------

#define MAX_LOCAL_USAGES 16

typedef struct {
    // Global (persist across items)
    uint16_t usage_page;
    int32_t  logical_min;
    int32_t  logical_max;
    uint8_t  report_size;
    uint8_t  report_count;
    uint8_t  report_id;
    // Local (reset after each Main item)
    uint32_t usages[MAX_LOCAL_USAGES];
    uint8_t  usage_count;
    uint32_t usage_range_min;
    uint32_t usage_range_max;
    bool     has_range;
} parse_state_t;

static void local_reset(parse_state_t *s) {
    s->usage_count = 0;
    s->has_range   = false;
}

// Global state snapshot for PUSH/POP items (commercial mouse descriptors —
// Logitech et al. — use them around button/axis declarations; without support
// the usage page leaks and the buttons field stops matching the Button page).
#define GLOBAL_STACK_MAX 4

typedef struct {
    uint16_t usage_page;
    int32_t  logical_min;
    int32_t  logical_max;
    uint8_t  report_size;
    uint8_t  report_count;
    uint8_t  report_id;
} global_state_t;

static void global_save(global_state_t *g, const parse_state_t *s) {
    g->usage_page   = s->usage_page;
    g->logical_min  = s->logical_min;
    g->logical_max  = s->logical_max;
    g->report_size  = s->report_size;
    g->report_count = s->report_count;
    g->report_id    = s->report_id;
}

static void global_restore(parse_state_t *s, const global_state_t *g) {
    s->usage_page   = g->usage_page;
    s->logical_min  = g->logical_min;
    s->logical_max  = g->logical_max;
    s->report_size  = g->report_size;
    s->report_count = g->report_count;
    s->report_id    = g->report_id;
}

// Build a full (page << 16 | id) usage.
// For 1-byte usage items: combine with current usage_page.
// For 2-byte usage items: value may be an extended usage (page|id in upper/lower 16 bits)
//   or just a 16-bit usage id under the current page.  We treat values > 0xFF that fit
//   in 16 bits as usage-id only (page taken from current usage_page state), which is
//   the correct interpretation for the Consumer page where usage IDs like 0x0238 are common.
static uint32_t full_usage(const parse_state_t *s, uint32_t raw, uint8_t sc) {
    if (sc <= 1u) {
        return HID_USAGE(s->usage_page, raw);
    }
    // 2-byte item: if upper 16 bits are set it is an extended usage (4-byte encoding
    // collapsed to 2 bytes is unusual, but handle it).  Otherwise it is a 16-bit usage
    // id under the current page.
    if (raw & 0xFFFF0000u) return raw;               // already has page in high word
    return HID_USAGE(s->usage_page, raw & 0xFFFFu);  // combine with current page
}

// ---------------------------------------------------------------------------
// int hid_parse_descriptor()
// ---------------------------------------------------------------------------

int hid_parse_descriptor(const uint8_t *desc, uint16_t len, hid_report_map_t *out) {
    out->count = 0;

    parse_state_t s;
    memset(&s, 0, sizeof(s));

    rpt_bits_t rpt_tbl[MAX_RPT_TRACK];
    uint8_t    rpt_cnt = 0;
    memset(rpt_tbl, 0, sizeof(rpt_tbl));

    global_state_t g_stack[GLOBAL_STACK_MAX];
    uint8_t        g_sp = 0;

    uint16_t pos = 0;
    while (pos < len) {
        uint8_t prefix = desc[pos++];

        // Long item (0xFE): skip entirely
        if (prefix == 0xFEu) {
            if (pos >= len) break;
            uint8_t lsz = desc[pos++];
            pos += 1u + lsz;
            continue;
        }

        uint8_t sc   = ITEM_SIZE(prefix);
        uint8_t type = ITEM_TYPE(prefix);
        uint8_t tag  = ITEM_TAG(prefix);
        uint8_t nb   = ITEM_NBYTES(sc);

        if (pos + nb > len) break;
        const uint8_t *dp = desc + pos;
        pos += nb;

        // --- Global items ---
        if (type == TYPE_GLOBAL) {
            switch (tag) {
            case TAG_USAGE_PAGE: s.usage_page   = (uint16_t)item_uint(dp, sc); break;
            case TAG_LOG_MIN:    s.logical_min  = item_int(dp, sc);            break;
            case TAG_LOG_MAX:    s.logical_max  = item_int(dp, sc);            break;
            case TAG_RPT_SIZE:   s.report_size  = (uint8_t)item_uint(dp, sc); break;
            case TAG_RPT_COUNT:  s.report_count = (uint8_t)item_uint(dp, sc); break;
            case TAG_RPT_ID:     s.report_id    = (uint8_t)item_uint(dp, sc); break;
            case TAG_PUSH:
                if (g_sp < GLOBAL_STACK_MAX) global_save(&g_stack[g_sp++], &s);
                break;
            case TAG_POP:
                if (g_sp > 0) global_restore(&s, &g_stack[--g_sp]);
                break;
            default: break;
            }

        // --- Local items ---
        } else if (type == TYPE_LOCAL) {
            uint32_t fu = full_usage(&s, item_uint(dp, sc), sc);
            switch (tag) {
            case TAG_USAGE:
                if (s.usage_count < MAX_LOCAL_USAGES)
                    s.usages[s.usage_count++] = fu;
                break;
            case TAG_USAGE_MIN:
                s.usage_range_min = fu;
                s.has_range       = true;
                break;
            case TAG_USAGE_MAX:
                s.usage_range_max = fu;
                s.has_range       = true;
                break;
            default: break;
            }

        // --- Main items ---
        } else if (type == TYPE_MAIN) {
            uint32_t total_bits = (uint32_t)s.report_size * s.report_count;

            if (tag == TAG_INPUT) {
                uint8_t   flags     = (uint8_t)item_uint(dp, sc);
                uint32_t *ibits_ptr = get_input_bits(rpt_tbl, &rpt_cnt, s.report_id);
                uint32_t  bit_start = ibits_ptr ? *ibits_ptr : 0u;

                if (!(flags & HID_INPUT_CONSTANT)) {
                    // Variable with explicit usage list → one field per usage
                    bool use_list = (bool)(flags & HID_INPUT_VARIABLE)
                                 && !s.has_range
                                 && (s.usage_count > 0);

                    if (use_list) {
                        for (uint8_t i = 0; i < s.report_count; i++) {
                            if (out->count >= HID_FIELD_MAX) { out->count = HID_FIELD_MAX; break; }
                            hid_field_t *f = &out->fields[out->count++];
                            f->report_id   = s.report_id;
                            f->bit_offset  = bit_start + (uint32_t)(i * s.report_size);
                            f->bit_count   = s.report_size;
                            f->logical_min = s.logical_min;
                            f->logical_max = s.logical_max;
                            f->input_flags = flags;
                            f->usage_first = (i < s.usage_count) ? s.usages[i] : 0u;
                            f->usage_last  = f->usage_first;
                        }
                    } else {
                        // Usage range or array → one aggregate field
                        if (out->count < HID_FIELD_MAX) {
                            hid_field_t *f = &out->fields[out->count++];
                            f->report_id   = s.report_id;
                            f->bit_offset  = bit_start;
                            f->bit_count   = (total_bits <= 255u) ? (uint8_t)total_bits : 255u;
                            f->logical_min = s.logical_min;
                            f->logical_max = s.logical_max;
                            f->input_flags = flags;
                            if (s.has_range) {
                                f->usage_first = s.usage_range_min;
                                f->usage_last  = s.usage_range_max;
                            } else {
                                f->usage_first = (s.usage_count > 0) ? s.usages[0]                    : 0u;
                                f->usage_last  = (s.usage_count > 0) ? s.usages[s.usage_count - 1] : 0u;
                            }
                        }
                    }
                }
                // Always advance the Input bit counter (including Constant padding)
                if (ibits_ptr) *ibits_ptr += total_bits;
            }
            // Output / Feature items: do NOT advance the Input bit counter.
            local_reset(&s);
        }
    }

    return (out->count >= HID_FIELD_MAX) ? -1 : (int)out->count;
}

// ---------------------------------------------------------------------------
// int32_t hid_extract_bits()
// ---------------------------------------------------------------------------

int32_t hid_extract_bits(const uint8_t *data, uint16_t data_len,
                          uint32_t bit_offset, uint8_t bit_count,
                          int32_t logical_min) {
    if (bit_count == 0u) return 0;
    // Aggregate fields from a malformed/hostile upstream descriptor can claim up
    // to 255 bits; shifting past 31 is UB. Clamp to the 32-bit accumulator.
    if (bit_count > 32u) bit_count = 32u;

    uint32_t raw = 0u;
    for (uint8_t i = 0; i < bit_count; i++) {
        uint32_t byte_idx = (bit_offset + i) >> 3u;
        uint8_t  bit_idx  = (uint8_t)((bit_offset + i) & 7u);
        if (byte_idx < data_len && ((data[byte_idx] >> bit_idx) & 1u))
            raw |= 1u << i;
    }

    // Sign-extend if the field is signed (logical_min < 0)
    if (logical_min < 0 && bit_count < 32u) {
        if (raw & (1u << (bit_count - 1u)))
            raw |= ~((1u << bit_count) - 1u);
    }

    return (int32_t)raw;
}

// ---------------------------------------------------------------------------
// bool hid_decode_mouse()
// ---------------------------------------------------------------------------

bool hid_decode_mouse(const hid_report_map_t *map, uint8_t report_id,
                      const uint8_t *data, uint16_t data_len,
                      hid_mouse_report_t *out) {
    memset(out, 0, sizeof(*out));
    bool found = false;

    for (uint8_t i = 0; i < map->count; i++) {
        const hid_field_t *f = &map->fields[i];
        if (f->report_id != report_id) continue;

        uint16_t page = HID_USAGE_PAGE(f->usage_first);

        // --- Buttons (Usage Page 0x09) ---
        if (page == HID_USAGE_BTN_PAGE) {
            uint16_t first_btn = HID_USAGE_ID(f->usage_first);
            if (f->bit_count == 1u && first_btn >= 1u && first_btn <= 8u) {
                // Single button bit: place at the correct output position
                int32_t val = hid_extract_bits(data, data_len, f->bit_offset, 1u, 0);
                out->buttons |= (uint8_t)(val << (first_btn - 1u));
            } else {
                // Aggregate block (usage range): extract all bits as a bitmask
                // Shift so Button N lands on bit (N-1) of out->buttons.
                // first_btn comes from the (untrusted) descriptor: a block whose
                // first usage is past Button 8 can't land in the 8-bit output —
                // skip it instead of shifting by up to 65533 bits (UB).
                if (first_btn > 8u) continue;
                uint8_t shift = (first_btn >= 1u) ? (uint8_t)(first_btn - 1u) : 0u;
                int32_t raw   = hid_extract_bits(data, data_len, f->bit_offset, f->bit_count, 0);
                out->buttons |= (uint8_t)((raw << shift) & 0xFFu);
            }
            found = true;

        // --- X axis ---
        } else if (f->usage_first == HID_USAGE_X) {
            out->x = (int16_t)hid_extract_bits(data, data_len,
                                                f->bit_offset, f->bit_count,
                                                f->logical_min);
            found = true;

        // --- Y axis ---
        } else if (f->usage_first == HID_USAGE_Y) {
            out->y = (int16_t)hid_extract_bits(data, data_len,
                                                f->bit_offset, f->bit_count,
                                                f->logical_min);
            found = true;

        // --- Vertical wheel ---
        } else if (f->usage_first == HID_USAGE_WHEEL) {
            out->wheel = (int8_t)hid_extract_bits(data, data_len,
                                                   f->bit_offset, f->bit_count,
                                                   f->logical_min);
            found = true;

        // --- Horizontal scroll: AC Pan or Dial ---
        } else if (f->usage_first == HID_USAGE_AC_PAN ||
                   f->usage_first == HID_USAGE_DIAL) {
            out->pan = (int8_t)hid_extract_bits(data, data_len,
                                                 f->bit_offset, f->bit_count,
                                                 f->logical_min);
            found = true;
        }
    }

    return found;
}

// ---------------------------------------------------------------------------
// bool hid_report_is_mouse()
// ---------------------------------------------------------------------------

bool hid_report_is_mouse(const hid_report_map_t *map, uint8_t report_id) {
    for (uint8_t i = 0; i < map->count; i++) {
        const hid_field_t *f = &map->fields[i];
        if (f->report_id != report_id) continue;
        if (f->usage_first == HID_USAGE_X || f->usage_first == HID_USAGE_Y) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// void hid_pack_mouse()
// ---------------------------------------------------------------------------

void hid_pack_mouse(const hid_mouse_report_t *in, uint8_t out[7]) {
    uint16_t ux = (uint16_t)in->x;
    uint16_t uy = (uint16_t)in->y;
    out[0] = in->buttons;
    out[1] = (uint8_t)(ux & 0xFFu);         // X_L
    out[2] = (uint8_t)((ux >> 8u) & 0xFFu); // X_H
    out[3] = (uint8_t)(uy & 0xFFu);         // Y_L
    out[4] = (uint8_t)((uy >> 8u) & 0xFFu); // Y_H
    out[5] = (uint8_t)in->wheel;
    out[6] = (uint8_t)in->pan;
}
