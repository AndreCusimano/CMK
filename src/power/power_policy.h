// SPDX-License-Identifier: GPL-2.0-only
#pragma once
#include <stdbool.h>
#include <stdint.h>

// All times are Pico monotonic microseconds. Access through ipc_power_* only.
enum { POWER_RUNNING, POWER_REQUESTED, POWER_COMMITTED, POWER_OFF };
enum { POWER_VETO_USB=1, POWER_VETO_KEYS=2, POWER_VETO_HID=4,
       POWER_VETO_BUSY=8, POWER_VETO_STATUS=16, POWER_VETO_PEER=32 };
typedef struct {
    uint64_t key_us, remote_key_us, sample_us;
    bool held, usb, ready;
    // Bug #2: the DEEP window in force, in ms (0 = tier disabled). Core 0 owns
    // it (eeconfig) and publishes it here every tick; Core 1 must use THIS value
    // and never the compile-time default, or the two cores judge idleness by
    // different rules — Core 0 requesting the shutdown while Core 1 refuses to
    // commit it, or committing one the user's timeout no longer justifies.
    // Core 1 never reads EEPROM, so this field is its only source.
    uint32_t deep_ms;
} power_local_t;
typedef struct {
    uint64_t key_us, hid_us, sample_us, peer_key_us, peer_hid_us, peer_sample_us;
    bool held, pending, peer_connected, peer_ready, peer_held;
    uint16_t peer_ack;
} power_radio_t;
typedef struct {
    power_local_t local;
    power_radio_t radio;
    uint64_t boot_us, request_us, commit_us;
    uint16_t token;
    uint8_t phase;
} power_state_t;

uint64_t power_last_key(const power_state_t *s);
uint8_t power_veto(const power_state_t *s, uint64_t now, uint32_t deep_ms);
// Own-origin activity only (never echo activity received from the sibling).
bool power_origin_idle(const power_state_t *s, uint64_t now, uint32_t deep_ms);
uint16_t power_request(power_state_t *s, uint64_t now, uint32_t deep_ms);
void power_cancel(power_state_t *s);
bool power_commit(power_state_t *s, uint16_t token, uint64_t now, uint32_t deep_ms);

// 20-byte, MTU-23 compatible inter-half message. Ages, NOT foreign timestamps.
#define POWER_WIRE_SIZE 20
#define POWER_WIRE_VERSION 1
#define POWER_WIRE_HELD 1u
#define POWER_WIRE_PENDING 2u
#define POWER_WIRE_IDLE 4u
typedef struct {
    uint32_t session, key_age_ms, hid_age_ms;
    uint16_t sequence, request, ack;
    uint8_t flags;
} power_wire_t;
typedef struct {
    uint32_t session;
    uint16_t sequence;
    uint64_t key_us, hid_us;
    bool valid;
} power_peer_clock_t;
void power_wire_encode(uint8_t out[POWER_WIRE_SIZE], const power_wire_t *m);
bool power_wire_decode(power_wire_t *m, const uint8_t *in, unsigned len);
// Ignore out-of-order frames; duplicate refreshes never move activity forward.
bool power_peer_update(power_peer_clock_t *clock, const power_wire_t *m, uint64_t now);
uint32_t power_age_ms(uint64_t now, uint64_t then);
