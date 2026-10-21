// SPDX-License-Identifier: GPL-2.0-only
#include "ble_power.h"
#include "ble_internal.h"
#include "ipc/ipc_core.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"
#include "pico/time.h"
#include <string.h>

static power_radio_t s_radio;
static power_peer_clock_t s_peer;
static uint16_t s_handle=HCI_CON_HANDLE_INVALID, s_off_token, s_peer_request;
static uint32_t s_session;
static uint16_t s_sequence;
static uint64_t s_last_key_sent, s_last_hid_sent;
static uint8_t s_last_flags;
static bool s_up_held[3], s_up_origin[3];
static uint32_t s_peer_matrix;
static uint64_t s_peer_key_hint, s_peer_hid_hint;
static uint64_t later(uint64_t a, uint64_t b) { return a>b?a:b; }

void ble_power_hid(bool origin, unsigned kind, bool held) {
    uint64_t now=time_us_64();
    if (kind<3) { s_up_held[kind]=held; s_up_origin[kind]=origin; }
    if (origin) {
        s_radio.hid_us=now;
        if (kind==0) s_radio.key_us=now;
    } else {
        s_peer_hid_hint=now;
        if (kind==0) s_peer_key_hint=now;
    }
}
void ble_power_clear_upstream(void) {
    memset(s_up_held,0,sizeof(s_up_held));
    s_peer_matrix=0;
}
void ble_power_peer_keys(uint32_t bitmap) {
    if (bitmap!=s_peer_matrix) s_peer_key_hint=time_us_64();
    s_peer_matrix=bitmap;
}
void ble_power_link(uint16_t handle, bool present, bool ready) {
    if (!present) handle=HCI_CON_HANDLE_INVALID;
    if (s_handle!=handle) {
        s_handle=handle;
        s_peer=(power_peer_clock_t){0}; s_peer_request=0;
        s_radio.peer_ack=0; s_radio.peer_sample_us=0; s_radio.peer_held=false;
        // Token scope is the BLE connection; reconnect discards old RX state.
        s_session=(uint32_t)time_us_64(); if (!s_session) s_session=1;
        s_sequence=0; s_last_key_sent=s_last_hid_sent=0; s_last_flags=0xff;
    }
    s_radio.peer_connected=present;
    s_radio.peer_ready=ready && s_peer.valid;
}
bool ble_power_receive(uint16_t handle, const uint8_t *data, unsigned len) {
    if (handle!=s_handle || handle==HCI_CON_HANDLE_INVALID || gap_encryption_key_size(handle)<16) return false;
    power_wire_t m;
    if (!power_wire_decode(&m,data,len)) return false;
    // A session may change only on reconnect: delayed frames can't resurrect an
    // old session on the same authenticated connection.
    if (s_peer.valid && s_peer.session!=m.session) return false;
    if (!power_peer_update(&s_peer,&m,time_us_64())) return false;
    s_radio.peer_sample_us=time_us_64();
    s_radio.peer_key_us=later(s_peer.key_us,s_peer_key_hint);
    s_radio.peer_hid_us=later(s_peer.hid_us,s_peer_hid_hint);
    s_radio.peer_held=(m.flags & (POWER_WIRE_HELD|POWER_WIRE_PENDING))!=0;
    s_radio.peer_ack=(m.flags & POWER_WIRE_IDLE) ? m.ack : 0;
    s_peer_request=m.request;
    s_radio.peer_ready=true;
    return true;
}
void ble_power_frame(uint8_t out[POWER_WIRE_SIZE]) {
    power_state_t p; ipc_power_get(&p);
    uint64_t now=time_us_64();
    uint64_t key=later(p.boot_us,later(p.local.key_us,s_radio.key_us));
    uint64_t hid=s_radio.hid_us;
    bool held=p.local.held;
    for (unsigned i=0;i<3;++i) held |= s_up_held[i] && s_up_origin[i];
    // Do not include a sibling-held button in own-origin state (no echo).
    p.radio.held=held;
    uint8_t flags=(held?POWER_WIRE_HELD:0) | (s_radio.pending?POWER_WIRE_PENDING:0);
    // Bug #2: the window in force comes from Core 0 (eeconfig). 0 = DEEP
    // disabled, and then we are never "idle enough" to let the peer shut down.
    uint32_t deep_ms=p.local.deep_ms;
    if (deep_ms && power_origin_idle(&p,now,deep_ms)) flags|=POWER_WIRE_IDLE;
    // Only own-origin changes advance the sequence. ACK/refresh/peer events do not.
    if (key!=s_last_key_sent || hid!=s_last_hid_sent || (flags&3)!=(s_last_flags&3)) {
        ++s_sequence; s_last_key_sent=key; s_last_hid_sent=hid; s_last_flags=flags;
    }
    power_wire_t m={
        .session=s_session, .sequence=s_sequence, .flags=flags,
        .key_age_ms=power_age_ms(now,key), .hid_age_ms=power_age_ms(now,hid),
        .request=p.phase==POWER_REQUESTED ? p.token : 0,
        .ack=(flags&POWER_WIRE_IDLE) ? s_peer_request : 0
    };
    power_wire_encode(out,&m);
}
void ble_power_request_off(uint16_t token) { s_off_token=token; }
void ble_power_poll(void) {
    s_radio.sample_us=time_us_64();
    s_radio.held=s_up_held[0] || s_up_held[1] || s_up_held[2];
    s_radio.pending=ble_relay_power_busy() || ble_split_power_busy();
    s_radio.peer_key_us=later(s_radio.peer_key_us,s_peer_key_hint);
    s_radio.peer_hid_us=later(s_radio.peer_hid_us,s_peer_hid_hint);
    if (s_peer_matrix) s_radio.held=true;
    ble_split_power_poll(); // link state and bounded outgoing heartbeat (250 ms)
    ipc_power_radio(&s_radio);
#ifdef LOW_POWER_ENABLE
    power_state_t pc; ipc_power_get(&pc);
    if (s_off_token && pc.local.deep_ms &&
        ipc_power_commit(s_off_token,time_us_64(),pc.local.deep_ms)) {
        C1_DBG_STATUS("[LP] commit %u: radio shutdown\n",s_off_token);
        hci_power_control(HCI_POWER_OFF);
        btstack_run_loop_trigger_exit();
    }
#endif
}
