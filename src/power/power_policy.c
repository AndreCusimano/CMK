// SPDX-License-Identifier: GPL-2.0-only
#include "power_policy.h"
static uint64_t max64(uint64_t a, uint64_t b) { return a > b ? a : b; }
static bool recent(uint64_t now, uint64_t then, uint64_t window) {
    return now < then || now - then < window;
}
uint64_t power_last_key(const power_state_t *s) {
    return max64(max64(s->boot_us, s->local.key_us),
           max64(s->local.remote_key_us, max64(s->radio.key_us, s->radio.peer_key_us)));
}
bool power_origin_idle(const power_state_t *s, uint64_t now, uint32_t deep_ms) {
    uint64_t last = max64(s->boot_us, max64(s->local.key_us, max64(s->radio.key_us, s->radio.hid_us)));
    return s->local.ready && recent(now,s->local.sample_us,1000000) && !s->local.held && !s->radio.held && !s->radio.pending &&
           !recent(now, last, (uint64_t)deep_ms * 1000);
}
uint8_t power_veto(const power_state_t *s, uint64_t now, uint32_t deep_ms) {
    uint8_t v = 0;
    uint64_t window = (uint64_t)deep_ms * 1000;
    if (s->local.usb) v |= POWER_VETO_USB;
    if (s->local.held || recent(now, power_last_key(s), window)) v |= POWER_VETO_KEYS;
    if (recent(now, max64(s->boot_us, max64(s->radio.hid_us, s->radio.peer_hid_us)), window)) v |= POWER_VETO_HID;
    if (s->radio.held || s->radio.pending || s->radio.peer_held) v |= POWER_VETO_BUSY;
    if (!s->local.ready || !s->radio.sample_us ||
        !recent(now, s->local.sample_us, 1000000) || !recent(now, s->radio.sample_us, 1000000)) v |= POWER_VETO_STATUS;
    if (s->radio.peer_connected && (!s->radio.peer_ready ||
        !recent(now, s->radio.peer_sample_us, 1500000))) v |= POWER_VETO_PEER;
    return v;
}
uint16_t power_request(power_state_t *s, uint64_t now, uint32_t deep_ms) {
    if (s->phase != POWER_RUNNING || power_veto(s, now, deep_ms)) return 0;
    if (++s->token == 0) ++s->token;
    s->request_us = now;
    s->phase = POWER_REQUESTED;
    return s->token;
}
void power_cancel(power_state_t *s) {
    if (s->phase == POWER_REQUESTED) s->phase = POWER_RUNNING;
}
bool power_commit(power_state_t *s, uint16_t token, uint64_t now, uint32_t deep_ms) {
    if (s->phase != POWER_REQUESTED || token != s->token || power_veto(s, now, deep_ms)) return false;
    if (s->radio.peer_connected && s->radio.peer_ack != token) return false;
    s->phase = POWER_COMMITTED;
    s->commit_us = now;
    return true;
}
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, uint32_t v) { put16(p,(uint16_t)v); put16(p+2,(uint16_t)(v>>16)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0]|(uint16_t)p[1]<<8); }
static uint32_t get32(const uint8_t *p) { return get16(p)|(uint32_t)get16(p+2)<<16; }
void power_wire_encode(uint8_t out[POWER_WIRE_SIZE], const power_wire_t *m) {
    out[0]=POWER_WIRE_VERSION; out[1]=m->flags; put16(out+2,m->sequence);
    put32(out+4,m->key_age_ms); put32(out+8,m->hid_age_ms);
    put16(out+12,m->request); put16(out+14,m->ack); put32(out+16,m->session);
}
bool power_wire_decode(power_wire_t *m, const uint8_t *in, unsigned len) {
    if (len!=POWER_WIRE_SIZE || in[0]!=POWER_WIRE_VERSION || (in[1]&~7u)) return false;
    m->flags=in[1]; m->sequence=get16(in+2); m->key_age_ms=get32(in+4);
    m->hid_age_ms=get32(in+8); m->request=get16(in+12); m->ack=get16(in+14); m->session=get32(in+16);
    return m->session != 0;
}
uint32_t power_age_ms(uint64_t now, uint64_t then) {
    if (!then) return UINT32_MAX;
    uint64_t age = now >= then ? (now-then)/1000 : 0;
    return age >= UINT32_MAX ? UINT32_MAX-1 : (uint32_t)age;
}
static uint64_t from_age(uint64_t now, uint32_t age) {
    if (age == UINT32_MAX) return 0;
    uint64_t us = (uint64_t)age*1000;
    return now > us ? now-us : 1;
}
bool power_peer_update(power_peer_clock_t *c, const power_wire_t *m, uint64_t now) {
    if (c->valid && c->session == m->session) {
        uint16_t distance = (uint16_t)(m->sequence-c->sequence);
        if (distance >= 0x8000) return false;
        if (!distance) return true; // refresh/ACK only, not new activity
    }
    c->session=m->session; c->sequence=m->sequence; c->valid=true;
    c->key_us=from_age(now,m->key_age_ms); c->hid_us=from_age(now,m->hid_age_ms);
    return true;
}
