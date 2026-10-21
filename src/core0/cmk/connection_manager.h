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
// connection_manager.h — Core 0 FreeRTOS task: BLE/USB connection state management
//
// Manages:
//   - Upstream  (Central role):  scansione e connessione al device precedente nella catena
//   - Downstream BLE (Peripheral role): advertising e connessione al device successivo
//   - Downstream USB (futuro TinyUSB): output HID via USB verso il PC
//
// Comunica con Core 1 (BTstack) esclusivamente via IPC (multicore_fifo).
// Legge bonding_store per decidere auto-scan al boot.
//

#ifndef CMK_CONNECTION_MANAGER_H
#define CMK_CONNECTION_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// State types
// ---------------------------------------------------------------------------

typedef enum {
    CM_UPSTREAM_IDLE,        // nessuna attività upstream
    CM_UPSTREAM_SCANNING,    // IPC_START_SCAN inviato, in attesa di device
    CM_UPSTREAM_CONNECTING,  // device trovato, connessione/pairing in corso
    CM_UPSTREAM_CONNECTED,   // HID service upstream pronto, relay attivo
} cm_upstream_state_t;

typedef enum {
    CM_DS_BLE_IDLE,          // non sta advertisando
    CM_DS_BLE_ADVERTISING,   // general ADV (undirected) → fast blink LED
    CM_DS_BLE_ADV_DIRECTED,  // directed ADV high duty (1.28s) → slow blink LED
    CM_DS_BLE_CONNECTED,     // downstream BLE connesso → solid LED
} cm_ds_ble_state_t;

typedef enum {
    // Futuro TinyUSB output
    CM_DS_USB_IDLE,          // USB non connesso
    CM_DS_USB_CONNECTED,     // USB host connesso e subscribed
} cm_ds_usb_state_t;

// Stato PER-SLOT downstream (R5) — derivato da stato DS globale + slot pendente
// + bonding_store. Prima esisteva solo lo stato globale "active-centric": durante
// un ADV su uno slot diverso da quello attivo il LED lampeggiava sul tasto
// SBAGLIATO (lo slot attivo) mentre quello davvero in pairing restava spento.
typedef enum {
    CM_SLOT_EMPTY = 0,       // nessun bond salvato
    CM_SLOT_SAVED,           // bond salvato, non attivo ora
    CM_SLOT_ADV_GENERAL,     // pairing nuovo in corso su questo slot (fast blink)
    CM_SLOT_ADV_DIRECTED,    // reconnect directed in corso su questo slot (slow blink)
    CM_SLOT_CONNECTED,       // connessione viva su questo slot (solid)
} cm_ds_slot_state_t;

// Ruolo split runtime (Fase 18), deciso dal BLE su Core 1 e notificato via IPC.
typedef enum {
    CM_SPLIT_STANDALONE = 0, // half singola/normale (comportamento attuale)
    CM_SPLIT_MASTER,         // risolve la keymap 4×12 unificata per entrambe
    CM_SPLIT_SLAVE,          // invia solo eventi raw di matrice al master
    CM_SPLIT_MISMATCH,       // sibling CMK con split_mode discordante (strict matching):
                             // half autonoma, nessuna cooperazione — Core 1 scarta i
                             // report HID della sibling finché gli stati non combaciano
} cm_split_role_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Inizializza il modulo (call da main() prima di vTaskStartScheduler).
// I GPIO dei pulsanti devono essere già init in main().
void connection_manager_init(void);

// Crea il FreeRTOS task. Call da main() prima di vTaskStartScheduler.
void connection_manager_create_task(void);

// Accessors thread-safe per lo stato corrente.
cm_upstream_state_t connection_manager_upstream_state(void);
cm_ds_ble_state_t   connection_manager_ds_ble_state(void);
cm_ds_usb_state_t   connection_manager_ds_usb_state(void);

// Stato per-slot DS (R5) — per LED/overlay: lo slot in ADV è quello PENDENTE,
// lo slot connesso è quello attivo del bonding_store.
cm_ds_slot_state_t  connection_manager_ds_slot_state(uint8_t slot);

// Split runtime role (Fase 18) + the peer half's board side (valido se MASTER).
// Letti dal keyscan task ad ogni ciclo per decidere il path standalone/master/slave.
cm_split_role_t connection_manager_split_role(void);
uint8_t         connection_manager_remote_side(void);

// Bug #3: livelli batteria visti via BLE (sibling su CMK-Split, mouse su BAS
// client), campionati dallo status slot R3. 0xFF = sconosciuto. Letture uint8
// atomiche — chiamabile dal task USB (config protocol GET_BATTERY).
void connection_manager_get_ble_batt(uint8_t *peer_out, uint8_t *mouse_out);

// ---------------------------------------------------------------------------
// Azioni combo — chiamate da cmk_keycodes.c (Core 0 only)
// ---------------------------------------------------------------------------

// Td{slot} short: directed ADV verso l'addr salvato per slot (se presente e non connesso).
// Returns false se l'addr non è presente o è già connesso → keyscan ignora.
bool connection_manager_ds_adv_directed(uint8_t slot);

// Td{slot} long: cancella addr slot + general ADV. Salverà l'addr del device che si connette.
void connection_manager_ds_adv_general(uint8_t slot);

// Tu long: cancella addr upstream + general scan. Salverà il nuovo addr se trovato.
void connection_manager_us_scan_general(void);

// Tu short: re-link manuale dell'upstream (speculare a Td short) — reconnect
// diretto se esiste il bond, scan generale altrimenti. No-op se già
// connesso/in connessione. Azzera il backoff rapid-disconnect.
void connection_manager_us_reconnect(void);

// EE_CLR long: factory reset — cancella tutti i bond (DS slot 0/1/2 + US) E le relative
// chiavi SM nel TLV (via IPC_DELETE_BONDING su Core 1). Differito al CM task.
void connection_manager_clear_all_bonds(void);



#endif // CMK_CONNECTION_MANAGER_H
