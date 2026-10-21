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
// core1_entry.c — Core 1 entry point
//
// Core 1 is dedicated to BTstack + CYW43. No FreeRTOS.
//
// Sequence:
//   1. cyw43_arch_init()  — init CYW43439 with pico_cyw43_arch_none (IRQ-based async_context)
//   2. btstack_cyw43_init() — bind BTstack to async_context
//   3. ble_manager_init()   — register profiles and event handlers
//   4. hci_power_control()  — turn on HCI, triggers BTSTACK_EVENT_STATE/HCI_STATE_WORKING
//   5. Signal Core 0 via IPC: IPC_BLE_READY
//   6. BTstack run loop takes over — IPC polling runs as a BTstack repeating timer
//
// IPC polling runs inside the BTstack run loop as a periodic timer callback.
// This avoids acquiring async_context_lock (which disables IRQs on
// async_context_threadsafe_background), preventing BLE radio timing loss.
//

#include "ble/ble_manager.h"
#include "ipc/ipc_core.h"
#include "ble/ble_power.h"

#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "btstack_memory.h"
#include "btstack_tlv.h"
#include "tlv/btstack_tlv_eeprom.h"
#include "ble/le_device_db_tlv.h"
#include "pico/btstack_run_loop_async_context.h"
#include "pico/btstack_hci_transport_cyw43.h"

// IPC polling interval (ms) — fast enough for responsive button handling,
// slow enough to not interfere with BLE timing.
#define IPC_POLL_INTERVAL_MS  5

// ---------------------------------------------------------------------------
// Core 1 stack canary — detects stack overflow (no FreeRTOS on this core).
// The canary sits at the BOTTOM of the stack (lowest address).
// main.c paints the first 16 words with CANARY_PATTERN before launching Core 1.
// The IPC poll timer checks periodically; if overwritten → overflow detected.
// ---------------------------------------------------------------------------
#define STACK_CANARY_PATTERN  0xDEADBEEFu
#define STACK_CANARY_WORDS    16

extern uint32_t core1_stack[];
extern const size_t core1_stack_words;

// BTstack timer callback: polls IPC FIFO and dispatches to ble_manager.
// Runs inside the BTstack run loop — no async_context lock needed.
static void ipc_poll_timer_handler(btstack_timer_source_t *ts) {
    // Stack canary check — detect Core 1 stack overflow
    for (int i = 0; i < STACK_CANARY_WORDS; i++) {
        if (core1_stack[i] != STACK_CANARY_PATTERN) {
            core1_log("\n*** CORE 1 STACK OVERFLOW (canary[%d] = 0x%08lx) ***\n",
                      i, (unsigned long)core1_stack[i]);
            while (1) { __asm volatile("nop"); }
        }
    }

    // Process ONE message per tick — never call gap_connect() and
    // gap_advertisements_enable() in the same BTstack callback.
    // CYW43 rejects HCI LE Set Advertising Enable with 0x0C (Command Disallowed)
    // while LE Create Connection is outstanding (Initiating state active).
    // Spreading messages across 5 ms ticks keeps the HCI pipeline clear.
    uint32_t msg;
    if (ipc_pop_from_core0(&msg)) {
        ble_manager_handle_ipc(msg);
    }

    // R3: poll the report slots/rings and publish the BLE status snapshot every
    // tick, independent of the FIFO. Not an HCI command — safe alongside the
    // one-message-per-tick rule above (it only stages reports / requests
    // can-send-now and copies 5 bytes of state).
    ble_manager_ipc_slots_poll();
    ble_power_poll();

    // Re-arm the timer
    btstack_run_loop_set_timer(ts, IPC_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

void core1_entry(void) {
    core1_log("[Core1] Starting — init CYW43...\n");

    // Init CYW43 with pico_cyw43_arch_none:
    // Uses async_context_threadsafe_background (IRQ-driven, no FreeRTOS task).
    // All CYW43 IRQs run on Core 1 since we call cyw43_arch_init() here.
    int cyw_rc = cyw43_arch_init();
    core1_log("[Core1] cyw43_arch_init=%d\n", cyw_rc);
    if (cyw_rc != 0) {
        core1_log("[Core1] ERROR: cyw43_arch_init failed (rc=%d)!\n", cyw_rc);
        ipc_push_to_core0(IPC_ENCODE(IPC_BLE_READY, 0xFF));
        return;
    }

    // Retrieve the async_context created by cyw43_arch_init
    async_context_t *ctx = cyw43_arch_async_context();

    // Manual BTstack init — replaces btstack_cyw43_init(ctx) to avoid setup_tlv().
    //
    // btstack_cyw43_init() calls setup_tlv() which calls btstack_tlv_flash_bank_init_instance().
    // On first boot that erases + writes a flash sector via flash_safe_execute(), which requires
    // the other core to cooperate via an IRQ. With FreeRTOS on Core 0 that IRQ is masked → assert.
    //
    // We replicate the three useful lines of btstack_cyw43_init() and skip setup_tlv entirely,
    // using an in-memory (no-op) TLV instead. No flash is touched at any point.
    core1_log("[Core1] step: btstack_memory_init\n");
    btstack_memory_init();
    core1_log("[Core1] step: btstack_run_loop_init\n");
    btstack_run_loop_init(btstack_run_loop_async_context_get_instance(ctx));
    core1_log("[Core1] step: hci_init\n");
    hci_init(hci_transport_cyw43_instance(), NULL);

    // TLV on EEPROM: bonding keys are persisted across resets.
    // RAM mirror was loaded by Core 0 (btstack_tlv_eeprom_init) before launching Core 1.
    core1_log("[Core1] step: tlv_eeprom\n");
    const btstack_tlv_t *tlv = btstack_tlv_eeprom_instance();
    void *tlv_ctx = btstack_tlv_eeprom_context();
    btstack_tlv_set_instance(tlv, tlv_ctx);
    le_device_db_tlv_configure(tlv, tlv_ctx);

    // Configure and register all BLE profiles.
    // Must be called BEFORE hci_power_control.
    core1_log("[Core1] step: ble_manager_init\n");
    ble_manager_init();

    // Power on the HCI — this triggers HCI_STATE_WORKING asynchronously,
    // which causes ble_manager to start advertising + scanning.
    // The CYW43 SPI bus init + firmware download happens inside the HCI open
    // callback (cyw43_bt_init via btstack_hci_transport_cyw43.c). Any failure
    // there is logged by the SDK as "Failed to read test pattern" / "Failed
    // to start CYW43" / "Failed to open cyw43 hci controller". The return
    // value here only reports whether the HCI state machine accepted the
    // request — the actual transport result is async.
    core1_log("[Core1] step: hci_power_control\n");
    int hci_rc = hci_power_control(HCI_POWER_ON);
    core1_log("[Core1] hci_power_control=%d\n", hci_rc);

    // Signal Core 0: BTstack is up and running
    ipc_push_to_core0(IPC_ENCODE(IPC_BLE_READY, 0x00));
    core1_log("[Core1] BTstack running — IPC via btstack timer\n");

    // Start a BTstack repeating timer that polls IPC from Core 0.
    // Running inside the BTstack run loop means:
    //   - No async_context lock needed (already in BTstack context)
    //   - IRQs stay enabled between timer firings → no BLE packet loss
    static btstack_timer_source_t ipc_poll_timer;
    ipc_poll_timer.process = &ipc_poll_timer_handler;
    btstack_run_loop_set_timer(&ipc_poll_timer, IPC_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&ipc_poll_timer);

    // Enter the BTstack run loop. Normally never returns; it exits ONLY when the
    // deep-sleep handler calls btstack_run_loop_trigger_exit() (IPC_BLE_POWER_OFF).
    // Internally calls async_context_wait_for_work_until() which sleeps
    // (via __wfe) until BLE events or timer alarms need processing.
    btstack_run_loop_execute();

    // --- Low-power DEEP sleep: the run loop exited on request. Power the CYW43
    // module down (cyw43_arch_deinit drives WL_REG_ON low → chip off, the bulk of
    // the radio idle current) and halt. There is no in-place resume: Core 0
    // reboots the whole chip on the wake keypress (watchdog), which re-runs this
    // entry from scratch with a clean cyw43/BTstack bring-up.
    core1_log("[Core1] deep sleep — cyw43 power down + halt\n");
    cyw43_arch_deinit();
    ipc_power_off(); // final level survives after this core halts
    for (;;) { __asm volatile("wfi"); }
}
