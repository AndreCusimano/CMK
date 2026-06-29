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
// main.c — CMK entry point (Core 0, FreeRTOS)
//
// Core 0: FreeRTOS
//   - connection_manager:  gestione stati BLE upstream/downstream, retry
//   - keyscan:             key matrix scanner + debounce + FN combos (Phase 6)
//   - led_manager:         GP2 power LED + GP3 BLE status LED
//
// Core 1: BTstack + CYW43
//   - advertising, scanning, HID relay, keyboard merge
//

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include "config/cmk_config.h"
#include "config/debug_config.h"
#include "ipc/ipc_core.h"
#include "connection_manager.h"
#include "keyscan.h"
#include "action.h"
#include "board_side.h"
#include "keymap.h"
#include "dynamic_keymap.h"
#include "led_manager.h"
#include "bonding_store.h"
#include "eeconfig.h"
#include "bootmagic.h"
#include "power_manager.h"
#include "inter_half_sync.h"
#include "i2c0_bus.h"
#include "tlv/btstack_tlv_eeprom.h"

#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif

extern void core1_entry(void);

// ---------------------------------------------------------------------------
// Core 1 stack — explicit allocation so we control the size.
// Default PICO_CORE1_STACK_SIZE is only 0x800 (2 KB), far too small for
// BTstack + HID device + HID client + SM pairing + descriptor parser.
// 16 KB: BTstack run loop + CYW43 SPI IRQ handler share this stack.
// Peak occurs when a CYW43 IRQ fires during a nested BTstack API call
// (e.g. gap_advertisements_set_params → hci_send_cmd + IRQ on top).
// The RP2350 has 520 KB SRAM total, so 16 KB is cheap insurance.
// ---------------------------------------------------------------------------
uint32_t core1_stack[4096] __attribute__((aligned(8)));  // 4096 words = 16 KB
const size_t core1_stack_words = 4096;

// ---------------------------------------------------------------------------
// FreeRTOS stack overflow hook — called when configCHECK_FOR_STACK_OVERFLOW >= 1
// The task name and handle are available; printf may not be safe here so use
// a raw UART write to ensure the message appears before the system halts.
// ---------------------------------------------------------------------------

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    // Direct printf: UART TX is interrupt-driven but the TX FIFO drains on its own.
    // We freeze here intentionally so the message stays on screen.
    printf("\n*** STACK OVERFLOW in task \"%s\" — halted ***\n", pcTaskName);
    while (1) { __asm volatile("nop"); }
}

// ---------------------------------------------------------------------------
// FreeRTOS idle hook (configUSE_IDLE_HOOK) — baseline low-power lever.
// When Core 0 has no runnable task it sleeps in WFI until the next interrupt
// (SysTick at 1 kHz, or any peripheral/GPIO IRQ) instead of spinning in the
// idle loop. Zero impact on peripherals or tick timing — the core simply stops
// burning power between ticks. (Core 1 already idles in __wfe inside the BTstack
// run loop.) Deeper sleep tiers are layered on top by the low_power module.
// ---------------------------------------------------------------------------

void vApplicationIdleHook(void) {
    __asm volatile("wfi");
}

// ---------------------------------------------------------------------------
// Build tag — bump this on every change that needs re-flashing. Printed at boot
// so you can confirm from the UART log exactly which firmware is running.
// ---------------------------------------------------------------------------
#define CMK_BUILD_TAG "b004-usb-clean"

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(void) {
    stdio_init_all();

    // Detect which half this is (GP41/ADC1, one-shot) and point the runtime
    // keymap/LED selectors at it. MUST run before the banner (prints the side),
    // before action_init() (rgb_matrix_init reads g_led_config) and before
    // Core 1 launch (BLE name derives from the side).
    board_side_init();
    keymap_init(board_side_get());

    printf("\n=== CMK Firmware ===\n");
    printf("[VER] v%s (%s)  side=%s sense=%umV  built " __DATE__ " " __TIME__ "\n",
           CMK_FW_VERSION_STR, CMK_BUILD_TAG, board_side_short(), board_side_sense_mv());
    printf("Matrix: %dx%d  |  Debounce: %d ms\n", MATRIX_ROWS, MATRIX_COLS, DEBOUNCE);
    printf("Layers=%d  Tapping=%dms\n\n", NUM_LAYERS, TAPPING_TERM);

    // --- Status LEDs scritti da Core 1 (only if the board wires discrete LEDs) ---
#ifdef PIN_LED_CONN
    gpio_init(PIN_LED_CONN);
    gpio_set_dir(PIN_LED_CONN, GPIO_OUT);
    gpio_put(PIN_LED_CONN, 0);
#endif
#ifdef PIN_LED_RELAY
    gpio_init(PIN_LED_RELAY);
    gpio_set_dir(PIN_LED_RELAY, GPIO_OUT);
    gpio_put(PIN_LED_RELAY, 0);
#endif

    DBG_BOOT("[BOOT] led_manager_init...\n");
    led_manager_init();

    DBG_BOOT("[BOOT] keyscan_init...\n");
    keyscan_init();

    DBG_BOOT("[BOOT] i2c0_bus_init (must run before any I2C consumer)...\n");
    i2c0_bus_init();

    DBG_BOOT("[BOOT] bonding_store_init (EEPROM probe on I2C0)...\n");
    bonding_store_init();

    DBG_BOOT("[BOOT] eeconfig_init...\n");
    eeconfig_init();

#ifdef DYNAMIC_KEYMAP_ENABLE
    // Load the editable keymap from EEPROM (or seed from the side's compiled
    // default) and re-point `keymaps` at the RAM cache. Needs EEPROM up
    // (bonding_store_init ran) and the detected side (board_side_init ran).
    DBG_BOOT("[BOOT] dynamic_keymap_init...\n");
    dynamic_keymap_init(board_side_get());
#endif

    // action_init() depends on eeconfig (rgb_matrix_init / backlight_init
    // read defaults), so it must run AFTER eeconfig_init.
    DBG_BOOT("[BOOT] action_init (depends on eeconfig)...\n");
    action_init();

    DBG_BOOT("[BOOT] bootmagic_execute...\n");
    bootmagic_execute();

    DBG_BOOT("[BOOT] btstack_tlv_eeprom_init...\n");
    btstack_tlv_eeprom_init();

    DBG_BOOT("[BOOT] ipc_init...\n");
    ipc_init();

    DBG_BOOT("[BOOT] connection_manager_init...\n");
    connection_manager_init();

    DBG_BOOT("[BOOT] power_manager_init...\n");
    power_manager_init();

    // --- Paint Core 1 stack canary before launch ---
    // Bottom 16 words = canary; core1_entry.c checks periodically.
    for (int i = 0; i < 16; i++) core1_stack[i] = 0xDEADBEEFu;

    // --- Avvio Core 1 (BTstack + CYW43) — stack esplicito da 16 KB ---
    multicore_launch_core1_with_stack(core1_entry,
                                      core1_stack, sizeof(core1_stack));

    // --- FreeRTOS tasks ---
    connection_manager_create_task();
    keyscan_create_task();
    led_manager_create_task();
    power_manager_create_task();
    inter_half_sync_create_task();   // R6: link I2C1 fuori dal keyscan tick
#ifdef USB_ENABLE
    usb_hid_task_create();
#endif

    vTaskStartScheduler();

    while (1) {}
    return 0;
}
