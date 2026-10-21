# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(DEFINED ENV{VSCODE_CWD})
    message(STATUS "Rilevato VS Code, includo pico-vscode.cmake")
    if(WIN32)
        set(USERHOME $ENV{USERPROFILE})
    else()
        set(USERHOME $ENV{HOME})
    endif()
    set(sdkVersion 2.2.0)
    set(toolchainVersion 14_2_Rel1)
    set(picotoolVersion 2.2.0-a4)
    set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
    if (EXISTS ${picoVscode})
        include(${picoVscode})
    endif()
else()
    message(STATUS "Non sono in VS Code, salto pico-vscode.cmake")
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(PROJECT_NAME "CMK")

# Custom board header — pinout single source of truth consumed by pico-sdk.
# See boards/cmk_keeeyboard-split.h for UART/I2C/CYW43 defaults. CMK firmware-level
# definitions (matrix, I2C addresses, RGB count, etc.) live in config/board.h.
set(PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}/boards")
set(PICO_BOARD cmk_keeeyboard-split)
set(PICO_PLATFORM rp2350-arm-s)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# FreeRTOS — FreeRTOSConfig.h lives in config/
set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel")
set(FREERTOS_CMAKE_PATH  "${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ")
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/config" CACHE PATH "FreeRTOS config directory")
include(${FREERTOS_CMAKE_PATH}/FreeRTOS_Kernel_import.cmake)

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ===========================================================================
# CMK — Master firmware (Pico 2W — RP2350)
#
# Core 0: FreeRTOS — organised QMK-style:
#   src/core0/cmk/             — CMK-specific (no QMK equivalent)
#   src/core0/core/            — quantum/ core dispatcher (action, matrix, etc.)
#   src/core0/process_keycode/ — quantum/process_keycode/ (feature modules)
#   src/core0/{debounce,bootmagic,send_string,unicode,
#              rgblight,rgb_matrix,led_matrix,backlight,led}/
#                              — single-module subsystems mirroring QMK
#   src/core0/drivers/         — hardware drivers (EEPROM M24M01, WS2812 PIO)
#   src/ipc/                   — IPC ring buffer + multicore FIFO helpers
#
# Core 1: BTstack BLE (dual-role: Peripheral + Central)
#   src/ble/    — BLE manager (ADV, SCAN, HID relay, keyboard merge)
#   src/hid/    — HID descriptor parser (upstream format normalisation)
# ===========================================================================

# ===========================================================================
# Shared sources, include dirs and libraries (used by both CMK-l and CMK-r)
# ===========================================================================

set(CMK_SOURCES
        main.c
        src/ipc/ipc_core.c
        src/tlv/btstack_tlv_eeprom.c

        # --- src/core0/cmk/ — CMK-specific (no QMK equivalent) ---
        src/core0/cmk/connection_manager.c
        src/core0/cmk/bonding_store.c
        src/core0/cmk/led_manager.c
        src/core0/cmk/keyscan.c
        src/core0/cmk/cmk_keycodes.c
        src/core0/cmk/output_select.c
        src/core0/cmk/power_manager.c
        src/core0/cmk/board_side.c
        src/core0/cmk/fn_overlay.c
        src/core0/cmk/low_power.c
        src/core0/cmk/inter_half_sync.c

        # --- src/core0/core/ — quantum/ core dispatcher ---
        src/core0/core/matrix.c
        src/core0/core/action.c
        src/core0/core/action_layer.c
        src/core0/core/keymap.c
        src/core0/core/dynamic_keymap.c
        src/core0/core/macro_storage.c
        src/core0/core/action_util.c
        src/core0/core/action_tapping.c
        src/core0/core/process_record.c
        src/core0/core/eeconfig.c
        src/core0/core/swap_hands.c
        src/core0/core/wpm.c
        src/core0/core/velocikey.c

        # --- src/core0/process_keycode/ — quantum/process_keycode/ ---
        src/core0/process_keycode/process_combo.c
        src/core0/process_keycode/process_tap_dance.c
        src/core0/process_keycode/process_caps_word.c
        src/core0/process_keycode/process_key_override.c
        src/core0/process_keycode/process_mouse_keys.c
        src/core0/process_keycode/process_repeat_key.c
        src/core0/process_keycode/process_auto_shift.c
        src/core0/process_keycode/process_leader.c
        src/core0/process_keycode/process_dynamic_macro.c
        src/core0/process_keycode/process_macro.c
        src/core0/process_keycode/process_space_cadet.c
        src/core0/process_keycode/process_grave_esc.c
        src/core0/process_keycode/process_magic.c
        src/core0/process_keycode/process_tri_layer.c
        src/core0/process_keycode/process_layer_lock.c
        src/core0/process_keycode/process_key_lock.c
        src/core0/process_keycode/process_secure.c
        src/core0/process_keycode/process_unicode.c
        src/core0/process_keycode/process_unicodemap.c
        src/core0/process_keycode/process_ucis.c
        src/core0/process_keycode/process_programmable_button.c
        src/core0/process_keycode/process_consumer.c
        src/core0/process_keycode/process_rgb.c
        src/core0/process_keycode/process_backlight.c

        # --- single-module subsystems (mirror QMK quantum/ subdirs) ---
        src/core0/debounce/debounce.c
        src/core0/send_string/send_string.c
        src/core0/bootmagic/bootmagic.c
        src/core0/unicode/unicode.c
        src/core0/rgblight/rgblight.c
        src/core0/rgb_matrix/rgb_matrix.c
        src/core0/led_matrix/led_matrix.c
        src/core0/backlight/backlight.c
        src/core0/led/led_indicators.c

        # --- src/core0/drivers/ — hardware drivers (QMK drivers/) ---
        src/core0/drivers/i2c0_bus.c
        src/core0/drivers/i2c1_link.c
        src/core0/drivers/eeprom/eeprom.c
        src/core0/drivers/led/rgb_driver.c

        # --- src/core0/util/ — small leaf helpers ---
        src/core0/util/crc16.c
        src/core0/util/diag.c

        src/usb/usb_descriptors.c
        src/usb/usb_hid.c
        src/usb/usb_debug.c
        src/power/power_policy.c
        src/ble/ble_power.c
        src/usb/os_detection.c
        src/usb/raw_hid.c
        src/usb/cmk_config_protocol.c
        src/hid/hid_parser.c
        src/core1_entry.c
        src/ble/ble_manager.c
        src/ble/ble_periph.c
        src/ble/ble_central.c
        src/ble/ble_split.c
        src/ble/ble_batt.c
        src/ble/ble_relay.c
        src/ble/ble_link_policy.c
)

set(CMK_INCLUDE_DIRS
        ${CMAKE_CURRENT_LIST_DIR}           # root — "config/cmk_config.h", "ipc/ipc_core.h", etc.
        ${CMAKE_CURRENT_LIST_DIR}/config    # btstack_config.h, cmk_config.h, debug_config.h
        ${CMAKE_CURRENT_LIST_DIR}/pio       # ws2812.pio.h (generated)
        ${CMAKE_CURRENT_LIST_DIR}/src       # core1_entry.c + ipc/, ble/, hid/ subdirs
        ${CMAKE_CURRENT_LIST_DIR}/src/usb   # usb_hid.h, usb_descriptors.h, os_detection.h, raw_hid.h

        # core0 — QMK-style flat include paths (bare #include "name.h")
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/cmk
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/core
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/process_keycode
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/debounce
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/send_string
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/bootmagic
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/unicode
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/rgblight
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/rgb_matrix
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/led_matrix
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/backlight
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/led
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/drivers
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/drivers/eeprom
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/drivers/led
        ${CMAKE_CURRENT_LIST_DIR}/src/core0/util
)

set(CMK_LIBS
        pico_stdlib
        pico_unique_id
        pico_multicore                      # multicore_launch_core1()
        pico_cyw43_arch_none                # CYW43: none-variant — Core 1 manages CYW43 directly
        pico_btstack_ble                    # BLE + GATT services (hids_device, hids_host)
        pico_btstack_cyw43                  # CYW43 HCI transport + async_context integration
        FreeRTOS-Kernel-Heap4               # FreeRTOS (Core 0 only)
        hardware_gpio
        hardware_pio                        # ws2812 PIO (Fase 9 RGB)
        hardware_dma                        # DMA-fed WS2812 flush (preemption-immune)
        hardware_sync                       # spin_lock_* for IPC ring buffer
        hardware_i2c                        # M24M01-R EEPROM via I2C0
        pico_i2c_slave                      # I2C1 inter-half link slave role (Phase 20)
        hardware_adc                        # USB-C CC1/CC2/VBUS sense (Phase 17)
        hardware_pwm                        # PWM backlight (Phase 13d)
        hardware_watchdog                   # watchdog_reboot() for CMK_RESET
        tinyusb_device                      # TinyUSB device stack (Phase 14)
)

# ===========================================================================
# Helper function — creates one CMK build target
#
# TARGET_NAME  : CMK / CMK-l / CMK-r
# FORCE_DEFINE : ""                     → auto-detect the side at boot via ADC
#                                          (GP41/ADC1, see board_side.c) — DEFAULT
#                CMK_FORCE_SIDE_LEFT  → force LEFT, skip the ADC (fallback build)
#                CMK_FORCE_SIDE_RIGHT → force RIGHT, skip the ADC (fallback build)
# ===========================================================================

# ---------------------------------------------------------------------------
# Firmware version — parsed from config/version.h (the single source of truth)
# so the release .uf2 is named CMK_<maj>-<min>-<patch>.uf2. Keep version.h as
# the only place the version is edited; this regex just reads it.
# ---------------------------------------------------------------------------
# Re-run CMake configure whenever version.h changes, so the parsed version (and
# the .uf2 filename) stay in sync — a plain `cmake --build` would otherwise reuse
# the cached value since headers aren't configure dependencies by default.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/config/version.h")
file(READ "${CMAKE_CURRENT_LIST_DIR}/config/version.h" _cmk_version_h)
string(REGEX MATCH "CMK_FW_VERSION_MAJOR[ \t]+([0-9]+)" _ "${_cmk_version_h}")
set(_CMK_VER_MA "${CMAKE_MATCH_1}")
string(REGEX MATCH "CMK_FW_VERSION_MINOR[ \t]+([0-9]+)" _ "${_cmk_version_h}")
set(_CMK_VER_MI "${CMAKE_MATCH_1}")
string(REGEX MATCH "CMK_FW_VERSION_PATCH[ \t]+([0-9]+)" _ "${_cmk_version_h}")
set(_CMK_VER_PA "${CMAKE_MATCH_1}")
set(CMK_FW_VER "${_CMK_VER_MA}-${_CMK_VER_MI}-${_CMK_VER_PA}")
message(STATUS "CMK firmware version: ${_CMK_VER_MA}.${_CMK_VER_MI}.${_CMK_VER_PA} (uf2 suffix _${CMK_FW_VER})")

function(cmk_add_target TARGET_NAME FORCE_DEFINE)
    add_executable(${TARGET_NAME} ${CMK_SOURCES})

    # GATT profile header — pico_btstack_make_gatt_header adds build dir to includes
    pico_btstack_make_gatt_header(${TARGET_NAME} PRIVATE
            "${CMAKE_CURRENT_LIST_DIR}/src/ble/hid_keyboard.gatt")

    # PIO header for WS2812 (Fase 9 RGB).
    # Note: both targets invoke pico_generate_pio_header for the same output file.
    # CMake/Ninja deduplicates the rule so pioasm runs only once.
    pico_generate_pio_header(${TARGET_NAME}
            ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio
            OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/pio)

    target_include_directories(${TARGET_NAME} PRIVATE ${CMK_INCLUDE_DIRS})

    target_compile_definitions(${TARGET_NAME} PRIVATE
            # CYW43 / BTstack still calls flash_safe_execute() internally
            # (country code lookup etc.) — with FreeRTOS on Core 0 the
            # multicore-lockout IRQ path isn't usable. We've already routed
            # all our own persistence to the external I2C EEPROM, so the
            # internal flash ops can fail gracefully (return code is
            # checked by pico-sdk). Silence the debug assert.
            PICO_FLASH_ASSERT_ON_UNSAFE=0
    )

    # Optional compile-time side override. Default (empty) = auto-detect via ADC.
    if(FORCE_DEFINE)
        target_compile_definitions(${TARGET_NAME} PRIVATE ${FORCE_DEFINE}=1)
    endif()

    target_link_libraries(${TARGET_NAME} ${CMK_LIBS})

    # Debug output via UART (not USB)
    pico_enable_stdio_usb(${TARGET_NAME} 0)
    pico_enable_stdio_uart(${TARGET_NAME} 1)

    pico_add_extra_outputs(${TARGET_NAME})

    # Versioned copy of the .uf2 (e.g. CMK_0-1-0.uf2) alongside the plain one.
    # This is the artifact published for the "Update FW" flow; it matches the
    # version reported by GET_FW_VERSION. The plain CMK.uf2 stays for local flash.
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                "$<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.uf2"
                "$<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}_${CMK_FW_VER}.uf2"
            VERBATIM)
endfunction()

# ===========================================================================
# Build targets — ONE firmware for both halves; the side is auto-detected at
# boot (GP41/ADC1). CMK-l / CMK-r are fallback builds that force the side at
# compile time (useful when the side-sense pin is not wired on first-batch HW).
#   cmake --build cmake-build-debug --target CMK    →  CMK.uf2    (auto-detect)
#   cmake --build cmake-build-debug --target CMK-l  →  CMK-l.uf2  (forced LEFT)
#   cmake --build cmake-build-debug --target CMK-r  →  CMK-r.uf2  (forced RIGHT)
# ===========================================================================

cmk_add_target(CMK   "")
cmk_add_target(CMK-l CMK_FORCE_SIDE_LEFT)
cmk_add_target(CMK-r CMK_FORCE_SIDE_RIGHT)
