# CMK — Custom Mechanical Keyboard Firmware

Open-source firmware for a **split mechanical keyboard** based on **RP2350B + Raspberry Pi RM2 radio
module** (Pico 2 W class, CYW43439). Selectable **BLE and USB** output; over BLE it supports multi-hop
relay (an upstream mouse/keyboard is relayed transparently all the way to the PC). 4×6 matrix per half,
L/R side auto-detected at boot, battery power with charge management, RGB matrix, and runtime
configuration via WebHID. Keyboard logic derived from the QMK C core.

---

## AI Disclaimer

> This project was developed with the assistance of **Claude** (Anthropic) as a programming aid.
> Every contribution was then **reviewed, tested and validated manually**.
>
> While acknowledging that Claude was a very useful tool, the choice of architecture, the integration
> and the responsibility for the final result remain mine.
>
> In any case, anyone not satisfied with the result is free to modify the code as they please (within
> the terms of the GPLv2 license it is subject to) or not to use it at all.

---

## Features

- **Composite HID** — Keyboard (Report ID 1) + Mouse (Report ID 2) + Programmable Button (Report ID 3), over **BLE and USB**
- **Selectable USB/BLE output** — TinyUSB HID over cable (default USB) or BLE, switchable from the keyboard; persisted in eeconfig
- **Multi-hop BLE relay** — chain of CMK devices in series (Mouse -> CMK1 -> CMK2 -> PC), merging local keys into the in-transit stream
- **Downstream multi-host** — 3 independent BLE slots (unique Random Static Address), switch with a single key (Td0/Td1/Td2)
- **Persistent bonding** — BLE addresses + SM keys on external EEPROM (M24M01-R I2C); re-encryption after reboot
- **Split** — single firmware for both halves, side (L/R) auto-detected at boot via ADC; dynamic master/slave over BLE + I2C1 config link between the halves
- **Layer system** — 6 layers, 16-bit keycodes, Mod-Tap, Layer-Tap, One-Shot Layer, Layer Lock, Tri-Layer
- **Key Features** — combo, tap dance, caps word, key override, mouse keys, repeat key, auto shift, leader key, dynamic macro, space cadet, grave escape, magic, unicode, UCIS (inspired by QMK)
- **RGB matrix** — per-key effects (WS2812) + **status dashboard (FN overlay)**: connection, DS slot, battery while FN is held
- **Power management** — battery with charger (BQ24296) and fuel gauge (BQ27441) on I2C0; battery level via BLE Battery Service, RGB blanked below critical threshold
- **WebHID configurator** — runtime keymap on EEPROM (dynamic keymap) + RGB/settings via CMK Config Protocol over Raw HID USB; firmware update via WebHID/PICOBOOT
- **Persistent settings** — eeconfig on EEPROM, bootmagic factory reset, persistent magic swaps
- **Dual-core** — FreeRTOS on Core 0, BTstack BLE (CYW43439/RM2) on Core 1. Key logic derived from the QMK C core (OS-agnostic)

---

## Required hardware

Reference hardware: custom **CMK Proto v1** board.

| Component | Model | Notes |
|---|---|---|
| Microcontroller | **RP2350B** (QFN-80, 48 GPIO) | "CMK Proto v1" board |
| BLE/Wi-Fi radio | **Raspberry Pi RM2** | CYW43439 (Pico 2 W class); gSPI on GP31-34 |
| Matrix | 4×6 per half (COL2ROW) | split L/R, side auto-detected via ADC (GP41) |
| EEPROM | ST M24M01-R (1 Mbit) | I2C0 `0x50`; without it -> RAM-only |
| Charger | TI BQ24296 | I2C0 `0x6B` (battery charge, PSEL/IINLIM) |
| Fuel gauge | TI BQ27441-G1A | I2C0 `0x55` (SoC/voltage/current) |
| RGB | WS2812B on PIO | DIN GP35, 5V rail enabled by GP36 (VLED_EN) |
| Inter-half link | I2C1 | GP42/GP43 (config transfer between halves) |
| USB | USB-C | HID + Raw HID (WebHID); CC/VBUS sense on ADC |

Full GPIO pinout (matrix, I2C, RGB, side sense, USB-C) is defined in `config/board.h`.

---

## Connection topology

```
Mouse/Device --BLE--> CMK Device 1 --BLE--> CMK Device 2 --BLE--> PC
 (upstream)            relay + keys         relay + keys         reads everything
```

Each CMK device is both **Central** (upstream, toward the previous device) and **Peripheral**
(downstream, toward the next one), adds its own keys to the HID stream and manages 3 independent DS
hosts.

---

## Build and flash

### Prerequisites

```bash
# macOS
brew install cmake gcc-arm-embedded

# Linux
sudo apt install cmake gcc-arm-none-eabi

# Pico SDK
export PICO_SDK_PATH=/path/to/pico-sdk
```

### Compilation

```bash
git clone <repo>
cd CMK
cmake -B cmake-build-debug

# Single firmware — the side (L/R) is auto-detected at boot (GP41/ADC1):
cmake --build cmake-build-debug --target CMK      # -> CMK.uf2    (both halves)

# Fallback: force the side at compile-time (HW with the sense pin not wired):
cmake --build cmake-build-debug --target CMK-l    # -> CMK-l.uf2  (forced LEFT)
cmake --build cmake-build-debug --target CMK-r    # -> CMK-r.uf2  (forced RIGHT)
```

The compiled `.uf2` files are located in `cmake-build-debug/`.

### Flash

1. Hold down the **BOOTSEL** button on the Pico 2W
2. Connect via USB — it appears as an `RPI-RP2` drive
3. Copy the `.uf2` file to the drive
4. The Pico reboots automatically with the new firmware

Use `CMK.uf2` by default (side auto-detect); `CMK-l.uf2` / `CMK-r.uf2` only if the side sense pin is
not wired.

---

## First boot and pairing

The **FN** key (GP14) momentarily activates the Settings layer. Main combinations:

| Key | Press | Action |
|---|---|---|
| FN + Td0/Td1/Td2 | Short | Reconnect / switch to DS host slot 0/1/2 (directed ADV) |
| FN + Td0/Td1/Td2 | Long 3s | Forget slot + general ADV (first setup) |
| FN + Tu | Long 3s | Forget upstream + general scan (look for the previous device) |
| FN + EE_CLR | Long 3s | Factory reset — clear all bonding |
| FN + RESET | Short | Reboot (watchdog) |

### Chain setup (e.g. Mouse -> CMK1 -> CMK2 -> PC)

> Output: by default HID reports go over **USB** (wired). To use the BLE chain set the output to BLE
> (`FN + Td{x}`, which selects the BLE slot and BLE output). The current state is visible in the RGB
> overlay under FN.

1. **CMK1 upstream**: `FN + Tu` (3s) -> scan -> find mouse -> auto-connect
2. **CMK1 downstream**: `FN + Td0` (3s) -> general ADV -> CMK2 connects
3. **CMK2 upstream**: `FN + Tu` (3s) -> scan -> find CMK1 -> auto-connect
4. **CMK2 downstream**: `FN + Td0` (3s) -> general ADV -> PC connects
5. **PC**: Bluetooth settings -> look for **`CMK-Keeeyboard-Split-L`** / **`-R`** -> pair (Just Works, no PIN)

After the first setup the chain reconnects automatically at boot (US `gap_connect`, DS directed ADV);
the SM keys persist on EEPROM -> re-encryption without fresh pairing. DS has priority over US (CYW43
does not support ADV + Initiating simultaneously).

### Status (RGB dashboard on FN)

There are no discrete status LEDs: the state is shown on the **RGB matrix as an overlay activated
while FN is held** (`fn_overlay` + `led_indicators` modules). Holding FN you can read: upstream/downstream
connection state, active DS slot (0/1/2), USB vs BLE output, and battery level (with charge/discharge
indication). Below the critical threshold the RGB matrix goes black to save power, but the battery
overlay stays readable under FN.

---

## Debug

Diagnostic output via UART (Core 1 never touches UART — logs go through an IPC ring buffer drained by
Core 0).

---

## Compatibility

Tested on **Windows 10/11**, **macOS 12+** and **Linux (BlueZ)**. iOS / Android not tested.

---

## License

CMK is distributed under the **GNU General Public License, version 2** (GPLv2).
See the [`LICENSE`](LICENSE) file for the full license text and the [`NOTICE`](NOTICE) file for the
copyright attributions and the list of third-party dependencies.

[![License: GPL v2](https://img.shields.io/badge/License-GPLv2-blue.svg)](LICENSE)

CMK inherits the GPLv2 because its keyboard logic is **derived from the QMK C core**.
Everything else — QMK/TMK derivation, third-party components and licenses (Pico SDK, cyw43-driver,
BTstack with the Raspberry Pi/RM2 exception, FreeRTOS, TinyUSB, ws2812.pio), written offer for the
source (GPL §3), RM2 radio certification (FCC `2ABCB-RMC2GW4B52`, CE/RED, IC) and trademarks — is
documented in [`NOTICE`](NOTICE).
