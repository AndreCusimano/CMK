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
// crc16.h — CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection).
//
// Used for integrity checking on links that may glitch — currently the I2C1
// inter-half config transfer (Phase 20). Bit-by-bit, no lookup table (the
// payloads are small and transfers rare, so RAM/flash beats throughput here).
//
// Self-check: crc16_ccitt("123456789", 9) == 0x29B1.
//

#ifndef CMK_CRC16_H
#define CMK_CRC16_H

#include <stdint.h>
#include <stddef.h>

uint16_t crc16_ccitt(const uint8_t *buf, size_t len);

#endif // CMK_CRC16_H
