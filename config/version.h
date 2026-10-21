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
// version.h — SINGLE SOURCE OF TRUTH for the firmware version (semver).
//
// Bump these three numbers by hand before cutting a release. Everything else
// derives from here:
//   - the HID command GET_FW_VERSION (0x03) and GET_KEYBOARD_INFO version bytes;
//   - the boot banner printed by main.c;
//   - the versioned .uf2 filename (CMakeLists.txt parses this file with a regex,
//     producing e.g. CMK_0-1-0.uf2 — see the build section).
//
// The server-side update manifest MUST advertise the same version the firmware
// reports here, so the configurator's semver comparison is meaningful.
//
// CMakeLists.txt parses the three integer #defines below by regex — keep them on
// single lines in the form `#define CMK_FW_VERSION_MAJOR <int>`.
//

#ifndef CMK_VERSION_H
#define CMK_VERSION_H

#define CMK_FW_VERSION_MAJOR  1
#define CMK_FW_VERSION_MINOR  1
#define CMK_FW_VERSION_PATCH  0

// Human-readable string for the boot banner, derived from the integers above so
// there is nothing to keep in sync — bump only the three numbers.
#define CMK_VER_STR_(x)       #x
#define CMK_VER_STR(x)        CMK_VER_STR_(x)
#define CMK_FW_VERSION_STR    CMK_VER_STR(CMK_FW_VERSION_MAJOR) "." \
                              CMK_VER_STR(CMK_FW_VERSION_MINOR) "." \
                              CMK_VER_STR(CMK_FW_VERSION_PATCH)

#endif // CMK_VERSION_H
