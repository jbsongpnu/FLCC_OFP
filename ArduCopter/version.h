#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.0"

// NOTE: 5.1.0 is the OFFICIAL release closing the Copter-4.7.1 -> FLCC migration.
// It carries the V5.0.8 feature set plus the fixes for the issues found while
// migrating it - 13 of the 15 PNU-ISSUEs are resolved; D10 and D11 (KGCS camera
// routing for non-Viewpro mounts) are deliberately deferred past this release and
// are blocked on the Gremsy capability answers.  The 5.0.x series that preceded
// this was carried as DEV builds for the duration of the migration.
// See README "Deferred Issues", PNU-ISSUE Dn.

// NOTE: write every component as plain decimal, with NO leading zero.  A leading zero
// makes a C integer literal octal: 08 and 09 are not valid octal and fail to compile,
// while 00-07 compile but only work by coincidence.  So for any value below 10 write
// 8, not 08.
// The same rule applies to OFP_VER_MAIN/SUB/REV in libraries/GCS_MAVLink/GCS.h, which
// must stay in step with the three defines below (static_assert in Copter.cpp, D4).

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 5,1,0,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
