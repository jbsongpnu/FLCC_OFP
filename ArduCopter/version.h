#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.2"

// NOTE: 5.1.2 is a DEV build on top of the 5.1.0 OFFICIAL release, which closed the
// Copter-4.7.1 -> FLCC migration.  It carries, on top of 5.1.1's D10 Stage 1 work:
//   - the Gremsy VIO protocol analysis, which answers part of the ACTION ON PNU and
//     moves the EO/IR source and IR palette functions out of the FC entirely
//   - the dual-GCS compatibility strategy: a standard GCS (Mission Planner, QGC) keeps
//     full gimbal and camera control, and KGCS-specific convention lives in AP_Q30
//   - the KGCS pitch reversal relocated out of AP_Mount_Backend into AP_Q30, which
//     fixes inverted pitch for Mission Planner and AUTO missions on a Viewpro
// REQUIRES a parameter change on the aircraft: MNT1_PITCH_MIN -45 -> -90 and
// MNT1_PITCH_MAX 90 -> 45.  The firmware cannot do this itself.  See README.
// D10 routing (Stage 2) and D11 remain open.  It becomes OFFICIAL when they land.
// See README "Deferred Issues", PNU-ISSUE Dn.

// NOTE: write every component as plain decimal, with NO leading zero.  A leading zero
// makes a C integer literal octal: 08 and 09 are not valid octal and fail to compile,
// while 00-07 compile but only work by coincidence.  So for any value below 10 write
// 8, not 08.
// The same rule applies to OFP_VER_MAIN/SUB/REV in libraries/GCS_MAVLink/GCS.h, which
// must stay in step with the three defines below (static_assert in Copter.cpp, D4).

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 5,1,2,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 2
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
