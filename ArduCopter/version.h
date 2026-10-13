#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.5"

// NOTE: 5.1.5 is a DEV build on top of the 5.1.0 OFFICIAL release, which closed the
// Copter-4.7.1 -> FLCC migration.  It closes D10 Stage 2b by NOT implementing it in the
// flight controller: AP_Q30 now sends an absolute zoom only to a Viewpro, which reads
// ZoomType::PCT as zoom times.  On any other mount it stays silent and says so once,
// because ZOOM_TYPE_RANGE is a percentage of a mode-dependent range (240x combine / 30x
// super-resolution on a Gremsy VIO) selected by an extended parameter the FC cannot read -
// ArduPilot implements no PARAM_EXT_*.  KGCS drives absolute zoom directly with
// MAV_CMD_USER_4 "zoom to level" in times.  A rate-pulse fallback was rejected: it would
// fight the command KGCS is sending.
//
// The FC-side work on D10 is now essentially complete.  What remains is KGCS
// implementation against mappings that are all documented in README and in
// VIO_MAVLink_Messages.md, plus the Stage 3 decision on where TM2 gets its zoom value.
//
// Carried forward from 5.1.1 to 5.1.4: D10 Stage 1 (no fabricated camera values, every
// declined camera command reported), the Gremsy VIO protocol analysis, the dual-GCS
// compatibility strategy, the KGCS pitch reversal relocated into AP_Q30, D10 Stage 2a
// (camera calls routed through AP::camera(), CAM1_TYPE selecting the destination), and
// the Gremsy hardware validation - including a one-shot guard on the "Camera: ..."
// statustext, an upstream defect worth submitting to ArduPilot.
//
// REQUIRES these settings on the aircraft, which the firmware cannot do itself:
//   - MNT1_PITCH_MIN / MNT1_PITCH_MAX in PHYSICAL convention since 5.1.2.  Viewpro
//     -90 / 45; the Gremsy VIO bench-verified at -90 / 15
//   - MNT1_YAW_MIN / MNT1_YAW_MAX set explicitly (-180 / 180) on a Gremsy, to override
//     the bad default the gimbal supplies
//   - CAM1_TYPE: 4 for Viewpro, 6 for Gremsy.  Load-bearing for the KGCS camera path
//     since 5.1.3, where before it only served a standard GCS
//   - a Gremsy VIO's camera component set to 100, not its default 101 - ArduPilot
//     instance 0 searches for 100, and AP_Q30 hardcodes instance 0
// D10 Stage 2b (zoom units) and D11 remain open.  It becomes OFFICIAL when they land.
// See README "Deferred Issues", PNU-ISSUE Dn.

// NOTE: write every component as plain decimal, with NO leading zero.  A leading zero
// makes a C integer literal octal: 08 and 09 are not valid octal and fail to compile,
// while 00-07 compile but only work by coincidence.  So for any value below 10 write
// 8, not 08.
// The same rule applies to OFP_VER_MAIN/SUB/REV in libraries/GCS_MAVLink/GCS.h, which
// must stay in step with the three defines below (static_assert in Copter.cpp, D4).

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 5,1,5,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 5
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
