#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.4"

// NOTE: 5.1.4 is a DEV build on top of the 5.1.0 OFFICIAL release, which closed the
// Copter-4.7.1 -> FLCC migration.  It is the first version validated against real Gremsy
// VIO hardware, and carries one fix found there: AP_Camera_MAVLinkCamV2 printed the
// "Camera: ..." statustext on every CAMERA_INFORMATION received, and the VIO broadcasts
// that message unsolicited - so it spammed one statustext per broadcast to every link.
// Now printed once.  This is an upstream defect and should be submitted to ArduPilot
// rather than carried as another permanent divergence.
//
// Bench results recorded in README: Stage 2a validated with no code change, both
// ACTION ON PNU identity questions answered, HAS_BASIC_ZOOM proven, and a Gremsy firmware
// defect found - the yaw fields of GIMBAL_DEVICE_INFORMATION are sent in degrees where
// msg 283 specifies radians, which gives MNT1_YAW_MAX a default of 18334.
//
// Carried forward from 5.1.1 to 5.1.3: D10 Stage 1 (no fabricated camera values, every
// declined camera command reported), the Gremsy VIO protocol analysis, the dual-GCS
// compatibility strategy, the KGCS pitch reversal relocated into AP_Q30, and D10 Stage 2a
// (camera calls routed through AP::camera(), CAM1_TYPE selecting the destination).
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
#define FIRMWARE_VERSION 5,1,4,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 4
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
