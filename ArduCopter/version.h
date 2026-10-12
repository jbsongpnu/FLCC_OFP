#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.3"

// NOTE: 5.1.3 is a DEV build on top of the 5.1.0 OFFICIAL release, which closed the
// Copter-4.7.1 -> FLCC migration.  It carries D10 Stage 2a: the six camera functions
// now route through AP::camera() instead of AP::mount(), so CAM1_TYPE selects the
// destination - 4 (Mount) reaches a Viewpro exactly as before, 6 (MAVLinkCamV2) reaches
// a Gremsy camera component the mount cannot address at all.  get_zoom_times() and
// IR_Color_Change() stay on AP::mount(); they have no AP_Camera equivalent.
// Zoom units are deliberately NOT converted - that only matters at CAM1_TYPE=6 and
// needs the VIO's maximum zoom, which MAVLink cannot report.
//
// Carried forward from 5.1.1 and 5.1.2: D10 Stage 1 (no fabricated camera values, every
// declined camera command reported), the Gremsy VIO protocol analysis, the dual-GCS
// compatibility strategy, and the KGCS pitch reversal relocated into AP_Q30.
//
// REQUIRES two parameter settings on the aircraft, which the firmware cannot do itself:
//   - MNT1_PITCH_MIN -45 -> -90 and MNT1_PITCH_MAX 90 -> 45 (pitch relocation, 5.1.2)
//   - CAM1_TYPE correct - 4 for Viewpro.  It is now load-bearing for the KGCS camera
//     path, where before it only served a standard GCS (Stage 2a, 5.1.3)
// D10 Stage 2b (zoom units) and D11 remain open.  It becomes OFFICIAL when they land.
// See README "Deferred Issues", PNU-ISSUE Dn.

// NOTE: write every component as plain decimal, with NO leading zero.  A leading zero
// makes a C integer literal octal: 08 and 09 are not valid octal and fail to compile,
// while 00-07 compile but only work by coincidence.  So for any value below 10 write
// 8, not 08.
// The same rule applies to OFP_VER_MAIN/SUB/REV in libraries/GCS_MAVLink/GCS.h, which
// must stay in step with the three defines below (static_assert in Copter.cpp, D4).

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 5,1,3,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 3
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
