#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.25"

// RELEASE NOTES LIVE IN CHANGELOG.md, in the repository root, newest first.  They are
// deliberately NOT duplicated here: two copies of the same history drifted once already -
// see GCS-001 in ISSUES.md, where the version this file REPORTED fell seven versions
// behind the string above while nobody noticed.
//
// WHEN BUMPING THE VERSION, change all four together:
//     THISFIRMWARE                    above
//     FIRMWARE_VERSION, FW_PATCH      below
//     OFP_VER_REV                     libraries/GCS_MAVLink/GCS.h
// The static_asserts in Copter.cpp (PNU-ISSUE D4) tie OFP_VER_* to FW_*, but nothing can
// check either against the string, so keeping them in step is a procedural rule.  The
// numeric defines are what MAVLink AUTOPILOT_VERSION and TM13's Version_FLCC_* report to
// the ground station.

// NOTE: write every component as plain decimal, with NO leading zero.  A leading zero
// makes a C integer literal octal: 08 and 09 are not valid octal and fail to compile,
// while 00-07 compile but only work by coincidence.  So for any value below 10 write
// 8, not 08.
// The same rule applies to OFP_VER_MAIN/SUB/REV in libraries/GCS_MAVLink/GCS.h, which
// must stay in step with the three defines below (static_assert in Copter.cpp, D4).

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 5,1,25,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 25
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
