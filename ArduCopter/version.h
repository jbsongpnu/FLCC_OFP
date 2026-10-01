#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.1"

// NOTE: 5.1.1 is a DEV build on top of the 5.1.0 OFFICIAL release, which closed the
// Copter-4.7.1 -> FLCC migration.  It carries the D10 Stage 1 work: every fabricated
// camera value is removed and every declined camera command is now reported, so the
// D10 symptom is visible on a non-Viewpro mount instead of silent.  Routing itself is
// unchanged - the D10 decision and all of D11 remain blocked on the Gremsy capability
// answers (README, "ACTION ON PNU").  It becomes OFFICIAL when that work lands.
// See README "Deferred Issues", PNU-ISSUE Dn.

// NOTE: write every component as plain decimal, with NO leading zero.  A leading zero
// makes a C integer literal octal: 08 and 09 are not valid octal and fail to compile,
// while 00-07 compile but only work by coincidence.  So for any value below 10 write
// 8, not 08.
// The same rule applies to OFP_VER_MAIN/SUB/REV in libraries/GCS_MAVLink/GCS.h, which
// must stay in step with the three defines below (static_assert in Copter.cpp, D4).

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 5,1,1,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 1
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
