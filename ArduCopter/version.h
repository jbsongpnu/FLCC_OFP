#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.0.9"

// NOTE: 5.0.8 is carried here as a DEV build for the duration of the
// Copter-4.7.1 -> FLCC step-by-step migration.  The official release after
// this migration completes will be 5.1.0 / FIRMWARE_VERSION_TYPE_OFFICIAL,
// because it also incorporates the fixes for the issues found while
// migrating 5.0.8 (see README "Deferred Issues", PNU-ISSUE Dn).

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 5,0,9,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 0
#define FW_PATCH 9
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
