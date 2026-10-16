#pragma once

#include <stdint.h>

#include <AP_PNU_CAN/AP_PNU_CAN_config.h>

// PNU : the gimbal is one of the protocols AP_PNU_CAN speaks on the shared bus.  It is
// present when CAN_Dn_PNU_OPTS bit 1 is set; these flags decide whether the code is in
// the build at all.
#ifndef AP_PNUGIMBAL_ENABLED
#define AP_PNUGIMBAL_ENABLED AP_PNU_CAN_ENABLED
#endif

// One flag per motor protocol, so a build that only ever flies one of them does not
// carry the others.
#ifndef AP_PNUGIMBAL_CUBEMARS_ENABLED
#define AP_PNUGIMBAL_CUBEMARS_ENABLED AP_PNUGIMBAL_ENABLED
#endif

#if AP_PNUGIMBAL_CUBEMARS_ENABLED && !AP_PNUGIMBAL_ENABLED
#error "AP_PNUGIMBAL_CUBEMARS_ENABLED requires AP_PNUGIMBAL_ENABLED"
#endif

// The axis identity is deliberately OUTSIDE the enable guard.  AP_PNU_CAN sizes its
// parameter arrays from AP_PNUGIMBAL_NUM_AXES whether or not a gimbal backend is in the
// build - an aircraft with a PMU and a SAR but no gimbal is a real configuration - and
// AGENTS.md requires the base system to compile with optional features disabled.
#define AP_PNUGIMBAL_NUM_AXES 3U

// Axis order is fixed, and it is the order of the three CAN_Dn_PNU_*ID parameters.  It is
// also the homing order settled 2026-10-16 - roll first because the mass offset is on the
// roll axis, yaw last because it is safer to swing the payload with pitch already neutral.
enum AP_PNUGimbal_Axis : uint8_t {
    AP_PNUGIMBAL_AXIS_ROLL  = 0,
    AP_PNUGIMBAL_AXIS_PITCH = 1,
    AP_PNUGIMBAL_AXIS_YAW   = 2,
};
