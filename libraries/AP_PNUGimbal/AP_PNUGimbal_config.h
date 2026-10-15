#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_CANManager/AP_CANManager_config.h>

// PNU : AP_PNUGimbal is a CAN driver for the PNU-developed gimbal mount.  It speaks a
// private CAN protocol, NOT DroneCAN, in the same shape as AP_PMUCAN.  Enabled only where
// there is a CAN interface to speak it on.
#ifndef AP_PNUGIMBAL_ENABLED
#define AP_PNUGIMBAL_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 0)
#endif

// PNU : the protocol spoken on that interface lives in a backend, because the mount is
// expected to accommodate more than one motor type.  Each backend has its own flag so a
// build that only ever flies one of them does not carry the others.
#ifndef AP_PNUGIMBAL_CUBEMARS_ENABLED
#define AP_PNUGIMBAL_CUBEMARS_ENABLED AP_PNUGIMBAL_ENABLED
#endif

#if AP_PNUGIMBAL_CUBEMARS_ENABLED && !AP_PNUGIMBAL_ENABLED
#error "AP_PNUGIMBAL_CUBEMARS_ENABLED requires AP_PNUGIMBAL_ENABLED"
#endif
