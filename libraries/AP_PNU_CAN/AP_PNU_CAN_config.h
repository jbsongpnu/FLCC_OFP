#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_CANManager/AP_CANManager_config.h>

// PNU : AP_PNU_CAN is the CAN driver for PNU's own devices - the PMU, the gimbal and the
// SAR payload.  They share one bus, so they share one driver: AP_CANManager binds each
// interface to exactly one driver, and a driver with no interface is never allocated at
// all.  Each protocol lives in an AP_PNU_CAN_Backend.
//
// None of it is DroneCAN.  CAN_Dn_PNU_OPTS selects which devices are present.
#ifndef AP_PNU_CAN_ENABLED
#define AP_PNU_CAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 0)
#endif
