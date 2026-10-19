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

// The driver thread rate, and the ONE clock every backend derives its sub-rates from -
// update() is handed the tick count rather than each backend inventing a timer.  It lives
// here rather than in AP_PNU_CAN.h so that a backend can reach it without pulling in the
// whole driver: AP_PMUCAN derives its ICD-specified 10 Hz per message from it.
#define AP_PNU_CAN_LOOP_HZ            (100U)    // thread rate, 10 ms
