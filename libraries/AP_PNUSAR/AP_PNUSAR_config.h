#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_CANManager/AP_CANManager_config.h>

// PNU : AP_PNUSAR is a CAN driver for the SAR radar payload.  It speaks a
// private CAN protocol, NOT DroneCAN, in the same shape as AP_PMUCAN.  Enabled only where
// there is a CAN interface to speak it on.
#ifndef AP_PNUSAR_ENABLED
#define AP_PNUSAR_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 0)
#endif
