#pragma once

#include <AP_PNU_CAN/AP_PNU_CAN_config.h>

// PNU : AP_PNUSAR is a PROTOCOL BACKEND of AP_PNU_CAN, not a CAN driver of its own.
// Protocol 17 (PNUSAR) was retired in step 3a - see README's SAR Integration Plan.  The
// SAR shares CAN1 with the PMU and the gimbal, and AP_CANManager cannot hand one
// interface to three drivers, so all three are backends behind one driver.
#ifndef AP_PNUSAR_ENABLED
#define AP_PNUSAR_ENABLED AP_PNU_CAN_ENABLED
#endif
