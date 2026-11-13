#pragma once

#include <AP_PNU_CAN/AP_PNU_CAN_config.h>

// PNU : AP_PMUCAN is the PMU protocol backend of AP_PNU_CAN, so it is in the build
// whenever the driver is - there is no separate flag for the PMU itself.
//
// PMU-005a stage 3 : THE ECU MESSAGES ARE THE PART THAT CAN BE LEFT OUT.  MODESTS and
// ENERGYSTS exist only on ECU software 2.00+, and an aircraft fitted with a legacy PMU
// carries their decoder, their counters and their two dataflash records for nothing.
// Everything the two generations share - the five commands, the five status messages,
// the health bucket and the step 2c interlock - stays outside this flag, so a build with
// it off is exactly the V5.1.28 PMU behaviour.
#ifndef AP_PMUCAN_ECU_ENABLED
#define AP_PMUCAN_ECU_ENABLED AP_PNU_CAN_ENABLED
#endif

#if AP_PMUCAN_ECU_ENABLED && !AP_PNU_CAN_ENABLED
#error "AP_PMUCAN_ECU_ENABLED requires AP_PNU_CAN_ENABLED"
#endif
