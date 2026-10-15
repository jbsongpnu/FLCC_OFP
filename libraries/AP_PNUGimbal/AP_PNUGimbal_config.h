#pragma once

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
