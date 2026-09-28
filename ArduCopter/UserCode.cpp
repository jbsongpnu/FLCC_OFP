#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // PNU : report avoidance as off until KGCS commands a level
    gcs().OA_Status.Object_Avoidance_Mode = 0;
    gcs().GCS_Ctrl_OA_Mode.OA_Mode = 0;
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    gcs().send_message(MSG_CAM_STATUS);                 // PNU : TM2 camera attitude / zoom

#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED
    // PNU : TM5 reports the avoidance level actually in force, not the one KGCS
    // asked for.  proximity_avoidance_enable() can also be cleared elsewhere, and
    // proximity_avoidance_enabled() additionally requires AVOID_ENABLE bit 1
    // (AC_AVOID_USE_PROXIMITY_SENSOR), so the commanded level is echoed back only
    // while avoidance is genuinely active.
    if (avoid.proximity_avoidance_enabled()) {
        gcs().OA_Status.Object_Avoidance_Mode = gcs().GCS_Ctrl_OA_Mode.OA_Mode;
    } else {
        gcs().OA_Status.Object_Avoidance_Mode = 0;
    }

    gcs().send_message(MSG_OBJECT_AVOIDANCE_STATUS);    // PNU : TM5 object avoidance status
#endif
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
