#include "Copter.h"

#ifdef USERHOOK_INIT

extern mavlink_sys_icd_flcc_gcs_inv_state_t MAV_GCSTX_INV_State;

void Copter::userhook_init()
{
    MAV_GCSTX_INV_State.Inverter_OnOff = 2;
    MAV_GCSTX_INV_State.Control_Mode = 4;
    MAV_GCSTX_INV_State.Target_Motor_Acceleration = 100; //100rpm/s
    MAV_GCSTX_INV_State.Motor_Speed_Limit = 5000;

    MAV_GCSTX_INV_State.Motor_Speed = 0;
    MAV_GCSTX_INV_State.Target_Motor_Speed = 0;
    MAV_GCSTX_INV_State.i_a = 0;
    MAV_GCSTX_INV_State.i_b = 0;
    MAV_GCSTX_INV_State.i_c = 0;
    
    MAV_GCSTX_INV_State.Theta_Offset = 55;
    MAV_GCSTX_INV_State.t_a = 0;
    MAV_GCSTX_INV_State.t_b = 0;
    MAV_GCSTX_INV_State.t_c = 0;
    MAV_GCSTX_INV_State.V_dc = 0;
    MAV_GCSTX_INV_State.Fault_Flags = 0;

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
    //User Code for Coaxial Helicopter 

    gcs().send_message(MSG_INV_STATE); //Send to GCS 
    //Loop rate : 10Hz
    static uint16_t Count = 0;
    if (Count > 9) {
        //gcs().send_text(MAV_SEVERITY_INFO, "Testing FCC Ready %u", (cxdata().fcrdy & 0x01));
        Count = 0;
    }
    Count++;

    AP::logger().Write("INV1", "TimeUS,ONOFF,RPM,RPMCMD,IA,IB,IC", "QBHHHHH",
        AP_HAL::micros64(),
        MAV_GCSTX_INV_State.Inverter_OnOff,
        MAV_GCSTX_INV_State.Motor_Speed,
        MAV_GCSTX_INV_State.Target_Motor_Speed,
        MAV_GCSTX_INV_State.i_a,
        MAV_GCSTX_INV_State.i_b,
        MAV_GCSTX_INV_State.i_c
    );
    AP::logger().Write("INV2", "TimeUS,MODE,RPMLIM,ACC,OFFSET,TA,TB,TC,VIN,FLTBIT", "QBHHHHHHH",
        AP_HAL::micros64(),
        MAV_GCSTX_INV_State.Control_Mode,
        MAV_GCSTX_INV_State.Motor_Speed_Limit,
        MAV_GCSTX_INV_State.Target_Motor_Acceleration,
        MAV_GCSTX_INV_State.Theta_Offset,
        MAV_GCSTX_INV_State.t_a,
        MAV_GCSTX_INV_State.t_b,
        MAV_GCSTX_INV_State.t_c,
        MAV_GCSTX_INV_State.V_dc,
        MAV_GCSTX_INV_State.Fault_Flags
    );
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
