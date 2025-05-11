#include "Copter.h"

#ifdef USERHOOK_INIT

extern mavlink_sys_icd_flcc_gcs_inv_state_t MAV_GCSTX_INV_State;
extern mavlink_sys_icd_flcc_gcs_ccb_state_t MAV_GCSTX_CCB_State;
extern mavlink_sys_icd_flcc_gcs_hbsys_t MAV_GCSTX_HBSYS;

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

    MAV_GCSTX_CCB_State.Active_Mode = 1;
    MAV_GCSTX_CCB_State.Motor_MAX = 0;
    MAV_GCSTX_CCB_State.Motor_ON = 0;
    MAV_GCSTX_CCB_State.Brd_temp = 1;
    MAV_GCSTX_CCB_State.Flow_mL = 2;
    MAV_GCSTX_CCB_State.ThCp1x10 = 5;
    MAV_GCSTX_CCB_State.ThCp2x10 = 6;
    MAV_GCSTX_CCB_State.Thermistor1x10 = 1;
    MAV_GCSTX_CCB_State.Thermistor2x10 = 2;
    MAV_GCSTX_CCB_State.Thermistor3x10 = 3;
    MAV_GCSTX_CCB_State.Thermistor4x10 = 4;

    MAV_GCSTX_HBSYS.IFCU_State = 1;
    MAV_GCSTX_HBSYS.PMS_State = 0;
    MAV_GCSTX_HBSYS.HDC_Vout = 0.1;
    MAV_GCSTX_HBSYS.HDC_Cout = 0.2;
    MAV_GCSTX_HBSYS.HDC_Vin = 0.3;
    MAV_GCSTX_HBSYS.HDC_Cin = 0.4;
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
    //Loop rate : 10Hz
    static uint16_t Count = 0;
    //Send to GCS at 5Hz
    if (Count == 1) {
        gcs().send_message(MSG_INV_STATE); //
    }else if(Count == 3) {
        gcs().send_message(MSG_CCB_STATE);
    }else if(Count == 6) {
        gcs().send_message(MSG_HBSYS);
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
    AP::logger().Write("CCB", "TimeUS,FLOW,ACTIVE,MMAX,MON,TC1,TC2,TI1,TI2,TI3,TI4,BDTEMP", "QHBBBHHHHHH",
        AP_HAL::micros64(),
        MAV_GCSTX_CCB_State.Flow_mL,
        MAV_GCSTX_CCB_State.Active_Mode,
        MAV_GCSTX_CCB_State.Motor_MAX,
        MAV_GCSTX_CCB_State.Motor_ON,
        MAV_GCSTX_CCB_State.ThCp1x10,
        MAV_GCSTX_CCB_State.ThCp2x10,
        MAV_GCSTX_CCB_State.Thermistor1x10,
        MAV_GCSTX_CCB_State.Thermistor2x10,
        MAV_GCSTX_CCB_State.Thermistor3x10,
        MAV_GCSTX_CCB_State.Thermistor4x10
    );
    AP::logger().Write("HBSYS", "TimeUS,ISTAT,PSTAT,HVOUT,HCOUT,HVIN,HCIN", "QBBffff",
        AP_HAL::micros64(),
        MAV_GCSTX_HBSYS.IFCU_State,
        MAV_GCSTX_HBSYS.PMS_State,
        MAV_GCSTX_HBSYS.HDC_Vout,
        MAV_GCSTX_HBSYS.HDC_Cout,
        MAV_GCSTX_HBSYS.HDC_Cout,
        MAV_GCSTX_HBSYS.HDC_Cin
    );
    /*
    Format characters in the format string for binary log messages
    a   : int16_t[32]
    b   : int8_t
    B   : uint8_t
    h   : int16_t
    H   : uint16_t
    i   : int32_t
    I   : uint32_t
    f   : float
    d   : double
    n   : char[4]
    N   : char[16]
    Z   : char[64]
    c   : int16_t * 100
    C   : uint16_t * 100
    e   : int32_t * 100
    E   : uint32_t * 100
    L   : int32_t latitude/longitude
    M   : uint8_t flight mode
    q   : int64_t
    Q   : uint64_t
    */
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
