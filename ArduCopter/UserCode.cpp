#include "Copter.h"

#ifdef USERHOOK_INIT

extern mavlink_sys_icd_flcc_gcs_inv_state_t MAV_GCSTX_INV_State;
extern mavlink_sys_icd_flcc_gcs_ccb_state_t MAV_GCSTX_CCB_State;
extern mavlink_sys_icd_flcc_gcs_hbsys_t MAV_GCSTX_HBSYS;

void Copter::userhook_init()
{
    //Initialize UART port for Pegasus Actuators
    if (1 == Actuator_UART->is_initialized())
    {
        Actuator_UART->end();
    }
    Actuator_UART->begin(115200);
    gcs().send_text(MAV_SEVERITY_INFO, "Coaxial Actuator UART port initialized");

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

    //static variables
    static uint16_t Count1Hz = 0;
    static uint8_t Count5Hz = 0;

    //Send to GCS at 5Hz
    if(Count5Hz>1) {
        //gcs().send_message(MSG_COAXSERVO); 
        Count5Hz = 0;
    }
    Count5Hz++;

    //Send to GCS at 1Hz
    if (Count1Hz == 1) {
        //Get latest Inverter data
        MAV_GCSTX_INV_State.Inverter_OnOff = cxdata().INV_data.CMD_Flag.bits.Inverter_ONOFF;
        MAV_GCSTX_INV_State.Control_Mode = cxdata().INV_data.CMD_Flag.bits.Ctrl_Mode;
        MAV_GCSTX_INV_State.Motor_Speed = (uint16_t)cxdata().INV_data.motor_Spd;
        MAV_GCSTX_INV_State.Target_Motor_Speed = (uint16_t)cxdata().INV_data.Motor_RPM_CMD;
        MAV_GCSTX_INV_State.Motor_Speed_Limit = (uint16_t)cxdata().INV_data.Speed_Limit;
        MAV_GCSTX_INV_State.Target_Motor_Acceleration = (uint16_t)cxdata().INV_data.Motor_ACC_CMD;
        MAV_GCSTX_INV_State.Theta_Offset = (uint16_t)cxdata().INV_data.Theta_Offset;
        MAV_GCSTX_INV_State.i_a = (uint16_t)(cxdata().INV_data.i_a * 100.0);
        MAV_GCSTX_INV_State.i_b = (uint16_t)(cxdata().INV_data.i_b * 100.0);
        MAV_GCSTX_INV_State.i_c = (uint16_t)(cxdata().INV_data.i_c * 100.0);
        MAV_GCSTX_INV_State.t_a = (uint16_t)(cxdata().INV_data.t_a * 100.0);
        MAV_GCSTX_INV_State.t_b = (uint16_t)(cxdata().INV_data.t_b * 100.0);
        MAV_GCSTX_INV_State.t_c = (uint16_t)(cxdata().INV_data.t_c * 100.0);
        MAV_GCSTX_INV_State.V_dc = (uint16_t)(cxdata().INV_data.V_dc_input * 10.0);
        MAV_GCSTX_INV_State.Fault_Flags = cxdata().INV_data.FLT.ALL;
        gcs().send_message(MSG_INV_STATE); //
    }else if(Count1Hz == 3) {
        MAV_GCSTX_CCB_State.Active_Mode = cxdata().CCB_data.State.bits.IsActive;
        MAV_GCSTX_CCB_State.Motor_ON = ( cxdata().CCB_data.State.bits.Motor1_run | cxdata().CCB_data.State.bits.Motor2_run);
        MAV_GCSTX_CCB_State.Motor_MAX = cxdata().CCB_data.State.bits.IsForcedMax;
        MAV_GCSTX_CCB_State.Thermistor1x10 = cxdata().CCB_data.Thermistor1x10;
        MAV_GCSTX_CCB_State.Thermistor2x10 = cxdata().CCB_data.Thermistor2x10;
        MAV_GCSTX_CCB_State.Thermistor3x10 = cxdata().CCB_data.Thermistor3x10;
        MAV_GCSTX_CCB_State.Thermistor4x10 = cxdata().CCB_data.Thermistor4x10;
        MAV_GCSTX_CCB_State.ThCp1x10 = cxdata().CCB_data.ThCp1x10;
        MAV_GCSTX_CCB_State.ThCp2x10 = cxdata().CCB_data.ThCp2x10;
        MAV_GCSTX_CCB_State.Flow_mL = cxdata().CCB_data.Flow_mL;
        MAV_GCSTX_CCB_State.Brd_temp = cxdata().CCB_data.Brd_temp;
        gcs().send_message(MSG_CCB_STATE);
    }else if(Count1Hz == 6) {
        MAV_GCSTX_HBSYS.PMS_State = cxdata().DMI_PMS_data.PMS_State;
        MAV_GCSTX_HBSYS.IFCU_State = cxdata().IFCU_data.State;
        MAV_GCSTX_HBSYS.HDC_Vout = cxdata().DMI_PMS_data.HDC_OutputVoltage;
        MAV_GCSTX_HBSYS.HDC_Cout = cxdata().DMI_PMS_data.HDC_OutputCurrent;
        MAV_GCSTX_HBSYS.HDC_Vin = cxdata().DMI_PMS_data.HDC_InputVoltage;
        MAV_GCSTX_HBSYS.HDC_Cin = cxdata().DMI_PMS_data.HDC_InputCurrent;
        gcs().send_message(MSG_HBSYS);
        Count1Hz = 0;
    }
    Count1Hz++;

    AP::logger().Write("INV1", "TimeUS,ONOFF,RPM,RPMCMD,IA,IB,IC", "QBfffff",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().INV_data.CMD_Flag.bits.Inverter_ONOFF, //B     ONOFF
        cxdata().INV_data.motor_Spd,                    //f     RPM
        cxdata().INV_data.Motor_RPM_CMD,                //f     RPMCMD
        cxdata().INV_data.i_a,                          //f     IA
        cxdata().INV_data.i_b,                          //f     IB
        cxdata().INV_data.i_c                           //f     IC
    );
    AP::logger().Write("INV2", "TimeUS,MODE,RPMLIM,ACC,OFFSET,TA,TB,TC,VIN,FLTBIT", "QBHffffffB",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().INV_data.CMD_Flag.bits.Ctrl_Mode,      //B     MODE
        cxdata().INV_data.Speed_Limit,                  //H     RPMLIM
        cxdata().INV_data.Motor_ACC_CMD,                //f     ACC
        cxdata().INV_data.Theta_Offset,                 //f     OFFSET
        cxdata().INV_data.t_a,                          //f     TA
        cxdata().INV_data.t_b,                          //f     TB
        cxdata().INV_data.t_c,                          //f     TC
        cxdata().INV_data.V_dc_input,                   //f     VIN
        MAV_GCSTX_INV_State.Fault_Flags                 //B     FLTBIT
    );
    {
        uint8_t tempMON = cxdata().CCB_data.State.bits.Motor1_run | cxdata().CCB_data.State.bits.Motor2_run;
        AP::logger().Write("CCB", "TimeUS,FLOW,ACTIVE,MMAX,MON,TC1,TC2,TI1,TI2,TI3,TI4,BDTEMP", "QHBBBHHHHHHB",
            AP_HAL::micros64(),                         //Q     TimeUS
            cxdata().CCB_data.Flow_mL,                  //H     FLOW
            cxdata().CCB_data.State.bits.IsActive,      //B     ACTIVE
            cxdata().CCB_data.State.bits.IsForcedMax,   //B     MMAX
            tempMON,                                    //B     MON
            cxdata().CCB_data.ThCp1x10,                 //H     TC1
            cxdata().CCB_data.ThCp2x10,                 //H     TC2
            cxdata().CCB_data.Thermistor1x10,           //H     TI1
            cxdata().CCB_data.Thermistor1x10,           //H     TI2
            cxdata().CCB_data.Thermistor1x10,           //H     TI3
            cxdata().CCB_data.Thermistor1x10,           //H     TI4
            cxdata().CCB_data.Brd_temp                  //B     BDTEMP
        );
    }
    AP::logger().Write("HBSYS", "TimeUS,ISTAT,PSTAT,HVOUT,HCOUT,HVIN,HCIN", "QBBffff",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().IFCU_data.State,                       //B     ISTAT
        cxdata().DMI_PMS_data.PMS_State,                //B     PSTAT
        cxdata().DMI_PMS_data.HDC_OutputVoltage,        //f     HVOUT
        cxdata().DMI_PMS_data.HDC_OutputCurrent,        //f     HCOUT
        cxdata().DMI_PMS_data.HDC_InputVoltage,         //f     HVIN
        cxdata().DMI_PMS_data.HDC_InputCurrent          //f     HCIN
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
