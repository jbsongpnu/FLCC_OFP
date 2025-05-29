#include "Copter.h"

#define COAXSERVO_TEST 0
#define COAXCAN_LOGGING 0
#ifdef USERHOOK_INIT

extern mavlink_sys_icd_flcc_gcs_inv_state_t     MAV_GCSTX_INV_State;
extern mavlink_sys_icd_flcc_gcs_ccb_state_t     MAV_GCSTX_CCB_State;
extern mavlink_sys_icd_flcc_gcs_hbsys_t         MAV_GCSTX_HBSYS;
extern mavlink_sys_icd_flcc_gcs_cxsv_pos_t      MAV_GCSTX_CXSV_POS;
extern mavlink_sys_icd_flcc_gcs_cxsv_swash_t    MAV_GCSTX_CXSV_SWASH;
extern mavlink_sys_icd_flcc_gcs_dmi_data_t      MAV_GCSTX_DMI_data;
extern mavlink_sys_icd_flcc_gcs_hdm_data_t      MAV_GCSTX_HDM_data;

void Copter::userhook_init()
{
    //Initialize UART port for Pegasus Actuators
    // if (1 == Actuator_UART->is_initialized())
    // {
    //     Actuator_UART->end();
    // }
    // Actuator_UART->begin(115200);
    // gcs().send_text(MAV_SEVERITY_INFO, "Coaxial Actuator UART port initialized");

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

    MAV_GCSTX_CXSV_POS.Servo_State = 0;
    MAV_GCSTX_CXSV_POS.SV1_POS_RAW = 2000; 
    MAV_GCSTX_CXSV_POS.SV2_POS_RAW = 2001;
    MAV_GCSTX_CXSV_POS.SV3_POS_RAW = 2002;
    MAV_GCSTX_CXSV_POS.SV4_POS_RAW = 2003;
    MAV_GCSTX_CXSV_POS.SV5_POS_RAW = 2004;
    MAV_GCSTX_CXSV_POS.SV6_POS_RAW = 2005;

    MAV_GCSTX_CXSV_SWASH.Swash_State = 0; 
    MAV_GCSTX_CXSV_SWASH.Collective = -0.5;
    MAV_GCSTX_CXSV_SWASH.Cyclic_Lon = 0.2;
    MAV_GCSTX_CXSV_SWASH.Cyclic_Lat = 0.3;
    MAV_GCSTX_CXSV_SWASH.Pedal = 0.4;
    MAV_GCSTX_CXSV_SWASH.CMD_Collective = -0.4; 
    MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lon = 0.3;
    MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lat = 0.4;
    MAV_GCSTX_CXSV_SWASH.CMD_Pedal = 0.5;

    MAV_GCSTX_DMI_data.LDC_State = 0;
    MAV_GCSTX_DMI_data.PMS_Mv_Battery_VoltageX10 = 11;
    MAV_GCSTX_DMI_data.PMS_Mv_Output_CurrentX10 = 12;
    MAV_GCSTX_DMI_data.PMS_Batt_Out_CurrentX10 = 13;
    MAV_GCSTX_DMI_data.PMS_LDC_Output_CurrentX10 = 14;
    MAV_GCSTX_DMI_data.PMS_LDC_Output_VoltageX10 = 15;
    MAV_GCSTX_DMI_data.PMS_Output_PowerX10 = 100;
    MAV_GCSTX_DMI_data.PMS_Input_PowerX10 = 101;
    MAV_GCSTX_DMI_data.PMS_MAX_TempX10 = 200;

    MAV_GCSTX_HDM_data.Ifcu_PpCurLimX100 = 10000;
    MAV_GCSTX_HDM_data.Ifcu_PpH2SofX2 = 100;
    MAV_GCSTX_HDM_data.Ifcu_H2LkLmp = 0;
    MAV_GCSTX_HDM_data.Ifcu_FcNetVltX10 = 200;
    MAV_GCSTX_HDM_data.Ifcu_FcNetCurx10 = 10;
    MAV_GCSTX_HDM_data.Ifcu_FcInClntTmp = 11;
    MAV_GCSTX_HDM_data.Ifcu_AmbTemp = 12;
    MAV_GCSTX_HDM_data.Ifcu_RoomTemp = 13;
    MAV_GCSTX_HDM_data.Ifcu_H2TnkTmp = 14;
    MAV_GCSTX_HDM_data.Ifcu_H2TnkPrsX10 = 101;

    cxdata().SV_TX[0].SV_pos = 2024;
    cxdata().SVinitialized = 0;
    cxdata().CX_State = CoaxState::CXSTATE_0_INIT;
    cxdata().Swash.Col = 2.1;
    cxdata().Swash.Lat = 0.5;
    cxdata().Swash.Lon = -1.2;
    cxdata().Swash.Rud = 3.4;
    cxdata().Swash_CMD.Col = 2.1;
    cxdata().Swash_CMD.Lat = 0.5;
    cxdata().Swash_CMD.Lon = -1.2;
    cxdata().Swash_CMD.Rud = 3.4;
}
#endif

#ifdef USERHOOK_FASTLOOP
//100Hz
void Copter::userhook_FastLoop()
{
#if COAXSERVO_TEST == 0 //disabled during servo test
    static uint8_t SV_num = 0;
    uint8_t  CSBuf[64] = {0};

    AP_CoaxServo *ServoUART = AP::PegasusSV();

    switch (cxdata().CX_State) {
        case CoaxState::CXSTATE_0_INIT: {
            //1)Check for buffered data first
            uint16_t rcv = (uint16_t)ServoUART->receive_CoaxServo_uart_data(CSBuf);
            if(ServoUART->Parse_Buffer(CSBuf, rcv)) {
                uint8_t length = ServoUART->GET_RX_data_Length(0);
                if (length <=5) {
                    ServoUART->interprete_msg(0, ID_CMD_PADATA_PING);//Only expecting one message of ping-response
                }
            }
            //Expecting cxdata().SV_state[id].connected sets to 1 for all motors
            if ((cxdata().SV_state[0].connected) &&
                (cxdata().SV_state[1].connected) &&
                (cxdata().SV_state[2].connected) &&
                (cxdata().SV_state[3].connected) &&
                (cxdata().SV_state[4].connected) &&
                (cxdata().SV_state[5].connected) ) {
                cxdata().SVinitialized = 1;
            }
            
            //2) Send Ping to a Servo motor 1~6
            SV_num++; //looping 1~6 instead of 0~5
            ServoUART->CMD_PADATA_PING(SV_num);
            
            SV_num = (SV_num + 1) % 6;

            //3) Condition for next phase
            if(cxdata().SVinitialized) {
                cxdata().CX_State = CoaxState::CXSTATE_1_CHECK;
            }
        }break;
        case CoaxState::CXSTATE_1_CHECK: {
            //1)Check for buffered data first
            uint16_t rcv = ServoUART->receive_CoaxServo_uart_data(CSBuf);
            uint16_t messages = ServoUART->Parse_Buffer(CSBuf, rcv);
            for (int i=0; i<messages; i++) {
                uint8_t length = ServoUART->GET_RX_data_Length(i);
                if (length <= 5) {
                    ServoUART->interprete_msg(i, ID_CMD_PADATA_PING);
                } else {
                    ServoUART->interprete_msg(i, ID_CMD_PADATA_GET_PARAMETER);
                }
            }
            //3) Condition for next phase
            if ((cxdata().SV_state[0].got_param) &&
                (cxdata().SV_state[1].got_param) &&
                (cxdata().SV_state[2].got_param) &&
                (cxdata().SV_state[3].got_param) &&
                (cxdata().SV_state[4].got_param) &&
                (cxdata().SV_state[5].got_param)) {
                cxdata().CX_State = CoaxState::CXSTATE_2_WAIT;
            }
        }break;
        //Frome CXSTATE_2, servos will be controlled from 400Hz loop
        default:
            
        break;
    }
#endif
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

#if COAXSERVO_TEST == 1
    // Pegasus Servo testing
    static uint16_t temp_Pegasus_setting = 0;
    uint8_t  CSBuf[128] = {0};  //this should be reduced, but can only be usded for temporary servo test
    AP_CoaxServo *Pegasus = AP::PegasusSV();
#endif

    //Send to GCS at 1Hz Testing
    // MSG_INV_STATE,  // mavlink message to send Inverter state
    // MSG_HBSYS,      // mavlink message to send Hybrid-system state1
    // MSG_CCB_STATE,  // mavlink message to send CCB state
    // MSG_CXSV_POS,   // CoaxServo Position
    // MSG_CXSV_SWASH, // CoaxServo Swash-plate Angle
    // MSG_DMI_DATA,   // Data Requested by DMI
    // MSG_HDM_DATA,   // Data Requested by Hyundai Car
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
    } else if (Count1Hz == 2) {
        MAV_GCSTX_HBSYS.PMS_State = cxdata().DMI_PMS_data.PMS_State;
        MAV_GCSTX_HBSYS.IFCU_State = cxdata().IFCU_data.State;
        MAV_GCSTX_HBSYS.HDC_Vout = cxdata().DMI_PMS_data.HDC_OutputVoltage;
        MAV_GCSTX_HBSYS.HDC_Cout = cxdata().DMI_PMS_data.HDC_OutputCurrent;
        MAV_GCSTX_HBSYS.HDC_Vin = cxdata().DMI_PMS_data.HDC_InputVoltage;
        MAV_GCSTX_HBSYS.HDC_Cin = cxdata().DMI_PMS_data.HDC_InputCurrent;
        gcs().send_message(MSG_HBSYS);
    } else if (Count1Hz == 3) {
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
    } else if (Count1Hz == 4) {
        MAV_GCSTX_CXSV_POS.Servo_State = cxdata().SVinitialized;
        MAV_GCSTX_CXSV_POS.SV1_POS_RAW = cxdata().SV_Pos[0].raw;
        MAV_GCSTX_CXSV_POS.SV2_POS_RAW = cxdata().SV_Pos[1].raw;
        MAV_GCSTX_CXSV_POS.SV3_POS_RAW = cxdata().SV_Pos[2].raw;
        MAV_GCSTX_CXSV_POS.SV4_POS_RAW = cxdata().SV_Pos[3].raw;
        MAV_GCSTX_CXSV_POS.SV5_POS_RAW = cxdata().SV_Pos[4].raw;
        MAV_GCSTX_CXSV_POS.SV6_POS_RAW = cxdata().SV_Pos[5].raw;
        gcs().send_message(MSG_CXSV_POS);
    } else if (Count1Hz == 5) {
        MAV_GCSTX_CXSV_SWASH.Swash_State = static_cast<uint8_t>(cxdata().CX_State);
        MAV_GCSTX_CXSV_SWASH.Collective = cxdata().Swash.Col;
        MAV_GCSTX_CXSV_SWASH.Cyclic_Lon = cxdata().Swash.Lon;
        MAV_GCSTX_CXSV_SWASH.Cyclic_Lat = cxdata().Swash.Lat;
        MAV_GCSTX_CXSV_SWASH.Pedal      = cxdata().Swash.Rud;
        MAV_GCSTX_CXSV_SWASH.CMD_Collective = cxdata().Swash_CMD.Col;
        MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lon = cxdata().Swash_CMD.Lon;
        MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lat = cxdata().Swash_CMD.Lat;
        MAV_GCSTX_CXSV_SWASH.CMD_Pedal      = cxdata().Swash_CMD.Rud;
        gcs().send_message(MSG_CXSV_SWASH);
    } else if (Count1Hz == 6) {
        MAV_GCSTX_DMI_data.LDC_State = 0;
        MAV_GCSTX_DMI_data.PMS_Mv_Battery_VoltageX10 = 11;
        MAV_GCSTX_DMI_data.PMS_Mv_Output_CurrentX10 = 12;
        MAV_GCSTX_DMI_data.PMS_Batt_Out_CurrentX10 = 13;
        MAV_GCSTX_DMI_data.PMS_LDC_Output_CurrentX10 = 14;
        MAV_GCSTX_DMI_data.PMS_LDC_Output_VoltageX10 = 15;
        MAV_GCSTX_DMI_data.PMS_Output_PowerX10 = 100;
        MAV_GCSTX_DMI_data.PMS_Input_PowerX10 = 101;
        MAV_GCSTX_DMI_data.PMS_MAX_TempX10 = 200;

        gcs().send_message(MSG_DMI_DATA);
    } else if (Count1Hz == 10) {
        MAV_GCSTX_HDM_data.Ifcu_PpCurLimX100 = 10000;
        MAV_GCSTX_HDM_data.Ifcu_PpH2SofX2 = 100;
        MAV_GCSTX_HDM_data.Ifcu_H2LkLmp = 0;
        MAV_GCSTX_HDM_data.Ifcu_FcNetVltX10 = 200;
        MAV_GCSTX_HDM_data.Ifcu_FcNetCurx10 = 10;
        MAV_GCSTX_HDM_data.Ifcu_FcInClntTmp = 11;
        MAV_GCSTX_HDM_data.Ifcu_AmbTemp = 12;
        MAV_GCSTX_HDM_data.Ifcu_RoomTemp = 13;
        MAV_GCSTX_HDM_data.Ifcu_H2TnkTmp = 14;
        MAV_GCSTX_HDM_data.Ifcu_H2TnkPrsX10 = 101;
        gcs().send_message(MSG_HDM_DATA);
        Count1Hz = 0;
    }
    Count1Hz++;
#if COAXSERVO_TEST == 1 //For temporary servo test. Disabled for normal operation
    //Pegasus Servo Testing
    temp_Pegasus_setting++;
    {
        uint16_t rcv;
        rcv = (uint16_t)Pegasus->receive_CoaxServo_uart_data(CSBuf);
        Pegasus->Parse_Buffer(CSBuf, rcv);
        if(rcv == 5) {
            Pegasus->interprete_msg(0, ID_CMD_PADATA_PING);
        } else if (rcv > 5) {
            Pegasus->interprete_msg(0, ID_CMD_PADATA_GET_MOT_TEMP_C); 
        }
        
    }
    //There was error since V0.01.27, some memory overflow that yields 0x400020 Internal Error+Panic
    if((temp_Pegasus_setting % 5)==1) {
        Pegasus->CMD_PADATA_PING(0x1F); 
    } else if ((temp_Pegasus_setting % 5)==2) {
        //Pegasus->Request_Servo_Pos(1);
        //Pegasus->Request_Servo_Current(1);
        //Pegasus->Request_all_Param(1);
        Pegasus->Request_Servo_Temp(1);

    } else {// ((temp_Pegasus_setting % 5)==3) {
        cxdata().SV_TX[0].SV_pos = 2024;
        //gcs().send_text(MAV_SEVERITY_ERROR, "TX SV : %u", cxdata().SV_TX[0].SV_pos);
        Pegasus->Set_Coax_ServoPosition();
    }
    
    temp_Pegasus_setting = temp_Pegasus_setting % 100;
#endif
#if COAXCAN_LOGGING == 1
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
#endif
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
