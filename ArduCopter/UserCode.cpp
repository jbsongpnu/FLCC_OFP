#include "Copter.h"

//Pegasus suervo has changed to HiTech servos. 
//Following code contains temporary debugging 
#define COAXSERVO_TEST 0
#define COAXCAN_LOGGING 0
#define CCB_AUTOSEQUENCE 1
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
    //Initialize UART port for HiTech Actuators
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
    
    MAV_GCSTX_INV_State.Motor_Aligned = 55;
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

    cxdata().INV_data.Rdy2useINV = 0;
    cxdata().INV_data.pre_Rdy2useINV = 0;
    cxdata().Command_Received.NewCMD.bits.CCB_Motor_MAX = 0;

    cxdata().SV_TX[0].SV_pos = 950;
    cxdata().SV_TX[1].SV_pos = 1024;
    cxdata().SV_TX[2].SV_pos = 950;
    cxdata().SV_TX[3].SV_pos = 1024;
    cxdata().SV_TX[4].SV_pos = 1024;
    cxdata().SV_TX[5].SV_pos = 890;

    cxdata().SVTestState.ServoTestingID = 0;//Initiate with ID 0
}
#endif

#ifdef USERHOOK_FASTLOOP
//100Hz
void Copter::userhook_FastLoop()
{
// #if COAXSERVO_TEST == 0 //disabled during servo test
//     static uint8_t SV_num = 0;
//     uint8_t  CSBuf[64] = {0};

//     AP_CoaxServo *ServoUART = AP::HiTechSV();

//     switch (cxdata().CX_State) {
//         case CoaxState::CXSTATE_0_INIT: {
//             //1)Check for buffered data first
//             uint16_t rcv = (uint16_t)ServoUART->receive_CoaxServo_uart_data(CSBuf);
//             if(ServoUART->Parse_Buffer(CSBuf, rcv)) {
//                 uint8_t length = ServoUART->GET_RX_data_Length(0);
//                 if (length <=5) {
//                     ServoUART->interprete_msg(0, ID_CMD_PADATA_PING);//Only expecting one message of ping-response
//                 }
//             }
//             //Expecting cxdata().SV_state[id].connected sets to 1 for all motors
//             if ((cxdata().SV_state[0].connected) &&
//                 (cxdata().SV_state[1].connected) &&
//                 (cxdata().SV_state[2].connected) &&
//                 (cxdata().SV_state[3].connected) &&
//                 (cxdata().SV_state[4].connected) &&
//                 (cxdata().SV_state[5].connected) ) {
//                 cxdata().SVinitialized = 1;
//             }
            
//             //2) Send Ping to a Servo motor 1~6
//             SV_num++; //looping 1~6 instead of 0~5
//             ServoUART->CMD_PADATA_PING(SV_num);
            
//             SV_num = (SV_num + 1) % 6;

//             //3) Condition for next phase
//             if(cxdata().SVinitialized) {
//                 cxdata().CX_State = CoaxState::CXSTATE_1_CHECK;
//             }
//         }break;
//         case CoaxState::CXSTATE_1_CHECK: {
//             //1)Check for buffered data first
//             uint16_t rcv = ServoUART->receive_CoaxServo_uart_data(CSBuf);
//             uint16_t messages = ServoUART->Parse_Buffer(CSBuf, rcv);
//             for (int i=0; i<messages; i++) {
//                 uint8_t length = ServoUART->GET_RX_data_Length(i);
//                 if (length <= 5) {
//                     ServoUART->interprete_msg(i, ID_CMD_PADATA_PING);
//                 } else {
//                     ServoUART->interprete_msg(i, ID_CMD_PADATA_GET_PARAMETER);
//                 }
//             }
//             //3) Condition for next phase
//             if ((cxdata().SV_state[0].got_param) &&
//                 (cxdata().SV_state[1].got_param) &&
//                 (cxdata().SV_state[2].got_param) &&
//                 (cxdata().SV_state[3].got_param) &&
//                 (cxdata().SV_state[4].got_param) &&
//                 (cxdata().SV_state[5].got_param)) {
//                 cxdata().CX_State = CoaxState::CXSTATE_2_WAIT;
//             }
//         }break;
//         //Frome CXSTATE_2, servos will be controlled from 400Hz loop
//         default:
            
//         break;
//     }
// #endif
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

    //uint8_t NewServoMessages = 0;
#if COAXSERVO_TEST == 1
    static uint16_t ServoTestStep = 0;
    static uint8_t ServoTestingID = 0;
    static uint8_t SVDataRequested = 0;
    static uint8_t SVConfigModified = 0;
    static uint8_t ServoCheckFinished = 0;
    static uint8_t temp_debug_retry = 0;
#endif    

    //uint16_t rcv = 0;
    // HiTech Servo testing
    //static uint16_t temp_HiTech_setting = 0;
    //uint8_t  CSBuf[16] = {0};  //this should be reduced, but can only be usded for temporary servo test
//    AP_CoaxServo *HiTech = AP::HiTechSV();

    //static uint8_t ID_init = 0;
    //static uint8_t ID_Cycle = 0;


    //Send to GCS at 1Hz Testing
    // MSG_INV_STATE,  // mavlink message to send Inverter state
    // MSG_HBSYS,      // mavlink message to send Hybrid-system state1
    // MSG_CCB_STATE,  // mavlink message to send CCB state
    // MSG_CXSV_POS,   // CoaxServo Position
    // MSG_CXSV_SWASH, // CoaxServo Swash-plate Angle
    // MSG_DMI_DATA,   // Data Requested by DMI
    // MSG_HDM_DATA,   // Data Requested by Hyundai Car
    if (Count1Hz%10 == 1) {
        //Get latest Inverter data
        MAV_GCSTX_INV_State.Inverter_OnOff = cxdata().INV_data.CMD_Flag.bits.Inverter_ONOFF;
        MAV_GCSTX_INV_State.Control_Mode = cxdata().INV_data.CMD_Flag.bits.Ctrl_Mode;
        MAV_GCSTX_INV_State.Motor_Speed = (uint16_t)cxdata().INV_data.motor_Spd;
        MAV_GCSTX_INV_State.Target_Motor_Speed = (uint16_t)cxdata().INV_data.Motor_RPM_CMD;
        MAV_GCSTX_INV_State.Motor_Speed_Limit = (uint16_t)cxdata().INV_data.Speed_Limit;
        MAV_GCSTX_INV_State.Target_Motor_Acceleration = (uint16_t)cxdata().INV_data.Motor_ACC_CMD;
        MAV_GCSTX_INV_State.Motor_Aligned = 1;//(uint16_t)cxdata().INV_data.Motor_Align_flag;//Thetaoffset has changed to Align_flag
        MAV_GCSTX_INV_State.i_a = (uint16_t)(cxdata().INV_data.i_a * 100.0);
        MAV_GCSTX_INV_State.i_b = (uint16_t)(cxdata().INV_data.i_b * 100.0);
        MAV_GCSTX_INV_State.i_c = (uint16_t)(cxdata().INV_data.i_c * 100.0);
        MAV_GCSTX_INV_State.t_a = (uint16_t)(cxdata().INV_data.t_a * 100.0);
        MAV_GCSTX_INV_State.t_b = (uint16_t)(cxdata().INV_data.t_b * 100.0);
        MAV_GCSTX_INV_State.t_c = (uint16_t)(cxdata().INV_data.t_c * 100.0);
        MAV_GCSTX_INV_State.V_dc = (uint16_t)(cxdata().INV_data.V_dc_input * 10.0);
        MAV_GCSTX_INV_State.Fault_Flags = 7;//cxdata().INV_data.FLT.ALL;
        gcs().send_message(MSG_INV_STATE); //
    } else if (Count1Hz%10 == 2) {
        MAV_GCSTX_HBSYS.PMS_State = cxdata().DMI_PMS_data.PMS_State;
        MAV_GCSTX_HBSYS.IFCU_State = cxdata().IFCU_data.State;
        MAV_GCSTX_HBSYS.HDC_Vout = cxdata().DMI_PMS_data.HDC_OutputVoltage;
        MAV_GCSTX_HBSYS.HDC_Cout = cxdata().DMI_PMS_data.HDC_OutputCurrent;
        MAV_GCSTX_HBSYS.HDC_Vin = cxdata().DMI_PMS_data.HDC_InputVoltage;
        MAV_GCSTX_HBSYS.HDC_Cin = cxdata().DMI_PMS_data.HDC_InputCurrent;
        gcs().send_message(MSG_HBSYS);
    } else if (Count1Hz%10 == 3) {
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
    } else if (Count1Hz%10 == 4) {
        MAV_GCSTX_CXSV_POS.Servo_State = cxdata().SVinitialized;
        MAV_GCSTX_CXSV_POS.SV1_POS_RAW = cxdata().SV_Pos[0].raw;
        MAV_GCSTX_CXSV_POS.SV2_POS_RAW = cxdata().SV_Pos[1].raw;
        MAV_GCSTX_CXSV_POS.SV3_POS_RAW = cxdata().SV_Pos[2].raw;
        MAV_GCSTX_CXSV_POS.SV4_POS_RAW = cxdata().SV_Pos[3].raw;
        MAV_GCSTX_CXSV_POS.SV5_POS_RAW = cxdata().SV_Pos[4].raw;
        MAV_GCSTX_CXSV_POS.SV6_POS_RAW = cxdata().SV_Pos[5].raw;
        gcs().send_message(MSG_CXSV_POS);
    } else if (Count1Hz%10 == 5) {
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
    } else if (Count1Hz%10 == 6) {
        MAV_GCSTX_DMI_data.LDC_State = cxdata().DMI_PMS_data.LDC_State;
        MAV_GCSTX_DMI_data.PMS_Mv_Battery_VoltageX10 = (uint16_t)(cxdata().DMI_PMS_data.Mv_Battery_Voltage * 10.0);
        MAV_GCSTX_DMI_data.PMS_Mv_Output_CurrentX10 = (uint16_t)(cxdata().DMI_PMS_data.Mv_Output_Current * 10.0);
        MAV_GCSTX_DMI_data.PMS_Batt_Out_CurrentX10 = (uint16_t)(cxdata().DMI_PMS_data.Batt_Output_Current * 10.0);
        MAV_GCSTX_DMI_data.PMS_LDC_Output_CurrentX10 = (uint16_t)(cxdata().DMI_PMS_data.LDC_Output_Current * 10.0);
        MAV_GCSTX_DMI_data.PMS_LDC_Output_VoltageX10 = (uint16_t)(cxdata().DMI_PMS_data.PMS_LDC_Out_Volt * 10.0);
        MAV_GCSTX_DMI_data.PMS_Output_PowerX10 = (uint16_t)(cxdata().DMI_PMS_data.PMS_Out_Power * 10.0);
        MAV_GCSTX_DMI_data.PMS_Input_PowerX10 = (uint16_t)(cxdata().DMI_PMS_data.PMS_In_Power * 10.0);
        MAV_GCSTX_DMI_data.PMS_MAX_TempX10 = (uint16_t)(cxdata().DMI_PMS_data.PMS_Max_Temp * 10.0);

        gcs().send_message(MSG_DMI_DATA);
    //} else if (Count1Hz%10 == 7) {
    //    gcs().send_text(MAV_SEVERITY_INFO, "SVID %u Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
    } else if (Count1Hz%10 == 0) {
        MAV_GCSTX_HDM_data.Ifcu_PpCurLimX100 = (uint16_t)(cxdata().IFCU_data.PpCurLim * 100.0);
        MAV_GCSTX_HDM_data.Ifcu_PpH2SofX2 = (uint16_t)(cxdata().IFCU_data.PpH2Sof * 2);
        MAV_GCSTX_HDM_data.Ifcu_H2LkLmp = cxdata().IFCU_data.H2LkLmp;
        MAV_GCSTX_HDM_data.Ifcu_FcNetVltX10 = (uint16_t)(cxdata().IFCU_data.FcNetVlt * 10.0);
        MAV_GCSTX_HDM_data.Ifcu_FcNetCurx10 = (uint16_t)(cxdata().IFCU_data.FcNetCur * 10.0);
        MAV_GCSTX_HDM_data.Ifcu_FcInClntTmp = cxdata().IFCU_data.FcInClntTmp;
        MAV_GCSTX_HDM_data.Ifcu_AmbTemp = cxdata().IFCU_data.AmbTemp;
        MAV_GCSTX_HDM_data.Ifcu_RoomTemp = cxdata().IFCU_data.RoomTemp;
        MAV_GCSTX_HDM_data.Ifcu_H2TnkTmp = cxdata().IFCU_data.H2TnkTmp;
        MAV_GCSTX_HDM_data.Ifcu_H2TnkPrsX10 = cxdata().IFCU_data.H2TnkPrs;
        gcs().send_message(MSG_HDM_DATA);
#if CCB_AUTOSEQUENCE == 1
        if (Count1Hz > 50) {
            cxdata().Command_Received.NewCMD.bits.CCB_Motor_MAX = 1;
        }
#endif
    }
//#if COAXSERVO_TEST == 1
    if((Count1Hz == 108) && (cxdata().SVTestState.ServoTestingID == 0)) { 
        cxdata().SVTestState.ServoTestingID = 1;

    }
//#endif
    //============Check HiTech Servo Configurations=======
    //HiTech Servo ID starts from 1 to 6
    //cxdata's servo array starts from 0 to 5
    //NewServoMessages = HiTech->receive_CoaxServo_uart_data();
#if COAXSERVO_TEST == 1
    //gcs().send_text(MAV_SEVERITY_INFO, "%d New Servo MSGs at %u for %u", NewServoMessages, Count1Hz, ServoTestingID);

    if((ServoTestingID > 0) && (ServoTestingID < 7)) {
        switch (ServoTestStep) {
            case 0 :    //Check ID
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_SERVO_ID); 
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting ID check to SV %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].connected) {   
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Connected and under check", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Request_SVData(ServoTestingID, REG_SERVO_ID);//Retry
                        }
                    } 
                    else {//temp test : TX not working
                        temp_debug_retry++;
                        // if(temp_debug_retry <= 4) {
                            HiTech->Request_SVData(ServoTestingID, REG_SERVO_ID);//temp test : TX not working
                        // }
                    }
                }
            break;
            case 1 :    //Check Return Delay
                if(SVDataRequested == 0) {
                    HiTech->Set_dummyTX();
                    HiTech->Request_SVData(ServoTestingID, REG_NORMAL_RETURN_DELAY);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_NORMAL_RETURN_DELAY %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Delay == ServoTestingID) { 
                            //servo 1 : 1ms, servo 2 : 2ms .... delay is same as servo number
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Return delay OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_NORMAL_RETURN_DELAY, (uint16_t)ServoTestingID);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Return Delay : Servo %u Return delay=%u", ServoTestingID, ServoTestingID);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_NORMAL_RETURN_DELAY);
                    }
                }
            break;
            case 2 :    //Check Power Config
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_POWER_CONFIG);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_POWER_CONFIG %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Power_Config == PARAM_POWER_CONFIG) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Power Config OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_POWER_CONFIG, PARAM_POWER_CONFIG);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Power Config : Servo %u Config value=%u", ServoTestingID, PARAM_POWER_CONFIG);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_POWER_CONFIG);
                    }
                }
            break;
            case 3 :    //Check Emergency Stop
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_EMERGENCY_STOP);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_EMERGENCY_STOP %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Emergency_Stop == PARAM_EMERGENCY_STOP) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u EM Stop failsafe OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_EMERGENCY_STOP, PARAM_EMERGENCY_STOP);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure EM Stop failsafe : Servo %u  Config value=%u", ServoTestingID, PARAM_EMERGENCY_STOP);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_EMERGENCY_STOP);
                    }
                }
            break;
            case 4 :    //Check Action Mode
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_ACTION_MODE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_ACTION_MODE %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Action_Mode == PARAM_ACTION_MODE) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Action Mode OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_ACTION_MODE, PARAM_ACTION_MODE);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Action Mode : Servo %u  Config value=%u", ServoTestingID, PARAM_ACTION_MODE);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_ACTION_MODE);
                    }
                }
            break;
            case 5 :    //Check Position Slope
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_POSITION_SLOPE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_POSITION_SLOPE %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Pos_Slope == PARAM_POSITION_SLOPE) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Position Slope OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_POSITION_SLOPE, PARAM_POSITION_SLOPE);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Position Slope : Servo %u  Config value=%u", ServoTestingID, PARAM_POSITION_SLOPE);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_POSITION_SLOPE);
                    }
                }
            break;
            case 6 :    //Check Dead Band
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_DEAD_BAND);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_DEAD_BAND %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Dead_band == PARAM_DEAD_BAND) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Dead Band OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_DEAD_BAND, PARAM_DEAD_BAND);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Dead Band : Servo %u  Config value=%u", ServoTestingID, PARAM_DEAD_BAND);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_DEAD_BAND);
                    }
                }
            break;
            case 7 :    //Check Velocity Max
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_VELOCITY_MAX);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_DEAD_BAND %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Velocity_Max == PARAM_VELOCITY_MAX) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Velocity Max OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_VELOCITY_MAX, PARAM_VELOCITY_MAX);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Velocity Max : Servo %u  Config value=%u", ServoTestingID, PARAM_VELOCITY_MAX);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_VELOCITY_MAX);
                    }
                }
            break;
            case 8 :    //Check Torque Max
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_TORQUE_MAX);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_DEAD_BAND %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Torque_Max == PARAM_TORQUE_MAX) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Torque Max OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_TORQUE_MAX, PARAM_TORQUE_MAX);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Torque Max : Servo %u  Config value=%u", ServoTestingID, PARAM_TORQUE_MAX);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_TORQUE_MAX);
                    }
                }
            break;
            case 9 :    //Check Voltage Max
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_VOLTAGE_MAX);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting Voltage Max %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Volt_Max == PARAM_VOLTAGE_MAX) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Voltage Max OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_VOLTAGE_MAX, PARAM_VOLTAGE_MAX);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Voltage Max : Servo %u  Config value=%u", ServoTestingID, PARAM_VOLTAGE_MAX);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_VOLTAGE_MAX);
                    }
                }
            break;
            case 10 :    //Check Voltage Min
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_VOLTAGE_MIN);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting Voltage Min %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Volt_Min == PARAM_VOLTAGE_MIN) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Voltage Min OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_VOLTAGE_MIN, PARAM_VOLTAGE_MIN);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Voltage Min : Servo %u  Config value=%u", ServoTestingID, PARAM_VOLTAGE_MIN);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_VOLTAGE_MIN);
                    }
                }
            break;
            case 11 :    //Check Temperature Max
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_TEMP_MAX);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting Temperature Max %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Temp_Max == PARAM_TEMP_MAX) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Temperature Max OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_TEMP_MAX, PARAM_TEMP_MAX);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Temperature Max : Servo %u  Config value=%u", ServoTestingID, PARAM_TEMP_MAX);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_TEMP_MAX);
                    }
                }
            break;
            case 12 :    //Check Temperature Min
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_TEMP_MIN);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting Temperature Min %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Temp_Min == PARAM_TEMP_MIN) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Temperature Min OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_UINT_Config(ServoTestingID, REG_TEMP_MIN, PARAM_TEMP_MIN);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Temperature Min : Servo %u  Config value=%u", ServoTestingID, PARAM_TEMP_MIN);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_TEMP_MIN);
                    }
                }
            break;
            case 13 :    //Check Position Start
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_POS_START);
                    gcs().send_text(MAV_SEVERITY_INFO, "Position Start %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Pos_Start == PARAM_POS_START) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Position Start OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_INT_Config(ServoTestingID, REG_POS_START, PARAM_POS_START);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Position Start : Servo %u  Config value=%u", ServoTestingID, PARAM_POS_START);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_POS_START);
                    }
                }
            break;
            case 14 :    //Check Position End
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_POS_END);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting Position End %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Pos_End == PARAM_POS_END) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Position End OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_INT_Config(ServoTestingID, REG_POS_END, PARAM_POS_END);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Position End : Servo %u  Config value=%u", ServoTestingID, PARAM_POS_END);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_POS_END);
                    }
                }
            break;
            case 15 :    //Check Position Neutral
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_POS_NEUTRAL);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting Position Neutral %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        if(cxdata().SV_state[(ServoTestingID-1)].Config_Pos_Neutral == PARAM_POS_NEUTRAL) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Position Neutral OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            HiTech->Set_INT_Config(ServoTestingID, REG_POS_NEUTRAL, PARAM_POS_NEUTRAL);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Position Neutral : Servo %u  Config value=%u", ServoTestingID, PARAM_POS_NEUTRAL);
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    } else {
                        HiTech->Request_SVData(ServoTestingID, REG_POS_NEUTRAL);
                    }
                }
            break;
            case 16 :    //Check Turn Direction
            {   uint8_t isDirectionOK = 0;
                if(SVDataRequested == 0) {
                    HiTech->Request_SVData(ServoTestingID, REG_MOTOR_TURN_DIRECT);
                    gcs().send_text(MAV_SEVERITY_INFO, "Requesting Turn Direction %u ", ServoTestingID);
                    SVDataRequested = 1;
                } else {
                    if(NewServoMessages) {
                        switch(ServoTestingID) {
                            case 1 :
                                if(cxdata().SV_state[(ServoTestingID-1)].Config_Direnction == PARAM_SV1_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    HiTech->Set_UINT_Config(ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV1_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Turn Direction : Servo %u  Config value=%u", ServoTestingID, PARAM_SV1_T_Direction);
                                }
                            break;
                            case 2 :
                                if(cxdata().SV_state[(ServoTestingID-1)].Config_Direnction == PARAM_SV2_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    HiTech->Set_UINT_Config(ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV2_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Turn Direction : Servo %u  Config value=%u", ServoTestingID, PARAM_SV2_T_Direction);
                                }
                            break;
                            case 3 :
                                if(cxdata().SV_state[(ServoTestingID-1)].Config_Direnction == PARAM_SV3_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    HiTech->Set_UINT_Config(ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV3_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Turn Direction : Servo %u  Config value=%u", ServoTestingID, PARAM_SV3_T_Direction);
                                }
                            break;
                            case 4 :
                                if(cxdata().SV_state[(ServoTestingID-1)].Config_Direnction == PARAM_SV4_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    HiTech->Set_UINT_Config(ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV4_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Turn Direction : Servo %u  Config value=%u", ServoTestingID, PARAM_SV4_T_Direction);
                                }
                            break;
                            case 5 :
                                if(cxdata().SV_state[(ServoTestingID-1)].Config_Direnction == PARAM_SV5_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    HiTech->Set_UINT_Config(ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV5_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Turn Direction : Servo %u  Config value=%u", ServoTestingID, PARAM_SV5_T_Direction);
                                }
                            break;
                            case 6 :
                                if(cxdata().SV_state[(ServoTestingID-1)].Config_Direnction == PARAM_SV6_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    HiTech->Set_UINT_Config(ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV6_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfigure Turn Direction : Servo %u  Config value=%u", ServoTestingID, PARAM_SV6_T_Direction);
                                }
                            break;
                            default :
                            break;
                        }
                        if(isDirectionOK) { 
                            gcs().send_text(MAV_SEVERITY_INFO, "Servo %u Turn Direction OK", ServoTestingID);
                            ServoTestStep++;
                            SVDataRequested = 0;
                        } else {
                            SVDataRequested = 0;    //Reset SVDataRequested to check again
                            SVConfigModified = 1;   //Notice configuration chagne
                        }
                    }
                     else {
                        HiTech->Request_SVData(ServoTestingID, REG_MOTOR_TURN_DIRECT);
                    }
                }
            }
            break;
            case 17 :
                if(SVConfigModified) {
                    HiTech->Set_UINT_Config(ServoTestingID, REG_CONFIG_SAVE, 0xFFFF);
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Servo %u Needs Rebooting!!", ServoTestingID);
                } 
                ServoTestStep = 0;
                SVDataRequested = 0;
                SVConfigModified = 0;
                if(ServoTestingID == 6) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Servo Check Finished. Check messages and reboot if reuired");
                    ServoCheckFinished = 1;
                    Count1Hz = 0;
                }
                ServoTestingID++;
            break;
            default :
            break;
        }
    }


    if(ServoCheckFinished){
        
        if (Count1Hz%10 == 1) {
            HiTech->Request_SVData(1,REG_POSITION);
            HiTech->Request_SVData(2,REG_POSITION);
            HiTech->Request_SVData(3,REG_POSITION);
            HiTech->Request_SVData(4,REG_POSITION);
            HiTech->Request_SVData(5,REG_POSITION);
            HiTech->Request_SVData(6,REG_POSITION);
        }else if (Count1Hz%10 == 2) {
            HiTech->CMD_SET_VELOCITY(1,4095);HiTech->CMD_SET_VELOCITY(2,4095);HiTech->CMD_SET_VELOCITY(3,4095);
            HiTech->CMD_SET_VELOCITY(4,4095);HiTech->CMD_SET_VELOCITY(5,4095);HiTech->CMD_SET_VELOCITY(6,4095);
        }else if (Count1Hz%10 == 3) {
            HiTech->CMD_SET_TORQUE(1,4095);HiTech->CMD_SET_TORQUE(2,4095);HiTech->CMD_SET_TORQUE(3,4095);
            HiTech->CMD_SET_TORQUE(4,4095);HiTech->CMD_SET_TORQUE(5,4095);HiTech->CMD_SET_TORQUE(6,4095);
        }else {
            HiTech->CMD_SET_POSITION(1,cxdata().SV_TX[0].SV_pos);
            HiTech->CMD_SET_POSITION(2,cxdata().SV_TX[1].SV_pos);
            HiTech->CMD_SET_POSITION(3,cxdata().SV_TX[2].SV_pos);
            HiTech->CMD_SET_POSITION(4,cxdata().SV_TX[3].SV_pos);
            HiTech->CMD_SET_POSITION(5,cxdata().SV_TX[4].SV_pos);
            HiTech->CMD_SET_POSITION(6,cxdata().SV_TX[5].SV_pos);
        }
    }
#else
    /*if (Count1Hz%10 == 1) {
        HiTech->Request_SVData(1,REG_POSITION);
        HiTech->Request_SVData(2,REG_POSITION);
        HiTech->Request_SVData(3,REG_POSITION);
        HiTech->Request_SVData(4,REG_POSITION);
        HiTech->Request_SVData(5,REG_POSITION);
        HiTech->Request_SVData(6,REG_POSITION);
    }else if (Count1Hz%10 == 2) {
        if(NewServoMessages == 0) {
            HiTech->Request_SVData(1,REG_POSITION);
            HiTech->Request_SVData(2,REG_POSITION);
            HiTech->Request_SVData(3,REG_POSITION);
            HiTech->Request_SVData(4,REG_POSITION);
            HiTech->Request_SVData(5,REG_POSITION);
            HiTech->Request_SVData(6,REG_POSITION);
        }
    }else if (Count1Hz%10 == 3) {
        HiTech->CMD_SET_VELOCITY(1,4095);HiTech->CMD_SET_VELOCITY(2,4095);HiTech->CMD_SET_VELOCITY(3,4095);
        HiTech->CMD_SET_VELOCITY(4,4095);HiTech->CMD_SET_VELOCITY(5,4095);HiTech->CMD_SET_VELOCITY(6,4095);
    }else if (Count1Hz%10 == 4) {
        HiTech->CMD_SET_TORQUE(1,4095);HiTech->CMD_SET_TORQUE(2,4095);HiTech->CMD_SET_TORQUE(3,4095);
        HiTech->CMD_SET_TORQUE(4,4095);HiTech->CMD_SET_TORQUE(5,4095);HiTech->CMD_SET_TORQUE(6,4095);
    }else if (Count1Hz%10 == 5) {
        HiTech->CMD_SET_POSITION(1,cxdata().SV_TX[0].SV_pos);
        HiTech->CMD_SET_POSITION(2,cxdata().SV_TX[1].SV_pos);
        HiTech->CMD_SET_POSITION(3,cxdata().SV_TX[2].SV_pos);
        HiTech->CMD_SET_POSITION(4,cxdata().SV_TX[3].SV_pos);
        HiTech->CMD_SET_POSITION(5,cxdata().SV_TX[4].SV_pos);
        HiTech->CMD_SET_POSITION(6,cxdata().SV_TX[5].SV_pos);
    }*/
#endif

    //cxdata().SVinitialized = 1;
      
    //temp_HiTech_setting = (temp_HiTech_setting + 1) % 100;

    Count1Hz++;

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
