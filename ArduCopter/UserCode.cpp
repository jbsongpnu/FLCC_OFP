#include "Copter.h"

//Pegasus suervo has changed to HiTech servos. 
//Following code contains temporary debugging 
#define COAXSERVO_TEST 0
#define COAXCAN_LOGGING 1
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
    MAV_GCSTX_INV_State.Target_Motor_Acceleration = 0; //100rpm/s
    MAV_GCSTX_INV_State.Motor_Speed_Limit = 0;

    MAV_GCSTX_INV_State.Motor_Speed = 0;
    MAV_GCSTX_INV_State.Target_Motor_Speed = 0;
    MAV_GCSTX_INV_State.i_a = 0;
    MAV_GCSTX_INV_State.i_b = 0;
    MAV_GCSTX_INV_State.i_c = 0;
    
    MAV_GCSTX_INV_State.Motor_Aligned = 0;
    MAV_GCSTX_INV_State.t_a = 0;
    MAV_GCSTX_INV_State.t_b = 0;
    MAV_GCSTX_INV_State.t_c = 0;
    MAV_GCSTX_INV_State.V_dc = 0;
    MAV_GCSTX_INV_State.Fault_Flags = 0;

    MAV_GCSTX_CCB_State.Active_Mode = 0;
    MAV_GCSTX_CCB_State.Motor_MAX = 0;
    MAV_GCSTX_CCB_State.Motor_ON = 0;
    MAV_GCSTX_CCB_State.Brd_temp = 0;
    MAV_GCSTX_CCB_State.Flow_mL = 0;
    MAV_GCSTX_CCB_State.ThCp1x10 = 0;
    MAV_GCSTX_CCB_State.ThCp2x10 = 0;
    MAV_GCSTX_CCB_State.Thermistor1x10 = 0;
    MAV_GCSTX_CCB_State.Thermistor2x10 = 0;
    MAV_GCSTX_CCB_State.Thermistor3x10 = 0;
    MAV_GCSTX_CCB_State.Thermistor4x10 = 0;

    MAV_GCSTX_HBSYS.IFCU_State = 0;
    MAV_GCSTX_HBSYS.PMS_State = 0;
    MAV_GCSTX_HBSYS.HDC_Vout = 0;
    MAV_GCSTX_HBSYS.HDC_Cout = 0;
    MAV_GCSTX_HBSYS.HDC_Vin = 0;
    MAV_GCSTX_HBSYS.HDC_Cin = 0;

    MAV_GCSTX_CXSV_POS.Servo_State = 0;
    MAV_GCSTX_CXSV_POS.SV1_POS_RAW = 0; 
    MAV_GCSTX_CXSV_POS.SV2_POS_RAW = 0;
    MAV_GCSTX_CXSV_POS.SV3_POS_RAW = 0;
    MAV_GCSTX_CXSV_POS.SV4_POS_RAW = 0;
    MAV_GCSTX_CXSV_POS.SV5_POS_RAW = 0;
    MAV_GCSTX_CXSV_POS.SV6_POS_RAW = 0;

    MAV_GCSTX_CXSV_SWASH.Swash_State = 0;
    MAV_GCSTX_CXSV_SWASH.Collective = 0;
    MAV_GCSTX_CXSV_SWASH.Cyclic_Lon = 0;
    MAV_GCSTX_CXSV_SWASH.Cyclic_Lat = 0;
    MAV_GCSTX_CXSV_SWASH.Pedal = 0;
    MAV_GCSTX_CXSV_SWASH.CMD_Collective = 4; 
    MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lon = 0;
    MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lat = 0;
    MAV_GCSTX_CXSV_SWASH.CMD_Pedal = 0;

    MAV_GCSTX_DMI_data.LDC_State = 0;
    MAV_GCSTX_DMI_data.PMS_Mv_Battery_VoltageX10 = 0;
    MAV_GCSTX_DMI_data.PMS_Mv_Output_CurrentX10 = 0;
    MAV_GCSTX_DMI_data.PMS_Batt_Out_CurrentX10 = 0;
    MAV_GCSTX_DMI_data.PMS_LDC_Output_CurrentX10 = 0;
    MAV_GCSTX_DMI_data.PMS_LDC_Output_VoltageX10 = 0;
    MAV_GCSTX_DMI_data.PMS_Output_PowerX10 = 0;
    MAV_GCSTX_DMI_data.PMS_Input_PowerX10 = 0;
    MAV_GCSTX_DMI_data.PMS_MAX_TempX10 = 0;

    MAV_GCSTX_HDM_data.Ifcu_PpCurLimX100 = 0;
    MAV_GCSTX_HDM_data.Ifcu_PpH2SofX2 = 0;
    MAV_GCSTX_HDM_data.Ifcu_H2LkLmp = 0;
    MAV_GCSTX_HDM_data.Ifcu_FcNetVltX10 = 0;
    MAV_GCSTX_HDM_data.Ifcu_FcNetCurx10 = 0;
    MAV_GCSTX_HDM_data.Ifcu_FcInClntTmp = 0;
    MAV_GCSTX_HDM_data.Ifcu_AmbTemp = 0;
    MAV_GCSTX_HDM_data.Ifcu_RoomTemp = 0;
    MAV_GCSTX_HDM_data.Ifcu_H2TnkTmp = 0;
    MAV_GCSTX_HDM_data.Ifcu_H2TnkPrsX10 = 0;

    cxdata().SVinitialized = 0;
    cxdata().CX_State = CoaxState::CXSTATE_0_INIT;
    cxdata().Swash.Col = 4.0;
    cxdata().Swash.Lat = 0.0;
    cxdata().Swash.Lon = 0.0;
    cxdata().Swash.Rud = 0.0;
    cxdata().Swash.Pedal_trim = 0.0;
    cxdata().Swash_CMD.Col = 4.0;   //Start with minimum : Minimum Collective set to 0 deg 2026.01.19
    cxdata().Swash_CMD.Lat = 0.0;
    cxdata().Swash_CMD.Lon = 0.0;
    cxdata().Swash_CMD.Rud = 0.0;
    cxdata().Swash_CMD.Pedal_trim = 0.0;

    cxdata().INV_data.Rdy2useINV = 0;
    cxdata().INV_data.pre_Rdy2useINV = 0;
    cxdata().Command_Received.NewCMD.bits.CCB_Motor_MAX = 0;

    //After using table look-up method for collective, neutral point for servo has not much meaning
    cxdata().SV_TX[0].SV_pos = PARAM_SV1_POS_NEUTRAL;
    cxdata().SV_TX[1].SV_pos = PARAM_SV2_POS_NEUTRAL;
    cxdata().SV_TX[2].SV_pos = PARAM_SV3_POS_NEUTRAL;
    cxdata().SV_TX[3].SV_pos = PARAM_SV4_POS_NEUTRAL;
    cxdata().SV_TX[4].SV_pos = PARAM_SV5_POS_NEUTRAL;
    cxdata().SV_TX[5].SV_pos = PARAM_SV6_POS_NEUTRAL;

    //Software reverse for servo
    cxdata().SV_state[0].SW_Reversed = PARAM_SV1_SW_REVERSE;
    cxdata().SV_state[1].SW_Reversed = PARAM_SV2_SW_REVERSE;
    cxdata().SV_state[2].SW_Reversed = PARAM_SV3_SW_REVERSE;
    cxdata().SV_state[3].SW_Reversed = PARAM_SV4_SW_REVERSE;
    cxdata().SV_state[4].SW_Reversed = PARAM_SV5_SW_REVERSE;
    cxdata().SV_state[5].SW_Reversed = PARAM_SV6_SW_REVERSE;

    //Zero degree collective pitch position
    cxdata().SV_state[0].Position_Zero = PARAM_SV1_POS_ZERO;
    cxdata().SV_state[1].Position_Zero = PARAM_SV2_POS_ZERO;
    cxdata().SV_state[2].Position_Zero = PARAM_SV3_POS_ZERO;
    cxdata().SV_state[3].Position_Zero = PARAM_SV4_POS_ZERO;
    cxdata().SV_state[4].Position_Zero = PARAM_SV5_POS_ZERO;
    cxdata().SV_state[5].Position_Zero = PARAM_SV6_POS_ZERO;

    cxdata().SVTestState.ServoTestingID = 0;//Initiate with ID 0
}
#endif

#ifdef USERHOOK_FASTLOOP
//100Hz
void Copter::userhook_FastLoop()
{
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP

#define DEF_CYC_OPTION_RC_CH    6
void Copter::userhook_MediumLoop()
{
    //User Code for Coaxial Helicopter 
    //Loop rate : 10Hz

    //static variables
    static uint16_t Count1Hz = 0;

    // Read RC channel for cyclic option
    RC_Channel *cyc_opt_ch = RC_Channels::rc_channel(DEF_CYC_OPTION_RC_CH - 1);
    //if (!rc().has_had_rc_receiver() && !rc().has_had_rc_override()) 
    if ((cyc_opt_ch != nullptr) && rc().has_had_rc_receiver()) {
        int16_t rc_in  = cyc_opt_ch->get_radio_in();
        int16_t rc_min = cyc_opt_ch->get_radio_min();
        int16_t rc_max = cyc_opt_ch->get_radio_max();
        int16_t rc_trim = cyc_opt_ch->get_radio_trim();
        int16_t thresh_low  = (rc_min + rc_trim) / 2;
        int16_t thresh_high = (rc_trim + rc_max) / 2;
        if (rc_in < thresh_low) {
            if(cxdata().ctrl.cyc_option != 1) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Lower-Rotor Cyclic Mode");
                cxdata().ctrl.cyc_option = 1;       // near minimum: lower-only
            }
        } else if (rc_in < thresh_high) {
            if(cxdata().ctrl.cyc_option != 3) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Both-Rotor Cyclic Mode");
                cxdata().ctrl.cyc_option = 3;       // near trim: both
            }
        } else {
            if(cxdata().ctrl.cyc_option != 2) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Upper-Rotor Cyclic Mode");
                cxdata().ctrl.cyc_option = 2;       // near maximum: upper-only
            }
        }
    }

    // 5Hz GCS data
    if (Count1Hz%2 == 0) {
        MAV_GCSTX_CXSV_SWASH.Swash_State = static_cast<uint8_t>(cxdata().CX_State);
        MAV_GCSTX_CXSV_SWASH.Collective = cxdata().Swash.Col;
        MAV_GCSTX_CXSV_SWASH.Cyclic_Lon = cxdata().Swash.Lon;
        MAV_GCSTX_CXSV_SWASH.Cyclic_Lat = cxdata().Swash.Lat;
        MAV_GCSTX_CXSV_SWASH.Pedal      = cxdata().Swash.Rud;   //Rud is added with pedal trim within servo control code
        MAV_GCSTX_CXSV_SWASH.CMD_Collective = cxdata().Swash_CMD.Col;
        MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lon = cxdata().Swash_CMD.Lon;
        MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lat = cxdata().Swash_CMD.Lat;
        MAV_GCSTX_CXSV_SWASH.CMD_Pedal      = cxdata().Swash_CMD.Rud;
        gcs().send_message(MSG_CXSV_SWASH);
    }
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
        MAV_GCSTX_INV_State.Motor_Aligned = (uint16_t)cxdata().INV_data.Motor_Align_flag;//Thetaoffset has changed to Align_flag
        MAV_GCSTX_INV_State.i_a = (uint16_t)(cxdata().INV_data.i_a * 100.0);
        MAV_GCSTX_INV_State.i_b = (uint16_t)(cxdata().INV_data.i_b * 100.0);
        MAV_GCSTX_INV_State.i_c = (uint16_t)(cxdata().INV_data.i_c * 100.0);
        MAV_GCSTX_INV_State.t_a = (uint16_t)(cxdata().INV_data.t_a * 100.0);
        MAV_GCSTX_INV_State.t_b = (uint16_t)(cxdata().INV_data.t_b * 100.0);
        MAV_GCSTX_INV_State.t_c = (uint16_t)(cxdata().INV_data.t_c * 100.0);
        MAV_GCSTX_INV_State.V_dc = (uint16_t)(cxdata().INV_data.V_dc_input * 10.0);
        MAV_GCSTX_INV_State.Fault_Flags = cxdata().INV_data.FLT.ALL;
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
        // //=== If sending feedback position
        // MAV_GCSTX_CXSV_POS.SV1_POS_RAW = cxdata().SV_Pos[0].raw;
        // MAV_GCSTX_CXSV_POS.SV2_POS_RAW = cxdata().SV_Pos[1].raw;
        // MAV_GCSTX_CXSV_POS.SV3_POS_RAW = cxdata().SV_Pos[2].raw;
        // MAV_GCSTX_CXSV_POS.SV4_POS_RAW = cxdata().SV_Pos[3].raw;
        // MAV_GCSTX_CXSV_POS.SV5_POS_RAW = cxdata().SV_Pos[4].raw;
        // MAV_GCSTX_CXSV_POS.SV6_POS_RAW = cxdata().SV_Pos[5].raw;
        // === If sending commanded position
        MAV_GCSTX_CXSV_POS.SV1_POS_RAW = cxdata().SV_TX[0].SV_pos;
        MAV_GCSTX_CXSV_POS.SV2_POS_RAW = cxdata().SV_TX[1].SV_pos;
        MAV_GCSTX_CXSV_POS.SV3_POS_RAW = cxdata().SV_TX[2].SV_pos;
        MAV_GCSTX_CXSV_POS.SV4_POS_RAW = cxdata().SV_TX[3].SV_pos;
        MAV_GCSTX_CXSV_POS.SV5_POS_RAW = cxdata().SV_TX[4].SV_pos;
        MAV_GCSTX_CXSV_POS.SV6_POS_RAW = cxdata().SV_TX[5].SV_pos;
        // //=== If comparing Upper rotor TX and RX
        // MAV_GCSTX_CXSV_POS.SV1_POS_RAW = cxdata().SV_TX[0].SV_pos;
        // MAV_GCSTX_CXSV_POS.SV2_POS_RAW = cxdata().SV_TX[1].SV_pos;
        // MAV_GCSTX_CXSV_POS.SV3_POS_RAW = cxdata().SV_TX[2].SV_pos;
        // MAV_GCSTX_CXSV_POS.SV4_POS_RAW = cxdata().SV_Pos[0].raw;
        // MAV_GCSTX_CXSV_POS.SV5_POS_RAW = cxdata().SV_Pos[1].raw;
        // MAV_GCSTX_CXSV_POS.SV6_POS_RAW = cxdata().SV_Pos[2].raw;
        // //=== If comparing Lower rotor TX and RX
        // MAV_GCSTX_CXSV_POS.SV1_POS_RAW = cxdata().SV_TX[3].SV_pos;
        // MAV_GCSTX_CXSV_POS.SV2_POS_RAW = cxdata().SV_TX[4].SV_pos;
        // MAV_GCSTX_CXSV_POS.SV3_POS_RAW = cxdata().SV_TX[5].SV_pos;
        // MAV_GCSTX_CXSV_POS.SV4_POS_RAW = cxdata().SV_Pos[3].raw;
        // MAV_GCSTX_CXSV_POS.SV5_POS_RAW = cxdata().SV_Pos[4].raw;
        // MAV_GCSTX_CXSV_POS.SV6_POS_RAW = cxdata().SV_Pos[5].raw;
        gcs().send_message(MSG_CXSV_POS);
    // } else if (Count1Hz%10 == 5) {
    //     MAV_GCSTX_CXSV_SWASH.Swash_State = static_cast<uint8_t>(cxdata().CX_State);
    //     MAV_GCSTX_CXSV_SWASH.Collective = cxdata().Swash.Col;
    //     MAV_GCSTX_CXSV_SWASH.Cyclic_Lon = cxdata().Swash.Lon;
    //     MAV_GCSTX_CXSV_SWASH.Cyclic_Lat = cxdata().Swash.Lat;
    //     MAV_GCSTX_CXSV_SWASH.Pedal      = cxdata().Swash.Rud;   //Rud is added with pedal trim within servo control code
    //     MAV_GCSTX_CXSV_SWASH.CMD_Collective = cxdata().Swash_CMD.Col;
    //     MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lon = cxdata().Swash_CMD.Lon;
    //     MAV_GCSTX_CXSV_SWASH.CMD_Cyclic_Lat = cxdata().Swash_CMD.Lat;
    //     MAV_GCSTX_CXSV_SWASH.CMD_Pedal      = cxdata().Swash_CMD.Rud;
    //     gcs().send_message(MSG_CXSV_SWASH);
    } else if (Count1Hz%10 == 6) {
        MAV_GCSTX_DMI_data.LDC_State = cxdata().DMI_PMS_data.LDC_State;
        MAV_GCSTX_DMI_data.PMS_Mv_Battery_VoltageX10 = (uint16_t)(cxdata().DMI_PMS_data.Mv_Battery_Voltage * 10.0);
        MAV_GCSTX_DMI_data.PMS_Mv_Output_CurrentX10 = (uint16_t)(cxdata().DMI_PMS_data.Mv_Output_Current * 10.0);
        MAV_GCSTX_DMI_data.PMS_Batt_Out_CurrentX10 = (uint16_t)(cxdata().DMI_PMS_data.Batt_Output_Current * 10.0);
        MAV_GCSTX_DMI_data.PMS_LDC_Output_CurrentX10 = (uint16_t)(cxdata().DMI_PMS_data.LDC_Output_Current * 10.0);
        MAV_GCSTX_DMI_data.PMS_LDC_Output_VoltageX10 = (uint16_t)(cxdata().DMI_PMS_data.PMS_LDC_Out_Volt * 10.0);
        MAV_GCSTX_DMI_data.PMS_Output_PowerX10 = (uint16_t)(cxdata().DMI_PMS_data.PMS_Out_Power * 10.0);
        MAV_GCSTX_DMI_data.PMS_Input_PowerX10 = (uint16_t)(cxdata().DMI_PMS_data.PMS_In_Power * 10.0);
        MAV_GCSTX_DMI_data.PMS_MAX_TempX10 = (int16_t)(cxdata().DMI_PMS_data.PMS_Max_Temp * 10.0);

        gcs().send_message(MSG_DMI_DATA);
    // } else if (Count1Hz%10 == 7) {
    //    gcs().send_text(MAV_SEVERITY_INFO, "DEBUG : ped %1.1f ", cxdata().Swash.Pedal_trim);
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
        MAV_GCSTX_HDM_data.Ifcu_H2TnkPrsX10 = (uint16_t)(cxdata().IFCU_data.H2TnkPrs * 10.0f);
        gcs().send_message(MSG_HDM_DATA);
#if CCB_AUTOSEQUENCE == 1
        if (Count1Hz > 50) {
            cxdata().Command_Received.NewCMD.bits.CCB_Motor_MAX = 1;
        }
#endif
    }

    if((Count1Hz == 158) && (cxdata().SVTestState.ServoTestingID == 0)) { 
        cxdata().SVTestState.ServoTestingID = 1;

    }

    Count1Hz++;

//----For 10Hz Logging
#if COAXCAN_LOGGING == 1
    AP::logger().Write("CXCT", "TimeUS,RIN,PIN,TIN,YIN,ZSAS,PTS,DEBUG", "QffffffB",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().ctrl.roll_in,
        cxdata().ctrl.pitch_in,
        cxdata().ctrl.throttle_in,
        cxdata().ctrl.yaw_in,
        cxdata().ctrl.SAS,
        cxdata().ctrl.Pilot_th_scaled,
        cxdata().ctrl.debug
    );

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

    AP::logger().Write("CCB", "TimeUS,FLOW,TI1,TI2,TI3,TI4,TC1,TC2,BDTEMP", "QHHHHHHHB",
        AP_HAL::micros64(),                         //Q     TimeUS
        cxdata().CCB_data.Flow_mL,                  //H     FLOW
        cxdata().CCB_data.Thermistor1x10,           //H     TI1
        cxdata().CCB_data.Thermistor2x10,           //H     TI2
        cxdata().CCB_data.Thermistor3x10,           //H     TI3
        cxdata().CCB_data.Thermistor4x10,           //H     TI4
        cxdata().CCB_data.ThCp1x10,                 //H     TC1
        cxdata().CCB_data.ThCp2x10,                 //H     TC2
        cxdata().CCB_data.Brd_temp                  //B     BDTEMP
    );

    //Hybrid system 1
    AP::logger().Write("HBS1", "TimeUS,PSTAT,BOC,LOC,LOV,POV,POC,PIV,PIC", "QBfffffff",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().DMI_PMS_data.PMS_State,                //B     PSTAT
        cxdata().DMI_PMS_data.Batt_Output_Current,      //f     BOC
        cxdata().DMI_PMS_data.LDC_Output_Current,       //f     LOC
        cxdata().DMI_PMS_data.PMS_LDC_Out_Volt,         //f     LOV
        cxdata().DMI_PMS_data.HDC_OutputVoltage,        //f     POV
        cxdata().DMI_PMS_data.HDC_OutputCurrent,        //f     POC
        cxdata().DMI_PMS_data.HDC_InputVoltage,         //f     PIV
        cxdata().DMI_PMS_data.HDC_InputCurrent          //f     PIC
    );
    //Hybrid system 2
    AP::logger().Write("HBS2", "TimeUS,LSTAT,MOC,MBV,POP,PIP,PMT", "QBfffff",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().DMI_PMS_data.LDC_State,                //B     LSTAT
        cxdata().DMI_PMS_data.Mv_Output_Current,        //f     MOC
        cxdata().DMI_PMS_data.Mv_Battery_Voltage,       //f     MBV
        cxdata().DMI_PMS_data.PMS_Out_Power,            //f     POP
        cxdata().DMI_PMS_data.PMS_In_Power,             //f     PIP
        cxdata().DMI_PMS_data.PMS_Max_Temp              //f     PMT
    );
    //Hybrid system 3
    AP::logger().Write("HBS3", "TimeUS,PPVLT,PPCUR,CURLIM,H2SOF", "Qffff",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().IFCU_data.PpVlt,                       //f     PPVLT
        cxdata().IFCU_data.PpCur,                       //f     PPCUR
        cxdata().IFCU_data.PpCurLim,                    //f     CURLIM
        cxdata().IFCU_data.PpH2Sof                      //f     H2SOF
    );
    //Hybrid system 4
    AP::logger().Write("HBS4", "TimeUS,HSTAT,HFLT,DTC,H2LK,FCNV,FCNC", "QBBBBff",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().IFCU_data.State,                       //B     HSTAT    
        cxdata().IFCU_data.FltSts,                      //B     HFLT
        cxdata().IFCU_data.DTC,                         //B     DTC
        cxdata().IFCU_data.H2LkLmp,                     //B     H2LK
        cxdata().IFCU_data.FcNetVlt,                    //f     FCNV     
        cxdata().IFCU_data.FcNetCur                     //f     FCNC
    );
    //Hybrid system 5
    AP::logger().Write("HBS5", "TimeUS,CINT,AMBT,ROMT,HTP,HTT,HTFC,MAXC,NCCL,MIDP", "QbbbfhHffH",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().IFCU_data.FcInClntTmp,                 //b     CINT
        cxdata().IFCU_data.AmbTemp,                     //b     AMBT
        cxdata().IFCU_data.RoomTemp,                    //b     ROMT
        cxdata().IFCU_data.H2TnkPrs,                    //f     HTP
        cxdata().IFCU_data.H2TnkTmp,                    //h     HTT
        cxdata().IFCU_data.H2TnkFillCnt,                //H     HTFC
        cxdata().IFCU_data.FcMxCurLim,                  //f     MAXC
        cxdata().IFCU_data.FcNetCustCurLim,             //f     NCCL
        cxdata().IFCU_data.H2MidPrs                     //H     MIDP
    );

    //FDC-1 for DMI
    AP::logger().Write("FDC1", "TimeUS,STAT,AUXV,MXT,FLG1,FLG2", "QBfhBB",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().DMI_PMS_data.FDC_State,
        cxdata().DMI_PMS_data.FDC_Aux_Volt,
        cxdata().DMI_PMS_data.FDC_Max_Temp,
        cxdata().DMI_PMS_data.FDC_Flag1.ALL,
        cxdata().DMI_PMS_data.FDC_Flag2.ALL
    );

    //FDC-2 for DMI
    AP::logger().Write("FDC2", "TimeUS,FOV,FOC,FIV,FIC", "Qffff",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().DMI_PMS_data.FDC_OutputVoltage,
        cxdata().DMI_PMS_data.FDC_OutputCurrent,
        cxdata().DMI_PMS_data.FDC_InputVoltage,
        cxdata().DMI_PMS_data.FDC_InputCurrent
    );

    AP::logger().Write("CSV1", "TimeUS,SVSTAT,TXCOL,TXLAT,TXLON,TXRUD,COL,LAT,LON,RUD", "QBffffffff", 
        AP_HAL::micros64(),                             //Q     TimeUS
        ((uint8_t)cxdata().CX_State),                   //B     Coax Servo State
        cxdata().Swash_CMD.Col,                         //f     collective command
        cxdata().Swash_CMD.Lat,                         //f     lateral cyclic command
        cxdata().Swash_CMD.Lon,                         //f     longitudinal cyclic command
        cxdata().Swash_CMD.Rud,                         //f     command pedal = rudder
        cxdata().Swash.Col,                             //f     current collective 
        cxdata().Swash.Lat,                             //f     current lateral cyclic
        cxdata().Swash.Lon,                             //f     current longitudinal cyclic
        cxdata().Swash.Rud                             //f     current pedal = rudder
    );

    AP::logger().Write("CSV2", "TimeUS,TXSV1,TXSV2,TXSV3,TXSV4,TXSV5,TXSV6", "Qhhhhhh",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().SV_TX[0].SV_pos,                       //h     TX_SV1
        cxdata().SV_TX[1].SV_pos,                       //h     TX_SV2
        cxdata().SV_TX[2].SV_pos,                       //h     TX_SV3
        cxdata().SV_TX[3].SV_pos,                       //h     TX_SV4
        cxdata().SV_TX[4].SV_pos,                       //h     TX_SV5
        cxdata().SV_TX[5].SV_pos                       //h     TX_SV6
    );

    AP::logger().Write("CSV3", "TimeUS,RXSV1,RXSV2,RXSV3,RXSV4,RXSV5,RXSV6", "Qhhhhhh",
        AP_HAL::micros64(),                             //Q     TimeUS
        cxdata().SV_Pos[0].raw,                         //h     TX_SV1
        cxdata().SV_Pos[1].raw,                         //h     TX_SV2
        cxdata().SV_Pos[2].raw,                         //h     TX_SV3
        cxdata().SV_Pos[3].raw,                         //h     TX_SV4
        cxdata().SV_Pos[4].raw,                         //h     TX_SV5
        cxdata().SV_Pos[5].raw                          //h     TX_SV6
    );
    //Addiitional logging items
    AP::logger().Write("CSV4", "TimeUS,RXTQ1,RXTQ2,RXTQ3,RXTQ4,RXTQ5,RXTQ6", "Qhhhhhh",
    AP_HAL::micros64(),                             //Q     TimeUS
    cxdata().SV_state[0].Status_Torque,             //h     RX_TorQue1
    cxdata().SV_state[1].Status_Torque,             //h     RX_TorQue2
    cxdata().SV_state[2].Status_Torque,             //h     RX_TorQue3
    cxdata().SV_state[3].Status_Torque,             //h     RX_TorQue4
    cxdata().SV_state[4].Status_Torque,             //h     RX_TorQue5
    cxdata().SV_state[5].Status_Torque              //h     RX_TorQue6
    );
    AP::logger().Write("CSV5", "TimeUS,CSV1,CSV2,CSV3,CSV4,CSV5,CSV6", "Qffffff",
    AP_HAL::micros64(),
    cxdata().SV_Pos[0].CtrlOut,
    cxdata().SV_Pos[1].CtrlOut,
    cxdata().SV_Pos[2].CtrlOut,
    cxdata().SV_Pos[3].CtrlOut,
    cxdata().SV_Pos[4].CtrlOut,
    cxdata().SV_Pos[5].CtrlOut
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
