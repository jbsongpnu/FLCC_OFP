#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_CoaxCAN2.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include "Coaxial_data.h"

//Debug Control
#define DEBUG_IFCU  0   //IFCU Test
#define DEBUG_PMS   0   //PMS Test

extern const AP_HAL::HAL& hal;

#define TEMP_EXP 0		//Initial value

// Table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_COAXCAN2::var_info[] = {
    // @Param: example
    // @DisplayName: User example
    // @Description: Example for user
    AP_GROUPINFO("Ex", 1, AP_COAXCAN2, _examp, TEMP_EXP),
    //AP_GROUPINFO("Examp", 1, AP_COAXCAN, TEMP_EXP, TEMP_EXP),
    // AP_GROUPINFO("PARAM1", 1, AP_COAXCAN, _pmu_param1, TEMP_EXP),
    // AP_GROUPINFO("PARAM2", 2, AP_COAXCAN, _pmu_param2, TEMP_EXP),
    // AP_GROUPINFO("PARAM3", 3, AP_COAXCAN, _pmu_param3, TEMP_EXP),
    // AP_GROUPINFO("PARAM4", 4, AP_COAXCAN, _pmu_param4, TEMP_EXP),

    AP_GROUPEND
};

mavlink_sys_icd_flcc_gcs_hbsys_t        MAV_GCSTX_HBSYS = {0};
mavlink_sys_icd_flcc_gcs_dmi_data_t     MAV_GCSTX_DMI_data = {0};
mavlink_sys_icd_flcc_gcs_hdm_data_t     MAV_GCSTX_HDM_data = {0};

AP_COAXCAN2::AP_COAXCAN2()
{
    AP_Param::setup_object_defaults(this, var_info);
    
    _examp.set_default(0);

    _initialized    = false;
    _iface                  = nullptr;

    _FCC_AlivCnt = 0;
    _FCC_CmdFcRunStop = 0;
    _FCC_CmdPmsBatCut = 0;
    _FCC_Ready = 0;
    _FCC_Reserved1 = 0;
    _FCC_FcPwrReq = 0;
    _FCC_FcThrottle = 0;
    _FCC_FcThrottlePrdct = 0;

    _PMS1.AlivCnt = 0;
    _PMS1.LDC_State = 0;
    _PMS1.LDC_State = 0;
    _PMS1.Fault_LDC_No = 0;
    _PMS1.Batt_SW_On = 0;
    _PMS1.Mv_SW_On = 0;
    _PMS1.Lv_SW_On = 0;
    _PMS1.Batt_Charger_On = 0;

    _PMS2.Batt_Output_Current_raw = 0;
    _PMS2.LDC_Output_Current_raw = 0;
    _PMS2.Mv_Output_Current_raw = 0;
    _PMS2.Mv_Battery_Voltage_raw = 0;

    _PMS3.OutputVoltage_raw = 0;
    _PMS3.OutputVoltage_raw = 0;
    _PMS3.OutputCurrent_raw = 0;
    _PMS3.InputVoltage_raw = 0;
    _PMS3.InputCurrent_raw = 0;

    _FDC1.AliveCnt = 0;
    _FDC1.State = 0;
    _FDC1.Aux_Volt_raw = 0;
    _FDC1.Max_Temp = 0;
    _FDC1.Flag1.ALL = 0;
    _FDC1.Flag2.ALL = 0;

    _FDC2.OutputVoltage_raw = 0;
    _FDC2.OutputCurrent_raw = 0;
    _FDC2.InputVoltage_raw = 0;
    _FDC2.InputCurrent_raw = 0;

    _VCUFDC1.AliveCnt = 0;
    _VCUFDC1.SET_CMD = 0;
    _VCUFDC1.Fault_Reset = 0;
    _VCUFDC1.Target_OutputVoltage_raw = 0;
    _VCUFDC1.Target_InputCurrent_raw = 0;
    _VCUFDC1.Target_InputPower_raw = 0;

    _IFCU1.PpCur_raw = 0;
    _IFCU1.PpCurLim_raw = 0;
    _IFCU1.PpH2Sof_raw = 0;
    _IFCU1.PpVlt_raw = 0;
    _IFCU1.reserved = 0;

    _IFCU2.DTC = 0;
    _IFCU2.FltSts = 0;
    _IFCU2.H2LkLmp = 0;
    _IFCU2.State = 0;
    _IFCU2.SvmlsoRVlu = 0;

    _IFCU3.FcNetCur_raw = 0;
    _IFCU3.FcNetVlt_raw = 0;
    _IFCU3.LdcCur_raw = 0;
    _IFCU3.LdcVlt_raw = 0;

    _IFCU4.AmbTemp = 0;
    _IFCU4.FcInClntTmp = 0;
    _IFCU4.H2TnkFillCnt = 0;
    _IFCU4.H2TnkTmp = 0;
    _IFCU4.H2TnkPrs_raw = 0;
    _IFCU4.RoomTemp = 0;

    _IFCU5.ExWtrTrpFill = 0;
    _IFCU5.HvBsaCur_raw = 0;
    _IFCU5.HvBsaSoC_raw = 0;
    _IFCU5.HvBsaSoH_raw = 0;
    _IFCU5.HvBsaVlt_raw = 0;

    _IFCU6.FcClntFiltChk = 0;
    _IFCU6.FcClntSplChk = 0;
    _IFCU6.FcMxCurLim_raw = 0;
    _IFCU6.FcNetCustCurLim_raw = 0;
    _IFCU6.H2MidPrs_raw = 0;
    
    coaxcan2_period_us = 1000000UL / COAXCAN2_LOOP_HZ;

    _cmd_id[TX_ID::TX_ID_FCC1]  = ID_FCC1;
    _cmd_id[TX_ID::TX_ID_FCC2]  = ID_FCC2;
    _cmd_id[TX_ID::TX_ID_FCC3]  = ID_FCC3;
    
}

AP_COAXCAN2::~AP_COAXCAN2()
{    
}

AP_COAXCAN2 *AP_COAXCAN2::get_coaxcan2(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_CoaxCAN2) {
        return nullptr;
    }
    return static_cast<AP_COAXCAN2*>(AP::can().get_driver(driver_index));
}

// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
bool AP_COAXCAN2::add_interface(AP_HAL::CANIface* can_iface) {

    if (_can_iface != nullptr) {
    	hal.console->printf("COAXCAN2: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
    	hal.console->printf("COAXCAN2: CAN driver not found\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN2: CAN driver not found");
        return false;
    }

    if (!_can_iface->is_initialized()) {
    	hal.console->printf("COAXCAN2: Driver not initialized\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN2: Driver not initialized");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
    	hal.console->printf("COAXCAN2: Cannot add event handle\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN2: Cannot add event handle");
        return false;
    }
    return true;
}

// -------------------------------------------------------------------------
// Initialize COAXCAN bus
// -------------------------------------------------------------------------
void AP_COAXCAN2::init(uint8_t driver_index, bool enable_filters)
{
	_driver_index = driver_index;

    if (_initialized) {
        return;
    }

    if (_can_iface == nullptr) {
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_COAXCAN2::loop, void), _thread_name, 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;


    //gcs().send_text(MAV_SEVERITY_INFO, "[COAX] Initialized");
}

// -------------------------------------------------------------------------
// Task
// -------------------------------------------------------------------------
void AP_COAXCAN2::loop(void)
{
    while (true)
    {
        if (!_initialized)
        {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        hal.scheduler->delay_microseconds(coaxcan2_period_us);    // 5ms period loop

        run();
                    
        _AP_COAXCAN2_loop_cnt++;                                  // 5ms period increase
    }
}

// -------------------------------------------------------------------------
// Run : 200Hz
// [_AP_COAXCAN2_loop_cnt] is increased from loop() at every 5ms(200Hz)
// -------------------------------------------------------------------------
void AP_COAXCAN2::run(void)
{
    //Receive
	RXspin();

    //Check data
    if(_AP_COAXCAN2_loop_cnt%10==0) {
        Check_ALL_data();
    }

    if(_AP_COAXCAN2_loop_cnt%20==0)
    // if(_AP_COAXCAN2_loop_cnt%100==0)
    {
        TX_FCC1_MSG();
    }
    else if(_AP_COAXCAN2_loop_cnt%20 == 5)
    // // else if(_AP_COAXCAN2_loop_cnt%100 == 50)
    {
        TX_FCC2_MSG();
    }
    // else
    // {
    //     TXspin();//Transmit unscheduled messages : command from GCS
    // }
    //

    //TXspin();
}

// -------------------------------------------------------------------------
// RX 
// -------------------------------------------------------------------------
void AP_COAXCAN2::RXspin()
{
    uint64_t time, timeout;
    int res = 0;

    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), COAXCAN2_SEND_TIMEOUT_US);
    AP_HAL::CANIface::CanIOFlags flags = 0;
    AP_HAL::CANFrame frame;                     // receive frame

    // wait for space in buffer to read
    bool read_select    = true; 
    bool write_select   = false;    //Read-only
    timeout = timeout_us + AP_HAL::micros64();

    //'Root/AP_HAL_ChibiOS/CanIface.cpp',
    // bool CANIface::select(bool &read, bool &write, const AP_HAL::CANFrame* pending_tx, uint64_t blocking_deadline)
    int ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret) {
        //gcs().send_text(MAV_SEVERITY_INFO, "COAXCAN2 COMMUNICATION_ERROR");
        // return if no data is available to read
        // if(COAXCAN2_Fail_Status==COAXCAN2_STATUS::CONNECTION_FAILURE)
        // {
        //     COAXCAN2_Fail_Status = COAXCAN2_STATUS::CONNECTION_FAILURE;
        //     gcs().send_text(MAV_SEVERITY_INFO, "COAXCAN2 CONNECTION_FAILURE");
        // }
        // else
        // {
        //     COAXCAN2_Fail_Status = COAXCAN2_STATUS::COMMUNICATION_ERROR;
        //     gcs().send_text(MAV_SEVERITY_INFO, "COAXCAN2 COMMUNICATION_ERROR");
        // }

        return;
    }


    if(COAXCAN2_Fail_Status == COAXCAN2_STATUS::CONNECTED)     // Normal Connection 
    {
        res = _can_iface->receive(frame, time, flags);

        if(res > 0) // Data Received Normaly
        {
            if(COAXCAN2_ErrCnt > 0) // Check Error Count
            {
                COAXCAN2_ErrCnt = COAXCAN2_ErrCnt - 1;    // Decrease Error Count
            }

            while(res > 0)
            {
                handleFrame(frame);
                res = _can_iface->receive(frame, time, flags);     // Try Receive
            }
        }
        else        // Data Not Received
        {
            COAXCAN2_ErrCnt = COAXCAN2_ErrCnt + 1;          // Increase Error Count

            if(COAXCAN2_ErrCnt == 10) // Check Max Err Count
            {
                COAXCAN2_Fail_Status  = COAXCAN2_STATUS::COMMUNICATION_ERROR; // Set Communication Error Flag 
                gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN2 COMMUNICATION_ERROR ErrCnt10");
                COAXCAN2_ErrCnt       = 0; // Reset Error Count
            }
        }
    }
    else                            // Abnormal Connection 
    {
        res = _can_iface->receive(frame, time, flags);

        if(res > 0) // Data Received Normaly
        {
            COAXCAN2_RcvrCnt = COAXCAN2_RcvrCnt + 1;    // Increase Receive Count

            if(COAXCAN2_RcvrCnt == 5) // Check Max Recv Count
            {
                COAXCAN2_Fail_Status  = COAXCAN2_STATUS::CONNECTED; // Clear Communication Error Flag 
                COAXCAN2_RcvrCnt      = 0; // Reset Receive Count
                gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN2 Connected Err %d", COAXCAN2_ErrCnt);
            }

            while(res > 0)
            {
                handleFrame(frame);
                res = _can_iface->receive(frame, time, flags);     // Receive again
            }

        }
        else        // Data Not Received
        {
            if((COAXCAN2_RcvrCnt > 0) & (res < 0)) // Check Receive Count
            {
                COAXCAN2_RcvrCnt = COAXCAN2_RcvrCnt - 1;    // Decrease Receive Count
            }
        }

    }

}

// -------------------------------------------------------------------------
// Handle Frame : process each CAN frame
// -------------------------------------------------------------------------
void AP_COAXCAN2::handleFrame(const AP_HAL::CANFrame& can_rxframe)
{
    uint16_t    uint16_temp1 = 0;
    uint16_t    uint16_temp2 = 0;
    uint8_t     uint8_temp = 0;

    switch(can_rxframe.id&can_rxframe.MaskStdID)
    {
        case RX_ID_IFCU1 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _IFCU1.PpVlt_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _IFCU1.PpCur_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _IFCU1.PpCurLim_raw = uint16_temp2 * 256 + uint16_temp1;
            _IFCU1.PpH2Sof_raw = can_rxframe.data[6];
            // _IFCU1.reserved = can_rxframe.data[7];

            _NewIFCU_msg = _NewIFCU_msg | 0x01;//bit0 : IFCU1
            #if DEBUG_IFCU == 1
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU1 : %u, %u, %u, %u", _IFCU1.PpVlt_raw, _IFCU1.PpCur_raw, _IFCU1.PpCurLim_raw, _IFCU1.PpH2Sof_raw);
            #endif
            break;
        case RX_ID_IFCU2 :
            _IFCU2.State = can_rxframe.data[0];
            _IFCU2.FltSts = can_rxframe.data[1]; 
            _IFCU2.DTC = can_rxframe.data[2];
            _IFCU2.H2LkLmp = can_rxframe.data[3];
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _IFCU2.SvmlsoRVlu = uint16_temp2 * 256 + uint16_temp1;

            _NewIFCU_msg = _NewIFCU_msg | 0x02;//bit1 : IFCU2
            #if DEBUG_IFCU == 1
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU2 : %u, %u, %u, %u, %u", _IFCU2.State, _IFCU2.FltSts, _IFCU2.DTC, _IFCU2.H2LkLmp, _IFCU2.SvmlsoRVlu);
            #endif
            break;
        case RX_ID_IFCU3 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _IFCU3.FcNetVlt_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _IFCU3.FcNetCur_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _IFCU3.LdcVlt_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[6];
            uint16_temp2 = can_rxframe.data[7];
            _IFCU3.LdcCur_raw = uint16_temp2 * 256 + uint16_temp1;

            _NewIFCU_msg = _NewIFCU_msg | 0x04;//bit2 : IFCU3
            #if DEBUG_IFCU == 1
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU3 : %u, %u, %u, %u", _IFCU3.FcNetVlt_raw, _IFCU3.FcNetCur_raw, _IFCU3.LdcVlt_raw, _IFCU3.LdcCur_raw);
            #endif
            break;
        case RX_ID_IFCU4 :
            _IFCU4.FcInClntTmp = (int8_t)can_rxframe.data[0];
            _IFCU4.AmbTemp = (int8_t)can_rxframe.data[1];
            _IFCU4.RoomTemp = (int8_t)can_rxframe.data[2];
            _IFCU4.H2TnkPrs_raw = can_rxframe.data[3];
            _IFCU4.H2TnkTmp = can_rxframe.data[4];
            uint16_temp1 = can_rxframe.data[5];
            uint16_temp2 = can_rxframe.data[6];
            _IFCU4.H2TnkFillCnt = uint16_temp2 * 256 + uint16_temp1;

            _NewIFCU_msg = _NewIFCU_msg | 0x08;//bit3 : IFCU4
            #if DEBUG_IFCU == 1
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU4 : %d, %d, %d, %u, %u, %u", _IFCU4.FcInClntTmp, _IFCU4.AmbTemp, _IFCU4.RoomTemp, _IFCU4.H2TnkPrs_raw, _IFCU4.H2TnkTmp, _IFCU4.H2TnkFillCnt);
            #endif
            break;
        case RX_ID_IFCU5 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _IFCU5.HvBsaVlt_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _IFCU5.HvBsaCur_raw = uint16_temp2 * 256 + uint16_temp1;
            _IFCU5.HvBsaSoC_raw = can_rxframe.data[4];
            _IFCU5.HvBsaSoH_raw = can_rxframe.data[5];
            _IFCU5.ExWtrTrpFill = can_rxframe.data[6] & 0x01;

            _NewIFCU_msg = _NewIFCU_msg | 0x10;//bit4 : IFCU5
            #if DEBUG_IFCU == 1
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU5 : %u, %u, %u, %u, %u", _IFCU5.HvBsaVlt_raw, _IFCU5.HvBsaCur_raw, _IFCU5.HvBsaSoC_raw, _IFCU5.HvBsaSoH_raw, _IFCU5.ExWtrTrpFill);
            #endif
            break;
        case RX_ID_IFCU6 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _IFCU6.FcMxCurLim_raw= uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _IFCU6.FcNetCustCurLim_raw = uint16_temp2 * 256 + uint16_temp1;
            _IFCU6.H2MidPrs_raw = can_rxframe.data[4];
            uint8_temp = can_rxframe.data[5];
            _IFCU6.FcClntFiltChk = uint8_temp & 0x01;
            _IFCU6.FcClntSplChk = (uint8_temp >> 1) & 0x01;

            _NewIFCU_msg = _NewIFCU_msg | 0x20;//bit5 : IFCU6
            #if DEBUG_IFCU == 1
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU6 : %u, %u, %u, %u, %u", _IFCU6.FcMxCurLim_raw, _IFCU6.FcNetCustCurLim_raw, _IFCU6.H2MidPrs_raw, _IFCU6.FcClntFiltChk, _IFCU6.FcClntSplChk);
            #endif
            break;
        case RX_ID_PMS1 :
            _PMS1.AlivCnt = can_rxframe.data[0] & 0x0F;
            _PMS1.State = ((can_rxframe.data[0] >> 4) & 0x0F)
                            + (can_rxframe.data[1] & 0x0F);
            _PMS1.LDC_State = ((can_rxframe.data[1] >> 4) & 0x0F)
                            + (can_rxframe.data[2] & 0x0F);
            _PMS1.Fault_LDC_No = ((can_rxframe.data[2] >> 4) & 0x0F)
                            + (can_rxframe.data[3] & 0x0F);
            _PMS1.Batt_SW_On =  (can_rxframe.data[3] >> 4) & 0x01;
            _PMS1.Mv_SW_On = (can_rxframe.data[3] >> 5) & 0x01;
            _PMS1.Lv_SW_On = (can_rxframe.data[3] >> 6) & 0x01;
            _PMS1.Batt_Charger_On = (can_rxframe.data[3] >> 7) & 0x01;

            _NewDMI_msg = _NewDMI_msg | 0x01;//bit0 : PMS1
            #if DEBUG_PMS == 1
            gcs().send_text(MAV_SEVERITY_INFO, "PMS1 : %u, %u, %u, %u, %u%u%u%u", 
                        _PMS1.AlivCnt, _PMS1.State, _PMS1.LDC_State, _PMS1.Fault_LDC_No, 
                        _PMS1.Batt_SW_On, _PMS1.Mv_SW_On, _PMS1.Lv_SW_On, _PMS1.Batt_Charger_On);
            #endif
            break;
        case RX_ID_PMS2 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _PMS2.Batt_Output_Current_raw = (int16_t)(uint16_temp2 * 256 + uint16_temp1);
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _PMS2.LDC_Output_Current_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _PMS2.Mv_Output_Current_raw = (int16_t)(uint16_temp2 * 256 + uint16_temp1);
            uint16_temp1 = can_rxframe.data[6];
            uint16_temp2 = can_rxframe.data[7];
            _PMS2.Mv_Battery_Voltage_raw = uint16_temp2 * 256 + uint16_temp1;

            _NewDMI_msg = _NewDMI_msg | 0x02;//bit1 : PMS2
            #if DEBUG_PMS == 1
            gcs().send_text(MAV_SEVERITY_INFO, "PMS2 : %d, %u, %d, %u", 
                        _PMS2.Batt_Output_Current_raw, _PMS2.LDC_Output_Current_raw, _PMS2.Mv_Output_Current_raw, _PMS2.Mv_Battery_Voltage_raw);
            #endif
            break;
        case RX_ID_PMS3 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _PMS3.OutputVoltage_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _PMS3.OutputCurrent_raw = (int16_t)(uint16_temp2 * 256 + uint16_temp1);
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _PMS3.InputVoltage_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[6];
            uint16_temp2 = can_rxframe.data[7];
            _PMS3.InputCurrent_raw = (int16_t)(uint16_temp2 * 256 + uint16_temp1);

            _NewDMI_msg = _NewDMI_msg | 0x04;//bit2 : PMS3
            #if DEBUG_PMS == 1
            gcs().send_text(MAV_SEVERITY_INFO, "PMS3 : %u, %d, %u, %d", 
                        _PMS3.OutputVoltage_raw, _PMS3.OutputCurrent_raw, _PMS3.InputVoltage_raw, _PMS3.InputCurrent_raw);
            #endif
            break;
        case RX_ID_FDC1 :
            _FDC1.AliveCnt   = can_rxframe.data[0];
            _FDC1.State     = can_rxframe.data[1];
            _FDC1.Aux_Volt_raw  = can_rxframe.data[2];
            _FDC1.Max_Temp  = can_rxframe.data[3];
            _FDC1.Flag1.ALL = can_rxframe.data[4];
            _FDC1.Flag2.ALL = can_rxframe.data[5];

            _NewDMI_msg = _NewDMI_msg | 0x08;//bit3 : FDC1
            #if DEBUG_PMS == 1
            gcs().send_text(MAV_SEVERITY_INFO, "FDC1 : %u, %u, %u, %u, %u%u%u%u %u%u%u%u", 
                        _FDC1.AliveCnt, _FDC1.State, _FDC1.Aux_Volt_raw, _FDC1.Max_Temp,
                        _FDC1.Flag1.bits.Ind1_OC_Fault, _FDC1.Flag1.bits.Ind2_OC_Fault,
                        _FDC1.Flag1.bits.Ind3_OC_Fault, _FDC1.Flag1.bits.Ind4_OC_Fault,
                        _FDC1.Flag1.bits.Current_Unbalance_Fault, _FDC1.Flag1.bits.Output_OC_Fault,
                        _FDC1.Flag1.bits.SiC1_Fault, _FDC1.Flag1.bits.SiC2_Fault);
            #endif
            break;
        case RX_ID_FDC2 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _FDC2.OutputVoltage_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _FDC2.OutputCurrent_raw = (int16_t)(uint16_temp2 * 256 + uint16_temp1);
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _FDC2.InputVoltage_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[6];
            uint16_temp2 = can_rxframe.data[7];
            _FDC2.InputCurrent_raw = (int16_t)(uint16_temp2 * 256 + uint16_temp1);

            _NewDMI_msg = _NewDMI_msg | 0x10;//bit4 : FDC2
            #if DEBUG_PMS == 1
            gcs().send_text(MAV_SEVERITY_INFO, "FDC2 : %u, %d, %u, %d", 
                        _FDC2.OutputVoltage_raw, _FDC2.OutputCurrent_raw, _FDC2.InputVoltage_raw, _FDC2.InputCurrent_raw);
            #endif
            break;
        case RX_ID_VCUF1 :
            _VCUFDC1.AliveCnt   = can_rxframe.data[0];
            uint8_temp = can_rxframe.data[1];
            _VCUFDC1.SET_CMD = uint8_temp & 0x0F;
            _VCUFDC1.Fault_Reset = ((uint8_temp >> 4) & 0x0F);
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _VCUFDC1.Target_OutputVoltage_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _VCUFDC1.Target_InputCurrent_raw = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _VCUFDC1.Target_InputPower_raw = uint16_temp2 * 256 + uint16_temp1;

            _NewDMI_msg = _NewDMI_msg | 0x20;//bit5 : VCUF1
            #if DEBUG_PMS == 1
            gcs().send_text(MAV_SEVERITY_INFO, "PMS2 : %u, %u, %u, %u, %u, %u", 
                        _VCUFDC1.AliveCnt, _VCUFDC1.SET_CMD, _VCUFDC1.Fault_Reset,
                        _VCUFDC1.Target_OutputVoltage_raw, _VCUFDC1.Target_InputCurrent_raw, _VCUFDC1.Target_InputPower_raw);
            #endif
            break;
        default:

            break;
    }
}

// -------------------------------------------------------------------------
// Check multiple received Inverter data : Check each messages and transfer to global memory
// -------------------------------------------------------------------------
void AP_COAXCAN2::Check_ALL_data(void)
{   
    //IFCU1
    if(_NewIFCU_msg & 0x01) {
        cxdata().IFCU_data.PpVlt = (float)_IFCU1.PpVlt_raw * 0.01;
        cxdata().IFCU_data.PpCur = (float)_IFCU1.PpCur_raw * 0.01;
        cxdata().IFCU_data.PpCurLim = (float)_IFCU1.PpCurLim_raw * 0.01;
        cxdata().IFCU_data.PpH2Sof = (float)_IFCU1.PpH2Sof_raw * 0.5;
        _IFCU_has_Initialized |= 0x01;
#if DEBUG_IFCU == 1
        gcs().send_text(MAV_SEVERITY_INFO, "IFCU1-i : %.2f, %.2f, %.2f, %.1f",
            cxdata().IFCU_data.PpVlt, cxdata().IFCU_data.PpCur, cxdata().IFCU_data.PpCurLim, cxdata().IFCU_data.PpH2Sof);
#endif
    }
    //IFCU2
    if(_NewIFCU_msg & 0x02) {
        cxdata().IFCU_data.State = _IFCU2.State;
        cxdata().IFCU_data.FltSts = _IFCU2.FltSts;
        cxdata().IFCU_data.DTC = _IFCU2.DTC;
        cxdata().IFCU_data.H2LkLmp = _IFCU2.H2LkLmp;
        cxdata().IFCU_data.SvmlsoRVlu = _IFCU2.SvmlsoRVlu;
        _IFCU_has_Initialized |= 0x02;
    }
    //IFCU3
    if(_NewIFCU_msg & 0x04) {
        cxdata().IFCU_data.FcNetVlt = (float)_IFCU3.FcNetVlt_raw * 0.1;
        cxdata().IFCU_data.FcNetCur = (float)_IFCU3.FcNetCur_raw * 0.1 - 1000.0;
        cxdata().IFCU_data.LdcVlt = (float)_IFCU3.LdcVlt_raw * 0.01;
        cxdata().IFCU_data.LdcCur = (float)_IFCU3.LdcCur_raw * 0.1;
        _IFCU_has_Initialized |= 0x04;
#if DEBUG_IFCU == 1
        gcs().send_text(MAV_SEVERITY_INFO, "IFCU3-i : %.1f, %.1f, %.2f, %.1f",
            cxdata().IFCU_data.FcNetVlt, cxdata().IFCU_data.FcNetCur, cxdata().IFCU_data.LdcVlt, cxdata().IFCU_data.LdcCur);
#endif
    }
    //IFCU4
    if(_NewIFCU_msg & 0x08) {
        cxdata().IFCU_data.FcInClntTmp = _IFCU4.FcInClntTmp;
        cxdata().IFCU_data.AmbTemp = _IFCU4.AmbTemp;
        cxdata().IFCU_data.RoomTemp = _IFCU4.RoomTemp;
        cxdata().IFCU_data.H2TnkPrs = (float)_IFCU4.H2TnkPrs_raw * 0.4;
        cxdata().IFCU_data.H2TnkTmp = (int16_t)_IFCU4.H2TnkTmp - 50;
        cxdata().IFCU_data.H2TnkFillCnt = _IFCU4.H2TnkFillCnt;
        _IFCU_has_Initialized |= 0x08;
#if DEBUG_IFCU == 1
        gcs().send_text(MAV_SEVERITY_INFO, "IFCU1-4 : H2TnkPrs %.1f, H2TnkTmp %d",
            cxdata().IFCU_data.H2TnkPrs, cxdata().IFCU_data.H2TnkTmp);
#endif
    }
    //IFCU5 => not really used, just initialize it
    if(_NewIFCU_msg & 0x10) {
        _IFCU_has_Initialized |= 0x10;
    }
    //IFCU6
    if(_NewIFCU_msg & 0x20) {
        cxdata().IFCU_data.FcMxCurLim = (float)_IFCU6.FcMxCurLim_raw * 0.01;
        cxdata().IFCU_data.FcNetCustCurLim = (float)_IFCU6.FcNetCustCurLim_raw * 0.01;
        cxdata().IFCU_data.H2MidPrs = (uint16_t)_IFCU6.H2MidPrs_raw * 20;
        cxdata().IFCU_data.FcClntFiltChk = _IFCU6.FcClntFiltChk;
        cxdata().IFCU_data.FcClntSplChk = _IFCU6.FcClntSplChk;
        _IFCU_has_Initialized |= 0x20;
#if DEBUG_IFCU == 1
        gcs().send_text(MAV_SEVERITY_INFO, "IFCU6-i : %.2f, %.2f, %d",
            cxdata().IFCU_data.FcMxCurLim, cxdata().IFCU_data.FcNetCustCurLim, cxdata().IFCU_data.H2MidPrs);
#endif
    }
    //PMS1
    if(_NewDMI_msg & 0x01) {
        cxdata().DMI_PMS_data.AlivCnt = _PMS1.AlivCnt;
        cxdata().DMI_PMS_data.PMS_State = _PMS1.State;
        cxdata().DMI_PMS_data.LDC_State = _PMS1.LDC_State;
        cxdata().DMI_PMS_data.Fault_LDC_No = _PMS1.Fault_LDC_No;
        cxdata().DMI_PMS_data.Batt_SW_On = _PMS1.Batt_SW_On;
        cxdata().DMI_PMS_data.Mv_SW_On = _PMS1.Mv_SW_On;
        cxdata().DMI_PMS_data.Lv_SW_On = _PMS1.Lv_SW_On;
        cxdata().DMI_PMS_data.Batt_Charger_On = _PMS1.Batt_Charger_On;
        _DMI_has_Initialized |= 0x01;
    }
    //PMS2
    if(_NewDMI_msg & 0x02) {
        cxdata().DMI_PMS_data.Batt_Output_Current = (float)_PMS2.Batt_Output_Current_raw * 0.1;
        cxdata().DMI_PMS_data.LDC_Output_Current = (float)_PMS2.LDC_Output_Current_raw * 0.1;
        cxdata().DMI_PMS_data.Mv_Output_Current = (float)_PMS2.Mv_Output_Current_raw * 0.1;
        cxdata().DMI_PMS_data.Mv_Battery_Voltage = (float)_PMS2.Mv_Battery_Voltage_raw * 0.1;
        _DMI_has_Initialized |= 0x02;
#if DEBUG_PMS == 1
        gcs().send_text(MAV_SEVERITY_INFO, "PMS2-i : %.1f, %.1f, %.1f, %.1f",
            cxdata().DMI_PMS_data.Batt_Output_Current, cxdata().DMI_PMS_data.LDC_Output_Current, 
            cxdata().DMI_PMS_data.Mv_Output_Current, cxdata().DMI_PMS_data.Mv_Battery_Voltage);
#endif
    }
    //PMS3
    if(_NewDMI_msg & 0x04) {
        cxdata().DMI_PMS_data.HDC_OutputVoltage = (float)_PMS3.OutputVoltage_raw * 0.1;
        cxdata().DMI_PMS_data.HDC_OutputCurrent = (float)_PMS3.OutputCurrent_raw * 0.1;// - 350.0;
        cxdata().DMI_PMS_data.HDC_InputVoltage = (float)_PMS3.InputVoltage_raw * 0.1;
        cxdata().DMI_PMS_data.HDC_InputCurrent = (float)_PMS3.InputCurrent_raw * 0.1;// - 350.0;
        _DMI_has_Initialized |= 0x02;
#if DEBUG_PMS == 1
        gcs().send_text(MAV_SEVERITY_INFO, "PMS2-i : %.1f, %.1f, %.1f, %.1f",
            cxdata().DMI_PMS_data.Batt_Output_Current, cxdata().DMI_PMS_data.LDC_Output_Current, 
            cxdata().DMI_PMS_data.Mv_Output_Current, cxdata().DMI_PMS_data.Mv_Battery_Voltage);
#endif
    }
    //FDC1
    if(_NewDMI_msg & 0x08) {
        cxdata().DMI_PMS_data.FDC_State = _FDC1.State;
        cxdata().DMI_PMS_data.FDC_Aux_Volt = (float)_FDC1.Aux_Volt_raw * 0.1;
        cxdata().DMI_PMS_data.FDC_Max_Temp = (int16_t)_FDC1.Max_Temp - 40;
        cxdata().DMI_PMS_data.FDC_Flag1.ALL = _FDC1.Flag1.ALL;
        cxdata().DMI_PMS_data.FDC_Flag2.ALL = _FDC1.Flag2.ALL;
        _DMI_has_Initialized |= 0x04;
#if DEBUG_IFCU == 1
        gcs().send_text(MAV_SEVERITY_INFO, "FDC1-i : AuxV %.1f, MaxTemp %d",
            cxdata().DMI_PMS_data.FDC_Aux_Volt, cxdata().DMI_PMS_data.FDC_Max_Temp);
#endif
    }
    //FDC2 => not really used, just initialize it
    if(_NewDMI_msg & 0x10) {
        _DMI_has_Initialized |= 0x10;
    }
    //VCUF1 => not really used, just initialize it
    if(_NewDMI_msg & 0x20) {
        _DMI_has_Initialized |= 0x20;
    }

    _NewIFCU_msg = 0;
    _NewDMI_msg = 0;
}

// -------------------------------------------------------------------------
// Transmit data
// -------------------------------------------------------------------------
int AP_COAXCAN2::TXspin()
{
    int cmd_send_res    = 0;
    uint8_t can_data[8] = {0,0,0,0,0,0,0,0};
    uint8_t msgdlc = 8;
    uint32_t can_id = 0xFE;

    can_data[0] = 5;

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::native_micros64() + COAXCAN2_SEND_TIMEOUT_US; 
    
    out_frame = {can_id, can_data, msgdlc};
    cmd_send_res    = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);

    //if(cmd_send_res==1)
    //{
    //    gcs().send_text(MAV_SEVERITY_INFO, "CAN TX OK");
    //}

	return cmd_send_res;
}

// -------------------------------------------------------------------------
// Transmit data
// -------------------------------------------------------------------------
int  AP_COAXCAN2::CAN_TX_std(uint16_t can_id, uint8_t data_cmd[], uint8_t msgdlc)
{
    int cmd_send_res    = 0;
    uint8_t can_data[8] = {0,0,0,0,0,0,0,0};

    memcpy(can_data, data_cmd, msgdlc);

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::native_micros64() + COAXCAN2_SEND_TIMEOUT_US;                      // Should have timeout value

    out_frame       = {can_id, can_data, msgdlc};                                               // id, data[8], dlc
    cmd_send_res    = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);

    return cmd_send_res;
    
}
// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN2::TX_FCC1_MSG(void)
{
    uint8_t temp_data[8] = {0} ;
    uint8_t tempjoin = 0;

    //_FCC_AlivCnt : looping 0~15
    _FCC_CmdFcRunStop = 0;
    _FCC_CmdPmsBatCut = 0;
    _FCC_Ready = cxdata().fcrdy;
    _FCC_Reserved1 = 0;

    tempjoin = _FCC_AlivCnt + ((_FCC_CmdFcRunStop & 0x01) << 4) 
            + ((_FCC_CmdPmsBatCut & 0x01) << 5) + ((_FCC_Ready & 0x01) << 6)
            + ((_FCC_Reserved1 & 0x01) << 7);

    temp_data[0] = tempjoin;

    CAN_TX_std(_cmd_id[TX_ID::TX_ID_FCC1], temp_data, 1);
    _FCC_AlivCnt++;
    _FCC_AlivCnt = _FCC_AlivCnt%16;
}

// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN2::TX_FCC2_MSG(void)
{
    uint8_t temp_data[8] = {0,0,0,0,0,0,0,0} ;

    _FCC_FcPwrReq = 35000;
    _FCC_FcThrottle = 10;
    _FCC_FcThrottlePrdct = 20;

    temp_data[0] = _FCC_FcPwrReq & 0x00FF;
    temp_data[1] = (_FCC_FcPwrReq >> 8) & 0x00FF;
    temp_data[2] = _FCC_FcThrottle & 0x00FF;
    temp_data[3] = (_FCC_FcThrottle >> 8) & 0x00FF;
    temp_data[4] = _FCC_FcThrottlePrdct & 0x00FF;
    temp_data[5] = (_FCC_FcThrottlePrdct >> 8) & 0x00FF;

    CAN_TX_std(_cmd_id[TX_ID::TX_ID_FCC2], temp_data, 6);

}