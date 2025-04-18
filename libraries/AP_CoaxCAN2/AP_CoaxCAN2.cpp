#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_CoaxCAN2.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>

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

AP_COAXCAN2::AP_COAXCAN2()
{
    AP_Param::setup_object_defaults(this, var_info);
    
    _examp.set_default(0);

    _initialized    = false;
    _iface                  = nullptr;

    _rx_ex1_data1 = 0;
    _rx_ex1_data2 = 0;
    _rtr_tx_cnt = 0;

    _coaxcan2_last_send_us = 0;
    _FCC_AlivCnt = 0;
    _FCC_CmdFcRunStop = 0;
    _FCC_CmdPmsBatCut = 0;
    _FCC_Ready = 0;
    _FCC_Reserved1 = 0;
    _FCC_FcPwrReq = 0;
    _FCC_FcThrottle = 0;
    _FCC_FcThrottlePrdct = 0;

    _IFCU1.PpCur = 0;
    _IFCU1.PpCurLim = 0;
    _IFCU1.PpH2Sof = 0;
    _IFCU1.PpVlt = 0;
    _IFCU1.reserved = 0;

    _IFCU2.DTC = 0;
    _IFCU2.FltSts = 0;
    _IFCU2.H2LkLmp = 0;
    _IFCU2.State = 0;
    _IFCU2.SvmlsoRVlu = 0;

    _IFCU3.FcNetCur = 0;
    _IFCU3.FcNetVlt = 0;
    _IFCU3.LdcCur = 0;
    _IFCU3.LdcVlt = 0;

    _IFCU4.AmbTemp = 0;
    _IFCU4.FcInClntTmp = 0;
    _IFCU4.H2TnkFillCnt = 0;
    _IFCU4.H2TnkTmp = 0;
    _IFCU4.H2TnkPrs = 0;
    _IFCU4.RoomTemp = 0;

    _IFCU5.ExWtrTrpFill = 0;
    _IFCU5.HvBsaCur = 0;
    _IFCU5.HvBsaSoC = 0;
    _IFCU5.HvBsaSoH = 0;
    _IFCU5.HvBsaVlt = 0;

    _IFCU6.FcClntFiltChk = 0;
    _IFCU6.FcClntSplChk = 0;
    _IFCU6.FcMxCurLim = 0;
    _IFCU6.FcNetCustCurLim = 0;
    _IFCU6.H2MidPrs = 0;
    
    coaxcan2_period_us = 1000000UL / COAXCAN2_LOOP_HZ;
    
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
                    
        if(_AP_COAXCAN2_loop_cnt%COAXCAN2_MALVINK_INTERVAL==0)      // 100ms period send2ppc
        {
            //send2gcs();     // Send COAX Data to GCS
            //if(COAXCAN2_Fail_Status == COAXCAN2_STATUS::CONNECTED)
            //{
            //    gcs().send_text(MAV_SEVERITY_INFO, "CCB1 %d,%d,%d,%d", _rx_raw_thermist1, _rx_raw_thermist2, _rx_raw_thermist3, _rx_raw_thermist4); // For test
            //    gcs().send_text(MAV_SEVERITY_INFO, "CCB2 %d,%d,%d,%d,%d", _rx_raw_thermocp1, _rx_raw_thermocp2, _rx_raw_wflow, _rx_raw_bdtemp, _rx_raw_state); // For test
            //    gcs().send_text(MAV_SEVERITY_INFO, "TX %d RX %d",(uint16_t)(_rtr_tx_cnt & 0xFFFF),(uint16_t)(_handleFrame_cnt & 0xFFFF));
            //}
        }

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

    if(_AP_COAXCAN2_loop_cnt%20==0)
    // if(_AP_COAXCAN2_loop_cnt%100==0)
    {
        TX_FCC1_MSG();
        _rtr_tx_cnt++;
    }
    else if(_AP_COAXCAN2_loop_cnt%20 == 5)
    // // else if(_AP_COAXCAN2_loop_cnt%100 == 50)
    {
        TX_FCC2_MSG();
        _rtr_tx_cnt++;
    }
    // else
    // {
    //     TXspin();//Transmit unscheduled messages : command from GCS
    //     _rtr_tx_cnt++;
    // }
    //

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
    uint16_t    uint16_temp = 0;
    uint16_t    uint16_temp1 = 0;
    uint16_t    uint16_temp2 = 0;
    uint8_t     uint8_temp = 0;

//    int16_t int16_temp = 0U;

    switch(can_rxframe.id&can_rxframe.MaskStdID)
    {
        case RX_ID_IFCU1 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _IFCU1.PpVlt = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _IFCU1.PpCur = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _IFCU1.PpCurLim = uint16_temp2 * 256 + uint16_temp1;
            _IFCU1.PpH2Sof = can_rxframe.data[6];
            // _IFCU1.reserved = can_rxframe.data[7];
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU1 : %d, %d, %d, %d", _IFCU1.PpVlt, _IFCU1.PpCur, _IFCU1.PpCurLim, _IFCU1.PpH2Sof);
            break;
        case RX_ID_IFCU2 :
            _IFCU2.State = can_rxframe.data[0];
            _IFCU2.FltSts = can_rxframe.data[1]; 
            _IFCU2.DTC = can_rxframe.data[2];
            _IFCU2.H2LkLmp = can_rxframe.data[3];
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _IFCU2.SvmlsoRVlu = uint16_temp2 * 256 + uint16_temp1;
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU2 : %d, %d, %d, %d, %d", _IFCU2.State, _IFCU2.FltSts, _IFCU2.DTC, _IFCU2.H2LkLmp, _IFCU2.SvmlsoRVlu);
            break;
        case RX_ID_IFCU3 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _IFCU3.FcNetVlt = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _IFCU3.FcNetCur = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            _IFCU3.LdcVlt = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[6];
            uint16_temp2 = can_rxframe.data[7];
            _IFCU3.LdcCur = uint16_temp2 * 256 + uint16_temp1;
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU3 : %d, %d, %d, %d", _IFCU3.FcNetVlt, _IFCU3.FcNetCur, _IFCU3.LdcVlt, _IFCU3.LdcCur);
            break;
        case RX_ID_IFCU4 :
            _IFCU4.FcInClntTmp = (int8_t)can_rxframe.data[0];
            _IFCU4.AmbTemp = (int8_t)can_rxframe.data[1];
            _IFCU4.RoomTemp = (int8_t)can_rxframe.data[2];
            _IFCU4.H2TnkPrs = can_rxframe.data[3];
            _IFCU4.H2TnkTmp = can_rxframe.data[4];
            uint16_temp1 = can_rxframe.data[5];
            uint16_temp2 = can_rxframe.data[6];
            _IFCU4.H2TnkFillCnt = uint16_temp2 * 256 + uint16_temp1;
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU4 : %d, %d, %d, %u, %u, %u", _IFCU4.FcInClntTmp, _IFCU4.AmbTemp, _IFCU4.RoomTemp, _IFCU4.H2TnkPrs, _IFCU4.H2TnkTmp, _IFCU4.H2TnkFillCnt);
            break;
        case RX_ID_IFCU5 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _IFCU5.HvBsaVlt = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _IFCU5.HvBsaCur = uint16_temp2 * 256 + uint16_temp1;
            _IFCU5.HvBsaSoC = can_rxframe.data[4];
            _IFCU5.HvBsaSoH = can_rxframe.data[5];
            _IFCU5.ExWtrTrpFill = can_rxframe.data[6] & 0x01;
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU5 : %d, %d, %d, %d, %d", _IFCU5.HvBsaVlt, _IFCU5.HvBsaCur, _IFCU5.HvBsaSoC, _IFCU5.HvBsaSoH, _IFCU5.ExWtrTrpFill);
            break;
        case RX_ID_IFCU6 :
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            _IFCU6.FcMxCurLim = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            _IFCU6.FcNetCustCurLim = uint16_temp2 * 256 + uint16_temp1;
            _IFCU6.H2MidPrs = can_rxframe.data[4];
            uint8_temp = can_rxframe.data[5];
            _IFCU6.FcClntFiltChk = uint8_temp & 0x01;
            _IFCU6.FcClntSplChk = (uint8_temp >> 1) & 0x01;
            gcs().send_text(MAV_SEVERITY_INFO, "IFCU6 : %d, %d, %d, %d, %d", _IFCU6.FcMxCurLim, _IFCU6.FcNetCustCurLim, _IFCU6.H2MidPrs, _IFCU6.FcClntFiltChk, _IFCU6.FcClntSplChk);
            break;
        case RX_ID_CCB1:
            //Thermist 1 temperature
            uint16_temp = can_rxframe.data[0];
            _rx_raw_thermist1 = uint16_temp * 256 + can_rxframe.data[1];
            //Thermist 2 temperature
            uint16_temp = can_rxframe.data[2];
            _rx_raw_thermist2 = uint16_temp * 256 + can_rxframe.data[3];
            //Thermist 3 temperature
            uint16_temp = can_rxframe.data[4];
            _rx_raw_thermist3 = uint16_temp * 256 + can_rxframe.data[5];
            //Thermist 4 temperature
            uint16_temp = can_rxframe.data[6];
            _rx_raw_thermist4 = uint16_temp * 256 + can_rxframe.data[7];
            
            _handleFrame_cnt++;

            break;

        case RX_ID_CCB2:

            //Thermocouple 1 temperature
            uint16_temp = can_rxframe.data[0];
            _rx_raw_thermocp1 = uint16_temp * 256 + can_rxframe.data[1];
            //Thermocouple 2 temperature
            uint16_temp = can_rxframe.data[2];
            _rx_raw_thermocp2 = uint16_temp * 256 + can_rxframe.data[3];
            //(Water)Flow sensor
            uint16_temp = can_rxframe.data[4];
            _rx_raw_wflow = uint16_temp * 256 + can_rxframe.data[5];
            //Board temperature
            _rx_raw_bdtemp = can_rxframe.data[6];
            //Cooling controller state
            _rx_raw_state = can_rxframe.data[7];

            _handleFrame_cnt++;

            break;

        default:

            break;
    }
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

    if(cmd_send_res==1)
	{
		//success
	    _cmd_tx_cnt++;
	}
	else if(cmd_send_res==0)
	{
		_cmd_tx_err++;
		//CMD TX buffer full
	}
	else
	{
		_cmd_tx_err++;
		//CMD TX error
	}

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
    _FCC_CmdFcRunStop = 1;
    _FCC_CmdPmsBatCut = 0;
    //_FCC_Ready : set by GCS
    _FCC_Reserved1 = 0;

    _FCC_Ready = 1;//Temp debugging

    tempjoin = _FCC_AlivCnt + ((_FCC_CmdFcRunStop & 0x01) << 4) 
            + ((_FCC_CmdPmsBatCut & 0x01) << 5) + ((_FCC_Ready & 0x01) << 6)
            + ((_FCC_Reserved1 & 0x01) << 7);

    temp_data[0] = tempjoin;

    CAN_TX_std(CMD_ID::CMD_ID_FCC1, temp_data, 1);

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
    _FCC_FcThrottle = 100;
    _FCC_FcThrottlePrdct = 100;

    temp_data[0] = _FCC_FcPwrReq & 0x00FF;
    temp_data[1] = (_FCC_FcPwrReq >> 8) & 0x00FF;
    temp_data[2] = _FCC_FcThrottle & 0x00FF;
    temp_data[3] = (_FCC_FcThrottle >> 8) & 0x00FF;
    temp_data[4] = _FCC_FcThrottlePrdct & 0x00FF;
    temp_data[5] = (_FCC_FcThrottlePrdct >> 8) & 0x00FF;

    CAN_TX_std(CMD_ID::CMD_ID_FCC2, temp_data, 6);

}