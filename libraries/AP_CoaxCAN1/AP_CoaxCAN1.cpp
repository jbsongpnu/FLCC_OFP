#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_CoaxCAN1.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_CoaxCAN2/Coaxial_data.h>

extern const AP_HAL::HAL& hal;

#define TEMP_EXP 0		//Initial value
#define DEBUG_INVERTER 1

// Table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_COAXCAN1::var_info[] = {
    // @Param: example
    // @DisplayName: User example
    // @Description: Example for user
    AP_GROUPINFO("Ex", 1, AP_COAXCAN1, _examp, TEMP_EXP),
    //AP_GROUPINFO("Examp", 1, AP_COAXCAN, TEMP_EXP, TEMP_EXP),
    // AP_GROUPINFO("PARAM1", 1, AP_COAXCAN, _pmu_param1, TEMP_EXP),
    // AP_GROUPINFO("PARAM2", 2, AP_COAXCAN, _pmu_param2, TEMP_EXP),
    // AP_GROUPINFO("PARAM3", 3, AP_COAXCAN, _pmu_param3, TEMP_EXP),
    // AP_GROUPINFO("PARAM4", 4, AP_COAXCAN, _pmu_param4, TEMP_EXP),

    AP_GROUPEND
};

AP_COAXCAN1::AP_COAXCAN1()
{
    AP_Param::setup_object_defaults(this, var_info);
    
    _examp.set_default(0);

    _initialized    = false;
    _iface                  = nullptr;

    _rx_ex1_data1 = 0;
    _rx_ex1_data2 = 0;
    _rtr_tx_cnt = 0;

    _coaxcan1_last_send_us = 0;

    coaxcan1_period_us = 1000000UL / COAXCAN1_LOOP_HZ;

    _cmd_id[TX_ID::TX_ID_EX1]         = 0               | coaxcan1::CanFrame::FlagEFF;
    _cmd_id[TX_ID::TX_ID_EX2]         = 1               | coaxcan1::CanFrame::FlagEFF;
    _cmd_id[TX_ID::TX_ID_INV_SET_CMD] = ID_INV_SET_CMD  | coaxcan1::CanFrame::FlagEFF;
    _cmd_id[TX_ID::TX_ID_INV_SET_CC]  = ID_INV_SET_CC   | coaxcan1::CanFrame::FlagEFF;
    _cmd_id[TX_ID::TX_ID_INV_SET_SC]  = ID_INV_SET_SC   | coaxcan1::CanFrame::FlagEFF;
    _cmd_id[TX_ID::TX_ID_INV_SET_FLT] = ID_INV_SET_FLT  | coaxcan1::CanFrame::FlagEFF;
    
}

AP_COAXCAN1::~AP_COAXCAN1()
{    
}

AP_COAXCAN1 *AP_COAXCAN1::get_coaxcan1(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_CoaxCAN1) {
        return nullptr;
    }
    return static_cast<AP_COAXCAN1*>(AP::can().get_driver(driver_index));
}

// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
bool AP_COAXCAN1::add_interface(AP_HAL::CANIface* can_iface) {

    if (_can_iface != nullptr) {
    	hal.console->printf("COAXCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
    	hal.console->printf("COAXCAN: CAN driver not found\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN: CAN driver not found");
        return false;
    }

    if (!_can_iface->is_initialized()) {
    	hal.console->printf("COAXCAN: Driver not initialized\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN: Driver not initialized");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
    	hal.console->printf("COAXCAN: Cannot add event handle\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN: Cannot add event handle");
        return false;
    }
    return true;
}

// -------------------------------------------------------------------------
// Initialize COAXCAN bus
// -------------------------------------------------------------------------
void AP_COAXCAN1::init(uint8_t driver_index, bool enable_filters)
{
	_driver_index = driver_index;

    if (_initialized) {
        return;
    }

    if (_can_iface == nullptr) {
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_COAXCAN1::loop, void), _thread_name, 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;


    //gcs().send_text(MAV_SEVERITY_INFO, "[COAX] Initialized");
}

// -------------------------------------------------------------------------
// Task
// -------------------------------------------------------------------------
void AP_COAXCAN1::loop(void)
{
    while (true)
    {
        if (!_initialized)
        {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        hal.scheduler->delay_microseconds(coaxcan1_period_us);    // 5ms period loop

        run();
                    
        if(_AP_COAXCAN1_loop_cnt%COAXCAN1_MALVINK_INTERVAL==0)      // 100ms period send2ppc
        {
            //send2gcs();     // Send COAX Data to GCS
            //if(COAXCAN1_Fail_Status == COAXCAN1_STATUS::CONNECTED)
            //{
            //    gcs().send_text(MAV_SEVERITY_INFO, "CCB1 %d,%d,%d,%d", _rx_raw_thermist1, _rx_raw_thermist2, _rx_raw_thermist3, _rx_raw_thermist4); // For test
            //    gcs().send_text(MAV_SEVERITY_INFO, "CCB2 %d,%d,%d,%d,%d", _rx_raw_thermocp1, _rx_raw_thermocp2, _rx_raw_wflow, _rx_raw_bdtemp, _rx_raw_state); // For test
            //    gcs().send_text(MAV_SEVERITY_INFO, "TX %d RX %d",(uint16_t)(_rtr_tx_cnt & 0xFFFF),(uint16_t)(_handleFrame_cnt & 0xFFFF));
            //}
        }

        _AP_COAXCAN1_loop_cnt++;                                  // 5ms period increase
    }
}

// -------------------------------------------------------------------------
// Run : 200Hz
// [_AP_COAXCAN1_loop_cnt] is increased from loop() at every 5ms(200Hz)
// -------------------------------------------------------------------------
void AP_COAXCAN1::run(void)
{
    //Receive
	RXspin();

    //Check data 
    if(_AP_COAXCAN1_loop_cnt%10==0) {
        Check_INV_data();
    }
    //if(_AP_COAXCAN1_loop_cnt%20==0)
    if(_AP_COAXCAN1_loop_cnt%100==0)
    {
        TX_INV_SETCMD_MSG();
        _rtr_tx_cnt++;
    }
    // else if(_AP_COAXCAN1_loop_cnt%20 == 5)
    else if(_AP_COAXCAN1_loop_cnt%100 == 20)
    {
        TX_INV_SETCC_MSG();
        _rtr_tx_cnt++;
    }
    else if(_AP_COAXCAN1_loop_cnt%100 == 40)
    {
        TX_INV_SETSC_MSG();
        _rtr_tx_cnt++;
    }
    else if(_AP_COAXCAN1_loop_cnt%100 == 60)
    {
        TX_INV_SETFLT_MSG();
        _rtr_tx_cnt++;
    }

}

// -------------------------------------------------------------------------
// RX 
// -------------------------------------------------------------------------
void AP_COAXCAN1::RXspin()
{
    uint64_t time, timeout;
    int res = 0;

    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), COAXCAN1_SEND_TIMEOUT_US);
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
        //gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 COMMUNICATION_ERROR");
        // return if no data is available to read
        // if(COAXCAN1_Fail_Status==COAXCAN1_STATUS::CONNECTION_FAILURE)
        // {
        //     COAXCAN1_Fail_Status = COAXCAN1_STATUS::CONNECTION_FAILURE;
        //     gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 CONNECTION_FAILURE");
        // }
        // else
        // {
        //     COAXCAN1_Fail_Status = COAXCAN1_STATUS::COMMUNICATION_ERROR;
        //     gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 COMMUNICATION_ERROR");
        // }

        return;
    }


    if(COAXCAN1_Fail_Status == COAXCAN1_STATUS::CONNECTED)     // Normal Connection 
    {
        res = _can_iface->receive(frame, time, flags);

        if(res > 0) // Data Received Normaly
        {
            if(COAXCAN1_ErrCnt > 0) // Check Error Count
            {
                COAXCAN1_ErrCnt = COAXCAN1_ErrCnt - 1;    // Decrease Error Count
            }

            while(res > 0)
            {
                handleFrame(frame);
                res = _can_iface->receive(frame, time, flags);     // Try Receive
            }
        }
        else        // Data Not Received
        {
            COAXCAN1_ErrCnt = COAXCAN1_ErrCnt + 1;          // Increase Error Count

            if(COAXCAN1_ErrCnt == 10) // Check Max Err Count
            {
                COAXCAN1_Fail_Status  = COAXCAN1_STATUS::COMMUNICATION_ERROR; // Set Communication Error Flag 
                gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 COMMUNICATION_ERROR ErrCnt10");
                COAXCAN1_ErrCnt       = 0; // Reset Error Count
            }
        }
    }
    else                            // Abnormal Connection 
    {
        res = _can_iface->receive(frame, time, flags);

        if(res > 0) // Data Received Normaly
        {
            COAXCAN1_RcvrCnt = COAXCAN1_RcvrCnt + 1;    // Increase Receive Count

            if(COAXCAN1_RcvrCnt == 5) // Check Max Recv Count
            {
                COAXCAN1_Fail_Status  = COAXCAN1_STATUS::CONNECTED; // Clear Communication Error Flag 
                COAXCAN1_RcvrCnt      = 0; // Reset Receive Count
                gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 Connected Err %d", COAXCAN1_ErrCnt);
            }

            while(res > 0)
            {
                handleFrame(frame);
                res = _can_iface->receive(frame, time, flags);     // Receive again
            }

        }
        else        // Data Not Received
        {
            if((COAXCAN1_RcvrCnt > 0) & (res < 0)) // Check Receive Count
            {
                COAXCAN1_RcvrCnt = COAXCAN1_RcvrCnt - 1;    // Decrease Receive Count
            }
        }

    }

}

// -------------------------------------------------------------------------
// Handle Frame : process each CAN frame
// -------------------------------------------------------------------------
void AP_COAXCAN1::handleFrame(const AP_HAL::CANFrame& can_rxframe)
{
    uint16_t    uint16_temp = 0;
    uint16_t    uint16_temp1 = 0;
    uint16_t    uint16_temp2 = 0;
    //uint8_t     uint8_temp = 0;

    int16_t int16_temp1 = 0;
    int16_t int16_temp2 = 0;
    int32_t int32_temp1 = 0;
    int32_t int32_temp2 = 0;
    //gcs().send_text(MAV_SEVERITY_INFO, "can received %lu", can_rxframe.id);
    switch(can_rxframe.id&can_rxframe.MaskExtID)
    {
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
            break;

        case RX_ID_INV_GET_CMD:
            INV_GET_CMD.BYTE0.ALL = can_rxframe.data[0];
            int32_temp1 = can_rxframe.data[1];
            int32_temp2 = can_rxframe.data[2];
            INV_GET_CMD.Ref1_RAW = (int32_temp2 << 8 ) + int32_temp1;
            int32_temp1 = can_rxframe.data[3];
            int32_temp2 = can_rxframe.data[4];
            INV_GET_CMD.Ref1_RAW += (int32_temp2 << 24 ) + (int32_temp2 << 16 );
            int16_temp1 = can_rxframe.data[5];
            int16_temp2 = can_rxframe.data[6];
            INV_GET_CMD.Ref2_RAW = (int16_temp2 << 8 ) + int16_temp1;
            
            INV_GET_CMD.Reference1 = (float)INV_GET_CMD.Ref1_RAW * 0.1;
            INV_GET_CMD.Reference2 = (float)INV_GET_CMD.Ref2_RAW * 0.1;

            _NewINV_msg = _NewINV_msg | 0x01;//bit0 : CMD
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVGETCMD : (Mode%u) %u, %ld, %d", 
                INV_GET_CMD.BYTE0.bits.Ctrl_Mode, INV_GET_CMD.BYTE0.ALL, 
                INV_GET_CMD.Ref1_RAW, INV_GET_CMD.Ref2_RAW);
            gcs().send_text(MAV_SEVERITY_INFO, "ref1=%f, ref2=%f", 
                INV_GET_CMD.Reference1, INV_GET_CMD.Reference2);
#endif
            break;
        case RX_ID_INV_GET_CC:
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            INV_GET_CC.Gain_Kpc_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            INV_GET_CC.Gain_Kic_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            INV_GET_CC.Current_Limit = uint16_temp2 * 256 + uint16_temp1;

            INV_GET_CC.Gain_Kpc = (float)INV_GET_CC.Gain_Kpc_RAW * 0.01;
            INV_GET_CC.Gain_Kic = (float)INV_GET_CC.Gain_Kic_RAW * 0.1;
            //Current_Limit has no scale factor

            _NewINV_msg = _NewINV_msg | 0x02;//bit1 : CC
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVGETCC : %u, %u, %u", 
                INV_GET_CC.Gain_Kpc_RAW, INV_GET_CC.Gain_Kic_RAW, INV_GET_CC.Current_Limit);
            gcs().send_text(MAV_SEVERITY_INFO, "Kpc %f, Kic %f", 
                INV_GET_CC.Gain_Kpc, INV_GET_CC.Gain_Kic);
#endif
            break;
        case RX_ID_INV_GET_SC:
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            INV_GET_SC.Gain_Kps_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            INV_GET_SC.Gain_Kis_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            INV_GET_SC.Theta_Offset_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[6];
            uint16_temp2 = can_rxframe.data[7];
            INV_GET_SC.Speed_Limit = uint16_temp2 * 256 + uint16_temp1;
            
            INV_GET_SC.Gain_Kps = (float)INV_GET_SC.Gain_Kps_RAW * 0.01;
            INV_GET_SC.Gain_Kis = (float)INV_GET_SC.Gain_Kis_RAW * 0.1;
            INV_GET_SC.Theta_Offset = (float)INV_GET_SC.Theta_Offset_RAW * 0.1;
            //Speed_Limit has no scale factor

            _NewINV_msg = _NewINV_msg | 0x04;//bit2 : SC
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVGETSC : %u, %u, %u, %u", 
                INV_GET_SC.Gain_Kps_RAW, INV_GET_SC.Gain_Kis_RAW, 
                INV_GET_SC.Theta_Offset_RAW, INV_GET_SC.Speed_Limit);
            gcs().send_text(MAV_SEVERITY_INFO, "Kps %f, Kis %f, thoffset %f", 
                INV_GET_SC.Gain_Kps, INV_GET_SC.Gain_Kis, INV_GET_SC.Theta_Offset);
#endif
            break;
        case RX_ID_INV_GET_FLT:
            INV_GET_FLT.OVL_RAW = can_rxframe.data[0];
            INV_GET_FLT.UVL_RAW = can_rxframe.data[1];
            INV_GET_FLT.OCL_RAW = can_rxframe.data[2];
            INV_GET_FLT.OTL_RAW = can_rxframe.data[3];
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            INV_GET_FLT.OSL_RAW = uint16_temp2 * 256 + uint16_temp1;
            
            INV_GET_FLT.OVL = (uint16_t)INV_GET_FLT.OVL_RAW * 0.1;
            INV_GET_FLT.UVL = (uint16_t)INV_GET_FLT.UVL_RAW * 0.1;
            INV_GET_FLT.OCL = (uint16_t)INV_GET_FLT.OCL_RAW * 0.1;
            INV_GET_FLT.OTL = (uint16_t)INV_GET_FLT.OTL_RAW * 0.1;
            INV_GET_FLT.OSL = INV_GET_FLT.OSL_RAW;

            _NewINV_msg = _NewINV_msg | 0x08;//bit3 : FLT
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVGETFLT : real %u, %u, %u, %u, %u", 
                INV_GET_FLT.OVL, INV_GET_FLT.UVL, INV_GET_FLT.OCL,
                INV_GET_FLT.OTL, INV_GET_FLT.OSL);
#endif
            break;
        case RX_ID_INV_GET_STATUS1:
            int16_temp1 = can_rxframe.data[0];
            int16_temp2 = can_rxframe.data[1];
            INV_Status1.Motor_Spd_RAW = int16_temp2 * 256 + int16_temp1;
            int16_temp1 = can_rxframe.data[2];
            int16_temp2 = can_rxframe.data[3];
            INV_Status1.i_a_RAW = int16_temp2 * 256 + int16_temp1;
            int16_temp1 = can_rxframe.data[4];
            int16_temp2 = can_rxframe.data[5];
            INV_Status1.i_b_RAW = int16_temp2 * 256 + int16_temp1;
            int16_temp1 = can_rxframe.data[6];
            int16_temp2 = can_rxframe.data[7];
            INV_Status1.i_c_RAW = int16_temp2 * 256 + int16_temp1;

            INV_Status1.motor_Spd = (float)INV_Status1.Motor_Spd_RAW * 0.2;
            INV_Status1.i_a = (float)INV_Status1.i_a_RAW * 0.01;
            INV_Status1.i_b = (float)INV_Status1.i_b_RAW * 0.01;
            INV_Status1.i_c = (float)INV_Status1.i_c_RAW * 0.01;

            _NewINV_msg = _NewINV_msg | 0x10;//bit4 : st1
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVStatus1 : %u, %u, %u, %u", 
                INV_Status1.Motor_Spd_RAW , INV_Status1.i_a_RAW,
                INV_Status1.i_b_RAW, INV_Status1.i_c_RAW);
            gcs().send_text(MAV_SEVERITY_INFO, "MS %f, Ia %f, Ib %f, Ic %f", 
                INV_Status1.motor_Spd, INV_Status1.i_a, 
                INV_Status1.i_b, INV_Status1.i_c);
#endif
            break;
        case RX_ID_INV_GET_STATUS2:
            //Byte0 ~ 7 are not used
            int16_temp1 = can_rxframe.data[6];
            int16_temp2 = can_rxframe.data[7];
            INV_Status2.t_a_RAW = int16_temp2 * 256 + int16_temp1;
            INV_Status2.t_a = (float)INV_Status2.t_a_RAW * 0.01;

            _NewINV_msg = _NewINV_msg | 0x20;//bit5 : st2
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVStatus2 : %u, ta %f", 
                INV_Status2.t_a_RAW, INV_Status2.t_a);
#endif
            break;
        case RX_ID_INV_GET_STATUS3:
            int16_temp1 = can_rxframe.data[0];
            int16_temp2 = can_rxframe.data[1];
            INV_Status3.t_b_RAW = int16_temp2 * 256 + int16_temp1;
            int16_temp1 = can_rxframe.data[2];
            int16_temp2 = can_rxframe.data[3];
            INV_Status3.t_c_RAW = int16_temp2 * 256 + int16_temp1;
            INV_Status3.t_b = (float)INV_Status3.t_b_RAW * 0.01;
            INV_Status3.t_c = (float)INV_Status3.t_c_RAW * 0.01;

            _NewINV_msg = _NewINV_msg | 0x40;//bit6 : st3
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVStatus3 : %u, %u, ta %f, tc %f", 
                INV_Status3.t_b_RAW, INV_Status3.t_c_RAW, INV_Status3.t_b, INV_Status3.t_c);
#endif
            break;
        case RX_ID_INV_GET_STATUS4:
            //Bytes 0 ~ 1 not used
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            INV_Status4.Flagset.ALL = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            INV_Status4.V_dc_input_RAW = uint16_temp2 * 256 + uint16_temp1;
            INV_Status4.MI = can_rxframe.data[6];
            INV_Status4.Motor_Align_flag = can_rxframe.data[7] & 0x01;

            INV_Status4.V_dc_input = (float)INV_Status4.V_dc_input_RAW * 0.1;

            _NewINV_msg = _NewINV_msg | 0x80;//bit7 : st4
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVStatus4 : %u, %u, %u, %u, Vdc %f", 
                INV_Status4.Flagset.ALL, INV_Status4.V_dc_input_RAW, 
                INV_Status4.MI, INV_Status4.Motor_Align_flag, INV_Status4.V_dc_input);
#endif
            break;
        default:

            break;
    }
}

// -------------------------------------------------------------------------
// Check multiple received data : Check each messages and transfer to global memory
// -------------------------------------------------------------------------
void AP_COAXCAN1::Check_INV_data(void)
{   //To do : check min/max, change for command and etc. later
    //CMD
    if(_NewINV_msg & 0x01) {
        cxdata().INV_data.CMD_Flag.ALL = INV_GET_CMD.BYTE0.ALL;
        cxdata().INV_data.Reference1 = INV_GET_CMD.Reference1;
        cxdata().INV_data.Reference2 = INV_GET_CMD.Reference2;
        if(INV_GET_CMD.BYTE0.bits.Ctrl_Mode==4){
            cxdata().INV_data.Motor_RPM_CMD = INV_GET_CMD.Ref1_RAW / 10;
            cxdata().INV_data.Motor_RPM_CMD = INV_GET_CMD.Ref1_RAW / 10;
        }
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x01;
    }
    //CC
    if(_NewINV_msg & 0x02) {
        cxdata().INV_data.Gain_Kpc = INV_GET_CC.Gain_Kpc;
        cxdata().INV_data.Gain_Kic = INV_GET_CC.Gain_Kic;
        cxdata().INV_data.Current_Limit = INV_GET_CC.Current_Limit;
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x02;
    }
    //SC
    if(_NewINV_msg & 0x04) {
        cxdata().INV_data.Gain_Kps = INV_GET_SC.Gain_Kps;
        cxdata().INV_data.Gain_Kis = INV_GET_SC.Gain_Kis;
        cxdata().INV_data.Theta_Offset = INV_GET_SC.Theta_Offset;
        cxdata().INV_data.Speed_Limit = INV_GET_SC.Speed_Limit;
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x04;
    }
    //FLT
    if(_NewINV_msg & 0x08) {
        cxdata().INV_data.OVL = INV_GET_FLT.OVL;
        cxdata().INV_data.UVL = INV_GET_FLT.UVL;
        cxdata().INV_data.OCL = INV_GET_FLT.OCL;
        cxdata().INV_data.OTL = INV_GET_FLT.OTL;
        cxdata().INV_data.OSL = INV_GET_FLT.OSL;
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x08;
    }
    //status1
    if(_NewINV_msg & 0x10) {
        cxdata().INV_data.motor_Spd = INV_Status1.motor_Spd;
        cxdata().INV_data.i_a = INV_Status1.i_a;
        cxdata().INV_data.i_b = INV_Status1.i_b;
        cxdata().INV_data.i_c = INV_Status1.i_c;

        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x10;
    }
    //status2
    if(_NewINV_msg & 0x20) {
        cxdata().INV_data.t_a = INV_Status2.t_a;
        
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x20;
    }
    //status3
    if(_NewINV_msg & 0x40) {
        cxdata().INV_data.t_b = INV_Status3.t_b;
        cxdata().INV_data.t_c = INV_Status3.t_c;
        
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x40;
    }
    //status4
    if(_NewINV_msg & 0x80) {
        cxdata().INV_data.FLT.ALL = INV_Status4.Flagset.ALL;
        cxdata().INV_data.V_dc_input = INV_Status4.V_dc_input;
        cxdata().INV_data.MI = INV_Status4.MI;
        cxdata().INV_data.Motor_Align_flag = INV_Status4.Motor_Align_flag;

        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x80;
    }
    _NewINV_msg = 0;
}
// -------------------------------------------------------------------------
// Transmit data
// -------------------------------------------------------------------------
int AP_COAXCAN1::TXspin()
{
    int cmd_send_res    = 0;
    uint8_t can_data[8] = {0,0,0,0,0,0,0,0};
    uint8_t msgdlc = 8;
    uint32_t can_id = 0xFE;

    can_data[0] = 5;

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::native_micros64() + COAXCAN1_SEND_TIMEOUT_US; 
    
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
int  AP_COAXCAN1::CAN_TX_Ext(uint32_t can_id, uint8_t data_cmd[], uint8_t msgdlc)
{
    int cmd_send_res    = 0;
    uint8_t can_data[8] = {0,0,0,0,0,0,0,0};

    memcpy(can_data, data_cmd, msgdlc);

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::native_micros64() + COAXCAN1_SEND_TIMEOUT_US;                      // Should have timeout value

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
void AP_COAXCAN1::TX_INV_SETCMD_MSG(void)
{
    static uint8_t count = 0;
    uint8_t temp_data[8] = {0} ;

    if(count%2 == 0) {
        INV_SET_CMD.BYTE0.ALL = 0;
        INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = 0;
        INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 0;
        INV_SET_CMD.BYTE0.bits.Fault_Clear = 0;
        INV_SET_CMD.Reference1 = 0;
        INV_SET_CMD.Reference2 = 0;
    }else {
        INV_SET_CMD.BYTE0.ALL = 0xFF;
        // INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = 1;
        // INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 1;
        // INV_SET_CMD.BYTE0.bits.Fault_Clear = 1;
        INV_SET_CMD.Reference1 = -0.1;//3276.7;
        INV_SET_CMD.Reference2 = -0.1;//-3276.8;
    }
    count++;
    INV_SET_CMD.Ref1_RAW = (int16_t)(INV_SET_CMD.Reference1 * 10.0);
    INV_SET_CMD.Ref2_RAW = (int16_t)(INV_SET_CMD.Reference2 * 10.0);

    temp_data[0] = INV_SET_CMD.BYTE0.ALL;
    temp_data[1] = INV_SET_CMD.Ref1_RAW & 0x00FF;
    temp_data[2] = (INV_SET_CMD.Ref1_RAW >> 8) & 0x00FF;
    temp_data[3] = INV_SET_CMD.Ref2_RAW & 0x00FF;
    temp_data[4] = (INV_SET_CMD.Ref2_RAW >> 8) & 0x00FF;

    CAN_TX_Ext(_cmd_id[TX_ID::TX_ID_INV_SET_CMD], temp_data, 5);//DLC changed to 5
}

// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN1::TX_INV_SETCC_MSG(void)
{
    uint8_t temp_data[8] = {0} ;

    INV_SET_CC.Gain_Kpc = 655.35;
    INV_SET_CC.Gain_Kic = 6553.5;
    INV_SET_CC.Current_Limit = 180;

    INV_SET_CC.Gain_Kpc_RAW = (int16_t)(INV_SET_CC.Gain_Kpc * 100);
    INV_SET_CC.Gain_Kic_RAW = (int16_t)(INV_SET_CC.Gain_Kic * 10);

    temp_data[0] = INV_SET_CC.Gain_Kpc_RAW & 0x00FF;
    temp_data[1] = (INV_SET_CC.Gain_Kpc_RAW >> 8) & 0x00FF;
    temp_data[2] = INV_SET_CC.Gain_Kic_RAW & 0x00FF;
    temp_data[3] = (INV_SET_CC.Gain_Kic_RAW >> 8) & 0x00FF;
    temp_data[4] = INV_SET_CC.Current_Limit & 0x00FF;
    temp_data[5] = (INV_SET_CC.Current_Limit >> 8) & 0x00FF;
    //bytes 6 ~7 reserved => sent with zeros

    CAN_TX_Ext(_cmd_id[TX_ID::TX_ID_INV_SET_CC], temp_data, 8);
}

// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN1::TX_INV_SETSC_MSG(void)
{
    uint8_t temp_data[8] = {0} ;

    INV_SET_CC.Gain_Kpc = 655.35;
    INV_SET_CC.Gain_Kic = 6553.5;
    INV_SET_CC.Current_Limit = 180;

    INV_SET_CC.Gain_Kpc_RAW = (int16_t)(INV_SET_CC.Gain_Kpc * 100);
    INV_SET_CC.Gain_Kic_RAW = (int16_t)(INV_SET_CC.Gain_Kic * 10);

    temp_data[0] = INV_SET_CC.Gain_Kpc_RAW & 0x00FF;
    temp_data[1] = (INV_SET_CC.Gain_Kpc_RAW >> 8) & 0x00FF;
    temp_data[2] = INV_SET_CC.Gain_Kic_RAW & 0x00FF;
    temp_data[3] = (INV_SET_CC.Gain_Kic_RAW >> 8) & 0x00FF;
    temp_data[4] = INV_SET_CC.Current_Limit & 0x00FF;
    temp_data[5] = (INV_SET_CC.Current_Limit >> 8) & 0x00FF;
    //bytes 6 ~7 reserved => sent with zeros

    CAN_TX_Ext(_cmd_id[TX_ID::TX_ID_INV_SET_SC], temp_data, 8);
}

// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN1::TX_INV_SETFLT_MSG(void)
{
    uint8_t temp_data[8] = {0} ;

    INV_SET_FLT.OVL = 500;
    INV_SET_FLT.UVL = 350;
    INV_SET_FLT.OCL = 200;
    INV_SET_FLT.OTL = 80;
    INV_SET_FLT.OSL = 6000;

    INV_SET_FLT.OVL_RAW = INV_SET_FLT.OVL / 10;
    INV_SET_FLT.UVL_RAW = INV_SET_FLT.UVL / 10;
    INV_SET_FLT.OCL_RAW = INV_SET_FLT.OCL / 10;
    INV_SET_FLT.OTL_RAW = INV_SET_FLT.OTL / 10;
    INV_SET_FLT.OSL_RAW = INV_SET_FLT.OSL;

    temp_data[0] = INV_SET_FLT.OVL_RAW;
    temp_data[1] = INV_SET_FLT.UVL_RAW;
    temp_data[2] = INV_SET_FLT.OCL_RAW;
    temp_data[3] = INV_SET_FLT.OTL_RAW;
    //temp_data[4] = 0; //reserved
    temp_data[5] = INV_SET_FLT.OSL_RAW;
    //bytes 6 ~7 reserved => sent with zeros

    CAN_TX_Ext(_cmd_id[TX_ID::TX_ID_INV_SET_FLT], temp_data, 6);//DLC changed to 6
}
