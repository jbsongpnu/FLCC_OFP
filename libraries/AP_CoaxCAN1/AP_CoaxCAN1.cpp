#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_CoaxCAN1.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

#define TEMP_EXP 0		//Initial value

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

        hal.scheduler->delay_microseconds(coaxcan1_period_us);    // 10ms period loop

        if(_AP_COAXCAN1_loop_cnt%COAXCAN1_MINOR_INTERVAL==0)        // 20ms period run
        {    
            run();
            //gcs().send_text(MAV_SEVERITY_INFO, "[COAX] run"); // For test
        }
            
        if(_AP_COAXCAN1_loop_cnt%COAXCAN1_MALVINK_INTERVAL==0)      // 100ms period send2ppc
        {
            //send2gcs();     // Send COAX Data to GCS
            if(COAXCAN1_Fail_Status == COAXCAN1_STATUS::CONNECTED)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "CCB1 %d,%d,%d,%d", _rx_raw_thermist1, _rx_raw_thermist2, _rx_raw_thermist3, _rx_raw_thermist4); // For test
                gcs().send_text(MAV_SEVERITY_INFO, "CCB2 %d,%d,%d,%d,%d", _rx_raw_thermocp1, _rx_raw_thermocp2, _rx_raw_wflow, _rx_raw_bdtemp, _rx_raw_state); // For test
                gcs().send_text(MAV_SEVERITY_INFO, "TX %d RX %d",(uint16_t)(_rtr_tx_cnt & 0xFFFF),(uint16_t)(_handleFrame_cnt & 0xFFFF));
            }
        }

        _AP_COAXCAN1_loop_cnt++;                                  // 10ms period increase
    }
}

// -------------------------------------------------------------------------
// Run
// -------------------------------------------------------------------------
void AP_COAXCAN1::run(void)
{
    //Receive
	RXspin();

	//Transmit
	if(TXspin())
    {
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
        gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 COMMUNICATION_ERROR");
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
                COAXCAN1_RcvrCnt      = 0; // Reset Error Count
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
    uint16_t    uint16_temp = 0U;

//    int16_t int16_temp = 0U;

    switch(can_rxframe.id&can_rxframe.MaskStdID)
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

