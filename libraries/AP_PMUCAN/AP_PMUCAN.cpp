/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Eugene Shamaev, Siddharth Bharat Purohit
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_PMUCAN.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>


extern const AP_HAL::HAL& hal;

//#define debug_pmucan(level_debug, fmt, args...) do { if ((level_debug) <= AP::pmucan().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

// this is the timeout in milliseconds for periodic message types. We
// set this to 1 to minimise resend of stale msgs
//#define CAN_PERIODIC_TX_TIMEOUT_MS 2
#define TEMP_EXP_INITVAL 0		//Initial value

// JBSong - table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_PMUCAN::var_info[] = {
    // @Param: example
    // @DisplayName: User example
    // @Description: Example for user
    AP_GROUPINFO("Examp", 1, AP_PMUCAN, _examp, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM1", 1, AP_PMUCAN, _pmu_param1, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM2", 2, AP_PMUCAN, _pmu_param2, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM3", 3, AP_PMUCAN, _pmu_param3, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM4", 4, AP_PMUCAN, _pmu_param4, TEMP_EXP_INITVAL),

    AP_GROUPEND
};


// -------------------------------------------------------------------------
// Define variables for CAM
mavlink_sys_icd_flcc_gcs_pmu_status_t       PMU_Status    = {0};  // MAVLINK Message for PMU Status (KAL)
mavlink_sys_icd_gcs_flcc_pmu_ctrl_echo_t    PMU_Ctrl_Echo = {0};  // MAVLINK Message for PMU Command ECHO (KAL)


AP_PMUCAN::AP_PMUCAN()
{
    AP_Param::setup_object_defaults(this, var_info);	//JBSong =>singleton-type object initialization
    // _examp = TEMP_EXP_INITVAL;						//JBSong => trivial example variable // It makes error! BW
    _examp.set_default(0);				//JBSong => trivial example variable // Correct grammar

    // _pmu_param1.set_default(TEMP_EXP_INITVAL+4);
    // _pmu_param2.set_default(TEMP_EXP_INITVAL+1);
    // _pmu_param3.set_default(TEMP_EXP_INITVAL+2);
    // _pmu_param4.set_default(TEMP_EXP_INITVAL+3);


    _initialized            = false;
    _iface                  = nullptr;

	//_driver=nullptr;
	RTR_MSG=PMU_BATSTS;

    // _perf_loop_elapse       = nullptr; // KAL 23.05.23 -- REMOVED 
    // _perf_err_cnt           = nullptr;
    // _perf_send_err_cnt      = nullptr;
    // _perf_send_num_cnt      = nullptr;
    // _perf_rcv_err_cnt       = nullptr;
    // _perf_rcv_num_cnt       = nullptr;
    // _perf_no_mailbox_cnt    = nullptr;
    // _perf_rtr_tx_elapse     = nullptr;
    // _perf_cmd_tx_elapse     = nullptr;
    // _perf_rx_elapse         = nullptr;

    _handleFrame_cnt        = 0;
    _rtr_tx_cnt             = 0;
    _cmd_tx_cnt             = 0;
    _rtr_tx_err             = 0;
    _cmd_tx_err             = 0;

    _rtr_idx                = 0;
    _cmd_idx                = CMD_ID::CMD_ID_BATCTRL;       // _cmd_idx=0;

    _engineonoffmode        = 0;                            // OFF
    _engineoncnt            = 0;
    _engineoffcnt           = 0;

    _pmucan_last_send_us    =0;

    pmucan_period_us        = 1000000UL / PMUCAN_LOOP_HZ;	// 10ms

    // Initialize Performance Info.
    // _perf_loop_elapse       = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED,    "PMUCAN_LOOP_TIME"); // KAL 23.05.23 -- REMOVED 
    // _perf_rtr_tx_elapse     = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED,    "PMUCAN_RTR_TX_TIME");
    // _perf_cmd_tx_elapse     = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED,    "PMUCAN_CMD_TX_TIME");
    // _perf_rx_elapse         = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED,    "PMUCAN_RX_TIME");
    // _perf_err_cnt           = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT,      "PMUCAN_ERR_CNT");
    // _perf_send_err_cnt      = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT,      "PMUCAN_SEND_ERR_CNT");
    // _perf_rcv_err_cnt       = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT,      "PMUCAN_RCV_ERR_CNT");
    // _perf_send_num_cnt      = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT,      "PMUCAN_SEND_NUM_CNT");
    // _perf_rcv_num_cnt       = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT,      "PMUCAN_RCV_NUM_CNT");
    // _perf_no_mailbox_cnt    = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT,      "PMUCAN_NO_SEND_MAILBOX_CNT");

    // Initialize RTR ID
    _rtr_id[0]  = PMU_BATSTS    | pmucan::CanFrame::FlagEFF | pmucan::CanFrame::FlagRTR;
    _rtr_id[1]  = PMU_ENGSTS    | pmucan::CanFrame::FlagEFF | pmucan::CanFrame::FlagRTR;
    _rtr_id[2]  = PMU_AUX1STS   | pmucan::CanFrame::FlagEFF | pmucan::CanFrame::FlagRTR;
    _rtr_id[3]  = PMU_AUX2STS   | pmucan::CanFrame::FlagEFF | pmucan::CanFrame::FlagRTR;
    _rtr_id[4]  = PMU_VERSTS    | pmucan::CanFrame::FlagEFF | pmucan::CanFrame::FlagRTR;

    // Initialize Command ID
    _cmd_id[CMD_ID::CMD_ID_ENGONOFF]	= PMU_ENGONOFF  | pmucan::CanFrame::FlagEFF;
    _cmd_id[CMD_ID::CMD_ID_BATCTRL]		= PMU_BATCTRL   | pmucan::CanFrame::FlagEFF;
    _cmd_id[CMD_ID::CMD_ID_ENGMANUAL]	= PMU_ENGMANUAL | pmucan::CanFrame::FlagEFF;
    _cmd_id[CMD_ID::CMD_ID_ENGPCL]		= PMU_ENGPCL    | pmucan::CanFrame::FlagEFF;
    _cmd_id[CMD_ID::CMD_ID_ENGCHK]		= PMU_ENGCHK    | pmucan::CanFrame::FlagEFF;

    // Initialize Etc.
    PMUCAN_Fail_Status      = PMUCAN_STATUS::CONNECTION_FAILURE;
    PMUCAN_Fail_Status_prev = PMUCAN_STATUS::VALUE_END;
    PMUCAN_ErrCnt           = 0;
    PMUCAN_RcvrCnt          = 0;

    PMU_Ctrl_Echo.PMUCAN_Fail = 2;

    PMU_Status.Version_FLCC_Main_Number = OFP_VER_MAIN;
    PMU_Status.Version_FLCC_Sub_Number  = OFP_VER_SUB;
    PMU_Status.Version_FLCC_REV_Number  = OFP_VER_REV;
}


AP_PMUCAN::~AP_PMUCAN()
{
}

// KAL 23.05.23
// -------------------------------------------------------------------------
// get_pmucan : Return pmucan from @driver_index or nullptr if it's not ready or doesn't exist
//'Driver_Type_PMUCAN' : defined in AP_CANManager.h 
// -------------------------------------------------------------------------
AP_PMUCAN *AP_PMUCAN::get_pmucan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_PMUCAN) {
        return nullptr;
    }
    return static_cast<AP_PMUCAN*>(AP::can().get_driver(driver_index));
}


// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
bool AP_PMUCAN::add_interface(AP_HAL::CANIface* can_iface) {

    if (_can_iface != nullptr) {
    	hal.console->printf("PMUCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
    	hal.console->printf("PMUCAN: CAN driver not found\n\r");
        gcs().send_text(MAV_SEVERITY_WARNING, "PMUCAN: CAN driver not found");
        return false;
    }

    if (!_can_iface->is_initialized()) {
    	hal.console->printf("PMUCAN: Driver not initialized\n\r");
        gcs().send_text(MAV_SEVERITY_WARNING, "PMUCAN: Driver not initialized");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
    	hal.console->printf("PMUCAN: Cannot add event handle\n\r");
        gcs().send_text(MAV_SEVERITY_WARNING, "PMUCAN: Cannot add event handle");
        return false;
    }
    return true;
}


// -------------------------------------------------------------------------
// Initialize PMUCAN bus
// -------------------------------------------------------------------------
void AP_PMUCAN::init(uint8_t driver_index, bool enable_filters)
{
	_driver_index = driver_index;

    if (_initialized) {
        return;
    }

    if (_can_iface == nullptr) {
        return;
    }

    snprintf(_thread_name, sizeof(_thread_name), "pmucan");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PMUCAN::loop, void), _thread_name, 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;


    // gcs().send_text(MAV_SEVERITY_INFO, "[PMU] Initialized");
}


// -------------------------------------------------------------------------
// Task
// -------------------------------------------------------------------------
void AP_PMUCAN::loop(void)
{
    while (true)
    {
        if (!_initialized)
        {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        hal.scheduler->delay_microseconds(pmucan_period_us);    // 10ms period loop

        if(_AP_PMUCAN_loop_cnt%PMUCAN_MINOR_INTERVAL==0)        // 20ms period run
            run();
            
        if(_AP_PMUCAN_loop_cnt%PMUCAN_MALVINK_INTERVAL==0)      // 100ms period send2ppc
        {
            send2gcs();     // Send PMU Data to GCS
            // gcs().send_text(MAV_SEVERITY_INFO, "[PMU] send2gcs"); // KAL
        }

        if(_AP_PMUCAN_loop_cnt%PMUCAN_ONRUNNING_INTERVAL==0)      // 2 sec period KAL
        {
        //     gcs().send_text(MAV_SEVERITY_INFO, "[PMU] %8ld V%5d %5d", PMU_Status.Date, PMU_Status.System_Voltage*100, PMU_Status.PMU_Status); // KAL
            sendchanges();
        }

        _AP_PMUCAN_loop_cnt++;                                  // 10ms period increase
    }
}


// -------------------------------------------------------------------------
// Run
// -------------------------------------------------------------------------
void AP_PMUCAN::run(void)
{
	//perf
	// hal.util->perf_begin(_perf_loop_elapse);// KAL 23.05.23 -- REMOVED 

	//RECEIVE
	// hal.util->perf_begin(_perf_rx_elapse);// KAL 23.05.23 -- REMOVED 
	RXspin();
	// hal.util->perf_end(_perf_rx_elapse);// KAL 23.05.23 -- REMOVED 

	//TRANSMIT
	TXspin();

	//perf
	// hal.util->perf_end(_perf_loop_elapse); // KAL 23.05.23 -- REMOVED 
}


// -------------------------------------------------------------------------
// Send PMU Status & Command(Echo) // KAL
// -------------------------------------------------------------------------
void AP_PMUCAN::send2gcs()
{
    // Update PMU Failsafe Status
    PMU_Ctrl_Echo.PMUCAN_Fail = PMUCAN_Fail_Status;

    // Send Message
    gcs().send_message(MSG_PMU_STATUS);         // Send PMU Status to GCS with Mavlink Message (KAL)
    gcs().send_message(MSG_PMU_CTRL_ECHO);      // Send PMU Command(Echo) to GCS with Mavlink Message (KAL)

}

// -------------------------------------------------------------------------
// Send Some Change of PMU Status
// -------------------------------------------------------------------------
void AP_PMUCAN::sendchanges()
{
    bool changes = (PMUCAN_Fail_Status_prev != PMUCAN_Fail_Status);
    if (changes == true){
        switch (PMUCAN_Fail_Status)
        {
        case PMUCAN_STATUS::CONNECTED:
            gcs().send_text(MAV_SEVERITY_INFO,      "[PMU] Connected!");
            break;
        case PMUCAN_STATUS::COMMUNICATION_ERROR:
            gcs().send_text(MAV_SEVERITY_WARNING,   "[PMU] Communication Error");
            break;
        case PMUCAN_STATUS::CONNECTION_FAILURE:
            gcs().send_text(MAV_SEVERITY_WARNING,   "[PMU] Connection Failure!");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_WARNING,   "[PMU] WRONG STATUS");
        }
    }
    PMUCAN_Fail_Status_prev = PMUCAN_Fail_Status;
}


// -------------------------------------------------------------------------
// JBSong - return type changed to void
// -------------------------------------------------------------------------
void AP_PMUCAN::RXspin()
{
    uint64_t time, timeout;
    int res = 0;

    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), PMUCAN_SEND_TIMEOUT_US);
    AP_HAL::CANIface::CanIOFlags flags = 0;
    AP_HAL::CANFrame frame;                     // receive frame

    // wait for space in buffer to read
    bool read_select    = true;
    bool write_select   = false;
    timeout = timeout_us + AP_HAL::micros64();

    //'Root/AP_HAL_ChibiOS/CanIface.cpp',
    // bool CANIface::select(bool &read, bool &write, const AP_HAL::CANFrame* pending_tx, uint64_t blocking_deadline)
    int ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret) {
        // return if no data is available to read
        if(PMUCAN_Fail_Status==PMUCAN_STATUS::CONNECTION_FAILURE)
        {
            PMUCAN_Fail_Status = PMUCAN_STATUS::CONNECTION_FAILURE;
        }
        else
        {
            PMUCAN_Fail_Status = PMUCAN_STATUS::COMMUNICATION_ERROR;
        }

        return;
    }


    if(PMUCAN_Fail_Status == PMUCAN_STATUS::CONNECTED)     // Normal Connection with PMU
    {
        res = _can_iface->receive(frame, time, flags);

        if(res > 0) // Data Received Normaly
        {
            if(PMUCAN_ErrCnt > 0) // Check Error Count
            {
                PMUCAN_ErrCnt = PMUCAN_ErrCnt - 1;    // Decrease Error Count
            }

            while(res > 0)
            {
                if (flags & pmucan::CanIOFlagLoopback)
                {
//                    Not Used
                }
                else
                {
                    // hal.util->perf_count(_perf_rcv_num_cnt); // KAL 23.05.23 -- REMOVED 
                    handleFrame(frame);
                }

                res = _can_iface->receive(frame, time, flags);     // Try Receive
            }
        }
        else        // Data Not Received
        {
            // hal.util->perf_count(_perf_rcv_err_cnt); // KAL 23.05.23 -- REMOVED 
            PMUCAN_ErrCnt = PMUCAN_ErrCnt + 1;          // Increase Error Count

            if(PMUCAN_ErrCnt == 5) // Check Max Err Count
            {
                PMUCAN_Fail_Status  = PMUCAN_STATUS::COMMUNICATION_ERROR; // Set Communication Error Flag with PMU
                PMUCAN_ErrCnt       = 0; // Reset Error Count
            }
        }
    }
    else                            // Abnormal Connection with PMU
    {
        res = _can_iface->receive(frame, time, flags);

        if(res > 0) // Data Received Normaly
        {
            PMUCAN_RcvrCnt = PMUCAN_RcvrCnt + 1;    // Increase Receive Count

            if(PMUCAN_RcvrCnt == 5) // Check Max Recv Count
            {
                PMUCAN_Fail_Status  = PMUCAN_STATUS::CONNECTED; // Clear Communication Error Flag with PMU
                PMUCAN_RcvrCnt      = 0; // Reset Error Count
            }

            while(res > 0)
            {
                if (flags & pmucan::CanIOFlagLoopback)
                {
//                    Not Used
                }
                else
                {
                    // hal.util->perf_count(_perf_rcv_num_cnt); // KAL 23.05.23 -- REMOVED 
                    handleFrame(frame);
                }

                res = _can_iface->receive(frame, time, flags);     // Try Receive
            }

        }
        else        // Data Not Received
        {
            // hal.util->perf_count(_perf_rcv_err_cnt); // KAL 23.05.23 -- REMOVED 

            if((PMUCAN_RcvrCnt > 0) & (res < 0)) // Check Receive Count
            {
                PMUCAN_RcvrCnt = PMUCAN_RcvrCnt - 1;    // Decrease Receive Count
            }
        }

    }

}


// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
void AP_PMUCAN::handleFrame(const AP_HAL::CANFrame& can_rxframe)
{
    uint8_t     uint8_temp  = 0U;
    uint16_t    uint16_temp = 0U;
    uint32_t    uint32_temp = 0U;

    int8_t  int8_temp  = 0U;
    int16_t int16_temp = 0U;

    switch(can_rxframe.id&can_rxframe.MaskExtID)
    {
        case PMU_BATSTS:
            // Parse Battery Status
            memcpy(&uint16_temp, &can_rxframe.data[0], 2);
            PMU_Status.Battery_Status = uint16_temp;

            // Parse Battery Temperature
            memcpy(&int16_temp, &can_rxframe.data[2], 2);
            PMU_Status.Battery_Temp = int16_temp;

            // Parse Baterry Current
            memcpy(&int16_temp, &can_rxframe.data[4], 2);
            PMU_Status.Battery_Current = int16_temp;

            // Not Used
            memcpy(&int16_temp, &can_rxframe.data[6], 2);


            _handleFrame_cnt++;

            break;

        case PMU_ENGSTS:

            // Parse Engine Head Temperature #1
            memcpy(&int16_temp, &can_rxframe.data[0], 2);
            PMU_Status.Engine_Head1_Temp = int16_temp;

            // Parse Engine Head Temperature #2
            memcpy(&int16_temp, &can_rxframe.data[2], 2);
            PMU_Status.Engine_Head2_Temp = int16_temp;

            // Parse Engine RPM
            memcpy(&uint16_temp, &can_rxframe.data[4], 2);
            PMU_Status.Engine_RPM = uint16_temp;

            // Parse Fuel Quentity
            memcpy(&int8_temp, &can_rxframe.data[6], 1);
            PMU_Status.Fuel_Quantity = int8_temp;

            // Parse Throtle Position(PCL)
            memcpy(&int8_temp, &can_rxframe.data[7], 1);
            PMU_Status.Throttle_Position_Report = int8_temp;

            _handleFrame_cnt++;

            break;

        case PMU_AUX1STS:

            // Parse Current Contol Command
            memcpy(&int16_temp, &can_rxframe.data[0], 2);
            PMU_Status.Current_Control_Command = int16_temp;

            // Parse System Load Current
            memcpy(&int16_temp, &can_rxframe.data[2], 2);
            PMU_Status.Load_Current = int16_temp;

            // Parse System Voltage
            memcpy(&int16_temp, &can_rxframe.data[4], 2);
            PMU_Status.System_Voltage = int16_temp;

            // Parse Battery Quentity
            memcpy(&int16_temp, &can_rxframe.data[6], 2);
            PMU_Status.Battery_Quantity_Command = int16_temp;

            _handleFrame_cnt++;

            break;

        case PMU_AUX2STS:

            // Parse Engine Operating Time(hour)
            memcpy(&uint32_temp, &can_rxframe.data[0], 3);
            PMU_Status.Engine_Hour_Count = uint32_temp;

            // Parse PMU Status
            memcpy(&uint16_temp, &can_rxframe.data[4], 2);
            PMU_Status.PMU_Status = uint16_temp;

            // Parse PMU Temperature
            memcpy(&int16_temp, &can_rxframe.data[6], 2);
            PMU_Status.PMU_Temp = int16_temp;

            _handleFrame_cnt++;

            break;

        case PMU_VERSTS:

            // Parse Date Information
            memcpy(&uint32_temp, &can_rxframe.data[0], 4);
            PMU_Status.Date = uint32_temp;

            // Parse PMU SW version(sub)
            memcpy(&uint8_temp, &can_rxframe.data[4], 1);
            PMU_Status.Version_Sub_Number = uint8_temp;

            // Parse PMU SW version(main)
            memcpy(&uint8_temp, &can_rxframe.data[5], 1);
            PMU_Status.Version_Main_Number = uint8_temp;

            _handleFrame_cnt++;

            break;

        default:

            break;
    }
}


// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
int AP_PMUCAN::TXspin()
{
    // Update New Command
    if (PMU_Ctrl_Seq == gcs().PMU_Ctrl_Seq)
    {
        // Not Received
    }
    else
    {
        // Save Previous Command
        _pmu_ctrl_cmd_prv.Engine_OnOff          = _pmu_ctrl_cmd.Engine_OnOff;
        _pmu_ctrl_cmd_prv.Battery_Control_CMD   = _pmu_ctrl_cmd.Battery_Control_CMD;
        _pmu_ctrl_cmd_prv.Engine_Manual         = _pmu_ctrl_cmd.Engine_Manual;
        _pmu_ctrl_cmd_prv.Engine_Throttle_CMD   = _pmu_ctrl_cmd.Engine_Throttle_CMD;
        _pmu_ctrl_cmd_prv.Engine_CHK_CMD        = _pmu_ctrl_cmd.Engine_CHK_CMD;

        // Update GCS Command
        _pmu_ctrl_cmd.Engine_OnOff              = gcs().PMU_Ctrl.Engine_OnOff;
        _pmu_ctrl_cmd.Battery_Control_CMD       = gcs().PMU_Ctrl.Battery_Control_CMD;
        _pmu_ctrl_cmd.Engine_Manual             = gcs().PMU_Ctrl.Engine_Manual;
        _pmu_ctrl_cmd.Engine_Throttle_CMD       = gcs().PMU_Ctrl.Engine_Throttle_CMD;
        _pmu_ctrl_cmd.Engine_CHK_CMD            = gcs().PMU_Ctrl.Engine_CHK_CMD;

        // Update Sequence Number
        PMU_Ctrl_Seq = gcs().PMU_Ctrl_Seq;
    }


    // Send PMU Control Command to PMU
    // hal.util->perf_begin(_perf_cmd_tx_elapse);  // KAL 23.05.23 -- REMOVED 

    switch(_cmd_idx)
    {
        case CMD_ID::CMD_ID_BATCTRL:
            if(pmucan_cmd(_cmd_id[CMD_ID::CMD_ID_BATCTRL], _pmu_ctrl_cmd.Battery_Control_CMD)>0)        // 0~9, 10~100%
            {
                PMU_Ctrl_Echo.Battery_Control_CMD_Echo = _pmu_ctrl_cmd.Battery_Control_CMD;
            }
            break;

        case CMD_ID::CMD_ID_ENGONOFF:
            engineonoffstate();

            break;

        case CMD_ID::CMD_ID_ENGMANUAL:
            if(pmucan_cmd(_cmd_id[CMD_ID::CMD_ID_ENGMANUAL], _pmu_ctrl_cmd.Engine_Manual)>0)            // 1: MANUAL 0: AUTO
            {
                PMU_Ctrl_Echo.Engine_Manual_Echo = _pmu_ctrl_cmd.Engine_Manual;
            }
            break;

        case CMD_ID::CMD_ID_ENGPCL:
            if(pmucan_cmd(_cmd_id[CMD_ID::CMD_ID_ENGPCL], _pmu_ctrl_cmd.Engine_Throttle_CMD)>0)         // 0~100%
            {
                PMU_Ctrl_Echo.Engine_Throttle_CMD_Echo = _pmu_ctrl_cmd.Engine_Throttle_CMD;
            }
            break;

        case CMD_ID::CMD_ID_ENGCHK:
            if(pmucan_cmd(_cmd_id[CMD_ID::CMD_ID_ENGCHK], _pmu_ctrl_cmd.Engine_CHK_CMD)>0)              // 1: close 0: open
            {
                PMU_Ctrl_Echo.Engine_CHK_CMD_Echo = _pmu_ctrl_cmd.Engine_CHK_CMD;
            }
            break;

        default:

            break;
    }

    _cmd_idx++;
    if(_cmd_idx>=CMD_ID::CMD_ID_NUM)
    {
        _cmd_idx=CMD_ID::CMD_ID_BATCTRL;	//_cmd_idx=0;
    }

    // hal.util->perf_end(_perf_cmd_tx_elapse); // KAL 23.05.23 -- REMOVED 

    /*send RTR*/
    // hal.util->perf_begin(_perf_rtr_tx_elapse); // KAL 23.05.23 -- REMOVED 
    pmucan_rtr(_rtr_id[_rtr_idx]);

    _rtr_idx++;
    if(_rtr_idx>=RTR_ID_NUM)
    {
        _rtr_idx=0;
    }

    // hal.util->perf_end(_perf_rtr_tx_elapse); // KAL 23.05.23 -- REMOVED 

	return 0;//dummy_res;
}


// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
int AP_PMUCAN::pmucan_cmd(uint32_t can_id, uint32_t data_cmd)
{
    int cmd_send_res    = 0;
    uint8_t can_data[8] = {0};
    uint8_t msgdlc      = PMUCAN_CMD_DLC;

    memcpy(can_data, &data_cmd, msgdlc);

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::native_micros64() + PMUCAN_SEND_TIMEOUT_US;                      // Should have timeout value

    out_frame       = {can_id, can_data, msgdlc};                                               // id, data[8], dlc
    cmd_send_res    = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);     // using CANIface::send from libraries/AP_HAL_ChibiOS/CANIface.cpp

    if(cmd_send_res==1)
	{
		//success
		// hal.util->perf_count(_perf_send_num_cnt); // KAL 23.05.23 -- REMOVED 
		_cmd_tx_cnt++;
	}
	else if(cmd_send_res==0)
	{
		// hal.util->perf_count(_perf_no_mailbox_cnt); // KAL 23.05.23 -- REMOVED 
		_cmd_tx_err++;
		//CMD TX buffer full
	}
	else
	{
		// hal.util->perf_count(_perf_send_err_cnt); // KAL 23.05.23 -- REMOVED 
		_cmd_tx_err++;
		//CMD TX error
	}

    return cmd_send_res;
}


// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
void AP_PMUCAN::engineonoffstate(void)
{
    if (_engineonoffmode==0U)	// OFF STATE
    {
        if(_engineoncnt>=10)        // transition to ON STATE
        {
            _engineonoffmode    = 1U;                               // ON
            _engineoncnt        = 0U;
            if(pmucan_cmd(_cmd_id[CMD_ID::CMD_ID_ENGONOFF], 1U)>0)  // ON(1)
            {
                PMU_Ctrl_Echo.Engine_OnOff_Echo = 1;
            }
            engineonmode();
        }
        else                        //during in OFF STATE
        {
            engineoffmode();
        }
    }
    else                        // ON STATE
    {
        if(_engineoffcnt>=10)       //transition to OFF STATE
        {
            _engineonoffmode    = 0U;                               // OFF
            _engineoffcnt       = 0U;
            if(pmucan_cmd(_cmd_id[CMD_ID::CMD_ID_ENGONOFF], 0U)>0)  // OFF(0)
            {
                PMU_Ctrl_Echo.Engine_OnOff_Echo = 0;
            }
            engineoffmode();
        }
        else                        // during in ON STATE
        {
            engineonmode();
        }
    }
}


// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
void AP_PMUCAN::engineonmode(void)
{
    if(_pmu_ctrl_cmd.Engine_OnOff==0b100)
    {
        if(_pmu_ctrl_cmd_prv.Engine_OnOff==0b101)
        {
            _engineoffcnt++;
        }
        else
        {
            _engineoffcnt=0;
        }
    }
    else if(_pmu_ctrl_cmd.Engine_OnOff==0b101)
    {
        if(_pmu_ctrl_cmd_prv.Engine_OnOff==0b100)
        {
            _engineoffcnt++;
        }
        else
        {
            _engineoffcnt=0;
        }
    }
    else
    {
        _engineoffcnt=0;
    }
}


// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
void AP_PMUCAN::engineoffmode(void)
{
    if(_pmu_ctrl_cmd.Engine_OnOff==0b10)
    {
        if(_pmu_ctrl_cmd_prv.Engine_OnOff==0b11)
        {
            _engineoncnt++;
        }
        else
        {
            _engineoncnt=0;
        }
    }
    else if(_pmu_ctrl_cmd.Engine_OnOff==0b11)
    {
        if(_pmu_ctrl_cmd_prv.Engine_OnOff==0b10)
        {
            _engineoncnt++;
        }
        else
        {
            _engineoncnt=0;
        }
    }
    else
    {
        _engineoncnt=0;
    }
}


// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
int AP_PMUCAN::pmucan_rtr(uint32_t can_id)
{
    int rtr_send_res=0;
    uint8_t can_data[8]={0};

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::native_micros64() + PMUCAN_SEND_TIMEOUT_US;                      // Should have timeout value

    out_frame       = {can_id, can_data, 8};                                                    // id, data[8], dlc
    rtr_send_res    = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);     // using CANIface::send from libraries/AP_HAL_ChibiOS/CANIface.cpp

    if(rtr_send_res==1)
    {
        //success
        // hal.util->perf_count(_perf_send_num_cnt); // KAL 23.05.23 -- REMOVED 
        _rtr_tx_cnt++;
    }
    else if(rtr_send_res==0)
    {
        // hal.util->perf_count(_perf_no_mailbox_cnt); // KAL 23.05.23 -- REMOVED 
        _rtr_tx_err++;
        //"RTR TX buffer full
    }
    else
    {
        // hal.util->perf_count(_perf_send_err_cnt); // KAL 23.05.23 -- REMOVED 
        _rtr_tx_err++;
        //RTR TX error
    }

    return 0;
}


