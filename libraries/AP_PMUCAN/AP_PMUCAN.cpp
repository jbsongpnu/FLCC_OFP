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
//  ==================================================================================
//  PNU-KAL OFP Firmware version
//  ==================================================================================

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_PMUCAN.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// -------------------------------------------------------------------------
// Define variables for CAM
mavlink_sys_icd_flcc_gcs_pmu_status_t       PMU_Status    = {0};  // MAVLINK Message for PMU Status (KAL)
mavlink_sys_icd_gcs_flcc_pmu_ctrl_echo_t    PMU_Ctrl_Echo = {0};  // MAVLINK Message for PMU Command ECHO (KAL)

AP_PMUCAN::AP_PMUCAN()
{
    _initialized            = false;

    // PNU : must start null - add_interface() reads this before assigning to
    // detect a second bind, so the "multiple interface" guard depends on it.
    // Do not rely on the allocator zeroing it.
    _can_iface              = nullptr;
    _driver_index           = 0;

    RTR_MSG=PMU_BATSTS;

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

    // Initialize RTR ID
    _rtr_id[0]  = PMU_BATSTS    | AP_HAL::CANFrame::FlagEFF | AP_HAL::CANFrame::FlagRTR;
    _rtr_id[1]  = PMU_ENGSTS    | AP_HAL::CANFrame::FlagEFF | AP_HAL::CANFrame::FlagRTR;
    _rtr_id[2]  = PMU_AUX1STS   | AP_HAL::CANFrame::FlagEFF | AP_HAL::CANFrame::FlagRTR;
    _rtr_id[3]  = PMU_AUX2STS   | AP_HAL::CANFrame::FlagEFF | AP_HAL::CANFrame::FlagRTR;
    _rtr_id[4]  = PMU_VERSTS    | AP_HAL::CANFrame::FlagEFF | AP_HAL::CANFrame::FlagRTR;

    // Initialize Command ID
    _cmd_id[CMD_ID::CMD_ID_ENGONOFF]	= PMU_ENGONOFF  | AP_HAL::CANFrame::FlagEFF;
    _cmd_id[CMD_ID::CMD_ID_BATCTRL]		= PMU_BATCTRL   | AP_HAL::CANFrame::FlagEFF;
    _cmd_id[CMD_ID::CMD_ID_ENGMANUAL]	= PMU_ENGMANUAL | AP_HAL::CANFrame::FlagEFF;
    _cmd_id[CMD_ID::CMD_ID_ENGPCL]		= PMU_ENGPCL    | AP_HAL::CANFrame::FlagEFF;
    _cmd_id[CMD_ID::CMD_ID_ENGCHK]		= PMU_ENGCHK    | AP_HAL::CANFrame::FlagEFF;

    // Initialize Etc.
    PMUCAN_Fail_Status      = PMUCAN_STATUS::CONNECTION_FAILURE;
    PMUCAN_Fail_Status_prev = PMUCAN_STATUS::VALUE_END;
    PMUCAN_ErrCnt           = 0;
    PMUCAN_RcvrCnt          = 0;
    PMU_Ctrl_Seq            = 0;    // PNU : must match gcs().PMU_Ctrl_Seq at boot
                                    // so the first real GCS command is detected

    PMU_Ctrl_Echo.PMUCAN_Fail = 2;

    PMU_Status.Version_FLCC_Main_Number = OFP_VER_MAIN;
    PMU_Status.Version_FLCC_Sub_Number  = OFP_VER_SUB;
    PMU_Status.Version_FLCC_REV_Number  = OFP_VER_REV;
}

AP_PMUCAN::~AP_PMUCAN()
{
}

// -------------------------------------------------------------------------
// get_pmucan : Return pmucan from @driver_index or nullptr if it's not ready or doesn't exist
// -------------------------------------------------------------------------
AP_PMUCAN *AP_PMUCAN::get_pmucan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::PMUCAN) {
        return nullptr;
    }
    return static_cast<AP_PMUCAN*>(AP::can().get_driver(driver_index));
}

// -------------------------------------------------------------------------
// add_interface : binds the driver to one CAN interface. Called by AP_CANManager during init, 
// after it allocates the driver for whichever bus has CAN_Dn_PROTOCOL = 15.
// -------------------------------------------------------------------------
bool AP_PMUCAN::add_interface(AP_HAL::CANIface* can_iface) {

    if (_can_iface != nullptr) {
    	hal.console->printf("PMUCAN: Multiple Interface not supported\n\r");
        gcs().send_text(MAV_SEVERITY_WARNING, "PMUCAN: Multiple Interface not supported");
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

    if (!_can_iface->set_event_handle(&sem_handle)) {
    	hal.console->printf("PMUCAN: Cannot add event handle\n\r");
        gcs().send_text(MAV_SEVERITY_WARNING, "PMUCAN: Cannot add event handle");
        return false;
    }
    return true;
}

// -------------------------------------------------------------------------
// init : Initialize PMUCAN loop task
// -------------------------------------------------------------------------
void AP_PMUCAN::init(uint8_t driver_index)
{
    if (_initialized) {
        return;
    }

    if (_can_iface == nullptr) {
        return;
    }

    // PNU : assign only on the call that actually starts the driver, so a second
    // init() cannot repoint it while the loop thread is already running.
    // (currently write-only - kept for parity with the other AP_*CAN drivers)
    _driver_index = driver_index;

    snprintf(_thread_name, sizeof(_thread_name), "pmucan");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PMUCAN::loop, void), _thread_name, 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;

}


// -------------------------------------------------------------------------
// loop : 100Hz Task - 50 Hz CAN, 10Hz GCS TX, 1Hz state report
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

        // PNU : fixed delay AFTER the work, matching AP_KDECAN / AP_PiccoloCAN.
        // The real period is pmucan_period_us plus the body execution time, so
        // the rates below are nominal upper bounds - worst case (select timeout
        // + full RX drain budget + 2 TX timeouts) is ~12.5 ms, i.e. ~80 Hz loop
        // / ~40 Hz CAN. Sized for that, not for exact periodicity.
        hal.scheduler->delay_microseconds(pmucan_period_us);    // nominal 100Hz task

        if(_AP_PMUCAN_loop_cnt%PMUCAN_MINOR_INTERVAL==0)        // nominal 50 Hz CAN
        {
            run();
        }

        if(_AP_PMUCAN_loop_cnt%PMUCAN_MAVLINK_INTERVAL==0)      // nominal 10Hz GCS TX
        {
            send2gcs();     // Send PMU Data to GCS
        }

        if(_AP_PMUCAN_loop_cnt%PMUCAN_ONRUNNING_INTERVAL==0)    // nominal 1Hz state report
        {
            sendchanges();  // Send PMU state to GCS if changed
        }

        _AP_PMUCAN_loop_cnt++;                                  // 10ms period increase
    }
}


// -------------------------------------------------------------------------
// Run : receive and transmit CAN data
// -------------------------------------------------------------------------
void AP_PMUCAN::run(void)
{
    // Receive data 
    RXspin();

	// Transmit data
	TXspin();
}


// -------------------------------------------------------------------------
// send2gcs : Send PMU Status & Command(Echo)
// -------------------------------------------------------------------------
void AP_PMUCAN::send2gcs()
{
    // Update PMU Failsafe Status
    PMU_Ctrl_Echo.PMUCAN_Fail = PMUCAN_Fail_Status;

    // Send Message
    gcs().send_message(MSG_PMU_STATUS);         // Send PMU Status to GCS with Mavlink Message
    gcs().send_message(MSG_PMU_CTRL_ECHO);      // Send PMU Command(Echo) to GCS with Mavlink Message

}

// -------------------------------------------------------------------------
// sendchanges :Send Changed PMU Status
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
// RXspin : parse buffer and process received data
// -------------------------------------------------------------------------
void AP_PMUCAN::RXspin()
{
    uint64_t time, timeout;
    int res = 0;

    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), PMUCAN_SEND_TIMEOUT_US);
    AP_HAL::CANIface::CanIOFlags flags = 0;
    AP_HAL::CANFrame frame;                     // receive frame

    bool read_select    = true;
    bool write_select   = false;
    timeout = timeout_us + AP_HAL::micros64();

    //'Root/AP_HAL_ChibiOS/CanIface.cpp',
    // bool CANIface::select(bool &read, bool &write, const AP_HAL::CANFrame* pending_tx, uint64_t blocking_deadline)
    int ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret) {
        // return if no data is available to read
        // PNU : stay in CONNECTION_FAILURE until the PMU has been seen at least
        // once (events.cpp reads that as "not connected yet" and holds off the
        // failsafe).  After a first successful connect, a select() timeout is a
        // genuine communication error.
        if(PMUCAN_Fail_Status != PMUCAN_STATUS::CONNECTION_FAILURE)
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

            RXdrain(frame, flags);
        }
        else        // Data Not Received
        {
            PMUCAN_ErrCnt = PMUCAN_ErrCnt + 1;          // Increase Error Count

            if(PMUCAN_ErrCnt >= PMUCAN_ERR_MAX_CNT) // Check Max Err Count
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

            if(PMUCAN_RcvrCnt >= PMUCAN_RCVR_MAX_CNT) // Check Max Recv Count
            {
                PMUCAN_Fail_Status  = PMUCAN_STATUS::CONNECTED; // Clear Communication Error Flag with PMU
                PMUCAN_RcvrCnt      = 0; // Reset Error Count
            }

            RXdrain(frame, flags);

        }
        else        // Data Not Received
        {
            if((PMUCAN_RcvrCnt > 0) && (res < 0)) // Check Receive Count
            {
                PMUCAN_RcvrCnt = PMUCAN_RcvrCnt - 1;    // Decrease Receive Count
            }
        }

    }

}



// -------------------------------------------------------------------------
// RXdrain : handle the frame already held in 'frame', then drain whatever else
//           the controller has queued.  Bounded by PMUCAN_RX_MAX_FRAMES and
//           PMUCAN_RX_MAX_TIME_US so it cannot stall the PMUCAN thread. (PNU)
// -------------------------------------------------------------------------
void AP_PMUCAN::RXdrain(AP_HAL::CANFrame& frame, AP_HAL::CANIface::CanIOFlags& flags)
{
    uint64_t time;
    int res = 1;                            // caller guarantees an unhandled frame

    const uint64_t rx_start_us = AP_HAL::micros64();
    uint8_t frame_count = 0;
    //Preventing stuck in a while-loop
    while(res > 0 &&
          frame_count < PMUCAN_RX_MAX_FRAMES &&
          (AP_HAL::micros64() - rx_start_us) < PMUCAN_RX_MAX_TIME_US)
    {
        if (!(flags & AP_HAL::CANIface::Loopback))
        {
            handleFrame(frame);
        }

        res = _can_iface->receive(frame, time, flags);     // Try Receive
        frame_count++;
    }

    // the frame-count / time budget can break the loop with a frame already
    // dequeued but not yet handled - process it rather than drop it
    if (res > 0 && !(flags & AP_HAL::CANIface::Loopback))
    {
        handleFrame(frame);
    }
}

// -------------------------------------------------------------------------
// handleFrame : processes 1 CAN frame data
// -------------------------------------------------------------------------
// PNU-ISSUE(D6) blocked-until: PMU ICD confirmation / bench test.
//   No DLC validation below: every case memcpy's from fixed offsets up to
//   data[7] regardless of can_rxframe.dlc.  data[] is a fixed 8-byte array so
//   there is no out-of-bounds read, but a short or malformed PMU frame is
//   parsed silently and yields stale bytes as battery current, RPM, fuel
//   quantity etc.  See README "Deferred Issues".
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
            // Parse Battery Status - Not Used
            // memcpy(&uint16_temp, &can_rxframe.data[0], 2);
            // PMU_Status.Battery_Status = uint16_temp;

            // Parse Battery Temperature
            memcpy(&int16_temp, &can_rxframe.data[2], 2);
            PMU_Status.Battery_Temp = int16_temp;

            // Parse Baterry Current
            memcpy(&int16_temp, &can_rxframe.data[4], 2);
            PMU_Status.Battery_Current = int16_temp;

            // Battery GAGE - Not Used
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

            // Parse Engine Operating Time(hour) - 24-bit field.
            uint32_temp = 0U;
            memcpy(&uint32_temp, &can_rxframe.data[0], 3);
            PMU_Status.Engine_Hour_Count = uint32_temp;

            // 1-byte reserved here - Not used

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
// TXspin : Transmit data to PMU
// -------------------------------------------------------------------------
void AP_PMUCAN::TXspin()
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

    // send RTR
    pmucan_rtr(_rtr_id[_rtr_idx]);

    _rtr_idx++;
    if(_rtr_idx>=RTR_ID_NUM)
    {
        _rtr_idx=0;
    }

}


// -------------------------------------------------------------------------
// pmucan_cmd : sends command to PMU
// -------------------------------------------------------------------------
int AP_PMUCAN::pmucan_cmd(uint32_t can_id, uint32_t data_cmd)
{
    int cmd_send_res    = 0;
    uint8_t can_data[8] = {0};
    uint8_t msgdlc      = PMUCAN_CMD_DLC;

    memcpy(can_data, &data_cmd, msgdlc);

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::micros64() + PMUCAN_SEND_TIMEOUT_US;                      // Should have timeout value

    out_frame       = {can_id, can_data, msgdlc};                                               // id, data[8], dlc
    cmd_send_res    = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);     // using CANIface::send from libraries/AP_HAL_ChibiOS/CANIface.cpp

    if(cmd_send_res==1)
	{
		//success
		_cmd_tx_cnt++;
	}
	else if(cmd_send_res==0)
	{
        //CMD TX buffer full
		_cmd_tx_err++;
	}
	else
	{
        //CMD TX error
		_cmd_tx_err++;
	}

    return cmd_send_res;
}


// -------------------------------------------------------------------------
// engineonoffstate : two-state engine interlock from toggling command value
// -------------------------------------------------------------------------
// PNU-ISSUE(D7) blocked-until: KGCS TC1 transmit behaviour / bench test.
//   The engine ON/OFF interlock below counts to 10 at ~10 Hz, but the pair it
//   inspects (_pmu_ctrl_cmd / _pmu_ctrl_cmd_prv) only changes when a new TC1
//   arrives - it is STICKY between messages.  Two consequences, and which one
//   applies depends entirely on how KGCS sends TC1:
//     - TC1 sent only on operator action: two messages (e.g. 3 then 2) leave a
//       valid pair frozen, the counter then climbs unattended and the engine
//       STARTS after ~1 s of silence.  Inactivity completes the interlock.
//     - TC1 streamed with a held value: prv == cmd hits the reset branch every
//       time, the counter never passes 1, and the engine can never be commanded
//       OFF.  Failure to stop is the more serious direction.
//   Also rate-dependent (pairs are overwritten above ~10 Hz) and loss-sensitive
//   (a dropped TC1 mid-sequence resets progress).  See README "Deferred Issues"
//   for the 4-case bench matrix.  Test with the engine disconnected.
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
// engineonmode : checks OFF command when engine is ON
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
// engineoffmode : checks ON command when engine is OFF
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
//  pmucan_rtr : requesting transmission for data designated from "can_id" 
//  _rtr_id[5] is predefined, and used as pmucan_rtr(_rtr_id[i])
// -------------------------------------------------------------------------
int AP_PMUCAN::pmucan_rtr(uint32_t can_id)
{
    int rtr_send_res=0;
    uint8_t can_data[8]={0};    //Always dummy zeros

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::micros64() + PMUCAN_SEND_TIMEOUT_US;                      // Should have timeout value

    out_frame       = {can_id, can_data, 8};                                                    // id, data[8], dlc
    rtr_send_res    = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);     // using CANIface::send from libraries/AP_HAL_ChibiOS/CANIface.cpp

    if(rtr_send_res==1)
    {
        //success
        _rtr_tx_cnt++;
    }
    else if(rtr_send_res==0)
    {
        //"RTR TX buffer full
        _rtr_tx_err++;
    }
    else
    {
        //RTR TX error
        _rtr_tx_err++;
    }

    return rtr_send_res;    // PNU : was unconditionally 0; match pmucan_cmd()
}
