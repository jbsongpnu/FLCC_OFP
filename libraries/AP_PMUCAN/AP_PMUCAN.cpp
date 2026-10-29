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

// -------------------------------------------------------------------------
// Define variables for CAM
//
// Defined OUTSIDE the AP_PNU_CAN_ENABLED guard: GCS_PNU.cpp and ArduCopter/events.cpp
// reference them unconditionally, so with the CAN driver compiled out they keep their
// static-init values - exactly the "no PMU fitted" case the failsafe already handles.
mavlink_sys_icd_flcc_gcs_pmu_status_t       PMU_Status    = {0};  // MAVLINK Message for PMU Status (KAL)
mavlink_sys_icd_gcs_flcc_pmu_ctrl_echo_t    PMU_Ctrl_Echo = {0};  // MAVLINK Message for PMU Command ECHO (KAL)

#if AP_PNU_CAN_ENABLED

extern const AP_HAL::HAL& hal;

AP_PMUCAN::AP_PMUCAN(AP_PNU_CAN &frontend) :
    AP_PNU_CAN_Backend(frontend)
{
    _handleFrame_cnt        = 0;
    _rtr_tx_cnt             = 0;
    _cmd_tx_cnt             = 0;
    _rtr_tx_err             = 0;
    _cmd_tx_err             = 0;
    _short_frame_cnt            = 0;
    _last_short_frame_warn_ms   = 0;


    _engineonoffmode        = 0;                            // OFF
    _engineoncnt            = 0;
    _engineoffcnt           = 0;

    // PNU-ISSUE(D1)/(D7) step 2c : zero means "no TC1 and no Stop seen this boot", which
    // is what refuses the very first Start of a power cycle until a Stop arrives.
    _last_tc1_ms            = 0;
    _last_stop_pair_ms      = 0;
    _start_pair_first_ms    = 0;
    _stop_window_armed      = false;
    _last_start_refused_ms  = 0;

    _pmu_frames_seen        = 0;
    _last_pmu_frame_ms      = 0;

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
// init : validate and prepare.  No thread and no interface - AP_PNU_CAN owns both.
// -------------------------------------------------------------------------
bool AP_PMUCAN::init(void)
{
    return true;
}


// -------------------------------------------------------------------------
// update : the driver's tick, at AP_PNU_CAN_LOOP_HZ
// -------------------------------------------------------------------------
// The cadence below reproduces the pre-5.1.15 behaviour exactly, but derived from
// PMUCAN_MSG_RATE_HZ instead of emerging from a 50 Hz rotation nobody had written down.
// PNU-ISSUE(D7)'s engine interlock depends on engineonoffstate() being reached at 10 Hz;
// that is now a stated requirement rather than an accident, and it must stay one.
// -------------------------------------------------------------------------
void AP_PMUCAN::update(uint32_t tick)
{
    if ((tick % PMUCAN_HEALTH_CHECK_TICKS) == 0) {      // nominal 50 Hz
        check_health();
    }

    // one of the five command/RTR slots per stride, so each message goes out at exactly
    // PMUCAN_MSG_RATE_HZ
    const uint32_t stride = PMUCAN_SLOT_PERIOD_TICKS / CMD_ID::CMD_ID_NUM;
    const uint32_t phase  = tick % PMUCAN_SLOT_PERIOD_TICKS;
    if ((stride > 0) && ((phase % stride) == 0)) {
        transmit_slot(uint8_t(phase / stride));
    }

    if ((tick % PMUCAN_MAVLINK_INTERVAL) == 0) {        // nominal 10 Hz
        send2gcs();
    }

    if ((tick % PMUCAN_ONRUNNING_INTERVAL) == 0) {      // nominal 1 Hz
        sendchanges();
    }
}


// -------------------------------------------------------------------------
// check_health : leaky bucket, fed by RECOGNISED PMU STATUS FRAMES
// -------------------------------------------------------------------------
// This is the PNU-ISSUE(D6) fix that the shared bus forced.  The old RXspin() treated any
// successful receive as proof of life, so on a bus shared with the gimbal a dead PMU would
// have read as CONNECTED forever and failsafe_pmucan_check() would never have fired.
// -------------------------------------------------------------------------
void AP_PMUCAN::check_health(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool pmu_seen = (_pmu_frames_seen > 0) &&
                          ((now_ms - _last_pmu_frame_ms) <= PMUCAN_FRAME_TIMEOUT_MS);
    _pmu_frames_seen = 0;

    if (PMUCAN_Fail_Status == PMUCAN_STATUS::CONNECTED) {
        if (pmu_seen) {
            if (PMUCAN_ErrCnt > 0) {
                PMUCAN_ErrCnt--;
            }
        } else {
            PMUCAN_ErrCnt++;
            if (PMUCAN_ErrCnt >= PMUCAN_ERR_MAX_CNT) {
                PMUCAN_Fail_Status = PMUCAN_STATUS::COMMUNICATION_ERROR;
                PMUCAN_ErrCnt = 0;
            }
        }
        return;
    }

    if (pmu_seen) {
        PMUCAN_RcvrCnt++;
        if (PMUCAN_RcvrCnt >= PMUCAN_RCVR_MAX_CNT) {
            PMUCAN_Fail_Status = PMUCAN_STATUS::CONNECTED;
            PMUCAN_RcvrCnt = 0;
        }
    } else if (PMUCAN_RcvrCnt > 0) {
        PMUCAN_RcvrCnt--;
    }
}


bool AP_PMUCAN::healthy(void) const
{
    return PMUCAN_Fail_Status == PMUCAN_STATUS::CONNECTED;
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


// PNU-ISSUE(D6) RESOLVED: the PMU ICD specifies DLC 8 for all five status
//   messages, so anything shorter is rejected below rather than parsed from the
//   same fixed offsets - which previously yielded stale bytes as battery
//   current, RPM, fuel quantity etc.  Drops are counted and reported.
// PNU-ISSUE(D15) RESOLVED: the PMU ICD specifies little-endian (LSB first), which
//   matches the STM32, so every field below is read by memcpy straight into a
//   native integer with no byte swap.  Correct as written - do not "fix" it with
//   be16toh/be32toh.
bool AP_PMUCAN::handle_frame(const AP_HAL::CANFrame& can_rxframe)
{
    uint8_t     uint8_temp  = 0U;
    uint16_t    uint16_temp = 0U;
    uint32_t    uint32_temp = 0U;

    int8_t  int8_temp  = 0U;
    int16_t int16_temp = 0U;

    // The PMU speaks 29-bit EXTENDED frames throughout; the gimbal and the SAR are 11-bit
    // standard.  Returning false rather than true for anything else is what lets them
    // share one bus - the driver offers each frame to every backend until one claims it.
    if (!can_rxframe.isExtended() || can_rxframe.isRemoteTransmissionRequest() ||
        can_rxframe.isErrorFrame()) {
        return false;
    }

    const uint32_t frame_id = can_rxframe.id & can_rxframe.MaskExtID;

    // PNU-ISSUE(D6) : every field parsed below sits within the ICD's 8-byte
    // payload, so a short frame means the tail bytes are not ours to read.
    switch(frame_id)
    {
        case PMU_BATSTS:
        case PMU_ENGSTS:
        case PMU_AUX1STS:
        case PMU_AUX2STS:
        case PMU_VERSTS:
            if (can_rxframe.dlc < PMUCAN_STS_DLC) {
                // still OUR frame, so it is claimed - reporting it twice, once here and
                // once as the driver's unclaimed-frame warning, would be worse
                report_short_frame(frame_id, can_rxframe.dlc);
                return true;
            }
            break;

        default:
            return false;                               // not a PMU status frame
    }

    // a recognised PMU status frame: this, and only this, is proof the PMU is alive
    _pmu_frames_seen++;
    _last_pmu_frame_ms = AP_HAL::millis();

    switch(frame_id)
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
            // PNU-ISSUE(D15) : the zeroing is load-bearing.  LSB-first means the
            // three bytes land in the low three of the uint32, so the top byte
            // must already be 0 - it is never written by the memcpy below.
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

    // claimed: the id was one of ours, so the driver must not offer it to the gimbal or
    // the SAR and must not count it as an unclaimed frame
    return true;
}

// -------------------------------------------------------------------------
// report_short_frame : PNU-ISSUE(D6) count and report a short PMU status frame
// -------------------------------------------------------------------------
//   Rate limited, because a misbehaving PMU would otherwise emit one message
//   per frame at the poll rate.  The first occurrence always reports.
void AP_PMUCAN::report_short_frame(uint32_t frame_id, uint8_t dlc)
{
    _short_frame_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_short_frame_cnt > 1) &&
        ((now_ms - _last_short_frame_warn_ms) < PMUCAN_SHORT_FRAME_WARN_MS)) {
        return;
    }
    _last_short_frame_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PMUCAN: short frame 0x%lX dlc %u (%lu dropped)",
                  (unsigned long)frame_id, (unsigned)dlc, (unsigned long)_short_frame_cnt);
}


// -------------------------------------------------------------------------
// transmit_slot : send command @slot and poll status @slot
// -------------------------------------------------------------------------
// One slot per call, five slots, called at PMUCAN_MSG_RATE_HZ * CMD_ID_NUM - so each
// individual message goes out at exactly PMUCAN_MSG_RATE_HZ.  That is the ICD rate, and
// PNU-ISSUE(D7)'s interlock threshold of 10 ticks is calibrated on this function reaching
// CMD_ID_ENGONOFF at 10 Hz.  Do not restructure the rotation without re-deriving it.
// -------------------------------------------------------------------------
void AP_PMUCAN::transmit_slot(uint8_t slot)
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

        // PNU-ISSUE(D7) / step 2c : the only place either interlock window is stamped
        note_new_tc1();
    }


    // Send PMU Control Command to PMU
    switch(slot)
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

    // poll the matching status message.  The command and RTR indices advanced together
    // before 5.1.15 and still do - one slot drives both.
    if (slot < RTR_ID_NUM) {
        pmucan_rtr(_rtr_id[slot]);
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

    // PNU-ISSUE(D15) : host and PMU are both LSB-first, so the uint32 packs
    // straight into the frame with no byte swap.
    memcpy(can_data, &data_cmd, msgdlc);

    // AP_PNU_CAN owns the interface and is the single place a frame reaches the bus.
    // @can_id already carries FlagEFF; send_frame() passes the full 32-bit id through.
    cmd_send_res    = send_frame(can_id, can_data, msgdlc) ? 1 : 0;

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
// PNU-ISSUE(D7) : KGCS transmit behaviour is now known (2026-09-30, from the
//   KGCS side - it is a closed project, so this is its specified behaviour
//   rather than a bench measurement):
//     - TC1 is sent ONLY while a button is pressed, never unsolicited.
//     - Start sends Engine_OnOff alternating 2/3 for 14 messages, then stops.
//       Stop sends 4/5 the same way.  KGCS blocks Start once started.
//     - The burst rate is constant, ~10 Hz and never above 11 Hz.
//
//   READ THIS BEFORE CHANGING THE COUNTERS BELOW.  _engineoncnt / _engineoffcnt
//   are NOT a count of operator alternations.  The pair they inspect
//   (_pmu_ctrl_cmd / _pmu_ctrl_cmd_prv) only changes when a new TC1 arrives, but
//   this function is called at ~10 Hz regardless, so a frozen valid pair keeps
//   incrementing.  The counters are therefore a ~100 ms tick timer: the
//   threshold of 10 means "~1.0 s after the first valid alternating pair".
//   Consequences, all deliberate and accepted:
//     - Two messages plus ~1 s would satisfy the interlock; the other 12 are not
//       required.  This is SOUND ONLY BECAUSE KGCS never sends TC1 unsolicited.
//       If that ever changes, or another MAVLink sender appears on the link,
//       revisit this - handle_gcs_flcc_pmu_ctrl() does not check the sender.
//     - The threshold is 10, not 14, on purpose.  At 14 the trigger lands at
//       ~1.4 s, exactly when the burst ends, racing the FC's sample clock
//       against KGCS's send clock.  At 10 it fires ~1.0 s in, around message 10
//       of 14, with ~0.4 s of margin.  Do not raise it back.
//     - Message loss is tolerated for the same reason the timer works: a frozen
//       pair keeps counting through a dropout.
//     - Ingest is not rate-limited in practice: TC1 at <=11 Hz against TXspin's
//       50 Hz sampling means no alternation is missed at the sequence check.
//   The one real defect here has been fixed - see the send-result check below.
void AP_PMUCAN::engineonoffstate(void)
{
    const uint32_t now_ms = AP_HAL::millis();

    // PNU-ISSUE(D7) / step 2c, GUARD 1 of 2 : TC1 STALENESS.
    //
    // The counters below are a tick timer, not a count of operator alternations - the
    // command pair only changes when a new TC1 arrives, yet this runs at 10 Hz anyway, so
    // a pair frozen in a valid alternating state keeps counting.  A link dying mid-burst
    // therefore used to complete the start on its own about a second later.  Stop
    // counting instead.  Returning here sends nothing: ENGONOFF only goes out on a
    // transition, and a stale link cannot have a counter anywhere near the threshold.
    if (tc1_is_stale(now_ms)) {
        _engineoncnt  = 0U;
        _engineoffcnt = 0U;
        return;
    }

    if (_engineonoffmode==0U)	// OFF STATE
    {
        if(_engineoncnt>=10)        // transition to ON STATE
        {
            // PNU-ISSUE(D7) / step 2c, GUARD 2 of 2 : STOP BEFORE START.
            // Covers the link dying BETWEEN Stop and Start, where a queued Start arrives
            // after reconnect.  The counter is cleared so that a later Stop cannot find
            // it still armed and start the engine with no fresh Start press.
            if (!start_permitted(now_ms)) {
                _engineoncnt = 0U;
                return;
            }

            // PNU-ISSUE(D7) : only believe the engine is running once the frame
            // actually went out.  This used to advance unconditionally, leaving
            // the FC in ON while the PMU had never been told - and KGCS blocks
            // its Start button from that point, so the operator was left with
            // only Stop to resolve a state divergence they could not see.
            if(pmucan_cmd(_cmd_id[CMD_ID::CMD_ID_ENGONOFF], 1U)>0)  // ON(1)
            {
                _engineonoffmode    = 1U;                           // ON
                _engineoncnt        = 0U;
                PMU_Ctrl_Echo.Engine_OnOff_Echo = 1;
                engineonmode();
            }
            else
            {
                // send failed: stay OFF and leave the counter where it is, so
                // the next tick retries while the command pair is still valid.
                // pmucan_cmd() has already counted the failure in _cmd_tx_err.
                engineoffmode();
            }
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
            // PNU-ISSUE(D7) : as above - a failed stop must not read as stopped
            if(pmucan_cmd(_cmd_id[CMD_ID::CMD_ID_ENGONOFF], 0U)>0)  // OFF(0)
            {
                _engineonoffmode    = 0U;                           // OFF
                _engineoffcnt       = 0U;
                PMU_Ctrl_Echo.Engine_OnOff_Echo = 0;
                engineoffmode();
            }
            else
            {
                // send failed: stay ON and retry next tick
                engineonmode();
            }
        }
        else                        // during in ON STATE
        {
            engineonmode();
        }
    }
}


// -------------------------------------------------------------------------
// note_new_tc1 : PNU-ISSUE(D7) / step 2c - a NEW TC1 has arrived
// -------------------------------------------------------------------------
// Called from transmit_slot(), at the one point where a new command is recognised by its
// sequence number.  BOTH WINDOWS ARE STAMPED HERE AND NOWHERE ELSE.  Doing it from
// engineonoffstate() instead would read the FROZEN pair at 10 Hz and keep refreshing
// them, which defeats the staleness reset and holds the Stop window permanently open -
// the pair stays a valid Stop alternation from the end of a Stop burst until the first
// message of the next burst, so "a Stop was seen recently" would always be true.
// -------------------------------------------------------------------------
void AP_PMUCAN::note_new_tc1(void)
{
    _last_tc1_ms = AP_HAL::millis();

    // A valid 4/5 alternation arms the stop-before-start window, in EITHER engine state:
    // stop-while-off is the KGCS procedure, stop-while-on is a real shutdown, and both
    // should give permission to start.  A single stray Stop message does not arm it -
    // Start is validated as a pair, so permission to start is too.
    const uint32_t cmd = _pmu_ctrl_cmd.Engine_OnOff;
    const uint32_t prv = _pmu_ctrl_cmd_prv.Engine_OnOff;
    if (((cmd == 0b100U) && (prv == 0b101U)) ||
        ((cmd == 0b101U) && (prv == 0b100U))) {
        _last_stop_pair_ms = _last_tc1_ms;
        _stop_window_armed = true;
    }
}


// -------------------------------------------------------------------------
// tc1_is_stale : PNU-ISSUE(D7) / step 2c
// -------------------------------------------------------------------------
// Before the first TC1 of a boot there is nothing to be stale: KGCS sends TC1 only while
// a button is held, so silence is the normal resting state and the counters are zero
// anyway.  Treating that as stale would be harmless but would also be a lie.
// -------------------------------------------------------------------------
bool AP_PMUCAN::tc1_is_stale(uint32_t now_ms) const
{
    if (_last_tc1_ms == 0U) {
        return false;
    }
    return (now_ms - _last_tc1_ms) > PMUCAN_TC1_STALE_MS;
}


// -------------------------------------------------------------------------
// start_permitted : PNU-ISSUE(D7) / step 2c - did a recent Stop precede this Start?
// -------------------------------------------------------------------------
// A REFUSED START MUST SAY WHY.  At boot no Stop has been seen, so the first Start of a
// power cycle is refused - correct per the KGCS procedure, but a button that silently
// does nothing is the PNU-ISSUE(D6) problem over again.  Rate limited, because this is
// reached at 10 Hz for as long as the burst keeps arriving.
// -------------------------------------------------------------------------
bool AP_PMUCAN::start_permitted(uint32_t now_ms)
{
    // MEASURED FROM WHEN THE OPERATOR PRESSED START, NOT FROM NOW.  The guard asks "did a
    // Stop precede this Start by less than the window" - and the counter spends about
    // 1.0 s of that window by itself, since the threshold of 10 is a tick timer at 10 Hz.
    // Testing at the completion instant would charge the operator for the FC's own
    // interlock delay, shortening the time they actually have to press the button by
    // roughly that second on top of KGCS's ~5 s lockout.  _start_pair_first_ms is when
    // this attempt's first valid alternation arrived, and the staleness guard above
    // bounds how far in the past that can be - at most one stale window.
    const uint32_t pressed_ms = (_start_pair_first_ms != 0U) ? _start_pair_first_ms : now_ms;

    if (_stop_window_armed &&
        ((pressed_ms - _last_stop_pair_ms) <= PMUCAN_STOP_WINDOW_MS)) {
        return true;
    }

    if ((_last_start_refused_ms == 0U) ||
        ((now_ms - _last_start_refused_ms) >= PMUCAN_START_REFUSED_WARN_MS)) {
        _last_start_refused_ms = now_ms;
        if (!_stop_window_armed) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PMU: start refused, press Stop first");
        } else {
            // kept under 50 characters: MAVLink STATUSTEXT truncates there and a
            // longer message is split into chunks that not every GCS reassembles.
            // The age is clamped for the same reason - it is only ever read as
            // "longer ago than the limit".
            const uint32_t age_s = MIN((pressed_ms - _last_stop_pair_ms) / 1000U, 999U);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "PMU: start refused, Stop %us ago (max %us)",
                          (unsigned)age_s,
                          (unsigned)(PMUCAN_STOP_WINDOW_MS / 1000U));
        }
    }
    return false;
}


// -------------------------------------------------------------------------
// count_start_pair : PNU-ISSUE(D7) / step 2c - advance the Start counter
// -------------------------------------------------------------------------
// Identical to the bare _engineoncnt++ it replaces, except that the FIRST increment of an
// attempt is timestamped.  start_permitted() measures the stop-before-start window from
// that instant rather than from the moment the counter completes, so the operator is not
// charged for the interlock's own ~1.0 s.
// -------------------------------------------------------------------------
void AP_PMUCAN::count_start_pair(void)
{
    if (_engineoncnt == 0U) {
        _start_pair_first_ms = AP_HAL::millis();
    }
    _engineoncnt++;
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
            count_start_pair();
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
            count_start_pair();
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

    // @can_id carries FlagEFF | FlagRTR; the driver's send_frame() does not mask them
    rtr_send_res    = send_frame(can_id, can_data, 8) ? 1 : 0;

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

#endif  // AP_PNU_CAN_ENABLED
