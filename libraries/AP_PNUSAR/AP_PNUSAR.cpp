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
 */

#include "AP_PNUSAR.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// PNU-ISSUE(D14) : defined OUTSIDE the AP_PNUSAR_ENABLED guard, beside its declaration in
// the header, because GCS_PNU.cpp reads it whether or not the CAN driver is built.  The
// static-init values are the honest answer for a build with no SAR: Disconnected, and no
// heartbeat ever counted.
PNU_SAR_Status SAR_Status;

#if AP_PNUSAR_ENABLED

extern const AP_HAL::HAL &hal;

// -------------------------------------------------------------------------
// init : nothing to validate yet
// -------------------------------------------------------------------------
// The SAR has no configuration of its own in 3a.  Its ids are fixed by the ICD rather
// than by parameters - unlike the gimbal, whose motor ids are configured - so there is
// nothing here that can be set wrongly.  The bitrate is the driver's and is already
// 1 Mbit/s for the CubeMars motors, which is also what the ICD requires.
// -------------------------------------------------------------------------
bool AP_PNUSAR::init(void)
{
    SAR_Status.status = uint8_t(PNUSAR_State::DISCONNECTED);
    SAR_Status.live_counter = 0;
    return true;
}


// -------------------------------------------------------------------------
// update : the heartbeat timeout, and nothing else in 3a
// -------------------------------------------------------------------------
// The SAR is silent until it has something to say: in 3a the FC transmits nothing at all.
// Commands 0x501/0x502 arrive in 3d and the 10 Hz navigation bundle in 3e.
// -------------------------------------------------------------------------
void AP_PNUSAR::update(uint32_t tick)
{
    if ((tick % AP_PNUSAR_HEALTH_CHECK_TICKS) == 0) {
        check_health();
    }
}


// -------------------------------------------------------------------------
// check_health : SAR_ICD_PNU.md section 3.3, T_HB = 3000 ms
// -------------------------------------------------------------------------
// Unlike the PMU's leaky bucket this is a hard deadline, because the ICD states one.  A
// single heartbeat per second leaves no room for a counted-failure scheme: three missed
// beats IS the timeout.
// -------------------------------------------------------------------------
void AP_PNUSAR::check_health(void)
{
    if (!_ever_connected) {
        // Waiting for the first heartbeat after a common power reset is NOT a prior-link
        // failure and NOT recovery pending - ICD 14.2.1.  Staying quiet here is what lets
        // that first heartbeat reach Ready.
        return;
    }

    if (_state == PNUSAR_State::DISCONNECTED) {
        return;                         // already lost; nothing further to detect
    }

    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_hb_ms) < AP_PNUSAR_HB_TIMEOUT_MS) {
        return;
    }

    // ICD 14.2.4 : at least 3 s without a valid heartbeat means Disconnected, navigation
    // off and pending commands cancelled.  Only the first of those exists in 3a; the
    // other two land with the code that can create them.
    _recovery_pending = true;
    set_state(PNUSAR_State::DISCONNECTED);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR disconnected");
}


// -------------------------------------------------------------------------
// set_state : one place that publishes SAR_STATUS
// -------------------------------------------------------------------------
void AP_PNUSAR::set_state(PNUSAR_State state)
{
    _state = state;
    SAR_Status.status = uint8_t(state);
}


// -------------------------------------------------------------------------
// handle_frame : claim the SAR's ids, 11-bit standard only
// -------------------------------------------------------------------------
// Returning false for anything else is what lets the PMU and the gimbal share this bus -
// the driver offers each frame to every backend until one claims it.  The PMU is 29-bit
// extended throughout and the gimbal lives in 0x000-0x2FF, so the three never overlap.
// -------------------------------------------------------------------------
bool AP_PNUSAR::handle_frame(const AP_HAL::CANFrame &frame)
{
    if (frame.isExtended() || frame.isRemoteTransmissionRequest() || frame.isErrorFrame()) {
        return false;                   // the PMU's, or not ours
    }

    const uint32_t id = frame.id & frame.MaskStdID;

    switch (id) {
    case AP_PNUSAR_ID_HEARTBEAT:
        return handle_heartbeat(frame);

    case AP_PNUSAR_ID_START_RESP:
    case AP_PNUSAR_ID_STOP_RESP:
        // 3d owns these.  Claimed now so that a real device's responses are not reported
        // as unknown traffic, and deliberately not acted on: confirming a Start before
        // the state machine that can refuse one exists would be worse than ignoring it.
        return true;

    case AP_PNUSAR_ID_PARAM_RESP:
        // RESERVED.  ICD 12.4 : eight opaque bytes, no invented interpretation.  Counted
        // and reported rarely, because a device sending it means the payload has been
        // defined and this file has not caught up.
        _reserved_rx_cnt++;
        {
            const uint32_t now_ms = AP_HAL::millis();
            if ((_reserved_rx_cnt == 1) ||
                ((now_ms - _last_reserved_warn_ms) >= 60000U)) {
                _last_reserved_warn_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "SAR: 0x523 reserved, payload still TBD (%lu)",
                              (unsigned long)_reserved_rx_cnt);
            }
        }
        return true;

    default:
        break;
    }

    // The navigation ids and the commands are FC -> SAR, and the driver has already
    // discarded loopback, so receiving one means A SECOND TALKER IS USING OUR IDS.  That
    // is left UNCLAIMED on purpose: the driver's unclaimed-frame report is exactly the
    // signal a mis-assigned id needs, and claiming it here would hide a real bus fault.
    return false;
}


// -------------------------------------------------------------------------
// handle_heartbeat : 0x510, SAR_ICD_PNU.md sections 3.3, 12.1 and 14.2
// -------------------------------------------------------------------------
// THE PAYLOAD IS DELIBERATELY NOT USED.  The SAR's own Raw_Counter is a four-byte field
// that we must NOT copy into Live_Counter - ICD 3.3 says so twice.  Live_Counter counts
// what the FC actually received, which is a different quantity and the one the operator
// needs.  The DLC is still checked, because a short frame is not a valid heartbeat.
// -------------------------------------------------------------------------
bool AP_PNUSAR::handle_heartbeat(const AP_HAL::CANFrame &frame)
{
    if (frame.dlc != AP_PNUSAR_HEARTBEAT_DLC) {
        // ICD 3.3 : a validation failure must not increment the count or refresh
        // freshness.  Claimed anyway - the id is ours, and reporting it twice, here and
        // as the driver's unknown-frame warning, would be worse.  report_short_frame()
        // does its own counting and rate limiting, so there is no counter here: the old
        // skeleton's warning against write-only counters applies to its own author too.
        report_short_frame(AP_PNUSAR_ID_HEARTBEAT, frame.dlc);
        return true;
    }

    SAR_Status.live_counter++;          // ICD 3.3 : +1 per VALID reception, never per TM
    _last_hb_ms = AP_HAL::millis();

    if (!_ever_connected) {
        _ever_connected = true;
        set_state(PNUSAR_State::READY);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SAR connected");
        return true;
    }

    if (_recovery_pending) {
        // ICD 14.8 : THE RECONNECTION NOTIFICATION IS NOT A RETURN TO READY.  A
        // reconnecting heartbeat must not restore Ready, request Start or resume
        // navigation, because the data contaminated by the outage has not been
        // inspected.  So the event is announced once and the state stays Disconnected.
        // The release procedure is deliberately deferred by the ICD; until it exists,
        // the only way back is a power cycle, and that is the intended behaviour rather
        // than a missing feature.
        _recovery_pending = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SAR reconnected, inspection pending");
    }

    return true;
}


// -------------------------------------------------------------------------
// healthy : a fresh heartbeat, nothing more
// -------------------------------------------------------------------------
// Deliberately NOT the same question as SAR_STATUS.  After a reconnect the link is live
// and healthy() is true while the published state stays Disconnected, because physical
// freshness and permission to operate are different things - ICD 14.8 names that split.
// -------------------------------------------------------------------------
bool AP_PNUSAR::healthy(void) const
{
    if (!_ever_connected) {
        return false;
    }
    return (AP_HAL::millis() - _last_hb_ms) < AP_PNUSAR_HB_TIMEOUT_MS;
}

#endif  // AP_PNUSAR_ENABLED
