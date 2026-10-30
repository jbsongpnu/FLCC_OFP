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

#include <AP_PNU_CAN/AP_PNU_CAN.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL &hal;

// -------------------------------------------------------------------------
// init : read the one parameter this backend has
// -------------------------------------------------------------------------
// The SAR's ids are fixed by the ICD rather than by parameters - unlike the gimbal, whose
// motor ids are configured - so there is nothing here that can be set to a wrong value.
// The bitrate is the driver's and is already 1 Mbit/s for the CubeMars motors, which is
// also what the ICD requires.
//
// CAN_Dn_PNU_SCHK is read live, at each Start, rather than snapshotted here - and it
// carries no @RebootRequired for the same reason.  A bench operator tightens or relaxes
// the interlock between one Start and the next, and making them reboot to do it would
// turn a two-minute check into a two-minute wait.  Nothing reads it mid-window.
// -------------------------------------------------------------------------
bool AP_PNUSAR::init(void)
{
    // Match whatever the GCS layer has already counted, so a TC that arrived before the
    // backend existed is not replayed as a fresh command at boot.
    _ctrl_seq = gcs().SAR_Ctrl_Seq;

    SAR_Status.status = uint8_t(PNUSAR_State::DISCONNECTED);
    SAR_Status.live_counter = 0;
    return true;
}


// -------------------------------------------------------------------------
// update : the command layer and the heartbeat timeout
// -------------------------------------------------------------------------
// The FC transmits nothing periodically in 3d.  0x501 and 0x502 go out once each, when
// the GCS asks; the 10 Hz navigation bundle arrives in 3e behind _nav_tx_enabled.
// -------------------------------------------------------------------------
void AP_PNUSAR::update(uint32_t tick)
{
    report_schk_once();
    check_ctrl();
    check_pending();

    if ((tick % AP_PNUSAR_HEALTH_CHECK_TICKS) == 0) {
        check_health();
    }
}


// -------------------------------------------------------------------------
// report_schk_once : say so when the Start interlock is not the ICD's
// -------------------------------------------------------------------------
// Sent on the same trigger as the driver's own startup line and for the same reason: a
// one-shot statustext emitted during boot is dropped before any GCS is listening, and
// because it is one-shot it never comes back.  The condition that matters is a GCS being
// there to hear it, not a fixed delay.
// -------------------------------------------------------------------------
void AP_PNUSAR::report_schk_once(void)
{
    if (_schk_reported || (GCS_MAVLINK::active_channel_mask() == 0)) {
        return;
    }
    _schk_reported = true;

    const uint8_t checks = _frontend.get_sar_start_checks();

    if ((checks & AP_PNUSAR_SCHK_ATT_AGE) == 0) {
        // The default, and it will stay the default until step 3f.  0x404 ATTITUDE carries
        // hard-coded zeros (PNU's decision, 2026-10-30), so there is no attitude source
        // whose age could be measured - and ICD 16.4 forbids presenting a substitute as
        // valid.  Enforcing a 40 ms freshness rule against a constant would be claiming a
        // guarantee we do not have, which is worse than saying this out loud every boot.
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: ATT age check off, no source");
    }

    if ((checks & AP_PNUSAR_SCHK_ICD) != AP_PNUSAR_SCHK_ICD) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: Start checks relaxed, SCHK=%u",
                      (unsigned)checks);
    }
}


// -------------------------------------------------------------------------
// check_ctrl : consume one TC 50050, SAR_ICD_PNU.md sections 3.1 and 8
// -------------------------------------------------------------------------
// GCS_PNU.cpp has already refused a Control above 2 and a sender that MAV_GCS_SYSID does
// not cover, so everything arriving here is a command this FC is allowed to act on.  The
// sequence number is the whole handshake: the GCS layer increments it, this reads the
// increment exactly once, and a TC that repeats the same Control still counts as a new
// command because the ICD requires the retry to be forwarded again.
// -------------------------------------------------------------------------
void AP_PNUSAR::check_ctrl(void)
{
    const uint8_t seq = gcs().SAR_Ctrl_Seq;
    if (seq == _ctrl_seq) {
        return;
    }
    _ctrl_seq = seq;

    switch (gcs().SAR_Ctrl.Observation_Control) {
    case 0:
        // Prepare.  ICD 8: "Receive Prepare without additional device transmission - do
        // not invent a state change."  Heart_Beat reception already determines readiness,
        // so there is genuinely nothing for the FC to do and nothing to report.
        break;

    case 1:
        handle_start_request();
        break;

    case 2:
        handle_stop_request();
        break;

    default:
        // Unreachable: handle_gcs_flcc_sar_ctrl() refuses 3-255 before it ever gets here.
        break;
    }

    // Observation_Mode is deliberately not touched.  It stays in gcs().SAR_Ctrl, whole,
    // because every one of its 256 values is legal (ICD 3.1) and its CAN encoding lives
    // in 0x503, which the manufacturer has not defined.  ICD 13 is explicit: do not place
    // Mode at B0 of the command and do not send eight zero bytes in its place.
}


// -------------------------------------------------------------------------
// handle_start_request : the RTK interlock, SAR_ICD_PNU.md section 14.3
// -------------------------------------------------------------------------
// 14.3.3 : "If rejected, send no 0x501.  RTK rejection is not SAR nonresponse and must
// not create state 4."  That second sentence is the reason this returns early instead of
// falling into the pending machinery - a blocked Start leaves the published state exactly
// where it was, and the GCS is told why in words rather than through a state it would
// have to interpret.
// -------------------------------------------------------------------------
void AP_PNUSAR::handle_start_request(void)
{
    const char *blocked = start_blocked_reason();
    if (blocked != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: Start blocked, %s", blocked);
        return;
    }

    if (!send_command(AP_PNUSAR_ID_START_CMD, PNUSAR_Pending::START)) {
        return;
    }

    if ((_frontend.get_sar_start_checks() & AP_PNUSAR_SCHK_ICD) != AP_PNUSAR_SCHK_ICD) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: Start sent, checks bypassed");
    }

    // Repeating Start at an already-Started SAR is correct and must not be suppressed
    // here: ICD 14.3 requires the device to answer 0x521/01 again without toggling, and
    // the FC's job is to forward the attempt, not to decide it is redundant.
}


// -------------------------------------------------------------------------
// handle_stop_request : SAR_ICD_PNU.md section 14.4
// -------------------------------------------------------------------------
// "Do not block Stop based on RTK Fixed status" - the ICD puts that in bold, and it is
// the whole asymmetry between this function and the one above.  Stopping a payload that
// is producing contaminated data must never depend on the quality of the data.
// -------------------------------------------------------------------------
void AP_PNUSAR::handle_stop_request(void)
{
    if (!_link_up) {
        // ICD 14.6 : "Do not force commands while disconnected."  There is no path to the
        // device, and a frame sent into a dead bus that then times out would publish a
        // state 4 that means nothing.
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: Stop blocked, disconnected");
        return;
    }

    // Navigation is deliberately left ON while the response is outstanding (14.4.3): the
    // Stop is not confirmed yet, and 3e's bundle gate reads _nav_tx_enabled.
    send_command(AP_PNUSAR_ID_STOP_CMD, PNUSAR_Pending::STOP);
}


// -------------------------------------------------------------------------
// start_blocked_reason : ICD 14.3.3, or nullptr when the Start may go out
// -------------------------------------------------------------------------
// CROSS-THREAD READ, STATED DELIBERATELY: this runs on the AP_PNU_CAN driver thread and
// consults AP_GPS and AP_AHRS, which the vehicle thread writes.  Every value read here is
// a scalar - an enum, a uint32 timestamp, a bool - so a read can be stale by one loop but
// cannot be torn into a value that was never written.  Nothing is locked, because taking
// an AHRS lock from the CAN thread would be a far worse trade.  The consequence of a
// stale read is that one operator Start is refused or allowed one loop late, and a Start
// is a button press that can be pressed again.
// -------------------------------------------------------------------------
const char *AP_PNUSAR::start_blocked_reason(void) const
{
    const uint8_t checks = _frontend.get_sar_start_checks();

    if (!_ever_connected || !_link_up) {
        return "disconnected";
    }

    // ICD 14.8 : while recovery is pending the FC must not request Start, because the
    // data that the outage contaminated has not been inspected.  There is no release
    // procedure in Rev1.0 and this file does not invent one - a power cycle is the way
    // out, and that is the specified behaviour rather than a missing feature.
    if (_recovery_pending) {
        return "recovery pending";
    }

#if AP_GPS_ENABLED
    const AP_GPS &gps = AP::gps();

    if (checks & AP_PNUSAR_SCHK_RTK_FIXED) {
        // The automatically selected Primary, which is what AP_GPS::status() reports.
        // Status 6 exactly: ICD 14.7 keeps an observation running through a drop to Float,
        // but starting one needs Fixed.
        if (gps.status() != AP_GPS::GPS_OK_FIX_3D_RTK_FIXED) {
            return "no RTK Fixed";
        }
    }

    if (checks & AP_PNUSAR_SCHK_GPS_AGE) {
        if ((AP_HAL::millis() - gps.last_fix_time_ms()) > AP_PNUSAR_GPS_AGE_MAX_MS) {
            return "GPS fix stale";
        }
    }
#else
    if (checks & (AP_PNUSAR_SCHK_RTK_FIXED | AP_PNUSAR_SCHK_GPS_AGE)) {
        return "no GPS in build";
    }
#endif  // AP_GPS_ENABLED

    if (checks & AP_PNUSAR_SCHK_ATT_AGE) {
        // Off by default and refusing when on, because 0x404 carries hard-coded zeros and
        // a constant has no age.  Step 3f supplies the gimbal's measured attitude and
        // turns this into a real comparison against AP_PNUSAR_ATT_AGE_MAX_MS.
        return "no ATT source";
    }

#if AP_AHRS_ENABLED
    if (checks & AP_PNUSAR_SCHK_NAV_HEALTH) {
        if (!AP::ahrs().healthy()) {
            return "nav unhealthy";
        }
    }
#else
    if (checks & AP_PNUSAR_SCHK_NAV_HEALTH) {
        return "no AHRS in build";
    }
#endif  // AP_AHRS_ENABLED

    return nullptr;
}


// -------------------------------------------------------------------------
// send_command : one frame out, one window open, SAR_ICD_PNU.md section 14.5
// -------------------------------------------------------------------------
// "Open an FC 2 s response window only for a Start/Stop ACTUALLY TRANSMITTED to SAR."
// That is why a refused send returns without a window: a frame the interface would not
// accept never reached the device, so letting it expire into state 4 would blame the SAR
// for a fault on this side of the connector.
// -------------------------------------------------------------------------
bool AP_PNUSAR::send_command(uint32_t can_id, PNUSAR_Pending pending)
{
    const uint8_t payload = AP_PNUSAR_CMD_PAYLOAD;

    if (!send_frame(can_id, &payload, AP_PNUSAR_CMD_DLC)) {
        // The driver has already counted the failure and rate-limits its own report; this
        // one names the command, because "a send failed" and "your Start did not happen"
        // are different pieces of news to an operator holding a button.
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SAR: %s not sent",
                      (pending == PNUSAR_Pending::START) ? "Start" : "Stop");
        return false;
    }

    _pending = pending;
    _pending_sent_ms = AP_HAL::millis();

    // ICD 14.5 : a GCS retry clears the preceding attempt's latch and publishes
    // LAST_CONFIRMED_STATE while the new window is open.  publish_state() does both,
    // because the latch is one of the inputs it ranks.
    _not_responding = false;
    publish_state();
    return true;
}


// -------------------------------------------------------------------------
// check_pending : the 2 s window expiring, SAR_ICD_PNU.md section 14.5
// -------------------------------------------------------------------------
void AP_PNUSAR::check_pending(void)
{
    if (_pending == PNUSAR_Pending::NONE) {
        return;
    }
    if ((AP_HAL::millis() - _pending_sent_ms) < AP_PNUSAR_RESP_TIMEOUT_MS) {
        return;
    }

    const bool was_start = (_pending == PNUSAR_Pending::START);

    // Closed BEFORE the latch is set, so that a response arriving in the same tick finds
    // no open window and is discarded: 14.5 forbids retroactively counting an expired
    // response as success.
    _pending = PNUSAR_Pending::NONE;
    _not_responding = true;

    if (was_start) {
        // 14.5 : keep navigation OFF on a failed Start.  It was never on - the gate only
        // opens on a confirmed 0x521 - but saying so here is what makes the asymmetry
        // with the Stop case below legible.
        _nav_tx_enabled = false;
    }
    // A failed normal user Stop deliberately RETAINS the prior navigation state, because
    // stopping is unconfirmed and the device may well still be transmitting.  Turning the
    // bundle off here would make the FC's record disagree with the payload's behaviour.

    publish_state();

    // Deliberately NOT the ICD's "SAR not responding to Start".  That exact text belongs
    // to the GCS and means the whole two-attempt transaction has failed (14.6); this is
    // one FC window expiring, and the GCS may still have a retry in hand.  The navigation
    // state is included because 14.5 makes it differ between the two cases.
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: no %s response, nav %s",
                  was_start ? "Start" : "Stop",
                  _nav_tx_enabled ? "ON" : "OFF");
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

    if (!_link_up) {
        return;                         // already lost; nothing further to detect
    }

    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_hb_ms) < AP_PNUSAR_HB_TIMEOUT_MS) {
        return;
    }

    // ICD 14.2.4 : at least 3 s without a valid heartbeat means Disconnected, navigation
    // off and cancellation of pending commands.  All three now exist.
    _link_up = false;
    _recovery_pending = true;
    _announce_reconnect = true;
    _nav_tx_enabled = false;

    // Cancelled, NOT timed out.  A command whose device has gone off the bus has not
    // failed to answer within its window; it has no window left to answer in, and
    // publishing state 4 for it would claim the SAR was present but silent.  ICD 14.1
    // ranks Disconnected above no-response for exactly this reason.
    _pending = PNUSAR_Pending::NONE;

    publish_state();
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR disconnected");
}


// -------------------------------------------------------------------------
// publish_state : the one place SAR_STATUS is decided
// -------------------------------------------------------------------------
// SAR_ICD_PNU.md section 8 states a PRIORITY, not a sequence of assignments:
//
//     disconnection / heartbeat loss / recovery pending  (0)
//   > current-attempt response timeout                   (4)
//   > last confirmed state                               (1, 2, 3)
//
// Step 3a could get away with assigning the state at each event because there were only
// two of them.  With five states and two independent latches, last-writer-wins would
// quietly violate the document - "Heartbeat must not overwrite states 2/3/4 with Ready"
// (14.1) is only one of the orderings it would break.  Computing the published value from
// the inputs makes the priority true by construction instead of by remembering it.
// -------------------------------------------------------------------------
void AP_PNUSAR::publish_state(void)
{
    PNUSAR_State state;

    if (!_ever_connected || !_link_up || _recovery_pending) {
        state = PNUSAR_State::DISCONNECTED;
    } else if (_not_responding) {
        state = PNUSAR_State::NOT_RESPONDING;
    } else {
        state = _last_confirmed;
    }

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
        return handle_response(frame, PNUSAR_Pending::START, PNUSAR_State::STARTED);

    case AP_PNUSAR_ID_STOP_RESP:
        return handle_response(frame, PNUSAR_Pending::STOP, PNUSAR_State::STOPPED);

    case AP_PNUSAR_ID_PARAM_RESP:
        // RESERVED.  ICD 12.4 : eight opaque bytes, no invented interpretation.  Counted
        // and reported rarely, because a device sending it means the payload has been
        // defined and this file has not caught up.
        _reserved_rx_cnt++;
        {
            const uint32_t now_ms = AP_HAL::millis();
            if ((_reserved_rx_cnt == 1) ||
                ((now_ms - _last_reserved_warn_ms) >= AP_PNUSAR_WARN_INTERVAL_MS)) {
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
    _link_up = true;

    if (!_ever_connected) {
        _ever_connected = true;
        _last_confirmed = PNUSAR_State::READY;   // ICD 14.2.3, the only path to Ready
        publish_state();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SAR connected");
        return true;
    }

    if (_announce_reconnect) {
        // ICD 14.8 : THE RECONNECTION NOTIFICATION IS NOT A RETURN TO READY.  A
        // reconnecting heartbeat must not restore Ready, request Start or resume
        // navigation, because the data contaminated by the outage has not been
        // inspected.  So the event is announced once and _recovery_pending stays set,
        // which keeps publish_state() on Disconnected until the next power cycle.  The
        // release procedure is deliberately deferred by the ICD; that is the specified
        // behaviour rather than a missing feature.
        _announce_reconnect = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SAR reconnected, inspection pending");
    }

    // Not a state change of its own: 14.1 forbids a heartbeat overwriting 2/3/4 with
    // Ready, and publish_state() enforces that by ranking rather than by assignment.
    publish_state();
    return true;
}


// -------------------------------------------------------------------------
// handle_response : 0x521 / 0x522, SAR_ICD_PNU.md sections 12.2, 12.3 and 14.3-14.5
// -------------------------------------------------------------------------
// A response is applied ONLY when it matches an open window of the same kind.  The
// protocol carries no transaction id - ICD 3.1 says so and accepts the consequence - so a
// late or duplicated response is indistinguishable from a fresh one except by whether
// anything is waiting for it.  Discarding the unmatched ones is the whole of the defence.
// -------------------------------------------------------------------------
bool AP_PNUSAR::handle_response(const AP_HAL::CANFrame &frame, PNUSAR_Pending expect,
                                PNUSAR_State confirmed)
{
    if (frame.dlc != AP_PNUSAR_RESP_DLC) {
        report_short_frame(frame.id & frame.MaskStdID, frame.dlc);
        return true;
    }

    if (frame.data[0] != AP_PNUSAR_CMD_PAYLOAD) {
        // ICD 12.2 and 12.3 define exactly one payload, 0x01.  Anything else is not the
        // matching response the window is waiting for, so it must not confirm the
        // command - the window is left open to expire honestly into state 4.
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: 0x%03lx payload 0x%02x",
                      (unsigned long)(frame.id & frame.MaskStdID), (unsigned)frame.data[0]);
        return true;
    }

    if (_pending != expect) {
        // 14.5 : "Do not retroactively count an expired response as success."  This is
        // also where the idempotent-repeat case lands harmlessly - a device answering a
        // Start it had already completed, after our window closed.
        _stray_resp_cnt++;
        const uint32_t now_ms = AP_HAL::millis();
        if ((_stray_resp_cnt == 1) ||
            ((now_ms - _last_stray_resp_warn_ms) >= AP_PNUSAR_WARN_INTERVAL_MS)) {
            _last_stray_resp_warn_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: unmatched 0x%03lx (%lu)",
                          (unsigned long)(frame.id & frame.MaskStdID),
                          (unsigned long)_stray_resp_cnt);
        }
        return true;
    }

    _pending = PNUSAR_Pending::NONE;
    _not_responding = false;
    _last_confirmed = confirmed;

    // ICD 14.3.5 and 14.4.4 : the confirmed response is the only thing that moves the
    // navigation gate, in either direction.  3e reads it; nothing transmits a bundle yet.
    _nav_tx_enabled = (confirmed == PNUSAR_State::STARTED);

    publish_state();

    // _state rather than @confirmed, because recovery-pending outranks both (14.8) and an
    // operator told "SAR started" while the TM still reads 0 would be owed an explanation.
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SAR %s confirmed, state %u",
                  (confirmed == PNUSAR_State::STARTED) ? "Start" : "Stop",
                  (unsigned)_state);
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
