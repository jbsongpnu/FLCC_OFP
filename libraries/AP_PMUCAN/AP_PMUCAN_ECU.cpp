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
//  ==================================================================================
//  PMU-005a stage 3 : MODESTS and ENERGYSTS, the two ECU-only status messages
//  ==================================================================================

#include "AP_PMUCAN_ECU.h"

#if AP_PMUCAN_ECU_ENABLED

#include "AP_PMUCAN.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <string.h>


// -------------------------------------------------------------------------
// update : poll the two ECU messages on the phases the legacy rotation leaves free
// -------------------------------------------------------------------------
// WHY PHASES AND NOT A TIMER.  AP_PMUCAN's five command/RTR slots fire on the even
// phases of each 10-tick window - stride = (100/10)/5 = 2 - so phases 1, 3, 5, 7 and 9
// have never carried a frame.  Polling there means the ECU traffic cannot delay a legacy
// slot by even one tick, which matters because PNU-ISSUE(D7)'s engine interlock is
// calibrated on engineonoffstate() being reached at exactly 10 Hz.
//
// ONLY WHILE THE ECU GENERATION IS SELECTED, fallback included.  protocol() applies the
// fallback in one place, so a device that never answers VERSTS is polled as an ECU - the
// approved behaviour, and the reason a legacy PMU can be asked for a message it does not
// know.  The ECU draft says it ignores unexpected frames; the legacy PMU's response to an
// unknown RTR is unspecified, which is a bench item under PMU-005a.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::update(uint32_t tick)
{
    if (_pmu.protocol() != PROTO_ECU_V2) {
        return;
    }

    const uint32_t phase = tick % PMUCAN_SLOT_PERIOD_TICKS;

    // 10 Hz, the ICD's 100 ms status poll
    if (phase == MODESTS_PHASE) {
        poll(ECU_MODESTS | AP_HAL::CANFrame::FlagEFF | AP_HAL::CANFrame::FlagRTR);
    }

    // 1 Hz.  ENERGYSTS carries configured capacities and an endurance estimate, none of
    // which move at the status rate, and the ICD asks for 1 s.
    if ((tick % AP_PNU_CAN_LOOP_HZ) == ENERGYSTS_PHASE) {
        poll(ECU_ENERGYSTS | AP_HAL::CANFrame::FlagEFF | AP_HAL::CANFrame::FlagRTR);
    }
}


// -------------------------------------------------------------------------
// poll : one RTR request, counted in this module's own counters
// -------------------------------------------------------------------------
// NOT through AP_PMUCAN::pmucan_rtr().  That function increments PMUC's RtrTx and
// RtrErr, whose meaning is "the five legacy polls" at a rate a reader can predict from
// the ICD; adding ECU polls to it would silently change what every existing PMUC record
// says.  The counters here are separate for exactly that reason.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::poll(uint32_t can_id)
{
    if (_pmu.send_ecu_frame(can_id, ECU_STS_DLC)) {
        _rtr_tx++;
    } else {
        _rtr_err++;
    }
}


// -------------------------------------------------------------------------
// handle_frame : decode MODESTS and ENERGYSTS
// -------------------------------------------------------------------------
// Reached only for extended data frames AP_PMUCAN did not recognise as one of its five
// legacy status ids, so the legacy path is untouched by this file.
//
// CLAIMED EVEN WHEN NOT DECODED.  Both ids belong to the PMU/ECU whatever generation is
// selected, so returning true stops the driver offering them to the gimbal or the SAR and
// stops them being counted as unclaimed.  Decoding is a separate question: the ICD says
// not to decode these layouts unless VERSTS confirms 2.00 or higher, and in legacy mode
// they are never polled in the first place.
//
// THEY DO NOT FEED THE HEALTH BUCKET.  _pmu_frames_seen counts the five legacy status
// messages and nothing else; that input set is fixed by PNU-ISSUE(D6) and stays fixed.
// The five legacy polls continue on both generations, so PMU health means the same thing
// on an ECU as it does on a legacy PMU.
//
// PNU-ISSUE(D15) : LSB-first on both sides, so every multi-byte field below is a straight
// memcpy into a native integer.  No byte swap - do not "fix" it with be16toh.
// -------------------------------------------------------------------------
bool AP_PMUCAN_ECU::handle_frame(const AP_HAL::CANFrame &frame)
{
    const uint32_t frame_id = frame.id & frame.MaskExtID;

    if ((frame_id != ECU_MODESTS) && (frame_id != ECU_ENERGYSTS)) {
        return false;
    }

    // PNU-ISSUE(D6), applied to the new messages: the ICD gives both a DLC of 8 and every
    // field below sits inside those 8 bytes, so a short frame is counted and dropped
    // rather than parsed from the same fixed offsets.  Counted here and not in
    // _short_frame_cnt, which is PMUC's column for the legacy five.
    if (frame.dlc < ECU_STS_DLC) {
        _short_frames++;
        return true;
    }

    if (_pmu.protocol() != PROTO_ECU_V2) {
        return true;
    }

    uint16_t uint16_temp = 0U;

    switch (frame_id) {
        case ECU_MODESTS:
            // Byte 0 is TWO 4-bit fields, decoded explicitly as the ICD requires.  A zero
            // in either is not a state: it is undefined for both, and the ECU also sends
            // reserved fields as zero, so it is carried through raw and given no meaning
            // here.  The same applies to every undefined ctrl_mode, ems_mode and
            // start_gate code - none of them is mapped onto the nearest defined one.
            _mode.system_state   = frame.data[0] & 0x0FU;
            _mode.operation_mode = (frame.data[0] >> 4) & 0x0FU;
            _mode.ctrl_mode      = frame.data[1];
            _mode.alert_level    = frame.data[2];
            _mode.ems_mode       = frame.data[3];
            _mode.start_gate     = frame.data[4];

            // initial_fail is a bitfield with four bits undefined in this draft.  It is
            // recorded whole, including those bits.  It does NOT gate anything: the ECU
            // performs that check itself and the ICD is explicit that the Host must not
            // build a second Start interlock on it.
            _mode.initial_fail   = frame.data[5];

            // bytes 6-7 are rpm_target, 1 rpm per count - NOT a continuation of
            // initial_fail and not reserved, which the ICD calls out because the draft
            // invited exactly that reading.
            memcpy(&uint16_temp, &frame.data[6], 2);
            _mode.rpm_target     = uint16_temp;

            _modests_rx++;
            break;

        case ECU_ENERGYSTS:
            memcpy(&uint16_temp, &frame.data[0], 2);
            _energy.tank = uint16_temp;             // 0.01 L

            memcpy(&uint16_temp, &frame.data[2], 2);
            _energy.battery = uint16_temp;          // 0.1 Ah

            // endurance keeps its sentinels: 0xFFFF is unknown, 0xFFFE is 65,534 s or
            // more, and neither is scaled as a count.  Stored raw so that the sentinel
            // survives into the log and, in stage 4, to the GCS - an estimate computed
            // here would be a Host-side invention the ICD explicitly does not ask for.
            memcpy(&uint16_temp, &frame.data[4], 2);
            _energy.endurance = uint16_temp;

            // bytes 6-7 are reserved and always zero; nothing reads them

            _energysts_rx++;
            break;

        default:
            break;
    }

    return true;
}


#if HAL_LOGGING_ENABLED
// -------------------------------------------------------------------------
// log : ECUM, ECUE and ECUC
// -------------------------------------------------------------------------
// Three records, for three different questions, on three different deadlines:
//
//   ECUM - what the ECU says it is doing, at the 10 Hz the message is polled at.
//   ECUE - the configured capacities and the endurance estimate, at 1 Hz.
//   ECUC - the generation this backend selected, and the ECU link's own counters.
//
// ECUC IS WRITTEN ON BOTH GENERATIONS, which the other two are not.  It is what closes
// the gap stage 1 left: the selected generation was announced by statustext only, so a
// flight with no GCS attached kept no record of which protocol the FC decided it was
// speaking - the same hole V5.1.25 closed for TM11-TM13.  Its Proto column is the RAW
// latched value, so 0 means "nothing has identified itself and the ECU fallback is in
// use" and is distinguishable from a reported 2.
//
// The other two are written only once a reply of that kind has actually been decoded.
// Without that, the ECU-fallback case - a silent device polled as an ECU - would fill
// the log with zeroed records at 10 Hz that say nothing about any device.
//
// Paced on millis() like AP_PMUCAN's own records, and for the same reason: a tick count
// would make the log rate depend on the driver thread meeting its deadline.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::log(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint64_t now_us = AP_HAL::micros64();

    if ((int32_t)(now_ms - _next_mode_log_ms) >= 0) {
        _next_mode_log_ms += PMUCAN_LOG_INTERVAL_MS;
        if ((int32_t)(now_ms - _next_mode_log_ms) >= 0) {
            _next_mode_log_ms = now_ms + PMUCAN_LOG_INTERVAL_MS;
        }

        if (mode_status_valid()) {
            AP::logger().Write("ECUM", "TimeUS,SSt,OMd,CMd,Alrt,EMd,Gate,IFl,RPMT", "QBBBBBBBH",
                               now_us,
                               _mode.system_state,
                               _mode.operation_mode,
                               _mode.ctrl_mode,
                               _mode.alert_level,
                               _mode.ems_mode,
                               _mode.start_gate,
                               _mode.initial_fail,
                               _mode.rpm_target);
        }
    }

    if ((int32_t)(now_ms - _next_slow_log_ms) < 0) {
        return;
    }
    _next_slow_log_ms += PMUCAN_COUNTER_LOG_MS;
    if ((int32_t)(now_ms - _next_slow_log_ms) >= 0) {
        _next_slow_log_ms = now_ms + PMUCAN_COUNTER_LOG_MS;
    }

    if (energy_status_valid()) {
        // Raw as received: Tank is 0.01 L, Batt 0.1 Ah, Endr seconds with 0xFFFE and
        // 0xFFFF as sentinels.  Scaling here would hide the sentinels behind a float.
        AP::logger().Write("ECUE", "TimeUS,Tank,Batt,Endr", "QHHH",
                           now_us,
                           _energy.tank,
                           _energy.battery,
                           _energy.endurance);
    }

    AP::logger().Write("ECUC", "TimeUS,Proto,RtrTx,RtrErr,ModeRx,EnrgRx,Short", "QBIIIII",
                       now_us,
                       (uint8_t)_pmu.protocol_raw(),
                       _rtr_tx,
                       _rtr_err,
                       _modests_rx,
                       _energysts_rx,
                       _short_frames);
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_PMUCAN_ECU_ENABLED
