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
#include <GCS_MAVLink/GCS.h>
#include <string.h>


// PMU-005a stage 4 : the two ECU telemetry payloads and their compatibility gate.
// Zero-initialised, and ECU_TM_Enabled false, so an aircraft that never identifies an
// ECU never queues either message - which is the whole of the KGCS compatibility rule.
mavlink_sys_icd_flcc_gcs_ecu_mode_status_t    ECU_Mode_Status   = {0};
mavlink_sys_icd_flcc_gcs_ecu_energy_status_t  ECU_Energy_Status = {0};
bool                                          ECU_TM_Enabled    = false;


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
    // PMU-005a stage 4 : TWO DIFFERENT GENERATION TESTS, ON PURPOSE.
    //
    // The CAN polls below use protocol(), which applies the missing-VERSTS fallback, and
    // that behaviour is unchanged from stage 3 - a device that has not identified itself
    // is still polled as an ECU.
    //
    // MAVLink uses protocol_raw(), which does not.  An old KGCS reports an error on an
    // id it does not know, so a message must not be transmitted on a guess; a CAN poll to
    // a device that ignores it costs nothing.  That asymmetry is the approved policy.
    ECU_TM_Enabled = (_pmu.protocol_raw() == PROTO_ECU_V2);

    // PMU-005a stage 5 : BEFORE the generation gate below, deliberately.  A TC60 that
    // arrives on a legacy aircraft must be refused with a reason the operator can see -
    // PNU-ISSUE(D6) - and that cannot happen inside a branch legacy never enters.
    service_config();

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
            _last_modests_ms = AP_HAL::millis();
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
            _last_energysts_ms = AP_HAL::millis();
            break;

        default:
            break;
    }

    return true;
}


// -------------------------------------------------------------------------
// age_ms : how long ago, saturating
// -------------------------------------------------------------------------
uint16_t AP_PMUCAN_ECU::age_ms(uint32_t now_ms, uint32_t stamp_ms)
{
    const uint32_t age = now_ms - stamp_ms;
    return (age > 65535U) ? 65535U : (uint16_t)age;
}


// -------------------------------------------------------------------------
// publish_mode_tm : TM60, MODESTS as received
// -------------------------------------------------------------------------
// RAW, FIELD FOR FIELD.  The ICD requires undefined codes to be reported as received and
// not mapped onto the nearest defined state, so there is nothing to convert and nothing
// to interpret here - KGCS applies the scales and the code tables.
//
// STALE DATA HOLDS ITS LAST VALUE.  Zero-filling would be worse than useless: zero is a
// value the ECU itself transmits, and the ICD's own rule is that it means "unknown", so a
// zeroed field and a genuine one are indistinguishable.  Age and Data_Valid carry the
// difference instead, and before the first reply the age is reported as its maximum
// rather than as zero - which would otherwise read as perfectly fresh.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::publish_mode_tm(void)
{
    if (!ECU_TM_Enabled) {
        return;
    }

    const bool     seen = mode_status_valid();
    const uint16_t age  = seen ? age_ms(AP_HAL::millis(), _last_modests_ms) : 65535U;

    ECU_Mode_Status.RPM_Target     = _mode.rpm_target;
    ECU_Mode_Status.Mode_Age       = age;
    ECU_Mode_Status.System_State   = _mode.system_state;
    ECU_Mode_Status.Operation_Mode = _mode.operation_mode;
    ECU_Mode_Status.Ctrl_Mode      = _mode.ctrl_mode;
    ECU_Mode_Status.Alert_Level    = _mode.alert_level;
    ECU_Mode_Status.EMS_Mode       = _mode.ems_mode;
    ECU_Mode_Status.Start_Gate     = _mode.start_gate;
    ECU_Mode_Status.Initial_Fail   = _mode.initial_fail;
    ECU_Mode_Status.Data_Valid     = (seen ? ECU_VALID_SEEN : 0U) |
                                     ((seen && (age <= MODESTS_FRESH_MS)) ? ECU_VALID_FRESH : 0U);

    gcs().send_message(MSG_ECU_MODE_STATUS);
}


// -------------------------------------------------------------------------
// publish_energy_tm : TM61, ENERGYSTS as received, plus the SOC readback
// -------------------------------------------------------------------------
// endurance keeps its sentinels.  0xFFFF is "unknown - nothing is being burned" and
// 0xFFFE is "65,534 s or more"; both override the scale, so converting either into a
// duration here would destroy the distinction the ICD spends a paragraph on.
//
// Battery_SOC comes from BATSTS GAGE, which is a LEGACY status message decoded on both
// generations.  It is carried here rather than added to TM1 because TM1 is the message an
// existing KGCS parses, and the approved rule is to add messages rather than extend one.
// Its own validity bit is separate from ENERGYSTS's: BATSTS arrives at 10 Hz and
// ENERGYSTS at 1 Hz, so one can be fresh while the other is not.
//
// Cfg_Result and Cfg_Attempts are the two MAVLink2 extension fields.  Stage 4 leaves them
// at NONE / 0 - stage 5 is what moves them - and because they are extensions they are
// outside this message's CRC_EXTRA, so stage 5 cannot invalidate a KGCS built on stage 4.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::publish_energy_tm(void)
{
    if (!ECU_TM_Enabled) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const bool     seen   = energy_status_valid();
    const uint16_t age    = seen ? age_ms(now_ms, _last_energysts_ms) : 65535U;

    const bool soc_seen  = _pmu.batsts_gage_seen();
    const bool soc_fresh = soc_seen &&
                           ((now_ms - _pmu.batsts_gage_ms()) <= GAGE_FRESH_MS);

    ECU_Energy_Status.Tank        = _energy.tank;
    ECU_Energy_Status.Battery     = _energy.battery;
    ECU_Energy_Status.Endurance   = _energy.endurance;
    ECU_Energy_Status.Energy_Age  = age;
    ECU_Energy_Status.Battery_SOC = soc_fresh ? _pmu.batsts_gage() : 0;
    ECU_Energy_Status.Data_Valid  = (seen ? ECU_VALID_SEEN : 0U) |
                                    ((seen && (age <= ENERGYSTS_FRESH_MS)) ? ECU_VALID_FRESH : 0U) |
                                    (soc_fresh ? ECU_VALID_SOC : 0U);

    // PMU-005a stage 5, in the extension fields.  LATCHED: the last transaction's outcome
    // stays here until another one is accepted, so a ground station that attached after
    // the operator pressed the button still learns what happened.  A one-shot statustext
    // does not survive that, which is the lesson the 2026-10-19 bench session taught.
    ECU_Energy_Status.Cfg_Result   = _cfg_result;
    ECU_Energy_Status.Cfg_Attempts = _cfg_attempts;

    gcs().send_message(MSG_ECU_ENERGY_STATUS);
}


// =========================================================================
// PMU-005a stage 5 : the ENERGYCFG transaction
// =========================================================================
// ONE AT A TIME, BOUNDED, AND NEVER REPLAYED.  The whole of it is: one operator action
// produces one accepted command, which produces at most three identical frames, which
// either read back or do not.  There is no queue, no transaction id and no persistence -
// _cfg_seq_seen matches gcs().ECU_Cfg_Seq at boot, so a reconnect replays nothing and
// neither does a reboot.
//
// IT IS NOT AN ACKNOWLEDGMENT PROTOCOL.  The ECU defines none and this does not invent
// one: "verified" here means the FC compared the values it sent against ENERGYSTS,
// ENGSTS and BATSTS frames that arrived after the command did.  Cfg_Result says which of
// those two things happened.
//
// IT DOES NOT TOUCH THE ENGINE.  Step 2c's state, counters and windows are not read and
// not written anywhere below; the only MODESTS field consulted is operation_mode, and
// that is this command's own ICD send condition, not a Start interlock.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::service_config(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint8_t  seq    = gcs().ECU_Cfg_Seq;

    if (seq != _cfg_seq_seen) {
        _cfg_seq_seen = seq;

        if (_cfg_active) {
            // A SECOND COMMAND DURING A TRANSACTION IS REFUSED, NOT QUEUED AND NOT
            // SWALLOWED.  Overwriting would retry a payload the operator never saw
            // confirmed; reporting success would be a lie.  The running transaction
            // keeps its own payload, its attempt count and its deadline.
            _cfg_busy++;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "ECU: ENERGYCFG busy (attempt %u), TC60 ignored",
                          (unsigned)_cfg_attempts);
        } else {
            const mavlink_sys_icd_gcs_flcc_ecu_energy_cfg_t cfg = gcs().ECU_Cfg;
            uint8_t why = ECU_CFG_NONE;
#if HAL_LOGGING_ENABLED
            // EVERY accepted or refused command writes an ECUX line, even when its
            // verdict repeats the one before it.  cfg_log_state() is otherwise driven by
            // a CHANGE of result and attempt count, and two identical refusals in a row -
            // the same value typed twice, the same mode refusal twice - would have left
            // only one record of two operator actions.
            _cfg_log_force = true;
#endif

            if (cfg_validate(cfg, now_ms, why)) {
                cfg_begin(cfg, now_ms);
            } else {
                // The refused values are recorded before the result, so the ECUX record
                // below says WHICH command was refused rather than leaving the previous
                // transaction's numbers beside the new verdict.
                _cfg_tank     = cfg.Tank;
                _cfg_fuel     = cfg.Fuel;
                _cfg_batt     = cfg.Battery;
                _cfg_soc      = cfg.SOC;
                _cfg_attempts = 0;
                cfg_finish(why);
                switch (why) {
                case ECU_CFG_REJECTED_NOT_ECU:
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ECU: TC60 needs a confirmed ECU v2.00");
                    break;
                case ECU_CFG_REJECTED_MODE:
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ECU: TC60 refused, mode %u not STANDBY",
                                  (unsigned)_mode.operation_mode);
                    break;
                default:
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ECU: TC60 refused, value out of range");
                    break;
                }
            }
        }
    }

    if (_cfg_active) {
        // ICD rule 4 : cancel the moment operation_mode leaves DO_NOTHING or STANDBY.
        // Only a FRESH MODESTS can cancel - a stale one would abandon the transaction
        // every time the ECU missed a reply.
        if (cfg_mode_known(now_ms) && !cfg_mode_permits()) {
            cfg_finish(ECU_CFG_CANCELLED_MODE_CHANGE);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ECU: ENERGYCFG cancelled, mode %u",
                          (unsigned)_mode.operation_mode);
        } else if (_cfg_awaiting_tx) {
            cfg_transmit(now_ms);
        } else {
            cfg_check_readback(now_ms);
        }
    }

#if HAL_LOGGING_ENABLED
    cfg_log_state();
#endif
}


// operation_mode is one of the two the ECU accepts ENERGYCFG in
bool AP_PMUCAN_ECU::cfg_mode_permits(void) const
{
    return (_mode.operation_mode == CFG_OPMODE_DO_NOTHING) ||
           (_mode.operation_mode == CFG_OPMODE_STANDBY);
}


// is there a MODESTS recent enough to decide that question at all
bool AP_PMUCAN_ECU::cfg_mode_known(uint32_t now_ms) const
{
    return mode_status_valid() && ((now_ms - _last_modests_ms) <= MODESTS_FRESH_MS);
}


// -------------------------------------------------------------------------
// cfg_validate : every ICD rule that can be decided before transmitting
// -------------------------------------------------------------------------
// ATOMIC AND NEVER CLAMPED.  One bad field refuses the whole command, as the ECU itself
// does, and no value is coerced into range - a silently corrected tank capacity is worse
// than a refused one.  Same shape as TC50's "values 3 through 255 are undefined".
//
// 65535 is checked FIRST in every field, so "keep the current value" is never compared
// against a range it was never meant to satisfy.  None of the four ranges reaches 65535,
// so there is no value that is both a setting and a keep.
//
// THE MODE GATE IS CONDITIONAL ON MODESTS BEING USABLE.  MODESTS is still under
// development and a real ECU may not answer it at all; refusing every ENERGYCFG in that
// case would make the aircraft unconfigurable for a message that is informational by the
// ICD's own description.  With no fresh MODESTS the frame goes out and the ECU - which
// is the final authority - applies or ignores it.
// -------------------------------------------------------------------------
bool AP_PMUCAN_ECU::cfg_validate(const mavlink_sys_icd_gcs_flcc_ecu_energy_cfg_t &cfg,
                                 uint32_t now_ms, uint8_t &why) const
{
    if (!ECU_TM_Enabled) {
        // Legacy, and the unconfirmed ECU_V2 fallback, both land here.  The CAN fallback
        // still polls MODESTS and ENERGYSTS; what it must not do is write to a device
        // whose generation is a guess.
        why = ECU_CFG_REJECTED_NOT_ECU;
        return false;
    }

    why = ECU_CFG_REJECTED_RANGE;

    if ((cfg.Tank != ECU_CFG_KEEP) &&
        ((cfg.Tank < CFG_TANK_MIN) || (cfg.Tank > CFG_TANK_MAX))) {
        return false;
    }
    if ((cfg.Battery != ECU_CFG_KEEP) &&
        ((cfg.Battery < CFG_BATT_MIN) || (cfg.Battery > CFG_BATT_MAX))) {
        return false;
    }
    if ((cfg.SOC != ECU_CFG_KEEP) &&
        ((cfg.SOC < CFG_SOC_MIN) || (cfg.SOC > CFG_SOC_MAX))) {
        return false;
    }
    if (cfg.Fuel != ECU_CFG_KEEP) {
        // "fuel may not exceed the supplied or retained tank".  The supplied one if there
        // is one, otherwise the last ENERGYSTS readback.  If neither is known the FC does
        // NOT guess a capacity - it sends, and the ECU refuses the whole frame if the
        // retained tank is smaller.
        const uint16_t tank = (cfg.Tank != ECU_CFG_KEEP) ? cfg.Tank
                              : (energy_status_valid() ? _energy.tank : 0U);
        if ((tank != 0U) && (cfg.Fuel > tank)) {
            return false;
        }
    }

    if (cfg_mode_known(now_ms) && !cfg_mode_permits()) {
        why = ECU_CFG_REJECTED_MODE;
        return false;
    }

    why = ECU_CFG_NONE;
    return true;
}


// -------------------------------------------------------------------------
// cfg_begin : accept one command and build the eight bytes once
// -------------------------------------------------------------------------
// The payload is built HERE and kept, because every retry must be the IDENTICAL frame
// (ICD rule 3).  Rebuilding it from gcs().ECU_Cfg on each attempt would silently pick up
// a TC60 that arrived in between - which is exactly the overwrite the busy refusal above
// exists to prevent.
//
// PNU-ISSUE(D15) : LSB first on both sides, so the four fields are stored byte by byte in
// little-endian order with no swap.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::cfg_begin(const mavlink_sys_icd_gcs_flcc_ecu_energy_cfg_t &cfg,
                              uint32_t now_ms)
{
    _cfg_tank = cfg.Tank;
    _cfg_fuel = cfg.Fuel;
    _cfg_batt = cfg.Battery;
    _cfg_soc  = cfg.SOC;

    _cfg_payload[0] = (uint8_t)(cfg.Tank    & 0xFFU);
    _cfg_payload[1] = (uint8_t)(cfg.Tank    >> 8);
    _cfg_payload[2] = (uint8_t)(cfg.Fuel    & 0xFFU);
    _cfg_payload[3] = (uint8_t)(cfg.Fuel    >> 8);
    _cfg_payload[4] = (uint8_t)(cfg.Battery & 0xFFU);
    _cfg_payload[5] = (uint8_t)(cfg.Battery >> 8);
    _cfg_payload[6] = (uint8_t)(cfg.SOC     & 0xFFU);
    _cfg_payload[7] = (uint8_t)(cfg.SOC     >> 8);

    _cfg_active      = true;
    _cfg_awaiting_tx = true;
    _cfg_attempts    = 0;
    _cfg_result      = ECU_CFG_IN_PROGRESS;
    _cfg_arm_ms      = now_ms;
    _cfg_accepted++;

    cfg_transmit(now_ms);
}


// -------------------------------------------------------------------------
// cfg_transmit : put the frame on the bus, or give up on the bus
// -------------------------------------------------------------------------
// A FAILED SUBMISSION IS NOT AN ATTEMPT.  Cfg_Attempts counts frames that reached the
// interface, because that is what the ICD's "no more than 3 total transmissions" counts;
// a full mailbox for one tick is a bus condition and is simply retried next tick.  After
// CFG_TX_RETRY_MS of that the transaction is abandoned with TX_FAIL rather than retried
// forever against a dead bus.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::cfg_transmit(uint32_t now_ms)
{
    if (_pmu.send_ecu_data(ECU_ENERGYCFG | AP_HAL::CANFrame::FlagEFF,
                           _cfg_payload, ECU_CFG_DLC)) {
        _cfg_attempts++;
        _cfg_tx_ms       = now_ms;
        _cfg_awaiting_tx = false;
        return;
    }

    if ((now_ms - _cfg_arm_ms) >= CFG_TX_RETRY_MS) {
        cfg_finish(ECU_CFG_TX_FAIL);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ECU: ENERGYCFG could not be sent");
    }
}


// has every readback this command actually needs arrived since @deadline_ms
bool AP_PMUCAN_ECU::cfg_readback_ready(uint32_t deadline_ms) const
{
    // Only the supplied fields are waited for.  A command that sets nothing but the
    // battery capacity must not hang waiting for an ENGSTS it does not read.
    if ((_cfg_tank != ECU_CFG_KEEP) || (_cfg_batt != ECU_CFG_KEEP)) {
        if (!energy_status_valid() ||
            ((int32_t)(_last_energysts_ms - deadline_ms) < 0)) {
            return false;
        }
    }
    if (_cfg_fuel != ECU_CFG_KEEP) {
        if (!_pmu.engsts_seen() ||
            ((int32_t)(_pmu.engsts_ms() - deadline_ms) < 0)) {
            return false;
        }
    }
    if (_cfg_soc != ECU_CFG_KEEP) {
        if (!_pmu.batsts_gage_seen() ||
            ((int32_t)(_pmu.batsts_gage_ms() - deadline_ms) < 0)) {
            return false;
        }
    }
    return true;
}


// -------------------------------------------------------------------------
// cfg_readback_ok : the ICD's four comparisons, each at its own tolerance
// -------------------------------------------------------------------------
//   tank    - ENERGYSTS.tank,    exact
//   battery - ENERGYSTS.battery, exact
//   fuel    - ENGSTS.GAS, an INTEGER PERCENT of the tank.  floor(100 * fuel / tank), or
//             one point less, which the ICD allows for the ECU's own float rounding.
//             This is NOT a 0.01 L verification and must never be reported as one: a
//             quarter of a litre in an 8 L tank is three percent, and a litre either way
//             can hide inside one count on a large tank.
//   soc     - BATSTS.GAGE, within one raw count, which is 0.1 percentage point.
// -------------------------------------------------------------------------
bool AP_PMUCAN_ECU::cfg_readback_ok(void) const
{
    if ((_cfg_tank != ECU_CFG_KEEP) && (_energy.tank != _cfg_tank)) {
        return false;
    }
    if ((_cfg_batt != ECU_CFG_KEEP) && (_energy.battery != _cfg_batt)) {
        return false;
    }
    if (_cfg_fuel != ECU_CFG_KEEP) {
        const uint16_t tank = (_cfg_tank != ECU_CFG_KEEP) ? _cfg_tank
                              : (energy_status_valid() ? _energy.tank : 0U);
        if (tank == 0U) {
            return false;           // no capacity is known, so no percentage can be formed
        }
        const int32_t expect = (int32_t)((100UL * (uint32_t)_cfg_fuel) / (uint32_t)tank);
        const int32_t gas    = (int32_t)PMU_Status.Fuel_Quantity;
        if ((gas != expect) && (gas != (expect - 1))) {
            return false;
        }
    }
    if (_cfg_soc != ECU_CFG_KEEP) {
        const int32_t diff = (int32_t)_pmu.batsts_gage() - (int32_t)_cfg_soc;
        if ((diff > 1) || (diff < -1)) {
            return false;
        }
    }
    return true;
}


// -------------------------------------------------------------------------
// cfg_check_readback : wait at least a second, then compare fresh data only
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::cfg_check_readback(uint32_t now_ms)
{
    const uint32_t deadline_ms = _cfg_tx_ms + CFG_READBACK_MIN_MS;

    if ((int32_t)(now_ms - deadline_ms) < 0) {
        return;                     // the ICD's mandatory one second has not passed
    }

    if (!cfg_readback_ready(deadline_ms)) {
        if ((now_ms - _cfg_tx_ms) >= CFG_READBACK_TIMEOUT_MS) {
            // Nothing to compare against.  That is an unverified attempt, not a silent
            // success, and it is treated exactly as a mismatch - retry, then report.
            cfg_attempt_failed(now_ms, "no readback");
        }
        return;
    }

    if (cfg_readback_ok()) {
        cfg_finish(ECU_CFG_OK);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ECU: ENERGYCFG verified after %u frame%s",
                      (unsigned)_cfg_attempts, (_cfg_attempts == 1) ? "" : "s");
        return;
    }

    cfg_attempt_failed(now_ms, "readback differs");
}


// -------------------------------------------------------------------------
// cfg_attempt_failed : resend the identical frame, or stop and say what came back
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::cfg_attempt_failed(uint32_t now_ms, const char *why)
{
    if (_cfg_attempts < CFG_MAX_ATTEMPTS) {
        _cfg_awaiting_tx = true;
        _cfg_arm_ms      = now_ms;
        cfg_transmit(now_ms);
        return;
    }

    cfg_finish(ECU_CFG_MISMATCH_AFTER_3);

    // The readback values go out WITH the failure, because "it did not take" is not
    // actionable on its own - the ICD asks for the values that were read back, and the
    // ECUX record below keeps them whether or not anyone is listening.
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ECU: ENERGYCFG failed (%s) after 3 frames", why);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ECU: read back tank %u batt %u gas %d soc %d",
                  (unsigned)_energy.tank, (unsigned)_energy.battery,
                  (int)PMU_Status.Fuel_Quantity, (int)_pmu.batsts_gage());
}


// end the transaction, keeping the attempt count for TM61 and the log
void AP_PMUCAN_ECU::cfg_finish(uint8_t result)
{
    _cfg_result      = result;
    _cfg_active      = false;
    _cfg_awaiting_tx = false;
}


#if HAL_LOGGING_ENABLED
// -------------------------------------------------------------------------
// cfg_log_state : ECUX, one record per transaction state change
// -------------------------------------------------------------------------
// NOT PERIODIC.  Every change of result or attempt count writes one record, so a whole
// transaction is four or five lines: accepted, one line per frame, then the outcome.
// That is the permanent record the statustexts above cannot be - a message sent before a
// ground station attached is gone, and this is a ground procedure that is often run with
// nobody watching the link.
//
// It carries BOTH SIDES: cTank/cFuel/cBatt/cSOC are what was commanded, raw as sent, and
// rTank/rBatt/rGas/rSOC are what the ECU read back at that moment.  A failure is then
// diagnosable from the log alone.
//
// NO EXISTING RECORD CHANGES.  ECUM, ECUE, ECUC, PMUC, PMUE and TM11-TM13/TM3 keep their
// columns exactly as they were.
// -------------------------------------------------------------------------
void AP_PMUCAN_ECU::cfg_log_state(void)
{
    if (!_cfg_log_force &&
        (_cfg_result == _cfg_logged_result) && (_cfg_attempts == _cfg_logged_att)) {
        return;
    }
    _cfg_log_force     = false;
    _cfg_logged_result = _cfg_result;
    _cfg_logged_att    = _cfg_attempts;

    if (_cfg_result == ECU_CFG_NONE) {
        return;                     // nothing has ever been commanded; no record to write
    }

    AP::logger().Write("ECUX", "TimeUS,Res,Att,cTank,cFuel,cBatt,cSOC,rTank,rBatt,rGas,rSOC",
                       "QBBHHHHHHbh",
                       AP_HAL::micros64(),
                       _cfg_result,
                       _cfg_attempts,
                       _cfg_tank,
                       _cfg_fuel,
                       _cfg_batt,
                       _cfg_soc,
                       _energy.tank,
                       _energy.battery,
                       PMU_Status.Fuel_Quantity,
                       _pmu.batsts_gage());
}
#endif  // HAL_LOGGING_ENABLED


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

    AP::logger().Write("ECUC", "TimeUS,Proto,RtrTx,RtrErr,ModeRx,EnrgRx,Short,CfgAcc,CfgBsy",
                       "QBIIIIIII",
                       now_us,
                       (uint8_t)_pmu.protocol_raw(),
                       _rtr_tx,
                       _rtr_err,
                       _modests_rx,
                       _energysts_rx,
                       _short_frames,
                       // PMU-005a stage 5 : TC60s that started a transaction, and TC60s
                       // refused because one was already running.  Appended to ECUC
                       // rather than given a record of their own - they are counters of
                       // this link, like the six beside them, and ECUC is written on both
                       // generations so a refused TC60 on a legacy aircraft is counted.
                       _cfg_accepted,
                       _cfg_busy);
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_PMUCAN_ECU_ENABLED
