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
#pragma once

#include "AP_PMUCAN_config.h"

#if AP_PMUCAN_ECU_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/CANIface.h>
#include <AP_Logger/AP_Logger_config.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_PMUCAN;

// PMU-005a STAGE 4 : THE TWO ECU TELEMETRY MESSAGES, AND THE FLAG THAT GATES THEM.
//
// Produced here on the CAN thread and sent by GCS_PNU.cpp, which is the same split every
// other PNU message uses - PMU_Status, SAR_Status, CAM_ATTITUDE_STATUS.  The alternative,
// building the payload inside the GCS sender, is what V5.1.25 removed from TM11-TM13:
// it makes the content depend on how many ground stations are attached.
//
// ECU_TM_Enabled IS THE COMPATIBILITY GATE, and it is deliberately not a function of
// protocol().  It follows protocol_raw() == PROTO_ECU_V2, so VERSTS must have POSITIVELY
// confirmed ECU software 2.00 or higher: a legacy PMU and the unconfirmed ECU_V2 CAN
// fallback both leave it false and neither message is ever queued.  The CAN fallback
// itself is unchanged - it still polls MODESTS and ENERGYSTS - because what an old KGCS
// cannot tolerate is an unknown MAVLink id, not a CAN poll.
extern mavlink_sys_icd_flcc_gcs_ecu_mode_status_t    ECU_Mode_Status;
extern mavlink_sys_icd_flcc_gcs_ecu_energy_status_t  ECU_Energy_Status;
extern bool                                          ECU_TM_Enabled;

// Data_Valid bits, shared by TM60 and TM61 so that bit 0 and bit 1 mean the same thing
// in both.  Bit 2 exists only in TM61.
#define ECU_VALID_SEEN      (1U << 0)   // at least one reply of this kind has been decoded
#define ECU_VALID_FRESH     (1U << 1)   // the last one is inside this message's threshold
#define ECU_VALID_SOC       (1U << 2)   // TM61 only: BATSTS GAGE is available and fresh

// PMU-005a STAGE 3 : THE TWO MESSAGES THAT EXIST ONLY ON THE ECU.
//
// MODESTS (0x10000150) and ENERGYSTS (0x10000160) are new in ICD V2.0 and are the only
// ECU additions that are not a reinterpretation of something the legacy PMU already
// sends.  That is why they are a separate file and the HOURCNT width was not: stage 2 was
// one memcpy length inside an existing decode, this is two messages the legacy device has
// never heard of.
//
// NOT A BACKEND.  This is a member of AP_PMUCAN, driven by its update() and its
// handle_frame(), and it sends through it.  AP_PNU_CAN dispatches a frame to each backend
// until one claims it, so a second backend claiming PMU ids would have to be ordered
// against the first; and backends are constructed before any CAN frame exists, so one
// could not have been selected by generation anyway.
//
// INFORMATIONAL ONLY, and the ICD says so.  MODESTS is still under development, the ECU
// enforces its own start permissions, and nothing here feeds an FC decision: the decoded
// values are written to the flight log and will reach KGCS in stage 4.  There is
// deliberately no interlock on start_gate or initial_fail.
class AP_PMUCAN_ECU {
public:
    AP_PMUCAN_ECU(AP_PMUCAN &pmu) : _pmu(pmu) {}

    CLASS_NO_COPY(AP_PMUCAN_ECU);

    // Driven by AP_PMUCAN::update() with the driver's tick.  Polls on the phases the
    // five legacy slots leave free, and only while the ECU generation is selected.
    void update(uint32_t tick);

    // Offered every extended data frame AP_PMUCAN did not recognise as one of its five
    // legacy status ids.  Returns true for the two ECU ids, claimed or not decoded.
    bool handle_frame(const AP_HAL::CANFrame &frame);

#if HAL_LOGGING_ENABLED
    void log(void);                             // ECUM, ECUE and ECUC, self-paced
#endif

    // PMU-005a stage 4 : fill the two globals above and queue TM60 / TM61.  Called from
    // AP_PMUCAN's existing 10 Hz and 1 Hz branches rather than from a timer of their own,
    // so the ECU telemetry shares TM1's and the state report's cadence exactly.  Both
    // return without queueing anything unless ECU_TM_Enabled.
    void publish_mode_tm(void);                 // 10 Hz, TM60
    void publish_energy_tm(void);               // 1 Hz, TM61

    // Decoded MODESTS, raw as received.  Undefined codes are NOT mapped onto the nearest
    // defined state - the ICD requires them to be carried through as the raw value - so
    // these are plain integers and not enums.
    struct ModeStatus {
        uint8_t  system_state;                  // byte 0, low nibble
        uint8_t  operation_mode;                // byte 0, high nibble
        uint8_t  ctrl_mode;                     // byte 1
        uint8_t  alert_level;                   // byte 2
        uint8_t  ems_mode;                      // byte 3
        uint8_t  start_gate;                    // byte 4, result of the last Start
        uint8_t  initial_fail;                  // byte 5, bitfield
        uint16_t rpm_target;                    // bytes 6-7, 1 rpm per count
    };

    // Decoded ENERGYSTS, raw as received.  endurance keeps its sentinels: 0xFFFF is
    // unknown (nothing burning) and 0xFFFE is 65,534 s or more, and neither may be
    // scaled as an ordinary count - the display rule is the GCS's, stage 4.
    struct EnergyStatus {
        uint16_t tank;                          // bytes 0-1, 0.01 L
        uint16_t battery;                       // bytes 2-3, 0.1 Ah
        uint16_t endurance;                     // bytes 4-5, 1 s, with sentinels
    };

    const ModeStatus &mode_status(void) const { return _mode; }
    const EnergyStatus &energy_status(void) const { return _energy; }
    bool mode_status_valid(void) const { return _modests_rx > 0; }
    bool energy_status_valid(void) const { return _energysts_rx > 0; }

private:

    static constexpr unsigned ECU_MODESTS   = 0x10000150;
    static constexpr unsigned ECU_ENERGYSTS = 0x10000160;

    // ICD V2.0 : both replies are DLC 8, and so are the RTR requests for them - the ECU
    // sections specify RTR DLC 8 where the legacy ICD left it unstated.
    static constexpr uint8_t ECU_STS_DLC = 8U;

    // The free phases of AP_PMUCAN's 10-tick window.  The five legacy slots fire on the
    // EVEN phases (stride 2), so 1, 3, 5, 7 and 9 are unused; MODESTS takes phase 1 of
    // every window for its 10 Hz, ENERGYSTS phase 3 of every hundredth tick for its 1 Hz.
    // Nothing here can displace a legacy slot, which is the point: PNU-ISSUE(D7)'s
    // interlock is calibrated on engineonoffstate() being reached at exactly 10 Hz.
    static constexpr uint32_t MODESTS_PHASE   = 1U;
    static constexpr uint32_t ENERGYSTS_PHASE = 3U;

    void poll(uint32_t can_id);

    AP_PMUCAN &_pmu;

    // The freshness thresholds behind Data_Valid bit 1.  MODESTS is polled at 10 Hz, so
    // 500 ms is five missed replies; ENERGYSTS is polled at 1 Hz, so 2500 ms is two.
    // Both are deliberately loose enough that ordinary scheduling jitter never clears the
    // bit and tight enough that a stopped ECU is reported within a GCS display update.
    static constexpr uint32_t MODESTS_FRESH_MS   = 500U;
    static constexpr uint32_t ENERGYSTS_FRESH_MS = 2500U;

    // TM61 bit 2 has its own threshold because its source is not ENERGYSTS: BATSTS is a
    // legacy status message polled at 10 Hz, so it gets the 10 Hz number.
    static constexpr uint32_t GAGE_FRESH_MS      = 500U;

    // age in ms, saturating at 65535 - the TM field is uint16 and an ECU that answered
    // 66 seconds ago is no more usable than one that answered 65
    static uint16_t age_ms(uint32_t now_ms, uint32_t stamp_ms);

    ModeStatus   _mode   {};
    EnergyStatus _energy {};

    // When each kind of reply last arrived.  Zero is never read as a timestamp: the
    // _rx counters below decide whether a reply has ever been seen, and until one has,
    // the TM reports the maximum age with Data_Valid bit 0 clear rather than a zero age
    // that would read as perfectly fresh.
    uint32_t _last_modests_ms   {};
    uint32_t _last_energysts_ms {};

    // ECU-ONLY COUNTERS, deliberately not PMUC's.  PMUC's RtrTx means "the five legacy
    // polls", at a rate anyone reading a log can predict from the ICD; folding the ECU
    // polls into it would change what every existing PMUC record means.  These are
    // cumulative for the same reason PMUC's are - the useful reading is a difference.
    uint32_t _rtr_tx    {};
    uint32_t _rtr_err   {};
    uint32_t _modests_rx    {};
    uint32_t _energysts_rx  {};
    uint32_t _short_frames  {};                 // ECU replies rejected for a short DLC

#if HAL_LOGGING_ENABLED
    uint32_t _next_mode_log_ms  {};
    uint32_t _next_slow_log_ms  {};
#endif
};

#endif  // AP_PMUCAN_ECU_ENABLED
