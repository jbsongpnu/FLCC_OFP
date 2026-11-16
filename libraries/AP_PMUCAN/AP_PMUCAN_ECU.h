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

// PMU-005a STAGE 5 : TM61's Cfg_Result.  THESE VALUES ARE ON THE WIRE AND IN FLIGHT
// LOGS, so they are append-only: never renumber one and never reuse a retired one.
//
// OK MEANS THE FC VERIFIED THE READBACK.  The ECU defines no application-layer
// acknowledgment and this firmware does not invent one: every code below describes what
// the FC did and what it then observed in ENERGYSTS, ENGSTS and BATSTS.
enum ECU_CfgResult : uint8_t {
    ECU_CFG_NONE                  = 0,  // no ENERGYCFG has been accepted since boot
    ECU_CFG_IN_PROGRESS           = 1,  // accepted; transmitting or waiting for readback
    ECU_CFG_OK                    = 2,  // every supplied field read back within tolerance
    ECU_CFG_MISMATCH_AFTER_3      = 3,  // three frames sent, readback never agreed
    ECU_CFG_CANCELLED_MODE_CHANGE = 4,  // operation_mode left DO_NOTHING/STANDBY
    ECU_CFG_REJECTED_RANGE        = 5,  // a field was outside the ICD range
    ECU_CFG_REJECTED_MODE         = 6,  // the ECU was not in DO_NOTHING or STANDBY
    ECU_CFG_REJECTED_NOT_ECU      = 7,  // VERSTS has not confirmed ECU 2.00 or higher
    ECU_CFG_TX_FAIL               = 8,  // the frame could not be put on the bus
};

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

    // PMU-005a stage 5 : the ENERGYCFG transaction.  Driven from update(), so it runs on
    // BOTH generations - a TC60 that arrives on a legacy aircraft has to be refused out
    // loud rather than ignored (PNU-ISSUE D6), and that refusal is produced here.
    void service_config(void);

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
    static constexpr unsigned ECU_ENERGYCFG = 0x10000050;   // stage 5, Host -> ECU

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

    // ---- PMU-005a stage 5 : ENERGYCFG ----------------------------------------------
    //
    // ICD ranges, in RAW counts.  None of them can collide with ECU_CFG_KEEP, which is
    // why "keep the current value" needs no separate flag.
    static constexpr uint8_t  ECU_CFG_DLC  = 8U;            // ICD: a short frame is ignored
    static constexpr uint16_t ECU_CFG_KEEP = 0xFFFFU;
    static constexpr uint16_t CFG_TANK_MIN = 10U,   CFG_TANK_MAX = 10000U;
    static constexpr uint16_t CFG_BATT_MIN = 50U,   CFG_BATT_MAX = 1000U;
    static constexpr uint16_t CFG_SOC_MIN  = 0U,    CFG_SOC_MAX  = 1000U;

    // The only two operation_mode values that may receive an ENERGYCFG.  This is the
    // ICD's own send condition for this one command - NOT a new FC interlock, and
    // nothing here gates engine Start.
    static constexpr uint8_t CFG_OPMODE_DO_NOTHING = 1U;
    static constexpr uint8_t CFG_OPMODE_STANDBY    = 3U;

    static constexpr uint8_t CFG_MAX_ATTEMPTS = 3U;         // ICD: three transmissions total

    // THE READBACK WINDOW, and the arithmetic behind the timeout.
    //
    // The ICD says "wait 1 second, then compare".  Taken literally that can compare
    // against an ENERGYSTS that arrived BEFORE the command did, because ENERGYSTS is
    // polled once a second: the readback only proves anything if the frame carrying it
    // arrived after the wait expired, which is what CFG_READBACK_MIN_MS below is used
    // for - as a deadline, not as a sleep.
    //
    // Worst case to satisfy that: 1000 ms of mandatory wait, plus up to one whole
    // ENERGYSTS period (1000 ms) before the next reply, plus scheduling jitter - about
    // 2010 ms with no margin at all.  2500 ms would leave 490 ms and would fail on a
    // single lost reply.  3500 ms absorbs exactly one lost ENERGYSTS and still bounds a
    // completely unverifiable transaction at 3 x 3.5 s.
    static constexpr uint32_t CFG_READBACK_MIN_MS     = 1000U;
    static constexpr uint32_t CFG_READBACK_TIMEOUT_MS = 3500U;

    // How long a frame that cannot be put on the bus keeps being retried before the
    // transaction is abandoned.  A busy mailbox for 200 ms is a bus problem, not a
    // readback problem, so it does NOT consume one of the three ICD attempts - those
    // count frames that actually went out.
    static constexpr uint32_t CFG_TX_RETRY_MS = 200U;

    bool cfg_mode_permits(void) const;
    bool cfg_mode_known(uint32_t now_ms) const;
    bool cfg_validate(const mavlink_sys_icd_gcs_flcc_ecu_energy_cfg_t &cfg,
                      uint32_t now_ms, uint8_t &why) const;
    void cfg_begin(const mavlink_sys_icd_gcs_flcc_ecu_energy_cfg_t &cfg, uint32_t now_ms);
    void cfg_transmit(uint32_t now_ms);
    void cfg_check_readback(uint32_t now_ms);
    bool cfg_readback_ready(uint32_t deadline_ms) const;
    bool cfg_readback_ok(void) const;
    void cfg_attempt_failed(uint32_t now_ms, const char *why);
    void cfg_finish(uint8_t result);
#if HAL_LOGGING_ENABLED
    void cfg_log_state(void);                   // ECUX, on every state change
#endif

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

    // PMU-005a stage 5 : ONE TRANSACTION AT A TIME.  There is no queue and no transaction
    // id: a TC60 that arrives while _cfg_active is refused out loud and the running
    // transaction is left exactly as it was.  _cfg_seq_seen matches gcs().ECU_Cfg_Seq at
    // boot, so nothing is replayed on startup or on reconnect.
    bool     _cfg_active        {};             // a transaction owns the machine
    bool     _cfg_awaiting_tx   {};             // accepted or retrying; frame not yet sent
    uint8_t  _cfg_seq_seen      {};             // last gcs().ECU_Cfg_Seq consumed
    uint8_t  _cfg_result        {};             // ECU_CFG_NONE
    uint8_t  _cfg_attempts      {};             // frames actually put on the bus, 0-3
    uint8_t  _cfg_payload[8]    {};             // the EXACT bytes, kept for the retries
    uint16_t _cfg_tank          {ECU_CFG_KEEP}; // what was supplied, for the comparison
    uint16_t _cfg_fuel          {ECU_CFG_KEEP};
    uint16_t _cfg_batt          {ECU_CFG_KEEP};
    uint16_t _cfg_soc           {ECU_CFG_KEEP};
    uint32_t _cfg_tx_ms         {};             // when the last frame reached the bus
    uint32_t _cfg_arm_ms        {};             // when the current send attempt was armed
    uint32_t _cfg_accepted      {};             // ECUC: TC60s that started a transaction
    uint32_t _cfg_busy          {};             // ECUC: TC60s refused for an active one
#if HAL_LOGGING_ENABLED
    uint8_t  _cfg_logged_result {255};          // forces the first ECUX record
    uint8_t  _cfg_logged_att    {255};
#endif

#if HAL_LOGGING_ENABLED
    uint32_t _next_mode_log_ms  {};
    uint32_t _next_slow_log_ms  {};
#endif
};

#endif  // AP_PMUCAN_ECU_ENABLED
