#pragma once

#include "AP_PNUSAR_config.h"

#include <AP_Common/AP_Common.h>
#include <stdint.h>

// PNU : the SAR's published state, as MAVLink TM 51050 reports it.
//
// PNU-ISSUE(D14) : declared here and defined in AP_PNUSAR.cpp, and BOTH SIT OUTSIDE the
// AP_PNUSAR_ENABLED guard below, because GCS_PNU.cpp will reference them whether or not
// the CAN driver is in the build - AGENTS.md requires the base system to compile with
// optional features disabled.  With no backend they keep their static-init values, which
// read as "Disconnected, never heard from", exactly the truth in that configuration.
struct PNU_SAR_Status {
    uint8_t  status;            // ICD SAR_STATUS: 0 Disconnected, 1 Ready, 2 Stopped,
                                // 3 Started, 4 SAR Not Responding
    uint32_t live_counter;      // FC-maintained count of VALID Heart_Beat receptions
};
extern PNU_SAR_Status SAR_Status;

// CAN_Dn_PNU_SCHK bits - which of ICD 14.3.3's pre-transmission checks are enforced.
// The parameter exists because a bench or an integration test with Lumir has no RTK fix
// and would otherwise be unable to send a single Start.  Relaxing it is announced, once
// per boot and again on every Start that goes out under it: a silently disabled interlock
// is exactly the failure PNU-ISSUE(D6) exists to prevent.
//
// OUTSIDE the guard below for the same reason SAR_Status is: AP_PNU_CAN.cpp's parameter
// table names the default, and that table is compiled whenever the driver is, whether or
// not this backend is.
#define AP_PNUSAR_SCHK_RTK_FIXED    (1U << 0)
#define AP_PNUSAR_SCHK_GPS_AGE      (1U << 1)
#define AP_PNUSAR_SCHK_ATT_AGE      (1U << 2)
#define AP_PNUSAR_SCHK_NAV_HEALTH   (1U << 3)
// The ICD-required set, less the gimbal attitude age - there is no attitude source until
// step 3f, and a check that cannot be performed must not claim to pass.  Anything short
// of this set is "relaxed" and says so, once per boot and on every Start.
#define AP_PNUSAR_SCHK_ICD          (AP_PNUSAR_SCHK_RTK_FIXED | \
                                     AP_PNUSAR_SCHK_GPS_AGE | \
                                     AP_PNUSAR_SCHK_NAV_HEALTH)

// PNU : the 10 Hz navigation snapshot, called from the VEHICLE THREAD.
//
// It is taken there and not on the CAN thread on purpose.  AP_GPS::update() runs on the
// vehicle thread, so a task there is serialised with it and reads one consistent sample;
// the same twenty-odd fields read from the CAN thread could straddle a GPS update and
// produce a bundle built from two different samples, which is precisely what ICD 7.3's
// "same acquisition time" forbids.  The CAN thread then only transmits what it is given.
//
// @nav_failsafe is the vehicle's own navigation/EKF failsafe, which a library cannot see
// - ICD 14.7 makes it an immediate observation termination with no age threshold.
//
// Outside the AP_PNUSAR_ENABLED guard, like SAR_Status above: ArduCopter calls it
// unconditionally, and with no backend it does nothing at all.
void PNU_SAR_nav_snapshot(bool nav_failsafe);

#if AP_PNUSAR_ENABLED

#include <AP_HAL/CANIface.h>
#include <AP_PNU_CAN/AP_PNU_CAN_Backend.h>

// PNU : the SAR radar payload, as specified by SAR_ICD_PNU.md Rev1.0 (repository root).
//
// Classical CAN, STANDARD 11-bit data frames, ids 0x400-0x7FF, 1 Mbit/s.  Nothing here is
// DroneCAN.  The PMU on the same bus speaks 29-bit extended frames and the gimbal speaks
// 11-bit in 0x000-0x2FF, so the three never collide - the ICD confirmed a range this
// project had already reserved before it existed.
//
// STEP 3a IMPLEMENTED THE CONNECTION LAYER: Heart_Beat 0x510, the Live_Counter, the 3 s
// timeout and SAR_STATUS 0 and 1.  STEP 3d ADDS THE COMMAND LAYER: 0x501/0x502 out,
// 0x521/0x522 in, the FC's 2 s response window, SAR_STATUS 2, 3 and 4, and the Start
// interlock of ICD 14.3.3.  The 10 Hz navigation bundle is step 3e.
//
// WHAT THE FC DELIBERATELY DOES NOT DO (ICD 14.5): it never retries a command of its own.
// The GCS owns the retry and is limited to two attempts; an FC retry loop underneath it
// would exceed that limit and is prohibited by the document.
//
// THINGS THE ICD LEAVES TBD, and which this backend therefore does not invent:
//   - 0x503 Observation_Mode_Command and 0x523 Observation_Param_Response payloads.
//     Lumir expects to define them; they are named, never transmitted, never parsed.
//   - Observation_Mode's CAN encoding, which depends on 0x503.  The value is retained in
//     gcs().SAR_Ctrl and goes nowhere near the wire.
//   - the detailed recovery-inhibit feature (ICD 14.8), deliberately deferred.

// ids, SAR_ICD_PNU.md section 5.  All 11-bit standard.
#define AP_PNUSAR_ID_NAV_FIRST      (0x400U)    // GPS_POSITION, first of the 9-frame bundle
#define AP_PNUSAR_ID_NAV_LAST       (0x408U)    // SNAPSHOT_TIME_US, closes the bundle
#define AP_PNUSAR_ID_START_CMD      (0x501U)    // FC -> SAR, DLC 1, payload 0x01
#define AP_PNUSAR_ID_STOP_CMD       (0x502U)    // FC -> SAR, DLC 1, payload 0x01
#define AP_PNUSAR_ID_MODE_CMD       (0x503U)    // RESERVED, payload TBD - never transmitted
#define AP_PNUSAR_ID_HEARTBEAT      (0x510U)    // SAR -> FC, DLC 4, 1 Hz
#define AP_PNUSAR_ID_START_RESP     (0x521U)    // SAR -> FC, DLC 1, payload 0x01
#define AP_PNUSAR_ID_STOP_RESP      (0x522U)    // SAR -> FC, DLC 1, payload 0x01
#define AP_PNUSAR_ID_PARAM_RESP     (0x523U)    // RESERVED, payload TBD - never parsed

// DLCs, SAR_ICD_PNU.md sections 12 and 13.  A frame of the wrong length is rejected and
// counted, never parsed from the same offsets - PNU-ISSUE(D6), which cost the PMU a long
// run of silently stale values.
#define AP_PNUSAR_HEARTBEAT_DLC     (4U)
#define AP_PNUSAR_CMD_DLC           (1U)
#define AP_PNUSAR_RESP_DLC          (1U)
#define AP_PNUSAR_NAV_DLC           (8U)
#define AP_PNUSAR_RESERVED_DLC      (8U)

// SAR_ICD_PNU.md section 13.  BOTH commands carry byte 0x01, and so do both responses.
// The ICD says it twice, in its own words: "Do not encode the Stop payload as 02 merely
// because it is Stop."  One constant for all four removes the place that mistake lives.
#define AP_PNUSAR_CMD_PAYLOAD       (0x01U)

// T_HB, SAR_ICD_PNU.md section 3.3.  At least this long without a valid Heart_Beat means
// Disconnected and navigation off.  The ICD fixes it; it is not tuning.
#define AP_PNUSAR_HB_TIMEOUT_MS     (3000U)

// The FC's command-response window, SAR_ICD_PNU.md section 14.1.  Opened only for a
// command ACTUALLY TRANSMITTED (14.5) and closed exactly once.  Not tuning either: the
// GCS's own 3 s window is sized around it.
#define AP_PNUSAR_RESP_TIMEOUT_MS   (2000U)

// Start interlock thresholds, SAR_ICD_PNU.md sections 14.3.3 and 14.7.  The same two
// numbers drive the automatic-termination checks in step 3e, which is why they live here
// rather than inside the one function that currently reads them.
#define AP_PNUSAR_GPS_AGE_MAX_MS    (300U)
#define AP_PNUSAR_ATT_AGE_MAX_MS    (40U)

// The navigation bundle, SAR_ICD_PNU.md sections 5 and 7.  Nine frames, 0x400 through
// 0x408, eight bytes each: one logical snapshot is 72 payload bytes and NOT one CAN
// frame.  They go out in ascending id order, which is also the ICD's required order -
// and, conveniently, the order CAN arbitration would impose anyway.
#define AP_PNUSAR_NAV_FRAMES        (9U)

// A snapshot older than this when the CAN thread reaches it is DROPPED rather than sent
// late.  ICD 7.2: "Check at acquisition and again before delayed transmission if
// necessary."
//
// ONE BUNDLE INTERVAL, deliberately, and it was 50 ms until measurement said otherwise:
// SITL dropped 6% of bundles at 50 ms because the driver thread's 10 ms loop is not that
// regular.  Anything under a full interval is still a sample well inside every source-age
// limit the ICD sets - 300 ms for GPS - whereas a bundle later than the NEXT snapshot is
// strictly worse than no bundle, because its Snapshot_TimeUS would arrive out of order.
#define AP_PNUSAR_SNAPSHOT_MAX_AGE_MS (100U)

// Rate limit for the bundle-drop report.  Shorter than AP_PNUSAR_WARN_INTERVAL_MS, which
// covers events that should never happen at all; a drop is a timing symptom and is worth
// hearing about while it is still happening.
#define AP_PNUSAR_BUNDLE_WARN_MS    (10000U)

// Sentinels, SAR_ICD_PNU.md 2.3.  "Do not use an otherwise valid value of zero to
// represent invalid data", which is the whole reason these exist as named constants.
#define AP_PNUSAR_INVALID_U16       (0xFFFFU)
#define AP_PNUSAR_INVALID_I16       (INT16_MIN)
#define AP_PNUSAR_INVALID_I32       (INT32_MIN)
#define AP_PNUSAR_INVALID_U32       (0xFFFFFFFFUL)

// GPS_Flags, B7 of 0x406 - SAR_ICD_PNU.md 6.5.  Bit 0 is the LSB.
#define AP_PNUSAR_GPSF_USED         (1U << 0)   // the sample came from the AP_GPS primary
#define AP_PNUSAR_GPSF_VV           (1U << 1)   // vertical velocity is available
#define AP_PNUSAR_GPSF_FRESH        (1U << 2)   // source age within 300 ms
#define AP_PNUSAR_GPSF_TIME_VALID   (1U << 3)   // GMS/GWk are valid
#define AP_PNUSAR_GPSF_ENC_ERROR    (1U << 4)   // a RANGE error, not merely unavailable
#define AP_PNUSAR_GPSF_AEL_VALID    (1U << 5)   // AEl in 0x407 is actually available
#define AP_PNUSAR_GPSF_RTCM_STATS   (1U << 6)   // the API provides RTCM fragment counts

// ATT_Flags, B7 of 0x404 - SAR_ICD_PNU.md 6.5.  All four are zero until step 3f: there
// is no attitude source, and ICD 16.4 forbids marking a substitute valid.
#define AP_PNUSAR_ATTF_ROLL_VALID   (1U << 0)
#define AP_PNUSAR_ATTF_PITCH_VALID  (1U << 1)
#define AP_PNUSAR_ATTF_YAW_VALID    (1U << 2)
#define AP_PNUSAR_ATTF_FRESH        (1U << 3)

// Health is evaluated at this rate, matching the PMU's leaky bucket so both devices on
// the shared bus are judged on the same clock.
#define AP_PNUSAR_HEALTH_CHECK_TICKS (2U)       // 50 Hz at AP_PNU_CAN_LOOP_HZ

// Rate limit for the repeated-cause reports below.  One limiter per cause - sharing one
// lets a permanent condition crowd out a genuine fault.
#define AP_PNUSAR_WARN_INTERVAL_MS  (60000U)

// ICD SAR_STATUS values, section 14.1.  START_PENDING and STOP_PENDING are internal and
// are deliberately NOT in this enum - the ICD says they are not MAVLink states.
enum class PNUSAR_State : uint8_t {
    DISCONNECTED   = 0,
    READY          = 1,
    STOPPED        = 2,
    STARTED        = 3,
    NOT_RESPONDING = 4,
};

// The internal pending states the ICD names but keeps off the wire.
enum class PNUSAR_Pending : uint8_t {
    NONE  = 0,
    START = 1,
    STOP  = 2,
};

// One navigation snapshot, ALREADY ENCODED.
//
// The vehicle thread packs the nine frames; the CAN thread only writes them to the bus.
// Encoding where the data is read is what makes ICD 7.3's "same acquisition time" true
// by construction - there is no window in which half a bundle could be rebuilt from a
// newer GPS sample - and it keeps float arithmetic off the CAN thread entirely.
struct PNUSAR_Bundle {
    uint8_t  data[AP_PNUSAR_NAV_FRAMES][8];
    uint64_t time_us;                   // Snapshot_TimeUS, also the staleness reference
    bool     ready;                     // packed and not yet transmitted
};

class AP_PNUSAR : public AP_PNU_CAN_Backend {
public:
    using AP_PNU_CAN_Backend::AP_PNU_CAN_Backend;

    CLASS_NO_COPY(AP_PNUSAR);

    // AP_PNU_CAN_Backend
    bool init(void) override;
    void update(uint32_t tick) override;
    bool handle_frame(const AP_HAL::CANFrame &frame) override;
    bool healthy(void) const override;
    const char *name(void) const override { return "SAR"; }

private:

    void check_health(void);
    void check_ctrl(void);
    void check_pending(void);
    void report_schk_once(void);

    // ONE place computes SAR_STATUS, because the ICD states a priority between the
    // sources rather than a last-writer-wins rule - see the definition.
    void publish_state(void);

    void handle_start_request(void);
    void handle_stop_request(void);
    const char *start_blocked_reason(void) const;
    bool send_command(uint32_t can_id, PNUSAR_Pending pending, bool automatic = false);

    // AP_PNUSAR_Nav.cpp - the 10 Hz navigation bundle, SAR_ICD_PNU.md 5, 6 and 7
public:
    // The ICD's conversion and serialisation rules, as functions.
    //
    // PUBLIC AND STATIC so that the worked examples in SAR_ICD_PNU.md section 9.1 can be
    // run against the code that actually packs the bundle, rather than against a second
    // implementation written for the test.  Those vectors give exact expected bytes for
    // five of the nine frames; a codec that agrees with them agrees with Lumir.
    //
    // @range_err is set only for a value that CANNOT BE REPRESENTED - NaN, infinity or
    // out of range after scaling - and becomes GPS_Flags bit 4.  A field the source
    // simply does not provide takes its sentinel without setting it (ICD 6.5).
    static uint16_t encode_u16(float physical, float mult, bool &range_err);
    static int16_t  encode_i16(float physical, float mult, bool &range_err);
    static uint16_t encode_angle(float deg, bool &range_err);

    // Little-endian, least significant byte first - SAR_ICD_PNU.md 2.2, and never a
    // memcpy of a struct.  PNU-ISSUE(D15).
    static void put_le16(uint8_t *p, uint16_t v);
    static void put_le32(uint8_t *p, uint32_t v);
    static void put_le64(uint8_t *p, uint64_t v);
    static void put_le_i16(uint8_t *p, int16_t v) { put_le16(p, uint16_t(v)); }
    static void put_le_i32(uint8_t *p, int32_t v) { put_le32(p, uint32_t(v)); }

    // Called from the vehicle thread through PNU_SAR_nav_snapshot(), with the driver
    // semaphore held.  Evaluates ICD 14.7's automatic-termination conditions and, if
    // none fire and navigation is enabled, packs one bundle.
    void nav_snapshot(bool nav_failsafe);

private:
    void send_bundle(void);             // CAN thread: nine frames, in order
    void auto_terminate(const char *reason);
    void report_bundle_drop(const char *why);

    bool handle_heartbeat(const AP_HAL::CANFrame &frame);
    bool handle_response(const AP_HAL::CANFrame &frame, PNUSAR_Pending expect,
                         PNUSAR_State confirmed);

    // PNU-ISSUE(D1) : every member carries an initialiser.  Backends are allocated with
    // NEW_NOTHROW, which resolves to calloc, so this is hygiene rather than a bug fix.
    PNUSAR_State _state = PNUSAR_State::DISCONNECTED;        // what TM 51050 is showing

    // LAST_CONFIRMED_STATE, ICD 14.1.  Held separately from _state precisely so that a
    // state 4 can be published without destroying what was last actually confirmed, and
    // so that a retry can publish it again while the new window is open (ICD 14.5).
    PNUSAR_State _last_confirmed = PNUSAR_State::DISCONNECTED;

    uint32_t _last_hb_ms = 0;           // arrival of the last VALID Heart_Beat
    bool     _ever_connected = false;   // a Heart_Beat has been seen since boot
    bool     _link_up = false;          // a Heart_Beat has been seen within T_HB

    // ICD 14.8.  These two were ONE flag in step 3a and had to be split here: the
    // notification fires once, but the inhibition is latched until the next power cycle,
    // and 3d added a publish_state() that would otherwise have promoted the state back
    // out of Disconnected the moment the notification was consumed.
    bool     _recovery_pending = false; // link was lost; no release procedure exists yet
    bool     _announce_reconnect = false;

    PNUSAR_Pending _pending = PNUSAR_Pending::NONE;
    uint32_t _pending_sent_ms = 0;      // when the pending command actually went out
    bool     _not_responding = false;   // the state-4 latch for the CURRENT attempt
    bool     _nav_tx_enabled = false;   // NAV_TX_ENABLED, ICD 14.1.  Gates step 3e

    bool     _schk_reported = false;    // the once-per-boot CAN_Dn_PNU_SCHK notice

    uint8_t  _ctrl_seq = 0;             // mirrors gcs().SAR_Ctrl_Seq

    // ICD 14.7 : an FC-initiated Stop is NOT a GCS transaction.  Its response and its
    // timeout are internal results and must not produce SAR_STATUS 2 or 4 - the
    // recovery-pending indication is 0, and this flag is what keeps the two apart.
    bool     _pending_is_auto = false;
    uint32_t _auto_stop_cnt = 0;        // terminations since boot, reported with each

    // Set by the vehicle thread, acted on by the CAN thread.  The 0x502 is NOT sent from
    // nav_snapshot() directly, because AP_HAL::CANIface::send() is only ever called from
    // the driver thread in this tree and nothing documents it as thread-safe.  Navigation
    // is already off by then, so the ICD's "immediately" is satisfied; only the courtesy
    // Stop waits, by at most one 100 Hz tick.
    bool     _auto_stop_request = false;

    PNUSAR_Bundle _bundle = {};         // the vehicle thread fills it, the CAN thread drains it
    uint8_t  _att_seq = 0;              // 0x404 B6, wraps 0-255
    uint32_t _bundle_tx_cnt = 0;        // bundles actually written to the bus
    uint32_t _bundle_drop_cnt = 0;      // packed but too old, or a frame refused
    uint32_t _last_bundle_warn_ms = 0;
    bool     _session_first_bundle = false;     // ICD 15.1, the Status 6 session rule

    uint32_t _reserved_rx_cnt = 0;      // 0x523 seen while its payload is still TBD
    uint32_t _last_reserved_warn_ms = 0;
    uint32_t _stray_resp_cnt = 0;       // 0x521/0x522 that matched no open window
    uint32_t _last_stray_resp_warn_ms = 0;
};

#endif  // AP_PNUSAR_ENABLED
