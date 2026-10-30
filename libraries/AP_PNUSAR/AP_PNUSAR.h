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
// STEP 3a IMPLEMENTS THE CONNECTION LAYER AND NOTHING ELSE: Heart_Beat 0x510, the
// Live_Counter, the 3 s timeout and SAR_STATUS 0 and 1.  Commands, responses and the
// navigation bundle arrive in 3d and 3e.  Ids are named here so that an early frame from
// a real device is claimed and counted rather than reported as unknown traffic.
//
// THINGS THE ICD LEAVES TBD, and which this backend therefore does not invent:
//   - 0x503 Observation_Mode_Command and 0x523 Observation_Param_Response payloads.
//     Lumir expects to define them; they are named, never transmitted, never parsed.
//   - Observation_Mode's CAN encoding, which depends on 0x503.
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

// T_HB, SAR_ICD_PNU.md section 3.3.  At least this long without a valid Heart_Beat means
// Disconnected and navigation off.  The ICD fixes it; it is not tuning.
#define AP_PNUSAR_HB_TIMEOUT_MS     (3000U)

// Health is evaluated at this rate, matching the PMU's leaky bucket so both devices on
// the shared bus are judged on the same clock.
#define AP_PNUSAR_HEALTH_CHECK_TICKS (2U)       // 50 Hz at AP_PNU_CAN_LOOP_HZ

// ICD SAR_STATUS values, section 14.1.  START_PENDING and STOP_PENDING are internal and
// are deliberately NOT in this enum - the ICD says they are not MAVLink states.
enum class PNUSAR_State : uint8_t {
    DISCONNECTED   = 0,
    READY          = 1,
    STOPPED        = 2,
    STARTED        = 3,
    NOT_RESPONDING = 4,
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
    void set_state(PNUSAR_State state);
    bool handle_heartbeat(const AP_HAL::CANFrame &frame);

    // PNU-ISSUE(D1) : every member carries an initialiser.  Backends are allocated with
    // NEW_NOTHROW, which resolves to calloc, so this is hygiene rather than a bug fix.
    PNUSAR_State _state = PNUSAR_State::DISCONNECTED;

    uint32_t _last_hb_ms = 0;           // arrival of the last VALID Heart_Beat
    bool     _ever_connected = false;   // a Heart_Beat has been seen since boot
    bool     _recovery_pending = false; // ICD 14.8 : connection was lost and not released

    uint32_t _reserved_rx_cnt = 0;      // 0x523 seen while its payload is still TBD
    uint32_t _last_reserved_warn_ms = 0;
};

#endif  // AP_PNUSAR_ENABLED
