#pragma once

#include "AP_PNUSAR_config.h"

#if AP_PNUSAR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL/Semaphores.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

// PNU : CAN driver for the SAR radar payload.
//
// Modelled on AP_PMUCAN: a private CAN protocol, NOT DroneCAN.  Like AP_PMUCAN it owns its
// own thread, created in init(), and does all CAN work there - nothing here runs on the
// vehicle scheduler.
//
// INTENT, recorded 2026-10-15 before the implementation session:
//
//   - This is the SAR radar payload itself.  It is **stabilised by AP_PNUGimbal**, which
//     drives three CubeMars GL60 II KV28 motors and carries no camera.
//   - Unlike AP_PNUGimbal, this one probably IS a device driver in the AP_PMUCAN sense:
//     a complete device with its own CAN protocol, reporting status and taking commands.
//
// Open design question for the next session: where does SAR data go?  Candidates are
// AP_Proximity (if it produces range/bearing usable for avoidance), a PNU-KAL ICD message
// of its own in pnu_kal.xml alongside TM1/TM2/TM5, or a private path to the ground.  That
// choice decides most of the rest of this file, so settle it first.
//
// TODO(AP_PNUSAR) : the ICD is not yet available.  Everything below marked TODO needs the
// real CAN ids, DLCs and field layouts before this driver does anything useful.

#define AP_PNUSAR_LOOP_HZ          (100U)     // thread rate, 10 ms
#define AP_PNUSAR_CMD_DLC          (8U)       // TODO : confirm against the ICD
#define AP_PNUSAR_STS_DLC          (8U)       // TODO : confirm against the ICD
#define AP_PNUSAR_SHORT_FRAME_WARN_MS (10000U) // rate limit for the short-frame warning

class AP_PNUSAR : public AP_CANDriver {
public:
    AP_PNUSAR();
    ~AP_PNUSAR();

    CLASS_NO_COPY(AP_PNUSAR);

    // AP_CANDriver
    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // return the driver registered on @driver_index, or nullptr
    static AP_PNUSAR *get_pnusar(uint8_t driver_index);

    static AP_PNUSAR *get_singleton() { return _singleton; }

private:

    void loop(void);                // thread entry
    void rx_spin(void);             // drain the interface and dispatch
    void tx_spin(void);             // send whatever is pending

    // PNU-ISSUE(D6) lesson from AP_PMUCAN: validate the DLC before reading any field.
    // A short or malformed frame must be rejected and counted, never parsed from fixed
    // offsets - that produced silently stale values on the PMU for a long time.
    void handle_frame(const AP_HAL::CANFrame &frame);
    void report_short_frame(uint32_t frame_id, uint8_t dlc);

    // PNU-ISSUE(D15) lesson: the PMU is little-endian and so is the STM32, so fields are
    // memcpy'd straight into native integers with no byte swap.  Confirm the same for this
    // device before copying that pattern - and if it holds, say so at the site, because the
    // absence of a swap otherwise looks like an oversight.
    int send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

    static AP_PNUSAR *_singleton;

    // PNU-ISSUE(D1/D10) lesson: every member carries an initialiser.  Backends are
    // allocated with NEW_NOTHROW, which resolves to calloc, so this is hygiene rather than
    // a bug fix - but it removes a dependency on allocator behaviour that nothing states as
    // a contract, and it keeps working if an instance is ever placed differently.
    char _thread_name[12] = {};
    bool _initialized = false;
    uint8_t _driver_index = 0;
    AP_HAL::CANIface *_can_iface = nullptr;

    // the interface signals this when a frame arrives; handed to set_event_handle() in
    // add_interface(), same as AP_PMUCAN
    HAL_BinarySemaphore sem_handle;

    uint32_t _loop_cnt = 0;
    uint32_t _short_frame_cnt = 0;          // frames rejected for a short DLC
    uint32_t _last_short_frame_warn_ms = 0; // rate limit for the warning above

    // PNU-ISSUE(D6) note: AP_PMUCAN accumulated five counters that were incremented and
    // never read anywhere in the tree.  Do not repeat that - add a counter only when
    // something reports it, or it is RAM that tells nobody anything.
    uint32_t _tx_err_cnt = 0;               // reported by report_short_frame()'s sibling
};

namespace AP {
    AP_PNUSAR *pnusar();
};

#endif  // AP_PNUSAR_ENABLED
