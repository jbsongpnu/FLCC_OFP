#pragma once

#include "AP_PNUGimbal_config.h"

#if AP_PNUGIMBAL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL/Semaphores.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

// PNU : CAN driver for the PNU-developed gimbal mount.
//
// Modelled on AP_PMUCAN: a private CAN protocol, NOT DroneCAN.  Like AP_PMUCAN it owns its
// own thread, created in init(), and does all CAN work there - nothing here runs on the
// vehicle scheduler.
//
// INTENT, recorded 2026-10-15 before the implementation session:
//
//   - The first option drives **three CubeMars GL60 II KV28 motors** over CAN, one per axis,
//     for a 3-axis gimbal mount.
//   - **No camera is associated with this mount.**  What it stabilises is the SAR device,
//     whose driver is AP_PNUSAR.
//
// That makes this NOT the same kind of driver as AP_PMUCAN, and the skeleton below is only
// a starting shape.  Two consequences worth settling before writing code:
//
//   1. **The CAN protocol is the MOTOR's, not a PNU device protocol.**  AP_PMUCAN talks to
//      one complete device that reports status; this talks to three actuators and must
//      close a stabilisation loop around them.  The "ICD" here is CubeMars' CAN protocol
//      (MIT or servo mode) plus whatever per-axis addressing is chosen.
//   2. **It is a control loop, not a message decoder.**  It needs AHRS attitude, and the
//      100 Hz thread this skeleton creates is very likely too slow for motor control - check
//      what rate the chosen CubeMars mode wants before keeping it.
//
// Open design question for the next session: does this back an **AP_Mount backend**, which
// would give targeting, mount modes, RC and MAVLink control for free - or does it stand
// alone and expose its own interface?  AP_Mount assumes a camera in places but does not
// require one (AP_Mount_Servo has none), so backing a backend looks plausible and would
// avoid reimplementing targeting.  Decide before writing the loop, not after.

#define AP_PNUGIMBAL_LOOP_HZ          (100U)     // thread rate, 10 ms
#define AP_PNUGIMBAL_CMD_DLC          (8U)       // TODO : confirm against the ICD
#define AP_PNUGIMBAL_STS_DLC          (8U)       // TODO : confirm against the ICD
#define AP_PNUGIMBAL_SHORT_FRAME_WARN_MS (10000U) // rate limit for the short-frame warning

class AP_PNUGimbal : public AP_CANDriver {
public:
    AP_PNUGimbal();
    ~AP_PNUGimbal();

    CLASS_NO_COPY(AP_PNUGimbal);

    // AP_CANDriver
    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // return the driver registered on @driver_index, or nullptr
    static AP_PNUGimbal *get_pnugimbal(uint8_t driver_index);

    static AP_PNUGimbal *get_singleton() { return _singleton; }

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

    static AP_PNUGimbal *_singleton;

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
    AP_PNUGimbal *pnugimbal();
};

#endif  // AP_PNUGIMBAL_ENABLED
