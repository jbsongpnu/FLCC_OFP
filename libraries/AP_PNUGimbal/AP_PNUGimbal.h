#pragma once

#include "AP_PNUGimbal_config.h"

#if AP_PNUGIMBAL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "AP_PNUGimbal_Backend.h"

// PNU : CAN driver for the PNU-developed gimbal mount.
//
// A private CAN protocol, NOT DroneCAN.  Like AP_PMUCAN it owns its own thread, created
// in init(), and does all CAN work there - nothing here runs on the vehicle scheduler.
//
// INTENT, recorded 2026-10-15:
//
//   - It drives **three CubeMars GL60 II KV28 motors** over CAN, one per axis, for a
//     3-axis gimbal mount.
//   - **No camera is associated with this mount.**  What it stabilises is the SAR
//     device, whose driver is AP_PNUSAR.
//
// STRUCTURE.  Unlike AP_PMUCAN, which talks to one complete device over one fixed
// protocol, this frontend owns only the parts that do not depend on which motor is
// fitted: the interface, the thread, the parameters, the axis mapping and the one place
// a frame is written to the bus.  The protocol itself lives in an AP_PNUGimbal_Backend -
// AP_PNUGimbal_CubeMars is the first of them.  CAN_Dn_PG_TYPE selects which.
//
// WHAT THIS LAYER DOES NOT DO, as of the backend implementation session:
//   - it closes no stabilisation loop.  set_target() is the entry point a control layer
//     will call; nothing in the tree calls it yet, so the driver transmits nothing until
//     something does.
//   - it is not an AP_Mount backend.  Whether it becomes one - which would give
//     targeting, mount modes, RC and MAVLink control for free, as AP_Mount_Servo shows a
//     camera-less mount can - is still open, and is the decision that should come before
//     the loop rather than after it.
//   - AP_PNUGIMBAL_LOOP_HZ is still 100.  That is the skeleton's figure and it has not
//     been checked against what the chosen CubeMars mode wants; the manual specifies no
//     command period at all (sec 1, sec 9.6 of the protocol note), so it needs measuring.

#define AP_PNUGIMBAL_LOOP_HZ             (100U)     // thread rate, 10 ms
#define AP_PNUGIMBAL_SELECT_TIMEOUT_US   (2000U)    // RX select(), well inside one tick
#define AP_PNUGIMBAL_SEND_TIMEOUT_US     (2000U)    // TX deadline handed to the interface
#define AP_PNUGIMBAL_RX_MAX_FRAMES       (16U)      // drain budget, frames per tick
#define AP_PNUGIMBAL_RX_MAX_TIME_US      (1000U)    // drain budget, microseconds per tick
#define AP_PNUGIMBAL_SHORT_FRAME_WARN_MS (10000U)   // rate limit for the short-frame warning
#define AP_PNUGIMBAL_TX_ERR_WARN_MS      (10000U)   // rate limit for the send-failure warning
#define AP_PNUGIMBAL_STARTUP_REPORT_MS   (5000U)    // when the one-shot startup line goes out

class AP_PNUGimbal : public AP_CANDriver {
public:
    AP_PNUGimbal();
    ~AP_PNUGimbal();

    CLASS_NO_COPY(AP_PNUGimbal);

    static const struct AP_Param::GroupInfo var_info[];

    // which protocol backend to run.  Default is NONE: selecting CAN_Dn_PROTOCOL = 16
    // alone must not start commanding motors.
    enum class Type : uint8_t {
        NONE     = 0,
        CUBEMARS = 1,
    };

    // AP_CANDriver
    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // return the driver registered on @driver_index, or nullptr
    static AP_PNUGimbal *get_pnugimbal(uint8_t driver_index);

    static AP_PNUGimbal *get_singleton() { return _singleton; }

    // ---- interface for a control layer -------------------------------------------
    // All of these are safe to call from any thread and safe to call with no backend
    // configured, in which case they do nothing and report nothing.
    void set_target(uint8_t axis, const AP_PNUGimbal_Target &target);
    bool get_feedback(uint8_t axis, AP_PNUGimbal_Feedback &feedback);
    void set_enable(bool enable);
    void request_zero(uint8_t axis);
    void request_clear_errors(uint8_t axis);
    bool healthy(void);

    // ---- used by backends ---------------------------------------------------------
    // Write one standard 11-bit data frame.  True if the interface accepted it; a false
    // return is already counted and reported here, so a backend only decides whether to
    // retry.
    bool send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

    // Report a frame rejected for a short DLC, rate limited, first always reported.
    void report_short_frame(uint32_t frame_id, uint8_t dlc);

    // Record why a backend could not start.  Reported once from the thread rather than
    // immediately, because init() runs during boot and a statustext sent then is lost
    // before any GCS is listening.
    void set_init_error(const char *fmt, ...) FMT_PRINTF(2, 3);

    // parameter accessors for backends
    uint8_t  get_mode(void) const      { return uint8_t(_mode.get()); }
    uint16_t get_master_id(void) const { return uint16_t(_master_id.get()); }
    uint8_t  get_motor_id(uint8_t axis) const;
    float    get_p_max(void) const     { return _p_max.get(); }
    float    get_v_max(void) const     { return _v_max.get(); }
    float    get_kp_max(void) const    { return _kp_max.get(); }
    float    get_kd_max(void) const    { return _kd_max.get(); }
    float    get_t_max(void) const     { return _t_max.get(); }

private:

    void loop(void);                // thread entry
    void rx_spin(void);             // drain the interface and dispatch to the backend
    void tx_spin(void);             // let the backend send whatever it has pending

    bool create_backend(void);      // allocate and validate the configured backend
    void report_startup(void);      // the one-shot line described at set_init_error()
    void report_tx_error(uint32_t can_id, int result);

    static AP_PNUGimbal *_singleton;

    // PNU-ISSUE(D1/D10) lesson: every member carries an initialiser.  Backends are
    // allocated with NEW_NOTHROW, which resolves to calloc, so this is hygiene rather
    // than a bug fix - but it removes a dependency on allocator behaviour that nothing
    // states as a contract, and it keeps working if an instance is ever placed
    // differently.
    char _thread_name[12] = {};
    bool _initialized = false;
    uint8_t _driver_index = 0;
    AP_HAL::CANIface *_can_iface = nullptr;

    AP_PNUGimbal_Backend *_backend = nullptr;

    // guards _backend's state.  The driver thread writes it from rx_spin()/tx_spin();
    // a control layer reads and writes it from the vehicle thread.  Held only around
    // backend calls, never across select(), which blocks.
    HAL_Semaphore _sem;

    // the interface signals this when a frame arrives; handed to set_event_handle() in
    // add_interface(), same as AP_PMUCAN
    HAL_BinarySemaphore sem_handle;

    char _init_error[58] = {};              // empty means "no failure recorded"
    bool _startup_reported = false;

    uint32_t _loop_cnt = 0;
    uint32_t _short_frame_cnt = 0;          // frames rejected for a short DLC
    uint32_t _last_short_frame_warn_ms = 0; // rate limit for the warning above

    // PNU-ISSUE(D6) note: AP_PMUCAN accumulated five counters that were incremented and
    // never read anywhere in the tree.  Both counters here are read by the message that
    // reports them; do not add a third without one.
    uint32_t _tx_err_cnt = 0;               // sends the interface would not accept
    uint32_t _last_tx_err_warn_ms = 0;

    AP_Int8  _type;
    AP_Int8  _mode;
    AP_Int16 _master_id;
    AP_Int8  _motor_id[AP_PNUGIMBAL_NUM_AXES];
    AP_Float _p_max;
    AP_Float _v_max;
    AP_Float _kp_max;
    AP_Float _kd_max;
    AP_Float _t_max;
};

namespace AP {
    AP_PNUGimbal *pnugimbal();
};

#endif  // AP_PNUGIMBAL_ENABLED
