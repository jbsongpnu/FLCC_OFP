#pragma once

#include "AP_PNU_CAN_config.h"

#if AP_PNU_CAN_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "AP_PNU_CAN_Backend.h"
#include <AP_PNUGimbal/AP_PNUGimbal_config.h>
#if AP_PNUGIMBAL_ENABLED
#include <AP_PNUGimbal/AP_PNUGimbal_Backend.h>
#endif

// PNU : CAN driver for PNU's own devices.
//
// A private CAN protocol, NOT DroneCAN.  Like AP_PMUCAN it owns its own thread, created
// in init(), and does all CAN work there - nothing here runs on the vehicle scheduler.
//
// WHY ONE DRIVER FOR THREE DEVICES.  The PMU, the gimbal (three CubeMars GL60 II KV28
// motors, one per axis) and the SAR payload share one 1 Mbit/s bus.  AP_CANManager binds
// each interface to exactly ONE driver - many interfaces to one driver is supported, one
// interface to many drivers is not, and a driver with no interface is never allocated at
// all, so it fails silently.  CAN_Dn_PROTOCOL2 does not help either: it is a
// DroneCAN-hosted auxiliary slot, it accepts one aux driver, and that driver must be a
// CANSensor with no thread.  So three protocols on one wire means one driver with three
// backends.
//
// The demux is nearly free.  The PMU is 29-bit extended throughout; the gimbal and the
// SAR are 11-bit standard and are told apart by id - see the CAN id allocation in
// README.md, which reserves 0x000-0x2FF for the gimbal and 0x400-0x7FF for the SAR.
//
// WHICH DEVICES ARE PRESENT is CAN_Dn_PNU_OPTS, a bitmask.  It exists because "no PMU"
// used to mean not setting CAN_Dn_PROTOCOL, and one driver carrying all three protocols
// takes that away - an aircraft with a gimbal and a SAR but no PMU is a configuration
// that will fly.
//
// WHAT THIS LAYER DOES NOT DO:
//   - it closes no stabilisation loop.  The gimbal backend's set_target() is the entry
//     point a control layer will call; nothing in the tree calls it yet, so the gimbal
//     transmits nothing until something does.
//   - it is not an AP_Mount backend.  Whether the gimbal becomes one - which would give
//     targeting, mount modes, RC and MAVLink control for free, as AP_Mount_Servo shows a
//     camera-less mount can - is still open, and is the decision that should come before
//     the loop rather than after it.
//   - AP_PNU_CAN_LOOP_HZ is still 100.  That is the skeleton's figure and it has not been
//     checked against what the chosen CubeMars mode wants; the manual specifies no
//     command period at all, so it needs measuring.

#define AP_PNU_CAN_SELECT_TIMEOUT_US   (2000U)    // RX select(), well inside one tick
#define AP_PNU_CAN_SEND_TIMEOUT_US     (2000U)    // TX deadline handed to the interface
#define AP_PNU_CAN_RX_MAX_FRAMES       (24U)      // drain budget, frames per tick
#define AP_PNU_CAN_RX_MAX_TIME_US      (1500U)    // drain budget, microseconds per tick
#define AP_PNU_CAN_SHORT_FRAME_WARN_MS (10000U)   // rate limit, short-frame warning
#define AP_PNU_CAN_TX_ERR_WARN_MS      (10000U)   // rate limit, send-failure warning
#define AP_PNU_CAN_UNCLAIMED_WARN_MS   (10000U)   // rate limit, unclaimed-frame warning
#define AP_PNU_CAN_STARTUP_REPORT_MS   (5000U)    // when the one-shot startup line goes out
#define AP_PNU_CAN_MAX_BACKENDS        (3U)

class AP_PNU_CAN : public AP_CANDriver {
public:
    AP_PNU_CAN();
    ~AP_PNU_CAN();

    CLASS_NO_COPY(AP_PNU_CAN);

    static const struct AP_Param::GroupInfo var_info[];

    // CAN_Dn_PNU_OPTS bits.  A device that is not selected is not constructed at all,
    // so it costs no RAM and reports nothing.
    enum Device : uint8_t {
        DEVICE_PMU    = (1U << 0),
        DEVICE_GIMBAL = (1U << 1),
        DEVICE_SAR    = (1U << 2),
    };

    // AP_CANDriver
    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // return the driver registered on @driver_index, or nullptr
    static AP_PNU_CAN *get_pnu_can(uint8_t driver_index);

    static AP_PNU_CAN *get_singleton() { return _singleton; }

    // The gimbal backend, or nullptr when CAN_Dn_PNU_OPTS bit 1 is clear.  A control
    // layer takes this once and drives it through the AP_PNUGimbal_Backend interface,
    // which names no motor protocol.  Callers must hold get_semaphore() across any call,
    // because the driver thread writes the same state.
#if AP_PNUGIMBAL_ENABLED
    AP_PNUGimbal_Backend *get_gimbal(void) const { return _gimbal; }
#endif

    // guards every backend's state.  The driver thread writes it from its own loop; a
    // control layer reads and writes it from the vehicle thread.
    HAL_Semaphore &get_semaphore(void) { return _sem; }

    // ---- used by backends ---------------------------------------------------------
    // Write one frame.  @can_id is the full 32-bit id including FlagEFF / FlagRTR where
    // the protocol needs them.  True if the interface accepted it; a false return is
    // already counted and reported here, so a backend only decides whether to retry.
    bool send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

    // Report a frame rejected for a short DLC, rate limited, first always reported.
    void report_short_frame(uint32_t frame_id, uint8_t dlc);

    // Record why a backend could not start.  Reported once from the thread rather than
    // immediately, because init() runs during boot and a statustext sent then is lost
    // before any GCS is listening.
    void set_init_error(const char *fmt, ...) FMT_PRINTF(2, 3);

    // parameter accessors for backends
    uint8_t  get_opts(void) const       { return uint8_t(_opts.get()); }
    uint8_t  get_gimbal_type(void) const{ return uint8_t(_gimbal_type.get()); }
    uint8_t  get_gimbal_mode(void) const{ return uint8_t(_gimbal_mode.get()); }
    uint16_t get_master_id(void) const  { return uint16_t(_master_id.get()); }
    uint8_t  get_motor_id(uint8_t axis) const;
    float    get_mech_limit_rad(uint8_t axis) const;
    float    get_slew_rate_degs(void) const { return _slew_rate_degs.get(); }
    bool     get_maintenance(void) const;
    float    get_axis_direction(uint8_t axis) const;
    float    get_operating_margin_degs(void) const { return _operating_margin_degs.get(); }
    bool     get_homing_enabled(void) const    { return _homing_enabled.get() != 0; }
    float    get_homing_vel_degs(void) const   { return _homing_vel_degs.get(); }
    float    get_homing_kd(void) const         { return _homing_kd.get(); }
    float    get_homing_kp(void) const         { return _homing_kp.get(); }
    uint32_t get_homing_stall_ms(void) const   { const int32_t v = _homing_stall_ms.get(); return (v > 0) ? uint32_t(v) : 0U; }
    float    get_homing_stall_delta_degs(void) const    { return _homing_stall_delta_degs.get(); }
    float    get_homing_range_tol_degs(void) const      { return _homing_range_tol_degs.get(); }
    float    get_homing_reconfirm_tol_degs(void) const  { return _homing_reconfirm_tol_degs.get(); }
    float    get_p_max(void) const      { return _p_max.get(); }
    float    get_v_max(void) const      { return _v_max.get(); }
    float    get_kp_max(void) const     { return _kp_max.get(); }
    float    get_kd_max(void) const     { return _kd_max.get(); }
    float    get_t_max(void) const      { return _t_max.get(); }

private:

    void loop(void);                // thread entry
    void rx_spin(void);             // drain the interface and dispatch to the backends
    void tx_spin(void);             // let each backend send whatever it has pending

    void create_backends(void);     // allocate and validate the selected backends
    bool add_backend(AP_PNU_CAN_Backend *backend);
    void report_startup(void);      // the one-shot line described at set_init_error()
    void report_tx_error(uint32_t can_id, int result);
    void report_unclaimed(uint32_t can_id);

    static AP_PNU_CAN *_singleton;

    // PNU-ISSUE(D1/D10) lesson: every member carries an initialiser.  Drivers are
    // allocated with NEW_NOTHROW, which resolves to calloc, so this is hygiene rather
    // than a bug fix - but it removes a dependency on allocator behaviour that nothing
    // states as a contract, and it keeps working if an instance is ever placed
    // differently.
    char _thread_name[10] = {};
    bool _initialized = false;
    uint8_t _driver_index = 0;
    AP_HAL::CANIface *_can_iface = nullptr;

    AP_PNU_CAN_Backend *_backends[AP_PNU_CAN_MAX_BACKENDS] = {};
    uint8_t _num_backends = 0;

#if AP_PNUGIMBAL_ENABLED
    // typed alias into _backends, so a control layer does not have to search
    AP_PNUGimbal_Backend *_gimbal = nullptr;
#endif

    HAL_Semaphore _sem;

    // the interface signals this when a frame arrives; handed to set_event_handle() in
    // add_interface(), same as AP_PMUCAN
    HAL_BinarySemaphore sem_handle;

    char _init_error[58] = {};              // empty means "no failure recorded"
    bool _startup_reported = false;

    uint32_t _loop_cnt = 0;

    // PNU-ISSUE(D6) note: AP_PMUCAN accumulated five counters that were incremented and
    // never read anywhere in the tree.  Every counter here is read by the message that
    // reports it; do not add one without a reader.  One rate limiter per cause, too -
    // sharing a limiter lets a permanent condition crowd out a genuine fault, which is
    // the 2026-10-14 lesson from the camera path.
    uint32_t _short_frame_cnt = 0;
    uint32_t _last_short_frame_warn_ms = 0;
    uint32_t _tx_err_cnt = 0;
    uint32_t _last_tx_err_warn_ms = 0;
    uint32_t _unclaimed_cnt = 0;
    uint32_t _last_unclaimed_warn_ms = 0;

    AP_Int8  _opts;
    AP_Int8  _gimbal_type;
    AP_Int8  _gimbal_mode;
    AP_Int16 _master_id;
    AP_Int8  _motor_id[AP_PNUGIMBAL_NUM_AXES];
    AP_Float _p_max;
    AP_Float _v_max;
    AP_Float _kp_max;
    AP_Float _kd_max;
    AP_Float _t_max;
    AP_Float _mech_limit_deg[AP_PNUGIMBAL_NUM_AXES];
    AP_Float _slew_rate_degs;
    AP_Int8  _maintenance;
    AP_Int8  _homing_enabled;
    AP_Float _homing_vel_degs;
    AP_Float _homing_kd;
    AP_Float _homing_kp;
    AP_Int32 _homing_stall_ms;
    AP_Float _homing_stall_delta_degs;
    AP_Float _homing_range_tol_degs;
    AP_Float _homing_reconfirm_tol_degs;
    AP_Float _operating_margin_degs;
    AP_Int8  _axis_reversed[AP_PNUGIMBAL_NUM_AXES];
};

namespace AP {
    AP_PNU_CAN *pnu_can();
};

#endif  // AP_PNU_CAN_ENABLED
