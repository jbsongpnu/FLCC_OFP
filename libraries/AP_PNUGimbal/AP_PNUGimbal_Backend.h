#pragma once

#include "AP_PNUGimbal_config.h"

#if AP_PNUGIMBAL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/CANIface.h>

class AP_PNUGimbal;

// PNU : protocol backend for AP_PNUGimbal.
//
// AP_PNUGimbal owns the CAN interface, the thread and the parameters; a backend owns the
// protocol spoken on that interface.  The split exists because the mount is expected to
// accommodate more than one motor type, and between them only the protocol differs - the
// thread, the interface, the axis mapping and the way the vehicle asks for an angle are
// the same either way.  AP_PMUCAN has no such split because it only ever talks to one
// device.
//
// Everything in this interface is axis-oriented and protocol-free: the caller asks for a
// per-axis position / velocity / torque and reads back per-axis feedback.  Nothing here
// mentions CAN ids, bit packing or DLCs, and nothing above this line should have to.
//
// What a backend must NOT do:
//   - touch _can_iface.  All CAN access goes through send_frame() below, so there is one
//     place that validates an outgoing frame and one place that counts a failed send.
//   - command a motor before the caller has set a target for it.  A zero-initialised
//     target is not a safe command on any of these protocols.

#define AP_PNUGIMBAL_NUM_AXES 3U

// Axis order is fixed, and it is the order of the three CAN_Dn_PG_*ID parameters.
enum AP_PNUGimbal_Axis : uint8_t {
    AP_PNUGIMBAL_AXIS_ROLL  = 0,
    AP_PNUGIMBAL_AXIS_PITCH = 1,
    AP_PNUGIMBAL_AXIS_YAW   = 2,
};

// One axis command.  This is a superset: each protocol, and each control mode within a
// protocol, uses the subset it understands and ignores the rest.  The backend documents
// which fields it reads.
struct AP_PNUGimbal_Target {
    float position_rad = 0.0f;      // target angle
    float velocity_rads = 0.0f;     // target angular rate, or a speed limit
    float kp = 0.0f;                // stiffness, impedance-style modes only
    float kd = 0.0f;                // damping, impedance-style modes only
    float torque_nm = 0.0f;         // feed-forward torque
};

// One axis' most recent decoded feedback.
struct AP_PNUGimbal_Feedback {
    float position_rad = 0.0f;
    float velocity_rads = 0.0f;
    float torque_nm = 0.0f;
    int8_t driver_temp_c = 0;
    int8_t motor_temp_c = 0;
    uint8_t status = 0;             // raw protocol status code, NOT normalised
    uint32_t last_update_ms = 0;    // AP_HAL::millis() of the frame this came from
    uint32_t frame_cnt = 0;         // feedback frames accepted for this axis
    bool valid = false;             // false until the axis has reported at least once
};

class AP_PNUGimbal_Backend {
public:
    AP_PNUGimbal_Backend(AP_PNUGimbal &frontend) : _frontend(frontend) {}
    virtual ~AP_PNUGimbal_Backend() {}

    CLASS_NO_COPY(AP_PNUGimbal_Backend);

    // Validate the configuration and prepare to run.  Returning false means the driver
    // must not transmit; the backend has already recorded why through
    // AP_PNUGimbal::set_init_error(), which is reported once the GCS is listening.
    virtual bool init(void) = 0;

    // Called from the driver thread at AP_PNUGIMBAL_LOOP_HZ, after rx_spin().
    virtual void update(void) = 0;

    // Called for every frame received on the interface.  The frontend has already
    // discarded loopback frames and nothing else: IDE / RTR / DLC validation belongs
    // here, because only the backend knows what its protocol accepts.
    virtual void handle_frame(const AP_HAL::CANFrame &frame) = 0;

    // Per-axis command.  Implementations must reject a non-finite field rather than
    // store it, so a bad value can never be encoded or left behind as the last good
    // target.
    virtual void set_target(uint8_t axis, const AP_PNUGimbal_Target &target) = 0;

    // Most recent decoded feedback for one axis.  false if that axis has never reported,
    // or has stopped reporting for longer than the backend's timeout.
    virtual bool get_feedback(uint8_t axis, AP_PNUGimbal_Feedback &feedback) const = 0;

    // Enabling, zeroing and clearing faults are explicit actions, never automatic
    // startup steps - zero-setting in particular is persistent on some drives.
    virtual void set_enable(bool enable) = 0;
    virtual void request_zero(uint8_t axis) = 0;
    virtual void request_clear_errors(uint8_t axis) = 0;

    // True when every configured axis is reporting fresh feedback.
    virtual bool healthy(void) const = 0;

    // Short name for GCS messages, e.g. "CubeMars".
    virtual const char *name(void) const = 0;

protected:
    // CAN access, provided by the frontend.  Returns true if the frame was accepted by
    // the interface; a false return is already counted and reported by the frontend, so
    // a backend only has to decide whether to retry.
    bool send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

    // Report a frame rejected for a short DLC, rate limited in the frontend.
    void report_short_frame(uint32_t frame_id, uint8_t dlc);

    AP_PNUGimbal &_frontend;
};

#endif  // AP_PNUGIMBAL_ENABLED
