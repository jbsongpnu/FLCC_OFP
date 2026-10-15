#pragma once

#include "AP_PNUGimbal_config.h"

#if AP_PNUGIMBAL_ENABLED

#include <AP_PNU_CAN/AP_PNU_CAN_Backend.h>

// PNU : the gimbal-shaped view of an AP_PNU_CAN backend.
//
// AP_PNU_CAN_Backend says what every protocol on the bus must do - init, update, claim a
// frame.  This adds what a *gimbal* must do, and it is deliberately a separate layer:
// the mount is expected to accommodate more than one motor type, and a control layer
// above should be able to drive any of them without naming CubeMars.
//
// Everything here is axis-oriented and protocol-free.  Nothing mentions CAN ids, bit
// packing or DLCs, and nothing above this line should have to.

#define AP_PNUGIMBAL_NUM_AXES 3U

// Axis order is fixed, and it is the order of the three CAN_Dn_PNU_*ID parameters.
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

class AP_PNUGimbal_Backend : public AP_PNU_CAN_Backend {
public:
    using AP_PNU_CAN_Backend::AP_PNU_CAN_Backend;

    CLASS_NO_COPY(AP_PNUGimbal_Backend);

    // Per-axis command.  Implementations must reject a non-finite field rather than
    // store it, so a bad value can never be encoded or left behind as the last good
    // target.  Nothing is sent to a motor until a target has been set for it.
    virtual void set_target(uint8_t axis, const AP_PNUGimbal_Target &target) = 0;

    // Most recent decoded feedback for one axis.  false if that axis has never reported,
    // or has stopped reporting for longer than the backend's timeout.
    virtual bool get_feedback(uint8_t axis, AP_PNUGimbal_Feedback &feedback) const = 0;

    // Enabling, zeroing and clearing faults are explicit actions, never automatic
    // startup steps - zero-setting in particular is persistent on some drives.
    virtual void set_enable(bool enable) = 0;
    virtual void request_zero(uint8_t axis) = 0;
    virtual void request_clear_errors(uint8_t axis) = 0;
};

#endif  // AP_PNUGIMBAL_ENABLED
