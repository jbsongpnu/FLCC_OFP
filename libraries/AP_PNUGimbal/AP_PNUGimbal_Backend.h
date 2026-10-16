#pragma once

#include "AP_PNUGimbal_config.h"

#if AP_PNUGIMBAL_ENABLED

#include <AP_PNU_CAN/AP_PNU_CAN_Backend.h>

// PNU : the gimbal layer of an AP_PNU_CAN backend.
//
// AP_PNU_CAN_Backend says what every protocol on the shared bus must do - init, update,
// claim a frame.  This layer adds what a *gimbal* must do, and it is where the mount's
// LIFECYCLE lives: whether the hardware is in a state to be pointed at all.
//
// THE LIFECYCLE IS NOT THE MOUNT MODE.  MAV_MOUNT_MODE_* says *where to point*; the State
// below says *whether the hardware can be pointed*.  RETRACT is a targeting mode - point
// at the retract angles - not "motors off".  They are orthogonal and both are needed.
//
// WHY THE STATE MACHINE IS HERE, and not in AP_Mount_PNUGimbal or AP_PNUGimbal_CubeMars:
//   - it runs in the CAN thread, where feedback actually arrives, so a fault transition is
//     immediate rather than waiting for the next 50 Hz mount tick;
//   - it survives HAL_MOUNT_ENABLED=0, which this project supports (PNU-ISSUE D13);
//   - a second motor protocol would need the same lifecycle, so it has to be protocol
//     neutral.  Everything protocol-specific is a pure virtual in the protocol-primitives
//     block below.
//
// Everything in the public interface is axis-oriented and protocol-free.  Nothing here
// mentions CAN ids, bit packing or DLCs, and nothing above this line should have to.

// How long an axis may go without feedback before it stops counting as valid.  Section 9
// of the protocol note leaves the stale-feedback policy to the application; this layer's
// policy is to invalidate, report once, and let the state machine decide what it means.
#define AP_PNUGIMBAL_FB_TIMEOUT_MS   (500U)

// How long INIT waits for every axis to come up enabled before giving up.
#define AP_PNUGIMBAL_INIT_TIMEOUT_MS (5000U)

// Rate limit for this layer's warnings.  Deliberately one limiter per cause: a 2026-10-14
// lesson from the camera path is that several permanent warnings sharing one limiter crowd
// out genuine faults.
#define AP_PNUGIMBAL_WARN_MS         (10000U)

// One axis command.  This is a superset: each protocol, and each control mode within a
// protocol, uses the subset it understands and ignores the rest.
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

    enum class State : uint8_t {
        INIT        = 0,    // clearing faults, enabling, waiting for all axes
        HOMING      = 1,    // the Appendix 1 endpoint search - STEP 1d, NOT IMPLEMENTED
        ALIGN       = 2,    // seed the command from the measured angle
        RUN         = 3,    // normal operation; mount targets accepted
        MAINTENANCE = 4,    // FC off the bus so a PC can configure a motor over UART
        FAULT       = 5,    // motors disabled; operator must clear
    };

    // AP_PNU_CAN_Backend.  update() and healthy() are final: the lifecycle is this
    // layer's job and a protocol backend must not be able to bypass it.
    void update(uint32_t tick) override final;
    bool healthy(void) const override final;
    const char *name(void) const override final { return "Gimbal"; }

    // ---- interface for the mount layer -------------------------------------------
    // Per-axis command, in the gimbal frame.  Clamped to the mechanical limit and
    // rejected if non-finite.  Setting a target moves nothing unless the state machine
    // has reached RUN.
    void set_target(uint8_t axis, const AP_PNUGimbal_Target &target);

    // Most recent decoded feedback for one axis.  false if that axis has never reported,
    // or has stopped reporting for longer than AP_PNUGIMBAL_FB_TIMEOUT_MS.
    bool get_feedback(uint8_t axis, AP_PNUGimbal_Feedback &feedback) const;

    State get_state(void) const { return _state; }
    static const char *state_str(State state);

    // Operator actions.  Zero-setting persistence is unspecified by the protocol note, so
    // it stays explicit and is refused where it could surprise anyone.
    void request_zero(uint8_t axis);
    void request_clear_errors(uint8_t axis);

protected:

    // ---- protocol primitives, provided by the derived class -----------------------
    // Validate the protocol's own configuration.  Called by init() once the shared
    // parameters have been read and checked.
    virtual bool protocol_init(void) = 0;

    // Queue an enable or disable for every axis.  Queued, not sent: the driver's single
    // send_frame() is the only thing that touches the bus.
    virtual void queue_enable(bool enable) = 0;
    virtual void queue_zero(uint8_t axis) = 0;
    virtual void queue_clear_errors(uint8_t axis) = 0;

    // True while any axis still has a queued command that has not gone out.  The state
    // machine uses it to know when a disable has actually reached the motors.
    virtual bool any_pending(void) const = 0;

    // Send whatever is outstanding.  Called only when allow_transmit() is true, and the
    // implementation must consult allow_motion() before sending a motion command.
    virtual void transmit(uint32_t tick) = 0;

    // Interpret a raw protocol status code.
    virtual bool status_is_enabled(uint8_t status) const = 0;
    virtual bool status_is_fault(uint8_t status) const = 0;

    // ---- for the derived class to consult ------------------------------------------
    // True only in RUN.  A motion command must never go out in any other state.
    bool allow_motion(void) const { return _state == State::RUN; }

    // The slew-limited command for one axis - what should actually be sent, which is not
    // the same as what the mount asked for.
    const AP_PNUGimbal_Target &commanded(uint8_t axis) const { return _axis[axis].commanded; }
    bool commanded_valid(uint8_t axis) const { return _axis[axis].target_valid; }

    // Drop to FAULT, naming the reason.  Called from a protocol backend when it decodes a
    // fault status, so the transition happens on the frame rather than a tick later.
    void report_axis_fault(uint8_t axis, const char *reason);

    bool init(void) override;

    static const char *axis_name(uint8_t axis);

    // PNU-ISSUE(D1/D10) lesson: every member carries an initialiser, inside this struct
    // as well as outside it.
    struct AxisState {
        AP_PNUGimbal_Target target;      // what the mount asked for, mechanically clamped
        AP_PNUGimbal_Target commanded;   // slew-limited; what transmit() should send
        AP_PNUGimbal_Feedback feedback;
        bool target_valid = false;       // nothing is commanded until a target arrives
        bool timeout_reported = false;
    };
    AxisState _axis[AP_PNUGIMBAL_NUM_AXES];

    // Mechanical travel bounds, radians, symmetric - the hard stops.  MNT1_*_MIN/MAX carry
    // the operating limits 5 degrees inside these.  See set_target().
    float _mech_limit_rad[AP_PNUGIMBAL_NUM_AXES] = {};

private:

    void run_state_machine(void);
    void set_state(State state, const char *reason);
    bool allow_transmit(void) const;
    void advance_commanded(void);
    void seed_commanded_from_feedback(void);
    void check_feedback_timeout(void);
    bool all_axes_enabled(void) const;
    void report_rejected_target(uint8_t axis);

    State _state = State::INIT;
    uint32_t _state_entered_ms = 0;
    bool _maintenance_latched = false;   // entered once, held until reboot

    float _slew_rate_rads = 0.0f;        // 0 means no limit

    uint32_t _rejected_target_cnt = 0;
    uint32_t _last_rejected_target_warn_ms = 0;
};

#endif  // AP_PNUGIMBAL_ENABLED
