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

// HOMING: how far an axis backs off between the first approach to an endpoint and the
// reconfirm approach, as a fraction of that axis' nominal travel.  Derived rather than a
// parameter so it scales with the axis - 2% is 1.2 degrees on a 60 degree axis and 0.8 on
// a 40 degree one - and it must exceed CAN_Dn_PNU_HRTOL or the reconfirm is not a real
// second approach.  init() checks that.
#define AP_PNUGIMBAL_HOMING_BACKOFF_FRAC (0.02f)

// HOMING: slack added to every derived phase timeout, for the time a drive takes to react.
#define AP_PNUGIMBAL_HOMING_SLACK_MS     (2000U)

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
        HOMING      = 1,    // the Appendix 1 endpoint search - step 1d, CAN_Dn_PNU_HOME
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

    // The Appendix 1 endpoint search, one axis at a time in the order roll, pitch, yaw.
    // Each axis: approach one stop, back off, approach again and compare the two readings;
    // repeat the other way; validate the travel between them; move to the midpoint; settle;
    // zero; verify.  See AP_PNUGimbal_Homing.cpp.
    enum class HomingPhase : uint8_t {
        SEEK_CW     = 0,
        BACKOFF_CW  = 1,
        RESEEK_CW   = 2,
        SEEK_CCW    = 3,
        BACKOFF_CCW = 4,
        RESEEK_CCW  = 5,
        VALIDATE    = 6,
        GOTO_MID    = 7,
        SETTLE      = 8,
        ZERO        = 9,
        VERIFY      = 10,
    };
    static const char *homing_phase_str(HomingPhase phase);

    HomingPhase get_homing_phase(void) const { return _homing_phase; }
    uint8_t get_homing_axis(void) const { return _homing_axis; }

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

    // False when the protocol cannot run the Appendix 1 search safely in its current
    // configuration.  For CubeMars that means "not in MIT mode": MIT is the only mode in
    // which the host bounds the torque pushed into a hard stop, and the appendix specifies
    // it for exactly that reason.  @reason is filled in with something an operator can act
    // on.
    virtual bool homing_supported(const char *&reason) const = 0;

    // ---- for the derived class to consult ------------------------------------------
    // True when a motion command may go out: RUN, and HOMING.
    //
    // HOMING is the exception that has to be understood.  Its commands deliberately bypass
    // BOTH the mechanical clamp and the slew limit, because before the zero is found the
    // clamp is relative to wherever the mount powered up and would prevent ever reaching an
    // endpoint.  During HOMING the ONLY thing protecting the payload is the bounded
    // stiffness and torque the homing commands carry - which is why MIT mode is mandatory.
    bool allow_motion(void) const {
        return (_state == State::RUN) || (_state == State::HOMING);
    }

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
    // the operating limits CAN_Dn_PNU_MARG inside these.  See set_target().
    float _mech_limit_rad[AP_PNUGIMBAL_NUM_AXES] = {};

    // How far inside the mechanical limit the operating limit sits, per side.
    float _operating_margin_rad = 0.0f;

    // SIGN CONVENTION.  +1 or -1 per axis, from CAN_Dn_PNU_DRLL / DPIT / DYAW.
    //
    // Everything stored in this class is in the MOTOR frame - targets, commanded values,
    // feedback, homing endpoints.  The conversion to the mount's convention happens in
    // exactly two places, set_target() on the way in and get_feedback() on the way out, so
    // there is one pair of boundaries to reason about rather than a sign sprinkled through
    // the code.  The gimbal is still undergoing mechanical design changes, which is why
    // this is configuration rather than a constant.
    float _axis_dir[AP_PNUGIMBAL_NUM_AXES] = {1.0f, 1.0f, 1.0f};

private:

    void run_state_machine(void);
    void set_state(State state, const char *reason);
    void enter_init(void);              // INIT entry actions, shared with init()

    // AP_PNUGimbal_Homing.cpp
    void start_homing(void);
    void run_homing(void);
    void homing_set_phase(HomingPhase phase);
    void homing_abort(const char *reason);
    bool homing_stalled(void);
    bool homing_phase_elapsed(uint32_t limit_ms) const;
    uint32_t homing_sweep_timeout_ms(uint8_t axis) const;
    void homing_command_velocity(uint8_t axis, float velocity_rads);
    void homing_command_position(uint8_t axis, float position_rad, float kp);
    void homing_hold_idle_axes(void);
    float homing_nominal_travel_rad(uint8_t axis) const;
    bool allow_transmit(void) const;
    void advance_commanded(void);
    void seed_commanded_from_feedback(void);
    void check_feedback_timeout(void);
    bool all_axes_enabled(void) const;
    void report_rejected_target(uint8_t axis);

    State _state = State::INIT;
    uint32_t _state_entered_ms = 0;
    bool _maintenance_latched = false;   // entered once, held until reboot

    // HOMING.  All angles are in the pre-zero encoder frame, which is relative to wherever
    // the mount powered up - that is the whole reason this sequence exists.
    bool _homing_enabled = false;        // CAN_Dn_PNU_HOME
    bool _homed_this_boot = false;       // the zero survives a disable, only a power cycle loses it
    HomingPhase _homing_phase = HomingPhase::SEEK_CW;
    uint8_t _homing_axis = 0;
    uint32_t _phase_entered_ms = 0;
    float _phase_start_angle_rad = 0.0f; // angle on entering the phase, for backoff
    float _stall_ref_angle_rad = 0.0f;   // stall detector reference
    uint32_t _stall_ref_ms = 0;
    float _theta_first_rad = 0.0f;       // first approach, awaiting its reconfirm
    float _theta_cw_rad = 0.0f;
    float _theta_ccw_rad = 0.0f;
    float _theta_mid_rad = 0.0f;
    float _hold_angle_rad[AP_PNUGIMBAL_NUM_AXES] = {};

    // homing tuning, all read once at init - see the CAN_Dn_PNU_H* parameters
    float _homing_vel_rads = 0.0f;
    float _homing_kd = 0.0f;
    float _homing_kp = 0.0f;
    uint32_t _homing_stall_ms = 0;
    float _homing_stall_delta_rad = 0.0f;
    float _homing_range_tol_rad = 0.0f;
    float _homing_reconfirm_tol_rad = 0.0f;

    float _slew_rate_rads = 0.0f;        // 0 means no limit

    uint32_t _rejected_target_cnt = 0;
    uint32_t _last_rejected_target_warn_ms = 0;
};

#endif  // AP_PNUGIMBAL_ENABLED
