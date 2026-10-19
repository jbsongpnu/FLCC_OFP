#include "AP_PNUGimbal_Backend.h"

#if AP_PNUGIMBAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_PNU_CAN/AP_PNU_CAN.h>

extern const AP_HAL::HAL &hal;

const char *AP_PNUGimbal_Backend::axis_name(uint8_t axis)
{
    switch (axis) {
    case AP_PNUGIMBAL_AXIS_ROLL:  return "roll";
    case AP_PNUGIMBAL_AXIS_PITCH: return "pitch";
    case AP_PNUGIMBAL_AXIS_YAW:   return "yaw";
    }
    return "?";
}

const char *AP_PNUGimbal_Backend::state_str(State state)
{
    switch (state) {
    case State::INIT:        return "INIT";
    case State::HOMING:      return "HOMING";
    case State::ALIGN:       return "ALIGN";
    case State::RUN:         return "RUN";
    case State::MAINTENANCE: return "MAINTENANCE";
    case State::FAULT:       return "FAULT";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

bool AP_PNUGimbal_Backend::init(void)
{
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        _mech_limit_rad[axis] = _frontend.get_mech_limit_rad(axis);
        if (!isfinite(_mech_limit_rad[axis]) || !is_positive(_mech_limit_rad[axis])) {
            _frontend.set_init_error("%s mechanical limit must be > 0", axis_name(axis));
            return false;
        }
    }

    _operating_margin_rad = radians(_frontend.get_operating_margin_degs());
    if (!isfinite(_operating_margin_rad) || !is_positive(_operating_margin_rad)) {
        _frontend.set_init_error("PNU_MARG must be > 0");
        return false;
    }
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (_operating_margin_rad >= _mech_limit_rad[axis]) {
            _frontend.set_init_error("PNU_MARG exceeds %s mech limit", axis_name(axis));
            return false;
        }
        _axis_dir[axis] = _frontend.get_axis_direction(axis);
    }

    const float slew_degs = _frontend.get_slew_rate_degs();
    if (!isfinite(slew_degs) || is_negative(slew_degs)) {
        _frontend.set_init_error("PNU_SLEW must be >= 0");
        return false;
    }
    _slew_rate_rads = radians(slew_degs);

    _homing_enabled = _frontend.get_homing_enabled();
    if (_homing_enabled) {
        _homing_vel_rads         = radians(_frontend.get_homing_vel_degs());
        _homing_kd               = _frontend.get_homing_kd();
        _homing_kp               = _frontend.get_homing_kp();
        _homing_stall_ms         = _frontend.get_homing_stall_ms();
        _homing_stall_delta_rad  = radians(_frontend.get_homing_stall_delta_degs());
        _homing_range_tol_rad    = radians(_frontend.get_homing_range_tol_degs());
        _homing_reconfirm_tol_rad = radians(_frontend.get_homing_reconfirm_tol_degs());

        if (!is_positive(_homing_vel_rads) || !is_positive(_homing_kd) ||
            !is_positive(_homing_kp) || (_homing_stall_ms == 0) ||
            !is_positive(_homing_stall_delta_rad) ||
            !is_positive(_homing_range_tol_rad) ||
            !is_positive(_homing_reconfirm_tol_rad)) {
            _frontend.set_init_error("PNU_H* homing values must all be > 0");
            return false;
        }

        // the reconfirm tolerance has to be smaller than the backoff, or the second
        // approach starts inside the tolerance band and proves nothing
        for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
            const float backoff_rad = homing_nominal_travel_rad(axis) *
                                      AP_PNUGIMBAL_HOMING_BACKOFF_FRAC;
            if (_homing_reconfirm_tol_rad >= backoff_rad) {
                _frontend.set_init_error("PNU_HRTOL too large for %s backoff",
                                         axis_name(axis));
                return false;
            }
        }
    }

    if (!protocol_init()) {
        // protocol_init() has recorded why
        return false;
    }

    // asked after protocol_init(), because only the protocol knows whether its current
    // configuration can run the sequence safely - for CubeMars that means MIT mode
    if (_homing_enabled) {
        // Refusing the whole gimbal backend rather than just skipping homing is
        // deliberate.  Skipping it would let the mount reach RUN, which makes healthy()
        // true and lets the vehicle arm on a mount whose limits are relative to wherever
        // it powered up - while the operator believes homing happened.  Refusing keeps
        // healthy() false, so the arming interlock still bites.
        //
        // The parameter that has to change is @RebootRequired, hence the instruction.
        const char *reason = nullptr;
        if (!homing_supported(reason)) {
            _frontend.set_init_error("homing needs %s, reboot",
                                     reason != nullptr ? reason : "?");
            return false;
        }
    }

    // MAINTENANCE is latched at init and held until reboot.  Entering is live, leaving is
    // not: the vendor utility can change the CAN id, the control mode and the P/V/T
    // ranges, and the central warning of the protocol note is that a host/device mismatch
    // does not fail, it silently rescales every command and every reading.  A reboot
    // forces a fresh init and - once step 1d lands - a fresh homing run against whatever
    // the motor now believes.
    if (_frontend.get_maintenance()) {
        _maintenance_latched = true;
    }

    _state_entered_ms = AP_HAL::millis();
    return true;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void AP_PNUGimbal_Backend::update(uint32_t tick)
{
    check_feedback_timeout();
    run_state_machine();

    if (_state == State::RUN) {
        advance_commanded();
    }

    if (allow_transmit()) {
        transmit(tick);
    }
}

void AP_PNUGimbal_Backend::set_state(State state, const char *reason)
{
    if (state == _state) {
        return;
    }

    const State previous = _state;
    _state = state;
    _state_entered_ms = AP_HAL::millis();

    // Edge triggered, so no rate limit is needed and none is wanted: a mount flapping
    // between two states is itself the thing worth seeing.
    const MAV_SEVERITY severity = (state == State::FAULT) ? MAV_SEVERITY_CRITICAL
                                                          : MAV_SEVERITY_INFO;
    GCS_SEND_TEXT(severity, "PNUGimbal: %s -> %s (%s)",
                  state_str(previous), state_str(state), reason);

    switch (state) {
    case State::INIT:
        // clear any latched fault first, then ask for motor control
        for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
            queue_clear_errors(axis);
            _axis[axis].target_valid = false;
        }
        queue_enable(true);
        break;

    case State::HOMING:
        start_homing();
        break;

    case State::ALIGN:
        seed_commanded_from_feedback();
        break;

    case State::MAINTENANCE:
    case State::FAULT:
        // the disable is queued here and flushed by allow_transmit(); after that the
        // driver sends nothing at all for this backend
        queue_enable(false);
        for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
            _axis[axis].target_valid = false;
        }
        break;

    case State::RUN:
        break;
    }
}

void AP_PNUGimbal_Backend::run_state_machine(void)
{
    // MAINTENANCE wins over everything and is latched until reboot.  Its purpose is a PC
    // on a motor's UART running the vendor utility, so the FC must be completely off the
    // bus: CAN has priority over PWM on these drives and a live command stream while the
    // utility is writing configuration is asking for trouble.
    if (_maintenance_latched) {
        set_state(State::MAINTENANCE, "PNU_MAINT set");
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    switch (_state) {

    case State::INIT:
        // any axis reporting a fault during startup aborts, per Appendix 1 item 6 -
        // "abort if the feedback Status/Error value is not 1"
        for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
            if (_axis[axis].feedback.valid && status_is_fault(_axis[axis].feedback.status)) {
                report_axis_fault(axis, "fault during init");
                return;
            }
        }
        if (all_axes_enabled()) {
            // HOMING is needed once per POWER CYCLE, not once per INIT.  The encoder zero
            // lives in the drive and survives a disable/enable - only removing power loses
            // it - so a second INIT after a fault has been cleared is still working in the
            // frame the first homing established.  Re-running the endpoint search then
            // would drive the payload into its stops for no reason.
            //
            // And it must never run while armed.  Normally that cannot happen, because
            // healthy() is false outside RUN so the vehicle cannot be armed before homing
            // finishes - but ARMING_SKIPCHK bit 16 lets an operator arm anyway, and a
            // fault cleared in flight would otherwise re-enter HOMING mid-air.
            const bool armed = hal.util->get_soft_armed();
            if (_homing_enabled && !_homed_this_boot && !armed) {
                set_state(State::HOMING, "all axes enabled");
            } else if (_homing_enabled && armed) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "PNUGimbal: homing skipped, vehicle is armed");
                set_state(State::ALIGN, "armed, homing skipped");
            } else if (_homing_enabled) {
                set_state(State::ALIGN, "already homed this boot");
            } else {
                // CAN_Dn_PNU_HOME is 0, so the zero is wherever the mount powered up -
                // the encoder is relative and its zero is assumed not to persist.  The
                // mechanical clamp therefore protects nothing and the zero has to be set
                // by hand.  Said once per boot rather than left to be discovered.
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "PNUGimbal: PNU_HOME=0, zero not found - limits are relative");
                set_state(State::ALIGN, "all axes enabled, homing skipped");
            }
            return;
        }
        if ((now_ms - _state_entered_ms) > AP_PNUGIMBAL_INIT_TIMEOUT_MS) {
            set_state(State::FAULT, "init timeout, no feedback");
        }
        return;

    case State::HOMING:
        run_homing();
        return;

    case State::ALIGN:
        // ALIGN is a one-shot: it seeds the command from the measured angle so the first
        // mount target is approached from where the mount actually is, rather than
        // arriving as a step from zero.  The slew limiter does the rest, which is why
        // there is no ramp here - one mechanism, two jobs.
        set_state(State::RUN, "command seeded from measured");
        return;

    case State::RUN:
        for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
            if (!_axis[axis].feedback.valid) {
                report_axis_fault(axis, "feedback lost");
                return;
            }
            if (status_is_fault(_axis[axis].feedback.status)) {
                report_axis_fault(axis, "fault reported");
                return;
            }
        }
        return;

    case State::MAINTENANCE:
    case State::FAULT:
        // both are terminal until something explicitly clears them.  FAULT is left by
        // request_clear_errors(); MAINTENANCE only by a reboot.
        return;
    }
}

// Transmission is permitted in RUN, during startup while commands are being queued, and
// in MAINTENANCE or FAULT only for as long as it takes the queued disable to go out.
bool AP_PNUGimbal_Backend::allow_transmit(void) const
{
    switch (_state) {
    case State::INIT:
    case State::HOMING:
    case State::ALIGN:
    case State::RUN:
        return true;
    case State::MAINTENANCE:
    case State::FAULT:
        return any_pending();
    }
    return false;
}

bool AP_PNUGimbal_Backend::all_axes_enabled(void) const
{
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (!_axis[axis].feedback.valid) {
            return false;
        }
        if (!status_is_enabled(_axis[axis].feedback.status)) {
            return false;
        }
    }
    return true;
}

void AP_PNUGimbal_Backend::report_axis_fault(uint8_t axis, const char *reason)
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return;
    }
    // the axis name is wanted in the message, and set_state() only takes one reason
    // string, so build it here rather than losing which axis failed
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PNUGimbal: %s axis %s (status 0x%x)",
                  axis_name(axis), reason, (unsigned)_axis[axis].feedback.status);
    set_state(State::FAULT, reason);
}

// ---------------------------------------------------------------------------
// Targets
// ---------------------------------------------------------------------------

void AP_PNUGimbal_Backend::set_target(uint8_t axis, const AP_PNUGimbal_Target &target)
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return;
    }

    // Section 9: reject non-finite command floats.  Rejecting here rather than at encode
    // time means a bad value never becomes the last good target either.
    if (!isfinite(target.position_rad) || !isfinite(target.velocity_rads) ||
        !isfinite(target.kp) || !isfinite(target.kd) || !isfinite(target.torque_nm)) {
        report_rejected_target(axis);
        return;
    }

    _axis[axis].target = target;

    // Into the MOTOR frame.  See _axis_dir - this and get_feedback() are the only two
    // places the mount convention and the motor convention meet.
    const float dir = _axis_dir[axis];
    _axis[axis].target.position_rad = target.position_rad * dir;
    _axis[axis].target.velocity_rads = target.velocity_rads * dir;
    _axis[axis].target.torque_nm = target.torque_nm * dir;

    // MECHANICAL LIMIT - the hard stop, applied in the layer that owns the wire.
    //
    // MNT1_ROLL/PITCH/YAW_MIN/MAX carry the OPERATING limits and AP_Mount has already
    // applied them; this is 5 degrees further out and is applied again on purpose.  The
    // targeting layer and the layer that owns the wire are different layers, and a bug in
    // the first must not be able to drive a bulky payload into the landing skid struts.
    //
    // It only means anything once homing has put zero at the centre of travel: the encoder
    // is relative and its zero is assumed not to persist, so before homing these bounds
    // are relative to wherever the mount powered up.
    const float mech_rad = _mech_limit_rad[axis];
    if (is_positive(mech_rad)) {
        _axis[axis].target.position_rad = constrain_float(_axis[axis].target.position_rad,
                                                          -mech_rad, mech_rad);
    }

    _axis[axis].target_valid = true;
}

bool AP_PNUGimbal_Backend::get_feedback(uint8_t axis, AP_PNUGimbal_Feedback &feedback) const
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return false;
    }
    feedback = _axis[axis].feedback;

    // back out of the MOTOR frame into the mount's convention
    const float dir = _axis_dir[axis];
    feedback.position_rad *= dir;
    feedback.velocity_rads *= dir;
    feedback.torque_nm *= dir;

    return feedback.valid;
}

// Walk the commanded angle toward the target at no more than CAN_Dn_PNU_SLEW deg/s.
//
// This earns its place twice.  ALIGN needs it so that the first mount target - which may
// be 25 degrees away from where the mount is sitting - does not arrive as a step at full
// stiffness.  RUN needs the same limit to bound how fast anything can command a bulky
// payload to move.  One mechanism, and cheaper than ramping stiffness.
void AP_PNUGimbal_Backend::advance_commanded(void)
{
    const float max_step_rad = (_slew_rate_rads > 0.0f)
                               ? _slew_rate_rads / float(AP_PNU_CAN_LOOP_HZ)
                               : 0.0f;

    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        AxisState &state = _axis[axis];
        if (!state.target_valid) {
            continue;
        }

        // everything except the angle passes straight through - the gains and the
        // feed-forward torque are not things a rate limit means anything for
        const float held_angle = state.commanded.position_rad;
        state.commanded = state.target;

        if (max_step_rad > 0.0f) {
            const float err = state.target.position_rad - held_angle;
            state.commanded.position_rad = held_angle + constrain_float(err, -max_step_rad,
                                                                        max_step_rad);
        }
    }
}

// Start from where the mount actually is.  Section 9 of the protocol note requires
// position control to start from the measured current angle; without this the mount snaps
// to the first target the moment stiffness comes up, which with a heavy SAR payload is the
// one thing in this design that could break hardware.
void AP_PNUGimbal_Backend::seed_commanded_from_feedback(void)
{
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        AxisState &state = _axis[axis];
        state.commanded = AP_PNUGimbal_Target{};
        if (state.feedback.valid) {
            state.commanded.position_rad = state.feedback.position_rad;
        }
    }
}

void AP_PNUGimbal_Backend::check_feedback_timeout(void)
{
    const uint32_t now_ms = AP_HAL::millis();

    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        AxisState &state = _axis[axis];
        if (!state.feedback.valid) {
            continue;
        }
        if ((now_ms - state.feedback.last_update_ms) < AP_PNUGIMBAL_FB_TIMEOUT_MS) {
            continue;
        }

        state.feedback.valid = false;
        if (!state.timeout_reported) {
            state.timeout_reported = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNUGimbal: %s feedback lost after %u frames",
                          axis_name(axis), (unsigned)state.feedback.frame_cnt);
        }
    }
}

// ---------------------------------------------------------------------------
// Operator actions
// ---------------------------------------------------------------------------

void AP_PNUGimbal_Backend::request_zero(uint8_t axis)
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return;
    }
    // Zero-setting redefines the coordinate system every limit is expressed in, so it is
    // refused while the mount is being flown.  MAINTENANCE and FAULT are the states where
    // it cannot surprise anyone.
    if ((_state != State::MAINTENANCE) && (_state != State::FAULT)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNUGimbal: zero refused in %s",
                      state_str(_state));
        return;
    }
    queue_zero(axis);
}

void AP_PNUGimbal_Backend::request_clear_errors(uint8_t axis)
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return;
    }
    queue_clear_errors(axis);

    // clearing a fault is also how FAULT is left.  MAINTENANCE is deliberately NOT left
    // this way - see init().
    if (_state == State::FAULT) {
        set_state(State::INIT, "errors cleared");
    }
}

// ---------------------------------------------------------------------------
// Health
// ---------------------------------------------------------------------------

// Healthy means RUN with every axis reporting fresh feedback, and nothing less.
//
// AP_Mount::pre_arm_checks() fails on !healthy() as Check::CAMERA, so this is the arming
// interlock: an unhomed or faulted mount cannot be armed.  That matters because with a
// relative encoder whose zero is assumed not to persist, an unhomed mount's angles are
// relative to wherever it powered up and the mechanical clamp means nothing.  The
// documented operator override is ARMING_SKIPCHK bit 16 (Camera, decimal 65536), the same
// escape hatch PNU-ISSUE(D9b) established for the PMU.
bool AP_PNUGimbal_Backend::healthy(void) const
{
    if (_state != State::RUN) {
        return false;
    }
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (!_axis[axis].feedback.valid) {
            return false;
        }
    }
    return true;
}

void AP_PNUGimbal_Backend::report_rejected_target(uint8_t axis)
{
    _rejected_target_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_rejected_target_cnt > 1) &&
        ((now_ms - _last_rejected_target_warn_ms) < AP_PNUGIMBAL_WARN_MS)) {
        return;
    }
    _last_rejected_target_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNUGimbal: %s target not finite (%u dropped)",
                  axis_name(axis), (unsigned)_rejected_target_cnt);
}

#endif  // AP_PNUGIMBAL_ENABLED
