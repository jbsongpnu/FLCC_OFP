#include "AP_PNUGimbal_Backend.h"

#if AP_PNUGIMBAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_PNU_CAN/AP_PNU_CAN.h>

// PNU : the Appendix 1 gimbal payload initialisation procedure, from
// libraries/PNUGimbal_CAN_Protocol_Type_CubeMars.md.
//
// THIS IS THE MOST HAZARDOUS ROUTINE IN THIS DRIVER.  It deliberately drives a bulky SAR
// payload into three sets of mechanical hard stops, and it does so with the mechanical
// clamp disabled - before the zero is found that clamp is relative to wherever the mount
// powered up, so honouring it would prevent ever reaching an endpoint.  The only thing
// protecting the payload is the bounded stiffness and torque the commands below carry.
//
// WHY MIT MODE IS MANDATORY.  Appendix 1 item 2 specifies it, and the reason is that MIT is
// the only mode where the host bounds the push into a stop.  During a sweep this code sends
// kp = 0, kd = CAN_Dn_PNU_HKD and v = +-CAN_Dn_PNU_HVEL, which is velocity-style impedance
// control: at stall the measured velocity is zero, so the torque settles at
//
//     tau_stall = HKD * HVEL
//
// a number the operator chooses directly.  A position/velocity-mode command runs a cascaded
// loop whose torque the host cannot limit from the CAN frame at all.  init() refuses to arm
// this sequence in any other mode.
//
// ENDPOINT DETECTION IS BY ANGLE ONLY.  Appendix 1 item 4 is explicit: the overcurrent,
// undervoltage and overload status codes are **emergency stop conditions, not endpoint
// detection criteria**.  So an endpoint is "the angle stopped changing while motion was
// still commanded" - a stall detector - and any fault status aborts the whole sequence
// rather than being read as an endpoint.
//
// WHAT IS STILL UNMEASURED.  The stall threshold and dwell, the approach velocity and the
// two gains are all parameters with provisional defaults, and the only one PNU has settled
// is the 1 second dwell.  They need a bench.  CAN_Dn_PNU_HOME therefore DEFAULTS TO 0 - this
// sequence does not run until somebody turns it on deliberately.

const char *AP_PNUGimbal_Backend::homing_phase_str(HomingPhase phase)
{
    switch (phase) {
    case HomingPhase::SEEK_CW:     return "seek CW";
    case HomingPhase::BACKOFF_CW:  return "backoff CW";
    case HomingPhase::RESEEK_CW:   return "reseek CW";
    case HomingPhase::SEEK_CCW:    return "seek CCW";
    case HomingPhase::BACKOFF_CCW: return "backoff CCW";
    case HomingPhase::RESEEK_CCW:  return "reseek CCW";
    case HomingPhase::VALIDATE:    return "validate";
    case HomingPhase::GOTO_MID:    return "goto mid";
    case HomingPhase::SETTLE:      return "settle";
    case HomingPhase::ZERO:        return "zero";
    case HomingPhase::VERIFY:      return "verify";
    }
    return "?";
}

// Nominal total travel of one axis: the mechanical limit is a symmetric plus/minus bound,
// so the stop-to-stop range is twice it - 60 / 60 / 40 degrees for roll / pitch / yaw.
float AP_PNUGimbal_Backend::homing_nominal_travel_rad(uint8_t axis) const
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return 0.0f;
    }
    return 2.0f * _mech_limit_rad[axis];
}

// How long one sweep may take before it is called a failure: the time to cross the whole
// axis at the approach velocity, doubled because the sweep may start from the far end, plus
// slack.  Derived rather than a parameter - a timeout that can be set inconsistently with
// the velocity is a trap.
uint32_t AP_PNUGimbal_Backend::homing_sweep_timeout_ms(uint8_t axis) const
{
    if (!is_positive(_homing_vel_rads)) {
        return AP_PNUGIMBAL_HOMING_SLACK_MS;
    }
    const float sweep_s = (2.0f * homing_nominal_travel_rad(axis)) / _homing_vel_rads;
    return uint32_t(sweep_s * 1000.0f) + AP_PNUGIMBAL_HOMING_SLACK_MS;
}

void AP_PNUGimbal_Backend::start_homing(void)
{
    _homing_axis = AP_PNUGIMBAL_AXIS_ROLL;

    // Hold every axis where it powered up before anything starts moving.  The manual does
    // not define what an enabled motor does with no command outstanding, and a bulky
    // payload swinging on an uncommanded axis while another drives into a stop is exactly
    // the collision the mechanical limits exist to prevent.
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        _hold_angle_rad[axis] = _axis[axis].feedback.position_rad;
    }

    homing_set_phase(HomingPhase::SEEK_CW);
}

void AP_PNUGimbal_Backend::homing_set_phase(HomingPhase phase)
{
    _homing_phase = phase;
    _phase_entered_ms = AP_HAL::millis();
    _phase_start_angle_rad = _axis[_homing_axis].feedback.position_rad;

    // the stall detector is armed fresh on every phase, so a phase that begins already
    // stationary does not immediately read as an endpoint
    _stall_ref_angle_rad = _phase_start_angle_rad;
    _stall_ref_ms = _phase_entered_ms;
}

bool AP_PNUGimbal_Backend::homing_phase_elapsed(uint32_t limit_ms) const
{
    return (AP_HAL::millis() - _phase_entered_ms) > limit_ms;
}

// Angle-only stall detection, per Appendix 1 item 4.  The axis counts as stopped when it
// has moved less than CAN_Dn_PNU_HDLT in CAN_Dn_PNU_HSTL milliseconds while motion was
// being commanded.  Any movement beyond the threshold re-arms the window.
bool AP_PNUGimbal_Backend::homing_stalled(void)
{
    const float angle_rad = _axis[_homing_axis].feedback.position_rad;
    const uint32_t now_ms = AP_HAL::millis();

    if (fabsf(angle_rad - _stall_ref_angle_rad) > _homing_stall_delta_rad) {
        // still moving - restart the dwell from here
        _stall_ref_angle_rad = angle_rad;
        _stall_ref_ms = now_ms;
        return false;
    }

    return (now_ms - _stall_ref_ms) >= _homing_stall_ms;
}

// Velocity-style MIT: kp = 0 so position is ignored, kd carries the damping, and the stall
// torque is kd * v.  See the file header.
void AP_PNUGimbal_Backend::homing_command_velocity(uint8_t axis, float velocity_rads)
{
    AP_PNUGimbal_Target target;
    target.position_rad = 0.0f;
    target.velocity_rads = velocity_rads;
    target.kp = 0.0f;
    target.kd = _homing_kd;
    target.torque_nm = 0.0f;

    // written straight to the commanded value: homing bypasses the mechanical clamp in
    // set_target() and the slew limiter in advance_commanded(), deliberately
    _axis[axis].commanded = target;
    _axis[axis].target_valid = true;
}

// Position-style MIT.  Section 4.3 warns explicitly against kd = 0 for position control, so
// the damping term is always carried.
void AP_PNUGimbal_Backend::homing_command_position(uint8_t axis, float position_rad, float kp)
{
    AP_PNUGimbal_Target target;
    target.position_rad = position_rad;
    target.velocity_rads = 0.0f;
    target.kp = kp;
    target.kd = _homing_kd;
    target.torque_nm = 0.0f;

    _axis[axis].commanded = target;
    _axis[axis].target_valid = true;
}

void AP_PNUGimbal_Backend::homing_hold_idle_axes(void)
{
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (axis == _homing_axis) {
            continue;
        }
        homing_command_position(axis, _hold_angle_rad[axis], _homing_kp);
    }
}

void AP_PNUGimbal_Backend::homing_abort(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PNUGimbal: homing %s axis %s: %s",
                  axis_name(_homing_axis), homing_phase_str(_homing_phase), reason);
    set_state(State::FAULT, "homing failed");
}

void AP_PNUGimbal_Backend::run_homing(void)
{
    const uint8_t axis = _homing_axis;

    // Appendix 1 item 6: abort if the feedback status is anything but 1.  Checked before
    // anything else, every tick, because a fault during a sweep means the payload is
    // already pushing on something it should not be.
    for (uint8_t i = 0; i < AP_PNUGIMBAL_NUM_AXES; i++) {
        if (!_axis[i].feedback.valid) {
            homing_abort("feedback lost");
            return;
        }
        if (!status_is_enabled(_axis[i].feedback.status)) {
            homing_abort("status not enabled");
            return;
        }
    }

    homing_hold_idle_axes();

    const float angle_rad = _axis[axis].feedback.position_rad;
    const float travel_rad = homing_nominal_travel_rad(axis);
    const float backoff_rad = travel_rad * AP_PNUGIMBAL_HOMING_BACKOFF_FRAC;

    switch (_homing_phase) {

    case HomingPhase::SEEK_CW:
    case HomingPhase::RESEEK_CW:
        homing_command_velocity(axis, _homing_vel_rads);
        if (homing_stalled()) {
            if (_homing_phase == HomingPhase::SEEK_CW) {
                _theta_first_rad = angle_rad;
                homing_set_phase(HomingPhase::BACKOFF_CW);
            } else {
                // reconfirm: the two approaches to the same stop must agree, or what was
                // hit the first time was a transient obstruction rather than the endpoint
                if (fabsf(angle_rad - _theta_first_rad) > _homing_reconfirm_tol_rad) {
                    // the two approaches disagree, so the first "stop" was probably a
                    // transient obstruction rather than the endpoint.  Print both readings:
                    // without them PNU_HRTOL cannot be tuned, and telling an operator only
                    // that something failed is the silence PNU-ISSUE(D6) set out to remove.
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                  "PNUGimbal: %s CW %.2f then %.2f deg", axis_name(axis),
                                  (double)degrees(_theta_first_rad),
                                  (double)degrees(angle_rad));
                    homing_abort("CW endpoint did not reconfirm");
                    return;
                }
                _theta_cw_rad = angle_rad;
                homing_set_phase(HomingPhase::SEEK_CCW);
            }
            return;
        }
        if (homing_phase_elapsed(homing_sweep_timeout_ms(axis))) {
            homing_abort("no CW endpoint found");
        }
        return;

    case HomingPhase::SEEK_CCW:
    case HomingPhase::RESEEK_CCW:
        homing_command_velocity(axis, -_homing_vel_rads);
        if (homing_stalled()) {
            if (_homing_phase == HomingPhase::SEEK_CCW) {
                _theta_first_rad = angle_rad;
                homing_set_phase(HomingPhase::BACKOFF_CCW);
            } else {
                if (fabsf(angle_rad - _theta_first_rad) > _homing_reconfirm_tol_rad) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                  "PNUGimbal: %s CCW %.2f then %.2f deg", axis_name(axis),
                                  (double)degrees(_theta_first_rad),
                                  (double)degrees(angle_rad));
                    homing_abort("CCW endpoint did not reconfirm");
                    return;
                }
                _theta_ccw_rad = angle_rad;
                homing_set_phase(HomingPhase::VALIDATE);
            }
            return;
        }
        if (homing_phase_elapsed(homing_sweep_timeout_ms(axis))) {
            homing_abort("no CCW endpoint found");
        }
        return;

    case HomingPhase::BACKOFF_CW:
    case HomingPhase::BACKOFF_CCW: {
        const bool from_cw = (_homing_phase == HomingPhase::BACKOFF_CW);
        homing_command_velocity(axis, from_cw ? -_homing_vel_rads : _homing_vel_rads);
        if (fabsf(angle_rad - _phase_start_angle_rad) >= backoff_rad) {
            homing_set_phase(from_cw ? HomingPhase::RESEEK_CW : HomingPhase::RESEEK_CCW);
            return;
        }
        if (homing_phase_elapsed(homing_sweep_timeout_ms(axis))) {
            homing_abort("backoff did not move");
        }
        return;
    }

    case HomingPhase::VALIDATE: {
        // Appendix 1 item 5: validate the travel between the endpoints.  This is the only
        // runtime check in the whole design that can catch a CAN_Dn_PNU_PMAX mismatch -
        // the failure the protocol note warns about, which does not fail but silently
        // rescales every command and every reading.  A factor-of-two travel reading is its
        // signature, and nothing else in the system would ever notice.
        // The bounds are ASYMMETRIC, and deliberately so - decided 2026-10-19.
        //
        // LOWER: the mount must be able to reach its own operating range, which is twice
        // (mechanical limit - PNU_MARG) - 50 degrees on a 60 degree axis with the defaults.
        // Anything less and something is obstructing the payload, typically the landing
        // skid struts, so it aborts.  Note that a measurement between the operating range
        // and the mechanical range is ACCEPTED and the operating limits stay at their
        // nominal MNT1_* values: stop compliance under a heavy payload eats into the
        // measurement, and deriving tighter limits from it every boot would make the
        // mount's reach vary with how hard it happened to press on its stops.
        //
        // UPPER: nothing should read MORE than the mechanical travel.  Stop compliance only
        // ever makes the measurement smaller, so an over-reading means the endpoint was not
        // an endpoint - or, at about twice nominal, that PNU_PMAX does not match the range
        // configured in the motor.  That is the failure the protocol note warns about, the
        // one that does not fail but silently rescales every command and every reading, and
        // this is the only runtime check in the design that can catch it.
        const float measured_rad = fabsf(_theta_cw_rad - _theta_ccw_rad);
        const float min_rad = 2.0f * (_mech_limit_rad[axis] - _operating_margin_rad);
        const float max_rad = travel_rad + _homing_range_tol_rad;

        if ((measured_rad < min_rad) || (measured_rad > max_rad)) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "PNUGimbal: %s travel %.1f deg, need %.1f-%.1f",
                          axis_name(axis), (double)degrees(measured_rad),
                          (double)degrees(min_rad), (double)degrees(max_rad));
            homing_abort((measured_rad < min_rad) ? "travel too small, obstructed?"
                                                  : "travel too large, check PNU_PMAX");
            return;
        }
        _theta_mid_rad = (_theta_cw_rad + _theta_ccw_rad) * 0.5f;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PNUGimbal: %s travel %.1f deg, mid %.1f",
                      axis_name(axis), (double)degrees(measured_rad),
                      (double)degrees(_theta_mid_rad));
        homing_set_phase(HomingPhase::GOTO_MID);
        return;
    }

    case HomingPhase::GOTO_MID:
        homing_command_position(axis, _theta_mid_rad, _homing_kp);
        if (fabsf(angle_rad - _theta_mid_rad) <= _homing_reconfirm_tol_rad) {
            homing_set_phase(HomingPhase::SETTLE);
            return;
        }
        if (homing_phase_elapsed(homing_sweep_timeout_ms(axis))) {
            homing_abort("could not reach midpoint");
        }
        return;

    case HomingPhase::SETTLE:
        // Appendix 1 item 5: confirm the midpoint has been reached AND motion has stopped,
        // before zeroing.  Zeroing while still moving would put the new origin wherever the
        // axis happened to be at that instant.
        homing_command_position(axis, _theta_mid_rad, _homing_kp);
        if (homing_stalled()) {
            homing_set_phase(HomingPhase::ZERO);
            return;
        }
        if (homing_phase_elapsed(homing_sweep_timeout_ms(axis))) {
            homing_abort("did not settle at midpoint");
        }
        return;

    case HomingPhase::ZERO:
        // hold position while the zero-set goes out, so the origin is taken where we
        // settled rather than wherever the axis drifts to
        homing_command_position(axis, _theta_mid_rad, _homing_kp);
        queue_zero(axis);
        homing_set_phase(HomingPhase::VERIFY);
        return;

    case HomingPhase::VERIFY:
        // Appendix 1 item 5: verify the zero position has been updated.  Feedback should
        // now read about zero for this axis.  Zero-setting persistence and acknowledgement
        // are both unspecified by the manual, so this read-back is the only confirmation
        // available that the drive accepted the command at all.
        if (any_pending()) {
            // The zero has NOT gone out yet, so keep holding the midpoint in the OLD
            // frame.  Commanding 0 here would mean the old origin, which can be tens of
            // degrees away - the payload would lurch across its travel on the way to a
            // position that is about to stop existing.
            homing_command_position(axis, _theta_mid_rad, _homing_kp);
            return;
        }
        // the zero has been sent, so 0 is now where the axis is standing and this is a
        // hold rather than a move
        homing_command_position(axis, 0.0f, _homing_kp);
        if (fabsf(angle_rad) <= _homing_reconfirm_tol_rad) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PNUGimbal: %s homed", axis_name(axis));
            _hold_angle_rad[axis] = 0.0f;

            if ((uint32_t(axis) + 1U) < AP_PNUGIMBAL_NUM_AXES) {
                // next axis, in the order roll, pitch, yaw
                _homing_axis = axis + 1;
                homing_set_phase(HomingPhase::SEEK_CW);
                return;
            }

            _homed_this_boot = true;
            set_state(State::ALIGN, "homing complete");
            return;
        }
        if (homing_phase_elapsed(AP_PNUGIMBAL_HOMING_SLACK_MS)) {
            homing_abort("zero not accepted");
        }
        return;
    }
}

#endif  // AP_PNUGIMBAL_ENABLED
