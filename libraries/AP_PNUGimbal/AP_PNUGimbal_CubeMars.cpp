#include "AP_PNUGimbal_CubeMars.h"

#if AP_PNUGIMBAL_CUBEMARS_ENABLED

#include <string.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_PNU_CAN/AP_PNU_CAN.h>

// ---------------------------------------------------------------------------
// Pure codec
// ---------------------------------------------------------------------------

// sec 4.1 : u = floor((clamp(x, xmin, xmax) - xmin) * Q / (xmax - xmin)), Q = (1<<bits)-1
uint16_t AP_PNUGimbal_CubeMars::encode_unsigned(float x, float x_min, float x_max, uint8_t bits)
{
    const uint32_t q = (1UL << bits) - 1UL;
    const float span = x_max - x_min;

    // a non-finite value or an invalid range cannot be mapped; the configured ranges are
    // validated in init(), so reaching this is a programming error rather than a user one
    if (!isfinite(x) || !isfinite(span) || !is_positive(span)) {
        return 0;
    }

    const float scaled = (constrain_float(x, x_min, x_max) - x_min) * float(q) / span;
    if (scaled <= 0.0f) {
        return 0;
    }
    // truncation is the floor here because scaled is non-negative.  The clamp above makes
    // scaled <= q in exact arithmetic; MIN() covers the float rounding that can put it a
    // fraction over at x == x_max, which would otherwise overflow the packed field.
    return uint16_t(MIN(uint32_t(scaled), q));
}

float AP_PNUGimbal_CubeMars::decode_unsigned(uint16_t u, float x_min, float x_max, uint8_t bits)
{
    const uint32_t q = (1UL << bits) - 1UL;
    return x_min + (float(u) * (x_max - x_min)) / float(q);
}

// sec 4.2
void AP_PNUGimbal_CubeMars::encode_mit(const AP_PNUGimbal_Target &target, const Limits &limits,
                                       uint8_t data[8])
{
    const uint16_t p  = encode_unsigned(target.position_rad,  limits.p_min,  limits.p_max,  16);
    const uint16_t v  = encode_unsigned(target.velocity_rads, limits.v_min,  limits.v_max,  12);
    const uint16_t kp = encode_unsigned(target.kp,            limits.kp_min, limits.kp_max, 12);
    const uint16_t kd = encode_unsigned(target.kd,            limits.kd_min, limits.kd_max, 12);
    const uint16_t t  = encode_unsigned(target.torque_nm,     limits.t_min,  limits.t_max,  12);

    data[0] = uint8_t(p >> 8);
    data[1] = uint8_t(p & 0xFF);
    data[2] = uint8_t((v >> 4) & 0xFF);
    data[3] = uint8_t(((v & 0x0F) << 4) | ((kp >> 8) & 0x0F));
    data[4] = uint8_t(kp & 0xFF);
    data[5] = uint8_t((kd >> 4) & 0xFF);
    data[6] = uint8_t(((kd & 0x0F) << 4) | ((t >> 8) & 0x0F));
    data[7] = uint8_t(t & 0xFF);
}

// the wire format is IEEE-754 binary32 least significant byte first (sec 5, sec 6).  That is
// also the STM32's own layout, so a memcpy would work - put_le32_ptr is used instead
// because it states the byte order at the site rather than relying on the host's, which
// is the PNU-ISSUE(D15) lesson: an absent byte swap must be visibly deliberate.
static void put_float_le(uint8_t *p, float f)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    put_le32_ptr(p, bits);
}

// sec 5 : target angle, then maximum absolute travel speed.  The speed field is a limit and
// not a signed tracking target, so the caller's sign is dropped by send_motion().
void AP_PNUGimbal_CubeMars::encode_pos_vel(float position_rad, float speed_limit_rads,
                                           uint8_t data[8])
{
    put_float_le(&data[0], position_rad);
    put_float_le(&data[4], speed_limit_rads);
}

// sec 6
void AP_PNUGimbal_CubeMars::encode_vel(float velocity_rads, uint8_t data[4])
{
    put_float_le(&data[0], velocity_rads);
}

// sec 7
void AP_PNUGimbal_CubeMars::decode_feedback(const uint8_t data[8], const Limits &limits,
                                            uint8_t &status, uint8_t &motor_id_low4,
                                            AP_PNUGimbal_Feedback &feedback)
{
    status        = data[0] >> 4;
    motor_id_low4 = data[0] & 0x0F;

    // unsigned intermediates throughout.  The manual's own example uses int16_t for the
    // position, which misrepresents every value at or above 0x8000 - that is half the
    // travel.
    const uint16_t p = (uint16_t(data[1]) << 8) | uint16_t(data[2]);
    // D3 is the high 8 bits of velocity, despite the manual's table calling it position;
    // the unpacking code and D4's layout both say velocity (sec 9.4).
    const uint16_t v = (uint16_t(data[3]) << 4) | uint16_t(data[4] >> 4);
    const uint16_t t = (uint16_t(data[4] & 0x0F) << 8) | uint16_t(data[5]);

    feedback.position_rad  = decode_unsigned(p, limits.p_min, limits.p_max, 16);
    feedback.velocity_rads = decode_unsigned(v, limits.v_min, limits.v_max, 12);
    feedback.torque_nm     = decode_unsigned(t, limits.t_min, limits.t_max, 12);
    feedback.driver_temp_c = int8_t(data[6]);
    feedback.motor_temp_c  = int8_t(data[7]);
    feedback.status        = status;
}

const char *AP_PNUGimbal_CubeMars::status_str(uint8_t status)
{
    switch (status) {
    case 0x0: return "disabled";
    case 0x1: return "enabled";
    case 0x8: return "overvoltage";
    case 0x9: return "undervoltage";
    case 0xA: return "overcurrent";
    case 0xB: return "MOS overtemp";
    case 0xC: return "winding overtemp";
    case 0xD: return "comms loss";
    case 0xE: return "overload";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

bool AP_PNUGimbal_CubeMars::protocol_init(void)
{
    const uint8_t mode = _frontend.get_gimbal_mode();
    if (mode > uint8_t(Mode::VEL)) {
        _frontend.set_init_error("PNU_MODE %u is not 0/1/2", (unsigned)mode);
        return false;
    }
    _mode = Mode(mode);

    _master_id = _frontend.get_master_id();
    if (_master_id > AP_HAL::CANFrame::MaskStdID) {
        _frontend.set_init_error("PNU_MSTID %u exceeds 11 bits", (unsigned)_master_id);
        return false;
    }

    // sec 7 ID conflict: the upper nibble of D0 is the status code, so feedback carries
    // only FOUR bits of motor identity.  Restricting ids to 1..15 makes that nibble a
    // complete demultiplexer and keeps the prose's eight-bit claim out of the code.
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        const uint8_t id = _frontend.get_motor_id(axis);
        if ((id < 1) || (id > 15)) {
            _frontend.set_init_error("%s motor id %u not 1..15", axis_name(axis), (unsigned)id);
            return false;
        }
        for (uint8_t prev = 0; prev < axis; prev++) {
            if (_motor_id[prev] == id) {
                _frontend.set_init_error("%s and %s share motor id %u",
                                         axis_name(prev), axis_name(axis), (unsigned)id);
                return false;
            }
        }
        _motor_id[axis] = id;
    }

    // the three motors report to one Master ID and are told apart by that nibble, so the
    // feedback id must not also be a command id for any of them
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (command_id(_mode, _motor_id[axis]) == _master_id) {
            _frontend.set_init_error("PNU_MSTID %u collides with the %s command id",
                                     (unsigned)_master_id, axis_name(axis));
            return false;
        }
    }

    const float p_max  = _frontend.get_p_max();
    const float v_max  = _frontend.get_v_max();
    const float kp_max = _frontend.get_kp_max();
    const float kd_max = _frontend.get_kd_max();
    const float t_max  = _frontend.get_t_max();

    if (!isfinite(p_max)  || !is_positive(p_max)  ||
        !isfinite(v_max)  || !is_positive(v_max)  ||
        !isfinite(kp_max) || !is_positive(kp_max) ||
        !isfinite(kd_max) || !is_positive(kd_max) ||
        !isfinite(t_max)  || !is_positive(t_max)) {
        _frontend.set_init_error("PNU_PMAX/VMAX/KPMAX/KDMAX/TMAX must all be > 0");
        return false;
    }

    // the base layer has already read and range-checked the mechanical limits; what only
    // this layer can check is whether the encoding range can express them
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (_mech_limit_rad[axis] > p_max) {
            // the limit is unreachable: the encoding range cannot express it, so the
            // clamp would never fire and the stops would be the only protection
            _frontend.set_init_error("%s mech limit exceeds PNU_PMAX", axis_name(axis));
            return false;
        }
    }

    // symmetric about zero for p, v and torque; 0 .. max for the two gains, which have
    // no meaning below zero.  See the Limits comment in the header for why asymmetric
    // device ranges are not supported.
    _limits.p_min  = -p_max;  _limits.p_max  = p_max;
    _limits.v_min  = -v_max;  _limits.v_max  = v_max;
    _limits.kp_min = 0.0f;    _limits.kp_max = kp_max;
    _limits.kd_min = 0.0f;    _limits.kd_max = kd_max;
    _limits.t_min  = -t_max;  _limits.t_max  = t_max;

    return true;
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------

void AP_PNUGimbal_CubeMars::transmit(uint32_t tick)
{
    // one clock for the whole driver; see AP_PNUGIMBAL_CUBEMARS_CMD_DIVISOR
    if ((tick % AP_PNUGIMBAL_CUBEMARS_CMD_DIVISOR) != 0) {
        return;
    }

    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (_pending[axis] != 0) {
            // a universal command takes the whole tick for that axis.  They are rare, and
            // nothing is gained by also sending a motion command in the same tick.
            // PNU-ISSUE(D1) lesson: the request is cleared only on a successful send, so
            // a momentarily full TX buffer retries next tick instead of dropping it.
            send_universal(axis);
            continue;
        }

        // allow_motion() is the state machine's gate and it is true only in RUN.  Nothing
        // moves in INIT, ALIGN, MAINTENANCE or FAULT, whatever targets are sitting here.
        if (!allow_motion() || !commanded_valid(axis)) {
            continue;
        }

        send_motion(axis);
    }
}

bool AP_PNUGimbal_CubeMars::any_pending(void) const
{
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (_pending[axis] != 0) {
            return true;
        }
    }
    return false;
}

// sec 7 : 0 is disabled and 1 is enabled; everything the table lists above that is a
// fault.  Codes the table does not list are deliberately NOT treated as faults - they are
// unknown, and guessing either way would be worse than reporting the raw value.
// Appendix 1 item 2 specifies MIT mode, and the reason is that MIT is the only mode in
// which the host bounds the torque pushed into a hard stop - kp = 0 with kd and v chosen
// gives a stall torque of kd * v.  A position/velocity-mode command runs a cascaded loop
// whose torque cannot be limited from the CAN frame at all, so driving a bulky payload into
// a mechanical stop in that mode is refused rather than attempted.
bool AP_PNUGimbal_CubeMars::homing_supported(const char *&reason) const
{
    if (_mode != Mode::MIT) {
        reason = "PNU_MODE=0";
        return false;
    }
    return true;
}

bool AP_PNUGimbal_CubeMars::status_is_fault(uint8_t status) const
{
    switch (status) {
    case 0x8:   // overvoltage
    case 0x9:   // undervoltage
    case 0xA:   // overcurrent
    case 0xB:   // MOS overtemperature
    case 0xC:   // winding overtemperature
    case 0xD:   // communication loss
    case 0xE:   // overload
        return true;
    }
    return false;
}

bool AP_PNUGimbal_CubeMars::send_universal(uint8_t axis)
{
    uint8_t &pending = _pending[axis];

    uint8_t bit;
    uint8_t action;
    if (pending & PENDING_CLEAR) {
        bit = PENDING_CLEAR;   action = UNIVERSAL_CLEAR;
    } else if (pending & PENDING_DISABLE) {
        bit = PENDING_DISABLE; action = UNIVERSAL_DISABLE;
    } else if (pending & PENDING_ENABLE) {
        bit = PENDING_ENABLE;  action = UNIVERSAL_ENABLE;
    } else if (pending & PENDING_ZERO) {
        bit = PENDING_ZERO;    action = UNIVERSAL_ZERO;
    } else {
        return false;
    }

    uint8_t data[AP_PNUGIMBAL_CUBEMARS_UNIVERSAL_DLC];
    memset(data, 0xFF, sizeof(data));
    data[sizeof(data) - 1] = action;

    // sec 3 is derived, not specified: the manual gives no separate arbitration id for the
    // universal commands, so they go out on the command id of the configured mode.  In
    // velocity mode that means a DLC-8 frame on an id whose ordinary command is DLC 4 -
    // the one place this is most likely to be wrong.  Confirm on hardware.
    if (!send_frame(command_id(_mode, _motor_id[axis]), data, sizeof(data))) {
        return false;
    }
    pending &= ~bit;
    return true;
}

void AP_PNUGimbal_CubeMars::send_motion(uint8_t axis)
{
    // commanded(), not the raw target: the slew limit lives one layer up and bounds how
    // fast a bulky payload can be told to move
    const AP_PNUGimbal_Target &target = commanded(axis);

    uint8_t data[8] = {};
    uint8_t dlc;

    switch (_mode) {
    case Mode::MIT:
        encode_mit(target, _limits, data);
        dlc = AP_PNUGIMBAL_CUBEMARS_MIT_DLC;
        break;
    case Mode::POS_VEL:
        // the second field is a maximum absolute travel speed, not a signed velocity
        // target, and the manual does not define a negative one (sec 5)
        encode_pos_vel(target.position_rad, fabsf(target.velocity_rads), data);
        dlc = AP_PNUGIMBAL_CUBEMARS_POSVEL_DLC;
        break;
    case Mode::VEL:
        encode_vel(target.velocity_rads, data);
        dlc = AP_PNUGIMBAL_CUBEMARS_VEL_DLC;
        break;
    default:
        // unreachable: init() rejects anything else before the thread starts
        return;
    }

    send_frame(command_id(_mode, _motor_id[axis]), data, dlc);
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

bool AP_PNUGimbal_CubeMars::handle_frame(const AP_HAL::CANFrame &frame)
{
    // sec 1 / sec 9 : classical CAN, standard 11-bit data frames.  Anything else is not this
    // protocol and must be rejected before the payload is looked at.  Returning false
    // rather than true matters on a shared bus: the PMU's extended frames must reach the
    // backend after this one.
    if (frame.isExtended() || frame.isRemoteTransmissionRequest() || frame.isErrorFrame()) {
        return false;
    }

    if ((frame.id & AP_HAL::CANFrame::MaskStdID) != _master_id) {
        // not our feedback id - the SAR, or another device sharing the bus
        return false;
    }

    // PNU-ISSUE(D6) lesson from AP_PMUCAN: validate the DLC before reading any field.
    // A short frame parsed from fixed offsets yields values that look live and are not.
    // It is still OUR frame, so it is claimed - reporting it twice would be worse.
    if (frame.dlc < AP_PNUGIMBAL_CUBEMARS_FB_DLC) {
        report_short_frame(frame.id, frame.dlc);
        return true;
    }

    AP_PNUGimbal_Feedback feedback;
    uint8_t status = 0;
    uint8_t motor_id_low4 = 0;
    decode_feedback(frame.data, _limits, status, motor_id_low4, feedback);

    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if ((_motor_id[axis] & 0x0F) != motor_id_low4) {
            continue;
        }

        AxisState &state = _axis[axis];

        // 0xFF is not a reachable status - the field is a nibble - so the first frame
        // after boot or after a feedback gap always reports the state it found
        const uint8_t prev_status = state.feedback.valid ? state.feedback.status : 0xFF;

        feedback.frame_cnt = state.feedback.frame_cnt + 1;
        feedback.last_update_ms = AP_HAL::millis();
        feedback.valid = true;
        state.feedback = feedback;
        state.timeout_reported = false;

        if (status != prev_status) {
            report_status_change(axis, status);
        }
        return true;
    }

    // arrived on our Master ID but with a motor id we do not drive.  Claimed anyway:
    // the id is ours, so letting it fall through to the driver's unclaimed-frame
    // warning would report the same frame under two different causes.
    report_unmatched_feedback(motor_id_low4);
    return true;
}

// ---------------------------------------------------------------------------
// Protocol primitives for the lifecycle in AP_PNUGimbal_Backend
// ---------------------------------------------------------------------------

void AP_PNUGimbal_CubeMars::queue_enable(bool enable)
{
    for (uint8_t axis = 0; axis < AP_PNUGIMBAL_NUM_AXES; axis++) {
        if (enable) {
            _pending[axis] = (_pending[axis] & ~PENDING_DISABLE) | PENDING_ENABLE;
        } else {
            _pending[axis] = (_pending[axis] & ~PENDING_ENABLE) | PENDING_DISABLE;
        }
    }
}

void AP_PNUGimbal_CubeMars::queue_zero(uint8_t axis)
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return;
    }
    // sec 3 : zero-setting persistence is unspecified, which is exactly why the layer
    // above only permits this in MAINTENANCE or FAULT.
    _pending[axis] |= PENDING_ZERO;
}

void AP_PNUGimbal_CubeMars::queue_clear_errors(uint8_t axis)
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return;
    }
    _pending[axis] |= PENDING_CLEAR;
}

// ---------------------------------------------------------------------------
// Reporting.  One rate limiter per cause, deliberately: sharing one lets a permanent
// condition crowd out a genuine fault, which is the 2026-10-14 lesson from the camera
// path.  The counter below is read by the message that reports it - PNU-ISSUE(D6) says
// not to accumulate counters nobody surfaces.
// ---------------------------------------------------------------------------

void AP_PNUGimbal_CubeMars::report_unmatched_feedback(uint8_t motor_id_low4)
{
    _unmatched_fb_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_unmatched_fb_cnt > 1) &&
        ((now_ms - _last_unmatched_fb_warn_ms) < AP_PNUGIMBAL_WARN_MS)) {
        return;
    }
    _last_unmatched_fb_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNUGimbal: feedback from motor id ?%u (%u dropped)",
                  (unsigned)motor_id_low4, (unsigned)_unmatched_fb_cnt);
}

void AP_PNUGimbal_CubeMars::report_status_change(uint8_t axis, uint8_t status)
{
    // edge triggered, so no rate limit is needed and none is wanted: a motor flapping
    // between two states is itself the thing worth seeing
    const MAV_SEVERITY severity = (status <= 0x1) ? MAV_SEVERITY_INFO : MAV_SEVERITY_CRITICAL;

    GCS_SEND_TEXT(severity, "PNUGimbal: %s %s (0x%x)",
                  axis_name(axis), status_str(status), (unsigned)status);
}

#endif  // AP_PNUGIMBAL_CUBEMARS_ENABLED
