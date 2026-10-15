#pragma once

#include "AP_PNUGimbal_config.h"

#if AP_PNUGIMBAL_CUBEMARS_ENABLED

#include "AP_PNUGimbal_Backend.h"

// PNU : CubeMars GLII driver-family CAN protocol, as specified in
// libraries/PNUGimbal_CAN_Protocol_Type_CubeMars.md.  Section references below are to
// that file, which is itself a reading of gl-ii-gimbal-motor-drive-user-manual.pdf
// V1.0.0.  Target hardware is three GL60 II KV28, one per axis.
//
// Classical CAN, standard 11-bit data frames, fixed 1 Mbit/s (sec 1).  Nothing here is
// DroneCAN and there is no node discovery: every id is configured, not negotiated.
//
// THINGS THE MANUAL DOES NOT SPECIFY, and which this backend therefore does not invent
// (sec 1, sec 9).  If a future session is tempted to add any of these, it needs a bench
// measurement first, not a plausible default:
//   - command period, feedback trigger and feedback rate
//   - any loss timeout, watchdog interval or recovery sequence
//   - acknowledgement of the universal commands
//   - a runtime mode-switch command.  The mode is written by the vendor PC utility and
//     the motor is power-cycled; CAN_Dn_PG_MODE only tells this backend what the motors
//     were already configured for, and a mismatch is undefined behaviour.
//
// The motors are enabled at power-up per the manual (sec 1), so this backend does not send
// an enable at startup either - it would be commanding a state it cannot read back.
// set_enable() is an explicit action from the layer above.

#define AP_PNUGIMBAL_CUBEMARS_FB_DLC        (8U)        // sec 7, feedback is always 8 bytes
#define AP_PNUGIMBAL_CUBEMARS_MIT_DLC       (8U)        // sec 4
#define AP_PNUGIMBAL_CUBEMARS_POSVEL_DLC    (8U)        // sec 5
#define AP_PNUGIMBAL_CUBEMARS_VEL_DLC       (4U)        // sec 6, deliberately not 8
#define AP_PNUGIMBAL_CUBEMARS_UNIVERSAL_DLC (8U)        // sec 3

// How long an axis may go without feedback before get_feedback() stops reporting it as
// valid.  sec 9 leaves the stale-feedback policy to the application; this backend's policy
// is to invalidate and say so once, and to do nothing else - disabling a motor or
// changing a target on a feedback gap is a control-layer decision, not a protocol one.
#define AP_PNUGIMBAL_CUBEMARS_FB_TIMEOUT_MS (500U)

// Rate limits.  Deliberately one per cause: a 2026-10-14 lesson from the camera path is
// that several permanent warnings sharing one limiter crowd out genuine faults.
#define AP_PNUGIMBAL_CUBEMARS_WARN_MS       (10000U)

class AP_PNUGimbal_CubeMars : public AP_PNUGimbal_Backend {
public:
    using AP_PNUGimbal_Backend::AP_PNUGimbal_Backend;

    CLASS_NO_COPY(AP_PNUGimbal_CubeMars);

    // The configured control mode, which is also bits 10..8 of every command
    // arbitration id (sec 2).  It must match what the PC utility wrote to the motors.
    enum class Mode : uint8_t {
        MIT     = 0,    // sec 4, impedance style: p, v, kp, kd, feed-forward torque
        POS_VEL = 1,    // sec 5, float32 target angle + float32 speed limit
        VEL     = 2,    // sec 6, float32 target rate, DLC 4
    };

    // Encode / decode limits.  sec 4 and sec 7 map physical values onto unsigned offset
    // integers between these bounds, so THEY MUST MATCH THE VALUES THE PC UTILITY WROTE
    // TO THE MOTOR.  A host/device mismatch does not fail, it silently scales every
    // command and every reading.
    //
    // The parameters expose one magnitude per field and this struct is filled
    // symmetrically from them (-max .. +max for p, v and torque; 0 .. max for kp and
    // kd), matching the manual's preset table.  Asymmetric device limits are legal in
    // the PC utility and are NOT supported here; they would need five more parameters.
    struct Limits {
        float p_min = 0.0f, p_max = 0.0f;
        float v_min = 0.0f, v_max = 0.0f;
        float kp_min = 0.0f, kp_max = 0.0f;
        float kd_min = 0.0f, kd_max = 0.0f;
        float t_min = 0.0f, t_max = 0.0f;
    };

    // AP_PNUGimbal_Backend
    bool init(void) override;
    void update(void) override;
    void handle_frame(const AP_HAL::CANFrame &frame) override;
    void set_target(uint8_t axis, const AP_PNUGimbal_Target &target) override;
    bool get_feedback(uint8_t axis, AP_PNUGimbal_Feedback &feedback) const override;
    void set_enable(bool enable) override;
    void request_zero(uint8_t axis) override;
    void request_clear_errors(uint8_t axis) override;
    bool healthy(void) const override;
    const char *name(void) const override { return "CubeMars"; }

    // ---------------------------------------------------------------------------
    // Pure codec.  No CAN, no parameters, no state - everything below is a function of
    // its arguments, which is what makes the published vectors in sec 8 usable as tests.
    // ---------------------------------------------------------------------------

    // sec 2 : command_id = (mode << 8) | motor_id
    static uint32_t command_id(Mode mode, uint8_t motor_id) {
        return (uint32_t(mode) << 8) | uint32_t(motor_id);
    }

    // sec 4.1 : unsigned offset mapping.  NOT two's complement and NOT float32 - packed
    // zero sits near the middle of the integer range, so an all-zero MIT payload is not
    // a zero command.
    //
    // Q is (1 << bits) - 1.  The manual's own transmit function uses (1 << bits), which
    // overflows the field at maximum; its receive function uses (1 << bits) - 1.  We use
    // the receive form in both directions so that encode and decode are inverses, and
    // saturate rather than wrap.  sec 9.2 - verify against firmware.
    static uint16_t encode_unsigned(float x, float x_min, float x_max, uint8_t bits);
    static float decode_unsigned(uint16_t u, float x_min, float x_max, uint8_t bits);

    // sec 4.2 : p(16) v(12) kp(12) kd(12) t(12) bit-packed into 8 bytes.
    static void encode_mit(const AP_PNUGimbal_Target &target, const Limits &limits,
                           uint8_t data[8]);

    // sec 5 : float32 little-endian target angle, then float32 little-endian speed limit.
    static void encode_pos_vel(float position_rad, float speed_limit_rads, uint8_t data[8]);

    // sec 6 : float32 little-endian target rate, 4 bytes.
    static void encode_vel(float velocity_rads, uint8_t data[4]);

    // sec 7 : one feedback frame.  status and motor_id_low4 come out separately because
    // they share D0 and neither is part of the physical state.
    static void decode_feedback(const uint8_t data[8], const Limits &limits,
                                uint8_t &status, uint8_t &motor_id_low4,
                                AP_PNUGimbal_Feedback &feedback);

    // sec 7 status table.  0 and 1 are normal operating states, not faults; anything the
    // table does not list is reported by its raw value rather than guessed at.
    static const char *status_str(uint8_t status);

    // sec 3 universal commands, the last payload byte of FF FF FF FF FF FF FF xx.
    static const uint8_t UNIVERSAL_ENABLE  = 0xFC;
    static const uint8_t UNIVERSAL_DISABLE = 0xFD;
    static const uint8_t UNIVERSAL_ZERO    = 0xFE;
    static const uint8_t UNIVERSAL_CLEAR   = 0xFB;

private:

    // pending universal commands, sent one per axis per update() in this bit order
    static const uint8_t PENDING_CLEAR   = 1U << 0;
    static const uint8_t PENDING_DISABLE = 1U << 1;
    static const uint8_t PENDING_ENABLE  = 1U << 2;
    static const uint8_t PENDING_ZERO    = 1U << 3;

    // PNU-ISSUE(D1/D10) lesson: every member carries an initialiser, including the ones
    // inside this struct.
    struct AxisState {
        AP_PNUGimbal_Target target;
        AP_PNUGimbal_Feedback feedback;
        uint8_t pending = 0;
        bool target_valid = false;      // nothing is commanded until a target arrives
        bool timeout_reported = false;
    };

    bool send_universal(uint8_t axis);
    void send_motion(uint8_t axis);
    void check_feedback_timeout(void);

    void report_unmatched_feedback(uint8_t motor_id_low4);
    void report_rejected_target(uint8_t axis);
    void report_status_change(uint8_t axis, uint8_t status);
    static const char *axis_name(uint8_t axis);

    AxisState _axis[AP_PNUGIMBAL_NUM_AXES];

    Mode _mode = Mode::MIT;
    Limits _limits;
    uint16_t _master_id = 0;                        // feedback arbitration id (sec 7)
    uint8_t _motor_id[AP_PNUGIMBAL_NUM_AXES] = {};  // 1..15, distinct (sec 7 ID conflict)
    bool _enabled = false;

    uint32_t _unmatched_fb_cnt = 0;
    uint32_t _last_unmatched_fb_warn_ms = 0;
    uint32_t _rejected_target_cnt = 0;
    uint32_t _last_rejected_target_warn_ms = 0;
};

#endif  // AP_PNUGIMBAL_CUBEMARS_ENABLED
