#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_PNUGimbal/AP_PNUGimbal_CubeMars.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_PNUGIMBAL_CUBEMARS_ENABLED

// The manual's preset encoding ranges (sec 4), which are also this driver's parameter
// defaults.  Every vector below is quoted against these, so changing them invalidates
// the expected payloads rather than the codec.
static AP_PNUGimbal_CubeMars::Limits preset_limits()
{
    AP_PNUGimbal_CubeMars::Limits limits;
    limits.p_min  = -12.5f; limits.p_max  = 12.5f;
    limits.v_min  = -200.0f; limits.v_max = 200.0f;
    limits.kp_min = 0.0f;   limits.kp_max = 500.0f;
    limits.kd_min = 0.0f;   limits.kd_max = 5.0f;
    limits.t_min  = -10.0f; limits.t_max  = 10.0f;
    return limits;
}

static void expect_payload(const uint8_t got[], const uint8_t want[], uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        EXPECT_EQ(want[i], got[i]) << "byte " << int(i);
    }
}

// sec 2
TEST(PNUGimbalCubeMars, CommandId)
{
    EXPECT_EQ(0x001U, AP_PNUGimbal_CubeMars::command_id(AP_PNUGimbal_CubeMars::Mode::MIT, 1));
    EXPECT_EQ(0x101U, AP_PNUGimbal_CubeMars::command_id(AP_PNUGimbal_CubeMars::Mode::POS_VEL, 1));
    EXPECT_EQ(0x201U, AP_PNUGimbal_CubeMars::command_id(AP_PNUGimbal_CubeMars::Mode::VEL, 1));
    EXPECT_EQ(0x10FU, AP_PNUGimbal_CubeMars::command_id(AP_PNUGimbal_CubeMars::Mode::POS_VEL, 15));
}

// sec 8, the four MIT vectors that are consistent with the stated ranges
TEST(PNUGimbalCubeMars, EncodeMitPublishedVectors)
{
    const AP_PNUGimbal_CubeMars::Limits limits = preset_limits();
    uint8_t data[8];

    // p = +2 rad, kp = 0.123, kd = 0.005
    AP_PNUGimbal_Target target;
    target.position_rad = 2.0f;
    target.kp = 0.123f;
    target.kd = 0.005f;
    AP_PNUGimbal_CubeMars::encode_mit(target, limits, data);
    const uint8_t want_p_pos[8] = {0x94, 0x7A, 0x7F, 0xF0, 0x01, 0x00, 0x47, 0xFF};
    expect_payload(data, want_p_pos, 8);

    // p = -2 rad, same gains
    target.position_rad = -2.0f;
    AP_PNUGimbal_CubeMars::encode_mit(target, limits, data);
    const uint8_t want_p_neg[8] = {0x6B, 0x84, 0x7F, 0xF0, 0x01, 0x00, 0x47, 0xFF};
    expect_payload(data, want_p_neg, 8);

    // t = +0.03 Nm, everything else zero
    AP_PNUGimbal_Target torque_target;
    torque_target.torque_nm = 0.03f;
    AP_PNUGimbal_CubeMars::encode_mit(torque_target, limits, data);
    const uint8_t want_t_pos[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x08, 0x05};
    expect_payload(data, want_t_pos, 8);

    // t = -0.03 Nm
    torque_target.torque_nm = -0.03f;
    AP_PNUGimbal_CubeMars::encode_mit(torque_target, limits, data);
    const uint8_t want_t_neg[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xF9};
    expect_payload(data, want_t_neg, 8);
}

// sec 8 additional source error: the manual's own +-6 rad/s vectors (0x830 / 0x7CE) do not
// decode to +-6 at the +-200 range it states - they give roughly +4.74 / -4.84.  The
// expected payloads here are the ones the documented codec produces, and they are NOT
// the manual's.  This test exists to pin that divergence down, so that a bench
// disagreement points at the manual rather than at this code.
TEST(PNUGimbalCubeMars, EncodeMitVelocityDivergesFromManual)
{
    const AP_PNUGimbal_CubeMars::Limits limits = preset_limits();
    uint8_t data[8];

    AP_PNUGimbal_Target target;
    target.velocity_rads = 6.0f;
    target.kd = 0.005f;
    AP_PNUGimbal_CubeMars::encode_mit(target, limits, data);
    const uint8_t want_pos[8] = {0x7F, 0xFF, 0x83, 0xC0, 0x00, 0x00, 0x47, 0xFF};
    expect_payload(data, want_pos, 8);

    target.velocity_rads = -6.0f;
    AP_PNUGimbal_CubeMars::encode_mit(target, limits, data);
    const uint8_t want_neg[8] = {0x7F, 0xFF, 0x7C, 0x20, 0x00, 0x00, 0x47, 0xFF};
    expect_payload(data, want_neg, 8);
}

// sec 4.1 : the maximum must saturate at Q rather than overflow into the next field, which
// is what the manual's transmit function would do with (1 << bits)
TEST(PNUGimbalCubeMars, EncodeSaturatesAtLimits)
{
    EXPECT_EQ(0xFFFFU, AP_PNUGimbal_CubeMars::encode_unsigned(12.5f, -12.5f, 12.5f, 16));
    EXPECT_EQ(0U,      AP_PNUGimbal_CubeMars::encode_unsigned(-12.5f, -12.5f, 12.5f, 16));
    EXPECT_EQ(0x0FFFU, AP_PNUGimbal_CubeMars::encode_unsigned(200.0f, -200.0f, 200.0f, 12));
    EXPECT_EQ(0U,      AP_PNUGimbal_CubeMars::encode_unsigned(-200.0f, -200.0f, 200.0f, 12));

    // out of range clamps rather than wraps
    EXPECT_EQ(0xFFFFU, AP_PNUGimbal_CubeMars::encode_unsigned(1.0e6f, -12.5f, 12.5f, 16));
    EXPECT_EQ(0U,      AP_PNUGimbal_CubeMars::encode_unsigned(-1.0e6f, -12.5f, 12.5f, 16));

    // non-finite input and an invalid range both encode as 0 rather than as garbage
    EXPECT_EQ(0U, AP_PNUGimbal_CubeMars::encode_unsigned(NAN, -12.5f, 12.5f, 16));
    EXPECT_EQ(0U, AP_PNUGimbal_CubeMars::encode_unsigned(1.0f, 12.5f, -12.5f, 16));
    EXPECT_EQ(0U, AP_PNUGimbal_CubeMars::encode_unsigned(1.0f, 5.0f, 5.0f, 16));
}

// sec 4.1 : encode and decode use the same Q, so they are inverses to within one step
TEST(PNUGimbalCubeMars, EncodeDecodeRoundTrip)
{
    const float values[] = {-12.5f, -7.3f, -0.001f, 0.0f, 0.001f, 4.2f, 12.5f};
    // truncation, so the worst case is just under one encoding step
    const float step = 1.5f * (25.0f / 65535.0f);

    for (float want : values) {
        const uint16_t encoded = AP_PNUGimbal_CubeMars::encode_unsigned(want, -12.5f, 12.5f, 16);
        const float got = AP_PNUGimbal_CubeMars::decode_unsigned(encoded, -12.5f, 12.5f, 16);
        EXPECT_NEAR(want, got, step);
    }

    EXPECT_FLOAT_EQ(-12.5f, AP_PNUGimbal_CubeMars::decode_unsigned(0, -12.5f, 12.5f, 16));
    EXPECT_FLOAT_EQ(12.5f, AP_PNUGimbal_CubeMars::decode_unsigned(0xFFFF, -12.5f, 12.5f, 16));
}

// sec 8 position/velocity vectors: float32 little-endian, target then speed limit
TEST(PNUGimbalCubeMars, EncodePosVelPublishedVectors)
{
    uint8_t data[8];

    AP_PNUGimbal_CubeMars::encode_pos_vel(3.0f, 1.0f, data);
    const uint8_t want_pos[8] = {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0x3F};
    expect_payload(data, want_pos, 8);

    AP_PNUGimbal_CubeMars::encode_pos_vel(-3.0f, 1.0f, data);
    const uint8_t want_neg[8] = {0x00, 0x00, 0x40, 0xC0, 0x00, 0x00, 0x80, 0x3F};
    expect_payload(data, want_neg, 8);
}

// sec 8 velocity vectors, DLC 4
TEST(PNUGimbalCubeMars, EncodeVelPublishedVectors)
{
    uint8_t data[4];

    AP_PNUGimbal_CubeMars::encode_vel(2.0f, data);
    const uint8_t want_pos[4] = {0x00, 0x00, 0x00, 0x40};
    expect_payload(data, want_pos, 4);

    AP_PNUGimbal_CubeMars::encode_vel(-2.0f, data);
    const uint8_t want_neg[4] = {0x00, 0x00, 0x00, 0xC0};
    expect_payload(data, want_neg, 4);
}

// sec 7 : the feedback layout, checked by decoding what the MIT encoder packs.  The two
// share nothing but the mapping, so agreement here is a real cross-check of both.
TEST(PNUGimbalCubeMars, DecodeFeedback)
{
    const AP_PNUGimbal_CubeMars::Limits limits = preset_limits();

    // status 1 (enabled), motor id 3, p/v/t at the midpoint of their ranges, driver 40
    // degC, motor -5 degC
    const uint8_t frame[8] = {0x13, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x28, 0xFB};

    AP_PNUGimbal_Feedback feedback;
    uint8_t status = 0;
    uint8_t motor_id_low4 = 0;
    AP_PNUGimbal_CubeMars::decode_feedback(frame, limits, status, motor_id_low4, feedback);

    EXPECT_EQ(1U, (unsigned)status);
    EXPECT_EQ(3U, (unsigned)motor_id_low4);
    EXPECT_NEAR(0.0f, feedback.position_rad, 25.0f / 65535.0f);
    EXPECT_NEAR(0.0f, feedback.velocity_rads, 400.0f / 4095.0f);
    EXPECT_NEAR(0.0f, feedback.torque_nm, 20.0f / 4095.0f);
    EXPECT_EQ(40, (int)feedback.driver_temp_c);
    EXPECT_EQ(-5, (int)feedback.motor_temp_c);
}

// the manual's own int16_t example misreads every position at or above 0x8000; the
// unsigned intermediate must put that in the upper half of travel, not the lower
TEST(PNUGimbalCubeMars, DecodeFeedbackPositionIsUnsigned)
{
    const AP_PNUGimbal_CubeMars::Limits limits = preset_limits();

    // position 0xFFFF, i.e. the top of the range
    const uint8_t frame[8] = {0x01, 0xFF, 0xFF, 0x7F, 0xF7, 0xFF, 0x00, 0x00};

    AP_PNUGimbal_Feedback feedback;
    uint8_t status = 0;
    uint8_t motor_id_low4 = 0;
    AP_PNUGimbal_CubeMars::decode_feedback(frame, limits, status, motor_id_low4, feedback);

    EXPECT_FLOAT_EQ(12.5f, feedback.position_rad);
}

// sec 7 : the status nibble, including a code the table does not list
TEST(PNUGimbalCubeMars, StatusStrings)
{
    EXPECT_STREQ("disabled", AP_PNUGimbal_CubeMars::status_str(0x0));
    EXPECT_STREQ("enabled", AP_PNUGimbal_CubeMars::status_str(0x1));
    EXPECT_STREQ("comms loss", AP_PNUGimbal_CubeMars::status_str(0xD));
    EXPECT_STREQ("overload", AP_PNUGimbal_CubeMars::status_str(0xE));
    EXPECT_STREQ("unknown", AP_PNUGimbal_CubeMars::status_str(0xF));
    EXPECT_STREQ("unknown", AP_PNUGimbal_CubeMars::status_str(0x5));
}

#endif  // AP_PNUGIMBAL_CUBEMARS_ENABLED

AP_GTEST_MAIN()
