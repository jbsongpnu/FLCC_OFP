#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_PNUSAR/AP_PNUSAR.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_PNUSAR_ENABLED

// SAR_ICD_PNU.md section 9.1 gives exact expected bytes for five of the nine navigation
// frames.  Every vector below is quoted from that table unchanged, so a failure here
// means the firmware and Lumir's receiver would disagree on the wire - which is the only
// thing these tests are for.  The rule tests that follow come from sections 2.1 to 2.3
// and 6.2.

static void expect_bytes(const uint8_t got[], const uint8_t want[], uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        EXPECT_EQ(want[i], got[i]) << "byte " << int(i);
    }
}

// 9.1 : Lat 35.0 deg, Lng 129.0 deg
// "The 0x400 example transmits the scaled integers 350,000,000 and 1,290,000,000.  Do
// not multiply these integers by 10^7 again."  AP_GPS hands us exactly those int32s.
TEST(PNUSARNav, Icd91GpsPosition)
{
    uint8_t data[8] = {};
    AP_PNUSAR::put_le_i32(&data[0], 350000000);
    AP_PNUSAR::put_le_i32(&data[4], 1290000000);
    const uint8_t want[8] = { 0x80, 0x93, 0xDC, 0x14, 0x80, 0xD6, 0xE3, 0x4C };
    expect_bytes(data, want, 8);
}

// 9.1 : Alt 100.0 m, Spd 5.0 m/s, GCrs 90.0 deg
// Alt arrives from Location::alt already in centimetres and is NOT scaled again (2.1).
TEST(PNUSARNav, Icd91GpsMotion)
{
    uint8_t data[8] = {};
    bool rng = false;
    AP_PNUSAR::put_le_i32(&data[0], 10000);
    AP_PNUSAR::put_le16(&data[4], AP_PNUSAR::encode_u16(5.0f, 100.0f, rng));
    AP_PNUSAR::put_le16(&data[6], AP_PNUSAR::encode_angle(90.0f, rng));
    const uint8_t want[8] = { 0x10, 0x27, 0x00, 0x00, 0xF4, 0x01, 0x28, 0x23 };
    expect_bytes(data, want, 8);
    EXPECT_FALSE(rng);
}

// 9.1 : VZ -0.5 m/s, GPS_Yaw 50.0 deg, HDop 0.8, VDop 1.2
// HDop/VDop reach the packer already multiplied by 100 (AP_GPS), so the test supplies
// the integers rather than the physical values - double-scaling them is the mistake 2.1
// names explicitly.
TEST(PNUSARNav, Icd91GpsAux)
{
    uint8_t data[8] = {};
    bool rng = false;
    AP_PNUSAR::put_le_i16(&data[0], AP_PNUSAR::encode_i16(-0.5f, 100.0f, rng));
    AP_PNUSAR::put_le16(&data[2], AP_PNUSAR::encode_angle(50.0f, rng));
    AP_PNUSAR::put_le16(&data[4], 80);
    AP_PNUSAR::put_le16(&data[6], 120);
    const uint8_t want[8] = { 0xCE, 0xFF, 0x88, 0x13, 0x50, 0x00, 0x78, 0x00 };
    expect_bytes(data, want, 8);
    EXPECT_FALSE(rng);
}

// 9.1 : HAcc 0.02 m, VAcc 0.03 m, SAcc 0.04 m/s, YAcc 0.05 deg
TEST(PNUSARNav, Icd91GpsAccuracy)
{
    uint8_t data[8] = {};
    bool rng = false;
    AP_PNUSAR::put_le16(&data[0], AP_PNUSAR::encode_u16(0.02f, 100.0f, rng));
    AP_PNUSAR::put_le16(&data[2], AP_PNUSAR::encode_u16(0.03f, 100.0f, rng));
    AP_PNUSAR::put_le16(&data[4], AP_PNUSAR::encode_u16(0.04f, 100.0f, rng));
    AP_PNUSAR::put_le16(&data[6], AP_PNUSAR::encode_u16(0.05f, 100.0f, rng));
    const uint8_t want[8] = { 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00 };
    expect_bytes(data, want, 8);
    EXPECT_FALSE(rng);
}

// 9.1 : Roll -1.23 deg, Pitch 2.34 deg, Yaw 50.0 deg, ATT_Seq 1, Flags 0x0F
// The firmware does not transmit these values yet - 0x404 carries sentinels until step
// 3f - but the LAYOUT is what this proves, and it is the layout 3f will fill.
TEST(PNUSARNav, Icd91Attitude)
{
    uint8_t data[8] = {};
    bool rng = false;
    AP_PNUSAR::put_le_i16(&data[0], AP_PNUSAR::encode_i16(-1.23f, 100.0f, rng));
    AP_PNUSAR::put_le_i16(&data[2], AP_PNUSAR::encode_i16(2.34f, 100.0f, rng));
    AP_PNUSAR::put_le16(&data[4], AP_PNUSAR::encode_angle(50.0f, rng));
    data[6] = 1;
    data[7] = AP_PNUSAR_ATTF_ROLL_VALID | AP_PNUSAR_ATTF_PITCH_VALID |
              AP_PNUSAR_ATTF_YAW_VALID | AP_PNUSAR_ATTF_FRESH;
    const uint8_t want[8] = { 0x85, 0xFF, 0xEA, 0x00, 0x88, 0x13, 0x01, 0x0F };
    expect_bytes(data, want, 8);
    EXPECT_FALSE(rng);
    // The ICD's own flag value, 0x0F, must be exactly the four defined bits - if a
    // RESERVED bit ever crept into that set this would catch it.
    EXPECT_EQ(0x0FU, (unsigned)data[7]);
}

// 2.2 : little-endian, least significant byte first, for the 64-bit snapshot time too
TEST(PNUSARNav, SnapshotTimeIsLittleEndian)
{
    uint8_t data[8] = {};
    AP_PNUSAR::put_le64(data, 0x0123456789ABCDEFULL);
    const uint8_t want[8] = { 0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01 };
    expect_bytes(data, want, 8);
}

// 2.1 : "The same rule applies to negative values: -1.235 * 100 rounds to -124."
TEST(PNUSARNav, RoundsHalfAwayFromZero)
{
    bool rng = false;
    EXPECT_EQ(-124, AP_PNUSAR::encode_i16(-1.235f, 100.0f, rng));
    EXPECT_EQ(124, AP_PNUSAR::encode_i16(1.235f, 100.0f, rng));
    EXPECT_EQ(1234, AP_PNUSAR::encode_i16(12.34f, 100.0f, rng));
    EXPECT_FALSE(rng);
}

// 6.2 : normalise to [0, 360) BEFORE rounding, then replace the rounded 36000 with zero
// so 359.999 deg does not survive as a second representation of north.
TEST(PNUSARNav, AngleWrapAndTheThirtySixThousandRule)
{
    bool rng = false;
    EXPECT_EQ(0U, AP_PNUSAR::encode_angle(359.999f, rng));
    EXPECT_EQ(0U, AP_PNUSAR::encode_angle(360.0f, rng));
    EXPECT_EQ(0U, AP_PNUSAR::encode_angle(0.0f, rng));
    EXPECT_EQ(35999U, AP_PNUSAR::encode_angle(359.99f, rng));
    EXPECT_EQ(27000U, AP_PNUSAR::encode_angle(-90.0f, rng));    // wraps to 270
    EXPECT_EQ(9000U, AP_PNUSAR::encode_angle(450.0f, rng));     // wraps to 90
    EXPECT_FALSE(rng);
}

// 2.3 : out of range becomes the SENTINEL and sets ENCODING_ERROR.  6.3 : "Transmit
// 0xFFFF for an out-of-range value rather than saturating to a valid value."
TEST(PNUSARNav, OutOfRangeIsSentinelNotSaturation)
{
    bool rng = false;
    EXPECT_EQ(AP_PNUSAR_INVALID_U16, AP_PNUSAR::encode_u16(700.0f, 100.0f, rng));
    EXPECT_TRUE(rng);

    rng = false;
    EXPECT_EQ(AP_PNUSAR_INVALID_U16, AP_PNUSAR::encode_u16(-0.01f, 100.0f, rng));
    EXPECT_TRUE(rng);

    rng = false;
    EXPECT_EQ(int16_t(AP_PNUSAR_INVALID_I16), AP_PNUSAR::encode_i16(400.0f, 100.0f, rng));
    EXPECT_TRUE(rng);

    // The boundary itself is valid: u16 runs to 65534, i16 to +/-32767.
    rng = false;
    EXPECT_EQ(65534U, AP_PNUSAR::encode_u16(655.34f, 100.0f, rng));
    EXPECT_EQ(32767, AP_PNUSAR::encode_i16(327.67f, 100.0f, rng));
    EXPECT_EQ(-32767, AP_PNUSAR::encode_i16(-327.67f, 100.0f, rng));
    EXPECT_FALSE(rng);
}

// 2.3 : NaN and infinity take the sentinel too
TEST(PNUSARNav, NonFiniteIsSentinel)
{
    bool rng = false;
    EXPECT_EQ(AP_PNUSAR_INVALID_U16, AP_PNUSAR::encode_u16(nanf(""), 100.0f, rng));
    EXPECT_TRUE(rng);

    rng = false;
    EXPECT_EQ(int16_t(AP_PNUSAR_INVALID_I16), AP_PNUSAR::encode_i16(INFINITY, 100.0f, rng));
    EXPECT_TRUE(rng);
}

// 2.3 : the sentinels are the document's, not ours
TEST(PNUSARNav, SentinelValues)
{
    EXPECT_EQ(0xFFFFU, (unsigned)AP_PNUSAR_INVALID_U16);
    EXPECT_EQ(-32768, (int)AP_PNUSAR_INVALID_I16);
    EXPECT_EQ(0xFFFFFFFFUL, (unsigned long)AP_PNUSAR_INVALID_U32);
}

// 5 and 7.3 : nine frames, 0x400 through 0x408, 72 payload bytes in one logical snapshot
TEST(PNUSARNav, BundleShape)
{
    EXPECT_EQ(9U, (unsigned)AP_PNUSAR_NAV_FRAMES);
    EXPECT_EQ(AP_PNUSAR_ID_NAV_LAST,
              AP_PNUSAR_ID_NAV_FIRST + AP_PNUSAR_NAV_FRAMES - 1U);
    EXPECT_EQ(72U, (unsigned)(AP_PNUSAR_NAV_FRAMES * AP_PNUSAR_NAV_DLC));
}

#endif  // AP_PNUSAR_ENABLED

AP_GTEST_MAIN()
