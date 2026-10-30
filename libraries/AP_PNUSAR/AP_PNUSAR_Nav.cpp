/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// PNU : the SAR's 10 Hz navigation bundle, SAR_ICD_PNU.md sections 5, 6 and 7.
//
// Nine frames, 0x400 through 0x408, eight bytes each, in ascending id order, every
// 100 ms while an observation is running.  One logical snapshot is 72 payload bytes and
// NOT one CAN frame; 0x408 closes the bundle with its acquisition timestamp rather than
// wrapping the others.
//
// The split between threads is the design, not an implementation detail.  nav_snapshot()
// runs on the VEHICLE thread, where AP_GPS::update() also runs, so it reads one
// consistent GPS sample and encodes the whole bundle before anything else can touch it;
// send_bundle() runs on the CAN thread and does nothing but write the bytes out.  Reading
// twenty-odd GPS fields from the CAN thread could straddle a GPS update and build a
// bundle from two different samples, which is exactly what ICD 7.3 forbids.

#include "AP_PNUSAR.h"

#if AP_PNUSAR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_PNU_CAN/AP_PNU_CAN.h>

#if AP_GPS_ENABLED
#include <AP_GPS/AP_GPS.h>
#endif

// -------------------------------------------------------------------------
// Serialisation helpers
// -------------------------------------------------------------------------
// PNU-ISSUE(D15) : the byte order is stated here, at the only place that writes it.
// SAR_ICD_PNU.md 2.2 : every multi-byte field is LITTLE-ENDIAN, least significant byte
// first, and a structure is never serialised by copying its memory - that would inherit
// host byte order and whatever padding the compiler chose.
// -------------------------------------------------------------------------
void AP_PNUSAR::put_le16(uint8_t *p, uint16_t v)
{
    p[0] = uint8_t(v & 0xFFU);
    p[1] = uint8_t((v >> 8) & 0xFFU);
}

void AP_PNUSAR::put_le32(uint8_t *p, uint32_t v)
{
    p[0] = uint8_t(v & 0xFFU);
    p[1] = uint8_t((v >> 8) & 0xFFU);
    p[2] = uint8_t((v >> 16) & 0xFFU);
    p[3] = uint8_t((v >> 24) & 0xFFU);
}

void AP_PNUSAR::put_le64(uint8_t *p, uint64_t v)
{
    put_le32(&p[0], uint32_t(v & 0xFFFFFFFFULL));
    put_le32(&p[4], uint32_t((v >> 32) & 0xFFFFFFFFULL));
}


// -------------------------------------------------------------------------
// Scaling helpers, SAR_ICD_PNU.md 2.1 and 2.3
// -------------------------------------------------------------------------
// transmitted_integer = round_half_away_from_zero(physical_value * multiplication), which
// is exactly what roundf() does.
//
// @range_err is set ONLY for a value that could not be represented - NaN, infinity, or a
// number outside the field after scaling.  It becomes GPS_Flags bit 4 ENCODING_ERROR,
// which ICD 6.5 defines as "a range error, not merely an unavailable value": a field the
// source simply does not provide takes its sentinel without setting the bit.  Keeping
// those two apart is the whole point of the bit.
//
// A value out of range is NOT saturated to the largest valid one (ICD 6.3 says so in as
// many words); it becomes the sentinel, because a plausible number is worse than an
// honest refusal.
// -------------------------------------------------------------------------
uint16_t AP_PNUSAR::encode_u16(float physical, float mult, bool &range_err)
{
    if (!isfinite(physical)) {
        range_err = true;
        return AP_PNUSAR_INVALID_U16;
    }
    const float scaled = roundf(physical * mult);
    if ((scaled < 0.0f) || (scaled > 65534.0f)) {
        range_err = true;
        return AP_PNUSAR_INVALID_U16;
    }
    return uint16_t(scaled);
}

int16_t AP_PNUSAR::encode_i16(float physical, float mult, bool &range_err)
{
    if (!isfinite(physical)) {
        range_err = true;
        return int16_t(AP_PNUSAR_INVALID_I16);
    }
    const float scaled = roundf(physical * mult);
    if ((scaled < -32767.0f) || (scaled > 32767.0f)) {
        range_err = true;
        return int16_t(AP_PNUSAR_INVALID_I16);
    }
    return int16_t(scaled);
}

// A quantity the source may simply not provide.  Unavailable takes the sentinel WITHOUT
// setting @range_err, because ICD 6.5 keeps "the receiver did not report this" and "the
// number would not fit" apart, and only the second is ENCODING_ERROR.
//
// @available is passed in rather than called here on purpose: the AP_GPS accessors write
// their value through a reference, and reading that reference in the same argument list
// as the call would be indeterminately sequenced.
static uint16_t enc_opt_u16(bool available, float physical, float mult, bool &range_err)
{
    if (!available) {
        return AP_PNUSAR_INVALID_U16;
    }
    return AP_PNUSAR::encode_u16(physical, mult, range_err);
}

// Angles: ICD 6.2 requires normalisation to [0, 360) BEFORE rounding, and then that the
// rounded integer 36000 is replaced by zero - otherwise 359.999 deg survives as a second
// representation of north that a receiver has to know about.
uint16_t AP_PNUSAR::encode_angle(float deg, bool &range_err)
{
    const uint16_t v = encode_u16(wrap_360(deg), 100.0f, range_err);
    return (v == 36000U) ? 0U : v;
}


// -------------------------------------------------------------------------
// nav_snapshot : acquire and encode one bundle, on the VEHICLE thread
// -------------------------------------------------------------------------
// Called at 10 Hz with the driver semaphore held.  It also owns ICD 14.7's automatic
// termination, because the conditions that trigger it are the same source values the
// snapshot reads, and checking them anywhere else would mean reading them twice.
// -------------------------------------------------------------------------
void AP_PNUSAR::nav_snapshot(bool nav_failsafe)
{
    if (!_nav_tx_enabled) {
        // ICD 7.4 : whenever navigation is off, unsent bundles are discarded rather than
        // held for a later resume.  Data acquired before a stop is not data after it.
        _bundle.ready = false;
        return;
    }

    // ICD 14.7 : "FC navigation/EKF failsafe - terminate immediately; do not add a wait
    // for age thresholds."  Checked first, and NOT gated on CAN_Dn_PNU_SCHK: the other
    // conditions describe the quality of the data, this one is the vehicle saying it no
    // longer knows where it is, and "SAR handling must not replace or delay the FC's own
    // flight failsafe actions."
    if (nav_failsafe) {
        auto_terminate("nav failsafe");
        return;
    }

#if AP_GPS_ENABLED
    const AP_GPS &gps = AP::gps();
    const uint8_t checks = _frontend.get_sar_start_checks();

    // ICD 4 : latch the Primary instance for the whole bundle.  Reading it once here is
    // what makes "do not alternate or mix GPS1/GPS2" true - every field below comes from
    // this instance - and a Primary change therefore applies at the next bundle boundary,
    // which is where the ICD wants it so the receiver can see the source change.
    const uint8_t inst = gps.primary_sensor();
    const AP_GPS::GPS_Status status = gps.status(inst);
    const uint32_t gps_age_ms = AP_HAL::millis() - gps.last_message_time_ms(inst);

    // ---- ICD 14.7 automatic termination ------------------------------------------
    // The thresholds are the ICD's; which of them are ENFORCED follows CAN_Dn_PNU_SCHK,
    // the same bitmask that gates the Start interlock.  That is deliberate: a bench with
    // no RTK fix can be allowed to Start, and it would be useless if the first snapshot
    // 100 ms later tore the observation down again.  The default enforces the ICD set.
    if ((checks & AP_PNUSAR_SCHK_RTK_FIXED) && (status <= AP_GPS::GPS_OK_FIX_3D)) {
        // "Ordinary 3D_FIX (3) or lower" only.  DGPS (4) and RTK_FLOAT (5) continue:
        // Float explicitly does (14.7), and in-observation DGPS handling is left TBD by
        // the ICD itself (10, remaining check 3), so nothing is invented for it here.
        auto_terminate("fix degraded");
        return;
    }
    if ((checks & AP_PNUSAR_SCHK_GPS_AGE) && (gps_age_ms > AP_PNUSAR_GPS_AGE_MAX_MS)) {
        auto_terminate("GPS age");
        return;
    }
    if (checks & AP_PNUSAR_SCHK_ATT_AGE) {
        // There is no attitude source until step 3f, so an enforced ATT age check can
        // only ever fail.  Saying so is better than a bundle that quietly never flows.
        auto_terminate("no ATT source");
        return;
    }

    // ---- the bundle ---------------------------------------------------------------
    // A snapshot still sitting here means the CAN thread did not reach the last one
    // before this one replaced it.  That IS a dropped bundle and is counted as one:
    // without this the loss would be invisible on the FC and show up only as a gap in
    // ATT_Seq at the far end, which is precisely the silent failure PNU-ISSUE(D6) exists
    // to remove.
    if (_bundle.ready) {
        _bundle_drop_cnt++;
        report_bundle_drop("overrun");
    }

    memset(_bundle.data, 0, sizeof(_bundle.data));
    uint8_t (&f)[AP_PNUSAR_NAV_FRAMES][8] = _bundle.data;
    bool rng = false;

    // 0x400 GPS_POSITION : B0-3 Lat i32 *1e7, B4-7 Lng i32 *1e7.
    // Location already holds degrees * 1e7 as int32.  ICD 2.1 : use those values
    // directly, and never through a float - 1e-7 deg does not survive float32, and the
    // document calls that out by name.
    const Location &loc = gps.location(inst);
    put_le_i32(&f[0][0], loc.lat);
    put_le_i32(&f[0][4], loc.lng);

    // 0x401 GPS_MOTION : Alt i32 *100, Spd u16 *100, GCrs u16 *100.
    // Location::alt is ALREADY centimetres above MSL.  ICD 2.1 : do not apply *100 again.
    put_le_i32(&f[1][0], loc.alt);
    put_le16(&f[1][4], encode_u16(gps.ground_speed(inst), 100.0f, rng));
    // GCrs is ground course, not vehicle heading.  The ICD warns against using it as
    // heading when stationary; that is the receiver's business, and the FC transmits it
    // as what it is.
    put_le16(&f[1][6], encode_angle(gps.ground_course(inst), rng));

    // 0x402 GPS_AUX : VZ i16 *100, GPS_Yaw u16 *100, HDop u16, VDop u16.
    // VZ is the NED down component, POSITIVE DOWNWARD (ICD 6.2).  AP_GPS velocity() is
    // already NED, so .z carries that sign and must not be negated.
    const bool have_vv = gps.have_vertical_velocity(inst);
    int16_t vz = int16_t(AP_PNUSAR_INVALID_I16);
    if (have_vv) {
        vz = encode_i16(gps.velocity(inst).z, 100.0f, rng);
    }
    put_le_i16(&f[2][0], vz);
    float yaw_deg = 0.0f;
    float yaw_acc_deg = 0.0f;
    uint32_t yaw_time_ms = 0;
    const bool have_yaw = gps.gps_yaw_deg(inst, yaw_deg, yaw_acc_deg, yaw_time_ms);
    uint16_t gps_yaw = AP_PNUSAR_INVALID_U16;
    if (have_yaw) {
        gps_yaw = encode_angle(yaw_deg, rng);
    }
    put_le16(&f[2][2], gps_yaw);
    // get_hdop()/get_vdop() already carry the *100 scaling - ICD 2.1 : "Do not
    // double-scale the internal *100 representations of HDop/VDop" - and AP_GPS already
    // uses 0xFFFF (GPS_UNKNOWN_DOP) for unknown, which is the ICD's own sentinel.
    put_le16(&f[2][4], gps.get_hdop(inst));
    put_le16(&f[2][6], gps.get_vdop(inst));

    // 0x403 GPS_ACCURACY : HAcc, VAcc, SAcc, YAcc, all u16 *100.
    // Each accessor returns false when the receiver does not report that quantity, which
    // is the distinction ICD 2.3 asks for: "A zero in a log must not, by itself, be
    // interpreted as zero uncertainty."
    float hacc = 0.0f;
    float vacc = 0.0f;
    float sacc = 0.0f;
    const bool have_hacc = gps.horizontal_accuracy(inst, hacc);
    const bool have_vacc = gps.vertical_accuracy(inst, vacc);
    const bool have_sacc = gps.speed_accuracy(inst, sacc);
    // YAcc only when the SENSOR reported one.  gps_yaw_deg() substitutes a flat 10
    // degrees when it did not, and ICD 6.2 asks for that fallback to be distinguished -
    // transmitting it would be inventing an accuracy the receiver would then trust.
    const bool have_yacc = gps.have_gps_yaw_accuracy(inst);
    put_le16(&f[3][0], enc_opt_u16(have_hacc, hacc, 100.0f, rng));
    put_le16(&f[3][2], enc_opt_u16(have_vacc, vacc, 100.0f, rng));
    put_le16(&f[3][4], enc_opt_u16(have_sacc, sacc, 100.0f, rng));
    put_le16(&f[3][6], enc_opt_u16(have_yacc, yaw_acc_deg, 100.0f, rng));

    // 0x404 ATTITUDE : sentinels until step 3f.
    // ICD 16.4 : "If actual attitude is unavailable, use existing sentinels and per-axis
    // ATT_Flags.  Do not substitute zero/segment_heading setpoints and mark them valid."
    // Zeros would read as a level, north-facing SAR to anything that looks at the values
    // before the flags, so the sentinels go out and ATT_Flags stays empty - no VALID bit
    // and no ATT_FRESH.  ATT_Seq still advances, so a bundle dropped before it reached
    // the bus leaves a visible gap rather than vanishing.
    put_le_i16(&f[4][0], int16_t(AP_PNUSAR_INVALID_I16));
    put_le_i16(&f[4][2], int16_t(AP_PNUSAR_INVALID_I16));
    put_le16(&f[4][4], AP_PNUSAR_INVALID_U16);
    f[4][6] = _att_seq++;
    f[4][7] = 0;

    // 0x405 GPS_TIME_STATUS : GMS u32, GWk u16, Status u8, NSats u8.
    const uint16_t gwk = gps.time_week(inst);
    const bool time_valid = (gwk != 0);
    const uint32_t gms = time_valid ? gps.time_week_ms(inst) : uint32_t(AP_PNUSAR_INVALID_U32);
    put_le32(&f[5][0], gms);
    put_le16(&f[5][4], time_valid ? gwk : AP_PNUSAR_INVALID_U16);
    f[5][6] = uint8_t(status);
    // ICD 6.4 lists 255 for an unknown satellite count, but AP_GPS reports 0 both for
    // "none in view" and for "the receiver did not say", so 255 is never transmitted
    // here.  A zero means what AP_GPS means by zero and nothing more.
    f[5][7] = gps.num_sats(inst);

    // 0x406 GPS_SAMPLE_INFO : SMS u32, Delta u16, I u8, GPS_Flags u8.
    // SMS is exempt from the sentinel rules (ICD 2.3) because it is the raw millisecond
    // counter: 0xFFFFFFFF is a legitimate value just before it rolls over to zero.  It
    // is also NOT a measurement epoch - it is when the FC received the message.
    put_le32(&f[6][0], gps.last_message_time_ms(inst));
    put_le16(&f[6][4], gps.last_message_delta_time_ms(inst));
    f[6][6] = inst;

    // 0x407 GPS_DIAGNOSTICS : AEl i32 *100, RTCMFU u16, RTCMFD u16.
    // AEl is height above the WGS84 ellipsoid, built the same way the GPA log record
    // builds it: AMSL centimetres minus the geoid separation.  ICD 6.1 : "Do not copy Alt
    // into an unavailable AEl field and label it ellipsoid height."
    float undulation = 0.0f;
    const bool have_ael = gps.get_undulation(inst, undulation);
    int32_t ael_cm = int32_t(AP_PNUSAR_INVALID_I32);
    if (have_ael) {
        ael_cm = loc.alt - int32_t(roundf(undulation * 100.0f));
    }
    put_le_i32(&f[7][0], ael_cm);
    uint16_t rtcm_used = 0;
    uint16_t rtcm_discarded = 0;
    gps.get_rtcm_stats(rtcm_used, rtcm_discarded);
    // ICD 6.4 : preserve the source counters' own rollover and reset behaviour; the
    // no-reset requirement does not apply to these two.
    put_le16(&f[7][4], rtcm_used);
    put_le16(&f[7][6], rtcm_discarded);

    // 0x408 SNAPSHOT_TIME_US : u64, microseconds since FC boot.
    // The same time axis as the dataflash TimeUS (ICD 6.4).  It timestamps acquisition of
    // this bundle, not any GPS measurement epoch - those stay in SMS and GMS/GWk.
    const uint64_t snapshot_us = AP_HAL::micros64();
    put_le64(&f[8][0], snapshot_us);

    // GPS_Flags last, because ENCODING_ERROR can only be known once every field above
    // has been encoded.
    uint8_t gps_flags = 0;
    if (inst == gps.primary_sensor()) {
        // Always true by construction here.  Transmitted from the comparison rather than
        // as a constant 1, because ICD 6.5 defines the bit as a fact about the sample and
        // this is where that fact lives.
        gps_flags |= AP_PNUSAR_GPSF_USED;
    }
    if (have_vv) {
        gps_flags |= AP_PNUSAR_GPSF_VV;
    }
    if (gps_age_ms <= AP_PNUSAR_GPS_AGE_MAX_MS) {
        gps_flags |= AP_PNUSAR_GPSF_FRESH;
    }
    if (time_valid) {
        gps_flags |= AP_PNUSAR_GPSF_TIME_VALID;
    }
    if (rng) {
        gps_flags |= AP_PNUSAR_GPSF_ENC_ERROR;
    }
    if (have_ael) {
        gps_flags |= AP_PNUSAR_GPSF_AEL_VALID;
    }
    // The counters are real now that AP_GPS exposes them, so the bit says so.  ICD 6.5
    // keeps it as the way a receiver tells unavailable statistics from genuine zeroes.
    gps_flags |= AP_PNUSAR_GPSF_RTCM_STATS;
    f[6][7] = gps_flags;

    _bundle.time_us = snapshot_us;
    _bundle.ready = true;

    // ICD 15.1 : a session must START in RTK_FIXED, and post-processing discards the
    // whole session if the first transmitted snapshot is not Status 6.  The FC cannot
    // enforce that - the Start interlock already passed - but flying an entire sortie
    // whose data will be thrown away is worth one line at the time it becomes true.
    if (_session_first_bundle) {
        _session_first_bundle = false;
        if (status != AP_GPS::GPS_OK_FIX_3D_RTK_FIXED) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: session starts at Status %u, not 6",
                          (unsigned)status);
        }
    }
#else
    (void)nav_failsafe;
    auto_terminate("no GPS in build");
#endif  // AP_GPS_ENABLED
}


// -------------------------------------------------------------------------
// auto_terminate : SAR_ICD_PNU.md section 14.7
// -------------------------------------------------------------------------
// "Disable navigation immediately and discard unsent bundles to avoid contaminated data.
// If heartbeat remains live, send 0x502/01 to request physical SAR Stop and wait 2 s for
// 0x522.  Record response/timeout as internal results."
//
// The order matters: the data stops whether or not the device can be told.
// -------------------------------------------------------------------------
void AP_PNUSAR::auto_terminate(const char *reason)
{
    _nav_tx_enabled = false;
    _bundle.ready = false;
    _auto_stop_cnt++;

    // 14.7 and 14.8 : the current indication for this is recovery-pending, which is
    // SAR_STATUS 0 with data off - deliberately NOT state 2, which would claim a
    // confirmed normal Stop, and not state 4 either.  A reconnect does not release it,
    // so the way back is a power cycle, exactly as after a heartbeat loss.
    _recovery_pending = true;
    publish_state();

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: observation aborted, %s", reason);

    if (!_link_up) {
        // "Do not assume FC actually stopped SAR hardware across a broken connection."
        return;
    }
    _auto_stop_request = true;
}


// -------------------------------------------------------------------------
// send_bundle : nine frames in order, on the CAN thread
// -------------------------------------------------------------------------
void AP_PNUSAR::send_bundle(void)
{
    if (!_bundle.ready) {
        return;
    }
    _bundle.ready = false;

    // ICD 7.2 : check the age again before a delayed transmission.  A bundle the CAN
    // thread could not reach in time is DROPPED, not sent late - its Snapshot_TimeUS
    // would then disagree with when the data actually reached the bus, and the receiver
    // has no way to notice.
    if ((AP_HAL::micros64() - _bundle.time_us) >
        (uint64_t)AP_PNUSAR_SNAPSHOT_MAX_AGE_MS * 1000ULL) {
        _bundle_drop_cnt++;
        report_bundle_drop("stale");
        return;
    }

    for (uint8_t i = 0; i < AP_PNUSAR_NAV_FRAMES; i++) {
        if (!send_frame(AP_PNUSAR_ID_NAV_FIRST + i, _bundle.data[i], AP_PNUSAR_NAV_DLC)) {
            // ICD 7.3 : the receiver discards a bundle with a missing id anyway, so the
            // remaining frames are abandoned rather than put on a congested bus to be
            // thrown away at the other end.  The driver has already counted and reported
            // the send failure itself.
            _bundle_drop_cnt++;
            report_bundle_drop("TX");
            return;
        }
    }
    _bundle_tx_cnt++;
}


// -------------------------------------------------------------------------
// report_bundle_drop : rate limited, and it reads both counters
// -------------------------------------------------------------------------
// PNU-ISSUE(D6) : _bundle_tx_cnt exists because this message quotes it.  A drop count
// with nothing to compare it against says far less than "3 of 1204".
// -------------------------------------------------------------------------
void AP_PNUSAR::report_bundle_drop(const char *why)
{
    const uint32_t now_ms = AP_HAL::millis();
    if ((_bundle_drop_cnt != 1) &&
        ((now_ms - _last_bundle_warn_ms) < AP_PNUSAR_BUNDLE_WARN_MS)) {
        return;
    }
    _last_bundle_warn_ms = now_ms;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SAR: bundle %s, %lu of %lu",
                  why, (unsigned long)_bundle_drop_cnt,
                  (unsigned long)(_bundle_tx_cnt + _bundle_drop_cnt));
}

#endif  // AP_PNUSAR_ENABLED


// -------------------------------------------------------------------------
// PNU_SAR_nav_snapshot : the vehicle thread's entry point
// -------------------------------------------------------------------------
// Outside the AP_PNUSAR_ENABLED guard, like SAR_Status, because ArduCopter's scheduler
// calls it whether or not the CAN driver is in the build.
// -------------------------------------------------------------------------
void PNU_SAR_nav_snapshot(bool nav_failsafe)
{
#if AP_PNUSAR_ENABLED
    AP_PNU_CAN *pnu_can = AP_PNU_CAN::get_singleton();
    if (pnu_can == nullptr) {
        return;
    }
    AP_PNUSAR *sar = pnu_can->get_sar();
    if (sar == nullptr) {
        return;                         // CAN_Dn_PNU_OPTS bit 2 is clear
    }
    // The CAN thread transmits from the same bundle, so the hand-off is guarded.  The
    // hold is short on purpose: encoding, no I/O.
    WITH_SEMAPHORE(pnu_can->get_semaphore());
    sar->nav_snapshot(nav_failsafe);
#else
    (void)nav_failsafe;
#endif  // AP_PNUSAR_ENABLED
}
