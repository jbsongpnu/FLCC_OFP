#include "AP_Mount_config.h"

#if HAL_MOUNT_PNUGIMBAL_ENABLED

#include "AP_Mount_PNUGimbal.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_PNU_CAN/AP_PNU_CAN.h>

extern const AP_HAL::HAL& hal;

void AP_Mount_PNUGimbal::init()
{
    AP_Mount_Backend::init();
}

void AP_Mount_PNUGimbal::update()
{
    AP_Mount_Backend::update();

    update_mnt_target();

    // have our base class call send_target_angles to command the gimbal
    send_target_to_gimbal();
}

// called by the base class once it has a target for us
void AP_Mount_PNUGimbal::send_target_angles(const MountAngleTarget& angle_rad)
{
    update_angle_outputs(angle_rad);

    AP_PNU_CAN *pnu_can = AP::pnu_can();
    AP_PNUGimbal_Backend *gimbal = (pnu_can != nullptr) ? pnu_can->get_gimbal() : nullptr;
    if (gimbal == nullptr) {
        // MNT1_TYPE selects this mount but no gimbal exists on the CAN bus - either
        // CAN_Dn_PROTOCOL does not select AP_PNU_CAN, or CAN_Dn_PNU_OPTS bit 1 is clear.
        // Say so once: a mount that is configured and silently does nothing is the
        // PNU-ISSUE(D6) problem again.
        if (!_reported_no_driver) {
            _reported_no_driver = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "PNUGimbal: no gimbal on CAN, check CAN_Dn_PNU_OPTS bit 1");
        }
        return;
    }

    // Writing a target does NOT move anything yet.  AP_PNUGimbal_CubeMars transmits only
    // when it has been enabled, and the lifecycle state machine that enables it is step
    // 1c.  So this is the full path, exercised and logged, with the motors silent - see
    // the class comment.
    AP_PNUGimbal_Target target;
    target.position_rad = _angle_bf_output_rad.x;
    target.velocity_rads = 0.0f;
    WITH_SEMAPHORE(pnu_can->get_semaphore());
    gimbal->set_target(AP_PNUGIMBAL_AXIS_ROLL, target);

    target.position_rad = _angle_bf_output_rad.y;
    gimbal->set_target(AP_PNUGIMBAL_AXIS_PITCH, target);

    target.position_rad = _angle_bf_output_rad.z;
    gimbal->set_target(AP_PNUGIMBAL_AXIS_YAW, target);
}

// Report what we would command.  AP_Mount_Backend::write_log() turns this into the MNT
// message's "actual" columns, so MNT.DRoll against MNT.Roll is the earth-frame target
// against the body-frame demand - and the difference between them IS the AHRS correction.
// That is what makes step 1b provable from a log with no motors connected.
//
// TODO(step 1c): once the state machine drives the motors, report the MEASURED angles from
// gimbal->get_feedback() when they are valid.  Unlike AP_Mount_Servo, which notes "No
// feedback from gimbal so simply report demanded servo angles", this mount has real
// encoder feedback, so GIMBAL_DEVICE_ATTITUDE_STATUS can carry truth rather than intent.
bool AP_Mount_PNUGimbal::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(constrain_float(_angle_bf_output_rad.x,
                                        radians(_params.roll_angle_min), radians(_params.roll_angle_max)),
                        constrain_float(_angle_bf_output_rad.y,
                                        radians(_params.pitch_angle_min), radians(_params.pitch_angle_max)),
                        constrain_float(_angle_bf_output_rad.z,
                                        radians(_params.yaw_angle_min), radians(_params.yaw_angle_max)));
    return true;
}

// Healthy means the gimbal exists and every axis is reporting fresh feedback.
//
// NOTE for step 1c: this must also return false outside the RUN state, so that
// AP_Mount::pre_arm_checks() blocks arming on a mount that has not homed.  With a relative
// encoder whose zero is assumed not to persist, an unhomed mount's angles are relative to
// wherever it powered up, and the mechanical clamp means nothing.
bool AP_Mount_PNUGimbal::healthy() const
{
    AP_PNU_CAN *pnu_can = AP::pnu_can();
    if (pnu_can == nullptr) {
        return false;
    }
    AP_PNUGimbal_Backend *gimbal = pnu_can->get_gimbal();
    if (gimbal == nullptr) {
        return false;
    }
    return gimbal->healthy();
}

// private methods

// Convert an earth-frame angle target into the body-frame angles the motors must hold.
//
// This is AP_Mount_Servo::update_angle_outputs() adapted: the maths is the same because
// the problem is the same - a mount with no attitude reference of its own, stabilised by
// the FC.  The CubeMars drives are the requires_stabilization == true case.
void AP_Mount_PNUGimbal::update_angle_outputs(const MountAngleTarget& angle_rad)
{
    const AP_AHRS &ahrs = AP::ahrs();

    // target yaw in body frame, with the operating limits applied
    const float yaw_bf_rad = constrain_float(angle_rad.get_bf_yaw(),
                                             radians(_params.yaw_angle_min),
                                             radians(_params.yaw_angle_max));

    // default output to the target earth-frame roll and pitch, body-frame yaw
    _angle_bf_output_rad.x = angle_rad.roll;
    _angle_bf_output_rad.y = angle_rad.pitch;
    _angle_bf_output_rad.z = yaw_bf_rad;

    // no stabilisation in retract or neutral - those are mechanical positions, not
    // attitudes to hold
    switch (mnt_target.target_type) {
    case MountTargetType::NEUTRAL:
    case MountTargetType::RETRACTED:
        return;
    case MountTargetType::ANGLE:
    case MountTargetType::RATE:
    case MountTargetType::LOCATION:
        break;
    }

    // lean angles from AHRS, rotated into the gimbal's yaw frame
    Vector2f ahrs_angle_rad = {ahrs.get_roll_rad(), ahrs.get_pitch_rad()};
    ahrs_angle_rad.rotate(-yaw_bf_rad);

    // subtract the vehicle's lean so the payload holds its earth-frame attitude.
    //
    // This is where the travel budget goes, and it is worth knowing: at the +-25 degree
    // operating limit, a vehicle leaning 25 degrees consumes the entire range, and a
    // multicopter in forward flight leans 15-25 degrees routinely.  The clamp below is
    // then correct behaviour, not a fault - see the README.
    if (angle_rad.roll_is_ef) {
        _angle_bf_output_rad.x -= ahrs_angle_rad.x;
    }
    if (angle_rad.pitch_is_ef) {
        _angle_bf_output_rad.y -= ahrs_angle_rad.y;
    }

    // lead filter, MNT1_LEAD_RLL / MNT1_LEAD_PTCH.  Compensates the lag between the
    // vehicle leaning and the mount reacting; zero by default.
    const Vector3f &gyro = ahrs.get_gyro();

    if (!is_zero(_params.roll_stb_lead) && fabsf(ahrs.get_pitch_rad()) < M_PI/3.0f) {
        const float roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) *
                                (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
        _angle_bf_output_rad.x -= roll_rate * _params.roll_stb_lead;
    }
    if (!is_zero(_params.pitch_stb_lead)) {
        const float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;
        _angle_bf_output_rad.y -= pitch_rate * _params.pitch_stb_lead;
    }

    // apply the OPERATING limits.  The MECHANICAL limits are applied again in the gimbal
    // backend, which is the layer that owns the wire - see the class comment.
    _angle_bf_output_rad.x = constrain_float(_angle_bf_output_rad.x,
                                             radians(_params.roll_angle_min),
                                             radians(_params.roll_angle_max));
    _angle_bf_output_rad.y = constrain_float(_angle_bf_output_rad.y,
                                             radians(_params.pitch_angle_min),
                                             radians(_params.pitch_angle_max));
}

#endif  // HAL_MOUNT_PNUGIMBAL_ENABLED
