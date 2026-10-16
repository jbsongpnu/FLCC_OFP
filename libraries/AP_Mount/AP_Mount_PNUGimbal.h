#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_PNUGIMBAL_ENABLED

#include "AP_Mount_Backend.h"

// PNU : AP_Mount backend for the PNU 3-axis gimbal.
//
// The mount itself is three CubeMars GL60 II KV28 motors on the shared PNU CAN bus; the
// protocol lives in AP_PNUGimbal_CubeMars under AP_PNU_CAN.  This class is the targeting
// adapter - it takes the mount target AP_Mount has already worked out, converts it from
// earth frame to body frame against AHRS, and hands the per-axis angles to the gimbal
// backend.  It carries no protocol knowledge and no CAN.
//
// WHY AN AP_Mount BACKEND, settled 2026-10-16.  AP_Mount already does FC-attitude
// stabilisation - AP_Mount_Servo::update_angle_outputs() is the whole of it - and it does
// not require a camera: the only pure virtuals in AP_Mount_Backend are has_pan_control()
// and get_attitude_quaternion(), and every camera method defaults to returning false.
// Backing a backend gives mount modes, RC targeting, the MAVLink gimbal manager, ROI,
// MNT1_*_MIN/MAX limits and the lead filter for free.
//
// THIS MOUNT HAS NO CAMERA.  What it stabilises is the SAR payload driven by AP_PNUSAR.
// Every camera method is left at its base-class default on purpose.
//
// LIMITS ARE SPLIT ACROSS TWO LAYERS, and that is deliberate:
//
//   - MNT1_ROLL/PITCH/YAW_MIN/MAX carry the OPERATING limits (+-25 / +-25 / +-15 deg) and
//     AP_Mount clamps targets to them, as it does for every backend.
//   - The MECHANICAL limits (+-30 / +-30 / +-20 deg) live in the gimbal backend, applied
//     to every target as the last thing before a frame is built.
//
// Defence in depth: a bug in the targeting layer cannot drive the payload into the landing
// skid struts, because the layer that owns the wire clamps again.  The 5 degree margin
// between the two is what absorbs stabilisation overshoot.
//
// WHAT THIS DOES NOT DO YET - CAN Update Plan step 1b:
//
//   - **It commands no motors.**  Targets are written through to the gimbal backend, but
//     AP_PNUGimbal_CubeMars transmits nothing until set_enable(true), and nothing in the
//     tree calls that yet.  The lifecycle state machine - INIT, HOMING, ALIGN, RUN,
//     MAINTENANCE, FAULT - is step 1c/1d and is what will enable the motors.
//   - There is therefore no homing.  The encoder is relative and its zero is assumed NOT
//     to persist, so until step 1d runs the Appendix 1 endpoint search the mechanical
//     clamp protects nothing: plus or minus 30 in encoder terms is not plus or minus 30
//     mechanically until zero is at the centre of travel.
//   - healthy() is therefore NOT yet gated on the state machine.  Step 1c must make it
//     false outside RUN, so that AP_Mount::pre_arm_checks() blocks arming on an unhomed
//     mount.

class AP_Mount_PNUGimbal : public AP_Mount_Backend
{
public:
    using AP_Mount_Backend::AP_Mount_Backend;

    CLASS_NO_COPY(AP_Mount_PNUGimbal);

    // AP_Mount_Backend
    void init() override;
    void update() override;
    bool healthy() const override;

    // three motors, one per axis
    bool has_roll_control() const override { return true; }
    bool has_pitch_control() const override { return true; }
    bool has_pan_control() const override { return true; }

    bool get_attitude_quaternion(Quaternion& att_quat) override;

protected:

    // angles only.  AP_Mount_Backend converts a rate target into an angle target for us,
    // which is what we want: the CubeMars drives take a position command and close their
    // own loop around it.
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_ONLY;
    }

    void send_target_angles(const MountAngleTarget &angle_rad) override;

private:

    // earth-frame target to body-frame motor angles, against AHRS.  Same shape as
    // AP_Mount_Servo::update_angle_outputs(), which is the only other FC-stabilised
    // backend in the tree.
    void update_angle_outputs(const MountAngleTarget &angle_rad);

    // PNU-ISSUE(D1/D10) lesson: every member carries an initialiser.
    Vector3f _angle_bf_output_rad;      // what we would command, body frame
    bool _reported_no_driver = false;   // one-shot "no gimbal on the CAN bus" notice
};

#endif  // HAL_MOUNT_PNUGIMBAL_ENABLED
