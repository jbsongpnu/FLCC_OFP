#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for stabilize flight mode for trad heli
 */

// Vertical SAS (Stability Augmentation System) damping gain
// Units: collective_fraction / (m/s), e.g. 0.05 means 1 m/s descent speed increases collective by 0.05 (5%)
#define VERTICAL_SAS_DAMPING_GAIN 0.03f

// stabilize_init - initialise stabilize controller
bool ModeStabilize_Heli::init(bool ignore_checks)
{
    // be aware that when adding code to this function that it is *NOT
    // RUN* at vehicle startup!

    // set stab collective true to use stabilize scaled collective pitch range
    copter.input_manager.set_use_stab_col(true);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize_Heli::run()
{
    float target_roll, target_pitch;
    float pilot_throttle_scaled;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot's desired throttle
    pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup while flying, because
    // we may be in autorotation flight.  This is so that the servos move in a realistic fashion while disarmed
    // for operational checks. Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero
    // so the swash servos move.

    //To Do : Allow idle state
    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        // heli will not let the spool state progress to THROTTLE_UNLIMITED until motor interlock is enabled
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // If aircraft is landed, set target heading to current and reset the integrator
        // Otherwise motors could be at ground idle for practice autorotation
        if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
            attitude_control->reset_yaw_target_and_rate(false);
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (copter.ap.land_complete && !motors->using_leaky_integrator()) {
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // Vertical SAS: apply damping to collective based on vertical speed
    // Only active when spooled up, not landed, and not in autorotation
    if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED &&
        !copter.ap.land_complete &&
        !copter.heli_flags.in_autorotation) {
        Vector3f vel_ned;
        if (AP::ahrs().get_velocity_NED(vel_ned)) {
            // vel_ned.z is positive-down (m/s), so adding it increases collective on descent
            cxdata().ctrl.V_z = vel_ned.z;
            const float sas_correction = constrain_float(VERTICAL_SAS_DAMPING_GAIN * vel_ned.z, -0.1f, 0.1f);
            cxdata().ctrl.SAS = sas_correction;
            pilot_throttle_scaled = constrain_float(pilot_throttle_scaled + sas_correction, 0.0f, 1.0f);
            cxdata().ctrl.Pilot_th_scaled = pilot_throttle_scaled;
        }
    }

    // output pilot's throttle - note that TradHeli does not used angle-boost
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}

#endif  //HELI_FRAME
