# Coaxial Rotor Heli Project

@ Main Code Branch : CoaxialV2   
@ Sub Code Branch : LoadCell_4p2p2
***

## Version History
- V0.01.33 Bug fixed in AP_MotorsHeli_Dual::move_actuators(), organized code with many information
- V0.01.32 Fixing error: "Internal_Error 0x4000020". Also, working with CoaxServo tests and finding swashplate logics.
- V0.01.31 Added Mavlink processing functions related to V0.01.30
- V0.01.30 Updating common.xml for CoaxServo, IFCU and PMS
- V0.01.29 Added CoaxServo initialization, checking parameters, scheduled broadcasting and reading data
- V0.01.28 Added function to receive response from Pegasus servo motors.
- V0.01.27 Added functions to transmit commands to Pegasus servo motors. Added basic UART receive testing functions. Modefied common.xml.
- V0.01.26 Adding basic structure of library for a new CoaxServo with Pegasus Actuators' servo motor. Also modifying CoaxialCAN1 and CoaxialCAN2
- V0.01.25 Updating IFCU and PMS data processing functions. Added SYS_ICD_FLCC_GCS_HBSYS and logging. Also fixed bug for sending MSG_CCB_STATE.
- V0.01.24 Added CCB ICD with AP_CoaxialCAN1. Added Mavlink processing MAV_CMD_CCB_CONTROL, SYS_ICD_FLCC_GCS_CCB_STATE. Added logging too.
- V0.01.23 Logging problem solved. GCS TX and Logging from User-Medium Loop. Few bug fixes
- V0.01.22 Trying to fix the problem with downstream : currently 62000 down stream is not working
- V0.01.21 Adding Mavlink downstream and logging : SYS_ICD_FLCC_GCS_INV_STATE for motor inverter
- V0.01.20 Adding Mavlink command processes : MAV_CMD_COAX_SET_MOTOR, MAV_CMD_INVERTER_OPTION
- V0.01.19 Creating global variables for Inverter code of CoaxCAN1, and initial processes
- V0.01.18 Adding CoaxCAN1 with Inverter ICD. Few fixes with CoaxCAN2 
- V0.01.17 Testing 10Hz user-hook from scheduler : USERHOOK_MEDIUMLOOP userhook_MediumLoop()
- V0.01.16 Testing global data storage : Coaxial_data.cpp/h
- V0.01.15 Testing new Mavlink command : MAV_CMD_COAX_FCC_READY
- V0.01.14 AP_CoaxCAN2 is updated with receving PMS codes (+ some debugging)
- V0.01.13 Reorganizing CAN ICD(IFCU and PMS) with AP_CoaxCAN_msg_List.h
- V0.01.12 Adding New CoaxCAN2, moving CoaxCAN1's IFCU code to CoaxCAN2
- V0.01.11 Adding CAN Protocol for IFCU - 2. RX (Testing codes)
- V0.01.10 Adding CAN Protocol for IFCU - 1. TX (Testing codes)
- V0.01.09 Test code for Cooling Control Board RX with TX polling message
- V0.01.08 Adding Cooling Control Board RX messages - part1
- V0.01.07 Test code for CAN TX and RX : Passed test
- V0.01.06 Error in AP_CoaxCAN1.cpp is fixed. "Examp" was too long for AP_Param to initialize
- V0.01.05 Adding preliminary loop and run code with receive and transmit an example CAN frame
- V0.01.04 Added required codes for AP_CANManager::init() to initialize AP_COAXCAN1 class
- V0.01.03 Starting to link AP_CoaxCAN1 to CAN Manager
- V0.01.02 A new structure for AP_CoaxCAN1 library has been added
- V0.01.01 Creating new project with 4.4.2 version

***
## Parameter Setup (for Pixhawk6X)
### CAN Port Setup
- CAN_D1_PROTOCOL :   1       (CoaxCAN1)
- CAN_D2_PROTOCOL :   2       (CoaxCAN2)
- CAN_P1_DRIVER :     13      (CoaxCAN1 - CAN1)
- CAN_P2_DRIVER :     14      (CoaxCAN2 - CAN2)
- CAN_P1_BITRATE :    500000  (500Kbps)
- CAN_P2_BITRATE :    500000  (500Kbps)
### SERIAL Port Setup
- SERIAL0_BAUD :      115     (115,200bps)
- SERIAL0_PROTOCOL :  2       (MAVLink2 - USB)
- SERIAL1_BAUD :      57      (57,600bps)
- SERIAL1_PROTOCOL :  2       (MAVLink2 - Telemetry1 - ADT1)
- SERIAL2_BAUD :      57      (57,600bps)
- SERIAL2_PROTOCOL :  2       (MAVLink2 - Telemetry2 - ADT2)
- SERIAL3_BAUD :      38      (38,400bps)
- SERIAL3_PROTOCOL :  5       (GPS1)
- SERIAL4_BAUD :      921     (921,600bps)      
- SERIAL4_PROTOCOL :  36      (AHRS - GPS2 - VN-200)
- SERIAL5_BAUD :      115     (115,200bps)      
- SERIAL5_PROTOCOL :  45      (CoaxServo - Telemetry3 - PegasusActuators)
### More
- For VectorNAV VN-200, refer to [https://ardupilot.org/copter/docs/common-external-ahrs-vectornav.html](https://ardupilot.org/copter/docs/common-external-ahrs-vectornav.html)
- Helicopter Setup to Coaxial Rotor, refer to [https://ardupilot.org/copter/docs/dual-helicopter.html#coaxial](https://ardupilot.org/copter/docs/dual-helicopter.html#coaxial)
- FRAME_CLASS :       11      (Heli_Dual including Coaxial Rotor)
- H_DUAL_MODE :       2       (Intermeshing or Coaxial)
- H_SW_TYPE :         0       (H3 Generic Swashplate : counter clockwise rotation)
- H_SW2_TYPE :        0       (H3 Generic Swashplate : clockwise rotation)
***
## Tuning Control Parameters
- ATC_ACCEL_x_Max should be less than 36000 for heavy-weight helicopter
- ATC_INPUT_TC 0.15 ~ 0.25
- ATC_RAT_xxx_IMAX : maximum I-term. 0.4 is sufficient
- ATC_RAT_xxx_ILMI : I-term leak minimum. Less than 0.1
- H_OPTION : 1 for using Leaky-I, 0 for not using Leaky-I => should be determined later!! 
***
## GCS for Coaxial Rotor
- [https://github.com/Yisak2023/COAX_GCS/tree/master](https://github.com/Yisak2023/COAX_GCS/tree/master)
***
## Coaxial Rotor CCPM and Actuator
### Flow
@@ With FRAME_CONFIG => HELI_FRAME (/ArduCopter/config.h)   
1) SCHED_TASK(**rc_loop**, 250Hz, max 130micro sec, priority-3) @Copter.cpp   
- ==> Read RC input channels, -4500 ~ 4500 for roll, pitch, yaw in, 0 ~ 1000 for throttle  
- **To Do : not much to do in reading RC input, but should add mode_check algorithm to check Coax state**
- 1-1) [Copter::read_radio()] - read radio and set failsafe   
  - 1-1-1) [RC_Channels::read_input()] : check for a new input or initialize RC   
    - 1-1-1-1) for all 16 RC channels, do [channel(i)->update()] : set [radio_in] value from [read(ch_in)] or [override_value], check [type_in] as RANGE or ANGLE   
    - 1-1-1-2) For Range type_in, do [pwm_to_range()] => [RC_Channel::pwm_to_range_dz()] : int16 value 0 ~ high_in (high_in is set as 1000 from [Copter::init_rc_in()])   
    - 1-1-1-3) For Angle type_in, do [pwm_to_angle()] => [RC_Channel::pwm_to_angle_dz_trim()] : int16 value -4500 ~ +4500 centidegree   
  - 1-1-2) Set throttle failsafe, passthrough, throttle filter   
    - ==> Requires override or actual RC input   
    - ==> RC inputs are acquired through background thread with HAL   
- 1-2) [rc().read_mode_switch()] : reads flight mode switch   
  - 1-2-1) [RC_Channel_Copter::mode_switch_changed()] : chages flight mode class   
  ==> enum class Number for Mode is defined in mode.h (ex. STABILIZE = 0, ACRO = 1, ALT_HOLD = 2, AUTO = 3, GUIDED = 4, LOITER=5)
    - 1-2-1-1) [Copter::set_mode()] : check if new flight mode's requirements are met, clean previous flight mode   
    - 1-2-1-2) [flightmode = new_flightmode] Update the flightmode class pointer to new flight mode   
2) SCHED_TASK(**throttle_loop**, 50Hz, max 75micro sec, priority-6) @Copter.cpp  
- This is a supplemntory throttle update loop. There is a ground effect detection algorithm here, and a rangefinder can stabilize terrain height
- **To do : install rangefinder H/W, should not use Passthrough of pilot RC(We use custom motor with CAN 2.0b**
- In final stage, [AP_MotorsHeli_Dual::set_desired_rotor_speed] sets [AP_MotorHeli_RSC::_desired_speed]
- 2-1) [Copter::update_auto_armed()] : update status of auto_armed flag   
- 2-2) [Copter::heli_update_rotor_speed_targets()] : reads pilot input and passes new rotor speed targets to heli motors object    
  - 2-2-1) [motors->get_rsc_mode()] : get rotor speed control mode   
  - 2-2-2) Passthrough pilot input to rsc [motors->set_desired_rotor_speed(get_pilot_desired_rotor_speed())] of Heli_Dual   
  - 2-2-3) Check interlock [motors->get_interlock()] and [motors->set_desired_rotor_speed(motors->get_rsc_setpoint())] of Heli_Dual    
  - 2-2-4) Check runup [motors->rotor_runup_complete()] and set [rotor_runup_complete_last]   
- 2-3) [Copter::heli_update_landing_swash()]
  - 2-3-1) [Copter::should_use_landing_swash()] : check all conditions for throttle update using 'landing_swash'   
  ==>(1) landing (2) landed (3) not armed (4) not in dynamic_flight
  - 2-3-2) [motors->set_collective_for_landing()] : set [_heliflags.landing_collective] in AP_MotorsHeli class
  - 2-3-3) [Copter::update_collective_low_flag()] : set flag to force collective to zero for at least 400 millisecond
- 2-4) [Copter::update_ground_effect_detector()] : detect ground effect at takeoff and landing state
- 2-5) [Copter::update_ekf_terrain_height_stable()] : set EKF terrrain height stable setting to allow EKF to stabilize terratin height with rangefinder
3) **Stabilize flight control**
- @Copter.h, [mode_stabilize] class is created with [ModeStabilize_Heli] instead of ModeStabilize
- **[FAST_TASK(update_flight_mode)] : [Copter::update_flight_mode()] => [flightmode->run();]**
- All flight mode inherits class Mode @mode.h
- **Stablize mode class is in mode_stabilize_heli.cpp**
- **To Do : Allow idle state in step 3-5**
- 3-1) [Copter::update_simple_mode()] : rotate pilot input for simple mode   
  ==> channel_roll->set_control_in(rollx * ahrs.cos_yaw() + pitchx * ahrs.sin_yaw());   
  ==> channel_pitch->set_control_in(-rollx * ahrs.sin_yaw() + pitchx * ahrs.cos_yaw());
- 3-2) [Mode::get_pilot_desired_lean_angles()] : transform pilot's roll or pitch input into a desired lean angle
- 3-3) [Mode::get_pilot_desired_yaw_rate()] : transform pilot's yaw input into a desired yaw rate
- 3-4) [AC_InputManager_Heli::get_pilot_desired_collective)] : from throttle channel('channel_throttle's control_in' from 1-1-1-2), it rescale's pilot collective pitch input in Stabilize and Acro modes   
  ==> 'pilot_throttle_scaled' is then used in attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);   
- 3-5) Set desired spool state : (1)SHUT_DOWN if not armed, (2)THROTTLE_UNLIMITED if armed   
  ==> Problem here, is that there's no desired idle state
- 3-6) Control with respect to the spool state 
- 3-7) [AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_euler_rate_yaw()] : consider inverted flight and call normal attitude control
  - 3-7-1) If in inverted flight, [euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd + 18000);]
  - 3-7-2) [AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw()] 
    - 3-7-2-1) Compute _euler_rate_target.x, y, z from angle error and acceleration limit with respect to [_rate_bf_ff_enabled]
    - 3-7-2-2) [euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target)] : _ang_vel_target.x, y, z are computed from _euler_angle_target, _euler_rate_target, _ang_vel_target   
    ==>passing Vecto3f pointers
    - 3-7-2-3) From computed _ang_vel_target, apply anular velocity limit roll/pitch/yaw from paramters
    - 3-7-2-4) [ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target)] : Compute _euler_rate_target
  - 3-7-3) [AC_AttitudeControl::attitude_controller_run_quat()]
    - 3-7-3-1) [AC_AttitudeControl::update_ang_vel_target_from_att_error()] : Calculate attitude angular velocity target   
    ==> sets _ang_vel_body, which is acuatlly target rate values
  - 3-7-4) [AC_AttitudeControl_Heli::set_throttle_out()] : This function's main purpose is to apply angle boos in MultiCopter frames, but in heli, there's no angle boost
4) FAST_TASK(**update_heli_control_dynamics**) : pushes several important factors up into AP_MotorsHeli.
- 4-1) Check H_OPTION for for USE_LEAKY_I   
  ==> AP_GROUPINFO("OPTIONS", 28, AP_MotorsHeli, _heli_options, (uint8_t)HeliOption::USE_LEAKY_I)   
  - 4-1-1) **Attitude controller is set to use Leaky-I only at slow-speed, on ground**
  - 4-1-2) hover_roll_trim_scalar_slew is set   
    ==> It seems the Coaxial Rotor does not need roll trim. Only a single rotor heli with tail-rotor requires roll trim   
    ==> **To Do : Consider removing roll trim**
5) FAST_TASK(**run_rate_controller**) : **update rate controllers and output to roll, pitch and yaw actuators**
- 5-1) set attitude and position controller loop time : set dt
- 5-2) [AC_AttitudeControl_Heli::rate_controller_run()] : run lowest level rate controller and send outputs to the motors
  - 5-2-1) For passthrough, directly apply _motors.set_roll and _motors.set_pitch
  - 5-2-2) Normally, run roll/pitch rate PID controller [rate_bf_to_motor_roll_pitch(gyro_latest, _ang_vel_body.x, _ang_vel_body.y)]
    - 5-2-2-1) [AC_AttitudeControl_Heli::rate_bf_to_motor_roll_pitch] : **body-frame rate controller**   
    ==> **leaky Integrator should be considered here.** _flags_heli.leaky_i is true by default, and set from the paramter H_OPTIONS bit 0     
    ==> Using Leaky I : Prevents I term from building up by constatnly reducing I term at low speed, on ground, or hover   
    ==> Not using leaky : use landing and takeoff detection algorithm to zero the I terms when on ground   
    ==> **Output roll_out, pitch_out to motors, almost same as lateral and longitudinal cyclic, repectively.**   
  - 5-2-3) [rate_target_to_motor_yaw()] : runs yaw-rate controller   
    ==> **Output yaw_out : should be transformed to differential pitch for Coaxial Rotor**   
6) FAST_TASK(heli_update_autorotation) : determines if aircraft is in autorotation and sets motors flag and switches to autorotation flight mode if manual collective is not being used.   
   ==> *To Do : consider removing this part and autorotation control*   
7) **FAST_TASK(motors_output)** : [Copter::motors_output()] 
- FAST_TASK(motors_output) => Copter::motors_output()  =>  **flightmode->output_to_motors();  => motors->output();  => AP_MotorsHeli_Dual::output_to_motors()**
- To Do : Upgrade with PegasusSV, CoaxCAN motor
- 7-1) Update arming delay state : clears delay state here
- 7-2) [SRV_Channels::calc_pwm()] : This should calculate PWM from scaled output depending on type_angle(range or angle type)   
  ==> But it is recalculated for swash and motor   
  - 7-2-1) Check for slew, override
  - 7-2-2) For all 32 servo channels, run [channel[i].calc_pwm(output_scaled)]
    - 7-2-2-1) Check for emergency stop command
    - 7-2-2-2) If override is active, exit
    - 7-2-2-3) pwm_from_angle() for angle type (roll, pitch, ywa), pwm_from_range() for throttle \
- 7-3) [SRV_Channels::cork()] : delay rcout until push command is given
- 7-4) [SRV_Channels::output_ch_all()] : update output on any aux channels, for manual passthru
- 7-5) Update motors interlock state
- 7-6) [flightmode->output_to_motors()] : runs [motors->output()] for all flight modes, where motors is AP_MotorsHeli class
- 7-7) [AP_MotorsHeli::output()] : sends commands to the servos
  - 7-7-1) [AP_MotorsHeli::update_throttle_filter()] : this function applies filter to _throttle
  - 7-7-2) [AP_MotorsHeli::output_logic()] : run spool logic   
    ==> State transition : SHUT_DOWN → GROUND_IDLE → SPOOLING_UP → THROTTLE_UNLIMITED  OR THROTTLE_UNLIMITED → SPOOLING_DOWN → GROUND_IDLE → SHUT_DOWN   
  - 7-7-3) If armed : calculate_armed_scalars(), check interlock and output_armed_zero_throttle() or output_armed_stabilizing()
      - 7-7-3-1) [AP_MotorsHeli_Dual::calculate_armed_scalars()] : setup thrrotle curve, blocks RSC mode change during armed state, set autorotation flag   
      ==> **May add something related to CoaxCAN motor**   
      - 7-7-3-2) Check interlock (_interlock is true for motor run, false for motor stop)
      - 7-7-3-3) If _interlock is false, run output_armed_zero_throttle(), or if true, run output_armed_stabilizing()   
      ==> but, output_armed_zero_throttle() and output_armed_stabilizing() are same function to run move_actuators()   
      - 7-7-3-4) [AP_MotorsHeli_Dual::move_actuators()] : inputs are _roll_in, _pitch_in, filtered collective, _yaw_in   
      ==> **limits pitch_out, roll_out within _cyclic_max/4500, where _cyclic_max is 'H_CYC_MAX'**   
      ==> Limit collective to proper range and sets collective out   
      ==> Set takeoff_collective flag if collective is above 30%(may be 50%) between H_COL_MID and H_COL_MAX   
      ==> If _servo_mode is in auto mode, process pre-compensation factors : related to H_YAW_REV_EXPO   
      ==> **Calculate collective for upper and lower swash from yaw_out as differential pitch**   
      ==> **Calculate _servo_out for CH_1 ~ CH_6 from swashplate matrix**   
  - 7-7-4) If disarmed : run [AP_MotorsHeli::output_disarmed()] => supports test mode, but simillarlly call move_actuators() in the end
  - 7-7-5) [AP_MotorsHeli_Dual::output_to_motors()] : execute final pwm calculations
    - 7-7-5-1) [AP_MotorsHeli::rc_write_swash] : convert input in -1 to +1 range to pwm output for swashplate servo.   
      ==> Servo range is fixed to 1000, trim as 1500 : 1000 ~ 2000 range
    - 7-7-5-2) update_motor_control() with respect to spool state
- 7-8) [SRV_Channels::push()] : unlock cork and push to all channels
