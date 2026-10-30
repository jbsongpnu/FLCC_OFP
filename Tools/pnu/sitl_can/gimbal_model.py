#!/usr/bin/env python3
'''
PNU : a three-axis gimbal as a KINEMATIC model with compliant hard stops.

WHAT THIS MODEL IS, AND WHAT IT DELIBERATELY IS NOT.

The fake PMU carries a real engine model because an engine has physics the project knows:
a choke, a starter, an RPM curve, a generator.  A gimbal motor does not come with that.
The GLII manual gives no inertia, no friction, no thermal coefficients and no current
model, and PNUGimbal_CAN_Protocol_Type_CubeMars.md sec 9 keeps saying the same thing -
measure it, do not invent it.  So this model covers EXACTLY what the Appendix 1 homing
sequence needs in order to be exercised, and no more:

  - where each axis is, and how fast it is moving
  - where its mechanical stops are, and what happens when it is pushed into one
  - the encoder zero, and what the set-zero command does to it
  - which status code the motor is reporting

Temperatures, currents and fault conditions are NOT simulated.  They are set from the
console, because a fault that fires on invented physics teaches the wrong lesson.

THE MOTION LAW.  MIT mode's conceptual impedance law is

    tau = kp * (p_des - p) + kd * (v_des - v) + t_ff

(sec 4.3, itself labelled an interpretation).  A massless joint settles where tau is
zero, which gives the velocity this model follows:

    v_target = v_des + (kp / kd) * (p_des - p)          for kd > 0

That is KINEMATICS, not dynamics: there is no mass, so there is no overshoot, no settling
time and no torque ripple, and the fake must not be read as evidence about any of them.
It is enough for homing, whose endpoint detection is "the angle stopped changing while
motion was still commanded" - Appendix 1 item 4, angle only.

THE STOPS ARE COMPLIANT, and that is the point.  A real heavy SAR payload gives a little
when a motor leans on its stop, and the README already warns that stop compliance may
consume most of CAN_Dn_PNU_HTOL.  A rigid stop would make homing look easier in SITL than
it will be on the bench and would leave PNU_HRTOL untested.  The deflection is

    deflection = clamp(|tau|, 0, t_max) / stop_stiffness

and stop_stiffness is a KNOB FOR TESTING, not a measurement - nobody has measured this
mount.  Lower it until homing fails, which is the useful experiment.

AP_FLAKE8_CLEAN
'''

import math

import gimbal_frames as icd

AXIS_NAMES = ('roll', 'pitch', 'yaw')

# Appendix 1 item 3, and also the CAN_Dn_PNU_MRLL/MPIT/MYAW defaults: the MECHANICAL
# limits, as a symmetric half-range in degrees.  Appendix 1 item 7's operating limits
# (25/25/15) are the driver's business, not the motor's, and are not modelled here.
DEFAULT_TRAVEL_DEG = (30.0, 30.0, 20.0)

# N.m/rad.  See the header: a knob, not a measurement.  At the homing defaults
# (HKD 0.5, HVEL 10 deg/s) the stall torque is about 0.087 N.m, so this gives roughly a
# quarter of a degree of deflection - visible, and half of PNU_HRTOL's default.
DEFAULT_STOP_STIFFNESS = 20.0


class Axis(object):
    def __init__(self, name, motor_id, travel_deg, limits, stop_stiffness):
        self.name = name
        self.motor_id = motor_id
        self.limits = limits
        self.stop_stiffness = stop_stiffness

        if isinstance(travel_deg, tuple):
            low_deg, high_deg = travel_deg
        else:
            low_deg, high_deg = -travel_deg, travel_deg
        self.stop_low = math.radians(low_deg)
        self.stop_high = math.radians(high_deg)

        # true mechanical angle, zero at the mechanical centre of the modelled travel
        self.pos_rad = 0.0
        # the encoder's zero reference.  Reported position is pos_rad - zero_rad, which
        # is why a motor that has never been homed reports an angle with no physical
        # meaning - the whole reason Appendix 1 exists.
        self.zero_rad = 0.0
        self.vel_rads = 0.0
        self.torque_nm = 0.0

        # sec 1 : "Motor is enabled at power-up per manual."
        self.enabled = True
        self.status = icd.STATUS_ENABLED
        self.driver_temp_c = 35
        self.motor_temp_c = 30

        self.cmd = None             # last decoded command, for reporting
        self.cmd_count = 0
        self.fb_count = 0
        self.at_stop = None         # 'low', 'high' or None

    # ---- reporting ----------------------------------------------------------------
    def reported_rad(self):
        return self.pos_rad - self.zero_rad

    def set_zero(self):
        '''sec 3 : set current position to zero.  Persistence is unspecified (sec 9.7).'''
        self.zero_rad = self.pos_rad

    def set_status(self, status):
        self.status = status
        # Status 0 and 1 are normal states; anything else is a fault and the axis stops.
        # Appendix 1 item 6 aborts initialisation on any status that is not 1.
        if status not in (icd.STATUS_DISABLED, icd.STATUS_ENABLED):
            self.vel_rads = 0.0

    def enable(self, on):
        self.enabled = on
        if self.status in (icd.STATUS_DISABLED, icd.STATUS_ENABLED):
            self.status = icd.STATUS_ENABLED if on else icd.STATUS_DISABLED
        if not on:
            self.vel_rads = 0.0

    # ---- motion -------------------------------------------------------------------
    def target_velocity(self):
        '''(v_target, tau_stall) - see the header

        tau_stall is the impedance torque the axis would feel IF IT WERE HELD STILL, so
        it is evaluated at v = 0.  That is the physically meaningful quantity at a hard
        stop, and it is what the driver's own homing comment derives:

            tau_stall = HKD * HVEL

        It must NOT be computed from the achieved velocity.  Doing that makes the stop's
        deflection depend on the motion the deflection itself produces, and the axis
        chatters in and out of the stop forever instead of settling - which is exactly
        what the first version of this model did, and what the stop tests caught.
        '''
        if self.cmd is None:
            return 0.0, 0.0

        kind = self.cmd[0]
        if kind == 'mit':
            _, p_des, v_des, kp, kd, t_ff = self.cmd
            err = p_des - self.reported_rad()
            tau = kp * err + kd * v_des + t_ff
            if kd > 0.0:
                v_target = v_des + (kp / kd) * err
            elif kp > 0.0:
                # sec 4.3 warns against kd = 0 for position control.  Modelled as a move
                # toward the target at full speed, which is roughly what an undamped
                # position loop does, and flagged rather than silently smoothed.
                v_target = math.copysign(self.limits.v_max, err) if err else 0.0
            else:
                # pure torque command: a massless joint has no equilibrium velocity, so
                # there is nothing honest to compute.  The axis holds.
                v_target = 0.0
            return v_target, tau

        if kind == 'posvel':
            _, p_des, v_limit = self.cmd
            err = p_des - self.reported_rad()
            v_limit = abs(v_limit)
            v_target = math.copysign(min(abs(err) * 10.0, v_limit), err) if err else 0.0
            # The cascaded loop's torque is not host-visible at all (sec 5), so there is
            # no tau to report beyond what the stop needs.  t_max is the honest bound.
            return v_target, self.limits.t_max

        if kind == 'vel':
            return self.cmd[1], self.limits.t_max

        return 0.0, 0.0

    def step(self, dt):
        if not self.enabled or self.status not in (icd.STATUS_DISABLED, icd.STATUS_ENABLED):
            self.vel_rads = 0.0
            self.torque_nm = 0.0
            return
        if self.status == icd.STATUS_DISABLED:
            self.vel_rads = 0.0
            self.torque_nm = 0.0
            return

        v_target, tau_stall = self.target_velocity()
        v_target = min(max(v_target, self.limits.v_min), self.limits.v_max)

        new_pos = self.pos_rad + v_target * dt

        # the compliant stop: the axis may pass the mechanical limit by the deflection
        # the STALL torque produces against the stop's stiffness, and no further
        deflection = min(abs(tau_stall), self.limits.t_max) / self.stop_stiffness
        high = self.stop_high + deflection
        low = self.stop_low - deflection

        self.at_stop = None
        if new_pos >= high:
            new_pos = high
            self.at_stop = 'high'
        elif new_pos <= low:
            new_pos = low
            self.at_stop = 'low'

        # The reported velocity is the one actually achieved, so an axis leaning on a
        # stop reports zero - which is what the driver's angle-only stall detector sees.
        self.vel_rads = (new_pos - self.pos_rad) / dt if dt > 0.0 else 0.0
        self.pos_rad = new_pos

        # ...and the reported torque follows from that achieved velocity, so it is near
        # zero while the axis runs free and settles at tau_stall once it is held.
        self.torque_nm = min(max(self.impedance_torque(), self.limits.t_min),
                             self.limits.t_max)

    def impedance_torque(self):
        if self.cmd is None or self.cmd[0] != 'mit':
            # sec 5 and sec 6 : the cascaded loops' torque is not host-visible at all,
            # so there is nothing honest to report beyond the configured bound
            return self.limits.t_max if self.cmd is not None else 0.0
        _, p_des, v_des, kp, kd, t_ff = self.cmd
        return kp * (p_des - self.reported_rad()) + kd * (v_des - self.vel_rads) + t_ff

    # ---- the wire -----------------------------------------------------------------
    def feedback(self):
        self.fb_count += 1
        return icd.build_feedback(self.motor_id, self.status, self.reported_rad(),
                                  self.vel_rads, self.torque_nm,
                                  self.driver_temp_c, self.motor_temp_c, self.limits)

    def describe(self):
        return ('%-5s id %u %-10s mech %+7.2f deg  reported %+7.2f deg  vel %+7.2f deg/s'
                '  tau %+6.3f Nm  stops %+.1f/%+.1f%s  cmds %u  fb %u'
                % (self.name, self.motor_id, icd.status_name(self.status),
                   math.degrees(self.pos_rad), math.degrees(self.reported_rad()),
                   math.degrees(self.vel_rads), self.torque_nm,
                   math.degrees(self.stop_low), math.degrees(self.stop_high),
                   '' if self.at_stop is None else '  AT %s STOP' % self.at_stop.upper(),
                   self.cmd_count, self.fb_count))


class GimbalModel(object):
    def __init__(self, motor_ids=(1, 2, 3), travel_deg=DEFAULT_TRAVEL_DEG,
                 limits=None, stop_stiffness=DEFAULT_STOP_STIFFNESS):
        self.limits = limits or icd.Limits()
        self.axes = []
        for i, name in enumerate(AXIS_NAMES):
            self.axes.append(Axis(name, motor_ids[i], travel_deg[i],
                                  self.limits, stop_stiffness))
        self.by_motor_id = {axis.motor_id: axis for axis in self.axes}

    def axis_by_name(self, name):
        for axis in self.axes:
            if axis.name.startswith(name.lower()):
                return axis
        return None

    def step(self, dt):
        for axis in self.axes:
            axis.step(dt)
