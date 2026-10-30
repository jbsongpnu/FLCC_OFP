#!/usr/bin/env python3
'''
PNU : tests for the fake gimbal's CAN framing and its motion model, without a vcan
interface.

Creating a vcan interface needs root, so these drive FakeGimbal.handle_frame() with
hand-built SocketCAN frames and capture what it would have transmitted.  What they
cannot cover is the socket itself - binding, the kernel filter and sharing the wire with
the fake PMU and the fake SAR - which is only provable by running against a real vcan;
see README.md in this directory.

Two of these read files rather than asserting constants: the DLCs and the universal
command bytes are checked against AP_PNUGimbal_CubeMars.h, and the published command
vectors against PNUGimbal_CAN_Protocol_Type_CubeMars.md's own section 8 table.  A fake
that agrees only with itself proves nothing.

Run with pytest, or directly:  Tools/pnu/sitl_can/test_fake_gimbal.py

AP_FLAKE8_CLEAN
'''

import math
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gimbal_frames as icd                                                 # noqa: E402
from fake_gimbal import FakeGimbal, CAN_FRAME_FMT, CAN_RTR_FLAG             # noqa: E402
from gimbal_model import GimbalModel                                        # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         '..', '..', '..'))
CUBEMARS_HEADER = os.path.join(REPO_ROOT, 'libraries', 'AP_PNUGimbal',
                               'AP_PNUGimbal_CubeMars.h')
PROTOCOL_DOC = os.path.join(REPO_ROOT, 'libraries',
                            'PNUGimbal_CAN_Protocol_Type_CubeMars.md')

LIMITS = icd.Limits()


def make_gimbal(**kwargs):
    '''a FakeGimbal with no socket, recording what it would have transmitted'''
    gimbal = FakeGimbal('test0', **kwargs)
    gimbal.sent = []
    gimbal.send = lambda can_id, payload: gimbal.sent.append((can_id, bytes(payload)))
    gimbal.log = lambda msg: None
    return gimbal


def frame(can_id, data, dlc=None):
    data = bytes(data)
    return struct.pack(CAN_FRAME_FMT, can_id, dlc if dlc is not None else len(data),
                       data.ljust(8, b'\x00'))


def near(got, want, tol=1e-3):
    return abs(got - want) <= tol


# ---------------------------------------------------------------------------
# the wire format - sec 2, 3, 4, 5, 6, 7, 8
# ---------------------------------------------------------------------------

def test_the_arbitration_id_carries_the_mode():
    # sec 2 : command_id = (mode << 8) | motor_id, and the manual's own worked row
    assert icd.command_id(icd.MODE_MIT, 1) == 0x001
    assert icd.command_id(icd.MODE_POSVEL, 1) == 0x101
    assert icd.command_id(icd.MODE_VEL, 1) == 0x201
    assert icd.command_id(icd.MODE_POSVEL, 15) == 0x10F
    assert icd.split_command_id(0x203) == (icd.MODE_VEL, 3)


def test_the_published_float_vectors_decode():
    '''sec 8 : the position/velocity and velocity rows, which the doc does not dispute'''
    assert icd.decode_posvel(bytes.fromhex('000040400000803F')) == (3.0, 1.0)
    assert icd.decode_posvel(bytes.fromhex('000040C00000803F')) == (-3.0, 1.0)
    assert icd.decode_vel(bytes.fromhex('00000040')) == 2.0
    assert icd.decode_vel(bytes.fromhex('000000C0')) == -2.0


def test_the_published_mit_position_and_torque_vectors_decode():
    '''sec 8 : p = +/-2 with kp 0.123 kd 0.005, and t = +/-0.03

    These four rows are consistent with the proposed (1 << bits) - 1 codec.  The speed
    rows in the same table are NOT, and the test below says so rather than quietly
    skipping them.
    '''
    p, v, kp, kd, t = icd.decode_mit(bytes.fromhex('947A7FF0010047FF'), LIMITS)
    assert near(p, 2.0, 1e-3), p
    assert near(kp, 0.123, 1e-2), kp
    assert near(kd, 0.005, 1e-3), kd
    assert near(v, 0.0, 0.1), v
    assert near(t, 0.0, 0.01), t

    p, _, _, _, _ = icd.decode_mit(bytes.fromhex('6B847FF0010047FF'), LIMITS)
    assert near(p, -2.0, 1e-3), p

    _, _, _, _, t = icd.decode_mit(bytes.fromhex('7FFF7FF00000 0805'.replace(' ', '')), LIMITS)
    assert near(t, 0.03, 1e-2), t
    _, _, _, _, t = icd.decode_mit(bytes.fromhex('7FFF7FF0000007F9'), LIMITS)
    assert near(t, -0.03, 1e-2), t


def test_the_published_speed_vectors_are_the_documented_source_error():
    '''sec 8 : "do not use its speed rows as passing codec tests"

    The published v = +/-6 payloads encode 0x830 / 0x7CE, which decode to about +4.74 /
    -4.84 at the stated +/-200 range.  This test PINS that discrepancy rather than
    hiding it: if a future session "fixes" the codec until these rows give +/-6, it will
    have broken agreement with the receive equation the rest of the manual uses.
    '''
    _, v, _, _, _ = icd.decode_mit(bytes.fromhex('7FFF83000000 47FF'.replace(' ', '')), LIMITS)
    assert near(v, 4.737, 0.01), v
    _, v, _, _, _ = icd.decode_mit(bytes.fromhex('7FFF7CE0000047FF'), LIMITS)
    assert near(v, -4.835, 0.01), v

    # and the corrected payloads the doc supplies for the same intent do give +/-6,
    # to within one quantum: 12 bits over a 400 rad/s span is 0.098 rad/s per LSB, so
    # 5.910 and -6.007 are the nearest representable values and not an error
    lsb = (LIMITS.v_max - LIMITS.v_min) / ((1 << 12) - 1)
    _, v, _, _, _ = icd.decode_mit(bytes.fromhex('7FFF83C0000047FF'), LIMITS)
    assert near(v, 6.0, lsb), (v, lsb)
    _, v, _, _, _ = icd.decode_mit(bytes.fromhex('7FFF7C20000047FF'), LIMITS)
    assert near(v, -6.0, lsb), (v, lsb)


def test_mit_round_trips_through_the_codec():
    for value in (0.0, 1.5, -1.5, 12.5, -12.5):
        payload = icd.encode_mit(value, 0.0, 0.0, 0.5, 0.0, LIMITS)
        p, _, _, kd, _ = icd.decode_mit(payload, LIMITS)
        assert near(p, value, 1e-3), (value, p)
        assert near(kd, 0.5, 1e-2), kd
    # sec 4.1 : packed zero is near the MIDDLE of the range, so an all-zero MIT payload
    # is not a zero command - it is the minimum of every field.
    p, v, kp, kd, t = icd.decode_mit(bytes(8), LIMITS)
    assert near(p, LIMITS.p_min), p
    assert near(v, LIMITS.v_min), v
    assert near(t, LIMITS.t_min), t


def test_the_universal_commands_are_recognised():
    for action in (icd.UNIVERSAL_ENABLE, icd.UNIVERSAL_DISABLE,
                   icd.UNIVERSAL_SET_ZERO, icd.UNIVERSAL_CLEAR_ERRORS):
        assert icd.universal_command(icd.build_universal(action)) == action
    # an undocumented trailing byte is NOT invented into an action
    assert icd.universal_command(icd.UNIVERSAL_PREFIX + b'\x00') is None
    # and neither is a short frame that happens to start with 0xFF
    assert icd.universal_command(b'\xFF' * 4) is None


def test_feedback_packs_status_and_identity_into_d0():
    '''sec 7 : status in the high nibble, the motor id's low four bits in the low nibble'''
    data = icd.build_feedback(3, 0xA, 1.0, -2.0, 0.5, -5, 40, LIMITS)
    assert len(data) == icd.FEEDBACK_DLC
    assert data[0] == 0xA3
    out = icd.decode_feedback(data, LIMITS)
    assert out['status'] == 0xA
    assert out['motor_id_low4'] == 3
    assert near(out['position_rad'], 1.0, 1e-3)
    assert near(out['velocity_rads'], -2.0, 0.2)
    assert near(out['torque_nm'], 0.5, 0.02)
    # sec 7 : both temperatures are SIGNED int8
    assert out['driver_temp_c'] == -5
    assert out['motor_temp_c'] == 40


# ---------------------------------------------------------------------------
# the fake's command handling
# ---------------------------------------------------------------------------

def test_a_mit_command_moves_the_axis_and_answers_once():
    gimbal = make_gimbal()
    # velocity-style MIT, exactly as the homing sweep sends it: kp 0, kd > 0
    payload = icd.encode_mit(0.0, math.radians(10.0), 0.0, 0.5, 0.0, LIMITS)
    gimbal.handle_frame(frame(icd.command_id(icd.MODE_MIT, 1), payload))
    assert len(gimbal.sent) == 1, 'per-command feedback must answer exactly once'
    assert gimbal.sent[0][0] == gimbal.master_id, 'feedback goes to the Master ID'

    roll = gimbal.model.axes[0]
    gimbal.model.step(0.5)
    assert roll.vel_rads > 0.0
    assert roll.pos_rad > 0.0


def test_a_command_in_the_wrong_mode_is_refused():
    '''sec 1 : the command's mode bits must match the configured mode'''
    gimbal = make_gimbal(mode=icd.MODE_MIT)
    gimbal.handle_frame(frame(icd.command_id(icd.MODE_POSVEL, 1), bytes(8)))
    assert gimbal._bad_mode_seen == 1
    assert gimbal.sent == [], 'a refused command must not be acknowledged'
    assert gimbal.model.axes[0].cmd is None


def test_a_velocity_command_with_the_wrong_dlc_is_refused():
    '''sec 6 : velocity mode is DLC 4, deliberately not 8'''
    gimbal = make_gimbal(mode=icd.MODE_VEL)
    gimbal.handle_frame(frame(icd.command_id(icd.MODE_VEL, 1), bytes(8), dlc=8))
    assert gimbal.model.axes[0].cmd is None
    assert gimbal.sent == []
    gimbal.handle_frame(frame(icd.command_id(icd.MODE_VEL, 1),
                              struct.pack('<f', 1.0), dlc=4))
    assert gimbal.model.axes[0].cmd == ('vel', 1.0)


def test_an_unknown_motor_id_is_counted_not_answered():
    gimbal = make_gimbal()
    gimbal.handle_frame(frame(icd.command_id(icd.MODE_MIT, 9), bytes(8)))
    assert gimbal._unknown_id_seen == 1
    assert gimbal.sent == []


def test_remote_frames_are_rejected():
    '''the PMU's RTR polls are extended and never arrive, so an RTR here is a fault'''
    gimbal = make_gimbal()
    gimbal.handle_frame(frame(icd.command_id(icd.MODE_MIT, 1) | CAN_RTR_FLAG, bytes(8)))
    assert gimbal.sent == []
    assert gimbal.model.axes[0].cmd is None


def test_feedback_modes():
    off = make_gimbal(fb_mode='off')
    payload = icd.encode_mit(0.0, 0.0, 0.0, 0.5, 0.0, LIMITS)
    off.handle_frame(frame(icd.command_id(icd.MODE_MIT, 1), payload))
    assert off.sent == [], '--fb-mode off must transmit nothing at all'

    periodic = make_gimbal(fb_mode='periodic', fb_rate=100.0)
    periodic.handle_frame(frame(icd.command_id(icd.MODE_MIT, 1), payload))
    assert periodic.sent == [], 'periodic mode must not answer the command'
    periodic._next_fb = 0.0
    periodic.tick(1.0)
    assert len(periodic.sent) == 3, 'periodic mode sends one frame per axis'


def test_the_universal_commands_reach_the_model():
    gimbal = make_gimbal()
    roll = gimbal.model.axes[0]
    can_id = icd.command_id(icd.MODE_MIT, 1)

    gimbal.handle_frame(frame(can_id, icd.build_universal(icd.UNIVERSAL_DISABLE)))
    assert roll.enabled is False and roll.status == icd.STATUS_DISABLED

    gimbal.handle_frame(frame(can_id, icd.build_universal(icd.UNIVERSAL_ENABLE)))
    assert roll.enabled is True and roll.status == icd.STATUS_ENABLED

    roll.pos_rad = math.radians(12.0)
    gimbal.handle_frame(frame(can_id, icd.build_universal(icd.UNIVERSAL_SET_ZERO)))
    assert near(roll.reported_rad(), 0.0, 1e-9), 'set zero must rebase the report'
    assert near(math.degrees(roll.pos_rad), 12.0, 1e-9), 'and must not move the axis'


# ---------------------------------------------------------------------------
# the motion model
# ---------------------------------------------------------------------------

def sweep(axis, vel_degs, seconds, dt=0.01):
    '''drive one axis with a velocity-style MIT command, as homing does'''
    axis.cmd = ('mit', 0.0, math.radians(vel_degs), 0.0, 0.5, 0.0)
    steps = int(seconds / dt)
    for _ in range(steps):
        axis.step(dt)


def test_an_axis_stops_at_its_mechanical_stop():
    model = GimbalModel()
    roll = model.axes[0]                                    # stops at +/-30 deg
    sweep(roll, 20.0, 5.0)
    assert roll.at_stop == 'high'
    # it rests just BEYOND the stop, by the compliant deflection - never short of it
    assert math.degrees(roll.pos_rad) >= 30.0
    assert math.degrees(roll.pos_rad) < 31.0
    assert near(roll.vel_rads, 0.0, 1e-6), 'a stalled axis must report zero velocity'


def test_the_stop_is_compliant_and_the_stiffness_sets_how_much():
    stiff = GimbalModel(stop_stiffness=1000.0).axes[0]
    soft = GimbalModel(stop_stiffness=5.0).axes[0]
    sweep(stiff, 20.0, 5.0)
    sweep(soft, 20.0, 5.0)
    stiff_deflection = math.degrees(stiff.pos_rad) - 30.0
    soft_deflection = math.degrees(soft.pos_rad) - 30.0
    assert soft_deflection > stiff_deflection
    # the soft case must be big enough to matter against PNU_HRTOL's 0.5 deg default,
    # which is the whole reason compliance is modelled at all
    assert soft_deflection > 0.5, soft_deflection


def test_both_stops_and_the_travel_between_them():
    model = GimbalModel()
    yaw = model.axes[2]                                     # stops at +/-20 deg
    sweep(yaw, 30.0, 5.0)
    high = math.degrees(yaw.pos_rad)
    sweep(yaw, -30.0, 5.0)
    low = math.degrees(yaw.pos_rad)
    assert yaw.at_stop == 'low'
    # Compliance INFLATES the measured travel, by one deflection at each end.  That is
    # the effect PNU_HTOL has to absorb, and the README already warns it may consume
    # most of it - so the expected figure is DERIVED from the stall torque rather than
    # written down, and a change to the model has to change the physics to pass.
    tau_stall = 0.5 * math.radians(30.0)                    # kd * v_des, see sweep()
    deflection_deg = math.degrees(tau_stall / yaw.stop_stiffness)
    assert near(high - low, 40.0 + 2.0 * deflection_deg, 0.05), (high - low, deflection_deg)
    assert near((high + low) / 2.0, 0.0, 0.1), 'and its midpoint should be centred'


def test_an_asymmetric_mount_moves_the_midpoint():
    '''the case homing exists to find: the stops are not where the encoder thinks'''
    model = GimbalModel(travel_deg=((-10.0, 50.0), 30.0, 20.0))
    roll = model.axes[0]
    sweep(roll, 30.0, 10.0)
    high = math.degrees(roll.pos_rad)
    sweep(roll, -30.0, 10.0)
    low = math.degrees(roll.pos_rad)
    assert near((high + low) / 2.0, 20.0, 0.5), 'the true midpoint is +20 deg'


def test_a_fault_status_stops_the_axis():
    '''Appendix 1 item 6 : homing aborts on any status that is not 1'''
    model = GimbalModel()
    roll = model.axes[0]
    roll.set_status(0xA)                                    # overcurrent
    sweep(roll, 20.0, 1.0)
    assert near(math.degrees(roll.pos_rad), 0.0, 1e-9), 'a faulted axis must not move'
    assert near(roll.vel_rads, 0.0)


def test_a_disabled_axis_does_not_move():
    model = GimbalModel()
    roll = model.axes[0]
    roll.enable(False)
    sweep(roll, 20.0, 1.0)
    assert near(math.degrees(roll.pos_rad), 0.0, 1e-9)


def test_the_encoder_zero_is_meaningless_until_it_is_set():
    '''why Appendix 1 exists: a reported angle has no physical meaning before homing'''
    model = GimbalModel()
    pitch = model.axes[1]
    pitch.zero_rad = math.radians(17.0)                     # powered up somewhere random
    pitch.pos_rad = 0.0
    assert near(math.degrees(pitch.reported_rad()), -17.0, 1e-9)
    pitch.set_zero()
    assert near(pitch.reported_rad(), 0.0, 1e-9)


# ---------------------------------------------------------------------------
# agreement with the firmware and with the protocol document
# ---------------------------------------------------------------------------

def test_the_dlcs_and_universal_bytes_match_the_firmware_header():
    source = open(CUBEMARS_HEADER, encoding='utf-8').read()
    defines = dict(re.findall(r'#define\s+(AP_PNUGIMBAL_CUBEMARS_\w+)\s+\((\d+)U?\)', source))
    expected = {
        'AP_PNUGIMBAL_CUBEMARS_FB_DLC': icd.FEEDBACK_DLC,
        'AP_PNUGIMBAL_CUBEMARS_MIT_DLC': icd.MIT_DLC,
        'AP_PNUGIMBAL_CUBEMARS_POSVEL_DLC': icd.POSVEL_DLC,
        'AP_PNUGIMBAL_CUBEMARS_VEL_DLC': icd.VEL_DLC,
        'AP_PNUGIMBAL_CUBEMARS_UNIVERSAL_DLC': icd.UNIVERSAL_DLC,
    }
    for name, value in expected.items():
        assert name in defines, '%s is gone from AP_PNUGimbal_CubeMars.h' % name
        assert int(defines[name], 0) == value, \
            '%s is %s, the fake says %u' % (name, defines[name], value)

    universals = dict(re.findall(r'UNIVERSAL_(\w+)\s*=\s*(0x[0-9A-Fa-f]+)', source))
    assert int(universals['ENABLE'], 16) == icd.UNIVERSAL_ENABLE
    assert int(universals['DISABLE'], 16) == icd.UNIVERSAL_DISABLE
    assert int(universals['ZERO'], 16) == icd.UNIVERSAL_SET_ZERO
    assert int(universals['CLEAR'], 16) == icd.UNIVERSAL_CLEAR_ERRORS


def test_the_mechanical_limits_match_appendix_1():
    '''Appendix 1 item 3 : roll -30..+30, tilt -30..+30, pan -20..+20'''
    doc = open(PROTOCOL_DOC, encoding='utf-8').read()
    line = [ln for ln in doc.splitlines() if 'Mechanical limits' in ln]
    assert line, 'Appendix 1 item 3 is gone from the protocol document'
    numbers = [abs(int(n)) for n in re.findall(r'-?\d+(?= degrees|\b)', line[0])]
    for want in (30, 30, 20):
        assert want in numbers, 'Appendix 1 no longer lists %u degrees' % want

    model = GimbalModel()
    for axis, want in zip(model.axes, (30.0, 30.0, 20.0)):
        assert near(math.degrees(axis.stop_high), want, 1e-9), axis.name
        assert near(math.degrees(axis.stop_low), -want, 1e-9), axis.name


if __name__ == '__main__':
    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_') or not callable(fn):
            continue
        try:
            fn()
            print('PASS  %s' % name)
        except AssertionError as err:
            failures += 1
            print('FAIL  %s  %s' % (name, err))
    print('%s' % ('all passed' if failures == 0 else '%u FAILED' % failures))
    sys.exit(1 if failures else 0)
