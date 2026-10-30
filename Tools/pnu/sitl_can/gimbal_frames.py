#!/usr/bin/env python3
'''
PNU : the CubeMars GLII CAN wire format, as a gimbal motor speaks it.

Wire format only - no sockets, no state, no motion.  Every id, bit layout, DLC and
encoding here comes from libraries/PNUGimbal_CAN_Protocol_Type_CubeMars.md, which is
itself a reading of gl-ii-gimbal-motor-drive-user-manual.pdf V1.0.0, and is cross-checked
against AP_PNUGimbal_CubeMars.h by test_fake_gimbal.py.

This file is the MOTOR's side of that protocol: it DECODES what the FC sends and ENCODES
what a motor replies.  AP_PNUGimbal_CubeMars.cpp does the mirror image, and the two were
written from the same document rather than from each other - a fake that agrees only with
the driver would prove nothing about either.

Four facts this file is the only place to get right:

  - THE ARBITRATION ID CARRIES THE MODE (sec 2).  command_id = (mode << 8) | motor_id,
    so motor 1 is 0x001 in MIT, 0x101 in position/velocity and 0x201 in velocity.  The
    mode bits must match what the motor was configured for with the PC utility; the
    manual documents no runtime mode switch.
  - FEEDBACK COMES BACK ON THE MASTER ID (sec 7), not on the command id and not on the
    motor id.  All three motors answer on the same id and are told apart by the low
    nibble of D0 - which is also where the status code lives, in the high nibble.
  - MIT FIELDS ARE UNSIGNED OFFSET INTEGERS, bit-packed (sec 4).  They are not two's
    complement and not float32.  Packed zero sits near the middle of the range, so an
    all-zero MIT payload is NOT a zero command.
  - POSITION/VELOCITY AND VELOCITY COMMANDS ARE IEEE-754 float32, LITTLE-ENDIAN (sec 5,
    sec 6), and the velocity command is DLC 4, not 8.

AP_FLAKE8_CLEAN
'''

import struct

# sec 2 : command arbitration id = (mode << 8) | motor_id
MODE_MIT = 0
MODE_POSVEL = 1
MODE_VEL = 2

MODE_NAMES = {MODE_MIT: 'MIT', MODE_POSVEL: 'position/velocity', MODE_VEL: 'velocity'}

# sec 1 : DLCs.  Velocity is deliberately 4; the others are 8.
MIT_DLC = 8
POSVEL_DLC = 8
VEL_DLC = 4
UNIVERSAL_DLC = 8
FEEDBACK_DLC = 8

# sec 3 : the four universal commands, all DLC 8, all seven leading 0xFF bytes
UNIVERSAL_PREFIX = b'\xFF' * 7
UNIVERSAL_ENABLE = 0xFC
UNIVERSAL_DISABLE = 0xFD
UNIVERSAL_SET_ZERO = 0xFE
UNIVERSAL_CLEAR_ERRORS = 0xFB

UNIVERSAL_NAMES = {
    UNIVERSAL_ENABLE: 'enable',
    UNIVERSAL_DISABLE: 'disable',
    UNIVERSAL_SET_ZERO: 'set zero',
    UNIVERSAL_CLEAR_ERRORS: 'clear errors',
}

# sec 7 : status codes.  0 and 1 are normal operating states, NOT faults.  Appendix 1
# item 6 aborts initialisation on anything that is not 1, and item 4 is explicit that the
# fault codes are emergency stops rather than endpoint detection criteria.
STATUS_DISABLED = 0x0
STATUS_ENABLED = 0x1
STATUS_NAMES = {
    0x0: 'Disabled',
    0x1: 'Enabled',
    0x8: 'Overvoltage',
    0x9: 'Undervoltage',
    0xA: 'Overcurrent',
    0xB: 'MOS overtemperature',
    0xC: 'Motor winding overtemperature',
    0xD: 'Communication loss',
    0xE: 'Overload',
}


class Limits(object):
    '''the PC-utility encoding ranges, which the host MUST match (sec 4)

    These are the manual's presets and also AP_PNU_CAN's CAN_Dn_PNU_PMAX/VMAX/KPMAX/
    KDMAX/TMAX defaults.  A host/device mismatch does not fail: it silently rescales
    every command and every reading, which is why the fake takes them as arguments
    instead of hard-coding them.
    '''

    def __init__(self, p_max=12.5, v_max=200.0, kp_max=500.0, kd_max=5.0, t_max=10.0):
        self.p_min, self.p_max = -p_max, p_max
        self.v_min, self.v_max = -v_max, v_max
        self.kp_min, self.kp_max = 0.0, kp_max
        self.kd_min, self.kd_max = 0.0, kd_max
        self.t_min, self.t_max = -t_max, t_max


def command_id(mode, motor_id):
    '''sec 2 : bits 10..8 are the mode, bits 7..0 the configured motor CAN id'''
    return ((mode & 0x07) << 8) | (motor_id & 0xFF)


def split_command_id(can_id):
    '''(mode, motor_id) for a command arbitration id'''
    return (can_id >> 8) & 0x07, can_id & 0xFF


# sec 4.1 : the proposed codec, (1 << bits) - 1.  The manual's TRANSMIT function uses
# (1 << bits), which overflows the field at the maximum; its RECEIVE function uses this
# one.  sec 9 item 2 records the discrepancy and says to verify on firmware - so the fake
# uses the receive form, the same choice AP_PNUGimbal_CubeMars made, and the tests say so.
def encode_unsigned(value, vmin, vmax, bits):
    span = vmax - vmin
    q = (1 << bits) - 1
    if span <= 0.0:
        return 0
    clamped = min(max(value, vmin), vmax)
    return int((clamped - vmin) * q / span)


def decode_unsigned(raw, vmin, vmax, bits):
    q = (1 << bits) - 1
    return vmin + (raw * (vmax - vmin) / float(q))


def decode_mit(data, limits):
    '''sec 4.2 : unpack one MIT command into (p_des, v_des, kp, kd, t_ff)'''
    p = (data[0] << 8) | data[1]
    v = (data[2] << 4) | (data[3] >> 4)
    kp = ((data[3] & 0x0F) << 8) | data[4]
    kd = (data[5] << 4) | (data[6] >> 4)
    t = ((data[6] & 0x0F) << 8) | data[7]
    return (decode_unsigned(p, limits.p_min, limits.p_max, 16),
            decode_unsigned(v, limits.v_min, limits.v_max, 12),
            decode_unsigned(kp, limits.kp_min, limits.kp_max, 12),
            decode_unsigned(kd, limits.kd_min, limits.kd_max, 12),
            decode_unsigned(t, limits.t_min, limits.t_max, 12))


def encode_mit(p_des, v_des, kp, kd, t_ff, limits):
    '''the inverse, for the tests and for anything that wants to drive the fake'''
    p = encode_unsigned(p_des, limits.p_min, limits.p_max, 16)
    v = encode_unsigned(v_des, limits.v_min, limits.v_max, 12)
    k_p = encode_unsigned(kp, limits.kp_min, limits.kp_max, 12)
    k_d = encode_unsigned(kd, limits.kd_min, limits.kd_max, 12)
    t = encode_unsigned(t_ff, limits.t_min, limits.t_max, 12)
    return bytes([(p >> 8) & 0xFF,
                  p & 0xFF,
                  (v >> 4) & 0xFF,
                  ((v & 0x0F) << 4) | ((k_p >> 8) & 0x0F),
                  k_p & 0xFF,
                  (k_d >> 4) & 0xFF,
                  ((k_d & 0x0F) << 4) | ((t >> 8) & 0x0F),
                  t & 0xFF])


def decode_posvel(data):
    '''sec 5 : target position and a NON-NEGATIVE speed limit, both float32 LE'''
    return struct.unpack('<ff', data[:8])


def decode_vel(data):
    '''sec 6 : target velocity, float32 LE, DLC 4'''
    return struct.unpack('<f', data[:4])[0]


def universal_command(data):
    '''the universal action in this payload, or None - sec 3'''
    if len(data) != UNIVERSAL_DLC or bytes(data[:7]) != UNIVERSAL_PREFIX:
        return None
    return data[7] if data[7] in UNIVERSAL_NAMES else None


def build_universal(action):
    return UNIVERSAL_PREFIX + bytes([action])


def build_feedback(motor_id, status, position_rad, velocity_rads, torque_nm,
                   driver_temp_c, motor_temp_c, limits):
    '''sec 7 : one feedback frame, transmitted on the Master ID

    D0 carries BOTH the status code and the motor identity - status in the high nibble,
    the low four bits of the motor id in the low nibble.  sec 7's "ID conflict" note is
    why only four bits are used: the prose claims eight, but D0's upper nibble is already
    spoken for, so motor ids are kept to 1-15 on both sides.
    '''
    p = encode_unsigned(position_rad, limits.p_min, limits.p_max, 16)
    v = encode_unsigned(velocity_rads, limits.v_min, limits.v_max, 12)
    t = encode_unsigned(torque_nm, limits.t_min, limits.t_max, 12)
    return bytes([((status & 0x0F) << 4) | (motor_id & 0x0F),
                  (p >> 8) & 0xFF,
                  p & 0xFF,
                  (v >> 4) & 0xFF,
                  ((v & 0x0F) << 4) | ((t >> 8) & 0x0F),
                  t & 0xFF,
                  driver_temp_c & 0xFF,
                  motor_temp_c & 0xFF])


def decode_feedback(data, limits):
    '''the inverse, so the tests can read back what the fake would transmit'''
    status = data[0] >> 4
    motor_id_low4 = data[0] & 0x0F
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0x0F) << 8) | data[5]
    driver_temp = data[6] - 256 if data[6] >= 128 else data[6]
    motor_temp = data[7] - 256 if data[7] >= 128 else data[7]
    return {
        'status': status,
        'motor_id_low4': motor_id_low4,
        'position_rad': decode_unsigned(p, limits.p_min, limits.p_max, 16),
        'velocity_rads': decode_unsigned(v, limits.v_min, limits.v_max, 12),
        'torque_nm': decode_unsigned(t, limits.t_min, limits.t_max, 12),
        'driver_temp_c': driver_temp,
        'motor_temp_c': motor_temp,
    }


def status_name(status):
    return STATUS_NAMES.get(status, 'undocumented 0x%X' % status)
