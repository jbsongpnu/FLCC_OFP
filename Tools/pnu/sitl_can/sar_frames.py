#!/usr/bin/env python3
'''
PNU : the SAR's CAN ids and the four frames a SAR device transmits.

Wire format only - no sockets, no state.  Every id, DLC, byte order and payload here
comes from SAR_ICD_PNU.md (repository root) sections 4, 5, 12 and 13, and is
cross-checked against AP_PNUSAR.h by test_fake_sar.py.

Three ICD facts that this file is the only place to get right:

  - EVERY SAR FRAME IS AN 11-BIT STANDARD DATA FRAME (section 4).  Nothing here sets
    CAN_EFF_FLAG.  The PMU sharing this bus is 29-bit extended throughout, and that
    difference is what keeps the two apart.
  - Heart_Beat carries the SAR'S OWN Raw_Counter, little-endian, B0 = bits 7-0
    (section 12.1).  The FC must NOT copy it into Live_Counter - section 3.3 says so
    twice - so the fake deliberately makes the two differ.
  - Start and Stop responses are ONE BYTE 0x01, both of them.  Section 13 forbids
    encoding Stop as 0x02 "merely because it is Stop".

AP_FLAKE8_CLEAN
'''

import struct

# navigation bundle, FC -> SAR, 10 Hz, DLC 8 each.  Named individually because the fake
# logs what it receives by name, and "0x404" says much less than "ATTITUDE".
NAV_IDS = {
    0x400: 'GPS_POSITION',
    0x401: 'GPS_MOTION',
    0x402: 'GPS_AUX',
    0x403: 'GPS_ACCURACY',
    0x404: 'ATTITUDE',
    0x405: 'GPS_TIME_STATUS',
    0x406: 'GPS_SAMPLE_INFO',
    0x407: 'GPS_DIAGNOSTICS',
    0x408: 'SNAPSHOT_TIME_US',
}
NAV_FIRST = 0x400
NAV_LAST = 0x408

# commands, FC -> SAR
START_CMD = 0x501
STOP_CMD = 0x502
MODE_CMD = 0x503                # RESERVED, payload TBD - never built here

# device messages, SAR -> FC
HEARTBEAT = 0x510
START_RESP = 0x521
STOP_RESP = 0x522
PARAM_RESP = 0x523              # RESERVED, payload TBD - opaque filler only

HEARTBEAT_DLC = 4
CMD_DLC = 1
RESP_DLC = 1
NAV_DLC = 8
RESERVED_DLC = 8

# ICD 2.3 : the invalid sentinels.  "Do not use an otherwise valid value of zero to
# represent invalid data" - so a decoder that silently turns these into numbers would
# hide exactly the condition they exist to signal.  decode_bundle() returns None instead.
INVALID_U16 = 0xFFFF
INVALID_I16 = -32768
INVALID_I32 = -2147483648
INVALID_U32 = 0xFFFFFFFF

# ICD 6.5, bit 0 is the LSB
GPS_FLAG_NAMES = ('GPS_USED', 'VV_AVAIL', 'GPS_FRESH', 'TIME_VALID',
                  'ENCODING_ERROR', 'AEL_VALID', 'RTCM_STATS', 'RESERVED')
ATT_FLAG_NAMES = ('ROLL_VALID', 'PITCH_VALID', 'YAW_VALID', 'ATT_FRESH',
                  'RSV4', 'RSV5', 'RSV6', 'RSV7')

# ICD 6.4
GPS_STATUS_NAMES = {0: 'NO_GPS', 1: 'NO_FIX', 2: 'FIX_2D', 3: 'FIX_3D',
                    4: 'DGPS', 5: 'RTK_FLOAT', 6: 'RTK_FIXED'}

# ICD 13 : both commands and both responses are the single byte 0x01.
COMMAND_BYTE = 0x01
RESPONSE_BYTE = 0x01

# Not a protocol value: a deliberate violation, so that the FC's "the only defined payload
# is 0x01" rule (ICD 12.2 and 12.3) can be exercised rather than assumed.
BAD_RESPONSE_BYTE = 0xFF

# ICD 3.3 / 14.1 : T_HB.  The FC owns this deadline; it is here so that the fake's
# console and the tests can reason about it, and so one place fails if it ever moves.
HB_TIMEOUT_MS = 3000

ID_NAMES = dict(NAV_IDS)
ID_NAMES.update({
    START_CMD: 'Observation_Start_Command',
    STOP_CMD: 'Observation_Stop_Command',
    MODE_CMD: 'Observation_Mode_Command (RESERVED)',
    HEARTBEAT: 'Heart_Beat',
    START_RESP: 'Observation_Start_Response',
    STOP_RESP: 'Observation_Stop_Response',
    PARAM_RESP: 'Observation_Param_Response (RESERVED)',
})

FC_TO_SAR_IDS = tuple(sorted(NAV_IDS)) + (START_CMD, STOP_CMD, MODE_CMD)
SAR_TO_FC_IDS = (HEARTBEAT, START_RESP, STOP_RESP, PARAM_RESP)

# MODEL CHOICE.  0x523's eight bytes are manufacturer-reserved and ICD 12.4 forbids
# inventing an interpretation, so the fake sends a filler that cannot be mistaken for
# a plausible value.  Nothing may read meaning into it, and the FC does not parse it.
PARAM_RESPONSE_FILLER = b'\xA5' * RESERVED_DLC


def name_of(can_id):
    return ID_NAMES.get(can_id, 'unknown')


def build_heartbeat(raw_counter):
    '''0x510, DLC 4 : Raw_Counter uint32 little-endian, B0 = bits 7-0'''
    return struct.pack('<I', raw_counter & 0xFFFFFFFF)


def _u16(data, offset):
    return struct.unpack_from('<H', data, offset)[0]


def _i16(data, offset):
    return struct.unpack_from('<h', data, offset)[0]


def _u32(data, offset):
    return struct.unpack_from('<I', data, offset)[0]


def _i32(data, offset):
    return struct.unpack_from('<i', data, offset)[0]


def _scaled(raw, sentinel, mult):
    '''physical value, or None when the transmitter sent the unavailable sentinel'''
    return None if raw == sentinel else raw / float(mult)


def flag_names(value, names):
    return [name for bit, name in enumerate(names) if value & (1 << bit)]


def decode_bundle(frames):
    '''turn one complete 0x400-0x408 bundle into physical values - ICD 5, 6 and 7

    @frames maps each navigation id to its 8 payload bytes.  Every multi-byte field is
    little-endian (ICD 2.2) and every sentinel comes back as None rather than as a
    number, because a receiver that cannot tell "unavailable" from "zero" is the exact
    failure ICD 2.3 is written to prevent.
    '''
    f = frames
    out = {}

    # 0x400 GPS_POSITION - already deg * 1e7, so dividing is the only conversion
    out['Lat'] = _scaled(_i32(f[0x400], 0), INVALID_I32, 1e7)
    out['Lng'] = _scaled(_i32(f[0x400], 4), INVALID_I32, 1e7)

    # 0x401 GPS_MOTION
    out['Alt'] = _scaled(_i32(f[0x401], 0), INVALID_I32, 100)
    out['Spd'] = _scaled(_u16(f[0x401], 4), INVALID_U16, 100)
    out['GCrs'] = _scaled(_u16(f[0x401], 6), INVALID_U16, 100)

    # 0x402 GPS_AUX.  VZ is NED down: POSITIVE DOWNWARD (ICD 6.2).
    out['VZ'] = _scaled(_i16(f[0x402], 0), INVALID_I16, 100)
    out['GPS_Yaw'] = _scaled(_u16(f[0x402], 2), INVALID_U16, 100)
    out['HDop'] = _scaled(_u16(f[0x402], 4), INVALID_U16, 100)
    out['VDop'] = _scaled(_u16(f[0x402], 6), INVALID_U16, 100)

    # 0x403 GPS_ACCURACY
    out['HAcc'] = _scaled(_u16(f[0x403], 0), INVALID_U16, 100)
    out['VAcc'] = _scaled(_u16(f[0x403], 2), INVALID_U16, 100)
    out['SAcc'] = _scaled(_u16(f[0x403], 4), INVALID_U16, 100)
    out['YAcc'] = _scaled(_u16(f[0x403], 6), INVALID_U16, 100)

    # 0x404 ATTITUDE - the SAR's actual attitude, not the airframe's (ICD 16.1)
    out['Roll'] = _scaled(_i16(f[0x404], 0), INVALID_I16, 100)
    out['Pitch'] = _scaled(_i16(f[0x404], 2), INVALID_I16, 100)
    out['Yaw'] = _scaled(_u16(f[0x404], 4), INVALID_U16, 100)
    out['ATT_Seq'] = f[0x404][6]
    out['ATT_Flags'] = f[0x404][7]

    # 0x405 GPS_TIME_STATUS
    gms = _u32(f[0x405], 0)
    gwk = _u16(f[0x405], 4)
    out['GMS'] = None if gms == INVALID_U32 else gms
    out['GWk'] = None if gwk == INVALID_U16 else gwk
    out['Status'] = f[0x405][6]
    out['NSats'] = f[0x405][7]

    # 0x406 GPS_SAMPLE_INFO.  SMS is exempt from the sentinel rules (ICD 2.3): it is the
    # raw millisecond counter and 0xFFFFFFFF is a legitimate value before it rolls over.
    out['SMS'] = _u32(f[0x406], 0)
    delta = _u16(f[0x406], 4)
    out['Delta'] = None if delta == INVALID_U16 else delta
    out['I'] = f[0x406][6]
    out['GPS_Flags'] = f[0x406][7]

    # 0x407 GPS_DIAGNOSTICS
    out['AEl'] = _scaled(_i32(f[0x407], 0), INVALID_I32, 100)
    out['RTCMFU'] = _u16(f[0x407], 4)
    out['RTCMFD'] = _u16(f[0x407], 6)

    # 0x408 SNAPSHOT_TIME_US - closing timestamp, us since FC boot (ICD 7.3)
    out['Snapshot_TimeUS'] = struct.unpack_from('<Q', f[0x408], 0)[0]
    return out


def format_bundle(b):
    '''one decoded bundle as readable lines; None prints as "n/a"'''
    def v(key, fmt='%s'):
        return 'n/a' if b.get(key) is None else fmt % b[key]

    status = b['Status']
    return [
        'Lat %s Lng %s Alt %s m AEl %s m'
        % (v('Lat', '%.7f'), v('Lng', '%.7f'), v('Alt', '%.2f'), v('AEl', '%.2f')),
        'Spd %s m/s GCrs %s deg VZ %s m/s GPS_Yaw %s deg'
        % (v('Spd', '%.2f'), v('GCrs', '%.2f'), v('VZ', '%.2f'), v('GPS_Yaw', '%.2f')),
        'HDop %s VDop %s HAcc %s VAcc %s SAcc %s YAcc %s'
        % (v('HDop', '%.2f'), v('VDop', '%.2f'), v('HAcc', '%.2f'),
           v('VAcc', '%.2f'), v('SAcc', '%.2f'), v('YAcc', '%.2f')),
        'Status %u (%s) NSats %u I %u GMS %s GWk %s'
        % (status, GPS_STATUS_NAMES.get(status, 'OUTSIDE PROFILE'),
           b['NSats'], b['I'], v('GMS'), v('GWk')),
        'SMS %u ms Delta %s ms RTCMFU %u RTCMFD %u'
        % (b['SMS'], v('Delta'), b['RTCMFU'], b['RTCMFD']),
        'GPS_Flags 0x%02X [%s]'
        % (b['GPS_Flags'], ' '.join(flag_names(b['GPS_Flags'], GPS_FLAG_NAMES)) or 'none'),
        'Roll %s Pitch %s Yaw %s ATT_Seq %u ATT_Flags 0x%02X [%s]'
        % (v('Roll', '%.2f'), v('Pitch', '%.2f'), v('Yaw', '%.2f'), b['ATT_Seq'],
           b['ATT_Flags'],
           ' '.join(flag_names(b['ATT_Flags'], ATT_FLAG_NAMES)) or 'none'),
        'Snapshot_TimeUS %u' % b['Snapshot_TimeUS'],
    ]


def build_start_response():
    '''0x521, DLC 1, payload 0x01 - ICD 12.2'''
    return bytes([RESPONSE_BYTE])


def build_stop_response():
    '''0x522, DLC 1, payload 0x01 - ICD 12.3.  Not 0x02.'''
    return bytes([RESPONSE_BYTE])


def build_param_response():
    '''0x523, DLC 8, opaque - ICD 12.4'''
    return PARAM_RESPONSE_FILLER
