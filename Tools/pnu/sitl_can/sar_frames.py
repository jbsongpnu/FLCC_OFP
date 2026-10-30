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


def build_start_response():
    '''0x521, DLC 1, payload 0x01 - ICD 12.2'''
    return bytes([RESPONSE_BYTE])


def build_stop_response():
    '''0x522, DLC 1, payload 0x01 - ICD 12.3.  Not 0x02.'''
    return bytes([RESPONSE_BYTE])


def build_param_response():
    '''0x523, DLC 8, opaque - ICD 12.4'''
    return PARAM_RESPONSE_FILLER
