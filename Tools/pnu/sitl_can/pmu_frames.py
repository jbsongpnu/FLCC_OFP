#!/usr/bin/env python3
'''
PNU : the PMU's five status frames, built from a PMUModel.

Wire format only - no sockets, no state.  Every offset, width, type and scale here comes
from "PMU CAN Comm Protocol.md" (repository root) and is cross-checked against
AP_PMUCAN::handle_frame(); see README.md for that comparison.

Two ICD facts that are easy to get wrong and that this file is the only place to get
right: every INT16 analog field is physical = raw * 0.1, and HOURCNT is a 24-bit count of
SECONDS despite its name.

AP_FLAKE8_CLEAN
'''

import struct

# status ids, 29-bit extended
BATSTS = 0x10000100
ENGSTS = 0x10000110
AUX1STS = 0x10000120
AUX2STS = 0x10000130
VERSTS = 0x10000140
STATUS_IDS = (BATSTS, ENGSTS, AUX1STS, AUX2STS, VERSTS)

# command ids, 29-bit extended, DLC 4, payload one little-endian UINT32
BATCTRL = 0x10000000
ENGONOFF = 0x10000010
ENGMANUAL = 0x10000020
ENGPCL = 0x10000030
ENGCHK = 0x10000040
COMMAND_IDS = (BATCTRL, ENGONOFF, ENGMANUAL, ENGPCL, ENGCHK)

# ECU V2.0 only.  Two RTR-requested status ids and one command, all DLC 8.  A legacy PMU
# does not have them, so build_ecu() below is the only path that answers them and
# fake_pmu.py only reaches it in --ecu mode.
MODESTS = 0x10000150
ENERGYSTS = 0x10000160
ECU_STATUS_IDS = (MODESTS, ENERGYSTS)
ENERGYCFG = 0x10000050

STATUS_DLC = 8
COMMAND_DLC = 4
ENERGYCFG_DLC = 8

# MODEL CHOICE.  The ICD defines DATE only as "UINT32 BCD" and fixes no component order,
# so this is a plausible BCD date and nothing should depend on it.  MJ/MN are 1 and 0 by
# PNU's instruction - 0x01 and 0x00 read the same in BCD and in binary.
VERSION_DATE_BCD = 0x20261029
VERSION_MAJOR_BCD = 0x01
VERSION_MINOR_BCD = 0x00

# MODEL CHOICE.  GAGE is not modelled; the FC ignores it and it only has to be plausible.
GAGE_PERCENT = 100.0

# GAS is fixed: the aircraft has no fuel sensor, and PSTS bit 8 is held clear to match.
FUEL_PERCENT = 100

VBUS_NOMINAL = 48.0

# ---- ECU V2.0 ----------------------------------------------------------------------
#
# VERSTS: the ICD's own example reply, `01 12 26 20 00 02 00 00` - DATE 2026-12-01 BCD,
# MN 0x00, MJ 0x02, so decimal 2.00.  Anything at or above 2.0 selects ECU_V2 in the FC.
ECU_VERSION_DATE_BCD = 0x20261201
ECU_VERSION_MAJOR_BCD = 0x02
ECU_VERSION_MINOR_BCD = 0x00

# MODESTS and ENERGYSTS are FIXED PAYLOADS, taken verbatim from the ICD's encoding
# examples.  Stage 0 verifies protocol encoding and decoding, not ECU behaviour: there is
# no operating-mode machine, no governor and no EMS here, so a changing payload would be
# invention rather than a test vector.  test_fake_pmu.py decodes both back to the values
# the ICD states, which is what makes a typo in these constants a failing test.
#
#   MODESTS   53 06 00 03 00 00 CC 10
#             byte 0 = ENGINE_RUN (3) / RUNNING (5), ctrl_mode EMS (6), alert NONE (0),
#             ems_mode CHARGE (3), start_gate 0, initial_fail 0, rpm_target 4300
#   ENERGYSTS 20 03 DC 00 20 1C 00 00
#             tank 8.00 L, battery 22.0 Ah, endurance 7200 s
MODESTS_EXAMPLE = bytes((0x53, 0x06, 0x00, 0x03, 0x00, 0x00, 0xCC, 0x10))
ENERGYSTS_EXAMPLE = bytes((0x20, 0x03, 0xDC, 0x00, 0x20, 0x1C, 0x00, 0x00))

# HOURCNT is UINT24 on the legacy PMU and UINT32 on the ECU, and byte 3 is the only
# difference.  In ECU mode the count is offset by this base so that BYTE 3 IS NON-ZERO:
# a decoder still reading 24 bits reports the model's own seconds, a correct 32-bit
# decoder reports base + seconds, and the two can never be confused.  2^24 exactly, so
# the low three bytes still read as the plain running time.
ECU_HOURCNT_BASE = 0x01000000


def scaled(value):
    '''ICD: physical = raw * 0.1 for every INT16 analog field'''
    return max(-32768, min(32767, int(round(value * 10.0))))


def build_batsts(model):
    '''BSTS, TEMP, IBAT, GAGE.  BSTS is unused; battery temperature is not modelled.'''
    return struct.pack('<Hhhh', 0, 0, scaled(model.battery_current), scaled(GAGE_PERCENT))


def build_engsts(model):
    '''TEMP1, TEMP2, RPM, GAS, PCL.  GAS and PCL are INT8 raw percent, scale 1.0.'''
    return struct.pack('<hhHbb', scaled(model.head_temp), scaled(model.head_temp),
                       int(round(model.rpm)), FUEL_PERCENT, model.throttle_position)


def build_aux1sts(model):
    '''ICMD, ILD, VBUS, GCMD.  ICMD's type and scale are TBD in the ICD - it is a
       developer-debug field, so the last BATCTRL is echoed and nothing depends on it.'''
    return struct.pack('<hhhh', model.battery_cmd, scaled(model.load_current),
                       scaled(VBUS_NOMINAL), scaled(float(model.battery_cmd)))


def build_aux2sts(model):
    '''HOURCNT (UINT24, seconds), one reserved byte, PSTS, PTEMP.  PTEMP is not modelled.'''
    seconds = int(model.hour_count_s) & 0xFFFFFF
    return struct.pack('<I', seconds)[:3] + struct.pack('<BHh', 0, model.psts, 0)


def build_versts(model):
    '''DATE, MN, MJ, RSV - all BCD except the reserved tail.'''
    return struct.pack('<IBBH', VERSION_DATE_BCD, VERSION_MINOR_BCD, VERSION_MAJOR_BCD, 0)


def build_versts_ecu(model):
    '''ECU V2.0 VERSTS: software 2.00.  Equals the ICD's example reply byte for byte.'''
    return struct.pack('<IBBH', ECU_VERSION_DATE_BCD, ECU_VERSION_MINOR_BCD,
                       ECU_VERSION_MAJOR_BCD, 0)


def build_aux2sts_ecu(model):
    '''ECU V2.0 AUX2STS: HOURCNT is UINT32 over bytes 0-3, so the legacy reserved byte is
       part of the count.  See ECU_HOURCNT_BASE for why byte 3 is deliberately non-zero.'''
    seconds = (ECU_HOURCNT_BASE + int(model.hour_count_s)) & 0xFFFFFFFF
    return struct.pack('<IHh', seconds, model.psts, 0)


def build_modests(model):
    '''fixed ICD example - see MODESTS_EXAMPLE'''
    return MODESTS_EXAMPLE


def build_energysts(model):
    '''fixed ICD example - see ENERGYSTS_EXAMPLE'''
    return ENERGYSTS_EXAMPLE


_BUILDERS = {
    BATSTS: build_batsts,
    ENGSTS: build_engsts,
    AUX1STS: build_aux1sts,
    AUX2STS: build_aux2sts,
    VERSTS: build_versts,
}

# ECU V2.0 changes two of the five legacy replies and adds two of its own.  BATSTS,
# ENGSTS and AUX1STS are byte-identical on both generations, which is why they are
# reached through _BUILDERS below rather than copied.
_ECU_BUILDERS = {
    VERSTS: build_versts_ecu,
    AUX2STS: build_aux2sts_ecu,
    MODESTS: build_modests,
    ENERGYSTS: build_energysts,
}


def build(can_id, model):
    payload = _BUILDERS[can_id](model)
    assert len(payload) == STATUS_DLC
    return payload


def build_ecu(can_id, model):
    '''ECU V2.0 reply for @can_id, falling through to the legacy builder for the three
       messages the ECU did not change.'''
    builder = _ECU_BUILDERS.get(can_id) or _BUILDERS[can_id]
    payload = builder(model)
    assert len(payload) == STATUS_DLC
    return payload


def decode_energycfg(data):
    '''ENERGYCFG's four little-endian UINT16 fields, raw.  0xFFFF means "keep the current
       value" in every one of them (ICD rule 1), and the ECU - not this decoder - is what
       validates ranges and decides whether to accept the frame.'''
    tank, fuel, battery, soc = struct.unpack('<HHHH', data)
    return {'tank': tank, 'fuel': fuel, 'battery': battery, 'soc': soc}
