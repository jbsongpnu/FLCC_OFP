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

STATUS_DLC = 8
COMMAND_DLC = 4

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


_BUILDERS = {
    BATSTS: build_batsts,
    ENGSTS: build_engsts,
    AUX1STS: build_aux1sts,
    AUX2STS: build_aux2sts,
    VERSTS: build_versts,
}


def build(can_id, model):
    payload = _BUILDERS[can_id](model)
    assert len(payload) == STATUS_DLC
    return payload
