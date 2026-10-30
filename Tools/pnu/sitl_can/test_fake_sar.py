#!/usr/bin/env python3
'''
PNU : tests for the fake SAR's CAN framing, without a vcan interface.

Creating a vcan interface needs root, so these drive FakeSAR.handle_frame() with
hand-built SocketCAN frames and capture what it would have transmitted.  What they
cannot cover is the socket itself - binding, the 0x400-0x7FF kernel filter and sharing
the wire with the fake PMU - which is only provable by running against a real vcan;
see README.md in this directory.

Two of these tests read files rather than asserting constants: the ids and DLCs are
checked against SAR_ICD_PNU.md's own table and against AP_PNUSAR.h.  A fake that agrees
only with itself proves nothing, and the ICD is the document both sides signed.

Run with pytest, or directly:  Tools/pnu/sitl_can/test_fake_sar.py

AP_FLAKE8_CLEAN
'''

import os
import re
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sar_frames as icd                                                        # noqa: E402
from fake_sar import FakeSAR, CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG          # noqa: E402
from fake_sar import CAN_FRAME_FMT                                              # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         '..', '..', '..'))


def make_sar(**kwargs):
    '''a FakeSAR with no socket, recording what it would have sent

       The heartbeat is OFF unless a test asks for it.  Leaving it at 1 Hz put a
       Heart_Beat in the middle of every test that advances the clock to wait for a
       response, which is exactly what the fake should do and exactly what makes an
       assertion about what was sent unreadable.'''
    kwargs.setdefault('hb_rate', 0.0)
    sar = FakeSAR('test0', **kwargs)
    sar.sent = []
    sar.send = lambda can_id, payload: sar.sent.append((can_id, payload))
    sar.log = lambda msg: None
    return sar


def frame(can_id, data=b'', rtr=False, extended=False):
    flags = (CAN_EFF_FLAG if extended else 0) | (CAN_RTR_FLAG if rtr else 0)
    return struct.pack(CAN_FRAME_FMT, can_id | flags, len(data), data.ljust(8, b'\x00'))


# -- the heartbeat, ICD 12.1 ----------------------------------------------------------

def test_heartbeat_is_four_bytes_little_endian():
    assert icd.build_heartbeat(0) == b'\x00\x00\x00\x00'
    assert icd.build_heartbeat(1) == b'\x01\x00\x00\x00'
    # ICD 12.1 byte mapping: B0 bits 7-0 ... B3 bits 31-24
    assert icd.build_heartbeat(0x01020304) == b'\x04\x03\x02\x01'
    assert len(icd.build_heartbeat(0)) == icd.HEARTBEAT_DLC


def test_each_heartbeat_advances_the_counter():
    sar = make_sar()
    for _ in range(3):
        sar.send_heartbeat()
    assert [payload for _, payload in sar.sent] == [icd.build_heartbeat(n) for n in range(3)]
    assert sar.raw_counter == 3


def test_the_counter_wraps_at_32_bits():
    sar = make_sar(counter=0xFFFFFFFF)
    sar.send_heartbeat()
    assert sar.raw_counter == 0


def test_the_counter_may_be_driven_backwards():
    '''ICD 3.3 : the FC must not infer a reboot from a Raw_Counter decrease, so the fake
       has to be able to produce one'''
    sar = make_sar(counter=100)
    sar.do_command('counter 5')
    assert sar.raw_counter == 5


def test_a_short_heartbeat_does_not_advance_the_counter():
    sar = make_sar()
    sar.send_short_heartbeat()
    assert sar.sent == [(icd.HEARTBEAT, icd.build_heartbeat(0)[:3])]
    assert sar.raw_counter == 0


def test_heartbeats_are_periodic_and_stop_when_silent():
    sar = make_sar(hb_rate=1.0)
    sar._next_hb = 0.0
    sar.tick(0.0)
    sar.tick(0.5)                       # inside the period, nothing due
    sar.tick(1.0)
    assert len(sar.sent) == 2
    sar.set_silent(True)
    for t in range(2, 8):
        sar.tick(float(t))
    assert len(sar.sent) == 2
    sar.set_silent(False)
    sar.tick(8.0)
    assert len(sar.sent) == 3


def test_hb_rate_zero_never_beats():
    sar = make_sar(hb_rate=0.0)
    for t in range(10):
        sar.tick(float(t))
    assert sar.sent == []


# -- frames are 11-bit standard, ICD section 4 ----------------------------------------

def test_transmitted_frames_are_standard_not_extended():
    '''the PMU on the same bus is 29-bit extended; setting EFF here would collide with
       its id space and never reach AP_PNUSAR'''
    sar = FakeSAR('test0')
    sar.log = lambda msg: None
    captured = []
    sar.sock = type('S', (), {'send': lambda _self, raw: captured.append(raw)})()
    sar.send(icd.HEARTBEAT, icd.build_heartbeat(7))
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, captured[0])
    assert can_id == icd.HEARTBEAT
    assert not (can_id & CAN_EFF_FLAG)
    assert not (can_id & CAN_RTR_FLAG)
    assert dlc == icd.HEARTBEAT_DLC
    assert data[:dlc] == icd.build_heartbeat(7)


def test_extended_and_error_frames_are_ignored():
    sar = make_sar()
    sar.handle_frame(frame(icd.START_CMD, b'\x01', extended=True))
    sar.handle_frame(struct.pack(CAN_FRAME_FMT, icd.START_CMD | CAN_ERR_FLAG, 1, b'\x01' + b'\x00' * 7))
    assert sar.sent == []
    assert sar._rx_counts == {}


def test_a_remote_frame_is_counted_not_answered():
    '''the SAR ICD has no RTR at all - the PMU's polling must not produce a response'''
    sar = make_sar()
    sar.handle_frame(frame(icd.HEARTBEAT, rtr=True))
    assert sar.sent == []
    assert sar._rtr_seen == 1


# -- commands and responses, ICD 12.2, 12.3 and 13 ------------------------------------

def test_both_responses_are_the_single_byte_one():
    '''ICD 13 : "do not encode the Stop payload as 02 merely because it is Stop"'''
    assert icd.build_start_response() == b'\x01'
    assert icd.build_stop_response() == b'\x01'
    assert len(icd.build_start_response()) == icd.RESP_DLC


def test_a_start_command_is_answered_after_the_delay():
    sar = make_sar(resp_delay=0.5)
    now = time.monotonic()
    sar.handle_frame(frame(icd.START_CMD, b'\x01'))
    sar.tick(now)
    assert sar.sent == []               # not instant: the FC's window is 2 s, not 0 s
    sar.tick(now + 0.6)
    assert sar.sent == [(icd.START_RESP, b'\x01')]


def test_a_stop_command_is_answered_on_0x522():
    sar = make_sar(resp_delay=0.0)
    sar.handle_frame(frame(icd.STOP_CMD, b'\x01'))
    sar.tick(time.monotonic() + 1.0)
    assert sar.sent == [(icd.STOP_RESP, b'\x01')]


def test_a_command_with_the_wrong_dlc_is_not_answered():
    sar = make_sar(resp_delay=0.0)
    sar.handle_frame(frame(icd.START_CMD, b'\x01\x01'))
    sar.tick(time.monotonic() + 1.0)
    assert sar.sent == []


def test_responses_off_produces_silence():
    '''ICD 14.1 state 4: the FC's 2 s window expiring needs a device that stays quiet'''
    sar = make_sar(respond=False, resp_delay=0.0)
    sar.handle_frame(frame(icd.START_CMD, b'\x01'))
    sar.tick(time.monotonic() + 1.0)
    assert sar.sent == []
    sar.do_command('respond on')
    sar.handle_frame(frame(icd.START_CMD, b'\x01'))
    sar.tick(time.monotonic() + 1.0)
    assert sar.sent == [(icd.START_RESP, b'\x01')]


def test_the_mode_command_gets_no_invented_answer():
    '''ICD 13 : 0x503's payload is TBD and 0x523's trigger is TBD'''
    sar = make_sar(resp_delay=0.0)
    sar.handle_frame(frame(icd.MODE_CMD, b'\x00' * 8))
    sar.tick(time.monotonic() + 1.0)
    assert sar.sent == []
    assert sar._rx_counts[icd.MODE_CMD] == 1


def test_the_param_response_is_eight_opaque_bytes():
    assert len(icd.build_param_response()) == icd.RESERVED_DLC
    sar = make_sar()
    sar.do_command('param-resp')
    assert sar.sent == [(icd.PARAM_RESP, icd.PARAM_RESPONSE_FILLER)]


def test_navigation_frames_are_counted_not_answered():
    sar = make_sar(resp_delay=0.0)
    for can_id in icd.NAV_IDS:
        sar.handle_frame(frame(can_id, b'\x00' * 8))
        sar.handle_frame(frame(can_id, b'\x00' * 8))
    sar.tick(time.monotonic() + 1.0)
    assert sar.sent == []
    assert all(sar._rx_counts[can_id] == 2 for can_id in icd.NAV_IDS)


def test_intrude_transmits_on_an_fc_to_sar_id():
    '''AP_PNUSAR leaves these unclaimed on purpose, so the driver reports a second
       talker instead of hiding one'''
    sar = make_sar()
    sar.do_command('intrude')
    assert sar.sent == [(icd.NAV_FIRST, b'\x00' * icd.NAV_DLC)]
    sar.do_command('intrude 0x502')
    assert sar.sent[-1][0] == icd.STOP_CMD


# -- agreement with the two documents -------------------------------------------------

def parse_icd_table():
    '''the id / DLC rows of SAR_ICD_PNU.md section 5'''
    path = os.path.join(REPO_ROOT, 'SAR_ICD_PNU.md')
    rows = {}
    pattern = re.compile(r'^\|\s*(0x[0-9A-Fa-f]{3})\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*(\d+)\s*\|')
    with open(path, 'r') as handle:
        for line in handle:
            match = pattern.match(line)
            if match:
                rows[int(match.group(1), 16)] = (match.group(2), match.group(3), int(match.group(4)))
    return rows


def test_every_id_and_dlc_matches_the_icd_table():
    rows = parse_icd_table()
    assert len(rows) == 16, 'ICD section 5 lists 16 unique ids, found %u' % len(rows)
    expected_dlc = {icd.HEARTBEAT: icd.HEARTBEAT_DLC,
                    icd.START_CMD: icd.CMD_DLC, icd.STOP_CMD: icd.CMD_DLC,
                    icd.MODE_CMD: icd.RESERVED_DLC,
                    icd.START_RESP: icd.RESP_DLC, icd.STOP_RESP: icd.RESP_DLC,
                    icd.PARAM_RESP: icd.RESERVED_DLC}
    for can_id in icd.NAV_IDS:
        expected_dlc[can_id] = icd.NAV_DLC

    assert sorted(rows) == sorted(expected_dlc), 'ids disagree with the ICD table'
    for can_id, (name, direction, dlc) in rows.items():
        assert dlc == expected_dlc[can_id], '0x%03X DLC %u, ICD says %u' % (can_id, expected_dlc[can_id], dlc)
        assert icd.name_of(can_id).split()[0] == name, '0x%03X named %s, ICD says %s' % (
            can_id, icd.name_of(can_id), name)
        # direction decides who may transmit; the fake only ever sends SAR -> FLCC ones
        to_fc = direction.strip().startswith('SAR')
        assert to_fc == (can_id in icd.SAR_TO_FC_IDS), '0x%03X direction %s' % (can_id, direction)


def test_the_ids_match_the_firmware_header():
    '''a fake that agrees only with the ICD can still miss the firmware drifting'''
    path = os.path.join(REPO_ROOT, 'libraries', 'AP_PNUSAR', 'AP_PNUSAR.h')
    with open(path, 'r') as handle:
        header = handle.read()
    defines = dict(re.findall(r'#define\s+(AP_PNUSAR_\w+)\s+\((0x[0-9A-Fa-f]+|\d+)U\)', header))
    expected = {
        'AP_PNUSAR_ID_NAV_FIRST': icd.NAV_FIRST,
        'AP_PNUSAR_ID_NAV_LAST': icd.NAV_LAST,
        'AP_PNUSAR_ID_START_CMD': icd.START_CMD,
        'AP_PNUSAR_ID_STOP_CMD': icd.STOP_CMD,
        'AP_PNUSAR_ID_MODE_CMD': icd.MODE_CMD,
        'AP_PNUSAR_ID_HEARTBEAT': icd.HEARTBEAT,
        'AP_PNUSAR_ID_START_RESP': icd.START_RESP,
        'AP_PNUSAR_ID_STOP_RESP': icd.STOP_RESP,
        'AP_PNUSAR_ID_PARAM_RESP': icd.PARAM_RESP,
        'AP_PNUSAR_HEARTBEAT_DLC': icd.HEARTBEAT_DLC,
        'AP_PNUSAR_CMD_DLC': icd.CMD_DLC,
        'AP_PNUSAR_RESP_DLC': icd.RESP_DLC,
        'AP_PNUSAR_NAV_DLC': icd.NAV_DLC,
        'AP_PNUSAR_RESERVED_DLC': icd.RESERVED_DLC,
        'AP_PNUSAR_HB_TIMEOUT_MS': icd.HB_TIMEOUT_MS,
    }
    for name, value in expected.items():
        assert name in defines, '%s is gone from AP_PNUSAR.h' % name
        assert int(defines[name], 0) == value, '%s is %s, the fake says %u' % (name, defines[name], value)


def test_an_unsolicited_response_can_carry_a_wrong_payload():
    sar = make_sar()
    sar.do_command('start-resp')
    sar.do_command('stop-resp bad')
    assert sar.sent[0] == (icd.START_RESP, bytes([icd.RESPONSE_BYTE]))
    assert sar.sent[1] == (icd.STOP_RESP, bytes([icd.BAD_RESPONSE_BYTE]))
    # The ICD defines one payload and the fake must not quietly normalise the other one
    # away: the whole point of "bad" is that the FC gets to refuse it.
    assert icd.BAD_RESPONSE_BYTE != icd.RESPONSE_BYTE


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
