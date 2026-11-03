#!/usr/bin/env python3
'''
PNU : tests for mav_tap.py's hand-written MAVLink2 parser, without SITL.

The tap parses the wire itself rather than using pymavlink, because the message it
exists to watch - TM 51050 - is in the PNU dialect and pymavlink hands it back as
BAD_DATA.  That makes the parser the one place where a silent mistake would turn the
tap's log into confident fiction, so it is tested against hand-built frames.

The heartbeat test pins the exact bytes the tap sends.  They were verified once against
pymavlink's own parser, which accepted the frame and read back MAV_TYPE_GCS: that is the
cross-check, and the hex here is its record.

Run with pytest, or directly:  Tools/pnu/sitl_can/test_mav_tap.py

AP_FLAKE8_CLEAN
'''

import io
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mav_tap                                                                  # noqa: E402


def make_tap():
    tap = mav_tap.Tap('tcp:127.0.0.1:5765', io.StringIO())
    return tap


def lines(tap):
    return [line.split('] ', 1)[1] for line in tap.out.getvalue().splitlines()]


def frame(msgid, payload, seq=0, sysid=1, compid=1, signed=False):
    '''a MAVLink2 frame with a deliberately wrong CRC

       The tap does not check CRCs - it has no CRC_EXTRA table for the PNU dialect -
       so the tests must not accidentally depend on one being right.'''
    incompat = 0x01 if signed else 0x00
    header = struct.pack('<BBBBBBBBH', 0xFD, len(payload), incompat, 0,
                         seq, sysid, compid, msgid & 0xFF, msgid >> 8)
    tail = b'\x00\x00' + (b'\x5a' * 13 if signed else b'')
    return header + payload + tail


def statustext(severity, text):
    return frame(mav_tap.MSG_STATUSTEXT,
                 bytes([severity]) + text.encode('ascii').ljust(50, b'\x00'))


def chunk(severity, text, msg_id, seq):
    '''one chunk of a long statustext, as GCS_Common.cpp:2564-2592 sends them'''
    body = text.encode('ascii')
    assert len(body) <= 50
    return frame(mav_tap.MSG_STATUSTEXT,
                 bytes([severity]) + body.ljust(50, b'\x00')
                 + struct.pack('<HB', msg_id, seq))


def trimmed_chunk(severity, text, msg_id, seq):
    '''a chunk as it really arrives: MAVLink2 drops every trailing zero byte, so an id
       below 256 loses its high byte and a chunk_seq of 0 disappears entirely'''
    payload = bytes([severity]) + text.encode('ascii').ljust(50, b'\x00')
    payload += struct.pack('<HB', msg_id, seq)
    return frame(mav_tap.MSG_STATUSTEXT, payload.rstrip(b'\x00'))


def feed(tap, *frames_in):
    for one in frames_in:
        tap.buf = one
        for msgid, payload in tap.frames():
            tap.handle(msgid, payload)


def tm50(live, status):
    '''WIRE ORDER: Live_Counter uint32 first, then SAR_STATUS - not the XML's order'''
    return frame(mav_tap.MSG_TM50, struct.pack('<IB', live, status))


def test_the_gcs_heartbeat_is_the_bytes_pymavlink_accepted():
    assert mav_tap.heartbeat(0).hex() == 'fd09000000ffbe0000000000000006080000035c2b'
    # the sequence number is the only thing that moves, at offset 4
    assert mav_tap.heartbeat(7)[4] == 7
    assert mav_tap.heartbeat(0)[:4] == mav_tap.heartbeat(7)[:4]


def test_two_frames_in_one_chunk_are_both_parsed():
    tap = make_tap()
    tap.buf = statustext(6, 'first') + statustext(4, 'second')
    got = [(msgid, payload) for msgid, payload in tap.frames()]
    assert len(got) == 2
    assert tap.buf == b''


def test_a_split_frame_waits_for_the_rest():
    tap = make_tap()
    whole = statustext(6, 'PNU_CAN: CAN_D1 PMU+Gimbal+SAR ready')
    tap.buf = whole[:20]
    assert list(tap.frames()) == []
    tap.buf += whole[20:]
    got = list(tap.frames())
    assert len(got) == 1
    tap.handle(*got[0])
    assert lines(tap) == ['INFO   PNU_CAN: CAN_D1 PMU+Gimbal+SAR ready']


def test_garbage_before_a_frame_is_skipped():
    tap = make_tap()
    tap.buf = b'\x01\x02\x03' + statustext(3, 'ERROR here')
    got = list(tap.frames())
    assert len(got) == 1
    tap.handle(*got[0])
    assert lines(tap) == ['ERROR  ERROR here']


def test_a_signed_frame_is_measured_with_its_signature():
    # 13 extra bytes; getting this wrong resynchronises in the middle of the next frame
    tap = make_tap()
    tap.buf = frame(mav_tap.MSG_STATUSTEXT,
                    bytes([6]) + b'signed'.ljust(50, b'\x00'), signed=True)
    tap.buf += statustext(6, 'after')
    got = list(tap.frames())
    assert len(got) == 2
    for msgid, payload in got:
        tap.handle(msgid, payload)
    assert lines(tap) == ['INFO   signed', 'INFO   after']


def test_tm50_is_read_in_wire_order():
    tap = make_tap()
    tap.buf = tm50(123456, 3)
    for msgid, payload in tap.frames():
        tap.handle(msgid, payload)
    assert lines(tap) == ['TM50   SAR_STATUS=3 Observing    Live_Counter=123456']


def test_a_zero_trimmed_tm50_still_decodes():
    '''MAVLink2 drops trailing zero bytes, so state 0 arrives as a 4 or 5 byte payload'''
    tap = make_tap()
    tap.buf = frame(mav_tap.MSG_TM50, struct.pack('<I', 9)[:4])
    for msgid, payload in tap.frames():
        tap.handle(msgid, payload)
    assert lines(tap) == ['TM50   SAR_STATUS=0 Disconnected Live_Counter=9']


def test_tm50_logs_only_when_the_state_changes():
    '''5 Hz unconditional, Live_Counter moving every time: logging each one buries
       every statustext in the file'''
    tap = make_tap()
    for live, status in ((1, 1), (2, 1), (3, 1), (4, 3), (5, 3), (6, 0)):
        tap.buf = tm50(live, status)
        for msgid, payload in tap.frames():
            tap.handle(msgid, payload)
    assert [line.split()[1] for line in lines(tap)] == ['SAR_STATUS=1',
                                                        'SAR_STATUS=3',
                                                        'SAR_STATUS=0']


def test_the_heartbeat_reports_arming_and_mode_changes_only():
    tap = make_tap()

    def hb(custom, armed):
        return frame(mav_tap.MSG_HEARTBEAT,
                     struct.pack('<IBBBBB', custom, 2, 3, 0x80 if armed else 0, 4, 3))
    for payload in (hb(0, False), hb(0, False), hb(5, False), hb(5, True)):
        tap.buf = payload
        for msgid, body in tap.frames():
            tap.handle(msgid, body)
    assert lines(tap) == ['HB     mode=0 armed=False',
                          'HB     mode=5 armed=False',
                          'HB     mode=5 armed=True']


def test_a_long_statustext_is_reassembled_from_its_chunks():
    '''"PNUGimbal: INIT -> ALIGN (all axes enabled, homing skipped)" is 59 characters,
       so it arrives as two chunks.  Logging them separately reads like a firmware bug
       and did: the first version of this tap reported it as truncated on the wire.'''
    whole = 'PNUGimbal: INIT -> ALIGN (all axes enabled, homing skipped)'
    tap = make_tap()
    feed(tap, chunk(6, whole[:50], 7, 0), chunk(6, whole[50:], 7, 1))
    assert lines(tap) == ['INFO   ' + whole]
    assert tap.statustexts == 1


def test_a_long_statustext_survives_zero_trimming():
    '''the form the FC actually sends: chunk 0 is 52 bytes, not 54.  Measured against
       SITL on 2026-11-03 - the untrimmed test above passed while real runs still split
       the message in two.'''
    whole = 'PNUGimbal: ALIGN -> RUN (command seeded from measured angles)'
    tap = make_tap()
    feed(tap, trimmed_chunk(6, whole[:50], 7, 0), trimmed_chunk(6, whole[50:], 7, 1))
    assert lines(tap) == ['INFO   ' + whole]


def test_a_trimmed_short_statustext_keeps_its_whole_text():
    # "PNU_CAN: CAN_D1 PMU+Gimbal+SAR ready" arrives as 37 bytes, text included
    tap = make_tap()
    feed(tap, frame(mav_tap.MSG_STATUSTEXT,
                    bytes([6]) + b'PNU_CAN: CAN_D1 PMU+Gimbal+SAR ready'))
    assert lines(tap) == ['INFO   PNU_CAN: CAN_D1 PMU+Gimbal+SAR ready']


def test_an_unfinished_message_is_flushed_when_the_next_one_starts():
    tap = make_tap()
    feed(tap, chunk(6, 'A' * 50, 9, 0), chunk(4, 'something else', 10, 0))
    assert lines(tap) == ['INFO   ' + 'A' * 50, 'WARN   something else']


def test_a_short_statustext_has_no_id_and_is_not_held():
    # id and chunk_seq are MAVLink2 extensions and are zero-trimmed off entirely
    tap = make_tap()
    feed(tap, statustext(6, 'SAR connected'))
    assert lines(tap) == ['INFO   SAR connected']


def test_statustext_stops_at_the_first_nul():
    tap = make_tap()
    tap.buf = statustext(4, 'SAR: ATT age check off, no source')
    for msgid, payload in tap.frames():
        tap.handle(msgid, payload)
    assert lines(tap) == ['WARN   SAR: ATT age check off, no source']
    assert tap.statustexts == 1


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
