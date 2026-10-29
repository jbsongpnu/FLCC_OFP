#!/usr/bin/env python3
'''
PNU : TC1 injector and interlock stopwatch, for measuring CAN Update Plan step 2c.

Step 2c added two windows to the engine-start interlock (PNU-ISSUE(D7), AP_PMUCAN):

  - TC1 STALENESS, 500 ms   : stop counting when TC1 goes quiet, so a link dying
                              mid-burst no longer completes the start on its own
  - STOP BEFORE START, 10 s : refuse a Start that does not follow a recent Stop

Neither can be measured by pressing buttons, because the interesting cases are a link
dying at a chosen instant.  This sends the TC1 bursts KGCS would send, with the timing
under our control, and reports what the FC did about it.

IT WATCHES BOTH ENDS ITSELF.  The MAVLink side sends TC1 and reads STATUSTEXT; the CAN
side listens on vcan0 for the ENGONOFF command the FC emits when the interlock completes.
One process, one clock, so the measured interval needs no correlation between logs.

NOTHING IS TRANSMITTED ON CAN - the socket is listen-only.  Run the fake PMU alongside;
this does not replace it, and the FC needs it to consider the PMU healthy.

    Tools/pnu/sitl_can/tc1_inject.py stop-start
    Tools/pnu/sitl_can/tc1_inject.py --list

DO NOT RUN KGCS AT THE SAME TIME.  This tool stands in for it, and the FC does not check
who sent a TC1 (an accepted residual of PNU-ISSUE(D7)), so two senders would interleave
their bursts and the result would mean nothing.  It defaults to SITL's SERIAL1 - the port
KGCS would otherwise be on - which leaves SERIAL0 for Mission Planner and SERIAL2 for the
fake PMU, exactly as in that procedure.

SCENARIO ORDER MATTERS.  The FC latches the engine state, so a successful "stop-start"
leaves it ON and later Start scenarios then do nothing.  Run "start-only" first on a fresh
boot, since it tests the case where no Stop has EVER been seen, and run "stop" to get back
to OFF.

AP_FLAKE8_CLEAN
'''

import argparse
import os
import select
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pmu_frames as icd                # noqa: E402

# -- the TC1 message, hand-packed ------------------------------------------------------
#
# SYS_ICD_GCS_FLCC_PMU_CTRL, Tools/pnu/mavlink_defs/pnu_kal.xml.  The PNU dialect is not
# in a stock pymavlink, and generating one would put a build step in front of a 40-line
# tool, so the frame is packed by hand.  CRC_EXTRA is derived the standard way and checked
# against the firmware's own generated header - see --self-test.
TC1_MSGID = 50001
TC1_NAME = 'SYS_ICD_GCS_FLCC_PMU_CTRL'
TC1_FIELDS = (
    ('uint8_t', 'Engine_OnOff'),
    ('uint8_t', 'Battery_Control_CMD'),
    ('uint8_t', 'Engine_Manual'),
    ('uint8_t', 'Engine_Throttle_CMD'),
    ('uint8_t', 'Engine_CHK_CMD'),
)
TC1_LEN = 5                             # every field is one byte, so wire order is XML order

# KGCS alternates these while a button is held, ~10 Hz, 14 messages, then stops.
START_VALUES = (2, 3)
STOP_VALUES = (4, 5)
KGCS_BURST = 14
KGCS_RATE_HZ = 10.0

# step 2c's two windows, for the expectations printed below.  Kept in step with
# AP_PMUCAN.h by --self-test, which reads them back out of the header.
TC1_STALE_MS = 500
STOP_WINDOW_MS = 10000

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FMT = '<IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


def x25crc(data):
    crc = 0xFFFF
    for b in bytearray(data):
        t = b ^ (crc & 0xFF)
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def crc_extra():
    '''the MAVLink message checksum seed: name and field types/names, then folded'''
    seed = TC1_NAME + ' '
    for ftype, fname in TC1_FIELDS:
        seed += '%s %s ' % (ftype, fname)
    crc = x25crc(seed.encode())
    return (crc & 0xFF) ^ (crc >> 8)


TC1_CRC_EXTRA = crc_extra()


class Timeline(object):
    '''one monotonic clock for everything this tool prints'''

    def __init__(self):
        self.t0 = time.monotonic()
        self.events = []

    def now(self):
        return time.monotonic() - self.t0

    def log(self, kind, msg):
        t = self.now()
        self.events.append((t, kind, msg))
        print('[%8.3f] %-4s %s' % (t, kind, msg), flush=True)

    def first(self, kind, contains=''):
        for t, k, m in self.events:
            if k == kind and contains in m:
                return t
        return None


class Injector(object):

    def __init__(self, url, interface, timeline, sysid=255, compid=190):
        self.url = url
        self.interface = interface
        self.tl = timeline
        self.sysid = sysid
        self.compid = compid
        self.seq = 0
        self.conn = None
        self.can = None

    def open(self, link_timeout=60.0):
        from pymavlink import mavutil
        # SITL's SERIAL1 and SERIAL2 (TCP 5762 and 5763) DO NOT EXIST until something
        # connects to SERIAL0: it is tcp:0:wait, so SITL blocks at startup with only 5760
        # open.  Waiting beats failing - the usual reason this is refused is that Mission
        # Planner has not attached yet, and it will.
        deadline = time.monotonic() + link_timeout
        announced = False
        while True:
            try:
                self.conn = mavutil.mavlink_connection(self.url, source_system=self.sysid,
                                                       source_component=self.compid)
                break
            except Exception as err:
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        '%s never opened (%s).\n'
                        'SITL opens SERIAL1 and SERIAL2 only after something connects to\n'
                        'SERIAL0 on tcp:5760 - attach Mission Planner first, or start SITL\n'
                        'with -A "--serial0 tcp:0" so it does not wait for one.' % (self.url, err))
                if not announced:
                    announced = True
                    self.tl.log('LINK', '%s not open yet - waiting up to %.0f s. '
                                        'SERIAL1/2 appear once SERIAL0 (tcp:5760) has a client'
                                % (self.url, link_timeout))
                time.sleep(2.0)
        self.tl.log('LINK', 'MAVLink %s, sending as %u/%u' % (self.url, self.sysid, self.compid))
        if self.interface.lower() != 'none':
            self.can = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # listen only, and only for the PMU command block
            self.can.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                struct.pack('=II', 0x10000000 | CAN_EFF_FLAG,
                                            0x1FFFFF00 | CAN_EFF_FLAG))
            self.can.bind((self.interface,))
            self.tl.log('LINK', 'watching %s for ENGONOFF (listen only)' % self.interface)

    def send_tc1(self, engine_onoff, battery=0, manual=0, throttle=0, choke=0):
        payload = struct.pack('<BBBBB', engine_onoff, battery, manual, throttle, choke)
        frame = struct.pack('<BBBBBBB', 0xFD, TC1_LEN, 0, 0, self.seq, self.sysid, self.compid)
        frame += struct.pack('<I', TC1_MSGID)[:3] + payload
        frame += struct.pack('<H', x25crc(frame[1:] + struct.pack('<B', TC1_CRC_EXTRA)))
        self.seq = (self.seq + 1) & 0xFF
        self.conn.write(frame)

    def burst(self, values, count, rate_hz, label):
        '''the alternating burst KGCS sends while a button is held'''
        self.tl.log('TC1', '%s burst: %u messages alternating %s at %.1f Hz'
                    % (label, count, '/'.join(str(v) for v in values), rate_hz))
        period = 1.0 / rate_hz
        for i in range(count):
            self.send_tc1(values[i % len(values)])
            deadline = time.monotonic() + period
            self.pump(max(0.0, deadline - time.monotonic()))
        self.tl.log('TC1', '%s burst ended after %u messages' % (label, count))

    def pump(self, seconds):
        '''read both links for a while, logging anything that matters'''
        deadline = time.monotonic() + seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            socks = [self.can] if self.can is not None else []
            ready, _, _ = select.select(socks, [], [], min(remaining, 0.02))
            for s in ready:
                raw = s.recv(CAN_FRAME_SIZE)
                if len(raw) == CAN_FRAME_SIZE:
                    self.handle_can(raw)
            while True:
                msg = self.conn.recv_match(type='STATUSTEXT', blocking=False)
                if msg is None:
                    break
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode('utf-8', 'replace')
                self.tl.log('GCS', text)

    def handle_can(self, raw):
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        if can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG):
            return
        if (can_id & CAN_EFF_MASK) != icd.ENGONOFF:
            return
        value = struct.unpack('<I', data[:4])[0] if dlc >= 4 else None
        self.tl.log('CAN', 'ENGONOFF CMD=%s  <-- the FC commanded the PMU' % value)


# -- scenarios -------------------------------------------------------------------------

def sc_start(inj, args):
    '''a Start burst on its own'''
    inj.burst(START_VALUES, args.count, args.rate, 'Start')
    inj.pump(args.settle)


def sc_stop(inj, args):
    '''a Stop burst on its own'''
    inj.burst(STOP_VALUES, args.count, args.rate, 'Stop')
    inj.pump(args.settle)


def sc_stop_start(inj, args):
    '''the correct KGCS procedure: Stop, wait out the button lockout, Start'''
    inj.burst(STOP_VALUES, args.count, args.rate, 'Stop')
    inj.tl.log('WAIT', 'holding %.1f s - KGCS locks both buttons for about 5 s' % args.wait)
    inj.pump(args.wait)
    inj.burst(START_VALUES, args.count, args.rate, 'Start')
    inj.pump(args.settle)


def sc_start_only(inj, args):
    '''Start with no Stop ever seen - the first Start of a boot'''
    inj.burst(START_VALUES, args.count, args.rate, 'Start')
    inj.pump(args.settle)


def sc_late_start(inj, args):
    '''Stop, then a Start after the 10 s window has passed'''
    inj.burst(STOP_VALUES, args.count, args.rate, 'Stop')
    hold = args.wait if args.wait > (STOP_WINDOW_MS / 1000.0) else (STOP_WINDOW_MS / 1000.0) + 4.0
    inj.tl.log('WAIT', 'holding %.1f s, past the %.0f s stop-before-start window'
               % (hold, STOP_WINDOW_MS / 1000.0))
    inj.pump(hold)
    inj.burst(START_VALUES, args.count, args.rate, 'Start')
    inj.pump(args.settle)


def sc_cut_start(inj, args):
    '''THE ONE STEP 2C EXISTS FOR: a Start burst cut off after a couple of messages,
       as if the link died.  Before step 2c the frozen pair kept the counter running and
       the engine started about a second later with no further operator input.'''
    inj.burst(STOP_VALUES, args.count, args.rate, 'Stop')
    inj.pump(args.wait)
    inj.burst(START_VALUES, args.cut, args.rate, 'Start (CUT SHORT)')
    inj.tl.log('WAIT', 'link is now dead - watching %.1f s for a start that should not come'
               % args.settle)
    inj.pump(args.settle)


def sc_cut_stop(inj, args):
    '''the symmetric case: a Stop burst cut short.  Step 2c means this no longer
       completes either, which is a side effect rather than a goal.'''
    inj.burst(STOP_VALUES, args.cut, args.rate, 'Stop (CUT SHORT)')
    inj.pump(args.settle)


SCENARIOS = {
    'start': (sc_start, 'Start burst alone'),
    'stop': (sc_stop, 'Stop burst alone'),
    'stop-start': (sc_stop_start, 'the correct procedure - the engine SHOULD start'),
    'start-only': (sc_start_only, 'Start with no Stop - should be REFUSED'),
    'late-start': (sc_late_start, 'Start after the 10 s window - should be REFUSED'),
    'cut-start': (sc_cut_start, 'Start burst cut short - should NOT start (guard 1)'),
    'cut-stop': (sc_cut_stop, 'Stop burst cut short - should NOT stop (side effect)'),
}

EXPECT_START = {
    'stop-start': True,
    'start-only': False,
    'late-start': False,
    'cut-start': False,
}


def self_test(root):
    '''check the hand-packed message against the firmware's own generated header'''
    ok = True
    print('crc_extra computed from the message definition: %u' % TC1_CRC_EXTRA)
    found = False
    for board in ('CubeOrangePlus', 'sitl'):
        path = os.path.join(root, 'build', board, 'libraries', 'GCS_MAVLink', 'include',
                            'mavlink', 'v2.0', 'pnu_kal',
                            'mavlink_msg_sys_icd_gcs_flcc_pmu_ctrl.h')
        if not os.path.exists(path):
            continue
        found = True
        wanted = {'_CRC': TC1_CRC_EXTRA, '_LEN': TC1_LEN}
        for line in open(path):
            for key, value in wanted.items():
                token = 'MAVLINK_MSG_ID_SYS_ICD_GCS_FLCC_PMU_CTRL%s ' % key
                if line.startswith('#define ' + token):
                    got = int(line.split()[-1])
                    status = 'MATCH' if got == value else 'MISMATCH'
                    if got != value:
                        ok = False
                    print('  %-12s header=%-4u tool=%-4u %s' % (board + key, got, value, status))
    if not found:
        print('  no generated header found - build first to check against the firmware')
        return False
    hdr = os.path.join(root, 'libraries', 'AP_PMUCAN', 'AP_PMUCAN.h')
    for name, value in (('PMUCAN_TC1_STALE_MS', TC1_STALE_MS),
                        ('PMUCAN_STOP_WINDOW_MS', STOP_WINDOW_MS)):
        got = None
        for ln in open(hdr):
            if ln.startswith('#define %s ' % name):
                got = int(ln.split('(')[1].split('U')[0])
        status = 'MATCH' if got == value else 'MISMATCH'
        if got != value:
            ok = False
        print('  %-24s AP_PMUCAN.h=%-6s tool=%-6u %s' % (name, got, value, status))
    return ok


def main():
    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        '..', '..', '..'))
    parser = argparse.ArgumentParser(description='TC1 injector and step 2c stopwatch')
    parser.add_argument('scenario', nargs='?', help='which sequence to send')
    parser.add_argument('--list', action='store_true', help='list the scenarios and exit')
    parser.add_argument('--self-test', action='store_true',
                        help='check the packed message and the windows against the firmware')
    parser.add_argument('--mavlink', default='tcp:127.0.0.1:5762',
                        help='FC MAVLink endpoint (default SITL SERIAL1, the port KGCS would use)')
    parser.add_argument('--interface', default='vcan0',
                        help='SocketCAN interface to watch, or "none"')
    parser.add_argument('--rate', type=float, default=KGCS_RATE_HZ, help='burst rate, Hz')
    parser.add_argument('--count', type=int, default=KGCS_BURST, help='messages per burst')
    parser.add_argument('--cut', type=int, default=2,
                        help='messages sent before a cut-short burst stops')
    parser.add_argument('--wait', type=float, default=6.0,
                        help='pause between Stop and Start, seconds')
    parser.add_argument('--settle', type=float, default=4.0,
                        help='how long to keep watching after the last message')
    parser.add_argument('--link-timeout', type=float, default=60.0,
                        help='how long to wait for the MAVLink endpoint to exist')
    args = parser.parse_args()

    if args.list:
        for name, (_, doc) in sorted(SCENARIOS.items()):
            print('  %-12s %s' % (name, doc))
        return 0

    if args.self_test:
        return 0 if self_test(root) else 1

    if args.scenario not in SCENARIOS:
        parser.error('scenario must be one of: %s (try --list)' % ', '.join(sorted(SCENARIOS)))

    timeline = Timeline()
    inj = Injector(args.mavlink, args.interface, timeline)
    try:
        inj.open(args.link_timeout)
    except RuntimeError as err:
        print(str(err), file=sys.stderr)
        return 1
    except OSError as err:
        print('cannot open %s: %s' % (args.interface, err), file=sys.stderr)
        print('is vcan0 up?  sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up',
              file=sys.stderr)
        return 1

    fn, doc = SCENARIOS[args.scenario]
    timeline.log('PLAN', '%s - %s' % (args.scenario, doc))
    fn(inj, args)

    # -- what happened -----------------------------------------------------------------
    print('')
    started = timeline.first('CAN', 'ENGONOFF CMD=1')
    stopped = timeline.first('CAN', 'ENGONOFF CMD=0')
    first_tc1 = timeline.first('TC1', 'Start burst')
    print('RESULT for "%s":' % args.scenario)
    if started is not None:
        if first_tc1 is not None:
            print('  engine START commanded %.3f s after the Start burst began' % (started - first_tc1))
        else:
            print('  engine START commanded at %.3f s' % started)
    else:
        print('  no engine START was commanded')
    if stopped is not None:
        print('  engine STOP commanded at %.3f s' % stopped)
    for t, kind, msg in timeline.events:
        if kind == 'GCS' and 'refused' in msg:
            print('  FC refusal: %s' % msg)

    expected = EXPECT_START.get(args.scenario)
    if expected is None:
        print('  (no expectation defined for this scenario)')
        return 0
    observed = started is not None
    print('  expected a start: %s   observed: %s   -> %s'
          % (expected, observed, 'AS EXPECTED' if expected == observed else 'NOT AS EXPECTED'))
    if args.interface.lower() == 'none':
        print('  NOTE: CAN was not watched, so "no start" here means "not observed", not "did not happen"')
    return 0 if expected == observed else 1


if __name__ == '__main__':
    sys.exit(main())
