#!/usr/bin/env python3
'''
PNU : a fake SAR on SocketCAN, for exercising AP_PNUSAR in SITL without hardware.

Wire format is sar_frames.py; this file is I/O and the operator console, nothing else.
There is no separate model file, because in SAR Integration Plan step 3b the device has
almost no behaviour: it beats once a second and answers commands.  The observation state
machine belongs to step 3d and lands with the FC side that drives it.

THE SAR TRANSMITS ON ITS OWN, unlike the PMU.  Heart_Beat 0x510 is periodic at 1 Hz and
nothing asks for it, so "going silent" here means ceasing to BEAT, which is what the
FC's 3 s T_HB timeout is written against.

IT IS DESIGNED TO RUN BESIDE fake_pmu.py ON THE SAME vcan0.  That is the point of step
3b: the PMU is 29-bit extended and the SAR is 11-bit standard in 0x400-0x7FF, so each
process filters in the kernel for its own frames and neither sees the other's.

    sudo modprobe vcan
    sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up
    Tools/pnu/sitl_can/fake_sar.py --interface vcan0

Console commands on stdin: silent, resume, status, hb, short, counter N, start-resp,
stop-resp, param-resp, respond on|off, intrude [id], help, quit.  SIGUSR1 toggles
silence, for scripted tests.

AP_FLAKE8_CLEAN
'''

import argparse
import os
import queue
import select
import signal
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sar_frames as icd                # noqa: E402

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_FRAME_FMT = '<IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

TICK_HZ = 100.0


class FakeSAR(object):

    def __init__(self, interface, hb_rate=1.0, respond=True, resp_delay=0.05,
                 counter=0, verbose=False):
        self.interface = interface
        self.verbose = verbose
        self.silent = False
        self.respond = respond
        self.resp_delay = resp_delay
        self.running = True
        self.commands = queue.Queue()

        # ICD 12.1 : the SAR's OWN counter, which the FC must not copy.  --counter can
        # start it anywhere, including a value the FC's Live_Counter will never reach,
        # which is how "do not copy Raw_Counter" is tested rather than assumed.
        self.raw_counter = counter & 0xFFFFFFFF
        self.hb_period = 1.0 / hb_rate if hb_rate > 0.0 else None

        self._t0 = time.monotonic()
        self._next_hb = self._t0
        self._pending = []              # [(due_monotonic, can_id, payload)]
        self._hb_sent = 0
        self._rx_counts = {}            # can_id -> frames received
        self._rtr_seen = 0

        self.sock = None

    def connect(self):
        '''open the SocketCAN interface.  Separate from __init__ so that the framing and
           command handling below can be tested without a vcan interface, which needs
           root to create.'''
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # One filter covering 11-bit ids 0x400-0x7FF and nothing else.  Every id in that
        # range has bit 10 set, and CAN_EFF_FLAG is included in the mask so the PMU's
        # 29-bit frames on the same wire never reach this process at all.
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                             struct.pack('=II', 0x400, 0x400 | CAN_EFF_FLAG))
        self.sock.bind((self.interface,))

    def log(self, msg):
        print('[%8.3f] %s' % (time.monotonic() - self._t0, msg), flush=True)

    # -- CAN I/O ----------------------------------------------------------------------

    def send(self, can_id, payload):
        '''11-bit STANDARD data frame - no CAN_EFF_FLAG, ICD section 4'''
        frame = struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, len(payload),
                            payload.ljust(8, b'\x00'))
        try:
            self.sock.send(frame)
        except OSError as err:
            self.log('TX failed on 0x%03X: %s' % (can_id, err))

    def handle_frame(self, raw):
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        if can_id & CAN_ERR_FLAG:
            return
        if can_id & CAN_EFF_FLAG:
            return                      # the PMU's; the kernel filter should have eaten it
        frame_id = can_id & CAN_SFF_MASK
        if can_id & CAN_RTR_FLAG:
            # The SAR protocol has no remote frames at all.  Counted and reported,
            # because an RTR in our id range means somebody is polling a device that
            # does not answer polls.
            self._rtr_seen += 1
            self.log('RX RTR on 0x%03X - the SAR ICD has no remote frames' % frame_id)
            return
        self.handle_data(frame_id, dlc, data[:dlc])

    def handle_data(self, frame_id, dlc, data):
        first = frame_id not in self._rx_counts
        self._rx_counts[frame_id] = self._rx_counts.get(frame_id, 0) + 1

        # The first frame on each id is always logged and the rest are counted.  The
        # navigation bundle is 90 frames a second once 3e exists, and a line each would
        # bury everything else; the first one is the event worth seeing.
        if first or self.verbose:
            self.log('RX 0x%03X %s dlc %u %s%s'
                     % (frame_id, icd.name_of(frame_id), dlc, data.hex(),
                        '' if self.verbose else '  (further frames on this id are counted, not logged)'))

        if frame_id == icd.START_CMD:
            self.answer(frame_id, icd.START_RESP, icd.build_start_response(), dlc)
        elif frame_id == icd.STOP_CMD:
            self.answer(frame_id, icd.STOP_RESP, icd.build_stop_response(), dlc)
        elif frame_id == icd.MODE_CMD:
            # ICD 13 : 0x503's payload is TBD and 0x523's trigger is TBD, so there is no
            # defined answer to invent.  Noted and ignored.
            self.log('RX 0x503 Observation_Mode_Command - payload TBD, no response defined')

    def answer(self, cmd_id, resp_id, payload, dlc):
        '''queue the ICD response to a command, or decline to - ICD 12.2 and 12.3'''
        if dlc != icd.CMD_DLC:
            self.log('0x%03X has DLC %u, ICD says %u - not answered' % (cmd_id, dlc, icd.CMD_DLC))
            return
        if not self.respond:
            # ICD 14.1 state 4 : the FC's 2 s window expiring is a real case and the only
            # way to test it is a device that stays quiet.  3d measures that window.
            self.log('0x%03X received, responses are OFF - no 0x%03X' % (cmd_id, resp_id))
            return
        self._pending.append((time.monotonic() + self.resp_delay, resp_id, payload))

    def send_heartbeat(self):
        '''ICD 12.1 : 0x510, DLC 4, Raw_Counter increasing from zero'''
        self.send(icd.HEARTBEAT, icd.build_heartbeat(self.raw_counter))
        self._hb_sent += 1
        self.log('TX Heart_Beat Raw_Counter=%u' % self.raw_counter)
        self.raw_counter = (self.raw_counter + 1) & 0xFFFFFFFF

    def send_short_heartbeat(self):
        '''a 3-byte 0x510.  ICD 3.3 : a validation failure must not increment the FC's
           Live_Counter or refresh freshness, and the FC must say so rather than parse
           the short payload from the same offsets - PNU-ISSUE(D6).

           The counter is deliberately NOT advanced, so the FC's Live_Counter and this
           counter stay directly comparable across the test.'''
        self.send(icd.HEARTBEAT, icd.build_heartbeat(self.raw_counter)[:3])
        self.log('TX Heart_Beat with DLC 3 - the FC must reject it')

    def tick(self, now):
        '''one 100 Hz pass: the heartbeat schedule and any due response'''
        if self.hb_period is not None and not self.silent and now >= self._next_hb:
            # scheduled from the deadline, not from now, so a late tick does not make
            # the beat drift slower than 1 Hz
            self._next_hb += self.hb_period
            if self._next_hb < now:
                self._next_hb = now + self.hb_period
            self.send_heartbeat()

        if not self._pending:
            return
        due = [item for item in self._pending if item[0] <= now]
        self._pending = [item for item in self._pending if item[0] > now]
        for _, can_id, payload in due:
            self.send(can_id, payload)
            self.log('TX 0x%03X %s' % (can_id, icd.name_of(can_id)))

    # -- console ----------------------------------------------------------------------

    def console_thread(self):
        '''EOF means "quit" only from a terminal - the fake PMU's lesson: with stdin
           redirected or closed, EOF arrives at once and means "there is no console
           here", not "shut down".'''
        interactive = sys.stdin is not None and sys.stdin.isatty()
        for line in sys.stdin:
            self.commands.put(line.strip())
        if interactive:
            self.commands.put('quit')
        else:
            self.log('no console on stdin - SIGUSR1 toggles silence, SIGINT or SIGTERM quits')

    def drain_console(self):
        while True:
            try:
                line = self.commands.get_nowait()
            except queue.Empty:
                return
            self.do_command(line)

    def do_command(self, line):
        parts = line.split()
        if not parts:
            return
        cmd = parts[0].lower()
        arg = parts[1].lower() if len(parts) > 1 else ''

        if cmd in ('silent', 'quiet'):
            self.set_silent(True)
        elif cmd in ('resume', 'live'):
            self.set_silent(False)
        elif cmd == 'toggle-silent':
            self.set_silent(not self.silent)
        elif cmd == 'status':
            self.log(self.summary())
        elif cmd == 'hb':
            self.send_heartbeat()
        elif cmd == 'short':
            self.send_short_heartbeat()
        elif cmd == 'counter':
            self.set_counter(arg)
        elif cmd == 'start-resp':
            self.send(icd.START_RESP, icd.build_start_response())
            self.log('TX 0x521 Observation_Start_Response, unsolicited')
        elif cmd == 'stop-resp':
            self.send(icd.STOP_RESP, icd.build_stop_response())
            self.log('TX 0x522 Observation_Stop_Response, unsolicited')
        elif cmd == 'param-resp':
            self.send(icd.PARAM_RESP, icd.build_param_response())
            self.log('TX 0x523 Observation_Param_Response, eight opaque bytes')
        elif cmd == 'respond':
            self.respond = not arg.startswith('off')
            self.log('command responses are %s' % ('ON' if self.respond else 'OFF'))
        elif cmd == 'intrude':
            self.intrude(arg)
        elif cmd in ('quit', 'exit'):
            self.running = False
        elif cmd == 'help':
            self.log('silent | resume | status | hb | short | counter N | start-resp | '
                     'stop-resp | param-resp | respond on|off | intrude [id] | quit')
        else:
            self.log('unknown command "%s", try help' % cmd)

    def set_counter(self, arg):
        '''ICD 3.3 : the FC must not infer a reboot from a Raw_Counter decrease, so this
           deliberately allows one.'''
        try:
            self.raw_counter = int(arg, 0) & 0xFFFFFFFF
        except ValueError:
            self.log('counter needs a number')
            return
        self.log('Raw_Counter set to %u' % self.raw_counter)

    def intrude(self, arg):
        '''transmit on an FC -> SAR id, which no SAR may ever do

           AP_PNUSAR leaves those ids UNCLAIMED on receive on purpose, so the driver
           reports them as unclaimed traffic rather than hiding a second talker.  This
           is how that is proved rather than argued.'''
        try:
            can_id = int(arg, 0) if arg else icd.NAV_FIRST
        except ValueError:
            self.log('intrude needs an id, default 0x%03X' % icd.NAV_FIRST)
            return
        self.send(can_id, b'\x00' * icd.NAV_DLC)
        self.log('TX 0x%03X %s - an FC -> SAR id, the FC should report it unclaimed'
                 % (can_id, icd.name_of(can_id)))

    def set_silent(self, silent):
        if silent == self.silent:
            return
        self.silent = silent
        self.log('*** SAR IS NOW %s ***'
                 % ('SILENT - no heartbeats, FC should declare Disconnected after %u ms'
                    % icd.HB_TIMEOUT_MS if silent else 'BEATING'))

    def summary(self):
        received = ', '.join('0x%03X %s x%u' % (can_id, icd.name_of(can_id), count)
                             for can_id, count in sorted(self._rx_counts.items())) or 'nothing'
        return ('silent=%s  responses=%s  heartbeats sent %u  next Raw_Counter %u  '
                'RTRs %u\n            received: %s'
                % (self.silent, 'on' if self.respond else 'off', self._hb_sent,
                   self.raw_counter, self._rtr_seen, received))

    # -- main loop --------------------------------------------------------------------

    def run(self):
        self.log('fake SAR on %s, Heart_Beat 0x%03X every %s'
                 % (self.interface, icd.HEARTBEAT,
                    '%.2f s' % self.hb_period if self.hb_period else 'never (--hb-rate 0)'))
        last = time.monotonic()
        period = 1.0 / TICK_HZ

        while self.running:
            timeout = max(0.0, (last + period) - time.monotonic())
            ready, _, _ = select.select([self.sock], [], [], timeout)
            if ready:
                try:
                    raw = self.sock.recv(CAN_FRAME_SIZE)
                except OSError as err:
                    self.log('RX failed: %s' % err)
                    continue
                if len(raw) == CAN_FRAME_SIZE:
                    self.handle_frame(raw)

            now = time.monotonic()
            if now - last >= period:
                last = now
            self.tick(now)
            self.drain_console()

        self.log('stopped')
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description='fake SAR on SocketCAN for SITL')
    parser.add_argument('--interface', default='vcan0', help='SocketCAN interface (default vcan0)')
    parser.add_argument('--hb-rate', type=float, default=1.0,
                        help='Heart_Beat rate in Hz, ICD says 1.0; 0 disables it entirely')
    parser.add_argument('--counter', type=int, default=0,
                        help='initial Raw_Counter - the FC must not copy it into Live_Counter')
    parser.add_argument('--resp-delay', type=float, default=0.05,
                        help='delay before answering a command, seconds (FC window is 2 s)')
    parser.add_argument('--no-respond', action='store_true',
                        help='never answer 0x501/0x502, to exercise the FC 2 s window')
    parser.add_argument('--silent', action='store_true', help='start without beating')
    parser.add_argument('--verbose', action='store_true', help='log every received frame')
    args = parser.parse_args()

    sar = FakeSAR(args.interface, hb_rate=args.hb_rate, respond=not args.no_respond,
                  resp_delay=args.resp_delay, counter=args.counter, verbose=args.verbose)
    try:
        sar.connect()
    except OSError as err:
        print('cannot open %s: %s' % (args.interface, err), file=sys.stderr)
        print('try: sudo modprobe vcan && sudo ip link add %s type vcan && sudo ip link set %s up'
              % (args.interface, args.interface), file=sys.stderr)
        return 1

    if args.silent:
        sar.set_silent(True)

    signal.signal(signal.SIGUSR1, lambda *_: sar.commands.put('toggle-silent'))
    signal.signal(signal.SIGTERM, lambda *_: sar.commands.put('quit'))
    threading.Thread(target=sar.console_thread, daemon=True).start()

    try:
        sar.run()
    except KeyboardInterrupt:
        sar.log('interrupted')
    return 0


if __name__ == '__main__':
    sys.exit(main())
