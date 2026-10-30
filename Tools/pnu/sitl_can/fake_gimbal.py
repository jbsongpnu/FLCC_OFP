#!/usr/bin/env python3
'''
PNU : three fake CubeMars GLII gimbal motors on SocketCAN, for exercising AP_PNUGimbal
in SITL without hardware.

Wire format is gimbal_frames.py and the motion is gimbal_model.py; this file is I/O and
the operator console, nothing else.

WHY THIS EXISTS.  AP_PNUGimbal has never had a motor turn.  The protocol backend, the
AP_Mount backend, the lifecycle state machine and the Appendix 1 homing sequence are all
written and none of them has been run against a device - and homing is the most hazardous
routine in the driver, because it deliberately drives a bulky SAR payload into three sets
of mechanical hard stops with the mechanical clamp disabled.  Rehearsing that here costs
nothing; rehearsing it on the real mount costs the payload.

THE FEEDBACK TRIGGER IS AN ASSUMPTION, NOT A MEASUREMENT.  The manual never says what
makes a motor send a feedback frame - PNUGimbal_CAN_Protocol_Type_CubeMars.md sec 1 and
sec 9.6 both record the gap and say to measure it.  So --fb-mode selects the behaviour
and the fake says loudly which one it is using, every time it starts and in every status
line.  Do not read a working SITL run as evidence about the real motors.

IT IS DESIGNED TO RUN BESIDE fake_pmu.py AND fake_sar.py ON THE SAME vcan0.  The PMU is
29-bit extended, the gimbal is 11-bit standard below 0x300 and the SAR is 11-bit standard
in 0x400-0x7FF, so each process filters in the kernel for its own frames.

    sudo modprobe vcan
    sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up
    Tools/pnu/sitl_can/fake_gimbal.py --interface vcan0

Console commands on stdin: status, axes, fault AXIS CODE, clear [AXIS], temp AXIS D M,
pos AXIS DEG, travel AXIS LOW HIGH, stiffness X, fb per-command|periodic N|off,
silent, resume, help, quit.  SIGUSR1 toggles silence, for scripted tests.

AP_FLAKE8_CLEAN
'''

import argparse
import math
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

import gimbal_frames as icd                                     # noqa: E402
from gimbal_model import GimbalModel, DEFAULT_TRAVEL_DEG, DEFAULT_STOP_STIFFNESS  # noqa: E402

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_FRAME_FMT = '<IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

TICK_HZ = 200.0         # finer than the driver's 100 Hz so a command is never missed


class FakeGimbal(object):

    def __init__(self, interface, mode=icd.MODE_MIT, master_id=0, motor_ids=(1, 2, 3),
                 travel_deg=DEFAULT_TRAVEL_DEG, limits=None,
                 stop_stiffness=DEFAULT_STOP_STIFFNESS,
                 fb_mode='per-command', fb_rate=0.0, verbose=False):
        self.interface = interface
        self.verbose = verbose
        self.silent = False
        self.running = True
        self.commands = queue.Queue()

        # The configured mode.  sec 1 : "A command's mode bits must match the configured
        # mode."  A mismatch is undefined on real hardware, so the fake reports it rather
        # than guessing - that is a configuration fault worth seeing in SITL.
        self.mode = mode
        self.master_id = master_id
        self.model = GimbalModel(motor_ids, travel_deg, limits, stop_stiffness)

        self.fb_mode = fb_mode
        self.fb_rate = fb_rate
        self._next_fb = 0.0

        self._t0 = time.monotonic()
        self._last_step = self._t0
        self._rx_counts = {}
        self._bad_mode_seen = 0
        self._unknown_id_seen = 0
        self._tx_frames = 0

        self.sock = None

    # ---- I/O ----------------------------------------------------------------------
    def connect(self):
        '''open the SocketCAN interface.  Separate from __init__ so that the framing and
           command handling below can be tested without a vcan interface, which needs
           root to create.'''
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Standard 11-bit ids below 0x400 only.  Every gimbal command id is 0x000-0x2FF,
        # so this one filter drops the PMU's 29-bit frames and the SAR's 0x400-0x7FF
        # without this process ever waking for them.
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                             struct.pack('=II', 0x000, 0x400 | CAN_EFF_FLAG))
        self.sock.bind((self.interface,))

    def log(self, msg):
        print('[%8.3f] %s' % (time.monotonic() - self._t0, msg), flush=True)

    def send(self, can_id, payload):
        if self.silent or self.sock is None:
            return
        frame = struct.pack(CAN_FRAME_FMT, can_id, len(payload), bytes(payload))
        try:
            self.sock.send(frame)
            self._tx_frames += 1
        except OSError as err:
            self.log('TX failed on 0x%03X: %s' % (can_id, err))

    # ---- receive ------------------------------------------------------------------
    def handle_frame(self, raw):
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        if can_id & (CAN_ERR_FLAG | CAN_EFF_FLAG):
            return
        if can_id & CAN_RTR_FLAG:
            # The GLII protocol has no remote frames; the PMU's RTR polls are extended
            # and never reach here, so an RTR in this range is somebody else's mistake.
            self.log('RX RTR on 0x%03X - the GLII protocol has no remote frames'
                     % (can_id & CAN_SFF_MASK))
            return
        self.handle_command(can_id & CAN_SFF_MASK, dlc, data[:dlc])

    def handle_command(self, can_id, dlc, data):
        mode, motor_id = icd.split_command_id(can_id)
        axis = self.model.by_motor_id.get(motor_id)
        if axis is None:
            self._unknown_id_seen += 1
            if self._unknown_id_seen == 1 or self.verbose:
                self.log('RX 0x%03X addresses motor %u, which this gimbal does not have'
                         % (can_id, motor_id))
            return

        first = can_id not in self._rx_counts
        self._rx_counts[can_id] = self._rx_counts.get(can_id, 0) + 1

        if mode != self.mode:
            # sec 1 : the command's mode bits must match the configured mode.  Reported
            # once and then counted, because a misconfigured CAN_Dn_PNU_MODE would
            # otherwise look exactly like a dead motor.
            self._bad_mode_seen += 1
            if self._bad_mode_seen == 1 or self.verbose:
                self.log('RX 0x%03X is %s but these motors are configured %s'
                         ' - undefined on real hardware, ignored here'
                         % (can_id, icd.MODE_NAMES.get(mode, 'mode %u' % mode),
                            icd.MODE_NAMES[self.mode]))
            return

        action = icd.universal_command(data)
        if action is not None:
            self.handle_universal(axis, action)
            return

        if not self.decode_motion(axis, mode, dlc, data):
            return

        axis.cmd_count += 1
        if first or self.verbose:
            self.log('RX 0x%03X %s %s dlc %u %s%s'
                     % (can_id, axis.name, icd.MODE_NAMES[mode], dlc, bytes(data).hex(),
                        '' if self.verbose else
                        '  (further frames on this id are counted, not logged)'))

        if self.fb_mode == 'per-command':
            self.send_feedback(axis)

    def decode_motion(self, axis, mode, dlc, data):
        '''store the decoded command on the axis, or reject the frame'''
        want = {icd.MODE_MIT: icd.MIT_DLC,
                icd.MODE_POSVEL: icd.POSVEL_DLC,
                icd.MODE_VEL: icd.VEL_DLC}[mode]
        if dlc != want:
            self.log('RX %s command with DLC %u, the manual says %u - rejected'
                     % (icd.MODE_NAMES[mode], dlc, want))
            return False

        if mode == icd.MODE_MIT:
            p, v, kp, kd, t = icd.decode_mit(data, self.model.limits)
            axis.cmd = ('mit', p, v, kp, kd, t)
        elif mode == icd.MODE_POSVEL:
            p, v_limit = icd.decode_posvel(data)
            axis.cmd = ('posvel', p, v_limit)
        else:
            axis.cmd = ('vel', icd.decode_vel(data))
        return True

    def handle_universal(self, axis, action):
        self.log('RX universal "%s" for %s' % (icd.UNIVERSAL_NAMES[action], axis.name))
        if action == icd.UNIVERSAL_ENABLE:
            axis.enable(True)
        elif action == icd.UNIVERSAL_DISABLE:
            axis.enable(False)
        elif action == icd.UNIVERSAL_SET_ZERO:
            axis.set_zero()
            self.log('    %s zero set at mech %+.2f deg; it now reports %+.2f deg'
                     % (axis.name, math.degrees(axis.pos_rad),
                        math.degrees(axis.reported_rad())))
        elif action == icd.UNIVERSAL_CLEAR_ERRORS:
            axis.set_status(icd.STATUS_ENABLED if axis.enabled else icd.STATUS_DISABLED)
        # sec 3 : the universal commands have no documented acknowledgement, so none is
        # invented.  Whether they produce a feedback frame is part of the same unknown
        # as every other feedback trigger, so they follow --fb-mode like anything else.
        if self.fb_mode == 'per-command':
            self.send_feedback(axis)

    # ---- transmit -----------------------------------------------------------------
    def send_feedback(self, axis):
        self.send(self.master_id, axis.feedback())

    def tick(self, now):
        dt = now - self._last_step
        self._last_step = now
        if dt > 0.0:
            self.model.step(dt)

        if self.fb_mode == 'periodic' and self.fb_rate > 0.0:
            if now >= self._next_fb:
                # drift-free: advance from the scheduled instant, not from now
                period = 1.0 / self.fb_rate
                self._next_fb = max(now, self._next_fb + period)
                for axis in self.model.axes:
                    self.send_feedback(axis)

    # ---- console ------------------------------------------------------------------
    def console_thread(self):
        for line in sys.stdin:
            self.commands.put(line.strip())
        if not sys.stdin.isatty():
            # EOF on a pipe means "no console", not "quit" - the lesson from the fake
            # PMU, which used to exit 0.01 s after starting under a test harness.
            self.log('no console on stdin - SIGUSR1 toggles silence, SIGINT or SIGTERM quits')
            return
        self.running = False

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
        args = parts[1:]

        if cmd == 'status':
            self.log(self.summary())
        elif cmd == 'axes':
            for axis in self.model.axes:
                self.log('    ' + axis.describe())
        elif cmd == 'fault':
            self.set_fault(args)
        elif cmd == 'clear':
            self.clear_faults(args)
        elif cmd == 'temp':
            self.set_temp(args)
        elif cmd == 'pos':
            self.set_pos(args)
        elif cmd == 'travel':
            self.set_travel(args)
        elif cmd == 'stiffness':
            self.set_stiffness(args)
        elif cmd == 'fb':
            self.set_fb_mode(args)
        elif cmd in ('silent', 'quiet'):
            self.set_silent(True)
        elif cmd in ('resume', 'live'):
            self.set_silent(False)
        elif cmd == 'toggle-silent':
            self.set_silent(not self.silent)
        elif cmd in ('quit', 'exit'):
            self.running = False
        elif cmd == 'help':
            self.log('status | axes | fault AXIS CODE | clear [AXIS] | temp AXIS D M | '
                     'pos AXIS DEG | travel AXIS LOW HIGH | stiffness X | '
                     'fb per-command|periodic N|off | silent | resume | quit')
        else:
            self.log('unknown command "%s", try help' % cmd)

    def _axis_args(self, args, want, usage):
        if len(args) < want:
            self.log('usage: %s' % usage)
            return None
        axis = self.model.axis_by_name(args[0])
        if axis is None:
            self.log('no axis "%s" - try roll, pitch or yaw' % args[0])
            return None
        return axis

    def set_fault(self, args):
        axis = self._axis_args(args, 2, 'fault AXIS CODE   (CODE in hex, e.g. A for overcurrent)')
        if axis is None:
            return
        try:
            code = int(args[1], 16)
        except ValueError:
            self.log('CODE must be hex, e.g. 8 9 A B C D E')
            return
        axis.set_status(code)
        self.log('%s now reports status 0x%X (%s)' % (axis.name, code, icd.status_name(code)))
        if code != icd.STATUS_ENABLED:
            self.log('    Appendix 1 item 6: homing must ABORT on any status that is not 1')

    def clear_faults(self, args):
        axes = self.model.axes if not args else [self.model.axis_by_name(args[0])]
        for axis in axes:
            if axis is None:
                continue
            axis.set_status(icd.STATUS_ENABLED if axis.enabled else icd.STATUS_DISABLED)
        self.log('status reset on %s' % ('all axes' if not args else args[0]))

    def set_temp(self, args):
        axis = self._axis_args(args, 3, 'temp AXIS DRIVER_C MOTOR_C')
        if axis is None:
            return
        axis.driver_temp_c = int(args[1])
        axis.motor_temp_c = int(args[2])
        self.log('%s temperatures now %d / %d C' % (axis.name, axis.driver_temp_c, axis.motor_temp_c))

    def set_pos(self, args):
        '''teleport an axis, to start a run from somewhere interesting'''
        axis = self._axis_args(args, 2, 'pos AXIS DEG')
        if axis is None:
            return
        axis.pos_rad = math.radians(float(args[1]))
        axis.vel_rads = 0.0
        self.log('%s moved to mech %+.2f deg (reports %+.2f deg)'
                 % (axis.name, math.degrees(axis.pos_rad), math.degrees(axis.reported_rad())))

    def set_travel(self, args):
        '''move the mechanical stops - the way to make homing fail on purpose'''
        axis = self._axis_args(args, 3, 'travel AXIS LOW_DEG HIGH_DEG')
        if axis is None:
            return
        axis.stop_low = math.radians(float(args[1]))
        axis.stop_high = math.radians(float(args[2]))
        self.log('%s stops now %+.1f / %+.1f deg, travel %.1f deg'
                 % (axis.name, float(args[1]), float(args[2]),
                    float(args[2]) - float(args[1])))

    def set_stiffness(self, args):
        if not args:
            self.log('usage: stiffness NM_PER_RAD')
            return
        value = float(args[0])
        for axis in self.model.axes:
            axis.stop_stiffness = value
        self.log('stop stiffness now %.1f Nm/rad on every axis - a TEST KNOB, not a '
                 'measurement' % value)

    def set_fb_mode(self, args):
        if not args:
            self.log('usage: fb per-command | periodic RATE_HZ | off')
            return
        mode = args[0].lower()
        if mode == 'periodic':
            self.fb_rate = float(args[1]) if len(args) > 1 else 50.0
            self._next_fb = time.monotonic()
        self.fb_mode = mode
        self.log('feedback mode now %s - STILL AN ASSUMPTION, the manual specifies no '
                 'trigger' % self.describe_fb())

    def set_silent(self, silent):
        self.silent = silent
        self.log('*** GIMBAL FEEDBACK IS NOW %s ***'
                 % ('SILENT - every axis should go stale' if silent else 'LIVE AGAIN'))

    def describe_fb(self):
        if self.fb_mode == 'periodic':
            return 'periodic %.1f Hz per axis' % self.fb_rate
        if self.fb_mode == 'off':
            return 'off'
        return 'one frame per command received'

    def summary(self):
        received = ', '.join('0x%03X x%u' % (can_id, count)
                             for can_id, count in sorted(self._rx_counts.items())) or 'nothing'
        return ('mode %s  master id 0x%03X  feedback %s  silent=%s  TX %u\n'
                '            wrong-mode frames %u, unknown motor ids %u\n'
                '            received: %s'
                % (icd.MODE_NAMES[self.mode], self.master_id, self.describe_fb(),
                   self.silent, self._tx_frames, self._bad_mode_seen,
                   self._unknown_id_seen, received))

    # ---- main loop ----------------------------------------------------------------
    def run(self):
        self.log('fake gimbal on %s: 3 x CubeMars GLII, motor ids %s, master id 0x%03X'
                 % (self.interface,
                    '/'.join(str(a.motor_id) for a in self.model.axes), self.master_id))
        self.log('configured mode %s; stops %s deg'
                 % (icd.MODE_NAMES[self.mode],
                    ', '.join('%s %+.0f/%+.0f' % (a.name, math.degrees(a.stop_low),
                                                  math.degrees(a.stop_high))
                              for a in self.model.axes)))
        self.log('FEEDBACK IS SIMULATED %s - the manual does not specify the trigger '
                 '(sec 1, sec 9.6).' % self.describe_fb())
        self.log('    This is an ASSUMPTION, not a measurement.  Bench the real motors '
                 'before trusting any rate seen here.')

        period = 1.0 / TICK_HZ
        last = time.monotonic()
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


def parse_travel(text):
    '''"30,30,20" or "-30:30,-30:30,-20:20" - symmetric or explicit, per axis'''
    out = []
    for part in text.split(','):
        if ':' in part:
            low, high = part.split(':')
            out.append((float(low), float(high)))
        else:
            out.append(float(part))
    if len(out) != 3:
        raise argparse.ArgumentTypeError('--travel needs three axes: roll,pitch,yaw')
    return tuple(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--interface', default='vcan0')
    parser.add_argument('--mode', default='mit', choices=('mit', 'posvel', 'vel'),
                        help='the mode the motors were configured for with the PC utility')
    parser.add_argument('--master-id', type=lambda s: int(s, 0), default=0,
                        help='feedback arbitration id, must match CAN_Dn_PNU_MSTID')
    parser.add_argument('--motor-ids', default='1,2,3',
                        help='roll,pitch,yaw CAN ids; keep them 1-15, see sec 7 ID conflict')
    parser.add_argument('--travel', type=parse_travel, default=DEFAULT_TRAVEL_DEG,
                        help='mechanical stops in degrees, "30,30,20" or "-30:30,..."')
    parser.add_argument('--stop-stiffness', type=float, default=DEFAULT_STOP_STIFFNESS,
                        help='Nm/rad at the hard stops; a TEST KNOB, nobody has measured this mount')
    parser.add_argument('--fb-mode', default='per-command',
                        choices=('per-command', 'periodic', 'off'),
                        help='what triggers a feedback frame - UNSPECIFIED by the manual')
    parser.add_argument('--fb-rate', type=float, default=50.0,
                        help='frames per second per axis when --fb-mode periodic')
    parser.add_argument('--p-max', type=float, default=12.5, help='must match CAN_Dn_PNU_PMAX')
    parser.add_argument('--v-max', type=float, default=200.0, help='must match CAN_Dn_PNU_VMAX')
    parser.add_argument('--kp-max', type=float, default=500.0, help='must match CAN_Dn_PNU_KPMAX')
    parser.add_argument('--kd-max', type=float, default=5.0, help='must match CAN_Dn_PNU_KDMAX')
    parser.add_argument('--t-max', type=float, default=10.0, help='must match CAN_Dn_PNU_TMAX')
    parser.add_argument('--verbose', action='store_true', help='log every frame')
    args = parser.parse_args()

    modes = {'mit': icd.MODE_MIT, 'posvel': icd.MODE_POSVEL, 'vel': icd.MODE_VEL}
    motor_ids = tuple(int(x, 0) for x in args.motor_ids.split(','))
    limits = icd.Limits(args.p_max, args.v_max, args.kp_max, args.kd_max, args.t_max)

    gimbal = FakeGimbal(args.interface, mode=modes[args.mode], master_id=args.master_id,
                        motor_ids=motor_ids, travel_deg=args.travel, limits=limits,
                        stop_stiffness=args.stop_stiffness, fb_mode=args.fb_mode,
                        fb_rate=args.fb_rate, verbose=args.verbose)

    signal.signal(signal.SIGUSR1, lambda *_: gimbal.set_silent(not gimbal.silent))
    signal.signal(signal.SIGTERM, lambda *_: setattr(gimbal, 'running', False))

    try:
        gimbal.connect()
    except OSError as err:
        print('cannot open %s: %s' % (args.interface, err), file=sys.stderr)
        return 1

    threading.Thread(target=gimbal.console_thread, daemon=True).start()
    try:
        gimbal.run()
    except KeyboardInterrupt:
        gimbal.log('interrupted')
    return 0


if __name__ == '__main__':
    sys.exit(main())
