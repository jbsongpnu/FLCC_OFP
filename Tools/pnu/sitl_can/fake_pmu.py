#!/usr/bin/env python3
'''
PNU : a fake PMU on SocketCAN, for exercising AP_PMUCAN in SITL without hardware.

Engine behaviour is pmu_model.py, wire format is pmu_frames.py.  This file is I/O and
the operator console, nothing else.

THE PMU NEVER TRANSMITS ON ITS OWN.  Status is strictly RTR-request / Data-response: the
FC sends a Remote Frame on one of the five status ids and the PMU answers with a Data
Frame on the same id, DLC 8.  So does this.  "Going silent" therefore means ceasing to
ANSWER, which is the real failure mode - not a stopped broadcast.

Raw AF_CAN sockets are used rather than python-can, so there is no dependency to install
and RTR and extended-id handling stay explicit.  pymavlink is needed only for automatic
takeoff detection, is imported lazily, and can be left out entirely - the console's
"takeoff" does the same job by hand.

    sudo modprobe vcan
    sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up
    Tools/pnu/sitl_can/fake_pmu.py --interface vcan0

Console commands on stdin: silent, resume, status, start, stop, choke open|close,
takeoff, land, help, quit.  SIGUSR1 toggles silence, for scripted tests.

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

import pmu_frames as icd                # noqa: E402
from pmu_model import PMUModel          # noqa: E402

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FMT = '<IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

TICK_HZ = 100.0


class FakePMU(object):

    def __init__(self, interface, verbose=False):
        self.interface = interface
        self.verbose = verbose
        self.silent = False
        self.running = True
        self.commands = queue.Queue()
        self.model = PMUModel(log=self.log)
        self.watcher = None             # set by main() when MAVLink is enabled

        self._t0 = time.monotonic()
        self._rtr_seen = 0
        self._rtr_answered = 0
        self._cmd_seen = 0

        self.sock = None

    def connect(self):
        '''open the SocketCAN interface.  Separate from __init__ so that the framing and
           command handling below can be tested without a vcan interface, which needs
           root to create.'''
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # one filter covering 0x10000000 - 0x100001FF: our five commands and five status
        # ids, and nothing a gimbal or a SAR sharing the bus would send
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                             struct.pack('=II', 0x10000000 | CAN_EFF_FLAG, 0x1FFFFE00 | CAN_EFF_FLAG))
        self.sock.bind((self.interface,))

    def log(self, msg):
        print('[%8.3f] %s' % (time.monotonic() - self._t0, msg), flush=True)

    # -- CAN I/O ----------------------------------------------------------------------

    def send(self, can_id, payload):
        frame = struct.pack(CAN_FRAME_FMT, can_id | CAN_EFF_FLAG, len(payload), payload)
        try:
            self.sock.send(frame)
        except OSError as err:
            self.log('TX failed on 0x%08X: %s' % (can_id, err))

    def handle_frame(self, raw):
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        if can_id & CAN_ERR_FLAG:
            return
        frame_id = can_id & CAN_EFF_MASK
        if can_id & CAN_RTR_FLAG:
            self.handle_rtr(frame_id)
        else:
            self.handle_command(frame_id, dlc, data[:dlc])

    def handle_rtr(self, frame_id):
        if frame_id not in icd.STATUS_IDS:
            return
        self._rtr_seen += 1
        if self.silent:
            return
        self._rtr_answered += 1
        self.send(frame_id, icd.build(frame_id, self.model))

    def handle_command(self, frame_id, dlc, data):
        handlers = {
            icd.BATCTRL: self.model.cmd_battery,
            icd.ENGONOFF: self.model.cmd_engine_onoff,
            icd.ENGMANUAL: self.model.cmd_manual,
            icd.ENGPCL: self.model.cmd_throttle,
            icd.ENGCHK: self.model.cmd_choke,
        }
        if frame_id not in handlers:
            return
        if dlc != icd.COMMAND_DLC:
            self.log('command 0x%08X has DLC %u, ICD says %u - ignored' % (frame_id, dlc, icd.COMMAND_DLC))
            return
        self._cmd_seen += 1
        value = struct.unpack('<I', data)[0]
        if frame_id == icd.ENGONOFF:
            # logged unconditionally: this is the frame step 2c's interlock timing is
            # measured against, and the timestamp on the left is the measurement
            self.log('RX ENGONOFF CMD=%u' % value)
        elif self.verbose:
            self.log('RX 0x%08X CMD=%u' % (frame_id, value))
        handlers[frame_id](value)

    # -- console ----------------------------------------------------------------------

    def console_thread(self):
        for line in sys.stdin:
            self.commands.put(line.strip())
        self.commands.put('quit')

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
            self.log(self.model.summary())
            self.log('silent=%s  RTR seen %u answered %u  commands %u'
                     % (self.silent, self._rtr_seen, self._rtr_answered, self._cmd_seen))
            # the MAVLink side is reported too: when takeoff is not detected, this is
            # where the answer is, and guessing at it once was enough
            self.log(self.watcher.summary() if self.watcher is not None else 'MAVLink: disabled')
        elif cmd == 'start':
            self.model.cmd_engine_onoff(1)
        elif cmd == 'stop':
            self.model.cmd_engine_onoff(0)
        elif cmd == 'choke':
            self.model.cmd_choke(1 if arg.startswith('cl') else 0)
        elif cmd == 'takeoff':
            self.model.set_in_air(True)
        elif cmd == 'land':
            self.model.set_in_air(False)
        elif cmd in ('quit', 'exit'):
            self.running = False
        elif cmd == 'help':
            self.log('silent | resume | status | start | stop | choke open|close | takeoff | land | quit')
        else:
            self.log('unknown command "%s", try help' % cmd)

    def set_silent(self, silent):
        if silent == self.silent:
            return
        self.silent = silent
        self.log('*** PMU IS NOW %s ***'
                 % ('SILENT - RTRs will not be answered' if silent else 'ANSWERING'))

    # -- main loop --------------------------------------------------------------------

    def run(self):
        self.log('fake PMU on %s, answering RTRs on %u status ids' % (self.interface, len(icd.STATUS_IDS)))
        self.log(self.model.summary())
        last = time.monotonic()
        period = 1.0 / TICK_HZ
        warned_no_vehicle = False

        while self.running:
            if (not warned_no_vehicle and self.watcher is not None
                    and self.watcher.sysid is None and time.monotonic() - self._t0 > 15.0):
                warned_no_vehicle = True
                self.log('MAVLink: no vehicle telemetry after 15 s - takeoff will NOT be detected. '
                         'Check --mavlink, or use the "takeoff" command')
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
                self.model.update(now - last)
                last = now
            self.drain_console()

        self.log('stopped')
        self.sock.close()


# MAVLink enumerants used below, so that the magic numbers are named once
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8                   # what a GCS or a peripheral reports
LANDED_STATE_UNDEFINED = 0
LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2
LANDED_STATE_TAKEOFF = 3
LANDED_STATE_LANDING = 4


# Messages only a vehicle sends.  A ground station sends HEARTBEAT and commands, never
# these, so whoever sends one IS the vehicle - which is how the system id is identified.
VEHICLE_ONLY_MESSAGES = ('GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE')

# How often to re-ask a system for telemetry while no vehicle has been identified.
STREAM_REQUEST_PERIOD_S = 5.0
STREAM_REQUEST_HZ = 2.0


def is_vehicle_heartbeat(msg):
    """Is this HEARTBEAT from a vehicle, or from a ground station on the same link?

       IT MATTERS.  A GCS heartbeat has no SAFETY_ARMED bit, so taking the armed flag
       from whichever heartbeat arrived last makes it alternate at the heartbeat rate
       once Mission Planner or KGCS is also connected.

       This is a filter, NOT the way the vehicle is identified - a ground station that
       does not label itself MAV_TYPE_GCS would pass it.  See consume()."""
    return msg.type != MAV_TYPE_GCS and msg.autopilot != MAV_AUTOPILOT_INVALID


class TakeoffWatcher(threading.Thread):
    '''Takeoff is FC state and never appears on CAN, so it is read from SITL's MAVLink
       output instead.  Optional - the console can force it by hand.'''

    def __init__(self, url, pmu, alt_threshold, sysid=None):
        super(TakeoffWatcher, self).__init__(daemon=True)
        self.url = url
        self.pmu = pmu
        self.alt_threshold = alt_threshold
        self.sysid = sysid              # the vehicle, once one has identified itself
        self.armed = False
        self.rel_alt = 0.0
        self.landed_state = None
        self.in_air = False
        self.heartbeats = 0             # from the locked vehicle only
        self._requested = {}            # (sysid, compid) -> when we last asked

    def summary(self):
        return ('MAVLink: vehicle %s  armed=%s  rel_alt=%.1f m  landed_state=%s  heartbeats=%u'
                % (self.sysid if self.sysid is not None else 'NOT IDENTIFIED',
                   self.armed, self.rel_alt, self.landed_state, self.heartbeats))

    def airborne(self):
        '''the airborne decision, from whatever the vehicle has told us so far'''
        # EXTENDED_SYS_STATE is authoritative when the vehicle sends it.  UNDEFINED is
        # not an answer, so it is treated as no data rather than as "on the ground".
        #
        # ONLY AN EXPLICIT ON_GROUND COUNTS AS LANDED.  Testing for IN_AIR and TAKEOFF
        # instead looked equivalent and was not: RTL reports LANDING all the way down, so
        # the engine dropped to the 5000 rpm ground idle WHILE DESCENDING.  Anything that
        # is not a positive statement of being on the ground keeps flight power, which is
        # also the safe direction for any landed_state added to MAVLink later.
        if self.landed_state is not None:
            return self.landed_state != LANDED_STATE_ON_GROUND
        # Otherwise armed-and-climbing, which needs no message-interval request.  The
        # band is hysteresis: a hover sitting exactly on the threshold would otherwise
        # chatter, and every flip snaps ILD and PCL to the other state's value.
        if not self.armed:
            return False
        if self.rel_alt > self.alt_threshold:
            return True
        if self.rel_alt < self.alt_threshold * 0.5:
            return False
        return self.in_air

    def request_target(self, msg):
        '''the (system, component) this heartbeat says we should ask for telemetry, or None

           COPTER'S DEFAULT STREAM RATES ARE ALL ZERO (`GCS_MAVLink_Parameters.cpp:60-70`:
           every `AP_MAV_DEFAULT_STREAM_RATE_*` is 0 for `APM_BUILD_COPTER_OR_HELI`).  A
           passive listener therefore receives HEARTBEAT and nothing else - no position,
           no extended state, so no takeoff, ever.  MAVProxy and Mission Planner work
           because they ASK; this has to ask too rather than assume somebody else has.

           The heartbeat is used only to ADDRESS the request.  The vehicle is still
           identified by the telemetry that comes back, never by the heartbeat.'''
        if self.sysid is not None:
            return None
        if msg.get_type() != 'HEARTBEAT' or not is_vehicle_heartbeat(msg):
            return None
        key = (msg.get_srcSystem(), msg.get_srcComponent())
        now = time.monotonic()
        if now - self._requested.get(key, -1e9) < STREAM_REQUEST_PERIOD_S:
            return None
        self._requested[key] = now
        return key

    def request_streams(self, conn, sysid, compid):
        from pymavlink import mavutil
        interval_us = int(1e6 / STREAM_REQUEST_HZ)
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                       mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE):
            conn.mav.command_long_send(sysid, compid,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       msg_id, interval_us, 0, 0, 0, 0, 0)
        self.pmu.log('MAVLink: asked system %u/%u for position and extended state' % (sysid, compid))

    def consume(self, msg):
        '''fold one message in, ignoring anything that is not our vehicle

           THE VEHICLE IS IDENTIFIED BY ITS TELEMETRY, NOT BY ITS HEARTBEAT.  Locking onto
           the first non-GCS heartbeat looked right and was not: a ground station that
           does not label itself MAV_TYPE_GCS captures the lock, and then no position
           ever matches and takeoff is never detected.  GLOBAL_POSITION_INT and
           EXTENDED_SYS_STATE are sent by vehicles and by nothing else, so whoever sends
           one is the vehicle.  Heartbeats are then read only from that system.'''
        kind = msg.get_type()
        srcsystem = msg.get_srcSystem()

        if kind in VEHICLE_ONLY_MESSAGES and self.sysid is None:
            self.sysid = srcsystem
            self.pmu.log('MAVLink: following vehicle system %u (identified by %s)' % (srcsystem, kind))

        if self.sysid is None or srcsystem != self.sysid:
            return

        if kind == 'HEARTBEAT':
            # still filtered: a ground station sharing the vehicle's system id would
            # otherwise clear the armed flag on every one of its heartbeats
            if not is_vehicle_heartbeat(msg):
                return
            self.heartbeats += 1
            self.armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == 'GLOBAL_POSITION_INT':
            self.rel_alt = msg.relative_alt / 1000.0
        elif kind == 'EXTENDED_SYS_STATE':
            if msg.landed_state != LANDED_STATE_UNDEFINED:
                self.landed_state = msg.landed_state

    def connect(self, mavutil):
        """keep trying until the endpoint exists

           SITL's SERIAL1 and SERIAL2 listeners (TCP 5762 and 5763) DO NOT EXIST until
           something connects to SERIAL0, because SERIAL0 is `tcp:0:wait` - SITL blocks
           at startup until a ground station attaches.  Starting the fake first therefore
           gets ECONNREFUSED, and a watcher that dies on that leaves takeoff detection
           off for the rest of the session with only a traceback to say so."""
        announced = False
        while self.pmu.running:
            try:
                return mavutil.mavlink_connection(self.url)
            except Exception as err:
                if not announced:
                    announced = True
                    self.pmu.log('MAVLink: %s not available yet (%s) - retrying' % (self.url, err))
                time.sleep(2.0)
        return None

    def run(self):
        try:
            from pymavlink import mavutil
        except ImportError:
            self.pmu.log('pymavlink not installed - automatic takeoff detection off, use "takeoff"')
            return
        self.pmu.log('watching %s for takeoff' % self.url)
        # the whole loop is guarded: this thread going down silently is how two sessions
        # were spent wondering why takeoff was never detected
        try:
            self.watch(mavutil)
        except Exception as err:
            self.pmu.log('MAVLink: watcher stopped on %s: %s - use the "takeoff" command'
                         % (type(err).__name__, err))

    def watch(self, mavutil):
        conn = self.connect(mavutil)
        if conn is None:
            return
        self.pmu.log('MAVLink: connected to %s' % self.url)
        while self.pmu.running:
            msg = conn.recv_match(type=['HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE'],
                                  blocking=True, timeout=1.0)
            if msg is None:
                continue
            target = self.request_target(msg)
            if target is not None:
                self.request_streams(conn, *target)
            self.consume(msg)
            in_air = self.airborne()
            if in_air != self.in_air:
                self.in_air = in_air
                self.pmu.commands.put('takeoff' if in_air else 'land')


def main():
    parser = argparse.ArgumentParser(description='fake PMU on SocketCAN for SITL')
    parser.add_argument('--interface', default='vcan0', help='SocketCAN interface (default vcan0)')
    parser.add_argument('--mavlink', default='udp:127.0.0.1:14550',
                        help='SITL MAVLink for takeoff detection, or "none"')
    parser.add_argument('--takeoff-alt', type=float, default=1.0,
                        help='relative altitude counted as airborne when EXTENDED_SYS_STATE is absent')
    parser.add_argument('--sysid', type=int, default=None,
                        help='pin the vehicle MAVLink system id instead of identifying it from telemetry')
    parser.add_argument('--silent', action='store_true', help='start without answering RTRs')
    parser.add_argument('--verbose', action='store_true', help='log every command frame')
    args = parser.parse_args()

    pmu = FakePMU(args.interface, verbose=args.verbose)
    try:
        pmu.connect()
    except OSError as err:
        print('cannot open %s: %s' % (args.interface, err), file=sys.stderr)
        print('try: sudo modprobe vcan && sudo ip link add %s type vcan && sudo ip link set %s up'
              % (args.interface, args.interface), file=sys.stderr)
        return 1

    if args.silent:
        pmu.set_silent(True)

    signal.signal(signal.SIGUSR1, lambda *_: pmu.commands.put('toggle-silent'))
    threading.Thread(target=pmu.console_thread, daemon=True).start()

    if args.mavlink.lower() != 'none':
        pmu.watcher = TakeoffWatcher(args.mavlink, pmu, args.takeoff_alt, args.sysid)
        pmu.watcher.start()

    try:
        pmu.run()
    except KeyboardInterrupt:
        pmu.log('interrupted')
    return 0


if __name__ == '__main__':
    sys.exit(main())
