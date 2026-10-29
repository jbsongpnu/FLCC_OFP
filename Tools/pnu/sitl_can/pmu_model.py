#!/usr/bin/env python3
'''
PNU : engine and power model for the fake PMU.

Pure state machine - no CAN, no sockets, no MAVLink.  Everything here is a function of
the commands fed in and the time elapsed, which is what makes it testable on its own and
what keeps fake_pmu.py down to framing and I/O.

The model is specified in README.md, "The fake PMU's engine model".  The wire format it
feeds is specified in "PMU CAN Comm Protocol.md" (repository root), the English rendering
of the PMU's ICD.  Where the two disagree the ICD wins; where the ICD is silent the value
is marked MODEL CHOICE below and is not a protocol fact.

  RPM ladder, no step skippable:
    STOPPED -> CHOKING (choke closed)  : 1000 rpm, 2 s, then STOPPED.  Still not running.
    STOPPED -> CRANK   (choke open)    : 1000 rpm, 2 s
            -> WARMUP                  : 4200 rpm, heads climb 25 -> 100 degC at 5 degC/s
            -> GROUND_IDLE  at 80 degC : 5000 rpm, cooling fan on, load 6.3 A
            -> FLIGHT       on takeoff : 6200 rpm, load 87.5 A, heads climb to 180 degC
    takeoff attempted below 5000 rpm   : ABORT - engine stops, battery takes the load,
                                         latched until a fresh Start

AP_FLAKE8_CLEAN
'''

import enum


class State(enum.Enum):
    STOPPED = 'STOPPED'
    CHOKING = 'CHOKING'          # choke crank: reaches 1000 rpm, stops again, never runs
    CRANK = 'CRANK'              # real crank: 1000 rpm before the warm-up ramp
    WARMUP = 'WARMUP'            # 4200 rpm warm-up idle
    GROUND_IDLE = 'GROUND_IDLE'  # 5000 rpm, virtual cooling device running
    FLIGHT = 'FLIGHT'            # 6200 rpm
    STOPPING = 'STOPPING'        # commanded stop, RPM winding down
    ABORT = 'ABORT'              # cold-start protection tripped, latched


# RPM targets.  The ladder's own figures; PNU 2026-10-27.
RPM_CRANK = 1000
RPM_WARMUP = 4200
RPM_GROUND_IDLE = 5000
RPM_FLIGHT = 6200
RPM_OVERSPEED = 6500        # PSTS bit 6 trips above this
RPM_CEILING = 7000          # never exceeded, even at 100% throttle

# MODEL CHOICE.  The specification gives exactly one ramp time - 1000 -> 4200 rpm in 1 s -
# so a single slew rate of 3200 rpm/s reproduces it exactly and every other transition
# falls out of the same mechanism instead of needing its own invented number.
RPM_SLEW_PER_S = float(RPM_WARMUP - RPM_CRANK)

# Hold at 1000 rpm before the warm-up ramp, and the whole duration of the choke crank.
CRANK_HOLD_S = 2.0

# Cylinder head temperatures, degC.  One rate in both directions; PNU 2026-10-29.
TEMP_AMBIENT = 25.0
TEMP_WARM = 100.0
TEMP_FLIGHT = 180.0
TEMP_COOLING_ON = 80.0      # trips GROUND_IDLE and the cooling fan
TEMP_OVERHEAT = 230.0       # PSTS bit 10
TEMP_SLEW_PER_S = 5.0

# Throttle position report, percent.  Raw percent on the wire, scale 1.0.
PCL_STOPPED = 0
PCL_STARTING = 8            # choke crank, crank and warm-up idle
PCL_GROUND_IDLE = 13        # the 5% the cooling device's load opens
PCL_FLIGHT = 50

# Currents and voltage, amperes and volts.  ILD is always positive (ICD AUX1STS);
# IBAT is positive for CHARGING and negative for DISCHARGING (ICD BATSTS).
ILD_IDLE = 0.0
ILD_COOLING = 6.3           # 302 W at 48 V - the cooling device; PNU 2026-10-29
ILD_FLIGHT = 87.5           # 4200 W at 48 V

# PSTS bit positions, from the ICD.  Bits 5, 11 and 12 are reserved.
PSTS_MANUAL = 0
PSTS_STARTING = 1
PSTS_STOPPING = 2
PSTS_CHOKE_CLOSED = 3
PSTS_COOLING_FAN = 4
PSTS_RPM_HIGH = 6
PSTS_RANDOM_STOP = 7
PSTS_FUEL_LOW = 8
PSTS_HOURS_LIMIT = 9
PSTS_HEAD_HOT = 10
PSTS_BAT_OVERCURRENT = 13
PSTS_BAT_UNDERVOLT = 14
PSTS_BAT_OVERVOLT = 15

# Per-state constants.  Reading the ladder out of one table beats scattering it through
# the update path, and it is the same table as README.md's.
_TARGET_RPM = {
    State.STOPPED: 0,
    State.CHOKING: RPM_CRANK,
    State.CRANK: RPM_CRANK,
    State.WARMUP: RPM_WARMUP,
    State.GROUND_IDLE: RPM_GROUND_IDLE,
    State.FLIGHT: RPM_FLIGHT,
    State.STOPPING: 0,
    State.ABORT: 0,
}

_TARGET_TEMP = {
    State.STOPPED: TEMP_AMBIENT,
    State.CHOKING: TEMP_AMBIENT,
    State.CRANK: TEMP_AMBIENT,
    State.WARMUP: TEMP_WARM,
    State.GROUND_IDLE: TEMP_WARM,
    State.FLIGHT: TEMP_FLIGHT,
    State.STOPPING: TEMP_AMBIENT,
    State.ABORT: TEMP_AMBIENT,
}

_PCL = {
    State.STOPPED: PCL_STOPPED,
    State.CHOKING: PCL_STARTING,
    State.CRANK: PCL_STARTING,
    State.WARMUP: PCL_STARTING,
    State.GROUND_IDLE: PCL_GROUND_IDLE,
    State.FLIGHT: PCL_FLIGHT,
    State.STOPPING: PCL_STOPPED,
    State.ABORT: PCL_STOPPED,
}

# Load current.  ABORT keeps the flight load because the motors are still turning - the
# engine is what stopped, not the aircraft.  That is the whole observable of the abort.
_ILD = {
    State.STOPPED: ILD_IDLE,
    State.CHOKING: ILD_IDLE,
    State.CRANK: ILD_IDLE,
    State.WARMUP: ILD_IDLE,
    State.GROUND_IDLE: ILD_COOLING,
    State.FLIGHT: ILD_FLIGHT,
    State.STOPPING: ILD_IDLE,
    State.ABORT: ILD_FLIGHT,
}


def _slew(current, target, rate, dt):
    '''move current towards target at rate per second, without overshooting'''
    step = rate * dt
    if current < target:
        return min(target, current + step)
    return max(target, current - step)


class PMUModel(object):
    '''the fake PMU's engine, generator and battery'''

    def __init__(self, log=None):
        self._log = log if log is not None else (lambda msg: None)

        self.state = State.STOPPED
        self.rpm = 0.0
        self.head_temp = TEMP_AMBIENT
        self.hour_count_s = 0.0     # ICD: HOURCNT is SECONDS, despite the name

        # latched commands, as last received on the wire
        self.choke_closed = False   # ENGCHK, 1 = close
        self.manual_mode = False    # ENGMANUAL, 1 = manual
        self.throttle_cmd = 0       # ENGPCL, manual only - recorded, see below
        self.battery_cmd = 0        # BATCTRL

        self.in_air = False         # from MAVLink, or forced from the console
        self.abort_latched = False

        self._hold_s = 0.0          # time spent at the crank RPM

    # -- commands ---------------------------------------------------------------------

    def cmd_engine_onoff(self, value):
        '''ENGONOFF.  1 = start, 0 = stop.  The FC sends this only on a transition.'''
        if value == 1:
            self._start()
        elif value == 0:
            self._stop()
        else:
            self._log('ENGONOFF: ignoring undefined CMD %u' % value)

    def cmd_choke(self, value):
        closed = (value == 1)
        if closed != self.choke_closed:
            self._log('ENGCHK: choke %s' % ('CLOSED' if closed else 'OPEN'))
        self.choke_closed = closed

    def cmd_manual(self, value):
        manual = (value == 1)
        if manual != self.manual_mode:
            self._log('ENGMANUAL: %s' % ('MANUAL' if manual else 'AUTO'))
        self.manual_mode = manual

    def cmd_throttle(self, value):
        # ENGPCL is valid only in manual mode (ICD).  It is recorded and echoed, but it
        # does NOT drive RPM: the ladder is state-driven and the ICD gives no
        # throttle-to-RPM relation, so deriving one would be invention, not simulation.
        self.throttle_cmd = value

    def cmd_battery(self, value):
        self.battery_cmd = value

    def _start(self):
        if self.state not in (State.STOPPED, State.ABORT):
            self._log('ENGONOFF start: already running in %s, ignored' % self.state.value)
            return
        if self.abort_latched:
            self._log('cold-start abort cleared by a fresh Start')
            self.abort_latched = False
        self._hold_s = 0.0
        self._enter(State.CHOKING if self.choke_closed else State.CRANK)

    def _stop(self):
        if self.state in (State.STOPPED, State.STOPPING):
            return
        self._enter(State.STOPPING)

    def set_in_air(self, in_air):
        '''takeoff / landing, from MAVLink or the console.  Edge triggered.'''
        if in_air == self.in_air:
            return
        self.in_air = in_air
        if in_air:
            self._takeoff()
        elif self.state == State.FLIGHT:
            # MODEL CHOICE: landing drops back to ground idle so a run can be repeated
            # without restarting the engine.  Not specified.
            self._enter(State.GROUND_IDLE)

    def _takeoff(self):
        if self.state == State.GROUND_IDLE:
            self._enter(State.FLIGHT)
            return
        # Cold-start protection: the ladder was skipped.  The engine stops and the
        # battery alone carries the flight load.
        self.abort_latched = True
        self._log('COLD START ABORT: takeoff at %u rpm, below %u' % (int(self.rpm), RPM_GROUND_IDLE))
        self._enter(State.ABORT)

    def _enter(self, state):
        if state == self.state:
            return
        self._log('state %s -> %s' % (self.state.value, state.value))
        self.state = state
        self._hold_s = 0.0

    # -- time -------------------------------------------------------------------------

    def update(self, dt):
        if dt <= 0.0:
            return

        self.rpm = _slew(self.rpm, _TARGET_RPM[self.state], RPM_SLEW_PER_S, dt)
        self.rpm = min(self.rpm, float(RPM_CEILING))
        self.head_temp = _slew(self.head_temp, _TARGET_TEMP[self.state], TEMP_SLEW_PER_S, dt)

        if self.rpm > 0.0:
            self.hour_count_s += dt

        if self.state in (State.CHOKING, State.CRANK):
            # the hold starts when the crank RPM is actually reached, not when the
            # command arrived, so the ramp is not counted against the 2 s
            if self.rpm >= RPM_CRANK:
                self._hold_s += dt
                if self._hold_s >= CRANK_HOLD_S:
                    if self.state == State.CHOKING:
                        self._log('choke crank complete, engine remains stopped')
                        self._enter(State.STOPPED)
                    else:
                        self._enter(State.WARMUP)
        elif self.state == State.WARMUP:
            if self.head_temp >= TEMP_COOLING_ON:
                self._log('heads at %.1f degC, cooling device on' % self.head_temp)
                self._enter(State.GROUND_IDLE)
        elif self.state == State.STOPPING:
            if self.rpm <= 0.0:
                self._enter(State.STOPPED)

    # -- derived values, in physical units --------------------------------------------

    @property
    def running(self):
        '''There is no "engine running" bit on the wire - KGCS infers it from the RPM
           range, and so does this.  See README.md.'''
        return self.rpm > 0.0 and self.state in (State.WARMUP, State.GROUND_IDLE, State.FLIGHT)

    @property
    def load_current(self):
        return _ILD[self.state]

    @property
    def battery_current(self):
        '''generated = ILD + IBAT.  Zero while the engine runs, by specification; the
           abort is the only case where the battery carries anything.'''
        if self.state == State.ABORT:
            return -ILD_FLIGHT
        return 0.0

    @property
    def throttle_position(self):
        return _PCL[self.state]

    @property
    def starting(self):
        if self.state in (State.CHOKING, State.CRANK):
            return True
        return self.state == State.WARMUP and self.rpm < RPM_WARMUP

    @property
    def psts(self):
        bits = 0
        if self.manual_mode:
            bits |= 1 << PSTS_MANUAL
        if self.starting:
            bits |= 1 << PSTS_STARTING
        if self.state == State.STOPPING:
            bits |= 1 << PSTS_STOPPING
        if self.choke_closed:
            bits |= 1 << PSTS_CHOKE_CLOSED
        if self.state in (State.GROUND_IDLE, State.FLIGHT):
            bits |= 1 << PSTS_COOLING_FAN
        if self.rpm > RPM_OVERSPEED:
            bits |= 1 << PSTS_RPM_HIGH
        if self.abort_latched:
            bits |= 1 << PSTS_RANDOM_STOP
        if self.head_temp > TEMP_OVERHEAT:
            bits |= 1 << PSTS_HEAD_HOT
        # PSTS_FUEL_LOW is held clear: the aircraft has no fuel sensor and GAS is fixed
        # at 100.  PSTS_HOURS_LIMIT and the three battery warnings are not modelled.
        return bits

    def summary(self):
        return ('%-12s rpm %5u  heads %6.1f degC  PCL %3u%%  ILD %6.2f A  IBAT %7.2f A  '
                'PSTS 0x%04X%s' % (self.state.value, int(round(self.rpm)), self.head_temp,
                                   self.throttle_position, self.load_current, self.battery_current,
                                   self.psts, '  [LATCHED]' if self.abort_latched else ''))
