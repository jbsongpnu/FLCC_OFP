# Simulated PNU CAN devices for SITL

CAN Update Plan **step 2b**. See the main `README.md` for the plan and for the fake PMU's
engine model; the wire format is `PMU CAN Comm Protocol.md` in the repository root.

Everything in the PNU CAN stack except the CubeMars codec used to be compile-verified
only. These fakes exist so the state machines, the health logic and the engine-start
interlock timing can be exercised on a desk instead of against a running engine.

| File | What it is |
|---|---|
| `pmu_model.py` | the engine, generator and battery. Pure state machine, no I/O |
| `pmu_frames.py` | the five status frames. Wire format only, no state |
| `fake_pmu.py` | SocketCAN I/O, the operator console and MAVLink takeoff detection |
| `tc1_inject.py` | TC1 injector and stopwatch for the step 2c interlock windows |
| `test_pmu_model.py` | 17 tests: the RPM ladder, and every field at the offset `AP_PMUCAN` reads |
| `test_fake_pmu.py` | 26 tests: RTR dispatch, command decode, the silent trigger, takeoff detection |

## Setting up the bus

`AP_HAL_SITL`'s SocketCAN transport hard-codes the interface name as `vcan<instance>`
(`CAN_SocketCAN.cpp:32`), so the first CAN interface is always `vcan0`. Creating it needs
root, once per boot:

```sh
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set vcan0 up
```

SocketCAN is used rather than the multicast-UDP transport because it carries **extended
ids and RTR frames** faithfully, and the PMU protocol is built entirely on RTR.

## Running - the validated procedure

This is the order PNU used on 2026-10-29 to fly the whole sequence. **The order matters**,
for the two reasons explained after it.

```sh
# 0. once per boot - the bus must exist before SITL starts, because the CAN driver
#    attaches to it during boot
sudo modprobe vcan
sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up

# 1. build, if not already built
./waf configure --board sitl && ./waf copter

# 2. start SITL.  --no-mavproxy because KGCS needs a TCP endpoint, and MAVProxy does not
#    offer one by default.  SITL now prints "Waiting for connection ...." and ONLY 5760
#    is open
./Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy --no-rebuild
```

```text
# 3. connect Mission Planner to TCP 127.0.0.1:5760  (SERIAL0)
#    SITL unblocks and finishes booting, and 5762 and 5763 appear
# 4. connect KGCS to              TCP 127.0.0.1:5762  (SERIAL1)
```

```sh
# 5. start the fake PMU on the remaining port
Tools/pnu/sitl_can/fake_pmu.py --interface vcan0 --mavlink tcp:127.0.0.1:5763
```

First time only, set these from Mission Planner and reboot. SITL waits on SERIAL0 again
after a reboot, so Mission Planner has to reattach before the rest comes back:

```text
SIM_CAN_TYPE1    2     # SocketCAN on vcan0, instead of the default multicast UDP
CAN_P1_DRIVER    1
CAN_D1_PROTOCOL  15    # PNU_CAN
CAN_D1_PNU_OPTS  1     # PMU only; 3 adds the gimbal once that fake exists
```

`candump vcan0` from `can-utils` is useful alongside it.

**Mission Planner alone does not need `--no-mavproxy`** - it is happy on MAVProxy's UDP
14550. The option is there for KGCS. With MAVProxy instead, give the fake an output of its
own and leave 14550 and 14551 for the ground stations:

```sh
./Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild --out=udp:127.0.0.1:14560
Tools/pnu/sitl_can/fake_pmu.py --interface vcan0 --mavlink udp:127.0.0.1:14560
```

Takeoff detection can also be switched off entirely with `--mavlink none` and driven by
the `takeoff` console command.

### Why the order matters

**1. SERIAL1 and SERIAL2 do not exist until SERIAL0 is connected.** SERIAL0 is
`tcp:0:wait` (`SITL_State.h:33`), so SITL blocks at startup and opens nothing else.
Measured, not assumed:

```text
before a SERIAL0 client:  LISTEN 0.0.0.0:5760
after  a SERIAL0 client:  LISTEN 0.0.0.0:5760  5762  5763
```

The fake retries a refused connection rather than dying, so starting it early costs
seconds rather than the session - but nothing is detected until the port exists.

**2. `udp:127.0.0.1:14550` is MAVProxy's output, not SITL's.** With `--no-mavproxy`
nothing listens there at all, which is why the fake's default endpoint is wrong for this
procedure and `--mavlink tcp:127.0.0.1:5763` is needed.

**And one that is not about order: Copter's default stream rates are all zero**
(`GCS_MAVLink_Parameters.cpp:60-70` - every `AP_MAV_DEFAULT_STREAM_RATE_*` is 0 for
`APM_BUILD_COPTER_OR_HELI`). A passive listener receives HEARTBEAT and nothing else.
Mission Planner and MAVProxy work because they ask; **the fake asks too**, with
`MAV_CMD_SET_MESSAGE_INTERVAL`, so it does not depend on a ground station being attached
at all. Measured on SERIAL2:

```text
A) PASSIVE listener, 8 s : HEARTBEAT 82, TIMESYNC 9, SYS_STATUS 1, msg 51005 x803
                           - no GLOBAL_POSITION_INT, no EXTENDED_SYS_STATE
B) WATCHER, 14 s         : asked system 1/1, identified vehicle 1 from
                           GLOBAL_POSITION_INT, landed_state=1 (ON_GROUND)
```


## Measuring the step 2c interlock windows

`tc1_inject.py` sends the TC1 bursts KGCS would send - 14 messages alternating 2/3 for
Start, 4/5 for Stop, at 10 Hz - with the timing under our control, and **watches `vcan0`
itself** for the `ENGONOFF` the FC emits when the interlock completes. One process, one
clock, so the measured interval needs no correlation between logs.

**Do not run KGCS at the same time.** This tool stands in for it, and the FC does not check
who sent a TC1 - an accepted residual of PNU-ISSUE(D7) - so two senders would interleave
their bursts and the result would mean nothing. The default endpoint is SERIAL1, the port
KGCS would otherwise use, which leaves SERIAL0 for Mission Planner and SERIAL2 for the
fake PMU.

**SERIAL1 does not exist until SERIAL0 has a client**, so the injector cannot connect to
5762 straight after starting SITL. Either attach Mission Planner to tcp:5760 first, or
start SITL so that it never waits for one:

```sh
./Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy --no-rebuild -A "--serial0 tcp:0"
```

`-A` passes arguments to the SITL binary, and `--serial0 tcp:0` is `tcp:0:wait` without the
wait. Verified: all three ports open immediately with no ground station attached, which is
what makes the whole measurement scriptable. The injector waits for the endpoint either
way - `--link-timeout` bounds it.

```sh
# 1-2 as in the procedure above: vcan0, then SITL.  KGCS stays OFF.
# 3. the fake PMU, which does not need MAVLink for this test
Tools/pnu/sitl_can/fake_pmu.py --interface vcan0 --mavlink none

# 4. check the tool agrees with the firmware before trusting any result
Tools/pnu/sitl_can/tc1_inject.py --self-test

# 5. the scenarios, IN THIS ORDER - see below
Tools/pnu/sitl_can/tc1_inject.py start-only
Tools/pnu/sitl_can/tc1_inject.py cut-start
Tools/pnu/sitl_can/tc1_inject.py late-start
Tools/pnu/sitl_can/tc1_inject.py stop-start
```

`--self-test` checks the hand-packed TC1 message's CRC_EXTRA and length against the
firmware's own generated header, and the two window constants against `AP_PMUCAN.h`. The
PNU dialect is not in a stock pymavlink, so the frame is packed by hand; this is what makes
that safe.

**Order matters, because the FC latches the engine state.** A successful `stop-start`
leaves it ON, and later Start scenarios then do nothing. `start-only` must come first on a
fresh boot, since it tests the case where no Stop has ever been seen. `stop` returns the
FC to OFF between runs.

| Scenario | What it does | Expected |
|---|---|---|
| `start-only` | Start with no Stop ever seen | refused, `press Stop first` |
| `cut-start` | Stop, wait, then a Start burst cut after 2 messages | **no start** - this is the defect step 2c closes |
| `late-start` | Stop, wait 14 s, then Start | refused, `Stop 14s ago (max 10s)` |
| `stop-start` | the correct KGCS procedure | **starts**, about 1.5 s after the burst begins |
| `stop` / `start` | a single burst on its own | - |
| `cut-stop` | a Stop burst cut short | no stop - the side effect, not a goal |

Each run prints a timeline and a verdict. Useful switches: `--cut N` messages before a
cut-short burst stops, `--wait N` seconds between Stop and Start, `--settle N` how long to
keep watching afterwards, `--rate` and `--count` for the burst shape, `--interface none` to
skip the CAN watch.

**To see the defect rather than the fix**, raise `PMUCAN_TC1_STALE_MS` in `AP_PMUCAN.h` to
something huge, rebuild, and run `cut-start` again: the engine starts about 1.3 s after the
last message, with no further input. Put the constant back afterwards.

## The console

`fake_pmu.py` reads commands on stdin:

| Command | Effect |
|---|---|
| `silent` / `resume` | **stop / resume answering RTRs** - the whole point of step 2b |
| `status` | current model state, and the RTR and command counts |
| `start` / `stop` | drive the engine directly, without the FC |
| `choke open` / `choke close` | set the choke, which selects the crank behaviour |
| `takeoff` / `land` | force the airborne state when not watching MAVLink |
| `quit` | exit |

`kill -USR1 <pid>` toggles silence, for scripted tests.

Takeoff is FC state and never appears on CAN, so it is read from SITL's MAVLink output
(`--mavlink udp:127.0.0.1:14550`, or `none` to disable). `EXTENDED_SYS_STATE.landed_state`
is used when the vehicle sends it, otherwise armed-and-above-`--takeoff-alt`. pymavlink is
imported lazily and is not needed if the console is used instead.

## Tests

No dependencies, no vcan interface, no SITL:

```sh
Tools/pnu/sitl_can/test_pmu_model.py
Tools/pnu/sitl_can/test_fake_pmu.py
```

The frame tests transcribe the byte offsets out of `AP_PMUCAN::handle_frame()` by hand
rather than importing them, so that a change on either side fails the test instead of
agreeing with itself.

## Status

**VALIDATED IN SITL BY PNU, 2026-10-29**, with Mission Planner on SERIAL0 and KGCS on
SERIAL1. The full sequence flew as specified:

| Checked | Result |
|---|---|
| choke valve open and close | correct |
| choke crank - 1000 rpm, 2 s, engine stays stopped | correct |
| Start, warm-up idle, cooling trip to ground idle | correct |
| Stop | correct |
| takeoff - 6200 rpm, 87.5 A, heads to 180 degC | correct |
| RTL, descending, landing as the final stage | correct after the fix below |
| **`silent` while hovering - the D9 failsafe** | **correct: failsafe to RTL** |
| **`resume` - recovery** | **correct: reconnects, failsafe clears** |

Five defects were found on the way, **all in `TakeoffWatcher`, none in the engine model
and none on CAN**. The second was introduced by the fix for the first. That split is the
useful part: the CAN side had a written ICD and a specification to build against, and the
takeoff watcher is the one piece nobody specified.

1. **ILD, the head temperatures and PCL oscillated after takeoff.** `recv_match()` accepts
   `HEARTBEAT` from any source system, so with another ground station on the link the
   armed flag alternated at the heartbeat rate and the engine state flapped between ground
   idle and flight.
2. **Then takeoff was not detected at all.** The first fix identified the vehicle as the
   first heartbeat that is not `MAV_TYPE_GCS` - a sound filter but an unsound identifier,
   because a ground station that does not label itself as one captures the lock and no
   position message ever matches again.
3. **The engine dropped to ground idle while descending under RTL.** `MAV_LANDED_STATE_LANDING`
   is reported all the way down, and the airborne test asked for `IN_AIR` or `TAKEOFF`.
   **Only an explicit `ON_GROUND` counts as landed now** - anything else keeps flight
   power, which is the safe direction.
4. **The watcher died instead of retrying** when its endpoint did not exist yet, leaving
   takeoff detection off for the session with only a traceback. It now retries, and the
   loop is wrapped so the thread cannot disappear quietly.
5. **It could not work as a passive listener at all.** Copter's default stream rates are
   all zero, so it now requests `GLOBAL_POSITION_INT` and `EXTENDED_SYS_STATE` itself.

**The vehicle is now identified by its telemetry**, not its heartbeat:
`GLOBAL_POSITION_INT` and `EXTENDED_SYS_STATE` come from vehicles and from nothing else.
Heartbeats are read only from that system id. `--sysid` pins it by hand. The altitude
fallback also gained hysteresis, and `LANDED_STATE_UNDEFINED` is treated as no data.

**If takeoff is not detected, type `status`** - it reports the locked system id, armed,
relative altitude, landed state and heartbeat count, and the fake warns once if no vehicle
telemetry has arrived after 15 s.

## What is still NOT covered

- **Step 2c's engine-start interlock timing**, which is what step 2b was ordered first to
  measure. The fake logs every `ENGONOFF` frame with a timestamp for exactly this.
- A **measured** failsafe latency. The silent trigger fires it correctly, but nobody has
  timed it against the ~100-200 ms the constants predict.
- The gimbal and SAR fakes, which do not exist yet, and therefore the shared bus.
- Stop compliance, stall thresholds, real motor dynamics and real PMU firmware behaviour.
  These fakes do not replace the bench; they move the logic errors off it.

## What the model is and is not

The engine model is PNU's specification, not a physical simulation. RPM follows a scripted
ladder; it is not derived from throttle, because the ICD gives no throttle-to-RPM relation
and inventing one would make the fake agree with nothing. Values marked `MODEL CHOICE` in
the source are not protocol facts - they are plausible fillers for fields the ICD leaves
open, and each is a one-line change.
