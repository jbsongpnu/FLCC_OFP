# Simulated PNU CAN devices for SITL

CAN Update Plan **step 2b** (the PMU) and SAR Integration Plan **step 3b** (the SAR). See
the main `README.md` for both plans and for the fake PMU's engine model; the wire formats
are `PMU CAN Comm Protocol.md` and `SAR_ICD_PNU.md` in the repository root.

Everything in the PNU CAN stack except the CubeMars codec used to be compile-verified
only. These fakes exist so the state machines, the health logic and the engine-start
interlock timing can be exercised on a desk instead of against a running engine.

**The two fakes run together on one `vcan0`**, which is what `CAN_D1_PNU_OPTS = 5`
exercises and what the project had never been able to test before 3b: the PMU is 29-bit
extended, the SAR is 11-bit standard in `0x400`-`0x7FF`, and each process filters in the
kernel for its own frames.

| File | What it is |
|---|---|
| `pmu_model.py` | the engine, generator and battery. Pure state machine, no I/O |
| `pmu_frames.py` | the five status frames. Wire format only, no state |
| `fake_pmu.py` | SocketCAN I/O, the operator console and MAVLink takeoff detection |
| `tc1_inject.py` | TC1 injector and stopwatch for the step 2c interlock windows |
| `sar_frames.py` | the SAR's 16 ids and the four frames a SAR transmits. Wire format only |
| `fake_sar.py` | SocketCAN I/O and the operator console for the SAR |
| `test_pmu_model.py` | 17 tests: the RPM ladder, and every field at the offset `AP_PMUCAN` reads |
| `test_fake_pmu.py` | 26 tests: RTR dispatch, command decode, the silent trigger, takeoff detection |
| `test_fake_sar.py` | 21 tests: heartbeat framing, command responses, and the ids checked against both the ICD and `AP_PNUSAR.h` |

There is no `sar_model.py`. In 3b the device has almost no behaviour - it beats once a
second and answers commands - and the observation state machine belongs to step 3d, where
it lands with the FC side that drives it.

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

## The fake PMU console

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

## The fake SAR

`fake_sar.py` stands in for Lumir's radar payload. Unlike the PMU it **transmits on its
own**: `Heart_Beat 0x510` is periodic at 1 Hz and nothing asks for it, so `silent` here
means ceasing to beat, which is what the FC's 3 s `T_HB` is written against.

```sh
Tools/pnu/sitl_can/fake_sar.py --interface vcan0 --counter 777000
```

| Command | Effect |
|---|---|
| `silent` / `resume` | **stop / resume beating** - the FC must report Disconnected 3 s later |
| `status` | heartbeats sent, next `Raw_Counter`, every id received with its count, and the bundle tally |
| `nav` | the **last complete navigation bundle**, decoded field by field, with the complete and discarded counts |
| `hb` | one heartbeat now, to end a silence at a chosen instant |
| `short` | a **DLC 3** heartbeat. The FC must reject it and say so, and must not count it |
| `counter N` | set `Raw_Counter`, **including downwards** - ICD 3.3 forbids inferring a reboot from a decrease |
| `start-resp [bad]` / `stop-resp [bad]` | an unsolicited `0x521` / `0x522`; `bad` sends `0xFF` instead of `0x01`, which ICD 12.2 says the FC must not accept as a confirmation |
| `param-resp` | `0x523`, eight opaque bytes |
| `respond on` / `off` | answer `0x501`/`0x502` or stay quiet; `off` is how the FC's 2 s window is driven into `SAR_STATUS` 4 |
| `intrude [id]` | transmit on an **FC -> SAR** id, default `0x400`. With an observation running this also lands mid-bundle, which is how the ICD 7.3 boundary rules get exercised |
| `quit` | exit |

`kill -USR1 <pid>` toggles silence, as with the PMU.

**The fake assembles the 10 Hz navigation bundle the way the ICD requires a receiver to.**
Section 7.3: collect freshly received ids into a new buffer after the previous boundary,
apply the closing `0x408` timestamp, and **discard a bundle with missing ids, duplicates
or order errors** rather than filling the gap from an older one. There is no per-frame
sequence number to help - the ICD declines to add one - so arrival order is the only thing
a boundary can be built from. A receiver willing to accept anything would prove nothing
about the FC's transmitter, which is why the rule is enforced here and not just described.

The first two complete bundles are printed field by field, then one summary line every
10 s; `nav` prints the latest on demand. Sentinels decode as `n/a`, never as numbers -
ICD 2.3 - so `0x404`'s `Roll n/a Pitch n/a Yaw n/a ATT_Flags 0x00` is what a correct FC
looks like until step 3f supplies real attitude.

Switches: `--hb-rate` in Hz (`0` disables beating entirely), `--counter` the initial
`Raw_Counter`, `--resp-delay` seconds before answering a command, `--no-respond` to never
answer one, `--silent` to start quiet, `--verbose` to log every received frame rather
than the first on each id.

**`--counter` is not cosmetic.** ICD 3.3 says the FC's `Live_Counter` counts *receptions*
and must not copy the SAR's `Raw_Counter`; starting the fake at 777000 makes the two
impossible to confuse. The FC's value is not observable until the 51050 TM arrives in
step 3c.

**`intrude` is the test for a deliberate omission.** `AP_PNUSAR::handle_frame()` leaves
the FC -> SAR ids unclaimed, so a navigation or command frame arriving from the wire is
reported by the driver as unclaimed traffic instead of being quietly swallowed. That is
how a second talker on our ids becomes visible, and `intrude` is how it is proved rather
than argued.

## The fake gimbal

Three CubeMars GLII motors on `vcan0`, so that `AP_PNUGimbal` can be driven without
hardware. `gimbal_frames.py` is the wire format, `gimbal_model.py` is the motion, and
`fake_gimbal.py` is I/O and the console.

```sh
Tools/pnu/sitl_can/fake_gimbal.py --interface vcan0
```

**Start it BEFORE SITL.** The driver's whole bring-up - clear errors, enable, and the
first state transitions - happens in the first second or two after boot. A fake started
afterwards sees none of it, and neither does a GCS: those statustexts fire long before
anything has connected, which is how the first bug below stayed invisible.

**THE FEEDBACK TRIGGER IS AN ASSUMPTION, NOT A MEASUREMENT.** The manual never says what
makes a motor send a feedback frame - `PNUGimbal_CAN_Protocol_Type_CubeMars.md` sec 1 and
sec 9.6 both record the gap and say to measure it. So `--fb-mode` selects the behaviour
and the fake announces which one it is using at startup and in every `status` line:

| `--fb-mode` | Behaviour |
|---|---|
| `per-command` *(default)* | one frame per command received, including the universal commands |
| `periodic` with `--fb-rate N` | N frames per second per axis, independent of commands |
| `off` | nothing at all, for testing the driver's loss handling |

Nothing in SITL can tell you which one the real motors do. Run the bench measurement
before trusting any rate seen here.

| Command | Effect |
|---|---|
| `status` | mode, master id, feedback mode, wrong-mode and unknown-id counts, every command id received |
| `axes` | per axis: status, mechanical angle, **reported** angle, velocity, torque, stops |
| `fault AXIS CODE` | inject a status code in hex - `8` overvoltage, `9` undervoltage, `A` overcurrent, `B` MOS overtemp, `C` winding overtemp, `D` comms loss, `E` overload |
| `clear [AXIS]` | back to Enabled/Disabled, as the universal clear-errors would |
| `temp AXIS DRIVER MOTOR` | set the two signed int8 temperatures |
| `pos AXIS DEG` | teleport an axis, to start a run from somewhere awkward |
| `travel AXIS LOW HIGH` | **move the mechanical stops at runtime** - the way to make homing fail on purpose |
| `stiffness X` | stop stiffness in Nm/rad across all axes |
| `fb per-command\|periodic N\|off` | change the feedback trigger without restarting |
| `silent` / `resume` | stop and resume feedback entirely - every axis should go stale |
| `quit` | exit |

`kill -USR1 <pid>` toggles silence, as with the other two.

### What the gimbal model is, and is not

The fake PMU carries a real engine model because an engine has physics this project
knows. A gimbal motor does not: the manual gives no inertia, no friction, no thermal
coefficients and no current model. So the model covers exactly what Appendix 1's homing
sequence needs and nothing more - where each axis is, how fast it is moving, where its
stops are, and what the encoder zero means. **Temperatures and faults are not simulated**,
they are set from the console, because a fault that fires on invented physics teaches the
wrong lesson.

The joint is **massless**: it follows the velocity at which the MIT impedance law's torque
is zero, so there is no overshoot, no settling time and no torque ripple, and the fake
must not be read as evidence about any of them. Appendix 1 item 4 detects an endpoint by
angle alone, and that is what this supports.

**The stops are compliant, and that is deliberate.** A real heavy SAR payload gives a
little when a motor leans on its stop, and `--stop-stiffness` sets how much. A rigid stop
would make homing look easier in SITL than it will be on the bench and would leave
`PNU_HRTOL` untested. The default produces about a quarter of a degree at the homing
defaults - half of `PNU_HRTOL`. **It is a knob, not a measurement**: nobody has measured
this mount. Lower it until homing fails; that is the useful experiment.

## The shared-bus run

Both fakes, one bus, `CAN_D1_PNU_OPTS = 5`:

```sh
# 1. the bus, once per boot
sudo modprobe vcan
sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up

# 2. SITL.  Running the binary directly keeps sim_vehicle.py from reconfiguring the
#    waf board, and --serial0 tcp:0 opens all three ports with no ground station
build/sitl/bin/arducopter -S -I0 --model + --speedup 1 --serial0 tcp:0 \
    --defaults Tools/autotest/default_params/copter.parm,/tmp/pnu_sar.parm \
    --home 35.2345,129.0789,10,0

# 3. both fakes, in either order
Tools/pnu/sitl_can/fake_pmu.py --interface vcan0 --mavlink none
Tools/pnu/sitl_can/fake_sar.py --interface vcan0 --counter 777000
```

`/tmp/pnu_sar.parm`, or the same four values set from Mission Planner followed by a
reboot:

```text
SIM_CAN_TYPE1    2     # SocketCAN on vcan0, instead of the default multicast UDP
CAN_P1_DRIVER    1
CAN_D1_PROTOCOL  15    # PNU_CAN
CAN_D1_PNU_OPTS  5     # bit 0 PMU + bit 2 SAR; 7 adds the gimbal once that fake exists
```

Watch the FC's messages in Mission Planner, or on SERIAL1 with any MAVLink client. The
sequence worth driving, and what each step must produce:

| Drive | The FC must say |
|---|---|
| boot, both fakes running | `PNU_CAN: CAN_D1 PMU+SAR ready`, then `SAR connected` on the first heartbeat |
| `silent` | `SAR disconnected`, **3 s** after the last heartbeat |
| `resume` | `SAR reconnected, inspection pending` - and the state stays Disconnected, ICD 14.8 |
| `short` | `PNU_CAN: short frame 0x510 dlc 3` |
| `param-resp` | `SAR: 0x523 reserved, payload still TBD (1)` |
| `intrude` | `PNU_CAN: unclaimed frame 0x400` |

The PMU must stay healthy throughout, and its `status` must show the RTR count still
climbing: that is the shared bus working, and it is the part of 3b that no unit test can
reach.

### Without root

Creating `vcan0` needs root, but only in the namespace the test runs in. A user
namespace is enough, which makes the whole run scriptable on a machine where `sudo`
needs a password:

```sh
unshare -rn bash -c '
    ip link set lo up
    ip route add 224.0.0.0/4 dev lo    # SITL exits if its multicast socket cannot connect
    ip link add vcan0 type vcan && ip link set vcan0 up
    ... SITL and both fakes, all inside this namespace ...'
```

Everything - SITL, both fakes, any MAVLink client - has to run inside that namespace,
because it has its own loopback as well as its own `vcan0`. The multicast route is not
optional: `SITL_State.cpp:452` calls `exit(1)` when the servo multicast socket cannot
connect, and a fresh namespace has no route for `239.x`. The `vcan` module still has to
be loadable; if `ip link add` fails there, it needs `sudo modprobe vcan` once.

## Tests

No dependencies, no vcan interface, no SITL:

```sh
Tools/pnu/sitl_can/test_pmu_model.py
Tools/pnu/sitl_can/test_fake_pmu.py
Tools/pnu/sitl_can/test_fake_sar.py
Tools/pnu/sitl_can/test_fake_gimbal.py
```

The frame tests transcribe the byte offsets out of `AP_PMUCAN::handle_frame()` by hand
rather than importing them, so that a change on either side fails the test instead of
agreeing with itself. The SAR and gimbal tests go further and read the documents: the SAR
ids and DLCs against `SAR_ICD_PNU.md`'s own table and `AP_PNUSAR.h`, and the gimbal's DLCs
and universal command bytes against `AP_PNUGimbal_CubeMars.h`, with its published command
vectors taken byte for byte out of `PNUGimbal_CAN_Protocol_Type_CubeMars.md` section 8.

One gimbal test deliberately **pins a known error in that manual**. The published MIT
speed vectors decode to 4.74 and -4.84 rather than the +/-6 they claim, because the
manual's transmit function uses `2^N` where its receive function uses `2^N - 1`. The test
asserts the wrong values on purpose, so that a future session cannot "fix" the codec until
those rows pass and thereby break agreement with everything else in the document.

## Status

### Step 2b, the fake PMU

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

### Step 3b, the fake SAR and the first shared bus

**MEASURED IN SITL 2026-10-30**: fake PMU and fake SAR on one `vcan0`, `CAN_D1_PNU_OPTS
= 5`. This is the first time the driver has carried two devices at once - until now
every PNU CAN test had exactly one device on the wire. The run was scripted rather than
hand-flown, with the four parameters loaded as boot defaults and the SAR console driven
from a file.

| Drive | What the FC said | Interval |
|---|---|---|
| boot, both fakes up | `PNU_CAN: CAN_D1 PMU+SAR ready` | - |
| first heartbeat | `SAR connected` | 15 ms after the frame |
| `silent` | `SAR disconnected` | **3.058 s** after the last heartbeat |
| `resume` | `SAR reconnected, inspection pending` | 16 ms; **state stayed Disconnected** |
| `short` | `PNU_CAN: short frame 0x510 dlc 3 (1 dropped)` | - |
| `param-resp` | `SAR: 0x523 reserved, payload still TBD (1)` | - |
| `intrude` | `PNU_CAN: unclaimed frame 0x400 (1 seen)` | - |
| the PMU, throughout | `[PMU] Connected!`, and `status` showed **RTR seen 505, answered 505, commands 404** in 25 s | - |

3.058 s against a 3,000 ms deadline is the expected shape: `check_health()` runs every
20 ms and the rest is MAVLink transit and the observer's own timestamp.

**A second run at `--hb-rate 0.4`** - 2.5 s between beats, inside `T_HB` - produced no
disconnect at all across 11 heartbeats, and then `SAR disconnected` 3.027 s after the
fake stopped. The deadline is a wall clock, exactly as ICD 3.3 words it, and not "three
missed beats".

**The fake SAR received nothing from the FC**, which is what step 3a intends: the FC
transmits no SAR frame at all until 3d.

Three things this run did not reach:

- **`Live_Counter` is still unobservable.** It leaves the FC only through TM 51050,
  which is step 3c. The fake starts `Raw_Counter` at 777000 precisely so that the two
  cannot be confused on the day it becomes visible.
- **The command and response path** is covered only by the unit tests. Nothing on the
  FC side can send `0x501`/`0x502` before 3d.
- **No SAR hardware.** The 1 Hz on the wire, and its jitter, are this fake's, not
  Lumir's.

### Step 2b's other half, the fake gimbal - MEASURED 2026-10-30

Written at last, long after the fake PMU it was supposed to accompany, and it earned its
keep in its first run by finding a bug that would have stopped the gimbal dead on the
bench.

**Finding 1, fixed: `AP_PNUGimbal` never brought the motors up at all.** The backend's
`_state` is initialised to `State::INIT`, and `set_state(State::INIT)` early-returns when
the state is unchanged - so INIT's entry actions, the per-axis clear-errors and the
enable, were never performed at boot. The driver transmitted **nothing**: no feedback
could arrive, `all_axes_enabled()` stayed false, and INIT timed out into FAULT after 5 s,
whose own entry queued a disable. Those three disables were the only frames the motors
ever saw in a 47-second run. `init()` now performs the entry actions through a shared
`enter_init()`.

It was invisible for a reason worth remembering: every statustext that would have
explained it - `INIT -> FAULT`, the timeout, the disable - fires in the first seconds
after boot, **before any GCS has connected and before the dataflash logger has started**.
The same shape as the one-shot startup line that went missing in 5.1.16. Start the fake
**before** SITL or you will not see the bring-up either.

**Finding 2, open, and it decides step 3f: with per-command feedback the driver faults.**
Nothing in `AP_PNUGimbal` keeps the conversation alive between the enable and the first
motion command, and motion is transmitted only in RUN. With `--fb-mode per-command` the
last feedback arrives with the enable, and `AP_PNUGIMBAL_FB_TIMEOUT_MS` expires 500 ms
later - measured at exactly **523 ms** from enable to the FAULT disable. With
`--fb-mode periodic --fb-rate 50` the same build reaches RUN and streams motion normally.

So the driver as written **assumes the motors feed back autonomously**. The manual does
not say whether they do. That makes it the first thing to measure on the bench, ahead of
the rate question: if the real motors only answer commands, the driver needs a keep-alive
and the 40 ms `ATT_FRESH` budget of step 3f has to be re-derived around it.

**The Appendix 1 homing sequence ran for the first time**, with `PNU_HOME=1` and periodic
feedback:

| What the FC reported | |
|---|---|
| `PNUGimbal: roll travel 60.6 deg, mid -0.1` | roll homed at 14.4 s |
| `PNUGimbal: pitch travel 60.6 deg, mid -0.1` | pitch homed at 28.9 s |
| `PNUGimbal: yaw travel 40.5 deg, mid -0.1` | yaw homed at 40.4 s |
| `PNUGimbal: HOMING -> ALIGN -> RUN` | the whole sequence, roll then pitch then yaw, in 40 s |

**The measured travel carries the stop compliance, and that is the number the bench
tuning needs.** The mechanical travel is 60 and 40 degrees; the FC measured 60.6 and 40.5,
because each endpoint is found a quarter of a degree inside the stop's deflection. At the
homing defaults the stall torque is `HKD x HVEL` = 0.087 Nm, and the model's default
20 Nm/rad turns that into 0.25 deg per end. **0.6 deg of a 3 deg `PNU_HTOL` is consumed by
compliance alone** - a fifth of the tolerance, before any real-world asymmetry. A softer
mount eats more; that is what `stiffness` is for.

**All three devices on one bus at last.** `CAN_D1_PNU_OPTS = 7`, fake PMU, fake gimbal and
fake SAR on one `vcan0`: the gimbal homed all three axes while exchanging 11,356 commands
and 8,118 feedback frames, and the SAR beat 57 times, with no unclaimed-frame or
demultiplexing complaint from the driver. That configuration had never been run before.

**What this does NOT prove.** The feedback trigger and rate are the fake's, not the
motors'. There is no inertia, no friction, no backlash and no thermal behaviour, so
nothing here says anything about tuning `HKD`, `HVEL` or `HSTL` against a real payload.
The stop stiffness is a knob somebody chose, not a measurement. Homing on the bench is
still the hazardous operation it always was - this rehearsal only means the sequence
itself, and the driver that runs it, work.

## What is still NOT covered

- **Step 2c's engine-start interlock timing**, which is what step 2b was ordered first to
  measure. The fake logs every `ENGONOFF` frame with a timestamp for exactly this.
- A **measured** failsafe latency. The silent trigger fires it correctly, but nobody has
  timed it against the ~100-200 ms the constants predict.
- The real motors' **feedback trigger and rate**, which the manual does not specify and
  no fake can supply. See finding 2 above: the driver currently assumes autonomous
  feedback, and that assumption is untested against hardware.
- Stop compliance, stall thresholds, real motor dynamics and real PMU firmware behaviour.
  These fakes do not replace the bench; they move the logic errors off it.

## What the model is and is not

The engine model is PNU's specification, not a physical simulation. RPM follows a scripted
ladder; it is not derived from throttle, because the ICD gives no throttle-to-RPM relation
and inventing one would make the fake agree with nothing. Values marked `MODEL CHOICE` in
the source are not protocol facts - they are plausible fillers for fields the ICD leaves
open, and each is a one-line change.
