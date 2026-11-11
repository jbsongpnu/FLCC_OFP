# PNU-KAL FLCC OFP - Development Version

| | |
|---|---|
| **Current version** | `PNU_OFP V5.1.24` (DEV) |
| **Baseline** | ArduPilot Copter **4.7.1** |
| **Last official release** | `PNU-KAL-FLCC-5.1.0` (same code as 5.1.0 dev) |
| **Vehicle** | **ArduCopter only.** Plane, Rover and Sub do not link - by construction |
| **Documents** | `CHANGELOG.md` - version history and verification. `ISSUES.md` - issues, risks and decisions |

---

## 1. Project Overview

PNU-KAL FLCC OFP is a fork of ArduPilot Copter for a gas-engine multicopter carrying a
gimbal-stabilised SAR payload, flown from PNU-GCS (replacing the legacy KGCS) over a
PNU-KAL MAVLink dialect.

Three devices share **CAN1**, behind one driver with three backends:

| Port | Bitrate | Carries | Driver |
|---|---|---|---|
| **CAN1** | **1,000,000 bit/s** | PMU + gimbal (3 CubeMars motors) + SAR | one `AP_PNU_CAN`, three backends |
| **CAN2** | 1,000,000 bit/s | DroneCAN GPS (Here4) only | `AP_DroneCAN`, unchanged |

**Why one driver and not three:** `AP_CANManager` binds each interface to exactly one driver, and
a driver with no interface fails *silently*. The full reasoning is in `ISSUES.md`, "Resolved
design questions".

**The bitrate is forced by the gimbal, not chosen.** The CubeMars GLII is fixed at 1 Mbit/s, so
`CAN_P1_BITRATE` stays at its default and **the PMU must be reconfigured from 500 k on the
device**.

### CAN ID allocation

The PMU is 29-bit extended and everything else is 11-bit standard, so `isExtended()` separates the
PMU for free. **It does not separate the gimbal from the SAR**, so they are told apart by id:

| Range | Owner | Notes |
|---|---|---|
| `0x000` | gimbal feedback (Master ID) | CubeMars default; `CAN_D1_PNU_MSTID` |
| `0x001`-`0x00F` | gimbal MIT commands | `(0 << 8) \| motor_id`, motor ids 1..15 |
| `0x101`-`0x10F` | gimbal position/velocity commands | `(1 << 8) \| motor_id` |
| `0x201`-`0x20F` | gimbal velocity commands | `(2 << 8) \| motor_id` |
| `0x300`-`0x3FF` | **reserved** | headroom for a second Master ID or a 4th axis |
| **`0x400`-`0x7FF`** | **SAR** | every SAR message has a distinct id in here |
| `0x10000000`+ | PMU | 29-bit extended, `FlagEFF` set on every id |

**Do not give the SAR an id below `0x400`.** `AP_PNU_CAN::init()` rejects a configuration where a
SAR id collides with the gimbal Master ID or any gimbal command id. The SAR ICD, which arrived
later, independently specified the same range, bitrate and MAVLink ids.

### Software layers

```text
AP_Mount  (50 Hz SCHED_TASK)
  +- AP_Mount_PNUGimbal : AP_Mount_Backend      targeting adapter, MNT1_TYPE = 15
       +- AP::pnu_can()->get_gimbal()           AP_PNUGimbal_Backend: lifecycle, limits, homing
AP_PNU_CAN  (100 Hz thread, owns the interface and the one send_frame())
  +- AP_PMUCAN                                  PMU protocol
  +- AP_PNUGimbal_CubeMars                      wire format only
  +- AP_PNUSAR                                  SAR protocol + the 10 Hz navigation bundle
```

---

## 2. Current Development Status

### CAN Update Plan

| # | Step | Status |
|:--:|---|---|
| 0 | Confirm device bitrates and the SAR id plan | **DONE 2026-10-15** |
| 1 | `AP_PNU_CAN`, the multi-backend driver | **DONE in 5.1.11** |
| 1b | `AP_Mount_PNUGimbal`, the mount backend | **DONE in 5.1.12**, bench-confirmed 2026-10-16 |
| 1c | The gimbal lifecycle state machine | **DONE in 5.1.13** (and only actually ran after the 5.1.23 fix) |
| 1d | HOMING, the Appendix 1 endpoint search | **DONE in 5.1.14, DEFAULTS OFF.** Not bench-run - `GIMBAL-002` |
| 2 | The PMU becomes a backend | **DONE in 5.1.15**, bench-tested 2026-10-19 **except the shared bus** |
| 2b | Simulated CAN devices for SITL | **DONE** - fake PMU 2026-10-29, fake SAR and fake gimbal 2026-10-30, one runner 2026-11-03 |
| 2c | Engine-start interlock guards | **DONE in 5.1.18**, measured in SITL with the original defect reproduced |
| 3 | SAR backend | see the SAR plan below |
| 4 | Bus documentation, bitrate note, README and `version.h` | **DONE** - this document, and the version defines corrected 2026-11-03 |

### SAR Integration Plan

`SAR_ICD_PNU.md` (repository root) is Rev1.0, supplied by PNU 2026-10-30 and jointly owned with
Lumir. **Read it before touching any of this** - it is unusually precise about byte order,
sentinels and what is still TBD, and it explicitly forbids inventing the TBDs.

**Step 3 was promoted ahead of the fake gimbal by PNU**, against a real schedule for connection
testing with Lumir's device. The gimbal is a stabiliser; the SAR provides the mission function.

| Phase | Content | Status |
|---|---|---|
| **3a** | `AP_PNUSAR` becomes a backend; protocol 17 retires; `0x510` Heart_Beat, Live_Counter, the 3 s timeout, states 0 and 1 | **DONE in 5.1.19** |
| **3b** | Fake SAR on `vcan0`; the first shared-bus run | **DONE 2026-10-30**, tools only |
| **3c** | MAVLink **50050** TC and **51050** TM; TM at 5 Hz unconditionally; TC source check | **DONE in 5.1.20** |
| **3d** | Commands `0x501`/`0x502`, responses `0x521`/`0x522`, the 2 s FC window, states 2/3/4, the Start interlock behind `CAN_Dn_PNU_SCHK` | **DONE in 5.1.21** |
| **3e** | The nine-frame **10 Hz navigation bundle** `0x400`-`0x408`, ICD 14.7 automatic termination. `0x404` is sentinels, not zeros | **DONE in 5.1.22** |
| **3f** | Real gimbal attitude into `0x404`, and the `ATT_FRESH` 40 ms contract | **NEXT. Blocked** - see section 5 |
| **3g** | ICD section 16: the 25 deg envelope and yaw to the waypoint-segment heading | after 3f |

**It is two bodies of work, not one.** **A** is the SAR interface - 16 CAN ids, the bundle, two
MAVLink messages, a five-state machine with 3 s and 2 s timers, the RTK-Fixed start interlock and
automatic termination. That is step 3. **B** is gimbal semantics for the SAR (ICD section 16),
which the ICD assigns to PNU Drone but which is `AP_PNUGimbal` work needing mission access and
real gimbal feedback. That is 3g and is not costed here.

### Camera and gimbal payload track

`PNU-ISSUE D10` and `D11` are the only two original issues still open, and **the FC-side work on
both is complete**. What remains is KGCS implementation: EO/IR source, IR palette, object tracking
and absolute zoom are all KGCS-to-payload direct and need no firmware. All the mappings are
recorded in `ISSUES.md`.

---

## 3. Current Development Version

**V5.1.28** makes the PMU say what it decided and what it sent. Two dataflash records,
no behaviour change. `PMUE` is event driven and records the step 2c interlock's own
decisions - how far a Start attempt got, which of the two `D7` guards refused it, how old
the preceding Stop was, and whether the `ENGONOFF` actually went out. All of that used to
live only in a statustext that was rate limited, clamped to fit MAVLink, and lost entirely
with no ground station attached; the whole five-scenario step 2c suite now produces seven
records, one of them the cut-short Start burst that step 2c exists to refuse. `PMUC` is the
cumulative PMU communication counters at 1 Hz, which closes `PMU-003` by surfacing the five
write-only counters rather than deleting them, plus the short-frame count. See
`CHANGELOG.md`. Two of `PMUC`'s six columns and two of `PMUE`'s seven event codes
**cannot occur in SITL at all** - `AP_HAL_SITL` never fails a CAN send - and were exercised
by forcing a failure in a temporary build.

**V5.1.27** makes the gimbal slew limiter measure its own interval. The per-tick step
was `CAN_Dn_PNU_SLEW` divided by the *nominal* loop rate, which is the right step only if
the driver thread achieves that rate; measured per tick, the realised limit ranged from
25.7 to 180.1 deg/s against a 45 deg/s parameter. It is now the rate multiplied by the
interval that actually elapsed, clamped to five nominal periods so a stalled thread cannot
repay its lateness in one command step - 45.00 deg/s on every limited tick, no step over
the limit in 3238. Measuring it also caught a defect in the first version of the fix, which
let a zero-length interval through the "no limit" branch and put 1038 deg/s on the bus. See
`CHANGELOG.md`. **`PNUCAN-001` stays open**: no CubeMars motor has ever been connected.

**V5.1.26** makes the CAN driver loop keep time. `AP_PNU_CAN::loop()` slept for a whole
period and *then* did its work, so a tick cost the period plus the body - 13.1 to 13.5 ms in
SITL against a nominal 10 ms. Every rate in the driver derives from that tick, so the PMU's
ICD 10 Hz was really 7.5 Hz and the `D7` engine Start interlock, which the source says is
1.0 s, measured 1.46 to 1.68 s - completing *after* the 1.4 s KGCS burst had ended. Sleeping
to the next deadline fixes all of them at once without changing a single constant. It also
fixes an `AP_Q30` scheduler entry left out of priority order in 5.1.25, which raised an
internal error and **blocked arming** on every boot. See `CHANGELOG.md`. **`PMU-004` stays
open**: the fix is SITL-verified and the target was never measured, before or after.

**V5.1.25** stops the flight log depending on who is watching. Every PNU telemetry record -
`TM11`, `TM12`, `TM13`, `SARS`, `TC_R` - used to be written inside the MAVLink send function
that transmitted it, so with two ground stations attached every record was written **twice**,
and with none attached **nothing was written at all**: an aircraft flying with its link down
kept no PMU and no SAR history. Each record now comes from the subsystem that produces the
data, on its own clock. `TM3` is logged for the first time. See `CHANGELOG.md`, and `PMU-004`
for the driver-tick finding that came out of it.

**V5.1.24** makes the SAR navigation bundle observable: a `SARN` dataflash record per snapshot,
drop counters split by cause, the last CAN TX error code, and a session summary statustext on
every Stop. Until it, a session that worked perfectly said nothing on the FC side - see
`SAR-002`.

**V5.1.23** carried one `AP_PNUGimbal` fix, found by the fake gimbal on its first run: the driver
**never brought the motors up at all**, and would have stopped the gimbal dead on the bench with
no message to say why. Wrong since 5.1.13. See `CHANGELOG.md`.

**Implemented capabilities at this version**

| Area | State |
|---|---|
| PMU | Full telemetry, commands, prearm, in-flight failsafe, engine-start interlock with two guards. **Bench-verified** except the shared bus and Stop-command timing |
| Gimbal | Mount backend, lifecycle state machine, Appendix 1 homing, per-axis direction parameters. **SITL only - no motor has ever turned** |
| SAR | Connection layer, both MAVLink messages, the command layer with the Start interlock, the 10 Hz navigation bundle with automatic termination, and `SARN` bundle accounting. **SITL only - no SAR hardware** |
| Camera | Viewpro via the KGCS TC2/TM2 interface; Gremsy VIO via standard MAVLink with four functions GCS-direct. **Bench-verified on both**, though the TM2 producer moved to `AP_Q30::update()` in 5.1.25 and that move is SITL-verified only |
| Logging | Every PNU telemetry record is written by the subsystem that produces it, at a rate independent of how many ground stations are attached. **SITL only** |

---

## 4. Current Development Plan

**Nothing is in progress.** The tree is at a clean point: every applied step is recorded, and the
next step is blocked on measurements only PNU can make.

---

## 5. Next Development Plan

### NEXT - step 3f: real gimbal attitude into `0x404`

It is also what makes `CAN_Dn_PNU_SCHK` bit 2 usable: setting it today refuses every Start and
terminates every observation, because a constant has no age.

**Blocked on four things, none of which is code in this repository:**

1. **A measured CubeMars feedback trigger, then its rate** - `GIMBAL-001`. If the motors only
   answer commands, `AP_PNUGimbal` needs a keep-alive and **3f's 40 ms budget has to be
   re-derived**.
2. **Bench-tuned homing, then `CAN_Dn_PNU_HOME=1`** - `GIMBAL-002`.
3. **Mechanical facts from PNU**: axis stacking order; whether the SAR body axes coincide with the
   inner-axis frame; per-axis rotation sign for `DRLL`/`DPIT`/`DYAW`.
4. **An agreement with Lumir** on the fixed transform between the SAR reference axes and the
   antenna electrical pointing axis (ICD 16.1).

### ALSO NEXT - items awaiting PNU

- **Sign off the two step 2c decisions** - `PMU-001`. The Stop window is measured from the Start
  press, and a Stop burst cut off mid-way no longer completes either.
- **Tell Lumir in writing that `0x404` is unpopulated and what the sentinel values are** -
  `SAR-001`. This is the mitigation that actually matters and it is still outstanding.
- **Bench the shared bus.** Every PMU bench run so far had `PNU_OPTS = 1`; the id-based health
  rewrite has only ever run in SITL.
- **KGCS/PNU-GCS side:** TC50 must be sent from a sysid `MAV_GCS_SYSID` covers (default 255), and
  both endpoints must use CRC_EXTRA **231** (TC50) and **7** (TM50) - `GCS-002`.

### THEN - step 3g

ICD section 16: the 25 deg stabilisation envelope and yaw held to the waypoint-segment heading.

---

## 6. Verification Summary

Detail is per version in `CHANGELOG.md`. **Nothing below promotes SITL evidence to hardware
evidence.**

### Verified on hardware

| What | When |
|---|---|
| PMU telemetry, KGCS and FC command paths, prearm with the PMU absent | bench 2026-10-19 |
| **In-flight PMU failsafe** - CAN connector pulled while armed, RTL taken immediately, clean recovery on reconnect | bench 2026-10-19 |
| `ARMING_SKIPCHK` bit 13 bypass and the one-shot startup report, both after the 5.1.16 fixes | bench 2026-10-19 |
| Engine interlock duration, ~1 to 1.5 s against an expected ~1.2 s | bench 2026-10-19 |
| `MNT0.Roll` / `MNT0.Pitch` move opposite vehicle roll and pitch (step 1b) | bench 2026-10-16 |
| Gremsy VIO identity, zoom through Mission Planner, gimbal slew / neutral / point-down | bench 2026-10-13 |
| Viewpro TC2 function matrix; pitch limits; the old-vs-new camera rate difference | bench 2026-09-29 |

### Verified in SITL only

| What | When |
|---|---|
| The whole PMU engine sequence including takeoff, RTL and the D9 failsafe via the silent trigger | 2026-10-29 |
| **Step 2c's two windows, with the original defect reproduced** when guard 1 was disabled | 2026-10-29 |
| SAR connection layer, TM50 at 5.01 Hz, `Live_Counter` semantics | 2026-10-30 |
| SAR command layer: every `SCHK` bit refusing and permitting, state 4 at 2.14 s, the retry clearing the latch | 2026-10-30 |
| The 10 Hz navigation bundle at **9.99 Hz** with `ATT_Seq` 1:1 and no gaps; three automatic terminations | 2026-10-30 |
| The Appendix 1 homing sequence end to end | 2026-10-30 |
| **All three devices on one bus** at `PNU_OPTS = 7`, with the SAR observing and the gimbal homing | 2026-10-30, 2026-11-03 |
| Every PNU log record at its stated rate with 0 duplicates, **both with a GCS attached and with none** - `TM11`/`TM12`/`TM13`/`TM3` at 9.95-10.00 Hz, `SARS` at 0.99-1.02 Hz plus every status transition, `TC_R` at 10.00 Hz | 2026-11-11 |
| TM2 still transmitted at 9.98 Hz with a payload matching `TC_R`, and **not** transmitted with `MNT1_TYPE = 0` | 2026-11-11 |
| Driver tick mean 10.41-10.46 ms at `PNU_OPTS` 1, 3 and 7, from 13.09-13.53 ms; no catch-up burst longer than one tick | 2026-11-11 |
| PMU RTR and command 9.49-9.54 Hz per id, from 7.32-7.58; gimbal command 90.0-90.2 Hz per axis, from 69.4-71.8 | 2026-11-11 |
| All five step 2c cases give the same verdicts as V5.1.25; Start interlock 1.161-1.347 s, from 1.462-1.677 s | 2026-11-11 |
| Gimbal homing unchanged to the tenth of a degree (60.6 / 60.6 / 40.5, midpoints -0.1); SAR 100 bundles, 0 drops | 2026-11-11 |
| Slew limiter holds **45.00 deg/s** on every limited tick, 0 of 3238 steps over the limit, from 25.7-180.1 deg/s | 2026-11-11 |
| A 1.3 s interval yields one 2.2500 deg step and no catch-up; a zero-length interval yields 0.000000 deg | 2026-11-11 |
| Homing, fault latching and feedback loss at 0.51 s unchanged across the slew-limiter change | 2026-11-11 |
| `PMUE` correct in all five step 2c scenarios - 7 records for the whole suite, including the cut-short burst | 2026-11-11 |
| `PMUE` bounded: 199 consecutive failing `ENGONOFF` ticks give **19** records, and `Mode` stays 0 throughout | 2026-11-11 |
| `PMUC` at 1.000 Hz with and without a GCS; `CmdTx`/`RtrTx` exactly 0.800; 8 injected short frames give `Short` 8 | 2026-11-11 |
| Scheduler unchanged by the two new records: 20 long loops, 3332 us max, identical to V5.1.27 | 2026-11-11 |

### Verified by build and unit test only

- Five optional-feature guards compile out on every CAN change: `AP_PNU_CAN_ENABLED`,
  `AP_PNUGIMBAL_ENABLED`, `AP_PNUGIMBAL_CUBEMARS_ENABLED`, `HAL_MOUNT_PNUGIMBAL_ENABLED`,
  `HAL_MOUNT_ENABLED`.
- 10 CubeMars codec tests, 12 SAR navigation codec tests against the ICD's own byte vectors, and
  five Python suites for the fake devices (17 + 26 + 29 + 23 + 13).

### Never verified

- **No CubeMars motor has ever turned.** No SAR hardware has ever been connected. No KGCS has
  driven TC50. The shared bus has never run on real CAN.

---

## 7. Open Issues Summary

Full detail in `ISSUES.md`.

| ID | Title | Blocks |
|---|---|---|
| `GIMBAL-001` | The driver assumes the motors feed back autonomously | **step 3f** |
| `GIMBAL-002` | Homing parameters are provisional; `PNU_HOME` stays 0 | step 3f, first flight with the gimbal |
| `GIMBAL-004` | Seven things the CubeMars manual does not specify | the bench session |
| `SAR-001` | `0x404` carries no measured attitude | step 3f, and Lumir's integration |
| `GCS-002` | KGCS must send TC50 from a permitted system id | SAR commanding from the GCS |
| `PMU-004` | The driver tick ran long. **Fixed in 5.1.26, SITL only** - the target was never measured | the bench session: CAN rates, step 2c timing, CPU, shared bus |
| `PNUCAN-001` | The gimbal slew limiter divided by a nominal rate. **Fixed in 5.1.27, SITL only** | the bench session: realised rate at the motor, and whether 45 deg/s suits the payload |
| `PNUSAR-003` | `SNAPSHOT_MAX_AGE_MS` was widened to 100 ms to absorb `PMU-004` | nothing - re-measure on the bench |
| `CAM-001` | The Viewpro tracking freeze can strand the operator | - |
| `CAM-002` | Viewpro zoom maxima are hard-coded and unverified | - |
| `D10`, `D11` | Camera routing and the MAVLink camera capability gap | KGCS work only; FC side complete |

**Pending decisions** needing PNU: the two step 2c deviations (`PMU-001`); whether the IR
prelude is needed at all (D1); the
10 m / 15 m avoidance thresholds (D12); whether KGCS should time out TM2; and whether any serial
port uses NMEA output.

---

## 8. Development Rules and Constraints

### Working rules

- **Never stage or commit.** Staging and committing are done manually after review. This matters
  mechanically: `git checkout <tree> -- <path>` writes to the index. Apply snapshot files with
  `git show v5.0.8-snapshot:<path> > <path>` instead.
- **Board is CubeOrangePlus and `-Werror` is ON.** `./waf copter` must finish with **zero warnings
  and zero errors**; anything else is a regression. Check after every change. If a build is
  configured for SITL to run a test, **configure it back**.
- **`./waf heli` is blocked on purpose.** Nothing in the PNU code is frame-dependent, so the heli
  target would otherwise build happily and produce a flashable binary with the wrong motor output
  and swashplate handling. A `#error` in `ArduCopter/config.h` stops it. Remove it deliberately -
  and re-validate the PMU, mount and failsafe behaviour - if a helicopter airframe is ever adopted.
- **`git diff v5.0.8-snapshot` is NOT a to-do list.** 30 files already differ deliberately. The
  per-file register is in `CHANGELOG.md` Appendix A.
- **AGENTS.md applies** to AI-assisted contributions: never claim untested work, never invent APIs
  or parameters, never change parameter indices, respect the `#if AP_<FEATURE>_ENABLED` guards,
  and keep the base system compiling with optional features disabled.

### The MAVLink dialect

`modules/mavlink` is left **completely pristine**. The 9 PNU message definitions live in
`Forced Submodule File/pnu_kal.xml` and are staged outside the submodule:

```sh
Tools/pnu/stage_mavlink_defs.sh            # stage; safe to re-run
Tools/pnu/stage_mavlink_defs.sh --check    # exit 1 if missing or stale
```

**Re-run the script after any `modules/mavlink` change** - a stale stage builds happily against
last month's upstream definitions. **The staging directory must never be committed.** See D3.

### Aircraft parameters

| | |
|---|---|
| CAN bus | `CAN_D1_PROTOCOL = 15` (PMU, gimbal and SAR share it), `CAN_P1_DRIVER = 1`, `CAN_P1_BITRATE` default 1,000,000 |
| Devices fitted | `CAN_D1_PNU_OPTS` - bit 0 PMU, bit 1 gimbal, bit 2 SAR. **Default 1** (PMU only) |
| SAR Start interlock | `CAN_D1_PNU_SCHK` - bit 0 RTK Fixed, bit 1 GPS fix age, bit 2 gimbal attitude age, bit 3 navigation health. **Default 11** |
| Gimbal | `MNT1_TYPE = 15`; operating limits in `MNT1_*_MIN/MAX`; mechanical limits in `CAN_D1_PNU_MRLL/MPIT/MYAW`; `CAN_D1_PNU_HOME` **0** |
| KGCS telemetry port | `SERIALx_PROTOCOL = 2` (MAVLink2) - **never 1.** Every PNU-KAL ICD message is id >= 50001, which MAVLink1 cannot encode. See D2 |
| Viewpro mount | `MNT1_TYPE = 11`, `CAM1_TYPE = 4`, `SERIALx_BAUD = 115`, `SERIALx_PROTOCOL = 8` |
| Gremsy mount | `MNT1_TYPE = 6`, `CAM1_TYPE = 6`, `SERIALx_BAUD = 115`, `SERIALx_PROTOCOL = 2`. **Set the VIO's camera component to 100**, not its default 101. **Set `MNT1_YAW_MIN/MAX` explicitly** - `GCS-003` |
| Pitch limits | `MNT1_PITCH_MIN = -(degrees of DOWN travel)`, `MNT1_PITCH_MAX = +(degrees of UP travel)`. Viewpro **-90 / 45**; Gremsy VIO **-90 / 15** |

**`PNU_OPTS` and the gimbal parameters do not exist until the driver is allocated.** That needs
`CAN_Dn_PROTOCOL = 15`, `CAN_Pn_DRIVER = n` (**defaults to 0 = Disabled** on CubeOrangePlus) and
**a reboot**. A GCS parameter refresh before the reboot shows nothing; that is the step most
likely to look like a bug when it is not.

**`PNU_OPTS = 0` still allocates the driver.** It has to: otherwise the subgroup would vanish and
the bitmask could never be set back.

### Architectural rules that must not be undone

- **`PNU_CAN`, never `PNUCAN`.** The underscore is the only thing distinguishing it from `PMUCAN`
  at a glance in a parameter dropdown or a statustext. **M and N are one letter apart in a
  sans-serif font.** Do not "tidy" it away.
- **Retired parameter index 7 and protocol 17 are never reused.** AP_Param stores by index, not by
  name, so a reused index reads a saved value back as a different parameter; a reused protocol
  number silently becomes a different device.
- **Keep `PMU_Status` and `PMU_Ctrl_Echo` declared in `AP_PMUCAN.h` beside their definitions, and
  outside the feature guard** (D14). That is what lets the vehicle files stay unchanged and what
  lets TM50 compile with no CAN driver at all.
- **`AP_Mount_Backend` carries no PNU-specific pitch logic.** Re-adding a sign flip there
  double-negates the KGCS paths and inverts pitch for every standard GCS. The negation lives in
  `AP_Q30` at the call sites:

  | Path | Reversed |
  |---|:-:|
  | KGCS angle (TC2 `Control_Mode` 2) - `AP_Q30::send_cmd_angle()` | **yes** |
  | KGCS rate (TC2 `Control_Mode` 1) - `AP_Q30::send_cmd_speed()` | **yes** |
  | TM2 pitch readback - `GCS_PNU.cpp` | **yes** |
  | MAVLink angle commands, RC sticks, ROI, rate integration | no - **standard physical convention** |

  This is a KGCS operator convention, not a protocol requirement: pressing pitch-down should pan
  the *video image* down, which means the gimbal pitches up.
- **A core, non-optional component must never depend on an optional one.** Five guards are checked
  on every CAN change; `HAL_MOUNT_ENABLED=0` must keep building (D13).

### Supported GCS and gimbal combinations - decided 2026-10-14

*PNU-GCS is replacing KGCS. KGCS is a closed project and **whitelists MAVLink message ids**,
discarding or erroring on anything it does not already know.*

| Ground station | Gimbal | Interface | Status |
|---|---|---|---|
| **KGCS** | **Viewpro only** | TC2/TM2 (PNU-KAL ICD) | supported - the legacy path, must keep working unchanged |
| KGCS | Gremsy, or any MAVLink gimbal | - | **NOT SUPPORTED. Do not retest this** |
| **PNU-GCS** | any | TC2/TM2 (KGCS interface) | supported - the `AP_Q30` path; four functions are GCS-direct on a non-Viewpro |
| **PNU-GCS** | any | standard MAVLink2 | supported - `AP_Mount` / `AP_Camera` handlers; `AP_Q30` is not involved at all |
| Mission Planner, QGC | any | standard MAVLink2 | supported |

**Why KGCS + Gremsy cannot work, so nobody spends a day rediscovering it.** Tried on the bench
2026-10-14: KGCS connects, then its data stream stops after a few seconds. A Gremsy adds message
ids KGCS has never had to parse - `GIMBAL_DEVICE_ATTITUDE_STATUS` (285) from ArduPilot, and from
the payload everything routing forwards: `HEARTBEAT` from three components (26, 100, 154),
259, 260, 283, 285, 253, and on request 261, 262, 269, 271, 322, 324, 397, 132, 248. **This is not
a firmware defect and there is nothing to fix in the FC.**

**Do NOT set `MAVn_OPTIONS` bit 1 (`NO_FORWARD`) on the VIO's link.** It would break PNU-GCS:
**`PARAM_EXT_ACK` (324) carries no target fields** and is therefore broadcast, so it would be
blocked on the way back while the commands still got through. Every palette and source change
would succeed silently and look like a failure.

**Per-channel message suppression exists and is narrower than it looks.** A GCS can stop
ArduPilot sending any single message on its own link with
`MAV_CMD_SET_MESSAGE_INTERVAL (511), param1 = <msgid>, param2 = -1` - but **it stops ArduPilot's
own messages only, not forwarded payload traffic**, because `MAVLink_routing::forward()` never
consults stream intervals. On a Gremsy, msg 285 reaches the GCS from two independent sources and
this stops only the first, which is the **worst** outcome: the message still appears, at an
irregular rate, from a different component id. It is also **not persistent** (RAM only, re-send
after every reboot) and **a stream-rate parameter change undoes it** (285 lives in
`STREAM_EXTRA3`). The recommendation on record is **do not suppress 285** - see `ISSUES.md`.

### Dual-GCS design rules

A standard MAVLink GCS must keep full control of the gimbal and camera. KGCS gets as much of the
TC2/TM2 function set as the payload allows. Where the two cannot both be served, **the standard
path wins** and the KGCS-specific behaviour moves into `AP_Q30`.

1. KGCS-specific convention belongs in `AP_Q30`. Never in `AP_Mount`, `AP_Camera` or their
   backends.
2. If a shared library genuinely must behave differently, gate it on an **opt-in backend
   capability** - the D8a `suppress_gimbal_device_attitude_status()` precedent.
3. A TC2 function with no FC path goes **GCS to payload direct**, not into new FC code.
4. Check return values and report (D10 Stage 1) rather than diverging silently.
5. Anything new on the wire must be additive and ignorable by a standard GCS.

### Testing against the fake CAN devices

`Tools/pnu/sitl_can/` holds fake PMU, SAR and CubeMars gimbal devices that run on a Linux `vcan0`
beside SITL, and `run_all3.sh` starts all three plus SITL in the order that works. **The order is
not optional** - the CAN driver attaches during boot, so a fake started after SITL misses the
conversation meant to bring it up. That directory's `README.md` has the procedure, the console
reference and the measured results.

**A green SITL run is not evidence about hardware.** `vcan` performs no arbitration, the gimbal's
feedback trigger is the fake's assumption, and no PMU, SAR or motor firmware is modelled.

### Bench test order when hardware is next available

1. **Gimbal, before anything else:** the feedback trigger, then the rate (`GIMBAL-001`), then the
   homing tuning in `GIMBAL-002`, then zero persistence across a power cycle.
2. **Shared bus:** `PNU_OPTS = 3`, then 7, with the PMU health rewrite under real contention.
3. **Viewpro + KGCS** with the new pitch parameters - direction correct, full nadir and 45 deg up
   both reachable.
4. **Viewpro + Mission Planner** - `GIMBAL_MANAGER_SET_PITCHYAW` down now moves the gimbal down,
   where it previously moved up. **This is the real test of the 5.1.2 pitch relocation**; the
   Gremsy bench session did not exercise it.
5. **Any AUTO mission with a gimbal pitch item**, since the relocation silently changes its
   behaviour.
