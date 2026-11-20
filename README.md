# PNU-KAL FLCC OFP - Development Version

| | |
|---|---|
| **Current version** | `PNU_OFP V5.1.30` (DEV) |
| **Baseline** | ArduPilot Copter **4.7.1** |
| **Last official release** | `PNU-KAL-FLCC-5.1.0` (same code as 5.1.0 dev) |
| **Vehicle** | **ArduCopter only.** Plane, Rover and Sub do not link - by construction |
| **Documents** | `CHANGELOG.md` - version history and verification. `ISSUES.md` - issues, risks and decisions. `PMU CAN Comm Protocol.md` - the PMU/ECU CAN ICD, V2.0 **Draft**. `SAR_ICD_PNU.md` - the SAR ICD, Rev1.0 |

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
`CAN_P1_BITRATE` stays at its default and **the PMU is configured on the device to match** -
ours arrived at 500 k. The rate is a bus property on both PMU generations, never a way to tell
them apart; `VERSTS` does that.

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

**PMU V2.0 ran end to end on a real ECU on 2026-11-19**, ground and then flight, on V5.1.29.
Engine commands, ECU telemetry, the MAVLink path and a `TC60`/`ENERGYCFG` Tank/Fuel
transaction all worked, against a **development** ground station built to exercise
`pnu_kal.xml` - not the production KGCS. That was integration, not protocol conformance:
section 6 lists what it established and `PMU-005` lists what it did not.

**V5.1.30** makes the logs readable and fixes the one column that was wrong. `README.md`
sections 9 and 10 now document **every PNU dataflash record field by field** - the twelve
PMU and ECU records, the three SAR records, and what `MNT` and `MSG` do and do not say
about the gimbal, which writes no record of its own. Writing that down found three defects
in the records themselves. **`SARN.Seq` logged the sequence counter rather than the byte
packed into `0x404` B6**, so every row read one higher than the wire - an off-by-one in the
single column a dispute with Lumir over a missing bundle would be settled on; `_att_seq_tx`
now mirrors B6 the way `_att_flags` already mirrors B7. `TM13.PMRV` and `TM11.BSTS` are
recorded as permanently zero rather than left to look like data. The ECU r3 ICD, the MAVLink
dialect and the fake PMU were also brought into line, including the `fuel == tank` read-back
blind spot, now reproduced in SITL. **No flight behaviour changed in this version** - one
logged value, documentation, and a simulator. See `CHANGELOG.md`.

**V5.1.29** puts the ECU on the KGCS link. `PMU-005a` stages 4 and 5 add three messages -
`TM60 51060` (ECU modes, 10 Hz), `TM61 51061` (ECU energy, 1 Hz) and `TC60 50060`
(ENERGYCFG, operator action only) - and `pnu_kal.xml` now documents them to the standard a
ground station can be written from without the CAN ICD. **Not one byte of what a legacy
aircraft transmits changed**: all three are gated on VERSTS having positively confirmed ECU
software 2.00 or higher, so a legacy PMU, an unconfirmed fallback and a build with the ECU
compiled out are all byte-identical on the link to V5.1.28, and no existing message id,
field or CRC_EXTRA moved. `TC60` runs one bounded transaction at a time: at most three
identical CAN frames, verified against status frames that arrived after the ICD's mandatory
one-second wait, with the outcome latched in TM61's two extension fields and recorded as
`ECUT` and `ECUX`. `BATSTS.GAGE` is decoded for the first time, for the SOC read-back. **When this version was
recorded the entire ECU side was SITL-only against a fake built from the ICD.** It is not
any more - this is the firmware that was run against a real ECU on 2026-11-19, ground and
flight. `MODESTS` remains a V2.0 draft carrying no layout version. See section 6 and
`CHANGELOG.md`.

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
| PMU | Full telemetry, commands, prearm, in-flight failsafe, engine-start interlock with two guards. **Bench-verified** except the shared bus and Stop-command timing. Both generations are decoded as of `PMU-005a` stages 1-3 - VERSTS selects, HOURCNT follows the sender's width, MODESTS and ENERGYSTS are polled and logged - and stage 4 sends them to KGCS as `TM60`/`TM61`, only once VERSTS has confirmed ECU 2.00+. `ENERGYCFG` is stage 5's `TC60`, one readback-verified transaction at a time. **The ECU side is hardware-validated end to end** - ground and flight on a real ECU, 2026-11-19, against a *development* GCS. Integration, not protocol conformance: see section 6 |
| Gimbal | Mount backend, lifecycle state machine, Appendix 1 homing, per-axis direction parameters. **SITL only - no motor has ever turned** |
| SAR | Connection layer, both MAVLink messages, the command layer with the Start interlock, the 10 Hz navigation bundle with automatic termination, and `SARN` bundle accounting. **SITL only - no SAR hardware** |
| Camera | Viewpro via the KGCS TC2/TM2 interface; Gremsy VIO via standard MAVLink with four functions GCS-direct. **Bench-verified on both**, though the TM2 producer moved to `AP_Q30::update()` in 5.1.25 and that move is SITL-verified only |
| Logging | Every PNU telemetry record is written by the subsystem that produces it, at a rate independent of how many ground stations are attached. **The rate-independence measurement is SITL only.** Dataflash BIN logs were saved and reviewed from the 2026-11-19 ECU tests, but a ground station was attached throughout, so that run does not test the property |

---

## 4. Current Development Plan

**ECU V2.0 support, Stages 0-5**, approved 2026-11-13 and recorded as `PMU-005a` in
`ISSUES.md`. One `AP_PMUCAN` backend with a latched protocol generation, one version branch in
the AUX2STS decode and a separate module for the two new ECU messages - no Legacy/ECU backend
split. Stage 0 is the minimal fake ECU in `Tools/pnu/sitl_can/`, because no ECU hardware was
available when the plan was approved and nothing downstream was testable without a device
that speaks V2.0.

**Stages 0-5 are done.** The fake ECU, the VERSTS generation selection, the HOURCNT width
branch and `AP_PMUCAN_ECU` - MODESTS at 10 Hz and ENERGYSTS at 1 Hz on the scheduling phases
the five legacy slots leave free, behind `AP_PMUCAN_ECU_ENABLED`, logged as `ECUM`, `ECUE`
and `ECUC`. The real legacy PMU was bench-checked on 2026-11-13 and answers VERSTS at v1.00
with a zero AUX2STS reserved byte, so legacy hardware selects the legacy path on its own.

Stage 4 added `TM60 51060` (ECU modes, 10 Hz) and `TM61 51061` (ECU energy, 1 Hz). **Neither
is transmitted unless VERSTS has positively confirmed ECU software 2.00 or higher**, so a
legacy aircraft - with or without SAR and the gimbal - is byte-identical on the link to the
firmware before stage 4, and the existing KGCS keeps working untouched. No existing message
definition or CRC_EXTRA changed.

**Stage 5 is done too.** `TC60 50060` carries the ENERGYCFG CAN payload itself - four raw
little-endian UINT16, `0xFFFF` meaning keep - and the FC performs no arithmetic on it. One
operator action starts one bounded transaction: at most three identical frames, verified
against `ENERGYSTS`, `ENGSTS.GAS` and `BATSTS.GAGE` frames that arrived after the ICD's
mandatory one-second wait, with the outcome in TM61's two extension fields and in the new
`ECUT`/`ECUX` records. A second TC60 during a transaction is refused, never queued and never
allowed to overwrite the first. On a legacy PMU and during the unconfirmed fallback, TC60 is
refused out loud and no CAN frame is sent.

**PMU-005a is complete, and validated end to end on a real ECU.** Ground and flight tests on
**2026-11-19**, on V5.1.29, exercised the GCS -> FC -> ECU command path including the step 2c
interlock, ECU telemetry reception and decode, `TM1`/`TM3`/`TM60`/`TM61` on the link, and a
`TC60`/`ENERGYCFG` transaction whose Tank and Fuel changes came back in the reported values.
VERSTS reported **2.00** from the device, so `PROTO_ECU_V2` was selected by a real ECU rather
than reached through the fallback. The ground station was a **development build, V1.0.1
Pre-release, not the production KGCS**. `AP_PMUCAN` is unchanged between V5.1.29 and V5.1.30,
so that evidence applies to this build as it stands.

**That was integration, not conformance.** `MODESTS` is still a draft with no layout version
and its field order still has no bit-level capture behind it; the r3 edge cases, the
`fuel == tank` read-back blind spot, `GAS` rounding, arbitrary Battery/SOC settings and the
three questions for the ECU author are all untouched by it. The run was `PNU_OPTS = 1` at
1 Mbps, so the shared bus is no closer to being verified. See section 6 and `PMU-005`.

Everything else is at a clean point: every applied step is recorded, and step 3f remains
blocked on measurements only PNU can make.

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
- **Bench the shared bus.** Every PMU bench run so far had `PNU_OPTS = 1`, the 2026-11-19 ECU
  ground and flight tests included; the id-based health rewrite has only ever run in SITL.
- **Put three questions to the ECU author** - `PMU-005`. A reliable completed-stop indication,
  whether `GCMD` reports the requested or the clamped setpoint, and the undefined `MODESTS`
  codes. The third one now has a decoder behind it: stage 3 logs `MODESTS` raw, and the frame
  carries no layout version. **2026-11-19 weakened the field-order doubt without closing it:**
  a real ECU's `MODESTS` gated a `TC60` transaction and displayed sensibly across a whole
  ground and flight session, which is inference from plausible values, not a bit-level capture
  checked against the author's definition. The undefined codes are untouched by it.
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
| **ECU V2.0 end to end.** GCS -> FC -> ECU engine commands - Start, Stop, `Engine_CHK_CMD`, `Battery_Control_CMD`, `Engine_Manual`, `Engine_Throttle_CMD` - and the ECU -> FC -> GCS telemetry path, both directions, on the ground and then in the air | bench + flight 2026-11-19 |
| **VERSTS reports 2.00 from a real device**, so `PROTO_ECU_V2` is selected by an ECU rather than reached through the unconfirmed fallback | bench 2026-11-19 |
| `TM1`, `TM3`, `TM60` and `TM61` received and displayed - existing PMU fields and the new ECU fields alike. **Reception and display only**, not calibration or per-field accuracy | bench + flight 2026-11-19 |
| Step 2c **permit** path on real hardware: Start accepted about 5 s after Stop, inside the 10 s window. The refusal paths were **not** exercised; this project's record for them is SITL - 2026-10-29 and 2026-11-11 - and no hardware run of them is recorded here | bench 2026-11-19 |
| `TC60`/`ENERGYCFG` transmitted to a real ECU, with Tank and Fuel changes reflected in the reported values. **Battery and SOC were sent as `0xFFFF` (keep) only**, deliberately, because a wrong value there could make the ECU misbehave | bench 2026-11-19 |
| Fuel estimate over **one** 15-minute flight: 3 L initial, 2 L actual remaining, 2 L ECU-estimated - agreement at the reported resolution. One observation, one condition, 1 L of resolution | flight 2026-11-19 |
| Conditions for every 2026-11-19 row above: FC **V5.1.29**, development GCS **V1.0.1 Pre-release**, `CAN_Dn_PNU_OPTS = 1`, **1 Mbps**, dataflash BIN logs saved and reviewed | 2026-11-19 |
| PMU telemetry, KGCS and FC command paths, prearm with the PMU absent | bench 2026-10-19 |
| **In-flight PMU failsafe** - CAN connector pulled while armed, RTL taken immediately, clean recovery on reconnect. **Legacy PMU**, not an ECU; not retested on 2026-11-19 | bench 2026-10-19 |
| `ARMING_SKIPCHK` bit 13 bypass and the one-shot startup report, both after the 5.1.16 fixes | bench 2026-10-19 |
| Engine interlock duration, ~1 to 1.5 s against an expected ~1.2 s. **Legacy PMU**, not an ECU, and a permitted Start | bench 2026-10-19 |
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
- **No production KGCS has driven anything.** The 2026-11-19 ECU validation used a development
  ground station, V1.0.1 Pre-release, built to exercise the `pnu_kal.xml` interface. That an
  independent implementation parsed and displayed every message is evidence about the interface
  definition - `TM60`'s zero CRC_EXTRA included - and not about the production KGCS.

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
| `GCS-004` | The ECU interface is validated against a **development** GCS, not the production KGCS | production KGCS integration |
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

---

## 9. PMU Log Records

Every PMU and ECU dataflash record the firmware writes, with what each column holds and
where it comes from. **Raw counts, as received** - the scale column is what to multiply by,
and the FC applies none of it. Field names are what appears in the log, so this table is the
only thing needed to read one.

Written from the CAN driver thread, not from the telemetry path: the log does not depend on
whether a ground station is attached, which is what `V5.1.25` fixed. The ECU records exist
only when `AP_PMUCAN_ECU_ENABLED` is built in.

### 9.1 Legacy and shared - written on both PMU generations

**`TM11`** - battery, engine and hours, 10 Hz.

| Field | Source | Type | Scale | Meaning |
|---|---|---|---|---|
| `HOURCNT` | AUX2STS 0-2 legacy, 0-3 ECU | UINT24 / UINT32 | 1 s | Accumulated engine running time. **Seconds, despite the name** |
| `DATE` | VERSTS 0-3 | UINT32 | BCD | PMU software date, BCD `YYYYMMDD` |
| `IBAT` | BATSTS 4-5 | INT16 | 0.1 A | Battery current. **Positive charging, negative discharging** |
| `TEMP` | BATSTS 2-3 | INT16 | 0.1 degC | Battery temperature |
| `BSTS` | - | - | - | **Always 0.** The BATSTS decode is commented out; the ICD calls the field reserved |
| `RPM` | ENGSTS 4-5 | UINT16 | 1 rpm | Engine speed |
| `TEMP1` | ENGSTS 0-1 | INT16 | 0.1 degC | Cylinder head 1 temperature |
| `TEMP2` | ENGSTS 2-3 | INT16 | 0.1 degC | Cylinder head 2 temperature |
| `GCMD` | AUX1STS 6-7 | INT16 | 0.1 % | ECU: accepted `BATCTRL` SOC setpoint, 84% = raw 840. **Not a copy of the command integer** |

**`TM12`** - power, status word and versions, 10 Hz.

| Field | Source | Type | Scale | Meaning |
|---|---|---|---|---|
| `VBUS` | AUX1STS 4-5 | INT16 | 0.1 V | System voltage |
| `ILD` | AUX1STS 2-3 | INT16 | 0.1 A | Load current, described as always positive |
| `ICMD` | AUX1STS 0-1 | ECU UINT16, legacy TBD | ECU 0.1 A | ECU: EMS charge-current target. **Legacy type and scale are undefined in the ICD** |
| `PTEMP` | AUX2STS 6-7 | INT16 | 0.1 degC | PMU temperature |
| `PSTS` | AUX2STS 4-5 | UINT16 | bitfield | PMU status and warnings - see 7.4 |
| `PCL` | ENGSTS 7 | INT8 | 1 % | ECU: throttle **setpoint**, not a measured position. Legacy: throttle position |
| `GAS` | ENGSTS 6 | INT8 | 1 % | Remaining fuel. **ECU: an estimate from RPM/load and the configured inventory, and 100 may be the uninitialised placeholder** - see 7.3 `ECUM.IFl` bit 0x20 |
| `MN` | VERSTS 4 | UINT8 | BCD | PMU software minor version |
| `MJ` | VERSTS 5 | UINT8 | BCD | PMU software major version. **`MJ` 2 or higher selects the ECU message set** |

**`TM13`** - version identity, 10 Hz.

| Field | Source | Meaning |
|---|---|---|
| `FLMJ` | `OFP_VER_MAIN` | FLCC firmware major - 5 |
| `FLMN` | `OFP_VER_SUB` | FLCC firmware minor - 1 |
| `FLRV` | `OFP_VER_REV` | FLCC firmware revision - **30 as of V5.1.30**, and the value `TM1` reports to KGCS |
| `PMRV` | - | **Always 0.** Nothing in the tree writes `Version_REV_Number`; the PMU reports no revision field |

**`TM3`** - the command echo and link health, 10 Hz. The five command columns carry `TC1`'s own
names so commanded and echoed values line up.

| Field | Meaning |
|---|---|
| `EGOF` | `Engine_OnOff` as committed by the step 2c interlock |
| `BCTC` | `Battery_Control_CMD`, the SOC setpoint |
| `EGM` | `Engine_Manual`, 1 manual / 0 automatic |
| `ETRC` | `Engine_Throttle_CMD`, percent, manual only |
| `ECHK` | `Engine_CHK_CMD`, choke: 1 close / 0 open |
| `FAIL` | PMU link health: **0 CONNECTED, 1 COMMUNICATION_ERROR, 2 CONNECTION_FAILURE** |

**`PMUC`** - cumulative communication counters, 1 Hz. **Totals since boot**, so the useful
reading is a difference between two records.

| Field | Meaning |
|---|---|
| `RxSts` | Recognised status frames decoded |
| `RtrTx` | RTR polls sent - **the five legacy slots only**; ECU polls are counted in `ECUC` |
| `RtrErr` | RTR polls that could not be sent |
| `CmdTx` | Command frames sent. `CmdTx`/`RtrTx` is **0.800** in steady state: four of five slots carry a command |
| `CmdErr` | Command frames that could not be sent |
| `Short` | Status frames dropped for a DLC below 8 |

**`PMUE`** - one record per engine-interlock decision, event driven. Not periodic.

| Field | Meaning |
|---|---|
| `Mode` | Engine state the FC holds **after** this event: 0 OFF, 1 ON |
| `OnCnt` | `_engineoncnt` as it stood **before** the event reset it |
| `OffCnt` | `_engineoffcnt`, likewise |
| `Armed` | 1 once a valid Stop pair has been seen this boot |
| `StopAge` | ms between the last valid Stop pair and the instant this attempt began - the quantity guard 2 tests - or **-1** when `Armed` is 0 or the event is not a Start decision |
| `Evt` | **Append-only** event code: 1 START_CMD, 2 STOP_CMD, 3 START_NO_STOP, 4 START_STALE_STOP, 5 TC1_STALE, 6 START_TX_FAIL, 7 STOP_TX_FAIL |

**`TC1`** - every PMU command from KGCS, as received, one record per message. Columns match
`TM3`'s five.

### 9.2 Commanding - written when a command arrives

**`ECUT`** - every `TC60` at ingress, **including ones refused for their source**, which are
the ones somebody will be looking for afterwards.

| Field | Scale | Meaning |
|---|---|---|
| `Tank` `Fuel` | 0.01 L | As received. **65535 means keep the current value** |
| `Batt` | 0.1 Ah | As received, or 65535 |
| `SOC` | 0.1 % | As received, or 65535 |
| `SYS` `COMP` `CHAN` | - | Sender system and component id, and the MAVLink channel it arrived on |

**`ECUX`** - one record per `ENERGYCFG` transaction state change, and one per accepted or
refused command. **The `c` columns are what was commanded, the `r` columns what the ECU read
back** at that moment, so a failure is diagnosable from the log alone.

| Field | Scale | Meaning |
|---|---|---|
| `Res` | - | `Cfg_Result`: 0 NONE, 1 IN_PROGRESS, 2 OK, 3 MISMATCH_AFTER_3, 4 CANCELLED_MODE_CHANGE, 5 REJECTED_RANGE, 6 REJECTED_MODE, 7 REJECTED_NOT_ECU, 8 TX_FAIL |
| `Att` | - | ENERGYCFG frames actually put on the bus, 0-3. **0 for anything refused before transmission** |
| `cTank` `cFuel` | 0.01 L | Commanded, 65535 = keep |
| `cBatt` | 0.1 Ah | Commanded, 65535 = keep |
| `cSOC` | 0.1 % | Commanded, 65535 = keep |
| `rTank` | 0.01 L | Read back from ENERGYSTS |
| `rBatt` | 0.1 Ah | Read back from ENERGYSTS |
| `rGas` | 1 % | Read back from ENGSTS - an **integer percent**, which is why `Res` 2 cannot confirm 0.01 L |
| `rSOC` | 0.1 % | Read back from BATSTS `GAGE` |

### 9.3 ECU only - written when `VERSTS` selects the ECU generation

**`ECUM`** - MODESTS as received, 10 Hz. **Raw codes; an unlisted value is unknown and is not
mapped onto the nearest defined one.** Written only once a reply has been decoded.

| Field | Meaning |
|---|---|
| `SSt` | `system_state`: 1 INITIALIZATION, 2 READY, 3 ENGINE_RUN, 4 USER_SHELL |
| `OMd` | `operation_mode`: 1 DO_NOTHING, 3 STANDBY, 4 STARTUP, 5 RUNNING, 6 STOP. **0 and 2 undefined.** The two that accept `ENERGYCFG` are 1 and 3 |
| `CMd` | `ctrl_mode`: 1 NONE, 2 MANUAL, 3 STARTUP, 4 STOP, 5 FIXED_RPM, 6 EMS, 7 INACTIVE, 8 FAIL_TACHO, 9 FAIL_CURR_SENSOR, 10 FAIL_VOLT_SENSOR |
| `Alrt` | `alert_level`: 0 NONE, 1 ADVISORY (plan to land), 2 IMMEDIATE (land now) |
| `EMd` | `ems_mode`: 0 INIT, 1 START, 2 WARMUP, 3 CHARGE, 4 IDLE, 5 DISCHARGE, 6 EMERGENCY, 7 STOPPING, 8 THERMAL_HOLD, 9 FORCED_CHARGE |
| `Gate` | `start_gate`, result of the last start attempt: 0 OK **or no attempt since power-on**, 4-13 BLOCKED_*, 15-18 ABORTED_*, 19 BLOCKED_CURRENT_FAULT. **1-3 and 14 undefined.** Full names in `pnu_kal.xml` |
| `IFl` | `initial_fail` bitmask: 0x01 HEADER_BAD, 0x02 EEPROM_LAYOUT, 0x04 CALIB_MISSING, **0x20 FUEL_LEVEL - while set the ECU substitutes a full tank and `TM12.GAS` reads 100 as a placeholder**. 0x08/0x10/0x40/0x80 undefined |
| `RPMT` | `rpm_target`, 1 rpm. **Zero means not governing RPM, not a confirmed engine stop** |

**`ECUE`** - ENERGYSTS as received, 1 Hz. Written only once a reply has been decoded.

| Field | Scale | Meaning |
|---|---|---|
| `Tank` | 0.01 L | Configured tank capacity |
| `Batt` | 0.1 Ah | Configured battery capacity |
| `Endr` | 1 s | Remaining endurance, **with two sentinels that override the scale: 65534 means 65534 s or more, 65535 means unknown**. The ECU reports 65535 whenever no fuel is burning |

**`ECUC`** - the ECU link's own counters and the selected generation, 1 Hz. **Written on both
generations**, which is what keeps the selection in the log when no ground station is
attached to hear the boot statustext.

| Field | Meaning |
|---|---|
| `Proto` | Raw latched selection: **0 UNKNOWN (fallback in use), 1 LEGACY_PMU, 2 ECU_V2** |
| `RtrTx` `RtrErr` | ECU-only polls sent and failed. **Separate from `PMUC` by design** - on a legacy aircraft this stops at 2, the boot frames sent before VERSTS answered |
| `ModeRx` `EnrgRx` | MODESTS and ENERGYSTS replies decoded |
| `Short` | ECU replies dropped for a DLC below 8 |
| `CfgAcc` `CfgBsy` | `TC60`s that started a transaction, and ones refused because one was already running |

### 9.4 `PSTS` bits - `TM12.PSTS`, from AUX2STS

| Bit | Set means | Bit | Set means |
|---:|---|---:|---|
| 15 | Battery overvoltage | 7 | ECU STALL_UNDER_LOAD; legacy unintended stop |
| 14 | Battery undervoltage | 6 | Engine RPM above threshold |
| 13 | Battery over-discharge | 5 | Reserved |
| 12 | ECU CRITICAL_FUEL, below 5% | 4 | Cooling fan running |
| 11 | ECU GENERATOR_OVERLOAD | 3 | Choke closed |
| 10 | Cylinder head over-temperature | 2 | Stop sequence in progress |
| 9 | Running-time limit | 1 | Start sequence in progress |
| 8 | Low fuel | 0 | Manual operation |

Thresholds are ECU parameter defaults and are listed in `PMU CAN Comm Protocol.md`. Bits 12,
11 and 7 are reserved on the legacy PMU.

---

## 10. PNUGimbal and PNUSAR Log Records

The two are documented together because they are one subsystem in everything but the class
name: both ride the same `AP_PNU_CAN` bus, both are enabled by `CAN_Dn_PNU_OPTS` bits, both
share the 40 ms slot budget, and **step 3f joins them outright** - the gimbal's encoder
feedback is what is supposed to fill the SAR attitude frame `0x404`. A log question about
one is usually a log question about both.

**This subsystem logs far less than the PMU does, and the asymmetry is deliberate reading
material, not an oversight to gloss over.** SAR writes three records; **PNUGimbal writes
none at all**. What that means in practice is set out in 10.5 to 10.7.

**Every record below needs an open log, which disarmed means `LOG_DISARMED`.**
`AP_Logger::should_log()` refuses unless the vehicle is armed or `log_while_disarmed()` is
set, and a `Write()` that reaches no backend is discarded. This bites harder here than on
the PMU: gimbal homing is **skipped when the vehicle is armed**, so the entire homing
sequence falls in the window where nothing is logged by default.

The three SAR records use `AP::logger().Write()`, which applies **no `LOG_BITMASK` gate at
all** - unlike `MNT`, which needs `MASK_LOG_CAMERA` (bit 15). The trade is criticality:
`WriteV()` defaults `is_critical` to false, so `SARS`, `SARN` and `SARC` are **dropped first
when the log buffer runs short**, whereas `MNT` and `MSG` are written with
`WriteCriticalBlock()` and keep a reserved share of it.

### 10.1 What the subsystem actually writes

| Record | Written by | Rate | Thread | Gated on |
|---|---|---|---|---|
| `SARS` | `AP_PNUSAR::log_status()` | 1 Hz **and on every status change** | CAN driver | open log only |
| `SARN` | `AP_PNUSAR::log_nav()` | 10 Hz, **only while a session is transmitting** | vehicle (`sar_nav_snapshot`) | open log only |
| `SARC` | `handle_gcs_flcc_sar_ctrl()` | one per `TC50`, **including refused ones** | MAVLink receive | open log only |
| `MNT` | `AP_Mount_Backend::write_log()` | 10 Hz | vehicle (`ten_hz_logging_loop`) | **`MASK_LOG_CAMERA` and `should_log()`** |
| `MSG` | `GCS::send_textv()` | per statustext | caller's thread | open log only, **first 50 characters** |
| *(none)* | `AP_PNUGimbal_*` | - | - | - |

"Open log only" still means armed, or `LOG_DISARMED` set. All five are compiled out without
`HAL_LOGGING_ENABLED`.

### 10.2 `SARS` - connection state

Written from the driver's own tick since `V5.1.25`. It used to be a side effect of the `TM50`
telemetry sender, which duplicated it once per attached ground station - **440 of 662 records
in one SITL run were duplicates under 5 ms apart** - and omitted it entirely when none was
attached, so an aircraft flying on a dead link kept no SAR history at all.

| Field | Type | Meaning |
|---|---|---|
| `STAT` | UINT8 | ICD `SAR_STATUS`, section 14.1: **0 Disconnected, 1 Ready, 2 Stopped, 3 Started, 4 SAR Not Responding.** The internal `START_PENDING` / `STOP_PENDING` states are not MAVLink states and never appear here |
| `LIVE` | UINT32 | FC-maintained count of **valid** `Heart_Beat` receptions since boot |

A status change resets the periodic slot, so a transition is never followed a millisecond
later by the record that was already due. **`STAT` 3 with a frozen `LIVE` is the diagnosis
to look for**: a session nominally running over a link that has stopped answering.

### 10.3 `SARN` - navigation bundle accounting

One record per 10 Hz snapshot, and **the absence of `SARN` is itself the finding** - nothing
is written unless navigation transmit is on, so a gap means no session was running.

**A `SARN` row means a bundle was built, not that one was sent.** `log_nav()` is called from
`nav_snapshot()` on the vehicle thread (`AP_PNUSAR_Nav.cpp:372`), after the nine frames are
encoded and `_bundle.ready` is set, and **before the CAN thread has touched them**. Read 10.3.1
before drawing any conclusion from this record about what reached the bus.

| Field | Type | Meaning |
|---|---|---|
| `TX` | UINT32 | Bundles whose **nine frames were all accepted by the CAN driver**, cumulative since boot. Submission, not confirmed transmission - see 10.3.1 |
| `Drop` | UINT32 | **The sum of the three below**, not a separate cause |
| `Ovr` | UINT32 | Vehicle thread overwrote a snapshot the CAN thread had not sent. **The overwritten bundle had already consumed its `ATT_Seq`** |
| `Stale` | UINT32 | CAN thread reached the snapshot later than `AP_PNUSAR_SNAPSHOT_MAX_AGE_MS`, **100 ms - widened to absorb PMU-004, see PNUSAR-003** |
| `TxF` | UINT32 | **Bundles** abandoned because the interface refused a frame. One per bundle, however many of the nine had already gone |
| `Seq` | UINT8 | **The `ATT_Seq` actually packed into `0x404` B6**, wrapping 0-255, so it lines up directly with a receiver log. It is `_att_seq_tx`, the same byte the frame carries - not the counter, which by then holds the next value |
| `AFlg` | UINT8 | `ATT_Flags` **as transmitted**, `0x404` B7: bit 0 roll valid, bit 1 pitch valid, bit 2 yaw valid, bit 3 fresh. **All four are zero until step 3f** - see SAR-001 |
| `TxErr` | INT16 | **This backend's** last refused frame, not the bus-wide value the PMU and gimbal also write. **1 means no SAR frame has ever been refused**; 0 is a genuine "TX queue full" |

#### 10.3.1 What `SARN` proves, and what it does not

The record sits at the first of five distinct stages, and conflating them is the easiest way
to misread a SAR log.

| Stage | Where | Observable in the log? |
|---|---|---|
| 1. Snapshot encoded, `ATT_Seq` consumed | `nav_snapshot()`, vehicle thread | **Yes - this is the `SARN` row itself** |
| 2. Transmission attempted | `send_bundle()`, CAN thread | Only afterwards, via the next row's `TX` / `Drop` |
| 3. All nine frames accepted by the interface | `send_frame()` returning true nine times | Next row's `TX`, which increments only after frame 9 |
| 4. Frames physically on the wire | CAN peripheral | **No. Nothing in the FC observes this** |
| 5. SAR received the bundle | Lumir's equipment | **No. A nav bundle is never acknowledged** |

Consequences, in order of how often they are got wrong:

1. **`TX`, `Drop` and `TxErr` describe the state before this snapshot**, because the row is
   written before the CAN thread runs. They lag `Seq` by one bundle.
2. **A `SARN` row with a given `Seq` does not mean that bundle was transmitted.**
   `send_bundle()` re-checks the age and drops a bundle older than 100 ms
   (`AP_PNUSAR_Nav.cpp:439`), and abandons the remaining frames the moment one is refused
   (`:446`). Either way `ATT_Seq` has already been consumed, so **the sequence number exists
   in the log and never existed on the bus.** The ICD comment at `:283` says exactly this: the
   counter advances so a dropped bundle "leaves a visible gap rather than vanishing".
3. **`TX` is submission, not transmission.** `AP_PNU_CAN::send_frame()` counts success as
   `CANIface::send()` returning 1 (`AP_PNU_CAN.cpp:627`), which means the frame was accepted
   into the driver's TX queue against a deadline, with `AbortOnError`. A frame that then
   expires, loses arbitration or dies to a bus-off **is not reported back**, and `TX` has
   already counted it.
4. **Nothing here speaks for the receiver.** The only evidence the SAR is alive is
   `SARS.LIVE`, the `Heart_Beat` count, and that is a link-level fact about the device, not an
   acknowledgement of any particular bundle.

So when Lumir says "we did not receive it", `SARN` narrows the question: **`Seq` against
their stream** identifies which bundle, and the next row's `TX` and `Drop` say whether the FC
got as far as handing it to the CAN peripheral. It cannot settle whether the bits left the
connector.

Builds up to and including `V5.1.29` logged the counter rather than the transmitted byte, so
**a log from one of those reads one higher than the wire** - subtract 1 before comparing it
with a receiver's stream.

### 10.4 `SARC` - every `TC50` from KGCS

Written **before** both the source check and the value test, so a command refused for its
sender is still in the log. It is the only record of who sent one.

| Field | Meaning |
|---|---|
| `CTL` | `Observation_Control`: **0 Prepare, 1 Start, 2 Stop.** 3-255 undefined |
| `MODE` | `Observation_Mode`: all 256 values allowed, **detailed mapping is TBD in the ICD** |
| `SYS` `COMP` | Sender system and component id |
| `CHAN` | MAVLink channel it arrived on |

**`COMP` is logged and deliberately not enforced.** The ICD names no permitted component id
and ground stations differ - Mission Planner is 190, KGCS need not be - so a filter would be
invented rather than specified. `SYS` *is* enforced, against `MAV_GCS_SYSID`: see GCS-002.

### 10.5 The gimbal writes no record of its own

`AP_PNUGimbal_Backend`, `AP_PNUGimbal_CubeMars` and `AP_PNUGimbal_Homing` contain **no call
to `AP::logger()` anywhere**. The only dataflash record touching the gimbal is the standard
ArduPilot `MNT`, written by the mount front end, and **both halves of it come from the same
command chain**. The CubeMars encoder feedback reaches it nowhere.

| `MNT` field | What it holds for this mount | Frame |
|---|---|---|
| `I` | Mount instance | - |
| `Mode` | `MAV_MOUNT_MODE` from `get_mode()` | - |
| `DRoll` `DPitch` | `mnt_target.angle_rad.roll` / `.pitch`, **unclamped**, and **only when the target type is `ANGLE`** - NaN in rate, retract, neutral and location modes | earth (`roll_is_ef` / `pitch_is_ef` default true) |
| `DYawB` *xor* `DYawE` | The target yaw, written to **whichever column matches `yaw_is_ef`**; the other is always NaN | per `yaw_is_ef` |
| `Roll` `Pitch` `YawB` | **NOT measured.** `AP_Mount_PNUGimbal::get_attitude_quaternion()` builds these from `_angle_bf_output_rad`, the angle the FC decided to command | body |
| `YawE` | `YawB` + the AHRS yaw, wrapped to +-180 | earth |
| `Dist` | Rangefinder distance, unused here | - |

**`DRoll` against `Roll` is not a tracking error and must never be read as one.** Both are
derived from the same demand; no motor measurement is involved. Four things, and only these
four, separate the two columns (`AP_Mount_PNUGimbal.cpp:117-182`):

1. **The earth-to-body rotation.** The AHRS lean, rotated into the gimbal's yaw frame, is
   subtracted from the target when `roll_is_ef` / `pitch_is_ef` - the stabilisation itself.
2. **The lead filter**, `MNT1_LEAD_RLL` and `MNT1_LEAD_PTCH`, zero by default.
3. **The operating-limit clamp**, `MNT1_ROLL_MIN/MAX` and the pitch pair. The desired columns
   are logged *before* it and the actual columns *after*, so a saturated axis shows as a
   difference that is a clamp, not a lag.
4. **A yaw frame change**, for `YawE` only.

Two cases need care. In **retract and neutral**, `update_angle_outputs()` returns early
(`:133-136`), so no stabilisation and no clamp are applied there - the only clamp those
targets ever see is the one `get_attitude_quaternion()` applies when the record is written.
In **rate mode**, the base class integrates the rate into an angle target and still calls
`send_target_angles()`, so the actual columns are populated while **the desired columns stay
NaN** - the target type is still `RATE`.

Read correctly, `DRoll` against `Roll` is what makes step 1b provable from a log with no
motors connected: the difference **is** the AHRS stabilisation correction the FC computed.
What no `MNT` row can show is whether a motor went anywhere near it.

### 10.6 Statustext is the gimbal's real log

Everything the gimbal knows about itself reaches the log as `GCS_SEND_TEXT()`, which
`GCS::send_textv()` passes to `AP_Logger::Write_Message()` and thence to the standard `MSG`
record. Searching `MSG.Message` for `PNUGimbal:` is how a gimbal session is reconstructed
today.

| Family | Severity | Carries |
|---|---|---|
| `PNUGimbal: <from> -> <to> (<why>)` | varies | **Lifecycle transitions: 0 INIT, 1 HOMING, 2 ALIGN, 3 RUN, 4 MAINTENANCE, 5 FAULT** |
| `PNUGimbal: <axis> axis <fault> (status 0x..)` | CRITICAL | Per-axis fault with the raw protocol status byte |
| `PNUGimbal: homing <axis> ...`, `... travel ... deg, mid ...`, `... homed` | CRITICAL / INFO | The Appendix 1 endpoint search, per axis |
| `PNUGimbal: <axis> feedback lost after N frames` | WARNING | Feedback timeout, with the count that preceded it |
| `PNUGimbal: feedback from motor id ?N (M dropped)` | WARNING | Frames from an unexpected motor id |
| `PNUGimbal: zero refused in <state>`, `PNU_HOME=0, zero not found` | WARNING | Homing refused or never performed |
| `PNUGimbal: no gimbal on CAN, check CAN_Dn_PNU_OPTS bit 1` | WARNING | Mount selected but no backend on the bus |

#### 10.6.1 When a statustext does and does not become a `MSG` record

**Sending a statustext does not guarantee a `MSG` record.** Four conditions, verified in
`GCS_Common.cpp:2539-2613`, `AP_Logger_Backend.cpp:471-514` and `LogFile.cpp:341-374`:

| | Behaviour |
|---|---|
| **No ground station attached** | **Still logged.** The `Write_Message()` call sits *outside* the block that gives up when the destination mask is empty, so a statustext with nowhere to send is written to the log anyway. This is the opposite of the `TM50` problem `V5.1.25` fixed |
| **No `LOG_BITMASK` bit** | **Not gated at all.** `Write_Message()` applies no mask - but it still needs an open log, so disarmed it needs `LOG_DISARMED` |
| **Log buffer under pressure** | **Survives longer than most records.** `MSG` is written with `WriteCriticalBlock()`, which keeps a reserved share of the buffer. It is still dropped if the buffer has no room for the packet at all |
| **Early boot, from the CAN thread** | **Can be lost.** `ShouldLog()` refuses any non-main-thread write until the startup message writer has finished, and every `AP_PNUGimbal` and `AP_PNUSAR` statustext originates on the CAN driver thread |

And one that always applies:

**Only the first 50 characters are logged.** `GCS::send_textv()` copies the formatted text
into a `MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN`-sized buffer before handing it to the logger,
so the link gets the whole string in chunks while `MSG` keeps 50 bytes. Two existing messages
already exceed it - `PNUGimbal: no gimbal on CAN, check CAN_Dn_PNU_OPTS bit 1` (56) and
`PNUGimbal: PNU_HOME=0, zero not found - limits are relative` (59) - and both lose their tail
in the log.

Rate limiting, where it exists, is in the **callers**, not in the GCS or logger layer:
`report_tx_error()`, `report_short_frame()` and `report_bundle_drop()` each hold their own
timer, and several gimbal notices are one-shot per boot.

This is adequate for a bench session and **inadequate for flight analysis**: the messages
carry no numbers that can be plotted, lose anything past 50 characters, and depend on a log
being open at the moment they are emitted - which, for homing, means `LOG_DISARMED`. It is
the same gap that `V5.1.25` closed for `SARS` and the PMU records, still open on the gimbal.

### 10.7 Decoded, timestamped, and never logged

`AP_PNUGimbal_Feedback` is populated per axis on every accepted CAN frame and **nothing
consumes it**. Each of the three axes already holds:

| Member | Meaning |
|---|---|
| `position_rad` `velocity_rads` | Measured angle and rate from the encoder |
| `torque_nm` | Measured torque |
| `driver_temp_c` `motor_temp_c` | Drive and motor temperature, degC |
| `status` | Raw protocol status code, **not normalised** |
| `last_update_ms` `frame_cnt` `valid` | Freshness, accepted-frame count, and whether the axis has ever reported |

This is the same data step 3f needs for `SARN.AFlg` and `0x404`, and it is why GIMBAL-001
blocks that step: **if the motors answer only when commanded rather than streaming, the
driver needs a keep-alive and step 3f's 40 ms budget has to be re-derived.** Until that is
measured on real hardware, neither a gimbal record nor a populated `AFlg` can be specified
honestly - no CubeMars motor has ever turned.

### 10.8 Records that look like gimbal records and are not

`TC_R`, `TC_C` and `CM_C` belong to the **Q30 / Viewpro camera gimbal** (`AP_Q30`), a serial
device on a different bus with its own ICD messages `TC2` and `TM2`. They have nothing to
do with `AP_PNUGimbal` or the CubeMars drives, and confusing the two when reading a log is an
easy mistake to make given that both are "the gimbal".

| Record | Device | Content |
|---|---|---|
| `TC_C` | Q30 camera | Every `TC2` from KGCS as received: roll/pitch/yaw speed and angle demands, control mode, zoom, shutter, tracking |
| `CM_C` | Q30 camera | What the FC decided to send the camera: **gimbal command type 0 none, 1 rate, 2 angle, 3 fix/hold**, plus zoom, focus, record, track and IR actions |
| `TC_R` | Q30 camera | Camera attitude readback: relative, IMU and RC-target angles per axis, and zoom position |
