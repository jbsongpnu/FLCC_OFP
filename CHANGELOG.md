# CHANGELOG - PNU-KAL FLCC OFP

Version history and verification record for the PNU-KAL FLCC OFP development branch,
newest first. The active status and plan are in `README.md`; the issue register is in
`ISSUES.md`.

## How to read the verification entries

Verification is labelled by the strongest evidence that actually exists. These are not
interchangeable and must never be promoted:

| Label | Means |
|---|---|
| **Build** | compiles, including the optional-feature guards. No execution |
| **Unit test** | C++ gtest or Python test, offline, no vehicle |
| **SITL** | executed in simulation, usually against the fake CAN devices in `Tools/pnu/sitl_can/` |
| **Bench** | executed against real hardware on the ground |
| **Flight** | executed in the air |

A feature exercised only in SITL is **not** hardware-verified. Entries say so explicitly.

---

## V5.1.23 - the gimbal driver never brought the motors up

**Changes**

- None. This version is one defect fix.

**Bug Fixes**

- **`AP_PNUGimbal` transmitted nothing at boot.** `AP_PNUGimbal_Backend::_state` is
  initialised to `State::INIT`, and `set_state(State::INIT)` early-returns when the state
  is unchanged, so INIT's entry actions - the per-axis clear-errors and the enable - were
  never performed. No feedback could arrive, `all_axes_enabled()` stayed false, and INIT
  timed out into FAULT after 5 s, whose own entry queued a disable. **Those three disables
  were the only frames the motors ever saw.** `init()` now performs the entry actions
  through a shared `enter_init()`.
- **Wrong since the lifecycle state machine landed in 5.1.13.** It was invisible because
  every statustext that would have explained it fires in the first seconds after boot -
  before any GCS has connected and before the dataflash logger has started. Same shape as
  the one-shot startup line that went missing in 5.1.16.
- The `HOMING` enum comment in `AP_PNUGimbal_Backend.h` still said "STEP 1d, NOT
  IMPLEMENTED"; corrected. The measured-angle TODO in `AP_Mount_PNUGimbal.cpp` was
  relabelled from step 1c to step 3f.

**Verification**

- **SITL**, against the fake gimbal on `vcan0`. Found by the fake gimbal on its very first
  run: the only frames in a 47 s run were three `disable` universals at ~5 s.
- **SITL**: the Appendix 1 homing sequence then ran end to end for the first time - roll
  travel 60.6 deg, pitch 60.6, yaw 40.5, each midpoint -0.1, zero set per axis, then
  `HOMING -> ALIGN -> RUN` in 40 s. The excess over the true 60 and 40 is the modelled stop
  compliance, which consumes 0.6 deg of `PNU_HTOL`'s 3 degree default.
- **SITL**: `CAN_Dn_PNU_OPTS = 7` - PMU, gimbal and SAR on one bus - ran for the first
  time, with no demultiplexing complaint.
- **Build**: CubeOrangePlus **1,737,920 B, 228,152 free**.

**Known Limitations**

- **No gimbal hardware.** No CubeMars motor has ever turned. Everything about the gimbal
  remains SITL-only.
- The homing figures are the fake's stop-compliance model, not a measurement of the real
  mount. See `GIMBAL-002` in `ISSUES.md`.
- **The driver assumes the motors feed back autonomously.** With feedback only in reply to
  a command it faults 523 ms after the enable, against the 500 ms
  `AP_PNUGIMBAL_FB_TIMEOUT_MS`. See `GIMBAL-001` in `ISSUES.md` - this decides step 3f.

---

## Tools, 2026-11-03 - three fake devices from one script (no version bump)

No firmware file changed.

**Changes**

- `Tools/pnu/sitl_can/run_all3.sh` starts the fake gimbal, SAR and PMU, then SITL, and
  holds until interrupted. **The order is the point**: the CAN driver attaches to the bus
  during boot and the gimbal driver sends its enable within the first few hundred
  milliseconds, so a fake started after SITL misses the conversation meant to bring it up.
  Each fake's console stays reachable through a FIFO (`run_all3.sh say gimbal fault 0 2`);
  `--scenario` runs a script and tears everything down afterwards.
- `Tools/pnu/sitl_can/mav_tap.py` logs the FC's statustexts and every `SAR_STATUS` change.
  It exists because **the dataflash logger opens about six seconds after boot**, so
  `PNU_CAN: CAN_D1 PMU+Gimbal+SAR ready` and everything the gimbal says leaving INIT were
  previously unrecoverable on a run with no ground station. It parses MAVLink2 by hand: no
  dependencies, and pymavlink cannot decode TM 51050 because it is in the PNU dialect.

**Bug Fixes**

- **The fake PMU burned a core when SITL went away.** pymavlink's `mavtcp` prints
  `EOF on TCP socket` and calls `reconnect()` on every zero-length read, and a blocking
  `recv_match` keeps reading for its whole timeout, so one second of a closed link is
  thousands of reconnect attempts and thousands of log lines - 7,455 of them in one second
  when SITL was stopped under a running fake. The watcher now traps EOF and reconnects at a
  human rate. This matters because restarting SITL with the fakes left running is the normal
  workflow.

**Verification**

- **SITL**, `run_all3.sh --duration 70 --gimbal-home`, `CAN_D1_PNU_OPTS = 7`, no ground
  station. Times are from the tap's connect, about a tenth of a second after SITL starts:

  | | |
  |---|---|
  | FC | `PNU_CAN: CAN_D1 PMU+Gimbal+SAR ready` at 0.235 s, `SAR connected` at 0.819 s, `[PMU] Connected!` at 1.222 s - 34 statustexts in all |
  | gimbal | all three axes homed at 14.3 / 28.7 / 40.2 s, then `HOMING -> ALIGN -> RUN`; 10,437 frames out, ~5,184 commands per axis in, 0 wrong-mode, 0 unknown ids |
  | SAR | 71 heartbeats, `SAR_STATUS` 1 Ready throughout |
  | PMU | 2,592 RTRs seen, 2,592 answered |

  The homing numbers match the gimbal-only run of 2026-10-30 to a tenth of a degree, so the
  PMU and SAR traffic did not disturb it.
- An earlier busy-bus run with the SAR actually observing produced **280 complete navigation
  bundles, 0 discarded, 9.99 Hz**, alongside 3,496 gimbal commands per axis and 8,082
  feedback frames, with no wrong-mode frames, no unknown ids and no PreArm failure.
- **Unit tests**: `test_mav_tap.py`, 13 tests. All five Python suites and flake8 pass.

**Known Limitations**

- **vcan performs no arbitration**, so none of this says anything about contention on a real
  1 Mbit/s wire.
- The gimbal answered with `--fb-mode periodic` because that is what the driver needs. The
  real feedback trigger is still unmeasured - `GIMBAL-001`.
- `0x404` carried sentinels in every bundle. The gimbal is running and feeding back while
  the SAR bundle flows, and the attitude frame in it is still empty, because nothing joins
  the two. That is step 3f - `SAR-001`.

---

## Tools, 2026-10-30 - the fake CubeMars gimbal (no version bump)

**Changes**

- `Tools/pnu/sitl_can/gimbal_frames.py` (wire format only), `gimbal_model.py` (three axes:
  impedance following and compliant hard stops), `fake_gimbal.py` (SocketCAN I/O and the
  operator console). Step 2b's other half, written long after the fake PMU it was supposed
  to accompany.
- `--fb-mode per-command|periodic|off` with `--fb-rate` makes the **feedback trigger
  selectable**, because the manual specifies none. The fake says loudly which one it is
  using, every time it starts and in every status line.
- Compliant stops with `--stop-stiffness`: a rigid stop would make homing look easier in
  SITL than it will be on the bench and would leave `PNU_HRTOL` untested. It is a test knob,
  not a measurement - nobody has measured this mount.

**Bug Fixes**

- Two defects in the fake's own model, found by its stop tests: the compliant-stop deflection
  was computed from the *achieved* velocity, producing a limit cycle where the axis chattered
  in and out of the stop forever; and a travel assertion used a magic tolerance instead of a
  figure derived from the stall torque. Both fixed.

**Verification**

- **Unit tests**: 23, including the manual's published command vectors. One test
  **deliberately pins a known error in that manual** - the published MIT speed vectors decode
  to 4.74 and -4.84 rather than the +/-6 they claim, because the manual's transmit function
  uses `2^N` where its receive function uses `2^N - 1`.
- **SITL**: found the 5.1.23 firmware defect on its first run.

**Known Limitations**

- Kinematics only: no stop compliance measurement, no stall thresholds, no real motor
  dynamics, no CubeMars firmware behaviour. Faults are console-injected.

---

## V5.1.22 - SAR Integration Plan step 3e: the 10 Hz navigation bundle

**Changes**

- Nine frames `0x400`-`0x408` every 100 ms while an observation runs, 72 payload bytes per
  logical snapshot, plus ICD 14.7 **automatic termination**.
- **The snapshot is taken on the VEHICLE thread**, in a new 10 Hz Copter `SCHED_TASK`, and
  the CAN thread only writes out what it is handed. `AP_GPS::update()` runs on the vehicle
  thread, so a snapshot taken there cannot straddle a GPS update - the same twenty-odd
  fields read from the CAN thread could have built one bundle from two different samples,
  which is what ICD 7.3's "same acquisition time" forbids. Encoding where the data is read
  makes that true by construction rather than by hoping the window is narrow.
- **`0x404` carries SENTINELS, not zeros** - `INT16_MIN` for Roll and Pitch, `0xFFFF` for
  Yaw, `ATT_Flags` empty. ICD 16.4 asks for sentinels when actual attitude is unavailable,
  and zeros would read as a level, north-facing SAR to anything that checks values before
  flags. **`ATT_Seq` still advances**, so a bundle lost before the bus leaves a visible gap.
- **`CAN_Dn_PNU_SCHK` now governs ICD 14.7 as well as 14.3.3.** The same bits gate the Start
  interlock and the matching automatic-termination conditions, because a bench allowed to
  Start without RTK would otherwise have its observation torn down 100 ms later. **The
  vehicle's own navigation/EKF failsafe is not gated** and always terminates.
- **An automatic termination is not a normal Stop.** Navigation goes off first, then `0x502`
  if the heartbeat is still live, and its response or its 2 s timeout is an **internal
  result** - never `SAR_STATUS` 2 or 4. The published state is ICD 14.8's recovery-pending 0,
  which a reconnect does not release.
- **Two small public accessors added to `AP_GPS`**: `get_rtcm_stats()` for `0x407`, which was
  private, and `have_gps_yaw_accuracy()` so `YAcc` carries the unavailable sentinel instead
  of `gps_yaw_deg()`'s 10 degree fallback - a distinction ICD 6.2 asks for by name. Both are
  `const`, four lines each, change no behaviour, and are a clean upstreamable commit on their
  own. The ICD's `RTCM_STATS_AVAILABLE` bit exists for exactly the private-field case, and 3e
  could have shipped with the bit clear and both counters zero, which ICD 6.5 defines as
  correct - but **the FC has the numbers**, so PNU's decision 2026-10-30 was to add the accessor
  rather than transmit a blank field for no reason. **Everything else the ICD needs was already
  public**: `get_undulation()` for `AEl`, `last_message_time_ms()` and
  `last_message_delta_time_ms()` for `SMS` and `Delta`, the accuracy and DOP getters, and
  `gps_yaw_deg()` for yaw.

**Design decisions recorded with the change**

| Decision | Why |
|---|---|
| `ATT_Seq` advances per **snapshot**, not per transmission | A bundle lost before it reached the bus then leaves a visible gap at the receiver. A dense counter would hide exactly the event worth seeing |
| DGPS (4) does **not** terminate | 14.7 lists "ordinary 3D_FIX (3) or lower", and the ICD leaves in-observation DGPS explicitly TBD (section 10). Nothing is invented for it |
| a failed `send_frame` **abandons** the rest of the bundle | The receiver discards a bundle with a missing id anyway (7.3), so the remaining frames would only congest the bus on their way to being thrown away |
| the snapshot is **one slot**, not a queue | 7.4 discards unsent bundles rather than holding them. A queue would deliver stale data after a gap, which is worse than a gap |

**Bug Fixes**

- **A 50 ms staleness window dropped 6% of bundles.** The first run reported
  `SAR: bundle stale, 1 of 21` and then went quiet, while `ATT_Seq` showed **7 of 118 bundles
  missing** over 12 s. Two faults: the window was tighter than the driver thread's 10 ms loop
  actually keeps in SITL, and **a snapshot overwritten before the CAN thread reached it was
  not counted as a drop at all** - it would have shown up only as an `ATT_Seq` gap at the far
  end, which is the silent failure PNU-ISSUE(D6) exists to remove. The window is now one full
  bundle interval, still far inside the ICD's 300 ms source-age limit, and an overrun is
  counted and reported. **Loss after the fix: zero.**

**Verification**

- **Unit tests**: `libraries/AP_PNUSAR/tests/test_pnusar_nav.cpp` runs the ICD's own section
  9.1 byte vectors against the firmware's packer, not against a second implementation written
  for the test. It also covers the document's `-1.235 -> -124` rounding rule, the
  `36000 -> 0` angle rule, out-of-range becoming the sentinel rather than saturating, and NaN.
  **12 tests.** 29 Python tests for the fake SAR.
- **SITL**, fake SAR and fake PMU on one `vcan0`, `CAN_D1_PNU_OPTS = 5`:

  | Drive | Result |
  |---|---|
  | Start, `SCHK` at its default | first bundle **5 ms** after `0x521`; the fake assembled it under ICD 7.3's boundary rules |
  | the decoded bundle | `Lat 35.2345000 Lng 129.0789000 Alt 10.10 m AEl 10.10 m`, `Status 6 RTK_FIXED NSats 10 I 0`, `GMS 459864600 GWk 2442`, `SMS/Delta 200 ms`, `GPS_Flags 0x6F` |
  | `0x404` on the wire | `00 80 00 80 FF FF nn 00` - Roll/Pitch `INT16_MIN`, Yaw `0xFFFF`, `ATT_Flags` empty, as decided |
  | `GPS_Yaw` and `YAcc` | both **n/a**: the sim u-blox has no heading, and the 10 deg fallback was correctly suppressed |
  | rate over 12 s | **9.99 Hz**, 119 bundles, `ATT_Seq` 1:1 with the bundles received - no gaps |
  | `Delta` | **200 ms** - the sim GPS is 5 Hz and consecutive bundles reuse the sample, the case ICD 7.2 calls out |
  | normal Stop | bundle count frozen at 140 across the next 3 s |
  | GPS cut, `SCHK=11` | `SAR: observation aborted, GPS age` **294 ms** later, then `SAR: auto Stop confirmed (1)` |
  | GPS cut, `SCHK=9` (age check off) | `SAR: observation aborted, fix degraded` 3.88 s later, when AP_GPS dropped the fix |
  | the same abort with the device mute | `SAR: auto Stop unanswered (1)` exactly **2.02 s** later, and **TM50 stayed at 0** - not 4 |
  | heartbeat loss mid-observation | `SAR disconnected` 3.03 s after the last beat, bundles stopped at 80 and stayed at 80 across the reconnect; no `0x502` attempted |
  | a Start after any abort | `SAR: Start blocked, recovery pending` - ICD 14.8, and the only way back is a power cycle |

- **Build**: CubeOrangePlus **1,737,892 B, 228,184 free** (+2,108 B over 5.1.21), SITL, and
  SITL with `AP_PNU_CAN_ENABLED=0` - with no CAN driver the 10 Hz task still compiles and
  does nothing, which is what the entry point outside the feature guard is for. flake8 clean.

**Known Limitations**

- **No SAR hardware and no KGCS.**
- `0x404` is sentinels until 3f, so a receiver has never yet seen a valid attitude from this
  FC - `SAR-001`.
- The ICD 15 Float-interval bookkeeping is post-processing and deliberately absent from the
  FC (15.4 says Float alone adds no automatic Stop).
- Nothing exercises a Primary GPS change at a bundle boundary, because SITL was run with one
  GPS.

---

## V5.1.21 - SAR Integration Plan step 3d: the observation command layer

**Changes**

- `0x501`/`0x502` go out, `0x521`/`0x522` come back, `SAR_STATUS` reaches 2, 3 and 4, and
  TC50 is no longer recorded-and-ignored. `gcs().SAR_Ctrl_Seq` finally has a consumer: the
  GCS layer stores the command and increments, the backend reads the increment once from the
  CAN thread and acts.
- **`SAR_STATUS` is now COMPUTED, not assigned.** ICD section 8 states a *priority* between
  the sources - disconnected or recovery-pending (0) outranks the current attempt's timeout
  (4), which outranks the last confirmed state (1/2/3) - and with five states and two latches,
  last-writer-wins would quietly violate the document. One `publish_state()` decides, so
  "a heartbeat must not overwrite 2/3/4 with Ready" (14.1) is **true by construction** rather
  than by remembering it. **Step 3a's single `_recovery_pending` flag had to split in two**:
  the reconnection notification fires once, the inhibition stays latched until the next power
  cycle. In 3a the state stayed Disconnected only because nothing wrote it again.
- **The FC opens a 2 s window and NEVER retries** (ICD 14.5 prohibits it - an FC retry loop
  would exceed the GCS's two-attempt limit). A window opens only for a command **actually
  transmitted**, so a refused send does not become state 4; a new command clears the previous
  latch and republishes `LAST_CONFIRMED_STATE` while it waits; an expired response is never
  counted retroactively. **A failed Start leaves navigation OFF, a failed Stop RETAINS it**,
  because stopping is unconfirmed.
- **New `CAN_Dn_PNU_SCHK`**, the ICD 14.3.3 Start interlock as a bitmask at parameter index
  30 - bit 0 RTK Fixed, bit 1 GPS fix age, bit 2 gimbal attitude age, bit 3 navigation health.
  **Default 11 is the ICD's set less the attitude age.** It exists because a bench or an
  integration test has no RTK fix and could otherwise never send one Start. **Relaxing it is
  announced** once per boot and again on every Start under it, and it is read live - no reboot.
- **Stop is never gated** (ICD 14.4.2, in bold there): stopping a payload producing
  contaminated data must not depend on the quality of that data.
- **Four frames, one payload byte `0x01`.** The ICD says in its own words not to encode Stop
  as `0x02` merely because it is Stop, so one constant serves commands and responses alike. A
  response carrying anything else is refused and the window left to expire.
- **A TC50 that can reach no device now says so** (`SAR: no backend, check PNU_OPTS`). Without
  it the GCS's own 3 s window would report the SAR as not responding, blaming the payload for
  a configuration fault on this aircraft - PNU-ISSUE(D6).

**Design decisions recorded with the change**

| Decision | Why |
|---|---|
| the FC never retries | ICD 14.5 prohibits it outright - an FC retry loop would exceed the GCS's two-attempt limit (14.6), which is the GCS's to own |
| a window opens only for a command **transmitted** | 14.5's own wording. A frame the interface refused never reached the device, so letting it expire into state 4 would blame the SAR for a fault on this side of the connector |
| a failed Start leaves nav OFF, a failed Stop **retains** it | 14.5. Stopping is unconfirmed, and the device may well still be transmitting |
| an unmatched response is counted and discarded | The protocol has no transaction id and ICD 3.1 accepts that. Whether anything is waiting is the only thing that distinguishes a fresh response from a late or duplicated one |
| the timeout message is **not** the ICD's wording | "SAR not responding to Start" belongs to the GCS and means the whole two-attempt transaction failed (14.6). This is one FC window expiring, and the GCS may still hold a retry |
| a repeated Start is forwarded, not suppressed | 14.3: the device answers `0x521/01` again without toggling. Deciding it is redundant is not the FC's job |
| `0x503` stays a named id and nothing else | ICD 13 asks for a stub only. With no defined serialization there is nothing to stub that would not be invented - Mode is not placed at B0 and eight zero bytes are not sent |

**Verification**

- **SITL**, fake SAR and fake PMU on one `vcan0`, `CAN_D1_PNU_OPTS = 5`:

  | Drive | Result |
  |---|---|
  | boot, default `SCHK` | `SAR: ATT age check off, no source`; with `SCHK=0` at boot, also `SAR: Start checks relaxed, SCHK=0` |
  | Start, ICD set, sim GPS healthy | allowed, `0x501` `dlc 1 01` on the wire, `0x521` back in **58 ms**, `SAR Start confirmed, state 3` |
  | Stop while Started | `0x502` out, `0x522` back, `SAR Stop confirmed, state 2` |
  | `SCHK=4` (attitude only) | `SAR: Start blocked, no ATT source`, **and nothing on the bus** |
  | `SCHK=2`, 1.5 s after the GPS was cut | `SAR: Start blocked, GPS fix stale` |
  | `SCHK=1`, once the driver dropped the fix | `SAR: Start blocked, no RTK Fixed` |
  | `SCHK=11`, no GPS at all | `SAR: Start blocked, no RTK Fixed` |
  | `SCHK=11`, GPS restored | allowed, and no bypass line - the permitted path, not only the refusals |
  | `SCHK` changed by `PARAM_SET` mid-run | took effect on the next Start, **no reboot** |
  | `respond off`, then Stop | `SAR: no Stop response, nav ON` at **2.03 s**; TM50 `status=3 -> status=4` at **+2.14 s** |
  | the GCS retry | state **returned to 3** within 0.14 s and timed out to 4 again at +2.17 s - the latch cleared, `LAST_CONFIRMED_STATE` republished while waiting |
  | unsolicited `0x521` with no window | `SAR: unmatched 0x521 (1)`, state unchanged |
  | `0x522` carrying `0xFF` | `SAR: 0x522 payload 0xFF`, the window stayed open and expired honestly into state 4 |
  | Start and Stop while disconnected | `SAR: Start blocked, disconnected` / `SAR: Stop blocked, disconnected`, nothing transmitted |
  | Start after reconnect | `SAR: Start blocked, recovery pending` - ICD 14.8, and the TM stayed at 0 throughout |
  | TM50 rate across it all | **5.00 Hz** over 45 s |

- **SITL's simulated u-blox reports `GPS.Status = 6`** - RTK Fixed - for 206 of 207 dataflash
  records in the first run. That is what made the *permitted* path testable rather than only
  the refusals, and it is also why the first attempt at this measurement looked like a bug: a
  Start that was expected to be blocked was correctly allowed.
- **Build**: CubeOrangePlus **1,735,784 B, 230,288 free** (+1,268 B over 5.1.20), SITL, and
  SITL with `AP_PNU_CAN_ENABLED=0`. `test_fake_sar.py` at 22 tests, flake8 clean - the new one
  covers the deliberately wrong `0xFF` response payload, because a rule the fake is unwilling
  to break is a rule the FC never gets to refuse.

**Known Limitations**

- **No SAR hardware and no KGCS**, so the GCS-side 3 s windows and the single retry
  (ICD 14.6) are still PNU-GCS's to test.
- `NAV_TX_ENABLED` moves but nothing reads it until 3e.
- ICD 14.7's automatic termination is deliberately absent at this version, because its
  triggers are 3e's and a mechanism with no trigger is code nobody has run.

---

## V5.1.20 - SAR Integration Plan step 3c: the SAR's two MAVLink messages

**Changes**

- TC **50050** `SYS_ICD_GCS_FLCC_SAR_CTRL` and TM **51050**
  `SYS_ICD_FLCC_GCS_SAR_STATUS` in `Forced Submodule File/pnu_kal.xml`, taken from
  `SAR_ICD_PNU.md` 3.4 verbatim. Two deliberate departures from the house style of the other
  seven messages, both required by ICD section 3: **no `units="N/A"`** - the ICD says to omit
  the attribute rather than set it - and the ICD's own wording for every description, so that
  the FC dialect and Lumir's generated code are diff-able.
- TM50 is its own **5 Hz `SCHED_TASK`**, not a stream-group entry: ICD 3.2 fixes 200 ms
  "regardless of connection or observation state", a stream-group entry would be
  user-settable, and Copter's default stream rates are all zero. With no SAR backend in the
  build it reports Disconnected and a zero count, which is the truth in that configuration -
  this is what the D14 globals outside the enable guard were for.
- TC ingest checks its source with `sysid_is_gcs()`. TC50 has no `target_system` field, so
  ICD 3.1 asks the FC to check the sender. The component id is deliberately **not** filtered -
  the ICD names no permitted value - and is logged instead.
- `Observation_Control` 3-255 is **refused, not clamped** (ICD 3.1), and `Observation_Mode` is
  retained whole and never interpreted, because its CAN encoding depends on `0x503` and the
  manufacturer has not defined it.
- Every TC50 is logged as `SARC` with its sender, channel and the refusal that followed; TM50
  is logged as `SARS`.

**Generated CRC_EXTRA: `231` for TC50, `7` for TM50.** Both endpoints have to agree, and the
ICD refuses to assign unverified values, so they are recorded here. Re-derived independently
from the field list and matched against the firmware's generated headers. **TM50's fields are
seeded in WIRE order** - `Live_Counter` (uint32) at offset 0, `SAR_STATUS` (uint8) at offset 4
- which is not the order the XML lists them in; seeding in XML order gives 218 and is wrong.

**Verification**

- **SITL** with the fake SAR on `vcan0`, `CAN_D1_PNU_OPTS = 5`:

  | Step | Result |
  |---|---|
  | before any heartbeat | TM50 flowing, `status=0 Live_Counter=0` |
  | six heartbeats | `status=1 Live_Counter=6`, while the device's `Raw_Counter` read **777005** |
  | `short` (DLC 3) | `PNU_CAN: short frame 0x510 dlc 3`, and `Live_Counter` advanced by exactly 3 over the next 3 s - **the rejected frame did not count** |
  | `silent` | `SAR disconnected` 3.07 s later, `status=0`, counter held at 10 |
  | `resume` | `SAR reconnected, inspection pending`, counter continued to 14, **state stayed 0** |
  | TC50 `Control=0` Prepare | recorded, no device command and no message - ICD 8 |
  | TC50 `Control=1 Mode=7` | `SAR: Start not implemented yet`; the log shows `MODE=7` retained |
  | TC50 `Control=2` | `SAR: Stop not implemented yet` |
  | TC50 `Control=7` | `SAR: TC50 Control 7 undefined` |
  | TC50 from sysid 42 | `SAR: TC50 sysid 42 not MAV_GCS_SYSID`; the repeat 2 s later was logged but **not** announced again |
  | TM50 rate | **5.01 Hz** over 36 s |
  | dataflash | `SARC` has all six TC50s including both refused ones, with sender, component and channel; `SARS` has 181 records |

- **Build**: CubeOrangePlus **1,734,516 B, 231,560 free** (+644 B over 5.1.19), SITL, and
  SITL with `AP_PNU_CAN_ENABLED=0` - the last one is the point of the D14 globals: TM50 still
  compiles and still reports Disconnected when there is no CAN driver at all.

**Known Limitations**

- **No SAR hardware and no KGCS**, so the GCS-side 3 s windows and the single retry
  (ICD 14.6, PNU-GCS's work) are untested.
- Nothing yet consumes `SAR_Ctrl_Seq` - that is step 3d.
- **OPERATIONAL**: KGCS must send TC50 from a system id `MAV_GCS_SYSID` covers (default 255),
  or it is refused with a message naming the parameter. **TC1 never checked its sender**, an
  accepted residual of PNU-ISSUE(D7), so nothing before TC50 depended on that parameter being
  right. See `GCS-002` in `ISSUES.md`.

---

## Tools, 2026-10-30 - the fake SAR and the first shared bus (no version bump)

**Changes**

- `Tools/pnu/sitl_can/sar_frames.py` (the 16 ids and the four frames a SAR transmits, wire
  format only), `fake_sar.py` (SocketCAN I/O and the console), `test_fake_sar.py`.
- **There is no `sar_model.py`.** The PMU needed one because an engine has behaviour; at this
  stage the SAR beats once a second and answers commands, and the observation state machine
  belongs to 3d, where it lands with the FC side that drives it. Inventing it now would have
  meant inventing the parts the ICD leaves TBD.
- The fake answers `0x501`/`0x502` by default, with `--no-respond` to stay quiet and
  `--resp-delay` to move the answer inside or outside the FC's 2 s window.
- `--counter` starts `Raw_Counter` at 777000 in the validated run. ICD 3.3 says the FC's
  `Live_Counter` counts receptions and must never copy `Raw_Counter`; with the fake starting
  at zero the two would agree by accident and prove nothing.
- `intrude` transmits on an FC -> SAR id. `AP_PNUSAR` deliberately leaves those unclaimed, so
  the driver reports a second talker rather than swallowing it.

**Verification**

- **SITL**, the first shared-bus run this project has ever done - fake PMU and fake SAR
  together on one `vcan0` at `CAN_D1_PNU_OPTS = 5`:

  | Drive | What the FC said | Interval |
  |---|---|---|
  | boot | `PNU_CAN: CAN_D1 PMU+SAR ready` | - |
  | first heartbeat | `SAR connected` | 15 ms |
  | `silent` | `SAR disconnected` | **3.058 s** after the last heartbeat, deadline 3,000 ms |
  | `resume` | `SAR reconnected, inspection pending` | 16 ms, **state stayed Disconnected** |
  | `short` | `PNU_CAN: short frame 0x510 dlc 3 (1 dropped)` | - |
  | `param-resp` | `SAR: 0x523 reserved, payload still TBD (1)` | - |
  | `intrude` | `PNU_CAN: unclaimed frame 0x400 (1 seen)` | - |
  | the PMU throughout | `[PMU] Connected!`, **505 of 505 RTRs answered**, 404 commands in 25 s | - |

  A second run at **0.4 Hz** - 2.5 s between beats, inside `T_HB` - produced no disconnect
  across 11 heartbeats, and then `SAR disconnected` 3.027 s after the fake stopped. The
  deadline is a wall clock, as ICD 3.3 words it, and not "three missed beats".
- **Unit tests**: 21 at the time of writing, later 29. The ids and DLCs are checked against
  `SAR_ICD_PNU.md`'s own section 5 table **and** against `AP_PNUSAR.h`, because a fake that
  agrees only with itself proves nothing and the ICD is the document both sides signed.

**Known Limitations**

- **`Live_Counter` was still unobservable** at this point - it leaves the FC only through
  TM 51050, which is 3c.
- The command path was not driven from the FC, because nothing on the FC side transmits
  before 3d. Responses were covered by unit tests only.
- **No SAR hardware.** The 1 Hz on the wire, and its jitter, are the fake's.
- The gimbal was still absent, so `PNU_OPTS = 7` remained untested until the fake gimbal
  landed later the same day.

---

## V5.1.19 - SAR Integration Plan step 3a: the SAR becomes a backend

**Changes**

- **`AP_PNUSAR` is no longer an `AP_CANDriver`.** It is a backend selected by
  `CAN_Dn_PNU_OPTS` bit 2 on protocol 15, so the PMU, the gimbal and the SAR are three
  protocols behind one driver - which is the only arrangement `AP_CANManager` allows on one
  interface.
- **Protocol 17 is retired and NOT reused.** A stored 17 falls through to "no driver" rather
  than silently becoming a different device.
- The connection layer: Heart_Beat `0x510`, the FC-maintained `Live_Counter`, the 3 s `T_HB`
  timeout and `SAR_STATUS` 0 and 1.
- **`Live_Counter` counts what the FC RECEIVED**, not the SAR's own `Raw_Counter`, which the
  heartbeat carries and which this code never reads (ICD 3.3).
- **A reconnecting heartbeat does not restore Ready** (ICD 14.8). It is announced once and the
  state stays Disconnected, because the data contaminated by the outage has not been inspected
  and the release procedure is deferred by the ICD. Until it exists the only way back is a
  power cycle.
- Command, response and navigation ids are named and claimed already, so a real device's early
  traffic is counted rather than reported as unknown.
- `AP_PNUSAR: add the SAR interface control document, Rev1.0` added `SAR_ICD_PNU.md` to the
  repository root.

**Verification**

- **Build only.** Not tested beyond builds at this version; first exercised against the fake
  SAR by the tools work on 2026-10-30.

**Known Limitations**

- No SAR hardware; nothing had driven the connection layer at this version.

---

## V5.1.18 - CAN Update Plan step 2c: the two engine-start interlock windows

`AP_PMUCAN` only; nothing else is touched.

**Changes**

| Added | Where |
|---|---|
| `note_new_tc1()` | stamps both windows, called from `transmit_slot()`'s new-command branch |
| `tc1_is_stale()` | the 500 ms test |
| `start_permitted()` | the 10 s test, and the refusal message |
| `count_start_pair()` | `_engineoncnt++`, stamping the first increment of an attempt |
| `PMUCAN_TC1_STALE_MS` 500, `PMUCAN_STOP_WINDOW_MS` 10000, `PMUCAN_START_REFUSED_WARN_MS` 3000 | `AP_PMUCAN.h`, with the reasoning at the site |

- **TC1 staleness, 500 ms.** `engineonoffstate()`'s counters are a tick timer, not a count of
  operator alternations - the command pair only changes when a new TC1 arrives but the
  function runs at 10 Hz regardless, so a frozen pair kept counting and **a link dying
  mid-burst completed the start on its own about a second later**. Counting now stops.
- **Stop before start, 10 s.** KGCS forces a Stop before a Start, so the FC requires the same
  and refuses a Start that does not follow a recent one - covering the link dying *between*
  the two.
- **Both windows are stamped only where a NEW TC1 is detected**, and that is load-bearing. The
  obvious place to test the Stop alternation is `engineonoffstate()`, which already inspects
  the command pair - and it would be wrong. The pair is frozen between TC1 arrivals, so it
  stays a valid 4/5 Stop alternation from the end of one burst until the first message of the
  next. Testing it at 10 Hz would refresh `_last_stop_pair_ms` continuously and the 10 s window
  would never expire.
- **Constants, not parameters.** The design left this undecided and proposed constants. Both
  numbers come from the KGCS operating procedure rather than from tuning, and neither has room
  to move much, so a parameter would mostly offer a way to break the interlock from a ground
  station.
- **A refused Start reports itself**, rate limited to 3 s because the check is reached at 10 Hz
  for as long as the burst keeps arriving:

  ```text
  PMU: start refused, press Stop first              (no Stop seen this boot)
  PMU: start refused, Stop was 14s before it (limit 10s)
  ```

  **The first Start after any boot is refused until a Stop has been seen.** That is the KGCS
  procedure working as specified, not a defect - but it is the behaviour most likely to be
  reported as one, so the message exists to explain itself.

**Two deliberate deviations, both open for PNU's sign-off - see `PMU-001` in `ISSUES.md`**

- The Stop window is measured from the **Start press**, not from the moment permission is
  checked.
- The staleness guard is symmetric, so a **Stop** burst cut off mid-way no longer completes
  either.

**Bug Fixes**

- The refusal message was 53 characters. MAVLink STATUSTEXT caps at 50, so it arrived as two
  chunks and read as broken text on anything that does not reassemble them. Both messages are
  now under 50, and the reported age is clamped to keep it that way.
- (tools) The fake PMU exited 0.01 s after starting whenever stdin was not a terminal. EOF on
  a redirected stdin was treated as the console's "quit". Invisible when run by hand, fatal
  for any scripted run. EOF now means "there is no console here" unless stdin is a TTY, and
  SIGTERM quits.
- (tools) `tc1_inject.py` died instead of waiting when its endpoint did not exist yet - the
  same mistake as the takeoff watcher's in step 2b. It now waits, bounded by `--link-timeout`.

**Verification**

- **SITL**, driven by `Tools/pnu/sitl_can/tc1_inject.py`, which sends the TC1 bursts KGCS
  would send and **watches `vcan0` itself** for the `ENGONOFF` the FC emits - one process, one
  clock, so the interval needs no correlation between logs:

  | Scenario | Expected | Observed |
  |---|---|---|
  | `start-only` - first Start of a boot, no Stop | refused | **refused**, `PMU: start refused, press Stop first` |
  | `cut-start` - Start burst cut after 2 messages | no start | **no start** after 6 s of watching |
  | `late-start` - Stop, wait 14 s, Start | refused | **refused**, `PMU: start refused, Stop 14s ago (max 10s)` |
  | `stop-start` - the correct procedure | starts | **started, 1.50 s** after the burst began |

- **Re-run independently by PNU 2026-10-29 on their own SITL - every scenario AS EXPECTED.**
- **THE DEFECT WAS REPRODUCED, NOT JUST THE FIX.** With `PMUCAN_TC1_STALE_MS` temporarily
  raised to 600000 to disable guard 1 and nothing else changed, the identical `cut-start`
  scenario **started the engine 1.34 s after the last TC1 arrived**, with no further operator
  input - the bench report of 2026-10-19, reproduced on a desk. Restoring the constant returned
  it to no-start.
- **Interlock duration measured at 1.50-1.55 s** from the first message of the Start burst to
  `ENGONOFF=1` on the wire. The bench measured ~1 to 1.5 s, so this agrees. It is longer than
  the naive 1.0 s the threshold of 10 implies: ~0.1 s goes to the second message making the
  first valid pair, up to 0.1 s to the ENGONOFF slot's phase in the five-slot rotation, and the
  remainder is consistent with an occasional counter reset when a 10 Hz burst is sampled by a
  10 Hz loop. **That last part is a hypothesis, not a measurement.**
- **Build**: CubeOrangePlus, SITL, and SITL with `AP_PNU_CAN_ENABLED=0`.

**Known Limitations**

- **No real PMU, no real engine, no real KGCS.** The injector sends what the KGCS protocol
  specifies, not what KGCS actually emits.
- The LTE link is not simulated, so the message-bunching sensitivity is not exercised -
  `PMU-002`.
- **The numbers are confirmed to behave as designed; they are not confirmed to be the right
  numbers for the aircraft.**

---

## Tools, 2026-10-29 - the fake PMU (no version bump)

**No firmware file changed and no version was bumped.** Every previous CAN Update Plan step
carried a `version.h` bump because it changed the aircraft; this one does not change the binary
at all, so bumping `THISFIRMWARE` would have decoupled the version from what the version
describes.

**Changes**

| File | What it is | Lines |
|---|---|---|
| `pmu_model.py` | the engine, generator and battery - pure state machine, no I/O | 335 |
| `pmu_frames.py` | the five status frames - wire format only, no state | 99 |
| `fake_pmu.py` | SocketCAN I/O, the operator console, MAVLink takeoff detection | 304 |
| `test_pmu_model.py` | 17 tests - the RPM ladder, and every field at the offset `AP_PMUCAN` reads | |
| `test_fake_pmu.py` | 26 tests - RTR dispatch, command decode, the silent trigger, takeoff detection | |

The three-way split is deliberate: the model is testable without CAN, the frames are testable
without a socket, and a second protocol reuses neither. **Raw `AF_CAN` sockets, not
`python-can`**, so there is no dependency to install and the RTR and extended-id handling stays
explicit. `pymavlink` is imported lazily and only for automatic takeoff detection.

**Bug Fixes - five defects found in SITL, all in the takeoff watcher, none in the model or on
CAN. The second was introduced by the fix for the first.**

1. **ILD, the head temperatures and PCL oscillated after takeoff.** `recv_match()` accepts
   `HEARTBEAT` from any source system, and the armed flag was taken from whichever heartbeat
   arrived last. With Mission Planner and KGCS also on the link, `armed` alternated at the
   heartbeat rate, so the engine state flapped GROUND_IDLE <-> FLIGHT. Invisible before
   takeoff, which is why it survived to flight. Fixed by filtering to one system id, plus
   hysteresis on the altitude fallback and treating `LANDED_STATE_UNDEFINED` as no data.
2. **Then takeoff was not detected at all.** The fix above identified the vehicle as the first
   heartbeat that is not `MAV_TYPE_GCS` - a sound filter and an unsound identifier. **The
   vehicle is now identified by its TELEMETRY**: `GLOBAL_POSITION_INT` and `EXTENDED_SYS_STATE`
   are sent by vehicles and by nothing else.
3. **It could not have worked at all as a passive listener.** Two SITL facts neither the fake
   nor the documentation had recorded: `udp:127.0.0.1:14550` is MAVProxy's output, not SITL's;
   and **Copter's default stream rates are ALL ZERO**
   (`GCS_MAVLink_Parameters.cpp:60-70`), so a listener that does not ask receives HEARTBEAT and
   nothing else. The watcher now requests its own streams with `MAV_CMD_SET_MESSAGE_INTERVAL`.
   Both facts were **measured against a live SITL**, not reasoned about.
4. **The watcher died instead of retrying.** SERIAL1 and SERIAL2 (TCP 5762 and 5763) do not
   exist until something connects to SERIAL0, because SERIAL0 is `tcp:0:wait`. Starting the
   fake first got `ECONNREFUSED` and the thread died with a traceback and no other symptom.
   The connection is now retried and the whole loop wrapped.
5. **The engine dropped to ground idle WHILE DESCENDING UNDER RTL.** `MAV_LANDED_STATE_LANDING`
   is reported all the way down, and the airborne test asked whether the state was `IN_AIR` or
   `TAKEOFF` - which looks equivalent to "not on the ground" and is not. **Only an explicit
   `ON_GROUND` now counts as landed.** On a real aircraft that is the only acceptable
   direction: an engine that drops to idle during a descent is far worse than one that holds
   flight power after touchdown.

**Verification**

- **SITL, flown by PNU 2026-10-29 - the whole sequence.** `vcan0`, Mission Planner on SERIAL0
  and KGCS on SERIAL1. Correct: choke valve open and close, the choke crank, Start, the warm-up
  idle, the cooling trip to ground idle, Stop, takeoff, **RTL with landing as the final stage**,
  and **the silent trigger: `silent` while hovering trips the D9 failsafe to RTL, `resume`
  reconnects and clears it**. The failsafe that until now could only be tested by pulling a CAN
  connector is a one-word console command.
- **Unit tests**: 43 offline tests at the time.
- **Confirmed by reading the SITL transport rather than assuming**: `AP_HAL_SITL` hard-codes
  the SocketCAN interface name as `vcan<instance>` (`CAN_SocketCAN.cpp:32`), so the first
  interface is always `vcan0`; `SIM_CAN_TYPE1 = 2` selects SocketCAN, default 1 is multicast
  UDP; and `AP_HAL::CANFrame`'s `FlagEFF`, `FlagRTR` and `FlagERR` are bit-identical to
  SocketCAN's, with `send()` copying `frame.id` through unmasked - **so RTR survives the SITL
  transport in both directions**.

**Known Limitations**

- RPM follows the scripted ladder and is **not** derived from `ENGPCL`, because the ICD gives
  no throttle-to-RPM relation. A manual throttle command is recorded and echoed but moves
  nothing. Deriving a curve would make the fake agree with an invention rather than with the PMU.
- Values marked `MODEL CHOICE` in the source are not protocol facts - GAGE, the BCD DATE, the
  landing transition, the stop ramp.
- No stop compliance, no real motor dynamics, no real PMU firmware behaviour. It does not
  replace the bench; it moves the logic errors off it.

The engine model the fake implements is specified in **Appendix B** below.

---

## V5.1.17 - the PMU failsafe moves out of a logging task

**Changes**

- `failsafe_pmucan_check()` becomes its own `SCHED_TASK` at priority 92, same 10 Hz.

**Bug Fixes**

- **A logging-free build had no PMU failsafe at all, and said nothing about it.**
  `failsafe_pmucan_check()` was called from `ten_hz_logging_loop()`, which sits inside
  `#if HAL_LOGGING_ENABLED`.
- **Found by reading, not by a failure** - and no shippable image was ever exposed, because
  Copter does not currently build with logging off anyway.

**Verification**

- **Build.**

**Known Limitations**

- The failsafe itself was unchanged; the 5.1.15/5.1.16 bench evidence still stands.

---

## V5.1.16 - two bench fixes: the arming bypass and the startup report

**Bug Fixes**

- **`ARMING_SKIPCHK` bit 13 had NEVER worked, and D9(b)'s design rested on it.**
  `check_failed()` only formats and reports - it **never consults `check_enabled()`** - so a
  check must wrap *itself*, the way `AP_Arming.cpp:1134` and `:1303` do. `pmucan_checks()` did
  not, so bit 13 did nothing and **an aircraft with no PMU could not be armed at all** except
  by disabling every check. Wrong since D9(b) landed in 5.0.x; nothing in step 2 caused it.
  Skipping leaves `PMUCAN_Fail` untouched, so `arm()` still latches
  `pmucan_armed_healthy = false` and the in-flight failsafe stays out of the way - exactly the
  "launched without a PMU deliberately" case D9(b) designed.
- **The one-shot startup report was emitted before anyone was listening.** It fired at a flat
  5 s from the CAN thread; KGCS had not connected, the statustext was dropped, and being
  one-shot it never returned - so **neither `PMU ready` nor the protocol-16 deprecation warning
  was ever seen, and 15 and 16 looked identical on the bench.** It now waits for
  `GCS_MAVLINK::active_channel_mask()` to be non-zero, with **no upper bound**. A one-shot
  message nobody receives is the silence D6 exists to remove.

**Verification**

- **Bench, 2026-10-19, both fixes confirmed**: `PNU_CAN: CAN_D1 PMU ready` now appears at
  boot, `PNU_CAN: CAN_D1_PROTOCOL 16 is deprecated, use 15` appears on 16,
  `[PMU] Connection Failure!` on an absent PMU, and **`ARMING_SKIPCHK` bit 13 now bypasses the
  check as D9(b) always intended**.
- **Bench, 2026-10-19 (validated in 5.1.15, unchanged here)**: PMU telemetry downlink, KGCS and
  FC command paths, the prearm block with the PMU absent, and the **in-flight failsafe** -
  pulling the CAN connector while armed took the `set_mode_RTL_or_land_with_pause()` branch
  immediately and recovered cleanly on reconnect.
- **Bench, 2026-10-19**: the ~1.0 s engine interlock survived the 10 Hz rework, measured ~1 to
  1.5 s against an expected ~1.2 s (0.2 s to establish the first alternating pair, then 10
  counts at 10 Hz). That closed the main risk of making the rate explicit.

**Known Limitations**

- **STILL UNTESTED: the shared bus.** Every bench run had `CAN_Dn_PNU_OPTS = 1`, so the
  id-based health rewrite - the thing step 2 exists for - had never been exercised on hardware.
  It still has not; the SITL shared-bus runs from 2026-10-30 are not hardware.
- Stop-command timing was outstanding from this bench session and is still not bench-measured.

---

## V5.1.15 - CAN Update Plan step 2: the PMU becomes an `AP_PNU_CAN` backend

**Changes**

- **Protocol 15 now instantiates `AP_PNU_CAN`**, with `CAN_Dn_PNU_OPTS` bit 0 creating
  `AP_PMUCAN` as one of its backends. **The enum value is unchanged and `PNU_OPTS` defaults to
  1**, so an upgrade is a **no-op** for every deployed aircraft: PMU only, exactly as before.
- **16 survives as `PNU_CAN_LEGACY`**, a deprecated alias that warns once per boot, so a bench
  aircraft configured during step 1 keeps working instead of hitting the factory's `default:`
  and silently getting no driver, no parameters and no message. 16 is removed from the
  `PROTOCOL` `@Values` list at the same time: the dropdown is for what an operator should
  *choose*, the enum and factory case for what already exists in a stored configuration.
  **Delete the enumerator, the factory case, the arming case and the warning when the 5.1.x DEV
  series goes OFFICIAL.**
- **The 10 Hz is now EXPLICIT.** It used to be an accident: `TXspin()` ran at 50 Hz and rotated
  five commands, so each *happened* to go out at 10 Hz and nothing said so. D7's interlock is
  calibrated on it - threshold 10 ticks ~ 1.0 s only because `engineonoffstate()` is reached
  once per rotation. Now derived from `PMUCAN_MSG_RATE_HZ` and the driver tick, reproducing the
  old cadence exactly (slot *n* at `tick % 10 == n*2`) while saying why.
- **Health is now keyed on PMU STATUS FRAMES.** `RXspin()` treated *any* successful receive as
  proof of life - harmless on a dedicated bus, but on the shared bus **gimbal traffic would
  have held `PMUCAN_Fail` at CONNECTED with the PMU dead, and `failsafe_pmucan_check()` would
  never have fired.** Only the five PMU status ids feed the leaky bucket now; its thresholds
  and timing are unchanged. **This fix is not optional and is the reason step 2 could not be a
  mechanical move.**
- **The directory layout is untouched, deliberately.** `AP_PMUCAN/AP_PMUCAN.h` keeps its path
  and still declares `PMU_Status` / `PMU_Ctrl_Echo` beside their definitions (D14), so
  `events.cpp`, `AP_Arming_Copter.cpp` and `GCS_PNU.cpp` need no change - **verified by diff**.
  Both globals and their definitions sit *outside* the `AP_PNU_CAN_ENABLED` guard, because
  those files reference them unconditionally.
- **Deviation from the plan, deliberate: protocol 17 is NOT dropped here.** Removing it means
  `AP_PNUSAR` stops being an `AP_CANDriver`, which is step 3's job. Bundling it into the risky
  PMU conversion would have mixed two unrelated changes.
- Dead members the conversion created were removed rather than left (D6): `_can_iface`,
  `sem_handle`, `_thread_name`, `_initialized`, `_driver_index`, `pmucan_period_us`,
  `_AP_PMUCAN_loop_cnt`, `_cmd_idx`, `_rtr_idx`, `RTR_MSG`, `_pmucan_last_send_us`,
  `PMUCAN_SEND_TIMEOUT_US`.

**Verification**

- **Build**: CubeOrangePlus **1,733,336 B**. All five guards compile out, including
  `AP_PNU_CAN_ENABLED=0` at 1,715,412 B. `CAN_Dn_PROTOCOL` 15 now reads `PNU_CAN`.
- **Unit tests**: 10 CubeMars codec tests pass.
- **Bench, 2026-10-19 (reported under 5.1.16, same code path)**: telemetry, command paths,
  prearm, in-flight failsafe and recovery all PASSED.

**Known Limitations**

- **NOT BENCH-TESTED at the time of release.**
- The shared bus (`PNU_OPTS=3`) and Stop-command timing remained outstanding.

---

## V5.1.14 - CAN Update Plan step 1d: HOMING, the Appendix 1 endpoint search

**Changes**

- **`CAN_Dn_PNU_HOME` DEFAULTS TO 0, deliberately.** This routine drives a bulky payload into
  three sets of hard stops with the mechanical clamp *disabled*, and the tuning is provisional -
  only the 1 s stall dwell is settled. **Nothing drives into a stop until it is switched on.**
  With it off the driver says so once per boot and the zero is set by hand.
- **MIT mode is mandatory and enforced.** `init()` refuses to arm the sequence unless
  `PNU_MODE=0`, naming it in the message. MIT is the only mode where the host bounds the push
  into a stop: a sweep sends `kp=0, kd=PNU_HKD, v=+/-PNU_HVEL`, so the stall torque settles at
  **HKD x HVEL** - a number the operator picks. Position/velocity mode runs a cascaded loop
  whose torque the CAN frame cannot limit at all.
- **Endpoint detection is by ANGLE ONLY** (Appendix 1 item 4) - overcurrent, undervoltage and
  overload are *emergency stops, not endpoint criteria*. An endpoint is "the angle stopped
  changing while motion was still commanded"; any fault status aborts.
- Sequence per axis, **roll then pitch then yaw**: seek a stop, back off 2% of travel, seek
  again and require both readings within `PNU_HRTOL`; repeat the other way; validate
  stop-to-stop travel against twice the mechanical limit within `PNU_HTOL`; move to
  `Theta_mid = (Theta_CW + Theta_CCW) / 2`; confirm motion stopped; zero; verify the feedback
  reads zero. **The idle axes are
  HELD at their power-up angle throughout.**
- **Travel validation is the only runtime check that can catch a `PNU_PMAX` mismatch** - the
  failure that does not fail but silently rescales everything. A factor-of-two travel reading
  is its signature and nothing else in the system would notice.
- **Travel validation is ASYMMETRIC** (settled 2026-10-19). Lower bound is twice
  (mechanical - `PNU_MARG`) = **50 deg on a 60 deg axis**: the mount must reach its own
  operating range, and below that it aborts as obstructed. A measurement between the operating
  and mechanical ranges is **accepted, and `MNT1_*` stays nominal** - limits are *not* derived
  from the measurement, because stop compliance under a heavy payload would make the mount's
  reach vary with how hard it happened to press. Upper bound stays mechanical + `PNU_HTOL`.
- **Per-axis rotation direction: `PNU_DRLL` / `DPIT` / `DYAW`** (0 normal, 1 reversed). The
  mount is still undergoing mechanical design changes, so motor direction is configuration, not
  a source constant. Everything inside `AP_PNUGimbal_Backend` is stored in the **motor frame**
  and the conversion happens in exactly **two places** - `set_target()` in, `get_feedback()`
  out. **Homing is unaffected**: it finds both stops and takes the midpoint, which is
  direction-agnostic.
- **12 new parameters**: `HOME`, `HVEL`, `HKD`, `HKP`, `HSTL`, `HDLT`, `HTOL`, `HRTOL`, `MARG`,
  `DRLL`, `DPIT`, `DYAW`. Phase timeouts are **derived** from travel and velocity rather than
  parameterised, so they cannot be set inconsistently with the speed.

**Bug Fixes - two hazards found while writing it**

- VERIFY originally commanded position 0 *before* confirming the zero had been sent - that
  means the OLD origin, possibly tens of degrees away, and would have lurched the payload
  across its travel.
- HOMING was reachable from a *second* INIT, so clearing a fault **in flight** would have
  re-run the endpoint search mid-air. Homing is therefore **once per POWER CYCLE**, not once
  per INIT - the zero lives in the drive and survives a disable/enable - and is **refused while
  armed** (`hal.util->get_soft_armed()`), because `ARMING_SKIPCHK` bit 16 lets an operator arm
  an unhealthy mount.

**Verification**

- **Build**: CubeOrangePlus **1,734,332 B**. All five guards compile out. 10 codec tests pass.
  29 `CAN_Dn_PNU_*` parameters, none over 16 characters.
- **SITL, 2026-10-30 (after 5.1.23's fix)**: the sequence ran end to end for the first time
  against the fake gimbal.

**Known Limitations**

- **NOT bench-run.** `PNU_HOME` is 0 until the tuning in `GIMBAL-002` is measured.
- The stall threshold, approach velocity and the two gains are provisional and unmeasured.

---

## V5.1.13 - CAN Update Plan step 1c: the gimbal lifecycle state machine

**Changes**

- **INIT -> ALIGN -> RUN, plus MAINTENANCE and FAULT.** `HOMING` is in the enum and
  deliberately not implemented at this version - that is step 1d, and until the Appendix 1
  endpoint search runs, **the zero must be set by hand on the bench**.
- **It lives one layer BELOW the mount**, in `AP_PNUGimbal_Backend`: it runs in the CAN thread
  where feedback arrives, so a fault transition happens *on the frame* rather than a 50 Hz
  mount tick later; it survives `HAL_MOUNT_ENABLED=0` (D13); and a second motor protocol would
  need the same lifecycle unchanged.
- That meant **moving the per-axis target and feedback storage, the mechanical limits and the
  stale-feedback policy up out of `AP_PNUGimbal_CubeMars`**, which now owns the wire format and
  nothing else - every remaining member is a CAN id, a bit layout or a DLC.
- **The lifecycle is NOT the mount mode.** `MAV_MOUNT_MODE_*` says *where to point*; the state
  says *whether the hardware can be pointed*. RETRACT is a targeting mode, not "motors off".
- **ALIGN exists because section 9 requires position control to start from the measured angle.**
  It seeds the command from feedback so the first mount target is approached from where the
  mount *is*, instead of arriving as a step from zero - with a heavy SAR payload that is the one
  thing here that could break hardware.
- **New `CAN_Dn_PNU_SLEW`** (deg/s, default 45, provisional) bounds the approach *and* every
  later move. One mechanism, two jobs, cheaper than ramping stiffness.
- **New `CAN_Dn_PNU_MAINT`** takes the FC completely off the bus so a PC can configure a motor
  over UART. Entering is immediate; **leaving requires a reboot**, and the state is latched at
  `init()` for exactly that reason - the utility can change the CAN id, the mode and the
  encoding ranges, and a mismatch *silently rescales everything*.
- **Arming interlock: `healthy()` is now true only in RUN with every axis fresh.**
  `AP_Mount::pre_arm_checks()` fails on `!healthy()` as `Check::CAMERA`, so an unhomed or
  faulted mount **cannot be armed**. Override is `ARMING_SKIPCHK` **bit 16** (Camera, decimal
  65536) - the same escape hatch D9(b) established for the PMU.

**Bug Fixes**

- **`AP_PNUGIMBAL_ENABLED=0` did not compile**, and it predated this step, because `AP_PNU_CAN`
  sizes its parameter arrays from `AP_PNUGIMBAL_NUM_AXES`. The axis identity moved out of the
  enable guard into `AP_PNUGimbal_config.h` - an aircraft with a PMU and a SAR but no gimbal is
  a real configuration, and AGENTS.md requires the base system to compile with optional features
  disabled.

**Verification**

- **Build**: CubeOrangePlus **1,730,780 B**. **All five guards compile out** -
  `AP_PNU_CAN_ENABLED=0` 1,717,984 / `AP_PNUGIMBAL_ENABLED=0` 1,719,824 /
  `AP_PNUGIMBAL_CUBEMARS_ENABLED=0` 1,721,392 / `HAL_MOUNT_PNUGIMBAL_ENABLED=0` 1,724,660 /
  `HAL_MOUNT_ENABLED=0` 1,656,240.
- **Unit tests**: the 10 codec tests still pass.

**Known Limitations**

- Bench work needs the SAR mass fitted and the zero set by hand. Not done.
- **The state machine never actually ran until 5.1.23 fixed INIT.** See that entry.

---

## V5.1.12 - CAN Update Plan step 1b: `AP_Mount_PNUGimbal`, the mount backend

**Changes**

- **`MNT1_TYPE = 15`** selects a new `AP_Mount` backend for the PNU 3-axis gimbal. `AP_Mount`
  already does FC-attitude stabilisation and **needs no camera**, so the backend inherits mount
  modes, RC targeting, the MAVLink gimbal manager, ROI, the `MNT1_*_MIN/MAX` limits and the
  lead filter, and adds only the stabilisation maths.
- **IT COMMANDS NO MOTORS.** Targets are written all the way through to
  `AP_PNUGimbal_CubeMars`, but that backend transmits nothing until `set_enable(true)` and
  nothing in the tree calls that yet - the lifecycle state machine is step 1c. The whole path
  is exercised with the motors silent.
- **Limits split across two layers, deliberately.** `MNT1_ROLL/PITCH/YAW_MIN/MAX` carry the
  **operating** limits (+/-25 / +/-25 / +/-15) and `AP_Mount` clamps to them; the new
  **`CAN_Dn_PNU_MRLL/MPIT/MYAW`** carry the **mechanical** limits (+/-30 / +/-30 / +/-20) and
  the gimbal backend clamps again as the last thing before a frame is built. A bug in the
  targeting layer cannot drive the payload into the landing skid struts.
- **The mechanical clamp means nothing until homing runs.** The encoder is relative and its
  zero is assumed not to persist, so until step 1d +/-30 in encoder terms is not +/-30
  mechanically. `init()` also rejects a mechanical limit larger than `PNU_PMAX`, which would be
  unreachable and leave the stops as the only protection.

**Verification**

- **Bench-confirmed 2026-10-16** from a dataflash log: `MNT0.Roll` and `MNT0.Pitch` move
  opposite vehicle roll and pitch, which is the AHRS correction and the whole of what step 1b
  set out to prove. **Provable with no new logging code** - `AP_Mount_Backend::write_log()`
  already writes `MNT`, whose *desired* columns are the earth-frame target and whose *actual*
  columns this backend fills with the body-frame demand.
- **Yaw looked wrong and is not** - see `GIMBAL-003` in `ISSUES.md`: `MNT0.YawB` did not move
  and `MNT0.YawE` read the same as `ANG.Yaw`, because yaw stabilisation goes through
  `MountAngleTarget::get_bf_yaw()`, which subtracts heading **only when `yaw_is_ef` is set**.
  That is the earth-frame yaw lock not being switched on, not a defect.
- **Build**: CubeOrangePlus **1,729,440 B**; `HAL_MOUNT_PNUGIMBAL_ENABLED=0` builds at
  1,728,304 B so the backend costs **1,136 B**, and `HAL_MOUNT_ENABLED=0` still builds at
  1,659,580 B (PNU-ISSUE D13 intact). Registered in `build_options.py` as `Gimbal / PNUGIMBAL`.
- **Unit tests**: the 10 codec tests still pass.

**Known Limitations**

- **Found while implementing: ArduPlane has been unbuildable on this branch since `ed055d2f24`
  (5.0.10)** - `GCS_PNU.cpp` references `PMU_Ctrl_Echo` and `AP::ac_avoid()`, both Copter-only.
  A vehicle gate on the new backend was written and then **removed as unnecessary**: it would
  have protected a build configuration this branch has never supported.

---

## V5.1.11 - CAN Update Plan step 1: `AP_PNU_CAN`, the multi-backend CAN driver

**Changes**

- **Three protocols, one bus, one driver.** `AP_CANManager` binds each interface to exactly
  **one** driver - one interface to many drivers is not supported, and a driver with no
  interface is never allocated at all, so it fails *silently*. `CAN_Dn_PROTOCOL2` does not
  help: it is a DroneCAN-hosted auxiliary slot taking one `CANSensor` with no thread of its own.
- `AP_PNU_CAN` owns the interface, the thread, the parameters and the one place a frame reaches
  the bus; `AP_PNU_CAN_Backend` owns a wire format and nothing else. `AP_PNUGimbal_Backend` sits
  between them as the gimbal-shaped view, so a control layer can drive any motor protocol
  without naming CubeMars.
- **`handle_frame()` now returns `bool`** and the driver stops at the first backend to claim a
  frame - the `MultiCAN` pattern upstream already uses for the CAN rangefinder. **An unclaimed
  frame is counted and reported**, which is how a mis-assigned CAN id gets noticed instead of
  vanishing.
- **`send_frame()` takes the FULL 32-bit id** including `FlagEFF` / `FlagRTR`, because the PMU
  polls with extended RTR frames and the gimbal sends standard data frames.
- **`update()` is handed the driver's tick count**, so every backend derives its rate from one
  clock. The PMU's 10 Hz turned out to be an accident of an undocumented command rotation; that
  must not happen twice.
- **The PMU is untouched on protocol 15 and nothing that flies changes in this version.**
  `AP_PNU_CAN` is on protocol 16 (`PNUGimbal` renamed `PNU_CAN`) so the gimbal stays benchable
  while the PMU conversion waits for hardware.
- **The name carries an underscore on purpose: `PNU_CAN`, never `PNUCAN`.** The two drivers sit
  beside each other everywhere an operator looks - `15:PMUCAN` and `16:PNU_CAN` in the same
  `CAN_Dn_PROTOCOL` dropdown, `PMUCAN:` and `PNU_CAN:` statustexts on the same GCS screen - and
  **M and N are one letter apart in a sans-serif font**. Do not "tidy" it away.
- **New `CAN_Dn_PNU_*` parameters replace 5.1.10's `CAN_Dn_PG_*`**, including
  **`CAN_Dn_PNU_OPTS`**, the device bitmask - bit 0 PMU, bit 1 gimbal, bit 2 SAR, **default 1**.
- **One flat group, not the sibling subgroups the plan assumed.** `AP_SUBGROUPPTR` resolves
  `var_info` from the *class* (`AP_Param.h:165`), so two subgroups pointing at this one object
  would get the same table, and nesting eats so much of the 16-character budget that the gimbal
  parameters become unreadable. All 12 names fit under `CAN_Dn_PNU_`.
- **Parameter index 7 - the old `PG_` group - is retired and NOT reused.** AP_Param stores by
  index rather than by name, so a bench aircraft that saved a `PG_` value would have it read
  back as a different parameter.
- **`CAN_Dn_PNU_TYPE` drops `0:None` and defaults to `1:CubeMars`.** Bit 1 of `PNU_OPTS` now
  answers whether a gimbal is fitted. Two switches for one thing is how a mount ends up silently
  disabled by the one nobody looked at.
- **A one-per-boot line names the running backends** - `PNU_CAN: CAN_D1 Gimbal ready, no PMU`.
  That is what makes a forgotten `PNU_OPTS` bit visible: with bit 0 clear,
  `PMU_Ctrl_Echo.PMUCAN_Fail` stays 0 and reads as **healthy**, so a missing PMU is otherwise
  completely silent.

**Verification**

- **Build**: SITL copter clean, CubeOrangePlus **1,727,948 B**; with
  `--define AP_PNU_CAN_ENABLED=0` it still builds at 1,722,636 B, so the driver costs
  **5,312 B**.
- **Unit tests**: the 10 CubeMars codec tests still pass unchanged.

**Known Limitations**

- Nothing executed. No hardware, no SITL at this version.

---

## V5.1.10 - `AP_PNUGimbal` gets a protocol backend: `AP_PNUGimbal_CubeMars`

**Changes**

- **The structure deviates from `AP_PMUCAN` deliberately.** The 5.1.9 skeleton was one class
  that would have held one protocol. It is now a **frontend plus an `AP_PNUGimbal_Backend`**,
  because the mount is expected to accommodate more than one motor type and only the protocol
  differs between them. A backend owns the wire format and **never touches `_can_iface`**,
  which keeps one place that validates an outgoing frame and one place that counts a failed
  send.
- **`AP_PNUGimbal_CubeMars`** implements the CubeMars GLII family protocol from
  `libraries/PNUGimbal_CAN_Protocol_Type_CubeMars.md`: all three control modes (MIT,
  position/velocity, velocity), the four universal commands (enable / disable / zero / clear
  errors), and the common feedback frame.
- **PROTOCOL LAYER ONLY.** No stabilisation loop, and not an `AP_Mount` backend at this version.
  `set_target()` exists and nothing in the tree calls it, so the driver transmits nothing until
  a control layer does.
- **New parameters, `CAN_Dn_PG_*`** - a subgroup on `CANDriver_Params` at index 7, the same
  mechanism PiccoloCAN uses for `PC_`. `CAN_Dn_PG_TYPE` defaults to **0 (None)**, so
  `CAN_Dn_PROTOCOL = 16` on its own still does nothing: selecting a CAN protocol must not start
  commanding motors.
- The five encoding limits (`PMAX`/`VMAX`/`KPMAX`/`KDMAX`/`TMAX`) are **parameters, not
  constants**, because they MUST match what the vendor PC utility wrote to the motors. A
  host/device mismatch **does not fail** - it silently rescales every command and every reading,
  which is the worst failure mode this protocol has.
- **Feedback carries only four bits of motor identity** (the upper nibble of D0 is the status
  code), so motor IDs are restricted to **1..15 and required distinct** at init, and all three
  motors share one Master ID. The manual's prose says you can read all eight bits of D0 as an
  ID; its own table says you cannot.
- **Nothing is commanded until a target is set, and nothing while disabled.** Section 9 requires
  position control to start from the measured current angle, so a zero-initialised target must
  never reach the wire. `set_enable(false)` also invalidates the targets, because the mount will
  have moved while disabled.

**Verification**

- **Unit tests**: the codec is verified against the manual's own published command vectors in
  `libraries/AP_PNUGimbal/tests/` - four MIT, two position/velocity, two velocity, plus
  saturation, round-trip and feedback decode. **10 tests, all passing.** Two of the manual's MIT
  vectors are **not** reproduced and cannot be - its +/-6 rad/s rows do not decode to +/-6 at
  the +/-200 range it states. That divergence has its own test, so a bench disagreement points
  at the manual rather than at this code.
- **Build**: SITL copter clean, CubeOrangePlus **1,727,364 B**. With
  `--define AP_PNUGIMBAL_ENABLED=0` it still builds at 1,722,636 B - so the whole driver,
  skeleton included, costs **4,724 B** and the guard is intact.

**Known Limitations**

- **What the manual does not specify was not invented**: no command period, no feedback rate, no
  loss timeout, no watchdog interval, no acknowledgment, no runtime mode switch. All seven are
  listed at the top of the backend so a later session does not fill them in with a plausible
  default. See `GIMBAL-001` and `GIMBAL-004`.
- `AP_PNUGIMBAL_LOOP_HZ` is **still 100 and still unchecked** - the manual specifies no command
  period at all, so it needs measuring.

---

## V5.1.9 - skeletons for two new CAN drivers: `AP_PNUGimbal` and `AP_PNUSAR`

**Changes**

- Both speak **private CAN protocols, NOT DroneCAN**, in the same shape as `AP_PMUCAN` - their
  own thread created in `init()`, nothing on the vehicle scheduler.
- Registered as `AP_CAN::Protocol` **16** (`PNUGimbal`) and **17** (`PNUSAR`). Slots in
  `AP_CANManager`, factory cases behind `#if`, and both listed in `ArduCopter/wscript`.
- **STRUCTURE ONLY**: `rx_spin()`, `tx_spin()`, `handle_frame()` and `send_frame()` were stubs
  marked `TODO`, with no parameters, no `CAN_Dn_PROTOCOL` `@Values` entry and no vehicle-side
  consumers.
- The skeletons carry this project's own lessons rather than copying `AP_PMUCAN` verbatim:
  **DLC validation before reading any field** with a rate-limited short-frame report (D6),
  **initialisers on every member** (D1/D10), a note **not to add write-only counters** (D6), and
  a reminder to **confirm endianness and say so at the site** if no byte swap is needed (D15).
- **Design intent recorded in both headers** (2026-10-15): `AP_PNUGimbal` drives three CubeMars
  motors and carries **no camera**; it stabilises the SAR device that `AP_PNUSAR` will drive.

**Bug Fixes**

- **Adding protocol enum values breaks exhaustive switches** - `-Werror=switch` caught
  `AP_Arming.cpp`, which now lists both beside PMUCAN's existing no-op case.

**Verification**

- **Build.**

**Known Limitations**

- Values 16/17 could collide with future upstream protocol numbers, exactly as `PMUCAN = 15`
  might; watch this on the next ArduPilot bump. 16 was later reused as the deprecated
  `PNU_CAN_LEGACY`; 17 was retired in 5.1.19 and must not be reused.

---

## V5.1.8 - backward-compatibility fix for KGCS; D10 is FC-complete

**Changes**

- **Tracking, video source and IR palette no longer report as failures on a Gremsy.** They had
  already left the FC for GCS-direct routing, but still produced `CAM: mount rejected ...`
  **every 10 s, forever**, because the GCS keeps sending TC2 and nothing on that payload can
  serve them. That mattered beyond the wording: `report_cam_unsupported()` shares one rate
  limiter, so three permanent warnings would have **crowded out genuine faults** - destroying
  the signal D10 Stage 1 exists to provide.
- All four GCS-direct functions now announce themselves **once** at `MAV_SEVERITY_INFO` and then
  stay silent: `CAM: <function> is GCS-direct on this mount`.
- **A Viewpro that genuinely declines still warns** - the gate is on the payload, not on the
  return value, so a real fault on a payload that should work is never silenced.
- Renamed `KGCS-direct` to **`GCS-direct`** in the four notices and in the code (`GCS_DIRECT_*`,
  `report_gcs_direct()`) - the ground station driving them is PNU-GCS. "KGCS interface" now
  means the TC2/TM2 ICD; "KGCS" means the product.

**Verification**

- **Verified by reading the tree, not assumed: KGCS-to-VIO routing already works.**
  `MAVLink_routing::get_targets()` reads the **generated** `mavlink_get_msg_entry()` table, so
  any dialect message carrying `target_system`/`target_component` routes correctly with nothing
  hard-coded. `PARAM_EXT_SET` (323) and `COMMAND_LONG` for `USER_4` are in the staged dialect
  and do carry those fields. A message addressed to the camera's component fails
  `process_locally` - ArduPilot's own compid is 1 - and falls through to `forward()`.
  Preconditions: the VIO heartbeating (it does), the links differing (they do), and
  `MAVn_OPTIONS` bit 1 (`NO_FORWARD`) being clear.
- All seven PNU-KAL messages checked and unaffected - none carries a target field, so they are
  broadcast and FC-handled.
- **Build**: `./waf copter` clean, zero warnings, **1,722,228 B**. Also builds with
  `HAL_MOUNT_VIEWPRO_ENABLED=0` (1,717,352 B) and `HAL_MOUNT_ENABLED=0` (1,653,516 B).

**Known Limitations**

- **Not bench-tested.**
- Two consequences, both meaning *no* code or parameter change: the msg 285 suppression is
  correctly keyed per backend (KGCS only ever flies a Viewpro, which suppresses it), and
  **`NO_FORWARD` must NOT be set on the VIO link** - `PARAM_EXT_ACK` carries no target fields,
  so acks would be blocked and every palette/source change would look like a failure.

---

## V5.1.7 - Stage 3 bench-confirmed; the FC-side work on D10 is COMPLETE

**Verification**

- **Bench, 2026-10-13: `zoomLevel` follows the active EO/IR source.** Switching the main source
  switches the reported value, so the camera does the selection itself and no sensor check was
  needed in the FC. This was the last thing that could have made Stage 3 wrong:
  `AP_Mount_Viewpro` is sensor-aware internally, so had the VIO reported EO zoom while the
  operator watched IR, TM2 would have carried a number that was not what it claimed to be.
- Both ranges fit TM2's `int8_t` field: EO reaches 30x in super-resolution (matching the ICD's
  `MIN:1, MAX:30`), IR reaches 8x. Only combine mode at 240x would overflow, and it is not used.

**Changes**

- **The bench aid is kept but disabled**: `AP_CAMERA_MAVLINKCAMV2_ZOOM_DEBUG` defaults to **0**;
  set it to 1 to report zoom, focus and the reading's **age** to the GCS every 2 s. The age field
  is the point - it separates a live value from a stale one, which a raw number cannot. The decode
  is permanent, so only the reporting is conditional and the off build is byte-identical to having
  no debug code at all. Same pattern as `GCS_VIEWPRO_IR_DEBUG` in `GCS_PNU.cpp`.

**Known Limitations**

- A more expensive route using the payload app on component 26 was considered and is **not
  needed** - recorded in `ISSUES.md` under D11 because the idea is sound and will come up again.

---

## V5.1.6 - D10 Stage 3: TM2 reports real zoom on a Gremsy

**Changes**

- **TM2's `Zoom_POS_FB` now carries real zoom on a Gremsy.** Bench measurement showed the VIO
  reports `CAMERA_SETTINGS.zoomLevel` in **zoom times**, not the percentage the MAVLink spec
  defines - 1.00 at wide, 30.00 at full tele in SR mode, matching TM2's ICD range of 1..30
  exactly. That removed both obstacles at once: no percentage conversion, and no `int8_t`
  overflow.
- **Implemented as three layers, so no shared library learns a Gremsy-specific fact:**

  | Layer | Does | Vendor-specific? |
  |---|---|---|
  | `GCS_Common.cpp` | routes `CAMERA_SETTINGS` - **one `case` label**, the only core-file cost | no |
  | `AP_Camera_Backend.h` | `virtual bool get_reported_zoom_level(float&)`, default false | no |
  | `AP_Camera.{h,cpp}` | frontend forwarder | no |
  | `AP_Camera_MAVLinkCamV2` | decodes msg 260, polls it, serves the **raw** value | no - never claims to know the units |
  | `AP_Q30::get_tm2_zoom_times()` | mount first (Viewpro), then camera - **interprets as times** | yes, and this is where the strategy says it belongs |
  | `GCS_PNU.cpp` | TM2 asks `AP_Q30` instead of the mount directly | - |

- **Polled, not streamed.** The VIO answers a request rather than broadcasting, so the backend
  requests at 1 Hz with a 3 s staleness timeout. If the camera stops answering,
  `get_reported_zoom_level()` returns false and TM2 falls back to 0 instead of holding a frozen
  reading.
- **Only `MAV_CMD_REQUEST_MESSAGE` is sent.** The VIO was observed answering both that and the
  superseded `MAV_CMD_REQUEST_CAMERA_SETTINGS` (522), so the legacy form was dropped.
- **`NaN` is rejected explicitly.** `zoomLevel` is a MAVLink2 extension field and is
  legitimately absent on a camera that does not know its zoom.
- **Bonus fix, vendor-neutral:** `send_camera_settings()` now forwards what the camera actually
  reported. Upstream sends `NaN` placeholders, so a standard GCS saw "zoom unknown" even while
  the camera was reporting it.

**Bug Fixes**

- **Found while implementing: `CAMERA_SETTINGS` was never routed to `AP_Camera` at all**, so any
  backend decode would have been dead code. `GCS_Common.cpp` routed only `DIGICAM_CONTROL`,
  `GOPRO_HEARTBEAT` and `CAMERA_INFORMATION`. Fixed with one `case` label. This also means
  ArduPilot's `request_camera_information()` may never have worked on this payload:
  `CAMERA_INFORMATION` arrives because the VIO **broadcasts** it.

**Verification**

- **Bench, 2026-10-13** for the zoom units measurement.
- **Build**: `./waf copter` clean, zero warnings, **1,722,152 B** with the debug off,
  1,722,328 B with it on. Also builds with `HAL_MOUNT_ENABLED=0` (1,653,516 B),
  `AP_CAMERA_ENABLED=0` (1,695,896 B) and `AP_CAMERA_MAVLINKCAMV2_ENABLED=0` (1,720,704 B).

**Known Limitations**

- **TM2 itself not bench-tested at this version** - on hardware `Zoom_POS_FB` should track 1-30
  on EO and 1-8 on IR, and the one-shot `CAM: zoom not reported` warning should stop appearing.
- **Accepted risk, recorded at the code site:** a spec-compliant camera reporting a true
  percentage would be misreported as times. TM2 is a PNU message on PNU aircraft with known
  payloads. Revisit if a third camera type is ever fitted.
- **Known, pre-existing, not changed:** `GCS_PNU.cpp:102` casts `(int8_t)zoom_times`, which
  **truncates** - a camera reporting 29.97 at full tele sends 29. The Viewpro path has always
  done this. Rounding instead would change what goes on the wire, so it is an ICD decision.
- **Worth submitting upstream.** ArduPilot currently cannot read camera zoom at all - the
  routing gap, the decode, the accessor and the `send_camera_settings()` placeholder are one
  coherent gap, none of it PNU-specific.

---

## V5.1.5 - D10 Stage 2b closed by NOT implementing it in the FC

**Changes**

- `AP_Q30::fc_handles_absolute_zoom()` (later renamed `mount_is_viewpro()`) returns true only
  when the mount is `Type::Viewpro`, the one backend whose `set_zoom()` reads `ZoomType::PCT` as
  zoom **times**. The `Tracking_CMD` 21-24 EO branch sends its absolute zoom only in that case.
  Anywhere else the FC sends **nothing** and notes once per boot:
  `CAM: absolute zoom is GCS-direct on this mount`.
- Reason: `ZOOM_TYPE_RANGE` is a percentage of the camera's *full* range, and that range is
  **mode-dependent**, selected by `C_V_ZM_MODE` - an extended parameter **the FC cannot read**,
  since ArduPilot implements no `PARAM_EXT_*`:

  | `C_V_ZM_MODE` | Maximum |
  |---|---|
  | 0 combine | **240x** (`C_V_ZM_CB_LV` idx 0-21) |
  | 2 super-resolution | **30x** (`C_V_ZM_SR_LV` idx 0-11) |

  So converting `EO_zoom_times` to a percentage needs a divisor of 240 or 30 depending on a value
  the FC cannot see; at 100% in combine mode the camera would go to 240x, not the 30x the operator
  asked for. (Incidentally `_Max_zoom_EO = 30` matches the SR-mode maximum exactly, which suggests
  that constant never described the Viewpro - see `CAM-002`.) KGCS drives absolute zoom directly
  with `MAV_CMD_USER_4` "zoom to level", in times.
- **A rate-pulse fallback was rejected**: it would fight the command KGCS is sending. Two sources
  commanding one camera is worse than one source doing less.
- **`VIO_MAVLink_Messages.md`** was added to the repository root - the message reference
  extracted from Gremsy PayloadSDK 3.2.0 source.

**Verification**

- **Build.** Not bench-tested at this version.

---

## V5.1.4 - first version validated against real Gremsy VIO hardware

**Verification**

- **Bench, 2026-10-13.** A Gremsy VIO was connected and driven from Mission Planner.

  | Device | vendor / model | Firmware |
  |---|---|---|
  | Gimbal (`MNT1_TYPE=6`) | `GREMSY` / `VIO` | 7.8.7.0 |
  | Camera (`CAM1_TYPE=6`) | `Gremsy` / `Vio Payload G1` | 18.4.1.3 |

  The two firmware versions differ, confirming the gimbal and camera are genuinely separate
  devices rather than one unit answering on two component IDs.
- **D10 Stage 2a validated on hardware with zero code changes**: with the VIO's camera component
  set to 100 and `CAM1_TYPE=6`, ArduPilot finds the camera and zoom works through Mission
  Planner. `CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM` is **effectively proven** -
  `AP_Camera_MAVLinkCamV2::set_zoom()` returns false *before transmitting* when that flag is
  clear, so zoom reaching the camera means the flag is set.
- **Both remaining ACTION ON PNU questions are now ANSWERED** - the identity strings above, and
  the capability flag.
- **Gimbal control from Mission Planner's slew interface - all paths verified**: rate slew on
  all three axes (a clean passthrough, so 1 deg/s really is 1 deg/s - **not a defect**); Set
  Neutral (**converted by the base class, not native** - the gimbal never sees a "neutral"
  command, only an angle); point down to pitch -90 deg, confirming physical convention and that
  `MNT1_PITCH_MIN = -90` permits full nadir.
- **The dual-GCS compatibility strategy is now hardware-validated**: slew, Set Neutral,
  point-down and zoom all work from Mission Planner against a Gremsy while KGCS convention stays
  confined to `AP_Q30`. Until this session that was a reasoning claim only.

**Bug Fixes**

- **Upstream defect found on the bench**: the `"Camera: ..."` statustext was printed on every
  `CAMERA_INFORMATION`, and the VIO broadcasts that message unsolicited, so it spammed one
  statustext per broadcast to every link. `_got_camera_info` was set *after* the print and never
  read. Now printed once. **Worth submitting to ArduPilot** rather than carried as a divergence.

**Known Limitations**

- **Gremsy firmware defect recorded**: the yaw fields of `GIMBAL_DEVICE_INFORMATION` are sent in
  degrees where msg 283 specifies radians, giving `MNT1_YAW_MAX` a default of
  18334 = `degrees(320)`. Roll and pitch are correct. See `GCS-003` in `ISSUES.md`.
- **Aircraft settings required** - see the release note in `ArduCopter/version.h`: pitch limits
  in physical convention, yaw limits set explicitly on a Gremsy, `CAM1_TYPE` 4 for Viewpro / 6
  for Gremsy, and a Gremsy VIO's camera component set to 100 rather than its default 101.
- **Point-down does not test the 5.1.2 pitch relocation.** On a Gremsy
  `pitch_target_is_reversed()` returned false from the base class even *before* the relocation,
  so Mission Planner was always correct here. **The relocation's real test is Viewpro + Mission
  Planner**, which is where pitch was inverted and should now be right. Still not done.

---

## V5.1.3 - D10 Stage 2a: camera calls routed through `AP::camera()`

**Changes**

- The six camera functions (zoom, focus, record, picture, tracking, video source) now go through
  `AP::camera()` instead of `AP::mount()`, so **`CAM1_TYPE` selects the destination** - 4
  (Mount) reaches a Viewpro exactly as before, 6 (MAVLinkCamV2) reaches a Gremsy camera
  component the mount cannot address at all.
- **It is not the dual-dispatch the earlier plan assumed**: `AP_Camera_Mount` already forwards
  to `AP_Mount`, so one route serves both payloads. A try-mount-then-fall-back-to-camera scheme
  would have been actively pointless: the fallback calls `AP_Camera_Mount`, which calls back
  into the very mount that just failed.
- `get_zoom_times()` and `IR_Color_Change()` stay on `AP::mount()` - neither exists in the
  `AP_Camera` interface at all.
- **`set_camera_source` is wrapped in `AP_Q30::set_video_source()`** because
  `AP_CAMERA_SET_CAMERA_SOURCE_ENABLED` resolves to `SIYI || XACTI || VIEWPRO` - a future
  Gremsy-only build would not have the function at all.
- **`AP_Q30_ENABLED` now requires `AP_CAMERA_ENABLED`** as well as `HAL_MOUNT_ENABLED`.

**Verification**

- **Build**: `./waf copter` clean, zero warnings, **1,721,584 B** (+240 B over 5.1.2). Verified
  `--define AP_CAMERA_ENABLED=0` builds at 1,695,892 B and `--define HAL_MOUNT_ENABLED=0` at
  1,653,160 B.

**Known Limitations**

- **Not bench-tested at this version.** The Viewpro no-regression claim rests on `CAM1_TYPE=4`
  routing back into identical code.
- **NEW OPERATIONAL DEPENDENCY - `CAM1_TYPE` now matters to KGCS.** Before Stage 2a the KGCS
  camera path depended only on `MNT1_TYPE`. Now a wrong or zero `CAM1_TYPE` silently disables
  KGCS camera control on a Viewpro that otherwise looks healthy.
- **Zoom units deliberately not converted** - that only bites at `CAM1_TYPE=6` and needs the
  VIO's maximum zoom, which MAVLink cannot report. Deferred as Stage 2b.
- **Still outstanding from 5.1.2**: `MNT1_PITCH_MIN` -45 -> -90 and `MNT1_PITCH_MAX` 90 -> 45 on
  the aircraft.

---

## V5.1.2 - analysis, groundwork and the pitch relocation

**Changes**

- Pitch reversal moved to accommodate the general convention of gimbals; the previous reversal
  now resides in `AP_Q30.cpp`. This fixes inverted pitch for Mission Planner and AUTO missions
  on a Viewpro. Five files: `AP_Mount_Backend.{h,cpp}` (hook and flip removed),
  `AP_Mount_Viewpro.h` (override removed), `AP_Q30.cpp` (negates at the call site),
  `GCS_PNU.cpp` (TM2 comment).
- The ACTION ON PNU regarding the Gremsy protocol is **partly** answered: IR palette, EO/IR
  source and camera component ID are answered; `CAMERA_CAP_FLAGS` and the vendor/model strings
  were still open at this version.
- The dual-GCS compatibility strategy was recorded - now in `README.md`.

**Verification**

- **Build**: clean at 1,721,348 B, and with `HAL_MOUNT_ENABLED=0` at 1,653,164 B.

**Known Limitations**

- **REQUIRES a parameter change on the aircraft: `MNT1_PITCH_MIN` -45 -> -90 and
  `MNT1_PITCH_MAX` 90 -> 45.** The firmware cannot do this itself. Flying with the old pair
  clips KGCS pitch to 45 deg down and allows 90 deg up, past the mechanical stop.
- **Not bench-tested.** The bench order when hardware is next available is in `README.md`.

---

## V5.1.1 - D10 Stage 1: stop fabricating camera values

**Changes** - seven changes across six files. Nothing here changes routing, so Viewpro behaviour
is unchanged except where a value was previously fabricated.

| # | Change | Where |
|:-:|---|---|
| 1 | `get_zoom_times()` becomes `bool get_zoom_times(uint8_t, float&)` - the ArduPilot out-param idiom its sibling `get_attitude_euler()` already uses. The base class returns `false` and writes nothing instead of fabricating `0.0f` | `AP_Mount_Backend.h`, `AP_Mount.{h,cpp}` |
| 2 | `_zoom_times` gains its missing initialiser; the Viewpro override returns `false` until the gimbal has actually reported | `AP_Mount_Viewpro.{h,cpp}` |
| 3 | All 27 camera calls check their return and report via one rate-limited helper, `report_cam_unsupported()` | `AP_Q30.{h,cpp}` |
| 4 | TM2 reports zoom honestly. **The wire format is unchanged** - 0 still goes out, because `Zoom_POS_FB` has no "unknown" encoding and adding one is an ICD change, exactly the call D12 made for TM5 | `GCS_PNU.cpp`, `GCS.h` |
| 5 | TM2 is suppressed entirely when no gimbal is configured (`get_mount_type(0) == Type::None`), where every field would be zero | `GCS_PNU.cpp` |
| 6 | `EO_zoom_pct` renamed `EO_zoom_times` with the PCT-means-times trap documented at the declaration | `AP_Q30.cpp` |
| 7 | `_current_zoom_EO` / `_current_zoom_IR` hold their last known value when zoom is unavailable, instead of being overwritten with a fabricated 0 | `AP_Q30.cpp` |

**Design points worth not re-deriving**

- **The rate limit is not optional.** KGCS streams TC2 at ~10 Hz, so on a mount without camera
  support *every* call fails on *every* tick. One message per 10 s with the first always
  reported - the same shape, and the same interval, as D6's PMU short-frame message.
- **One shared limiter, not one per function**, deliberately. The operator needs to learn "this
  mount does not do camera commands", not receive an itemised list.
- **The zoom readback in `IR_operation()` reports nothing on failure**, on purpose. It runs at
  the TC2 rate and TM2 already announces the same condition once, on the edge.
- **TM2's zoom warning has a 10 s grace period** (`PNU_CAM_ZOOM_GRACE_MS`). A working Viewpro
  reports zoom only after its first telemetry frame, so an edge evaluated from boot would warn
  on *every* startup and then immediately recover.

**Verification**

- **Build**: `./waf copter` clean, zero warnings, **1,721,424 B** (was 1,720,608 B, so Stage 1
  costs **+816 bytes**). The D13 invariant still holds: `--define HAL_MOUNT_ENABLED=0` builds at
  1,653,156 B.

**Known Limitations**

- **Not bench-tested** - no hardware was connected, so the Viewpro no-regression claim is a
  build-and-reasoning claim only.
- `_current_zoom_EO` / `_current_zoom_IR` are **write-only** - assigned in `IR_operation()` and
  read nowhere in the tree, the same situation as the D6 PMU counters. Kept rather than deleted
  so the keep-or-remove call stays with PNU. See `PMU-003`.

---

## V5.1.0 OFFICIAL - the Copter-4.7.1 migration closed

**Changes**

- Closed the V5.0.8 -> Copter 4.7.1 migration. All 8 numbered steps applied, both free-floaters
  closed as not-applied, and **13 of the 15 PNU-ISSUEs resolved**. Left open: **D10** and
  **D11**.
- The step map, the substantive deviations from V5.0.8 and the step-7 defect fixes are in
  **Appendix A** below.

**Verification**

- **Build and bench** across the 5.0.x series; see the per-issue records in `ISSUES.md`.

---

## V5.0.15 and earlier

| Version | Content |
|---|---|
| **5.0.15** | Issue 9(b) resolved. All issues except 10/11 now resolved. 10 and 11 decided to be resolved after 5.1.0. Building as "heli" model is blocked due to fatal errors it may cause |
| **5.0.14** | Issues 13 and 14 resolved together |
| **5.0.13** | Issue 7 resolved |
| **5.0.12** | Issue 1 resolved |
| **5.0.11** | Issue D6 resolved. Issue 15 was added during the analysis of D6, but also resolved |
| **5.0.10** | Issue D5 resolved but found new Issue D14. **ArduPlane, Rover and Sub have been unbuildable since `ed055d2f24` in this version** |
| **5.0.9** | Issue D1 status flipped - reviewed but not closed - but Viewpro worked fine. Issue D2 resolved. Issue D3 resolved: **do not go back to a previous version, this change is critical**. Viewpro bench findings recorded |

---

## Appendix A - the V5.0.8 migration: steps, deviations and defect fixes

Historical record. It is kept because applying a V5.0.8 snapshot file wholesale would silently
revert decisions made here, and because the step map explains why the tree is shaped as it is.

### The 8 migration steps

| # | Step | Files | Needs | Unblocks |
|:---:|---|---|---|---|
| 1 | MAVLink dialect | `Forced Submodule File/pnu_kal.xml`, `Tools/pnu/stage_mavlink_defs.sh`, `wscript` | - | 2, 4, 5, 7, 8 |
| 2 | Enum & ID registry | `ModeReason.h`, `AP_Logger.h`, `AP_Arming.h`, `AP_CAN.h`, `ap_message.h`, `GCS.h`, `AP_Arming.cpp` | 1 | 4, 5, 6, 7, 8 |
| 3 | Gimbal driver | `AP_Mount.{cpp,h}`, `AP_Mount_Backend.{cpp,h}`, `AP_Mount_Viewpro.{cpp,h}` | - | 5, 7 |
| 4 | PMU CAN stack / version.h | `AP_PMUCAN/` x6, `AP_CANManager.{h,cpp}`, `wscript` (PMUCAN line only), `version.h` | 1, 2 | 6, 7 |
| 5 | Camera adapter | `AP_Q30/` x2, `AP_SerialManager.cpp`, `wscript` (Q30 line only), `Copter.h` (include + `AP_Q30 q30`) | 1, 2, 3 | 7 |
| 6 | PMU failsafe | `events.cpp`, `Copter.h` (failsafe bit + decl), `Copter.cpp` (10 Hz call) | 2, 4 | - |
| 7 | GCS handler bodies | `GCS_PNU.cpp` (bodies), `GCS_Common.cpp` (3 hooks), `GCS.h` (D12 edge latch) | 1, 2, 3, 4, 5 | 8 |
| 8 | Object avoidance | `UserCode.cpp`, `APM_Config.h` | 1, 2, 7 | - |
| - | NMEA output | `AP_NMEA_Output.{cpp,h}` | - | **NOT APPLIED** |
| - | Vehicle tuning | `config.h` | - | **NOT APPLIED** |

### Files that must never be taken wholesale from the snapshot

`git diff v5.0.8-snapshot` is **not** a to-do list - 30 files already differ deliberately.
Before taking any file whole, check it is a pure addition:

```
git diff --stat HEAD v5.0.8-snapshot -- <file>      # 0 deletions => safe to take whole
```

| File | Would be lost |
|---|---|
| `ArduCopter/Copter.cpp` | 3x `static_assert(OFP_VER... == FW_...)` (step 4, D4); mount task kept at **50 Hz** (D8) |
| `ArduCopter/Copter.h` | `AP_Q30 q30` member behind `#if AP_Q30_ENABLED` (D13); `failsafe.pmucan` and `failsafe.pmucan_armed_healthy` bits (D9) |
| `libraries/AP_PMUCAN/*` | the whole step-4 cleanup; 4 vendored `pmucan_*.hpp` were **deleted**; DLC validation (D6), endianness comments (D15), global declarations (D14), engine-interlock send check (D7) |
| `libraries/AP_Mount/AP_Mount_Backend.{h,cpp}` | `suppress_gimbal_device_attitude_status()` (D8a); the `get_zoom_times()` out-param signature (D10 Stage 1). **Deliberately carries NO pitch sign flip** - re-adding one double-negates KGCS and inverts every standard GCS |
| `libraries/AP_Mount/AP_Mount.{h,cpp}` | the `get_zoom_times()` out-param signature (D10 Stage 1) |
| `libraries/AP_Mount/AP_Mount_Viewpro.{h,cpp}` | `suppress_gimbal_device_attitude_status()` override; D1 markers; `_zoom_times` initialiser and the `get_zoom_times()` validity test |
| `libraries/AP_SerialManager/AP_SerialManager.h` | IOMCU renumber deliberately **not** applied |
| `libraries/AP_OSD/AP_OSD_ParamSetting.cpp` | `"Q30"` padding deliberately **not** applied |
| `libraries/GCS_MAVLink/GCS_Common.cpp` | upward-proximity block deliberately **not** commented out; the handler bodies live in `GCS_PNU.cpp`, only 3 hooks remain here (D5); plus one `case MAVLINK_MSG_ID_CAMERA_SETTINGS` label (D10 Stage 3) |
| `libraries/GCS_MAVLink/GCS_PNU.cpp` | PNU-only file; the step-7 defect fixes and `#if` guards; the D10 Stage 1 TM2 zoom-truth and no-gimbal gate |
| `libraries/AP_Q30/AP_Q30.{h,cpp}` | `send_cmd_speed()` **and** `send_cmd_angle()` negate pitch at the call site; `AP_Q30_ENABLED` guard and the D14 global declarations; the 27 return checks and `report_cam_unsupported()`; camera calls routed through `AP::camera()` and `set_video_source()` |
| `ArduCopter/UserCode.cpp` | `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` guard around the OA block |
| `ArduCopter/events.cpp` | D9(a) recovery-clear fix and D9(b) arm-time latch gate; `LOGGER_WRITE_ERROR` portability fix |
| `ArduCopter/AP_Arming_Copter.{h,cpp}` | `pmucan_checks()` prearm warning and the arm-time healthy latch (D9b) |
| `libraries/GCS_MAVLink/GCS.h` | `OFP_VER_*`; the D3 dialect `#error` guard; the D12 `prev_prx_failed` latch; the D10 `prev_zoom_unavailable` latch; PNU handler declarations |
| `libraries/AP_Q30/AP_Q30_config.h` | PNU-only file - the `AP_Q30_ENABLED` flag (D13), which also requires `AP_CAMERA_ENABLED` since Stage 2a |
| `libraries/AP_Camera/AP_Camera_MAVLinkCamV2.{h,cpp}` | the one-shot guard on the `"Camera: ..."` statustext; the `CAMERA_SETTINGS` decode, 1 Hz poll, `get_reported_zoom_level()` and the `send_camera_settings()` override. All of it upstream-submittable |
| `libraries/AP_Camera/AP_Camera.{h,cpp}`, `AP_Camera_Backend.h` | the `get_reported_zoom_level()` accessor and its backend virtual - deliberately unit-neutral |
| `libraries/AP_PNU_CAN/*` | PNU-only files - the shared CAN driver (5.1.11) |
| `libraries/AP_PNUGimbal/*` | PNU-only files - `AP_PNUGimbal_Backend`, `AP_PNUGimbal_Homing`, `AP_PNUGimbal_CubeMars` + codec tests |
| `libraries/AP_Mount/AP_Mount_PNUGimbal.{h,cpp}` | PNU-only files - the `MNT1_TYPE=15` mount backend (5.1.12) |
| `libraries/AP_Mount/AP_Mount.{h,cpp}`, `AP_Mount_config.h`, `AP_Mount_Params.cpp` | `PNUGimbal = 15` in the `Type` enum, its factory case, `HAL_MOUNT_PNUGIMBAL_ENABLED`, and `15:PNUGimbal` in the `MNT1_TYPE` `@Values` |
| `libraries/AP_PNUSAR/*` | PNU-only files |
| `libraries/AP_CANManager/AP_CANManager_CANDriver_Params.cpp` | the `CAN_Dn_PNU_` subgroup at index 8 (index 7 retired, not reused), and 15/16 added to the `PROTOCOL` `@Values` list |
| `libraries/AP_CANManager/AP_CAN.h`, `AP_CANManager.{h,cpp}` | `PNU_CAN = 15`, `PNU_CAN_LEGACY = 16` (deprecated alias), and their factory cases |
| `libraries/AP_Arming/AP_Arming.cpp` | the PNU protocols listed in the `AP_CAN::Protocol` switch - required to compile under `-Werror=switch`, zero runtime effect |
| `Tools/scripts/build_options.py` | the `Camera / Q30` build option (D13) and `Gimbal / PNUGIMBAL` (5.1.12) |
| `ArduCopter/version.h` | the DEV version and the release note describing what it carries |
| `README.md`, `CHANGELOG.md`, `ISSUES.md` | these files - never taken from the snapshot |

Most other deviations are KAL -> PNU / PNU-KAL comment renames, which are cosmetic but still
make a wholesale copy a regression.

### Substantive deviations from V5.0.8, with reasons

| Deviation | Why |
|---|---|
| `SerialProtocol_IOMCU` renumber skipped, `"Q30"` OSD padding skipped | `SerialProtocol_Q30` was deleted in V5.0.6; the renumber only broke param compatibility |
| 4 `pmucan_*.hpp` deleted (1163 lines) | vendored libuavcan v0; only 3 constants were used, all identical in `AP_HAL::CANFrame` / `CANIface` |
| Mount task kept at 50 Hz | `AP_Mount_Viewpro::update()` self-throttles to 100 ms, so gimbal traffic is 10 Hz either way; at a 10 Hz task rate the period *equals* the throttle and late ticks cause 5 Hz bursts. See D8 |
| msg 285 suppression made opt-in | V5.0.8 suppressed it for every gimbal, not just Viewpro. See D8 |
| `GCS.h` dead CAM macros removed, `OFP_VER_*` tracked against `version.h` | were stale/unused; guarded by `static_assert` in `Copter.cpp` |
| `AP_PMUCAN` fixes | dropped-frame at RX budget, dead branch, `&` -> `&&`, `RXdrain()` extraction, named constants, ctor init, `TXspin` void |
| `send_proximity()` upward-distance block **not** commented out | V5.0.8 wrapped it in `/* Currently, no upward sensor */`. `AP_Proximity::get_upward_distance()` already returns false when no backend supplies one, so the comment-out is a no-op here and a silent regression for `AP_Proximity_RangeFinder` / `_MAV` / scripting users |
| OA sender/handler guarded `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` | `GCS_Common.cpp` is core; `AC_Avoid` and `AP_Proximity` are optional. Without it a proximity-disabled build hits the `try_send_message()` default case, which spams "Sending unknown message" and panics in SITL |
| step 8: `copter.avoid.` -> `avoid.` | inside a `Copter` member function; `copter.` is the global instance and redundant |
| step 8: tabs -> 4 spaces in the added lines | V5.0.8 mixed tabs and spaces in both hunks |

### Step 7 defects fixed while applying the V5.0.8 GCS block

The block itself now lives in `GCS_PNU.cpp` (D5); these fixes travelled with it. None change the
wire format of any ICD message except where stated.

| Defect | Fix |
|---|---|
| `CAM_ATTITUDE_STATUS.Yaw_REL_ANG = CAM_ATTITUDE_STATUS.Pitch_IMU_ANG = yaw * 10;` - copy-paste typo breaking the `X_REL = X_IMU` pattern of the two lines above it. `Pitch_IMU_ANG` carried **yaw**, and `Yaw_IMU_ANG` was never assigned, so TM2 reported it as 0 forever | assign `Yaw_IMU_ANG`. **Changes what KGCS receives** in two TM2 fields |
| `send_message_flcc_gcs_object_avoidance_status()` called `proximity->distance_min_m()`, `distance_max_m()` and `get_horizontal_distances()` **before** the `proximity == nullptr` check | null check moved to the top |
| `handle_gcs_flcc_object_avoidance_cmd()` dereferenced `AP::ac_avoid()` with no null check | added |
| TM12 logged `Load_Current` twice - the `ICMD` column got `Load_Current` instead of `Current_Control_Command` | log-only |
| TM13 labels `FLMJ,FLMN` were fed `Version_FLCC_Sub_Number, Version_FLCC_Main_Number` - major and minor swapped | log-only |
| `Zoom_POS_FB` cast `(int16_t)` into an `int8_t` ICD field | cast `(int8_t)` |
| `AP::logger().Write()` calls unguarded | wrapped in `#if HAL_LOGGING_ENABLED` |
| `GCS_DEBUG` / `GCS_DEBUG_PRINT` defined and never used; `extern PREV_CAM_CMD` declared and never used | removed |
| `get_horizontal_distances()` return value ignored, `dist_array.valid(i)` never consulted, sector loop hard-coded to `8` | gate on `sensor_failed()` + check the return + skip invalid sectors + `PROXIMITY_MAX_DIRECTION`. See D12 |
| hard-coded 1000 / 1500 cm OA thresholds | `PNU_OA_ALERT_DISTANCE_CM` / `PNU_OA_WARN_DISTANCE_CM` - see D12 |
| tabs, commented-out `//Debug only - remove later` blocks, commented-out legacy angle-compare block | removed / converted to 4 spaces |

Left alone on purpose: the `warning_level |= 2` / `|= 1` then `> 2 -> 2` clamp (obscure but
correct - alert wins) and `MAV_SEVERITY_CRITICAL` on the avoidance on/off statustext (operator
feedback KGCS relies on).

### Step 8 notes

- `userhook_MediumLoop()` runs at 10 Hz with a 75 us budget (`Copter.cpp:267`). Both calls only
  set bits in the per-channel deferred-message mask, so the cost is trivial.
- `USERHOOK_INIT` fires from `system.cpp:132`, long after `GCS::_singleton` is set, so the
  `gcs()` writes there are safe. The two fields it zeroes are already zero from static init - it
  is documentation, not a fix.
- `Object_Avoidance_Mode` is recomputed every tick from `proximity_avoidance_enabled()`, which is
  `_proximity_enabled && (AVOID_ENABLE & AC_AVOID_USE_PROXIMITY_SENSOR)`. **So with
  `AVOID_ENABLE` bit 1 clear, KGCS gets "Avoidance ON Level n" from the TC4 handler but TM5 keeps
  reporting mode 0.** That is correct - TM5 reports what is in force, not what was asked for -
  but it is a confusing setup trap worth checking on the bench.
- `gcs().send_message()` broadcasts to **every** channel, so a second GCS on another link also
  receives TM2/TM5 (~520 B/s combined). Harmless - MAVLink ignores unknown ids - but it is
  bandwidth spent on a link that cannot use it.

### Free-floaters - reviewed, deliberately NOT applied

Both carry an in-file note; list them with `grep -rn "PNU-NOT-APPLIED" libraries/ ArduCopter/`.

| File | V5.0.8 change | Why not applied |
|---|---|---|
| `ArduCopter/config.h` | `LAND_DETECTOR_ACCEL_MAX` 1.0f -> 3.0f | Probably needed for gas-engine vibration, but it relaxes a safety threshold past upstream's own WoW-corroborated `land_detector_scalar = 2`. Belongs in the board hwdef, not the vehicle source, and needs flight-log evidence first |
| `libraries/AP_NMEA_Output/*` | Korean drone-ID / UTM rewrite | Comments out `HAL_NMEA_OUTPUT_ENABLED` in the `.cpp` only -> fails to build wherever the feature is off; deletes GPRMC and PASHR but keeps their bits; passes a double to `%d`; hardcodes the station ID and the 1 Hz rate; non-standard VTG. Re-specify as separate PNU work if needed |

Open question for the NMEA decision: is any serial port on the aircraft set to
`SERIALn_PROTOCOL = 20` (NMEAOutput)? If not, this is dead code and the issue closes.

---

## Appendix B - the fake PMU engine model, as specified

Specified by PNU 2026-10-27, closed against `PMU CAN Comm Protocol.md` and PNU's answers
2026-10-29. The ICD is an English rendering of the PMU's Korean PDF, written by PNU; the fake is
built from that, not from `AP_PMUCAN::handle_frame()`, so it does not merely reproduce our own
decoder's assumptions.

### RPM ladder. No step may be skipped.

| State | RPM | Throttle | Entered by | Load current | Head temps |
|---|---|---|---|---|---|
| STOPPED | 0 | 0% | boot, Stop, cold-start abort | - | decay to 25 at 5 degC/s |
| CHOKING | ramp to 1000, hold 2 s, then 0 | 8% | Start with **choke closed** | - | - |
| CRANK | 1000, hold 2 s | 8% | Start with **choke open** | - | - |
| WARM-UP idle | ramp to **4200** over 1 s, hold | 8% | 2 s at 1000 elapsed | - | climb 25 -> 100 at 5 degC/s |
| GROUND idle, cooling on | **5000** | 13% | head temps reach **80** | **6.3 A** | continues climbing to 100 |
| FLIGHT | **6200** | 50% | takeoff detected **from 5000 only** | **87.5 A** | climb to **180** at 5 degC/s |
| COLD-START ABORT | -> 0 | 0% | takeoff attempted below 5000 | - | decay to 25 |

CHOKING leaves the engine **stopped**: it reaches 1000 rpm for 2 s and stops, and the status word
must still read as not-running afterwards. Ceiling is ~6500 and the model never exceeds 7000 even
at 100% throttle; above 6500 sets the over-speed bit rather than clipping silently.

**Battery and generator are tightly coupled** - same voltage, same current. The flight load is
87.5 A and *something* carries it: the engine's generator while the engine runs, the battery the
instant it stops. That is what makes the cold-start abort observable - RPM falls to zero, ILD
stays at the flight load, and **IBAT goes to -87.5 A (raw -875), negative for discharge**, which
is the only place in the whole model where IBAT is not zero.

`System_Voltage` is **48.0 V at all times**. `Battery_Temp` and `PMU_Temp` are **0** -
deliberately not modelled. `Fuel_Quantity` is **100** and the Fuel Low bit is held clear, because
the real aircraft has no fuel sensor.

### `PMU_Status.PMU_Status` - the 16-bit status word

**NOT decoded anywhere in the FC today**; `handle_frame()` stores it whole and `GCS_PNU.cpp`
forwards it whole. Written down because the fake has to generate it and nothing else in the tree
records it.

| Bit | 1 means | 0 means |
|---|---|---|
| 0 | Manual throttle | PMU auto throttle |
| 1 | Engine is in **starting** mode | after starting, or not starting |
| 2 | Engine is **stopping** | after stop, or already stopped |
| 3 | Choke valve **closed** | choke valve open |
| 4 | Cooling fan ON | cooling fan OFF |
| 5 | *reserved* | |
| 6 | Engine RPM too high (> 6500) | |
| 7 | Engine stopped from unknown reason | |
| 8 | Fuel low | |
| 9 | Engine exceeded MTBF, needs overhaul | |
| 10 | Engine cylinder high-temperature warning | |
| 11, 12 | *reserved* | |
| 13 | Battery overcurrent warning | |
| 14 | Battery undervoltage warning | |
| 15 | Battery overvoltage warning | |

**Bits 13, 14 and 15 come from the ICD, not from the 2026-10-27 specification**, which listed
11-15 as reserved; PNU confirmed that was a slip in that message, so the ICD stands. The ICD gives
no numeric threshold for bits 6 and 10, so the 6500 rpm and 230 degC figures are PNU's and the
fake uses them as the trip points.

**There is no "engine is running" bit.** Bit 1 is set only *during* the start, bit 2 only *during*
the stop, and both read 0 once the engine is up. Running is inferred from the RPM range, and
**that inference belongs to KGCS, not to the FC** - confirmed by PNU 2026-10-29. Over-speed (bit
6) is the same division: it is how KGCS annunciates an abnormal generator, and the FC's only job
is to carry the word intact. **So the FC passing `PMU_Status` through undecoded is the design, not
an omission.**

### Scales, signs and the four things the ICD records that are nowhere else

- **Scale is 0.1, not x100.** Every INT16 analog field - TEMP, IBAT, TEMP1, TEMP2, ILD, VBUS,
  GCMD, GAGE, PTEMP - is `physical = raw * 0.1`. The FC itself applies no scale at all, so the
  conversion remains entirely KGCS's. `Throttle_Position_Report` (PCL) and `Fuel_Quantity` (GAS)
  are `INT8` and carry **raw percent**, scale 1.0.
- **`IBAT` is positive for CHARGING, negative for DISCHARGING.** Only ILD and IBAT are measured,
  and the generator's output is inferred: `generated = ILD + IBAT`. While the engine runs the fake
  holds IBAT at 0, by PNU's instruction.
- **The cold-start abort latches** and needs a fresh Start.
- **The cooling-device current is 6.3 A, raw 63.** The original figure was 6.25 A, which is raw
  62.5 and does not exist at 0.1 A resolution. The pair 6.25 A / 87.5 A at 48 V is 300 W / 4200 W,
  so the round numbers were always the watts. Every other quantity lands exactly: 48.0 V -> 480,
  87.5 A -> 875, -87.5 A -> -875, 25 degC -> 250, 80 -> 800, 100 -> 1000, 180 -> 1800,
  230 -> 2300. All well inside INT16.
- **The PMU never transmits on its own.** Status is strictly RTR-request / Data-response, which is
  what `_rtr_id[]` and the five-slot rotation do. The fake PMU must be RTR-driven, not
  free-running - and the silent trigger therefore means *stop answering RTRs*, which is a truer
  failure mode than a dropped broadcast.
- **`HOURCNT` is in SECONDS.** The ICD says so explicitly - "despite its name". Our field is called
  `Engine_Hour_Count` and the MAVLink field `HOURCNT`, both of which read as hours. The FC only
  forwards it raw so there is **no FC bug**, but KGCS must divide by 3600, and nothing in the FC
  should ever present this value as hours.
- **The ICD's bit rate of 500 kbps does not apply here.** The CubeMars motors have no option but
  1 Mbit/s, so the whole of CAN1 runs at 1 Mbit/s and the PMU is reconfigured upward to match.
  Confirmed by PNU 2026-10-29 - the ICD figure is superseded, not overlooked.
- **`ICMD` width is inferred, and its type and scale are `TBD` in the ICD.** We decode it as INT16,
  which is the only width that fits, but the signedness and scale are assumed. It is a
  developer-debug field; the fake echoes the last BATCTRL into it and nothing should depend on its
  value.
- `MJ` and `MN` are **packed BCD** - `0x12` is version 12, not 18. The FC forwards them raw. The
  fake reports MJ = 1, MN = 0, which reads identically in BCD and binary, so the choice sidesteps
  the encoding rather than exercising it.

### `AP_PMUCAN` cross-checked against the ICD, 2026-10-29. Clean.

The ICD warns twice that the PDF prints field *order* without byte offsets, and says not to
finalize a decoder on the printed left-to-right order alone. **Our decoder's offsets match that
order in all five frames**, and the 5.1.16 bench run read sane values from real hardware, which is
the capture validation the ICD asks for:

| Frame | ICD order | `handle_frame()` offsets | |
|---|---|---|---|
| BATSTS | BSTS, TEMP, IBAT, GAGE | 0:2, 2:4, 4:6, 6:8 | match |
| ENGSTS | TEMP1, TEMP2, RPM, GAS, PCL | 0:2, 2:4, 4:6, 6, 7 | match |
| AUX1STS | ICMD, ILD, VBUS, GCMD | 0:2, 2:4, 4:6, 6:8 | match |
| AUX2STS | HOURCNT, RSV, PSTS, PTEMP | 0:3, (3 skipped), 4:6, 6:8 | match |
| VERSTS | DATE, MN, MJ, RSV | 0:4, 4, 5, (6:8 ignored) | match |

Also confirmed: the ICD's instruction 2 - *"frames with an invalid DLC must not overwrite the
latest valid values or valid reception timestamps"* - is already satisfied, because the D6
short-frame check returns before `_pmu_frames_seen` and `_last_pmu_frame_ms` are touched.

### Defaults chosen by this project, not specified by PNU

Each is a one-line change: `Engine_Hour_Count` increments whenever RPM > 0;
`Current_Control_Command` and `Battery_Quantity_Command` echo the last BATCTRL received; bit 1 is
held from Start until the warm-up idle RPM is reached and bit 2 for the stop ramp; the cold-start
abort latches, needing a fresh Start, and sets bit 7.
