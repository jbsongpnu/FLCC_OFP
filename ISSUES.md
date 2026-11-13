# ISSUES - PNU-KAL FLCC OFP

Technical issues, pending engineering decisions, known risks and the resolved-issue history.
Current status and plan are in `README.md`; the version-by-version record is in `CHANGELOG.md`.

## Identifiers

- **`PNU-ISSUE D1`-`D15`** are the original migration issue ids. **They are never renumbered**,
  and each has a `PNU-ISSUE(<id>)` marker at the code site. List them with:

  ```
  grep -rn "PNU-ISSUE" libraries/ ArduCopter/
  ```

- New issues use subsystem identifiers: `PMU-nnn`, `GIMBAL-nnn`, `SAR-nnn`, `GCS-nnn`,
  `CAM-nnn`. These are documentation ids; they have no code markers unless stated.

## Contents

1. [Open Issues](#1-open-issues)
2. [Pending Engineering Decisions](#2-pending-engineering-decisions)
3. [Known Risks and Unverified Assumptions](#3-known-risks-and-unverified-assumptions)
4. [Resolved Issues](#4-resolved-issues)

---

# 1. Open Issues

## GIMBAL-001 - the driver assumes the motors feed back autonomously

| | |
|---|---|
| **Status** | OPEN. **Blocks step 3f**, and therefore 3g |
| **Component** | `libraries/AP_PNUGimbal/AP_PNUGimbal_Backend.cpp`, `AP_PNUGimbal_CubeMars.{h,cpp}` |
| **Versions** | present since 5.1.13; exposed by the fake gimbal 2026-10-30 |
| **Related** | `SAR-001`, `GIMBAL-004`, `CHANGELOG.md` V5.1.23 |

**Description and impact.** `AP_PNUGimbal_Backend` treats feedback as a stream and faults when
none arrives for `AP_PNUGIMBAL_FB_TIMEOUT_MS` (500 ms). With the fake gimbal in
`--fb-mode per-command` - feedback only in reply to a command - the driver faults **523 ms after
the enable, every time**, because nothing keeps the conversation alive between the enable and the
first motion command, and motion is only sent in RUN. With `--fb-mode periodic --fb-rate 50` the
same build reaches RUN and streams normally.

**Root cause.** `PNUGimbal_CAN_Protocol_Type_CubeMars.md` sections 1 and 9.6 both record that
**the manual never says what triggers a feedback frame**, and this project refused to invent it.
The driver was written assuming autonomous feedback.

**Required verification - on real motors, in this order.**

1. **The feedback trigger**, before the rate. Enable one motor, send nothing, and watch the bus.
   Does it report on its own, or only in answer to a command?
2. **The rate**, if it is autonomous.

**If the motors only answer commands**, two consequences follow: `AP_PNUGimbal` needs a
keep-alive, and **step 3f's 40 ms `ATT_FRESH` budget has to be re-derived** - ICD 14.7 makes an
ATT source older than 40 ms an automatic observation termination, which needs at least 25 Hz.

**Remaining limitation.** A green SITL run is not evidence about the real motors. The fake says
loudly which trigger mode it is using, every time it starts and in every status line, for exactly
this reason.

## GIMBAL-002 - the homing parameters are provisional and unmeasured

| | |
|---|---|
| **Status** | OPEN. `CAN_Dn_PNU_HOME` defaults to **0** and must stay there until this is done |
| **Component** | `libraries/AP_PNUGimbal/AP_PNUGimbal_Homing.cpp`, `AP_PNU_CAN` parameters |
| **Versions** | step 1d implemented in 5.1.14 |
| **Related** | `GIMBAL-001` (homing needs feedback), `CHANGELOG.md` V5.1.14, V5.1.23 |

**Description and impact.** The Appendix 1 endpoint search deliberately drives a bulky SAR
payload into three sets of mechanical hard stops **with the mechanical clamp disabled** - the
encoder is relative, so the clamp protects nothing until the zero is found. During homing the
only thing protecting the payload is the low `kp` and the bounded feed-forward torque. That makes
MIT mode mandatory and makes it the single most load-bearing fact in the design.

**Bench tuning order. Each one makes the next safe to try.**

| # | Parameter | Default | How to get the real value |
|:--:|---|---|---|
| 1 | `PNU_HKD`, `PNU_HVEL` | 0.5, 10 deg/s | **Stall torque is HKD x HVEL (rad/s).** Pick the gentlest push that still reaches a stop against the payload's own friction. Start low and raise HKD; do NOT raise HVEL to get there, because that raises impact energy as well as torque |
| 2 | `PNU_HDLT`, `PNU_HSTL` | 0.1 deg, 1000 ms | Log an axis creeping into a stop and read the residual motion. Too small and encoder noise means a stall is never detected; too large and the axis is called stopped while still creeping. **The dwell is the only value PNU has settled** |
| 3 | `PNU_HRTOL` | 0.5 deg | Approach one stop twice by hand and measure the spread. Must stay below the 2% backoff - 1.2 deg on a 60 deg axis, 0.8 on 40 - or the second approach starts inside the tolerance and proves nothing. `init()` rejects it if not |
| 4 | `PNU_HTOL` | 3 deg | Measure stop-to-stop travel a few times with the SAR fitted. **Stop compliance under a heavy payload may consume most of 3 degrees on its own**, so this is the one most likely to need raising - but raising it blunts the obstruction check |
| 5 | `PNU_HKP` | 5 | Holding stiffness for the idle axes and the move to the midpoint. Enough to stop the payload sagging, no more |

**Run the first homing with the SAR mass fitted but the aircraft restrained**, and watch the
statustexts: each axis reports its measured travel and midpoint before it zeroes, which is the
number that tells you whether the geometry is what everyone thinks it is.

**Also set `PNU_DRLL` / `DPIT` / `DYAW` before the first homing run**, and check them the cheap
way: command a small positive angle on one axis in RUN and confirm the payload moves the way the
mount convention says it should. Getting one wrong means that axis stabilises *into* the
disturbance rather than against it.

**Verification results so far.** SITL only. The rehearsal against the fake gimbal on 2026-10-30
ran the sequence end to end - roll travel 60.6 deg, pitch 60.6, yaw 40.5, each midpoint -0.1,
`HOMING -> ALIGN -> RUN` in 40 s - and showed the modelled stop compliance consuming **0.6 deg of
`PNU_HTOL`'s 3 deg default** before any real-world asymmetry. That de-risks the run and
**measures none of the five values above**; the compliance figure is a test knob in the fake, not
a measurement of this mount.

## GIMBAL-004 - seven things the CubeMars manual does not specify

| | |
|---|---|
| **Status** | OPEN, by design. Listed at the top of `AP_PNUGimbal_CubeMars.h` so a later session does not fill one in with a plausible default |
| **Component** | `libraries/AP_PNUGimbal/AP_PNUGimbal_CubeMars.h` |
| **Related** | `GIMBAL-001` is the first of these and the one that blocks work |

Command period, feedback trigger, feedback rate, loss timeout, watchdog interval, acknowledgment
of the universal commands, and any runtime mode-switch command. `AP_PNUGIMBAL_LOOP_HZ` is still
the skeleton's **100** for the same reason - nothing has said what rate the chosen mode wants.

**Bench checklist, when the motors are available. None of this has met hardware.**

1. Confirm the configured mode, CAN ID and Master ID of each motor in the PC utility, and that
   `CAN_Dn_PNU_MODE` / `*ID` match. Confirm the P/V/T/KP/KD ranges and set `PNU_*MAX` to the same
   numbers.
2. **Velocity units.** The manual mixes rad/s and rev/s. Measure one slow rotation before any
   physical-unit control is exposed, and do not silently multiply by 2*pi.
3. **Quantization.** Send the four MIT vectors the tests already pin and check what the motor
   does. The +/-6 rad/s rows in the manual are the ones expected to disagree.
4. **Universal commands in velocity mode.** They are DLC 8 on an ID whose ordinary command is
   DLC 4, and the manual gives no example. **This is the single most likely thing to be wrong.**
5. **Zero-setting persistence** and what clearing a fault actually recovers - both unspecified.

## SAR-001 - `0x404` carries no measured attitude

| | |
|---|---|
| **Status** | OPEN. This is step 3f |
| **Component** | `libraries/AP_PNUSAR/AP_PNUSAR_Nav.cpp`, `libraries/AP_PNUGimbal/` |
| **Versions** | sentinels since 5.1.22 |
| **Related** | `GIMBAL-001` blocks it; ICD sections 14.7, 16.1, 16.4 |

**Description and impact.** ICD section 16.4 forbids substituting setpoints and marking them
valid. The hazard is not to the FC: it is that **Lumir may validate their processing against
attitude that never varies, and not discover it until the gimbal is fitted.**

**What shipped in 3e changes the shape of this risk without removing it.** `0x404` carries
sentinels - `INT16_MIN`, `INT16_MIN`, `0xFFFF`, `ATT_Flags` empty - so a receiver that reads
values before flags sees obviously invalid numbers rather than a level, north-facing payload.
That is what 16.4 asks for. What it cannot do is guarantee that Lumir's receiver handles
sentinels at all: **an implementation that clamps or casts them produces something worse than
zeros would have been.**

**Mitigations, and the third is the one that matters.**

1. **Sentinels rather than zeros**, decided when 3e was written. The planned alternative - a
   parameter holding a placeholder attitude, switchable without a firmware build - was dropped
   and **never implemented**: there is nothing to parameterise once the frame itself says
   "unavailable".
2. **A boot-time warning while there is no attitude source** - `SAR: ATT age check off, no
   source` - on the pattern of the `PNU_HOME=0` warning.
3. **Lumir is told in writing that `0x404` is unpopulated for this phase, and what the sentinel
   values are.** A project action, not a code change. **STILL OUTSTANDING.**

**Consequence for `CAN_Dn_PNU_SCHK`.** Bit 2 (gimbal attitude age) is off by default and
**setting it today refuses every Start and terminates every observation**, because a constant has
no age. 3f is what makes that bit usable.

**What step 3f needs that this project does not yet have.**

- **A measured CubeMars feedback trigger and rate** - `GIMBAL-001`.
- Bench-tuned homing and `PNU_HOME=1` - `GIMBAL-002`.
- **Mechanical facts from PNU**: the axis stacking order; whether the SAR body axes coincide with
  the inner-axis frame; and the per-axis rotation sign for `DRLL`/`DPIT`/`DYAW`.
- **An agreement with Lumir** on the fixed transform between the SAR reference axes and the
  antenna electrical pointing axis (ICD 16.1).

## PNU-ISSUE D10 - `AP_Q30` camera routing

| | |
|---|---|
| **Status** | OPEN as an issue id. **FC-side work is COMPLETE**; what remains is KGCS implementation, which is not this repository |
| **Component** | `AP_Q30.{h,cpp}`, `AP_Mount*.{h,cpp}`, `GCS_PNU.cpp`, `GCS.h` |
| **Related** | `D11` is the capability gap behind it; `CAM-001`, `CAM-002` |

**Original description.** `AP_Q30` routed every camera function through `AP::mount()`, never
`AP::camera()`. Valid for Viewpro, where one serial protocol carries gimbal + camera, but on any
other mount all 27 camera calls fell through to the base class and did nothing, silently. Of
`AP_Q30`'s 30 `mount->` calls only three are genuinely gimbal operations - `set_angle_target()`
at line 53, `set_rate_target()` at 91 and `set_rate_target(0,0,0,0)` at 137. The other 27 are
camera operations addressed to the mount object: `set_zoom` x7, `IR_Color_Change` x6,
`set_camera_source` x4, `set_focus` x2, `record_video` x2, `set_tracking` x2, `get_zoom_times`
x2, `take_picture`.

**Result before Stage 1:** gimbal pointing works, every camera command silently does nothing, and
KGCS is told zoom is 0x. No error, no warning, no build failure.

**Staged resolution.**

| Stage | What | Status |
|:--:|---|---|
| 0 | Ask PNU whether KGCS TC2 must drive non-Viewpro cameras at all | answered: yes, via GCS-direct routing |
| 1 | Stop fabricating values; check the 27 unchecked camera return values | **APPLIED 2026-10-01 (5.1.1)** |
| 2a | Route through `AP::camera()`, `CAM1_TYPE` selects the destination | **APPLIED 2026-10-12 (5.1.3)**, hardware-validated 2026-10-13 |
| 2b | Absolute zoom | **CLOSED 2026-10-13 (5.1.5) by not implementing it in the FC** |
| 3 | `get_zoom_times` via `CAMERA_SETTINGS` | **DONE 2026-10-13 (5.1.6)**, bench-confirmed 5.1.7 |
| KGCS | EO/IR source (`C_SOURCE`), IR palette (`C_T_PALETTE`), object tracking and absolute zoom (`USER_4`) move to KGCS-to-VIO direct routing | **PNU/KGCS work, not this repo** |

**Why Stage 2 was not dual-dispatch.** `AP_Camera_Mount` - what `CAM1_TYPE=4` selects - already
forwards `set_zoom`, `set_focus`, `record_video`, `trigger_pic`, `set_tracking`, `set_lens` and
`set_camera_source` straight to `AP_Mount`. A try-mount-then-fall-back-to-camera scheme would
have been actively pointless: the fallback calls `AP_Camera_Mount`, which calls back into the
very mount that just failed.

**The Gremsy protocol answers arrived 2026-10-08 and cut the remaining work down rather than
unblocking it wholesale.** IR palette is `C_T_PALETTE`, EO/IR source is `C_SOURCE`, both over
`PARAM_EXT_SET`, and the camera is a separate component ID. Because **ArduPilot implements no
`PARAM_EXT_*` at all**, those two functions can only be reached by direct KGCS-to-payload
routing - they leave the FC entirely, and no amount of `AP_Q30` routing would have reached them.

**Mappings the GCS-direct work needs, now known.**

TC2 video source maps 1:1 onto `C_SOURCE` (`PARAM_EXT_SET`, uint32 in bytes 0-3, **binary not
ASCII**):

| TC2 `Tracking_CMD` | `C_SOURCE` |
|:--:|---|
| 10 EO full + IR PIP | **0** (EO+IR) |
| 11 IR full | **2** (IR) |
| 12 IR full + EO PIP | **3** (IR+EO) |
| 13 EO full | **1** (EO) |

IR palette is `C_T_PALETTE` 0-9, named: WhiteHot, BlackHot, Rainbow, RainbowHC, Ironbow, Lava,
Arctic, Globow, Gradedfire, Hottest. TC2 has six codes (14-19), so there is room to map them
properly instead of the current Viewpro workaround where Colors 3 and 4 fall back to
WhiteHot/BlackHot.

**Tracking is on the payload app via `USER_4`** (`p1=4, p2=0, p3=0, p4` 0 stop / 1 active /
2 EagleEyes; target rectangle with `p3=1`), **not** `MAV_CMD_CAMERA_TRACK_POINT` on the camera.
It also requires `C_SOURCE=1` and `TRACK_MODE=0` first.

**Absolute zoom is `MAV_CMD_USER_4` 31013, `p1=2, p2=0, p3=4, p4 = zoom x`** to the payload app
(component 26) - absolute zoom **in times**, no percentage, no divisor, no mode dependency. Being
`USER_4` to the payload app, ArduPilot has no route to it.

**Remaining limitation.** `MAVn_OPTIONS` bit 1 (`NO_FORWARD`) must be **clear** on the VIO link
for any of the GCS-direct routing to work.

## PNU-ISSUE D11 - what the MAVLink camera backend cannot do

| | |
|---|---|
| **Status** | OPEN as an issue id. One of the two gaps is resolved; the other is vendor-specific by nature and has left the FC |
| **Component** | `AP_Camera_MAVLinkCamV2.cpp` |
| **Related** | `D10` is *where* calls go; D11 is *what the destination can do* |

Measured against pristine 4.7.1:

| `AP_Q30` needs | `AP_Camera_MAVLinkCamV2` | `AP_Camera` base | Status |
|---|:-:|:-:|---|
| `record_video`, `set_zoom`, `set_focus` | yes | yes | works |
| `take_picture` | - | yes | works via base |
| `set_tracking`, `set_camera_source` | - | yes | base only - check it reaches a MAVLink camera |
| `get_zoom_times` | - | - | **RESOLVED 2026-10-13 (Stage 3)** |
| `IR_Color_Change` | - | - | **no path, and none possible** |

- **`get_zoom_times`** - resolved. `AP_Camera_MAVLinkCamV2` now decodes `CAMERA_SETTINGS`
  (msg 260). The gap was bigger than first assumed: the message was not even *routed* to
  `AP_Camera`, so a backend decode alone would have been dead code. A genuine upstream gap, and
  the fix is written to be submittable.
- **`IR_Color_Change`** - MAVLink defines no thermal-palette message. Vendor-specific by nature;
  would need Gremsy's own extension or the camera's parameter protocol. Cannot be ported from the
  Viewpro implementation. It is now GCS-direct.

**A more expensive route was considered and is not needed**, recorded because the idea is sound
and will come up again: the VIO streams `EO_ZOOM` and `IR_ZOOM` separately as telemetry indexes 0
and 1 from the payload app on component 26, so the FC could have selected the right one itself.
That would have required discovering component 26 (ArduPilot has **no** reference to
`MAV_COMP_ID_USER2` anywhere), sending a `SET_MESSAGE_INTERVAL` whose param1 Gremsy repurposes as
a vendor index, decoding `PARAM_VALUE`/`DEBUG` from that component, and routing those to `AP_Q30`
- which has no message handler, so a **second** `GCS_Common.cpp` hook.

## Gremsy VIO integration reference

Supporting material for `D10` and `D11`. The fuller message reference extracted from Gremsy
PayloadSDK 3.2.0 source lives in `VIO_MAVLink_Messages.md` in the repository root; it is the
message list the VIO actually implements and **overrides Gremsy's own
`Mavlink_Command_List_v2.0.0.pdf` where the two differ**. Taken as given - it has not been
verified against a running payload beyond the bench results recorded in `CHANGELOG.md` V5.1.4.

**The VIO addresses three components, not two.** Much of what KGCS needs is not on the camera at
all:

| | Component | Default |
|---|---|---|
| **C** camera | 100-105 | **101** |
| **G** gimbal | 154 or 171-175 | 154 |
| **P** payload app | 26 (`USER2`) | fixed |

`AP_Camera_MAVLinkCamV2::find_camera()` locates the payload with
`find_by_mavtype_and_compid(MAV_TYPE_CAMERA, compid, _sysid)` - see the component ID trap in
section 3.

### How the three control paths work

| Path | GCS -> ArduPilot | ArduPilot -> Gremsy payload |
|:--:|---|---|
| FC-managed gimbal | `GIMBAL_MANAGER_SET_ATTITUDE`, `GIMBAL_MANAGER_SET_PITCHYAW`, or `COMMAND_LONG` containing `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` | `AP_Mount` processes the target; `AP_Mount_MAVLink` generates `GIMBAL_DEVICE_SET_ATTITUDE` |
| FC-managed camera | standard camera commands addressed to the FC | `AP_Camera` dispatches supported operations to `AP_Camera_MAVLinkCamV2` |
| Direct payload control | commands or `PARAM_EXT_*` addressed to the VIO's actual System/Component IDs | `MAVLink_routing::check_and_forward()` routes the message without converting its command semantics |

Direct routing requires a learned route to the VIO and enabled forwarding. **The parameter for
this moved in 4.7**: it is now `MAVn_OPTIONS` bit 1 (`NO_FORWARD`). The old `SERIALx_OPTIONS`
bit 10 is deprecated and raises a prearm failure (`AP_SerialManager.cpp:860`).

### Message lists

**Gimbal Protocol V2** - an integration list, not a claim that the VIO implements every feature:

| Direction | Messages |
|:--:|---|
| GCS -> FC | `GIMBAL_MANAGER_SET_ATTITUDE`, `GIMBAL_MANAGER_SET_PITCHYAW` |
| FC -> GCS | `GIMBAL_MANAGER_INFORMATION`, `GIMBAL_MANAGER_STATUS` |
| FC -> Gimbal | `GIMBAL_DEVICE_SET_ATTITUDE`, `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` |
| Gimbal -> FC/GCS | `GIMBAL_DEVICE_INFORMATION`, `GIMBAL_DEVICE_ATTITUDE_STATUS` |

Command-based manager control also uses `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` and
`MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`.

**Camera** - transport `COMMAND_LONG` / `COMMAND_ACK`; control `MAV_CMD_SET_CAMERA_MODE`,
`SET_CAMERA_ZOOM`, `SET_CAMERA_FOCUS`, `IMAGE_START_CAPTURE`, `IMAGE_STOP_CAPTURE`,
`VIDEO_START_CAPTURE`, `VIDEO_STOP_CAPTURE`; information `CAMERA_INFORMATION`,
`CAMERA_SETTINGS`, `STORAGE_INFORMATION`, `CAMERA_CAPTURE_STATUS`, `VIDEO_STREAM_INFORMATION`;
extended parameters `PARAM_EXT_REQUEST_LIST`, `_REQUEST_READ`, `_VALUE`, `_SET`, `_ACK`.
Gremsy parameter examples: `C_SOURCE`, `C_V_REC`, `C_T_PALETTE`, `C_V_AE`, `GB_MODE`. These
parameters belong to the VIO, not to ArduPilot's parameter database.

### What the FC can actually see, verified against 4.7.1

Of the five camera information messages, ArduPilot originally had an inbound decode for exactly
**one**, `CAMERA_INFORMATION`. `CAMERA_SETTINGS` was added by D10 Stage 3 in 5.1.6.
`STORAGE_INFORMATION`, `CAMERA_CAPTURE_STATUS` and `VIDEO_STREAM_INFORMATION` still have **zero**
inbound decode sites - every occurrence in the tree is ArduPilot *sending* them as a camera to a
GCS. **So those three reach a GCS by direct routing only, never the FC.**

**`PARAM_EXT_*` is not implemented in ArduPilot at all** - no handler for `PARAM_EXT_SET`,
`_VALUE`, `_REQUEST_LIST`, `_REQUEST_READ` or `_ACK` anywhere in the tree. That is what settles
the D10 Stage 2 question for `C_SOURCE` and `C_T_PALETTE`: they do not belong in the FC.

`MAV_CMD_SET_CAMERA_MODE` (530) appears **nowhere** in ArduPilot - neither sent nor handled - so
there is no FC path for it either.

### Integer zoom without an explicit level command, if it is ever needed again

1. **`ZOOM_TYPE_RANGE`** - if it ACKs, map `EO_zoom_times` 1..30 onto 0..100%. One command,
   clean - but see the mode-dependent divisor in `CHANGELOG.md` V5.1.5, which is why this was
   rejected.
2. **`ZOOM_TYPE_FOCAL_LENGTH` (mm) or `ZOOM_TYPE_HORIZONTAL_FOV` (deg)** - also absolute and also
   one command, but both need the lens specification from Gremsy to convert.
3. **Dead-reckoning** - `AP_Q30` already counts steps into `EO_zoom_times`; send timed continuous
   pulses and trust the counter. Needs nothing from the VIO, but it drifts with no feedback.
   **Note the ICD consequence:** reporting that counter in TM2 would put a *commanded* value on
   the wire as though it were measured, partly undoing what D10 Stage 1 fixed.

Capability flags cannot answer whether absolute zoom works: there is **one** flag,
`CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM`, covering all five `CAMERA_ZOOM_TYPE` values. Only a live
command can - send `MAV_CMD_SET_CAMERA_ZOOM` with `param1 = 2` and read the `COMMAND_ACK`. This
needs no firmware change; Mission Planner or pymavlink can send it.

### Also worth knowing from the reference

- **Focus supports only continuous (`p1=1`) and auto (`p1=4`)** - no range - which matches what
  ArduPilot sends for `FocusType::RATE`.
- The gimbal mode flags carry a Gremsy extension **`0x4000`** for mapping mode.
- **`DO_MOUNT_CONTROL` (205) is listed as legacy v1.**
- A laser rangefinder reports through **`DISTANCE_SENSOR` (132)**, for which TC2/TM2 have no
  field.
- **Rate commands expire after 3 s.** 4.7.x added `mnt_target.last_rate_request_ms` and a 3000 ms
  timeout that zeroes all three rate axes (`AP_Mount_Backend.cpp:1192`); 4.6.2 had no such
  timeout, so a rate persisted until replaced. **KGCS must keep sending TC2 while a rate is
  commanded** - confirmed harmless in practice, because KGCS streams, and confirmed to behave the
  same on a Gremsy as on a Viewpro.

### Bench checklist, in order - all disarmed

1. Power up with the VIO connected. Record the `"Mount: %s %s fw:..."` and
   `"Camera: %s %s fw:..."` statustexts. Seeing **both** confirms gimbal and camera are separate
   components.
2. Record the camera's actual **component ID** and whether it heartbeats as `MAV_TYPE_CAMERA`.
3. Record the **`CAMERA_CAP_FLAGS`** bitmask from `CAMERA_INFORMATION`.
4. Send `MAV_CMD_SET_CAMERA_ZOOM` with `ZOOM_TYPE_RANGE` and read the `COMMAND_ACK`.
5. Check whether `CAMERA_SETTINGS` (260) is emitted after a zoom command, and whether its
   `zoomLevel` extension field is populated rather than NaN.
6. List the VIO's `PARAM_EXT` parameters and look for a zoom-level entry. **If one exists, KGCS
   can read it directly** over the routing path it already uses.

## PMU-004 - the driver tick was slower than its nominal rate

| | |
|---|---|
| **Status** | **IMPLEMENTED IN V5.1.26, PENDING HARDWARE VERIFICATION.** The loop is fixed and verified in SITL. Nothing about the target is known |
| **Component** | `libraries/AP_PNU_CAN/AP_PNU_CAN.cpp` `loop()`, and every tick-derived rate above it |
| **Found** | 2026-11-11, while giving the V5.1.25 log records a 10 Hz interval |
| **Fixed** | 2026-11-11, V5.1.26, in SITL only |
| **Related** | `PNUCAN-001`, `PNUSAR-003`, `PMU-001`, `PMU-002`, `PNU-ISSUE D7` |

**What it was.** `AP_PNU_CAN::loop()` slept for a whole period and *then* did its work, so
one tick cost the period **plus** the body. Every rate in the CAN backends is derived from
the tick count, so every one of them was slow by the same factor.

**Measured in SITL before the fix**, `CAN_D1_PNU_OPTS` 1, 3 and 7, 45 s per run:

| | Nominal | Before | After |
|---|---|---|---|
| driver tick, mean | 10.00 ms | 13.09 - 13.53 ms | 10.41 - 10.46 ms |
| PMU RTR / command per id | 10 Hz (ICD) | 7.32 - 7.58 Hz | 9.49 - 9.54 Hz |
| gimbal command per axis | 100 Hz | 69.4 - 71.8 Hz | 90.0 - 90.2 Hz |
| D7 Start interlock | 1.0 s | 1.462 - 1.677 s | 1.161 - 1.347 s |
| D7 Stop interlock | 1.0 s | 1.625 - 1.688 s | 1.273 - 1.300 s |

The step 2c effect is the one that mattered: the KGCS Start burst is 14 messages at 10 Hz
and ends at about 1.4 s, so the interlock was completing **after the burst had already
finished** - the race `D7` explicitly rejects a threshold of 14 for causing. The fix
restores the intended margin rather than creating a new behaviour.

**The fix.** Sleep to the next deadline, so the body's execution time comes out of the next
sleep. **No tick constant was changed**; they became true. An overrun runs its tick
immediately and re-bases the deadline on *now*, never accumulating and never repaying a
stall with a burst - measured, the longest run of consecutive short ticks is 1.

**Why SITL cannot close this.** Two reasons, and both are mechanism, not margin.

1. `delay_microseconds()` on a non-main SITL thread spins on `usleep(1000)` until the
   *simulated* clock passes the target (`AP_HAL_SITL/SITL_State.cpp:118`), so it is
   quantised to the 0.833 ms FDM step and overshoots every request by 1.2 - 5.3 ms at the
   median. The shortest sleep obtainable is 1.67 ms however little is asked for. That is the
   whole of the residual 0.4 ms, and ChibiOS has no such floor.
2. **`select()` behaves differently on the two HALs, and that is what sets the loop rate on
   the aircraft.** `AP_HAL_SITL/CANSocketIface.cpp:262` always enters the semaphore wait and
   always returns `true`. `AP_HAL_ChibiOS/CANFDIface.cpp:954` calls `checkAvailable()` first
   and **returns immediately when a frame is already queued**, blocking only on an empty
   FIFO and returning `false` on timeout. On the target the 2 ms
   `AP_PNU_CAN_SELECT_TIMEOUT_US` is therefore paid only on ticks that find nothing waiting,
   so the pre-fix loop rate was a function of **bus traffic**, which SITL cannot reproduce.
   Whether the fix changes the target by 30 % or by 3 % depends entirely on that, and it is
   unmeasured.

**No pre-fix hardware measurement exists, and none was taken.** It was considered and
declined on 2026-11-11: the FC and PMU are on a completed CAN harness and fitting an
analyser would mean modifying the physical wiring. The decision to proceed on the structural
argument alone was explicit. **Do not infer target timing from any figure above.**

**Required on the bench, before this is closed.**

1. **Actual CAN message rates.** `0x10000100` RTR frames per second, at `PNU_OPTS` 1, 3 and
   7. `tick_rate = RTR_rate_per_id x 10`. The acceptance criterion is that the rate no
   longer varies with `PNU_OPTS` - that was the real defect - not the absolute number.
2. **Step 2c engine Start and Stop timing**, on the real PMU, with PNU present. This is the
   sign-off item. PNU's 2026-10-19 figure (~1 to 1.5 s against an expected ~1.2 s) was
   measured with the defect present, so it does not carry over.
3. **CPU utilisation and stack headroom** for the `pnu_can` thread at the corrected rate.
   `SCHED_DEBUG` plus thread stack high-water. Estimated under 3 % of a core from reading
   the code; unmeasured.
4. **Shared-bus operation at `PNU_OPTS` 7**, which has never been exercised on hardware at
   all. Bus utilisation is estimated to rise from roughly 8.5 % to 10.7 % at 1 Mbit/s.
5. **Whether the real PMU minds 10 Hz.** The ICD specifies it, but the device has run
   against 7.5 - 9.5 Hz in every session so far, so compliance has never actually been
   exercised.

**What V5.1.28 does and does not contribute.** `PMUE` records the step 2c interlock's own
decisions and `PMUC` records the PMU message counts, both from the flight log rather than
from a statustext, so item 2 becomes diagnosable after the fact and item 1 gains a
cross-check: `PMUC`'s `RtrTx` delta divided by 5 is the per-id rate the FC believes it
achieved. **Neither replaces the measurement.** `RtrTx` counts frames handed to the
interface, not frames on the wire, and nothing in either record measures the driver loop
interval directly. Items 1 and 3 still need a CAN analyser and `SCHED_DEBUG` respectively.

## PMU-005 - the ECU (2nd-generation PMU) protocol is specified, not implemented

| | |
|---|---|
| **Status** | OPEN. Specification only - no firmware, nothing blocked |
| **Component** | `PMU CAN Comm Protocol.md` V2.0 (Draft). No code; `AP_PMUCAN` speaks V1.05 only |
| **Source** | `ECU_CAN_Protocol_r2.pdf`, draft for review 2026-11-12, **not stored in this repository** |
| **Related** | `PMU-001`, `PMU-003`, `CHANGELOG.md` "Documents, 2026-11-12" |

**What it is.** The second-generation PMU is an ECU whose software 2.00+ adds three messages -
`ENERGYCFG 0x10000050`, `MODESTS 0x10000150`, `ENERGYSTS 0x10000160` - and defines ECU-specific
behaviour for the five existing commands and the existing status fields. The ICD now covers both
generations, in **two compatibility classes keyed on `VERSTS`**: below 2.0 is legacy messages and
legacy behaviour, 2.0 or higher enables the new set, and a `VERSTS` that cannot be obtained or
decoded counts as below 2.0.

**Nothing in this tree decodes any of it.** `git grep` finds no `0x10000150`, `0x10000160` or
`0x10000050`. The FC is unaffected until someone implements ECU support, and this issue exists so
the decisions behind the ICD are not re-litigated from memory when they do.

**The one thing worth knowing for the FC.** The ECU draft asked for `ENGONOFF` Stop to be repeated
every 500 ms until confirmed; that was **not adopted** - Start and Stop are each sent once when the
requested state changes. `engineonoffstate()` already behaves exactly that way, retrying only a
failed CAN send, and V5.1.28's `PMUC` measured it at `CmdTx`/`RtrTx` = 0.800. So the decision costs
no work, and a future ECU implementation must not add a retransmission loop.

**Three questions for the ECU author**, none of them blocking today:

1. **A reliable completed-stop indication.** With Stop sent once, the Host needs a status field
   that distinguishes a stop that *completed* from one that *started*. The draft itself says
   `PSTS` STOP_SEQUENCE may be too brief to catch and `operation_mode` STOP can pass between
   100 ms polls. Until this is answered, confirming a stop is best-effort.
2. **`AUX1STS.GCMD`, requested or effective.** The scale is settled - 0.1 % per raw count, raw
   100 is 10.0%, confirmed against `BATSTS.GAGE`'s own example. What is open is whether `GCMD`
   reports the setpoint as requested or as clamped into the EMS's internal 54-90% range. Until
   answered, `GCMD` cannot be used to confirm that a `BATCTRL` value was accepted unchanged.
3. **Undefined `MODESTS` codes** - `operation_mode` 2, a zero nibble in either half of byte 0,
   `start_gate` 1-3 and 14, `initial_fail` bits `0x08`/`0x10`/`0x40`/`0x80` - and the fact that
   the frame carries **no layout version**. `MODESTS` is still under development, so its field
   order must be re-validated against a capture before any decoder is frozen.

**Decided, and not to be reopened without a reason:** choke polarity is `0 = open, 1 = close` with
the ECU's `choke_setpoint < 50%` rule informational only; `ENERGYSTS.endurance` is displayed as
encoded with `0xFFFF` shown as Unknown and no Host-side estimate; `PSTS` bit 9 is a maintenance
indication cleared through the ECU's own UART service shell, never over CAN; the bit rate is a bus
property on both generations and never a version marker; `MODESTS` and `initial_fail` are
informational while under development, with the ECU enforcing its own start permissions; and
`ENERGYCFG.fuel` is the current **total measured** tank quantity, `round(litres * 100)`, with no
incremental-addition path.

**Remaining limitation.** No ECU hardware exists on this project and the source draft has not been
verified against a device by anyone, here or at the vendor. The source PDF is not in the repository,
so the ECU sections cannot be re-checked from a clone alone - keep a copy with the project records.

### PMU-005a - the approved implementation plan, Stages 0-5

Approved 2026-11-13. Status by stage is kept in this table; `CHANGELOG.md` carries the
per-stage record once a stage lands.

| Stage | Content | Status |
|---|---|---|
| **0** | Minimal fake ECU in `Tools/pnu/sitl_can/` - VERSTS 2.00, the new ids and DLCs, fixed ICD-example MODESTS/ENERGYSTS payloads, a 32-bit HOURCNT payload. Legacy stays the default. Protocol encode/decode only, no ECU behaviour model | **DONE 2026-11-13** |
| **1** | VERSTS-based protocol identification in `AP_PMUCAN`: `UNKNOWN` / `LEGACY_PMU` / `ECU_V2` | **DONE 2026-11-13** |
| **2** | `AUX2STS` HOURCNT width by generation - UINT24 legacy, UINT32 ECU. One branch, no schema change | **NEXT** |
| **3** | `AP_PMUCAN_ECU.{h,cpp}`: MODESTS at 10 Hz, ENERGYSTS at 1 Hz, on the free scheduling phases, active only in `ECU_V2` | after 2 |
| **4** | MAVLink telemetry for ECU status - **design proposal only, needs approval before implementation** | blocked on approval |
| **5** | MAVLink telecommand for `ENERGYCFG` - **design proposal only, needs approval before implementation** | blocked on approval |

**Architecture: hybrid, decided rather than assumed.** One `AP_PMUCAN` backend with a
latched protocol generation, one version branch in the AUX2STS decode, and a separate
compile-guarded module for the two genuinely new messages. No Legacy/ECU backend split, no
inheritance, no protocol abstraction layer, no code moved for symmetry. The decisive reason
against a backend split: `create_backends()` runs at `AP_PNU_CAN.cpp:349`, **before**
`thread_create()` at line 353, so backends are constructed before any CAN frame can exist
and therefore before VERSTS can be known. Selecting a backend by protocol is impossible;
constructing both would duplicate `_engineonoffmode`, the two step 2c windows and the health
bucket, and both would claim the same five ids in a dispatch that stops at the first claim.

**The whole existing-message delta is one memcpy length.** Every other ECU difference -
ICMD type, GCMD scaling, PSTS bit meanings - is interpretation, and the FC interprets
nothing: every use of `PMU_Status.*` outside the decoder is a logger argument or a MAVLink
argument. The FC is a relay for PMU status, so those differences belong to KGCS.

**Scheduling headroom already exists.** `update()` uses phases 0, 2, 4, 6 and 8 of each
10-tick window (`stride = (100/10)/5 = 2`); phases 1, 3, 5, 7 and 9 are free. The ECU polls
go there, so the five legacy slots and `engineonoffstate()`'s calibrated 10 Hz are untouched.

**No new VERSTS polling mechanism is needed.** Slot 4 of the existing rotation already polls
VERSTS at 10 Hz and must keep doing so - removing it would change the measured RTR rate and
PMUC's meaning. "Identification happens once" therefore means *the decision latches*, not
that the frames stop; late-arriving legacy detection comes free from the existing poll.

**Three decisions taken against the earlier analysis**, recorded so they are not silently
re-litigated:

1. **Missing VERSTS falls back to `ECU_V2`, not to legacy behaviour**, with a GCS message
   once a second until a version arrives, and no interlock or inhibition of engine control.
2. **No five-sample hysteresis** on a generation change, and no forced-generation parameter.
   A single valid-BCD frame can therefore flip the generation; DLC 8 and nibble validation
   are the only filters. Accepted as simple-until-proven-otherwise.
3. **A late legacy VERSTS switches live**, without reboot, disabling ECU polls and leaving
   every step 2c counter and the engine state exactly as they were.

**The three findings, decided 2026-11-13:**

1. **Missing-VERSTS warning - APPROVED and implemented in stage 1.** The 1 Hz message is
   gated on `PMUCAN_Fail_Status == CONNECTED`, the health this backend already keeps, so an
   aircraft flying with no PMU fitted (`PNU-ISSUE D9b`, a supported configuration) and a PMU
   that has stopped answering are both silent. `PMUCAN_Fail`'s own semantics, including the
   difference between never-detected and lost, are untouched.
2. **HOURCNT width during the fallback - WORKAROUND REJECTED, kept as a bench item.** The
   width follows the selected generation, fallback included: `ECU_V2` decodes 32 bits even
   when that generation was assumed rather than reported. No `version_confirmed` flag, no
   special HOURCNT path. The residual risk is a real legacy PMU that never answers VERSTS
   **and** puts a non-zero value in the AUX2STS `RSV` byte, which the V1.05 source leaves
   undefined: engine hours would then read up to 16.7 Ms high. It is bounded by the fact
   that a module needs overhaul long before 100 h, so a valid count stays inside UINT24 and
   the low three bytes still read correctly. **Check the `RSV` byte on the first bench run
   with a real legacy PMU**; reconsider only if it is non-zero.
3. **Existing VERSTS polling - APPROVED.** Slot 4 keeps polling at 10 Hz, the rotation is
   untouched, and no new polling mechanism was added. "Identify once" means the decision
   latches, not that the frames stop; the late-switch path exists because the frames keep
   coming.

**Open items, to settle at the stage that reaches them:**

- **Stage 1, carried forward - the generation is not in the flight log.** It is announced by
  statustext only, so a flight with no GCS attached keeps no record of which generation was
  selected - the gap V5.1.25 closed for the telemetry records. The natural home is stage 3's
  ECU record, and stage 4 asks the same question for MAVLink.
- **Stage 3 - ECU-only RTR polls reach an unidentified device** during the fallback window.
  The ECU draft says it ignores unexpected frames; the legacy PMU's behaviour is unspecified.
  Risk is low and the window is ~100 ms, but it is the first thing to check on the bench.

**Not in scope at any stage:** FC-side interlocks built on MODESTS or `initial_fail`, changes
to TM11/TM12/TM13/TM3/TM1/TC1, changes to PMUC's columns or meanings, changes to the health
leaky bucket's input set, and any periodic retransmission of `ENGONOFF`.


## PNUCAN-001 - the gimbal slew limiter divided by a nominal rate

| | |
|---|---|
| **Status** | **IMPLEMENTED IN V5.1.27, SITL-VERIFIED, PENDING CUBEMARS HARDWARE** |
| **Component** | `libraries/AP_PNUGimbal/AP_PNUGimbal_Backend.cpp`, `advance_commanded()` |
| **Related** | `PMU-004`, `GIMBAL-002`, `RISK - no gimbal hardware has ever been connected` |

The slew limiter computed its per-tick step as `_slew_rate_rads / float(AP_PNU_CAN_LOOP_HZ)`
- a constant, not the elapsed interval. It was correct only if the loop actually ran at that
rate. Before V5.1.26 it did not, so the realised limit was 0.70 - 0.72 of
`CAN_Dn_PNU_SLEW` in SITL; `PMU-004` made it accidentally correct, and it did not make it
right.

**V5.1.27 measures the interval between successive calls.** The first call after entering
`RUN` has nothing to measure from and uses the nominal period; a measured interval is
clamped to `AP_PNUGIMBAL_SLEW_DT_MAX_US`, five nominal periods, so a stalled thread cannot
repay its lateness in one command step. Measured per tick in SITL: the realised rate on a
slew-limited tick went from **25.7 - 180.1 deg/s** to **45.00 deg/s** against a 45 deg/s
parameter, with 0 steps over the limit in 3238 ticks, and a synthetic 1.3 s interval
producing one 2.2500 degree step and no catch-up. Nothing else changed - not the parameter,
the mechanical limits, homing, the state machine, the protocols or the driver loop.

**A defect in the first version of the fix was caught by measuring it.** The guard
`if (max_step_rad > 0.0f)` meant "the limit is enabled" while the step was a constant, but
a measured interval of zero reaches the same branch and means the opposite. SITL hands out
the same `micros()` twice when a tick overruns, and that build let up to **1038 deg/s**
onto the bus. The two cases are now separate tests. Anyone restructuring this function must
keep them separate.

**What the bench session must still establish.** No CubeMars motor has ever been connected
to this firmware, so every figure above is SITL against a fake whose joint is massless and
whose feedback trigger is an assumption. The first bench session must:

1. measure the realised slew rate at the motor and confirm it matches `CAN_Dn_PNU_SLEW`;
2. decide whether 45 deg/s is a sane default for this payload - it is still a provisional
   number, chosen with no measurement of the mount;
3. check that a clamped step taken after a stall does not upset the drive;
4. confirm that the loop interval seen on the target hardware keeps the step where SITL
   says it is, which is the same bench dependency `PMU-004` carries.

## PNUSAR-003 - `AP_PNUSAR_SNAPSHOT_MAX_AGE_MS` was widened to work around `PMU-004`

| | |
|---|---|
| **Status** | OPEN. Re-measure now that the loop is fixed; do not simply revert it |
| **Component** | `libraries/AP_PNUSAR/AP_PNUSAR.h`, `AP_PNUSAR_Nav.cpp` `send_bundle()` |
| **Related** | `PMU-004`, `SAR_ICD_PNU.md` 7.2 |

The ICD-aligned value was 50 ms - one bundle interval. It was raised to 100 ms because SITL
dropped 6 % of bundles at 50 ms, and the recorded reason was that "the driver thread's 10 ms
loop is not that regular". That was `PMU-004`.

The fix removes the **mean** component of the irregularity but not the **tail**: ticks over
50 ms fell from 15 - 19 per 45 s run to 7 - 13, and the worst single tick is still over
100 ms in SITL. Since that tail is SITL thread scheduling rather than the algorithm, the
question cannot be settled here either - it needs the same bench session as `PMU-004`
item 1. Measured at 100 ms after the fix: **100 bundles, 0 drops** over a 10 s observation,
against 99 bundles and 0 drops on V5.1.25.

## GCS-002 - KGCS must send TC50 from a permitted system id

| | |
|---|---|
| **Status** | OPEN - operational, needs PNU-GCS/KGCS work |
| **Component** | `GCS_PNU.cpp`, TC 50050 |
| **Versions** | since 5.1.20 |

TC50 has no `target_system` field, so ICD 3.1 asks the FC to check the sender, and
`sysid_is_gcs()` is that gate - `MAV_GCS_SYSID`, default 255. A refusal says
`SAR: TC50 sysid N not MAV_GCS_SYSID`, which names the parameter to fix.

**TC1 never checked its sender** - an accepted residual of D7 - so nothing before TC50 depended
on that parameter being right. Both endpoints must also agree on **CRC_EXTRA 231 (TC50) and
7 (TM50)**.

The component id is deliberately **not** filtered - the ICD names no permitted value - and is
logged instead.

## CAM-001 - the Viewpro tracking freeze can strand the operator

| | |
|---|---|
| **Status** | OPEN, untested. From the Viewpro bench notes, 2026-09-29 |
| **Component** | `AP_Mount_Viewpro.cpp:290`, `update()` |

`AP_Mount_Viewpro::update()` returns early while the tracking status is `SEARCHING` **or**
`TRACKING`, sending the gimbal no angle, rate or hold target at all. **Tracking status comes from
the camera, not from KGCS** - `_last_tracking_status` is written in exactly one place, parsing
bits 3-4 of the gimbal's `T1_F1_B1_D1` telemetry frame. So that freeze can begin **with no KGCS
action** - the camera re-acquiring after losing a target, for example - and the operator loses
gimbal control with only a statustext to explain it. `LOST` is *not* in the freeze condition, so
control returns on a lost target.

**Required verification.** Test a target-lost / re-acquire cycle and confirm the operator is
never stranded without gimbal control.

## CAM-002 - the Viewpro zoom maxima are hard-coded and unverified

| | |
|---|---|
| **Status** | OPEN |
| **Component** | `AP_Q30.h:89-90` |

`_Max_zoom_EO = 30` and `_Max_zoom_IR = 4` clamp the absolute-zoom command and have never been
checked against the new camera's real maxima. Note that `_Max_zoom_EO = 30` matches the Gremsy
VIO's super-resolution maximum exactly, **which suggests that constant never described the
Viewpro at all**.

---

# 2. Pending Engineering Decisions

## PMU-001 - two step 2c decisions awaiting PNU's sign-off

Both are deliberate deviations recorded at the time, not oversights. See `CHANGELOG.md` V5.1.18.

1. **The Stop window is measured from the START PRESS**, not from the moment permission is
   checked. The guard asks "did a Stop precede this Start by less than 10 s". The counter
   threshold of 10 is a tick timer at 10 Hz, so it consumes about 1.0 s of the window on its own,
   and testing at the completion instant charges the operator for the FC's own interlock delay on
   top of KGCS's ~5 s button lockout. `_start_pair_first_ms` records when this attempt's first
   valid alternation arrived, and the staleness guard bounds how stale that can be. **It does not
   weaken the guard**: a Start queued during an outage arrives late, so its first alternation is
   late too. The design note fixed the other end of the window - "measure from the LAST Stop
   message seen" - and said nothing about this end.
2. **The staleness guard is symmetric, so a STOP burst cut off mid-way no longer completes
   either.** `engineonoffstate()` returns early while TC1 is stale, which zeroes both counters.
   An operator whose link dies during a Stop press must press Stop again. On the ground that is
   an annoyance; **in the air an engine that declines to stop on a comms glitch is the safer of
   the two failures**, which is why this is recorded rather than special-cased. **If PNU wants
   Stop exempted, say so** - it is one condition, but it reopens exactly the mechanism step 2c
   exists to close.

## PMU-003 - five write-only counters: surface or delete

| | |
|---|---|
| **Status** | **CLOSED IN V5.1.28.** Surfaced as `PMUC`, not deleted |

D6 residual. `_handleFrame_cnt`, `_rtr_tx_cnt`, `_cmd_tx_cnt`, `_rtr_tx_err` and `_cmd_tx_err`
were incremented but **never read anywhere in the tree** - they cost RAM and told nobody
anything. The choice was to surface them or delete them, and they are surfaced: `_rtr_tx_err`
and `_cmd_tx_err` are what you want when chasing an intermittent PMU link, and the D7
send-result fix already depends on `_cmd_tx_err` for diagnosis.

V5.1.28 writes all five, plus `_short_frame_cnt`, as a cumulative `PMUC` record at 1 Hz from
`AP_PMUCAN::log_counters()`, beside the `log_status()` that V5.1.25 created.
`_short_frame_cnt` had been left out of this issue because it already had a reader - a
rate-limited statustext - but a statustext nobody is attached to receive is not a record.

**Two of the six columns have never been seen move.** `RtrErr` and `CmdErr` count CAN send
failures, and `AP_HAL_SITL`'s `CANIface::send()` returns the base class's unconditional `1`
(`AP_HAL/CANIface.cpp:103`), so **no send can fail in SITL**. They were exercised by forcing
`ENGONOFF` to fail in a temporary build. On ChibiOS `send()` returns 0 for a full queue and
negative for an interface error, so both are reachable on the target; the bench session is
the first chance to see either one non-zero, and a non-zero `RtrErr` there would be the
first real evidence of a bus problem rather than a PMU problem.

The same keep-or-delete question still applies to `_current_zoom_EO` / `_current_zoom_IR` in
`AP_Q30`, added by D10 Stage 1. If they are ever read, note they are `uint8_t` holding a
`float` zoom.

## PNU-ISSUE D1 residual - is the IR prelude needed at all?

The deferral mechanism was hardware-verified and the three races it had were fixed. What was
never tested is whether the prelude is needed at all: that requires **removing it and retesting
on hardware**.

## PNU-ISSUE D12 residual - confirm the two operator thresholds

Confirm that 10 m / 15 m are the intended operator thresholds
(`PNU_OA_ALERT_DISTANCE_CM` / `PNU_OA_WARN_DISTANCE_CM`). They are hard-coded and unrelated to
`AVOID_MARGIN`. The ICD decision that accompanies them - no `Object_Avoidance_Status = 3` for
"sensor unhealthy" - is **provisional and PNU will revisit it**.

## Should KGCS time out TM2?

**TM2 reports a dead gimbal as perfectly level.** If `get_attitude_euler()` fails,
`send_message_gcs_flcc_cam_status()` zeroes all angles, so a gimbal that has stopped reporting is
indistinguishable from one pointing straight ahead - the same pattern as D12. Decide whether KGCS
should time out TM2.

## The NMEA output rewrite

Is any serial port on the aircraft set to `SERIALn_PROTOCOL = 20` (NMEAOutput)? If not, the
V5.0.8 NMEA rewrite is dead code and the question closes. If it is, the rewrite must be
re-specified as separate PNU work - see `CHANGELOG.md` Appendix A for why it was not applied.

## `0x503` and `0x523` stay reserved

Lumir expects to define them soon, which is why they are named rather than left as holes.
**Interfaces and names only; no transmission, no invented payload, no parsing** - ICD sections 5
and 13 both say so explicitly. The eight bytes stay opaque. `Observation_Mode` is retained whole
and never interpreted for the same reason: its CAN encoding depends on `0x503`.

## Whether to suppress MAVLink msg 285

**Recommendation on record: do not suppress it.** PNU-GCS handles it, the payload's copies arrive
regardless, and the only real decision is which source of gimbal attitude PNU-GCS reads - msg 285
(standard, and arriving anyway) or TM2 (the KGCS convention). The per-backend suppression stays
as it is for Viewpro/KGCS, which cannot send the suppression command at all. The mechanics are in
`README.md`.

---

# 3. Known Risks and Unverified Assumptions

## RISK - no gimbal hardware has ever been connected

**No CubeMars motor has ever turned.** The protocol backend, the `AP_Mount` backend, the
lifecycle state machine and the Appendix 1 homing sequence are all written, and every one of
them has only ever run against the fake gimbal in SITL. The fake found a blocking firmware
defect on its first run (`CHANGELOG.md` V5.1.23), which is evidence that SITL is useful and
**not** evidence that the driver works.

## RISK - a PMAX mismatch does not fail, it silently rescales everything

The five encoding limits (`PNU_PMAX`/`VMAX`/`KPMAX`/`KDMAX`/`TMAX`) **must** match what the
vendor PC utility wrote to the motors. A host/device mismatch produces no error - it rescales
every command and every reading. This is the worst failure mode the CubeMars protocol has.

**Travel validation during homing is the only runtime check in the whole design that can detect
it**, because it is the only place an encoder-derived number is compared against a known physical
quantity:

| Measured travel | What it means |
|---|---|
| about 60 deg | normal |
| **much less**, e.g. 45 deg | **something is obstructing** - the SAR fouling a skid strut, a snagged cable, a bolt, ice. The case the limits exist for, caught on the ground |
| **much more**, e.g. 90 deg | the endpoint was not an endpoint - stall detection fired early on friction or a cable drag point |
| **about 2x or 0.5x**, e.g. 120 or 30 deg | **`PNU_PMAX` does not match the range configured in the motor** |
| 58 or 62 deg | normal variation - stop compliance under a heavy payload, stall detection firing just before true contact, encoder quantisation |

Set the range tolerance too tight and every boot aborts; too loose and a partially obstructed
axis passes, and the aircraft flies with a reduced range the mechanical clamp does not know about.

## ASSUMPTION - the gimbal zero does not persist across a power cycle

**Assume it never does.** The drives use a relative incremental encoder and the manual does not
state persistence either way, so the safe assumption is the pessimistic one. Three consequences:

- **HOMING runs on every boot**, so it is on the critical path to being flight-ready and arming
  must be blocked until it completes - `healthy()` returns false in every state but RUN.
- **At power-up the encoder reads 0 wherever the mount happens to be sitting**, so the boot
  feedback angle is meaningless as an absolute angle.
- Therefore **the mechanical-limit clamp protects nothing until homing completes**.

This is **a one-hour bench test** - home, power cycle, read the angle - and it changes whether
homing is a startup step or a commanded action. It has not been done.

## EXPECTED BEHAVIOUR, not a defect - stabilisation will saturate in normal flight

With roll and pitch operating at +/-25 degrees, a vehicle leaning 25 degrees consumes the entire
range, and a multicopter in forward flight leans 15-25 degrees routinely. Yaw is tighter still:
an earth-frame yaw lock breaks as soon as the vehicle heading moves more than 15 degrees from
where the lock was set. **Clipping at the limit is correct behaviour.** Recorded here so nobody
reports it as a bug. The mount should be able to say that it is on a limit rather than silently
appearing to stop tracking.

## RISK - the shared bus has never been exercised on hardware

Every PMU bench run had `CAN_Dn_PNU_OPTS = 1`. The id-based health rewrite of 5.1.15 - the thing
step 2 exists for - has only ever run in SITL. The 2026-10-30 and 2026-11-03 SITL runs with
`OPTS = 5` and `OPTS = 7` are real executions of the demultiplexing logic and are **not**
hardware.

**`vcan` performs no arbitration.** None of the SITL shared-bus evidence says anything about
contention on a real 1 Mbit/s wire. The calculated load is about 11%:

| Source | Frames/s | Share of 1 Mbit/s |
|---|---:|---:|
| SAR navigation, 9 frames at 10 Hz | 90 | ~1.2% |
| PMU, 5 RTR + 5 responses + 5 commands at 10 Hz | ~150 | ~2.2% |
| Gimbal, 3 axes command and feedback at 100 Hz | ~600 | ~7.8% |

The nine-frame bundle takes roughly 1.2 ms of a 100 ms period, so the ICD's "do not overlap or
interleave navigation bundles" is comfortable. **What does need care is that all nine frames are
enqueued in one driver tick**: a full TX queue would publish a partial bundle, which the receiver
must discard. That is counted, never papered over.

## PMU-002 - LTE message bunching, which no window size fixes

The FC samples TC1 at 50 Hz in `transmit_slot()` - one `prv = cmd; cmd = latest` per sample,
however many messages arrived. D7 records the assumption behind that: *"TC1 at <=11 Hz against
TXspin's 50 Hz sampling means no alternation is missed at the sequence check."* **That holds for
a clean link and breaks under bunching.** If a cellular stall delivers four messages inside one
20 ms sample, the FC sees only the last, so `2,3,2,3` can read as `3` then `3` - not an
alternation - and the counter **resets to zero**.

Symptom: Start occasionally doing nothing on a poor link and needing a second press. It fails
safe, so this is a usability sensitivity rather than a hazard. **Measured 2026-10-19 at ~1 to
1.5 s, consistent with the expected ~1.2 s, so bunching is not severe today.** If Start ever
becomes flaky over LTE, look here first - and the fix is on the **ingest** side, by latching
alternations as they arrive in `handle_gcs_flcc_pmu_ctrl()` rather than sampling at 50 Hz.
Neither of the step 2c windows helps.

## ASSUMPTION - the step 2c numbers behave as designed, not that they are right

500 ms and 10 s are PNU's numbers, derived from the KGCS operating procedure. Both are bounded
from opposite directions and neither has much room to move:

| threshold | counter reaches before the reset | protects? |
|---|---|---|
| 300 ms | 3 of 10 | yes, large margin |
| **500 ms** | **5 of 10** | **yes - chosen** |
| 800 ms | 8 of 10 | marginal |
| >=1000 ms | 10, so the start fires first | **no, useless** |

500 ms must exceed the genuine inter-message gap and land before the counter reaches 10; it was
raised from 300 ms by PNU 2026-10-19 to absorb LTE server and network delay. **Do not go past
~700 ms.** 10 s must exceed KGCS's ~5 s button lockout (it disables **both** Start and Stop for
about 5 s after either is pressed, so a 5 s FC window would have expired before the button became
available again) and stay short enough that a comms outage spanning more than one button cycle
invalidates the sequence.

**A real PMU, a real engine, a real KGCS and the LTE link are all unsimulated.** The injector
sends what the KGCS protocol specifies, not what KGCS actually emits.

## ASSUMPTION - `CAMERA_SETTINGS.zoomLevel` means zoom times

Accepted risk, recorded at the code site. A spec-compliant camera reporting a true percentage
would be misreported as times. TM2 is a PNU message on PNU aircraft with known payloads. Revisit
if a third camera type is ever fitted. The reference also leaves it **unknown** whether the field
holds a percentage per the MAVLink spec or times as a vendor deviation; moot if KGCS reads the
`EO_ZOOM` telemetry index instead.

## UNVERIFIED - the Gremsy `CAMERA_CAP_FLAGS` value

The PayloadSDK reference gives the expected value as `0x1C7` = 455 and **says explicitly that it
is unverified**. It should be read off the device - one line in Mission Planner's MAVLink
Inspector, the `flags` field of `CAMERA_INFORMATION` from the camera component.

| Bit | Flag | Gates, in `AP_Camera_MAVLinkCamV2` |
|:--:|---|---|
| 1 | `CAPTURE_VIDEO` | `record_video()` - passes |
| 2 | `CAPTURE_IMAGE` | `trigger_pic()` - passes |
| 4 | `HAS_MODES` | `MAV_CMD_SET_CAMERA_MODE`, which ArduPilot never sends - unreachable from the FC either way |
| 64 | `HAS_BASIC_ZOOM` | `set_zoom()` - passes, and independently proven on the bench |
| 128 | `HAS_BASIC_FOCUS` | `set_focus()` - passes |
| 256 | `HAS_VIDEO_STREAM` | not used by ArduPilot |

The tracking bits (512 / 1024) are expected to be clear, consistent with VIO tracking living on
`USER_4` to the payload app - and moot regardless, since `AP_Camera_MAVLinkCamV2` implements no
`set_tracking()`.

**With Viewpro the cap flags are pure advertising** - `AP_Mount_Viewpro::set_zoom()` checks only
`_initialised` and never reads the flags it advertises. **With Gremsy they are a hard gate**:
`set_zoom`/`set_focus`/`trigger_pic`/`record_video` each return false **before transmitting** if
their flag is clear.

## RISK - the Gremsy camera component ID trap

`AP_Camera_MAVLinkCamV2::find_camera()` uses
`compid = MIN(MAV_COMP_ID_CAMERA + _instance, MAV_COMP_ID_CAMERA6)`:

| `AP_Camera` instance | Parameter | compid it looks for |
|:--:|:--:|:--:|
| 0 | `CAM1_TYPE=6` | **100** |
| 1 | `CAM2_TYPE=6` | **101** |

**The VIO ships on 101.** Resolved by setting the VIO's camera component to 100, which its
configuration allows and which needs no firmware change. The alternative - driving it as
`CAM2_TYPE=6` - **would not work without code changes**, because all 17 of `AP_Q30`'s camera
calls hardcode instance 0. If a future payload has a fixed component ID, that is the change
required: an instance selector in `AP_Q30`.

Two further conditions, both easy to miss: the camera must emit a **HEARTBEAT with
`MAV_TYPE_CAMERA`** (the only thing that populates `mavtype` in the routing table), and the
search **starts 10 s after boot, runs for 60 s, and stops once armed** - so it has to work on the
bench, disarmed.

A malformed `model_name` is harmless - it is used only in a statustext and in
`AP_Mount_MAVLink::natively_supported_mount_target_types()` for the AVTA/CM41 special case, and a
failed `strncmp` just takes the normal conversion path. **A missing `CAMERA_INFORMATION` is not
harmless**, because then `_cam_info.flags` stays 0 and every camera call fails. The two failure
modes look similar at the GCS and must not be confused.

## GCS-003 - the Gremsy firmware sends yaw limits in degrees, not radians

`MNT1_YAW_MAX` took a default of **18334**, which is `degrees(320)`. Msg 283 specifies
`yaw_min`/`yaw_max` as `units="rad"`, so ArduPilot's conversion is correct and **the VIO is
sending 320 - a sensible degrees yaw range - in a radians field.** The bug is confined to the yaw
fields; roll and pitch arrive correctly. Worth reporting to Gremsy against gimbal firmware
7.8.7.0.

| Use of `yaw_angle_max` | Effect at 18334 |
|---|---|
| RC targeting (`AP_Mount_Backend.cpp:138`) | **broken** - a *centred* stick commands `0.5 x (18334 + 180) - 180 = 9077 deg`. PNU does not use RC gimbal control |
| `GIMBAL_MANAGER_INFORMATION` | **wrong data to a standard GCS** - advertises `radians(18334)` = 320 rad as the yaw limit |
| `set_angle_target()` | harmless - yaw is wrapped to <= 180 *before* the constraint |

**Workaround: set the parameters explicitly.** `set_default()` only applies `if (!configured())`,
so any saved value is permanent. Verified on the bench - `MNT1_YAW_MIN/MAX` = -180/180 works, as
do `MNT1_ROLL_MIN/MAX` = -15/15 and `MNT1_PITCH_MIN/MAX` = -90/15 for this gimbal.

**Residual trap.** `set_default()` calls `add_default()` **before** the `configured()` check, and
`AP_PARAM_DEFAULTS_ENABLED` is on for this build. So 18334 remains registered as the parameter's
*default* even while the live value is 180. Mission Planner will show `MNT1_YAW_MAX` as
modified-from-default, parameter diff tooling will flag the correct value as the anomaly, and
**a "reset to defaults" would restore 18334.**

## ACCEPTED - this branch builds ArduCopter only

ArduPlane, Rover and Sub have been unbuildable since `ed055d2f24` (5.0.10) because two shared
libraries reference PNU-only code that only `ArduCopter/wscript` lists:

| Library | Undefined symbols |
|---|---|
| `AP_CANManager.cpp` | `AP_PMUCAN::AP_PMUCAN()` - the PMUCAN case in the driver factory |
| `GCS_MAVLink/GCS_PNU.cpp` | `AP::Q30()`, `AP_Q30::*`, `CAM_ATTITUDE_STATUS`, `debug_cam_*` |

5.1.11 added `AP_PNU_CAN::var_info` to the same list. This is **by construction, not an
accident** - but it means **a vehicle gate on PNU code in a shared library protects nothing, so
do not add one.** Fixing it would mean an `AP_PMUCAN_ENABLED`-style flag on both libraries, the
same shape as D13's `AP_Q30_ENABLED` - worth doing only if a non-Copter vehicle is ever needed
from this branch. Deliberately **not** raised as a PNU-ISSUE.

## ACCEPTED - gimbal commands are last-writer-wins with two GCSs connected

Inherent to MAVLink, not a defect. MAVLink's own arbitration is
`MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` (primary/secondary control); adopt it only if simultaneous
connection is a real operating case rather than a bench convenience.

## ACCEPTED - with `PNU_OPTS` bit 0 clear, KGCS is told the PMU is healthy

With no PMU backend, nothing ever writes `PMU_Ctrl_Echo.PMUCAN_Fail`, so it stays at its
static-init value of **0**, `failsafe_pmucan_check()` takes the `== 0` branch and returns, and
`pmucan_checks()` likewise returns true. So **no failsafe and no prearm warning**, which is
correct for an aircraft deliberately flown without a PMU - but KGCS is told
`PMUCAN_Fail = 0`, i.e. a healthy PMU that is not fitted.

That is exactly today's behaviour when `CAN_Dn_PROTOCOL` is not 15, so it is not a regression -
but **one forgotten bit now produces it where a whole wrong protocol number used to be needed.**
The mitigation is the startup report naming the enabled backends once per boot.

That safety rests on `PMUCAN_Fail` only ever becoming 1 from inside the PMU backend's own code.
True by construction, and commented at the site: **raising the global's static initialiser to 2
"for honesty" would fire `"PMU not healthy (state 2)"` on every prearm of a PMU-less aircraft** -
the permanent-warning-crowds-out-real-faults problem 5.1.8 removed.

## UNVERIFIED - calling `AP::ahrs()` from the CAN thread

A split-rate option was considered for gimbal stabilisation: `AP_Mount` hands over an
*earth-frame* target at 50 Hz and the `AP_PNU_CAN` thread subtracts AHRS at 100 Hz just before
transmitting, giving fresh compensation without raising the mount task rate. **Nothing has
checked that calling `AP::ahrs()` from the CAN thread is safe.** Verify before relying on it. The
fallback costs nothing and is what the code does today: do the subtraction in the mount task and
let the CAN thread transmit the latched body-frame value.

## UNVERIFIED - protocol enum values may collide with upstream

`PMUCAN/PNU_CAN = 15`, the deprecated `PNU_CAN_LEGACY = 16` and the retired 17 could collide with
future upstream `AP_CAN::Protocol` numbers. **Watch this on the next ArduPilot bump.** Adding
enum values also breaks exhaustive switches under `-Werror=switch`, which is how `AP_Arming.cpp`
came to list them.

## The CubeMars manual's own errors, pinned deliberately

Five protocol facts worth not rediscovering, all from
`libraries/PNUGimbal_CAN_Protocol_Type_CubeMars.md`:

1. **The encoding limits are device configuration, not constants.** See the PMAX risk above.
2. **MIT fields are unsigned offset integers**, not two's complement and not float32. Packed zero
   sits near the middle of the range, so an all-zero MIT payload is *not* a zero command.
3. **The manual's transmit function uses `1 << bits` and its receive function
   `(1 << bits) - 1`.** This driver uses the receive form in both directions, so encode and
   decode are inverses and the maximum saturates instead of overflowing into the next field.
   **One fake-gimbal test deliberately asserts the manual's wrong values** (4.74 and -4.84 where
   it claims +/-6), so that a future session cannot "fix" the codec until those rows pass and
   thereby break agreement with everything else in the document.
4. **Feedback carries only four bits of motor identity** - D0's upper nibble is the status code.
   Motor IDs are restricted to 1..15 and required distinct, which makes that nibble a complete
   demultiplexer. The prose claims eight bits; its own table does not.
5. **Position feedback must be read through an unsigned intermediate.** The manual's `int16_t`
   example misrepresents every value at or above 0x8000, which is half the travel.

## SITL facts that are not aircraft facts

- **SITL's simulated u-blox reports `GPS.Status = 6` (RTK Fixed).** That is what made the
  *permitted* Start path testable rather than only the refusals - and it is also why the first
  attempt at that measurement looked like a bug.
- **Copter's default stream rates are all zero.** A passive MAVLink listener receives HEARTBEAT
  and nothing else.
- **SERIAL1 and SERIAL2 (TCP 5762, 5763) do not exist until something connects to SERIAL0**,
  because SERIAL0 is `tcp:0:wait`. `--serial0 tcp:0` removes the dependency.
- **The dataflash logger opens about six seconds after boot**, so earlier statustexts appear in
  neither the log nor a late-connecting GCS.
- `SIM_CAN_TYPE1 = 2` selects SocketCAN and the interface name is hard-coded as `vcan<instance>`
  (`CAN_SocketCAN.cpp:32`); the default 1 is multicast UDP. SITL is configured here with
  `HAL_NUM_CAN_IFACES 2`, so the CAN driver really does run in simulation.

---

# 4. Resolved Issues

Resolved issues keep their engineering history: the original defect, the root cause, the solution,
the verification evidence and any accepted residual. **A resolved issue with a residual is listed
in section 2 as well.**

## PNU-ISSUE D1 - IR palette deferral races

| | |
|---|---|
| **Status** | RESOLVED 2026-09-30. **Residual open** - see section 2 |
| **Component** | `AP_Mount_Viewpro.{h,cpp}` |

**Original.** For colour codes >= `IR_COLOR_1` the driver sends an `IR_RAINBOW` prelude and defers
the colour itself to a later `update()` tick, because the gimbal MCU services at most one C1
command per its own scheduler cycle. The mechanism was hardware-verified; three races remained.

**Why it mattered.** A bench test confirmed **KGCS streams TC2**, so the races were reachable.

**Applied.** (a) the display is **captured at defer time** into `_palette_pending_sensor`, so the
colour reaches the sensor it was meant for even if gimbal telemetry changes `_image_sensor`
inside the window; (b) the pending colour is **cleared only on a successful send**, so a
momentarily full UART retries next tick instead of dropping the command; (c) a **400 ms
deadline** bounds those retries so a stale colour is never sent late; (d) a duplicate request for
the palette already in flight no longer restarts the sequence - relevant precisely because KGCS
streams.

**Also fixed while here.** `_palette_pending_color` had **no initialiser** in a heap-allocated
class with an inherited constructor. All three members now carry initialisers, with a comment
saying they must keep them.

**Correction, 2026-10-13.** This note, and the matching one beside `AP_Mount_Viewpro::_zoom_times`
in D10 Stage 1, both said such a member is *indeterminate* at boot. **That is wrong in this
codebase**: mount and camera backends are allocated with `NEW_NOTHROW`, and
`AP_Common/c++.cpp:45` implements `operator new(nothrow)` as **`calloc`**, so the whole object
arrives zeroed. The initialisers are therefore **defensive hygiene, not bug fixes** - they remove
a dependency on allocator behaviour that nothing states as a contract, and they keep working if
an object is ever placed on the stack or allocated another way. Keep them; just do not go hunting
for the boot-time garbage the earlier wording implied.

**Follow-up, decided and deliberately NOT done.** `AP_Q30::IR_operation()` fires
`IR_Color_Change()` on every TC2 whose `Tracking_CMD` names a palette, unlike zoom which guards
on `prev_EO_zoom_cmd` / `prev_IR_zoom_cmd`. With KGCS streaming, a **held** palette value
re-requests it ~10x a second. The D1 fix absorbs the harm, so this is traffic, not a defect.
**Adding change detection was considered and rejected**: if KGCS *holds* the value, change
detection would suppress a re-press of the **same** palette, which is exactly the recovery action
D1's original note assumed the operator would take. Before revisiting, settle the fact: **does
KGCS hold `Tracking_CMD` at the palette value, or return it to 0 after the button press?**
`TC_C.TRAK` shows it directly - a held value is a constant run, a momentary one a single sample.
Palette changes work today, which suggests momentary.

## PNU-ISSUE D2 - PNU-KAL messages need MAVLink2

| | |
|---|---|
| **Status** | RESOLVED 2026-09-29 - documented as a setup constraint, no code change |
| **Component** | `Forced Submodule File/common.xml` |

All 7 PNU-KAL ICD messages are id >= 50001, so a link set to `SERIALn_PROTOCOL=1` silently
carries no PNU-KAL telemetry or commands.

**A runtime warning was designed and rejected as unnecessary**: a MAVLink1 link means *no* PMU
messages at all, which is self-evident at the GCS; the port parameter is checked regardless; and
PNU's offline preflight procedure already fixes the protocol.

**Verified mechanism, so nobody need re-investigate:** the failure is a clean drop, not
corruption - `mavlink_helpers.h:339` refuses msgid > 255 and counts a parse error, so a
misconfigured link also shows a rising parse-error count. Such a channel never self-upgrades
either, since the auto-upgrade in `packetReceived()` (`GCS_Common.cpp:1904`) requires the
configured protocol to already be MAVLink2.

## PNU-ISSUE D3 - the mavlink submodule must stay pristine

| | |
|---|---|
| **Status** | RESOLVED 2026-09-29 |
| **Component** | `Forced Submodule File/`, `Tools/pnu/stage_mavlink_defs.sh`, `wscript` |

**The problem.** The parent repo cannot track submodule file contents, so any edit inside
`modules/mavlink` is invisible to git and is silently reverted by `git submodule update`, a fresh
clone or a submodule bump - and the build then fails with ~21 unknown-type errors.

**How it used to be.** The 7 PNU messages were appended to the submodule's `common.xml`, which
meant keeping a full **7579-line copy** of `common.xml` in `Forced Submodule File/` and
overwriting the upstream file with it - so every ArduPilot bump would have **silently reverted
upstream's `common.xml` changes**. `cubepilot.xml` also had 5 ids renumbered 50001-50005 ->
70001-70005 to dodge an id collision.

**How it works now.** Nothing inside the submodule is touched. The definitions are staged beside
it and the build reads the staged copy.

| Piece | Tracked? | Role |
|---|:--:|---|
| `Forced Submodule File/pnu_kal.xml` | yes | the 9 message definitions (7 PNU-KAL, plus the SAR's TC50/TM50 from 5.1.20) |
| `Tools/pnu/stage_mavlink_defs.sh` | yes | copies the 19 upstream XMLs + `pnu_kal.xml` into the staging dir, drops `cubepilot.xml`, stamps the source commit |
| `Tools/pnu/mavlink_defs/` | **no - gitignored** | the staged copy (~944 KB), regenerated, never committed |
| `wscript` (1 line) | yes | builds from `Tools/pnu/mavlink_defs/all.xml` |
| `modules/mavlink` | - | **pristine** |

`cubepilot.xml` is simply not staged rather than renumbered: its ids collide with the PNU-KAL
ICD, nothing in ArduPilot references `CUBEPILOT_*` or `HERELINK_*`, and PNU uses no Herelink or
CubePilot raw RC. It must be dropped from **both** `all.xml` and `ardupilotmega.xml`, which
includes it directly as well.

**Three failure paths, each verified by reproducing it:**

| Failure | What you get |
|---|---|
| nothing staged (fresh clone) | waf stops: `Tools/pnu/mavlink_defs/all.xml is missing - run Tools/pnu/stage_mavlink_defs.sh` |
| stale stage | `--check` exits 1, printing both commits |
| staged but PNU messages absent | compile stops at `GCS.h`: `PNU-KAL MAVLink dialect missing` |

That last guard has to live in `GCS.h`, not `AP_Q30.h` - `GCS.h` declares the PNU message types
itself and is compiled first, so a guard placed later never fires. The waf-level check is not
optional either: without it, a missing staging dir fails with
`TypeError: 'NoneType' object is not iterable` out of waf's node resolution, which says nothing
about the cause.

**Alternatives tested and rejected.** Keeping the definitions in the submodule and scripting the
re-apply worked but still needed 3 lines inside the submodule. Pointing `wscript` at the
submodule's `all.xml` plus a PNU root file also worked for mavgen, but `ardupilotmega.xml`
includes `cubepilot.xml` itself, so the id collision returns and cannot be avoided without either
editing the submodule or renumbering the three colliding PNU ids (50001, 50002, 50004), which is
an ICD change requiring KGCS work.

**Residual risk, accepted and mitigated.** A stale stage still contains the PNU messages, so it
builds happily against *last month's* upstream definitions with nothing to show for it. The stage
records the submodule commit in `.source-commit` and `--check` compares it against HEAD. **Worth
wiring into CI.**

## PNU-ISSUE D4 - two hand-maintained copies of the version

| | |
|---|---|
| **Status** | RESOLVED (step 4). **A third copy was found open in 2026-11-03 - see `GCS-001`** |
| **Component** | `GCS.h`, `version.h`, `Copter.cpp` |

`OFP_VER_MAIN/SUB/REV` in `GCS.h` and `FW_MAJOR/MINOR/PATCH` in `version.h` had nothing enforcing
agreement. Resolved with `static_assert`s in `Copter.cpp` - the only translation unit that sees
both, since `AP_PMUCAN.cpp` cannot include the vehicle `version.h`.

## PNU-ISSUE D5 - the GCS_Common.cpp merge cost

| | |
|---|---|
| **Status** | RESOLVED 2026-09-30 |
| **Component** | `GCS_Common.cpp` -> `libraries/GCS_MAVLink/GCS_PNU.cpp` |

The 356-line block appended at `GCS_Common.cpp`'s EOF was the main recurring merge cost on every
ArduPilot bump, and **the only issue here whose cost grew with time**.

**Result.** `GCS_Common.cpp` vs pristine 4.7.1 went from **429 added lines in 7 hunks to 45 in
3** - better than the 72 estimated, because the two includes and the extern/macro block moved out
too. The three survivors are unavoidable: they insert into upstream's
`mavlink_id_to_ap_message_id()` map, `handle_message()` and `try_send_message()` switches. Moved
code is **byte-identical**. Costs **+136 bytes** of flash, since the handlers are no longer in the
same translation unit as their callers.

## PNU-ISSUE D6 - no DLC validation in the PMU decoder

| | |
|---|---|
| **Status** | RESOLVED 2026-09-30. **Residual open** - the write-only counters, see `PMU-003` |
| **Component** | `AP_PMUCAN.cpp` |

**Original.** `handleFrame()` performed no DLC validation - every case `memcpy`d from fixed
offsets up to `data[7]` whatever `can_rxframe.dlc` said. No out-of-bounds read (`data[]` is a
fixed 8 bytes), but a short or malformed PMU frame was parsed silently and yielded stale values
for battery current, RPM, fuel quantity and so on.

**Resolution.** The ICD confirms **DLC 8 for all five status messages** (BATSTS, ENGSTS, AUX1STS,
AUX2STS, VERSTS), which also confirms every field the code reads is present. **TX confirmed too**:
4 for all five commands, matching `PMUCAN_CMD_DLC`; all five route through the single
`pmucan_cmd()` path. RTR is N/A in the ICD, so `pmucan_rtr()`'s 8 is unconstrained. No TX change
needed. Frames shorter than `PMUCAN_STS_DLC` are now rejected before parsing, counted in
`_short_frame_cnt`, and reported as `PMUCAN: short frame 0x<id> dlc <n> (<count> dropped)`, rate
limited to one message per 10 s with the first always reported. Unknown ids now return early
rather than falling through the switch.

**D6 is the project's standing rule**, and the later work keeps citing it: no write-only
counters, no silent failures, no rejection the operator cannot see.

## PNU-ISSUE D7 - the engine ON/OFF interlock counters

| | |
|---|---|
| **Status** | RESOLVED 2026-09-30 - one defect fixed, the rest analysed and accepted. **Tightened by step 2c in 5.1.18**; residuals in section 2 |
| **Component** | `AP_PMUCAN.cpp` |

**Original.** The interlock's counters are not a count of operator alternations: the pair they
inspect only changes when a new TC1 arrives, but `engineonoffstate()` runs at ~10 Hz regardless,
so a frozen valid pair keeps incrementing. **They are a ~100 ms tick timer** - threshold 10 means
*~1.0 s after the first valid alternating pair*.

**Fixed.** The state no longer advances on a failed CAN send. It previously set
`_engineonoffmode` unconditionally, so a send failure left the FC believing the engine was running
while the PMU had never been told - and KGCS blocks Start from that point, leaving the operator
only Stop to resolve a divergence they could not see. On failure the state now holds and the
still-valid pair retries on the next tick.

**KGCS behaviour, supplied by PNU 2026-09-30.** KGCS is a closed project, so this is its
*specified* behaviour, not a bench measurement. A bench matrix is impractical for the same reason.

| | |
|---|---|
| When TC1 is sent | **only while a button is pressed** - never unsolicited |
| Start gesture | `Engine_OnOff` alternating **2/3 for 14 messages**, then stops |
| Stop gesture | `Engine_OnOff` alternating **4/5 for 14 messages**, then stops |
| Burst rate | constant, **~10 Hz, never above 11 Hz** |
| After a start | KGCS **blocks its Start button**; only Stop remains available |

*Why the timing works out.* The burst lasts ~1.4 s; the threshold of 10 fires at ~1.0 s, around
message 10 of 14. At the original threshold of 14 it landed at ~1.4 s - exactly where the burst
ends - which raced the FC's ~10 Hz sample clock against KGCS's ~10 Hz send clock and left the
outcome to drift. **Do not raise the threshold back to 14.**

*Why message loss does not matter.* A frozen pair keeps counting through a dropout, so a lost TC1
neither aborts nor delays the gesture. This is the same mechanism as the timer behaviour - it is
not independent robustness.

**Residuals, accepted rather than fixed** (deliberately not raised as separate PNU-ISSUEs):

- **The interlock is satisfied by two messages plus ~1 s of silence**, not by ten alternations.
  Sound only because KGCS never sends TC1 unsolicited - a contract that lives in a closed project
  and is not enforced anywhere in the firmware. **Step 2c addresses this**: the 500 ms staleness
  reset deliberately **tightens D7's unlimited loss tolerance**, recorded here rather than left as
  a silent change.
- **`handle_gcs_flcc_pmu_ctrl()` does not check the sender.** Any MAVLink source on any channel -
  a second GCS, a test tool, a log replay - can supply engine-start authority, against an
  effective bar of two messages and a second. (TC50 does check, from 5.1.20 - see `GCS-002`.)
- **`_cmd_tx_err` is write-only** - the strongest candidate in `PMU-003`.

*If this is ever revisited*, the sound form is to count alternations on arrival, treat a repeated
value as **hold** rather than reset (so a dropped TC1 neither advances nor aborts), and reset on
staleness. 14 messages give 13 alternations against a threshold of 10, so up to 3 may be lost. It
needs no KGCS change and is testable without KGCS or the engine: pymavlink with `pnu_kal.xml` can
emit TC1 bursts at any rate while you watch `Engine_OnOff_Echo` in TM3.

## PNU-ISSUE D8 - mount scheduler rate and msg 285 suppression

| | |
|---|---|
| **Status** | RESOLVED (step 5), both parts |
| **Component** | `Copter.cpp`, `AP_Mount_Backend`, `AP_Mount_Viewpro` |

**(a)** msg 285 suppression is now an **opt-in backend capability**
(`suppress_gimbal_device_attitude_status()`), so only Viewpro suppresses it; every other gimbal
keeps standard MAVLink behaviour. **Confirmed correctly keyed 2026-10-14** - KGCS only ever flies
a Viewpro, so it never sees 285.

**(b)** V5.0.8 lowered the `AP_Mount` task 50 -> 10 Hz; **deliberately reverted to 50 Hz**.
`AP_Mount_Viewpro::update()` self-throttles to 100 ms (upstream), so gimbal traffic is 10 Hz at
*either* scheduler rate - the scheduler does not control it. At 10 Hz the scheduler period
**equals** that throttle interval, so any late tick defers the update a full period (200 ms ->
5 Hz bursts); 50 Hz oversamples it 5x (worst case 120 ms). 50 Hz also keeps
`AP_Mount_Backend::update()` - servo retract and `update_poi_lock_target()`, which run *before*
the throttle - at full rate.

**To reduce gimbal traffic, raise `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS`, not the scheduler rate.**

## PNU-ISSUE D9 - the PMU failsafe

| | |
|---|---|
| **Status** | RESOLVED 2026-09-30, both parts. **(b) CORRECTED 2026-10-19** - the `ARMING_SKIPCHK` bypass it relied on never worked; fixed in 5.1.16 |
| **Component** | `events.cpp`, `AP_Arming_Copter.{h,cpp}`, `Copter.h` |

Both "no PMU" paths correctly avoid a false trigger: `PMUCAN_Fail` stays 2 when the driver runs
with no PMU, and stays 0 from static init when `CAN_Dn_PROTOCOL` is not 15.

**(a)** `failsafe.pmucan` was set but never cleared, so the failsafe could fire at most once per
power cycle. It now clears on recovery with `ERROR_RESOLVED`, matching `failsafe.terrain` /
`.deadreckon` / `.ekf`. The mode change is deliberately **not** undone, matching upstream
precedent.

**(b)** The failsafe could not tell "PMU lost in flight" from "armed without one", so an
emergency launch after a PMU dropout was disarmed ~100 ms after arming, with no prearm warning.
Implemented as designed, **with no new parameter** - `FS_PMU_ENABLE` was considered and rejected:

| Scenario | Prearm | In flight |
|---|---|---|
| PMU healthy, arm, PMU dies | passes | latch set -> failsafe fires as before |
| PMU dead, emergency launch | fails `"PMU not healthy (state n)"`; operator sets `ARMING_SKIPCHK` = 8192 or force-arms | latch clear -> no failsafe, flight proceeds |
| No PMU configured at all | `PMUCAN_Fail == 0` from static init -> passes | `== 0` early return, unchanged |

`AP_Arming_Copter::arm()` latches `failsafe.pmucan_armed_healthy = (PMUCAN_Fail == 0)`, and
`failsafe_pmucan_check()` returns early unless it is set. `failsafe.pmucan_armed_healthy` is one
bit in the existing `failsafe` struct; `Copter` is a global, so the bit is zero-initialised and
the failsafe is inert until something arms.

**`ARMING_CHECK` no longer exists in 4.7:** it was replaced by `ARMING_SKIPCHK` with the
*inverted* sense - you SET a bit to skip a check, where the old parameter had you clear one. A
migration at `AP_Arming.cpp:230` converts existing values on first boot, so only hand-entered
parameters are affected. **Bit 13 skips all System checks, not just this one** - inherent to
`Check::SYSTEM`, and the trade D9(b) accepted when it rejected a dedicated parameter.

**One deviation from the written design:** it lives in `AP_Arming_Copter`, not
`AP_Arming::can_checks()` - `AP_PMUCAN` is listed only in `ArduCopter/wscript`, so referencing
`PMU_Ctrl_Echo` from the shared library would fail to link for Plane, Rover and Sub.

**Verification.** Bench 2026-10-19: pulling the CAN connector while armed took the
`set_mode_RTL_or_land_with_pause()` branch immediately and recovered cleanly on reconnect. SITL
2026-10-29: the fake PMU's `silent` console command trips the failsafe to RTL and `resume` clears
it, with no connector pulled.

## PNU-ISSUE D12 - TM5 could not distinguish "nothing nearby" from "sensor dead"

| | |
|---|---|
| **Status** | RESOLVED (step 7). **The ICD decision is provisional and PNU will revisit** - see section 2 |
| **Component** | `GCS_Common.cpp` |

**Original.** The V5.0.8 code ignored the return of `get_horizontal_distances()`, which fills
every sector with `dist_max` on failure (`AP_Proximity_Boundary_3D.cpp:434`) and reads back as "no
object" - **so a dead sensor was byte-identical to open sky.**

**Fixed.** Sector scanning is gated on `sensor_failed()`, the return value is checked, and
`dist_array.valid(i)` excludes sectors that never reported.

**Decided for now: no ICD change.** `Object_Avoidance_Status = 3` ("sensor unhealthy") was
considered and **rejected** - it would need a KGCS update, and the failure already reaches KGCS
two other ways. TM5 stays all-zero when the sensor is dead; the health signal is the
`MAV_SEVERITY_CRITICAL` statustext on the healthy->failed edge (plus one if avoidance is switched
on while already failed) and the `MAV_SYS_STATUS_SENSOR_PROXIMITY` bit, which `GCS_Copter.cpp:67`
drives from the *same* `sensor_failed()` predicate - **so TM5 and `SYS_STATUS` cannot contradict
each other.**

## PNU-ISSUE D13 - the KGCS camera path hard-depended on `HAL_MOUNT_ENABLED`

| | |
|---|---|
| **Status** | RESOLVED 2026-09-30 |
| **Component** | `AP_Q30_config.h`, `AP_Q30.{h,cpp}`, `GCS_PNU.cpp`, `GCS_Common.cpp`, `Copter.h`, `UserCode.cpp`, `build_options.py` |

`AP::mount()` is declared only inside `#if HAL_MOUNT_ENABLED`, but `AP_Q30` called it unguarded
and `AP_Q30.h` had no `#if` at all, **so a MOUNT-disabled build failed to compile rather than
degrading.**

New `AP_Q30_ENABLED` flag in `AP_Q30/AP_Q30_config.h`, defaulting to `HAL_MOUNT_ENABLED`, guards
the library and all six call sites, and is registered in `build_options.py` (`Camera / Q30`,
depends on `MOUNT`) so the custom build server can drop it. **Verified by building with
`--define HAL_MOUNT_ENABLED=0`**: previously a compile failure, now succeeds at 1,653,036 B
(67 KB smaller). The normal build is byte-identical at 1,720,608 B, so the guards cost nothing
when enabled.

**This invariant is checked on every subsequent build** and is one of the five optional-feature
guards the CAN work also maintains.

## PNU-ISSUE D14 - globals shared by hand-written `extern`s

| | |
|---|---|
| **Status** | RESOLVED 2026-09-30 |
| **Component** | `AP_Q30.h`, `AP_PMUCAN.h`, `GCS_PNU.cpp`, `events.cpp` |

11 declarations across `GCS_PNU.cpp` and `ArduCopter/events.cpp`, with `PMU_Ctrl_Echo` declared
independently in both and nothing checking them against their definitions. **A type change at a
definition would have compiled silently in every translation unit and become memory
misinterpretation at link time.**

Each library now declares its own globals in its header, beside the definition: `AP_Q30.h` for
`tracking_counter`, the six `debug_cam_*`, `PREV_CAM_CMD` and `CAM_ATTITUDE_STATUS`;
`AP_PMUCAN.h` for `PMU_Status` and `PMU_Ctrl_Echo`. Both defining `.cpp` files include their own
header, so the compiler now checks declaration against definition and a mismatch is a build error.
All hand-written `extern`s removed.

**This rule is load-bearing for the later CAN work.** Keeping the globals' names and keeping them
declared in `AP_PMUCAN/AP_PMUCAN.h` is what let step 2 convert the PMU to a backend with **no
change at all** to `events.cpp`, `AP_Arming_Copter.cpp` or `GCS_PNU.cpp` - verified by diff. Both
globals and their definitions sit *outside* the `AP_PNU_CAN_ENABLED` guard, because those files
reference them unconditionally; that is also why TM50 still compiles and reports Disconnected with
`AP_PNU_CAN_ENABLED=0`. **Do not "tidy" the layout.**

## PNU-ISSUE D15 - PMU byte order

| | |
|---|---|
| **Status** | RESOLVED 2026-09-30 - no code change; the implementation was already correct |
| **Component** | `AP_PMUCAN.cpp` |

Every field in `handleFrame()` is read by `memcpy` straight into a native integer, which assumes
the PMU transmits little-endian. **The ICD confirms the PMU is little-endian (LSB first)**, which
matches the STM32, so no byte swap is needed on read or write.

Verified end to end: RX `memcpy` into native ints; the 24-bit `Engine_Hour_Count`, where LSB-first
puts the three bytes in the uint32's low three and the explicit `uint32_temp = 0U` supplies the
top byte; and TX `pmucan_cmd()`, which packs a `uint32` straight into the frame. ArduPilot has no
big-endian targets. **Commented at all three sites so nobody "fixes" it with `be16toh`/`be32toh` -
the pre-zero in particular looks removable and is not.**

## GCS-001 - the firmware version had not been bumped since 5.1.16

| | |
|---|---|
| **Status** | RESOLVED 2026-11-03 |
| **Component** | `ArduCopter/version.h`, `libraries/GCS_MAVLink/GCS.h` |
| **Related** | `D4`, which this was a residual of |

**Original.** `THISFIRMWARE` read `"PNU_OFP V5.1.23"`, but `FIRMWARE_VERSION`, `FW_PATCH` and
`OFP_VER_REV` all still read **16**.

**Impact.** `OFP_VER_MAIN/SUB/REV` are what `AP_PMUCAN.cpp:94-96` put into
`PMU_Status.Version_FLCC_*`, which TM13 reports to KGCS, and `FW_*` is what MAVLink
`AUTOPILOT_VERSION` reports. **Both told the ground station the aircraft was running 5.1.16 while
the firmware string said 5.1.23** - seven development versions of drift.

**Root cause.** D4 added `static_assert`s tying `OFP_VER_*` to `FW_*`, and those two always
agreed. **Nothing checked either of them against `THISFIRMWARE`**, which is the string every
version entry in `CHANGELOG.md` is named after, so the drift was invisible to the build.

**Fixed.** `FIRMWARE_VERSION`, `FW_PATCH` and `OFP_VER_REV` set to 23. Verified at runtime
against a live SITL: `AUTOPILOT_VERSION.flight_sw_version` now reads **5.1.23, type DEV**.

**Remaining limitation - the drift can recur.** A `static_assert` cannot parse `THISFIRMWARE`, so
there is still no build-time check that the three numeric defines agree with the version string.
**Bumping `THISFIRMWARE` must now be accompanied by `FIRMWARE_VERSION`, `FW_PATCH` and
`OFP_VER_REV`**, and that is a procedural rule, not an enforced one. Making it enforced would
need a build-time check or a release-procedure step. **TM13's FLMJ/FLMN/FLREV now report 5/1/23
to KGCS**, where they previously reported 5/1/16.

## SAR-002 - the navigation bundle was not observable from the FC

| | |
|---|---|
| **Status** | RESOLVED in 5.1.24. **Residual: the drop paths are unexercised** |
| **Component** | `libraries/AP_PNUSAR/AP_PNUSAR_Nav.cpp`, `libraries/AP_PNU_CAN/AP_PNU_CAN.{h,cpp}` |
| **Related** | `SAR-001`; found while planning the Lumir connection test, 2026-11-03 |

**Original.** `AP_PNUSAR` wrote nothing to the dataflash - `SARS` and `SARC` are both written
from `GCS_PNU.cpp` and cover the MAVLink side only. `_bundle_tx_cnt` and `_bundle_drop_cnt`
were read in exactly one place, inside `report_bundle_drop()`, which speaks only when
something is wrong.

**Impact.** A session in which every bundle flowed produced **no FC-side evidence at all**.
Silence meant either "all of them went out" or "it never started", and the only way to tell
them apart was `TM50` reaching state 3 - which says the observation was *started*, not that
nine frames left the bus ten times a second. For a first integration with Lumir that failed
twice over: nothing to answer "we are not receiving" with, and nothing to compare against
their log afterwards. **This is PNU-ISSUE(D6)'s silence pointing the other way** - not a
failure nobody sees, but a success nobody can prove.

**Fixed** by the `SARN` record, the per-cause drop counters, `get_last_tx_error()` and the
session summary statustext. See `CHANGELOG.md` V5.1.24.

**Remaining limitation - and SITL cannot close it.** The run that verified the record dropped
nothing, 0 of 200. Two deliberate attempts were then made to force a TX failure during an
active observation, inside the `--no-root` namespace where the bus can be manipulated:
`ip link set vcan0 down`, and then `ip link del vcan0`. **Neither produced a single send
failure** - SocketCAN accepted the writes in both cases, so `CANIface::send()` never returned
anything but success and `TxF` stayed 0. So `Ovr`, `Stale`, `TxF` and `TxErr` have never been
observed carrying a non-zero value.

The drop *detection* was measured in 5.1.22, when a 50 ms staleness window was losing 6% of
bundles; what has not been exercised is the per-cause accounting built on top of it. **Confirm
it on the bench**, where a disconnected transceiver, a bus-off condition or a congested
1 Mbit/s wire produces `TxF` and a negative `TxErr` naturally. Both attempts did confirm one
thing: `TxErr` held its sentinel of 1 throughout, which is what distinguishes "no SAR frame has
ever been refused" from the legitimate `0` meaning "TX queue full".

## GIMBAL-003 - yaw looked broken on the first bench log and is not

| | |
|---|---|
| **Status** | RESOLVED by explanation, 2026-10-16. No code change |
| **Component** | `AP_Mount_PNUGimbal`, `AP_Mount_Backend.cpp:120`, `:1013` |

Roll and pitch were confirmed on the first bench log: `MNT0.Roll` and `MNT0.Pitch` move opposite
vehicle roll and pitch, which is the AHRS correction and the whole of what step 1b set out to
prove. **`MNT0.YawB` did not move, and `MNT0.YawE` read the same as `ANG.Yaw`.** Both are one
cause.

`AP_Mount_Backend::write_log()` computes `yaw_ef = wrap_180(yaw_bf + degrees(ahrs_yaw))`, so with
`YawB` at zero, `YawE` equals the vehicle heading **by construction**. And `YawB` is zero because
**yaw stabilisation is not in `AP_Mount_PNUGimbal` at all** - roll and pitch subtract AHRS
explicitly, but yaw goes through `MountAngleTarget::get_bf_yaw()`, which subtracts heading **only
when `yaw_is_ef` is set**:

```text
if (yaw_is_ef) { return wrap_PI(yaw - AP::ahrs().get_yaw_rad()); }   // earth -> body
return yaw;                                                          // already body-frame
```

and `yaw_is_ef` comes straight from the yaw lock. With the lock off - "follow", the default - the
target is body-frame, nothing subtracts heading, and `YawB` sits at the target forever. **That is
the "yaw fix" not being switched on, not a defect.**

*Diagnosing it from a log without touching the code:* `write_log()` puts the desired yaw in
`DYawB` **or** `DYawE` depending on the frame, leaving the other NaN. `DYawB` populated and
`DYawE` NaN means the target is body-frame, so the lock is off.

*Turning it on:* `RCn_OPTION = 163` (`MOUNT_YAW_LOCK`), switch HIGH, with the mount in
RC_TARGETING (`MNT1_DEFLT_MODE = 3`). Or `GIMBAL_MANAGER_FLAGS_YAW_LOCK` over MAVLink. Note that
`set_yaw_lock()` **captures the heading at the moment it is switched** - the lock holds whatever
direction you were facing, not north - and that at `MNT1_YAW_MIN/MAX` of +/-15 degrees it clips
after about 15 degrees of heading change, which is the expected saturation recorded in section 3.

## Resolved design questions

Kept because the reasoning is worth more than the answer.

**Does `AP_PNUGimbal` back an `AP_Mount` backend? YES** - answered 2026-10-16, implemented in
5.1.12. `AP_Mount` already stabilises against FC attitude and the precedent is camera-less:
`AP_Mount_Servo::update_angle_outputs()` is the whole of it in about 40 lines - take the
earth-frame target, rotate the AHRS lean angles into the gimbal's yaw frame, subtract them from
roll and pitch, then apply a gyro lead filter. The `requires_stabilization` constructor flag is
what separates the two cases: `Type::Servo` passes **true** and `Type::BrushlessPWM` passes
**false**, because a self-stabilising brushless gimbal does its own. **We are the `true` case** -
the CubeMars drives have no attitude reference of their own. **A camera is not required**: the
only pure virtuals in `AP_Mount_Backend` are `has_pan_control()` and
`get_attitude_quaternion()`; every camera method has a `return false` default.

**Is 50 Hz fast enough? Very likely yes.** The framing matters more than the numbers: **the
CubeMars closes its own position, velocity and current loops at the motor.** The FC feeds
setpoints, it does not close a servo loop. `AP_Mount_Servo` does exactly this at 50 Hz into real
RC servos. `AP_Mount::update()` is a 50 Hz `SCHED_TASK`; `update_fast()` is a FAST_TASK at 400 Hz
on Copter, used only by SoloGimbal. **The bus is not the constraint** - three command frames is
~0.4 ms at 1 Mbit/s, so 100 Hz costs ~4% and even 400 Hz ~16%.

**Where does the state machine live? In the gimbal layer under `AP_PNU_CAN`** - not in
`AP_Mount_Backend`, which is a shared library where AGENTS.md forbids PNU-specific logic, and not
in `AP_PNUGimbal_CubeMars`, because a second motor protocol would need the same lifecycle. Three
reasons: it runs in the CAN thread where the feedback actually arrives, so transitions happen
immediately rather than waiting for the next 50 Hz tick; it survives `HAL_MOUNT_ENABLED=0` (D13);
and it stays protocol-neutral.

**Why one driver and not three? Because `AP_CANManager` allows nothing else.**
`AP_CANManager::init()` loops over *interfaces*, and each interface carries exactly one driver
number (`AP_CANManager.cpp:139-152`). Many interfaces to one driver is supported; **one interface
to many drivers is not**, and a driver with no interface is never allocated at all, so it fails
silently. `CAN_Dn_PROTOCOL2` does not help: it is a DroneCAN-hosted auxiliary slot, it requires
the primary driver to implement `add_11bit_driver()` - **only `AP_DroneCAN` does** - it accepts
**one** aux driver, and that driver must be a `CANSensor` with no thread of its own.

**Are the two limit sets one or two? Two**, answered by PNU 2026-10-16. Mechanical
(+/-30 / +/-30 / +/-20) and operating (+/-25 / +/-25 / +/-15), both in the gimbal frame. The
payload is bulky and can foul the landing skid struts. `AP_Mount` has only one set, so they are
split by layer: `MNT1_*_MIN/MAX` carry the **operating** limits and `AP_Mount` clamps to them,
while the **mechanical limits live in the PNU gimbal backend as the last clamp before a frame is
transmitted**. The 5 degree margin absorbs stabilisation overshoot.

**Are operating limits derived from measured travel? NO** - answered 2026-10-19. As long as the
measured travel is at least twice (mechanical - `PNU_MARG`) - 50 degrees on a 60 degree axis -
`MNT1_*` keeps its +/-25 and homing passes. Below that it aborts as obstructed. Deriving tighter
limits from each boot's measurement was rejected: **stop compliance under a heavy payload would
make the mount's usable reach vary with how hard it happened to press on its stops**, which is
worse than a fixed number everyone can reason about.

**Which axis order? Roll, then pitch, then yaw** - answered by PNU 2026-10-16. Roll first because
**the mass offset is on the roll axis only**, so it is the axis that sags and swings under gravity
and wants to be under control first. Yaw last because it is safer to swing the payload in yaw with
pitch already neutral, which keeps the swept volume small. **The two idle axes must be HELD, not
left floppy** - the manual does not define what an enabled motor does with no command outstanding.

**How is MAINTENANCE entered and left?** By parameter, not an RC switch, because a stray RC glitch
must not be able to drop the FC off the bus. Its purpose is connecting a PC to one motor's UART,
so in MAINTENANCE the FC must **transmit nothing at all** - CAN has priority over PWM on these
drives (manual p.38). Receiving is harmless and useful, so the driver keeps listening.
**The hazard is leaving MAINTENANCE, not entering it**: the utility can change the CAN ID, the
control mode and the P/V/T/KP/KD ranges, so **leaving requires a reboot**, which forces a fresh
INIT and a fresh homing run against whatever the motor now believes.

**What does FAULT do? A policy PNU still has to choose.** Status codes 0x8-0xE are all faults but
they are not equivalent: comms loss (0xD) and MOS overtemperature (0xB) are recoverable,
overcurrent (0xA) and overload (0xE) may not be. Feedback staleness belongs here too. **This one
is still open** and is tracked under `GIMBAL-004`'s bench checklist item 5.
