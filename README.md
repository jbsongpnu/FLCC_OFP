# Migration Break Point for Future Updates
- This branch will update Copter-4.7.1 to FLCC V5.0.8 step-by-step to find new commit break points for future updates.  
- IN THIS VERSION : Step 7 - GCS handler bodies (`GCS_Common.cpp`)  
   - PMU telemetry (TM1/TM3) and PMU/CAM commands (TC1/TC2) now reach the GCS end-to-end  
   - CAM status (TM2) and object-avoidance status (TM5) senders are in place but not yet streamed - step 8 adds the callers  
   - New PNU-ISSUE D12, resolved in the same step : TM5 could not distinguish "nothing nearby" from "sensor reporting nothing"  
   - D12 closed **provisionally** without an ICD change - to be revisited once the migration break points are established  
   - PNU-ISSUE D5 is now unblocked by step 7, but **not started** - the EOF block has not been extracted  
- PNU-KAL Specific options  
   - CAN Driver Option for PMU : CAN_D1_PROTOCOL = 15  
- Viewpro Mount Specific options  
   - MNT1_TYPE = 11 (Viewpro) / CAM1_TYPE = 4 (Mount) / SERIALx_BAUD = 115 (115,200 bps) / SERIALx_PROTOCOL = 8 (Viewpro)  
- Gremsy Mount Specific options  
   - MNT1_TYPE = 6 (MAVLink-Gremsy) / CAM1_TYPE = 6 (MAVLinkCAMV2) / SERIALx_BAUD = 115 (115,200 bps) / SERIALx_PROTOCOL = 2 (MAVLink2)   

<Planned Commit Breaks and Migration Steps>

| # | Step | Files | Needs | Unblocks |
|:---:|---|---|---|---|
| 1 | MAVLink dialect | `Forced Submodule File/` ×2 | — | 2, 4, 5, 7, 8 |
| 2 | Enum & ID registry | `ModeReason.h`, `AP_Logger.h`, `AP_Arming.h`, `AP_CAN.h`, `ap_message.h`, `GCS.h`, `AP_Arming.cpp` | 1 | 4, 5, 6, 7, 8 |
| 3 | Gimbal driver | `AP_Mount.{cpp,h}`, `AP_Mount_Backend.{cpp,h}`, `AP_Mount_Viewpro.{cpp,h}` | — | 5, 7 |
| 4 | PMU CAN stack / version.h | `AP_PMUCAN/` ×6, `AP_CANManager.{h,cpp}`, `wscript` (PMUCAN line only), `version.h` | 1, 2 | 6, 7 |
| 5 | Camera adapter | `AP_Q30/` ×2, `AP_SerialManager.cpp`, `wscript` (Q30 line only), `Copter.h` (include + `AP_Q30 q30`) | 1, 2, 3 | 7 |
| 6 | PMU failsafe | `events.cpp`, `Copter.h` (failsafe bit + decl), `Copter.cpp` (10 Hz call) | 2, 4 | — |
| 7 | GCS handler bodies | `GCS_Common.cpp`, `GCS.h` (D12 edge latch) | 1, 2, 3, 4, 5 | 8 |
| 8 | Object avoidance | `UserCode.cpp`, `APM_Config.h` | 1, 2, 7 | — |
| — | NMEA output | `AP_NMEA_Output.{cpp,h}` | — | — |
| — | Vehicle tuning | `config.h` | — | — |

<Deferred Issues>

Open decisions parked until a later step can validate them. Each has a `PNU-ISSUE(<id>)`
marker at the code site. List them all with:

```
grep -rn "PNU-ISSUE" libraries/ ArduCopter/
```

| ID | Issue | Site | Blocked until | Decision needed |
|:--:|---|---|:--:|---|
| D1 | **IR palette deferral - mechanism hardware-verified; two residual races.** Hardware test confirms the prelude &rarr; 100 ms &rarr; colour sequence works, including alongside `set_camera_source`. Two defects remain but are narrow races that normal operation will not reach: (a) `_image_sensor` is not re-checked at send time - it is assigned *only* from gimbal telemetry (`AP_Mount_Viewpro.cpp:313`; `set_camera_source()` never touches it), so it would have to change inside the 100 ms window; (b) the send result is ignored and the pending colour cleared regardless, which needs a UART txspace failure at that instant. Consequence of either is cosmetic - one dropped palette command the operator re-presses; nothing safety-relevant. The prelude-to-colour gap is always **100 ms** (`update()` self-throttles to `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS`), independent of the `AP_Mount` scheduler rate. Note: the `0x23`/`0x24` substitution in `AP_Q30` is a camera *vendor* change (newer units dropped the extended pseudo-colour options; `0x0E`, `0x0F`, `0x21`, `0x22` work, IR_RAINBOW offers Red only) - not evidence of deferral misbehaviour. | `AP_Mount_Viewpro.cpp` | — | **Closure depends on one fact, shared with D7: does KGCS send TC2 only on operator action, or stream it?** On-action &rarr; two messages cannot arrive within 100 ms, race is unreachable, **close with no code change**. Streamed &rarr; apply the ~10-line fix (snapshot sensor + staleness deadline, clear only on success). Still untested: whether the deferral is *needed* at all - that requires removing the prelude and retesting. |
| D2 | All 7 PNU-KAL ICD messages are id ≥ 50001, so MAVLink **v2 only**. A link set to `SERIALn_PROTOCOL=1` (MAVLink1) silently carries no PNU-KAL telemetry or commands. | `Forced Submodule File/common.xml` | 7 | Assert v2 on the PNU-KAL link, or document as a setup constraint |
| D3 | The `modules/mavlink` dialect edits are invisible to git — a `submodule update` or fresh clone silently reverts them and the build fails at step 7 with ~21 unknown-type errors. | `Forced Submodule File/ReadMe.txt` | — | Write `Tools/pnu/apply_mavlink_dialect.sh` (idempotent re-apply) |
| D4 | `OFP_VER_MAIN/SUB/REV` in `GCS.h` and `FW_MAJOR/MINOR/PATCH` in `version.h` are two hand-maintained copies of the same version with nothing enforcing agreement. | `GCS.h`, `version.h` | 4 | **RESOLVED (step 4)** - `static_assert` in `Copter.cpp` (only TU seeing both; `AP_PMUCAN.cpp` cannot include the vehicle `version.h`) |
| D5 | **OPEN - not started.** 357 of `GCS_Common.cpp`'s 429 added lines are one block appended at EOF — the main recurring merge cost on every future ArduPilot bump, and the only issue here whose cost grows with time rather than staying flat. | `GCS_Common.cpp` | — (was 7) | **Not done.** Step 7 cleared the prerequisite only - the block now exists to be moved; `libraries/GCS_MAVLink/GCS_PNU.cpp` has not been created. Extract the 357-line EOF block there: it is self-contained and touches no `GCS_Common.cpp` statics. **72 lines must stay behind** - the 4 `try_send_message()` cases, 3 `handle_message()` cases, 4 id-map entries and 2 includes - and that is the residual per-bump merge cost. Worth doing *before* the D12 proximity rework, so that rework lands in a file that does not re-conflict on every upstream merge |
| D6 | `AP_PMUCAN::handleFrame()` performs no DLC validation - each case `memcpy`s from fixed offsets up to `data[7]` whatever `can_rxframe.dlc` says. No out-of-bounds read (`data[]` is fixed 8 bytes), but a short or malformed PMU frame is parsed silently and produces stale values for battery current, RPM, fuel quantity etc. | `AP_PMUCAN.cpp` | — | Confirm expected DLC per PMU message ID in the ICD, then reject or pad short frames |
| D7 | **Engine ON/OFF interlock is satisfied by inactivity.** `engineonoffstate()` counts to 10 at ~10 Hz, but `_pmu_ctrl_cmd`/`_pmu_ctrl_cmd_prv` are sticky between TC1 messages. If KGCS sends TC1 only on operator action, two messages (e.g. 3 then 2) freeze a valid pair and the counter climbs unattended - **engine STARTS after ~1 s of silence**. If KGCS streams TC1 with a held value, `prv == cmd` resets every tick and the **engine can never be commanded OFF**. Also rate-dependent (pairs overwritten above ~10 Hz) and loss-sensitive (a dropped TC1 resets progress). | `AP_PMUCAN.cpp` | — | Bench-test the matrix below, then re-specify the interlock (edge-latched count, or an explicit hold-duration) |
| D8 | **Mount scheduler rate vs. the Viewpro self-throttle.** (a) *RESOLVED (step 5)* - msg 285 suppression is now an opt-in backend capability (`suppress_gimbal_device_attitude_status()`), so only Viewpro suppresses it; every other gimbal keeps standard MAVLink behaviour. (b) *RESOLVED (step 5)* - V5.0.8 lowered the `AP_Mount` task 50&rarr;10 Hz; **deliberately reverted to 50 Hz**. `AP_Mount_Viewpro::update()` self-throttles to `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS` (100 ms, upstream), so gimbal traffic is 10 Hz at *either* scheduler rate - the scheduler does not control it. At 10 Hz the scheduler period **equals** that throttle interval, so any late tick defers the update a full period (200 ms &rarr; 5 Hz bursts); 50 Hz oversamples it 5x (worst case 120 ms). 50 Hz also keeps `AP_Mount_Backend::update()` - servo retract and `update_poi_lock_target()`, which run *before* the throttle - at full rate. | `Copter.cpp` | — | Done. To reduce gimbal traffic, raise `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS`, not the scheduler rate |
| D9 | **PMU failsafe: no-PMU case is correct; one gap fixed, one open.** Both "no PMU" paths correctly avoid a false trigger (`PMUCAN_Fail` stays 2 when the driver runs with no PMU; stays 0 from static init when `CAN_Dn_PROTOCOL` is not 15). (a) *RESOLVED (step 6)* - `failsafe.pmucan` was set but never cleared, so the failsafe could fire at most once per power cycle. It now clears on recovery with `ERROR_RESOLVED`, matching `failsafe.terrain` / `.deadreckon` / `.ekf`. The mode change is deliberately **not** undone, matching upstream precedent. (b) **OPEN** - once the PMU has been seen and then lost, state is 1 (`COMMUNICATION_ERROR`) with no path back to 2 and no disable parameter, so **emergency takeoff after a PMU dropout is blocked**. Worse, PMUCAN has *no prearm check* (`AP_Arming.cpp:1354` is a bare `break`), so prearm passes silently, arming succeeds, and `should_disarm_on_failsafe()` disarms ~100 ms later with no prior warning. | `events.cpp`, `AP_Arming.cpp` | — | Design agreed, **dedicated commit after the migration** - it changes flight-safety behaviour and should not be folded into a migration step. No new parameter. See design below. |
| D10 | **`AP_Q30` routes every camera function through `AP::mount()`, never `AP::camera()`.** Valid for Viewpro (one serial protocol carries gimbal + camera), but on any other mount all ~27 camera calls fall through to the base class and silently do nothing, and `get_zoom_times()` returns a fabricated `0.0f`. Dormant if the camera is driven by standard MAVLink2 instead of KGCS TC2. | `AP_Q30.cpp`, `UserCode.cpp` | 8 | Decide whether KGCS TC2 must drive non-Viewpro cameras; if so, route camera calls via `AP::camera()` - but see **D11**, the destination cannot do everything. See details below. |
| D11 | **Even with correct routing (D10), the MAVLink camera backend cannot cover everything KGCS needs.** Pristine 4.7.1 *does* have a MAVLink camera path (`AP_Camera_MAVLinkCamV2`, `CAM1_TYPE=6`) - it is the *mount* that is gimbal-only. Of `AP_Q30`'s 8 camera functions, 4 work, 2 exist only in the `AP_Camera` base, and 2 have **no path at all**: `get_zoom_times` (the backend never decodes `CAMERA_SETTINGS` msg 260, where `zoomLevel` lives) and `IR_Color_Change` (MAVLink has no standard thermal-palette message). | `AP_Camera_MAVLinkCamV2.cpp` | — | Bench a real VIO first; then decide per function - upstream fix, local fix, or vendor-specific. See details below. |
| D12 | **TM5 could not distinguish "nothing nearby" from "sensor reporting nothing".** *RESOLVED (step 7), decision provisional - see revisit note below.* The V5.0.8 code ignored the return of `get_horizontal_distances()`, which fills every sector with `dist_max` on failure (`AP_Proximity_Boundary_3D.cpp:434`) and reads back as "no object" - so a dead sensor was byte-identical to open sky. Sector scanning is now gated on `sensor_failed()`, the return value is checked, and `dist_array.valid(i)` excludes sectors that never reported. | `GCS_Common.cpp` | — | **Decided for now: no ICD change** (provisional - PNU will revisit). `Object_Avoidance_Status = 3` ("sensor unhealthy") was considered and **rejected** - it would need a KGCS update, and the failure already reaches KGCS two other ways. TM5 stays all-zero when the sensor is dead; the health signal is the `MAV_SEVERITY_CRITICAL` statustext on the healthy&rarr;failed edge (plus one if avoidance is switched on while already failed) and the `MAV_SYS_STATUS_SENSOR_PROXIMITY` bit, which `GCS_Copter.cpp:67` drives from the *same* `sensor_failed()` predicate - so TM5 and `SYS_STATUS` cannot contradict each other. **Residual:** confirm 10 m / 15 m are the intended operator thresholds (`PNU_OA_ALERT_DISTANCE_CM` / `PNU_OA_WARN_DISTANCE_CM`); they are hard-coded and unrelated to `AVOID_MARGIN` |

**PNU-ISSUE D7 bench matrix** - engine disconnected, watch `Engine_OnOff_Echo` in TM3:

| # | TC1 pattern from KGCS | Expected if interlock is sound | Actual defect it exposes |
|:-:|---|---|---|
| 1 | Two messages (`Engine_OnOff` 3 then 2), then stop sending | no engine command | engine ON after ~1 s of silence |
| 2 | Stream at 10 Hz with `Engine_OnOff` held at 2 | engine ON after the hold | counter resets every tick, never fires |
| 3 | Alternate 2/3 at 5 Hz | engine ON after 10 toggles | fires after ~5 toggles (each pair sampled twice) |
| 4 | Alternate 2/3 at 20 Hz | engine ON after 10 toggles | needs >10 toggles (pairs overwritten unsampled) |

Repeat 1-4 with 4/5 in place of 2/3 to cover the ON&rarr;OFF direction; case 2 there is the
serious one (engine cannot be stopped).


**Viewpro pitch reversal is intentional - do not "fix" it.**
`AP_Mount_Backend::set_angle_target()` negates the pitch target when the backend
returns true from `pitch_target_is_reversed()`, which only `AP_Mount_Viewpro`
does. This is a KGCS operator convention, not a protocol requirement: when the
operator presses gimbal pitch-down they expect the *video image* to pan
downwards, which means the gimbal must physically pitch up. It applies to KGCS
users, who so far use the Viewpro camera only.

Deliberately scoped:

| Path | Writer | Where the flip happens | Reversed |
|---|---|---|:-:|
| MAVLink angle commands (`DO_MOUNT_CONTROL`, `GIMBAL_MANAGER_*`, AUTO mission, `AP_Q30::send_cmd_angle()`) | `set_angle_target()` | `pitch_target_is_reversed()` inside `set_angle_target()` | **yes** - this is what KGCS drives |
| **KGCS rate commands (TC2 `Control_Mode` 1)** | `set_rate_target()` | **`AP_Q30::send_cmd_speed()` negates `Pitch_Speed_CMD` at the call site** - `set_rate_target()` itself does *not* apply `pitch_target_is_reversed()` | **yes** - same operator convention, different mechanism |
| RC stick control | `update_mnt_target_from_rc_target()` | — | no - physical convention |
| ROI / point-at-location | `get_angle_target_to_location()` | — | no - physical convention |
| Rate integration into an angle target | `update_angle_target_from_rate()` | — | no - operates on the already-correct rate |

So the operator sees image convention on **both** angle and rate, by two different
mechanisms. `set_rate_target()` is the trap: it looks inconsistent with
`set_angle_target()` and invites a "consistency fix", but adding
`pitch_target_is_reversed()` there would **double-negate** the KGCS rate path and invert
gimbal pitch rate. A comment at `AP_Mount_Backend::set_rate_target()` says so.

`AP_Q30::send_cmd_hold_angle()` sends an all-zero rate, so the sign is irrelevant there.

The limits (`MNT1_PITCH_MIN` / `MNT1_PITCH_MAX`) are applied in ArduPilot
convention *before* the sign flip, so they keep their documented meaning: set
them to the gimbal's true mechanical travel, negative = down.

Any other gimbal (Gremsy etc.) inherits `false` from the base class and is
unaffected.


**PNU-ISSUE D10 details** - resume at Step 8.

`AP_Q30` holds no `AP::camera()` reference. Of its 30 `mount->` calls only three are
genuinely gimbal operations:

| Line | Call | Kind |
|---|---|---|
| 53 | `set_angle_target()` | gimbal |
| 91 | `set_rate_target()` | gimbal |
| 137 | `set_rate_target(0,0,0,0)` | gimbal |

The other ~27 are camera operations addressed to the mount object: `set_zoom` x7,
`IR_Color_Change` x6, `set_camera_source` x4, `set_focus` x2, `record_video` x2,
`set_tracking` x2, `get_zoom_times` x2, `take_picture`.

On a non-Viewpro mount (Gremsy is `AP_Mount_MAVLink`, which implements none of them)
every call hits the base-class default, and **`AP_Q30` checks no return value** - there
is not one `if (mount->...)` guard in the file:

| Method | Base-class return on a Gremsy |
|---|---|
| `set_zoom` / `record_video` / `take_picture` / `set_tracking` / `set_camera_source` / `IR_Color_Change` | `false` |
| `set_focus` | `SetFocusResult::UNSUPPORTED` |
| `get_zoom_times` | **`0.0f`** - a fabricated value, not an error |

Result: gimbal pointing works, every camera command silently does nothing, and KGCS is
told zoom is 0x. No error, no warning, no build failure.

**Scope - when this does and does not matter:**

- *Standard MAVLink2 camera control* (`MAV_CMD_SET_CAMERA_ZOOM`, `MAV_CMD_IMAGE_START_CAPTURE`,
  `GIMBAL_MANAGER_*`) goes through `AP_Camera` / `AP_Mount` handlers and never enters
  `AP_Q30`. The issue is dormant. `AP_Q30` is passive - its constructor only sets
  `_singleton`, it registers no scheduler task, timer or thread, and its sole entry point
  is `handle_gcs_flcc_cam_cmd()` (`GCS_Common.cpp`), reached only by msg 50002 (TC2).
- *KGCS TC2 driving a non-Viewpro camera* is the case that breaks.

**Related residual, also Step 8:** `UserCode.cpp::userhook_MediumLoop()` sends
`MSG_CAM_STATUS` unconditionally at 10 Hz. `send_message_gcs_flcc_cam_status()` guards
`AP::mount() == nullptr` but not whether the mount supports zoom - so with a Gremsy it
emits a 10 Hz TM2 stream reporting zoom 0x to every connected GCS. Gate it on the mount
actually supporting zoom, or on TC2 having been seen recently.

**If camera routing is added:** units differ across the boundary. `AP_Mount_Viewpro::set_zoom(PCT)`
was modified to take zoom *times* (`zoom_value * 10`), while `AP_Camera_MAVLinkCamV2::set_zoom(PCT)`
follows the MAVLink contract of 0-100 percent. A shared route must convert.


**PNU-ISSUE D11 details** - capability gap behind D10. Resume with Gremsy hardware.

D10 is *where* `AP_Q30` sends camera calls. D11 is *what the destination can actually do*.
They are separable: fixing the routing still leaves two functions with no path.

Measured against pristine 4.7.1:

| `AP_Q30` needs | `AP_Camera_MAVLinkCamV2` | `AP_Camera` base | Status |
|---|:-:|:-:|---|
| `record_video`, `set_zoom`, `set_focus` | yes | yes | works |
| `take_picture` | - | yes | works via base |
| `set_tracking`, `set_camera_source` | - | yes | base only - check it reaches a MAVLink camera |
| `get_zoom_times` | - | - | **no path** |
| `IR_Color_Change` | - | - | **no path** |

The two with no path:

- **`get_zoom_times`** - `AP_Camera_MAVLinkCamV2::handle_message()` decodes only
  `CAMERA_INFORMATION`; it never handles `CAMERA_SETTINGS` (msg 260), which carries
  `zoomLevel`. Self-contained to add, and a genuine upstream gap - **check ArduPilot master
  before writing it locally**, or it becomes another permanent divergence like D5.
- **`IR_Color_Change`** - MAVLink defines no thermal-palette message. Vendor-specific by
  nature; would need Gremsy's own extension or the camera's parameter protocol. Cannot be
  ported from the Viewpro implementation.

**Why this was deferred rather than done during the migration:**

- Nothing is blocked. Viewpro works; a Gremsy driven by *standard* MAVLink2 works (gimbal via
  `AP_Mount_MAVLink`, camera via `AP_Camera_MAVLinkCamV2`). Only KGCS-TC2-drives-Gremsy breaks.
- Step 7 does not force the decision: it applies `GCS_Common.cpp` verbatim and its handler only
  calls `Q30->...`, so routing is internal to `AP_Q30`. Nothing gets cheaper by deciding early.
- Designing now would be blind. Nothing in this tree references ZIO or VIO. Unknown: what a VIO
  reports as `model_name`, which `CAMERA_CAP_FLAGS` it sets, whether its camera is a separate
  component ID, whether it exposes thermal at all. Each changes the design.
- It is new functionality, not migration. Mixing it in breaks the 1:1 mapping between each step
  and the known V5.0.8 delta, which is the point of this branch.

**First step when resuming:** connect a VIO and capture `CAMERA_INFORMATION` plus the
`"Mount: %s %s fw:..."` statustext from `AP_Mount_MAVLink::handle_gimbal_device_information()`.
That answers most of the unknowns above in minutes. `AP_Mount_MAVLink.h:51` already shows the
in-tree precedent for branching on `vendor_name` / `model_name` (the AVTA/CM41 case).

**Suggested sequencing:** finish steps 6-8, cut 5.1.0 with the deferred issues resolved, then
take Gremsy support as its own piece of work starting from that bench session.

> **ACTION ON PNU - blocking D10 and D11.**
> PNU must establish with Gremsy, and deliver to this project, exactly which camera
> functions a ZIO / VIO supports and over which protocol. Nothing in ArduPilot answers
> this, and no code decision for D10 or D11 can be made without it.
>
> Specifically to confirm, per model:
>
> | Question | Why it matters |
> |---|---|
> | Is there any thermal-palette / IR colour control at all, and over what protocol? | `IR_Color_Change` has no MAVLink standard. If Gremsy has no equivalent, KGCS must hide the IR colour buttons for this camera rather than send commands that do nothing. |
> | Does the camera report `CAMERA_SETTINGS` (msg 260) with `zoomLevel`? | Decides whether `get_zoom_times` can be implemented at all, or whether TM2 must report zoom as unavailable. |
> | Is the camera a separate MAVLink component ID from the gimbal? | Decides whether `AP_Camera` (`CAM1_TYPE=6`) can reach it, and what `AP_Q30` must address. |
> | Which `CAMERA_CAP_FLAGS` does it advertise? | Tells us zoom / focus / video / tracking support without guessing. |
> | Exact `vendor_name` / `model_name` strings per model | Needed to tell ZIO from VIO at runtime, per the `AP_Mount_MAVLink.h:51` precedent. |
> | Is `set_camera_source` (EO / IR / PIP switching) supported, and how? | Four of `AP_Q30`'s calls; no MAVLink standard path. |
>
> Until this is delivered, D10 and D11 stay open and no Gremsy camera work should start.


**PNU-ISSUE D9(b) design** - PMU failsafe cannot distinguish "lost in flight" from
"never had one". Agreed approach, to be implemented as its own commit after the migration.
`FS_PMU_ENABLE` was considered and **rejected** - no new parameter.

*Core change - only latch if the PMU was healthy at arming.* A failsafe should protect
against losing a resource in flight, not against starting without one:

```cpp
// on arm (Copter::arm_motors() or alongside existing failsafe init - find a clean
// hook rather than edge-detecting motors->armed() inside failsafe_pmucan_check())
failsafe.pmucan_armed_healthy = (PMU_Ctrl_Echo.PMUCAN_Fail == 0);

// in failsafe_pmucan_check(), after the ==2 / ==0 early returns
if (!failsafe.pmucan_armed_healthy) {
    return;     // armed deliberately without a healthy PMU - nothing to protect
}
```

One extra bit in the existing `failsafe` struct. No parameter.

*Prearm check - for the warning, not the block.* `can_checks()` is called unconditionally
from the aggregate at `AP_Arming.cpp:1722` and does not gate on a bit itself, so use
`check_failed(Check::SYSTEM, ...)` (the `AP_Arming.cpp:354` idiom). That makes it bypassable
via the standard `ARMING_CHECK` bit 13 - ArduPilot's documented operator override, so no new
parameter is needed for the emergency case. Replaces the bare `break` at `AP_Arming.cpp:1354`.

| Scenario | Prearm | In flight |
|---|---|---|
| PMU healthy, arm, PMU dies | passes | latch healthy -> failsafe fires |
| PMU dead, emergency launch | fails "PMU not healthy"; operator clears `ARMING_CHECK` bit 13 or force-arms | latch unhealthy -> no failsafe, flight proceeds |
| No PMU configured at all | `get_pmucan()` returns nullptr -> passes | `PMUCAN_Fail == 0` from static init -> early return |

*Why a prearm check alone is not enough:* even after bypassing prearm, the current code
fires the failsafe ~100 ms after arming and `should_disarm_on_failsafe()` disarms on the
ground. Without the arm-time latch the operator trades a confusing disarm for a refusal
plus the same disarm.



**PNU-ISSUE D12 revisit note.** The step-7 resolution is **provisional**. PNU plans to
change the proximity sensor *and* KGCS, with major rework of the proximity-related
functions, so the D12 decision should be re-opened once the migration break points are
established and that rework is scoped - not before.

What to re-examine then, and why each may flip:

| Item | Why it may change |
|---|---|
| "no ICD change" | Rejected only because it needed a KGCS update. If KGCS is being changed anyway, `Object_Avoidance_Status = 3` ("sensor unhealthy") becomes nearly free and is the honest encoding |
| statustext as the health signal | A workaround for an unchangeable KGCS. Once KGCS is in scope, in-band health in TM5 is better than a message the operator can scroll past |
| `sensor_failed()` as the predicate | Chosen to match `MAV_SYS_STATUS_SENSOR_PROXIMITY` (`GCS_Copter.cpp:67`). A different sensor may need per-instance or per-sector health instead of one aggregate bool |
| 10 m / 15 m thresholds | Hard-coded, unrelated to `AVOID_MARGIN`, and tuned for TeraRanger Tower Evo's 0.5-60 m range. A sensor with a different range almost certainly needs different numbers, or parameters instead of `#define`s |
| 8 fixed sectors | TM5's `Object_Existence` is one bit per `PROXIMITY_MAX_DIRECTION` sector. A sensor with different angular coverage may not map onto 8 sectors at all |

What **not** to undo in the meantime: the checked `get_horizontal_distances()` return,
the `dist_array.valid(i)` per-sector guard and the `sensor_failed()` gate are correctness
fixes independent of the encoding question. They stop a dead sensor reading as a clear
scene whatever TM5 ends up looking like.

---

<Working Notes - read before applying any step>

**Board / build.** CubeOrangePlus, `-Werror` is ON. `./waf copter` must finish with
**zero warnings and zero errors**; anything else is a regression. Check after every change.

**Never stage anything.** Staging and committing are done manually after review.
This matters mechanically: `git checkout <tree> -- <path>` writes to the index.
Apply snapshot files with `git show v5.0.8-snapshot:<path> > <path>` instead.

**`git diff v5.0.8-snapshot` is NOT a to-do list.** 30 files already differ deliberately.
Applying a snapshot file wholesale will silently revert decisions. Before taking any file
whole, check it is a pure addition:

```
git diff --stat HEAD v5.0.8-snapshot -- <file>      # 0 deletions => safe to take whole
```

If it shows deletions, apply targeted edits only.

**Files that must never be taken wholesale** (they carry deliberate deviations):

| File | Would be lost |
|---|---|
| `ArduCopter/Copter.cpp` | 3x `static_assert(OFP_VER... == FW_...)` (step 4, D4); mount task kept at **50 Hz** (D8) |
| `ArduCopter/Copter.h` | `AP_Q30 q30` member; `failsafe.pmucan` bit |
| `libraries/AP_PMUCAN/*` | the whole step-4 cleanup; 4 vendored `pmucan_*.hpp` were **deleted** |
| `libraries/AP_Mount/AP_Mount_Backend.{h,cpp}` | `pitch_target_is_reversed()`, `suppress_gimbal_device_attitude_status()` (D8a) |
| `libraries/AP_Mount/AP_Mount_Viewpro.{h,cpp}` | both overrides; D1 markers |
| `libraries/AP_SerialManager/AP_SerialManager.h` | IOMCU renumber deliberately **not** applied |
| `libraries/AP_OSD/AP_OSD_ParamSetting.cpp` | `"Q30"` padding deliberately **not** applied |
| `libraries/GCS_MAVLink/GCS_Common.cpp` | upward-proximity block deliberately **not** commented out; the step-7 defect fixes and `#if` guards below |
| `libraries/AP_Q30/AP_Q30.cpp` | `send_cmd_speed()` negates pitch at the call site - see the pitch-reversal table |
| `ArduCopter/events.cpp` | D9(a) recovery-clear fix; `LOGGER_WRITE_ERROR` portability fix |
| `ArduCopter/version.h` | 5.0.8 **DEV** + the 5.1.0 release note |
| `README.md` | this file - never taken from the snapshot |

Most other deviations are KAL -> PNU / PNU-KAL comment renames, which are cosmetic but still
make a wholesale copy a regression.

**Substantive deviations from V5.0.8, with reasons:**

| Deviation | Why |
|---|---|
| `SerialProtocol_IOMCU` renumber skipped, `"Q30"` OSD padding skipped | `SerialProtocol_Q30` was deleted in V5.0.6; the renumber only broke param compatibility |
| 4 `pmucan_*.hpp` deleted (1163 lines) | vendored libuavcan v0; only 3 constants were used, all identical in `AP_HAL::CANFrame` / `CANIface` |
| Mount task kept at 50 Hz | `AP_Mount_Viewpro::update()` self-throttles to 100 ms, so gimbal traffic is 10 Hz either way; at a 10 Hz task rate the period *equals* the throttle and late ticks cause 5 Hz bursts. See D8 |
| msg 285 suppression made opt-in | V5.0.8 suppressed it for every gimbal, not just Viewpro. See D8 |
| `version.h` is DEV, not OFFICIAL | migration in progress; release will be 5.1.0 |
| `GCS.h` dead CAM macros removed, `OFP_VER_*` corrected to 5/0/8 | were stale/unused; guarded by `static_assert` in `Copter.cpp` |
| `AP_PMUCAN` fixes | dropped-frame at RX budget, dead branch, `&`->`&&`, `RXdrain()` extraction, named constants, ctor init, `TXspin` void. See git log |
| `send_proximity()` upward-distance block **not** commented out | V5.0.8 wrapped it in `/* Currently, no upward sensor */`. `AP_Proximity::get_upward_distance()` already returns false when no backend supplies one (`AP_Proximity.cpp:498`), and TeraRanger Tower Evo is not one of the backends that does - so the comment-out is a no-op here and a silent regression for `AP_Proximity_RangeFinder` / `_MAV` / scripting users. Not applied |
| OA sender/handler guarded `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` | `GCS_Common.cpp` is core; `AC_Avoid` and `AP_Proximity` are optional. Step 8's `UserCode.cpp` caller needs the same guard - without it a proximity-disabled build hits the `try_send_message()` default case, which spams "Sending unknown message" and panics in SITL |
| step-7 defect fixes in the `GCS_Common.cpp` block | see the table below |

**Step 7 defects fixed while applying `GCS_Common.cpp`.** All are in the V5.0.8 block; none
change the wire format of any ICD message except where stated.

| Defect | Fix |
|---|---|
| `CAM_ATTITUDE_STATUS.Yaw_REL_ANG = CAM_ATTITUDE_STATUS.Pitch_IMU_ANG = yaw * 10;` - copy-paste typo breaking the `X_REL = X_IMU` pattern of the two lines above it. `Pitch_IMU_ANG` carried **yaw**, and `Yaw_IMU_ANG` was never assigned, so TM2 reported it as 0 forever | assign `Yaw_IMU_ANG`. **Changes what KGCS receives** in two TM2 fields |
| `send_message_flcc_gcs_object_avoidance_status()` called `proximity->distance_min_m()`, `distance_max_m()` and `get_horizontal_distances()` **before** the `proximity == nullptr` check | null check moved to the top |
| `handle_gcs_flcc_object_avoidance_cmd()` dereferenced `AP::ac_avoid()` with no null check | added |
| TM12 logged `Load_Current` twice - the `ICMD` column got `Load_Current` instead of `Current_Control_Command` | log-only |
| TM13 labels `FLMJ,FLMN` were fed `Version_FLCC_Sub_Number, Version_FLCC_Main_Number` - major and minor swapped | log-only |
| `Zoom_POS_FB` cast `(int16_t)` into an `int8_t` ICD field | cast `(int8_t)` |
| `AP::logger().Write()` calls unguarded | wrapped in `#if HAL_LOGGING_ENABLED` |
| `GCS_DEBUG` / `GCS_DEBUG_PRINT` defined and never used; `extern PREV_CAM_CMD` declared and never used (it is only touched in `AP_Q30.cpp`) | removed |
| `get_horizontal_distances()` return value ignored, `dist_array.valid(i)` never consulted, sector loop hard-coded to `8` | gate on `sensor_failed()` + check the return + skip invalid sectors + `PROXIMITY_MAX_DIRECTION`. See **D12** |
| hard-coded 1000 / 1500 cm OA thresholds | `PNU_OA_ALERT_DISTANCE_CM` / `PNU_OA_WARN_DISTANCE_CM` - see D12 |
| tabs, commented-out `//Debug only - remove later` blocks, commented-out legacy angle-compare block | removed / converted to 4 spaces |

Left alone on purpose: the `warning_level |= 2` / `|= 1` then `> 2 -> 2` clamp (obscure but
correct - alert wins) and `MAV_SEVERITY_CRITICAL` on the avoidance on/off statustext
(operator feedback KGCS relies on).

**Remaining work:** step 8 (`UserCode.cpp`, `APM_Config.h`) and the two free-floaters
(`AP_NMEA_Output.{cpp,h}`, `config.h`) which have no dependencies and can land anywhere.

**Open issues** are the `PNU-ISSUE` table above; code markers carry the same ids.
List them with:

```
grep -rn "PNU-ISSUE" libraries/ ArduCopter/
```
