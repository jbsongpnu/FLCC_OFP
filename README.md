# Migration Break Point for Future Updates
- This branch will update Copter-4.7.1 to FLCC V5.0.8 step-by-step to find new commit break points for future updates.
- All essential works to make migration break points are done and currently reviewing and revising code for PNU-ISSUE
- IN THIS VERSION : More Updates to organize releasable verion 5.0.9 from 5.0.8
   - Issue D1 status flipped - reviewed but not closed - but Viewpro worked fine
   - Issue D2 is resolved 
   - Issue D3 is resolved : Do not go back to previous version. This change is critical. See notes.
   - Viewpro related instructions are added as "bench findings" in this README.md
- PNU-KAL Specific options  
   - CAN Driver Option for PMU : CAN_D1_PROTOCOL = 15  
   - KGCS telemetry port : SERIALx_PROTOCOL = 2 (MAVLink2) - **never 1**. Every PNU-KAL ICD message is id >= 50001, which MAVLink1 cannot encode, so a port left on 1 carries no PMU/CAM telemetry and accepts no TC commands at all. See PNU-ISSUE D2  
- Viewpro Mount Specific options  
   - MNT1_TYPE = 11 (Viewpro) / CAM1_TYPE = 4 (Mount) / SERIALx_BAUD = 115 (115,200 bps) / SERIALx_PROTOCOL = 8 (Viewpro)  
- Gremsy Mount Specific options  
   - MNT1_TYPE = 6 (MAVLink-Gremsy) / CAM1_TYPE = 6 (MAVLinkCAMV2) / SERIALx_BAUD = 115 (115,200 bps) / SERIALx_PROTOCOL = 2 (MAVLink2)   

<Planned Commit Breaks and Migration Steps>

| # | Step | Files | Needs | Unblocks |
|:---:|---|---|---|---|
| 1 | MAVLink dialect | `Forced Submodule File/pnu_kal.xml`, `Tools/pnu/stage_mavlink_defs.sh`, `wscript` | — | 2, 4, 5, 7, 8 |
| 2 | Enum & ID registry | `ModeReason.h`, `AP_Logger.h`, `AP_Arming.h`, `AP_CAN.h`, `ap_message.h`, `GCS.h`, `AP_Arming.cpp` | 1 | 4, 5, 6, 7, 8 |
| 3 | Gimbal driver | `AP_Mount.{cpp,h}`, `AP_Mount_Backend.{cpp,h}`, `AP_Mount_Viewpro.{cpp,h}` | — | 5, 7 |
| 4 | PMU CAN stack / version.h | `AP_PMUCAN/` ×6, `AP_CANManager.{h,cpp}`, `wscript` (PMUCAN line only), `version.h` | 1, 2 | 6, 7 |
| 5 | Camera adapter | `AP_Q30/` ×2, `AP_SerialManager.cpp`, `wscript` (Q30 line only), `Copter.h` (include + `AP_Q30 q30`) | 1, 2, 3 | 7 |
| 6 | PMU failsafe | `events.cpp`, `Copter.h` (failsafe bit + decl), `Copter.cpp` (10 Hz call) | 2, 4 | — |
| 7 | GCS handler bodies | `GCS_Common.cpp`, `GCS.h` (D12 edge latch) | 1, 2, 3, 4, 5 | 8 |
| 8 | Object avoidance | `UserCode.cpp`, `APM_Config.h` | 1, 2, 7 | — |
| — | NMEA output | `AP_NMEA_Output.{cpp,h}` | — | **NOT APPLIED** - see file notes |
| — | Vehicle tuning | `config.h` | — | **NOT APPLIED** - see file notes |

<Deferred Issues>

Open decisions parked until a later step can validate them. Each has a `PNU-ISSUE(<id>)`
marker at the code site. List them all with:

```
grep -rn "PNU-ISSUE" libraries/ ArduCopter/
```

| ID | Issue | Site | Blocked until | Decision needed |
|:--:|---|---|:--:|---|
| D1 | **IR palette deferral - mechanism hardware-verified; two residual races.** Hardware test confirms the prelude &rarr; 100 ms &rarr; colour sequence works, including alongside `set_camera_source`. Two defects remain but are narrow races that normal operation will not reach: (a) `_image_sensor` is not re-checked at send time - it is assigned *only* from gimbal telemetry (`AP_Mount_Viewpro.cpp:313`; `set_camera_source()` never touches it), so it would have to change inside the 100 ms window; (b) the send result is ignored and the pending colour cleared regardless, which needs a UART txspace failure at that instant. Consequence of either is cosmetic - one dropped palette command the operator re-presses; nothing safety-relevant. The prelude-to-colour gap is always **100 ms** (`update()` self-throttles to `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS`), independent of the `AP_Mount` scheduler rate. Note: the `0x23`/`0x24` substitution in `AP_Q30` is a camera *vendor* change (newer units dropped the extended pseudo-colour options; `0x0E`, `0x0F`, `0x21`, `0x22` work, IR_RAINBOW offers Red only) - not evidence of deferral misbehaviour. | `AP_Mount_Viewpro.cpp` | — | **ANSWERED (2026-09-29): KGCS streams TC2.** Bench test confirmed the camera works with KGCS sending commands both intermittently and constantly, so the streamed case is real and the race is reachable. Per this row's own decision rule that means **apply the ~10-line fix** (snapshot `_image_sensor` + staleness deadline, clear the pending colour only on success) - D1 can no longer be closed as unreachable. Add to the same fix: `IR_operation()` has **no change detection** on the palette commands (unlike zoom, which guards on `prev_EO_zoom_cmd` / `prev_IR_zoom_cmd`), so a *held* `Tracking_CMD` while streaming re-fires `IR_Color_Change()` every cycle and overwrites the pending-colour slot continuously. Confirm from `TC_C.TRAK` whether KGCS holds the palette value or returns it to 0; palette changes currently work, which suggests momentary. Still untested: whether the deferral is *needed* at all - that requires removing the prelude and retesting. |
| D2 | All 7 PNU-KAL ICD messages are id ≥ 50001, so MAVLink **v2 only**. A link set to `SERIALn_PROTOCOL=1` (MAVLink1) silently carries no PNU-KAL telemetry or commands. | `Forced Submodule File/common.xml` | — | **RESOLVED (2026-09-29) - documented as a setup constraint**, see the header. A runtime warning was designed (gate the four PNU cases in `try_send_message()` on the existing `sending_mavlink1()`, warn once per channel via STATUSTEXT, which is id 253 and so does reach a MAVLink1 GCS) and **rejected as unnecessary**: a MAVLink1 link means *no* PMU messages at all, which is self-evident at the GCS; the port parameter is checked regardless; and PNU's offline preflight procedure already fixes the protocol. No code change. Verified mechanism, so nobody need re-investigate: the failure is a clean drop, not corruption - `mavlink_helpers.h:339` refuses msgid > 255 and counts a parse error, so a misconfigured link also shows a rising parse-error count. Such a channel never self-upgrades either, since the auto-upgrade in `packetReceived()` (`GCS_Common.cpp:1904`) requires the configured protocol to already be MAVLink2 |
| D3 | The `modules/mavlink` dialect edits are invisible to git — a `submodule update` or fresh clone silently reverts them and the build fails at step 7 with ~21 unknown-type errors. | `Forced Submodule File/ReadMe.txt` | — | **RESOLVED (2026-09-29).** The submodule is now left **completely pristine** - the definitions are staged outside it and the build runs from the staged copy. See the section below |
| D4 | `OFP_VER_MAIN/SUB/REV` in `GCS.h` and `FW_MAJOR/MINOR/PATCH` in `version.h` are two hand-maintained copies of the same version with nothing enforcing agreement. | `GCS.h`, `version.h` | 4 | **RESOLVED (step 4)** - `static_assert` in `Copter.cpp` (only TU seeing both; `AP_PMUCAN.cpp` cannot include the vehicle `version.h`) |
| D5 | **OPEN - not started.** 357 of `GCS_Common.cpp`'s 429 added lines are one block appended at EOF — the main recurring merge cost on every future ArduPilot bump, and the only issue here whose cost grows with time rather than staying flat. | `GCS_Common.cpp` | — (was 7) | **Not done.** Step 7 cleared the prerequisite only - the block now exists to be moved; `libraries/GCS_MAVLink/GCS_PNU.cpp` has not been created. Extract the 357-line EOF block there: it is self-contained and touches no `GCS_Common.cpp` statics. **72 lines must stay behind** - the 4 `try_send_message()` cases, 3 `handle_message()` cases, 4 id-map entries and 2 includes - and that is the residual per-bump merge cost. Worth doing *before* the D12 proximity rework, so that rework lands in a file that does not re-conflict on every upstream merge |
| D6 | `AP_PMUCAN::handleFrame()` performs no DLC validation - each case `memcpy`s from fixed offsets up to `data[7]` whatever `can_rxframe.dlc` says. No out-of-bounds read (`data[]` is fixed 8 bytes), but a short or malformed PMU frame is parsed silently and produces stale values for battery current, RPM, fuel quantity etc. | `AP_PMUCAN.cpp` | — | Confirm expected DLC per PMU message ID in the ICD, then reject or pad short frames |
| D7 | **Engine ON/OFF interlock is satisfied by inactivity.** `engineonoffstate()` counts to 10 at ~10 Hz, but `_pmu_ctrl_cmd`/`_pmu_ctrl_cmd_prv` are sticky between TC1 messages. If KGCS sends TC1 only on operator action, two messages (e.g. 3 then 2) freeze a valid pair and the counter climbs unattended - **engine STARTS after ~1 s of silence**. If KGCS streams TC1 with a held value, `prv == cmd` resets every tick and the **engine can never be commanded OFF**. Also rate-dependent (pairs overwritten above ~10 Hz) and loss-sensitive (a dropped TC1 resets progress). | `AP_PMUCAN.cpp` | — | Bench-test the matrix below, then re-specify the interlock (edge-latched count, or an explicit hold-duration) |
| D8 | **Mount scheduler rate vs. the Viewpro self-throttle.** (a) *RESOLVED (step 5)* - msg 285 suppression is now an opt-in backend capability (`suppress_gimbal_device_attitude_status()`), so only Viewpro suppresses it; every other gimbal keeps standard MAVLink behaviour. (b) *RESOLVED (step 5)* - V5.0.8 lowered the `AP_Mount` task 50&rarr;10 Hz; **deliberately reverted to 50 Hz**. `AP_Mount_Viewpro::update()` self-throttles to `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS` (100 ms, upstream), so gimbal traffic is 10 Hz at *either* scheduler rate - the scheduler does not control it. At 10 Hz the scheduler period **equals** that throttle interval, so any late tick defers the update a full period (200 ms &rarr; 5 Hz bursts); 50 Hz oversamples it 5x (worst case 120 ms). 50 Hz also keeps `AP_Mount_Backend::update()` - servo retract and `update_poi_lock_target()`, which run *before* the throttle - at full rate. | `Copter.cpp` | — | Done. To reduce gimbal traffic, raise `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS`, not the scheduler rate |
| D9 | **PMU failsafe: no-PMU case is correct; one gap fixed, one open.** Both "no PMU" paths correctly avoid a false trigger (`PMUCAN_Fail` stays 2 when the driver runs with no PMU; stays 0 from static init when `CAN_Dn_PROTOCOL` is not 15). (a) *RESOLVED (step 6)* - `failsafe.pmucan` was set but never cleared, so the failsafe could fire at most once per power cycle. It now clears on recovery with `ERROR_RESOLVED`, matching `failsafe.terrain` / `.deadreckon` / `.ekf`. The mode change is deliberately **not** undone, matching upstream precedent. (b) **OPEN** - once the PMU has been seen and then lost, state is 1 (`COMMUNICATION_ERROR`) with no path back to 2 and no disable parameter, so **emergency takeoff after a PMU dropout is blocked**. Worse, PMUCAN has *no prearm check* (`AP_Arming.cpp:1354` is a bare `break`), so prearm passes silently, arming succeeds, and `should_disarm_on_failsafe()` disarms ~100 ms later with no prior warning. | `events.cpp`, `AP_Arming.cpp` | — | Design agreed, **dedicated commit after the migration** - it changes flight-safety behaviour and should not be folded into a migration step. No new parameter. See design below. |
| D10 | **`AP_Q30` routes every camera function through `AP::mount()`, never `AP::camera()`.** Valid for Viewpro (one serial protocol carries gimbal + camera), but on any other mount all ~27 camera calls fall through to the base class and silently do nothing, and `get_zoom_times()` returns a fabricated `0.0f`. Dormant if the camera is driven by standard MAVLink2 instead of KGCS TC2. | `AP_Q30.cpp`, `UserCode.cpp` | 8 | Decide whether KGCS TC2 must drive non-Viewpro cameras; if so, route camera calls via `AP::camera()` - but see **D11**, the destination cannot do everything. See details below. |
| D11 | **Even with correct routing (D10), the MAVLink camera backend cannot cover everything KGCS needs.** Pristine 4.7.1 *does* have a MAVLink camera path (`AP_Camera_MAVLinkCamV2`, `CAM1_TYPE=6`) - it is the *mount* that is gimbal-only. Of `AP_Q30`'s 8 camera functions, 4 work, 2 exist only in the `AP_Camera` base, and 2 have **no path at all**: `get_zoom_times` (the backend never decodes `CAMERA_SETTINGS` msg 260, where `zoomLevel` lives) and `IR_Color_Change` (MAVLink has no standard thermal-palette message). | `AP_Camera_MAVLinkCamV2.cpp` | — | Bench a real VIO first; then decide per function - upstream fix, local fix, or vendor-specific. See details below. |
| D12 | **TM5 could not distinguish "nothing nearby" from "sensor reporting nothing".** *RESOLVED (step 7), decision provisional - see revisit note below.* The V5.0.8 code ignored the return of `get_horizontal_distances()`, which fills every sector with `dist_max` on failure (`AP_Proximity_Boundary_3D.cpp:434`) and reads back as "no object" - so a dead sensor was byte-identical to open sky. Sector scanning is now gated on `sensor_failed()`, the return value is checked, and `dist_array.valid(i)` excludes sectors that never reported. | `GCS_Common.cpp` | — | **Decided for now: no ICD change** (provisional - PNU will revisit). `Object_Avoidance_Status = 3` ("sensor unhealthy") was considered and **rejected** - it would need a KGCS update, and the failure already reaches KGCS two other ways. TM5 stays all-zero when the sensor is dead; the health signal is the `MAV_SEVERITY_CRITICAL` statustext on the healthy&rarr;failed edge (plus one if avoidance is switched on while already failed) and the `MAV_SYS_STATUS_SENSOR_PROXIMITY` bit, which `GCS_Copter.cpp:67` drives from the *same* `sensor_failed()` predicate - so TM5 and `SYS_STATUS` cannot contradict each other. **Residual:** confirm 10 m / 15 m are the intended operator thresholds (`PNU_OA_ALERT_DISTANCE_CM` / `PNU_OA_WARN_DISTANCE_CM`); they are hard-coded and unrelated to `AVOID_MARGIN` |
| D13 | **The KGCS camera path hard-depends on `HAL_MOUNT_ENABLED` with no guard.** `AP::mount()` is declared only inside `#if HAL_MOUNT_ENABLED` (`AP_Mount.h`), but `AP_Q30.cpp` calls it 5x with no guard and `AP_Q30.h` has no `#if` at all; `GCS_Common.cpp::send_message_gcs_flcc_cam_status()` and now `UserCode.cpp`'s 10 Hz `MSG_CAM_STATUS` send sit on the same chain. `HAL_MOUNT_ENABLED` defaults to 1 so CubeOrangePlus is unaffected, but it is a `build_options.py` feature - a custom build with MOUNT disabled fails to compile, not gracefully degrade. Contradicts the AGENTS.md rule that a core component must not depend on an optional one. | `AP_Q30.{h,cpp}`, `GCS_Common.cpp`, `UserCode.cpp` | — | Wrap the `AP_Q30` class body and every CAM call site in `#if HAL_MOUNT_ENABLED`, or give `AP_Q30` its own `AP_Q30_ENABLED` flag defaulting to `HAL_MOUNT_ENABLED` and add it to `build_options.py`. Cheap and self-contained; deferred only to keep step 8 to the OA change. Verify with a `HAL_MOUNT_ENABLED=0` build, not by inspection |


**PNU-ISSUE D3 - resolved 2026-09-29.** `modules/mavlink` is left **completely pristine**.

*The problem:* the parent repo cannot track submodule file contents, so any edit inside
`modules/mavlink` is invisible to git and is silently reverted by `git submodule update`,
a fresh clone or a submodule bump.

*How it used to be:* the 7 PNU messages were appended to the submodule's `common.xml`,
which meant keeping a full **7579-line copy** of `common.xml` in `Forced Submodule File/`
and overwriting the upstream file with it - so every ArduPilot bump would have **silently
reverted upstream's `common.xml` changes**. `cubepilot.xml` also had 5 ids renumbered
50001-50005 &rarr; 70001-70005 to dodge an id collision.

*How it works now:* nothing inside the submodule is touched. The definitions are staged
beside it and the build reads the staged copy.

| Piece | Tracked? | Role |
|---|:--:|---|
| `Forced Submodule File/pnu_kal.xml` | yes | the 7 message definitions |
| `Tools/pnu/stage_mavlink_defs.sh` | yes | copies the 19 upstream XMLs + `pnu_kal.xml` into the staging dir, drops `cubepilot.xml`, stamps the source commit |
| `Tools/pnu/mavlink_defs/` | **no - gitignored** | the staged copy (~944 KB), regenerated, never committed |
| `wscript` (1 line) | yes | builds from `Tools/pnu/mavlink_defs/all.xml` |
| `modules/mavlink` | — | **pristine** |

```
Tools/pnu/stage_mavlink_defs.sh            # stage; safe to re-run
Tools/pnu/stage_mavlink_defs.sh --check    # exit 1 if missing or stale
```

`cubepilot.xml` is simply not staged, rather than renumbered: its ids collide with the
PNU-KAL ICD, nothing in ArduPilot references `CUBEPILOT_*` or `HERELINK_*`, and PNU uses
no Herelink or CubePilot raw RC. It must be dropped from **both** `all.xml` (the build
root) and `ardupilotmega.xml`, which includes it directly as well.

**The staging dir must never be committed.** Tracking it would put a ~944 KB duplicate of
upstream in the repo that rots on every bump - precisely the `common.xml` problem this
replaced. Only the transform is tracked.

**Re-run the script after any `modules/mavlink` change.** This is the one hazard the
staged approach introduces: a stale stage still contains the PNU messages, so it builds
happily against *last month's* upstream definitions with nothing to show for it. The stage
records the submodule commit in `.source-commit` and `--check` compares it against HEAD,
which is what catches this. Worth wiring into CI.

*Three failure paths, all reported clearly - each one verified by reproducing it:*

| Failure | What you get |
|---|---|
| nothing staged (fresh clone) | waf stops: `Tools/pnu/mavlink_defs/all.xml is missing - run Tools/pnu/stage_mavlink_defs.sh` |
| stale stage | `--check` exits 1, printing both commits |
| staged but PNU messages absent | compile stops at `GCS.h`: `PNU-KAL MAVLink dialect missing` |

That last guard has to live in `GCS.h`, not `AP_Q30.h` - `GCS.h` declares the PNU message
types itself (step 2) and is compiled first, so a guard placed later never fires. With
`-Wfatal-errors` it is then the only error reported.

The waf-level check is not optional: without it, a missing staging dir fails with
`TypeError: 'NoneType' object is not iterable` out of waf's node resolution, which says
nothing about the cause.

*Why not keep the message definitions in the submodule and just script the re-apply?*
That was implemented first and worked, but it still needed 3 lines inside the submodule,
which vanish silently on every update. *Why not point `wscript` straight at the submodule's
`all.xml` plus a PNU root file?* Tested: mavgen resolves the external root correctly, but
`ardupilotmega.xml` includes `cubepilot.xml` itself, so the id collision returns and cannot
be avoided without either editing the submodule or renumbering the three colliding PNU ids
(50001, 50002, 50004), which is an ICD change requiring KGCS work.

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

**The pitch limits are in KGCS convention, not physical.** `set_angle_target()`
constrains *before* it negates, so `MNT1_PITCH_MIN` / `MNT1_PITCH_MAX` bound the
**commanded** value, and the achievable physical range is their negation:

```
MNT1_PITCH_MAX = degrees of DOWN travel allowed   (positive)
MNT1_PITCH_MIN = -(degrees of UP travel allowed)  (negative)
```

The stock defaults (`MIN -90`, `MAX 20`) therefore give only 20 deg of look-down and
90 deg of look-up on a Viewpro - backwards for a camera. Symmetric limits are **not** a
safe shortcut: the travel is asymmetric, and `MIN -90` commands 90 deg up, past the
mechanical stop, which saturates the gimbal and raises an internal error.

Measured on the bench (Viewpro, 2026-09-29): up saturates near **60 deg physical**
(commanded -60), so `MNT1_PITCH_MIN` must stay at or above -60.

**Final values, bench-tested 2026-09-29:**

| Parameter | Value | Gives |
|---|:--:|---|
| `MNT1_PITCH_MAX` | **90** | 90 deg **down** - full nadir |
| `MNT1_PITCH_MIN` | **-45** | 45 deg **up** |

The -45 is a deliberate ~15 deg margin inside the measured 60 deg up stop - do not
"optimise" it to -60, that is the saturation point where the gimbal raises an internal
error.

Note the limits apply to the **angle** path only. Viewpro declares
`NATIVE_ANGLES_AND_RATES_ONLY`, so rate commands go straight to
`send_target_rates()` and are never constrained - only the gimbal's own firmware
stops them. The RC path applies the same parameters in *physical* convention with no
flip, so RC and KGCS disagree unless the limits are symmetric.

Any other gimbal (Gremsy etc.) inherits `false` from the base class and is
unaffected.



<Viewpro bench findings - 2026-09-29>

Hardware: **two Viewpro units, old and new, behave differently.** On the *old* camera,
KGCS rate (TC2 `Control_Mode` 1) pitch saturated after ~4 deg of travel while yaw was
normal; on the *new* camera the same firmware and parameters work correctly. The
firmware path is symmetric between the axes - same function, same A1 packet, same
`x100` scale, only the sign differs (`AP_Mount_Viewpro.cpp:485-486`) - and a diff of the
whole rate path against `origin/A0_KAL_HD_FC_Based4.6.2` (head `486548e8f3`, "V4.0.8")
found `AP_Q30::send_cmd_speed()`, the TC2 handler and the rate encoding byte-identical.
So the fault is in the old gimbal, not in this tree.

**Record the model name and firmware version of both units.**
`AP_Mount_Viewpro.cpp:275` prints the model as a statustext at boot and line 262 parses
the firmware version. That string is the only field-identifiable difference between a
unit that works and one that does not, and it is the same precedent
`AP_Mount_MAVLink.h:51` uses for branching on vendor/model.

| Function | TC2 trigger | Status |
|---|---|---|
| Gimbal angle (`Control_Mode` 2) | `Pitch/Yaw_Angle_CMD` | works; limits as documented above |
| Gimbal rate (`Control_Mode` 1) | `Pitch/Yaw_Speed_CMD` | works on the new camera; **fails on the old one** |
| EO / IR / PIP source | `Tracking_CMD` 10-13 | works |
| IR colour palette | `Tracking_CMD` 14-19 | works - but see **D1**, no change detection |
| Zoom, EO and IR | `Tracking_CMD` 20-24 | works |
| Record start / stop | `Shutter_CMD` 1 / 2 | works |
| Take picture | `Shutter_CMD` 4 | works |
| Tracking start / stop | `Tracking_CMD` 1 / 2 | works |
| Focus in / out | `Zoom_Focus_Stop_CMD` 3 / 4 | **confirmed no-op** - camera is always autofocus and KGCS no longer sends it. Left in place: the TC2 field stays in the ICD either way, `CM_C.FOCS` simply reads 0. Do not re-test |

**Tracking status comes from the camera, not from KGCS.** `_last_tracking_status` is
written in exactly one place - parsing bits 3-4 of the gimbal's `T1_F1_B1_D1` telemetry
frame (`AP_Mount_Viewpro.cpp:290`). `set_tracking()` only *asks* the camera to start;
the camera reports `STOPPED` / `SEARCHING` / `TRACKING` / `LOST` back, and each
transition emits a statustext.

> **Operational consequence, still untested.** `AP_Mount_Viewpro::update()` returns early
> while the status is `SEARCHING` **or** `TRACKING`, sending the gimbal no angle, rate or
> hold target at all. Because `SEARCHING` is camera-driven, that freeze can begin with no
> KGCS action - e.g. the camera re-acquiring after losing a target - and the operator
> loses gimbal control with only a statustext to explain it. `LOST` is *not* in the freeze
> condition, so control returns on a lost target. **Test a target-lost / re-acquire cycle
> and confirm the operator is never stranded without gimbal control.**

**Rate commands expire after 3 s.** 4.7.x added `mnt_target.last_rate_request_ms` and a
3000 ms timeout that zeroes all three rate axes (`AP_Mount_Backend.cpp:1192`); 4.6.2 had
no such timeout, so a rate persisted until replaced. This was *not* the cause of the old
camera's fault, but it is a real behavioural difference from V4.0.8: KGCS must keep
sending TC2 while a rate is commanded. Confirmed harmless in practice - KGCS streams.

Still to measure: whether `_Max_zoom_EO = 30` / `_Max_zoom_IR = 4` (`AP_Q30.h:89-90`, hard-coded, they
clamp the absolute-zoom command) match the new camera's real maxima.

**TM2 reports a dead gimbal as perfectly level.** If `get_attitude_euler()` fails,
`send_message_gcs_flcc_cam_status()` zeroes all angles, so a gimbal that has stopped
reporting is indistinguishable from one pointing straight ahead - the same pattern as
**D12**. Decide whether KGCS should time out TM2.

**PNU-ISSUE D10 details** - step 8 is done; this is now free-standing work.

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

**Related residual - now LIVE as of step 8:** `UserCode.cpp::userhook_MediumLoop()` sends
`MSG_CAM_STATUS` unconditionally at 10 Hz. `send_message_gcs_flcc_cam_status()` guards
`AP::mount() == nullptr` but not whether the mount supports zoom - so with a Gremsy it
emits a 10 Hz TM2 stream reporting zoom 0x to every connected GCS. Gate it on the mount
actually supporting zoom, or on TC2 having been seen recently.

**If camera routing is added:** units differ across the boundary. `AP_Mount_Viewpro::set_zoom(PCT)`
was modified to take zoom *times* (`zoom_value * 10`), while `AP_Camera_MAVLinkCamV2::set_zoom(PCT)`
follows the MAVLink contract of 0-100 percent. A shared route must convert.



**D10 resolution plan, and what a Stage 1 prototype found (2026-09-28).**

A staged plan, cheapest first:

| Stage | What | Blocked on |
|:--:|---|---|
| 0 | Ask PNU: **must KGCS TC2 drive non-Viewpro cameras at all?** If no, D10 closes by rejecting TC2 camera commands on a non-Viewpro mount and gating the TM2 stream - roughly 20 lines, no refactor | one question to PNU |
| 1 | Stop fabricating values: give `get_zoom_times()` an error signal, and check the ~26 unchecked camera return values | nothing |
| 2 | Dual-dispatch in `AP_Q30` - try `AP::mount()`, fall back to `AP::camera()` | stage 0 answer |
| 3 | The two functions with no `AP_Camera` path at all | **D11** / the PNU action item |

**Stage 1 was prototyped on 2026-09-28 and reverted** - it is correct but touches 8 files
and ~26 call sites, which is too broad to carry alongside the migration. Deferred, not
rejected. Findings worth keeping, so they need not be rediscovered:

- **`AP_Mount` and `AP_Camera` are signature-compatible for 6 of the 8 camera functions** -
  same names, same `ZoomType` / `FocusType` / `TrackingType` / `SetFocusResult` enums, same
  `bool` returns. Stage 2 is therefore mechanical forwarding, not a redesign. Only
  `set_camera_source` differs (`uint8_t` vs a `CameraSource` enum).
- `get_zoom_times()` has only **three callers in the whole tree** (`AP_Q30.cpp` x2,
  `GCS_Common.cpp` x1), so changing its signature is cheap.
- `AP_Mount::get_zoom_times()` contains `return false;` inside a `float` function - a second
  fabricated `0.0f`, on the no-backend path.
- `AP_Mount_Viewpro::_zoom_times` has **no initialiser** and the class uses an inherited
  constructor, so before the first gimbal report it is *indeterminate*, not 0. Any
  "is this value real?" test must account for that. EO and IR zoom are both >= 1x, so
  `is_positive()` is a usable validity test once it is initialised.
- There is **not one `if (mount->...)` in `AP_Q30.cpp`** - 26 camera calls, no return checked.
- TM2's `Zoom_POS_FB` has no "unknown" encoding, exactly like TM5 in D12, so a zoom-truth fix
  cannot change what goes on the wire without an ICD decision.
- `EO_zoom_pct` is constrained to `[1, _Max_zoom_EO]` with `_Max_zoom_EO = 30.0`: it carries
  zoom **times** while being passed as `ZoomType::PCT`, and only works because
  `AP_Mount_Viewpro::set_zoom()` was modified to reinterpret PCT as times. **This is the trap
  in Stage 2** - forwarding it to `AP_Camera_MAVLinkCamV2::set_zoom(PCT)`, which honours the
  MAVLink 0-100 % contract, turns 10x into 10 %, zoomed out instead of in. Rename the
  variable and convert at the boundary as a separate commit before any routing change.

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
| `ArduCopter/UserCode.cpp` | `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` guard around the OA block |
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
| step 8: OA block in `userhook_MediumLoop()` guarded `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` | `avoid` is itself `#if AP_AVOIDANCE_ENABLED` (`Copter.h:511`), and step 7 put `MSG_OBJECT_AVOIDANCE_STATUS` behind the same guard. Without it a proximity-disabled build hits the `try_send_message()` default case, which sends "Sending unknown message" and panics in SITL |
| step 8: `copter.avoid.` &rarr; `avoid.` | inside a `Copter` member function; `copter.` is the global instance and redundant |
| step 8: tabs &rarr; 4 spaces in the added lines | V5.0.8 mixed tabs and spaces in both hunks |

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

**Step 8 notes.**

- `userhook_MediumLoop()` runs at 10 Hz with a 75 us budget (`Copter.cpp:267`). Both calls
  only set bits in the per-channel deferred-message mask, so the cost is trivial and the
  actual sending stays in the GCS task.
- `USERHOOK_INIT` fires from `system.cpp:132`, long after `GCS::_singleton` is set in the
  `GCS` constructor (`GCS.h:1153`), so the `gcs()` writes there are safe. The two fields it
  zeroes are already zero from static init - it is documentation, not a fix.
- `Object_Avoidance_Mode` is recomputed every tick from `proximity_avoidance_enabled()`,
  which is `_proximity_enabled && (AVOID_ENABLE & AC_AVOID_USE_PROXIMITY_SENSOR)`. **So with
  `AVOID_ENABLE` bit 1 clear, KGCS gets "Avoidance ON Level n" from the TC4 handler but TM5
  keeps reporting mode 0.** That is correct - TM5 reports what is in force, not what was
  asked for - but it is a confusing setup trap worth checking on the bench.
- `gcs().send_message()` broadcasts to **every** channel, so a second GCS on another link
  also receives TM2/TM5 (~520 B/s combined). Harmless - MAVLink ignores unknown ids - but it
  is bandwidth spent on a link that cannot use it.

**Free-floaters - reviewed, deliberately NOT applied.** Both carry an in-file note:

```
grep -rn "PNU-NOT-APPLIED" libraries/ ArduCopter/
```

| File | V5.0.8 change | Why not applied |
|---|---|---|
| `ArduCopter/config.h` | `LAND_DETECTOR_ACCEL_MAX` 1.0f &rarr; 3.0f | Probably needed for gas-engine vibration, but it relaxes a safety threshold past upstream's own WoW-corroborated `land_detector_scalar = 2`. Belongs in the board hwdef, not the vehicle source, and needs flight-log evidence first |
| `libraries/AP_NMEA_Output/*` | Korean drone-ID / UTM rewrite | Comments out `HAL_NMEA_OUTPUT_ENABLED` in the `.cpp` only &rarr; fails to build wherever the feature is off; deletes GPRMC and PASHR but keeps their bits; passes a double to `%d`; hardcodes the station ID and the 1 Hz rate; non-standard VTG. Re-specify as separate PNU work if needed |

**Open question for the NMEA decision:** is any serial port on the aircraft set to
`SERIALn_PROTOCOL = 20` (NMEAOutput)? If not, this is dead code and the issue closes.

**Remaining work:** all 8 numbered steps are applied, and both free-floaters are closed as
not-applied. Left over: the open issues D1, D2, D3, D5, D6, D7, D9(b), D10, D11, D12 (provisional) and D13.

**Open issues** are the `PNU-ISSUE` table above; code markers carry the same ids.
List them with:

```
grep -rn "PNU-ISSUE" libraries/ ArduCopter/
```
