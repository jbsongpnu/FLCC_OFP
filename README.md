# PNU-KAL FLCC OFP Development Version
- This branch currently developing PNU-KAL FLCC OFP after official version (TAG) PNU-KAL-FLCC-5.1.0 (this is same code as 5.0.15 dev)  
- PNU-ISSUE 10 & 11 still remains : For Gremsy camera and general camera mount library structure  
- IN THIS VERSION : V5.1.1 working with issues 10 and 11 - Stage 1 
   - Stage 1 applied : (see below); the routing decision itself stays blocked on the ACTION ON PNU answers  
   - README.md notes are updated to indicate dev version  
***
Residuals recorded against resolved issues

- D1 — is the IR prelude needed at all? Requires removing it and retesting on hardware.  
- D6 — five PMU counters are still write-only; decide to surface (a PMUC log msg) or delete. _rtr_tx_err / _cmd_tx_err have the real diagnostic value.  
- D7 — accepted interlock residuals (no sender check on handle_gcs_flcc_pmu_ctrl(); the 2-messages-plus-1s shortcut rests on a KGCS contract not enforced in firmware).  
- D12 — confirm 10 m / 15 m are the intended operator thresholds.
- Untested, from the bench notes: the Viewpro SEARCHING/TRACKING freeze (operator can lose gimbal control with no KGCS action), and whether _Max_zoom_EO = 30 / _Max_zoom_IR = 4 match the new camera.  
***
- For 5.0.15 version :
   - Issue 9(b) is resolved : All issues except 10/11 are now resolved
   - Issue 10 and 11 is decided to be resolved after 5.1.0.
   - Building as "heli" model is blocked due to fatal errors it may cause 
- For 5.0.14 version :  
   - Issues 13 and 14 are resolved together
- For 5.0.13 version :
   - Issue 7 is resolved 
- For 5.0.12 version :
   - Issue 1 is resolved
- For 5.0.11 version :
   - Issue D6 is resolved 
   - Issue 15 was added during the anlysis of D6, but also resolved
- For 5.0.10 version :
   - Issue D5 is resolved but found new Issue D14 
- For 5.0.9 version :
   - Issue D1 status flipped - reviewed but not closed - but Viewpro worked fine
   - Issue D2 is resolved 
   - Issue D3 is resolved : Do not go back to previous version. This change is critical. See notes.
   - Viewpro related instructions are added as "bench findings" in this README.md
- PNU-KAL Specific options  
   - **WARNING :** Do not use heli model. Only multicopter frames are supported  
   - CAN Driver Option for PMU : CAN_D1_PROTOCOL = 15  
   - KGCS telemetry port : SERIALx_PROTOCOL = 2 (MAVLink2) - **never 1**. Every PNU-KAL ICD message is id >= 50001, which MAVLink1 cannot encode, so a port left on 1 carries no PMU/CAM telemetry and accepts no TC commands at all. See PNU-ISSUE D2  
- Viewpro Mount Specific options  
   - MNT1_TYPE = 11 (Viewpro) / CAM1_TYPE = 4 (Mount) / SERIALx_BAUD = 115 (115,200 bps) / SERIALx_PROTOCOL = 8 (Viewpro)  
- Gremsy Mount Specific options
   - **WARNING :** Gremsy is not supported yet
   - MNT1_TYPE = 6 (MAVLink-Gremsy) / CAM1_TYPE = 6 (MAVLinkCAMV2) / SERIALx_BAUD = 115 (115,200 bps) / SERIALx_PROTOCOL = 2 (MAVLink2)   
   - For more information with Gremsy camera, see online manual at https://docs.gremsy.com/payloads/vio

<Planned Commit Breaks and Migration Steps>

| # | Step | Files | Needs | Unblocks |
|:---:|---|---|---|---|
| 1 | MAVLink dialect | `Forced Submodule File/pnu_kal.xml`, `Tools/pnu/stage_mavlink_defs.sh`, `wscript` | — | 2, 4, 5, 7, 8 |
| 2 | Enum & ID registry | `ModeReason.h`, `AP_Logger.h`, `AP_Arming.h`, `AP_CAN.h`, `ap_message.h`, `GCS.h`, `AP_Arming.cpp` | 1 | 4, 5, 6, 7, 8 |
| 3 | Gimbal driver | `AP_Mount.{cpp,h}`, `AP_Mount_Backend.{cpp,h}`, `AP_Mount_Viewpro.{cpp,h}` | — | 5, 7 |
| 4 | PMU CAN stack / version.h | `AP_PMUCAN/` ×6, `AP_CANManager.{h,cpp}`, `wscript` (PMUCAN line only), `version.h` | 1, 2 | 6, 7 |
| 5 | Camera adapter | `AP_Q30/` ×2, `AP_SerialManager.cpp`, `wscript` (Q30 line only), `Copter.h` (include + `AP_Q30 q30`) | 1, 2, 3 | 7 |
| 6 | PMU failsafe | `events.cpp`, `Copter.h` (failsafe bit + decl), `Copter.cpp` (10 Hz call) | 2, 4 | — |
| 7 | GCS handler bodies | `GCS_PNU.cpp` (bodies), `GCS_Common.cpp` (3 hooks), `GCS.h` (D12 edge latch) | 1, 2, 3, 4, 5 | 8 |
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
| D1 | **RESOLVED (2026-09-30).** IR palette deferral: for colour codes >= `IR_COLOR_1` the driver sends an `IR_RAINBOW` prelude and defers the colour itself to a later `update()` tick, because the gimbal MCU services at most one C1 command per its own scheduler cycle. The mechanism was hardware-verified; three races remained. | `AP_Mount_Viewpro.{h,cpp}` | — | Bench test confirmed **KGCS streams TC2**, so the races were reachable and this row's own rule required the fix. Applied: (a) the display is **captured at defer time** into `_palette_pending_sensor`, so the colour reaches the sensor it was meant for even if gimbal telemetry changes `_image_sensor` inside the window; (b) the pending colour is **cleared only on a successful send**, so a momentarily full UART retries next tick instead of dropping the command; (c) a **400 ms deadline** bounds those retries so a stale colour is never sent late; (d) a duplicate request for the palette already in flight no longer restarts the sequence - relevant precisely because KGCS streams. Also fixed while here: `_palette_pending_color` had **no initialiser** in a heap-allocated class with an inherited constructor, so a non-zero value at boot would have made the first `update()` send a garbage `CameraCommand` to the gimbal; all three members now carry initialisers, with a comment saying they must keep them. **Still untested:** whether the deferral is needed at all - that requires removing the prelude and retesting on hardware |
| D2 | All 7 PNU-KAL ICD messages are id ≥ 50001, so MAVLink **v2 only**. A link set to `SERIALn_PROTOCOL=1` (MAVLink1) silently carries no PNU-KAL telemetry or commands. | `Forced Submodule File/common.xml` | — | **RESOLVED (2026-09-29) - documented as a setup constraint**, see the header. A runtime warning was designed (gate the four PNU cases in `try_send_message()` on the existing `sending_mavlink1()`, warn once per channel via STATUSTEXT, which is id 253 and so does reach a MAVLink1 GCS) and **rejected as unnecessary**: a MAVLink1 link means *no* PMU messages at all, which is self-evident at the GCS; the port parameter is checked regardless; and PNU's offline preflight procedure already fixes the protocol. No code change. Verified mechanism, so nobody need re-investigate: the failure is a clean drop, not corruption - `mavlink_helpers.h:339` refuses msgid > 255 and counts a parse error, so a misconfigured link also shows a rising parse-error count. Such a channel never self-upgrades either, since the auto-upgrade in `packetReceived()` (`GCS_Common.cpp:1904`) requires the configured protocol to already be MAVLink2 |
| D3 | The `modules/mavlink` dialect edits are invisible to git — a `submodule update` or fresh clone silently reverts them and the build fails at step 7 with ~21 unknown-type errors. | `Forced Submodule File/ReadMe.txt` | — | **RESOLVED (2026-09-29).** The submodule is now left **completely pristine** - the definitions are staged outside it and the build runs from the staged copy. See the section below |
| D4 | `OFP_VER_MAIN/SUB/REV` in `GCS.h` and `FW_MAJOR/MINOR/PATCH` in `version.h` are two hand-maintained copies of the same version with nothing enforcing agreement. | `GCS.h`, `version.h` | — (was 4) | **RESOLVED (step 4)** - `static_assert` in `Copter.cpp` (only TU seeing both; `AP_PMUCAN.cpp` cannot include the vehicle `version.h`) |
| D5 | **RESOLVED (2026-09-30).** The 356-line block appended at `GCS_Common.cpp`'s EOF was the main recurring merge cost on every ArduPilot bump, and the only issue here whose cost grew with time. It now lives in `libraries/GCS_MAVLink/GCS_PNU.cpp`. | `GCS_Common.cpp`, `GCS_PNU.cpp` | — | Done. `GCS_Common.cpp` vs pristine 4.7.1 went from **429 added lines in 7 hunks** to **45 in 3** - better than the 72 estimated, because the two includes and the extern/macro block moved out too. The three survivors are unavoidable: they insert into upstream's `mavlink_id_to_ap_message_id()` map, `handle_message()` and `try_send_message()` switches. Moved code is **byte-identical** - no logic touched. Costs **+136 bytes** of flash, since the handlers are no longer in the same translation unit as their callers |
| D6 | **RESOLVED (2026-09-30).** `AP_PMUCAN::handleFrame()` performed no DLC validation - every case `memcpy`d from fixed offsets up to `data[7]` whatever `can_rxframe.dlc` said. No out-of-bounds read (`data[]` is a fixed 8 bytes), but a short or malformed PMU frame was parsed silently and yielded stale values for battery current, RPM, fuel quantity etc. | `AP_PMUCAN.cpp` | — | **ICD confirms DLC 8 for all five status messages** (BATSTS, ENGSTS, AUX1STS, AUX2STS, VERSTS), which also confirms every field the code reads is present. **TX confirmed too**: 4 for all five commands (BATCTRL, ENGONOFF, ENGMANUAL, ENGPCL, ENGCHK), matching `PMUCAN_CMD_DLC`; all five route through the single `pmucan_cmd()` path, and the driver has only two TX paths in total. RTR is N/A in the ICD, so `pmucan_rtr()`'s 8 is unconstrained. No TX change needed. Frames shorter than `PMUCAN_STS_DLC` are now rejected before parsing, counted in `_short_frame_cnt`, and reported to the GCS as `PMUCAN: short frame 0x<id> dlc <n> (<count> dropped)`, rate limited to one message per 10 s with the first always reported. Unknown ids now return early rather than falling through the switch. **Still open - see D15** (byte order) and the note below on the write-only counters |
| D7 | **RESOLVED (2026-09-30) - one defect fixed, the rest analysed and accepted.** The engine ON/OFF interlock's counters are not a count of operator alternations: the pair they inspect only changes when a new TC1 arrives, but `engineonoffstate()` runs at ~10 Hz regardless, so a frozen valid pair keeps incrementing. They are a ~100 ms tick timer - threshold 10 means *~1.0 s after the first valid alternating pair*. | `AP_PMUCAN.cpp` | — | **Fixed:** the state no longer advances on a failed CAN send. It previously set `_engineonoffmode` unconditionally, so a send failure left the FC believing the engine was running while the PMU had never been told - and KGCS blocks Start from that point, leaving the operator only Stop to resolve a divergence they could not see. On failure the state now holds and the still-valid pair retries on the next tick. **Accepted, with the reasoning recorded at the code site:** the two-messages-plus-1 s shortcut is sound *only because KGCS never sends TC1 unsolicited*; threshold 10 is deliberate and must not be raised back to 14; loss tolerance and rate immunity both follow from the timer behaviour. See the KGCS behaviour and residuals below |
| D8 | **Mount scheduler rate vs. the Viewpro self-throttle.** (a) *RESOLVED (step 5)* - msg 285 suppression is now an opt-in backend capability (`suppress_gimbal_device_attitude_status()`), so only Viewpro suppresses it; every other gimbal keeps standard MAVLink behaviour. (b) *RESOLVED (step 5)* - V5.0.8 lowered the `AP_Mount` task 50&rarr;10 Hz; **deliberately reverted to 50 Hz**. `AP_Mount_Viewpro::update()` self-throttles to `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS` (100 ms, upstream), so gimbal traffic is 10 Hz at *either* scheduler rate - the scheduler does not control it. At 10 Hz the scheduler period **equals** that throttle interval, so any late tick defers the update a full period (200 ms &rarr; 5 Hz bursts); 50 Hz oversamples it 5x (worst case 120 ms). 50 Hz also keeps `AP_Mount_Backend::update()` - servo retract and `update_poi_lock_target()`, which run *before* the throttle - at full rate. | `Copter.cpp` | — | Done. To reduce gimbal traffic, raise `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS`, not the scheduler rate |
| D9 | **RESOLVED (2026-09-30) - both parts.** Both "no PMU" paths correctly avoid a false trigger (`PMUCAN_Fail` stays 2 when the driver runs with no PMU; stays 0 from static init when `CAN_Dn_PROTOCOL` is not 15). (a) *step 6* - `failsafe.pmucan` was set but never cleared, so the failsafe could fire at most once per power cycle. It now clears on recovery with `ERROR_RESOLVED`, matching `failsafe.terrain` / `.deadreckon` / `.ekf`. The mode change is deliberately **not** undone, matching upstream precedent. (b) the failsafe could not tell "PMU lost in flight" from "armed without one", so an emergency launch after a PMU dropout was disarmed ~100 ms after arming, with no prearm warning. | `events.cpp`, `AP_Arming_Copter.{h,cpp}`, `Copter.h` | — | **(b) implemented as designed, no new parameter.** `AP_Arming_Copter::arm()` latches `failsafe.pmucan_armed_healthy = (PMUCAN_Fail == 0)`, and `failsafe_pmucan_check()` returns early unless it is set - so losing the PMU in flight still triggers the failsafe, while arming deliberately without one flies on unmolested. `AP_Arming_Copter::pmucan_checks()` warns before arming as a `Check::SYSTEM`, bypassable via `ARMING_SKIPCHK` bit 13 (System, decimal 8192). **One deviation from the written design:** it lives in `AP_Arming_Copter`, not `AP_Arming::can_checks()` - `AP_PMUCAN` is listed only in `ArduCopter/wscript`, so referencing `PMU_Ctrl_Echo` from the shared library would fail to link for Plane, Rover and Sub |
| D10 | **`AP_Q30` routes every camera function through `AP::mount()`, never `AP::camera()`.** Valid for Viewpro (one serial protocol carries gimbal + camera), but on any other mount all 27 camera calls fall through to the base class and do nothing. **Stage 1 applied 2026-10-01** - they no longer do it *silently*, and `get_zoom_times()` no longer fabricates `0.0f`. Dormant if the camera is driven by standard MAVLink2 instead of KGCS TC2. | `AP_Q30.{h,cpp}`, `AP_Mount*.{h,cpp}`, `GCS_PNU.cpp`, `GCS.h` | — (was 8; step 8 applied). **The routing decision is blocked on the ACTION ON PNU answers below**, not on any step | Decide whether KGCS TC2 must drive non-Viewpro cameras; if so, route camera calls via `AP::camera()` - but see **D11**, the destination cannot do everything. See details below. |
| D11 | **Even with correct routing (D10), the MAVLink camera backend cannot cover everything KGCS needs.** Pristine 4.7.1 *does* have a MAVLink camera path (`AP_Camera_MAVLinkCamV2`, `CAM1_TYPE=6`) - it is the *mount* that is gimbal-only. Of `AP_Q30`'s 8 camera functions, 4 work, 2 exist only in the `AP_Camera` base, and 2 have **no path at all**: `get_zoom_times` (the backend never decodes `CAMERA_SETTINGS` msg 260, where `zoomLevel` lives) and `IR_Color_Change` (MAVLink has no standard thermal-palette message). | `AP_Camera_MAVLinkCamV2.cpp` | — | Bench a real VIO first; then decide per function - upstream fix, local fix, or vendor-specific. See details below. |
| D12 | **TM5 could not distinguish "nothing nearby" from "sensor reporting nothing".** *RESOLVED (step 7), decision provisional - see revisit note below.* The V5.0.8 code ignored the return of `get_horizontal_distances()`, which fills every sector with `dist_max` on failure (`AP_Proximity_Boundary_3D.cpp:434`) and reads back as "no object" - so a dead sensor was byte-identical to open sky. Sector scanning is now gated on `sensor_failed()`, the return value is checked, and `dist_array.valid(i)` excludes sectors that never reported. | `GCS_Common.cpp` | — | **Decided for now: no ICD change** (provisional - PNU will revisit). `Object_Avoidance_Status = 3` ("sensor unhealthy") was considered and **rejected** - it would need a KGCS update, and the failure already reaches KGCS two other ways. TM5 stays all-zero when the sensor is dead; the health signal is the `MAV_SEVERITY_CRITICAL` statustext on the healthy&rarr;failed edge (plus one if avoidance is switched on while already failed) and the `MAV_SYS_STATUS_SENSOR_PROXIMITY` bit, which `GCS_Copter.cpp:67` drives from the *same* `sensor_failed()` predicate - so TM5 and `SYS_STATUS` cannot contradict each other. **Residual:** confirm 10 m / 15 m are the intended operator thresholds (`PNU_OA_ALERT_DISTANCE_CM` / `PNU_OA_WARN_DISTANCE_CM`); they are hard-coded and unrelated to `AVOID_MARGIN` |
| D13 | **RESOLVED (2026-09-30).** The KGCS camera path hard-depended on `HAL_MOUNT_ENABLED` with no guard: `AP::mount()` is declared only inside `#if HAL_MOUNT_ENABLED`, but `AP_Q30` called it unguarded and `AP_Q30.h` had no `#if` at all, so a MOUNT-disabled build failed to compile rather than degrading. | `AP_Q30_config.h`, `AP_Q30.{h,cpp}`, `GCS_PNU.cpp`, `GCS_Common.cpp`, `Copter.h`, `UserCode.cpp`, `build_options.py` | — | New `AP_Q30_ENABLED` flag in `AP_Q30/AP_Q30_config.h`, defaulting to `HAL_MOUNT_ENABLED`, guards the library and all six call sites, and is registered in `build_options.py` (`Camera / Q30`, depends on `MOUNT`) so the custom build server can drop it. **Verified by building with `--define HAL_MOUNT_ENABLED=0`**: previously a compile failure, now succeeds at 1,653,036 B (67 KB smaller). The normal build is byte-identical at 1,720,608 B, so the guards cost nothing when enabled |
| D14 | **RESOLVED (2026-09-30).** The PNU globals were shared by hand-written `extern`s with nothing checking them against their definitions - 11 declarations across `GCS_PNU.cpp` and `ArduCopter/events.cpp`, with `PMU_Ctrl_Echo` declared independently in both. A type change at a definition would have compiled silently in every translation unit and become memory misinterpretation at link time. | `AP_Q30.h`, `AP_PMUCAN.h`, `GCS_PNU.cpp`, `events.cpp` | — | Each library now declares its own globals in its header, beside the definition: `AP_Q30.h` for `tracking_counter`, the six `debug_cam_*`, `PREV_CAM_CMD` and `CAM_ATTITUDE_STATUS`; `AP_PMUCAN.h` for `PMU_Status` and `PMU_Ctrl_Echo`. Both defining `.cpp` files include their own header, so the compiler now checks declaration against definition and a mismatch is a build error. All hand-written `extern`s removed |
| D15 | **RESOLVED (2026-09-30) - no code change; the implementation was already correct.** Every field in `handleFrame()` is read by `memcpy` straight into a native integer, which assumes the PMU transmits little-endian. | `AP_PMUCAN.cpp` | — | **The ICD confirms the PMU is little-endian (LSB first)**, which matches the STM32, so no byte swap is needed on read or write. Verified end to end: RX `memcpy` into native ints; the 24-bit `Engine_Hour_Count`, where LSB-first puts the three bytes in the uint32's low three and the explicit `uint32_temp = 0U` supplies the top byte; and TX `pmucan_cmd()`, which packs a `uint32` straight into the frame. ArduPilot has no big-endian targets. Commented at all three sites so nobody "fixes" it with `be16toh`/`be32toh` - the pre-zero in particular looks removable and is not |


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


**PNU-ISSUE D6 note - the PMU counters are write-only.** `_handleFrame_cnt`, `_rtr_tx_cnt`,
`_cmd_tx_cnt`, `_rtr_tx_err` and `_cmd_tx_err` are incremented but **never read anywhere in
the tree** - they cost RAM and tell nobody anything. D6's new `_short_frame_cnt` was
deliberately not added in that style: it is reported to the GCS, because a rejection nobody
sees is the same silence D6 set out to remove.

Worth deciding, as a small follow-up: either surface the existing five (a `PMUC` log message
alongside the `TM1x` ones would be the natural place, since `AP_PMUCAN` does no logging at
all today) or delete them. `_rtr_tx_err` and `_cmd_tx_err` are the two with real diagnostic
value - they count CAN send failures, which is exactly what you would want when chasing an
intermittent PMU link.


**PNU-ISSUE D1 follow-up - `IR_operation()` has no change detection.** `AP_Q30::IR_operation()`
fires `IR_Color_Change()` on every TC2 whose `Tracking_CMD` names a palette, unlike zoom which
guards on `prev_EO_zoom_cmd` / `prev_IR_zoom_cmd`. With KGCS streaming, a **held** palette value
therefore re-requests it ~10x a second.

The D1 fix absorbs the harm - a duplicate request for the palette already in flight now returns
early instead of restarting the prelude - so this is traffic, not a defect.

**Adding change detection was considered and deliberately NOT done.** If KGCS *holds* the value,
change detection would suppress a re-press of the **same** palette, which is exactly the recovery
action D1's original note assumed the operator would take. The safe fix is the retry logic now in
place, not suppression. Before revisiting, settle the fact: **does KGCS hold `Tracking_CMD` at the
palette value, or return it to 0 after the button press?** `TC_C.TRAK` shows it directly - a held
value is a constant run, a momentary one a single sample. Palette changes work today, which
suggests momentary.

**PNU-ISSUE D7 - KGCS behaviour and accepted residuals.**

KGCS is a closed project, so the following is its *specified* behaviour supplied by PNU
(2026-09-30), not a bench measurement. A bench matrix is impractical for the same reason.

| | |
|---|---|
| When TC1 is sent | **only while a button is pressed** - never unsolicited |
| Start gesture | `Engine_OnOff` alternating **2/3 for 14 messages**, then stops |
| Stop gesture | `Engine_OnOff` alternating **4/5 for 14 messages**, then stops |
| Burst rate | constant, **~10 Hz, never above 11 Hz** |
| After a start | KGCS **blocks its Start button**; only Stop remains available |

*Why the timing works out.* The burst lasts ~1.4 s; the threshold of 10 fires at ~1.0 s,
around message 10 of 14. At the original threshold of 14 it landed at ~1.4 s - exactly
where the burst ends - which raced the FC's ~10 Hz sample clock against KGCS's ~10 Hz send
clock and left the outcome to drift. **Do not raise the threshold back to 14.**

*Why message loss does not matter.* A frozen pair keeps counting through a dropout, so a
lost TC1 neither aborts nor delays the gesture. This is the same mechanism as the timer
behaviour - it is not independent robustness.

*Why the ingest rate is not a problem.* TC1 at <=11 Hz against `TXspin()`'s 50 Hz sampling
means every message is seen at the sequence check; no alternation is discarded.

**Residuals, accepted rather than fixed** (deliberately not raised as separate PNU-ISSUEs):

- **The interlock is satisfied by two messages plus ~1 s of silence**, not by ten
  alternations. It is sound only because KGCS never sends TC1 unsolicited. That contract
  lives in a closed project and is not enforced anywhere in the firmware.
- **`handle_gcs_flcc_pmu_ctrl()` does not check the sender.** Any MAVLink source on any
  channel - a second GCS, a test tool, a log replay - can supply engine-start authority,
  against an effective bar of two messages and a second.
- **`_cmd_tx_err` is write-only.** The send-result fix now depends on it for diagnosis, so
  it is the strongest candidate in the D6 "PMU counters are write-only" note.

*If this is ever revisited*, the sound form is to count alternations on arrival, treat a
repeated value as **hold** rather than reset (so a dropped TC1 neither advances nor aborts),
and reset on staleness. 14 messages give 13 alternations against a threshold of 10, so up to
3 may be lost. It needs no KGCS change and is testable without KGCS or the engine: pymavlink
with `pnu_kal.xml` can emit TC1 bursts at any rate while you watch `Engine_OnOff_Echo` in TM3.


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

The other 27 are camera operations addressed to the mount object: `set_zoom` x7,
`IR_Color_Change` x6, `set_camera_source` x4, `set_focus` x2, `record_video` x2,
`set_tracking` x2, `get_zoom_times` x2, `take_picture`.

On a non-Viewpro mount (Gremsy is `AP_Mount_MAVLink`, which implements none of them)
every call hits the base-class default. **Before Stage 1, `AP_Q30` checked no return
value** - there was not one `if (mount->...)` guard in the file. All 27 are checked now,
which changes what the operator is told, not where the calls go:

| Method | Base-class return on a Gremsy |
|---|---|
| `set_zoom` / `record_video` / `take_picture` / `set_tracking` / `set_camera_source` / `IR_Color_Change` | `false` |
| `set_focus` | `SetFocusResult::UNSUPPORTED` |
| `get_zoom_times` | was **`0.0f`** - a fabricated value, not an error. **Stage 1 changed the signature to `bool get_zoom_times(uint8_t, float&)`**, so the base now returns `false` and writes nothing |

Result *before* Stage 1: gimbal pointing works, every camera command silently does
nothing, and KGCS is told zoom is 0x. No error, no warning, no build failure.

**After Stage 1 the behaviour is unchanged but no longer silent** - each declined call
is reported to the operator as `CAM: mount rejected <what> (<n> failed)`, and TM2 no
longer passes off "no zoom information" as a reading of 0x. The routing itself is
untouched: this makes the D10 symptom visible, it does not fix D10.

**Scope - when this does and does not matter:**

- *Standard MAVLink2 camera control* (`MAV_CMD_SET_CAMERA_ZOOM`, `MAV_CMD_IMAGE_START_CAPTURE`,
  `GIMBAL_MANAGER_*`) goes through `AP_Camera` / `AP_Mount` handlers and never enters
  `AP_Q30`. The issue is dormant. `AP_Q30` is passive - its constructor only sets
  `_singleton`, it registers no scheduler task, timer or thread, and its sole entry point
  is `handle_gcs_flcc_cam_cmd()` (`GCS_Common.cpp`), reached only by msg 50002 (TC2).
- *KGCS TC2 driving a non-Viewpro camera* is the case that breaks.

**Related residual - RESOLVED (Stage 1, 2026-10-01).** `UserCode.cpp::userhook_MediumLoop()`
sends `MSG_CAM_STATUS` unconditionally at 10 Hz, and `send_message_gcs_flcc_cam_status()`
guarded only `AP::mount() == nullptr` - so with a Gremsy it emitted a 10 Hz TM2 stream
reporting zoom 0x to every connected GCS.

Gating it *on the mount supporting zoom*, as this note originally suggested, would have
been **wrong**: TM2 is mostly gimbal attitude, which `AP_Mount_MAVLink` reports perfectly
well on a Gremsy even though it answers no camera call. Suppressing the whole message
would have thrown away working attitude telemetry to hide one bad field. Applied instead:
the sender returns early when `get_mount_type(0) == Type::None` - no gimbal configured, so
every field would be zero - and the zoom field is now reported honestly on its own (see
the TM2 entry in the Stage 1 table below). `UserCode.cpp` is unchanged.

**If camera routing is added:** units differ across the boundary. `AP_Mount_Viewpro::set_zoom(PCT)`
was modified to take zoom *times* (`zoom_value * 10`), while `AP_Camera_MAVLinkCamV2::set_zoom(PCT)`
follows the MAVLink contract of 0-100 percent. A shared route must convert.



**D10 resolution plan, and what a Stage 1 prototype found (2026-09-28).**

A staged plan, cheapest first:

| Stage | What | Blocked on |
|:--:|---|---|
| 0 | Ask PNU: **must KGCS TC2 drive non-Viewpro cameras at all?** If no, D10 closes by rejecting TC2 camera commands on a non-Viewpro mount and gating the TM2 stream - roughly 20 lines, no refactor | one question to PNU |
| 1 | **APPLIED 2026-10-01.** Stop fabricating values: give `get_zoom_times()` an error signal, and check the 27 unchecked camera return values | nothing |
| 2 | Dual-dispatch in `AP_Q30` - try `AP::mount()`, fall back to `AP::camera()` | stage 0 answer |
| 3 | The two functions with no `AP_Camera` path at all | **D11** / the PNU action item |

**Stage 1 was prototyped on 2026-09-28 and reverted**, because it touches 8 files and ~26
call sites, which was too broad to carry alongside the migration. **That objection expired
when the migration closed at 5.1.0, and Stage 1 was applied on 2026-10-01** - see the table
below. The findings from the prototype, all re-verified against the tree before the work:

- **`AP_Mount` and `AP_Camera` are signature-compatible for 6 of the 8 camera functions** -
  same names, same `ZoomType` / `FocusType` / `TrackingType` / `SetFocusResult` enums, same
  `bool` returns. Stage 2 is therefore mechanical forwarding, not a redesign. Only
  `set_camera_source` differs (`uint8_t` vs a `CameraSource` enum).
- `get_zoom_times()` has only **three callers in the whole tree** (`AP_Q30.cpp` x2,
  `GCS_Common.cpp` x1), so changing its signature is cheap.
- `AP_Mount::get_zoom_times()` contains `return false;` inside a `float` function - a second
  fabricated `0.0f`, on the no-backend path. **Fixed in Stage 1** - the function now returns
  `bool`, so that line is correct rather than accidental.
- `AP_Mount_Viewpro::_zoom_times` has **no initialiser** and the class uses an inherited
  constructor, so before the first gimbal report it is *indeterminate*, not 0. **Fixed in
  Stage 1**, which also adopted the `is_positive()` validity test suggested here: EO and IR
  zoom are both >= 1x, so the 0 it is now initialised to cannot collide with a real reading.
  This is the same defect class as D1's `_palette_pending_color`, and it was sitting two
  lines above the comment D1 left saying those members must keep their initialisers.
- There is **not one `if (mount->...)` in `AP_Q30.cpp`** - 27 camera calls, no return checked.
  **Fixed in Stage 1**: all 27 are checked and report through one rate-limited helper. The
  three gimbal calls need no check - `set_angle_target()` and `set_rate_target()` return `void`.
- TM2's `Zoom_POS_FB` has no "unknown" encoding, exactly like TM5 in D12, so a zoom-truth fix
  cannot change what goes on the wire without an ICD decision.
- `EO_zoom_pct` is constrained to `[1, _Max_zoom_EO]` with `_Max_zoom_EO = 30.0`: it carries
  zoom **times** while being passed as `ZoomType::PCT`, and only works because
  `AP_Mount_Viewpro::set_zoom()` was modified to reinterpret PCT as times. **This is the trap
  in Stage 2** - forwarding it to `AP_Camera_MAVLinkCamV2::set_zoom(PCT)`, which honours the
  MAVLink 0-100 % contract, turns 10x into 10 %, zoomed out instead of in. **Renamed to
  `EO_zoom_times` in Stage 1**, with the conversion requirement spelled out at the
  declaration. The rename is cosmetic; **the conversion itself is still Stage 2 work** and
  has deliberately not been written, because which direction it converts depends on the
  routing decision that is still blocked.

**Stage 1 as applied, 2026-10-01.** Seven changes across six files. Nothing here changes
routing, so Viewpro behaviour is unchanged except where a value was previously fabricated.

| # | Change | Where |
|:-:|---|---|
| 1 | `get_zoom_times()` becomes `bool get_zoom_times(uint8_t, float&)` - the ArduPilot out-param idiom its sibling `get_attitude_euler()` already uses. The base class returns `false` and writes nothing instead of fabricating `0.0f` | `AP_Mount_Backend.h`, `AP_Mount.{h,cpp}` |
| 2 | `_zoom_times` gains its missing initialiser; the Viewpro override returns `false` until the gimbal has actually reported | `AP_Mount_Viewpro.{h,cpp}` |
| 3 | All 27 camera calls check their return and report via one rate-limited helper, `report_cam_unsupported()` | `AP_Q30.{h,cpp}` |
| 4 | TM2 reports zoom honestly. **The wire format is unchanged** - 0 still goes out, because `Zoom_POS_FB` has no "unknown" encoding and adding one is an ICD change, exactly the call D12 made for TM5 | `GCS_PNU.cpp`, `GCS.h` |
| 5 | TM2 is suppressed entirely when no gimbal is configured (`get_mount_type(0) == Type::None`), where every field would be zero | `GCS_PNU.cpp` |
| 6 | `EO_zoom_pct` renamed `EO_zoom_times` with the PCT-means-times trap documented at the declaration | `AP_Q30.cpp` |
| 7 | `_current_zoom_EO` / `_current_zoom_IR` hold their last known value when zoom is unavailable, instead of being overwritten with a fabricated 0 | `AP_Q30.cpp` |

*Design points worth not re-deriving:*

- **The rate limit is not optional.** KGCS streams TC2 at ~10 Hz, so on a mount without
  camera support *every* call fails on *every* tick. One message per 10 s with the first
  always reported - the same shape, and the same interval, as D6's PMU short-frame message.
- **One shared limiter, not one per function,** deliberately. On a non-Viewpro essentially
  every camera call fails, so the operator needs to learn "this mount does not do camera
  commands", not receive an itemised list. The first message names the first thing to fail,
  which is enough to diagnose; the counter shows it is persistent.
- **The zoom readback in `IR_operation()` reports nothing on failure,** on purpose. It runs
  at the TC2 rate and TM2 already announces the same condition once, on the edge. Two
  reports of one fact would just be noise.
- **TM2's zoom warning has a 10 s grace period** (`PNU_CAM_ZOOM_GRACE_MS`). A working
  Viewpro reports zoom only after its first telemetry frame, so an edge evaluated from boot
  would warn on *every* startup and then immediately recover. Past the grace period,
  still-unavailable means the mount genuinely does not report zoom.
- **`_current_zoom_EO` / `_current_zoom_IR` are write-only** - assigned in `IR_operation()`
  and read nowhere in the tree, the same situation as the D6 PMU counters. Kept rather than
  deleted, so the keep-or-remove call stays with PNU. If they are ever read, note they are
  `uint8_t` holding a `float` zoom.

*Verified:* `./waf copter` clean, zero warnings, **1,721,424 B** (was 1,720,608 B, so Stage 1
costs **+816 bytes** - the warning strings and the checks). The D13 invariant still holds:
`--define HAL_MOUNT_ENABLED=0` builds at 1,653,156 B. **Not bench-tested** - no hardware was
connected, so the Viewpro no-regression claim is a build-and-reasoning claim only, and the
one behavioural change a Viewpro operator should see is the absence of any new message.

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

**Suggested sequencing:** done - the migration closed at **5.1.0 OFFICIAL**, with every issue
resolved except D10 and D11. Gremsy support is the next piece of work, and it starts with the
bench session described above, not with code.

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


**PNU-ISSUE D9(b) - implemented 2026-09-30.** `FS_PMU_ENABLE` was considered and
**rejected**; no new parameter was added.

| Scenario | Prearm | In flight |
|---|---|---|
| PMU healthy, arm, PMU dies | passes | latch set &rarr; failsafe fires as before |
| PMU dead, emergency launch | fails `"PMU not healthy (state n)"`; operator sets `ARMING_SKIPCHK` = 8192 or force-arms | latch clear &rarr; no failsafe, flight proceeds |
| No PMU configured at all | `PMUCAN_Fail == 0` from static init &rarr; passes | `== 0` early return, unchanged |

`failsafe.pmucan_armed_healthy` is one bit in the existing `failsafe` struct, written in
`AP_Arming_Copter::arm()` beside `arm_time_ms`. `Copter` is a global, so the bit is
zero-initialised and the failsafe is inert until something arms.

The prearm check uses `Check::SYSTEM`, so **`ARMING_SKIPCHK` bit 13** (System, decimal
8192) bypasses it - ArduPilot's documented operator override - which is what makes the
emergency case work without a new parameter. **`ARMING_CHECK` no longer exists in 4.7:**
it was replaced by `ARMING_SKIPCHK` with the *inverted* sense - you SET a bit to skip a
check, where the old parameter had you clear one. A migration at `AP_Arming.cpp:230`
converts existing values on first boot, so only hand-entered parameters are affected.
Bit 13 skips **all** System checks, not just this one - inherent to `Check::SYSTEM`, and
the trade D9(b) accepted when it rejected a dedicated parameter. It reports states 1 (lost after being seen) and 2 (never seen); 2 is also the
state with no PMU fitted, which is why this warns rather than blocks.

*Why not `AP_Arming::can_checks()` as originally designed:* `AP_PMUCAN` appears only in
`ArduCopter/wscript`. Referencing `PMU_Ctrl_Echo` from the shared `AP_Arming` library would
fail to link for Plane, Rover and Sub. The check belongs to the vehicle that has the PMU.

---

<Working Notes - read before applying any step>

**Board / build.** CubeOrangePlus, `-Werror` is ON. `./waf copter` must finish with
**zero warnings and zero errors**; anything else is a regression. Check after every change.

**`./waf heli` is blocked on purpose.** Nothing in the PNU code is frame-dependent, so the
heli target would otherwise build happily and produce a flashable `arducopter-heli` binary
with the wrong motor output and swashplate handling for this airframe. A `#error` in
`ArduCopter/config.h`, right after `FRAME_CONFIG` resolves, stops it with an explanatory
message. Remove it deliberately - and re-validate the PMU, mount and failsafe behaviour -
if a helicopter airframe is ever adopted.

**Only Copter builds; Plane, Rover and Sub do not link.** Two shared libraries reference
PNU-only code that only `ArduCopter/wscript` lists:

| Library | Undefined symbols |
|---|---|
| `AP_CANManager.cpp` | `AP_PMUCAN::AP_PMUCAN()` - the PMUCAN case in the driver factory |
| `GCS_MAVLink/GCS_PNU.cpp` | `AP::Q30()`, `AP_Q30::*`, `CAM_ATTITUDE_STATUS`, `debug_cam_*` |

This dates from **step 4** (`ee4f54c95f`, the `AP_CANManager` PMUCAN case) and **step 7**
(the GCS handlers, moved into `GCS_PNU.cpp` by D5) - it is not a regression from any later
work. It does not matter for this project, which ships Copter only, and the other vehicles
live on their own branches. Recorded so nobody mistakes it for something they broke.
Fixing it would mean an `AP_PMUCAN_ENABLED`-style flag on both libraries, the same shape as
D13's `AP_Q30_ENABLED` - worth doing only if a non-Copter vehicle is ever needed from this
branch. Deliberately **not** raised as a PNU-ISSUE.

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
| `ArduCopter/Copter.h` | `AP_Q30 q30` member behind `#if AP_Q30_ENABLED` (D13); `failsafe.pmucan` and `failsafe.pmucan_armed_healthy` bits (D9) |
| `libraries/AP_PMUCAN/*` | the whole step-4 cleanup; 4 vendored `pmucan_*.hpp` were **deleted**; DLC validation (D6), endianness comments (D15), global declarations (D14), engine-interlock send check (D7) |
| `libraries/AP_Mount/AP_Mount_Backend.{h,cpp}` | `pitch_target_is_reversed()`, `suppress_gimbal_device_attitude_status()` (D8a); the `get_zoom_times()` out-param signature (D10 Stage 1) |
| `libraries/AP_Mount/AP_Mount.{h,cpp}` | the `get_zoom_times()` out-param signature (D10 Stage 1) |
| `libraries/AP_Mount/AP_Mount_Viewpro.{h,cpp}` | both overrides; D1 markers; `_zoom_times` initialiser and the `get_zoom_times()` validity test (D10 Stage 1) |
| `libraries/AP_SerialManager/AP_SerialManager.h` | IOMCU renumber deliberately **not** applied |
| `libraries/AP_OSD/AP_OSD_ParamSetting.cpp` | `"Q30"` padding deliberately **not** applied |
| `libraries/GCS_MAVLink/GCS_Common.cpp` | upward-proximity block deliberately **not** commented out; the handler bodies live in `GCS_PNU.cpp`, only 3 hooks remain here (D5) |
| `libraries/GCS_MAVLink/GCS_PNU.cpp` | PNU-only file; the step-7 defect fixes and `#if` guards live here; the D10 Stage 1 TM2 zoom-truth and no-gimbal gate |
| `libraries/AP_Q30/AP_Q30.{h,cpp}` | `send_cmd_speed()` negates pitch at the call site (see the pitch-reversal table); `AP_Q30_ENABLED` guard and the D14 global declarations; the 27 return checks and `report_cam_unsupported()` (D10 Stage 1) |
| `ArduCopter/UserCode.cpp` | `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` guard around the OA block |
| `ArduCopter/events.cpp` | D9(a) recovery-clear fix and D9(b) arm-time latch gate; `LOGGER_WRITE_ERROR` portability fix |
| `ArduCopter/AP_Arming_Copter.{h,cpp}` | `pmucan_checks()` prearm warning and the arm-time healthy latch (D9b) |
| `libraries/GCS_MAVLink/GCS.h` | `OFP_VER_*`; the D3 dialect `#error` guard; the D12 `prev_prx_failed` latch; the D10 `prev_zoom_unavailable` latch; PNU handler declarations |
| `libraries/AP_Q30/AP_Q30_config.h` | PNU-only file - the `AP_Q30_ENABLED` flag (D13) |
| `Tools/scripts/build_options.py` | the `Camera / Q30` build option (D13) |
| `ArduCopter/version.h` | **5.1.1 DEV** and the release note describing what it carries |
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
| `version.h` is **5.1.1 DEV** | 5.1.0 OFFICIAL closed the migration; 5.1.1 carries the post-release D10 Stage 1 work and goes OFFICIAL when D10/D11 land |
| `GCS.h` dead CAM macros removed, `OFP_VER_*` tracked against `version.h` | were stale/unused; guarded by `static_assert` in `Copter.cpp` |
| `AP_PMUCAN` fixes | dropped-frame at RX budget, dead branch, `&`->`&&`, `RXdrain()` extraction, named constants, ctor init, `TXspin` void. See git log |
| `send_proximity()` upward-distance block **not** commented out | V5.0.8 wrapped it in `/* Currently, no upward sensor */`. `AP_Proximity::get_upward_distance()` already returns false when no backend supplies one (`AP_Proximity.cpp:498`), and TeraRanger Tower Evo is not one of the backends that does - so the comment-out is a no-op here and a silent regression for `AP_Proximity_RangeFinder` / `_MAV` / scripting users. Not applied |
| OA sender/handler guarded `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` | `GCS_Common.cpp` is core; `AC_Avoid` and `AP_Proximity` are optional. Step 8's `UserCode.cpp` caller needs the same guard - without it a proximity-disabled build hits the `try_send_message()` default case, which spams "Sending unknown message" and panics in SITL |
| step-7 defect fixes in the `GCS_Common.cpp` block | see the table below |
| step 8: OA block in `userhook_MediumLoop()` guarded `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` | `avoid` is itself `#if AP_AVOIDANCE_ENABLED` (`Copter.h:511`), and step 7 put `MSG_OBJECT_AVOIDANCE_STATUS` behind the same guard. Without it a proximity-disabled build hits the `try_send_message()` default case, which sends "Sending unknown message" and panics in SITL |
| step 8: `copter.avoid.` &rarr; `avoid.` | inside a `Copter` member function; `copter.` is the global instance and redundant |
| step 8: tabs &rarr; 4 spaces in the added lines | V5.0.8 mixed tabs and spaces in both hunks |

**Step 7 defects fixed while applying the V5.0.8 GCS block.** The block itself now lives in
`GCS_PNU.cpp` (D5); these fixes travelled with it. None
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

**Remaining work:** all 8 numbered steps are applied, both free-floaters are closed as
not-applied, and **13 of the 15 issues are resolved**. Left open: **D10** and **D11**.

**D10 Stage 1 is applied (2026-10-01)** - every fabricated camera value is gone and every
declined camera command is now reported. **The D10 routing decision and all of D11 remain
blocked on the ACTION ON PNU answers above**, and nothing in this repo will unblock them.
Stage 1 deliberately did not touch routing: it makes the symptom visible so that a Gremsy
bench session produces a diagnosis instead of silence.

What is left on D10/D11, in order:

| | Work | Blocked on |
|:--:|---|---|
| Stage 2 | Dual-dispatch in `AP_Q30` - try `AP::mount()`, fall back to `AP::camera()`. **Convert `EO_zoom_times` at the boundary first** (renamed but not converted) | the ACTION ON PNU answers |
| Stage 3 | The two functions with no `AP_Camera` path - `get_zoom_times` (needs `CAMERA_SETTINGS` msg 260 decoding; **check ArduPilot master first**) and `IR_Color_Change` (no MAVLink standard; vendor-specific) | D11 / a VIO on the bench |

Residuals recorded against resolved issues: D1 (is the prelude needed at all?), D6 (the PMU
counters are still write-only), D7 (accepted interlock residuals), D12 (confirm the 10 m /
15 m thresholds). New from Stage 1: `_current_zoom_EO` / `_current_zoom_IR` are write-only,
the same keep-or-delete question as D6's counters.

**Open issues** are the `PNU-ISSUE` table above; code markers carry the same ids.
List them with:

```
grep -rn "PNU-ISSUE" libraries/ ArduCopter/
```
