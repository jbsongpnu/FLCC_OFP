# PNU-KAL FLCC OFP Development Version
- This branch currently developing PNU-KAL FLCC OFP after official version (TAG) PNU-KAL-FLCC-5.1.0 (this is same code as 5.0.15 dev)  
- PNU-ISSUE 10 & 11 still remains : For Gremsy camera and general camera mount library structure  
- **CAN Update Plan step 2c is APPLIED and MEASURED in 5.1.18** - a **500 ms TC1 staleness reset** and a **10 s stop-before-start window**. **The defect was reproduced, not just the fix**: with guard 1 disabled the same cut-short Start burst started the engine 1.34 s after the last message; with it enabled, no start. Interlock duration measured at **1.50 s**, matching the bench's ~1-1.5 s. Driven by `Tools/pnu/sitl_can/tc1_inject.py`, and **re-run independently by PNU 2026-10-29 with every scenario reporting AS EXPECTED**  
- **SAR Integration Plan step 3a is DONE in 5.1.19** - `AP_PNUSAR` is an `AP_PNU_CAN` backend, **protocol 17 is retired**, and the connection layer works: Heart_Beat `0x510`, the FC-maintained Live_Counter, the 3 s `T_HB` timeout and `SAR_STATUS` 0 and 1. **Now exercised against the fake SAR** by step 3b; still **no SAR hardware**  
- **SAR Integration Plan step 3b is DONE, tools only with no version bump** - `Tools/pnu/sitl_can/fake_sar.py`, and with it **the first shared-bus run this project has ever done**: fake PMU and fake SAR on one `vcan0` at `CAN_D1_PNU_OPTS = 5`. Measured 2026-10-30 - `SAR disconnected` **3.058 s** after the last heartbeat against the ICD's 3,000 ms, the reconnection announced in 16 ms **with the state staying Disconnected**, and the short frame, the reserved `0x523` and an intruding `0x400` each reported. The PMU answered **505 of 505 RTRs** on the shared wire throughout  
- **SAR Integration Plan step 3c is DONE in 5.1.20** - TC **50050** and TM **51050**, the TM at **5 Hz unconditionally**, and TC ingest that checks its source. **`Live_Counter` is now observable and was measured against the fake SAR**: 6 receptions counted while the device's own `Raw_Counter` read 777005  
- **SAR Integration Plan step 3d is DONE in 5.1.21** - the **observation command layer**: `0x501`/`0x502` out, `0x521`/`0x522` in, the FC's **2 s window**, `SAR_STATUS` 2, 3 and 4, and the ICD 14.3.3 **Start interlock** behind the new **`CAN_Dn_PNU_SCHK`** bitmask. Measured in SITL against the fake SAR on a shared bus with the fake PMU; **no SAR hardware and no KGCS**  
- **SAR Integration Plan step 3e is DONE in 5.1.22** - the **10 Hz navigation bundle**, nine frames `0x400`-`0x408`, 72 payload bytes per snapshot, plus ICD 14.7 **automatic termination**. Measured at **9.99 Hz** against the fake SAR with `ATT_Seq` 1:1 and every field decoding to its expected physical value. **`0x404` carries sentinels, not zeros** - ICD 16.4 - until the gimbal exists  
- NEXT : **step 3f**, real gimbal attitude into `0x404` and the 40 ms `ATT_FRESH` contract. It is also what makes `CAN_Dn_PNU_SCHK` bit 2 usable: setting it today refuses every Start and terminates every observation, because a constant has no age. **Blocked on a measured CubeMars feedback rate**  
- **The ICD confirms the ID reservation made before it existed** - `0x400`-`0x7FF`, 11-bit, 1 Mbit/s - so the gimbal, PMU and SAR still share CAN1 without collision, and MAVLink 50050/51050 are free  
- ALSO NEXT : **PNU's eye on two step 2c decisions** - the Stop window is measured from the **Start press** rather than from the interlock completing, and a **Stop** burst cut off mid-way no longer completes either. Both are recorded in the step 2c section. **Not tested against a real PMU, engine, KGCS or LTE link**  
- **The fake gimbal is DONE 2026-10-30** - `Tools/pnu/sitl_can/fake_gimbal.py`, step 2b's other half at last. It found a blocking bug on its first run: **`AP_PNUGimbal` never enabled the motors at boot**, because the backend starts in `INIT` and `set_state(INIT)` early-returns, so INIT's entry actions never ran. Fixed. **The Appendix 1 homing sequence then ran end to end for the first time**, and `CAN_D1_PNU_OPTS = 7` - all three devices on one bus - ran for the first time too  
- ALSO NEXT, and it now decides step 3f : **the driver assumes the motors feed back on their own.** With feedback only in reply to a command it faults 500 ms after the enable, every time. Measure the real motors' **feedback trigger** before their rate - if they only answer commands, `AP_PNUGimbal` needs a keep-alive and 3f's 40 ms budget has to be re-derived  
- ALSO NEXT : **Bench-tune the homing parameters, then set `CAN_Dn_PNU_HOME=1`.** Step 1d is implemented but **defaults to OFF** - the stall threshold, approach velocity and the two gains are provisional and unmeasured. Nothing drives into a stop until somebody turns it on. See 6f near the end of this file for the tuning order. **The sequence itself is now rehearsed in SITL**, which de-risks the run but measures none of those four numbers - and the rehearsal showed stop compliance eating 0.6 deg of the 3 deg `PNU_HTOL` before any real-world asymmetry  
- THEN : **step 3g**, ICD section 16 - the 25 deg stabilisation envelope and **yaw to the waypoint-segment heading**  
- *Done in step 2b, tools only with no version bump :* the **fake PMU is validated** (`Tools/pnu/sitl_can/`, flown in SITL 2026-10-29 with Mission Planner on SERIAL0 and KGCS on SERIAL1 - choke, the choke crank, Start, warm-up idle, the cooling trip, Stop, takeoff and RTL with landing). **The D9 failsafe now runs end to end on a desk**: `silent` trips it to RTL, `resume` clears it, no connector pulled. **The start-up ORDER is not optional** - `vcan0` before SITL, Mission Planner on TCP 5760 before SERIAL1 and SERIAL2 exist at all; the validated sequence is in `Tools/pnu/sitl_can/README.md`  
- **BENCH STATUS (5.1.16)** : PMU telemetry, command paths, prearm and the **in-flight failsafe** all PASSED 2026-10-19. **Outstanding** : the **shared bus** (`PNU_OPTS=3`, the test step 2 exists for), **Stop-command timing**, and the **~1.0 s interlock duration** after the 10 Hz rework  
- *Done in the gimbal track :* step 1b (`AP_Mount_PNUGimbal`, 5.1.12) and step 1c (lifecycle state machine, 5.1.13). `AP_Mount` does stabilise against FC attitude and needs no camera, which is why the gimbal is an `AP_Mount` backend  
- IN THIS VERSION : V5.1.23 - **one `AP_PNUGimbal` fix, found by the fake gimbal on its first run.** The driver never brought the motors up at all, and would have stopped the gimbal dead on the bench with no message to say why  
   - **`AP_PNUGimbal` transmitted NOTHING at boot.** `_state` is initialised to `State::INIT` and `set_state(State::INIT)` early-returns when the state is unchanged, so INIT's entry actions - the per-axis clear-errors and the enable - were never performed. No feedback could arrive, `all_axes_enabled()` stayed false, and INIT timed out into FAULT after 5 s, whose own entry queued a disable. **Those three disables were the only frames the motors ever saw.** `init()` now runs the entry actions through a shared `enter_init()`  
   - **Wrong since the lifecycle state machine landed in 5.1.13**, and invisible because every statustext that would have explained it fires in the first seconds after boot - **before any GCS has connected and before the dataflash logger has started**. The same shape as the one-shot startup line that went missing in 5.1.16  
   - **The Appendix 1 homing sequence then ran end to end for the first time**: roll travel 60.6 deg, pitch 60.6, yaw 40.5, each midpoint -0.1, zero set per axis, then `HOMING -> ALIGN -> RUN` in 40 s. The excess over the true 60 and 40 is stop compliance, which **eats a fifth of `PNU_HTOL`'s 3 degree default** before any real-world asymmetry  
   - **`CAN_Dn_PNU_OPTS = 7`** - PMU, gimbal and SAR on one bus - ran for the first time, with no demultiplexing complaint  
   - **STILL OPEN, and it decides step 3f:** the driver assumes the motors feed back autonomously. With feedback sent only in reply to a command it faults **523 ms** after the enable, against the 500 ms `AP_PNUGIMBAL_FB_TIMEOUT_MS`. The manual specifies no feedback trigger, so that is the first thing to measure on real motors, ahead of the rate  
- For 5.1.22 version : **SAR Integration Plan step 3e: the 10 Hz navigation bundle.** Nine frames `0x400`-`0x408` every 100 ms while an observation runs, plus ICD 14.7 automatic termination. **Measured in SITL against the fake SAR**; no SAR hardware and no KGCS  
   - **The snapshot is taken on the VEHICLE thread**, in a new 10 Hz Copter `SCHED_TASK`, and the CAN thread only writes out what it is handed. `AP_GPS::update()` runs on the vehicle thread, so a snapshot there cannot straddle a GPS update - the same twenty-odd fields read from the CAN thread could have built one bundle from **two different samples**, which is what ICD 7.3's "same acquisition time" forbids  
   - **`0x404` carries SENTINELS, not zeros** - `INT16_MIN` for Roll and Pitch, `0xFFFF` for Yaw, `ATT_Flags` empty. ICD 16.4 asks for sentinels when actual attitude is unavailable, and zeros would read as a level, north-facing SAR to anything that checks values before flags. **`ATT_Seq` still advances**, so a bundle lost before the bus leaves a visible gap  
   - **`CAN_Dn_PNU_SCHK` now governs ICD 14.7 as well as 14.3.3.** The same bits gate the Start interlock and the matching automatic-termination conditions, because a bench allowed to Start without RTK would otherwise have its observation torn down 100 ms later. **The vehicle's own navigation/EKF failsafe is not gated** and always terminates  
   - **An automatic termination is not a normal Stop.** Navigation goes off first, then `0x502` if the heartbeat is still live, and its response or its 2 s timeout is an **internal result** - never `SAR_STATUS` 2 or 4. The published state is ICD 14.8's recovery-pending 0, which a reconnect does not release  
   - **Two small public accessors added to `AP_GPS`**: `get_rtcm_stats()` for `0x407`, which was private, and `have_gps_yaw_accuracy()` so `YAcc` carries the unavailable sentinel instead of `gps_yaw_deg()`'s 10 degree fallback - a distinction ICD 6.2 asks for by name  
   - **12 new codec tests run the ICD's own section 9.1 byte vectors against the firmware's packer**, including its `-1.235 -> -124` rounding rule and the `36000 -> 0` angle rule. The fake SAR now assembles and decodes bundles under ICD 7.3's boundary rules; 29 Python tests  
   - **A 50 ms staleness window dropped 6% of bundles**, and finding out why exposed an uncounted drop - a snapshot overwritten before the CAN thread reached it. Both fixed; at one full interval the loss is **zero**  
- For 5.1.21 version : **SAR Integration Plan step 3d: the SAR's observation command layer.** `0x501`/`0x502` go out, `0x521`/`0x522` come back, `SAR_STATUS` reaches 2, 3 and 4, and TC50 is no longer recorded-and-ignored. **Measured in SITL against the fake SAR**; no SAR hardware and no KGCS  
   - **`SAR_STATUS` is now COMPUTED, not assigned.** ICD section 8 states a priority - disconnected or recovery-pending (0) outranks the current attempt's timeout (4), which outranks the last confirmed state (1/2/3) - and with five states and two latches, last-writer-wins would quietly break it. One `publish_state()` decides, so **"a heartbeat must not overwrite 2/3/4 with Ready" is true by construction** rather than by remembering it  
   - **The FC opens a 2 s window and NEVER retries** (ICD 14.5 prohibits it - an FC retry loop would exceed the GCS's two-attempt limit). A window opens only for a command **actually transmitted**, so a refused send does not become state 4; a new command clears the previous latch and republishes `LAST_CONFIRMED_STATE` while it waits; an expired response is never counted retroactively. **A failed Start leaves navigation OFF, a failed Stop RETAINS it**, because stopping is unconfirmed  
   - **New `CAN_Dn_PNU_SCHK`**, the Start interlock as a bitmask - RTK Fixed, GPS fix age, gimbal attitude age, navigation health. **Default 11 is the ICD's set less the attitude age**, which is off because `0x404` is hard-coded zeros until 3f and **a constant has no age**; setting that bit refuses every Start, which is the honest behaviour. It exists because a bench or an integration test has no RTK fix and could otherwise never send one Start. **Relaxing it is announced** once per boot and again on every Start under it, and it is read live - no reboot  
   - **Stop is never gated** (ICD 14.4.2, in bold there): stopping a payload producing contaminated data must not depend on the quality of that data  
   - **Four frames, one payload byte `0x01`.** The ICD says in its own words not to encode Stop as `0x02` merely because it is Stop, so one constant serves commands and responses alike. A response carrying anything else is refused and the window left to expire - measured with `0xFF`  
   - **A TC50 that can reach no device now says so** (`SAR: no backend, check PNU_OPTS`). Without it the GCS's own 3 s window would report the SAR as not responding, blaming the payload for a configuration fault on this aircraft - PNU-ISSUE(D6)  
   - **`SIM_GPS1_*` reports RTK Fixed in SITL**, which is what made the *permitted* path testable and not only the refusals  
- For 5.1.20 version : **SAR Integration Plan step 3c: the SAR's two MAVLink messages.** TC **50050** and TM **51050** in `pnu_kal.xml`, the TM at **5 Hz unconditionally**, TC ingest with a source check. **Measured in SITL against the fake SAR**; no SAR hardware and no KGCS  
   - **`Live_Counter` is observable for the first time.** Measured: the FC reported **6** after six heartbeats while the device's own `Raw_Counter` read **777005** - two different quantities, which is what ICD 3.3 requires. A rejected short heartbeat did **not** increment it, and it **held its value across a disconnect** and continued on reconnect  
   - **TM50 is 5 Hz whatever else is true** (ICD 3.2), so it is its own `SCHED_TASK` and not a stream-group entry - Copter's default stream rates are all zero, and a rate the ICD fixes is not the operator's to set. With no SAR backend in the build it reports Disconnected and a zero count, which is the truth in that configuration. Measured at **5.01 Hz**  
   - **OPERATIONAL : KGCS must send TC50 from a system id `MAV_GCS_SYSID` covers** (default 255), or it is refused with a message naming the parameter. TC50 has no `target_system` field, so ICD 3.1 asks the FC to check the sender, and `sysid_is_gcs()` is that gate. **TC1 never checked, so nothing before this depended on that parameter being right.** The component id is deliberately not filtered - the ICD names no permitted value - and is logged instead  
   - **`Observation_Control` 3-255 is refused, not clamped** (ICD 3.1), and `Observation_Mode` is retained whole and never interpreted, because its CAN encoding depends on `0x503` and the manufacturer has not defined it  
   - **3c records the command and does not act on it.** Start and Stop answer `SAR: Start/Stop not implemented yet` until step 3d. A press that vanished silently is PNU-ISSUE(D6)  
   - **CRC_EXTRA is 231 (TC50) and 7 (TM50)**, generated from the dialect and independently re-derived; both endpoints must agree. Every TC50 is logged as `SARC` with its sender, channel and the refusal that followed; TM50 is logged as `SARS`  
- For 5.1.19 version : **SAR Integration Plan step 3a: the SAR becomes an `AP_PNU_CAN` backend and protocol 17 retires.** See the SAR Integration Plan; **not tested beyond builds**  
   - **`AP_PNUSAR` is no longer an `AP_CANDriver`.** It is a backend selected by `CAN_Dn_PNU_OPTS` bit 2 on protocol 15, so the PMU, the gimbal and the SAR are three protocols behind one driver - which is the only arrangement `AP_CANManager` allows on one interface  
   - **Protocol 17 is retired and NOT reused.** A stored 17 falls through to "no driver" rather than silently becoming a different device  
   - **Live_Counter counts what the FC RECEIVED**, not the SAR's own `Raw_Counter`, which the heartbeat carries and which this code never reads (ICD 3.3)  
   - **A reconnecting heartbeat does not restore Ready** (ICD 14.8). It is announced once and the state stays Disconnected, because the data contaminated by the outage has not been inspected and the release procedure is deferred by the ICD. Until it exists the only way back is a power cycle  
   - Command, response and navigation ids are named and claimed already, so a real device's early traffic is counted rather than reported as unknown  
- For 5.1.18 version :  
   - **CAN Update Plan step 2c, the two engine-start interlock windows** (`AP_PMUCAN` only), **measured in SITL and with the original defect reproduced**. See the step 2c section  
   - **TC1 staleness, 500 ms.** `engineonoffstate()`'s counters are a tick timer, not a count of operator alternations - the command pair only changes when a new TC1 arrives but the function runs at 10 Hz regardless, so a frozen pair kept counting and **a link dying mid-burst completed the start on its own about a second later**. Counting now stops  
   - **Stop before start, 10 s.** KGCS forces a Stop before a Start, so the FC requires the same and refuses a Start that does not follow a recent one - covering the link dying *between* the two  
   - **Both windows are stamped only where a NEW TC1 is detected.** The frozen pair stays a valid Stop alternation from the end of one burst to the start of the next, so testing it at 10 Hz would have held the window permanently open. That is why this is in `transmit_slot()` and not in `engineonoffstate()`  
   - **Measured, and the defect reproduced**: with guard 1 disabled the identical cut-short burst started the engine 1.34 s after the last message; with it enabled, no start. `stop-start` completes in 1.50 s, matching the bench's ~1-1.5 s  
   - **Two decisions that want PNU's eye** - the Stop window is measured from the **Start press** rather than from the interlock completing, and a **Stop** burst cut off mid-way no longer completes either  
   - Two defects found while measuring and fixed: the refusal message exceeded STATUSTEXT's 50 characters and arrived split in two, and the fake PMU exited immediately whenever stdin was not a terminal  
- For 5.1.17 version :  
   - **The PMU failsafe moves out of a logging task.** `failsafe_pmucan_check()` was called from `ten_hz_logging_loop()`, which sits inside `#if HAL_LOGGING_ENABLED`, so a logging-free build had **no PMU failsafe at all** and said nothing about it. Now its own `SCHED_TASK` at priority 92, same 10 Hz. **Found by reading, not by a failure** - and no shippable image was ever exposed, because Copter does not currently build with logging off anyway  
- For 5.1.16 version :  
   - **`ARMING_SKIPCHK` bit 13 had NEVER worked, and D9(b)'s design rested on it.** `check_failed()` only formats and reports - it **never consults `check_enabled()`** - so a check must wrap *itself*, the way `AP_Arming.cpp:1134` and `:1303` do. `pmucan_checks()` did not, so bit 13 did nothing and **an aircraft with no PMU could not be armed at all** except by disabling every check. Wrong since D9(b) landed in 5.0.x; **nothing in step 2 caused it**  
   - Skipping leaves `PMUCAN_Fail` untouched, so `arm()` still latches `pmucan_armed_healthy = false` and the in-flight failsafe stays out of the way - exactly the "launched without a PMU deliberately" case D9(b) designed  
   - **The one-shot startup report was emitted before anyone was listening.** It fired at a flat 5 s from the CAN thread; KGCS had not connected, the statustext was dropped, and being one-shot it never returned - so **neither `PMU ready` nor the protocol-16 deprecation warning was ever seen, and 15 and 16 looked identical on the bench**  
   - It now waits for `GCS_MAVLINK::active_channel_mask()` to be non-zero - the condition that actually matters - with **no upper bound**. **A one-shot message nobody receives is the silence D6 exists to remove**, which is the lesson  
   - **Bench-validated in 5.1.15, unchanged here** : PMU telemetry downlink, KGCS and FC command paths, the prearm block with the PMU absent, and the **in-flight failsafe** - pulling the CAN connector while armed took the `set_mode_RTL_or_land_with_pause()` branch immediately and recovered cleanly on reconnect  
   - **Both fixes bench-confirmed 2026-10-19** : `PNU_CAN: CAN_D1 PMU ready` now appears at boot, `PNU_CAN: CAN_D1_PROTOCOL 16 is deprecated, use 15` appears on 16, `[PMU] Connection Failure!` on an absent PMU, and **`ARMING_SKIPCHK` bit 13 now bypasses the check as D9(b) always intended**  
   - **The ~1.0 s engine interlock survived the 10 Hz rework** - measured ~1 to 1.5 s on the bench 2026-10-19 against an expected ~1.2 s (0.2 s to establish the first alternating pair, then 10 counts at 10 Hz). That closes the main risk of making the rate explicit  
   - **STILL UNTESTED : the shared bus.** Every bench run had `CAN_Dn_PNU_OPTS = 1`, so the id-based health rewrite - the thing step 2 exists for - has never been exercised  
- For 5.1.15 version :  
   - **CAN Update Plan step 2 : the PMU becomes an `AP_PNU_CAN` backend**  
   - **Protocol 15 now instantiates `AP_PNU_CAN`**, with `CAN_Dn_PNU_OPTS` bit 0 creating `AP_PMUCAN` as one of its backends. **The enum value is unchanged and `PNU_OPTS` defaults to 1**, so an upgrade is a **no-op** for every deployed aircraft : PMU only, exactly as before  
   - **16 survives as `PNU_CAN_LEGACY`**, a deprecated alias that warns once per boot, so a bench aircraft configured during step 1 keeps working instead of hitting the factory's `default:` and silently getting no driver, no parameters and no message. Delete it when 5.1.x goes OFFICIAL  
   - **The 10 Hz is now EXPLICIT.** It used to be an accident : `TXspin()` ran at 50 Hz and rotated five commands, so each *happened* to go out at 10 Hz and nothing said so. D7's interlock is calibrated on it - threshold 10 ticks ≈ 1.0 s only because `engineonoffstate()` is reached once per rotation. Now derived from `PMUCAN_MSG_RATE_HZ` and the driver tick, reproducing the old cadence exactly (slot *n* at `tick % 10 == n*2`) while saying why  
   - **Health is now keyed on PMU STATUS FRAMES.** `RXspin()` treated *any* successful receive as proof of life - harmless on a dedicated bus, but on the shared bus **gimbal traffic would have held `PMUCAN_Fail` at CONNECTED with the PMU dead, and `failsafe_pmucan_check()` would never have fired.** Only the five PMU status ids feed the leaky bucket now; its thresholds and timing are unchanged  
   - **The directory layout is untouched, deliberately.** `AP_PMUCAN/AP_PMUCAN.h` keeps its path and still declares `PMU_Status` / `PMU_Ctrl_Echo` beside their definitions (D14), so **`events.cpp`, `AP_Arming_Copter.cpp` and `GCS_PNU.cpp` need no change - verified by diff**. Both globals and their definitions sit *outside* the `AP_PNU_CAN_ENABLED` guard, because those files reference them unconditionally  
   - **Deviation from the plan, deliberate : protocol 17 is NOT dropped here.** Removing it means `AP_PNUSAR` stops being an `AP_CANDriver`, which is step 3's job. Bundling it into the risky PMU conversion would have mixed two unrelated changes  
   - Dead members the conversion created were removed rather than left (D6) : `_can_iface`, `sem_handle`, `_thread_name`, `_initialized`, `_driver_index`, `pmucan_period_us`, `_AP_PMUCAN_loop_cnt`, `_cmd_idx`, `_rtr_idx`, `RTR_MSG`, `_pmucan_last_send_us`, `PMUCAN_SEND_TIMEOUT_US`  
   - Builds : CubeOrangePlus **1,733,336 B**. All five guards compile out, including `AP_PNU_CAN_ENABLED=0` at 1,715,412 B. `CAN_Dn_PROTOCOL` 15 now reads **`PNU_CAN`**. 10 codec tests pass  
   - **NOT BENCH-TESTED** - see the warning at the top of this file  
- For 5.1.14 version :  
   - **CAN Update Plan step 1d : HOMING, the Appendix 1 endpoint search**  
   - **`CAN_Dn_PNU_HOME` DEFAULTS TO 0, deliberately.** This routine drives a bulky payload into three sets of hard stops with the mechanical clamp *disabled*, and the tuning is provisional - only the 1 s stall dwell is settled. **Nothing drives into a stop until it is switched on.** With it off the driver says so once per boot and the zero is set by hand  
   - **MIT mode is mandatory and enforced.** `init()` refuses to arm the sequence unless `PNU_MODE=0`, naming it in the message. MIT is the only mode where the host bounds the push into a stop : a sweep sends `kp=0, kd=PNU_HKD, v=±PNU_HVEL`, so the stall torque settles at **HKD × HVEL** - a number the operator picks. Position/velocity mode runs a cascaded loop whose torque the CAN frame cannot limit at all  
   - **Endpoint detection is by ANGLE ONLY** (Appendix 1 item 4) - overcurrent, undervoltage and overload are *emergency stops, not endpoint criteria*. An endpoint is "the angle stopped changing while motion was still commanded"; any fault status aborts  
   - Sequence per axis, **roll then pitch then yaw** : seek a stop, back off 2% of travel, seek again and require both readings within `PNU_HRTOL`; repeat the other way; validate stop-to-stop travel against twice the mechanical limit within `PNU_HTOL`; move to the midpoint; confirm motion stopped; zero; verify the feedback reads zero. **The idle axes are HELD at their power-up angle throughout**  
   - **Travel validation is the only runtime check that can catch a `PNU_PMAX` mismatch** - the failure that does not fail but silently rescales everything. A factor-of-two travel reading is its signature and nothing else in the system would notice  
   - **Two hazards found while writing it, both fixed.** VERIFY originally commanded position 0 *before* confirming the zero had been sent - that means the OLD origin, possibly tens of degrees away, and would have lurched the payload across its travel. And HOMING was reachable from a *second* INIT, so clearing a fault **in flight** would have re-run the endpoint search mid-air  
   - Homing is therefore **once per POWER CYCLE**, not once per INIT - the zero lives in the drive and survives a disable/enable - and is **refused while armed** (`hal.util->get_soft_armed()`), because `ARMING_SKIPCHK` bit 16 lets an operator arm an unhealthy mount  
   - **Travel validation is ASYMMETRIC** (settled 2026-10-19). Lower bound is twice (mechanical − `PNU_MARG`) = **50 deg on a 60 deg axis** : the mount must reach its own operating range, and below that it aborts as obstructed. A measurement between the operating and mechanical ranges is **accepted, and `MNT1_*` stays nominal** - limits are *not* derived from the measurement, because stop compliance under a heavy payload would make the mount's reach vary with how hard it happened to press. Upper bound stays mechanical + `PNU_HTOL`  
   - **Per-axis rotation direction : `PNU_DRLL` / `DPIT` / `DYAW`** (0 normal, 1 reversed). The mount is still undergoing mechanical design changes, so motor direction is configuration, not a source constant. Everything inside `AP_PNUGimbal_Backend` is stored in the **motor frame** and the conversion happens in exactly **two places** - `set_target()` in, `get_feedback()` out - so there is one pair of boundaries rather than a sign scattered through the code. **Homing is unaffected**: it finds both stops and takes the midpoint, which is direction-agnostic  
   - **12 new parameters** : `HOME`, `HVEL`, `HKD`, `HKP`, `HSTL`, `HDLT`, `HTOL`, `HRTOL`, `MARG`, `DRLL`, `DPIT`, `DYAW`. Phase timeouts are **derived** from travel and velocity rather than parameterised, so they cannot be set inconsistently with the speed  
   - Builds : CubeOrangePlus **1,734,332 B**. All five guards compile out. 10 codec tests pass. 29 `CAN_Dn_PNU_*` parameters, none over 16 characters  
- For 5.1.13 version :  
   - **CAN Update Plan step 1c : the gimbal lifecycle state machine**  
   - **INIT -> ALIGN -> RUN, plus MAINTENANCE and FAULT.** `HOMING` is in the enum and **deliberately not implemented** - that is step 1d, and until the Appendix 1 endpoint search runs, **the zero must be set by hand on the bench**  
   - **It lives one layer BELOW the mount**, in `AP_PNUGimbal_Backend`: it runs in the CAN thread where feedback arrives, so a fault transition happens *on the frame* rather than a 50 Hz mount tick later; it survives `HAL_MOUNT_ENABLED=0` (D13); and a second motor protocol would need the same lifecycle unchanged  
   - That meant **moving the per-axis target and feedback storage, the mechanical limits and the stale-feedback policy up out of `AP_PNUGimbal_CubeMars`**, which now owns the wire format and nothing else - every remaining member is a CAN id, a bit layout or a DLC  
   - **The lifecycle is NOT the mount mode.** `MAV_MOUNT_MODE_*` says *where to point*; the state says *whether the hardware can be pointed*. RETRACT is a targeting mode, not "motors off". Conflating them would have been the easy mistake  
   - **ALIGN exists because §9 requires position control to start from the measured angle.** It seeds the command from feedback so the first mount target is approached from where the mount *is*, instead of arriving as a step from zero - with a heavy SAR payload that is the one thing here that could break hardware  
   - **New `CAN_Dn_PNU_SLEW`** (deg/s, default 45, provisional) bounds the approach *and* every later move. One mechanism, two jobs, cheaper than ramping stiffness  
   - **New `CAN_Dn_PNU_MAINT`** takes the FC completely off the bus so a PC can configure a motor over UART. Entering is immediate; **leaving requires a reboot**, and the state is latched at `init()` for exactly that reason - the utility can change the CAN id, the mode and the encoding ranges, and a mismatch *silently rescales everything*  
   - **Arming interlock : `healthy()` is now true only in RUN with every axis fresh.** `AP_Mount::pre_arm_checks()` fails on `!healthy()` as `Check::CAMERA`, so an unhomed or faulted mount **cannot be armed**. Override is `ARMING_SKIPCHK` **bit 16** (Camera, decimal 65536) - the same escape hatch D9(b) established for the PMU  
   - **Also fixed, and it predated this step : `AP_PNUGIMBAL_ENABLED=0` did not compile**, because `AP_PNU_CAN` sizes its parameter arrays from `AP_PNUGIMBAL_NUM_AXES`. The axis identity moved out of the enable guard into `AP_PNUGimbal_config.h` - an aircraft with a PMU and a SAR but no gimbal is a real configuration, and AGENTS.md requires the base system to compile with optional features disabled  
   - Builds : CubeOrangePlus **1,730,780 B**. **All five guards compile out** - `AP_PNU_CAN_ENABLED=0` 1,717,984 / `AP_PNUGIMBAL_ENABLED=0` 1,719,824 / `AP_PNUGIMBAL_CUBEMARS_ENABLED=0` 1,721,392 / `HAL_MOUNT_PNUGIMBAL_ENABLED=0` 1,724,660 / `HAL_MOUNT_ENABLED=0` 1,656,240. The 10 codec tests still pass  
- For 5.1.12 version :  
   - **CAN Update Plan step 1b : `AP_Mount_PNUGimbal`, the mount backend**  
   - **`MNT1_TYPE = 15`** selects a new `AP_Mount` backend for the PNU 3-axis gimbal. `AP_Mount` already does FC-attitude stabilisation and **needs no camera**, so the backend inherits mount modes, RC targeting, the MAVLink gimbal manager, ROI, the `MNT1_*_MIN/MAX` limits and the lead filter, and adds only the stabilisation maths  
   - **IT COMMANDS NO MOTORS.** Targets are written all the way through to `AP_PNUGimbal_CubeMars`, but that backend transmits nothing until `set_enable(true)` and **nothing in the tree calls that yet** - the lifecycle state machine is step 1c. The whole path is exercised with the motors silent  
   - **Provable from a log with no new logging code.** `AP_Mount_Backend::write_log()` already writes `MNT`, whose *desired* columns are the earth-frame target and whose *actual* columns this backend fills with the body-frame demand. **`MNT.DRoll` against `MNT.Roll` is the AHRS correction**  
   - **Limits split across two layers, deliberately.** `MNT1_ROLL/PITCH/YAW_MIN/MAX` carry the **operating** limits (±25/±25/±15) and `AP_Mount` clamps to them; the new **`CAN_Dn_PNU_MRLL/MPIT/MYAW`** carry the **mechanical** limits (±30/±30/±20) and the gimbal backend clamps again as the last thing before a frame is built. A bug in the targeting layer cannot drive the payload into the landing skid struts  
   - **The mechanical clamp means nothing until homing runs.** The encoder is relative and its zero is assumed not to persist, so until step 1d ±30 in encoder terms is not ±30 mechanically. `init()` also rejects a mechanical limit larger than `PNU_PMAX`, which would be unreachable and leave the stops as the only protection  
   - **Found while implementing : ArduPlane has been unbuildable on this branch since `ed055d2f24` (5.0.10)** - `GCS_PNU.cpp` references `PMU_Ctrl_Echo` and `AP::ac_avoid()`, both Copter-only. A vehicle gate on the new backend was written and then **removed as unnecessary**: it would have protected a build configuration this branch has never supported. See the note below  
   - Builds : CubeOrangePlus **1,729,440 B**; `HAL_MOUNT_PNUGIMBAL_ENABLED=0` builds at 1,728,304 B so the backend costs **1,136 B**, and `HAL_MOUNT_ENABLED=0` still builds at 1,659,580 B (PNU-ISSUE D13 intact). Registered in `build_options.py` as `Gimbal / PNUGIMBAL`. The 10 codec tests still pass  
- For 5.1.11 version :  
   - **CAN Update Plan step 1 : `AP_PNU_CAN`, the multi-backend CAN driver**  
   - **Three protocols, one bus, one driver.** `AP_CANManager` binds each interface to exactly **one** driver - one interface to many drivers is not supported, and a driver with no interface is never allocated at all, so it fails *silently*. `CAN_Dn_PROTOCOL2` does not help: it is a DroneCAN-hosted auxiliary slot taking one `CANSensor` with no thread of its own  
   - `AP_PNU_CAN` owns the interface, the thread, the parameters and the one place a frame reaches the bus; `AP_PNU_CAN_Backend` owns a wire format and nothing else. `AP_PNUGimbal_Backend` sits between them as the gimbal-shaped view, so a control layer can drive any motor protocol without naming CubeMars  
   - **`handle_frame()` now returns `bool`** and the driver stops at the first backend to claim a frame - the `MultiCAN` pattern upstream already uses for the CAN rangefinder. **An unclaimed frame is counted and reported**, which is how a mis-assigned CAN id gets noticed instead of vanishing  
   - **`send_frame()` takes the FULL 32-bit id** including `FlagEFF` / `FlagRTR`, because the PMU polls with extended RTR frames and the gimbal sends standard data frames. The shape of a frame is the backend's business  
   - **`update()` is handed the driver's tick count**, so every backend derives its rate from one clock. The PMU's 10 Hz turned out to be an accident of an undocumented command rotation; that must not happen twice  
   - **The PMU is untouched on protocol 15 and nothing that flies changes in this version.** `AP_PNU_CAN` is on protocol 16 (`PNUGimbal` renamed `PNU_CAN`) so the gimbal stays benchable while the PMU conversion waits for hardware  
   - **The name carries an underscore on purpose : `PNU_CAN`, never `PNUCAN`.** The two drivers sit beside each other everywhere an operator looks - `15:PMUCAN` and `16:PNU_CAN` in the same `CAN_Dn_PROTOCOL` dropdown, `PMUCAN:` and `PNU_CAN:` statustexts on the same GCS screen - and **M and N are one letter apart in a sans-serif font**. The underscore is the only thing making them distinguishable at a glance. Do not "tidy" it away  
   - **New `CAN_Dn_PNU_*` parameters replace 5.1.10's `CAN_Dn_PG_*`**, including **`CAN_Dn_PNU_OPTS`**, the device bitmask - bit 0 PMU, bit 1 gimbal, bit 2 SAR, **default 1**  
   - **One flat group, not the sibling subgroups the plan assumed.** `AP_SUBGROUPPTR` resolves `var_info` from the *class* (`AP_Param.h:165`), so two subgroups pointing at this one object would get the same table, and nesting eats so much of the 16-character budget that the gimbal parameters become unreadable. All 12 names fit under `CAN_Dn_PNU_`  
   - **Parameter index 7 - the old `PG_` group - is retired and NOT reused.** AP_Param stores by index rather than by name, so a bench aircraft that saved a `PG_` value would have it read back as a different parameter  
   - **`CAN_Dn_PNU_TYPE` drops `0:None` and defaults to `1:CubeMars`.** Bit 1 of `PNU_OPTS` now answers whether a gimbal is fitted. Two switches for one thing is how a mount ends up silently disabled by the one nobody looked at  
   - **A one-per-boot line names the running backends** - `PNU_CAN: CAN_D1 Gimbal ready, no PMU`. That is what makes a forgotten `PNU_OPTS` bit visible: with bit 0 clear, `PMU_Ctrl_Echo.PMUCAN_Fail` stays 0 and reads as **healthy**, so a missing PMU is otherwise completely silent  
   - Builds : SITL copter clean, CubeOrangePlus **1,727,948 B**; with `--define AP_PNU_CAN_ENABLED=0` it still builds at 1,722,636 B, so the driver costs **5,312 B**. The 10 CubeMars codec tests still pass unchanged  
- For 5.1.10 version :  
   - **`AP_PNUGimbal` gets a protocol backend : `AP_PNUGimbal_CubeMars`**  
   - **The structure deviates from `AP_PMUCAN` deliberately.** The 5.1.9 skeleton was one class that would have held one protocol. It is now a **frontend plus an `AP_PNUGimbal_Backend`**, because the mount is expected to accommodate more than one motor type and only the protocol differs between them  
   - The **frontend** keeps what does not depend on which motor is fitted : the interface, the thread, the parameters, the axis mapping, the public `set_target()` / `get_feedback()` interface, and the **one place a frame reaches the bus**. A **backend** owns the wire format and nothing else - it never touches `_can_iface`  
   - **`AP_PNUGimbal_CubeMars`** implements the CubeMars GLII family protocol from `libraries/PNUGimbal_CAN_Protocol_Type_CubeMars.md` : all three control modes (MIT, position/velocity, velocity), the four universal commands (enable / disable / zero / clear errors), and the common feedback frame  
   - **PROTOCOL LAYER ONLY.** There is no stabilisation loop and this is **not** an `AP_Mount` backend - that question is still open and the rule below still stands : decide it before writing the loop, not after. `set_target()` exists and **nothing in the tree calls it**, so the driver transmits nothing until a control layer does  
   - **New parameters, `CAN_Dn_PG_*`** - a subgroup on `CANDriver_Params` at index 7, the same mechanism PiccoloCAN uses for `PC_`. `CAN_Dn_PG_TYPE` defaults to **0 (None)**, so `CAN_Dn_PROTOCOL = 16` on its own still does nothing : selecting a CAN protocol must not start commanding motors  
   - The five encoding limits (`PMAX`/`VMAX`/`KPMAX`/`KDMAX`/`TMAX`) are **parameters, not constants**, because they MUST match what the vendor PC utility wrote to the motors. A host/device mismatch **does not fail** - it silently rescales every command and every reading, which is the worst failure mode this protocol has  
   - **The codec is verified against the manual's own published command vectors**, in `libraries/AP_PNUGimbal/tests/` : four MIT, two position/velocity, two velocity, plus saturation, round-trip and feedback decode. 10 tests, all passing. Two of the manual's MIT vectors are **not** reproduced and cannot be - its ±6 rad/s rows do not decode to ±6 at the ±200 range it states. That divergence has its own test, so a bench disagreement points at the manual rather than at this code  
   - **Feedback carries only four bits of motor identity** (the upper nibble of D0 is the status code), so motor IDs are restricted to **1..15 and required distinct** at init, and all three motors share one Master ID. Do not be tempted to read all eight bits of D0 as an ID - the manual's prose says you can and its own table says you cannot  
   - **Nothing is commanded until a target is set, and nothing while disabled.** §9 of the protocol note requires position control to start from the measured current angle, so a zero-initialised target must never reach the wire. `set_enable(false)` also invalidates the targets, because the mount will have moved while disabled  
   - **What the manual does not specify is not invented** : no command period, no feedback rate, no loss timeout, no watchdog interval, no acknowledgment, no runtime mode switch. All seven are listed at the top of the backend so a later session does not fill them in with a plausible default  
   - `AP_PNUGIMBAL_LOOP_HZ` is **still 100 and still unchecked** - the manual specifies no command period at all, so it needs measuring  
   - Builds : SITL copter clean, CubeOrangePlus **1,727,364 B**. With `--define AP_PNUGIMBAL_ENABLED=0` it still builds, at 1,722,636 B - so the whole driver, skeleton included, costs **4,724 B** of flash and the guard is intact  
- For 5.1.9 version :  
   - **skeletons for two new CAN drivers : `AP_PNUGimbal` and `AP_PNUSAR`**  
   - Both speak **private CAN protocols, NOT DroneCAN**, in the same shape as `AP_PMUCAN` - their own thread created in `init()`, nothing on the vehicle scheduler  
   - Registered as `AP_CAN::Protocol` **16** (`PNUGimbal`) and **17** (`PNUSAR`), so `CAN_Dn_PROTOCOL = 16` or `17` selects them. Slots in `AP_CANManager`, factory cases behind `#if`, and both listed in `ArduCopter/wscript`  
   - **STRUCTURE ONLY** at 5.1.9; `rx_spin()`, `tx_spin()`, `handle_frame()` and `send_frame()` were stubs marked `TODO`, with no parameters, no `CAN_Dn_PROTOCOL` `@Values` entry and no vehicle-side consumers. **`AP_PNUGimbal` is implemented as of 5.1.10; `AP_PNUSAR` is still a skeleton**  
   - The skeletons carry this project's own lessons rather than copying `AP_PMUCAN` verbatim : **DLC validation before reading any field** with a rate-limited short-frame report (D6), **initialisers on every member** (D1/D10), a note **not to add write-only counters** (D6), and a reminder to **confirm endianness and say so at the site** if no byte swap is needed (D15)  
   - **Adding protocol enum values breaks exhaustive switches** - `-Werror=switch` caught `AP_Arming.cpp`, which now lists both beside PMUCAN's existing no-op case. Values 16/17 could collide with future upstream protocol numbers, exactly as `PMUCAN = 15` might; watch this on the next ArduPilot bump  
   - **Design intent recorded in both headers** (2026-10-15), see below - `AP_PNUGimbal` drives three CubeMars motors and carries **no camera**; it stabilises the SAR device that `AP_PNUSAR` will drive  
- For 5.1.8 version :  
   - **backward-compatibility fix for KGCS; D10 is FC-complete**  
   - **Tracking, video source and IR palette no longer report as failures on a Gremsy.** They had already left the FC for GCS-direct routing, but still produced `CAM: mount rejected ...` **every 10 s, forever**, because the GCS keeps sending TC2 and nothing on that payload can serve them  
   - That mattered beyond the wording : `report_cam_unsupported()` shares one rate limiter, so three permanent warnings would have **crowded out genuine faults** - destroying the signal D10 Stage 1 exists to provide  
   - All four GCS-direct functions now announce themselves **once** at `MAV_SEVERITY_INFO` and then stay silent : `CAM: <function> is GCS-direct on this mount`  
   - **A Viewpro that genuinely declines still warns** - the gate is on the payload, not on the return value, so a real fault on a payload that should work is never silenced  
   - **Verified with no code needed : KGCS-to-VIO routing already works.** `get_targets()` reads the generated message-entry table; `PARAM_EXT_SET` and `COMMAND_LONG` both carry target fields and are in the staged dialect; a message addressed to the camera's component falls through to `forward()`  
   - All seven PNU-KAL messages checked and unaffected - none carries a target field, so they are broadcast and FC-handled  
   - **Supported GCS and gimbal combinations recorded** (new section at the end of this file). KGCS whitelists MAVLink message ids and is **Viewpro-only**; PNU-GCS replaces it and works with any gimbal, over either the KGCS interface or standard MAVLink2. **KGCS + Gremsy is not supported - do not retest it**  
   - Two consequences, both meaning *no* code or parameter change : the msg 285 suppression is **correctly keyed per backend** (KGCS only ever flies a Viewpro, which suppresses it), and **`NO_FORWARD` must NOT be set on the VIO link** - `PARAM_EXT_ACK` carries no target fields, so acks would be blocked and every palette/source change would look like a failure  
   - Renamed `KGCS-direct` to **`GCS-direct`** in the four notices and in the code (`GCS_DIRECT_*`, `report_gcs_direct()`) - the ground station driving them is PNU-GCS. "KGCS interface" now means the TC2/TM2 ICD; "KGCS" means the product  
- For 5.1.7 version :  
   - **Stage 3 bench-confirmed; the FC-side work on D10 is COMPLETE**  
   - **`zoomLevel` follows the active EO/IR source** - bench-measured. Switching the main source switches the reported value, so the camera does the selection itself and no sensor check was needed in the FC  
   - This was the last thing that could have made Stage 3 wrong : `AP_Mount_Viewpro` is sensor-aware internally, so had the VIO reported EO zoom while the operator watched IR, TM2 would have carried a number that was not what it claimed to be  
   - Both ranges fit TM2's `int8_t` field : EO reaches 30x in super-resolution (matching the ICD's `MIN:1, MAX:30`), IR reaches 8x. Only combine mode at 240x would overflow, and it is not used  
   - **The bench aid is kept but disabled** : `AP_CAMERA_MAVLINKCAMV2_ZOOM_DEBUG` defaults to **0**. The decode is permanent, so only the reporting is conditional and the off build is byte-identical to having no debug code at all  
   - A more expensive route using the payload app on component 26 was considered and is **not needed** - recorded in the README because the idea is sound and will come up again  
- For 5.1.6 version :  
   - **D10 Stage 3 done; the FC-side work on D10 is COMPLETE**  
   - **TM2's `Zoom_POS_FB` now carries real zoom on a Gremsy.** Bench measurement showed the VIO reports `CAMERA_SETTINGS.zoomLevel` in **zoom times**, not the percentage the MAVLink spec defines - 1.00 at wide, 30.00 at full tele in SR mode, matching TM2's ICD range of 1..30 exactly  
   - That removed both obstacles at once : no percentage conversion, and no `int8_t` overflow, since combine mode (240x) is not used  
   - **Found while implementing : `CAMERA_SETTINGS` was never routed to `AP_Camera` at all**, so any backend decode would have been dead code. Fixed with **one `case` label** in `GCS_Common.cpp` - the only core-file cost  
   - Implemented in three layers so **no shared library learns a Gremsy-specific fact** : `AP_Camera` reports the raw number and says nothing about its units; only `AP_Q30` decides it means times  
   - Bonus, vendor-neutral : `send_camera_settings()` now forwards the camera's real values instead of upstream's `NaN` placeholders, so a standard GCS no longer sees "zoom unknown" while the camera is reporting it  
   - **The whole of it is worth submitting upstream** - ArduPilot currently cannot read camera zoom at all, and none of the fix is PNU-specific  
   - What remains on D10/D11 is **KGCS implementation only** : EO/IR source, IR palette, object tracking and absolute zoom are all KGCS-to-payload direct, needing no firmware  
- For 5.1.5 version :  
   - **D10 Stage 2b closed; the FC-side work on D10 is essentially complete**  
   - **Stage 2b is closed by NOT implementing it in the FC.** `AP_Q30` sends an absolute zoom only to a Viewpro, which reads `ZoomType::PCT` as zoom times. On any other mount it stays silent and says so once per boot  
   - Reason : `ZOOM_TYPE_RANGE` is a percentage of a **mode-dependent** range - 240x combine / 30x super-resolution on a Gremsy VIO, selected by an extended parameter **the FC cannot read**, since ArduPilot implements no `PARAM_EXT_*`. KGCS drives absolute zoom directly with `MAV_CMD_USER_4` "zoom to level", in times  
   - A rate-pulse fallback was **rejected** : it would fight the command KGCS is sending. Two sources commanding one camera is worse than one source doing less  
   - **`VIO_MAVLink_Messages.md`** was added to this folder - the message reference extracted from Gremsy PayloadSDK 3.2.0 source. It answers both Stage 2b and Stage 3, and supplies the `C_SOURCE`, `C_T_PALETTE` and tracking mappings the GCS-direct work needs  
   - What remains is **KGCS implementation**, against mappings that are all documented, plus the Stage 3 decision on where TM2 gets its zoom value  
- For 5.1.4 version :  
   - **First version validated against real Gremsy VIO hardware**  
   - **D10 Stage 2a validated on hardware with no code change** : with the VIO's camera component set to 100 and `CAM1_TYPE=6`, ArduPilot finds the camera and zoom works through Mission Planner  
   - **Both remaining ACTION ON PNU questions are now ANSWERED** : gimbal `GREMSY`/`VIO` fw 7.8.7.0, camera `Gremsy`/`Vio Payload G1` fw 18.4.1.3, and `CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM` is proven by zoom reaching the camera at all  
   - **Fixed an upstream defect found on the bench** : the `"Camera: ..."` statustext was printed on every `CAMERA_INFORMATION`, and the VIO broadcasts that message unsolicited, so it spammed one statustext per broadcast to every link. Now printed once. Worth submitting to ArduPilot rather than carried as a divergence  
   - **Gremsy firmware defect recorded** : the yaw fields of `GIMBAL_DEVICE_INFORMATION` are sent in degrees where msg 283 specifies radians, giving `MNT1_YAW_MAX` a default of 18334 = `degrees(320)`. Roll and pitch are correct. Workaround is to set `MNT1_YAW_MIN`/`MAX` explicitly  
   - **The dual-GCS compatibility strategy is now hardware-validated** : slew, Set Neutral, point-down and zoom all work from Mission Planner against a Gremsy while KGCS convention stays confined to `AP_Q30`  
   - **Aircraft settings required** - see the release note in `ArduCopter/version.h` for the full list : pitch limits in physical convention, yaw limits set explicitly on a Gremsy, `CAM1_TYPE` 4 for Viewpro / 6 for Gremsy, and a Gremsy VIO's camera component set to 100 rather than its default 101  
- For 5.1.3 version :  
   - **D10 Stage 2a applied : camera calls routed through `AP::camera()`**  
   - The six camera functions (zoom, focus, record, picture, tracking, video source) now go through `AP::camera()` instead of `AP::mount()`, so **`CAM1_TYPE` selects the destination** - 4 (Mount) reaches a Viewpro exactly as before, 6 (MAVLinkCamV2) reaches a Gremsy camera component the mount cannot address at all  
   - It is **not** the dual-dispatch the earlier plan assumed : `AP_Camera_Mount` already forwards to `AP_Mount`, so one route serves both payloads. `get_zoom_times()` and `IR_Color_Change()` stay on `AP::mount()` - they have no `AP_Camera` equivalent  
   - **REQUIRES `CAM1_TYPE` to be correct (4 for Viewpro).** It is now load-bearing for the KGCS camera path, where before it only served a standard GCS. A wrong or zero value silently disables KGCS camera control  
   - Zoom units deliberately **not** converted - that only bites at `CAM1_TYPE=6` and needs the VIO's maximum zoom, which MAVLink cannot report. Deferred as Stage 2b  
   - **Still outstanding from 5.1.2 : `MNT1_PITCH_MIN` -45 -> -90 and `MNT1_PITCH_MAX` 90 -> 45** on the aircraft, for the pitch relocation  
- For 5.1.2 version :  
   - Analysis and groundwork, NOT Stage 2 (the D10 routing was still blocked at that point)  
   - The ACTION ON PNU regarding Gremsy protocol is **partly** answered and recorded in README.md along with analysis on this issue. IR palette, EO/IR source and camera component ID are answered; `CAMERA_CAP_FLAGS` and the vendor/model strings are still open  
   - Pitch reversal is now changed to accommodate the general convention of gimbals, and the previous pitch reversal now resides in AP_Q30.cpp. This fixes inverted pitch for Mission Planner and AUTO missions on a Viewpro  
   - **REQUIRES a parameter change on the aircraft : `MNT1_PITCH_MIN` -45 -> -90 and `MNT1_PITCH_MAX` 90 -> 45.** The firmware cannot do this itself. Flying with the old pair clips KGCS pitch to 45 deg down and allows 90 deg up, past the mechanical stop  
   - Dual-GCS compatibility strategy recorded : a standard GCS (Mission Planner, QGC) keeps full gimbal and camera control, and KGCS-specific convention lives in AP_Q30. See the section at the end of this file  
- For 5.1.1 version :  
   - Stage 1 applied : (see below); the routing decision itself stays blocked on the ACTION ON PNU answers  
   - README.md notes are updated to indicate dev version  
***
Residuals recorded against resolved issues

- D1 — is the IR prelude needed at all? Requires removing it and retesting on hardware.  
- D6 — five PMU counters are still write-only; decide to surface (a PMUC log msg) or delete. _rtr_tx_err / _cmd_tx_err have the real diagnostic value.  
- D7 — accepted interlock residuals (no sender check on handle_gcs_flcc_pmu_ctrl(); the 2-messages-plus-1s shortcut rests on a KGCS contract not enforced in firmware). **Step 2c addresses the shortcut**: a link dropping mid-burst currently completes the start on its own ~1 s later, because the frozen command pair keeps the counter running. The 500 ms staleness reset deliberately **tightens D7's unlimited loss tolerance** - recorded here rather than left as a silent change.  
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
- **This branch builds ArduCopter ONLY.** ArduPlane, Rover and Sub have been unbuildable since `ed055d2f24` (5.0.10) because `GCS_PNU.cpp` lives in the shared `GCS_MAVLink` library and references `PMU_Ctrl_Echo` and `AP::ac_avoid()`, both of which exist only for Copter. 5.1.11 added `AP_PNU_CAN::var_info` to the same list. This is **by construction, not an accident** - but it means a vehicle gate on PNU code in a shared library protects nothing, so do not add one  
- PNU-KAL Specific options  
   - **WARNING :** Do not use heli model. Only multicopter frames are supported  
   - CAN Driver Option for the PNU bus : **CAN_D1_PROTOCOL = 15** (PMU, gimbal and SAR share it; `CAN_D1_PNU_OPTS` selects which are fitted, default 1 = PMU only)  
   - **CAN1 bitrate is 1,000,000 bit/s** (`CAN_P1_BITRATE` default). Forced by the CubeMars gimbal, which is fixed at 1 Mbit/s; **the PMU must be reconfigured from 500 k on the device**. See the CAN Update Plan Steps  
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
| D8 | **Mount scheduler rate vs. the Viewpro self-throttle.** (a) *RESOLVED (step 5)* - msg 285 suppression is now an opt-in backend capability (`suppress_gimbal_device_attitude_status()`), so only Viewpro suppresses it; every other gimbal keeps standard MAVLink behaviour. **Confirmed correctly keyed 2026-10-14** - KGCS only ever flies a Viewpro, so it never sees 285; see "Supported GCS and gimbal combinations". (b) *RESOLVED (step 5)* - V5.0.8 lowered the `AP_Mount` task 50&rarr;10 Hz; **deliberately reverted to 50 Hz**. `AP_Mount_Viewpro::update()` self-throttles to `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS` (100 ms, upstream), so gimbal traffic is 10 Hz at *either* scheduler rate - the scheduler does not control it. At 10 Hz the scheduler period **equals** that throttle interval, so any late tick defers the update a full period (200 ms &rarr; 5 Hz bursts); 50 Hz oversamples it 5x (worst case 120 ms). 50 Hz also keeps `AP_Mount_Backend::update()` - servo retract and `update_poi_lock_target()`, which run *before* the throttle - at full rate. | `Copter.cpp` | — | Done. To reduce gimbal traffic, raise `AP_MOUNT_VIEWPRO_UPDATE_INTERVAL_MS`, not the scheduler rate |
| D9 | **(b) CORRECTED 2026-10-19 - the ARMING_SKIPCHK bypass it relied on never existed; fixed in 5.1.16.** **RESOLVED (2026-09-30) - both parts.** Both "no PMU" paths correctly avoid a false trigger (`PMUCAN_Fail` stays 2 when the driver runs with no PMU; stays 0 from static init when `CAN_Dn_PROTOCOL` is not 15). (a) *step 6* - `failsafe.pmucan` was set but never cleared, so the failsafe could fire at most once per power cycle. It now clears on recovery with `ERROR_RESOLVED`, matching `failsafe.terrain` / `.deadreckon` / `.ekf`. The mode change is deliberately **not** undone, matching upstream precedent. (b) the failsafe could not tell "PMU lost in flight" from "armed without one", so an emergency launch after a PMU dropout was disarmed ~100 ms after arming, with no prearm warning. | `events.cpp`, `AP_Arming_Copter.{h,cpp}`, `Copter.h` | — | **(b) implemented as designed, no new parameter.** `AP_Arming_Copter::arm()` latches `failsafe.pmucan_armed_healthy = (PMUCAN_Fail == 0)`, and `failsafe_pmucan_check()` returns early unless it is set - so losing the PMU in flight still triggers the failsafe, while arming deliberately without one flies on unmolested. `AP_Arming_Copter::pmucan_checks()` warns before arming as a `Check::SYSTEM`, bypassable via `ARMING_SKIPCHK` bit 13 (System, decimal 8192). **One deviation from the written design:** it lives in `AP_Arming_Copter`, not `AP_Arming::can_checks()` - `AP_PMUCAN` is listed only in `ArduCopter/wscript`, so referencing `PMU_Ctrl_Echo` from the shared library would fail to link for Plane, Rover and Sub |
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
| `Forced Submodule File/pnu_kal.xml` | yes | the 9 message definitions (7 PNU-KAL, plus the SAR's TC50/TM50 from 5.1.20) |
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


**KGCS pitch reversal is intentional - do not "fix" it. It lives in `AP_Q30`.**

*Relocated 2026-10-12 (see "Dual-GCS compatibility strategy" at the end of this file).*
It used to live in `AP_Mount_Backend::set_angle_target()` behind a
`pitch_target_is_reversed()` backend hook that only `AP_Mount_Viewpro` set. Because that
function is the common funnel for **every** angle source, the KGCS convention also reached
Mission Planner, QGC and AUTO missions, which therefore got **inverted pitch** on a Viewpro.
The hook is gone; both KGCS paths now negate at their own call sites in `AP_Q30`.

This is a KGCS operator convention, not a protocol requirement: when the operator presses
gimbal pitch-down they expect the *video image* to pan downwards, which means the gimbal
must physically pitch up.

| Path | Where the flip happens | Reversed |
|---|---|:-:|
| **KGCS angle (TC2 `Control_Mode` 2)** | `AP_Q30::send_cmd_angle()` negates `Pitch_Angle_CMD` at the call site | **yes** |
| **KGCS rate (TC2 `Control_Mode` 1)** | `AP_Q30::send_cmd_speed()` negates `Pitch_Speed_CMD` at the call site | **yes** |
| **TM2 pitch readback** | `GCS_PNU.cpp` negates `pitch` before filling `Pitch_REL_ANG` / `Pitch_IMU_ANG` | **yes** |
| MAVLink angle commands (`DO_MOUNT_CONTROL`, `GIMBAL_MANAGER_*`, AUTO mission) | — | no - **standard physical convention** |
| RC stick control | — | no - physical convention |
| ROI / point-at-location | — | no - physical convention |
| Rate integration into an angle target | — | no - operates on the already-correct rate |

All three KGCS paths are now unconditional negations in the KGCS adapter, with no backend
dependency, so they behave identically on Viewpro, Gremsy or any future gimbal. **Do not
re-add a sign flip to `AP_Mount_Backend`** - a comment at `set_rate_target()` says so. It
would double-negate the KGCS paths and invert pitch for every standard GCS.

`AP_Q30::send_cmd_hold_angle()` sends an all-zero rate, so the sign is irrelevant there.

**The pitch limits are now physical, i.e. standard ArduPilot convention.**
`set_angle_target()` constrains the value `AP_Q30` already negated, so `MNT1_PITCH_MIN` /
`MNT1_PITCH_MAX` mean what ArduPilot documents them to mean, and Mission Planner's own
limit handling is correct:

```
MNT1_PITCH_MIN = -(degrees of DOWN travel allowed)   (negative)
MNT1_PITCH_MAX = +(degrees of UP travel allowed)     (positive)
```

Measured on the bench (Viewpro, 2026-09-29): up saturates near **60 deg physical**, so
`MNT1_PITCH_MAX` must stay at or below 60.

**Required values. These MUST be changed together with the relocation** - the old pair was
expressed in the old (commanded, pre-negation) convention, and leaving it in place clips
KGCS pitch to 45 deg down and allows 90 deg up, past the mechanical stop:

| Parameter | Was (old convention) | **Now** | Gives |
|---|:--:|:--:|---|
| `MNT1_PITCH_MIN` | -45 | **-90** | 90 deg **down** - full nadir |
| `MNT1_PITCH_MAX` | 90 | **45** | 45 deg **up** |

The range is unchanged in physical terms - the numbers were negated and swapped, which is
exactly what moving the negation upstream of the constraint requires. The 45 is the same
deliberate ~15 deg margin inside the measured 60 deg up stop - do not "optimise" it to 60,
that is the saturation point where the gimbal raises an internal error.

Note the limits apply to the **angle** path only. Viewpro declares
`NATIVE_ANGLES_AND_RATES_ONLY`, so rate commands go straight to
`send_target_rates()` and are never constrained - only the gimbal's own firmware
stops them. The RC path now applies these parameters in the **same** physical convention as
the limits themselves, so RC and the limits no longer disagree.

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
| 2 | **APPLIED 2026-10-12 as Stage 2a (routing only).** Not dual-dispatch after all - a single route through `AP::camera()`, with `CAM1_TYPE` selecting mount or MAVLink camera. Zoom units deliberately deferred | nothing (the units half needs a VIO) |
| 3 | **DONE 2026-10-13** - `get_zoom_times` resolved via `CAMERA_SETTINGS`; `IR_Color_Change` left the FC entirely as a `C_T_PALETTE` extended parameter | - |

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
  constructor. **Fixed in Stage 1**, which also adopted the `is_positive()` validity test
  suggested here: EO and IR zoom are both >= 1x, so the 0 it is now initialised to cannot
  collide with a real reading.

  **Correction, 2026-10-13.** This note, and D1's beside `_palette_pending_color`, both said
  such a member is *indeterminate* at boot. That is wrong in this codebase: mount and camera
  backends are allocated with `NEW_NOTHROW`, and `AP_Common/c++.cpp:45` implements
  `operator new(nothrow)` as **`calloc`**, so the whole object arrives zeroed. The
  initialisers are therefore **defensive hygiene, not bug fixes** - they remove a dependency
  on allocator behaviour that nothing states as a contract, and they keep working if an
  object is ever placed on the stack or allocated another way. Keep them; just do not go
  hunting for the boot-time garbage the earlier wording implied.
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

**Stage 2a as applied, 2026-10-12 - routing only, units deferred.**

**The design changed once the Gremsy notes were checked against the tree. Stage 2 is NOT
dual-dispatch.** `AP_Camera_Mount` - what `CAM1_TYPE=4` selects - already forwards
`set_zoom`, `set_focus`, `record_video`, `trigger_pic`, `set_tracking`, `set_lens` and
`set_camera_source` straight to `AP_Mount`. So a single route through `AP::camera()` serves
both payloads and `CAM1_TYPE` does the switching, which is what that parameter is for. A
try-mount-then-fall-back-to-camera scheme would have been actively pointless on the current
aircraft: the fallback calls `AP_Camera_Mount`, which calls back into the very mount that
just failed.

| `CAM1_TYPE` | `AP::camera()` resolves to | Result |
|:--:|---|---|
| 4 (Mount) - **today's Viewpro config** | `AP_Camera_Mount` -> `AP_Mount` -> Viewpro | **behaviourally identical to before** |
| 6 (MAVLinkCamV2) - Gremsy | `AP_Camera_MAVLinkCamV2` -> VIO camera component | the new path |

| Moved to `AP::camera()` | Stayed on `AP::mount()` |
|---|---|
| `set_zoom` (8 call sites), `set_focus` (2), `record_video` (2), `take_picture`, `set_tracking` (2), `set_camera_source` (4) | `set_angle_target`, `set_rate_target` x2 (gimbal); `get_zoom_times`; `IR_Color_Change` (6) |

`get_zoom_times` and `IR_Color_Change` stayed because **neither exists in the `AP_Camera`
interface at all**. Both are Viewpro-only and silently do nothing on any other gimbal - on a
Gremsy the palette is a `C_T_PALETTE` extended parameter that KGCS sets directly on the
payload, and zoom readback is Stage 3.

*Points worth not re-deriving:*

- **`AP_Camera::CameraSource` matches the magic numbers `AP_Q30` already used** - `DEFAULT=0`,
  `RGB=1`, `IR=2` - and `AP_Mount.h:306` documents its own `uint8_t` as that enum cast. The
  migration was a type change, not a value change.
- **`set_camera_source` is wrapped in `AP_Q30::set_video_source()`** because
  `AP_CAMERA_SET_CAMERA_SOURCE_ENABLED` resolves to `SIYI || XACTI || VIEWPRO` - a future
  Gremsy-only build would not have the function at all. Wrapping it once keeps the `#if` out
  of the four call sites. The previous `mount->set_camera_source()` calls were unguarded and
  would have failed to build in that configuration.
- **`AP_Q30_ENABLED` now requires `AP_CAMERA_ENABLED`** as well as `HAL_MOUNT_ENABLED`, since
  the library now needs both singletons. Verified: `--define AP_CAMERA_ENABLED=0` builds at
  1,695,892 B, and `--define HAL_MOUNT_ENABLED=0` still builds at 1,653,160 B.

> **NEW OPERATIONAL DEPENDENCY - `CAM1_TYPE` now matters to KGCS.** Before Stage 2a the KGCS
> camera path depended only on `MNT1_TYPE`; `CAM1_TYPE` existed solely so a standard GCS could
> drive the camera. Now a wrong or zero `CAM1_TYPE` silently disables KGCS camera control on a
> Viewpro that otherwise looks healthy. `CAM1_TYPE=4` is already the documented Viewpro setting
> (see the header of this file), so no change is needed on the current aircraft - but it is now
> load-bearing rather than optional, and Stage 1's return checks are what will report it as
> `CAM: mount rejected ...` if it is wrong.

**Units deliberately NOT converted - this is the "convert later" half.** `AP_Q30` still sends
zoom **times** as `ZoomType::PCT`, which works only because `AP_Mount_Viewpro::set_zoom()` was
locally modified to reinterpret PCT as times x10. Routing through `AP_Camera_Mount` lands in
that same modified function, so **Viewpro is unaffected**. The mismatch appears only at
`CAM1_TYPE=6`, where `AP_Camera_MAVLinkCamV2::set_zoom(PCT)` honours the real MAVLink 0-100 %
contract and 10x would become 10 %. Converting requires the target camera's maximum zoom, which
is `_Max_zoom_EO = 30` for Viewpro (hard-coded, itself unverified) and **unknown for a VIO** -
MAVLink has no message that reports zoom range. Decide it with the VIO on the bench, not before.

*Verified:* `./waf copter` clean, zero warnings, **1,721,584 B** (+240 B over 5.1.2's
1,721,344 B). **Not bench-tested** - no hardware was connected. The Viewpro no-regression claim
rests on `CAM1_TYPE=4` routing back into the identical code, which is a reasoning claim until
someone drives a gimbal with it.

**Stage 2b as closed, 2026-10-13 - the FC stops sending absolute zoom off a Viewpro.**

`AP_Q30::fc_handles_absolute_zoom()` returns true only when `AP::mount()->get_mount_type(0)`
is `Type::Viewpro`, which is the one backend whose `set_zoom()` reads `ZoomType::PCT` as zoom
**times** rather than the MAVLink 0-100 % contract. The `Tracking_CMD` 21-24 EO branch sends
its absolute zoom only in that case.

Anywhere else the FC sends **nothing** and notes once per boot:
`CAM: absolute zoom is GCS-direct on this mount`. One-shot and `MAV_SEVERITY_INFO` on
purpose - this is designed behaviour, not a failure, so it must not read like the Stage 1
`CAM: mount rejected ...` warnings.

*Why silence rather than a degraded fallback:* a rate pulse here would **fight** the
`USER_4` zoom-to-level command KGCS is sending directly. Two sources commanding the same
camera is worse than one source doing less.

`EO_zoom_times` keeps counting on both paths, so the cached value stays meaningful if the
mount changes. The IR branch is untouched - it sends `ZoomType::RATE`, which is continuous and
correct on every payload.

**Extended to the other three GCS-direct functions, 2026-10-13 - the backward-compatibility
fix.** Absolute zoom was the first function to leave the FC; tracking, video source and IR
palette had already left but were still reporting as failures. On a Gremsy they produced
`CAM: mount rejected tracking start` / `video source` / `IR palette` **every 10 s, forever**,
because KGCS keeps sending TC2 for backward compatibility and nothing on that payload can
serve those commands.

That was wrong in two ways. "Rejected" reads as a fault when it is the designed split. And
`report_cam_unsupported()` shares one rate limiter, so three permanent warnings would have
**crowded out genuine faults** - destroying the signal D10 Stage 1 existed to create.

All four now behave the same way: on a non-Viewpro payload the FC stays silent and says so
**once per function per boot**, at `MAV_SEVERITY_INFO`:

```
CAM: absolute zoom is GCS-direct on this mount
CAM: tracking is GCS-direct on this mount
CAM: video source is GCS-direct on this mount
CAM: IR palette is GCS-direct on this mount
```

*Named GCS-direct, not KGCS-direct: the ground station driving them is PNU-GCS. See
"Supported GCS and gimbal combinations".*

| TC2 | On a Viewpro | On a Gremsy |
|---|---|---|
| `Tracking_CMD` 1/2 - tracking | served by the FC; a decline is a real failure and still warns | announced once, then silent. KGCS uses `USER_4` to the payload app |
| `Tracking_CMD` 10-13 - video source | as above | announced once. KGCS sets `C_SOURCE` by `PARAM_EXT` |
| `Tracking_CMD` 14-19 - IR palette | as above | announced once. KGCS sets `C_T_PALETTE` by `PARAM_EXT` |
| `Tracking_CMD` 21-24 - absolute zoom | as above | announced once. KGCS uses `USER_4` zoom-to-level |

**A Viewpro that genuinely declines still warns**, through the unchanged
`report_cam_unsupported()` path - the gate is on the *payload*, not on the return value, so a
real fault on a payload that should work is never silenced.

*Shape of the change:* one predicate `mount_is_viewpro()` (renamed from
`fc_handles_absolute_zoom()`, since the same test now serves four functions for two different
reasons - PCT-means-times for zoom, sole implementor for the other three), one
`report_gcs_direct()` with a 4-bit mask, and three wrappers - `set_ir_palette()`,
`set_tracking_state()` and the existing `set_video_source()` - so the gate and the
`AP_CAMERA_SET_CAMERA_SOURCE_ENABLED` guard stay out of the twelve call sites. Those call
sites each collapsed to a single line.

**KGCS-to-VIO routing needs no FC change at all** - verified, not assumed.
`MAVLink_routing::get_targets()` reads the **generated** `mavlink_get_msg_entry()` table, so
any dialect message carrying `target_system`/`target_component` routes correctly with nothing
hard-coded. `PARAM_EXT_SET` (323) and its family are in the staged dialect and do carry those
fields; `COMMAND_LONG` for `USER_4` likewise. A message addressed to the camera's component
fails `process_locally` - ArduPilot's own compid is 1 - and falls through to `forward()`, which
sends it out on the link whose learned route matches. The preconditions are the VIO
heartbeating (it does), the links differing (they do), and `MAVn_OPTIONS` bit 1 (`NO_FORWARD`)
being clear.

*Also checked:* all seven PNU-KAL messages in `pnu_kal.xml` are accounted for - three inbound
TC handled in `GCS_Common.cpp`, four outbound TM in the id map and `try_send_message()`. None
carries a target field, so they are broadcast and handled by the FC, unaffected by any VIO
routing.

*Verified:* `./waf copter` clean, zero warnings, **1,722,228 B**. Also builds with
`HAL_MOUNT_VIEWPRO_ENABLED=0` (1,717,352 B) and `HAL_MOUNT_ENABLED=0` (1,653,516 B).
**Not bench-tested.**

The predicate is wrapped in `#if HAL_MOUNT_VIEWPRO_ENABLED`, since `AP_Mount::Type::Viewpro`
only exists inside that guard; it returns false when Viewpro is compiled out. Verified by
building with `--define HAL_MOUNT_VIEWPRO_ENABLED=0` at 1,717,140 B.

*Verified:* `./waf copter` clean, zero warnings, **1,721,712 B**. The `HAL_MOUNT_ENABLED=0`
invariant still holds at 1,653,168 B. **Not bench-tested.**

**Stage 3 as applied, 2026-10-13 - TM2 reports real zoom on a Gremsy.**

Bench measurement settled the question the SDK reference could not: **the VIO reports
`CAMERA_SETTINGS.zoomLevel` in zoom TIMES, not the percentage the MAVLink spec defines.**
Read 1.00 at wide and 30.00 at full tele in super-resolution mode, matching the camera's own
video overlay (`{1.00 ~ 30.00} x, SR`) and matching TM2's ICD range of `MIN:1, MAX:30`
exactly. That removed both obstacles at once - no percentage conversion, and no `int8_t`
overflow, since combine mode (which reaches 240x) is not used.

*Found while building the test harness:* **`CAMERA_SETTINGS` was never reaching `AP_Camera`
at all.** `GCS_Common.cpp` routes only three message ids to `AP_Camera::handle_message()` -
`DIGICAM_CONTROL`, `GOPRO_HEARTBEAT` and `CAMERA_INFORMATION`. Any decode added to a backend
was dead code until that list grew. This also means ArduPilot's `request_camera_information()`
may never have worked on this payload: `CAMERA_INFORMATION` arrives because the VIO
**broadcasts** it, which is the same reason the statustext repeated (fixed in 5.1.4).

**Implemented as three layers, so no shared library learns a Gremsy-specific fact:**

| Layer | Does | Vendor-specific? |
|---|---|---|
| `GCS_Common.cpp` | routes `CAMERA_SETTINGS` - **one `case` label**, the only core-file cost | no |
| `AP_Camera_Backend.h` | `virtual bool get_reported_zoom_level(float&)`, default false | no |
| `AP_Camera.{h,cpp}` | frontend forwarder | no |
| `AP_Camera_MAVLinkCamV2` | decodes msg 260, polls it, serves the **raw** value | **no** - never claims to know the units |
| `AP_Q30::get_tm2_zoom_times()` | mount first (Viewpro), then camera - **interprets as times** | yes, and this is where the strategy says it belongs |
| `GCS_PNU.cpp` | TM2 asks `AP_Q30` instead of the mount directly | - |

`AP_Camera` reports the number and says nothing about what it means; only `AP_Q30` - PNU
code by definition - decides it is times. That satisfies design rule 1 of the dual-GCS
compatibility strategy, and it is why this did **not** need the vendor/model gate that
`AP_Mount_MAVLink.h:51` uses for the AVTA/CM41 case.

*Design points worth not re-deriving:*

- **Polled, not streamed.** The VIO answers a request rather than broadcasting this message,
  so the backend requests it at 1 Hz with a 3 s staleness timeout. If the camera stops
  answering, `get_reported_zoom_level()` returns false and TM2 falls back to 0 instead of
  holding a frozen reading - the same honesty rule as Stage 1.
- **Only `MAV_CMD_REQUEST_MESSAGE` is sent.** The VIO was observed answering both that and
  the superseded `MAV_CMD_REQUEST_CAMERA_SETTINGS` (522) - the debug counter advanced by two
  per cycle - so the legacy form was dropped as redundant.
- **`NaN` is rejected explicitly.** `zoomLevel` is a MAVLink2 extension field and is
  legitimately absent on a camera that does not know its zoom.
- **Accepted risk, recorded at the code site:** a spec-compliant camera reporting a true
  percentage would be misreported as times. TM2 is a PNU message on PNU aircraft with known
  payloads. Revisit if a third camera type is ever fitted.

**Bonus fix, vendor-neutral:** `AP_Camera_MAVLinkCamV2::send_camera_settings()` now forwards
what the camera actually reported. Upstream sends `NaN` placeholders, so a standard GCS saw
"zoom unknown" even while the camera was reporting it. ArduPilot now proxies it faithfully.

**Worth submitting upstream.** ArduPilot currently cannot read camera zoom at all - the
routing gap, the decode, the accessor and the `send_camera_settings()` placeholder are one
coherent gap, none of it PNU-specific. If upstream takes it, the last divergence goes with it.

**Bench-confirmed 2026-10-13: `zoomLevel` follows the active EO/IR source**, so no sensor
check is needed in the FC and none was added.

This was the one thing that could still have made Stage 3 wrong. `AP_Mount_Viewpro` is
sensor-aware internally - `AP_Mount_Viewpro.cpp:334-346` reads a *different* telemetry field
depending on `_image_sensor` - so TM2 on a Viewpro reports the zoom of whatever the operator
is looking at. Had the VIO reported EO zoom while the operator watched IR, TM2 would have
carried a number that was not what it claimed to be, which is exactly the defect class Stage 1
removed. Measured on the bench: switching the main source switches the reported value, so the
camera does the selection itself and `AP_Q30` can pass the value straight through.

Both ranges fit TM2's `int8_t` field: EO reaches 30x in super-resolution (matching the ICD's
own `MIN:1, MAX:30`) and IR reaches 8x. Only combine mode, at 240x, would overflow - and it
is not used.

*A more expensive route was considered and is not needed:* the VIO streams `EO_ZOOM` and
`IR_ZOOM` separately as telemetry indexes 0 and 1 from the payload app on component 26, so the
FC could have selected the right one itself. That would have required discovering component 26
(ArduPilot has **no** reference to `MAV_COMP_ID_USER2` anywhere), sending a
`SET_MESSAGE_INTERVAL` whose param1 Gremsy repurposes as a vendor index where MAVLink defines
it as a message id, decoding `PARAM_VALUE`/`DEBUG` from that component, and routing those to
`AP_Q30` - which has no message handler, so a **second** `GCS_Common.cpp` hook. Recorded
because the idea is sound and will come up again; it is simply unnecessary here.

**The bench aid is kept, switched off.** `AP_CAMERA_MAVLINKCAMV2_ZOOM_DEBUG` in
`AP_Camera_MAVLinkCamV2.h` defaults to **0**; set it to 1 to report zoom, focus and the
reading's **age** to the GCS every 2 s. The age field is the point - it separates a live value
from a stale one, which a raw number cannot. The decode is permanent, so only the reporting is
conditional and the off build is byte-identical to having no debug code at all. Same pattern as
`GCS_VIEWPRO_IR_DEBUG` in `GCS_PNU.cpp`. It answered both Stage 3 questions and is worth having
for the next payload.

*Verified:* `./waf copter` clean, zero warnings, **1,722,152 B** with the debug off,
1,722,328 B with it on. Also builds with `HAL_MOUNT_ENABLED=0` (1,653,516 B),
`AP_CAMERA_ENABLED=0` (1,695,896 B) and `AP_CAMERA_MAVLINKCAMV2_ENABLED=0` (1,720,704 B).
**TM2 itself not yet bench-tested** - on hardware `Zoom_POS_FB` should track 1-30 on EO and
1-8 on IR, and the one-shot `CAM: zoom not reported` warning should stop appearing, which is
the quickest confirmation. Viewpro is unchanged, as the mount path is tried first and answers.

*Known, pre-existing, not changed:* `GCS_PNU.cpp:102` casts `(int8_t)zoom_times`, which
**truncates** - a camera reporting 29.97 at full tele sends 29. The Viewpro path has always
done this. Rounding instead would change what goes on the wire, so it is an ICD decision, not
a code one.

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

- **`get_zoom_times`** - **RESOLVED 2026-10-13 (Stage 3).** `AP_Camera_MAVLinkCamV2` now
  decodes `CAMERA_SETTINGS` (msg 260). The gap was bigger than this note assumed: the message
  was not even *routed* to `AP_Camera`, so a backend decode alone would have been dead code.
  It was indeed a genuine upstream gap, and the fix is written to be submittable.
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
> | Which `CAMERA_CAP_FLAGS` does it advertise? | **ANSWERED 2026-10-13** - `HAS_BASIC_ZOOM` proven by zoom working through Mission Planner. See "Gremsy VIO bench results". |
> | Exact `vendor_name` / `model_name` strings per model | **ANSWERED 2026-10-13** - gimbal `GREMSY`/`VIO` fw 7.8.7.0, camera `Gremsy`/`Vio Payload G1` fw 18.4.1.3. `model_name` gates nothing anyway. |
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
| `libraries/AP_Mount/AP_Mount_Backend.{h,cpp}` | `suppress_gimbal_device_attitude_status()` (D8a); the `get_zoom_times()` out-param signature (D10 Stage 1). **Deliberately carries NO pitch sign flip** - re-adding one double-negates KGCS and inverts every standard GCS |
| `libraries/AP_Mount/AP_Mount.{h,cpp}` | the `get_zoom_times()` out-param signature (D10 Stage 1) |
| `libraries/AP_Mount/AP_Mount_Viewpro.{h,cpp}` | `suppress_gimbal_device_attitude_status()` override; D1 markers; `_zoom_times` initialiser and the `get_zoom_times()` validity test (D10 Stage 1) |
| `libraries/AP_SerialManager/AP_SerialManager.h` | IOMCU renumber deliberately **not** applied |
| `libraries/AP_OSD/AP_OSD_ParamSetting.cpp` | `"Q30"` padding deliberately **not** applied |
| `libraries/GCS_MAVLink/GCS_Common.cpp` | upward-proximity block deliberately **not** commented out; the handler bodies live in `GCS_PNU.cpp`, only 3 hooks remain here (D5); plus one `case MAVLINK_MSG_ID_CAMERA_SETTINGS` label routing that message to `AP_Camera` (D10 Stage 3) |
| `libraries/GCS_MAVLink/GCS_PNU.cpp` | PNU-only file; the step-7 defect fixes and `#if` guards live here; the D10 Stage 1 TM2 zoom-truth and no-gimbal gate |
| `libraries/AP_Q30/AP_Q30.{h,cpp}` | `send_cmd_speed()` **and** `send_cmd_angle()` negate pitch at the call site (see the pitch-reversal table); `AP_Q30_ENABLED` guard and the D14 global declarations; the 27 return checks and `report_cam_unsupported()` (D10 Stage 1); camera calls routed through `AP::camera()` and `set_video_source()` (D10 Stage 2a) |
| `ArduCopter/UserCode.cpp` | `#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED` guard around the OA block |
| `ArduCopter/events.cpp` | D9(a) recovery-clear fix and D9(b) arm-time latch gate; `LOGGER_WRITE_ERROR` portability fix |
| `ArduCopter/AP_Arming_Copter.{h,cpp}` | `pmucan_checks()` prearm warning and the arm-time healthy latch (D9b) |
| `libraries/GCS_MAVLink/GCS.h` | `OFP_VER_*`; the D3 dialect `#error` guard; the D12 `prev_prx_failed` latch; the D10 `prev_zoom_unavailable` latch; PNU handler declarations |
| `libraries/AP_Q30/AP_Q30_config.h` | PNU-only file - the `AP_Q30_ENABLED` flag (D13), which also requires `AP_CAMERA_ENABLED` since Stage 2a |
| `libraries/AP_Camera/AP_Camera_MAVLinkCamV2.{h,cpp}` | the one-shot guard on the `"Camera: ..."` statustext; the `CAMERA_SETTINGS` decode, 1 Hz poll, `get_reported_zoom_level()` and the `send_camera_settings()` override (D10 Stage 3). All of it upstream-submittable rather than PNU-specific |
| `libraries/AP_Camera/AP_Camera.{h,cpp}`, `AP_Camera_Backend.h` | the `get_reported_zoom_level()` accessor and its backend virtual (D10 Stage 3) - deliberately unit-neutral |
| `libraries/AP_PNU_CAN/*` | PNU-only files - the shared CAN driver for PNU devices (5.1.11). Owns the interface, the thread, the `CAN_Dn_PNU_*` parameters and the one `send_frame()`; protocols live in backends |
| `libraries/AP_PNUGimbal/*` | PNU-only files - `AP_PNUGimbal_Backend` (gimbal layer: lifecycle state machine, mechanical limits, slew limit, arming interlock), `AP_PNUGimbal_Homing` (the Appendix 1 search), `AP_PNUGimbal_CubeMars` (wire format only) + codec tests |
| `libraries/AP_Mount/AP_Mount_PNUGimbal.{h,cpp}` | PNU-only files - the `MNT1_TYPE=15` mount backend (5.1.12). Targeting adapter and stabilisation maths only; no protocol, no CAN |
| `libraries/AP_Mount/AP_Mount.{h,cpp}`, `AP_Mount_config.h`, `AP_Mount_Params.cpp` | `PNUGimbal = 15` in the `Type` enum, its factory case, `HAL_MOUNT_PNUGIMBAL_ENABLED`, and `15:PNUGimbal` in the `MNT1_TYPE` `@Values` |
| `libraries/AP_PNUSAR/*` | PNU-only files - CAN driver skeleton (5.1.9), structure only until the ICD arrives |
| `libraries/AP_CANManager/AP_CANManager_CANDriver_Params.cpp` | the `CAN_Dn_PNU_` subgroup at index 8 (index 7 retired, not reused), and 15/16 added to the `PROTOCOL` `@Values` list |
| `libraries/AP_CANManager/AP_CAN.h`, `AP_CANManager.{h,cpp}` | `PNU_CAN = 15`, `PNU_CAN_LEGACY = 16` (deprecated alias), `PNUSAR = 17` plus their factory cases |
| `libraries/AP_Arming/AP_Arming.cpp` | the three PNU protocols listed in the `AP_CAN::Protocol` switch - required to compile under `-Werror=switch`, zero runtime effect |
| `Tools/scripts/build_options.py` | the `Camera / Q30` build option (D13) and `Gimbal / PNUGIMBAL` (5.1.12) |
| `ArduCopter/version.h` | **5.1.9 DEV** and the release note describing what it carries |
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
| `version.h` is **5.1.19 DEV** | 5.1.0 OFFICIAL closed the migration; the 5.1.x DEV series carries the post-release D10 work - Stage 1 in 5.1.1, the Gremsy analysis and pitch relocation in 5.1.2, Stage 2a routing in 5.1.3, Gremsy hardware validation in 5.1.4, Stage 2b closure in 5.1.5, Stage 3 in 5.1.6, its bench confirmation in 5.1.7, the KGCS backward-compatibility fix in 5.1.8, the `AP_PNUGimbal` / `AP_PNUSAR` CAN driver skeletons in 5.1.9, `AP_PNUGimbal`'s CubeMars protocol backend in 5.1.10, CAN Update Plan step 1 in 5.1.11, step 1b in 5.1.12, step 1c in 5.1.13, step 1d in 5.1.14, step 2 in 5.1.15, the bench fixes in 5.1.16, the PMU failsafe's scheduling fix in 5.1.17 CAN Update Plan step 2c in 5.1.18 and SAR Integration Plan step 3a in 5.1.19 - and goes OFFICIAL when D10/D11 land |
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
declined camera command is now reported. Stage 1 deliberately did not touch routing: it makes
the symptom visible so that a Gremsy bench session produces a diagnosis instead of silence.

**The Gremsy protocol answers arrived 2026-10-08** (see "Standard Control for Gremsy VIO
EO/IR payload" at the end of this file) and **cut the remaining work down rather than
unblocking it wholesale.** Three of the six ACTION ON PNU questions are answered: IR palette
is `C_T_PALETTE`, EO/IR source is `C_SOURCE`, both over `PARAM_EXT_SET`, and the camera is a
separate component ID. Because **ArduPilot implements no `PARAM_EXT_*` at all**, those two
functions can only be reached by direct KGCS-to-payload routing - they leave the FC entirely,
and no amount of `AP_Q30` routing would have reached them.

What is left on D10/D11, in order:

| | Work | Blocked on |
|:--:|---|---|
| Stage 2a | **DONE 2026-10-12** - the 6 camera functions route through `AP::camera()`; `CAM1_TYPE` selects the destination. Behaviour-neutral on Viewpro | - |
| Stage 2b | **CLOSED 2026-10-13 by not implementing it in the FC.** `AP_Q30` now sends an absolute zoom only to a Viewpro; on any other mount it stays silent and KGCS drives zoom directly with `USER_4` "zoom to level". See "Gremsy PayloadSDK message reference" | - |
| Stage 3 | **DONE 2026-10-13.** Bench measurement showed the VIO reports `zoomLevel` in **times**, not the spec's percentage, so TM2 now carries real zoom on a Gremsy. Implemented in three layers so no shared library learns a vendor fact; cost is one `case` label in `GCS_Common.cpp` | - |
| KGCS side | EO/IR source (`C_SOURCE`), IR palette (`C_T_PALETTE`), object tracking (`USER_4` to the payload app) and **absolute zoom (`USER_4` zoom-to-level, decided 2026-10-13)** move to KGCS-to-VIO direct routing. No FC code. **All the mappings are now known** - see "Gremsy PayloadSDK message reference". Requires `MAVn_OPTIONS` bit 1 (`NO_FORWARD`) clear on the link | PNU/KGCS work, not this repo |

Residuals recorded against resolved issues: D1 (is the prelude needed at all?), D6 (the PMU
counters are still write-only), D7 (accepted interlock residuals), D12 (confirm the 10 m /
15 m thresholds). New from Stage 1: `_current_zoom_EO` / `_current_zoom_IR` are write-only,
the same keep-or-delete question as D6's counters.

**Open issues** are the `PNU-ISSUE` table above; code markers carry the same ids.
List them with:

```
grep -rn "PNU-ISSUE" libraries/ ArduCopter/
```
---

**Standard Control for Gremsy VIO EO/IR payload**

*Here, basic control methods and protocol provided by Gremsy are described :*

1) Basic gimbal movement: controlled by Gimbal Protocol V2
- Related Parameter Setting: MNT1_TYPE=6, SERIAL2_PROTOCOL=2, SERIAL2_BAUD=115. Use MNT1_DEFLT_MODE=2 for MAVLink targeting. RC control additionally requires appropriate RCx_OPTION assignments (No plan to use RC).
- ArduPilot Library: libraries/AP_Mount/, primarily AP_Mount.cpp and AP_Mount_MAVLink.cpp.
- Controllable behaviors: Target attitude, angular rate, yaw lock/follow, and retract.
- Accessible data from gimbal: Device information, attitude quaternion, angular velocity, operating flags, and failure flags through Gimbal V2 messages.  
Available fields depend on the payload firmware.  
The MAVLink backend generates device-level commands through functions such as send_target_angles(), send_target_rates(), and send_target_retracted().

2) Camera-related control: controlled by MAVLink commands and extended parameters
- Related Parameter Setting: CAM1_TYPE=6 for ArduPilot-managed camera control; MAVLink2 and matching UART baud rate. Direct GCS-to-payload routing does not depend on CAM1_TYPE.
- ArduPilot Library: libraries/AP_Camera/, primarily AP_Camera.cpp and AP_Camera_MAVLinkCamV2.cpp for standard camera operations; libraries/GCS_MAVLink/ for directly routed payload messages.
- Controllable behaviors: Capture, recording, zoom, and focus using standard commands. EO/IR source, palette, exposure, and other Gremsy settings use PARAM_EXT_SET.
- Accessible data from payload: Camera information, settings, storage information, capture status, stream information, and extended parameter values.  
ArduPilot's camera backend includes set_zoom() and set_focus(). Gremsy-specific extended settings should be handled by the GCS and routed to VIO unless additional FC-side integration is implemented.

> **Which of that the FC can actually see, verified against 4.7.1.** Of the five information
> messages, ArduPilot has an inbound decode for exactly **one**: `CAMERA_INFORMATION`
> (`AP_Camera_MAVLinkCamV2::handle_message()`). `CAMERA_SETTINGS`, `STORAGE_INFORMATION`,
> `CAMERA_CAPTURE_STATUS` and `VIDEO_STREAM_INFORMATION` have **zero** inbound decode sites -
> every occurrence in the tree is ArduPilot *sending* them as a camera to a GCS. So those four
> reach a GCS by direct routing only, never the FC. This is what blocks TM2 from reporting real
> zoom (**D11**): `CAMERA_SETTINGS.zoomLevel` carries it, but nothing decodes the message. Note
> too that `zoomLevel` is a percentage of full range, not zoom times, and is a MAVLink2
> extension field that may legitimately be NaN.
>
> **`PARAM_EXT_*` is not implemented in ArduPilot at all** - no handler for `PARAM_EXT_SET`,
> `_VALUE`, `_REQUEST_LIST`, `_REQUEST_READ` or `_ACK` anywhere in the tree. So `C_SOURCE`,
> `C_T_PALETTE` and every other Gremsy extended parameter can **only** be reached by direct
> GCS-to-payload routing. Routing `AP_Q30`'s calls to `AP::camera()` would not reach them,
> which settles the **D10** Stage 2 question for those two functions: they do not belong in
> the FC.
>
> **`set_zoom()` / `set_focus()` are gated on `CAMERA_CAP_FLAGS`** taken from
> `CAMERA_INFORMATION` - `HAS_BASIC_ZOOM` and `HAS_BASIC_FOCUS` respectively. If VIO does not
> advertise them, `AP_Camera_MAVLinkCamV2` returns false/FAILED **without transmitting
> anything**. Confirming which flags VIO advertises is therefore the top remaining unknown.
> Note also that `MAV_CMD_SET_CAMERA_MODE` (530) appears **nowhere** in ArduPilot - it is
> neither sent nor handled, so there is no FC path for it.

3) Data flow when using a standard open source GCS  

| Path | GCS → ArduPilot | ArduPilot → Gremsy payload |
|:--:|:--:|:--:|
| FC-managed gimbal | GIMBAL_MANAGER_SET_ATTITUDE, GIMBAL_MANAGER_SET_PITCHYAW, or COMMAND_LONG containing MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW | AP_Mount processes the target; AP_Mount_MAVLink generates GIMBAL_DEVICE_SET_ATTITUDE |
| FC-managed camera | Standard camera commands addressed to the FC | AP_Camera dispatches supported operations to AP_Camera_MAVLinkCamV2, which generates camera commands |
| Direct payload control | Commands or PARAM_EXT_* messages addressed to VIO's actual System/Component IDs | MAVLink_routing::check_and_forward() routes the message without converting its command semantics |


- Direct routing requires a learned route to VIO and enabled forwarding. Do not set the "no forward MAVLink" option on links used for this path. The payload executes the command and returns responses through the routing path.
- **The parameter for this moved in 4.7.** It is now `MAVn_OPTIONS` bit 1 (`NO_FORWARD`). The old `SERIALx_OPTIONS` bit 10 is deprecated and raises a prearm failure - `AP_SerialManager.cpp:860`: "SERIAL%u_OPTIONS bit 10 ('no forward mavlink') must not be set; use MAVn_OPTIONS bit 1 instead".

4) MAVLink message lists

- *Gimbal Protocol V2 — main integration messages*

| Direction | Messages |
|:--:|:--:|
| GCS → FC | GIMBAL_MANAGER_SET_ATTITUDE, GIMBAL_MANAGER_SET_PITCHYAW |
| FC → GCS | GIMBAL_MANAGER_INFORMATION, GIMBAL_MANAGER_STATUS |
| FC → Gimbal | GIMBAL_DEVICE_SET_ATTITUDE, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE |
| Gimbal → FC/GCS | GIMBAL_DEVICE_INFORMATION, GIMBAL_DEVICE_ATTITUDE_STATUS |

- Command-based manager control also uses MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW and MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE. This is an integration list, not a claim that VIO implements every Gimbal V2 feature.  

- *Camera-related messages*
   - Command transport: COMMAND_LONG, COMMAND_ACK.
   - Control commands: MAV_CMD_SET_CAMERA_MODE, MAV_CMD_SET_CAMERA_ZOOM, MAV_CMD_SET_CAMERA_FOCUS, MAV_CMD_IMAGE_START_CAPTURE, MAV_CMD_IMAGE_STOP_CAPTURE, MAV_CMD_VIDEO_START_CAPTURE, MAV_CMD_VIDEO_STOP_CAPTURE.
   - Information messages: CAMERA_INFORMATION, CAMERA_SETTINGS, STORAGE_INFORMATION, CAMERA_CAPTURE_STATUS, VIDEO_STREAM_INFORMATION.
   - Extended parameters: PARAM_EXT_REQUEST_LIST, PARAM_EXT_REQUEST_READ, PARAM_EXT_VALUE, PARAM_EXT_SET, PARAM_EXT_ACK.
   - Gremsy parameter examples: C_SOURCE, C_V_REC, C_T_PALETTE, C_V_AE, GB_MODE.
- Gremsy's examples commonly target camera component 101, but the actual ID is configurable. These payload parameters belong to VIO, not to ArduPilot's parameter database.

> **A fuller message reference extracted from Gremsy PayloadSDK 3.2.0 source lives in
> `VIO_MAVLink_Messages.md` in this folder**, with the conclusions drawn from it recorded
> below under "Gremsy PayloadSDK message reference". It supersedes the summary above where
> the two differ, and notes that the VIO addresses **three** components - camera, gimbal and
> a payload app on comp 26 that carries tracking, absolute zoom and several other functions.

---

**Gremsy integration mechanics - verified against 4.7.1, 2026-10-12**

*Answers to three questions raised while reviewing the Gremsy notes. All of this is read out
of the tree, not from vendor documentation.*

**1. `CAMERA_INFORMATION` / `CAMERA_CAP_FLAGS` are not a new concept here - the direction is.**

The current Viewpro build already uses both, **outbound**: `AP_Mount_Viewpro::send_camera_information()`
builds the message and declares six flags (`CAPTURE_VIDEO`, `CAPTURE_IMAGE`, `HAS_BASIC_ZOOM`,
`HAS_BASIC_FOCUS`, `HAS_TRACKING_POINT`, `HAS_TRACKING_RECTANGLE`) - ArduPilot acting *as* the
camera toward a GCS.

For Gremsy the role reverses: `AP_Camera_MAVLinkCamV2` **requests and consumes**
`CAMERA_INFORMATION` from the payload. The consequence is the part with no Viewpro precedent:

| | Viewpro today | Gremsy via `AP_Camera_MAVLinkCamV2` |
|---|---|---|
| Who produces `CAMERA_INFORMATION` | ArduPilot | the VIO |
| Do the cap flags gate anything? | **No** - `AP_Mount_Viewpro::set_zoom()` checks only `_initialised`; the flags it advertises are never read by its own code | **Yes** - `set_zoom`/`set_focus`/`trigger_pic`/`record_video` each return false **before transmitting** if their flag is clear |

So with Viewpro the flags are pure advertising, which is why they have never mattered. With
Gremsy they are a hard gate, and a VIO that does not advertise `HAS_BASIC_ZOOM` gets no zoom
command sent at all - the D10 Stage 1 return checks will report it as
`CAM: mount rejected zoom in` once routing exists.

**2. How ArduPilot finds the camera - the component ID trap.**

`AP_Camera_MAVLinkCamV2::find_camera()` locates the payload with
`find_by_mavtype_and_compid(MAV_TYPE_CAMERA, compid, _sysid)` where
`compid = MIN(MAV_COMP_ID_CAMERA + _instance, MAV_COMP_ID_CAMERA6)`:

| `AP_Camera` instance | Parameter | compid it looks for |
|:--:|:--:|:--:|
| 0 | `CAM1_TYPE=6` | **100** (`MAV_COMP_ID_CAMERA`) |
| 1 | `CAM2_TYPE=6` | **101** (`MAV_COMP_ID_CAMERA2`) |

**The Gremsy notes above say the camera is commonly component 101.** If that is so on the
actual unit, `CAM1_TYPE=6` searches for 100 and **never finds it**; the payload must either be
configured to 100 or be driven as `CAM2_TYPE=6`. Confirm the real compid before concluding the
camera is unsupported.

Two further conditions, both easy to miss:

- The camera must emit a **HEARTBEAT with `MAV_TYPE_CAMERA`**. That is the only thing that
  populates `mavtype` in the routing table (`MAVLink_routing.cpp:375-386`), and the search
  matches on it. A payload that answers commands but sends no heartbeat is never found.
- The search **starts 10 s after boot, runs for 60 s, and stops once armed**. This has to work
  on the bench, disarmed.

**3. What `model_name` does and does not affect.**

`model_name` is used in exactly two places, neither of them a gate:

- the `"Camera: %.32s %.32s fw:..."` statustext in `AP_Camera_MAVLinkCamV2::handle_message()` - cosmetic;
- `AP_Mount_MAVLink::natively_supported_mount_target_types()`, for the AVTA/CM41 special case.
  A garbage or empty string simply fails the `strncmp`, `LOCATION` is not claimed as natively
  supported, and the base class converts ROI to angles - **which is the normal path anyway**.

`_got_camera_info` is set on decode regardless of field contents. **So a malformed `model_name`
is harmless; a missing `CAMERA_INFORMATION` is not**, because then `_cam_info.flags` stays 0 and
every camera call fails. The two failure modes look similar at the GCS and must not be confused.

**4. Zoom: what the capability flag cannot tell you.**

There is **one** flag, `CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM`, covering all five `CAMERA_ZOOM_TYPE`
values. MAVLink provides no way to advertise "continuous yes, range no", so capability flags
**cannot** answer whether absolute zoom works. Only a live command can:

```
send MAV_CMD_SET_CAMERA_ZOOM with param1 = 2 (ZOOM_TYPE_RANGE), param2 = 50
  COMMAND_ACK = MAV_RESULT_ACCEPTED     -> absolute zoom works, Stage 2 maps 1..30 -> 0..100 %
  COMMAND_ACK = MAV_RESULT_UNSUPPORTED  -> continuous only, see the options below
```

This needs no firmware change - it can be sent from Mission Planner or pymavlink.

Mapping TC2 onto a buttons-only payload:

| TC2 / TM2 | Needs | If the VIO is continuous-only |
|---|---|---|
| `Zoom_Focus_Stop_CMD` 1 / 2 - zoom in / out | `ZOOM_TYPE_CONTINUOUS` | **works as-is** - this is exactly what a zoom-in/zoom-out button is |
| `Tracking_CMD` 21-24 - absolute EO zoom | `ZOOM_TYPE_RANGE` | must become timed continuous pulses |
| TM2 `Zoom_POS_FB` - reports 1..30 | `CAMERA_SETTINGS.zoomLevel` | no measured value available |

**Integer zoom without an explicit level command is possible three ways**, best first:

1. **`ZOOM_TYPE_RANGE`** - if it ACKs, map `EO_zoom_times` 1..30 onto 0..100 %. One command, clean.
2. **`ZOOM_TYPE_FOCAL_LENGTH` (mm) or `ZOOM_TYPE_HORIZONTAL_FOV` (deg)** - also absolute and also
   one command, but both need the lens specification from Gremsy to convert.
3. **Dead-reckoning** - `AP_Q30` already counts steps into `EO_zoom_times`; send timed continuous
   pulses and trust the counter. Needs nothing from the VIO, but it drifts with no feedback.
   **Note the ICD consequence:** reporting that counter in TM2 would put a *commanded* value on
   the wire as though it were measured, partly undoing what D10 Stage 1 fixed. It is an honest
   estimate rather than a fabricated 0, but it is an ICD decision, not a code one.

**Worth checking before any of that:** whether the VIO exposes zoom level as a `PARAM_EXT`
parameter alongside `C_SOURCE` and `C_T_PALETTE`. If it does, KGCS can read it directly over the
routing path it already uses for palette and source, TM2's gap stops mattering, and **Stage 3
closes with no ArduPilot change at all**.

**Bench checklist, in order** - all disarmed, all before writing any routing code:

1. Power up with the VIO connected. Record the `"Mount: %s %s fw:..."` and `"Camera: %s %s fw:..."`
   statustexts. Seeing **both** confirms gimbal and camera are separate components.
2. Record the camera's actual **component ID** and whether it heartbeats as `MAV_TYPE_CAMERA`.
   This decides `CAM1_TYPE` vs `CAM2_TYPE`, and whether `AP_Camera` can reach it at all.
3. Record the **`CAMERA_CAP_FLAGS`** bitmask from `CAMERA_INFORMATION`. This decides which of
   zoom / focus / video / image will transmit.
4. Send `MAV_CMD_SET_CAMERA_ZOOM` with `ZOOM_TYPE_RANGE` and read the `COMMAND_ACK`.
5. Check whether `CAMERA_SETTINGS` (msg 260) is emitted after a zoom command, and whether its
   `zoomLevel` extension field is populated rather than NaN.
6. List the VIO's `PARAM_EXT` parameters and look for a zoom-level entry.

Steps 1-3 answer the two remaining ACTION ON PNU questions. Steps 4-6 decide Stage 3.

---

**Gremsy VIO bench results - 2026-10-13. First real hardware.**

*A Gremsy VIO was connected and driven from Mission Planner. This closes both remaining
ACTION ON PNU questions and validates D10 Stage 2a on hardware.*

**Identity - both ACTION ON PNU identity questions answered:**

| Device | vendor / model | Firmware |
|---|---|---|
| Gimbal (`MNT1_TYPE=6`) | `GREMSY` / `VIO` | 7.8.7.0 |
| Camera (`CAM1_TYPE=6`) | `Gremsy` / `Vio Payload G1` | 18.4.1.3 |

The two firmware versions differ, confirming the gimbal and camera are genuinely separate
devices rather than one unit answering on two component IDs.

**Component ID: the VIO camera ships on 101, ArduPilot instance 0 looks for 100.**
`find_camera()` uses `compid = MAV_COMP_ID_CAMERA + _instance`, so `CAM1_TYPE=6` searches for
**100** while the VIO defaults to **101** - and the camera is simply never found, with no
error beyond the absence of the `"Camera: ..."` statustext.

**Resolved by setting the VIO's camera component to 100**, which its configuration allows.
That needs **no firmware change** and is the recommended setup. The alternative - driving it
as `CAM2_TYPE=6` (instance 1) - **would not work without code changes**, because all 17 of
`AP_Q30`'s camera calls hardcode instance 0. If a future payload has a fixed component ID,
that is the change required: an instance selector in `AP_Q30`.

**Confirmed working with `CAM1_TYPE=6` and the camera on component 100:**

| | Result |
|---|---|
| `CAMERA_INFORMATION` received, camera found | yes |
| Zoom via Mission Planner's camera interface | **works** |
| `CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM` (bit 64) | **effectively proven** - `AP_Camera_MAVLinkCamV2::set_zoom()` returns false *before transmitting* when that flag is clear, so zoom reaching the camera means the flag is set |
| Video streaming | separate from ArduPilot; works, and is not ArduPilot's concern |

**D10 Stage 2a is validated on hardware with zero code changes.** The routing applied in
5.1.3 reaches a real Gremsy camera through `AP::camera()`.

**Gimbal control from Mission Planner's slew interface - all paths verified.**

| Control | Result | Mechanism |
|---|---|---|
| Rate slew, roll / pitch / yaw | works; 1 deg/s moves very little, 10 deg/s as expected | `send_target_rates()` is a clean passthrough - rad/s straight into `GIMBAL_DEVICE_SET_ATTITUDE` angular velocity with `attitude = {NaN,NaN,NaN,NaN}`. No scaling, deadzone or quantisation anywhere in ArduPilot, so 1 deg/s really is 1 deg/s - about 0.5 deg per half-second press. **Not a defect** |
| Set Neutral | moves to `MNT1_NEUTRAL_{X,Y,Z}` | **Converted by the base class, not native.** `AP_Mount_MAVLink::natively_supported_mount_target_types()` returns only `ANGLE \| RATE \| RETRACTED`, so `MountTargetType::NEUTRAL` falls to the conversion path, which reads `_params.neutral_angles` and calls `send_target_angles()`. The gimbal never sees a "neutral" command - only an angle |
| Point down | pitch -90 deg | Confirms **physical convention** on the standard MAVLink path, and that `MNT1_PITCH_MIN = -90` permits full nadir on this gimbal - the constraint is `[-90, 15]` and -90 passes unclipped |

*Two notes worth not rediscovering:*

- **`NEUTRAL` being converted rather than native** is the same mechanism that handles ROI on
  this gimbal. Someone changing `MNT1_NEUTRAL_*` and seeing it take effect will reasonably
  assume the Gremsy implements neutral natively. It does not.
- **Point-down does not test the 5.1.2 pitch relocation.** On a Gremsy
  `pitch_target_is_reversed()` returned false from the base class even *before* the
  relocation, so Mission Planner was always correct here. **The relocation's real test is
  Viewpro + Mission Planner**, which is where pitch was inverted and should now be right.

**The dual-GCS compatibility strategy is now validated on hardware.** Slew, neutral,
point-down and zoom all work from Mission Planner against a Gremsy while the KGCS-specific
convention stays confined to `AP_Q30`. Until this session that was a reasoning claim only.

**Relevant to KGCS:** 4.7.x added a 3 s rate timeout (`mnt_target.last_rate_request_ms`,
`AP_Mount_Backend.cpp:1192`) that zeroes all three rate axes. Mission Planner streams while a
button is held so it never trips; KGCS must likewise keep sending TC2 while a rate is
commanded, which it does. Confirmed to behave the same on a Gremsy as on a Viewpro.

**Both Stage 2b and Stage 3 questions were answered 2026-10-13 from the Gremsy PayloadSDK -
see the next section.** Mission Planner's buttons only exercised `ZOOM_TYPE_CONTINUOUS`; the
SDK reference settles the rest.

**Gremsy firmware defect - the yaw fields of `GIMBAL_DEVICE_INFORMATION` are in degrees.**
`MNT1_YAW_MAX` took a default of **18334**, which is `degrees(320)`. Msg 283 specifies
`yaw_min`/`yaw_max` as `units="rad"`, so ArduPilot's `degrees()` conversion is correct and the
VIO is sending 320 - a sensible *degrees* yaw range - in a radians field.

**The bug is confined to the yaw fields.** Roll and pitch defaults arrived as sensible
degrees, which means the VIO sends those correctly in radians. Worth reporting to Gremsy
against gimbal firmware 7.8.7.0. Note this is not caused by leaving yaw unset:
`set_default()` computes `degrees(info.X)` for all six fields on every message, regardless of
what the operator has configured.

*Consequences, and why it is not merely cosmetic:*

| Use of `yaw_angle_max` | Effect at 18334 |
|---|---|
| RC targeting (`AP_Mount_Backend.cpp:138`) | **broken** - the stick maps linearly over `[min,max]`, so a *centred* stick commands `0.5 x (18334 + 180) - 180 = 9077 deg`. PNU does not use RC gimbal control |
| `GIMBAL_MANAGER_INFORMATION` (`:476`) | **wrong data to a standard GCS** - advertises `radians(18334)` = 320 rad as the yaw limit |
| `set_angle_target()` (`:192`) | harmless - yaw is wrapped to <= 180 *before* the constraint, so the upper bound never binds |

**Workaround: set the parameters explicitly.** `set_default()` only applies
`if (!configured())` (`AP_ParamT.cpp:39`), so any saved value is permanent. Verified on the
bench - `MNT1_YAW_MIN/MAX` = -180/180 works correctly, as do `MNT1_ROLL_MIN/MAX` = -15/15 and
`MNT1_PITCH_MIN/MAX` = -90/15 for this gimbal.

> **Residual trap.** `set_default()` calls `add_default()` **before** the `configured()`
> check, and `AP_PARAM_DEFAULTS_ENABLED` is on for this build. So 18334 remains registered as
> the parameter's *default* even while the live value is 180. Mission Planner will show
> `MNT1_YAW_MAX` as modified-from-default, parameter diff tooling will flag the correct value
> as the anomaly, and **a "reset to defaults" would restore 18334.** Harmless until someone
> resets or audits parameters.

**Note the VIO's pitch range is not the Viewpro's.** `MNT1_PITCH_MIN/MAX` = **-90/15** was
verified on this gimbal, against -90/45 for the Viewpro. Both are in physical convention
since the 5.1.2 pitch relocation: MIN is degrees of down travel, MAX degrees of up travel.

**Upstream defect found and fixed here: the `"Camera: ..."` statustext repeated endlessly.**
The VIO **broadcasts `CAMERA_INFORMATION` unsolicited**, and
`AP_Camera_MAVLinkCamV2::handle_message()` printed the statustext on every copy with no
guard - `_got_camera_info` was set *after* the print and never read. ArduPilot was not
re-requesting; `update()` stops calling `find_camera()` once `_initialised` is set. The result
was one statustext per broadcast on every connected link.

Fixed by reading `_got_camera_info` before printing, so the message appears once. This is a
genuine upstream bug, small and self-contained, and **worth submitting to ArduPilot** rather
than carried as another permanent divergence - see D5 for why divergence is expensive here.

---

**Gremsy PayloadSDK message reference - 2026-10-13. Closes Stage 2b and Stage 3.**

`VIO_MAVLink_Messages.md` in this folder is a condensed reference extracted from Gremsy
PayloadSDK 3.2.0 source (`libs/payloadSdkInterface.cpp`, `libs/payload-define/vio_sdk.h`,
`examples/`). It is the message list the VIO actually implements, and it overrides Gremsy's
own `Mavlink_Command_List_v2.0.0.pdf` where the two differ. **Taken as given - it has not been
verified against a running payload beyond the bench results above.**

**The VIO addresses three components, not two.** This matters because much of what KGCS needs
is not on the camera at all:

| | Component | Default |
|---|---|---|
| **C** camera | 100-105 | **101** |
| **G** gimbal | 154 or 171-175 | 154 |
| **P** payload app | 26 (`USER2`) | fixed |

**Stage 2b - ANSWERED. `ZOOM_TYPE_RANGE` is supported**, along with the other two:
`SET_CAMERA_ZOOM 531`, `p1` type 0 step / 1 continuous / 2 range, `p2` = -1 out / 0 stop /
1 in for step and continuous, 0-100 % for range.

**But absolute zoom through ArduPilot is still the wrong answer**, for a reason the bench
could not have shown. `ZOOM_TYPE_RANGE` is a percentage of the camera's *full* range, and that
range is **mode-dependent**:

| `C_V_ZM_MODE` | Maximum |
|---|---|
| 0 combine | **240x** (`C_V_ZM_CB_LV` idx 0-21) |
| 2 super-resolution | **30x** (`C_V_ZM_SR_LV` idx 0-11) |

So converting `EO_zoom_times` to a percentage needs a divisor of 240 or 30 depending on a
`PARAM_EXT` value **ArduPilot cannot read**, since it implements no `PARAM_EXT_*` at all. At
100 % in combine mode the camera would go to 240x, not the 30x the operator asked for.
(Incidentally `_Max_zoom_EO = 30` matches the SR-mode maximum exactly, which suggests that
constant never described this camera.)

**There is a clean absolute-zoom path, and it is not on the camera:**

```
Zoom to level -> P (comp 26) : MAV_CMD_USER_4 31013, p1=2, p2=0, p3=4, p4 = zoom x (e.g. 13.5)
```

Absolute zoom **in times**, no percentage, no divisor, no mode dependency - exactly what TC2
`Tracking_CMD` 21-24 wants. Being `USER_4` to the payload app, ArduPilot has no route to it.

> **DECIDED 2026-10-13: absolute zoom moves to GCS-direct**, alongside palette, source and
> tracking. ArduPilot handles continuous zoom only, which already works and needs no units
> conversion. The alternative - a mode-dependent percentage conversion in `AP_Q30` against a
> parameter the FC cannot read - would have been wrong whenever the operator changed zoom
> mode. **Stage 2b is closed by not implementing it.** See the implementation note below.

**Stage 3 - ANSWERED. `CAMERA_SETTINGS` (260) is emitted** with `mode_id`, `zoomLevel` and
`focusLevel`, and can be requested with `REQUEST_CAMERA_SETTINGS 522`. So decoding msg 260
would work.

**But again there is a better source outside ArduPilot:** telemetry index **0 = `EO_ZOOM`**
(and 1 = `IR_ZOOM`), streamed as a float via `SET_MESSAGE_INTERVAL` and delivered in
`PARAM_VALUE` (22) or `DEBUG` (254). Almost certainly zoom *times* rather than the percentage
`CAMERA_SETTINGS.zoomLevel` carries - which is what TM2's `Zoom_POS_FB` actually needs. If
KGCS reads that directly, **TM2's zoom gap stops mattering and Stage 3 closes with no
ArduPilot change at all.**

**Three mappings the GCS-direct work needs, now known:**

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

**Tracking is on P via `USER_4`** (`p1=4, p2=0, p3=0, p4` 0 stop / 1 active / 2 EagleEyes;
target rectangle with `p3=1`), **not** `MAV_CMD_CAMERA_TRACK_POINT` on the camera. That
settles the tracking question definitively: no ArduPilot path exists, and none could without
vendor-specific code. It also requires `C_SOURCE=1` and `TRACK_MODE=0` first.

**`CAMERA_CAP_FLAGS` - expected `0x1C7`, and every ArduPilot gate passes.** The reference
gives the expected value as `0x1C7` = 455, which decodes to:

| Bit | Flag | Gates, in `AP_Camera_MAVLinkCamV2` |
|:--:|---|---|
| 1 | `CAPTURE_VIDEO` | `record_video()` - **passes** |
| 2 | `CAPTURE_IMAGE` | `trigger_pic()` - **passes** |
| 4 | `HAS_MODES` | `MAV_CMD_SET_CAMERA_MODE`, which ArduPilot never sends - unreachable from the FC either way |
| 64 | `HAS_BASIC_ZOOM` | `set_zoom()` - **passes**, and independently proven on the bench |
| 128 | `HAS_BASIC_FOCUS` | `set_focus()` - **passes** |
| 256 | `HAS_VIDEO_STREAM` | not used by ArduPilot; the SDK checks it before requesting the RTSP URI |

So **all four camera functions ArduPilot can reach are enabled** and nothing is gated off.
The tracking bits (512 / 1024) are expected to be clear, which is consistent with VIO tracking
living on `USER_4` to the payload app rather than the standard tracking commands - and moot
regardless, since `AP_Camera_MAVLinkCamV2` implements no `set_tracking()`.

**This value is unverified** - the reference says so explicitly, and it should be read off the
device. It is one line in Mission Planner's MAVLink Inspector: the `flags` field of
`CAMERA_INFORMATION` from the camera component.

**Still unknown, not blocking:** whether `CAMERA_SETTINGS.zoomLevel` holds a percentage (per
the MAVLink spec) or times (a vendor deviation). Moot if KGCS reads `EO_ZOOM` telemetry
instead.

**Also worth knowing from the reference:** focus supports only continuous (`p1=1`) and auto
(`p1=4`) - no range - which matches what ArduPilot sends for `FocusType::RATE`. The gimbal
mode flags carry a Gremsy extension `0x4000` for mapping mode. `DO_MOUNT_CONTROL 205` is
listed as legacy v1. A laser rangefinder reports through `DISTANCE_SENSOR` (132), for which
TC2/TM2 have no field.

---

**AP_PNUGimbal and AP_PNUSAR - design intent, recorded 2026-10-15**

*Written when both were skeletons at 5.1.9. `AP_PNUGimbal`'s protocol layer landed in
5.1.10 - see the structure note after this section for what changed and what did not.
`AP_PNUSAR` is still a skeleton and everything below still applies to it unchanged. The
same notes are at the top of each header, which is where you will actually look.*

| | `AP_PNUGimbal` | `AP_PNUSAR` |
|---|---|---|
| What it is | a 3-axis gimbal **mount controller** | the **SAR radar payload** |
| Hardware | three **CubeMars GL60 II KV28** motors, one per axis, over CAN | one device over CAN |
| Camera | **none** | n/a |
| What it carries | **the SAR device** - it stabilises the `AP_PNUSAR` payload | - |
| `CAN_Dn_PROTOCOL` | 16 | 17 |

**`AP_PNUGimbal` is NOT the same kind of driver as `AP_PMUCAN`, and the skeleton is only a
starting shape.** `AP_PMUCAN` talks to one complete device that reports status. This talks to
**three actuators** and has to close a stabilisation loop around them, so:

- **The CAN protocol is the motor's, not a PNU device protocol.** The "ICD" is CubeMars' CAN
  protocol - MIT or servo mode - plus whatever per-axis addressing is chosen. There may be no
  PNU document to wait for at all.
- **It is a control loop, not a message decoder.** It needs AHRS attitude, and the 100 Hz
  thread the skeleton creates is very likely too slow for motor control. Check what rate the
  chosen CubeMars mode wants before keeping it.

**The open question for `AP_PNUGimbal` - ANSWERED 2026-10-16, see "Gimbal stabilisation and
the mount state machine" below.** `AP_Mount` does provide FC-attitude stabilisation and does
not require a camera; `AP_Mount_Servo` is both. The recommendation is an `AP_Mount` backend.

**The open question for `AP_PNUSAR`:** where does SAR data go? Candidates are `AP_Proximity`
(if it yields range/bearing usable for avoidance), a PNU-KAL ICD message of its own in
`pnu_kal.xml` alongside TM1/TM2/TM5, or a private path to the ground. That choice decides most
of the driver, so settle it first.

*Note the dependency direction:* `AP_PNUGimbal` stabilises what `AP_PNUSAR` drives, so the two
are coupled at the vehicle level even though neither needs to include the other. Whether the
gimbal needs anything **from** the SAR - a boresight offset, a scan-synchronised rate - is
itself an open question.

---

**AP_PNUGimbal structure - what 5.1.10 settled, and what it deliberately did not**

*The protocol layer is done. The control layer is not started, and the decision that gates
it is still open.*

| Layer | File | State |
|---|---|---|
| frontend - interface, thread, parameters, axis mapping, the one `send_frame()` | `AP_PNUGimbal.{h,cpp}` | done |
| protocol interface - axis-oriented, no CAN in it anywhere | `AP_PNUGimbal_Backend.{h,cpp}` | done |
| CubeMars GLII protocol - 3 modes, 4 universal commands, feedback | `AP_PNUGimbal_CubeMars.{h,cpp}` | done, **not bench-tested** |
| codec tests against the manual's published vectors | `AP_PNUGimbal/tests/` | 10 tests, passing |
| **stabilisation loop** | - | **not started** |
| **`AP_Mount` backend, or a standalone interface** | - | **still the open question** |

**Why a backend split and not one class, as `AP_PMUCAN` is.** `AP_PMUCAN` talks to one
complete device over one fixed protocol, so a second class would buy nothing. This mount is
expected to accommodate more than one motor type, and between them *only the wire format
differs* - the thread, the interface, the parameters, the axis mapping and the way a control
layer asks for an angle are all the same. So the split is along that line and no other: a
backend owns the wire format and **never touches `_can_iface`**, which keeps one place that
validates an outgoing frame and one place that counts a failed send.

**The parameters are `CAN_Dn_PG_*`**, a subgroup on `CANDriver_Params` at index 7 - the same
mechanism PiccoloCAN uses for `PC_`. They are frontend parameters even though the encoding
limits are CubeMars-specific; a second backend would ignore the ones it does not need and
add its own at new indices. `CAN_Dn_PG_TYPE` defaults to **0 (None)** on purpose:
`CAN_Dn_PROTOCOL = 16` alone must not start commanding motors.

**`CAN_Dn_PROTOCOL` now lists 15 and 16** in its `@Values`. 17 (`PNUSAR`) is deliberately
absent until that driver does something - offering a no-op in a dropdown is worse than
offering nothing.

**Five protocol facts worth not rediscovering**, all from
`libraries/PNUGimbal_CAN_Protocol_Type_CubeMars.md`:

1. **The encoding limits are device configuration, not constants.** A host/device mismatch
   does not fail - it silently rescales every command and every reading. That is why they
   are parameters.
2. **MIT fields are unsigned offset integers**, not two's complement and not float32. Packed
   zero sits near the middle of the range, so an all-zero MIT payload is *not* a zero
   command.
3. **The manual's transmit function uses `1 << bits` and its receive function `(1 << bits) - 1`.**
   This driver uses the receive form in both directions, so encode and decode are inverses
   and the maximum saturates instead of overflowing into the next field.
4. **Feedback carries only four bits of motor identity** - D0's upper nibble is the status
   code. Motor IDs are restricted to 1..15 and required distinct, which makes that nibble a
   complete demultiplexer. The prose claims eight bits; its own table does not.
5. **Position feedback must be read through an unsigned intermediate.** The manual's
   `int16_t` example misrepresents every value at or above 0x8000, which is half the travel.

**Seven things the manual does not specify, and which were not invented:** command period,
feedback trigger, feedback rate, loss timeout, watchdog interval, acknowledgment of the
universal commands, and any runtime mode-switch command. They are listed at the top of
`AP_PNUGimbal_CubeMars.h` so a later session does not fill one in with a plausible default.
`AP_PNUGIMBAL_LOOP_HZ` is still the skeleton's 100 for the same reason - nothing has said
what rate the chosen mode wants.

**Bench checklist, when the motors are available.** None of this has met hardware:

1. Confirm the configured mode, CAN ID and Master ID of each motor in the PC utility, and
   that `CAN_Dn_PG_MODE` / `*ID` match. Confirm the P/V/T/KP/KD ranges and set `PG_*MAX` to
   the same numbers.
2. **Velocity units.** The manual mixes rad/s and rev/s. Measure one slow rotation before
   any physical-unit control is exposed, and do not silently multiply by 2*pi.
3. **Quantization.** Send the four MIT vectors the tests already pin and check what the
   motor does. The +-6 rad/s rows in the manual are the ones expected to disagree.
4. **Universal commands in velocity mode.** They are DLC 8 on an ID whose ordinary command
   is DLC 4, and the manual gives no example. This is the single most likely thing to be
   wrong.
5. Zero-setting persistence and what clearing a fault actually recovers - both unspecified.

---

**CAN Update Plan Steps - one bus, three protocols - PARTLY APPLIED**

*Decided 2026-10-15 after 5.1.10. **Steps 1, 1b and 1c are applied** (5.1.11 to 5.1.13);
steps 1d, 2, 3 and 4 are not - see the status column in the steps table below. This
supersedes the 5.1.9 assumption that each PNU CAN device would get its own
`AP_CAN::Protocol` value and its own bus.*

**The bus map.**

| Port | Bitrate | Carries | Driver |
|---|---|---|---|
| **CAN1** | **1,000,000 bit/s** | PMU + gimbal (3 CubeMars) + SAR | one `AP_PNU_CAN`, three backends |
| **CAN2** | 1,000,000 bit/s | DroneCAN GPS (Here4) only | `AP_DroneCAN`, unchanged |

**The bitrate is forced by the gimbal, not chosen.** The CubeMars GLII is **fixed at
1 Mbit/s** - the manual lists bitrate as a bus property and *not* among the things the vendor
PC utility can set (`PNUGimbal_CAN_Protocol_Type_CubeMars.md`, sec 1). A 500 kbit/s CAN1 was
planned and **abandoned for this reason**; **the PMU is reconfigured from 500 k up to
1 Mbit/s** instead. `CAN_P1_BITRATE` therefore stays at its default of 1000000 and needs no
parameter change - but the PMU does, on the device.

Secondary benefit, worth recording because it bears on the stabilisation loop: 1 Mbit/s
roughly **halves** the bus cost of gimbal traffic. Three 8-byte standard frames out and three
back is ~0.8 ms, so 100 Hz costs ~8% of the bus and 500 Hz ~39%; at 500 kbit/s those were
~16% and ~78%, the latter unusable. PMU traffic is ~1.8%. The note that
`AP_PNUGIMBAL_LOOP_HZ = 100` "is very likely too slow for motor control" is therefore no
longer constrained by the wire.

**Why one driver and not three.** `AP_CANManager::init()` loops over *interfaces*, and each
interface carries exactly one driver number (`AP_CANManager.cpp:139-152`). Many interfaces to
one driver is supported; **one interface to many drivers is not**, and a driver with no
interface is never allocated at all, so it fails silently. `CAN_Dn_PROTOCOL2` does not help:
it is a DroneCAN-hosted auxiliary slot, it requires the primary driver to implement
`add_11bit_driver()` - **only `AP_DroneCAN` does** (`AP_CANDriver.h:39`) - it accepts **one**
aux driver, and that driver must be a `CANSensor` with no thread of its own
(`AP_CANSensor.cpp:34-43`). Three protocols on one wire means one driver with three backends.

**CAN ID allocation - SETTLE THIS BEFORE THE SAR ICD IS WRITTEN.**

The PMU is 29-bit extended and everything else is 11-bit standard, so `isExtended()` separates
the PMU for free. **It does NOT separate the gimbal from the SAR** - both are 11-bit, so they
must be told apart by ID, and the SAR ICD is still being written. Reserve as follows:

| Range | Owner | Notes |
|---|---|---|
| `0x000` | gimbal feedback (Master ID) | CubeMars default; configurable via `CAN_D1_PG_MSTID` |
| `0x001`-`0x00F` | gimbal MIT commands | `(0 << 8) \| motor_id`, motor ids 1..15 |
| `0x101`-`0x10F` | gimbal position/velocity commands | `(1 << 8) \| motor_id` |
| `0x201`-`0x20F` | gimbal velocity commands | `(2 << 8) \| motor_id` |
| `0x300`-`0x3FF` | **reserved** | headroom for a second gimbal Master ID or a 4th axis |
| **`0x400`-`0x7FF`** | **SAR** | assign every SAR message a distinct id in here |
| `0x10000000`+ | PMU | 29-bit extended, `FlagEFF` set on every id |

The gimbal occupies `0x000`-`0x2FF` because its command id is `(mode << 8) | motor_id` and all
three modes must stay addressable without re-allocating. **Do not give the SAR an id below
`0x400`.** `AP_PNU_CAN::init()` will reject a configuration where a SAR id collides with the
gimbal Master ID or any gimbal command id, rather than letting the demux be ambiguous.

**The steps.** Each is independently revertable and each builds on its own.

| # | Step | Files | Risk | Verified by |
|:--:|---|---|:--:|---|
| **0** | Confirm device bitrates and the SAR id plan | - | - | **DONE 2026-10-15** - bus is 1 Mbit/s, SAR is 11-bit with distinct ids per message |
| **1** | **DONE in 5.1.11.** **`AP_PNU_CAN` multi-backend driver.** Generalise the 5.1.10 frontend: array of backends rather than one pointer, `AP_PNUGimbal_Backend` becomes the protocol-neutral `AP_PNU_CAN_Backend`, `handle_frame()` returns `bool` (first backend to claim a frame wins, the `MultiCAN` pattern at `AP_CANSensor.h:94`), `send_frame()` carries flags. CubeMars becomes a backend. **`CAN_Dn_PNU_OPTS` lands here** (see the device selection section below), and `CAN_Dn_PG_TYPE` drops `0:None` and defaults to `1:CubeMars`. Registered on protocol **16 for now**, PMUCAN untouched on 15 | new `AP_PNU_CAN/`, `AP_PNUGimbal/` | **none to flying code** | **DONE** - SITL + CubeOrangePlus build, `AP_PNU_CAN_ENABLED=0` builds, 10 codec tests pass |
| **1b-1d** | **The gimbal track**, branching off step 1 - mount backend, lifecycle state machine, HOMING. **1b and 1c are DONE (5.1.12, 5.1.13); 1d is not.** Its own table and risk notes are in "Gimbal stabilisation and the mount state machine" near the end of this file | `AP_Mount/AP_Mount_PNUGimbal.*`, `AP_PNUGimbal/` | low to **HIGH** for 1d | see that section |
| **2** | **DONE in 5.1.15, BENCH-TESTED 2026-10-19 except the shared bus.** **PMU becomes a backend.** `AP_PMUCAN` changes base class; protocol **15** instantiates `AP_PNU_CAN`, so `CAN_Dn_PNU_OPTS` bit 0 becomes live; the 10 Hz per-message rate is **explicit**; health is **id-based**; **16 kept as a deprecated alias**. Protocol 17 deferred to step 3 | `AP_PMUCAN/`, `AP_PNU_CAN/`, `AP_CAN.h`, `AP_CANManager.{h,cpp}`, `AP_Arming.cpp` | **HIGH** | **PASSED**: telemetry, command paths, prearm, in-flight failsafe and recovery. **OUTSTANDING**: shared bus with `PNU_OPTS=3`, and Stop-command timing. The ~1.0 s interlock duration PASSED at ~1-1.5 s |
| **2b** | **Simulated CAN devices for SITL.** **Fake PMU VALIDATED in SITL 2026-10-29.** Fake CubeMars motors still to come. **Ordered before 2c on purpose** - it is how 2c's windows get measured instead of reasoned about. See the section below | `Tools/pnu/sitl_can/` - no firmware change | low | **SITL-flown by PNU**: choke, the choke crank, Start, warm-up idle, cooling trip, Stop, takeoff, RTL with landing, and **the silent trigger tripping the D9 failsafe to RTL**. 43 offline tests pass |
| **2c** | **DONE in 5.1.18, MEASURED IN SITL.** Two independent time windows on PNU-ISSUE(D7)'s start sequence - a **500 ms TC1 staleness reset** and a **10 s stop-before-start window**. See the section below | `AP_PMUCAN/` | **HIGH** - engine start logic | **all four scenarios AS EXPECTED, and the original defect reproduced with guard 1 disabled.** Not tested against a real PMU, engine, KGCS or LTE link |
| **3** | **SAR backend** under the same driver, 11-bit, ids `0x400`-`0x7FF`. Protocol 17 retires here, deferred from step 2. **The ICD arrived 2026-10-30 (`SAR_ICD_PNU.md`) and PNU promoted this ahead of the fake gimbal** - see the SAR Integration Plan below, which breaks it into 3a-3g | `AP_PNUSAR/`, `pnu_kal.xml`, `GCS_PNU.cpp`, `Tools/pnu/sitl_can/` | **MEDIUM** - observation lifecycle and an RTK start interlock | **3a done in 5.1.19**, builds only; 3b-3g ahead |
| **4** | Bus documentation, bitrate note, README and `version.h` | `README.md`, `version.h` | none | - |

**Protocol number: 15 is kept.** The enumerator is renamed `PMUCAN = 15` -> `PNU_CAN = 15`; the
**value is unchanged**, so no deployed aircraft needs a parameter change. 17 is removed - it
was added at 5.1.9 and never left the bench.

**16 survives step 2 as a deprecated alias - decided 2026-10-15.**

Step 1 put `AP_PNU_CAN` on **16** so that the PMU could stay undisturbed on 15. Step 2 moves
it to 15, which leaves every bench aircraft configured during step 1 pointing at a value that
no longer exists - and the factory's `default: continue` skips an unknown protocol **in
silence**. No driver, no parameters, no message, nothing on the PNU bus at all. That is the
same shape of failure as PNU-ISSUE(D6) and D10 Stage 1, and it is not worth re-learning on a
bench at midnight.

So step 2 keeps 16 working and says so:

```text
AP_CAN.h            PNU_CAN        = 15,   // was PMUCAN
                    PNU_CAN_LEGACY = 16,   // deprecated; delete when 5.1.x goes OFFICIAL
                    // 17 was PNUSAR -- do not re-use

AP_CANManager.cpp   case AP_CAN::Protocol::PNU_CAN:
                    case AP_CAN::Protocol::PNU_CAN_LEGACY:
                        ... allocate AP_PNU_CAN ...

AP_Arming.cpp       both enumerators listed, or -Werror=switch fails the build

AP_PNU_CAN.cpp      get_pnu_can() accepts either value
                    report_startup() adds, at MAV_SEVERITY_WARNING and once per boot:
                      "PNU_CAN: CAN_D%u_PROTOCOL 16 is deprecated, use 15"
```

**16 is removed from the `PROTOCOL` `@Values` list** at the same time. The dropdown is for
what an operator should *choose*; the enum and the factory case are for what already exists
in a stored configuration. Anyone still on 16 keeps flying and is told to move, and nobody
picks it fresh.

It rides the existing one-shot startup report, so it costs no new rate limiter and no new
state - about six lines in total. **Delete the enumerator, the factory case, the arming case
and the warning when the 5.1.x DEV series goes OFFICIAL.**

**Device selection - `CAN_Dn_PNU_OPTS`, settled 2026-10-15, lands in step 1.**

Until now, "no PMU" meant not setting `CAN_Dn_PROTOCOL = 15`. One driver carrying all three
protocols takes that away, because 15 is also how the gimbal and the SAR are reached - and an
aircraft with a gimbal and a SAR but **no PMU** is a configuration that will certainly fly.
So presence becomes a bitmask.

| Param | Chars | Meaning |
|---|---:|---|
| **`CAN_Dn_PNU_OPTS`** | 15 | bit 0 PMU, bit 1 gimbal, bit 2 SAR. **Default 1** (PMU only) |
| `CAN_Dn_PG_*` | <=15 | gimbal settings, **names unchanged from 5.1.10** |
| **`CAN_Dn_PNU_SCHK`** | 15 | SAR Start interlock bitmask, added in step 3d. bit 0 RTK Fixed, bit 1 GPS fix age, bit 2 gimbal attitude age, bit 3 navigation health. **Default 11** |

It is a **new `PNU_` subgroup** on `CANDriver_Params` at index 8, and it holds **everything** -
the device bitmask and the gimbal settings in one flat group. The plan originally said `PG_`
would survive as a sibling; **that turned out to be impossible** and was corrected while
implementing step 1. `AP_SUBGROUPPTR` resolves `var_info` from the *class*
(`AP_Param.h:165`), so two sibling subgroups both pointing at the one `AP_PNU_CAN` object would
get the same table, and nesting a per-device group eats so much of the 16-character name
budget that the gimbal parameters become unreadable (`CAN_D1_PNU_G` leaves four characters).
One flat group fits all twelve names: `CAN_D1_PNU_` is 11 characters, leaving 5.

**Index 7, the 5.1.10 `PG_` group, is retired and NOT reused.** AP_Param stores by index
rather than by name, so reusing it would make a `PG_` value saved on a bench aircraft read
back as a different parameter. PNU's own earlier suggestion, `PNUOPTION`, is **not usable**
either - any suffix must fit the 16-character total.

**The default is 1 so that step 2 is a no-op for every existing aircraft:** `CAN_D1_PROTOCOL
= 15` with nothing stored gives PMU-only, exactly today's behaviour. One consequence to
accept: during step 1 `AP_PNU_CAN` sits on protocol 16 and the PMU backend does not exist yet,
so gimbal bench work uses `PNU_OPTS = 2`. Better one awkward step than a default that changes
between steps.

**`CAN_Dn_PG_TYPE` stops being a second on/off switch.** `0:None` is dropped and the default
becomes `1:CubeMars`; the parameter now only answers *which* gimbal protocol, while bit 1
answers *whether a gimbal is fitted*. Two switches for one thing is how a mount ends up
silently disabled by the one nobody looked at. This is free to change because `PG_TYPE`
shipped in 5.1.10 and is on no aircraft.

**What happens with no PMU - verified against the code, not assumed.**

| `PNU_OPTS` | Result |
|---|---|
| `0` | Driver allocated, **zero backends**. Valid - bus wired, everything disabled. The startup report says so |
| `1` (default) | PMU only. Identical to today |
| `2` / `4` / `6` | Gimbal and/or SAR, **no PMU**. No PMU failsafe, no prearm warning |
| `7` | All three |

With no PMU backend, nothing ever writes `PMU_Ctrl_Echo.PMUCAN_Fail`, so it stays at its
static-init value of **0**, and `failsafe_pmucan_check()` takes the `== 0` branch and returns
(`events.cpp:561`). `AP_Arming_Copter::pmucan_checks()` likewise returns true at `== 0`. So
**no failsafe and no prearm warning**, which is correct for an aircraft deliberately flown
without a PMU. The gimbal and the SAR have no failsafe of their own at all -
`AP_PNUGimbal::healthy()` exists and nothing in the tree consumes it.

That safety rests on `PMUCAN_Fail` only ever becoming 1 from inside the PMU backend's own
code. True by construction, but **comment it at the site**: raising the global's static
initialiser to 2 "for honesty" would fire `"PMU not healthy (state 2)"` on every prearm of a
PMU-less aircraft - the permanent-warning-crowds-out-real-faults problem 5.1.8 removed.

**The residual, accepted:** with bit 0 clear, KGCS is told `PMUCAN_Fail = 0`, i.e. a healthy
PMU that is not fitted. That is exactly today's behaviour when `CAN_Dn_PROTOCOL` is not 15,
so it is not a regression - but one forgotten bit now produces it where a whole wrong protocol
number used to be needed. The mitigation is free and belongs in step 1: **the startup report
names the enabled backends once per boot**, e.g. `PNU_CAN: CAN_D1 Gimbal+SAR ready (no PMU)`
or `PNU_CAN: CAN_D1 no backends enabled (PNU_OPTS=0)`. A forgotten bit is then visible at
once, with no recurring warning and no ICD change.

**When the parameters actually appear.** `AP_SUBGROUPPTR` is a *pointer* subgroup, and
`AP_Param::adjust_group_offset()` returns false while that pointer is null
(`AP_Param.cpp:417-428`), so `PNU_OPTS` and `PG_*` **do not exist** - they are skipped during
enumeration, not merely hidden - until `AP_CANManager::init()` has allocated the driver. That
needs three things, not one:

1. `CAN_Dn_PROTOCOL = 15`
2. `CAN_Pn_DRIVER = n` - **defaults to 0 (Disabled)** on CubeOrangePlus, since
   `HAL_CAN_DRIVER_DEFAULT` is 0 and the hwdef does not override it
3. **a reboot** - `PROTOCOL` is `@RebootRequired: True` and allocation happens once in `init()`

A GCS parameter refresh before the reboot shows nothing. That is the step most likely to look
like a bug when it is not. Appearing at defaults also does not mean written to EEPROM - normal
AP_Param behaviour, only non-default values are stored.

Note that `PNU_OPTS = 0` still allocates the driver. It has to: otherwise the subgroup would
vanish and the bitmask could never be set back.

**The directory layout does not change, and that is deliberate.** Every reference to
`AP_PMUCAN` from outside its own folder goes through two globals - `PMU_Status` and
`PMU_Ctrl_Echo` - plus `gcs().PMU_Ctrl` / `PMU_Ctrl_Seq`:

| File | Uses |
|---|---|
| `ArduCopter/events.cpp` | `PMU_Ctrl_Echo.PMUCAN_Fail` |
| `ArduCopter/AP_Arming_Copter.cpp` | `PMU_Ctrl_Echo.PMUCAN_Fail` |
| `libraries/GCS_MAVLink/GCS_PNU.cpp` | `PMU_Status.*`, `PMU_Ctrl_Echo.*` |
| `AP_CANManager.{h,cpp}` | factory case and driver slot |

Keep the globals' names and keep them declared in `AP_PMUCAN/AP_PMUCAN.h`, and **those three
vehicle files need no change at all** - the failsafe, the prearm check and TM1/TM1x keep
working, and PNU-ISSUE(D14)'s declaration-beside-definition rule survives. That is the
largest single piece of de-risking available on step 2, so do not "tidy" the layout.

**Four things that break silently - the risk register for step 2.**

1. **The PMU's 10 Hz is currently an accident of the code, and must be made explicit.**
   `TXspin()` runs at 50 Hz and rotates `_cmd_idx` through 5 commands, so each command happens
   to go out at 10 Hz; `_rtr_idx` rotates 5 RTR polls the same way. **The PMU ICD specifies
   10 Hz** - confirmed by PNU 2026-10-15 - so the rate is correct but nothing in the source
   says so. PNU-ISSUE(D7)'s engine interlock is calibrated on it: `engineonoffstate()` is
   reached once per rotation, which is why its threshold of 10 ticks means ~1.0 s. Define the
   rate as a named constant and derive the rotation from it, so the behaviour is byte
   identical and the intent is readable. **Do not let a shared loop change the rotation** -
   that changes what Start and Stop mean, with nothing failing visibly.
2. **PMU health must become id-based.** `RXspin()` treats *any* `receive() > 0` as proof the
   PMU is alive (`AP_PMUCAN.cpp:303-325`). On a shared bus, gimbal traffic would hold
   `PMUCAN_Fail = CONNECTED` with the PMU dead, and `failsafe_pmucan_check()` would never
   fire. Liveness has to key on the five PMU status ids. **This fix is not optional and is the
   reason step 2 cannot be a mechanical move.**
3. **`send_frame()` must carry flags.** The 5.1.10 version rejects `can_id > MaskStdID`.
   `pmucan_rtr()` sends **extended + RTR** frames with the flags baked into the id
   (`AP_PMUCAN.cpp:69-73`), so the shared sender takes the full 32-bit id and each backend
   validates its own frame shape.
4. **Sub-rate hooks.** PMU needs `send2gcs()` at 10 Hz and `sendchanges()` at 1 Hz off the
   shared tick. The driver hands backends the tick count rather than each inventing a timer.

**What this costs from 5.1.10 - about 15% of that diff.**

- **Kept unchanged:** the CubeMars codec and all 10 tests, the backend-interface concept, the
  bounded RX drain, the per-cause rate-limited reporting, the startup-report and init-error
  mechanism, the `CAN_Dn_PG_*` parameters.
- **Changed:** class and file names, `send_frame()` signature, `handle_frame()` return type,
  one backend pointer becomes an array, the `PG_` group gains a sibling.
- **Reverted:** `PNUGimbal = 16` and `PNUSAR = 17` in `AP_CAN.h`, their two cases in
  `AP_Arming.cpp`, their factory cases, and `16:PNUGimbal` in the `PROTOCOL` `@Values`. 15
  stays.

---

**Gimbal stabilisation and the mount state machine - researched 2026-10-16, PARTLY APPLIED**

*This answered the 5.1.9 open question "does `AP_PNUGimbal` back an `AP_Mount` backend?" -
it does. **The mount backend landed in 5.1.12 (step 1b) and the state machine in 5.1.13
(step 1c).** What is still only a design record here is **HOMING (step 1d)**, which is
blocked on the four bench-measured values in 6d.*

**1. `AP_Mount` already stabilises against FC attitude, and the precedent is camera-less.**

`AP_Mount_Servo::update_angle_outputs()` (`AP_Mount_Servo.cpp:94-152`) is the whole of it, in
about 40 lines: take the earth-frame target, rotate the AHRS lean angles into the gimbal's
yaw frame (`ahrs_angle_rad.rotate(-yaw_bf_rad)`), subtract them from roll and pitch, then
apply a **gyro lead filter** (`MNT1_LEAD_RLL` / `MNT1_LEAD_PTCH`). It correctly does no
stabilisation in RETRACT or NEUTRAL.

The `requires_stabilization` constructor flag is what separates the two cases:
`Type::Servo = 1` passes **true** (`AP_Mount.cpp:80`) and `Type::BrushlessPWM = 7` passes
**false** (line 126), because a self-stabilising brushless gimbal does its own. **We are the
`true` case** - the CubeMars drives have no attitude reference of their own.

**A camera is not required.** The only pure virtuals in `AP_Mount_Backend` are
`has_pan_control()` and `get_attitude_quaternion()`. Every camera method - zoom, focus,
record, tracking, lens, palette - has a `return false` default, and `AP_Mount_Servo`
implements none of them. The 5.1.9 worry that "`AP_Mount` assumes a camera in places" is
answered: it assumes nothing.

**2. The rate worry from 5.1.9 is smaller than it looked.**

| Hook | Rate | Used by |
|---|---|---|
| `AP_Mount::update()` | **50 Hz** SCHED_TASK (`Copter.cpp:230`) | every backend |
| `AP_Mount::update_fast()` | **FAST_TASK**, main loop - 400 Hz on Copter (`Copter.cpp:153`) | only SoloGimbal, and only to accumulate gyro delta-angles for its own EKF |

The framing matters more than the numbers: **the CubeMars closes its own position, velocity
and current loops at the motor.** The FC feeds setpoints, it does not close a servo loop.
`AP_Mount_Servo` does exactly this at 50 Hz into real RC servos. 100 Hz is very likely
enough, 400 Hz is available through a hook nothing else contends for, and **the bus is not
the constraint** - three command frames is ~0.4 ms at 1 Mbit/s, so 100 Hz costs ~4% and even
400 Hz ~16%.

**A split-rate option, worth measuring before adopting:** targeting (where to point, in earth
frame) changes slowly; attitude compensation changes fast. `AP_Mount` could hand over an
*earth-frame* target at 50 Hz and the `AP_PNU_CAN` thread could subtract AHRS at 100 Hz just
before transmitting, giving fresh compensation without raising the mount task rate.
**UNVERIFIED:** that calls `AP::ahrs()` from the CAN thread, and nothing here has checked
that is safe. Verify before relying on it. The fallback costs nothing - do the subtraction in
the mount task and let the CAN thread transmit the latched body-frame value, which is what
the code already does.

**3. Recommended structure.**

```text
AP_Mount  (50 Hz SCHED_TASK, update_fast available)
  +- AP_Mount_PNUGimbal : AP_Mount_Backend      thin targeting adapter, MNT1_TYPE = 15
       +- AP::pnu_can()->get_gimbal()           AP_PNUGimbal_Backend, protocol-neutral
AP_PNU_CAN  (100 Hz thread)
  +- AP_PNUGimbal_CubeMars                      protocol only
```

Free, with no new code: mount modes, RC targeting, the MAVLink gimbal manager, ROI /
GPS_POINT / SYSID / HOME, the `MNT1_*_MIN/MAX` limits, the lead filter, `healthy()` feeding
`AP_Mount::pre_arm_checks()` (`AP_Mount.cpp:733`), and the dual-GCS compatibility strategy
recorded further down this file.

To write: `has_pan_control()` returning true, `get_attitude_quaternion()`,
`send_target_angles()`, `natively_supported_mount_target_types()` returning
`NATIVE_ANGLES_ONLY`, `healthy()`, and the ~40 stabilisation lines adapted from
`AP_Mount_Servo`. `MNT1_TYPE = 15` - 14 (XFRobot) is the highest today.

*Bonus worth having:* `AP_Mount_Servo::get_attitude_quaternion()` admits "No feedback from
gimbal so simply report demanded servo angles". **This mount has real measured angles**, so
`GIMBAL_DEVICE_ATTITUDE_STATUS` (285) would carry truth rather than intent.

**4. The state machine is NOT the mount mode, and conflating them would be a mistake.**

`MAV_MOUNT_MODE_*` says **where to point**. The lifecycle state machine says **whether the
hardware can be pointed at all**. RETRACT is a targeting mode - point at the retract angles -
**not** "motors off". They are orthogonal and both are needed.

| State | Motors | What it does |
|---|---|---|
| **INIT** | enabling | clear errors, enable, wait for status `ENABLED` and first feedback on all three axes |
| **HOMING** | enabled, **MIT, low stiffness** | the Appendix 1 endpoint search - its own sub-sequence, see below |
| **ALIGN** | enabled | at the new zero; ramp stiffness and slew-limit onto the first `AP_Mount` target |
| **RUN** | enabled | normal stabilisation; `AP_Mount` targets accepted |
| **MAINTENANCE** | **disabled, FC transmits nothing** | the PC utility is on a motor's UART; see below |
| **FAULT** | disabled | reported, no commands; operator-cleared back to INIT |

**ALIGN is not optional.** Section 9 of `PNUGimbal_CAN_Protocol_Type_CubeMars.md` requires
position control to start from the measured current angle. Enable the motors and immediately
apply an `AP_Mount` target at operating stiffness and the mount **snaps** to it - with a
heavy SAR payload that is the one thing in this design that could break hardware. The backend
already refuses to command until `set_target()` is called; ALIGN is what makes the *first*
target safe.

**MAINTENANCE must be blocked while armed.** Disabling the motors in flight drops the SAR
onto its stops. That needs an arming interlock, and `healthy()` must return false in
MAINTENANCE so `AP_Mount::pre_arm_checks()` catches it before anyone tries.

**FAULT needs a policy PNU has to choose.** Status codes 0x8-0xE are all faults but they are
not equivalent: comms loss (0xD) and MOS overtemperature (0xB) are recoverable, overcurrent
(0xA) and overload (0xE) may not be. Feedback staleness belongs here too - the backend
already detects it at 500 ms.

**5. Where the state machine lives - lower than might be expected.**

Not in `AP_Mount_Backend`: it is a shared library and AGENTS.md forbids PNU-specific logic
there. Not in `AP_PNUGimbal_CubeMars`: a second motor protocol would need the same lifecycle.

Put it in the **gimbal layer under `AP_PNU_CAN`**, with `AP_Mount_PNUGimbal` as a thin
targeting adapter. Three reasons: it runs in the CAN thread where the feedback actually
arrives, so transitions happen immediately rather than waiting for the next 50 Hz tick; it
survives `HAL_MOUNT_ENABLED=0`, which this project supports (PNU-ISSUE D13); and it stays
protocol-neutral.

**6. The four questions - ANSWERED BY PNU 2026-10-16.**

**(1) Both frames, with an earth-frame yaw lock.** The gimbal must switch between body frame
and earth frame, and the intended normal mode is a **yaw fix** that holds the line of sight
in earth frame regardless of vehicle heading, **clipped at the mechanical limit**.

This maps straight onto existing `AP_Mount` machinery and needs no new concept:
`MountAngleTarget` already carries `yaw_is_ef`, `roll_is_ef` and `pitch_is_ef` per axis, and
`get_bf_yaw()` converts an earth-frame yaw target to body frame by subtracting vehicle
heading. `AP_Mount_Servo` then constrains the result to `MNT1_YAW_MIN/MAX`. Clipping at the
limit is what the existing code already does.

**(2) Two limit sets, not one.**

| Axis | Mechanical | Operating |
|---|---|---|
| Roll | -30 .. +30 deg | -25 .. +25 deg |
| Pitch (tilt) | -30 .. +30 deg | -25 .. +25 deg |
| Yaw (pan) | -20 .. +20 deg | -15 .. +15 deg |

All in the **gimbal frame**. The payload is bulky and can foul the landing skid struts, which
is what the limits exist to prevent.

`AP_Mount` has only one set, so they are split by layer: **`MNT1_*_MIN/MAX` carry the
OPERATING limits** and `AP_Mount` clamps targets to them, while the **mechanical limits live
in the PNU gimbal backend as the last clamp before a frame is transmitted**. Defence in
depth - a bug in the targeting layer cannot drive the payload into the struts, because the
layer that owns the wire clamps again. The 5 degree margin is what absorbs stabilisation
overshoot.

**(3) No home switch and no index - zero is encoder-relative, so INIT must find it.**
The procedure is **Appendix 1 of `PNUGimbal_CAN_Protocol_Type_CubeMars.md`**: drive to the CW
endpoint and reconfirm, drive to the CCW endpoint and reconfirm, validate the travel between
them, move to `Theta_mid = (Theta_CW + Theta_CCW) / 2`, confirm motion has stopped, send the
zero-set command, verify, then apply targets in the new coordinate system.

Two details from the appendix that are easy to miss and change the implementation:

- **Endpoint detection is by ANGLE ONLY.** Overcurrent, undervoltage and overload are
  **emergency stop conditions, not endpoint criteria**. So an endpoint is "the angle stopped
  changing while motion was still commanded" - a stall detector with an angle-delta threshold
  and a dwell time, neither of which the appendix specifies. Both need to be parameters and
  both need bench measurement.
- **MIT mode is mandatory for homing.** The appendix specifies it, and the reason is
  almost certainly that MIT is the only mode where the host bounds the push into the hard
  stop - low `kp` plus a small feed-forward torque. Position/velocity mode runs a cascaded
  loop whose torque the host cannot limit from the CAN frame. So if `CAN_Dn_PNU_MODE` is not
  0, the homing sequence **must refuse to run and say so**, rather than driving a bulky
  payload into a mechanical stop at whatever torque the motor decides.

The travel validation is more valuable than it looks: `Theta_CW - Theta_CCW` should be about
60 / 60 / 40 degrees. Materially less means **something is obstructing the payload** - the
landing-skid fouling case - and that is caught on the ground instead of in the air.

Abort condition, per the appendix: **any feedback status other than 1 (ENABLED)**.

**(4) MAINTENANCE is entered by a new parameter, and it means more than "motors off".**
Its purpose is connecting a PC to one motor's **UART** with an external cable, to run the
vendor configuration utility. So in MAINTENANCE the FC must **transmit nothing at all** on
the bus - not commands, not enables. CAN has priority over PWM on these drives (manual p.38),
and a live command stream while the utility is writing configuration is asking for trouble.
Receiving is harmless and useful, so the driver keeps listening and keeps reporting.

**The hazard is leaving MAINTENANCE, not entering it.** The utility can change the CAN ID,
the control mode and the P/V/T/KP/KD ranges - and the central warning of the protocol note is
that a host/device mismatch **does not fail, it silently rescales every command and every
reading**. So: entering MAINTENANCE is live, but **leaving it requires a reboot**, which
forces a fresh INIT and a fresh homing run against whatever the motor now believes. A
parameter was the right choice over an RC switch for exactly this reason - a stray RC glitch
must not be able to drop the FC off the bus.

**6a. What the answers changed - five consequences, none of them obvious.**

**(i) HOMING is the most hazardous routine in this driver, and it needs its own guards.**
It deliberately drives a bulky payload into three sets of hard stops. Non-negotiable:
**it must not run while armed**; the vehicle should be on the ground and level; each endpoint
search needs a **timeout** as well as a stall threshold, because a motor that never reaches a
stop must give up rather than push forever; and the whole sequence aborts on any status other
than 1. The reconfirm pass in the appendix exists for the same reason - one reading can be a
transient obstruction, two agreeing readings is an endpoint.

**(ii) The axis order is a PNU decision, not a code decision.** Homing all three at once
sweeps the largest volume and is the most likely to foul the struts. Sequential is safer, but
which order is safe depends on the geometry - and parking the other two axes somewhere
specific while one homes may be required. **Open.**

**(iii) Stabilisation WILL saturate in normal flight, and that is physics, not a defect.**
With roll and pitch operating at +-25 degrees, a vehicle leaning 25 degrees consumes the
entire range, and a multicopter in forward flight leans 15-25 degrees routinely. Yaw is
tighter still: an earth-frame yaw lock breaks as soon as the vehicle heading moves more than
15 degrees from where the lock was set. Clipping at the limit is correct behaviour. It needs
**recording here so nobody reports it as a bug**, and the mount should be able to say that it
is on a limit rather than silently appearing to stop tracking.

**(iv) Does the zero survive a power cycle? UNKNOWN, and it decides how often HOMING runs.**
Section 3 of the protocol note states that zero-setting persistence is **unspecified**. If
the zero persists, home once at commissioning and merely sanity-check the angle at boot. If
it does not, the payload is driven into its stops **before every flight**, which is both wear
and a daily hazard. **This is a one-hour bench test** - home, power cycle, read the angle -
and it should be done before HOMING is written, because it changes whether homing is a
startup step or a commanded action.

**(v) A target slew-rate limit earns its place twice.** ALIGN needs one so that the first
`AP_Mount` target - which may be 25 degrees away from the new zero - does not arrive as a
step at full stiffness. RUN needs the same limit to bound how fast anything can command the
payload to move. One mechanism, two jobs, and it is cheaper than ramping stiffness.

**6b. HOMING detail - settled by PNU 2026-10-16.**

**The zero does NOT persist. Assume it never does.** The drives use a **relative incremental
encoder** and the manual does not state persistence either way, so the safe assumption is the
pessimistic one. Three consequences, and the third is the important one:

- **HOMING runs on every boot**, so it is on the critical path to being flight-ready and
  **arming must be blocked until it completes** - `healthy()` returns false in every state but
  RUN. The `ARMING_SKIPCHK` bypass that PNU-ISSUE(D9b) established for the PMU is the same
  escape hatch for an operator who must launch anyway.
- **At power-up the encoder reads 0 wherever the mount happens to be sitting**, so the boot
  feedback angle is meaningless as an absolute angle.
- Therefore **the mechanical-limit clamp protects nothing until homing completes** - plus or
  minus 30 in encoder terms is not plus or minus 30 mechanically until zero is at centre.
  During homing the *only* thing protecting the payload is the low `kp` and the bounded
  feed-forward torque. That is why MIT mode is mandatory, and it makes that requirement the
  single most load-bearing fact in this design.

**Axis order: roll, then pitch, then yaw.** Roll first because **the mass offset is on the
roll axis only**, so it is the axis that sags and swings under gravity and wants to be under
control first. Yaw last because it is safer to swing the payload in yaw with pitch already
neutral, which keeps the swept volume small.

**The two idle axes must be HELD, not left floppy.** Enable all three and command each to hold
its power-up position before homing begins; the manual does not define what an enabled motor
does with no command outstanding, and a bulky payload swinging on an uncommanded axis while
another drives into a stop is exactly the collision the limits exist to prevent.

**Stall dwell: 1 second, provisional, for the first bench run.** The angle-delta threshold
that goes with it, the approach velocity, and the homing `kp` and feed-forward torque are all
still unmeasured - they become parameters with provisional defaults and are tuned on the
bench. Note the trade: a slow approach is gentle on the stops but makes a three-axis homing
run long, and homing now happens before every flight.

**6c. Travel validation - what it is and why it is worth the tolerance argument.**

After both endpoints are found, `Theta_CW - Theta_CCW` is the axis' total mechanical travel,
and the expected value is already known: **60 / 60 / 40 degrees** for roll / pitch / yaw. The
check is `|measured - expected| < tolerance`, and its value is that **the direction of the
error names the fault**:

| Measured travel | What it means |
|---|---|
| about 60 deg | normal |
| **much less**, e.g. 45 deg | **something is obstructing** - the SAR fouling a skid strut, a snagged cable, a bolt, ice. The case these limits exist for, caught on the ground |
| **much more**, e.g. 90 deg | the endpoint was not an endpoint - stall detection fired early on friction or a cable drag point |
| **about 2x or 0.5x**, e.g. 120 or 30 deg | **`CAN_Dn_PNU_PMAX` does not match the range configured in the motor** |
| 58 or 62 deg | normal variation - stop compliance under a heavy payload, stall detection firing just before true contact, encoder quantisation |

**The fourth row is why this check earns its place.** A PMAX mismatch is precisely the failure
the protocol note warns about - it does not fail, it silently rescales every command and every
reading - and **travel validation is the only runtime check in this design that can detect
it**, because it is the only place an encoder-derived number is compared against a known
physical quantity. A factor-of-two travel reading is an unmistakable signature. Nothing else
in the system would ever notice.

**Two tolerances, and they must not be conflated:**

| Tolerance | Provisional | Question it answers |
|---|---|---|
| **Reconfirm** | ~0.5 deg | the same endpoint found twice must agree - *was that a real stop?* This is what the appendix' reconfirm step is for |
| **Range** | ~3 deg on 60, ~2 deg on 40 | `CW - CCW` versus nominal - *is the axis obstructed, or is my scaling wrong?* |

Both figures are starting points for the bench, not measurements. Stop compliance with the SAR
fitted could consume most of 3 degrees by itself. Set the range tolerance too tight and every
boot aborts; too loose and a partially obstructed axis passes, and the aircraft flies with a
reduced range that the mechanical clamp does not know about.

*ANSWERED 2026-10-19 - the operating limits stay NOMINAL.* As long as the measured travel is
at least twice (mechanical minus `PNU_MARG`) - 50 degrees on a 60 degree axis - `MNT1_*` keeps
its +-25 and homing passes. Below that it aborts as obstructed. Deriving tighter limits from
each boot's measurement was rejected: stop compliance under a heavy payload would make the
mount's usable reach vary with how hard it happened to press on its stops, which is worse than
a fixed number everyone can reason about.

**6d. Still open.**

1. **Angle-delta stall threshold, approach velocity, homing `kp` and feed-forward torque** -
   bench measurement; 1 second dwell is the only provisional value settled so far.
2. **Reconfirm and range tolerances** - the 0.5 / 3 / 2 degree figures above are guesses.
3. **Whether operating limits are derived from measured travel** or stay nominal (6c).
4. **Sign convention - ANSWERED 2026-10-19, and it is a parameter.** The mount is still
   undergoing mechanical design changes, so the rotation direction of each axis is
   `CAN_Dn_PNU_DRLL` / `DPIT` / `DYAW` (0 normal, 1 reversed) rather than a source constant.
   Everything inside the gimbal layer is stored in the motor frame; the conversion happens
   only in `set_target()` and `get_feedback()`. This project was bitten once by a
   pitch-convention mismatch (5.1.2 / 5.1.3) - a parameter with one pair of boundaries is the
   answer to not repeating it.

**6e. Bench finding 2026-10-16 - yaw looks broken and is not.**

Roll and pitch confirmed on the first bench log: `MNT0.Roll` and `MNT0.Pitch` move opposite
vehicle roll and pitch, which is the AHRS correction and the whole of what step 1b set out to
prove. Yaw looked wrong and is not - **`MNT0.YawB` did not move, and `MNT0.YawE` read the same
as `ANG.Yaw`.** Both are one cause:

`AP_Mount_Backend::write_log()` computes `yaw_ef = wrap_180(yaw_bf + degrees(ahrs_yaw))`, so
with `YawB` at zero, `YawE` equals the vehicle heading **by construction**. And `YawB` is zero
because **yaw stabilisation is not in `AP_Mount_PNUGimbal` at all** - roll and pitch subtract
AHRS explicitly, but yaw goes through `MountAngleTarget::get_bf_yaw()`
(`AP_Mount_Backend.cpp:1013`), which subtracts heading **only when `yaw_is_ef` is set**:

```text
if (yaw_is_ef) { return wrap_PI(yaw - AP::ahrs().get_yaw_rad()); }   // earth -> body
return yaw;                                                          // already body-frame
```

and `yaw_is_ef` comes straight from the yaw lock (`AP_Mount_Backend.cpp:120`). With the lock
off - "follow", the default - the target is body-frame, nothing subtracts heading, and `YawB`
sits at the target forever. **That is the "yaw fix" not being switched on, not a defect.**

*Diagnosing it from a log without touching the code:* `write_log()` puts the desired yaw in
`DYawB` **or** `DYawE` depending on the frame, leaving the other NaN. `DYawB` populated and
`DYawE` NaN means the target is body-frame, so the lock is off.

*Turning it on:* `RCn_OPTION = 163` (`MOUNT_YAW_LOCK`), switch HIGH, with the mount in
RC_TARGETING (`MNT1_DEFLT_MODE = 3`). Or `GIMBAL_MANAGER_FLAGS_YAW_LOCK` over MAVLink. Note
that `set_yaw_lock()` **captures the heading at the moment it is switched** - the lock holds
whatever direction you were facing, not north - and that at `MNT1_YAW_MIN/MAX` of +-15 degrees
it clips after about 15 degrees of heading change, which is the expected behaviour recorded in
6a(iii).

**6f. Homing parameters - bench tuning order, 5.1.14.**

`CAN_Dn_PNU_HOME` is **0**.  Leave it there until the four values below are measured, and
work in this order - each one makes the next safe to try.

| # | Parameter | Default | How to get the real value |
|:--:|---|---|---|
| 1 | `PNU_HKD`, `PNU_HVEL` | 0.5, 10 deg/s | **Stall torque is HKD x HVEL (rad/s).** Pick the gentlest push that still reaches a stop against the payload's own friction. Start low and raise HKD; do NOT raise HVEL to get there, because that raises impact energy as well as torque |
| 2 | `PNU_HDLT`, `PNU_HSTL` | 0.1 deg, 1000 ms | Log an axis creeping into a stop and read the residual motion. Too small and encoder noise means a stall is never detected; too large and the axis is called stopped while still creeping. The dwell is the only value PNU has settled |
| 3 | `PNU_HRTOL` | 0.5 deg | Approach one stop twice by hand and measure the spread. Must stay below the 2% backoff - 1.2 deg on a 60 deg axis, 0.8 on 40 - or the second approach starts inside the tolerance and proves nothing. `init()` rejects it if not |
| 4 | `PNU_HTOL` | 3 deg | Measure stop-to-stop travel a few times with the SAR fitted. **Stop compliance under a heavy payload may consume most of 3 degrees on its own**, so this is the one most likely to need raising - but raising it blunts the obstruction check, so raise it only as far as the measurements demand |
| 5 | `PNU_HKP` | 5 | Holding stiffness for the idle axes and the move to the midpoint. Enough to stop the payload sagging, no more |

**Run the first homing with the SAR mass fitted but the aircraft restrained**, and watch the
statustexts: each axis reports its measured travel and midpoint before it zeroes, which is
the number that tells you whether the geometry is what everyone thinks it is.

**Both remaining design questions were answered 2026-10-19** - see 6c for the travel policy
and 6d item 4 for the direction parameters. What is left is purely bench measurement.

**Set `PNU_DRLL` / `DPIT` / `DYAW` before the first homing run**, and check them the cheap
way: command a small positive angle on one axis in RUN and confirm the payload moves the way
the mount convention says it should. Getting one wrong means that axis stabilises *into* the
disturbance rather than against it.

**7. Suggested steps, independent of the PMU track.**

Neither track touches the other, so they can run in either order.

| # | Step | Risk | Verified by |
|:--:|---|:--:|---|
| **1b** | **DONE in 5.1.12.** `AP_Mount_PNUGimbal` backend and the stabilisation maths, **commanding nothing**. Mechanical-limit clamp in the gimbal backend; `MNT1_*` carry the operating limits | low | **DONE and BENCH-CONFIRMED 2026-10-16** - `MNT0.Roll` and `MNT0.Pitch` move opposite vehicle roll and pitch in a log. Yaw needs the earth-frame lock enabled, see below |
| **1c** | **DONE in 5.1.13.** the lifecycle state machine - INIT, ALIGN, RUN, MAINTENANCE, FAULT - the arming interlock and the slew-rate limit. **No HOMING yet** | medium | **DONE** - builds, all five guards compile out, 10 codec tests pass. Bench needs the SAR mass fitted and the zero set by hand |
| **1d** | **DONE in 5.1.14, DEFAULTS OFF.** **HOMING**, the Appendix 1 sequence - roll then pitch then yaw, idle axes held, travel validated | **HIGH** - drives the payload into hard stops, and the mechanical clamp cannot protect it until this completes | **code DONE** - builds, guards compile out. **NOT bench-run**: `PNU_HOME` is 0 until the tuning in 6f is measured |

---

**Step 2b - simulated CAN devices for SITL - PROPOSED 2026-10-19**

*Everything in the CAN stack except the CubeMars codec is currently compile-verified only.
The codec has 10 unit tests; the state machines, the health logic and the interlock timing
have never been executed.*

**The hooks already exist upstream.** `AP_HAL_SITL` has two CAN transports
(`CANSocketIface.cpp:216-220`): multicast UDP, and **SocketCAN** which attaches to a Linux
`vcan%u` interface (`CAN_SocketCAN.cpp:32`). SITL is configured here with
`HAL_NUM_CAN_IFACES 2`, so the driver really does run in simulation.

**Use SocketCAN**, because it preserves real CAN semantics - extended ids and **RTR frames**,
which the PMU needs and a UDP shim may not carry faithfully.

```text
sudo modprobe vcan
sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up
# fake devices in python-can, SITL attached to vcan0
```

**What it would finally make testable:**

| Device | Covers |
|---|---|
| fake PMU | status frame decode; the leaky-bucket health transitions; **stop answering while a fake gimbal keeps chattering and confirm the failsafe fires** - the exact bug step 2 fixed, unprovable any other way short of hardware |
| fake PMU | the D7 engine interlock, including **step 2c's two windows**, measured rather than reasoned about |
| fake CubeMars | INIT -> ALIGN -> RUN, fault transitions, slew limiting |
| fake CubeMars with a travel limit | **HOMING** - the reconfirm mismatch and the travel-validation bounds, otherwise only testable by driving real hardware into real stops |
| both at once | the shared bus, which nothing has ever exercised |

**What it cannot do:** stop compliance, stall thresholds, real motor dynamics, real PMU
firmware behaviour. It does not replace the bench - it moves the logic errors off it.

**Cost:** a few hundred lines of Python per device. The CubeMars one is cheap because the
codec is written and tested, and the fake motor is the same maths in reverse; the PMU one
needs its five status frames built from the field layouts in `handleFrame()`.

**The fake PMU's engine model - SPECIFIED BY PNU 2026-10-27.**

*Superseded 2026-10-29: **`PMU CAN Comm Protocol.md` is now in the repository root** - an
English rendering of the PMU's Korean PDF ICD, written by PNU. The fake is built from that,
not from `AP_PMUCAN::handle_frame()`, so it no longer merely reproduces our own decoder's
assumptions. The cross-check of the two is below and it came out clean.*

**RPM ladder. No step may be skipped.**

| State | RPM | Throttle | Entered by | Load current | Head temps |
|---|---|---|---|---|---|
| STOPPED | 0 | 0% | boot, Stop, cold-start abort | - | decay to 25 at 5 degC/s |
| CHOKING | ramp to 1000, hold 2 s, then 0 | 8% | Start with **choke closed** | - | - |
| CRANK | 1000, hold 2 s | 8% | Start with **choke open** | - | - |
| WARM-UP idle | ramp to **4200** over 1 s, hold | 8% | 2 s at 1000 elapsed | - | climb 25 -> 100 at 5 degC/s |
| GROUND idle, cooling on | **5000** | 13% | head temps reach **80** | **6.3 A** | continues climbing to 100 |
| FLIGHT | **6200** | 50% | takeoff detected **from 5000 only** | **87.5 A** | climb to **180** at 5 degC/s |
| COLD-START ABORT | -> 0 | 0% | takeoff attempted below 5000 | - | decay to 25 |

CHOKING leaves the engine **stopped**: it reaches 1000 rpm for 2 s and stops, and the status
word must still read as not-running afterwards. Ceiling is ~6500 and the model never exceeds
7000 even at 100% throttle; above 6500 sets the over-speed bit rather than clipping silently.

**Battery and generator are tightly coupled** - same voltage, same current. The flight load is
87.5 A and *something* carries it: the engine's generator while the engine runs, the battery
the instant it stops. That is what makes the cold-start abort observable - RPM falls to zero,
ILD stays at the flight load, and **IBAT goes to -87.5 A (raw -875), negative for discharge**,
which is the only place in the whole model where IBAT is not zero.

`System_Voltage` is **48.0 V at all times**. `Battery_Temp` and `PMU_Temp` are **0** -
deliberately not modelled. `Fuel_Quantity` is **100** and the Fuel Low bit is held clear,
because the real aircraft has no fuel sensor.

**`PMU_Status.PMU_Status` - the 16-bit status word. NOT decoded anywhere in the FC today**;
`handle_frame()` stores it whole and `GCS_PNU.cpp` forwards it whole. Written down here
because the fake has to generate it and nothing else in the tree records it.

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

**Bits 13, 14 and 15 come from the ICD, not from the 2026-10-27 specification**, which
listed 11-15 as reserved; PNU has confirmed that was a slip in that message, so the ICD
stands. The ICD also gives no numeric
threshold for bits 6 and 10 - "above warning threshold" and "high-temperature warning" - so
the 6500 rpm and 230 degC figures are PNU's, and the fake uses them as the trip points.

**There is no "engine is running" bit.** Bit 1 is set only *during* the start, bit 2 only
*during* the stop, and both read 0 once the engine is up. Running is inferred from the RPM
range, and **that inference belongs to KGCS, not to the FC** - confirmed by PNU 2026-10-29.
Over-speed (bit 6) is the same division: it is how KGCS annunciates an abnormal generator,
and the FC's only job is to carry the word intact.

**So the FC passing `PMU_Status` through undecoded is the design, not an omission.** Nothing
here is a TODO. If a future session is tempted to parse this word in the FC, it needs a new
FC-side requirement first - the bits above exist to let the *fake* generate them faithfully.

**The silent trigger - the reason step 2b exists.** The fake PMU takes a runtime command
that **stops it transmitting without killing the process**, and a second that resumes.
Three lines, and it is the only way to reach the two tests that matter:

- the leaky-bucket health transitions and the D9 failsafe, start to finish, repeatably
- **the shared bus**: the PMU goes quiet while the fake gimbal keeps chattering. That is
  exactly the bug step 2 fixed - the old `RXspin()` took any traffic as proof of life - and
  it is unprovable on the desk any other way

**Defaults chosen here, not specified by PNU** - each is a one-line change:

- `Engine_Hour_Count` increments whenever RPM > 0
- `Current_Control_Command` and `Battery_Quantity_Command` echo the last BATCTRL received
- bit 1 is held from Start until the warm-up idle RPM is reached; bit 2 for the stop ramp
- the cold-start abort **latches**, needing a fresh Start, and sets bit 7

**All three open questions closed by the ICD and by PNU, 2026-10-29:**

1. **Scale is 0.1, not x100.** Every INT16 analog field - TEMP, IBAT, TEMP1, TEMP2, ILD,
   VBUS, GCMD, GAGE, PTEMP - is `physical = raw * 0.1`. The x100 guess was **wrong**, and
   it mattered: see the 6.25 A note below. The FC itself still applies no scale at all, so
   the conversion remains entirely KGCS's.
2. **`IBAT` is positive for CHARGING, negative for DISCHARGING** - ICD BATSTS, and confirmed
   by PNU. Only ILD and IBAT are measured, and the generator's output is inferred:

   ```text
   generated = ILD + IBAT
   ```

   A charging battery *adds* to what the generator must produce; a discharging one subtracts,
   because IBAT is then negative. **While the engine runs the fake holds IBAT at 0**, by
   PNU's instruction, rather than modelling a charge current - so `generated == ILD`
   throughout the normal ladder, and the only non-zero IBAT in the model is the abort.
3. **The cold-start abort latches** and needs a fresh Start. Confirmed.

**The cooling-device current is 6.3 A, raw 63** - settled by PNU 2026-10-29. The original
figure was 6.25 A, which is raw 62.5 and does not exist at 0.1 A resolution. The pair
6.25 A / 87.5 A at 48 V is 300 W / 4200 W, so the round numbers were always the watts and
the amps were the derived quantity, free to move; 6.3 A is 302 W.

Every other quantity lands exactly: 48.0 V -> 480, 87.5 A -> 875, -87.5 A -> -875,
25 degC -> 250, 80 -> 800, 100 -> 1000, 180 -> 1800, 230 -> 2300. All well inside INT16.

**With that, the engine model is fully specified and nothing in it is assumed.**

`Throttle_Position_Report` (PCL) and `Fuel_Quantity` (GAS) are `INT8` and carry **raw
percent**, scale 1.0 - the throttle column above is literal.

**`AP_PMUCAN` cross-checked against `PMU CAN Comm Protocol.md`, 2026-10-29. Clean.**

The ICD warns twice that the PDF prints field *order* without byte offsets, and says not to
finalize a decoder on the printed left-to-right order alone. **Our decoder's offsets match
that order in all five frames**, and the 5.1.16 bench run read sane values from real
hardware, which is the capture validation the ICD asks for:

| Frame | ICD order | `handle_frame()` offsets | |
|---|---|---|---|
| BATSTS | BSTS, TEMP, IBAT, GAGE | 0:2, 2:4, 4:6, 6:8 | match |
| ENGSTS | TEMP1, TEMP2, RPM, GAS, PCL | 0:2, 2:4, 4:6, 6, 7 | match |
| AUX1STS | ICMD, ILD, VBUS, GCMD | 0:2, 2:4, 4:6, 6:8 | match |
| AUX2STS | HOURCNT, RSV, PSTS, PTEMP | 0:3, (3 skipped), 4:6, 6:8 | match |
| VERSTS | DATE, MN, MJ, RSV | 0:4, 4, 5, (6:8 ignored) | match |

Also confirmed: the ICD's instruction 2 - *"frames with an invalid DLC must not overwrite
the latest valid values or valid reception timestamps"* - is already satisfied, because the
D6 short-frame check returns before `_pmu_frames_seen` and `_last_pmu_frame_ms` are touched.

**Four things the ICD records that are not otherwise written down anywhere in this tree:**

- **The PMU never transmits on its own.** Status is strictly RTR-request / Data-response,
  which is exactly what `_rtr_id[]` and the five-slot rotation do. **The fake PMU must be
  RTR-driven, not free-running** - and the silent trigger therefore means *stop answering
  RTRs*, which is a truer failure mode than a dropped broadcast.
- **`HOURCNT` is in SECONDS.** The ICD says so explicitly - *"despite its name"*. Our field
  is called `Engine_Hour_Count` and the MAVLink field `HOURCNT`, both of which read as
  hours. The FC only forwards it raw so there is **no FC bug**, but KGCS must divide by
  3600, and nothing in the FC should ever present this value as hours.
- **The ICD's bit rate of 500 kbps does not apply here.** The CubeMars motors have no option
  but 1 Mbit/s, so **the whole of CAN1 runs at 1 Mbit/s once the gimbal and the SAR are on
  it** and the PMU is reconfigured upward to match. Confirmed by PNU 2026-10-29 - the ICD
  figure is superseded, not overlooked.
- **`ICMD` width is inferred, and its type and scale are `TBD` in the ICD.** We decode it as
  INT16, which is the only width that fits, but the signedness and scale are assumed. It is
  a developer-debug field; the fake will echo the last BATCTRL into it and nothing should
  depend on its value.

`MJ` and `MN` are **packed BCD** - `0x12` is version 12, not 18. The FC forwards them raw,
so the conversion is KGCS's. The fake reports **MJ = 1, MN = 0** - version 1.0, which is
`0x01` and `0x00` and reads identically in BCD and binary, so the choice sidesteps the
encoding rather than exercising it. Nothing in the model depends on the PMU version.

**Step 2b AS BUILT - the fake PMU, 2026-10-29. `Tools/pnu/sitl_can/`.**

**No firmware file changed and no version was bumped.** Every previous CAN Update Plan
step carried a `version.h` bump because it changed the aircraft; this one does not change
the binary at all, so bumping `THISFIRMWARE` would have decoupled the version from what
the version describes. If PNU wants a tag for it, it should say "tools" rather than a
5.1.x firmware number.

| File | What it is | Lines |
|---|---|---|
| `pmu_model.py` | the engine, generator and battery - pure state machine, no I/O | 335 |
| `pmu_frames.py` | the five status frames - wire format only, no state | 99 |
| `fake_pmu.py` | SocketCAN I/O, the operator console, MAVLink takeoff detection | 304 |
| `test_pmu_model.py` | 17 tests - the RPM ladder, and every field at the offset `AP_PMUCAN` reads | |
| `test_fake_pmu.py` | 26 tests - RTR dispatch, command decode, the silent trigger, takeoff detection | |

The three-way split is deliberate: the model is testable without CAN, the frames are
testable without a socket, and **a second protocol - the gimbal - reuses neither, which is
the point**. `Tools/pnu/sitl_can/README.md` has the `vcan0` setup, the four SITL
parameters and the console reference.

**Raw `AF_CAN` sockets, not `python-can`**, so there is no dependency to install and the
RTR and extended-id handling stays explicit and auditable. `pymavlink` is imported lazily
and only for automatic takeoff detection; the console's `takeoff` replaces it.

**Confirmed while building it, by reading the SITL transport rather than assuming:**

- `AP_HAL_SITL` hard-codes the SocketCAN interface name as `vcan<instance>`
  (`CAN_SocketCAN.cpp:32`), so the first interface is always **`vcan0`** - it is not
  configurable, and the fake's default matches.
- **`SIM_CAN_TYPE1 = 2`** selects SocketCAN; the default is 1, multicast UDP.
- `AP_HAL::CANFrame`'s `FlagEFF`, `FlagRTR` and `FlagERR` are **bit-identical** to
  SocketCAN's `CAN_EFF_FLAG`, `CAN_RTR_FLAG` and `CAN_ERR_FLAG` (`CANIface.h:32-34`), and
  `CAN_SocketCAN::send()` copies `frame.id` through unmasked. **So RTR survives the SITL
  transport in both directions** - which the whole design depends on, and which was worth
  checking rather than hoping.

**VALIDATED IN SITL BY PNU, 2026-10-29 - THE WHOLE SEQUENCE FLEW.** `vcan0`, Mission
Planner on SERIAL0 and KGCS on SERIAL1. Correct: choke valve open and close, the choke
crank, Start, the warm-up idle, the cooling trip to ground idle, Stop, takeoff,
**RTL with landing as the final stage**, and **the silent trigger: `silent` while hovering
trips the D9 failsafe to RTL, `resume` reconnects and clears it**. The failsafe that until
now could only be tested by pulling a CAN connector is a one-word console command. The socket path, the CAN filter and the kernel's
RTR delivery are therefore all exercised, which the offline tests could not reach.

**The validated start-up order is in `Tools/pnu/sitl_can/README.md`** and it is not
optional: `vcan0` must exist before SITL starts, and **Mission Planner must attach to TCP
5760 before SERIAL1 and SERIAL2 exist at all**. `--no-mavproxy` is used because KGCS needs
a TCP endpoint; Mission Planner on its own does not need it and is happy on MAVProxy's UDP
14550.

**FIVE DEFECTS FOUND IN SITL, ALL IN THE TAKEOFF WATCHER, NONE IN THE MODEL OR ON CAN.
The second was introduced by the fix for the first**, which is worth recording as plainly
as the bugs themselves. **The CAN side has not needed a single fix.** Every one of the
five was in the FC-state bridge - the part that is not CAN, not the ICD, and not specified
by anybody, which is where the guessing was.

**(1) ILD, the head temperatures and PCL oscillated after takeoff.**

`recv_match()` accepts `HEARTBEAT` from **any source system**, and the armed flag was taken
from whichever heartbeat arrived last. With Mission Planner and KGCS also on the link - and
both send their own heartbeats, with no `SAFETY_ARMED` bit - `armed` alternated at the
heartbeat rate, so the airborne state alternated, so the engine state flapped
GROUND_IDLE <-> FLIGHT. ILD and PCL are instantaneous functions of state and snap on every
flip; the temperatures slew and so wobbled. **Invisible before takeoff**, because the
airborne state is false either way, which is exactly why it survived to flight.

Fixed by filtering to one system id, plus two hardenings found on the way: the altitude
fallback gained **hysteresis**, so a hover sitting on the threshold cannot chatter - a
real-aircraft hazard, not only a simulated one - and `LANDED_STATE_UNDEFINED` is treated
as no data rather than as "on the ground".

**(2) Then takeoff was not detected at all.** The fix above identified the vehicle as *the
first heartbeat that is not `MAV_TYPE_GCS` and not `MAV_AUTOPILOT_INVALID`*. That is a
sound filter and an **unsound identifier**: a ground station that does not label itself
`MAV_TYPE_GCS` captures the lock, and from then on no `GLOBAL_POSITION_INT` matches, the
altitude never updates, and nothing is ever airborne. KGCS is the likely candidate; the
point is that it should never have depended on what any GCS chooses to put in a heartbeat.

**The vehicle is now identified by its TELEMETRY.** `GLOBAL_POSITION_INT` and
`EXTENDED_SYS_STATE` are sent by vehicles and by nothing else, so whoever sends one is the
vehicle. Heartbeats are then read only from that system id - the filter from (1) is kept,
because a ground station sharing the vehicle's system id would otherwise still clear the
armed flag. `--sysid` pins it by hand if ever needed.

**(3) And it could not have worked at all as a passive listener.** Two facts about SITL
that neither the fake nor this file had recorded:

- **`udp:127.0.0.1:14550` is MAVProxy's output, not SITL's.** With `--no-mavproxy` nothing
  listens there at all. SITL's own endpoints are TCP **5760** (SERIAL0, `tcp:0:wait`, so
  SITL blocks until something connects), **5762** and **5763** (`SITL_State.h:33`).
- **Copter's default stream rates are ALL ZERO.** `GCS_MAVLink_Parameters.cpp:60-70` sets
  every `AP_MAV_DEFAULT_STREAM_RATE_*` to 0 for `APM_BUILD_COPTER_OR_HELI`. A listener
  that does not ask receives HEARTBEAT and nothing else - no position, no extended state,
  so no takeoff, ever, on any endpoint. MAVProxy and Mission Planner work because they
  request streams; the first SITL run only worked because MAVProxy was doing it.

**The watcher now requests its own streams** with `MAV_CMD_SET_MESSAGE_INTERVAL`, addressed
using the heartbeat and re-sent every 5 s until a vehicle has been identified. It no longer
depends on another ground station having asked for anything.

**Both facts were measured against a live SITL, not reasoned about**, by running a passive
listener and the watcher side by side on SERIAL2:

```text
A) PASSIVE listener, 8 s : HEARTBEAT 82, TIMESYNC 9, SYS_STATUS 1, 51005 x803
                           - no GLOBAL_POSITION_INT, no EXTENDED_SYS_STATE
B) WATCHER, 14 s         : asked system 1/1, identified vehicle 1 from
                           GLOBAL_POSITION_INT, landed_state=1 (ON_GROUND)
```

**(5) The engine dropped to ground idle WHILE DESCENDING UNDER RTL.** Reported by PNU
after a flight in SITL: changing to RTL took the engine to the 5000 rpm ground idle in the
air. `MAV_LANDED_STATE_LANDING` is reported all the way down, and the airborne test asked
whether the state was `IN_AIR` or `TAKEOFF` - which looks equivalent to "not on the
ground" and is not. **Only an explicit `ON_GROUND` now counts as landed**; anything else,
including any `landed_state` MAVLink adds later, keeps flight power. That is the safe
direction, and on a real aircraft it is the only acceptable one - an engine that drops to
idle during a descent is a far worse failure than one that holds flight power after
touchdown.

**(4) The watcher died instead of retrying.** `SERIAL1` and `SERIAL2` (TCP 5762 and 5763)
**do not exist until something connects to SERIAL0**, because SERIAL0 is `tcp:0:wait` and
SITL blocks at startup until a ground station attaches - measured: only 5760 is open
beforehand, 5762 and 5763 appear afterwards. Starting the fake first therefore got
`ECONNREFUSED`, `mavutil.mavlink_connection()` raised, and **the thread died with a
traceback and no other symptom** - takeoff detection silently off for the rest of the
session. The connection is now retried, and the whole watcher loop is wrapped so that this
thread can never again disappear quietly.

**And the console now answers this question itself.** `status` reports the MAVLink side -
the locked system id, armed, relative altitude, landed state and the heartbeat count - and
the fake warns once if no vehicle telemetry has arrived after 15 s. All three defects above
were diagnosed by reasoning from a symptom; there is now no need to do that again.

**WHAT STILL HAS NOT BEEN RUN:**

- the leaky-bucket health transitions, the **D9 failsafe** and the **silent trigger** -
  the tests step 2b exists for. The fake supports them; nobody has driven them yet
- **step 2c's interlock timing**, which is the measurement 2b was ordered before 2c to get
- the fake gimbal, and therefore the shared bus. **Both landed 2026-10-30**

**One deliberate limitation.** RPM follows the scripted ladder and is **not** derived from
`ENGPCL`, because the ICD gives no throttle-to-RPM relation. A manual throttle command is
recorded and echoed but moves nothing. Deriving a curve would make the fake agree with an
invention rather than with the PMU.

**Values marked `MODEL CHOICE` in the source are not protocol facts** - GAGE, the BCD
DATE, the landing transition, the stop ramp. Each is a plausible filler for something the
ICD leaves open and each is a one-line change.

---

**Step 2c - engine-start interlock guards - APPLIED 2026-10-29 in 5.1.18**

*Raised from the bench: KGCS lost its link at the moment the Start button was pressed, and
sent the queued data after reconnecting.*

**There is no timeout today, and that is deliberate - but it has a consequence nobody had
costed.** `engineoffmode()` only ever compares `_pmu_ctrl_cmd` against `_pmu_ctrl_cmd_prv`,
and that pair **only changes when a new TC1 arrives** - yet the function runs at 10 Hz
regardless. So when TC1 stops arriving, the pair **freezes in a valid alternating state and
`_engineoncnt` keeps counting**. PNU-ISSUE(D7) records this as the mechanism that makes the
interlock tolerate dropped messages. It also means:

| t | Event |
|---|---|
| 0.0 s | Start pressed; KGCS begins the 2/3 burst |
| ~0.2 s | second message arrives, pair valid, counter starts |
| ~0.2 s | **link drops** |
| **~1.2 s** | counter reaches 10 - **`ENGONOFF=1` goes out and the engine starts** |

Only about **2 of the 14 messages** were ever needed. The engine starts roughly a second
after the link died, with no further operator input.

**A guard on PNU-GCS cannot fix this.** It can stop a spurious re-trigger on reconnect, which
is worth having, but by the time the link drops the FC is already counting on its own and the
GCS is not there to stop it. The outage case is FC-side or nowhere.

**Two independent windows, both wanted by PNU 2026-10-19:**

| Guard | Window | Failure mode it covers |
|---|---|---|
| **TC1 staleness** | **500 ms** | the link dies *during* a burst - stop counting rather than completing the sequence alone |
| **Stop-before-start** | **10 s** | the link dies *between* Stop and Start - a late or queued Start after reconnect is not honoured |

The second one exists because **KGCS forces Stop before Start as a procedural safety guard**,
so the FC can require the same thing and refuse a Start that does not follow a recent Stop.

**The window is 10 s, not 5 - corrected by PNU 2026-10-19, and 5 would not have worked at
all.** KGCS **disables BOTH the Start and Stop buttons for about 5 s after either is
pressed**, so the operator physically cannot press Start until ~5 s after the Stop - a 5 s FC
window would have expired before the button became available again, and Start would have been
impossible. 10 s leaves roughly 5 s of margin for the lockout plus human reaction, and is
still short enough that a comms outage spanning more than one button cycle invalidates the
sequence. **Measure the window from the LAST Stop message seen**, not from the start of the
Stop burst.

**Design points, to be settled while implementing:**

- **Arm the 5 s window on a valid alternating 4/5 PAIR, not a single message.** Start is
  validated as a pair; a stray single Stop message should not arm permission to start.
- **Arm it in EITHER engine state.** Stop-while-off is the KGCS procedure; stop-while-on is a
  real shutdown. Both should arm the window.
- **A refused Start MUST say why.** At boot `_last_stop_seen_ms` is zero, so the very first
  Start is refused until a Stop has been seen - correct per the procedure, but an operator
  pressing a button that silently does nothing is the PNU-ISSUE(D6) problem again.
- **The staleness threshold is squeezed from both sides, and 500 ms is the midpoint.**  It
  must exceed the genuine inter-message gap and land before the counter reaches 10.  Worst
  case is the link dying as the counter starts from zero, which needs the full 1.0 s:

  | threshold | counter reaches before the reset | protects? |
  |---|---|---|
  | 300 ms | 3 of 10 | yes, large margin |
  | **500 ms** | **5 of 10** | **yes - chosen** |
  | 800 ms | 8 of 10 | marginal |
  | >=1000 ms | 10, so the start fires first | **no, useless** |

  **Raised from 300 ms to 500 ms by PNU 2026-10-19** to absorb LTE server and network delay -
  the link is a cellular modem, not a radio.  Do not go past ~700 ms.
- **Both windows are bounded from opposite directions, and neither number is free to move
  much.** 500 ms must exceed the burst interval and stay under the fire time; 10 s must
  exceed KGCS's button lockout and stay short enough to still be a guard.
- **This slightly tightens D7's documented loss tolerance**, which currently survives
  arbitrarily long dropouts. That is the point, and it should be recorded against D7 rather
  than left as a silent change.
- **Constants or parameters?** Both numbers are procedure-derived rather than tuning, so
  constants with the reasoning at the site are proposed - but if the bench disagrees they
  become parameters. UNDECIDED.
- The staleness timestamp comes from `transmit_slot()`, where a new TC1 is already detected
  by its sequence number. Sampling is 50 Hz, so ~20 ms of quantisation against a 500 ms
  window - irrelevant.

**A SEPARATE LTE SENSITIVITY, which no window size fixes.**  The FC samples TC1 at 50 Hz in
`transmit_slot()` - one `prv = cmd; cmd = latest` per sample, however many messages arrived.
PNU-ISSUE(D7) records the assumption behind that: *"TC1 at <=11 Hz against TXspin's 50 Hz
sampling means no alternation is missed at the sequence check."*  **That holds for a clean
link and breaks under bunching.**  If a cellular stall delivers four messages inside one
20 ms sample, the FC sees only the last, so `2,3,2,3` can read as `3` then `3` - not an
alternation - and the counter **resets to zero**.

Symptom: Start occasionally doing nothing on a poor link and needing a second press.  It
fails safe, so this is a usability sensitivity rather than a hazard.  **Measured 2026-10-19
at ~1 to 1.5 s, consistent with the expected ~1.2 s, so bunching is not severe today.**  If
Start ever becomes flaky over LTE, look here first - and the fix is on the INGEST side, by
latching alternations as they arrive in `handle_gcs_flcc_pmu_ctrl()` rather than sampling at
50 Hz.  Neither window helps.

**Both guards are engine-start logic, so neither goes in without PNU sign-off on the
numbers** - 500 ms and 10 s are PNU's, recorded here so the reasoning is not lost.

**Step 2c AS BUILT - 5.1.18, 2026-10-29. `AP_PMUCAN` only; nothing else is touched.**

| Added | Where |
|---|---|
| `note_new_tc1()` | stamps both windows, called from `transmit_slot()`'s new-command branch |
| `tc1_is_stale()` | the 500 ms test |
| `start_permitted()` | the 10 s test, and the refusal message |
| `count_start_pair()` | `_engineoncnt++`, stamping the first increment of an attempt |
| `PMUCAN_TC1_STALE_MS` 500, `PMUCAN_STOP_WINDOW_MS` 10000, `PMUCAN_START_REFUSED_WARN_MS` 3000 | `AP_PMUCAN.h`, with the reasoning at the site |

**Constants, not parameters** - the design left this UNDECIDED and proposed constants. Both
numbers come from the KGCS operating procedure rather than from tuning, and neither has
room to move much, so a parameter would mostly offer a way to break the interlock from a
ground station. If the bench disagrees they become parameters.

**BOTH WINDOWS ARE STAMPED ONLY WHEN A NEW TC1 ARRIVES, and that is load-bearing.** The
obvious place to test the Stop alternation is `engineonoffstate()`, which already inspects
the command pair - and it would be wrong. The pair is FROZEN between TC1 arrivals, so it
stays a valid 4/5 Stop alternation from the end of a Stop burst until the first message of
the next burst. Testing it at 10 Hz would refresh `_last_stop_pair_ms` continuously and the
10 s window would never expire. Same for staleness: the thing being timed is the ARRIVAL of
messages, so it can only be stamped where arrivals are detected.

**DEVIATION, deliberate: the Stop window is measured from the START PRESS, not from the
moment permission is checked.** The guard asks "did a Stop precede this Start by less than
10 s". The counter threshold of 10 is a tick timer at 10 Hz, so it consumes about 1.0 s of
the window on its own, and testing at the completion instant charges the operator for the
FC's own interlock delay on top of KGCS's ~5 s button lockout. `_start_pair_first_ms`
records when this attempt's first valid alternation arrived, and the staleness guard bounds
how stale that can be. The design note fixed the other end of the window - "measure from
the LAST Stop message seen" - and said nothing about this end, so this is a choice within
its latitude rather than a contradiction of it. **It does not weaken the guard**: a Start
queued during an outage arrives late, so its first alternation is late too.

**SIDE EFFECT WORTH KNOWING: the staleness guard is symmetric, so a STOP burst cut off
mid-way no longer completes either.** `engineonoffstate()` returns early while TC1 is
stale, which zeroes both counters, not just the Start one. An operator whose link dies
during a Stop press must press Stop again. On the ground that is an annoyance; in the air
an engine that declines to stop on a comms glitch is the safer of the two failures, which
is why this is recorded rather than special-cased. **If PNU wants Stop exempted, say so** -
it is one condition, but it reopens exactly the mechanism step 2c exists to close.

**A refused Start reports itself**, rate limited to 3 s because the check is reached at
10 Hz for as long as the burst keeps arriving:

```text
PMU: start refused, press Stop first              (no Stop seen this boot)
PMU: start refused, Stop was 14s before it (limit 10s)
```

**The first Start after any boot is refused until a Stop has been seen.** That is the KGCS
procedure working as specified, not a defect - but it is the behaviour most likely to be
reported as one, so the message exists to explain itself.

**Builds: CubeOrangePlus, SITL, and SITL with `AP_PNU_CAN_ENABLED=0`.**

**MEASURED IN SITL 2026-10-29 - both windows, and the defect reproduced and closed.**

`Tools/pnu/sitl_can/tc1_inject.py` sends the TC1 bursts KGCS would send, with the timing
under our control, and **watches `vcan0` itself** for the `ENGONOFF` the FC emits. One
process, one clock, so the interval needs no correlation between logs. The frame is packed
by hand - the PNU dialect is not in a stock pymavlink - and `--self-test` checks its
CRC_EXTRA and length against the firmware's own generated header, and the two windows
against `AP_PMUCAN.h`.

| Scenario | Expected | Observed |
|---|---|---|
| `start-only` - first Start of a boot, no Stop | refused | **refused**, `PMU: start refused, press Stop first` |
| `cut-start` - Start burst cut after 2 messages | no start | **no start** after 6 s of watching |
| `late-start` - Stop, wait 14 s, Start | refused | **refused**, `PMU: start refused, Stop 14s ago (max 10s)` |
| `stop-start` - the correct procedure | starts | **started, 1.50 s** after the burst began |

**Re-run independently by PNU 2026-10-29 on their own SITL - every scenario AS EXPECTED.**

**THE DEFECT WAS REPRODUCED, NOT JUST THE FIX.** With `PMUCAN_TC1_STALE_MS` temporarily
raised to 600000 to disable guard 1 and nothing else changed, the identical `cut-start`
scenario **started the engine 1.34 s after the last TC1 arrived**, with no further operator
input - the bench report of 2026-10-19, reproduced on a desk. Restoring the constant
returned it to no-start. That is the counterfactual the guard needed, and it is the
difference between "the code is present" and "the code fixes the bug".

**Interlock duration measured at 1.50-1.55 s** from the first message of the Start burst to
`ENGONOFF=1` on the wire. The bench measured ~1 to 1.5 s, so this agrees. It is longer than
the naive 1.0 s the threshold of 10 implies: ~0.1 s goes to the second message making the
first valid pair, up to 0.1 s to the ENGONOFF slot's phase in the five-slot rotation, and
the remainder is consistent with an occasional counter reset when a 10 Hz burst is sampled
by a 10 Hz loop and the same value is seen twice. **That last part is a hypothesis, not a
measurement** - it is the bunching sensitivity PNU-ISSUE(D7) already records, and it fails
safe. If Start ever becomes flaky, look there.

**Two defects found while measuring, both now fixed:**

- **The refusal message was 53 characters.** MAVLink STATUSTEXT caps at 50, so it arrived
  as two chunks and read as broken text on anything that does not reassemble them. Both
  messages are now under 50, and the reported age is clamped to keep it that way.
- **The fake PMU exited 0.01 s after starting whenever stdin was not a terminal.** EOF on
  a redirected stdin was treated as the console's "quit". Invisible when run by hand, fatal
  for any scripted or background run - including this measurement. EOF now means "there is
  no console here" unless stdin is a TTY, and SIGTERM quits.
- **The injector died instead of waiting when its endpoint did not exist yet** - found by
  PNU on the first run, and the same mistake as the takeoff watcher's in step 2b. SERIAL1
  and SERIAL2 do not exist until SERIAL0 has a client, so a tool started before Mission
  Planner got ECONNREFUSED and gave up. It now waits, bounded by `--link-timeout`, and says
  what to do. `-A "--serial0 tcp:0"` on `sim_vehicle.py` removes the dependency entirely -
  **verified: all three ports open with no ground station attached**, which is what makes
  the measurement scriptable.

**What this does NOT cover.** A real PMU, a real engine, and a real KGCS: the injector
sends what the KGCS protocol specifies, not what KGCS actually emits. The LTE link is not
simulated, so the bunching above is not exercised. **The numbers are confirmed to behave as
designed; they are not confirmed to be the right numbers for the aircraft.**

---

## SAR Integration Plan - PLANNED 2026-10-30, STEPS 3a, 3b AND 3c DONE

*`SAR_ICD_PNU.md` (repository root) is Rev1.0 of the SAR ICD, supplied by PNU 2026-10-30
and jointly owned with Lumir. It replaces the placeholder assumptions step 3 was written
against. **Read it before touching any of this** - it is unusually precise about byte
order, sentinels and what is still TBD, and it explicitly forbids inventing the TBDs.*

**Step 3 is promoted ahead of the fake gimbal, by PNU 2026-10-30**, against a real
schedule for connection testing with Lumir's device. The gimbal is a stabiliser; the SAR
provides the mission function.

### What the ICD got right before we asked

| Item | ICD | What we had already reserved |
|---|---|---|
| ID range | `0x400`-`0x7FF`, 11-bit standard | the same, reserved in step 1 before the ICD existed |
| Bitrate | 1 Mbit/s | the same, forced by the CubeMars motors |
| MAVLink ids | 50050 TC, 51050 TM | free in `pnu_kal.xml` (50001/2/4, 51001/2/3/5 in use) |

So no collision with the gimbal's `0x000`-`0x2FF` or the PMU's 29-bit extended ids, and
`CAN_Dn_PNU_OPTS` bit 2 already exists for the SAR.

### It is two bodies of work, not one

**A - the SAR interface.** 16 CAN ids, a nine-frame 10 Hz snapshot bundle, two MAVLink
messages, a five-state machine with 3 s and 2 s timers, the RTK-Fixed start interlock and
automatic termination. This is step 3.

**B - gimbal semantics for the SAR**, ICD section 16: a per-axis 25 deg stabilisation
envelope, yaw held to the **waypoint-segment heading** within 15 deg of error, and actual
attitude feedback with mounting transforms. The ICD assigns it to PNU Drone, but it is
`AP_PNUGimbal` work needing mission access and real gimbal feedback. **It is not step 3**
and is not costed here beyond the phase that carries it.

### The dependency that shapes the order

`0x404 ATTITUDE` must carry the **actual** attitude of the gimbal-stabilised SAR. Section
16.4 forbids substituting setpoints and marking them valid. Two consequences:

- Section 14.7 makes **ATT source age over 40 ms an automatic observation termination**,
  so with no attitude source every Start would abort at once.
- 40 ms freshness needs at least 25 Hz of gimbal feedback, and **the CubeMars feedback
  rate is unknown** - `AP_PNUGimbal_CubeMars.h` lists it among the things the manual does
  not specify and that we refused to invent. Homing is also still untuned, `PNU_HOME=0`.

**PNU's decision 2026-10-30: `0x404` carries no real Roll/Pitch/Yaw until the gimbal can
supply it.** Recorded with its cost in "The one real risk" below.

**Refined when 3e was written, and the refinement matters on the wire.** "Zeros" was the
original wording; ICD 16.4 asks for **sentinels** when actual attitude is unavailable -
`INT16_MIN`, `INT16_MIN`, `0xFFFF` with `ATT_Flags` empty - because zeros read as a level,
north-facing SAR to anything that checks values before flags. PNU chose the sentinels.
**Lumir must be told in writing** either way, because a receiver has to know that `0x404`
is unpopulated rather than reporting a perfectly stabilised payload.

### Phases

| Phase | Content | Files | Depends on |
|---|---|---|---|
| **3a** | **DONE in 5.1.19.** `AP_PNUSAR` becomes an `AP_PNU_CAN_Backend`; **protocol 17 retires**, deferred from step 2; `PNU_OPTS` bit 2 wires it up. `0x510` Heart_Beat, Live_Counter, the 3 s timeout, `SAR_STATUS` 0 and 1 | `AP_PNUSAR/`, `AP_CAN.h`, `AP_PNU_CAN.cpp`, `AP_CANManager/`, `AP_Arming.cpp` | nothing |
| **3b** | **DONE 2026-10-30, tools only, no version bump.** **Fake SAR** on `vcan0`, same shape as the fake PMU. Heartbeat first, then grown phase by phase. **Carried the first shared-bus run there has ever been** - fake PMU and fake SAR together at `PNU_OPTS=5` | `Tools/pnu/sitl_can/` | 3a |
| **3c** | **DONE in 5.1.20.** MAVLink **50050** TC and **51050** TM in `pnu_kal.xml`; TM at 5 Hz unconditionally; TC ingest with a source check. The `SAR connected` / `SAR reconnected` STATUSTEXT already landed in 3a | `pnu_kal.xml`, `GCS_PNU.cpp`, `GCS.h`, `GCS_Common.cpp`, `ap_message.h`, `Copter.cpp`, `UserCode.cpp` | 3a |
| **3d** | **DONE in 5.1.21.** Commands `0x501`/`0x502`, responses `0x521`/`0x522`, the 2 s FC window, states 2/3/4, the **RTK-Fixed start interlock** behind the new `CAN_Dn_PNU_SCHK`, and `SAR_Ctrl_Seq` finally consumed | `AP_PNUSAR/`, `AP_PNU_CAN/`, `GCS_PNU.cpp`, `Tools/pnu/sitl_can/` | 3c |
| **3e** | **DONE in 5.1.22.** The nine-frame **10 Hz navigation bundle** `0x400`-`0x408`, GPS freshness at 300 ms, ICD 14.7 automatic termination. **`0x404` is sentinels, not zeros** | `AP_PNUSAR/`, `AP_GPS.h`, `Copter.cpp`, `UserCode.cpp`, `Tools/pnu/sitl_can/` | 3d |
| **3f** | Real gimbal attitude into `0x404`, and the `ATT_FRESH` 40 ms contract. Also what makes `CAN_Dn_PNU_SCHK` bit 2 usable at last | `AP_PNUGimbal/`, `AP_PNUSAR/` | **measured gimbal feedback rate** |
| **3g** | ICD section 16: the 25 deg envelope and **yaw to waypoint-segment heading** | `AP_PNUGimbal/`, mission access | 3f |

The fake SAR lands at 3b on purpose. Step 2c is the precedent: the fake PMU is what turned
the interlock windows from reasoned into measured, and the same tool will do it for the
3 s heartbeat timeout, the 2 s response windows and the bundle ordering rules.

### Step 3b as built - MEASURED 2026-10-30

Three files in `Tools/pnu/sitl_can/`: `sar_frames.py` (the 16 ids and the four frames a
SAR transmits, wire format only), `fake_sar.py` (SocketCAN I/O and the console) and
`test_fake_sar.py` (21 tests). Procedure, console table and the full measured results are
in that directory's `README.md`.

**There is no `sar_model.py`.** The PMU needed one because an engine has behaviour; in 3b
the SAR beats once a second and answers commands, and the observation state machine
belongs to 3d, where it lands with the FC side that drives it. Inventing it now would
have meant inventing the parts the ICD leaves TBD.

Four decisions worth recording:

- **The fake answers `0x501`/`0x502` by default**, with `--no-respond` to stay quiet. A
  real device answers, and 3d's state 4 - the FC's 2 s window expiring - can only be
  tested by a device that does not. `--resp-delay` moves the answer inside or outside
  that window.
- **`--counter` starts `Raw_Counter` at 777000** in the validated run. ICD 3.3 says the
  FC's `Live_Counter` counts receptions and must never copy `Raw_Counter`; with the fake
  starting at zero the two would agree by accident and prove nothing.
- **`intrude` transmits on an FC -> SAR id.** `AP_PNUSAR` deliberately leaves those
  unclaimed, so the driver reports a second talker rather than swallowing it. The console
  command is how that is proved rather than argued.
- **The tests read both documents.** The ids and DLCs are checked against
  `SAR_ICD_PNU.md`'s own section 5 table and against `AP_PNUSAR.h`, because a fake that
  agrees only with itself proves nothing and the ICD is the document both sides signed.

**The shared bus, finally.** Until 3b every PNU CAN test had exactly one device on the
wire; the whole reason the three protocols became one driver had never been exercised.
With `CAN_D1_PNU_OPTS = 5` and both fakes on one `vcan0`:

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

**What it did not reach**, which matters as much as what it did:

- **`Live_Counter` is still unobservable.** It leaves the FC only through TM 51050, which
  is 3c. The run proves heartbeats were received and judged; it cannot yet show the count.
- **The command path was not driven from the FC**, because nothing on the FC side
  transmits before 3d. Responses are covered by unit tests only.
- **No SAR hardware.** The 1 Hz on the wire, and its jitter, are the fake's.
- The gimbal was still absent at 3b, so `PNU_OPTS = 7` remained untested until the fake
  gimbal landed on 2026-10-30 and ran all three at once.

### Step 3c as built - MEASURED 2026-10-30

The two messages come from `SAR_ICD_PNU.md` 3.4 verbatim, into `Forced Submodule File/pnu_kal.xml`
(the tracked dialect; `Tools/pnu/stage_mavlink_defs.sh` stages it, the submodule is never
touched). Two deliberate departures from the house style of the other seven messages, both
required by ICD section 3: **no `units="N/A"`** - the ICD says to omit the attribute rather
than set it - and the ICD's own wording for every description, so that the FC dialect and
Lumir's generated code are diff-able.

**Generated CRC_EXTRA: `231` for TC50, `7` for TM50.** Both endpoints have to agree, and the
ICD refuses to assign unverified values, so they are recorded here. Re-derived independently
from the field list and matched against the firmware's generated headers.

| Decision | Why |
|---|---|
| TM50 is its own 5 Hz `SCHED_TASK` | ICD 3.2 fixes 200 ms "regardless of connection or observation state". A stream-group entry would be user-settable and **Copter's defaults are all zero**; `userhook_MediumLoop` is 10 Hz |
| TM50 is **not** gated on a SAR backend | With none in the build, `SAR_Status` holds its static-init values and the TM reports Disconnected with a zero count - the truth in that configuration. This is what the D14 globals outside the enable guard were for |
| `Live_Counter` is not touched here | It counts **valid `0x510` receptions** (ICD 3.3) and belongs to `AP_PNUSAR`. Incrementing per TM would be five times too fast and would measure the sender |
| TC50 source check is `sysid_is_gcs()` | ICD 3.1: the message has no `target_system`, so the sender is the only thing to check. `MAV_GCS_SYSID` is the parameter ArduPilot already uses for RC override and GCS failsafe |
| the component id is **not** filtered | The ICD names no permitted value and ground stations differ. A filter here would be invented; it is logged instead |
| `Control` 3-255 refused, not clamped | ICD 3.1: "undefined and must not be executed" |
| `Observation_Mode` retained whole | All 256 values are legal (ICD 3.1) and its CAN encoding waits on `0x503`, which is TBD |

**OPERATIONAL, and new: KGCS must send TC50 from a system id `MAV_GCS_SYSID` covers.** The
default is 255. TC1 never checked its sender - an accepted residual of PNU-ISSUE(D7) - so
nothing before TC50 depended on that parameter being right. A refusal says
`SAR: TC50 sysid N not MAV_GCS_SYSID`, which names the parameter to fix.

Measured in SITL with the fake SAR on `vcan0`, `CAN_D1_PNU_OPTS = 5`:

| Step | Result |
|---|---|
| before any heartbeat | TM50 flowing, `status=0 Live_Counter=0` |
| six heartbeats | `status=1 Live_Counter=6`, while the device's `Raw_Counter` read **777005** |
| `short` (DLC 3) | `PNU_CAN: short frame 0x510 dlc 3`, and `Live_Counter` advanced by exactly 3 over the next 3 s - **the rejected frame did not count** |
| `silent` | `SAR disconnected` 3.07 s later, `status=0`, **counter held at 10** |
| `resume` | `SAR reconnected, inspection pending`, counter continued to 14, **state stayed 0** |
| TC50 `Control=0` Prepare | recorded, **no device command and no message** - ICD 8 |
| TC50 `Control=1 Mode=7` | `SAR: Start not implemented yet`; the log shows `MODE=7` retained |
| TC50 `Control=2` | `SAR: Stop not implemented yet` |
| TC50 `Control=7` | `SAR: TC50 Control 7 undefined` |
| TC50 from sysid 42 | `SAR: TC50 sysid 42 not MAV_GCS_SYSID`; the repeat 2 s later was logged but **not** announced again |
| TM50 rate | **5.01 Hz** over 36 s |
| dataflash | `SARC` has all six TC50s including both refused ones, with sender, component and channel; `SARS` has 181 records |

**Not covered:** no SAR hardware and no KGCS, so the GCS-side 3 s windows and the single
retry (ICD 14.6, PNU-GCS's work) are untested; and nothing yet consumes `SAR_Ctrl_Seq` -
that is step 3d.

*Verified:* CubeOrangePlus **1,734,516 B, 231,560 free** (+644 B over 5.1.19), SITL, and
**SITL with `AP_PNU_CAN_ENABLED=0`** - the last one is the point of the D14 globals: TM50
still compiles and still reports Disconnected when there is no CAN driver at all.

### Step 3d as built - MEASURED 2026-10-30

The command layer, in `AP_PNUSAR` plus one parameter on the driver and one guard in
`GCS_PNU.cpp`. `gcs().SAR_Ctrl_Seq` finally has a consumer: the GCS layer stores the
command and increments, the backend reads the increment once from the CAN thread and acts.

**The structural change is `publish_state()`.** ICD section 8 states a *priority* between
the sources of `SAR_STATUS`, not a sequence of assignments - disconnected or
recovery-pending (0) outranks the current attempt's timeout (4), which outranks the last
confirmed state (1/2/3). Step 3a could assign the state at each event because there were
two of them; with five states and two latches, last-writer-wins would quietly violate the
document, and "a heartbeat must not overwrite 2/3/4 with Ready" (14.1) is only one of the
orderings it would break. Computing the published value from the inputs makes the priority
true by construction. **Step 3a's single `_recovery_pending` flag had to split in two for
that**: the reconnection notification fires once, the inhibition stays latched until the
next power cycle. In 3a the state stayed Disconnected only because nothing wrote it again.

| Decision | Why |
|---|---|
| the FC never retries | ICD 14.5 prohibits it outright - an FC retry loop would exceed the GCS's two-attempt limit (14.6), which is the GCS's to own |
| a window opens only for a command **transmitted** | 14.5's own wording. A frame the interface refused never reached the device, so letting it expire into state 4 would blame the SAR for a fault on this side of the connector |
| a failed Start leaves nav OFF, a failed Stop **retains** it | 14.5. Stopping is unconfirmed, and the device may well still be transmitting; turning the gate off would make the FC's record disagree with the payload |
| an unmatched response is counted and discarded | The protocol has no transaction id and ICD 3.1 accepts that. Whether anything is waiting is the only thing that distinguishes a fresh response from a late or duplicated one |
| the timeout message is **not** the ICD's wording | "SAR not responding to Start" belongs to the GCS and means the whole two-attempt transaction failed (14.6). This is one FC window expiring, and the GCS may still hold a retry |
| a repeated Start is forwarded, not suppressed | 14.3: the device answers `0x521/01` again without toggling. The FC forwards the attempt; deciding it is redundant is not its job |
| `0x503` stays a named id and nothing else | ICD 13 asks for a stub only. With no defined serialization there is nothing to stub that would not be invented - Mode is not placed at B0 and eight zero bytes are not sent |
| TC50 with no SAR backend is reported | PNU-ISSUE(D6). Otherwise the GCS's 3 s window blames the payload for a `PNU_OPTS` bit |

**`CAN_Dn_PNU_SCHK`, and why it exists.** ICD 14.3.3 has the FC recheck Primary RTK Fixed,
GPS age, ATT age and navigation health before it may transmit `0x501`. A strict build can
therefore never send a Start on a bench, which is exactly where the Lumir integration test
happens. The bitmask names each check; **the default 11 is the ICD's set less the attitude
age**, because `0x404` carries hard-coded zeros until step 3f and a constant has no age -
ICD 16.4 forbids presenting a substitute as valid, so the bit is off and **setting it
refuses every Start**. Relaxation is never silent: once per boot, and again on every Start
that goes out under it. It is read live, so a bench operator does not reboot between tries.

Measured in SITL, fake SAR and fake PMU on one `vcan0`, `CAN_D1_PNU_OPTS = 5`:

| Drive | Result |
|---|---|
| boot, default `SCHK` | `SAR: ATT age check off, no source`; with `SCHK=0` at boot, also `SAR: Start checks relaxed, SCHK=0` |
| Start, ICD set, sim GPS healthy | allowed, `0x501` `dlc 1 01` on the wire, `0x521` back in **58 ms**, `SAR Start confirmed, state 3` |
| Stop while Started | `0x502` out, `0x522` back, `SAR Stop confirmed, state 2` |
| `SCHK=4` (attitude only) | `SAR: Start blocked, no ATT source`, **and nothing on the bus** |
| `SCHK=2`, 1.5 s after the GPS was cut | `SAR: Start blocked, GPS fix stale` |
| `SCHK=1`, once the driver dropped the fix | `SAR: Start blocked, no RTK Fixed` |
| `SCHK=11`, no GPS at all | `SAR: Start blocked, no RTK Fixed` |
| `SCHK=11`, GPS restored | allowed, and **no bypass line** - the permitted path, not only the refusals |
| `SCHK` changed by `PARAM_SET` mid-run | took effect on the next Start, **no reboot** |
| `respond off`, then Stop | `SAR: no Stop response, nav ON` at **2.03 s**; TM50 `status=3 -> status=4` at **+2.14 s** |
| the GCS retry | state **returned to 3** within 0.14 s and timed out to 4 again at +2.17 s - the latch cleared, `LAST_CONFIRMED_STATE` republished while waiting |
| unsolicited `0x521` with no window | `SAR: unmatched 0x521 (1)`, state unchanged |
| `0x522` carrying `0xFF` | `SAR: 0x522 payload 0xFF`, **the window stayed open** and expired honestly into state 4 |
| Start and Stop while disconnected | `SAR: Start blocked, disconnected` / `SAR: Stop blocked, disconnected`, nothing transmitted |
| Start after reconnect | `SAR: Start blocked, recovery pending` - ICD 14.8, and the TM stayed at 0 throughout |
| TM50 rate across it all | **5.00 Hz** over 45 s |

**SITL's simulated u-blox reports `GPS.Status = 6`** - RTK Fixed - for 206 of 207 dataflash
records in the first run. That is what made the *permitted* path testable rather than only
the refusals, and it is also why the first attempt at this measurement looked like a bug:
a Start that was expected to be blocked was correctly allowed.

**Not covered:** no SAR hardware and no KGCS, so the GCS-side 3 s windows and the single
retry (ICD 14.6) are still PNU-GCS's to test; `NAV_TX_ENABLED` moves but nothing reads it
until 3e; and ICD 14.7's **automatic termination** - the FC sending its own `0x502` on a
navigation fault - is deliberately absent, because its triggers are 3e's and a mechanism
with no trigger is code nobody has run.

*Verified:* CubeOrangePlus **1,735,784 B, 230,288 free** (+1,268 B over 5.1.20), SITL, and
**SITL with `AP_PNU_CAN_ENABLED=0`**. `Tools/pnu/sitl_can/test_fake_sar.py` is at 22 tests,
flake8 clean - the new one covers the deliberately wrong `0xFF` response payload, because
a rule the fake is unwilling to break is a rule the FC never gets to refuse.

### Step 3e as built - MEASURED 2026-10-30

The nine-frame bundle in a new `AP_PNUSAR_Nav.cpp`, a 10 Hz Copter `SCHED_TASK`, two
accessors on `AP_GPS`, and the ICD 14.7 automatic terminations.

**The thread split is the design, not an implementation detail.** `nav_snapshot()` runs on
the **vehicle thread**, beside `AP_GPS::update()`, and encodes all nine frames before
anything else can touch the data; the CAN thread's `send_bundle()` only writes bytes out.
Reading twenty-odd GPS fields from the CAN thread could straddle a GPS update and build
one bundle out of two different samples, which is exactly what ICD 7.3's "same acquisition
time" forbids. Encoding where the data is read makes that true by construction rather than
by hoping the window is narrow.

| Decision | Why |
|---|---|
| `0x404` carries **sentinels**, not zeros | ICD 16.4 asks for sentinels when actual attitude is unavailable. Zeros read as a level, north-facing SAR to anything that checks values before flags - and plenty does |
| `ATT_Seq` advances per **snapshot**, not per transmission | A bundle lost before it reached the bus then leaves a visible gap at the receiver. A dense counter would hide exactly the event worth seeing |
| `CAN_Dn_PNU_SCHK` gates 14.7 as well as 14.3.3 | A bench allowed to Start without RTK would have had the observation torn down 100 ms later, which would make the parameter useless for the thing it exists for |
| the **navigation failsafe is not gated** | "SAR handling must not replace or delay the FC's own flight failsafe actions" (14.7). The other conditions describe data quality; this one is the vehicle saying it no longer knows where it is |
| an automatic Stop is an **internal result** | 14.7 and 14.8. State 2 would claim a confirmed normal Stop and state 4 would invite the GCS to retry a transaction it never started (14.6 warns about precisely that) |
| DGPS (4) does **not** terminate | 14.7 lists "ordinary 3D_FIX (3) or lower", and the ICD leaves in-observation DGPS explicitly TBD (section 10). Nothing is invented for it |
| a failed `send_frame` **abandons** the rest of the bundle | The receiver discards a bundle with a missing id anyway (7.3), so the remaining frames would only congest the bus on their way to being thrown away |
| the snapshot is **one slot**, not a queue | 7.4 discards unsent bundles rather than holding them. A queue would deliver stale data after a gap, which is worse than a gap |

**Two accessors on `AP_GPS`, and the second one matters more than the first.**
`get_rtcm_stats()` makes `0x407` carry real counters instead of the ICD-legal blank.
`have_gps_yaw_accuracy()` exists because `gps_yaw_deg()` **substitutes a flat 10 degrees**
when the receiver reports no heading accuracy - without it the FC would have transmitted
10.00 deg as `YAcc` and a receiver would have had every reason to believe it. ICD 6.2 asks
for that fallback to be distinguished by name.

**The ICD's own byte vectors are now a unit test.** Section 9.1 publishes exact expected
bytes for five of the nine frames; `libraries/AP_PNUSAR/tests/test_pnusar_nav.cpp` runs
them against the firmware's packer, not against a second implementation written for the
test. It also covers the document's `-1.235 -> -124` rounding rule, the `36000 -> 0` angle
rule, out-of-range becoming the sentinel rather than saturating, and NaN. **12 tests.**

Measured in SITL, fake SAR and fake PMU on one `vcan0`, `CAN_D1_PNU_OPTS = 5`:

| Drive | Result |
|---|---|
| Start, `SCHK` at its default | first bundle **5 ms** after `0x521`; the fake assembled it under ICD 7.3's boundary rules |
| the decoded bundle | `Lat 35.2345000 Lng 129.0789000 Alt 10.10 m AEl 10.10 m`, `Status 6 RTK_FIXED NSats 10 I 0`, `GMS 459864600 GWk 2442`, `SMS/Delta 200 ms`, `GPS_Flags 0x6F` |
| `0x404` on the wire | `00 80 00 80 FF FF nn 00` - Roll/Pitch `INT16_MIN`, Yaw `0xFFFF`, `ATT_Flags` empty, as decided |
| `GPS_Yaw` and `YAcc` | both **n/a**: the sim u-blox has no heading, and the 10 deg fallback was correctly suppressed |
| rate over 12 s | **9.99 Hz**, 119 bundles, `ATT_Seq` 1:1 with the bundles received - **no gaps** |
| `Delta` | **200 ms** - the sim GPS is 5 Hz and consecutive bundles reuse the sample, which is the case ICD 7.2 calls out |
| normal Stop | bundle count frozen at 140 across the next 3 s |
| GPS cut, `SCHK=11` | `SAR: observation aborted, GPS age` **294 ms** later, then `SAR: auto Stop confirmed (1)` |
| GPS cut, `SCHK=9` (age check off) | `SAR: observation aborted, fix degraded` 3.88 s later, when AP_GPS dropped the fix |
| the same abort with the device mute | `SAR: auto Stop unanswered (1)` exactly **2.02 s** later, and **TM50 stayed at 0** - not 4 |
| heartbeat loss mid-observation | `SAR disconnected` 3.03 s after the last beat, bundles stopped at 80 and **stayed at 80** across the reconnect; no `0x502` attempted |
| a Start after any abort | `SAR: Start blocked, recovery pending` - ICD 14.8, and the only way back is a power cycle |

**One defect found by measuring, which is the point of measuring.** The first run reported
`SAR: bundle stale, 1 of 21` and then went quiet, while `ATT_Seq` showed **7 of 118
bundles missing** over 12 s. Two things were wrong: the 50 ms staleness window was tighter
than the driver thread's 10 ms loop actually keeps in SITL, and **a snapshot overwritten
before the CAN thread reached it was not counted as a drop at all** - it would have shown
up only as an `ATT_Seq` gap at the far end, which is the silent failure PNU-ISSUE(D6)
exists to remove. The window is now one full bundle interval, still far inside the ICD's
300 ms source-age limit, and an overrun is counted and reported. **Loss after the fix:
zero.**

**Not covered:** no SAR hardware and no KGCS; `0x404` is sentinels until 3f, so a receiver
has never yet seen a valid attitude from this FC; the ICD 15 Float-interval bookkeeping is
post-processing and deliberately absent from the FC (15.4 says Float alone adds no
automatic Stop); and nothing exercises a Primary GPS change at a bundle boundary, because
SITL was run with one GPS.

*Verified:* CubeOrangePlus **1,737,892 B, 228,184 free** (+2,108 B over 5.1.21), SITL, and
**SITL with `AP_PNU_CAN_ENABLED=0`** - with no CAN driver the 10 Hz task still compiles and
does nothing, which is what the entry point outside the feature guard is for. 12 C++ codec
tests and 29 Python tests pass, flake8 clean.

### The GCS side is not ours

ICD section 14.6 - the 3 s per-attempt windows, the single retry, Start-button enabling -
is **PNU-GCS work and is already under way**. This plan is FC-side only. The FC implements
its own 2 s window and performs **no retry of its own**; section 14.5 prohibits one,
because an FC retry loop would exceed the GCS's two-attempt limit.

### The one codebase gap - CLOSED in 3e, by two accessors

`RTCMFU`/`RTCMFD` came from `AP_GPS::rtcm_stats`, which was **private**. The ICD's
`RTCM_STATS_AVAILABLE` bit exists for exactly that case and 3e could have shipped with the
bit clear and both counters zero, which ICD 6.5 defines as correct - but the FC *has* the
numbers, so **PNU's decision 2026-10-30 was to add the accessor** rather than transmit a
blank field for no reason.

A second one came with it, and it is the more interesting of the two.
`gps_yaw_deg()` **substitutes a flat 10 degrees** when the receiver reports no heading
accuracy, and ICD 6.2 asks for that fallback to be distinguished from a measured value.
Without `have_gps_yaw_accuracy()` the FC would have transmitted 10.00 deg as `YAcc` and a
receiver would have had every reason to believe it. It now sends the unavailable sentinel.

Both are `const`, four lines each, change no behaviour, and are a clean upstreamable commit
on their own. Everything else the ICD needs was already public: `get_undulation()` for
`AEl`, `last_message_time_ms()` and `last_message_delta_time_ms()` for `SMS` and `Delta`,
the accuracy and DOP getters, and `gps_yaw_deg()` for yaw.

### `0x503` and `0x523` stay reserved

Lumir expects to define them soon, which is why they are named rather than left as holes.
**Interfaces and names only; no transmission, no invented payload, no parsing** - ICD
sections 5 and 13 both say so explicitly. The eight bytes stay opaque.

### Bus loading is not a constraint

| Source | Frames/s | Share of 1 Mbit/s |
|---|---:|---:|
| SAR navigation, 9 frames at 10 Hz | 90 | ~1.2% |
| PMU, 5 RTR + 5 responses + 5 commands at 10 Hz | ~150 | ~2.2% |
| Gimbal, 3 axes command and feedback at 100 Hz | ~600 | ~7.8% |

About 11% in total. The nine-frame bundle takes roughly 1.2 ms of a 100 ms period, so the
ICD's "do not overlap or interleave navigation bundles" is comfortable. What does need
care is that **all nine frames are enqueued in one driver tick**: a full TX queue would
publish a partial bundle, which the receiver must discard. Count that, never paper over it.

### The one real risk

**Hard-coded zero attitude in `0x404` is what ICD section 16.4 forbids** - "do not
substitute zero/segment_heading setpoints and mark them valid". The hazard is not to the
FC. It is that **Lumir may validate their processing against attitude that is always
level, and not discover it until the gimbal is fitted.**

Three mitigations, and the third is the one that matters:

1. A parameter rather than a literal, so the placeholder can be switched off without a
   firmware build, and so its existence is visible in a parameter dump.
2. A boot-time `GCS_SEND_TEXT` warning whenever the placeholder is active, on the pattern
   of the `PNU_HOME=0` warning, so no flight starts without somebody being told.
3. **Lumir is told in writing that `0x404` is a placeholder for this phase.** That is a
   project action, not a code change, and no amount of firmware care substitutes for it.

---

**Supported GCS and gimbal combinations - decided 2026-10-14**

*PNU-GCS is replacing KGCS. KGCS is a closed project and **whitelists MAVLink message ids**,
discarding or erroring on anything it does not already know - which fixes the set of payloads
it can fly with.*

| Ground station | Gimbal | Interface | Status |
|---|---|---|---|
| **KGCS** | **Viewpro only** | TC2/TM2 (PNU-KAL ICD) | supported - the legacy path, must keep working unchanged |
| KGCS | Gremsy, or any MAVLink gimbal | - | **NOT SUPPORTED.** Do not retest this |
| **PNU-GCS** | any | TC2/TM2 (KGCS interface) | supported - the `AP_Q30` path; four functions are GCS-direct on a non-Viewpro |
| **PNU-GCS** | any | standard MAVLink2 | supported - `AP_Mount` / `AP_Camera` handlers; `AP_Q30` is not involved at all |
| Mission Planner, QGC | any | standard MAVLink2 | supported - see the dual-GCS compatibility strategy below |

**Why KGCS + Gremsy cannot work, so nobody spends a day rediscovering it.** Tried on the bench
2026-10-14: KGCS connects and keeps seeing the FC, then its data stream stops after a few
seconds. A Gremsy adds message ids KGCS has never had to parse - from ArduPilot
`GIMBAL_DEVICE_ATTITUDE_STATUS` (285), and from the payload itself everything MAVLink routing
forwards: `HEARTBEAT` from three components (26, 100, 154), `CAMERA_INFORMATION` (259),
`CAMERA_SETTINGS` (260), `GIMBAL_DEVICE_INFORMATION` (283), `GIMBAL_DEVICE_ATTITUDE_STATUS`
(285), `STATUSTEXT` (253), and on request 261, 262, 269, 271, 322, 324, 397, 132, 248.
**This is not a firmware defect and there is nothing to fix in the FC.**

**The msg 285 suppression is correctly keyed, because of this table.** `AP_Mount_Viewpro`
suppresses `GIMBAL_DEVICE_ATTITUDE_STATUS` (D8a) and nothing else does. Since KGCS only ever
flies a Viewpro, KGCS never sees 285; every other combination has a ground station that copes
with it. A *per-channel* suppression was considered when KGCS+Gremsy was still thought
supportable, and is **not needed** - the per-backend hook is right.

**Do NOT set `MAVn_OPTIONS` bit 1 (`NO_FORWARD`) on the VIO's link.** It was considered as a
way to stop payload traffic reaching KGCS, and would break PNU-GCS: a private channel does
still receive exactly-targeted messages, so commands would get through, but
**`PARAM_EXT_ACK` (324) carries no target fields** and is therefore broadcast - so it would be
blocked on the way back. Every palette and source change would succeed silently and look like
a failure. `PARAM_EXT_VALUE` (322) and `COMMAND_ACK` (77) do carry targets and would return
normally, which makes the failure mode confusingly partial. Leave forwarding on.

**Naming.** The four one-shot notices say `CAM: <function> is GCS-direct on this mount` - not
"KGCS-direct", because the ground station driving them is PNU-GCS. In the code, `GCS_DIRECT_*`
and `report_gcs_direct()` follow the same rule. References to the **KGCS interface** mean the
TC2/TM2 ICD, which both ground stations speak; references to **KGCS** mean the product.

**What PNU-GCS already has working.** Its standard-MAVLink2 mode needs nothing new: the
Mission Planner bench session on 2026-10-13 - slew, Set Neutral, point-down and zoom against a
Gremsy - exercises the identical handlers, and the dual-GCS compatibility strategy below exists
to keep that path clean. Its KGCS-interface mode uses `AP_Q30` exactly as KGCS does, with the
four GCS-direct functions on a non-Viewpro payload.

**Per-channel message suppression - it exists, and it is narrower than it looks.**

A ground station can stop ArduPilot sending any single message on **its own link**, with one
standard command and **no firmware change**:

```
MAV_CMD_SET_MESSAGE_INTERVAL (511)
  param1 = <message id>     e.g. 285 for GIMBAL_DEVICE_ATTITUDE_STATUS
  param2 = -1               "stop sending"
```

`GCS_Common.cpp:3227` treats -1 as stop and clears that message's scheduling interval.
`set_message_interval()` is a `GCS_MAVLINK` **member**, so it applies only to the channel the
command arrived on - genuinely per-channel. `mavlink_id_to_ap_message_id()` maps 285 at
`GCS_Common.cpp:1134`, guarded only by `HAL_MOUNT_ENABLED`, so 285 is addressable this way.

> **It stops ArduPilot's own messages only. It does NOT stop forwarded payload traffic.**
> `MAVLink_routing::forward()` calls `_mavlink_resend_uart()` directly and never consults
> stream intervals or the deferred-message buckets - routing is a separate path that copies
> bytes to the matching link. So on a Gremsy, msg 285 reaches the GCS from **two independent
> sources**, and this command stops only the first:
>
> | Source | Stopped by `SET_MESSAGE_INTERVAL(285,-1)`? |
> |---|---|
> | ArduPilot's `send_gimbal_device_attitude_status()` | **yes** |
> | the VIO's gimbal component broadcasting its own 285, forwarded by routing | **no** |
>
> Suppressing ArduPilot's copy while the payload's still arrives is the **worst** outcome: the
> message still appears, at an irregular rate, from a different component id.

Two further limits, both normal but easy to forget:

- **Not persistent.** The interval lives in RAM, so it must be re-sent after every reboot or
  reconnect. That belongs in the GCS's connection sequence, not in a one-time setup step.
- **A stream-rate parameter change undoes it.** `GCS_Param.cpp:160,202` call
  `initialise_message_intervals_for_stream()` whenever an `SRn_*` parameter changes, which
  re-applies the group rate to every message in that group - and 285 lives in `STREAM_EXTRA3`.

*To stop the payload's own copies* you would have to send `SET_MESSAGE_INTERVAL` to the gimbal
component (sysid 1, compid 154) and hope the VIO implements the standard command - **untested**,
and the PayloadSDK only documents 511 against the payload app with a vendor-repurposed
`param1`. The other route, `NO_FORWARD` on the VIO link, is ruled out above.

**Recommendation: do not suppress 285.** PNU-GCS handles it, the payload's copies arrive
regardless, and the only real decision is which source of gimbal attitude PNU-GCS reads - msg
285 (standard, and arriving anyway) or TM2 (the KGCS convention). The per-backend suppression
stays as it is for Viewpro/KGCS, which cannot send this command at all.

---

**Dual-GCS compatibility strategy - Mission Planner and KGCS on the same aircraft**

*Decided 2026-10-12. This is a standing constraint on all remaining D10/D11 work, not a
one-off note. Everything below was verified against the 4.7.1 tree.*

**The rule.** A standard MAVLink GCS (Mission Planner, QGC) must keep full control of the
gimbal and camera. KGCS gets as much of the TC2/TM2 function set as the payload allows.
Where the two cannot both be served by one mechanism, **the standard path wins** and the
KGCS-specific behaviour moves into the KGCS adapter (`AP_Q30`), never into a shared library.

**Why this already holds, with the mechanism in each case:**

| Mechanism | Why a standard GCS is unaffected |
|---|---|
| `AP_Q30` is passive | Its only entry point is `handle_gcs_flcc_cam_cmd()`, reached solely by msg 50002 (TC2). It registers no scheduler task, timer or thread; its constructor only sets `_singleton`. Standard `GIMBAL_MANAGER_*` and `MAV_CMD_SET_CAMERA_*` never enter it |
| msg 285 suppression is opt-in (D8a) | Only Viewpro returns true from `suppress_gimbal_device_attitude_status()`. Every other gimbal keeps standard `GIMBAL_DEVICE_ATTITUDE_STATUS`. V5.0.8 suppressed it for *all* gimbals - already corrected in step 5 |
| `AP_Q30_ENABLED` (D13) | The entire KGCS camera path can be compiled out while keeping the mount |
| Direct GCS-to-payload routing | The answer for every function with no FC path. Inherently compatible: no FC code is added, and MP can use the identical mechanism |
| D10 Stage 1 | Reporting only - return checks and honest zoom. No behaviour change on any path |
| TM2 / TM5 broadcast | `gcs().send_message()` reaches every channel, but MP ignores unknown msgids |

**The one known violation: pitch reversal - RESOLVED 2026-10-12.** `pitch_target_is_reversed()`
lived in `AP_Mount_Backend`, so it applied to **every** angle source, not just KGCS:

| Caller | Reaches `AP_Mount_Backend::set_angle_target()` |
|---|:--:|
| `AP_Q30.cpp:80` - KGCS TC2 | yes |
| `AP_Mount.cpp:407` - `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` | yes |
| `AP_Mount.cpp:485` - `GIMBAL_MANAGER_SET_ATTITUDE` | yes |
| `AP_Mount.cpp:530` - `GIMBAL_MANAGER_SET_PITCHYAW` | yes |
| `AP_Mission_Commands.cpp:336`, `mode_auto.cpp:2046` - AUTO missions | yes |

So on Viewpro, Mission Planner and AUTO missions got **inverted pitch**, and on a Gremsy it
would have been internally inconsistent as well, because the base class returns false while
`AP_Q30::send_cmd_speed()` negates at the call site regardless of backend:

| | KGCS | Mission Planner / AUTO |
|---|---|---|
| Before, Viewpro | image convention | **inverted** |
| Before, Gremsy | angle physical, rate image - **inconsistent** | correct |
| **After (any gimbal)** | image convention on all three paths | physical, standard |

*The fix direction followed from the rule, and it was the opposite of the obvious one:* do
**not** make Gremsy reverse pitch to match Viewpro - that would have broken MP on Gremsy.
The negation moved out of `AP_Mount_Backend::set_angle_target()` and into `AP_Q30`, beside
the rate path that already did it. The `pitch_target_is_reversed()` hook was removed
entirely - `AP_Mount_Backend` now carries no PNU-specific pitch logic.

**Applied 2026-10-12**, on PNU confirmation that nobody flies Viewpro from MP or in AUTO.
Five files: `AP_Mount_Backend.{h,cpp}` (hook and flip removed), `AP_Mount_Viewpro.h`
(override removed), `AP_Q30.cpp` (negates at the call site), `GCS_PNU.cpp` (TM2 comment).
Builds clean at 1,721,348 B, and with `HAL_MOUNT_ENABLED=0` at 1,653,164 B.

> **`MNT1_PITCH_MIN` / `MNT1_PITCH_MAX` must be changed on the aircraft** - the parameters
> are vehicle configuration, not source, so this commit does not and cannot do it.
> **MIN -45 -> -90, MAX 90 -> 45.** The physical range is unchanged; the numbers are negated
> and swapped because the constraint now sees the already-negated value. Flying with the old
> pair clips KGCS pitch to 45 deg down and permits 90 deg up, past the mechanical stop. See
> the pitch-reversal section above.

**Bench test order when the hardware is next available:** (1) Viewpro + KGCS with the new
parameters - pitch direction correct, full nadir and 45 deg up both reachable; (2) Viewpro +
Mission Planner - `GIMBAL_MANAGER_SET_PITCHYAW` down now moves the gimbal down, where it
previously moved up; (3) any AUTO mission with a gimbal pitch item, since this silently
changes its behaviour. **Not yet bench-tested - no hardware was connected.**

**Design rules for the remaining work:**

1. KGCS-specific convention belongs in `AP_Q30`. Never in `AP_Mount`, `AP_Camera` or their
   backends.
2. If a shared library genuinely must behave differently, gate it on an **opt-in backend
   capability** - the D8a `suppress_gimbal_device_attitude_status()` precedent - so every
   other vehicle and gimbal keeps stock behaviour.
3. A TC2 function with no FC path goes **KGCS to payload direct**, not into new FC code.
   This is the most compatible option available and costs nothing to maintain.
4. Check return values and report (D10 Stage 1) rather than diverging silently.
5. Anything new on the wire must be additive and ignorable by a standard GCS.

**Accepted, not fixed:** with both GCSs connected, gimbal commands are last-writer-wins.
That is inherent to MAVLink, not a defect. MAVLink's own arbitration is
`MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` (primary/secondary control); adopt it only if
simultaneous connection is a real operating case rather than a bench convenience.
