# Migration Break Point for Future Updates
- This branch will update Copter-4.7.1 to FLCC V5.0.8 step-by-step to find new commit break points for future updates.  
- IN THIS VERSION : Step 4 - PMU CAN stack/version.h  
   - Changed plan : step 4 is now combined with previous step 5, and all steps 5~9 shifted down one.  
   - More issues were found that should be dealt with before issuing 5.1.0 later.  
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
| 5 | Camera adapter | `AP_Q30/` ×2, `AP_SerialManager.cpp`, `wscript` (Q30 line only), `Copter.h` (include + `AP_Q30 q30`), `Copter.cpp` (mount rate 50&rarr;10) | 1, 2, 3 | 7 |
| 6 | PMU failsafe | `events.cpp`, `Copter.h` (failsafe bit + decl), `Copter.cpp` (10 Hz call) | 2, 4 | — |
| 7 | GCS handler bodies | `GCS_Common.cpp` | 1, 2, 3, 4, 5 | 8 |
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
| D1 | IR palette deferral: `_image_sensor` not re-checked at send time; send result ignored. Underlying question — does the gimbal really service only one C1 command per cycle? If not, the whole deferral can be deleted; if so, a general C1 queue may be needed (a rapid Colour-1-then-IR-full sequence is currently unguarded). | `AP_Mount_Viewpro.cpp` | 5, 7 | Bench-test the premise, then: delete / point-fix / C1 queue |
| D2 | All 7 PNU-KAL ICD messages are id ≥ 50001, so MAVLink **v2 only**. A link set to `SERIALn_PROTOCOL=1` (MAVLink1) silently carries no PNU-KAL telemetry or commands. | `Forced Submodule File/common.xml` | 7 | Assert v2 on the PNU-KAL link, or document as a setup constraint |
| D3 | The `modules/mavlink` dialect edits are invisible to git — a `submodule update` or fresh clone silently reverts them and the build fails at step 7 with ~21 unknown-type errors. | `Forced Submodule File/ReadMe.txt` | — | Write `Tools/pnu/apply_mavlink_dialect.sh` (idempotent re-apply) |
| D4 | `OFP_VER_MAIN/SUB/REV` in `GCS.h` and `FW_MAJOR/MINOR/PATCH` in `version.h` are two hand-maintained copies of the same version with nothing enforcing agreement. | `GCS.h`, `version.h` | 4 | **RESOLVED (step 4)** - `static_assert` in `Copter.cpp` (only TU seeing both; `AP_PMUCAN.cpp` cannot include the vehicle `version.h`) |
| D5 | 375 of `GCS_Common.cpp`'s 454 added lines are one block appended at EOF — the main recurring merge cost on every future ArduPilot bump. | `GCS_Common.cpp` | 7 | Extract to `libraries/GCS_MAVLink/GCS_PNU.cpp` |
| D6 | `AP_PMUCAN::handleFrame()` performs no DLC validation - each case `memcpy`s from fixed offsets up to `data[7]` whatever `can_rxframe.dlc` says. No out-of-bounds read (`data[]` is fixed 8 bytes), but a short or malformed PMU frame is parsed silently and produces stale values for battery current, RPM, fuel quantity etc. | `AP_PMUCAN.cpp` | — | Confirm expected DLC per PMU message ID in the ICD, then reject or pad short frames |
| D7 | **Engine ON/OFF interlock is satisfied by inactivity.** `engineonoffstate()` counts to 10 at ~10 Hz, but `_pmu_ctrl_cmd`/`_pmu_ctrl_cmd_prv` are sticky between TC1 messages. If KGCS sends TC1 only on operator action, two messages (e.g. 3 then 2) freeze a valid pair and the counter climbs unattended - **engine STARTS after ~1 s of silence**. If KGCS streams TC1 with a held value, `prv == cmd` resets every tick and the **engine can never be commanded OFF**. Also rate-dependent (pairs overwritten above ~10 Hz) and loss-sensitive (a dropped TC1 resets progress). | `AP_PMUCAN.cpp` | — | Bench-test the matrix below, then re-specify the interlock (edge-latched count, or an explicit hold-duration) |
| D8 | **Viewpro-specific gimbal workarounds are applied to every gimbal type.** (a) The early `return` in `AP_Mount_Backend::send_gimbal_device_attitude_status()` is not virtual and `AP_Mount::send_gimbal_device_attitude_status()` calls it for every instance, so **msg 285 is suppressed for all gimbals** - fine for KGCS (it uses TM2) but any other GCS loses gimbal attitude for a Gremsy/Siyi. (b) Step 5 drops the mount scheduler 50&rarr;10 Hz. Angle maths is safe (Viewpro and Gremsy both declare `NATIVE_ANGLES_AND_RATES_ONLY`, so `AP_MOUNT_UPDATE_DT` is never used), but `send_m_ahrs()` then feeds the gimbal 5x staler vehicle attitude for its own stabilisation loop, and the UART RX buffer must hold 100 ms of telemetry per read. | `AP_Mount_Backend.cpp`, `Copter.cpp` | — | Check both against a Gremsy / MAVLink2 camera; make 285 suppression an opt-in backend capability |
| D9 | **PMU failsafe: no-PMU case is correct, but it can only fire once and cannot be escaped.** Both "no PMU" paths correctly avoid a false trigger (`PMUCAN_Fail` stays 2 when the driver runs with no PMU; stays 0 from static init when `CAN_Dn_PROTOCOL` is not 15). Two gaps: (a) **`failsafe.pmucan` is set but never cleared** - unlike `failsafe.terrain`/`deadreckon` - so the PMU failsafe fires at most **once per power cycle**; after landing, disarming and re-arming there is silently no PMU protection. (b) Once the PMU has been seen and then lost, state is 1 (`COMMUNICATION_ERROR`) with no path back to 2 and no disable parameter, so **emergency takeoff after a PMU dropout is blocked** - arming leads to disarm within ~100 ms, with no prearm warning (PMUCAN has no prearm check). | `events.cpp`, `AP_PMUCAN.cpp` | 6 | Clear `failsafe.pmucan` on recovery/disarm; decide whether an emergency override parameter is needed |

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

| Path | Writer | Reversed |
|---|---|:-:|
| MAVLink angle commands (`DO_MOUNT_CONTROL`, `GIMBAL_MANAGER_*`, AUTO mission, `AP_Q30`) | `set_angle_target()` | **yes** - this is what KGCS drives |
| RC stick control | `update_mnt_target_from_rc_target()` | no - physical convention |
| ROI / point-at-location | `get_angle_target_to_location()` | no - physical convention |
| Rate control | `update_angle_target_from_rate()` | no - physical convention |

The limits (`MNT1_PITCH_MIN` / `MNT1_PITCH_MAX`) are applied in ArduPilot
convention *before* the sign flip, so they keep their documented meaning: set
them to the gimbal's true mechanical travel, negative = down.

Any other gimbal (Gremsy etc.) inherits `false` from the base class and is
unaffected.


# ArduPilot Project

[![Discord](https://img.shields.io/discord/674039678562861068.svg)](https://ardupilot.org/discord)

[![Test Copter](https://github.com/ArduPilot/ardupilot/workflows/test%20copter/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_copter.yml) [![Test Plane](https://github.com/ArduPilot/ardupilot/workflows/test%20plane/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_plane.yml) [![Test Rover](https://github.com/ArduPilot/ardupilot/workflows/test%20rover/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_rover.yml) [![Test Sub](https://github.com/ArduPilot/ardupilot/workflows/test%20sub/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_sub.yml) [![Test Tracker](https://github.com/ArduPilot/ardupilot/workflows/test%20tracker/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_tracker.yml)

[![Test AP_Periph](https://github.com/ArduPilot/ardupilot/workflows/test%20ap_periph/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_periph.yml) [![Test Chibios](https://github.com/ArduPilot/ardupilot/workflows/test%20chibios/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_chibios.yml) [![Test Linux SBC](https://github.com/ArduPilot/ardupilot/workflows/test%20Linux%20SBC/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_linux_sbc.yml) [![Test Replay](https://github.com/ArduPilot/ardupilot/workflows/test%20replay/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_replay.yml)

[![Test Unit Tests](https://github.com/ArduPilot/ardupilot/workflows/test%20unit%20tests%20and%20sitl%20building/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_unit_tests.yml)[![test size](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml)

[![Test Environment Setup](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml)

[![Cygwin Build](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml) [![Macos Build](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml)

[![Coverity Scan Build Status](https://scan.coverity.com/projects/5331/badge.svg)](https://scan.coverity.com/projects/ardupilot-ardupilot)

[![Test Coverage](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml)

[![Autotest Status](https://autotest.ardupilot.org/autotest-badge.svg)](https://autotest.ardupilot.org/)

[![OpenSSF Best Practices](https://www.bestpractices.dev/projects/10598/badge)](https://www.bestpractices.dev/projects/10598)

ArduPilot is the most advanced, full-featured, and reliable open source autopilot software available.
It has been under development since 2010 by a diverse team of professional engineers, computer scientists, and community contributors.
Our autopilot software is capable of controlling almost any vehicle system imaginable, from conventional airplanes, quad planes, multi-rotors, and helicopters to rovers, boats, balance bots, and even submarines.
It is continually being expanded to provide support for new emerging vehicle types.

## The ArduPilot project is made up of

- ArduCopter: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter), [wiki](https://ardupilot.org/copter/index.html)

- ArduPlane: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane), [wiki](https://ardupilot.org/plane/index.html)

- Rover: [code](https://github.com/ArduPilot/ardupilot/tree/master/Rover), [wiki](https://ardupilot.org/rover/index.html)

- ArduSub : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub), [wiki](http://ardusub.com/)

- Antenna Tracker : [code](https://github.com/ArduPilot/ardupilot/tree/master/AntennaTracker), [wiki](https://ardupilot.org/antennatracker/index.html)

## User Support & Discussion Forums

- Support Forum: <https://discuss.ardupilot.org/>

- Community Site: <https://ardupilot.org>

## Developer Information

- Github repository: <https://github.com/ArduPilot/ardupilot>

- Main developer wiki: <https://ardupilot.org/dev/>

- Developer discussion: <https://discuss.ardupilot.org>

- Developer chat: <https://discord.com/channels/ardupilot>

## Top Contributors

- [Flight code contributors](https://github.com/ArduPilot/ardupilot/graphs/contributors)
- [Wiki contributors](https://github.com/ArduPilot/ardupilot_wiki/graphs/contributors)
- [Most active support forum users](https://discuss.ardupilot.org/u?order=post_count&period=quarterly)
- [Partners who contribute financially](https://ardupilot.org/about/Partners)

## How To Get Involved

- The ArduPilot project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](https://ardupilot.org/dev/docs/contributing.html)

- We have an active group of Beta Testers to help us improve our code: [release procedures](https://ardupilot.org/dev/docs/release-procedures.html)

- Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/ArduPilot/ardupilot/issues).

- Help other users with log analysis in the [support forums](https://discuss.ardupilot.org/)

- Improve the wiki and chat with other [wiki editors on Discord #documentation](https://discord.com/channels/ardupilot)

- Contact the developers on one of the [communication channels](https://ardupilot.org/copter/docs/common-contact-us.html)

## License

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

## Maintainers

ArduPilot is comprised of several parts, vehicles and boards. The list below
contains the people that regularly contribute to the project and are responsible
for reviewing patches on their specific area.

- [Andrew Tridgell](https://github.com/tridge):
  - ***Vehicle***: Plane, AntennaTracker
  - ***Board***: Pixhawk, Pixhawk2, PixRacer
- [Francisco Ferreira](https://github.com/oxinarf):
  - ***Bug Master***
- [Grant Morphett](https://github.com/gmorph):
  - ***Vehicle***: Rover
- [Willian Galvani](https://github.com/williangalvani):
  - ***Vehicle***: Sub
  - ***Board***: Navigator
- [Michael du Breuil](https://github.com/WickedShell):
  - ***Subsystem***: Batteries
  - ***Subsystem***: GPS
  - ***Subsystem***: Scripting
- [Peter Barker](https://github.com/peterbarker):
  - ***Subsystem***: DataFlash, Tools
- [Randy Mackay](https://github.com/rmackay9):
  - ***Vehicle***: Copter, Rover, AntennaTracker
- [Siddharth Purohit](https://github.com/bugobliterator):
  - ***Subsystem***: CAN, Compass
  - ***Board***: Cube*
- [Tom Pittenger](https://github.com/magicrub):
  - ***Vehicle***: Plane
- [Bill Geyer](https://github.com/bnsgeyer):
  - ***Vehicle***: TradHeli
- [Emile Castelnuovo](https://github.com/emilecastelnuovo):
  - ***Board***: VRBrain
- [Georgii Staroselskii](https://github.com/staroselskii):
  - ***Board***: NavIO
- [Gustavo José de Sousa](https://github.com/guludo):
  - ***Subsystem***: Build system
- [Julien Beraud](https://github.com/jberaud):
  - ***Board***: Bebop & Bebop 2
- [Leonard Hall](https://github.com/lthall):
  - ***Subsystem***: Copter attitude control and navigation
- [Matt Lawrence](https://github.com/Pedals2Paddles):
  - ***Vehicle***: 3DR Solo & Solo based vehicles
- [Matthias Badaire](https://github.com/badzz):
  - ***Subsystem***: FRSky
- [Mirko Denecke](https://github.com/mirkix):
  - ***Board***: BBBmini, BeagleBone Blue, PocketPilot
- [Paul Riseborough](https://github.com/priseborough):
  - ***Subsystem***: AP_NavEKF2
  - ***Subsystem***: AP_NavEKF3
- [Víctor Mayoral Vilches](https://github.com/vmayoral):
  - ***Board***: PXF, Erle-Brain 2, PXFmini
- [Amilcar Lucas](https://github.com/amilcarlucas):
  - ***Subsystem***: Marvelmind
- [Samuel Tabor](https://github.com/samuelctabor):
  - ***Subsystem***: Soaring/Gliding
- [Henry Wurzburg](https://github.com/Hwurzburg):
  - ***Subsystem***: OSD
  - ***Site***: Wiki
- [Peter Hall](https://github.com/IamPete1):
  - ***Vehicle***: Tailsitters
  - ***Vehicle***: Sailboat
  - ***Subsystem***: Scripting
- [Andy Piper](https://github.com/andyp1per):
  - ***Subsystem***: Crossfire
  - ***Subsystem***: ESC
  - ***Subsystem***: OSD
  - ***Subsystem***: SmartAudio
- [Alessandro Apostoli](https://github.com/yaapu):
  - ***Subsystem***: Telemetry
  - ***Subsystem***: OSD
- [Rishabh Singh](https://github.com/rishabsingh3003):
  - ***Subsystem***: Avoidance/Proximity
- [David Bussenschutt](https://github.com/davidbuzz):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
- [Charles Villard](https://github.com/Silvanosky):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
