#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.17"

// NOTE: 5.1.6 is a DEV build on top of the 5.1.0 OFFICIAL release, which closed the
// Copter-4.7.1 -> FLCC migration.  It closes D10 Stage 3: TM2's Zoom_POS_FB now carries
// real zoom on a Gremsy VIO.  Bench measurement showed the VIO reports
// CAMERA_SETTINGS.zoomLevel in zoom TIMES rather than the percentage the MAVLink spec
// defines - 1.00 at wide, 30.00 at full tele in super-resolution mode - which removed both
// the conversion and the int8_t overflow problems at once.
//
// Found while implementing it: CAMERA_SETTINGS was never routed to AP_Camera at all, so
// any backend decode was dead code.  Fixed with one case label in GCS_Common.cpp - the
// only core-file cost.  The decode, the unit-neutral get_reported_zoom_level() accessor
// and a send_camera_settings() override that forwards real values instead of upstream's
// NaN placeholders are all vendor-neutral and worth submitting to ArduPilot; only
// AP_Q30 decides the value means times, which is where PNU-specific facts belong.
//
// 5.1.17 moves the PMU failsafe out of a logging task.
//
//   failsafe_pmucan_check() was called from the tail of ten_hz_logging_loop(), and that
//   SCHED_TASK sits inside #if HAL_LOGGING_ENABLED (Copter.cpp).  SO A BUILD WITH LOGGING
//   COMPILED OUT HAD NO PMU FAILSAFE AT ALL - the check was simply never scheduled, with
//   no prearm warning, no message and nothing in any log to say the protection was gone.
//   AGENTS.md: a core, non-optional component must never depend on a compile-time
//   optional one.
//
//   HOW EXPOSED WAS IT, HONESTLY: not at all, today.  ArduCopter does not currently build
//   with HAL_LOGGING_ENABLED=0 whatever this file does - AC_CustomControl.cpp:190 calls
//   AP::logger() with no guard and the build stops there (verified 2026-10-29, and the
//   failure is upstream's, not ours).  So no shippable image could have lost the failsafe
//   this way.  The coupling was still wrong and the fix still belongs here, but this is a
//   latent structural defect that was made safe, not a bug that bit anyone.
//
//   It is now its own SCHED_TASK at priority 92, beside ekf_check, check_vibration and
//   gpsglitch_check - the other 10 Hz safety checks, none of which are logging-gated.
//   THE RATE IS UNCHANGED AT 10 Hz on purpose: PNU-ISSUE(D9)'s ~100-200 ms response was
//   measured against it, confirmed on the bench 2026-10-19 by pulling the CAN connector
//   and again in SITL 2026-10-29 with the fake PMU's silent trigger.
//
//   Found by reading the failsafe path while preparing the SITL silent-trigger test, not
//   by a failure - every build PNU has flown has logging enabled, so nothing was at risk.
//
// 5.1.16 fixes two defects found on the PMU bench 2026-10-19.
//
//   1. ARMING_SKIPCHK BIT 13 HAD NEVER WORKED, and PNU-ISSUE(D9b)'s design rested on it.
//      check_failed() only formats and reports - it never consults check_enabled() - so a
//      check has to wrap ITSELF the way AP_Arming.cpp:1134 and :1303 do.
//      AP_Arming_Copter::pmucan_checks() did not, so bit 13 did nothing and there was no
//      emergency-launch override at all: an aircraft with no PMU could not be armed except
//      by disabling every check.  This has been wrong since D9(b) landed in 5.0.x; nothing
//      in the step 2 conversion caused it.  Skipping leaves PMUCAN_Fail untouched, so arm()
//      still latches failsafe.pmucan_armed_healthy = false and the in-flight failsafe stays
//      out of the way - exactly the "launched without a PMU deliberately" case D9(b) wanted.
//
//   2. THE ONE-SHOT STARTUP REPORT WAS EMITTED BEFORE ANYONE WAS LISTENING.  It fired at a
//      flat 5 s from the CAN thread; KGCS had not connected by then, the statustext was
//      dropped, and being one-shot it never came back - so neither "PMU ready" nor the
//      protocol-16 deprecation warning was ever seen, and 15 and 16 looked identical on the
//      bench.  It now waits for GCS_MAVLINK::active_channel_mask() to be non-zero, which is
//      the condition that actually matters, with no upper bound: a GCS connecting ten
//      minutes in still gets the line, and if none ever connects there is nobody to tell.
//      A one-shot message nobody receives is the silence PNU-ISSUE(D6) exists to remove.
//
// Bench-validated in 5.1.15 and unchanged here: PMU telemetry downlink, KGCS and FC command
// paths, the prearm block when the PMU is absent, and the IN-FLIGHT FAILSAFE - pulling the
// CAN connector while armed took the set_mode_RTL_or_land_with_pause() branch immediately
// and recovered cleanly on reconnect.
//
// STILL UNTESTED, and it is the test step 2 exists for: the SHARED BUS.  Every bench run so
// far had CAN_Dn_PNU_OPTS = 1, so the id-based health rewrite - which stops another device's
// traffic masking a dead PMU - has never been exercised.
//
// 5.1.15 is CAN Update Plan step 2: the PMU becomes an AP_PNU_CAN backend.
//
// Protocol 15 now instantiates AP_PNU_CAN, with CAN_Dn_PNU_OPTS bit 0 creating AP_PMUCAN as
// one of its backends.  THE ENUM VALUE IS UNCHANGED, so no deployed aircraft needs a
// parameter change, and PNU_OPTS defaults to 1 so an upgrade is a no-op: PMU only, exactly
// as before.  16 survives as PNU_CAN_LEGACY, a deprecated alias that warns once per boot,
// so a bench aircraft configured during step 1 keeps working instead of silently getting no
// driver at all.  Delete the alias when the 5.1.x DEV series goes OFFICIAL.
//
// TWO THINGS THAT HAD TO CHANGE, not cosmetics:
//
//   1. THE 10 Hz IS NOW EXPLICIT.  It used to be an accident - TXspin() ran at 50 Hz and
//      rotated five commands, so each happened to go out at 10 Hz, and nothing in the source
//      said so.  PNU-ISSUE(D7)'s engine interlock is calibrated on it: the threshold of 10
//      ticks means ~1.0 s only because engineonoffstate() is reached once per rotation.  It
//      is now derived from PMUCAN_MSG_RATE_HZ and the driver's tick, which reproduces the
//      old cadence exactly - command slot n fires at tick % 10 == n*2 - while saying why.
//      Do not restructure the rotation without re-deriving the interlock.
//
//   2. HEALTH IS NOW KEYED ON PMU STATUS FRAMES.  RXspin() treated ANY successful receive as
//      proof the PMU was alive.  That was harmless while the PMU had the bus to itself, but
//      on the shared bus the gimbal's traffic would have held PMUCAN_Fail at CONNECTED with
//      the PMU dead, and failsafe_pmucan_check() would NEVER HAVE FIRED.  Only the five PMU
//      status ids feed the leaky bucket now; its thresholds and timing are unchanged.
//
// The directory layout is deliberately untouched.  AP_PMUCAN/AP_PMUCAN.h keeps its path and
// still declares PMU_Status and PMU_Ctrl_Echo beside their definitions (PNU-ISSUE D14), so
// ArduCopter/events.cpp, ArduCopter/AP_Arming_Copter.cpp and GCS_PNU.cpp need NO CHANGE -
// verified by diff.  Those two globals, and their definitions, sit outside the
// AP_PNU_CAN_ENABLED guard because those files reference them unconditionally.
//
// Deviation from the plan, deliberate: protocol 17 (PNUSAR) is NOT dropped here.  Removing
// it means AP_PNUSAR stops being an AP_CANDriver, which is step 3's job when it becomes a
// backend.  Bundling it into the risky PMU conversion would have mixed two unrelated changes.
//
// NOT BENCH-TESTED.  This must be run against a real PMU before it flies: engine Start/Stop
// interlock timing, failsafe trigger and recovery, and TM1/TM1x content.
//
// 5.1.14 is CAN Update Plan step 1d: HOMING, the Appendix 1 endpoint search.
//
// This is the most hazardous routine in the driver.  It deliberately drives a bulky SAR
// payload into three sets of mechanical hard stops, with the mechanical clamp DISABLED -
// before the zero is found that clamp is relative to wherever the mount powered up, so
// honouring it would prevent ever reaching an endpoint.  The only protection is the bounded
// stiffness and torque the homing commands carry.
//
// CAN_Dn_PNU_HOME THEREFORE DEFAULTS TO 0.  The stall threshold, approach velocity and the
// two gains are parameters with provisional defaults and are NOT yet bench-measured; the
// only value PNU has settled is the 1 second stall dwell.  Nothing drives into a stop until
// somebody turns this on deliberately.  With it off the driver says so once per boot -
// "PNU_HOME=0, zero not found - limits are relative" - and the zero is set by hand.
//
// WHY MIT MODE IS MANDATORY, and it is enforced: MIT is the only mode in which the host
// bounds the push into a stop.  A sweep sends kp=0, kd=PNU_HKD, v=+-PNU_HVEL, which is
// velocity-style impedance control, so at stall the torque settles at HKD*HVEL - a number
// the operator picks.  A position/velocity-mode command runs a cascaded loop whose torque
// cannot be limited from the CAN frame at all.  init() refuses to arm the sequence in any
// other mode, naming PNU_MODE in the message.
//
// ENDPOINT DETECTION IS BY ANGLE ONLY, per Appendix 1 item 4: overcurrent, undervoltage and
// overload are emergency stop conditions, NOT endpoint criteria.  An endpoint is "the angle
// stopped changing while motion was still commanded"; any fault status aborts the sequence.
//
// Sequence per axis, in the order roll, pitch, yaw - roll first because the mass offset is
// on that axis, yaw last because it is safer to swing the payload with pitch neutral: seek
// a stop, back off 2% of travel, seek again and require the two readings to agree within
// PNU_HRTOL; repeat the other way; validate the stop-to-stop travel against twice the
// mechanical limit within PNU_HTOL; move to the midpoint; confirm motion has stopped; send
// the zero; verify the feedback reads zero.  The idle axes are HELD at their power-up angle
// throughout, because the manual does not define what an enabled motor does with no command
// outstanding and a swinging payload is the collision the limits exist to prevent.
//
// TRAVEL VALIDATION IS THE ONLY RUNTIME CHECK that can catch a PNU_PMAX mismatch - the
// failure the protocol note warns about, which does not fail but silently rescales every
// command and every reading.  A factor-of-two travel reading is its signature and nothing
// else in the system would notice.
//
// Settled with PNU 2026-10-19 and in this version:
//   - TRAVEL VALIDATION IS ASYMMETRIC.  The lower bound is twice (mechanical limit minus
//     CAN_Dn_PNU_MARG), i.e. the mount must reach its own operating range - 50 degrees on a
//     60 degree axis - and below that it aborts as obstructed.  A measurement between the
//     operating and mechanical ranges is ACCEPTED and the MNT1_* operating limits stay at
//     their nominal values; they are NOT derived from the measurement, because stop
//     compliance under a heavy payload would make the mount's reach vary with how hard it
//     happened to press.  The upper bound stays mechanical + PNU_HTOL, since compliance only
//     ever reads smaller and an over-reading means a false endpoint or a PMAX mismatch.
//   - PER-AXIS ROTATION DIRECTION, CAN_Dn_PNU_DRLL / DPIT / DYAW.  The mount is still
//     undergoing mechanical design changes, so which way a motor turns for a positive
//     command is configuration rather than a source constant.  Everything inside
//     AP_PNUGimbal_Backend is stored in the MOTOR frame and the conversion happens in
//     exactly two places - set_target() in, get_feedback() out - so there is one pair of
//     boundaries to reason about instead of a sign scattered through the code.  Homing is
//     unaffected: it finds both stops and takes the midpoint, which is direction-agnostic.
//
// Two hazards found while writing it, both fixed: the VERIFY phase originally commanded
// position 0 before confirming the zero had been sent, which would have meant the OLD origin
// - possibly tens of degrees away - and lurched the payload across its travel; and HOMING
// was reachable from a second INIT, so clearing a fault IN FLIGHT would have re-run the
// endpoint search mid-air.  Homing is now once per POWER CYCLE (the zero lives in the drive
// and survives a disable/enable) and is refused while hal.util->get_soft_armed().
//
// 5.1.13 is CAN Update Plan step 1c: the gimbal lifecycle state machine.
//
// INIT -> ALIGN -> RUN, plus MAINTENANCE and FAULT.  HOMING is defined in the enum and
// deliberately NOT implemented - that is step 1d, and until it runs the Appendix 1 endpoint
// search the zero must be set by hand on the bench.
//
// The state machine lives in AP_PNUGimbal_Backend, one layer BELOW the mount: it runs in the
// CAN thread where feedback actually arrives, so a fault transition happens on the frame
// rather than a 50 Hz mount tick later; it survives HAL_MOUNT_ENABLED=0 (PNU-ISSUE D13); and
// a second motor protocol would need the same lifecycle unchanged.  That meant moving the
// per-axis target and feedback storage, the mechanical limits and the stale-feedback policy
// up out of AP_PNUGimbal_CubeMars, which now owns the wire format and nothing else.
//
// THE LIFECYCLE IS NOT THE MOUNT MODE.  MAV_MOUNT_MODE_* says where to point; the state says
// whether the hardware can be pointed.  RETRACT is a targeting mode, not "motors off".
//
// ALIGN exists because section 9 of the protocol note requires position control to start from
// the measured current angle.  It seeds the command from feedback so the first mount target
// is approached from where the mount actually is instead of arriving as a step from zero -
// with a heavy SAR payload that is the one thing in this design that could break hardware.
// The new CAN_Dn_PNU_SLEW rate limit then bounds the approach, and bounds every later move
// as well: one mechanism, two jobs, and cheaper than ramping stiffness.
//
// CAN_Dn_PNU_MAINT takes the FC completely off the bus so a PC can configure a motor over
// UART.  Entering is immediate; LEAVING REQUIRES A REBOOT, because the vendor utility can
// change the CAN id, the control mode and the encoding ranges, and a host/device mismatch
// does not fail - it silently rescales every command and every reading.  The state is
// latched at init() for exactly that reason.
//
// ARMING INTERLOCK: healthy() is now true only in RUN with every axis reporting fresh
// feedback, and AP_Mount::pre_arm_checks() fails on !healthy() as Check::CAMERA - so an
// unhomed or faulted mount cannot be armed.  The documented override is ARMING_SKIPCHK bit 16
// (Camera, decimal 65536), the same escape hatch PNU-ISSUE(D9b) established for the PMU.
//
// Also fixed here, and it predated this step: AP_PNUGIMBAL_ENABLED=0 did not compile, because
// AP_PNU_CAN sizes its parameter arrays from AP_PNUGIMBAL_NUM_AXES.  The axis identity moved
// out of the enable guard into AP_PNUGimbal_config.h - an aircraft with a PMU and a SAR but no
// gimbal is a real configuration, and AGENTS.md requires the base system to compile with
// optional features disabled.
//
// 5.1.12 is CAN Update Plan step 1b: AP_Mount_PNUGimbal, the mount backend.
//
// MNT1_TYPE = 15 selects a new AP_Mount backend for the PNU 3-axis gimbal.  AP_Mount already
// does FC-attitude stabilisation - AP_Mount_Servo::update_angle_outputs() is the whole of it,
// and the requires_stabilization flag is what separates a dumb servo mount from a
// self-stabilising brushless one.  The CubeMars drives are the "true" case.  AP_Mount also
// needs no camera: its only pure virtuals are has_pan_control() and
// get_attitude_quaternion(), and every camera method defaults to returning false.  So the
// backend inherits mount modes, RC targeting, the MAVLink gimbal manager, ROI, the
// MNT1_*_MIN/MAX limits and the lead filter, and this file adds only the stabilisation maths.
//
// IT COMMANDS NO MOTORS.  Targets are written all the way through to AP_PNUGimbal_CubeMars,
// but that backend transmits nothing until set_enable(true) and nothing in the tree calls
// that yet - the lifecycle state machine is step 1c.  So the whole path is exercised with the
// motors silent, and it is provable from a log: AP_Mount_Backend::write_log() already records
// the MNT message, whose "desired" columns are the earth-frame target and whose "actual"
// columns this backend fills with the body-frame demand.  The difference between them IS the
// AHRS correction.  No new log message was needed.
//
// LIMITS ARE SPLIT ACROSS TWO LAYERS on purpose.  MNT1_ROLL/PITCH/YAW_MIN/MAX carry the
// OPERATING limits (+-25 / +-25 / +-15 deg) and AP_Mount clamps to them; the new
// CAN_Dn_PNU_MRLL/MPIT/MYAW carry the MECHANICAL limits (+-30 / +-30 / +-20 deg) and the
// gimbal backend clamps again as the last thing before a frame is built.  A bug in the
// targeting layer therefore cannot drive a bulky payload into the landing skid struts.  The
// 5 degree margin absorbs stabilisation overshoot.
//
// NOTE that the mechanical clamp means nothing until homing has put zero at the centre of
// travel: the encoder is relative and its zero is assumed not to persist.  Until step 1d runs
// the Appendix 1 endpoint search, plus or minus 30 in encoder terms is not plus or minus 30
// mechanically.
//
// Found while implementing: ArduPlane has been unbuildable on this branch since ed055d2f24
// (5.0.10) - GCS_PNU.cpp references PMU_Ctrl_Echo and AP::ac_avoid(), both Copter-only.  A
// vehicle gate on the new backend was written and then REMOVED as unnecessary: it would have
// protected a build configuration this branch has never supported.  See README.
//
// 5.1.11 is CAN Update Plan step 1: AP_PNU_CAN, the multi-backend CAN driver.
//
// The PMU, the gimbal and the SAR share one 1 Mbit/s bus, and AP_CANManager binds each
// interface to exactly ONE driver - one interface to many drivers is not supported, and a
// driver with no interface is never allocated at all, so it fails silently.  CAN_Dn_PROTOCOL2
// does not help either: it is a DroneCAN-hosted auxiliary slot taking one CANSensor with no
// thread.  So three protocols on one wire means one driver with three backends.
//
// AP_PNU_CAN owns the interface, the thread, the parameters and the one place a frame reaches
// the bus.  AP_PNU_CAN_Backend owns a wire format and nothing else.  handle_frame() now returns
// bool and the driver stops at the first backend to claim a frame, the MultiCAN pattern
// upstream already uses for the CAN rangefinder; an unclaimed frame is counted and reported,
// which is how a mis-assigned CAN id gets noticed.  send_frame() takes the FULL 32-bit id
// including FlagEFF/FlagRTR, because the PMU polls with extended RTR frames.  update() is
// handed the driver's tick count so every backend derives its rate from one clock - the PMU's
// 10 Hz turned out to be an accident of an undocumented command rotation, and that must not
// happen twice.
//
// The name carries an underscore on purpose: PNU_CAN, never PNUCAN.  The two drivers sit
// beside each other wherever an operator looks - 15:PMUCAN and 16:PNU_CAN in one
// CAN_Dn_PROTOCOL dropdown, "PMUCAN:" and "PNU_CAN:" statustexts on one GCS screen - and M
// and N are one letter apart in a sans-serif font.  The underscore is the only thing telling
// them apart at a glance; do not tidy it away.
//
// Registered on AP_CAN::Protocol 16 (renamed PNUGimbal -> PNU_CAN).  THE PMU IS UNTOUCHED on
// protocol 15 and nothing that flies changes in this version; the PMU becomes a backend in
// step 2, which needs a bench against real hardware first.
//
// New CAN_Dn_PNU_* parameters replace 5.1.10's CAN_Dn_PG_*.  CAN_Dn_PNU_OPTS is a device
// bitmask - bit 0 PMU, bit 1 gimbal, bit 2 SAR, default 1 - and it exists because "no PMU"
// used to mean not setting CAN_Dn_PROTOCOL, which one shared driver takes away.  An aircraft
// with a gimbal and a SAR but no PMU is a configuration that will fly.  One flat group rather
// than a per-device subgroup: AP_SUBGROUPPTR resolves var_info from the class, so two sibling
// subgroups pointing at this one object would get the same table.  Parameter index 7 (the old
// PG_ group) is retired and NOT reused - AP_Param stores by index, not by name.
//
// CAN_Dn_PNU_TYPE drops 0:None and defaults to 1:CubeMars; bit 1 of PNU_OPTS now answers
// whether a gimbal is fitted.  Two switches for one thing is how a mount ends up silently
// disabled by the one nobody looked at.
//
// A one-per-boot line names the backends that are running, e.g. "PNU_CAN: CAN_D1 Gimbal ready,
// no PMU".  That is what makes a forgotten PNU_OPTS bit visible: with bit 0 clear,
// PMU_Ctrl_Echo.PMUCAN_Fail stays 0 and reads as healthy, so a missing PMU is otherwise
// completely silent.  Raising that global to 2 "for honesty" would instead fire a prearm
// warning on every arm of a PMU-less aircraft - the problem 5.1.8 removed.
//
// 5.1.10 gives AP_PNUGimbal a protocol backend.  The 5.1.9 skeleton was one class that
// would have held one protocol; it is now a frontend plus an AP_PNUGimbal_Backend, because
// the mount is expected to accommodate more than one motor type and only the protocol
// differs between them.  The frontend keeps the interface, the thread, the parameters, the
// axis mapping and the single place a frame reaches the bus; a backend owns the wire format
// and nothing else.  AP_PNUGimbal_CubeMars is the first, implementing the CubeMars GLII
// family protocol recorded in libraries/PNUGimbal_CAN_Protocol_Type_CubeMars.md: all three
// control modes (MIT, position/velocity, velocity), the four universal commands, and the
// common feedback frame.
//
// It is the PROTOCOL LAYER ONLY.  There is no stabilisation loop and this is not an
// AP_Mount backend - whether it becomes one is still the open question, and the README's
// rule that it be decided before the loop is written still stands.  set_target() exists and
// nothing in the tree calls it, so the driver transmits nothing until a control layer does.
//
// New parameters, CAN_Dn_PG_*.  CAN_Dn_PG_TYPE defaults to 0 (None), so CAN_Dn_PROTOCOL=16
// on its own still does nothing - selecting a CAN protocol must not start commanding
// motors.  The five encoding limits (PMAX/VMAX/KPMAX/KDMAX/TMAX) are parameters rather than
// constants because they MUST match what the vendor PC utility wrote to the motors: a
// host/device mismatch does not fail, it silently rescales every command and every reading.
//
// The codec is verified against the manual's own published command vectors - four MIT, two
// position/velocity, two velocity - in libraries/AP_PNUGimbal/tests/.  Two of the manual's
// MIT vectors are NOT reproduced and cannot be: its +-6 rad/s rows do not decode to +-6 at
// the +-200 range it states.  That divergence is pinned down by its own test so a bench
// disagreement points at the manual rather than at this code.
//
// 5.1.9 adds skeletons for two new CAN drivers: AP_PNUGimbal (a PNU-developed gimbal
// mount) and AP_PNUSAR (a SAR radar payload).  Both speak private CAN protocols, NOT
// DroneCAN, in the same shape as AP_PMUCAN - their own thread created in init(), nothing on
// the vehicle scheduler.  Registered as AP_CAN::Protocol 16 and 17, so CAN_Dn_PROTOCOL = 16
// or 17 selects them.
//
// They are STRUCTURE ONLY.  The ICDs are not yet available, so rx_spin(), tx_spin(),
// handle_frame() and send_frame() are stubs marked TODO; no parameters, no CAN_Dn_PROTOCOL
// @Values entry, and no vehicle-side consumers yet.  What the skeletons do carry is this
// project's own hard-won lessons: DLC validation before reading any field with a
// rate-limited short-frame report (D6), initialisers on every member (D1/D10), a note not
// to add write-only counters (D6), and a reminder to confirm endianness and say so at the
// site if no byte swap is needed (D15).
//
// Note that adding protocol enum values breaks exhaustive switches - -Werror=switch caught
// AP_Arming.cpp, which now lists both beside PMUCAN's existing no-op case.  Values 16 and 17
// could collide with future upstream protocol numbers, exactly as PMUCAN = 15 might; worth
// watching on the next ArduPilot bump.
//
// 5.1.8 is the backward-compatibility fix for KGCS.  Tracking, video source and IR palette
// had already left the FC for GCS-direct routing, but were still reported as failures -
// on a Gremsy they produced "CAM: mount rejected ..." every 10 s forever, because KGCS
// keeps sending TC2 and nothing on that payload can serve those commands.  Since
// report_cam_unsupported() shares one rate limiter, three permanent warnings would have
// crowded out genuine faults, destroying the signal D10 Stage 1 exists to provide.
// All four GCS-direct functions now announce themselves ONCE at MAV_SEVERITY_INFO and
// then stay silent.  A Viewpro that genuinely declines still warns: the gate is on the
// payload, not on the return value.
//
// Also recorded in 5.1.8: the supported GCS/gimbal combinations.  KGCS whitelists MAVLink
// message ids and is VIEWPRO-ONLY - a Gremsy adds ids it has never parsed, and its stream
// stops.  That is not a firmware defect and KGCS + Gremsy must not be retested.  PNU-GCS
// replaces it and works with any gimbal, over the KGCS interface or standard MAVLink2.
// Two consequences, both needing no change: the msg 285 suppression is correctly keyed per
// backend, and NO_FORWARD must NOT be set on the VIO link because PARAM_EXT_ACK carries no
// target fields and would be blocked on the way back.  The four one-shot notices are now
// "GCS-direct" rather than "KGCS-direct", since the station driving them is PNU-GCS.
//
// Also verified, with no code needed: GCS-to-VIO routing already works.  get_targets()
// reads the generated message-entry table, PARAM_EXT_SET and COMMAND_LONG both carry
// target fields and are in the staged dialect, and a message addressed to the camera's
// component falls through to forward().  All seven PNU-KAL messages are likewise
// unaffected - none carries a target field, so they are broadcast and FC-handled.
//
// 5.1.7 added the bench confirmation that settled the last Stage 3 question: the VIO's
// zoomLevel FOLLOWS THE ACTIVE EO/IR SOURCE, so no sensor check is needed in the FC and
// none was added.  Had it reported EO zoom while the operator watched IR, TM2 would have
// carried a number that was not what it claimed to be.  Both ranges fit TM2's int8_t
// field - EO reaches 30x in super-resolution, IR 8x.
//
// The bench aid that answered it is kept but disabled:
// AP_CAMERA_MAVLINKCAMV2_ZOOM_DEBUG in AP_Camera_MAVLinkCamV2.h defaults to 0.  The decode
// is permanent, so only the reporting is conditional and the off build is byte-identical
// to having no debug code at all.
//
// With Stage 3 done, the FC-side work on D10 is complete.  What remains is KGCS
// implementation against mappings documented in README and VIO_MAVLink_Messages.md:
// EO/IR source, IR palette, object tracking and absolute zoom are all KGCS-to-payload
// direct, needing no firmware.
//
// Carried forward from 5.1.1 to 5.1.4: D10 Stage 1 (no fabricated camera values, every
// declined camera command reported), the Gremsy VIO protocol analysis, the dual-GCS
// compatibility strategy, the KGCS pitch reversal relocated into AP_Q30, D10 Stage 2a
// (camera calls routed through AP::camera(), CAM1_TYPE selecting the destination), and
// the Gremsy hardware validation - including a one-shot guard on the "Camera: ..."
// statustext, an upstream defect worth submitting to ArduPilot.
//
// REQUIRES these settings on the aircraft, which the firmware cannot do itself:
//   - MNT1_PITCH_MIN / MNT1_PITCH_MAX in PHYSICAL convention since 5.1.2.  Viewpro
//     -90 / 45; the Gremsy VIO bench-verified at -90 / 15
//   - MNT1_YAW_MIN / MNT1_YAW_MAX set explicitly (-180 / 180) on a Gremsy, to override
//     the bad default the gimbal supplies
//   - CAM1_TYPE: 4 for Viewpro, 6 for Gremsy.  Load-bearing for the KGCS camera path
//     since 5.1.3, where before it only served a standard GCS
//   - a Gremsy VIO's camera component set to 100, not its default 101 - ArduPilot
//     instance 0 searches for 100, and AP_Q30 hardcodes instance 0
// D10 Stage 2b (zoom units) and D11 remain open.  It becomes OFFICIAL when they land.
// See README "Deferred Issues", PNU-ISSUE Dn.

// NOTE: write every component as plain decimal, with NO leading zero.  A leading zero
// makes a C integer literal octal: 08 and 09 are not valid octal and fail to compile,
// while 00-07 compile but only work by coincidence.  So for any value below 10 write
// 8, not 08.
// The same rule applies to OFP_VER_MAIN/SUB/REV in libraries/GCS_MAVLink/GCS.h, which
// must stay in step with the three defines below (static_assert in Copter.cpp, D4).

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 5,1,16,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 16
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
