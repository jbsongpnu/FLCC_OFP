#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.12"

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
#define FIRMWARE_VERSION 5,1,12,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 12
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
