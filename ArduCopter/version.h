#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// PNU : This is a custom code modified by PNU Drone
#define THISFIRMWARE "PNU_OFP V5.1.8"

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
#define FIRMWARE_VERSION 5,1,8,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 5
#define FW_MINOR 1
#define FW_PATCH 8
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
