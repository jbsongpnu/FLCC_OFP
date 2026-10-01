/*
   PNU-KAL HD FLCC ICD handlers for the KGCS link.

   Split out of GCS_Common.cpp (PNU-ISSUE D5): as one block appended at that file's
   EOF these ~356 lines re-conflicted on every ArduPilot version bump, and the cost
   grew with each one.  Here they cost nothing to merge.

   What necessarily stays in GCS_Common.cpp, because it is interleaved with upstream
   code: the four cases in try_send_message(), the three in handle_message(), and the
   four entries in the mavlink_id -> ap_message map.

   Messages handled (all MAVLink2 only, id >= 50001 - see README PNU-ISSUE D2):
     TC1  50001  SYS_ICD_GCS_FLCC_PMU_CTRL                 PMU control from KGCS
     TC2  50002  SYS_ICD_GCS_FLCC_CAM_CMD                  camera / gimbal from KGCS
     TC4  50004  SYS_ICD_GCS_FLCC_OBJECT_AVOIDANCE_CMD     avoidance level from KGCS
     TM1  51001  SYS_ICD_FLCC_GCS_PMU_STATUS               PMU status to KGCS
     TM2  51002  SYS_ICD_FLCC_GCS_CAM_ATTITUDE_STATUS      camera attitude to KGCS
     TM3  51003  SYS_ICD_GCS_FLCC_PMU_CTRL_ECHO            PMU command echo to KGCS
     TM5  51005  SYS_ICD_FLCC_GCS_OBJECT_AVOIDANCE_STATUS  avoidance status to KGCS
 */

#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "GCS.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AC_Avoidance/AC_Avoid.h>

// PNU-ISSUE(D14) : the shared PNU globals are declared by the libraries that define
// them - AP_Q30.h for the camera state, AP_PMUCAN.h for PMU_Status / PMU_Ctrl_Echo.
// They used to be extern'd by hand here, unchecked against their definitions.
#include <AP_Q30/AP_Q30.h>
#include <AP_PMUCAN/AP_PMUCAN.h>

// PNU : incoming-message debug for Viewpro IR pseudo-color/palette commands.
// Set to 1 to report the Tracking_CMD received by handle_gcs_flcc_cam_cmd() to the GCS.
#define GCS_VIEWPRO_IR_DEBUG 0

// PNU-ISSUE(D10) : how long after boot TM2 waits before deciding the mount cannot
// report zoom, in milliseconds.  Covers the gimbal's startup telemetry delay.
#define PNU_CAM_ZOOM_GRACE_MS 10000

// PNU : object-avoidance warning thresholds reported to KGCS in TM5, in centimetres
#define PNU_OA_ALERT_DISTANCE_CM 1000
#define PNU_OA_WARN_DISTANCE_CM  1500

// PNU-ISSUE(D13) : the two camera handlers below reach the gimbal through AP::mount()
// and AP::Q30(), neither of which exists when the mount is compiled out.
#if AP_Q30_ENABLED

// -------------------------------------------------------------------------
// Send CAM status to GCS (TM2 / msg 51002)
// -------------------------------------------------------------------------
void GCS_MAVLINK::send_message_gcs_flcc_cam_status() const
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }

    // PNU-ISSUE(D10) : with no gimbal configured every field below is zero, so TM2 would
    // stream a perfectly level, 0x-zoom camera at 10 Hz to every connected GCS - the same
    // "nothing fitted looks like nothing wrong" shape as TM5 in D12.  Note this gates on
    // a mount being configured, NOT on the mount supporting zoom: TM2 is mostly gimbal
    // attitude, which a Gremsy reports correctly even though it answers no camera call.
    if (mount->get_mount_type(0) == AP_Mount::Type::None) {
        return;
    }

    float roll = 0, pitch = 0, yaw = 0;
    if (mount->get_attitude_euler(0, roll, pitch, yaw)) {
        CAM_ATTITUDE_STATUS.Roll_REL_ANG = CAM_ATTITUDE_STATUS.Roll_IMU_ANG = roll * 10;
        // reverse the pitch angle to match the KGCS image convention, as
        // AP_Mount_Backend::set_angle_target() does for the command direction
        CAM_ATTITUDE_STATUS.Pitch_REL_ANG = CAM_ATTITUDE_STATUS.Pitch_IMU_ANG = -pitch * 10;
        CAM_ATTITUDE_STATUS.Yaw_REL_ANG = CAM_ATTITUDE_STATUS.Yaw_IMU_ANG = yaw * 10;
    } else {
        CAM_ATTITUDE_STATUS.Roll_REL_ANG = CAM_ATTITUDE_STATUS.Roll_IMU_ANG = 0;
        CAM_ATTITUDE_STATUS.Pitch_REL_ANG = CAM_ATTITUDE_STATUS.Pitch_IMU_ANG = 0;
        CAM_ATTITUDE_STATUS.Yaw_REL_ANG = CAM_ATTITUDE_STATUS.Yaw_IMU_ANG = 0;
    }

    // PNU-ISSUE(D10) : the wire format is deliberately unchanged.  Zoom_POS_FB has no
    // "unknown" encoding, so a mount that cannot report zoom still sends 0 - adding one
    // is an ICD change needing KGCS work, the same decision taken for TM5 in D12.  What
    // changes is that 0 is now sent because the value is unknown, not because a
    // fabricated 0.0f was mistaken for a reading, and the operator is told once on the
    // edge rather than left to infer it from a camera that appears stuck at 0x.
    float zoom_times = 0;
    const bool zoom_known = mount->get_zoom_times(0, zoom_times);
    CAM_ATTITUDE_STATUS.Zoom_POS_FB = zoom_known ? (int8_t)zoom_times : 0;

    // A working Viewpro reports zoom only once the gimbal's first telemetry frame
    // arrives, so evaluating the edge from boot would warn on every startup and then
    // immediately recover.  Past the grace period, still-unavailable means the mount
    // genuinely does not report zoom, which is the condition worth announcing.
    const bool zoom_unavailable = !zoom_known && (AP_HAL::millis() > PNU_CAM_ZOOM_GRACE_MS);
    if (zoom_unavailable != gcs().prev_zoom_unavailable) {
        gcs().prev_zoom_unavailable = zoom_unavailable;
        if (zoom_unavailable) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAM: zoom not reported, TM2 sends 0");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAM: zoom reporting active");
        }
    }

    mavlink_msg_sys_icd_flcc_gcs_cam_attitude_status_send(
            chan,
            CAM_ATTITUDE_STATUS.Roll_REL_ANG,
            CAM_ATTITUDE_STATUS.Pitch_REL_ANG,
            CAM_ATTITUDE_STATUS.Yaw_REL_ANG,
            CAM_ATTITUDE_STATUS.Roll_IMU_ANG,
            CAM_ATTITUDE_STATUS.Roll_RC_Target_ANG,
            CAM_ATTITUDE_STATUS.Pitch_IMU_ANG,
            CAM_ATTITUDE_STATUS.Pitch_RC_Target_ANG,
            CAM_ATTITUDE_STATUS.Yaw_IMU_ANG,
            CAM_ATTITUDE_STATUS.Yaw_RC_Target_ANG,
            CAM_ATTITUDE_STATUS.Zoom_POS_FB);

#if HAL_LOGGING_ENABLED
    AP::logger().Write("TC_R", "TimeUS,RRLA,PRLA,YRLA,RIMA,RRCA,PIMA,PRCA,YIMA,YRCA,ZPOS", "Qiiihhhhhhb",
                       AP_HAL::micros64(),
                       CAM_ATTITUDE_STATUS.Roll_REL_ANG,
                       CAM_ATTITUDE_STATUS.Pitch_REL_ANG,
                       CAM_ATTITUDE_STATUS.Yaw_REL_ANG,
                       CAM_ATTITUDE_STATUS.Roll_IMU_ANG,
                       CAM_ATTITUDE_STATUS.Roll_RC_Target_ANG,
                       CAM_ATTITUDE_STATUS.Pitch_IMU_ANG,
                       CAM_ATTITUDE_STATUS.Pitch_RC_Target_ANG,
                       CAM_ATTITUDE_STATUS.Yaw_IMU_ANG,
                       CAM_ATTITUDE_STATUS.Yaw_RC_Target_ANG,
                       CAM_ATTITUDE_STATUS.Zoom_POS_FB);
#endif
}


// -------------------------------------------------------------------------
// Receive CAM control command from GCS (TC2 / msg 50002)
// -------------------------------------------------------------------------
void GCS_MAVLINK::handle_gcs_flcc_cam_cmd(const mavlink_message_t &msg)
{
    mavlink_sys_icd_gcs_flcc_cam_cmd_t cam_cmd;
    mavlink_msg_sys_icd_gcs_flcc_cam_cmd_decode(&msg, &cam_cmd);

#if GCS_VIEWPRO_IR_DEBUG
    // Echo the incoming Tracking_CMD so we can confirm which IR-color request
    // (14: WhiteHot, 15: BlackHot, 16-19: Color1-4) was received from the GCS.
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FLCC CAM_CMD RX Track=%u Ctrl=%u",
                  (unsigned)cam_cmd.Tracking_CMD,
                  (unsigned)cam_cmd.Control_Mode);
#endif

    AP_Q30 *Q30 = AP::Q30();
    if (Q30 == nullptr) {
        return;
    }

    debug_cam_gimbal_cmd    = 0U;
    debug_cam_zoom_cmd      = 0U;
    debug_cam_focus_cmd     = 0U;
    debug_cam_record_cmd    = 0U;
    debug_cam_track_cmd     = 0U;
    debug_cam_ir_cmd        = 0U;

    switch (cam_cmd.Control_Mode) {

    case 0: // no control
        Q30->no_control_mode_operation(cam_cmd);
        break;

    case 1: // speed control
        // legacy protocol: 2-axis speed control only, roll is not controllable
        if ((cam_cmd.Pitch_Speed_CMD != 0) || (cam_cmd.Yaw_Speed_CMD != 0)) {
            tracking_counter = 0U;
            Q30->send_cmd_speed(cam_cmd);
            debug_cam_gimbal_cmd = 1U;      // 1: rate control
        } else if (tracking_counter == 0U) {
            tracking_counter = 1U;
            Q30->send_cmd_hold_angle();
            debug_cam_gimbal_cmd = 3U;      // 3: fix/hold
        }
        break;

    case 2: // angle control
        Q30->send_cmd_angle(cam_cmd);
        debug_cam_gimbal_cmd = 2U;          // 2: angle control
        break;

    case 3: // ROI control - not implemented
    default:
        break;
    }

    Q30->IR_operation(cam_cmd);

#if HAL_LOGGING_ENABLED
    AP::logger().Write("TC_C", "TimeUS,RSPD,RAGL,PSPD,PANG,YSPD,YANG,CNTM,ZOOM,SHUT,TRAK", "QhhhhhhBBBB",
                       AP_HAL::micros64(),
                       cam_cmd.Roll_Speed_CMD,
                       cam_cmd.Roll_Angle_CMD,
                       cam_cmd.Pitch_Speed_CMD,
                       cam_cmd.Pitch_Angle_CMD,
                       cam_cmd.Yaw_Speed_CMD,
                       cam_cmd.Yaw_Angle_CMD,
                       cam_cmd.Control_Mode,
                       cam_cmd.Zoom_Focus_Stop_CMD,
                       cam_cmd.Shutter_CMD,
                       cam_cmd.Tracking_CMD);

    AP::logger().Write("CM_C", "TimeUS,GIMB,ZOOM,FOCS,RECD,TRAK,IR", "QBBBBBB",
                       AP_HAL::micros64(),
                       debug_cam_gimbal_cmd,
                       debug_cam_zoom_cmd,
                       debug_cam_focus_cmd,
                       debug_cam_record_cmd,
                       debug_cam_track_cmd,
                       debug_cam_ir_cmd);
#endif
}


#endif  // AP_Q30_ENABLED

// -------------------------------------------------------------------------
// Send PMU status to GCS (TM1 / msg 51001)
// -------------------------------------------------------------------------
void GCS_MAVLINK::send_message_gcs_flcc_pmu_status() const
{
    mavlink_msg_sys_icd_flcc_gcs_pmu_status_send(
            chan,
            PMU_Status.Engine_Hour_Count,
            PMU_Status.Date,
            PMU_Status.Battery_Current,
            PMU_Status.Battery_Temp,
            PMU_Status.Battery_Status,
            PMU_Status.Engine_RPM,
            PMU_Status.Engine_Head1_Temp,
            PMU_Status.Engine_Head2_Temp,
            PMU_Status.Battery_Quantity_Command,
            PMU_Status.System_Voltage,
            PMU_Status.Load_Current,
            PMU_Status.Current_Control_Command,
            PMU_Status.PMU_Temp,
            PMU_Status.PMU_Status,
            PMU_Status.Throttle_Position_Report,
            PMU_Status.Fuel_Quantity,
            PMU_Status.Version_Sub_Number,
            PMU_Status.Version_Main_Number,
            PMU_Status.Version_FLCC_Sub_Number,
            PMU_Status.Version_FLCC_Main_Number,
            PMU_Status.Version_FLCC_REV_Number,
            PMU_Status.Version_REV_Number);

#if HAL_LOGGING_ENABLED
    AP::logger().Write("TM11", "TimeUS,HOURCNT,DATE,IBAT,TEMP,BSTS,RPM,TEMP1,TEMP2,GCMD", "QIIhhHHhhh",
                       AP_HAL::micros64(),
                       PMU_Status.Engine_Hour_Count,
                       PMU_Status.Date,
                       PMU_Status.Battery_Current,
                       PMU_Status.Battery_Temp,
                       PMU_Status.Battery_Status,
                       PMU_Status.Engine_RPM,
                       PMU_Status.Engine_Head1_Temp,
                       PMU_Status.Engine_Head2_Temp,
                       PMU_Status.Battery_Quantity_Command);

    AP::logger().Write("TM12", "TimeUS,VBUS,ILD,ICMD,PTEMP,PSTS,PCL,GAS,MN,MJ", "QhhhhHbbbb",
                       AP_HAL::micros64(),
                       PMU_Status.System_Voltage,
                       PMU_Status.Load_Current,
                       PMU_Status.Current_Control_Command,
                       PMU_Status.PMU_Temp,
                       PMU_Status.PMU_Status,
                       PMU_Status.Throttle_Position_Report,
                       PMU_Status.Fuel_Quantity,
                       PMU_Status.Version_Sub_Number,
                       PMU_Status.Version_Main_Number);

    AP::logger().Write("TM13", "TimeUS,FLMJ,FLMN,FLRV,PMRV", "Qbbbb",
                       AP_HAL::micros64(),
                       PMU_Status.Version_FLCC_Main_Number,
                       PMU_Status.Version_FLCC_Sub_Number,
                       PMU_Status.Version_FLCC_REV_Number,
                       PMU_Status.Version_REV_Number);
#endif
}


// -------------------------------------------------------------------------
// Send PMU command echo to GCS (TM3 / msg 51003)
// -------------------------------------------------------------------------
void GCS_MAVLINK::send_message_gcs_flcc_pmu_ctrl_echo() const
{
    mavlink_msg_sys_icd_gcs_flcc_pmu_ctrl_echo_send(
            chan,
            PMU_Ctrl_Echo.Engine_OnOff_Echo,
            PMU_Ctrl_Echo.Battery_Control_CMD_Echo,
            PMU_Ctrl_Echo.Engine_Manual_Echo,
            PMU_Ctrl_Echo.Engine_Throttle_CMD_Echo,
            PMU_Ctrl_Echo.Engine_CHK_CMD_Echo,
            PMU_Ctrl_Echo.Componet_ID,
            PMU_Ctrl_Echo.PMUCAN_Fail,
            PMU_Ctrl_Echo.Reserved);
}


// -------------------------------------------------------------------------
// Receive PMU control command from GCS (TC1 / msg 50001)
// -------------------------------------------------------------------------
void GCS_MAVLINK::handle_gcs_flcc_pmu_ctrl(const mavlink_message_t &msg)
{
    mavlink_msg_sys_icd_gcs_flcc_pmu_ctrl_decode(&msg, &gcs().PMU_Ctrl);

    // consumed by AP_PMUCAN, which acts on each increment
    gcs().PMU_Ctrl_Seq = gcs().PMU_Ctrl_Seq + 1;

#if HAL_LOGGING_ENABLED
    AP::logger().Write("TC1", "TimeUS,EGOF,BCTC,EGM,ETRC,ECHK", "QBBBBB",
                       AP_HAL::micros64(),
                       gcs().PMU_Ctrl.Engine_OnOff,
                       gcs().PMU_Ctrl.Battery_Control_CMD,
                       gcs().PMU_Ctrl.Engine_Manual,
                       gcs().PMU_Ctrl.Engine_Throttle_CMD,
                       gcs().PMU_Ctrl.Engine_CHK_CMD);
#endif
}

#if HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED
// -------------------------------------------------------------------------
// Send object avoidance status to GCS (TM5 / msg 51005)
// -------------------------------------------------------------------------
void GCS_MAVLINK::send_message_flcc_gcs_object_avoidance_status() const
{
    AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity == nullptr) {
        return;
    }

    uint8_t obj_exists = 0;     // object presence per sector, bit0..bit7
    uint8_t warning_level = 0;  // 0:none, 1:warning, 2:alert

    // A configured sensor that has stopped reporting must not be scanned for
    // objects: get_horizontal_distances() fills every sector with dist_max on
    // failure, which reads back as "no object" and is indistinguishable from a
    // clear scene.  sensor_failed() is the same predicate that drives the
    // MAV_SYS_STATUS_SENSOR_PROXIMITY health bit, so TM5 and SYS_STATUS agree.
    // It is false when no sensor is configured at all.  See README PNU-ISSUE D12.
    const bool prx_failed = proximity->sensor_failed();

    Proximity_Distance_Array dist_array;
    if (!prx_failed && proximity->get_horizontal_distances(dist_array)) {
        // min/max come from the driver, not from a parameter - e.g. TeraRanger
        // Tower Evo hard-codes 0.5 m / 60 m in AP_Proximity_TeraRangerTowerEvo.h
        const uint16_t dist_min_cm = (uint16_t)(proximity->distance_min_m() * 100.0f);
        const uint16_t dist_max_cm = (uint16_t)(proximity->distance_max_m() * 100.0f);

        for (uint8_t i = 0; i < PROXIMITY_MAX_DIRECTION; i++) {
            if (!dist_array.valid(i)) {
                // sector never reported a distance: unknown, not clear
                continue;
            }
            const uint16_t distance_cm = (uint16_t)(dist_array.distance[i] * 100.0f);
            if ((distance_cm >= dist_min_cm) && (distance_cm < dist_max_cm)) {
                obj_exists |= (1U << i);
                if (distance_cm <= PNU_OA_ALERT_DISTANCE_CM) {
                    warning_level |= 2;
                } else if (distance_cm <= PNU_OA_WARN_DISTANCE_CM) {
                    warning_level |= 1;
                }
            }
        }
        if (warning_level > 2) {
            warning_level = 2;      // alert wins over warning
        }
    }

    // TM5 has no encoding for "sensor unhealthy", so report the edge out of band.
    // KGCS can also read it from the SYS_STATUS proximity health bit.
    if (prx_failed != gcs().prev_prx_failed) {
        gcs().prev_prx_failed = prx_failed;
        if (prx_failed) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Avoidance: proximity sensor failed");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Avoidance: proximity sensor recovered");
        }
    }

    gcs().OA_Status.Object_Avoidance_Status = warning_level;
    gcs().OA_Status.Object_Existence = obj_exists;

    mavlink_msg_sys_icd_flcc_gcs_object_avoidance_status_send(
            chan,
            gcs().OA_Status.Object_Avoidance_Status,
            gcs().OA_Status.Object_Avoidance_Mode,
            gcs().OA_Status.Object_Existence);
}

// -------------------------------------------------------------------------
// Receive object avoidance level command from GCS (TC4 / msg 50004)
// -------------------------------------------------------------------------
void GCS_MAVLINK::handle_gcs_flcc_object_avoidance_cmd(const mavlink_message_t &msg)
{
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid == nullptr) {
        return;
    }

    mavlink_msg_sys_icd_gcs_flcc_object_avoidance_cmd_decode(&msg, &gcs().GCS_Ctrl_OA_Mode);

    // KGCS streams this message, so act on a change of mode only
    if (gcs().prev_Ctrl_OA_Mode == gcs().GCS_Ctrl_OA_Mode.OA_Mode) {
        return;
    }

    if (gcs().GCS_Ctrl_OA_Mode.OA_Mode) {
        // OA level 1 or 2: force avoidance on
        gcs().OA_Status.Object_Avoidance_Mode = gcs().GCS_Ctrl_OA_Mode.OA_Mode;
        avoid->proximity_avoidance_enable(true);
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "GCS command Avoidance ON Level %u",
                      (unsigned)gcs().GCS_Ctrl_OA_Mode.OA_Mode);
        // enabling avoidance against a dead sensor achieves nothing - say so now,
        // rather than leaving TM5 to report a permanently clear scene (D12)
        const AP_Proximity *proximity = AP_Proximity::get_singleton();
        if (proximity != nullptr && proximity->sensor_failed()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Avoidance ON but proximity sensor failed");
        }
    } else {
        // OA level 0: turn avoidance off
        gcs().OA_Status.Object_Avoidance_Mode = 0;
        avoid->proximity_avoidance_enable(false);
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "GCS command Avoidance OFF");
    }
    gcs().prev_Ctrl_OA_Mode = gcs().GCS_Ctrl_OA_Mode.OA_Mode;
}
#endif // HAL_PROXIMITY_ENABLED && AP_AVOIDANCE_ENABLED

#endif  // HAL_GCS_ENABLED
