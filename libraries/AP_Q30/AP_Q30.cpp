#include "AP_Q30.h"

#if AP_Q30_ENABLED
#include <AP_Mount/AP_Mount.h>
#include <AP_Camera/AP_Camera.h>

//  ==================================================================================
//  PNU-KAL OFP Firmware version
//  ==================================================================================

extern const AP_HAL::HAL& hal;

// PNU-ISSUE(D10) : minimum gap between "mount rejected ..." reports, in milliseconds.
// Matches the interval the D6 PMU short-frame message uses, for the same reason: the
// condition repeats at the message rate, so an unlimited report would be a 10 Hz stream.
#define AP_Q30_CAM_WARN_INTERVAL_MS 10000


// -------------------------------------------------------------------------
// Define variables for CAM
// -------------------------------------------------------------------------
int32_t tracking_counter     = 0U;                                          // Tracking Counter for CAM (KAL)
uint8_t debug_cam_gimbal_cmd = 0U;                                          // Gimbal status for logging (KAL)
uint8_t debug_cam_zoom_cmd   = 0U;                                          // Zoom status for logging (KAL)
uint8_t debug_cam_focus_cmd  = 0U;                                          // Focus status for logging (KAL)
uint8_t debug_cam_record_cmd = 0U;                                          // Record status for logging (KAL)
uint8_t debug_cam_track_cmd  = 0U;                                          // Tracking status for logging (KAL)
uint8_t debug_cam_ir_cmd     = 0U;                                          // IR status for logging (KAL)

mavlink_sys_icd_gcs_flcc_cam_cmd_t              PREV_CAM_CMD = {0};         // Backup CAM_CMD (KAL)
mavlink_sys_icd_flcc_gcs_cam_attitude_status_t  CAM_ATTITUDE_STATUS = {0};  // MAVLINK Message for CAM (KAL)

AP_Q30::AP_Q30()
{
    if (_singleton) {
        return;
    }
    _singleton = this;
}


// -------------------------------------------------------------------------
// Get the AP_Q30 singleton
// -------------------------------------------------------------------------
AP_Q30 *AP_Q30::_singleton = nullptr;
AP_Q30 *AP_Q30::get_singleton()
{
    return _singleton;
}

// -------------------------------------------------------------------------
// PNU-ISSUE(D10) : report a camera command the mount would not carry out
// -------------------------------------------------------------------------
void AP_Q30::report_cam_unsupported(const char *what)
{
    _cam_unsupported_cnt++;

    // always report the first one, then at most one message per interval: the GCS streams
    // TC2 at ~10 Hz, so on a mount without camera support every tick fails
    const uint32_t now_ms = AP_HAL::millis();
    if ((_cam_unsupported_cnt > 1) &&
        ((now_ms - _cam_unsupported_last_ms) < AP_Q30_CAM_WARN_INTERVAL_MS)) {
        return;
    }
    _cam_unsupported_last_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAM: mount rejected %s (%u failed)",
                  what, (unsigned)_cam_unsupported_cnt);
}

// -------------------------------------------------------------------------
// PNU-ISSUE(D10) Stage 3 : zoom times for TM2, from whichever payload is fitted
// -------------------------------------------------------------------------
bool AP_Q30::get_tm2_zoom_times(float &zoom_times)
{
    // A Viewpro carries gimbal and camera on one serial protocol and reports zoom in its
    // own telemetry, so the mount answers directly and already in times.
    AP_Mount *mount = AP::mount();
    if ((mount != nullptr) && mount->get_zoom_times(0, zoom_times)) {
        return true;
    }

    // A MAVLink camera reports zoom in CAMERA_SETTINGS.zoomLevel.
    //
    // **The MAVLink spec defines that field as a percentage of full range, but the Gremsy
    // VIO reports zoom TIMES** - bench-verified 2026-10-13 reading 1.00 at wide and 30.00
    // at full tele in super-resolution mode, matching its video overlay exactly, and
    // matching TM2's own ICD range of 1..30.
    //
    // That reinterpretation is a fact about the payloads THIS firmware flies, not about
    // MAVLink, which is why it lives here in the KGCS-interface adapter rather than in
    // AP_Camera.  (The interface is shared: KGCS speaks it, and PNU-GCS can select it.)
    // AP_Camera deliberately reports the raw number without claiming to know its units;
    // see README "Dual-GCS compatibility strategy", design rule 1.
    //
    // A spec-compliant camera reporting a percentage would therefore be misreported here
    // as times.  Accepted: TM2 is a PNU message carried on PNU aircraft, whose payloads
    // are known.  Revisit if a third camera type is ever fitted.
    AP_Camera *camera = AP::camera();
    if ((camera != nullptr) && camera->get_reported_zoom_level(0, zoom_times)) {
        return true;
    }

    return false;
}

// -------------------------------------------------------------------------
// PNU-ISSUE(D10) : is the fitted gimbal a Viewpro?
// -------------------------------------------------------------------------
bool AP_Q30::mount_is_viewpro() const
{
#if HAL_MOUNT_VIEWPRO_ENABLED
    const AP_Mount *mount = AP::mount();
    return (mount != nullptr) && (mount->get_mount_type(0) == AP_Mount::Type::Viewpro);
#else
    return false;
#endif
}

// -------------------------------------------------------------------------
// PNU-ISSUE(D10) : note once that a TC2 function is GCS-direct on this payload
// -------------------------------------------------------------------------
void AP_Q30::report_gcs_direct(uint8_t which, const char *what)
{
    if (_gcs_direct_reported & which) {
        return;
    }
    _gcs_direct_reported |= which;

    // MAV_SEVERITY_INFO, and said once: this is the designed split, not a fault
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAM: %s is GCS-direct on this mount", what);
}

// -------------------------------------------------------------------------
// PNU-ISSUE(D10) : IR palette.  Only AP_Mount_Viewpro implements IR_Color_Change() -
// MAVLink defines no thermal-palette message at all.  On a Gremsy VIO the palette is the
// C_T_PALETTE extended parameter, which the GCS sets directly on the camera because
// ArduPilot implements no PARAM_EXT_*.  Note that means PNU-GCS, not KGCS - KGCS is only
// ever used with a Viewpro.  See README "Supported GCS and gimbal combinations".
// -------------------------------------------------------------------------
void AP_Q30::set_ir_palette(uint8_t color)
{
    if (!mount_is_viewpro()) {
        report_gcs_direct(GCS_DIRECT_IR_PALETTE, "IR palette");
        return;
    }

    AP_Mount *mount = AP::mount();
    if ((mount == nullptr) || !mount->IR_Color_Change(0, color)) {
        // a Viewpro that declines is a real failure, so warn as usual
        report_cam_unsupported("IR palette");
    }
}

// -------------------------------------------------------------------------
// PNU-ISSUE(D10) : object tracking.  AP_Camera_MAVLinkCamV2 implements no set_tracking(),
// and ArduPilot never *sends* MAV_CMD_CAMERA_TRACK_POINT - it only receives it.  On a
// Gremsy VIO tracking lives on the payload app (component 26) behind MAV_CMD_USER_4, which
// no AP_Camera path can reach, so the GCS drives it directly.
// -------------------------------------------------------------------------
void AP_Q30::set_tracking_state(TrackingType tracking_type)
{
    if (!mount_is_viewpro()) {
        report_gcs_direct(GCS_DIRECT_TRACKING, "tracking");
        return;
    }

    //check do_aux_function_camera_image_tracking() in RC_Channel.cpp for usage
    AP_Camera *camera = AP::camera();
    if ((camera == nullptr) ||
        !camera->set_tracking(0, tracking_type, Vector2f{0.5, 0.5}, Vector2f{})) {
        report_cam_unsupported("tracking");
    }
}

// -------------------------------------------------------------------------
// PNU-ISSUE(D10) : select the EO/IR video source, guarded for builds without it
// -------------------------------------------------------------------------
void AP_Q30::set_video_source(AP_Camera::CameraSource primary, AP_Camera::CameraSource secondary)
{
    // PNU-ISSUE(D10) : only AP_Mount_Viewpro implements set_camera_source(); MAVLink has no
    // standard EO/IR/PIP selection.  On a Gremsy VIO it is the C_SOURCE extended parameter,
    // which the GCS sets directly on the camera.
    if (!mount_is_viewpro()) {
        report_gcs_direct(GCS_DIRECT_VIDEO_SOURCE, "video source");
        return;
    }

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
    AP_Camera *camera = AP::camera();
    if ((camera == nullptr) || !camera->set_camera_source(0, primary, secondary)) {
        report_cam_unsupported("video source");
    }
#else
    (void)primary;
    (void)secondary;
    report_cam_unsupported("video source");
#endif
}

// -------------------------------------------------------------------------
// Send "set_angle" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_angle(mavlink_sys_icd_gcs_flcc_cam_cmd_t cmd)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }

    // PNU : negate pitch HERE, not in AP_Mount_Backend::set_angle_target().  That
    // function is shared with Mission Planner, QGC and AUTO missions, which must keep
    // the standard physical convention; only KGCS wants the image convention.  This
    // matches send_cmd_speed(), which has always negated at the call site.
    // NOTE: set_angle_target() constrains BEFORE this value reaches the gimbal, so
    // MNT1_PITCH_MIN/MAX are now in physical convention - MIN = -(degrees of down
    // travel), MAX = +(degrees of up travel).  See README "Dual-GCS compatibility
    // strategy" for the parameter values this requires.
    mount->set_angle_target(0, cmd.Roll_Angle_CMD, -cmd.Pitch_Angle_CMD, cmd.Yaw_Angle_CMD, 0);
    
}


// -------------------------------------------------------------------------
// Send "set_speed" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_speed(mavlink_sys_icd_gcs_flcc_cam_cmd_t cmd)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }

    // Check Roll Status & Set Command
    if (PREV_CAM_CMD.Roll_Speed_CMD != cmd.Roll_Speed_CMD)
    {
        PREV_CAM_CMD.Roll_Speed_CMD = cmd.Roll_Speed_CMD;
        //Note : Roll can't be controlled anymore - JBS 23.11.08
    }

    // Check Pitch Status & Set Command
    if (PREV_CAM_CMD.Pitch_Speed_CMD != cmd.Pitch_Speed_CMD)
    {
        PREV_CAM_CMD.Pitch_Speed_CMD = cmd.Pitch_Speed_CMD;
    }

    // Check Yaw Status & Set Command
    if (PREV_CAM_CMD.Yaw_Speed_CMD != cmd.Yaw_Speed_CMD)
    {
        PREV_CAM_CMD.Yaw_Speed_CMD = cmd.Yaw_Speed_CMD;
    }

    // sets rate target in deg/s
    // yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
    // void AP_Mount_Backend::set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_is_earth_frame)
    // Pitch_Speed_CMD is now reversed
    mount->set_rate_target(0,-cmd.Pitch_Speed_CMD,cmd.Yaw_Speed_CMD, 0);
    
}

// -------------------------------------------------------------------------
// Removed : Send "start_track" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::send_cmd_track_start()
// -------------------------------------------------------------------------
// Removed : Send "end_track" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::send_cmd_track_end()
// -------------------------------------------------------------------------
// Removed : Send "set ir_color" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::send_cmd_ir_color(uint8_t Color, uint8_t White, uint8_t checksum)
// -------------------------------------------------------------------------
// Removed : Send "set eo/ir_mode" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::send_cmd_eo_ir_mode(uint8_t mode, uint8_t checksum)
// -------------------------------------------------------------------------
// Removed : Send "set ir zoom" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::send_cmd_ir_digital_zoom(uint8_t ratio)
// -------------------------------------------------------------------------
// Removed : Send "set zoom" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::send_cmd_zoom(uint8_t zoom)
// -------------------------------------------------------------------------
// Removed : Send "set focus" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::send_cmd_focus(uint8_t focus)
// -------------------------------------------------------------------------
// Removed : Send "set shutter" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::send_cmd_shutter(uint8_t shutter)
// -------------------------------------------------------------------------
// Send "Hold angle" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_hold_angle(void)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }

    mount->set_rate_target(0,0,0,0);
}

// -------------------------------------------------------------------------
// Removed : Send "get_angle" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::get_cmd_angle() const
// -------------------------------------------------------------------------
// Removed : Send "get_zoom" Command to CAM
// -------------------------------------------------------------------------
// void AP_Q30::get_cmd_zoom() const
// -------------------------------------------------------------------------
// Stop CAM & Stabilize
// -------------------------------------------------------------------------
void AP_Q30::no_control_mode_operation(mavlink_sys_icd_gcs_flcc_cam_cmd_t cam_cmd)
{
    // PNU-ISSUE(D10) : every call in this function is a CAMERA operation, so it goes
    // through AP::camera() rather than AP::mount().  With CAM1_TYPE=4 (Mount) that
    // forwards straight back to the mount, so a Viewpro behaves exactly as before; with
    // CAM1_TYPE=6 (MAVLinkCamV2) it reaches a camera the mount cannot address at all,
    // which is the Gremsy case.  CAM1_TYPE now has to be set correctly for the TC2 camera
    // path to work - see README "Dual-GCS compatibility strategy".
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return;
    }

    switch (cam_cmd.Zoom_Focus_Stop_CMD)
    {
        // Stop //19.5.20 수정
        case 0:
            if (!camera->set_zoom(0, ZoomType::RATE, 0)) {
                report_cam_unsupported("zoom stop");
            }
            //Focus also stops. So, same as :  camera->set_focus(0, FocusType::RATE, 0);

            debug_cam_zoom_cmd = 3U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            debug_cam_focus_cmd = 3U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Zoom In
        case 1:
            if (!camera->set_zoom(0, ZoomType::RATE, 1)) {
                report_cam_unsupported("zoom in");
            }
            debug_cam_zoom_cmd = 1U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Zoom Out
        case 2:
            if (!camera->set_zoom(0, ZoomType::RATE, -1)) {
                report_cam_unsupported("zoom out");
            }
            debug_cam_zoom_cmd = 2U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Focus In
        case 3:
            if (camera->set_focus(0, FocusType::RATE, 1) != SetFocusResult::ACCEPTED) {
                report_cam_unsupported("focus in");
            }
            debug_cam_focus_cmd = 1U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Focus Out
        case 4:
            if (camera->set_focus(0, FocusType::RATE, -1) != SetFocusResult::ACCEPTED) {
                report_cam_unsupported("focus out");
            }
            debug_cam_focus_cmd = 2U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        default:
            break;
    }

    PREV_CAM_CMD.Zoom_Focus_Stop_CMD = cam_cmd.Zoom_Focus_Stop_CMD;

    if (0 == cam_cmd.Zoom_Focus_Stop_CMD)
    {
        switch (cam_cmd.Shutter_CMD)
        {
        // Record Start
        case 1:
            if (!camera->record_video(0, 1)) {
                report_cam_unsupported("record start");
            }
            debug_cam_record_cmd = 1U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        // Record Stop
        case 2:
            if (!camera->record_video(0, 0)) {
                report_cam_unsupported("record stop");
            }
            debug_cam_record_cmd = 2U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        // Shutter
        case 4:
            if (!camera->take_picture(0)) {
                report_cam_unsupported("take picture");
            }
            debug_cam_record_cmd = 3U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        default:
            break;
        }
    }

    // TRACKING
    if(cam_cmd.Tracking_CMD==1U)        //start
    {
        set_tracking_state(TrackingType::TRK_POINT);
        debug_cam_track_cmd = 1U; //debug //0: not sent, 1: start, 2: stop
    }
    else if(cam_cmd.Tracking_CMD==2U)   //stop
    {
        set_tracking_state(TrackingType::TRK_NONE);
        debug_cam_track_cmd = 2U; //debug //0: not sent, 1: start, 2: stop
    }

}

// -------------------------------------------------------------------------
// Control IR Functions
// -------------------------------------------------------------------------
void AP_Q30::IR_operation(mavlink_sys_icd_gcs_flcc_cam_cmd_t cam_cmd)
{
    static uint8_t prev_IR_zoom_cmd = 20;
    static uint8_t prev_EO_zoom_cmd = 0;
    // PNU-ISSUE(D10) : named _times, not _pct, because that is what it holds - it counts
    // 1.._Max_zoom_EO zoom TIMES and is then passed as ZoomType::PCT, which works only
    // because AP_Mount_Viewpro::set_zoom() was modified to reinterpret PCT as times x10.
    // AP_Camera_MAVLinkCamV2::set_zoom(PCT) honours the MAVLink 0-100 % contract instead,
    // so routing this value there unconverted would turn 10x into 10 %, zooming out.
    // Convert at the boundary before any such routing change.
    static uint8_t EO_zoom_times = 1;

    // PNU-ISSUE(D10) : this function needs BOTH handles.  Zoom and video-source are
    // camera operations and go through AP::camera().  get_zoom_times() and
    // IR_Color_Change() have no AP_Camera equivalent at all - they exist only on
    // AP_Mount_Viewpro - so they stay on AP::mount() and simply do nothing on any other
    // gimbal.  On a Gremsy the palette is a C_T_PALETTE extended parameter that the GCS
    // sets directly on the payload; see README "Standard Control for Gremsy VIO".
    AP_Mount *mount = AP::mount();
    AP_Camera *camera = AP::camera();
    if ((mount == nullptr) || (camera == nullptr)) {
        return;
    }
    
    // Control Window Combination
    if(cam_cmd.Tracking_CMD==10)                // EO FULL & IR PIP
    {
        set_video_source(AP_Camera::CameraSource::RGB, AP_Camera::CameraSource::IR);
        _primary_EOIR_source = 1;
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==11)           // IR FULL
    {
        set_video_source(AP_Camera::CameraSource::IR, AP_Camera::CameraSource::DEFAULT);
        _primary_EOIR_source = 2;
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==12)           // IR FULL & EO PIP
    {
        set_video_source(AP_Camera::CameraSource::IR, AP_Camera::CameraSource::RGB);
        _primary_EOIR_source = 2;
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==13U)          // EO FULL
    {
        set_video_source(AP_Camera::CameraSource::RGB, AP_Camera::CameraSource::DEFAULT);
        _primary_EOIR_source = 1;
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else
    {
        // PNU-ISSUE(D10) : a mount that cannot report zoom leaves the cached value alone
        // rather than overwriting it with a fabricated 0.  No warning here on purpose -
        // this runs at the TC2 rate and send_message_gcs_flcc_cam_status() already
        // announces the same condition once, on the edge.
        float zoom_times = 0;
        if (mount->get_zoom_times(0, zoom_times)) {
            if(_primary_EOIR_source == 1) {     //if EO is the main source, get zoom for EO
                _current_zoom_EO = zoom_times;
            } else if (_primary_EOIR_source == 2) { //if IR is the main source, get zoom for IR
                _current_zoom_IR = zoom_times;
            }
        }
    }
    // gcs().send_text(MAV_SEVERITY_ERROR,"Zoom %u Foc %u Rec %u Trk %u IR %u", debug_cam_zoom_cmd, debug_cam_focus_cmd, debug_cam_record_cmd, debug_cam_track_cmd, debug_cam_ir_cmd);

    // Control Image Color (IR pseudo-color palette via Viewpro C1 packet)
    if(cam_cmd.Tracking_CMD==14U)               // White Hot
    {
        set_ir_palette(0x0E);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==15U)          // Black Hot
    {
        set_ir_palette(0x0F);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==16U)          // Color 1
    {
        set_ir_palette(0x21);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==17U)          // Color 2
    {
        set_ir_palette(0x22);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==18U)          // Color 3
    {
        // mount->IR_Color_Change(0, 0x23);
        set_ir_palette(0x0E);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==19U)          // Color 4
    {
        // mount->IR_Color_Change(0, 0x24);
        set_ir_palette(0x0F);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    // Tracking_CMD == 20 is intentionally left unassigned for future use.
    // NOTE : For recent Viewpro camear, color options 2~4 are not supported, but only Red color is supported by IR Rainbow command
    // NOTE : Thereore, any color 1~4 may show Red IR option.

    //IR zoom => Now control main source zoom

    if ((cam_cmd.Zoom_Focus_Stop_CMD == 0)&&(cam_cmd.Tracking_CMD >= 21)&&(cam_cmd.Tracking_CMD <= 24)) {

        debug_cam_ir_cmd = 3U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom

        if (_primary_EOIR_source == 1) {    //This case will lead to zoom EO
            if (cam_cmd.Tracking_CMD > prev_EO_zoom_cmd) 
            {
                EO_zoom_times++;
            }
            else if(cam_cmd.Tracking_CMD < prev_EO_zoom_cmd) 
            {
                EO_zoom_times--;
            } 
            else //for cam_cmd.Tracking_CMD == prev_IR_zoom_cmd case
            {
                if (cam_cmd.Tracking_CMD == 21)  //same as ((prev_IR_zoom_cmd == 21) && (cam_cmd.Tracking_CMD == 21)
                {
                    EO_zoom_times--;
                }
                else if (cam_cmd.Tracking_CMD == 24)
                {
                    EO_zoom_times++;
                }
            }
            //constrain
            if(EO_zoom_times > _Max_zoom_EO) 
            {
                EO_zoom_times = _Max_zoom_EO;
            }
            if(EO_zoom_times < 1) 
            {
                EO_zoom_times = 1;
            }
            // PNU-ISSUE(D10) Stage 2b : absolute zoom is sent from here ONLY to a Viewpro,
            // which reads ZoomType::PCT as zoom times.  Anywhere else PCT becomes
            // ZOOM_TYPE_RANGE, a percentage of the camera's full range - and on a Gremsy VIO
            // that range is mode-dependent (240x combine / 30x super-resolution, selected by
            // the C_V_ZM_MODE extended parameter), which the FC cannot read because ArduPilot
            // implements no PARAM_EXT_*.  Sending 1..30 there would be read as 1-30 %, i.e.
            // zoomed out.  The GCS drives absolute zoom directly instead, with
            // MAV_CMD_USER_4 "zoom to level" in times, so the FC must stay out of it
            // entirely - a rate pulse here would fight the command the GCS is sending.
            // See README "Gremsy PayloadSDK message reference".  EO_zoom_times keeps counting either way so the cached
            // value stays meaningful if the mount changes.
            if (mount_is_viewpro()) {
                if (!camera->set_zoom(0, ZoomType::PCT, EO_zoom_times)) {
                    report_cam_unsupported("zoom level");
                }
            } else {
                report_gcs_direct(GCS_DIRECT_ABS_ZOOM, "absolute zoom");
            }
            prev_EO_zoom_cmd = cam_cmd.Tracking_CMD;
        } else if (_primary_EOIR_source == 2) { //This case will lead to zoom IR
            if (cam_cmd.Tracking_CMD > prev_IR_zoom_cmd) 
            {
                if (!camera->set_zoom(0, ZoomType::RATE, 1.0)) {
                    report_cam_unsupported("IR zoom in");
                }
            }
            else if(cam_cmd.Tracking_CMD < prev_IR_zoom_cmd) 
            {
                if (!camera->set_zoom(0, ZoomType::RATE, -1.0)) {
                    report_cam_unsupported("IR zoom out");
                }
            } 
            else //for cam_cmd.Tracking_CMD == prev_IR_zoom_cmd case
            {
                if (cam_cmd.Tracking_CMD == 21)  //same as ((prev_IR_zoom_cmd == 21) && (cam_cmd.Tracking_CMD == 21)
                {
                    if (!camera->set_zoom(0, ZoomType::RATE, -1.0)) {
                        report_cam_unsupported("IR zoom out");
                    }
                }
                else if (cam_cmd.Tracking_CMD == 24)
                {
                    if (!camera->set_zoom(0, ZoomType::RATE, 1.0)) {
                        report_cam_unsupported("IR zoom in");
                    }
                }
            }
            prev_IR_zoom_cmd = cam_cmd.Tracking_CMD;
        }
    }
    
}

// -------------------------------------------------------------------------
// Removed : Receive Data from CAM
// -------------------------------------------------------------------------
// int32_t AP_Q30::receive_cam_uart_data(uint16_t* buffer) const

// -------------------------------------------------------------------------
// Removed : Calculate Checksum - Legacy method for old packets
// -------------------------------------------------------------------------
// uint8_t AP_Q30::get_cam_checksum(uint8_t* buffer, int pos, int size) const

// -------------------------------------------------------------------------
// Removed : Calculate Checksum for new Viewpro protocol v3.4.9 - JBS 23.11.08
// -------------------------------------------------------------------------
// uint8_t AP_Q30::get_cam_checksumX(uint8_t* buffer, int pos, int size) const

 // -------------------------------------------------------------------------
 // Removed : Parse the "angle" Data from CAM
 // -------------------------------------------------------------------------
// void AP_Q30::parse_cam_angle(uint16_t* buffer) const

// -------------------------------------------------------------------------
// Removed : Parse the "zoom" Data from CAM
// -------------------------------------------------------------------------
// void AP_Q30::parse_zoom_position(uint16_t* buffer) const

// -------------------------------------------------------------------------
// Removed : calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
// -------------------------------------------------------------------------
// bool AP_Q30::calc_angle_to_location(Vector3f& angles_to_target_rad)

// -------------------------------------------------------------------------
// Removed : Decode CAM angle for 2byte buffer
// -------------------------------------------------------------------------
// float AP_Q30::get_cam_angle_16(uint16_t* buffer) const

// -------------------------------------------------------------------------
// Removed : Decode CAM angle for 4byte buffer
// -------------------------------------------------------------------------
// float AP_Q30::get_cam_angle_32(uint16_t* buffer) const

// -------------------------------------------------------------------------
// Removed :Encode angle to lower byte
// -------------------------------------------------------------------------
// uint8_t AP_Q30::get_cam_angle_byte_l(int16_t angle)

// -------------------------------------------------------------------------
// Removed :Encode angle to upper byte
// -------------------------------------------------------------------------
//  Changed by JBS - 23.11.08
// uint8_t AP_Q30::get_cam_angle_byte_h(int16_t angle)

// -------------------------------------------------------------------------
// Removed :Encode speed to lower byte
// -------------------------------------------------------------------------
// uint8_t AP_Q30::get_cam_speed_byte_l(int16_t speed)

// -------------------------------------------------------------------------
// Removed : Encode speed to upper byte
// -------------------------------------------------------------------------
// uint8_t AP_Q30::get_cam_speed_byte_h(int16_t speed)

// -------------------------------------------------------------------------
// Removed : Calculate pan angle cmd -2pi~2pi (KAL)
// -------------------------------------------------------------------------
// float AP_Q30::pan_angle_calc(float pan_angle, bool new_loc)

// -------------------------------------------------------------------------
// Removed : Limit pan angle cmd accroding to gimbal spec (KAL)
// -------------------------------------------------------------------------
// float AP_Q30::pan_angle_limit(float pan_angle, float pan_original, float pan_limit)
namespace AP {

AP_Q30 *Q30()
{
    return AP_Q30::get_singleton();
}

};

#endif  // AP_Q30_ENABLED
