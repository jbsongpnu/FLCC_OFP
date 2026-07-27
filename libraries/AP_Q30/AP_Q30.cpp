#include "AP_Q30.h"
#include <AP_Mount/AP_Mount.h>

extern const AP_HAL::HAL& hal;


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
// Send "set_angle" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_angle(mavlink_sys_icd_gcs_flcc_cam_cmd_t cmd)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }

    mount->set_angle_target(0, cmd.Roll_Angle_CMD, cmd.Pitch_Angle_CMD, cmd.Yaw_Angle_CMD, 0);
    
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
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }

    switch (cam_cmd.Zoom_Focus_Stop_CMD)
    {
        // Stop //19.5.20 수정
        case 0:
            mount->set_zoom(0, ZoomType::RATE, 0);
            //Focus also stops. So, same as :  mount->set_focus(0, FocusType::RATE, 0);

            debug_cam_zoom_cmd = 3U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            debug_cam_focus_cmd = 3U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Zoom In
        case 1:
            mount->set_zoom(0, ZoomType::RATE, 1);
            debug_cam_zoom_cmd = 1U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Zoom Out
        case 2:
            mount->set_zoom(0, ZoomType::RATE, -1);
            debug_cam_zoom_cmd = 2U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Focus In
        case 3:
            mount->set_focus(0, FocusType::RATE, 1);
            debug_cam_focus_cmd = 1U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Focus Out
        case 4:
            mount->set_focus(0, FocusType::RATE, -1);
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
            mount->record_video(0, 1);
            debug_cam_record_cmd = 1U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        // Record Stop
        case 2:
            mount->record_video(0, 0);
            debug_cam_record_cmd = 2U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        // Shutter
        case 4:
            mount->take_picture(0);
            debug_cam_record_cmd = 3U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        default:
            break;
        }
    }

    // TRACKING
    if(cam_cmd.Tracking_CMD==1U)        //start
    {
        mount->set_tracking(0, TrackingType::TRK_POINT, Vector2f{0.5, 0.5}, Vector2f{});    //check do_aux_function_camera_image_tracking() in RC_Channel.cpp for usage
        debug_cam_track_cmd = 1U; //debug //0: not sent, 1: start, 2: stop
    }
    else if(cam_cmd.Tracking_CMD==2U)   //stop
    {
        mount->set_tracking(0, TrackingType::TRK_NONE, Vector2f{0.5, 0.5}, Vector2f{});    //check do_aux_function_camera_image_tracking() in RC_Channel.cpp for usage
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
    static uint8_t EO_zoom_pct = 1;
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    
    // Control Window Combination
    if(cam_cmd.Tracking_CMD==10)                // EO FULL & IR PIP
    {
        mount->set_camera_source(0, 1, 2);
        _primary_EOIR_source = 1;
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==11)           // IR FULL
    {
        mount->set_camera_source(0, 2, 0);
        _primary_EOIR_source = 2;
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==12)           // IR FULL & EO PIP
    {
        mount->set_camera_source(0, 2, 1);
        _primary_EOIR_source = 2;
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==13U)          // EO FULL
    {
        mount->set_camera_source(0, 1, 0);
        _primary_EOIR_source = 1;
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else
    {
        if(_primary_EOIR_source == 1) {     //if EO is the main source, get zoom for EO
            _current_zoom_EO = mount->get_zoom_times(0);
        } else if (_primary_EOIR_source == 2) { //if IR is the main source, get zoom for IR
            _current_zoom_IR = mount->get_zoom_times(0);
        }
    }
    // gcs().send_text(MAV_SEVERITY_ERROR,"Zoom %u Foc %u Rec %u Trk %u IR %u", debug_cam_zoom_cmd, debug_cam_focus_cmd, debug_cam_record_cmd, debug_cam_track_cmd, debug_cam_ir_cmd);

    // Control Image Color (IR pseudo-color palette via Viewpro C1 packet)
    if(cam_cmd.Tracking_CMD==14U)               // White Hot
    {
        mount->IR_Color_Change(0, 0x0E);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==15U)          // Black Hot
    {
        mount->IR_Color_Change(0, 0x0F);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==16U)          // Color 1
    {
        mount->IR_Color_Change(0, 0x21);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==17U)          // Color 2
    {
        mount->IR_Color_Change(0, 0x22);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==18U)          // Color 3
    {
        // mount->IR_Color_Change(0, 0x23);
        mount->IR_Color_Change(0, 0x0E);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==19U)          // Color 4
    {
        // mount->IR_Color_Change(0, 0x24);
        mount->IR_Color_Change(0, 0x0F);
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
                EO_zoom_pct++;
            }
            else if(cam_cmd.Tracking_CMD < prev_EO_zoom_cmd) 
            {
                EO_zoom_pct--;
            } 
            else //for cam_cmd.Tracking_CMD == prev_IR_zoom_cmd case
            {
                if (cam_cmd.Tracking_CMD == 21)  //same as ((prev_IR_zoom_cmd == 21) && (cam_cmd.Tracking_CMD == 21)
                {
                    EO_zoom_pct--;
                }
                else if (cam_cmd.Tracking_CMD == 24)
                {
                    EO_zoom_pct++;
                }
            }
            //constrain
            if(EO_zoom_pct > _Max_zoom_EO) 
            {
                EO_zoom_pct = _Max_zoom_EO;
            }
            if(EO_zoom_pct < 1) 
            {
                EO_zoom_pct = 1;
            }
            mount->set_zoom(0, ZoomType::PCT, EO_zoom_pct);
            prev_EO_zoom_cmd = cam_cmd.Tracking_CMD;
        } else if (_primary_EOIR_source == 2) { //This case will lead to zoom IR
            if (cam_cmd.Tracking_CMD > prev_IR_zoom_cmd) 
            {
                mount->set_zoom(0, ZoomType::RATE, 1.0);
            }
            else if(cam_cmd.Tracking_CMD < prev_IR_zoom_cmd) 
            {
                mount->set_zoom(0, ZoomType::RATE, -1.0);
            } 
            else //for cam_cmd.Tracking_CMD == prev_IR_zoom_cmd case
            {
                if (cam_cmd.Tracking_CMD == 21)  //same as ((prev_IR_zoom_cmd == 21) && (cam_cmd.Tracking_CMD == 21)
                {
                    mount->set_zoom(0, ZoomType::RATE, -1.0);
                }
                else if (cam_cmd.Tracking_CMD == 24)
                {
                    mount->set_zoom(0, ZoomType::RATE, 1.0);
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
