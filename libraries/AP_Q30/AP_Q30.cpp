#include "AP_Q30.h"

extern const AP_HAL::HAL& hal;


// -------------------------------------------------------------------------
// Define variables for CAM
// -------------------------------------------------------------------------
int32_t tracking_counter     = 0U;                                          // Tracking Counter for CAM (KAL)
uint32_t CAM_Scheduler_Count = 0U;                                          // Scheduler Counter for CAM (KAL)
uint8_t debug_cam_gimbal_cmd = 0U;                                          // Gimbal status for logging (KAL)
uint8_t debug_cam_zoom_cmd   = 0U;                                          // Zoom status for logging (KAL)
uint8_t debug_cam_focus_cmd  = 0U;                                          // Focus status for logging (KAL)
uint8_t debug_cam_record_cmd = 0U;                                          // Record status for logging (KAL)
uint8_t debug_cam_track_cmd  = 0U;                                          // Tracking status for logging (KAL)
uint8_t debug_cam_ir_cmd     = 0U;                                          // IR status for logging (KAL)
static float Pan_CMD_Prev    = 0.0f;

mavlink_sys_icd_gcs_flcc_cam_cmd_t              PREV_CAM_CMD = {0};         // Backup CAM_CMD (KAL)
mavlink_sys_icd_flcc_gcs_cam_attitude_status_t  CAM_ATTITUDE_STATUS = {0};  // MAVLINK Message for CAM (KAL)


struct TYPE_Q30_TARGET Q30_Target {0, 0, 0, false};


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
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE];

    // Update Buffer (Header)
    buffer[0] = 0xFF;
    buffer[1] = 0x01;
    buffer[2] = 0x0F;
    buffer[3] = 0x10;

    // Check Roll Status & Set Command
    if (PREV_CAM_CMD.Roll_Angle_CMD != cmd.Roll_Angle_CMD)
    {
        PREV_CAM_CMD.Roll_Angle_CMD = cmd.Roll_Angle_CMD;

        buffer[4] = 0x05; // RM
    }
    else if (0 == cmd.Roll_Angle_CMD)
    {
        buffer[4] = 0x05; // RM//19.5.20수정
    }
    else
    {
        buffer[4] = 0x05; // stop cam flow -> restore 0x05 20.08.10
    }

    // Check Pitch Status & Set Command
    if (PREV_CAM_CMD.Pitch_Angle_CMD  != cmd.Pitch_Angle_CMD)
    {
        PREV_CAM_CMD.Pitch_Angle_CMD = cmd.Pitch_Angle_CMD;

        buffer[5] = 0x05; // PM
    }
    else if (0 == cmd.Pitch_Angle_CMD)
    {
        buffer[5] = 0x05; // PM//19.5.20수정
    }
    else
    {
        buffer[5] = 0x05; // stop cam flow -> restore 0x05 20.08.10
    }

    // Check Yaw Status & Set Command
    if (PREV_CAM_CMD.Yaw_Angle_CMD  != cmd.Yaw_Angle_CMD)
    {
        PREV_CAM_CMD.Yaw_Angle_CMD = cmd.Yaw_Angle_CMD;

        buffer[6] = 0x05; // YM
    }
    else if (0 == cmd.Yaw_Angle_CMD)
    {
        buffer[6] = 0x05; // YM//19.5.20수정
    }
    else
    {
        buffer[6] = 0x05; // stop cam flow -> restore 0x05 20.08.10
    }

    // Update Buffer
    buffer[7] = 0x00;   // RSL
    buffer[8] = 0x00;   // RSH
    buffer[9] = get_cam_angle_byte_l(cmd.Roll_Angle_CMD);   // RAL
    buffer[10] = get_cam_angle_byte_h(cmd.Roll_Angle_CMD);  // RAH
    buffer[11] = 0x00;  // PSL
    buffer[12] = 0x00;  // PSH
    buffer[13] = get_cam_angle_byte_l(cmd.Pitch_Angle_CMD); // PAL
    buffer[14] = get_cam_angle_byte_h(cmd.Pitch_Angle_CMD); // PAH
    buffer[15] = 0x00;  // YSL
    buffer[16] = 0x00;  // YSH
    buffer[17] = get_cam_angle_byte_l(cmd.Yaw_Angle_CMD);   // YAL
    buffer[18] = get_cam_angle_byte_h(cmd.Yaw_Angle_CMD);   // YAH
    buffer[19] = get_cam_checksum(buffer, 4, 19);

    // Send Buffer
    CAM_UART->write(buffer, 20);
}


// -------------------------------------------------------------------------
// Send "set_speed" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_speed(mavlink_sys_icd_gcs_flcc_cam_cmd_t cmd)
{
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE];

    // Update Buffer (Header)
    buffer[0] = 0xFF;
    buffer[1] = 0x01;
    buffer[2] = 0x0F;
    buffer[3] = 0x10;

    // Check Roll Status & Set Command
    if (PREV_CAM_CMD.Roll_Speed_CMD != cmd.Roll_Speed_CMD)
    {
        PREV_CAM_CMD.Roll_Speed_CMD = cmd.Roll_Speed_CMD;

        buffer[4] = 0x01; // RM
    }
    else if (0 == cmd.Roll_Speed_CMD)
    {
        buffer[4] = 0x01; // RM//19.5.20수정
    }
    else
    {
        buffer[4] = 0x01; // RM//19.5.20수정
    }

    // Check Pitch Status & Set Command
    if (PREV_CAM_CMD.Pitch_Speed_CMD != cmd.Pitch_Speed_CMD)
    {
        PREV_CAM_CMD.Pitch_Speed_CMD = cmd.Pitch_Speed_CMD;

        buffer[5] = 0x01; // PM
    }
    else if (0 == cmd.Pitch_Speed_CMD)
    {
        buffer[5] = 0x01; // PM//19.5.20수정
    }
    else
    {
        buffer[5] = 0x01; // PM//19.5.20수정
    }

    // Check Yaw Status & Set Command
    if (PREV_CAM_CMD.Yaw_Speed_CMD != cmd.Yaw_Speed_CMD)
    {
        PREV_CAM_CMD.Yaw_Speed_CMD = cmd.Yaw_Speed_CMD;

        buffer[6] = 0x01; // YM
    }
    else if (0 == cmd.Yaw_Speed_CMD)
    {
        buffer[6] = 0x01; // YM//19.5.20수정
    }
    else
    {
        buffer[6] = 0x01; // YM//19.5.20수정
    }

    // Update Buffer
    buffer[7] = get_cam_speed_byte_l(cmd.Roll_Speed_CMD);   // RSL
    buffer[8] = get_cam_speed_byte_h(cmd.Roll_Speed_CMD);   // RSH
    buffer[9] = 0x00;   // RAL
    buffer[10] = 0x00;  // RAH
    buffer[11] = get_cam_speed_byte_l(cmd.Pitch_Speed_CMD); // PSL
    buffer[12] = get_cam_speed_byte_h(cmd.Pitch_Speed_CMD); // PSH
    buffer[13] = 0x00;  // PAL
    buffer[14] = 0x00;  // PAH
    buffer[15] = get_cam_speed_byte_l(cmd.Yaw_Speed_CMD);   // YSL
    buffer[16] = get_cam_speed_byte_h(cmd.Yaw_Speed_CMD);   // YSH//19.5.13 수정 iajo l->h
    buffer[17] = 0x00;  // YAL
    buffer[18] = 0x00;  // YAH
    buffer[19] = get_cam_checksum(buffer, 4, 19);

    // Send Buffer
    CAM_UART->write(buffer, 20);
}


// -------------------------------------------------------------------------
// Send "start_track" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_track_start()
{
    // Declare Variables
    uint8_t buffer[CAM_TRACK_UART_BUFFER_SIZE]={0U};

    // Initialize Buffer
    memset(buffer, 0, CAM_TRACK_UART_BUFFER_SIZE);

    // Set Start Command
    buffer[0]=0x7e;
    buffer[1]=0x7e;
    buffer[2]=0x44;
    buffer[3]=0x00;
    buffer[4]=0x00;
    buffer[5]=0x71;
    buffer[6]=0xfe;
    buffer[7]=0x00;
    buffer[8]=0x00;
    buffer[9]=0x00;
    buffer[10]=0x00;
    buffer[11]=0x01;
    buffer[12]=0x00;
    buffer[13]=0x3c;
    buffer[47]=0xEC;

    // Send Buffer
    CAM_UART->write(buffer, CAM_TRACK_UART_BUFFER_SIZE);
}


// -------------------------------------------------------------------------
// Send "end_track" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_track_end()
{
    // Declare Variables
    uint8_t buffer[CAM_TRACK_UART_BUFFER_SIZE]={0U};

    // Initialize Buffer
    memset(buffer, 0, CAM_TRACK_UART_BUFFER_SIZE);

    // Set End Command
    buffer[0]=0x7e;
    buffer[1]=0x7e;
    buffer[2]=0x44;
    buffer[3]=0x00;
    buffer[4]=0x00;
    buffer[5]=0x00;
    buffer[6]=0xfe;
    buffer[7]=0x00;
    buffer[8]=0x00;
    buffer[9]=0x00;
    buffer[10]=0x00;
    buffer[11]=0x00;
    buffer[12]=0x00;
    buffer[13]=0x3c;
    buffer[47]=0x7A;

    // Send Buffer
    CAM_UART->write(buffer, CAM_TRACK_UART_BUFFER_SIZE);
}


// -------------------------------------------------------------------------
// Send "set ir_color" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_ir_color(uint8_t Color, uint8_t White, uint8_t checksum)
{
    // Declare Variables
    uint8_t buffer[CAM_TRACK_UART_BUFFER_SIZE]={0U};

    // Initialize Buffer
    memset(buffer, 0, CAM_TRACK_UART_BUFFER_SIZE);

    // Set Color Command
    buffer[0]=0x7e;
    buffer[1]=0x7e;
    buffer[2]=0x44;
    buffer[3]=0x00;
    buffer[4]=0x00;
    buffer[5]=0x78;
    buffer[6]=Color;
    buffer[7]=0x00;
    buffer[8]=0x00;
    buffer[9]=0x00;
    buffer[10]=0x00;
    buffer[11]=0x00;
    buffer[12]=0x00;
    buffer[13]=0x00;
    buffer[14]=0x01;
    buffer[15]=White;
    buffer[47]=checksum;

    // Send Buffer
    CAM_UART->write(buffer, CAM_TRACK_UART_BUFFER_SIZE);
}


// -------------------------------------------------------------------------
// Send "set eo/ir_mode" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_eo_ir_mode(uint8_t mode, uint8_t checksum)
{
    // Declare Variables
    uint8_t buffer[CAM_TRACK_UART_BUFFER_SIZE]={0U};

    // Initialize Buffer
    memset(buffer, 0, CAM_TRACK_UART_BUFFER_SIZE);

    // Set EO/IR Mode
    buffer[0]=0x7e;
    buffer[1]=0x7e;
    buffer[2]=0x44;
    buffer[3]=0x00;
    buffer[4]=0x00;
    buffer[5]=0x78;
    buffer[6]=0x00;
    buffer[7]=0x00;
    buffer[8]=0x00;
    buffer[9]=0x00;
    buffer[10]=0x00;
    buffer[11]=0x00;
    buffer[12]=0x00;
    buffer[13]=0x00;
    buffer[14]=mode;
    buffer[47]=checksum;

    // Send Buffer
    CAM_UART->write(buffer, CAM_TRACK_UART_BUFFER_SIZE);
}


// -------------------------------------------------------------------------
// Send "set ir zoom" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_ir_digital_zoom(uint8_t ratio)
{
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE] = {0};

    // Update Buffer
    buffer[0] = 0x7E;
    buffer[1] = 0x7E;
    buffer[2] = 0x44;
    buffer[3] = 0x00;
    buffer[4] = 0x00;
    buffer[5] = 0x7D;
    buffer[6] = 0x80 + ratio;
    buffer[14] = 0x00;
    buffer[47] = get_cam_checksum(buffer, 0, 47);

    // Send Buffer
    CAM_UART->write(buffer, 48);
}


// -------------------------------------------------------------------------
// Send "set zoom" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_zoom(uint8_t zoom)
{
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE];

    // Update Buffer
    buffer[0] = 0x81;
    buffer[1] = 0x01;
    buffer[2] = 0x04;
    buffer[3] = 0x07;
    buffer[4] = zoom;
    buffer[5] = 0xFF;

    // Send Buffer
    CAM_UART->write(buffer, 6);
}


// -------------------------------------------------------------------------
// Send "set focus" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_focus(uint8_t focus)
{
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE];

    // Update Buffer
    buffer[0] = 0x81;
    buffer[1] = 0x01;
    buffer[2] = 0x04;
    buffer[3] = 0x08;
    buffer[4] = focus;
    buffer[5] = 0xFF;

    // Send Buffer
    CAM_UART->write(buffer, 6);
}


// -------------------------------------------------------------------------
// Send "set shutter" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_shutter(uint8_t shutter)
{
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE] = {0};

    // Update Buffer
    buffer[0] = 0x7E;
    buffer[1] = 0x7E;
    buffer[2] = 0x44;
    buffer[3] = 0x00;
    buffer[4] = 0x00;
    buffer[5] = 0x7C;
    buffer[6] = shutter;
    buffer[47] = get_cam_checksum(buffer, 0, 47);

    // Send Buffer
    CAM_UART->write(buffer, 48);
}


// -------------------------------------------------------------------------
// Send "Hold angle" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::send_cmd_hold_angle(void)
{
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE];

    // Update Buffer
    buffer[0] = 0xFF;   //header
    buffer[1] = 0x01;   //header
    buffer[2] = 0x0F;   //header
    buffer[3] = 0x10;   //header

    buffer[4] = 0x00;   //Angel ref frame control mode
    buffer[5] = 0x00;   //Pitch ref frame control mode
    buffer[6] = 0x00;   //Yaw ref frame control mode


    buffer[7] = 0x00; // RSL    //Roll_rate
    buffer[8] = 0x00; // RSH
    buffer[9] = 0x00;
    buffer[10] = 0x00;
    buffer[11] = 0x00; // PSL   //Pitch_rate
    buffer[12] = 0x00; // PSH

    buffer[13] = 0x00;
    buffer[14] = 0x00;
    buffer[15] = 0x00; // YSL   //Yaw_rate
    buffer[16] = 0x00; // YSH
    buffer[17] = 0x00;
    buffer[18] = 0x00;
    buffer[19] = get_cam_checksum(buffer, 4, 19);

    // Send Buffer
    CAM_UART->write(buffer, 20);
}


// -------------------------------------------------------------------------
// Send "get_angle" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::get_cmd_angle() const
{
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE];

    // Update Buffer
    buffer[0] = 0x3E;
    buffer[1] = 0x3D;
    buffer[2] = 0x00;
    buffer[3] = 0x3D;
    buffer[4] = 0x00;

    // Send Buffer
    CAM_UART->write(buffer, 5);
}


// -------------------------------------------------------------------------
// Send "get_zoom" Command to CAM
// -------------------------------------------------------------------------
void AP_Q30::get_cmd_zoom() const
{
    // Declare Variables
    uint8_t buffer[CAM_UART_BUFFER_SIZE];

    // Update Buffer
    buffer[0] = 0x81;
    buffer[1] = 0x09;
    buffer[2] = 0x04;
    buffer[3] = 0x47;
    buffer[4] = 0xFF;

    // Send Buffer
    CAM_UART->write(buffer, 5);
}


// -------------------------------------------------------------------------
// Stop CAM & Stabilize
// -------------------------------------------------------------------------
void AP_Q30::no_control_mode_operation(mavlink_sys_icd_gcs_flcc_cam_cmd_t cam_cmd)
{
    switch (cam_cmd.Zoom_Focus_Stop_CMD)
    {
        // Stop //19.5.20 수정
        case 0:
            send_cmd_zoom(0x00);
            send_cmd_focus(0x00);

            debug_cam_zoom_cmd = 3U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            debug_cam_focus_cmd = 3U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Zoom In
        case 1:
            send_cmd_zoom(0x27);

            debug_cam_zoom_cmd = 1U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Zoom Out
        case 2:
            send_cmd_zoom(0x37);

            debug_cam_zoom_cmd = 2U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Focus In
        case 3:
            send_cmd_focus(0x27);
            debug_cam_focus_cmd = 1U; //debug //0: not sent, 1: IN, 2: OUT, 3: STOP
            break;

        // Focus Out
        case 4:
            send_cmd_focus(0x37);
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
            send_cmd_shutter(0x01);
            debug_cam_record_cmd = 1U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        // Record Stop
        case 2:
            send_cmd_shutter(0x00);
            debug_cam_record_cmd = 2U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        // Shutter
        case 4:
            send_cmd_shutter(0x02);
            debug_cam_record_cmd = 3U; //debug //0: not sent, 1: start, 2: stop, 3: shutter
            break;

        default:
            break;
        }
    }

    // TRACKING
    if(cam_cmd.Tracking_CMD==1U)        //start
    {
        send_cmd_track_start();
        debug_cam_track_cmd = 1U; //debug //0: not sent, 1: start, 2: stop
    }
    else if(cam_cmd.Tracking_CMD==2U)   //stop
    {
        send_cmd_track_end();
        debug_cam_track_cmd = 2U; //debug //0: not sent, 1: start, 2: stop
    }
    else
    {
        //nothing
    }

}


// -------------------------------------------------------------------------
// Control IR Functions
// -------------------------------------------------------------------------
void AP_Q30::IR_operation(mavlink_sys_icd_gcs_flcc_cam_cmd_t cam_cmd)
{
    // Control Window Combination
    if(cam_cmd.Tracking_CMD==10)                // EO FULL & IR PIP
    {
        send_cmd_eo_ir_mode(0x00, 0xb8);
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==11)           // IR FULL
    {
        send_cmd_eo_ir_mode(0x01, 0xb9);
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==12)           // IR FULL & EO PIP
    {
        send_cmd_eo_ir_mode(0x02, 0xba);
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==13U)          // EO FULL
    {
        send_cmd_eo_ir_mode(0x03, 0xbb);
        debug_cam_ir_cmd = 1U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else
    {
        //nothing
    }


    // Control Image Color
    if(cam_cmd.Tracking_CMD==14U)               // White Hot
    {
        send_cmd_ir_color(0x00, 0x01, 0xba);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==15U)          // Black Hot
    {
        send_cmd_ir_color(0x00, 0x00, 0xb9);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==16U)          // Color 1
    {
        send_cmd_ir_color(0x01, 0x00, 0xba);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==17U)          // Color 2
    {
        send_cmd_ir_color(0x02, 0x00, 0xbb);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==18U)          // Color 3
    {
        send_cmd_ir_color(0x03, 0x00, 0xbc);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==19U)          // Color 4
    {
        send_cmd_ir_color(0x04, 0x00, 0xbd);
        debug_cam_ir_cmd = 2U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else
    {
        //nothing
    }


    // Control IR Zoom
    if(cam_cmd.Tracking_CMD==21U)           // IR Zoom 1x
    {
        send_cmd_ir_digital_zoom(1);
        debug_cam_ir_cmd = 3U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==22U)      // IR Zoom 2x
    {
        send_cmd_ir_digital_zoom(2);
        debug_cam_ir_cmd = 3U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==23U)      // IR Zoom 3x
    {
        send_cmd_ir_digital_zoom(3);
        debug_cam_ir_cmd = 3U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else if(cam_cmd.Tracking_CMD==24U)      // IR Zoom 4x
    {
        send_cmd_ir_digital_zoom(4);
        debug_cam_ir_cmd = 3U; //debug //0: not sent, 1: mode, 2: coler, 3: zoom
    }
    else
    {
        //nothing
    }
}


// -------------------------------------------------------------------------
// Receive Data from CAM
// -------------------------------------------------------------------------
int32_t AP_Q30::receive_cam_uart_data(uint16_t* buffer) const
{
    int32_t recv_size = 0;

    while ((CAM_UART->available() > 0) && (CAM_UART_BUFFER_SIZE > recv_size))
    {
        buffer[recv_size] = CAM_UART->read();
        recv_size = recv_size + 1;
    }

    return recv_size;
}


// -------------------------------------------------------------------------
// Calculate Checksum
// -------------------------------------------------------------------------
uint8_t AP_Q30::get_cam_checksum(uint8_t* buffer, int pos, int size)
{
    uint8_t checksum = 0;

    for (int i = pos; i < size; i++)
    {
        checksum = (uint8_t)(checksum + buffer[i]);
    }

    return checksum;
}


 // -------------------------------------------------------------------------
 // Parse the "angle" Data from CAM
 // -------------------------------------------------------------------------
void AP_Q30::parse_cam_angle(uint16_t* buffer) const
{
    if ((0x003E == buffer[0]) && (0x003D == buffer[1]) && (0x0036 == buffer[2]) && (0x0073 == buffer[3]))
    {
        CAM_ATTITUDE_STATUS.Roll_REL_ANG        = (int32_t)(get_cam_angle_32(&buffer[8]) * 10.0f);
        CAM_ATTITUDE_STATUS.Pitch_REL_ANG       = (int32_t)(get_cam_angle_32(&buffer[26]) * 10.0f);
        CAM_ATTITUDE_STATUS.Yaw_REL_ANG         = (int32_t)(get_cam_angle_32(&buffer[44]) * 10.0f);
        CAM_ATTITUDE_STATUS.Roll_IMU_ANG        = (int16_t)(get_cam_angle_16(&buffer[4]) * 10.0f);
        CAM_ATTITUDE_STATUS.Roll_RC_Target_ANG  = (int16_t)(get_cam_angle_16(&buffer[6]) * 10.0f);
        CAM_ATTITUDE_STATUS.Pitch_IMU_ANG       = (int16_t)(get_cam_angle_16(&buffer[22]) * 10.0f);
        CAM_ATTITUDE_STATUS.Pitch_RC_Target_ANG = (int16_t)(get_cam_angle_16(&buffer[24]) * 10.0f);
        CAM_ATTITUDE_STATUS.Yaw_IMU_ANG         = (int16_t)(get_cam_angle_16(&buffer[40]) * 10.0f);
        CAM_ATTITUDE_STATUS.Yaw_RC_Target_ANG   = (int16_t)(get_cam_angle_16(&buffer[42]) * 10.0f);
    }
}


// -------------------------------------------------------------------------
// Parse the "zoom" Data from CAM
// -------------------------------------------------------------------------
void AP_Q30::parse_zoom_position(uint16_t* buffer) const
{
    if ((0x0090 == buffer[0]) && (0x0050 == buffer[1]))
    {
        uint16_t Zoom = (uint16_t)(((buffer[2] << 12) & 0xF000) | ((buffer[3] << 8) & 0x0F00) | ((buffer[4] << 4) & 0x00F0) | (buffer[5] & 0x000F));

        if(Zoom <= 0)//iajo
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 1;
            }

        else if(Zoom > 0 && Zoom <= 0x16A1)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 2;
            }

        else if(Zoom > 0x16A1 && Zoom <= 0x2063)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 3;
            }

        else if(Zoom > 0x2063 && Zoom <= 0x2628)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 4;
            }

        else if(Zoom > 0x2628 && Zoom <= 0x2A1D)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 5;
            }
        else if(Zoom > 0x2A1D && Zoom <= 0x2D13)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 6;
            }
        else if(Zoom > 0x2D13 && Zoom <= 0x2F6D)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 7;
            }
        else if(Zoom > 0x2F6D && Zoom <= 0x3161)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 8;
            }
        else if(Zoom > 0x3161 && Zoom <= 0x330D)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 9;
            }
        else if(Zoom > 0x330D && Zoom <= 0x3486)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 10;
            }
        else if(Zoom > 0x3486 && Zoom <= 0x35D7)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 11;
            }
        else if(Zoom > 0x35D7 && Zoom <= 0x3709)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 12;
            }
        else if(Zoom > 0x3709 && Zoom <= 0x3820)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 13;
            }
        else if(Zoom > 0x3820 && Zoom <= 0x3920)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 14;
            }
        else if(Zoom > 0x3920 && Zoom <= 0x3A0A)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 15;
            }
        else if(Zoom > 0x3A0A && Zoom <= 0x3ADD)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 16;
            }
        else if(Zoom > 0x3ADD && Zoom <= 0x3B9C)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 17;
            }
        else if(Zoom > 0x3B9C && Zoom <= 0x3C46)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 18;
            }
        else if(Zoom > 0x3C46 && Zoom <= 0x3CDC)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 19;
            }
        else if(Zoom > 0x3CDC && Zoom <= 0x3D60)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 20;
            }
        else if(Zoom > 0x3D60 && Zoom <= 0x3DD4)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 21;
            }
        else if(Zoom > 0x3DD4 && Zoom <= 0x3E39)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 22;
            }
        else if(Zoom > 0x3E39 && Zoom <= 0x3E90)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 23;
            }
        else if(Zoom > 0x3E90 && Zoom <= 0x3EDC)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 24;
            }
        else if(Zoom > 0x3EDC && Zoom <= 0x3F1E)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 25;
            }
        else if(Zoom > 0x3F1E && Zoom <= 0x3F57)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 26;
            }
        else if(Zoom > 0x3F57 && Zoom <= 0x3F8A)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 27;
            }
        else if(Zoom > 0x3F8A && Zoom <= 0x3FB6)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 28;
            }
        else if(Zoom > 0x3FB6 && Zoom <= 0x3FDC)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 29;
            }
        else if(Zoom > 0x3FDC && Zoom <= 0x4000)
            {
                CAM_ATTITUDE_STATUS.Zoom_POS_FB = 30;
            }
        else
        {

        }
    }
}


// -------------------------------------------------------------------------
// calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
// -------------------------------------------------------------------------
bool AP_Q30::calc_angle_to_location(Vector3f& angles_to_target_rad)
{
    Location current_loc;

    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    // Calculate relative distance from target to vehicle position
    // Now North direction in X, East direction is Y (KAL)
    const float GPS_vector_y = (Q30_Target.lng-current_loc.lng)*cosf(ToRad((current_loc.lat+Q30_Target.lat)*0.00000005f))*0.01113195f;
    const float GPS_vector_x = (Q30_Target.lat-current_loc.lat)*0.01113195f;

    int32_t current_alt_cm = 0;
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt_cm)) {
        return false;
    }

    float GPS_vector_z = (float)((current_alt_cm - Q30_Target.alt)*0.01);// Convert to meter (KAL)

    // 3-2-1 DCM Matrix for Gimbal coordinate (Gimbal Coordinate is just same as KUS-HD3 Attitude) (KAL)
    float phi   = wrap_PI(AP::ahrs().roll);
    float theta = wrap_PI(AP::ahrs().pitch);
    float psi   = wrap_PI(AP::ahrs().yaw);

    float body_x = 0.0f;
    float body_y = 0.0f;
    float body_z = 0.0f;

    body_x =                                 cosf(theta)*cosf(psi)*GPS_vector_x +                                 cosf(theta)*sinf(psi)*GPS_vector_y -           sinf(theta)*GPS_vector_z;
    body_y = (sinf(phi)*sinf(theta)*cosf(psi)-cosf(phi)*sinf(psi))*GPS_vector_x + (sinf(phi)*sinf(theta)*sinf(psi)+cosf(phi)*cosf(psi))*GPS_vector_y + sinf(phi)*cosf(theta)*GPS_vector_z;
    body_z = (cosf(phi)*sinf(theta)*cosf(psi)+sinf(phi)*sinf(psi))*GPS_vector_x + (cosf(phi)*sinf(theta)*sinf(psi)-sinf(phi)*cosf(psi))*GPS_vector_y + cosf(phi)*cosf(theta)*GPS_vector_z;

    float target_distance = norm(body_x, body_y); // Now every thing changed to meter Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    // Initialize all angles to zero
    angles_to_target_rad.zero();

    // Calculate tilt angle
    angles_to_target_rad.y = atan2f(body_z,target_distance);// Using body axis coordinate 21.10.12

    //Gimbal cmd expend
    float pan_cmd = 0.0f;
    float pan_limit = radians(290);
    float pan_original = wrap_PI(atan2f(body_y,body_x));//Using body axis coordinate 21.10.12

    pan_cmd = pan_angle_calc(pan_original, Q30_Target.new_loc);
    pan_cmd = pan_angle_limit(pan_cmd, pan_original, pan_limit);

    angles_to_target_rad.z = pan_cmd;

    return true;
}


// -------------------------------------------------------------------------
// Decode CAM angle for 2byte buffer
// -------------------------------------------------------------------------
float AP_Q30::get_cam_angle_16(uint16_t* buffer) const
{
    int16_t dummy = (int16_t)(((buffer[1] << 8) & 0xFF00) | (buffer[0] & 0x00FF));
    float value = (float)dummy * 0.02197f;

    return value;
}


// -------------------------------------------------------------------------
// Decode CAM angle for 4byte buffer
// -------------------------------------------------------------------------
float AP_Q30::get_cam_angle_32(uint16_t* buffer) const
{
    int32_t dummy = (int32_t)(((buffer[3] << 24) & 0xFF000000) | ((buffer[2] << 16) & 0x00FF0000) | ((buffer[1] << 8) & 0x0000FF00) | (buffer[0] & 0x000000FF));
    float value = (float)dummy * 0.02197f;

    return value;
}


// -------------------------------------------------------------------------
// Encode angle to lower byte
// -------------------------------------------------------------------------
uint8_t AP_Q30::get_cam_angle_byte_l(int16_t angle)
{
    angle = (int16_t)(angle / 0.02197F);
    uint8_t byte = (uint8_t)(angle & 0x00FF) ;

    return byte;
}


// -------------------------------------------------------------------------
// Encode angle to upper byte
// -------------------------------------------------------------------------
uint8_t AP_Q30::get_cam_angle_byte_h(int16_t angle)
{
    angle = (int16_t)(angle / 0.02197F);
    uint8_t byte = (uint8_t)((angle >> 8) & 0x00FF);

    return byte;
}


// -------------------------------------------------------------------------
// Encode speed to lower byte
// -------------------------------------------------------------------------
uint8_t AP_Q30::get_cam_speed_byte_l(int16_t speed)
{
    speed = (int16_t)(speed / 0.122F);
    uint8_t byte = (uint8_t)(speed & 0x00FF) ;

    return byte;
}


// -------------------------------------------------------------------------
// Encode speed to upper byte
// -------------------------------------------------------------------------
uint8_t AP_Q30::get_cam_speed_byte_h(int16_t speed)
{
    speed = (int16_t)(speed / 0.122F);
    uint8_t byte = (uint8_t)((speed >> 8) & 0x00FF);

    return byte;
}


// -------------------------------------------------------------------------
// Calculate pan angle cmd -2pi~2pi (KAL)
// -------------------------------------------------------------------------
float AP_Q30::pan_angle_calc(float pan_angle, bool new_loc)
{
    float sign = 1.0f;
    float pan_res = 0.0f;

    if(new_loc)
    {
        pan_res         = pan_angle;
        Pan_CMD_Prev    = pan_res;
        new_loc         = false;
        return pan_res;
    }

    if(((pan_angle * Pan_CMD_Prev)<0.0f) && (fabsf(pan_angle - Pan_CMD_Prev) > radians(270.0f)))
    {
        if(pan_angle < 0.0f)
        {
            sign = 1.0f;
        } else {
            sign = -1.0f;
        }
        pan_res = radians(360.0f)*sign + pan_angle;
    } else{
        pan_res = pan_angle;
    }

    Pan_CMD_Prev = pan_res;

    return pan_res;
}

// -------------------------------------------------------------------------
// Limit pan angle cmd accroding to gimbal spec (KAL)
// -------------------------------------------------------------------------
float AP_Q30::pan_angle_limit(float pan_angle, float pan_original, float pan_limit)
{
    if(fabsf(pan_angle)<pan_limit)
    {
        return pan_angle;
    } else {
        return pan_original;
    }
}

namespace AP {

AP_Q30 *Q30()
{
    return AP_Q30::get_singleton();
}

};
