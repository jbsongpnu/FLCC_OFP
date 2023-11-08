/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System
// protocols.
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


// -------------------------------------------------------------------------
// Define Parameters for CAM
#define CAM_UART                        hal.serial(4)       // Serial Port for CAM Interface (KAL)
#define CAM_UART_BUFFER_SIZE            64                  // Serial Buffer Size (KAL)
#define CAM_TRACK_UART_BUFFER_SIZE      48                  // Serial Buffer Size for Track (KAL)


// -------------------------------------------------------------------------
/// @class	Q30
/// @brief	Q30 Equipment Control Class
///
class AP_Q30
{
public:

    AP_Q30();

    static AP_Q30 *get_singleton();
    static AP_Q30 *_singleton;


    // -------------------------------------------------------------------------
    // Declare Functions to Control CAM
    void send_cmd_angle(mavlink_sys_icd_gcs_flcc_cam_cmd_t cmd);                        // Send "set_angle" Command to CAM (KAL)
    void send_cmd_speed(mavlink_sys_icd_gcs_flcc_cam_cmd_t cmd);                        // Send "set_speed" Command to CAM (KAL)
    void send_cmd_track_start();                                                        // Send "start_track" Command to CAM (KAL)
    void send_cmd_track_end();                                                          // Send "end_track" Command to CAM (KAL)
    void send_cmd_ir_color(uint8_t Color, uint8_t White, uint8_t checksum);             // Send "set ir_color" Command to CAM (KAL)
    void send_cmd_eo_ir_mode(uint8_t mode, uint8_t checksum);                           // Send "set eo/ir_mode" Command to CAM (KAL)
    void send_cmd_ir_digital_zoom(uint8_t ratio);                                       // Send "set ir zoom" Command to CAM (KAL)
    void send_cmd_zoom(uint8_t zoom);                                                   // Send "set zoom" Command to CAM (KAL)
    void send_cmd_focus(uint8_t focus);                                                 // Send "set focus" Command to CAM (KAL)
    void send_cmd_shutter(uint8_t shutter);                                             // Send "set shutter" Command to CAM (KAL)
    void send_cmd_hold_angle(void);                                                     // Send "Hold angle" Command to CAM (KAL)

    void get_cmd_angle() const;                                                         // Send "get_angle" Command to CAM (KAL)
    void get_cmd_zoom() const;                                                          // Send "get_zoom" Command to CAM (KAL)

    void no_control_mode_operation(mavlink_sys_icd_gcs_flcc_cam_cmd_t cmd);             // Stop CAM & Stabilize (KAL)
    void IR_operation(mavlink_sys_icd_gcs_flcc_cam_cmd_t cam_cmd);                      // Control IR Functions (KAL)

    int32_t receive_cam_uart_data(uint16_t* buffer) const;                              // Receive Data from CAM (KAL)

    void parse_cam_angle(uint16_t* buffer) const;                                       // Parse the "angle" Data from CAM (KAL)
    void parse_zoom_position(uint16_t* buffer) const;                                   // Parse the "zoom" Data from CAM (KAL)

    bool calc_angle_to_location(Vector3f& angles_to_target_rad);


    // ROI angle expend (KAL)
    float pan_angle_calc(float pan_angle, bool new_loc);                                // Calculate pan angle cmd -2pi~2pi
    float pan_angle_limit(float pan_angle, float pan_original, float pan_limit);        // Limit pan angle cmd accroding to gimbal spec

private:

    // -------------------------------------------------------------------------
    // Declare Functions to parse data with CAM
    uint8_t get_cam_checksum(uint8_t* buffer, int pos, int size);                       // Calculate Checksum (KAL)

    float get_cam_angle_16(uint16_t* buffer) const;                                     // Decode CAM angle for 2byte buffer (KAL)
    float get_cam_angle_32(uint16_t* buffer) const;                                     // Decode CAM angle for 4byte buffer (KAL)

    uint8_t get_cam_angle_byte_l(int16_t angle);                                        // Encode angle to lower byte (KAL)
    uint8_t get_cam_angle_byte_h(int16_t angle);                                        // Encode angle to upper byte (KAL)
    uint8_t get_cam_speed_byte_l(int16_t speed);                                        // Encode speed to lower byte (KAL)
    uint8_t get_cam_speed_byte_h(int16_t speed);                                        // Encode speed to upper byte (KAL)    

};


struct TYPE_Q30_TARGET {
    int32_t alt;
    int32_t lat;
    int32_t lng;
    bool new_loc;                                                                     // Check New ROI target or not
};


namespace AP {
    AP_Q30 *Q30();
};
