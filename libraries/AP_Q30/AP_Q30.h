/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System
// protocols.
#pragma once

//  ==================================================================================
//  PNU-KAL OFP Firmware version
//  ==================================================================================

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_Q30_config.h"

#if AP_Q30_ENABLED

// PNU-ISSUE(D14) : these live in AP_Q30.cpp and are read by GCS_PNU.cpp.  Declared
// here, and included by the defining .cpp, so the compiler checks declaration against
// definition - a hand-written extern elsewhere would not be checked, and a type change
// would become silent memory misinterpretation at link time instead of a build error.
extern int32_t tracking_counter;                                            // PNU : tracking counter for CAM
extern uint8_t debug_cam_gimbal_cmd;                                        // PNU : gimbal status for logging
extern uint8_t debug_cam_zoom_cmd;                                          // PNU : zoom status for logging
extern uint8_t debug_cam_focus_cmd;                                         // PNU : focus status for logging
extern uint8_t debug_cam_record_cmd;                                        // PNU : record status for logging
extern uint8_t debug_cam_track_cmd;                                         // PNU : tracking status for logging
extern uint8_t debug_cam_ir_cmd;                                            // PNU : IR status for logging

extern mavlink_sys_icd_gcs_flcc_cam_cmd_t               PREV_CAM_CMD;       // PNU : previous CAM_CMD
extern mavlink_sys_icd_flcc_gcs_cam_attitude_status_t   CAM_ATTITUDE_STATUS;// PNU : MAVLINK message for CAM


// -------------------------------------------------------------------------
// Define Parameters for CAM
// #define CAM_UART                        hal.serial(4)       // Serial Port for CAM Interface (KAL)
// #define CAM_UART_BUFFER_SIZE            64                  // Serial Buffer Size (KAL)
// #define CAM_TRACK_UART_BUFFER_SIZE      48                  // Serial Buffer Size for Track (KAL)

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
    // Removed : void send_cmd_track_start();                                                        // Send "start_track" Command to CAM (KAL)
    // Removed : void send_cmd_track_end();                                                          // Send "end_track" Command to CAM (KAL)
    // Removed : void send_cmd_ir_color(uint8_t Color, uint8_t White, uint8_t checksum);             // Send "set ir_color" Command to CAM (KAL)
    // Removed : void send_cmd_eo_ir_mode(uint8_t mode, uint8_t checksum);                           // Send "set eo/ir_mode" Command to CAM (KAL)
    // Removed : void send_cmd_ir_digital_zoom(uint8_t ratio);                                       // Send "set ir zoom" Command to CAM (KAL)
    // Removed : void send_cmd_zoom(uint8_t zoom);                                                   // Send "set zoom" Command to CAM (KAL)
    // Removed : void send_cmd_focus(uint8_t focus);                                                 // Send "set focus" Command to CAM (KAL)
    // Removed : void send_cmd_shutter(uint8_t shutter);                                             // Send "set shutter" Command to CAM (KAL)
    void send_cmd_hold_angle(void);                                                     // Send "Hold angle" Command to CAM (KAL)

    // Removed : void get_cmd_angle() const;                                                         // Send "get_angle" Command to CAM (KAL)
    // Removed : void get_cmd_zoom() const;                                                          // Send "get_zoom" Command to CAM (KAL)

    void no_control_mode_operation(mavlink_sys_icd_gcs_flcc_cam_cmd_t cmd);             // Stop CAM & Stabilize (KAL)
    void IR_operation(mavlink_sys_icd_gcs_flcc_cam_cmd_t cam_cmd);                      // Control IR Functions (KAL)

    // Removed : int32_t receive_cam_uart_data(uint16_t* buffer) const;                              // Receive Data from CAM (KAL)

    // Removed : void parse_b1_feedback(uint16_t* buffer) const;
    // Removed : void parse_cam_angle(uint16_t* buffer) const;                                       // Parse the "angle" Data from CAM (KAL)
    // Removed : void parse_zoom_position(uint16_t* buffer) const;                                   // Parse the "zoom" Data from CAM (KAL)
    // Removed : void parse_zoom_value(uint16_t zoom_raw) const;

    // Removed : bool calc_angle_to_location(Vector3f& angles_to_target_rad);


    // ROI angle expend (KAL)
    // Removed : float pan_angle_calc(float pan_angle, bool new_loc);                                // Calculate pan angle cmd -2pi~2pi
    // Removed : float pan_angle_limit(float pan_angle, float pan_original, float pan_limit);        // Limit pan angle cmd accroding to gimbal spec

private:

    // -------------------------------------------------------------------------
    // Declare Functions to parse data with CAM
    // Removed : uint8_t get_cam_checksumX(uint8_t* buffer, int pos, int size) const;               // New Checksum calculation for New Viewpro Protocol
    // Removed : uint8_t get_cam_checksum(uint8_t* buffer, int pos, int size) const;                // Calculate Checksum (KAL)

    // Removed : float get_cam_angle_16(uint16_t* buffer) const;                                     // Decode CAM angle for 2byte buffer (KAL)
    // Removed : float get_cam_angle_32(uint16_t* buffer) const;                                     // Decode CAM angle for 4byte buffer (KAL)

    // Removed : uint8_t get_cam_angle_byte_l(int16_t angle);                                        // Encode angle to lower byte (KAL)
    // Removed : uint8_t get_cam_angle_byte_h(int16_t angle);                                        // Encode angle to upper byte (KAL)
    // Removed : uint8_t get_cam_speed_byte_l(int16_t speed);                                        // Encode speed to lower byte (KAL)
    // Removed : uint8_t get_cam_speed_byte_h(int16_t speed);                                        // Encode speed to upper byte (KAL)

    // PNU-ISSUE(D10) : report a camera command the mount declined.  Every camera call in
    // this file used to ignore its return value, so on any mount that is not a Viewpro the
    // whole KGCS camera path did nothing at all and said nothing about it.  Rate limited
    // because KGCS streams TC2 at ~10 Hz, so an unsupported command fails on every tick -
    // the first failure is always reported, as for the D6 PMU short-frame message.
    void report_cam_unsupported(const char *what);

    uint32_t _cam_unsupported_cnt = 0;      // total camera commands the mount declined
    uint32_t _cam_unsupported_last_ms = 0;  // system time the last report was sent

    uint8_t _primary_EOIR_source = 1;   //Main video source in PIP - 1: EO 2: IR
    float _Max_zoom_EO = 30.0;  //Maximum zoom level of EO
    float _Max_zoom_IR = 4.0;   //Maximum zoom level of IR
    // PNU-ISSUE(D10) : these two are write-only - assigned below and read nowhere in the
    // tree, the same situation as the D6 PMU counters.  Kept rather than deleted so the
    // keep-or-remove decision stays with PNU; they now hold their last known value when
    // the mount cannot report zoom, instead of being overwritten with a fabricated 0.
    uint8_t _current_zoom_EO = 1;
    uint8_t _current_zoom_IR = 1;

};

namespace AP {
    AP_Q30 *Q30();
};

#endif  // AP_Q30_ENABLED
