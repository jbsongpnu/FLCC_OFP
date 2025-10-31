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