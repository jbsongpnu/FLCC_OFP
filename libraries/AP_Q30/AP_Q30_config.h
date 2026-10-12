#pragma once

#include <AP_Mount/AP_Mount_config.h>
#include <AP_Camera/AP_Camera_config.h>

// PNU-ISSUE(D13) : AP_Q30 drives the gimbal through AP::mount() and the camera through
// AP::camera(), which are declared only inside #if HAL_MOUNT_ENABLED and
// #if AP_CAMERA_ENABLED respectively.  Following both flags by default means a build with
// either disabled compiles with the KGCS camera path simply absent, instead of failing on
// a pile of unknown-type errors.  Override to 0 to drop AP_Q30 while keeping both.
// PNU-ISSUE(D10) : AP_CAMERA_ENABLED was added to this condition by Stage 2, when the
// camera calls moved off AP::mount().
#ifndef AP_Q30_ENABLED
#define AP_Q30_ENABLED (HAL_MOUNT_ENABLED && AP_CAMERA_ENABLED)
#endif
