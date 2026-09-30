#pragma once

#include <AP_Mount/AP_Mount_config.h>

// PNU-ISSUE(D13) : AP_Q30 drives the camera entirely through AP::mount(), which is
// declared only inside #if HAL_MOUNT_ENABLED.  Following that flag by default means a
// MOUNT-disabled build compiles with the KGCS camera path simply absent, instead of
// failing on a pile of unknown-type errors.  Override to 0 to drop AP_Q30 while keeping
// the mount.
#ifndef AP_Q30_ENABLED
#define AP_Q30_ENABLED HAL_MOUNT_ENABLED
#endif
