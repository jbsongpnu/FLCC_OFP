/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  Camera driver for cameras that implement the newer MAVLink camera v2 protocol
  see https://mavlink.io/en/services/camera.html
 */
#pragma once

// PNU : set to 1 to report the camera's CAMERA_SETTINGS zoom and focus to the GCS every
// 2 s, as "CAMdbg: zoom <z> focus <f> age <n>ms".  Off by default - this is a bench aid,
// not flight output.  The decode itself is permanent (D10 Stage 3 feeds TM2 from it), so
// only the reporting is conditional and enabling this changes nothing else.
//
// It answered two questions on the Gremsy VIO bench and is kept for the next payload:
// whether zoomLevel carries a percentage or zoom times, and whether it follows the active
// EO/IR source.  The "age" field is the point - it distinguishes a live reading from a
// stale one, which a raw value cannot.  Same spirit as GCS_VIEWPRO_IR_DEBUG in GCS_PNU.cpp.
#define AP_CAMERA_MAVLINKCAMV2_ZOOM_DEBUG 0

#include "AP_Camera_Backend.h"

#if AP_CAMERA_MAVLINKCAMV2_ENABLED

class AP_Camera_MAVLinkCamV2 : public AP_Camera_Backend
{
public:

    // Constructor
    using AP_Camera_Backend::AP_Camera_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_MAVLinkCamV2);

    // update - should be called at 50hz
    void update() override;

    // entry point to actually take a picture
    bool trigger_pic() override;

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg) override;

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    // send camera settings message to GCS.  Overridden to forward what the camera actually
    // reported instead of the base class's NaN placeholders. (PNU)
    void send_camera_settings(mavlink_channel_t chan) const override;

    // PNU : get the zoom level the camera last reported in CAMERA_SETTINGS.  Returns false
    // if none has been received or the last one is stale.
    //
    // NOTE the units are deliberately NOT interpreted here.  The MAVLink spec defines
    // CAMERA_SETTINGS.zoomLevel as a percentage of full range, but cameras exist that
    // report zoom times instead, so this reports the raw value and leaves the meaning to
    // the caller, which is the only place that knows which camera is fitted.
    bool get_reported_zoom_level(float &zoom_level) const override;

private:

    // search for camera in GCS_MAVLink routing table
    void find_camera();

    // request CAMERA_INFORMATION (holds vendor and model name)
    void request_camera_information() const;

    // PNU : request CAMERA_SETTINGS (holds zoomLevel and focusLevel)
    void request_camera_settings() const;

#if AP_CAMERA_MAVLINKCAMV2_ZOOM_DEBUG
    uint32_t _dbg_last_report_ms = 0;   // system time of the last CAMdbg report
#endif

    // internal members
    bool _initialised;          // true once the camera has provided a CAMERA_INFORMATION
    bool _got_camera_info;      // true once camera has provided CAMERA_INFORMATION
    mavlink_camera_information_t _cam_info {}; // latest camera information received from camera
    uint32_t _last_caminfo_req_ms;  // system time that CAMERA_INFORMATION was last requested (used to throttle requests)

    // PNU : latest CAMERA_SETTINGS.  The camera answers a request rather than streaming
    // this, so it is polled; _cam_settings_ms == 0 means none has ever arrived.
    mavlink_camera_settings_t _cam_settings {};
    uint32_t _cam_settings_ms = 0;          // system time the last CAMERA_SETTINGS arrived
    uint32_t _last_camsettings_req_ms = 0;  // system time CAMERA_SETTINGS was last requested
    class GCS_MAVLINK *_link;   // link we have found the camera on. nullptr if not seen yet
    uint8_t _sysid;             // sysid of camera
    uint8_t _compid;            // component id of gimbal
};

#endif // AP_CAMERA_MAVLINKCAMV2_ENABLED
