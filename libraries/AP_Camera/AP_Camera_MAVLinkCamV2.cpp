#include "AP_Camera_MAVLinkCamV2.h"

#if AP_CAMERA_MAVLINKCAMV2_ENABLED
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// PNU : how often CAMERA_SETTINGS is requested, and how long a reported value stays valid.
// The camera answers a request rather than streaming the message, so it has to be polled;
// 1 Hz is ample for a zoom readback, which only changes when an operator drives it.
#define AP_CAMERA_MAVLINKCAMV2_SETTINGS_REQ_MS     1000
#define AP_CAMERA_MAVLINKCAMV2_SETTINGS_TIMEOUT_MS 3000

#define AP_CAMERA_MAVLINKCAMV2_SEARCH_MS    60000   // search for camera for 60 seconds after startup

// update - should be called at 50hz
void AP_Camera_MAVLinkCamV2::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_camera();
    }

    // call parent update
    AP_Camera_Backend::update();

    // PNU : keep CAMERA_SETTINGS fresh.  The camera answers a request rather than
    // streaming this message, so poll it once the camera has been found.
    if (_initialised) {
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_camsettings_req_ms >= AP_CAMERA_MAVLINKCAMV2_SETTINGS_REQ_MS) {
            _last_camsettings_req_ms = now_ms;
            request_camera_settings();
        }

#if AP_CAMERA_MAVLINKCAMV2_ZOOM_DEBUG
        // PNU : bench aid, off by default.  See the flag in the header.
        if (now_ms - _dbg_last_report_ms >= 2000) {
            _dbg_last_report_ms = now_ms;
            if (_cam_settings_ms == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAMdbg: no CAMERA_SETTINGS yet");
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAMdbg: zoom %.2f focus %.2f age %ums",
                              (double)_cam_settings.zoomLevel,
                              (double)_cam_settings.focusLevel,
                              (unsigned)(now_ms - _cam_settings_ms));
            }
        }
#endif
    }
}

// entry point to actually take a picture.  returns true on success
bool AP_Camera_MAVLinkCamV2::trigger_pic()
{
    // exit immediately if have not found camera or does not support taking pictures
    if (_link == nullptr || !(_cam_info.flags & CAMERA_CAP_FLAGS_CAPTURE_IMAGE)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.target_system = _sysid;
    pkt.target_component = _compid;
    pkt.command = MAV_CMD_IMAGE_START_CAPTURE;
    pkt.param3 = 1;             // number of images to take
    pkt.param4 = image_index+1; // starting sequence number

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Camera_MAVLinkCamV2::record_video(bool start_recording)
{
    // exit immediately if have not found camera or does not support recording video
    if (_link == nullptr || !(_cam_info.flags & CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.target_system = _sysid;
    pkt.target_component = _compid;

    if (start_recording) {
        pkt.command = MAV_CMD_VIDEO_START_CAPTURE;
        // param1 = 0, video stream id. 0 for all streams
        // param2 = 0, status frequency.  frequency that CAMERA_CAPTURE_STATUS messages should be sent while recording. 0 for no messages
    } else {
        pkt.command = MAV_CMD_VIDEO_STOP_CAPTURE;
        // param1 = 0, video stream id. 0 for all streams
    }

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// set zoom specified as a rate or percentage
bool AP_Camera_MAVLinkCamV2::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if have not found camera or does not support zoom
    if (_link == nullptr || !(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.target_system = _sysid;
    pkt.target_component = _compid;
    pkt.command = MAV_CMD_SET_CAMERA_ZOOM;
    switch (zoom_type) {
    case ZoomType::RATE:
        pkt.param1 = ZOOM_TYPE_CONTINUOUS;
        break;
    case ZoomType::PCT:
        pkt.param1 = ZOOM_TYPE_RANGE;
        break;
    }
    pkt.param2 = zoom_value;            // Zoom Value

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera_MAVLinkCamV2::set_focus(FocusType focus_type, float focus_value)
{
    // exit immediately if have not found camera or does not support focus
    if (_link == nullptr || !(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS)) {
        return SetFocusResult::FAILED;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.target_system = _sysid;
    pkt.target_component = _compid;
    pkt.command = MAV_CMD_SET_CAMERA_FOCUS;
    switch (focus_type) {
    case FocusType::RATE:
        // focus in, out or hold (focus in = -1, hold = 0, focus out = 1). Same as FOCUS_TYPE_CONTINUOUS
        pkt.param1 = FOCUS_TYPE_CONTINUOUS;
        break;
    case FocusType::PCT:
        // focus to a percentage (from 0 to 100) of the full range. Same as FOCUS_TYPE_RANGE
        pkt.param1 = FOCUS_TYPE_RANGE;
        break;
    case FocusType::AUTO:
        // focus automatically. Same as FOCUS_TYPE_AUTO
        pkt.param1 = FOCUS_TYPE_AUTO;
        break;
    }
    pkt.param2 = focus_value;

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return SetFocusResult::ACCEPTED;
}

// handle incoming mavlink message including CAMERA_INFORMATION
void AP_Camera_MAVLinkCamV2::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    // handle CAMERA_INFORMATION
    if (msg.msgid == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
        mavlink_msg_camera_information_decode(&msg, &_cam_info);

        const uint8_t fw_ver_major = _cam_info.firmware_version & 0x000000FF;
        const uint8_t fw_ver_minor = (_cam_info.firmware_version & 0x0000FF00) >> 8;
        const uint8_t fw_ver_revision = (_cam_info.firmware_version & 0x00FF0000) >> 16;
        const uint8_t fw_ver_build = (_cam_info.firmware_version & 0xFF000000) >> 24;

        // display camera info to user, once.  CAMERA_INFORMATION is normally sent in
        // response to a request, but a camera may also broadcast it periodically - the
        // Gremsy VIO Payload G1 does - and printing on every copy spams a statustext to
        // every connected link.  _got_camera_info was already being set here; it is now
        // read first so the message is emitted only the first time. (PNU)
        if (!_got_camera_info) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Camera: %.32s %.32s fw:%u.%u.%u.%u",
                    _cam_info.vendor_name,
                    _cam_info.model_name,
                    (unsigned)fw_ver_major,
                    (unsigned)fw_ver_minor,
                    (unsigned)fw_ver_revision,
                    (unsigned)fw_ver_build);
        }

        _got_camera_info = true;
    }

    // PNU : handle CAMERA_SETTINGS.  Nothing else in ArduPilot decodes this message -
    // elsewhere it is only ever sent outwards - so this is the whole of the inbound path.
    // zoomLevel and focusLevel are MAVLink2 extension fields and are legitimately NaN on a
    // camera that does not know them, which get_reported_zoom_level() rejects.
    if (msg.msgid == MAVLINK_MSG_ID_CAMERA_SETTINGS) {
        mavlink_msg_camera_settings_decode(&msg, &_cam_settings);
        _cam_settings_ms = AP_HAL::millis();
    }
}

// PNU : get the zoom level the camera last reported, in whatever units it reports them.
// See the declaration - the units are deliberately not interpreted here.
bool AP_Camera_MAVLinkCamV2::get_reported_zoom_level(float &zoom_level) const
{
    // never received
    if (_cam_settings_ms == 0) {
        return false;
    }

    // stale: the camera has stopped answering, so the last value is no longer evidence
    if (AP_HAL::millis() - _cam_settings_ms > AP_CAMERA_MAVLINKCAMV2_SETTINGS_TIMEOUT_MS) {
        return false;
    }

    // the field is an extension and may be NaN when the camera does not know its zoom
    if (isnan(_cam_settings.zoomLevel)) {
        return false;
    }

    zoom_level = _cam_settings.zoomLevel;
    return true;
}

// PNU : forward what the camera actually reported, rather than the base class's NaNs
void AP_Camera_MAVLinkCamV2::send_camera_settings(mavlink_channel_t chan) const
{
    if (_cam_settings_ms == 0) {
        // nothing received yet - let the base class send its "unknown" placeholders
        AP_Camera_Backend::send_camera_settings(chan);
        return;
    }

    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),           // time_boot_ms
        _cam_settings.mode_id,      // camera mode reported by the camera
        _cam_settings.zoomLevel,    // as reported, units not reinterpreted
        _cam_settings.focusLevel);
}

// send camera information message to GCS
void AP_Camera_MAVLinkCamV2::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if we have not yet received cam info
    if (!_got_camera_info) {
        return;
    }

    // send CAMERA_INFORMATION message
    mavlink_msg_camera_information_send(
        chan,
        AP_HAL::millis(),           // time_boot_ms
        _cam_info.vendor_name,      // vendor_name uint8_t[32]
        _cam_info.model_name,       // model_name uint8_t[32]
        _cam_info.firmware_version, // firmware version uint32_t
        _cam_info.focal_length,     // focal_length float (mm)
        _cam_info.sensor_size_h,    // sensor_size_h float (mm)
        _cam_info.sensor_size_v,    // sensor_size_v float (mm)
        _cam_info.resolution_h,     // resolution_h uint16_t (pix)
        _cam_info.resolution_v,     // resolution_v uint16_t (pix)
        _cam_info.lens_id,          // lens_id, uint8_t
        _cam_info.flags,            // flags uint32_t (CAMERA_CAP_FLAGS)
        _cam_info.cam_definition_version,   // cam_definition_version uint16_t
        _cam_info.cam_definition_uri,       // cam_definition_uri char[140]
        get_gimbal_device_id());    // gimbal_device_id uint8_t
}

// search for camera in GCS_MAVLink routing table
void AP_Camera_MAVLinkCamV2::find_camera()
{
    // do not look for camera for first 10 seconds so user may see banner
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms < 10000) {
        return;
    }

    // search for camera for 60 seconds or until armed
    if ((now_ms > AP_CAMERA_MAVLINKCAMV2_SEARCH_MS) && hal.util->get_soft_armed()) {
        return;
    }

    // search for a mavlink enabled camera
    if (_link == nullptr) {
        // we expect that instance 0 has compid = MAV_COMP_ID_CAMERA, instance 1 has compid = MAV_COMP_ID_CAMERA2, etc
        uint8_t compid = MIN(MAV_COMP_ID_CAMERA + _instance, MAV_COMP_ID_CAMERA6);
        _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_CAMERA, compid, _sysid);
        if (_link == nullptr) {
            // have not yet found a camera so return
            return;
        }
        _compid = compid;
    }

    // request CAMERA_INFORMATION
    if (!_got_camera_info) {
        if (now_ms - _last_caminfo_req_ms > 1000) {
            _last_caminfo_req_ms = now_ms;
            request_camera_information();
        }
        return;
    }

    _initialised = true;
}

// PNU : request CAMERA_SETTINGS (holds zoomLevel and focusLevel)
void AP_Camera_MAVLinkCamV2::request_camera_settings() const
{
    if (_link == nullptr) {
        return;
    }

    // MAV_CMD_REQUEST_MESSAGE is the current form.  A Gremsy VIO was bench-verified
    // answering both this and the superseded MAV_CMD_REQUEST_CAMERA_SETTINGS (522), so
    // only the modern one is sent.
    const mavlink_command_long_t pkt {
        MAVLINK_MSG_ID_CAMERA_SETTINGS, // param1
        0,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_REQUEST_MESSAGE,
        _sysid,
        _compid,
        0   // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
}

// request CAMERA_INFORMATION (holds vendor and model name)
void AP_Camera_MAVLinkCamV2::request_camera_information() const
{
    if (_link == nullptr) {
        return;
    }

    const mavlink_command_long_t pkt {
        MAVLINK_MSG_ID_CAMERA_INFORMATION,  // param1
        0,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_REQUEST_MESSAGE,
        _sysid,
        _compid,
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
}

#endif // AP_CAMERA_MAVLINKCAMV2_ENABLED
