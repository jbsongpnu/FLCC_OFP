# VIO Payload MAVLink Reference (from Gremsy PayloadSDK 3.2.0 source)

Source of truth: `libs/payloadSdkInterface.cpp`, `libs/payload-define/vio_sdk.h`, `libs/detection_packet.h`, `examples/`. Overrides `Mavlink_Command_List_v2.0.0.pdf` where they differ.

## Link
- MAVLink v2, dialect `ardupilotmega` (includes `common`).
- UDP to `192.168.55.1:14566`, or UART 115200 8N1.
- Our IDs: sys=1, comp=193 (`ONBOARD_COMPUTER3`).
- Targets (sys=1 unless learned otherwise). Learn sysid/compid from any received msg:
  - **P** = payload app, comp=26 (`USER2`). Fixed.
  - **C** = camera, comp in 100–105. Default 101.
  - **G** = gimbal, comp 154 or 171–175. Default 154.
- Ready = first msg received from C or G.
- On first msg from P: send `SET_MESSAGE_INTERVAL` for telemetry (see §4).

## 1. PARAM_EXT_SET (323) → C: camera/payload settings
- `param_id` = string below.
- `param_type` = 5 (`MAV_PARAM_EXT_TYPE_UINT32`).
- `param_value[128]` = raw uint32, little-endian, in bytes 0–3 (binary, NOT ASCII text, despite the PDF). Use memcpy. The SDK uses strcpy, which truncates values whose low byte is 0.
- Reply: `PARAM_EXT_ACK` (324): `param_result` 0=accepted.
- Read back: `PARAM_EXT_REQUEST_READ` (320) (`param_id`, `param_index=-1`), or `PARAM_EXT_REQUEST_LIST` (321).
  - Reply: `PARAM_EXT_VALUE` (322), value in the same uint32 encoding.

| param_id | Values |
|---|---|
| C_SOURCE (view) | 0 EO+IR, 1 EO, 2 IR, 3 IR+EO, 4 sync, 6 side-by-side |
| C_V_REC (rec src) | 0 both, 1 EO, 2 IR, 5 OSD |
| OSD_MODE | 0 off, 1 debug, 2 status |
| C_V_FLIP | 2 on, 3 off |
| C_T_PALETTE | 0–9 (F1: WhiteHot, BlackHot, Rainbow, RainbowHC, Ironbow, Lava, Arctic, Globow, Gradedfire, Hottest) |
| C_T_ZOOM (IR digital) | 0–7 = 1x–8x |
| C_T_G (IR gain) | 0 high (-50~150°C), 1 low (-50~550°C) |
| C_T_ISO_U | 0 K, 1 °C |
| C_T_ISO_M (isotherm) | 0 off, 1 on |
| C_T_RG{n}_CLR_M / _TMP / _CLR_MN / _CLR_MX | isotherm region n=0–5: mode / temp / color low / color high (n=5 has no _TMP) |
| C_V_ZM_MODE | 0 combine, 2 super-resolution |
| C_V_ZM_CB_LV (combine zoom) | idx 0–21 = 1,2,4,6,8,10,12,14,16,18,20,40,60,80,100,120,140,160,180,200,220,240x |
| C_V_ZM_SR_LV (SR zoom) | idx 0–11 = 1,2,4,6,8,10,12,14,16,18,20,30x |
| C_V_Z_SPD / C_V_F_SPD | zoom / focus speed 0–7 |
| C_V_FREEZE | 2 on, 3 off |
| C_V_DEFOG | 2 on, 3 off |
| C_V_DEFOG_LV | 0 lowest … 3 high |
| C_V_HS (high sensitivity) | 2 on, 3 off |
| C_V_AE (exposure) | 0 auto, 3 manual, 10 shutter-pri, 11 iris-pri, 13 bright, 14 gain |
| C_V_SP (shutter) | 6=1/1, 7=2/3, 8=1/2, 9=1/3, 10=1/4, 11=1/6, 12=1/8, 13=1/10, 14=1/15, 15=1/20, 16=1/30, 17=1/50, 18=1/60, 19=1/90, 20=1/100, 21=1/125, 22=1/180, 23=1/250, 24=1/350, 25=1/500, 26=1/725, 27=1/1000, 28=1/1500, 29=1/2000, 30=1/3000, 31=1/4000, 32=1/6000, 33=1/10000 |
| C_V_MinSP | min shutter, same code table, 13–29 |
| C_V_IrP (aperture) | 25 F2.0, 24 F2.2, 23 F2.4, 22 F2.6, 21 F2.8, 20 F3.1, 19 F3.4, 17 F4.0, 14 F5.2, 11 F6.8, 10 F7.3, 8 F8.7, 7 F9.6, 6 F10, 5 F11, 0 close |
| C_V_GAIN_HS | (HS on) 1–17 = 0–48 dB, 3 dB steps |
| C_V_GAIN_LS | (HS off) 1–13 = 0–36 dB, 3 dB steps |
| C_V_BrP_HS / C_V_BrP_LS | brightness 0–41 / 0–37 |
| C_V_WB | 0 auto, 1 indoor, 2 outdoor, 3 one-push, 4 ATW, 5 manual |
| C_V_RGAIN / C_V_BGAIN | 0–255 (WB manual) |
| C_V_FM (focus mode) | 0 manual, 1 AF, 2 one-push AF |
| C_V_FV (manual focus) | 0–61440 |
| C_V_ICR | 2 auto, 3 manual |
| C_V_ICR_THR | 0–255 (auto) |
| C_V_ICR_MAN | 2 on, 3 off |
| TRACK_MODE | 0 object tracking, 1 object detection |
| RC_MODE | 0 Gremsy, 1 standard |
| LRF_MODE | 0 1Hz, 1 4Hz, 2 10Hz, 3 off |
| GB_MODE (legacy, prefer §3) | 0 off, 1 lock, 2 follow, 3 mapping, 4 reset |

## 2. COMMAND_LONG (76): `confirmation=1`; reply `COMMAND_ACK` (77) {command, result(MAV_RESULT), progress}
| Function | Tgt | command | params (unlisted = 0) |
|---|---|---|---|
| Camera mode | C | SET_CAMERA_MODE 530 | p2: 0 image, 1 video |
| Capture start | C | IMAGE_START_CAPTURE 2000 | p2: interval s (0 = single) |
| Capture stop | C | IMAGE_STOP_CAPTURE 2001 | – |
| Record start/stop | C | VIDEO_START_CAPTURE 2500 / VIDEO_STOP_CAPTURE 2501 | – |
| Zoom | C | SET_CAMERA_ZOOM 531 | p1 type: 0 step, 1 continuous, 2 range. p2: step/cont = -1 out, 0 stop, 1 in; range = 0–100 % |
| Zoom to level | P | USER_4 31013 | p1=2, p2=0, p3=4, p4=target zoom x (e.g. 13.5) |
| Focus | C | SET_CAMERA_FOCUS 532 | p1 type: 1 continuous (p2 -1 out / 0 stop / 1 in), 4 auto |
| Get camera info | C | REQUEST_CAMERA_INFORMATION 521 | → CAMERA_INFORMATION |
| Get settings | C | REQUEST_CAMERA_SETTINGS 522 | → CAMERA_SETTINGS |
| Get storage | C | REQUEST_STORAGE_INFORMATION 525 | → STORAGE_INFORMATION |
| Get capture status | C | REQUEST_CAMERA_CAPTURE_STATUS 527 | → CAMERA_CAPTURE_STATUS |
| Get stream info | C | REQUEST_VIDEO_STREAM_INFORMATION 2504 | p1 stream_id → VIDEO_STREAM_INFORMATION |
| Get FOV | C | REQUEST_MESSAGE 512 | p1=271, p2 cam: 0 EO, 1 IR → CAMERA_FOV_STATUS |
| Get model/SW/serial | P | REQUEST_MESSAGE 512 | p1=397 → COMPONENT_INFORMATION_BASIC |
| Telemetry rate | P | SET_MESSAGE_INTERVAL 511 | p1 = param index (§4, Gremsy-specific, not a msg id), p2 = interval µs, p7=1 |
| IR FFC mode | P | USER_4 31013 | p1=2, p2=6, p3: 0 manual, 1 auto |
| IR FFC trigger | P | USER_4 | p1=2, p2=7 |
| WB one-push trigger | P | USER_4 | p1=2, p2=8 |
| Tracking start/stop | P | USER_4 | p1=4, p2=0, p3=0, p4: 0 stop, 1 active, 2 EagleEyes |
| Tracking target | P | USER_4 | p1=4, p2=0, p3=1, p4..p7 = x, y, w, h (px) |
| Stream bitrate | P | USER_4 | p1=4, p2=2, p3 cam (1 EO, 2 IR), p4=0, p5 = bps (512000–16000000) |
| Stream resolution | P | USER_4 | p1=4, p2=2, p3 cam, p4=1, p5 level (EO 0–2: 1920x1080…960x540; IR 0–4: 1280x1024…160x128) |
| Stream encoder profile | P | USER_4 | p1=4, p2=2, p3 cam, p4=2, p5: 0 low-latency … 2 high-perf |
| Standby | P | USER_4 | p1=4, p2=3, p4: 1 standby, 0 wake |
| Restart app | P | USER_4 | p1=4, p2=4, p3: 0 payload, 1 streaming, 2 tracking, 3 web UI, 4 gimbal |
| Spot AE display | P | USER_4 | p1=4, p2=5, p3=0, p4: 1 on, 0 off |
| Spot AE mode | P | USER_4 | p1=4, p2=5, p3=1, p4: 2 on, 3 off |
| Spot AE position | P | USER_4 | p1=4, p2=5, p3=2, p4..p7 = x, y, w, h (example: 6,6,2,2) |
| Gimbal calib gyro / accel | G | GIMBAL_REQUEST_AXIS_CALIBRATION 42503 | p7: 1 gyro, 2 accel |
| Gimbal calib motor / search home | G | DO_SET_HOME 179 | p7: 1 motor, 2 search home |
| Gimbal auto-tune | G | USER_3 31012 | p7: 1 start, 0 stop |
| Gimbal move (legacy v1) | G | DO_MOUNT_CONTROL 205 | p1 pitch, p2 roll, p3 yaw (deg or deg/s), p6: 1 angle, 2 speed, p7=2 |

Workflow notes:
- Tracking: set `C_SOURCE=1` and `TRACK_MODE=0` (optionally `OSD_MODE=2`). Then send the target, then start.
- Detection: set `TRACK_MODE=1`; results arrive in `V2_EXTENSION` (§3).

## 3. GIMBAL_DEVICE_SET_ATTITUDE (284) → G (Gimbal Protocol v2)
Always start from the current `flags` reported in `GIMBAL_DEVICE_ATTITUDE_STATUS`.

| Use | flags | q[4] | angular_velocity_x/y/z |
|---|---|---|---|
| Move to angle | current | from euler (rad); roll, yaw ±180°, pitch ±90° | NaN |
| Move by speed | current | NaN | roll, pitch, yaw (rad/s) |
| Set mode | new flags | NaN | 0 |

Mode flags (bits of flags):
- RETRACT=1 (motors off)
- NEUTRAL=2 (reset/home)
- YAW_LOCK=16
- 0x4000 = Gremsy mapping

| Mode | How to set |
|---|---|
| LOCK | set YAW_LOCK, clear 0x4000, NEUTRAL, RETRACT |
| FOLLOW | clear YAW_LOCK, 0x4000, NEUTRAL, RETRACT |
| OFF | set RETRACT, clear 0x4000, NEUTRAL |
| ON | clear RETRACT |
| HOME | set NEUTRAL, clear RETRACT and 0x4000. Send, then about 100 ms later clear NEUTRAL and send again |
| MAPPING | set 0x4000 |

Gimbal params: `PARAM_SET` (23), `PARAM_REQUEST_READ` (20), `PARAM_REQUEST_LIST` (21) → G. Reply: `PARAM_VALUE` (22).

## 4. Payload → us (receive)
| Msg (id) | From | Fields used / meaning |
|---|---|---|
| HEARTBEAT (0) | all | liveness; source of sys/comp ids |
| COMMAND_ACK (77) | C, P, G | command, result, progress |
| PARAM_EXT_ACK (324) | C | param_id, param_result |
| PARAM_EXT_VALUE (322) | C | param_id, param_index, value (uint32 in bytes 0–3) |
| CAMERA_INFORMATION (259) | C | flags = CAMERA_CAP_FLAGS (see below) |
| CAMERA_SETTINGS (260) | C | mode_id (0 image, 1 video), zoomLevel, focusLevel |
| STORAGE_INFORMATION (261) | C | total/used/available_capacity (MiB), status |
| CAMERA_CAPTURE_STATUS (262) | C | image_status, video_status, image_count, recording_time_ms |
| VIDEO_STREAM_INFORMATION (269) | C | stream_id, type, resolution_h/v, bitrate, uri (RTSP) |
| CAMERA_FOV_STATUS (271) | C | camera_device_id, hfov, vfov (deg) |
| COMPONENT_INFORMATION_BASIC (397) | P | model_name, software_version, serial_number |
| GIMBAL_DEVICE_ATTITUDE_STATUS (285) | G | q → euler (roll, pitch, yaw), angular_velocity_xyz, flags (mode decode: YAW_LOCK→LOCK, RETRACT→OFF, NEUTRAL→RESET, 0x4000→MAPPING, else FOLLOW) |
| MOUNT_ORIENTATION (265) | G | pitch, roll, yaw (deg). Use yaw_absolute when YAW_LOCK is set |
| DISTANCE_SENSOR (132) | any | LRF: current_distance, horizontal_fov, vertical_fov |
| STATUSTEXT (253) | any | Record status JSON (see below) |
| PARAM_VALUE (22) / DEBUG (254) | P | Telemetry: `param_index`/`ind` = index below, `param_value`/`value` = float |
| PARAM_VALUE (22) / DEBUG (254) | G | Gimbal param values |
| V2_EXTENSION (248) | P | Detection boxes (see below) |

CAMERA_CAP_FLAGS (standard MAVLink bits):
- 1 CAPTURE_VIDEO
- 2 CAPTURE_IMAGE
- 4 HAS_MODES
- 8 IMAGE_IN_VIDEO_MODE
- 16 VIDEO_IN_IMAGE_MODE
- 32 IMAGE_SURVEY_MODE
- 64 HAS_BASIC_ZOOM
- 128 HAS_BASIC_FOCUS
- 256 HAS_VIDEO_STREAM (the SDK checks this before requesting the stream URI)
- 512 TRACKING_POINT
- 1024 TRACKING_RECTANGLE
- 2048 TRACKING_GEO_STATUS
- 4096 THERMAL_RANGE

The VIO value is set by firmware. The expected value is 0x1C7 (bits 1|2|4|64|128|256); this is unverified, so read it from the device. Tracking bits are likely 0, because VIO tracking uses USER_4 (§2), not the standard tracking commands.

Record status JSON (in STATUSTEXT):
- Reassembly: `id=0` is a single chunk. Otherwise collect by `id`, ordered by `chunk_seq`; the last chunk has length < 50.
- JSON keys: `{rec_src, rec_status(1 recording, 0 stopped), rec_time, timestamp}`.
- When stopped, also: `size`, `duration`, `url` (HTTP download).

Telemetry indexes. Request each with `SET_MESSAGE_INTERVAL` (examples use 100 ms for LRF/track, 1000 ms for others):

| Idx | Name | Idx | Name |
|---|---|---|---|
| 0 | EO_ZOOM | 14 | PAY_LAT |
| 1 | IR_ZOOM | 15 | PAY_ALT |
| 2 | LRF_RANGE | 16–18 | APP_VER_X/Y/Z |
| 3–6 | TRK_POS_X/Y/W/H | 19 | VIEW_MODE |
| 7 | TRK_STATUS = (mode<<8) \| status (status: 0 idle, 1 tracked, 2 lost) | 20 | REC_SRC |
| 8–9 | LRF_OFFSET_X/Y | 21 | IR_TYPE |
| 10–12 | TARGET_LON/LAT/ALT | 22 | PALETTE_ID |
| 13 | PAY_LON | 23 | FFC_MODE |
| 24 | GB_MODE | 25–27 | IR_TEMP_MAX/MIN/MEAN |

Detection boxes in `V2_EXTENSION.payload`:
- `message_type` = 0x4001.
- Packed little-endian layout:
  - Header (8 B): `magic u16 = 0x4744`, `version u8 = 1`, `num_boxes u8 ≤ 20`, `frame_id u32`.
  - Then `num_boxes` × 12 B: `x, y, w, h u16` (top-left, px in 1920x1080 space), `track_id u16` (0 = none), `class_id u8`, `confidence u8` (/255).
- Drop the packet if magic or version doesn't match.

## 5. Us → payload (data streams)
| Msg (id) | Purpose / fields |
|---|---|
| GPS_RAW_INT (24) | 10 Hz. fix_type, lat/lon (deg·1e7), alt and alt_ellipsoid (mm), eph (HDOP·100), vel (cm/s), cog (cdeg), satellites_visible |
| GLOBAL_POSITION_INT (33) | 10 Hz, only when fix is good. time_boot_ms, lat/lon (·1e7), alt (mm), relative_alt, hdg |
| SYSTEM_TIME (2) | time_unix_usec, time_boot_ms (payload clock sync) |

Not MAVLink:
- Video: RTSP (port 8554). URI comes from VIDEO_STREAM_INFORMATION.
- Media download: HTTP `http://<ip>:8000`, login via `/smb-login`.
