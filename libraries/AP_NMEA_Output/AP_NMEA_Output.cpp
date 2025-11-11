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


   Author: Francisco Ferreira (some code is copied from sitl_gps.cpp)

 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS
#define NMEA_UPDATE_RATE_HZ		1   // JBS - 23.11.09
#include "AP_NMEA_Output.h"

// #if HAL_NMEA_OUTPUT_ENABLED

#include <AP_Math/definitions.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>
#include <time.h>
#include <AP_AHRS/AP_AHRS_config.h>

#if AP_AHRS_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#endif

#ifndef AP_NMEA_OUTPUT_MESSAGE_ENABLED_DEFAULT
#define AP_NMEA_OUTPUT_MESSAGE_ENABLED_DEFAULT      3   // GPGGA and GPRMC
#endif

#define AP_NMEA_OUTPUT_INTERVAL_MS_MIN              10
#define AP_NMEA_OUTPUT_INTERVAL_MS_MAX              5000

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NMEA_Output::var_info[] = {

    // @Param: RATE_MS
    // @DisplayName: NMEA Output rate
    // @Description: NMEA Output rate. This controls the interval at which all the enabled NMEA messages are sent. Most NMEA systems expect 100ms (10Hz) or slower.
    // @Range: 20 2000
    // @Increment: 1
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("RATE_MS", 1, AP_NMEA_Output, _interval_ms, 100),

    // @Param: MSG_EN
    // @DisplayName: Messages Enable bitmask
    // @Description: This is a bitmask of enabled NMEA messages. All messages will be sent consecutively at the same rate interval
    // @Bitmask: 0:GPGGA,1:GPRMC,2:PASHR
    // @User: Standard
    AP_GROUPINFO("MSG_EN", 2, AP_NMEA_Output, _message_enable_bitmask, AP_NMEA_OUTPUT_MESSAGE_ENABLED_DEFAULT),

    AP_GROUPEND
};

void AP_NMEA_Output::init()
{
    const AP_SerialManager& sm = AP::serialmanager();

    _num_outputs = 0;
    for (uint8_t i = 0; i < ARRAY_SIZE(_uart); i++) {
        _uart[i] = sm.find_serial(AP_SerialManager::SerialProtocol_NMEAOutput, i);

        if (_uart[i] == nullptr) {
            break;
        }
        _num_outputs++;
    }
}

void AP_NMEA_Output::update()
{
    // if (_num_outputs == 0 || _message_enable_bitmask == 0) { //New in 4.6.2 but disabling - JBS 20201104
    //     return;
    // }

    const uint32_t now_ms = AP_HAL::millis();

    // AP_GPS *gps = AP_GPS::get_singleton();  //Prepare GPS class, 
    // if(gps == nullptr)return;               //return if GPS is not available - JBS - 23.11.09

    //if ((now_ms - _last_run_ms) < static_cast<uint32_t>(MAX(_interval_ms.get(), 20))) {
    // only send at NMEA_UPDATE_RATE_HZ - JBS - 23.11.09
    if ((now_ms - _last_run_ms) < (1000/NMEA_UPDATE_RATE_HZ)) { //JBS - 23.11.09
        return;
    }
    _last_run_ms = now_ms;

    // get time and date
    uint64_t time_usec;
#if AP_RTC_ENABLED
    if (!AP::rtc().get_utc_usec(time_usec)) {
        return;
    }
#else
    time_usec = 0;
#endif

    uint32_t space_required = 0;

    // not completely accurate, our time includes leap seconds and time_t should be without
    const time_t time_sec = time_usec / 1000000;
    struct tm tmd {};
    struct tm* tm = gmtime_r(&time_sec, &tmd);

    // format time string
    char tstring[11];
    hal.util->snprintf(tstring, sizeof(tstring), "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (time_usec % 1000000) * 1.0e-6);

    Location loc;
    const auto &gps = AP::gps();    //This was added in 4.6.2. Use this.
    const AP_GPS::GPS_Status gps_status = gps.status();

#if AP_AHRS_ENABLED
    auto &ahrs = AP::ahrs();
    // NOTE: ahrs.get_location() always returns true after having GPS position once because it will be dead-reckoning
    const bool pos_valid = ahrs.get_location(loc);
#else
    const bool pos_valid = (gps_status >= AP_GPS::GPS_OK_FIX_3D);
    loc = gps.location();
#endif

    // format latitude
    char lat_string[13];
    double deg = fabs(loc.lat * 1.0e-7f);
    double min_dec = ((fabs(loc.lat) - (unsigned)deg * 1.0e7)) * 60 * 1.e-7f;
    hal.util->snprintf(lat_string,
            sizeof(lat_string),
            "%02u%08.5f,%c",
            (unsigned) deg,
            min_dec,
            loc.lat < 0 ? 'S' : 'N');

    // format longitude
    char lng_string[14];
    deg = fabs(loc.lng * 1.0e-7f);
    min_dec = ((fabs(loc.lng) - (unsigned)deg * 1.0e7)) * 60 * 1.e-7f; 
    hal.util->snprintf(lng_string,
            sizeof(lng_string),
            "%03u%08.5f,%c",
            (unsigned) deg,
            min_dec,
            loc.lng < 0 ? 'W' : 'E');

    // format GGA message with real data and Drone ID - JBS - 23.11.09
    char gga[100];
    uint16_t HDOP = gps.get_hdop();	//get HDOP in cm - JBS - 23.11.09
    uint16_t NoSV = gps.num_sats();	//get number of satellites in view - JBS - 23.11.09
    uint16_t gga_length = 0;
    if ((_message_enable_bitmask.get() & static_cast<int16_t>(Enabled_Messages::GPGGA)) != 0) {
        // format GGA message

        // Convert AP_GPS::GPS_Status:
        // 0 = NO_GPS
        // 1 = NO_FIX
        // 2 = GPS_OK_FIX_2D
        // 3 = GPS_OK_FIX_3D
        // 4 = GPS_OK_FIX_3D_DGPS
        // 5 = GPS_OK_FIX_3D_RTK_FLOAT
        // 6 = GPS_OK_FIX_3D_RTK_FIXED

        // To NMEA "Fix Quality" per Trimble definition:
        // 0: Fix not valid
        // 1: GPS fix
        // 2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
        // 3: Not applicable
        // 4: RTK Fixed, xFill
        // 5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
        // 6: INS Dead reckoning
        // sources:
        // https://orolia.com/manuals/VSP/Content/NC_and_SS/Com/Topics/APPENDIX/NMEA_GGAmess.htm
        // http://aprs.gids.nl/nmea/#gga
        // https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
        // https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html <-using this one

        uint8_t fix_quality;
        switch (gps_status) {
            default:
            case AP_GPS::NO_GPS:
            case AP_GPS::NO_FIX:
            case AP_GPS::GPS_OK_FIX_2D:
                // NOTE: ahrs.get_location() always returns pos_valid=true after having GPS position once because it will be dead-reckoning
                 fix_quality = pos_valid ? 6 : 0;
                break;
            case AP_GPS::GPS_OK_FIX_3D:
                fix_quality = 1;
                break;
            case AP_GPS::GPS_OK_FIX_3D_DGPS:
                fix_quality = 2;
                break;
            case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
                fix_quality = 5;
                break;
            case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
                fix_quality = 4;
                break;
        }
        // format GGA messages - JBS 23.11.09, 25.11.04
        gga_length = nmea_printf_buffer(gga, sizeof(gga),
                                    "$GPGGA,%s,%s,%s,%01d,%d,%d.%02d,%.2f,M,0.0,M,,PNUDRONE,", //Drone ID : Embed C4NM0008084 or C4NM0008820 for Jeju
                                    tstring,
                                    lat_string,
                                    lng_string,
                                    fix_quality,
                                    NoSV,
                                    HDOP * 0.01,
                                    HDOP%100,
                                    loc.alt * 0.01f);

        space_required += gga_length;
    }

    char vtg[100];
    uint16_t vtg_length = 0;
    if ((_message_enable_bitmask.get() & static_cast<int16_t>(Enabled_Messages::GPVTG)) != 0) {

        // get speed
        Vector2f speed = ahrs.groundspeed_vector();
        float speed_mps = norm(speed.x, speed.y);//get speed in m/sec - JBS - 23.11.09
        float speed_kph = speed_mps * 3.6;//convert speed to km/hour - JBS - 23.11.09
        float speed_knots = speed_mps * M_PER_SEC_TO_KNOTS;//convert speed to knots - JBS - 23.11.09
        float course_heading = wrap_360(degrees(atan2f(speed.x, speed.y)));//get course heading - JBS - 23.11.09
        float mag_heading = wrap_360(degrees(ahrs.get_yaw()));//get magnetic heading - JBS - 23.11.09

        // format VTG message instead of RMC - JBS - 23.11.09, 25.11.04
        vtg_length = nmea_printf_buffer(vtg, sizeof(vtg),
                                    "$GPVTG,%s,%.1f,T,%.1f,M,%.1f,N,%.1f,K,",
                                    tstring,
                                    course_heading,
                                    mag_heading,
                                    speed_knots,
                                    speed_kph);

        space_required += vtg_length;
    }

    // send to all NMEA output ports
    for (uint8_t i = 0; i < _num_outputs; i++) {
        if (_uart[i]->txspace() < space_required) {
            continue;
        }

        if (gga_length > 0) {
            _uart[i]->write(gga);
        }

        if (vtg_length > 0) {
            _uart[i]->write(vtg);
        }
    }
}


// #endif  // HAL_NMEA_OUTPUT_ENABLED
