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

#if HAL_NMEA_OUTPUT_ENABLED

#include <AP_Math/definitions.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_GPS/AP_GPS.h> // JBS - 23.11.09

#include <stdio.h>
#include <time.h>

AP_NMEA_Output::AP_NMEA_Output()
{
    AP_SerialManager& sm = AP::serialmanager();

    for (uint8_t i = 0; i < ARRAY_SIZE(_uart); i++) {
        _uart[i] = sm.find_serial(AP_SerialManager::SerialProtocol_NMEAOutput, i);

        if (_uart[i] == nullptr) {
            break;
        }
        _num_outputs++;
    }
}

AP_NMEA_Output* AP_NMEA_Output::probe()
{
    AP_NMEA_Output *ret = new AP_NMEA_Output();
    if (ret == nullptr || ret->_num_outputs == 0) {
        delete ret;
        return nullptr;
    }
    return ret;
}

uint8_t AP_NMEA_Output::_nmea_checksum(const char *str)
{
    uint8_t checksum = 0;
    const uint8_t* bytes = (const uint8_t*) str;

    for (uint16_t i = 1; str[i]; i++) {
        checksum ^= bytes[i];
    }

    return checksum;
}

void AP_NMEA_Output::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    AP_GPS *gps = AP_GPS::get_singleton();  //Prepare GPS class, 
    if(gps == nullptr)return;               //return if GPS is not available - JBS - 23.11.09

    // only send at NMEA_UPDATE_RATE_HZ - JBS - 23.11.09
    if ((now_ms - _last_run_ms) < (1000/NMEA_UPDATE_RATE_HZ)) { //JBS - 23.11.09
        return;
    }
    _last_run_ms = now_ms;

    // get time and date
    uint64_t time_usec;
    if (!AP::rtc().get_utc_usec(time_usec)) {
        return;
    }

    // not completely accurate, our time includes leap seconds and time_t should be without
    const time_t time_sec = time_usec / 1000000;
    struct tm* tm = gmtime(&time_sec);

    // format time string
    char tstring[11];
    snprintf(tstring, sizeof(tstring), "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (time_usec % 1000000) * 1.0e-6);

    // format date string
    char dstring[7];
    snprintf(dstring, sizeof(dstring), "%02u%02u%02u", tm->tm_mday, tm->tm_mon+1, tm->tm_year % 100);

    auto &ahrs = AP::ahrs();

    // get location (note: get_location from AHRS always returns true after having GPS position once)
    Location loc;
    bool pos_valid = ahrs.get_location(loc);

    // format latitude
    char lat_string[13];
    double deg = fabs(loc.lat * 1.0e-7f);
    double min_dec = ((fabs(loc.lat) - (unsigned)deg * 1.0e7)) * 60 * 1.e-7f; 
    snprintf(lat_string,
             sizeof(lat_string),
             "%02u%08.5f,%c",
             (unsigned) deg,
             min_dec,
             loc.lat < 0 ? 'S' : 'N');

    // format longitude
    char lng_string[14];
    deg = fabs(loc.lng * 1.0e-7f);
    min_dec = ((fabs(loc.lng) - (unsigned)deg * 1.0e7)) * 60 * 1.e-7f; 
    snprintf(lng_string,
             sizeof(lng_string),
             "%03u%08.5f,%c",
             (unsigned) deg,
             min_dec,
             loc.lng < 0 ? 'W' : 'E');

    // format GGA message with real data and Drone ID - JBS - 23.11.09
    char* gga = nullptr;
    uint16_t HDOP = gps->get_hdop();	//get HDOP in cm - JBS - 23.11.09
    uint16_t NoSV = gps->num_sats();	//get number of satellites in view - JBS - 23.11.09
    int16_t gga_res = asprintf(&gga,
                               "$GPGGA,%s,%s,%s,%01d,%d,%d.%02d,%.2f,M,0.0,M,,PNUDRONE,", //Drone ID : Embed C4NM0008084 or C4NM0008820 for Jeju
                               tstring,
                               lat_string,
                               lng_string,
                               pos_valid ? 1 : 0,
                               NoSV,
                               HDOP/100,
                               HDOP%100,
                               loc.alt * 0.01f);
    if (gga_res == -1) {
        return;
    }
    char gga_end[6];
    snprintf(gga_end, sizeof(gga_end), "*%02X\r\n", (unsigned) _nmea_checksum(gga));

    // get speed and heading
    Vector2f speed = ahrs.groundspeed_vector();
    float speed_mps = norm(speed.x, speed.y);//get speed in m/sec - JBS - 23.11.09
    float speed_kph = speed_mps * 3.6;//convert speed to km/hour - JBS - 23.11.09
    float speed_knots = speed_mps * M_PER_SEC_TO_KNOTS;//convert speed to knots - JBS - 23.11.09
    float course_heading = wrap_360(degrees(atan2f(speed.x, speed.y)));//get course heading - JBS - 23.11.09
    float mag_heading = wrap_360(degrees(ahrs.get_yaw()));//get magnetic heading - JBS - 23.11.09

    // format VTG message instead of RMC - JBS - 23.11.09
    char* vtg = nullptr;
    int16_t vtg_res = asprintf(&vtg,
                               "$GPVTG,%s,%.1f,T,%.1f,M,%.1f,N,%.1f,K,",
                               tstring,
                               course_heading,
                               mag_heading,
                               speed_knots,
                               speed_kph);
    if (vtg_res == -1) {    //JBS - 23.11.09
        free(gga);
        return;
    }
    char vtg_end[6];    //JBS - 23.11.09
    snprintf(vtg_end, sizeof(vtg_end), "*%02X\r\n", (unsigned) _nmea_checksum(vtg)); //JBS - 23.11.09

    const uint32_t space_required = strlen(gga) + strlen(gga_end) + strlen(vtg) + strlen(vtg_end); //JBS - 23.11.09

    // send to all NMEA output ports
    for (uint8_t i = 0; i < _num_outputs; i++) {
        if (_uart[i]->txspace() < space_required) {
            continue;
        }

        _uart[i]->write(gga);
        _uart[i]->write(gga_end);

        _uart[i]->write(vtg);       //JBS - 23.11.09
        _uart[i]->write(vtg_end);   //JBS - 23.11.09
    }

    free(gga);
    free(vtg);  //JBS - 23.11.09
}

#endif  // HAL_NMEA_OUTPUT_ENABLED
