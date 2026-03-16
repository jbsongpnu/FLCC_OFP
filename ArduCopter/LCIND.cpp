#include "LCIND.h"

extern const AP_HAL::HAL& hal;

LCIND_class::LCIND_class()
{
    if (_singleton) {
        return;
    }
    _singleton = this;
}

LCIND_class *LCIND_class::_singleton = nullptr;
LCIND_class *LCIND_class::get_singleton()
{
    return _singleton;
}

void LCIND_class::process_stream(const uint8_t* incoming, uint16_t length, uint8_t device) {
    //Check device number
    if(device > 3) {
        return;
    }
    //Byte0~Byte1 : Header1 : ST=Stabilized, OL=Overload, US=weight stab
    //Byte2 : ','
    //Byte3~4 : Header2 : NT=Net-weight, GS=Gross-weight
    //Byte5 : ','
    if(length < 16) {
        return;
    }
    if((incoming[0] == 'S') && (incoming[1] == 'T')) {
        if((incoming[2] != ',') || (incoming[5] != ',')) {
            // gcs().send_text(MAV_SEVERITY_INFO, "Invalid comma location %u", device);
            return;
        }
        // --- DEBUG CODE BLOCK
        // if((incoming[3] == 'N') && (incoming[4] == 'T')) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "got header ST NT : len %u, device %u", length, device);
        // }else if ((incoming[3] == 'G') && (incoming[4] == 'S')) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "got header ST GS : len %u, device %u", length, device);
        // }
        // gcs().send_text(MAV_SEVERITY_INFO, "got header ST : len %u, device %u", length, device);
        // --- DEBUG CODE BLOCK
    }
    // --- DEBUG CODE BLOCK
    //else if ((incoming[0] == 'O') && (incoming[1] == 'L')) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Overloaded device %u", device);
    // }else if ((incoming[0] == 'U') && (incoming[1] == 'S')) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Header1=US at device %u", device);
    // }
    // --- DEBUG CODE BLOCK

    //Byte6 : + or - sign
    //Byte7~13 : weight number 0000.00
    //Byte14~17 : CR LF => no need to check
    int8_t sign = 1;
    float temp_weight = 0.0;
    if (incoming[6] == '+') {
        sign = 1;
    } else if (incoming[6] == '-') {
        sign = -1;
    } else {
        return;
    }
    uint32_t intPart = 0;
    uint32_t fracPart = 0;
    uint32_t fracDiv = 1;
    bool dotFound = false;

    for (int8_t i = 7; i < 14; i++) {
        uint8_t c = incoming[i];

        if (c == '.') {
            if (dotFound) {
                // if there's more than one dot, return false
                return;
            }
            dotFound = true;
        }
        else if (c >= '0' && c <= '9') {
            uint32_t digit = (uint32_t)(c - '0');

            if (!dotFound) {
                intPart = intPart * 10U + digit;
            } else {
                fracPart = fracPart * 10U + digit;
                fracDiv *= 10U;
            }
        }
        else {
            // only dot and numbers 0~9 are allowed
            return;
        }
    }
    temp_weight = (float)intPart;

    if (fracDiv > 1U) {
        temp_weight += ((float)fracPart / (float)fracDiv);
    }

    temp_weight *= (float)sign;
    weight[device] = temp_weight;
    valid[device] = true;
    // --- DEBUG CODE BLOCK
    // gcs().send_text(MAV_SEVERITY_INFO, "Weight : %f device %u", temp_weight, device);
}

namespace AP 
{
LCIND_class *LCIND_g()
{
    return LCIND_class::get_singleton();

}
};