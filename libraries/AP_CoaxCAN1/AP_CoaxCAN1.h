#ifndef AP_COAXCAN1_H_
#define AP_COAXCAN1_H_

#include <AP_Common/AP_Common.h>
#include <AP_CANManager/AP_CANDriver.h>

#include <AP_Param/AP_Param.h>

class AP_COAXCAN1 : AP_CANDriver {
public:

    AP_COAXCAN1();
    ~AP_COAXCAN1();

    static const struct AP_Param::GroupInfo var_info[];

    // Return coaxcan1 from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_COAXCAN1 *get_coaxcan1(uint8_t driver_index);	

private:


    bool _initialized;

    AP_Int8 _examp;

    uint8_t _driver_index;

};

#endif