#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_CoaxCAN.h"
#include <AP_CANManager/AP_CANManager.h>

#define TEMP_EXP_INITVAL 0		//Initial value

// JBSong - table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_COAXCAN::var_info[] = {
    // @Param: example
    // @DisplayName: User example
    // @Description: Example for user
    AP_GROUPINFO("Examp", 1, AP_COAXCAN, _examp, TEMP_EXP_INITVAL),
    //AP_GROUPINFO("Examp", 1, AP_COAXCAN, TEMP_EXP_INITVAL, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM1", 1, AP_COAXCAN, _pmu_param1, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM2", 2, AP_COAXCAN, _pmu_param2, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM3", 3, AP_COAXCAN, _pmu_param3, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM4", 4, AP_COAXCAN, _pmu_param4, TEMP_EXP_INITVAL),

    AP_GROUPEND
};

AP_COAXCAN::AP_COAXCAN()
{
    _initialized    = false;
    _examp.set_default(0);
}

AP_COAXCAN::~AP_COAXCAN()
{    
}

// -------------------------------------------------------------------------
// get_coaxcan : Return coaxcan from @driver_index or nullptr if it's not ready or doesn't exist
//'Driver_Type_CoaxCAN' : defined in AP_CANManager.h 
// -------------------------------------------------------------------------
AP_COAXCAN *AP_COAXCAN::get_coaxcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::CoaxCAN) {
        return nullptr;
    }
    return static_cast<AP_COAXCAN*>(AP::can().get_driver(driver_index));
}