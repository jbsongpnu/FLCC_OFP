#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_CoaxCAN1.h"
#include <AP_CANManager/AP_CANManager.h>

AP_COAXCAN1::AP_COAXCAN1()
{
    _initialized    = false;
    _examp.set_default(0);
}

AP_COAXCAN1::~AP_COAXCAN1()
{    
}