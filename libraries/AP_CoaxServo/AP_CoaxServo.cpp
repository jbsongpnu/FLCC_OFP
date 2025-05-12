#include "AP_CoaxServo.h"

AP_CoaxServo::AP_CoaxServo()
{
    if (_singleton) {
        return;
    }
    _singleton = this;
}

AP_CoaxServo *AP_CoaxServo::_singleton = nullptr;
AP_CoaxServo *AP_CoaxServo::get_singleton()
{
    return _singleton;
}