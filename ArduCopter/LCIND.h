#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

class LCIND_class {
public:
    LCIND_class();

    static LCIND_class *get_singleton();
    static LCIND_class *_singleton;
    LCIND_class *LCIND_g();
    void testf(uint16_t a);
    uint16_t get();
private:
    uint16_t test;
};


namespace AP {
LCIND_class *LCIND_g();
};