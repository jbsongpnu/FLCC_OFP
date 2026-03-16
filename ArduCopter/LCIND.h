#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#define UART_LC_1   hal.serial(1)
#define UART_LC_2   hal.serial(2)
#define UART_LC_3   hal.serial(5)
#define UART_LC_4   hal.serial(4)

#define DEVICE_ID_1 0
#define DEVICE_ID_2 1
#define DEVICE_ID_3 2
#define DEVICE_ID_4 3

class LCIND_class {
public:
    LCIND_class();

    static constexpr uint16_t FRAME_SIZE = 21;

    static LCIND_class *get_singleton();
    static LCIND_class *_singleton;
    
    void process_stream(const uint8_t* incoming, uint16_t length, uint8_t device);

    bool valid[4] = {0,0,0,0};
    
    float weight[4] = {0.0};
    float total_weight = 0;
    float length_x = 0;
    float length_y = 0;

private:
    
};

namespace AP {
LCIND_class *LCIND_g();
};