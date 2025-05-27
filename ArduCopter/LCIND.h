#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#define UART_LC_1   hal.serial(2)
#define UART_LC_2   hal.serial(3)
#define UART_LC_3   hal.serial(4)
#define UART_LC_4   hal.serial(5)

#define DEVICE_ID_1 0
#define DEVICE_ID_2 1
#define DEVICE_ID_3 2
#define DEVICE_ID_4 3

class RingBuffer {
public:

    RingBuffer();

    bool push(uint8_t byte);
    bool pop_frame(uint8_t* out_frame);
    void reset();
private:
    static constexpr uint16_t BUF_SIZE = 64;
    
    uint8_t buffer[BUF_SIZE] = {0};
    uint16_t head = 0;
    uint16_t tail = 0;

    uint16_t distance(uint16_t from, uint16_t to) const;
};
class LCIND_class {
public:
    LCIND_class();

    static constexpr uint16_t FRAME_SIZE = 21;

    static LCIND_class *get_singleton();
    static LCIND_class *_singleton;
    LCIND_class *LCIND_g();
    
    bool parse(const uint8_t* data, uint16_t len, uint8_t device);
    void process_stream(RingBuffer& rb, const uint8_t* incoming, uint16_t length, uint8_t device);
    void TX_Set_Zero();
    void TX_request();

    enum State1 : uint8_t { Max_Exceeded, Stable, Unstable };
    enum State2 : uint8_t { Real_Weight, Gross_Weight };
    enum Unit : uint8_t { KG, LB };

    bool valid[4] = {0};
    
    float weight[4] = {0.0};
    float total_weight = 0;

private:
    
    bool isdigit(uint8_t ASCIIinput) const;
    uint8_t id[4] = {0};
    State1 state1[4] = {Stable, Stable, Stable, Stable};
    State2 state2[4] = {Real_Weight, Real_Weight, Real_Weight, Real_Weight};
    Unit unit[4] = {KG, KG, KG, KG};
    uint8_t frame[FRAME_SIZE] = {0};
};


namespace AP {
LCIND_class *LCIND_g();
};