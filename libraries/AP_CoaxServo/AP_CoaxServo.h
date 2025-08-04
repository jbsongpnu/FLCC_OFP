#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

//CoaxServo actuator port with Pixhawk6X 
//----Serial port map------
// SERIAL0 -> USB
// SERIAL1 -> UART7 (Telem1) RTS/CTS pins
// SERIAL2 -> UART5 (Telem2) RTS/CTS pins
// SERIAL3 -> USART1 (GPS1)
// SERIAL4 -> UART8 (GPS2)
// SERIAL5 -> USART2 (Telem3) RTS/CTS pins
#define Actuator_UART   hal.serial(5)
// SERIAL6 -> UART4 (User)
// SERIAL7 -> USART3 (Debug)
// SERIAL8 -> USB (MAVLink, can be used for SLCAN with protocol change)

#define START_BYTE  255U
#define COAXSV_UART_BUFFER_SIZE 64

union Err_msg{
    uint8_t ALL;
    struct {
        uint8_t Angle_limit_err : 1;
        uint8_t Param_range_err : 1;
        uint8_t Over_temperature : 1;
        uint8_t Overloaded : 1;
        uint8_t Power_stage_err : 1;
        uint8_t rsvd : 3;
    }bits;
};

struct RX_Frame {
    uint8_t ID = 0;
    uint8_t LENGTH = 2;
    union Err_msg  ERROR;
    uint8_t Parameters[20];
};
class AP_CoaxServo {
public:
    AP_CoaxServo();

    static AP_CoaxServo *get_singleton();
    static AP_CoaxServo *_singleton;
    AP_CoaxServo *HiTechSV();

    void Set_dummyTX(void);
    void CMD_SET_POSITION(uint8_t id, int16_t setpoint);
    void CMD_SET_VELOCITY(uint8_t id, uint16_t speed);
    void CMD_SET_TORQUE(uint8_t id, uint16_t Trq);
    void Set_Servo_ID(uint8_t pre_id, uint8_t aft_id);
    void Set_UINT_Config(uint8_t id, uint8_t addrs, uint16_t value);
    void Set_INT_Config(uint8_t id, uint8_t addrs, int16_t value);
    void Request_SVData(uint8_t id, uint8_t addrs);
    void CMD_SET_MULTI_POSITIONS(void);

    uint16_t receive_CoaxServo_uart_data(void);
    void interprete_msg(uint8_t sv_id, uint8_t msg_id, uint8_t data_low, uint8_t data_high);

private:

    RX_Frame _RX_data[6];

};

namespace AP {
    AP_CoaxServo *HiTechSV();
};

