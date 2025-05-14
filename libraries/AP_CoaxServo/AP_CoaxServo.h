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
#define COAXSV_UART_BUFFER_SIZE 128

//====Description of Pegasus Actuators Protocol===
//1) 2 types of frames : Command and Response
//2) Command Frame structure
//      Byte1   START       0xFF
//      Byte2   ID          0~30 (0x00~0x1E) assigned to each motor
//                          Broadcasting ID is 31 (0x1F), Initial ID 1
//      Byte3   LENGTH      Command + num of params + checksum (=param+2)
//      Byte4   COMMAND     Command ID
//      Byte5~N Parameters  Additional parameters
//      ByteN+1 Checksum    ~((ID + LENGTH + COMMAND + sum(Parameters)) & 0x000000FF)
//3) Response Frame structure
//      Byte1   START       0xFF
//      Byte2   ID          0~30 (0x00~0x1E) assigned to each motor
//                          Broadcasting ID is 31 (0x1F)
//      Byte3   LENGTH      Command + num of params + checksum (=param+2)
//      Byte4   Error       Actuator status : See Error Code
//      Byte5~N Parameters  Additional parameters
//      ByteN+1 Checksum    ~((ID + LENGTH + COMMAND + sum(Parameters)) & 0x000000FF)
//4) Error Code
//      Bit0    Angle Limit Error       If SET_POSITION exceeds valid range
//      Bit1    Parameter Range Error   Value range of parameter exceeded (only for write access)
//      Bit2    Over temperature Error  Motor temperature exceeds specified limit
//      Bit3    Overload Error          Actuator is in overload condition
//      Bit4    Power stage Error       Power stage or motor failure
//      Bit5~7  Reserved
//================================================

struct CMD_Frame {
    uint8_t ID = 0;
    uint8_t LENGTH = 2;
    uint8_t COMMAND = 0;
    uint8_t Parameters[20] = {0};
    uint8_t Checksum = 0;
};

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
    AP_CoaxServo *PegasusSV();
    void Send_CMD_Frame(void);
    void CMD_PADATA_PING(uint8_t id);
    void CMD_PADATA_SET_POSITION(uint8_t id, uint16_t setpoint);
    void Set_Servo_ID(uint8_t pre_id, uint8_t aft_id);
    void Set_Angle_limit(uint8_t id, uint16_t min, uint16_t max);
    void Set_Reverse(uint8_t id, bool rev);
    void Set_Neutral(uint8_t id, int16_t neutral);
    void Request_all_Param(uint8_t id);
    void Set_Coax_ServoPosition(void);
    void Request_Servo_Pos(uint8_t id);
    void Request_Servo_Temp(uint8_t id);
    void Request_Servo_Current(uint8_t id);
    void Reset_Servo(uint8_t id);

    int32_t receive_CoaxServo_uart_data(uint8_t* buffer);
    uint8_t Parse_Buffer(uint8_t* buffer, uint16_t size);
    void interprete_msg(uint16_t msg_box_id, uint8_t cmd);
private:

    CMD_Frame _TX_data;
    RX_Frame _RX_data[6];

    void reset_RX_data(void);

};

namespace AP {
    AP_CoaxServo *PegasusSV();
};

