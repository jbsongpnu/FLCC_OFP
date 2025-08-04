#include "AP_CoaxServo.h"
#include <AP_CoaxCAN2/Coaxial_data.h>

#define DEBUG_COAXSERVO 1

//Will use Copter's motors_output(400Hz FAST_TASK) to send out CMD_SET_MULTI_POSITIONS
//motors_output() @/ArduCopter/motors.cpp
//Due to nature of RS-485 communication, all communitions should be scheduled in a single thread

extern const AP_HAL::HAL& hal;

mavlink_sys_icd_flcc_gcs_cxsv_pos_t     MAV_GCSTX_CXSV_POS = {0};
mavlink_sys_icd_flcc_gcs_cxsv_swash_t   MAV_GCSTX_CXSV_SWASH = {0};

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

void AP_CoaxServo::Set_dummyTX(void) {
    uint8_t buffer[32] = {0, };
    Actuator_UART->write(buffer, 32);
}

// ====== CMD_SET_POSITION
// The CMD_SET_POSITION command makes the actuator run to the defined position.
// 
void AP_CoaxServo::CMD_SET_POSITION(uint8_t id, int16_t setpoint) {
    //check validity
    if(id>6) return; //Servo ID numbering has changed from 0~5 => 1~6, but 0 is required for initial setting
    if((setpoint < 0) || (setpoint > 2048)) return; //Only allow 90degs turn clockwise and counter-clockwise

    uint8_t buffer[7];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = 0x1E;       //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(setpoint & 0x00FF);
    buffer[5] = (uint8_t)((setpoint >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)

    Actuator_UART->write(buffer, 7);
}

// ====== CMD_SET_VELOCITY
// The CMD_SET_VELOCITY command makes the actuator run to the defined position.
// 
void AP_CoaxServo::CMD_SET_VELOCITY(uint8_t id, uint16_t speed) {
    //check validity
    if(id>6) return; //Servo ID numbering has changed from 0~5 => 1~6, but 0 is required for initial setting
    if(speed > 4095) return; //Only allow 4095

    uint8_t buffer[7];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = 0x20;       //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(speed & 0x00FF);
    buffer[5] = (uint8_t)((speed >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)

    Actuator_UART->write(buffer, 7);
}

// ====== CMD_SET_TORQUE
// The CMD_SET_TORQUE command makes the actuator run to the defined position.
// 
void AP_CoaxServo::CMD_SET_TORQUE(uint8_t id, uint16_t Trq) {
    //check validity
    if(id>6) return; //Servo ID numbering has changed from 0~5 => 1~6, but 0 is required for initial setting
    if(Trq > 4095) return; //Only allow 4095

    uint8_t buffer[7];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = 0x22;       //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(Trq & 0x00FF);
    buffer[5] = (uint8_t)((Trq >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)

    Actuator_UART->write(buffer, 7);
}

// ====== Set_Servo_ID
// Only allowed to set up to 6 servos
void AP_CoaxServo::Set_Servo_ID(uint8_t pre_id, uint8_t aft_id) {
    //check validity
    if((pre_id>6)||(aft_id>6)) return;
    
    uint8_t buffer[7];
    buffer[0] = 0x96;       //Header
    buffer[1] = pre_id;     //Target ID
    buffer[2] = 0x32;       //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = aft_id;       
    buffer[5] = 0;
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)

    Actuator_UART->write(buffer, 7);
}

void AP_CoaxServo::Set_UINT_Config(uint8_t id, uint8_t addrs, uint16_t value) {
    //check validity
    if(id>6) return;
    
    uint8_t buffer[7];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = addrs;      //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(value & 0x00FF);
    buffer[5] = (uint8_t)((value >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)

    Actuator_UART->write(buffer, 7);
}

void AP_CoaxServo::Set_INT_Config(uint8_t id, uint8_t addrs, int16_t value) {
    //check validity
    if(id>6) return;
    
    uint8_t buffer[7];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = addrs;      //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(value & 0x00FF);
    buffer[5] = (uint8_t)((value >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)

    Actuator_UART->write(buffer, 7);
}

void AP_CoaxServo::Request_SVData(uint8_t id, uint8_t addrs) {
    //temp test
    static uint8_t count = 0;
    //check validity
    if((id>6)||(addrs > 0xC2)) return;
    
    uint8_t buffer[32] = {0, }; //to avoid TX interrupt delay, we send large amount of data with dummy bytes
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = addrs;      //Address
    buffer[3] = 0;          //Registry Length
    buffer[4] = (buffer[1] + buffer[2] + buffer[3]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)
    buffer[5] = count;
    count++;

    Actuator_UART->write(buffer, 32);

#if DEBUG_COAXSERVO == 1
    //gcs().send_text(MAV_SEVERITY_INFO, "Req SVData to %u for %u", id, addrs);
#endif
}

// CMD_SET_MULTI_POSITIONS
void AP_CoaxServo::CMD_SET_MULTI_POSITIONS(void) {
    
    for(int i=0; i<6; i++) {
        CMD_SET_VELOCITY((i+1), cxdata().SV_TX[i].SV_Vel);
        CMD_SET_TORQUE((i+1), cxdata().SV_TX[i].SV_Trq);
        CMD_SET_POSITION((i+1),cxdata().SV_TX[i].SV_pos);
    }
}

// Handling RX frame
// -------------------------------------------------------------------------
// Receive Data from CoaxServo UART
// Returns number of messages received
// -------------------------------------------------------------------------
uint16_t AP_CoaxServo::receive_CoaxServo_uart_data(void)
{
    uint8_t tempbyte1;
    uint8_t tempbyte2;
    uint8_t sv_id;
    uint8_t msg_id;
    uint8_t length;
    uint8_t data_low;
    uint8_t data_high;
    uint8_t chcksm_rx;
    uint8_t chcksm_cal;
    int16_t num_msg=0;
    //uint8_t tempdebugCheck = 0;

    while (Actuator_UART->available() > 0)
    {
        tempbyte1 = (uint8_t)Actuator_UART->read();
        if(tempbyte1 == 0xff) {
            //tempdebugCheck = 1;
            tempbyte2 = (uint8_t)Actuator_UART->read();
            if(tempbyte2 == 0x69) {
                //tempdebugCheck = 2;
                sv_id = (uint8_t)Actuator_UART->read();
                msg_id = (uint8_t)Actuator_UART->read();
                length = (uint8_t)Actuator_UART->read();
                data_low = (uint8_t)Actuator_UART->read();
                data_high = (uint8_t)Actuator_UART->read();
                chcksm_rx = (uint8_t)Actuator_UART->read();
                chcksm_cal = (sv_id + msg_id + length + data_low + data_high) % 256;
                if (chcksm_cal == chcksm_rx) {
                    interprete_msg(sv_id, msg_id, data_low, data_high);
                    num_msg = num_msg + 1;
                    //tempdebugCheck = 3;
                }
            }
        }
    }

    //gcs().send_text(MAV_SEVERITY_INFO, "CoaxUART debug %u", tempdebugCheck);
    return num_msg;
}

void AP_CoaxServo::interprete_msg(uint8_t sv_id, uint8_t msg_id, uint8_t data_low, uint8_t data_high) {
    uint16_t tempUint16 = 0;
    int16_t tempInt16 = 0;
    uint8_t isSignedInt = 0;
    if ((sv_id == 0)||(sv_id > 6)||(msg_id>0xC2)) { return; }
    
    uint8_t SV_index = sv_id - 1;

    switch (msg_id) {
        // case REG_PRODUCT_NO :
        //     break;
        // case REG_PRODUCT_VERSION :
        //     break;
        // case REG_FIRMWARE_VERSION :
        //     break;
        // case REG_SERIAL_NO_SUB :
        //     break;
        // case REG_SERIAL_NO_MAIN :
        //     break;
        case REG_STATUS_FLAG :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].ErrorCode.ALL = tempUint16;
            break;
        case REG_POSITION :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            isSignedInt = 1;
            cxdata().SV_Pos_prv[SV_index].raw = cxdata().SV_Pos[SV_index].raw;
            cxdata().SV_Pos[SV_index].raw = tempInt16;
            break;
        case REG_VELOCITY :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            isSignedInt = 1;
            cxdata().SV_state[SV_index].Status_Velocity = tempInt16;
            break;
        case REG_TORQUE :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            isSignedInt = 1;
            cxdata().SV_state[SV_index].Status_Velocity = tempInt16;
            break;
        case REG_VOLTAGE :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Status_Voltage = tempUint16;
            break;
        case REG_MCU_TEMP :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Status_MCU_Temp = tempUint16;
            break;
        case REG_MOTOR_TEMP :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Status_Motor_Temp = tempUint16;
            break;
        case REG_HUMIDITY :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Status_Humidity = tempUint16;
            break;
        // case REG_HUMIDITY_MAX :
        //     break;
        // case REG_HUMIDITY_MIN :
        //     break;
        case REG_POSITION_NEW :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            isSignedInt = 1;
            cxdata().SV_TXPos_feedback[SV_index] = tempInt16;
            break;
        case REG_VELOCITY_NEW :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Action_Velocity = tempUint16;
            break;
        case REG_TORQUE_NEW :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Action_Torque = tempUint16;
            break;
        // case REG_360DEG_TURN_NEW :
        //     break;
        case REG_SERVO_ID :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            if (sv_id == tempUint16) {
                cxdata().SV_state[SV_index].connected = 1;
            } else {
                gcs().send_text(MAV_SEVERITY_ERROR, "Invalid Servo ID Error %u %u %u %u", sv_id, data_low, data_high, tempUint16);
            }
            break;
        // case REG_BAUD_RATE :
        //     break;
        case REG_NORMAL_RETURN_DELAY :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Config_Delay = tempUint16;
            break;
        case REG_POWER_CONFIG :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Config_Power_Config = tempUint16;
            break;
        case REG_EMERGENCY_STOP :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Config_Emergency_Stop = tempUint16;
            break;
        case REG_ACTION_MODE :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Config_Action_Mode = tempUint16;
            break;
        case REG_POSITION_SLOPE :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Pos_Slope = tempUint16;
            break;
        case REG_DEAD_BAND :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Dead_band = tempUint16;
            break;
        case REG_VELOCITY_MAX :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Velocity_Max = tempUint16;
            break;
        case REG_TORQUE_MAX :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Torque_Max = tempUint16;
            break;
        case REG_VOLTAGE_MAX :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Volt_Max = tempUint16;
            break;
        case REG_VOLTAGE_MIN :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Volt_Min = tempUint16;
            break;
        case REG_TEMP_MAX :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Temp_Max = tempUint16;
            break;
        case REG_TEMP_MIN :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Temp_Min = tempUint16;
            break;
        case REG_POS_START :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            isSignedInt = 1;
            cxdata().SV_state[SV_index].Config_Pos_Start = tempInt16;
            break;
        case REG_POS_END :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            isSignedInt = 1;
            cxdata().SV_state[SV_index].Config_Pos_End = tempInt16;
            break;
        case REG_POS_NEUTRAL :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            isSignedInt = 1;
            cxdata().SV_state[SV_index].Config_Pos_Neutral = tempInt16;
            break;
        // case REG_FACTORY_DEFAULT :
        //     break;
        // case REG_CONFIG_SAVE :
        //     break;
        case REG_MOTOR_TURN_DIRECT :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Direnction = tempUint16;
            break;
        default :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            break;
    }
#if DEBUG_COAXSERVO == 1
    if(isSignedInt) {
        gcs().send_text(MAV_SEVERITY_INFO, "HiTech SV %u, MSG %u, Value %d", sv_id, msg_id, tempInt16);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "HiTech SV %u, MSG %u, Value %u", sv_id, msg_id, tempUint16);
    }
#endif
}
namespace AP {

AP_CoaxServo *HiTechSV()
{
    return AP_CoaxServo::get_singleton();
}
};
