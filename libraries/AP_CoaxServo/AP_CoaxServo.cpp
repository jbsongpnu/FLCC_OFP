#include "AP_CoaxServo.h"
#include <AP_CoaxCAN2/Coaxial_data.h>

extern const AP_HAL::HAL& hal;

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

void AP_CoaxServo::Send_CMD_Frame(void) {
    uint8_t buffer[30]={0U};
    uint8_t size = 0;
    uint8_t i = 0;
    uint32_t temp_checksum = 0;

    //Check following 
        // _TX_data.ID = 2;
        // _TX_data.LENGTH = 5;
        // _TX_data.COMMAND = 1;
        // _TX_data.Parameters[0] = 0x30;
        // _TX_data.Parameters[1] = 0x31;
        // _TX_data.Parameters[2] = 0x32;
    //Should send with Checksum = 0x64

    if(_TX_data.LENGTH < 2) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Error! CoaxServo Length < 2");
        return;
    } else if (_TX_data.LENGTH > 27){
        gcs().send_text(MAV_SEVERITY_ERROR, "Error! CoaxServo TX Buffer overflow");
        return;
    } else {
        size = _TX_data.LENGTH - 2;//3
    }
    // Initialize Buffer
    memset(buffer, 0, size+5);

    // Set Start Command
    buffer[0]= START_BYTE;
    buffer[1]=_TX_data.ID;
    buffer[2]=_TX_data.LENGTH;
    buffer[3]=_TX_data.COMMAND;
    temp_checksum += (_TX_data.ID + _TX_data.LENGTH + _TX_data.COMMAND);
    for (i=0;i<size;i++) {
        buffer[i+4] = _TX_data.Parameters[i];
        temp_checksum += _TX_data.Parameters[i];
    }
    temp_checksum = temp_checksum & 0x000000FF;
    buffer[i+4] = ~temp_checksum; 
    // Send Buffer
    Actuator_UART->write(buffer, (size+5));

    // gcs().send_text(MAV_SEVERITY_INFO, "BufTx %02x %02x %02x %02x %02x %02x %02x %02x",
    //     buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
}

// COMMAND                              Command ID  Number of   Number of Response
//                                                  Parameters  Parameters
// CMD_PADATA_PING                      0x01        0           0
// CMD_PADATA_SET_POSITION              0x02        2           2
// CMD_PADATA_SET_PARAMETER             0x03        2-20        0
// CMD_PADATA_GET_PARAMETER             0x04        2           1-20
// CMD_PADATA_SET_MULTI_POSITIONS       0x07        3-60        N/A
// CMD_PADATA_GET_POSITION              0x10        0           2
// CMD_PADATA_GET_MOT_TEMP_C            0x11        0           1
// CMD_PADATA_GET_MOT_TEMP_F            0x12        0           1
// CMD_PADATA_GET_SW_REV                0x13        0           2
// CMD_PADATA_GET_CURRENT               0x14        0           2
// CMD_PADATA_GET_SIGNED_CURRENT        0x15        0           2
// CMD_PADATA_POWERSTAGE_DISABLE        0x20        1           0
// CMD_PADATA_RECALL_FACTORY_SETTING    0x21        0           0
// CMD_PADATA_RESET                     0x22        0           0

// ====== CMD_PADATA_PING
// Is used to detect a particular actuator. Broadcast ID 0x1F address any connected
// actuator, therefore the broadcast ID should not be used in a data bus system by
// reason that all connected servo actuators will answer simultaneously.
// Answered by Response Frame
void AP_CoaxServo::CMD_PADATA_PING(uint8_t id) {
    //check validity
    if(id>31) return; //if id is 31(=0x1F), ping is broadcasted to all servos
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 2;
    _TX_data.COMMAND = 1;
    //Send command
    Send_CMD_Frame();
}

// ====== CMD_PADATA_SET_POSITION
// The CMD_PADATA_SET_POSITION command makes the actuator run to the defined position.
// If the required position exceeds the specified maximum deflection angle, the
// actuator runs to the specified maximum position. In this case the Angle Limit Error
// bit is set into the answers error byte
// Answered by Response Frame
void AP_CoaxServo::CMD_PADATA_SET_POSITION(uint8_t id, uint16_t setpoint) {
    //check validity
    if(id>30) return;
    if(setpoint>4048) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 4;
    _TX_data.COMMAND = 2;
    _TX_data.Parameters[0] = (uint8_t)(setpoint & 0x00FF);
    _TX_data.Parameters[1] = (uint8_t)((setpoint >> 8) & 0x00FF);
    //Send command
    Send_CMD_Frame();
}

// ====== CMD_PADATA_SET_PARAMETER or CMD_PADATA_GET_PARAMETER
// PARAMETER                       Parameter ID     Read    Parameter Range
//                                 / Address        / Write
// PARM_PADATA_ACTUATOR_ID         0x01             R/W     0 - 30
// PARM_PADATA_ANGLE_LIMIT_MIN_L   0x02             R/W
// PARM_PADATA_ANGLE_LIMIT_MIN_H   0x03             R/W     48 - 4048
// PARM_PADATA_ANGLE_LIMIT_MAX_L   0x04             R/W
// PARM_PADATA_ANGLE_LIMIT_MAX_H   0x05             R/W     48 - 4048
// PARM_PADATA_EXPANSION           0x06             R/W     10 - 200            => Not used
// PARM_PADATA_REVERSE             0x07             R/W     1 (TRUE) / 0 (FALSE)
// PARM_PADATA_NEUTRAL_OFFSET_L    0x08             R/W
// PARM_PADATA_NEUTRAL_OFFSET_H    0x09             R/W     -500 - +500
// PARM_PADATA_FAILSAFE_POSITION_L 0x0A             R/W                         => Not used
// PARM_PADATA_FAILSAFE_POSITION_H 0x0B             R/W     48 - 4048           => Not used
// PARM_PADATA_FAILSAFE_TIMEOUT    0x0C             R/W     0 - 127             => Not used
// PARM_PADATA_SENSOR_DB           0x0D             R/W     0 - 50
// PARM_PADATA_PROFILE             0x0E             R/W     1 - 5

// ====== CMD_PADATA_SET_PARAMETER : PARM_PADATA_ACTUATOR_ID
// Answered by Response Frame
void AP_CoaxServo::Set_Servo_ID(uint8_t pre_id, uint8_t aft_id) {
    //check validity
    if((pre_id>30)||(aft_id>30)) return;
    //Set TX data
    _TX_data.ID = pre_id;
    _TX_data.LENGTH = 4;
    _TX_data.COMMAND = 3;
    _TX_data.Parameters[0] = 1;
    _TX_data.Parameters[1] = aft_id;
    //Send command
    Send_CMD_Frame();
}
// ===== CMD_PADATA_SET_PARAMETER : PARM_PADATA_ANGLE_LIMIT_MIN_x and PARM_PADATA_ANGLE_LIMIT_MAX_x
// Answered by Response Frame
void AP_CoaxServo::Set_Angle_limit(uint8_t id, uint16_t min, uint16_t max) {
    //check validity
    if(id>30) return;
    if(min<48) return;
    if(max>4048) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 7;
    _TX_data.COMMAND = 3;
    _TX_data.Parameters[0] = 2;
    _TX_data.Parameters[1] = (uint8_t)(min & 0x00FF);
    _TX_data.Parameters[2] = (uint8_t)((min >> 8) & 0x00FF);
    _TX_data.Parameters[3] = (uint8_t)(max & 0x00FF);
    _TX_data.Parameters[4] = (uint8_t)((max >> 8) & 0x00FF);
    //Send command
    Send_CMD_Frame();
}

// ===== CMD_PADATA_SET_PARAMETER : PARM_PADATA_REVERSE
// Answered by Response Frame
void AP_CoaxServo::Set_Reverse(uint8_t id, bool rev) {
    //check validity
    if(id>30) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 4;
    _TX_data.COMMAND = 3;
    _TX_data.Parameters[0] = 6;
    _TX_data.Parameters[1] = rev;
    //Send command
    Send_CMD_Frame();
}

// ===== CMD_PADATA_SET_PARAMETER : PARM_PADATA_NEUTRAL_OFFSET_x
// Answered by Response Frame
void AP_CoaxServo::Set_Neutral(uint8_t id, int16_t neutral) {
    //check validity
    if(id>30) return;
    if((neutral < -500)||(neutral>500)) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 5;
    _TX_data.COMMAND = 3;
    _TX_data.Parameters[0] = 8;
    _TX_data.Parameters[1] = (uint8_t)(neutral & 0x00FF);
    _TX_data.Parameters[2] = (uint8_t)((neutral >> 8) & 0x00FF);
    //Send command
    Send_CMD_Frame();
}

// ===== PARM_PADATA_SENSOR_DB
// ===== PARM_PADATA_PROFILE

// ===== CMD_PADATA_GET_PARAMETER
// Parameters can be acquired one by one, but this function acquires them all
// Answered by Response Frame
void AP_CoaxServo::Request_all_Param(uint8_t id) {
    //check validity
    if(id>30) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 0x10;
    _TX_data.COMMAND = 4;
    _TX_data.Parameters[0] = 1;
    _TX_data.Parameters[1] = 0x0E; //All param
    //Send command
    Send_CMD_Frame();
}

// ===== CMD_PADATA_SET_MULTI_POSITIONS
// The CMD_PADATA_SET_MULTI_POSITIONS command is a special command designed to allow
// for fast control of up to 20 actuators with just one command.
// This command is special in two ways:
//  - It must be sent with the actuator broadcast ID (31)
//  - No response to the command will be sent
// The command parameters consist of groups of three bytes, containing the ID of the
// actuator to be set and the position for that actuator.
// Every addressed actuator will behave as if a regular CMD_PADATA_SET_POSITION command
// was set, except that it will not transmit a response. The same angle limits and
// parameters apply.
// NOT answered by Response Frame
void AP_CoaxServo::Set_Coax_ServoPosition(void) {
    //check validity
    
    //Set TX data
    _TX_data.ID = 0x1F;
    _TX_data.LENGTH = 20;//3bytes for each servo, X6 servos + 2
    _TX_data.COMMAND = 7;
    for(int i=0; i<6; i++) {
        _TX_data.Parameters[(i * 3)] = i+1;
        _TX_data.Parameters[(i * 3)+1] = (uint8_t)(cxdata().SV_TX.SV_pos[i] & 0x00FF); 
        _TX_data.Parameters[(i * 3)+2] = (uint8_t)((cxdata().SV_TX.SV_pos[i] >> 8) & 0x00FF);    
    }
    //Send command
    Send_CMD_Frame();
}

// ===== CMD_PADATA_GET_POSITION
// Report the current position.
// Answered by Response Frame
void AP_CoaxServo::Request_Servo_Pos(uint8_t id) {
    //check validity
    if(id>30) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 2;
    _TX_data.COMMAND = 0x10;
    //Send command
    Send_CMD_Frame();
}

// ===== CMD_PADATA_GET_MOT_TEMP_C
// The CMD_PADATA_GET_MOT_TEMP_C reports the current motor temperature of the Pegasus
// actuator in degrees Celsius.
// The measurement range of the actuators motor-temperature is 20 to 120 degree
// Celsius. The actuator responds 20 degree Celsius even if the effective motor
// temperature is below.
// Answered by Response Frame
void AP_CoaxServo::Request_Servo_Temp(uint8_t id) {
    //check validity
    if(id>30) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 2;
    _TX_data.COMMAND = 0x11;
    //Send command
    Send_CMD_Frame();
}

// ===== CMD_PADATA_GET_CURRENT
// The actual measured motor current can be read out in mA with this command. The
// resolution is 10 mA.
// Answered by Response Frame
void AP_CoaxServo::Request_Servo_Current(uint8_t id) {
    //check validity
    if(id>30) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 2;
    _TX_data.COMMAND = 0x14;
    //Send command
    Send_CMD_Frame();
}

// Handling RX frame
// -------------------------------------------------------------------------
// Receive Data from CoaxServo UART
// -------------------------------------------------------------------------
int32_t AP_CoaxServo::receive_CoaxServo_uart_data(uint8_t* buffer)
{
    int32_t recv_size = 0;

    while ((Actuator_UART->available() > 0) && (COAXSV_UART_BUFFER_SIZE > recv_size))
    {
        buffer[recv_size] = (uint8_t)Actuator_UART->read();
        recv_size = recv_size + 1;
    }

    return recv_size;
}

namespace AP {

AP_CoaxServo *PegasusSV()
{
    return AP_CoaxServo::get_singleton();
}
};
