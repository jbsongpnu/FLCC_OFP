#include "AP_CoaxServo.h"
#include <AP_CoaxCAN2/Coaxial_data.h>

//Will use Copter's motors_output(400Hz FAST_TASK) to send out CMD_PADATA_SET_MULTI_POSITIONS
//motors_output() @/ArduCopter/motors.cpp
//Due to nature of RS-485 communication, all communitions should be scheduled in a single thread

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
    _TX_data.COMMAND = ID_CMD_PADATA_PING;
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
    _TX_data.COMMAND = ID_CMD_PADATA_SET_POSITION;
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
    _TX_data.COMMAND = ID_CMD_PADATA_SET_PARAMETER;
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
    _TX_data.COMMAND = ID_CMD_PADATA_SET_PARAMETER;
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
    _TX_data.COMMAND = ID_CMD_PADATA_SET_PARAMETER;
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
    _TX_data.COMMAND = ID_CMD_PADATA_SET_PARAMETER;
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
    _TX_data.COMMAND = ID_CMD_PADATA_GET_PARAMETER;
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
    _TX_data.COMMAND = ID_CMD_PADATA_SET_MULTI_POSITIONS;
    for(int i=0; i<6; i++) {
        _TX_data.Parameters[(i * 3)] = i+1;
        _TX_data.Parameters[(i * 3)+1] = (uint8_t)(cxdata().SV_TX[i].SV_pos & 0x00FF); 
        _TX_data.Parameters[(i * 3)+2] = (uint8_t)((cxdata().SV_TX[i].SV_pos >> 8) & 0x00FF);    
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
    _TX_data.COMMAND = ID_CMD_PADATA_GET_POSITION;
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
    _TX_data.COMMAND = ID_CMD_PADATA_GET_MOT_TEMP_C;
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
    _TX_data.COMMAND = ID_CMD_PADATA_GET_CURRENT;
    //Send command
    Send_CMD_Frame();
}
// ===== CMD_PADATA_RESET
// Reset the actuator
// Answered by Response Frame
void AP_CoaxServo::Reset_Servo(uint8_t id) {
    //check validity
    if(id>30) return;
    //Set TX data
    _TX_data.ID = id;
    _TX_data.LENGTH = 2;
    _TX_data.COMMAND = ID_CMD_PADATA_RESET;
    //Send command
    Send_CMD_Frame();
}

// Handling RX frame
// -------------------------------------------------------------------------
// Receive Data from CoaxServo UART
// -------------------------------------------------------------------------
uint16_t AP_CoaxServo::receive_CoaxServo_uart_data(uint8_t* buffer)
{
    int16_t recv_size = 0;

    while ((Actuator_UART->available() > 0) && (COAXSV_UART_BUFFER_SIZE > recv_size))
    {
        buffer[recv_size] = (uint8_t)Actuator_UART->read();
        recv_size = recv_size + 1;
    }

    return recv_size;
}

// ===== Parse buffer
// Returns num_of_msgs : number of received RX frame => if returned 2, _RX_data[0] and _RX_data[1] are valid
uint8_t AP_CoaxServo::Parse_Buffer(uint8_t* buffer, uint16_t size) {
    uint16_t index = 0;
    uint16_t state = 0; //0 : check header, 1: check ID, 2: check Length, 3: Get Error, 4: getting parameters, 5: Chekcsum
    uint8_t  temp_length;
    uint16_t num_of_msgs = 0;
    uint16_t temp_index;
    uint16_t last_checked = 0;
    uint32_t temp_checksum;

    //set all _RX_data[] to zeros
    reset_RX_data();

    if (size > COAXSV_UART_BUFFER_SIZE) {
        return 0;
    }

    while(index < size) {   //smallest frame size is 5 bytes, should use 5-1=4
        //State 0 : check header
        if ((state == 0) && (buffer[index] == 0xFF)) {
            state = 1;
            last_checked = index;
            temp_index = 0;
            temp_length = 0;
            temp_checksum = 0;
            index++;
        //State 1: check ID
        } else if (state==1) {
            if (buffer[index] > 30) {    //Valid ID : 0~30
                state = 0;
                index = last_checked + 1;
            } else {
                _RX_data[num_of_msgs].ID = buffer[index];   //ID of servo motor that sent this message
                state = 2;
                temp_checksum += buffer[index];
                index++;
            }
        //State 2: check Length
        } else if (state==2) {
            if ((buffer[index] < 2 ) || (buffer[index] > 16)) {
                state = 0;
                index = last_checked + 1;
            } else {
                temp_length = _RX_data[num_of_msgs].LENGTH = buffer[index];
                state = 3;
                temp_checksum += buffer[index];
                index++;
            }
        //State 3: Get Error
        } else if (state==3) {
            _RX_data[num_of_msgs].ERROR.ALL =  buffer[index];
            state = 4;
            temp_checksum += buffer[index];
            index++;
        //State 4: getting parameters
        } else if (state==4) {
            if (temp_length <=2) {   // go to next step if there's no parameter left to process
                state = 5;
            } else {
                _RX_data[num_of_msgs].Parameters[temp_index] = buffer[index];
                temp_index++;
                temp_length--;
                temp_checksum += buffer[index];
                index++;
            }
        //State 5: Chekcsum
        } else if (state==5) {
            state = 0;
            temp_checksum = ~(temp_checksum & 0x000000FF);
            if (buffer[index] == (uint8_t)temp_checksum) {    //Checksum OK
                
                gcs().send_text(MAV_SEVERITY_INFO, "ID= %u, Len = %u, Err = %u, Param = %u %u %u %u", 
                    _RX_data[num_of_msgs].ID, 
                    _RX_data[num_of_msgs].LENGTH,
                    _RX_data[num_of_msgs].ERROR.ALL,
                    _RX_data[num_of_msgs].Parameters[0], _RX_data[num_of_msgs].Parameters[1], _RX_data[num_of_msgs].Parameters[1],
                    _RX_data[num_of_msgs].Parameters[4]);

                num_of_msgs++;
                if(num_of_msgs >= 6) {
                    gcs().send_text(MAV_SEVERITY_ERROR, "CoaxServo RX_Buffer full - may lose data");
                    return num_of_msgs;
                } else {
                    index++;
                }
            } else {    //Checksum bad
                index = last_checked + 1;
            }
        }

    }

    return num_of_msgs;
}

// ===== reset_RX_data
// Reset all _RX_data[] arrays
void AP_CoaxServo::reset_RX_data(void) {
    //can't use memset() function due to non-trivial type
    for(int i=0;i<6;i++){
        _RX_data[i].ID = 0;
        _RX_data[i].LENGTH = 0;
        _RX_data[i].ERROR.ALL = 0;
        for(int j=0;j<20;j++){
            _RX_data[i].Parameters[j] = 0;
        }
    }
    
}

// ===== Get length of _RX_data
uint8_t AP_CoaxServo::GET_RX_data_Length(uint8_t msgbox) {
    return _RX_data[msgbox].LENGTH;
}

// ===== interprete_msg
// Inerprete parsed message box
// Response frame has no reference message ID, but responds to each command differently
// Therefore, one needs to know expected message, and here, it is passed through cmd, and sv_ID
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
void AP_CoaxServo::interprete_msg(uint16_t msg_box_id, uint8_t cmd) {
    uint16_t uint16_temp1;
    uint16_t uint16_temp2;
    uint8_t id = _RX_data[msg_box_id].ID;
    if (id > 6) { 
        return;
    }
    if (cmd > 0x22) {
        return;
    }
    //same for all
    cxdata().SV_state[id].ErrorCode.ALL = _RX_data[msg_box_id].ERROR.ALL;
    //interprete specific command response
    switch(cmd) {
        case ID_CMD_PADATA_PING:
            cxdata().SV_state[id].connected = 1;
            break;
        case ID_CMD_PADATA_SET_POSITION: //CMD_PADATA_SET_POSITION
            //not used
            break;
        case ID_CMD_PADATA_SET_PARAMETER: // CMD_PADATA_SET_PARAMETER
            //no need to interprete
            break;
        case ID_CMD_PADATA_GET_PARAMETER:
            //_RX_data[msg_box_id].Parameters[0] is the actuator ID, which we know it already
            uint16_temp1 = _RX_data[msg_box_id].Parameters[1];
            uint16_temp2 = _RX_data[msg_box_id].Parameters[2];
            cxdata().SV_state[id].angle_limit_min =  uint16_temp1 + (uint16_temp2 << 8);
            uint16_temp1 = _RX_data[msg_box_id].Parameters[3];
            uint16_temp2 = _RX_data[msg_box_id].Parameters[4];
            cxdata().SV_state[id].angle_limit_max = uint16_temp1 + (uint16_temp2 << 8);
            cxdata().SV_state[id].expansion = _RX_data[msg_box_id].Parameters[5];
            cxdata().SV_state[id].reverse = _RX_data[msg_box_id].Parameters[6];
            uint16_temp1 = _RX_data[msg_box_id].Parameters[7];
            uint16_temp2 = _RX_data[msg_box_id].Parameters[8];
            cxdata().SV_state[id].offset = (int16_t)(uint16_temp1 + (uint16_temp2 << 8));
            uint16_temp1 = _RX_data[msg_box_id].Parameters[9];
            uint16_temp2 = _RX_data[msg_box_id].Parameters[10];
            cxdata().SV_state[id].failsafe_pos = (int16_t)(uint16_temp1 + (uint16_temp2 << 8));
            cxdata().SV_state[id].timeout = _RX_data[msg_box_id].Parameters[11];
            cxdata().SV_state[id].got_param = 1;
            break;
        case ID_CMD_PADATA_GET_POSITION:
            uint16_temp1 = _RX_data[msg_box_id].Parameters[0];
            uint16_temp2 = _RX_data[msg_box_id].Parameters[1];
            cxdata().SV_Pos[id].raw = uint16_temp1 + (uint16_temp2 << 8);
            break;
        case ID_CMD_PADATA_GET_MOT_TEMP_C:
            cxdata().SV_state[id].temperature = _RX_data[msg_box_id].Parameters[0];
            break;
        case ID_CMD_PADATA_GET_SW_REV:
            //not used
            break;
        case ID_CMD_PADATA_GET_CURRENT:
            uint16_temp1 = _RX_data[msg_box_id].Parameters[0];
            uint16_temp2 = _RX_data[msg_box_id].Parameters[1];
            cxdata().SV_state[id].current = (float)(uint16_temp1 + (uint16_temp2 << 8)) / 100.0;
            break;
        case ID_CMD_PADATA_GET_SIGNED_CURRENT:
        case ID_CMD_PADATA_POWERSTAGE_DISABLE:
        case ID_CMD_PADATA_RECALL_FACTORY_SETTING:
        case ID_CMD_PADATA_RESET:
            break;
        default: 
            break;
    }
    
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
namespace AP {

AP_CoaxServo *PegasusSV()
{
    return AP_CoaxServo::get_singleton();
}
};
