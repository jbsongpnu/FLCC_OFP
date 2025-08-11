#pragma once

#include <AP_Common/AP_Common.h>

//===HiTech Servo Registry
//Read-only
#define REG_PRODUCT_NO          0x00
#define REG_PRODUCT_VERSION     0X02
#define REG_FIRMWARE_VERSION    0X04
#define REG_SERIAL_NO_SUB       0X06
#define REG_SERIAL_NO_MAIN      0X08
#define REG_STATUS_FLAG         0x0A
#define REG_POSITION            0x0C
#define REG_VELOCITY            0x0E
#define REG_TORQUE              0x10
#define REG_VOLTAGE             0x12
#define REG_MCU_TEMP            0x14
#define REG_MOTOR_TEMP          0x16
#define REG_WORKING_TIME        0x1A
#define REG_HUMIDITY            0x3C
#define REG_HUMIDITY_MAX        0x40
#define REG_HUMIDITY_MIN        0x42
//Read + write Action
#define REG_POSITION_NEW        0x1E
#define REG_VELOCITY_NEW        0x20
#define REG_TORQUE_NEW          0x22
#define REG_360DEG_TURN_NEW     0X24
//Read + write Configuration
#define REG_SERVO_ID            0x32
#define REG_BAUD_RATE           0x34
#define REG_NORMAL_RETURN_DELAY 0x3A
#define REG_POWER_CONFIG        0x46
#define REG_EMERGENCY_STOP      0x48
#define REG_ACTION_MODE         0x4A
#define REG_POSITION_SLOPE      0x4C
#define REG_DEAD_BAND           0x4E
#define REG_VELOCITY_MAX        0x54
#define REG_TORQUE_MAX          0x56
#define REG_VOLTAGE_MAX         0x58
#define REG_VOLTAGE_MIN         0x5A
#define REG_TEMP_MAX            0x5C
#define REG_TEMP_MIN            0x5E
#define REG_POS_START           0x96
#define REG_POS_END             0x94
#define REG_POS_NEUTRAL         0XC2
#define REG_FACTORY_DEFAULT     0X6E
#define REG_CONFIG_SAVE         0X70
#define REG_MOTOR_TURN_DIRECT   0X84

//Default Config Parameter values for HiTech Servos
#define PARAM_RETURN_DELAY      1
#define PARAM_POWER_CONFIG      2
#define PARAM_EMERGENCY_STOP    0x1800    //Stop at Over-voltage and Under-voltage
#define PARAM_ACTION_MODE       0x0060    //CR disabled, Velocity Mode, Acceleration Disabled
#define PARAM_POSITION_SLOPE    3800      //3800 = 0x0ED8, Max position slope => large torque at target point
#define PARAM_DEAD_BAND         2         //Set at minimum value of 2, smaller value creates vibration
#define PARAM_VELOCITY_MAX      4095      //4095 = 0x0FFF, Max velocity set to max value of 4095 
#define PARAM_TORQUE_MAX        4095      //4095 = 0x0FFF, Max torque set to max value of 4095
#define PARAM_VOLTAGE_MAX       290
#define PARAM_VOLTAGE_MIN       180
#define PARAM_TEMP_MAX          800
#define PARAM_TEMP_MIN          0
// #define PARAM_POS_START         455
// #define PARAM_POS_END           1593
// #define PARAM_POS_NEUTRAL       1024
#define PARAM_SV1_POS_START     381
#define PARAM_SV1_POS_NEUTRAL   950
#define PARAM_SV1_POS_END       1519
#define PARAM_SV2_POS_START     455
#define PARAM_SV2_POS_NEUTRAL   1024
#define PARAM_SV2_POS_END       1593
#define PARAM_SV3_POS_START     381
#define PARAM_SV3_POS_NEUTRAL   950
#define PARAM_SV3_POS_END       1519
#define PARAM_SV4_POS_START     381
#define PARAM_SV4_POS_NEUTRAL   950
#define PARAM_SV4_POS_END       1519
#define PARAM_SV5_POS_START     455     //0x01c7
#define PARAM_SV5_POS_NEUTRAL   1024    //0x0400
#define PARAM_SV5_POS_END       1593    //0x0639
#define PARAM_SV6_POS_START     321
#define PARAM_SV6_POS_NEUTRAL   890
#define PARAM_SV6_POS_END       1459
//Servo 1, 3 Start 381 Neutral 950 End 1519  //range 569 for 50deg
//Servo6 Start 321 Neutral 890 End 1459
#define PARAM_SV1_T_Direction   0 //CCW : 0, CW : 1
#define PARAM_SV2_T_Direction   1
#define PARAM_SV3_T_Direction   0
#define PARAM_SV4_T_Direction   0
#define PARAM_SV5_T_Direction   1
#define PARAM_SV6_T_Direction   0
typedef union {
    uint8_t ALL;
    struct {
        uint8_t Inverter_ONOFF : 2; //Inverter out On/Off 1: ON, 2: Off
        uint8_t Ctrl_Mode : 3; //Control mode
            //0: stop(standby), 1: vf(voltage control), 2:align motor, 3: current control, 4: speed control
        uint8_t reserved1 : 2;
        uint8_t Fault_Clear : 1; //0 or 1 : Fault clear (can clear during Inverter-off state)
    }bits;
}Uni_CMD_Flag1;

typedef union {
    //16-bit fault-flag set
    uint16_t ALL;
    struct {
        uint8_t Fault_OSD : 1;
        uint8_t Fault_OCD_A : 1;
        uint8_t Fault_OCD_B : 1;
        uint8_t Fault_OCD_C : 1;
        uint8_t nc1 : 1;
        uint8_t nc2 : 1;
        uint8_t nc3 : 1;
        uint8_t nc4 : 1;
        uint8_t Fault_OTD_A : 1;
        uint8_t Fault_OTD_B : 1;
        uint8_t Fault_OTD_C : 1;
        uint8_t nc5 : 1;
        uint8_t nc6 : 1;
        uint8_t nc7 : 1;
        uint8_t Fault_OVD_DC : 1;
        uint8_t Fault_UVD_DC : 1;
    }bits;
}Sts4_FlagsSet;

struct datadef_INV {
    Uni_CMD_Flag1 CMD_Flag;
    float Reference1;        //Reference1 data : 0.1 precision
        // Control Mode 1: v_qe
        // Control Mode 2: v_out
        // Control Mode 3: i_qe
        // Control Mode 4: speed (rpm)
    float Reference2;        //Reference2 data : 0.1 precision
        // Control Mode 1: freq
        // Control Mode 2: -
        // Control Mode 3: i_de
        // Control Mode 4: accel (rpm/s)
    float Gain_Kpc; //Gain_Kpc for current control : 0.01 precision
    float Gain_Kic; //Gain_Kic for current control : 0.1 precision
    uint16_t Current_Limit; //Current_Limit A : 1.0 precision
    float Gain_Kps; //Gain_Kps for speed control : 0.01 precision
    float Gain_Kis; //Gain_Kis for speed control : 0.1 precision
    float Theta_Offset; //phase angle degree : 0.1 precision
    uint16_t Speed_Limit; //speed limit rpm : 1.0 precision
    uint16_t OVL;   //Over-voltage protection V : 10 precision
    uint16_t UVL;   //Under-voltage protection V : 10 precision
    uint16_t OCL;   //Over-current protection A : 10 precision
    uint16_t OTL;   //Over-temperature protection deg : 10 precision
    uint16_t OSL;   //Over-speed limit rpm : 1 precision
    float motor_Spd;//motor speed rpm : 0.2 precision
    float i_a;      //phase A current A : 0.01 precision
    float i_b;      //phase B current A : 0.01 precision
    float i_c;      //phase C current A : 0.01 precision
    float t_a;      //phase A temperature deg C : 0.01 precision
    float t_b;      //phase B temperature deg C : 0.01 precision
    float t_c;      //phase C temperature deg C : 0.01 precision
    Sts4_FlagsSet FLT;  //Fault flag set
    float V_dc_input;   //Voltage input V : 0.1 precision
    uint16_t MI;    //Motor voltage usage % : 1 precision
    bool Motor_Align_flag;   //0 or 1 : align
    //Update indication flag
    uint8_t isNew; //bit0 : CMD, bit1 : CC, bit2 : SC, bit3 : FLT
                    //bit4 : st1, bit5 : st2, bit6 : st3, bit7 : st4
    //Application
    uint8_t Rdy2useINV;//Until other systems are ready to use the inverter, this is cleared to zero
    uint8_t pre_Rdy2useINV;//previous Rdy2useINV
    float Motor_RPM_CMD;
    float Motor_ACC_CMD;
};

typedef union {
    uint16_t ALL;
    struct {
        uint16_t Inverter_ONOFF : 1;//Inverter out On/Off 
        uint16_t Inverter_STOP : 1; //Inverter stop
        uint16_t Motor_RPM : 1;     //Set motor
        uint16_t Set_CC : 1;        //Set current control param
        uint16_t Set_SC : 1;        //Set speed control param
        uint16_t Set_FLT : 1;       //Set fault level
        uint16_t CCB_ActiveON : 1;  //CCB - Active control mode on
        uint16_t CCB_Motor_Off : 1; //CCB - All motors off
        uint16_t CCB_Motor_MAX : 1; //CCB - All motors should be forced to run with Max power
        uint16_t CCB_FAN_toggle : 1;//CCB - Toggle Fan on/off
        uint16_t reserved1 : 6;
    }bits;
}Uni_GCS_CMD_Flag1;

struct datadef_Control_CMD {
    Uni_GCS_CMD_Flag1 NewCMD;
    uint8_t Inv_On_Off;
    float Target_INV_RPM;
    float Target_INV_ACC;
    datadef_INV INVSetValue;
};

typedef union {
    uint8_t ALL;
    struct {
        uint8_t IsActive : 1;       //Active control on/off
        uint8_t Motor1_run : 1;     //Motor1 run (greater than zero)
        uint8_t Motor2_run : 1;     //Motor2 run (greater than zero)
        uint8_t IsForcedMax : 1;    //Motors are forced to run at full speed
        uint8_t FanOnOff : 1;       //Fan on or off
        uint8_t Tact1 : 1;          //Tact switch1 on or off
        uint8_t Tact2 : 1;          //Tact switch2 on or off
        uint8_t RS422_Enabled : 1;  //RS422 Enabled or not
    }bits;
}CC_State_Flag1;
struct datadef_CCB_data {
    uint16_t Thermistor1x10;    //Thermistor temperature x 10 (99.9deg => 999)
    uint16_t Thermistor2x10;    
    uint16_t Thermistor3x10;
    uint16_t Thermistor4x10;
    CC_State_Flag1 State;       //State bits
    uint16_t ThCp1x10;          //Thermo-coupler temperature
    uint16_t ThCp2x10;
    uint16_t Flow_mL;           //Flow at mL / minute
    uint8_t Brd_temp;           //Board temperature 0 ~ 99
    //Update indication flag
    uint8_t isNew;              //bit0 : MSG1, bit1 : MSG2
};

typedef union {
    uint8_t ALL;
    struct {
        uint8_t Ind1_OC_Fault : 1;
        uint8_t Ind2_OC_Fault : 1;
        uint8_t Ind3_OC_Fault : 1;
        uint8_t Ind4_OC_Fault : 1;
        uint8_t Current_Unbalance_Fault : 1;
        uint8_t Output_OC_Fault : 1;
        uint8_t SiC1_Fault : 1;
        uint8_t SiC2_Fault : 1;
        
    }bits;
}gUni_FDC1_Flag1;

typedef union {
    uint8_t ALL;
    struct {
        uint8_t SiC3_Fault : 1;
        uint8_t SiC4_Fault : 1;
        uint8_t Communication_Fault : 1;
        uint8_t Input_OV_Fault : 1;
        uint8_t Output_OV_Fault : 1;
        uint8_t Heatsink_OT_Fault : 1;
        uint8_t Input_UV_Fault : 1;
        uint8_t Output_UV_Fault : 1;
        
    }bits;
}gUni_FDC1_Flag2;

struct datadef_PMS_data {
    uint8_t isAlive;
    //PMS1
    uint8_t AlivCnt;        //looping 0 ~ 15 at 10Hz
    uint8_t PMS_State;          //0:Init, 1:Run, 2:Warning, 3:Fault, 4:Reset
    uint8_t LDC_State;      //0:Off, 1:Run, 2:Warning, 3:Fault (Warning:1.2kW, Fault:900W)
    uint8_t Fault_LDC_No;   //1:0x01, 2:0x02, 3:0x04, 4:0x08, 5:0x10
    bool    Batt_SW_On;     //0:Off, 1:On
    bool    Mv_SW_On;       //0:Off, 1:On
    bool    Lv_SW_On;       //0:Off, 1:On
    bool    Batt_Charger_On;//0:Off, 1:On
    //PMS2
    float Batt_Output_Current;    //
    float LDC_Output_Current;     //16bit : Byte2 ~ Byte3
    float Mv_Output_Current;      //16bit : Byte4 ~ Byte5
    float Mv_Battery_Voltage;     //16bit : Byte6 ~ Byte7
    //PMS3
    float HDC_OutputVoltage;     //16bit : Byte0 ~ Byte1
    float HDC_OutputCurrent;     //16bit : Byte2 ~ Byte3
    float HDC_InputVoltage;      //16bit : Byte4 ~ Byte5
    float HDC_InputCurrent;      //16bit : Byte6 ~ Byte7
    //PMS4
    float PMS_Out_Power;     //16bit : Byte0 ~ Byte1
    float PMS_In_Power;      //16bit : Byte2 ~ Byte3
    float PMS_LDC_Out_Volt;  //16bit : Byte4 ~ Byte5
    float PMS_Max_Temp;          //16bit : Byte6 ~ Byte7    
    //FDC1
    uint8_t FDC_State;              //8bit : Byte1
    float FDC_Aux_Volt;           //8bit : Byte2
    uint16_t FDC_Max_Temp;           //8bit : Byte3
    gUni_FDC1_Flag1 FDC_Flag1;       //8 Flag bits : Byte4
    gUni_FDC1_Flag2 FDC_Flag2;       //8 Flag bits : Byte5
    //FDC2
        //Not needed
    //VCUFDC1
        //Not needed
};

struct datadef_IFCU_data {
    uint8_t isAlive;
    //IFCU1
    float PpVlt;         //16bit : Byte0 ~ Byte1
    float PpCur;         //16bit : Byte2 ~ Byte3
    float PpCurLim;      //16bit : Byte4 ~ Byte5
    float PpH2Sof;       //8bit : Byte6
    //IFCU2
    uint8_t State;          //8bit : Byte0
    uint8_t FltSts;         //2bit : Byte1 - bit 0~1
    uint8_t DTC;            //8bit : Byte2
    uint8_t H2LkLmp;        //2bit : Byte3 - bit 0~1
    uint16_t SvmlsoRVlu;    //16bit : Byte4 ~ Byte5
    //IFCU3
    float FcNetVlt;      //16bit : Byte0 ~ Byte1
    float FcNetCur;      //16bit : Byte2 ~ Byte3
    float LdcVlt;        //16bit : Byte4 ~ Byte5
    float LdcCur;        //16bit : Byte6 ~ Byte7
    //IFCU4
    int8_t FcInClntTmp;     //8bit : Byte0
    int8_t AmbTemp;         //8bit : Byte1
    int8_t RoomTemp;        //8bit : Byte2
    float H2TnkPrs;       //8bit : Byte3
    int16_t H2TnkTmp;       //8bit : Byte4
    uint16_t H2TnkFillCnt;  //16bit : Byte5 ~ Byte6
    //IFCU5
        //Not needed
    //IFCU6
    float FcMxCurLim;     
    float FcNetCustCurLim;
    uint16_t H2MidPrs;
    uint8_t FcClntFiltChk;
    uint8_t FcClntSplChk; 
};

struct TX_CoaxServo_data {
    int16_t SV_pos;
    uint16_t SV_Vel;
    uint16_t SV_Trq;
};

struct RX_CoaxServo_pos {
    int16_t raw;
    float angle;
};

union Err_msg_g{
    uint16_t ALL;
    struct {
        uint16_t reserved1 : 10;         //bit 0~9
        uint16_t Temp_Too_Low : 1;       //bit 10
        uint16_t Temp_Too_High : 1;      //bit 11
        uint16_t reserved2 : 1;          //bit 12
        uint16_t Volt_Too_Low : 1;       //bit 13
        uint16_t Volt_Too_High : 1;      //bit 14
        uint16_t reserved3 : 1;          //bit 15
    }bits;
};

struct Data_CoaxSerovs {
    //User-defined
    uint8_t connected = 0;
    //Read from HiTech servo registry
    union Err_msg_g ErrorCode;          //REG_STATUS_FLAG
    int16_t Status_Velocity = 0;        //REG_VELOCITY
    int16_t Status_Torque = 0;          //REG_TORQUE
    uint16_t Status_Voltage = 0;        //REG_VOLTAGE
    uint16_t Status_MCU_Temp = 0;       //REG_MCU_TEMP
    uint16_t Status_Motor_Temp = 0;     //REG_MOTOR_TEMP
    uint16_t Status_Humidity = 0;       //REG_HUMIDITY
    int16_t Action_Velocity = 0;        //REG_VELOCITY_NEW
    int16_t Action_Torque = 0;          //REG_TORQUE_NEW
    uint16_t Config_Delay = 0;          //REG_NORMAL_RETURN_DELAY
    uint16_t Config_Power_Config = 0;   //REG_POWER_CONFIG
    uint16_t Config_Emergency_Stop = 0; //REG_EMERGENCY_STOP
    uint16_t Config_Action_Mode = 0;    //REG_ACTION_MODE
    uint16_t Config_Pos_Slope = 0;      //REG_POSITION_SLOPE
    uint16_t Config_Dead_band = 0;      //REG_DEAD_BAND
    uint16_t Config_Velocity_Max = 0;   //REG_VELOCITY_MAX
    uint16_t Config_Torque_Max = 0;     //REG_TORQUE_MAX
    uint16_t Config_Volt_Max = 0;       //REG_VOLTAGE_MAX
    uint16_t Config_Volt_Min = 0;       //REG_VOLTAGE_MIN
    uint16_t Config_Temp_Max = 0;       //REG_TEMP_MAX
    uint16_t Config_Temp_Min = 0;       //REG_TEMP_MIN
    int16_t Config_Pos_Start = 0;       //REG_POS_START
    int16_t Config_Pos_End = 0;         //REG_POS_END
    int16_t Config_Pos_Neutral = 0;     //REG_POS_NEUTRAL
    uint8_t Config_Direnction = 0;      //REG_MOTOR_TURN_DIRECT
};

struct CoaxSwashState {
    float Col;
    float Lat;
    float Lon;
    float Rud;
};

//Coaxial State-machine state
enum class CoaxState {
    CXSTATE_0_INIT,
    CXSTATE_1_CHECK,
    CXSTATE_2_WAIT,
    CXSTATE_3_READY,
    CXSTATE_4_GNDTEST,
    CXSTATE_5_MOTSPOOL,
    CXSTATE_6_IDLERPM,
    CXSTATE_7_ONFLIGHT,
    CXSTATE_8_LANDED
};

struct HiTechTestState {
    uint8_t ServoTestStep = 0;
    uint8_t ServoTestingID = 0;
    uint8_t SVDataRequested = 0;
    uint8_t SVConfigModified = 0;
    uint8_t ServoCheckFinished = 0;
    uint8_t Request_retry = 0;
};

class CoaxData
{
public:

    static CoaxData& get_instance();

    //====IFCU and PMU====
    uint8_t fcrdy = 0;
    datadef_PMS_data DMI_PMS_data;
    datadef_IFCU_data IFCU_data;
    //End of IFCU and PMU
    //====Inverter====
    datadef_INV INV_data;
    //End of Inverter
    //====CCB : Cooling Control Board====
    datadef_CCB_data CCB_data;
    //End of CCB
    datadef_Control_CMD Command_Received;

    //====CoaxServo====
    TX_CoaxServo_data SV_TX[6];        //TX coaxial servo data @current session
    TX_CoaxServo_data SV_TX_prev[6];   //TX coaxial servo data @previous session
    RX_CoaxServo_pos SV_Pos[6];        //Current servo position
    RX_CoaxServo_pos SV_Pos_prv[6];    //Previous servo position
    int16_t SV_TXPos_feedback[6];        //Feedback of position TX
    Data_CoaxSerovs SV_state[6];       //Current servo states
    uint8_t SVinitialized;              //All-motors responded to Ping, comm link initialized
    CoaxSwashState Swash;
    CoaxSwashState Swash_prev;
    CoaxSwashState Swash_CMD;    
    HiTechTestState SVTestState;

    //====State Machine
    CoaxState CX_State; //Coaxial State-machine state
private:
    CoaxData();
    
    CoaxData(const CoaxData&) = delete;
    CoaxData& operator=(const CoaxData&) = delete;

    static CoaxData* _instance;
    // static CoaxData *_singleton;
    
};

CoaxData &cxdata();

