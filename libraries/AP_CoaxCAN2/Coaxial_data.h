#pragma once

#include <AP_Common/AP_Common.h>

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

class CoaxData
{
public:

    static CoaxData& get_instance();

    //====IFCU and PMU====
    uint8_t fcrdy = 0;
    //End of IFCU and PMU
    //====Inverter====
    datadef_INV INV_data;
    //End of Inverter
    //====CCB : Cooling Control Board===
    datadef_CCB_data CCB_data;
    //End of CCB
    datadef_Control_CMD Command_Received;

private:
    CoaxData();
    
    CoaxData(const CoaxData&) = delete;
    CoaxData& operator=(const CoaxData&) = delete;

    static CoaxData* _instance;
    // static CoaxData *_singleton;
};

CoaxData &cxdata();

//CoaxData &coaxdata();

