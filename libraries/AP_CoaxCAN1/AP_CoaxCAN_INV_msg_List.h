#ifndef AP_COAXCAN_INV_MSG_LIST_H
#define AP_COAXCAN_INV_MSG_LIST_H

#include <AP_Common/AP_Common.h>

typedef union {
    uint8_t ALL;
    struct {
        uint8_t Inverter_ONOFF : 2;
        uint8_t Ctrl_Mode : 3;
        uint8_t reserved1 : 2;
        uint8_t Fault_Clear : 1;
    }bits;
}Uni_INV_Flag1;

struct INV_CMD_msg {
    //DLC = 7
    Uni_INV_Flag1 BYTE0;        //8bit : Byte0
    int32_t Ref1_RAW;           //32bit : Byte1 ~ Byte4
    int16_t Ref2_RAW;           //16bit : Byte5 ~ Byte6
    float Reference1;           //Reference1 data
    float Reference2;           //Reference2 data
    //Not used : Byte7
};

struct INV_CC_msg {
    //DLC = 8
    uint16_t Gain_Kpc_RAW;      //16bit : Byte0 ~ Byte1
    uint16_t Gain_Kic_RAW;      //16bit : Byte2 ~ Byte3
    uint16_t Current_Limit;     //16bit : Byte4 ~ Byte5
    uint16_t reserved2;         //16bit : Byte6 ~ Byte7
    float Gain_Kpc;
    float Gain_Kic;
};

struct INV_SC_msg {
    //DLC = 8
    uint16_t Gain_Kps_RAW;     //16bit : Byte0 ~ Byte1
    uint16_t Gain_Kis_RAW;     //16bit : Byte2 ~ Byte3
    uint16_t Theta_Offset_RAW; //16bit : Byte4 ~ Byte5
    uint16_t Speed_Limit;      //16bit : Byte6 ~ Byte7
    float Gain_Kps;
    float Gain_Kis;
    float Theta_Offset;
};
   
struct INV_FLT_msg {
    //DLC = 6
    uint8_t OVL_RAW;           //8bit : Byte0
    uint8_t UVL_RAW;           //8bit : Byte1
    uint8_t OCL_RAW;           //8bit : Byte2
    uint8_t OTL_RAW;           //8bit : Byte3
    int16_t OSL_RAW;           //16bit : Byte4 ~ Byte5
    uint16_t OVL;
    uint16_t UVL;
    uint16_t OCL;
    uint16_t OTL;
    uint16_t OSL;
};

struct INV_STATUS1_msg {
    //DLC = 8
    int16_t Motor_Spd_RAW;      //16bit : Byte0 ~ Byte1
    int16_t i_a_RAW;            //16bit : Byte2 ~ Byte3
    int16_t i_b_RAW;            //16bit : Byte4 ~ Byte5
    int16_t i_c_RAW;            //16bit : Byte6 ~ Byte7
    float motor_Spd;
    float i_a;
    float i_b;
    float i_c;
};

struct INV_STATUS2_msg {
    //DLC = 8
    int16_t Byte_0_1_RAW;       //16bit : Byte0 ~ Byte1
    int16_t Byte_2_3_RAW;       //16bit : Byte2 ~ Byte3
    int16_t Byte_4_5_RAW;       //16bit : Byte4 ~ Byte5
    int16_t t_a_RAW;            //16bit : Byte6 ~ Byte7
    float t_a;
};

struct INV_STATUS3_msg {
    //DLC = 8
    int16_t t_b_RAW;           //16bit : Byte0 ~ Byte1
    int16_t t_c_RAW;           //16bit : Byte2 ~ Byte3
    int16_t Byte_4_5_RAW;      //16bit : Byte4 ~ Byte5
    int16_t Byte_6_7_RAW;      //16bit : Byte6 ~ Byte7
    float t_b;
    float t_c;
};

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
}Status4_FlagsSet;

struct INV_STATUS4_msg {
    //DLC = 8
    uint16_t reserved;          //16bit : Byte0 ~ Byte1
    Status4_FlagsSet Flagset;   //16bit : Byte2 ~ Byte3
    uint16_t V_dc_input_RAW;    //16bit : Byte4 ~ Byte5
    uint16_t MI;                //8bit : Byte6
    uint8_t Motor_Align_flag;   //1bit : Byte7 - bit0
    float V_dc_input;
};
#endif
