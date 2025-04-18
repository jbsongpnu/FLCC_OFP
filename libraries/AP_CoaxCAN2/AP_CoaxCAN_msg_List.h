#ifndef AP_COAXCAN_MSG_LIST_H
#define AP_COAXCAN_MSG_LIST_H

#include <AP_Common/AP_Common.h>

struct PMS1_msg {
    //0x6F0 DLC = 4
    uint8_t AlivCnt;        //4bit : Byte0 - bit 0~3 
    uint8_t State;          //8bit : Byte0 - bit 4~7  + Byte1 0~3 bit
    uint8_t LDC_State;      //8bit : Byte1 - bit 4~7 + Byte2 0~3 bit
    uint8_t Fault_LDC_No;   //8bit : Byte2 - bit 4~7 + Byte3 0~3 bit
    bool    Batt_SW_On;     //1bit : Byte3 - bit 4
    bool    Mv_SW_On;       //1bit : Byte3 - bit 5 
    bool    Lv_SW_On;       //1bit : Byte3 - bit 6
    bool    Batt_Charger_On;//1bit : Byte3 - bit 7
};

struct PMS2_msg {
    //0x6F1 DLC = 8
    int16_t Batt_Output_Current;    //16bit : Byte0 ~ Byte1
    uint16_t LDC_Output_Current;     //16bit : Byte2 ~ Byte3
    int16_t Mv_Output_Current;      //16bit : Byte4 ~ Byte5
    uint16_t Mv_Battery_Voltage;     //16bit : Byte6 ~ Byte7
};

struct PMS3_msg {
    //0x6F2 DLC = 8
    uint16_t OutputVoltage;     //16bit : Byte0 ~ Byte1
    int16_t OutputCurrent;     //16bit : Byte2 ~ Byte3
    uint16_t InputVoltage;      //16bit : Byte4 ~ Byte5
    int16_t InputCurrent;      //16bit : Byte6 ~ Byte7
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
}Uni_FDC1_Flag1;

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
}Uni_FDC1_Flag2;

struct FDC1_msg {
    //0x300 DLC = 8
    uint8_t AliveCnt;            //8bit : Byte0
    uint8_t State;              //8bit : Byte1
    uint8_t Aux_Volt;           //8bit : Byte2
    uint8_t Max_Temp;           //8bit : Byte3
    Uni_FDC1_Flag1 Flag1;       //8 Flag bits : Byte4
    Uni_FDC1_Flag2 Flag2;       //8 Flag bits : Byte5
};

struct FDC2_msg {
    //0x301 DLC = 8
    uint16_t OutputVoltage;     //16bit : Byte0 ~ Byte1
    int16_t OutputCurrent;     //16bit : Byte2 ~ Byte3
    uint16_t InputVoltage;      //16bit : Byte4 ~ Byte5
    int16_t InputCurrent;      //16bit : Byte6 ~ Byte7
};

struct VCUFDC1_msg {
    //0x400 DLC = 8
    uint8_t AliveCnt;           //8bit : Byte0
    uint8_t SET_CMD;                //4bit : Byte1 - Bit 0~3
    uint8_t Fault_Reset;            //4bit : Byte1 - Bit 4~7
    uint16_t Target_OutputVoltage;  //16bit : Byte2 ~ Byte3
    uint16_t Target_InputCurrent;   //16bit : Byte4 ~ Byte5
    uint16_t Target_InputPower;     //16bit : Byte6 ~ Byte7
};

struct IFCU1_msg {
    //0x1F0 DLC = 8
    uint16_t PpVlt;         //16bit : Byte0 ~ Byte1
    uint16_t PpCur;         //16bit : Byte2 ~ Byte3
    uint16_t PpCurLim;      //16bit : Byte4 ~ Byte5
    uint8_t  PpH2Sof;       //8bit : Byte6
    uint8_t  reserved;      //8bit : Byte7
};

struct IFCU2_msg {
    //0x2F0 DLC = 8
    uint8_t State;          //8bit : Byte0
    uint8_t FltSts;         //2bit : Byte1 - bit 0~1
    uint8_t DTC;            //8bit : Byte2
    uint8_t H2LkLmp;        //2bit : Byte3 - bit 0~1
    uint16_t SvmlsoRVlu;    //16bit : Byte4 ~ Byte5
    //uint8_t reserved2       //8bit : Byte6
    //uint8_t reserved3       //8bit : Byte7
};

struct IFCU3_msg {
    //0x2F1 DLC = 8
    uint16_t FcNetVlt;      //16bit : Byte0 ~ Byte1
    uint16_t FcNetCur;      //16bit : Byte2 ~ Byte3
    uint16_t LdcVlt;        //16bit : Byte4 ~ Byte5
    uint16_t LdcCur;        //16bit : Byte6 ~ Byte7
};

struct IFCU4_msg {
    //0x3F0 DLC = 8
    int8_t FcInClntTmp;     //8bit : Byte0
    int8_t AmbTemp;         //8bit : Byte1
    int8_t RoomTemp;        //8bit : Byte2
    uint8_t H2TnkPrs;       //8bit : Byte3
    uint8_t H2TnkTmp;       //8bit : Byte4
    uint16_t H2TnkFillCnt;  //16bit : Byte5 ~ Byte6
    // uint8_t  reserved4;     //8bit : Byte7
};

struct IFCU5_msg {
    //0x4F0 DLC = 8
    uint16_t HvBsaVlt;      //16bit : Byte0 ~ Byte1
    uint16_t HvBsaCur;      //16bit : Byte2 ~ Byte3
    uint8_t HvBsaSoC;       //8bit : Byte4
    uint8_t HvBsaSoH;       //8bit : Byte5
    uint8_t ExWtrTrpFill;   //1bit : Byte6 - bit 0
    // uint8_t  reserved5;     //8bit : Byte7
};

struct IFCU6_msg {
    //0x5F0 DLC = 8
    uint16_t FcMxCurLim;     //16bit : Byte0 ~ Byte1
    uint16_t FcNetCustCurLim;//16bit : Byte2 ~ Byte3
    uint8_t H2MidPrs;        //8bit : Byte4
    uint8_t FcClntFiltChk;   //1bit : Byte5 - bit 0
    uint8_t FcClntSplChk;    //1bit : Byte5 - bit 1
    //uint8_t reserved6       //8bit : Byte6
    //uint8_t reserved7       //8bit : Byte7
};

#endif
