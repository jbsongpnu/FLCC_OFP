#ifndef AP_COAXCAN2_H_
#define AP_COAXCAN2_H_

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include "CoaxCAN_driver.hpp"

#define COAXCAN2_LOOP_HZ              (200U)      // 200Hz 5ms
#define COAXCAN2_MINOR_INTERVAL       (2U)      //(2U)        // 100Hz  5ms*2=10ms
#define COAXCAN2_MALVINK_INTERVAL     (20U)       // 10Hz  5ms*20=100ms
#define COAXCAN2_ONRUNNING_INTERVAL   (200U)      // 1Hz   5ms*200=1000ms

class COAX2_CTRL_CMD
{
public:
	COAX2_CTRL_CMD()
	{
        EX1 = 0;
        EX2 = 84;
	}

	uint32_t EX1;
	uint32_t EX2;
};

class AP_COAXCAN2 : public AP_CANDriver {
public:

    AP_COAXCAN2();
    ~AP_COAXCAN2();

    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    AP_COAXCAN2(const AP_COAXCAN2 &other) = delete;
    AP_COAXCAN2 &operator=(const AP_COAXCAN2&) = delete;

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // Return coaxcan2 from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_COAXCAN2 *get_coaxcan2(uint8_t driver_index);	

    void loop(void);
    void run(void);
    void RXspin(void);
    int TXspin(void);
    void handleFrame(const AP_HAL::CANFrame& can_rxframe);
    //TX Function to Devices
    int  CAN_TX_std(uint16_t can_id, uint8_t data_cmd[], uint8_t msgdlc);    //send with standard ID
    void TX_FCC1_MSG(void); //0x720
    void TX_FCC2_MSG(void); //0x721

    struct IFCU1_msg {
        uint16_t PpVlt;
        uint16_t PpCur;
        uint16_t PpCurLim;
        uint8_t  PpH2Sof;
        uint8_t  reserved;
    };
    IFCU1_msg _IFCU1;

    struct IFCU2_msg {
        uint8_t State;
        uint8_t FltSts;
        uint8_t DTC;
        uint8_t H2LkLmp;
        uint8_t SvmlsoRVlu;
    };
    IFCU2_msg _IFCU2;

    struct IFCU3_msg {
        uint16_t FcNetVlt;
        uint16_t FcNetCur;
        uint16_t LdcVlt;
        uint16_t LdcCur;
    };
    IFCU3_msg _IFCU3;

    struct IFCU4_msg {
        int8_t FcInClntTmp;
        int8_t AmbTemp;
        int8_t RoomTemp;
        uint8_t H2TnkPrs;
        uint8_t H2TnkTmp;
        uint16_t H2TnkFillCnt;
    };
    IFCU4_msg _IFCU4;

    struct IFCU5_msg {
        uint16_t HvBsaVlt;
        uint16_t HvBsaCur;
        uint8_t HvBsaSoC;
        uint8_t HvBsaSoH;
        uint8_t ExWtrTrpFill;
    };
    IFCU5_msg _IFCU5;

    struct IFCU6_msg {
        uint16_t FcMxCurLim;
        uint16_t FcNetCustCurLim;
        uint8_t H2MidPrs;
        uint8_t FcClntFiltChk;
        uint8_t FcClntSplChk;
    };
    IFCU6_msg _IFCU6;

private:

    char _thread_name[9];
    bool _initialized;
    uint8_t _driver_index;                                          // JBSong
    AP_HAL::CANIface* _can_iface;                                   // JBSong => alternative handler to _iface
                                                                    // '_can_iface' is acquired from new add_interface() function
    HAL_EventHandle _event_handle;                                  // JBSong

    coaxcan2::ICanIface* _iface;

    //-----Receive ID definition-----
    //CCB Test
    static constexpr unsigned RX_ID_CCB1  = 0x00000001;  //CCB1 message
    static constexpr unsigned RX_ID_CCB2  = 0x00000010;  //CCB2 message
    //Receive ID for IFCU
    static constexpr unsigned RX_ID_IFCU1 = 0x000001F0;     //IFCU Voltage-out, Current-out, CurrentLimit, H-tank
    static constexpr unsigned RX_ID_IFCU2 = 0x000002F0;     //IFCU State, Fault-state, What's Fault, 
    static constexpr unsigned RX_ID_IFCU3 = 0x000002F1;
    static constexpr unsigned RX_ID_IFCU4 = 0x000003F0;
    static constexpr unsigned RX_ID_IFCU5 = 0x000004F0;
    static constexpr unsigned RX_ID_IFCU6 = 0x000005F0;
    static constexpr unsigned RX_ID_NUM = 8U;   //Number of RX_ID
    //End of---Receive ID definition-----

    //Command ID definition class
    class CMD_ID
	{
    public:
        //CCB Test
        static constexpr unsigned CMD_ID_EX1 = 0U; //example 1
        static constexpr unsigned CMD_ID_EX2 = 1U; //example 2
        //IFCU 
        static constexpr unsigned CMD_ID_FCC1 = 0x720;  //FCC Alive, Ready, RunStop, CurrentReq
        static constexpr unsigned CMD_ID_FCC2 = 0x721;  //PowerReqruied, ThrottleNow, ThrottleFuture
        static constexpr unsigned CMD_ID_FCC3 = 0x720;  //Reserved
        static constexpr unsigned CMD_ID_NUM = 5U; //Number of CMD_ID
	};

    //example data state
    uint16_t _rx_ex1_data1;
    uint16_t _rx_ex1_data2;
    //Receive data for Cooling Control Board by NextFoam
    uint16_t _rx_raw_thermist1; //Thermist 1 temperature x10 deg 0~10,237 deg
    uint16_t _rx_raw_thermist2; //Thermist 2 temperature x10 deg 0~10,237 deg
    uint16_t _rx_raw_thermist3; //Thermist 3 temperature x10 deg 0~10,237 deg
    uint16_t _rx_raw_thermist4; //Thermist 4 temperature x10 deg 0~10,237 deg
    uint16_t _rx_raw_thermocp1; //Thermocouple 1 temperature x10 deg 0~10,237 deg
    uint16_t _rx_raw_thermocp2; //Thermocouple 2 temperature x10 deg 0~10,237 deg
    uint16_t _rx_raw_wflow;     //(Water)Flow sensor 0~30,000 mL/min 
    uint16_t _rx_raw_bdtemp;    //Board temperature 0~99 deg
    uint16_t _rx_raw_state;     //Cooling controller state

    uint32_t _cmd_id[CMD_ID::CMD_ID_NUM];
    uint32_t _rx_id[RX_ID_NUM];

    uint32_t RX_MSG;   //Received message
    uint32_t _rx_idx;  //Received message index
    uint32_t _cmd_idx;  //Command index

    COAX2_CTRL_CMD _coax2_ctrl_cmd;
    COAX2_CTRL_CMD _coax2_ctrl_cmd_prv;

    uint32_t _handleFrame_cnt;
	uint32_t _rtr_tx_cnt;
	uint32_t _cmd_tx_cnt;
	uint32_t _rtr_tx_err;
	uint32_t _cmd_tx_err;

    uint32_t _coaxcan2_last_send_us;

    uint64_t _AP_COAXCAN2_loop_cnt = 0;
    uint32_t coaxcan2_period_us;

    AP_Int8 _examp;
    static const uint16_t COAXCAN2_SEND_TIMEOUT_US = 500;

    enum COAXCAN2_STATUS : uint8_t{ // Status Manage
        CONNECTED               = 0,
        COMMUNICATION_ERROR     = 1,
        CONNECTION_FAILURE      = 2,
        VALUE_END               = 255,
    };
    enum COAXCAN2_STATUS COAXCAN2_Fail_Status;
    enum COAXCAN2_STATUS COAXCAN2_Fail_Status_prev;
    
    uint16_t COAXCAN2_ErrCnt;
    uint16_t COAXCAN2_RcvrCnt;

    uint8_t COAXCAN2_Ctrl_Seq;
    //CAN ICD
    uint8_t _FCC_AlivCnt;
    uint8_t _FCC_CmdFcRunStop;
    uint8_t _FCC_CmdPmsBatCut;
    uint8_t _FCC_Ready;
    uint8_t _FCC_Reserved1;
    uint16_t _FCC_FcPwrReq;
    uint16_t _FCC_FcThrottle;
    uint16_t _FCC_FcThrottlePrdct;

};

#endif