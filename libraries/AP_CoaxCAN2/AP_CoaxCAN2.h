#ifndef AP_COAXCAN2_H_
#define AP_COAXCAN2_H_

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include "CoaxCAN_driver.hpp"
#include <AP_CoaxCAN2/AP_CoaxCAN_msg_List.h>

#define COAXCAN2_LOOP_HZ              (200U)      // 200Hz 5ms
#define COAXCAN2_MINOR_INTERVAL       (2U)      //(2U)        // 100Hz  5ms*2=10ms
#define COAXCAN2_MALVINK_INTERVAL     (20U)       // 10Hz  5ms*20=100ms
#define COAXCAN2_ONRUNNING_INTERVAL   (200U)      // 1Hz   5ms*200=1000ms

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

    void Check_ALL_data(void);

    IFCU1_msg _IFCU1;
    IFCU2_msg _IFCU2;
    IFCU3_msg _IFCU3;
    IFCU4_msg _IFCU4;
    IFCU5_msg _IFCU5;
    IFCU6_msg _IFCU6;
    PMS1_msg _PMS1;
    PMS2_msg _PMS2;
    PMS3_msg _PMS3;
    FDC1_msg _FDC1;
    FDC2_msg _FDC2;
    VCUFDC1_msg _VCUFDC1;

private:

    uint8_t _NewIFCU_msg = 0; //bit0~bit6 : IFCU1 ~ 6
    uint8_t _NewDMI_msg = 0; //bit0~2 : PMS1~3, bit3 : FDC1, bit4 : FDC2, bit5 : VCUFDC1
    uint8_t _IFCU_has_Initialized = 0;  //0 : not connected, each bit corresponds to _NewIFCU_msg
    uint8_t _DMI_has_Initialized = 0;   //0 : not connected, each bit corresponds to _NewDMI_msg

    char _thread_name[9];
    bool _initialized;
    uint8_t _driver_index;                                          // JBSong
    AP_HAL::CANIface* _can_iface;                                   // JBSong => alternative handler to _iface
                                                                    // '_can_iface' is acquired from new add_interface() function
    HAL_EventHandle _event_handle;                                  // JBSong

    coaxcan2::ICanIface* _iface;

    //-----Receive ID definition-----
    //Receive ID for IFCU
    static constexpr unsigned RX_ID_IFCU1 = 0x000001F0;
    static constexpr unsigned RX_ID_IFCU2 = 0x000002F0;
    static constexpr unsigned RX_ID_IFCU3 = 0x000002F1;
    static constexpr unsigned RX_ID_IFCU4 = 0x000003F0;
    static constexpr unsigned RX_ID_IFCU5 = 0x000004F0;
    static constexpr unsigned RX_ID_IFCU6 = 0x000005F0;
    static constexpr unsigned RX_ID_PMS1  = 0x000006F0;
    static constexpr unsigned RX_ID_PMS2  = 0x000006F1;
    static constexpr unsigned RX_ID_PMS3  = 0x000006F2;
    static constexpr unsigned RX_ID_FDC1  = 0x00000300;
    static constexpr unsigned RX_ID_FDC2  = 0x00000301;
    static constexpr unsigned RX_ID_VCUF1 = 0x00000400;
      
    //static constexpr unsigned RX_ID_NUM = 12U;   //Number of RX_ID
    //End of---Receive ID definition-----

    //TX definition class
    static constexpr unsigned ID_FCC1 = 0x720;  //FCC Alive, Ready, RunStop, CurrentReq
    static constexpr unsigned ID_FCC2 = 0x721;  //PowerReqruied, ThrottleNow, ThrottleFuture
    static constexpr unsigned ID_FCC3 = 0x720;  //Reserved

    class TX_ID
	{
    public:
        //IFCU & PMU
        static constexpr unsigned TX_ID_FCC1 = 0U;  //FCC Alive, Ready, RunStop, CurrentReq
        static constexpr unsigned TX_ID_FCC2 = 1U;  //PowerReqruied, ThrottleNow, ThrottleFuture
        static constexpr unsigned TX_ID_FCC3 = 2U;  //Reserved
        static constexpr unsigned TX_ID_NUM  = 3U; //Number of CMD_ID
	};

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

    uint32_t _cmd_id[TX_ID::TX_ID_NUM];
    //uint32_t _rx_id[RX_ID_NUM];

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