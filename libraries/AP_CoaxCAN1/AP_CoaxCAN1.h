#ifndef AP_COAXCAN1_H_
#define AP_COAXCAN1_H_

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include "CoaxCAN_driver.hpp"
#include <AP_CoaxCAN1/AP_CoaxCAN_INV_msg_List.h>

#define COAXCAN1_LOOP_HZ              (200U)      // 200Hz 5ms
#define COAXCAN1_MINOR_INTERVAL       (2U)      //(2U)        // 100Hz  5ms*2=10ms
#define COAXCAN1_MALVINK_INTERVAL     (20U)       // 10Hz  5ms*20=100ms
#define COAXCAN1_ONRUNNING_INTERVAL   (200U)      // 1Hz   5ms*200=1000ms

class AP_COAXCAN1 : public AP_CANDriver {
public:

    AP_COAXCAN1();
    ~AP_COAXCAN1();

    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    AP_COAXCAN1(const AP_COAXCAN1 &other) = delete;
    AP_COAXCAN1 &operator=(const AP_COAXCAN1&) = delete;

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // Return coaxcan1 from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_COAXCAN1 *get_coaxcan1(uint8_t driver_index);	

    void loop(void);
    void run(void);
    void RXspin(void);
    int TXspin(void);
    void handleFrame(const AP_HAL::CANFrame& can_rxframe);
    //TX Function to Devices
    int  CAN_TX_Ext(uint32_t can_id, uint8_t data_cmd[], uint8_t msgdlc);    //send with Ext ID

    void TX_INV_SETCMD_MSG(void); 
    void TX_INV_SETCC_MSG(void);
    void TX_INV_SETSC_MSG(void);
    void TX_INV_SETFLT_MSG(void);

    void Check_INV_data(void);

    INV_CMD_msg INV_SET_CMD;
    INV_CMD_msg INV_GET_CMD;
    INV_CC_msg  INV_SET_CC;
    INV_CC_msg  INV_GET_CC;
    INV_SC_msg  INV_SET_SC;
    INV_SC_msg  INV_GET_SC;
    INV_FLT_msg INV_SET_FLT;
    INV_FLT_msg INV_GET_FLT;
    INV_STATUS1_msg INV_Status1;
    INV_STATUS2_msg INV_Status2;
    INV_STATUS3_msg INV_Status3;
    INV_STATUS4_msg INV_Status4;

private:

    uint8_t _NewINV_msg = 0; //bit0 : CMD, bit1 : CC, bit2 : SC, bit3 : FLT
                            //bit4 : st1, bit5 : st2, bit6 : st3, bit7 : st4
    char _thread_name[9];
    bool _initialized;
    uint8_t _driver_index;                                          // JBSong
    AP_HAL::CANIface* _can_iface;                                   // JBSong => alternative handler to _iface
                                                                    // '_can_iface' is acquired from new add_interface() function
    HAL_EventHandle _event_handle;                                  // JBSong

    coaxcan1::ICanIface* _iface;

    //-----Receive ID definition-----
    //CCB Test
    static constexpr unsigned RX_ID_CCB1  = 0x00000001;  //CCB1 message
    static constexpr unsigned RX_ID_CCB2  = 0x00000010;  //CCB2 message
    //Receive ID for Inverter
    static constexpr unsigned RX_ID_INV_GET_CMD     = 0x016E0102; //Get Command
    static constexpr unsigned RX_ID_INV_GET_CC      = 0x016F0102; //Get Current control
    static constexpr unsigned RX_ID_INV_GET_SC      = 0x01700102; //Get Speed control
    static constexpr unsigned RX_ID_INV_GET_FLT     = 0x01710102; //Get Fault
    static constexpr unsigned RX_ID_INV_GET_STATUS1 = 0x01720102; //Get Status1
    static constexpr unsigned RX_ID_INV_GET_STATUS2 = 0x01730102; //Get Status2
    static constexpr unsigned RX_ID_INV_GET_STATUS3 = 0x01740102; //Get Status3
    static constexpr unsigned RX_ID_INV_GET_STATUS4 = 0x01750102; //Get Status4
    static constexpr unsigned RX_ID_NUM = 10U;   //Number of RX_ID
    //End of---Receive ID definition-----

    //Command ID definition
    static constexpr unsigned ID_INV_SET_CMD    = 0x010A0201; //Set Command
    static constexpr unsigned ID_INV_SET_CC     = 0x010B0201; //Set Current control
    static constexpr unsigned ID_INV_SET_SC     = 0x010C0201; //Set Speed control
    static constexpr unsigned ID_INV_SET_FLT    = 0x010D0201; //Set Fault

    class TX_ID
	{
        public:
        //CCB Test
        static constexpr unsigned TX_ID_EX1 = 0U; //example 1
        static constexpr unsigned TX_ID_EX2 = 1U; //example 2
        //Inverter
        static constexpr unsigned TX_ID_INV_SET_CMD    = 2U; //Set Command
        static constexpr unsigned TX_ID_INV_SET_CC     = 3U; //Set Current control
        static constexpr unsigned TX_ID_INV_SET_SC     = 4U; //Set Speed control
        static constexpr unsigned TX_ID_INV_SET_FLT    = 5U; //Set Fault
        static constexpr unsigned TX_ID_NUM = 6U;
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

    uint32_t _cmd_id[TX_ID::TX_ID_NUM];
    uint32_t _rx_id[RX_ID_NUM];

    uint32_t RX_MSG;   //Received message
    uint32_t _rx_idx;  //Received message index
    uint32_t _cmd_idx;  //Command index

    //COAX1_CTRL_CMD _coax1_ctrl_cmd;
    //COAX1_CTRL_CMD _coax1_ctrl_cmd_prv;

    uint32_t _handleFrame_cnt;
	uint32_t _rtr_tx_cnt;
	uint32_t _cmd_tx_cnt;
	uint32_t _rtr_tx_err;
	uint32_t _cmd_tx_err;

    uint32_t _coaxcan1_last_send_us;

    uint64_t _AP_COAXCAN1_loop_cnt = 0;
    uint32_t coaxcan1_period_us;

    AP_Int8 _examp;
    static const uint16_t COAXCAN1_SEND_TIMEOUT_US = 500;

    enum COAXCAN1_STATUS : uint8_t{ // Status Manage
        CONNECTED               = 0,
        COMMUNICATION_ERROR     = 1,
        CONNECTION_FAILURE      = 2,
        VALUE_END               = 255,
    };
    enum COAXCAN1_STATUS COAXCAN1_Fail_Status;
    enum COAXCAN1_STATUS COAXCAN1_Fail_Status_prev;
    
    uint16_t COAXCAN1_ErrCnt;
    uint16_t COAXCAN1_RcvrCnt;

    uint8_t COAXCAN1_Ctrl_Seq;
    //CAN ICD

};

#endif