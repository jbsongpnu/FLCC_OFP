#ifndef AP_COAXCAN1_H_
#define AP_COAXCAN1_H_

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include "CoaxCAN_driver.hpp"

#define COAXCAN1_LOOP_HZ              (100U)      // 100Hz 10ms
#define COAXCAN1_MINOR_INTERVAL       (50U)      //(2U)        // 50Hz  10ms*2=20ms
#define COAXCAN1_MALVINK_INTERVAL     (100U)       // 10Hz  10ms*10=100ms
#define COAXCAN1_ONRUNNING_INTERVAL   (100U)      // 1Hz   10ms*100=1000ms

class COAX1_CTRL_CMD
{
public:
	COAX1_CTRL_CMD()
	{
        EX1 = 0;
        EX2 = 84;
	}

	uint32_t EX1;
	uint32_t EX2;
};

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

private:

    char _thread_name[9];
    bool _initialized;
    uint8_t _driver_index;                                          // JBSong
    AP_HAL::CANIface* _can_iface;                                   // JBSong => alternative handler to _iface
                                                                    // '_can_iface' is acquired from new add_interface() function
    HAL_EventHandle _event_handle;                                  // JBSong

    coaxcan::ICanIface* _iface;

    //Receive ID definition
    static constexpr unsigned RX_ID_CCB1 = 0x00000001;  //CCB1 message
    static constexpr unsigned RX_ID_CCB2 = 0x00000010;  //CCB2 message
    static constexpr unsigned RX_ID_NUM = 2U;   //Number of RX_ID

    //Command ID definition class
    class CMD_ID
	{
    public:
        static constexpr unsigned CMD_ID_EX1 = 0U; //example 1
        static constexpr unsigned CMD_ID_EX2 = 1U; //example 2
        static constexpr unsigned CMD_ID_NUM = 2U; //Number of CMD_ID
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

    COAX1_CTRL_CMD _coax1_ctrl_cmd;
    COAX1_CTRL_CMD _coax1_ctrl_cmd_prv;

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

    uint8_t  COAXCAN1_Ctrl_Seq;
};

#endif