/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Eugene Shamaev, Siddharth Bharat Purohit
 */
#ifndef AP_PMUCAN_H_
#define AP_PMUCAN_H_


#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include "pmucan_driver_can.hpp"
//#include <uORB/topics/gcs_flcc_pmu_ctrl.h>	//igpark //=>ORB threading not used - JBSong
//#include <uORB/topics/flcc_gcs_pmu_status.h>	//igpark //=>ORB threading not used - JBSong

#define PMUCAN_LOOP_HZ              (100U)      // 100Hz 10ms
#define PMUCAN_MINOR_INTERVAL       (2U)        // 50Hz  10ms*2=20ms
#define PMUCAN_MALVINK_INTERVAL     (10U)       // 10Hz  10ms*10=100ms
#define PMUCAN_ONRUNNING_INTERVAL   (100U)      // 1Hz   10ms*100=1000ms

#define PMUCAN_CMD_DLC (4U)


class PMU_CTRL_CMD
{
public:
	PMU_CTRL_CMD()
	{
        Engine_OnOff        = 0;
        Battery_Control_CMD = 84;
        Engine_Manual       = 0;
        Engine_Throttle_CMD = 0;
        Engine_CHK_CMD      = 0;
	}

	uint32_t Engine_OnOff;
	uint32_t Battery_Control_CMD;
	uint32_t Engine_Manual;
	uint32_t Engine_Throttle_CMD;
	uint32_t Engine_CHK_CMD;
};


class AP_PMUCAN : public AP_CANDriver {
public:
	AP_PMUCAN();
	~AP_PMUCAN();

    static const struct AP_Param::GroupInfo var_info[];				//JBSong

    /* Do not allow copies */
    AP_PMUCAN(const AP_PMUCAN &other) = delete;						// JBSong
    AP_PMUCAN &operator=(const AP_PMUCAN&) = delete;				// JBSong

    void init(uint8_t driver_index, bool enable_filters) override;	// JBSong
    bool add_interface(AP_HAL::CANIface* can_iface) override;		// JBSong

	// Return pmucan from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_PMUCAN *get_pmucan(uint8_t driver_index);				// JBSong

    void loop(void);
    void run(void);
    void RXspin(void);
    int TXspin(void);
    void engineonoffstate(void);
    void engineonmode(void);
    void engineoffmode(void);
    void handleFrame(const AP_HAL::CANFrame& can_rxframe);
    int pmucan_cmd(uint32_t can_id, uint32_t data_cmd);
    int pmucan_rtr(uint32_t can_id);
    /**
     * Returns:
     *  0 - rejected/timedout/enqueued
     *  1+ - sent/received
     *  negative - failure
     */
    void send2gcs(); //igpark
    void sendchanges(); // KAL

private:

    char _thread_name[9];
    bool _initialized;

    uint8_t _driver_index;                                          // JBSong
    AP_HAL::CANIface* _can_iface;                                   // JBSong => alternative handler to _iface
                                                                    // '_can_iface' is acquired from new add_interface() function
    HAL_EventHandle _event_handle;                                  // JBSong

    pmucan::ICanIface* _iface;                                      // igpark

    static constexpr unsigned PMU_BATSTS    = 0x10000100;
    static constexpr unsigned PMU_ENGSTS    = 0x10000110;
    static constexpr unsigned PMU_AUX1STS   = 0x10000120;
    static constexpr unsigned PMU_AUX2STS   = 0x10000130;
    static constexpr unsigned PMU_VERSTS	= 0x10000140;

    static constexpr unsigned PMU_BATCTRL   = 0x10000000;
    static constexpr unsigned PMU_ENGONOFF  = 0x10000010;
    static constexpr unsigned PMU_ENGMANUAL = 0x10000020;
    static constexpr unsigned PMU_ENGPCL    = 0x10000030;
    static constexpr unsigned PMU_ENGCHK    = 0x10000040;

    static constexpr unsigned RTR_ID_NUM = 5;

    class CMD_ID
	{
    public:
        static constexpr unsigned CMD_ID_BATCTRL   = 0U;
        static constexpr unsigned CMD_ID_ENGONOFF  = 1U;
        static constexpr unsigned CMD_ID_ENGMANUAL = 2U;
        static constexpr unsigned CMD_ID_ENGPCL    = 3U;
        static constexpr unsigned CMD_ID_ENGCHK    = 4U;
        static constexpr unsigned CMD_ID_NUM = 5U;
	};

    uint32_t _cmd_id[CMD_ID::CMD_ID_NUM];
    uint32_t _rtr_id[RTR_ID_NUM];

    uint32_t RTR_MSG;
    uint32_t _rtr_idx;
    uint32_t _cmd_idx;

    PMU_CTRL_CMD _pmu_ctrl_cmd;
    PMU_CTRL_CMD _pmu_ctrl_cmd_prv;

    // perf counters  // KAL 23.05.23 -- REMOVED 
    // AP_HAL::Util::perf_counter_t _perf_loop_elapse;
    // AP_HAL::Util::perf_counter_t _perf_rtr_tx_elapse;
    // AP_HAL::Util::perf_counter_t _perf_cmd_tx_elapse;
    // AP_HAL::Util::perf_counter_t _perf_rx_elapse;
    // AP_HAL::Util::perf_counter_t _perf_err_cnt;
    // AP_HAL::Util::perf_counter_t _perf_send_err_cnt;
    // AP_HAL::Util::perf_counter_t _perf_send_num_cnt;
    // AP_HAL::Util::perf_counter_t _perf_rcv_err_cnt;
    // AP_HAL::Util::perf_counter_t _perf_rcv_num_cnt;
    // AP_HAL::Util::perf_counter_t _perf_no_mailbox_cnt;

	uint32_t _handleFrame_cnt;
	uint32_t _rtr_tx_cnt;
	uint32_t _cmd_tx_cnt;
	uint32_t _rtr_tx_err;
	uint32_t _cmd_tx_err;

    uint32_t _engineonoffmode;                                      // 0: OFF, 1: ON
	uint32_t _engineoncnt;
	uint32_t _engineoffcnt;

    uint32_t _pmucan_last_send_us;

	uint64_t _AP_PMUCAN_loop_cnt = 0;
    uint32_t pmucan_period_us;

    AP_Int8 _examp;                                                 // JBSong - example user-parameter
    // AP_Int8 _pmu_param1;                                                 // example user-parameter
    // AP_Int8 _pmu_param2;                                                 // example user-parameter
    // AP_Int8 _pmu_param3;                                                 // example user-parameter
    // AP_Int8 _pmu_param4;                                                 // example user-parameter
    static const uint16_t PMUCAN_SEND_TIMEOUT_US = 500;

    enum PMUCAN_STATUS : uint8_t{ // Status Manage
        CONNECTED               = 0,
        COMMUNICATION_ERROR     = 1,
        CONNECTION_FAILURE      = 2,
        VALUE_END               = 255,
    };
    enum PMUCAN_STATUS PMUCAN_Fail_Status;
    enum PMUCAN_STATUS PMUCAN_Fail_Status_prev;
    
    uint16_t PMUCAN_ErrCnt;
    uint16_t PMUCAN_RcvrCnt;

    uint8_t  PMU_Ctrl_Seq;


};
#endif
