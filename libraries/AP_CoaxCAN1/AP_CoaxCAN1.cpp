//CoaxCAN1 is for the Inverter and CCB of Coaxial rotor helicopter
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_CoaxCAN1.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_CoaxCAN2/Coaxial_data.h>

extern const AP_HAL::HAL& hal;

#define RS485_CAN_MSGID 0xFD

#define TEMP_EXP 0		//Initial value
#define DEBUG_INVERTER 0
#define DEBUG_CCB 0
#define DEBUG_GCSCMD 1
#define DEBUG_COAXSERVO 1
// Table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_COAXCAN1::var_info[] = {
    // @Param: example
    // @DisplayName: User example
    // @Description: Example for user
    AP_GROUPINFO("Ex", 1, AP_COAXCAN1, _examp, TEMP_EXP),
    //AP_GROUPINFO("Examp", 1, AP_COAXCAN, TEMP_EXP, TEMP_EXP),
    // AP_GROUPINFO("PARAM1", 1, AP_COAXCAN, _pmu_param1, TEMP_EXP),
    // AP_GROUPINFO("PARAM2", 2, AP_COAXCAN, _pmu_param2, TEMP_EXP),
    // AP_GROUPINFO("PARAM3", 3, AP_COAXCAN, _pmu_param3, TEMP_EXP),
    // AP_GROUPINFO("PARAM4", 4, AP_COAXCAN, _pmu_param4, TEMP_EXP),

    AP_GROUPEND
};

mavlink_sys_icd_flcc_gcs_inv_state_t MAV_GCSTX_INV_State = {0};   // Mavlink downstream for Inverter State ID=62000
mavlink_sys_icd_flcc_gcs_ccb_state_t MAV_GCSTX_CCB_State = {0};   // Mavlink downstream for CCB State ID=62002

AP_COAXCAN1::AP_COAXCAN1()
{
    AP_Param::setup_object_defaults(this, var_info);
    
    _examp.set_default(0);

    _initialized    = false;
    _iface                  = nullptr;

    coaxcan1_period_us = 1000000UL / COAXCAN1_LOOP_HZ;

    _cmd_id[TX_ID::TX_ID_INV_SET_CMD] = ID_INV_SET_CMD  | coaxcan1::CanFrame::FlagEFF;
    _cmd_id[TX_ID::TX_ID_INV_SET_CC]  = ID_INV_SET_CC   | coaxcan1::CanFrame::FlagEFF;
    _cmd_id[TX_ID::TX_ID_INV_SET_SC]  = ID_INV_SET_SC   | coaxcan1::CanFrame::FlagEFF;
    _cmd_id[TX_ID::TX_ID_INV_SET_FLT] = ID_INV_SET_FLT  | coaxcan1::CanFrame::FlagEFF;
    
}

AP_COAXCAN1::~AP_COAXCAN1()
{    
}

AP_COAXCAN1 *AP_COAXCAN1::get_coaxcan1(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_CoaxCAN1) {
        return nullptr;
    }
    return static_cast<AP_COAXCAN1*>(AP::can().get_driver(driver_index));
}

// -------------------------------------------------------------------------
//
// -------------------------------------------------------------------------
bool AP_COAXCAN1::add_interface(AP_HAL::CANIface* can_iface) {

    if (_can_iface != nullptr) {
    	hal.console->printf("COAXCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
    	hal.console->printf("COAXCAN: CAN driver not found\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN: CAN driver not found");
        return false;
    }

    if (!_can_iface->is_initialized()) {
    	hal.console->printf("COAXCAN: Driver not initialized\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN: Driver not initialized");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
    	hal.console->printf("COAXCAN: Cannot add event handle\n\r");
        //gcs().send_text(MAV_SEVERITY_WARNING, "COAXCAN: Cannot add event handle");
        return false;
    }
    return true;
}

// -------------------------------------------------------------------------
// Initialize COAXCAN bus
// -------------------------------------------------------------------------
void AP_COAXCAN1::init(uint8_t driver_index, bool enable_filters)
{
	_driver_index = driver_index;

    if (_initialized) {
        return;
    }

    if (_can_iface == nullptr) {
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_COAXCAN1::loop, void), _thread_name, 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;


    //gcs().send_text(MAV_SEVERITY_INFO, "[COAX] Initialized");
}

// -------------------------------------------------------------------------
// Task
// -------------------------------------------------------------------------
void AP_COAXCAN1::loop(void)
{
    while (true)
    {
        if (!_initialized)
        {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        hal.scheduler->delay_microseconds(coaxcan1_period_us);    // 2.5ms period loop

        run();
                    
        _AP_COAXCAN1_loop_cnt++;                                  // 2.55ms period increase
    }
}

// -------------------------------------------------------------------------
// Run : 400Hz
// [_AP_COAXCAN1_loop_cnt] is increased from loop() at every 2.5ms(400Hz)
// -------------------------------------------------------------------------
void AP_COAXCAN1::run(void)
{
    //Receive
	RXspin();

    //Check data 
    Check_INV_data();
    Check_CCB_data();

    //from %8, loops 0~6 for servo, 7 for CCB and Inverter
    if(_AP_COAXCAN1_loop_cnt%8 == 7) { 
        //Inverter and CCB 
        TXspin();
    } else {
        //Coax Servo loop 
        CoaxServoRun();
    }
    //Coax Servo loop and Inverter/CCB loops are designed to never overlap
    
}

//Coax Servo loop at 200Hz
void AP_COAXCAN1::CoaxServoRun(void)
{   //_AP_COAXCAN1_loop_cnt still usable
    switch (cxdata().CX_State) {
        case CoaxState::CXSTATE_0_INIT : 
            if(_AP_COAXCAN1_loop_cnt%4 == 0) {
                //Check Configuration of servos at Init state
                if(cxdata().SVTestState.ServoCheckFinished) { //If finished 
                    uint8_t All_OK = 1;
                    
                    for (uint8_t i = 0;i<6;i++) {
                        if ( (cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].connected == 0) 
                           || (cxdata().SVError.SV_Config_Error[i])) {
                        
                            All_OK = 0;
                        }
                    }
                    if (All_OK) {
                        //transit to next state
                        cxdata().CX_State = CoaxState::CXSTATE_1_CHECK;
                        cxdata().SVTestState.ServoCheckFinished = 0;
                        gcs().send_text(MAV_SEVERITY_INFO, "All Servo Config OK");
                    } else {
                        cxdata().CX_State = CoaxState::CXSTATE_F1_SERVOFAIL;
                    }
                } else { //If not finished checking
                    //Check configuration
                    SV_Config_Test();
                }
            }
        break;
        case CoaxState::CXSTATE_1_CHECK :
            if(_AP_COAXCAN1_loop_cnt%4 == 0) {//Reduce to 100Hz for Checking
                if(cxdata().SVTestState.ServoCheckFinished) { //If finished 
                    uint8_t All_OK = 1;
                    for (uint8_t i = 0;i<6;i++) {
                        if ( (cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].connected == 0) 
                           || (cxdata().SVError.SV_Config_Error[i])) {
                        
                            All_OK = 0;
                        }
                    }
                    if (All_OK) {
                        //transit to next state
                        cxdata().CX_State = CoaxState::CXSTATE_2_WAIT;
                        cxdata().SVTestState.ServoCheckFinished = 0;
                        gcs().send_text(MAV_SEVERITY_INFO, "Servos at Wait-state");
                    } else {
                        cxdata().CX_State = CoaxState::CXSTATE_F1_SERVOFAIL;
                    }
                } else {
                    SV_Check_State();
                }
            }
        break;
        case CoaxState::CXSTATE_2_WAIT :
            SV_Waiting_StateLoop(); //Enable this line for normal test
            //SV_Waiting_State_TESTLoop(); //Enable this line for temporary test
            if(_AP_COAXCAN1_loop_cnt%4000 == 0) {
                gcs().send_text(MAV_SEVERITY_INFO, "Servo Motor at Wait State");
            }
        break;
        case CoaxState::CXSTATE_F1_SERVOFAIL :
            if(_AP_COAXCAN1_loop_cnt%4000 == 0) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Critical : Servo Motor at Fail State");
            }
        break;
        default : 
        break;
    }
}

// -------------------------------------------------------------------------
// RX 
// -------------------------------------------------------------------------
void AP_COAXCAN1::RXspin()
{
    uint64_t time, timeout;
    int res = 0;

    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), COAXCAN1_SEND_TIMEOUT_US);
    AP_HAL::CANIface::CanIOFlags flags = 0;
    AP_HAL::CANFrame frame;                     // receive frame

    // wait for space in buffer to read
    bool read_select    = true; 
    bool write_select   = false;    //Read-only
    timeout = timeout_us + AP_HAL::micros64();

    //'Root/AP_HAL_ChibiOS/CanIface.cpp',
    // bool CANIface::select(bool &read, bool &write, const AP_HAL::CANFrame* pending_tx, uint64_t blocking_deadline)
    int ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret) {
        //gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 COMMUNICATION_ERROR");
        // return if no data is available to read
        // if(COAXCAN1_Fail_Status==COAXCAN1_STATUS::CONNECTION_FAILURE)
        // {
        //     COAXCAN1_Fail_Status = COAXCAN1_STATUS::CONNECTION_FAILURE;
        //     gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 CONNECTION_FAILURE");
        // }
        // else
        // {
        //     COAXCAN1_Fail_Status = COAXCAN1_STATUS::COMMUNICATION_ERROR;
        //     gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 COMMUNICATION_ERROR");
        // }

        return;
    }


    if(COAXCAN1_Fail_Status == COAXCAN1_STATUS::CONNECTED)     // Normal Connection 
    {
        res = _can_iface->receive(frame, time, flags);

        if(res > 0) // Data Received Normaly
        {
            if(COAXCAN1_ErrCnt > 0) // Check Error Count
            {
                COAXCAN1_ErrCnt = COAXCAN1_ErrCnt - 1;    // Decrease Error Count
            }

            while(res > 0)
            {
                handleFrame(frame);
                res = _can_iface->receive(frame, time, flags);     // Try Receive
            }
        }
        else        // Data Not Received
        {
            COAXCAN1_ErrCnt = COAXCAN1_ErrCnt + 1;          // Increase Error Count

            if(COAXCAN1_ErrCnt == 10) // Check Max Err Count
            {
                COAXCAN1_Fail_Status  = COAXCAN1_STATUS::COMMUNICATION_ERROR; // Set Communication Error Flag 
                gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 COMMUNICATION_ERROR ErrCnt10");
                COAXCAN1_ErrCnt       = 0; // Reset Error Count
            }
        }
    }
    else                            // Abnormal Connection 
    {
        res = _can_iface->receive(frame, time, flags);

        if(res > 0) // Data Received Normaly
        {
            COAXCAN1_RcvrCnt = COAXCAN1_RcvrCnt + 1;    // Increase Receive Count

            if(COAXCAN1_RcvrCnt == 5) // Check Max Recv Count
            {
                COAXCAN1_Fail_Status  = COAXCAN1_STATUS::CONNECTED; // Clear Communication Error Flag 
                COAXCAN1_RcvrCnt      = 0; // Reset Receive Count
                gcs().send_text(MAV_SEVERITY_INFO, "CoaxCAN1 Connected Err %d", COAXCAN1_ErrCnt);
            }

            while(res > 0)
            {
                handleFrame(frame);
                res = _can_iface->receive(frame, time, flags);     // Receive again
            }

        }
        else        // Data Not Received
        {
            if((COAXCAN1_RcvrCnt > 0) & (res < 0)) // Check Receive Count
            {
                COAXCAN1_RcvrCnt = COAXCAN1_RcvrCnt - 1;    // Decrease Receive Count
            }
        }

    }

}

// -------------------------------------------------------------------------
// Handle Frame : process each CAN frame
// -------------------------------------------------------------------------
void AP_COAXCAN1::handleFrame(const AP_HAL::CANFrame& can_rxframe)
{
    uint16_t    uint16_temp = 0;
    uint16_t    uint16_temp1 = 0;
    uint16_t    uint16_temp2 = 0;
    //uint8_t     uint8_temp = 0;

    int16_t int16_temp1 = 0;
    int16_t int16_temp2 = 0;
    int32_t int32_temp1 = 0;
    int32_t int32_temp2 = 0;
    //gcs().send_text(MAV_SEVERITY_INFO, "can received %lu", can_rxframe.id);
    switch(can_rxframe.id&can_rxframe.MaskExtID)
    {
        case RX_ID_CCB1:
            //Thermist 1 temperature
            uint16_temp = can_rxframe.data[0];
            CC_MSG1.Thermistor1x10 = uint16_temp * 256 + can_rxframe.data[1];
            //Thermist 2 temperature
            uint16_temp = can_rxframe.data[2];
            CC_MSG1.Thermistor2x10 = uint16_temp * 256 + can_rxframe.data[3];
            //Thermist 3 temperature
            uint16_temp = can_rxframe.data[4];
            CC_MSG1.Thermistor3x10 = uint16_temp * 256 + can_rxframe.data[5];
            //Thermist 4 temperature
            uint16_temp = can_rxframe.data[6];
            CC_MSG1.Thermistor4x10 = uint16_temp * 256 + can_rxframe.data[7];

            _NewCC_msg = _NewCC_msg | 0x01;//bit0 : MSG1
#if DEBUG_CCB == 1
            gcs().send_text(MAV_SEVERITY_INFO, "CCB_1 : %u, %u, %u, %u", 
                CC_MSG1.Thermistor1x10, CC_MSG1.Thermistor2x10, CC_MSG1.Thermistor3x10, CC_MSG1.Thermistor4x10 );
#endif
            break;

        case RX_ID_CCB2:

            //Thermocouple 1 temperature
            uint16_temp = can_rxframe.data[0];
            CC_MSG2.ThCp1x10 = uint16_temp * 256 + can_rxframe.data[1];
            //Thermocouple 2 temperature
            uint16_temp = can_rxframe.data[2];
            CC_MSG2.ThCp2x10 = uint16_temp * 256 + can_rxframe.data[3];
            //(Water)Flow sensor
            uint16_temp = can_rxframe.data[4];
            CC_MSG2.Flow_mL = uint16_temp * 256 + can_rxframe.data[5];
            //Board temperature
            CC_MSG2.Brd_temp = can_rxframe.data[6];
            //Cooling controller state
            CC_MSG2.State.ALL = can_rxframe.data[7];

            _NewCC_msg = _NewCC_msg | 0x02;//bit1 : MSG2
#if DEBUG_CCB == 1
            gcs().send_text(MAV_SEVERITY_INFO, "CCB_2 : %u, %u, %u, %u, %u", 
                CC_MSG2.ThCp1x10, CC_MSG2.ThCp2x10, CC_MSG2.Flow_mL, CC_MSG2.Brd_temp, CC_MSG2.State.ALL );
#endif
            break;
        case RX_ID_RS485 : {
            uint8_t sv_id;
            uint8_t msg_id;
            uint8_t length;
            uint8_t data_low;
            uint8_t data_high;
            uint8_t chcksm_rx;
            uint8_t chcksm_cal;
            if((can_rxframe.data[0] == 0xff) && (can_rxframe.data[1] == 0x69)) {
                //tempdebugCheck = 1;
                sv_id = can_rxframe.data[2];
                msg_id = can_rxframe.data[3];
                length = can_rxframe.data[4];
                data_low = can_rxframe.data[5];
                data_high = can_rxframe.data[6];
                chcksm_rx = can_rxframe.data[7];
                chcksm_cal = (sv_id + msg_id + length + data_low + data_high) % 256;
                if (chcksm_cal == chcksm_rx) {
                    interprete_msg(sv_id, msg_id, data_low, data_high);
                    _num_SVmsg = _num_SVmsg + 1;
                    _new_SVmsg_ID = msg_id;
                    _new_MSG_SVID = sv_id;
                    //tempdebugCheck = 2;
                }
            }
        }
            break;

        case RX_ID_INV_GET_CMD:
            INV_GET_CMD.BYTE0.ALL = can_rxframe.data[0];
            int32_temp1 = can_rxframe.data[1];
            int32_temp2 = can_rxframe.data[2];
            INV_GET_CMD.Ref1_RAW = (int32_temp2 << 8 ) + int32_temp1;
            int32_temp1 = can_rxframe.data[3];
            int32_temp2 = can_rxframe.data[4];
            INV_GET_CMD.Ref1_RAW += (int32_temp2 << 24 ) + (int32_temp1 << 16 );
            int16_temp1 = can_rxframe.data[5];
            int16_temp2 = can_rxframe.data[6];
            INV_GET_CMD.Ref2_RAW = (int16_temp2 << 8 ) + int16_temp1;
            
            INV_GET_CMD.Reference1 = (float)INV_GET_CMD.Ref1_RAW * 0.1;
            INV_GET_CMD.Reference2 = (float)INV_GET_CMD.Ref2_RAW * 0.1;

            _NewINV_msg = _NewINV_msg | 0x01;//bit0 : CMD
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVGETCMD : (Mode%u) %u, %ld, %d", 
                INV_GET_CMD.BYTE0.bits.Ctrl_Mode, INV_GET_CMD.BYTE0.ALL, 
                INV_GET_CMD.Ref1_RAW, INV_GET_CMD.Ref2_RAW);
            gcs().send_text(MAV_SEVERITY_INFO, "ref1=%f, ref2=%f", 
                INV_GET_CMD.Reference1, INV_GET_CMD.Reference2);
#endif
            break;
        case RX_ID_INV_GET_CC:
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            INV_GET_CC.Gain_Kpc_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            INV_GET_CC.Gain_Kic_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            INV_GET_CC.Current_Limit = uint16_temp2 * 256 + uint16_temp1;

            INV_GET_CC.Gain_Kpc = (float)INV_GET_CC.Gain_Kpc_RAW * 0.01;
            INV_GET_CC.Gain_Kic = (float)INV_GET_CC.Gain_Kic_RAW * 0.1;
            //Current_Limit has no scale factor

            _NewINV_msg = _NewINV_msg | 0x02;//bit1 : CC
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVGETCC : %u, %u, %u", 
                INV_GET_CC.Gain_Kpc_RAW, INV_GET_CC.Gain_Kic_RAW, INV_GET_CC.Current_Limit);
            gcs().send_text(MAV_SEVERITY_INFO, "Kpc %f, Kic %f", 
                INV_GET_CC.Gain_Kpc, INV_GET_CC.Gain_Kic);
#endif
            break;
        case RX_ID_INV_GET_SC:
            uint16_temp1 = can_rxframe.data[0];
            uint16_temp2 = can_rxframe.data[1];
            INV_GET_SC.Gain_Kps_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            INV_GET_SC.Gain_Kis_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            INV_GET_SC.Theta_Offset_RAW = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[6];
            uint16_temp2 = can_rxframe.data[7];
            INV_GET_SC.Speed_Limit = uint16_temp2 * 256 + uint16_temp1;
            
            INV_GET_SC.Gain_Kps = (float)INV_GET_SC.Gain_Kps_RAW * 0.01;
            INV_GET_SC.Gain_Kis = (float)INV_GET_SC.Gain_Kis_RAW * 0.1;
            INV_GET_SC.Theta_Offset = (float)INV_GET_SC.Theta_Offset_RAW * 0.1;
            //Speed_Limit has no scale factor

            _NewINV_msg = _NewINV_msg | 0x04;//bit2 : SC
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVGETSC : %u, %u, %u, %u", 
                INV_GET_SC.Gain_Kps_RAW, INV_GET_SC.Gain_Kis_RAW, 
                INV_GET_SC.Theta_Offset_RAW, INV_GET_SC.Speed_Limit);
            gcs().send_text(MAV_SEVERITY_INFO, "Kps %f, Kis %f, thoffset %f", 
                INV_GET_SC.Gain_Kps, INV_GET_SC.Gain_Kis, INV_GET_SC.Theta_Offset);
#endif
            break;
        case RX_ID_INV_GET_FLT:
            INV_GET_FLT.OVL_RAW = can_rxframe.data[0];
            INV_GET_FLT.UVL_RAW = can_rxframe.data[1];
            INV_GET_FLT.OCL_RAW = can_rxframe.data[2];
            INV_GET_FLT.OTL_RAW = can_rxframe.data[3];
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            INV_GET_FLT.OSL_RAW = uint16_temp2 * 256 + uint16_temp1;
            
            INV_GET_FLT.OVL = (uint16_t)INV_GET_FLT.OVL_RAW * 10;
            INV_GET_FLT.UVL = (uint16_t)INV_GET_FLT.UVL_RAW * 10;
            INV_GET_FLT.OCL = (uint16_t)INV_GET_FLT.OCL_RAW * 10;
            INV_GET_FLT.OTL = (uint16_t)INV_GET_FLT.OTL_RAW * 10;
            INV_GET_FLT.OSL = INV_GET_FLT.OSL_RAW;

            _NewINV_msg = _NewINV_msg | 0x08;//bit3 : FLT
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVGETFLT : real %u, %u, %u, %u, %u", 
                INV_GET_FLT.OVL, INV_GET_FLT.UVL, INV_GET_FLT.OCL,
                INV_GET_FLT.OTL, INV_GET_FLT.OSL);
#endif
            break;
        case RX_ID_INV_GET_STATUS1:
            int16_temp1 = can_rxframe.data[0];
            int16_temp2 = can_rxframe.data[1];
            INV_Status1.Motor_Spd_RAW = int16_temp2 * 256 + int16_temp1;
            int16_temp1 = can_rxframe.data[2];
            int16_temp2 = can_rxframe.data[3];
            INV_Status1.i_a_RAW = int16_temp2 * 256 + int16_temp1;
            int16_temp1 = can_rxframe.data[4];
            int16_temp2 = can_rxframe.data[5];
            INV_Status1.i_b_RAW = int16_temp2 * 256 + int16_temp1;
            int16_temp1 = can_rxframe.data[6];
            int16_temp2 = can_rxframe.data[7];
            INV_Status1.i_c_RAW = int16_temp2 * 256 + int16_temp1;

            INV_Status1.motor_Spd = (float)INV_Status1.Motor_Spd_RAW * 0.2;
            INV_Status1.i_a = (float)INV_Status1.i_a_RAW * 0.01;
            INV_Status1.i_b = (float)INV_Status1.i_b_RAW * 0.01;
            INV_Status1.i_c = (float)INV_Status1.i_c_RAW * 0.01;

            _NewINV_msg = _NewINV_msg | 0x10;//bit4 : st1
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVStatus1 : %u, %u, %u, %u", 
                INV_Status1.Motor_Spd_RAW , INV_Status1.i_a_RAW,
                INV_Status1.i_b_RAW, INV_Status1.i_c_RAW);
            gcs().send_text(MAV_SEVERITY_INFO, "MS %f, Ia %f, Ib %f, Ic %f", 
                INV_Status1.motor_Spd, INV_Status1.i_a, 
                INV_Status1.i_b, INV_Status1.i_c);
#endif
            break;
        case RX_ID_INV_GET_STATUS2:
            //Byte0 ~ 7 are not used
            int16_temp1 = can_rxframe.data[6];
            int16_temp2 = can_rxframe.data[7];
            INV_Status2.t_a_RAW = int16_temp2 * 256 + int16_temp1;
            INV_Status2.t_a = (float)INV_Status2.t_a_RAW * 0.01;

            _NewINV_msg = _NewINV_msg | 0x20;//bit5 : st2
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVStatus2 : %d, ta %f", 
                INV_Status2.t_a_RAW, INV_Status2.t_a);
#endif
            break;
        case RX_ID_INV_GET_STATUS3:
            int16_temp1 = can_rxframe.data[0];
            int16_temp2 = can_rxframe.data[1];
            INV_Status3.t_b_RAW = int16_temp2 * 256 + int16_temp1;
            int16_temp1 = can_rxframe.data[2];
            int16_temp2 = can_rxframe.data[3];
            INV_Status3.t_c_RAW = int16_temp2 * 256 + int16_temp1;
            INV_Status3.t_b = (float)INV_Status3.t_b_RAW * 0.01;
            INV_Status3.t_c = (float)INV_Status3.t_c_RAW * 0.01;

            _NewINV_msg = _NewINV_msg | 0x40;//bit6 : st3
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVStatus3 : %d, %d, tb %f, tc %f", 
                INV_Status3.t_b_RAW, INV_Status3.t_c_RAW, INV_Status3.t_b, INV_Status3.t_c);
#endif
            break;
        case RX_ID_INV_GET_STATUS4:
            //Bytes 0 ~ 1 not used
            uint16_temp1 = can_rxframe.data[2];
            uint16_temp2 = can_rxframe.data[3];
            INV_Status4.Flagset.ALL = uint16_temp2 * 256 + uint16_temp1;
            uint16_temp1 = can_rxframe.data[4];
            uint16_temp2 = can_rxframe.data[5];
            INV_Status4.V_dc_input_RAW = uint16_temp2 * 256 + uint16_temp1;
            INV_Status4.MI = can_rxframe.data[6];
            INV_Status4.Motor_Align_flag = can_rxframe.data[7] & 0x01;

            INV_Status4.V_dc_input = (float)INV_Status4.V_dc_input_RAW * 0.1;

            _NewINV_msg = _NewINV_msg | 0x80;//bit7 : st4
#if DEBUG_INVERTER == 1
            gcs().send_text(MAV_SEVERITY_INFO, "INVStatus4 : %u, %u, %u, %u, Vdc %f", 
                INV_Status4.Flagset.ALL, INV_Status4.V_dc_input_RAW, 
                INV_Status4.MI, INV_Status4.Motor_Align_flag, INV_Status4.V_dc_input);
#endif
            break;
        default:

            break;
    }
}

// -------------------------------------------------------------------------
// Check multiple received Inverter data : Check each messages and transfer to global memory
// -------------------------------------------------------------------------
void AP_COAXCAN1::Check_INV_data(void)
{   //To do : check min/max, change for command and etc. later

    //CMD
    if(_NewINV_msg & 0x01) {
        cxdata().INV_data.CMD_Flag.ALL = INV_GET_CMD.BYTE0.ALL;
        cxdata().INV_data.Reference1 = INV_GET_CMD.Reference1;
        cxdata().INV_data.Reference2 = INV_GET_CMD.Reference2;
        if(INV_GET_CMD.BYTE0.bits.Ctrl_Mode==4){
            cxdata().INV_data.Motor_RPM_CMD = INV_GET_CMD.Ref1_RAW / 10;
            cxdata().INV_data.Motor_ACC_CMD = INV_GET_CMD.Ref2_RAW / 10;
        } else {
            //We don't use other modes : reset to zero for RPM and ACC
            cxdata().INV_data.Motor_RPM_CMD = 0;
            cxdata().INV_data.Motor_ACC_CMD = 0;
        }
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x01;
        _INV_has_Initialized |= 0x01;
    }
    //CC
    if(_NewINV_msg & 0x02) {
        cxdata().INV_data.Gain_Kpc = INV_GET_CC.Gain_Kpc;
        cxdata().INV_data.Gain_Kic = INV_GET_CC.Gain_Kic;
        cxdata().INV_data.Current_Limit = INV_GET_CC.Current_Limit;
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x02;
        _INV_has_Initialized |= 0x02;
    }
    //SC
    if(_NewINV_msg & 0x04) {
        cxdata().INV_data.Gain_Kps = INV_GET_SC.Gain_Kps;
        cxdata().INV_data.Gain_Kis = INV_GET_SC.Gain_Kis;
        cxdata().INV_data.Theta_Offset = INV_GET_SC.Theta_Offset;
        cxdata().INV_data.Speed_Limit = INV_GET_SC.Speed_Limit;
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x04;
        _INV_has_Initialized |= 0x04;
    }
    //FLT
    if(_NewINV_msg & 0x08) {
        cxdata().INV_data.OVL = INV_GET_FLT.OVL;
        cxdata().INV_data.UVL = INV_GET_FLT.UVL;
        cxdata().INV_data.OCL = INV_GET_FLT.OCL;
        cxdata().INV_data.OTL = INV_GET_FLT.OTL;
        cxdata().INV_data.OSL = INV_GET_FLT.OSL;
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x08;
        _INV_has_Initialized |= 0x08;
    }
    //status1
    if(_NewINV_msg & 0x10) {
        cxdata().INV_data.motor_Spd = INV_Status1.motor_Spd;
        cxdata().INV_data.i_a = INV_Status1.i_a;
        cxdata().INV_data.i_b = INV_Status1.i_b;
        cxdata().INV_data.i_c = INV_Status1.i_c;

        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x10;
        _INV_has_Initialized |= 0x10;
    }
    //status2
    if(_NewINV_msg & 0x20) {
        cxdata().INV_data.t_a = INV_Status2.t_a;
        
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x20;
        _INV_has_Initialized |= 0x20;
    }
    //status3
    if(_NewINV_msg & 0x40) {
        cxdata().INV_data.t_b = INV_Status3.t_b;
        cxdata().INV_data.t_c = INV_Status3.t_c;
        
        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x40;
        _INV_has_Initialized |= 0x40;
    }
    //status4
    if(_NewINV_msg & 0x80) {
        cxdata().INV_data.FLT.ALL = INV_Status4.Flagset.ALL;
        cxdata().INV_data.V_dc_input = INV_Status4.V_dc_input;
        cxdata().INV_data.MI = INV_Status4.MI;
        cxdata().INV_data.Motor_Align_flag = INV_Status4.Motor_Align_flag;

        cxdata().INV_data.isNew = cxdata().INV_data.isNew | 0x80;
        _INV_has_Initialized |= 0x80;
    }
    _NewINV_msg = 0;
}

// -------------------------------------------------------------------------
// Check multiple received CCB data : Check each messages and transfer to global memory
// -------------------------------------------------------------------------
void AP_COAXCAN1::Check_CCB_data(void)
{
    //MSG1
    if(_NewCC_msg & 0x01) {
        cxdata().CCB_data.Thermistor1x10 = CC_MSG1.Thermistor1x10;
        cxdata().CCB_data.Thermistor2x10 = CC_MSG1.Thermistor2x10;
        cxdata().CCB_data.Thermistor3x10 = CC_MSG1.Thermistor3x10;
        cxdata().CCB_data.Thermistor4x10 = CC_MSG1.Thermistor4x10;
        _CCB_has_Initialized |= 0x01;
    }
    //MSG2
    if(_NewCC_msg & 0x02) {
        cxdata().CCB_data.ThCp1x10 = CC_MSG2.ThCp1x10;
        cxdata().CCB_data.ThCp2x10 = CC_MSG2.ThCp2x10;
        cxdata().CCB_data.Flow_mL = CC_MSG2.Flow_mL;
        cxdata().CCB_data.Brd_temp = CC_MSG2.Brd_temp;
        cxdata().CCB_data.State.ALL = CC_MSG2.State.ALL;
        _CCB_has_Initialized |= 0x02;
    }
    _NewCC_msg = 0;
}

// -------------------------------------------------------------------------
// Transmit data
// -------------------------------------------------------------------------
void AP_COAXCAN1::TXspin()
{   static uint32_t local_count = 0;

    if (local_count%10 == 0) {
        //====Send GCS 61112 command to CCB
        if(cxdata().Command_Received.NewCMD.bits.CCB_ActiveON) {
            //Active Mode on
            cxdata().Command_Received.NewCMD.bits.CCB_ActiveON = 0;
            CC_CMD.Command = 1;
        }else if(cxdata().Command_Received.NewCMD.bits.CCB_Motor_Off) {
            //Motor off
            cxdata().Command_Received.NewCMD.bits.CCB_Motor_Off = 0;
            CC_CMD.Command = 2;
        }else if(cxdata().Command_Received.NewCMD.bits.CCB_Motor_MAX) {
            //Motor maximized
            cxdata().Command_Received.NewCMD.bits.CCB_Motor_MAX = 0;
            CC_CMD.Command = 3;
        }else if(cxdata().Command_Received.NewCMD.bits.CCB_FAN_toggle) {
            //Toggle FAN
            cxdata().Command_Received.NewCMD.bits.CCB_FAN_toggle = 0;
            CC_CMD.Command = 4;
        }else {
            CC_CMD.Command = 5;//Normal data request to CCB
        }
        TX_CCB();
        CC_CMD.Command = 0;

    } else if (local_count%10 == 5) {
    
        //====Send GCS 61110 command to Inverter
        //1) Check ready-to-use state
        if(  (cxdata().INV_data.Rdy2useINV == 0) 
        && (cxdata().INV_data.pre_Rdy2useINV==0) ) {   //Always send something until GCS command is given
            INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = 2;//off (1 is on)
            INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 4;
            INV_SET_CMD.BYTE0.bits.Fault_Clear = 0;
            INV_SET_CMD.Reference1 = 0; //reset rpm
            INV_SET_CMD.Reference2 = 0; //reset acc
            TX_INV_SETCMD_MSG();
        }else if ((cxdata().INV_data.Rdy2useINV == 1) 
        && (cxdata().INV_data.pre_Rdy2useINV == 0) ){  //Detecting rise
            INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = 2;//off (1 is on)
            INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 4;
            INV_SET_CMD.BYTE0.bits.Fault_Clear = 1;
            INV_SET_CMD.Reference1 = 0; //reset rpm
            INV_SET_CMD.Reference2 = 0; //reset acc
            TX_INV_SETCMD_MSG();
        } else {
        //2) After set ready-to-use
            if(cxdata().Command_Received.NewCMD.bits.Inverter_ONOFF) {
                //Command for Inverter On/Off only
                cxdata().Command_Received.NewCMD.bits.Inverter_ONOFF = 0;
                if(_INV_has_Initialized & 0x01) {
                    //when connection is established
                    INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = cxdata().Command_Received.Inv_On_Off; //On/Off from GCS
                    INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 4; //maintain control mode (usually 4 - speed control mode)
                    INV_SET_CMD.BYTE0.bits.Fault_Clear = 1; //clear fault
    #if DEBUG_GCSCMD == 1
                    gcs().send_text(MAV_SEVERITY_INFO, "INV SET OnOff=%u, M=%u, R1=%.0f, R2=%.0f ", 
                        INV_SET_CMD.BYTE0.bits.Inverter_ONOFF, INV_SET_CMD.BYTE0.bits.Ctrl_Mode, 
                        INV_SET_CMD.Reference1, INV_SET_CMD.Reference2);
    #endif
                }else {
                    //send On/Off even if no connection is established yet
                    INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = cxdata().Command_Received.Inv_On_Off;
                    INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 4;   //default mode
                    INV_SET_CMD.BYTE0.bits.Fault_Clear = 1; //clear fault
                    INV_SET_CMD.Reference1 = 0; //reset rpm
                    INV_SET_CMD.Reference2 = 0; //reset acc
    #if DEBUG_GCSCMD == 1
                    gcs().send_text(MAV_SEVERITY_INFO, "Inverter is not connected");
    #endif
                }
                TX_INV_SETCMD_MSG();
            }else if(cxdata().Command_Received.NewCMD.bits.Motor_RPM){
                //Send RPM and ACC command - can only be sent after connection
                cxdata().Command_Received.NewCMD.bits.Motor_RPM = 0;
                INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = INV_GET_CMD.BYTE0.bits.Inverter_ONOFF;//maintain On/Off state
                INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 4; //speed mode
                INV_SET_CMD.BYTE0.bits.Fault_Clear = 1; //no need for fault-clear
                INV_SET_CMD.Reference1 = cxdata().Command_Received.Target_INV_RPM;
                INV_SET_CMD.Reference2 = cxdata().Command_Received.Target_INV_ACC;
    #if DEBUG_GCSCMD == 1
                gcs().send_text(MAV_SEVERITY_INFO, "INV Motor PRM OnOff=%u, R1=%.0f, R2=%.0f ", 
                    INV_SET_CMD.BYTE0.bits.Inverter_ONOFF,  
                    INV_SET_CMD.Reference1, INV_SET_CMD.Reference2);
    #endif
                TX_INV_SETCMD_MSG();
            }else if(cxdata().Command_Received.NewCMD.bits.Inverter_STOP){
                //Emergency Stop
                cxdata().Command_Received.NewCMD.bits.Inverter_STOP = 0;
                INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = 2;
                INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 4; //always maintain speed mode
                INV_SET_CMD.BYTE0.bits.Fault_Clear = 0; //no need for fault-clear
                INV_SET_CMD.Reference1 = INV_GET_CMD.Reference1; //maintain previous values
                INV_SET_CMD.Reference2 = INV_GET_CMD.Reference2;
    #if DEBUG_GCSCMD == 1
                gcs().send_text(MAV_SEVERITY_INFO, "INV Stop command sent OnOff=%u, R1=%.0f, R2=%.0f ", 
                    INV_SET_CMD.BYTE0.bits.Inverter_ONOFF,  
                    INV_SET_CMD.Reference1, INV_SET_CMD.Reference2);
    #endif
                TX_INV_SETCMD_MSG();
            }
        }
        cxdata().INV_data.pre_Rdy2useINV = cxdata().INV_data.Rdy2useINV;
        //====End of GCS 61110 

    } else if (local_count%10 == 6) {
        //====Send GCS 61111 command to Inverter
        if(cxdata().Command_Received.NewCMD.bits.Set_CC) {
            //send Current control parameters
            cxdata().Command_Received.NewCMD.bits.Set_CC = 0;
            INV_SET_CC.Gain_Kpc = cxdata().Command_Received.INVSetValue.Gain_Kpc;
            INV_SET_CC.Gain_Kic = cxdata().Command_Received.INVSetValue.Gain_Kic;
            INV_SET_CC.Current_Limit = cxdata().Command_Received.INVSetValue.Current_Limit;
            TX_INV_SETCC_MSG();
        }
        if(cxdata().Command_Received.NewCMD.bits.Set_SC) {
            //send speed control parameters
            cxdata().Command_Received.NewCMD.bits.Set_SC = 0;
            INV_SET_SC.Gain_Kps = cxdata().Command_Received.INVSetValue.Gain_Kps;
            INV_SET_SC.Gain_Kis = cxdata().Command_Received.INVSetValue.Gain_Kis;
            INV_SET_SC.Theta_Offset = cxdata().Command_Received.INVSetValue.Theta_Offset;
            INV_SET_SC.Speed_Limit = cxdata().Command_Received.INVSetValue.Speed_Limit;
            TX_INV_SETSC_MSG();
        }
        if(cxdata().Command_Received.NewCMD.bits.Set_FLT) {
            //send fault level setting parameters
            cxdata().Command_Received.NewCMD.bits.Set_FLT = 0;
            INV_SET_FLT.OVL = cxdata().Command_Received.INVSetValue.OVL;
            INV_SET_FLT.UVL = cxdata().Command_Received.INVSetValue.UVL;
            INV_SET_FLT.OCL = cxdata().Command_Received.INVSetValue.OCL;
            INV_SET_FLT.OTL = cxdata().Command_Received.INVSetValue.OTL;
            INV_SET_FLT.OSL = cxdata().Command_Received.INVSetValue.OSL;
            TX_INV_SETFLT_MSG();
        }
    }
    local_count++;
}

// -------------------------------------------------------------------------
// Transmit data
// -------------------------------------------------------------------------
int  AP_COAXCAN1::CAN_TX_Ext(uint32_t can_id, uint8_t data_cmd[], uint8_t msgdlc)
{
    int cmd_send_res    = 0;
    uint8_t can_data[8] = {0,0,0,0,0,0,0,0};

    memcpy(can_data, data_cmd, msgdlc);

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::native_micros64() + COAXCAN1_SEND_TIMEOUT_US;                      // Should have timeout value

    out_frame       = {can_id, can_data, msgdlc};                                               // id, data[8], dlc
    cmd_send_res    = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);

    return cmd_send_res;
    
}
int  AP_COAXCAN1::CAN_TX_Std(uint16_t can_id, uint8_t data_cmd[], uint8_t msgdlc)
{
    int cmd_send_res    = 0;
    uint8_t can_data[8] = {0,0,0,0,0,0,0,0};

    memcpy(can_data, data_cmd, msgdlc);

    AP_HAL::CANFrame out_frame;
    uint64_t timeout = AP_HAL::native_micros64() + COAXCAN1_SEND_TIMEOUT_US;                      // Should have timeout value

    out_frame       = {can_id, can_data, msgdlc};                                               // id, data[8], dlc
    cmd_send_res    = _can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError);

    return cmd_send_res;
    
}
// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN1::TX_INV_SETCMD_MSG(void)
{
    // static uint8_t count = 0;
    uint8_t temp_data[8] = {0} ;

    // if(count%2 == 0) {
    //     INV_SET_CMD.BYTE0.ALL = 0;
    //     INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = 0;
    //     INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 0;
    //     INV_SET_CMD.BYTE0.bits.Fault_Clear = 0;
    //     INV_SET_CMD.Reference1 = 0;
    //     INV_SET_CMD.Reference2 = 0;
    // }else {
    //     INV_SET_CMD.BYTE0.ALL = 0xFF;
    //     // INV_SET_CMD.BYTE0.bits.Inverter_ONOFF = 1;
    //     // INV_SET_CMD.BYTE0.bits.Ctrl_Mode = 1;
    //     // INV_SET_CMD.BYTE0.bits.Fault_Clear = 1;
    //     INV_SET_CMD.Reference1 = -0.1;//3276.7;
    //     INV_SET_CMD.Reference2 = -0.1;//-3276.8;
    // }
    // count++;
    if((INV_SET_CMD.Reference1 > 9999.9)||(INV_SET_CMD.Reference1 < -9999.9)) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Bad Parameter for INV command RPM %f", INV_SET_CMD.Reference1);
        return;
    }else if ((INV_SET_CMD.Reference2 > 3276.7)||(INV_SET_CMD.Reference2 < -3276.8)) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Bad Parameter for INV command ACC %f", INV_SET_CMD.Reference2);
        return;
    }
    INV_SET_CMD.Ref1_RAW = (int32_t)(INV_SET_CMD.Reference1 * 10.0);
    INV_SET_CMD.Ref2_RAW = (int16_t)(INV_SET_CMD.Reference2 * 10.0);

    temp_data[0] = INV_SET_CMD.BYTE0.ALL;
    temp_data[1] = (uint8_t)(INV_SET_CMD.Ref1_RAW & 0x000000FF);
    temp_data[2] = (uint8_t)((INV_SET_CMD.Ref1_RAW >> 8) & 0x000000FF);
    temp_data[3] = (uint8_t)((INV_SET_CMD.Ref1_RAW >> 16) & 0x000000FF);
    temp_data[4] = (uint8_t)((INV_SET_CMD.Ref1_RAW >> 24) & 0x000000FF);
    temp_data[5] = (uint8_t)(INV_SET_CMD.Ref2_RAW & 0x00FF);
    temp_data[6] = (uint8_t)((INV_SET_CMD.Ref2_RAW >> 8) & 0x00FF);

    CAN_TX_Ext(_cmd_id[TX_ID::TX_ID_INV_SET_CMD], temp_data, 7);//DLC changed to 5
}

// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN1::TX_INV_SETCC_MSG(void)
{
    uint8_t temp_data[8] = {0} ;

    // INV_SET_CC.Gain_Kpc = 655.35;
    // INV_SET_CC.Gain_Kic = 6553.5;
    // INV_SET_CC.Current_Limit = 180;
    if((INV_SET_CC.Gain_Kpc > 655.35)||(INV_SET_CC.Gain_Kpc < 0)
        ||(INV_SET_CC.Gain_Kic > 6553.5)||(INV_SET_CC.Gain_Kic < 0)
        ||(INV_SET_CC.Current_Limit > 180)) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Bad Parameter for INV command CC");
        return;
    }

    INV_SET_CC.Gain_Kpc_RAW = (uint16_t)(INV_SET_CC.Gain_Kpc * 100);
    INV_SET_CC.Gain_Kic_RAW = (uint16_t)(INV_SET_CC.Gain_Kic * 10);

    temp_data[0] = INV_SET_CC.Gain_Kpc_RAW & 0x00FF;
    temp_data[1] = (INV_SET_CC.Gain_Kpc_RAW >> 8) & 0x00FF;
    temp_data[2] = INV_SET_CC.Gain_Kic_RAW & 0x00FF;
    temp_data[3] = (INV_SET_CC.Gain_Kic_RAW >> 8) & 0x00FF;
    temp_data[4] = INV_SET_CC.Current_Limit & 0x00FF;
    temp_data[5] = (INV_SET_CC.Current_Limit >> 8) & 0x00FF;
    //bytes 6 ~7 reserved => sent with zeros

    CAN_TX_Ext(_cmd_id[TX_ID::TX_ID_INV_SET_CC], temp_data, 8);
}

// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN1::TX_INV_SETSC_MSG(void)
{
    uint8_t temp_data[8] = {0} ;

    // INV_SET_CC.Gain_Kpc = 655.35;
    // INV_SET_CC.Gain_Kic = 6553.5;
    // INV_SET_CC.Current_Limit = 180;
    if((INV_SET_SC.Gain_Kps > 655.35)||(INV_SET_SC.Gain_Kps < 0)
        ||(INV_SET_SC.Gain_Kis > 6553.5)||(INV_SET_SC.Gain_Kis < 0)
        ||(INV_SET_SC.Theta_Offset > 6553.5)||(INV_SET_SC.Theta_Offset < 0)
        ||(INV_SET_SC.Speed_Limit > 5000)) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Bad Parameter for INV command SC");
        return;
    }

    INV_SET_SC.Gain_Kps_RAW = (uint16_t)(INV_SET_SC.Gain_Kps * 100);
    INV_SET_SC.Gain_Kis_RAW = (uint16_t)(INV_SET_SC.Gain_Kis * 10);
    INV_SET_SC.Theta_Offset_RAW = (uint16_t)(INV_SET_SC.Theta_Offset * 10);
    
    temp_data[0] = INV_SET_SC.Gain_Kps_RAW & 0x00FF;
    temp_data[1] = (INV_SET_SC.Gain_Kps_RAW >> 8) & 0x00FF;
    temp_data[2] = INV_SET_SC.Gain_Kis_RAW & 0x00FF;
    temp_data[3] = (INV_SET_SC.Gain_Kis_RAW >> 8) & 0x00FF;
    temp_data[4] = INV_SET_SC.Theta_Offset_RAW & 0x00FF;
    temp_data[5] = (INV_SET_SC.Theta_Offset_RAW >> 8) & 0x00FF;
    temp_data[6] = INV_SET_SC.Speed_Limit & 0x00FF;
    temp_data[7] = (INV_SET_SC.Speed_Limit >> 8) & 0x00FF;

    CAN_TX_Ext(_cmd_id[TX_ID::TX_ID_INV_SET_SC], temp_data, 8);
}

// -------------------------------------------------------------------------
// 
// -------------------------------------------------------------------------
void AP_COAXCAN1::TX_INV_SETFLT_MSG(void)
{
    uint8_t temp_data[8] = {0} ;

    // INV_SET_FLT.OVL = 500;
    // INV_SET_FLT.UVL = 350;
    // INV_SET_FLT.OCL = 200;
    // INV_SET_FLT.OTL = 80;
    // INV_SET_FLT.OSL = 6000;
    if((INV_SET_FLT.OVL>550)||(INV_SET_FLT.UVL>350)||(INV_SET_FLT.OCL>200)
        ||(INV_SET_FLT.OTL>100)||(INV_SET_FLT.OSL>6000)) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Bad Parameter for INV command FLT");
        return;
    }
    INV_SET_FLT.OVL_RAW = INV_SET_FLT.OVL / 10;
    INV_SET_FLT.UVL_RAW = INV_SET_FLT.UVL / 10;
    INV_SET_FLT.OCL_RAW = INV_SET_FLT.OCL / 10;
    INV_SET_FLT.OTL_RAW = INV_SET_FLT.OTL / 10;
    INV_SET_FLT.OSL_RAW = INV_SET_FLT.OSL;

    temp_data[0] = INV_SET_FLT.OVL_RAW;
    temp_data[1] = INV_SET_FLT.UVL_RAW;
    temp_data[2] = INV_SET_FLT.OCL_RAW;
    temp_data[3] = INV_SET_FLT.OTL_RAW;
    temp_data[4] = INV_SET_FLT.OSL_RAW & 0x00FF; //reserved
    temp_data[5] = (INV_SET_FLT.OSL_RAW >> 8) & 0x00FF;;
    //bytes 6 ~7 reserved => sent with zeros

    CAN_TX_Ext(_cmd_id[TX_ID::TX_ID_INV_SET_FLT], temp_data, 6);//DLC changed to 6
}

// -------------------------------------------------------------------------
// Send Command to CCB
// -------------------------------------------------------------------------
void AP_COAXCAN1::TX_CCB(void)
{
    uint8_t temp_data[8] = {0} ;
    uint16_t id = 0x0FE;// | coaxcan1::CanFrame::FlagEFF;
    
    temp_data[0] = CC_CMD.Command;
    //bytes 1 ~7 reserved => sent with zeros

    CAN_TX_Std(id, temp_data, 8);

#if DEBUG_CCB == 1
    gcs().send_text(MAV_SEVERITY_NOTICE, "CCBTX %u", CC_CMD.Command);
#endif
}

void AP_COAXCAN1::interprete_msg(uint8_t sv_id, uint8_t msg_id, uint8_t data_low, uint8_t data_high) {
    uint16_t tempUint16 = 0;
    int16_t tempInt16 = 0;
    //uint8_t isSignedInt = 0;
    if ((sv_id == 0)||(sv_id > 6)||(msg_id>0xC2)) { return; }
    
    uint8_t SV_index = sv_id - 1;

    switch (msg_id) {
        // case REG_PRODUCT_NO :
        //     break;
        // case REG_PRODUCT_VERSION :
        //     break;
        // case REG_FIRMWARE_VERSION :
        //     break;
        // case REG_SERIAL_NO_SUB :
        //     break;
        // case REG_SERIAL_NO_MAIN :
        //     break;
        case REG_STATUS_FLAG :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].ErrorCode.ALL = tempUint16;
            break;
        case REG_POSITION :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            //isSignedInt = 1;
            cxdata().SV_Pos_prv[SV_index].raw = cxdata().SV_Pos[SV_index].raw;
            cxdata().SV_Pos[SV_index].raw = tempInt16;
            break;
        case REG_VELOCITY :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            //isSignedInt = 1;
            cxdata().SV_state[SV_index].Status_Velocity = tempInt16;
            break;
        case REG_TORQUE :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            //isSignedInt = 1;
            cxdata().SV_state[SV_index].Status_Velocity = tempInt16;
            break;
        case REG_VOLTAGE :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Status_Voltage = tempUint16;
            break;
        case REG_MCU_TEMP :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Status_MCU_Temp = tempUint16;
            break;
        case REG_MOTOR_TEMP :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Status_Motor_Temp = tempUint16;
            break;
        case REG_HUMIDITY :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Status_Humidity = tempUint16;
            break;
        // case REG_HUMIDITY_MAX :
        //     break;
        // case REG_HUMIDITY_MIN :
        //     break;
        case REG_POSITION_NEW :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            //isSignedInt = 1;
            cxdata().SV_TXPos_feedback[SV_index] = tempInt16;
            break;
        case REG_VELOCITY_NEW :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Action_Velocity = tempUint16;
            break;
        case REG_TORQUE_NEW :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Action_Torque = tempUint16;
            break;
        // case REG_360DEG_TURN_NEW :
        //     break;
        case REG_SERVO_ID :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            if (sv_id == tempUint16) {
                cxdata().SV_state[SV_index].connected = 1;
            } else {
                gcs().send_text(MAV_SEVERITY_ERROR, "Invalid Servo ID Error %u %u %u %u", sv_id, data_low, data_high, tempUint16);
            }
            break;
        // case REG_BAUD_RATE :
        //     break;
        case REG_NORMAL_RETURN_DELAY :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Config_Delay = tempUint16;
            break;
        case REG_POWER_CONFIG :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Config_Power_Config = tempUint16;
            break;
        case REG_EMERGENCY_STOP :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Config_Emergency_Stop = tempUint16;
            break;
        case REG_ACTION_MODE :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8); //Uint
            cxdata().SV_state[SV_index].Config_Action_Mode = tempUint16;
            break;
        case REG_POSITION_SLOPE :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Pos_Slope = tempUint16;
            break;
        case REG_DEAD_BAND :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Dead_band = tempUint16;
            break;
        case REG_VELOCITY_MAX :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Velocity_Max = tempUint16;
            break;
        case REG_TORQUE_MAX :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Torque_Max = tempUint16;
            break;
        case REG_VOLTAGE_MAX :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Volt_Max = tempUint16;
            break;
        case REG_VOLTAGE_MIN :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Volt_Min = tempUint16;
            break;
        case REG_TEMP_MAX :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Temp_Max = tempUint16;
            break;
        case REG_TEMP_MIN :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Temp_Min = tempUint16;
            break;
        case REG_POS_START :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            //isSignedInt = 1;
            cxdata().SV_state[SV_index].Config_Pos_Start = tempInt16;
            break;
        case REG_POS_END :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            //isSignedInt = 1;
            cxdata().SV_state[SV_index].Config_Pos_End = tempInt16;
            break;
        case REG_POS_NEUTRAL :
            tempUint16 = data_low;
            tempInt16 = (int16_t)(tempUint16 | ((uint16_t)data_high << 8)); //Int
            //isSignedInt = 1;
            cxdata().SV_state[SV_index].Config_Pos_Neutral = tempInt16;
            break;
        // case REG_FACTORY_DEFAULT :
        //     break;
        // case REG_CONFIG_SAVE :
        //     break;
        case REG_MOTOR_TURN_DIRECT :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            cxdata().SV_state[SV_index].Config_Direnction = tempUint16;
            break;
        default :
            tempUint16 = data_low;
            tempUint16 = tempUint16 | ((uint16_t)data_high << 8);  //Uint
            break;
    }
// #if DEBUG_COAXSERVO == 1
//     if(isSignedInt) {
//         gcs().send_text(MAV_SEVERITY_INFO, "HiTech SV %u, MSG %u, Value %d", sv_id, msg_id, tempInt16);
//     } else {
//         gcs().send_text(MAV_SEVERITY_INFO, "HiTech SV %u, MSG %u, Value %u", sv_id, msg_id, tempUint16);
//     }
// #endif
}

// ====== CMD_SET_POSITION
// The CMD_SET_POSITION command makes the actuator run to the defined position.
void AP_COAXCAN1::CMD_SET_POSITION(uint8_t id, int16_t setpoint) {
    //check validity
    if(id>6) return; //Servo ID numbering has changed from 0~5 => 1~6, but 0 is required for initial setting
    if((setpoint < 0) || (setpoint > 2048)) return; //Only allow 90degs turn clockwise and counter-clockwise

    uint8_t buffer[8];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = 0x1E;       //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(setpoint & 0x00FF);
    buffer[5] = (uint8_t)((setpoint >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)
    buffer[7] = 0;

    CAN_TX_Std(RS485_CAN_MSGID, buffer, 8);
}

// ====== CMD_SET_VELOCITY
// The CMD_SET_VELOCITY command makes the actuator run to the defined position.
// 
void AP_COAXCAN1::CMD_SET_VELOCITY(uint8_t id, uint16_t speed) {
    //check validity
    if(id>6) return; //Servo ID numbering has changed from 0~5 => 1~6, but 0 is required for initial setting
    if(speed > 4095) return; //Only allow 4095

    uint8_t buffer[8];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = 0x20;       //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(speed & 0x00FF);
    buffer[5] = (uint8_t)((speed >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)
    buffer[7] = 0;

    CAN_TX_Std(RS485_CAN_MSGID, buffer, 8);
}

// ====== CMD_SET_TORQUE
// The CMD_SET_TORQUE command makes the actuator run to the defined position.
// 
void AP_COAXCAN1::CMD_SET_TORQUE(uint8_t id, uint16_t Trq) {
    //check validity
    if(id>6) return; //Servo ID numbering has changed from 0~5 => 1~6, but 0 is required for initial setting
    if(Trq > 4095) return; //Only allow 4095

    uint8_t buffer[8];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = 0x22;       //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(Trq & 0x00FF);
    buffer[5] = (uint8_t)((Trq >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)
    buffer[7] = 0;

    CAN_TX_Std(RS485_CAN_MSGID, buffer, 8);
}

// ====== Set_Servo_ID
// Only allowed to set up to 6 servos
void AP_COAXCAN1::Set_Servo_ID(uint8_t pre_id, uint8_t aft_id) {
    //check validity
    if((pre_id>6)||(aft_id>6)) return;
    
    uint8_t buffer[8];
    buffer[0] = 0x96;       //Header
    buffer[1] = pre_id;     //Target ID
    buffer[2] = 0x32;       //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = aft_id;       
    buffer[5] = 0;
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)
    buffer[7] = 0;

    CAN_TX_Std(RS485_CAN_MSGID, buffer, 8);
}

void AP_COAXCAN1::Set_UINT_Config(uint8_t id, uint8_t addrs, uint16_t value) {
    //check validity
    if(id>6) return;
    
    uint8_t buffer[8];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = addrs;      //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(value & 0x00FF);
    buffer[5] = (uint8_t)((value >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)
    buffer[7] = 0;

    CAN_TX_Std(RS485_CAN_MSGID, buffer, 8);
}

void AP_COAXCAN1::Set_INT_Config(uint8_t id, uint8_t addrs, int16_t value) {
    //check validity
    if(id>6) return;
    
    uint8_t buffer[8];
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = addrs;      //Address
    buffer[3] = 0x02;       //Registry Length
    buffer[4] = (uint8_t)(value & 0x00FF);
    buffer[5] = (uint8_t)((value >> 8) & 0x00FF);
    buffer[6] = (buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)
    buffer[7] = 0;

    CAN_TX_Std(RS485_CAN_MSGID, buffer, 8);
}

void AP_COAXCAN1::Request_SVData(uint8_t id, uint8_t addrs) {
    //temp test
    static uint8_t count = 0;
    //check validity
    if((id>6)||(addrs > 0xC2)) return;
    
    uint8_t buffer[8] = {0, }; //to avoid TX interrupt delay, we send large amount of data with dummy bytes
    buffer[0] = 0x96;       //Header
    buffer[1] = id;         //Target ID
    buffer[2] = addrs;      //Address
    buffer[3] = 0;          //Registry Length
    buffer[4] = (buffer[1] + buffer[2] + buffer[3]) % 256;   //Checksum : sum(id : Registry Length)%256  (meaning low byte)
    count++;

    CAN_TX_Std(RS485_CAN_MSGID, buffer, 8);

#if DEBUG_COAXSERVO == 1
    //gcs().send_text(MAV_SEVERITY_INFO, "Req SVData to %u for %u", id, addrs);
#endif
}


void AP_COAXCAN1::SV_Config_Test(void) {
    uint8_t NewServoMessages = _num_SVmsg;

    if((cxdata().SVTestState.ServoTestingID > 0) && (cxdata().SVTestState.ServoTestingID < 7)) {
        switch (cxdata().SVTestState.ServoTestStep) {
            case 0 :    //Check ID
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_SERVO_ID); 
                    //gcs().send_text(MAV_SEVERITY_INFO, "Req ID chck SV %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_SERVO_ID) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].connected) {   
                            gcs().send_text(MAV_SEVERITY_INFO, "Checking SV %u Config", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                            cxdata().SVTestState.Request_retry = 0;
                        } else {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_SERVO_ID);//Retry
                            cxdata().SVTestState.Request_retry++;
                        }
                    } 
                    else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 250) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_SERVO_ID);//temp test : TX not working
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next motor instead of next test step
                            gcs().send_text(MAV_SEVERITY_INFO, "SV %u NOT CONNECTED.", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestingID++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 1 :    //Check Return Delay
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_NORMAL_RETURN_DELAY);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Req RETURN DELAY %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_NORMAL_RETURN_DELAY) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Delay == PARAM_RETURN_DELAY) { 
                            //servo 1 : 1ms, servo 2 : 2ms .... delay is same as servo number
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u RETURN DELAY OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_NORMAL_RETURN_DELAY, PARAM_RETURN_DELAY);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Return Delay: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_RETURN_DELAY);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_NORMAL_RETURN_DELAY);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].connected = 0;
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 2 :    //Check Power Config
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POWER_CONFIG);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_POWER_CONFIG %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_POWER_CONFIG) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Power_Config == PARAM_POWER_CONFIG) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Power Config OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_POWER_CONFIG, PARAM_POWER_CONFIG);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Power Config: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_POWER_CONFIG);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POWER_CONFIG);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 3 :    //Check Emergency Stop
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_EMERGENCY_STOP);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_EMERGENCY_STOP %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_EMERGENCY_STOP) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Emergency_Stop == PARAM_EMERGENCY_STOP) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u EM Stop failsafe OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_EMERGENCY_STOP, PARAM_EMERGENCY_STOP);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig EM Stop failsafe: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_EMERGENCY_STOP);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_EMERGENCY_STOP);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 4 :    //Check Action Mode
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_ACTION_MODE);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_ACTION_MODE %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_ACTION_MODE) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Action_Mode == PARAM_ACTION_MODE) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Action Mode OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_ACTION_MODE, PARAM_ACTION_MODE);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Action Mode: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_ACTION_MODE);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_ACTION_MODE);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 5 :    //Check Position Slope
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POSITION_SLOPE);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_POSITION_SLOPE %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_POSITION_SLOPE) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Slope == PARAM_POSITION_SLOPE) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Position Slope OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_POSITION_SLOPE, PARAM_POSITION_SLOPE);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Position Slope: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_POSITION_SLOPE);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POSITION_SLOPE);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 6 :    //Check Dead Band
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_DEAD_BAND);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting REG_DEAD_BAND %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_DEAD_BAND) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Dead_band == PARAM_DEAD_BAND) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Dead Band OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_DEAD_BAND, PARAM_DEAD_BAND);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Dead Band: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_DEAD_BAND);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_DEAD_BAND);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 7 :    //Check Velocity Max
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_VELOCITY_MAX);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Velocity Max %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_VELOCITY_MAX) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Velocity_Max == PARAM_VELOCITY_MAX) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Velocity Max OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_VELOCITY_MAX, PARAM_VELOCITY_MAX);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Velocity Max: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_VELOCITY_MAX);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_VELOCITY_MAX);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 8 :    //Check Torque Max
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_TORQUE_MAX);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Torque Max %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_TORQUE_MAX) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Torque_Max == PARAM_TORQUE_MAX) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Torque Max OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_TORQUE_MAX, PARAM_TORQUE_MAX);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Torque Max: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_TORQUE_MAX);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_TORQUE_MAX);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 9 :    //Check Voltage Max
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_VOLTAGE_MAX);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Voltage Max %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_VOLTAGE_MAX) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Volt_Max == PARAM_VOLTAGE_MAX) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Voltage Max OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_VOLTAGE_MAX, PARAM_VOLTAGE_MAX);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Voltage Max: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_VOLTAGE_MAX);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_VOLTAGE_MAX);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 10 :    //Check Voltage Min
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_VOLTAGE_MIN);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Voltage Min %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_VOLTAGE_MIN) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Volt_Min == PARAM_VOLTAGE_MIN) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Voltage Min OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_VOLTAGE_MIN, PARAM_VOLTAGE_MIN);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Voltage Min: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_VOLTAGE_MIN);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_VOLTAGE_MIN);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 11 :    //Check Temperature Max
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_TEMP_MAX);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Temp Max %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_TEMP_MAX) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Temp_Max == PARAM_TEMP_MAX) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Temp Max OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_TEMP_MAX, PARAM_TEMP_MAX);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Temperature Max: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_TEMP_MAX);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_TEMP_MAX);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 12 :    //Check Temperature Min
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_TEMP_MIN);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Temp Min %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_TEMP_MIN) ) {
                        if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Temp_Min == PARAM_TEMP_MIN) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Temp Min OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_TEMP_MIN, PARAM_TEMP_MIN);
                            gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Temperature Min: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_TEMP_MIN);
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_TEMP_MIN);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            break;
            case 13 :    //Check Position Start
            {   uint8_t isPosStartOK = 0;
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POS_START);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Position Start %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_POS_START) ) {
                        switch(cxdata().SVTestState.ServoTestingID) {
                            case 1 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Start == PARAM_SV1_POS_START) {
                                    isPosStartOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_START, PARAM_SV1_POS_START);
                                    isPosStartOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Start: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV1_POS_START);
                                }
                            break;
                            case 2 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Start == PARAM_SV2_POS_START) {
                                    isPosStartOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_START, PARAM_SV2_POS_START);
                                    isPosStartOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Start: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV2_POS_START);
                                }
                            break;
                            case 3 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Start == PARAM_SV3_POS_START) {
                                    isPosStartOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_START, PARAM_SV3_POS_START);
                                    isPosStartOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Start: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV3_POS_START);
                                }
                            break;
                            case 4 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Start == PARAM_SV4_POS_START) {
                                    isPosStartOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_START, PARAM_SV4_POS_START);
                                    isPosStartOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Start: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV4_POS_START);
                                }
                            break;
                            case 5 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Start == PARAM_SV5_POS_START) {
                                    isPosStartOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_START, PARAM_SV5_POS_START);
                                    isPosStartOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Start: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV5_POS_START);
                                }
                            break;
                            case 6 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Start == PARAM_SV6_POS_START) {
                                    isPosStartOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_START, PARAM_SV6_POS_START);
                                    isPosStartOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Start: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV6_POS_START);
                                }
                            break;
                            default :
                            break;
                        }
                        if(isPosStartOK) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Position Start OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POS_START);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            }
            break;
            case 14 :    //Check Position End
            {   uint8_t isPosEndOK = 0;
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POS_END);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Position End %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_POS_END) ) {
                        switch(cxdata().SVTestState.ServoTestingID) {
                            case 1 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_End == PARAM_SV1_POS_END) {
                                    isPosEndOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_END, PARAM_SV1_POS_END);
                                    isPosEndOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos End: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV1_POS_END);
                                }
                            break;
                            case 2 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_End == PARAM_SV2_POS_END) {
                                    isPosEndOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_END, PARAM_SV2_POS_END);
                                    isPosEndOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos End: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV2_POS_END);
                                }
                            break;
                            case 3 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_End == PARAM_SV3_POS_END) {
                                    isPosEndOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_END, PARAM_SV3_POS_END);
                                    isPosEndOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos End: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV3_POS_END);
                                }
                            break;
                            case 4 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_End == PARAM_SV4_POS_END) {
                                    isPosEndOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_END, PARAM_SV4_POS_END);
                                    isPosEndOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos End: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV4_POS_END);
                                }
                            break;
                            case 5 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_End == PARAM_SV5_POS_END) {
                                    isPosEndOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_END, PARAM_SV5_POS_END);
                                    isPosEndOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos End: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV5_POS_END);
                                }
                            break;
                            case 6 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_End == PARAM_SV6_POS_END) {
                                    isPosEndOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_END, PARAM_SV6_POS_END);
                                    isPosEndOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos End: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV6_POS_END);
                                }
                            break;
                            default :
                            break;
                        }
                        if(isPosEndOK) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Position End OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POS_END);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            }
            break;
            case 15 :    //Check Position Neutral
            {   uint8_t isPosNeutralOK = 0;
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POS_NEUTRAL);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Position Neutral %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_POS_NEUTRAL) ) {
                        switch(cxdata().SVTestState.ServoTestingID) {
                            case 1 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Neutral == PARAM_SV1_POS_NEUTRAL) {
                                    isPosNeutralOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_NEUTRAL, PARAM_SV1_POS_NEUTRAL);
                                    isPosNeutralOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Neutral: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV1_POS_NEUTRAL);
                                }
                            break;
                            case 2 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Neutral == PARAM_SV2_POS_NEUTRAL) {
                                    isPosNeutralOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_NEUTRAL, PARAM_SV2_POS_NEUTRAL);
                                    isPosNeutralOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Neutral: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV2_POS_NEUTRAL);
                                }
                            break;
                            case 3 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Neutral == PARAM_SV3_POS_NEUTRAL) {
                                    isPosNeutralOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_NEUTRAL, PARAM_SV3_POS_NEUTRAL);
                                    isPosNeutralOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Neutral: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV3_POS_NEUTRAL);
                                }
                            break;
                            case 4 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Neutral == PARAM_SV4_POS_NEUTRAL) {
                                    isPosNeutralOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_NEUTRAL, PARAM_SV4_POS_NEUTRAL);
                                    isPosNeutralOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Neutral: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV4_POS_NEUTRAL);
                                }
                            break;
                            case 5 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Neutral == PARAM_SV5_POS_NEUTRAL) {
                                    isPosNeutralOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_NEUTRAL, PARAM_SV5_POS_NEUTRAL);
                                    isPosNeutralOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Neutral: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV5_POS_NEUTRAL);
                                }
                            break;
                            case 6 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Neutral == PARAM_SV6_POS_NEUTRAL) {
                                    isPosNeutralOK = 1;
                                } else {
                                    Set_INT_Config(cxdata().SVTestState.ServoTestingID, REG_POS_NEUTRAL, PARAM_SV6_POS_NEUTRAL);
                                    isPosNeutralOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Pos Neutral: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV6_POS_NEUTRAL);
                                }
                            break;
                            default :
                            break;
                        }
                        if(isPosNeutralOK) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Position Neutral OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POS_NEUTRAL);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }
                }
            }
            break;
            case 16 :    //Check Turn Direction
            {   uint8_t isDirectionOK = 0;
                if(cxdata().SVTestState.SVDataRequested == 0) {
                    Request_SVData(cxdata().SVTestState.ServoTestingID, REG_MOTOR_TURN_DIRECT);
                    //gcs().send_text(MAV_SEVERITY_INFO, "Requesting Turn Direction %u ", cxdata().SVTestState.ServoTestingID);
                    cxdata().SVTestState.SVDataRequested = 1;
                } else {
                    if ( (NewServoMessages) && (_new_SVmsg_ID == REG_MOTOR_TURN_DIRECT) ) {
                        switch(cxdata().SVTestState.ServoTestingID) {
                            case 1 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Direnction == PARAM_SV1_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV1_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Turn Direction: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV1_T_Direction);
                                }
                            break;
                            case 2 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Direnction == PARAM_SV2_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV2_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Turn Direction: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV2_T_Direction);
                                }
                            break;
                            case 3 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Direnction == PARAM_SV3_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV3_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Turn Direction: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV3_T_Direction);
                                }
                            break;
                            case 4 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Direnction == PARAM_SV4_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV4_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Turn Direction: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV4_T_Direction);
                                }
                            break;
                            case 5 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Direnction == PARAM_SV5_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV5_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Turn Direction: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV5_T_Direction);
                                }
                            break;
                            case 6 :
                                if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Direnction == PARAM_SV6_T_Direction) {
                                    isDirectionOK = 1;
                                } else {
                                    Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_MOTOR_TURN_DIRECT, PARAM_SV6_T_Direction);
                                    isDirectionOK = 0;
                                    gcs().send_text(MAV_SEVERITY_INFO, "Reconfig Turn Direction: SV %u Val=%u", cxdata().SVTestState.ServoTestingID, PARAM_SV6_T_Direction);
                                }
                            break;
                            default :
                            break;
                        }
                        if(isDirectionOK) { 
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Turn Direction OK", cxdata().SVTestState.ServoTestingID);
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        } else {
                            cxdata().SVTestState.SVDataRequested = 0;    //Reset SVDataRequested to check again
                            cxdata().SVTestState.SVConfigModified = 1;   //Notice configuration chagne
                            cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                        }
                        cxdata().SVTestState.Request_retry = 0;
                    } else {
                        //Retry up to 20 tiems
                        if(cxdata().SVTestState.Request_retry <= 20) {
                            Request_SVData(cxdata().SVTestState.ServoTestingID, REG_MOTOR_TURN_DIRECT);
                            //gcs().send_text(MAV_SEVERITY_INFO, "RetrySV %u Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);//Delete this line later
                            cxdata().SVTestState.Request_retry++;
                        } else {
                            //Time-over 2sec, Move on to next test step
                            //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Check Error Step %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                            cxdata().SVTestState.Request_retry = 0;
                            cxdata().SVTestState.ServoTestStep++;
                            cxdata().SVTestState.SVDataRequested = 0;
                        }
                    }

                }
            }
            break;
            case 17 :
                if(cxdata().SVTestState.SVConfigModified) {
                    Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_CONFIG_SAVE, 0xFFFF);
                    gcs().send_text(MAV_SEVERITY_INFO, "SV %u Needs Rebooting!!", cxdata().SVTestState.ServoTestingID);
                } 
                cxdata().SVTestState.ServoTestStep = 0;
                cxdata().SVTestState.SVDataRequested = 0;
                cxdata().SVTestState.SVConfigModified = 0;
                if(cxdata().SVTestState.ServoTestingID == 6) {
                    //gcs().send_text(MAV_SEVERITY_INFO, "SV Config Check Finished. Reboot if required");
                    cxdata().SVTestState.ServoCheckFinished = 1;
                    cxdata().SVTestState.ServoTestingID = 1;
                } else {
                    cxdata().SVTestState.ServoTestingID++;
                }
            break;
            default :
            break;
        }
    }

    _num_SVmsg = 0;
}

void AP_COAXCAN1::SV_Check_State(void) 
{   //_AP_COAXCAN1_loop_cnt still usable
    uint8_t NewServoMessages = _num_SVmsg;
    switch (cxdata().SVTestState.ServoTestStep) {
        case 0 : //Read Status
            if(cxdata().SVTestState.SVDataRequested == 0) {
                Request_SVData(cxdata().SVTestState.ServoTestingID, REG_STATUS_FLAG);
                cxdata().SVTestState.SVDataRequested = 1;
            } else {
                if ( (NewServoMessages) && (_new_SVmsg_ID == REG_STATUS_FLAG) && (_new_MSG_SVID == cxdata().SVTestState.ServoTestingID) ) {
                    if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID - 1)].ErrorCode.ALL == 0) {
                        //gcs().send_text(MAV_SEVERITY_INFO, "SV %u Status OK", cxdata().SVTestState.ServoTestingID);
                    } else {
                        //gcs().send_text(MAV_SEVERITY_INFO, "Abnormal Servo %u Status %u", cxdata().SVTestState.ServoTestingID, cxdata().SV_state[(cxdata().SVTestState.ServoTestingID - 1)].ErrorCode.ALL);
                    }
                    cxdata().SVTestState.SVDataRequested = 0;
                    cxdata().SVTestState.Request_retry = 0;
                    if(cxdata().SVTestState.ServoTestingID < 6) {
                        cxdata().SVTestState.ServoTestingID++;
                    } else {
                        cxdata().SVTestState.ServoTestingID = 1;
                        cxdata().SVTestState.ServoTestStep++;
                    }
                } else {
                        //Retry up to 5 tiems
                    if(cxdata().SVTestState.Request_retry <= 15) {
                        Request_SVData(cxdata().SVTestState.ServoTestingID, REG_STATUS_FLAG);
                        cxdata().SVTestState.Request_retry++;
                    } else {
                        //Time-over, Move on to next test step
                        //gcs().send_text(MAV_SEVERITY_INFO, "SV %u S-Check Err Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                        cxdata().SVTestState.SVDataRequested = 0;
                        cxdata().SVTestState.Request_retry = 0;
                        if (cxdata().SVTestState.ServoTestingID < 6) {
                            cxdata().SVTestState.ServoTestingID++;
                        } else {
                            cxdata().SVTestState.ServoTestingID = 1;
                            cxdata().SVTestState.ServoTestStep++;
                        }
                    }
                }
            }
        break;
        case 1 : //Set Position to Neutral
            CMD_SET_POSITION(cxdata().SVTestState.ServoTestingID, cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Neutral);
            //gcs().send_text(MAV_SEVERITY_INFO, "Preset SV %u Pos Neutral", cxdata().SVTestState.ServoTestingID);
            if (cxdata().SVTestState.ServoTestingID < 6) {
                cxdata().SVTestState.ServoTestingID++;
            } else {
                cxdata().SVTestState.ServoTestingID = 1;
                cxdata().SVTestState.ServoTestStep++;
                cxdata().SVTestState.SVDataRequested = 0;
                cxdata().SVTestState.Request_retry = 0;
            }
        break;
        case 2 : //Set Action-Torque
            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_TORQUE_NEW, 4095);
            //gcs().send_text(MAV_SEVERITY_INFO, "Set SV %u Max Trq", cxdata().SVTestState.ServoTestingID);
            if (cxdata().SVTestState.ServoTestingID < 6) {
                cxdata().SVTestState.ServoTestingID++;
            } else {
                cxdata().SVTestState.ServoTestingID = 1;
                cxdata().SVTestState.ServoTestStep++;
                cxdata().SVTestState.SVDataRequested = 0;
                cxdata().SVTestState.Request_retry = 0;
            }
        break;
        case 3 : //Set Action-Velocity
            Set_UINT_Config(cxdata().SVTestState.ServoTestingID, REG_VELOCITY_NEW, 4095);
            //gcs().send_text(MAV_SEVERITY_INFO, "Set SV %u Max Vel", cxdata().SVTestState.ServoTestingID);
            if (cxdata().SVTestState.ServoTestingID < 6) {
                cxdata().SVTestState.ServoTestingID++;
            } else {
                cxdata().SVTestState.ServoTestingID = 1;
                cxdata().SVTestState.ServoTestStep++;
                cxdata().SVTestState.SVDataRequested = 0;
                cxdata().SVTestState.Request_retry = 0;
            }
        break;
        case 4 : //Read Position_New
            if(cxdata().SVTestState.SVDataRequested == 0) {
                Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POSITION_NEW);
                cxdata().SVTestState.SVDataRequested = 1;
            } else {
                if ( (NewServoMessages) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == cxdata().SVTestState.ServoTestingID) ) {
                    if(cxdata().SV_TXPos_feedback[(cxdata().SVTestState.ServoTestingID - 1)] == cxdata().SV_state[(cxdata().SVTestState.ServoTestingID-1)].Config_Pos_Neutral) {
                        gcs().send_text(MAV_SEVERITY_INFO, "SV %u Neutral Preset OK", cxdata().SVTestState.ServoTestingID);
                    } else {
                        gcs().send_text(MAV_SEVERITY_INFO, "Abnormal Servo %u", cxdata().SVTestState.ServoTestingID);
                        cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                    }
                    cxdata().SVTestState.SVDataRequested = 0;
                    cxdata().SVTestState.Request_retry = 0;
                    if(cxdata().SVTestState.ServoTestingID < 6) {
                        cxdata().SVTestState.ServoTestingID++;
                    } else {
                        cxdata().SVTestState.ServoTestingID = 1;
                        cxdata().SVTestState.ServoTestStep++;
                    }
                } else {
                        //Retry up to 5 tiems
                    if(cxdata().SVTestState.Request_retry <= 15) {
                        Request_SVData(cxdata().SVTestState.ServoTestingID, REG_POSITION_NEW);
                        cxdata().SVTestState.Request_retry++;
                    } else {
                        //Time-over, Move on to next test step
                        gcs().send_text(MAV_SEVERITY_INFO, "SV %u S-Check Err Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                        cxdata().SVTestState.SVDataRequested = 0;
                        cxdata().SVTestState.Request_retry = 0;
                        if (cxdata().SVTestState.ServoTestingID < 6) {
                            cxdata().SVTestState.ServoTestingID++;
                        } else {
                            cxdata().SVTestState.ServoTestingID = 1;
                            cxdata().SVTestState.ServoTestStep++;
                        }
                    }
                }
            }
        break;
        case 5 : //Read Action-Torque
        {
            if(cxdata().SVTestState.SVDataRequested == 0) {
                Request_SVData(cxdata().SVTestState.ServoTestingID, REG_TORQUE_NEW);
                cxdata().SVTestState.SVDataRequested = 1;
            } else {
                if ( (NewServoMessages) && (_new_SVmsg_ID == REG_TORQUE_NEW)  && (_new_MSG_SVID == cxdata().SVTestState.ServoTestingID) ) {
                    if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID - 1)].Action_Torque == 4095) {
                        gcs().send_text(MAV_SEVERITY_INFO, "SV %u Max Trq 4095", cxdata().SVTestState.ServoTestingID);
                    } else {
                        gcs().send_text(MAV_SEVERITY_INFO, "Max Trq Not Set for Servo %u", cxdata().SVTestState.ServoTestingID);
                        cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                    }
                    cxdata().SVTestState.SVDataRequested = 0;
                    cxdata().SVTestState.Request_retry = 0;
                    if(cxdata().SVTestState.ServoTestingID < 6) {
                        cxdata().SVTestState.ServoTestingID++;
                    } else {
                        cxdata().SVTestState.ServoTestingID = 1;
                        cxdata().SVTestState.ServoTestStep++;
                    }
                } else {
                        //Retry up to 5 tiems
                    if(cxdata().SVTestState.Request_retry <= 15) {
                        Request_SVData(cxdata().SVTestState.ServoTestingID, REG_STATUS_FLAG);
                        cxdata().SVTestState.Request_retry++;
                    } else {
                        //Time-over, Move on to next test step
                        gcs().send_text(MAV_SEVERITY_INFO, "SV %u S-Check Err Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                        cxdata().SVTestState.SVDataRequested = 0;
                        cxdata().SVTestState.Request_retry = 0;
                        if (cxdata().SVTestState.ServoTestingID < 6) {
                            cxdata().SVTestState.ServoTestingID++;
                        } else {
                            cxdata().SVTestState.ServoTestingID = 1;
                            cxdata().SVTestState.ServoTestStep++;
                        }
                    }
                }
            }
        }
        break;
        case 6 : //Read Action-Velocity
        {
            if(cxdata().SVTestState.SVDataRequested == 0) {
                Request_SVData(cxdata().SVTestState.ServoTestingID, REG_VELOCITY_NEW);
                cxdata().SVTestState.SVDataRequested = 1;
            } else {
                if ( (NewServoMessages) && (_new_SVmsg_ID == REG_VELOCITY_NEW)  && (_new_MSG_SVID == cxdata().SVTestState.ServoTestingID) ) {
                    if(cxdata().SV_state[(cxdata().SVTestState.ServoTestingID - 1)].Action_Torque == 4095) {
                        gcs().send_text(MAV_SEVERITY_INFO, "SV %u Max Vel 4095", cxdata().SVTestState.ServoTestingID);
                    } else {
                        gcs().send_text(MAV_SEVERITY_INFO, "Max Vel Not Set for Servo %u", cxdata().SVTestState.ServoTestingID);
                        cxdata().SVError.SV_Config_Error[(cxdata().SVTestState.ServoTestingID - 1)] = 1;
                    }
                    cxdata().SVTestState.SVDataRequested = 0;
                    cxdata().SVTestState.Request_retry = 0;
                    if(cxdata().SVTestState.ServoTestingID < 6) {
                        cxdata().SVTestState.ServoTestingID++;
                    } else {
                        cxdata().SVTestState.ServoTestingID = 1;
                        cxdata().SVTestState.ServoTestStep++;
                    }
                } else {
                        //Retry up to 5 tiems
                    if(cxdata().SVTestState.Request_retry <= 15) {
                        Request_SVData(cxdata().SVTestState.ServoTestingID, REG_STATUS_FLAG);
                        cxdata().SVTestState.Request_retry++;
                    } else {
                        //Time-over, Move on to next test step
                        gcs().send_text(MAV_SEVERITY_INFO, "SV %u S-Check Err Stp %u", cxdata().SVTestState.ServoTestingID, cxdata().SVTestState.ServoTestStep);
                        cxdata().SVTestState.SVDataRequested = 0;
                        cxdata().SVTestState.Request_retry = 0;
                        if (cxdata().SVTestState.ServoTestingID < 6) {
                            cxdata().SVTestState.ServoTestingID++;
                        } else {
                            cxdata().SVTestState.ServoTestingID = 1;
                            cxdata().SVTestState.ServoTestStep++;
                        }
                    }
                }
            }
        }
        break;
        case 7 :
            cxdata().SVTestState.ServoTestStep = 0;
            cxdata().SVTestState.SVDataRequested = 0;
            cxdata().SVTestState.SVConfigModified = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "SV State Check Finished.");
            cxdata().SVTestState.ServoCheckFinished = 1;

        break;
        default :
        break;
    }

    _num_SVmsg = 0;
}

void AP_COAXCAN1::SV_Waiting_StateLoop(void) {
    
}

//Test verstion 1 : Just send position_new data. One frame per loop
// void AP_COAXCAN1::SV_Waiting_State_TESTLoop(void)
// {
//     static uint32_t loopcount = 0;
//     static uint8_t direction = 1; //0 :decreasing, 1 : increasing
//     static int16_t sv_test_tx = 0;
//     if(loopcount%6 == 0) {
//         if(direction) {
//             sv_test_tx += 10;
//             if(sv_test_tx > 569) {
//                 direction = 0;
//                 sv_test_tx = 569;
//             }
//         } else {
//             sv_test_tx -= 10;
//             if(sv_test_tx < -569) {
//                 direction = 1;
//                 sv_test_tx = -569;
//             }
//         }
//         cxdata().SV_TX[0].SV_pos = sv_test_tx + PARAM_SV1_POS_NEUTRAL;
//         CMD_SET_POSITION(1,cxdata().SV_TX[0].SV_pos);
//     } else if (loopcount%6 == 1) {
//         cxdata().SV_TX[1].SV_pos = sv_test_tx + PARAM_SV2_POS_NEUTRAL;
//         CMD_SET_POSITION(2,cxdata().SV_TX[1].SV_pos);
//     } else if (loopcount%6 == 2) {
//         cxdata().SV_TX[2].SV_pos = sv_test_tx + PARAM_SV2_POS_NEUTRAL;
//         CMD_SET_POSITION(3,cxdata().SV_TX[2].SV_pos);
//     } else if (loopcount%6 == 3) {
//         cxdata().SV_TX[3].SV_pos = sv_test_tx + PARAM_SV4_POS_NEUTRAL;
//         CMD_SET_POSITION(4,cxdata().SV_TX[3].SV_pos);
//     } else if (loopcount%6 == 4) {
//         cxdata().SV_TX[4].SV_pos = sv_test_tx + PARAM_SV5_POS_NEUTRAL;
//         CMD_SET_POSITION(5,cxdata().SV_TX[4].SV_pos);
//     } else if (loopcount%6 == 5) {
//         cxdata().SV_TX[5].SV_pos = sv_test_tx + PARAM_SV6_POS_NEUTRAL;
//         CMD_SET_POSITION(6,cxdata().SV_TX[5].SV_pos);
//     }
//     loopcount++;
// }

//Test verstion 2 : Send three position_new frames per loop
// void AP_COAXCAN1::SV_Waiting_State_TESTLoop(void)
// {    //@100Hz, 3 msg at a time, msg1,2 broken but 3 ok, msg 4,5 broken but 6 OK
//     static uint32_t loopcount = 0;
//     static uint8_t direction = 1; //0 :decreasing, 1 : increasing
//     static int16_t sv_test_tx = 0;
//     if(loopcount%2 == 0) {
//         if(direction) {
//             sv_test_tx += 1;
//             if(sv_test_tx > 569) {
//                 direction = 0;
//                 sv_test_tx = 568;
//             }
//         } else {
//             sv_test_tx -= 1;
//             if(sv_test_tx < -569) {
//                 direction = 1;
//                 sv_test_tx = -568;
//             }
//         }
//         cxdata().SV_TX[0].SV_pos = sv_test_tx + PARAM_SV1_POS_NEUTRAL;
//         CMD_SET_POSITION(1,cxdata().SV_TX[0].SV_pos);
//         cxdata().SV_TX[1].SV_pos = sv_test_tx + PARAM_SV2_POS_NEUTRAL;
//         CMD_SET_POSITION(2,cxdata().SV_TX[1].SV_pos);
//         cxdata().SV_TX[2].SV_pos = sv_test_tx + PARAM_SV2_POS_NEUTRAL;
//         CMD_SET_POSITION(3,cxdata().SV_TX[2].SV_pos);
//     } else if (loopcount%2 == 1) {
//         cxdata().SV_TX[3].SV_pos = sv_test_tx + PARAM_SV4_POS_NEUTRAL;
//         CMD_SET_POSITION(4,cxdata().SV_TX[3].SV_pos);
//         cxdata().SV_TX[4].SV_pos = sv_test_tx + PARAM_SV5_POS_NEUTRAL;
//         CMD_SET_POSITION(5,cxdata().SV_TX[4].SV_pos);
//         cxdata().SV_TX[5].SV_pos = sv_test_tx + PARAM_SV6_POS_NEUTRAL;
//         CMD_SET_POSITION(6,cxdata().SV_TX[5].SV_pos);
//     }
//     loopcount++;
// }

//Test verstion 3 : Set with single SET msgs and check. One frame/loop. Slowly moving. 
//                  This version checks error rate
void AP_COAXCAN1::SV_Waiting_State_TESTLoop(void)
{
    static uint64_t loopcount = 6;
    static int16_t sv_test_tx = 0;
    static uint8_t direction = 1; //0 :decreasing, 1 : increasing
    static uint32_t goodTX = 0;
    static uint32_t badTX = 0;
    static uint8_t localGoodCount = 0;
    static uint16_t globalGoodCount = 0;
    static uint16_t globalBadCount = 0;
    //3 pos set messages at once
    if(loopcount%12 == 0) {
        if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 6) ) {
            goodTX++;
            if (cxdata().SV_TXPos_feedback[5] == cxdata().SV_TX[5].SV_pos) {
                localGoodCount++;
            }
            if(localGoodCount == 6) {
                globalGoodCount++;
            } else {
                globalBadCount++;
            }
            if(direction) {
                sv_test_tx += 1;
                if(sv_test_tx > 569) {
                    direction = 0;
                    sv_test_tx = 568;
                }
            } else {
                sv_test_tx -= 1;
                if(sv_test_tx < -569) {
                    direction = 1;
                    sv_test_tx = -568;
                }
            }
            cxdata().SV_TX[0].SV_pos = sv_test_tx + PARAM_SV1_POS_NEUTRAL;
            CMD_SET_POSITION(1,cxdata().SV_TX[0].SV_pos);
        } else {
            badTX++;
            Request_SVData(6, REG_POSITION_NEW);
            loopcount--;
        }
        
    } else if (loopcount%12 == 1) {
        cxdata().SV_TX[1].SV_pos = sv_test_tx + PARAM_SV2_POS_NEUTRAL;
        CMD_SET_POSITION(2,cxdata().SV_TX[1].SV_pos);
    } else if (loopcount%12 == 2) {
        cxdata().SV_TX[2].SV_pos = sv_test_tx + PARAM_SV3_POS_NEUTRAL;
        CMD_SET_POSITION(3,cxdata().SV_TX[2].SV_pos);
    } else if (loopcount%12 == 3) {
        cxdata().SV_TX[3].SV_pos = sv_test_tx + PARAM_SV4_POS_NEUTRAL;
        CMD_SET_POSITION(4,cxdata().SV_TX[3].SV_pos);
    } else if (loopcount%12 == 4) {
        cxdata().SV_TX[4].SV_pos = sv_test_tx + PARAM_SV5_POS_NEUTRAL;
        CMD_SET_POSITION(5,cxdata().SV_TX[4].SV_pos);
    } else if (loopcount%12 == 5) {
        cxdata().SV_TX[5].SV_pos = sv_test_tx + PARAM_SV6_POS_NEUTRAL;
        CMD_SET_POSITION(6,cxdata().SV_TX[5].SV_pos);
    }
     else if (loopcount%12 == 6) {
        Request_SVData(1, REG_POSITION_NEW);
        if(loopcount%240 == 6) {
            gcs().send_text(MAV_SEVERITY_INFO, "MMTest %u, %u, %u, %lu, %lu", localGoodCount, globalGoodCount, globalBadCount, goodTX, badTX);
        }
        localGoodCount = 0;
    } else if (loopcount%12 == 7) {
        if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 1) ) {
            goodTX++;
            if (cxdata().SV_TXPos_feedback[0] == cxdata().SV_TX[0].SV_pos) {
                localGoodCount++;
            }
            Request_SVData(2, REG_POSITION_NEW);
        } else {
            //Retry
            Request_SVData(1, REG_POSITION_NEW);
            loopcount--;
            badTX++;
        }
    } else if (loopcount%12 == 8) {
        if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 2) ) {
            goodTX++;
            if (cxdata().SV_TXPos_feedback[1] == cxdata().SV_TX[1].SV_pos) {
                localGoodCount++;
            }
            Request_SVData(3, REG_POSITION_NEW);
        } else {
            Request_SVData(2, REG_POSITION_NEW);
            loopcount--;
            badTX++;
        }
    } else if (loopcount%12 == 9) {
        if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 3) ) {
            goodTX++;
            if (cxdata().SV_TXPos_feedback[2] == cxdata().SV_TX[2].SV_pos) {
                localGoodCount++;
            }
            Request_SVData(4, REG_POSITION_NEW);
        } else {
            Request_SVData(3, REG_POSITION_NEW);
            loopcount--;
            badTX++;
        }
    } else if (loopcount%12 == 10) {
        if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 4) ) {
            goodTX++;
            if (cxdata().SV_TXPos_feedback[3] == cxdata().SV_TX[3].SV_pos) {
                localGoodCount++;
            }
            Request_SVData(5, REG_POSITION_NEW);
        } else {
            Request_SVData(4, REG_POSITION_NEW);
            loopcount--;
            badTX++;
        }
    } else if (loopcount%12 == 11) {
        if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 5) ) {
            goodTX++;
            if (cxdata().SV_TXPos_feedback[4] == cxdata().SV_TX[4].SV_pos) {
                localGoodCount++;
            }
            Request_SVData(6, REG_POSITION_NEW);
        } else {
            Request_SVData(5, REG_POSITION_NEW);
            loopcount--;
            badTX++;
        }
    } 

    loopcount++;
    _num_SVmsg = 0;
    _new_SVmsg_ID = 0;
    _new_MSG_SVID = 0;
}

//Test verstion 4 : Set with mulitple SET msgs and check. 2 frame/loop. Slowly moving. 
//                  This version checks error rate
// void AP_COAXCAN1::SV_Waiting_State_TESTLoop(void)
// {
//     //Set with multiple SET msgs and check
//     static uint64_t loopcount = 3;
//     static int16_t sv_test_tx = 0;
//     static uint8_t direction = 1; //0 :decreasing, 1 : increasing
//     static uint32_t goodTX = 0;
//     static uint32_t badTX = 0;
//     static uint8_t localGoodCount = 0;
//     static uint16_t globalGoodCount = 0;
//     static uint16_t globalBadCount = 0;
//     //3 pos set messages at once
//     if(loopcount%9 == 0) {
//         if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 6) ) {
//             goodTX++;
//             if (cxdata().SV_TXPos_feedback[5] == cxdata().SV_TX[5].SV_pos) {
//                 localGoodCount++;
//             }
//             if(localGoodCount == 6) {
//                 globalGoodCount++;
//             } else {
//                 globalBadCount++;
//             }
//             if(direction) {
//                 sv_test_tx += 1;
//                 if(sv_test_tx > 569) {
//                     direction = 0;
//                     sv_test_tx = 568;
//                 }
//             } else {
//                 sv_test_tx -= 1;
//                 if(sv_test_tx < -569) {
//                     direction = 1;
//                     sv_test_tx = -568;
//                 }
//             }
//             cxdata().SV_TX[0].SV_pos = sv_test_tx + PARAM_SV1_POS_NEUTRAL;
//             cxdata().SV_TX[1].SV_pos = sv_test_tx + PARAM_SV2_POS_NEUTRAL;
//             CMD_SET_POSITION(1,cxdata().SV_TX[0].SV_pos);
//             CMD_SET_POSITION(2,cxdata().SV_TX[1].SV_pos);
//         } else {
//             badTX++;
//             Request_SVData(6, REG_POSITION_NEW);
//             loopcount--;
//         }
        
//     } else if (loopcount%9 == 1) {
//         cxdata().SV_TX[2].SV_pos = sv_test_tx + PARAM_SV3_POS_NEUTRAL;
//         cxdata().SV_TX[3].SV_pos = sv_test_tx + PARAM_SV4_POS_NEUTRAL;
//         CMD_SET_POSITION(3,cxdata().SV_TX[2].SV_pos);
//         CMD_SET_POSITION(4,cxdata().SV_TX[3].SV_pos);
//     } else if (loopcount%9 == 2) {
//         cxdata().SV_TX[4].SV_pos = sv_test_tx + PARAM_SV5_POS_NEUTRAL;
//         cxdata().SV_TX[5].SV_pos = sv_test_tx + PARAM_SV6_POS_NEUTRAL;
//         CMD_SET_POSITION(5,cxdata().SV_TX[4].SV_pos);
//         CMD_SET_POSITION(6,cxdata().SV_TX[5].SV_pos);
//     }
//      else if (loopcount%9 == 3) {
//         Request_SVData(1, REG_POSITION_NEW);
//         if(loopcount%160 == 2) {
//             gcs().send_text(MAV_SEVERITY_INFO, "MMTest %u, %u, %u, %lu, %lu", localGoodCount, globalGoodCount, globalBadCount, goodTX, badTX);
//         }
//         localGoodCount = 0;
//     } else if (loopcount%9 == 4) {
//         if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 1) ) {
//             goodTX++;
//             if (cxdata().SV_TXPos_feedback[0] == cxdata().SV_TX[0].SV_pos) {
//                 localGoodCount++;
//             }
//             Request_SVData(2, REG_POSITION_NEW);
//         } else {
//             //Retry
//             Request_SVData(1, REG_POSITION_NEW);
//             loopcount--;
//             badTX++;
//         }
//     } else if (loopcount%9 == 5) {
//         if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 2) ) {
//             goodTX++;
//             if (cxdata().SV_TXPos_feedback[1] == cxdata().SV_TX[1].SV_pos) {
//                 localGoodCount++;
//             }
//             Request_SVData(3, REG_POSITION_NEW);
//         } else {
//             Request_SVData(2, REG_POSITION_NEW);
//             loopcount--;
//             badTX++;
//         }
//     } else if (loopcount%9 == 6) {
//         if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 3) ) {
//             goodTX++;
//             if (cxdata().SV_TXPos_feedback[2] == cxdata().SV_TX[2].SV_pos) {
//                 localGoodCount++;
//             }
//             Request_SVData(4, REG_POSITION_NEW);
//         } else {
//             Request_SVData(3, REG_POSITION_NEW);
//             loopcount--;
//             badTX++;
//         }
//     } else if (loopcount%9 == 7) {
//         if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 4) ) {
//             goodTX++;
//             if (cxdata().SV_TXPos_feedback[3] == cxdata().SV_TX[3].SV_pos) {
//                 localGoodCount++;
//             }
//             Request_SVData(5, REG_POSITION_NEW);
//         } else {
//             Request_SVData(4, REG_POSITION_NEW);
//             loopcount--;
//             badTX++;
//         }
//     } else if (loopcount%9 == 8) {
//         if ( (_num_SVmsg) && (_new_SVmsg_ID == REG_POSITION_NEW) && (_new_MSG_SVID == 5) ) {
//             goodTX++;
//             if (cxdata().SV_TXPos_feedback[4] == cxdata().SV_TX[4].SV_pos) {
//                 localGoodCount++;
//             }
//             Request_SVData(6, REG_POSITION_NEW);
//         } else {
//             Request_SVData(5, REG_POSITION_NEW);
//             loopcount--;
//             badTX++;
//         }
//     } 

//     loopcount++;
//     _num_SVmsg = 0;
//     _new_SVmsg_ID = 0;
//     _new_MSG_SVID = 0;
// }