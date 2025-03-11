#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_CoaxCAN1.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_CANManager/AP_CANManager.h>

extern const AP_HAL::HAL& hal;

#define TEMP_EXP_INITVAL 0		//Initial value

// Table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_COAXCAN1::var_info[] = {
    // @Param: example
    // @DisplayName: User example
    // @Description: Example for user
    AP_GROUPINFO("Examp", 1, AP_COAXCAN1, _examp, TEMP_EXP_INITVAL),
    //AP_GROUPINFO("Examp", 1, AP_COAXCAN, TEMP_EXP_INITVAL, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM1", 1, AP_COAXCAN, _pmu_param1, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM2", 2, AP_COAXCAN, _pmu_param2, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM3", 3, AP_COAXCAN, _pmu_param3, TEMP_EXP_INITVAL),
    // AP_GROUPINFO("PARAM4", 4, AP_COAXCAN, _pmu_param4, TEMP_EXP_INITVAL),

    AP_GROUPEND
};

AP_COAXCAN1::AP_COAXCAN1()
{
    AP_Param::setup_object_defaults(this, var_info);
    
    _examp.set_default(0);

    _initialized    = false;
    
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

    //snprintf(_thread_name, sizeof(_thread_name), "pmucan");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_COAXCAN1::loop, void), _thread_name, 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;


    // gcs().send_text(MAV_SEVERITY_INFO, "[COAX] Initialized");
}

// -------------------------------------------------------------------------
// Task
// -------------------------------------------------------------------------
void AP_COAXCAN1::loop(void)
{
   /* while (true)
    {
        if (!_initialized)
        {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        hal.scheduler->delay_microseconds(pmucan_period_us);    // 10ms period loop

        if(_AP_COAXCAN1_loop_cnt%COAXCAN_MINOR_INTERVAL==0)        // 20ms period run
            run();
            
        if(_AP_COAXCAN1_loop_cnt%COAXCAN_MALVINK_INTERVAL==0)      // 100ms period send2ppc
        {
            send2gcs();     // Send COAX Data to GCS
            // gcs().send_text(MAV_SEVERITY_INFO, "[COAX] send2gcs"); // KAL
        }

        if(_AP_COAXCAN1_loop_cnt%COAXCAN_ONRUNNING_INTERVAL==0)      // 2 sec period KAL
        {
        //     gcs().send_text(MAV_SEVERITY_INFO, "[COAX] %8ld V%5d %5d", COAX_Status.Date, COAX_Status.System_Voltage*100, COAX_Status.COAX_Status); // KAL
            sendchanges();
        }

        _AP_COAXCAN1_loop_cnt++;                                  // 10ms period increase
    }*/
}
