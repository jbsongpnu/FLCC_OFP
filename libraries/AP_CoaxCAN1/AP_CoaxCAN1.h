#ifndef AP_COAXCAN1_H_
#define AP_COAXCAN1_H_

#include <AP_Common/AP_Common.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>

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

private:

    char _thread_name[9];
    bool _initialized;
    uint8_t _driver_index;                                          // JBSong
    AP_HAL::CANIface* _can_iface;                                   // JBSong => alternative handler to _iface
                                                                    // '_can_iface' is acquired from new add_interface() function
    HAL_EventHandle _event_handle;                                  // JBSong

    //pmucan::ICanIface* _iface;                                      // igpark
    AP_Int8 _examp;
};

#endif