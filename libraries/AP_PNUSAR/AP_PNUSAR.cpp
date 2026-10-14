#include "AP_PNUSAR.h"

#if AP_PNUSAR_ENABLED

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_PNUSAR *AP_PNUSAR::_singleton;

AP_PNUSAR::AP_PNUSAR()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_PNUSAR must be singleton");
    }
    _singleton = this;
}

AP_PNUSAR::~AP_PNUSAR()
{
    _singleton = nullptr;
}

// return the driver registered on @driver_index, or nullptr if it is not ready
AP_PNUSAR *AP_PNUSAR::get_pnusar(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::PNUSAR) {
        return nullptr;
    }
    return static_cast<AP_PNUSAR*>(AP::can().get_driver(driver_index));
}

bool AP_PNUSAR::add_interface(AP_HAL::CANIface *can_iface)
{
    if (_can_iface != nullptr) {
        // only one interface is supported, as for AP_PMUCAN
        return false;
    }
    if (can_iface == nullptr) {
        return false;
    }
    if (!can_iface->set_event_handle(&sem_handle)) {
        return false;
    }
    _can_iface = can_iface;
    return true;
}

void AP_PNUSAR::init(uint8_t driver_index)
{
    if (_initialized || (_can_iface == nullptr)) {
        return;
    }

    // assign only on the call that actually starts the driver, so a second init() cannot
    // repoint it while the loop thread is already running (AP_PMUCAN precedent)
    _driver_index = driver_index;

    snprintf(_thread_name, sizeof(_thread_name), "pnusar");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PNUSAR::loop, void),
                                      _thread_name, 2048,
                                      AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;
}

// thread entry.  All CAN work happens here; nothing runs on the vehicle scheduler.
void AP_PNUSAR::loop(void)
{
    const uint32_t period_us = 1000000UL / AP_PNUSAR_LOOP_HZ;

    while (true) {
        hal.scheduler->delay_microseconds(period_us);

        if (!_initialized || (_can_iface == nullptr)) {
            continue;
        }

        _loop_cnt++;

        rx_spin();
        tx_spin();
    }
}

// TODO(AP_PNUSAR) : drain the interface and pass each frame to handle_frame().
// See AP_PMUCAN::RXspin()/RXdrain() for the shape - in particular, drain until empty so a
// burst cannot back up, and do not block.
void AP_PNUSAR::rx_spin(void)
{
}

// TODO(AP_PNUSAR) : send whatever is pending.  Check the result of every send and surface
// failures - a send error that nobody sees is the D6 problem again.
void AP_PNUSAR::tx_spin(void)
{
}

// TODO(AP_PNUSAR) : decode one frame.
//
// Before reading ANY field, check can_rxframe.dlc against the expected length and reject
// anything short via report_short_frame().  AP_PMUCAN parsed from fixed offsets regardless
// of DLC for a long time and yielded silently stale values - that is PNU-ISSUE(D6).
//
// Unknown ids must return early rather than falling through the switch.
void AP_PNUSAR::handle_frame(const AP_HAL::CANFrame &frame)
{
    (void)frame;
}

// Report a frame rejected for a short DLC, rate limited, with the first always reported.
// Same shape as AP_PMUCAN::report_short_frame() - a rejection nobody sees is the silence
// D6 set out to remove.
void AP_PNUSAR::report_short_frame(uint32_t frame_id, uint8_t dlc)
{
    _short_frame_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_short_frame_cnt > 1) &&
        ((now_ms - _last_short_frame_warn_ms) < AP_PNUSAR_SHORT_FRAME_WARN_MS)) {
        return;
    }
    _last_short_frame_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNUSAR: short frame 0x%x dlc %u (%u dropped)",
                  (unsigned)frame_id, (unsigned)dlc, (unsigned)_short_frame_cnt);
}

// TODO(AP_PNUSAR) : write one frame.  Returns 0 on success, negative on failure.
int AP_PNUSAR::send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    (void)can_id;
    (void)data;
    (void)dlc;
    return -1;
}

namespace AP {

AP_PNUSAR *pnusar()
{
    return AP_PNUSAR::get_singleton();
}

};

#endif  // AP_PNUSAR_ENABLED
