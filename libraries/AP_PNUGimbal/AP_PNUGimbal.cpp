#include "AP_PNUGimbal.h"

#if AP_PNUGIMBAL_ENABLED

#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_PNUGimbal_CubeMars.h"

extern const AP_HAL::HAL &hal;

AP_PNUGimbal *AP_PNUGimbal::_singleton;

// Parameters live under CAN_Dn_PG_.  The encoding limits are here rather than in the
// source because they MUST match what the vendor PC utility wrote to the motors, and
// that is field configuration: a host/device mismatch does not fail, it silently scales
// every command and every reading.
const AP_Param::GroupInfo AP_PNUGimbal::var_info[] = {

    // @Param: TYPE
    // @DisplayName: PNU gimbal motor protocol
    // @Description: Motor protocol spoken on this CAN driver. 0 leaves the driver idle, which is the default: selecting CAN_Dn_PROTOCOL=16 alone must not start commanding motors.
    // @Values: 0:None,1:CubeMars GLII
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TYPE", 1, AP_PNUGimbal, _type, float(AP_PNUGimbal::Type::NONE)),

    // @Param: MODE
    // @DisplayName: PNU gimbal motor control mode
    // @Description: Control mode the motors are already configured for. This does not change the motor configuration - the vendor PC utility writes that and the motor is power-cycled - it tells the driver which command format to use, and a mismatch is undefined behaviour.
    // @Values: 0:MIT,1:Position-Velocity,2:Velocity
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MODE", 2, AP_PNUGimbal, _mode, 0),

    // @Param: MSTID
    // @DisplayName: PNU gimbal feedback CAN ID
    // @Description: Arbitration ID the motors report feedback on, the Master ID set in the PC utility. All three motors share it and are told apart by the motor ID carried in the payload.
    // @Range: 0 2047
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MSTID", 3, AP_PNUGimbal, _master_id, 0),

    // @Param: ROLID
    // @DisplayName: PNU gimbal roll motor CAN ID
    // @Description: CAN ID of the roll axis motor. Feedback carries only four bits of motor identity, so IDs must be 1 to 15 and the three axes must not share one.
    // @Range: 1 15
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ROLID", 4, AP_PNUGimbal, _motor_id[AP_PNUGIMBAL_AXIS_ROLL], 1),

    // @Param: PITID
    // @DisplayName: PNU gimbal pitch motor CAN ID
    // @Description: CAN ID of the pitch axis motor. Feedback carries only four bits of motor identity, so IDs must be 1 to 15 and the three axes must not share one.
    // @Range: 1 15
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PITID", 5, AP_PNUGimbal, _motor_id[AP_PNUGIMBAL_AXIS_PITCH], 2),

    // @Param: YAWID
    // @DisplayName: PNU gimbal yaw motor CAN ID
    // @Description: CAN ID of the yaw axis motor. Feedback carries only four bits of motor identity, so IDs must be 1 to 15 and the three axes must not share one.
    // @Range: 1 15
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("YAWID", 6, AP_PNUGimbal, _motor_id[AP_PNUGIMBAL_AXIS_YAW], 3),

    // @Param: PMAX
    // @DisplayName: PNU gimbal position encoding limit
    // @Description: Position encoding limit, used as plus and minus this value. Must match the range configured in the motor, not the mechanical travel of the mount.
    // @Units: rad
    // @Range: 0.1 100
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PMAX", 7, AP_PNUGimbal, _p_max, 12.5f),

    // @Param: VMAX
    // @DisplayName: PNU gimbal velocity encoding limit
    // @Description: Velocity encoding limit, used as plus and minus this value. Must match the range configured in the motor. The manual is inconsistent about whether this field is rad/s or rev/s - confirm with a measured low-speed rotation before trusting the physical units.
    // @Units: rad/s
    // @Range: 1 1000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("VMAX", 8, AP_PNUGimbal, _v_max, 200.0f),

    // @Param: KPMAX
    // @DisplayName: PNU gimbal stiffness encoding limit
    // @Description: Stiffness encoding limit for MIT mode, used as 0 to this value. Must match the range configured in the motor.
    // @Range: 1 5000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("KPMAX", 9, AP_PNUGimbal, _kp_max, 500.0f),

    // @Param: KDMAX
    // @DisplayName: PNU gimbal damping encoding limit
    // @Description: Damping encoding limit for MIT mode, used as 0 to this value. Must match the range configured in the motor.
    // @Range: 0.1 100
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("KDMAX", 10, AP_PNUGimbal, _kd_max, 5.0f),

    // @Param: TMAX
    // @DisplayName: PNU gimbal torque encoding limit
    // @Description: Torque encoding limit in newton metres, used as plus and minus this value. This is an encoding range and not a permissible torque for the fitted motor. Must match the range configured in the motor.
    // @Range: 0.1 500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TMAX", 11, AP_PNUGimbal, _t_max, 10.0f),

    AP_GROUPEND
};

AP_PNUGimbal::AP_PNUGimbal()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_PNUGimbal must be singleton");
    }
    _singleton = this;
}

// _backend is deliberately NOT deleted here.  The loop thread holds a reference to it
// for the life of the vehicle and is never stopped, so freeing it would be a
// use-after-free rather than a tidy-up.  This destructor is unreachable in practice.
AP_PNUGimbal::~AP_PNUGimbal()
{
    _singleton = nullptr;
}

// return the driver registered on @driver_index, or nullptr if it is not ready
AP_PNUGimbal *AP_PNUGimbal::get_pnugimbal(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::PNUGimbal) {
        return nullptr;
    }
    return static_cast<AP_PNUGimbal*>(AP::can().get_driver(driver_index));
}

bool AP_PNUGimbal::add_interface(AP_HAL::CANIface *can_iface)
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

void AP_PNUGimbal::init(uint8_t driver_index)
{
    if (_initialized || (_can_iface == nullptr)) {
        return;
    }

    // assign only on the call that actually starts the driver, so a second init() cannot
    // repoint it while the loop thread is already running (AP_PMUCAN precedent)
    _driver_index = driver_index;

    // A failure here is not fatal and does not skip the thread: the thread is what
    // reports the failure, once report_startup() can expect a GCS to be listening.  With
    // no backend it does nothing else.
    create_backend();

    snprintf(_thread_name, sizeof(_thread_name), "pnugimbal");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PNUGimbal::loop, void),
                                      _thread_name, 2048,
                                      AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;
}

bool AP_PNUGimbal::create_backend(void)
{
    switch (Type(_type.get())) {
    case Type::NONE:
        // not an error - report_startup() says so at INFO and the driver idles
        return false;

    case Type::CUBEMARS:
#if AP_PNUGIMBAL_CUBEMARS_ENABLED
        _backend = NEW_NOTHROW AP_PNUGimbal_CubeMars(*this);
        break;
#else
        set_init_error("PG_TYPE 1 (CubeMars) is not in this build");
        return false;
#endif

    default:
        set_init_error("PG_TYPE %d is not a known backend", (int)_type.get());
        return false;
    }

    if (_backend == nullptr) {
        set_init_error("backend allocation failed");
        return false;
    }

    if (!_backend->init()) {
        // init() has already recorded why through set_init_error()
        delete _backend;
        _backend = nullptr;
        return false;
    }

    return true;
}

void AP_PNUGimbal::set_init_error(const char *fmt, ...)
{
    // keep the first failure: it is the one that explains the rest
    if (_init_error[0] != '\0') {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(_init_error, sizeof(_init_error), fmt, ap);
    va_end(ap);
}

// thread entry.  All CAN work happens here; nothing runs on the vehicle scheduler.
void AP_PNUGimbal::loop(void)
{
    const uint32_t period_us = 1000000UL / AP_PNUGIMBAL_LOOP_HZ;

    while (true) {
        hal.scheduler->delay_microseconds(period_us);

        if (!_initialized || (_can_iface == nullptr)) {
            continue;
        }

        _loop_cnt++;

        report_startup();

        if (_backend == nullptr) {
            continue;
        }

        rx_spin();
        tx_spin();
    }
}

// One line per boot saying what this driver is doing, sent late enough that a GCS is
// listening.  A driver that is configured but cannot run must say so - a misconfigured
// CAN_Dn_PG_* that produced silence would be the PNU-ISSUE(D6) problem again.
void AP_PNUGimbal::report_startup(void)
{
    if (_startup_reported || (AP_HAL::millis() < AP_PNUGIMBAL_STARTUP_REPORT_MS)) {
        return;
    }
    _startup_reported = true;

    if (_backend != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PNUGimbal: CAN_D%u %s ready",
                      (unsigned)(_driver_index + 1), _backend->name());
    } else if (_init_error[0] != '\0') {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNUGimbal: CAN_D%u %s",
                      (unsigned)(_driver_index + 1), _init_error);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PNUGimbal: CAN_D%u_PG_TYPE is 0, idle",
                      (unsigned)(_driver_index + 1));
    }
}

// Drain the interface and hand every frame to the backend.
//
// Each pass receives at the top of the loop, so a frame can never be dequeued and then
// dropped when the budget runs out.  AP_PMUCAN::RXdrain() receives at the bottom and
// needs a trailing handleFrame() to cover exactly that case; this shape does not.
void AP_PNUGimbal::rx_spin(void)
{
    bool read_select = true;
    bool write_select = false;
    const uint64_t deadline = AP_HAL::micros64() + AP_PNUGIMBAL_SELECT_TIMEOUT_US;

    // select() blocks for up to the deadline, so it is deliberately outside the
    // semaphore below - holding _sem across it would stall the vehicle thread
    if (!_can_iface->select(read_select, write_select, nullptr, deadline)) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    const uint64_t drain_start_us = AP_HAL::micros64();
    for (uint8_t frames = 0; frames < AP_PNUGIMBAL_RX_MAX_FRAMES; frames++) {
        if ((AP_HAL::micros64() - drain_start_us) >= AP_PNUGIMBAL_RX_MAX_TIME_US) {
            break;
        }

        AP_HAL::CANFrame frame;
        uint64_t rx_time_us = 0;
        AP_HAL::CANIface::CanIOFlags flags = 0;
        if (_can_iface->receive(frame, rx_time_us, flags) <= 0) {
            break;
        }
        if (flags & AP_HAL::CANIface::Loopback) {
            continue;
        }

        _backend->handle_frame(frame);
    }
}

void AP_PNUGimbal::tx_spin(void)
{
    WITH_SEMAPHORE(_sem);
    _backend->update();
}

bool AP_PNUGimbal::send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if ((_can_iface == nullptr) || (data == nullptr) ||
        (dlc == 0) || (dlc > AP_HAL::CANFrame::NonFDCANMaxDataLen) ||
        (can_id > AP_HAL::CANFrame::MaskStdID)) {
        return false;
    }

    // standard 11-bit data frame on classical CAN: FlagEFF and FlagRTR are both left
    // clear, and no backend here has any use for an extended or remote frame
    const AP_HAL::CANFrame out_frame { can_id, data, dlc };
    const uint64_t deadline = AP_HAL::micros64() + AP_PNUGIMBAL_SEND_TIMEOUT_US;

    const int16_t res = _can_iface->send(out_frame, deadline, AP_HAL::CANIface::AbortOnError);
    if (res != 1) {
        // 0 is a full TX buffer and negative is an interface error; both mean the frame
        // did not go, which is all a backend needs to know
        report_tx_error(can_id, res);
        return false;
    }
    return true;
}

// Report a frame rejected for a short DLC, rate limited, with the first always reported.
// Same shape as AP_PMUCAN::report_short_frame() - a rejection nobody sees is the silence
// PNU-ISSUE(D6) set out to remove.
void AP_PNUGimbal::report_short_frame(uint32_t frame_id, uint8_t dlc)
{
    _short_frame_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_short_frame_cnt > 1) &&
        ((now_ms - _last_short_frame_warn_ms) < AP_PNUGIMBAL_SHORT_FRAME_WARN_MS)) {
        return;
    }
    _last_short_frame_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNUGimbal: short frame 0x%x dlc %u (%u dropped)",
                  (unsigned)frame_id, (unsigned)dlc, (unsigned)_short_frame_cnt);
}

// Its own rate limiter, not the short-frame one: a bus that is down would otherwise
// produce a send failure every tick and crowd out every other warning this driver has.
void AP_PNUGimbal::report_tx_error(uint32_t can_id, int result)
{
    _tx_err_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_tx_err_cnt > 1) &&
        ((now_ms - _last_tx_err_warn_ms) < AP_PNUGIMBAL_TX_ERR_WARN_MS)) {
        return;
    }
    _last_tx_err_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNUGimbal: send 0x%x failed %d (%u dropped)",
                  (unsigned)can_id, result, (unsigned)_tx_err_cnt);
}

uint8_t AP_PNUGimbal::get_motor_id(uint8_t axis) const
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        // 0 is not a configurable id, so a backend's own validation rejects this
        return 0;
    }
    return uint8_t(_motor_id[axis].get());
}

// ---------------------------------------------------------------------------
// Interface for a control layer.  Every one of these is a no-op with no backend, so a
// caller never has to know whether this driver is configured.
// ---------------------------------------------------------------------------

void AP_PNUGimbal::set_target(uint8_t axis, const AP_PNUGimbal_Target &target)
{
    WITH_SEMAPHORE(_sem);
    if (_backend == nullptr) {
        return;
    }
    _backend->set_target(axis, target);
}

bool AP_PNUGimbal::get_feedback(uint8_t axis, AP_PNUGimbal_Feedback &feedback)
{
    WITH_SEMAPHORE(_sem);
    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_feedback(axis, feedback);
}

void AP_PNUGimbal::set_enable(bool enable)
{
    WITH_SEMAPHORE(_sem);
    if (_backend == nullptr) {
        return;
    }
    _backend->set_enable(enable);
}

void AP_PNUGimbal::request_zero(uint8_t axis)
{
    WITH_SEMAPHORE(_sem);
    if (_backend == nullptr) {
        return;
    }
    _backend->request_zero(axis);
}

void AP_PNUGimbal::request_clear_errors(uint8_t axis)
{
    WITH_SEMAPHORE(_sem);
    if (_backend == nullptr) {
        return;
    }
    _backend->request_clear_errors(axis);
}

bool AP_PNUGimbal::healthy(void)
{
    WITH_SEMAPHORE(_sem);
    if (_backend == nullptr) {
        return false;
    }
    return _backend->healthy();
}

namespace AP {

AP_PNUGimbal *pnugimbal()
{
    return AP_PNUGimbal::get_singleton();
}

};

#endif  // AP_PNUGIMBAL_ENABLED
