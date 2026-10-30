#include "AP_PNU_CAN.h"

#if AP_PNU_CAN_ENABLED

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_PMUCAN/AP_PMUCAN.h>
#include <AP_PNUSAR/AP_PNUSAR.h>
#include <AP_PNUGimbal/AP_PNUGimbal_CubeMars.h>

extern const AP_HAL::HAL &hal;

AP_PNU_CAN *AP_PNU_CAN::_singleton;

// Parameters live under CAN_Dn_PNU_, one flat group for the whole driver.
//
// They are NOT split into a per-device subgroup, and that is a constraint rather than a
// preference: AP_SUBGROUPPTR resolves var_info from the class (AP_Param.h:165), so two
// sibling subgroups both pointing at this one object would get the same table, and
// nesting costs so much of the 16-character name budget that the gimbal's parameters
// become unreadable.  CAN_Dn_PNU_ is 11 characters, which leaves 5.
//
// The gimbal's encoding limits are parameters rather than constants because they MUST
// match what the vendor PC utility wrote to the motors: a host/device mismatch does not
// fail, it silently rescales every command and every reading.
const AP_Param::GroupInfo AP_PNU_CAN::var_info[] = {

    // @Param: OPTS
    // @DisplayName: PNU CAN devices present
    // @Description: Which PNU devices are on this CAN bus. A device that is not selected is not created at all. 0 leaves the driver allocated with no backends, which is a valid state - the bus is wired and everything is disabled - and is reported once per boot.
    // @Bitmask: 0:PMU,1:Gimbal,2:SAR
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OPTS", 1, AP_PNU_CAN, _opts, AP_PNU_CAN::DEVICE_PMU),

    // @Param: TYPE
    // @DisplayName: PNU gimbal motor protocol
    // @Description: Which motor protocol the gimbal speaks. Whether a gimbal is fitted at all is PNU_OPTS bit 1, not this parameter.
    // @Values: 1:CubeMars GLII
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TYPE", 2, AP_PNU_CAN, _gimbal_type, 1),

    // @Param: MODE
    // @DisplayName: PNU gimbal motor control mode
    // @Description: Control mode the gimbal motors are already configured for. This does not change the motor configuration - the vendor PC utility writes that and the motor is power-cycled - it tells the driver which command format to use, and a mismatch is undefined behaviour.
    // @Values: 0:MIT,1:Position-Velocity,2:Velocity
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MODE", 3, AP_PNU_CAN, _gimbal_mode, 0),

    // @Param: MSTID
    // @DisplayName: PNU gimbal feedback CAN ID
    // @Description: Arbitration ID the gimbal motors report feedback on, the Master ID set in the PC utility. All three motors share it and are told apart by the motor ID carried in the payload.
    // @Range: 0 2047
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MSTID", 4, AP_PNU_CAN, _master_id, 0),

    // @Param: ROLID
    // @DisplayName: PNU gimbal roll motor CAN ID
    // @Description: CAN ID of the roll axis motor. Feedback carries only four bits of motor identity, so IDs must be 1 to 15 and the three axes must not share one.
    // @Range: 1 15
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ROLID", 5, AP_PNU_CAN, _motor_id[AP_PNUGIMBAL_AXIS_ROLL], 1),

    // @Param: PITID
    // @DisplayName: PNU gimbal pitch motor CAN ID
    // @Description: CAN ID of the pitch axis motor. Feedback carries only four bits of motor identity, so IDs must be 1 to 15 and the three axes must not share one.
    // @Range: 1 15
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PITID", 6, AP_PNU_CAN, _motor_id[AP_PNUGIMBAL_AXIS_PITCH], 2),

    // @Param: YAWID
    // @DisplayName: PNU gimbal yaw motor CAN ID
    // @Description: CAN ID of the yaw axis motor. Feedback carries only four bits of motor identity, so IDs must be 1 to 15 and the three axes must not share one.
    // @Range: 1 15
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("YAWID", 7, AP_PNU_CAN, _motor_id[AP_PNUGIMBAL_AXIS_YAW], 3),

    // @Param: PMAX
    // @DisplayName: PNU gimbal position encoding limit
    // @Description: Position encoding limit, used as plus and minus this value. Must match the range configured in the motor, not the mechanical travel of the mount.
    // @Units: rad
    // @Range: 0.1 100
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PMAX", 8, AP_PNU_CAN, _p_max, 12.5f),

    // @Param: VMAX
    // @DisplayName: PNU gimbal velocity encoding limit
    // @Description: Velocity encoding limit, used as plus and minus this value. Must match the range configured in the motor. The manual is inconsistent about whether this field is rad/s or rev/s - confirm with a measured low-speed rotation before trusting the physical units.
    // @Units: rad/s
    // @Range: 1 1000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("VMAX", 9, AP_PNU_CAN, _v_max, 200.0f),

    // @Param: KPMAX
    // @DisplayName: PNU gimbal stiffness encoding limit
    // @Description: Stiffness encoding limit for MIT mode, used as 0 to this value. Must match the range configured in the motor.
    // @Range: 1 5000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("KPMAX", 10, AP_PNU_CAN, _kp_max, 500.0f),

    // @Param: KDMAX
    // @DisplayName: PNU gimbal damping encoding limit
    // @Description: Damping encoding limit for MIT mode, used as 0 to this value. Must match the range configured in the motor.
    // @Range: 0.1 100
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("KDMAX", 11, AP_PNU_CAN, _kd_max, 5.0f),

    // @Param: TMAX
    // @DisplayName: PNU gimbal torque encoding limit
    // @Description: Torque encoding limit in newton metres, used as plus and minus this value. This is an encoding range and not a permissible torque for the fitted motor. Must match the range configured in the motor.
    // @Range: 0.1 500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TMAX", 12, AP_PNU_CAN, _t_max, 10.0f),

    // @Param: MRLL
    // @DisplayName: PNU gimbal roll mechanical limit
    // @Description: Mechanical travel limit of the roll axis, used as plus and minus this value. This is the hard stop, not the operating range - keep MNT1_ROLL_MIN/MAX inside it with a margin for stabilisation overshoot. Applied as the last clamp before a motor frame is built, so a fault in the targeting layer cannot drive the payload into its stops.
    // @Units: deg
    // @Range: 1 180
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MRLL", 13, AP_PNU_CAN, _mech_limit_deg[AP_PNUGIMBAL_AXIS_ROLL], 30.0f),

    // @Param: MPIT
    // @DisplayName: PNU gimbal pitch mechanical limit
    // @Description: Mechanical travel limit of the pitch axis, used as plus and minus this value. See MRLL.
    // @Units: deg
    // @Range: 1 180
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MPIT", 14, AP_PNU_CAN, _mech_limit_deg[AP_PNUGIMBAL_AXIS_PITCH], 30.0f),

    // @Param: MYAW
    // @DisplayName: PNU gimbal yaw mechanical limit
    // @Description: Mechanical travel limit of the yaw axis, used as plus and minus this value. See MRLL.
    // @Units: deg
    // @Range: 1 180
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MYAW", 15, AP_PNU_CAN, _mech_limit_deg[AP_PNUGIMBAL_AXIS_YAW], 20.0f),

    // @Param: SLEW
    // @DisplayName: PNU gimbal slew rate limit
    // @Description: Maximum rate at which the gimbal angle command may change, in degrees per second. Bounds how fast anything can tell a bulky payload to move, and is what makes the first target after startup safe - it is approached from the measured angle rather than arriving as a step. 0 disables the limit. Provisional default, needs bench tuning.
    // @Units: deg/s
    // @Range: 0 360
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SLEW", 16, AP_PNU_CAN, _slew_rate_degs, 45.0f),

    // @Param: MAINT
    // @DisplayName: PNU gimbal maintenance mode
    // @Description: Set to 1 to take the flight controller completely off the CAN bus for the gimbal, so a PC can be connected to a motor's UART to run the vendor configuration utility. The motors are disabled and nothing is transmitted. Entering is immediate; LEAVING REQUIRES A REBOOT, because the utility can change the CAN ID, the control mode and the encoding ranges, and a host/device mismatch does not fail - it silently rescales every command and every reading.
    // @Values: 0:Normal,1:Maintenance
    // @User: Advanced
    AP_GROUPINFO("MAINT", 17, AP_PNU_CAN, _maintenance, 0),

    // @Param: HOME
    // @DisplayName: PNU gimbal homing at boot
    // @Description: Run the Appendix 1 endpoint search at every boot to find the mechanical centre. The drives use a relative encoder whose zero is assumed not to persist, so without this the angle limits are relative to wherever the mount powered up and the mechanical clamp protects nothing. DEFAULTS TO 0 because homing deliberately drives the payload into its hard stops and the PNU_H* tuning below is not yet bench-measured - turn it on once it is. Requires PNU_MODE 0 (MIT).
    // @Values: 0:Disabled - zero set by hand,1:Home at boot
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HOME", 18, AP_PNU_CAN, _homing_enabled, 0),

    // @Param: HVEL
    // @DisplayName: PNU gimbal homing approach speed
    // @Description: Speed at which an axis approaches its mechanical stop while homing. With PNU_HKD this sets how hard the payload pushes on the stop - the stall torque is HKD multiplied by this speed in radians per second. Provisional default, needs bench measurement.
    // @Units: deg/s
    // @Range: 1 60
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HVEL", 19, AP_PNU_CAN, _homing_vel_degs, 10.0f),

    // @Param: HKD
    // @DisplayName: PNU gimbal homing damping
    // @Description: Damping term used throughout homing. During an approach the stall torque is this value multiplied by PNU_HVEL in radians per second, which is the only thing bounding the push into a hard stop - choose it deliberately. Provisional default, needs bench measurement.
    // @Range: 0.01 5
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HKD", 20, AP_PNU_CAN, _homing_kd, 0.5f),

    // @Param: HKP
    // @DisplayName: PNU gimbal homing stiffness
    // @Description: Stiffness used to hold an axis in position during homing - both for the idle axes while another is sweeping, and for the move to the midpoint. Deliberately low. Provisional default, needs bench measurement.
    // @Range: 0.1 100
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HKP", 21, AP_PNU_CAN, _homing_kp, 5.0f),

    // @Param: HSTL
    // @DisplayName: PNU gimbal homing stall dwell
    // @Description: How long an axis must move less than PNU_HDLT, while motion is still commanded, before it counts as having reached its mechanical stop. Endpoint detection is by angle only - the overcurrent and overload status codes are emergency stops, not endpoint criteria.
    // @Units: ms
    // @Range: 100 5000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HSTL", 22, AP_PNU_CAN, _homing_stall_ms, 1000),

    // @Param: HDLT
    // @DisplayName: PNU gimbal homing stall threshold
    // @Description: Movement below this over the PNU_HSTL window counts as stopped. Too small and encoder noise prevents a stall ever being detected; too large and the axis is called stopped while still creeping. Provisional default, needs bench measurement.
    // @Units: deg
    // @Range: 0.01 5
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HDLT", 23, AP_PNU_CAN, _homing_stall_delta_degs, 0.1f),

    // @Param: HTOL
    // @DisplayName: PNU gimbal homing travel tolerance
    // @Description: How far the measured stop-to-stop travel may differ from twice the mechanical limit before homing fails. Much less than expected means something is obstructing the payload; much more means an endpoint was not an endpoint; about double or half means PNU_PMAX does not match the range configured in the motor, which is the only runtime check that can catch that. Provisional default - stop compliance under a heavy payload may consume most of it.
    // @Units: deg
    // @Range: 0.5 20
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HTOL", 24, AP_PNU_CAN, _homing_range_tol_degs, 3.0f),

    // @Param: HRTOL
    // @DisplayName: PNU gimbal homing reconfirm tolerance
    // @Description: How closely the two approaches to the same mechanical stop must agree. Tighter than PNU_HTOL and answering a different question - was that a real endpoint, rather than is the axis obstructed. Must be smaller than the backoff between approaches, which is 2 percent of the axis travel.
    // @Units: deg
    // @Range: 0.05 5
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HRTOL", 25, AP_PNU_CAN, _homing_reconfirm_tol_degs, 0.5f),

    // @Param: MARG
    // @DisplayName: PNU gimbal operating margin
    // @Description: How far inside the mechanical limit the operating limit sits, per side. Homing requires the measured stop-to-stop travel to be at least twice the resulting operating limit - with the defaults that is 50 degrees on a 60 degree axis - because a mount that cannot reach its own operating range has something obstructing it. Keep MNT1_ROLL/PITCH/YAW_MIN/MAX consistent with this; they are not derived from it.
    // @Units: deg
    // @Range: 0.5 30
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MARG", 26, AP_PNU_CAN, _operating_margin_degs, 5.0f),

    // @Param: DRLL
    // @DisplayName: PNU gimbal roll direction
    // @Description: Rotation direction of the roll motor relative to the mount convention. 0 if a positive commanded angle increases the reported encoder position, 1 if it decreases it. The gimbal is still undergoing mechanical design changes, so this is a parameter rather than a source constant.
    // @Values: 0:Normal,1:Reversed
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("DRLL", 27, AP_PNU_CAN, _axis_reversed[AP_PNUGIMBAL_AXIS_ROLL], 0),

    // @Param: DPIT
    // @DisplayName: PNU gimbal pitch direction
    // @Description: Rotation direction of the pitch motor relative to the mount convention. 0 if a positive commanded angle increases the reported encoder position, 1 if it decreases it. See DRLL.
    // @Values: 0:Normal,1:Reversed
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("DPIT", 28, AP_PNU_CAN, _axis_reversed[AP_PNUGIMBAL_AXIS_PITCH], 0),

    // @Param: DYAW
    // @DisplayName: PNU gimbal yaw direction
    // @Description: Rotation direction of the yaw motor relative to the mount convention. 0 if a positive commanded angle increases the reported encoder position, 1 if it decreases it. See DRLL.
    // @Values: 0:Normal,1:Reversed
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("DYAW", 29, AP_PNU_CAN, _axis_reversed[AP_PNUGIMBAL_AXIS_YAW], 0),

    // @Param: SCHK
    // @DisplayName: PNU SAR start interlock checks
    // @Description: Which SAR source-quality conditions the FC enforces: the ICD section 14.3.3 checks before it sends an observation Start, and the matching section 14.7 conditions that automatically terminate an observation in progress. The default is the set the ICD requires, less the gimbal attitude age, for which there is no source until the gimbal supplies real attitude - a Start with that bit set is always refused. Clearing a bit is a bench and integration aid, not a flight configuration: it is announced once per boot and again on every Start that goes out under it. Stop is never gated, because the ICD forbids blocking it, and neither is the vehicle navigation failsafe, which always terminates an observation.
    // @Bitmask: 0:Primary GPS RTK Fixed,1:GPS fix age under 300ms,2:Gimbal attitude age under 40ms,3:Navigation health
    // @User: Advanced
    AP_GROUPINFO("SCHK", 30, AP_PNU_CAN, _sar_start_checks, AP_PNUSAR_SCHK_ICD),

    AP_GROUPEND
};

AP_PNU_CAN::AP_PNU_CAN()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_PNU_CAN must be singleton");
    }
    _singleton = this;
}

// The backends are deliberately NOT deleted here.  The loop thread holds references to
// them for the life of the vehicle and is never stopped, so freeing them would be a
// use-after-free rather than a tidy-up.  This destructor is unreachable in practice.
AP_PNU_CAN::~AP_PNU_CAN()
{
    _singleton = nullptr;
}

// return the driver registered on @driver_index, or nullptr if it is not ready
AP_PNU_CAN *AP_PNU_CAN::get_pnu_can(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers()) {
        return nullptr;
    }
    const AP_CAN::Protocol type = AP::can().get_driver_type(driver_index);
    if ((type != AP_CAN::Protocol::PNU_CAN) &&
        (type != AP_CAN::Protocol::PNU_CAN_LEGACY)) {
        return nullptr;
    }
    return static_cast<AP_PNU_CAN*>(AP::can().get_driver(driver_index));
}

bool AP_PNU_CAN::add_interface(AP_HAL::CANIface *can_iface)
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

void AP_PNU_CAN::init(uint8_t driver_index)
{
    if (_initialized || (_can_iface == nullptr)) {
        return;
    }

    // assign only on the call that actually starts the driver, so a second init() cannot
    // repoint it while the loop thread is already running (AP_PMUCAN precedent)
    _driver_index = driver_index;

    // A failure here is not fatal and does not skip the thread: the thread is what
    // reports the failure, once report_startup() can expect a GCS to be listening.  With
    // no backends it does nothing else.
    create_backends();

    snprintf(_thread_name, sizeof(_thread_name), "pnu_can");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PNU_CAN::loop, void),
                                      _thread_name, 2048,
                                      AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }

    _initialized = true;
}

// Allocate every backend CAN_Dn_PNU_OPTS asks for.  One device failing to validate does
// not stop the others - a mistyped motor id should not take the SAR off the bus - but
// each failure is recorded and reported.
void AP_PNU_CAN::create_backends(void)
{
    const uint8_t opts = get_opts();

    if (opts & DEVICE_PMU) {
        AP_PMUCAN *pmu = NEW_NOTHROW AP_PMUCAN(*this);
        if (!add_backend(pmu)) {
            set_init_error("PMU backend failed to start");
        }
    }

#if AP_PNUGIMBAL_CUBEMARS_ENABLED
    if (opts & DEVICE_GIMBAL) {
        switch (get_gimbal_type()) {
        case 1:
            _gimbal = NEW_NOTHROW AP_PNUGimbal_CubeMars(*this);
            break;
        default:
            set_init_error("PNU_TYPE %u is not a known gimbal", (unsigned)get_gimbal_type());
            break;
        }
        if ((_gimbal != nullptr) && !add_backend(_gimbal)) {
            _gimbal = nullptr;
        }
    }
#endif

#if AP_PNUSAR_ENABLED
    if (opts & DEVICE_SAR) {
        AP_PNUSAR *sar = NEW_NOTHROW AP_PNUSAR(*this);
        if (!add_backend(sar)) {
            set_init_error("SAR backend failed to start");
        } else {
            // typed alias into _backends, so the vehicle's 10 Hz snapshot task does not
            // have to search for it every tick
            _sar = sar;
        }
    }
#endif
}

// Take ownership of a backend, or destroy it if it will not run.
bool AP_PNU_CAN::add_backend(AP_PNU_CAN_Backend *backend)
{
    if (backend == nullptr) {
        set_init_error("backend allocation failed");
        return false;
    }
    if (_num_backends >= AP_PNU_CAN_MAX_BACKENDS) {
        set_init_error("too many backends");
        delete backend;
        return false;
    }
    if (!backend->init()) {
        // init() has already recorded why through set_init_error()
        delete backend;
        return false;
    }

    _backends[_num_backends++] = backend;
    return true;
}

void AP_PNU_CAN::set_init_error(const char *fmt, ...)
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
void AP_PNU_CAN::loop(void)
{
    const uint32_t period_us = 1000000UL / AP_PNU_CAN_LOOP_HZ;

    while (true) {
        hal.scheduler->delay_microseconds(period_us);

        if (!_initialized || (_can_iface == nullptr)) {
            continue;
        }

        _loop_cnt++;

        report_startup();

        if (_num_backends == 0) {
            continue;
        }

        rx_spin();
        tx_spin();
    }
}

// One line per boot naming the backends that are running, sent late enough that a GCS is
// listening.  This is what makes a forgotten CAN_Dn_PNU_OPTS bit visible: with bit 0
// clear, PMU_Ctrl_Echo.PMUCAN_Fail stays 0 and reads as "healthy", so a missing PMU is
// otherwise completely silent.  A one-shot line costs nothing and does not become the
// permanent prearm warning that raising that global to 2 would.
void AP_PNU_CAN::report_startup(void)
{
    if (_startup_reported) {
        return;
    }

    // Wait for a GCS to be LISTENING, not for a fixed delay.
    //
    // This used to fire at a flat 5 s, and on the bench 2026-10-19 that lost the message
    // entirely: KGCS had not connected yet, the statustext was dropped, and because it is
    // one-shot it never came back.  Neither "PMU ready" nor the protocol-16 deprecation
    // warning was ever seen.  A one-shot message nobody receives is precisely the silence
    // PNU-ISSUE(D6) exists to remove, so the trigger is now the condition that actually
    // matters.  There is no upper bound on purpose - a GCS connecting ten minutes in
    // still gets the line, and if none ever connects there is nobody to tell anyway.
    if (GCS_MAVLINK::active_channel_mask() == 0) {
        return;
    }

    _startup_reported = true;

    const unsigned bus = (unsigned)(_driver_index + 1);

    if (_num_backends == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PNU_CAN: CAN_D%u no backends (PNU_OPTS=%u)",
                      bus, (unsigned)get_opts());
    } else {
        char names[32] = {};
        for (uint8_t i = 0; i < _num_backends; i++) {
            if (i != 0) {
                strncat(names, "+", sizeof(names) - strlen(names) - 1);
            }
            strncat(names, _backends[i]->name(), sizeof(names) - strlen(names) - 1);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PNU_CAN: CAN_D%u %s ready%s", bus, names,
                      (get_opts() & DEVICE_PMU) ? "" : ", no PMU");
    }

    if (AP::can().get_driver_type(_driver_index) == AP_CAN::Protocol::PNU_CAN_LEGACY) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "PNU_CAN: CAN_D%u_PROTOCOL 16 is deprecated, use 15", bus);
    }
    if (_init_error[0] != '\0') {
        // CRITICAL, not WARNING: an init error means a backend this configuration asked
        // for is NOT RUNNING.  It is reported once per boot and the operator has to act.
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PNU_CAN: CAN_D%u %s", bus, _init_error);
    }
}

// Drain the interface and offer every frame to the backends in turn.
//
// Each pass receives at the top of the loop, so a frame can never be dequeued and then
// dropped when the budget runs out.  AP_PMUCAN::RXdrain() receives at the bottom and
// needs a trailing handleFrame() to cover exactly that case; this shape does not.
void AP_PNU_CAN::rx_spin(void)
{
    bool read_select = true;
    bool write_select = false;
    const uint64_t deadline = AP_HAL::micros64() + AP_PNU_CAN_SELECT_TIMEOUT_US;

    // select() blocks for up to the deadline, so it is deliberately outside the
    // semaphore below - holding _sem across it would stall the vehicle thread
    if (!_can_iface->select(read_select, write_select, nullptr, deadline)) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    const uint64_t drain_start_us = AP_HAL::micros64();
    for (uint8_t frames = 0; frames < AP_PNU_CAN_RX_MAX_FRAMES; frames++) {
        if ((AP_HAL::micros64() - drain_start_us) >= AP_PNU_CAN_RX_MAX_TIME_US) {
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

        bool claimed = false;
        for (uint8_t i = 0; i < _num_backends; i++) {
            if (_backends[i]->handle_frame(frame)) {
                claimed = true;
                break;
            }
        }
        if (!claimed) {
            report_unclaimed(frame.id);
        }
    }
}

void AP_PNU_CAN::tx_spin(void)
{
    WITH_SEMAPHORE(_sem);

    for (uint8_t i = 0; i < _num_backends; i++) {
        _backends[i]->update(_loop_cnt);
    }
}

bool AP_PNU_CAN::send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if ((_can_iface == nullptr) || (data == nullptr) ||
        (dlc > AP_HAL::CANFrame::NonFDCANMaxDataLen)) {
        return false;
    }

    // @can_id carries its own flags: the PMU polls with extended RTR frames, the gimbal
    // sends standard data frames.  The shape of a frame is the backend's business, so
    // nothing is masked or added here beyond what CANFrame itself needs.
    const AP_HAL::CANFrame out_frame { can_id, data, dlc };
    const uint64_t deadline = AP_HAL::micros64() + AP_PNU_CAN_SEND_TIMEOUT_US;

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
void AP_PNU_CAN::report_short_frame(uint32_t frame_id, uint8_t dlc)
{
    _short_frame_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_short_frame_cnt > 1) &&
        ((now_ms - _last_short_frame_warn_ms) < AP_PNU_CAN_SHORT_FRAME_WARN_MS)) {
        return;
    }
    _last_short_frame_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNU_CAN: short frame 0x%x dlc %u (%u dropped)",
                  (unsigned)frame_id, (unsigned)dlc, (unsigned)_short_frame_cnt);
}

// Its own rate limiter, not the short-frame one: a bus that is down would otherwise
// produce a send failure every tick and crowd out every other warning this driver has.
void AP_PNU_CAN::report_tx_error(uint32_t can_id, int result)
{
    _tx_err_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_tx_err_cnt > 1) &&
        ((now_ms - _last_tx_err_warn_ms) < AP_PNU_CAN_TX_ERR_WARN_MS)) {
        return;
    }
    _last_tx_err_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNU_CAN: send 0x%x failed %d (%u dropped)",
                  (unsigned)can_id, result, (unsigned)_tx_err_cnt);
}

// A frame no backend recognised.  On a shared bus this is how a mis-assigned CAN id
// announces itself - see the id allocation in README.md, which keeps the gimbal below
// 0x300 and the SAR at 0x400 and above precisely so that this stays quiet.
void AP_PNU_CAN::report_unclaimed(uint32_t can_id)
{
    _unclaimed_cnt++;

    const uint32_t now_ms = AP_HAL::millis();
    if ((_unclaimed_cnt > 1) &&
        ((now_ms - _last_unclaimed_warn_ms) < AP_PNU_CAN_UNCLAIMED_WARN_MS)) {
        return;
    }
    _last_unclaimed_warn_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PNU_CAN: unclaimed frame 0x%x (%u seen)",
                  (unsigned)can_id, (unsigned)_unclaimed_cnt);
}

// Mechanical travel limit for one axis, in radians, as a symmetric plus/minus bound.
//
// NOTE, and it matters: the encoder is relative and its zero is assumed NOT to persist, so
// this limit means nothing until the Appendix 1 homing sequence has put zero at the centre
// of travel.  Until step 1d runs that sequence, the only thing protecting the payload is
// the bounded stiffness and torque used during homing itself.
// True while CAN_Dn_PNU_MAINT asks for maintenance.  The gimbal layer LATCHES this at
// init rather than following it live, so clearing the parameter does not silently put a
// reconfigured motor back on the bus - see the parameter description.
// +1.0 or -1.0 for one axis.  The gimbal is still undergoing mechanical design changes, so
// which way a motor turns for a positive command is configuration, not a source constant.
float AP_PNU_CAN::get_axis_direction(uint8_t axis) const
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return 1.0f;
    }
    return (_axis_reversed[axis].get() != 0) ? -1.0f : 1.0f;
}

bool AP_PNU_CAN::get_maintenance(void) const
{
    return _maintenance.get() != 0;
}

float AP_PNU_CAN::get_mech_limit_rad(uint8_t axis) const
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        return 0.0f;
    }
    return radians(_mech_limit_deg[axis].get());
}

uint8_t AP_PNU_CAN::get_motor_id(uint8_t axis) const
{
    if (axis >= AP_PNUGIMBAL_NUM_AXES) {
        // 0 is not a configurable id, so a backend's own validation rejects this
        return 0;
    }
    return uint8_t(_motor_id[axis].get());
}

namespace AP {

AP_PNU_CAN *pnu_can()
{
    return AP_PNU_CAN::get_singleton();
}

};

#endif  // AP_PNU_CAN_ENABLED
