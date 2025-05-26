#include "Copter.h"

#include "LCIND.h"

extern const AP_HAL::HAL& hal;

#define UART_LC_1   hal.serial(2)
#define UART_LC_2   hal.serial(3)
#define UART_LC_3   hal.serial(4)
#define UART_LC_4   hal.serial(5)

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    LCIND_class *LCIND = AP::LCIND_g();
    LCIND->testf(10);
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    uint8_t buf[2];
    buf[0] = 0;
    buf[1] = 2;
    LCIND_class *LCIND = AP::LCIND_g();
    buf[0] = LCIND->get();
    gcs().send_text(MAV_SEVERITY_INFO, "testing %u", LCIND->get());
    UART_LC_1->write(buf, 2);
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
