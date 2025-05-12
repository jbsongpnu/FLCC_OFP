#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

//----Serial port map------
// SERIAL0 -> USB
// SERIAL1 -> UART7 (Telem1) RTS/CTS pins
// SERIAL2 -> UART5 (Telem2) RTS/CTS pins
// SERIAL3 -> USART1 (GPS1)
// SERIAL4 -> UART8 (GPS2)
// SERIAL5 -> USART2 (Telem3) RTS/CTS pins
// SERIAL6 -> UART4 (User)
// SERIAL7 -> USART3 (Debug)
// SERIAL8 -> USB (MAVLink, can be used for SLCAN with protocol change)


class AP_CoaxServo {
public:
    AP_CoaxServo();

    static AP_CoaxServo *get_singleton();
    static AP_CoaxServo *_singleton;

private:
    


};

