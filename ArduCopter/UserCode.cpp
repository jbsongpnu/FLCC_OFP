#include "Copter.h"

#include "LCIND.h"
// Following definitions are defiend in LCIND.h
// #define UART_LC_1   hal.serial(1)
// #define UART_LC_2   hal.serial(2)
// #define UART_LC_3   hal.serial(5)
// #define UART_LC_4   hal.serial(4)

extern mavlink_loadcell_indicators_t MAV_GCSTX_LCID;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
}
#endif

#ifdef USERHOOK_FASTLOOP
#define BUF_SIZE_PARSE 64
void Copter::userhook_FastLoop()
{
    static uint32_t s100Hz_Flag = 0;

    LCIND_class *LCIND = AP::LCIND_g();
    uint8_t parsebuff[BUF_SIZE_PARSE] = {0, };
    uint16_t recv_size = 0;

    while ((UART_LC_1->available() > 0) && (recv_size < BUF_SIZE_PARSE))
    {
        parsebuff[recv_size] = (uint8_t)UART_LC_1->read();
        recv_size++;
    }
    if(recv_size > 13) {
        // gcs().send_text(MAV_SEVERITY_INFO, "procssing stream : size %u, id %u", recv_size, 1);
        LCIND->process_stream(parsebuff, recv_size, DEVICE_ID_1);
    }

    recv_size = 0;
    while ((UART_LC_2->available() > 0) && (recv_size < BUF_SIZE_PARSE))
    {
        parsebuff[recv_size] = (uint8_t)UART_LC_2->read();
        recv_size++;
    }
    if(recv_size > 13) {
        // gcs().send_text(MAV_SEVERITY_INFO, "procssing stream : size %u, id %u", recv_size, 2);
        LCIND->process_stream(parsebuff, recv_size, DEVICE_ID_2);
    }

    recv_size = 0;
    while ((UART_LC_3->available() > 0) && (recv_size < BUF_SIZE_PARSE))
    {
        parsebuff[recv_size] = (uint8_t)UART_LC_3->read();
        recv_size++;
    }
    if(recv_size > 13) {
        // gcs().send_text(MAV_SEVERITY_INFO, "procssing stream : size %u, id %u", recv_size, 3);
        LCIND->process_stream(parsebuff, recv_size, DEVICE_ID_3);
    }
    
    recv_size = 0;
    while ((UART_LC_4->available() > 0) && (recv_size < BUF_SIZE_PARSE))
    {
        parsebuff[recv_size] = (uint8_t)UART_LC_4->read();
        recv_size++;
    }
    if(recv_size > 13) {
        // gcs().send_text(MAV_SEVERITY_INFO, "procssing stream : size %u, id %u", recv_size, 4);
        LCIND->process_stream(parsebuff, recv_size, DEVICE_ID_4);
    }

    LCIND->total_weight = -1.0f * (LCIND->weight[0] + LCIND->weight[1] + LCIND->weight[2] + LCIND->weight[3]);

    MAV_GCSTX_LCID.Total_weight = LCIND->total_weight;
    MAV_GCSTX_LCID.LC1 = LCIND->weight[0] * -1.0f;
    MAV_GCSTX_LCID.LC2 = LCIND->weight[1] * -1.0f;
    MAV_GCSTX_LCID.LC3 = LCIND->weight[2] * -1.0f;
    MAV_GCSTX_LCID.LC4 = LCIND->weight[3] * -1.0f;
    //LC1 : FL, LC2 : FR, LC3 : RL, LC4 : RR
    LCIND->length_x = 0.68;
    LCIND->length_y = 1.31;
    MAV_GCSTX_LCID.Qx = ((LCIND->weight[1]+LCIND->weight[3]) - (LCIND->weight[0]+LCIND->weight[2])) * LCIND->length_x * 0.5f;//* 0.68 * 0.5 ;
    MAV_GCSTX_LCID.Qy = ((LCIND->weight[2]+LCIND->weight[3]) - (LCIND->weight[0]+LCIND->weight[1])) * LCIND->length_y * 0.5f;//* 1.31 * 0.5 ;

    // --- DEBUG CODE BLOCK
    // if(s100Hz_Flag % 500 == 0) {
    //     // gcs().send_text(MAV_SEVERITY_INFO, "FASTLOOP %lu", s100Hz_Flag);
    //     gcs().send_text(MAV_SEVERITY_INFO, "Weights : %f, %f, %f, %f", LCIND->weight[0], LCIND->weight[1], LCIND->weight[2], LCIND->weight[3]);
    // }
    // --- DEBUG CODE BLOCK

    if((s100Hz_Flag % 10) == 0) {
        //Send to ground program at 10Hz
        if((LCIND->valid[0]) && (LCIND->valid[1]) && (LCIND->valid[2]) && (LCIND->valid[3])) {
            gcs().send_message(MSG_LCID);
        }
        //Save log file to SD card
        AP::logger().Write("LCEL", "TimeUS,TOTW,TH1,TH2,TH3,TH4,MOX,MOY,LX,LY", "Qfffffffff",
            AP_HAL::micros64(),                             //Q     TimeUS
            MAV_GCSTX_LCID.Total_weight, 
            MAV_GCSTX_LCID.LC1,
            MAV_GCSTX_LCID.LC2,
            MAV_GCSTX_LCID.LC3,
            MAV_GCSTX_LCID.LC4,
            MAV_GCSTX_LCID.Qx,
            MAV_GCSTX_LCID.Qy,
            LCIND->length_x,
            LCIND->length_y
        );
        /*
            Format characters in the format string for binary log messages
            a   : int16_t[32]
            b   : int8_t
            B   : uint8_t
            h   : int16_t
            H   : uint16_t
            i   : int32_t
            I   : uint32_t
            f   : float
            d   : double
            n   : char[4]
            N   : char[16]
            Z   : char[64]
            c   : int16_t * 100
            C   : uint16_t * 100
            e   : int32_t * 100
            E   : uint32_t * 100
            L   : int32_t latitude/longitude
            M   : uint8_t flight mode
            q   : int64_t
            Q   : uint64_t
        */
    }

    s100Hz_Flag++;
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
