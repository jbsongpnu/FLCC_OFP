#include "Copter.h"

#include "LCIND.h"

extern const AP_HAL::HAL& hal;
extern mavlink_loadcell_indicators_t MAV_GCSTX_LCID;
#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    LCIND_class *LCIND = AP::LCIND_g();
    LCIND->valid[0] = LCIND->valid[1] = LCIND->valid[2] = LCIND->valid[3] = 0;
}
#endif

#ifdef USERHOOK_FASTLOOP
#define BUF_SIZE_PARSE 64
void Copter::userhook_FastLoop()
{
    static RingBuffer Rb1;
    static RingBuffer Rb2;
    static RingBuffer Rb3;
    static RingBuffer Rb4;
    static uint32_t runcounter = 0;

    LCIND_class *LCIND = AP::LCIND_g();
    uint8_t parsebuff[BUF_SIZE_PARSE] = {0};
    uint16_t recv_size = 0;

    while ((UART_LC_1->available() > 0) && (recv_size < BUF_SIZE_PARSE))
    {
        parsebuff[recv_size] = (uint8_t)UART_LC_1->read();
        recv_size++;
    }
    if(recv_size > 0) {
        //gcs().send_text(MAV_SEVERITY_INFO, "procssing stream : size %u, id %u", recv_size, 1);
        LCIND->process_stream(Rb1, parsebuff, recv_size, DEVICE_ID_1);
    }

    recv_size = 0;
    while ((UART_LC_2->available() > 0) && (recv_size < BUF_SIZE_PARSE))
    {
        parsebuff[recv_size] = (uint8_t)UART_LC_2->read();
        recv_size++;
    }
    if(recv_size > 0) {
        //gcs().send_text(MAV_SEVERITY_INFO, "procssing stream : size %u, id %u", recv_size, 2);
        LCIND->process_stream(Rb2, parsebuff, recv_size, DEVICE_ID_2);
    }

    recv_size = 0;
    while ((UART_LC_3->available() > 0) && (recv_size < BUF_SIZE_PARSE))
    {
        parsebuff[recv_size] = (uint8_t)UART_LC_3->read();
        recv_size++;
    }
    if(recv_size > 0) {
        //gcs().send_text(MAV_SEVERITY_INFO, "procssing stream : size %u, id %u", recv_size, 3);
        LCIND->process_stream(Rb3, parsebuff, recv_size, DEVICE_ID_3);
    }
    
    recv_size = 0;
    while ((UART_LC_4->available() > 0) && (recv_size < BUF_SIZE_PARSE))
    {
        parsebuff[recv_size] = (uint8_t)UART_LC_4->read();
        recv_size++;
    }
    if(recv_size > 0) {
        //gcs().send_text(MAV_SEVERITY_INFO, "procssing stream : size %u, id %u", recv_size, 4);
        LCIND->process_stream(Rb3, parsebuff, recv_size, DEVICE_ID_4);
    }

    if((LCIND->valid[0]) || (LCIND->valid[1]) || (LCIND->valid[2]) || (LCIND->valid[3])) {
        LCIND->total_weight = LCIND->weight[0] + LCIND->weight[1] + LCIND->weight[2] + LCIND->weight[3];
        
        MAV_GCSTX_LCID.Total_weight = LCIND->total_weight;
        MAV_GCSTX_LCID.LC1 = LCIND->weight[0];
        MAV_GCSTX_LCID.LC2 = LCIND->weight[1];
        MAV_GCSTX_LCID.LC3 = LCIND->weight[2];
        MAV_GCSTX_LCID.LC4 = LCIND->weight[3];
        //LC1 : FL, LC2 : FR, LC3 : RL, LC4 : RR
        MAV_GCSTX_LCID.Qx = ((LCIND->weight[0]+LCIND->weight[2]) - (LCIND->weight[1]+LCIND->weight[3])) * 0.5 ;
        MAV_GCSTX_LCID.Qy = ((LCIND->weight[0]+LCIND->weight[1]) - (LCIND->weight[2]+LCIND->weight[3])) * 0.5 ;

        if((runcounter % 100) == 0) {
            //debug message at 1Hz
            gcs().send_text(MAV_SEVERITY_INFO, "ID bit %u%u%u%u, Total %.2f, Qx %.2f, Qy %.2f", LCIND->valid[0], LCIND->valid[1], LCIND->valid[2], LCIND->valid[3], LCIND->total_weight, MAV_GCSTX_LCID.Qx, MAV_GCSTX_LCID.Qy);
        }
        if((runcounter % 10) == 0) {
            //Send to ground program at 10Hz
            gcs().send_message(MSG_LCID);
        }

        LCIND->valid[0] = LCIND->valid[1] = LCIND->valid[2] = LCIND->valid[3] = 0;
        
    }

    LCIND->TX_request();
    runcounter++;
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
