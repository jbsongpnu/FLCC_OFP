#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    gcs().OA_Status.Object_Avoidance_Mode = 0;
	gcs().GCS_Ctrl_OA_Mode.OA_Mode = 0;
    copter.avoid.proximity_avoidance_enable(false);
    if (1 == CAM_UART->is_initialized()) // byungwchoi
    {
        CAM_UART->end();
    }

    CAM_UART->begin(115200);
    gcs().send_text(MAV_SEVERITY_INFO, "MAVLINK CAM UART Initialising OK.");

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
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
    static uint16_t counter = 0;
    // put your 10Hz code here
    //Object_Avoidance_Mode is mainly determined by proximity_avoidance_enabled flag
	//because proximity_avoidance_enabled_flag can be turned off from automatic algorithm and emergency controller
	if(copter.avoid.proximity_avoidance_enabled()){
		gcs().OA_Status.Object_Avoidance_Mode = 1;//forece to 1 or gcs().GCS_Ctrl_OA_Mode.OA_Mode; 
        if(counter>2) {
            gcs().send_message(MSG_DISTANCE_SENSOR);
            counter=0;
        }
        counter++;
	}else{
		gcs().OA_Status.Object_Avoidance_Mode = 0;
	}
    gcs().send_message(MSG_CAM_STATUS); // KAL : Send CAM Status with Mavlink 
    gcs().send_message(MSG_OBJECT_AVOIDANCE_STATUS); // Send object avoidance status to GCS with Mavlink Message (PNU & KAL)
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
