#include "Coaxial_data.h"


CoaxData::CoaxData() {
    fcrdy = 0;
    INV_data.CMD_Flag.ALL = 0;
    INV_data.Reference1 = 0;
    INV_data.Reference2 = 0;
    INV_data.Gain_Kpc = 1.8;
    INV_data.Gain_Kic = 150.0;
    INV_data.Current_Limit = 180;
    INV_data.Gain_Kps = 1.75; //Gain_Kps for speed control : 0.01 precision
    INV_data.Gain_Kis = 2.2; //Gain_Kis for speed control : 0.1 precision
    INV_data.Theta_Offset = 0; //phase angle degree : 0.1 precision
    INV_data.Speed_Limit = 5000; //speed limit rpm : 1.0 precision
    INV_data.OVL = 550;
    INV_data.UVL = 350;
    INV_data.OCL = 200;
    INV_data.OTL = 80;
    INV_data.OSL = 6000;   //Over-speed limit rpm : 1 precision
    INV_data.motor_Spd = 0;
    INV_data.i_a = 0;
    INV_data.i_b = 0;
    INV_data.i_c = 0;
    INV_data.t_a = 0;
    INV_data.t_b = 0;
    INV_data.t_c = 0;
    INV_data.FLT.ALL = 0;
    INV_data.V_dc_input = 0;
    INV_data.MI = 0;    
    INV_data.Motor_Align_flag = 0;

    INV_data.isNew = 0;
    INV_data.Motor_RPM_CMD = 0;
    INV_data.Motor_ACC_CMD = 0;

}

CoaxData* CoaxData::_instance = nullptr;

CoaxData& CoaxData::get_instance() {
    if(_instance == nullptr) {
        _instance = new CoaxData();
    }
    return *_instance;
    
}

CoaxData &cxdata() {
    return CoaxData::get_instance();
}