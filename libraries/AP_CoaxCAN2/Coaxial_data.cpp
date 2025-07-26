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
    INV_data.Theta_Offset = 55; //phase angle degree : 0.1 precision
    INV_data.Speed_Limit = 5000; //speed limit rpm : 1.0 precision
    INV_data.OVL = 550;
    INV_data.UVL = 350;
    INV_data.OCL = 200;
    INV_data.OTL = 80;
    INV_data.OSL = 6000;   //Over-speed limit rpm : 1 precision
    INV_data.motor_Spd = 0;//4123;
    INV_data.i_a = 10.11;
    INV_data.i_b = 10.22;
    INV_data.i_c = 10.33;
    INV_data.t_a = 20.11;
    INV_data.t_b = 20.22;
    INV_data.t_c = 20.33;
    INV_data.FLT.ALL = 0;
    INV_data.V_dc_input = 450.5;
    INV_data.MI = 0;    
    INV_data.Motor_Align_flag = 0;

    INV_data.isNew = 0;
    INV_data.Motor_RPM_CMD = 0;//4124;
    INV_data.Motor_ACC_CMD = 0;

    CCB_data.isNew = 0;
    CCB_data.Brd_temp = 1;
    CCB_data.Flow_mL = 2;
    CCB_data.State.ALL = 0;
    CCB_data.State.bits.IsActive = 1;
    CCB_data.ThCp1x10 = 5;
    CCB_data.ThCp2x10 = 6;
    CCB_data.Thermistor1x10 = 1;
    CCB_data.Thermistor2x10 = 2;
    CCB_data.Thermistor3x10 = 3;
    CCB_data.Thermistor4x10 = 4;

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