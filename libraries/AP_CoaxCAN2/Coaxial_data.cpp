#include "Coaxial_data.h"


CoaxData::CoaxData() {
    fcrdy = 0;
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