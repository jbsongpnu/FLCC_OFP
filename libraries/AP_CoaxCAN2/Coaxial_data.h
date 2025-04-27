#pragma once

#include <AP_Common/AP_Common.h>

class CoaxData
{
public:

    // CoaxData() {
    //     if (_singleton  == nullptr) {
    //         _singleton = this;
    //     } 
    // };

    static CoaxData& get_instance();

    // static class CoaxData *get_singleton() {
    //     return _singleton;
    // }

    uint8_t fcrdy;

private:
    CoaxData();
    
    CoaxData(const CoaxData&) = delete;
    CoaxData& operator=(const CoaxData&) = delete;

    static CoaxData* _instance;
    // static CoaxData *_singleton;
};

CoaxData &cxdata();

//CoaxData &coaxdata();

