#include "LCIND.h"

LCIND_class::LCIND_class()
{
    if (_singleton) {
        return;
    }
    _singleton = this;
}

LCIND_class *LCIND_class::_singleton = nullptr;
LCIND_class *LCIND_class::get_singleton()
{
    return _singleton;
}

void LCIND_class::testf(uint16_t a){
    test = a;
}

uint16_t LCIND_class::get(){
    return test;
}

namespace AP 
{
LCIND_class *LCIND_g()
{
    return LCIND_class::get_singleton();

}
};