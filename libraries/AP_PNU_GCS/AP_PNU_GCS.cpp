#include "AP_PNU_GCS.h"

#include <AP_HAL/AP_HAL.h>

AP_PNU_GCS *AP_PNU_GCS::_singleton;

const AP_Param::GroupInfo AP_PNU_GCS::var_info[] = {

    // @Param: GCS_COMPAT
    // @DisplayName: PNU ground station compatibility mode
    // @Description: Suppresses the FC to GCS telemetry messages that did not exist in FC V5.1.0, for a ground station that cannot tolerate unknown MAVLink message ids. 0 transmits everything this firmware defines and is the normal setting. 1 suppresses TM50 (51050 SAR status), TM60 (51060 ECU mode status) and TM61 (51061 ECU energy status). TM1, TM2, TM3 and TM5 are never affected - their field sets are unchanged since V5.1.0. This controls TRANSMISSION ONLY: CAN polling, ECU decoding, engine commands, the engine start interlock, the ENERGYCFG transaction and every dataflash record are unchanged at either setting. Values other than 0 and 1 are undefined and behave as 0.
    // @Values: 0:Normal - all messages transmitted,1:Legacy GCS - suppress TM50 TM60 TM61
    // @User: Standard
    AP_GROUPINFO("GCS_COMPAT", 1, AP_PNU_GCS, _gcs_compat, AP_PNU_GCS::COMPAT_NORMAL),

    AP_GROUPEND
};

AP_PNU_GCS::AP_PNU_GCS()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_PNU_GCS must be singleton");
    }
    _singleton = this;
}

namespace AP {

AP_PNU_GCS *pnu_gcs()
{
    return AP_PNU_GCS::get_singleton();
}

}
