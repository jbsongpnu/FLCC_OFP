#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

// PNU : vehicle-level settings for the PNU-KAL GCS link.
//
// WHY THIS EXISTS, AND WHY IT IS NOT IN AP_PNU_CAN.  PNU_GCS_COMPAT is a property of the
// MAVLink link, not of the CAN bus, and it has to gate TM50 - which is transmitted by an
// unconditional scheduler task (Copter.cpp send_sar_status_tm) whose sender sits outside
// every AP_PNU_CAN_ENABLED guard, exactly as AP_PNUSAR.h's comment on SAR_Status says.  A
// parameter inside the CAN driver could not reach that path in a build with the driver
// compiled out, and a core path must not depend on an optional component.
//
// It is also not in GCS::var_info, whose group prefix is MAV: the parameter would have
// come out as MAV_PNU_GCS_COMPAT, which is 18 characters and over AP_Param's limit.
//
// So this is the PNU_ top-level group.  It holds one parameter today and is the home for
// any further PNU link-layer setting.  NOTHING HERE TOUCHES A DEVICE: the class holds
// parameters and answers questions about them, and owns no state, no thread and no I/O.

class AP_PNU_GCS {
public:
    AP_PNU_GCS();

    CLASS_NO_COPY(AP_PNU_GCS);

    static AP_PNU_GCS *get_singleton() { return _singleton; }

    // PNU_GCS_COMPAT.  Deliberately an enum and not a bitmask: the two values are the two
    // ground stations this aircraft is flown from, and per-message control has never been
    // wanted.  Values 2 and above are reserved for a further GCS generation.
    enum GCSCompat : uint8_t {
        COMPAT_NORMAL      = 0,     // transmit everything this firmware defines
        COMPAT_LEGACY_V510 = 1,     // suppress the three messages added since FC V5.1.0
    };

    // True when TM50 51050, TM60 51060 and TM61 51061 must not be transmitted.
    //
    // TESTED AGAINST THE DEFINED VALUE, NOT AGAINST ZERO.  An undefined setting must not
    // silently suppress telemetry: 2 and above fall through to normal transmission, which
    // is this parameter's default behaviour and the one that loses no data.
    bool suppress_post_v510_tm() const {
        return _gcs_compat == (int8_t)COMPAT_LEGACY_V510;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_PNU_GCS *_singleton;

    AP_Int8 _gcs_compat;
};

namespace AP {
    AP_PNU_GCS *pnu_gcs();
};
