#include "AP_PNU_CAN_Backend.h"

#if AP_PNU_CAN_ENABLED

#include "AP_PNU_CAN.h"

// The two protected helpers exist only so that a backend never holds the interface
// itself.  They are out of line because the driver's definition is not available in
// AP_PNU_CAN_Backend.h - the driver includes this header, not the other way round.

bool AP_PNU_CAN_Backend::send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    return _frontend.send_frame(can_id, data, dlc);
}

void AP_PNU_CAN_Backend::report_short_frame(uint32_t frame_id, uint8_t dlc)
{
    _frontend.report_short_frame(frame_id, dlc);
}

#endif  // AP_PNU_CAN_ENABLED
