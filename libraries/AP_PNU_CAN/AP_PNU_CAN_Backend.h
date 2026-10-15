#pragma once

#include "AP_PNU_CAN_config.h"

#if AP_PNU_CAN_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/CANIface.h>

class AP_PNU_CAN;

// PNU : protocol backend for AP_PNU_CAN.
//
// The driver owns the interface, the thread, the parameters and the one place a frame
// reaches the bus.  A backend owns a wire format and nothing else.  The split is what
// lets the PMU, the gimbal and the SAR share one bus: they are three protocols, not
// three drivers, because AP_CANManager cannot give one interface to three drivers.
//
// What a backend must NOT do:
//   - touch the CANIface.  All CAN access goes through send_frame() below, so there is
//     one place that validates an outgoing frame and one place that counts a failed send.
//   - keep its own timer.  update() is handed the driver's tick count, so every backend
//     derives its sub-rates from one clock.
//   - claim a frame it does not recognise.  handle_frame() returning true stops the
//     dispatch, so a greedy backend silences the ones after it.

class AP_PNU_CAN_Backend {
public:
    AP_PNU_CAN_Backend(AP_PNU_CAN &frontend) : _frontend(frontend) {}
    virtual ~AP_PNU_CAN_Backend() {}

    CLASS_NO_COPY(AP_PNU_CAN_Backend);

    // Validate the configuration and prepare to run.  Returning false means this backend
    // must not transmit; it has already recorded why through AP_PNU_CAN::set_init_error(),
    // which is reported once the GCS is listening.
    virtual bool init(void) = 0;

    // Called from the driver thread at AP_PNU_CAN_LOOP_HZ, after the RX drain.  @tick is
    // the driver's loop counter: a backend that wants 10 Hz does its own modulo on it
    // rather than inventing a timer, so every rate in this driver has one source.
    virtual void update(uint32_t tick) = 0;

    // Called for each received frame until one backend claims it.  The driver has already
    // discarded loopback frames and nothing else: IDE / RTR / DLC validation belongs here,
    // because only the backend knows what its protocol accepts.
    //
    // Return true ONLY for a frame this backend recognises as its own.  An unclaimed
    // frame is counted and reported by the driver, which is how a mis-assigned CAN id
    // gets noticed instead of silently vanishing.
    virtual bool handle_frame(const AP_HAL::CANFrame &frame) = 0;

    // True when the device this backend talks to is present and reporting.
    virtual bool healthy(void) const = 0;

    // Short name for GCS messages, e.g. "Gimbal".  Used in the one-per-boot line that
    // says which backends are running, so keep it to a few characters.
    virtual const char *name(void) const = 0;

protected:
    // Write one frame.  @can_id is the FULL 32-bit id including AP_HAL::CANFrame::FlagEFF
    // and FlagRTR where the protocol needs them - the PMU sends extended RTR polls, the
    // gimbal sends standard data frames, and the driver does not care which.  Each
    // backend validates its own frame shape before calling.
    //
    // Returns true if the interface accepted the frame.  A false return is already
    // counted and reported by the driver, so a backend only decides whether to retry.
    bool send_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

    // Report a frame rejected for a short DLC, rate limited in the driver.
    void report_short_frame(uint32_t frame_id, uint8_t dlc);

    AP_PNU_CAN &_frontend;
};

#endif  // AP_PNU_CAN_ENABLED
