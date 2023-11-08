/*
 * CAN bus driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef PMUCAN_DRIVER_CAN_HPP_INCLUDED
#define PMUCAN_DRIVER_CAN_HPP_INCLUDED

#include <cassert>
#include <AP_PMUCAN/pmucan_std.hpp> //#include <uavcan/std.hpp>
#include <AP_PMUCAN/pmucan_build_config.hpp> //#include <uavcan/build_config.hpp>
#include <AP_PMUCAN/pmucan_templates.hpp>	//#include <uavcan/driver/system_clock.hpp> --> time.hpp --> templates.hpp

//by igpark form pmucan_templates.hpp
#if 0
namespace pmucan
{
/**
 * Replacement for std::fill(..)
 */
template <typename ForwardIt, typename T>
PMUCAN_EXPORT
void fill(ForwardIt first, ForwardIt last, const T& value)
{
    while (first != last)
    {
        *first = value;
        ++first;
    }
}

/**
 * Replacement for std::fill_n(..)
 */
template<typename OutputIt, typename T>
PMUCAN_EXPORT
void fill_n(OutputIt first, std::size_t n, const T& value)
{
    while (n--)
    {
        *first++ = value;
    }
}

/**
 * Replacement for std::copy(..)
 */
template <typename InputIt, typename OutputIt>
PMUCAN_EXPORT
OutputIt copy(InputIt first, InputIt last, OutputIt result)
{
    while (first != last)
    {
        *result = *first;
        ++first;
        ++result;
    }
    return result;
}

/**
 * Replacement for std::equal(..)
 */
template<typename InputIt1, typename InputIt2>
PMUCAN_EXPORT
bool equal(InputIt1 first1, InputIt1 last1, InputIt2 first2)
{
    while (first1 != last1)
    {
        if (*first1 != *first2)
        {
            return false;
        }
        ++first1;
        ++first2;
    }
    return true;
}

/**
 * Replacement for std::max(..)
 */
template <typename T>
PMUCAN_EXPORT
const T& max(const T& a, const T& b)
{
    return (a < b) ? b : a;
}

}
#endif

namespace pmucan
{
/**
 * This limit is defined by the specification.
 */
enum { MaxCanIfaces = 3 };

/**
 * Raw CAN frame, as passed to/from the CAN driver.
 */
struct PMUCAN_EXPORT CanFrame
{
    static const uint32_t MaskStdID = 0x000007FFU;
    static const uint32_t MaskExtID = 0x1FFFFFFFU;
    static const uint32_t FlagEFF = 1U << 31;                  ///< Extended frame format
    static const uint32_t FlagRTR = 1U << 30;                  ///< Remote transmission request
    static const uint32_t FlagERR = 1U << 29;                  ///< Error frame

    static const uint8_t MaxDataLen = 8;

    uint32_t id;                ///< CAN ID with flags (above)
    uint8_t data[MaxDataLen];
    uint8_t dlc;                ///< Data Length Code

    CanFrame() :
        id(0),
        dlc(0)
    {
        fill(data, data + MaxDataLen, uint8_t(0));
    }

    CanFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len) :
        id(can_id),
        dlc((data_len > MaxDataLen) ? MaxDataLen : data_len)
    {
    	PMUCAN_ASSERT(can_data != PMUCAN_NULLPTR);
        PMUCAN_ASSERT(data_len == dlc);
        (void)copy(can_data, can_data + dlc, this->data);
    }

    bool operator!=(const CanFrame& rhs) const { return !operator==(rhs); }
    bool operator==(const CanFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && equal(data, data + dlc, rhs.data);
    }

    bool isExtended()                  const { return id & FlagEFF; }
    bool isRemoteTransmissionRequest() const { return id & FlagRTR; }
    bool isErrorFrame()                const { return id & FlagERR; }

#if 0//#if UAVCAN_TOSTRING	//commented by igpark
    enum StringRepresentation
    {
        StrTight,   ///< Minimum string length (default)
        StrAligned  ///< Fixed formatting for any frame
    };

    std::string toString(StringRepresentation mode = StrTight) const;

#endif

    /**
     * CAN frame arbitration rules, particularly STD vs EXT:
     *     Marco Di Natale - "Understanding and using the Controller Area Network"
     *     http://www6.in.tum.de/pub/Main/TeachingWs2013MSE/CANbus.pdf
     */
    bool priorityHigherThan(const CanFrame& rhs) const;
    bool priorityLowerThan(const CanFrame& rhs) const { return rhs.priorityHigherThan(*this); }
};


/**
 * CAN hardware filter config struct.
 * Flags from @ref CanFrame can be applied to define frame type (EFF, EXT, etc.).
 * @ref ICanIface::configureFilters().
 */
struct PMUCAN_EXPORT CanFilterConfig
{
    uint32_t id;
    uint32_t mask;

    bool operator==(const CanFilterConfig& rhs) const
    {
        return rhs.id == id && rhs.mask == mask;
    }

    CanFilterConfig() :
        id(0),
        mask(0)
    { }
};

/**
 * Events to look for during @ref ICanDriver::select() call.
 * Bit position defines iface index, e.g. read = 1 << 2 to read from the third iface.
 */
struct PMUCAN_EXPORT CanSelectMasks
{
    uint8_t read;
    uint8_t write;

    CanSelectMasks() :
        read(0),
        write(0)
    { }
};

/**
 * Special IO flags.
 *
 * @ref CanIOFlagLoopback       - Send the frame back to RX with true TX timestamps.
 *
 * @ref CanIOFlagAbortOnError   - Abort transmission on first bus error instead of retransmitting. This does not
 *                                affect the case of arbitration loss, in which case the retransmission will work
 *                                as usual. This flag is used together with anonymous messages which allows to
 *                                implement CSMA bus access. Read the spec for details.
 */


typedef uint16_t CanIOFlags;
static const CanIOFlags CanIOFlagLoopback = 1;
static const CanIOFlags CanIOFlagAbortOnError = 2;//iajo 19/06/14



/**
 * Single non-blocking CAN interface.
 */
class PMUCAN_EXPORT ICanIface
{
public:
    virtual ~ICanIface() { }

    /**
     * Non-blocking transmission.
     *
     * If the frame wasn't transmitted upon TX deadline, the driver should discard it.
     *
     * Note that it is LIKELY that the library will want to send the frames that were passed into the select()
     * method as the next ones to transmit, but it is NOT guaranteed. The library can replace those with new
     * frames between the calls.
     *
     * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
     */
    virtual int16_t send(const CanFrame& frame, uint64_t tx_deadline_usec, CanIOFlags flags) = 0;	//virtual int16_t send(const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags) = 0;

    /**
     * Non-blocking reception.
     *
     * Timestamps should be provided by the CAN driver, ideally by the hardware CAN controller.
     *
     * Monotonic timestamp is required and can be not precise since it is needed only for
     * protocol timing validation (transfer timeouts and inter-transfer intervals).
     *
     * UTC timestamp is optional, if available it will be used for precise time synchronization;
     * must be set to zero if not available.
     *
     * Refer to @ref ISystemClock to learn more about timestamps.
     *
     * @param [out] out_ts_monotonic Monotonic timestamp, mandatory.
     * @param [out] out_ts_utc       UTC timestamp, optional, zero if unknown.
     * @return 1 = one frame received, 0 = RX buffer empty, negative for error.
     */

    //commented by igpark
//    virtual int16_t receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic, UtcTime& out_ts_utc,
//                            CanIOFlags& out_flags) = 0;
    virtual int16_t receive(pmucan::CanFrame& out_frame, pmucan::CanIOFlags& out_flags);
    /**
     * Configure the hardware CAN filters. @ref CanFilterConfig.
     *
     * @return 0 = success, negative for error.
     */
    virtual int16_t configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs) = 0;

    /**
     * Number of available hardware filters.
     */
    virtual uint16_t getNumFilters() const = 0;

    /**
     * Continuously incrementing counter of hardware errors.
     * Arbitration lost should not be treated as a hardware error.
     */
    virtual uint64_t getErrorCount() const = 0;
};

/**
 * Generic CAN driver.
 */
class PMUCAN_EXPORT ICanDriver
{
public:
    virtual ~ICanDriver() { }

    /**
     * Returns an interface by index, or null pointer if the index is out of range.
     */
    virtual ICanIface* getIface(uint8_t iface_index) = 0;

    /**
     * Default implementation of this method calls the non-const overload of getIface().
     * Can be overriden by the application if necessary.
     */
    virtual const ICanIface* getIface(uint8_t iface_index) const
    {
        return const_cast<ICanDriver*>(this)->getIface(iface_index);
    }

    /**
     * Total number of available CAN interfaces.
     * This value shall not change after initialization.
     */
    virtual uint8_t getNumIfaces() const = 0;

    /**
     * Block until the deadline, or one of the specified interfaces becomes available for read or write.
     *
     * Iface masks will be modified by the driver to indicate which exactly interfaces are available for IO.
     *
     * Bit position in the masks defines interface index.
     *
     * Note that it is allowed to return from this method even if no requested events actually happened, or if
     * there are events that were not requested by the library.
     *
     * The pending TX argument contains an array of pointers to CAN frames that the library wants to transmit
     * next, per interface. This is intended to allow the driver to properly prioritize transmissions; many
     * drivers will not need to use it. If a write flag for the given interface is set to one in the select mask
     * structure, then the corresponding pointer is guaranteed to be valid (not UAVCAN_NULLPTR).
     *
     * @param [in,out] inout_masks        Masks indicating which interfaces are needed/available for IO.
     * @param [in]     pending_tx         Array of frames, per interface, that are likely to be transmitted next.
     * @param [in]     blocking_deadline  Zero means non-blocking operation.
     * @return Positive number of ready interfaces or negative error code.
     */

    //commented by igpark
    virtual int16_t TXselect(CanSelectMasks& inout_masks, const CanFrame* (& pending_tx)[MaxCanIfaces]) = 0;
    virtual int16_t RXselect(pmucan::CanSelectMasks& inout_masks) = 0;	//igpark
};

}

#endif // PMUCAN_DRIVER_CAN_HPP_INCLUDED
