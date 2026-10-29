# PMU CAN Communication Protocol

This document defines how a Host (FLCC or another upper-level controller) queries the PMU (Power Management Unit) and sends engine and power-control commands. Distinguish documented protocol values from implementation items requiring confirmation. Do not implement `TBD` or `confirmation required` items as arbitrary fixed values.

## CAN Communication Settings

| Item | Setting |
|---|---|
| CAN specification | Classic CAN, CAN Specification 2.0 B-Active |
| Bit rate | **500 kbps = 500,000 bit/s** |
| Identifier | **29-bit Extended ID**, IDE = 1 |
| Status request | Host -> PMU, Remote Frame, RTR = 1, no data payload |
| Status response | PMU -> Host, Data Frame, RTR = 0, DLC = 8 |
| Control command | Host -> PMU, Data Frame, RTR = 0, DLC = 4 |
| Multibyte integers | **Little-endian, Least Significant Byte first** |
| Bit numbering | **bit 0 = LSB (Least Significant Bit)** |
| CAN FD | Not used |

Request status by sending an **RTR frame**, not a Data Frame with the same ID. The source marks request DLC as `N/A`. This indicates the absence of a payload; it does not establish a numeric request DLC of zero. If the CAN API requires an RTR DLC, verify PMU compatibility with using the expected response length of 8. Even with DLC = 8, a Remote Frame carries no data bytes.

Preserve Extended/RTR frame attributes in addition to the numeric ID. Classify received frames using ID, IDE, RTR and DLC together.

## Data Representation and Conversion

The source uses `INT8/16/32` for signed integers and `UINT8/16/24/32` for unsigned integers of the indicated width. `UINT24` is a three-byte integer, not a standard C/C++ built-in integer type. Interpret signed integers as two's complement.

| Source type | Implementation storage type | Notes |
|---|---|---|
| INT8 | int8_t | 1 byte |
| UINT8 | uint8_t | 1 byte |
| INT16 | int16_t | Little-endian, 2 bytes |
| UINT16 | uint16_t | Little-endian, 2 bytes |
| UINT24 | uint32_t | Read 3 Little-endian bytes and zero-extend the upper 8 bits |
| INT32 | int32_t | Little-endian, 4 bytes |
| UINT32 | uint32_t | Little-endian, 4 bytes |

The source's `Scale` is **multiplied by the received raw integer**.

```text
physical_value = raw_integer * Scale
```

For example, TEMP raw = 253 with Scale = 0.1 means 25.3 degC. IBAT raw = -125 means -12.5 A. Scale = 1.0 uses the integer directly. Encode command CMD values directly as the integers defined in the command tables.

Transmitted byte[0] corresponds to bits 7:0 of a 64-bit payload integer; byte[7] corresponds to bits 63:56. This specifies **payload byte order** and does not redefine the CAN hardware's physical bit transmission order as LSB-first. Decode Little-endian explicitly, regardless of receiver CPU endianness.

### Confirmation of Status Payload Field Positions

The `Source field arrangement` columns below reproduce the PDF's printed **left-to-right arrangement**. The source does not label field byte offsets or bit ranges, nor explicitly state whether the left side represents the first transmitted byte or the high bits of a 64-bit value. The Little-endian explanation alone cannot resolve these two interpretations.

Do not assign status fields to `data[0]` based solely on their left/right position in the source tables. Verify field positions against actual PMU transmission code, manufacturer confirmation or a CAN capture with known values before fixing decoder offsets. Field widths, meanings and byte order are specified below, but **individual byte offsets within status frames require confirmation**. Control command payloads contain only one uint32 CMD, so their field offset is unambiguously zero.

## Status Requests and Responses

During normal operation, the PMU does not spontaneously transmit periodic status packets. The Host sends an RTR request using the desired status ID; the PMU responds with a Data Frame using the same ID. The source does not define polling periods, response timeouts or retry counts.

| Status | Extended ID | Request direction | Request RTR | Request DLC | Response direction | Response RTR | Response DLC |
|---|---|---|---:|---|---|---:|---:|
| BATSTS | 0x10000100 | Host -> PMU | 1 | Source: N/A | PMU -> Host | 0 | 8 |
| ENGSTS | 0x10000110 | Host -> PMU | 1 | Source: N/A | PMU -> Host | 0 | 8 |
| AUX1STS | 0x10000120 | Host -> PMU | 1 | Source: N/A | PMU -> Host | 0 | 8 |
| AUX2STS | 0x10000130 | Host -> PMU | 1 | Source: N/A | PMU -> Host | 0 | 8 |
| VERSTS | 0x10000140 | Host -> PMU | 1 | Source: N/A | PMU -> Host | 0 | 8 |

Do not put command values into a status request payload. Responses have no defined transaction ID. Track responses by ID and reception time after a request; do not assume that delayed responses can be perfectly distinguished.

### BATSTS - Battery Information

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000100 | 0 | 8 | BSTS, TEMP, IBAT, GAGE |

| Field | Width | Type | Scale | Unit | Meaning |
|---|---:|---|---:|---|---|
| BSTS | 16 bits | Signedness not specified in source | N/A | - | Battery status, unused |
| TEMP | 16 bits | INT16 | 0.1 | degC | Battery temperature |
| IBAT | 16 bits | INT16 | 0.1 | A | Battery current; **positive = charging, negative = discharging** |
| GAGE | 16 bits | INT16 | 0.1 | % | Battery state of charge |

BSTS may be retained as an uninterpreted raw 16-bit value. Do not decode TEMP, IBAT or GAGE as UINT16.

### ENGSTS - Engine Information

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000110 | 0 | 8 | TEMP1, TEMP2, RPM, GAS, PCL |

| Field | Width | Type | Scale | Unit | Meaning |
|---|---:|---|---:|---|---|
| TEMP1 | 16 bits | INT16 | 0.1 | degC | Engine cylinder head 1 temperature |
| TEMP2 | 16 bits | INT16 | 0.1 | degC | Engine cylinder head 2 temperature |
| RPM | 16 bits | UINT16 | 1.0 | rpm | Engine rotational speed |
| GAS | 8 bits | INT8 | 1.0 | % | Remaining fuel |
| PCL | 8 bits | INT8 | 1.0 | % | Engine throttle position |

The source defines GAS and PCL as INT8. It does not define errors or sentinels for raw values outside normal percentage ranges; do not invent them.

### AUX1STS - Power and Control Information

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000120 | 0 | 8 | ICMD, ILD, VBUS, GCMD |

| Field | Width | Type | Scale | Unit | Meaning |
|---|---:|---|---|---|---|
| ICMD | Inferred as 16 bits; see below | TBD | TBD | A | Current control command, for developer debugging |
| ILD | 16 bits | INT16 | 0.1 | A | Load current; described as always positive |
| VBUS | 16 bits | INT16 | 0.1 | V | System voltage |
| GCMD | 16 bits | INT16 | 0.1 | % | Received battery-maintenance command value |

The descriptive text omits ICMD width, type and Scale. DLC 8 minus the other three INT16 fields leaves 16 bits, allowing only the width to be inferred. Do not assume its signedness or Scale matches adjacent fields. Treat it as uninterpreted raw data until confirmed.

### AUX2STS - PMU Status and Engine Operating Time

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000130 | 0 | 8 | HOURCNT, RSV, PSTS, PTEMP |

| Field | Width | Type | Scale | Unit | Meaning |
|---|---:|---|---:|---|---|
| HOURCNT | 24 bits | UINT24 | 1.0 | s | Accumulated engine operating time |
| RSV | 8 bits | UINT8 | N/A | - | Reserved |
| PSTS | 16 bits | UINT16 | N/A | Bit field | PMU status and warnings |
| PTEMP | 16 bits | INT16 | 0.1 | degC | PMU temperature |

Despite its name, HOURCNT is measured in **seconds**. For an hours display, use `hours = raw_seconds / 3600.0`. UINT24 ranges from 0 to 16,777,215. Behavior at the maximum value, counter reset conditions and persistence are not defined in the source.

#### PSTS Bit Definitions

Bit numbers refer to the decoded uint16 PSTS value. Test a bit using `bit = (PSTS >> n) & 1`.

| Bit | Mask | Meaning of 0 | Meaning of 1 |
|---:|---|---|---|
| 15 | 0x8000 | Warning inactive | Battery overvoltage warning |
| 14 | 0x4000 | Warning inactive | Battery undervoltage warning |
| 13 | 0x2000 | Warning inactive | Battery overcurrent warning |
| 12 | 0x1000 | Reserved | Reserved, unused |
| 11 | 0x0800 | Reserved | Reserved, unused |
| 10 | 0x0400 | Warning inactive | Engine cylinder high-temperature warning |
| 9 | 0x0200 | Warning inactive | Engine operating-time limit warning |
| 8 | 0x0100 | Warning inactive | Low-fuel warning |
| 7 | 0x0080 | Warning inactive | Unintended engine-stop warning; source wording: "random stop" |
| 6 | 0x0040 | Warning inactive | Engine RPM above warning threshold |
| 5 | 0x0020 | Reserved | Reserved, unused |
| 4 | 0x0010 | Engine cooling fan stopped | Engine cooling fan running |
| 3 | 0x0008 | Engine choke valve open | Engine choke valve closed |
| 2 | 0x0004 | Cleared after stopping operation completes | Engine stopping operation in progress |
| 1 | 0x0002 | Cleared after starting operation completes | Engine starting operation in progress |
| 0 | 0x0001 | PMU automatic operation | PMU manual operation |

Bits 1 and 2 describe starting/stopping **operations in progress** and return to zero after completion. Do not treat them as persistent Engine Running/Engine Stopped flags. The translation of bit 2 is somewhat ambiguous, but explicitly states that it clears after stopping completes; do not use it alone to establish the final stopped state. Warning thresholds, clearing delays and latch behavior are not specified.

### VERSTS - PMU Software Information

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000140 | 0 | 8 | DATE, MN, MJ, RSV |

| Field | Width | Type | Encoding | Meaning |
|---|---:|---|---|---|
| DATE | 32 bits | UINT32 | BCD | Software date |
| MN | 8 bits | UINT8 | BCD | Minor version |
| MJ | 8 bits | UINT8 | BCD | Major version |
| RSV | 16 bits | UINT16 | Reserved | Do not interpret |

MJ and MN use packed BCD, not ordinary binary integer encoding. Each valid nibble ranges from 0 to 9. For example, `0x12` represents version number 12, not decimal 18.

```text
bcd8_to_decimal(x) = 10 * ((x >> 4) & 0x0F) + (x & 0x0F)
```

Validate BCD nibbles before conversion. DATE is defined only as UINT32 BCD; an exact date format such as `YYYYMMDD` is not specified. Reconstruct the Little-endian raw uint32 and inspect its BCD digits, but do not assume a date-component order before confirmation.

## Control Commands

All control commands use Extended IDs, RTR = 0 and DLC = 4. The **entire four-byte payload is one UINT32 CMD**. Do not append four extra bytes or transmit DLC 8. `N/A` in the source table represents an untransmitted region.

| Command | Extended ID | Direction | RTR | DLC | Payload |
|---|---|---|---:|---:|---|
| BATCTRL | 0x10000000 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| ENGONOFF | 0x10000010 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| ENGMANUAL | 0x10000020 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| ENGPCL | 0x10000030 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| ENGCHK | 0x10000040 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |

| Payload offset | Width | Field | Position |
|---:|---:|---|---|
| 0 | 1 byte | CMD[7:0] | Least significant byte |
| 1 | 1 byte | CMD[15:8] | - |
| 2 | 1 byte | CMD[23:16] | - |
| 3 | 1 byte | CMD[31:24] | Most significant byte |

The source does not define command-specific ACK/echo frames, response timeouts or command retries. CAN hardware acknowledgment or successful TX queue submission does not establish successful PMU command execution. Status queries can verify operational state, but do not invent a separate application ACK and present it as part of the existing protocol.

### BATCTRL - Battery Maintenance and Generator/Battery Power Sharing

| CMD value | Operation |
|---:|---|
| 0-9 | Select generator/battery load-power sharing ratio |
| 10-100 | Battery charging/maintenance target [%] |

Although the source labels the 0-9 range with `%`, its examples define a **power-sharing code in tenths**. Do not interpret command 2 as 2% generator output.

| CMD | Generator : battery power share | Notes |
|---:|---|---|
| 0 | 0 : 10 | Battery supplies the entire load |
| 2 | 2 : 8 | Source example |
| 5 | 5 : 5 | Source example |
| 7 | 7 : 3 | Source example |

Generalizing the examples and range description gives `CMD : (10 - CMD)` for commands 0-9. This is an interpretation of the examples; actual control behavior at every value depends on the device implementation.

In the 10-100 range, charging occurs when battery state of charge is below the command value. The next source sentence translates as "If the command value is greater than the load power, the generator bears all of the load power." The comparison between a percentage command and load power is unclear. Exact power-control and charge-termination behavior requires confirmation; do not implement an additional Host power-control algorithm based solely on that sentence.

### ENGONOFF - Engine Start/Stop

| CMD | Operation |
|---:|---|
| 0 | Stop engine |
| 1 | Start engine |

Other values are undefined. The Host API must accept only 0/1. Device behavior for repeated start/stop commands and whether retries are permitted are not defined in the source.

### ENGMANUAL - Automatic/Manual Mode Selection

| CMD | Operation |
|---:|---|
| 0 | Automatic engine operation |
| 1 | Manual engine operation |

Check the operating mode through AUX2STS.PSTS bit 0. Do not use other CMD values.

### ENGPCL - Throttle Command in Manual Operation

| CMD | Unit | Applicability |
|---|---|---|
| UINT32, 0-100 | % | **Valid only in manual engine operating mode** |

The source does not specify a delay between ENGMANUAL mode selection and ENGPCL application. Do not assume throttle commands take effect in automatic mode.

### ENGCHK - Engine Choke Valve

| CMD | Operation |
|---:|---|
| 0 | Open choke valve |
| 1 | Close choke valve |

Check valve state through AUX2STS.PSTS bit 3. Its polarity matches the command: 1 = closed, 0 = open.

## Encoding Examples

These are synthetic examples of defined integer encodings, not actual captures. They do not establish the byte offsets of fields in status frames.

| Frame or value | Settings | Payload or field bytes |
|---|---|---|
| ENGONOFF start | ID 0x10000010, IDE 1, RTR 0, DLC 4, CMD 1 | 01 00 00 00 |
| ENGONOFF stop | ID 0x10000010, IDE 1, RTR 0, DLC 4, CMD 0 | 00 00 00 00 |
| ENGPCL 50% | ID 0x10000030, IDE 1, RTR 0, DLC 4, CMD 50 | 32 00 00 00 |
| BATCTRL target 80% | ID 0x10000000, IDE 1, RTR 0, DLC 4, CMD 80 | 50 00 00 00 |
| BATSTS request | ID 0x10000100, IDE 1, RTR 1 | No payload; RTR DLC requires confirmation |
| INT16 TEMP = 25.3 degC | raw = 253 | FD 00 |
| INT16 IBAT = -12.5 A | raw = -125 | 83 FF |
| UINT24 HOURCNT = 3600 s | raw = 3600 | 10 0E 00 |
| BCD version number 12 | raw = 0x12 | 12 |

## Implementation Instructions for AI Coding Agents

1. Implement Extended CAN RTR request transmission separately from Data Frame transmission/reception. Maintain five status-request IDs and five control-command IDs.
2. Decode status responses only after checking IDE = 1, RTR = 0, the appropriate status ID and DLC = 8. Frames with an invalid DLC must not overwrite the latest valid values or valid reception timestamps.
3. Maintain a separate last-valid-reception timestamp for each status. Determine polling periods, timeouts and retries from higher-level requirements; do not introduce them as fixed protocol values in this document.
4. Serialize every command as exactly four Little-endian UINT32 bytes. Validate command ranges; do not wrap invalid values or convert them into a different command.
5. Do not interpret reserved fields. Do not add undocumented status codes, sentinels, heartbeats, command ACKs or command retransmission behavior.
6. Use explicit multibyte loads/stores. Do not rely on C/C++ bitfield layout, structure padding, unaligned pointer casts or host endianness. Preserve sign extension for signed fields.
7. **Confirm before finalizing status decoders:** per-field byte offsets, ICMD type/Scale, DATE format and compatible RTR DLC. Do not label inferred decoders as confirmed implementations before manufacturer confirmation or capture validation.
8. Distinguish PSTS bits 1/2 operation-in-progress flags from actual operating state such as RPM. Test BATCTRL boundary values 9/10, mode changes, repeated commands, missing responses and delayed responses.
