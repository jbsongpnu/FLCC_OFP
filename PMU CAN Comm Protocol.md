# PMU CAN Communication Protocol

**Document version: 2.0** - **Status: Draft** - **Revision date: 2026-11-11**

Version 2.0 is under development. It is **not approved and has no official release date**.
The revision date above is the date carried by the Version 2.0 row of the revision history
below; it is neither an approval date nor a release date, and it is independent of the
dates of the ECU source documents cited at the end of this file.

## Revision History

| Version | Date | Status | Description |
|---|---|---|---|
| 1.05 | 2023.1.1 | Baseline | Modified for PNU from KAL version |
| 2.0 | 2026.11.11 | **Draft** | New messages for ECU(2nd version PMU) updated |

**Revision history policy.** This table carries **one row per protocol version, not per
source draft**. Version 2.0 is still being developed, so changes made while it is in draft
are folded into the Version 2.0 specification itself and do **not** get rows of their own:
no entry is added for an intermediate ECU draft such as r1 or r2, and the Version 2.0 date
does not move when a newer ECU source document arrives. The next row is added only when a
new protocol version is released.

## Version 2.0 Change Summary

Version 2.0 adopts a **reference CAN bit rate of 1.0 Mbps** where Version 1.05 quoted 500 kbps, and adds **ENERGYCFG (0x10000050)**, **MODESTS (0x10000150)**, and **ENERGYSTS (0x10000160)** for ECU software **2.00+**. It also defines ECU-specific behavior for existing commands, status flags and fields: the SOC charging setpoint, start/stop permissions, manual-command expiry, HOURCNT as UINT32, ICMD scaling, DATE encoding, and additional PSTS warnings. The prior five status IDs and five command IDs remain allocated unchanged. **ECU 2.00+ and legacy PMU behavior must be distinguished using VERSTS.** Both generations allow their bit rate to be configured, so the rate belongs to the installed bus and is not a version marker: match the bus before anything else, since at the wrong rate nothing is received at all, then read VERSTS to establish the generation.


## Applicability and compatibility

- **Version 1.05 / Legacy PMU:** Preserve the existing five status messages, five control commands, and the legacy command semantics described below where explicitly identified as legacy. Its status RTR DLC and certain source byte positions were not authoritatively specified in the existing document.
- **Version 2.0 / ECU software 2.00+:** Apply the ECU-specific sections of this document. Read **VERSTS** first; do not send ENERGYCFG or decode MODESTS/ENERGYSTS as the layouts defined here unless VERSTS confirms ECU software 2.00 or higher. If VERSTS is unavailable, fall back to the legacy message set.
- **Two compatibility classes, keyed on VERSTS:** a device reporting a version **below 2.0** gets legacy protocol behavior and the legacy messages only; a device reporting **2.0 or higher** enables the Version 2.0 definitions and the new ECU messages. If VERSTS cannot be obtained or decoded reliably, enable no Version 2.0-only feature. Backward compatibility with the older protocol is retained in every case.
- The ECU specification is **a draft for review**, not evidence of deployment or successful bench verification. It describes on-wire protocol behavior, not the current ArduPilot implementation state.

This document defines how a Host (FLCC or another upper-level controller) queries a legacy PMU (Power Management Unit) or the second-generation ECU (Engine Control Unit), and sends engine and power-control commands. Distinguish documented protocol values from implementation items requiring confirmation. Do not implement `TBD` or `confirmation required` items as arbitrary fixed values.

## CAN Communication Settings

| Item | Setting |
|---|---|
| CAN specification | Classic CAN, CAN Specification 2.0 B-Active |
| Bit rate | **Configured on the device.** ECU 2.00+ reference value **1.0 Mbps = 1,000,000 bit/s**; Version 1.05 reference value 500 kbps. Every node uses the rate its bus is configured for - see the note below |
| Identifier | **29-bit Extended ID**, IDE = 1 |
| Status request | Host -> PMU/ECU, Remote Frame, RTR = 1, no data payload; **ECU 2.00+: DLC = 8** |
| Status response | PMU/ECU -> Host, Data Frame, RTR = 0, DLC = 8 |
| Control command | Host -> PMU/ECU, Data Frame, RTR = 0, DLC = 4 for legacy commands; **ENERGYCFG DLC = 8** |
| Multibyte integers | **Little-endian, Least Significant Byte first** |
| Bit numbering | **bit 0 = LSB (Least Significant Bit)** |
| CAN FD | Not used |

**THE BIT RATE IS A BUS PROPERTY, NOT A VERSION MARKER.** Version 1.05 quotes 500 kbps and the
ECU 2.00+ draft quotes 1.0 Mbps, but both generations allow the rate to be configured on the
device, so neither figure identifies what is actually on a given bus. Set the Host to the rate
the bus is configured for; every node must share it, and a node at a different rate
communicates nothing at all rather than producing wrong values. **Do not infer the protocol
generation from the bit rate** - bring the link up first, then read VERSTS.

Request status by sending an **RTR frame**, not a Data Frame with the same ID. The Version 1.05 source marks request DLC as `N/A` and does not establish a numeric RTR DLC. **The Version 2.0 ECU explicitly requires RTR DLC = 8**. Even with DLC = 8, a Remote Frame carries no data bytes. Validate the legacy PMU RTR DLC separately before assuming ECU 2.00+ rules apply to it.

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

### Status Payload Byte Positions

The Version 2.0 ECU draft **explicitly defines byte offsets 0-7** for each field, with byte 0 being the first transmitted data byte. The offsets below therefore describe **ECU software 2.00+**. The earlier Version 1.05 document did not establish those positions unambiguously. Confirm legacy PMU offsets separately; do not infer that a decoder verified only against ECU 2.00+ is verified against the legacy PMU.

## Status Requests and Responses

During normal operation, the PMU does not spontaneously transmit periodic status packets. The Host sends an RTR request using the desired status ID; the PMU responds with a Data Frame using the same ID. The Version 1.05 source does not define polling periods, response timeouts or retry counts. **The Version 2.0 ECU draft defines a host polling/refresh recommendation**, documented in the ECU-specific section below; host timeouts remain implementation choices.

| Status | Extended ID | Request direction | Request RTR | Request DLC | Response direction | Response RTR | Response DLC |
|---|---|---|---:|---|---|---:|---:|
| BATSTS | 0x10000100 | Host -> PMU/ECU | 1 | V1.05 N/A; ECU 8 | PMU/ECU -> Host | 0 | 8 |
| ENGSTS | 0x10000110 | Host -> PMU/ECU | 1 | V1.05 N/A; ECU 8 | PMU/ECU -> Host | 0 | 8 |
| AUX1STS | 0x10000120 | Host -> PMU/ECU | 1 | V1.05 N/A; ECU 8 | PMU/ECU -> Host | 0 | 8 |
| AUX2STS | 0x10000130 | Host -> PMU/ECU | 1 | V1.05 N/A; ECU 8 | PMU/ECU -> Host | 0 | 8 |
| VERSTS | 0x10000140 | Host -> PMU/ECU | 1 | V1.05 N/A; ECU 8 | PMU/ECU -> Host | 0 | 8 |
| **MODESTS (new)** | **0x10000150** | Host -> ECU | 1 | **8** | ECU -> Host | 0 | **8** |
| **ENERGYSTS (new)** | **0x10000160** | Host -> ECU | 1 | **8** | ECU -> Host | 0 | **8** |

Do not put command values into a status request payload. Responses have no defined transaction ID. Track responses by ID and reception time after a request; do not assume that delayed responses can be perfectly distinguished.

### BATSTS - Battery Information

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000100 | 0 | 8 | BSTS, TEMP, IBAT, GAGE |

| Field | Width | Type | Scale | Unit | Meaning |
|---|---:|---|---:|---|---|
| BSTS | 16 bits | **ECU: UINT16**; legacy signedness unspecified | N/A | - | Reserved; **ECU always sends 0** |
| TEMP | 16 bits | INT16 | 0.1 | degC | Battery temperature |
| IBAT | 16 bits | INT16 | 0.1 | A | Battery current; **positive = charging, negative = discharging** |
| GAGE | 16 bits | INT16 | 0.1 | % | Battery state of charge |

**ECU byte offsets:** BSTS 0-1, TEMP 2-3, IBAT 4-5, GAGE 6-7. Legacy BSTS may be retained as an uninterpreted raw 16-bit value. Do not decode TEMP, IBAT or GAGE as UINT16.

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
| PCL | 8 bits | INT8 | 1.0 | % | **ECU: throttle setpoint**, not a measured position; legacy: throttle position |

**ECU byte offsets:** TEMP1 0-1, TEMP2 2-3, RPM 4-5, GAS 6, PCL 7. The ECU reports GAS as a 0-100% estimate derived from RPM/load and its configured tank/fuel inventory; GAS is not an independent fuel sensor reading. **The ECU retains its estimated remaining fuel level across power cycles**, saving the level as fuel is consumed and restoring it at boot; send ENERGYCFG.fuel with the measured tank quantity after refueling. ECU PCL is a throttle **setpoint**. The declared wire types remain INT8; the ECU states both values are never negative. For the legacy PMU, out-of-range/sentinel behavior remains unspecified.

### AUX1STS - Power and Control Information

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000120 | 0 | 8 | ICMD, ILD, VBUS, GCMD |

| Field | Width | Type | Scale | Unit | Meaning |
|---|---:|---|---|---|---|
| ICMD | 16 bits | **ECU: UINT16**, legacy TBD | **ECU: 0.1**, legacy TBD | A | **ECU:** EMS charge-current target; legacy: current control command for debugging |
| ILD | 16 bits | INT16 | 0.1 | A | Load current; described as always positive |
| VBUS | 16 bits | INT16 | 0.1 | V | System voltage |
| GCMD | 16 bits | INT16 | 0.1 | % | **ECU:** accepted BATCTRL SOC setpoint as a percentage at 0.1 %/count (84% = raw 840), 84% after power-on and each engine start; legacy: received battery-maintenance command |

**ECU byte offsets:** ICMD 0-1, ILD 2-3, VBUS 4-5, GCMD 6-7. For ECU 2.00+, ICMD is explicitly UINT16 with scale 0.1 A. The legacy source omitted ICMD type and scale; its old ambiguity is not retroactively resolved for legacy PMUs. ECU ILD/VBUS/GCMD are nonnegative in normal output, while the documented signed wire types remain unchanged.

**GCMD scaling is confirmed:** GCMD follows the declared scale of **0.1 % per raw count**. A raw value of **100 is 10.0%**, and 84% appears as raw 840 - the same convention as BATSTS.GAGE (84.0% = raw 0x0348 = 840 in the ECU example below) and consistent with the general rule `physical_value = raw * Scale`. **GCMD is not a numeric copy of the BATCTRL command.** The two carry the same quantity in different representations: BATCTRL `CMD = 10` means a 10% target and reads back as GCMD raw 100. Convert explicitly with `gcmd_percent = raw * 0.1` and never compare the raw field against the command integer. What remains open is whether GCMD reports the **requested** setpoint or the **effective** one after the EMS clamps it into its internal 54-90% range.

### AUX2STS - PMU Status and Engine Operating Time

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000130 | 0 | 8 | HOURCNT, RSV, PSTS, PTEMP |

| Field | Width | Type | Scale | Unit | Meaning |
|---|---:|---|---:|---|---|
| HOURCNT | **ECU: 32 bits; legacy: 24 bits** | **ECU: UINT32; legacy: UINT24** | 1.0 | s | Accumulated engine running time (not startup or stopping); ECU persists in whole minutes across power cycles |
| RSV | 8 bits | UINT8 | N/A | - | **Legacy only:** byte 3 reserved; **ECU:** byte 3 belongs to HOURCNT |
| PSTS | 16 bits | UINT16 | N/A | Bit field | PMU status and warnings |
| PTEMP | 16 bits | INT16 | 0.1 | degC | PMU temperature |

Despite its name, HOURCNT is measured in **seconds**. For an hours display, use `hours = raw_seconds / 3600.0`. **ECU offsets:** HOURCNT 0-3 (UINT32), PSTS 4-5, PTEMP 6-7. **Legacy layout:** HOURCNT 0-2 (UINT24), RSV 3, PSTS 4-5, PTEMP 6-7, subject to legacy offset verification. ECU HOURCNT counts only during running and is persisted in whole-minute increments. Legacy rollover/reset behavior was unspecified.

#### PSTS Bit Definitions

Bit numbers refer to the decoded uint16 PSTS value. Test a bit using `bit = (PSTS >> n) & 1`.

| Bit | Mask | Meaning of 0 | Meaning of 1 |
|---:|---|---|---|
| 15 | 0x8000 | Warning inactive | Battery overvoltage warning |
| 14 | 0x4000 | Warning inactive | Battery undervoltage warning |
| 13 | 0x2000 | Warning inactive | **ECU:** battery over-discharge (discharge >120 A default); **legacy:** battery overcurrent |
| 12 | 0x1000 | **Legacy:** reserved; **ECU:** inactive | **ECU: CRITICAL_FUEL**, fuel below 5%; also sets bit 8 |
| 11 | 0x0800 | **Legacy:** reserved; **ECU:** inactive | **ECU: GENERATOR_OVERLOAD**, running and load power greater than generator maximum |
| 10 | 0x0400 | Warning inactive | Engine cylinder high-temperature warning |
| 9 | 0x0200 | Warning inactive | Engine operating-time limit warning |
| 8 | 0x0100 | Warning inactive | Low-fuel warning |
| 7 | 0x0080 | Warning inactive | **ECU: STALL_UNDER_LOAD**, engine not running and load >30 A; **legacy:** unintended engine stop |
| 6 | 0x0040 | Warning inactive | Engine RPM above warning threshold |
| 5 | 0x0020 | Reserved | Reserved, unused |
| 4 | 0x0010 | Engine cooling fan stopped | Engine cooling fan running |
| 3 | 0x0008 | Engine choke valve open | Engine choke valve closed |
| 2 | 0x0004 | Cleared after stopping operation completes | Engine stopping operation in progress |
| 1 | 0x0002 | Cleared after starting operation completes | Engine starting operation in progress |
| 0 | 0x0001 | PMU automatic operation | PMU manual operation |

**ECU default PSTS thresholds and semantics:** bit 15 VBUS >50.0 V; 14 VBUS <46.0 V; 13 battery discharge >120 A; 12 fuel <5%; 11 running with generator overloaded; 10 either head >230 degC; 9 running time >=100 h; 8 fuel <15%; 7 engine not running and load >30 A; 6 RPM >6800; 4 both fan inputs running; 3 choke setpoint <50% (not set for uncalibrated choke); 2 stop sequence in progress; 1 start sequence in progress; 0 manual mode actively refreshed. ECU PSTS is refreshed every 10 ms; non-fuel thresholds are ECU parameter defaults, and flags clear when conditions end. The exact source and caution-bit treatment of invalid sensor readings are in the ECU error-handling section below.

Bits 1 and 2 describe starting/stopping **operations in progress** and return to zero after completion. Do not treat them as persistent Engine Running/Engine Stopped flags. The legacy translation of bit 2 is somewhat ambiguous, but explicitly states that it clears after stopping completes; do not use it alone to establish the final stopped state. **Legacy** warning thresholds, clearing delays and latch behavior are unspecified; ECU 2.00+ definitions and defaults appear above.

**Bit 9 is a maintenance indication, not an in-flight emergency.** The ECU sets it when accumulated engine operating time reaches 100 h. HOURCNT advances only while running but persists across power cycles, so the bit stays set on subsequent flights and the general rule that flags clear when their condition ends does not apply to it. The GCS checks it automatically before flight as part of pre-flight status monitoring and presents it as maintenance due, not as a flight alarm. Clearing the condition is an **ECU maintenance action performed through the ECU's internal UART service shell**: the FC/GCS neither resets the counter nor clears the bit, and no CAN-based clearing, counter-reset command or additional maintenance interface is defined or to be added.

### VERSTS - PMU Software Information

| Extended ID | RTR | DLC | Source field arrangement, left to right |
|---|---:|---:|---|
| 0x10000140 | 0 | 8 | DATE, MN, MJ, RSV |

| Field | Width | Type | Encoding | Meaning |
|---|---:|---|---|---|
| DATE | 32 bits | UINT32 | **ECU: BCD YYYYMMDD**; legacy date layout unknown | Software date |
| MN | 8 bits | UINT8 | BCD | Minor version |
| MJ | 8 bits | UINT8 | BCD | Major version |
| RSV | 16 bits | UINT16 | Reserved | Do not interpret |

MJ and MN use packed BCD, not ordinary binary integer encoding. Each valid nibble ranges from 0 to 9. For example, `0x12` represents version number 12, not decimal 18.

```text
bcd8_to_decimal(x) = 10 * ((x >> 4) & 0x0F) + (x & 0x0F)
```

Validate BCD nibbles before conversion. **ECU byte offsets:** DATE 0-3, MN 4, MJ 5, RSV 6-7 (zero). ECU DATE is specified as **BCD YYYYMMDD**. The legacy document only specified a BCD UINT32 date and left its component order unresolved. Read ECU VERSTS before enabling the new message set.

**VERSTS is the generation discriminator, and it needs a working link first.** The bit rate is configured per bus and is not evidence of generation, so bring the link up at the rate the bus is configured for and then request VERSTS. MJ/MN alone selects the message set: **below 2.0 -> legacy messages and legacy behavior only; 2.0 or higher -> the Version 2.0 definitions and the new ECU messages**. Treat an unanswered or undecodable VERSTS as below 2.0: do not send ENERGYCFG and do not decode MODESTS or ENERGYSTS in that case.

## Control Commands

All **five legacy control commands** use Extended IDs, RTR = 0 and DLC = 4. The **entire four-byte payload is one UINT32 CMD**. The **new ECU-only ENERGYCFG is a distinct 8-byte command** and must not be treated as a legacy UINT32 command. Do not append four extra bytes or transmit DLC 8 for a legacy command. `N/A` in the source table represents an untransmitted region.

| Command | Extended ID | Direction | RTR | DLC | Payload |
|---|---|---|---:|---:|---|
| BATCTRL | 0x10000000 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| ENGONOFF | 0x10000010 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| ENGMANUAL | 0x10000020 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| ENGPCL | 0x10000030 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| ENGCHK | 0x10000040 | Host -> PMU | 0 | 4 | CMD UINT32, Little-endian |
| **ENERGYCFG (new)** | **0x10000050** | Host -> ECU | 0 | **8** | Four UINT16 fields: tank/fuel/battery/soc |

| Payload offset | Width | Field | Position |
|---:|---:|---|---|
| 0 | 1 byte | CMD[7:0] | Least significant byte |
| 1 | 1 byte | CMD[15:8] | - |
| 2 | 1 byte | CMD[23:16] | - |
| 3 | 1 byte | CMD[31:24] | Most significant byte |

The legacy source does not define command-specific ACK/echo frames, response timeouts or command retries. ECU 2.00+ likewise has no application command ACK, but adds state-dependent command behavior and recommended host refresh and readback rules below. CAN hardware acknowledgment or successful TX queue submission does not establish successful PMU command execution. Status queries can verify operational state, but do not invent a separate application ACK and present it as part of the existing protocol.

### BATCTRL - Battery Maintenance and Generator/Battery Power Sharing (Legacy vs ECU)

**The two tables that follow describe the legacy PMU.** ECU 2.00+ ignores CMD 0-9 entirely and narrows the 10-100 range; read the ECU paragraph after them before implementing either table.

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

**ECU 2.00+ behavior:** CMD 0-9 is **ignored** (no generator/battery power-share mode). CMD 10-100 is an SOC setpoint; the EMS internally limits this to **54-90%** using ECU parameters. GCMD reports the accepted setpoint as a percentage at 0.1 %/count rather than as a copy of the CMD integer (see AUX1STS above); the setpoint returns to **84%** after power-on and on engine start. Refresh BATCTRL on change and every 500 ms per the new ECU draft. **The following sentence applies only to legacy documentation:** In the 10-100 range, charging occurs when battery state of charge is below the command value. The next source sentence translates as "If the command value is greater than the load power, the generator bears all of the load power." The comparison between a percentage command and load power is unclear. Exact power-control and charge-termination behavior requires confirmation; do not implement an additional Host power-control algorithm based solely on that sentence.

### ENGONOFF - Engine Start/Stop

| CMD | Operation |
|---:|---|
| 0 | Stop engine |
| 1 | Start engine |

Other values are undefined. The Host API must accept only 0/1. **ECU 2.00+:** The **ECU** accepts a Start only when the user shell is inactive, operation_mode is DO_NOTHING or STANDBY, no initial boot fault is present and its start gates pass (see MODESTS.start_gate); it enforces those conditions itself and reports a refusal in MODESTS.start_gate, so the Host does not reproduce them as a precondition. Start **only on change**; do not continuously resend engine start. Stop aborts a start and acts in any state except a stop already in progress; that case is handled inside the ECU. **Send each command once**, when the operator issues it or the requested state changes - Start on change, Stop on change, with **no periodic retransmission of either**. A successful CAN transmission means the frame reached the bus, not that the engine started or stopped: observe the outcome through ECU status, and do not expect or invent an application-level acknowledgment. The legacy source did not define those special rules.

### ENGMANUAL - Automatic/Manual Mode Selection

| CMD | Operation |
|---:|---|
| 0 | Automatic engine operation |
| 1 | Manual engine operation |

Check the operating mode through AUX2STS.PSTS bit 0. Do not use other CMD values. **ECU 2.00+:** manual control returns to automatic if ENGMANUAL is not received for **2 seconds**. Refresh ENGMANUAL on change and every **500 ms**; manual takes effect when running or in STANDBY after the engine stops.

### ENGPCL - Throttle Command in Manual Operation

| CMD | Unit | Applicability |
|---|---|---|
| UINT32, 0-100 | % | **Legacy: valid only in manual engine operating mode.** ECU 2.00+ accepts the command in any state but applies it only while manual control is active - see below |

The Version 1.05 source does not specify a delay between ENGMANUAL mode selection and ENGPCL application. **ECU 2.00+:** throttle commands are accepted in any state but used only while manual control is active; ECU applies a **50 %/s** slew limit (configurable ECU parameter), remembers ENGPCL through power-on runtime, and recommends sending ENGPCL **before** switching to manual mode. Refresh on change and every 500 ms.

### ENGCHK - Engine Choke Valve

| CMD | Operation |
|---:|---|
| 0 | Open choke valve |
| 1 | Close choke valve |

Check valve state through AUX2STS.PSTS bit 3. Its polarity matches the command: 1 = closed, 0 = open. **ECU 2.00+:** ENGCHK is ignored while the user shell is active. ECU opens the choke at power-on and does not move it automatically afterward; close can act as an emergency engine stop without normal ENGONOFF slowdown. No movement occurs if choke calibration is missing. This is a potentially safety-critical command; verify in an isolated bench setup.

**Choke polarity:** `CMD = 0` **opens** the choke and `CMD = 1` **closes** it, on ECU 2.00+ as on the legacy PMU. The ECU determines its internal choke setpoint itself and reports PSTS bit 3 by its own rule, `choke_setpoint < 50%`. That threshold is **descriptive of ECU internal behavior and informational for the Host**: the FC/GCS does not calculate, validate or reproduce it, and no Host-side setpoint computation or extra polarity check is required. Bit 3 is not set while choke calibration is missing. A close command can stop a running engine, so ENGCHK remains safety-critical - verify it on an isolated bench.

## Encoding Examples

These are synthetic examples of defined integer encodings, not actual captures. They do not establish the byte offsets of fields in status frames.

| Frame or value | Settings | Payload or field bytes |
|---|---|---|
| ENGONOFF start | ID 0x10000010, IDE 1, RTR 0, DLC 4, CMD 1 | 01 00 00 00 |
| ENGONOFF stop | ID 0x10000010, IDE 1, RTR 0, DLC 4, CMD 0 | 00 00 00 00 |
| ENGPCL 50% | ID 0x10000030, IDE 1, RTR 0, DLC 4, CMD 50 | 32 00 00 00 |
| BATCTRL target 80% | ID 0x10000000, IDE 1, RTR 0, DLC 4, CMD 80 | 50 00 00 00 |
| BATSTS request | ID 0x10000100, IDE 1, RTR 1 | No payload; **ECU DLC 8** (legacy DLC requires confirmation) |
| INT16 TEMP = 25.3 degC | raw = 253 | FD 00 |
| INT16 IBAT = -12.5 A | raw = -125 | 83 FF |
| UINT24 HOURCNT = 3600 s | raw = 3600 | 10 0E 00 |
| BCD version number 12 | raw = 0x12 | 12 |

## Implementation Instructions for AI Coding Agents

1. Implement Extended CAN RTR request transmission separately from Data Frame transmission/reception. Maintain the five legacy status-request IDs and five legacy control-command IDs, plus **two new ECU-only status IDs and one ECU-only command ID** when the ECU 2.00+ capability is confirmed.
2. Decode status responses only after checking IDE = 1, RTR = 0, the appropriate status ID and DLC = 8. Frames with an invalid DLC must not overwrite the latest valid values or valid reception timestamps.
3. Maintain a separate last-valid-reception timestamp for each status. Determine polling periods, timeouts and retries from higher-level requirements; do not introduce them as fixed protocol values in this document.
4. Serialize each **legacy command** as exactly four Little-endian UINT32 bytes; encode ENERGYCFG as four Little-endian UINT16 fields (DLC 8). Validate command ranges; do not wrap invalid values or convert them into a different command.
5. Do not interpret reserved fields. Do not add undocumented status codes, sentinels, heartbeats, or command ACKs; apply the **explicit Version 2.0 ECU status sentinels and resending rules** only to ECU 2.00+.
6. Use explicit multibyte loads/stores. Do not rely on C/C++ bitfield layout, structure padding, unaligned pointer casts or host endianness. Preserve sign extension for signed fields.
7. **For legacy PMUs, confirm before finalizing status decoders:** per-field byte offsets, ICMD type/Scale, DATE format and compatible RTR DLC. **For ECU 2.00+, these values are explicitly defined in the ECU draft and are listed above**. Do not label inferred decoders as confirmed implementations before manufacturer confirmation or capture validation.
8. Distinguish PSTS bits 1/2 operation-in-progress flags from actual operating state such as RPM. Test BATCTRL boundary values 9/10, mode changes, repeated commands, missing responses and delayed responses.


## Version 2.0 - ECU-Specific Message Map and Timing

The following rules come from **ECU_CAN_Protocol_r2.pdf, draft for review dated 2026-11-12** and apply to **ECU software 2.00+** only. The original command/status IDs, Little-endian encoding and base field widths (except HOURCNT) are retained. **Reference bit rate:** the ECU draft quotes 1.0 Mbps where Version 1.05 quoted 500 kbps, but both are configurable on the device and the operating rate is a property of the bus (see the CAN settings note above). The ECU accepts the reserved ID blocks 0x10000000-0x100000F0 (commands) and 0x10000100-0x100001F0 (status), on 0x10 boundaries; only the allocated IDs below are defined.

| ID (29-bit Extended) | Name | Direction | Frame / DLC | ECU host timing |
|---|---|---|---|---|
| 0x10000000 | BATCTRL | Host -> ECU | Data / 4 | On change and every 500 ms |
| 0x10000010 | ENGONOFF | Host -> ECU | Data / 4 | Start and Stop each sent once on change; no periodic retransmission |
| 0x10000020 | ENGMANUAL | Host -> ECU | Data / 4 | On change and every 500 ms; reverts to automatic after 2 s silence |
| 0x10000030 | ENGPCL | Host -> ECU | Data / 4 | On change and every 500 ms |
| 0x10000040 | ENGCHK | Host -> ECU | Data / 4 | On change and every 500 ms |
| **0x10000050** | **ENERGYCFG (new)** | Host -> ECU | **Data / 8** | Operator action only; read back after 1 s; at most 3 transmissions, abort pending retries if ECU leaves DO_NOTHING/STANDBY |
| 0x10000100 | BATSTS | ECU -> Host | RTR request / 8-byte response | Poll every 100 ms |
| 0x10000110 | ENGSTS | ECU -> Host | RTR request / 8-byte response | Poll every 100 ms |
| 0x10000120 | AUX1STS | ECU -> Host | RTR request / 8-byte response | Poll every 100 ms |
| 0x10000130 | AUX2STS | ECU -> Host | RTR request / 8-byte response | Poll every 100 ms |
| 0x10000140 | VERSTS | ECU -> Host | RTR request / 8-byte response | Once after connecting |
| **0x10000150** | **MODESTS (new)** | ECU -> Host | RTR request / 8-byte response | Poll every 100 ms |
| **0x10000160** | **ENERGYSTS (new)** | ECU -> Host | RTR request / 8-byte response | Poll every 1 s |

ECU status is **request/response only**; the ECU does not spontaneously broadcast these status frames. No application-layer command acknowledgment is defined. ECU 2.00+ specifically uses **RTR DLC 8**. Legacy five-command payloads remain DLC 4 even though ENERGYCFG is DLC 8. The ECU estimates its worst-case bus load as approximately **13.4 kbit/s** using its documented timing assumptions, which is **1.3% of a 1.0 Mbps bus**. The ECU draft quotes that same absolute figure as 2.7% against 500 kbit/s: bus load in bit/s does not change with the configured bit rate, only the percentage does. This is a draft estimate, not a measurement.

### MODESTS - ECU Operating Modes, RPM Target, and Start Permission (New)

**MODESTS is still under development.** The message ID, byte layout, field definitions, values and encoding example below are the current definition and are preserved as such, but they may change while Version 2.0 is in draft. The message is **informational for the Host at this stage**: the ECU enforces its own modes and start permissions, so no new FC/GCS interlock, mode-dependent command algorithm or safety check is required on top of it, beyond the send conditions already stated for individual commands such as ENERGYCFG. Describing an ECU state in this ICD does not make the Host responsible for enforcing it.

| Property | Value |
|---|---|
| Extended ID | **0x10000150** |
| Direction | ECU -> Host, in response to RTR |
| Frame | Data, IDE 1, RTR 0, DLC 8 |
| Encoding | Byte 0 contains two 4-bit mode fields; bytes 1-5 are UINT8 fields; bytes 6-7 hold a little-endian UINT16 |

| Byte(s) | Bits | Field | Type | Meaning |
|---|---|---|---|---|
| 0 | 0-3 | system_state | UINT4 | 1 INITIALIZATION; 2 READY; 3 ENGINE_RUN; 4 USER_SHELL |
| 0 | 4-7 | operation_mode | UINT4 | 1 DO_NOTHING; 3 STANDBY; 4 STARTUP; 5 RUNNING; 6 STOP |
| 1 | 0-7 | ctrl_mode | UINT8 | 1 NONE; 2 MANUAL; 3 STARTUP; 4 STOP; 5 FIXED_RPM; 6 EMS; 7 INACTIVE; 8 FAIL_TACHO; 9 FAIL_CURR_SENSOR; 10 FAIL_VOLT_SENSOR |
| 2 | 0-7 | alert_level | UINT8 | 0 NONE; 1 ADVISORY (plan to land); 2 IMMEDIATE (land now) |
| 3 | 0-7 | ems_mode | UINT8 | 0 INIT; 1 START; 2 WARMUP; 3 CHARGE; 4 IDLE; 5 DISCHARGE; 6 EMERGENCY; 7 STOPPING; 8 THERMAL_HOLD; 9 FORCED_CHARGE |
| 4 | 0-7 | start_gate | UINT8 | Result of last start attempt; codes below |
| 5 | 0-7 | initial_fail | UINT8 bitfield | Boot/calibration/fuel prerequisites below; any set bit blocks Start |
| 6-7 | 0-15 | rpm_target | UINT16 | Governor RPM target, **1 rpm per count**, rounded; zero when RPM is not under governor control |

Decode byte 0 explicitly: `system_state = data[0] & 0x0F`, `operation_mode = (data[0] >> 4) & 0x0F`. Decode `rpm_target = uint16(data[6]) | (uint16(data[7]) << 8)`; do **not** interpret bytes 6-7 as `initial_fail` or reserved.

**Undefined codes and the ambiguous zero.** `system_state` defines 1-4 and `operation_mode` defines 1 and 3-6: **0 is defined for neither, and 2 is not defined for `operation_mode`**, while the ECU also transmits reserved fields as 0. A decoder cannot distinguish "not yet initialized" from "field not populated". Decode 0 and every undefined code as **unknown**: report the raw value, do not map it onto the nearest defined state, and do not assign it a meaning. The same rule applies to `ctrl_mode`, `ems_mode` and `start_gate` values not listed below.

**rpm_target behavior:** This is the governor's current target, including EMS corrections to align IBAT with ICMD, so the target can change while battery current differs from its target. It is zero when the ECU is not controlling RPM (engine stopped, cranking, MANUAL, and certain fail/stop modes that hold the throttle or regulate bus voltage). During an over-RPM cut, the throttle may be forced to zero while `rpm_target` still reports the target. Temporary missing tachometer pulses can also leave it nonzero; once `FAIL_TACHO` is entered, it reports zero. Do not equate `rpm_target=0` with confirmed engine stop.

**start_gate (byte 4):**

| Code | Name / description |
|---:|---|
| 0 | OK - start gates passed (cranking or engine started), **or** no attempt since power-on; not proof of running |
| 4 | BLOCKED_ALREADY_RUNNING - engine already spinning |
| 5 | BLOCKED_EEPROM_BAD - invalid EEPROM, initial_fail 0x01 or 0x02 |
| 6 | BLOCKED_CALIB_MISSING - missing sensor calibration, initial_fail 0x04 |
| 7 | BLOCKED_TACHO_FAULT - invalid tachometer |
| 8 | BLOCKED_THROTTLE_UNCALIBRATED |
| 9 | BLOCKED_CHOKE_UNCALIBRATED |
| 10 | BLOCKED_STARTER_UNCALIBRATED |
| 11 | BLOCKED_SHELL_ACTIVE |
| 12 | BLOCKED_NOT_STANDBY - mode not STANDBY or DO_NOTHING |
| 13 | BLOCKED_FUEL_LEVEL - fuel level not initialized, initial_fail 0x20 |
| 15 | ABORTED_TIMEOUT - cranked but engine did not run within permitted start time |
| 16 | ABORTED_NO_ROTATION - starter driven but no tachometer pulses |
| 17 | ABORTED_STOPPED_AT_HANDOVER - engine fired but stopped during starter handover |
| 18 | ABORTED_STOP_REQUESTED - Stop received during cranking |

`BLOCKED_*` denotes a rejected Start **before cranking**. `ABORTED_*` denotes a Start terminated **after cranking began**. Following an aborted Start, `operation_mode` returns to STANDBY (via STOP for code 18). The last `start_gate` code persists until the next Start attempt. Codes 1-3 and 14 are not defined in this draft; do not assign meanings.

**initial_fail (byte 5) bit masks:** `0x01 HEADER_BAD`, `0x02 EEPROM_LAYOUT`, `0x04 CALIB_MISSING`, `0x20 FUEL_LEVEL`. Any set bit blocks engine start **inside the ECU**, which performs that check itself. Bits `0x08`, `0x10`, `0x40` and `0x80` are **not defined in this draft** and are given no meaning here. The field is **still under development** and is carried as ECU status information: display it, raw value included, but do not evaluate it in the Host or build an additional Start interlock on it.

**alert_level:** Level 2 is raised for SOC at landing reserve or SOC-low in combination with low fuel or cylinder temperature above 260 degC; clears to the current level five seconds after the causes clear. Level 1 includes low SOC, EMS THERMAL_HOLD/FORCED_CHARGE, or low fuel/fuel endurance; clears two seconds after all causes clear. Consult the ECU's configured thresholds for actual behavior.

### ENERGYSTS - ECU Energy Configuration and Endurance (New)

| Property | Value |
|---|---|
| Extended ID | **0x10000160** |
| Direction | ECU -> Host, RTR-requested status |
| Frame | Data, IDE 1, RTR 0, DLC 8 |

| Bytes | Field | Type | Scale | Meaning |
|---|---|---|---|---|
| 0-1 | tank | UINT16 | 0.01 L | Configured tank capacity |
| 2-3 | battery | UINT16 | 0.1 Ah | Configured battery capacity |
| 4-5 | endurance | UINT16 | 1 s | Estimated remaining fuel endurance at current consumption |
| 6-7 | reserved | UINT16 | - | Always zero |

`endurance=0xFFFE`: 65,534 seconds or more. `endurance=0xFFFF`: unknown (no fuel currently being burned). These **sentinels override the normal scale**.

`endurance` is therefore **only meaningful while fuel is being burned**; with the engine stopped the ECU reports 0xFFFF. The GCS displays the received value according to this encoding, with an explicit **Unknown** indication for 0xFFFF and "65,534 s or more" for 0xFFFE. Never scale either sentinel as an ordinary count, and never render Unknown as zero remaining endurance. The Host is **not** required to compute or estimate a pre-flight endurance; such a feature may be considered separately in future and is not part of the current implementation requirements.

### ENERGYCFG - ECU Energy Configuration Command (New)

| Property | Value |
|---|---|
| Extended ID | **0x10000050** |
| Direction | Host -> ECU |
| Frame | Data, IDE 1, RTR 0, **DLC 8** |
| Encoding | Four UINT16 little-endian fields; not the legacy UINT32 CMD command format |

| Bytes | Field | Type | Scale | Raw range | Operation / readback |
|---|---|---|---|---|---|
| 0-1 | tank | UINT16 | 0.01 L | 10-10000 | Tank capacity; verify exactly using ENERGYSTS.tank |
| 2-3 | fuel | UINT16 | 0.01 L | 0-tank | **Current total fuel in the tank**, measured directly from the fuel tank level indication; stored across power cycles. Operator enters litres; encode as `fuel_raw = round(fuel_litres * 100)`. Verify through ENGSTS.GAS using `floor(100 * fuel / tank)` or one percentage point less due to floating-point rounding |
| 4-5 | battery | UINT16 | 0.1 Ah | 50-1000 | Battery capacity; verify exactly using ENERGYSTS.battery |
| 6-7 | soc | UINT16 | 0.1 % | 0-1000 | **Optional SOC correction**; otherwise send `0xFFFF` to leave the voltage-derived estimate unchanged. Check BATSTS.GAGE within 0.1 percentage point, accounting for truncation and ongoing battery current |

**Special and acceptance rules:**

1. A field equal to `0xFFFF` means **keep the current value**. All provided fields are validated before any change; if one is invalid, **none** is applied. `fuel` may not exceed the supplied or retained `tank`.
2. The ECU accepts ENERGYCFG only when the user shell is inactive and `operation_mode` is **DO_NOTHING (1) or STANDBY (3)**. Frames outside these conditions are ignored, without a CAN acknowledgment.
3. **Operator action only.** Never send ENERGYCFG automatically on startup, reconnection, or from a saved command list. Send one frame per operator action, wait 1 second, and compare supplied values with the corresponding readbacks. If they disagree beyond the field-specific tolerances, resend the **same** frame, for **no more than 3 total transmissions**. After 3 unsuccessful frames, notify the operator and show the values read back.
4. **Cancel pending retries immediately** if `operation_mode` leaves DO_NOTHING or STANDBY (e.g., engine startup). Never send a delayed retry after the engine has started.
5. The `fuel` field is the **current total fuel quantity in the tank**, not an amount added. Read it from the fuel tank level indication, have the operator enter it in litres, and encode it as `fuel_raw = round(fuel_litres * 100)`. On receipt the ECU **replaces its estimated fuel quantity with the received value** and stores it across power cycles, then decrements that estimate as fuel is consumed. Transmit ENERGYCFG.fuel **after refueling**. Do not automatically reinitialize the tank quantity at boot.
6. The `soc` field is optional. On power-up the ECU estimates SOC from battery voltage; supply a trusted SOC only when intentionally correcting that estimate, otherwise use `0xFFFF`.
7. **The protocol has no incremental fuel addition.** The Host shall never compute the field by adding a refueling amount to a previously estimated or read-back quantity; send the measured total only. Readback is in any case indirect and quantized: `GAS` is an integer percentage of tank capacity rounded down and may read **one additional percentage point lower** due to floating-point rounding, so a value derived from it is not an accurate base to build on and may differ slightly from the quantity entered.
8. A successful fuel setting clears `initial_fail` bit `0x20 FUEL_LEVEL`. Merely observing a retained/estimated fuel quantity does not independently establish that the ECU's fuel-level prerequisite is satisfied.

### ECU 2.00+ - Error Handling and Field Validity

| Situation | ECU behavior / Host rule |
|---|---|
| Unexpected 11-bit CAN frame | ECU ignores; use 29-bit Extended identifiers |
| RTR on command ID | ECU ignores; Host shall not send |
| Status request | Use RTR DLC 8; response IDE 1, RTR 0, DLC 8, matching ID |
| Data frame on a status ID | ECU draft says it is treated as a status request; Host should still use RTR only |
| Command too short | ECU ignores |
| Legacy command DLC > 4 | ECU uses bytes 0-3; Host should still send DLC 4 |
| ENERGYCFG DLC < 8 | ECU ignores |
| ENERGYCFG during USER_SHELL or outside DO_NOTHING/STANDBY | ECU ignores the frame without CAN acknowledgment; Host must not send it in these conditions |
| ENERGYCFG readback mismatch | Recheck after 1 s; retry the identical frame up to 3 total sends, then report failure and readback values |
| ENERGYCFG pending retry when operation_mode changes | Cancel pending transmissions as soon as operation_mode leaves DO_NOTHING or STANDBY; never replay on startup/reconnect |
| Out-of-range command | Ignored with no partial update; Host shall validate before sending |
| Command disallowed in current state | Ignored; refused Start updates MODESTS.start_gate |
| ENGMANUAL silent for 2 s | ECU returns to automatic; Host refreshes every 500 ms when manual is intended |
| New RTR request before previous same-ID response is sent | ECU may not answer new request; retry at next poll |
| Missing/malformed reply | Host retains last valid decoded value and timestamp and handles timeout/retry |
| CAN bus-off | ECU states automatic recovery; this does not replace Host health monitoring |
| Reserve / unknown fields | ECU transmits reserved bytes/bits as 0 and receiver ignores them |
| Out-of-range measured value | ECU saturates to representable type bounds, no wrapping; negative values in unsigned fields become zero |
| Sensor reading unavailable | ECU sends raw 0 for failed reading and raises relevant PSTS caution bits, subject to exceptions below |

**Sensor failure and caution bits from ECU draft:** TEMP1 or TEMP2 failure raises PSTS bit 10; IBAT raises bit 13; invalid RPM raises bit 6, and tachometer loss while running can put `ctrl_mode=8 FAIL_TACHO`; VBUS raises bits 15 and 14 (and bit 11 while RUNNING); ILD raises bit 7 when not turning or bit 11 while RUNNING, but neither during start/stop rotation; battery TEMP and ECU PTEMP failure are sent as 0.0 degC with **no caution bit**. Therefore a measured zero is not always a reliable validity indicator. Do not infer a stopped engine from RPM=0 without considering mode and fault flags.

**PSTS and modes are not interchangeable:** PSTS bits 1/2 indicate start/stop *sequences*, not stable RUNNING/STOP states; MODESTS.operation_mode carries the ECU's own statement of its state. Alerts and faults must be handled according to their documented definitions, not by assuming all 1-bits mean the same fault severity.

## Version 2.0 Source Comparison and Open Clarifications

The following notes distinguish **explicit ECU changes**, **newly defined details previously unknown**, and **remaining implementation clarifications**:

1. **Status field positions:** The Version 1.05 Markdown explicitly considered byte positions **unconfirmed**; the ECU draft defines offsets. Treat offsets as confirmed **by the new draft for ECU 2.00+**, not retroactively confirmed on old PMU hardware.
2. **RTR request DLC:** Version 1.05 source said N/A (no payload), whereas the ECU draft explicitly says **DLC 8**. This is a newly defined requirement, not proof that old PMU request DLC was different.
3. **VERSTS date and ICMD:** Previously unspecified, now defined for ECU 2.00+ (DATE BCD YYYYMMDD, ICMD UINT16 x 0.1 A). These are **clarifications**, not evidence that old values encoded differently.
4. **Legacy request/refresh cadence:** The original Markdown said polling rates and retries were not defined. The ECU draft gives 100 ms status polls (except VERSTS on connection, ENERGYSTS 1 s) and specific 500 ms command refresh recommendations. Do not apply these automatically to legacy PMUs.
5. **Observing that a Stop has completed:** ENGONOFF Stop is sent once; the draft's 500 ms repeat is **not adopted** (see ENGONOFF above). The Host therefore needs a status indication that a stop has **completed** rather than merely **started**, and the draft notes that PSTS STOP_SEQUENCE may be too brief to catch and `operation_mode` STOP can pass between polls. Ask the ECU author which field gives a reliable completed-stop indication at a 100 ms poll rate.
6. **ECU `ENERGYSTS.endurance` encoding:** The example `0x1C20` represents **7200 s (2 hours)** and matches the scale, confirming the little-endian example `20 1C` is self-consistent.
7. **ECU draft `BATSTS.GAGE` range notation:** It shows `0-1000` even though scale is 0.1%; interpret that as raw 0-1000 corresponding to physical 0-100%, not 0-1000%.
8. **PMU-ECU compatibility with FLCC Step 2c:** Start-on-change-only applies to the **Host/FC -> ECU** CAN link, not necessarily to the **GCS -> FC** command-burst scheme. Preserve that distinction when reviewing the implementation; this protocol specification does **not** automatically authorize altering the existing FC interlock.
9. **Generator overload, failed-reading warnings and fuel model:** The new draft specifies detailed ECU internals and sensor-failure behaviors that the Version 1.05 source did not define. They are ECU-specific, not grounds to rewrite legacy PMU failure semantics.
10. **AUX1STS.GCMD, requested or effective setpoint:** The scale is settled - 0.1 % per raw count, raw 100 = 10.0%. What is not settled is whether GCMD reports the setpoint **as requested** or **as applied** after the EMS clamps it to its internal 54-90% range. Ask the ECU author; until then do not use GCMD to confirm that a BATCTRL value was accepted unchanged.
11. **MODESTS undefined codes:** Confirm `operation_mode` 2, the meaning of a zero nibble in either half of byte 0, `start_gate` 1-3 and 14, and `initial_fail` bits 0x08/0x10/0x40/0x80. They are decoded as unknown and displayed raw; the Host does not act on them.
12. **MODESTS is under development and its layout is unversioned:** The frame carries no layout version and VERSTS identifies software rather than frame layout. Re-validate the field order against a capture before freezing a MODESTS decoder.

### ECU Encoding Examples (from the new draft)

| Message | Encoded bytes (hex) | Explanation |
|---|---|---|
| BATSTS reply | `00 00 FD 00 83 FF 48 03` | BSTS=0, TEMP=25.3 degC, IBAT=-12.5 A, GAGE=84.0% |
| AUX2STS reply | `10 0E 00 00 11 00 9F 01` | HOURCNT=3600 s, PSTS=0x0011, PTEMP=41.5 degC |
| VERSTS reply | `01 12 26 20 00 02 00 00` | DATE=2026-12-01 BCD, ECU version 2.00 |
| MODESTS reply | `53 06 00 03 00 00 CC 10` | Byte 0: ENGINE_RUN (3) / RUNNING (5); ctrl_mode=EMS (6); alert=NONE (0); ems_mode=CHARGE (3); start_gate=0; initial_fail=0; rpm_target=4300 rpm (0x10CC) |
| ENERGYSTS reply | `20 03 DC 00 20 1C 00 00` | Tank=8.00 L, battery=22.0 Ah, endurance=7200 s |
| ENERGYCFG command | `20 03 8A 02 DC 00 B6 03` | Tank=8.00 L, fuel=6.50 L, battery=22.0 Ah, SOC=95.0% |
| ENERGYCFG keep other settings | `FF FF 8A 02 FF FF FF FF` | Update fuel to 6.50 L, retain the other three fields |

**References:** Original `PMU CAN Comm Protocol.md` (Version 1.05 baseline); `ECU_CAN_Protocol_r2.pdf` (9 pages, draft for review dated 2026-11-12). The ECU draft defines the ECU 2.00+ behavior above. No physical ECU communication verification has been performed as part of this document revision. `ECU_CAN_Protocol_r2.pdf` is **not stored in this repository**, so nothing in the ECU sections can be re-checked against its source from a clone alone; keep a copy with the project records and cite that exact file when raising any of the clarifications above with the ECU author.
