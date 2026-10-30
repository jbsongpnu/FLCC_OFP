# SAR CAN and MAVLink Interface Control Document

Filename: SAR_ICD_Rev1.0_ENG.md  
Revision: Rev1.0  
Date: 2026-10-20  
Status: Rev1.0 interface specification; explicitly deferred items remain TBD  
Audience: GCS, FLCC, and SAR device developers, including AI coding agents

## 0 Bilingual document maintenance

The Korean document SAR_ICD_Rev1.0.md and this English document SAR_ICD_Rev1.0_ENG.md are two language versions of the same specification. Any future change must be applied to both documents in the same update. Keep revision identifiers, message IDs, field names, types, byte offsets, scaling, units, enumeration values, validity rules, transmission rates, examples, and open issues synchronized. Do not update only one language version.

Technical requirements, proposals, and unresolved items have the same status in both versions. A translation must not promote a proposal or a TBD item into an approved requirement. If the two versions disagree, identify and resolve the discrepancy in both before implementation. Both filenames use the same revision notation; the _ENG suffix identifies the English version.

This English version translates the Korean Rev1.0 specification. It does not introduce additional protocol behavior. Instructions expressed as must/must not apply to the defined behavior within this specification; provisions explicitly marked proposed, recommended, optional, or TBD retain that qualification and must not be treated as approved requirements.

## 1 Purpose and scope

This document defines the MAVLink interface between GCS and FLCC and the CAN interface between FLCC and SAR for integrating a SAR device. FC and FLCC refer to the same flight controller. SAR denotes the mission equipment interface, including its companion-computer communication function.

- GCS to FLCC: observation preparation, start, stop, and observation-mode commands.
- FLCC to GCS: actual SAR status and the FC-maintained valid-heartbeat reception count at 5 Hz.
- FLCC to SAR: GPS/GPA navigation data and actual Roll/Pitch/Yaw of the gimbal-stabilized SAR device.
- FLCC to SAR / SAR to FLCC: forwarding of control commands and return of actual device status.

Fixed requirements are the MAVLink message IDs, names, fields, status values and 5 Hz TM rate; SAR command/response IDs and DLCs; the 1 Hz Heart_Beat; the RTK Fixed start interlock; and response-driven state transitions and navigation transmission gating. The navigation integer scales/resolutions, CAN 1 Mbps, heartbeat 3 s, FC response 2 s, GCS 3 s per attempt with at most one retry, source-data age limits, and operating decisions below are fixed. Only manufacturer-reserved payloads and explicitly identified follow-up implementation items remain unresolved. This document supersedes the previously considered CAN IDs 0x10 through 0x12 and float64 latitude/longitude encoding.

### 1.1 Development and ICD ownership

| Scope | Owner | Responsibility |
|---|---|---|
| Overall ICD and configuration management | PNU Drone | Synchronize Korean/English documents, integrate ID/interface changes and agreed vendor decisions |
| FC and GCS software | PNU Drone | ArduPilot changes, MAVLink 50050/51050, CAN TX/RX and conversion, states/timers/retry policy, GCS buttons and message window |
| FC to SAR ICD specification | PNU Drone | Maintain navigation 0x400-0x408 and command 0x501-0x503 specifications |
| SAR device software/firmware | Lumir | Implement SAR-side CAN reception/decoding, command execution, observation control, responses and heartbeat generation |
| SAR to FC ICD content | Lumir | Supply transmission definitions/semantics and proposed changes for 0x510, 0x521, 0x522, 0x523; PNU Drone integrates them into the overall ICD |
| Integration validation | PNU Drone / Lumir | Validate each owned implementation and perform joint tests |

Specification authorship does not remove receiver implementation responsibility. PNU Drone owns the FC-to-SAR command specification; Lumir implements reception/execution in SAR, idempotent repeated commands without toggling, and the corresponding responses. Changes to Lumir's SAR-to-FC definitions are integrated under PNU Drone's overall ICD management and agreement between both vendors.

## 2 Representation and conversion rules

### 2.1 Transmitter-side multiplication

In every table, multiplication is the **factor applied to the physical value by the transmitter**.

```text
transmitted_integer = round_half_away_from_zero(physical_value * multiplication)
reconstructed_physical_value = received_integer / multiplication
```

For example, multiplication = 1000 converts 1.234 to transmitted integer 1234. Multiplication = 100 converts 12.34 m to 1234. The same rule applies to negative values: -1.235 * 100 rounds to -124. The theoretical quantization error is at most 0.5 LSB within the valid range.

The input units in the tables are physical units interpreted by a log viewer, such as deg, m, and m/s. Do not multiply an ArduPilot internal value again if it is already scaled. For example, if Location.lat/lng already contain deg * 10^7, use those int32 values directly. If altitude is already in cm, do not apply *100 again. Do not double-scale the internal *100 representations of HDop/VDop either.

Do not use a float32 intermediate when converting latitude/longitude. Use the internal int32 representation directly or perform the conversion with double precision. A resolution of 10^-7 deg corresponds to approximately 1.1 cm of latitude near the equator; representation resolution does not imply actual positioning accuracy.

### 2.2 Types and byte order

| Notation | Width in bits | Description |
|---|---:|---|
| u8 | 8 | uint8_t |
| i16 | 16 | int16_t, two's complement |
| u16 | 16 | uint16_t |
| i32 | 32 | int32_t, two's complement |
| u32 | 32 | uint32_t |
| u64 | 64 | uint64_t, for the required snapshot timestamp |

All multi-byte fields are transmitted in **Little-endian, Least Significant Byte first** order. This byte-order rule applies to every multi-byte payload field in both CAN and MAVLink. B0 means CAN data[0]; B0-3 means data[0] through data[3], inclusive. Do not serialize a C/C++ structure by blindly copying its memory, relying on structure padding or host byte order. No floating-point fields are used in the CAN payloads.

For every bitfield, **bit 0 is the LSB (Least Significant Bit)**. For example, the uint32 value 0x12345678 occupies B0 through B3 as 78 56 34 12; the first byte contains value bits 7 through 0. In a single-byte uint8 field, bit 0 is also the LSB, but no inter-byte reordering is involved. Distinguish LSB, a bit-significance term, from Least Significant Byte first, a byte-order rule. These rules define the logical payload representation; they do not change or redefine the CAN controller's physical bit transmission order within a frame as LSB-first.

### 2.3 Unavailable values and out-of-range values

The following sentinels apply to navigation physical quantities. Counts, enums, bitfields, and counters follow their individual definitions.

| Type | Invalid sentinel | Valid integer range |
|---|---|---|
| i16 | -32768 / 0x8000 | -32767 to 32767 |
| u16 | 65535 / 0xFFFF | 0 to 65534 |
| i32 | -2147483648 / 0x80000000 | -2147483647 to 2147483647 |
| u32 timestamp | 4294967295 / 0xFFFFFFFF | Apply the field-specific rule |

Transmit the corresponding sentinel for NaN, Inf, unavailable source data, invalid source data, or a value outside the representable range after scaling. Do not wrap an out-of-range value or replace it with the largest valid value. Check the range using a sufficiently wide intermediate type before transmission. Do not use an otherwise valid value of zero to represent invalid data.

Exception: SMS preserves the original uint32 millisecond counter. Therefore 0xFFFFFFFF is a valid SMS value and is followed by rollover to zero. The sentinel rules above do not apply to TimeUS, Live_Counter, or the RTCM counters. Follow the individual tables for the use of 255 in GPS NSats and instance fields.

For accuracy fields, check whether the source API actually provides an accuracy value. A zero in a log must not, by itself, be interpreted as zero uncertainty. DOP is dimensionless; HAcc/VAcc are in m, and SAcc is in m/s. Do not interpret SAcc as acceleration in m/s^2.

## 3 MAVLink common specification

| Item | Definition |
|---|---|
| Protocol | MAVLink 2 |
| Number of messages | 2 |
| TC | 50050, GCS to FLCC, one initial request plus at most one retry on failure |
| TM | 51050, FLCC to GCS, 5 Hz / 200 ms |
| Unit N/A | Dimensionless, enumeration value, or counter |
| Generated code | Generate both endpoints from the same custom-dialect XML |

Both message IDs exceed 255 and cannot be transmitted using MAVLink 1. Check for duplicate IDs throughout the active dialect and its includes. This document does not require editing submodules or existing message-definition files; add the messages to the project's custom dialect.

Preserve the capitalization and underscores in all field names exactly as defined below. For fields without units, omit the XML units attribute rather than setting units="N/A"; show N/A in the documentation. Generate CRC_EXTRA from the final XML and compare the generated results at both endpoints. This specification does not assign unverified numeric CRC_EXTRA values.

### 3.1 TC message 50050

Message name: **SYS_ICD_GCS_FLCC_SAR_CTRL**  
Direction: GCS to FLCC  
Transmission: once per operator request, plus at most one GCS retry on failure; no periodic transmission  
Logical payload length: 2 bytes

| Field | Type | Unit | Allowed values | Meaning |
|---|---|---|---|---|
| Observation_Control | uint8_t | N/A | 0, 1, 2 | 0 Prepare, 1 Start observation, 2 Stop observation |
| Observation_Mode | uint8_t | N/A | All values 0 to 255 | Observation-mode selection; detailed mapping TBD |

| Payload offset | Length | Field |
|---|---:|---|
| 0 | 1 | Observation_Control |
| 1 | 1 | Observation_Mode |

Observation_Control values 3 through 255 are undefined and must not be executed. Control = 1 corresponds to the Start Button; Control = 2 corresponds to the Stop Button. No additional device command is defined for Control = 0; Heart_Beat reception determines readiness. All Observation_Mode values 0 through 255 may be received and retained, but do not invent their CAN encoding while 0x503 remains undefined. Start/Stop command payloads contain only byte 0x01, not Mode.

For each allowed TC reception, FC transmits the matching CAN 0x501 or 0x502 once and waits 2 s. FC does not retry on its own timeout. GCS may resend the same TC once; FC forwards that attempt again. SAR commands must be idempotent state assignments: repeated Start keeps Started, repeated Stop keeps Stopped, and retransmission must never toggle On/Off. SAR returns the normal matching response for repeated commands. For each Start, GCS and FC check valid, fresh RTK_FIXED (Status = 6) from the automatically selected Primary GPS. Do not add COMMAND_ACK. Without a transaction ID, perfect discrimination of delayed responses is impossible; use the per-attempt rules in Section 14.

The TC payload has no target_system/target_component fields. The destination FLCC must therefore be selected through a dedicated link or explicit routing configuration. These two fields alone do not provide standard MAVLink targeting for multiple vehicles on the same network. FLCC checks the permitted GCS source sysid/compid and ingress path.

### 3.2 TM message 51050

Message name: **SYS_ICD_FLCC_GCS_SAR_STATUS**  
Direction: FLCC to GCS  
Transmission: **5 Hz**, periodically regardless of connection or observation state  
Logical payload length: 5 bytes

| XML declaration order | Field | Type | Unit | Meaning |
|---:|---|---|---|---|
| 1 | SAR_STATUS | uint8_t | N/A | Actual SAR observation status |
| 2 | Live_Counter | uint32_t | N/A | FC-maintained valid SAR heartbeat reception count |

| SAR_STATUS | Meaning |
|---:|---|
| 0 | Disconnected |
| 1 | Ready |
| 2 | Stopped |
| 3 | Started |
| 4 | SAR Not Responding; FC did not receive the SAR command response within 2 s |
| 5 to 255 | Undefined; do not interpret as a normal state |

**The actual wire order differs from the XML declaration order.** MAVLink reorders base fields by native type size, so the uint32 field comes first.

| Payload offset | Length | Field |
|---|---:|---|
| 0-3 | 4 | Live_Counter, Little-endian |
| 4 | 1 | SAR_STATUS |

MAVLink 2 trailing-zero truncation may shorten the actual wire payload below its logical length. The generated decoder restores omitted trailing bytes as zeros. Do not reject an otherwise valid packet merely because its payload length is not exactly 5. FC increments MAVLink Live_Counter by one per valid CAN 0x510 reception; derive SAR_STATUS from the operating state machine in Section 14. Do not use the earlier proposed CAN 0x40A.

### 3.3 Live_Counter and heartbeat

- Live_Counter is the FC-maintained count of valid Heart_Beat **receptions**. Increment exactly once per valid 0x510, never on 5 Hz TM transmission.
- This is distinct from SAR's uint32 Raw_Counter in 0x510; do not copy its value directly.
- SAR and FC share the drone's **single battery source** and reboot together through a common power reset. Do not implement separate SAR reboot detection or infer reboot from a Raw_Counter decrease.
- Initialize Live_Counter to zero on FC boot. Do not persist/restore it across FC reboot. The first valid heartbeat after common power reset produces one. Independent device reboots are outside this operating assumption.
- Start/Stop, observation termination, heartbeat timeout and reconnection do not reset the count. Hold the last value while disconnected and continue counting on reconnection.
- At 1 Hz the uint32 maximum takes approximately 136 years; no separate terminal-count, wrap or saturation handling is required within the operating lifetime.
- **T_HB = 3,000 ms.** Before first valid 0x510 or at least 3,000 ms after the last valid reception, report SAR_STATUS = 0 and disable navigation transmission. Other SAR data or command responses do not establish initial connection.
- The first heartbeat changes initial state 0 to 1 and triggers an initial-connection notification in the GCS message window. The first heartbeat after link loss triggers a reconnection notification, while the current recovery-pending display remains state 0 (Section 14.8).
- Invalid DLC or other validation failures must not increment the count or refresh heartbeat freshness. Normal heartbeats must not overwrite Started/Stopped/Not Responding with Ready.

### 3.4 MAVLink XML message definitions

Insert the following two message elements inside the messages element of the project's custom dialect. These are insertion snippets, not a complete standalone XML document.

```xml
<message id="50050" name="SYS_ICD_GCS_FLCC_SAR_CTRL">
  <description>SAR observation control from GCS to FLCC. One initial request and at most one GCS retry; repeated commands must not toggle observation.</description>
  <field type="uint8_t" name="Observation_Control">Observation control: 0 Prepare, 1 Start, 2 Stop. Values 3 through 255 are undefined.</field>
  <field type="uint8_t" name="Observation_Mode">Observation mode selection. All values from 0 through 255 are allowed; detailed mapping is TBD.</field>
</message>

<message id="51050" name="SYS_ICD_FLCC_GCS_SAR_STATUS">
  <description>SAR device status from FLCC to GCS, sent periodically at 5 Hz.</description>
  <field type="uint8_t" name="SAR_STATUS">SAR observation status: 0 Disconnected, 1 Ready, 2 Stopped, 3 Started, 4 SAR Not Responding after the FC 2-second response window.</field>
  <field type="uint32_t" name="Live_Counter">FC-maintained valid SAR Heart_Beat reception count. Increment by one per valid CAN 0x510, not per TM transmission. Reset on FC boot during the common single-battery power reset; no persistence or separate SAR reboot detection. No terminal-count handling required in the operating lifetime.</field>
</message>
```

## 4 CAN common specification and data sources

| Item | Definition |
|---|---|
| Specification | Classic CAN 2.0, Standard Data Frame |
| Identifier | 11-bit, IDE = 0 |
| Data frame | RTR = 0; CAN FD is not used |
| Allocated range | 0x400 to 0x7FF, 1,024 IDs in total |
| IDs used in this revision | Navigation 0x400 to 0x408; commands 0x501 to 0x503; Heart_Beat 0x510; responses 0x521 to 0x523 |
| Remaining range | Reserve 0x409 to 0x500, 0x504 to 0x50F, 0x511 to 0x520, and 0x524 to 0x7FF; earlier proposed 0x409/0x40A are inactive |
| CAN bitrate | **1 Mbps = 1,000,000 bit/s** |
| Byte order | Little-endian |
| Application responses | Start 0x501 -> 0x521; Stop 0x502 -> 0x522; no separate MAVLink ACK |

Obtain GPS/GPA from their runtime logging sources. CAN 0x404 ATTITUDE represents actual attitude of the gimbal-stabilized SAR device, rather than copying the FC ATT log (Section 16). Do not transmit DataFlash record bytes directly.

GPS/GPA and both GCS/FC RTK start checks follow the **same automatically selected ArduPilot Primary GPS**. Latch the Primary instance at snapshot acquisition and obtain all GPS/GPA fields and I for that bundle from it. Do not alternate or mix GPS1/GPS2. Apply a changed Primary and I at the next bundle boundary so the receiver can detect the source change. Follow ArduPilot configuration for Primary selection/blending; do not add an independent SAR GPS selector.

The reference object for CAN 0x404 Roll/Pitch/Yaw is the **actual SAR device**. Distinguish these values from FC airframe attitude, GPS.Yaw and gimbal setpoints. Section 16 defines attitude conventions, stabilization conditions and data-source limitations.

## 5 CAN message layout

Multiplication is applied to the physical value at the transmitter; Section 6 defines units and ranges. Gate navigation IDs 0x400 through 0x408 using the confirmed Start/Stop procedure in Section 14. Heart_Beat, commands, and responses are independent of this navigation transmission gate; Start/Stop are nonperiodic.

| CAN ID | Message | Direction | DLC | Payload layout | Transmission condition and rate |
|---|---|---|---:|---|---|
| 0x400 | GPS_POSITION | FLCC to SAR | 8 | B0-3 Lat i32 *10^7, B4-7 Lng i32 *10^7 | **10 Hz**, same snapshot bundle, during active observation |
| 0x401 | GPS_MOTION | FLCC to SAR | 8 | B0-3 Alt i32 *100, B4-5 Spd u16 *100, B6-7 GCrs u16 *100 | **10 Hz**, same snapshot bundle, during active observation |
| 0x402 | GPS_AUX | FLCC to SAR | 8 | B0-1 VZ i16 *100, B2-3 GPS_Yaw u16 *100, B4-5 HDop u16 *100, B6-7 VDop u16 *100 | **10 Hz**, same snapshot bundle, during active observation |
| 0x403 | GPS_ACCURACY | FLCC to SAR | 8 | B0-1 HAcc u16 *100, B2-3 VAcc u16 *100, B4-5 SAcc u16 *100, B6-7 YAcc u16 *100 | **10 Hz**, same snapshot bundle, during active observation |
| 0x404 | ATTITUDE | FLCC to SAR | 8 | B0-1 Roll i16 *100, B2-3 Pitch i16 *100, B4-5 Yaw u16 *100, B6 ATT_Seq u8, B7 ATT_Flags u8 | **10 Hz**, same snapshot bundle, during active observation |
| 0x405 | GPS_TIME_STATUS | FLCC to SAR | 8 | B0-3 GMS u32 *1, B4-5 GWk u16 *1, B6 Status u8, B7 NSats u8 | **10 Hz**, same snapshot bundle, during active observation |
| 0x406 | GPS_SAMPLE_INFO | FLCC to SAR | 8 | B0-3 SMS u32 *1, B4-5 Delta u16 *1, B6 I u8, B7 GPS_Flags u8 | **10 Hz**, same snapshot bundle, during active observation |
| 0x407 | GPS_DIAGNOSTICS | FLCC to SAR | 8 | B0-3 AEl i32 *100, B4-5 RTCMFU u16 *1, B6-7 RTCMFD u16 *1 | **10 Hz**, same snapshot bundle, during active observation |
| 0x408 | SNAPSHOT_TIME_US | FLCC to SAR | 8 | B0-7 Snapshot_TimeUS u64 *1 | **10 Hz**, same snapshot bundle, during active observation |
| 0x501 | Observation_Start_Command | FLCC to SAR | 1 | B0 Command u8 = **1 (0x01)**; no other values | Once for TC Control = 1 after the RTK Fixed start interlock passes |
| 0x502 | Observation_Stop_Command | FLCC to SAR | 1 | B0 Command u8 = **1 (0x01)**; no other values | Once for TC Control = 2, independent of RTK status |
| 0x503 | Observation_Mode_Command | FLCC to SAR | 8 | B0-7 **manufacturer-reserved; fields, types, and semantics TBD** | Do not transmit until payload is defined |
| 0x510 | Heart_Beat | SAR to FLCC | 4 | B0-3 Raw_Counter u32 *1, increasing from 0, least-significant byte first **(Keep Alive)** | **1 Hz, periodic** |
| 0x521 | Observation_Start_Response | SAR to FLCC | 1 | B0 Response u8 = **1 (0x01)**; no other values | Response to 0x501 |
| 0x522 | Observation_Stop_Response | SAR to FLCC | 1 | B0 Response u8 = **1 (0x01)**; no other values | Response to 0x502 |
| 0x523 | Observation_Param_Response | SAR to FLCC | 8 | B0-7 **manufacturer-reserved; fields, types, and semantics TBD** | Response trigger TBD |

There are **16 unique IDs**: 9 periodic navigation/diagnostic/timestamp messages (0x400-0x408), 3 device commands and 4 heartbeat/response messages. AEl, RTCMFU/RTCMFD and snapshots are required. Message 0x407 is a required member of the 10 Hz bundle; the previous nonperiodic-diagnostic exception and unresolved event trigger are superseded. Expose only a function interface for 0x503; do not transmit it.

The previously proposed 0x409 SAR_CTRL and 0x40A SAR_STATUS are superseded by these command/response/heartbeat procedures and excluded from the operating profile. Do not use them alongside the new mapping. Navigation and diagnostic transmission are both 10 Hz; both are limited to enabled observation transmission. MAVLink TM remains at 5 Hz regardless of observation state.


## 6 Detailed CAN field definitions

### 6.1 Position and altitude

| Field | Source | Type | Input unit | multiplication | Transmission resolution | Valid physical range |
|---|---|---|---|---:|---|---|
| Lat | GPS.Lat | i32 | deg | 10,000,000 | 0.0000001 deg | -90 to 90 |
| Lng | GPS.Lng | i32 | deg | 10,000,000 | 0.0000001 deg | -180 to 180 |
| Alt | GPS.Alt | i32 | m | 100 | 0.01 m | -21,474,836.47 to 21,474,836.47 m |
| AEl | GPA.AEl | i32 | m | 100 | 0.01 m | Same i32 range as Alt |

Lat/Lng are WGS84 latitude/longitude. Alt is GPS altitude above MSL/AMSL, not home-relative altitude or EKF-fused altitude. AEl is height above the WGS84 ellipsoid; transmit INT32_MIN if the source does not provide it. Do not copy Alt into an unavailable AEl field and label it ellipsoid height.

### 6.2 Speeds and angles

| Field | Source | Type | Input unit | multiplication | Resolution | Valid range |
|---|---|---|---|---:|---|---|
| Spd | GPS.Spd | u16 | m/s | 100 | 0.01 m/s | 0 to 655.34 m/s |
| GCrs | GPS.GCrs | u16 | deg | 100 | 0.01 deg | 0 to 359.99 deg |
| VZ | GPS.VZ | i16 | m/s | 100 | 0.01 m/s | -327.67 to 327.67 m/s |
| GPS_Yaw | GPS.Yaw | u16 | deg | 100 | 0.01 deg | 0 to 359.99 deg |
| Roll | SAR_Attitude.Roll | i16 | deg | 100 | 0.01 deg | -180 to 180 deg |
| Pitch | SAR_Attitude.Pitch | i16 | deg | 100 | 0.01 deg | -90 to 90 deg |
| Yaw | SAR_Attitude.Yaw | u16 | deg | 100 | 0.01 deg | 0 to 359.99 deg |

Spd is horizontal ground speed, sqrt(V_N^2 + V_E^2). VZ is the NED D component: **positive downward and negative upward**. GCrs is ground course, not vehicle yaw. Do not use GCrs as vehicle heading when stationary or at low speed.

Normalize GCrs/GPS_Yaw/SAR_Attitude.Yaw to [0, 360) deg before rounding. After rounding, replace integer 36000 with zero so that, for example, 359.999 deg does not remain a separate 360.00 deg representation. Normalize Roll/Pitch to the standard Euler ranges for actual attitude.

Check the source API's validity and data age for GPS yaw and its accuracy. Some logs may record an unavailable yaw as zero, so the numeric value zero alone does not determine validity. In this ICD, transmit 0xFFFF for an unavailable value. If the API substitutes a default accuracy, distinguish that fallback from accuracy actually reported by the sensor when implementing YAcc.

### 6.3 DOP and accuracy

| Field | Source | Type | Input unit | multiplication | Resolution | Valid range |
|---|---|---|---|---:|---|---|
| HDop | GPS.HDop | u16 | Dimensionless | 100 | 0.01 | 0 to 655.34 |
| VDop | GPA.VDop | u16 | Dimensionless | 100 | 0.01 | 0 to 655.34 |
| HAcc | GPA.HAcc | u16 | m | 100 | 0.01 m | 0 to 655.34 m |
| VAcc | GPA.VAcc | u16 | m | 100 | 0.01 m | 0 to 655.34 m |
| SAcc | GPA.SAcc | u16 | m/s | 100 | 0.01 m/s | 0 to 655.34 m/s |
| YAcc | GPA.YAcc | u16 | deg | 100 | 0.01 deg | 0 to 655.34 deg |

Transmit 0xFFFF for an out-of-range value rather than saturating to a valid value. For Network RTK operation, **1 cm position/accuracy, 1 cm/s speed and 0.01 deg angular resolution are accepted**. Retain the existing 10^-7 deg latitude/longitude encoding. No millimeter-resolution extension is required.

### 6.4 Time and status

| Field | Source or meaning | Type | multiplication | Definition |
|---|---|---|---:|---|
| GMS | GPS.GMS | u32 | 1 | Milliseconds within the GPS week; valid 0 to 604,799,999; unknown 0xFFFFFFFF |
| GWk | GPS.GWk | u16 | 1 | GPS week number; unknown 0xFFFF |
| Status | GPS.Status | u8 | 1 | ArduPilot GPS fix-type enumeration |
| NSats | GPS.NSats | u8 | 1 | Satellite count from the source API; unknown 255; zero is a valid representation |
| SMS | GPA.SMS | u32 | 1 | FC reception time of the most recent GPS message, in ms since FC boot |
| Delta | GPA.Delta | u16 | 1 | Interval between reception of the two most recent GPS messages, in ms; unknown/out of range 0xFFFF |
| I | GPS.I / GPA.I | u8 | 1 | Selected GPS instance number; unknown 255 |
| RTCMFU | GPA.RTCMFU | u16 | 1 | RTCM fragments-used counter supplied by the source API |
| RTCMFD | GPA.RTCMFD | u16 | 1 | RTCM fragments-discarded counter supplied by the source API |
| Snapshot_TimeUS | Required time axis | u64 | 1 | 10 Hz navigation snapshot acquisition time, us since FC boot |
| ATT_Seq | Interface-added field | u8 | 1 | ATTITUDE transmission sample sequence; wraps over 0 to 255 |

The GPS week uses the GPS timescale. Converting it to UTC requires separate leap-second handling. SMS is not the actual GNSS measurement epoch; the source logging code uses the FC reception time represented by last_message_time_ms. Do not use SMS as a precise synchronized timestamp corrected for measurement latency.

Snapshot_TimeUS uses the **same microseconds-since-FC-boot time axis as the basic FC log TimeUS**. Here it timestamps acquisition of the 10 Hz bundle, not each separate log record or the actual GPS measurement epoch. Preserve GPS source times separately in SMS and GMS/GWk. Depending on the receiving-module implementation, the RTCM counters may also represent shared statistics rather than statistics exclusive to the selected GPS. Preserve the source uint16 counters' rollover/reset behavior; the no-reset requirement does not apply to these two counters.

| GPS Status | Meaning |
|---:|---|
| 0 | NO_GPS |
| 1 | NO_FIX |
| 2 | FIX_2D |
| 3 | FIX_3D |
| 4 | DGPS |
| 5 | RTK_FLOAT |
| 6 | RTK_FIXED |
| 7 to 255 | Outside this operating profile; do not interpret as a normal observation status |

This status uses the project's ArduPilot GPS status values. Do not directly interchange these values with fix-status enumerations from other protocols. Confirm them against the actual firmware enumeration.

### 6.5 Bitfields

GPS_Flags is located at B7 of 0x406.

| Bit | Name | Meaning when set to 1 |
|---:|---|---|
| 0 | GPS_USED | GPS.U: the selected GPS is the AP_GPS primary |
| 1 | VERTICAL_VELOCITY_AVAILABLE | GPA.VV: vertical velocity is available |
| 2 | GPS_FRESH | Source GPS data is within its permitted age; source age must be no more than **300 ms** |
| 3 | GPS_TIME_VALID | GMS/GWk are valid |
| 4 | ENCODING_ERROR | The current GPS group contains a physical-range or scaled-range error |
| 5 | AEL_VALID | AEl is actually available; applies to 0x407 in the same 10 Hz bundle |
| 6 | RTCM_STATS_AVAILABLE | The API provides RTCM fragment statistics |
| 7 | RESERVED | Transmit zero; ignore on reception |

GPS_USED does not guarantee that the FC's final EKF is actually fusing this GPS. ENCODING_ERROR denotes a range error, not merely an unavailable value; individual invalid fields are identified by their sentinels. For unavailable statistics, transmit RTCM_STATS_AVAILABLE = 0 and zero counters. Use this bit to distinguish unavailable statistics from genuinely zero counts.

ATT_Flags is located at B7 of 0x404.

| Bit | Name | Meaning when set to 1 |
|---:|---|---|
| 0 | ROLL_VALID | Roll is valid |
| 1 | PITCH_VALID | Pitch is valid |
| 2 | YAW_VALID | Yaw is valid |
| 3 | ATT_FRESH | Attitude data is within its permitted age; source age must be no more than **40 ms** |
| 4-7 | RESERVED | Transmit zero; ignore on reception |

The receiver checks validity bits and sentinels; contradictions are invalid. Finite numbers or setpoints alone do not establish validity. For 0x404, valid/fresh applies to the actual SAR attitude acquisition/estimate; valid FC AHRS alone does not establish valid SAR attitude. Follow Section 16 for the data-source contract.

## 7 Navigation snapshots and transmission rules

### 7.1 Accepted packing and data source

Retain Section 6 integer types, multiplication, sentinels and ranges; current resolution is accepted. Follow automatically selected Primary GPS and latch its instance for each snapshot. Do not mix instances inside a bundle when Primary changes.

### 7.2 Source validity and transmission rates

- During enabled observation, transmit 0x400-0x408 as a bundle at **10 Hz, every 100 ms**. ATT 0x404 is also 10 Hz.
- GPS measurement updates are **5 Hz or 10 Hz**. At 5 Hz, consecutive bundles may reuse the same source GPS sample; preserve SMS/GMS/GWk rather than relabeling it as a new measurement. Each bundle has a new Snapshot_TimeUS acquisition timestamp.
- Permit only GPS source age **at most 300 ms** and ATT source age **at most 40 ms**. Check at acquisition and again before delayed transmission if necessary.
- The ATT 40 ms limit applies to the actual SAR attitude source/estimate in Section 16. If FC AHRS and gimbal state are combined, check update times of all required inputs. Repackaging does not refresh stale source data. Do not apply a 40 ms receive timeout to the 100 ms CAN transmission interval.
- Diagnostics 0x407 (AEl/RTCMFU/RTCMFD) are also **10 Hz** and belong to the same snapshot bundle. Start/Stop remain nonperiodic requests.
- A source-age violation or FC navigation/EKF failsafe immediately invokes automatic observation termination. The earlier rule allowing observation to continue with 10 Hz invalid-data updates is superseded.

### 7.3 Same-acquisition-time bundle and Snapshot_TimeUS

Every 100 ms at time S, acquire a consistent runtime snapshot of Primary GPS/GPA and valid current SAR attitude. Same-time means the latest available source values acquired at S, not simultaneous GPS and IMU measurement epochs at S. Do not invent interpolated or new measurements.

Transmit the complete ordered bundle 0x400 -> 0x401 -> 0x402 -> 0x403 -> 0x404 -> 0x405 -> 0x406 -> 0x407 -> **0x408(S)**. Do not overlap or interleave navigation bundles. Commands/heartbeat are handled separately while preserving navigation bundle order. Diagnostics 0x407 are required bundle members acquired at the same time S.

SAR collects freshly received periodic IDs into a new buffer after the previous bundle boundary and applies the closing 0x408 timestamp. Do not assemble arbitrary cached latest values. Discard a bundle with missing IDs, duplicates or order errors; never fill gaps from an older bundle. Resynchronize at a valid subsequent boundary. Do not publish partial bundles as complete data.

**One logical snapshot consists of 9 CAN frames and 72 payload bytes.** Classic CAN allows 8 bytes per frame, so this is not one physical CAN frame. Message 0x408 is a closing timestamp, not an envelope containing all data.

No common sample ID is present in every frame, so total bundle loss or loss of boundaries cannot be perfectly verified. This is the requested logical bundle, not a guarantee of lossless or measurement-epoch synchronization. Discard unverifiable boundaries and resynchronize. Do not invent additional sequence/CRC wire fields in this revision.

### 7.4 Transmission ON and OFF

Set NAV_TX_ENABLED true after confirmed normal Start and false after confirmed normal Stop. On heartbeat timeout, degradation to ordinary 3D_FIX or lower, source-age violation, or navigation/EKF failsafe, disable it immediately without waiting for a response. Stop generating/enqueuing data and discard unsent bundles. Frames already on the bus cannot be canceled. Continue FC GPS/EKF processing and onboard flight logging.

## 8 MAVLink and CAN mapping

| Input or event | FC handling | Output or transition |
|---|---|---|
| TC 50050 Control = 0 | Receive Prepare without additional device transmission | Do not invent a state change |
| TC Control = 1 | Check connection, recovery inhibit and fresh Primary RTK Fixed | If allowed, 0x501/01, START_PENDING, 2 s window |
| TC Control = 2 | Handle Stop independently of RTK | 0x502/01, STOP_PENDING, 2 s window |
| GCS repeats the same TC | Forward the same state-setting command once more | No SAR toggle; return matching response |
| Valid 0x510 | Live_Counter += 1; refresh connection time | Ready on first connection; current count in TM |
| Valid matching 0x521/01 | Confirm Start | State 3; navigation ON |
| Valid matching 0x522/01 | Confirm Stop | State 2; navigation OFF |
| FC response window expires at 2 s | Latch failure for the attempt | State 4 in next 5 Hz TM |
| Heartbeat timeout at 3 s | Disconnect; inhibit recovery | State 0; navigation OFF |
| Degradation to Float (5) | Continue while source data is valid; track continuous duration | Section 15 defines post-processing for below/at least 10 s |
| 3D_FIX (3) or lower, source-age violation, navigation failsafe | Automatic termination | Navigation OFF immediately; 0x502/01 if possible |
| Observation_Mode / reserved 0x503 and 0x523 | Keep function interfaces only | No actual transmission or invented parsing |

TM remains on a 200 ms schedule. State 4 means no command response despite an otherwise live connection, distinct from disconnected state 0. Priority is initial disconnection, heartbeat loss or recovery-pending indication 0 > current-attempt response timeout 4 > last confirmed state 1/2/3 (Section 14.8). State 4 alone does not establish that SAR physically stopped.

## 9 Implementation verification examples

### 9.1 CAN scaling and byte order

| ID | Test physical values | Expected CAN data starting at B0 |
|---|---|---|
| 0x400 | Lat 35.0 deg, Lng 129.0 deg | 80 93 DC 14 80 D6 E3 4C |
| 0x401 | Alt 100.0 m, Spd 5.0 m/s, GCrs 90.0 deg | 10 27 00 00 F4 01 28 23 |
| 0x402 | VZ -0.5 m/s, GPS_Yaw 50.0 deg, HDop 0.8, VDop 1.2 | CE FF 88 13 50 00 78 00 |
| 0x403 | HAcc 0.02 m, VAcc 0.03 m, SAcc 0.04 m/s, YAcc 0.05 deg | 02 00 03 00 04 00 05 00 |
| 0x404 | Roll -1.23 deg, Pitch 2.34 deg, Yaw 50.0 deg, ATT_Seq 1, Flags 0x0F | 85 FF EA 00 88 13 01 0F |

The 0x400 example transmits the scaled integers 350,000,000 and 1,290,000,000. Do not multiply these integers by 10^7 again. All test values are synthetic encoding examples.

### 9.2 MAVLink logical payload

| Message ID | Input fields | Serialized logical payload |
|---|---|---|
| 50050 | Observation_Control 1, Observation_Mode 255 | 01 FF |
| 51050 | SAR_STATUS 3, Live_Counter 0x12345678 | 78 56 34 12 03 |

These are not complete MAVLink packets. The MAVLink library handles the header, CRC, optional signing, and trailing-zero truncation.

Verify negative latitude/longitude, rounding of -0.005, angle wrap at 359.999 deg, unavailable accuracy, range errors, GPS disconnection, common-power reboot and FC counter initialization, Primary changes, CAN loss, and Section 14 per-attempt timeout/retry behavior. No terminal-count handling is required in the operating lifetime; do not retain it as an undecided operating policy.

## 10 Accepted operating decisions and remaining implementation checks

| Decision | Fixed value |
|---|---|
| CAN bitrate | 1 Mbps |
| Heartbeat timeout | 3 s |
| FC-to-SAR Start/Stop response window | 2 s each |
| GCS window per attempt | 3 s each; react immediately to early state 4 |
| Retry | GCS at most once, two attempts total; no SAR toggle |
| Failure reporting | State 4; retry notice; final SAR not responding to Start or Stop |
| Live_Counter | FC +1 per valid heartbeat; zero on FC boot; common single-battery reset, no persistence/restoration or separate reboot detection |
| GPS source and age | Automatic Primary; at most 300 ms |
| ATT source age | Actual SAR attitude source/estimate at most 40 ms; not a direct copy of FC airframe attitude |
| Rates | Navigation/diagnostics/snapshot 0x400-0x408 all 10 Hz; GPS source 5/10 Hz; Start/Stop nonperiodic |
| AEl/RTCM and snapshot | All in the same 10 Hz bundle; TimeUS is us since FC boot |
| Resolution | Existing integer scales accepted: cm, cm/s, 0.01 deg |
| RTK degradation during observation | Float <10 s requires review; Float >=10 s requires an earlier valid prefix or reacquisition; ordinary 3D or lower aborts; see Section 15 |
| Disconnect and recovery | Navigation OFF/state 0; show reconnection notification but currently retain Disconnected; detailed recovery inhibition definition/implementation deferred |
| Mode and Param | Function interfaces only; no actual exchange |

Remaining checks concern implementation boundaries, not reopening the accepted operating values:

1. Manufacturer payloads for 0x503/0x523 are undefined; actual exchanges remain deferred.
2. Detailed recovery inhibition definition/implementation, inspection completion/release procedure and dedicated GCS reporting are deferred. Currently retain Disconnected while recovery is pending.
3. Confirm in-observation DGPS (4) handling, post-processing ownership/interval storage in Section 15, and actual SAR attitude acquisition plus behavior outside the stabilization envelope in Section 16 before implementation.
4. Confirm the target ArduPilot commit, source timestamp/failsafe APIs and GCS Primary identification during implementation. No arbitrary submodule changes are required.
5. Absence of transaction IDs and per-frame common sequences remains a limitation; do not add wire fields.

## 11 References and applicability

Initial GPS/GPA mappings and serialization rules use the public references below. SAR attitude semantics for CAN 0x404 and Sections 15-16 are project requirements; the default ArduPilot ATT log definition does not establish that behavior. Verify actual APIs and field availability in the target 4.7.1-based custom firmware during implementation. Apply unavailable-value rules if AEl/RTCM statistics are absent.

1. ArduPilot GPS/GPA log field definitions  
   https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_GPS/LogStructure.h
2. Official ArduPilot GPS logging implementation and value-acquisition paths  
   https://firmware.ardupilot.org/coverage/AP_GPS/AP_GPS.cpp.gcov.html
3. ArduPilot ATT log definitions  
   https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_AHRS/LogStructure.h
4. MAVLink serialization, field reordering, and trailing-zero truncation  
   https://mavlink.io/en/guide/serialization.html
5. MAVLink custom-message XML rules  
   https://mavlink.io/en/guide/define_xml_element.html


## 12 Device messages from SAR to FLCC

Both language versions are maintained at Rev1.0. All messages use 11-bit Standard Frames and Section 2.2 Little-endian byte order with bit 0 as the LSB.

### 12.1 Heart_Beat

| Item | Definition |
|---|---|
| Direction | SAR to FC |
| Msg ID | 0x510 |
| Message name | Heart_Beat |
| DLC | 4 |
| Payload layout | B0-3 Raw_Counter uint32_t, multiplication = 1, increasing from zero **(Keep Alive)** |
| Rate | 1 Hz, periodic |
| Byte mapping | B0 bits 7-0, B1 bits 15-8, B2 bits 23-16, B3 bits 31-24 |

Raw_Counter retains the existing four-byte wire field. FC Live_Counter increments per valid reception and initializes to zero on FC boot. Use the common single-battery power reset, without separate reboot detection or counter persistence. Follow Sections 3.3 and 14 for initial/reconnection notifications and state 0 while recovery is pending.

### 12.2 Observation_Start_Response

| Item | Definition |
|---|---|
| Direction | SAR to FC |
| Msg ID | 0x521 |
| Message name | Observation_Start_Response |
| DLC | 1 |
| Payload layout | B0 Response uint8_t = 1 (0x01); no other values |
| Matching command | 0x501 Observation_Start_Command |

A valid matching response received while START_PENDING **within 2 s of that Start transmission** confirms observation Start. Set SAR_STATUS = 3 and enable navigation transmission for 0x400 through 0x408. An incorrect payload or DLC must not confirm Start.

### 12.3 Observation_Stop_Response

| Item | Definition |
|---|---|
| Direction | SAR to FC |
| Msg ID | 0x522 |
| Message name | Observation_Stop_Response |
| DLC | 1 |
| Payload layout | B0 Response uint8_t = 1 (0x01); no other values |
| Matching command | 0x502 Observation_Stop_Command |

A valid matching response received while STOP_PENDING **within 2 s of that Stop transmission** confirms observation Stop. Set SAR_STATUS = 2 and disable navigation transmission for 0x400 through 0x408. An incorrect payload or DLC must not confirm Stop.

### 12.4 Observation_Param_Response

| Item | Definition |
|---|---|
| Direction | SAR to FC |
| Msg ID | **0x523** |
| Message name | Observation_Param_Response |
| DLC | 8 |
| Payload layout | B0-7 manufacturer-reserved; fields, types, units, scaling, and values TBD |
| Transmission condition | TBD |

Changing Param Response from 0x521 to 0x523 resolves the duplicate allocation. Classify Start/Stop/Param Response by ID, then validate DLC. Param Response remains eight opaque reserved bytes; do not invent uint64, float, or status interpretations. Do not automatically connect it to undefined Mode exchanges or state transitions.

## 13 Commands from FC to SAR

| Msg ID | Message name | Direction | DLC | Payload layout |
|---|---|---|---:|---|
| 0x501 | Observation_Start_Command | FC to SAR | 1 | B0 Command uint8_t = 1 (0x01); no other values |
| 0x502 | Observation_Stop_Command | FC to SAR | 1 | B0 Command uint8_t = 1 (0x01); no other values |
| 0x503 | Observation_Mode_Command | FC to SAR | 8 | B0-7 manufacturer-reserved; all fields TBD |

Both 0x501 and 0x502 transmit exactly one byte 01. Repeated forwarding on the one allowed GCS retry must preserve the target SAR state and return the same response; On/Off toggling is prohibited. Do not encode the Stop payload as 02 merely because it is Stop. Bit 0 of byte 0 is the LSB. Create function interfaces/stubs only for 0x503 and 0x523; no actual CAN transmission, invented payload or parsing. Since 0x503 has no defined serialization yet, do not place Mode at B0 or send eight zero bytes. Retain MAVLink Observation_Mode if needed, but do not actually transmit Mode Command until the manufacturer defines the payload.

## 14 Operating procedures and command-attempt management

### 14.1 States and fixed timers

| Item | Definition |
|---|---|
| SAR_STATUS = 0 | Disconnected; before first heartbeat, 3 s timeout, or the current recovery-pending indication |
| SAR_STATUS = 1 | Ready; initial connection confirmed by first heartbeat, before observation |
| SAR_STATUS = 2 | Stopped; matching Stop response confirmed |
| SAR_STATUS = 3 | Started; matching Start response confirmed |
| SAR_STATUS = 4 | SAR Not Responding; FC 2 s command-response window expired |
| FC Start/Stop window | **2,000 ms** after each CAN transmission; no independent FC retry |
| GCS Start/Stop window | **3,000 ms** after each TC; early state 4 fails that attempt immediately |
| GCS attempt limit | One initial transmission plus one retry = two attempts total |
| TM | 51050, always 5 Hz / 200 ms |
| NAV_TX_ENABLED | Navigation transmission gate; false at boot |
| RECOVERY_INHIBIT | Detailed feature, release procedure and implementation deferred; currently retain state 0/navigation OFF while recovery is pending |
| LAST_CONFIRMED_STATE | Last normally confirmed state, retained separately from state 4 |

START_PENDING and STOP_PENDING are internal states, not MAVLink enum additions. Heartbeat must not overwrite states 2/3/4 with Ready. Disconnected 0 takes precedence over no-response 4. Lack of a response does not establish SAR physical On/Off state.

### 14.2 Preparation and heartbeat

1. FC and SAR boot together through the common single-battery power reset. FC initializes Live_Counter = 0, navigation OFF and TM 51050 state 0 at 5 Hz. Remain Disconnected until the first valid 0x510. Initial waiting is not a prior-link failure or a reconnection.
2. SAR to FC: 0x510, DLC 4, 1 Hz, (Keep Alive). FC increments the counter and refreshes reception time on each valid reception.
3. The first heartbeat after boot changes initial state 0 to 1. Report Ready/count in the next TM and show one **initial-connection notification** in the GCS message window.
4. Once a connection has existed, at least 3,000 ms without valid heartbeat results in state 0, navigation OFF and cancellation of pending commands.
5. The first valid heartbeat after that loss shows one **reconnection notification** in the GCS message window. Continue counting and refreshing reception time, but currently retain state 0/navigation OFF while recovery is pending. Do not repeat notifications on every 1 Hz heartbeat.
6. PNU Drone uses the existing MAVLink STATUSTEXT path to display FC connection events in the GCS message window. Example text is `SAR connected` / `SAR reconnected`; PNU Drone owns final wording. This adds no custom message. Because reconnect currently retains state 0, do not infer its notification solely from a TM state change.

### 14.3 Start and RTK Fixed interlock

1. GCS enables Start Button only with valid, fresh automatic Primary **RTK_FIXED = 6**, live connection and no recovery inhibit. Float, ordinary 3D or GPS age above 300 ms disables it.
2. Start Button sends MAVLink 50050, Observation_Control = 1; GCS records Start attempt 1 and its 3 s deadline.
3. Before CAN transmission, FC rechecks the same Primary RTK Fixed, GPS age <=300 ms, ATT age <=40 ms, navigation health, connection and the current recovery-pending condition (detailed recovery functionality is deferred). If rejected, send no 0x501. RTK rejection is not SAR nonresponse and must not create state 4.
4. If allowed, FC to SAR: 0x501, DLC 1, payload 01; set START_PENDING and a 2 s deadline. Keep navigation OFF.
5. On valid matching 0x521/01 within the deadline, set state 3, LAST_CONFIRMED_STATE = 3, clear pending and enable NAV_TX_ENABLED.
6. GCS completes the attempt on a post-request TM with target state 3; FC starts the 10 Hz snapshot bundles.
7. Also check Status = 6 in the first transmitted snapshot. Post-processing rejects the complete observation session if its first snapshot is not 6 or the initial status cannot be verified (Section 15).

The GCS retry must not bypass Start conditions. If GPS quality now prohibits a new Start, show Start blocked rather than forcibly retrying. Repeating Start to already Started SAR must not restart or toggle it; SAR returns 0x521/01 again.

### 14.4 Normal Stop

1. Stop Button sends 50050 Control = 2; GCS records the Stop attempt and 3 s deadline.
2. FC sends 0x502, DLC 1, payload 01 and starts STOP_PENDING with a 2 s deadline. **Do not block Stop based on RTK Fixed status.**
3. For normal user Stop while observing, keep existing navigation ON while awaiting the response.
4. Matching 0x522/01 within the 2 s window sets state 2, LAST_CONFIRMED_STATE = 2, disables NAV_TX_ENABLED and discards unsent bundles.
5. GCS completes on TM target state 2; failure uses the one-retry rules below.

Repeated Stop must not toggle SAR back On and must return 0x522/01. Frames already on the bus cannot be canceled. Continue internal GPS/EKF processing and FC file logging.

### 14.5 FC 2 s timeout and state 4

Open an FC 2 s response window only for a Start/Stop actually transmitted to SAR. If the matching response is absent by the deadline, close pending and latch SAR_STATUS = 4 for the next 5 Hz TM. Do not retroactively count an expired response as success.

Keep navigation OFF on failed Start. For failed normal user Stop, retain the prior navigation state because stopping is unconfirmed. Navigation-fault aborts and heartbeat disconnection independently force data OFF immediately.

When GCS retries the TC, FC forwards the command once and opens a new 2 s window. Clear the preceding attempt's state-4 latch and publish LAST_CONFIRMED_STATE while waiting; publish the target on success or 4 on another timeout. Heartbeat alone does not clear 4. An independent FC CAN retry loop would exceed the GCS one-retry limit and is prohibited.

### 14.6 GCS 3 s windows and one retry

Each GCS operation tracks the target (Start = 3, Stop = 2), transmission time, attempt number and retry consumption. Success means **reaching the requested target state**, not an arbitrary SAR_STATUS change. On first failure, resend the identical TC exactly once and notify the operator of the retry attempt.

| Event | First attempt | Second attempt |
|---|---|---|
| TM reaches target state | Success; finish | Success; finish |
| New state 4 for this attempt | Retry immediately without waiting 3 s | Report final failure immediately |
| 3 s without target or a new state 4 | Show retry notice and resend TC once | Report final failure |

The final error text is **SAR not responding to Start** or **SAR not responding to Stop**. Without earlier state 4, retry at 3 s and fail after another 3 s, at 6 s total. Earlier FC state 4 can advance both retry and final failure; do not always wait until 6 s.

Because state 4 repeats at 5 Hz, repeated reports of the first attempt's 4 must not consume the second attempt. Consume state 4 once per attempt. After retry, observe a new FC TM clearing 4 before interpreting a subsequent transition back to 4 as the second timeout. If that clear transition is not observed, do not fail solely on repeated 4; use the second 3 s deadline. The protocol lacks a transaction ID to perfectly classify delayed/duplicate TM and CAN responses.

Distinguish state 0, recovery inhibit and clear local RTK rejection from SAR response delay. Do not force commands while disconnected or Start is prohibited. A state 4 from an FC-initiated automatic Stop must not be interpreted as a new GCS button transaction and retried without an existing GCS operation.

### 14.7 Automatic observation termination and RTK degradation

| In-observation event | Handling |
|---|---|
| RTK_FIXED (6) to RTK_FLOAT (5), valid source data | Continue observation/navigation transmission |
| Ordinary 3D_FIX (3) or lower | Automatically terminate observation |
| GPS source age >300 ms | Automatically terminate observation |
| ATT source age >40 ms | Automatically terminate observation |
| FC navigation/EKF failsafe | Terminate immediately; do not add a wait for age thresholds |
| No heartbeat for 3 s | State 0, navigation OFF, disconnect handling |

Ordinary 3D_FIX is GPS Status 3, distinct from RTK_FIXED 6. Section 15 defines initial/post-processing quality rules. The Float 10 s threshold is a post-processing rule and adds no FC automatic Stop condition. GNSS status alone does not prove the precise cause of an NTRIP transport failure. In-observation DGPS (4) handling remains TBD.

For automatic termination, **disable navigation immediately** and discard unsent bundles to avoid contaminated data. If heartbeat remains live, send 0x502/01 to request physical SAR Stop and wait 2 s for 0x522. Record response/timeout as internal results; the current recovery-pending indication is state 0 and data remains OFF (Section 14.8). Distinguish this from states 2/4 for normal user Stop. Do not assume FC actually stopped SAR hardware across a broken connection. SAR handling must not replace or delay the FC's own flight failsafe actions.

### 14.8 Disconnect recovery and data contamination inhibition — deferred

Detailed recovery inhibition definition/implementation, inspection completion criteria, release commands/APIs and dedicated GCS reporting are **deferred**. Do not invent recovery parameters or commands from this specification.

Currently retain **SAR_STATUS = 0 (Disconnected), navigation OFF** while recovery is pending after link loss or navigation-fault termination. A reconnecting heartbeat must not automatically restore Ready, request Start or resume navigation. Distinguish this current display/transmission policy from the future detailed recovery inhibition feature. Internal state representation is an implementation choice; this document does not require a completed RECOVERY_INHIBIT feature.

Still show the reconnection event in the GCS message window per Section 14.2 and continue valid-heartbeat reception counting. Physical link freshness and permission to operate can therefore differ. Recovery-pending state 0 takes precedence over normal command confirmation states 2/3 and no-response state 4; retain automatic-Stop response/timeout as internal results. Earlier normal Start/Stop transitions apply when recovery is not pending.

After a common battery power reset, follow new-boot behavior with counter = 0. Do not interpret this as a separate inspection approval signal. Initial heartbeat waiting after boot is not recovery pending, so the first connection can transition to Ready.

### 14.9 Message flows and transmission gate

| Flow | Result |
|---|---|
| 0x510 at 1 Hz -> FC reception count +1 -> 51050 at 5 Hz | Initial Ready; count reporting |
| 50050(Control=1) -> RTK/age checks -> 0x501/01 -> 0x521/01 | State 3; navigation ON at 10 Hz |
| 50050(Control=2) -> 0x502/01 -> 0x522/01 | State 2; navigation OFF |
| No FC response for 2 s -> state 4 -> one GCS TC retry | Re-request same target; no toggle |
| No GCS result per 3 s window -> retry/final failure | Two attempts maximum; default final failure at 6 s |
| 3D/age/failsafe abort -> data OFF -> 0x502/0x522 if possible | Stop contaminated-data transmission regardless of response |
| 0x503/0x523 function call | Deferred stub; no actual CAN exchange |

### 14.10 Verification scenarios

Verify first/second-attempt success/timeout, early state 4, duplicate-4 consumption, 3 s/6 s timing, idempotent repeated SAR commands, 5 Hz GPS source reuse at 10 Hz, 40 ms source ATT versus 100 ms CAN transmission, Primary switching, Float continuation, ordinary-3D abort, heartbeat loss, failsafe-priority termination, one-time initial/reconnection notifications, state 0 retention on reconnect and common-power reboot counter initialization. Reject incomplete/duplicate/out-of-order 0x400 bundles as complete samples.

## 15 GPS Status data quality and post-processing

### 15.1 Decision field and observation start

Use **CAN 0x405 GPS_TIME_STATUS, B6 Status, uint8, multiplication = 1**. Preserve the automatically selected Primary GPS status with the same snapshot and record it with 0x408 Snapshot_TimeUS. MAVLink 51050 SAR_STATUS describes device operation and must not substitute for GPS Status.

An observation session is the acquisition unit following a valid Start response. It must start in **RTK_FIXED = 6**. In addition to the GCS button and FC pre-command checks, check the first complete transmitted snapshot. If its initial Status is not 6, **discard all data from that session**. A missing or unverifiable initial status cannot establish a valid session. Cropping the session to a later Fixed interval does not satisfy the start requirement.

### 15.2 In-observation quality decisions

| Condition | Data use and post-processing |
|---|---|
| Initial Status != 6 or initial status unverifiable | Discard the complete session; reacquire under correct conditions |
| RTK_FIXED (6) maintained | Usable if other validity requirements are met |
| Continuous RTK_FLOAT (5) duration <10 s | Flag the interval for post-processing review; do not automatically approve its quality |
| Continuous RTK_FLOAT (5) duration >=10 s | Use only valid data preceding the start of that Float interval, or reacquire data |
| Degradation to ordinary FIX_3D (3) or lower | Use only valid data preceding degradation, or reacquire; existing FC automatic termination also applies |

Exactly 10 s belongs to the second Float category (>=10 s). Short Float intervals also require review before approval. Post-processing determines whether the preceding usable data has sufficient length/continuity for SAR processing; otherwise reacquire. A later return to Fixed does not automatically approve subsequent data from the same session after a threshold violation.

### 15.3 Continuous Float duration and interval records

Record t_float_start as Snapshot_TimeUS of the first observed Status = 5 snapshot. While Float persists, compute `elapsed_us = Snapshot_TimeUS - t_float_start`; at `10,000,000 us` or more it is a long Float interval. Do not estimate time from the number of 10 Hz frames or repeated source GPS samples. Reusing 5 Hz GPS samples at 10 Hz uses the same elapsed-time rule.

On return to 6, use its first snapshot time as the interval end and reassess final duration. An end-minus-start duration of exactly 10 s is classified as >=10 s. Close the continuous Float interval; retain its start/end/duration and review requirement. Return to Fixed does not erase review history for intervals shorter than 10 s. Do not automatically sum separate Float intervals into one continuous 10 s interval. Missing statuses or incomplete snapshots must not automatically qualify as a valid short Float interval when continuity cannot be established. Transitions use recorded 10 Hz snapshot timing, not an asserted more precise physical GNSS transition time.

### 15.4 Real-time control versus post-processing ownership

Currently FC continues observation in Float while source data remains valid. **Float >=10 s alone does not add automatic Stop.** Existing immediate termination for ordinary 3D or lower, heartbeat timeout, source-age violations and navigation/EKF failsafe remains in force. Do not encode post-processing rejection as a new MAVLink SAR_STATUS value.

PNU Drone implements consistent original Status/time delivery in FC. The receiver must preserve snapshot/status/time history for quality reassessment. Both vendors must agree how this history is associated with SAR raw data, where quality intervals are stored and who implements the actual post-processing application. This revision defines post-processing decisions; it does not require the entire post-processing application to be implemented in FC.

## 16 Meaning and generation of gimbal-stabilized SAR attitude

### 16.1 CAN ATTITUDE reference object and coordinates

**CAN 0x404 ATTITUDE Roll/Pitch/Yaw are the actual earth-referenced attitude of the gimbal-stabilized SAR device.** The drone log ATT.Roll/Pitch/Yaw describes the airframe and has different semantics. Gimbal compensation can hold SAR attitude constant while airframe attitude changes.

Use forward/right/down (FRD) SAR mounting reference axes and north/east/down (NED) earth conventions. Positive Roll lowers the right SAR reference axis; positive Pitch raises the SAR forward axis. Yaw is 0 deg north, 90 deg east, normalized to [0, 360) deg. Agree and apply any fixed transformation between SAR reference axes and antenna electrical pointing axes during implementation; do not add mounting offsets to existing CAN payloads.

Retain 0x404 name, ID, DLC, layout, types, multiplication = 100 and 0.01 deg resolution. `SAR_Attitude` names a logical ICD data provider; it does not assert an existing ArduPilot API. GPS_Yaw remains GPS-derived heading and must not be replaced with SAR Yaw.

### 16.2 Roll/Pitch stabilization envelope

FC controls the SAR stabilization gimbal. Within the **25 deg** airframe Roll/Pitch operating envelope, stabilize earth-referenced SAR Roll and Pitch to **0 deg targets**. This revision interprets the envelope per axis as `abs(vehicle_roll) <=25 deg` and `abs(vehicle_pitch) <=25 deg`, including the 25 deg boundary. If a combined tilt-angle limit is intended, confirm that separately before implementation.

A zero target does not guarantee zero actual error. Transmit measured/estimated actual SAR Roll/Pitch. Do not force numeric zero merely because the airframe lies inside the envelope. Gimbal behavior, saturation handling and observation permission outside the 25 deg envelope remain undefined.

### 16.3 Yaw stabilization on straight waypoint segments

The direction of a straight observation segment is the **geometric heading from its entry waypoint to its exit waypoint**. Distinguish it from instantaneous airframe yaw, GPS ground course and vehicle heading fluctuations. Reverse traversal uses the waypoint order matching travel direction. Heading coordinates/projection must match the navigation implementation and be normalized relative to north.

Compute the smallest signed angular error as `yaw_error = wrap_180(vehicle_yaw - segment_heading)`. During straight flight with **abs(yaw_error) <=15 deg**, the gimbal holds SAR Yaw at segment_heading. Include the 15 deg boundary: 359 deg versus 1 deg is a 2 deg error, not 358 deg.

Transmit actual SAR Yaw. Geometric heading is a stabilization target; do not copy it as verified actual attitude without an attitude source. Confirm identification of valid straight segments and target transitions/rates/observation permission during turns, waypoint transitions and errors beyond 15 deg before implementation. Coincident waypoints do not define a valid heading.

### 16.4 Actual attitude acquisition, validity and implementation boundaries

Obtain actual SAR attitude from gimbal feedback/device attitude sensors, or an estimate combining FC airframe attitude with valid relative gimbal attitude and mounting transformations. A command angle alone does not establish actual attitude. Use proper rotation composition such as matrices/quaternions; general three-axis attitude cannot be obtained by independently adding/subtracting Euler components.

If actual attitude is unavailable, use existing sentinels and per-axis ATT_Flags. Do not substitute zero/segment_heading setpoints and mark them valid. ATT_FRESH and the existing age <=40 ms limit apply to this actual attitude estimate and required source inputs. Actual SAR attitude age violations follow the existing termination policy. Acquire it with the other navigation fields at the 0x408 snapshot time.

PNU Drone owns FC gimbal control and CAN attitude generation. Feedback APIs/sensors, actual update rate/latency, mounting-axis calibration, acceptable stabilization residuals and behavior outside the envelope still require confirmation. This definition adds no CAN IDs, payload fields or MAVLink fields and does not define a new gimbal-control protocol.

### 16.5 Additional verification scenarios

Verify complete-session rejection with initial status 5/3, Float boundaries at 9.9/exactly 10/10.1 s, final-duration classification on return to Fixed, and retention of separate short Float interval histories. For attitude, verify distinction between airframe and SAR angles, per-axis 25 deg and yaw 15 deg boundaries, 359/1 deg heading wrap, reverse waypoints, SAR stabilization during airframe tilt changes, and no valid setpoint substitution when actual feedback is unavailable.
