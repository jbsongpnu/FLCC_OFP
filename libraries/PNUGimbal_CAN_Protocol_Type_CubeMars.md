# PNUGimbal CAN Protocol Type CubeMars GLⅡ CAN protocol — coding agent reference

- Source: `gl-ii-gimbal-motor-drive-user-manual.pdf`, V1.0.0, 2024-07-18, 49 pages.
- Target hardware supplied by user: GL60Ⅱ KV28. Source is a GLⅡ driver-family manual; listed encoding ranges are not GL60Ⅱ KV28 mechanical/electrical operating limits.
- Page references below are PDF page numbers.
- Scope: CAN runtime control and feedback. UART setup, calibration and firmware update protocols are not specified in this manual.
- Labels: **specified** = explicit manual content; **derived** = interpretation from tables/examples; **proposed** = host implementation policy, requiring device validation.

## 1. Bus and configuration [pp.6–7, 17, 21–23, 41]

| Item | Value |
|---|---|
| Bus | Classical CAN, standard 11-bit data frame |
| Bitrate | Fixed 1,000,000 bit/s |
| IDE / RTR | Standard / data; not extended or remote |
| Command DLC | MIT: 8; position/velocity: 8; velocity: 4 |
| Feedback DLC | 8, per table Data[0..7] |
| Command address | Configured motor CAN ID, low 8 bits of command arbitration ID |
| Feedback arbitration ID | Configured Master ID; default 0 |
| Driver power | Rated 24 V, allowable 8–32 V; driver-family specification |
| Connector | Power/CAN: pin 1 +, pin 2 −, pin 3 CAN_H, pin 4 CAN_L |

- Configure control mode, CAN ID, Master ID, P/V/T ranges and control gains using the vendor PC utility before runtime control.
- Write mode configuration and power-cycle; confirm the stored mode before sending commands.
- Motor is **enabled at power-up** per manual.
- A command's mode bits must match the configured mode. They are not documented as a runtime mode-switch mechanism.
- CAN has priority over PWM when both are present [p.38].
- CAN configuration read/write commands, termination topology, command period, feedback trigger/rate, loss timeout, watchdog recovery and special-command acknowledgment are not specified. Do not invent their behavior.
- Do not assume this proprietary protocol is DroneCAN/UAVCAN or supports automatic node discovery.

## 2. Command arbitration ID [p.41]

```text
bits 10..8 = mode
bits  7..0 = motor CAN ID
command_id = (mode << 8) | motor_id
```

| Configured mode | mode | ID for motor_id=1 |
|---|---:|---:|
| MIT | 0 | 0x001 |
| Position/velocity | 1 | 0x101 |
| Velocity | 2 | 0x201 |

The manual labels bits 7..0 “Source node ID”; examples address the motor using its configured CAN ID. Treat this as the command's motor address (**derived**).

## 3. Universal commands [p.41]

All use DLC=8.

| Action | Data[0..7] |
|---|---|
| Enable / enter motor control | FF FF FF FF FF FF FF FC |
| Disable / exit motor control | FF FF FF FF FF FF FF FD |
| Set current position to zero | FF FF FF FF FF FF FF FE |
| Clear errors | FF FF FF FF FF FF FF FB |

**Derived:** use the command arbitration ID for the configured mode and motor address. The manual provides no separate universal-command ID example; confirm this on hardware, especially in velocity mode where ordinary commands have DLC=4.
Zero-setting persistence and error-clear side effects are unspecified. Zero-setting must be an explicit action, not an automatic startup step.

## 4. MIT command [pp.42–43]

DLC=8. Five physical values are mapped to **unsigned offset integers** and bit-packed. Do not encode these fields as signed two's-complement integers or float32.

| Field | Bits | Manual preset range | Unit / caveat |
|---|---:|---|---|
| p | 16 | −12.5 .. +12.5 | rad |
| v | 12 | −200 .. +200 | rad/s provisionally; unit conflict in §9 |
| kp | 12 | 0 .. 500 | Manual says N/rad; torque-control dimensional interpretation is N·m/rad |
| kd | 12 | 0 .. 5 | Manual says N·s/rad; dimensional interpretation is N·m·s/rad |
| t | 12 | −10 .. +10 | N·m, feedforward torque |

P/V/T encoding limits can be changed in the PC utility. Host encode/decode limits MUST match actual device settings. Do not treat ±10 N·m as the target motor's permissible torque.

### 4.1 Numeric mapping

**Proposed, consistent with receive equation and several published examples (not all):**

```text
Q = (1 << bits) - 1
u = floor((clamp(x, xmin, xmax) - xmin) * Q / (xmax - xmin))
x = xmin + u * (xmax - xmin) / Q
```

Validate finite values and xmax > xmin. Clamp before integer conversion. Packed p/v/t zero is near the middle of the integer range; an all-zero payload does not mean zero command.

The manual's transmit function uses `1 << bits` instead of `(1 << bits)-1`, while its receive function uses the latter. Literal transmit code can produce 65536/4096 at the maximum and overflow the field. Keep this discrepancy explicit; do not copy the function unchanged. Verify quantization on the actual firmware.

### 4.2 Byte layout

Let P, V, KP, KD, T denote encoded unsigned values:

```text
D0 = (P >> 8) & 0xFF
D1 = P & 0xFF
D2 = (V >> 4) & 0xFF
D3 = ((V & 0x0F) << 4) | ((KP >> 8) & 0x0F)
D4 = KP & 0xFF
D5 = (KD >> 4) & 0xFF
D6 = ((KD & 0x0F) << 4) | ((T >> 8) & 0x0F)
D7 = T & 0xFF
```

### 4.3 Control intent

- Position control: nonzero kp and **nonzero kd**; kd=0 is explicitly warned against.
- Velocity control: kp=0, kd>0, desired v; normally t=0.
- Torque control: kp=0, kd=0, desired t.
- Conceptual impedance law: `tau = kp*(p_des-p) + kd*(v_des-v) + t_ff` (**interpretation**).
- Do not use the PC demonstration gains as validated gains for the installed load.

## 5. Position/velocity command [pp.44–45]

- Configured mode=1, DLC=8.
- D0..D3: target position, 32-bit float, rad.
- D4..D7: maximum absolute travel speed, 32-bit float, rad/s; not a signed velocity tracking target.
- Wire order is least significant byte first. **Derived representation:** IEEE-754 binary32, little-endian, supported by the sample C and hexadecimal examples.

```text
payload = float32_le(p_des) || float32_le(v_limit)
```

Listed ranges: position ±12.5 rad, speed ±200 rad/s. Actual usable limits depend on configuration/hardware. Use a nonnegative speed limit as host policy; behavior for negative values is not defined.
Position, speed and current loops are cascaded. Acceleration/deceleration and positive damping are configured outside this runtime frame.

## 6. Velocity command [pp.45–46]

- Configured mode=2, **DLC=4**.
- D0..D3: target velocity as float32 little-endian.
- Units: use rad/s provisionally; p.46 mixes rad/s and r/s.
- Listed range ±200; actual permissible speed must follow configured motor limits.

```text
payload = float32_le(v_des)
```

PC utility automatic parameter calculation requires positive nonzero damping; manual gives typical 2–10 and recommends 4. This is a configuration value, not a CAN payload field.

## 7. Common feedback [pp.47–48]

All modes share this format. Arbitration ID is **Master ID**, default 0; do not expect the command ID or motor ID as feedback arbitration ID.

| Byte | Contents |
|---|---|
| D0[7:4] | status/error code |
| D0[3:0] | motor CAN ID low nibble, per table; see ID conflict below |
| D1 | position[15:8] |
| D2 | position[7:0] |
| D3 | velocity[11:4]; table incorrectly says position |
| D4[7:4] | velocity[3:0] |
| D4[3:0] | torque[11:8] |
| D5 | torque[7:0] |
| D6 | driver/MOS temperature, signed int8, °C |
| D7 | motor temperature, signed int8, °C |

```text
status = D0 >> 4
id_low4 = D0 & 0x0F
P = (uint16(D1) << 8) | D2
V = (uint16(D3) << 4) | (D4 >> 4)
T = (uint16(D4 & 0x0F) << 8) | D5
position = decode_unsigned(P, P_MIN, P_MAX, 16)
velocity = decode_unsigned(V, V_MIN, V_MAX, 12)
torque   = decode_unsigned(T, T_MIN, T_MAX, 12)
driver_temp = D6 if D6 < 128 else D6 - 256
motor_temp  = D7 if D7 < 128 else D7 - 256
```

Use unsigned intermediates, especially for P: the manual's int16_t example misrepresents values >=0x8000. Its pointer-assignment sample also contains invalid C; implement the layout above instead of copying it.

**ID conflict:** prose says ID occupies the lower 8 bits of CAN_ID, but D0's upper nibble already holds status. The documented payload carries only four motor-ID bits. Do not identify a motor by all eight bits of D0. Until verified, use motor IDs 1–15 and separate valid Master IDs where needed; prevent shared feedback-ID collisions and ambiguous demultiplexing.

| status (hex) | Meaning |
|---|---|
| 0 | Disabled |
| 1 | Enabled |
| 8 | Overvoltage |
| 9 | Undervoltage |
| A | Overcurrent |
| B | MOS overtemperature |
| C | Motor winding overtemperature |
| D | Communication loss |
| E | Overload |

Status 0 and 1 are normal operating states, not faults. Other undocumented codes remain unknown; preserve raw frames and codes.

## 8. Published command vectors [pp.48–49]

motor_id=1; preset scaling. MIT values in descriptions are nominal and quantized. These are manual reference vectors, not proof of device behavior.

**Additional source error:** the published MIT ±6 rad/s vectors do not match the stated ±200 range. Their encoded velocities are 0x830 / 0x7CE, decoding to approximately +4.737 / −4.835 at that range. Under the proposed codec, ±6 encodes to 0x83C / 0x7C2, giving payloads `7F FF 83 C0 00 00 47 FF` / `7F FF 7C 20 00 00 47 FF`. Preserve the original table below as evidence; do not use its speed rows as passing codec tests.

| Command | CAN ID | DLC | Hex payload |
|---|---|---:|---|
| MIT v=+6, kd=0.005 | 0x001 | 8 | 7F FF 83 00 00 00 47 FF |
| MIT v=−6, kd=0.005 | 0x001 | 8 | 7F FF 7C E0 00 00 47 FF |
| MIT p=+2, kp=0.123, kd=0.005 | 0x001 | 8 | 94 7A 7F F0 01 00 47 FF |
| MIT p=−2, kp=0.123, kd=0.005 | 0x001 | 8 | 6B 84 7F F0 01 00 47 FF |
| MIT t=+0.03 | 0x001 | 8 | 7F FF 7F F0 00 00 08 05 |
| MIT t=−0.03 | 0x001 | 8 | 7F FF 7F F0 00 00 07 F9 |
| Position +3 rad, limit 1 rad/s | 0x101 | 8 | 00 00 40 40 00 00 80 3F |
| Position −3 rad, limit 1 rad/s | 0x101 | 8 | 00 00 40 C0 00 00 80 3F |
| Velocity +2 rad/s | 0x201 | 4 | 00 00 00 40 |
| Velocity −2 rad/s | 0x201 | 4 | 00 00 00 C0 |

## 9. Unresolved manual issues / implementation contract

1. **Velocity units:** MIT and feedback sections say revolutions/s or r/s; mode descriptions and CAN examples say rad/s. Use rad/s only as an explicit provisional assumption. Confirm with a low-speed measured rotation before exposing physical-unit control; do not silently multiply/divide by 2π.
2. **Quantization/examples:** TX uses 2^N, RX uses 2^N−1; some examples align with RX, but MIT ±6 speed vectors conflict with the stated ±200 range. Proposed codec uses 2^N−1, saturates maxima and truncates; verify against firmware.
3. **Feedback identity:** four-bit table vs eight-bit prose; full 8-bit identity recovery is not documented.
4. **Feedback D3 label:** use velocity, supported by unpacking code and D4 layout.
5. **Mode transitions:** persist configuration and power-cycle; no documented runtime CAN mode-change command.
6. **Timing/ACK:** no documented rate, timeout duration, ACK, automatic feedback policy, or retry semantics. Expose these as integration parameters requiring measurement; never claim a specific watchdog interval.
7. **Zero/clear:** zero persistence and clear-fault recovery sequence require device verification.

Suggested host structure: immutable configured mode + explicit IDs + per-motor encoding limits; separate MIT/float encoders; one feedback decoder; raw-frame logging; explicit enable/disable/zero/clear APIs.
Reject wrong IDE/RTR/DLC, nonfinite command floats and invalid configured ranges. Start position control from the measured current angle; stale feedback must be handled by an application-defined policy.

