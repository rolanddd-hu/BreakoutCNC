# `RNR ECO MOTION 2.0` (`RnRMotion.dll`) Complete USB HID Protocol Specification

Documented for interoperability, so BreakoutCNC can drive the board in place of its Mach3 plugin (`RnRMotion.dll`).
No vendor code is part of this repository.

## 1. USB Hardware & HID Enumeration Identity
- **VID:** `0xA720`
- **PID:** `0xF803`
- **Version:** `0x0300`
- **Manufacturer:** `RobotAndRobot.com`
- **Product:** `RNR ECO MOTION 2.0`
- **HID Usage Page / Usage:** `0xFF00` / `0x0001`
- **Output Report Length (PC -> Board):** `27 bytes (0x1B)`
- **Input Report Length (Board -> PC):** `49 bytes (0x31)`

## 2. Output Report Layout (27 bytes)
- `buf[0]`: `0x02` (Short Control Packet) or `0x03` (Extended Motion/Config Packet)
- `buf[1]`: Command Opcode (`cmd`)
- `buf[2]`: Flags (`0x40` if Limit Override active, else `0x00`)
- `buf[3..4]`: `uint16_t` (LE) Sequence counter (`seq`, incremented by +1 per write)
- `buf[5..26]`: 22-byte Command Payload

### Command Opcodes (`buf[1]`)
- `0x01` (`buf[0]=0x02`): E-Stop / Abort immediately
- `0x02` (`buf[0]=0x02`): Clear E-Stop / Enable
- `0x03` (`buf[0]=0x02`): Set Outputs immediately (`buf[5]` = bitmask `OUT1..OUT4`)
- `0x04` (`buf[0]=0x03`): G-Code Motion Slice (`buf[5..8]`=hdr, `buf[9..12]`=dX, `buf[13..16]`=dY, `buf[17..20]`=dZ, `buf[21..24]`=dA, `buf[25]`=Outputs)
- `0x05` (`buf[0]=0x03`): Internal Jog/Home/Probe Motion Slice (same layout as `0x04`)
- `0x06` (`buf[0]=0x02`): End of Motion / Sync Outputs (`buf[5]` = bitmask `OUT1..OUT4`)
- `0x07` (`buf[0]=0x02`): Pre-Reset / Clear State
- `0x08` (`buf[0]=0x02`): Cancel / Stop Sequence
- `0x09` (`buf[0]=0x03`): Set 4-Axis Step Positions (`buf[5..8]`=X, `buf[9..12]`=Y, `buf[13..16]`=Z, `buf[17..20]`=A, `buf[21]`=`0x0F` axis mask)
- `0x0A` (`buf[0]=0x02`): Start Continuous Jog / Pull-off (`buf[5]`=axis `0..3`, `buf[6]`=slave flag `0` or `3`)
- `0x0B` (`buf[0]=0x02`): Start Step / MPG Jog (`buf[5]`=axis `0..3`, `buf[6]`=slave flag `0` or `3`)
- `0x0C` (`buf[0]=0x02`): Start Probe (`G31`)
- `0x0D` (`buf[0]=0x02`): Start Axis Homing (`buf[5]`=axis `0..3`, `buf[6]`=slave flag `0` or `3`)
- `0x0E` (`buf[0]=0x02`): Set Spindle PWM (`buf[5..6]`=PWM duty, `buf[7..8]`=PWM divider default `5`)
- `0x0F` (`buf[0]=0x03`): Board Pinout, Inversion & Timing Config (`buf[5..12]`=axis limit/home pins, `buf[13..14]`=probe/MPG/EStop pins, `buf[15]`=step/dir invert mask, `buf[16]`=PulseRate kHz, `buf[17]`=slice ms)
- `0x10` (`buf[0]=0x02`): Stop Jog Deceleration

## 3. Input Report Layout (49 bytes)
- `rx[0]`: Report Header
- `rx[1]`: Board Motion State (`0`=Idle, `1`=Busy/Seq)
- `rx[2]`: Hardware Fault Flags (`0x10`=Limit Switch Hit, `0x20`=E-Stop Hit)
- `rx[3..4]`: `uint16_t` (LE) Last Processed Packet Sequence (`ack_seq`)
- `rx[5..8]`: `uint32_t` (LE) Status / Actual Spindle Counter
- `rx[9..12]`: `int32_t` (LE) Live X Step Position
- `rx[13..16]`: `int32_t` (LE) Latched X Step Position (Probe/Home)
- `rx[17..20]`: `int32_t` (LE) Live Y Step Position
- `rx[21..24]`: `int32_t` (LE) Latched Y Step Position (Probe/Home)
- `rx[25..28]`: `int32_t` (LE) Live Z Step Position
- `rx[29..32]`: `int32_t` (LE) Latched Z Step Position (Probe/Home)
- `rx[33..36]`: `int32_t` (LE) Live A Step Position
- `rx[37..40]`: `int32_t` (LE) Latched A Step Position (Probe/Home)
- `rx[41]`: `uint8_t` Aux status
- `rx[42..43]`: `uint16_t` (LE) Digital Inputs Bitmask (`IN1..IN12`, bits `0..11`)
- `rx[44..45]`: `uint16_t` (LE) Hardware Tick Counter (200 Hz timebase for velocity)
- `rx[46..47]`: `uint16_t` (LE) MPG Handwheel Encoder Raw Value (`>> 2` for 14-bit count)
- `rx[48]`: Trailing byte
