# Huanyang VFD RS485 Communication Protocol Specification

Documented for interoperability with the Mach3 Huanyang VFD plugin (`vfdPlugin.dll` by Sébastien / *royaumedeole*), so
BreakoutCNC can drive the same inverters and import the plugin's settings. The Modbus-style frames themselves are
described in the Huanyang inverter manual (PD163-PD165).

---

## 1. Physical & Serial Link Layer
- **Interface:** RS485 Half-Duplex (two-wire: `RS+` / `A` and `RS-` / `B` terminals on the VFD).
- **Default Baud Rates:** 9600 bps or 19200 bps (configured on VFD via `PD164`: `0`=4800, `1`=9600, `2`=19200, `3`=38400).
- **Data Format:** 8 Data bits, No Parity (or Even Parity), 1 Stop bit (`8N1` / `PD165 = 3`).
- **Default Slave Address:** `0x01` (configured on VFD via `PD163`).
- **Timing Constraint:** Minimum 50ms–60ms inter-frame quiet time between queries to avoid buffer overrun on the VFD microcontroller.

---

## 2. Checksum Algorithm
Standard Modbus CRC-16 (Polynomial: `0xA001`, Initial Value: `0xFFFF`).
Appended to every query and response in **Little-Endian** order (`CRC_Low`, `CRC_High`).

```cpp
uint16_t calculateCrc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
```

---

## 3. Command Frames (Host -> VFD)

### 3.1 Function 0x03: Write Control Data (Run / Stop / Direction)
Format: `[Slave] [0x03] [0x01] [Control_Byte] [CRC_Lo] [CRC_Hi]` (6 bytes total)

| Control Byte | Description | G-Code Equivalent |
|---|---|---|
| `0x01` | Run Spindle Forward | `M3` |
| `0x11` | Run Spindle Reverse | `M4` |
| `0x08` | Stop Spindle | `M5` / E-Stop |
| `0x02` | Jog Forward | Manual Jog |
| `0x12` | Jog Reverse | Manual Jog |

*Example (Run Forward, Slave 1):*
`01 03 01 01 31 88`

*Example (Stop, Slave 1):*
`01 03 01 08 F1 8E`

### 3.2 Function 0x05: Write Frequency Data (Spindle Speed S)
Format: `[Slave] [0x05] [0x02] [Freq_High] [Freq_Low] [CRC_Lo] [CRC_Hi]` (7 bytes total)

The frequency is specified in **hundredths of a Hertz** ($0.01\text{ Hz}$).
- $\text{Frequency (Hz)} = \frac{\text{Commanded RPM}}{\text{Max Motor RPM}} \times \text{Max Frequency (Hz)}$
- For standard Chinese 24,000 RPM, 400 Hz spindle:
  $\text{Freq (Hz)} = \frac{\text{RPM}}{60.0}$
  $\text{Value} = \text{Freq (Hz)} \times 100 = \frac{\text{RPM} \times 100}{60}$
  - $24,000\text{ RPM} \rightarrow 400.00\text{ Hz} \rightarrow 40000 = \text{0x9C40}$
  - $12,000\text{ RPM} \rightarrow 200.00\text{ Hz} \rightarrow 20000 = \text{0x4E20}$
  - $6,000\text{ RPM} \rightarrow 100.00\text{ Hz} \rightarrow 10000 = \text{0x2710}$

*Example (Set 24,000 RPM / 400.00 Hz on Slave 1):*
`01 05 02 9C 40 D0 3C`

### 3.3 Function 0x04: Read Control Status (Telemetry Query)
Format: `[Slave] [0x04] [0x01] [Parameter_Code] [CRC_Lo] [CRC_Hi]` (6 bytes total)

| Parameter Code | Description | Scale Factor |
|---|---|---|
| `0x00` | Target / Set Frequency (`Set_F`) | $0.01\text{ Hz}$ |
| `0x01` | Actual Output Frequency (`Out_F`) | $0.01\text{ Hz}$ |
| `0x02` | Actual Output Current (`Out_A`) | $0.1\text{ A}$ |
| `0x03` | Actual Motor RPM (`RoTT`) | $1\text{ RPM}$ |
| `0x04` | DC Bus Voltage (`DCV`) | $1\text{ V}$ |
| `0x05` | AC Output Voltage (`ACV`) | $1\text{ V}$ |
| `0x06` | Inverter Cont State | Bitmask |
| `0x07` | Temperature (`Tmp`) | $1\text{ }^\circ\text{C}$ |

*Example (Query Actual Motor RPM on Slave 1):*
`01 04 01 03 31 8F`

### 3.4 Function 0x01: Read Internal Parameter PDxxx
Format: `[Slave] [0x01] [0x03] [PD_Index] [0x00] [0x00] [CRC_Lo] [CRC_Hi]` (8 bytes total)
- `PD001`: Control mode (`0` = operator panel, `1` = terminal, `2` = RS485 communication)
- `PD002`: Frequency source (`0` = operator knob, `1` = 0-10V analog terminal, `2` = RS485 communication)
- `PD005`: Max operating frequency (default `400.00` Hz)
- `PD144`: Rated motor speed at 50 Hz (default `3000` RPM for 2-pole motors)

---

## 4. Response Frames (VFD -> Host)

### 4.1 Response to Write Commands (0x03, 0x05)
Echoes slave address, function code, length, and acknowledge data, followed by 2-byte CRC.

### 4.2 Response to Read Status (0x04)
Format: `[Slave] [0x04] [Length] [Data_High] [Data_Low] [CRC_Lo] [CRC_Hi]` (6 to 8 bytes)
- For `0x03` (RPM): Returns 16-bit big-endian actual spindle RPM.
- For `0x01` (Out_F): Returns 16-bit big-endian actual output frequency $\times 100$.
- For `0x02` (Out_A): Returns 16-bit big-endian actual output current $\times 10$.

---

## 5. Required VFD Parameter Configuration
To allow RS485 digital control, the physical VFD keypad must be programmed with:
1. `PD001 = 2`: Run command source = RS485 Communication
2. `PD002 = 2`: Operating frequency source = RS485 Communication
3. `PD163 = 1`: Slave address = 1
4. `PD164 = 1`: Baud rate = 9600 bps (or `2` for 19200 bps)
5. `PD165 = 3`: 8N1 RTU mode

---

## 6. Plugin v2 (`vfdPlugin.dll`, 2013 release) - Settings & Supervision

The plugin stores its settings in the Mach3 profile XML (e.g. `C:\Mach3\Mach3Mill.xml`) in its own section next to
`<Preferences>`; `<PlugIns><VFD-Driver-by-royaumedeole.fr>1</...>` means the plugin is enabled.

| `<VfdDriver>` key | Default | Meaning | BreakoutCNC field |
|---|---|---|---|
| `ComPort` | 1 | COM port number, opened as `\.\COM%d` | `vfd.port_name` = `COM<n>` on Windows; on Linux / macOS the configured device path is kept |
| `BaudRate` | 9600 | Baud rate | `vfd.baud_rate` |
| `Address` | 1 | Slave address | `vfd.slave_address` |
| `PulleyOption` | 0 | "Pulley management": scale by Mach3 `SPINRATIO<n>` of the active pulley | not used (speed range from `SPEED<n>`/`LSPEED<n>`) |
| `EStopOption` | 0 | "Automatic EStop": speed monitoring on/off | `vfd.speed_monitor_enabled` |
| `Margin` | 3 | Speed tolerance, % of the commanded RPM | `vfd.speed_margin_pct` |
| `BadSpeedThreshold` | 3 | Consecutive out-of-band RPM readings at speed before E-Stop ("Bad Speed detected") | `vfd.bad_speed_threshold` |
| `BadtrackingThreshold` | 3 | Consecutive readings without getting closer to the target while the speed changes ("Tracking fail") | `vfd.tracking_threshold` |
| `EStopCauseOption` | 0 | "Notify": message box with the E-Stop cause | always shown in BreakoutCNC |
| `Status` | 62 | Bitmask of polled status values (bit i = status code i) | always polled |
| `Status<i>_dro` | 0 | Mach3 DRO that shows status value i | not used |
| `Status<i>_lim` | 0 | E-Stop when status value i goes above it ("Monitoring Estop"); checked whenever polled, independent of `EStopOption` | `vfd.limit_*` |

Status slot i is the function 0x04 status code i. Decimal places per code from the plugin table `{2, 2, 1, 0, 1, 1, 0, 1}`:
0 set frequency (Hz/100), 1 output frequency (Hz/100), 2 current (A/10), 3 RPM (1), 4 DC bus voltage (V/10),
5 AC output voltage (V/10), 6 counter (1), 7 temperature (C/10). Note: this differs from section 3.3 (older plugin)
for DC voltage and temperature; BreakoutCNC follows plugin v2.

Speed monitoring (plugin v2, reproduced in `VfdSpeedMonitor`): when the commanded speed changes, each RPM reading
must be closer to the target than the previous one; more than `BadtrackingThreshold` readings without progress ->
E-Stop. Once inside target +/- `Margin`%, more than `BadSpeedThreshold` consecutive readings outside the band -> E-Stop.
E-Stop = Mach3 OEM button 1021. The RPM reading depends on PD144 ("Verify your PD144 parameter (speed at 50hz)").

Parameter read (function 0x01, used by "Read Parameters from VFD"): `[Slave] [0x01] [0x03] [PD] [0x00] [0x00] [CRC]`,
reply `[Slave] [0x01] [0x03] [PD] [Hi] [Lo] [CRC]`. PD005 / PD011 in 0.01 Hz, PD144 in RPM. Speed at a frequency f is
`PD144 * f / 50` (plugin message "%s speed: %drpm at %dHz (PD%03d)"). The reply layout is taken from the plugin and
has not been checked against a VFD yet.
