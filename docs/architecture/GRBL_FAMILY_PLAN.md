# GRBL-family Boards (GRBL 1.1, grblHAL, FluidNC): Detailed Plan

Status: **implemented for 0.5, not yet validated on real boards.** Phases 1-4 and most of 5 and 7 are in the code
(`src/core/board/grbl/`, `src/core/motion/`, `src/ui_common/board_panels.*`, `tests/board_tests.cpp`) and were tried
in both applications on the built-in emulator (`--fake-board`). What is not built yet is listed at the end of
section 17; what needs real hardware is section 18. Differences from this plan: single moves use `G53` and probes
are incremental `G38.3`, so they never depend on the firmware frame offset F; laser jobs require `$32 = 1` and leave
rapids to the board's laser mode; plasma runs without THC instead of being refused, except on grblHAL with the
plasma plugin, where the board runs THC (capability `board_thc`, see section 12).

This document is the implementation plan for the first firmware-planned boards. It assumes Phase 0 of
[MULTI_BOARD_PLAN.md](MULTI_BOARD_PLAN.md) is done: the board registry, capabilities, the I/O model, the settings
split, `SegmentResolver`, `IMotionPath` and the transports exist. Everything here is new code behind those
interfaces.

Facts about the firmwares come from their own documentation and sources (section 19). Items that could not be
confirmed there are collected in section 18 and must be checked on real hardware or in the firmware source before
the code that depends on them is written.

## 1. The three flavours

One protocol class (`GrblProtocol`) serves all three. The differences are data in a `FirmwareProfile`, built when
connecting.

| Topic | GRBL 1.1 | grblHAL | FluidNC |
|---|---|---|---|
| Typical hardware | Arduino Uno / Nano, 3018 boards | STM32, Teensy, RP2040, ESP32 boards | ESP32 boards |
| Welcome line | `Grbl 1.1h ['$' for help]` | `GrblHAL 1.1f ['$' or '$HELP' for help]` | `Grbl 3.x [FluidNC v3.x.y ... '$' for help]` (configurable text) |
| Sure detection | `[VER:1.1...]` and neither of the others | `[FIRMWARE:grblHAL]` in `$I`; `FW:grblHAL` in the full status report | `FluidNC` in the welcome line and in `[VER:...]` |
| Buffer sizes | `[OPT:codes,blocks,rx]`, normally 15 or 16 blocks and 128 bytes | `[OPT:codes,blocks,rx,axes,tools]` | Not in `[OPT:]`; planner blocks from the config (default 16) |
| Longest line | 80 characters | Longer (to confirm) | Longer (to confirm) |
| Axes | 3 | 3 to 8, from `[AXS:n:letters]` | From the config |
| Reset when the port opens | Yes on Arduino (DTR) | No on native USB | Possible on ESP32 (DTR / RTS) |
| Real-time commands | ASCII `?` `!` `~`, `0x18`, `0x84`, `0x85`, `0x90`–`0x9D`, `0x9E`, `0xA0`, `0xA1` | The same, plus top-bit versions `0x80`–`0x82`, full report `0x87`, stop `0x19`, others | As GRBL 1.1, plus macro bytes |
| Status report extras | none | sub-states, `H:` (homed), `MPG:`, `WCS:`, `TLR:`, `In:`, more `Pn:` letters incl. `E` (E-stop) | job progress element; `T` (tool setter) in `Pn:` |
| Automatic status reports | No | Yes, toggled with `0x8C` (to confirm) | Yes, `$Report/Interval=<ms>` per channel |
| Homing | `$H`; single axis only with build option `H` | `$H`, `$HX`... when enabled | `$H`, `$HX`..., `$H` with an axis list |
| Homed state reported | No | Yes (`H:`) | No (to confirm) |
| Settings model | `$$` numbered | `$$` numbered, described by the controller (`$ES`, `$EG`, `$SED=n`) | `config.yaml` (`$CD` dump, `$/path=value`), plus stored settings (`$S`) |
| Canned cycles | No | G73, G81–G83, G85, G86, G89 | No (to confirm) |
| Tool length | G43.1 / G49 | G43 (option), G43.1, G43.2, G49 | G43.1 / G49 (to confirm) |
| Extra outputs / inputs | No | M62–M65, M66, M67 / M68; counts in `[AUX IO:di,do,ai,ao]` | `user_outputs` with M62–M65 (to confirm), `user_inputs` |
| Exact stop / blending | G61 accepted (the only mode) | G61 | G61 (to confirm) |
| Alarm and error texts | Fixed lists (1–9, 1–38) | `$EA`, `$EE` list them | `$A`, `$E` list them |
| Spindle on the board | PWM, laser mode | PWM, laser, VFD plugins | PWM, laser, VFDs |
| Transports | Serial | Serial, USB, and Ethernet / WiFi telnet on some boards | Serial, telnet (port 23), WebSocket |

The board selector offers the three as separate entries, because their settings panels and defaults differ. If the
board that answers is a different flavour than the selected one, the app says so and offers to switch the
selection (the connection settings are kept).

## 2. Files

```
src/core/board/grbl/
  grbl_messages.*        line -> typed message (status, ok, error, alarm, [PRB], [GC], $n=, [VER], ...)
  grbl_protocol.*        IFirmwareProtocol: connect, flow control, real-time bytes, reader thread
  firmware_profile.*     detection and capability building for the three flavours
  grbl_dialects.*        GcodeDialect tables: grbl11, grblhal, fluidnc
  grbl_codes.*           error and alarm texts (own wording) and their board-neutral FaultCode
  grbl_settings.*        IBoardSettings for $$ (GRBL 1.1) and enumerated settings (grblHAL)
  fluidnc_config.*       IBoardSettings for config.yaml and $S
  grbl_register.cpp      the three BoardDescriptors
src/core/motion/         firmware_planned_path.*  gcode_emitter.*  path_tracker.*  jog_feeder.*   (shared, family B)
src/ui_common/boards/    grbl_settings_panel.*  fluidnc_config_panel.*  console_panel.*
tests/                   grbl_messages_test  gcode_emitter_test  firmware_path_test  fakes/fake_grbl.*
tests/data/emitter/      *.nc with *.grbl11.expected, *.grblhal.expected, *.fluidnc.expected
```

## 3. Interfaces

```cpp
namespace rnr::board {

enum class Flavour : uint8_t { Grbl11, GrblHal, FluidNC, GenericGrbl };

struct FirmwareProfile {
    Flavour     flavour = Flavour::GenericGrbl;
    std::string version_text;                 // "[VER:...]" content
    std::string option_codes;                 // "[OPT:...]" letters
    size_t      planner_blocks = 15;
    size_t      rx_buffer_bytes = 128;
    size_t      max_line_chars = 80;
    uint8_t     axis_count = 3;
    std::string axis_letters = "XYZ";
    bool        reports_inches = false;       // $13
    bool        reports_line_numbers = false; // Ln: available
    bool        reports_homed = false;
    bool        top_bit_realtime = false;     // grblHAL RT+
    bool        auto_report = false;
    bool        home_single_axis = false;
    bool        homing_enabled = false;       // $22
    bool        origin_at_home = false;       // GRBL build option Z
    bool        laser_mode = false;           // $32
    bool        has_mist = false;             // M7
    uint8_t     aux_outputs = 0, aux_inputs = 0;
    bool        settings_enumeration = false; // grblHAL ENUMS
    double      max_spindle_rpm = 1000;       // $30
};

struct FirmwareMessage {
    enum class Kind : uint8_t { Ack, Nack, Status, Alarm, Probe, ParserState, Parameter, Setting, BuildInfo,
                                Welcome, Feedback, Enumeration, Text } kind;
    uint32_t    ticket = 0;      // Ack / Nack: the line it answers
    int         code = 0;        // Nack / Alarm
    StatusReport status;         // Status
    AxisArray<double> position;  // Probe (machine), Parameter
    bool        flag = false;    // Probe: contact
    std::string name, text;      // Parameter "G54", Setting "100", Feedback text, raw line
};

enum class LineKind : uint8_t { Streamed, SendResponse };   // SendResponse: sent only when nothing is unacknowledged

class IFirmwareProtocol {
public:
    virtual bool connect(IByteStream& stream, const BoardProfile& profile, std::string* error) = 0;
    virtual void disconnect() = 0;
    virtual const FirmwareProfile& profile() const = 0;
    virtual const GcodeDialect& dialect() const = 0;
    virtual void poll(std::vector<FirmwareMessage>& out) = 0;       // core tick: drain the reader queue
    virtual bool canSend(size_t bytes, LineKind kind) const = 0;    // flow-control window
    virtual uint32_t sendLine(std::string_view line, LineKind kind) = 0;   // returns the ticket
    virtual void realtime(RealtimeCommand c) = 0;                   // any thread
    virtual size_t unacknowledged() const = 0;
    virtual void forgetUnacknowledged() = 0;                        // after a reset
};

}
```

`RealtimeCommand`: `StatusReport`, `FullStatusReport`, `FeedHold`, `CycleStart`, `SoftReset`, `SafetyDoor`,
`JogCancel`, `FeedOvReset / Plus10 / Minus10 / Plus1 / Minus1`, `RapidOv100 / 50 / 25`,
`SpindleOvReset / Plus10 / Minus10 / Plus1 / Minus1`, `SpindleStopToggle`, `FloodToggle`, `MistToggle`.

Byte values (GRBL 1.1 command reference):

| Command | Byte | Command | Byte |
|---|---|---|---|
| Status report | `?` | Rapid override 100 / 50 / 25 % | `0x95` / `0x96` / `0x97` |
| Feed hold | `!` | Spindle override 100 % | `0x99` |
| Cycle start / resume | `~` | Spindle override +10 / −10 | `0x9A` / `0x9B` |
| Soft reset | `0x18` | Spindle override +1 / −1 | `0x9C` / `0x9D` |
| Safety door | `0x84` | Spindle stop toggle (in Hold) | `0x9E` |
| Jog cancel | `0x85` | Flood toggle | `0xA0` |
| Feed override 100 % | `0x90` | Mist toggle | `0xA1` |
| Feed override +10 / −10 | `0x91` / `0x92` | grblHAL: status / start / hold | `0x80` / `0x81` / `0x82` |
| Feed override +1 / −1 | `0x93` / `0x94` | grblHAL: full status report | `0x87` |

Feed and spindle overrides are limited to 10–200 %. Rapid override has only the three steps.

## 4. Connecting

### 4.1 Board profile keys

```ini
[Board.grbl]              ; also [Board.grblhal], [Board.fluidnc]
transport = serial        ; serial | tcp
address = COM5            ; or 192.168.1.20:23
baud = 115200
dtr_on_connect = 1        ; defaults: grbl 1, grblhal 1, fluidnc 0
rts_on_connect = 0        ; defaults: 0 for all
connect_wait_ms = 2500    ; defaults: grbl 2500, grblhal 600, fluidnc 600
status_interval_ms = 200  ; 100 while moving
spindle_driven_by = board ; board | app_vfd
spinup_dwell_s = 0.0      ; dwell after M3 when the board drives the spindle
```

### 4.2 Sequence

1. **Open** the stream with the control lines from the profile.
   - An Arduino resets and answers after its bootloader, 1–2 s later.
   - An ESP32 must not be pulled into its bootloader, so FluidNC opens with DTR and RTS off.
2. **Listen** for `connect_wait_ms`.
   - A welcome line identifies the firmware.
   - Status reports arriving unasked mean another sender or a pendant is active (grblHAL reports `MPG:1`); the
     app reports that and does not take control.
3. **No welcome line:** send an empty line, then `$I`. No answer within 2 s gives "No GRBL-compatible board
   answered on COM5". The app never sends a soft reset to find a board, because the machine may be moving.
4. **Identify:** parse `[VER:]`, `[OPT:]` and, on grblHAL, `[NEWOPT:]`, `[AXS:]`, `[AUX IO:]`,
   `[FIRMWARE:]`. Build the `FirmwareProfile`.
5. **State:** send `?`. In `Alarm` the next steps that need `Idle` are skipped; the app shows the alarm with
   **Unlock** and **Home**.
6. **Settings:** `$$` (GRBL, grblHAL), or `$S` and `$CD` (FluidNC). grblHAL with `ENUMS`: `$EG`, `$ES`, `$EA`,
   `$EE`, cached per version string so later connects skip them.
7. **Frame and modes:** `$#` and `$G` (section 6).
8. **Polling** starts. Capabilities are published to the core.

After connecting the app is in its E-stop state, as with the RNR board: nothing but status requests is sent until
the user presses **Reset E-stop**. Reset sends `$X` if the firmware is in `Alarm`, and prompts for homing if the
firmware requires it.

### 4.3 Reading `[OPT:]` on GRBL 1.1

The option letters tell the app what the build can do (interface documentation; to be re-checked when implementing):

| Letter | Meaning | Used for |
|---|---|---|
| `V` | Variable spindle | Spindle speed and laser power available |
| `N` | Line numbers | `Ln:` in status reports: exact line tracking |
| `M` | Mist coolant M7 | Mist output exists |
| `Z` | Homing sets the origin at the home position | Machine envelope (section 11.4) |
| `H` | Single-axis homing commands | `$HX`, `$HY`, `$HZ` |
| `C` | CoreXY | Shown in diagnostics only |
| `+` | Safety door input | Door state can occur |
| `2` | Dual-axis motors | Shown in diagnostics only |

The two numbers after the letters are the planner block count and the RX buffer size.

### 4.4 Unexpected events on the connection

| Event | Meaning | Action |
|---|---|---|
| A welcome line during a session, not caused by the app's own reset (8.3) | The board reset (power, reset button, brown-out) | `PositionLost`; forget unacknowledged lines; fault "The board restarted"; re-run steps 4–7 |
| Nothing received for 2 s while polling | Link or board dead | Fault "The board stopped responding". Not applied during quiet periods (5.4) |
| Write or read error | Cable unplugged | Fault "Connection lost", state `Offline` |

If the link dies during a job, the firmware still runs what it has buffered (up to its planner and RX buffer) and
then stops with the spindle on. The user guide must say that the hard-wired E-stop is the safety device.

## 5. Protocol layer

### 5.1 Reading

A reader thread splits the stream at `\n` (dropping `\r`) and classifies each line:

| Line starts with | Message |
|---|---|
| `ok` | `Ack` for the oldest unacknowledged line |
| `error:` | `Nack` with the code, for the oldest unacknowledged line |
| `<` | `Status` |
| `ALARM:` | `Alarm` with the code |
| `[PRB:` | `Probe`: position and contact flag |
| `[GC:` | `ParserState` |
| `[G54:` … `[G59:`, `[G28:`, `[G30:`, `[G92:`, `[TLO:`, grblHAL `[G59.1:`…, `[HOME:`, `[TLR:` | `Parameter` |
| `$` + digits + `=` | `Setting` |
| `[VER:`, `[OPT:`, `[NEWOPT:`, `[AXS:`, `[AUX IO:`, `[FIRMWARE:`, `[DRIVER`, `[BOARD:`, `[PLUGIN:` | `BuildInfo` |
| `[SETTING:`, `[SETTINGGROUP:`, `[ALARMCODE:`, `[ERRORCODE:` | `Enumeration` |
| `[MSG:` | `Feedback` |
| `Grbl ` or `GrblHAL ` | `Welcome` |
| anything else | `Text` (console only) |

Rules:
- Unknown fields, tags and lines are ignored, never treated as errors.
- A missing closing `]` or `>` is tolerated.
- Every line also goes to the protocol log.

### 5.2 Status reports

```
<Idle|MPos:0.000,0.000,0.000|FS:0,0|WCO:0.000,0.000,0.000>
<Run|WPos:12.500,3.000,-1.000|Bf:12,96|FS:800,12000|Ov:100,100,100|A:SF|Pn:XP|Ln:214>
<Hold:0|MPos:...>        <Door:1|...>        <Alarm|...>
```

| Field | Use |
|---|---|
| State (and `:n` sub-state) | Section 5.3 |
| `MPos:` or `WPos:` | Machine position. With `WPos`, machine = `WPos` + the last `WCO`. Inches when `$13=1`: converted |
| `WCO:` | Firmware frame offset (section 6). Sent only every few reports and when it changes |
| `FS:` or `F:` | Live feed (and spindle speed) |
| `Ov:` | Feed, rapid and spindle override in percent |
| `Pn:` | Active inputs by letter: `X` `Y` `Z` limits, `P` probe, `D` door, `H` hold, `R` reset, `S` start; grblHAL adds `A` `B` `C`, `E` E-stop and others; FluidNC adds `T` tool setter |
| `A:` | Outputs: `S` spindle CW, `C` spindle CCW, `F` flood, `M` mist |
| `Bf:` | Free planner blocks and RX bytes. Diagnostics only, never used for flow control |
| `Ln:` | Line number being executed, when the firmware reports it |
| grblHAL `H:` | Homed, with an axis mask |
| grblHAL `MPG:`, `In:`, `WCS:`, `TLR:` | Pendant active; M66 result; active WCS; tool reference set |

### 5.3 States

| Firmware state | `PathState` | Shown as |
|---|---|---|
| `Idle` | `Idle` | Idle |
| `Run` | `Moving` | Running, probing or a single move, according to what the core asked for |
| `Jog` | `Moving` | Jogging |
| `Hold:1` | `Holding` | Feed hold (stopping) |
| `Hold:0` | `Held` | Feed hold |
| `Home` | `Homing` | Homing |
| `Alarm` | `Fault` | Fault with the alarm text; **Unlock** and **Home** offered |
| `Door:0`–`3` | `Held` | "Door open" / "Door closed, press Cycle Start"; resumes with `~` |
| `Check` | `Fault` | "The board is in check mode"; the app offers to leave it (`$C`, which resets the board) |
| `Sleep` | `Fault` | "The board is asleep"; reset needed |
| `Tool` (grblHAL) | `Held` | Should not occur, because the app never sends M6. Shown with the firmware's message |
| unknown | unchanged | Logged |

### 5.4 Writing and flow control

- **Lines** end with `\n`, contain no spaces, and never exceed `max_line_chars`.
- **One write per line**, under the transport's write mutex. Real-time bytes are written only between lines.
  grblHAL treats `?`, `!` and `~` as text while it is reading a `$` line, so with `RT+` the top-bit versions are
  used.
- **Character counting:** the protocol keeps the lengths of unacknowledged lines. A streamed line is sent when
  `sum + length <= rx_buffer_bytes - 1`. Each `ok` or `error` removes the oldest.
- **Send-response** (the line is sent only when nothing is unacknowledged, and nothing follows until it is
  answered) is used for:
  - every `$` command and setting write;
  - the job preamble;
  - sync-point commands;
  - single moves and probing.

  GRBL 1.1 stops receiving while it writes its EEPROM, so setting writes must never be streamed.
- **Quiet periods:** during `$H` the firmware answers nothing until the cycle ends. The silence watchdog is
  suspended from sending `$H` until its `ok` or an alarm, with its own timeout (default 120 s).
- **Polling:** `?` every `status_interval_ms` (200 ms), 100 ms while moving. Automatic reports (grblHAL, FluidNC)
  replace polling in a later phase.

## 6. Coordinates: the app resolves everything

The firmware has its own work offsets (G54–G59, G92, tool length). The app does not use them and does not change
them. It treats the firmware's current offset as a constant it has to subtract.

- **Firmware frame offset `F`:** the `WCO` value (the firmware's active work offset + G92 + tool length). The app
  takes it from `$#` while idle and follows `WCO:` in status reports.
- **Every coordinate sent** is `app machine target − F`, where the machine target comes from `SegmentResolver`
  (work offset, tool length, G53 / G28, G68, height map already applied).
- **The job preamble** puts the firmware into a known state, so `F` cannot move during a job:

```
G21G90G94G17
G54
G49
G92.1
M5M9
$#          (answer gives F = the firmware's G54 offset)
```

- **Never sent:** G10, G92, G43.1, G28 / G30, G20, G91, G55–G59. The firmware's stored offsets and EEPROM are
  left alone.
- **Console:** after any command typed in the console, the path re-reads `$G` and `$#`. If a typed command changed
  the modes (G91, G20), the next job preamble restores them.

Why this way:
- One implementation of offsets, tool lengths and rotations serves all boards, with identical results.
- No EEPROM wear and no two-way synchronisation with its conflict cases.
- A firmware reset (which clears G92 and the tool length offset) cannot shift the work zero.

Consequences:
- A pendant or web UI attached to the board shows the firmware's work position, which differs from the app's.
- An optional later feature, "Mirror the work zero to the board", can write the app's offset with `G10 L2 P1`
  while idle. `F` then changes and is re-read; nothing else in the design changes.

**Machines without home switches.** GRBL starts with machine position 0 wherever it is powered up. The app keeps
its existing behaviour ("machine coordinates continue from the position saved at the last exit") with a
**machine shift** `S`: app machine position = firmware `MPos` + `S`. On connect to an unhomed board with
*Require homing* off, `S` is set so the app position equals the saved one. After homing, `S` is 0. Coordinates
sent are then `app machine target − S − F`.

## 7. G-code emitter

`GcodeEmitter` turns resolved moves into lines, driven by a `GcodeDialect`.

```cpp
struct GcodeDialect {
    size_t max_line_chars = 80;
    int    xyz_decimals = 3, ijk_decimals = 4, feed_decimals = 0;
    bool   line_numbers = false;            // emit N words (when the firmware reports Ln:)
    bool   arcs = true;
    bool   helical_arcs = true;
    bool   mist = false;                    // M7
    bool   aux_outputs = false;             // M62-M65
    std::string wait_idle = "G4P0";         // answered only after all queued motion has finished
    std::vector<std::string> preamble;      // section 6
    std::vector<std::string> sanitize;      // flavour extras, e.g. grblHAL "G40", "G50", "G97"
    double arc_radius_tolerance_mm = 0.002; // above this the arc is sent as chords
};
```

### 7.1 Rules

| Resolved item | Lines |
|---|---|
| Rapid | `G0X..Y..Z..` |
| Feed move | `G1X..Y..Z..F..` |
| Arc | `G2` / `G3` with `X Y Z` and `I J K`; a plane change (`G18`, `G19`) on its own line first, `G17` restored after |
| Dwell | `G4P<seconds>` |
| Spindle on, board-driven | `M3S<rpm>` or `M4S<rpm>`, then `G4P<spinup_dwell_s>` when set |
| Spindle speed change | `S<rpm>` |
| Spindle off | `M5` |
| Laser power (BreakoutCut) | `S<value>` on the move line |
| Coolant | `M8`, `M7`, `M9`. `M9` switches both off, so the one that stays on is sent again after it |
| Extra output inside a job | `M62P<n>` / `M63P<n>` (synchronised with motion), where the dialect has them |
| Extra output from the UI or Lua | `M64P<n>` / `M65P<n>` (immediate) |
| Tool change, program stop, macro, app-VFD spindle change | Not emitted: a sync point (section 8.2) |
| Program end | Not emitted. The runner waits for idle, then sends `M5M9` if outputs are still on |

- **Modal output:** the emitter remembers what it sent (motion mode, feed, spindle, coolant, plane) and writes
  only what changes. Unchanged axis words are left out. After a sync point, a hold followed by manual moves, or
  a resume, the memory is cleared and the next line states everything.
- **Numbers:** fixed decimals from the dialect, trailing zeros removed, no exponent form, `-0` written as `0`.
- **Line numbers:** when the firmware reports `Ln:`, each motion line starts with `N<n>`, n counting from 1 and
  wrapping below GRBL's limit. The path tracker maps n back to the source line.
- **Never emitted:** G10, G20, G28, G30, G43, G53, G54–G59 (after the preamble), G61, G64, G68, G81–G83, G90 /
  G91 changes, G92, M0, M1, M2, M6, M30, T words. The parser and `SegmentResolver` have already turned them into
  plain moves, sync points or nothing.

### 7.2 Arcs

Today the parser stores a G2 / G3 only as chords. It gains arc records:

```cpp
struct ArcInfo {                       // one per G2 / G3 block, in ParseResult::arcs
    std::array<double, 3> center_mm;   // work coordinates, after G68
    uint8_t plane;                     // 17, 18, 19
    bool    clockwise;
    uint32_t first_segment, segment_count;   // its chords in ParseResult::segments
};
// ToolpathSegment gains: uint32_t arc_index = kNoArc;
```

The emitter writes one arc for a run of chords when all of these hold; otherwise it writes the chords as `G1`:
- the run is complete (not started in the middle by Run From Line or a resume);
- no height map is active (warped chords no longer lie on a circle);
- the radius at the rounded start and end differs by no more than `arc_radius_tolerance_mm` (GRBL rejects larger
  differences with `error:33`);
- the laser Z lock or another rule has not changed individual chords.

Host-planned boards ignore `ArcInfo`; their output does not change.

### 7.3 Example

```
Program (work offsets X100 Y50 Z-20, tool 2 is 12.5 mm long)       Sent (F = 0)
G0 X0 Y0 Z5                                                         G0X100Y50Z-2.5
M3 S12000                                                           M3S12000
G1 Z-1 F200                                                         G1Z-8.5F200
G2 X10 Y0 I5 J0 F800                                                G2X110I5J0F800
M6 T3                                                               (sync point: G4P0, then m6.lua)
G81 X20 Y0 Z-3 R2 F150                                              G0X120  G0Z-6  G1Z-11F150  G0Z-6
M30                                                                 (wait idle)  M5M9
```

### 7.4 Throughput

At 115200 baud about 11,500 characters per second arrive at the board, and an 8-bit GRBL parses at most a few
hundred short lines per second. Long runs of very short `G1` chords at high feed (height-mapped arcs, dense CAM
output) can starve its planner, which shows as stuttering. Mitigations, in this order:
- arcs sent as arcs;
- words that do not change left out;
- an optional emitter setting that merges collinear chords within a tolerance. It is off by default and listed in
  the program check when on.

## 8. Jobs

### 8.1 Runner states

```
Idle -> Preamble -> Streaming <-> SyncWait -> SyncAction
                        |            (continueJob)
                        v
                    Draining -> Finished
Streaming / SyncWait -> Holding -> Held -> Streaming (resume)
any -> Stopping (hold, wait, reset, resync) -> Idle
any -> Faulted
```

- **Preamble:** the lines of section 6 in send-response mode, then the approach moves from `job_start_plan`
  (start lift to safe Z, Run From Line), then the program.
- **Streaming:** the runner pulls resolved moves, emits lines and sends while the window has room. It keeps a ring
  of sent moves for the path tracker.
- **Draining:** everything is sent; the runner sends `wait_idle` and finishes on its `ok` with state `Idle`.

### 8.2 Sync points

At a tool change, a program stop (M0 / M1), a macro (M100–M999), an app-VFD spindle change, or an output change
the board cannot do itself:
1. stop sending;
2. send `G4P0` in send-response mode: its `ok` arrives only when all queued motion has finished;
3. confirm `Idle` in the next status report;
4. emit `JobSyncPoint` to the core, which does the action (runs `m6.lua`, switches the VFD and waits for speed,
   shows the M0 message);
5. on `continueJob()`, clear the emitter's modal memory and continue.

During a sync point the firmware is idle, so scripts can jog, probe and move as usual.

### 8.3 Hold, resume, stop, E-stop

| Action | Sequence | Result |
|---|---|---|
| Feed hold | `!`; no new lines | `Hold:1`, then `Hold:0`. Spindle and coolant stay on |
| Resume | `~`; streaming continues | `Run` |
| Stop (running or held) | `!`; wait for `Hold:0` (10 s timeout); `0x18`; wait for the welcome line and `Idle`; forget unacknowledged lines; `$G`, `$#` | Position kept. The reset also stops spindle and coolant and clears the firmware's modes |
| Stop (idle, at a sync point) | Forget the job | Nothing sent |
| Software E-stop | `0x18` at once, from the calling thread; app VFD stopped | If the machine was moving: `ALARM:3`, position lost, axes marked not homed |
| Reset E-stop | `$X` when in `Alarm`; prompt to home if the axes were homed before | Idle |

- Jogging is not possible while the firmware is in `Hold`. The core disables jog controls in feed hold on these
  boards (capability `jog_while_held = false`).
- The reset in the stop sequence is safe only after `Hold:0`; a reset while still decelerating loses the position.
  If `Hold:0` does not arrive in time, the reset is sent anyway and the result is treated as an E-stop.

### 8.4 Errors and alarms during a job

| Event | Action |
|---|---|
| `error:N` for a streamed line | `!` at once; the job faults with the source line, the line sent and the error text; then the stop sequence. Lines sent after the bad one may run until the hold takes effect |
| `ALARM:1` hard limit | Fault; position lost; axes not homed |
| `ALARM:2` soft limit | Fault; position kept; unlock needed |
| `ALARM:3` reset while moving | Fault; position lost |
| Other alarms | Fault with the firmware's text |
| `Door` state | Held with a door message |
| Link lost | Fault (4.4) |

The emitter produces only syntax the dialect allows, and the program check catches travel and capability problems
before the start, so `error:N` during a job should be rare.

### 8.5 Which line is running: the path tracker

The firmware acknowledges a line when it is queued, not when it has run. The tracker finds the real position in
the program:

- **Window:** the sent moves between `lower` and `upper`. `upper` is the last line sent. `lower` is
  `acknowledged − planner_blocks`: every acknowledged line still holds at least one planner block, so older
  lines must have finished.
- **Each status report:** starting at the current index, advance while the machine position has reached the end of
  the move (within 0.05 mm, or the projection onto the move is at its end) or the index is below `lower`. The
  projection onto the current move gives the progress within it.
- **With `Ln:`:** the reported line number selects the move directly; the projection only gives the progress.
- **Result:** `active_segment`, `active_line`, `segment_progress` and `progress_is_estimate` (false with `Ln:`).
  At sync points and at the end the position is exact in both cases.

This keeps the toolpath view (cut and uncut colouring), the highlighted line and Run From Line working on every
firmware, without relying on firmware build options.

### 8.6 Overrides

- **Feed:** the UI target is clamped to 10–200 %. The path compares it with the last `Ov:` value and sends `0x91`
  / `0x92` for steps of 10 and `0x93` / `0x94` for steps of 1 (`0x90` for exactly 100), a few bytes per tick. It
  tracks the expected value and corrects it with each `Ov:` report.
- **Rapid:** three buttons, 100 / 50 / 25 %.
- **Spindle:** `0x99`–`0x9D`, only when the board drives the spindle. The laser power override in BreakoutCut uses
  the same bytes.

## 9. Manual control

### 9.1 Single moves and MDI

MDI lines are parsed by the app as today, so Mach3-style commands work. Each resulting move is sent in
send-response mode, followed by `G4P0`; the `ok` of the `G4P0` with state `Idle` is the completion event. The
same mechanism serves go-to-zero, tool change retracts, laser framing and the moves inside probing routines.

### 9.2 Jogging

GRBL jogs with `$J=` lines, which do not change the parser's modes, need an F word each time, and are cancelled
with `0x85` (the firmware decelerates and drops queued jogs).

**Step jog:** one line, for example `$J=G91G21X0.1F600`.

**Continuous jog:** the jog feeder keeps just enough motion queued to move smoothly, so the machine stops by
itself if the PC stops sending.

- **Inputs:** the velocity vector from the core (refreshed while the control is held, the existing dead-man), the
  axis accelerations from the firmware settings, and the machine position from status reports.
- **Values:** speed `s`; direction `u`; acceleration along the path `a`; stopping distance `d = s² / (2a)`.
- **Target lead:** `L = d + s × 0.10 s`. The lead is the distance sent in this jog minus the distance travelled
  (from `MPos`, extrapolated with the reported feed between reports).
- **Feeding:** while the lead is below `L`, send `$J=G91G21X..Y..Z..F<s×60>` for a chunk of `s × 0.05 s` along
  `u`. Chunks are clipped to the machine envelope when the axes are homed.
- **Speed change:** the next chunks carry the new feed; the firmware blends them.
- **Direction change:** `0x85`, wait for `Idle`, start again with the new vector.
- **Release, or no refresh for `kJogDeadmanMs`:** `0x85`; ignore input until `Idle`; then report `JogStopped`.
- **If the PC or the link dies:** the firmware finishes the queued chunks and decelerates to a stop. The overrun
  is at most `L`.
- **`error:15`** (travel exceeded, with firmware soft limits on): stop feeding and show "soft limit".

The two time constants (0.10 s and 0.05 s) are starting values, to be tuned on the bench for smoothness and stop
distance. Status polling runs at 100 ms while jogging.

### 9.3 Homing and unlocking

- **Home All:** `$H` in send-response mode with the quiet-period rule (5.4). Its `ok` means success: all axes are
  marked homed, and `?` and `$#` refresh the position and `F`. An alarm (6–9 on GRBL 1.1) ends it with the
  alarm's text.
- **Single axis:** `$HX`, `$HY`, `$HZ` where the profile allows it.
- **Homing settings** (speeds, direction, pull-off, order) are the firmware's. The app's own homing settings and
  `home_all.lua` are not used on these boards.
- **Unlock:** `$X`. Axes stay "not homed" when the position was lost.
- **Homed state:** from `H:` on grblHAL. On GRBL 1.1 and FluidNC the app tracks it: set by a successful `$H`,
  cleared by `ALARM:1`, `ALARM:3`, a board restart, or `$X` after a position loss.

### 9.4 Outputs and spindle

- **Idle:** `M3S..` / `M4S..` / `M5`, `M8` / `M7` / `M9`, `M64P<n>` / `M65P<n>` as lines.
- **While a job runs:** coolant with the toggle bytes `0xA0` / `0xA1`, checked against the `A:` field.
- **Spindle driven by the app's Huanyang link:** as today, straight from the PC; nothing is sent to the board.
- **Spindle driven by the board:** `M3` / `M4` / `M5` / `S`; the app's spin-up time becomes a dwell after `M3`.

## 10. Probing

- **Probe move:** `G38.3<axis><target>F<feed>` in send-response mode. `G38.3` is used instead of `G38.2` because
  it does not raise an alarm when nothing is touched.
- **Result:** `[PRB:x,y,z:1]` (contact) or `:0` (none), followed by `ok`. The position is in machine coordinates;
  the app adds the machine shift `S` and converts from inches when `$13=1`.
- **Before the move:** the app refuses to probe while `Pn:` shows `P` (already in contact), as it does today.
  The firmware would otherwise raise `ALARM:4`.
- **After contact** the machine stops a short distance past the contact point. The routines already retract
  from the recorded contact position.
- **Everything above the probe move is shared:** the Lua routines in
  [scripts/mill/probing](../../scripts/mill/probing), edge and hole-center probing, the tool setter, tool length
  measurement and the surface map all run on `startProbe` / `startMove`.
- **Inputs:** the probe is the firmware's probe pin (inverted with `$6`). GRBL 1.1 has one probe input, so a
  touch plate and a tool setter share it. grblHAL and FluidNC can have a separate tool-setter input.

## 11. Settings

### 11.1 GRBL 1.1: `$$`

Read with `$$` while idle. Shown in these groups, with units and descriptions written for BreakoutCNC:

| Group | Settings |
|---|---|
| General | `$0` step pulse (µs), `$1` step idle delay (ms), `$10` status report content, `$13` report in inches |
| Signals | `$2` step invert (axis mask), `$3` direction invert (axis mask), `$4` enable invert, `$5` limit pins invert, `$6` probe pin invert |
| Motion | `$11` junction deviation (mm), `$12` arc tolerance (mm) |
| Limits & homing | `$20` soft limits, `$21` hard limits, `$22` homing cycle, `$23` homing direction invert (axis mask), `$24` homing feed, `$25` homing seek (mm/min), `$26` debounce (ms), `$27` pull-off (mm) |
| Spindle & laser | `$30` max speed (rpm), `$31` min speed (rpm), `$32` laser mode |
| X / Y / Z | `$100`–`$102` steps/mm, `$110`–`$112` max rate (mm/min), `$120`–`$122` acceleration (mm/s²), `$130`–`$132` max travel (mm) |

- **Writing:** `$n=value` in send-response mode, only while idle, one at a time. After each `ok` the panel re-reads
  `$$`. An `error:N` shows the firmware's reason next to the field.
- **Dependencies shown in the panel:** soft limits need homing; laser mode changes how `S` and `M4` behave.
- **Unknown numbers** (forks add their own) are listed under "Other" with their raw value and remain editable.
- **Backup / restore:** a text file of `$n=value` lines. Restore shows the differences first and then replays them.

### 11.2 grblHAL

The same panel, filled by the controller when it reports `ENUMS`:
- `$EG` gives the groups (`[SETTINGGROUP:id|parent|name]`).
- `$ES` gives every setting: `[SETTING:id|group|name|unit|type|format|min|max|reboot|null-allowed]`.
- Types: boolean, bitfield, exclusive bitfield, radio buttons, axis mask, integer, decimal, string, password, IPv4.
- `$SED=<n>` fetches one description on demand.
- A "restart needed" mark comes from the reboot flag.

The lists are cached per firmware version string. Without `ENUMS` the GRBL 1.1 layout is used.

### 11.3 FluidNC

- **Read:** `$CD` dumps the active configuration as YAML; `$S` lists stored (non-YAML) settings. A small
  indentation-based reader builds the tree; no YAML library is needed for this subset.
- **Show:** a tree view (axes, motors, homing, spindles, control inputs, coolant, probe, user outputs, start).
- **Edit:** leaf values with `$/path/to/key=value`, for example `$/axes/x/steps_per_mm=80`.
- **Persist:** writing the running configuration back to the file (to confirm: `$CD=<file name>`).
- **Restart:** many keys (pins, motor drivers) only apply after a restart; the panel says so and offers one.
- **Not in the first version:** uploading and downloading whole files. FluidNC's own web UI does that.

### 11.4 What the app copies for its own use

`IBoardSettings::mirrorInto()` fills the read-only mirror the toolpath view, the program check, the time estimate,
the jog feeder and the simulator use:

| App value | GRBL / grblHAL | FluidNC |
|---|---|---|
| Steps per mm | `$100`–`$102` | `axes/<axis>/steps_per_mm` |
| Max speed (mm/s) | `$110`–`$112` ÷ 60 | `axes/<axis>/max_rate_mm_per_min` ÷ 60 |
| Acceleration | `$120`–`$122` | `axes/<axis>/acceleration_mm_per_sec2` |
| Travel | `$130`–`$132` | `axes/<axis>/max_travel_mm` |
| Firmware soft limits on | `$20` | `axes/<axis>/soft_limits` |
| Homing enabled | `$22` | homing sections present |
| Spindle range | `$30`, `$31` | spindle section |
| Laser mode | `$32` | laser spindle type |
| Reports in inches | `$13` | `report_inches` |

**Machine envelope.** The app needs the machine-coordinate range of each axis for its own travel check:
- GRBL 1.1 without build option `Z`: `[-travel, 0]`.
- GRBL 1.1 with `Z`: `[0, travel]` when the axis homes towards negative (`$23` bit set), otherwise `[-travel, 0]`.
- grblHAL: from its settings and the `[HOME:]` report (to confirm).
- FluidNC: from `homing/mpos_mm`, `homing/positive_direction` and `max_travel_mm` (to confirm).

When the envelope cannot be derived with certainty, the app's travel check gives warnings instead of blocking,
and the firmware's own soft limits remain the enforcing check.

## 12. Capabilities per flavour

| Capability | GRBL 1.1 | grblHAL | FluidNC |
|---|---|---|---|
| `app_owns_motor_tuning` | no | no | no |
| `realtime_process_control` (THC, Wait Arc OK by the app) | no | no | no |
| `board_thc` (THC and Arc OK by the board) | no | with the plasma plugin (`$350` > 0) | no |
| `native_arcs` | yes | yes | yes |
| `exact_stop_mode` (G61 / G64 choice) | no | no | no |
| `precise_line_tracking` | with option `N` | yes | with `use_line_numbers` |
| `homing` | board cycle | board cycle | board cycle |
| `home_single_axis` | with option `H` | when enabled | yes |
| `probe`, `probe_position_latch` | yes | yes | yes |
| `limit_override` | no (unlock, then jog) | no | no |
| `board_soft_limits` | `$20` | yes | per axis |
| Feed override range | 10–200 % | 10–200 % | 10–200 % |
| `rapid_override`, `spindle_override` | yes | yes | yes |
| `board_vfd` | no | with plugin | yes |
| `estop_input` | no | yes | no (to confirm) |
| `estop_keeps_position` | no | no | no |
| `jog_while_held` | no | no | no |
| Custom outputs | none | `aux_out_n` | `user_outputs` |
| `mpg` | no | on some boards (not used at first) | no |
| `settings_model` | numbered | enumerated | YAML |

What the user sees as a result, compared with the RNR board:
- hidden: Motors, the app's homing settings, pin assignment, Limit Override, THC, corner tolerance;
- added: the board's settings panel, the console, **Unlock**, rapid override buttons;
- changed: one **Home All** on plain GRBL 1.1; the feed override slider ends at 200 %; no jogging in feed hold.

## 13. Program check additions

For a firmware board the check adds:
- **errors:** moves on an axis the board lacks; extra outputs on GRBL 1.1; THC or Wait Arc OK programs (unless the
  board has `board_thc`);
- **warnings:** G61 / G64 in the program (the board blends corners its own way); feeds above the board's maximum
  rate (the board will limit them); moves outside the envelope when it is known only approximately;
- **notes:** the number of sync points (each is a full stop); arcs sent as chords and why (height map, tolerance).

In the code (0.5): the G61 / G64 warning, and THC / Wait Arc OK on a board without `board_thc` as a **warning**
(Cycle Start then refuses the job with the reason, and BreakoutCut offers *Switch THC off*). The other items are
open (end of section 17).

## 14. BreakoutCut on GRBL-family boards

- **Laser:** requires `$32=1`. The app checks it and refuses the job with "set $32 = 1 in the board settings"
  (offering to set it from that message is open). FluidNC has no `$32` and is not checked. `M4` is used when
  *Dynamic Laser Power* is on, `M3` otherwise. The app's 0–100 % power, after its min / max duty mapping, becomes `S = percent × $30 / 100`.
- **Rapids** switch the beam off by themselves in laser mode.
- **Test pulse:** `G1F<feed>` (no axis word), `M3S<power>`, `G4P<seconds>`, `M5` (to confirm on hardware).
- **Framing:** rapid or low-power moves around the bounding box through the normal move path.
- **Air assist:** `M8` / `M9` with the pre-flow and post-flow times as dwells.
- **Red pointer, exhaust:** need spare outputs: the mist pin on GRBL 1.1, aux outputs on grblHAL / FluidNC.
- **Interlocks:** only inputs the board reports can be used: the door on GRBL 1.1; more on grblHAL.
- **Plasma:** runs on every GRBL-family board without THC: torch on / off (`M3` / `M5`), pierce delays as `G4`, and
  the app's touch-off and initial height sensing through the normal probing path. THC and Wait Arc OK need either
  real-time control by the app (RNR only) or THC on the board:
  - **grblHAL with the plasma plugin** (`$350` > 0): the board waits for Arc OK after `M3` (with its own timeout and
    retries) and corrects the torch height itself. The app detects it from `[PLUGIN:PLASMA (<mode>) ...]`, `THC` in
    `[NEWOPT:]` and `$350` (capability `board_thc`, `board_thc_mode`), allows THC jobs, shows the board's state from
    the status field `|THC:<volts>,<flags>` (A arc ok, T torch on, E enabled, R active, V / H anti-dive and void lock,
    U / D up / down), lists `$350`-`$369`, `$674`, `$682` in a *Plasma* settings group, and turns `[MSG:arc failed]`
    (the plugin then holds the job) into a message; Cycle Start retries. The path tracker follows such jobs by XY only,
    because the board moves Z.
  - **Other boards:** the program check warns, Cycle Start refuses a job with THC or Wait Arc OK on, and
    *Torch Height Control* offers *Switch THC off*.

## 15. Fault and message texts

The app shows its own wording for the fixed GRBL 1.1 codes, and the controller's text on grblHAL (`$EA`, `$EE`)
and FluidNC (`$A`, `$E`).

| GRBL 1.1 alarm | Meaning | Position | Action offered |
|---|---|---|---|
| 1 | Hard limit triggered | lost | Unlock, jog off the switch, home |
| 2 | Move would leave the machine travel | kept | Unlock |
| 3 | Reset while moving | lost | Unlock, home |
| 4 | Probe was already in contact | kept | Unlock |
| 5 | Probe did not touch (only with `G38.2`) | kept | Unlock |
| 6 | Reset during homing | lost | Home |
| 7 | Door opened during homing | lost | Home |
| 8 | Homing could not clear the switch on pull-off | lost | Check `$27`, wiring |
| 9 | Home switch not found | lost | Check `$130`–`$132`, wiring |

| GRBL 1.1 errors | Handling |
|---|---|
| 1–3, 20–38 (syntax and G-code errors) | Job fault with the line; a bug report hint, because the emitter should never produce them |
| 8 (not idle), 9 (locked) | "The board is busy" / "The board is locked: unlock or home first" |
| 15 (jog travel exceeded) | Jog stops, "soft limit" |
| 5, 10, 12, 17 (setting dependencies) | Shown next to the setting being written |
| 22 (no feed rate) | Job fault; indicates a lost modal state, the preamble is re-sent on the next start |
| 11, 14 (line too long) | Job fault; emitter bug |

## 16. FakeGrbl: the test emulator

An in-process GRBL behind `IByteStream` ([fake_grbl.h](../../src/core/board/grbl/fake_grbl.h)), used by
`board_tests` and by the applications' test mode `--fake-board`, which connects the three board choices to
emulators instead of hardware (hardware access and the VFD link off, its own settings file). In BreakoutCut the
emulated grblHAL board has the plasma plugin (Arc OK after `M3`, `|THC:` status, Z drift during cuts, arc failures on
demand). Arcs (`G2` / `G3`, all three planes, helix) are traced along the arc.

- **Flavours:** GRBL 1.1, grblHAL, FluidNC: welcome line, `$I` output, buffer sizes, extra fields.
- **Buffers:** an RX buffer of the configured size that overflows if the sender miscounts (the test then fails),
  and a planner queue with the configured number of blocks.
- **Motion:** each queued line takes time according to its length and feed, with simple trapezoid timing, on a
  virtual clock the test advances.
- **Answers:** `ok` when a line is queued; `error:N` for configured lines or unsupported words; status reports
  with selectable fields (`MPos` or `WPos`, `WCO` cadence, `Bf`, `Ln`, `Ov`, `Pn`, `A`).
- **Real-time:** hold, resume, reset (with `ALARM:3` if moving), jog cancel, overrides, toggles.
- **Commands:** `$$`, `$n=`, `$#`, `$G`, `$I`, `$H` (silent while "homing"), `$X`, `$J=`, `G4`, `G38.x` with a
  configurable contact point, `G10`, `G92`, `G43.1`.
- **Faults on demand:** stop answering, restart (welcome line), raise an alarm, drop the link.

## 17. Work packages

Each package ends with a Release build, all tests passing, the conformance suite passing for the affected
flavours, and a CHANGELOG entry where users see a change.

### Phase 1: connect and see (M)

1. `grbl_messages`: the parser of section 5.1, with a test file of real captured lines per flavour.
2. `GrblProtocol`: reader thread, write rules, flow control, real-time bytes.
3. `firmware_profile`: detection and capabilities; the three descriptors registered.
4. `FirmwarePlannedPath` skeleton: connect sequence, polling, status to `PathFeedback`, faults of 4.4.
5. UI: board selector with serial port list and baud; console panel; DRO, state and inputs live.
6. `FakeGrbl` with enough behaviour for these tests.

- **Tests:** message parsing; connect to each fake flavour; wrong flavour selected; silent port; restart
  detection; flow control never exceeds the RX size.
- **Bench:** connect to a real board of each available flavour; unplug and reconnect; console commands.

### Phase 2: manual control (M)

1. Single moves and MDI (9.1); `G4P0` completion.
2. Jog feeder (9.2) with on-screen buttons, keyboard and joysticks.
3. Homing, unlock, E-stop, reset E-stop (9.3, 8.3).
4. Outputs and spindle (9.4); overrides (8.6).
5. Machine shift for unhomed machines (section 6).

- **Tests:** jog lead never exceeds `L`; dead-man stop; direction change; step jog; homing quiet period; alarm
  and unlock flows; override stepping.
- **Bench:** jog feel and stop distance at several speeds; pull the USB cable while jogging (the machine must
  stop within the lead distance); home; trip a limit switch.

### Phase 3: jobs (L)

1. Parser `ArcInfo`; `GcodeEmitter` with the three dialects; golden files.
2. Job runner (8.1), sync points (8.2), hold / resume / stop (8.3), error handling (8.4).
3. Path tracker (8.5); toolpath view and program panel use `progress_is_estimate`.
4. Run From Line and resume through `job_start_plan`.
5. Program check additions (section 13).
6. M6 with `m6.lua`, M0 / M1, M100+ macros, app-VFD spindle changes as sync points.

- **Tests:**
  - emitter golden files: the built-in demo programs and a set of Mach3 samples (drilling cycles, G68, tool
    lengths, G53 moves, arcs in three planes, helical arcs);
  - runner scenarios on `FakeGrbl`: hold and resume, stop while running and while held, `error:33` mid-job, an
    alarm mid-job, a restart mid-job, sync points;
  - tracker accuracy with and without `Ln:`.
- **Bench:** complete jobs including a tool change and a macro; feed hold and stop at random points with position
  checks afterwards; Run From Line; a long program with dense short moves.

### Phase 4: probing (M)

1. `startProbe` on `G38.3`; `[PRB:]` handling; inch conversion.
2. All probing routines, tool setter, tool length measurement, surface map on the firmware path.

- **Tests:** hit and miss on `FakeGrbl`; contact position with a non-zero `F` and a machine shift; every Lua
  probing routine against the fake.
- **Bench:** each routine on a real probe; repeatability against the RNR results.

### Phase 5: settings panels (M)

1. `IBoardSettings` implementations: `$$`, enumerated, FluidNC.
2. Generic settings panel; FluidNC config panel; backup / restore.
3. Mirror (11.4) and machine envelope; simulator uses the mirror.
4. Mach3 import hook for GRBL boards: a reviewed list of `$` changes.

- **Tests:** read / write / re-read round trips on the fakes; dependency errors; backup and restore; envelope
  derivation for each option combination.
- **Bench:** change and restore settings on each flavour; confirm the FluidNC persist command.

### Phase 6: grblHAL and FluidNC extras (M)

1. Fourth axis where the board has one.
2. Aux outputs (M62–M65) as custom outputs; `cnc.set_output`.
3. `TcpStream`; telnet connections.
4. Automatic status reports instead of polling.
5. Board-driven VFD option; grblHAL E-stop input and homed state.

- **Bench:** a grblHAL board and a FluidNC board, each over USB and over the network.

### Phase 7: BreakoutCut laser and documentation (M)

1. Laser mode (section 14).
2. USER_GUIDE: choosing a board, connecting, the differences table of section 12, safety notes.
3. GCODE_REFERENCE: a column for what each code becomes on GRBL-family boards.
4. HARDWARE_VALIDATION_CHECKLIST: a section per flavour.

### Not built yet (after 0.5)

- **Phase 3:** emitter golden files (demo programs, Mach3 samples, arcs in three planes, helical arcs); the emitter is
  covered by unit tests and the job scenarios on `FakeGrbl` only.
- **Phase 5:** the settings mirror for the simulator (11.4); the envelope on FluidNC (its `homing/mpos_mm` rules) and
  grblHAL's own origin options; the Mach3 import hook for GRBL boards (a reviewed list of `$` changes); caching the
  grblHAL setting descriptions (`$EG` / `$ES`) per firmware version (4.2), which are read again on every connect.
  Done: the envelope of GRBL 1.1 / grblHAL from `$130`-`$133`, `$23` and option `Z`
  (`FirmwarePlannedPath::travelEnvelope`), used by the program check as a warning.
- **Phase 6:** automatic status reports instead of polling; the board-driven VFD option. Done: the 4th axis (A from
  `[AXS:]` on grblHAL, from the status report on FluidNC; DRO, jog, homing, MDI, jobs; A positions not scaled by
  `$13`; the RNR gantry setting ignored), `TcpStream`, the grblHAL E-stop input and homed state, and the app's four
  custom outputs driving the board's first four aux outputs (`M64` / `M65` by hand and in jobs, so `cnc.set_output`
  works on them). None of it tried on hardware.
- **Phase 7:** a column in [GCODE_REFERENCE.md](../GCODE_REFERENCE.md) for what each code becomes on these boards.
- **Section 13:** the notes (number of sync points, arcs sent as chords). Done: A moves on a 3-axis board (start
  blocker), travel and maximum rates (warnings, with the board's values), custom outputs the board does not have.
- **Section 14:** offering to set `$32 = 1` from the laser refusal.
- **Section 15:** done. The connect sequence reads `$EA` / `$EE` (grblHAL) or `$A` / `$E` (FluidNC); the board's text
  is used for alarms from 10 and errors above 38, BreakoutCNC's own wording below that.
- **Diagnosis:** *Save board report* (Board Console) runs `$I` (`$I+` on grblHAL), `$$`, `$#`, `$G`, `$N` (FluidNC:
  `$CD`, `$S`) and saves the answers with the profile, the capabilities and the recent exchange. It answers most of
  section 18 from the first connection of each board.
- **Optional, later:** mirroring the work zero to the board (`G10 L2 P1`, section 6); MPG (pendant) support on grblHAL.

## 18. To confirm before or during implementation

| # | Item | Needed for | How |
|---|---|---|---|
| 1 | GRBL 1.1 `[OPT:]` letters (`N`, `Z`, `H`, `M`, `+`, `2`) | 4.3 | GRBL interface documentation, real board |
| 2 | grblHAL: whether plain `$I` already contains `[NEWOPT:]` / `[FIRMWARE:]`, or only `$I+` | 4.2 | Real board |
| 3 | grblHAL: automatic report command and its interval setting | 5.4, phase 6 | Wiki, real board |
| 4 | grblHAL and FluidNC: longest accepted line and RX buffer size | 5.4 | Source, real board. Until confirmed: 80 characters and 128 bytes |
| 5 | FluidNC: persisting runtime config changes (`$CD=<file>`) and which keys need a restart | 11.3 | FluidNC wiki, real board |
| 6 | FluidNC: M62–M65 numbering, canned cycles, tool length commands, G61 | 1, 7 | FluidNC wiki, source |
| 7 | FluidNC: config key names used in 11.4 | 11.4 | `$CD` output of a real board |
| 8 | Machine envelope rules on grblHAL and FluidNC | 11.4 | Home a real machine and compare |
| 9 | Laser test pulse sequence at standstill | 14 | Real laser, low power |
| 10 | Whether `G4P0` is answered only after motion ends on all three | 8.2 | Real boards. GRBL 1.1: yes by design (a dwell synchronises the planner) |
| 11 | Serial control-line behaviour of common ESP32 boards | 4.1 | Real boards |
| 12 | Jog feeder constants | 9.2 | Bench tuning |
| 13 | grblHAL plasma plugin: the exact `$I` plugin line (`[PLUGIN:PLASMA (<mode>) v...]`) and `THC` in `[NEWOPT:]` | 14 | Real board with the plugin |
| 14 | grblHAL plasma plugin: the `\|THC:` status field and its flag letters | 14 | Real board, plugin source |
| 15 | grblHAL plasma plugin: a feed hold after `[MSG:arc failed]`, and Cycle Start retrying | 14 | Real board, torch air off |
| 16 | grblHAL plasma plugin: the material M-code (`Plasma_SelectMaterial`, probably M190) if material tables are used | 14 | Plugin source |
| 17 | grblHAL `$EA` / `$EE` line format (`[ALARMCODE:n\|\|text]` assumed) and FluidNC `$A` / `$E` output (`[MSG:n: text]` assumed) | 15 | Board report |
| 18 | grblHAL travel envelope: whether its homing options (force origin, `$22` bits) change GRBL's `[-travel, 0]` rule, and how a rotary A reports `$133` | 11.4 | Board report, home a real machine |
| 19 | FluidNC: the axis count from the number of `MPos:` values, and `$S` output | 4.2 | Board report |

Smaller "to confirm" notes in sections 1 and 12 belong to the same list: FluidNC homed state, E-stop input, canned
cycles, tool length commands and G61; the grblHAL automatic report toggle (`0x8C`); the envelope rules in 11.4; the
laser test pulse in 14.

FluidNC's wiki was not reachable while this plan was written; its items come from the FluidNC source and are the
least certain.

## 19. Sources

- GRBL 1.1: [interface](https://github.com/gnea/grbl/blob/master/doc/markdown/interface.md),
  [commands](https://github.com/gnea/grbl/blob/master/doc/markdown/commands.md),
  [jogging](https://github.com/gnea/grbl/blob/master/doc/markdown/jogging.md),
  [settings](https://github.com/gnea/grbl/blob/master/doc/markdown/settings.md),
  [error codes](https://github.com/gnea/grbl/blob/master/doc/csv/error_codes_en_US.csv)
- grblHAL: [README (supported codes)](https://github.com/grblHAL/core/blob/master/README.md),
  [Report extensions](https://github.com/grblHAL/core/wiki/Report-extensions),
  [For sender developers](https://github.com/grblHAL/core/wiki/For-sender-developers),
  [Tool change modes](https://github.com/grblHAL/core/wiki/Manual,-semi-automatic-and-automatic-tool-change)
- FluidNC source: [ProcessSettings.cpp](https://github.com/bdring/FluidNC/blob/main/FluidNC/src/ProcessSettings.cpp),
  [Report.cpp](https://github.com/bdring/FluidNC/blob/main/FluidNC/src/Report.cpp),
  [RealtimeCmd.cpp](https://github.com/bdring/FluidNC/blob/main/FluidNC/src/RealtimeCmd.cpp),
  [MachineConfig.h](https://github.com/bdring/FluidNC/blob/main/FluidNC/src/Machine/MachineConfig.h)
