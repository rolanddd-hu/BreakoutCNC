# Multi-board Support: Architecture and Phase 0 Plan

Status: **partly implemented (0.5 development).** Done: board registry and capabilities (4.3, 4.4), I/O points for
firmware boards (4.5, without moving the RNR pins to roles), settings split with `board` and `[Board.<id>]`
(4.15, RNR pins still in their 0.4 places), transports with protocol log (4.11), the firmware-planned path (4.10),
capability-driven UI gating (4.16), `cnc.board()` (4.17), console and diagnostics (4.18), job approach logic shared
by both paths (`buildJobPrefixLocked`, part of 4.7). Not done: `HostPlannedPath` / moving the slice code out of the
controller (0c), `SimulatedBoard` (0d), `AxisArray` (0b), the RNR role-based pin map (0a step 2).

This document describes how BreakoutMill and BreakoutCut become board-independent, and the refactoring (Phase 0)
that has to happen before the first new board is written. The first new boards are GRBL 1.1, grblHAL and
FluidNC; their protocol, G-code conversion and work packages are in
[GRBL_FAMILY_PLAN.md](GRBL_FAMILY_PLAN.md). Today's core / RNR driver split is described in
[BOARD_DRIVERS.md](BOARD_DRIVERS.md).

The guiding rule: **everything that is expensive to change later is separated in Phase 0**: interfaces, the saved
settings format, the axis and I/O model, the motion pipeline. A later board then only adds files; it does not
change the core, the UI or other boards.

Reading order for implementers: section 3 (where the board assumptions are today), section 4 (target design),
section 6 (Phase 0 work packages), then GRBL_FAMILY_PLAN.md.

## 1. Goals and non-goals

**Goals**
- **Board selector:** choose the board in the Connection section. The choice and each board's settings are saved.
  First release: RNR Eco Motion 2.0, GRBL 1.1, grblHAL, FluidNC, and the simulator.
- **Open for more boards:** adding a board is a driver, a settings panel, tests and one registration.
- **Same programs on every board:** Mach3-style G-code loads unchanged; the app converts it for the selected
  board and the program check warns about what that board cannot do.
- **Capabilities, not board names:** outside board code nothing asks *which* board is connected, only *what it can
  do*.
- **One machine core:** work offsets, tool table, program handling, Lua, probing routines, VFD link, camera and UI
  are shared.

**Non-goals for the first release**
- plasma THC on firmware-planned boards
- MPG handwheels on GRBL boards
- firmware flashing
- more than 4 axes (prepared, not enabled)
- boards that need a real-time host (family C below)

## 2. Board families

| Family | How motion works | Examples | Status |
|---|---|---|---|
| **A. Host-planned, buffered** | The PC plans the path and sends short timed chunks; the board buffers them and makes step pulses | RNR Eco Motion (25 ms chunks of step counts, USB HID); PoKeys Pulse Engine v2 (1 ms slots of step frequencies, 128-slot buffer); Mach3-plugin USB / Ethernet cards with a known protocol | RNR exists; generalised in Phase 0 |
| **B. Firmware-planned** | The PC sends G-code lines; the board plans and executes | GRBL 1.1, grblHAL, FluidNC; later Marlin, RepRapFirmware, Smoothieware, g2core / TinyG | GRBL family first |
| **C. Real-time servo** | The PC must send a position every ~1 ms with hard timing | Mesa cards, Remora | Not feasible on desktop Windows / macOS. Not planned |
| **D. Vendor library** | Reachable only through the maker's DLL / SDK | Cards with a published SDK; SmoothStepper or UC-series only if their makers offer one | A family A or B driver that loads the user-installed library at runtime. Vendor files never enter this repository |

Cheap USB cards with undocumented protocols need the protocol documented first, as was done for RNR in
[PROTOCOL_SPEC.md](../protocol/PROTOCOL_SPEC.md). The architecture keeps such a driver small; it cannot make the
protocol known.

## 3. Where the board assumptions are today

Measured in the current tree (0.4). These are the places Phase 0 has to touch.

| Assumption | Where | Size |
|---|---|---|
| Direct driver calls from the machine core | `board_->` in [machine_controller.cpp](../../src/core/controller/machine_controller.cpp) | 89 calls |
| Slice queue as the core's motion model | `slice_queue_`, `in_flight_seqs_`, `hw_live_steps_` in the same file | 59 + 17 + 8 uses |
| 25 ms period | `kDtSec` / `kTickInterval` in the controller, `IBoardDriver::kSlicePeriodSec`, a `static_assert` joining them, `PlannerOptions::slice_dt_sec` | 22 uses of `kDtSec` |
| Simulator inside the core | `telemetry_.simulator_mode` branches in the controller | 68 |
| Four fixed axes | `std::array<..., 4>` | 193 uses in 23 files |
| 12 inputs / 4 outputs | `kInputCount`, `kPhysicalOutputCount`, `kOutputChannelCount`, pin fields `uint8_t ..._pin` (1..12), `uint16_t inputs_bitmask` | 44 uses in 7 files, plus 15 UI spots that print `IN%d` / `OUT%d` / pulse rate |
| RNR values in the machine settings | `[Machine] pulse_rate_khz`, `gantry_slave`, `estop_pin`; `[Axis_n]` pins and step / dir polarity; `[IO]` output pins; THC, probe and laser interlock pins | [settings_manager.cpp](../../src/core/config/settings_manager.cpp) |
| RNR fields in telemetry | `usb_connected`, `last_ack_seq`, `tick_counter`, `inputs_bitmask`, `physical_outputs_bitmask` | [motion_controller.h](../../src/core/controller/motion_controller.h) |
| Controller built for one board | `core::RnRController controller;` in both `main_*.cpp`; the constructor connects and falls back to the simulator | 2 apps |
| Serial port code private to the VFD | [huanyang_vfd.cpp](../../src/core/spindle/huanyang_vfd.cpp) | 1 class |
| Frame mapping private to the planner | `StreamingPlanner::targetFor`, `outputsFor`, laser Z lock | [trajectory_planner.cpp](../../src/core/planner/trajectory_planner.cpp) |
| Mach3 import targets RNR terminals | [mach3_xml_importer.h](../../src/core/config/mach3_xml_importer.h) | 1 module |

What is already board-neutral and stays as it is: the `IMotionController` interface the UI and Lua use, the
parser, the program check, the work-offset and tool-length logic, the VFD controller, the camera, scripting, and
[io_map.h](../../src/core/io/io_map.h) as the place where output channels are mapped to terminals.

## 4. Target design

### 4.1 Layers

```
 UI (BreakoutMill / BreakoutCut)
   board panels chosen through the board registry
   |  IMotionController                      (unchanged for UI and Lua)
 MachineController                           (shared machine core)
   state machine, telemetry, work offsets, tool table, program + check, probing / tool-length / surface
   routines, scripting, VFD link, soft limits
   |  SegmentResolver                        program segments -> machine-frame moves (shared by both paths)
   |  IMotionPath
   +-- HostPlannedPath      -> IHostPlannedBoard   : RnrBoard, SimulatedBoard, (PoKeysBoard, ...)
   |      look-ahead planner, timed chunks, pacing, real-time process hooks (THC, laser power)
   +-- FirmwarePlannedPath  -> IFirmwareProtocol   : GrblProtocol (+ FirmwareProfile), (MarlinProtocol, ...)
          G-code emitter + GcodeDialect, streaming, sync points, path tracker, jog feeder
                         |
 Transports: IByteStream (SerialPort, TcpStream)   IPacketLink (HidLink, UdpLink)   + discovery
```

### 4.2 Source layout

```
src/core/common/axis_array.h
src/core/board/          board_registry.*  board_capabilities.h  io_model.*  board_feedback.h  board_settings.h
src/core/board/rnr/      rnr_board.*  rnr_protocol.h  rnr_usb_device.*  rnr_settings.*     (moved from board/, hid/)
src/core/board/sim/      simulated_board.*
src/core/board/grbl/     see GRBL_FAMILY_PLAN.md
src/core/transport/      byte_stream.h  packet_link.h  serial_port.*  tcp_stream.*  hid_link.*  discovery.*
src/core/motion/         motion_path.h  host_planned_path.*  firmware_planned_path.*  segment_resolver.*
                         job_start_plan.*  gcode_emitter.*  gcode_dialect.h  path_tracker.*  jog_feeder.*
src/core/planner/        trajectory_planner.*                                            (period becomes a parameter)
src/ui_common/boards/    connection_panel.*  console_panel.*  rnr_panels.*  grbl_settings_panel.*  fluidnc_config_panel.*
tests/                   board_conformance_test.cpp  settings_migration_test.cpp  fakes/  data/
```

### 4.3 Board registry

Replaces `enum BoardType` and `createBoardDriver()`.

```cpp
namespace rnr::board {

enum class Family : uint8_t { HostPlanned, FirmwarePlanned };

struct ConnectionTarget {            // what the user connects to
    std::string transport;           // "hid", "serial", "tcp"
    std::string address;             // HID path / "COM5" / "192.168.1.20:23"
    std::string label;               // shown in the UI: "COM5 - USB-SERIAL CH340"
};

struct BoardDescriptor {
    std::string id;                  // saved in settings: "rnr_eco_motion_2", "grbl", "grblhal", "fluidnc", "simulator"
    std::string display_name;
    Family      family;
    std::vector<std::string> transports;
    std::function<std::vector<ConnectionTarget>()>                       discover;
    std::function<std::unique_ptr<core::IMotionPath>(const BoardProfile&)> create_path;
    std::function<std::unique_ptr<IBoardSettings>()>                     create_settings;
    std::function<BoardProfile()>                                        default_profile;
    std::function<void(const Mach3Profile&, BoardProfile&, ImportReport&)> import_mach3;  // optional
};

class BoardRegistry {
public:
    static BoardRegistry& instance();
    void add(BoardDescriptor d);
    const BoardDescriptor* find(std::string_view id) const;
    const std::vector<BoardDescriptor>& all() const;
};

}
```

- Ids are stable strings. A settings file with an unknown id (board removed, or a newer file) loads with that
  board's section preserved and the simulator selected.
- UI panels are registered in a parallel table in `ui_common` keyed by the same id, so `core` never depends on
  ImGui.
- `MachineController::selectBoard(id, profile)` is allowed only while disconnected and idle. The apps build the
  controller from the saved id; `RnRController` goes away.

### 4.4 Capabilities

One struct describes what a board can do. The core, the UI, the program check and Lua read it; nothing else
decides behaviour.

```cpp
struct BoardCapabilities {
    std::string display_name;
    Family      family;
    bool        accesses_real_hardware = true;

    // Axes
    uint8_t     axis_count = 3;                    // 3 or 4 today
    std::string axis_letters = "XYZ";
    bool        slaved_motor_option = false;       // RNR: A motor follows X / Y / Z, chosen in the app

    // Motion
    bool   app_owns_motor_tuning = true;           // false: steps/mm, speeds, accelerations live in the firmware
    bool   realtime_process_control = false;       // THC Z tracking, per-chunk laser power, Wait Arc OK
    bool   native_arcs = false;
    bool   exact_stop_mode = true;                 // G61 / G64 honoured
    bool   precise_line_tracking = true;
    double chunk_period_s = 0.025;                 // family A
    size_t chunk_buffer_depth = 3;                 // family A
    bool   ack_means_buffered = true;              // family A: an acknowledgement does not mean "executed"
    bool   can_set_machine_position = true;        // false: the board keeps its own machine position
    bool   jog_while_held = true;                  // false: no jogging during a feed hold

    // Homing, probing, limits
    enum class Homing : uint8_t { None, AppDriven, BoardCycle } homing = Homing::AppDriven;
    bool   home_single_axis = true;
    bool   probe = true;
    bool   probe_position_latch = true;
    bool   limit_override = false;                 // jog off an active end-stop
    bool   board_soft_limits = false;

    // Overrides
    float  feed_override_min_pct = 10, feed_override_max_pct = 250;
    bool   rapid_override = false;
    bool   spindle_override = false;

    // Spindle / outputs
    bool   spindle_pwm = true;
    bool   spindle_direction = true;
    bool   board_vfd = false;                      // the board drives a VFD itself
    bool   estop_input = false;                    // the board has / reports an E-stop input
    bool   estop_keeps_position = true;            // false: a software E-stop while moving loses the position
    bool   mpg = false;

    // Settings
    enum class SettingsModel : uint8_t { AppFields, NumberedDollar, Enumerated, YamlConfig } settings_model;

    std::vector<IoPoint> io;                       // see 4.5
};
```

Capabilities can change on connect (a GRBL-family board reports what it was built with), so the core re-reads
them after `connect()` and bumps a revision the UI watches.

### 4.5 I/O model

Today a pin is a number 1..12 or 1..4. The new model separates **machine functions** from **board terminals**.

```cpp
enum class IoKind : uint8_t { DigitalIn, DigitalOut, AnalogIn, PwmOut };

enum class IoRole : uint16_t {              // what the machine needs
    None,
    // inputs
    EStop, Probe, ToolSetter, Door,
    HomeX, HomeY, HomeZ, HomeA, LimitXMinus, LimitXPlus, /* ... per axis ... */
    ArcOk, ThcUp, ThcDown, ChillerFlow, LaserFault, CycleStart, FeedHold,
    // outputs
    SpindleEnable, SpindleDirection, Flood, Mist, RedPointer, AirAssist, Exhaust,
    Custom,
};

struct IoPoint {
    std::string id;           // stable, saved in settings: "IN3", "OUT1", "probe", "aux_out_0"
    std::string label;        // shown in the UI: "IN3", "Probe", "Aux output 0"
    IoKind      kind;
    IoRole      fixed_role = IoRole::None;   // None = the user assigns a role in the app
    bool        invertible_in_app = false;
};
```

- **RNR:** `IN1`..`IN12`, `OUT1`..`OUT4`, all assignable, inputs invertible in the app.
- **GRBL 1.1:** fixed points (`limit_x`, `probe`, `door`, `spindle_en`, `flood`, ...). Nothing assignable.
- **grblHAL / FluidNC:** the fixed points plus `aux_out_n` (M62–M65), usable as custom outputs.

The board profile stores `role -> point id (+ invert)`. `IoMap` (today's [io_map.h](../../src/core/io/io_map.h),
extended) answers `isActive(IoRole)`, `set(IoRole, bool)`, lists free points and resolves custom names. Input and
output states travel as `std::bitset<64>` indexed by the board's `io` list. `kInputCount` and
`kPhysicalOutputCount` disappear from everything except the RNR driver.

### 4.6 Axes

`AxisArray<T>` (a thin `std::array<T, kMaxAxes>` with `kMaxAxes = 4`) replaces the 193 hand-written arrays, and
`core::Axis` gains helpers (`axisLetter`, `forEachAxis(count)`). Phase 0 changes no behaviour. Loops that are
written against `kMaxAxes` or the board's `axis_count` make a later increase to 6 a contained change: planner
loops, per-axis settings sections, DRO rows and the status formats.

### 4.7 Shared motion pipeline: SegmentResolver

Both paths need the same answer to "where does this program segment end, in machine coordinates, and what is
switched on". Today that logic is private to the planner. It moves into `segment_resolver.*`:

- **Inputs:** a `ToolpathSegment`, the `FrameContext` (work offsets, tool lengths, plan workspace) and the mode
  rules (laser Z lock, dry run, ignore M6).
- **Outputs:** machine-frame start and end, feed, outputs, spindle speed, exact-stop and corner settings, and for
  arcs the machine-frame center (new `ArcInfo`, see GRBL_FAMILY_PLAN.md section 7.2).
- **Job start:** `job_start_plan.*` takes over the approach logic of `rebuildPlannedSlicesLocked` (start lift to
  safe Z, run from line, resume in place) and returns the prefix moves. Both paths use it.

The planner consumes resolved moves and produces chunks. The G-code emitter consumes the same resolved moves and
produces lines. Work offsets, tool lengths, G53 / G28, G68 and height maps are therefore applied once, by the app,
for every board.

### 4.8 IMotionPath

The seam inside the core. One instance is active: the selected board's path, or the simulator's.

```cpp
namespace rnr::core {

enum class PathState : uint8_t { Offline, Fault, Idle, Moving, Holding, Held, Homing };

struct PathFeedback {                         // refreshed every tick
    PathState            state = PathState::Offline;
    AxisArray<double>    machine_pos_mm{};
    AxisArray<double>    velocity_mm_min{};
    AxisArray<bool>      homed{};
    bool                 position_trusted = false;   // false after a lost-position event
    std::bitset<64>      inputs, outputs;            // indexed by BoardCapabilities::io
    bool                 estop_active = false, limit_active = false;
    float                feed_override_pct = 100, rapid_override_pct = 100, spindle_override_pct = 100;
    // job progress
    size_t               active_segment = 0, active_line = 0;
    float                segment_progress = 0;
    bool                 progress_is_estimate = false;
};

struct PathEvent {
    enum class Kind : uint8_t { MoveDone, ProbeDone, HomingDone, JobSyncPoint, JobFinished, JogStopped,
                                Fault, PositionLost, CapabilitiesChanged, ConsoleLine } kind;
    bool                 ok = true;
    AxisArray<double>    position_mm{};      // ProbeDone: contact position (machine)
    size_t               segment_index = 0;  // JobSyncPoint
    int                  code = 0;           // Fault: board-neutral FaultCode
    std::string          text;
};

class IMotionPath {
public:
    virtual ~IMotionPath() = default;
    virtual board::BoardCapabilities capabilities() const = 0;

    virtual bool connect(std::string* error) = 0;
    virtual void disconnect() = 0;
    virtual void tick(Clock::time_point now) = 0;          // core lock held, every core tick
    virtual const PathFeedback& feedback() const = 0;
    virtual bool pollEvent(PathEvent& out) = 0;

    // Single moves, machine frame. Completion arrives as an event.
    virtual bool startMove(const MoveRequest& m, std::string* error) = 0;      // linear, feed or rapid
    virtual bool startProbe(const ProbeRequest& p, std::string* error) = 0;    // stop on contact
    virtual bool startHoming(AxisMask axes, std::string* error) = 0;

    // Jog: velocity per axis; the core refreshes it while the control is held (dead-man). All zero = stop.
    virtual void setJogVelocity(const AxisArray<double>& mm_per_s) = 0;
    virtual bool jogStep(Axis axis, double delta_mm, double feed_mm_min, std::string* error) = 0;

    // Jobs
    virtual bool startJob(const JobRequest& job, std::string* error) = 0;      // program, frame, start, prefix
    virtual void continueJob() = 0;                                            // after a sync point
    virtual void hold() = 0;
    virtual void resume() = 0;
    virtual void stop(StopKind kind) = 0;          // Controlled (decelerate, keep position) or Immediate
    virtual void setOverrides(float feed_pct, float rapid_pct, float spindle_pct) = 0;

    // Safety
    virtual void emergencyStop() = 0;              // callable from any thread
    virtual bool clearEmergencyStop(std::string* error) = 0;

    // Outputs and spindle
    virtual void setOutput(IoRole role, bool on, OutputTiming when) = 0;   // Now or AfterQueuedMotion
    virtual void setSpindle(const SpindleCommand& c) = 0;                  // on / off / dir / speed, or laser power

    // Only where capabilities allow it
    virtual void setMachinePosition(const AxisArray<double>& mm) = 0;      // host-planned boards
    virtual void sendConsoleLine(const std::string& line) = 0;             // firmware boards, idle only
};

}
```

Rules:
- **The core owns `ControllerState`.** It derives it from what it asked for (job, jog, probe, script) and the
  path's `PathState`.
- **The path owns motion detail.** The core never sees slices, sequence numbers, G-code lines or `ok`s.
- **Sync points are the only way program actions reach the core.** At a tool change, program stop, macro, or an
  output / spindle change the board cannot synchronise itself, the path brings the machine to rest, emits
  `JobSyncPoint` and waits for `continueJob()`. On the host-planned path this replaces today's `is_tool_change` /
  `is_program_stop` slice markers.
- **Faults are board-neutral.** `FaultCode` covers: link lost, board not responding, E-stop input, hard limit,
  soft limit, probe failure, homing failure, motion data not acknowledged, firmware alarm (with the firmware's
  number and text attached), command rejected.

### 4.9 HostPlannedPath and IHostPlannedBoard

`HostPlannedPath` receives today's slice code from the controller: the job queue and refill, real-time pacing,
MDI-style program runs, feed-hold deceleration and re-planning, continuous jog ramps, the app-driven homing state
machine, probe arming and the settle wait, position feedback validation, the watchdogs.

Generalised for other boards of the family:

```cpp
struct MotionChunk {                     // one period of motion, board-neutral
    AxisArray<int32_t> end_steps;        // absolute step position at the end of the period
    AxisArray<double>  end_pos_mm;
    std::bitset<64>    outputs;
    uint16_t           spindle_duty_per_mille = 0;
};

class IHostPlannedBoard {
public:
    virtual board::BoardCapabilities capabilities() const = 0;   // period, buffer depth, ack semantics
    virtual bool open(const BoardProfile&, std::string* error) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual BoardInfo info() const = 0;
    virtual bool readFeedback(BoardFeedback& out, uint32_t timeout_ms) = 0;   // worker thread, no core lock
    virtual void configure(const BoardProfile&, const MachineProfile&, bool homing) = 0;
    virtual void emergencyStop() = 0;
    virtual void clearEmergencyStop() = 0;
    virtual void cancelMotion() = 0;
    virtual void setOutputs(const std::bitset<64>& outputs, OutputTiming when) = 0;
    virtual void setSpindlePwm(uint16_t duty_per_mille) = 0;
    virtual void setAxisPositions(const AxisArray<int32_t>& steps) = 0;
    virtual void beginStream(StreamKind kind, Axis axis) = 0;    // job / jog / home / probe
    virtual void armHomeSwitchStop(Axis axis) = 0;
    virtual void armProbe(Axis axis) = 0;
    virtual bool sendChunk(const MotionChunk& c, uint32_t* token) = 0;
};
```

- **Period from the board.** The planner's `slice_dt_sec` and the path's pacing use `chunk_period_s`. The core's
  own tick stays 25 ms; a board with a shorter period gets several chunks per tick. The `static_assert` that ties
  the tick to the slice period is removed.
- **Encoding in the driver.** RNR turns `end_steps` into per-chunk deltas and its OUT bits; a frequency-based
  board (PoKeys) turns them into step rates. `StepSlice` becomes private to the RNR driver.
- **Pacing from capabilities.** `chunk_buffer_depth` and `ack_means_buffered` drive the generic version of the
  0.4 pacing (`board_busy_until_`).
- **Real-time process hooks.** THC Z tracking, per-chunk laser power, Wait Arc OK and the spindle-at-speed gate
  stay here, as hooks that run on each chunk before it is sent. They are available only on boards with
  `realtime_process_control`.

### 4.10 FirmwarePlannedPath, IFirmwareProtocol, GcodeDialect

Written once for every firmware-planned board; details and the GRBL implementation are in
GRBL_FAMILY_PLAN.md.

- **`FirmwarePlannedPath`** owns the job streamer, the sync-point state machine, the hold / resume / stop
  sequences, the jog feeder, the path tracker (which program line the machine is on) and the mapping of firmware
  state to `PathFeedback`.
- **`IFirmwareProtocol`** is the wire protocol: connect and detect, send a line with flow control, real-time
  commands, status, probe result, alarms, settings access. `GrblProtocol` implements it for all three GRBL
  flavours; Marlin or g2core would be further classes.
- **`GcodeDialect`** is a data table: which words and codes the firmware accepts, number formats, maximum line
  length, the "wait until motion finished" command, how tool length, dwell, arcs, outputs and spindle are written.
  `GcodeEmitter` is driven by it.

### 4.11 Transports

```cpp
class IByteStream {                       // text protocols
public:
    virtual bool   open(const ConnectionTarget&, const StreamOptions&, std::string* error) = 0;
    virtual void   close() = 0;
    virtual bool   isOpen() const = 0;
    virtual size_t read(uint8_t* buf, size_t max, uint32_t timeout_ms) = 0;   // 0 = timeout
    virtual bool   write(const uint8_t* data, size_t n) = 0;                  // all or nothing
    virtual void   setControlLines(bool dtr, bool rts) {}                     // serial only
};

class IPacketLink {                       // packet protocols (HID reports, UDP datagrams)
public:
    virtual bool open(const ConnectionTarget&, std::string* error) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual bool read(std::vector<uint8_t>& packet, uint32_t timeout_ms) = 0;
    virtual bool write(const uint8_t* data, size_t n) = 0;
};
```

- **`SerialPort`** is the single serial implementation (Windows, Linux, macOS), moved out of the VFD class. It
  gains baud / parity options, DTR / RTS control (needed for ESP32 boards) and port enumeration with USB
  descriptions.
- **`HidLink`** wraps today's `RnrUsbDevice` I/O. `TcpStream` and `UdpLink` are added when a board needs them.
- **Discovery** lists `ConnectionTarget`s per transport: HID by VID / PID, serial ports, typed-in hosts.
- Every transport keeps a **protocol log** (ring buffer of sent / received data with timestamps) that feeds the
  console and the diagnostics report.

### 4.12 Threading and locking

- **Core worker thread** (exists): every 25 ms it takes the core lock, calls `path.tick()`, drains
  `path.pollEvent()` and runs the core state machines.
- **Path reader thread** (new, one per connection): blocking reads on the transport, parsing into messages, pushed
  to a queue the tick drains. It never takes the core lock. For RNR this replaces the read that today happens at
  the top of the worker loop.
- **Writes** go through one mutex per transport. A text line is written in one call, so nothing can be inserted
  into the middle of it.
- **`emergencyStop()`** may be called from any thread and writes straight to the transport, without waiting for
  the tick.
- **Lock order:** core lock, then path state, then transport write mutex. Reader threads take only their queue
  lock.

### 4.13 Simulator as a board

The 68 simulator branches leave the core. `SimulatedBoard` implements `IHostPlannedBoard`: it executes chunks
against a virtual machine and simulates switches, the probe, the E-stop and inputs toggled from the UI.

- **Simulator mode** means: the active path is a `HostPlannedPath` on a `SimulatedBoard`, while the *selected*
  board keeps supplying capabilities and the I/O list, so the same panels show.
- **Motion values:** for a firmware-planned board, the simulator takes steps/mm, speeds and travel from the last
  values read from the firmware (cached in the board profile), or defaults.
- **Simulating a GRBL board:** the `FakeGrbl` test emulator (GRBL_FAMILY_PLAN.md section 16) can later be offered
  as "simulate a GRBL board", which exercises the real streaming path.

### 4.14 Telemetry

`TelemetrySnapshot` keeps its machine-level fields. Board-specific fields change as follows:

| Today | Becomes |
|---|---|
| `usb_connected` | `board_connected` |
| `product_name`, `serial_number` | unchanged, plus `board_id`, `firmware_version` |
| `inputs_bitmask` (12 bits), `simulated_inputs_bitmask` | `inputs` / `simulated_inputs` as `std::bitset<64>` indexed by the I/O list |
| `physical_outputs_bitmask` | `outputs` as `std::bitset<64>` |
| `last_ack_seq`, `tick_counter` | moved into a `board_debug` string for the diagnostics report |
| (new) | `capabilities_revision`, `progress_is_estimate`, `position_trusted`, `rapid_override_pct` |

### 4.15 Saved settings

The file splits into a machine profile and one section group per board. Example:

```ini
[Machine]
board = grbl                      ; selected board id
linear_units = 0
require_homing = 1
; ... units, program start, corner settings, spindle ramp ...

[Axis_0]                          ; app-owned motion values (used by host-planned boards and the simulator)
steps_per_mm = 200.000
max_velocity_mm_s = 100.000
soft_limit_min_mm = -5.000
; ...

[Board.rnr_eco_motion_2]
pulse_rate_khz = 50
slaved_motor = y                  ; was [Machine] gantry_slave

[Board.rnr_eco_motion_2.Pins]     ; role = point id, optional "!" prefix = inverted
estop = IN1
home_x = !IN3
probe = IN5
spindle_enable = OUT1

[Board.rnr_eco_motion_2.Axis_0]
step_active_low = 0
dir_active_low = 0

[Board.grbl]
transport = serial
address = COM5
baud = 115200
dtr_on_connect = 1

[Board.grbl.Mirror]               ; read-only cache of firmware values, for the simulator and offline preview
x_steps_per_mm = 80.000
```

Migration of today's files (done once on load, keeping the existing `.bak`):

| Today | New place |
|---|---|
| (no board key) | `[Machine] board = rnr_eco_motion_2` |
| `[Machine] pulse_rate_khz`, `gantry_slave` | `[Board.rnr_eco_motion_2]` |
| `[Machine] estop_pin`, `estop_invert` | `[Board.rnr_eco_motion_2.Pins] estop` |
| `[Axis_n] home_pin`, `limit_plus_pin`, `limit_minus_pin` and their `_invert` | `[...Pins] home_x`, `limit_x_plus`, ... |
| `[Axis_n] step_active_low`, `dir_active_low` | `[Board.rnr_eco_motion_2.Axis_n]` |
| `[IO] out_channel_n_pin`, custom names and inverts | `[...Pins]` roles and `[...Names]` |
| `[THC] arc_ok_pin`, `thc_up_pin`, `thc_down_pin` (+ invert) | `[...Pins] arc_ok`, `thc_up`, `thc_down` |
| `[Probe] probe_pin`, `probe_invert` | `[...Pins] probe` |
| `[Laser] chiller_flow_pin`, `door_interlock_pin`, `laser_fault_pin` (+ invert) | `[...Pins] chiller_flow`, `door`, `laser_fault` |
| `[Axis_n]` motion values, homing speeds, soft limits | unchanged |
| Everything else | unchanged |

In memory, `MachineConfig` keeps the machine values, and gains `std::string board_id` and
`std::map<std::string, BoardProfile> board_profiles`. A `BoardProfile` is a typed key / value store with the pin
map, so the settings manager can save a profile for a board it does not know.

### 4.16 Settings panels and UI gating

```cpp
class IBoardSettings {                       // core side, no UI
public:
    virtual void attach(core::IMotionPath* path) = 0;        // nullptr while disconnected
    virtual bool refresh(std::string* error) = 0;            // read from the board, where the board holds settings
    virtual const std::vector<SettingItem>& items() const = 0;       // id, group, name, unit, type, limits, value
    virtual bool write(const std::string& id, const std::string& value, std::string* error) = 0;
    virtual bool backup(const std::string& file, std::string* error) = 0;
    virtual bool restore(const std::string& file, std::string* error) = 0;
    virtual void mirrorInto(core::MachineProfile& machine) const = 0;  // values the app needs a copy of
};
```

`IBoardSettingsPanel` (ui_common) renders one board's settings from its `IBoardSettings`. A generic panel that
renders any `SettingItem` list covers GRBL and grblHAL; FluidNC and RNR have their own.

What the UI shows follows capabilities:

| UI element | Shown when |
|---|---|
| Motors section (steps/mm, speeds, accelerations) | `app_owns_motor_tuning` |
| Limits & Homing: switch pins, homing speeds, pull-off | `homing == AppDriven` |
| Home buttons per axis | `home_single_axis`; otherwise one **Home All** |
| Unlock button | firmware boards (`family == FirmwarePlanned`) |
| Limit Override | `limit_override` |
| Inputs & Outputs: assignable terminals | any `IoPoint` without a fixed role |
| Custom outputs, `cnc.set_output` | assignable or aux outputs exist |
| THC, Wait Arc OK, dynamic per-chunk laser power | `realtime_process_control` |
| Rapid override, spindle override | the matching capability |
| Corner tolerance, curve speed limit | `family == HostPlanned` |
| Feed override range | `feed_override_min_pct` / `max_pct` |
| "approximate" mark on the current line | `progress_is_estimate` |
| Console | boards with a text protocol |
| Board settings panel | from the registry, by board id |

### 4.17 Lua API

The API stays the same on every board. Functions that need something the board lacks fail with a clear message
instead of doing nothing:

| Function | Behaviour by capability |
|---|---|
| `cnc.gcode`, `cnc.wait_idle`, moves, `cnc.probe` | Work on every board (through `IMotionController`) |
| `cnc.get_input(n)`, `cnc.get_inputs_mask()` | Index into the board's I/O list; unchanged numbering on RNR |
| `cnc.get_input_by_name`, `cnc.set_output`, `cnc.get_output` | Error if the board has no such point |
| `cnc.is_homed` | From `PathFeedback::homed` |
| new `cnc.board()` | Returns id, name, family and capability flags, so scripts can adapt |

### 4.18 Console and diagnostics

- **Console panel:** shows the protocol log, filterable (hide status polls). A command line is available on
  firmware boards while idle; after any typed command the path re-reads the firmware's modal state and offsets.
- **Diagnostics report:** board id, firmware / version string, capabilities, the last 200 protocol log lines.

## 5. Where today's controller code goes

| Group in `machine_controller.cpp` | Destination |
|---|---|
| `connect`, `disconnect`, `setSimulatorMode` | Core keeps the state handling; opening and closing move to the path; the simulator becomes a path swap |
| `processBoardReportLocked`, watchdogs, position feedback validation | `HostPlannedPath` |
| `sendBoardSliceLocked`, `boardBufferFullLocked`, `prepareProgramRunLocked`, `closeProgramRunLocked` | `HostPlannedPath` |
| `runMotionTickLocked`, `sendQueuedSliceLocked`, `finishQueueLocked`, `refillJobQueueLocked`, `replanFromQueueTailLocked` | `HostPlannedPath` (tool change, program stop and spindle changes become sync-point events to the core) |
| `rebuildPlannedSlicesLocked` | Approach logic to `job_start_plan`; slicing to `HostPlannedPath` |
| `pauseJobLocked`, `beginHoldDecelLocked`, `stopJobLocked` | Core keeps state and output rules; deceleration and re-planning move to the path |
| `jogStartContinuous`, `updateContinuousJogLocked`, `jogStep`, `jogStop` | Core keeps limits and dead-man; ramps and chunks move to the path (`setJogVelocity`) |
| `startNextHomingAxisLocked`, `homingTickLocked` | `HostPlannedPath` (app-driven homing variant) |
| `beginProbeMoveLocked`, `probingTickLocked` | Path: `startProbe` + `ProbeDone` |
| `onProbeMoveFinishedLocked`, `continueHoleCenterLocked`, tool-length and surface probing | Core, rewritten on `startMove` / `startProbe` events |
| `updateSpindleLocked`, laser power, THC | Core keeps the spindle / VFD state; per-chunk parts become hooks in `HostPlannedPath` |
| `evaluateInputsAndEndStopsLocked`, pin evaluation | Core, on `IoMap` roles |
| `setBoardOutputsLocked` and output mapping | Core decides roles; the path applies them |
| Work offsets, tool table, MDI parsing, program load / check, scripting | Core, unchanged |
| Simulator branches | `SimulatedBoard` |

A build check enforces the result: after Phase 0c, `machine_controller.cpp` contains no `board_->`, no
`slice_queue_` and no `simulator_mode` branches.

## 6. Phase 0 work packages

Every package is behaviour-neutral, ends with a Release build and all tests passing, and is checked on the RNR
bench machine before the next one starts. Sizes are relative: S (about a day), M (a few days), L (a week or more).

### 0a. Board model, I/O model, settings split (L)

1. Add `board_capabilities.h`, `io_model.*`, `BoardProfile`, `BoardRegistry`; register RNR with its I/O list.
2. Change pin fields to roles in a `BoardProfile`; extend `io_map` with `IoMap`; switch the controller's pin
   evaluation, the I/O panel, the bottom strip and the Lua I/O functions to it.
3. Split the settings file (4.15) with migration; keep reading the old layout.
4. Telemetry field changes (4.14); update the UI and diagnostics report.
5. Mach3 import writes roles through the RNR descriptor's import hook.

- **Tests:**
  - `settings_migration_test`: saved files from 0.1–0.4 (checked in under `tests/data/settings/`, no personal
    values) load, save and reload identically.
  - `IoMap` unit tests: role lookup, inversion, sanitising.
  - The existing tests pass unchanged.
- **Bench:** every input and output toggles as before; the E-stop input, limits, probe and custom outputs work;
  settings survive a restart.

### 0b. Axes and transports (M)

1. `AxisArray`, axis helpers; mechanical replacement in 23 files.
2. `SerialPort` extracted from the VFD class; VFD uses it.
3. `IByteStream`, `IPacketLink`, `HidLink` around the RNR HID code, discovery, protocol log.

- **Tests:**
  - Compile-time: no `std::array<..., 4>` left outside `axis_array.h` (grep in the test script).
  - A loopback test for the protocol log.
  - `huanyang_vfd_test` passes.
- **Bench:** RNR connects, reconnects after unplugging; VFD reads and runs.

### 0c. Motion path split (L, the risky one)

Done as a strangler: the seam first, then one group at a time.

1. `SegmentResolver` and `job_start_plan` extracted from the planner and `rebuildPlannedSlicesLocked`; the
   planner consumes resolved moves. Planner tests must produce bit-identical slices.
2. `IMotionPath` introduced with a `HostPlannedPath` that at first forwards to the existing controller code.
3. Move groups into the path, in this order, testing after each: board feedback and watchdogs; job queue and
   pacing; hold / stop deceleration; jog; homing; probe moves.
4. Rewrite the core's probing, tool-length, hole-center and surface routines on `startMove` / `startProbe` events.
5. `IHostPlannedBoard` with `MotionChunk`; RNR encodes its own slices; period, buffer depth and ack semantics
   come from capabilities; remove the `static_assert`.
6. Sync-point events replace the tool change / program stop slice markers.

- **Tests:**
  - A recording fake board compares the exact sequence of board calls and chunks before and after, for a set of
    scenarios: job, hold / resume, stop, jog, home, probe, M6, E-stop mid-job.
  - The 0.4 pacing assertion (`maxLeadMs() <= 76`) stays.
- **Bench:** the full [HARDWARE_VALIDATION_CHECKLIST.md](../HARDWARE_VALIDATION_CHECKLIST.md): jog, job, feed
  hold, software E-stop, probe routines, homing, M6, M30 outputs off, no delay behind the app.

### 0d. Simulator as a board, conformance suite (M)

1. `SimulatedBoard`; simulator mode becomes a path swap; delete the simulator branches.
2. Board conformance test suite (section 8), run against the RNR fake and the simulator.
3. `MachineController::selectBoard`; the apps build the controller from the registry; board selector UI skeleton
   (RNR and simulator only at this point).
4. Rewrite BOARD_DRIVERS.md around families and the new interfaces.

- **Tests:** conformance suite green for both boards; `--smoke-test` of both apps.
- **Bench:** simulator behaves as in 0.4 (demo programs, probing in the simulator, simulated inputs).

After Phase 0 the app behaves exactly as 0.4 for RNR users, and is released as such before GRBL work starts.

## 7. Adding a board later

**Family A (host-planned):**
1. Implement `IHostPlannedBoard`: encode `MotionChunk` in the board's format; map outputs, homing and probe
   arming; decode status into `BoardFeedback`.
2. Describe capabilities, the I/O list, chunk period and buffer.
3. Register it with a settings panel for its own values and a `discover()`.
4. Pass the conformance suite with a fake of the board's protocol; add its bench checklist.

**Family B (firmware-planned):**
1. Write a `GcodeDialect` table.
2. GRBL-like firmware: a `FirmwareProfile` variant. Otherwise implement `IFirmwareProtocol`.
3. Register it with a settings panel; pass the conformance suite with a fake; add the bench checklist.

**Family D (vendor library):** as A or B. The driver loads the user-installed library at runtime and tells the
user where to get it when it is missing. The library is not added to the repository or the installers unless its
licence allows redistribution.

## 8. Testing

- **Board conformance suite** (`tests/board_conformance_test.cpp`), parametrised by board, each against its fake:
  - connect, feedback, disconnect, reconnect;
  - E-stop and reset; fault on a silent board;
  - jog start / stop, dead-man timeout, step jog;
  - a job with hold, resume, stop, feed override and a sync point;
  - outputs now and after motion;
  - probe hit and miss; homing; position consistency after every stop.
- **Fakes:** `FakeRnrBoard` (today's `FakeBoardDriver`, recording), `SimulatedBoard`, `FakeGrbl`.
- **Golden files** for the G-code emitter and for the settings migration.
- **Test executables:** `script_engine_test` stays; board tests go into a new `board_tests` executable. AGENTS.md
  is updated to say that every test executable must pass.
- **Bench checklists** per board in HARDWARE_VALIDATION_CHECKLIST.md.

## 9. Decisions

| # | Decision | Reason |
|---|---|---|
| 1 | One `MachineController` with an internal `IMotionPath`, not one controller per family | ~4000 lines of shared machine logic would otherwise be duplicated |
| 2 | Board ids are strings; settings are split into machine profile and board profiles | Users' files must survive new and removed boards and switching between boards |
| 3 | Capabilities decide behaviour everywhere outside board code | Avoids `if (board == ...)` spreading through UI and core |
| 4 | Roles and I/O points replace pin numbers | Boards differ in what terminals exist and which are fixed |
| 5 | The app resolves work offsets, tool lengths, G53 / G28, G68 and height maps for every board | One implementation, identical results on all boards; firmware offsets are never relied on |
| 6 | The simulator is a board | Removes special cases from the core; every feature is tested through the board interface |
| 7 | The core tick stays 25 ms; boards with other chunk periods get several chunks per tick | No timing change for RNR; other periods are a parameter |
| 8 | `kMaxAxes` stays 4 in Phase 0 | Raising it is a separate, visible change (DRO, settings, planner tests) |
| 9 | Phase 0 is released on its own before any GRBL code | Regressions are found on the one board that exists, with nothing else changed |

## 10. Open questions

Each has a default, so implementation is not blocked.

| # | Question | Default if unanswered |
|---|---|---|
| 1 | Which GRBL-family boards are available for bench testing? | Develop against `FakeGrbl`; release each flavour only after a bench pass on real hardware |
| 2 | Is USB serial enough at first, or is WiFi / Ethernet needed early? | USB serial first; TCP in the grblHAL / FluidNC phase |
| 3 | Should BreakoutCut laser on GRBL be in the first GRBL release? | After the mill |
| 4 | Is plasma on grblHAL (its plasma plugin) wanted? | Not planned |
| 5 | Which other boards are likely next? | None assumed; the conformance suite and I/O model are checked against PoKeys' documentation as a second family A example |
| 6 | May the app change firmware settings it needs (for example the status report mask) without asking? | No: it asks once, and works without the change where it can |

## 11. Risks

- **Phase 0c touches safety-relevant code.** Mitigations: strangler order, recorded board-call comparison, the
  full bench checklist, and a separate release.
- **Settings migration.** Mitigations: the `.bak` copy, migration tests from every release, the old layout read
  for one more release cycle.
- **Scope creep in Phase 0.** Section 9, decision 8 and the non-goals keep it to separation only: no new
  features, no new boards.
- **Undocumented card protocols.** Outside the architecture's control; see section 2.
