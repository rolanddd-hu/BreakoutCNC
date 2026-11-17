# Board Drivers: Machine Core vs. Motion Board

BreakoutCNC separates the **machine core** (everything a CNC controller does) from the **board driver**
(how one particular motion control card is talked to). Two board families exist:

- **Host-planned** (the PC plans the motion and streams timed chunks): the RNR Eco Motion 2.0 USB card, through
  `IBoardDriver` (below).
- **Firmware-planned** (the board plans, the PC sends G-code lines): GRBL 1.1, grblHAL and FluidNC, through
  `core::motion::FirmwarePlannedPath` (section *Firmware-planned boards*).

The board is chosen by its registry id (`MachineConfig::board_id`, saved as `[Machine] board`); see
[board_registry.h](../../src/core/board/board_registry.h). The plans behind this are
[MULTI_BOARD_PLAN.md](MULTI_BOARD_PLAN.md) and [GRBL_FAMILY_PLAN.md](GRBL_FAMILY_PLAN.md).

## Layers

```
 BreakoutMill / BreakoutCut UI            (src/app_*, src/ui_common)
          |  IMotionController            (src/core/controller/motion_controller.h)
          v
 MachineController  - machine core        (src/core/controller/machine_controller.*)
   jobs, look-ahead planner, feed hold / overrides, work offsets, tool table, homing & probing
   sequences, soft limits, inputs & end-stops, spindle / Huanyang VFD, THC, laser rules, Lua scripting,
   simulator
          |  IBoardDriver                 (src/core/board/board_driver.h)
          v
 RnrBoardDriver     - RNR Eco Motion 2.0  (src/core/board/rnr_board_driver.*)
          |  RnrUsbDevice + RNR protocol  (src/core/hid/*)
          v
 USB HID board
```

- `RnRController` ([rnr_controller.h](../../src/core/controller/rnr_controller.h)) is a one-line convenience
  class: a `MachineController` constructed with an `RnrBoardDriver`. The applications use it.
- Board-neutral types (`core::Axis`, `core::OutputBit`, `core::StepSlice`) live in
  [board_types.h](../../src/core/board/board_types.h). Nothing outside `src/core/hid` and the RNR driver
  includes the RNR protocol header.
- `board::createBoardDriver(BoardType)` creates host-planned drivers; `board::BoardRegistry` lists every board
  (id, name, family, transports, capabilities, default settings).
- `board::BoardCapabilities` ([board_capabilities.h](../../src/core/board/board_capabilities.h)) says what the
  selected board can do. The UI, the program check and Lua (`cnc.board()`) read it instead of asking which board it
  is: sections such as Motors, Limits & Homing, pin assignment and THC are replaced by a note when the board keeps
  those settings itself.

## The driver contract

Motion model (host-planned): the core plans on the PC and streams fixed 25 ms `StepSlice`s
(`IBoardDriver::kSlicePeriodSec`), each holding signed step counts per axis plus the OUT1..OUT4 state.
It keeps at most `capabilities().max_slices_in_flight` unacknowledged slices on the board and matches
acknowledgements by the sequence number `sendSlice()` returns.

| Call | Meaning | RNR implementation |
|---|---|---|
| `open / close / isOpen / info` | Connection and product / serial name | HID open, device info |
| `readStatus` | Newest feedback: busy, E-Stop / limit fault, last acked sequence, live and latched step positions, raw inputs | 49-byte input report |
| `configure(cfg, homing)` | Pins, polarities, step rate from `MachineConfig` | opcode 0x0F |
| `emergencyStop` / `clearEmergencyStop` | Halt immediately / re-arm after reset | 0x01 / 0x07 + 0x02 |
| `cancelMotion` | Abort the sequence and drop buffered slices | 0x08 |
| `setOutputs` / `setOutputsAfterMotion` | OUT1..OUT4 now / when buffered motion ends | 0x03 / 0x06 |
| `setSpindlePwm` | 0..1000 per mille | 0x0E |
| `setAxisPositions` | Load step counters | 0x09 |
| `beginJog` | Start of a jog / pull-off slice stream | 0x0A |
| `armHomeSwitchStop` / `armProbe` | Next slices stop on the home switch / probe and latch positions | 0x0D / 0x0C |
| `sendSlice(Program or Internal)` | Queue one slice | 0x04 / 0x05 |

The core currently sends every slice as `Internal`, program motion included, in MDI-style runs: `setOutputs` while
the board is at rest, `beginJog`, slices, `setOutputsAfterMotion` when the board has run out of slices. The RNR
board acknowledges a slice once it is buffered, so the core also paces slices in real time: never more than
`max_slices_in_flight` slices (75 ms) of motion ahead of what the board can have run. `configure` is sent again
after `clearEmergencyStop`.

Rules a driver must follow:

- **Safety first.** `emergencyStop()` must stop motion at once and must work even if the board is in an
  unexpected state. A board that stops sending reports is treated as a fault by the core (watchdog: 2 s after
  connect, 500 ms afterwards), so `readStatus()` must deliver reports continuously.
- **Positions.** `live_steps` must be the board's real step position; the core validates it against the
  commanded position when idle and corrects from it after stops.
- **Threading.** `readStatus()` runs on the core's worker thread without the core lock; all other calls are
  made with the lock held. Both can run at the same time, so the driver must serialize its own I/O.
- **Hardware access in tests.** Unit tests call `MachineController::setHardwareAccessEnabled(false)`. A driver
  that talks to real hardware reports `accesses_real_hardware = true` and is then never opened; simulated or
  test drivers report `false`.

`capabilities()` also describes input/output counts, step-rate range and whether the board latches probe /
home positions in hardware. The core does not use all of these yet (pin numbers are still validated as
IN1..IN12); a second board with a different I/O layout should switch that validation to the capabilities.

## Adding a board

**Host-planned boards** (the board executes step chunks the PC sends; e.g. a card like RNR, Remora-style
firmware): implement `IBoardDriver`, add a `BoardType`, return it from `createBoardDriver()`, and construct
`MachineController(createBoardDriver(type))`. If the board needs a different slice period, the core's 25 ms
tick has to become configurable first (a `static_assert` guards the current assumption).

**Firmware-planned boards**: see the next section. A further GRBL-like firmware is a `FirmwareProfile` variant
plus a registry entry; another protocol (Marlin, g2core) would be a second protocol class next to `GrblProtocol`.

## Firmware-planned boards (GRBL 1.1, grblHAL, FluidNC)

```
 MachineController   state machine, work offsets, tool table, probing routines, MDI parsing, VFD, scripts
        |  intents: moves (machine frame), probe, homing, jog velocity, job, hold / resume / stop, outputs
        v
 motion::FirmwarePlannedPath   (src/core/motion/firmware_planned_path.*)
   connect + identify, status polling, single moves (G53), probing (G38.3, incremental), homing ($H),
   jog feeder ($J= chunks, 0x85 cancel), job streamer + sync points, stop sequence, overrides, settings
        |  GcodeEmitter + GcodeDialect (src/core/motion/gcode_emitter.*): lines, arcs as G2/G3, modal memory
        v
 board::grbl::GrblProtocol     (src/core/board/grbl/*): reader thread, character-counting flow control,
        |                      real-time bytes, message parser, FirmwareProfile (flavour detection), codes, settings
        v
 transport::IByteStream        SerialPort (Windows / Linux / macOS), TcpStream; ProtocolLog for the console
```

- **Coordinates**: the app resolves work offsets, tool lengths, G53 / G28, G68 and height maps. Single moves are
  sent as `G53` machine moves, probes and jogs are incremental, and jobs read the board's G54 offset (`$#`) after
  their preamble (`G21G90G94G17`, `G54`, `G49`, `G92.1`, `M5M9`). G10 / G92 are never sent.
  App machine position = firmware machine position + machine shift (used when the machine has no home switches).
- **Sync points**: at M6, M0 / M1 and (with the Huanyang VFD driven by the app) spindle changes, the streamer sends
  `G4P0`, waits until the board is idle and hands control to the core.
- **Which line runs**: a path tracker follows the reported position along the sent moves (or `Ln:` where the board
  reports line numbers).
- **Threading**: everything runs on the core worker tick under the core lock, except the protocol's reader thread
  (only its line queue) and `emergencyStop()` (a soft reset written straight to the port, from any thread).
- **Board texts and report**: the connect sequence reads the board's alarm / error texts (grblHAL `$EA` / `$EE`,
  FluidNC `$A` / `$E`); `startBoardReport()` runs read-only queries and `boardReportText()` assembles them with the
  profile, capabilities and recent exchange (*Save board report*). `travelEnvelope()` gives each axis's travel for
  the program check.
- **Board THC**: grblHAL's plasma plugin is detected from `[PLUGIN:PLASMA (<mode>) ...]` / `THC` in `[NEWOPT:]` and
  `$350`, giving the capability `board_thc`. The board then waits for Arc OK on `M3` and corrects Z itself; the
  status field `|THC:<volts>,<flags>` feeds the THC telemetry, `[MSG:arc failed]` becomes a pause with a message,
  and the path tracker follows plasma jobs by XY only (Z is moved by the board).
- **Simulator**: with a firmware board selected, Simulator Mode uses the host-planned simulator; the board's
  capabilities still decide what the UI shows.

Not done yet: moving the RNR slice code out of `MachineController` into a `HostPlannedPath` (Phase 0c of
[MULTI_BOARD_PLAN.md](MULTI_BOARD_PLAN.md)), the role-based I/O model for the RNR pins, and the GRBL-family items
listed at the end of section 17 of [GRBL_FAMILY_PLAN.md](GRBL_FAMILY_PLAN.md) (4th axis, automatic status reports,
machine envelope, controller fault texts, among others). The emulator test mode exists: `--fake-board` (see Testing).

## Testing

`tests/board_tests.cpp` runs the machine core against `FakeGrbl` ([fake_grbl.h](../../src/core/board/grbl/fake_grbl.h)),
an in-process GRBL / grblHAL / FluidNC emulator with a checked receive buffer: identification, MDI, jogging, homing,
probing, a job with arcs and a tool change, hold / resume / stop / E-stop, alarms, restarts, a silent board, firmware
settings, the console and flow control.

The same emulator drives the applications for manual and scripted UI tests: `BreakoutMill.exe --fake-board`
([fake_board_mode.cpp](../../src/ui_common/fake_board_mode.cpp)) switches hardware access off, uses
`*_settings_fakeboard.ini`, and connects the GRBL / grblHAL / FluidNC choices to emulators. With `--ui-remote` as well,
the app also reads `ui_remote_commands.txt` next to the executable (click, press / release, scroll, type, key, wait,
shot, quit; see [ui_remote.h](../../src/ui_common/ui_remote.h)) and saves screenshots to `ui_remote_shots/`, so a test
can operate the UI without the computer's mouse or keyboard.

`tests/script_engine_test.cpp` Test 39 runs a full `MachineController` against `FakeBoardDriver` (no USB):
connect, E-Stop reset, jog, outputs, a G-code job and E-Stop all have to arrive at the driver, the slice
budget must respect `max_slices_in_flight`, and the final step position must equal the commanded one. New
drivers should get a similar test with their protocol encoder checked against captured frames.
