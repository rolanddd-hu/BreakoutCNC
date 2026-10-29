# Board Drivers: Machine Core vs. Motion Board

BreakoutCNC separates the **machine core** (everything a CNC controller does) from the **board driver**
(how one particular motion control card is talked to). Today the only driver is the RNR Eco Motion 2.0 USB
card; the split exists so other boards can be added without touching the machine logic.

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
- `board::createBoardDriver(BoardType)` is the factory for future board selection.

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

**Board-planned boards** (GRBL, grblHAL, FluidNC: the board takes G-code and plans itself) need a second
motion path in the core, because there are no step slices to stream:

- jobs are sent as (pre-processed) G-code lines with the firmware's flow control instead of planned slices;
  height-map warping and tool length offsets are applied to the lines before sending;
- feed hold / resume / overrides / jog / homing / probing map to the firmware's own commands
  (`!`, `~`, override bytes, `$J=`, `$H`, `G38.2`);
- position and inputs come from the firmware's status reports.

The expected shape is a second driver interface (or a motion-model switch in `BoardCapabilities`) and a
G-code streaming job runner next to the slice planner; the UI, work offsets, tool table, scripts and VFD
stay in the shared core.

## Testing

`tests/script_engine_test.cpp` Test 39 runs a full `MachineController` against `FakeBoardDriver` (no USB):
connect, E-Stop reset, jog, outputs, a G-code job and E-Stop all have to arrive at the driver, the slice
budget must respect `max_slices_in_flight`, and the final step position must equal the commanded one. New
drivers should get a similar test with their protocol encoder checked against captured frames.
