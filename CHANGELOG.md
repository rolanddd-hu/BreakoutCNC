# Changelog

All notable changes to BreakoutCNC are listed here. The version number is set once, in `project(VERSION ...)` in
[CMakeLists.txt](CMakeLists.txt); the window titles and the `.exe` file properties pick it up from there.

## Unreleased

### Added
- Program panel: **Go to line** selects and shows a line by its line number or N number.
- The title bar shows **New version: x.y** when GitHub has a newer release. The apps check once at startup, and skip
  the label if there is no internet connection.
- G-code **G68 A\<x\> B\<y\> R\<degrees\> [I]** rotates the program about (A, B), as in Mach3, until **G69**. Before,
  G68 was an "Unsupported G-code" error.
- Program check: after loading, the program panel lists every problem with its line, in a list that scrolls, instead
  of the first three errors (warnings were hidden while there were errors). Click a message to show its line; problem
  lines also get a dot with the message as a tooltip. When there are only warnings, the × in the corner of the card
  clears them until the program is loaded again.
- The checks Cycle Start makes (soft limits with the current work offset, homing for G53 / G28, tool lengths in tool
  table mode) now show right after loading and update when you home, set a work zero or measure a tool. Cycle Start
  used to report only the first problem.
- New warnings: a feed above what the motor settings allow (the move runs slower), a spindle speed outside the VFD's
  range, a cut while the spindle is off (BreakoutMill), A moves while A follows another axis, and an M3 / M4 with no
  M5 after it.

### Fixed
- Program panel: the scrollbar can be dragged on long programs. Its grab was so small that a press next to it paged
  through the program instead of dragging. It is larger now, and a press anywhere on the bar follows the mouse.
- Program panel: while a job runs, the listing no longer jumps back to the active line during a scrollbar drag.

## 0.3.1

### Fixed
- G15, G40 and G50 no longer switch on exact stop. In 0.3, a program with one of these codes (G40 is common in CAM
  program headers) stopped at the end of every move until the next G64, which made 3D programs jerky and much slower,
  and the *Corners & curves* settings had no effect on it.
- A USB write to the board may now take up to 250 ms instead of 50 ms before it counts as failed, so a short pause on
  the PC is less likely to stop a job with *USB WRITE FAILED WHILE STREAMING MOTION*. A software E-stop still only waits
  up to 50 ms behind a stalled write.

## 0.3

### Added
- More color themes, inspired by editor color schemes: Monokai, Mariana and Solarized Dark next to Dark, and Ayu Light
  and Solarized Light next to Light. The title bar has a sun button (light themes) and a moon button (dark themes);
  clicking the active one goes to the next theme of that group. Both choices are saved.
- G-code **G61** (full stop at the end of every move) and **G64** / **G64 P<tolerance>** (constant velocity, with its
  own corner tolerance for the moves that follow) now take effect. Before, they were accepted and ignored.
- *Limits & Homing* > **Corners & curves**: **Corner tolerance** (was fixed at 0.02 mm), **Limit speed on curves**
  and **Curve acceleration** (25-200% of the axis acceleration).
- *Connection* > **Step pulse rate** (25 / 50 / 100 kHz, like the RNR Mach3 plugin), shown while the board is
  connected. Before, the rate could only be changed in the settings file and
  was always 100 kHz unless a Mach3 profile set it.

### Changed
- BreakoutMill: curves made of many short moves are no longer run at full feed: the speed is limited to keep the
  sideways acceleration near the axis limits and the tool close to the path. Curvy programs take longer and are more
  accurate; untick **Limit speed on curves** for the previous behaviour. In BreakoutCut this setting is off by default.
- The A jog pad sits to the left of the X/Y pad (Z stays on the right), so the jog pad is symmetrical.
- Pulse rates are limited to 25, 50 and 100 kHz: other values in old settings or Mach3 profiles (such as 35 or 75 kHz)
  are rounded up to the next of these.

### Fixed
- G82: a P word now applies to all following holes of the cycle. Before, only holes with their own P dwelled.
- G73 and G83 without a Q word now give a warning. They still peck with the last Q (2 mm if there was none).
- The *Dual-Motor Gantry* setting left the text wrap width pushed when A followed another axis, which could make
  ImGui assert or wrap later text wrongly.

## 0.2

### Added
- Jogging the A axis when it is a rotary / independent axis (BreakoutMill): an A pad next to Z, and Home / End as
  A+ / A- like in Mach3 (Ctrl + Home / End for one step). A slaved to another axis has no jog controls and refuses
  A jogs.
- The A motor can now also follow Z (dual-motor Z axis), not only X or Y; a Mach3 profile with A slaved to Z is
  imported as such instead of leaving A independent.
- *Limits & Homing* > **Program start**: the automatic Z lift before a program's first rapid can be turned off, so the
  first G0 runs as written with all axes moving together, and it has its own **Safe Z** height (before, the M6 safe Z
  was used; existing settings keep that value). Run From Line lifts to the same height.

### Fixed
- A script cancel or E-stop that arrived in the moment a Lua script was starting (a macro, M6 or homing script) could
  be lost, so the script kept running. The cancel now always reaches the script.
- G-code written without spaces between words (`G0X1Y2`, `g1x10f500`) is read correctly. Before, `G0X1` was read as
  `G1` with no X word, so the move did not run.

### Project
- Unit tests no longer depend on fixed waits: the multi-axis jog, camera and script cancel tests wait for the actual
  state (and keep continuous jogs refreshed), so they pass on slow or busy CI runners.

## 0.1 - first public release (GPL-3.0-or-later)

First public release. Experimental: BreakoutMill has been tested against the RNR board on the bench, BreakoutCut's
plasma and laser features only in the simulator. Test carefully on your own machine before trusting it with real
work.

### Applications
- **BreakoutMill**: 4-axis mill / router controller (X, Y, Z, A with optional gantry slaving of A to X or Y).
- **BreakoutCut**: plasma and laser cutter controller, switched with PLASMA / LASER in the title bar.
- Built-in simulator: everything works without a board, including the demo programs (a two-tool mill job, a plasma
  gusset bracket and a laser-engraved coaster).

### Motion and machine control
- RNR Eco Motion 2.0 USB motion board support (host-planned 25 ms step slices, sent from a separate writer thread so
  slow USB acknowledgements never stall the UI), behind a board driver layer (`IBoardDriver`) prepared for other
  boards.
- G-code parser and look-ahead trajectory planner, feed hold / resume, feed and spindle overrides, MDI, set next line.
- Work offsets G54-G59, G92, G53, G28 / G30, tool table with tool length offsets (G43) and a per-machine tool length
  mode.
- Homing (optional "Require homing for G53 / G28 / G30" for machines without home switches), soft limits, limit and
  E-stop handling with a limit override for jogging off a switch.
- A software E-stop goes out immediately, ahead of any queued motion.
- Probing: Z touch-off, edge / corner / bore / boss / slot / web routines, and height-map surface probing that warps
  the toolpath to the measured surface.
- Huanyang VFD spindle control over RS-485 Modbus (Windows, Linux and macOS), with speed monitoring and reading the
  VFD's limits.
- Plasma: torch height control (THC) with arc OK, anti-dive and Z limits, initial height sensing, dry run.
- Laser: PWM power set only by the laser power logic, M4 dynamic (speed-proportional) power, framing with a low-power
  beam or red-dot pointer, test pulse, air assist and exhaust, and chiller / door / PSU interlocks on any input.
- Pin assignment ("Inputs & Outputs"): choose the function of each output OUT1-OUT4, use unused outputs and inputs as
  named custom I/O, test outputs while idle.

### Interface
- Position readout with work and machine coordinates, 3D jog pad with continuous and step modes, keyboard jogging
  (several axes at once; hold Ctrl, Cmd on macOS, for single steps in continuous mode) and Esc as E-stop.
- 2D / 3D toolpath view: completed, active and remaining moves, rapids, pierce points, follow tool, per-tool filter,
  adjustable drawing detail; stays smooth with programs of hundreds of thousands of segments.
- Large programs load in the background; open them with Load File, by dropping them onto the window or from the
  command line ("Open with"), with a check for programs made for the other machine type.
- Bottom strip with every output and input and its function; inputs can be clicked to simulate them.
- Dark and light theme (sun / moon button in the title bar), FULL and MINIMAL layouts, full screen.
- Mach3 profile import (motor tuning, homing, limits, slaving, spindle, RNR and Huanyang VFD plugin settings) with a
  5-stage machine test afterwards.
- Camera view with toolpath overlay and an optional MJPEG stream / remote monitoring page.
- Diagnostics report for bug reports (no file paths or serial numbers) and an F12 performance overlay
  (`BREAKOUTCNC_PERF_LOG=<file>` writes it to a file).

### Scripting
- Lua 5.4 (`cnc.*` API) for the M6 tool change, homing, probing, macro buttons (with their own colors, pinnable to
  the control panel) and custom M-codes; common Mach3 script calls are accepted. Built-in guide in the app.

### Platforms and project
- Windows 10 / 11 (x64), the main platform; Linux (x86_64) and macOS 11+ (universal) builds, not yet tried on real
  hardware.
- Settings are saved next to the applications with a backup copy; the apps find their files also when started from
  another folder.
- User guide with screenshots ([docs/USER_GUIDE.md](docs/USER_GUIDE.md)), hardware validation checklist, protocol and
  architecture notes.
- Release packages for all three platforms (`cmake --build build --config Release --target package`); the Windows zip
  runs without the Visual C++ Redistributable.
- GitHub Actions: build and unit tests on Windows, Linux and macOS for every push, startup smoke test on Linux, and a
  release workflow that publishes the packages when a version tag is pushed.
