# Changelog

All notable changes to BreakoutCNC are listed here. The version number is set once, in `project(VERSION ...)` in
[CMakeLists.txt](CMakeLists.txt); the window titles and the `.exe` file properties pick it up from there.

## Unreleased

### Fixed
- A script cancel or E-stop that arrived in the moment a Lua script was starting (a macro, M6 or homing script) could
  be lost, so the script kept running. The cancel now always reaches the script.

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
