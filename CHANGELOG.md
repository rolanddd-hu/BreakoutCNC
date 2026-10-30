# Changelog

All notable changes to BreakoutCNC are listed here. The version number is set once, in `project(VERSION ...)` in
[CMakeLists.txt](CMakeLists.txt); the window titles and the `.exe` file properties pick it up from there.

## 0.3 - Linux / macOS downloads, user guide, safety fixes

First release with Linux and macOS downloads, a user guide, and several safety fixes around the E-stop and the
laser output.

### Safety
- A software E-stop no longer waits behind queued motion packets to the board: the queued slices are dropped
  and the E-stop goes out right after the USB write in progress. Before, it could wait up to about 150 ms.
- BreakoutCut: an S word (MDI, program start) or a spindle speed change no longer sets the board's PWM output
  from an RPM value. In laser mode, `S500` used to set a 2 % PWM duty while the laser was off; the PWM output is
  now set only by the laser power logic.
- BreakoutCut ignores spindle spin-up / spin-down delays (for example imported from a Mach3 profile): they would
  have kept the torch / laser output on for the spin-down time after M5.
- The apps always use the settings file, `scripts/` and `resources/` next to the executable, also when started
  from another folder (a Linux / macOS file manager, or "Open with" on Windows). Before, they could start with
  default settings (steps/mm, limits, pins) in that case.

### New
- Linux (x86_64) and macOS (universal, macOS 11+) downloads next to the Windows zip. They are built and tested
  automatically but not yet tried on a real machine: feedback is very welcome.
- User guide with screenshots ([docs/USER_GUIDE.md](docs/USER_GUIDE.md)), also in the download: install and first
  start, the simulator, setting up a new machine, migrating from Mach3, running jobs, probing, plasma / laser and
  troubleshooting.
- Open a G-code file by passing it on the command line ("Open with", double-click on an associated file) or by
  dropping it onto the window. It goes through the same machine-type check as Load File.
- Diagnostics: ticking "Diagnostics" (bottom right) opens a report with the version, OS, graphics, board state
  and key settings, with a "Copy to clipboard" button for bug reports. It contains no file paths or serial
  numbers.
- BreakoutCut has a laser demo program (a 90x90 mm coaster: raster-filled center, rose, rings and compass
  ticks, outline cut). The built-in demo follows the Plasma / Laser mode; a program you loaded stays loaded when
  you switch modes.

### Performance
- Large programs load in the background: the window stays responsive while a file is read and parsed ("Loading
  ..." next to the cycle buttons), including the last file at startup, so the window appears right away. A
  650k-line file froze the window for about 1.3 s on a desktop PC (several seconds on a laptop); the longest
  frame is now about 60 ms. The G-code list no longer copies every line, which saves ~150 ms and tens of MB on
  such files.
- Rotating or zooming the 3D view of very large programs (150k+ segments) is about 3x faster: the view keeps a
  compact copy of the geometry and draws a slightly simplified path while it moves, then full detail 0.2 s
  after the last move.

### Changed
- BreakoutCut no longer shows the spindle speed bar under the position readout (it only applies to the mill).
- The G-code list's line-number column widens for programs with more than 9999 lines.
- GLFW / OpenGL errors are printed to the terminal, which helps diagnose startup problems on Linux and macOS.

### Project
- README screenshots, a Download section, and GitHub issue forms for bug reports and Linux / macOS feedback.
- CI builds the release packages for all three platforms on every push and starts both applications
  (`--smoke-test`) on Linux and macOS to catch startup problems.
- Fixed a timing-dependent unit test that failed occasionally on slow CI machines.

## 0.2 - performance on older PCs, optional homing

Mainly makes both apps run smoothly on older laptops, with a real board connected and with large programs.

### Performance
- Fixed: with a real RNR board connected, the whole UI lagged during homing and jobs (about 9-15 FPS on a laptop).
  The board can take about 25 ms to accept each USB packet, and the controller waited for that while holding the
  lock every panel reads from. Packets now go through a queue written by a separate thread, in the same order.
- Fixed: the UI thread kept a whole CPU core busy (the graphics driver busy-waited for the display refresh),
  which made both apps lag badly on 2-core laptops. Frames are now paced by sleeping (60 FPS, 10 FPS while
  minimized); idle CPU use dropped from about 100 % to about 20 % of one core.
- Much faster toolpath view for large programs. The view keeps its geometry between frames and rebuilds it only
  when the program, zoom, 3D angle, Detail setting or tool filter changes, so panning, following the tool and job
  progress cost almost nothing: a 650k-segment 3D carving now stays at 60 FPS on a 2014 laptop (was about 13 FPS).
  Chains of short connected moves are drawn as one line while they stay within a pixel tolerance, lines outside
  the view are skipped, and rapid-move dashes are only drawn where visible.

### New
- Setting "Require homing for G53 / G28 / G30" (End-Stops, Homing & Soft Limits), on by default. Turn it off on a
  machine without home switches: programs and MDI commands that move in machine coordinates then run without
  homing. Machine coordinates then continue from the position saved when the app was last closed.
- "Detail" button in the toolpath toolbar: drawing tolerance of the toolpath view (0.5-4 px, default 1 px), saved
  with the settings. It changes only the display, never the machine motion.
- F12 performance overlay in both apps: frames per second, the time per frame spent in each panel, ImGui, the GPU
  and the swap, plus how long UI reads wait for the controller, how long each controller tick holds its lock and
  how long USB writes take. `BREAKOUTCNC_PERF_LOG=<file>` writes the same numbers to a file once per second.

### Changed
- Toolpath view layout: the color legend (Cut / Active / To Be Cut / Rapid) moved from the toolbar to the row
  below it, right-aligned next to the Zoom and 3D readouts, with color swatches; it is hidden when the view is too
  narrow. The Detail button and the tool selector are aligned to the right end of the toolbar.
- The overlays on the toolpath view (toolbar, readouts, legend, alarm and probing banners) have an 85 % opaque
  background, so the toolpath stays faintly visible behind them.
- All readout boxes on the toolpath view (Zoom, 3D RMB, camera status, Part Bounds, Progress, Elapsed) are as wide
  as their label and value instead of a fixed width. Completed cuts and rapids are drawn 1 px wide (was 1.2 px).

### Fixed
- The Progress readout overlapped its label on programs with 100k+ segments.
- The window sometimes did not appear at startup, or opened partly off-screen, when the saved position or size
  came from a larger or no longer connected monitor. The window now opens inside the screen's work area and is
  shown only after its first frame is drawn.
- After a settings change that re-checks the loaded program (tool length mode, G4 dwell units), the program could
  keep its old warnings / dwell times when the controller's background thread re-parsed it at the same time.

## 0.1 - first public release (GPL-3.0-or-later)

First public release. Experimental: test carefully on your own machine before trusting it with real work.

- **BreakoutMill**: 4-axis mill / router controller (X, Y, Z, A with optional gantry slaving).
- **BreakoutCut**: plasma and laser cutter controller with torch height control (THC) and laser mode.
  The plasma and laser features have only been tested in the simulator, not on a real cutting machine.
- RNR Eco Motion 2.0 USB motion board support (host-planned 25 ms step slices), plus a built-in simulator.
- G-code parser and look-ahead trajectory planner, feed hold, overrides, work offsets G54-G59, tool table.
- Homing, soft limits, probing (Z touch-off, edge / corner / bore / boss probing scripts), height-map surface
  probing.
- Huanyang VFD spindle control over RS-485 Modbus, with speed monitoring and reading limits.
- Lua 5.4 scripting (`cnc.*` API) for M6 tool change, homing, macros and custom M-codes.
- Mach3 profile (XML) import, including the RNR and Huanyang VFD plugin settings.
- Camera view with toolpath overlay and an optional MJPEG stream server.
- Board driver layer (`IBoardDriver`) prepared for supporting other motion boards later.
- Release zip via `cmake --build build --config Release --target package`; runs without the Visual C++ Redistributable.
- Huanyang VFD serial port support on Linux and macOS (POSIX termios), untested.
- GitHub Actions CI: build and tests on Windows, Linux and macOS; Windows release zip as a build artifact.
- Release workflow: pushing a version tag publishes a GitHub Release with the Windows zip.
