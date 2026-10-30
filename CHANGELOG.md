# Changelog

All notable changes to BreakoutCNC are listed here. The version number is set once, in `project(VERSION ...)` in
[CMakeLists.txt](CMakeLists.txt); the window titles and the `.exe` file properties pick it up from there.

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
