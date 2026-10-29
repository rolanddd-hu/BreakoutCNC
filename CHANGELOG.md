# Changelog

All notable changes to BreakoutCNC are listed here. The version number is set once, in `project(VERSION ...)` in
[CMakeLists.txt](CMakeLists.txt); the window titles and the `.exe` file properties pick it up from there.

## Unreleased

- Fixed: after a settings change that re-checks the loaded program (tool length mode, G4 dwell units), the program
  could keep its old warnings / dwell times when the controller's background thread re-parsed it at the same time.

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
