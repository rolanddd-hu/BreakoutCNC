# Changelog

All notable changes to BreakoutCNC are listed here. The version number is set once, in `project(VERSION ...)` in
[CMakeLists.txt](CMakeLists.txt); the window titles and the `.exe` file properties pick it up from there.

## 0.5 (in development)

### Added
- **Board selector** (*Connection* > **Board**): besides the RNR Eco Motion 2.0, BreakoutMill and BreakoutCut drive
  **GRBL 1.1**, **grblHAL** and **FluidNC** boards over USB serial or the network (telnet port). The app identifies
  the firmware when connecting and says so when a different flavour answers.
- On these boards the app converts Mach3-style programs into G-code the firmware accepts: arcs are sent as G2 / G3,
  drilling cycles, G68 rotation, tool lengths and G53 moves as plain moves, and tool changes (M6), program stops
  (M0 / M1) and Huanyang VFD spindle changes stop the board and are handled by the app. Work offsets stay in the app.
- Jogging (continuous and step), homing (the board's own cycle), probing (G38.3), MDI, feed hold, resume, stop,
  E-stop, feed / rapid / spindle overrides, spindle and coolant, laser jobs (`M4`), framing and the laser test pulse
  work on these boards. All probing routines, tool measuring and surface maps run on them as on the RNR board.
- **Board Settings**: the firmware's settings with descriptions (GRBL 1.1 `$$`, the grblHAL settings it describes
  itself, FluidNC's configuration), with write, backup and restore.
- **Board Console**: the data exchanged with the board, and a command line.
- **Plasma THC on grblHAL**: a grblHAL board with the plasma plugin runs torch height control itself. BreakoutCut
  detects the plugin, allows plasma jobs with THC on that board, shows its Arc OK, torch, THC and anti-dive state and
  the arc voltage under *Torch Height Control*, and lists the plugin's settings (`$350`-`$369`) in a *Plasma* group
  of *Board Settings*. When the torch does not start, the board pauses the job and the app says why.
- **4th axis (A)** on grblHAL and FluidNC boards that have one: DRO, jogging, homing, MDI and jobs. BreakoutMill shows A
  only when the board reports four axes; a program that moves A cannot start on a 3-axis board. A Dual-Motor Gantry
  setting from the RNR board does not apply to these boards (they gang motors in their firmware).
- **Save board report** (*Board Console*): asks the board for its build info, settings, offsets and modes (`$I`,
  `$$`, `$#`, `$G` and the like; nothing is changed) and saves them with the recent exchange to a text file next to
  the application, to diagnose a new board.
- Alarm and error texts from the board itself on grblHAL (`$EA` / `$EE`) and FluidNC (`$A` / `$E`) for codes beyond
  GRBL 1.1's, read when connecting.
- Program check on GRBL-family boards: moves beyond the board's travel (`$130`-`$133` and the homing direction, homed
  axes only), feeds above the board's maximum rates (`$110`-`$113`) and custom outputs the board does not have are
  warned about.
- Lua: `cnc.board()` returns the selected board and what it can do.
- Remote monitoring: an optional **access code** (the page asks for it once per browser session) and a **Listen on**
  choice (all networks, one network address, or this PC only).
- **Test mode without a board:** start BreakoutMill or BreakoutCut with `--fake-board` to drive a built-in GRBL 1.1,
  grblHAL and FluidNC emulator. Hardware access and the VFD link are off in this mode, and it uses its own settings
  file (`*_settings_fakeboard.ini`).
- Program check: warnings for G61 / G64 (no effect on these boards) and for THC programs on a board that cannot run
  THC.
- BreakoutCut laser: **S for 100% Power** (*Laser Power*), 1000 by default or 255 for CAM that writes `S0`-`S255`; the
  program check warns about S values above it.
- **Hooks** in *Scripts & Macros*: tick **M6 script** (BreakoutMill) and **Home script**. With the home script ticked,
  HOME ALL runs `home_all.lua`; the H buttons of single axes always use the built-in cycle. Lua: `cnc.home("Z")` and
  `cnc.home()` start the built-in homing cycle from a script.
- A settings file that cannot be saved (read-only folder) is reported in the window instead of being lost silently.
- The window draws about 15 frames per second instead of 60 while nothing changes (machine at rest, no camera, no
  input for a second), and wakes at once on any input.

### Fixed
- **Laser `S0` cut at full power** on the RNR board and kept the previous power on GRBL-family boards. `S0` is now 0 %
  (beam off).
- **Huanyang VFD:** switching the VFD off in the settings (or closing the application) while the spindle ran could
  leave the spindle running. A stop command is now sent and acknowledged first.
- The shipped `home_all.lua` scripts called a Lua function that did not exist (`cnc.home`).
- Lua: an unknown axis name (`cnc.zero_axis("B")`, a `nil` variable) acted on X; it is now an error. A negative
  spindle speed is refused.
- Lua scripts can no longer catch the E-stop cancellation (`pcall`, `coroutine`) or remove it (`debug.sethook`), and
  every script starts from a fresh Lua state (globals left by one script no longer change the next one).
- Many small feed override steps added up to nothing on the RNR board; every change now reaches the planner.
- A probe cancelled or failed during a tool change wait returned to Idle instead of the tool change.
- A refused MDI line (soft limit, bad word) could still switch the work offset (G54..G59) or the tool length offset.
- A G2 / G3 line with only I / J (a modal full circle) was ignored.
- File names with characters outside the Windows code page (for example `ő` on a Western system, Cyrillic) could not
  be opened from the file dialog, the command line, by drag and drop or from Lua scripts, and showed garbled.
- Connecting to an unreachable network board froze the window for several seconds.
- A file without moves replaced the loaded program; it is now refused with a message, and a remote upload refused
  during a job says why.
- Esc pressed to close a dropdown while the machine was at rest triggered the E-stop (it still stops a moving
  machine, whatever has the focus).
- Program file names in the remote upload folder are no longer kept forever (cleaned at startup).
- COM ports above COM128 are listed, in numeric order.
- Remote monitoring web server security:
  - Another web page opened on the shop network can no longer upload G-code or press the remote E-stop through the
    browser, and a web site cannot read the camera or the job state by pointing its own name at the PC (DNS
    rebinding). Actions need the dashboard's own header and origin, and the server only answers to the PC's address
    or name. The page cannot be shown inside another site.
  - An uploaded file named after a Windows device (`COM3.nc`, `CON`, `NUL`) is saved under a safe name instead of
    being written to that device.
  - Slow or idle connections can no longer block the server (and the remote E-stop): requests have a deadline,
    connections per device and camera viewers are limited, and a camera viewer that leaves frees its place even when
    no camera is selected. A request body is only read for an allowed upload, one upload at a time.
  - Changing the port or switching the server off and on can no longer hang or leak the port, and closing the app
    waits for an upload still being loaded instead of crashing. No other program can take over the server's port; a
    port already in use is shown in the panel.
- GRBL-family boards: coolant is switched with one M7 / M8 / M9 per line. GRBL refuses two words of that modal group
  in one line (error 21), which stopped jobs at their first coolant change and the manual Flood / Mist buttons.
- GRBL-family boards, BreakoutCut plasma: the torch is switched on with `M3 S<$30>` instead of a bare `M3`; with S0 a
  variable-spindle board (GRBL 1.1 on an Arduino) keeps its spindle output off, so the torch did not fire.
- GRBL-family boards: EMERGENCY STOP also resets the board while it is being identified after connecting; the app then
  identifies it again. A network connection that stalls is reported lost after 1 s instead of freezing the app (and
  the E-stop behind it). A line longer than the board accepts stops the job with a message before it is sent; a long
  line is first sent without its line number. FluidNC alarms 10 and above no longer clear the homed state when the
  position is kept (FluidNC numbers them differently from grblHAL).
- GRBL-family boards: during G2 / G3 arcs the highlighted line and the cut progress follow the torch around the arc.
  Before, a whole arc was marked as cut as soon as it started, and a full circle could be shown as active again after
  it was finished. A new job no longer briefly shows where the previous one ended.
- Test mode (`--fake-board`): the emulated boards move along G2 / G3 arcs instead of straight to their end point.
  BreakoutMill's emulated grblHAL and FluidNC boards have an A axis; the plasma plugin is only on BreakoutCut's
  grblHAL board (BreakoutMill had it too).
- GRBL-family boards reporting in inches (`$13 = 1`): the A position is no longer scaled as if it were inches.
- BreakoutCut: Cycle Start in the middle of a program no longer asks "Spindle Not Running ... start it again with
  12000 RPM?". That BreakoutMill question switched the torch or the laser on before the job moved; Cut now starts
  the job and the program's own M3 / M4 switches the output on.
- The MDI row no longer makes the program panel scroll sideways when the panel is narrow (shorter labels, input sized
  to fit, status moved to the tooltip).
- In step jog mode the status box under the jog pad is no longer cut off: the key hint is hidden when there is no room
  (its text moves to the STOP JOG tooltip).

### Changed
- Settings that the selected board keeps itself (motor tuning, switch inputs, homing speeds, pin assignment, THC,
  corner tolerance) are replaced by a short note pointing to *Board Settings*.
- The applications connect to the board after loading their settings, so the saved board is the one that is opened.
- The settings file has a `board = ...` key and one `[Board.<id>]` section per board. Files from 0.4 load unchanged and
  select the RNR board.
- The Huanyang VFD uses the new shared serial port code (same behaviour).

## 0.4

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
  range, a cut below work Z0 while the spindle is off (BreakoutMill), A moves while A follows another axis, and an
  M3 / M4 with no M5, M2 or M30 after it.
- Diagnostics report (Windows): the window's position and size as Windows sees it, and the mouse position as each
  layer reports it, to help with display problems.

### Changed
- Jog pad joysticks: a held knob can now travel over its whole pad instead of only its small socket, so the jog speed
  is easier to set. The X/Y cross turns into a circle in 0.1 s while its knob is held (and back on release), and the
  knob reaches full speed at the circle's edge. The Z and A knobs move along the full length of their pads.

### Fixed
- M2 / M30 now switch the coolant off as well, as in Mach3. The spindle already stopped at the end of every program,
  but coolant switched on with M7 / M8 stayed on without an M9.
- Programs ran 3-4 s behind the app on the RNR board: the motors started that much after the toolpath view, kept
  moving after the app showed the end, and Feed Hold and the feed override acted that late. The board confirms motion
  as soon as it has stored it, so the app sent motion up to three times faster than it ran until the board's
  buffer was full. Motion is now sent in real time, never more than 75 ms ahead of the board, and programs are sent
  the way MDI moves are. Spindle, torch and coolant outputs are
  switched with their own command where the program changes them (the machine is stopped there), and the spindle
  output now comes on while the app waits for it to spin up.
- RNR board: motion was jerky after Reset E-Stop until any setting, such as the pulse rate, was changed. The reset
  put the board back on its default timing; the board configuration is now sent again after every reset.
- Windows: a maximized window had an empty 8 px band around it. It now fills the screen's work area exactly.
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
