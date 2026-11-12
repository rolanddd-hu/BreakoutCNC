# BreakoutCNC code review findings

Review of the whole application on branch `grbl-family-boards` (working tree, including the uncommitted remote
monitoring changes). Nothing was changed in the code: this file is the to-do list for later.

## Outcome (fixes applied)

Status of every finding after the fix round. **Done** = fixed as described below the table; **Partly** = the part
listed was done, the rest is open; **Open** = not done, with the reason. Regression tests are in
`tests/regression_tests.cpp` unless another file is named; everything was also checked in the running apps
(`--fake-board --ui-remote`) where noted.

| ID | Status | What was done | Checked by |
|---|---|---|---|
| A1 | Done | `S0` is beam off on both board types; new setting **S for 100% Power** (`s_max`); program check warning above it. A fractional S is still rounded (no effect with S max 255 / 1000) | Regression test, board test 6e, BreakoutCut on the simulator (S0 line 0 %, S500 line 50 %) |
| A2 | Done | `stop()` commands a stop and waits up to 1 s for the acknowledgement; the link is injectable for tests. Disabling the VFD while it runs is not refused: it stops the spindle now | `huanyang_vfd_test` (fake RS-485 link) |
| A3 | Done | `cnc.home(axis)` / `cnc.home()` / `cnc.home_all()` run the built-in cycle; HOME ALL runs the script when **Home script** is ticked (new Hooks row); H buttons always built-in; scripts check each axis | Regression test, BreakoutMill on fake grblHAL |
| A4 | Done | Unknown axis and negative RPM raise a Lua error | Regression test |
| A5 | Done | All file names are UTF-8: wide file dialog, wide command line, drag and drop, `std::filesystem` from UTF-8, Lua `fopen` through `_wfopen`, old ANSI names converted when read | Regression test (folder with e acute, o double acute, Cyrillic), BreakoutMill opened and saved such a name |
| A6 | Done | Every override change reaches the planner; the queue is re-planned once the difference reaches 0.5 % | Regression test |
| A7 | Done | Failed save shown in the window until dismissed or a save succeeds. No fallback to a per-user folder (the install is portable on purpose) | BreakoutMill with a read-only settings file |
| A8 | Done | Shared `loadRemoteUpload()`: load first, keep a copy only when accepted, real refusal reason, old uploads cleaned at startup | BreakoutCut + curl (idle upload, upload during a job) |
| A9 | Done | `restStateLocked()` everywhere a move or probe ends | Regression test |
| A10 | Done | MDI undoes G54..G59 / G43 when the line is refused | Regression test |
| A11 | Done | A modal arc line with only I / J / K or R is executed (full circle) | Regression test |
| A12 | Done | `pcall` / `xpcall` / `coroutine.resume` / `coroutine.wrap` re-raise the cancel; `debug` reduced to `traceback`; `load` / `loadfile` / `dofile` text only | Regression test (4 uncatchable loops) |
| A13 | Done | Fresh Lua state per run (the API context is freed with it). `io.open` stays writable: scripts are the user's own | Regression test |
| A14 | Done (decided) | ESC always E-stops while the machine moves or a job is paused; at rest it first closes a dropdown or leaves a text field | BreakoutMill: dropdown at rest, at rest, during a job |
| A15 | Done | The board stream opens on the protocol's reader thread; `poll()` on POSIX; `[v6]:port` addresses | Regression test (slow stream) |
| A16 | Done | VFD driver started / stopped outside the controller lock (`syncVfd()`) | All suites |
| A17 | Done | One prompt snapshot; empty program refused without replacing; leftover removed; `exists()` without throw; COM ports above 128, numeric order; IPv6 | Regression tests |
| A18 | Done | All listed buttons are 28 px. Audit of every long single-line text: 11 wrapped; 3 left on purpose (the program row next to the cycle buttons, the performance overlay, a label followed by its value on the same line) | Audit script, screenshots |
| P1 | Done | `getConfigShared()`: a cached, shared snapshot (compared, copied only after a change); every UI read uses it | All suites, both apps |
| P2 | Partly | `tool_description` left the segment (looked up by tool number), source lines shared between program copies, segment is plain data (`static_assert`). Not done on purpose: moving machine targets to a side table. Every path that maps segments to the machine (planner, program check, firmware path, MDI) would have to find the table, and a missed one would silently send G53 / G28 moves to the wrong place, a crash risk for ~15 % of segment memory | Regression test |
| P3 | Done | Parser reuses its buffers; `strchr` | All suites |
| P4 | Done | Planner buffers are members | All suites |
| P5 | Done | One read and one write event per serial port | Board tests (fake stream); real ports not tested |
| P6 | Done | Idle frame rate ~15 FPS (perf log: 42, 26, 17, 15, 15 ...); `imgui_demo.cpp` removed; `getSliceQueue()` marked as a test accessor | BreakoutMill perf log |
| M1 | Partly | Stage 1: `machine_controller.cpp` split into 8 topic files. Stage 2 (a backend seam between the RNR slice path and the firmware path) not done: it is a redesign of the motion core, best done on its own branch. The new `rnr_hardware_check` (103 checks on a real board) and the board tests are the safety net it needs | Build with warnings as errors, all suites, RNR hardware check |
| M2 | Partly | Shared in `ui_common`: remote upload (load and hand-over to the UI), ESC rule, save warning, idle rule, window settings (layout, themes, viewport), close confirmation, smoke test, Limits & Homing and Motors sections. Each `main()` is still ~1 700 lines: the rest is the app-specific settings sections (spindle / VFD / tool table in Mill, THC / laser in Cut) and layout | Both apps: upload, theme, close dialog, Limits / Motors |
| M3 | Done | Helpers: spindle speed (3), `jobActiveLocked`, `restStateLocked`, `laserInterlocksOkLocked`, `planManualMovesLocked`, M6 script and tool change moves, `warpProgram` (also fixes the bounding box after load / re-parse), `syncVfd` | All suites |
| M4 | Done | `machine_config.h`, `surface_height_map.h`, `toolpath_segment.h`, `telemetry.h`; the parser includes only what it needs | Build |
| M5 | Done | One table drives loading and saving of the ~150 plain settings; list entries and old key names are explicit code. Round-trip test: no key lost. Re-saving the existing test settings files gave identical keys and values. A read-only backup file is made writable before it is replaced | Regression test, re-save comparison |
| M6 | Done | `parse()` is a `Parser` class: program state as members, per-line state reset for each block, one function per step (line, modal words, tool change, axis words and G68, G92 / G53 / G28, dwell, motion, canned cycle, arc, end of block). Old and new parser output compared on 16 049 dump lines (samples, tricky blocks, errors, options): identical | Parse dump comparison, all suites |
| M7 | Done | `firmware_planned_path.cpp` split into 4 topic files | Board tests |
| M8 | Done | Script engine test split into 4 files (same executable, 51/51); regression tests added; CTest | CTest |
| M9 | Done | `core/util/app_paths` used by the app and the script lookup (now cross-platform) | All suites |
| M10 | Done | `winmm` linked in CMake, old names in scripts, `kPi` | Build |
| B1 | Done | CTest for all 4 suites; both workflows run `ctest` | Local `ctest` |
| B2 | Done | `/W4` on own targets (clean, `/WX` in Windows CI); `-Wall -Wextra` on GCC / Clang (not errors); ASan + UBSan job, blocking since its first clean run | CI run of b5e000a: Windows, Linux, macOS, sanitizers all passed |
| B3 | Done | GLFW, Lua, hidapi pinned to commits | Configure + build |
| B4 | Done | One script / resource copy, `copy_directory_if_different` on CMake 3.26+; `.editorconfig` and `.clang-format`. Version bump left for the release | Build |
| D1-D3 | Done | User guide homing and laser S text, historical note on the scripting spec, changelog | - |


### Found with the RNR board on the bench (after the first fix round)

The new `tests/rnr_hardware_check.cpp` drives a real RNR Eco Motion 2.0 through the controller. Built against the code
from before this review (0c911d1), it showed the same problems as against the reviewed code, so none of them came
from the refactors. All four are fixed and covered by the check (103/103, twice) and by a regression test with an
RNR-like fake board.

| ID | Problem | Fix |
|---|---|---|
| R1 | Connecting reported "USB CONNECTION TO MOTION BOARD LOST" right away: the worker read whether the board was open before taking the lock, and the connect landed in between | Read under the lock |
| R2 | An MDI move (or jog) started right after another one was lost: the app went Idle when the board had buffered the last slices, and the board ignores a new start while it still runs | Idle only once the board has had the time to run every slice (moves, jogs, the stop after a feed hold) |
| R3 | A controlled stop could leave the board a few hundredths of a millimetre from the app's position: the cancel reached the board while it still ran the last slow-down slices | Same wait before the cancel |
| R4 | After homing (or a restored position) and an E-stop, the app could replace its machine position with the board's counters, moving every work offset by the homing distance. The board ignores Set Axis Positions and clears its counters on reset, so they are not machine coordinates | The counters count as proven only after they match again; the same flag guards probing's latched contact position |

## How to read this

- **Severity:** High = can damage a workpiece or leave a machine output on; Medium = wrong behaviour a user will
  meet; Low = edge case, hardening or polish.
- **Confidence:** *Confirmed* = the whole code path was read end to end; *Likely* = read, but one part was not
  followed or needs the running app; *Check* = a suspicion to verify first. Nothing here was reproduced in the
  running app or with a new test, because the task was review only.
- **Effort:** S = under an hour, M = a few hours, L = a day or more.
- Paths are repository-relative. Line numbers are from the reviewed working tree and will drift: the function name
  is given as well.

### What was read, and what was not

| Area | Depth |
|---|---|
| `src/core/controller/machine_controller.cpp/.h` (5 220 lines) | Every line |
| `src/core/scripting/script_runner.cpp`, `script_engine.cpp`, `script_path_helper.h`, `src/core/transport/*` | Every line |
| `src/core/scripting/script_api.cpp` | First third line by line, the rest by outline and the registration list |
| `src/core/gcode/gcode_parser.cpp` | The parser (`:1-1040`) line by line; samples and machine detection by outline |
| `src/core/planner/trajectory_planner.cpp` | About 60 % (profile, window planning, non-motion) |
| `src/core/spindle/huanyang_vfd.cpp`, `src/core/config/settings_manager.cpp` | Threading, start / stop, save / load structure |
| `src/core/motion/firmware_planned_path.cpp` | `open`, `close`, `tick`, `emitOutputsFor`; the rest by outline only (it had a full review with 10 fixes earlier on this branch) |
| `src/app_mill/main_mill.cpp`, `src/app_cut/main_cut.cpp` | Startup, frame loop head, structure; duplication measured by script |
| `src/ui_common/*` | Pattern searches only (controller calls per frame, file I/O, AGENTS.md rules), plus `gcode_panel.cpp` file loading |
| `CMakeLists.txt`, `.github/workflows/*`, `.gitignore` | Every line of CMake and `ci.yml`; `release.yml` steps |
| Not reviewed | `camera_manager.cpp`, `camera_stream_server.cpp` (reviewed and fixed in the previous step), `mach3_xml_importer.cpp`, `rnr_usb_device.cpp`, `program_check.cpp`, `io_map.cpp`, `board/grbl/*`, most UI panels line by line, the Lua scripts except `home_all.lua` |

### Top items

| ID | Severity | What |
|---|---|---|
| A1 | High | Laser: `S0` gives 100 % power on the RNR board and keeps the previous power on GRBL-family boards |
| A2 | High | Huanyang VFD thread can stop without sending a stop command (VFD disabled in settings, app exit) |
| A3 | Medium | "Home script" hook is broken: the shipped `home_all.lua` calls `cnc.home`, which does not exist |
| A4 | Medium | Lua: an unknown axis name silently means X |
| A5 | Medium | Windows file paths are ANSI: some file names cannot be opened and accented names may show garbled |
| B1 | Medium | CI and the release workflow never run `board_tests` |
| P1 | Medium | The UI deep-copies the machine configuration many times per frame under the controller lock |
| M1, M2 | n/a | The two biggest maintenance costs: the 5 220-line controller and the two 2 200-line `main()` functions |

---

## A. Bugs and safety

### A1. Laser: `S0` is treated as "no S word" (High, Confirmed, M)

**Where**
- `src/core/gcode/gcode_parser.cpp:196` (`spindle_rpm = 0; // 0 = not programmed`) and `:545` (S word stored as
  `uint16_t`, so `S0` becomes 0).
- `src/core/controller/machine_controller.cpp:4234`, `sendQueuedSliceLocked()`:
  `base_power_pct = 100.0; if (spindle_rpm > 0 && spindle_rpm <= 1000) base_power_pct = spindle_rpm / 10.0;`
- `src/core/motion/firmware_planned_path.cpp:1730` and `:1752`, `emitOutputsFor()`:
  `rpm = s.spindle_rpm > 0 ? s.spindle_rpm : job_rpm_;` then `pct = (rpm > 0 && rpm <= 1000) ? rpm / 10.0 : 100.0;`

**What happens**
A segment cannot tell "S0 was programmed" from "no S yet". With the beam output on (M3 / M4 active, feed move):
- RNR board (slice path): `S0` gives 100 % power.
- GRBL-family boards: `S0` keeps the power of the previous S word.

**Why it matters**
LightBurn and LaserGRBL write `G1 X.. S0` for overscan and for white pixels of an image, and often start with
`M4 S0`. Those moves would burn at full (or the previous) power.

**Fix**
- Carry "S programmed" separately from its value (for example `int32_t spindle_s = -1` for none, or a
  `bool spindle_s_set`), through `ToolpathSegment`, `PlannedSlice` and the firmware emitter.
- Laser mode: programmed `S0` = beam off (duty 0, not `min_duty_pct`). Firmware path: emit `S0`.
- Mill mode: decide what `M3 S0` means (today: keep the operator's speed) and say so in `docs/GCODE_REFERENCE.md`.

**Tests to add**
- Parser: `M4 S0` / `G1 X10 S0 F1000` gives a segment with "S set, value 0".
- Controller (simulator, laser mode): `live_laser_power_pct == 0` and `spindle_pwm_duty == 0` on the `S0` move.
- Board tests (FakeGrbl): the emitted line carries `S0`.

**Related, lower priority**
- The laser S scale is fixed at 0..1000; above 1000 is silently 100 %. A setting "S value for 100 %" (1000 or 255)
  plus a program-check warning for values above it would match what LightBurn offers.
- A fractional S (`S0.5`) rounds to 0 or 1.

### A2. Huanyang VFD: the worker thread ends without a stop command (High, Confirmed, S-M)

**Where**
- `src/core/spindle/huanyang_vfd.cpp:213` `stop()`: sets `running_ = false`, joins, closes the port.
- `:497` `workerLoop()`: leaves the loop as soon as `running_` is false; a pending `run_dirty_` stop is not sent.
- Callers: `machine_controller.cpp:554` (`applyConfigLocked`) and `:575` (`setHuanyangVfdConfig`) when the VFD is
  switched off in the settings; `~HuanyangVfdController()`.

**What happens**
- Spindle running through the VFD, the user unticks "enabled" for the VFD: the thread stops, the port closes, the
  VFD keeps running. The app then has no way to stop it.
- App exit: `triggerEStop()` sets `run_dirty_`, but nothing waits for the frame to go out before the thread is
  joined. It usually goes out (window teardown takes longer than one poll), but it is not guaranteed: the worker can
  be inside a 250 ms receive timeout plus the 60 ms frame gap.

**Fix**
- In `stop()` (before `running_ = false`), or at the end of `workerLoop()`: if the last commanded or confirmed
  state is not "stop", send the stop frame with 2-3 retries.
- Refuse to disable the VFD (or change its port) while the spindle is commanded on.

**Test to add** in `tests/huanyang_vfd_test.cpp` with a fake serial stream: run, then `stop()`, expect a stop
frame as the last frame.

### A3. The "Home script" hook is broken and half-wired (Medium, Confirmed, M)

**Where**
- `scripts/mill/home_all.lua:13,18,23` and `scripts/cut/home_all.lua:13,18,23` call `cnc.home("Z")`.
- `src/core/scripting/script_api.cpp` registration list (from about `:780`): there is no `home` function. A script
  cannot start homing at all.
- `machine_controller.cpp:1979` `homeAxis()` and `:2009` `homeAllAxes()` run `home_script_path` when
  `scripting.home_script_enabled` is set.
- No UI control was found that sets `home_script_enabled` (only `settings_manager.cpp:366` / `:694`).
- `docs/USER_GUIDE.md:113` says HOME ALL runs `scripts/mill/home_all.lua`; `src/ui_common/lua_guide.cpp:618`
  describes a 'Home script' option.

**What happens**
With the option on (by editing the ini file), a HOME button runs the script, which switches the spindle off and
then fails with "attempt to call a nil value (field 'home')". Nothing is homed.

**Also**
- `homeAxis(X)` runs the all-axes script, not a single-axis one.
- If `cnc.home` were added naively, it would call `homeAxis()`, which would try to start the script again and be
  refused ("Another script is already running").

**Fix (choose one)**
1. Implement it: `cnc.home(axis)` and `cnc.home_all()` that run the built-in cycle and bypass the hook (an internal
   `homeAxisBuiltIn()`), add the option to the Scripts panel, make single-axis H buttons use the built-in cycle.
2. Remove it: the hook in `homeAxis` / `homeAllAxes`, the two scripts, the settings keys and the doc text.

**Test to add**
A test that loads every file under `scripts/` and every example in `lua_guide_examples.h` and checks that each
`cnc.<name>` it uses is registered. Done by hand for this review: `cnc.home` is the only miss in the scripts;
`docs/architecture/SCRIPTING_ENGINE_SPEC.md` also names `cnc.message`, `cnc.set_current_tool` and
`cnc.set_work_offset`, which do not exist (see D2).

### A4. Lua: an unknown axis silently becomes X (Medium, Confirmed, S)

**Where** `src/core/scripting/script_api.cpp:40` `parseAxis()`.

**What happens** Any string that does not start with X / Y / Z / A, `nil`, or a non-integer number returns 0 (X);
integers are clamped to 0..3. `cnc.zero_axis("B")` or `cnc.set_work_pos(axis_var, 0)` with a `nil` variable zeroes
or sets X.

**Fix** `return luaL_error(L, "invalid axis ...")` for anything that is not a valid axis. Check the other
arguments for the same pattern (`cnc.set_spindle_rpm(-1)` casts a negative number to `uint32_t`).

### A5. Windows file paths are ANSI (Medium, Likely, M)

**Where**
- `src/ui_common/file_dialog.cpp:64` `GetOpenFileNameA`.
- `src/ui_common/app_window.cpp:382-390`: a dropped file path is converted from UTF-8 to the ANSI code page.
- `std::ifstream(std::string)` / `std::ofstream(std::string)` in `gcode_panel.cpp:589`,
  `board_panels.cpp:365,421,431`, `mach3_xml_importer.cpp:265`, `settings_manager.cpp`.
- `src/core/scripting/script_path_helper.h:32` `GetModuleFileNameA` with `MAX_PATH`.

**What happens**
- A file whose name has a character outside the system code page (Cyrillic on a Western system, or `ő` / `ű` on a
  CP1252 system) cannot be opened. `gcode_panel.cpp:589` then ignores it without a message.
- ImGui draws UTF-8. An ANSI path with accented letters would be shown garbled in the program panel and title
  (not checked in the running app).
- Paths longer than 260 characters fail.

**Fix** One encoding inside the app: UTF-8. Convert at the edges only: `GetOpenFileNameW`, `std::filesystem::path`
built from UTF-8 for every stream, `GetModuleFileNameW`. One helper (`openInputFile(utf8_path)`) instead of bare
`std::ifstream` calls. Show "Cannot open <file>" when it fails.

### A6. Small feed override steps never reach the planner (Low-Medium, Confirmed, S)

**Where** `machine_controller.cpp:3677` `setFeedOverride()`.

**What happens** `telemetry_.feed_override_pct` is updated first, then the function returns when the change is
under 0.5 %. Many small steps (a float slider, `cnc.set_feed_override` in a loop) move the displayed override while
the slice planner keeps the old one. GRBL-family boards are not affected (the board gets the value before the
early return).

**Fix** Compare against the value last given to the planner, not against the previous telemetry value.

### A7. A failed settings save is ignored (Low-Medium, Confirmed, S)

**Where** `src/app_mill/main_mill.cpp:277`, `src/app_cut/main_cut.cpp:259`: the result of
`SettingsManager::saveToFile()` is dropped.

**What happens** The settings file sits next to the executable. In a read-only folder (Program Files, a
write-protected USB stick) every save fails silently: work offsets, tool table and the saved position are lost on
exit.

**Fix** Show the failure in the status bar once. Longer term: fall back to a per-user folder when the executable
folder is not writable.

### A8. Remote G-code upload callback (Low, Confirmed, S)

**Where** `main_mill.cpp:104-135`, the same block in `main_cut.cpp:96-127`.

- The state check lists Running / Dwell / Probing / Homing. In PausedHold, ToolChangeHold or Jogging,
  `loadGCodeString()` refuses and the page shows "Failed to parse G-code: no valid toolpath moves found", which is
  the wrong reason.
- The file is written to the temp folder before the load is accepted; the write result is not checked.
- The `BreakoutCNC/uploads` folder in the temp directory is never cleaned (up to 32 MB per upload).
- `last_gcode_file_path` then points into the temp folder.

**Fix** Let `loadGCodeString` return a reason (enum or string) and pass it on; write the file only after the load
succeeded; delete old uploads at startup. Move the block into `ui_common` (see M2).

### A9. Probe failure during a tool change hold ends in Idle (Low, Confirmed, S)

**Where** `machine_controller.cpp:2161` (`onProbeMoveFinishedLocked`, no contact), `:2460` (surface probing sets
`probe_return_state_ = Idle`), the `fail` lambda in `continueHoleCenterLocked()`, `cancelSurfaceProbing()`.

**What happens** Probing and tool measuring are allowed in ToolChangeHold. When the probe finds nothing, the state
becomes Idle although `pending_tool_number > 0`. Cycle Start still resumes (it checks the pending tool), but the
badge says Idle and Idle-only commands (homing, go to zero, framing) are accepted in the middle of a tool change.

**Fix** One helper, `restStateLocked()`, returning `pending_tool_number > 0 ? ToolChangeHold : Idle`. The same
expression is already written out in six places; use the helper everywhere a move ends.

### A10. MDI applies side effects before it validates the line (Low, Confirmed, S-M)

**Where** `machine_controller.cpp` `executeMdiCommand()`, about `:2729-2860`.

**What happens** G54..G59 and G43 / G49 are applied first; the line is parsed and soft-limit-checked afterwards.
`G55 G1 X9999` is rejected, but the work coordinate system has already changed.

**Fix** Parse and check first, apply all effects after the last `return false`.

### A11. Parser drops a modal arc line without X / Y (Low, Confirmed, S)

**Where** `src/core/gcode/gcode_parser.cpp:859`.

**What happens** With G2 / G3 modal from an earlier line, a line that has only `I` / `J` / `K` or `R` (a full
circle) is neither executed nor reported: the condition needs a G word on the same line.

**Fix** Execute it like any modal arc, or add an error. An error is the safer first step.

### A12. A script can make itself uncancellable (Low, Confirmed, M)

**Where** `src/core/scripting/script_runner.cpp:14` (the cancel hook raises a Lua error),
`src/core/scripting/script_engine.cpp:58` (`luaL_openlibs` keeps `debug`, `pcall`, `load`).

**What happens**
- `while true do pcall(function() ... end) end` catches the cancel error every time.
- `debug.sethook()` removes the hook.

The runner then stays "running" for ever, and every later script (M6, macros) is refused with "Another script is
already running". Motion is still stopped by the E-stop itself.

**Fix** Remove `debug` from the sandbox (or at least `debug.sethook`), wrap `pcall` / `xpcall` so they re-raise the
cancel error, and switch the hook to every instruction once cancel is requested.

### A13. One Lua state for all scripts (Low, Confirmed, M)

**Where** `script_runner.cpp` `initLuaEnvironment()` is called once in the constructor.

**What happens** Globals set by one script (including an overwritten `cnc.gcode`) are still there when the next
script, for example `m6.lua`, runs. A typo in one macro can change how another behaves.

**Fix** A fresh environment table per run (`_ENV` with the API as `__index`), or re-create the VM between runs.
Hardening that fits here: `io.open` can write any file, `load` accepts binary chunks, `require` is open. Scripts
are the user's own files, so this is robustness, not a vulnerability.

### A14. ESC is an E-stop in every context (Low, design question, S)

**Where** `main_mill.cpp:352`, `main_cut.cpp:380`.

**What happens** ESC also closes ImGui popups, combos and text fields. Pressing it there during a job E-stops the
machine; on the RNR board without validated position feedback that also means re-homing.

**Options** Keep it (it is deliberate), skip it while a popup or a text field is active, or make it a setting.
Needs the owner's decision.

### A15. Connecting to a network board blocks the controller lock (Low, Confirmed, M)

**Where** `machine_controller.cpp:195` `connectLocked()` calls `FirmwarePlannedPath::open()`
(`firmware_planned_path.cpp:80`), which calls `TcpStream::open()` (`src/core/transport/tcp_stream.cpp`):
`getaddrinfo` plus a 3 s connect timeout per address, all under `MachineController::mutex_`.

**What happens** With an unreachable address the UI freezes for 3 s or more (every `getTelemetry()` waits) and the
motion tick stalls.

**Fix** Open the stream on the protocol's reader thread; the Connecting state and the ConnectFailed event already
exist.

**Also in `tcp_stream.cpp`** `select()` with `FD_SET` at `:93` and in `read()`: undefined behaviour on POSIX when
the descriptor is 1024 or higher. `poll.h` is already included; use `poll()`.

### A16. VFD start / stop joins a thread under the controller lock (Low, Confirmed, S)

**Where** `machine_controller.cpp:552-557` and `:573-579`.

**What happens** `vfd_controller_.stop()` waits for the VFD thread (up to about 300 ms: receive timeout plus frame
gap) while `mutex_` is held. Only on a settings change while idle, so the effect is a short stall.

**Fix** Decide under the lock, call `start()` / `stop()` after releasing it. The two blocks are also duplicates
(see M3).

### A17. Smaller correctness notes (Low, S each)

- `machine_controller.cpp:612-625` `getTelemetry()`: `hasPendingPrompt()` and `getPendingPrompt()` are two
  separate locked calls; the prompt can change in between. Return an `optional<PromptRequest>` in one call.
- `loadGCodeString()` returns false for a program without segments but has already replaced the loaded program.
  Decide which one it should be.
- `machine_controller.cpp:2147`: `(void)ws;` is a leftover unused variable.
- `script_api.cpp:92` `cnc.wait_idle()` sleeps 25 ms "so the motion thread can pick up the move". The state is
  already Jogging when `executeMdiCommand()` returns, so the sleep should not be needed. `cnc.is_moving()` and
  `cnc.wait_idle()` use two different definitions of moving (DwellPierce is missing in one).
- `applyConfigLocked()` `:484`: the tool table revision is increased on every `applyConfig`, the work offset
  revision only when the offsets changed. Harmless today because the UI re-reads the configuration every frame, but
  the two should follow the same rule.
- `settings_manager.cpp` `loadFromFile()`: `std::filesystem::exists(file_path)` without an `error_code` can throw.
- `serial_port.cpp` `enumerate()`: only COM1..COM128 are listed.
- `tcp_stream.cpp`: an IPv6 address in brackets (`[::1]:23`) is not parsed.

### A18. AGENTS.md rule 2: buttons under 28 px (Low, Confirmed, S)

- `src/app_cut/main_cut.cpp:714` (24), `:1206` (24), `:1394` (26)
- `src/ui_common/camera_panel.cpp:189`, `:220`, `:229`, `:297`, `:305` (26)
- `src/ui_common/scripting_panel.cpp:80` "ABORT SCRIPT" (26)

About 50 `Text` / `TextDisabled` / `TextColored` calls have a literal longer than 45 characters. Many sit inside a
`PushTextWrapPos` block; they were not checked one by one. To list them:

```bash
grep -n 'ImGui::Text("\|TextDisabled("\|TextColored(.*"' src/app_*/*.cpp src/ui_common/*.cpp | awk -F'"' 'length($2)>45'
```

---

## P. Performance

### P1. Configuration and telemetry are deep-copied many times per frame (Medium, Confirmed, M)

**Where**
- `machine_controller.cpp:560` `getConfig()` and `:598` `getTelemetry()` return by value under `mutex_`.
- Callers: 20 `getConfig()` in `main_mill.cpp`, 21 in `main_cut.cpp`, 23 across the `ui_common` panels; several
  run every frame (`main_mill.cpp:381`, `:382`, `:495`, each open settings section, `dro_panel`, `io_status_bar`).

**Cost** `MachineConfig` holds two maps (board profiles, tool table), the macro buttons and a dozen strings; each
copy allocates. Every copy holds the lock the 25 ms motion tick needs (`diag::controllerReadWait()` already
measures the wait).

**Fix**
- Cheap: read the configuration once per frame in `main()` and pass `const MachineConfig&` into the panels.
- Better: the controller keeps `std::shared_ptr<const MachineConfig>` and replaces it on change; `getConfig()`
  returns the pointer (no copy, lock held for a pointer copy only). Same idea for the rarely changing parts of the
  telemetry (texts, prompt, VFD block).

### P2. `ToolpathSegment` is large and the program is copied whole (Medium, Confirmed, M-L)

**Where**
- `src/core/controller/motion_controller.h:671`: about 200 bytes per segment, with a
  `std::string tool_description` in every segment (`:684`) and a 32-byte `machine_target_mm` only G53 / G28 moves
  use.
- Arcs become chords with 5 µm sagitta (`gcode_parser.cpp`, `kArcSagittaMm`), each a full segment.
- `std::make_shared<ParseResult>(*parsed)` copies the segments **and** all source lines to replace only the
  segments: `reparseLoadedProgramIfNeeded()`, `loadGCodeString()`, `applySurfaceHeightMap()`.
- The source text is held three times (`program_text_`, `ParseResult::lines`, the UI copy), plus the
  `std::istringstream` copy while parsing.

**Cost** One million segments is about 200 MB, twice that with a height map applied.

**Fix** Take `tool_description` out of the segment (`ParseResult::tool_descriptions` already has it by tool
number), move machine targets to a side table indexed from the segment, make the segment trivially copyable, share
`lines` through a `shared_ptr<const std::vector<std::string>>`. Measure with a large laser raster file first.

### P3. Parser allocations per line (Low, Confirmed, S)

`gcode_parser.cpp`: a new `words` vector and two strings per line in `tokenizeBlock()`, and
`std::string("XYZAIJKR").find(...)` for every word (`:568`). Reuse the buffers and use `std::strchr`. Only worth it
together with a timing of a 500 000-line file.

### P4. Planner allocations in the motion tick (Low, Confirmed, S)

`trajectory_planner.cpp` `StreamingPlanner::plan()`: `win`, `junction`, `v_exit` and `v_entry_max` are allocated on
every call, under the controller lock, from the 25 ms tick. Make them members. Check the effect with
`diag::workerTickLockHold()`.

### P5. Serial port on Windows creates an event per read and write (Low, Confirmed, S)

`src/core/transport/serial_port.cpp:115` and `:150`. Keep one read event and one write event as members.

### P6. Smaller items (Low)

- `machine_controller.h:126` `getSliceQueue()` copies the whole queue under the lock. Five call sites; if they are
  tests only, mark it as a test accessor.
- Manual moves copy the whole `MachineConfig` to flip `is_mill` (5 places, see M3).
- The window renders at a fixed frame rate while idle (`app_window.cpp` `runLoop`). Dropping to 10-15 FPS when
  nothing moves and no input arrives would cut idle CPU / GPU use on shop PCs. Optional.
- `CMakeLists.txt:79`: `imgui_demo.cpp` is compiled in but `ShowDemoWindow` is not called anywhere. Removing it
  makes both executables smaller.

---

## M. Maintainability and refactoring

### M1. `machine_controller.cpp`: 5 220 lines, one class, two boards interleaved (L, staged)

About 150 member variables. The RNR (host-planned) and GRBL-family (firmware-planned) paths meet inside some 40
methods as `if (fwActiveLocked()) { ... return; }`.

**Stage 1 (mechanical, low risk):** split the file by topic into several `.cpp` files of the same class:
connection and config, coordinates and tool table, jogging, homing, probing and surface probing, spindle /
outputs / laser, program and MDI, job planning and the motion tick, firmware board glue.

**Stage 2:** an `IMotionBackend` seam with two implementations.
- Slice backend: slice queue, in-flight sequence numbers, program run open / close, settle, continuous-jog ramps,
  the homing and probing state machines.
- Firmware backend: `FirmwarePlannedPath` plus the `fw_*` members and `fwHandleEventLocked()`.

The controller keeps what is shared: state, work frame, tool table, spindle / VFD, checks, scripting. Do Stage 2
only with the board tests and the controller tests green before and after each step.

### M2. Two `main()` functions of about 2 200 lines each (L, staged)

`src/app_mill/main_mill.cpp` and `src/app_cut/main_cut.cpp` are each a single function. Measured on significant
lines (comments and short lines removed, widget IDs normalised): 912 of about 1 530 are shared.

**Extract into `ui_common`:**
- An app shell: settings load, controller / camera / stream server setup, the remote upload callback,
  `saveCurrentSettings`, dirty-flag saving, close handling, the smoke test.
- The settings sections both apps have: Connection, Probing, Probe Settings, Work Offsets, Camera & Remote View,
  Inputs & Outputs, Limits & Homing, Motors, Scripts & Macros.
- The pattern `cfg = controller.getConfig(); ...; if (changed) controller.applyConfig(cfg);`, repeated about ten
  times per app.

The apps keep what differs: spindle / VFD / tool change / tool table (Mill), THC / laser (Cut). The existing panel
classes (`CameraPanel`, `ProbePanel`, ...) are the model.

### M3. Duplicated logic inside the controller (S-M each)

| What | Copies | Suggested helper |
|---|---|---|
| RPM to PWM duty + VFD speed, with `(vfd.enabled && vfd.max_rpm > 0) ? vfd.max_rpm : 24000` | 7 | `applySpindleSpeedLocked(rpm, from_program)`; name the constants 24000 / 12000 / 1000 |
| "A job is active" guard (`motionActiveLocked()` or PausedHold, sometimes ToolChangeHold) | 13 | `jobActiveLocked(include_tool_change)`; write down why some callers allow ToolChangeHold |
| Laser interlock check (door / chiller / PSU fault) | 3 | `laserInterlocksOkLocked(std::string* why)` |
| `MachineConfig manual_cfg = config_; manual_cfg.is_mill = true;` | 5 | A planner option "plain machine move" instead of a config copy |
| Manual tool change sequence | 2 (`sendQueuedSliceLocked` `:4106`, `fwToolChangeLocked` `:4931`) | One function that builds the retract / park moves |
| Parse + height-map warp | 3 (`loadGCodeString`, `reparseLoadedProgramIfNeeded`, `applySurfaceHeightMap`) | `buildActiveProgram(parsed, map)`; today only one of them recomputes the bounding box |
| VFD start / stop / update | 2 (`:552`, `:573`) | `syncVfdLocked()` |
| VFD alarm check in `resetFromEStop()` | 2 (firmware and RNR branch) | Shared pre-check |
| End-of-move state (pending tool: ToolChangeHold, else Idle) | 6 | `restStateLocked()` (see A9) |

### M4. `motion_controller.h` is a catch-all header (M)

859 lines: every configuration struct, the telemetry snapshot, `ToolpathSegment`, `SurfaceHeightMap` with about
120 lines of inline logic (`:139`), and the interface with about 100 methods. Almost every file includes it, so any
change rebuilds everything (the G-code parser needs one struct from it).

Split into `machine_config.h`, `telemetry.h`, `toolpath_segment.h`, `surface_height_map.h/.cpp` and the interface.

### M5. `settings_manager.cpp`: load and save are written out twice (M)

207 `if (key == ...)` lines on load and a parallel block on save; a new setting needs the struct, the load line and
the save line. A table of field descriptors (section, key, member, type, range) used by both would halve the file
and make a mismatch impossible.

**Test to add first:** save, load, compare, for a settings object with every field changed from its default. It
catches a key that is written but not read (or the reverse) today.

### M6. `gcode_parser.cpp` holds three things (M)

- The parser: `parse()` is one function of about 800 lines with some 40 local variables captured by lambdas. A
  `ParserState` struct with one function per block type (modal words, G68, G92, machine moves, canned cycles, arcs)
  would make each part testable.
- Three built-in sample programs (from `:1041`, about 210 lines): move to `gcode_samples.cpp`.
- Machine type detection (`:1257-1519`): move to `machine_type_detect.cpp`.

### M7. `firmware_planned_path.cpp` (2 116 lines) (M-L)

Handshake, settings, console, board report, jog feeder, job streaming and overrides in one class. Natural cuts:
the board report and settings reader, and the job streamer (`startJob`, `produceJobLines`, `pumpJob`,
`emitOutputsFor`, `updateTracker`).

### M8. Tests (M)

- `tests/script_engine_test.cpp` is 3 951 lines and tests the parser, the planner, the controller, the web server
  and Lua under the name "script engine test". Split by area; keep the small harness in a shared header.
- No CTest registration (see B1).
- Gaps found in this review: laser `S0` (A1), VFD stop frame on `stop()` (A2), shipped scripts against the API
  (A3), invalid Lua axis (A4), settings round trip (M5), feed override small steps (A6), MDI rejected without side
  effects (A10).

### M9. Two "where is the executable" implementations (S)

`src/core/scripting/script_path_helper.h` (Windows only; on Linux and macOS it searches the working directory
only) and `src/ui_common/app_dir.cpp`. One cross-platform helper in `core`.

### M10. Smaller items (S)

- `machine_controller.cpp:13-18`: `timeBeginPeriod` is declared by hand and linked with `#pragma comment(lib)`;
  MinGW ignores the pragma. Link `winmm` in CMake like the other system libraries.
- Old names remain in headers and comments ("RnRCncController" in the Lua scripts, `RnRController` as the class the
  apps create, the `rnr_mill_settings.ini` fallback). Harmless; rename when the files are touched.
- `machine_controller.cpp:2453`: a `3.14159` literal in the simulated surface.

---

## B. Build, CI and tooling

### B1. `board_tests` is not run in CI or in the release workflow (Medium, Confirmed, S)

`.github/workflows/ci.yml:60-66` and `release.yml:66-72` run `script_engine_test` and `huanyang_vfd_test` only. The
324 checks for the GRBL-family code do not gate a push or a release.

**Fix** `enable_testing()` and one `add_test()` per test executable in `CMakeLists.txt`, then
`ctest --test-dir build -C Release --output-on-failure` in both workflows. New test executables are then picked up
without touching the workflows.

### B2. No compiler warnings and no sanitizer run (Medium, Confirmed, M)

No `/W4` or `-Wall -Wextra` anywhere. Add an interface target with the warning flags and link it to the project's
own targets only (not GLFW, ImGui, Lua). Fix or silence what shows up once, then add `/WX` / `-Werror` in CI.

Add one Linux CI job with AddressSanitizer and UBSan that runs the three test executables. The code has raw
sockets, hand-written parsers and several threads, which is where those tools pay off. ThreadSanitizer for the
controller tests as a second step.

### B3. Dependencies fetched by tag (Low, Confirmed, S)

`CMakeLists.txt`: GLFW `3.4`, Lua `v5.4.7` and hidapi `hidapi-0.14.0` are tags; only ImGui is pinned to a commit.
Pin all four to commit hashes so a moved tag cannot change a release build.

### B4. Smaller items (Low, S)

- `deploy_scripts` / `deploy_resources` copy both folders twice on every build. Use
  `copy_directory_if_different` (CMake 3.26) or a stamp file.
- No `.clang-format` or `.editorconfig`; lines reach 170 characters. Add a format file that matches the present
  style and apply it to files as they are touched, not in one commit.
- `project(VERSION 0.4)` while `CHANGELOG.md` is collecting 0.5: bump at release.

---

## D. Documentation

- **D1.** `docs/USER_GUIDE.md:113` says HOME ALL runs `scripts/mill/home_all.lua`. By default it runs the built-in
  cycle, and the script does not work (A3).
- **D2.** `docs/architecture/SCRIPTING_ENGINE_SPEC.md` lists `cnc.message`, `cnc.set_current_tool` and
  `cnc.set_work_offset`. None is implemented. Mark the spec as historical or bring it in line with the API.
- **D3.** After A1: say in `docs/GCODE_REFERENCE.md` what `S0` and S above 1000 mean in laser mode.

---

## What already works well

Worth keeping as it is when refactoring:

- Settings are written to a temporary file and renamed, with a `.bak` and an integrity marker.
- `setHardwareAccessEnabled(false)` keeps tests and `--fake-board` away from real hardware.
- `ScriptRunner::startAsync()` never blocks a caller that holds the controller lock.
- `triggerEStop()` halts the board before it takes the controller lock.
- G-code files load and parse on a background thread; parsing never holds the controller lock.
- `GrblProtocol::close()` joins its reader before the stream is closed.
- The program check result is cached by its inputs.

---

## Suggested order

1. **A1** (laser `S0`) and **A2** (VFD stop frame): both are safety fixes with small diffs and clear tests.
2. **B1** (run `board_tests`, CTest): one small change that protects everything after it.
3. **A3**, **A4**, **A6**, **A9**, **A10**, **A11**: small, each with a test.
4. **B2** (warnings, sanitizer job) before the larger refactors, so they run under it.
5. **P1** (configuration snapshot) together with **M2** stage one (read the configuration once per frame).
6. **A5** (UTF-8 paths) as its own change: it touches every file open.
7. **M3** helpers, then **M1** stage 1 (split the file), then **M4**, **M5**, **M6**.
8. **P2**, **M1** stage 2, **M7**: the large ones, each on its own branch.
