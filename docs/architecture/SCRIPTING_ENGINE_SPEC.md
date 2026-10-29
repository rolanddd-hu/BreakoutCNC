# BreakoutCNC — Extensible Scripting Engine Specification (Mach3/Mach4 Architecture)

## 1. Executive Summary & Objective

This specification details the architecture, language selection, API surface, concurrency model, and integration plan for introducing **extensible scripting capabilities** to **BreakoutCNC** (`BreakoutMill` and `BreakoutCut`).

The goal is to provide CNC operators and machine integrators with an automation and customization framework similar to Mach3 and Mach4:
1. **Event Hooks:** Automatically execute scripts on tool change (`M6`), homing buttons (`HOME ALL`, per-axis `H`), probe routines, and plasma torch-on (`M3` / Initial Height Sense).
2. **Custom M-Codes:** Execute arbitrary user macro scripts when encountering `M100`..`M999` in G-code or MDI (e.g. pneumatic clamps, vacuum tables, mist spray, laser pointers).
3. **UI Macro Buttons & REPL:** User-configurable quick-action buttons in the UI and a live Lua console.
4. **Cross-Profile Support:** Shared core engine with tailored profiles and scripts for both CNC Milling and CNC Plasma Cutting.

---

## 2. Research & Comparative Evaluation: Mach3 vs. Mach4 vs. Modern CNC

### 2.1 The Mach3 Legacy (Cypress Enable / VBScript)
Mach3 used **Cypress Enable**, an embedded Visual Basic for Applications (VBA) / VBScript interpreter:
- **Macro Files:** File-based naming conventions in `C:\Mach3\macros\<ProfileName>\`:
  - `M6Start.m1s` and `M6End.m1s` for tool changes.
  - `M1000.m1s`..`M9999.m1s` for user macros.
  - `M3.m1s`, `M5.m1s`, etc.
- **Button Scripts:** Screen designer allowed attaching raw VB script code to any button.
- **Core Functions:**
  - `Code "G0 X10 Y20"` (send G-code string)
  - `While IsMoving() : Sleep 10 : Wend` (wait for physical motion)
  - `GetDRO(id)`, `SetDRO(id, val)`, `GetOEMLED(id)`, `DoOEMButton(id)`
  - `ActivateSignal(OUTPUT1)`, `DeActivateSignal(OUTPUT1)`
- **Fatal Deficiencies of Mach3's Approach:**
  1. **UI Freezing & Thread Lockups:** Mach3 executed VBScript synchronously on the Windows message thread. Without careful `Sleep()` calls inside `While IsMoving()`, the entire UI locked up and Windows declared the application "Not Responding".
  2. **Cryptic "Magic Numbers":** APIs relied on hundreds of undocumented integer OEM codes (`DoOEMButton(1022)` = Home All, `GetDRO(800)` = X pos, `GetOEMLED(800)` = Spindle ON). Operators required printed cheat sheets to write basic scripts.
  3. **Antiquated Language:** VBScript lacks modern data structures, closures, modules, exception safety, and has been officially deprecated by Microsoft.

### 2.2 The Mach4 Evolution (Lua Engine)
When Newfangled Solutions designed Mach4 from the ground up, they **abandoned VBScript entirely and adopted Lua**:
- Lua 5.2/5.3 was selected for its ultra-small footprint (<300 KB), lightning-fast startup, seamless C/C++ embedding, coroutines for non-blocking asynchronous execution, and modern table-based syntax.
- Macro structure:
  - `Profiles/<Profile>/Macros/m6.lua`
  - `Profiles/<Profile>/Macros/m101.lua`
  - Named API: `mc.mcCntlGcodeExecute(inst, "G0 X0 Y0")`, `mc.mcAxisGetPos(...)`, `mc.mcSignalGetState(...)`.
  - Periodic background PLC script (`Pmc.lua`) running every 50ms for background interlocks.

### 2.3 Evaluation of Scripting Languages for C++20 / Windows

| Criteria | **Lua 5.4 (Recommended)** | **Python (CPython)** | **JavaScript (QuickJS)** | **VBScript / Basic** |
| :--- | :--- | :--- | :--- | :--- |
| **Footprint / Binary Size** | **~300 KB** (ANSI C, ~30 files) | 50 MB+ (DLLs, stdlib, zip) | ~1.5 MB | Antiquated COM / DLL |
| **Build Integration** | **Native CMake FetchContent** (5s build) | Complex (Python headers/libs) | CMake build | Windows-only COM |
| **Startup / Exec Latency** | **Instant (< 1 ms)** | Slow (100–300 ms) | Fast (< 5 ms) | Moderate |
| **CNC Community Standard** | **Yes (Mach4 standard)** | LinuxCNC remap only | Rare | Mach3 legacy only |
| **Thread Safety & Async** | **Clean coroutines & hooks** | GIL issues across threads | Event loop required | Unsafe / locks UI |
| **Syntactic Ergonomics** | **Clean, simple table syntax** | Indentation-sensitive | Brackets, async/await | Verbose, 1990s syntax |

**Conclusion:** **Lua 5.4** is the undisputed optimal choice for `BreakoutCNC`. It preserves the familiar Mach4 standard while offering seamless integration with C++20 and CMake.

---

## 3. Concurrency & Threading Architecture

A fundamental design failure in Mach3 was executing scripts on the GUI thread. In `BreakoutCNC`, we employ a **strictly decoupled 3-tier threading architecture**:

```
+-------------------------------------------------------------------------+
|                         Tier 1: GUI Thread                              |
|   - Dear ImGui + GLFW @ 60 FPS (Zero Stutter, Always Responsive)        |
|   - Toolpath 3D Viewport, DRO readouts, Feed sliders, Alarm banners     |
|   - Renders Script Console & Modal Prompts (e.g. "Insert Tool T2")      |
+------------------------------------+------------------------------------+
                                     | Dispatches non-blocking script job
                                     v
+-------------------------------------------------------------------------+
|                     Tier 2: Script Worker Thread                        |
|   - Dedicated `std::thread` managed by `ScriptRunner`                   |
|   - Runs Lua 5.4 VM with cooperative cancellation hook                  |
|   - Calls `cnc.gcode()`, `cnc.wait_idle()`, `cnc.sleep_ms()`            |
|   - Yields/sleeps without blocking Tier 1 (GUI) or Tier 3 (Pulses)      |
|   - Aborts instantly if E-Stop, Stop, or script timeout occurs          |
+------------------------------------+------------------------------------+
                                     | Dispatches motion & queries state
                                     v
+-------------------------------------------------------------------------+
|                  Tier 3: Real-Time Motion / USB Engine                  |
|   - 40 Hz / 25 ms Worker Thread (`RnRController::workerLoop`)           |
|   - 1000 Hz USB HID packet transmission to STM32 (`rnr_usb_device`)     |
|   - Asynchronous RS485 Huanyang VFD thread                              |
|   - Hardware and simulated input/output bitmasks                        |
+-------------------------------------------------------------------------+
```

### 3.1 Non-Blocking Synchronization (`cnc.wait_idle()`)
When a script commands a move and waits for it to complete:
```lua
cnc.gcode("G0 G53 Z0") -- Safe Z
cnc.wait_idle()        -- Blocks script thread ONLY; UI and USB stay at 60 FPS / 40 Hz
```
Under the hood in C++:
```cpp
void ScriptApi::waitIdle(double timeout_seconds) {
    auto start = std::chrono::steady_clock::now();
    while (controller_.getTelemetry().state != ControllerState::Idle) {
        if (runner_.isCancellationRequested()) {
            throw ScriptCancelledException("Script aborted by operator");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (timeout_seconds > 0.0) {
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > timeout_seconds) throw ScriptTimeoutException("Motion timed out");
        }
    }
}
```

### 3.2 Instant Cooperative E-Stop Cancellation
To guarantee safety, the Lua VM attaches a debug instruction count hook (`lua_sethook` every 100 instructions).
If the operator presses **E-STOP** or **STOP** on the machine or in the UI:
1. `runner_.cancel()` sets `cancel_requested_ = true`.
2. The instruction hook detects the flag and invokes `luaL_error(L, "Script aborted by E-STOP")`.
3. The script terminates immediately, all motion is aborted via `controller_.triggerEStop()`, and outputs are de-energized.

---

## 4. Lua `cnc.*` API Specification

The scripting engine registers a clean, comprehensive `cnc` table in the Lua global environment.

### 4.1 Motion & G-Code Execution
- `cnc.gcode(string)`: Enqueues and executes a line of G-code or MDI (e.g. `cnc.gcode("G0 X10 Y20")`).
- `cnc.wait_idle([timeout_sec])`: Sleeps the script worker thread until all motion and dwells complete.
- `cnc.is_moving()`: Returns `boolean` (`true` if machine velocity $> 0$ or state is `RunningGCode`).
- `cnc.sleep_ms(milliseconds)`: Delay helper (sleeps the script worker thread safely).
- `cnc.stop()`: Halts motion cycle.
- `cnc.pause()`: Feed hold.
- `cnc.resume()`: Resumes paused motion.

### 4.2 Positions, Offsets & Coordinates
- `cnc.get_work_pos()`: Returns `{x = num, y = num, z = num, a = num}` in active work coordinates (mm).
- `cnc.get_machine_pos()`: Returns `{x = num, y = num, z = num, a = num}` in absolute machine coordinates (mm).
- `cnc.zero_axis(axis_name_or_index)`: Zeros work coordinate for specified axis (`"X"`, `"Y"`, `"Z"`, `"A"` or `0..3`).
- `cnc.zero_all()`: Zeros all work axes.
- `cnc.set_work_offset(axis, offset_mm)`: Sets work offset for specified axis.
- `cnc.is_homed(axis)`: Returns `boolean` indicating if axis has been referenced.

### 4.3 Digital I/O & Peripherals
- `cnc.get_input(pin_1_to_12)`: Returns `boolean` state of input pin `IN1..IN12`.
- `cnc.get_inputs_mask()`: Returns raw 16-bit input bitmask.
- `cnc.set_output(pin_1_to_4, bool)`: Directly drives auxiliary outputs `OUT1..OUT4`.
- `cnc.get_output(pin_1_to_4)`: Returns `boolean` state of `OUT1..OUT4`.

### 4.4 Spindle, VFD & Plasma Torch
- `cnc.set_spindle(bool)`: Turns mill spindle ON/OFF (`OUT1`).
- `cnc.is_spindle_on()`: Returns spindle state.
- `cnc.set_spindle_rpm(rpm)`: Updates commanded RPM, 0-10V analog PWM, and Huanyang RS485 frequency.
- `cnc.get_spindle_rpm()`: Returns active commanded RPM.
- `cnc.set_torch(bool)`: Turns plasma torch ON/OFF (`OUT1`).
- `cnc.is_torch_on()`: Returns torch state.

### 4.5 Probing & Sensing (G31)
- `cnc.probe(axis, max_travel_mm, feed_mm_min)`: Executes straight probing move until contact switch trips. Returns table `{hit = true/false, x = ..., y = ..., z = ..., a = ...}`.
- `cnc.is_probe_tripped()`: Reads instantaneous touch probe switch state.
- `cnc.get_last_probe_pos()`: Returns coordinates of last probe latch.

### 4.6 Tooling & Machine Modals
- `cnc.get_current_tool()`: Returns active tool number `T#`.
- `cnc.get_pending_tool()`: Returns tool number requested by `M6 T#`.
- `cnc.set_current_tool(num)`: Sets active tool number.
- `cnc.get_feed_override()`: Returns active Feed% (e.g. `100.0`).
- `cnc.set_feed_override(pct)`: Overrides feed rate percentage.

### 4.7 Safety & E-Stop
- `cnc.estop([reason])`: Immediately trips E-Stop, shuts down motion & outputs, and registers an alarm message.
- `cnc.is_estop()`: Returns `boolean`.
- `cnc.reset_estop()`: Attempts to reset E-Stop.

### 4.8 Logging & User Interaction
- `cnc.log(string)` / `print(string)`: Prints message to the application's Script Console log.
- `cnc.message(string)`: Displays transient message on the status bar.
- `cnc.prompt(title, message, {buttons})`: Opens a modal dialog in the UI and blocks script thread until operator clicks a button. Returns button string (e.g. `"OK"`, `"Cancel"`).

### 4.9 Mach3 Compatibility Shim
To make transition effortless for users with existing Mach3 script knowledge, a built-in helper shim provides:
```lua
function Code(g) cnc.gcode(g) end
function WhileIsMoving() cnc.wait_idle() end
function Sleep(ms) cnc.sleep_ms(ms) end
function GetDRO(id) ... end
function SetDRO(id, val) ... end
function DoOEMButton(id) ... end
function Message(msg) cnc.message(msg) end
```

---

## 5. Event Hooks & Integration Points

### 5.1 Tool Change Hook (`M6` / `m6.lua`)
- **Trigger:** When G-code encounters `M6 T<n>`, or operator commands MDI `M6 T<n>`.
- **Logic:**
  1. If `config.scripting.m6_script_enabled` is true and `scripts/<profile>/m6.lua` exists:
  2. The controller halts cutting, turns off spindle, and launches `m6.lua` on the script worker thread, passing `new_tool` and `old_tool` as arguments.
  3. Example `scripts/mill/m6.lua`:
     ```lua
     local new_tool = cnc.get_pending_tool()
     local old_tool = cnc.get_current_tool()
     cnc.log(string.format("Executing Tool Change: T%d -> T%d", old_tool, new_tool))

     -- 1. Stop spindle & lift to Safe Z
     cnc.set_spindle(false)
     cnc.gcode("G0 G53 Z0")
     cnc.wait_idle()

     -- 2. Move to tool change position
     cnc.gcode("G0 G53 X50 Y200")
     cnc.wait_idle()

     -- 3. Prompt operator
     local choice = cnc.prompt("Tool Change", string.format("Insert Tool T%d and click OK to touch off", new_tool), {"OK", "Cancel"})
     if choice ~= "OK" then
         cnc.estop("Tool change cancelled by operator")
         return false
     end

     -- 4. Move to Tool Setter and probe Z
     cnc.gcode("G0 G53 X20 Y20") -- Tool setter XY
     cnc.wait_idle()
     local res = cnc.probe("Z", -80.0, 300.0)
     if not res.hit then
         cnc.estop("Tool setter probe missed! Switch did not trigger.")
         return false
     end

     -- 5. Set tool height offset & retract
     cnc.gcode("G0 G53 Z0")
     cnc.wait_idle()
     cnc.set_current_tool(new_tool)
     cnc.log("Tool Change Complete!")
     return true
     ```
  4. If script returns `true`, the job automatically resumes cutting!

### 5.2 Homing Hook (`home_all.lua` / `home_axis.lua`)
- **Trigger:** Operator clicks `HOME ALL` or per-axis `H` on the DRO panel.
- **Benefit:** Allows custom multi-stage homing routines (e.g. Plasma gantry auto-squaring with dual limit switches, slow double-tap touch off, laser crosshair registration).
- If script homing is disabled in settings, defaults to native C++ homing.

### 5.3 Custom M-Codes (`M100`..`M999`)
- **Trigger:** Parser or MDI encounters any M-code not reserved by standard RS-274 (`M100`..`M999`).
- **Resolution:** Checks for `scripts/macros/M<number>.lua` (e.g. `scripts/macros/M101.lua`).
- **Execution:** Runs the file synchronously with respect to G-code execution.

### 5.4 Plasma Initial Height Sense Hook (`m3_touch.lua`)
- **Trigger:** Prior to plasma arc ignition on `M3`.
- **Sequence:** Probe downward until Ohmic sensor or Floating Head switch trips, back off by switch travel + pierce height, then ignite torch!

---

## 6. UI Components: Script Console, Macro Buttons & REPL

A new shared UI panel `ScriptingPanel` (`rnr_ui_common`) will provide:
1. **Interactive Lua REPL / MDI:** Single-line text input allowing operators to test Lua expressions or commands on the fly (e.g. `cnc.zero_axis("Z")` or `cnc.set_spindle_rpm(18000)`).
2. **Scrollable Color-Coded Script Console:** Displays timestamps, `cnc.log()` outputs, return values, and syntax/runtime error diagnostics in amber/red.
3. **Quick Macro Deck:** 4 customizable buttons with user-configurable labels and script file bindings.
4. **Interactive Prompt Modal:** Modal dialog triggered by `cnc.prompt()` for interactive operator confirmations.

---

## 7. Implementation Roadmap

### Phase 1: Engine & Build Integration
- Add Lua 5.4.7 static library via CMake `FetchContent`.
- Implement `ScriptEngine` (Lua VM management, standard library sandbox).
- Implement `ScriptApi` (binding all `IMotionController` and telemetry functions).
- Implement `ScriptRunner` (asynchronous worker thread, cancellation hooks, condition variables).
- Unit test suite: `tests/script_engine_test.cpp`.

### Phase 2: Controller & Hook Integration
- Integrate `ScriptRunner` into `RnRController`.
- Implement `M6` tool change script hook in trajectory planner & slice loop.
- Implement `M100..M999` custom macro dispatcher in G-Code parser and MDI.
- Implement homing script hook override.

### Phase 3: UI Console, Quick Buttons & Modals
- Implement `ScriptingPanel` in `rnr_ui_common`.
- Integrate into `BreakoutMill` and `BreakoutCut`.
- Wire `cnc.prompt()` modal dialog into Dear ImGui render loop.

### Phase 4: Standard Scripts & Settings Persistence
- Author standard script templates:
  - `scripts/mill/m6.lua`
  - `scripts/mill/probe_z.lua`
  - `scripts/mill/spindle_warmup.lua`
  - `scripts/cut/m3_touch.lua`
  - `scripts/cut/home_all.lua`
  - `scripts/macros/M100_clamp.lua`
  - `scripts/macros/M101_unclamp.lua`
- Persist script paths and macro button configuration in `SettingsManager` (`breakout_mill_settings.ini` / `breakout_cut_settings.ini`).
- Full end-to-end verification and testing.
