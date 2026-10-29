#pragma once
// Example scripts shown in the Lua guide (ui_common/lua_guide.cpp). Kept in a header without UI
// dependencies so the unit tests can syntax-check and run every example.

namespace rnr::ui::guide_examples {

inline constexpr const char* kFirstScript = R"LUA(-- My first macro: move to a safe height and report where we are
cnc.log("Hello from Lua!")

cnc.gcode("G90 G0 Z10")   -- queue one MDI command...
cnc.wait_idle()           -- ...and wait until the machine has finished it

local p = cnc.get_work_pos()
cnc.log(string.format("Now at X%.3f Y%.3f Z%.3f", p.x, p.y, p.z)))LUA";

inline constexpr const char* kDlgConfirm = R"LUA(-- Yes / No question (demo: only writes to the log)
local answer = cnc.prompt("Coolant", "Turn the flood coolant on?", {"Yes", "No"})
if answer == "Yes" then
    cnc.log("(demo) here you would call cnc.set_output(3, true)")
else
    cnc.log("Coolant left off")
end)LUA";

inline constexpr const char* kDlgMenu = R"LUA(-- A menu: cnc.prompt returns the label of the clicked button
local choice = cnc.prompt("Select Material",
    "Which material is clamped on the table?",
    {"Aluminium", "Hardwood", "MDF / Plywood", "Cancel"})

local rpm_for = { ["Aluminium"] = 18000, ["Hardwood"] = 16000, ["MDF / Plywood"] = 14000 }
if rpm_for[choice] then
    cnc.log(string.format("%s -> %d RPM", choice, rpm_for[choice]))
else
    cnc.log("No material selected")   -- "Cancel" (also returned if the script is cancelled)
end)LUA";

inline constexpr const char* kDlgText = R"LUA(-- Text input: returns the text, or nil when the operator presses Cancel
local name = cnc.input("Job Name", "Name of this job (used in the log):", "bracket-v2")
if name == nil then
    cnc.log("Cancelled")
    return
end
if name == "" then name = "unnamed" end
cnc.log("Starting job: " .. name))LUA";

inline constexpr const char* kDlgNumber = R"LUA(-- Number input with limits: OK is only enabled for 0.1 .. 25
local depth = cnc.input_number("Pocket Depth", "Total depth of cut (mm):", 3.0, 0.1, 25.0)
if not depth then return end           -- nil = Cancel

local passes = math.ceil(depth / 1.0)
cnc.log(string.format("Depth %.2f mm -> %d passes of %.2f mm", depth, passes, depth / passes)))LUA";

inline constexpr const char* kDlgWizard = R"LUA(-- Multi-step wizard with a summary page and an "Edit" loop
local p = { width = 100, height = 60, step = 5 }

local function ask(key, title, text, lo, hi)
    local v = cnc.input_number(title, text, p[key], lo, hi)
    if v == nil then return false end   -- Cancel
    p[key] = v
    return true
end

while true do
    if not ask("width",  "Step 1 of 3", "Area width X (mm):", 1, 1000) then return end
    if not ask("height", "Step 2 of 3", "Area length Y (mm):", 1, 1000) then return end
    if not ask("step",   "Step 3 of 3", "Stepover (mm):", 0.1, 50) then return end

    local summary = string.format("Width: %.1f mm\nLength: %.1f mm\nStepover: %.1f mm\n\n%d passes",
        p.width, p.height, p.step, math.ceil(p.height / p.step) + 1)
    local c = cnc.prompt("Summary", summary, {"Start", "Edit", "Cancel"})
    if c == "Start" then break end
    if c ~= "Edit" then return end      -- Cancel
end
cnc.log("Wizard finished - ready to cut"))LUA";

inline constexpr const char* kDlgChecklist = R"LUA(-- Pre-flight checklist: every item must be confirmed
local checks = {
    "Workpiece clamped firmly?",
    "Correct tool in the spindle?",
    "Work zero (X, Y, Z) set?",
    "Dust extraction on?",
}
for i, question in ipairs(checks) do
    local a = cnc.prompt(string.format("Checklist %d/%d", i, #checks), question, {"Yes", "No"})
    if a ~= "Yes" then
        cnc.prompt("Checklist", "Not ready:\n" .. question .. "\n\nFix it and run the checklist again.", {"OK"})
        return
    end
end
cnc.log("Checklist complete - all items confirmed"))LUA";

inline constexpr const char* kDlgError = R"LUA(-- Show a rejected command to the operator (cnc.gcode raises an error when
-- the controller refuses a command: busy, invalid G-code, soft limits, unhomed G53 ...)
local ok, err = pcall(cnc.gcode, "G53 G0 Z-2")
if not ok then
    cnc.prompt("Move Rejected", tostring(err), {"OK"})
    return
end
cnc.wait_idle())LUA";

inline constexpr const char* kExWarmup = R"LUA(-- Spindle warm-up: the operator picks the length, then 4 speed steps
local choice = cnc.prompt("Spindle Warm-Up", "Select warm-up length:",
                          {"Short (2 min)", "Long (8 min)", "Cancel"})
if choice == "Cancel" then return end
local stage_s = (choice == "Short (2 min)") and 30 or 120
local speeds = { 6000, 12000, 18000, 24000 }

cnc.set_spindle_rpm(speeds[1])
cnc.set_spindle(true)
for i, rpm in ipairs(speeds) do
    cnc.set_spindle_rpm(rpm)
    cnc.log(string.format("Stage %d/%d: %d RPM for %d s", i, #speeds, rpm, stage_s))
    cnc.sleep_ms(stage_s * 1000)        -- E-STOP / Cancel ends the script right here
end
cnc.set_spindle(false)
cnc.log("Warm-up complete"))LUA";

inline constexpr const char* kExTouchPlate = R"LUA(-- Z touch-off with a touch plate (fast touch, back off, slow touch)
local cfg = cnc.get_probe_config()
local plate = cnc.input_number("Touch Plate", "Plate thickness (mm):", cfg.plate_thickness, 0, 50)
if not plate then return end

if cnc.is_probe_tripped() then
    cnc.prompt("Probe", "The probe input is already active.\nCheck the clip and the plate.", {"OK"})
    return
end
if cnc.prompt("Touch Plate", "Place the plate under the tool and attach the clip.",
              {"Probe", "Cancel"}) ~= "Probe" then
    return
end

-- 1. Fast search downwards (a negative distance moves towards -Z)
local hit = cnc.probe("Z", -cfg.max_travel, cfg.fast_feed)
if not hit.hit then
    cnc.prompt("Probe", string.format("No contact within %.1f mm.", cfg.max_travel), {"OK"})
    return
end
-- 2. Back off, 3. slow and precise second touch
cnc.gcode(string.format("G91 G0 Z%.3f", cfg.retract))
cnc.wait_idle()
hit = cnc.probe("Z", -(cfg.retract * 2), cfg.slow_feed)
if not hit.hit then cnc.estop("Lost contact on the slow touch") return end

-- 4. The tool tip sits on the plate: that height is Z = plate thickness
cnc.set_work_pos("Z", plate)
cnc.gcode(string.format("G90 G0 Z%.3f", plate + 10))
cnc.wait_idle()
cnc.prompt("Touch Plate", "Z zero set. Remove the plate and the clip.", {"OK"}))LUA";

inline constexpr const char* kExGrid = R"LUA(-- Drill a grid of holes (G83 peck drilling) from operator input
local cols  = cnc.input_number("Hole Grid", "Holes in X:", 4, 1, 50)            if not cols  then return end
local rows  = cnc.input_number("Hole Grid", "Holes in Y:", 3, 1, 50)            if not rows  then return end
local pitch = cnc.input_number("Hole Grid", "Spacing (mm):", 20, 1, 500)        if not pitch then return end
local depth = cnc.input_number("Hole Grid", "Depth (mm, positive):", 5, 0.1, 50) if not depth then return end
cols, rows = math.floor(cols), math.floor(rows)

local msg = string.format("%d x %d holes, %.1f mm apart, %.1f mm deep.\nFirst hole = current X/Y position.",
                          cols, rows, pitch, depth)
if cnc.prompt("Hole Grid", msg, {"Drill", "Cancel"}) ~= "Drill" then return end

local start = cnc.get_work_pos()
-- One cnc.gcode() per command, and wait_idle() after each: the next command is only
-- accepted when the machine is idle again.
local function run(fmt, ...)
    cnc.gcode(string.format(fmt, ...))
    cnc.wait_idle()
end

cnc.set_spindle(true)
cnc.sleep_ms(3000)                      -- let the spindle reach speed
run("G90 G0 Z5")
for r = 0, rows - 1 do
    for c = 0, cols - 1 do
        run("G98 G83 X%.3f Y%.3f Z%.3f R2 Q1.5 F150",
            start.x + c * pitch, start.y + r * pitch, -depth)
    end
end
cnc.set_spindle(false)
cnc.log(string.format("Drilled %d holes", rows * cols)))LUA";

inline constexpr const char* kExFacing = R"LUA(-- Face (surface) a rectangle in zig-zag passes, starting at the current X/Y
local w    = cnc.input_number("Facing", "Width in X (mm):", 100, 1, 2000)    if not w    then return end
local h    = cnc.input_number("Facing", "Length in Y (mm):", 60, 1, 2000)    if not h    then return end
local step = cnc.input_number("Facing", "Stepover (mm):", 8, 0.1, 50)        if not step then return end
local doc  = cnc.input_number("Facing", "Depth of cut (mm):", 0.5, 0.05, 5)  if not doc  then return end
local feed = cnc.input_number("Facing", "Feed (mm/min):", 1500, 50, 6000)   if not feed then return end

local p0 = cnc.get_work_pos()
local function run(fmt, ...) cnc.gcode(string.format(fmt, ...)) cnc.wait_idle() end

cnc.set_spindle(true)
cnc.sleep_ms(3000)
run("G90 G0 Z2")
run("G1 Z%.3f F300", -doc)
local y, dir = 0, 1
while true do
    run("G1 X%.3f Y%.3f F%.0f", (dir == 1) and (p0.x + w) or p0.x, p0.y + y, feed)
    if y >= h then break end
    y = math.min(y + step, h)
    run("G1 Y%.3f F%.0f", p0.y + y, feed)
    dir = -dir
end
run("G0 Z10")
cnc.set_spindle(false))LUA";

inline constexpr const char* kExPark = R"LUA(-- Park at a fixed machine position. G53 moves need homed axes.
for _, axis in ipairs({"X", "Y", "Z"}) do
    if not cnc.is_homed(axis) then
        cnc.prompt("Park", axis .. " is not homed. Home the machine first.", {"OK"})
        return
    end
end
cnc.set_spindle(false)
cnc.gcode("G53 G0 Z-2")       -- just below the Z home switch (adjust for your machine)
cnc.wait_idle()
cnc.gcode("G53 G0 X10 Y10")
cnc.wait_idle()
cnc.log("Parked"))LUA";

inline constexpr const char* kExMeasureTools = R"LUA(-- Measure several tools for the tool table (Tool Length Mode = Tool table)
if cnc.get_tool_length_mode() ~= "tool_table" then
    cnc.prompt("Tool Table", "Set Tool Length Mode to 'Tool table' first.", {"OK"})
    return
end
local list = cnc.input("Measure Tools",
    "Tool numbers to measure, comma separated.\nStart with the reference tool:", "1,2,3")
if not list then return end
if cnc.prompt("Measure Tools", "Position the spindle over the tool setter.",
              {"Start", "Cancel"}) ~= "Start" then return end

for num in string.gmatch(list, "%d+") do
    local t = tonumber(num)
    cnc.gcode("G91 G0 Z30") cnc.wait_idle()          -- room to change the tool
    local a = cnc.prompt("Measure T" .. t, "Insert tool T" .. t .. " into the spindle.",
                         {"Measure", "Skip", "Stop"})
    if a == "Stop" then break end
    if a == "Measure" then
        local len = cnc.measure_tool_length(t)       -- probes down, stores the length
        cnc.log(string.format("T%d length = %+.4f mm", t, len))
    end
end
cnc.prompt("Measure Tools", "Done. Lengths are saved in the tool table.", {"OK"}))LUA";

inline constexpr const char* kExWaitInput = R"LUA(-- Wait for a sensor (e.g. vacuum / clamp pressure on IN6) with a timeout
local PIN, TIMEOUT_MS = 6, 10000
cnc.set_output(4, true)                     -- OUT4: clamp valve (example wiring)
local waited = 0
while not cnc.get_input(PIN) do
    if waited >= TIMEOUT_MS then
        cnc.set_output(4, false)
        cnc.prompt("Clamp", "No pressure after 10 s.\nCheck the air supply.", {"OK"})
        return
    end
    cnc.sleep_ms(100)
    waited = waited + 100
end
cnc.log("Clamp pressure OK"))LUA";

inline constexpr const char* kExJobLog = R"LUA(-- Append a line to a CSV job log (io.open works; os.execute / io.popen do not)
local name = cnc.input("Job Log", "Part / job name:", "")
if not name then return end
local p = cnc.get_work_pos()
local f = io.open("job_log.csv", "a")
if f then
    f:write(string.format("%s,%s,%.3f,%.3f,%.3f\n", os.date("%Y-%m-%d %H:%M:%S"), name, p.x, p.y, p.z))
    f:close()
    cnc.log("Logged " .. name)
else
    cnc.prompt("Job Log", "Could not open job_log.csv for writing.", {"OK"})
end)LUA";

inline constexpr const char* kHookM6 = R"LUA(-- scripts/mill/m6.lua  (Settings > Lua Scripting > M6 script)
local new_tool = cnc.get_pending_tool()

cnc.set_spindle(false)
cnc.gcode("G90 G0 Z25")
cnc.wait_idle()

if cnc.prompt("Tool Change", "Insert tool T" .. new_tool .. ".", {"Done", "Cancel"}) ~= "Done" then
    cnc.estop("Tool change cancelled")   -- an error / E-STOP keeps the job from resuming
end
-- Reaching the end without an error resumes the job automatically.
-- (Touch off Z here when using "Touch off Z after every tool change";
--  in "Tool table" mode the program's G43 H<n> applies the tool length instead.))LUA";

inline constexpr const char* kHookMacro = R"LUA(-- scripts/macros/M101.lua  ->  type M101 in MDI or bind it to a macro button
cnc.log("M101: vacuum table ON")
cnc.set_output(4, true)
cnc.sleep_ms(500))LUA";

inline constexpr const char* kMach3Example = R"LUA(-- Legacy Mach3-style macro running unchanged
Message("Setting Z with a 15 mm plate...")
Code("G90 G0 Z25")
WhileIsMoving()
local plate = Question("Plate thickness (mm)?")   -- number input dialog (nil = Cancel)
if plate then
    SetDRO(2, plate)          -- 2 = Z: current position reads <plate>
    MsgBox(string.format("Z set to %.3f", plate))
end)LUA";

struct Example {
    const char* id;
    const char* code;
};

// Every example in the guide (for tests)
inline constexpr Example kAll[] = {
    {"first", kFirstScript},       {"dlg_confirm", kDlgConfirm},   {"dlg_menu", kDlgMenu},
    {"dlg_text", kDlgText},        {"dlg_number", kDlgNumber},     {"dlg_wizard", kDlgWizard},
    {"dlg_checklist", kDlgChecklist}, {"dlg_error", kDlgError},    {"ex_warmup", kExWarmup},
    {"ex_touch", kExTouchPlate},   {"ex_grid", kExGrid},           {"ex_facing", kExFacing},
    {"ex_park", kExPark},          {"ex_tools", kExMeasureTools},  {"ex_wait", kExWaitInput},
    {"ex_log", kExJobLog},         {"hook_m6", kHookM6},           {"hook_macro", kHookMacro},
    {"mach3", kMach3Example},
};

} // namespace rnr::ui::guide_examples
