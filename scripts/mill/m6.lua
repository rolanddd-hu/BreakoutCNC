-- ============================================================================
-- RnRCncController - Mill Tool Change Macro (M6.lua)
-- Executed automatically on M6 T<n> or manual tool swap
-- ============================================================================

local new_tool = cnc.get_pending_tool()
local old_tool = cnc.get_current_tool()
if new_tool <= 0 then
    new_tool = 1
end

local tc_cfg = (cnc.get_tool_change_config and cnc.get_tool_change_config()) or {}
local safe_z = (tc_cfg.safe_z and tc_cfg.safe_z > 0.0) and tc_cfg.safe_z or 25.0
local cur_pos = cnc.get_work_pos()
local target_safe_z = math.max(safe_z, cur_pos.z)

cnc.log(string.format("=== M6 Tool Change Started: T%d -> T%d ===", old_tool, new_tool))

-- 1. Shut off spindle & lift to Safe Z (in Work Coordinates)
cnc.set_spindle(false)
cnc.log(string.format("Retracting Z to Safe Z (Z=%.3f mm)...", target_safe_z))
cnc.gcode(string.format("G90 G0 Z%.3f", target_safe_z))
cnc.wait_idle()

-- 2. Move to configured operator Park position (ONLY if explicitly enabled in Settings)
if tc_cfg.move_to_park_xy == true then
    local px = tc_cfg.park_x or 0.0
    local py = tc_cfg.park_y or 0.0
    cnc.log(string.format("Moving to configured Tool Change Park position: G53 X%.2f Y%.2f...", px, py))
    cnc.gcode(string.format("G0 G53 X%.3f Y%.3f", px, py))
    cnc.wait_idle()
end

-- 2b. Tool table mode: tool lengths come from the table (G43 H<n> in the program). Z must NOT be
--     touched off here - that would compensate the tool length twice.
local tl_mode = (cnc.get_tool_length_mode and cnc.get_tool_length_mode()) or "not_configured"
if tl_mode == "tool_table" then
    local length = cnc.get_tool_length(new_tool)
    if length == nil then
        local msg = string.format("Insert Tool T%d into spindle chuck.\nT%d has no length in the tool table yet.\n\n" ..
                                  "Jog the tool over the tool setter, then click 'Measure' (or Cancel to abort).", new_tool, new_tool)
        local choice = cnc.prompt("Tool Change (M6)", msg, {"Measure", "Cancel"})
        if choice ~= "Measure" then
            cnc.estop("Tool change cancelled by operator")
            return false
        end
        length = cnc.measure_tool_length(new_tool)
        cnc.log(string.format("Tool T%d measured: %.4f mm", new_tool, length))
        cnc.gcode(string.format("G90 G0 Z%.3f", math.max(safe_z, cnc.get_work_pos().z)))
        cnc.wait_idle()
    else
        local msg = string.format("Insert Tool T%d into spindle chuck (tool table length %.4f mm).\n\n" ..
                                  "No touch-off needed. Click 'Tool Ready' to continue, or Cancel to abort.", new_tool, length)
        local choice = cnc.prompt("Tool Change (M6)", msg, {"Tool Ready", "Cancel"})
        if choice ~= "Tool Ready" then
            cnc.estop("Tool change cancelled by operator")
            return false
        end
    end
    cnc.log(string.format("Tool T%d ready (length offset applied by the program's G43 H%d).", new_tool, new_tool))
    return true
end

-- 3. Prompt operator with modal confirmation dialog
local already_probed_label = string.format("T%d already probed", new_tool)
local prompt_msg = string.format("Insert Tool T%d into spindle chuck.\nAttach touch probe clip to tool and place probe plate underneath.\n\nClick OK to probe Z height, '%s' to skip probing, or Cancel to abort.", new_tool, already_probed_label)
local choice = cnc.prompt("Tool Change (M6)", prompt_msg, {"OK", already_probed_label, "Cancel"})

if choice == already_probed_label then
    cnc.log(string.format("Tool T%d already probed - skipping touch-off and resuming milling cycle.", new_tool))
    return true
elseif choice ~= "OK" then
    cnc.estop("Tool change cancelled by operator")
    return false
end

-- 4. Probe Z height (G31) directly in place over the touch plate
cnc.log("Starting Tool Setter Z probe touch-off...")

local probe_cfg = cnc.get_probe_config()
local fast_feed = (probe_cfg and probe_cfg.fast_feed and probe_cfg.fast_feed > 10.0) and probe_cfg.fast_feed or 300.0
local slow_feed = (probe_cfg and probe_cfg.slow_feed and probe_cfg.slow_feed > 5.0) and probe_cfg.slow_feed or 50.0
local retract_dist = (probe_cfg and probe_cfg.retract and probe_cfg.retract > 0.5) and probe_cfg.retract or 3.0
local plate_thickness = (probe_cfg and probe_cfg.plate_thickness) and probe_cfg.plate_thickness or 0.0

-- 5A. First fast approach touch
cnc.log(string.format("Probing Z touch 1 (Fast: %.0f mm/min)...", fast_feed))
local probe1 = cnc.probe("Z", -100.0, fast_feed)
if not probe1.hit then
    cnc.estop("Tool setter probe missed on initial touch! Switch did not trigger.")
    return false
end

-- Set Z zero immediately on first touch (plate thickness)
cnc.set_work_pos("Z", plate_thickness)
cnc.gcode(string.format("G90 G92 Z%.4f", plate_thickness))
cnc.wait_idle()
cnc.log(string.format("Touch 1 hit at Z=%.3f. Work Z set to %.3f mm. Retracting %.2f mm...", probe1.z, plate_thickness, retract_dist))

-- 5B. Retract slightly for precision touch
cnc.gcode(string.format("G91 G0 Z%.3f", retract_dist))
cnc.wait_idle()
cnc.gcode("G90")
cnc.sleep_ms(200) -- Brief settling delay

-- 5C. Second slow precision touch (checks and sets Z zero accurately again)
cnc.log(string.format("Probing Z touch 2 (Slow precision: %.0f mm/min)...", slow_feed))
local probe2 = cnc.probe("Z", -(retract_dist * 2.5), slow_feed)
if not probe2.hit then
    cnc.estop("Tool setter probe missed on precision touch!")
    return false
end

-- Set Z zero accordingly again with the precise second touch
cnc.set_work_pos("Z", plate_thickness)
cnc.gcode(string.format("G90 G92 Z%.4f", plate_thickness))
cnc.wait_idle()
cnc.log(string.format("Touch 2 hit at Z=%.3f. Work Z calibrated to %.3f mm.", probe2.z, plate_thickness))

-- 6. Retract back to Safe Z (in Work Coordinates)
cnc.log(string.format("Retracting to Safe Z (Z=%.3f mm)...", safe_z))
cnc.gcode(string.format("G90 G0 Z%.3f", safe_z))
cnc.wait_idle()

-- 7. Prompt operator that tool is ready; clicking Tool Ready starts milling!
local ready_msg = string.format("Tool T%d probing complete!\nRemove touch probe clip and touch plate from workpiece.\n\nClick 'Tool Ready' to start milling, or Cancel to abort.", new_tool)
local ready_choice = cnc.prompt("Tool Ready", ready_msg, {"Tool Ready", "Cancel"})
if ready_choice ~= "Tool Ready" then
    cnc.estop("Tool change aborted by operator")
    return false
end

cnc.log(string.format("Tool T%d ready! Starting milling...", new_tool))
return true
