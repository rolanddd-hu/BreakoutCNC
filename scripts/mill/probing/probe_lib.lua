-- ============================================================================
-- RnRCncController - Mill Probing Macro Library (probe_lib.lua)
-- Reusable routines for Outside, Inside, Center (Web/Slot/Bore/Boss), & Z touch
-- ============================================================================

local M = {}

-- Retrieve probing parameters configured in the UI (with safe fallbacks)
function M.get_config()
    local cfg = cnc.get_probe_config() or {}
    return {
        tip_dia       = cfg.tip_diameter or 6.0,
        tip_radius    = (cfg.tip_diameter or 6.0) * 0.5,
        plate_h       = cfg.plate_thickness or 15.0,
        fast_feed     = cfg.fast_feed or 300.0,
        slow_feed     = cfg.slow_feed or 60.0,
        max_travel    = cfg.max_travel or 40.0,
        retract       = cfg.retract or 3.0,
        diameter      = cfg.diameter or 25.0,
        xy_clearance  = cfg.xy_clearance or 10.0,
        probe_depth   = cfg.probe_depth or 6.0,
        size_x        = cfg.size_x or 50.0,
        size_y        = cfg.size_y or 50.0,
    }
end

-- Core two-stage probing along a single axis
-- Returns probed coordinate (number) on success, or nil on failure
function M.touch(axis, travel_dir, max_dist)
    axis = string.upper(axis)
    local cfg = M.get_config()
    max_dist = max_dist or cfg.max_travel
    local signed_travel = (travel_dir >= 0) and max_dist or -max_dist

    if cnc.is_probe_tripped() then
        cnc.log("Probe aborted: Touch probe is already in contact!", true)
        return nil
    end

    -- 1. Fast approach
    local res = cnc.probe(axis, signed_travel, cfg.fast_feed)
    if not res.hit then
        cnc.log(string.format("Probe %s: No contact within %.1f mm search travel.", axis, max_dist), true)
        return nil
    end

    -- 2. Small retract back
    local back_dir = (travel_dir >= 0) and -cfg.retract or cfg.retract
    cnc.gcode(string.format("G91 G0 %s%.3f", axis, back_dir))
    cnc.wait_idle()
    cnc.sleep_ms(80)

    -- 3. Slow precision latch
    local latch_travel = (travel_dir >= 0) and (cfg.retract * 1.6) or -(cfg.retract * 1.6)
    res = cnc.probe(axis, latch_travel, cfg.slow_feed)
    if not res.hit then
        cnc.log(string.format("Probe %s: Precision latch failed.", axis), true)
        return nil
    end

    local hit_pos = res[string.lower(axis)]

    -- 4. Retract back to start clearance
    cnc.gcode(string.format("G91 G0 %s%.3f", axis, back_dir))
    cnc.wait_idle()
    cnc.gcode("G90")

    return hit_pos
end

-- ----------------------------------------------------------------------------
-- PROBE Z ROUTINE (Tool Setter / Touch Plate or Direct Workpiece Surface)
-- ----------------------------------------------------------------------------
function M.probe_z(on_workpiece)
    local cfg = M.get_config()
    local is_direct_workpiece = (on_workpiece == true)
    local z_target = is_direct_workpiece and 0.0 or cfg.plate_h

    if is_direct_workpiece then
        cnc.log("Starting Z Direct Workpiece Touch-Off (Z=0 on metal stock)...")
    else
        cnc.log(string.format("Starting Z Tool Setter Touch-Off (Plate: %.3f mm)...", cfg.plate_h))
    end

    local hit_z = M.touch("Z", -1, cfg.max_travel)
    if not hit_z then return false end

    -- Set Work Z: M.touch already retracted by cfg.retract above contact point
    local current_z = z_target + cfg.retract
    cnc.set_work_pos("Z", current_z)
    cnc.gcode(string.format("G90 G92 Z%.4f", current_z))
    cnc.wait_idle()

    -- Retract to clearance above plate or workpiece
    local safe_retract_z = z_target + cfg.retract + 5.0
    cnc.gcode(string.format("G90 G0 Z%.3f", safe_retract_z))
    cnc.wait_idle()

    if is_direct_workpiece then
        cnc.log("Z touch-off complete! Work Z zeroed to workpiece surface (0.000 mm).")
    else
        cnc.log(string.format("Z touch-off complete! Work Z set to %.3f mm (Plate thickness).", cfg.plate_h))
    end
    return true
end

-- ----------------------------------------------------------------------------
-- OUTSIDE EDGE & CORNER ROUTINES
-- ----------------------------------------------------------------------------
-- Probes single outside edge along axis ("X" or "Y").
-- travel_dir: +1 (probing toward +) or -1 (probing toward -)
-- Operator positions probe outside the edge, at or slightly above probe depth.
function M.probe_outside_edge(axis, travel_dir, set_zero)
    axis = string.upper(axis)
    local cfg = M.get_config()
    cnc.log(string.format("Probing Outside Edge %s (%s)...", axis, (travel_dir > 0) and "+" or "-"))

    local hit_pos = M.touch(axis, travel_dir)
    if not hit_pos then return false end

    -- Edge position accounts for tip radius (contact occurs at tip boundary)
    local edge_coord = (travel_dir > 0) and (hit_pos + cfg.tip_radius) or (hit_pos - cfg.tip_radius)

    if set_zero ~= false then
        -- M.touch retracted by cfg.retract away from hit_pos; edge is at edge_coord
        local current_offset = -travel_dir * (cfg.retract + cfg.tip_radius)
        cnc.set_work_pos(axis, current_offset)
        cnc.gcode(string.format("G90 G92 %s%.4f", axis, current_offset))
        cnc.wait_idle()
        cnc.log(string.format("Outside %s Edge set to 0.000 mm (Tip compensated: %.3f mm)", axis, cfg.tip_radius))
    else
        cnc.log(string.format("Outside %s Edge found at %.3f mm", axis, edge_coord))
    end

    return true, edge_coord
end

-- Probes outside corner (e.g. Top-Left, Bottom-Right)
-- dir_x: +1 (probe right) or -1 (probe left)
-- dir_y: +1 (probe up/North) or -1 (probe down/South)
function M.probe_outside_corner(dir_x, dir_y)
    local cfg = M.get_config()
    cnc.log("Probing Outside Corner...")

    -- Probe X edge first
    local ok_x, x_val = M.probe_outside_edge("X", dir_x, true)
    if not ok_x then return false end

    -- Retract X slightly to avoid dragging tip along corner
    cnc.gcode(string.format("G91 G0 X%.3f", (dir_x > 0) and -cfg.retract or cfg.retract))
    cnc.wait_idle()

    -- Probe Y edge second
    local ok_y, y_val = M.probe_outside_edge("Y", dir_y, true)
    if not ok_y then return false end

    cnc.log("Outside Corner zeroed successfully! (X=0, Y=0)")
    return true
end

-- ----------------------------------------------------------------------------
-- INSIDE EDGE & CORNER ROUTINES (Pockets / Cavities)
-- ----------------------------------------------------------------------------
-- Operator positions probe inside the pocket.
-- travel_dir: +1 (moves toward + wall) or -1 (moves toward - wall)
function M.probe_inside_edge(axis, travel_dir, set_zero)
    axis = string.upper(axis)
    local cfg = M.get_config()
    cnc.log(string.format("Probing Inside Wall %s (%s)...", axis, (travel_dir > 0) and "+" or "-"))

    local hit_pos = M.touch(axis, travel_dir)
    if not hit_pos then return false end

    local wall_coord = (travel_dir > 0) and (hit_pos + cfg.tip_radius) or (hit_pos - cfg.tip_radius)

    if set_zero ~= false then
        local current_offset = -travel_dir * (cfg.retract + cfg.tip_radius)
        cnc.set_work_pos(axis, current_offset)
        cnc.gcode(string.format("G90 G92 %s%.4f", axis, current_offset))
        cnc.wait_idle()
        cnc.log(string.format("Inside %s Wall set to 0.000 mm", axis))
    end

    return true, wall_coord
end

function M.probe_inside_corner(dir_x, dir_y)
    local cfg = M.get_config()
    cnc.log("Probing Inside Pocket Corner...")

    local ok_x, x_val = M.probe_inside_edge("X", dir_x, true)
    if not ok_x then return false end

    cnc.gcode(string.format("G91 G0 X%.3f", (dir_x > 0) and -cfg.retract or cfg.retract))
    cnc.wait_idle()

    local ok_y, y_val = M.probe_inside_edge("Y", dir_y, true)
    if not ok_y then return false end

    cnc.log("Inside Corner zeroed successfully! (X=0, Y=0)")
    return true
end

-- ----------------------------------------------------------------------------
-- CENTER FINDING ROUTINES
-- ----------------------------------------------------------------------------

-- 1. Inside Bore / Pocket Circular Center (4-point probe: -X, +X, -Y, +Y)
function M.probe_inside_bore_center()
    local cfg = M.get_config()
    local approx_r = cfg.diameter * 0.5
    local search_dist = approx_r + 10.0
    cnc.log(string.format("Probing Inside Bore Center (Approx Diam: %.1f mm)...", cfg.diameter))

    -- Probe X-
    local x_minus = M.touch("X", -1, search_dist)
    if not x_minus then return false end

    -- Return to starting center X
    cnc.gcode(string.format("G90 G0 X%.3f", (x_minus + cfg.retract + (approx_r * 0.5))))
    cnc.wait_idle()

    -- Probe X+
    local x_plus = M.touch("X", 1, search_dist)
    if not x_plus then return false end

    -- Center X is midpoint
    local center_x = (x_minus + x_plus) * 0.5
    cnc.gcode(string.format("G90 G0 X%.3f", center_x))
    cnc.wait_idle()

    -- Probe Y-
    local y_minus = M.touch("Y", -1, search_dist)
    if not y_minus then return false end

    cnc.gcode(string.format("G90 G0 Y%.3f", (y_minus + cfg.retract + (approx_r * 0.5))))
    cnc.wait_idle()

    -- Probe Y+
    local y_plus = M.touch("Y", 1, search_dist)
    if not y_plus then return false end

    local center_y = (y_minus + y_plus) * 0.5
    cnc.gcode(string.format("G90 G0 Y%.3f", center_y))
    cnc.wait_idle()

    -- Set Work Coordinates to X=0, Y=0 at Bore Center
    cnc.set_work_pos("X", 0.0)
    cnc.set_work_pos("Y", 0.0)
    cnc.gcode("G90 G92 X0 Y0")
    cnc.wait_idle()

    local measured_diam_x = (x_plus - x_minus) + cfg.tip_dia
    local measured_diam_y = (y_plus - y_minus) + cfg.tip_dia
    cnc.log(string.format("Bore Center found! Diameters: X=%.3f mm, Y=%.3f mm. Zeroed X=0, Y=0.", measured_diam_x, measured_diam_y))
    return true
end

-- 2. Inside Slot Center (Single axis: "X" or "Y")
function M.probe_inside_slot_center(axis)
    axis = string.upper(axis)
    local cfg = M.get_config()
    local approx_w = (axis == "X") and cfg.size_x or cfg.size_y
    local search_dist = (approx_w * 0.5) + 10.0
    cnc.log(string.format("Probing Inside %s Slot Center...", axis))

    local p_minus = M.touch(axis, -1, search_dist)
    if not p_minus then return false end

    cnc.gcode(string.format("G90 G0 %s%.3f", axis, p_minus + cfg.retract + (approx_w * 0.4)))
    cnc.wait_idle()

    local p_plus = M.touch(axis, 1, search_dist)
    if not p_plus then return false end

    local center = (p_minus + p_plus) * 0.5
    cnc.gcode(string.format("G90 G0 %s%.3f", axis, center))
    cnc.wait_idle()

    cnc.set_work_pos(axis, 0.0)
    cnc.gcode(string.format("G90 G92 %s0", axis))
    cnc.wait_idle()

    local slot_width = (p_plus - p_minus) + cfg.tip_dia
    cnc.log(string.format("Inside %s Slot Center zeroed! Measured Width: %.3f mm.", axis, slot_width))
    return true
end

-- 3. Outside Web Center (Automated 2-point touch across boss / web)
-- Operator positions probe above the approximate center of the web at safe clearance height.
function M.probe_outside_web_center(axis)
    axis = string.upper(axis)
    local cfg = M.get_config()
    local approx_w = (axis == "X") and cfg.size_x or cfg.size_y
    local clearance_dist = (approx_w * 0.5) + cfg.retract + cfg.tip_dia + 4.0
    local plunge_depth = (cfg.probe_depth and cfg.probe_depth > 0) and cfg.probe_depth or 5.0

    cnc.log(string.format("Starting Automated Outside %s Web Probing (Approx width: %.2f mm)...", axis, approx_w))

    -- Step 1: Record safe Z clearance above the boss/web
    local cur_z = cnc.get_work_pos("Z")

    -- Step 2: Traverse to negative side
    cnc.gcode(string.format("G91 G0 %s-%.3f", axis, clearance_dist))
    cnc.wait_idle()

    -- Step 3: Plunge Z down into probe depth
    cnc.gcode(string.format("G91 G0 Z-%.3f", plunge_depth))
    cnc.wait_idle()
    cnc.gcode("G90")

    -- Step 4: Touch side 1 (+ direction toward center)
    local p_minus = M.touch(axis, 1, clearance_dist + 5.0)
    if not p_minus then
        cnc.gcode(string.format("G90 G0 Z%.3f", cur_z))
        cnc.wait_idle()
        return false
    end

    -- Step 5: Retract Z back to safe clearance
    cnc.gcode(string.format("G90 G0 Z%.3f", cur_z))
    cnc.wait_idle()

    -- Step 6: Traverse over web to positive side
    cnc.gcode(string.format("G91 G0 %s%.3f", axis, clearance_dist * 2.0))
    cnc.wait_idle()

    -- Step 7: Plunge Z down into probe depth
    cnc.gcode(string.format("G91 G0 Z-%.3f", plunge_depth))
    cnc.wait_idle()
    cnc.gcode("G90")

    -- Step 8: Touch side 2 (- direction toward center)
    local p_plus = M.touch(axis, -1, clearance_dist + 5.0)
    if not p_plus then
        cnc.gcode(string.format("G90 G0 Z%.3f", cur_z))
        cnc.wait_idle()
        return false
    end

    -- Step 9: Retract Z back to safe clearance
    cnc.gcode(string.format("G90 G0 Z%.3f", cur_z))
    cnc.wait_idle()

    -- Step 10: Calculate center and move there at safe Z
    local center = (p_minus + p_plus) * 0.5
    cnc.gcode(string.format("G90 G0 %s%.3f", axis, center))
    cnc.wait_idle()

    cnc.set_work_pos(axis, 0.0)
    cnc.gcode(string.format("G90 G92 %s0", axis))
    cnc.wait_idle()

    local web_width = (p_plus - p_minus) - cfg.tip_dia
    cnc.log(string.format("Outside %s Web Center zeroed! Measured Width: %.3f mm.", axis, web_width))
    return true
end

-- 4. Outside Circular Boss Center (4-point cross probing: X then Y)
-- Operator positions probe above the approximate boss center at safe clearance height.
function M.probe_outside_boss_center()
    cnc.log("Starting Automated Outside Circular Boss Center Probing...")
    local ok_x = M.probe_outside_web_center("X")
    if not ok_x then return false end

    local ok_y = M.probe_outside_web_center("Y")
    if not ok_y then return false end

    cnc.log("Outside Circular Boss Center successfully zeroed! (X=0, Y=0)")
    return true
end

return M
