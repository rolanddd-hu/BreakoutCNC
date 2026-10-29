-- ============================================================================
-- RnRCncController - Z Tool Setter Touch-Off Routine (probe_z.lua)
-- ============================================================================

local PLATE_THICKNESS_MM = 15.0
local MAX_SEARCH_MM      = 50.0
local FAST_FEED_MM_MIN   = 300.0
local SLOW_FEED_MM_MIN   = 60.0
local RETRACT_MM         = 3.0

cnc.log("Starting Z Tool Setter Probing Routine...")

if cnc.is_probe_tripped() then
    cnc.estop("Cannot probe: Probe input switch is already tripped!")
    return false
end

-- Fast approach probe
cnc.log("Fast approach probe down...")
local res = cnc.probe("Z", -MAX_SEARCH_MM, FAST_FEED_MM_MIN)
if not res.hit then
    cnc.log("Probe failed: target distance exceeded without contact.", true)
    return false
end

-- Short back-off retract
cnc.gcode(string.format("G91 G0 Z%.2f", RETRACT_MM))
cnc.wait_idle()
cnc.sleep_ms(100)

-- Fine latch touch-off
cnc.log("Fine slow touch-off...")
res = cnc.probe("Z", -(RETRACT_MM * 1.5), SLOW_FEED_MM_MIN)
if not res.hit then
    cnc.log("Probe fine latch failed.", true)
    return false
end

-- Set Work Z coordinate to plate thickness (G92 Z<plate_height>)
cnc.gcode(string.format("G90 G92 Z%.3f", PLATE_THICKNESS_MM))
cnc.wait_idle()

-- Retract tool to clearance
cnc.gcode(string.format("G90 G0 Z%.2f", PLATE_THICKNESS_MM + 10.0))
cnc.wait_idle()

cnc.log(string.format("Z Zero set successfully! (Plate thickness: %.2f mm)", PLATE_THICKNESS_MM))
return true
