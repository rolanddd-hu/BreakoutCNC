-- ============================================================================
-- RnRCncController - Plasma Laser Crosshair Offset Macro (laser_offset.lua)
-- Applies physical XY offset from laser crosshair pointer to plasma torch center
-- ============================================================================

local LASER_OFFSET_X_MM = -45.0 -- Distance from laser point to torch nozzle center
local LASER_OFFSET_Y_MM = 20.0

cnc.log("Shifting from Laser Crosshair to Torch Center...")

-- Shift work coordinates by offset
cnc.gcode(string.format("G91 G0 X%.2f Y%.2f", LASER_OFFSET_X_MM, LASER_OFFSET_Y_MM))
cnc.wait_idle()

-- Switch back to absolute mode and zero XY
cnc.gcode("G90")
cnc.zero_axis("X")
cnc.zero_axis("Y")

cnc.log("Work Origin (0, 0) aligned to Torch Center successfully!")
return true
