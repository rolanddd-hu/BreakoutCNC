-- ============================================================================
-- RnRCncController - Park Machine Macro (park.lua)
-- Retracts Z to Machine Safe Z and moves XY to back-left park coordinate
-- ============================================================================

cnc.log("Parking Machine...")

-- 1. Stop spindle
cnc.set_spindle(false)

-- 2. Lift Z to machine reference Safe Z (G53 Z0)
cnc.gcode("G0 G53 Z0")
cnc.wait_idle()

-- 3. Move XY to back-left park coordinate (adjust to your machine table size)
cnc.gcode("G0 G53 X0 Y200")
cnc.wait_idle()

cnc.log("Machine parked at Safe Clearance.")
return true
