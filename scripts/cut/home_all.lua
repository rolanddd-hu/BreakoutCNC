-- ============================================================================
-- BreakoutCNC - Plasma Safe Homing & Auto-Squaring (home_all.lua)
-- Safe CNC Homing Order: Z Torch Up first, then Y Gantry, then X Carriage
-- ============================================================================

cnc.log("Starting Plasma Safe Homing Sequence...")

-- 1. Ensure Plasma Torch is extinguished
cnc.set_torch(false)

-- 2. Home Z Axis (Torch Retract Up)
cnc.log("Retracting Torch to Z Home...")
cnc.home("Z")
cnc.wait_idle()
if not cnc.is_homed("Z") then error("Z axis was not homed") end

-- 3. Home Y Gantry (Dual-motor auto squaring if equipped)
cnc.log("Homing Y Axis (Gantry)...")
cnc.home("Y")
cnc.wait_idle()
if not cnc.is_homed("Y") then error("Y axis was not homed") end

-- 4. Home X Carriage
cnc.log("Homing X Axis (Carriage)...")
cnc.home("X")
cnc.wait_idle()
if not cnc.is_homed("X") then error("X axis was not homed") end

cnc.log("Plasma machine homed and referenced.")
return true
