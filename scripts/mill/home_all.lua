-- ============================================================================
-- RnRCncController - Mill Safe Homing Sequence (home_all.lua)
-- Safe CNC Homing Order: Z Up first, then Y, then X
-- ============================================================================

cnc.log("Starting Safe CNC Homing Sequence...")

-- 1. Ensure Spindle is strictly OFF
cnc.set_spindle(false)

-- 2. Home Z Axis first (Up to switch)
cnc.log("Homing Z Axis (Safe Retract Up)...")
cnc.home("Z")
cnc.wait_idle()

-- 3. Home Y Axis
cnc.log("Homing Y Axis...")
cnc.home("Y")
cnc.wait_idle()

-- 4. Home X Axis
cnc.log("Homing X Axis...")
cnc.home("X")
cnc.wait_idle()

cnc.log("All axes homed successfully.")
return true
