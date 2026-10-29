-- ============================================================================
-- RnRCncController - Plasma Initial Height Sense (IHS) Touch-Off Macro
-- Runs automatically on M3 before torch fire to establish material surface
-- ============================================================================

local SWITCH_OFFSET_MM = 3.2   -- Floating head microswitch travel before trigger
local PIERCE_HEIGHT_MM = 3.8   -- Retract height before torch fires
local MAX_DIVE_MM      = 60.0  -- Max Z travel down to find sheet surface
local TOUCH_FEED_MM    = 800.0 -- Touch speed mm/min

cnc.log("Plasma IHS Touch-Off Started...")

-- 1. Probe downward until Ohmic or Floating Head switch trips
local res = cnc.probe("Z", -MAX_DIVE_MM, TOUCH_FEED_MM)
if not res.hit then
    cnc.estop("Plasma IHS Failed: Sheet metal surface not detected!")
    return false
end

-- 2. Zero Work Z at sheet surface (accounting for floating head switch travel)
cnc.gcode(string.format("G92 Z-%.3f", SWITCH_OFFSET_MM))
cnc.wait_idle()

-- 3. Rapid to Pierce Height
cnc.gcode(string.format("G0 Z%.2f", PIERCE_HEIGHT_MM))
cnc.wait_idle()

-- 4. Ignite Plasma Torch!
cnc.log(string.format("At Pierce Height (%.2f mm). Firing Torch (M3)...", PIERCE_HEIGHT_MM))
cnc.set_torch(true)

return true
