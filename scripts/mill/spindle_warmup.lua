-- ============================================================================
-- RnRCncController - Spindle Warmup Macro (spindle_warmup.lua)
-- Gradually brings bearings up to operating temperature
-- ============================================================================

local stages = {
    { rpm = 6000,  seconds = 60 },
    { rpm = 12000, seconds = 60 },
    { rpm = 18000, seconds = 60 },
    { rpm = 24000, seconds = 30 }
}

cnc.log("=== Spindle Bearing Warmup Routine Started ===")

for i, stage in ipairs(stages) do
    cnc.log(string.format("Warmup Stage %d/%d: Running at %d RPM for %d seconds...", 
        i, #stages, stage.rpm, stage.seconds))
    
    cnc.set_spindle_rpm(stage.rpm)
    cnc.set_spindle(true)
    
    local remaining = stage.seconds
    while remaining > 0 do
        cnc.sleep_ms(1000)
        remaining = remaining - 1
    end
end

cnc.log("Warmup complete. Shutting down spindle...")
cnc.set_spindle(false)
cnc.log("=== Spindle Ready for Cutting ===")
return true
