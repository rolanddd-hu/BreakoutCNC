-- ============================================================================
-- RnRCncController - Custom Macro M100: Engage Pneumatic Clamp / Vacuum
-- Actuates auxiliary output OUT2
-- ============================================================================

cnc.log("Custom Macro M100: Engaging Workpiece Clamp (OUT2 ON)...")
cnc.set_output(2, true)
cnc.sleep_ms(250) -- Dwell for pneumatic pressure to build
cnc.log("Workpiece Clamp Engaged.")
