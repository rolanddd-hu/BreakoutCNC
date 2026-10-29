-- ============================================================================
-- RnRCncController - Custom Macro M101: Release Pneumatic Clamp / Vacuum
-- De-energizes auxiliary output OUT2
-- ============================================================================

cnc.log("Custom Macro M101: Releasing Workpiece Clamp (OUT2 OFF)...")
cnc.set_output(2, false)
cnc.sleep_ms(250)
cnc.log("Workpiece Clamp Released.")
