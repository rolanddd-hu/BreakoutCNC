-- Probing Macro: probe_z.lua
local p = dofile(cnc.resolve_path("scripts/mill/probing/probe_lib.lua")); p.probe_z(_G.__probe_z_on_workpiece == true)
