#pragma once
#include "core/controller/motion_controller.h"

namespace rnr::ui {

// "Inputs & Outputs" settings section (both applications): what each output OUT1..OUT4 does (machine function or
// named custom output, with a test switch), and what each input IN1..IN12 is used for (custom name and invert for
// unused inputs). Edits cfg; returns true when it changed and must be applied and saved by the caller.
bool renderIoSettings(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry,
                      core::MachineConfig& cfg);

} // namespace rnr::ui
