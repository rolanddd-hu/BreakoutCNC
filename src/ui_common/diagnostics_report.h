#pragma once
#include "core/controller/motion_controller.h"
#include <string>

namespace rnr::ui {

// Plain-text report for bug reports: app version, OS, graphics, board / simulator state, a few key settings and
// the last fault. Leaves out file paths and the board serial number (they can contain personal data).
std::string buildDiagnosticsReport(const core::MachineConfig& cfg,
                                   const core::TelemetrySnapshot& telemetry,
                                   size_t program_lines,
                                   size_t program_segments);

} // namespace rnr::ui
