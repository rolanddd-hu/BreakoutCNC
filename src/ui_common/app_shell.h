#pragma once
// Pieces both applications (BreakoutMill, BreakoutCut) share around their main loop
#include "core/controller/motion_controller.h"
#include <string>

namespace rnr::ui {

// ESC is the keyboard E-stop. While the machine moves (or a job is paused) it always stops it. At rest, ESC in an open
// popup or a text field only closes / leaves it, as everywhere else in the UI.
bool escapeTriggersEStop(const core::TelemetrySnapshot& telemetry);

// A settings file that cannot be written (read-only folder): a small warning at the bottom of the window until it is
// dismissed or a later save succeeds. `message` is cleared by the Dismiss button.
void renderSettingsSaveWarning(std::string& message);

// Nothing on screen changes by itself: the machine is at rest, no script runs and no camera is shown (the window may
// then draw fewer frames, see AppWindow::setIdlePredicate)
bool screenIsStatic(const core::TelemetrySnapshot& telemetry, const core::MachineConfig& cfg);

} // namespace rnr::ui
