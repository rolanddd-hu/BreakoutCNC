#pragma once
#include "core/controller/motion_controller.h"
#include <functional>
#include <array>

namespace rnr::ui {

class WorkspacePanel {
public:
    WorkspacePanel();
    ~WorkspacePanel() = default;

    void render(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        bool show_4th_axis,
        const std::function<void()>& on_changed = nullptr);

private:
    int                   edit_modal_ws_   = -1; // -1 = closed, 0..5 = open for G54..G59
    std::array<double, 4> edit_temp_offset_{0.0, 0.0, 0.0, 0.0};
};

} // namespace rnr::ui
