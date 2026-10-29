#pragma once
#include "core/controller/motion_controller.h"
#include <functional>

namespace rnr::ui {

// Tool length mode selector + tool table editor (G43 H<n> tool length offsets)
class ToolTablePanel {
public:
    ToolTablePanel() = default;

    void render(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        const std::function<void()>& on_changed = nullptr);

private:
    int new_tool_number_ = 1;
    int spindle_tool_    = 0; // Combo selection for "Tool in spindle"
};

} // namespace rnr::ui
