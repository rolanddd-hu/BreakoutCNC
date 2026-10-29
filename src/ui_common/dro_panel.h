#pragma once
#include "core/controller/motion_controller.h"
#include "imgui.h"
#include <cstddef>
#include <functional>

namespace rnr::ui {

class GCodeProgramPanel;

class DroAndJogPanel {
public:
    DroAndJogPanel() = default;

    // Render X/Y/Z (and optional A) DRO readouts + manual jogging pad + optional 2x2 Cycle Control Grid
    void render(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        bool show_fourth_axis_a = false,
        float allocated_h = -1.0f,
        GCodeProgramPanel* gcode_panel = nullptr);

    void setStepJogMode(bool v) noexcept { step_jog_mode_ = v; }
    bool isStepJogMode() const noexcept { return step_jog_mode_; }

    void setStepSizeMm(float v) noexcept { step_size_mm_ = v; }
    float getStepSizeMm() const noexcept { return step_size_mm_; }

    void setJogSpeedPct(float v) noexcept { jog_speed_pct_ = v; }
    float getJogSpeedPct() const noexcept { return jog_speed_pct_; }

    float getRenderedHeight() const noexcept { return rendered_height_; }
    float getMaxHeight() const noexcept { return max_height_; }
    float getMinHeight() const noexcept { return min_height_; }
    int   getKeyboardAxisDir(size_t axis) const noexcept { return (axis < 4) ? kb_axis_active_[axis] : 0; }

    void setOnConfigChanged(std::function<void()> cb) { on_config_changed_ = std::move(cb); }

private:
    // Keyboard jogging: Arrow Left/Right = X-/X+, Arrow Up/Down = Y+/Y-, PgUp/PgDn = Z+/Z-.
    // Called every frame from render(). Supports simultaneous multi-axis motion.
    void handleKeyboardJog(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry);

    bool  step_jog_mode_    = false;
    float step_size_mm_     = 10.0f;
    float jog_speed_pct_    = 50.0f;
    float rendered_height_  = 540.0f;
    float max_height_       = 620.0f;
    float min_height_       = 350.0f;
    float allocated_height_ = -1.0f;
    std::array<int, 4> kb_axis_active_{0, 0, 0, 0}; // -1 = negative jog, 0 = idle, +1 = positive jog per axis

    // Virtual Analog Joystick state (for X/Y and Z continuous drag)
    bool   js_xy_active_ = false;
    ImVec2 js_xy_drag_{0.0f, 0.0f};
    bool   js_z_active_  = false;
    float  js_z_drag_    = 0.0f;

    // Interactive Probe Z Modal state
    bool   show_probe_z_modal_ = false;

    std::function<void()> on_config_changed_;
};

} // namespace rnr::ui
