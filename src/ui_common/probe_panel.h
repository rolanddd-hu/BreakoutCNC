#pragma once
#include "imgui.h"
#include <string>

namespace rnr::core {
class IMotionController;
struct TelemetrySnapshot;
struct MachineConfig;
}

namespace rnr::ui {

class ToolpathViewport;

class ProbePanel {
public:
    ProbePanel() = default;
    ~ProbePanel() = default;

    // Renders the full graphical Touch Probe & Tool Setter deck (Surface, Outside, Inside, Center, Probe Z)
    void render(core::IMotionController& controller,
                const core::TelemetrySnapshot& telemetry,
                core::MachineConfig& cfg,
                ToolpathViewport* viewport = nullptr);

    void setViewport(ToolpathViewport* vp) noexcept { viewport_ = vp; }

private:
    void renderSurfaceDeck(core::IMotionController& controller,
                           const core::TelemetrySnapshot& telemetry,
                           core::MachineConfig& cfg,
                           float avail_w,
                           ToolpathViewport* viewport);
    void renderOutsideDeck(core::IMotionController& controller, float avail_w);
    void renderInsideDeck(core::IMotionController& controller, float avail_w);
    void renderCenterDeck(core::IMotionController& controller, float avail_w);
    void renderProbeZDeck(core::IMotionController& controller, float avail_w);

    bool renderHotspot(const char* id,
                       const ImVec2& center,
                       float radius,
                       const char* label,
                       const char* tooltip,
                       const std::string& script_rel_path,
                       core::IMotionController& controller);

    int selected_deck_ = 0; // 0: SURFACE, 1: OUTSIDE, 2: INSIDE, 3: CENTER, 4: PROBE Z
    ToolpathViewport* viewport_ = nullptr;

    // Surface Level Grid Probing parameters
    double surface_min_x_ = 0.0;
    double surface_max_x_ = 100.0;
    double surface_min_y_ = 0.0;
    double surface_max_y_ = 100.0;
    int    surface_points_x_ = 5;
    int    surface_points_y_ = 5;
    int    surface_max_points_ = 30;
    bool   surface_bounds_initialized_ = false;

    // Surface Smoothness / Interpolation (0.0 = Bilinear, 1.0 = Bicubic Spline)
    float  surface_smoothness_ = 0.5f;
    float  applied_smoothness_ = 0.5f;
    bool   smoothness_initialized_ = false;
};

} // namespace rnr::ui
