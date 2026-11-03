#pragma once
#include "core/controller/motion_controller.h"
#include "ui_common/theme.h"
#include <imgui.h>
#include <algorithm>

namespace rnr::ui {

// Constant velocity motion, shared by the machine settings of both applications.
// Returns true when a setting changed.
inline bool renderCornerSettings(core::MachineConfig& cfg) {
    bool changed = false;

    const bool imperial = (cfg.linear_units == core::LinearUnits::Imperial);
    const double scale  = imperial ? (1.0 / 25.4) : 1.0;
    float tolerance = static_cast<float>(cfg.corner_tolerance_mm * scale);
    const char* label = imperial ? "Corner tolerance (in)##CornerTol" : "Corner tolerance (mm)##CornerTol";
    const float label_w = ImGui::CalcTextSize(label, nullptr, true).x;
    ImGui::SetNextItemWidth(std::max(80.0f, ImGui::GetContentRegionAvail().x - label_w - ImGui::GetStyle().ItemInnerSpacing.x));
    if (ImGui::InputFloat(label, &tolerance, imperial ? 0.0005f : 0.01f, imperial ? 0.002f : 0.05f, imperial ? "%.4f" : "%.3f")) {
        cfg.corner_tolerance_mm = std::clamp(static_cast<double>(tolerance) / scale,
                                             core::MachineConfig::kMinCornerToleranceMm,
                                             core::MachineConfig::kMaxCornerToleranceMm);
        changed = true;
    }

    ImGui::Spacing();
    if (ImGui::Checkbox("Limit speed on curves##CurveLimit", &cfg.limit_curve_speed)) changed = true;

    if (cfg.limit_curve_speed) {
        ImGui::Spacing();
        const char* accel_label = "Curve acceleration##CurveAccel";
        const float accel_label_w = ImGui::CalcTextSize(accel_label, nullptr, true).x;
        ImGui::SetNextItemWidth(std::max(80.0f, ImGui::GetContentRegionAvail().x - accel_label_w - ImGui::GetStyle().ItemInnerSpacing.x));
        if (ImGui::SliderInt(accel_label, &cfg.curve_accel_pct, core::MachineConfig::kMinCurveAccelPct,
                             core::MachineConfig::kMaxCurveAccelPct, "%d%%", ImGuiSliderFlags_AlwaysClamp)) {
            changed = true;
        }
        if (cfg.curve_accel_pct > 100) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(theme::pal().text_warn, "Above the axis acceleration: the motors can lose steps on small curves.");
            ImGui::PopTextWrapPos();
        }
    }
    return changed;
}

} // namespace rnr::ui
