#pragma once
#include "core/controller/motion_controller.h"
#include "ui_common/theme.h"
#include <imgui.h>
#include <algorithm>

namespace rnr::ui {

// Automatic Z lift at program start, shared by the machine settings of both applications.
// Returns true when a setting changed.
inline bool renderProgramStartSettings(core::MachineConfig& cfg) {
    bool changed = false;
    if (ImGui::Checkbox("Lift Z before the first rapid##StartLift", &cfg.start_lift_to_safe_z)) changed = true;

    const bool imperial = (cfg.linear_units == core::LinearUnits::Imperial);
    const double scale  = imperial ? (1.0 / 25.4) : 1.0;
    float safe_z = static_cast<float>(cfg.start_safe_z_mm * scale);
    const char* label = imperial ? "Safe Z (work, in)##StartSafeZ" : "Safe Z (work, mm)##StartSafeZ";
    const float label_w = ImGui::CalcTextSize(label, nullptr, true).x;
    ImGui::SetNextItemWidth(std::max(80.0f, ImGui::GetContentRegionAvail().x - label_w - ImGui::GetStyle().ItemInnerSpacing.x));
    if (ImGui::InputFloat(label, &safe_z, imperial ? 0.1f : 1.0f, imperial ? 0.5f : 5.0f, imperial ? "%.3f" : "%.1f")) {
        cfg.start_safe_z_mm = std::max(0.0, static_cast<double>(safe_z) / scale);
        changed = true;
    }

    ImGui::PushTextWrapPos(0.0f);
    if (cfg.start_lift_to_safe_z) {
        ImGui::TextDisabled("When a program starts from the first line with a G0 that moves in X or Y, Z goes up first "
                            "and X / Y follow. A G0 without Z lifts to this height, a G0 to a higher Z lifts to that "
                            "Z. Run From Line also lifts to this height.");
    } else {
        ImGui::TextColored(theme::pal().text_warn,
                           "The first rapid runs exactly as written: all axes move together from where the tool is. "
                           "Make sure the tool is clear of the work and clamps before starting. Run From Line still "
                           "lifts to this height.");
    }
    ImGui::PopTextWrapPos();
    return changed;
}

} // namespace rnr::ui
