#pragma once
#include "core/controller/motion_controller.h"
#include <imgui.h>

namespace rnr::ui {

// Dual-motor gantry selector shared by the machine settings of both applications.
// Returns true when the selection changed.
inline bool renderGantrySlaveCombo(core::MachineConfig& cfg) {
    static const char* kItems[] = {
        "None (A is an independent axis)",
        "A motor follows X (dual-motor X gantry)",
        "A motor follows Y (dual-motor Y gantry)",
    };
    ImGui::TextUnformatted("Dual-Motor Gantry (A Motor Slaving)");
    int idx = static_cast<int>(cfg.gantry_slave);
    bool changed = false;
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::Combo("##GantrySlave", &idx, kItems, IM_ARRAYSIZE(kItems))) {
        cfg.gantry_slave = static_cast<core::GantrySlave>(idx);
        changed = true;
    }
    if (cfg.gantry_slave != core::GantrySlave::None) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("The A motor copies every step of its master axis, so give it the same Steps/mm and "
                            "use its Invert Dir setting to match the motor's mounting. Homing does not square the "
                            "gantry: both motors move together.");
        ImGui::PopTextWrapPos();
    }
    return changed;
}

} // namespace rnr::ui
