#include "ui_common/app_shell.h"
#include "ui_common/theme.h"
#include <imgui.h>
#include <imgui_internal.h> // ClosePopupsExceptModals
#include <algorithm>

namespace rnr::ui {

bool escapeTriggersEStop(const core::TelemetrySnapshot& telemetry) {
    if (!ImGui::IsKeyPressed(ImGuiKey_Escape, false)) return false;
    const auto s = telemetry.state;
    const bool at_rest = s == core::ControllerState::Idle || s == core::ControllerState::EStop ||
                         s == core::ControllerState::Disconnected || s == core::ControllerState::ToolChangeHold;
    if (!at_rest) return true;
    if (ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel)) {
        // A dropdown or menu: ESC closes it (ImGui only does that with keyboard navigation on). A modal dialog stays.
        ImGui::ClosePopupsExceptModals();
        return false;
    }
    return !ImGui::GetIO().WantTextInput; // A text field: ESC leaves it
}

bool screenIsStatic(const core::TelemetrySnapshot& telemetry, const core::MachineConfig& cfg) {
    const auto s = telemetry.state;
    const bool at_rest = s == core::ControllerState::Idle || s == core::ControllerState::EStop ||
                         s == core::ControllerState::Disconnected || s == core::ControllerState::ToolChangeHold;
    const bool camera = cfg.camera.top_down_camera_index >= 0 || cfg.camera.monitoring_camera_index >= 0;
    return at_rest && !telemetry.script_running && !telemetry.has_script_prompt && !telemetry.board_connecting &&
           !telemetry.surface_probing_active && !camera;
}

void renderSettingsSaveWarning(std::string& message) {
    if (message.empty()) return;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    // Bottom centre, above the I/O strip: never over the E-stop / cycle start buttons in the left column
    const float w = std::min(540.0f, vp->WorkSize.x - 32.0f);
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y - 56.0f),
                            ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowSize(ImVec2(w, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::pal().surface_warn);
    if (ImGui::Begin("##SettingsSaveWarning", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing)) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(theme::pal().text_warn, "%s", message.c_str());
        ImGui::PopTextWrapPos();
        if (ImGui::Button("Dismiss##SaveWarn", ImVec2(ImGui::CalcTextSize("Dismiss").x + 32.0f, 28.0f))) message.clear();
    }
    ImGui::End();
    ImGui::PopStyleColor();
}

} // namespace rnr::ui
