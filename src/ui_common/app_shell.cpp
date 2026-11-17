#include "ui_common/app_shell.h"
#include "ui_common/theme.h"
#include "ui_common/app_window.h"
#include "ui_common/toolpath_viewport.h"
#include <cstdio>
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

void connectWindowSettings(AppWindow& window, ToolpathViewport& viewport, config::AppUiSettings& ui_settings,
                           const std::function<void()>& mark_dirty, bool* minimal_layout) {
    // FULL / MINIMAL layout switcher in the custom title bar
    window.setMinimalLayout(minimal_layout, [&ui_settings, mark_dirty](bool is_min) {
        ui_settings.minimal_layout = is_min;
        mark_dirty();
    });
    // Light / dark theme buttons: restore the theme chosen in each family, then the active one
    theme::setTheme(theme::findTheme(theme::Mode::Dark, ui_settings.dark_theme_id.c_str()));
    theme::setTheme(theme::findTheme(theme::Mode::Light, ui_settings.light_theme_id.c_str()));
    theme::setMode(ui_settings.light_theme ? theme::Mode::Light : theme::Mode::Dark);
    window.setOnThemeChanged([&ui_settings, mark_dirty](bool light) {
        ui_settings.light_theme    = light;
        ui_settings.dark_theme_id  = theme::themeInfo(theme::familyTheme(theme::Mode::Dark)).id;
        ui_settings.light_theme_id = theme::themeInfo(theme::familyTheme(theme::Mode::Light)).id;
        mark_dirty();
    });
    // Viewport options are saved when toggled
    viewport.setOnViewChanged([&ui_settings, &viewport, mark_dirty]() {
        ui_settings.isometric_3d         = viewport.isIsometric3D();
        ui_settings.show_rapids          = viewport.getShowRapids();
        ui_settings.toolpath_simplify_px = viewport.getSimplifyPx();
        ui_settings.show_pierces         = viewport.getShowPierces();
        ui_settings.follow_torch         = viewport.getFollowTorch();
        mark_dirty();
    });
}

void installCloseHandling(AppWindow& window, core::IMotionController& controller, bool& force_close, bool& show_confirm,
                          std::function<void()> save_settings) {
    window.setCloseInterceptor(
        [&controller, &force_close]() {
            if (force_close) return true;
            const auto s = controller.getTelemetry().state;
            return !(s == core::ControllerState::RunningGCode || s == core::ControllerState::DwellPierce ||
                     s == core::ControllerState::Jogging);
        },
        [&show_confirm]() { show_confirm = true; });
    window.setOnCloseCallback([&controller, save_settings]() {
        controller.triggerEStop();
        save_settings();
    });
}

void SmokeTest::frame(AppWindow& window, core::IMotionController& controller) {
    if (!enabled) return;
    ++frames;
    if (frames == 10 && controller.resetFromEStop()) controller.startJob();
    if (frames == 240) { // ~4 s at 60 FPS
        const auto t = controller.getTelemetry();
        failed = (t.active_segment_index == 0);
        std::printf("Smoke test %s: %d frames, demo job at segment %zu\n", failed ? "FAILED" : "passed", frames,
                    t.active_segment_index);
        if (failed) std::printf("  state %d, message: %s\n", static_cast<int>(t.state), t.active_fault_message.c_str());
        std::fflush(stdout);
        window.closeWindow();
    }
}

} // namespace rnr::ui
