#include "ui_common/io_status_bar.h"
#include "ui_common/gcode_panel.h"
#include "ui_common/diagnostics_report.h"
#include "imgui.h"
#include <cstdio>
#include <algorithm>

namespace rnr::ui {

namespace {
const char* stateToString(core::ControllerState st) {
    switch (st) {
        case core::ControllerState::Disconnected:   return "DISCONNECTED";
        case core::ControllerState::EStop:          return "E-STOP ACTIVE";
        case core::ControllerState::Idle:           return "READY / IDLE";
        case core::ControllerState::RunningGCode:   return "CUTTING G-CODE";
        case core::ControllerState::PausedHold:     return "FEED HOLD (PAUSED)";
        case core::ControllerState::Jogging:        return "MANUAL JOG";
        case core::ControllerState::Homing:         return "HOMING";
        case core::ControllerState::Probing:        return "PROBING (G31)";
        case core::ControllerState::DwellPierce:    return "PIERCE DELAY (G4)";
        case core::ControllerState::ToolChangeHold: return "TOOL CHANGE (M6)";
        default:                                    return "UNKNOWN";
    }
}

ImVec4 stateToColor(core::ControllerState st) {
    switch (st) {
        case core::ControllerState::Disconnected:   return ImVec4(0.65f, 0.30f, 0.30f, 1.0f);
        case core::ControllerState::EStop:          return ImVec4(0.95f, 0.18f, 0.18f, 1.0f);
        case core::ControllerState::Idle:           return ImVec4(0.18f, 0.78f, 0.42f, 1.0f);
        case core::ControllerState::RunningGCode:   return ImVec4(0.00f, 0.82f, 0.96f, 1.0f);
        case core::ControllerState::DwellPierce:    return ImVec4(1.00f, 0.65f, 0.00f, 1.0f);
        case core::ControllerState::PausedHold:     return ImVec4(0.95f, 0.75f, 0.15f, 1.0f);
        case core::ControllerState::ToolChangeHold: return ImVec4(0.95f, 0.45f, 0.10f, 1.0f);
        default:                                    return ImVec4(0.35f, 0.65f, 0.95f, 1.0f);
    }
}
} // namespace

void IoStatusBar::renderTopBar(
    const char* app_title,
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    const std::function<void()>& on_save_settings,
    bool* minimal_layout)
{
    ImGui::BeginChild("##TopHeaderBar", ImVec2(0.0f, 48.0f), true, ImGuiWindowFlags_NoScrollbar);

    ImGui::SetWindowFontScale(1.18f);
    ImGui::TextColored(ImVec4(0.0f, 0.88f, 1.0f, 1.0f), "%s", app_title);
    ImGui::SetWindowFontScale(1.0f);

    ImGui::SameLine(235.0f);
    if (telemetry.usb_connected && !telemetry.simulator_mode) {
        ImGui::TextColored(ImVec4(0.20f, 0.95f, 0.50f, 1.0f),
                           "[USB LIVE: %s | SN: %s]",
                           telemetry.product_name.c_str(),
                           telemetry.serial_number.c_str());
    } else if (telemetry.simulator_mode) {
        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.25f, 1.0f),
                           "[SIMULATOR: Virtual RNR Eco Motion 2.0]");
    } else {
        ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.35f, 1.0f),
                           "[USB DISCONNECTED]");
    }

    ImGui::SameLine();
    if (ImGui::SmallButton("Connect USB")) {
        controller.connect();
    }
    ImGui::SameLine();
    bool sim = telemetry.simulator_mode;
    if (ImGui::Checkbox("Simulator", &sim)) {
        controller.setSimulatorMode(sim);
    }

    static double s_saved_flash_until = 0.0;
    if (on_save_settings) {
        ImGui::SameLine();
        bool flash = (ImGui::GetTime() < s_saved_flash_until);
        if (flash) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f, 0.62f, 0.34f, 1.0f));
        }
        if (ImGui::SmallButton(flash ? "Settings Saved!" : "Save Settings")) {
            on_save_settings();
            s_saved_flash_until = ImGui::GetTime() + 1.8;
        }
        if (flash) {
            ImGui::PopStyleColor();
        }
    }

    const auto& cur_cfg = controller.getConfig();
    ImGui::SameLine();
    if (cur_cfg.linear_units == core::LinearUnits::Metric) {
        ImGui::TextColored(ImVec4(0.35f, 0.75f, 1.0f, 1.0f), "[UNITS: MM]");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.70f, 0.30f, 1.0f), "[UNITS: INCH]");
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("System of Measurement: %s\nConfigured under Machine Motor Tuning in the Settings panel.",
                          cur_cfg.linear_units == core::LinearUnits::Metric ? "Metric (Millimeters)" : "Imperial (Inches)");
    }

    // Layout selector tabs (FULL / MINIMAL) centered in the top bar
    if (minimal_layout) {
        const float tab_w   = 96.0f;
        const float tab_h   = 30.0f;
        const float group_w = tab_w * 2.0f;
        ImGui::SameLine();
        const float min_x    = ImGui::GetCursorPosX() + 16.0f;
        const float center_x = (ImGui::GetWindowWidth() - group_w) * 0.5f;
        ImGui::SameLine(std::max(min_x, center_x));

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);
        const char* tab_labels[2] = {"FULL", "MINIMAL"};
        for (int t = 0; t < 2; ++t) {
            const bool selected = ((t == 1) == *minimal_layout);
            ImGui::PushStyleColor(ImGuiCol_Button, selected ? ImVec4(0.00f, 0.52f, 0.72f, 1.0f)
                                                            : ImVec4(0.13f, 0.16f, 0.21f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, selected ? ImVec4(0.00f, 0.60f, 0.82f, 1.0f)
                                                                   : ImVec4(0.20f, 0.26f, 0.34f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, selected ? ImVec4(1.0f, 1.0f, 1.0f, 1.0f)
                                                          : ImVec4(0.62f, 0.68f, 0.76f, 1.0f));
            if (t > 0) ImGui::SameLine();
            if (ImGui::Button(tab_labels[t], ImVec2(tab_w, tab_h))) {
                *minimal_layout = (t == 1);
            }
            ImGui::PopStyleColor(3);
        }
        ImGui::PopStyleVar(2);
    }

    // State badge
    ImGui::SameLine(ImGui::GetWindowWidth() - 370.0f);
    char badge_buf[48];
    const char* badge_txt = stateToString(telemetry.state);
    ImVec4 badge_col = stateToColor(telemetry.state);
    if (telemetry.spindle_ramping_up) {
        std::snprintf(badge_buf, sizeof(badge_buf), "SPINDLE RAMP (%3.0f%%)", telemetry.spindle_speed_pct);
        badge_txt = badge_buf;
        badge_col = ImVec4(0.95f, 0.58f, 0.08f, 1.0f);
    } else if (telemetry.spindle_ramping_down) {
        std::snprintf(badge_buf, sizeof(badge_buf), "SPINDLE STOP (%3.0f%%)", telemetry.spindle_speed_pct);
        badge_txt = badge_buf;
        badge_col = ImVec4(0.85f, 0.42f, 0.14f, 1.0f);
    }
    ImGui::PushStyleColor(ImGuiCol_Button, badge_col);
    ImGui::Button(badge_txt, ImVec2(175.0f, 32.0f));
    if (telemetry.state == core::ControllerState::EStop && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Emergency Stop Active:\n%s",
                          telemetry.active_fault_message.empty()
                              ? "E-Stop triggered manually or at startup."
                              : telemetry.active_fault_message.c_str());
    }
    ImGui::PopStyleColor();

    // E-Stop / Reset button on far right
    ImGui::SameLine(ImGui::GetWindowWidth() - 185.0f);
    if (telemetry.state == core::ControllerState::EStop) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.50f, 0.10f, 1.0f));
        if (ImGui::Button("RESET E-STOP", ImVec2(170.0f, 32.0f))) {
            controller.resetFromEStop();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Click to Clear Alarm and Reset Machine to Ready/Idle.\nFault: %s",
                              telemetry.active_fault_message.empty()
                                  ? "None"
                                  : telemetry.active_fault_message.c_str());
        }
        ImGui::PopStyleColor();
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.12f, 0.12f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.00f, 0.20f, 0.20f, 1.0f));
        if (ImGui::Button("EMERGENCY STOP", ImVec2(170.0f, 32.0f))) {
            controller.triggerEStop();
        }
        ImGui::PopStyleColor(2);
    }

    ImGui::EndChild();
}

void IoStatusBar::renderStatusAndEStopCard(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    float card_height,
    GCodeProgramPanel* gcode_panel)
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(ImGui::GetStyle().ItemSpacing.x, 5.0f));

    ImGui::BeginChild("##StatusEStopCard", ImVec2(0.0f, card_height), true,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    const float avail_w     = ImGui::GetContentRegionAvail().x;
    const float spacing_x   = ImGui::GetStyle().ItemSpacing.x;
    const float btn_w0      = std::floor((avail_w - spacing_x) * 0.5f);
    const float btn_w1      = avail_w - btn_w0 - spacing_x;
    const float btn_large_h = 64.0f;

    const bool is_estop       = (telemetry.state == core::ControllerState::EStop);
    const bool is_tool_change = (telemetry.state == core::ControllerState::ToolChangeHold ||
                                 telemetry.pending_tool_number > 0);

    // 1. Controller State Diagnostic Display (Recessed LCD status gauge - not a clickable button)
    char badge_buf[48];
    const char* badge_txt = stateToString(telemetry.state);
    ImVec4 badge_col = stateToColor(telemetry.state);
    if (telemetry.spindle_ramping_up) {
        std::snprintf(badge_buf, sizeof(badge_buf), "SPINDLE RAMP (%3.0f%%)", telemetry.spindle_speed_pct);
        badge_txt = badge_buf;
        badge_col = ImVec4(0.95f, 0.58f, 0.08f, 1.0f);
    } else if (telemetry.spindle_ramping_down) {
        std::snprintf(badge_buf, sizeof(badge_buf), "SPINDLE STOP (%3.0f%%)", telemetry.spindle_speed_pct);
        badge_txt = badge_buf;
        badge_col = ImVec4(0.85f, 0.42f, 0.14f, 1.0f);
    }

    const float badge_h = 26.0f;
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + avail_w, p0.y + badge_h);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 col_accent = ImGui::ColorConvertFloat4ToU32(badge_col);

    // Dark recessed LCD readout bezel
    dl->AddRectFilled(p0, p1, IM_COL32(11, 15, 22, 255), 4.0f);
    dl->AddRect(p0, p1, IM_COL32(36, 46, 62, 255), 4.0f, 0, 1.0f);

    // Pulsing soft warning aura when in E-Stop
    if (is_estop) {
        const float pulse = sinf(static_cast<float>(ImGui::GetTime()) * 5.0f) * 0.5f + 0.5f;
        dl->AddRectFilled(p0, p1, IM_COL32(220, 35, 35, static_cast<int>(16 + 24 * pulse)), 4.0f);
    }

    // Left vertical accent stripe
    dl->AddRectFilled(p0, ImVec2(p0.x + 4.0f, p1.y), col_accent, 4.0f, ImDrawFlags_RoundCornersLeft);

    // Centered LCD readout: [LED indicator] STATUS: <STATE_VALUE>
    const char* prefix = "STATUS: ";
    const ImVec2 pfx_sz = ImGui::CalcTextSize(prefix);
    const ImVec2 txt_sz = ImGui::CalcTextSize(badge_txt);
    const float led_radius = 3.5f;
    const float led_gap = 7.0f;
    const float total_content_w = (led_radius * 2.0f + led_gap) + pfx_sz.x + txt_sz.x;

    const float start_x = p0.x + std::max(10.0f, (avail_w - total_content_w) * 0.5f);
    const float led_cx = start_x + led_radius;
    const float led_cy = p0.y + badge_h * 0.5f;

    // Glowing LED outer halo
    dl->AddCircleFilled(ImVec2(led_cx, led_cy), 6.0f, (col_accent & 0x00FFFFFF) | 0x38000000, 16);
    // Glowing LED solid core
    dl->AddCircleFilled(ImVec2(led_cx, led_cy), led_radius, col_accent, 16);

    // Technical "STATUS:" label prefix
    const float pfx_x = start_x + led_radius * 2.0f + led_gap;
    const float ty = p0.y + (badge_h - txt_sz.y) * 0.5f;
    dl->AddText(ImVec2(pfx_x, ty), IM_COL32(130, 152, 178, 230), prefix);

    // Vibrant status value in state accent color
    const float val_x = pfx_x + pfx_sz.x;
    dl->AddText(ImVec2(val_x, ty), col_accent, badge_txt);

    ImGui::Dummy(ImVec2(avail_w, badge_h));
    if (telemetry.state == core::ControllerState::EStop && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Emergency Stop Active:\n%s",
                          telemetry.active_fault_message.empty()
                              ? "E-Stop triggered manually or at startup."
                              : telemetry.active_fault_message.c_str());
    } else if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Current Motion Controller State: %s", badge_txt);
    }

    // 2. Cycle Start & Emergency Stop Buttons: [ CYCLE START ] [ EMERGENCY STOP ]
    ImGui::BeginDisabled(is_estop);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.10f, 0.62f, 0.30f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.14f, 0.76f, 0.38f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.08f, 0.52f, 0.24f, 1.0f));

    const bool is_surf_comp = telemetry.surface_compensation_active;
    if (is_surf_comp) {
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.00f, 0.95f, 0.55f, 0.95f));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
    }

    const bool compact = (avail_w < 340.0f);
    std::string start_label_str;
    if (is_tool_change) {
        start_label_str = compact ? "RESUME (M6)" : "RESUME CYCLE (M6)";
    } else if (is_surf_comp) {
        start_label_str = compact ? "CYCLE START\n[SURF COMP]" : "CYCLE START\n[SURFACE COMPENSATED]";
    } else {
        start_label_str = "CYCLE START";
    }
    const char* start_label = start_label_str.c_str();

    if (ImGui::Button(start_label, ImVec2(btn_w0, btn_large_h))) {
        if (gcode_panel) {
            gcode_panel->triggerCycleStart(controller, telemetry);
        } else {
            controller.startJob();
        }
    }
    if (is_surf_comp) {
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
    }
    ImGui::PopStyleColor(3);
    ImGui::EndDisabled();
    if (is_estop && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Safety Lock: Cannot start cycle while E-Stop is active. Reset E-Stop first.");
    } else if (is_surf_comp && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Start G-Code execution with SURFACE COMPENSATION ACTIVE.\nAll Z movements contour to the probed height map.");
    } else if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Start G-Code execution / resume cut from current line");
    }

    ImGui::SameLine(0.0f, spacing_x);

    // Right side: EMERGENCY STOP / RESET E-STOP button
    if (is_estop) {
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.85f, 0.50f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.60f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.70f, 0.40f, 0.08f, 1.0f));
        if (ImGui::Button("RESET E-STOP", ImVec2(btn_w1, btn_large_h))) {
            controller.resetFromEStop();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Click to Clear Alarm and Reset Machine to Ready/Idle.\nFault: %s",
                              telemetry.active_fault_message.empty()
                                  ? "None"
                                  : telemetry.active_fault_message.c_str());
        }
        ImGui::PopStyleColor(3);
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.85f, 0.12f, 0.12f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.00f, 0.20f, 0.20f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.70f, 0.08f, 0.08f, 1.0f));
        if (ImGui::Button("EMERGENCY STOP", ImVec2(btn_w1, btn_large_h))) {
            controller.triggerEStop();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Emergency Stop: Immediately halt all motion and spindle.");
        }
        ImGui::PopStyleColor(3);
    }

    if (gcode_panel) {
        gcode_panel->renderSpindleModal(controller, telemetry);
    }

    ImGui::EndChild();
    ImGui::PopStyleVar(2);
}

void IoStatusBar::renderUsbConnectionSection(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    const std::function<void()>& on_settings_changed)
{
    // Row 1: Connection status badge
    if (telemetry.usb_connected && !telemetry.simulator_mode) {
        ImGui::TextColored(ImVec4(0.20f, 0.95f, 0.50f, 1.0f),
                           "[USB LIVE: %s | SN: %s]",
                           telemetry.product_name.c_str(),
                           telemetry.serial_number.c_str());
    } else if (telemetry.simulator_mode) {
        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.25f, 1.0f),
                           "[SIMULATOR: Virtual RNR Eco Motion 2.0 (200Hz Emulation)]");
    } else {
        ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.35f, 1.0f),
                           "[USB DISCONNECTED - Hardware Offline]");
    }

    // Row 2: Action buttons & toggles (both buttons side-by-side filling panel horizontally, checkbox at the end)
    const float avail_w = ImGui::GetContentRegionAvail().x;
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float chk_w   = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize("Simulator Mode").x + 10.0f;
    const float total_btn_w = std::max(180.0f, avail_w - chk_w - spacing * 2.0f);
    const float btn_w   = std::floor((total_btn_w - spacing) * 0.5f);
    const float btn_h   = 28.0f;

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 4.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5f, 0.5f));

    // Button 1: Reconnect USB / Connect USB
    if (ImGui::Button(telemetry.usb_connected ? "Reconnect USB" : "Connect USB", ImVec2(btn_w, btn_h))) {
        controller.connect();
    }

    // Button 2: Disconnect USB (placed next to Reconnect USB)
    ImGui::SameLine();
    const bool can_disconnect = (telemetry.usb_connected && !telemetry.simulator_mode);
    ImGui::BeginDisabled(!can_disconnect);
    if (ImGui::Button("Disconnect USB", ImVec2(btn_w, btn_h))) {
        controller.disconnect();
    }
    ImGui::EndDisabled();

    // Checkbox placed at the end, vertically aligned with buttons
    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 3.0f);
    bool sim = telemetry.simulator_mode;
    if (ImGui::Checkbox("Simulator Mode", &sim)) {
        controller.setSimulatorMode(sim);
        if (on_settings_changed) {
            on_settings_changed();
        }
    }
    ImGui::PopStyleVar(2);

    // Row 3: Hardware specs
    ImGui::TextColored(ImVec4(0.55f, 0.65f, 0.75f, 1.0f),
                       "Hardware: VID 0xA720 | PID 0xF803");
}

void IoStatusBar::renderGlobalAlarmBanner(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry)
{
    if (telemetry.state == core::ControllerState::EStop) {
        const float btn_w = 140.0f;
        const float btn_h = 26.0f;

        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.42f, 0.08f, 0.08f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.85f, 0.25f, 0.20f, 1.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 6.0f));

        ImGui::BeginChild("##GlobalAlarmBanner", ImVec2(0.0f, 38.0f), true, ImGuiWindowFlags_NoScrollbar);
        ImGui::AlignTextToFramePadding();
        const char* msg = telemetry.active_fault_message.empty()
                              ? "Emergency Stop Active"
                              : telemetry.active_fault_message.c_str();
        ImGui::TextColored(ImVec4(1.0f, 0.90f, 0.20f, 1.0f), "  [!] ALARM / EMERGENCY STOP: %s", msg);

        const float right_x = ImGui::GetWindowWidth() - btn_w - 10.0f;
        ImGui::SameLine(std::max(ImGui::GetCursorPosX() + 10.0f, right_x));

        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.00f, 0.55f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.70f, 0.35f, 0.08f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(1.00f, 1.00f, 1.00f, 1.0f));
        if (ImGui::Button("RESET E-STOP", ImVec2(btn_w, btn_h))) {
            controller.resetFromEStop();
        }
        ImGui::PopStyleColor(4);

        ImGui::EndChild();
        ImGui::PopStyleVar(3);
        ImGui::PopStyleColor(2);
        ImGui::Spacing();
    }
}

void IoStatusBar::renderFloatingAlarmBanner(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    ImVec2 screen_pos,
    float width)
{
    if (telemetry.state == core::ControllerState::EStop) {
        const float btn_w = 140.0f;
        const float btn_h = 26.0f;

        ImGui::SetNextWindowPos(screen_pos);
        ImGui::SetNextWindowSize(ImVec2(width, 38.0f));

        // 85 % like the other toolpath view overlays, so the toolpath stays faintly visible behind the banner
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.42f, 0.08f, 0.08f, 0.85f));
        ImGui::PushStyleColor(ImGuiCol_Border,   ImVec4(0.85f, 0.25f, 0.20f, 1.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 6.0f));

        if (ImGui::Begin("##ToolpathAlarmBannerOverlay", nullptr,
                         ImGuiWindowFlags_NoDecoration |
                         ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoDocking |
                         ImGuiWindowFlags_NoFocusOnAppearing |
                         ImGuiWindowFlags_NoNav))
        {
            ImGui::AlignTextToFramePadding();
            const char* msg = telemetry.active_fault_message.empty()
                                  ? "Emergency Stop Active"
                                  : telemetry.active_fault_message.c_str();
            ImGui::TextColored(ImVec4(1.0f, 0.90f, 0.20f, 1.0f), "  [!] ALARM / EMERGENCY STOP: %s", msg);

            const float right_x    = width - btn_w - 10.0f;
            const float after_tx_x = ImGui::GetCursorPosX() + 10.0f;
            ImGui::SameLine(std::max(after_tx_x, right_x));

            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.00f, 0.55f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.70f, 0.35f, 0.08f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(1.00f, 1.00f, 1.00f, 1.0f));
            if (ImGui::Button("RESET E-STOP", ImVec2(btn_w, btn_h))) {
                controller.resetFromEStop();
            }
            ImGui::PopStyleColor(4);
        }
        ImGui::End();
        ImGui::PopStyleVar(3);
        ImGui::PopStyleColor(2);
    }
}

void IoStatusBar::renderBottomIoStrip(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry)
{
    ImGui::BeginChild("##BottomIoStrip", ImVec2(0.0f, 40.0f), true, ImGuiWindowFlags_NoScrollbar);

    ImGui::TextDisabled("OUTPUTS:");
    ImGui::SameLine();
    const char* out_labels[4] = {"OUT1 (Torch/Spindle)", "OUT2 (Dir)", "OUT3 (Flood)", "OUT4 (Mist)"};
    for (int i = 0; i < 4; ++i) {
        if (i > 0) ImGui::SameLine();
        bool on = ((telemetry.outputs_bitmask >> i) & 1U) != 0;
        ImVec4 col = on ? ImVec4(1.0f, 0.48f, 0.05f, 1.0f) : ImVec4(0.18f, 0.21f, 0.26f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, col);
        if (ImGui::SmallButton(out_labels[i])) {
            controller.setAuxOutput(static_cast<uint8_t>(i + 1), !on);
        }
        ImGui::PopStyleColor();
    }

    ImGui::SameLine(0.0f, 18.0f);
    ImGui::TextDisabled(telemetry.simulator_mode ? "INPUTS (Click to Simulate):" : "INPUTS:");
    for (int i = 1; i <= 8; ++i) {
        ImGui::SameLine();
        bool in_on = ((telemetry.inputs_bitmask >> (i - 1)) & 1U) != 0;
        char lbl[16];
        std::snprintf(lbl, sizeof(lbl), "IN%d", i);
        if (telemetry.simulator_mode) {
            ImVec4 col = in_on ? ImVec4(0.14f, 0.72f, 0.36f, 1.0f) : ImVec4(0.18f, 0.21f, 0.26f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Button, col);
            if (ImGui::SmallButton(lbl)) {
                bool sim_pin = ((telemetry.simulated_inputs_bitmask >> (i - 1)) & 1U) != 0;
                controller.setSimulatedInputPin(static_cast<uint8_t>(i), !sim_pin);
            }
            ImGui::PopStyleColor();
        } else {
            ImGui::TextColored(in_on ? ImVec4(0.15f, 0.95f, 0.45f, 1.0f)
                                     : ImVec4(0.42f, 0.46f, 0.52f, 1.0f),
                               "[%s]", lbl);
        }
    }

    static bool show_diagnostics = false;
    if (show_diagnostics) {
        ImGui::SameLine(ImGui::GetWindowWidth() - 365.0f);
        ImGui::TextDisabled("AckSeq:%u | Tick:%u | %.0f FPS",
                            telemetry.last_ack_seq,
                            telemetry.tick_counter,
                            ImGui::GetIO().Framerate);
    }
    ImGui::SameLine(ImGui::GetWindowWidth() - 125.0f);
    ImGui::Checkbox("Diagnostics", &show_diagnostics);

    ImGui::EndChild();

    // Diagnostics report for bug reports: shown in full before it is copied, so users see what they share
    if (show_diagnostics) {
        static std::string report;
        static double report_time = -1.0;
        static double copied_time = -10.0;
        const double now = ImGui::GetTime();
        if (now - report_time > 1.0) { // Refreshed once a second: getProgram() / glGetString() are not free
            const auto program = controller.getProgram();
            report = buildDiagnosticsReport(controller.getConfig(), telemetry,
                                            program->lines.size(), program->segments.size());
            report_time = now;
        }
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 16.0f, vp->WorkPos.y + vp->WorkSize.y - 56.0f),
                                ImGuiCond_Appearing, ImVec2(1.0f, 1.0f));
        ImGui::SetNextWindowSize(ImVec2(560.0f, 0.0f), ImGuiCond_Appearing);
        if (ImGui::Begin("Diagnostics report", &show_diagnostics, ImGuiWindowFlags_NoCollapse)) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextDisabled("Paste this into a bug report on GitHub. It contains no file paths or serial numbers.");
            ImGui::PopTextWrapPos();
            ImGui::Separator();
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(report.c_str());
            ImGui::PopTextWrapPos();
            ImGui::Separator();
            const char* copy_label = "Copy to clipboard";
            if (ImGui::Button(copy_label, ImVec2(ImGui::CalcTextSize(copy_label).x + 32.0f, 32.0f))) {
                ImGui::SetClipboardText(report.c_str());
                copied_time = now;
            }
            if (now - copied_time < 2.0) {
                ImGui::SameLine();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.45f, 1.0f), "Copied");
            }
        }
        ImGui::End();
    }
}

} // namespace rnr::ui
