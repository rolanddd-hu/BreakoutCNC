#include "ui_common/io_status_bar.h"
#include "ui_common/theme.h"
#include "core/io/io_map.h"
#include "ui_common/gcode_panel.h"
#include "ui_common/diagnostics_report.h"
#include "ui_common/board_panels.h"
#include "core/board/io_model.h"
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
        case core::ControllerState::Disconnected:   return theme::pal().btn_danger_subtle;
        case core::ControllerState::EStop:          return theme::pal().btn_danger;
        case core::ControllerState::Idle:           return theme::pal().btn_ok;
        case core::ControllerState::RunningGCode:   return theme::pal().btn_primary;
        case core::ControllerState::DwellPierce:    return theme::pal().btn_warn;
        case core::ControllerState::PausedHold:     return theme::pal().btn_warn;
        case core::ControllerState::ToolChangeHold: return theme::pal().btn_warn;
        default:                                    return theme::pal().btn_primary;
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
    ImGui::TextColored(theme::pal().text_info, "%s", app_title);
    ImGui::SetWindowFontScale(1.0f);

    ImGui::SameLine(235.0f);
    if (telemetry.usb_connected && !telemetry.simulator_mode) {
        ImGui::TextColored(theme::pal().text_ok,
                           "Connected: %s (SN %s)",
                           telemetry.product_name.c_str(),
                           telemetry.serial_number.c_str());
    } else if (telemetry.simulator_mode) {
        ImGui::TextColored(theme::pal().text_warn,
                           "Simulator (no board)");
    } else {
        ImGui::TextColored(theme::pal().text_danger,
                           "Board not connected");
    }

    ImGui::SameLine();
    if (ImGui::SmallButton("Connect")) {
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
            ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_ok);
            ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));
        }
        if (ImGui::SmallButton(flash ? "Settings Saved!" : "Save Settings")) {
            on_save_settings();
            s_saved_flash_until = ImGui::GetTime() + 1.8;
        }
        if (flash) {
            ImGui::PopStyleColor(2);
        }
    }

    const auto& cur_cfg = controller.getConfig();
    ImGui::SameLine();
    if (cur_cfg.linear_units == core::LinearUnits::Metric) {
        ImGui::TextColored(theme::pal().text_info, "Units: mm");
    } else {
        ImGui::TextColored(theme::pal().text_warn, "Units: inch");
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("System of Measurement: %s\nConfigured under Motors in the Settings panel.",
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
            ImGui::PushStyleColor(ImGuiCol_Button, selected ? theme::pal().btn_primary
                                                            : theme::pal().btn_neutral);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, selected ? theme::pal().btn_primary_hov
                                                                   : theme::pal().btn_neutral_hov);
            ImGui::PushStyleColor(ImGuiCol_Text, selected ? theme::pal().text_on_fill
                                                          : theme::pal().text_muted);
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
        badge_col = theme::pal().btn_warn;
    } else if (telemetry.spindle_ramping_down) {
        std::snprintf(badge_buf, sizeof(badge_buf), "SPINDLE STOP (%3.0f%%)", telemetry.spindle_speed_pct);
        badge_txt = badge_buf;
        badge_col = theme::pal().btn_warn;
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
        ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_warn);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_warn));
        if (ImGui::Button("RESET E-STOP", ImVec2(170.0f, 32.0f))) {
            controller.resetFromEStop();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Click to Clear Alarm and Reset Machine to Ready/Idle.\nFault: %s",
                              telemetry.active_fault_message.empty()
                                  ? "None"
                                  : telemetry.active_fault_message.c_str());
        }
        ImGui::PopStyleColor(2);
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_danger);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_danger_hov);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_danger));
        if (ImGui::Button("EMERGENCY STOP", ImVec2(170.0f, 32.0f))) {
            controller.triggerEStop();
        }
        ImGui::PopStyleColor(3);
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

    // Controller State Diagnostic Display (Recessed LCD status gauge - not a clickable button)
    char badge_buf[48];
    const char* badge_txt = stateToString(telemetry.state);
    ImVec4 badge_col = stateToColor(telemetry.state);
    if (telemetry.spindle_ramping_up) {
        std::snprintf(badge_buf, sizeof(badge_buf), "SPINDLE RAMP (%3.0f%%)", telemetry.spindle_speed_pct);
        badge_txt = badge_buf;
        badge_col = theme::pal().btn_warn;
    } else if (telemetry.spindle_ramping_down) {
        std::snprintf(badge_buf, sizeof(badge_buf), "SPINDLE STOP (%3.0f%%)", telemetry.spindle_speed_pct);
        badge_txt = badge_buf;
        badge_col = theme::pal().btn_warn;
    }

    const float badge_h = 26.0f;
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + avail_w, p0.y + badge_h);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 col_accent = ImGui::ColorConvertFloat4ToU32(badge_col);

    // Dark recessed LCD readout bezel
    dl->AddRectFilled(p0, p1, theme::pal().readout_bezel, 4.0f);
    dl->AddRect(p0, p1, theme::pal().readout_bezel_border, 4.0f, 0, 1.0f);

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
    dl->AddText(ImVec2(pfx_x, ty), theme::pal().readout_prefix, prefix);

    // State text in the state's color
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

    // Cycle Start & Emergency Stop Buttons: [ CYCLE START ] [ EMERGENCY STOP ]
    ImGui::BeginDisabled(is_estop);
    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_ok);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_ok_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_ok_act);
    ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));

    const bool is_surf_comp = telemetry.surface_compensation_active;
    if (is_surf_comp) {
        ImGui::PushStyleColor(ImGuiCol_Border, theme::pal().border_ok);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
    }

    const bool compact = (avail_w < 340.0f);
    std::string start_label_str;
    if (is_tool_change) {
        start_label_str = compact ? "RESUME (M6)" : "RESUME CYCLE (M6)";
    } else if (is_surf_comp) {
        start_label_str = compact ? "CYCLE START\nsurface comp." : "CYCLE START\nsurface compensated";
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
    ImGui::PopStyleColor(4);
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
        ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_warn);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_warn_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_warn_act);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_warn));
        if (ImGui::Button("RESET E-STOP", ImVec2(btn_w1, btn_large_h))) {
            controller.resetFromEStop();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Click to Clear Alarm and Reset Machine to Ready/Idle.\nFault: %s",
                              telemetry.active_fault_message.empty()
                                  ? "None"
                                  : telemetry.active_fault_message.c_str());
        }
        ImGui::PopStyleColor(4);
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_danger);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_danger_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_danger_act);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_danger));
        if (ImGui::Button("EMERGENCY STOP", ImVec2(btn_w1, btn_large_h))) {
            controller.triggerEStop();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Emergency Stop: Immediately halt all motion and spindle.");
        }
        ImGui::PopStyleColor(4);
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
    // Row 0: which board, and how to reach it
    renderBoardSelector(controller, telemetry, on_settings_changed);
    const bool firmware_board = telemetry.board_firmware_planned;

    // Row 1: Connection status badge
    if (telemetry.usb_connected && !telemetry.simulator_mode && firmware_board) {
        ImGui::TextColored(telemetry.board_connecting ? theme::pal().text_warn : theme::pal().text_ok,
                           "%s: %s (%s)", telemetry.board_connecting ? "Connecting" : "Connected",
                           telemetry.product_name.c_str(), telemetry.serial_number.c_str());
    } else if (telemetry.usb_connected && !telemetry.simulator_mode) {
        ImGui::TextColored(theme::pal().text_ok,
                           "Connected: %s (SN %s)",
                           telemetry.product_name.c_str(),
                           telemetry.serial_number.c_str());
    } else if (telemetry.simulator_mode) {
        ImGui::TextColored(theme::pal().text_warn,
                           "Simulator (no board)");
    } else {
        ImGui::TextColored(theme::pal().text_danger,
                           "Board not connected");
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
    const char* connect_lbl = firmware_board ? (telemetry.usb_connected ? "Reconnect" : "Connect")
                                             : (telemetry.usb_connected ? "Reconnect USB" : "Connect USB");
    if (ImGui::Button(connect_lbl, ImVec2(btn_w, btn_h))) {
        controller.connect();
    }

    // Button 2: Disconnect USB (placed next to Reconnect USB)
    ImGui::SameLine();
    const bool can_disconnect = (telemetry.usb_connected && !telemetry.simulator_mode);
    ImGui::BeginDisabled(!can_disconnect);
    if (ImGui::Button(firmware_board ? "Disconnect" : "Disconnect USB", ImVec2(btn_w, btn_h))) {
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

    if (firmware_board) return;
    if (telemetry.usb_connected && !telemetry.simulator_mode) {
        renderPulseRateSetting(controller, telemetry, on_settings_changed);
    }

    // Row 3: Hardware specs
    ImGui::TextColored(theme::pal().text_muted,
                       "Hardware: VID 0xA720 | PID 0xF803");
}

void IoStatusBar::renderPulseRateSetting(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    const std::function<void()>& on_settings_changed)
{
    core::MachineConfig cfg = controller.getConfig();
    const auto& rates = core::MachineConfig::kPulseRatesKhz;

    ImGui::Spacing();
    ImGui::BeginChild("##PulseRateCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY,
                      ImGuiWindowFlags_NoScrollbar);
    ImGui::TextColored(theme::pal().heading, "STEP PULSE RATE");
    ImGui::Spacing();

    // Changing it reconfigures the board, so only while nothing moves
    const bool can_change = telemetry.state == core::ControllerState::Idle ||
                            telemetry.state == core::ControllerState::EStop;
    ImGui::BeginDisabled(!can_change);
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float n       = static_cast<float>(rates.size());
    const float btn_w   = std::max(ImGui::CalcTextSize("100 kHz").x + 32.0f,
                                   std::floor((ImGui::GetContentRegionAvail().x - spacing * (n - 1.0f)) / n));
    for (size_t i = 0; i < rates.size(); ++i) {
        if (i > 0) ImGui::SameLine(0.0f, spacing);
        char label[32];
        std::snprintf(label, sizeof(label), "%u kHz##PulseRate%u", rates[i], rates[i]);
        const bool selected = cfg.pulse_rate_khz == rates[i];
        if (selected) {
            ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_primary);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_primary_hov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_primary_act);
            ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_primary));
        }
        if (ImGui::Button(label, ImVec2(btn_w, 28.0f)) && !selected) {
            cfg.pulse_rate_khz = rates[i];
            controller.applyConfig(cfg);
            if (on_settings_changed) on_settings_changed();
        }
        if (selected) ImGui::PopStyleColor(4);
    }
    ImGui::EndDisabled();
    if (!can_change && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Stop the machine to change the step pulse rate.");
    }
    ImGui::EndChild();
}

void IoStatusBar::renderGlobalAlarmBanner(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry)
{
    if (telemetry.state == core::ControllerState::EStop) {
        const float btn_w = 140.0f;
        const float btn_h = 28.0f;

        ImGui::PushStyleColor(ImGuiCol_ChildBg, theme::pal().alarm_bg);
        ImGui::PushStyleColor(ImGuiCol_Border,  theme::pal().alarm_border);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 6.0f));

        ImGui::BeginChild("##GlobalAlarmBanner", ImVec2(0.0f, 40.0f), true, ImGuiWindowFlags_NoScrollbar);
        ImGui::AlignTextToFramePadding();
        const char* msg = telemetry.active_fault_message.empty()
                              ? "Emergency Stop Active"
                              : telemetry.active_fault_message.c_str();
        ImGui::TextColored(theme::pal().alarm_text, "  E-STOP: %s", msg);

        const float right_x = ImGui::GetWindowWidth() - btn_w - 10.0f;
        ImGui::SameLine(std::max(ImGui::GetCursorPosX() + 10.0f, right_x));

        ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().alarm_btn);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().alarm_btn_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().alarm_btn_act);
        ImGui::PushStyleColor(ImGuiCol_Text,          theme::pal().alarm_btn_text);
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
        const float btn_h = 28.0f;

        ImGui::SetNextWindowPos(screen_pos);
        ImGui::SetNextWindowSize(ImVec2(width, 40.0f));

        // 85 % like the other toolpath view overlays, so the toolpath stays faintly visible behind the banner
        ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::pal().alarm_bg);
        ImGui::PushStyleColor(ImGuiCol_Border,   theme::pal().alarm_border);
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
            ImGui::TextColored(theme::pal().alarm_text, "  E-STOP: %s", msg);

            const float right_x    = width - btn_w - 10.0f;
            const float after_tx_x = ImGui::GetCursorPosX() + 10.0f;
            ImGui::SameLine(std::max(after_tx_x, right_x));

            ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().alarm_btn);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().alarm_btn_hov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().alarm_btn_act);
            ImGui::PushStyleColor(ImGuiCol_Text,          theme::pal().alarm_btn_text);
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

    if (telemetry.board_firmware_planned && !telemetry.simulator_mode) {
        // GRBL-family board: its own fixed terminals, as it reports them (A: and Pn:)
        const auto& caps = boardCaps(controller, telemetry);
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("OUTPUTS:");
        struct OutPill { const char* label; uint8_t bit; int channel; const char* tip; };
        const bool mill = controller.getConfig().is_mill;
        const OutPill outs[] = {
            {mill ? "Spindle" : "Torch / laser", core::OutputBit::OUT1_TorchOrSpindle, 1, "M3 / M5"},
            {"Flood", core::OutputBit::OUT3_FloodCoolant, 3, "M8 / M9"},
            {"Mist", core::OutputBit::OUT4_MistCoolant, 4, "M7 / M9"},
        };
        for (const auto& o : outs) {
            if (o.bit == core::OutputBit::OUT4_MistCoolant && !caps.mist_output) continue;
            ImGui::SameLine();
            const bool on = (telemetry.outputs_bitmask & o.bit) != 0;
            ImGui::PushStyleColor(ImGuiCol_Button, on ? theme::pal().btn_warn : theme::pal().btn_neutral);
            char lbl[48];
            std::snprintf(lbl, sizeof(lbl), "%s##FwOut%d", o.label, o.channel);
            if (ImGui::SmallButton(lbl)) controller.setAuxOutput(static_cast<uint8_t>(o.channel), !on);
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s (%s): %s\nClick to switch it %s", o.label, o.tip,
                                                          on ? "on" : "off", on ? "off" : "on");
        }
        ImGui::SameLine(0.0f, 18.0f);
        ImGui::TextDisabled("BOARD INPUTS:");
        for (size_t i = 0; i < caps.io.size() && i < 64; ++i) {
            const auto& pt = caps.io[i];
            if (!board::isInput(pt)) continue;
            ImGui::SameLine();
            const bool in_on = ((telemetry.board_inputs_mask >> i) & 1ULL) != 0;
            ImGui::PushStyleColor(ImGuiCol_Button, in_on ? theme::pal().btn_ok : theme::pal().btn_neutral);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, in_on ? theme::pal().btn_ok_hov : theme::pal().btn_neutral_hov);
            char lbl[64];
            std::snprintf(lbl, sizeof(lbl), "%s##FwIn%zu", pt.label.c_str(), i);
            ImGui::SmallButton(lbl);
            ImGui::PopStyleColor(2);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s: %s\nReported by the board (Pn:%c). Its polarity is set in the board's settings.",
                                  pt.label.c_str(), in_on ? "active" : "not active", pt.status_letter ? pt.status_letter : '?');
            }
        }
    } else {
        // Labels from the pin assignment: function or custom name per output / input (full text in the tooltips)
        const core::MachineConfig cfg = controller.getConfig();
        const auto mode = cfg.operation_mode;
        const auto input_uses = core::io::describeInputs(cfg);
        std::array<std::string, core::kPhysicalOutputCount> out_names;
        for (int pin = 1; pin <= core::kPhysicalOutputCount; ++pin) {
            const int ch = core::io::channelForPin(cfg.io, pin);
            out_names[pin - 1] = ch >= 0 ? core::io::channelShortName(ch, cfg.is_mill, mode) : cfg.io.output_name[pin - 1];
        }
        std::array<std::string, core::kInputCount> in_names;
        for (int i = 0; i < core::kInputCount; ++i) {
            in_names[i] = core::io::inputShortLabel(input_uses[i], cfg.io.input_name[i]);
        }

        // Full labels ("OUT1 Spindle", "IN5 Probe") when they fit next to the Diagnostics box, else pin numbers only
        const ImGuiStyle& st = ImGui::GetStyle();
        const char* in_title = telemetry.simulator_mode ? "INPUTS (Click to Simulate):" : "INPUTS:";
        auto pillW = [&](const std::string& text) { return ImGui::CalcTextSize(text.c_str()).x + st.FramePadding.x * 2.0f + st.ItemSpacing.x; };
        float full_w = ImGui::CalcTextSize("OUTPUTS:").x + ImGui::CalcTextSize(in_title).x + 18.0f + 2.0f * st.ItemSpacing.x;
        for (int pin = 1; pin <= core::kPhysicalOutputCount; ++pin) {
            full_w += pillW("OUT" + std::to_string(pin) + (out_names[pin - 1].empty() ? "" : " " + out_names[pin - 1]));
        }
        for (int i = 0; i < core::kInputCount; ++i) {
            full_w += pillW("IN" + std::to_string(i + 1) + (in_names[i].empty() ? "" : " " + in_names[i]));
        }
        const bool compact = full_w > ImGui::GetContentRegionAvail().x - 135.0f;

        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("OUTPUTS:");
        for (int pin = 1; pin <= core::kPhysicalOutputCount; ++pin) {
            ImGui::SameLine();
            const bool on = ((telemetry.physical_outputs_bitmask >> (pin - 1)) & 1U) != 0;
            const std::string& name = out_names[pin - 1];
            char lbl[64];
            if (compact || name.empty()) {
                std::snprintf(lbl, sizeof(lbl), "OUT%d##Out%d", pin, pin);
            } else {
                std::snprintf(lbl, sizeof(lbl), "OUT%d %s##Out%d", pin, name.c_str(), pin);
            }
            ImGui::PushStyleColor(ImGuiCol_Button, on ? theme::pal().btn_warn : theme::pal().btn_neutral);
            if (compact && !name.empty() && !on) ImGui::PushStyleColor(ImGuiCol_Text, theme::pal().text_info);
            if (ImGui::SmallButton(lbl)) {
                controller.setPhysicalOutput(static_cast<uint8_t>(pin), !on);
            }
            if (compact && !name.empty() && !on) ImGui::PopStyleColor();
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) {
                const int ch = core::io::channelForPin(cfg.io, pin);
                ImGui::SetTooltip("OUT%d: %s (%s)\nClick to switch it %s", pin,
                                  ch >= 0 ? core::io::channelName(ch, cfg.is_mill, mode)
                                          : (name.empty() ? "custom output, no name" : name.c_str()),
                                  on ? "on" : "off", on ? "off" : "on");
            }
        }

        ImGui::SameLine(0.0f, 18.0f);
        ImGui::TextDisabled("%s", in_title);
        for (int i = 0; i < core::kInputCount; ++i) {
            ImGui::SameLine();
            const bool in_on = ((telemetry.inputs_bitmask >> i) & 1U) != 0;
            const std::string& name = in_names[i];
            char lbl[64];
            if (compact || name.empty()) {
                std::snprintf(lbl, sizeof(lbl), "IN%d##In%d", i + 1, i + 1);
            } else {
                std::snprintf(lbl, sizeof(lbl), "IN%d %s##In%d", i + 1, name.c_str(), i + 1);
            }
            // Used inputs show their state in green; in compact mode an off but used input has blue text
            ImGui::PushStyleColor(ImGuiCol_Button, in_on ? theme::pal().btn_ok : theme::pal().btn_neutral);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, in_on ? theme::pal().btn_ok_hov : theme::pal().btn_neutral_hov);
            const bool mark = compact && !name.empty() && !in_on;
            if (mark) ImGui::PushStyleColor(ImGuiCol_Text, theme::pal().text_info);
            if (ImGui::SmallButton(lbl) && telemetry.simulator_mode) {
                const bool sim_pin = ((telemetry.simulated_inputs_bitmask >> i) & 1U) != 0;
                controller.setSimulatedInputPin(static_cast<uint8_t>(i + 1), !sim_pin);
            }
            if (mark) ImGui::PopStyleColor();
            ImGui::PopStyleColor(2);
            if (ImGui::IsItemHovered()) {
                std::string tip = "IN" + std::to_string(i + 1) + (in_on ? ": on\n" : ": off\n");
                if (!input_uses[i].empty()) {
                    for (const auto& u : input_uses[i]) {
                        tip += u.label + (u.invert ? " (active low)\n" : " (active high)\n");
                    }
                } else if (!cfg.io.input_name[i].empty()) {
                    tip += "Custom input \"" + cfg.io.input_name[i] + "\"" + (cfg.io.input_invert[i] ? " (inverted)\n" : "\n");
                } else {
                    tip += "Not used\n";
                }
                if (telemetry.simulator_mode) tip += "Click to simulate";
                ImGui::SetTooltip("%s", tip.c_str());
            }
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
                ImGui::TextColored(theme::pal().text_ok, "Copied");
            }
        }
        ImGui::End();
    }
}

} // namespace rnr::ui
