// Settings sections both applications show the same way (BreakoutMill: 4 axes, BreakoutCut: 3)
#include "ui_common/machine_settings_sections.h"
#include "ui_common/board_panels.h"
#include "ui_common/mach3_migration_dialog.h"
#include "ui_common/theme.h"
#include "ui_common/imgui_text_helpers.h"
#include "ui_common/corner_settings.h"
#include "ui_common/gantry_slave_combo.h"
#include "ui_common/program_start_settings.h"
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace rnr::ui {

void renderLimitsHomingSection(const SettingsSectionContext& ctx) {
    auto& controller = ctx.controller;
    const auto& board_caps = ctx.board_caps;
    const bool host_board = board_caps.family == board::Family::HostPlanned;
    auto& ui_settings = ctx.ui_settings;
    auto markSettingsDirty = [&] { if (ctx.mark_dirty) ctx.mark_dirty(); };
    (void)host_board;
    (void)ui_settings;
    core::MachineConfig cfg = *controller.getConfigShared();
    bool changed = false;

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
    ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

    if (host_board) {
    // HARDWARE E-STOP INPUT PIN
    ImGui::BeginChild("##EStopLimitCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::TextColored(ui::theme::pal().heading, "E-STOP INPUT");
        ImGui::Spacing();

        int estop_pin = cfg.estop_pin;
        ImGui::SetNextItemWidth(140.0f);
        if (ImGui::SliderInt("E-Stop Pin (0=Off, IN1..12)", &estop_pin, 0, 12)) {
            cfg.estop_pin = static_cast<uint8_t>(estop_pin);
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Active Low##EStop", &cfg.estop_invert)) changed = true;
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Active Low: Checked = Normally-Closed (NC) switch connected to COM/GND.\nTriggers E-Stop when opened or pulled low. Unchecked = Active High (NO switch).");
        }
    }
    ImGui::EndChild();

    ImGui::Spacing();

    }
    // HOMING REQUIREMENT
    ImGui::BeginChild("##HomingRequiredCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::TextColored(ui::theme::pal().heading, "HOMING");
        ImGui::Spacing();
        if (ImGui::Checkbox("Require homing for G53 / G28 / G30##ReqHome", &cfg.require_homing)) changed = true;
        ImGui::PushTextWrapPos(0.0f);
        if (cfg.require_homing) {
            ImGui::TextDisabled("Programs and MDI commands that move in machine coordinates (G53, G28, G30) "
                                "are refused until those axes are homed.");
        } else {
            ImGui::TextColored(ui::theme::pal().text_warn,
                               "Machine coordinates are then not tied to switches: they continue from the position saved when the app last closed. "
                               "If the axes were moved by hand or lost steps, G53 / G28 / G30 moves go to the wrong place. "
                               "Use this only on a machine without home switches.");
        }
        ImGui::PopTextWrapPos();
    }
    ImGui::EndChild();

    ImGui::Spacing();

    // Automatic Z lift at program start
    ImGui::BeginChild("##ProgramStartCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::TextColored(ui::theme::pal().heading, "PROGRAM START");
        ImGui::Spacing();
        if (ui::renderProgramStartSettings(cfg)) changed = true;
    }
    ImGui::EndChild();

    ImGui::Spacing();

    if (host_board) {
    // Constant velocity: corner tolerance and curve speed limit
    ImGui::BeginChild("##CornersCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::TextColored(ui::theme::pal().heading, "CORNERS & CURVES");
        ImGui::Spacing();
        if (ui::renderCornerSettings(cfg)) changed = true;
    }
    ImGui::EndChild();

    ImGui::Spacing();

    }
    if (!host_board) {
        ui::renderNotUsedByBoardNote("Limit switches, homing speeds, pull-off and soft limits", board_caps);
    } else {
    const char* ax_names[4] = {"X AXIS LIMITS & HOMING", "Y AXIS LIMITS & HOMING", "Z AXIS LIMITS & HOMING", "A AXIS LIMITS & HOMING"};
    const char* card_ids[4] = {"##LimitsXCard", "##LimitsYCard", "##LimitsZCard", "##LimitsACard"};
    for (int a = 0; a < ctx.axis_count; ++a) {
        ImGui::PushID(100 + a);
        ImGui::BeginChild(card_ids[a], ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
        {
            ImGui::TextColored(ui::theme::pal().heading, "%s", ax_names[a]);
            ImGui::Spacing();

            int home_pin = cfg.axes[a].home_pin;
            int lim_m    = cfg.axes[a].limit_minus_pin;
            int lim_p    = cfg.axes[a].limit_plus_pin;

            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::SliderInt("Home Pin (0=Off, IN1..12)", &home_pin, 0, 12)) {
                cfg.axes[a].home_pin = static_cast<uint8_t>(home_pin);
                changed = true;
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Active Low##H", &cfg.axes[a].home_invert)) changed = true;
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Active Low: Checked = Switch connects to COM/GND when triggered.\nUnchecked = Active High.");
            }

            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::SliderInt("Limit- Pin (0=Off, IN1..12)", &lim_m, 0, 12)) {
                cfg.axes[a].limit_minus_pin = static_cast<uint8_t>(lim_m);
                changed = true;
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Active Low##LM", &cfg.axes[a].limit_minus_invert)) changed = true;
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Active Low: Checked = Switch connects to COM/GND when triggered.\nUnchecked = Active High.");
            }

            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::SliderInt("Limit+ Pin (0=Off, IN1..12)", &lim_p, 0, 12)) {
                cfg.axes[a].limit_plus_pin = static_cast<uint8_t>(lim_p);
                changed = true;
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Active Low##LP", &cfg.axes[a].limit_plus_invert)) changed = true;
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Active Low: Checked = Switch connects to COM/GND when triggered.\nUnchecked = Active High.");
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Checkbox("Home Dir (-)", &cfg.axes[a].homing_dir_negative)) {
                changed = true;
            }

            bool is_imp = (cfg.linear_units == core::LinearUnits::Imperial);
            double dist_scale = is_imp ? (1.0 / 25.4) : 1.0;

            float pulloff = static_cast<float>(cfg.axes[a].home_pulloff_mm * dist_scale);
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::InputFloat(is_imp ? "Home Pull-Off (in)" : "Home Pull-Off (mm)", &pulloff, is_imp ? 0.02f : 0.5f, is_imp ? 0.1f : 2.0f, is_imp ? "%.3f" : "%.2f")) {
                cfg.axes[a].home_pulloff_mm = std::max(0.01, static_cast<double>(pulloff) / dist_scale);
                changed = true;
            }

            float h_feed = static_cast<float>(cfg.axes[a].homing_feed_mm_min * dist_scale);
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::InputFloat(is_imp ? "Homing Speed (IPM)" : "Homing Speed (mm/min)", &h_feed, is_imp ? 2.0f : 50.0f, is_imp ? 10.0f : 200.0f, is_imp ? "%.1f" : "%.0f")) {
                cfg.axes[a].homing_feed_mm_min = std::max(1.0, static_cast<double>(h_feed) / dist_scale);
                changed = true;
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Checkbox("Enable Soft Limits", &cfg.axes[a].soft_limits_enabled)) {
                changed = true;
            }
            if (cfg.axes[a].soft_limits_enabled) {
                float smin = static_cast<float>(cfg.axes[a].soft_limit_min_mm * dist_scale);
                float smax = static_cast<float>(cfg.axes[a].soft_limit_max_mm * dist_scale);
                ImGui::SetNextItemWidth(140.0f);
                if (ImGui::InputFloat(is_imp ? "Soft Limit Min (in)" : "Soft Limit Min (mm)", &smin, is_imp ? 0.2f : 5.0f, is_imp ? 1.0f : 25.0f, is_imp ? "%.2f" : "%.1f")) {
                    cfg.axes[a].soft_limit_min_mm = static_cast<double>(smin) / dist_scale;
                    changed = true;
                }
                ImGui::SetNextItemWidth(140.0f);
                if (ImGui::InputFloat(is_imp ? "Soft Limit Max (in)" : "Soft Limit Max (mm)", &smax, is_imp ? 0.2f : 5.0f, is_imp ? 1.0f : 25.0f, is_imp ? "%.2f" : "%.1f")) {
                    cfg.axes[a].soft_limit_max_mm = static_cast<double>(smax) / dist_scale;
                    changed = true;
                }
            }
        }
        ImGui::EndChild();
        ImGui::PopID();
        ImGui::Spacing();
    }

    }
    ImGui::PopStyleVar(4);
    ImGui::PopStyleColor(2);

    if (changed) {
        controller.applyConfig(cfg);
        markSettingsDirty();
    }
}

void renderMotorsSection(const SettingsSectionContext& ctx) {
    auto& controller = ctx.controller;
    const auto& board_caps = ctx.board_caps;
    const bool host_board = board_caps.family == board::Family::HostPlanned;
    auto& ui_settings = ctx.ui_settings;
    auto markSettingsDirty = [&] { if (ctx.mark_dirty) ctx.mark_dirty(); };
    (void)host_board;
    (void)ui_settings;
    core::MachineConfig cfg = *controller.getConfigShared();
    bool changed = false;

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
    ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

    // GLOBAL SYSTEM OF MEASUREMENT & VELOCITY CONFIGURATION
    ImGui::BeginChild("##MotorUnitCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::TextColored(ui::theme::pal().heading, "UNITS & MACH3 IMPORT");
        ImGui::Spacing();

        // Mach3 Migration Action Button
        ImGui::PushStyleColor(ImGuiCol_Button,        ui::theme::pal().btn_primary);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ui::theme::pal().btn_primary_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ui::theme::pal().btn_primary_act);
        ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::contrastText(ui::theme::pal().btn_primary));
        if (ImGui::Button("MIGRATE FROM MACH3 XML...", ImVec2(-1.0f, 30.0f))) {
            if (ctx.mach3_wizard) ctx.mach3_wizard->openMigrationWizard(*controller.getConfigShared());
        }
        ImGui::PopStyleColor(4);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Import motor tuning (steps/unit, velocity, acceleration), pin assignments,\ninversions, homing speeds, soft limits, and spindle / torch outputs directly from a Mach3 XML profile\nfollowed by the interactive 5-Stage Post-Migration Machine Verification Routine.");
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Unit Selector
        ImGui::Text("Unit of Measurement:");
        ImGui::SameLine();
        const bool is_metric = (cfg.linear_units == core::LinearUnits::Metric);
        const bool is_imperial = (cfg.linear_units == core::LinearUnits::Imperial);

        if (is_metric) {
            ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_primary);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ui::theme::pal().btn_primary_hov);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_neutral);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ui::theme::pal().btn_neutral_hov);
        }
        ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::contrastText(ImGui::GetStyleColorVec4(ImGuiCol_Button)));
        if (ImGui::Button("  METRIC (mm)  ")) {
            if (!is_metric) {
                cfg.linear_units = core::LinearUnits::Metric;
                ui_settings.speed_unit_idx = 0;
                changed = true;
            }
        }
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Set system-wide machine units to Metric (Millimeters, mm/min, mm/s).");
        }

        ImGui::SameLine();

        if (is_imperial) {
            ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_primary);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ui::theme::pal().btn_primary_hov);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_neutral);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ui::theme::pal().btn_neutral_hov);
        }
        ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::contrastText(ImGui::GetStyleColorVec4(ImGuiCol_Button)));
        if (ImGui::Button("  IMPERIAL (Inches)  ")) {
            if (!is_imperial) {
                cfg.linear_units = core::LinearUnits::Imperial;
                ui_settings.speed_unit_idx = 0;
                changed = true;
            }
        }
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Set system-wide machine units to Imperial (Inches, in/min [IPM], in/s).");
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Filtered velocity configuration unit based on global unit
        const char* metric_speed_units[] = {
            "mm/min (Metric mm/m)",
            "mm/sec (Metric mm/s)"
        };
        const char* imperial_speed_units[] = {
            "in/min (Imperial IPM)",
            "in/sec (Imperial in/s)"
        };

        int current_speed_idx = std::clamp(ui_settings.speed_unit_idx, 0, 1);

        ImGui::SetNextItemWidth(220.0f);
        if (is_metric) {
            if (ImGui::Combo("Velocity Unit", &current_speed_idx, metric_speed_units, IM_ARRAYSIZE(metric_speed_units))) {
                ui_settings.speed_unit_idx = current_speed_idx;
                markSettingsDirty();
            }
        } else {
            if (ImGui::Combo("Velocity Unit", &current_speed_idx, imperial_speed_units, IM_ARRAYSIZE(imperial_speed_units))) {
                ui_settings.speed_unit_idx = current_speed_idx;
                markSettingsDirty();
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Configure the units used for max velocity and feedrate tuning for all axes.");
        }
    }
    ImGui::EndChild();

    ImGui::Spacing();

    const bool is_imperial = (cfg.linear_units == core::LinearUnits::Imperial);
    const int speed_sub_idx = std::clamp(ui_settings.speed_unit_idx, 0, 1);

    double to_disp_factor = 60.0;
    const char* speed_label = "Max Speed (mm/m)";
    const char* speed_fmt = "%.0f";
    float speed_step = 100.0f;
    float speed_step_fast = 500.0f;

    const char* steps_label = "Steps/mm";
    const char* accel_label = "Acceleration (mm/s^2)";
    const char* decel_label = "Deceleration (mm/s^2)";
    double steps_mult = 1.0;
    double accel_mult = 1.0;

    if (!is_imperial) {
        if (speed_sub_idx == 1) {
            to_disp_factor = 1.0;
            speed_label = "Max Speed (mm/s)";
            speed_fmt = "%.1f";
            speed_step = 5.0f;
            speed_step_fast = 25.0f;
        } else {
            to_disp_factor = 60.0;
            speed_label = "Max Speed (mm/m)";
            speed_fmt = "%.0f";
            speed_step = 100.0f;
            speed_step_fast = 500.0f;
        }
        steps_label = "Steps/mm";
        accel_label = "Acceleration (mm/s^2)";
        decel_label = "Deceleration (mm/s^2)";
        steps_mult = 1.0;
        accel_mult = 1.0;
    } else {
        if (speed_sub_idx == 1) {
            to_disp_factor = 1.0 / 25.4;
            speed_label = "Max Speed (in/s)";
            speed_fmt = "%.2f";
            speed_step = 0.5f;
            speed_step_fast = 2.0f;
        } else {
            to_disp_factor = 60.0 / 25.4;
            speed_label = "Max Speed (IPM)";
            speed_fmt = "%.1f";
            speed_step = 10.0f;
            speed_step_fast = 50.0f;
        }
        steps_label = "Steps/inch";
        accel_label = "Acceleration (in/s^2)";
        decel_label = "Deceleration (in/s^2)";
        steps_mult = 25.4;
        accel_mult = 1.0 / 25.4;
    }

    const char* ax_names[4] = {"X AXIS MOTOR TUNING", "Y AXIS MOTOR TUNING", "Z AXIS MOTOR TUNING", "A AXIS MOTOR TUNING"};
    const char* motor_card_ids[4] = {"##MotorXCard", "##MotorYCard", "##MotorZCard", "##MotorACard"};
    for (int a = 0; a < ctx.axis_count; ++a) {
        ImGui::PushID(200 + a);
        ImGui::BeginChild(motor_card_ids[a], ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
        {
            ImGui::TextColored(ui::theme::pal().heading, "%s", ax_names[a]);
            ImGui::Spacing();

            float steps = static_cast<float>(cfg.axes[a].steps_per_mm * steps_mult);
            float disp_v = static_cast<float>(cfg.axes[a].max_velocity_mm_s * to_disp_factor);
            float acc   = static_cast<float>(cfg.axes[a].acceleration_mm_s2 * accel_mult);
            float dec   = static_cast<float>(cfg.axes[a].deceleration_mm_s2 * accel_mult);

            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::InputFloat(steps_label, &steps, 1.0f, 10.0f, "%.2f")) {
                cfg.axes[a].steps_per_mm = std::max(0.1, static_cast<double>(steps) / steps_mult);
                changed = true;
            }
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::InputFloat(speed_label, &disp_v, speed_step, speed_step_fast, speed_fmt)) {
                cfg.axes[a].max_velocity_mm_s = std::max(0.01, static_cast<double>(disp_v) / to_disp_factor);
                changed = true;
            }
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::InputFloat(accel_label, &acc, is_imperial ? 1.0f : 25.0f, is_imperial ? 5.0f : 100.0f, "%.1f")) {
                cfg.axes[a].acceleration_mm_s2 = std::max(0.5, static_cast<double>(acc) / accel_mult);
                changed = true;
            }
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::InputFloat(decel_label, &dec, is_imperial ? 1.0f : 25.0f, is_imperial ? 5.0f : 100.0f, "%.1f")) {
                cfg.axes[a].deceleration_mm_s2 = std::max(0.5, static_cast<double>(dec) / accel_mult);
                changed = true;
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Checkbox("Step Active Low", &cfg.axes[a].step_active_low)) {
                changed = true;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Invert step pulse polarity (Active Low vs Active High) for stepper driver optocoupler.");
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Dir Invert (Active Low)", &cfg.axes[a].dir_active_low)) {
                changed = true;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Invert direction signal polarity to reverse motor rotation without swapping motor wires.");
            }
        }
        ImGui::EndChild();
        ImGui::PopID();
        ImGui::Spacing();
    }

    if (ui::renderGantrySlaveCombo(cfg)) {
        changed = true;
    }

    ImGui::PopStyleVar(4);
    ImGui::PopStyleColor(2);

    if (changed) {
        controller.applyConfig(cfg);
        markSettingsDirty();
    }
}

} // namespace rnr::ui
