#include "workspace_panel.h"
#include "ui_common/theme.h"
#include <imgui.h>
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace rnr::ui {

WorkspacePanel::WorkspacePanel() = default;

void WorkspacePanel::render(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    bool show_4th_axis,
    const std::function<void()>& on_changed)
{
    const char* ws_names[6] = {"G54", "G55", "G56", "G57", "G58", "G59"};
    const uint8_t active_ws = telemetry.active_workspace;
    const theme::Palette& pal = theme::pal();

    // Two columns of cards; the same gap between them horizontally and vertically
    const float gap    = ImGui::GetStyle().ItemSpacing.x;
    const float pad    = 8.0f;
    const float btn_h  = 28.0f;
    const float set_w  = 52.0f;
    const float text_h = ImGui::GetTextLineHeight();
    const float line_gap = 2.0f;
    const float block_h  = std::max(btn_h, 2.0f * text_h + line_gap); // Two value lines, Set button beside them
    const float cell_w = std::floor((ImGui::GetContentRegionAvail().x - gap) * 0.5f);
    const float cell_h = pad + btn_h + 6.0f + block_h + pad;

    const char* axis_labels[4] = {"X", "Y", "Z", "A"};
    const ImVec4 axis_cols[4] = {pal.text_danger, pal.text_ok, pal.text_info, pal.text_warn};

    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(gap, gap));
    for (int i = 0; i < 6; ++i) {
        if (i % 2 != 0) {
            ImGui::SameLine(0.0f, gap);
        }

        const bool is_active = (active_ws == i);
        const auto& offsets = telemetry.workspace_offsets_mm[i];
        ImGui::PushID(i);

        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(pad, pad));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, is_active ? pal.card_active_bg : pal.surface_inset);
        ImGui::PushStyleColor(ImGuiCol_Border,  is_active ? pal.border_accent : pal.card_border);
        ImGui::BeginChild("##WsCell", ImVec2(cell_w, cell_h), ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        const float inner_w = ImGui::GetContentRegionAvail().x;

        // Header button: selects the coordinate system
        char btn_label[32];
        if (is_active) {
            std::snprintf(btn_label, sizeof(btn_label), "%s  Active", ws_names[i]);
            ImGui::PushStyleColor(ImGuiCol_Button,        pal.btn_primary);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, pal.btn_primary_hov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  pal.btn_primary_act);
            ImGui::PushStyleColor(ImGuiCol_Text,          pal.text_on_fill);
        } else {
            std::snprintf(btn_label, sizeof(btn_label), "%s", ws_names[i]);
            ImGui::PushStyleColor(ImGuiCol_Button,        pal.btn_neutral);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, pal.btn_neutral_hov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  pal.btn_neutral_act);
            ImGui::PushStyleColor(ImGuiCol_Text,          pal.text_muted);
        }
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 0.0f));
        if (ImGui::Button(btn_label, ImVec2(inner_w, btn_h)) && !is_active) {
            controller.selectWorkspace(static_cast<uint8_t>(i));
            if (on_changed) on_changed();
        }
        ImGui::PopStyleVar();
        ImGui::PopStyleColor(4);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(is_active ? "%s is the active coordinate system"
                                        : "Click to make %s the active coordinate system", ws_names[i]);
        }

        // Offsets in two aligned columns (X / Z left, Y / A right), Set button vertically centered beside them
        const float block_y = pad + btn_h + 6.0f;
        const float lines_y = block_y + (block_h - (2.0f * text_h + line_gap)) * 0.5f;
        const float col_w   = std::floor((inner_w - set_w - 8.0f) * 0.5f);
        const float letter_w = ImGui::CalcTextSize("X").x + 4.0f;
        const int   n_axes  = show_4th_axis ? 4 : 3;
        for (int a = 0; a < n_axes; ++a) {
            const float x = pad + (a % 2) * col_w;
            const float y = lines_y + (a / 2) * (text_h + line_gap);
            char buf[24];
            std::snprintf(buf, sizeof(buf), a == 3 ? "%+.1f" : "%+.2f", offsets[a]);
            ImGui::SetCursorPos(ImVec2(x, y));
            ImGui::TextColored(axis_cols[a], "%s", axis_labels[a]);
            // Value right-aligned in its column so the decimal points line up
            const float val_w = ImGui::CalcTextSize(buf).x;
            ImGui::SetCursorPos(ImVec2(std::max(x + letter_w, x + col_w - 6.0f - val_w), y));
            ImGui::TextUnformatted(buf);
        }

        ImGui::SetCursorPos(ImVec2(pad + inner_w - set_w, block_y + (block_h - btn_h) * 0.5f));
        ImGui::PushStyleColor(ImGuiCol_Button,        pal.btn_neutral);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, pal.btn_neutral_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  pal.btn_neutral_act);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 0.0f));
        if (ImGui::Button("Set", ImVec2(set_w, btn_h))) {
            edit_modal_ws_    = i;
            edit_temp_offset_ = offsets;
        }
        ImGui::PopStyleVar();
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Set or zero the %s offsets", ws_names[i]);
        }

        ImGui::EndChild();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(2);
        ImGui::PopID();
    }
    ImGui::PopStyleVar();

    // Modal: Set Workspace Coordinates
    if (edit_modal_ws_ >= 0 && edit_modal_ws_ < 6) {
        char modal_title[64];
        std::snprintf(modal_title, sizeof(modal_title), "Set %s Coordinate Offsets##WsModal", ws_names[edit_modal_ws_]);

        if (!ImGui::IsPopupOpen(modal_title)) {
            ImGui::OpenPopup(modal_title);
        }

        const float kModalW = 440.0f;
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(kModalW, 0.0f));
        ImGui::SetNextWindowSizeConstraints(ImVec2(kModalW, -1.0f), ImVec2(kModalW, -1.0f));
        if (ImGui::BeginPopupModal(modal_title, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Spacing();
            ImGui::TextColored(theme::pal().text_info, "SET COORDINATE OFFSETS: %s", ws_names[edit_modal_ws_]);
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::TextDisabled("Current Machine Coordinates (MCS):");
            if (show_4th_axis) {
                ImGui::Text("X=%+8.3f  Y=%+8.3f  Z=%+8.3f  A=%+8.3f",
                            telemetry.machine_pos_mm[0], telemetry.machine_pos_mm[1],
                            telemetry.machine_pos_mm[2], telemetry.machine_pos_mm[3]);
            } else {
                ImGui::Text("X=%+8.3f  Y=%+8.3f  Z=%+8.3f",
                            telemetry.machine_pos_mm[0], telemetry.machine_pos_mm[1],
                            telemetry.machine_pos_mm[2]);
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::Text("Workspace Origin Offset (MCS distance in mm):");
            ImGui::Spacing();

            if (ImGui::BeginTable("##WsOffsetsTbl", 2, ImGuiTableFlags_None)) {
                ImGui::TableSetupColumn("AxisLabel", ImGuiTableColumnFlags_WidthFixed, 105.0f);
                ImGui::TableSetupColumn("AxisInput", ImGuiTableColumnFlags_WidthStretch);

                // X Axis
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(theme::pal().text_danger, "X Offset (mm):");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputDouble("##WsX", &edit_temp_offset_[0], 1.0, 10.0, "%.3f");

                // Y Axis
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(theme::pal().text_ok, "Y Offset (mm):");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputDouble("##WsY", &edit_temp_offset_[1], 1.0, 10.0, "%.3f");

                // Z Axis
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(theme::pal().text_info, "Z Offset (mm):");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputDouble("##WsZ", &edit_temp_offset_[2], 1.0, 10.0, "%.3f");

                if (show_4th_axis) {
                    // A Axis
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextColored(theme::pal().text_warn, "A Offset (deg):");
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::InputDouble("##WsA", &edit_temp_offset_[3], 1.0, 10.0, "%.3f");
                }

                ImGui::EndTable();
            }

            ImGui::Spacing();

            // Quick Action Buttons
            if (ImGui::Button("Zero Here (Set to Current MCS)", ImVec2(-1.0f, 28.0f))) {
                edit_temp_offset_ = telemetry.machine_pos_mm;
            }
            if (ImGui::Button("Clear All to 0.000", ImVec2(-1.0f, 28.0f))) {
                edit_temp_offset_ = {0.0, 0.0, 0.0, 0.0};
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Confirmation row
            const float btn_half_w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
            ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_ok);
            ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));
            if (ImGui::Button("Save & Apply", ImVec2(btn_half_w, 32.0f))) {
                controller.setWorkspaceOffset(static_cast<uint8_t>(edit_modal_ws_), edit_temp_offset_);
                if (on_changed) on_changed();
                edit_modal_ws_ = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);

            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(btn_half_w, 32.0f))) {
                edit_modal_ws_ = -1;
                ImGui::CloseCurrentPopup();
            }

            ImGui::EndPopup();
        }
    }
}

} // namespace rnr::ui
