#include "workspace_panel.h"
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
    const float avail_w   = ImGui::GetContentRegionAvail().x;
    const float spacing_x = ImGui::GetStyle().ItemSpacing.x;
    const float cell_w    = std::floor((avail_w - spacing_x) * 0.5f);
    const float cell_h    = show_4th_axis ? 84.0f : 80.0f;

    const char* ws_names[6] = {"G54", "G55", "G56", "G57", "G58", "G59"};
    const uint8_t active_ws = telemetry.active_workspace;

    for (int i = 0; i < 6; ++i) {
        if (i % 2 != 0) {
            ImGui::SameLine(0.0f, spacing_x);
        }

        const bool is_active = (active_ws == i);
        const auto& offsets = telemetry.workspace_offsets_mm[i];

        char child_id[32];
        std::snprintf(child_id, sizeof(child_id), "##WsCell_%d", i);

        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(5.0f, 4.0f));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, is_active ? ImVec4(0.07f, 0.16f, 0.22f, 0.90f)
                                                          : ImVec4(0.08f, 0.10f, 0.13f, 0.80f));
        ImGui::PushStyleColor(ImGuiCol_Border,  is_active ? ImVec4(0.00f, 0.65f, 0.90f, 0.85f)
                                                          : ImVec4(0.18f, 0.22f, 0.28f, 0.60f));

        ImGui::BeginChild(child_id, ImVec2(cell_w, cell_h), true,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

        // 1. Workspace Activation Button
        char btn_label[48];
        if (is_active) {
            std::snprintf(btn_label, sizeof(btn_label), "%s  [ACTIVE]", ws_names[i]);
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.00f, 0.55f, 0.80f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.00f, 0.65f, 0.92f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.00f, 0.45f, 0.70f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(1.00f, 1.00f, 1.00f, 1.0f));
        } else {
            std::snprintf(btn_label, sizeof(btn_label), "%s", ws_names[i]);
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.15f, 0.18f, 0.24f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.28f, 0.36f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.12f, 0.14f, 0.18f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(0.75f, 0.82f, 0.90f, 1.0f));
        }

        const float btn_w = ImGui::GetContentRegionAvail().x;
        if (ImGui::Button(btn_label, ImVec2(btn_w, 22.0f))) {
            controller.selectWorkspace(static_cast<uint8_t>(i));
            if (on_changed) on_changed();
        }
        ImGui::PopStyleColor(4);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(is_active ? "%s is currently the active work coordinate system."
                                        : "Click to select %s as the active coordinate system.", ws_names[i]);
        }

        ImGui::Dummy(ImVec2(0.0f, 2.0f));

        // 2. Coordinates Display + [ Set ] Button
        const float set_btn_w = 38.0f;
        const float set_btn_h = 32.0f;

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(3.0f, 1.0f));
        ImGui::BeginGroup();
        // Line 1: X & Y
        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.45f, 1.0f), "X");
        ImGui::SameLine(0.0f, 2.0f);
        ImGui::Text("%+6.2f", offsets[0]);
        ImGui::SameLine(0.0f, 4.0f);
        ImGui::TextColored(ImVec4(0.40f, 0.85f, 0.45f, 1.0f), "Y");
        ImGui::SameLine(0.0f, 2.0f);
        ImGui::Text("%+6.2f", offsets[1]);

        // Line 2: Z & (optional A)
        ImGui::TextColored(ImVec4(0.40f, 0.70f, 1.0f, 1.0f), "Z");
        ImGui::SameLine(0.0f, 2.0f);
        ImGui::Text("%+6.2f", offsets[2]);
        if (show_4th_axis) {
            ImGui::SameLine(0.0f, 4.0f);
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f), "A");
            ImGui::SameLine(0.0f, 2.0f);
            ImGui::Text("%+5.1f", offsets[3]);
        }
        ImGui::EndGroup();
        ImGui::PopStyleVar();

        // Set Button on right side
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - set_btn_w);
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.18f, 0.28f, 0.38f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.38f, 0.52f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.12f, 0.20f, 0.28f, 1.0f));
        char set_id[32];
        std::snprintf(set_id, sizeof(set_id), "Set##WsSet_%d", i);
        if (ImGui::Button(set_id, ImVec2(set_btn_w, set_btn_h))) {
            edit_modal_ws_   = i;
            edit_temp_offset_ = offsets;
        }
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Set or zero %s coordinate offsets (custom values or current tool position)", ws_names[i]);
        }

        ImGui::EndChild();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(2);
    }

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
            ImGui::TextColored(ImVec4(0.0f, 0.80f, 1.0f, 1.0f), "SET COORDINATE OFFSETS: %s", ws_names[edit_modal_ws_]);
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
                ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.35f, 1.0f), "X Offset (mm):");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputDouble("##WsX", &edit_temp_offset_[0], 1.0, 10.0, "%.3f");

                // Y Axis
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(ImVec4(0.40f, 0.90f, 0.40f, 1.0f), "Y Offset (mm):");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputDouble("##WsY", &edit_temp_offset_[1], 1.0, 10.0, "%.3f");

                // Z Axis
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(ImVec4(0.40f, 0.70f, 1.0f, 1.0f), "Z Offset (mm):");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputDouble("##WsZ", &edit_temp_offset_[2], 1.0, 10.0, "%.3f");

                if (show_4th_axis) {
                    // A Axis
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextColored(ImVec4(0.95f, 0.80f, 0.35f, 1.0f), "A Offset (deg):");
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
            if (ImGui::Button("Clear All to 0.000", ImVec2(-1.0f, 26.0f))) {
                edit_temp_offset_ = {0.0, 0.0, 0.0, 0.0};
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Confirmation row
            const float btn_half_w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.55f, 0.35f, 1.0f));
            if (ImGui::Button("Save & Apply", ImVec2(btn_half_w, 32.0f))) {
                controller.setWorkspaceOffset(static_cast<uint8_t>(edit_modal_ws_), edit_temp_offset_);
                if (on_changed) on_changed();
                edit_modal_ws_ = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor();

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
