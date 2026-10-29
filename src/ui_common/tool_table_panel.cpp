#include "ui_common/tool_table_panel.h"
#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace rnr::ui {

void ToolTablePanel::render(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    const std::function<void()>& on_changed)
{
    core::MachineConfig cfg = controller.getConfig();
    bool changed = false;
    const float spacing = ImGui::GetStyle().ItemSpacing.x;

    const bool imperial = (cfg.linear_units == core::LinearUnits::Imperial);
    const double to_disp   = imperial ? (1.0 / 25.4) : 1.0;
    const double from_disp = imperial ? 25.4 : 1.0;
    const char*  unit      = imperial ? "in" : "mm";

    const auto s = telemetry.state;
    const bool job_active = (s == core::ControllerState::RunningGCode || s == core::ControllerState::DwellPierce ||
                             s == core::ControllerState::PausedHold || s == core::ControllerState::Homing ||
                             s == core::ControllerState::Probing || s == core::ControllerState::Jogging);
    const bool can_move = (s == core::ControllerState::Idle || s == core::ControllerState::ToolChangeHold);

    // ---- Tool length mode ----------------------------------------------------------------------
    // Short option names (details are in the wrapped help text below) on a full-width combo, with the
    // label on its own line above it, so the selected option is never cut off
    const char* modes[] = {
        "Not configured (warn on G43)",
        "Touch off Z after every tool change",
        "Tool table (apply G43 H<n>)"
    };
    int mode = static_cast<int>(cfg.tool_table.mode);
    ImGui::BeginDisabled(job_active);
    ImGui::TextUnformatted("Tool Length Mode");
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::Combo("##ToolLengthMode", &mode, modes, 3)) {
        cfg.tool_table.mode = static_cast<core::ToolLengthMode>(mode);
        changed = true;
    }
    ImGui::EndDisabled();

    ImGui::PushTextWrapPos(0.0f);
    switch (cfg.tool_table.mode) {
        case core::ToolLengthMode::NotConfigured:
            ImGui::TextDisabled("G43 tool length offsets in programs are ignored and the program shows a warning. "
                                "Choose one of the other modes to state how this machine handles tool lengths.");
            break;
        case core::ToolLengthMode::TouchOffEachTool:
            ImGui::TextDisabled("G43 is ignored without a warning. After every tool change (M6) Z is touched off "
                                "with the touch plate / tool setter, which sets the work Z for the new tool.");
            break;
        case core::ToolLengthMode::ToolTable:
            ImGui::TextDisabled("Each tool's length is measured once on the tool setter, relative to the reference "
                                "tool. G43 H<n> in the program applies it, so tool changes need no touch-off. "
                                "Every M6 in a program must be followed by G43 H<tool>.");
            break;
    }
    ImGui::PopTextWrapPos();

    if (cfg.tool_table.mode != core::ToolLengthMode::ToolTable) {
        if (changed) {
            controller.applyConfig(cfg);
            if (on_changed) on_changed();
        }
        return;
    }

    ImGui::Spacing();

    // ---- Status card ---------------------------------------------------------------------------
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.07f, 0.12f, 0.16f, 0.90f));
    ImGui::BeginChild("##ToolTableStatus", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
    ImGui::PushTextWrapPos(0.0f);
    if (telemetry.active_tlo_tool > 0) {
        ImGui::TextColored(ImVec4(0.0f, 0.88f, 1.0f, 1.0f), "Tool in spindle: T%u  |  Active offset: G43 H%u = %+.4f %s",
                           static_cast<unsigned>(telemetry.current_tool_number),
                           static_cast<unsigned>(telemetry.active_tlo_tool),
                           telemetry.active_tlo_mm * to_disp, unit);
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.15f, 1.0f), "Tool in spindle: T%u  |  No tool length offset active (G49)",
                           static_cast<unsigned>(telemetry.current_tool_number));
    }
    if (telemetry.tool_reference_measured) {
        ImGui::TextColored(ImVec4(0.30f, 0.90f, 0.45f, 1.0f), "Reference tool T%u measured this session.",
                           static_cast<unsigned>(cfg.tool_table.reference_tool));
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.62f, 0.25f, 1.0f),
                           "Reference tool T%u not measured since startup / Z homing: measure it first before "
                           "measuring other tools.", static_cast<unsigned>(cfg.tool_table.reference_tool));
    }
    ImGui::PopTextWrapPos();
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // ---- Tool in spindle & reference tool -------------------------------------------------------
    {
        const float avail = ImGui::GetContentRegionAvail().x;
        const char* lbl = "Tool in Spindle";
        const float lbl_w = ImGui::CalcTextSize(lbl).x;
        const float load_w = ImGui::CalcTextSize("Load Tool (G43)").x + 32.0f;
        const float g49_w  = ImGui::CalcTextSize("Cancel Offset (G49)").x + 32.0f;
        if (spindle_tool_ <= 0) spindle_tool_ = std::max<int>(1, telemetry.current_tool_number);
        ImGui::BeginDisabled(!can_move && s != core::ControllerState::EStop);
        ImGui::SetNextItemWidth(std::max(70.0f, avail - lbl_w - load_w - g49_w - spacing * 4.0f));
        ImGui::InputInt(lbl, &spindle_tool_);
        spindle_tool_ = std::clamp(spindle_tool_, 1, 255);
        ImGui::SameLine();
        if (ImGui::Button("Load Tool (G43)", ImVec2(load_w, 28.0f))) {
            controller.selectToolInSpindle(static_cast<uint16_t>(spindle_tool_));
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("The selected tool is in the spindle: apply its length offset (like M6 T%d G43 H%d)",
                              spindle_tool_, spindle_tool_);
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel Offset (G49)", ImVec2(g49_w, 28.0f))) {
            controller.activateToolLengthOffset(0);
        }
        ImGui::EndDisabled();

        int ref = cfg.tool_table.reference_tool;
        const float ref_lbl_w = ImGui::CalcTextSize("Reference Tool").x;
        ImGui::BeginDisabled(job_active);
        ImGui::SetNextItemWidth(std::max(70.0f, ImGui::GetContentRegionAvail().x - ref_lbl_w - spacing));
        if (ImGui::InputInt("Reference Tool", &ref)) {
            cfg.tool_table.reference_tool = static_cast<uint16_t>(std::clamp(ref, 1, 255));
            changed = true;
        }
        ImGui::EndDisabled();
    }

    ImGui::Spacing();

    // ---- Tool table ----------------------------------------------------------------------------
    std::vector<uint16_t> to_delete;
    const ImGuiTableFlags tflags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
    // Column widths: Length / Dia get the share they would have as stretch columns (weights 1.2 / 0.9 /
    // 2.0 with Description), then the pixels saved by the narrower Tool column are moved to Dia.
    // Description keeps stretching, so it ends up exactly as wide as before.
    const float table_w   = ImGui::GetContentRegionAvail().x;
    const float cell_pad  = ImGui::GetStyle().CellPadding.x * 2.0f;
    const float actions_w = ImGui::CalcTextSize("Measure").x + ImGui::CalcTextSize("Delete").x + 64.0f + spacing;
    const float tool_w_old = ImGui::CalcTextSize("T255 *").x + 8.0f;
    const float tool_w     = std::max(ImGui::CalcTextSize("Tool").x, ImGui::CalcTextSize("T99*").x) + 2.0f;
    const float stretch_w  = std::max(60.0f, table_w - tool_w_old - actions_w - 5.0f * cell_pad - 6.0f);
    const float len_w      = stretch_w * (1.2f / 4.1f);
    const float dia_w      = stretch_w * (0.9f / 4.1f) + (tool_w_old - tool_w);
    if (ImGui::BeginTable("##ToolTable", 5, tflags)) {
        char len_hdr[32], dia_hdr[32];
        std::snprintf(len_hdr, sizeof(len_hdr), "Length (%s)", unit);
        std::snprintf(dia_hdr, sizeof(dia_hdr), "Dia (%s)", unit);
        ImGui::TableSetupColumn("Tool", ImGuiTableColumnFlags_WidthFixed, tool_w);
        ImGui::TableSetupColumn(len_hdr, ImGuiTableColumnFlags_WidthFixed, len_w);
        ImGui::TableSetupColumn(dia_hdr, ImGuiTableColumnFlags_WidthFixed, dia_w);
        ImGui::TableSetupColumn("Description", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("##Actions", ImGuiTableColumnFlags_WidthFixed, actions_w);
        ImGui::TableHeadersRow();

        for (auto& [num, entry] : cfg.tool_table.tools) {
            ImGui::PushID(static_cast<int>(num));
            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            const bool is_ref = (num == cfg.tool_table.reference_tool);
            if (num == telemetry.active_tlo_tool) {
                ImGui::TextColored(ImVec4(0.0f, 0.88f, 1.0f, 1.0f), "T%u%s", static_cast<unsigned>(num), is_ref ? "*" : "");
            } else {
                ImGui::Text("T%u%s", static_cast<unsigned>(num), is_ref ? "*" : "");
            }
            if (is_ref && ImGui::IsItemHovered()) ImGui::SetTooltip("Reference tool (length 0)");

            ImGui::BeginDisabled(job_active);
            ImGui::TableSetColumnIndex(1);
            double len = entry.length_mm * to_disp;
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputDouble("##len", &len, 0.0, 0.0, imperial ? "%.5f" : "%.4f")) {
                entry.length_mm = len * from_disp;
                changed = true;
            }

            ImGui::TableSetColumnIndex(2);
            double dia = entry.diameter_mm * to_disp;
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputDouble("##dia", &dia, 0.0, 0.0, imperial ? "%.4f" : "%.3f")) {
                entry.diameter_mm = std::max(0.0, dia * from_disp);
                changed = true;
            }

            ImGui::TableSetColumnIndex(3);
            char desc[128];
            std::snprintf(desc, sizeof(desc), "%s", entry.description.c_str());
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputText("##desc", desc, sizeof(desc))) {
                entry.description = desc;
                changed = true;
            }
            ImGui::EndDisabled();

            ImGui::TableSetColumnIndex(4);
            const bool can_measure = can_move && (is_ref || telemetry.tool_reference_measured);
            ImGui::BeginDisabled(!can_measure);
            if (ImGui::Button("Measure", ImVec2(0.0f, 28.0f))) {
                controller.measureToolLength(num);
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip(can_measure ? "Probe this tool on the tool setter below the current XY position"
                                              : "Measure the reference tool first (machine must be idle)");
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(job_active || num == telemetry.active_tlo_tool);
            if (ImGui::Button("Delete", ImVec2(0.0f, 28.0f))) {
                to_delete.push_back(num);
            }
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    for (uint16_t num : to_delete) {
        cfg.tool_table.tools.erase(num);
        changed = true;
    }

    // ---- Add tool ------------------------------------------------------------------------------
    {
        const float add_w = ImGui::CalcTextSize("Add Tool").x + 32.0f;
        const float lbl_w = ImGui::CalcTextSize("New Tool #").x;
        ImGui::BeginDisabled(job_active);
        ImGui::SetNextItemWidth(std::max(70.0f, ImGui::GetContentRegionAvail().x - lbl_w - add_w - spacing * 2.0f));
        ImGui::InputInt("New Tool #", &new_tool_number_);
        new_tool_number_ = std::clamp(new_tool_number_, 1, 255);
        ImGui::SameLine();
        const bool exists = cfg.tool_table.tools.count(static_cast<uint16_t>(new_tool_number_)) != 0;
        ImGui::BeginDisabled(exists);
        if (ImGui::Button("Add Tool", ImVec2(add_w, 28.0f))) {
            cfg.tool_table.tools[static_cast<uint16_t>(new_tool_number_)] = core::ToolTableEntry{};
            changed = true;
        }
        ImGui::EndDisabled();
        ImGui::EndDisabled();
    }

    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("Measuring: jog the tool over the tool setter, then press Measure. Start with the reference tool "
                        "(*) after every startup or Z homing. Then touch off work Z once with any measured tool loaded "
                        "(Load Tool), and every other tool will cut at the same depth.");
    ImGui::PopTextWrapPos();

    if (changed) {
        controller.applyConfig(cfg);
        if (on_changed) on_changed();
    }
}

} // namespace rnr::ui
