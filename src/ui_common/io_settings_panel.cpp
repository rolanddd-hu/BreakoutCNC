#include "ui_common/io_settings_panel.h"
#include "core/io/io_map.h"
#include "ui_common/theme.h"
#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

namespace rnr::ui {

namespace {

// Filled dot showing a pin's live state, vertically centered on the current frame-height row
void stateDot(bool on, const ImVec4& on_col) {
    const float h = ImGui::GetFrameHeight();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 c(p.x + 7.0f, p.y + h * 0.5f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 fill = on ? ImGui::ColorConvertFloat4ToU32(on_col)
                          : ImGui::ColorConvertFloat4ToU32(theme::pal().btn_neutral);
    dl->AddCircleFilled(c, 5.5f, fill, 16);
    dl->AddCircle(c, 5.5f, ImGui::ColorConvertFloat4ToU32(theme::pal().card_border), 16, 1.0f);
    ImGui::Dummy(ImVec2(14.0f, h));
}

// Text field for a custom name: edits a copy, stores it when editing ends (one config change per edit, not per key)
bool nameField(const char* id, const char* hint, std::string& name) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%s", name.c_str());
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputTextWithHint(id, hint, buf, sizeof(buf));
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        std::string trimmed = buf;
        trimmed.erase(0, trimmed.find_first_not_of(' '));
        trimmed.erase(trimmed.find_last_not_of(' ') + 1);
        if (trimmed != name) {
            name = trimmed;
            return true;
        }
    }
    return false;
}

void beginCard(const char* id) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, theme::pal().card_bg);
    ImGui::PushStyleColor(ImGuiCol_Border, theme::pal().card_border);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));
    ImGui::BeginChild(id, ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY,
                      ImGuiWindowFlags_NoScrollbar);
}

void endCard() {
    ImGui::EndChild();
    ImGui::PopStyleVar(4);
    ImGui::PopStyleColor(2);
}

} // namespace

bool renderIoSettings(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry,
                      core::MachineConfig& cfg) {
    bool changed = false;
    const theme::Palette& pal = theme::pal();
    const auto state = telemetry.state;
    // Re-assigning outputs switches them all off, so only while nothing runs
    const bool idle = (state == core::ControllerState::Idle || state == core::ControllerState::EStop ||
                       state == core::ControllerState::Disconnected) &&
                      !telemetry.script_running;
    const bool can_switch = idle && state != core::ControllerState::EStop &&
                            state != core::ControllerState::Disconnected;
    const auto mode = cfg.operation_mode;

    // ------------------------------------------------------------------ Outputs
    beginCard("##IoOutputsCard");
    {
        ImGui::TextColored(pal.heading, "OUTPUTS");
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Choose what each output does. An output without a machine function is a custom output: "
                            "name it and switch it from the I/O strip, a macro or Lua (cnc.set_output).");
        ImGui::PopTextWrapPos();
        if (!idle) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(pal.text_warn, "Output assignment can be changed only while the machine is idle.");
            ImGui::PopTextWrapPos();
        }
        ImGui::Spacing();

        const float test_w = ImGui::CalcTextSize("Turn off").x + 32.0f;
        if (ImGui::BeginTable("##IoOutTable", 4, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX)) {
            ImGui::TableSetupColumn("##State", ImGuiTableColumnFlags_WidthFixed, 14.0f);
            ImGui::TableSetupColumn("##Pin", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("OUT4").x);
            ImGui::TableSetupColumn("##Function", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("##Test", ImGuiTableColumnFlags_WidthFixed, test_w);

            for (int pin = 1; pin <= core::kPhysicalOutputCount; ++pin) {
                ImGui::PushID(pin);
                ImGui::TableNextRow();
                const bool on = (telemetry.physical_outputs_bitmask >> (pin - 1)) & 1u;
                const int channel = core::io::channelForPin(cfg.io, pin);

                ImGui::TableNextColumn();
                stateDot(on, pal.btn_warn);

                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("OUT%d", pin);

                ImGui::TableNextColumn();
                const char* current = channel >= 0 ? core::io::channelName(channel, cfg.is_mill, mode) : "Custom output";
                ImGui::BeginDisabled(!idle);
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::BeginCombo("##Function", current)) {
                    for (int ch = 0; ch < core::kOutputChannelCount; ++ch) {
                        const int old_pin = cfg.io.output_pin[ch];
                        char item[96];
                        if (old_pin != 0 && old_pin != pin) {
                            std::snprintf(item, sizeof(item), "%s  (now OUT%d)",
                                          core::io::channelName(ch, cfg.is_mill, mode), old_pin);
                        } else {
                            std::snprintf(item, sizeof(item), "%s", core::io::channelName(ch, cfg.is_mill, mode));
                        }
                        // Swapping with an unconnected function would disconnect the enable output
                        const bool blocked = channel == 0 && old_pin == 0;
                        ImGui::BeginDisabled(blocked);
                        if (ImGui::Selectable(item, ch == channel) && ch != channel) {
                            // Swap: the function moves here, and what was here moves to the function's old pin
                            if (channel >= 0) cfg.io.output_pin[channel] = static_cast<uint8_t>(old_pin);
                            cfg.io.output_pin[ch] = static_cast<uint8_t>(pin);
                            changed = true;
                        }
                        ImGui::EndDisabled();
                    }
                    // The enable output (spindle / torch / laser) must stay connected
                    const bool locked = channel == 0;
                    ImGui::BeginDisabled(locked);
                    if (ImGui::Selectable("Custom output", channel < 0) && channel > 0) {
                        cfg.io.output_pin[channel] = 0;
                        changed = true;
                    }
                    ImGui::EndDisabled();
                    if (locked && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                        ImGui::SetTooltip("Move %s to another output first", core::io::channelShortName(0, cfg.is_mill, mode));
                    }
                    ImGui::EndCombo();
                }
                ImGui::EndDisabled();
                if (channel < 0) {
                    changed |= nameField("##OutName", "Name, e.g. Vacuum or Clamp", cfg.io.output_name[pin - 1]);
                }

                ImGui::TableNextColumn();
                ImGui::BeginDisabled(!can_switch);
                if (ImGui::Button(on ? "Turn off" : "Turn on", ImVec2(test_w, 0.0f))) {
                    controller.setPhysicalOutput(static_cast<uint8_t>(pin), !on);
                }
                ImGui::EndDisabled();
                if (!can_switch && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                    ImGui::SetTooltip("Available when the machine is idle and not in E-stop");
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }

        // Functions that are not connected to any output
        std::string missing;
        for (int ch = 1; ch < core::kOutputChannelCount; ++ch) {
            if (cfg.io.output_pin[ch] != 0) continue;
            if (!cfg.is_mill && mode != core::OperationMode::Laser && ch == 1) continue; // Unused in plasma mode
            if (!missing.empty()) missing += ", ";
            missing += core::io::channelShortName(ch, cfg.is_mill, mode);
        }
        if (!missing.empty()) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(pal.text_muted, "Not connected (ignored): %s", missing.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("All outputs switch off when the assignment changes. Outputs cannot be inverted: the board "
                            "turns every output off on an E-stop, which would switch an inverted output on.");
        ImGui::PopTextWrapPos();
    }
    endCard();

    ImGui::Spacing();

    // ------------------------------------------------------------------ Inputs
    beginCard("##IoInputsCard");
    {
        ImGui::TextColored(pal.heading, "INPUTS");
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Machine functions are set in their own sections (end-stops, probe, THC, laser interlocks). "
                            "An unused input can be named as a custom input; Lua reads it with "
                            "cnc.get_input_by_name(\"name\").");
        ImGui::PopTextWrapPos();
        ImGui::Spacing();

        const auto uses = core::io::describeInputs(cfg);
        const float invert_w = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x +
                               ImGui::CalcTextSize("Invert").x + 4.0f;
        if (ImGui::BeginTable("##IoInTable", 4, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX)) {
            ImGui::TableSetupColumn("##State", ImGuiTableColumnFlags_WidthFixed, 14.0f);
            ImGui::TableSetupColumn("##Pin", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("IN12").x);
            ImGui::TableSetupColumn("##Use", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("##Invert", ImGuiTableColumnFlags_WidthFixed, invert_w);

            for (int i = 0; i < core::kInputCount; ++i) {
                ImGui::PushID(100 + i);
                ImGui::TableNextRow();
                const bool raw = (telemetry.inputs_bitmask >> i) & 1u;
                const bool custom = uses[i].empty();

                ImGui::TableNextColumn();
                stateDot(custom ? (raw != cfg.io.input_invert[i]) : raw, pal.btn_ok);

                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("IN%d", i + 1);

                ImGui::TableNextColumn();
                if (custom) {
                    changed |= nameField("##InName", "Unused - name it as a custom input", cfg.io.input_name[i]);
                } else {
                    std::string list;
                    for (const auto& u : uses[i]) {
                        if (!list.empty()) list += ", ";
                        list += u.label;
                    }
                    ImGui::AlignTextToFramePadding();
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextUnformatted(list.c_str());
                    ImGui::PopTextWrapPos();
                    if (ImGui::IsItemHovered()) {
                        std::string tip;
                        for (const auto& u : uses[i]) {
                            tip += u.label + (u.invert ? " (active low)" : " (active high)") + "\n  set in: " +
                                   u.section + "\n";
                        }
                        ImGui::SetTooltip("%s", tip.c_str());
                    }
                }

                ImGui::TableNextColumn();
                if (custom) {
                    bool inv = cfg.io.input_invert[i];
                    if (ImGui::Checkbox("Invert", &inv)) {
                        cfg.io.input_invert[i] = inv;
                        changed = true;
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Active when the input is off (normally closed switch)");
                    }
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }
    endCard();

    return changed;
}

} // namespace rnr::ui
