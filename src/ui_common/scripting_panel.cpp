#include "scripting_panel.h"
#include "ui_common/app_window.h"
#include "ui_common/file_dialog.h"
#include "ui_common/theme.h"
#include "core/scripting/script_path_helper.h"
#include <imgui.h>
#include <imgui_internal.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace rnr::ui {

namespace {

std::string getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#ifdef _WIN32
    localtime_s(&tm_buf, &tt);
#else
    localtime_r(&tt, &tm_buf);
#endif
    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%H:%M:%S");
    return ss.str();
}

} // namespace

ScriptingPanel::ScriptingPanel() {
    appendLog("Lua 5.4.7 Scripting Engine initialized.", false);
}

ScriptingPanel::~ScriptingPanel() = default;

void ScriptingPanel::appendLog(const std::string& message, bool is_error) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    LogEntry entry;
    entry.timestamp = getCurrentTimeString();
    entry.text      = message;
    entry.is_error  = is_error;

    logs_.push_back(std::move(entry));
    if (logs_.size() > max_logs_) {
        logs_.erase(logs_.begin(), logs_.begin() + (logs_.size() - max_logs_));
    }
}

void ScriptingPanel::clearLogs() {
    std::lock_guard<std::mutex> lock(log_mutex_);
    logs_.clear();
}

void ScriptingPanel::render(
    core::IMotionController& controller,
    core::MachineConfig& cfg,
    const std::function<void()>& on_config_changed)
{
    auto telem = controller.getTelemetry();
    const bool is_running = telem.script_running;

    // Script Status Banner
    if (is_running) {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, theme::pal().surface_warn);
        ImGui::BeginChild("##ScriptRunningBanner", ImVec2(0.0f, 42.0f), true, ImGuiWindowFlags_NoScrollbar);
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(theme::pal().text_warn, "SCRIPT RUNNING: %s", telem.active_script_name.c_str());
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 110.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_danger);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_danger));
        if (ImGui::Button("ABORT SCRIPT", ImVec2(100.0f, 26.0f))) {
            controller.cancelScript();
        }
        ImGui::PopStyleColor(2);
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }

    // Dynamic Quick Macro Buttons Deck (+ Add Button & Right-click Remove/Edit)
    if (!is_running) {
        ImGui::Dummy(ImVec2(0.0f, 2.0f));
    }

    auto& sc = cfg.scripting;
    const float avail_w = ImGui::GetContentRegionAvail().x;
    const float item_spacing_x = ImGui::GetStyle().ItemSpacing.x;
    const float plus_btn_w = 34.0f;
    const float btn_h = 32.0f;

    const size_t count = sc.macro_buttons.size();
    std::vector<size_t> visible_indices;
    visible_indices.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        if (!sc.macro_buttons[i].show_on_control_panel) {
            visible_indices.push_back(i);
        }
    }
    const size_t vis_count = visible_indices.size();

    float btn_w = 95.0f;
    if (vis_count > 0 && vis_count <= 5) {
        float fit_w = (avail_w - plus_btn_w - static_cast<float>(vis_count + 1) * item_spacing_x) / static_cast<float>(vis_count);
        if (fit_w >= 80.0f && fit_w <= 160.0f) {
            btn_w = fit_w;
        }
    }

    int button_to_remove = -1;
    int button_to_add_to_cp = -1;
    bool color_changed = false;

    color_menu_.beginFrame();
    for (size_t k = 0; k < vis_count; ++k) {
        size_t i = visible_indices[k];
        if (k > 0) {
            float next_x = ImGui::GetItemRectMax().x + item_spacing_x + btn_w;
            if (next_x < ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x) {
                ImGui::SameLine();
            }
        }

        ImGui::PushID(static_cast<int>(i));
        const auto& btn_cfg = sc.macro_buttons[i];
        std::string resolved = scripting::resolveScriptPath(btn_cfg.path);
        bool has_target = !resolved.empty();

        if (!has_target) {
            ImGui::BeginDisabled();
        }

        std::string btn_text = btn_cfg.label.empty() ? ("Macro " + std::to_string(i + 1)) : btn_cfg.label;
        if (macroButton(btn_text.c_str(), btn_cfg, ImVec2(btn_w, btn_h), nullptr, color_menu_.previewFor(i))) {
            appendLog("Executing macro: " + resolved, false);
            controller.runScriptFile(resolved, true);
        }

        if (ImGui::IsItemHovered()) {
            if (has_target) {
                ImGui::SetTooltip("File: %s\n(Right-click for options)", resolved.c_str());
            } else {
                ImGui::SetTooltip("File not found: %s\n(Right-click for options)", btn_cfg.path.empty() ? "(none)" : btn_cfg.path.c_str());
            }
        }

        if (!has_target) {
            ImGui::EndDisabled();
        }

        // Right-click context popup to Edit, Remove or Add to control panel
        if (ImGui::BeginPopupContextItem("##MacroBtnCtx")) {
            if (ImGui::MenuItem("Edit Button...")) {
                edit_btn_index_ = static_cast<int>(i);
                std::strncpy(modal_label_, btn_cfg.label.c_str(), sizeof(modal_label_) - 1);
                modal_label_[sizeof(modal_label_) - 1] = '\0';
                std::strncpy(modal_path_, btn_cfg.path.c_str(), sizeof(modal_path_) - 1);
                modal_path_[sizeof(modal_path_) - 1] = '\0';
                modal_has_color_ = btn_cfg.has_color;
                const ImVec4 c = btn_cfg.has_color ? theme::fromRgb(btn_cfg.color_rgb) : theme::pal().macro_btn;
                modal_color_[0] = c.x;
                modal_color_[1] = c.y;
                modal_color_[2] = c.z;
                show_edit_modal_ = true;
            }
            color_changed |= color_menu_.menu(i, sc.macro_buttons[i]);
            if (ImGui::MenuItem("Remove Button")) {
                button_to_remove = static_cast<int>(i);
            }
            if (ImGui::MenuItem("Add to control panel")) {
                button_to_add_to_cp = static_cast<int>(i);
            }
            ImGui::EndPopup();
        }

        ImGui::PopID();
    }
    color_menu_.endFrame();

    if (color_changed) {
        controller.applyConfig(cfg);
        if (on_config_changed) on_config_changed();
    }

    if (button_to_add_to_cp >= 0 && button_to_add_to_cp < static_cast<int>(sc.macro_buttons.size())) {
        sc.macro_buttons[button_to_add_to_cp].show_on_control_panel = true;
        controller.applyConfig(cfg);
        if (on_config_changed) on_config_changed();
    }

    if (button_to_remove >= 0 && button_to_remove < static_cast<int>(sc.macro_buttons.size())) {
        sc.macro_buttons.erase(sc.macro_buttons.begin() + button_to_remove);
        controller.applyConfig(cfg);
        if (on_config_changed) on_config_changed();
    }

    // '+' Add Macro Button
    if (vis_count > 0) {
        float next_x = ImGui::GetItemRectMax().x + item_spacing_x + plus_btn_w;
        if (next_x < ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x) {
            ImGui::SameLine();
        }
    }
    ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_neutral);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_neutral_hov);
    if (ImGui::Button("+", ImVec2(plus_btn_w, btn_h))) {
        modal_label_[0] = '\0';
        modal_path_[0]  = '\0';
        modal_has_color_ = false;
        const ImVec4 c = theme::pal().macro_btn;
        modal_color_[0] = c.x;
        modal_color_[1] = c.y;
        modal_color_[2] = c.z;
        show_add_modal_ = true;
    }
    ImGui::PopStyleColor(2);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Add New Custom Macro Button...");
    }

    // Handle opening modals
    if (show_add_modal_) {
        ImGui::OpenPopup("Add Custom Macro Button##AddModal");
        show_add_modal_ = false;
    }
    if (show_edit_modal_) {
        ImGui::OpenPopup("Edit Macro Button##EditModal");
        show_edit_modal_ = false;
    }

    // Modal: Add Custom Macro Button
    const float kMacroModalW = 480.0f;
    ImVec2 modal_center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(modal_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(kMacroModalW, 0.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(kMacroModalW, -1.0f), ImVec2(kMacroModalW, -1.0f));
    if (ImGui::BeginPopupModal("Add Custom Macro Button##AddModal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Spacing();
        ImGui::TextColored(theme::pal().heading, "ADD CUSTOM MACRO BUTTON");
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Button Label / Name:");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputTextWithHint("##NewMacroLabel", "e.g. Probe Plate, Park Spindle, Tool 2...",
                                modal_label_, sizeof(modal_label_));

        ImGui::Spacing();
        ImGui::Text("Lua Script File Path:");
        const float browse_btn_w = 85.0f;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - browse_btn_w - ImGui::GetStyle().ItemSpacing.x);
        ImGui::InputTextWithHint("##NewMacroPath", "e.g. scripts/mill/probe_z.lua",
                                modal_path_, sizeof(modal_path_));
        ImGui::SameLine();
        if (ImGui::Button("Browse...", ImVec2(browse_btn_w, 0.0f))) {
            std::string sel = openLuaScriptDialog();
            if (!sel.empty()) {
                std::strncpy(modal_path_, sel.c_str(), sizeof(modal_path_) - 1);
                modal_path_[sizeof(modal_path_) - 1] = '\0';
                if (modal_label_[0] == '\0') {
                    std::filesystem::path p(sel);
                    std::string stem = p.stem().string();
                    std::strncpy(modal_label_, stem.c_str(), sizeof(modal_label_) - 1);
                    modal_label_[sizeof(modal_label_) - 1] = '\0';
                }
            }
        }

        renderModalColorRow();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        bool can_add = (modal_label_[0] != '\0' && modal_path_[0] != '\0');
        if (!can_add) ImGui::BeginDisabled();
        ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_ok);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));
        if (ImGui::Button("ADD BUTTON", ImVec2(130.0f, 32.0f))) {
            core::MacroButtonConfig added{modal_label_, modal_path_};
            added.has_color = modal_has_color_;
            added.color_rgb = modal_has_color_ ? theme::toRgb(ImVec4(modal_color_[0], modal_color_[1], modal_color_[2], 1.0f)) : 0;
            sc.macro_buttons.push_back(added);
            controller.applyConfig(cfg);
            if (on_config_changed) on_config_changed();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(2);
        if (!can_add) ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(90.0f, 32.0f))) {
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }

    // Modal: Edit Macro Button
    ImGui::SetNextWindowPos(modal_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(kMacroModalW, 0.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(kMacroModalW, -1.0f), ImVec2(kMacroModalW, -1.0f));
    if (ImGui::BeginPopupModal("Edit Macro Button##EditModal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Spacing();
        ImGui::TextColored(theme::pal().heading, "EDIT MACRO BUTTON");
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Button Label / Name:");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputText("##EditMacroLabel", modal_label_, sizeof(modal_label_));

        ImGui::Spacing();
        ImGui::Text("Lua Script File Path:");
        const float browse_btn_w = 85.0f;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - browse_btn_w - ImGui::GetStyle().ItemSpacing.x);
        ImGui::InputText("##EditMacroPath", modal_path_, sizeof(modal_path_));
        ImGui::SameLine();
        if (ImGui::Button("Browse...", ImVec2(browse_btn_w, 0.0f))) {
            std::string sel = openLuaScriptDialog();
            if (!sel.empty()) {
                std::strncpy(modal_path_, sel.c_str(), sizeof(modal_path_) - 1);
                modal_path_[sizeof(modal_path_) - 1] = '\0';
            }
        }

        renderModalColorRow();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        bool can_save = (modal_label_[0] != '\0' && modal_path_[0] != '\0' &&
                         edit_btn_index_ >= 0 && edit_btn_index_ < static_cast<int>(sc.macro_buttons.size()));
        if (!can_save) ImGui::BeginDisabled();
        ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_ok);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));
        if (ImGui::Button("SAVE", ImVec2(100.0f, 32.0f))) {
            sc.macro_buttons[edit_btn_index_].label = modal_label_;
            sc.macro_buttons[edit_btn_index_].path  = modal_path_;
            sc.macro_buttons[edit_btn_index_].has_color = modal_has_color_;
            sc.macro_buttons[edit_btn_index_].color_rgb =
                modal_has_color_ ? theme::toRgb(ImVec4(modal_color_[0], modal_color_[1], modal_color_[2], 1.0f)) : 0;
            controller.applyConfig(cfg);
            if (on_config_changed) on_config_changed();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(2);
        if (!can_save) ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_danger);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_danger));
        if (ImGui::Button("REMOVE BUTTON", ImVec2(130.0f, 32.0f))) {
            if (edit_btn_index_ >= 0 && edit_btn_index_ < static_cast<int>(sc.macro_buttons.size())) {
                sc.macro_buttons.erase(sc.macro_buttons.begin() + edit_btn_index_);
                controller.applyConfig(cfg);
                if (on_config_changed) on_config_changed();
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(90.0f, 32.0f))) {
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }

    ImGui::Spacing();

    // Run Custom Script File Browser Bar
    if (ImGui::Button("Browse & Run Script (.lua)...", ImVec2(avail_w - 110.0f, 28.0f))) {
        std::string selected = openLuaScriptDialog();
        if (!selected.empty()) {
            appendLog("Running selected script: " + selected, false);
            controller.runScriptFile(selected, true);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Log", ImVec2(100.0f, 28.0f))) {
        clearLogs();
    }

    ImGui::Spacing();

    // Interactive Lua REPL Input Bar
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(theme::pal().heading, "Lua MDI:");
    ImGui::SameLine();

    const float exec_btn_w = 80.0f;
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - exec_btn_w - 8.0f);
    bool execute_repl = false;
    if (ImGui::InputTextWithHint("##LuaReplInput", "Enter Lua command (e.g. cnc.zero_axis('Z') or print(1+2))...",
        repl_input_, sizeof(repl_input_), ImGuiInputTextFlags_EnterReturnsTrue))
    {
        execute_repl = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Run", ImVec2(exec_btn_w, 0.0f))) {
        execute_repl = true;
    }

    if (execute_repl && repl_input_[0] != '\0') {
        std::string cmd = repl_input_;
        appendLog("> " + cmd, false);
        controller.runScriptString(cmd, true);
        repl_input_[0] = '\0';
    }

    ImGui::Spacing();

    // Scrollable Script Output Console
    const theme::Palette& pal = theme::pal();
    ImGui::PushStyleColor(ImGuiCol_ChildBg, pal.console_bg);
    ImGui::BeginChild("##ScriptConsoleChild", ImVec2(0.0f, 130.0f), true, ImGuiWindowFlags_HorizontalScrollbar);

    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        for (const auto& entry : logs_) {
            ImGui::TextColored(pal.console_time, "[%s]", entry.timestamp.c_str());
            ImGui::SameLine();
            if (entry.is_error) {
                ImGui::TextColored(pal.console_error, "%s", entry.text.c_str());
            } else {
                ImGui::TextColored(pal.console_ok, "%s", entry.text.c_str());
            }
        }
    }

    if (auto_scroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20.0f) {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
    ImGui::PopStyleColor();
}

void ScriptingPanel::renderModalColorRow() {
    ImGui::Spacing();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Button Color:");
    ImGui::SameLine();
    ImGui::Checkbox("Custom##MacroColorCustom", &modal_has_color_);
    if (modal_has_color_) {
        ImGui::SameLine();
        ImGui::ColorEdit3("##MacroColorPick", modal_color_,
                          ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel |
                              ImGuiColorEditFlags_PickerHueWheel);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Click to pick the button color");
    }

    // Preview of the button as it will look in the macro row (not clickable)
    ImGui::SameLine();
    ImGui::TextDisabled("Preview:");
    ImGui::SameLine();
    core::MacroButtonConfig preview;
    preview.has_color = modal_has_color_;
    preview.color_rgb = theme::toRgb(ImVec4(modal_color_[0], modal_color_[1], modal_color_[2], 1.0f));
    const char* text = modal_label_[0] != '\0' ? modal_label_ : "Macro";
    ImGui::BeginDisabled();
    ImGui::PushStyleVar(ImGuiStyleVar_DisabledAlpha, 1.0f);
    macroButton(text, preview, ImVec2(std::min(160.0f, ImGui::CalcTextSize(text).x + 32.0f), 32.0f));
    ImGui::PopStyleVar();
    ImGui::EndDisabled();
}

void ScriptingPanel::renderPromptModal(core::IMotionController& controller) {
    // While the guide window is open, script dialogs are drawn inside it (see renderHelpModal) so a
    // "Run demo" dialog stacks on top of the guide instead of replacing it
    if (guide_open_) return;
    renderPromptModalImpl(controller);
}

void ScriptingPanel::renderPromptModalImpl(core::IMotionController& controller) {
    auto telem = controller.getTelemetry();
    ImGuiContext& g = *ImGui::GetCurrentContext();
    if (!telem.has_script_prompt) {
        // The script went away (answered, cancelled or E-STOP): close a dialog that is still open
        if (prompt_was_active_ && g.OpenPopupStack.Size > prompt_popup_level_) {
            ImGui::ClosePopupToLevel(prompt_popup_level_, true);
        }
        prompt_was_active_ = false;
        return;
    }

    std::string modal_title = telem.script_prompt_title.empty()
                                  ? "Script Operator Confirmation"
                                  : telem.script_prompt_title;
    modal_title += "##ScriptPromptModal";

    const bool has_input = (telem.script_prompt_input_kind != 0);
    const bool numeric   = (telem.script_prompt_input_kind == 2);
    bool focus_input = false;
    if (!prompt_was_active_ || !ImGui::IsPopupOpen(modal_title.c_str())) {
        prompt_popup_level_ = g.BeginPopupStack.Size;
        ImGui::OpenPopup(modal_title.c_str());
        // New dialog: start from the script's default value
        std::snprintf(prompt_input_buf_, sizeof(prompt_input_buf_), "%s", telem.script_prompt_input_default.c_str());
        focus_input = has_input;
    }
    prompt_was_active_ = true;

    const auto& buttons = telem.script_prompt_buttons.empty()
                              ? std::vector<std::string>{"OK", "Cancel"}
                              : telem.script_prompt_buttons;

    const float kBtnH = 36.0f;
    const float spacing = 12.0f;
    std::vector<float> btn_widths(buttons.size());
    float total_w = 0.0f;
    for (size_t i = 0; i < buttons.size(); ++i) {
        float text_w = ImGui::CalcTextSize(buttons[i].c_str()).x;
        btn_widths[i] = std::max(110.0f, text_w + 32.0f);
        total_w += btn_widths[i];
    }
    if (!buttons.empty()) {
        total_w += static_cast<float>(buttons.size() - 1) * spacing;
    }

    const float kPromptWidth = std::max(520.0f, total_w + 48.0f);
    ImVec2 prompt_center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(prompt_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(kPromptWidth, 0.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(kPromptWidth, -1.0f), ImVec2(kPromptWidth, -1.0f));
    if (ImGui::BeginPopupModal(modal_title.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::Spacing();
        ImGui::TextColored(theme::pal().text_warn, "SCRIPT PROMPT");
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::TextWrapped("%s", telem.script_prompt_message.c_str());
        ImGui::Spacing();

        // Optional input field (cnc.input / cnc.input_number)
        bool input_valid = true;
        bool enter_pressed = false;
        if (has_input) {
            if (focus_input) ImGui::SetKeyboardFocusHere();
            ImGui::SetNextItemWidth(-FLT_MIN);
            const ImGuiInputTextFlags flags = ImGuiInputTextFlags_EnterReturnsTrue |
                                              (numeric ? ImGuiInputTextFlags_CharsScientific : 0);
            enter_pressed = ImGui::InputText("##ScriptPromptInput", prompt_input_buf_, sizeof(prompt_input_buf_), flags);
            if (numeric) {
                char* end = nullptr;
                const double v = std::strtod(prompt_input_buf_, &end);
                const bool parsed = (end != prompt_input_buf_) && (*end == '\0') && std::isfinite(v);
                const double lo = telem.script_prompt_input_min;
                const double hi = telem.script_prompt_input_max;
                input_valid = parsed && v >= lo && v <= hi;
                ImGui::PushTextWrapPos(0.0f);
                if (lo > -1e299 && hi < 1e299) {
                    ImGui::TextColored(input_valid ? theme::pal().text_muted : theme::pal().text_danger,
                                       "Enter a number between %g and %g", lo, hi);
                } else if (lo > -1e299) {
                    ImGui::TextColored(input_valid ? theme::pal().text_muted : theme::pal().text_danger,
                                       "Enter a number of at least %g", lo);
                } else if (hi < 1e299) {
                    ImGui::TextColored(input_valid ? theme::pal().text_muted : theme::pal().text_danger,
                                       "Enter a number of at most %g", hi);
                } else if (!input_valid) {
                    ImGui::TextColored(theme::pal().text_danger, "Enter a valid number");
                }
                ImGui::PopTextWrapPos();
            }
            ImGui::Spacing();
        }

        ImGui::Separator();
        ImGui::Spacing();

        if (has_input && enter_pressed && input_valid) {
            controller.submitScriptPromptInput("OK", prompt_input_buf_);
            ImGui::CloseCurrentPopup();
        }

        float start_x = (ImGui::GetWindowWidth() - total_w) * 0.5f;
        if (start_x > 15.0f) {
            ImGui::SetCursorPosX(start_x);
        }

        for (size_t i = 0; i < buttons.size(); ++i) {
            if (i > 0) ImGui::SameLine(0.0f, spacing);
            const bool blocked = has_input && !input_valid && buttons[i] == "OK";

            bool is_primary = (i == 0);
            bool is_already_probed = (buttons[i].find("already probed") != std::string::npos);
            if (is_primary) {
                ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_ok);
            } else if (is_already_probed) {
                ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_primary);
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_neutral);
            }
            ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(ImGui::GetStyleColorVec4(ImGuiCol_Button)));

            ImGui::BeginDisabled(blocked);
            if (ImGui::Button(buttons[i].c_str(), ImVec2(btn_widths[i], kBtnH))) {
                if (has_input) {
                    controller.submitScriptPromptInput(buttons[i], prompt_input_buf_);
                } else {
                    controller.submitScriptPromptResponse(buttons[i]);
                }
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndDisabled();

            ImGui::PopStyleColor(2);
        }

        ImGui::EndPopup();
    }
}

bool ScriptingPanel::renderHelpIconButton(const char* str_id, ImVec2 pos, float size) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems)
        return false;

    const ImGuiID id = window->GetID(str_id);
    const ImRect bb(pos, ImVec2(pos.x + size, pos.y + size));

    bool hovered = false;
    bool held    = false;
    bool pressed = false;

    if (ImGui::ItemAdd(bb, id)) {
        pressed = ImGui::ButtonBehavior(bb, id, &hovered, &held);
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 center(pos.x + size * 0.5f, pos.y + size * 0.5f);
    const float radius = size * 0.5f - 1.0f;

    // Outer circle
    ImU32 circle_col;
    ImU32 i_col;
    if (held) {
        circle_col = theme::pal().icon_active;
        i_col      = theme::pal().icon_active;
        dl->AddCircleFilled(center, radius, theme::withAlpha(theme::pal().icon_active, 40), 24);
    } else if (hovered) {
        circle_col = theme::pal().icon_hover;
        i_col      = theme::pal().icon_hover;
        dl->AddCircleFilled(center, radius, theme::withAlpha(theme::pal().icon_hover, 25), 24);
    } else {
        circle_col = theme::pal().icon_idle;
        i_col      = theme::pal().icon_idle;
    }

    dl->AddCircle(center, radius, circle_col, 24, 1.8f);

    // Stylized 'i'
    // Dot
    const float dot_r = size * 0.088f;
    const ImVec2 dot_c(center.x, center.y - radius * 0.42f);
    dl->AddCircleFilled(dot_c, dot_r, i_col, 16);

    // Stem
    const float stem_w   = size * 0.13f;
    const float stem_top = center.y - radius * 0.10f;
    const float stem_bot = center.y + radius * 0.55f;
    dl->AddRectFilled(ImVec2(center.x - stem_w * 0.5f, stem_top),
                      ImVec2(center.x + stem_w * 0.5f, stem_bot),
                      i_col, 1.0f);

    // Top serif
    dl->AddLine(ImVec2(center.x - stem_w * 0.5f - 2.0f, stem_top + 1.2f),
                ImVec2(center.x - stem_w * 0.5f, stem_top),
                i_col, 1.2f);
    // Base serif
    dl->AddLine(ImVec2(center.x - stem_w * 0.5f - 2.0f, stem_bot),
                ImVec2(center.x + stem_w * 0.5f + 2.0f, stem_bot),
                i_col, 1.2f);

    if (hovered) {
        ImGui::SetTooltip("Lua scripting guide and API reference");
    }

    return pressed;
}

void ScriptingPanel::renderHelpModal(core::IMotionController& controller) {
    const char* kTitle = "Lua Scripting & Macro Guide##LuaHelpModal";
    if (show_help_modal_) {
        ImGui::OpenPopup(kTitle);
        show_help_modal_ = false;
    }

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(std::min(1040.0f, vp->Size.x * 0.92f), std::min(780.0f, vp->Size.y * 0.90f)),
                             ImGuiCond_Appearing);

    bool open = true;
    if (ImGui::BeginPopupModal(kTitle, &open)) {
        ImGui::PushStyleColor(ImGuiCol_Text, theme::pal().text_info);
        ImGui::TextUnformatted("LUA 5.4 SCRIPTING & MACRO GUIDE");
        ImGui::PopStyleColor();
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Tutorials, dialog cookbook, ready-to-adapt machine scripts, hooks, searchable API "
                            "reference and safety notes.");
        ImGui::PopTextWrapPos();
        ImGui::Separator();

        // Guide content fills the window except for the footer row with the Close button
        const float footer_h = 32.0f + ImGui::GetStyle().ItemSpacing.y * 2.0f + 6.0f;
        ImGui::BeginChild("##GuideBody", ImVec2(0.0f, -footer_h));
        guide_.render(controller);
        ImGui::EndChild();

        ImGui::Separator();
        const float close_w = ImGui::CalcTextSize("Close Guide").x + 32.0f;
        ImGui::SetCursorPosX(ImGui::GetWindowWidth() - close_w - ImGui::GetStyle().WindowPadding.x);
        if (ImGui::Button("Close Guide", ImVec2(close_w, 32.0f))) {
            ImGui::CloseCurrentPopup();
        }

        // Script dialogs (e.g. from "Run demo") open as a child modal of the guide
        guide_open_ = true;
        renderPromptModalImpl(controller);
        ImGui::EndPopup();
    } else {
        guide_open_ = false;
    }
}

} // namespace rnr::ui
