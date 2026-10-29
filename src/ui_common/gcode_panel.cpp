#include "ui_common/gcode_panel.h"
#include "ui_common/app_window.h"
#include "ui_common/file_dialog.h"
#include "imgui.h"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace rnr::ui {

void GCodeProgramPanel::render(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    const std::string& loaded_filename,
    size_t pierce_count,
    double total_cut_length_mm,
    const std::function<void()>& on_load_file_clicked)
{
    const auto program = controller.getProgram(); // Snapshot stays valid even if a new file is loaded meanwhile
    const auto& gcode_lines = program->lines;
    updateLineNumberCache(gcode_lines, loaded_filename);
    const bool is_running = (telemetry.state == core::ControllerState::RunningGCode ||
                             telemetry.state == core::ControllerState::DwellPierce);

    if (selected_line_ >= gcode_lines.size() && !gcode_lines.empty()) {
        selected_line_ = 0;
    }

    const bool is_estop = (telemetry.state == core::ControllerState::EStop);
    const bool is_tool_change = (telemetry.state == core::ControllerState::ToolChangeHold ||
                                 telemetry.pending_tool_number > 0);

    // Sync active_file_path_ if loaded_filename points to an existing file
    if (!loaded_filename.empty() && loaded_filename != active_file_path_) {
        std::error_code ec;
        if (std::filesystem::exists(loaded_filename, ec)) {
            active_file_path_ = loaded_filename;
            last_file_write_time_ = std::filesystem::last_write_time(active_file_path_, ec);
            has_file_write_time_ = !ec;
        }
    }

    // Auto-reload check if active file was modified externally in an editor
    if (has_file_write_time_ && !active_file_path_.empty()) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_file_poll_time_ > std::chrono::milliseconds(250)) {
            last_file_poll_time_ = now;
            std::error_code ec;
            if (std::filesystem::exists(active_file_path_, ec)) {
                auto cur_time = std::filesystem::last_write_time(active_file_path_, ec);
                if (!ec && cur_time != last_file_write_time_) {
                    const bool is_cutting = (telemetry.state == core::ControllerState::RunningGCode ||
                                             telemetry.state == core::ControllerState::DwellPierce);
                    if (!is_cutting) {
                        std::ifstream ifs(active_file_path_);
                        if (ifs) {
                            std::ostringstream ss;
                            ss << ifs.rdbuf();
                            std::string new_content = ss.str();
                            if (!new_content.empty()) {
                                last_file_write_time_ = cur_time;
                                if (on_file_approved_) {
                                    on_file_approved_(active_file_path_, new_content);
                                    selected_line_           = 0;
                                    prev_active_line_        = 0;
                                    scroll_to_top_requested_ = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // 1. Top Control Row: STOP/REWIND | FEED HOLD | Set Next Line | Program Info
    const float btn_w = 110.0f;
    const float btn_h = 34.0f;

    // STOP / REWIND
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.65f, 0.18f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.78f, 0.24f, 0.24f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.55f, 0.14f, 0.14f, 1.0f));
    if (ImGui::Button("STOP / REWIND", ImVec2(btn_w, btn_h))) {
        stopAndRewind(controller, telemetry);
    }
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemHovered()) {
        const bool in_progress = (telemetry.state == core::ControllerState::RunningGCode ||
                                  telemetry.state == core::ControllerState::DwellPierce ||
                                  telemetry.state == core::ControllerState::PausedHold ||
                                  telemetry.state == core::ControllerState::ToolChangeHold ||
                                  telemetry.state == core::ControllerState::Probing);
        if (in_progress) {
            ImGui::SetTooltip("Stop milling immediately (stays on last active G-Code line).");
        } else {
            ImGui::SetTooltip("Rewind program back to line 1 (sets line 1 as next line).");
        }
    }

    ImGui::SameLine();

    // FEED HOLD
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.52f, 0.08f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.62f, 0.12f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.65f, 0.44f, 0.06f, 1.0f));
    if (ImGui::Button("FEED HOLD", ImVec2(btn_w, btn_h))) {
        controller.pauseJob();
    }
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Pause motion execution immediately (Feed Hold)");
    }

    ImGui::SameLine();

    // Set Next Line
    ImGui::BeginDisabled(is_running || gcode_lines.empty());
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.14f, 0.42f, 0.62f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.54f, 0.78f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.10f, 0.34f, 0.50f, 1.0f));
    if (ImGui::Button("Set Next Line", ImVec2(btn_w, btn_h))) {
        controller.setNextGCodeLine(selected_line_);
    }
    ImGui::PopStyleColor(3);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Set selected G-Code line as the next line to execute on Cycle Start");
    }

    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    std::string display_filename;
    if (!loaded_filename.empty()) {
        std::filesystem::path p(loaded_filename);
        display_filename = p.filename().string();
    }
    ImGui::TextDisabled("Next:#%04zu | %s (%zu pierces, %.0fmm)",
                        telemetry.active_gcode_line_index + 1,
                        display_filename.empty() ? "(none)" : display_filename.c_str(),
                        pierce_count,
                        total_cut_length_mm);

    // 1b. Prominent Manual Tool Change (M6) Prompt Banner when waiting for operator tool change
    if (is_tool_change) {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.32f, 0.16f, 0.04f, 0.95f));
        ImGui::BeginChild("##ManualToolChangeBanner", ImVec2(0.0f, 50.0f), true, ImGuiWindowFlags_NoScrollbar);
        
        const float btn_w = 220.0f;
        const float avail_w = ImGui::GetContentRegionAvail().x;
        const float text_w = std::max(120.0f, avail_w - btn_w - 14.0f);
        const bool script_active = (telemetry.script_running || telemetry.has_script_prompt);

        ImGui::BeginGroup();
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + text_w);
        if (script_active) {
            ImGui::TextColored(
                ImVec4(1.0f, 0.85f, 0.25f, 1.0f),
                "M6 TOOL CHANGE SCRIPT ACTIVE: T%d%s%s",
                telemetry.pending_tool_number,
                telemetry.pending_tool_description.empty() ? "" : " (",
                telemetry.pending_tool_description.empty() ? "" : (telemetry.pending_tool_description + ")").c_str());
            ImGui::TextDisabled("Follow operator dialog prompt or allow automated tool probe sequence to complete.");
        } else if (!telemetry.pending_tool_description.empty()) {
            ImGui::TextColored(
                ImVec4(1.0f, 0.85f, 0.25f, 1.0f),
                "MANUAL TOOL CHANGE (M6): Insert Tool T%d (%s)",
                telemetry.pending_tool_number,
                telemetry.pending_tool_description.c_str());
            ImGui::TextDisabled("Remove probe plate from workpiece, then press Cycle Start to begin milling.");
        } else {
            ImGui::TextColored(
                ImVec4(1.0f, 0.85f, 0.25f, 1.0f),
                "MANUAL TOOL CHANGE (M6): Insert Tool T%d",
                telemetry.pending_tool_number);
            ImGui::TextDisabled("Remove probe plate from workpiece, then press Cycle Start to begin milling.");
        }
        ImGui::PopTextWrapPos();
        ImGui::EndGroup();

        ImGui::SameLine(avail_w - btn_w);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 2.0f);
        if (script_active) {
            ImGui::BeginDisabled();
            ImGui::Button("RUNNING SCRIPT...", ImVec2(btn_w, 32.0f));
            ImGui::EndDisabled();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f, 0.68f, 0.32f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.82f, 0.40f, 1.0f));
            char resume_lbl[64];
            std::snprintf(resume_lbl, sizeof(resume_lbl), "TOOL T%d READY -> RESUME", telemetry.pending_tool_number);
            if (ImGui::Button(resume_lbl, ImVec2(btn_w, 32.0f))) {
                triggerCycleStart(controller, telemetry);
            }
            ImGui::PopStyleColor(2);
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
    }

    // 1b. Program diagnostics: errors block Cycle Start, warnings are informational
    if (!program->errors.empty() || !program->warnings.empty()) {
        const bool has_errors = !program->errors.empty();
        ImGui::PushStyleColor(ImGuiCol_ChildBg, has_errors ? ImVec4(0.30f, 0.06f, 0.06f, 1.0f) : ImVec4(0.26f, 0.20f, 0.04f, 1.0f));
        ImGui::BeginChild("##ProgramDiagnostics", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
        ImGui::PushTextWrapPos(0.0f);
        const auto& list = has_errors ? program->errors : program->warnings;
        const ImVec4 col = has_errors ? ImVec4(1.0f, 0.45f, 0.40f, 1.0f) : ImVec4(1.0f, 0.82f, 0.30f, 1.0f);
        ImGui::TextColored(col, has_errors ? "Program cannot run - %zu error(s):" : "%zu warning(s):", list.size());
        for (size_t i = 0; i < list.size() && i < 3; ++i) {
            ImGui::TextColored(col, "Line %zu: %s", list[i].line + 1, list[i].text.c_str());
        }
        if (list.size() > 3) {
            ImGui::TextDisabled("... and %zu more", list.size() - 3);
        }
        ImGui::PopTextWrapPos();
        ImGui::EndChild();
        ImGui::PopStyleColor();
    }

    // 2. Whole-Line Snap-Scrolled G-Code Listing
    constexpr float kRowHeight = 22.0f;
    const float mdi_needed_h = ImGui::GetStyle().ItemSpacing.y + 6.0f + ImGui::GetFrameHeight();
    const float avail_h = ImGui::GetContentRegionAvail().y - mdi_needed_h;
    const int visible_rows = std::max(3, static_cast<int>(std::floor((avail_h - 2.0f) / kRowHeight)));
    const float exact_scroll_h = static_cast<float>(visible_rows) * kRowHeight + 2.0f; // +2px for top/bottom 1px border

    ImFont* gcode_font = AppWindow::getGCodeFont();
    if (gcode_font) {
        ImGui::PushFont(gcode_font);
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
    ImGui::BeginChild("##GCodeScroll", ImVec2(0.0f, exact_scroll_h), true, ImGuiWindowFlags_NoScrollWithMouse);

    if (scroll_to_top_requested_ || scroll_to_top_frames_ > 0) {
        ImGui::SetScrollY(0.0f);
        scroll_to_top_requested_ = false;
        if (scroll_to_top_frames_ > 0) {
            --scroll_to_top_frames_;
        }
    }

    // Custom discrete whole-line mouse wheel scrolling
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
        float wheel = ImGui::GetIO().MouseWheel;
        if (std::abs(wheel) > 0.001f) {
            float step_lines = (wheel > 0.0f) ? -3.0f : 3.0f;
            float new_scroll = std::round(ImGui::GetScrollY() / kRowHeight + step_lines) * kRowHeight;
            new_scroll = std::clamp(new_scroll, 0.0f, ImGui::GetScrollMaxY());
            ImGui::SetScrollY(new_scroll);
        }
    }

    // Auto-scroll to active line while program is running (always quantized to exact multiples of kRowHeight)
    if (scroll_to_top_frames_ == 0 && (is_running || is_tool_change) && telemetry.active_gcode_line_index != prev_active_line_) {
        prev_active_line_ = telemetry.active_gcode_line_index;
        float active_y  = static_cast<float>(telemetry.active_gcode_line_index) * kRowHeight;
        float cur_scr   = ImGui::GetScrollY();
        float view_span = static_cast<float>(visible_rows) * kRowHeight;
        if (active_y < cur_scr || active_y >= cur_scr + view_span - kRowHeight) {
            float target_top_line = std::max(0.0f, static_cast<float>(telemetry.active_gcode_line_index) - std::floor(visible_rows * 0.35f));
            float snapped_target  = std::clamp(target_top_line * kRowHeight, 0.0f, ImGui::GetScrollMaxY());
            ImGui::SetScrollY(snapped_target);
        }
    }

    // Snap scrollbar drag to exact multiples of kRowHeight so half lines are never visible
    if (scroll_to_top_frames_ == 0 && !scroll_to_top_requested_) {
        float cur_scr = ImGui::GetScrollY();
        float snapped = std::round(cur_scr / kRowHeight) * kRowHeight;
        if (std::abs(cur_scr - snapped) > 0.05f) {
            ImGui::SetScrollY(snapped);
        }
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float row_full_w = ImGui::GetContentRegionAvail().x;

    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(gcode_lines.size()), kRowHeight);
    while (clipper.Step()) {
        for (int row_i = clipper.DisplayStart; row_i < clipper.DisplayEnd; ++row_i) {
            const size_t i = static_cast<size_t>(row_i);
            ImGui::PushID(row_i);

            ImVec2 row_p0 = ImGui::GetCursorScreenPos();
            ImVec2 row_p1(row_p0.x + row_full_w, row_p0.y + kRowHeight);

            const bool is_current_line  = (i == telemetry.active_gcode_line_index);
            const bool is_selected_line = (!is_running && i == selected_line_);

            // 2a. Row Background: Current Line Highlight vs. Selected Line vs. Faint Alternating Zebra
            ImU32 bg_col = (i % 2 == 0) ? IM_COL32(20, 24, 30, 255) : IM_COL32(25, 30, 37, 255);
            if (is_current_line) {
                bg_col = is_running ? IM_COL32(88, 62, 10, 240) : IM_COL32(16, 62, 82, 230);
            } else if (is_selected_line) {
                bg_col = IM_COL32(36, 54, 78, 240);
            }
            dl->AddRectFilled(row_p0, row_p1, bg_col);

            // Left accent bar for Current Line or Selected Line
            if (is_current_line) {
                ImU32 accent = is_running ? IM_COL32(255, 195, 0, 255) : IM_COL32(0, 225, 255, 255);
                dl->AddRectFilled(row_p0, ImVec2(row_p0.x + 4.0f, row_p1.y), accent);
            } else if (is_selected_line) {
                dl->AddRect(row_p0, row_p1, IM_COL32(85, 145, 220, 160));
            }

            // Invisible button across the row for selection when program is not running
            if (ImGui::InvisibleButton("##GCodeRowBtn", ImVec2(row_full_w, kRowHeight))) {
                if (!is_running) {
                    selected_line_ = i;
                }
            }
            if (!is_running && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                selected_line_ = i;
                controller.setNextGCodeLine(i);
            }

            const auto& pld = (i < parsed_display_lines_.size())
                                  ? parsed_display_lines_[i]
                                  : ParsedLineDisplay{};

            // 2b. Line Number Column (distinct color from G-code command)
            char num_buf[32];
            if (pld.line_num_str.empty()) {
                std::snprintf(num_buf, sizeof(num_buf), "%c", is_current_line ? '>' : ' ');
            } else {
                std::snprintf(num_buf, sizeof(num_buf), "%c%s", is_current_line ? '>' : ' ', pld.line_num_str.c_str());
            }
            ImU32 num_col = is_current_line
                                ? (is_running ? IM_COL32(255, 215, 60, 255) : IM_COL32(0, 230, 255, 255))
                                : IM_COL32(92, 148, 185, 255);
            dl->AddText(ImVec2(row_p0.x + 8.0f, row_p0.y + 3.0f), num_col, num_buf);

            // Subtle vertical column separator between line number and command
            dl->AddLine(ImVec2(row_p0.x + line_num_col_w_, row_p0.y),
                        ImVec2(row_p0.x + line_num_col_w_, row_p1.y),
                        IM_COL32(42, 50, 62, 180));

            // 2c. G-Code Command & Comment Column (split at ';' or '(' so Mach3 comments have a subtle color)
            const std::string& line_str = pld.command_str;
            size_t comment_pos = pld.comment_pos;
            ImU32 cmd_col = is_current_line
                                ? IM_COL32(255, 245, 190, 255)
                                : IM_COL32(230, 238, 248, 255);
            ImU32 cmt_col = is_current_line
                                ? IM_COL32(205, 190, 120, 240)
                                : IM_COL32(115, 138, 126, 240);

            const float cmd_x = row_p0.x + line_num_col_w_ + 10.0f;
            if (comment_pos == std::string::npos) {
                dl->AddText(ImVec2(cmd_x, row_p0.y + 3.0f), cmd_col, line_str.c_str());
            } else {
                std::string cmd_part = line_str.substr(0, comment_pos);
                std::string cmt_part = line_str.substr(comment_pos);
                ImVec2 cmd_pos(cmd_x, row_p0.y + 3.0f);
                dl->AddText(cmd_pos, cmd_col, cmd_part.c_str());
                ImVec2 cmd_sz = ImGui::CalcTextSize(cmd_part.c_str());
                dl->AddText(ImVec2(cmd_pos.x + cmd_sz.x, cmd_pos.y), cmt_col, cmt_part.c_str());
            }

            ImGui::PopID();
        }
    }

    ImGui::EndChild();
    ImGui::PopStyleVar(2);

    // 3. Bottom Control Row: Load File, Edit File, MDI Input Bar
    // Shifted 6px down so buttons and input field do not touch the G-code scroll window border
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6.0f);

    // Load File button moved down to the left of MDI G-Code
    if (ImGui::Button("Load File", ImVec2(120.0f, 0.0f))) {
        if (on_load_file_clicked) {
            on_load_file_clicked();
            selected_line_           = 0;
            prev_active_line_        = 0;
            scroll_to_top_requested_ = true;
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Open G-Code Program (*.nc, *.tap, *.gcode, *.txt)");
    }

    ImGui::SameLine();

    // Edit File button: active only if an external file is open on disk
    const bool has_open_file = !active_file_path_.empty() && std::filesystem::exists(active_file_path_);
    ImGui::BeginDisabled(!has_open_file);
    if (ImGui::Button("Edit File", ImVec2(120.0f, 0.0f))) {
        openFileInDefaultEditor(active_file_path_);
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        if (has_open_file) {
            ImGui::SetTooltip("Open file in default editor:\n%s\n(Changes will auto-reload when saved)", active_file_path_.c_str());
        } else {
            ImGui::SetTooltip("No active G-Code file currently open on disk to edit.");
        }
    }

    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(ImVec4(0.0f, 0.88f, 1.0f, 1.0f), "MDI G-Code:");

    ImGui::SameLine();
    ImGui::BeginDisabled(is_estop);

    // Input field length reduced to half the size of the previous width
    const float avail_w = ImGui::GetContentRegionAvail().x;
    const float mdi_input_w = std::clamp((avail_w - 180.0f) * 0.5f, 150.0f, 320.0f);
    ImGui::SetNextItemWidth(mdi_input_w);

    bool send_mdi = ImGui::InputTextWithHint(
        "##MdiInput",
        "Enter manual G-Code",
        mdi_input_buf_,
        sizeof(mdi_input_buf_),
        ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if (ImGui::Button("Send MDI", ImVec2(95.0f, 0.0f))) {
        send_mdi = true;
    }
    if (send_mdi && mdi_input_buf_[0] != '\0' && !is_estop) {
        if (controller.executeMdiCommand(mdi_input_buf_)) {
            mdi_input_buf_[0] = '\0';
        }
        ImGui::SetKeyboardFocusHere(-1);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (!telemetry.last_mdi_command.empty()) {
        ImGui::TextDisabled("Last: %s", telemetry.last_mdi_command.c_str());
    } else {
        ImGui::TextDisabled("Ready");
    }

    if (gcode_font) {
        ImGui::PopFont();
        gcode_font = nullptr;
    }

    // Spindle Start Warning Modal (prompted when cycle start is hit while cycle is stopped and spindle is off)
    renderSpindleModal(controller, telemetry);
    renderMismatchModal();
}

void GCodeProgramPanel::render(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    const std::string& loaded_filename,
    size_t pierce_count,
    double total_cut_length_mm,
    const std::function<void(const std::string& path, const std::string& content)>& on_file_approved)
{
    on_file_approved_ = on_file_approved;
    render(controller, telemetry, loaded_filename, pierce_count, total_cut_length_mm, [this]() {
        promptLoadFileDialog(on_file_approved_);
    });
}

void GCodeProgramPanel::promptLoadFileDialog(
    const std::function<void(const std::string& path, const std::string& content)>& on_file_approved)
{
    on_file_approved_ = on_file_approved;
    std::string path = ui::openGCodeFileDialog();
    if (path.empty()) return;

    std::ifstream ifs(path);
    if (!ifs) return;

    std::ostringstream ss;
    ss << ifs.rdbuf();
    std::string content = ss.str();

    checkAndPromptGCodeFile(path, content, on_file_approved);
}

void GCodeProgramPanel::checkAndPromptGCodeFile(
    const std::string& file_path,
    const std::string& gcode_content,
    const std::function<void(const std::string& path, const std::string& content)>& on_file_approved)
{
    on_file_approved_ = on_file_approved;
    auto check = gcode::GCodeParser::checkMachineCompatibility(gcode_content, machine_type_);
    if (!check.has_mismatch) {
        active_file_path_ = file_path;
        std::error_code ec;
        last_file_write_time_ = std::filesystem::last_write_time(active_file_path_, ec);
        has_file_write_time_ = !ec;
        if (on_file_approved_) {
            on_file_approved_(file_path, gcode_content);
            selected_line_           = 0;
            prev_active_line_        = 0;
            scroll_to_top_requested_ = true;
        }
    } else {
        pending_file_path_    = file_path;
        pending_file_content_ = gcode_content;
        pending_mismatch_     = check;
        show_mismatch_modal_  = true;
    }
}

void GCodeProgramPanel::triggerCycleStart(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry)
{
    const bool is_running = (telemetry.state == core::ControllerState::RunningGCode ||
                             telemetry.state == core::ControllerState::DwellPierce);
    const bool is_tool_change = (telemetry.state == core::ControllerState::ToolChangeHold ||
                                 telemetry.pending_tool_number > 0);
    const bool is_start_from_beginning = (telemetry.active_gcode_line_index == 0);
    const bool is_resuming = is_running || is_tool_change || (telemetry.state == core::ControllerState::PausedHold);

    // Only show spindle warning modal if cycle start is hit while program is NOT started from the beginning
    if (!is_resuming && !is_start_from_beginning && !telemetry.torch_or_spindle_on && !telemetry.dry_run_mode) {
        show_spindle_warning_modal_ = true;
    } else {
        controller.startJob();
    }
}

void GCodeProgramPanel::stopAndRewind(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry)
{
    const bool is_in_progress = (telemetry.state == core::ControllerState::RunningGCode ||
                                 telemetry.state == core::ControllerState::DwellPierce ||
                                 telemetry.state == core::ControllerState::PausedHold ||
                                 telemetry.state == core::ControllerState::ToolChangeHold ||
                                 telemetry.state == core::ControllerState::Probing);

    if (is_in_progress) {
        // 1. Milling/motion in progress: Stop motion immediately, but keep G-code on the last active line
        controller.cancelScript();
        controller.stopJob();
        selected_line_           = telemetry.active_gcode_line_index;
        controller.setNextGCodeLine(telemetry.active_gcode_line_index);
        scroll_to_top_frames_    = 0;
        scroll_to_top_requested_ = false;
    } else {
        // 2. Program is not running (already stopped or idle): Rewind back to line 1 and set line 1 as next line
        controller.cancelScript();
        controller.stopJob();
        controller.setNextGCodeLine(0);
        selected_line_           = 0;
        prev_active_line_        = 0;
        scroll_to_top_frames_    = 3;
        scroll_to_top_requested_ = true;
    }
}

void GCodeProgramPanel::renderCycleControlGrid(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    float target_w,
    float offset_x)
{
    const auto program = controller.getProgram(); // Snapshot stays valid even if a new file is loaded meanwhile
    const auto& gcode_lines = program->lines;
    const bool is_running = (telemetry.state == core::ControllerState::RunningGCode ||
                             telemetry.state == core::ControllerState::DwellPierce);
    const bool is_estop = (telemetry.state == core::ControllerState::EStop);
    const bool is_tool_change = (telemetry.state == core::ControllerState::ToolChangeHold ||
                                 telemetry.pending_tool_number > 0);

    if (selected_line_ >= gcode_lines.size() && !gcode_lines.empty()) {
        selected_line_ = 0;
    }

    if (target_w <= 0.0f) {
        target_w = ImGui::GetContentRegionAvail().x;
    }

    const float start_x = (offset_x >= 0.0f) ? offset_x : ImGui::GetCursorPosX();
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float btn_w0  = std::floor((target_w - spacing) * 0.5f);
    const float btn_w1  = target_w - btn_w0 - spacing;
    const float btn_h   = 36.0f;
    const bool compact  = (target_w < 340.0f);

    // Row 1: START CUT (CYCLE START) | FEED HOLD
    ImGui::SetCursorPosX(start_x);
    ImGui::BeginDisabled(is_estop);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.10f, 0.62f, 0.30f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.14f, 0.76f, 0.38f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.08f, 0.52f, 0.24f, 1.0f));

    const bool is_surf_comp = telemetry.surface_compensation_active;
    if (is_surf_comp) {
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.00f, 0.95f, 0.55f, 0.95f));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
    }

    std::string start_label_str;
    if (is_tool_change) {
        start_label_str = compact ? "RESUME (M6)" : "RESUME CYCLE (M6)";
    } else if (is_surf_comp) {
        start_label_str = compact ? "CYCLE START [COMP]" : "CYCLE START [SURFACE COMPENSATED]";
    } else {
        start_label_str = compact ? "CYCLE START" : "START CUT (CYCLE START)";
    }
    const char* start_label = start_label_str.c_str();

    if (ImGui::Button(start_label, ImVec2(btn_w0, btn_h))) {
        triggerCycleStart(controller, telemetry);
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

    ImGui::SameLine(0.0f, spacing);

    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.52f, 0.08f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.62f, 0.12f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.65f, 0.44f, 0.06f, 1.0f));
    if (ImGui::Button("FEED HOLD", ImVec2(btn_w1, btn_h))) {
        controller.pauseJob();
    }
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Pause motion execution immediately (Feed Hold)");
    }

    // Row 2: STOP / REWIND | Set Next Line
    ImGui::SetCursorPosX(start_x);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.65f, 0.18f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.78f, 0.24f, 0.24f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.55f, 0.14f, 0.14f, 1.0f));
    if (ImGui::Button("STOP / REWIND", ImVec2(btn_w0, btn_h))) {
        stopAndRewind(controller, telemetry);
    }
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemHovered()) {
        const bool in_progress = (telemetry.state == core::ControllerState::RunningGCode ||
                                  telemetry.state == core::ControllerState::DwellPierce ||
                                  telemetry.state == core::ControllerState::PausedHold ||
                                  telemetry.state == core::ControllerState::ToolChangeHold ||
                                  telemetry.state == core::ControllerState::Probing);
        if (in_progress) {
            ImGui::SetTooltip("Stop milling immediately (stays on last active G-Code line).");
        } else {
            ImGui::SetTooltip("Rewind program back to line 1 (sets line 1 as next line).");
        }
    }

    ImGui::SameLine(0.0f, spacing);

    ImGui::BeginDisabled(is_running || gcode_lines.empty());
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.14f, 0.42f, 0.62f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.54f, 0.78f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.10f, 0.34f, 0.50f, 1.0f));
    if (ImGui::Button("Set Next Line", ImVec2(btn_w1, btn_h))) {
        controller.setNextGCodeLine(selected_line_);
    }
    ImGui::PopStyleColor(3);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Set selected G-Code line as the next line to execute on Cycle Start");
    }

    renderSpindleModal(controller, telemetry);
}

void GCodeProgramPanel::renderSpindleModal(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry)
{
    if (show_spindle_warning_modal_) {
        ImGui::OpenPopup("Spindle Not Running##SpindleWarningModal");
        show_spindle_warning_modal_ = false;
    }

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(520.0f, 0.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(520.0f, -1.0f), ImVec2(520.0f, -1.0f));
    if (ImGui::BeginPopupModal("Spindle Not Running##SpindleWarningModal", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        int rpm = (telemetry.commanded_spindle_rpm > 0) ? telemetry.commanded_spindle_rpm : 12000;
        ImGui::Spacing();
        ImGui::TextWrapped("Spindle is not running, would you like to start it again with the last Spindle speed (%d RPM)?", rpm);
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        char btn_start_label[64];
        std::snprintf(btn_start_label, sizeof(btn_start_label), "Start with %d RPM", rpm);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.62f, 0.30f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.14f, 0.76f, 0.38f, 1.0f));
        if (ImGui::Button(btn_start_label, ImVec2(200.0f, 36.0f))) {
            uint16_t duty = static_cast<uint16_t>((std::clamp(rpm, 1000, 24000) * 1000) / 24000);
            controller.setSpindlePwm(duty);
            controller.setTorchOrSpindle(true);
            controller.startJob();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();
        if (ImGui::Button("Start Without Spindle", ImVec2(180.0f, 36.0f))) {
            controller.startJob();
            ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.16f, 0.16f, 1.0f));
        if (ImGui::Button("Cancel", ImVec2(100.0f, 36.0f))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor();
        ImGui::EndPopup();
    }
}

void GCodeProgramPanel::renderMismatchModal() {
    if (show_mismatch_modal_) {
        ImGui::OpenPopup("Machine Target Mismatch Warning##GCodeMismatchModal");
        show_mismatch_modal_ = false;
    }

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(620.0f, 0.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(620.0f, -1.0f), ImVec2(620.0f, -1.0f));

    ImGui::PushStyleColor(ImGuiCol_PopupBg,          ImVec4(0.10f, 0.12f, 0.15f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ModalWindowDimBg, ImVec4(0.00f, 0.00f, 0.00f, 0.70f));

    if (ImGui::BeginPopupModal("Machine Target Mismatch Warning##GCodeMismatchModal", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::Spacing();

        // 1. Warning badge & Title
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.20f, 1.0f));
        ImGui::SetWindowFontScale(1.15f);
        ImGui::Text("  [ ! ]  Machine Target Mismatch Warning");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        std::string filename_only = std::filesystem::path(pending_file_path_).filename().string();
        ImGui::TextWrapped("The selected G-Code file appears to be intended for a different machine process than the current controller profile:");

        ImGui::Spacing();

        // Framed details card (dynamically sized to fit all bullet points cleanly)
        const float card_h = 88.0f + static_cast<float>(pending_mismatch_.indicators.size()) * 22.0f;
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.13f, 0.15f, 0.19f, 0.95f));
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::BeginChild("##MismatchInfoCard", ImVec2(0.0f, card_h), true, 0);

        ImGui::TextDisabled("Selected File: ");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "%s", filename_only.c_str());

        ImGui::TextDisabled("Detected Target: ");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.25f, 1.0f), "%s", pending_mismatch_.detected_name.c_str());

        ImGui::TextDisabled("Active Profile: ");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.32f, 0.80f, 1.0f, 1.0f), "%s", pending_mismatch_.current_name.c_str());

        if (!pending_mismatch_.indicators.empty()) {
            ImGui::Spacing();
            ImGui::TextDisabled("Identified Features:");
            for (const auto& ind : pending_mismatch_.indicators) {
                ImGui::Bullet();
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.92f, 0.94f, 0.98f, 1.0f), "%s", ind.c_str());
            }
        }

        ImGui::EndChild();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();

        ImGui::Spacing();

        // 2. High-contrast Warning Alert Banner with full auto-wrapping
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.24f, 0.14f, 0.04f, 0.95f));
        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.92f, 0.62f, 0.15f, 1.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));

        ImGui::BeginChild("##WarningCalloutBanner", ImVec2(0.0f, 62.0f), true, 0);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.88f, 0.60f, 1.0f)); // High-contrast warm gold
        ImGui::TextWrapped("Executing this program on the wrong machine could trigger unexpected outputs (such as firing a torch instead of a spindle) or invalid Z-axis motion.");
        ImGui::PopStyleColor();
        ImGui::EndChild();

        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(2);

        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.96f, 0.98f, 1.0f));
        ImGui::TextWrapped("Are you sure you want to load this file anyway?");
        ImGui::PopStyleColor();
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Action Buttons
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.72f, 0.40f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.52f, 0.14f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.60f, 0.32f, 0.08f, 1.0f));
        if (ImGui::Button("Load File Anyway", ImVec2(170.0f, 36.0f))) {
            active_file_path_ = pending_file_path_;
            std::error_code ec;
            last_file_write_time_ = std::filesystem::last_write_time(active_file_path_, ec);
            has_file_write_time_ = !ec;
            if (on_file_approved_) {
                on_file_approved_(pending_file_path_, pending_file_content_);
                selected_line_           = 0;
                prev_active_line_        = 0;
                scroll_to_top_requested_ = true;
            }
            pending_file_path_.clear();
            pending_file_content_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(3);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.25f, 0.28f, 0.34f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.32f, 0.36f, 0.44f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.20f, 0.22f, 0.28f, 1.0f));
        if (ImGui::Button("Cancel", ImVec2(120.0f, 36.0f))) {
            pending_file_path_.clear();
            pending_file_content_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(3);

        ImGui::EndPopup();
    }

    ImGui::PopStyleColor(2);
}

float GCodeProgramPanel::calculateDesiredHeight(float max_available_h, bool is_tool_change, int exact_rows) const {
    constexpr float kRowHeight = 22.0f;
    const float pad_y = ImGui::GetStyle().WindowPadding.y * 2.0f;
    const float spacing_y = ImGui::GetStyle().ItemSpacing.y;
    const float top_row_h = 34.0f;
    const float tool_change_h = is_tool_change ? (50.0f + spacing_y) : 0.0f;
    const float mdi_needed_h = spacing_y + 6.0f + ImGui::GetFrameHeight();
    const float non_scroll_h = pad_y + top_row_h + spacing_y + tool_change_h + mdi_needed_h;

    int visible_rows;
    if (exact_rows > 0) {
        visible_rows = exact_rows;
    } else {
        const float avail_for_scroll = max_available_h - non_scroll_h;
        visible_rows = std::max(3, static_cast<int>(std::floor((avail_for_scroll - 2.0f) / kRowHeight)));
    }
    const float exact_scroll_h = static_cast<float>(visible_rows) * kRowHeight + 2.0f;

    return non_scroll_h + exact_scroll_h;
}

void GCodeProgramPanel::updateLineNumberCache(const std::vector<std::string>& gcode_lines, const std::string& filename) {
    if (gcode_lines.size() == last_parsed_lines_count_ && filename == last_parsed_filename_ && !parsed_display_lines_.empty()) {
        return;
    }
    last_parsed_lines_count_ = gcode_lines.size();
    last_parsed_filename_    = filename;
    parsed_display_lines_.clear();
    parsed_display_lines_.resize(gcode_lines.size());

    // 1. Check if the file contains line numbers before g-codes (e.g. N100, N110, N140)
    size_t n_lines_count = 0;
    for (const auto& line : gcode_lines) {
        size_t idx = line.find_first_not_of(" \t");
        if (idx != std::string::npos && (line[idx] == 'N' || line[idx] == 'n')) {
            size_t d_idx = idx + 1;
            while (d_idx < line.size() && (line[d_idx] == ' ' || line[d_idx] == '\t')) d_idx++;
            if (d_idx < line.size() && std::isdigit(static_cast<unsigned char>(line[d_idx]))) {
                n_lines_count++;
            }
        }
    }

    const bool has_file_line_numbers = (n_lines_count > 0);
    size_t max_num_chars = 4;

    for (size_t i = 0; i < gcode_lines.size(); ++i) {
        const auto& raw = gcode_lines[i];
        auto& pld = parsed_display_lines_[i];

        if (has_file_line_numbers) {
            size_t idx = raw.find_first_not_of(" \t");
            if (idx != std::string::npos && (raw[idx] == 'N' || raw[idx] == 'n')) {
                size_t d_idx = idx + 1;
                while (d_idx < raw.size() && (raw[d_idx] == ' ' || raw[d_idx] == '\t')) d_idx++;
                if (d_idx < raw.size() && std::isdigit(static_cast<unsigned char>(raw[d_idx]))) {
                    size_t end_digit = d_idx;
                    while (end_digit < raw.size() && std::isdigit(static_cast<unsigned char>(raw[end_digit]))) {
                        end_digit++;
                    }
                    std::string num_digits = raw.substr(d_idx, end_digit - d_idx);
                    pld.line_num_str = "N" + num_digits;
                    if (pld.line_num_str.size() > max_num_chars) {
                        max_num_chars = pld.line_num_str.size();
                    }
                    size_t rest_idx = raw.find_first_not_of(" \t", end_digit);
                    pld.command_str = (rest_idx != std::string::npos) ? raw.substr(rest_idx) : "";
                } else {
                    pld.line_num_str.clear();
                    pld.command_str = raw;
                }
            } else {
                pld.line_num_str.clear();
                pld.command_str = raw;
            }
        } else {
            char buf[24]; // fits any size_t
            std::snprintf(buf, sizeof(buf), "%04zu", i + 1);
            pld.line_num_str = buf;
            pld.command_str  = raw;
        }

        size_t semi  = pld.command_str.find(';');
        size_t paren = pld.command_str.find('(');
        pld.comment_pos = std::min(semi, paren);
    }

    line_num_col_w_ = std::max(62.0f, static_cast<float>(max_num_chars + 2) * 9.5f + 16.0f);
}

} // namespace rnr::ui
