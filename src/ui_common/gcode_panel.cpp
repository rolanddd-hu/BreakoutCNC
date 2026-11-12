#include "ui_common/gcode_panel.h"
#include "core/util/utf8_path.h"
#include "ui_common/theme.h"
#include "ui_common/app_window.h"
#include "ui_common/frame_stats.h"
#include "ui_common/file_dialog.h"
#include "ui_common/gcode_line_lookup.h"
#include "imgui.h"
#include "imgui_internal.h"
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
    perf::ScopedTimer perf_timer("G-code list + cycle buttons");
    const auto program = controller.getProgram(); // Snapshot stays valid even if a new file is loaded meanwhile
    const auto& gcode_lines = program->lines;
    updateLineNumberCache(gcode_lines);
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
        if (std::filesystem::exists(util::pathFromUtf8(loaded_filename), ec)) {
            active_file_path_ = loaded_filename;
            last_file_write_time_ = std::filesystem::last_write_time(util::pathFromUtf8(active_file_path_), ec);
            has_file_write_time_ = !ec;
        }
    }

    // Auto-reload check if active file was modified externally in an editor
    if (has_file_write_time_ && !active_file_path_.empty()) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_file_poll_time_ > std::chrono::milliseconds(250)) {
            last_file_poll_time_ = now;
            std::error_code ec;
            if (std::filesystem::exists(util::pathFromUtf8(active_file_path_), ec)) {
                auto cur_time = std::filesystem::last_write_time(util::pathFromUtf8(active_file_path_), ec);
                if (!ec && cur_time != last_file_write_time_) {
                    const bool is_cutting = (telemetry.state == core::ControllerState::RunningGCode ||
                                             telemetry.state == core::ControllerState::DwellPierce);
                    if (!is_cutting) {
                        // Re-read and parsed on the worker; an empty (half-saved) file fails to load and leaves
                        // the current program in place
                        last_file_write_time_ = cur_time;
                        requestLoadFile(active_file_path_, false);
                    }
                }
            }
        }
    }

    // Top Control Row: STOP/REWIND | FEED HOLD | Set Next Line | Program Info
    const float btn_w = 110.0f;
    const float btn_h = 34.0f;

    // STOP / REWIND
    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_danger);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_danger_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_danger_act);
    ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_danger));
    if (ImGui::Button("STOP / REWIND", ImVec2(btn_w, btn_h))) {
        stopAndRewind(controller, telemetry);
    }
    ImGui::PopStyleColor(4);
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
    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_warn);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_warn_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_warn_act);
    ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_warn));
    if (ImGui::Button("FEED HOLD", ImVec2(btn_w, btn_h))) {
        controller.pauseJob();
    }
    ImGui::PopStyleColor(4);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Pause motion execution immediately (Feed Hold)");
    }

    ImGui::SameLine();

    // Set Next Line
    ImGui::BeginDisabled(is_running || gcode_lines.empty());
    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_neutral);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_neutral_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_neutral_act);
    if (ImGui::Button("Set Next Line", ImVec2(btn_w, btn_h))) {
        controller.setNextGCodeLine(selected_line_);
    }
    ImGui::PopStyleColor(3);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Set selected G-Code line as the next line to execute on Cycle Start");
    }

    // Go to line: selects the line and shows it, the program position does not change
    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (btn_h - ImGui::GetFrameHeight()) * 0.5f);
    ImGui::BeginDisabled(is_running || gcode_lines.empty());
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::InputTextWithHint("##GoToLine", "Go to line", go_to_buf_, sizeof(go_to_buf_),
                                 ImGuiInputTextFlags_EnterReturnsTrue) && go_to_buf_[0] != '\0') {
        if (const auto line = findGCodeLine(gcode_lines, go_to_buf_)) {
            selected_line_            = *line;
            scroll_to_line_           = *line;
            scroll_to_line_requested_ = true;
            go_to_error_.clear();
        } else {
            go_to_error_ = std::string("Line ") + go_to_buf_ + " not found";
        }
    }
    if (ImGui::IsItemEdited()) go_to_error_.clear();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Line number or N number, then Enter");
    }

    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    std::string display_filename;
    if (!loaded_filename.empty()) {
        display_filename = util::pathToUtf8(util::pathFromUtf8(loaded_filename).filename());
    }
    if (isLoading()) {
        // Short text on purpose: this line shares the row with the cycle buttons and does not wrap
        // load_job_.path is never written by the worker, so reading it here is safe
        const std::string loading_name = util::pathToUtf8(util::pathFromUtf8(load_job_.path).filename());
        ImGui::TextColored(theme::pal().text_warn, "Loading %s...", loading_name.c_str());
    } else if (!go_to_error_.empty()) {
        ImGui::TextColored(theme::pal().text_warn, "%s", go_to_error_.c_str());
    } else if (!load_notice_.empty()) {
        ImGui::TextColored(theme::pal().text_danger, "%s", load_notice_.c_str());
    } else {
        ImGui::TextDisabled("Next:#%04zu | %s (%zu pierces, %.0fmm)",
                            telemetry.active_gcode_line_index + 1,
                            display_filename.empty() ? "(none)" : display_filename.c_str(),
                            pierce_count,
                            total_cut_length_mm);
    }

    // Prominent Manual Tool Change (M6) Prompt Banner when waiting for operator tool change
    if (is_tool_change) {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, theme::pal().surface_warn);
        ImGui::BeginChild("##ManualToolChangeBanner", ImVec2(0.0f, 50.0f), true, ImGuiWindowFlags_NoScrollbar);
        
        const float btn_w = 220.0f;
        const float avail_w = ImGui::GetContentRegionAvail().x;
        const float text_w = std::max(120.0f, avail_w - btn_w - 14.0f);
        const bool script_active = (telemetry.script_running || telemetry.has_script_prompt);

        ImGui::BeginGroup();
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + text_w);
        if (script_active) {
            ImGui::TextColored(
                theme::pal().text_warn,
                "M6 TOOL CHANGE SCRIPT ACTIVE: T%d%s%s",
                telemetry.pending_tool_number,
                telemetry.pending_tool_description.empty() ? "" : " (",
                telemetry.pending_tool_description.empty() ? "" : (telemetry.pending_tool_description + ")").c_str());
            ImGui::TextDisabled("Follow operator dialog prompt or allow automated tool probe sequence to complete.");
        } else if (!telemetry.pending_tool_description.empty()) {
            ImGui::TextColored(
                theme::pal().text_warn,
                "MANUAL TOOL CHANGE (M6): Insert Tool T%d (%s)",
                telemetry.pending_tool_number,
                telemetry.pending_tool_description.c_str());
            ImGui::TextDisabled("Remove probe plate from workpiece, then press Cycle Start to begin milling.");
        } else {
            ImGui::TextColored(
                theme::pal().text_warn,
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
            ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_ok);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_ok_hov);
            ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));
            char resume_lbl[64];
            std::snprintf(resume_lbl, sizeof(resume_lbl), "TOOL T%d READY -> RESUME", telemetry.pending_tool_number);
            if (ImGui::Button(resume_lbl, ImVec2(btn_w, 32.0f))) {
                triggerCycleStart(controller, telemetry);
            }
            ImGui::PopStyleColor(3);
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
    }

    // Program check: errors and Cycle Start problems stop the run, warnings are informational
    updateProgramCheck(controller, program);
    renderProgramDiagnostics(is_running);

    // Whole-Line Snap-Scrolled G-Code Listing
    constexpr float kRowHeight = 22.0f;
    const float mdi_needed_h = ImGui::GetStyle().ItemSpacing.y + 6.0f + std::max(28.0f, ImGui::GetFrameHeight());
    const float avail_h = ImGui::GetContentRegionAvail().y - mdi_needed_h;
    const int visible_rows = std::max(3, static_cast<int>(std::floor((avail_h - 2.0f) / kRowHeight)));
    const float exact_scroll_h = static_cast<float>(visible_rows) * kRowHeight + 2.0f; // +2px for top/bottom 1px border

    ImFont* gcode_font = AppWindow::getGCodeFont();
    if (gcode_font) {
        ImGui::PushFont(gcode_font);
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
    // The scrollbar is handled inside BeginChild. On a long program its grab would be only a few pixels tall, and a
    // press just next to it would page instead of drag: use a larger grab, and let the bar follow the mouse from any press.
    {
        ImGuiStyle& style = ImGui::GetStyle();
        ImGuiIO& io = ImGui::GetIO();
        const float grab_min_size = style.GrabMinSize;
        const bool  scroll_by_page = io.ConfigScrollbarScrollByPage;
        style.GrabMinSize = 32.0f;
        io.ConfigScrollbarScrollByPage = false;
        ImGui::BeginChild("##GCodeScroll", ImVec2(0.0f, exact_scroll_h), true, ImGuiWindowFlags_NoScrollWithMouse);
        style.GrabMinSize = grab_min_size;
        io.ConfigScrollbarScrollByPage = scroll_by_page;
    }
    const bool scrollbar_held = ImGui::GetActiveID() == ImGui::GetWindowScrollbarID(ImGui::GetCurrentWindow(), ImGuiAxis_Y);

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
        if (!scrollbar_held && (active_y < cur_scr || active_y >= cur_scr + view_span - kRowHeight)) {
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

    if (scroll_to_line_requested_ && scroll_to_top_frames_ == 0 && !scroll_to_top_requested_) {
        scroll_to_line_requested_ = false;
        const float top_line = std::max(0.0f, static_cast<float>(scroll_to_line_) - std::floor(visible_rows * 0.35f));
        ImGui::SetScrollY(std::clamp(top_line * kRowHeight, 0.0f, ImGui::GetScrollMaxY()));
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

            // Row Background: Current Line Highlight vs. Selected Line vs. Faint Alternating Zebra
            ImU32 bg_col = (i % 2 == 0) ? theme::pal().gc_row_even : theme::pal().gc_row_odd;
            if (is_current_line) {
                bg_col = is_running ? theme::pal().gc_row_run : theme::pal().gc_row_current;
            } else if (is_selected_line) {
                bg_col = theme::pal().gc_row_selected;
            }
            dl->AddRectFilled(row_p0, row_p1, bg_col);

            // Left accent bar for Current Line or Selected Line
            if (is_current_line) {
                ImU32 accent = is_running ? theme::pal().gc_accent_run : theme::pal().gc_accent_current;
                dl->AddRectFilled(row_p0, ImVec2(row_p0.x + 4.0f, row_p1.y), accent);
            } else if (is_selected_line) {
                dl->AddRect(row_p0, row_p1, theme::pal().gc_selected_border);
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

            // Problem marker: a dot at the right end of the row, the messages in a tooltip (none once warnings are cleared)
            if (const auto [first, last] = issuesOnLine(i); first < last && !warnings_cleared_) {
                bool blocks = false;
                for (size_t k = first; k < last; ++k) {
                    if (check_->issues[k].kind != core::ProgramIssueKind::Warning) blocks = true;
                }
                const ImVec4 mc = blocks ? theme::pal().text_danger : theme::pal().text_warn;
                dl->AddCircleFilled(ImVec2(row_p1.x - 10.0f, row_p0.y + kRowHeight * 0.5f), 4.0f,
                                    ImGui::GetColorU32(mc));
                if (ImGui::IsItemHovered()) {
                    // The listing runs with zero padding and spacing: give the tooltip its own
                    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 6.0f));
                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 4.0f));
                    ImGui::BeginTooltip();
                    ImGui::PushFont(ImGui::GetIO().Fonts->Fonts[0]); // UI font, not the listing's monospace font
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
                    for (size_t k = first; k < last; ++k) {
                        const auto& iss = check_->issues[k];
                        ImGui::TextColored(iss.kind == core::ProgramIssueKind::Warning ? theme::pal().text_warn
                                                                                       : theme::pal().text_danger,
                                           "%s%s", iss.kind == core::ProgramIssueKind::StartBlocker ? "Cycle Start: " : "",
                                           iss.text.c_str());
                    }
                    ImGui::PopTextWrapPos();
                    ImGui::PopFont();
                    ImGui::EndTooltip();
                    ImGui::PopStyleVar(2);
                }
            }

            const ParsedLineDisplay pld = formatLine(gcode_lines[i], i);

            // Line Number Column (distinct color from G-code command)
            char num_buf[32];
            if (pld.line_num_str.empty()) {
                std::snprintf(num_buf, sizeof(num_buf), "%c", is_current_line ? '>' : ' ');
            } else {
                std::snprintf(num_buf, sizeof(num_buf), "%c%s", is_current_line ? '>' : ' ', pld.line_num_str.c_str());
            }
            ImU32 num_col = is_current_line
                                ? (is_running ? theme::pal().gc_num_run : theme::pal().gc_num_current)
                                : theme::pal().gc_num;
            dl->AddText(ImVec2(row_p0.x + 8.0f, row_p0.y + 3.0f), num_col, num_buf);

            // Subtle vertical column separator between line number and command
            dl->AddLine(ImVec2(row_p0.x + line_num_col_w_, row_p0.y),
                        ImVec2(row_p0.x + line_num_col_w_, row_p1.y),
                        theme::pal().gc_separator);

            // G-Code Command & Comment Column (split at ';' or '(' so Mach3 comments have a subtle color)
            const std::string& line_str = pld.command_str;
            size_t comment_pos = pld.comment_pos;
            ImU32 cmd_col = is_current_line
                                ? theme::pal().gc_text_current
                                : theme::pal().gc_text;
            ImU32 cmt_col = is_current_line
                                ? theme::pal().gc_comment_current
                                : theme::pal().gc_comment;

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

    // Bottom Control Row: Load File, Edit File, MDI Input Bar
    // Shifted 6px down so buttons and input field do not touch the G-code scroll window border
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6.0f);

    // Everything in one row that fits the panel: the input takes the room that is left. In a narrow panel the labels
    // get shorter and the last command moves into the input's tooltip, so nothing reaches past the right edge (a
    // wider row made the panel scroll sideways as soon as the input got the focus).
    const float row_h   = std::max(28.0f, ImGui::GetFrameHeight());
    const float sp      = ImGui::GetStyle().ItemSpacing.x;
    const float row_w   = ImGui::GetContentRegionAvail().x;
    auto btnW = [](const char* t) { return ImGui::CalcTextSize(t).x + 32.0f; };
    const std::string last_txt = telemetry.last_mdi_command.empty() ? std::string("Ready")
                                                                      : "Last: " + telemetry.last_mdi_command;
    constexpr float kMinInputW = 120.0f;
    const float wide_fixed = btnW("Load File") + btnW("Edit File") + ImGui::CalcTextSize("MDI G-Code:").x +
                             btnW("Send MDI") + sp * 4.0f;
    const bool  compact = row_w - wide_fixed < kMinInputW;
    const char* load_lbl = compact ? "Load" : "Load File";
    const char* edit_lbl = compact ? "Edit" : "Edit File";
    const char* mdi_lbl  = compact ? "MDI:" : "MDI G-Code:";
    const char* send_lbl = compact ? "Send" : "Send MDI";
    const float fixed_w = btnW(load_lbl) + btnW(edit_lbl) + ImGui::CalcTextSize(mdi_lbl).x + btnW(send_lbl) + sp * 4.0f;
    const float status_w = ImGui::CalcTextSize(last_txt.c_str()).x;
    const bool  show_status = row_w - fixed_w - sp - status_w >= 200.0f; // The status only where the input stays usable
    const float mdi_input_w = std::clamp(row_w - fixed_w - (show_status ? sp + status_w : 0.0f), 80.0f, 420.0f);

    if (ImGui::Button(load_lbl, ImVec2(btnW(load_lbl), row_h))) {
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
    std::error_code exists_ec;
    const bool has_open_file = !active_file_path_.empty() &&
                               std::filesystem::exists(util::pathFromUtf8(active_file_path_), exists_ec);
    ImGui::BeginDisabled(!has_open_file);
    if (ImGui::Button(edit_lbl, ImVec2(btnW(edit_lbl), row_h))) {
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
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (row_h - ImGui::GetFrameHeight()) * 0.5f);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(theme::pal().text_info, "%s", mdi_lbl);

    ImGui::SameLine();
    ImGui::BeginDisabled(is_estop);
    ImGui::SetNextItemWidth(mdi_input_w);
    bool send_mdi = ImGui::InputTextWithHint(
        "##MdiInput",
        "Enter manual G-Code",
        mdi_input_buf_,
        sizeof(mdi_input_buf_),
        ImGuiInputTextFlags_EnterReturnsTrue);
    if (!show_status && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("%s", last_txt.c_str());
    }
    ImGui::SameLine();
    if (ImGui::Button(send_lbl, ImVec2(btnW(send_lbl), row_h))) {
        send_mdi = true;
    }
    if (send_mdi && mdi_input_buf_[0] != '\0' && !is_estop) {
        if (controller.executeMdiCommand(mdi_input_buf_)) {
            mdi_input_buf_[0] = '\0';
        }
        ImGui::SetKeyboardFocusHere(-1);
    }
    ImGui::EndDisabled();
    if (show_status) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", last_txt.c_str());
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
    controller_ = &controller;
    pollBackgroundLoad();
    if (!queued_file_path_.empty()) {
        const std::string path = std::move(queued_file_path_);
        queued_file_path_.clear();
        requestLoadFile(path, queued_file_check_);
    }
    render(controller, telemetry, loaded_filename, pierce_count, total_cut_length_mm, [this]() {
        promptLoadFileDialog(on_file_approved_);
    });
}

GCodeProgramPanel::~GCodeProgramPanel() {
    if (load_thread_.joinable()) load_thread_.join(); // Closing during a load: let the parse finish
}

void GCodeProgramPanel::requestLoad(const std::string& path, std::string content) {
    LoadJob job;
    job.path    = path;
    job.content = std::move(content);
    startLoadJob(std::move(job));
}

void GCodeProgramPanel::requestLoadFile(const std::string& path, bool check_machine_type) {
    LoadJob job;
    job.path               = path;
    job.read_file          = true;
    job.check_machine_type = check_machine_type;
    startLoadJob(std::move(job));
}

void GCodeProgramPanel::startLoadJob(LoadJob job) {
    if (!controller_) return;
    if (load_thread_.joinable()) {
        has_next_load_ = true; // The newest request wins
        next_load_job_ = std::move(job);
        return;
    }
    load_job_      = std::move(job);
    load_ok_       = false;
    load_mismatch_ = false;
    load_error_.clear();
    load_done_.store(false);
    // The worker only touches load_job_ / load_ok_ / load_mismatch_ / load_check_ until it sets load_done_.
    // loadGCodeString() parses without holding the controller lock and refuses while a job runs.
    load_thread_ = std::thread([this, ctrl = controller_, machine_type = machine_type_] {
        auto& j = load_job_;
        bool readable = true;
        if (j.read_file) {
            std::ifstream ifs(util::pathFromUtf8(j.path)); // UTF-8 name
            if (ifs) {
                std::ostringstream ss;
                ss << ifs.rdbuf();
                j.content = ss.str();
            } else {
                readable = false;
                load_error_ = "Cannot open " + util::pathToUtf8(util::pathFromUtf8(j.path).filename());
            }
        }
        if (readable && j.check_machine_type) {
            load_check_    = gcode::GCodeParser::checkMachineCompatibility(j.content, machine_type);
            load_mismatch_ = load_check_.has_mismatch;
        }
        if (readable && !load_mismatch_) {
            load_ok_ = ctrl->loadGCodeString(j.content);
        }
        load_done_.store(true, std::memory_order_release);
    });
}

void GCodeProgramPanel::pollBackgroundLoad() {
    if (!load_thread_.joinable() || !load_done_.load(std::memory_order_acquire)) return;
    load_thread_.join();
    load_notice_ = load_error_;
    load_error_.clear();
    if (load_mismatch_) {
        // Same dialog as a mismatching file from Load File: "Load File Anyway" loads it without the check
        pending_file_path_    = load_job_.path;
        pending_file_content_ = std::move(load_job_.content);
        pending_mismatch_     = load_check_;
        show_mismatch_modal_  = true;
    } else if (load_ok_) {
        if (load_job_.read_file) {
            active_file_path_ = load_job_.path; // Watched for external edits like any loaded file
            std::error_code ec;
            last_file_write_time_ = std::filesystem::last_write_time(util::pathFromUtf8(active_file_path_), ec);
            has_file_write_time_  = !ec;
        }
        if (on_file_approved_) on_file_approved_(load_job_.path, load_job_.content);
        selected_line_           = 0;
        prev_active_line_        = 0;
        scroll_to_top_requested_ = true;
    }
    load_job_ = LoadJob{};
    if (has_next_load_) {
        has_next_load_ = false;
        startLoadJob(std::move(next_load_job_));
        next_load_job_ = LoadJob{};
    }
}

void GCodeProgramPanel::promptLoadFileDialog(
    const std::function<void(const std::string& path, const std::string& content)>& on_file_approved)
{
    on_file_approved_ = on_file_approved;
    std::string path = ui::openGCodeFileDialog();
    if (path.empty()) return;
    requestLoadFile(path, true); // Read, machine-type check and parse on the worker thread
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
        last_file_write_time_ = std::filesystem::last_write_time(util::pathFromUtf8(active_file_path_), ec);
        has_file_write_time_ = !ec;
        requestLoad(file_path, gcode_content);
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

    // Only show spindle warning modal if cycle start is hit while program is NOT started from the beginning.
    // BreakoutMill only: in BreakoutCut the output is the plasma torch or the laser, which must never be switched on
    // from here (the program's M3 / M4 does that where it should).
    if (!is_resuming && !is_start_from_beginning && !telemetry.torch_or_spindle_on && !telemetry.dry_run_mode &&
        controller.getConfigShared()->is_mill) {
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
        // Milling/motion in progress: Stop motion immediately, but keep G-code on the last active line
        controller.cancelScript();
        controller.stopJob();
        selected_line_           = telemetry.active_gcode_line_index;
        controller.setNextGCodeLine(telemetry.active_gcode_line_index);
        scroll_to_top_frames_    = 0;
        scroll_to_top_requested_ = false;
    } else {
        // Program is not running (already stopped or idle): Rewind back to line 1 and set line 1 as next line
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
    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_ok);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_ok_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_ok_act);
    ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));

    const bool is_surf_comp = telemetry.surface_compensation_active;
    if (is_surf_comp) {
        ImGui::PushStyleColor(ImGuiCol_Border, theme::pal().border_ok);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
    }

    std::string start_label_str;
    if (is_tool_change) {
        start_label_str = compact ? "RESUME (M6)" : "RESUME CYCLE (M6)";
    } else if (is_surf_comp) {
        start_label_str = compact ? "CYCLE START (comp.)" : "CYCLE START (surface comp.)";
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
    ImGui::PopStyleColor(4);
    ImGui::EndDisabled();
    if (is_estop && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Safety Lock: Cannot start cycle while E-Stop is active. Reset E-Stop first.");
    } else if (is_surf_comp && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Start G-Code execution with SURFACE COMPENSATION ACTIVE.\nAll Z movements contour to the probed height map.");
    } else if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Start G-Code execution / resume cut from current line");
    }

    ImGui::SameLine(0.0f, spacing);

    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_warn);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_warn_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_warn_act);
    ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_warn));
    if (ImGui::Button("FEED HOLD", ImVec2(btn_w1, btn_h))) {
        controller.pauseJob();
    }
    ImGui::PopStyleColor(4);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Pause motion execution immediately (Feed Hold)");
    }

    // Row 2: STOP / REWIND | Set Next Line
    ImGui::SetCursorPosX(start_x);
    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_danger);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_danger_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_danger_act);
    ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_danger));
    if (ImGui::Button("STOP / REWIND", ImVec2(btn_w0, btn_h))) {
        stopAndRewind(controller, telemetry);
    }
    ImGui::PopStyleColor(4);
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
    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_neutral);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_neutral_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_neutral_act);
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

        ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_ok);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_ok_hov);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));
        if (ImGui::Button(btn_start_label, ImVec2(200.0f, 36.0f))) {
            uint16_t duty = static_cast<uint16_t>((std::clamp(rpm, 1000, 24000) * 1000) / 24000);
            controller.setSpindlePwm(duty);
            controller.setTorchOrSpindle(true);
            controller.startJob();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(3);

        ImGui::SameLine();
        if (ImGui::Button("Start Without Spindle", ImVec2(180.0f, 36.0f))) {
            controller.startJob();
            ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_danger);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_danger));
        if (ImGui::Button("Cancel", ImVec2(100.0f, 36.0f))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(2);
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

    ImGui::PushStyleColor(ImGuiCol_PopupBg,          theme::pal().surface_inset);
    ImGui::PushStyleColor(ImGuiCol_ModalWindowDimBg, theme::pal().modal_dim);

    if (ImGui::BeginPopupModal("Machine Target Mismatch Warning##GCodeMismatchModal", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::Spacing();

        // Warning badge & Title
        ImGui::PushStyleColor(ImGuiCol_Text, theme::pal().text_warn);
        ImGui::SetWindowFontScale(1.15f);
        ImGui::Text("  [ ! ]  Machine Target Mismatch Warning");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        std::string filename_only = util::pathToUtf8(util::pathFromUtf8(pending_file_path_).filename());
        ImGui::TextWrapped("The selected G-Code file appears to be intended for a different machine process than the current controller profile:");

        ImGui::Spacing();

        // Framed details card (dynamically sized to fit all bullet points cleanly)
        const float card_h = 88.0f + static_cast<float>(pending_mismatch_.indicators.size()) * 22.0f;
        ImGui::PushStyleColor(ImGuiCol_ChildBg, theme::pal().surface_inset);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::BeginChild("##MismatchInfoCard", ImVec2(0.0f, card_h), true, 0);

        ImGui::TextDisabled("Selected File: ");
        ImGui::SameLine();
        ImGui::TextUnformatted(filename_only.c_str());

        ImGui::TextDisabled("Detected Target: ");
        ImGui::SameLine();
        ImGui::TextColored(theme::pal().text_warn, "%s", pending_mismatch_.detected_name.c_str());

        ImGui::TextDisabled("Active Profile: ");
        ImGui::SameLine();
        ImGui::TextColored(theme::pal().text_info, "%s", pending_mismatch_.current_name.c_str());

        if (!pending_mismatch_.indicators.empty()) {
            ImGui::Spacing();
            ImGui::TextDisabled("Identified Features:");
            for (const auto& ind : pending_mismatch_.indicators) {
                ImGui::Bullet();
                ImGui::SameLine();
                ImGui::TextUnformatted(ind.c_str());
            }
        }

        ImGui::EndChild();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();

        ImGui::Spacing();

        // High-contrast Warning Alert Banner with full auto-wrapping
        ImGui::PushStyleColor(ImGuiCol_ChildBg, theme::pal().surface_warn);
        ImGui::PushStyleColor(ImGuiCol_Border,  theme::pal().border_warn);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));

        ImGui::BeginChild("##WarningCalloutBanner", ImVec2(0.0f, 62.0f), true, 0);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::pal().text_warn); // High-contrast warm gold
        ImGui::TextWrapped("Executing this program on the wrong machine could trigger unexpected outputs (such as firing a torch instead of a spindle) or invalid Z-axis motion.");
        ImGui::PopStyleColor();
        ImGui::EndChild();

        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(2);

        ImGui::Spacing();
        ImGui::TextWrapped("Are you sure you want to load this file anyway?");
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Action Buttons
        ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_warn);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_warn_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_warn_act);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_warn));
        if (ImGui::Button("Load File Anyway", ImVec2(170.0f, 36.0f))) {
            active_file_path_ = pending_file_path_;
            std::error_code ec;
            last_file_write_time_ = std::filesystem::last_write_time(util::pathFromUtf8(active_file_path_), ec);
            has_file_write_time_ = !ec;
            requestLoad(pending_file_path_, std::move(pending_file_content_));
            pending_file_path_.clear();
            pending_file_content_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(4);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_neutral);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_neutral_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_neutral_act);
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
    const float mdi_needed_h = spacing_y + 6.0f + std::max(28.0f, ImGui::GetFrameHeight());
    // The program check card (height of the last frame) sits above the listing and must not take its rows
    const float non_scroll_h = pad_y + top_row_h + spacing_y + tool_change_h + diag_card_h_ + mdi_needed_h;

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

void GCodeProgramPanel::updateProgramCheck(core::IMotionController& controller,
                                           const std::shared_ptr<const gcode::ParseResult>& program) {
    check_ = controller.getProgramCheck();
    if (check_ && check_->program == program) return;
    // No check from the controller, or it belongs to another program (a load in between): parse results only
    if (!parse_only_check_ || parse_only_check_->program != program) {
        parse_only_check_ = std::make_shared<const core::ProgramCheckResult>(
            core::buildProgramCheck(program, nullptr, core::MachineCheckInput{}));
    }
    check_ = parse_only_check_;
}

std::pair<size_t, size_t> GCodeProgramPanel::issuesOnLine(size_t line) const {
    if (!check_ || check_->issues.empty()) return {0, 0};
    const auto& v = check_->issues;
    const auto lo = std::lower_bound(v.begin(), v.end(), line,
                                     [](const core::ProgramIssue& a, size_t l) { return a.line < l; });
    auto hi = lo;
    while (hi != v.end() && hi->line == line) ++hi;
    return {static_cast<size_t>(lo - v.begin()), static_cast<size_t>(hi - v.begin())};
}

void GCodeProgramPanel::renderProgramDiagnostics(bool is_running) {
    diag_card_h_ = 0.0f;
    warnings_cleared_ = false;
    if (!check_ || check_->issues.empty()) return;
    const auto& c = *check_;
    const bool blocks = (c.error_count + c.blocker_count) > 0;
    // Warnings can be cleared until the program is loaded again; errors and Cycle Start problems always show
    if (!blocks && c.program && cleared_warnings_program_.lock() == c.program) {
        warnings_cleared_ = true;
        return;
    }

    std::string counts;
    auto addCount = [&](size_t n, const char* one, const char* many) {
        if (n == 0) return;
        if (!counts.empty()) counts += ", ";
        counts += std::to_string(n) + " " + (n == 1 ? one : many);
    };
    addCount(c.error_count, "error", "errors");
    addCount(c.blocker_count, "Cycle Start problem", "Cycle Start problems");
    addCount(c.warning_count, "warning", "warnings");
    const char* title = (c.error_count > 0)   ? "Program cannot run"
                        : (c.blocker_count > 0) ? "Cycle Start is blocked"
                                                : "Program warnings";

    ImGui::PushStyleColor(ImGuiCol_ChildBg, blocks ? theme::pal().surface_danger : theme::pal().surface_warn);
    ImGui::BeginChild("##ProgramDiagnostics", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
    constexpr float kClearBtn = 28.0f;
    const float content_right = ImGui::GetWindowContentRegionMax().x;
    // Leave room for the clear icon at the right end of the title line
    ImGui::PushTextWrapPos(blocks ? 0.0f : content_right - kClearBtn - 8.0f);
    ImGui::TextColored(blocks ? theme::pal().text_danger : theme::pal().text_warn, "%s: %s.", title, counts.c_str());
    if (!is_running) {
        ImGui::SameLine();
        ImGui::TextDisabled("Click a message to show its line.");
    }
    ImGui::PopTextWrapPos();

    if (!blocks) {
        // Clear icon in the top-right corner, drawn over the card's padding so the title line keeps its height
        const ImVec2 resume = ImGui::GetCursorPos();
        ImGui::SetCursorPos(ImVec2(content_right - kClearBtn + 4.0f, std::max(0.0f, ImGui::GetStyle().WindowPadding.y - 6.0f)));
        const bool clicked = ImGui::InvisibleButton("##ClearWarnings", ImVec2(kClearBtn, kClearBtn));
        const bool hovered = ImGui::IsItemHovered();
        const ImVec2 b0 = ImGui::GetItemRectMin(), b1 = ImGui::GetItemRectMax();
        ImDrawList* cdl = ImGui::GetWindowDrawList();
        if (hovered) cdl->AddRectFilled(b0, b1, ImGui::GetColorU32(ImGuiCol_ButtonHovered), 4.0f);
        const ImVec2 mid((b0.x + b1.x) * 0.5f, (b0.y + b1.y) * 0.5f);
        const float arm = 5.5f;
        const ImU32 xc = ImGui::GetColorU32(theme::pal().text_warn);
        cdl->AddLine(ImVec2(mid.x - arm, mid.y - arm), ImVec2(mid.x + arm, mid.y + arm), xc, 2.0f);
        cdl->AddLine(ImVec2(mid.x - arm, mid.y + arm), ImVec2(mid.x + arm, mid.y - arm), xc, 2.0f);
        if (hovered) ImGui::SetTooltip("Clear the warnings. They show again when the program is loaded again.");
        if (clicked) cleared_warnings_program_ = c.program;
        ImGui::SetCursorPos(resume);
    }

    // The list scrolls once it is taller than four lines, so every message can be read. Its height is the
    // content height of the previous frame, capped (an auto-resizing child would not scroll).
    const float line_h = ImGui::GetTextLineHeightWithSpacing();
    const float max_h  = std::floor(line_h * 4.0f);
    const float list_h = std::clamp(diag_list_content_h_, line_h, max_h);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::BeginChild("##ProgramDiagnosticsList", ImVec2(0.0f, list_h), ImGuiChildFlags_None);

    constexpr size_t kMaxListed = 1000; // A garbage file can produce an error on every line
    const size_t listed = std::min(c.issues.size(), kMaxListed);
    const size_t last_line = c.issues.empty() ? 0 : c.issues.back().line + 1;
    char widest[32];
    std::snprintf(widest, sizeof(widest), "Line %zu", last_line);
    const float line_col_w = ImGui::CalcTextSize(widest).x + 10.0f;
    // Channel 0: hover highlight, channel 1: text (the highlight is known only after the text is laid out)
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->ChannelsSplit(2);
    dl->ChannelsSetCurrent(1);

    for (size_t k = 0; k < listed; ++k) {
        const auto& iss = c.issues[k];
        const ImVec4 col = (iss.kind == core::ProgramIssueKind::Warning) ? theme::pal().text_warn
                                                                         : theme::pal().text_danger;
        ImGui::PushID(static_cast<int>(k));
        const float row_x = ImGui::GetCursorPosX();
        ImGui::BeginGroup();
        ImGui::TextColored(col, "Line %zu", iss.line + 1);
        ImGui::SameLine(row_x + line_col_w);
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(col, "%s%s", iss.kind == core::ProgramIssueKind::StartBlocker ? "Cycle Start: " : "",
                           iss.text.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndGroup();
        if (!is_running && iss.line < (c.program ? c.program->lines.size() : 0) && ImGui::IsItemHovered()) {
            const ImVec2 r0 = ImGui::GetItemRectMin();
            const ImVec2 r1(ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x, ImGui::GetItemRectMax().y);
            dl->ChannelsSetCurrent(0);
            dl->AddRectFilled(r0, r1, ImGui::GetColorU32(ImGuiCol_HeaderHovered));
            dl->ChannelsSetCurrent(1);
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                selected_line_            = iss.line;
                scroll_to_line_           = iss.line;
                scroll_to_line_requested_ = true;
            }
        }
        ImGui::PopID();
    }
    dl->ChannelsMerge();
    if (c.issues.size() > listed) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%zu more messages are not listed. The first %zu are shown.", c.issues.size() - listed, listed);
        ImGui::PopTextWrapPos();
    }
    diag_list_content_h_ = ImGui::GetCursorPosY() - ImGui::GetStyle().ItemSpacing.y;

    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::EndChild();
    ImGui::PopStyleColor();
    diag_card_h_ = ImGui::GetItemRectSize().y + ImGui::GetStyle().ItemSpacing.y; // For calculateDesiredHeight()
}

GCodeProgramPanel::ParsedLineDisplay GCodeProgramPanel::formatLine(const std::string& raw, size_t index) const {
    ParsedLineDisplay pld;
    if (has_file_line_numbers_) {
        size_t end_digit = 0;
        const size_t d_idx = fileLineNumberDigits(raw, end_digit);
        if (d_idx != std::string::npos) {
            pld.line_num_str = "N" + raw.substr(d_idx, end_digit - d_idx);
            const size_t rest_idx = raw.find_first_not_of(" \t", end_digit);
            pld.command_str = (rest_idx != std::string::npos) ? raw.substr(rest_idx) : "";
        } else {
            pld.command_str = raw;
        }
    } else {
        char buf[24]; // fits any size_t
        std::snprintf(buf, sizeof(buf), "%04zu", index + 1);
        pld.line_num_str = buf;
        pld.command_str  = raw;
    }
    const size_t semi  = pld.command_str.find(';');
    const size_t paren = pld.command_str.find('(');
    pld.comment_pos = std::min(semi, paren);
    return pld;
}

void GCodeProgramPanel::updateLineNumberCache(const std::vector<std::string>& gcode_lines) {
    // A new program comes with a new lines buffer: rescan only then (no allocations, a few ms for 650k lines)
    if (gcode_lines.data() == line_scan_data_ && gcode_lines.size() == line_scan_count_) {
        return;
    }
    line_scan_data_  = gcode_lines.data();
    line_scan_count_ = gcode_lines.size();

    // Does the file number its lines (N100, N110, ...)? Then those numbers are shown instead of 0001, 0002, ...
    has_file_line_numbers_ = false;
    size_t max_num_chars = 4;
    for (const auto& line : gcode_lines) {
        size_t end_digit = 0;
        const size_t d_idx = fileLineNumberDigits(line, end_digit);
        if (d_idx != std::string::npos) {
            has_file_line_numbers_ = true;
            max_num_chars = std::max(max_num_chars, 1 + end_digit - d_idx); // "N" + digits
        }
    }
    if (!has_file_line_numbers_) {
        max_num_chars = std::max<size_t>(max_num_chars, std::to_string(gcode_lines.size()).size());
    }

    line_num_col_w_ = std::max(62.0f, static_cast<float>(max_num_chars + 2) * 9.5f + 16.0f);
}

} // namespace rnr::ui
