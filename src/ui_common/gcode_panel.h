#pragma once
#include "core/controller/motion_controller.h"
#include "core/gcode/gcode_parser.h"
#include <string>
#include <functional>
#include <filesystem>
#include <chrono>

namespace rnr::ui {

class GCodeProgramPanel {
public:
    GCodeProgramPanel() = default;

    // Configure active machine type (Mill, Plasma, Laser) for compatibility checks
    void setMachineType(gcode::MachineType type) { machine_type_ = type; }
    gcode::MachineType getMachineType() const { return machine_type_; }

    // Render 2x2 Grid of Cycle Control Buttons (START CUT / FEED HOLD / STOP / Set Next Line)
    void renderCycleControlGrid(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        float target_w = -1.0f,
        float offset_x = -1.0f);

    // Render Load File, Snap-Scrolled G-Code View, and MDI Bar (legacy callback)
    void render(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        const std::string& loaded_filename,
        size_t pierce_count,
        double total_cut_length_mm,
        const std::function<void()>& on_load_file_clicked);

    // Render Load File, Snap-Scrolled G-Code View, and MDI Bar with automatic compatibility checking
    void render(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        const std::string& loaded_filename,
        size_t pierce_count,
        double total_cut_length_mm,
        const std::function<void(const std::string& path, const std::string& content)>& on_file_approved);

    // Check a file for compatibility, and either load directly or trigger mismatch modal
    void checkAndPromptGCodeFile(
        const std::string& file_path,
        const std::string& gcode_content,
        const std::function<void(const std::string& path, const std::string& content)>& on_file_approved);

    // Open native file dialog with automatic machine-type compatibility check
    void promptLoadFileDialog(
        const std::function<void(const std::string& path, const std::string& content)>& on_file_approved);

    // Computes the exact tight height required by this panel so that there is no extra unused space below the MDI row.
    // If exact_rows > 0, calculates height for exactly that number of visible G-code rows.
    float calculateDesiredHeight(float max_available_h, bool is_tool_change, int exact_rows = -1) const;

    // Trigger cycle start with spindle check
    void triggerCycleStart(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry);

    // Stop motion and rewind program back to line 1
    void stopAndRewind(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry);

    // Render spindle check warning popup modal
    void renderSpindleModal(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry);

    // Render machine mismatch warning popup modal
    void renderMismatchModal();

private:
    size_t selected_line_               = 0;
    size_t prev_active_line_            = 0;
    bool   scroll_to_top_requested_     = false;
    int    scroll_to_top_frames_        = 0;
    bool   show_spindle_warning_modal_  = false;
    char   mdi_input_buf_[256]{};

    // Machine type compatibility check state
    gcode::MachineType machine_type_    = gcode::MachineType::Mill;
    bool   show_mismatch_modal_         = false;
    std::string pending_file_path_;
    std::string pending_file_content_;
    gcode::MachineTypeCheckResult pending_mismatch_;
    std::function<void(const std::string&, const std::string&)> on_file_approved_;

    // Active file tracking for auto-reload on external edit
    std::string active_file_path_;
    std::filesystem::file_time_type last_file_write_time_{};
    bool has_file_write_time_ = false;
    std::chrono::steady_clock::time_point last_file_poll_time_{};

    // Formatted line display cache (supports file-provided N100/N110 numbering)
    struct ParsedLineDisplay {
        std::string line_num_str;
        std::string command_str;
        size_t      comment_pos = std::string::npos;
    };
    std::vector<ParsedLineDisplay> parsed_display_lines_;
    size_t last_parsed_lines_count_ = 0;
    std::string last_parsed_filename_;
    float line_num_col_w_ = 62.0f;
    void updateLineNumberCache(const std::vector<std::string>& gcode_lines, const std::string& filename);
};

} // namespace rnr::ui
