#pragma once
#include "core/controller/machine_config.h"
#include "core/controller/toolpath_segment.h"
#include <string>
#include <vector>
#include <map>
#include <memory>

namespace rnr::gcode {

struct ParseMessage {
    size_t      line = 0; // 0-based index into ParseResult::lines
    std::string text;
};

// One G2 / G3 block. Its chords are segments[first_segment .. first_segment + segment_count).
struct ArcInfo {
    std::array<double, 3> center_mm{};   // Work coordinates (after G68); the axis normal to the plane is unused
    uint8_t               plane = 17;    // 17 (XY), 18 (XZ), 19 (YZ)
    bool                  clockwise = true; // G2
    uint32_t              first_segment = 0;
    uint32_t              segment_count = 0;
};

// Source lines of a program. Copies of a ParseResult share them: a height-map warp or a re-parse copies the
// segments, never the text again. Only the parser appends (copy-on-write if the lines are shared).
class SharedLines {
public:
    size_t size() const noexcept { return v_->size(); }
    bool empty() const noexcept { return v_->empty(); }
    const std::string& operator[](size_t i) const { return (*v_)[i]; }
    const std::string* data() const noexcept { return v_->data(); }
    std::vector<std::string>::const_iterator begin() const { return v_->cbegin(); }
    std::vector<std::string>::const_iterator end() const { return v_->cend(); }
    // Read-only view as the vector (line lookup helpers take one)
    operator const std::vector<std::string>&() const noexcept { return *v_; }
    void push_back(std::string line) {
        if (v_.use_count() > 1) v_ = std::make_shared<std::vector<std::string>>(*v_);
        v_->push_back(std::move(line));
    }
private:
    std::shared_ptr<std::vector<std::string>> v_ = std::make_shared<std::vector<std::string>>();
};

struct ParseResult {
    SharedLines                        lines;
    std::vector<core::ToolpathSegment> segments;
    std::vector<ArcInfo>               arcs;
    std::array<double, 3>              bbox_min_mm           = {0.0, 0.0, 0.0};
    std::array<double, 3>              bbox_max_mm           = {0.0, 0.0, 0.0};
    double                             total_cut_length_mm   = 0.0;
    double                             total_rapid_length_mm = 0.0;
    size_t                             pierce_count          = 0;
    size_t                             tool_change_count     = 0;
    std::map<uint16_t, std::string>    tool_descriptions;

    // Diagnostics: a program with errors must not be executed (the controller refuses Cycle Start).
    std::vector<ParseMessage>          errors;
    std::vector<ParseMessage>          warnings;
    // Axes moved in machine coordinates (G53 / G28 / G30): these axes must be homed before running.
    uint8_t                            machine_axes_used     = 0;
    size_t                             spindle_on_without_m5_line = SIZE_MAX; // Last M3 / M4 with no M5 / M2 / M30 after it
    bool                               ends_with_program_end = false; // M2 / M30: all outputs off when the job ends

    bool hasErrors() const noexcept { return !errors.empty(); }
    std::string firstErrorText() const {
        if (errors.empty()) return {};
        return "Line " + std::to_string(errors.front().line + 1) + ": " + errors.front().text;
    }
};

struct ParseOptions {
    bool   dwell_p_milliseconds = false; // G4 P unit (Mach3 "Dwell in ms" option)
    double initial_feed_mm_min  = 0.0;   // Modal F carried in from previous MDI commands (0 = none)
    // How G43 is treated: only NotConfigured produces the "tool length offsets ignored" warning
    core::ToolLengthMode tool_length_mode = core::ToolLengthMode::NotConfigured;
};

struct GCodeWord {
    char   letter = 0;
    double value  = 0.0;
};

// Splits one block into upper-case letter/number words. Comments are returned in `comment`.
// Returns false (with `error`) for syntax the parser does not support (expressions, variables).
bool tokenizeBlock(const std::string& raw_line, std::vector<GCodeWord>& words, std::string* comment, std::string* error);

enum class MachineType : uint8_t {
    Unknown = 0,
    Mill    = 1,
    Plasma  = 2,
    Laser   = 3
};

struct MachineTypeCheckResult {
    bool has_mismatch = false;
    MachineType detected_type = MachineType::Unknown;
    int confidence_score = 0;
    std::string detected_name;
    std::string current_name;
    std::vector<std::string> indicators;
};

class GCodeParser {
public:
    // Mach3-style G-code parser:
    // Supports G0/G1/G2/G3 (IJK & R arcs in G17/G18/G19, G90.1/G91.1), G4, G20/G21/G70/G71,
    // G28/G30/G53 (machine coordinates), G40/G49, G54..G59, Canned Drilling Cycles (G73/G81/G82/G83/G80/G98/G99),
    // G68 A B R [I] / G69 (Mach3 coordinate rotation), G90/G91, G92/G92.1, M0/M1 (program stop),
    // M3/M4/M5/M6 (T#)/M7/M8/M9/M2/M30, F and S words.
    // Anything else that would change the toolpath (G41/G42, G93/G95, other cycles, expressions,
    // subprograms, invalid arcs, feed moves without F) is reported in ParseResult::errors.
    // G43 H<n> / G49 are recorded per segment (ToolpathSegment::tlo_tool); the controller applies them
    // in tool-table mode. With ToolLengthMode::NotConfigured, G43 also produces a warning.
    // `work_offset_mm` is only used to preview machine-coordinate moves (G53/G28) in work coordinates;
    // the controller re-maps those moves with the live offsets at run time.
    static ParseResult parse(
        const std::string& gcode_text,
        const std::array<double, 4>& start_pos_mm = {0.0, 0.0, 0.0, 0.0},
        double arc_chord_tolerance_mm = 0.25,
        const std::array<double, 4>& work_offset_mm = {0.0, 0.0, 0.0, 0.0},
        const ParseOptions& options = {});

    // Inspects G-code content for characteristics indicative of Mill, Plasma, or Laser operations
    static MachineType detectTargetMachine(
        const std::string& gcode_text,
        int* out_score = nullptr,
        std::vector<std::string>* out_indicators = nullptr);

    // Compares detected machine type against the current machine environment and returns compatibility result
    static MachineTypeCheckResult checkMachineCompatibility(
        const std::string& gcode_text,
        MachineType current_machine);

    // Built-in sample Plasma G-code program (Gusset Bracket with 2 bolt holes + outer contour)
    static std::string samplePlasmaBracketGCode();

    // Built-in sample Laser G-code program (90x90mm coaster: raster fill, rose curve and ring engraving + outline cut)
    static std::string sampleLaserCoasterGCode();

    // Built-in sample Mach3 ATC Mill G-code program (Multi-tool T1 Endmill Pocket/Contour + M6 T2 G83 Peck Drilling)
    static std::string sampleMillAtcGCode();
};

} // namespace rnr::gcode
