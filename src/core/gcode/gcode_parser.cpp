#include "core/gcode/gcode_parser.h"
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <algorithm>

namespace rnr::gcode {

namespace {
constexpr double kPi = 3.14159265358979323846;
// Maximum chord deviation from the true arc when tessellating G2/G3 (mm)
constexpr double kArcSagittaMm = 0.005;

double calcDist4D(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    double dx = b[0] - a[0];
    double dy = b[1] - a[1];
    double dz = b[2] - a[2];
    double da = b[3] - a[3];
    return std::sqrt(dx * dx + dy * dy + dz * dz + da * da);
}

void expandBBox(std::array<double, 3>& bmin, std::array<double, 3>& bmax, const std::array<double, 4>& pt, bool& first_pt) {
    if (first_pt) {
        bmin = {pt[0], pt[1], pt[2]};
        bmax = {pt[0], pt[1], pt[2]};
        first_pt = false;
    } else {
        for (size_t i = 0; i < 3; ++i) {
            bmin[i] = std::min(bmin[i], pt[i]);
            bmax[i] = std::max(bmax[i], pt[i]);
        }
    }
}

std::string trimString(const std::string& s) {
    size_t a = 0;
    while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    size_t b = s.size();
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

// Extract tool number from header comments like "(T1 D=6. CR=0. - flat end mill)" or "(TOOL 2: 3mm Drill)"
void tryExtractToolHeaderComment(const std::string& comment_str, std::map<uint16_t, std::string>& tool_map) {
    std::string c = trimString(comment_str);
    if (c.empty()) return;
    std::string upper;
    upper.reserve(c.size());
    for (char ch : c) upper.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));

    for (size_t i = 0; i < upper.size(); ++i) {
        if (upper[i] == 'T' && (i == 0 || !std::isalpha(static_cast<unsigned char>(upper[i - 1])))) {
            size_t j = i + 1;
            if (upper.compare(i, 4, "TOOL") == 0) {
                j = i + 4;
                while (j < upper.size() && (upper[j] == ' ' || upper[j] == '#')) ++j;
            }
            if (j < upper.size() && std::isdigit(static_cast<unsigned char>(upper[j]))) {
                char* endp = nullptr;
                long tnum = std::strtol(upper.c_str() + j, &endp, 10);
                if (tnum >= 1 && tnum <= 255) {
                    tool_map[static_cast<uint16_t>(tnum)] = c;
                    return;
                }
            }
        }
    }
}

std::string formatGCode(char letter, double value) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%c%g", letter, value);
    return buf;
}
// Reads a plain decimal number ([+-]digits[.digits]) at `pos` and advances past it. Not strtod: that would read
// "G0X1" as the hexadecimal number 0X1 and "X1E2" as 100, so words written without spaces would merge.
bool scanNumber(const std::string& s, size_t& pos, double& out) {
    size_t i = pos;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
    bool any_digit = false;
    while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { any_digit = true; ++i; }
    if (i < s.size() && s[i] == '.') {
        ++i;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { any_digit = true; ++i; }
    }
    if (!any_digit) return false;
    // Convert only the scanned characters, so strtod cannot read further
    out = std::strtod(s.substr(pos, i - pos).c_str(), nullptr);
    pos = i;
    return true;
}
} // namespace

bool tokenizeBlock(const std::string& raw_line, std::vector<GCodeWord>& words, std::string* comment, std::string* error) {
    words.clear();
    thread_local std::string clean; // Reused: tokenizing runs once per program line
    clean.clear();
    std::string line_comment;
    clean.reserve(raw_line.size());
    bool in_paren = false;
    for (size_t idx = 0; idx < raw_line.size(); ++idx) {
        char c = raw_line[idx];
        if (c == '\r') continue;
        if (!in_paren && c == ';') {
            line_comment = trimString(raw_line.substr(idx + 1));
            break;
        }
        if (c == '(') {
            in_paren = true;
            continue;
        }
        if (c == ')') {
            in_paren = false;
            continue;
        }
        if (in_paren) {
            line_comment.push_back(c);
        } else {
            clean.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
        }
    }
    if (comment) *comment = line_comment;

    // Ignore % tape delimiters
    if (!clean.empty() && clean[0] == '%') {
        return true;
    }

    size_t pos = 0;
    while (pos < clean.size()) {
        const char c = clean[pos];
        if (std::isspace(static_cast<unsigned char>(c)) || c == '/') {
            ++pos;
            continue;
        }
        if (c == '#' || c == '[' || c == ']' || c == '=') {
            if (error) *error = "Parameters/expressions (#, [ ]) are not supported";
            return false;
        }
        if (!std::isalpha(static_cast<unsigned char>(c))) {
            if (error) *error = std::string("Unexpected character '") + c + "'";
            return false;
        }
        ++pos;
        while (pos < clean.size() && std::isspace(static_cast<unsigned char>(clean[pos]))) ++pos;
        if (pos < clean.size() && (clean[pos] == '#' || clean[pos] == '[')) {
            if (error) *error = "Parameters/expressions (#, [ ]) are not supported";
            return false;
        }
        double val = 0.0;
        if (!scanNumber(clean, pos, val)) {
            if (error) *error = std::string("Letter '") + c + "' has no numeric value";
            return false;
        }
        words.push_back({c, val});
    }
    return true;
}

namespace {

// Variables that only describe the current block: reset for every line
struct LineState {
    size_t line_idx = 0;
    bool   is_dwell       = false;
    double dwell_p        = -1.0;
    bool   is_g92         = false;
    bool   is_g92_clear   = false;
    bool   is_g28         = false;
    bool   is_g53_machine = false;
    bool   is_tool_change = false;
    bool   is_program_stop = false;
    bool   ends_program   = false;
    bool   motion_word_on_line = false;
    int h_word = -1;
    bool is_g64 = false;
    bool is_g68 = false;
    bool has_xyz = false;
    uint8_t axes_given = 0;
    std::array<double, 4> target_pos{};
    std::array<double, 4> raw_axis{};
    bool has_ijk = false;
    bool has_r   = false;
    double arc_i = 0.0, arc_j = 0.0, arc_k = 0.0, arc_r = 0.0;
};

// GCodeParser::parse(): one object per program, one member function per kind of block content
class Parser : private LineState {
public:
    Parser(const std::array<double, 4>& start_pos_mm, double arc_chord_tolerance_mm,
           const std::array<double, 4>& work_offset_mm, const ParseOptions& options)
        : arc_chord_tolerance_mm(arc_chord_tolerance_mm), work_offset_mm(work_offset_mm), options(options) {
        cur_pos       = start_pos_mm;
        canned_init_z = cur_pos[2];
        feed_mm_min   = options.initial_feed_mm_min;
    }

    ParseResult run(const std::string& gcode_text);

private:
    // Steps of one line, in order. A step returning true has finished the line.
    bool beginLine(const std::string& raw_line);
    void scanLineFlags();
    void readModalWords();
    void emitToolChange();
    bool readAxisWords();
    void axisWord(size_t a, double v);
    bool handleCoordinateCommands();
    void handleDwell();
    bool handleMotion();
    bool emitCannedCycle();
    void emitArc();
    void endBlock();

    void rotateXY(double& x, double& y, bool inverse);
    void addError(size_t line, const std::string& text);
    void addWarning(size_t line, const std::string& text);
    core::ToolpathSegment baseSegment(core::SegmentType seg_type, size_t line_idx);
    void emitLinearSegment(const std::array<double, 4>& p0, const std::array<double, 4>& p1, core::SegmentType seg_type,
                           size_t line_idx, double seg_feed, bool keep_zero_length = false);
    void emitBlockMove(core::SegmentType seg_type);
    void releaseHeld(uint8_t axes);
    uint32_t heldTarget();
    void emitMachineMove(const std::array<double, 4>& machine_target, uint8_t machine_axes, bool rapid, size_t line_idx);

    const double arc_chord_tolerance_mm;
    const std::array<double, 4> work_offset_mm;
    const ParseOptions options;
    ParseResult result{};
    // Reused for every line (no allocations per line once they have grown)
    std::vector<GCodeWord> words;
    std::string line_comment;
    std::string tok_error;

    std::array<double, 4> cur_pos{}; // Work coordinates of the active WCS (G92 applied)
    std::array<double, 4> g92_shift{};            // Work = programmed + g92_shift
    bool   absolute_xyz    = true;  // G90 default (G91 = incremental XYZ)
    bool   ij_incremental  = true;  // G91.1 default in Mach3 (G90.1 = absolute IJ arc centers)
    double unit_scale      = 1.0;   // G21 (mm) default; G20 = 25.4
    int    active_plane    = 17;    // 17=XY (G17), 18=XZ (G18), 19=YZ (G19)
    int    active_g_motion = 0;     // 0=G0, 1=G1, 2=G2, 3=G3, 73=G73, 81=G81, 82=G82, 83=G83
    bool   canned_return_r = false; // false=G98 (return to initial Z), true=G99 (return to R plane)
    double canned_r_mm     = 5.0;
    double canned_q_mm     = 2.0;
    double canned_dwell_p  = -1.0;  // G82 P, kept for the following holes of the cycle
    bool   canned_q_given  = false; // A Q word was given for the current G73 / G83 cycle
    bool   warned_no_q     = false; // Warned once per cycle
    double canned_z_mm     = 0.0;
    double canned_init_z   = 0.0;
    bool   program_ended   = false;
    bool   warned_g43      = false;
    bool   warned_after_end = false;
    double   feed_mm_min       = 0.0; // 0 = no F programmed yet
    bool     torch_on          = false;
    size_t   spindle_on_line   = SIZE_MAX; // Last M3 / M4 not yet followed by M5
    uint8_t  out_mask          = 0;
    uint16_t current_tool      = 1;
    uint16_t selected_tool     = 1;
    uint16_t spindle_rpm       = 0; // 0 = not programmed (controller keeps the operator's speed)
    bool     spindle_s_set     = false; // An S word was programmed (S0 included): laser S0 = beam off
    uint8_t  current_wcs       = 0; // 0=G54 .. 5=G59
    uint16_t tlo_tool          = core::kTloInherit; // G43 H<n> in effect (0 = G49)
    bool   rot_active = false;
    double rot_cx = 0.0, rot_cy = 0.0, rot_deg = 0.0;
    // Axes a G53 / G28 / G30 left at a machine coordinate, until the program names them again in work coordinates.
    // Their work position is only known when the job runs (offsets and tool lengths may change after loading), so
    // the moves in between carry a MachineTarget that keeps them there instead of the parse-time preview.
    uint8_t               held_axes = 0;
    std::array<double, 4> held_mm{};
    uint32_t              held_target = core::kNoMachineTarget; // Table entry of the current held state (on first use)
    double rot_cos = 1.0, rot_sin = 0.0;
    bool     exact_stop        = false; // G61 / G61.1 in effect (G64 = constant velocity)
    double   corner_tolerance_mm = 0.0; // G64 P in effect, 0 = the machine setting
    std::array<double, 3> cut_bmin = {0.0, 0.0, 0.0}, cut_bmax = {0.0, 0.0, 0.0};
    std::array<double, 3> all_bmin = {0.0, 0.0, 0.0}, all_bmax = {0.0, 0.0, 0.0};
    bool first_cut_pt = true;
    bool first_all_pt = true;
};

void Parser::rotateXY(double& x, double& y, bool inverse) {
    const double s = inverse ? -rot_sin : rot_sin;
    const double dx = x - rot_cx, dy = y - rot_cy;
    x = rot_cx + dx * rot_cos - dy * s;
    y = rot_cy + dx * s + dy * rot_cos;
}

void Parser::addError(size_t line, const std::string& text) {
    result.errors.push_back({line, text});
}

void Parser::addWarning(size_t line, const std::string& text) {
    result.warnings.push_back({line, text});
}

core::ToolpathSegment Parser::baseSegment(core::SegmentType seg_type, size_t line_idx) {
    core::ToolpathSegment seg{};
    seg.type            = seg_type;
    seg.segment_index   = result.segments.size();
    seg.gcode_line      = line_idx;
    seg.start_xyza_mm   = cur_pos;
    seg.end_xyza_mm     = cur_pos;
    seg.feed_mm_min     = feed_mm_min;
    seg.torch_active    = torch_on;
    seg.outputs_mask    = out_mask;
    seg.tool_number     = current_tool;
    seg.spindle_rpm     = spindle_rpm;
    seg.spindle_s_set   = spindle_s_set;
    seg.workspace_index = current_wcs;
    seg.tlo_tool        = tlo_tool;
    seg.exact_stop      = exact_stop;
    seg.corner_tolerance_mm = corner_tolerance_mm;
    return seg;
}

void Parser::releaseHeld(uint8_t axes) {
    if ((held_axes & axes) == 0) return;
    held_axes = static_cast<uint8_t>(held_axes & ~axes);
    held_target = core::kNoMachineTarget;
}

uint32_t Parser::heldTarget() {
    if (held_axes == 0) return core::kNoMachineTarget;
    if (held_target == core::kNoMachineTarget) {
        held_target = static_cast<uint32_t>(result.machine_targets.size());
        result.machine_targets.push_back(core::MachineTarget{held_mm, held_axes});
    }
    return held_target;
}

// G0 / G1 of this block to target_pos. A held axis named in G90 is back in work coordinates; in G91 it moves by the
// increment from where the machine move left it.
void Parser::emitBlockMove(core::SegmentType seg_type) {
    uint8_t released = 0;
    for (size_t a = 0; a < 4; ++a) {
        const uint8_t bit = static_cast<uint8_t>(1U << a);
        if (!(axes_given & bit) || !(held_axes & bit)) continue;
        if (absolute_xyz || rot_active) {
            released |= bit;
        } else {
            held_mm[a] += raw_axis[a];
            held_target = core::kNoMachineTarget;
        }
    }
    releaseHeld(released);
    // Kept even when zero length at parse time: from the machine position the move may be long when the job runs
    emitLinearSegment(cur_pos, target_pos, seg_type, line_idx, feed_mm_min, released != 0);
    cur_pos = target_pos;
}

void Parser::emitLinearSegment(const std::array<double, 4>& p0, const std::array<double, 4>& p1,
                               core::SegmentType seg_type, size_t line_idx, double seg_feed, bool keep_zero_length) {
    double len = calcDist4D(p0, p1);
    if (len <= 1e-6 && !keep_zero_length) return;

    core::ToolpathSegment seg = baseSegment(seg_type, line_idx);
    seg.start_xyza_mm = p0;
    seg.end_xyza_mm   = p1;
    seg.feed_mm_min   = seg_feed;
    seg.length_mm     = len;
    seg.torch_active  = (seg_type != core::SegmentType::RapidMove) && torch_on;
    seg.machine_target = heldTarget();

    if (seg_type == core::SegmentType::RapidMove) {
        result.total_rapid_length_mm += len;
    } else {
        result.total_cut_length_mm += len;
        expandBBox(cut_bmin, cut_bmax, p0, first_cut_pt);
        expandBBox(cut_bmin, cut_bmax, p1, first_cut_pt);
    }
    expandBBox(all_bmin, all_bmax, p0, first_all_pt);
    expandBBox(all_bmin, all_bmax, p1, first_all_pt);
    result.segments.push_back(seg);
}

void Parser::emitMachineMove(const std::array<double, 4>& machine_target, uint8_t machine_axes, bool rapid, size_t line_idx) {
    std::array<double, 4> preview = cur_pos;
    for (size_t a = 0; a < 4; ++a) {
        if ((machine_axes >> a) & 1U) preview[a] = machine_target[a] - work_offset_mm[a];
    }
    core::ToolpathSegment seg = baseSegment(rapid ? core::SegmentType::RapidMove : core::SegmentType::CutLinear, line_idx);
    seg.end_xyza_mm       = preview;
    seg.feed_mm_min       = feed_mm_min;
    seg.length_mm         = calcDist4D(cur_pos, preview);
    seg.torch_active      = false;
    seg.outputs_mask      = out_mask & ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
    // Axes held by an earlier machine move and not named here stay held: they are part of this target too
    for (size_t a = 0; a < 4; ++a) {
        if ((machine_axes >> a) & 1U) held_mm[a] = machine_target[a];
    }
    held_axes = static_cast<uint8_t>(held_axes | machine_axes);
    held_target = core::kNoMachineTarget;
    seg.machine_target    = heldTarget();
    result.machine_axes_used |= machine_axes;
    // Kept even when zero length at parse time: the live offsets may differ when the job runs
    result.segments.push_back(seg);
    result.total_rapid_length_mm += seg.length_mm;
    expandBBox(all_bmin, all_bmax, preview, first_all_pt);
    cur_pos = preview;
}

void Parser::axisWord(size_t a, double v) {
    raw_axis[a] = v * unit_scale;
    target_pos[a] = absolute_xyz ? (raw_axis[a] + g92_shift[a]) : (target_pos[a] + raw_axis[a]);
    axes_given |= static_cast<uint8_t>(1U << a);
    has_xyz = true;
}

bool Parser::beginLine(const std::string& raw_line_in) {
    std::string raw_line = raw_line_in;
    static_cast<LineState&>(*this) = LineState{};
    if (!raw_line.empty() && raw_line.back() == '\r') {
        raw_line.pop_back();
    }
    line_idx = result.lines.size();
    result.lines.push_back(raw_line);

    line_comment.clear();
    tok_error.clear();
    if (!tokenizeBlock(raw_line, words, &line_comment, &tok_error)) {
        if (!program_ended) addError(line_idx, tok_error);
        return false; // Nothing to run on this line
    }
    if (!line_comment.empty()) {
        tryExtractToolHeaderComment(line_comment, result.tool_descriptions);
    }
    if (words.empty()) return false; // Nothing to run on this line

    if (program_ended) {
        if (!warned_after_end) {
            addWarning(line_idx, "Blocks after M2/M30 are ignored");
            warned_after_end = true;
        }
        return false; // Nothing to run on this line
    }

    return true;
}

void Parser::scanLineFlags() {
    for (const auto& w : words) {
        if (w.letter == 'H') h_word = static_cast<int>(std::lround(w.value));
        if (w.letter == 'G' && static_cast<int>(std::round(w.value * 10.0)) == 640) is_g64 = true;
        if (w.letter == 'G' && static_cast<int>(std::round(w.value * 10.0)) == 680) is_g68 = true;
    }

}

void Parser::readModalWords() {
    // Pass 1: modal setup G-codes, M-codes, Feed (F), Spindle (S), Tool (T), Dwell (P)
    // Evaluating G90/G91/G20/G21 first ensures coordinates on the same line use the right mode.
    for (const auto& w : words) {
        if (w.letter == 'G') {
            // Multiply by 10 to distinguish G90 vs G90.1 and G91 vs G91.1
            const int gx10 = static_cast<int>(std::round(w.value * 10.0));
            switch (gx10) {
                case 0:   // G0 / G00
                case 10:  // G1 / G01
                case 20:  // G2 / G02
                case 30:  // G3 / G03
                    active_g_motion = gx10 / 10;
                    motion_word_on_line = true;
                    break;
                case 40:  // G4 / G04 Dwell
                    is_dwell = true;
                    break;
                case 170: active_plane = 17; break; // G17 XY Plane
                case 180: active_plane = 18; break; // G18 XZ Plane
                case 190: active_plane = 19; break; // G19 YZ Plane
                case 200: // G20 Inches
                case 700: // G70 Inches
                    unit_scale = 25.4;
                    break;
                case 210: // G21 Millimeters
                case 710: // G71 Millimeters
                    unit_scale = 1.0;
                    break;
                case 280: // G28 Return to Home
                    is_g28 = true;
                    break;
                case 300: // G30 Return to Secondary Home (no separate G30 position is stored)
                    is_g28 = true;
                    addWarning(line_idx, "G30 is executed like G28 (machine zero); no secondary reference position is configured");
                    break;
                case 530: // G53 Move in Machine Coordinates (non-modal)
                    is_g53_machine = true;
                    break;
                case 730: // G73 High-Speed Peck Drill
                case 810: // G81 Simple Drill
                case 820: // G82 Spot / Dwell Drill
                case 830: // G83 Deep Peck Drill
                    active_g_motion = gx10 / 10;
                    canned_init_z   = cur_pos[2];
                    motion_word_on_line = true;
                    break;
                case 800: // G80 Cancel Canned Cycle
                    if (active_g_motion >= 73 && active_g_motion <= 89) {
                        active_g_motion = 0;
                    }
                    break;
                case 900: absolute_xyz = true;    break; // G90 Absolute XYZ Distance Mode
                case 910: absolute_xyz = false;   break; // G91 Incremental XYZ Distance Mode
                case 901: ij_incremental = false; break; // G90.1 Absolute IJK Arc Center Mode
                case 911: ij_incremental = true;  break; // G91.1 Incremental IJK Arc Center Mode
                case 920: // G92 Coordinate Offset
                    is_g92 = true;
                    break;
                case 921: // G92.1 Cancel G92 offsets
                case 922: // G92.2 Suspend G92 offsets
                    is_g92_clear = true;
                    break;
                case 980: canned_return_r = false; break; // G98 Canned Cycle Return to Initial Z
                case 990: canned_return_r = true;  break; // G99 Canned Cycle Return to R Level
                case 540: // G54 .. G59 Work Coordinate Systems
                case 550:
                case 560:
                case 570:
                case 580:
                case 590:
                    current_wcs = static_cast<uint8_t>((gx10 - 540) / 10);
                    break;
                case 430: { // G43 H<n> Tool Length Offset (no H word: the tool in the spindle)
                    const int h = (h_word >= 0) ? h_word : static_cast<int>(current_tool);
                    if (h < 0 || h > 255) {
                        addError(line_idx, "G43 H value out of range (H0..H255)");
                    } else {
                        tlo_tool = static_cast<uint16_t>(h);
                    }
                    if (options.tool_length_mode == core::ToolLengthMode::NotConfigured && !warned_g43) {
                        addWarning(line_idx, "G43 tool length offsets are ignored: touch off Z for every tool, or enable "
                                             "the tool table (Settings > Tool Table)");
                        warned_g43 = true;
                    }
                    break;
                }
                case 440:
                    addError(line_idx, "G44 (negative tool length offset) is not supported - use G43");
                    break;
                case 431:
                case 432:
                    addError(line_idx, formatGCode('G', w.value) + " (dynamic tool length offset) is not supported");
                    break;
                case 490: // G49 Tool length offset cancel
                    tlo_tool = 0;
                    break;
                case 610: // G61 Exact stop
                case 611: // G61.1
                    exact_stop = true;
                    break;
                case 640: // G64 Constant velocity (P word handled below)
                    exact_stop = false;
                    corner_tolerance_mm = 0.0;
                    break;
                case 150: // G15 Polar coordinates off
                case 400: // G40 Cutter compensation off
                case 500: // G50 Scaling off
                case 680: // G68 Coordinate rotation (applied after the words below are read)
                    break;
                case 690: // G69 Rotation off
                    rot_active = false;
                    rot_cx = rot_cy = rot_deg = 0.0;
                    rot_cos = 1.0;
                    rot_sin = 0.0;
                    break;
                case 940: // G94 Feed per minute
                    break;
                case 410:
                case 420:
                    addError(line_idx, formatGCode('G', w.value) + ": cutter radius compensation is not supported (post-process with compensation 'in computer')");
                    break;
                case 930:
                    addError(line_idx, "G93 inverse-time feed is not supported");
                    break;
                case 950:
                    addError(line_idx, "G95 feed-per-revolution is not supported");
                    break;
                default:
                    addError(line_idx, "Unsupported G-code " + formatGCode('G', w.value));
                    break;
            }
        } else if (w.letter == 'M') {
            const int mcode = static_cast<int>(std::round(w.value));
            switch (mcode) {
                case 0: // M0 Compulsory Stop
                case 1: // M1 Optional Stop (always honoured for safety)
                    is_program_stop = true;
                    break;
                case 3: // M3 Torch / Spindle CW ON
                    if (!torch_on) ++result.pierce_count;
                    torch_on = true;
                    spindle_on_line = line_idx;
                    out_mask |= core::OutputBit::OUT1_TorchOrSpindle;
                    out_mask &= ~core::OutputBit::OUT2_SpindleDir;
                    break;
                case 4: // M4 Spindle CCW ON (laser: dynamic power)
                    if (!torch_on) ++result.pierce_count;
                    torch_on = true;
                    spindle_on_line = line_idx;
                    out_mask |= (core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
                    break;
                case 5:  // M5 Spindle / Torch OFF
                    torch_on = false;
                    spindle_on_line = SIZE_MAX;
                    out_mask &= ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
                    break;
                case 2:  // M2 Program End
                case 30: // M30 Program End & Rewind
                    // Spindle and coolant off, as in Mach3 (the controller clears the outputs at the end)
                    torch_on = false;
                    spindle_on_line = SIZE_MAX;
                    out_mask = 0;
                    ends_program = true;
                    result.ends_with_program_end = true;
                    break;
                case 6: // M6 / M06 Tool Change
                    is_tool_change = true;
                    break;
                case 7: // M7 Mist Coolant ON
                    out_mask |= core::OutputBit::OUT4_MistCoolant;
                    break;
                case 8: // M8 Flood Coolant ON
                    out_mask |= core::OutputBit::OUT3_FloodCoolant;
                    break;
                case 9: // M9 Coolant OFF
                    out_mask &= ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant);
                    break;
                case 48: // Feed/speed override enable/disable: accepted, no effect
                case 49:
                    break;
                case 98:
                case 99:
                    addError(line_idx, "Subprograms (M98/M99) are not supported");
                    break;
                default:
                    addWarning(line_idx, "M" + std::to_string(mcode) + " is not executed inside programs (ignored)");
                    break;
            }
        } else if (w.letter == 'T') {
            int tval = static_cast<int>(std::round(w.value));
            if (tval >= 1 && tval <= 255) {
                selected_tool = static_cast<uint16_t>(tval);
                if (!line_comment.empty()) {
                    result.tool_descriptions[selected_tool] = line_comment;
                }
            } else if (tval != 0) {
                addError(line_idx, "Tool number out of range (T1..T255)");
            }
        } else if (w.letter == 'S') {
            if (w.value >= 0.0) {
                spindle_rpm = static_cast<uint16_t>(std::clamp(std::round(w.value), 0.0, 60000.0));
                spindle_s_set = true;
            } else {
                addError(line_idx, "Negative spindle speed");
            }
        } else if (w.letter == 'F') {
            if (w.value > 0.0) {
                feed_mm_min = w.value * unit_scale;
            } else {
                addError(line_idx, "Feed rate must be positive");
            }
        } else if (w.letter == 'P') {
            dwell_p = w.value;
        } else if (w.letter == 'Q') {
            if (w.value > 0.0 && !is_g64) {
                canned_q_given = true;
                canned_q_mm = w.value * unit_scale;
            }
        } else if (w.letter == 'O' || w.letter == 'N' || w.letter == 'H' || w.letter == 'D' || w.letter == 'L') {
            // Program number / line number / offset register words: no motion effect
        } else if (w.letter == 'B' && is_g68) {
            // G68 rotation centre Y, applied below
        } else if (std::strchr("XYZAIJKR", w.letter) == nullptr) {
            addError(line_idx, std::string("Unsupported word '") + w.letter + "'");
        }
    }

    if (is_g64 && dwell_p >= 0.0) {
        // P0 is the same as a plain G64: back to the machine setting
        const double tol_mm = dwell_p * unit_scale;
        corner_tolerance_mm = (tol_mm > 0.0) ? std::clamp(tol_mm, core::MachineConfig::kMinCornerToleranceMm,
                                                          core::MachineConfig::kMaxCornerToleranceMm)
                                             : 0.0;
        if (tol_mm > 0.0 && corner_tolerance_mm != tol_mm) {
            addWarning(line_idx, "G64 P is limited to 0.001 .. 5 mm");
        }
    }
    if (is_g64) dwell_p = -1.0;

    // Canned cycle words last until the cycle ends
    if (active_g_motion != 82) {
        canned_dwell_p = -1.0;
    } else if (!is_dwell && !is_g64 && dwell_p >= 0.0) {
        canned_dwell_p = dwell_p;
    }
    if (active_g_motion != 73 && active_g_motion != 83) {
        canned_q_given = false;
        warned_no_q    = false;
    }

}

void Parser::emitToolChange() {
    // Emit M6 ToolChange segment if M6/M06 was present on this line
    if (is_tool_change) {
        current_tool = selected_tool;
        ++result.tool_change_count;

        core::ToolpathSegment tc_seg = baseSegment(core::SegmentType::ToolChange, line_idx);
        tc_seg.torch_active  = false;
        tc_seg.outputs_mask  = out_mask & ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
        tc_seg.tool_number   = current_tool;
        // The description is looked up by tool number when the tool change runs (ParseResult::tool_descriptions)
        if (!result.tool_descriptions.count(current_tool) && !line_comment.empty()) {
            result.tool_descriptions[current_tool] = line_comment;
        }
        result.segments.push_back(tc_seg);
    }

}

bool Parser::readAxisWords() {
    target_pos = cur_pos;
    // With G68 active, X and Y words are read in the unrotated frame: the current position is turned back
    // first, and the target is rotated once all words are read
    if (rot_active) rotateXY(target_pos[0], target_pos[1], true);


    if (is_g68) {
        // G68 A<x> B<y> R<deg>: words left out keep their last value; with an I word (its value is ignored)
        // the given values are added to the current rotation, as in Mach3
        bool has_i = false;
        double a_val = 0.0, b_val = 0.0, r_val = 0.0;
        bool has_a = false, has_b = false, has_rr = false;
        for (const auto& w : words) {
            if (w.letter == 'I') has_i = true;
            if (w.letter == 'A') { a_val = w.value * unit_scale; has_a = true; }
            if (w.letter == 'B') { b_val = w.value * unit_scale; has_b = true; }
            if (w.letter == 'R') { r_val = w.value; has_rr = true; }
            if (w.letter == 'X' || w.letter == 'Y' || w.letter == 'Z' || w.letter == 'J' || w.letter == 'K') {
                has_xyz = true;
            }
        }
        if (has_xyz) {
            addError(line_idx, "G68 cannot be combined with X, Y, Z, J or K on the same line");
            return true;
        }
        if (has_i) {
            rot_cx += a_val;
            rot_cy += b_val;
            rot_deg += r_val;
        } else {
            if (has_a) rot_cx = a_val + g92_shift[0];
            if (has_b) rot_cy = b_val + g92_shift[1];
            if (has_rr) rot_deg = r_val;
        }
        rot_active = true;
        rot_cos = std::cos(rot_deg * kPi / 180.0);
        rot_sin = std::sin(rot_deg * kPi / 180.0);
        // No motion on this block, but an M0 / M30 on it still counts
        if (is_program_stop) result.segments.push_back(baseSegment(core::SegmentType::ProgramStop, line_idx));
        if (ends_program) program_ended = true;
        return true;
    }

    for (const auto& w : words) {
        switch (w.letter) {
            case 'X': axisWord(0, w.value); break;
            case 'Y': axisWord(1, w.value); break;
            case 'Z': axisWord(2, w.value); canned_z_mm = target_pos[2]; break;
            case 'A': axisWord(3, w.value); break;
            case 'I': arc_i = w.value * unit_scale; has_ijk = true; break;
            case 'J': arc_j = w.value * unit_scale; has_ijk = true; break;
            case 'K': arc_k = w.value * unit_scale; has_ijk = true; break;
            case 'R':
                arc_r       = w.value * unit_scale;
                canned_r_mm = w.value * unit_scale + (absolute_xyz ? g92_shift[2] : 0.0);
                has_r       = true;
                break;
            default:
                break;
        }
    }

    if (rot_active) {
        // Back to work coordinates: rotate the target, or keep the exact XY when neither X nor Y was given
        if (axes_given & 0x03) {
            rotateXY(target_pos[0], target_pos[1], false);
        } else {
            target_pos[0] = cur_pos[0];
            target_pos[1] = cur_pos[1];
        }
        // Arc centres turn with the program: I/J offsets as vectors, G90.1 centres as points
        if (has_ijk) {
            if (ij_incremental) {
                const double i0 = arc_i, j0 = arc_j;
                arc_i = i0 * rot_cos - j0 * rot_sin;
                arc_j = i0 * rot_sin + j0 * rot_cos;
            } else {
                double px = arc_i + g92_shift[0], py = arc_j + g92_shift[1];
                rotateXY(px, py, false);
                arc_i = px - g92_shift[0];
                arc_j = py - g92_shift[1];
            }
        }
    }

    return false;
}

bool Parser::handleCoordinateCommands() {
    // G92.1 / G92.2: cancel coordinate offsets
    if (is_g92_clear) {
        g92_shift = {0.0, 0.0, 0.0, 0.0};
    }

    // G92: make the current position read as the given coordinates (no motion)
    if (is_g92 && rot_active) {
        addError(line_idx, "G92 is not supported while G68 rotation is active (use G69 first)");
        return true;
    }
    if (is_g92) {
        if (!has_xyz) {
            addError(line_idx, "G92 requires at least one axis word");
        }
        for (size_t a = 0; a < 4; ++a) {
            if ((axes_given >> a) & 1U) {
                g92_shift[a] = cur_pos[a] - raw_axis[a];
            }
        }
        return true;
    }

    // Handle G53 (Move in Machine Coordinates, non-modal block)
    if (is_g53_machine) {
        if (!absolute_xyz) {
            addError(line_idx, "G53 cannot be used in incremental mode (G91)");
            return true;
        }
        if (active_g_motion != 0 && active_g_motion != 1) {
            addError(line_idx, "G53 requires G0 or G1 motion mode");
            return true;
        }
        if (has_xyz) {
            std::array<double, 4> machine_target{};
            for (size_t a = 0; a < 4; ++a) machine_target[a] = raw_axis[a];
            emitMachineMove(machine_target, axes_given, active_g_motion == 0, line_idx);
        }
        return true;
    }

    // Handle G28 / G30: optional intermediate point, then rapid to machine zero on the given axes
    if (is_g28) {
        uint8_t home_axes = axes_given;
        if (home_axes == 0) {
            home_axes = 0x07; // No axis words: X, Y and Z return home
        } else {
            emitBlockMove(core::SegmentType::RapidMove);
        }
        emitMachineMove({0.0, 0.0, 0.0, 0.0}, home_axes, true, line_idx);
        return true;
    }

    return false;
}

void Parser::handleDwell() {
    if (is_dwell) {
        if (dwell_p < 0.0) {
            addError(line_idx, "G4 requires a P word");
        } else {
            core::ToolpathSegment seg = baseSegment(core::SegmentType::Dwell, line_idx);
            seg.dwell_seconds = options.dwell_p_milliseconds ? (dwell_p / 1000.0) : dwell_p;
            if (seg.dwell_seconds > 60.0) {
                addWarning(line_idx, "Dwell longer than 60 s - check the G4 P unit setting (seconds vs. milliseconds)");
            }
            if (seg.dwell_seconds > 0.0) {
                result.segments.push_back(seg);
            }
        }
    }

}

// Canned drilling cycle (G73, G81, G82, G83) at the hole of this block. true = the line is finished (error).
bool Parser::emitCannedCycle() {
    if (!absolute_xyz) {
        addError(line_idx, "Canned cycles in incremental mode (G91) are not supported");
        return true;
    }
    if (feed_mm_min <= 0.0) {
        addError(line_idx, "Drilling cycle without a feed rate (F)");
    }
    if ((active_g_motion == 73 || active_g_motion == 83) && !canned_q_given && !warned_no_q) {
        addWarning(line_idx, formatGCode('G', active_g_motion) + " without a Q word: pecking every " +
                                 std::to_string(canned_q_mm) + " mm");
        warned_no_q = true;
    }
    const double drill_z   = canned_z_mm;
    const double retract_z = canned_return_r ? canned_r_mm : std::max(canned_init_z, canned_r_mm);

    // Z held at a machine coordinate (after G53 / G28): its work height is not known here. The XY move stays at
    // that height, then Z goes to the R plane.
    const bool z_held = (held_axes & 0x04U) != 0;
    releaseHeld(static_cast<uint8_t>(axes_given & ~0x04U));

    // If below the R-plane, rapid up to R first so the XY move cannot drag through the part
    if (!z_held && cur_pos[2] < canned_r_mm) {
        std::array<double, 4> up_pos = {cur_pos[0], cur_pos[1], canned_r_mm, cur_pos[3]};
        emitLinearSegment(cur_pos, up_pos, core::SegmentType::RapidMove, line_idx, feed_mm_min);
        cur_pos = up_pos;
    }

    // Rapid in XY to hole coordinates at current Z
    std::array<double, 4> xy_pos = {target_pos[0], target_pos[1], cur_pos[2], target_pos[3]};
    emitLinearSegment(cur_pos, xy_pos, core::SegmentType::RapidMove, line_idx, feed_mm_min);
    cur_pos = xy_pos;

    // Rapid Z to R-plane if above R-plane
    if (z_held) {
        releaseHeld(0x04U);
        std::array<double, 4> r_pos = {cur_pos[0], cur_pos[1], canned_r_mm, cur_pos[3]};
        emitLinearSegment(cur_pos, r_pos, core::SegmentType::RapidMove, line_idx, feed_mm_min, true);
        cur_pos = r_pos;
    } else if (cur_pos[2] > canned_r_mm) {
        std::array<double, 4> r_pos = {cur_pos[0], cur_pos[1], canned_r_mm, cur_pos[3]};
        emitLinearSegment(cur_pos, r_pos, core::SegmentType::RapidMove, line_idx, feed_mm_min);
        cur_pos = r_pos;
    }

    // Plunge or Peck to drill_z
    if ((active_g_motion == 83 || active_g_motion == 73) && canned_q_mm > 0.05 && drill_z < cur_pos[2]) {
        double z_curr = cur_pos[2];
        const double peck = std::max(0.2, canned_q_mm);
        while (z_curr > drill_z + 1e-4) {
            double next_z = std::max(drill_z, z_curr - peck);
            std::array<double, 4> plunge_pt = {cur_pos[0], cur_pos[1], next_z, cur_pos[3]};
            emitLinearSegment(cur_pos, plunge_pt, core::SegmentType::CutLinear, line_idx, feed_mm_min);
            cur_pos = plunge_pt;

            if (next_z > drill_z + 1e-4) {
                double clear_z = (active_g_motion == 83) ? canned_r_mm : std::min(canned_r_mm, next_z + 1.0);
                std::array<double, 4> up_pt = {cur_pos[0], cur_pos[1], clear_z, cur_pos[3]};
                emitLinearSegment(cur_pos, up_pt, core::SegmentType::RapidMove, line_idx, feed_mm_min);
                cur_pos = up_pt;

                double reapproach_z = std::min(canned_r_mm, next_z + 0.5);
                if (cur_pos[2] > reapproach_z) {
                    std::array<double, 4> down_pt = {cur_pos[0], cur_pos[1], reapproach_z, cur_pos[3]};
                    emitLinearSegment(cur_pos, down_pt, core::SegmentType::RapidMove, line_idx, feed_mm_min);
                    cur_pos = down_pt;
                }
            }
            z_curr = next_z;
        }
    } else {
        std::array<double, 4> bot_pt = {cur_pos[0], cur_pos[1], drill_z, cur_pos[3]};
        emitLinearSegment(cur_pos, bot_pt, core::SegmentType::CutLinear, line_idx, feed_mm_min);
        cur_pos = bot_pt;
    }

    // Optional G82 dwell at bottom
    if (active_g_motion == 82 && canned_dwell_p > 0.0) {
        core::ToolpathSegment dseg = baseSegment(core::SegmentType::Dwell, line_idx);
        dseg.dwell_seconds = options.dwell_p_milliseconds ? (canned_dwell_p / 1000.0) : canned_dwell_p;
        result.segments.push_back(dseg);
    }

    // Retract Z to retract_z (G98 initial Z or G99 R-plane)
    std::array<double, 4> ret_pt = {cur_pos[0], cur_pos[1], retract_z, cur_pos[3]};
    emitLinearSegment(cur_pos, ret_pt, core::SegmentType::RapidMove, line_idx, feed_mm_min);
    cur_pos = ret_pt;
    return false;
}

// G2 / G3 in the active plane: chords within the sagitta tolerance, one ArcInfo for the block
void Parser::emitArc() {
    // Determine plane axes (u_axis, v_axis, w_axis) for G17 (XY), G18 (XZ), or G19 (YZ)
    size_t u_ax = 0, v_ax = 1, w_ax = 2;
    double off_u = arc_i, off_v = arc_j;
    if (active_plane == 18) {
        u_ax = 0; v_ax = 2; w_ax = 1; // XZ plane
        off_u = arc_i; off_v = arc_k;
    } else if (active_plane == 19) {
        u_ax = 1; v_ax = 2; w_ax = 0; // YZ plane
        off_u = arc_j; off_v = arc_k;
    }

    // The arc is drawn in work coordinates: its plane axes (and the helix axis and A when named) leave a held
    // machine position; a held helix axis that is not named stays where it is
    releaseHeld(static_cast<uint8_t>((1U << u_ax) | (1U << v_ax) | (axes_given & ((1U << w_ax) | 0x08U))));

    // In G18 (XZ), standard G2/G3 CW/CCW around +Y flips the 2D (X,Z) angle orientation
    bool cw = (active_g_motion == 2);
    if (active_plane == 18) {
        cw = !cw;
    }

    const double u0 = cur_pos[u_ax];
    const double v0 = cur_pos[v_ax];
    const double u1 = target_pos[u_ax];
    const double v1 = target_pos[v_ax];

    double cu = 0.0, cv = 0.0, r = 0.0;
    bool arc_ok = true;
    if (has_ijk) {
        if (ij_incremental) {
            cu = u0 + off_u;
            cv = v0 + off_v;
        } else {
            // G90.1 Absolute IJK Center Mode (centres are programmed coordinates)
            cu = off_u + g92_shift[u_ax];
            cv = off_v + g92_shift[v_ax];
        }
        r = std::hypot(u0 - cu, v0 - cv);
        const double r_end = std::hypot(u1 - cu, v1 - cv);
        if (std::abs(r - r_end) > 0.05 + 0.001 * r) {
            addError(line_idx, "Arc end point is not on the arc (start radius " + std::to_string(r) +
                               " mm, end radius " + std::to_string(r_end) + " mm)");
            arc_ok = false;
        }
    } else if (has_r) {
        double du = u1 - u0;
        double dv = v1 - v0;
        double d  = std::hypot(du, dv);
        r = std::abs(arc_r);
        if (d <= 1e-6) {
            addError(line_idx, "R-format arc with identical start and end point (use I/J/K for full circles)");
            arc_ok = false;
        } else if (r < d * 0.5 - 1e-3) {
            addError(line_idx, "Arc radius R is smaller than half the chord length");
            arc_ok = false;
        } else {
            r = std::max(r, d * 0.5);
            double h = std::sqrt(std::max(0.0, r * r - (d * d * 0.25)));
            // Normal vector to chord
            double nu = -dv / d;
            double nv =  du / d;
            // For CW arc with R > 0, center is to the right of chord; R < 0 selects >180 deg arc
            bool flip = cw ^ (arc_r < 0.0);
            if (flip) {
                nu = -nu;
                nv = -nv;
            }
            cu = 0.5 * (u0 + u1) + h * nu;
            cv = 0.5 * (v0 + v1) + h * nv;
        }
    } else {
        addError(line_idx, "G2/G3 without I/J/K or R");
        arc_ok = false;
    }

    if (arc_ok && r <= 1e-4) {
        addError(line_idx, "Degenerate arc (radius ~ 0)");
        arc_ok = false;
    }

    if (arc_ok) {
        double start_ang = std::atan2(v0 - cv, u0 - cu);
        double end_ang   = std::atan2(v1 - cv, u1 - cu);
        double sweep     = end_ang - start_ang;

        if (std::hypot(u1 - u0, v1 - v0) < 1e-4) {
            sweep = cw ? (-2.0 * kPi) : (2.0 * kPi);
        } else if (cw) {
            if (sweep >= 0.0) sweep -= 2.0 * kPi;
        } else {
            if (sweep <= 0.0) sweep += 2.0 * kPi;
        }

        // Chord length limited both by the requested maximum and by a fixed sagitta tolerance
        const double max_chord = std::max(0.02, std::min(std::max(0.05, arc_chord_tolerance_mm),
                                                          2.0 * std::sqrt(2.0 * r * kArcSagittaMm)));
        double arc_len = std::abs(sweep) * r;
        int steps = std::max(4, static_cast<int>(std::ceil(arc_len / max_chord)));
        steps = std::min(steps, 20000);

        std::array<double, 4> prev_pt = cur_pos;
        ArcInfo arc_rec;
        arc_rec.center_mm[0] = cur_pos[0];
        arc_rec.center_mm[1] = cur_pos[1];
        arc_rec.center_mm[2] = cur_pos[2];
        arc_rec.center_mm[u_ax] = cu;
        arc_rec.center_mm[v_ax] = cv;
        arc_rec.plane = static_cast<uint8_t>(active_plane);
        arc_rec.clockwise = (active_g_motion == 2);
        arc_rec.first_segment = static_cast<uint32_t>(result.segments.size());
        arc_rec.segment_count = static_cast<uint32_t>(steps);
        const uint32_t arc_idx = static_cast<uint32_t>(result.arcs.size());
        result.arcs.push_back(arc_rec);
        const double r_start = std::hypot(u0 - cu, v0 - cv);
        const double r_end   = std::hypot(u1 - cu, v1 - cv);
        for (int s = 1; s <= steps; ++s) {
            double t = static_cast<double>(s) / static_cast<double>(steps);
            double ang = start_ang + sweep * t;
            double r_interp = r_start * (1.0 - t) + r_end * t;

            std::array<double, 4> next_pt = cur_pos;
            next_pt[u_ax] = (s == steps) ? u1 : (cu + r_interp * std::cos(ang));
            next_pt[v_ax] = (s == steps) ? v1 : (cv + r_interp * std::sin(ang));
            next_pt[w_ax] = cur_pos[w_ax] + (target_pos[w_ax] - cur_pos[w_ax]) * t;
            next_pt[3]    = cur_pos[3]    + (target_pos[3]    - cur_pos[3])    * t;

            core::ToolpathSegment seg = baseSegment(
                (active_g_motion == 2) ? core::SegmentType::CutArcCW : core::SegmentType::CutArcCCW, line_idx);
            seg.start_xyza_mm = prev_pt;
            seg.end_xyza_mm   = next_pt;
            seg.length_mm     = calcDist4D(prev_pt, next_pt);
            seg.exact_stop    = exact_stop && (s == steps); // G61: stop at the end of the arc, not at its chords
            seg.arc_index     = arc_idx;
            seg.machine_target = heldTarget();

            result.total_cut_length_mm += seg.length_mm;
            expandBBox(cut_bmin, cut_bmax, prev_pt, first_cut_pt);
            expandBBox(cut_bmin, cut_bmax, next_pt, first_cut_pt);
            expandBBox(all_bmin, all_bmax, prev_pt, first_all_pt);
            expandBBox(all_bmin, all_bmax, next_pt, first_all_pt);
            result.segments.push_back(seg);
            prev_pt = next_pt;
        }
    }
    cur_pos = target_pos;
}

bool Parser::handleMotion() {
    // Handle Canned Drilling Cycles (G73, G81, G82, G83)
    if ((active_g_motion == 73 || active_g_motion == 81 || active_g_motion == 82 || active_g_motion == 83) && has_xyz) {
        if (emitCannedCycle()) return true;
    } else if (has_xyz || ((active_g_motion == 2 || active_g_motion == 3) && (has_ijk || has_r))) {
        // A modal G2 / G3 line with only I / J / K (or R) is an arc like "G2 I.. J..": a full circle from here
        // Standard Motion (G0, G1, G2, G3)
        if (active_g_motion != 0 && feed_mm_min <= 0.0) {
            addError(line_idx, "Feed move without a feed rate (F word)");
        }
        if ((active_g_motion == 2 || active_g_motion == 3) && rot_active && active_plane != 17) {
            // A rotated XZ / YZ arc no longer lies in an axis plane
            addError(line_idx, "G2/G3 in G18/G19 is not supported while G68 rotation is active (G17 only)");
            cur_pos = target_pos;
        } else if (active_g_motion == 2 || active_g_motion == 3) {
            emitArc();
        } else if (active_g_motion == 0 || active_g_motion == 1) {
            // Linear move (G0 Rapid or G1 Linear Cut)
            emitBlockMove((active_g_motion == 0) ? core::SegmentType::RapidMove : core::SegmentType::CutLinear);
        }
    }

    return false;
}

void Parser::endBlock() {
    // Program stop / end markers come after any motion on the same block
    if (is_program_stop) {
        core::ToolpathSegment stop_seg = baseSegment(core::SegmentType::ProgramStop, line_idx);
        result.segments.push_back(stop_seg);
    }
    if (ends_program) {
        program_ended = true;
    }
}

ParseResult Parser::run(const std::string& gcode_text) {
    std::istringstream stream(gcode_text);
    std::string raw_line;
    while (std::getline(stream, raw_line)) {
        if (!beginLine(raw_line)) continue;
        scanLineFlags();
        readModalWords();
        emitToolChange();
        if (readAxisWords()) continue;
        if (handleCoordinateCommands()) continue;
        handleDwell();
        if (handleMotion()) continue;
        endBlock();
    }

    result.spindle_on_without_m5_line = spindle_on_line;

    // Use cutting bounding box if any cutting moves exist; otherwise use all moves bounding box
    if (!first_cut_pt) {
        result.bbox_min_mm = cut_bmin;
        result.bbox_max_mm = cut_bmax;
    } else if (!first_all_pt) {
        result.bbox_min_mm = all_bmin;
        result.bbox_max_mm = all_bmax;
    }

    return result;
}

} // namespace

ParseResult GCodeParser::parse(
    const std::string& gcode_text,
    const std::array<double, 4>& start_pos_mm,
    double arc_chord_tolerance_mm,
    const std::array<double, 4>& work_offset_mm,
    const ParseOptions& options)
{
    return Parser(start_pos_mm, arc_chord_tolerance_mm, work_offset_mm, options).run(gcode_text);
}

} // namespace rnr::gcode
