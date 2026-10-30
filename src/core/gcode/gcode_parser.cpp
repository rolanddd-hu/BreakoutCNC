#include "core/gcode/gcode_parser.h"
#include <cmath>
#include <cctype>
#include <cstdlib>
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
} // namespace

bool tokenizeBlock(const std::string& raw_line, std::vector<GCodeWord>& words, std::string* comment, std::string* error) {
    words.clear();
    std::string clean;
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
        char* end_ptr = nullptr;
        const double val = std::strtod(clean.c_str() + pos, &end_ptr);
        if (end_ptr == clean.c_str() + pos) {
            if (error) *error = std::string("Letter '") + c + "' has no numeric value";
            return false;
        }
        pos = static_cast<size_t>(end_ptr - clean.c_str());
        words.push_back({c, val});
    }
    return true;
}

ParseResult GCodeParser::parse(
    const std::string& gcode_text,
    const std::array<double, 4>& start_pos_mm,
    double arc_chord_tolerance_mm,
    const std::array<double, 4>& work_offset_mm,
    const ParseOptions& options)
{
    ParseResult result{};
    std::istringstream stream(gcode_text);
    std::string raw_line;

    std::array<double, 4> cur_pos = start_pos_mm; // Work coordinates of the active WCS (G92 applied)
    std::array<double, 4> g92_shift{};            // Work = programmed + g92_shift
    bool   absolute_xyz    = true;  // G90 default (G91 = incremental XYZ)
    bool   ij_incremental  = true;  // G91.1 default in Mach3 (G90.1 = absolute IJ arc centers)
    double unit_scale      = 1.0;   // G21 (mm) default; G20 = 25.4
    int    active_plane    = 17;    // 17=XY (G17), 18=XZ (G18), 19=YZ (G19)
    int    active_g_motion = 0;     // 0=G0, 1=G1, 2=G2, 3=G3, 73=G73, 81=G81, 82=G82, 83=G83
    bool   canned_return_r = false; // false=G98 (return to initial Z), true=G99 (return to R plane)
    double canned_r_mm     = 5.0;
    double canned_q_mm     = 2.0;
    double canned_z_mm     = 0.0;
    double canned_init_z   = cur_pos[2];
    bool   program_ended   = false;
    bool   warned_g43      = false;
    bool   warned_after_end = false;

    double   feed_mm_min       = options.initial_feed_mm_min; // 0 = no F programmed yet
    bool     torch_on          = false;
    uint8_t  out_mask          = 0;
    uint16_t current_tool      = 1;
    uint16_t selected_tool     = 1;
    uint16_t spindle_rpm       = 0; // 0 = not programmed (controller keeps the operator's speed)
    uint8_t  current_wcs       = 0; // 0=G54 .. 5=G59
    uint16_t tlo_tool          = core::kTloInherit; // G43 H<n> in effect (0 = G49)

    // Separate bounding boxes for cutting moves vs. rapid moves so G28/G53/origin rapids don't distort Fit View
    std::array<double, 3> cut_bmin = {0.0, 0.0, 0.0}, cut_bmax = {0.0, 0.0, 0.0};
    std::array<double, 3> all_bmin = {0.0, 0.0, 0.0}, all_bmax = {0.0, 0.0, 0.0};
    bool first_cut_pt = true;
    bool first_all_pt = true;

    auto addError = [&](size_t line, const std::string& text) {
        result.errors.push_back({line, text});
    };
    auto addWarning = [&](size_t line, const std::string& text) {
        result.warnings.push_back({line, text});
    };

    auto baseSegment = [&](core::SegmentType seg_type, size_t line_idx) {
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
        seg.workspace_index = current_wcs;
        seg.tlo_tool        = tlo_tool;
        return seg;
    };

    auto emitLinearSegment = [&](const std::array<double, 4>& p0,
                                 const std::array<double, 4>& p1,
                                 core::SegmentType seg_type,
                                 size_t line_idx,
                                 double seg_feed) {
        double len = calcDist4D(p0, p1);
        if (len <= 1e-6) return;

        core::ToolpathSegment seg = baseSegment(seg_type, line_idx);
        seg.start_xyza_mm = p0;
        seg.end_xyza_mm   = p1;
        seg.feed_mm_min   = seg_feed;
        seg.length_mm     = len;
        seg.torch_active  = (seg_type != core::SegmentType::RapidMove) && torch_on;

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
    };

    // Machine-coordinate move (G53 / G28 / G30). `machine_axes` bits select the axes given in machine
    // coordinates; the work-coordinate preview uses the offset known at parse time.
    auto emitMachineMove = [&](const std::array<double, 4>& machine_target, uint8_t machine_axes,
                               bool rapid, size_t line_idx) {
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
        seg.machine_axes_mask = machine_axes;
        seg.machine_target_mm = machine_target;
        result.machine_axes_used |= machine_axes;
        // Kept even when zero length at parse time: the live offsets may differ when the job runs
        result.segments.push_back(seg);
        result.total_rapid_length_mm += seg.length_mm;
        expandBBox(all_bmin, all_bmax, preview, first_all_pt);
        cur_pos = preview;
    };

    while (std::getline(stream, raw_line)) {
        if (!raw_line.empty() && raw_line.back() == '\r') {
            raw_line.pop_back();
        }
        const size_t line_idx = result.lines.size();
        result.lines.push_back(raw_line);

        std::vector<GCodeWord> words;
        std::string line_comment;
        std::string tok_error;
        if (!tokenizeBlock(raw_line, words, &line_comment, &tok_error)) {
            if (!program_ended) addError(line_idx, tok_error);
            continue;
        }
        if (!line_comment.empty()) {
            tryExtractToolHeaderComment(line_comment, result.tool_descriptions);
        }
        if (words.empty()) continue;

        if (program_ended) {
            if (!warned_after_end) {
                addWarning(line_idx, "Blocks after M2/M30 are ignored");
                warned_after_end = true;
            }
            continue;
        }

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

        // H word (tool length offset register) used by G43 on the same block
        int h_word = -1;
        for (const auto& w : words) {
            if (w.letter == 'H') h_word = static_cast<int>(std::lround(w.value));
        }

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
                                                 "the tool table (Settings > Tool Table & Tool Length Offsets)");
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
                    case 150: // G15 Polar coordinates off
                    case 400: // G40 Cutter compensation off
                    case 500: // G50 Scaling off
                    case 610: // G61 Exact stop
                    case 611: // G61.1
                    case 640: // G64 Constant velocity
                    case 690: // G69 Rotation off
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
                        out_mask |= core::OutputBit::OUT1_TorchOrSpindle;
                        out_mask &= ~core::OutputBit::OUT2_SpindleDir;
                        break;
                    case 4: // M4 Spindle CCW ON (laser: dynamic power)
                        if (!torch_on) ++result.pierce_count;
                        torch_on = true;
                        out_mask |= (core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
                        break;
                    case 5:  // M5 Spindle / Torch OFF
                        torch_on = false;
                        out_mask &= ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
                        break;
                    case 2:  // M2 Program End
                    case 30: // M30 Program End & Rewind
                        torch_on = false;
                        out_mask = 0;
                        ends_program = true;
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
                if (w.value > 0.0) {
                    canned_q_mm = w.value * unit_scale;
                }
            } else if (w.letter == 'O' || w.letter == 'N' || w.letter == 'H' || w.letter == 'D' || w.letter == 'L') {
                // Program number / line number / offset register words: no motion effect
            } else if (std::string("XYZAIJKR").find(w.letter) == std::string::npos) {
                addError(line_idx, std::string("Unsupported word '") + w.letter + "'");
            }
        }

        // Emit M6 ToolChange segment if M6/M06 was present on this line
        if (is_tool_change) {
            current_tool = selected_tool;
            ++result.tool_change_count;

            core::ToolpathSegment tc_seg = baseSegment(core::SegmentType::ToolChange, line_idx);
            tc_seg.torch_active  = false;
            tc_seg.outputs_mask  = out_mask & ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
            tc_seg.tool_number   = current_tool;
            if (result.tool_descriptions.count(current_tool)) {
                tc_seg.tool_description = result.tool_descriptions[current_tool];
            } else if (!line_comment.empty()) {
                tc_seg.tool_description = line_comment;
                result.tool_descriptions[current_tool] = line_comment;
            }
            result.segments.push_back(tc_seg);
        }

        // Pass 2: Evaluate coordinate & arc offset words (X, Y, Z, A, I, J, K, R)
        bool has_xyz = false;
        uint8_t axes_given = 0;
        std::array<double, 4> target_pos = cur_pos;
        std::array<double, 4> raw_axis{};
        bool has_ijk = false;
        bool has_r   = false;
        double arc_i = 0.0, arc_j = 0.0, arc_k = 0.0, arc_r = 0.0;

        auto axisWord = [&](size_t a, double v) {
            raw_axis[a] = v * unit_scale;
            target_pos[a] = absolute_xyz ? (raw_axis[a] + g92_shift[a]) : (cur_pos[a] + raw_axis[a]);
            axes_given |= static_cast<uint8_t>(1U << a);
            has_xyz = true;
        };

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

        // G92.1 / G92.2: cancel coordinate offsets
        if (is_g92_clear) {
            g92_shift = {0.0, 0.0, 0.0, 0.0};
        }

        // G92: make the current position read as the given coordinates (no motion)
        if (is_g92) {
            if (!has_xyz) {
                addError(line_idx, "G92 requires at least one axis word");
            }
            for (size_t a = 0; a < 4; ++a) {
                if ((axes_given >> a) & 1U) {
                    g92_shift[a] = cur_pos[a] - raw_axis[a];
                }
            }
            continue;
        }

        // Handle G53 (Move in Machine Coordinates, non-modal block)
        if (is_g53_machine) {
            if (!absolute_xyz) {
                addError(line_idx, "G53 cannot be used in incremental mode (G91)");
                continue;
            }
            if (active_g_motion != 0 && active_g_motion != 1) {
                addError(line_idx, "G53 requires G0 or G1 motion mode");
                continue;
            }
            if (has_xyz) {
                std::array<double, 4> machine_target{};
                for (size_t a = 0; a < 4; ++a) machine_target[a] = raw_axis[a];
                emitMachineMove(machine_target, axes_given, active_g_motion == 0, line_idx);
            }
            continue;
        }

        // Handle G28 / G30: optional intermediate point, then rapid to machine zero on the given axes
        if (is_g28) {
            uint8_t home_axes = axes_given;
            if (home_axes == 0) {
                home_axes = 0x07; // No axis words: X, Y and Z return home
            } else {
                emitLinearSegment(cur_pos, target_pos, core::SegmentType::RapidMove, line_idx, feed_mm_min);
                cur_pos = target_pos;
            }
            emitMachineMove({0.0, 0.0, 0.0, 0.0}, home_axes, true, line_idx);
            continue;
        }

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

        // Handle Canned Drilling Cycles (G73, G81, G82, G83)
        if ((active_g_motion == 73 || active_g_motion == 81 || active_g_motion == 82 || active_g_motion == 83) && has_xyz) {
            if (!absolute_xyz) {
                addError(line_idx, "Canned cycles in incremental mode (G91) are not supported");
                continue;
            }
            if (feed_mm_min <= 0.0) {
                addError(line_idx, "Drilling cycle without a feed rate (F)");
            }
            const double drill_z   = canned_z_mm;
            const double retract_z = canned_return_r ? canned_r_mm : std::max(canned_init_z, canned_r_mm);

            // 1. If below the R-plane, rapid up to R first so the XY move cannot drag through the part
            if (cur_pos[2] < canned_r_mm) {
                std::array<double, 4> up_pos = {cur_pos[0], cur_pos[1], canned_r_mm, cur_pos[3]};
                emitLinearSegment(cur_pos, up_pos, core::SegmentType::RapidMove, line_idx, feed_mm_min);
                cur_pos = up_pos;
            }

            // 2. Rapid in XY to hole coordinates at current Z
            std::array<double, 4> xy_pos = {target_pos[0], target_pos[1], cur_pos[2], target_pos[3]};
            emitLinearSegment(cur_pos, xy_pos, core::SegmentType::RapidMove, line_idx, feed_mm_min);
            cur_pos = xy_pos;

            // 3. Rapid Z to R-plane if above R-plane
            if (cur_pos[2] > canned_r_mm) {
                std::array<double, 4> r_pos = {cur_pos[0], cur_pos[1], canned_r_mm, cur_pos[3]};
                emitLinearSegment(cur_pos, r_pos, core::SegmentType::RapidMove, line_idx, feed_mm_min);
                cur_pos = r_pos;
            }

            // 4. Plunge or Peck to drill_z
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

            // 5. Optional G82 dwell at bottom
            if (active_g_motion == 82 && dwell_p > 0.0) {
                core::ToolpathSegment dseg = baseSegment(core::SegmentType::Dwell, line_idx);
                dseg.dwell_seconds = options.dwell_p_milliseconds ? (dwell_p / 1000.0) : dwell_p;
                result.segments.push_back(dseg);
            }

            // 6. Retract Z to retract_z (G98 initial Z or G99 R-plane)
            std::array<double, 4> ret_pt = {cur_pos[0], cur_pos[1], retract_z, cur_pos[3]};
            emitLinearSegment(cur_pos, ret_pt, core::SegmentType::RapidMove, line_idx, feed_mm_min);
            cur_pos = ret_pt;
        } else if (has_xyz || ((active_g_motion == 2 || active_g_motion == 3) && (has_ijk || has_r) && motion_word_on_line)) {
            // Standard Motion (G0, G1, G2, G3)
            if (active_g_motion != 0 && feed_mm_min <= 0.0) {
                addError(line_idx, "Feed move without a feed rate (F word)");
            }
            if (active_g_motion == 2 || active_g_motion == 3) {
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
            } else if (active_g_motion == 0 || active_g_motion == 1) {
                // Linear move (G0 Rapid or G1 Linear Cut)
                core::SegmentType st = (active_g_motion == 0) ? core::SegmentType::RapidMove : core::SegmentType::CutLinear;
                emitLinearSegment(cur_pos, target_pos, st, line_idx, feed_mm_min);
                cur_pos = target_pos;
            }
        }

        // Program stop / end markers come after any motion on the same block
        if (is_program_stop) {
            core::ToolpathSegment stop_seg = baseSegment(core::SegmentType::ProgramStop, line_idx);
            result.segments.push_back(stop_seg);
        }
        if (ends_program) {
            program_ended = true;
        }
    }

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

std::string GCodeParser::samplePlasmaBracketGCode() {
    return R"(; =========================================================
; BreakoutCut Demo Part: Heavy-Duty Gusset Bracket (140x90mm)
; Features: 2x Bolt Holes (CCW Inside Cut) + Outer Contour
; Units: mm (G21) | Coordinate Mode: Absolute (G90)
; =========================================================
G21 G90 G91.1
F1800

; --- Hole 1: 24mm Diameter at (X35, Y45) ---
G0 Z10.0
G0 X35.0 Y45.0        ; Rapid to Hole 1 center (lead-in start)
G0 Z3.8               ; Pierce Height
M3                    ; Fire Plasma Torch ON (OUT1)
G4 P0.5               ; Pierce Delay 0.5s
G1 Z1.5 F900          ; Drop to Cut Height (1.5mm)
G1 X47.0 Y45.0 F1500  ; Lead-in to circle edge (R=12mm)
G3 X47.0 Y45.0 I-12.0 J0.0 F1600 ; Full CCW Inside Circle Cut
M5                    ; Torch OFF
G0 Z10.0              ; Retract to Safe Z

; --- Hole 2: 20mm Diameter at (X100, Y35) ---
G0 X100.0 Y35.0       ; Rapid to Hole 2 center (lead-in start)
G0 Z3.8               ; Pierce Height
M3                    ; Fire Plasma Torch ON (OUT1)
G4 P0.5               ; Pierce Delay 0.5s
G1 Z1.5 F900          ; Drop to Cut Height
G1 X110.0 Y35.0 F1500 ; Lead-in to circle edge (R=10mm)
G3 X110.0 Y35.0 I-10.0 J0.0 F1600 ; Full CCW Inside Circle Cut
M5                    ; Torch OFF
G0 Z10.0              ; Retract to Safe Z

; --- Outer Gusset Profile (CW Outside Cut) ---
G0 X-5.0 Y10.0        ; Rapid to exterior lead-in
G0 Z3.8               ; Pierce Height
M3                    ; Fire Plasma Torch ON (OUT1)
G4 P0.6               ; Pierce Delay 0.6s
G1 Z1.5 F900          ; Drop to Cut Height
G1 X0.0 Y10.0 F1800   ; Lead-in onto left edge
G1 X0.0 Y80.0         ; Left vertical edge
G2 X10.0 Y90.0 I10.0 J0.0 ; Top-left 10mm corner radius
G1 X75.0 Y90.0        ; Top horizontal edge
G1 X140.0 Y55.0       ; Angled gusset slope
G1 X140.0 Y10.0       ; Right vertical edge
G2 X130.0 Y0.0 I-10.0 J0.0 ; Bottom-right 10mm corner radius
G1 X10.0 Y0.0         ; Bottom horizontal edge
G2 X0.0 Y10.0 I0.0 J10.0  ; Bottom-left 10mm corner radius
M5                    ; Torch OFF
G0 Z15.0              ; Safe Z Park
G0 X0.0 Y0.0          ; Return to Work Origin
M30                   ; End of Program
)";
}

std::string GCodeParser::sampleLaserCoasterGCode() {
    // Generated rather than written out: the rose curve, raster fill and tick marks are ~1200 moves
    constexpr double kPi = 3.14159265358979323846;
    constexpr double cx = 45.0, cy = 45.0;
    std::ostringstream g;
    g.setf(std::ios::fixed);
    g.precision(3);
    auto polar = [&](double r, double a, double& x, double& y) {
        x = cx + r * std::cos(a);
        y = cy + r * std::sin(a);
    };

    g << "; =========================================================\n"
         "; BreakoutCut Laser Demo: Compass Rose Coaster (90x90mm, 3mm plywood)\n"
         "; Engrave: raster-filled center, 8-petal rose, rings, 72 compass ticks\n"
         "; Cut: rounded outline at full laser power\n"
         "; M4 = dynamic laser power (scaled by speed), S0-1000 = 0-100% power\n"
         "; =========================================================\n"
         "G21 G90 G17\n";

    // 1. Raster fill of the center disc (bidirectional scan lines, 0.25 mm line spacing)
    g << "\n; --- Raster engrave: center disc R7 (laser power 45%) ---\n";
    constexpr double kDiscR = 7.0, kLineStep = 0.25;
    bool first_line = true, left_to_right = true;
    for (double dy = -kDiscR + kLineStep * 0.5; dy < kDiscR; dy += kLineStep) {
        const double hw = std::sqrt(kDiscR * kDiscR - dy * dy);
        const double x0 = left_to_right ? cx - hw : cx + hw;
        const double x1 = left_to_right ? cx + hw : cx - hw;
        g << "G0 X" << x0 << " Y" << cy + dy << "\n";
        if (first_line) { g << "M4 S450\n"; first_line = false; }
        g << "G1 X" << x1 << " F4000\n";
        left_to_right = !left_to_right;
    }
    g << "M5\n";

    // 2. Rose curve r = R * cos(4t): 8 petals, one continuous line
    g << "\n; --- Vector engrave: 8-petal rose R28 (laser power 35%) ---\n";
    constexpr double kRoseR = 28.0;
    constexpr int kRoseSteps = 960;
    for (int i = 0; i <= kRoseSteps; ++i) {
        const double t = 2.0 * kPi * i / kRoseSteps;
        double x, y;
        polar(kRoseR * std::cos(4.0 * t), t, x, y);
        if (i == 0) g << "G0 X" << x << " Y" << y << "\nM4 S350\n";
        else        g << "G1 X" << x << " Y" << y << (i == 1 ? " F2400" : "") << "\n";
    }
    g << "M5\n";

    // 3. Two rings (full arcs)
    g << "\n; --- Vector engrave: rings R32 and R34 (laser power 35%) ---\n";
    for (double r : {32.0, 34.0}) {
        g << "G0 X" << cx + r << " Y" << cy << "\nM4 S350\n"
          << "G3 X" << cx + r << " Y" << cy << " I" << -r << " J0.000 F2400\nM5\n";
    }

    // 4. Compass ticks every 5 degrees, longer at 45 / 90 degrees
    g << "\n; --- Vector engrave: 72 compass ticks (laser power 40%) ---\n";
    for (int i = 0; i < 72; ++i) {
        const double a = 2.0 * kPi * i / 72.0;
        const double r_out = (i % 18 == 0) ? 41.0 : (i % 9 == 0) ? 39.5 : (i % 2 == 0) ? 38.0 : 37.0;
        double x0, y0, x1, y1;
        polar(35.5, a, x0, y0);
        polar(r_out, a, x1, y1);
        g << "G0 X" << x0 << " Y" << y0 << "\nM4 S400\nG1 X" << x1 << " Y" << y1 << " F2000\nM5\n";
    }

    // 5. Outline cut: 90x90 square with R8 corners, constant power (M3), 2 passes
    g << "\n; --- Cut: 90x90 outline, R8 corners, 2 passes (laser power 100%, air assist) ---\n"
         "G0 X0.000 Y8.000\n"
         "M3 S1000\n";
    for (int pass = 0; pass < 2; ++pass) {
        g << "G1 X0.000 Y82.000 F600\n"
             "G2 X8.000 Y90.000 I8.000 J0.000\n"
             "G1 X82.000 Y90.000\n"
             "G2 X90.000 Y82.000 I0.000 J-8.000\n"
             "G1 X90.000 Y8.000\n"
             "G2 X82.000 Y0.000 I-8.000 J0.000\n"
             "G1 X8.000 Y0.000\n"
             "G2 X0.000 Y8.000 I0.000 J8.000\n";
    }
    g << "M5\n"
         "G0 X0.000 Y0.000\n"
         "M30\n";
    return g.str();
}

std::string GCodeParser::sampleMillAtcGCode() {
    return R"((=========================================================)
(BreakoutMill Mach3 ATC Demo: Bearing Housing Block 120x80mm   )
(T1 D=6.0mm Flat Endmill - Circular Pocket & Outer Contour)
(T2 D=4.0mm Spot/Twist Drill - 4x Corner Bolt Holes G83   )
(=========================================================)
G90 G94 G91.1 G40 G49 G17
G21
G28 G91 Z0.
G90

(--- OPERATION 1: 6mm Flat Endmill Pocket & Profile ---)
T1 M06 (T1 D=6.0mm 4-Flute Carbide Flat Endmill)
G43 H1 Z15.0
S14000 M03
G0 X60.0 Y40.0
G0 Z3.0
G1 Z-3.0 F400
G1 X75.0 Y40.0 F1200
G3 X75.0 Y40.0 I-15.0 J0.0 F1400
G1 X60.0 Y40.0
G0 Z10.0

(--- Outer Rounded Rectangle Contour ---)
G0 X0.0 Y10.0
G0 Z2.0
G1 Z-4.0 F400
G1 X0.0 Y70.0 F1500
G2 X10.0 Y80.0 R10.0
G1 X110.0 Y80.0
G2 X120.0 Y70.0 R10.0
G1 X120.0 Y10.0
G2 X110.0 Y0.0 R10.0
G1 X10.0 Y0.0
G2 X0.0 Y10.0 R10.0
G0 Z15.0
M05

(--- OPERATION 2: Tool Change to T2 4mm Drill + G83 Peck Drilling ---)
T2 M06 (T2 D=4.0mm HSS Twist Drill)
G43 H2 Z15.0
S8000 M03
G0 X15.0 Y15.0
G98 G83 X15.0 Y15.0 Z-10.0 R2.0 Q3.0 F350
X105.0 Y15.0
X105.0 Y65.0
X15.0 Y65.0
G80
G0 Z20.0
M05
G28 G91 Z0.
G90
G0 X0.0 Y0.0
M30
)";
}

static bool containsCi(const std::string& str, const std::string& sub) {
    if (sub.empty() || str.size() < sub.size()) return false;
    auto it = std::search(
        str.begin(), str.end(),
        sub.begin(), sub.end(),
        [](char ch1, char ch2) {
            return std::tolower(static_cast<unsigned char>(ch1)) == std::tolower(static_cast<unsigned char>(ch2));
        }
    );
    return it != str.end();
}

static void addUniqueIndicator(std::vector<std::string>& vec, const std::string& item) {
    for (const auto& existing : vec) {
        if (existing == item) return;
    }
    vec.push_back(item);
}

MachineType GCodeParser::detectTargetMachine(
    const std::string& gcode_text,
    int* out_score,
    std::vector<std::string>* out_indicators)
{
    if (gcode_text.empty()) {
        if (out_score) *out_score = 0;
        return MachineType::Unknown;
    }

    int mill_score   = 0;
    int plasma_score = 0;
    int laser_score  = 0;

    std::vector<std::string> mill_reasons;
    std::vector<std::string> plasma_reasons;
    std::vector<std::string> laser_reasons;

    std::istringstream stream(gcode_text);
    std::string line;
    int line_num = 0;
    int last_m3_line = -100;
    int g1_inline_s_count = 0;

    // Scan up to the first 4000 lines to maintain ultra-fast UI loading
    while (std::getline(stream, line) && line_num < 4000) {
        ++line_num;

        // 1. Check comments in the line
        std::string comment;
        auto paren_open = line.find('(');
        if (paren_open != std::string::npos) {
            auto paren_close = line.find(')', paren_open + 1);
            if (paren_close != std::string::npos) {
                comment += " " + line.substr(paren_open + 1, paren_close - paren_open - 1);
            }
        }
        auto semi = line.find(';');
        if (semi != std::string::npos) {
            comment += " " + line.substr(semi + 1);
        }

        if (!comment.empty()) {
            // Plasma CAM signatures
            if (containsCi(comment, "sheetcam")) {
                plasma_score += 45;
                addUniqueIndicator(plasma_reasons, "Header specifies SheetCam Plasma CAM");
            }
            if (containsCi(comment, "hypertherm") || containsCi(comment, "powermax")) {
                plasma_score += 40;
                addUniqueIndicator(plasma_reasons, "Comment references Hypertherm Plasma Torch");
            }
            if (containsCi(comment, "pierce height") || containsCi(comment, "cut height") || containsCi(comment, "pierce delay")) {
                plasma_score += 35;
                addUniqueIndicator(plasma_reasons, "Plasma process parameters (Pierce / Cut Height)");
            }
            if (containsCi(comment, "ohmic") || containsCi(comment, "floating head") || containsCi(comment, "torch on")) {
                plasma_score += 30;
                addUniqueIndicator(plasma_reasons, "Plasma torch sensing & ignition comments");
            }
            if (containsCi(comment, "plasma")) {
                plasma_score += 25;
                addUniqueIndicator(plasma_reasons, "Header/comment explicitly references Plasma");
            }

            // Laser CAM signatures
            if (containsCi(comment, "lightburn")) {
                laser_score += 50;
                addUniqueIndicator(laser_reasons, "Header specifies LightBurn Laser Software");
            }
            if (containsCi(comment, "lasergrbl") || containsCi(comment, "laserweb") || containsCi(comment, "rdworks")) {
                laser_score += 45;
                addUniqueIndicator(laser_reasons, "Header specifies Laser engraving CAM software");
            }
            if (containsCi(comment, "air assist") || containsCi(comment, "laser power")) {
                laser_score += 30;
                addUniqueIndicator(laser_reasons, "Laser optics & air assist references");
            }
            if (containsCi(comment, "laser")) {
                laser_score += 20;
                addUniqueIndicator(laser_reasons, "Header/comment explicitly references Laser");
            }

            // Mill CAM signatures
            if (containsCi(comment, "fusion 360") || containsCi(comment, "autodesk") ||
                containsCi(comment, "vcarve") || containsCi(comment, "aspire") || containsCi(comment, "vectric") ||
                containsCi(comment, "mastercam") || containsCi(comment, "solidcam") || containsCi(comment, "carbide create")) {
                mill_score += 45;
                addUniqueIndicator(mill_reasons, "Header specifies Milling CAM software (Fusion 360 / Vectric / Mastercam)");
            }
            if (containsCi(comment, "endmill") || containsCi(comment, "end mill") || containsCi(comment, "ball nose") ||
                containsCi(comment, "v-bit") || containsCi(comment, "face mill") || containsCi(comment, "twist drill") ||
                containsCi(comment, "chamfer mill")) {
                mill_score += 40;
                addUniqueIndicator(mill_reasons, "Tool definitions specify Milling Cutters (Endmill / Drill)");
            }
            if (containsCi(comment, "spindle speed") || containsCi(comment, "spindle on")) {
                mill_score += 20;
                addUniqueIndicator(mill_reasons, "Header specifies Milling Spindle Parameters");
            }
        }

        // 2. Strip comments to inspect raw executable G-code
        std::string code = line;
        auto semi_pos = code.find(';');
        if (semi_pos != std::string::npos) code = code.substr(0, semi_pos);
        while (true) {
            auto p1 = code.find('(');
            if (p1 == std::string::npos) break;
            auto p2 = code.find(')', p1 + 1);
            if (p2 == std::string::npos) { code = code.substr(0, p1); break; }
            code.erase(p1, p2 - p1 + 1);
        }

        if (code.empty()) continue;

        // Convert to uppercase
        for (char& c : code) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

        // Tool change (M6 / M06 / Tx M6) -> Strong Mill
        if (code.find("M6") != std::string::npos || code.find("M06") != std::string::npos) {
            mill_score += 50;
            addUniqueIndicator(mill_reasons, "Tool change command (M6)");
        }

        // Tool length offset (G43 / G49) -> Strong Mill
        if (code.find("G43") != std::string::npos) {
            mill_score += 35;
            addUniqueIndicator(mill_reasons, "Tool length offset compensation (G43)");
        }

        // Canned drilling cycles (G81, G82, G83, G73, G84, G85) -> Exclusive to Mill
        if (code.find("G81") != std::string::npos || code.find("G82") != std::string::npos ||
            code.find("G83") != std::string::npos || code.find("G73") != std::string::npos ||
            code.find("G84") != std::string::npos || code.find("G85") != std::string::npos) {
            mill_score += 50;
            addUniqueIndicator(mill_reasons, "Canned drilling / pecking cycles (G81-G83)");
        }

        // Cutter radius compensation (G41 / G42)
        if (code.find("G41") != std::string::npos || code.find("G42") != std::string::npos) {
            mill_score += 25;
            addUniqueIndicator(mill_reasons, "Cutter radius compensation (G41/G42)");
        }

        // Spindle speed checks (S...)
        auto s_pos = code.find('S');
        if (s_pos != std::string::npos && s_pos + 1 < code.size() && std::isdigit(static_cast<unsigned char>(code[s_pos + 1]))) {
            try {
                size_t num_len = 0;
                long s_val = std::stol(code.substr(s_pos + 1), &num_len);
                if (s_val >= 2000) {
                    mill_score += 30;
                    addUniqueIndicator(mill_reasons, "High spindle RPM commanded (S" + std::to_string(s_val) + ")");
                } else if (s_val > 0 && s_val <= 1000) {
                    if (code.find("M4") != std::string::npos || code.find("G1") != std::string::npos) {
                        laser_score += 25;
                        addUniqueIndicator(laser_reasons, "PWM laser power level (S" + std::to_string(s_val) + ")");
                    }
                }
            } catch (...) {}
        }

        // Dynamic laser mode (M4 / M04)
        if (code.find("M4") != std::string::npos || code.find("M04") != std::string::npos) {
            laser_score += 45;
            addUniqueIndicator(laser_reasons, "Dynamic laser power mode (M4)");
        }

        // Inline power adjustments on cutting moves (e.g. G1 X... Y... S...)
        if (code.find("G1") != std::string::npos && code.find('S') != std::string::npos && code.find('Z') == std::string::npos) {
            if (++g1_inline_s_count >= 5) {
                laser_score += 35;
                addUniqueIndicator(laser_reasons, "Continuous inline laser power modulation on cut vectors");
            }
        }

        // Plasma Torch Fire (M3 / M03) and Pierce Dwell (G4 / G04) pattern
        if (code.find("M3") != std::string::npos || code.find("M03") != std::string::npos) {
            // If line does NOT command a high spindle RPM (e.g. S12000), record M3 line for pierce sequence check
            if (code.find("S1") == std::string::npos && code.find("S2") == std::string::npos) {
                last_m3_line = line_num;
            }
        }

        if (code.find("G4") != std::string::npos || code.find("G04") != std::string::npos) {
            if (line_num - last_m3_line <= 2 && (line_num - last_m3_line) >= 0) {
                plasma_score += 55;
                addUniqueIndicator(plasma_reasons, "Plasma torch pierce dwell sequence (M3 followed by G4 dwell)");
            }
        }

        // Deep negative Z plunge (Z-1.0 or deeper) typical for milling operations
        auto z_pos = code.find("Z-");
        if (z_pos != std::string::npos && z_pos + 2 < code.size() && std::isdigit(static_cast<unsigned char>(code[z_pos + 2]))) {
            mill_score += 15;
            addUniqueIndicator(mill_reasons, "Multi-depth material cutting (Z negative plunge)");
        }
    }

    // Determine target machine type with confidence threshold
    const int kConfidenceThreshold = 35;
    MachineType detected = MachineType::Unknown;
    int detected_score = 0;
    std::vector<std::string> chosen_reasons;

    if (mill_score >= kConfidenceThreshold && mill_score > plasma_score + 15 && mill_score > laser_score + 15) {
        detected = MachineType::Mill;
        detected_score = mill_score;
        chosen_reasons = mill_reasons;
    } else if (plasma_score >= kConfidenceThreshold && plasma_score > mill_score + 15 && plasma_score > laser_score + 15) {
        detected = MachineType::Plasma;
        detected_score = plasma_score;
        chosen_reasons = plasma_reasons;
    } else if (laser_score >= kConfidenceThreshold && laser_score > mill_score + 15 && laser_score > plasma_score + 15) {
        detected = MachineType::Laser;
        detected_score = laser_score;
        chosen_reasons = laser_reasons;
    }

    if (out_score) *out_score = detected_score;
    if (out_indicators) *out_indicators = chosen_reasons;

    return detected;
}

MachineTypeCheckResult GCodeParser::checkMachineCompatibility(
    const std::string& gcode_text,
    MachineType current_machine)
{
    int score = 0;
    std::vector<std::string> indicators;
    MachineType detected = detectTargetMachine(gcode_text, &score, &indicators);

    MachineTypeCheckResult result;
    result.detected_type = detected;
    result.confidence_score = score;
    result.indicators = indicators;

    auto typeToName = [](MachineType t) -> std::string {
        switch (t) {
            case MachineType::Mill:   return "CNC Milling Machine";
            case MachineType::Plasma: return "CNC Plasma Cutter";
            case MachineType::Laser:  return "CNC Laser Cutter";
            default:                  return "Standard CNC Machine";
        }
    };

    result.detected_name = typeToName(detected);
    result.current_name  = typeToName(current_machine);

    // If ambiguous or detected matches current machine environment, there is no mismatch
    if (detected == MachineType::Unknown || detected == current_machine) {
        result.has_mismatch = false;
        return result;
    }

    // Mismatch detected!
    result.has_mismatch = true;
    return result;
}

} // namespace rnr::gcode
