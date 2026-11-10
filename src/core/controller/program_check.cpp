#include "core/controller/program_check.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rnr::core {

namespace {

const char* const kAxisName[4] = {"X", "Y", "Z", "A"};

bool isMotion(const ToolpathSegment& s) {
    return s.type == SegmentType::RapidMove || s.type == SegmentType::CutLinear ||
           s.type == SegmentType::CutArcCW || s.type == SegmentType::CutArcCCW;
}

bool isFeedMove(const ToolpathSegment& s) {
    return s.type == SegmentType::CutLinear || s.type == SegmentType::CutArcCW || s.type == SegmentType::CutArcCCW;
}

std::string lineText(size_t line) { return "line " + std::to_string(line + 1); }

std::string moreText(size_t more, const char* one, const char* many) {
    if (more == 0) return {};
    return " (" + std::to_string(more) + " more " + (more == 1 ? one : many) + ")";
}

// A problem found on many lines: reported once, on the first line, with the number of other lines
struct LineTally {
    bool   found = false;
    size_t first_line = 0;
    size_t last_line = 0;
    size_t extra_lines = 0;
    void add(size_t line) {
        if (!found) {
            found = true;
            first_line = last_line = line;
        } else if (line != last_line) {
            ++extra_lines;
            last_line = line;
        }
    }
};

// Every programmed point, with the offsets the job would start with, against a per-axis range: the farthest value
// and the lines beyond each side
struct RangeSide {
    LineTally tally;
    double    worst = 0.0;
};
struct RangeScan {
    std::array<std::array<RangeSide, 2>, 4> sides{}; // [axis][0 = below min, 1 = above max]
    bool   found = false;                             // The first point out of range
    size_t first_line = 0, first_axis = 0;
    double first_value = 0.0;
};
RangeScan scanAxisRanges(const std::vector<ToolpathSegment>& segs, const MachineCheckInput& in,
                         const std::array<bool, 4>& use, const std::array<double, 4>& lo,
                         const std::array<double, 4>& hi, double tolerance) {
    RangeScan r;
    const auto& plan_off = in.frame.workspace_offsets[std::min<uint8_t>(in.frame.plan_workspace, 5)];
    for (size_t i = in.start_segment; i < segs.size(); ++i) {
        const auto& seg = segs[i];
        if (!isMotion(seg)) continue;
        const auto t = planner::TrajectoryPlanner::transformTarget(seg, in.frame);
        for (size_t a = 0; a < 4; ++a) {
            if (!use[a]) continue;
            const double m = t[a] + plan_off[a];
            const bool below = m < lo[a] - tolerance;
            const bool above = m > hi[a] + tolerance;
            if (!below && !above) continue;
            if (!r.found) {
                r.found = true;
                r.first_line = seg.gcode_line;
                r.first_axis = a;
                r.first_value = m;
            }
            RangeSide& sd = r.sides[a][above ? 1 : 0];
            if (!sd.tally.found || (above ? m > sd.worst : m < sd.worst)) sd.worst = m;
            sd.tally.add(seg.gcode_line);
        }
    }
    return r;
}

} // namespace

MachineCheckResult checkProgramAgainstMachine(const gcode::ParseResult& prog, const MachineConfig& cfg,
                                              const MachineCheckInput& in) {
    MachineCheckResult out;
    const auto& segs = prog.segments;
    auto block = [&](size_t line, std::string text, std::string fault) {
        if (out.start_refusal.empty()) out.start_refusal = "Cannot start: " + fault;
        out.blockers.push_back({ProgramIssueKind::StartBlocker, line, std::move(text)});
    };

    // Homing: machine coordinates (G53 / G28 / G30) mean nothing on an axis that is not homed
    if (!in.simulator && cfg.require_homing) {
        for (size_t a = 0; a < 4; ++a) {
            if (!((prog.machine_axes_used >> a) & 1U) || in.homed[a]) continue;
            size_t line = 0;
            for (const auto& s : segs) {
                if ((s.machine_axes_mask >> a) & 1U) {
                    line = s.gcode_line;
                    break;
                }
            }
            block(line,
                  std::string("Uses machine coordinates (G53 / G28) on ") + kAxisName[a] +
                      ", which is not homed. Home the machine, or turn off \"Require homing\" for a machine without "
                      "home switches.",
                  std::string("the program uses machine coordinates (G53/G28) on ") + kAxisName[a] +
                      ", which is not homed. Home the machine first, or turn off \"Require homing\" for a machine "
                      "without home switches.");
        }
    }

    // Moves on an axis the connected board does not have: the board would never make them
    if (in.board.known && in.board.axis_count < 4) {
        LineTally tally;
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& s = segs[i];
            if (isMotion(s) && std::abs(s.end_xyza_mm[3] - s.start_xyza_mm[3]) > 1e-6) tally.add(s.gcode_line);
        }
        if (tally.found) {
            block(tally.first_line,
                  "Moves the A axis, but " + in.board.name + " has " + std::to_string(in.board.axis_count) +
                      " axes: the A moves would not be made." + moreText(tally.extra_lines, "line", "lines"),
                  lineText(tally.first_line) + " moves the A axis, which " + in.board.name + " does not have");
        }
    }

    // Tool table: every G43 H<n> needs a measured length, and every tool change must be followed by its G43
    if (cfg.tool_table.mode == ToolLengthMode::ToolTable) {
        const auto& tools = cfg.tool_table.tools;
        std::vector<bool> reported(256, false);
        bool expect_tlo_after_change = false;
        uint16_t changed_to = 0;
        size_t change_line = 0;
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& seg = segs[i];
            if (seg.tlo_tool != kTloInherit && seg.tlo_tool != 0 && tools.find(seg.tlo_tool) == tools.end() &&
                !reported[std::min<size_t>(seg.tlo_tool, 255)]) {
                reported[std::min<size_t>(seg.tlo_tool, 255)] = true;
                const std::string t = std::to_string(seg.tlo_tool);
                block(seg.gcode_line, "G43 H" + t + ": T" + t + " has no length in the tool table. Measure it first.",
                      lineText(seg.gcode_line) + " uses G43 H" + t + ", but T" + t + " has no length in the tool table");
            }
            if (seg.type == SegmentType::ToolChange && cfg.tool_change.mode != ToolChangeMode::IgnoreM6) {
                expect_tlo_after_change = true;
                changed_to  = seg.tool_number;
                change_line = seg.gcode_line;
                continue;
            }
            if (expect_tlo_after_change && isMotion(seg) && seg.machine_axes_mask == 0) {
                if (seg.tlo_tool != changed_to) {
                    const std::string t = std::to_string(changed_to);
                    block(change_line, "The tool change to T" + t + " is not followed by G43 H" + t +
                                           " (required in tool table mode).",
                          "the tool change to T" + t + " on " + lineText(change_line) + " is not followed by G43 H" + t +
                              " (required in tool table mode)");
                }
                expect_tlo_after_change = false;
            }
        }
    }

    // Soft limits of homed axes: every programmed point, with the offsets the job would start with
    std::array<bool, 4> soft_use{};
    std::array<double, 4> soft_lo{}, soft_hi{};
    bool any_soft = false;
    for (size_t a = 0; a < 4; ++a) {
        soft_use[a] = cfg.axes[a].soft_limits_enabled && in.homed[a];
        soft_lo[a] = cfg.axes[a].soft_limit_min_mm;
        soft_hi[a] = cfg.axes[a].soft_limit_max_mm;
        any_soft = any_soft || soft_use[a];
    }
    if (any_soft) {
        const RangeScan scan = scanAxisRanges(segs, in, soft_use, soft_lo, soft_hi, 1e-6);
        if (scan.found) {
            // The Cycle Start message names the first move that is out of range, as before
            char first_err[160];
            const auto& fa = cfg.axes[scan.first_axis];
            std::snprintf(first_err, sizeof(first_err), "%s %.3f mm (machine) is outside the soft limits %.1f..%.1f mm",
                          kAxisName[scan.first_axis], scan.first_value, fa.soft_limit_min_mm, fa.soft_limit_max_mm);
            out.start_refusal = out.start_refusal.empty()
                                    ? "Cannot start: " + lineText(scan.first_line) + " exceeds soft limits (" +
                                          first_err + ")"
                                    : out.start_refusal;
            for (size_t a = 0; a < 4; ++a) {
                for (int side = 0; side < 2; ++side) {
                    const RangeSide& sd = scan.sides[a][side];
                    if (!sd.tally.found) continue;
                    const auto& ax = cfg.axes[a];
                    const double limit = side ? ax.soft_limit_max_mm : ax.soft_limit_min_mm;
                    char buf[200];
                    std::snprintf(buf, sizeof(buf),
                                  "%s goes to %.3f mm (machine), %.3f mm %s the soft limit %.1f mm with the current "
                                  "work offset.",
                                  kAxisName[a], sd.worst, std::abs(sd.worst - limit), side ? "above" : "below", limit);
                    out.blockers.push_back({ProgramIssueKind::StartBlocker, sd.tally.first_line,
                                            std::string(buf) + moreText(sd.tally.extra_lines, "line", "lines")});
                }
            }
        }
    }

    if (!in.include_warnings) return out;

    // Feed rates the axes cannot reach: the planner runs these moves at the highest speed the axes allow
    {
        std::array<double, 4> vmax{};
        const bool board_rates = in.board.known && in.board.rates_known;
        for (size_t a = 0; a < 4; ++a) {
            vmax[a] = board_rates ? in.board.max_rate_mm_min[a] / 60.0 : planner::TrajectoryPlanner::axisVelocityLimit(cfg, a);
        }
        if (const int master = cfg.gantryMasterAxis(); master >= 0 && !in.board.known) {
            vmax[master] = std::min(vmax[master], vmax[3]);
        }
        const bool laser_locks_z = cfg.operation_mode == OperationMode::Laser && !cfg.is_mill && cfg.laser.ignore_z_motion;

        LineTally tally;
        double first_feed = 0.0, first_reach = 0.0;
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& s = segs[i];
            if (!isFeedMove(s) || s.machine_axes_mask != 0 || s.feed_mm_min <= 0.0) continue;
            std::array<double, 4> d{};
            double len2 = 0.0;
            for (size_t a = 0; a < 4; ++a) {
                d[a] = s.end_xyza_mm[a] - s.start_xyza_mm[a];
                if (a == 2 && laser_locks_z) d[a] = 0.0;
                len2 += d[a] * d[a];
            }
            if (len2 <= 1e-12) continue;
            const double len = std::sqrt(len2);
            double v_lim = 1e12;
            for (size_t a = 0; a < 4; ++a) {
                const double u = std::abs(d[a]) / len;
                if (u > 1e-9 && vmax[a] > 1e-9) v_lim = std::min(v_lim, vmax[a] / u);
            }
            const double reach = v_lim * 60.0;
            if (s.feed_mm_min > reach * 1.005 + 0.5) {
                if (!tally.found) {
                    first_feed = s.feed_mm_min;
                    first_reach = reach;
                }
                tally.add(s.gcode_line);
            }
        }
        if (tally.found) {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                          board_rates ? "F%.0f mm/min is faster than the axes can move here (at most %.0f mm/min with "
                                        "the board's maximum rates, $110-$113); the board runs the move slower."
                                      : "F%.0f mm/min is faster than the axes can move here (at most %.0f mm/min with "
                                        "the motor settings); the move runs at the lower speed.",
                          first_feed, first_reach);
            out.warnings.push_back({ProgramIssueKind::Warning, tally.first_line,
                                    std::string(buf) + moreText(tally.extra_lines, "line", "lines")});
        }
    }

    // Spindle speeds outside the VFD's range: the VFD controller limits them
    if (cfg.is_mill && cfg.vfd.enabled && cfg.vfd.max_rpm > 0) {
        uint16_t prev = 0;
        size_t reported = 0, unreported = 0;
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& s = segs[i];
            if (s.spindle_rpm == prev) continue;
            prev = s.spindle_rpm;
            if (s.spindle_rpm == 0) continue;
            const bool high = s.spindle_rpm > cfg.vfd.max_rpm;
            const bool low  = s.spindle_rpm < cfg.vfd.min_rpm;
            if (!high && !low) continue;
            if (reported >= 20) {
                ++unreported;
                continue;
            }
            char buf[200];
            std::snprintf(buf, sizeof(buf), "S%u is %s the VFD's %s speed (%u RPM): the spindle runs at %u RPM.",
                          static_cast<unsigned>(s.spindle_rpm), high ? "above" : "below", high ? "maximum" : "minimum",
                          static_cast<unsigned>(high ? cfg.vfd.max_rpm : cfg.vfd.min_rpm),
                          static_cast<unsigned>(high ? cfg.vfd.max_rpm : cfg.vfd.min_rpm));
            out.warnings.push_back({ProgramIssueKind::Warning, s.gcode_line, buf});
            ++reported;
        }
        if (unreported > 0 && !out.warnings.empty()) {
            out.warnings.back().text += moreText(unreported, "spindle speed out of range", "spindle speeds out of range");
        }
    }

    // BreakoutMill: cutting before the program starts the spindle. Only feed moves below work Z0 count (Z0 is
    // normally the top of the stock): a feed to a clearance height or an XY move above the stock cuts nothing.
    // With Z0 on the bed this never warns, which is a missed warning rather than a false one.
    if (cfg.is_mill) {
        constexpr double kBelowZ0 = -1e-6;
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& s = segs[i];
            if (s.type == SegmentType::ToolChange) continue;
            const bool below_z0 = std::min(s.start_xyza_mm[2], s.end_xyza_mm[2]) < kBelowZ0;
            if (isFeedMove(s) && below_z0 && s.machine_axes_mask == 0 &&
                !(s.outputs_mask & OutputBit::OUT1_TorchOrSpindle)) {
                out.warnings.push_back({ProgramIssueKind::Warning, s.gcode_line,
                                        "Feed move below Z0 while the spindle is off (no M3 / M4 in effect). Start "
                                        "the spindle by hand if the app does not control it."});
                break;
            }
        }
    }

    // A slaved A motor follows its master: programmed A moves are not made (RNR; firmware boards gang motors themselves)
    if (const int master = cfg.gantryMasterAxis(); master >= 0 && !in.board.known) {
        LineTally tally;
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& s = segs[i];
            if (isMotion(s) && std::abs(s.end_xyza_mm[3] - s.start_xyza_mm[3]) > 1e-6) tally.add(s.gcode_line);
        }
        if (tally.found) {
            out.warnings.push_back({ProgramIssueKind::Warning, tally.first_line,
                                    std::string("A moves are not made: the A motor is set to follow ") +
                                        kAxisName[master] + " (Dual-Motor Gantry)." +
                                        moreText(tally.extra_lines, "line", "lines")});
        }
    }

    // The connected firmware board's travel ($130-$133): a warning only, the rules are not confirmed on every firmware
    if (in.board.known) {
        std::array<bool, 4> use{};
        bool any = false;
        for (size_t a = 0; a < 4; ++a) {
            use[a] = in.board.travel_known[a] && in.homed[a];
            any = any || use[a];
        }
        if (any) {
            const RangeScan scan = scanAxisRanges(segs, in, use, in.board.travel_min, in.board.travel_max, 1e-3);
            for (size_t a = 0; a < 4; ++a) {
                for (int side = 0; side < 2; ++side) {
                    const RangeSide& sd = scan.sides[a][side];
                    if (!sd.tally.found) continue;
                    const double limit = side ? in.board.travel_max[a] : in.board.travel_min[a];
                    char buf[320];
                    std::snprintf(buf, sizeof(buf),
                                  "%s goes to %.3f mm (machine), %.3f mm %s the travel %.1f..%.1f mm of %s ($13%zu): %s",
                                  kAxisName[a], sd.worst, std::abs(sd.worst - limit), side ? "beyond" : "below",
                                  in.board.travel_min[a], in.board.travel_max[a], in.board.name.c_str(), a,
                                  in.board.soft_limits ? "the board stops the job with a soft limit alarm."
                                                       : "the axis would run into its end.");
                    out.warnings.push_back({ProgramIssueKind::Warning, sd.tally.first_line,
                                            std::string(buf) + moreText(sd.tally.extra_lines, "line", "lines")});
                }
            }
        }
        // Custom outputs the board does not have (M64 / M65 P<n> are left out)
        LineTally aux;
        int highest = -1;
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& s = segs[i];
            for (int k = 0; k < 4; ++k) {
                if (k >= in.board.aux_outputs && ((s.outputs_mask >> (kCustomOutputShift + k)) & 1U)) {
                    highest = std::max(highest, k);
                    aux.add(s.gcode_line);
                    break;
                }
            }
        }
        if (aux.found) {
            out.warnings.push_back({ProgramIssueKind::Warning, aux.first_line,
                                    "Switches custom output " + std::to_string(highest + 1) + ", but " + in.board.name +
                                        " has " + std::to_string(in.board.aux_outputs) +
                                        " aux outputs: it is left out." + moreText(aux.extra_lines, "line", "lines")});
        }
    }

    // Firmware-planned boards (GRBL family) plan the motion themselves
    if (const auto* board_desc = board::BoardRegistry::instance().find(cfg.board_id);
        board_desc && board_desc->family == board::Family::FirmwarePlanned) {
        LineTally modes;
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& s = segs[i];
            if (isMotion(s) && (s.exact_stop || s.corner_tolerance_mm > 0.0)) modes.add(s.gcode_line);
        }
        if (modes.found) {
            out.warnings.push_back({ProgramIssueKind::Warning, modes.first_line,
                                    board_desc->display_name +
                                        " takes corners with its own junction deviation ($11): G61 / G64 have no effect." +
                                        moreText(modes.extra_lines, "line", "lines")});
        }
        if (!cfg.is_mill && cfg.operation_mode == OperationMode::Plasma && (cfg.thc.enabled || cfg.thc.wait_for_arc_ok) &&
            !in.board_thc) {
            out.warnings.push_back({ProgramIssueKind::Warning, 0,
                                    "Torch height control and Wait for Arc OK need real-time control, which " +
                                        board_desc->display_name + " does not offer: switch THC off to run this program."});
        }
    }

    // M3 / M4 without a later M5, M2 or M30: the program never switches the spindle (torch, laser) off itself
    if (prog.spindle_on_without_m5_line < prog.lines.size()) {
        const char* what = cfg.is_mill ? "spindle"
                           : (cfg.operation_mode == OperationMode::Laser) ? "laser" : "torch";
        out.warnings.push_back({ProgramIssueKind::Warning, prog.spindle_on_without_m5_line,
                                std::string("M3 / M4 is not followed by M5, M2 or M30: the program does not switch the ") +
                                    what + " off (the app does when the program ends)."});
    }
    return out;
}

ProgramCheckResult buildProgramCheck(std::shared_ptr<const gcode::ParseResult> program, const MachineConfig* cfg,
                                     const MachineCheckInput& in) {
    ProgramCheckResult r;
    r.program = program;
    if (!program) return r;
    const auto& prog = *program;
    for (const auto& e : prog.errors) r.issues.push_back({ProgramIssueKind::Error, e.line, e.text});
    for (const auto& w : prog.warnings) r.issues.push_back({ProgramIssueKind::Warning, w.line, w.text});
    if (cfg && !prog.segments.empty()) {
        auto m = checkProgramAgainstMachine(prog, *cfg, in);
        for (auto& b : m.blockers) r.issues.push_back(std::move(b));
        for (auto& w : m.warnings) r.issues.push_back(std::move(w));
    }
    std::stable_sort(r.issues.begin(), r.issues.end(), [](const ProgramIssue& a, const ProgramIssue& b) {
        if (a.line != b.line) return a.line < b.line;
        return static_cast<int>(a.kind) < static_cast<int>(b.kind);
    });
    for (const auto& i : r.issues) {
        switch (i.kind) {
            case ProgramIssueKind::Error:        ++r.error_count;   break;
            case ProgramIssueKind::StartBlocker: ++r.blocker_count; break;
            case ProgramIssueKind::Warning:      ++r.warning_count; break;
        }
    }
    return r;
}

} // namespace rnr::core
