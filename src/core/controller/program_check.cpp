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
    bool any_soft = false;
    for (size_t a = 0; a < 4; ++a) {
        if (cfg.axes[a].soft_limits_enabled && in.homed[a]) any_soft = true;
    }
    if (any_soft) {
        struct Side {
            LineTally tally;
            double    worst = 0.0;
        };
        std::array<std::array<Side, 2>, 4> sides{}; // [axis][0 = below min, 1 = above max]
        bool   first_found = false;
        size_t first_line = 0;
        std::string first_err;
        const auto& plan_off = in.frame.workspace_offsets[std::min<uint8_t>(in.frame.plan_workspace, 5)];
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& seg = segs[i];
            if (!isMotion(seg)) continue;
            const auto t = planner::TrajectoryPlanner::transformTarget(seg, in.frame);
            for (size_t a = 0; a < 4; ++a) {
                const auto& ax = cfg.axes[a];
                if (!ax.soft_limits_enabled || !in.homed[a]) continue;
                const double m = t[a] + plan_off[a];
                const bool below = m < ax.soft_limit_min_mm - 1e-6;
                const bool above = m > ax.soft_limit_max_mm + 1e-6;
                if (!below && !above) continue;
                if (!first_found) {
                    first_found = true;
                    first_line = seg.gcode_line;
                    char buf[160];
                    std::snprintf(buf, sizeof(buf), "%s %.3f mm (machine) is outside the soft limits %.1f..%.1f mm",
                                  kAxisName[a], m, ax.soft_limit_min_mm, ax.soft_limit_max_mm);
                    first_err = buf;
                }
                Side& sd = sides[a][above ? 1 : 0];
                if (!sd.tally.found || (above ? m > sd.worst : m < sd.worst)) sd.worst = m;
                sd.tally.add(seg.gcode_line);
            }
        }
        if (first_found) {
            // The Cycle Start message names the first move that is out of range, as before
            out.start_refusal = out.start_refusal.empty()
                                    ? "Cannot start: " + lineText(first_line) + " exceeds soft limits (" + first_err + ")"
                                    : out.start_refusal;
            for (size_t a = 0; a < 4; ++a) {
                for (int side = 0; side < 2; ++side) {
                    const Side& sd = sides[a][side];
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
        for (size_t a = 0; a < 4; ++a) vmax[a] = planner::TrajectoryPlanner::axisVelocityLimit(cfg, a);
        if (const int master = cfg.gantryMasterAxis(); master >= 0) vmax[master] = std::min(vmax[master], vmax[3]);
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
                if (u > 1e-9) v_lim = std::min(v_lim, vmax[a] / u);
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
                          "F%.0f mm/min is faster than the axes can move here (at most %.0f mm/min with the motor "
                          "settings); the move runs at the lower speed.",
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

    // BreakoutMill: cutting before the program starts the spindle
    if (cfg.is_mill) {
        for (size_t i = in.start_segment; i < segs.size(); ++i) {
            const auto& s = segs[i];
            if (s.type == SegmentType::ToolChange) continue;
            if (isFeedMove(s) && s.machine_axes_mask == 0 && !(s.outputs_mask & OutputBit::OUT1_TorchOrSpindle)) {
                out.warnings.push_back({ProgramIssueKind::Warning, s.gcode_line,
                                        "Feed move while the spindle is off (no M3 / M4 in effect). Start the "
                                        "spindle by hand if the app does not control it."});
                break;
            }
        }
    }

    // A slaved A motor follows its master: programmed A moves are not made
    if (const int master = cfg.gantryMasterAxis(); master >= 0) {
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

    // M3 / M4 without a later M5: the program never switches the spindle (torch, laser) off itself
    if (prog.spindle_on_without_m5_line < prog.lines.size()) {
        const char* what = cfg.is_mill ? "spindle"
                           : (cfg.operation_mode == OperationMode::Laser) ? "laser" : "torch";
        out.warnings.push_back({ProgramIssueKind::Warning, prog.spindle_on_without_m5_line,
                                std::string("M3 / M4 is not followed by M5: the program does not switch the ") + what +
                                    " off."});
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
