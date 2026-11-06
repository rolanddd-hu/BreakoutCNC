#pragma once
#include "core/controller/motion_controller.h"
#include "core/gcode/gcode_parser.h"
#include "core/planner/trajectory_planner.h"
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace rnr::core {

// Checks of a loaded program against the machine settings and the machine's current state (work offsets,
// homed axes, tool table). The same code runs when a program is loaded (the program panel lists every problem)
// and on Cycle Start (which is refused while a StartBlocker remains).
enum class ProgramIssueKind : uint8_t {
    Error        = 0, // Parse error: the program cannot run until it is edited
    StartBlocker = 1, // Cycle Start is refused in the current machine state (homing, offsets, tool table)
    Warning      = 2, // The program runs, but differently from what it says
};

struct ProgramIssue {
    ProgramIssueKind kind = ProgramIssueKind::Warning;
    size_t           line = 0; // 0-based index into ParseResult::lines
    std::string      text;
};

struct MachineCheckInput {
    planner::FrameContext frame;              // Work offsets and tool lengths the job would start with
    std::array<bool, 4>   homed{};
    bool                  simulator = false;
    size_t                start_segment = 0;  // Run From Line: moves before this segment are not checked
    bool                  include_warnings = true;
};

struct MachineCheckResult {
    std::vector<ProgramIssue> blockers;      // In check order: homing, tool table, soft limits
    std::vector<ProgramIssue> warnings;
    std::string               start_refusal; // Fault message for Cycle Start (first blocker), empty if none
};

MachineCheckResult checkProgramAgainstMachine(const gcode::ParseResult& program, const MachineConfig& cfg,
                                              const MachineCheckInput& in);

// Everything the program panel shows: parse errors and warnings plus the machine checks, sorted by line
struct ProgramCheckResult {
    std::shared_ptr<const gcode::ParseResult> program; // The program these issues belong to
    std::vector<ProgramIssue> issues;
    size_t error_count   = 0;
    size_t blocker_count = 0;
    size_t warning_count = 0;
};

// cfg = nullptr: only the parse errors and warnings (no machine settings available)
ProgramCheckResult buildProgramCheck(std::shared_ptr<const gcode::ParseResult> program, const MachineConfig* cfg,
                                     const MachineCheckInput& in);

} // namespace rnr::core
