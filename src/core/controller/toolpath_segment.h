#pragma once
// One parsed move or action of a program (ToolpathSegment).
#include "core/board/board_types.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rnr::core {

// ToolpathSegment::tlo_tool value before the program issued any G43 / G49: the offset that is active
// when the job starts stays in effect.
constexpr uint16_t kTloInherit = 0xFFFF;

// ToolpathSegment::arc_index of a segment that is not part of a G2 / G3
constexpr uint32_t kNoArc = 0xFFFFFFFFu;

// ToolpathSegment::machine_target of a segment that ends in work coordinates (no G53 / G28 / G30)
constexpr uint32_t kNoMachineTarget = 0xFFFFFFFFu;

// End point of a G53 / G28 / G30 move in machine coordinates (ParseResult::machine_targets). Only a few segments of
// a program have one, so it is kept out of the segment.
struct MachineTarget {
    std::array<double, 4> mm{};          // Absolute machine coordinate of every axis in axes_mask
    uint8_t               axes_mask = 0; // Bit a set: axis a ends at mm[a]; the other axes keep their work target
};

// ============================================================================
// Toolpath Segment (Used by 2D/3D OpenGL Viewport to draw Cut vs. Uncut paths)
// ============================================================================
enum class SegmentType : uint8_t {
    RapidMove,  // G0 - Dashed travel line (Torch/Spindle OFF)
    CutLinear,  // G1 - Solid cut line (or tessellated G2/G3 chord)
    CutArcCW,   // G2 - Clockwise arc
    CutArcCCW,  // G3 - Counter-clockwise arc
    Dwell,      // G4 - Pierce delay / dwell
    ToolChange, // M6 / M06 - Mach3 Tool Change (Auto ATC or Manual Tool Change Hold)
    ProgramStop, // M0 / M1 - Program stop: motion halts in Feed Hold until Cycle Start
};

// Fields are ordered by size so the segment has no padding holes (136 bytes on 64-bit builds).
struct ToolpathSegment {
    SegmentType           type            = SegmentType::RapidMove;
    bool                  torch_active    = false; // True when M3 is active on this segment
    uint8_t               outputs_mask    = 0;     // Synchronized OUT1..OUT4 bitmask
    uint8_t               workspace_index = 0;     // 0=G54 .. 5=G59
    uint16_t              tool_number     = 1;     // Active or target tool number (T1..T99)
    uint16_t              spindle_rpm     = 0;     // Commanded spindle speed (S word)
    size_t                segment_index   = 0;
    size_t                gcode_line      = 0;
    std::array<double, 4> start_xyza_mm   = {0.0, 0.0, 0.0, 0.0};
    // Work coordinates. For a machine-coordinate move (machine_target set) this is only the preview at parse time:
    // jobs resolve the end through the program's MachineTarget (planner::FrameContext::machine_targets).
    std::array<double, 4> end_xyza_mm     = {0.0, 0.0, 0.0, 0.0};
    double                feed_mm_min     = 1500.0;
    double                dwell_seconds   = 0.0;
    double                length_mm       = 0.0;
    double                corner_tolerance_mm = 0.0; // G64 P: 0 = use MachineConfig::corner_tolerance_mm
    // G2 / G3 chords: index into ParseResult::arcs (firmware-planned boards send the arc as one G2 / G3)
    uint32_t              arc_index = kNoArc;
    // G53 / G28 / G30: index into ParseResult::machine_targets
    uint32_t              machine_target = kNoMachineTarget;
    // Tool length offset in effect (G43 H<n>): 0 = none (G49), kTloInherit = not set by the program yet
    uint16_t              tlo_tool = kTloInherit;
    bool                  spindle_s_set = false; // An S word was programmed (S0 included) at or before this segment
    bool                  exact_stop    = false; // G61: full stop at the end of this move

    bool hasMachineTarget() const noexcept { return machine_target != kNoMachineTarget; }
};
// Programs hold up to millions of segments: keep them plain data (copied as bytes, no allocations) and small
static_assert(std::is_trivially_copyable_v<ToolpathSegment>, "ToolpathSegment must stay plain data");
static_assert(sizeof(size_t) != 8 || sizeof(ToolpathSegment) <= 136, "ToolpathSegment grew: check the field order");

} // namespace rnr::core
