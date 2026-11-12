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

struct ToolpathSegment {
    SegmentType           type            = SegmentType::RapidMove;
    size_t                segment_index   = 0;
    size_t                gcode_line      = 0;
    std::array<double, 4> start_xyza_mm   = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> end_xyza_mm     = {0.0, 0.0, 0.0, 0.0};
    double                feed_mm_min     = 1500.0;
    double                dwell_seconds   = 0.0;
    double                length_mm       = 0.0;
    bool                  torch_active    = false; // True when M3 is active on this segment
    uint8_t               outputs_mask    = 0;     // Synchronized OUT1..OUT4 bitmask
    uint16_t              tool_number     = 1;     // Active or target tool number (T1..T99)
    uint16_t              spindle_rpm     = 0;     // Commanded spindle speed (S word)
    bool                  spindle_s_set   = false; // An S word was programmed (S0 included) at or before this segment
    uint8_t               workspace_index = 0;     // 0=G54 .. 5=G59
    // Machine-coordinate targets (G53 / G28 / G30): bit a set => end axis a is the absolute machine
    // coordinate machine_target_mm[a]. end_xyza_mm holds the work-coordinate preview at parse time.
    uint8_t               machine_axes_mask = 0;
    std::array<double, 4> machine_target_mm = {0.0, 0.0, 0.0, 0.0};
    // Tool length offset in effect (G43 H<n>): 0 = none (G49), kTloInherit = not set by the program yet
    uint16_t              tlo_tool = kTloInherit;
    bool                  exact_stop = false;           // G61: full stop at the end of this move
    double                corner_tolerance_mm = 0.0;    // G64 P: 0 = use MachineConfig::corner_tolerance_mm
    // G2 / G3 chords: index into ParseResult::arcs (firmware-planned boards send the arc as one G2 / G3)
    uint32_t              arc_index = kNoArc;
};
// Programs hold up to millions of segments: keep them plain data (copied as bytes, no allocations)
static_assert(std::is_trivially_copyable_v<ToolpathSegment>, "ToolpathSegment must stay plain data");

} // namespace rnr::core
