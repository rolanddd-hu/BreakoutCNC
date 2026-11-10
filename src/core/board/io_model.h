#pragma once
#include <cstdint>
#include <string>
#include <vector>

// Board I/O model: the terminals a board has (IoPoint) and the machine function each one serves (IoRole).
// See docs/architecture/MULTI_BOARD_PLAN.md section 4.5.
namespace rnr::board {

enum class IoKind : uint8_t { DigitalIn, DigitalOut, AnalogIn, PwmOut };

enum class IoRole : uint16_t {
    None = 0, // Assignable in the app (RNR terminals) or a plain status input
    // Inputs
    EStop,
    Probe,
    ToolSetter,
    Door,
    LimitX,
    LimitY,
    LimitZ,
    LimitA,
    CycleStart,
    FeedHold,
    ArcOk,
    Reset,
    // Outputs
    SpindleEnable,
    SpindleDirection,
    Flood,
    Mist,
    AuxOutput,  // grblHAL / FluidNC M62-M65 outputs
};

struct IoPoint {
    std::string id;       // Stable: "IN3", "OUT1", "probe", "limit_x", "aux_out_0"
    std::string label;    // Shown in the UI
    IoKind      kind = IoKind::DigitalIn;
    IoRole      fixed_role = IoRole::None; // None = the user assigns a function in the app
    bool        invertible_in_app = false;
    char        status_letter = 0;         // GRBL-family: the Pn: letter that reports this input
};

inline bool isInput(const IoPoint& p) noexcept { return p.kind == IoKind::DigitalIn || p.kind == IoKind::AnalogIn; }

// Index of the first point with this role, or -1
inline int findIoRole(const std::vector<IoPoint>& io, IoRole role) noexcept {
    for (size_t i = 0; i < io.size(); ++i) {
        if (io[i].fixed_role == role) return static_cast<int>(i);
    }
    return -1;
}

} // namespace rnr::board
