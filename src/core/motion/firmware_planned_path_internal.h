#pragma once
// Shared by the firmware_planned_path*.cpp files (one class, split by topic). Not for other code.
#include "core/motion/firmware_planned_path.h"
#include "core/board/board_registry.h"
#include "core/board/grbl/grbl_codes.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rnr::core::motion {

using board::grbl::FirmwareMessage;
using board::grbl::MessageKind;
using board::grbl::Realtime;
using LogDir = transport::ProtocolLogEntry::Dir;

namespace fpp_detail {

constexpr auto kSilenceTimeout   = std::chrono::milliseconds(2500);
constexpr auto kHomingTimeout    = std::chrono::seconds(180);
constexpr auto kIdentifyTimeout  = std::chrono::milliseconds(2500);
constexpr auto kSettingsTimeout  = std::chrono::seconds(6);
constexpr auto kHoldTimeout      = std::chrono::seconds(10);
constexpr auto kResetTimeout     = std::chrono::seconds(3);
constexpr double kJogLeadTime_s  = 0.10; // Starting values, to be tuned on the bench (GRBL_FAMILY_PLAN 9.2)
constexpr double kJogChunkTime_s = 0.05;
inline constexpr char kAxis[4] = {'X', 'Y', 'Z', 'A'};

inline bool isMotion(SegmentType t) {
    return t == SegmentType::RapidMove || t == SegmentType::CutLinear || t == SegmentType::CutArcCW ||
           t == SegmentType::CutArcCCW;
}

inline std::string fmt(double v) {
    return GcodeEmitter::num(v, 3);
}

} // namespace fpp_detail

} // namespace rnr::core::motion
