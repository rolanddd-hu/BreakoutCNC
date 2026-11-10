#pragma once
#include "core/board/board_capabilities.h"
#include "core/board/board_profile.h"
#include <functional>
#include <string>
#include <string_view>
#include <vector>

// Board registry: every board the applications can drive, by stable string id (saved in the settings file).
// Adding a board = a driver, its descriptor here, a settings panel and tests (MULTI_BOARD_PLAN.md section 7).
namespace rnr::board {

inline constexpr const char* kRnrBoardId     = "rnr_eco_motion_2";
inline constexpr const char* kGrblBoardId    = "grbl";
inline constexpr const char* kGrblHalBoardId = "grblhal";
inline constexpr const char* kFluidNcBoardId = "fluidnc";

struct BoardDescriptor {
    std::string id;
    std::string display_name;
    std::string description;              // One line for the board selector
    Family      family = Family::HostPlanned;
    std::vector<std::string> transports;  // "hid", "serial", "tcp"
    // Capabilities before connecting (a firmware board refines them from what it reports)
    std::function<BoardCapabilities()> capabilities;
    std::function<BoardProfile()>      default_profile;
};

class BoardRegistry {
public:
    static const BoardRegistry& instance();

    const BoardDescriptor* find(std::string_view id) const;
    const std::vector<BoardDescriptor>& all() const { return boards_; }

    // Capabilities of a board id; unknown ids give the RNR board's
    BoardCapabilities capabilitiesFor(std::string_view id) const;

private:
    BoardRegistry();
    std::vector<BoardDescriptor> boards_;
};

// Capabilities of the RNR Eco Motion 2.0 (IN1..IN12, OUT1..OUT4, host-planned 25 ms slices)
BoardCapabilities rnrEcoMotionCapabilities();

// Fills missing keys of `profile` with the board's defaults; returns true when something was added
bool completeProfile(std::string_view board_id, BoardProfile& profile);

} // namespace rnr::board
