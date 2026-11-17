#pragma once
#include "core/board/board_capabilities.h"
#include <array>
#include <cstddef>
#include <string>
#include <string_view>

// What a connected GRBL-family board is and what it was built with (GRBL_FAMILY_PLAN.md sections 1, 4 and 12).
// Built from the welcome line and the $I build info while connecting.
namespace rnr::board::grbl {

enum class Flavour : uint8_t { Grbl11, GrblHal, FluidNC, GenericGrbl };

const char* flavourName(Flavour f) noexcept;
Flavour flavourForBoardId(std::string_view board_id) noexcept;
const char* boardIdForFlavour(Flavour f) noexcept;

struct FirmwareProfile {
    Flavour     flavour = Flavour::GenericGrbl;
    bool        detected = false;           // A welcome line or $I identified the firmware
    std::string welcome;                    // Welcome line
    std::string version_text;               // "[VER:...]" content
    std::string option_codes;               // "[OPT:...]" letters
    std::string new_options;                // grblHAL "[NEWOPT:...]"
    std::string firmware_name;              // grblHAL "[FIRMWARE:...]"
    std::string board_name;                 // grblHAL "[BOARD:...]"
    size_t      planner_blocks = 15;
    size_t      rx_buffer_bytes = 128;
    size_t      max_line_chars = 80;
    uint8_t     axis_count = 3;
    std::string axis_letters = "XYZ";
    bool        reports_line_numbers = false; // Ln: in status reports (we then send N words)
    bool        reports_homed = false;        // grblHAL H:
    bool        top_bit_realtime = false;     // grblHAL RT+: use 0x80..0x82 instead of ? ! ~
    bool        home_single_axis = false;
    bool        has_mist = false;             // M7
    bool        variable_spindle = true;
    bool        safety_door = false;
    bool        origin_at_home = false;       // GRBL build option Z
    bool        settings_enumeration = false; // grblHAL ENUMS
    uint8_t     aux_outputs = 0, aux_inputs = 0;
    bool        plasma_plugin = false;        // grblHAL plasma plugin present ([PLUGIN:PLASMA ...])
    bool        board_thc = false;            // ...with a THC mode switched on (THC in [NEWOPT:])
    std::string plasma_mode;                  // "Voltage", "Up/down", "Arc ok", "Off"

    // From the board's settings ($$ / config), when read
    bool        reports_inches = false;       // $13
    bool        homing_enabled = false;       // $22
    bool        soft_limits = false;          // $20
    bool        laser_mode = false;           // $32
    double      max_spindle_rpm = 1000.0;     // $30
    double      min_spindle_rpm = 0.0;        // $31
    std::array<double, 4> steps_per_mm{250.0, 250.0, 250.0, 250.0};
    std::array<double, 4> max_rate_mm_min{500.0, 500.0, 500.0, 500.0};
    std::array<double, 4> accel_mm_s2{10.0, 10.0, 10.0, 10.0};
    std::array<double, 4> max_travel_mm{200.0, 200.0, 200.0, 200.0};
    uint8_t     homing_dir_invert_mask = 0;   // $23
    bool        settings_read = false;

    // Display string, e.g. "grblHAL 1.1f.20230610"
    std::string describe() const;
};

// Defaults for a flavour before anything was read from the board
FirmwareProfile defaultProfile(Flavour f);

// Flavour from a welcome line / build info (GenericGrbl if not decisive)
Flavour detectFromWelcome(const std::string& welcome) noexcept;

// Applies one build info message ("VER", "OPT", "NEWOPT", "AXS", "AUX IO", "FIRMWARE", "BOARD") to the profile
void applyBuildInfo(FirmwareProfile& p, const std::string& tag, const std::string& body);

// Applies one numbered setting ($n=value) to the profile values the app mirrors
void applyNumberedSetting(FirmwareProfile& p, int number, double value);

// Capabilities of a GRBL-family board: the flavour's defaults, refined by what the board reported
BoardCapabilities firmwareCapabilities(Flavour f, const FirmwareProfile* detected = nullptr);

} // namespace rnr::board::grbl
