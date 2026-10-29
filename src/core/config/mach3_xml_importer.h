#pragma once
#include "core/controller/motion_controller.h"
#include <string>
#include <vector>
#include <map>

namespace rnr::config {

enum class MigrationSeverity : uint8_t {
    Ok            = 0, // Clean 1:1 or standard unit conversion
    AutoCorrected = 1, // Safely adjusted/translated (e.g. Dir XOR Reversed, Min/Max swap)
    Warning       = 2, // Requires operator attention (e.g. wiring change, missing E-Stop, Z homing down)
};

struct Mach3MigrationItem {
    std::string       category;
    std::string       parameter_name;
    std::string       mach3_raw_value;
    std::string       migrated_value;
    MigrationSeverity severity = MigrationSeverity::Ok;
    std::string       note;
};

// One input signal that was wired to the parallel port in Mach3 and needs a new RNR terminal
struct LptPinRemapEntry {
    std::string signal_name;
    int         lpt_port = 1;
    int         lpt_pin  = 0;
    uint8_t     rnr_in_pin = 0; // 1..12 (IN1..IN12), 0 = left disabled
    bool        active_low = false;
};

struct Mach3ImportOptions {
    bool combine_reversed_with_dir     = true;  // Effective Dir invert = Motor DirNeg XOR Homing/Limits "Reversed"
    bool auto_fix_inverted_soft_limits = true;  // Swap soft Min/Max if Min > Max
    int  units_override                = -1;    // -1 = Mach3 <Units> (0 = mm, 1 = inch), 0 = force mm, 1 = force inch
    int  velocity_unit_mode            = 0;     // 0 = auto (Mach3 stores units/sec), 1 = units/sec, 2 = units/min
    // Parallel-port profiles: RNR terminal chosen by the operator per signal (signal name -> IN pin, 0 = off).
    // Signals not listed get the suggested terminal (LPT pins 10, 11, 12, 13, 15 -> IN1..IN5).
    std::map<std::string, uint8_t> lpt_pin_overrides;
};

struct Mach3MigrationReport {
    bool                             success = false;
    std::string                      error_message;
    std::string                      source_file_path;
    std::string                      profile_name;
    bool                             detected_imperial_units = false;
    bool                             detected_lpt_parallel_port = false; // Inputs came from a parallel-port profile
    bool                             detected_rnr_profile    = false;    // Profile was used with the RNR Mach3 plugin
    bool                             detected_plasma_profile = false;
    bool                             detected_mill_profile   = false;
    bool                             detected_lathe_profile  = false;
    bool                             detected_vfd_plugin     = false;    // <VfdDriver> section (royaumedeole.fr plugin)

    core::MachineConfig              migrated_config{};
    std::vector<Mach3MigrationItem>  items;
    std::vector<std::string>         safety_warnings;
    std::vector<LptPinRemapEntry>    lpt_remaps;

    size_t countBySeverity(MigrationSeverity sev) const noexcept {
        size_t n = 0;
        for (const auto& it : items) {
            if (it.severity == sev) ++n;
        }
        return n;
    }
};

// Imports a Mach3 profile XML (e.g. C:\Mach3\Mach3Mill.xml). Reads <Preferences> (Mach3 itself),
// <RnRMotion> (RNR Mach3 plugin) and <VfdDriver> (Huanyang VFD plugin by royaumedeole.fr).
class Mach3XmlImporter {
public:
    // Parse a Mach3 XML profile from disk, starting from `base_config` for any unmentioned fields
    static Mach3MigrationReport importFromFile(
        const std::string& xml_file_path,
        const core::MachineConfig& base_config,
        const Mach3ImportOptions& options = {});

    // Parse a Mach3 XML profile directly from an XML string (used by file loader & unit tests)
    static Mach3MigrationReport importFromString(
        const std::string& xml_content,
        const core::MachineConfig& base_config,
        const Mach3ImportOptions& options = {},
        const std::string& source_name = "Mach3Profile.xml");

    // Scan standard Mach3 installation folders (e.g. C:\Mach3) for *.xml profile files
    static std::vector<std::string> discoverLocalMach3Profiles(
        const std::string& search_dir = "C:/Mach3");

    // Mach3 kernel speed list position (<PulseSpeed>) in kHz: 25, 35, 45, 60, 65, 75, 100
    static int kernelSpeedKhz(int pulse_speed_index) noexcept;
};

} // namespace rnr::config
