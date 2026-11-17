#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

// Firmware settings as the settings panel shows them (GRBL_FAMILY_PLAN.md section 11):
// GRBL 1.1 $$ with BreakoutCNC's own descriptions, grblHAL's self-described settings ($ES / $EG), and FluidNC's
// configuration tree ($CD).
namespace rnr::board::grbl {

enum class SettingType : uint8_t { Number, Integer, Bool, Mask, Text };

struct SettingItem {
    std::string id;          // "100" ($100) or "/axes/x/steps_per_mm" (FluidNC)
    std::string group;       // "X axis"
    std::string name;        // "Steps per mm"
    std::string unit;        // "mm/min"
    std::string value;       // As the board reported it
    std::string description; // One or two sentences, may be empty
    SettingType type = SettingType::Number;
    bool        needs_restart = false;
    std::string error;       // The board's answer to the last write, empty when accepted
};

struct SettingsSnapshot {
    uint64_t revision = 0;
    bool     loading = false;
    bool     yaml = false;            // FluidNC tree: ids are config paths
    std::string status;               // "Read 34 settings", error text
    std::vector<SettingItem> items;   // Sorted for display
};

// GRBL 1.1 description of a numbered setting, or nullptr when unknown
struct NumberedSettingDef {
    int         number;
    const char* group;
    const char* name;
    const char* unit;
    SettingType type;
    const char* description;
};
const NumberedSettingDef* grbl11SettingDef(int number);

// grblHAL enumeration lines
struct EnumeratedSetting {
    std::string id, group_id, name, unit, format;
    int         type = 6;
    bool        reboot = false;
};
bool parseEnumeratedSetting(const std::string& body, EnumeratedSetting& out);   // [SETTING:...]
bool parseSettingGroup(const std::string& body, std::string& id, std::string& name); // [SETTINGGROUP:...]

// Builds the panel items from $n=value pairs (numbered), using the grblHAL enumeration where present
std::vector<SettingItem> buildNumberedItems(const std::map<int, std::string>& values,
                                            const std::map<std::string, EnumeratedSetting>& enums,
                                            const std::map<std::string, std::string>& group_names);

// FluidNC $CD output (YAML subset) -> one item per leaf, id = "/path/to/key"
std::vector<SettingItem> parseFluidNcConfig(const std::vector<std::string>& lines);

} // namespace rnr::board::grbl
