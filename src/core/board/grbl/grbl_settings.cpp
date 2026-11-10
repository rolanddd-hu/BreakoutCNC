#include "core/board/grbl/grbl_settings.h"
#include <algorithm>
#include <cstdlib>

namespace rnr::board::grbl {

namespace {

const NumberedSettingDef kGrbl11[] = {
    {0,   "General", "Step pulse time", "us", SettingType::Integer, "Length of each step pulse. Most drivers need 3 to 10 us."},
    {1,   "General", "Step idle delay", "ms", SettingType::Integer, "How long the motors stay powered after motion stops. 255 keeps them always on."},
    {2,   "Signals", "Step pulse invert", "axis mask", SettingType::Mask, "Inverts the step signal of the selected axes."},
    {3,   "Signals", "Direction invert", "axis mask", SettingType::Mask, "Reverses the direction of the selected axes."},
    {4,   "Signals", "Invert step enable", "", SettingType::Bool, "Inverts the stepper driver enable output."},
    {5,   "Signals", "Invert limit pins", "", SettingType::Bool, "Use for normally closed limit switches with pull-ups."},
    {6,   "Signals", "Invert probe pin", "", SettingType::Bool, "Use for a normally closed probe."},
    {10,  "General", "Status report options", "mask", SettingType::Mask, "Bit 0: report machine position (MPos) instead of work position. Bit 1: buffer state. BreakoutCNC works with either."},
    {11,  "Motion", "Junction deviation", "mm", SettingType::Number, "How fast corners are taken. Larger is faster and less precise."},
    {12,  "Motion", "Arc tolerance", "mm", SettingType::Number, "How finely the board splits arcs into lines."},
    {13,  "General", "Report in inches", "", SettingType::Bool, "BreakoutCNC converts the board's reports either way."},
    {20,  "Limits & homing", "Soft limits", "", SettingType::Bool, "The board refuses moves outside its travel. Needs homing."},
    {21,  "Limits & homing", "Hard limits", "", SettingType::Bool, "A limit switch stops the machine with an alarm."},
    {22,  "Limits & homing", "Homing cycle", "", SettingType::Bool, "Enables $H homing. Home switches are required."},
    {23,  "Limits & homing", "Homing direction invert", "axis mask", SettingType::Mask, "Selected axes home towards the negative end."},
    {24,  "Limits & homing", "Homing locate feed", "mm/min", SettingType::Number, "Slow feed that finds the switch precisely."},
    {25,  "Limits & homing", "Homing search seek", "mm/min", SettingType::Number, "Fast feed that searches for the switch."},
    {26,  "Limits & homing", "Homing debounce", "ms", SettingType::Integer, "Wait time for the switch to settle."},
    {27,  "Limits & homing", "Homing pull-off", "mm", SettingType::Number, "Distance moved off the switch after homing."},
    {30,  "Spindle & laser", "Maximum spindle speed", "RPM", SettingType::Number, "Spindle speed (S value) for full PWM output. Laser: the S value for full power."},
    {31,  "Spindle & laser", "Minimum spindle speed", "RPM", SettingType::Number, "Lowest spindle speed the PWM output produces."},
    {32,  "Spindle & laser", "Laser mode", "", SettingType::Bool, "The board does not stop at power changes and switches the beam off during rapids. Required for BreakoutCut laser work."},
    {100, "X axis", "X steps per mm", "steps/mm", SettingType::Number, ""},
    {101, "Y axis", "Y steps per mm", "steps/mm", SettingType::Number, ""},
    {102, "Z axis", "Z steps per mm", "steps/mm", SettingType::Number, ""},
    {103, "A axis", "A steps per mm", "steps/mm", SettingType::Number, ""},
    {110, "X axis", "X maximum rate", "mm/min", SettingType::Number, ""},
    {111, "Y axis", "Y maximum rate", "mm/min", SettingType::Number, ""},
    {112, "Z axis", "Z maximum rate", "mm/min", SettingType::Number, ""},
    {113, "A axis", "A maximum rate", "mm/min", SettingType::Number, ""},
    {120, "X axis", "X acceleration", "mm/s2", SettingType::Number, ""},
    {121, "Y axis", "Y acceleration", "mm/s2", SettingType::Number, ""},
    {122, "Z axis", "Z acceleration", "mm/s2", SettingType::Number, ""},
    {123, "A axis", "A acceleration", "mm/s2", SettingType::Number, ""},
    {130, "X axis", "X maximum travel", "mm", SettingType::Number, "Used by soft limits and by BreakoutCNC's travel check."},
    {131, "Y axis", "Y maximum travel", "mm", SettingType::Number, "Used by soft limits and by BreakoutCNC's travel check."},
    {132, "Z axis", "Z maximum travel", "mm", SettingType::Number, "Used by soft limits and by BreakoutCNC's travel check."},
    {133, "A axis", "A maximum travel", "mm", SettingType::Number, ""},
    // grblHAL plasma plugin
    {350, "Plasma", "Plasma mode", "", SettingType::Integer, "0 off, 1 arc voltage, 2 up/down inputs, 3 Arc OK only. Restart the board after changing it."},
    {351, "Plasma", "THC delay", "s", SettingType::Number, "Time from the start of the cut until height control starts."},
    {352, "Plasma", "THC threshold", "V", SettingType::Number, "Allowed deviation from the target voltage before Z is corrected."},
    {353, "Plasma", "THC P gain", "", SettingType::Number, ""},
    {354, "Plasma", "THC I gain", "", SettingType::Number, ""},
    {355, "Plasma", "THC D gain", "", SettingType::Number, ""},
    {356, "Plasma", "Anti-dive threshold", "%", SettingType::Integer, "Velocity anti-dive (VAD): height control holds Z while the speed is below this share of the programmed feed (corners)."},
    {357, "Plasma", "Void override", "%", SettingType::Integer, ""},
    {358, "Plasma", "Arc fail timeout", "s", SettingType::Number, "How long the board waits for Arc OK after M3."},
    {359, "Plasma", "Arc retry delay", "s", SettingType::Number, ""},
    {360, "Plasma", "Arc max retries", "", SettingType::Integer, "Restarts of the torch before the job pauses with \"arc failed\"."},
    {361, "Plasma", "Arc voltage scale", "", SettingType::Number, ""},
    {362, "Plasma", "Arc voltage offset", "", SettingType::Number, ""},
    {363, "Plasma", "Arc height per volt", "mm", SettingType::Number, ""},
    {364, "Plasma", "Arc OK high voltage", "V", SettingType::Number, ""},
    {365, "Plasma", "Arc OK low voltage", "V", SettingType::Number, ""},
    {366, "Plasma", "Arc voltage port", "", SettingType::Integer, "Analog aux input for the arc voltage; -1 none."},
    {367, "Plasma", "Arc OK port", "", SettingType::Integer, "Digital aux input for Arc OK; -1 none."},
    {368, "Plasma", "Cutter down port", "", SettingType::Integer, "Digital aux input for THC down; -1 none."},
    {369, "Plasma", "Cutter up port", "", SettingType::Integer, "Digital aux input for THC up; -1 none."},
    {674, "Plasma", "Plasma options", "mask", SettingType::Mask, ""},
    {682, "Plasma", "Z feed factor", "%", SettingType::Integer, "Speed of the height corrections as a share of the Z maximum rate."},
};

std::vector<std::string> splitBar(const std::string& s) {
    std::vector<std::string> out;
    size_t pos = 0;
    while (true) {
        const size_t bar = s.find('|', pos);
        out.push_back(s.substr(pos, bar == std::string::npos ? std::string::npos : bar - pos));
        if (bar == std::string::npos) break;
        pos = bar + 1;
    }
    return out;
}

SettingType typeFromEnum(int t) {
    switch (t) {
        case 0:  return SettingType::Bool;
        case 1: case 2: case 4: return SettingType::Mask;
        case 3: case 5: return SettingType::Integer;
        case 6:  return SettingType::Number;
        default: return SettingType::Text;
    }
}

int groupOrder(const std::string& g) {
    static const char* order[] = {"General", "Signals", "Motion", "Limits & homing", "Spindle & laser",
                                  "X axis", "Y axis", "Z axis", "A axis", "Plasma"};
    for (int i = 0; i < 10; ++i) {
        if (g == order[i]) return i;
    }
    return 100;
}

} // namespace

const NumberedSettingDef* grbl11SettingDef(int number) {
    for (const auto& d : kGrbl11) {
        if (d.number == number) return &d;
    }
    return nullptr;
}

bool parseEnumeratedSetting(const std::string& body, EnumeratedSetting& out) {
    const auto f = splitBar(body);
    if (f.size() < 5) return false;
    out.id = f[0];
    out.group_id = f[1];
    out.name = f[2];
    out.unit = f[3];
    out.type = std::atoi(f[4].c_str());
    out.format = f.size() > 5 ? f[5] : "";
    out.reboot = f.size() > 8 && f[8] == "1";
    return !out.id.empty();
}

bool parseSettingGroup(const std::string& body, std::string& id, std::string& name) {
    const auto f = splitBar(body);
    if (f.size() < 3) return false;
    id = f[0];
    name = f[2];
    return true;
}

std::vector<SettingItem> buildNumberedItems(const std::map<int, std::string>& values,
                                            const std::map<std::string, EnumeratedSetting>& enums,
                                            const std::map<std::string, std::string>& group_names) {
    std::vector<SettingItem> items;
    for (const auto& [n, v] : values) {
        SettingItem it;
        it.id = std::to_string(n);
        it.value = v;
        const auto e = enums.find(it.id);
        if (e != enums.end()) {
            const auto g = group_names.find(e->second.group_id);
            it.group = (g != group_names.end()) ? g->second : "Other";
            it.name = e->second.name;
            it.unit = e->second.unit;
            it.type = typeFromEnum(e->second.type);
            it.needs_restart = e->second.reboot;
            if (const auto* d = grbl11SettingDef(n)) it.description = d->description;
        } else if (const auto* d = grbl11SettingDef(n)) {
            it.group = d->group;
            it.name = d->name;
            it.unit = d->unit;
            it.type = d->type;
            it.description = d->description;
        } else {
            it.group = "Other";
            it.name = "$" + it.id;
            it.type = SettingType::Text;
        }
        items.push_back(std::move(it));
    }
    std::stable_sort(items.begin(), items.end(), [](const SettingItem& a, const SettingItem& b) {
        const int ga = groupOrder(a.group), gb = groupOrder(b.group);
        if (ga != gb) return ga < gb;
        if (ga == 100 && a.group != b.group) return a.group < b.group;
        return std::atoi(a.id.c_str()) < std::atoi(b.id.c_str());
    });
    return items;
}

std::vector<SettingItem> parseFluidNcConfig(const std::vector<std::string>& lines) {
    std::vector<SettingItem> items;
    std::vector<std::pair<size_t, std::string>> stack; // indentation, key
    for (const auto& raw : lines) {
        std::string line = raw;
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        size_t indent = 0;
        while (indent < line.size() && line[indent] == ' ') ++indent;
        if (indent >= line.size()) continue;
        const std::string body = line.substr(indent);
        if (body[0] == '#' || body[0] == '-' || body[0] == '[' || body[0] == '<' || body == "ok") continue;
        const size_t colon = body.find(':');
        if (colon == std::string::npos) continue;
        const std::string key = body.substr(0, colon);
        std::string value = body.substr(colon + 1);
        while (!value.empty() && value.front() == ' ') value.erase(value.begin());
        while (!stack.empty() && stack.back().first >= indent) stack.pop_back();
        if (value.empty()) {
            stack.push_back({indent, key});
            continue;
        }
        if (value.size() >= 2 && (value.front() == '"' || value.front() == '\'') && value.back() == value.front()) {
            value = value.substr(1, value.size() - 2);
        }
        SettingItem it;
        std::string path;
        for (const auto& s : stack) path += "/" + s.second;
        it.group = stack.empty() ? "General" : path.substr(1);
        path += "/" + key;
        it.id = path;
        it.name = key;
        it.value = value;
        it.type = (value == "true" || value == "false") ? SettingType::Bool : SettingType::Text;
        if (it.type == SettingType::Text) {
            char* end = nullptr;
            std::strtod(value.c_str(), &end);
            if (end && *end == '\0' && !value.empty()) it.type = SettingType::Number;
        }
        // Pins, motor drivers and the board itself only change after a restart
        it.needs_restart = key.find("pin") != std::string::npos || path.find("/motor") != std::string::npos ||
                           path.rfind("/board", 0) == 0;
        items.push_back(std::move(it));
    }
    return items;
}

} // namespace rnr::board::grbl
