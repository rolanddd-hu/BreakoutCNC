#include "core/config/mach3_xml_importer.h"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>

namespace rnr::config {

namespace {

std::string toLowerAscii(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

std::string trimWhitespace(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(start, end - start);
}

std::string decodeEntities(const std::string& s) {
    if (s.find('&') == std::string::npos) return s;
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '&') {
            static const std::pair<const char*, char> kEntities[] = {
                {"&amp;", '&'}, {"&lt;", '<'}, {"&gt;", '>'}, {"&quot;", '"'}, {"&apos;", '\''}};
            bool matched = false;
            for (const auto& [ent, ch] : kEntities) {
                const size_t n = std::char_traits<char>::length(ent);
                if (s.compare(i, n, ent) == 0) {
                    out += ch;
                    i += n - 1;
                    matched = true;
                    break;
                }
            }
            if (matched) continue;
        }
        out += s[i];
    }
    return out;
}

std::string fmt(const char* f, ...) {
    char buf[512];
    va_list args;
    va_start(args, f);
    std::vsnprintf(buf, sizeof(buf), f, args);
    va_end(args);
    return buf;
}

// ----------------------------------------------------------------------------
// Mach3 profile XML: <profile> holds sections (<Preferences>, <RnRMotion>, <VfdDriver>, <PlugIns>, ...)
// whose children are leaf values. Keys are kept per section because names repeat across sections
// (e.g. <ComPort> exists in <Preferences> for Mach3's own serial port and in <VfdDriver>).
// ----------------------------------------------------------------------------
using KeyMap = std::unordered_map<std::string, std::string>;

struct ProfileXml {
    std::string root;                                // Lower-case name of the outermost element
    std::unordered_map<std::string, KeyMap> sections; // Lower-case section -> lower-case key -> value
};

ProfileXml parseProfileXml(const std::string& xml) {
    ProfileXml doc;
    std::vector<std::string> stack;
    size_t pos = 0;
    const size_t len = xml.size();

    auto sectionName = [&]() -> std::string {
        if (stack.size() >= 2) return stack[1];
        return stack.empty() ? std::string() : stack[0];
    };

    while (pos < len) {
        const size_t lt = xml.find('<', pos);
        if (lt == std::string::npos) break;

        // Comments, processing instructions and declarations
        if (xml.compare(lt, 4, "<!--") == 0) {
            const size_t end = xml.find("-->", lt + 4);
            if (end == std::string::npos) break;
            pos = end + 3;
            continue;
        }
        if (lt + 1 < len && (xml[lt + 1] == '?' || xml[lt + 1] == '!')) {
            const size_t gt = xml.find('>', lt + 2);
            if (gt == std::string::npos) break;
            pos = gt + 1;
            continue;
        }

        const size_t gt = xml.find('>', lt + 1);
        if (gt == std::string::npos) break;

        // Closing tag of a container
        if (xml[lt + 1] == '/') {
            const std::string name = toLowerAscii(trimWhitespace(xml.substr(lt + 2, gt - lt - 2)));
            for (size_t i = stack.size(); i-- > 0;) {
                if (stack[i] == name) {
                    stack.resize(i);
                    break;
                }
            }
            pos = gt + 1;
            continue;
        }

        if (xml[gt - 1] == '/') { // <Tag/>
            pos = gt + 1;
            continue;
        }

        std::string name = trimWhitespace(xml.substr(lt + 1, gt - lt - 1));
        const size_t sp = name.find_first_of(" \t\r\n");
        if (sp != std::string::npos) name = name.substr(0, sp);
        name = toLowerAscii(name);
        if (name.empty()) {
            pos = gt + 1;
            continue;
        }
        if (doc.root.empty()) doc.root = name;

        // Leaf value: <Tag>text</Tag>
        const size_t next_lt = xml.find('<', gt + 1);
        if (next_lt != std::string::npos && next_lt + 1 < len && xml[next_lt + 1] == '/') {
            const size_t close_gt = xml.find('>', next_lt + 2);
            if (close_gt != std::string::npos &&
                toLowerAscii(trimWhitespace(xml.substr(next_lt + 2, close_gt - next_lt - 2))) == name) {
                if (!stack.empty()) {
                    doc.sections[sectionName()][name] =
                        decodeEntities(trimWhitespace(xml.substr(gt + 1, next_lt - gt - 1)));
                }
                pos = close_gt + 1;
                continue;
            }
        }

        stack.push_back(name);
        pos = gt + 1;
    }
    return doc;
}

// Typed, case-insensitive access to one section
class Section {
public:
    explicit Section(const KeyMap* map = nullptr) : map_(map) {}

    bool exists() const noexcept { return map_ != nullptr; }

    bool str(const std::string& key, std::string& out) const {
        if (!map_) return false;
        auto it = map_->find(toLowerAscii(key));
        if (it == map_->end() || it->second.empty()) return false;
        out = it->second;
        return true;
    }

    bool num(const std::string& key, double& out) const {
        std::string s;
        if (!str(key, s)) return false;
        std::replace(s.begin(), s.end(), ',', '.'); // European locale exports
        try {
            size_t idx = 0;
            const double v = std::stod(s, &idx);
            if (idx == 0 || !std::isfinite(v)) return false;
            out = v;
            return true;
        } catch (...) {
            return false;
        }
    }

    bool integer(const std::string& key, int& out) const {
        double d = 0.0;
        if (!num(key, d) || std::abs(d) > 1e9) return false;
        out = static_cast<int>(std::llround(d));
        return true;
    }

    bool flag(const std::string& key, bool& out) const {
        std::string s;
        if (!str(key, s)) return false;
        const std::string low = toLowerAscii(s);
        if (low == "true" || low == "yes" || low == "on") { out = true; return true; }
        if (low == "false" || low == "no" || low == "off") { out = false; return true; }
        int v = 0;
        if (!integer(key, v)) return false;
        out = (v != 0);
        return true;
    }

private:
    const KeyMap* map_;
};

constexpr const char* kAxisNames[4] = {"X", "Y", "Z", "A"};

// Mach3 input signal numbers (Config -> Ports & Pins -> Input Signals)
constexpr int kSigProbe = 22;
constexpr int kSigEStop = 25;

// Suggested RNR terminal for the usual parallel-port input pins
uint8_t suggestedLptTerminal(int port, int pin) {
    if (port != 1) return 0;
    switch (pin) {
        case 10: return 1;
        case 11: return 2;
        case 12: return 3;
        case 13: return 4;
        case 15: return 5;
        default: return 0;
    }
}

} // namespace

int Mach3XmlImporter::kernelSpeedKhz(int pulse_speed_index) noexcept {
    static constexpr int kTable[] = {25, 35, 45, 60, 65, 75, 100};
    if (pulse_speed_index < 0 || pulse_speed_index >= static_cast<int>(std::size(kTable))) return 0;
    return kTable[pulse_speed_index];
}

std::vector<std::string> Mach3XmlImporter::discoverLocalMach3Profiles(const std::string& search_dir) {
    std::vector<std::string> found;
    std::error_code ec;
    if (!std::filesystem::exists(search_dir, ec) || !std::filesystem::is_directory(search_dir, ec)) {
        return found;
    }
    for (const auto& entry : std::filesystem::directory_iterator(search_dir, ec)) {
        if (entry.is_regular_file(ec)) {
            auto ext = toLowerAscii(entry.path().extension().string());
            if (ext == ".xml") {
                found.push_back(entry.path().string());
            }
        }
    }
    std::sort(found.begin(), found.end());
    return found;
}

Mach3MigrationReport Mach3XmlImporter::importFromFile(
    const std::string& xml_file_path,
    const core::MachineConfig& base_config,
    const Mach3ImportOptions& options)
{
    std::ifstream ifs(xml_file_path, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        Mach3MigrationReport rep;
        rep.success = false;
        rep.source_file_path = xml_file_path;
        rep.error_message = "Could not open Mach3 XML file: " + xml_file_path;
        return rep;
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    std::string fname = std::filesystem::path(xml_file_path).filename().string();
    auto rep = importFromString(oss.str(), base_config, options, fname);
    rep.source_file_path = xml_file_path;
    return rep;
}

Mach3MigrationReport Mach3XmlImporter::importFromString(
    const std::string& xml_content,
    const core::MachineConfig& base_config,
    const Mach3ImportOptions& options,
    const std::string& source_name)
{
    Mach3MigrationReport report;
    report.source_file_path = source_name;
    report.profile_name     = source_name;
    report.migrated_config  = base_config;

    const ProfileXml doc = parseProfileXml(xml_content);
    auto sectionOf = [&](const char* name) {
        auto it = doc.sections.find(name);
        return Section(it == doc.sections.end() ? nullptr : &it->second);
    };
    const Section P = sectionOf("preferences"); // Mach3 itself
    const Section R = sectionOf("rnrmotion");   // RNR Mach3 plugin
    const Section V = sectionOf("vfddriver");   // Huanyang VFD plugin (royaumedeole.fr)
    const Section G = sectionOf("plugins");     // Plugin enable flags

    if (doc.root != "profile" || !P.exists()) {
        report.success = false;
        report.error_message = "This is not a Mach3 profile: expected a <profile> file with a <Preferences> section "
                               "(e.g. C:\\Mach3\\Mach3Mill.xml).";
        return report;
    }

    core::MachineConfig& cfg = report.migrated_config;
    auto add = [&](const std::string& cat, const std::string& name, const std::string& raw,
                   const std::string& migrated, MigrationSeverity sev, const std::string& note) {
        report.items.push_back({cat, name, raw, migrated, sev, note});
    };
    auto warn = [&](const std::string& msg) { report.safety_warnings.push_back(msg); };

    // ------------------------------------------------------------------------
    // 1. Profile name & machine type
    // ------------------------------------------------------------------------
    std::string prof_str;
    if (P.str("Profile", prof_str)) {
        report.profile_name = prof_str;
    }
    const std::string low_prof = toLowerAscii(report.profile_name + " " + source_name);
    bool plasma_mode = false;
    bool thc_active  = false;
    P.flag("PlasmaMode", plasma_mode);
    P.flag("Input26Active", thc_active); // THC On / Arc OK input in use
    report.detected_lathe_profile  = low_prof.find("lathe") != std::string::npos || low_prof.find("turn") != std::string::npos;
    report.detected_plasma_profile = !report.detected_lathe_profile &&
                                     (low_prof.find("plasma") != std::string::npos || plasma_mode || thc_active);
    report.detected_mill_profile   = !report.detected_lathe_profile && !report.detected_plasma_profile;
    report.detected_rnr_profile    = R.exists();
    if (report.detected_lathe_profile) {
        add("Units & Core", "Machine Type", report.profile_name, "Lathe profile", MigrationSeverity::Warning,
            "Lathe profiles (X as diameter, turning axes) do not map onto a mill or plasma machine. Check every value.");
        warn("This looks like a Mach3 LATHE profile. BreakoutCNC drives mills, routers and plasma tables: review every "
             "imported value before moving the machine.");
    }

    // ------------------------------------------------------------------------
    // 2. Native units (<Units>: 0 = mm, 1 = inch)
    // ------------------------------------------------------------------------
    int units_val = 0;
    const bool has_units = P.integer("Units", units_val);
    bool is_imperial = has_units && units_val == 1;
    std::string units_note = has_units ? "Mach3 Config -> Select Native Units." : "No <Units> in the profile: assuming mm.";
    MigrationSeverity units_sev = has_units ? MigrationSeverity::Ok : MigrationSeverity::Warning;
    if (options.units_override == 0 || options.units_override == 1) {
        is_imperial = (options.units_override == 1);
        units_note = "Forced by the import option (the profile says " +
                     std::string(has_units ? (units_val == 1 ? "inches" : "mm") : "nothing") + ").";
        units_sev = MigrationSeverity::AutoCorrected;
    }
    report.detected_imperial_units = is_imperial;
    cfg.linear_units = is_imperial ? core::LinearUnits::Imperial : core::LinearUnits::Metric;
    const double unit_to_mm = is_imperial ? 25.4 : 1.0;
    const char* unit_name = is_imperial ? "in" : "mm";
    add("Units & Core", "Native Units",
        has_units ? fmt("%d (%s)", units_val, units_val == 1 ? "inches" : "mm") : "missing",
        is_imperial ? "Inches (all values converted with 25.4)" : "Millimeters", units_sev, units_note);

    // ------------------------------------------------------------------------
    // 3. Gantry slaving (<Slave0..2> for X/Y/Z: 0 = A, 1 = B, 2 = C, 3 = none)
    // ------------------------------------------------------------------------
    int slave_master = -1;
    {
        int s[3] = {3, 3, 3};
        bool any = false;
        for (int i = 0; i < 3; ++i) {
            any |= P.integer(fmt("Slave%d", i), s[i]);
        }
        if (any) {
            std::string raw = fmt("X:%d Y:%d Z:%d", s[0], s[1], s[2]);
            if (s[0] == 0) slave_master = 0;
            else if (s[1] == 0) slave_master = 1;
            cfg.gantry_slave = (slave_master == 0) ? core::GantrySlave::AFollowsX
                             : (slave_master == 1) ? core::GantrySlave::AFollowsY
                                                   : core::GantrySlave::None;
            std::string note;
            MigrationSeverity sev = MigrationSeverity::Ok;
            if (s[2] == 0) {
                sev = MigrationSeverity::Warning;
                note = "Mach3 slaved A to Z, which BreakoutCNC does not support: A is left independent.";
                warn(note);
            } else if ((s[0] == 1 || s[0] == 2) || (s[1] == 1 || s[1] == 2) || (s[2] == 1 || s[2] == 2)) {
                sev = MigrationSeverity::Warning;
                note = "Mach3 slaved a B or C motor; BreakoutCNC only has the A motor output for a second gantry motor.";
                warn(note);
            }
            add("Units & Core", "Dual-Motor Gantry", raw,
                slave_master == 0 ? "A follows X" : (slave_master == 1 ? "A follows Y" : "None"), sev, note);
        }
    }

    // ------------------------------------------------------------------------
    // 4. Per-axis tuning, polarity, homing & soft limits (motors 0..3 = X, Y, Z, A)
    // ------------------------------------------------------------------------
    bool axis_tuned[4] = {false, false, false, false};
    bool soft_limits_on = false;
    const bool has_soft_flag = P.flag("SoftLimit", soft_limits_on);

    for (int a = 0; a < 4; ++a) {
        auto& ax = cfg.axes[a];
        const std::string axn = kAxisNames[a];
        const std::string cat = "Axis " + axn;

        bool motor_active = true;
        P.flag(fmt("Motor%dActive", a), motor_active);

        double steps = 0.0, vel = 0.0, acc = 0.0;
        const bool has_steps = P.num(fmt("Steps%d", a), steps) && steps > 0.0;
        const bool has_vel   = P.num(fmt("Vel%d", a), vel) && vel > 0.0;
        const bool has_acc   = P.num(fmt("Acc%d", a), acc) && acc > 0.0;
        axis_tuned[a] = has_steps;

        if (!has_steps) {
            if (a == 3 && slave_master >= 0) {
                // Handled below: the slaved A motor takes the master's tuning
            } else if (!motor_active) {
                add(cat, axn + " Motor", "Not enabled", "Unchanged", MigrationSeverity::Ok,
                    "Motor output not enabled in Mach3.");
                continue;
            } else {
                add(cat, axn + " Motor Tuning", "Missing", "Unchanged", MigrationSeverity::Warning,
                    "The motor is enabled but has no Motor Tuning values.");
            }
        }

        if (has_steps) {
            double steps_mm = steps / unit_to_mm;
            MigrationSeverity sev = MigrationSeverity::Ok;
            std::string note = is_imperial ? "Steps per inch / 25.4." : "";
            if (steps_mm < 1.0 || steps_mm > 25000.0) {
                sev = MigrationSeverity::Warning;
                note = "Unusual resolution: check the native units and the driver microstepping.";
                steps_mm = std::clamp(steps_mm, 1.0, 50000.0);
                warn(fmt("%s steps/mm (%.3f) is outside the usual range.", cat.c_str(), steps_mm));
            }
            ax.steps_per_mm = steps_mm;
            add(cat, axn + " Steps per Unit", fmt("%.6g steps/%s", steps, unit_name), fmt("%.4f steps/mm", steps_mm), sev, note);
            if (!motor_active) {
                add(cat, axn + " Motor", "Not enabled", "Imported anyway", MigrationSeverity::Warning,
                    "Mach3 has tuning for this motor but its output is disabled.");
            }
        }

        if (has_vel) {
            bool per_min = (options.velocity_unit_mode == 2);
            std::string note;
            MigrationSeverity sev = MigrationSeverity::Ok;
            if (options.velocity_unit_mode == 0 && ((!is_imperial && vel > 500.0) || (is_imperial && vel > 25.0))) {
                per_min = true;
                sev = MigrationSeverity::AutoCorrected;
                note = "Value looks like units/min (Mach3 stores units/sec); divided by 60.";
            }
            const double vel_mm_s = (per_min ? vel / 60.0 : vel) * unit_to_mm;
            ax.max_velocity_mm_s = std::clamp(vel_mm_s, 1.0, 1000.0);
            add(cat, axn + " Max Velocity", fmt("%.6g %s/%s", vel, unit_name, per_min ? "min" : "s"),
                fmt("%.0f mm/min", ax.max_velocity_mm_s * 60.0), sev, note);
        }

        if (has_acc) {
            double acc_mm = acc * unit_to_mm;
            MigrationSeverity sev = MigrationSeverity::Ok;
            std::string note;
            if (acc_mm > 5000.0) {
                sev = MigrationSeverity::Warning;
                note = "Above 5000 mm/s^2; capped.";
                warn(fmt("%s acceleration (%.0f mm/s^2) was capped at 5000 mm/s^2.", cat.c_str(), acc_mm));
                acc_mm = 5000.0;
            }
            ax.acceleration_mm_s2 = std::max(5.0, acc_mm);
            ax.deceleration_mm_s2 = ax.acceleration_mm_s2;
            add(cat, axn + " Acceleration", fmt("%.6g %s/s^2", acc, unit_name), fmt("%.0f mm/s^2", ax.acceleration_mm_s2), sev, note);
        }

        // Step / Dir polarity (Ports & Pins -> Motor Outputs) and Homing/Limits "Reversed"
        auto flagWithLegacy = [&](const std::string& key, const std::string& legacy, bool& out, bool& disagree) {
            bool v = false, lv = false;
            const bool has = P.flag(key, v);
            const bool has_legacy = P.flag(legacy, lv);
            disagree = has && has_legacy && v != lv;
            if (has) { out = v; return true; }
            if (has_legacy) { out = lv; return true; }
            return false;
        };
        bool step_neg = ax.step_active_low, dir_neg = false, rev = false;
        bool d1 = false, d2 = false, d3 = false;
        const bool has_step = flagWithLegacy(fmt("Motor%dStepNeg", a), axn + "StepNeg", step_neg, d1);
        const bool has_dir  = flagWithLegacy(fmt("Motor%dDirNeg", a), axn + "DirNeg", dir_neg, d2);
        const bool has_rev  = flagWithLegacy(fmt("M%dRev", a), axn + "Rev", rev, d3);
        if (has_step) ax.step_active_low = step_neg;
        if (has_dir || has_rev) {
            ax.dir_active_low = (options.combine_reversed_with_dir && rev) ? !dir_neg : dir_neg;
        }
        if (has_step || has_dir || has_rev) {
            MigrationSeverity sev = (rev && options.combine_reversed_with_dir) ? MigrationSeverity::AutoCorrected : MigrationSeverity::Ok;
            std::string note = (rev && options.combine_reversed_with_dir)
                ? "Mach3 'Reversed' is folded into Dir Invert so the axis moves the same way."
                : "Verify the travel direction in Stage 2 of the machine test.";
            if (d1 || d2 || d3) {
                sev = MigrationSeverity::Warning;
                note += " The profile also has older " + axn + "StepNeg/DirNeg/Rev entries that disagree; the Motor Outputs values were used.";
            }
            add(cat, axn + " Step/Dir Polarity",
                fmt("StepLow=%d DirLow=%d Reversed=%d", step_neg ? 1 : 0, dir_neg ? 1 : 0, rev ? 1 : 0),
                fmt("Step invert %s, Dir invert %s", ax.step_active_low ? "ON" : "OFF", ax.dir_active_low ? "ON" : "OFF"),
                sev, note);
        }

        // A slaved gantry motor homes, moves and is limited exactly like its master (copied below)
        if (a == 3 && slave_master >= 0) continue;

        // Homing direction & speed (Homing/Limits: Home Neg, Speed %)
        bool home_neg = false, dh = false;
        if (flagWithLegacy(fmt("M%dNeg", a), axn + "Neg", home_neg, dh)) {
            ax.homing_dir_negative = home_neg;
            MigrationSeverity sev = MigrationSeverity::Ok;
            std::string note;
            if (a == 2 && home_neg) {
                sev = MigrationSeverity::Warning;
                note = "Z homes DOWN (toward the table). Most machines home Z up: check the switch position.";
                warn("Z homes in the negative direction (down). Verify that the Z home switch really is at the bottom before homing.");
            }
            add(cat, axn + " Homing Direction", home_neg ? "Home Neg = 1" : "Home Neg = 0",
                home_neg ? "Toward -" : "Toward +", sev, note);
        }
        double home_pct = 0.0;
        if (P.num(fmt("RefSpeed%d", a), home_pct) && home_pct > 0.0) {
            const double pct = std::min(home_pct, 100.0);
            ax.homing_feed_mm_min = std::clamp(pct / 100.0 * ax.max_velocity_mm_s * 60.0, 50.0, 5000.0);
            add(cat, axn + " Homing Speed", fmt("%.0f%% of max velocity", home_pct),
                fmt("%.0f mm/min", ax.homing_feed_mm_min), MigrationSeverity::Ok, "");
        }

        // Home pull-off (RNR plugin, native units)
        double pull = 0.0;
        if (R.num(std::string(axn) + "PullOff", pull) && pull > 0.0) {
            ax.home_pulloff_mm = std::clamp(pull * unit_to_mm, 0.2, 50.0);
            add(cat, axn + " Home Pull-Off", fmt("%.4g %s", pull, unit_name), fmt("%.2f mm", ax.home_pulloff_mm),
                MigrationSeverity::Ok, "From the RNR plugin settings.");
        }

        // Soft limits (Homing/Limits: Soft Min / Soft Max, switched on by <SoftLimit>)
        double smin = 0.0, smax = 0.0;
        const bool has_min = P.num(fmt("M%dMin", a), smin);
        const bool has_max = P.num(fmt("M%dMax", a), smax);
        if (has_min && has_max) {
            double min_mm = smin * unit_to_mm;
            double max_mm = smax * unit_to_mm;
            MigrationSeverity sev = MigrationSeverity::Ok;
            std::string note;
            if (min_mm > max_mm && options.auto_fix_inverted_soft_limits) {
                std::swap(min_mm, max_mm);
                sev = MigrationSeverity::AutoCorrected;
                note = "Soft Min was above Soft Max; swapped.";
                warn(fmt("%s soft limits were inverted in Mach3 and have been swapped.", cat.c_str()));
            }
            ax.soft_limit_min_mm = min_mm;
            ax.soft_limit_max_mm = max_mm;
            ax.soft_limits_enabled = has_soft_flag && soft_limits_on && (max_mm - min_mm) > 1.0;
            if ((max_mm - min_mm) > 1.0) {
                ax.homing_max_travel_mm = std::clamp((max_mm - min_mm) + 20.0, 50.0, 5000.0);
            }
            if (!ax.soft_limits_enabled) {
                note += (note.empty() ? "" : " ") + std::string("Soft limits are off in Mach3; values kept but not enforced.");
            }
            add(cat, axn + " Soft Limits", fmt("[%.4g .. %.4g] %s", smin, smax, unit_name),
                fmt("[%.1f .. %.1f] mm, %s", ax.soft_limit_min_mm, ax.soft_limit_max_mm, ax.soft_limits_enabled ? "on" : "off"),
                sev, note);
        }
    }

    // The slaved A motor has no tuning of its own in Mach3: it moves exactly like its master
    if (slave_master >= 0) {
        auto& a_ax = cfg.axes[3];
        const auto& m = cfg.axes[static_cast<size_t>(slave_master)];
        a_ax.steps_per_mm        = m.steps_per_mm;
        a_ax.max_velocity_mm_s   = m.max_velocity_mm_s;
        a_ax.acceleration_mm_s2  = m.acceleration_mm_s2;
        a_ax.deceleration_mm_s2  = m.deceleration_mm_s2;
        a_ax.soft_limit_min_mm   = m.soft_limit_min_mm;
        a_ax.soft_limit_max_mm   = m.soft_limit_max_mm;
        a_ax.soft_limits_enabled = false; // The master's limits protect the gantry
        a_ax.homing_feed_mm_min  = m.homing_feed_mm_min;
        a_ax.homing_dir_negative = m.homing_dir_negative;
        add("Axis A", "A Motor Tuning", axis_tuned[3] ? "Own values" : "None (slaved)",
            fmt("Copied from %s", kAxisNames[slave_master]), MigrationSeverity::AutoCorrected,
            "The second gantry motor copies its master's steps, speed and acceleration; only its Step/Dir polarity is its own.");
    }

    // ------------------------------------------------------------------------
    // 5. Step pulse rate
    // ------------------------------------------------------------------------
    double needed_khz = 0.0;
    for (int a = 0; a < 4; ++a) {
        if (!axis_tuned[a] && !(a == 3 && slave_master >= 0)) continue;
        needed_khz = std::max(needed_khz, cfg.axes[a].steps_per_mm * cfg.axes[a].max_velocity_mm_s / 1000.0);
    }
    int pulse_rates = 0;
    if (R.integer("PulseRates", pulse_rates) && pulse_rates > 0) {
        cfg.pulse_rate_khz = static_cast<uint8_t>(std::clamp(pulse_rates, 10, 100));
        add("Units & Core", "Step Pulse Rate", fmt("%d kHz (RNR plugin)", pulse_rates), fmt("%u kHz", cfg.pulse_rate_khz),
            MigrationSeverity::Ok, "Same rate the RNR board used under Mach3.");
    } else {
        int idx = -1;
        const int kernel = P.integer("PulseSpeed", idx) ? kernelSpeedKhz(idx) : 0;
        // Keep the kernel speed the drives were used with, but never below what the fastest axis needs (+25%)
        int target = std::max(kernel, static_cast<int>(std::ceil(needed_khz * 1.25)));
        target = std::clamp(((target + 4) / 5) * 5, 10, 100);
        cfg.pulse_rate_khz = static_cast<uint8_t>(target);
        add("Units & Core", "Step Pulse Rate",
            kernel > 0 ? fmt("Kernel %d kHz (PulseSpeed=%d)", kernel, idx) : "Not set",
            fmt("%u kHz", cfg.pulse_rate_khz), MigrationSeverity::Ok,
            fmt("Fastest axis needs %.1f kHz at its max velocity.", needed_khz));
    }
    for (int a = 0; a < 4; ++a) {
        auto& ax = cfg.axes[a];
        const double max_hz = cfg.pulse_rate_khz * 1000.0 * 0.95;
        if (ax.steps_per_mm * ax.max_velocity_mm_s > max_hz) {
            const double capped = max_hz / std::max(1.0, ax.steps_per_mm);
            const std::string msg = fmt("%s max velocity capped to %.0f mm/min by the %u kHz step rate.",
                                        kAxisNames[a], capped * 60.0, cfg.pulse_rate_khz);
            if (axis_tuned[a] || (a == 3 && slave_master >= 0)) {
                add("Axis " + std::string(kAxisNames[a]), std::string(kAxisNames[a]) + " Velocity vs Step Rate",
                    fmt("%.0f mm/min", ax.max_velocity_mm_s * 60.0), fmt("%.0f mm/min", capped * 60.0),
                    MigrationSeverity::Warning, msg);
                warn(msg);
            }
            ax.max_velocity_mm_s = std::max(1.0, capped);
        }
    }

    // ------------------------------------------------------------------------
    // 6. Input signals. With the RNR plugin only Port 3 inputs reached the board; parallel-port profiles
    //    get a wiring plan (suggested terminal per signal, changeable by the operator).
    // ------------------------------------------------------------------------
    bool any_port3 = false;
    for (int i = 0; i < 31; ++i) {
        bool act = false;
        int port = 0;
        if (P.flag(fmt("Input%dActive", i), act) && act && P.integer(fmt("Input%dPort", i), port) && port == 3) {
            any_port3 = true;
        }
    }
    const bool rnr_inputs = report.detected_rnr_profile || any_port3;
    report.detected_rnr_profile = rnr_inputs;

    auto importSignal = [&](int sig, const std::string& label, uint8_t& out_pin, bool& out_invert) {
        int pin = 0;
        if (!P.integer(fmt("Input%dPin", sig), pin)) return; // Not in the profile: keep the current setting
        bool active = false, neg = false, emulated = false;
        int port = 0;
        P.flag(fmt("Input%dActive", sig), active);
        P.integer(fmt("Input%dPort", sig), port);
        P.flag(fmt("Input%dNeg", sig), neg);
        P.flag(fmt("Input%dEmulated", sig), emulated);

        out_invert = neg;
        const std::string raw = active ? fmt("Port %d Pin %d%s", port, pin, neg ? " (Active Low)" : "") : "Not used";
        if (!active || pin <= 0) {
            out_pin = 0;
            const bool critical = (sig == kSigEStop);
            add("Input Pins", label, raw, "Disabled", critical ? MigrationSeverity::Warning : MigrationSeverity::Ok,
                critical ? "No E-Stop input in the Mach3 profile." : "Not used in Mach3.");
            return;
        }
        if (emulated) {
            out_pin = 0;
            add("Input Pins", label, raw + " (keyboard emulated)", "Disabled", MigrationSeverity::Warning,
                "Mach3 emulated this signal with a key, there is no switch to wire.");
            return;
        }
        if (rnr_inputs) {
            if (port == 3 && pin >= 1 && pin <= 12) {
                out_pin = static_cast<uint8_t>(pin);
                add("Input Pins", label, raw, fmt("IN%d%s", pin, neg ? " (inverted)" : ""), MigrationSeverity::Ok, "");
            } else {
                out_pin = 0;
                add("Input Pins", label, raw, "Disabled", MigrationSeverity::Warning,
                    "The RNR plugin only used Port 3 inputs, so Mach3 ignored this one.");
            }
            return;
        }

        // Parallel-port profile: wiring plan
        report.detected_lpt_parallel_port = true;
        uint8_t chosen = suggestedLptTerminal(port, pin);
        const bool overridden = options.lpt_pin_overrides.count(label) > 0;
        if (overridden) chosen = std::min<uint8_t>(options.lpt_pin_overrides.at(label), 12);
        out_pin = chosen;
        report.lpt_remaps.push_back({label, port, pin, chosen, neg});
        add("Input Pins", label, raw, chosen ? fmt("IN%u%s", chosen, neg ? " (inverted)" : "") : "Disabled",
            MigrationSeverity::Warning,
            chosen ? fmt("Parallel-port input: wire this switch to IN%u%s. Re-check its polarity in Stage 1 of the test.",
                         chosen, overridden ? " (your choice)" : " (suggested)")
                   : "Parallel-port input without an RNR terminal: pick one in the wiring plan or leave it off.");
    };

    importSignal(0,  "X+ Limit", cfg.axes[0].limit_plus_pin,  cfg.axes[0].limit_plus_invert);
    importSignal(1,  "X- Limit", cfg.axes[0].limit_minus_pin, cfg.axes[0].limit_minus_invert);
    importSignal(2,  "X Home",   cfg.axes[0].home_pin,        cfg.axes[0].home_invert);
    importSignal(3,  "Y+ Limit", cfg.axes[1].limit_plus_pin,  cfg.axes[1].limit_plus_invert);
    importSignal(4,  "Y- Limit", cfg.axes[1].limit_minus_pin, cfg.axes[1].limit_minus_invert);
    importSignal(5,  "Y Home",   cfg.axes[1].home_pin,        cfg.axes[1].home_invert);
    importSignal(6,  "Z+ Limit", cfg.axes[2].limit_plus_pin,  cfg.axes[2].limit_plus_invert);
    importSignal(7,  "Z- Limit", cfg.axes[2].limit_minus_pin, cfg.axes[2].limit_minus_invert);
    importSignal(8,  "Z Home",   cfg.axes[2].home_pin,        cfg.axes[2].home_invert);
    importSignal(9,  "A+ Limit", cfg.axes[3].limit_plus_pin,  cfg.axes[3].limit_plus_invert);
    importSignal(10, "A- Limit", cfg.axes[3].limit_minus_pin, cfg.axes[3].limit_minus_invert);
    importSignal(11, "A Home",   cfg.axes[3].home_pin,        cfg.axes[3].home_invert);
    importSignal(kSigProbe, "Probe (G31)", cfg.probe.probe_pin, cfg.probe.probe_invert);
    importSignal(kSigEStop, "E-Stop", cfg.estop_pin, cfg.estop_invert);
    importSignal(26, "THC Arc OK", cfg.thc.arc_ok_pin,   cfg.thc.arc_ok_invert);
    importSignal(27, "THC Up",     cfg.thc.thc_up_pin,   cfg.thc.thc_up_invert);
    importSignal(28, "THC Down",   cfg.thc.thc_down_pin, cfg.thc.thc_down_invert);

    if (cfg.estop_pin == 0) {
        warn("No hardware E-Stop input after the import. Wire an E-Stop switch and assign it before running the machine.");
    }

    // Limit override is never imported: BreakoutCNC always starts with the end-stops active
    bool lim_ov = false;
    if (P.flag("LimitOverRide", lim_ov) && lim_ov) {
        add("Input Pins", "Limit Override", "On", "Not imported", MigrationSeverity::Warning,
            "BreakoutCNC always starts with the end-stops active; use the Limit Override button when needed.");
    }

    // ------------------------------------------------------------------------
    // 7. Spindle (delays, RNR PWM ratio, active pulley speeds)
    // ------------------------------------------------------------------------
    double spin_up = 0.0;
    if (P.num("SpinDelayCW", spin_up) && spin_up >= 0.0) {
        cfg.spindle_ramp_up_s = std::clamp(spin_up, 0.0, 30.0);
        add("Spindle & VFD", "Spin-Up Delay", fmt("%.1f s", spin_up), fmt("%.1f s", cfg.spindle_ramp_up_s), MigrationSeverity::Ok, "");
    }
    double spin_down = 0.0;
    if (P.num("SpinDownCW", spin_down) && spin_down >= 0.0) {
        cfg.spindle_ramp_down_s = std::clamp(spin_down, 0.0, 30.0);
        add("Spindle & VFD", "Spin-Down Delay", fmt("%.1f s", spin_down), fmt("%.1f s", cfg.spindle_ramp_down_s), MigrationSeverity::Ok, "");
    }
    double sp_ratio = 0.0;
    if (R.num("SpindleRatio", sp_ratio) && sp_ratio > 0.0) {
        cfg.spindle_pwm_ratio = static_cast<float>(std::clamp(sp_ratio, 0.1, 1.0));
        add("Spindle & VFD", "Spindle PWM Ratio", fmt("%.2f", sp_ratio), fmt("%.0f%%", cfg.spindle_pwm_ratio * 100.0f),
            MigrationSeverity::Ok, "From the RNR plugin settings.");
    }

    int pulley_idx = -1;
    double pulley_max = 0.0, pulley_min = 0.0;
    const bool has_pulley = P.integer("PULLEY", pulley_idx) && pulley_idx >= 0 &&
                            P.num(fmt("SPEED%d", pulley_idx + 1), pulley_max) && pulley_max >= 100.0;
    if (has_pulley) {
        P.num(fmt("LSPEED%d", pulley_idx + 1), pulley_min);
    }

    // ------------------------------------------------------------------------
    // 8. Huanyang VFD plugin (<VfdDriver>, royaumedeole.fr)
    // ------------------------------------------------------------------------
    if (V.exists()) {
        report.detected_vfd_plugin = true;
        bool plugin_on = false;
        G.flag("VFD-Driver-by-royaumedeole.fr", plugin_on);
        auto& vfd = cfg.vfd;

        int com = 0;
        // COM numbers only mean something on Windows; elsewhere keep the configured device path
#ifdef _WIN32
        if (V.integer("ComPort", com) && com >= 1 && com <= 255) {
            vfd.port_name = fmt("COM%d", com);
        }
        const char* port_note = "Check that the USB-RS485 adapter got the same COM number on this PC.";
#else
        V.integer("ComPort", com);
        const char* port_note = "Mach3 used a Windows COM port: select the adapter's device (e.g. /dev/ttyUSB0) in the VFD settings.";
#endif
        int baud = 9600;
        std::string baud_note;
        MigrationSeverity baud_sev = MigrationSeverity::Ok;
        if (V.integer("BaudRate", baud)) {
            if (baud == 4800 || baud == 9600 || baud == 19200 || baud == 38400) {
                vfd.baud_rate = static_cast<uint32_t>(baud);
            } else {
                baud_sev = MigrationSeverity::Warning;
                baud_note = "Not a Huanyang baud rate (PD164: 4800/9600/19200/38400); kept " + std::to_string(vfd.baud_rate) + ".";
            }
        }
        int addr = vfd.slave_address;
        if (V.integer("Address", addr)) {
            vfd.slave_address = static_cast<uint8_t>(std::clamp(addr, 1, 247));
        }
        vfd.enabled = plugin_on;
        add("Spindle & VFD", "Huanyang VFD Plugin", plugin_on ? "Enabled in Config -> Plugins" : "Installed, not enabled",
            plugin_on ? "RS485 VFD control ON" : "RS485 VFD control OFF", MigrationSeverity::Ok, "");
        add("Spindle & VFD", "VFD Serial Port", fmt("COM%d, %d baud, address %d", com, baud, addr),
            fmt("%s, %u baud, address %u", vfd.port_name.c_str(), vfd.baud_rate, static_cast<unsigned>(vfd.slave_address)),
            baud_sev, baud_note.empty() ? std::string(port_note) : baud_note);

        // Plugin v2 (with speed monitoring) writes <Margin>; older versions only know port and status DROs
        int margin = 0;
        const bool v2 = V.integer("Margin", margin);
        if (v2) {
            bool mon = false;
            int bad = vfd.bad_speed_threshold, track = vfd.tracking_threshold;
            V.flag("EStopOption", mon);
            V.integer("BadSpeedThreshold", bad);
            V.integer("BadtrackingThreshold", track);
            vfd.speed_monitor_enabled = mon;
            vfd.speed_margin_pct      = static_cast<uint8_t>(std::clamp(margin, 1, 50));
            vfd.bad_speed_threshold   = static_cast<uint8_t>(std::clamp(bad, 1, 255));
            vfd.tracking_threshold    = static_cast<uint8_t>(std::clamp(track, 1, 255));
            add("Spindle & VFD", "Speed Monitoring (Automatic EStop)",
                fmt("%s, margin %d%%, bad speed %d, tracking %d", mon ? "On" : "Off", margin, bad, track),
                fmt("%s, +/-%u%%, %u / %u readings", mon ? "On" : "Off", static_cast<unsigned>(vfd.speed_margin_pct),
                    static_cast<unsigned>(vfd.bad_speed_threshold), static_cast<unsigned>(vfd.tracking_threshold)),
                MigrationSeverity::Ok, "");
            bool cause = false;
            if (V.flag("EStopCauseOption", cause) && cause) {
                add("Spindle & VFD", "E-Stop Cause Message", "On", "Always shown", MigrationSeverity::Ok,
                    "BreakoutCNC always shows why the VFD stopped the machine.");
            }

            // Status slots = Huanyang status codes; <Status> is the bitmask of polled values
            int status_mask = 0;
            V.integer("Status", status_mask);
            static const char* kSlotNames[8] = {"Set frequency", "Output frequency", "Current", "Speed",
                                                "DC bus voltage", "Output voltage", "Counter", "Temperature"};
            static const char* kSlotUnits[8] = {"Hz", "Hz", "A", "RPM", "V", "V", "", "C"};
            for (int i = 0; i < 8; ++i) {
                double lim = 0.0;
                if (!((status_mask >> i) & 1) || !V.num(fmt("Status%d_lim", i), lim) || !(lim > 0.0)) continue;
                float* target = nullptr;
                switch (i) {
                    case 1: target = &vfd.limit_out_freq_hz; break;
                    case 2: target = &vfd.limit_current_a;   break;
                    case 3: target = &vfd.limit_rpm;         break;
                    case 4: target = &vfd.limit_dc_volts;    break;
                    case 5: target = &vfd.limit_ac_volts;    break;
                    case 7: target = &vfd.limit_temp_c;      break;
                    default: break;
                }
                const std::string name = std::string(kSlotNames[i]) + " Limit";
                const std::string raw = fmt("%.4g %s", lim, kSlotUnits[i]);
                if (target) {
                    *target = static_cast<float>(lim);
                    add("Spindle & VFD", name, raw, raw + ": E-Stop above", MigrationSeverity::Ok,
                        "The plugin stopped the machine when this reading went above the limit.");
                } else {
                    add("Spindle & VFD", name, raw, "Not imported", MigrationSeverity::Warning,
                        "BreakoutCNC has no limit for this reading.");
                }
            }
        } else {
            add("Spindle & VFD", "Plugin Monitoring Settings", "Older plugin version", "Defaults kept",
                MigrationSeverity::Ok, "This plugin version has no speed monitoring or limit settings to import.");
        }

        if (has_pulley) {
            vfd.max_rpm = static_cast<uint16_t>(std::clamp(pulley_max, 1000.0, 60000.0));
            if (pulley_min > 0.0) {
                vfd.min_rpm = static_cast<uint16_t>(std::clamp(pulley_min, 0.0, static_cast<double>(vfd.max_rpm)));
            }
            add("Spindle & VFD", "Spindle Speed Range", fmt("Pulley %d: %.0f .. %.0f RPM", pulley_idx + 1, pulley_min, pulley_max),
                fmt("%u .. %u RPM at %.0f Hz", static_cast<unsigned>(vfd.min_rpm), static_cast<unsigned>(vfd.max_rpm), vfd.max_freq_hz),
                MigrationSeverity::Warning,
                "The VFD frequency is max RPM -> max frequency. Mach3 does not store the VFD's max frequency (PD005): "
                "use 'Read Parameters from VFD' in the VFD settings before running the spindle.");
            warn(fmt("Spindle speed scaling assumes %u RPM at %.0f Hz. Confirm PD005/PD144 with 'Read Parameters from VFD' "
                     "before the first spindle start, otherwise the spindle may run faster than commanded.",
                     static_cast<unsigned>(vfd.max_rpm), vfd.max_freq_hz));
        }
    } else if (has_pulley) {
        add("Spindle & VFD", "Spindle Speed Range", fmt("Pulley %d: %.0f .. %.0f RPM", pulley_idx + 1, pulley_min, pulley_max),
            "Not imported", MigrationSeverity::Ok, "Only used for a Huanyang VFD; no VFD plugin in this profile.");
    }

    // ------------------------------------------------------------------------
    // 9. Tool change, Safe Z & G-code options
    // ------------------------------------------------------------------------
    bool ignore_tool = false, auto_tool = false;
    const bool has_ignore = P.flag("IGNORETOOL", ignore_tool);
    const bool has_auto   = P.flag("AutoTool", auto_tool);
    if (has_ignore || has_auto) {
        if (ignore_tool) cfg.tool_change.mode = core::ToolChangeMode::IgnoreM6;
        else if (auto_tool) cfg.tool_change.mode = core::ToolChangeMode::AutoAtc;
        else cfg.tool_change.mode = core::ToolChangeMode::ManualPrompt;
        add("Tool Change", "M6 Tool Change", ignore_tool ? "Ignore tool change" : (auto_tool ? "Auto tool changer" : "Stop spindle, wait"),
            ignore_tool ? "Ignore M6" : (auto_tool ? "Auto ATC" : "Manual prompt"),
            ignore_tool ? MigrationSeverity::Warning : MigrationSeverity::Ok,
            ignore_tool ? "Mach3 ignored M6: programs with several tools will not stop for a tool change." : "");
    }

    bool use_safe_z = false;
    double safe_z = 0.0;
    if (P.flag("UseSafeZ", use_safe_z) && use_safe_z && P.num("m_SafeZ", safe_z) && safe_z > 0.0) {
        cfg.tool_change.safe_z_mm = std::clamp(safe_z * unit_to_mm, 1.0, 200.0);
        add("Tool Change", "Safe Z", fmt("%.4g %s", safe_z, unit_name), fmt("%.1f mm", cfg.tool_change.safe_z_mm),
            MigrationSeverity::Ok, "");
    }

    bool dwell_ms = false;
    if (P.flag("DwellinMilli", dwell_ms)) {
        cfg.dwell_p_milliseconds = dwell_ms;
        add("Units & Core", "G4 Dwell P Units", dwell_ms ? "Milliseconds" : "Seconds",
            dwell_ms ? "Milliseconds" : "Seconds", MigrationSeverity::Ok, "");
    }

    // ------------------------------------------------------------------------
    // 10. Units plausibility (a wrong <Units> reading scales every G-code move by 25.4)
    // ------------------------------------------------------------------------
    if (axis_tuned[0]) {
        const auto& x = cfg.axes[0];
        const bool looks_inch_as_mm = !is_imperial && x.steps_per_mm > 2000.0 && x.max_velocity_mm_s < 5.0;
        const bool looks_mm_as_inch = is_imperial && x.steps_per_mm < 40.0 && x.max_velocity_mm_s > 300.0;
        if (looks_inch_as_mm || looks_mm_as_inch) {
            add("Units & Core", "Units Plausibility", fmt("%.1f steps/mm, %.0f mm/min", x.steps_per_mm, x.max_velocity_mm_s * 60.0),
                "Check units", MigrationSeverity::Warning,
                looks_inch_as_mm ? "These values look like an inch machine read as mm." : "These values look like a mm machine read as inches.");
            warn("The X axis values look implausible for the selected units. Compare Mach3 Config -> Select Native Units "
                 "and use the Units option if needed.");
        }
    }

    report.success = true;
    return report;
}

} // namespace rnr::config
