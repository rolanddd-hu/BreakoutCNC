#include "core/board/grbl/firmware_profile.h"
#include "core/board/board_registry.h"
#include <algorithm>
#include <array>
#include <cstdlib>

namespace rnr::board::grbl {

const char* flavourName(Flavour f) noexcept {
    switch (f) {
        case Flavour::Grbl11:  return "GRBL 1.1";
        case Flavour::GrblHal: return "grblHAL";
        case Flavour::FluidNC: return "FluidNC";
        default:               return "GRBL-compatible";
    }
}

Flavour flavourForBoardId(std::string_view id) noexcept {
    if (id == kGrblBoardId) return Flavour::Grbl11;
    if (id == kGrblHalBoardId) return Flavour::GrblHal;
    if (id == kFluidNcBoardId) return Flavour::FluidNC;
    return Flavour::GenericGrbl;
}

const char* boardIdForFlavour(Flavour f) noexcept {
    switch (f) {
        case Flavour::GrblHal: return kGrblHalBoardId;
        case Flavour::FluidNC: return kFluidNcBoardId;
        default:               return kGrblBoardId;
    }
}

std::string FirmwareProfile::describe() const {
    std::string s = flavourName(flavour);
    std::string ver = version_text;
    const size_t colon = ver.find(':');
    if (colon != std::string::npos) ver = ver.substr(0, colon);
    if (flavour == Flavour::FluidNC && ver.find("FluidNC") != std::string::npos) {
        const size_t at = ver.find("FluidNC");
        return ver.substr(at); // "FluidNC v3.7.8"
    }
    if (flavour == Flavour::Grbl11 || flavour == Flavour::GenericGrbl) return ver.empty() ? s : "GRBL " + ver;
    if (!ver.empty()) s += " " + ver;
    return s;
}

FirmwareProfile defaultProfile(Flavour f) {
    FirmwareProfile p;
    p.flavour = f;
    switch (f) {
        case Flavour::Grbl11:
            p.planner_blocks = 15;
            p.rx_buffer_bytes = 128;
            p.max_line_chars = 80;
            p.has_mist = false;
            break;
        case Flavour::GrblHal:
            p.planner_blocks = 16;
            p.rx_buffer_bytes = 128; // Until [OPT:] says more (GRBL_FAMILY_PLAN.md section 18, item 4)
            p.max_line_chars = 80;
            p.has_mist = true;
            p.reports_homed = true;
            p.home_single_axis = true;
            break;
        case Flavour::FluidNC:
            p.planner_blocks = 16;
            p.rx_buffer_bytes = 128;
            p.max_line_chars = 80;
            p.has_mist = true;
            p.home_single_axis = true;
            break;
        default:
            break;
    }
    return p;
}

Flavour detectFromWelcome(const std::string& w) noexcept {
    if (w.find("FluidNC") != std::string::npos) return Flavour::FluidNC;
    if (w.rfind("GrblHAL", 0) == 0 || w.find("grblHAL") != std::string::npos) return Flavour::GrblHal;
    if (w.rfind("Grbl 1.1", 0) == 0) return Flavour::Grbl11;
    return Flavour::GenericGrbl;
}

void applyBuildInfo(FirmwareProfile& p, const std::string& tag, const std::string& body) {
    if (tag == "VER") {
        p.version_text = body;
        if (body.find("FluidNC") != std::string::npos) {
            p.flavour = Flavour::FluidNC;
        } else if (p.flavour == Flavour::GenericGrbl && body.rfind("1.1", 0) == 0) {
            p.flavour = Flavour::Grbl11;
        }
        p.detected = true;
    } else if (tag == "OPT") {
        // letters,blocks,rx[,axes,tools...]
        const size_t c1 = body.find(',');
        p.option_codes = body.substr(0, c1);
        if (c1 != std::string::npos) {
            const char* rest = body.c_str() + c1 + 1;
            char* end = nullptr;
            const long blocks = std::strtol(rest, &end, 10);
            if (end != rest && blocks > 0) p.planner_blocks = static_cast<size_t>(blocks);
            if (end && *end == ',') {
                rest = end + 1;
                const long rx = std::strtol(rest, &end, 10);
                if (end != rest && rx >= 64) p.rx_buffer_bytes = static_cast<size_t>(rx);
                if (end && *end == ',') {
                    rest = end + 1;
                    const long axes = std::strtol(rest, &end, 10);
                    if (end != rest && axes >= 3 && axes <= 8) p.axis_count = static_cast<uint8_t>(axes);
                }
            }
        }
        const std::string& o = p.option_codes;
        auto has = [&](char c) { return o.find(c) != std::string::npos; };
        p.variable_spindle = has('V') || p.flavour == Flavour::FluidNC;
        p.reports_line_numbers = has('N');
        if (p.flavour == Flavour::Grbl11 || p.flavour == Flavour::GenericGrbl) {
            p.has_mist = has('M');
            p.home_single_axis = has('H');
        } else if (has('M')) {
            p.has_mist = true;
        }
        if (has('H')) p.home_single_axis = true;
        p.safety_door = has('+');
        p.origin_at_home = has('Z');
    } else if (tag == "NEWOPT") {
        p.new_options = body;
        auto hasOpt = [&](const char* name) {
            const std::string n(name);
            size_t pos = 0;
            while ((pos = body.find(n, pos)) != std::string::npos) {
                const bool start_ok = pos == 0 || body[pos - 1] == ',';
                const size_t e = pos + n.size();
                const bool end_ok = e == body.size() || body[e] == ',' || body[e] == '=';
                if (start_ok && end_ok) return true;
                pos = e;
            }
            return false;
        };
        p.settings_enumeration = hasOpt("ENUMS");
        p.top_bit_realtime = hasOpt("RT+");
        if (hasOpt("THC")) { // The plasma plugin adds THC while a mode is active
            p.plasma_plugin = true;
            p.board_thc = true;
        }
        if (p.flavour == Flavour::GenericGrbl || p.flavour == Flavour::Grbl11) p.flavour = Flavour::GrblHal;
    } else if (tag == "FIRMWARE") {
        p.firmware_name = body;
        if (body.find("grblHAL") != std::string::npos) p.flavour = Flavour::GrblHal;
        if (body.find("FluidNC") != std::string::npos) p.flavour = Flavour::FluidNC;
    } else if (tag == "BOARD") {
        p.board_name = body;
    } else if (tag == "PLUGIN") {
        // [PLUGIN:PLASMA (Up/down) v0.27]
        if (body.rfind("PLASMA", 0) == 0) {
            p.plasma_plugin = true;
            const size_t a = body.find('('), b = body.find(')');
            if (a != std::string::npos && b != std::string::npos && b > a) p.plasma_mode = body.substr(a + 1, b - a - 1);
            if (!p.plasma_mode.empty() && p.plasma_mode != "Off") p.board_thc = true;
        }
    } else if (tag == "AXS") {
        // [AXS:3:XYZ]
        const size_t c = body.find(':');
        const int n = std::atoi(body.c_str());
        if (n >= 3 && n <= 8) p.axis_count = static_cast<uint8_t>(n);
        if (c != std::string::npos) p.axis_letters = body.substr(c + 1);
    } else if (tag == "AUX IO") {
        // [AUX IO:di,do,ai,ao]
        std::array<double, 4> v{};
        const char* s = body.c_str();
        int n = 0;
        while (*s && n < 4) {
            char* end = nullptr;
            v[static_cast<size_t>(n++)] = std::strtod(s, &end);
            if (end == s) break;
            s = (*end == ',') ? end + 1 : end;
        }
        p.aux_inputs = static_cast<uint8_t>(std::clamp(v[0], 0.0, 32.0));
        p.aux_outputs = static_cast<uint8_t>(std::clamp(v[1], 0.0, 32.0));
    }
}

void applyNumberedSetting(FirmwareProfile& p, int n, double v) {
    if (n == 13) p.reports_inches = v != 0.0;
    else if (n == 20) p.soft_limits = v != 0.0;
    else if (n == 22) p.homing_enabled = static_cast<long>(v) != 0;
    else if (n == 23) p.homing_dir_invert_mask = static_cast<uint8_t>(v);
    else if (n == 30) p.max_spindle_rpm = v > 0.0 ? v : 1000.0;
    else if (n == 31) p.min_spindle_rpm = v;
    else if (n == 32) p.laser_mode = v != 0.0;
    else if (n >= 100 && n <= 103) p.steps_per_mm[static_cast<size_t>(n - 100)] = v;
    else if (n >= 110 && n <= 113) p.max_rate_mm_min[static_cast<size_t>(n - 110)] = v;
    else if (n >= 120 && n <= 123) p.accel_mm_s2[static_cast<size_t>(n - 120)] = v;
    else if (n >= 130 && n <= 133) p.max_travel_mm[static_cast<size_t>(n - 130)] = v;
    else if (n == 350 && p.plasma_plugin) {
        static const char* modes[] = {"Off", "Voltage", "Up/down", "Arc ok"};
        const long m = static_cast<long>(v);
        p.plasma_mode = (m >= 0 && m <= 3) ? modes[m] : "Off";
        p.board_thc = m >= 1 && m <= 3;
    }
    p.settings_read = true;
}

BoardCapabilities firmwareCapabilities(Flavour f, const FirmwareProfile* detected) {
    const FirmwareProfile p = detected ? *detected : defaultProfile(f);
    BoardCapabilities c;
    c.display_name = flavourName(f);
    c.family = Family::FirmwarePlanned;
    c.accesses_real_hardware = true;
    c.input_count = 0;
    c.output_count = 0;
    c.max_slices_in_flight = 0;
    c.hardware_probe_latch = true;
    c.hardware_home_latch = true;

    c.axis_count = static_cast<uint8_t>(std::clamp<int>(p.axis_count, 3, 4));
    c.axis_letters = std::string("XYZA").substr(0, c.axis_count);
    c.slaved_motor_option = false;

    c.app_owns_motor_tuning = false;
    c.realtime_process_control = false;
    c.native_arcs = true;
    c.exact_stop_mode = false;
    c.precise_line_tracking = p.reports_line_numbers;
    c.can_set_machine_position = false;
    c.jog_while_held = false;

    c.homing = HomingKind::BoardCycle;
    c.home_single_axis = p.home_single_axis;
    c.probe = true;
    c.limit_override = false;
    c.board_soft_limits = p.soft_limits;
    c.needs_unlock = true;

    c.feed_override_min_pct = 10.0f;
    c.feed_override_max_pct = 200.0f;
    c.rapid_override = true;
    c.spindle_override = true;

    c.spindle_pwm = p.variable_spindle;
    c.spindle_direction = true;
    c.estop_input = (f == Flavour::GrblHal);
    c.estop_keeps_position = false;
    c.mist_output = p.has_mist;
    c.aux_outputs = (f == Flavour::Grbl11) ? 0 : p.aux_outputs;

    c.settings_model = (f == Flavour::FluidNC) ? SettingsModel::YamlConfig
                     : (f == Flavour::GrblHal && p.settings_enumeration) ? SettingsModel::Enumerated
                     : SettingsModel::NumberedDollar;
    c.text_console = true;
    c.board_thc = (f == Flavour::GrblHal) && p.board_thc;
    c.board_thc_mode = p.plasma_mode;

    // Terminals: fixed functions, reported by their Pn: letter
    auto in = [&](const char* id, const char* label, IoRole role, char letter) {
        IoPoint pt;
        pt.id = id;
        pt.label = label;
        pt.kind = IoKind::DigitalIn;
        pt.fixed_role = role;
        pt.status_letter = letter;
        c.io.push_back(pt);
    };
    auto out = [&](const std::string& id, const std::string& label, IoRole role) {
        IoPoint pt;
        pt.id = id;
        pt.label = label;
        pt.kind = IoKind::DigitalOut;
        pt.fixed_role = role;
        c.io.push_back(pt);
    };
    in("limit_x", "X limit", IoRole::LimitX, 'X');
    in("limit_y", "Y limit", IoRole::LimitY, 'Y');
    in("limit_z", "Z limit", IoRole::LimitZ, 'Z');
    if (c.axis_count > 3) in("limit_a", "A limit", IoRole::LimitA, 'A');
    in("probe", "Probe", IoRole::Probe, 'P');
    if (f == Flavour::FluidNC || f == Flavour::GrblHal) in("toolsetter", "Tool setter", IoRole::ToolSetter, 'T');
    in("door", "Door", IoRole::Door, 'D');
    in("hold", "Feed hold", IoRole::FeedHold, 'H');
    in("start", "Cycle start", IoRole::CycleStart, 'S');
    in("reset", "Reset", IoRole::Reset, 'R');
    if (f == Flavour::GrblHal) in("estop", "E-stop", IoRole::EStop, 'E');
    if (c.board_thc) {
        // Plasma plugin signals come in the THC: status field (status_letter 0: not a Pn: input)
        in("arc_ok", "Arc OK", IoRole::ArcOk, 0);
    }
    out("spindle_en", "Spindle", IoRole::SpindleEnable);
    out("spindle_dir", "Spindle dir", IoRole::SpindleDirection);
    out("flood", "Flood", IoRole::Flood);
    if (c.mist_output) out("mist", "Mist", IoRole::Mist);
    for (uint8_t i = 0; i < c.aux_outputs && i < 8; ++i) {
        out("aux_out_" + std::to_string(i), "Aux " + std::to_string(i), IoRole::AuxOutput);
    }
    return c;
}

} // namespace rnr::board::grbl
