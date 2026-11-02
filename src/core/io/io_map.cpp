#include "core/io/io_map.h"
#include <cctype>

namespace rnr::core::io {

namespace {

bool validPin(int pin) noexcept { return pin >= 1 && pin <= kPhysicalOutputCount; }

bool equalsNoCase(const std::string& a, const std::string& b) noexcept {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

} // namespace

uint8_t physicalOutputs(const IoConfig& io, uint8_t logical_mask) noexcept {
    uint8_t phys = 0;
    uint8_t used = 0;
    for (int ch = 0; ch < kOutputChannelCount; ++ch) {
        const int pin = io.output_pin[ch];
        if (!validPin(pin)) continue;
        used |= static_cast<uint8_t>(1u << (pin - 1));
        if ((logical_mask >> ch) & 1u) phys |= static_cast<uint8_t>(1u << (pin - 1));
    }
    const uint8_t custom = static_cast<uint8_t>((logical_mask >> kCustomOutputShift) & 0x0Fu);
    return static_cast<uint8_t>(phys | (custom & ~used));
}

int channelForPin(const IoConfig& io, int pin_1_to_4) noexcept {
    if (!validPin(pin_1_to_4)) return -1;
    for (int ch = 0; ch < kOutputChannelCount; ++ch) {
        if (io.output_pin[ch] == pin_1_to_4) return ch;
    }
    return -1;
}

uint8_t logicalBitForPin(const IoConfig& io, int pin_1_to_4) noexcept {
    if (!validPin(pin_1_to_4)) return 0;
    const int ch = channelForPin(io, pin_1_to_4);
    return ch >= 0 ? static_cast<uint8_t>(1u << ch) : static_cast<uint8_t>(1u << (kCustomOutputShift + pin_1_to_4 - 1));
}

const char* channelName(int channel, bool is_mill, OperationMode mode) noexcept {
    if (is_mill) {
        static const char* kMill[] = {"Spindle on (M3 / M4)", "Spindle direction (M4)", "Flood coolant (M8)",
                                      "Mist coolant (M7)"};
        return (channel >= 0 && channel < kOutputChannelCount) ? kMill[channel] : "";
    }
    if (mode == OperationMode::Laser) {
        static const char* kLaser[] = {"Laser enable (M3 / M4)", "Red-dot pointer", "Air assist (M8)",
                                       "Exhaust fan (M7)"};
        return (channel >= 0 && channel < kOutputChannelCount) ? kLaser[channel] : "";
    }
    static const char* kPlasma[] = {"Torch on (M3)", "Not used in plasma mode", "Flood (M8)", "Mist (M7)"};
    return (channel >= 0 && channel < kOutputChannelCount) ? kPlasma[channel] : "";
}

const char* channelShortName(int channel, bool is_mill, OperationMode mode) noexcept {
    if (is_mill) {
        static const char* kMill[] = {"Spindle", "Spindle dir", "Flood", "Mist"};
        return (channel >= 0 && channel < kOutputChannelCount) ? kMill[channel] : "";
    }
    if (mode == OperationMode::Laser) {
        static const char* kLaser[] = {"Laser", "Red dot", "Air assist", "Exhaust"};
        return (channel >= 0 && channel < kOutputChannelCount) ? kLaser[channel] : "";
    }
    static const char* kPlasma[] = {"Torch", "Unused", "Flood", "Mist"};
    return (channel >= 0 && channel < kOutputChannelCount) ? kPlasma[channel] : "";
}

bool sanitize(IoConfig& io) noexcept {
    bool changed = false;
    uint8_t used = 0;
    for (int ch = 0; ch < kOutputChannelCount; ++ch) {
        uint8_t& pin = io.output_pin[ch];
        if (pin != 0 && (!validPin(pin) || ((used >> (pin - 1)) & 1u))) {
            pin = 0;
            changed = true;
        }
        if (pin != 0) used |= static_cast<uint8_t>(1u << (pin - 1));
    }
    if (io.output_pin[0] == 0) {
        for (int pin = 1; pin <= kPhysicalOutputCount; ++pin) {
            if (!((used >> (pin - 1)) & 1u)) {
                io.output_pin[0] = static_cast<uint8_t>(pin);
                changed = true;
                break;
            }
        }
    }
    return changed;
}

std::array<std::vector<InputUse>, kInputCount> describeInputs(const MachineConfig& cfg) {
    std::array<std::vector<InputUse>, kInputCount> uses;
    auto add = [&](int pin, std::string label, const char* section, bool invert) {
        if (pin >= 1 && pin <= kInputCount) uses[pin - 1].push_back({std::move(label), section, invert});
    };

    const char* kLimits = "Limits & Homing";
    static const char* kAxis[] = {"X", "Y", "Z", "A"};
    for (int a = 0; a < 4; ++a) {
        const AxisConfig& ax = cfg.axes[a];
        add(ax.home_pin, std::string(kAxis[a]) + " home", kLimits, ax.home_invert);
        add(ax.limit_minus_pin, std::string(kAxis[a]) + " limit-", kLimits, ax.limit_minus_invert);
        add(ax.limit_plus_pin, std::string(kAxis[a]) + " limit+", kLimits, ax.limit_plus_invert);
    }
    add(cfg.estop_pin, "E-stop", kLimits, cfg.estop_invert);

    if (cfg.is_mill) {
        add(cfg.probe.probe_pin, "Probe", "Probe Settings", cfg.probe.probe_invert);
    } else if (cfg.operation_mode == OperationMode::Laser) {
        const char* kLaser = "Laser Interlocks";
        add(cfg.laser.chiller_flow_pin, "Chiller flow", kLaser, cfg.laser.chiller_flow_invert);
        add(cfg.laser.door_interlock_pin, "Door / lid", kLaser, cfg.laser.door_interlock_invert);
        add(cfg.laser.laser_fault_pin, "Laser PSU alarm", kLaser, cfg.laser.laser_fault_invert);
    } else {
        add(cfg.probe.probe_pin, "Ohmic / float", "Initial Height Sensing", cfg.probe.probe_invert);
        if (cfg.thc.enabled || cfg.thc.wait_for_arc_ok) {
            add(cfg.thc.arc_ok_pin, "Arc OK", "Torch Height Control", cfg.thc.arc_ok_invert);
        }
        if (cfg.thc.enabled) {
            add(cfg.thc.thc_up_pin, "THC up", "Torch Height Control", cfg.thc.thc_up_invert);
            add(cfg.thc.thc_down_pin, "THC down", "Torch Height Control", cfg.thc.thc_down_invert);
        }
    }
    return uses;
}

std::string inputShortLabel(const std::vector<InputUse>& uses, const std::string& custom_name) {
    if (uses.empty()) return custom_name;
    if (uses.size() == 1) return uses[0].label;
    // All uses on one axis ("X home" + "X limit-"): "X home/lim"
    const std::string axis = uses[0].label.substr(0, 2);
    bool same_axis = axis.size() == 2 && axis[1] == ' ';
    for (const auto& u : uses) same_axis = same_axis && u.label.compare(0, 2, axis) == 0;
    if (same_axis) {
        bool home = false, limit = false;
        for (const auto& u : uses) {
            if (u.label.find("home") != std::string::npos) home = true;
            if (u.label.find("limit") != std::string::npos) limit = true;
        }
        if (home && limit) return axis + "home/lim";
        if (limit) return axis + "limits";
    }
    return std::to_string(uses.size()) + " functions";
}

int inputPinByName(const IoConfig& io, const std::string& name) noexcept {
    if (name.empty()) return 0;
    for (int i = 0; i < kInputCount; ++i) {
        if (equalsNoCase(io.input_name[i], name)) return i + 1;
    }
    return 0;
}

} // namespace rnr::core::io
