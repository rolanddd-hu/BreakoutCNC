#include "core/board/board_registry.h"
#include "core/board/grbl/firmware_profile.h"

namespace rnr::board {

BoardCapabilities rnrEcoMotionCapabilities() {
    BoardCapabilities caps;
    caps.display_name         = "RNR Eco Motion 2.0";
    caps.family               = Family::HostPlanned;
    caps.input_count          = 12;
    caps.output_count         = 4;
    caps.min_pulse_khz        = 10;
    caps.max_pulse_khz        = 100;
    caps.max_slices_in_flight = 3; // 75 ms ahead of the acknowledgement
    caps.hardware_probe_latch = true;
    caps.hardware_home_latch  = true;
    caps.spindle_pwm          = true;
    caps.accesses_real_hardware = true;
    caps.axis_count           = 4;
    caps.axis_letters         = "XYZA";
    caps.settings_model       = SettingsModel::AppFields;
    for (int i = 1; i <= 12; ++i) {
        IoPoint p;
        p.id = "IN" + std::to_string(i);
        p.label = p.id;
        p.kind = IoKind::DigitalIn;
        p.invertible_in_app = true;
        caps.io.push_back(p);
    }
    for (int i = 1; i <= 4; ++i) {
        IoPoint p;
        p.id = "OUT" + std::to_string(i);
        p.label = p.id;
        p.kind = IoKind::DigitalOut;
        caps.io.push_back(p);
    }
    return caps;
}

namespace {

BoardProfile grblDefaultProfile(grbl::Flavour f) {
    BoardProfile p;
    p.set("transport", "serial");
    p.set("address", "");
    p.setInt("baud", 115200);
    p.setInt("tcp_port", 23);
    p.setBool("dtr_on_connect", f != grbl::Flavour::FluidNC);
    p.setBool("rts_on_connect", false);
    p.setInt("connect_wait_ms", f == grbl::Flavour::Grbl11 ? 2500 : 800);
    p.setInt("status_interval_ms", 200);
    return p;
}

BoardDescriptor grblDescriptor(const char* id, const char* name, const char* description, grbl::Flavour f) {
    BoardDescriptor d;
    d.id = id;
    d.display_name = name;
    d.description = description;
    d.family = Family::FirmwarePlanned;
    d.transports = {"serial", "tcp"};
    d.capabilities = [f] { return grbl::firmwareCapabilities(f); };
    d.default_profile = [f] { return grblDefaultProfile(f); };
    return d;
}

} // namespace

BoardRegistry::BoardRegistry() {
    BoardDescriptor rnr;
    rnr.id = kRnrBoardId;
    rnr.display_name = "RNR Eco Motion 2.0";
    rnr.description = "USB motion card, motion planned by BreakoutCNC";
    rnr.family = Family::HostPlanned;
    rnr.transports = {"hid"};
    rnr.capabilities = [] { return rnrEcoMotionCapabilities(); };
    rnr.default_profile = [] { return BoardProfile{}; };
    boards_.push_back(rnr);

    boards_.push_back(grblDescriptor(kGrblBoardId, "GRBL 1.1",
                                     "Arduino / 3018-style boards, USB serial", grbl::Flavour::Grbl11));
    boards_.push_back(grblDescriptor(kGrblHalBoardId, "grblHAL",
                                     "32-bit grblHAL boards, USB serial or network", grbl::Flavour::GrblHal));
    boards_.push_back(grblDescriptor(kFluidNcBoardId, "FluidNC",
                                     "ESP32 FluidNC boards, USB serial or network", grbl::Flavour::FluidNC));
}

const BoardRegistry& BoardRegistry::instance() {
    static const BoardRegistry registry;
    return registry;
}

const BoardDescriptor* BoardRegistry::find(std::string_view id) const {
    for (const auto& b : boards_) {
        if (b.id == id) return &b;
    }
    return nullptr;
}

BoardCapabilities BoardRegistry::capabilitiesFor(std::string_view id) const {
    const auto* d = find(id);
    return (d && d->capabilities) ? d->capabilities() : rnrEcoMotionCapabilities();
}

bool completeProfile(std::string_view board_id, BoardProfile& profile) {
    const auto* d = BoardRegistry::instance().find(board_id);
    if (!d || !d->default_profile) return false;
    bool added = false;
    for (const auto& [k, v] : d->default_profile().values) {
        if (!profile.has(k)) {
            profile.set(k, v);
            added = true;
        }
    }
    return added;
}

} // namespace rnr::board
