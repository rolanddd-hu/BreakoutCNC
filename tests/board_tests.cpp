// Board tests: GRBL-family protocol parsing, the G-code emitter, and the machine core driving the FakeGrbl
// emulator through the firmware-planned motion path (GRBL 1.1, grblHAL, FluidNC). No hardware is touched.
#include "core/board/board_registry.h"
#include "core/board/grbl/firmware_profile.h"
#include "core/board/grbl/grbl_codes.h"
#include "core/board/grbl/grbl_messages.h"
#include "core/board/grbl/grbl_settings.h"
#include "core/config/settings_manager.h"
#include "core/controller/machine_controller.h"
#include "core/motion/gcode_emitter.h"
#include "core/transport/byte_stream.h"
#include "core/board/grbl/fake_grbl.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <thread>

using namespace rnr;
using test::FakeGrbl;

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                                  \
    do {                                                                             \
        ++g_checks;                                                                  \
        if (!(cond)) {                                                               \
            ++g_failures;                                                            \
            std::printf("  FAILED: %s (line %d)\n", #cond, __LINE__);                \
        }                                                                            \
    } while (0)

bool near(double a, double b, double tol = 0.01) { return std::abs(a - b) <= tol; }

bool waitFor(const std::function<bool()>& pred, int timeout_ms) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < until) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pred();
}

// A machine core connected to a FakeGrbl of the given flavour
struct Rig {
    std::shared_ptr<FakeGrbl> g;
    std::unique_ptr<core::MachineController> c;

    Rig(FakeGrbl::Flavour f, const std::string& board_id, const std::function<void(FakeGrbl&)>& setup = {}) {
        g = std::make_shared<FakeGrbl>(f);
        if (setup) setup(*g);
        auto fake = g;
        transport::registerStreamFactory("fake", [fake] { return std::make_unique<test::FakeGrblStream>(fake); });
        c = std::make_unique<core::MachineController>(nullptr, false);
        auto cfg = c->getConfig();
        cfg.is_mill = true;
        cfg.require_homing = false;
        cfg.scripting.m6_script_enabled = false;
        cfg.board_id = board_id;
        auto& p = cfg.board_profiles[board_id];
        p.set("transport", "fake");
        p.set("address", "fake0");
        p.setInt("connect_wait_ms", 200);
        c->applyConfig(cfg);
        c->connect();
    }
    ~Rig() {
        c.reset();
        transport::unregisterStreamFactory("fake");
    }
    core::TelemetrySnapshot t() const { return c->getTelemetry(); }
    bool connected(int ms = 5000) {
        return waitFor([&] { return !t().firmware_version.empty(); }, ms);
    }
    bool ready() {
        if (!connected()) return false;
        c->resetFromEStop();
        return waitFor([&] {
            const auto x = t();
            return x.state == core::ControllerState::Idle && !x.board_busy && g->currentState() == "Idle";
        }, 4000);
    }
    bool idle(int ms = 8000) {
        return waitFor([&] {
            const auto x = t();
            return x.state == core::ControllerState::Idle && !x.board_busy;
        }, ms);
    }
};

void testMessages() {
    std::printf("[Test 1] GRBL message parsing...\n");
    using namespace board::grbl;
    auto m = parseLine("<Run|MPos:1.000,-2.500,3.250|Bf:12,96|FS:800,12000|Ov:110,100,90|A:SF|Pn:XP|Ln:214>");
    CHECK(m.kind == MessageKind::Status);
    CHECK(m.status.state == "Run");
    CHECK(m.status.has_mpos && near(m.status.mpos[1], -2.5));
    CHECK(m.status.has_feed && near(m.status.feed, 800) && near(m.status.spindle, 12000));
    CHECK(m.status.has_ov && m.status.ov_feed == 110 && m.status.ov_spindle == 90);
    CHECK(m.status.pins == "XP" && m.status.accessories == "SF");
    CHECK(m.status.has_line && m.status.line == 214);
    m = parseLine("<Hold:0|WPos:0,0,0|WCO:10,20,30>");
    CHECK(m.status.state == "Hold" && m.status.substate == 0 && m.status.has_wco && near(m.status.wco[2], 30));
    m = parseLine("[PRB:1.000,2.000,-3.500:1]");
    CHECK(m.kind == MessageKind::Probe && m.flag && near(m.position[2], -3.5));
    m = parseLine("$110=3000.000");
    CHECK(m.kind == MessageKind::Setting && m.name == "110" && m.text == "3000.000");
    m = parseLine("error:33");
    CHECK(m.kind == MessageKind::Nack && m.code == 33);
    m = parseLine("ALARM:1");
    CHECK(m.kind == MessageKind::Alarm && m.code == 1);
    m = parseLine("Grbl 1.1h ['$' for help]");
    CHECK(m.kind == MessageKind::Welcome);
    m = parseLine("[G54:5.000,6.000,7.000]");
    CHECK(m.kind == MessageKind::Parameter && m.name == "G54" && near(m.position[1], 6.0));
    m = parseLine("[SETTING:100|10|X-axis travel resolution|step/mm|6|###0.000|||0|0]");
    CHECK(m.kind == MessageKind::Enumeration && m.name == "SETTING");
    EnumeratedSetting es;
    CHECK(parseEnumeratedSetting(m.text, es) && es.id == "100" && es.name == "X-axis travel resolution");

    FirmwareProfile p = defaultProfile(Flavour::Grbl11);
    applyBuildInfo(p, "OPT", "VNMHZ,15,128");
    CHECK(p.reports_line_numbers && p.has_mist && p.home_single_axis && p.origin_at_home && p.rx_buffer_bytes == 128);
    FirmwareProfile h = defaultProfile(Flavour::GenericGrbl);
    applyBuildInfo(h, "OPT", "VNMHSL,35,1024,4,0");
    applyBuildInfo(h, "NEWOPT", "ENUMS,RT+,HOME");
    CHECK(h.flavour == Flavour::GrblHal && h.rx_buffer_bytes == 1024 && h.planner_blocks == 35 && h.axis_count == 4);
    CHECK(h.settings_enumeration && h.top_bit_realtime);
    // grblHAL plasma plugin: the THC status field, the plugin report and the plasma mode setting
    m = parseLine("<Run|MPos:1.000,2.000,-3.000|FS:600,0|THC:118.5,ATER>");
    CHECK(m.kind == MessageKind::Status && m.status.has_thc && near(m.status.thc_voltage, 118.5));
    CHECK(m.status.thc_flags == "ATER");
    m = parseLine("<Idle|MPos:0.000,0.000,0.000|FS:0,0|THC:0.0>");
    CHECK(m.status.has_thc && m.status.thc_flags.empty());
    applyBuildInfo(h, "PLUGIN", "PLASMA (Up/down) v0.27");
    CHECK(h.plasma_plugin && h.board_thc && h.plasma_mode == "Up/down");
    CHECK(firmwareCapabilities(Flavour::GrblHal, &h).board_thc);
    CHECK(!firmwareCapabilities(Flavour::Grbl11, &h).board_thc);
    applyNumberedSetting(h, 350, 0);
    CHECK(!h.board_thc && h.plasma_mode == "Off");
    applyNumberedSetting(h, 350, 1);
    CHECK(h.board_thc && h.plasma_mode == "Voltage");
    // The boards' own alarm / error texts
    int code = 0;
    std::string text;
    CHECK(parseEnumeratedCode("10||EStop asserted. Clear and reset", code, text) && code == 10 &&
          text == "EStop asserted. Clear and reset");
    CHECK(parseEnumeratedCode("14|Spindle|Spindle at speed timeout", code, text) && text == "Spindle at speed timeout");
    CHECK(!parseEnumeratedCode("x||y", code, text));
    CHECK(parseCodeListLine("[MSG:10: Spindle Control]", code, text) && code == 10 && text == "Spindle Control");
    CHECK(parseCodeListLine("[MSG:INFO: 61: SD card failed to open file]", code, text) && code == 61);
    CHECK(parseCodeListLine("1: Hard Limit", code, text) && code == 1 && text == "Hard Limit");
    CHECK(!parseCodeListLine("[MSG:Active alarm: none]", code, text));
    CHECK(detectFromWelcome("Grbl 3.7 [FluidNC v3.7.8 (wifi) '$' for help]") == Flavour::FluidNC);

    auto yaml = parseFluidNcConfig({"name: test", "axes:", "  x:", "    steps_per_mm: 80.000", "    homing:",
                                    "      cycle: 2", "  y:", "    steps_per_mm: 81"});
    CHECK(yaml.size() == 4);
    CHECK(yaml.size() >= 3 && yaml[1].id == "/axes/x/steps_per_mm" && yaml[1].value == "80.000");
    CHECK(yaml.size() >= 3 && yaml[2].id == "/axes/x/homing/cycle");
    std::printf("  -> done\n");
}

void testEmitter() {
    std::printf("[Test 2] G-code emitter...\n");
    using core::motion::GcodeEmitter;
    CHECK(GcodeEmitter::num(-0.0004, 3) == "0");
    CHECK(GcodeEmitter::num(1.5000, 3) == "1.5");
    CHECK(GcodeEmitter::num(-12.34567, 3) == "-12.346");
    CHECK(GcodeEmitter::num(100.0, 3) == "100");
    GcodeEmitter e;
    std::vector<std::string> out;
    e.rapid({10, 20, 5, 0}, out);
    e.rapid({10, 30, 5, 0}, out);
    e.linear({10, 30, -1, 0}, 200, out);
    e.linear({20, 30, -1, 0}, 200, out);
    CHECK(out.size() == 4);
    CHECK(out.size() == 4 && out[0] == "G0X10Y20Z5");
    CHECK(out.size() == 4 && out[1] == "Y30");
    CHECK(out.size() == 4 && out[2] == "G1Z-1F200");
    CHECK(out.size() == 4 && out[3] == "X20");
    out.clear();
    e.arc(true, 18, {30, 30, -1, 0}, {5, 0, 0}, 300, out);
    CHECK(out.size() == 3 && out[0] == "G18" && out[2] == "G17");
    CHECK(out.size() == 3 && out[1].rfind("G2X30Y30Z-1I5K0F300", 0) == 0);
    out.clear();
    e.outputs(true, false, 12000, false, true, false, out);
    CHECK(out.size() == 3 && out[0] == "M3S12000" && out[1] == "M9" && out[2] == "M8");
    out.clear();
    e.outputs(true, false, 12000, false, true, false, out);
    CHECK(out.empty());
    e.outputs(false, false, -1, false, false, false, out);
    CHECK(out.size() == 2 && out[0] == "M5" && out[1] == "M9");
    CHECK(GcodeEmitter::jogLine({0.5, 0, -0.25, 0}, 600, 3) == "$J=G91G21X0.5Z-0.25F600");
    {
        // M7 / M8 / M9 are one modal group in GRBL: never two on a line
        GcodeEmitter m;
        core::motion::GcodeDialect d;
        d.mist = true;
        m.setDialect(d);
        std::vector<std::string> o;
        m.outputs(false, false, -1, false, true, true, o);
        CHECK(o.size() == 4 && o[0] == "M5" && o[1] == "M9" && o[2] == "M8" && o[3] == "M7");
        o.clear();
        m.outputs(false, false, -1, false, false, true, o);
        CHECK(o.size() == 2 && o[0] == "M9" && o[1] == "M7");
        // A line too long with its line number is sent without it
        GcodeEmitter n;
        core::motion::GcodeDialect dn;
        dn.line_numbers = true;
        dn.max_line_chars = 24;
        n.setDialect(dn);
        o.clear();
        n.linear({1234.567, -1234.567, -12.5, 0}, 1500, o);
        CHECK(o.size() == 1 && o[0].rfind("G1X1234.567", 0) == 0);
        o.clear();
        n.rapid({0, 0, 5, 0}, o);
        CHECK(o.size() == 1 && o[0][0] == 'N');
    }
    std::printf("  -> done\n");
}

void testConnect(FakeGrbl::Flavour f, const char* id, const char* name) {
    std::printf("[Test 3] Connect and identify %s...\n", name);
    Rig r(f, id);
    CHECK(r.connected());
    const auto t = r.t();
    CHECK(t.board_firmware_planned);
    CHECK(t.state == core::ControllerState::EStop);
    const auto caps = r.c->getBoardCapabilities();
    CHECK(caps.family == board::Family::FirmwarePlanned);
    CHECK(caps.homing == board::HomingKind::BoardCycle);
    CHECK(!caps.app_owns_motor_tuning);
    CHECK(caps.feed_override_max_pct == 200.0f);
    CHECK(r.ready());
    if (f == FakeGrbl::Flavour::GrblHal) {
        const auto st = r.c->getFirmwareSettings();
        bool enum_name = false;
        for (const auto& it : st.items) {
            if (it.id == "100" && it.name == "X-axis travel resolution") enum_name = true;
        }
        CHECK(enum_name);
        CHECK(caps.settings_model == board::SettingsModel::Enumerated);
        CHECK(caps.aux_outputs == 4);
    }
    std::printf("  -> %s: %s\n", name, t.firmware_version.c_str());
}

void testManualControl() {
    std::printf("[Test 4] MDI moves, work offsets, step jog, continuous jog...\n");
    Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId);
    CHECK(r.ready());
    if (!r.c->executeMdiCommand("G0 X10 Y5 Z-1")) {
        ++g_failures;
        std::printf("  MDI rejected: %s | state %d\n", r.t().active_fault_message.c_str(), static_cast<int>(r.t().state));
        for (const auto& e : r.c->getBoardLog(0)) std::printf("    %d %s\n", static_cast<int>(e.dir), e.text.c_str());
    }
    CHECK(r.idle());
    auto p = r.g->position();
    CHECK(near(p[0], 10) && near(p[1], 5) && near(p[2], -1));
    CHECK(near(r.t().machine_pos_mm[0], 10));
    // Work offsets stay in the app: the board gets machine coordinates (G53)
    r.c->setWorkspaceOffset(0, {5.0, 5.0, 0.0, 0.0});
    CHECK(r.c->executeMdiCommand("G0 X0 Y0"));
    CHECK(r.idle());
    p = r.g->position();
    CHECK(near(p[0], 5) && near(p[1], 5));
    CHECK(r.g->sawLineContaining("G53"));
    CHECK(!r.g->sawLineContaining("G10"));
    // A move right after an output command waits for it instead of being refused (m6.lua does exactly this)
    // Coolant by hand (Flood output): one coolant word per line, which the board accepts
    r.c->setAuxOutput(3, true);
    CHECK(waitFor([&] { return r.g->flood; }, 3000));
    r.c->setAuxOutput(3, false);
    CHECK(waitFor([&] { return !r.g->flood; }, 3000));
    CHECK(r.t().active_fault_message.find("rejected") == std::string::npos);
    CHECK(r.idle());
    CHECK(r.c->executeMdiCommand("M8"));
    CHECK(r.c->executeMdiCommand("G0 X6"));
    CHECK(r.idle());
    CHECK(near(r.g->position()[0], 11) && r.g->flood);
    CHECK(r.c->executeMdiCommand("M9"));
    CHECK(r.idle());
    // Step jog
    r.c->jogStep(core::Axis::X, 2.0, 600.0);
    CHECK(r.idle());
    CHECK(near(r.g->position()[0], 13));
    // Continuous jog (dead-man refreshed every 100 ms)
    const double x0 = r.g->position()[0];
    for (int i = 0; i < 6; ++i) {
        r.c->jogStartContinuous(core::Axis::X, true, 0.2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    r.c->jogStopAxis(core::Axis::X);
    CHECK(r.idle(4000));
    const double x1 = r.g->position()[0];
    CHECK(x1 > x0 + 1.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    CHECK(near(r.g->position()[0], x1, 0.001)); // Stopped and stays
    CHECK(!r.g->overflowed);
    std::printf("  -> done (jogged %.2f mm)\n", x1 - x0);
}

void testHomingAndProbing() {
    std::printf("[Test 5] Homing, probe hit and miss...\n");
    Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId);
    CHECK(r.ready());
    CHECK(r.c->executeMdiCommand("G0 X12 Y7 Z-2"));
    CHECK(r.idle());
    r.c->homeAllAxes();
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::Homing; }, 1000));
    CHECK(r.idle(8000));
    auto t = r.t();
    CHECK(t.axis_homed[0] && t.axis_homed[1] && t.axis_homed[2]);
    CHECK(near(r.g->position()[0], 0) && near(t.machine_pos_mm[0], 0));

    r.g->probe_contact_z = -3.0;
    r.c->probeRaw(core::Axis::Z, -10.0, 300.0);
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::Probing; }, 1000));
    CHECK(r.idle());
    t = r.t();
    CHECK(t.last_probe_hit);
    CHECK(near(t.last_probed_work_pos_mm[2], -3.0, 0.02));
    CHECK(r.g->sawLineContaining("G38.3"));

    CHECK(r.c->executeMdiCommand("G0 Z0"));
    CHECK(r.idle());
    r.g->probe_contact_z = NAN;
    r.c->probeRaw(core::Axis::Z, -2.0, 300.0);
    CHECK(r.idle());
    CHECK(!r.t().last_probe_hit);
    std::printf("  -> done\n");
}

const char* kJob =
    "G21 G90\n"
    "G0 X0 Y0 Z5\n"
    "M3 S1000\n"
    "M8\n"
    "G1 Z-1 F600\n"
    "G1 X10 F900\n"
    "G2 X20 Y0 I5 J0\n"
    "G1 Y5\n"
    "M6 T2\n"
    "G0 Z5\n"
    "G0 X0 Y0\n"
    "M30\n";

void testJob() {
    std::printf("[Test 6] Job with an arc, spindle, coolant and a tool change...\n");
    Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId);
    CHECK(r.ready());
    CHECK(r.c->loadGCodeString(kJob));
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::RunningGCode; }, 2000));
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::ToolChangeHold; }, 15000));
    CHECK(r.g->sawLineContaining("M3S1000"));
    CHECK(r.g->sawLineContaining("M8"));
    CHECK(r.g->sawLineContaining("G2X20Y0"));
    CHECK(r.g->sawLineContaining("G4P0"));
    CHECK(!r.g->sawLineContaining("M6"));
    CHECK(r.t().pending_tool_number == 2);
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::ToolChangeHold && !r.t().board_busy; }, 8000));
    CHECK(near(r.g->position()[2], 25.0)); // Retracted to the tool change height
    r.c->startJob(); // Tool installed: continue
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::Idle && r.g->currentState() == "Idle"; }, 15000));
    auto p = r.g->position();
    CHECK(near(p[0], 0) && near(p[1], 0) && near(p[2], 5));
    CHECK(r.g->sawLineContaining("M5M9"));
    CHECK(r.t().current_tool_number == 2);
    CHECK(!r.g->overflowed);
    CHECK(r.t().active_segment_index == r.c->getProgram()->segments.size());
    std::printf("  -> done (max receive buffer use %zu bytes)\n", r.g->max_rx_used);
}

void testLineTracking() {
    std::printf("[Test 6b] Highlighted line follows short moves that status reports never see the end of...\n");
    Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId, [](FakeGrbl& g) { g.time_scale = 1.0; });
    CHECK(r.ready());
    // Two 0.3 mm moves (30 ms each at 600 mm/min) and a long one: reports come every 100 ms
    CHECK(r.c->loadGCodeString("G1 Z-0.3 F600\nG1 Z-0.6\nG1 X30\n"));
    r.c->startJob();
    CHECK(waitFor([&] { return r.g->position()[0] > 5.0; }, 4000));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    const auto t = r.t();
    CHECK(t.active_gcode_line_index == 2);
    CHECK(t.segment_progress_0_to_1 > 0.1f && t.segment_progress_0_to_1 < 0.95f);
    CHECK(r.idle(8000));
    std::printf("  -> done (line %zu, progress %.2f while on the long move)\n", t.active_gcode_line_index + 1,
                t.segment_progress_0_to_1);
}

// BreakoutCut plasma with THC on: allowed only when the board runs THC itself (grblHAL plasma plugin)
const char* kPlasmaJob =
    "G0 X5 Y5\n"
    "M3 S1\n"
    "G4 P0.2\n"
    "G1 X45 F600\n"
    "G1 Y15\n"
    "M5\n"
    "G0 X0 Y0\n";

void usePlasmaThc(Rig& r) {
    auto cfg = r.c->getConfig();
    cfg.is_mill = false;
    cfg.operation_mode = core::OperationMode::Plasma;
    cfg.thc.enabled = true;
    cfg.thc.wait_for_arc_ok = true;
    r.c->applyConfig(cfg);
}

// Runs the loaded job to its end, sampling the highlighted segment: it must never go back
bool segmentNeverGoesBack(Rig& r, int ms, size_t* samples_out = nullptr) {
    size_t last = 0, samples = 0;
    bool ok = true, started = false;
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < until) {
        const auto t = r.t();
        const bool running = t.state == core::ControllerState::RunningGCode || t.state == core::ControllerState::DwellPierce;
        if (running) {
            started = true;
            ++samples;
            if (t.active_segment_index < last) {
                std::printf("    segment went back from %zu to %zu (line %zu)\n", last, t.active_segment_index,
                            t.active_gcode_line_index + 1);
                ok = false;
            }
            last = std::max(last, t.active_segment_index);
        } else if (started && t.state == core::ControllerState::Idle && !t.board_busy) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (samples_out) *samples_out = samples;
    return ok && started;
}

void testArcs() {
    std::printf("[Test 6d] Arcs: the board traces G2 / G3, the highlighted segment follows the arc...\n");
    Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId, [](FakeGrbl& g) { g.time_scale = 1.0; });
    CHECK(r.ready());
    // A full circle of radius 10 around (20, 10), then a quarter arc to (40, 30)
    CHECK(r.c->loadGCodeString("G0 X30 Y10\nG2 X30 Y10 I-10 J0 F1500\nG1 X40\nG3 X50 Y20 I0 J10\nG1 Y30\n"));
    const auto prog = r.c->getProgram();
    CHECK(prog && prog->arcs.size() == 2);
    r.c->startJob();
    // Half way round the circle the tool is on the circle's far side, not on its start point
    CHECK(waitFor([&] { return r.g->position()[0] > 29.9; }, 6000));
    CHECK(waitFor([&] { return r.g->position()[0] < 12.0; }, 6000));
    auto p = r.g->position();
    CHECK(std::abs(std::hypot(p[0] - 20.0, p[1] - 10.0) - 10.0) < 0.05);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    auto t = r.t();
    CHECK(t.active_gcode_line_index == 1);
    const size_t first = prog->arcs[0].first_segment, count = prog->arcs[0].segment_count;
    CHECK(count > 4);
    CHECK(t.active_segment_index > first + count / 4 && t.active_segment_index < first + count - 1);
    const size_t seg_mid_circle = t.active_segment_index;
    // On the quarter arc
    CHECK(waitFor([&] { const auto q = r.g->position(); return q[0] > 44.0 && q[1] > 11.0; }, 8000));
    p = r.g->position();
    CHECK(std::abs(std::hypot(p[0] - 40.0, p[1] - 20.0) - 10.0) < 0.05);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    t = r.t();
    CHECK(t.active_gcode_line_index == 3);
    const bool g2_sent = r.g->sawLineContaining("G2") && r.g->sawLineContaining("I-10J0");
    CHECK(g2_sent);
    if (!g2_sent) {
        for (const auto& l : r.g->receivedLines()) std::printf("    sent: %s\n", l.c_str());
    }
    CHECK(r.idle(8000));
    CHECK(r.t().active_segment_index == prog->segments.size());
    // Once more, sampling all the way: the highlighted segment only goes forward
    r.c->stopJob(); // Rewind
    CHECK(r.idle(4000));
    r.c->startJob();
    CHECK(segmentNeverGoesBack(r, 15000));
    std::printf("  -> done (segment %zu of the circle's %zu..%zu half way round)\n", seg_mid_circle, first,
                first + count - 1);
}

void testBoardThc() {
    std::printf("[Test 6c] Plasma with torch height control on the board (grblHAL plasma plugin)...\n");
    {
        // Without the plugin a plasma job with THC is refused
        Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId);
        CHECK(r.ready());
        usePlasmaThc(r);
        CHECK(!r.c->getBoardCapabilities().board_thc);
        CHECK(r.c->loadGCodeString(kPlasmaJob));
        r.c->startJob();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        CHECK(r.t().state == core::ControllerState::Idle);
        CHECK(r.t().active_fault_message.find("torch height control") != std::string::npos);
    }
    Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId, [](FakeGrbl& g) {
        g.time_scale = 4.0;
        g.plasma_plugin = true;
        g.thc_z_drift_mm = 0.5; // Height control moves Z during the cut
    });
    CHECK(r.ready());
    usePlasmaThc(r);
    const auto caps = r.c->getBoardCapabilities();
    CHECK(caps.board_thc && caps.board_thc_mode == "Up/down");
    CHECK(r.t().thc_on_board);
    CHECK(r.c->loadGCodeString(kPlasmaJob));
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::RunningGCode; }, 2000));
    CHECK(waitFor([&] { return r.g->position()[0] > 20.0; }, 8000));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    auto t = r.t();
    CHECK(t.thc_arc_ok_active && t.thc_torch_on && t.thc_board_active);
    CHECK(near(t.thc_arc_voltage, 118.5, 0.05));
    CHECK(t.active_gcode_line_index == 3); // Followed by XY although Z is off the planned height
    CHECK(r.idle(15000));
    CHECK(!r.t().thc_arc_ok_active && !r.t().thc_torch_on);
    CHECK(r.g->torch_starts == 1);
    CHECK(r.g->sawLineContaining("M3S1000")); // The torch is switched on with the board's full S ($30)
    // A full circle ends where it starts, then the torch retracts straight up: the highlight must not jump back to
    // the start of the circle while XY stays at that point
    CHECK(r.c->loadGCodeString("G0 X30 Y10\nM3 S1\nG2 X30 Y10 I-10 J0 F600\nM5\nG0 Z10\nG0 X0 Y0\n"));
    r.c->startJob();
    size_t samples = 0;
    CHECK(segmentNeverGoesBack(r, 20000, &samples));
    CHECK(samples > 20);
    CHECK(r.idle(5000));
    CHECK(r.g->torch_starts == 2);
    CHECK(r.c->loadGCodeString(kPlasmaJob));
    // A torch that does not start: the board holds the job, the app says why, Cycle Start retries
    r.g->arc_failures = 1;
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::PausedHold; }, 6000));
    CHECK(r.t().active_fault_message.find("Arc OK") != std::string::npos);
    CHECK(r.g->position()[0] < 5.01);
    r.c->startJob();
    CHECK(waitFor([&] { return r.g->position()[0] > 10.0; }, 6000));
    CHECK(r.idle(15000));
    CHECK(!r.g->overflowed);
    std::printf("  -> done (arc voltage %.1f V, line %zu during the cut)\n", t.thc_arc_voltage, t.active_gcode_line_index + 1);
}

bool hasIssue(const core::ProgramCheckResult& r, core::ProgramIssueKind kind, const char* text) {
    for (const auto& i : r.issues) {
        if (i.kind == kind && i.text.find(text) != std::string::npos) return true;
    }
    return false;
}
void printIssues(const core::ProgramCheckResult& r) {
    for (const auto& i : r.issues) std::printf("    issue (%d) line %zu: %s\n", static_cast<int>(i.kind), i.line + 1, i.text.c_str());
}

void testFourthAxis() {
    std::printf("[Test 12] 4th axis (A) on grblHAL and FluidNC, A moves on a 3-axis board...\n");
    {
        Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId, [](FakeGrbl& g) { g.useAxes(4); });
        CHECK(r.ready());
        const auto caps = r.c->getBoardCapabilities();
        CHECK(caps.axis_count == 4 && caps.axis_letters == "XYZA");
        CHECK(r.c->executeMdiCommand("G0 X10 A90"));
        CHECK(r.idle());
        auto p = r.g->position();
        CHECK(near(p[0], 10) && near(p[3], 90));
        CHECK(near(r.t().machine_pos_mm[3], 90));
        r.c->jogStep(core::Axis::A, -15.0, 600.0);
        CHECK(waitFor([&] { return near(r.g->position()[3], 75.0); }, 4000));
        CHECK(r.idle());
        // A job with A moves: sent and made
        CHECK(r.c->loadGCodeString("G0 X0 A0\nG1 X20 A180 F1200\nG0 A0\n"));
        r.c->startJob();
        CHECK(waitFor([&] { return r.t().state == core::ControllerState::RunningGCode; }, 2000));
        CHECK(r.idle(12000));
        CHECK(r.g->sawLineContaining("A180"));
        CHECK(near(r.g->position()[3], 0.0));
        // Homing all axes homes A as well
        r.c->homeAllAxes();
        CHECK(waitFor([&] { const auto t = r.t(); return t.axis_homed[0] && t.axis_homed[3]; }, 6000));
        CHECK(r.idle());
        std::this_thread::sleep_for(std::chrono::milliseconds(400)); // Later status reports (H:) keep A homed
        CHECK(r.t().axis_homed[3]);
    }
    {
        // FluidNC does not list its axes in $I: the status report shows four
        Rig r(FakeGrbl::Flavour::FluidNC, board::kFluidNcBoardId, [](FakeGrbl& g) { g.useAxes(4); });
        CHECK(r.ready());
        CHECK(waitFor([&] { return r.c->getBoardCapabilities().axis_count == 4; }, 2000));
    }
    {
        // A 3-axis board: a program that moves A cannot start
        Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId);
        CHECK(r.ready());
        CHECK(r.c->getBoardCapabilities().axis_count == 3);
        CHECK(r.c->loadGCodeString("G0 X5\nG1 A30 F600\nG1 X10\n"));
        const auto check = r.c->getProgramCheck();
        CHECK(check && hasIssue(*check, core::ProgramIssueKind::StartBlocker, "A axis"));
        r.c->startJob();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        CHECK(r.t().state == core::ControllerState::Idle);
        CHECK(r.t().active_fault_message.find("A axis") != std::string::npos);
    }
    std::printf("  -> done\n");
}

void testBoardTexts() {
    std::printf("[Test 13] Alarm and error texts from the board (grblHAL $EA / $EE, FluidNC $A / $E)...\n");
    {
        Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId);
        CHECK(r.ready());
        CHECK(r.g->sawLineContaining("$EA") && r.g->sawLineContaining("$EE"));
        r.g->raiseAlarm(10);
        CHECK(waitFor([&] { return r.t().state == core::ControllerState::EStop; }, 2000));
        CHECK(r.t().active_fault_message.find("EStop asserted") != std::string::npos);
        CHECK(r.c->resetFromEStop());
        CHECK(r.idle(3000));
        // Alarms 1-9 keep BreakoutCNC's wording
        r.g->raiseAlarm(1);
        CHECK(waitFor([&] { return r.t().state == core::ControllerState::EStop; }, 2000));
        CHECK(r.t().active_fault_message.find("Hard limit triggered") != std::string::npos);
        CHECK(r.c->resetFromEStop());
        CHECK(r.idle(3000));
        // An error beyond GRBL 1.1's codes: the board's text
        r.g->reject_substring = "X77";
        r.g->reject_code = 79;
        r.c->executeMdiCommand("G0 X77");
        CHECK(waitFor([&] { return r.t().active_fault_message.find("Emergency stop active") != std::string::npos; }, 3000));
    }
    {
        Rig r(FakeGrbl::Flavour::FluidNC, board::kFluidNcBoardId);
        CHECK(r.ready());
        r.g->raiseAlarm(10);
        CHECK(waitFor([&] { return r.t().state == core::ControllerState::EStop; }, 2000));
        CHECK(r.t().active_fault_message.find("Spindle Control") != std::string::npos);
    }
    {
        // GRBL 1.1 has no code lists: nothing extra is asked
        Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId);
        CHECK(r.ready());
        CHECK(!r.g->sawLineContaining("$EA") && !r.g->sawLineContaining("$A"));
    }
    std::printf("  -> done\n");
}

void testEstopWhileConnecting() {
    std::printf("[Test 13b] E-stop while connecting, FluidNC alarm consequences...\n");
    {
        Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId);
        // The handshake is under way: the app's E-stop still resets the board, which is then identified again
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        r.c->triggerEStop();
        CHECK(waitFor([&] { return r.g->resets >= 1; }, 2000));
        CHECK(r.ready());
        CHECK(r.c->getBoardCapabilities().axis_count == 3);
    }
    {
        Rig r(FakeGrbl::Flavour::FluidNC, board::kFluidNcBoardId);
        CHECK(r.ready());
        r.c->homeAllAxes();
        CHECK(waitFor([&] { const auto t = r.t(); return t.axis_homed[0] && t.axis_homed[2]; }, 6000));
        CHECK(r.idle());
        r.g->raiseAlarm(10); // FluidNC: spindle control, the position is kept
        CHECK(waitFor([&] { return r.t().state == core::ControllerState::EStop; }, 2000));
        CHECK(r.t().axis_homed[0] && r.t().axis_homed[2]);
    }
    std::printf("  -> done\n");
}

void testBoardChecks() {
    std::printf("[Test 14] Program check with the board's travel, maximum rates and aux outputs...\n");
    Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId);
    CHECK(r.ready());
    r.c->homeAllAxes();
    CHECK(waitFor([&] { const auto t = r.t(); return t.axis_homed[0] && t.axis_homed[1] && t.axis_homed[2]; }, 6000));
    CHECK(r.idle());
    // Travel $130 = 300 homing to +: X from -300 to 0. G54 at machine 0, so X50 is beyond it.
    CHECK(r.c->loadGCodeString("G0 X-20 Y-20\nG1 X50 F600\nG1 X-310 F9000\nG0 X-20\n"));
    auto check = r.c->getProgramCheck();
    CHECK(check);
    const bool beyond = check && hasIssue(*check, core::ProgramIssueKind::Warning, "beyond the travel -300.0..0.0");
    const bool below = check && hasIssue(*check, core::ProgramIssueKind::Warning, "below the travel -300.0..0.0");
    const bool rates = check && hasIssue(*check, core::ProgramIssueKind::Warning, "board's maximum rates");
    CHECK(beyond);
    CHECK(below);
    CHECK(rates);
    if (check && !(beyond && below && rates)) printIssues(*check);
    // Inside the travel and below the rates: no board warnings
    CHECK(r.c->loadGCodeString("G0 X-20 Y-20\nG1 X-100 F600\n"));
    check = r.c->getProgramCheck();
    CHECK(check && !hasIssue(*check, core::ProgramIssueKind::Warning, "travel") &&
          !hasIssue(*check, core::ProgramIssueKind::Warning, "maximum rates"));
    std::printf("  -> done\n");
}

void testBoardReport() {
    std::printf("[Test 15] Board report...\n");
    Rig r(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId, [](FakeGrbl& g) { g.useAxes(4); });
    CHECK(r.ready());
    std::this_thread::sleep_for(std::chrono::milliseconds(400)); // A few status reports
    CHECK(r.c->requestBoardReport());
    CHECK(waitFor([&] { const auto x = r.c->getBoardReport(); return !x.running && x.revision > 0; }, 5000));
    const auto rep = r.c->getBoardReport();
    auto has = [&](const char* s) { return rep.text.find(s) != std::string::npos; };
    CHECK(has("> $I\n") && has("[VER:1.1f") && has("> $I+") && has("[DRIVER:emulator]"));
    CHECK(has("> $$") && has("$110=") && has("$133=360.000") && has("> $#") && has("[G54:") && has("> $G"));
    CHECK(has("Axes: 4 (XYZA)") && has("board code texts: 3 alarms, 2 errors"));
    CHECK(has("---- Recent status reports ----") && has("MPos:"));
    CHECK(r.idle());
    // An unsupported command in the report does not stop it
    Rig g11(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId);
    CHECK(g11.ready());
    CHECK(g11.c->requestBoardReport());
    CHECK(waitFor([&] { const auto x = g11.c->getBoardReport(); return !x.running && x.revision > 0; }, 5000));
    const auto rep11 = g11.c->getBoardReport();
    CHECK(rep11.text.find("> $N") != std::string::npos && rep11.text.find("> $G") != std::string::npos);
    if (rep.text.size() < 200) std::printf("%s\n", rep.text.c_str());
    std::printf("  -> done (%zu characters)\n", rep.text.size());
}

void testHoldStopEstop() {
    std::printf("[Test 7] Feed hold, resume, controlled stop, E-stop...\n");
    Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId, [](FakeGrbl& g) { g.time_scale = 4.0; });
    CHECK(r.ready());
    CHECK(r.c->loadGCodeString("G1 X60 F600\nG1 X0\n"));
    r.c->startJob();
    CHECK(waitFor([&] { return r.g->position()[0] > 2.0; }, 4000));
    r.c->pauseJob();
    CHECK(waitFor([&] { return r.g->currentState() == "Hold:0"; }, 2000));
    CHECK(r.t().state == core::ControllerState::PausedHold);
    const double held_x = r.g->position()[0];
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    CHECK(near(r.g->position()[0], held_x, 0.001));
    r.c->startJob(); // Cycle start = resume
    CHECK(waitFor([&] { return r.g->position()[0] > held_x + 1.0; }, 4000));
    CHECK(r.t().state == core::ControllerState::RunningGCode);
    r.c->stopJob();
    CHECK(r.idle(6000));
    const double stop_x = r.g->position()[0];
    CHECK(r.g->currentState() == "Idle"); // Reset from Hold:0 keeps the position (no alarm)
    CHECK(r.g->resets >= 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    CHECK(near(r.g->position()[0], stop_x, 0.001));
    CHECK(near(r.t().machine_pos_mm[0], stop_x, 0.01));

    // E-stop while moving: the board resets with ALARM:3, the app goes to E-stop; Reset unlocks
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::RunningGCode; }, 2000));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    r.c->triggerEStop();
    CHECK(waitFor([&] { return r.g->currentState() == "Alarm"; }, 2000));
    CHECK(r.t().state == core::ControllerState::EStop);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    CHECK(r.c->resetFromEStop());
    CHECK(waitFor([&] { return r.g->currentState() == "Idle"; }, 2000));
    CHECK(r.t().state == core::ControllerState::Idle);
    std::printf("  -> done\n");
}

void testErrorsAndRestart() {
    std::printf("[Test 8] Rejected line, alarm, restart, silent board...\n");
    {
        Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId, [](FakeGrbl& g) { g.reject_substring = "Y7"; });
        CHECK(r.ready());
        CHECK(r.c->loadGCodeString("G1 X2 F600\nG1 Y7\nG1 X0\n"));
        r.c->startJob();
        CHECK(waitFor([&] { return r.t().active_fault_message.find("rejected") != std::string::npos; }, 6000));
        CHECK(r.idle(6000));
    }
    {
        Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId);
        CHECK(r.ready());
        r.g->raiseAlarm(1);
        CHECK(waitFor([&] { return r.t().state == core::ControllerState::EStop; }, 2000));
        CHECK(r.t().active_fault_message.find("Hard limit") != std::string::npos);
        CHECK(r.c->resetFromEStop());
        CHECK(r.idle(3000));
        r.g->restart();
        CHECK(waitFor([&] { return r.t().active_fault_message.find("restarted") != std::string::npos; }, 3000));
        CHECK(r.t().state == core::ControllerState::EStop);
    }
    {
        Rig r(FakeGrbl::Flavour::FluidNC, board::kFluidNcBoardId);
        CHECK(r.ready());
        r.g->silent = true;
        CHECK(waitFor([&] { return r.t().state == core::ControllerState::Disconnected; }, 6000));
        CHECK(r.t().active_fault_message.find("stopped responding") != std::string::npos);
    }
    std::printf("  -> done\n");
}

void testSettingsAndConsole() {
    std::printf("[Test 9] Firmware settings, console...\n");
    {
        Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId);
        CHECK(r.ready());
        auto st = r.c->getFirmwareSettings();
        bool has100 = false;
        for (const auto& it : st.items) {
            if (it.id == "100" && it.name == "X steps per mm") has100 = true;
        }
        CHECK(has100);
        CHECK(r.c->writeFirmwareSettings({{"100", "80.000"}}));
        CHECK(waitFor([&] { return r.g->setting(100) == "80.000"; }, 3000));
        CHECK(waitFor([&] {
            for (const auto& it : r.c->getFirmwareSettings().items) {
                if (it.id == "100" && it.value == "80.000") return true;
            }
            return false;
        }, 3000));
        CHECK(waitFor([&] { return !r.t().board_busy; }, 3000));
        CHECK(r.c->writeFirmwareSettings({{"10", "9"}}));
        CHECK(waitFor([&] {
            for (const auto& it : r.c->getFirmwareSettings().items) {
                if (it.id == "10" && !it.error.empty()) return true;
            }
            return false;
        }, 3000));
        CHECK(waitFor([&] { return !r.t().board_busy; }, 3000));
        CHECK(r.c->sendBoardConsoleLine("$G"));
        CHECK(waitFor([&] {
            for (const auto& e : r.c->getBoardLog(0)) {
                if (e.text.find("[GC:") != std::string::npos) return true;
            }
            return false;
        }, 3000));
    }
    {
        Rig r(FakeGrbl::Flavour::FluidNC, board::kFluidNcBoardId);
        CHECK(r.ready());
        r.c->requestFirmwareSettings();
        CHECK(waitFor([&] {
            const auto st = r.c->getFirmwareSettings();
            if (!st.yaml) return false;
            for (const auto& it : st.items) {
                if (it.id == "/axes/x/steps_per_mm") return true;
            }
            return false;
        }, 3000));
    }
    std::printf("  -> done\n");
}

void testFlowControl() {
    std::printf("[Test 10] Flow control with a long program of short moves...\n");
    Rig r(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId, [](FakeGrbl& g) { g.time_scale = 60.0; });
    CHECK(r.ready());
    std::string prog = "G1 F3000\n";
    for (int i = 0; i < 300; ++i) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f\n", (i % 20) * 0.5, (i / 20) * 0.5);
        prog += buf;
    }
    CHECK(r.c->loadGCodeString(prog));
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().state == core::ControllerState::RunningGCode; }, 2000));
    CHECK(r.idle(30000));
    CHECK(!r.g->overflowed);
    CHECK(r.g->max_rx_used <= 127);
    std::printf("  -> done (max receive buffer use %zu of 128 bytes)\n", r.g->max_rx_used);
}

void testSettingsFile() {
    std::printf("[Test 11] Board selection and board sections in the settings file...\n");
    config::PersistentSettings s;
    s.machine.board_id = board::kGrblHalBoardId;
    s.machine.board_profiles[board::kGrblHalBoardId].set("address", "COM7");
    s.machine.board_profiles[board::kGrblHalBoardId].setInt("baud", 115200);
    s.machine.board_profiles["future_board"].set("key", "value"); // A board this build does not know
    const std::string file = (std::filesystem::temp_directory_path() / "breakout_board_settings_test.ini").string();
    CHECK(config::SettingsManager::saveToFile(file, s));
    config::PersistentSettings l;
    CHECK(config::SettingsManager::loadFromFile(file, l));
    CHECK(l.machine.board_id == board::kGrblHalBoardId);
    CHECK(l.machine.board_profiles[board::kGrblHalBoardId].get("address") == "COM7");
    CHECK(l.machine.board_profiles["future_board"].get("key") == "value");
    // A file from 0.4 (no board key) selects the RNR board
    config::PersistentSettings old;
    CHECK(old.machine.board_id == board::kRnrBoardId);
    std::error_code ec;
    std::filesystem::remove(file, ec);
    std::filesystem::remove(file + ".bak", ec);
    std::printf("  -> done\n");
}

} // namespace

int main() {
    core::MachineController::setHardwareAccessEnabled(false);
    std::printf("====================================================\n");
    std::printf("BOARD TESTS (GRBL 1.1, grblHAL, FluidNC on FakeGrbl)\n");
    std::printf("====================================================\n");
    testMessages();
    testEmitter();
    testConnect(FakeGrbl::Flavour::Grbl11, board::kGrblBoardId, "GRBL 1.1");
    testConnect(FakeGrbl::Flavour::GrblHal, board::kGrblHalBoardId, "grblHAL");
    testConnect(FakeGrbl::Flavour::FluidNC, board::kFluidNcBoardId, "FluidNC");
    testManualControl();
    testHomingAndProbing();
    testJob();
    testLineTracking();
    testArcs();
    testBoardThc();
    testHoldStopEstop();
    testErrorsAndRestart();
    testSettingsAndConsole();
    testFlowControl();
    testSettingsFile();
    testFourthAxis();
    testBoardTexts();
    testEstopWhileConnecting();
    testBoardChecks();
    testBoardReport();
    std::printf("====================================================\n");
    if (g_failures == 0) {
        std::printf("ALL BOARD TESTS PASSED (%d checks)\n", g_checks);
        return 0;
    }
    std::printf("%d OF %d CHECKS FAILED\n", g_failures, g_checks);
    return 1;
}
