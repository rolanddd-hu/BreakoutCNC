// Regression tests for the findings of docs/CODE_REVIEW_FINDINGS.md (one block per finding).
// The checks use CHECK(): unlike assert() they are never compiled out, and a failure names the line.
#include "core/controller/motion_controller.h"
#include "core/controller/rnr_controller.h"
#include "core/gcode/gcode_parser.h"
#include "core/controller/program_check.h"
#include "core/planner/trajectory_planner.h"
#include "core/scripting/script_runner.h"
#include "ui_common/lua_guide_examples.h"
#include "core/board/board_registry.h"
#include "core/transport/byte_stream.h"
#include "core/board/board_driver.h"
#include "core/util/utf8_path.h"
#include "core/config/settings_manager.h"
#include "core/scripting/script_path_helper.h"
#include "core/gcode/gcode_parser.h"
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

int g_checks = 0;
int g_failures = 0;

#define CHECK(cond)                                                                              \
    do {                                                                                         \
        ++g_checks;                                                                              \
        if (!(cond)) {                                                                           \
            ++g_failures;                                                                        \
            std::fprintf(stderr, "  FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
        }                                                                                        \
    } while (0)

bool waitUntil(const std::function<bool()>& pred, int timeout_ms = 5000) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!pred()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
}

using rnr::core::ControllerState;
using rnr::gcode::GCodeParser;

// A laser controller in the simulator with the interlock inputs off, ready to run
void makeLaser(rnr::core::RnRController& c, uint16_t s_max = 1000) {
    auto cfg = c.getConfig();
    cfg.is_mill = false;
    cfg.operation_mode = rnr::core::OperationMode::Laser;
    cfg.laser.chiller_flow_pin = 0;
    cfg.laser.door_interlock_pin = 0;
    cfg.laser.laser_fault_pin = 0;
    cfg.laser.dynamic_power_m4 = false;
    cfg.laser.s_max = s_max;
    c.applyConfig(cfg);
    c.resetFromEStop();
}

// Runs the loaded program and records the laser power seen on every program line
std::vector<std::vector<float>> runAndSamplePower(rnr::core::RnRController& c, size_t lines) {
    std::vector<std::vector<float>> per_line(lines);
    c.startJob();
    waitUntil([&] { return c.getTelemetry().state == ControllerState::RunningGCode; }, 2000);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto t = c.getTelemetry();
        if (t.state != ControllerState::RunningGCode && t.state != ControllerState::DwellPierce) break;
        if (t.active_gcode_line_index < lines) per_line[t.active_gcode_line_index].push_back(t.live_laser_power_pct);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    return per_line;
}

void testA1LaserS0() {
    std::cout << "[A1] Laser S0 is beam off, no S is full power, S max is configurable\n";
    // Parser: S0 is a programmed value
    {
        const auto r = GCodeParser::parse("G21 G90\nG1 X1 F100\nM4 S0\nG1 X10\nS250\nG1 X20\n");
        CHECK(!r.hasErrors());
        CHECK(r.segments.size() == 3);
        CHECK(!r.segments[0].spindle_s_set);
        CHECK(r.segments[1].spindle_s_set && r.segments[1].spindle_rpm == 0);
        CHECK(r.segments[2].spindle_s_set && r.segments[2].spindle_rpm == 250);
    }
    // Power helpers
    {
        rnr::core::LaserConfig lz;
        CHECK(rnr::core::laserPowerPctForS(lz, false, 0) == 100.0);
        CHECK(rnr::core::laserPowerPctForS(lz, true, 0) == 0.0);
        CHECK(rnr::core::laserPowerPctForS(lz, true, 500) == 50.0);
        CHECK(rnr::core::laserPowerPctForS(lz, true, 2000) == 100.0);
        lz.s_max = 255;
        CHECK(std::abs(rnr::core::laserPowerPctForS(lz, true, 255) - 100.0) < 1e-9);
        lz.min_duty_pct = 10.0f;
        CHECK(rnr::core::laserDutyPctForPower(lz, 0.0) == 0.0);   // Off stays off despite the striking threshold
        CHECK(rnr::core::laserDutyPctForPower(lz, 100.0) == 100.0);
        CHECK(std::abs(rnr::core::laserDutyPctForPower(lz, 50.0) - 55.0) < 1e-9);
    }
    // Controller (simulator, RNR slice path): no power on the S0 line
    {
        rnr::core::RnRController c;
        makeLaser(c);
        CHECK(c.loadGCodeString("G21 G90\nM3 S500\nG1 X10 F1200\nG1 X20 S0\nG1 X30 S500\nM5\n"));
        const auto p = runAndSamplePower(c, 6);
        float max_s0 = 0.0f, max_s500 = 0.0f;
        for (float v : p[3]) max_s0 = std::max(max_s0, v);
        for (float v : p[2]) max_s500 = std::max(max_s500, v);
        CHECK(!p[3].empty());
        CHECK(max_s0 == 0.0f);
        CHECK(max_s500 > 49.0f && max_s500 < 51.0f);
    }
    // S for 100 % power = 255
    {
        rnr::core::RnRController c;
        makeLaser(c, 255);
        CHECK(c.loadGCodeString("G21 G90\nM3 S255\nG1 X10 F1200\nM5\n"));
        const auto p = runAndSamplePower(c, 4);
        float mx = 0.0f;
        for (float v : p[2]) mx = std::max(mx, v);
        CHECK(mx > 99.0f);
    }
    // Program check warns about S above S max in laser mode
    {
        rnr::core::MachineConfig cfg;
        cfg.is_mill = false;
        cfg.operation_mode = rnr::core::OperationMode::Laser;
        cfg.laser.s_max = 255;
        rnr::core::MachineCheckInput in;
        const auto r = std::make_shared<const rnr::gcode::ParseResult>(GCodeParser::parse("G21\nM3 S1000\nG1 X10 F500\nM5\n"));
        const auto check = rnr::core::buildProgramCheck(r, &cfg, in);
        bool warned = false;
        for (const auto& w : check.issues) warned = warned || w.text.find("S1000 is above") != std::string::npos;
        CHECK(warned);
    }
}


// A3: HOME ALL with the home script enabled runs home_all.lua, which homes with cnc.home()
void testA3HomeScript() {
    std::cout << "[A3] Home script homes every axis through cnc.home()\n";
    rnr::core::RnRController c;
    auto cfg = c.getConfig();
    cfg.is_mill = true;
    cfg.scripting.home_script_enabled = true;
    cfg.scripting.home_script_path = "scripts/mill/home_all.lua";
    c.applyConfig(cfg);
    CHECK(c.resetFromEStop());
    std::vector<std::string> errors;
    std::mutex m;
    c.setScriptLogCallback([&](const std::string& msg, bool is_err) {
        if (is_err) { std::lock_guard<std::mutex> lock(m); errors.push_back(msg); }
    });
    c.homeAllAxes();
    CHECK(waitUntil([&] { return c.isScriptRunning(); }, 2000));
    CHECK(waitUntil([&] { return !c.isScriptRunning(); }, 30000));
    const auto t = c.getTelemetry();
    CHECK(t.axis_homed[0] && t.axis_homed[1] && t.axis_homed[2]);
    CHECK(errors.empty());
    for (const auto& e : errors) std::cout << "  script error: " << e << "\n";
    // A single axis always uses the built-in cycle (no script)
    c.homeAxis(rnr::core::Axis::X);
    CHECK(!c.isScriptRunning());
    CHECK(c.getTelemetry().state == ControllerState::Homing);
    CHECK(waitUntil([&] { return c.getTelemetry().state == ControllerState::Idle; }, 20000));
}

// Every cnc.<name> used by a shipped script or a Lua guide example exists
void testScriptsUseOnlyTheApi() {
    std::cout << "[A3] Shipped scripts and guide examples call only registered cnc.* functions\n";
    std::set<std::string> used;
    const std::regex re(R"(cnc\.([a-z_]+))");
    auto scan = [&](const std::string& text) {
        for (std::sregex_iterator it(text.begin(), text.end(), re), end; it != end; ++it) used.insert((*it)[1]);
    };
    size_t files = 0;
    std::error_code ec;
    for (const auto& e : std::filesystem::recursive_directory_iterator("scripts", ec)) {
        if (e.path().extension() != ".lua") continue;
        std::ifstream f(e.path());
        std::stringstream ss;
        ss << f.rdbuf();
        scan(ss.str());
        ++files;
    }
    for (const auto& ex : rnr::ui::guide_examples::kAll) scan(ex.code);
    CHECK(files > 10);
    rnr::core::RnRController c;
    rnr::scripting::ScriptRunner runner(&c);
    for (const auto& name : used) {
        std::string err;
        const bool ok = runner.runStringBlocking("assert(cnc." + name + " ~= nil)", &err);
        CHECK(ok);
        if (!ok) std::cout << "  missing: cnc." << name << "\n";
    }
}

// A4: an unknown axis is an error, not X
void testA4Axis() {
    std::cout << "[A4] Lua axis arguments are checked\n";
    rnr::core::RnRController c;
    CHECK(c.resetFromEStop());
    rnr::scripting::ScriptRunner runner(&c);
    std::string err;
    CHECK(!runner.runStringBlocking("cnc.zero_axis('B')", &err));
    CHECK(err.find("axis must be") != std::string::npos);
    CHECK(!runner.runStringBlocking("cnc.set_work_pos(nil, 0)", &err));
    CHECK(!runner.runStringBlocking("cnc.zero_axis(4)", &err));
    CHECK(!runner.runStringBlocking("cnc.set_spindle_rpm(-1)", &err));
    CHECK(runner.runStringBlocking("cnc.set_work_pos('y', 12.5)", &err));
    CHECK(runner.runStringBlocking("cnc.set_work_pos(2.0, 3)", &err));
    const auto t = c.getTelemetry();
    CHECK(std::abs(t.work_pos_mm[1] - 12.5) < 1e-9);
    CHECK(std::abs(t.work_pos_mm[2] - 3.0) < 1e-9);
}

// A12 / A13: cancellation cannot be caught, and every run starts from a fresh Lua state
void testA12A13Sandbox() {
    std::cout << "[A12/A13] Script cancel cannot be caught; no globals leak between runs\n";
    rnr::core::RnRController c;
    rnr::scripting::ScriptRunner runner(&c);
    std::string err;
    CHECK(runner.runStringBlocking("leaked_global = 5", &err));
    CHECK(runner.runStringBlocking("assert(leaked_global == nil)", &err));
    CHECK(runner.runStringBlocking("cnc.gcode = nil", &err));
    CHECK(runner.runStringBlocking("assert(type(cnc.gcode) == 'function')", &err));
    CHECK(runner.runStringBlocking("assert(debug.sethook == nil and type(debug.traceback) == 'function')", &err));
    // load / loadfile: text chunks only; an environment table is passed on
    CHECK(runner.runStringBlocking("assert(load('return 7')() == 7)\n"
                                   "local e = {}\n"
                                   "load('x = 1', 'c', 't', e)()\n"
                                   "assert(e.x == 1 and x == nil)\n"
                                   "assert(not load(string.dump(function() end)))", &err));
    CHECK(runner.runStringBlocking("assert(select('#', pcall(function() return 1, 2 end)) == 3)", &err));
    for (const char* code : {"while true do pcall(function() cnc.sleep_ms(1) end) end",
                             "while true do pcall(function() local x = 0 for i = 1, 1000 do x = x + i end end) end",
                             "while true do coroutine.resume(coroutine.create(function() cnc.sleep_ms(1) end)) end",
                             "while true do xpcall(function() cnc.sleep_ms(1) end, function(e) return e end) end"}) {
        CHECK(runner.runStringAsync(code));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        CHECK(runner.isRunning());
        runner.cancel();
        const bool stopped = waitUntil([&] { return !runner.isRunning(); }, 2000);
        CHECK(stopped);
        if (!stopped) std::cout << "  not cancellable: " << code << "\n";
        waitUntil([&] { return runner.runStringBlocking("x = 1"); }, 2000);
    }
}

// A6: many small feed override steps add up
void testA6FeedOverrideSteps() {
    std::cout << "[A6] Small feed override steps reach the planner\n";
    rnr::core::RnRController c;
    CHECK(c.resetFromEStop());
    CHECK(c.loadGCodeString("G21 G90\nG1 X900 F3000\n"));
    c.startJob();
    CHECK(waitUntil([&] { return c.getTelemetry().current_feed_mm_min > 2950.0; }, 5000));
    for (int i = 1; i <= 20; ++i) {
        c.setFeedOverride(100.0f + 0.3f * static_cast<float>(i)); // 0.3 % steps up to 106 %
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    const double feed = c.getTelemetry().current_feed_mm_min;
    CHECK(feed > 3150.0 && feed < 3210.0);
    if (!(feed > 3150.0)) std::cout << "  feed " << feed << " mm/min\n";
    c.stopJob();
}

// A9: a probe sequence ending during a tool change hold returns to the hold, not to Idle
void testA9RestState() {
    std::cout << "[A9] Probing cancelled during a tool change hold returns to the hold\n";
    rnr::core::RnRController c;
    auto cfg = c.getConfig();
    cfg.is_mill = true;
    cfg.scripting.m6_script_enabled = false;
    cfg.tool_change.move_to_park_xy = false;
    c.applyConfig(cfg);
    CHECK(c.resetFromEStop());
    CHECK(c.loadGCodeString("G21 G90\nG0 X5\nT2 M6\nG0 X10\n"));
    c.startJob();
    CHECK(waitUntil([&] { const auto t = c.getTelemetry(); return t.state == ControllerState::ToolChangeHold; }, 8000));
    CHECK(c.getTelemetry().pending_tool_number == 2);
    c.startSurfaceProbing(0.0, 10.0, 0.0, 10.0, 2, 2);
    CHECK(waitUntil([&] { return c.getTelemetry().surface_probing_active; }, 2000));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    c.cancelSurfaceProbing();
    CHECK(waitUntil([&] { return !c.getTelemetry().surface_probing_active; }, 5000));
    const auto t = c.getTelemetry();
    CHECK(t.state == ControllerState::ToolChangeHold);
    CHECK(t.pending_tool_number == 2);
}

// A10: a refused MDI line changes nothing (no work offset switch)
void testA10MdiRollback() {
    std::cout << "[A10] A refused MDI line does not switch the work offset\n";
    rnr::core::RnRController c;
    auto cfg = c.getConfig();
    cfg.axes[0].soft_limits_enabled = true;
    cfg.axes[0].soft_limit_min_mm = -1.0;
    cfg.axes[0].soft_limit_max_mm = 100.0;
    c.applyConfig(cfg);
    CHECK(c.resetFromEStop());
    c.homeAllAxes();
    CHECK(waitUntil([&] { const auto t = c.getTelemetry(); return t.axis_homed[0] && t.state == ControllerState::Idle; }, 20000));
    CHECK(c.getTelemetry().active_workspace == 0);
    CHECK(!c.executeMdiCommand("G55 G1 X500 F100")); // Outside the soft limits
    CHECK(c.getTelemetry().active_workspace == 0);
    CHECK(!c.executeMdiCommand("G56 G1 X10 Q"));      // Bad word
    CHECK(c.getTelemetry().active_workspace == 0);
    CHECK(c.executeMdiCommand("G55"));                // Accepted: switches
    CHECK(c.getTelemetry().active_workspace == 1);
}

// A15: opening a slow connection never blocks the machine lock (UI, motion tick)
class SlowStream : public rnr::transport::IByteStream {
public:
    bool open(const rnr::transport::ConnectionTarget&, const rnr::transport::StreamOptions&, std::string* err) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        if (err) *err = "Cannot connect to 10.0.0.99:23";
        return false;
    }
    void close() override {}
    bool isOpen() const override { return false; }
    long read(uint8_t*, size_t, uint32_t) override { return -1; }
    bool write(const uint8_t*, size_t) override { return false; }
    bool accessesRealHardware() const override { return false; }
};

void testA15AsyncConnect() {
    std::cout << "[A15] Connecting to a slow board does not block the controller\n";
    rnr::transport::registerStreamFactory("slow", [] { return std::make_unique<SlowStream>(); });
    {
        rnr::core::MachineController c(nullptr, false);
        auto cfg = c.getConfig();
        cfg.board_id = rnr::board::kGrblHalBoardId;
        auto& prof = cfg.board_profiles[cfg.board_id];
        prof.set("transport", "slow");
        prof.set("address", "10.0.0.99:23");
        c.applyConfig(cfg);
        const auto t0 = std::chrono::steady_clock::now();
        c.connect();
        const auto connect_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
        CHECK(connect_ms < 500);
        const auto t1 = std::chrono::steady_clock::now();
        (void)c.getTelemetry();
        CHECK(std::chrono::steady_clock::now() - t1 < std::chrono::milliseconds(200));
        CHECK(c.getTelemetry().board_connecting);
        CHECK(waitUntil([&] { return c.getTelemetry().state == ControllerState::Disconnected; }, 5000));
        const auto msg = c.getTelemetry().active_fault_message;
        CHECK(msg.find("Cannot connect") != std::string::npos);
        if (msg.find("Cannot connect") == std::string::npos) std::cout << "  message: " << msg << "\n";
    }
    rnr::transport::unregisterStreamFactory("slow");
}

// A17: a file without moves keeps the loaded program
void testA17EmptyProgram() {
    std::cout << "[A17] A program without moves does not replace the loaded one\n";
    rnr::core::RnRController c;
    CHECK(c.loadGCodeString("G21 G90\nG1 X10 F500\n"));
    const auto before = c.getProgram();
    CHECK(!c.loadGCodeString("(only a comment)\n%\n"));
    CHECK(c.getProgram() == before);
    CHECK(c.getTelemetry().active_fault_message.find("no moves") != std::string::npos);
    CHECK(c.loadGCodeString("G21\nG1 X10\n")); // Errors (no feed): loaded so the errors can be shown
    CHECK(c.getProgram() != before && c.getProgram()->hasErrors());
}

// A5: file names are UTF-8 everywhere, including names outside the Windows code page
void testA5Utf8Paths() {
    std::cout << "[A5] UTF-8 file names: scripts, programs, legacy names\n";
    using rnr::util::isValidUtf8;
    CHECK(isValidUtf8("plain.nc"));
    CHECK(isValidUtf8("\xC3\xA9\xC5\x91")); // e acute, o double acute
    CHECK(!isValidUtf8("\xE9.nc"));          // Latin-1 e acute
    CHECK(!isValidUtf8("\xC3"));             // Truncated
    CHECK(rnr::util::utf8FromLegacy("plain.nc") == "plain.nc");
#ifdef _WIN32
    CHECK(isValidUtf8(rnr::util::utf8FromLegacy("\xE9.nc")));
#endif
    // A folder named with e acute, o double acute and a Cyrillic letter (not in any single ANSI code page)
    const std::string dir_utf8 = rnr::util::pathToUtf8(std::filesystem::temp_directory_path()) +
                                 "/BreakoutCNC_t\xC3\xA9st_\xC5\x91_\xD0\x96";
    const auto dir = rnr::util::pathFromUtf8(dir_utf8);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    CHECK(!ec);
    {
        std::ofstream lib(dir / "lib.lua");
        lib << "return { value = 42 }\n";
        std::ofstream main(dir / "main.lua");
        main << "local here = ...\nlocal lib = dofile(cnc.resolve_path(DIR .. '/lib.lua'))\nassert(lib.value == 42)\n";
    }
    const std::string script = dir_utf8 + "/main.lua";
    CHECK(rnr::scripting::resolveScriptPath(script) == rnr::util::pathToUtf8(std::filesystem::absolute(rnr::util::pathFromUtf8(script))));
    rnr::core::RnRController c;
    rnr::scripting::ScriptRunner runner(&c);
    std::string err;
    CHECK(runner.runStringBlocking("DIR = [[" + dir_utf8 + "]]\nlocal f = assert(loadfile([[" + script + "]]))\nf()", &err));
    if (!err.empty()) std::cout << "  " << err << "\n";
    CHECK(runner.runFileBlocking(rnr::scripting::resolveScriptPath(script), &err) == false); // DIR is not set in a fresh run
    std::filesystem::remove_all(dir, ec);
}

// A11: a modal G2 / G3 line with only I / J is an arc
void testA11ModalArc() {
    std::cout << "[A11] A modal arc line without X / Y is executed\n";
    const auto r = rnr::gcode::GCodeParser::parse("G21 G90\nG0 X10 Y0\nG2 X10 Y0 I-10 J0 F500\nI-10 J0\n");
    CHECK(!r.hasErrors());
    CHECK(r.arcs.size() == 2);
}

// P2: copies of a program share its source lines; a tool change finds its description by tool number
void testP2SharedProgram() {
    std::cout << "[P2] Program copies share the source lines; tool descriptions by tool number\n";
    const auto r = rnr::gcode::GCodeParser::parse("(T2 D=6 flat end mill)\nG21 G90\nG0 X1\nT2 M6\nG0 X2\n");
    const rnr::gcode::ParseResult copy = r;
    CHECK(copy.lines.data() == r.lines.data());
    CHECK(copy.lines.size() == 5 && copy.lines[3] == "T2 M6");
    CHECK(r.tool_descriptions.count(2) == 1);
    rnr::core::RnRController c;
    auto cfg = c.getConfig();
    cfg.is_mill = true;
    cfg.scripting.m6_script_enabled = false;
    c.applyConfig(cfg);
    CHECK(c.resetFromEStop());
    CHECK(c.loadGCodeString("(T2 D=6 flat end mill)\nG21 G90\nG0 X1\nT2 M6\nG0 X2\n"));
    c.startJob();
    CHECK(waitUntil([&] { return c.getTelemetry().state == ControllerState::ToolChangeHold; }, 8000));
    CHECK(c.getTelemetry().pending_tool_description.find("flat end mill") != std::string::npos);
    c.stopJob();
}

// M5: every setting that is written is also read back. The defaults are saved, every value in the file is changed
// (booleans flipped, numbers moved, texts extended), loaded and saved again: a line that comes back different
// belongs to a key that is not read (or is limited when it is read).
std::string mutateValue(const std::string& key, const std::string& v) {
    if (v == "0" || v == "1") return v == "0" ? "1" : "0";
    if (v == "true" || v == "false") return v == "true" ? "false" : "true";
    char* end = nullptr;
    const double d = std::strtod(v.c_str(), &end);
    if (!v.empty() && end && *end == '\0') {
        std::ostringstream os;
        os << (v.find('.') != std::string::npos ? d + 0.5 : d + 1.0);
        return os.str();
    }
    (void)key;
    return v + "x";
}

void testM5SettingsRoundTrip() {
    std::cout << "[M5] Every saved setting is read back\n";
    const std::string f1 = "regression_settings_a.ini", f2 = "regression_settings_b.ini";
    rnr::config::PersistentSettings def;
    def.machine.is_mill = true;
    CHECK(rnr::config::SettingsManager::saveToFile(f1, def));
    std::vector<std::string> lines;
    {
        std::ifstream in(f1);
        std::string line, section;
        while (std::getline(in, line)) {
            const auto eq = line.find(" = ");
            if (!line.empty() && line[0] == '[') section = line;
            if (eq != std::string::npos && section != "[Integrity]" && line[0] != ';') {
                line = line.substr(0, eq + 3) + mutateValue(line.substr(0, eq), line.substr(eq + 3));
            }
            lines.push_back(line);
        }
    }
    {
        std::ofstream out(f1, std::ios::trunc);
        for (const auto& l : lines) out << l << "\n";
    }
    rnr::config::PersistentSettings loaded;
    loaded.machine.is_mill = true;
    CHECK(rnr::config::SettingsManager::loadFromFile(f1, loaded));
    CHECK(rnr::config::SettingsManager::saveToFile(f2, loaded));
    std::map<std::string, std::string> back;
    {
        std::ifstream in(f2);
        std::string line, section;
        while (std::getline(in, line)) {
            if (!line.empty() && line[0] == '[') section = line;
            const auto eq = line.find(" = ");
            if (eq != std::string::npos) back[section + line.substr(0, eq)] = line.substr(eq + 3);
        }
    }
    // Values the loader limits to a valid range or a fixed set (not lost: corrected)
    const std::set<std::string> limited = {
        "pulse_rate_khz", "active_workspace", "gantry_slave", "operation_mode", "linear_units", "board_id", "mode",
        "source_type", "framing_mode", "reference_tool", "dark_theme_id", "light_theme_id", "macro_button_count",
        "stream_bind_address", "transport", "flavour", "count", "curve_accel_pct", "corner_tolerance_mm",
        "speed_margin_pct", "baud_rate", "slave_address", "estop_pin", "limit_plus_pin", "limit_minus_pin", "home_pin",
        "probe_pin", "arc_ok_pin", "thc_up_pin", "thc_down_pin", "chiller_flow_pin", "door_interlock_pin",
        "laser_fault_pin", "output_pin", "stream_port", "slice_interval_ms", "start_safe_z_mm", "spindle_ramp_up_s",
        "spindle_ramp_down_s", "window_width", "width", "height", "overlay_opacity"};
    std::string section;
    size_t checked = 0, lost = 0;
    for (const auto& l : lines) {
        if (!l.empty() && l[0] == '[') section = l;
        const auto eq = l.find(" = ");
        if (eq == std::string::npos || section == "[Integrity]" || l[0] == ';') continue;
        const std::string key = l.substr(0, eq), val = l.substr(eq + 3);
        std::string base = key;
        while (!base.empty() && std::isdigit(static_cast<unsigned char>(base.back()))) base.pop_back();
        while (!base.empty() && base.back() == '_') base.pop_back();
        if (limited.count(key) || limited.count(base) || key.find("pin") != std::string::npos) continue;
        ++checked;
        const auto it = back.find(section + key);
        const bool same = it != back.end() && (it->second == val ||
            (std::strtod(it->second.c_str(), nullptr) == std::strtod(val.c_str(), nullptr) && !val.empty() &&
             std::isdigit(static_cast<unsigned char>(val.back()))));
        if (!same) {
            ++lost;
            std::cout << "  not read back: " << section << " " << key << " = " << val << " -> "
                      << (it == back.end() ? std::string("(missing)") : it->second) << "\n";
        }
    }
    CHECK(checked > 150);
    CHECK(lost == 0);
    std::error_code ec;
    for (const auto& f : {f1, f2, f1 + ".bak", f2 + ".bak"}) std::filesystem::remove(f, ec);
}

// RNR board on the bench (rnr_hardware_check): the RNR Eco Motion 2.0 ignores Set Axis Positions, clears its step
// counters when the E-stop is reset and reports "sequence active" whenever it is armed. This fake board does the same.
class RnrLikeBoard final : public rnr::board::IBoardDriver {
public:
    rnr::board::BoardCapabilities capabilities() const override {
        rnr::board::BoardCapabilities c;
        c.accesses_real_hardware = false;
        return c;
    }
    bool open() override { open_ = true; return true; }
    void close() override { open_ = false; }
    bool isOpen() const override { return open_; }
    rnr::board::BoardInfo info() const override { return {"RNR-like test board", "TEST"}; }
    bool readStatus(rnr::board::BoardStatus& out, uint32_t timeout_ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(std::min<uint32_t>(timeout_ms, 2)));
        std::lock_guard<std::mutex> lock(m_);
        out = {};
        out.motion_active = armed_;            // Like the real board: set whenever it is armed, not only while moving
        out.ack_seq = seq_;
        out.live_steps = steps_;
        out.latched_steps = steps_;
        return true;
    }
    void configure(const rnr::core::MachineConfig&, bool) override {}
    void emergencyStop() override { std::lock_guard<std::mutex> lock(m_); armed_ = false; }
    void clearEmergencyStop() override { std::lock_guard<std::mutex> lock(m_); armed_ = true; steps_ = {}; }
    void cancelMotion() override {}
    void setOutputs(uint8_t, bool) override {}
    void setOutputsAfterMotion(uint8_t, bool) override {}
    void setSpindlePwm(uint16_t, bool) override {}
    void setAxisPositions(const std::array<int32_t, 4>&) override {} // Ignored, as on the real board
    void beginJog(rnr::core::Axis, bool) override {}
    void armHomeSwitchStop(rnr::core::Axis) override {}
    void armProbe(rnr::core::Axis) override {}
    bool sendSlice(rnr::board::SliceStream, const rnr::core::StepSlice& slice, bool, uint16_t* out_seq) override {
        std::lock_guard<std::mutex> lock(m_);
        for (int a = 0; a < 4; ++a) steps_[a] += slice.delta_steps[a];
        ++seq_;
        if (out_seq) *out_seq = seq_;
        return true;
    }
private:
    std::mutex m_;
    bool open_ = false, armed_ = false;
    uint16_t seq_ = 0;
    std::array<int32_t, 4> steps_{};
};

void testRnrCountersAreNotMachinePositions() {
    std::cout << "[RNR] A new machine position survives an E-stop although the board's counters keep their own zero\n";
    auto board = std::make_unique<RnrLikeBoard>();
    rnr::core::MachineController c(std::move(board), false);
    auto cfg = c.getConfig();
    cfg.is_mill = true;
    cfg.require_homing = false;
    cfg.probe.probe_pin = 0;
    c.applyConfig(cfg);
    CHECK(c.connect());
    CHECK(waitUntil([&] { return c.getTelemetry().usb_connected; }, 2000));
    CHECK(c.resetFromEStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    c.triggerEStop();                                  // Counters == position here: the feedback counts as proven
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    CHECK(c.resetFromEStop());
    c.restoreSavedPositions({100.0, 50.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}); // New machine zero (as homing sets)
    c.jogStep(rnr::core::Axis::X, 2.0, 1200.0);
    CHECK(waitUntil([&] { return c.getTelemetry().state == ControllerState::Idle; }, 5000));
    c.triggerEStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const auto t = c.getTelemetry();
    CHECK(std::abs(t.machine_pos_mm[0] - 102.0) < 1e-6); // Before the fix: 2.0 (the board's own count)
    CHECK(std::abs(t.machine_pos_mm[1] - 50.0) < 1e-6);
    // A move right after another one ends only when the board has had the time to run it
    CHECK(c.resetFromEStop());
    CHECK(c.executeMdiCommand("G1 X110 F3000"));
    const auto t0 = std::chrono::steady_clock::now();
    CHECK(waitUntil([&] { return c.getTelemetry().state == ControllerState::Idle; }, 5000));
    CHECK(std::chrono::steady_clock::now() - t0 >= std::chrono::milliseconds(150)); // 8 mm at 50 mm/s: ~160 ms + run-out
}

} // namespace

int main() {
    rnr::core::RnRController::setHardwareAccessEnabled(false);
    std::cout << "BreakoutCNC regression tests (code review findings)\n";

    testA1LaserS0();
    testA3HomeScript();
    testScriptsUseOnlyTheApi();
    testA4Axis();
    testA12A13Sandbox();
    testA6FeedOverrideSteps();
    testA9RestState();
    testA10MdiRollback();
    testA15AsyncConnect();
    testA17EmptyProgram();
    testA5Utf8Paths();
    testA11ModalArc();
    testP2SharedProgram();
    testM5SettingsRoundTrip();
    testRnrCountersAreNotMachinePositions();

    std::cout << "\n" << (g_checks - g_failures) << " of " << g_checks << " checks passed\n";
    if (g_failures) {
        std::cout << "REGRESSION TESTS FAILED\n";
        return 1;
    }
    std::cout << "ALL REGRESSION TESTS PASSED\n";
    return 0;
}
