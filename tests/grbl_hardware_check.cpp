// GRBL-family hardware check: drives a REAL grblHAL / GRBL 1.1 / FluidNC board on a serial port through the machine
// controller (connect, the board's settings, MDI, work offsets, jogging, a job with an arc, feed hold, resume, stop,
// a G53 job, spindle and coolant, a probe that finds nothing, homing, E-stop and reset, the console) and checks the
// position the board itself reports after every move.
//
// Only for a board on the bench: motors may turn and outputs switch, so nothing may be connected that can cut, burn or
// crash. Not a CTest test: run it by hand.
//   grbl_hardware_check.exe --yes-the-board-is-on-the-bench [--port COM3] [--baud 115200] [--board grblhal]
// The board's own settings are read, never written. The application side uses default settings; no settings file is
// read or written. Moves stay within 20 mm of where the board starts.
#include "core/board/board_registry.h"
#include "core/controller/machine_controller.h"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>

using namespace rnr;
using core::ControllerState;

namespace {

int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                                                    \
    do {                                                                               \
        ++g_checks;                                                                    \
        if (!(cond)) {                                                                 \
            ++g_failures;                                                              \
            std::printf("  FAILED line %d: %s\n", __LINE__, #cond);                    \
        }                                                                              \
    } while (0)

bool waitFor(const std::function<bool()>& pred, int ms) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < until) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return pred();
}

bool near(double a, double b, double tol = 0.01) { return std::abs(a - b) <= tol; }

struct Bench {
    std::unique_ptr<core::MachineController> c;
    std::array<double, 4> home{}; // Machine position when the check started

    core::TelemetrySnapshot t() const { return c->getTelemetry(); }
    void note(const char* what) const {
        const auto x = t();
        std::printf("  [%s] state %d busy %d fw '%s' X%.3f Y%.3f Z%.3f: %s\n", what, static_cast<int>(x.state),
                    x.board_busy ? 1 : 0, x.firmware_state.c_str(), x.machine_pos_mm[0], x.machine_pos_mm[1],
                    x.machine_pos_mm[2], x.active_fault_message.c_str());
    }
    bool idle(int ms = 20000) const {
        return waitFor([&] {
            const auto x = t();
            return x.state == ControllerState::Idle && !x.board_busy && x.firmware_state == "Idle";
        }, ms);
    }
    // The board's reported machine position equals home + offset (mm)
    bool at(double dx, double dy, double dz, double tol = 0.01) const {
        const auto p = t().machine_pos_mm;
        return near(p[0], home[0] + dx, tol) && near(p[1], home[1] + dy, tol) && near(p[2], home[2] + dz, tol);
    }
};

void reportPosition(const Bench& b, const char* what, double dx, double dy, double dz) {
    const auto p = b.t().machine_pos_mm;
    std::printf("  %-34s expected X%+.3f Y%+.3f Z%+.3f  board X%+.3f Y%+.3f Z%+.3f\n", what, dx, dy, dz,
                p[0] - b.home[0], p[1] - b.home[1], p[2] - b.home[2]);
}

} // namespace

int main(int argc, char** argv) {
    std::string port = "COM3", board_id = board::kGrblHalBoardId;
    int baud = 115200;
    bool confirmed = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--yes-the-board-is-on-the-bench") confirmed = true;
        else if (a == "--port" && i + 1 < argc) port = argv[++i];
        else if (a == "--baud" && i + 1 < argc) baud = std::atoi(argv[++i]);
        else if (a == "--board" && i + 1 < argc) board_id = argv[++i];
    }
    if (!confirmed) {
        std::printf("Drives a real GRBL-family board: motors may turn and outputs switch.\n"
                    "Run with --yes-the-board-is-on-the-bench [--port COM3] when nothing is connected that can cut, "
                    "burn or crash.\n");
        return 2;
    }

    Bench b;
    b.c = std::make_unique<core::MachineController>(nullptr, false);
    {
        auto cfg = b.c->getConfig();
        cfg.is_mill = true;
        cfg.require_homing = false;
        cfg.scripting.m6_script_enabled = false;
        cfg.board_id = board_id;
        auto& p = cfg.board_profiles[board_id];
        board::completeProfile(board_id, p);
        p.set("transport", "serial");
        p.set("address", port);
        p.setInt("baud", baud);
        b.c->applyConfig(cfg);
    }

    std::printf("[1] Connect to %s on %s\n", board_id.c_str(), port.c_str());
    CHECK(b.c->connect());
    CHECK(waitFor([&] { return !b.t().firmware_version.empty() && !b.t().board_connecting; }, 15000));
    b.note("connected");
    const auto caps = b.c->getBoardCapabilities();
    std::printf("  firmware %s, %d axes\n", b.t().firmware_version.c_str(), caps.axis_count);
    if (b.t().firmware_version.empty()) {
        std::printf("No board answered: stopping.\n");
        return 1;
    }

    std::printf("[2] The board's settings\n");
    b.c->requestFirmwareSettings();
    CHECK(waitFor([&] { return b.c->getFirmwareSettings().items.size() >= 20; }, 10000));
    {
        const auto st = b.c->getFirmwareSettings();
        size_t described = 0, probing = 0;
        for (const auto& it : st.items) {
            if (!it.name.empty()) ++described;
            if (it.group == "Probing") ++probing;
        }
        std::printf("  %zu settings, %zu with a name, %zu in group Probing\n", st.items.size(), described, probing);
        CHECK(described == st.items.size());
    }

    std::printf("[3] Reset E-stop\n");
    const bool armed = b.c->resetFromEStop();
    CHECK(armed);
    if (!armed) {
        // Without an enabled machine every motion check would fail: say why, as the application does, and stop
        b.note("reset refused");
        b.c->disconnect();
        std::printf("\nThe board cannot be enabled (see the message above): fix its inputs first.\n"
                    "%d of %d checks passed\nGRBL HARDWARE CHECK FAILED\n", g_checks - g_failures, g_checks);
        return 1;
    }
    if (b.t().firmware_state == "Alarm") b.c->unlockBoard();
    CHECK(b.idle());
    b.note("reset");
    b.home = b.t().machine_pos_mm;
    b.c->setWorkspaceOffset(0, {b.home[0], b.home[1], b.home[2], 0.0}); // Work zero where the board starts

    std::printf("[4] MDI moves and a work offset\n");
    CHECK(b.c->executeMdiCommand("G21 G90 G0 X10 Y5"));
    CHECK(b.idle());
    reportPosition(b, "G0 X10 Y5", 10, 5, 0);
    CHECK(b.at(10, 5, 0));
    CHECK(b.c->executeMdiCommand("G1 X12 Y6 Z-1 F300"));
    CHECK(b.idle());
    reportPosition(b, "G1 X12 Y6 Z-1", 12, 6, -1);
    CHECK(b.at(12, 6, -1));
    b.c->setWorkspaceOffset(0, {b.home[0] + 2.0, b.home[1] + 1.0, b.home[2], 0.0});
    CHECK(b.c->executeMdiCommand("G0 X0 Y0 Z0"));
    CHECK(b.idle());
    reportPosition(b, "G0 X0 Y0 Z0 (offset X+2 Y+1)", 2, 1, 0);
    CHECK(b.at(2, 1, 0));
    b.c->setWorkspaceOffset(0, {b.home[0], b.home[1], b.home[2], 0.0});

    std::printf("[5] Step jog and continuous jog\n");
    b.c->jogStep(core::Axis::X, 3.0, 300.0);
    CHECK(b.idle());
    reportPosition(b, "step jog X+3", 5, 1, 0);
    CHECK(b.at(5, 1, 0));
    const double jog_from = b.t().machine_pos_mm[1];
    for (int i = 0; i < 8; ++i) {
        b.c->jogStartContinuous(core::Axis::Y, true, 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    b.c->jogStopAxis(core::Axis::Y);
    CHECK(b.idle());
    const double jogged = b.t().machine_pos_mm[1] - jog_from;
    std::printf("  continuous jog Y+: %.3f mm, stopped\n", jogged);
    CHECK(jogged > 0.2 && jogged < 10.0);
    const double y_stopped = b.t().machine_pos_mm[1];
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    CHECK(near(b.t().machine_pos_mm[1], y_stopped, 0.001));
    CHECK(b.c->executeMdiCommand("G0 X0 Y0 Z0"));
    CHECK(b.idle());
    CHECK(b.at(0, 0, 0));

    std::printf("[6] Job with an arc, spindle and coolant\n");
    CHECK(b.c->loadGCodeString("G21 G90\nM3 S500\nM8\nG1 X10 F400\nG2 X20 Y0 I5 J0\nG1 Y4\nM9\nM5\nG0 X0 Y0\nM30\n"));
    b.c->startJob();
    CHECK(waitFor([&] { return b.t().state == ControllerState::RunningGCode; }, 5000));
    CHECK(waitFor([&] { return b.t().torch_or_spindle_on; }, 10000));
    b.note("spindle on");
    CHECK(waitFor([&] { return b.t().state == ControllerState::Idle; }, 90000));
    CHECK(b.idle());
    b.note("job done");
    reportPosition(b, "after the job", 0, 0, 0);
    CHECK(b.at(0, 0, 0));
    CHECK(!b.t().torch_or_spindle_on);

    std::printf("[7] Feed hold, resume, controlled stop\n");
    CHECK(b.c->loadGCodeString("G21 G90\nG1 X15 F300\nG1 X0\n"));
    b.c->startJob();
    CHECK(waitFor([&] { return b.t().machine_pos_mm[0] - b.home[0] > 2.0; }, 15000));
    b.c->pauseJob();
    CHECK(waitFor([&] { return b.t().state == ControllerState::PausedHold && b.t().firmware_state.rfind("Hold", 0) == 0; }, 5000));
    // The hold has ended once the position stays the same for 300 ms
    double held_x = b.t().machine_pos_mm[0];
    CHECK(waitFor([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        const double x = b.t().machine_pos_mm[0];
        const bool still = near(x, held_x, 0.0005);
        held_x = x;
        return still;
    }, 10000));
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    CHECK(near(b.t().machine_pos_mm[0], held_x, 0.001));
    std::printf("  held at X%+.3f\n", held_x - b.home[0]);
    b.c->startJob();
    CHECK(waitFor([&] { return b.t().machine_pos_mm[0] > held_x + 1.0; }, 10000));
    b.c->stopJob();
    CHECK(b.idle(20000));
    const double stop_x = b.t().machine_pos_mm[0];
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    CHECK(near(b.t().machine_pos_mm[0], stop_x, 0.001));
    b.note("stopped");

    std::printf("[8] G53 in a job after the work offset changed\n");
    CHECK(b.c->executeMdiCommand("G0 X0 Y0 Z0"));
    CHECK(b.idle());
    b.c->setWorkspaceOffset(0, {b.home[0], b.home[1], b.home[2] + 5.0, 0.0});
    char g53_job[160];
    std::snprintf(g53_job, sizeof(g53_job), "G21 G90\nG0 X3 Z1\nG53 G0 Z%.3f\nG0 X6\nM30\n", b.home[2] - 2.0);
    CHECK(b.c->loadGCodeString(g53_job));
    b.c->setWorkspaceOffset(0, {b.home[0], b.home[1], b.home[2] + 1.0, 0.0}); // Changed after loading
    b.c->startJob();
    CHECK(waitFor([&] { return b.t().state == ControllerState::RunningGCode; }, 5000));
    CHECK(waitFor([&] { return b.t().state == ControllerState::Idle; }, 30000));
    CHECK(b.idle());
    reportPosition(b, "G53 Z-2 then G0 X6", 6, 0, -2);
    CHECK(b.at(6, 0, -2)); // Z stays at the machine target; before the fix it followed the preview
    b.c->setWorkspaceOffset(0, {b.home[0], b.home[1], b.home[2], 0.0});
    CHECK(b.c->executeMdiCommand("G0 X0 Y0 Z0"));
    CHECK(b.idle());

    std::printf("[9] Outputs by hand\n");
    b.c->setAuxOutput(3, true); // Flood
    CHECK(waitFor([&] { return (b.t().outputs_bitmask & core::OutputBit::OUT3_FloodCoolant) != 0; }, 3000));
    b.c->setAuxOutput(3, false);
    CHECK(waitFor([&] { return (b.t().outputs_bitmask & core::OutputBit::OUT3_FloodCoolant) == 0; }, 3000));
    CHECK(b.c->executeMdiCommand("M3 S800"));
    CHECK(waitFor([&] { return b.t().torch_or_spindle_on; }, 3000));
    CHECK(b.c->executeMdiCommand("M5"));
    CHECK(waitFor([&] { return !b.t().torch_or_spindle_on; }, 5000));
    CHECK(b.idle());

    std::printf("[10] A probe move that touches nothing\n");
    b.c->probeRaw(core::Axis::Z, -3.0, 100.0);
    CHECK(waitFor([&] { return b.t().state == ControllerState::Probing; }, 3000));
    CHECK(waitFor([&] { return b.t().state != ControllerState::Probing; }, 60000));
    CHECK(b.idle());
    b.note("probe done");
    CHECK(!b.t().last_probe_hit);
    CHECK(b.c->executeMdiCommand("G0 Z0"));
    CHECK(b.idle());

    std::printf("[11] Homing (refused by a board with homing switched off)\n");
    b.c->homeAllAxes();
    CHECK(waitFor([&] { return b.t().state != ControllerState::Homing; }, 60000));
    b.note("after home");
    if (b.t().state == ControllerState::EStop) CHECK(b.c->resetFromEStop());
    if (b.t().firmware_state == "Alarm") b.c->unlockBoard();
    CHECK(b.idle());

    std::printf("[12] E-stop while moving, reset\n");
    CHECK(b.c->executeMdiCommand("G1 X15 F200"));
    CHECK(waitFor([&] { return b.t().firmware_state == "Run" && b.t().machine_pos_mm[0] - b.home[0] > 1.0; }, 10000));
    b.c->triggerEStop();
    CHECK(waitFor([&] { return b.t().state == ControllerState::EStop; }, 3000));
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    b.note("e-stop");
    CHECK(b.c->resetFromEStop());
    if (b.t().firmware_state == "Alarm") b.c->unlockBoard();
    CHECK(b.idle());
    b.note("after reset");

    std::printf("[13] Console\n");
    CHECK(b.c->sendBoardConsoleLine("$I"));
    CHECK(waitFor([&] {
        for (const auto& e : b.c->getBoardLog(0)) {
            if (e.text.find("[VER:") != std::string::npos) return true;
        }
        return false;
    }, 3000));

    std::printf("[14] Back to the start\n");
    char back[160];
    std::snprintf(back, sizeof(back), "G53 G0 Z%.3f", b.home[2]);
    CHECK(b.c->executeMdiCommand(back));
    CHECK(b.idle());
    std::snprintf(back, sizeof(back), "G53 G0 X%.3f Y%.3f", b.home[0], b.home[1]);
    CHECK(b.c->executeMdiCommand(back));
    CHECK(b.idle());
    reportPosition(b, "start position", 0, 0, 0);
    b.c->triggerEStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    b.c->disconnect();

    std::printf("\n%d of %d checks passed\n", g_checks - g_failures, g_checks);
    std::printf(g_failures == 0 ? "GRBL HARDWARE CHECK PASSED\n" : "GRBL HARDWARE CHECK FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
