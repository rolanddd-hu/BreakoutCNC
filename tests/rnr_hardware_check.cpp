// RNR Eco Motion 2.0 hardware check: drives a REAL board through the machine controller (jogging, MDI, jobs, feed
// hold, feed override, stop, E-stop, outputs, spindle PWM, laser S0) and compares the board's own step counters with
// the position the application commanded after every move.
//
// Only for a board on the bench (motors may turn, OUT1..OUT4 switch). Not a CTest test: run it by hand.
//   rnr_hardware_check.exe --yes-the-board-is-on-the-bench
// The controller uses its default settings (200 steps/mm, no switches); no settings file is read or written.
#include "core/board/board_driver.h"
#include "core/controller/machine_controller.h"
#include "core/gcode/gcode_parser.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pred();
}

// Forwards everything to the real RNR driver and keeps the last status report (the board's step counters)
class RecordingDriver final : public board::IBoardDriver {
public:
    explicit RecordingDriver(std::unique_ptr<board::IBoardDriver> real) : real_(std::move(real)) {}
    board::BoardCapabilities capabilities() const override { return real_->capabilities(); }
    bool open() override { return real_->open(); }
    void close() override { real_->close(); }
    bool isOpen() const override { return real_->isOpen(); }
    board::BoardInfo info() const override { return real_->info(); }
    bool readStatus(board::BoardStatus& out, uint32_t timeout_ms) override {
        const bool got = real_->readStatus(out, timeout_ms);
        if (got) {
            std::lock_guard<std::mutex> lock(m_);
            last_ = out;
            ++reports_;
        }
        return got;
    }
    void configure(const core::MachineConfig& cfg, bool homing) override { real_->configure(cfg, homing); }
    void emergencyStop() override { real_->emergencyStop(); }
    void clearEmergencyStop() override { real_->clearEmergencyStop(); }
    void cancelMotion() override { real_->cancelMotion(); }
    void setOutputs(uint8_t mask, bool lo) override {
        last_outputs_ = mask;
        real_->setOutputs(mask, lo);
    }
    void setOutputsAfterMotion(uint8_t mask, bool lo) override {
        last_outputs_ = mask;
        real_->setOutputsAfterMotion(mask, lo);
    }
    void setSpindlePwm(uint16_t duty, bool lo) override {
        last_pwm_ = duty;
        {
            std::lock_guard<std::mutex> lock(m_);
            if (pwm_log_.empty() || pwm_log_.back() != duty) pwm_log_.push_back(duty);
        }
        real_->setSpindlePwm(duty, lo);
    }
    void setAxisPositions(const std::array<int32_t, 4>& steps) override { real_->setAxisPositions(steps); }
    void beginJog(core::Axis axis, bool lo) override { real_->beginJog(axis, lo); }
    void armHomeSwitchStop(core::Axis axis) override { real_->armHomeSwitchStop(axis); }
    void armProbe(core::Axis axis) override { real_->armProbe(axis); }
    bool sendSlice(board::SliceStream stream, const core::StepSlice& slice, bool lo, uint16_t* seq) override {
        ++slices_;
        return real_->sendSlice(stream, slice, lo, seq);
    }

    board::BoardStatus last() const {
        std::lock_guard<std::mutex> lock(m_);
        return last_;
    }
    std::vector<uint16_t> pwmLog() const {
        std::lock_guard<std::mutex> lock(m_);
        return pwm_log_;
    }
    void clearPwmLog() {
        std::lock_guard<std::mutex> lock(m_);
        pwm_log_.clear();
    }
    uint64_t reports() const {
        std::lock_guard<std::mutex> lock(m_);
        return reports_;
    }
    std::atomic<uint8_t>  last_outputs_{0xFF};
    std::atomic<uint16_t> last_pwm_{0xFFFF};
    std::atomic<uint64_t> slices_{0};

private:
    std::unique_ptr<board::IBoardDriver> real_;
    mutable std::mutex m_;
    board::BoardStatus last_{};
    uint64_t reports_ = 0;
    std::vector<uint16_t> pwm_log_; // PWM values sent, in order (repeats folded)
};

struct Rig {
    RecordingDriver* drv = nullptr;
    std::unique_ptr<core::MachineController> c;
    core::TelemetrySnapshot t() const { return c->getTelemetry(); }
    bool idle(int ms = 20000) {
        return waitFor([&] {
            const auto x = t();
            return x.state == ControllerState::Idle || x.state == ControllerState::EStop;
        }, ms);
    }
    // Board step counters equal the commanded machine position (200 steps/mm) once the board is at rest
    // The board's counters restart from 0 at an E-stop reset and ignore Set Axis Positions: compare the movement since
    // the last re-zero (rebase), not absolute values
    std::array<double, 3> base_{};
    void rebase() {
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        const auto tel = t();
        const auto st = drv->last();
        for (int a = 0; a < 3; ++a) base_[a] = tel.machine_pos_mm[a] - st.live_steps[a] / 200.0;
    }
    bool positionMatches(const char* what) {
        std::this_thread::sleep_for(std::chrono::milliseconds(400)); // A few status reports after the last slice
        const auto tel = t();
        const auto st = drv->last();
        double worst = 0.0;
        for (int a = 0; a < 3; ++a) {
            const double board_mm = st.live_steps[a] / 200.0;
            worst = std::max(worst, std::abs(board_mm + base_[a] - tel.machine_pos_mm[a]));
        }
        std::printf("  %-34s app X%.3f Y%.3f Z%.3f | board X%.3f Y%.3f Z%.3f | diff %.4f mm%s\n", what,
                    tel.machine_pos_mm[0], tel.machine_pos_mm[1], tel.machine_pos_mm[2], st.live_steps[0] / 200.0,
                    st.live_steps[1] / 200.0, st.live_steps[2] / 200.0, worst,
                    st.motion_active ? " (board says moving)" : "");
        return worst <= 0.0051; // One step (the board's motion flag is reported only: it can stay set at rest)
    }
    void state(const char* where) const {
        const auto x = t();
        std::printf("  [%s] state %d usb %d: %s\n", where, static_cast<int>(x.state), x.usb_connected ? 1 : 0,
                    x.active_fault_message.c_str());
    }
    bool noCorrection() const {
        return t().active_fault_message.find("Position corrected") == std::string::npos;
    }
};

const char* kMillJob =
    "G21 G90 G17\n"
    "G0 X0 Y0 Z5\n"
    "G1 Z0 F600\n"
    "G1 X40 Y0 F2400\n"
    "G2 X40 Y30 I0 J15\n"
    "G1 X0 Y30\n"
    "G3 X0 Y0 I0 J-15\n"
    "G1 X20 Y10 F1800\n"
    "G1 X20.5 Y10.3\nG1 X21 Y10.4\nG1 X21.5 Y10.3\nG1 X22 Y10\n" // Short moves (curve speed limit)
    "G0 Z5\n"
    "G0 X0 Y0\n"
    "M30\n";

} // namespace

// Diagnostic: does the board keep positions written with Set Axis Positions, before and after an E-stop reset?
int setPositionExperiment() {
    auto drv = board::createBoardDriver(board::BoardType::RnrEcoMotion);
    if (!drv->open()) {
        std::printf("cannot open the board\n");
        return 1;
    }
    core::MachineConfig cfg;
    auto show = [&](const char* what) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        board::BoardStatus st{}, last{};
        bool got = false;
        for (int i = 0; i < 20; ++i) {
            if (drv->readStatus(st, 20)) {
                last = st;
                got = true;
            }
        }
        std::printf("  %-44s %s X%d Y%d Z%d state %d\n", what, got ? "" : "(no report)", last.live_steps[0],
                    last.live_steps[1], last.live_steps[2], last.motion_active ? 1 : 0);
    };
    drv->configure(cfg, false);
    drv->emergencyStop();
    show("open, configure, E-stop");
    drv->setAxisPositions({1000, 2000, 3000, 0});
    show("set 1000/2000/3000 while in E-stop");
    drv->clearEmergencyStop();
    show("clear E-stop");
    drv->configure(cfg, false);
    show("configure");
    drv->setAxisPositions({1100, 2100, 3100, 0});
    show("set 1100/2100/3100 right after");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    drv->setAxisPositions({1200, 2200, 3200, 0});
    show("set 1200/2200/3200 0.5 s later");
    drv->emergencyStop();
    show("E-stop");
    drv->clearEmergencyStop();
    show("clear E-stop");
    drv->close();
    return 0;
}

int main(int argc, char** argv) {
    if (argc >= 2 && std::strcmp(argv[1], "--set-position-experiment") == 0) return setPositionExperiment();
    if (argc < 2 || std::strcmp(argv[1], "--yes-the-board-is-on-the-bench") != 0) {
        std::printf("Drives a real RNR Eco Motion 2.0: motors turn and OUT1..OUT4 switch.\n"
                    "Run with --yes-the-board-is-on-the-bench when nothing is connected that can cut, burn or crash.\n");
        return 2;
    }
    std::printf("RNR Eco Motion 2.0 hardware check\n");
    Rig r;
    auto rec = std::make_unique<RecordingDriver>(board::createBoardDriver(board::BoardType::RnrEcoMotion));
    r.drv = rec.get();
    r.c = std::make_unique<core::MachineController>(std::move(rec), false);
    {
        auto cfg = r.c->getConfig();
        cfg.is_mill = true;
        cfg.require_homing = false;
        cfg.spindle_ramp_up_s = 0.0;
        cfg.spindle_ramp_down_s = 0.0;
        cfg.scripting.m6_script_enabled = false;
        cfg.probe.probe_pin = 0; // Nothing is wired: no probing input (the default is IN5)
        r.c->applyConfig(cfg);
    }

    std::printf("[1] Connect, status reports, reset\n");
    CHECK(r.c->connect());
    CHECK(waitFor([&] { return r.drv->reports() > 5; }, 3000));
    auto t0 = r.t();
    r.state("connected");
    std::printf("  board: %s, %llu reports\n", t0.product_name.c_str(), static_cast<unsigned long long>(r.drv->reports()));
    CHECK(t0.usb_connected && t0.state == ControllerState::EStop);
    CHECK(r.c->resetFromEStop());
    CHECK(r.t().state == ControllerState::Idle);
    CHECK(r.positionMatches("after reset"));

    std::printf("[2] Step jog and continuous jog\n");
    r.c->jogStep(core::Axis::X, 5.0, 1200.0);
    CHECK(waitFor([&] { return r.t().state == ControllerState::Jogging; }, 1000) || std::abs(r.t().machine_pos_mm[0] - 5.0) < 1e-6);
    CHECK(r.idle());
    CHECK(std::abs(r.t().machine_pos_mm[0] - 5.0) < 1e-6);
    CHECK(r.positionMatches("step jog X+5"));
    const auto jog_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
    while (std::chrono::steady_clock::now() < jog_until) {
        r.c->jogStartContinuous(core::Axis::Y, true, 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    r.c->jogStop();
    CHECK(r.idle());
    CHECK(r.t().machine_pos_mm[1] > 5.0);
    CHECK(r.positionMatches("continuous jog Y+"));
    // Dead-man: the jog stops on its own when the input is no longer refreshed
    r.c->jogStartContinuous(core::Axis::X, false, 0.3);
    CHECK(waitFor([&] { return r.t().state == ControllerState::Jogging; }, 500));
    CHECK(waitFor([&] { return r.t().state == ControllerState::Idle; }, 3000));
    CHECK(r.positionMatches("dead-man stop"));

    std::printf("[3] MDI moves, refused MDI changes nothing\n");
    r.state("start");
    CHECK(r.c->executeMdiCommand("G0 X0 Y0"));
    CHECK(r.idle());
    CHECK(r.c->executeMdiCommand("G1 X10 Y10 F1500"));
    CHECK(r.idle());
    CHECK(std::abs(r.t().work_pos_mm[0] - 10.0) < 1e-6 && std::abs(r.t().work_pos_mm[1] - 10.0) < 1e-6);
    CHECK(r.positionMatches("MDI G1 X10 Y10"));
    CHECK(!r.c->executeMdiCommand("G55 G1 X5 Q"));
    CHECK(r.t().active_workspace == 0);
    CHECK(r.c->executeMdiCommand("G0 X0 Y0 Z0"));
    CHECK(r.idle());
    r.c->zeroAllWorkAxes();

    std::printf("[4] Job to the end (arcs, short moves, spindle on OUT1)\n");
    r.state("start");
    CHECK(r.c->loadGCodeString(std::string("M3 S12000\n") + kMillJob));
    const auto prog = r.c->getProgram();
    const auto t_job = std::chrono::steady_clock::now();
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().state == ControllerState::RunningGCode; }, 2000));
    bool saw_spindle = false;
    CHECK(waitFor([&] {
        saw_spindle = saw_spindle || (r.drv->last_outputs_.load() & 0x01);
        return r.t().state == ControllerState::Idle;
    }, 60000));
    const double job_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_job).count();
    std::printf("  job took %.1f s, %llu slices sent\n", job_s, static_cast<unsigned long long>(r.drv->slices_.load()));
    CHECK(saw_spindle);
    CHECK(r.t().active_segment_index == prog->segments.size());
    CHECK((r.drv->last_outputs_.load() & 0x01) == 0); // Spindle off after M30
    CHECK(r.positionMatches("end of job"));
    CHECK(r.noCorrection());

    std::printf("[5] Feed hold and resume, small feed override steps\n");
    r.state("start");
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().active_segment_index >= 3; }, 5000));
    r.c->pauseJob();
    CHECK(waitFor([&] { return r.t().state == ControllerState::PausedHold && r.t().current_feed_mm_min < 0.1; }, 3000));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    CHECK(r.positionMatches("in feed hold"));
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().state == ControllerState::RunningGCode; }, 2000));
    for (int i = 1; i <= 10; ++i) {
        r.c->setFeedOverride(100.0f - 0.3f * i);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    CHECK(waitFor([&] { return r.t().state == ControllerState::Idle; }, 60000));
    r.c->setFeedOverride(100.0f);
    CHECK(r.t().active_segment_index == prog->segments.size());
    CHECK(r.positionMatches("hold, resume, override, end"));
    CHECK(r.noCorrection());

    std::printf("[6] Controlled stop mid-job, then E-stop mid-job\n");
    r.state("start");
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().active_segment_index >= 3; }, 5000));
    r.c->stopJob();
    CHECK(waitFor([&] { return r.t().state == ControllerState::Idle; }, 5000));
    CHECK(r.positionMatches("controlled stop"));
    r.c->setNextGCodeLine(0);
    r.c->startJob();
    CHECK(waitFor([&] { return r.t().active_segment_index >= 3; }, 5000));
    r.c->triggerEStop();
    CHECK(r.t().state == ControllerState::EStop);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    const auto st_after = r.drv->last();
    CHECK(!st_after.motion_active);
    const auto steps_a = st_after.live_steps;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    CHECK(r.drv->last().live_steps == steps_a); // Nothing moves after the E-stop
    CHECK(r.drv->last_outputs_.load() == 0);
    std::printf("  board counters after the E-stop: X%.3f Y%.3f Z%.3f (app X%.3f Y%.3f)\n", steps_a[0] / 200.0,
                steps_a[1] / 200.0, steps_a[2] / 200.0, r.t().machine_pos_mm[0], r.t().machine_pos_mm[1]);
    CHECK(r.c->resetFromEStop());
    r.rebase();
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    std::printf("  after E-stop: %s\n", r.t().active_fault_message.c_str());
    CHECK(r.positionMatches("after E-stop + reset"));

    std::printf("[7] Outputs and spindle PWM\n");
    r.state("start");
    for (uint8_t out = 1; out <= 4; ++out) {
        r.c->setAuxOutput(out, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        CHECK((r.drv->last_outputs_.load() & (1u << (out - 1))) != 0);
        r.c->setAuxOutput(out, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        CHECK((r.drv->last_outputs_.load() & (1u << (out - 1))) == 0);
    }
    r.c->setSpindleSpeedRpm(12000);
    CHECK(r.drv->last_pwm_.load() == 500);
    r.c->setSpindleSpeedRpm(24000);
    CHECK(r.drv->last_pwm_.load() == 1000);
    CHECK(r.c->executeMdiCommand("M8"));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    CHECK((r.drv->last_outputs_.load() & 0x04) != 0);
    CHECK(r.c->executeMdiCommand("M9"));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    CHECK((r.drv->last_outputs_.load() & 0x0C) == 0);

    std::printf("[8] Homing and probing without switches are refused, not stuck\n");
    r.state("start");
    r.c->homeAllAxes();
    CHECK(r.t().state == ControllerState::Idle);
    CHECK(r.t().active_fault_message.find("no home switch") != std::string::npos);
    r.state("after home all");
    r.c->probeAxis(core::Axis::Z, 0.0, 100.0);
    r.state("after probe");
    CHECK(r.t().state == ControllerState::Idle);
    CHECK(r.t().active_fault_message.find("no probe input") != std::string::npos);

    std::printf("[9] Laser: S0 lines send PWM 0 while the beam output is on\n");
    r.state("start");
    {
        auto cfg = r.c->getConfig();
        cfg.is_mill = false;
        cfg.operation_mode = core::OperationMode::Laser;
        cfg.laser.chiller_flow_pin = cfg.laser.door_interlock_pin = cfg.laser.laser_fault_pin = 0;
        cfg.laser.dynamic_power_m4 = false;
        cfg.laser.ignore_z_motion = true;
        r.c->applyConfig(cfg);
    }
    CHECK(r.c->executeMdiCommand("G0 X0 Y0"));
    CHECK(r.idle());
    CHECK(r.c->loadGCodeString("G21 G90\nM3 S500\nG1 X20 Y0 F1200\nG1 X40 S0\nG1 X60 S500\nM5\nM30\n"));
    r.drv->clearPwmLog();
    r.c->startJob();
    uint16_t max_pwm_s0 = 0, max_pwm_s500 = 0;
    bool beam_on_during_s0 = false;
    CHECK(waitFor([&] {
        const auto x = r.t();
        if (x.active_gcode_line_index == 3) {
            max_pwm_s0 = std::max<uint16_t>(max_pwm_s0, r.drv->last_pwm_.load());
            beam_on_during_s0 = beam_on_during_s0 || (r.drv->last_outputs_.load() & 0x01);
        }
        if (x.active_gcode_line_index == 4) max_pwm_s500 = std::max<uint16_t>(max_pwm_s500, r.drv->last_pwm_.load());
        return x.state == ControllerState::Idle && x.active_gcode_line_index >= 4;
    }, 30000));
    // The PWM is sent with the slices, ahead of the line the board reports: check the order of the commands
    const auto pwm = r.drv->pwmLog();
    std::string seq;
    for (auto d : pwm) seq += std::to_string(d) + " ";
    std::printf("  PWM commands in order: %s\n", seq.c_str());
    (void)max_pwm_s0;
    (void)max_pwm_s500;
    CHECK(beam_on_during_s0);
    bool zero_between = false;
    for (size_t i = 1; i + 1 < pwm.size(); ++i) zero_between = zero_between || (pwm[i] == 0 && pwm[i - 1] == 500 && pwm[i + 1] == 500);
    CHECK(zero_between);                                              // S500 -> S0 -> S500
    CHECK(std::find(pwm.begin(), pwm.end(), 1000) == pwm.end());      // Never full power
    CHECK(r.drv->last_pwm_.load() == 0);
    CHECK(r.positionMatches("end of laser job"));

    std::printf("[10] Disconnect and reconnect\n");
    r.state("start");
    for (int i = 0; i < 3; ++i) {
        r.c->disconnect();
        CHECK(!r.t().usb_connected);
        CHECK(r.c->connect());
        CHECK(waitFor([&] { return r.t().usb_connected; }, 2000));
    }
    CHECK(r.c->resetFromEStop());
    r.rebase();
    r.c->jogStep(core::Axis::X, 3.0, 1200.0);
    CHECK(r.idle());
    CHECK(r.positionMatches("jog after reconnects"));

    std::printf("[11] A new machine position (restored or homed) survives an E-stop\n");
    r.state("start");
    {
        // The board ignores Set Axis Positions: its counters keep their own count. The application's machine position
        // must never be replaced by them (it would move every work offset by the difference).
        r.c->restoreSavedPositions({100.0, 50.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0});
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        const auto before = r.t().machine_pos_mm;
        std::printf("  app X%.3f Y%.3f, board X%.3f Y%.3f\n", before[0], before[1], r.drv->last().live_steps[0] / 200.0,
                    r.drv->last().live_steps[1] / 200.0);
        r.c->jogStep(core::Axis::X, 2.0, 1200.0);
        CHECK(r.idle());
        r.c->triggerEStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        const auto after = r.t();
        std::printf("  after E-stop: app X%.3f Y%.3f: %s\n", after.machine_pos_mm[0], after.machine_pos_mm[1],
                    after.active_fault_message.c_str());
        CHECK(std::abs(after.machine_pos_mm[0] - 102.0) < 1e-6);
        CHECK(std::abs(after.machine_pos_mm[1] - 50.0) < 1e-6);
        CHECK(r.c->resetFromEStop());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        CHECK(std::abs(r.t().machine_pos_mm[0] - 102.0) < 1e-6);
    }

    std::printf("[12] Fresh start, then a new machine position (as homing sets it), then an E-stop\n");
    {
        r.c.reset();
        auto rec2 = std::make_unique<RecordingDriver>(board::createBoardDriver(board::BoardType::RnrEcoMotion));
        r.drv = rec2.get();
        r.c = std::make_unique<core::MachineController>(std::move(rec2), false);
        auto cfg = r.c->getConfig();
        cfg.is_mill = true;
        cfg.require_homing = false;
        cfg.probe.probe_pin = 0;
        r.c->applyConfig(cfg);
        CHECK(r.c->connect());
        CHECK(waitFor([&] { return r.drv->reports() > 5; }, 3000));
        CHECK(r.c->resetFromEStop());          // Counters cleared by the reset; app at 0
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        r.c->triggerEStop();                   // At rest in E-stop with counters == app: the feedback counts as proven
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        CHECK(r.c->resetFromEStop());
        r.c->restoreSavedPositions({100.0, 50.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}); // Like homing: new machine zero
        r.c->jogStep(core::Axis::X, 2.0, 1200.0);
        CHECK(r.idle());
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        r.c->triggerEStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        const auto after = r.t();
        std::printf("  app X%.3f Y%.3f, board X%.3f Y%.3f: %s\n", after.machine_pos_mm[0], after.machine_pos_mm[1],
                    r.drv->last().live_steps[0] / 200.0, r.drv->last().live_steps[1] / 200.0,
                    after.active_fault_message.c_str());
        CHECK(std::abs(after.machine_pos_mm[0] - 102.0) < 1e-6);
        CHECK(std::abs(after.machine_pos_mm[1] - 50.0) < 1e-6);
    }

    r.c.reset(); // E-stop and close
    std::printf("\n%d of %d checks passed\n", g_checks - g_failures, g_checks);
    std::printf(g_failures ? "RNR HARDWARE CHECK FAILED\n" : "RNR HARDWARE CHECK PASSED\n");
    return g_failures ? 1 : 0;
}
