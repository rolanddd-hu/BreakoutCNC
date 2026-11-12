#pragma once
// Shared by the script engine test files (one test executable, split by area)
// The checks in this file use assert(): keep them active in Release builds too (NDEBUG would
// silently compile every check out and every test would "pass").
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

#include "core/scripting/script_engine.h"
#include "core/scripting/script_api.h"
#include "core/scripting/script_runner.h"
#include "core/controller/motion_controller.h"
#include "core/controller/rnr_controller.h"
#include "core/board/board_driver.h"
#include <mutex>
#include "core/config/settings_manager.h"
#include "core/config/mach3_xml_importer.h"
#include "core/camera/camera_manager.h"
#include "core/camera/camera_stream_server.h"
#include "core/transport/tcp_stream.h"
#include "core/hid/rnr_protocol.h"
#include "core/gcode/gcode_parser.h"
#include "core/io/io_map.h"
#include "core/update/version_check.h"
#include "core/controller/program_check.h"
#include "ui_common/lua_guide_examples.h"
#include "ui_common/gcode_line_lookup.h"
#include <lua.h>
#include <lauxlib.h>
#include <map>
#include <iostream>
#include <cassert>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <thread>
#include <filesystem>
#include <functional>
#include <fstream>
#include <sstream>

// Polls until the controller's worker thread has caught up. Fixed sleeps were flaky on slow CI runners.
template <typename Pred>
inline bool waitUntil(Pred pred, int timeout_ms = 5000) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!pred()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

// Mock motion controller for scripting tests
class MockController : public rnr::core::IMotionController {
public:
    bool connect() override { return true; }
    void disconnect() override {}
    void setSimulatorMode(bool) override {}
    void setDryRunMode(bool) override {}
    void applyConfig(const rnr::core::MachineConfig& cfg) override { config_ = cfg; }
    rnr::core::MachineConfig getConfig() const override { return config_; }
    rnr::core::TelemetrySnapshot getTelemetry() const override { return telemetry_; }

    void triggerEStop() override {
        telemetry_.state = rnr::core::ControllerState::EStop;
    }
    bool resetFromEStop() override {
        telemetry_.state = rnr::core::ControllerState::Idle;
        return true;
    }
    void setLimitOverride(bool enabled) override {
        telemetry_.limit_override_active = enabled;
    }
    void setSimulatedInputPin(uint8_t pin, bool active) override {
        if (pin >= 1 && pin <= 12) {
            if (active) telemetry_.inputs_bitmask |= (1 << (pin - 1));
            else telemetry_.inputs_bitmask &= ~(1 << (pin - 1));
        }
    }

    void zeroWorkAxis(rnr::core::Axis axis) override {
        telemetry_.work_pos_mm[static_cast<size_t>(axis)] = 0.0;
    }
    void zeroAllWorkAxes() override {
        telemetry_.work_pos_mm = {0.0, 0.0, 0.0, 0.0};
    }
    void goToWorkZeroXY() override {}

    void selectWorkspace(uint8_t index) override {
        if (index < 6) {
            telemetry_.active_workspace = index;
            telemetry_.work_offset_mm   = telemetry_.workspace_offsets_mm[index];
            for (size_t a = 0; a < 4; ++a) {
                telemetry_.work_pos_mm[a] = telemetry_.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
            }
        }
    }
    void setWorkspaceOffset(uint8_t index, const std::array<double, 4>& offset_mm) override {
        if (index < 6) {
            telemetry_.workspace_offsets_mm[index] = offset_mm;
            if (telemetry_.active_workspace == index) {
                telemetry_.work_offset_mm = offset_mm;
                for (size_t a = 0; a < 4; ++a) {
                    telemetry_.work_pos_mm[a] = telemetry_.machine_pos_mm[a] - offset_mm[a];
                }
            }
        }
    }
    void setWorkspaceOffsetAxis(uint8_t index, rnr::core::Axis axis, double offset_mm) override {
        const size_t a = static_cast<size_t>(axis);
        if (index < 6 && a < 4) {
            telemetry_.workspace_offsets_mm[index][a] = offset_mm;
            if (telemetry_.active_workspace == index) {
                telemetry_.work_offset_mm[a] = offset_mm;
                telemetry_.work_pos_mm[a] = telemetry_.machine_pos_mm[a] - offset_mm;
            }
        }
    }
    uint8_t getActiveWorkspace() const override { return telemetry_.active_workspace; }
    std::array<std::array<double, 4>, 6> getWorkspaceOffsets() const override { return telemetry_.workspace_offsets_mm; }

    void jogStartContinuous(rnr::core::Axis, bool, double) override {}
    void jogStep(rnr::core::Axis, double, double) override {}
    void jogStopAxis(rnr::core::Axis) override {}
    void jogStop() override {}

    void setTorchOrSpindle(bool enabled) override {
        telemetry_.torch_or_spindle_on = enabled;
        if (enabled) telemetry_.outputs_bitmask |= rnr::core::OutputBit::OUT1_TorchOrSpindle;
        else telemetry_.outputs_bitmask &= ~rnr::core::OutputBit::OUT1_TorchOrSpindle;
    }
    void setAuxOutput(uint8_t out_index, bool enabled) override {
        if (out_index >= 1 && out_index <= 4) {
            if (enabled) telemetry_.outputs_bitmask |= (1 << (out_index - 1));
            else telemetry_.outputs_bitmask &= ~(1 << (out_index - 1));
            telemetry_.physical_outputs_bitmask = telemetry_.outputs_bitmask & 0x0F; // Default pin assignment
        }
    }
    void setSpindlePwm(uint16_t duty) override { telemetry_.spindle_pwm_duty = duty; }
    void setSpindleSpeedRpm(uint32_t rpm) override {
        telemetry_.commanded_spindle_rpm = static_cast<uint16_t>(rpm);
    }
    void resetSpindleSpeedOverride() override { telemetry_.spindle_override_active = false; }

    void setHuanyangVfdConfig(const rnr::HuanyangVfdConfig&) override {}
    rnr::HuanyangVfdConfig getHuanyangVfdConfig() const override { return {}; }
    rnr::HuanyangVfdTelemetry getHuanyangVfdTelemetry() const override { return {}; }
    void clearVfdAlarms() override {}
    void requestVfdParameterRead() override {}

    void homeAxis(rnr::core::Axis axis) override {
        telemetry_.axis_homed[static_cast<size_t>(axis)] = true;
    }
    void homeAllAxes() override {
        telemetry_.axis_homed = {true, true, true, true};
    }
    void probeAxis(rnr::core::Axis, double, double) override {
        telemetry_.probe_contact_active = true;
        telemetry_.last_probed_work_pos_mm = {10.0, 20.0, -15.5, 0.0};
    }
    void probeZWithToolSetter() override {}
    void probeWorkAxisEdge(rnr::core::Axis, bool) override {}
    void probeHoleCenterXY(double) override {}
    void probeRaw(rnr::core::Axis, double, double) override {
        telemetry_.probe_contact_active = true;
        telemetry_.last_probe_hit = true;
        telemetry_.last_probed_work_pos_mm = {10.0, 20.0, -15.5, 0.0};
    }
    void measureToolLength(uint16_t) override {}
    void selectToolInSpindle(uint16_t tool) override { telemetry_.current_tool_number = tool; }
    void activateToolLengthOffset(uint16_t tool) override { telemetry_.active_tlo_tool = tool; }
    void setWorkPosition(rnr::core::Axis axis, double value_mm) override {
        const size_t idx = static_cast<size_t>(axis);
        if (idx < 4) telemetry_.work_pos_mm[idx] = value_mm;
    }
    void restoreSavedPositions(
        const std::array<double, 4>& machine_pos_mm,
        const std::array<double, 4>& work_offset_mm) override
    {
        telemetry_.machine_pos_mm = machine_pos_mm;
        telemetry_.work_offset_mm = work_offset_mm;
        for (size_t i = 0; i < 4; ++i) {
            telemetry_.work_pos_mm[i] = machine_pos_mm[i] - work_offset_mm[i];
        }
    }

    void startSurfaceProbing(double, double, double, double, int, int, double = 0.5) override {
        telemetry_.surface_probing_active = true;
    }
    void cancelSurfaceProbing() override {
        telemetry_.surface_probing_active = false;
    }
    void applySurfaceHeightMap(const rnr::core::SurfaceHeightMap& map) override {
        telemetry_.surface_height_map = std::make_shared<const rnr::core::SurfaceHeightMap>(map);
        telemetry_.surface_compensation_active = map.is_valid;
    }
    void clearSurfaceHeightMap() override {
        telemetry_.surface_compensation_active = false;
    }
    bool isSurfaceCompensationActive() const override {
        return telemetry_.surface_compensation_active;
    }
    rnr::core::SurfaceHeightMap getSurfaceHeightMap() const override {
        return *telemetry_.surface_height_map;
    }
    bool getProgramBounds(std::array<double, 3>& out_min, std::array<double, 3>& out_max) const override {
        out_min = {0.0, 0.0, 0.0};
        out_max = {100.0, 100.0, 0.0};
        return true;
    }

    bool loadGCodeString(const std::string&) override { return true; }
    bool executeMdiCommand(const std::string& cmd) override {
        last_mdi_cmd_ = cmd;
        return true;
    }
    void setNextGCodeLine(size_t) override {}
    std::shared_ptr<const rnr::gcode::ParseResult> getProgram() const override { return program_; }
    std::shared_ptr<const rnr::core::ProgramCheckResult> getProgramCheck() const override { return nullptr; }
    void startJob() override { telemetry_.state = rnr::core::ControllerState::RunningGCode; }
    void pauseJob() override { telemetry_.state = rnr::core::ControllerState::PausedHold; }
    void stopJob() override { telemetry_.state = rnr::core::ControllerState::Idle; }
    void setFeedOverride(float pct) override { telemetry_.feed_override_pct = pct; }
    void setPierceDelayOverride(float) override {}

    void setOperationMode(rnr::core::OperationMode mode) override { telemetry_.operation_mode = mode; }
    rnr::core::OperationMode getOperationMode() const override { return telemetry_.operation_mode; }
    void setLaserPowerOverride(float pct) override { telemetry_.laser_power_override_pct = pct; }
    void testFireLaserPulse(float, float) override {}
    void frameBoundingBox() override {}
    void setAirAssist(bool enabled) override { telemetry_.laser_air_assist_active = enabled; }
    void setRedPointer(bool enabled) override { telemetry_.laser_red_pointer_active = enabled; }

    bool runScriptString(const std::string&, bool) override { return true; }
    bool runScriptFile(const std::string&, bool) override { return true; }
    void cancelScript() override {}
    bool isScriptRunning() const override { return false; }
    void submitScriptPromptResponse(const std::string&) override {}
    void setScriptLogCallback(std::function<void(const std::string&, bool)>) override {}

    // Public test inspection fields
    rnr::core::MachineConfig     config_;
    rnr::core::TelemetrySnapshot telemetry_;
    std::string                  last_mdi_cmd_;
    std::vector<rnr::core::ToolpathSegment> segments_;
    std::shared_ptr<const rnr::gcode::ParseResult> program_ = std::make_shared<const rnr::gcode::ParseResult>();
};

// Board driver test double: executes slices on the next status read and records every call
class FakeBoardDriver final : public rnr::board::IBoardDriver {
public:
    std::shared_ptr<std::atomic<bool>> closed = std::make_shared<std::atomic<bool>>(false);

    rnr::board::BoardCapabilities capabilities() const override {
        rnr::board::BoardCapabilities caps;
        caps.display_name = "Fake Board";
        caps.max_slices_in_flight = 2;
        caps.accesses_real_hardware = false; // Allowed while unit tests block real hardware
        return caps;
    }
    bool open() override { std::lock_guard<std::mutex> l(m_); open_ = true; return true; }
    void close() override { std::lock_guard<std::mutex> l(m_); open_ = false; closed->store(true); }
    bool isOpen() const override { std::lock_guard<std::mutex> l(m_); return open_; }
    rnr::board::BoardInfo info() const override { return {"Fake Board", "FAKE-1"}; }

    bool readStatus(rnr::board::BoardStatus& out, uint32_t) override {
        std::lock_guard<std::mutex> l(m_);
        for (const auto& s : pending_) {
            for (size_t a = 0; a < 4; ++a) steps_[a] += s.delta_steps[a];
        }
        pending_.clear();
        out.motion_active = false;
        out.ack_seq = seq_;
        out.live_steps = steps_;
        out.latched_steps = steps_;
        out.inputs_bitmask = 0;
        return true;
    }
    void configure(const rnr::core::MachineConfig&, bool) override { log("configure"); }
    void emergencyStop() override { log("estop"); std::lock_guard<std::mutex> l(m_); pending_.clear(); }
    void clearEmergencyStop() override { log("clear_estop"); }
    void cancelMotion() override { log("cancel"); std::lock_guard<std::mutex> l(m_); pending_.clear(); }
    void setOutputs(uint8_t mask, bool) override { log("outputs"); std::lock_guard<std::mutex> l(m_); outputs_ = mask; }
    void setOutputsAfterMotion(uint8_t mask, bool) override { log("outputs_after"); std::lock_guard<std::mutex> l(m_); outputs_ = mask; }
    void setSpindlePwm(uint16_t, bool) override { log("pwm"); }
    void setAxisPositions(const std::array<int32_t, 4>& s) override { log("positions"); std::lock_guard<std::mutex> l(m_); steps_ = s; }
    void beginJog(rnr::core::Axis, bool) override { log("begin_jog"); }
    void armHomeSwitchStop(rnr::core::Axis) override { log("arm_home"); }
    void armProbe(rnr::core::Axis) override { log("arm_probe"); }
    bool sendSlice(rnr::board::SliceStream stream, const rnr::core::StepSlice& slice, bool, uint16_t* out_seq) override {
        log(stream == rnr::board::SliceStream::Program ? "slice_program" : "slice_internal");
        std::lock_guard<std::mutex> l(m_);
        pending_.push_back(slice);
        max_in_flight_ = std::max(max_in_flight_, pending_.size());
        // A real board runs each slice for 25 ms: track how far ahead of real time the sender gets
        const auto now = std::chrono::steady_clock::now();
        busy_until_ = std::max(now, busy_until_) + std::chrono::milliseconds(25);
        max_lead_ = std::max(max_lead_, busy_until_ - now);
        ++seq_;
        if (out_seq) *out_seq = seq_;
        return true;
    }

    int count(const std::string& what) const { std::lock_guard<std::mutex> l(m_); return calls_.count(what) ? calls_.at(what) : 0; }
    int32_t steps(size_t axis) const { std::lock_guard<std::mutex> l(m_); return steps_[axis]; }
    uint8_t lastOutputs() const { std::lock_guard<std::mutex> l(m_); return outputs_; }
    size_t maxInFlight() const { std::lock_guard<std::mutex> l(m_); return max_in_flight_; }
    double maxLeadMs() const { std::lock_guard<std::mutex> l(m_); return std::chrono::duration<double, std::milli>(max_lead_).count(); }
    // Commands in the order received, consecutive slices merged into one "slices" entry
    std::vector<std::string> events() const { std::lock_guard<std::mutex> l(m_); return events_; }
    void clearEvents() { std::lock_guard<std::mutex> l(m_); events_.clear(); }

private:
    void log(const std::string& what) {
        std::lock_guard<std::mutex> l(m_);
        ++calls_[what];
        const bool slice = what.rfind("slice_", 0) == 0;
        if (!(slice && !events_.empty() && events_.back() == "slices")) events_.push_back(slice ? "slices" : what);
    }
    std::vector<std::string> events_;

    mutable std::mutex m_;
    bool open_ = false;
    uint16_t seq_ = 0;
    std::array<int32_t, 4> steps_{};
    std::vector<rnr::core::StepSlice> pending_;
    size_t max_in_flight_ = 0;
    std::chrono::steady_clock::time_point busy_until_{};
    std::chrono::steady_clock::duration max_lead_{};
    uint8_t outputs_ = 0;
    std::map<std::string, int> calls_;
};

// One raw HTTP exchange with a local server: the whole answer (the server closes the connection)
inline std::string httpRaw(uint16_t port, const std::string& request, int max_ms = 3000) {
    rnr::transport::TcpStream s;
    rnr::transport::ConnectionTarget t{"tcp", "127.0.0.1:" + std::to_string(port), ""};
    std::string err;
    if (!s.open(t, rnr::transport::StreamOptions{}, &err)) return "CONNECT FAILED: " + err;
    if (!request.empty() && !s.write(reinterpret_cast<const uint8_t*>(request.data()), request.size())) {
        return "WRITE FAILED";
    }
    std::string out;
    uint8_t buf[4096];
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(max_ms);
    while (std::chrono::steady_clock::now() < end) {
        const long n = s.read(buf, sizeof(buf), 100);
        if (n < 0) break;
        if (n > 0) out.append(reinterpret_cast<const char*>(buf), static_cast<size_t>(n));
    }
    return out;
}

inline std::string httpReq(const std::string& method, const std::string& path, const std::string& host,
                           const std::string& extra_headers, const std::string& body) {
    std::string r = method + " " + path + " HTTP/1.1\r\nHost: " + host + "\r\n" + extra_headers;
    if (method == "POST") r += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    return r + "\r\n" + body;
}

inline int httpStatus(const std::string& response) {
    return (response.rfind("HTTP/1.1 ", 0) == 0 && response.size() >= 12) ? std::atoi(response.c_str() + 9) : 0;
}


// Test blocks, in the order main() runs them
void test01();
void test02(MockController& mock_controller);
void test03(MockController& mock_controller);
void test04(MockController& mock_controller);
void test05(MockController& mock_controller);
void test06();
void test07();
void test08();
void test09();
void test11();
void test12();
void test13();
void test14();
void test15();
void test16();
void test17();
void test18();
void test19();
void test20();
void test21();
void test23();
void test23b();
void test24(MockController& mock_controller);
void test25();
void test26();
void test27();
void test28();
void test29();
void test30();
void test31();
void test32();
void test33();
void test34();
void test35();
void test36();
void test37();
void test38();
void test39();
void test41();
void test42();
void test43();
void test44();
void test45();
void test46();
void test47();
void test48();
void test49();
void test50();
void test51();
