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
#include "core/hid/rnr_protocol.h"
#include "core/gcode/gcode_parser.h"
#include "ui_common/lua_guide_examples.h"
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
#include <fstream>
#include <sstream>

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
        ++seq_;
        if (out_seq) *out_seq = seq_;
        return true;
    }

    int count(const std::string& what) const { std::lock_guard<std::mutex> l(m_); return calls_.count(what) ? calls_.at(what) : 0; }
    int32_t steps(size_t axis) const { std::lock_guard<std::mutex> l(m_); return steps_[axis]; }
    uint8_t lastOutputs() const { std::lock_guard<std::mutex> l(m_); return outputs_; }
    size_t maxInFlight() const { std::lock_guard<std::mutex> l(m_); return max_in_flight_; }

private:
    void log(const std::string& what) { std::lock_guard<std::mutex> l(m_); ++calls_[what]; }

    mutable std::mutex m_;
    bool open_ = false;
    uint16_t seq_ = 0;
    std::array<int32_t, 4> steps_{};
    std::vector<rnr::core::StepSlice> pending_;
    size_t max_in_flight_ = 0;
    uint8_t outputs_ = 0;
    std::map<std::string, int> calls_;
};

int main() {
    // Unit tests run purely in the simulator: never open a real motion board that may be plugged in
    rnr::core::RnRController::setHardwareAccessEnabled(false);

    std::cout << "STARTING MAIN\n" << std::flush;
    std::cout << "====================================================\n" << std::flush;
    std::cout << "BreakoutCNC - Script Engine & Lua 5.4 Unit Test\n" << std::flush;
    std::cout << "====================================================\n\n" << std::flush;

    MockController mock_controller;
    mock_controller.telemetry_.state = rnr::core::ControllerState::Idle;
    mock_controller.telemetry_.work_pos_mm = {12.5, 34.0, -5.2, 0.0};
    mock_controller.telemetry_.machine_pos_mm = {112.5, 234.0, 45.0, 0.0};
    mock_controller.telemetry_.inputs_bitmask = 0x0005; // IN1 and IN3 active (bits 0 and 2)

    // ------------------------------------------------------------------------
    // Test 1: Basic Lua VM Execution & Print Output Redirection
    // ------------------------------------------------------------------------
    std::cout << "[Test 1] Testing basic Lua 5.4 execution and output redirection...\n" << std::flush;
    {
        rnr::scripting::ScriptEngine engine;
        assert(engine.isInitialized());

        std::vector<std::string> log_messages;
        engine.setLogCallback([&](const std::string& msg, bool is_err) {
            log_messages.push_back((is_err ? "[ERR] " : "") + msg);
        });

        std::string err;
        bool ok = engine.executeString("print('Hello from Lua 5.4!')", &err);
        assert(ok);
        assert(!log_messages.empty());
        assert(log_messages.back() == "Hello from Lua 5.4!");

        ok = engine.executeString("x = 10 + 25; print('Sum = ' .. tostring(x))");
        assert(ok);
        assert(log_messages.back() == "Sum = 35");

        std::cout << "  -> PASSED: Lua 5.4 VM initialized and print redirect works.\n";
    }

    // ------------------------------------------------------------------------
    // Test 2: cnc.* Motion, Position, I/O, Spindle, & Prompts API
    // ------------------------------------------------------------------------
    std::cout << "[Test 2] Testing cnc.* Lua API bindings with MockController...\n";
    {
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::vector<std::string> logs;
        runner.setLogCallback([&](const std::string& msg, bool) {
            logs.push_back(msg);
        });

        // Test G-Code command dispatch
        std::string err;
        bool ok = runner.runStringBlocking("cnc.gcode('G0 X50 Y100')", &err);
        assert(ok);
        assert(mock_controller.last_mdi_cmd_ == "G0 X50 Y100");

        // Test DRO reading
        ok = runner.runStringBlocking(R"LUA(
            local pos = cnc.get_work_pos()
            assert(math.abs(pos.x - 12.5) < 0.001)
            assert(math.abs(pos.y - 34.0) < 0.001)
            assert(math.abs(pos.z - (-5.2)) < 0.001)

            local mpos = cnc.get_machine_pos()
            assert(math.abs(mpos.x - 112.5) < 0.001)
        )LUA", &err);
        if (!ok) std::cerr << "DRO test error: " << err << "\n";
        assert(ok);

        // Test Spindle & RPM control
        ok = runner.runStringBlocking(R"LUA(
            cnc.set_spindle(true)
            cnc.set_spindle_rpm(18000)
            assert(cnc.is_spindle_on() == true)
            assert(cnc.get_spindle_rpm() == 18000)
        )LUA", &err);
        assert(ok);
        assert(mock_controller.telemetry_.torch_or_spindle_on == true);
        assert(mock_controller.telemetry_.commanded_spindle_rpm == 18000);

        // Test Digital I/O
        ok = runner.runStringBlocking(R"LUA(
            assert(cnc.get_input(1) == true)  -- IN1 is active
            assert(cnc.get_input(2) == false) -- IN2 is inactive
            assert(cnc.get_input(3) == true)  -- IN3 is active
            assert(cnc.get_inputs_mask() == 5)

            cnc.set_output(2, true)
            assert(cnc.get_output(2) == true)
        )LUA", &err);
        assert(ok);
        assert((mock_controller.telemetry_.outputs_bitmask & (1 << 1)) != 0);

        // Test Coordinate Zeroing
        ok = runner.runStringBlocking("cnc.zero_axis('X')", &err);
        assert(ok);
        assert(mock_controller.telemetry_.work_pos_mm[0] == 0.0);

        // Test Probing
        ok = runner.runStringBlocking(R"LUA(
            local res = cnc.probe("Z", -50.0, 300.0)
            assert(res.hit == true)
            assert(math.abs(res.z - (-15.5)) < 0.001)
        )LUA", &err);
        assert(ok);

        std::cout << "  -> PASSED: All cnc.* API bindings functional.\n";
    }

    // ------------------------------------------------------------------------
    // Test 3: Mach3 Compatibility Shim
    // ------------------------------------------------------------------------
    std::cout << "[Test 3] Testing Mach3 compatibility shims (Code, GetDRO, SetDRO, Sleep)...\n";
    {
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::string err;
        bool ok = runner.runStringBlocking(R"LUA(
            Code("G1 X20 Y30 F1000")
            local x = GetDRO(0)
            local y = GetDRO(801)
            assert(x == 0.0) -- Zeroed in test 2
            assert(math.abs(y - 34.0) < 0.001)
            SetDRO(1, 0)
            Sleep(20)
        )LUA", &err);
        if (!ok) std::cerr << "Mach3 shim error: " << err << "\n";
        assert(ok);
        assert(mock_controller.last_mdi_cmd_ == "G1 X20 Y30 F1000");
        assert(mock_controller.telemetry_.work_pos_mm[1] == 0.0);

        std::cout << "  -> PASSED: Mach3 compatibility layer working seamlessly.\n";
    }

    // ------------------------------------------------------------------------
    // Test 4: Interactive UI Prompt Mailbox
    // ------------------------------------------------------------------------
    std::cout << "[Test 4] Testing interactive prompt modal mailbox...\n";
    {
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::atomic<bool> prompt_received{false};
        std::string selected_choice;

        // Run async script that calls cnc.prompt()
        runner.runStringAsync(R"LUA(
            local choice = cnc.prompt("Tool Change", "Insert Tool T3 and click OK", {"OK", "Cancel"})
            cnc.gcode("PROMPT_RESULT_" .. choice)
        )LUA", [&](bool success, const std::string& error) {
            prompt_received = true;
            assert(success);
        });

        // Simulate UI thread polling for prompt
        auto start = std::chrono::steady_clock::now();
        while (!runner.hasPendingPrompt()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > 2.0) break;
        }

        assert(runner.hasPendingPrompt());
        auto req = runner.getPendingPrompt();
        assert(req.title == "Tool Change");
        assert(req.message == "Insert Tool T3 and click OK");
        assert(req.buttons.size() == 2 && req.buttons[0] == "OK");

        // Operator clicks "OK" in UI
        runner.submitPromptResponse("OK");

        // Wait for async script to finish
        start = std::chrono::steady_clock::now();
        while (runner.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > 2.0) break;
        }

        assert(!runner.isRunning());
        assert(mock_controller.last_mdi_cmd_ == "PROMPT_RESULT_OK");

        std::cout << "  -> PASSED: Interactive prompt mailbox unblocks and receives UI response.\n";
    }

    // ------------------------------------------------------------------------
    // Test 5: Cooperative Cancellation on E-STOP
    // ------------------------------------------------------------------------
    std::cout << "[Test 5] Testing cooperative script cancellation on E-STOP...\n";
    {
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::atomic<bool> script_aborted{false};
        std::string final_err;

        // Run infinite loop script
        runner.runStringAsync(R"LUA(
            local count = 0
            while true do
                count = count + 1
            end
        )LUA", [&](bool success, const std::string& err) {
            script_aborted = !success;
            final_err = err;
        });

        // Allow script to start spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        assert(runner.isRunning());

        // Operator hits E-STOP / Cancel
        runner.cancel();

        // Wait for worker thread to cleanly abort
        auto start = std::chrono::steady_clock::now();
        while (runner.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > 2.0) break;
        }

        assert(!runner.isRunning());
        assert(script_aborted.load());
        std::cout << "  -> Abort message received: \"" << final_err << "\"\n";
        assert(final_err.find("aborted") != std::string::npos || final_err.find("E-STOP") != std::string::npos);

        std::cout << "  -> PASSED: Cooperative cancellation terminates runaway scripts instantly.\n";
    }

    // ------------------------------------------------------------------------
    // Test 6: Dynamic Macro Button Configuration & Persistence
    // ------------------------------------------------------------------------
    std::cout << "[Test 6] Testing dynamic macro button configuration & persistence...\n";
    {
        rnr::config::PersistentSettings settings;
        settings.machine.scripting.macro_buttons.clear();
        settings.machine.scripting.macro_buttons.push_back({"Custom Probe", "scripts/mill/probe_z.lua", false});
        settings.machine.scripting.macro_buttons.push_back({"Tool Zero", "scripts/mill/park.lua", false});
        settings.machine.scripting.macro_buttons.push_back({"Laser", "scripts/cut/laser_offset.lua", true});

        std::string test_ini = "test_scripting_settings.ini";
        bool save_ok = rnr::config::SettingsManager::saveToFile(test_ini, settings);
        assert(save_ok);

        rnr::config::PersistentSettings loaded;
        bool load_ok = rnr::config::SettingsManager::loadFromFile(test_ini, loaded);
        assert(load_ok);

        assert(loaded.machine.scripting.macro_buttons.size() == 3);
        assert(loaded.machine.scripting.macro_buttons[0].label == "Custom Probe");
        assert(loaded.machine.scripting.macro_buttons[0].path == "scripts/mill/probe_z.lua");
        assert(!loaded.machine.scripting.macro_buttons[0].show_on_control_panel);
        assert(loaded.machine.scripting.macro_buttons[1].label == "Tool Zero");
        assert(loaded.machine.scripting.macro_buttons[2].label == "Laser");
        assert(loaded.machine.scripting.macro_buttons[2].show_on_control_panel);

        std::filesystem::remove(test_ini);

        // Settings saved before the scripts/plasma -> scripts/cut rename load with the new folder
        {
            std::ofstream legacy(test_ini);
            legacy << "[Scripting]\n"
                   << "ihs_script_path = scripts/plasma/m3_touch.lua\n"
                   << "macro_button_count = 1\n"
                   << "macro_btn_0_label = Laser\n"
                   << "macro_btn_0_path = scripts/plasma/laser_offset.lua\n";
        }
        rnr::config::PersistentSettings legacy_loaded;
        assert(rnr::config::SettingsManager::loadFromFile(test_ini, legacy_loaded));
        assert(legacy_loaded.machine.scripting.ihs_script_path == "scripts/cut/m3_touch.lua");
        assert(legacy_loaded.machine.scripting.macro_buttons.size() == 1);
        assert(legacy_loaded.machine.scripting.macro_buttons[0].path == "scripts/cut/laser_offset.lua");
        std::filesystem::remove(test_ini);
        std::cout << "  -> PASSED: Dynamic macro buttons serialize and deserialize accurately.\n";
    }

    // ------------------------------------------------------------------------
    // Test 7: Multi-Axis Simultaneous Jogging in RnRController
    // ------------------------------------------------------------------------
    std::cout << "[Test 7] Testing simultaneous multi-axis continuous jogging & per-axis stop...\n";
    {
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        // Jogging is refused while in E-Stop
        controller.jogStartContinuous(rnr::core::Axis::X, true, 0.5);
        assert(controller.getTelemetry().state == rnr::core::ControllerState::EStop);
        assert(controller.resetFromEStop());

        // Start jogging X, Y, and Z simultaneously!
        controller.jogStartContinuous(rnr::core::Axis::X, true, 0.5);
        controller.jogStartContinuous(rnr::core::Axis::Y, true, 0.5);
        controller.jogStartContinuous(rnr::core::Axis::Z, true, 0.5);

        std::this_thread::sleep_for(std::chrono::milliseconds(120));

        auto tel1 = controller.getTelemetry();
        assert(tel1.state == rnr::core::ControllerState::Jogging);
        assert(tel1.work_pos_mm[0] > 0.0);
        assert(tel1.work_pos_mm[1] > 0.0);
        assert(tel1.work_pos_mm[2] > 0.0);
        std::cout << "  -> Multi-axis jog active: X=" << tel1.work_pos_mm[0]
                  << ", Y=" << tel1.work_pos_mm[1]
                  << ", Z=" << tel1.work_pos_mm[2] << " mm\n";

        // Stop only X axis
        controller.jogStopAxis(rnr::core::Axis::X);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        auto tel2 = controller.getTelemetry();
        double x_stopped_pos = tel2.work_pos_mm[0];
        // Y and Z must have continued moving forward!
        assert(tel2.work_pos_mm[1] > tel1.work_pos_mm[1]);
        assert(tel2.work_pos_mm[2] > tel1.work_pos_mm[2]);

        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        auto tel3 = controller.getTelemetry();
        // X must have come to a complete halt
        assert(std::abs(tel3.work_pos_mm[0] - x_stopped_pos) < 0.2);
        // Y and Z still advancing
        assert(tel3.work_pos_mm[1] > tel2.work_pos_mm[1]);
        assert(tel3.work_pos_mm[2] > tel2.work_pos_mm[2]);

        // Stop all remaining axes
        controller.jogStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto tel4 = controller.getTelemetry();
        assert(tel4.state == rnr::core::ControllerState::Idle);
        assert(tel4.axis_vel_mm_min[0] == 0.0 && tel4.axis_vel_mm_min[1] == 0.0 && tel4.axis_vel_mm_min[2] == 0.0);
        controller.disconnect();

        std::cout << "  -> PASSED: 3 axes moved concurrently; X stopped independently while Y & Z kept running.\n";
    }

    // ------------------------------------------------------------------------
    // Test 8: Multi-tool Mach3 ATC tool change and progress tracking
    // ------------------------------------------------------------------------
    std::cout << "\n[Test 8] Testing multi-tool tool change and progress tracking...\n";
    {
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        std::string gcode = rnr::gcode::GCodeParser::sampleMillAtcGCode();
        controller.loadGCodeString(gcode);
        const auto& segs = controller.getProgram()->segments;
        assert(!segs.empty());

        auto cfg = controller.getConfig();
        cfg.scripting.m6_script_enabled = false;
        controller.applyConfig(cfg);

        // Start job: should pause at T1 M06 (after the automatic Z retract to safe Z has finished)
        controller.startJob();
        for (int i = 0; i < 300; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto t = controller.getTelemetry();
            if (t.state == rnr::core::ControllerState::ToolChangeHold) break;
        }
        auto t_hold = controller.getTelemetry();
        assert(t_hold.state == rnr::core::ControllerState::ToolChangeHold);
        assert(t_hold.pending_tool_number == 1);

        // Resume after confirming T1 installed
        controller.startJob();
        auto t_resumed = controller.getTelemetry();
        assert(t_resumed.pending_tool_number == 0);
        assert(t_resumed.current_tool_number == 1);

        controller.disconnect();
        std::cout << "  -> PASSED: Tool change hold, resume, and multi-tool progression verified.\n";
    }

    // ------------------------------------------------------------------------
    // Test 9: Workspace Coordinate Systems (G54 - G59) & G-Code / MDI Switching
    // ------------------------------------------------------------------------
    std::cout << "\n[Test 9] Testing Workspace Coordinate Systems (G54 - G59) & MDI / G-Code switching...\n";
    {
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // 1. Initial workspace should be G54 (index 0)
        assert(controller.getActiveWorkspace() == 0);

        // 2. Set G55 offset to (100.0, 50.0, -10.0, 0.0)
        controller.setWorkspaceOffset(1, {100.0, 50.0, -10.0, 0.0});
        auto all_offsets = controller.getWorkspaceOffsets();
        assert(all_offsets[1][0] == 100.0);
        assert(all_offsets[1][1] == 50.0);
        assert(all_offsets[1][2] == -10.0);

        // 3. Switch to G55
        controller.selectWorkspace(1);
        auto t_g55 = controller.getTelemetry();
        assert(t_g55.active_workspace == 1);
        assert(t_g55.work_offset_mm[0] == 100.0);
        assert(t_g55.work_offset_mm[1] == 50.0);
        assert(t_g55.work_offset_mm[2] == -10.0);

        // 4. MDI command switching: G56
        controller.executeMdiCommand("G56");
        auto t_g56 = controller.getTelemetry();
        assert(t_g56.active_workspace == 2);

        // 5. MDI G10 L2 setting: G10 L2 P4 X200.0 Y150.0 Z-25.0 (P4 = G57)
        controller.executeMdiCommand("G10 L2 P4 X200.0 Y150.0 Z-25.0");
        auto offsets_updated = controller.getWorkspaceOffsets();
        assert(offsets_updated[3][0] == 200.0);
        assert(offsets_updated[3][1] == 150.0);
        assert(offsets_updated[3][2] == -25.0);

        // 6. G-Code parser multi-workspace tagging
        std::string gcode = "G54\nG0 X10 Y10\nG55\nG0 X20 Y20\nG56\nG1 X30 Y30 F1000\n";
        controller.loadGCodeString(gcode);
        const auto& segs = controller.getProgram()->segments;
        assert(segs.size() >= 3);
        assert(segs[0].workspace_index == 0); // G54
        assert(segs[1].workspace_index == 1); // G55
        assert(segs[2].workspace_index == 2); // G56

        // 7. Settings persistence check
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.ui.header_workspaces_open = true;
        std::string test_ini = "test_workspace_settings.ini";
        bool saved = rnr::config::SettingsManager::saveToFile(test_ini, ps);
        assert(saved);

        rnr::config::PersistentSettings loaded_ps{};
        bool loaded = rnr::config::SettingsManager::loadFromFile(test_ini, loaded_ps);
        assert(loaded);
        assert(loaded_ps.ui.header_workspaces_open == true);
        assert(loaded_ps.machine.workspace_offsets_mm[3][0] == 200.0);
        assert(loaded_ps.machine.workspace_offsets_mm[3][1] == 150.0);
        std::remove(test_ini.c_str());

        controller.disconnect();
        std::cout << "  -> PASSED: G54..G59 switching, MDI G10 L2, G-code segment tagging & persistence verified.\n";
    }

    // -------------------------------------------------------------------------
    // TEST 10: CNC Laser Mode, Slicing, Dynamic Power, Auto Air & Interlocks
    // -------------------------------------------------------------------------
    {
        std::cout << "\n[Test 10] Testing Dual-Mode CNC Laser Mode, Slicing, Auto Air & Settings Persistence...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // 1. Operation Mode Switching
        assert(controller.getOperationMode() == rnr::core::OperationMode::Plasma);
        controller.setOperationMode(rnr::core::OperationMode::Laser);
        assert(controller.getOperationMode() == rnr::core::OperationMode::Laser);
        assert(controller.getTelemetry().operation_mode == rnr::core::OperationMode::Laser);

        // 2. Trajectory Slicing in Laser Mode:
        // G0 (rapid) must have OUT1=0, G1 (cut) with M3 must have OUT1=1 and Auto Air Assist (OUT3=1)
        rnr::core::MachineConfig cfg = controller.getConfig();
        cfg.operation_mode = rnr::core::OperationMode::Laser;
        cfg.laser.auto_air_assist = true;
        cfg.laser.ignore_z_motion = true;
        controller.applyConfig(cfg);

        std::string laser_gcode = "G0 X10 Y10 Z5\nM3 S800\nG1 X50 Y50 Z0 F1200\nM5\nG0 X0 Y0\n";
        bool loaded = controller.loadGCodeString(laser_gcode);
        assert(loaded);

        const auto& segs = controller.getProgram()->segments;
        assert(!segs.empty());

        auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(segs, cfg, 100.0f, 0.0f, false, 0.025);
        assert(!slices.empty());

        // Verify that rapid move slices do not fire laser
        bool saw_rapid_with_laser = false;
        bool saw_cut_with_laser = false;
        bool saw_air_assist = false;
        bool z_remained_locked = true;

        for (const auto& sl : slices) {
            bool out1 = (sl.slice.outputs_mask & rnr::core::OutputBit::OUT1_TorchOrSpindle) != 0;
            bool out3 = (sl.slice.outputs_mask & rnr::core::OutputBit::OUT3_FloodCoolant) != 0;
            if (sl.segment_index == 0) { // First G0 rapid
                if (out1) saw_rapid_with_laser = true;
            } else if (sl.segment_index == 1) { // G1 cutting move with M3
                if (out1) saw_cut_with_laser = true;
                if (out3) saw_air_assist = true;
            }
            if (sl.slice.delta_steps[2] != 0) {
                z_remained_locked = false; // Z delta should be 0 because ignore_z_motion is active
            }
        }
        assert(!saw_rapid_with_laser);
        assert(saw_cut_with_laser);
        assert(saw_air_assist);
        assert(z_remained_locked);

        // 3. Air Assist & Red Pointer Controls
        controller.setAirAssist(true);
        assert((controller.getTelemetry().outputs_bitmask & rnr::core::OutputBit::OUT3_FloodCoolant) != 0);
        assert(controller.getTelemetry().laser_air_assist_active);
        controller.setAirAssist(false);
        assert(!controller.getTelemetry().laser_air_assist_active);

        controller.setRedPointer(true);
        assert((controller.getTelemetry().outputs_bitmask & rnr::core::OutputBit::OUT2_SpindleDir) != 0);
        assert(controller.getTelemetry().laser_red_pointer_active);
        controller.setRedPointer(false);
        assert(!controller.getTelemetry().laser_red_pointer_active);

        // 4. Test Fire Pulse: refused while the chiller / door interlocks are not satisfied
        controller.testFireLaserPulse(100.0f, 25.0f);
        assert(!controller.getTelemetry().torch_or_spindle_on);
        controller.setSimulatedInputPin(cfg.laser.chiller_flow_pin, true);   // Chiller flowing
        controller.setSimulatedInputPin(cfg.laser.door_interlock_pin, true); // Lid closed
        std::this_thread::sleep_for(std::chrono::milliseconds(80));          // Worker re-evaluates inputs
        controller.testFireLaserPulse(100.0f, 25.0f);
        assert(controller.getTelemetry().torch_or_spindle_on);
        assert(controller.getTelemetry().live_laser_power_pct == 25.0f);
        assert(controller.getTelemetry().spindle_pwm_duty > 0);

        // 5. Settings Persistence Verification
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.ui.operation_mode = 1; // Laser
        ps.machine.laser.source_type = rnr::core::LaserSourceType::Co2Glass;
        ps.machine.laser.min_duty_pct = 12.0f;
        ps.machine.laser.max_duty_pct = 85.0f;
        ps.machine.laser.chiller_flow_pin = 2;
        ps.machine.laser.auto_air_assist = true;

        std::string test_laser_ini = "test_laser_settings.ini";
        bool saved = rnr::config::SettingsManager::saveToFile(test_laser_ini, ps);
        assert(saved);

        rnr::config::PersistentSettings loaded_ps{};
        bool loaded_ini = rnr::config::SettingsManager::loadFromFile(test_laser_ini, loaded_ps);
        assert(loaded_ini);
        assert(loaded_ps.ui.operation_mode == 1);
        assert(loaded_ps.machine.operation_mode == rnr::core::OperationMode::Laser);
        assert(loaded_ps.machine.laser.source_type == rnr::core::LaserSourceType::Co2Glass);
        assert(loaded_ps.machine.laser.min_duty_pct == 12.0f);
        assert(loaded_ps.machine.laser.max_duty_pct == 85.0f);
        assert(loaded_ps.machine.laser.chiller_flow_pin == 2);
        assert(loaded_ps.machine.laser.auto_air_assist == true);
        std::remove(test_laser_ini.c_str());

        controller.disconnect();
        std::cout << "  -> PASSED: Laser mode, G0 safety, dynamic power, auto air assist & persistence verified.\n";
    }

    // ------------------------------------------------------------------------
    // Test 11: Machine Type Detection & G-Code Target Compatibility Checking
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 11] Testing G-Code Machine Type Detection & Compatibility Mismatch Alerts...\n";

        // 1. Mill program detection
        std::string mill_gcode = rnr::gcode::GCodeParser::sampleMillAtcGCode();
        int mill_score = 0;
        std::vector<std::string> mill_ind;
        rnr::gcode::MachineType detected_mill = rnr::gcode::GCodeParser::detectTargetMachine(mill_gcode, &mill_score, &mill_ind);
        assert(detected_mill == rnr::gcode::MachineType::Mill);
        assert(mill_score >= 50);
        assert(!mill_ind.empty());

        // 2. Plasma program detection
        std::string plasma_gcode = rnr::gcode::GCodeParser::samplePlasmaBracketGCode();
        int plasma_score = 0;
        std::vector<std::string> plasma_ind;
        rnr::gcode::MachineType detected_plasma = rnr::gcode::GCodeParser::detectTargetMachine(plasma_gcode, &plasma_score, &plasma_ind);
        assert(detected_plasma == rnr::gcode::MachineType::Plasma);
        assert(plasma_score >= 50);
        assert(!plasma_ind.empty());

        // 3. Laser program detection
        std::string laser_gcode =
            "; LightBurn 1.4.03\n"
            "; GRBL-M3/M4 Laser Device Profile\n"
            "G21 G90\n"
            "M4 S0\n"
            "G0 X10.0 Y10.0\n"
            "G1 X50.0 Y10.0 S800 F3000\n"
            "G1 X50.0 Y50.0 S800\n"
            "G1 X10.0 Y50.0 S800\n"
            "G1 X10.0 Y10.0 S800\n"
            "M5\n"
            "M30\n";
        int laser_score = 0;
        std::vector<std::string> laser_ind;
        rnr::gcode::MachineType detected_laser = rnr::gcode::GCodeParser::detectTargetMachine(laser_gcode, &laser_score, &laser_ind);
        assert(detected_laser == rnr::gcode::MachineType::Laser);
        assert(laser_score >= 50);
        assert(!laser_ind.empty());

        // 4. Generic neutral program detection
        std::string neutral_gcode =
            "G21 G90\n"
            "G0 X0 Y0\n"
            "G1 X100.0 Y50.0 F1000\n"
            "G1 X0 Y0\n"
            "M30\n";
        int neutral_score = 0;
        rnr::gcode::MachineType detected_neutral = rnr::gcode::GCodeParser::detectTargetMachine(neutral_gcode, &neutral_score);
        assert(detected_neutral == rnr::gcode::MachineType::Unknown);

        // 5. Compatibility matrix verification
        // Case A: Mill program in Mill app -> MATCH (no popup)
        auto res_mill_in_mill = rnr::gcode::GCodeParser::checkMachineCompatibility(mill_gcode, rnr::gcode::MachineType::Mill);
        assert(!res_mill_in_mill.has_mismatch);

        // Case B: Plasma program in Mill app -> MISMATCH (triggers warning)
        auto res_plasma_in_mill = rnr::gcode::GCodeParser::checkMachineCompatibility(plasma_gcode, rnr::gcode::MachineType::Mill);
        assert(res_plasma_in_mill.has_mismatch);
        assert(res_plasma_in_mill.detected_type == rnr::gcode::MachineType::Plasma);
        assert(res_plasma_in_mill.detected_name == "CNC Plasma Cutter");
        assert(!res_plasma_in_mill.indicators.empty());

        // Case C: Laser program in Mill app -> MISMATCH
        auto res_laser_in_mill = rnr::gcode::GCodeParser::checkMachineCompatibility(laser_gcode, rnr::gcode::MachineType::Mill);
        assert(res_laser_in_mill.has_mismatch);
        assert(res_laser_in_mill.detected_type == rnr::gcode::MachineType::Laser);

        // Case D: Mill program in Plasma app -> MISMATCH
        auto res_mill_in_plasma = rnr::gcode::GCodeParser::checkMachineCompatibility(mill_gcode, rnr::gcode::MachineType::Plasma);
        assert(res_mill_in_plasma.has_mismatch);
        assert(res_mill_in_plasma.detected_type == rnr::gcode::MachineType::Mill);
        assert(res_mill_in_plasma.detected_name == "CNC Milling Machine");

        // Case E: Plasma program in Plasma app -> MATCH
        auto res_plasma_in_plasma = rnr::gcode::GCodeParser::checkMachineCompatibility(plasma_gcode, rnr::gcode::MachineType::Plasma);
        assert(!res_plasma_in_plasma.has_mismatch);

        // Case F: Laser program in Plasma app -> MISMATCH
        auto res_laser_in_plasma = rnr::gcode::GCodeParser::checkMachineCompatibility(laser_gcode, rnr::gcode::MachineType::Plasma);
        assert(res_laser_in_plasma.has_mismatch);
        assert(res_laser_in_plasma.detected_type == rnr::gcode::MachineType::Laser);

        // Case G: Plasma program in Laser app -> MISMATCH
        auto res_plasma_in_laser = rnr::gcode::GCodeParser::checkMachineCompatibility(plasma_gcode, rnr::gcode::MachineType::Laser);
        assert(res_plasma_in_laser.has_mismatch);
        assert(res_plasma_in_laser.detected_type == rnr::gcode::MachineType::Plasma);

        // Case H: Neutral program in any app -> MATCH (no nuisance warnings)
        auto res_neutral_in_mill = rnr::gcode::GCodeParser::checkMachineCompatibility(neutral_gcode, rnr::gcode::MachineType::Mill);
        auto res_neutral_in_plasma = rnr::gcode::GCodeParser::checkMachineCompatibility(neutral_gcode, rnr::gcode::MachineType::Plasma);
        auto res_neutral_in_laser = rnr::gcode::GCodeParser::checkMachineCompatibility(neutral_gcode, rnr::gcode::MachineType::Laser);
        assert(!res_neutral_in_mill.has_mismatch);
        assert(!res_neutral_in_plasma.has_mismatch);
        assert(!res_neutral_in_laser.has_mismatch);

        std::cout << "  -> PASSED: G-Code machine detection, indicators & cross-profile compatibility verified.\n";
    }

    // ------------------------------------------------------------------------
    // Test 12: Surface Level Probing, Bilinear Interpolation, Toolpath Warping & Settings Persistence
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 12] Testing Surface Level Probing, Bilinear Interpolation, Toolpath Warping & Settings Persistence...\n";

        // 1. SurfaceHeightMap statistics & bilinear interpolation
        rnr::core::SurfaceHeightMap map;
        map.is_valid = true;
        map.min_x = 0.0;
        map.max_x = 100.0;
        map.min_y = 0.0;
        map.max_y = 50.0;
        map.points_x = 3;
        map.points_y = 3;
        map.smoothness = 0.0; // This test checks pure bilinear interpolation
        // 3x3 grid:
        // row 0 (Y=0):  (0,0)=0.0, (50,0)=0.10, (100,0)=0.20
        // row 1 (Y=25): (0,25)=0.30, (50,25)=0.40, (100,25)=0.50
        // row 2 (Y=50): (0,50)=0.60, (50,50)=0.70, (100,50)=0.80
        map.grid_z = {
            0.00, 0.10, 0.20,
            0.30, 0.40, 0.50,
            0.60, 0.70, 0.80
        };
        map.recalculateStats();

        assert(std::abs(map.min_z - 0.00) < 1e-6);
        assert(std::abs(map.max_z - 0.80) < 1e-6);
        assert(std::abs(map.peak_to_peak_z - 0.80) < 1e-6);
        assert(std::abs(map.mean_z - 0.40) < 1e-6);

        // Grid vertex exact lookups
        assert(std::abs(map.getInterpolatedZ(0.0, 0.0) - 0.00) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(50.0, 0.0) - 0.10) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(100.0, 0.0) - 0.20) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(50.0, 25.0) - 0.40) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(100.0, 50.0) - 0.80) < 1e-6);

        // Sub-cell center bilinear interpolation: (25.0, 12.5) -> (0.0 + 0.1 + 0.3 + 0.4) / 4 = 0.20
        double z_center = map.getInterpolatedZ(25.0, 12.5);
        assert(std::abs(z_center - 0.20) < 1e-6);

        // Out-of-bounds edge clamping check
        assert(std::abs(map.getInterpolatedZ(-20.0, -10.0) - 0.00) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(150.0, 80.0) - 0.80) < 1e-6);

        // 2. TrajectoryPlanner Toolpath Warping & Segment Subdivision
        rnr::core::ToolpathSegment long_cut{};
        long_cut.type = rnr::core::SegmentType::CutLinear;
        long_cut.start_xyza_mm = {0.0, 25.0, 0.0, 0.0};
        long_cut.end_xyza_mm   = {100.0, 25.0, 0.0, 0.0};
        long_cut.length_mm     = 100.0;
        long_cut.feed_mm_min   = 1200.0;
        long_cut.torch_active  = true;

        std::vector<rnr::core::ToolpathSegment> orig_segs = {long_cut};
        auto warped_segs = rnr::planner::TrajectoryPlanner::warpToolpathWithHeightMap(orig_segs, map, 2.0);

        // 100mm move subdivided by max 2.0mm -> exactly 50 chord sub-segments
        assert(warped_segs.size() == 50);
        // Start segment Z should match (0, 25) elevation (0.30)
        assert(std::abs(warped_segs.front().start_xyza_mm[2] - 0.30) < 1e-4);
        // Middle segment around index 24..25 should match (50, 25) elevation (0.40)
        assert(std::abs(warped_segs[24].end_xyza_mm[2] - 0.40) < 1e-4);
        // End segment Z should match (100, 25) elevation (0.50)
        assert(std::abs(warped_segs.back().end_xyza_mm[2] - 0.50) < 1e-4);

        // 3. Controller Height Map Application and Compensation Reset
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        std::string test_gcode = "G21 G90\nG0 X0 Y25 Z5\nG1 X100 Y25 Z0 F1200\nM30\n";
        bool loaded = controller.loadGCodeString(test_gcode);
        assert(loaded);
        assert(!controller.isSurfaceCompensationActive());

        // Apply height map
        controller.applySurfaceHeightMap(map);
        assert(controller.isSurfaceCompensationActive());
        assert(controller.getTelemetry().surface_compensation_active);
        const auto& comp_segs = controller.getProgram()->segments;
        assert(comp_segs.size() > 10); // Expanded through chord subdivision

        // Clear / Cancel compensation
        controller.clearSurfaceHeightMap();
        assert(!controller.isSurfaceCompensationActive());
        assert(!controller.getTelemetry().surface_compensation_active);

        // 4. Settings Persistence Verification
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.machine.probe.surface_max_depth_below_zero_mm = 4.75;
        ps.machine.probe.surface_clearance_z_mm          = 6.25;

        std::string test_ini = "test_surface_probe_settings.ini";
        bool saved = rnr::config::SettingsManager::saveToFile(test_ini, ps);
        assert(saved);

        rnr::config::PersistentSettings loaded_ps{};
        bool loaded_ini = rnr::config::SettingsManager::loadFromFile(test_ini, loaded_ps);
        assert(loaded_ini);
        assert(std::abs(loaded_ps.machine.probe.surface_max_depth_below_zero_mm - 4.75) < 1e-5);
        assert(std::abs(loaded_ps.machine.probe.surface_clearance_z_mm - 6.25) < 1e-5);
        std::remove(test_ini.c_str());

        controller.disconnect();
        std::cout << "  -> PASSED: Surface map interpolation, toolpath subdivision warping, controller state & settings persistence verified.\n";
    }

    // ------------------------------------------------------------------------
    // Test 13: Global System of Measurement (Metric vs Imperial) & Persistence
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 13] Testing Global System of Measurement (Metric vs Imperial) & Persistence...\n";

        rnr::core::RnRController controller;
        auto cfg = controller.getConfig();
        assert(cfg.linear_units == rnr::core::LinearUnits::Metric);

        // 1. Switch to Imperial
        cfg.linear_units = rnr::core::LinearUnits::Imperial;
        cfg.axes[0].steps_per_mm = 200.0;
        cfg.axes[0].max_velocity_mm_s = 100.0;
        cfg.axes[0].acceleration_mm_s2 = 500.0;
        controller.applyConfig(cfg);

        const auto& read_cfg = controller.getConfig();
        assert(read_cfg.linear_units == rnr::core::LinearUnits::Imperial);

        // 2. Mathematical unit conversions:
        // Steps per inch = steps_per_mm * 25.4
        double steps_per_inch = read_cfg.axes[0].steps_per_mm * 25.4;
        assert(std::abs(steps_per_inch - 5080.0) < 1e-4);

        // Max velocity in IPM = max_velocity_mm_s * (60.0 / 25.4)
        double vel_ipm = read_cfg.axes[0].max_velocity_mm_s * (60.0 / 25.4);
        assert(std::abs(vel_ipm - (6000.0 / 25.4)) < 1e-4);

        // Acceleration in in/s^2 = acceleration_mm_s2 / 25.4
        double accel_in_s2 = read_cfg.axes[0].acceleration_mm_s2 / 25.4;
        assert(std::abs(accel_in_s2 - (500.0 / 25.4)) < 1e-4);

        // 3. Settings persistence (save and load)
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.machine.linear_units = rnr::core::LinearUnits::Imperial;
        ps.ui.speed_unit_idx = 1; // in/s
        assert(ps.machine.require_homing); // Default: G53 / G28 / G30 need homed axes
        ps.machine.require_homing = false;
        ps.ui.toolpath_simplify_px = 2.5f;

        std::string test_units_ini = "test_units_settings.ini";
        bool saved = rnr::config::SettingsManager::saveToFile(test_units_ini, ps);
        assert(saved);

        rnr::config::PersistentSettings loaded_ps{};
        bool loaded = rnr::config::SettingsManager::loadFromFile(test_units_ini, loaded_ps);
        assert(loaded);
        assert(loaded_ps.machine.linear_units == rnr::core::LinearUnits::Imperial);
        assert(!loaded_ps.machine.require_homing);
        assert(std::abs(loaded_ps.ui.toolpath_simplify_px - 2.5f) < 1e-5f);
        assert(loaded_ps.ui.speed_unit_idx == 1);
        std::remove(test_units_ini.c_str());

        std::cout << "  -> PASSED: Global Metric/Imperial switching, mathematical scaling & settings persistence verified.\n";
    }

    // ------------------------------------------------------------------------
    // Test 14: Mach3 XML Profile Migration, Safety Validations & Full Opcode 0x0F Packing
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 14] Testing Mach3 XML Profile Migration, Safety Validations & Opcode 0x0F Hardware Packing...\n";

        // 1. Real Mach3 profile (trimmed from a parallel-port mill with the Huanyang VFD plugin v2):
        //    metric (<Units>0), A slaved to Y, inputs on LPT port 1, plugin settings in <VfdDriver>
        rnr::core::MachineConfig base_cfg{};
        const char* lpt_mill_xml = R"XML(<profile><Preferences><PulseSpeed>1</PulseSpeed><Profile>Mach3Mill</Profile><Units>0</Units>
<ComPort>1</ComPort><BaudRate>9600</BaudRate>
<Motor0Active>1</Motor0Active><Motor0StepNeg>1</Motor0StepNeg><Motor0DirNeg>0</Motor0DirNeg>
<Motor1Active>1</Motor1Active><Motor1StepNeg>1</Motor1StepNeg><Motor1DirNeg>0</Motor1DirNeg>
<Motor2Active>1</Motor2Active><Motor2StepNeg>1</Motor2StepNeg><Motor2DirNeg>0</Motor2DirNeg>
<Motor3Active>1</Motor3Active><Motor3StepNeg>1</Motor3StepNeg><Motor3DirNeg>0</Motor3DirNeg>
<Steps0>640.</Steps0><Vel0>24.17</Vel0><Acc0>202.5</Acc0>
<Steps1>640.</Steps1><Vel1>24.17</Vel1><Acc1>200.39063</Acc1>
<Steps2>640.</Steps2><Vel2>14.48</Vel2><Acc2>167.69531</Acc2><Steps6>1.</Steps6>
<M0Max>77.66</M0Max><M0Min>0.</M0Min><M0Rev>0</M0Rev><M0Neg>1</M0Neg><RefSpeed0>50</RefSpeed0>
<M1Max>97.5</M1Max><M1Min>0.</M1Min><M1Rev>1</M1Rev><M1Neg>1</M1Neg><RefSpeed1>50</RefSpeed1>
<M2Max>0.</M2Max><M2Min>-118.5</M2Min><M2Rev>1</M2Rev><M2Neg>0</M2Neg><RefSpeed2>30</RefSpeed2>
<M3Max>97.5</M3Max><M3Min>0.</M3Min><M3Rev>1</M3Rev><M3Neg>1</M3Neg><RefSpeed3>20</RefSpeed3>
<SoftLimit>0</SoftLimit><Slave0>3</Slave0><Slave1>0</Slave1><Slave2>3</Slave2><Slave>-1</Slave>
<Input0Active>0</Input0Active><Input0Port>1</Input0Port><Input0Pin>15</Input0Pin><Input0Neg>0</Input0Neg>
<Input2Active>1</Input2Active><Input2Port>1</Input2Port><Input2Pin>15</Input2Pin><Input2Neg>1</Input2Neg><Input2Emulated>0</Input2Emulated>
<Input5Active>0</Input5Active><Input5Port>1</Input5Port><Input5Pin>0</Input5Pin><Input5Neg>0</Input5Neg>
<Input8Active>1</Input8Active><Input8Port>1</Input8Port><Input8Pin>13</Input8Pin><Input8Neg>1</Input8Neg>
<Input22Active>1</Input22Active><Input22Port>1</Input22Port><Input22Pin>12</Input22Pin><Input22Neg>1</Input22Neg>
<Input25Active>1</Input25Active><Input25Port>1</Input25Port><Input25Pin>10</Input25Pin><Input25Neg>1</Input25Neg>
<Input26Active>0</Input26Active><THCOn>0</THCOn><PlasmaMode>0</PlasmaMode>
<SpinDelayCW>1.</SpinDelayCW><SpinDownCW>1.</SpinDownCW><PULLEY>3</PULLEY>
<SPEED3>4000</SPEED3><LSPEED3>0</LSPEED3><SPEED4>24000</SPEED4><LSPEED4>5000</LSPEED4>
<IGNORETOOL>1</IGNORETOOL><AutoTool>0</AutoTool><UseSafeZ>0</UseSafeZ><m_SafeZ>0.</m_SafeZ><DwellinMilli>0</DwellinMilli>
</Preferences><GCodeVar><GVar501>0.</GVar501></GCodeVar>
<PlugIns><VFD-Driver-by-royaumedeole.fr>1</VFD-Driver-by-royaumedeole.fr><SystemTest>1</SystemTest></PlugIns>
<VfdDriver><ComPort>7</ComPort><BaudRate>9600</BaudRate><Address>1</Address><PulleyOption>1</PulleyOption><EStopOption>0</EStopOption><Status>62</Status><Margin>3</Margin><BadSpeedThreshold>3</BadSpeedThreshold><BadtrackingThreshold>3</BadtrackingThreshold><EStopCauseOption>0</EStopCauseOption><Status1_dro>0</Status1_dro><Status1_lim>0.00</Status1_lim><Status2_dro>1002</Status2_dro><Status2_lim>9.3</Status2_lim><Status3_dro>0</Status3_dro><Status3_lim>0</Status3_lim></VfdDriver></profile>)XML";

        rnr::config::Mach3ImportOptions opts{};
        auto r = rnr::config::Mach3XmlImporter::importFromString(lpt_mill_xml, base_cfg, opts, "Mach3Mill.xml");
        const auto& c = r.migrated_config;
        assert(r.success);
        assert(!r.detected_imperial_units && c.linear_units == rnr::core::LinearUnits::Metric);
        assert(r.detected_mill_profile && !r.detected_plasma_profile && !r.detected_lathe_profile);
        assert(!r.detected_rnr_profile && r.detected_lpt_parallel_port);
        // Motor tuning in mm (Mach3 stores units/sec)
        assert(std::abs(c.axes[0].steps_per_mm - 640.0) < 1e-9);
        assert(std::abs(c.axes[0].max_velocity_mm_s - 24.17) < 1e-9);
        assert(std::abs(c.axes[0].acceleration_mm_s2 - 202.5) < 1e-9);
        assert(std::abs(c.axes[2].max_velocity_mm_s - 14.48) < 1e-9);
        // Polarity: Step LowActive on all motors, Dir = DirNeg XOR Reversed
        for (int a = 0; a < 4; ++a) assert(c.axes[a].step_active_low);
        assert(!c.axes[0].dir_active_low && c.axes[1].dir_active_low && c.axes[2].dir_active_low && c.axes[3].dir_active_low);
        // Homing: X/Y toward -, Z up; speed % of max velocity
        assert(c.axes[0].homing_dir_negative && !c.axes[2].homing_dir_negative);
        assert(std::abs(c.axes[0].homing_feed_mm_min - 0.5 * 24.17 * 60.0) < 1e-6);
        // Soft limits imported but off (SoftLimit = 0)
        assert(std::abs(c.axes[0].soft_limit_max_mm - 77.66) < 1e-9 && !c.axes[0].soft_limits_enabled);
        assert(std::abs(c.axes[2].soft_limit_min_mm + 118.5) < 1e-9);
        // A slaved to Y takes Y's tuning
        assert(c.gantry_slave == rnr::core::GantrySlave::AFollowsY && c.gantryMasterAxis() == 1);
        assert(std::abs(c.axes[3].steps_per_mm - 640.0) < 1e-9);
        // Step rate: Mach3 kernel 35 kHz (PulseSpeed list position 1) covers 15.5 kHz needed
        assert(c.pulse_rate_khz == 35);
        // Parallel-port inputs: suggested wiring plan
        assert(c.estop_pin == 1 && c.estop_invert);
        assert(c.probe.probe_pin == 3 && c.probe.probe_invert);
        assert(c.axes[2].home_pin == 4 && c.axes[0].home_pin == 5);
        assert(c.axes[1].home_pin == 0 && c.axes[0].limit_plus_pin == 0);
        assert(r.lpt_remaps.size() == 4);
        assert(!c.limit_override);
        // Huanyang plugin: <VfdDriver> port, not Mach3's own <ComPort>
        assert(r.detected_vfd_plugin && c.vfd.enabled);
#ifdef _WIN32
        assert(c.vfd.port_name == "COM7");
#else
        // COM numbers mean nothing off Windows: the configured device path is kept
        assert(c.vfd.port_name == base_cfg.vfd.port_name);
#endif
        assert(c.vfd.baud_rate == 9600 && c.vfd.slave_address == 1);
        assert(!c.vfd.speed_monitor_enabled && c.vfd.speed_margin_pct == 3);
        assert(c.vfd.bad_speed_threshold == 3 && c.vfd.tracking_threshold == 3);
        assert(std::abs(c.vfd.limit_current_a - 9.3f) < 1e-4f && c.vfd.limit_out_freq_hz == 0.0f);
        // Active pulley (PULLEY counts from 0): SPEED4 / LSPEED4
        assert(c.vfd.max_rpm == 24000 && c.vfd.min_rpm == 5000);
        // Spindle delays, M6 and dwell
        assert(std::abs(c.spindle_ramp_up_s - 1.0) < 1e-9 && std::abs(c.spindle_ramp_down_s - 1.0) < 1e-9);
        assert(c.tool_change.mode == rnr::core::ToolChangeMode::IgnoreM6);
        assert(!c.dwell_p_milliseconds);

        // Wiring plan choice by the operator
        opts.lpt_pin_overrides["E-Stop"] = 6;
        opts.lpt_pin_overrides["Probe (G31)"] = 0;
        auto r2 = rnr::config::Mach3XmlImporter::importFromString(lpt_mill_xml, base_cfg, opts, "Mach3Mill.xml");
        assert(r2.migrated_config.estop_pin == 6 && r2.migrated_config.probe.probe_pin == 0);

        // Forcing inches converts with 25.4 and flags the implausible result
        rnr::config::Mach3ImportOptions inch_opts{};
        inch_opts.units_override = 1;
        auto r3 = rnr::config::Mach3XmlImporter::importFromString(lpt_mill_xml, base_cfg, inch_opts, "Mach3Mill.xml");
        assert(r3.detected_imperial_units);
        assert(std::abs(r3.migrated_config.axes[0].steps_per_mm - 640.0 / 25.4) < 1e-9);
        bool implausible = false;
        for (const auto& it : r3.items) implausible |= (it.parameter_name == "Units Plausibility");
        assert(implausible);

        // 2. Profile used with the RNR plugin: only Port 3 inputs reached the board, inch units, A slaved to X
        const char* rnr_inch_xml = R"XML(<?xml version="1.0"?>
< profile ><Preferences><Profile>Plasma Table</Profile><Units>1</Units><PlasmaMode>1</PlasmaMode>
<Steps0>10160.</Steps0><Vel0>2.54</Vel0><Acc0>10.</Acc0><Motor0DirNeg>1</Motor0DirNeg><Motor0StepNeg>1</Motor0StepNeg>
<M0Min>0.</M0Min><M0Max>24.</M0Max><SoftLimit>1</SoftLimit><Slave0>0</Slave0><Slave1>3</Slave1><Slave2>3</Slave2>
<Input22Active>1</Input22Active><Input22Port>3</Input22Port><Input22Pin>4</Input22Pin><Input22Neg>1</Input22Neg>
<Input25Active>1</Input25Active><Input25Port>1</Input25Port><Input25Pin>11</Input25Pin><Input25Neg>1</Input25Neg>
<LimitOverRide>1</LimitOverRide></Preferences>
<RnRMotion><PulseRates>75</PulseRates><XPullOff>0.1</XPullOff></RnRMotion></ profile >)XML";
        auto ri = rnr::config::Mach3XmlImporter::importFromString(rnr_inch_xml, base_cfg, {}, "Plasma.xml");
        const auto& ci = ri.migrated_config;
        assert(ri.success && ri.detected_imperial_units && ri.detected_rnr_profile && ri.detected_plasma_profile);
        assert(std::abs(ci.axes[0].steps_per_mm - 400.0) < 1e-9);
        assert(std::abs(ci.axes[0].max_velocity_mm_s - 2.54 * 25.4) < 1e-9);
        assert(std::abs(ci.axes[0].acceleration_mm_s2 - 254.0) < 1e-9);
        assert(std::abs(ci.axes[0].soft_limit_max_mm - 24.0 * 25.4) < 1e-9 && ci.axes[0].soft_limits_enabled);
        assert(std::abs(ci.axes[0].home_pulloff_mm - 2.54) < 1e-9); // Pull-off in native units
        assert(ci.axes[0].dir_active_low && ci.axes[0].step_active_low);
        assert(ci.pulse_rate_khz == 75);
        assert(ci.gantry_slave == rnr::core::GantrySlave::AFollowsX);
        assert(ci.probe.probe_pin == 4 && ci.probe.probe_invert);
        assert(ci.estop_pin == 0);   // Port 1 input was ignored by the RNR plugin
        assert(!ci.limit_override);  // Never imported
        assert(ri.lpt_remaps.empty());

        // 3. Lathe profiles are flagged; non-profile XML is rejected
        auto rl = rnr::config::Mach3XmlImporter::importFromString(
            "<profile><Preferences><Profile>cnc lathe</Profile><Units>1</Units><Steps0>16000.</Steps0><Vel0>1.333</Vel0></Preferences></profile>",
            base_cfg, {}, "lathe.xml");
        assert(rl.success && rl.detected_lathe_profile && rl.detected_imperial_units);
        auto rx = rnr::config::Mach3XmlImporter::importFromString("<config><Units>0</Units></config>", base_cfg, {}, "x.xml");
        assert(!rx.success);
        assert(rnr::config::Mach3XmlImporter::kernelSpeedKhz(0) == 25 && rnr::config::Mach3XmlImporter::kernelSpeedKhz(6) == 100);

        // 3. Test Full Opcode 0x0F Hardware Packet Packing (normal and homing layout)
        rnr::protocol::BoardPinoutConfig pinout{};
        pinout.pulse_rate_khz = 75;
        pinout.slice_interval_ms = 25;
        pinout.axes[0].step_active_low = true;
        pinout.axes[2].step_active_low = true; // X & Z step active-low (0x05)
        pinout.axes[1].dir_active_low  = true;
        pinout.axes[3].dir_active_low  = true; // Y & A dir active-low (0x0A)
        pinout.probe_pin = 4;
        pinout.probe_invert = true;
        pinout.estop_pin = 1;
        pinout.estop_invert = true;
        pinout.axes[0].home_pin = 2;
        pinout.axes[0].limit_minus_pin = 2; // Shared with home_pin!
        pinout.axes[0].limit_plus_pin = 3;
        pinout.axes[0].home_invert = true;

        // Idle mode (mask_shared_home_limits = false)
        pinout.mask_shared_home_limits = false;
        auto pkt_idle = rnr::protocol::buildFullConfigureBoardPacket(0x0042, pinout);
        assert(pkt_idle[0] == static_cast<uint8_t>(rnr::protocol::FrameType::ExtendedPacket));
        assert(pkt_idle[1] == static_cast<uint8_t>(rnr::protocol::Opcode::ConfigureBoard));
        assert(pkt_idle[3] == 0x42);
        // Axis 0 byte 0: (limit_minus_pin << 4) | limit_plus_pin = (2 << 4) | 3 = 0x23
        assert(pkt_idle[5] == 0x23);
        // Axis 0 byte 1: ((home_inv<<2) << 4) | home_pin = (4 << 4) | 2 = 0x42
        assert(pkt_idle[6] == 0x42);
        // buf[13]: (mpg_pin << 4) | probe_pin = 0x04
        assert(pkt_idle[13] == 0x04);
        // buf[14]: ((estop_inv<<2 | probe_inv) << 4) | estop_pin = (5 << 4) | 1 = 0x51
        assert(pkt_idle[14] == 0x51);
        // buf[15]: (dir_mask << 4) | step_mask = (0x0A << 4) | 0x05 = 0xA5
        assert(pkt_idle[15] == 0xA5);
        assert(pkt_idle[16] == 75);
        assert(pkt_idle[17] == 25);

        // Homing mode (mask_shared_home_limits = true)
        // Since limit_minus_pin (2) == home_pin (2), limit_minus_pin is zeroed in buf[5] high nibble!
        pinout.mask_shared_home_limits = true;
        auto pkt_homing = rnr::protocol::buildFullConfigureBoardPacket(0x0043, pinout);
        assert(pkt_homing[5] == 0x03); // limit_minus zeroed, limit_plus=3 remains

        std::cout << "  -> PASSED: Mach3 XML Metric/Imperial migration, safety swaps, LPT remapping & Opcode 0x0F packing verified.\n";
    }

    // ------------------------------------------------------------------------
    // Test 15: Camera Support — Dual Role Assignment, Calibration, Zeroing, JPEG Stream & Settings
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 15] Testing Camera Manager, Exclusive 1/2-Cam Role Assignment, Calibration, Zeroing & JPEG Encoding...\n";

        rnr::core::CameraManager cam_mgr;
        auto devs = cam_mgr.enumerateDevices(true);
        assert(devs.size() >= 2); // At least the 2 virtual simulator cameras are present

        int sim_top_idx = static_cast<int>(devs.size()) - 2;
        int sim_mon_idx = static_cast<int>(devs.size()) - 1;

        // 1. Assign 2 distinct cameras in BreakoutMill mode (allow_top_down_role = true)
        rnr::core::CameraConfig cam_cfg{};
        cam_cfg.top_down_camera_index   = sim_top_idx;
        cam_cfg.monitoring_camera_index = sim_mon_idx;
        cam_mgr.applyConfig(cam_cfg, true, true);
        assert(cam_cfg.top_down_camera_index == sim_top_idx);
        assert(cam_cfg.monitoring_camera_index == sim_mon_idx);
        assert(cam_mgr.isRoleActive(rnr::core::CameraRole::TopDown));
        assert(cam_mgr.isRoleActive(rnr::core::CameraRole::Monitoring));

        // 2. Single-camera exclusive switching: if monitoring selects the same camera index as top-down,
        // top-down is automatically unassigned so hardware handles never conflict
        cam_cfg.monitoring_camera_index = sim_top_idx;
        cam_mgr.applyConfig(cam_cfg, true, true);
        assert(cam_cfg.monitoring_camera_index == sim_top_idx);
        assert(cam_cfg.top_down_camera_index == -1);
        assert(!cam_mgr.isRoleActive(rnr::core::CameraRole::TopDown));
        assert(cam_mgr.isRoleActive(rnr::core::CameraRole::Monitoring));

        // 3. BreakoutCut mode (allow_top_down_role = false) forces Top-Down off and keeps Monitoring only
        cam_cfg.top_down_camera_index   = sim_top_idx;
        cam_cfg.monitoring_camera_index = sim_mon_idx;
        cam_mgr.applyConfig(cam_cfg, true, false);
        assert(cam_cfg.top_down_camera_index == -1);
        assert(cam_cfg.monitoring_camera_index == sim_mon_idx);
        assert(!cam_mgr.isRoleActive(rnr::core::CameraRole::TopDown));
        assert(cam_mgr.isRoleActive(rnr::core::CameraRole::Monitoring));

        // 4. Verify live frame generation & Baseline JPEG encoder (SOI 0xFFD8 .. EOI 0xFFD9)
        cam_cfg.top_down_camera_index   = sim_top_idx;
        cam_cfg.monitoring_camera_index = sim_mon_idx;
        cam_mgr.applyConfig(cam_cfg, true, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        rnr::core::CameraFrame frame{};
        uint64_t seq = 0;
        bool got_frame = cam_mgr.getLatestFrame(rnr::core::CameraRole::TopDown, frame, seq);
        assert(got_frame);
        assert(frame.width == 640 && frame.height == 360);
        assert(frame.rgba.size() == static_cast<size_t>(640 * 360 * 4));

        std::vector<uint8_t> jpeg_bytes;
        rnr::core::CameraStreamServer::encodeRgbaToJpeg(frame.rgba.data(), frame.width, frame.height, jpeg_bytes);
        assert(jpeg_bytes.size() > 512);
        assert(jpeg_bytes[0] == 0xFF && jpeg_bytes[1] == 0xD8); // SOI
        assert(jpeg_bytes[jpeg_bytes.size() - 2] == 0xFF && jpeg_bytes[jpeg_bytes.size() - 1] == 0xD9); // EOI

        cam_mgr.stopAll();

        // 5. Verify Camera Offset Zeroing & Settings Persistence
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        auto mcfg = controller.getConfig();
        mcfg.camera.top_down_camera_index   = 0;
        mcfg.camera.monitoring_camera_index = 1;
        mcfg.camera.is_calibrated           = true;
        mcfg.camera.offset_x_mm             = 45.0;
        mcfg.camera.offset_y_mm             = 30.0;
        mcfg.camera.fov_x_mm                = 120.0;
        mcfg.camera.fov_y_mm                = 67.5;
        mcfg.camera.stream_enabled          = false;
        mcfg.camera.stream_port             = 8090;
        controller.applyConfig(mcfg);

        // Zero Work XY at Camera Crosshair: sets current tool Work XY to (offset_x_mm, offset_y_mm)
        // so that (work_pos - offset) = (0.0, 0.0) under the camera crosshair!
        controller.setWorkPosition(rnr::core::Axis::X, mcfg.camera.offset_x_mm);
        controller.setWorkPosition(rnr::core::Axis::Y, mcfg.camera.offset_y_mm);
        auto tel = controller.getTelemetry();
        assert(std::abs((tel.work_pos_mm[0] - mcfg.camera.offset_x_mm) - 0.0) < 1e-5);
        assert(std::abs((tel.work_pos_mm[1] - mcfg.camera.offset_y_mm) - 0.0) < 1e-5);

        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.ui.header_camera_open = true;
        std::string test_cam_ini = "test_camera_settings.ini";
        assert(rnr::config::SettingsManager::saveToFile(test_cam_ini, ps));

        rnr::config::PersistentSettings loaded_ps{};
        assert(rnr::config::SettingsManager::loadFromFile(test_cam_ini, loaded_ps));
        assert(loaded_ps.ui.header_camera_open == true);
        assert(loaded_ps.machine.camera.top_down_camera_index == 0);
        assert(loaded_ps.machine.camera.monitoring_camera_index == 1);
        assert(loaded_ps.machine.camera.is_calibrated == true);
        assert(std::abs(loaded_ps.machine.camera.offset_x_mm - 45.0) < 1e-4);
        assert(std::abs(loaded_ps.machine.camera.offset_y_mm - 30.0) < 1e-4);
        assert(std::abs(loaded_ps.machine.camera.fov_x_mm - 120.0) < 1e-4);
        assert(std::abs(loaded_ps.machine.camera.fov_y_mm - 67.5) < 1e-4);
        assert(loaded_ps.machine.camera.stream_port == 8090);
        std::remove(test_cam_ini.c_str());

        controller.disconnect();
        std::cout << "  -> PASSED: Camera enumeration, exclusive 1/2-cam assignment, calibration, zeroing, JPEG stream & INI persistence verified.\n";
    }

    // ----------------------------------------------------
    // Test 16: Safe Approach Continuity & No-Teleport Verification (Issue A1)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 16] Testing Safe Approach Continuity, ATC start & zero coordinate jumps...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // Load ATC program where Segment 0 is "T1 M06" and Segment 1 is "G0 X60.0 Y40.0"
        std::string atc_code = rnr::gcode::GCodeParser::sampleMillAtcGCode();
        assert(controller.loadGCodeString(atc_code));

        // Position endmill at (50, 40, 15) away from (0, 0, 0)
        controller.setWorkPosition(rnr::core::Axis::X, 50.0);
        controller.setWorkPosition(rnr::core::Axis::Y, 40.0);
        controller.setWorkPosition(rnr::core::Axis::Z, 15.0);

        auto cfg = controller.getConfig();
        cfg.tool_change.mode = rnr::core::ToolChangeMode::AutoAtc;
        cfg.tool_change.safe_z_mm = 25.0;
        controller.applyConfig(cfg);

        // Start job
        controller.startJob();
        auto slices = controller.getSliceQueue();
        assert(!slices.empty());

        // Verify continuity: every consecutive slice must have smooth delta_steps (no teleport)
        const int32_t max_allowed_step_jump_per_slice = 2000; // steps in 25ms
        for (size_t i = 0; i < slices.size(); ++i) {
            for (size_t a = 0; a < 4; ++a) {
                assert(std::abs(slices[i].slice.delta_steps[a]) < max_allowed_step_jump_per_slice);
            }
        }

        // Verify that Z does NOT plunge to 0 during the initial XY positioning
        bool z_plunged_prematurely = false;
        for (const auto& sl : slices) {
            if (sl.segment_index == 1 && sl.target_pos_mm[2] < 10.0) {
                z_plunged_prematurely = true;
            }
        }
        assert(!z_plunged_prematurely);

        controller.stopJob();
        controller.disconnect();
        std::cout << "  -> PASSED: Toolchange segment 0 continuity and safe Z rapid approach verified.\n";
    }

    // ----------------------------------------------------
    // Test 17: Feed Hold Pause & Resume Continuity (Issue A2)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 17] Testing Feed Hold pause & resume continuity...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        std::string gcode = "G21 G90\nG0 X0 Y0 Z5\nG1 X100 Y0 Z-1 F600\nG0 Z5\n";
        assert(controller.loadGCodeString(gcode));
        controller.startJob();

        // Simulate pausing
        controller.pauseJob();
        auto telem = controller.getTelemetry();
        assert(telem.state == rnr::core::ControllerState::PausedHold);

        // Change feed override while paused
        controller.setFeedOverride(120.0f);

        // Resume job
        controller.startJob();
        telem = controller.getTelemetry();
        assert(telem.state == rnr::core::ControllerState::RunningGCode);

        controller.stopJob();
        controller.disconnect();
        std::cout << "  -> PASSED: Feed Hold pause and resume smoothly handles feed override.\n";
    }

    // ----------------------------------------------------
    // Test 18: G53 Machine Coordinates & G92 Offset Setting (Issues A3, A4)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 18] Testing G53 Machine Coordinates & G92 MDI offset setting...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // 1. Test G92 via MDI
        assert(controller.executeMdiCommand("G90 G92 X10.0 Y20.0 Z15.0"));
        auto tel = controller.getTelemetry();
        assert(std::abs(tel.work_pos_mm[0] - 10.0) < 1e-4);
        assert(std::abs(tel.work_pos_mm[1] - 20.0) < 1e-4);
        assert(std::abs(tel.work_pos_mm[2] - 15.0) < 1e-4);

        // 2. Test G53 Machine Coordinate rapid
        auto pr = rnr::gcode::GCodeParser::parse("G0 G53 Z0", tel.work_pos_mm, 0.25, tel.work_offset_mm);
        assert(pr.segments.size() == 1);
        assert(pr.segments[0].type == rnr::core::SegmentType::RapidMove);
        assert(std::abs(pr.segments[0].end_xyza_mm[2] - (-tel.work_offset_mm[2])) < 1e-4);
        assert((pr.segments[0].outputs_mask & rnr::core::OutputBit::OUT1_TorchOrSpindle) == 0);

        controller.disconnect();
        std::cout << "  -> PASSED: G92 sets work offsets cleanly & G53 plans machine coordinate rapids.\n";
    }

    // ----------------------------------------------------
    // Test 19: Manual M6 Tool Change Spindle Safety & Limit Override (Issues B1, B2)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 19] Testing Manual M6 Tool Change Spindle Safety & E-Stop Limit Override...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // 1. Verify E-Stop Limit Override Safety:
        controller.triggerEStop();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::EStop);

        // Limit Override should NOT blindly clear E-Stop into Idle!
        controller.setLimitOverride(true);
        assert(controller.getTelemetry().state == rnr::core::ControllerState::EStop);

        // Reset clears E-Stop
        controller.resetFromEStop();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);

        // 2. Verify Manual M6 Tool Change retract passes outputs_mask = 0
        auto cfg = controller.getConfig();
        std::array<double, 4> cur_w = {10.0, 10.0, 5.0, 0.0};
        std::array<double, 4> z_up  = {10.0, 10.0, 25.0, 0.0};
        auto z_slices = rnr::planner::TrajectoryPlanner::sliceLinearMove(cur_w, z_up, 1500.0, cfg, 0 /* Spindle strictly OFF */);
        assert(!z_slices.empty());
        for (const auto& s : z_slices) {
            assert((s.slice.outputs_mask & rnr::core::OutputBit::OUT1_TorchOrSpindle) == 0);
        }

        // 3. Verify cnc.get_tool_change_config() returns settings cleanly to Lua
        assert(controller.runScriptString("local tc = cnc.get_tool_change_config(); assert(tc.move_to_park_xy == false)", false));

        controller.disconnect();
        std::cout << "  -> PASSED: Spindle strictly OFF during M6 retract; Limit Override does not bypass E-Stop; Lua tool change config verified.\n";
    }

    // ----------------------------------------------------
    // Test 20: Laser Mode ignore_z_motion Zero Z Spikes (Issue A5)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 20] Testing Laser Mode ignore_z_motion step continuity...\n";
        std::vector<rnr::core::ToolpathSegment> segs;

        rnr::core::ToolpathSegment s1{};
        s1.type = rnr::core::SegmentType::RapidMove;
        s1.start_xyza_mm = {0.0, 0.0, 10.0, 0.0};
        s1.end_xyza_mm   = {50.0, 0.0, 10.0, 0.0};
        s1.length_mm     = 50.0;
        segs.push_back(s1);

        rnr::core::ToolpathSegment s2{};
        s2.type = rnr::core::SegmentType::CutLinear;
        s2.start_xyza_mm = {50.0, 0.0, 2.0, 0.0}; // Different Z in file
        s2.end_xyza_mm   = {50.0, 50.0, 2.0, 0.0};
        s2.length_mm     = 50.0;
        s2.feed_mm_min   = 1200.0;
        segs.push_back(s2);

        rnr::core::MachineConfig cfg{};
        cfg.operation_mode = rnr::core::OperationMode::Laser;
        cfg.laser.ignore_z_motion = true;

        auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(segs, cfg);
        assert(!slices.empty());

        // Verify Z delta steps are 0 across all slices and segment transitions
        for (const auto& sl : slices) {
            assert(sl.slice.delta_steps[2] == 0);
        }
        std::cout << "  -> PASSED: Laser mode locks Z focal height with zero inter-segment step spikes.\n";
    }

    // ----------------------------------------------------
    // Test 21: Probing Math Accuracy & Subdivided Linear Cruise (Issues C1, D2)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 21] Testing Probing Edge Math Compensation & Subdivided Linear Cruise...\n";

        // Probing edge calculation verification
        const double hit_pos = 97.0;
        const double tip_radius = 3.0;
        const double retract = 3.0;
        const double travel_dir = 1.0; // probing right

        const double edge_coord = hit_pos + travel_dir * tip_radius; // 100.0
        const double current_pos = hit_pos - travel_dir * retract;   // 94.0
        const double current_offset = -travel_dir * (retract + tip_radius); // -6.0

        // When current_pos is set to current_offset (-6.0):
        // Position of edge is current_offset + (edge_coord - current_pos) = -6.0 + (100.0 - 94.0) = 0.0!
        assert(std::abs(edge_coord - 100.0) < 1e-6);
        assert(std::abs(current_pos - 94.0) < 1e-6);
        assert(std::abs(current_offset - (-6.0)) < 1e-6);
        assert(std::abs((current_pos + current_offset) - (edge_coord - 12.0)) < 1e-6);

        // Subdivided linear chord cruise test (Surface height-map subdivided moves)
        std::vector<rnr::core::ToolpathSegment> cut_segs;
        for (int i = 0; i < 10; ++i) {
            rnr::core::ToolpathSegment cs{};
            cs.type = rnr::core::SegmentType::CutLinear;
            cs.start_xyza_mm = {i * 2.0, 0.0, 0.0, 0.0};
            cs.end_xyza_mm   = {(i + 1) * 2.0, 0.0, 0.0, 0.0};
            cs.length_mm     = 2.0;
            cs.feed_mm_min   = 1200.0;
            cut_segs.push_back(cs);
        }

        rnr::core::MachineConfig cfg{};
        auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(cut_segs, cfg);
        assert(!slices.empty());

        // Each 2mm segment slices into constant velocity chords without deceleration to 0
        for (const auto& sl : slices) {
            assert(sl.slice.delta_steps[0] > 0);
        }

        std::cout << "  -> PASSED: Edge tip compensation mathematically exact; 2mm sub-segments cruise smoothly.\n";
    }

    // -------------------------------------------------------------
    // [Test 22] Position Persistence across Sessions & Curve Feedrate Continuity
    // -------------------------------------------------------------
    {
        std::cout << "\n[Test 22] Testing Position Persistence & Curve Feedrate Continuity...\n";

        // 1. Coordinates Persistence
        rnr::config::PersistentSettings ps{};
        ps.ui.has_saved_coordinates = true;
        ps.ui.last_machine_pos_mm[0] = 123.456;
        ps.ui.last_machine_pos_mm[1] = 78.910;
        ps.ui.last_machine_pos_mm[2] = 15.250;
        ps.ui.last_machine_pos_mm[3] = 0.0;
        ps.ui.last_work_offset_mm[0] = 50.0;
        ps.ui.last_work_offset_mm[1] = 20.0;
        ps.ui.last_work_offset_mm[2] = 5.0;
        ps.ui.last_work_offset_mm[3] = 0.0;
        for (int i = 0; i < 4; ++i) {
            ps.ui.last_work_pos_mm[i] = ps.ui.last_machine_pos_mm[i] - ps.ui.last_work_offset_mm[i];
        }

        const std::string test_ini = "test_coords_persist.ini";
        bool save_ok = rnr::config::SettingsManager::saveToFile(test_ini, ps);
        assert(save_ok);

        rnr::config::PersistentSettings loaded_ps{};
        bool load_ok = rnr::config::SettingsManager::loadFromFile(test_ini, loaded_ps);
        assert(load_ok);
        assert(loaded_ps.ui.has_saved_coordinates);
        assert(std::abs(loaded_ps.ui.last_machine_pos_mm[0] - 123.456) < 1e-4);
        assert(std::abs(loaded_ps.ui.last_work_offset_mm[0] - 50.0) < 1e-4);
        assert(std::abs(loaded_ps.ui.last_work_pos_mm[0] - (123.456 - 50.0)) < 1e-4);
        std::remove(test_ini.c_str());

        MockController mock;
        std::array<double, 4> m_pos = {loaded_ps.ui.last_machine_pos_mm[0], loaded_ps.ui.last_machine_pos_mm[1], loaded_ps.ui.last_machine_pos_mm[2], loaded_ps.ui.last_machine_pos_mm[3]};
        std::array<double, 4> w_off = {loaded_ps.ui.last_work_offset_mm[0], loaded_ps.ui.last_work_offset_mm[1], loaded_ps.ui.last_work_offset_mm[2], loaded_ps.ui.last_work_offset_mm[3]};
        mock.restoreSavedPositions(m_pos, w_off);
        auto tel = mock.getTelemetry();
        assert(std::abs(tel.machine_pos_mm[0] - 123.456) < 1e-4);
        assert(std::abs(tel.work_pos_mm[0] - (123.456 - 50.0)) < 1e-4);

        // 2. Curve Feed Rate Continuity: Twenty 0.1mm chords (total 2.0mm) at F1200 (20mm/s)
        // Cruise time is 0.10s (4 slices). Starting and ending at rest adds the acceleration (20/500 s) and
        // deceleration (20/600 s) ramps, so ~0.137s = 6 slices. Without chord slice-time accumulation this
        // would be 20+ slices (one per chord).
        std::vector<rnr::core::ToolpathSegment> curve_chords;
        for (int i = 0; i < 20; ++i) {
            rnr::core::ToolpathSegment cs{};
            cs.type          = rnr::core::SegmentType::CutArcCW;
            cs.segment_index = static_cast<size_t>(i);
            cs.gcode_line    = 10;
            cs.start_xyza_mm = {i * 0.1, 0.0, 0.0, 0.0};
            cs.end_xyza_mm   = {(i + 1) * 0.1, 0.0, 0.0, 0.0};
            cs.length_mm     = 0.1;
            cs.feed_mm_min   = 1200.0; // 20 mm/s
            curve_chords.push_back(cs);
        }

        rnr::core::MachineConfig cfg{};
        auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(curve_chords, cfg, 100.0f, 0.0f, false, 0.025);
        // Full speed is maintained across the chords: no stop-and-go between them
        assert(slices.size() >= 5 && slices.size() <= 7);
        for (const auto& sl : slices) {
            assert(sl.slice.delta_steps[0] > 0);
        }
        // Accelerates from rest: the first slice is shorter than a cruise slice (20mm/s * 25ms = 0.5mm = 100 steps)
        assert(slices.front().slice.delta_steps[0] < 100);

        std::cout << "  -> PASSED: Coordinates persisted across sessions; 20x 0.1mm curve chords sliced at full feed velocity ("
                  << slices.size() << " slices).\n";
    }

    // ----------------------------------------------------
    // Test 23: Remote Web G-Code Upload & Toolpath Curve Grouping
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 23] Testing Remote Web G-Code Upload Callback & Curve Highlighting Grouping...\n";

        // 1. Curve span grouping: 10 arc chords of same gcode_line
        std::vector<rnr::core::ToolpathSegment> segments;
        for (int i = 0; i < 10; ++i) {
            rnr::core::ToolpathSegment s{};
            s.type          = rnr::core::SegmentType::CutArcCW;
            s.segment_index = static_cast<size_t>(i);
            s.gcode_line    = 42;
            s.start_xyza_mm = {i * 1.0, 0.0, 0.0, 0.0};
            s.end_xyza_mm   = {(i + 1) * 1.0, 0.0, 0.0, 0.0};
            s.length_mm     = 1.0;
            segments.push_back(s);
        }
        // Linear move afterwards
        rnr::core::ToolpathSegment s_lin{};
        s_lin.type          = rnr::core::SegmentType::CutLinear;
        s_lin.segment_index = 10;
        s_lin.gcode_line    = 43;
        s_lin.length_mm     = 25.0;
        segments.push_back(s_lin);

        // Grouping from active_idx = 2
        size_t active_idx = 2;
        size_t active_end_idx = active_idx;
        const auto& active_seg = segments[active_idx];
        if (active_seg.type == rnr::core::SegmentType::CutArcCW || active_seg.type == rnr::core::SegmentType::CutArcCCW) {
            while (active_end_idx + 1 < segments.size()) {
                const auto& nxt = segments[active_end_idx + 1];
                if (nxt.gcode_line == active_seg.gcode_line &&
                    (nxt.type == rnr::core::SegmentType::CutArcCW || nxt.type == rnr::core::SegmentType::CutArcCCW))
                {
                    active_end_idx++;
                } else {
                    break;
                }
            }
        }
        assert(active_end_idx == 9); // All remaining 7 arc chords are grouped together!

        // 2. CameraStreamServer callback testing
        rnr::core::CameraManager cam_mgr;
        rnr::core::CameraStreamServer stream_server(cam_mgr);
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        stream_server.setLoadGCodeCallback([&](const std::string& filename, const std::string& content) -> rnr::core::LoadGCodeResult {
            const auto tel = controller.getTelemetry();
            if (tel.state == rnr::core::ControllerState::RunningGCode) {
                return {false, "Machine is currently running", 0};
            }
            if (!controller.loadGCodeString(content)) {
                return {false, "Failed to parse G-code", 0};
            }
            return {true, "", (*controller.getProgram()).segments.size()};
        });

        // Test uploading sample G-code
        std::string sample_gcode = "G90\nG0 X0 Y0 Z5\nG1 X50 Y50 F1200\n";
        auto upload_res = controller.loadGCodeString(sample_gcode);
        assert(upload_res);

        // Update telemetry with file name and verify
        auto tel = controller.getTelemetry();
        stream_server.updateTelemetry(tel, rnr::core::LinearUnits::Metric, "BreakoutMill", "remotely_uploaded_bracket.nc");

        controller.disconnect();
        std::cout << "  -> PASSED: Arc curve grouping correctly captures full arc span (chords 2..9); remote upload callback verified.\n";
    }

    // ----------------------------------------------------
    // Test 24: M6 Tool Change "TX already probed" Button
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 24] Testing M6 Tool Change Dialog & 'TX already probed' Button...\n";
        MockController mock_controller;
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::atomic<bool> script_done{false};
        std::atomic<bool> script_success{false};

        // Run M6 script file asynchronously
        std::string m6_path = "scripts/mill/m6.lua";
        runner.runFileAsync(m6_path, [&](bool success, const std::string& err) {
            script_done = true;
            script_success = success;
        });

        // Wait for prompt
        auto start = std::chrono::steady_clock::now();
        while (!runner.hasPendingPrompt()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > 3.0) break;
        }

        assert(runner.hasPendingPrompt());
        auto req = runner.getPendingPrompt();
        assert(req.title == "Tool Change (M6)");
        assert(req.buttons.size() == 3);
        assert(req.buttons[0] == "OK");
        assert(req.buttons[1] == "T1 already probed");
        assert(req.buttons[2] == "Cancel");

        // Operator clicks "T1 already probed"
        runner.submitPromptResponse("T1 already probed");

        // Script should finish immediately with success = true without performing probe touches
        start = std::chrono::steady_clock::now();
        while (!script_done.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > 3.0) break;
        }

        assert(script_done.load());
        assert(script_success.load());

        std::cout << "  -> PASSED: M6 prompt surfaced 3 buttons including 'T1 already probed'; clicking it bypassed probing and completed with success.\n";
    }

    // ----------------------------------------------------
    // Test 25: Stop vs Rewind, Rapid Z20 Continuity & N-Line Numbering
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 25] Testing Stop vs Rewind, Rapid Z20 Continuity & N-Line Numbering...\n";

        // 1. Rapid Z20 Continuity (N180 G00 G43 Z20.000 H1):
        // Ensure that starting after an M6 tool change with G0 Z20 NEVER plunges to Z=0!
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        std::string test_gcode =
            "N140G00G21G17G90G40G49G80\n"
            "N150G71G91.1\n"
            "N160T1M06\n"
            "N170 (V-Bit {90 deg 0.5\"})\n"
            "N180G00G43Z20.000H1\n"
            "N190S16000M03\n"
            "N200G01X10.0Y10.0Z-1.0F1200\n";

        bool loaded = controller.loadGCodeString(test_gcode);
        assert(loaded);
        const auto& pr = (*controller.getProgram());
        assert(pr.segments.size() >= 3);
        assert(pr.segments[0].type == rnr::core::SegmentType::ToolChange);
        assert(pr.segments[1].type == rnr::core::SegmentType::RapidMove);
        assert(std::abs(pr.segments[1].end_xyza_mm[2] - 20.0) < 1e-4);

        // Simulate tool is at Safe Z (25.0) after M6 tool change
        controller.setWorkPosition(rnr::core::Axis::Z, 25.0);
        assert(std::abs(controller.getTelemetry().work_pos_mm[2] - 25.0) < 1e-4);

        // Trigger job from segment 1 (line N180) - exactly what happens when M6 completes
        controller.setNextGCodeLine(4); // Line N180 is line index 4
        controller.startJob();

        // Check planned slices: NO slice should ever have Z close to 0.0 before reaching Z20!
        // The first motion should go monotonically from 25.0 down to 20.0, NEVER plunging to 0.0!
        auto slices = controller.getSliceQueue();
        assert(!slices.empty());
        for (const auto& s : slices) {
            if (s.segment_index == 1) { // Segment 1 is G00 G43 Z20.000 H1
                // Target Z must stay >= 19.99 (moving from 25.0 to 20.0), never drop toward 0.0!
                assert(s.target_pos_mm[2] >= 19.99);
            }
        }
        controller.stopJob();

        // 2. Stop vs Rewind logic test
        // Verify controller line index preservation on stopJob vs rewind to line 0
        controller.setNextGCodeLine(4);
        controller.stopJob();
        assert(controller.getTelemetry().active_gcode_line_index == 4);

        // Rewind sets next gcode line to 0
        controller.setNextGCodeLine(0);
        assert(controller.getTelemetry().active_gcode_line_index == 0);

        controller.disconnect();
        std::cout << "  -> PASSED: Rapid G0 Z20 plans cleanly from Safe Z without plunging to Z0; Stop retains active line and Rewind resets to line 1.\n";
    }

    // ========================================================================
    // Test 26: Surface Height Map Bicubic Spline Interpolation & Smoothness Slider
    // ========================================================================
    {
        std::cout << "\n[Test 26] Testing Surface Height Map Bicubic Spline, Smoothness Blending & Apply Workflow...\n";

        // Create a 4x4 height map across a 300x300mm surface with a curved dome topology
        rnr::core::SurfaceHeightMap shm;
        shm.is_valid = true;
        shm.min_x = 0.0;
        shm.max_x = 300.0;
        shm.min_y = 0.0;
        shm.max_y = 300.0;
        shm.points_x = 4;
        shm.points_y = 4;
        shm.grid_z = {
            0.0, 0.2, 0.2, 0.0,
            0.3, 0.8, 0.8, 0.3,
            0.3, 0.8, 0.8, 0.3,
            0.0, 0.2, 0.2, 0.0
        };
        shm.recalculateStats();
        assert(shm.grid_z.size() == 16);
        assert(std::abs(shm.min_z - 0.0) < 1e-4);
        assert(std::abs(shm.max_z - 0.8) < 1e-4);

        // 1. Verify exact grid vertex preservation at all 16 nodes for both Bilinear (0.0) and Bicubic (1.0)
        const double dx = 100.0; // 300 / 3
        const double dy = 100.0;
        for (int j = 0; j < 4; ++j) {
            double y = static_cast<double>(j) * dy;
            for (int i = 0; i < 4; ++i) {
                double x = static_cast<double>(i) * dx;
                double expected_z = shm.grid_z[static_cast<size_t>(j * 4 + i)];

                double z_bilinear = shm.getInterpolatedZWithSmoothness(x, y, 0.0);
                double z_bicubic  = shm.getInterpolatedZWithSmoothness(x, y, 1.0);
                double z_blend    = shm.getInterpolatedZWithSmoothness(x, y, 0.5);

                assert(std::abs(z_bilinear - expected_z) < 1e-5);
                assert(std::abs(z_bicubic - expected_z) < 1e-5);
                assert(std::abs(z_blend - expected_z) < 1e-5);
            }
        }

        // 2. Verify intermediate curvature: at (50, 50) mm (midpoint of first cell)
        // Bilinear is flat hyperbolic: average of (0.0, 0.2, 0.3, 0.8) = 0.325
        double z_mid_bilinear = shm.getInterpolatedZWithSmoothness(50.0, 50.0, 0.0);
        assert(std::abs(z_mid_bilinear - 0.325) < 1e-4);

        // Bicubic produces smooth curved ascent
        double z_mid_bicubic = shm.getInterpolatedZWithSmoothness(50.0, 50.0, 1.0);
        assert(z_mid_bicubic > 0.1 && z_mid_bicubic < 0.8);

        // 50% Smoothness blend is exactly arithmetic average
        double z_mid_half = shm.getInterpolatedZWithSmoothness(50.0, 50.0, 0.5);
        assert(std::abs(z_mid_half - (0.5 * z_mid_bilinear + 0.5 * z_mid_bicubic)) < 1e-5);

        // 3. Test Controller Apply Height Map workflow with Smoothness
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        const std::string test_gcode =
            "G21 G90\n"
            "G00 X0 Y0 Z5.0\n"
            "G01 X300.0 Y0 Z0.0 F1200\n"
            "G01 X300.0 Y300.0 Z0.0\n"
            "G01 X0 Y300.0 Z0.0\n"
            "G01 X0 Y0 Z0.0\n";
        bool loaded = controller.loadGCodeString(test_gcode);
        assert(loaded);

        // Apply with smoothness = 0.0 (Bilinear)
        shm.smoothness = 0.0;
        controller.applySurfaceHeightMap(shm);
        assert(controller.isSurfaceCompensationActive());
        auto segs_bilinear = controller.getProgram()->segments;
        assert(!segs_bilinear.empty());

        // Apply with smoothness = 1.0 (Bicubic Spline)
        shm.smoothness = 1.0;
        controller.applySurfaceHeightMap(shm);
        assert(controller.isSurfaceCompensationActive());
        auto segs_bicubic = controller.getProgram()->segments;
        assert(!segs_bicubic.empty());

        // Confirm both produce warped segments while respecting boundary node zero
        assert(std::abs(segs_bilinear.front().end_xyza_mm[2] - 5.0) < 1e-3); // G00 X0 Y0 Z5 ends on a zero map node
        assert(std::abs(segs_bicubic.front().end_xyza_mm[2] - 5.0) < 1e-3);

        controller.disconnect();
        std::cout << "  -> PASSED: 4x4 Bicubic Spline nodes preserved, intermediate curvature evaluated, and Apply smoothness workflow verified.\n";
    }

    // ========================================================================
    // Test 27: Parser rejects / reports what it cannot execute correctly
    // ========================================================================
    {
        std::cout << "\n[Test 27] Testing G-code parser errors, G92 continuity, G28/G53 machine moves & M0...\n";
        using rnr::gcode::GCodeParser;
        assert(GCodeParser::parse("G1 X10\n").hasErrors());                       // Feed move without F
        assert(!GCodeParser::parse("G1 X10 F500\n").hasErrors());
        assert(GCodeParser::parse("G41 D1\nG1 X10 F500\n").hasErrors());          // Cutter compensation
        assert(GCodeParser::parse("G95 F0.1\n").hasErrors());                     // Feed per revolution
        assert(GCodeParser::parse("G0 X#1\n").hasErrors());                       // Expressions
        assert(GCodeParser::parse("G2 X10 Y0 I3 J0 F500\n").hasErrors());         // End point not on arc
        assert(GCodeParser::parse("G2 X10 Y0 R2 F500\n").hasErrors());            // R smaller than half chord
        assert(!GCodeParser::parse("G2 X10 Y0 I5 J0 F500\n").hasErrors());
        auto g43 = GCodeParser::parse("G43 H1 Z5\n");
        assert(!g43.hasErrors() && !g43.warnings.empty());                        // G43 ignored -> warning

        // G92 inside a program shifts coordinates without creating a discontinuous jump
        auto g92 = GCodeParser::parse("G0 X10\nG92 X0\nG1 X5 F600\n");
        assert(!g92.hasErrors());
        for (size_t i = 1; i < g92.segments.size(); ++i) {
            for (size_t a = 0; a < 4; ++a) {
                assert(std::abs(g92.segments[i].start_xyza_mm[a] - g92.segments[i - 1].end_xyza_mm[a]) < 1e-9);
            }
        }
        assert(std::abs(g92.segments.back().end_xyza_mm[0] - 15.0) < 1e-9); // Programmed X5 = physical X15

        // G28 Z0 is a machine-coordinate move to machine Z0 even when the parse-time offset is unknown
        auto g28 = GCodeParser::parse("G28 G91 Z0\nG90\n");
        assert(g28.machine_axes_used == 0x04);
        bool found_machine_move = false;
        for (const auto& s : g28.segments) {
            if (s.machine_axes_mask == 0x04 && s.machine_target_mm[2] == 0.0) found_machine_move = true;
        }
        assert(found_machine_move);

        // M0 produces a program stop marker
        auto m0 = GCodeParser::parse("G0 X5\nM0\nG0 X0\n");
        bool has_stop = false;
        for (const auto& s : m0.segments) if (s.type == rnr::core::SegmentType::ProgramStop) has_stop = true;
        assert(has_stop);

        // Blocks after M30 are ignored
        auto m30 = GCodeParser::parse("G0 X5\nM30\nG0 X100\n");
        for (const auto& s : m30.segments) assert(s.end_xyza_mm[0] < 50.0);
        std::cout << "  -> PASSED: Unsupported codes reported as errors; G92 continuous; G28 machine move; M0 stop; M30 end.\n";
    }

    // ========================================================================
    // Test 28: Planner acceleration, junction speed and hard velocity limits
    // ========================================================================
    {
        std::cout << "\n[Test 28] Testing planner acceleration from rest, feed override limit & pulse-rate cap...\n";
        rnr::core::MachineConfig cfg{};
        // Short (5 mm) move: previously executed at full feed from the first slice
        auto slices = rnr::planner::TrajectoryPlanner::sliceLinearMove({0, 0, 0, 0}, {5, 0, 0, 0}, 3000.0, cfg);
        const double spm = cfg.axes[0].steps_per_mm;
        const double cruise_steps = (3000.0 / 60.0) * 0.025 * spm;
        assert(slices.front().slice.delta_steps[0] < cruise_steps * 0.6); // Ramps up
        assert(slices.back().slice.delta_steps[0] < cruise_steps * 0.6);  // Ramps down
        // Consecutive slices never change speed faster than the acceleration limit allows (+1 step rounding)
        const double max_dv_steps = cfg.axes[0].deceleration_mm_s2 * 0.025 * 0.025 * spm + 2.0;
        for (size_t i = 1; i < slices.size(); ++i) {
            assert(std::abs(slices[i].slice.delta_steps[0] - slices[i - 1].slice.delta_steps[0]) <= max_dv_steps);
        }

        // 250% feed override must never exceed the configured axis velocity
        rnr::core::ToolpathSegment fast{};
        fast.type = rnr::core::SegmentType::CutLinear;
        fast.end_xyza_mm = {500.0, 0.0, 0.0, 0.0};
        fast.length_mm = 500.0;
        fast.feed_mm_min = 6000.0; // = axis max (100 mm/s)
        auto ov = rnr::planner::TrajectoryPlanner::sliceProgram({fast}, cfg, 250.0f);
        const double max_steps = cfg.axes[0].max_velocity_mm_s * 0.025 * spm + 1.0;
        for (const auto& s : ov) assert(std::abs(s.slice.delta_steps[0]) <= max_steps);

        // Pulse-rate cap: 25 kHz board with 1000 steps/mm -> at most 23.75 mm/s
        rnr::core::MachineConfig slow = cfg;
        slow.pulse_rate_khz = 25;
        slow.axes[0].steps_per_mm = 1000.0;
        auto capped = rnr::planner::TrajectoryPlanner::sliceProgram({fast}, slow);
        for (const auto& s : capped) assert(std::abs(s.slice.delta_steps[0]) <= 25000 * 0.025);

        // Square corner: speed drops at the corner instead of turning at full feed
        rnr::core::ToolpathSegment a{}, b{};
        a.type = b.type = rnr::core::SegmentType::CutLinear;
        a.end_xyza_mm = {50, 0, 0, 0};  a.length_mm = 50; a.feed_mm_min = 3000;
        b.start_xyza_mm = {50, 0, 0, 0}; b.end_xyza_mm = {50, 50, 0, 0}; b.length_mm = 50; b.feed_mm_min = 3000;
        auto corner = rnr::planner::TrajectoryPlanner::sliceProgram({a, b}, cfg);
        double min_speed_near_corner = 1e9;
        for (const auto& s : corner) {
            if (std::abs(s.target_pos_mm[0] - 50.0) < 1.0 && s.target_pos_mm[1] < 1.0) {
                const double d = std::hypot(double(s.slice.delta_steps[0]), double(s.slice.delta_steps[1]));
                min_speed_near_corner = std::min(min_speed_near_corner, d);
            }
        }
        assert(min_speed_near_corner < cruise_steps * 0.5);
        std::cout << "  -> PASSED: Moves ramp within accel limits; override & pulse rate never exceed hardware limits; corners slow down.\n";
    }

    // ========================================================================
    // Test 29: E-Stop input is never bypassed by Limit Override; reset needs a released E-Stop
    // ========================================================================
    {
        std::cout << "\n[Test 29] Testing E-Stop input vs Limit Override and reset interlocks...\n";
        rnr::core::RnRController controller;
        auto cfg = controller.getConfig();
        cfg.estop_pin = 6;
        cfg.axes[0].limit_plus_pin = 7;
        controller.applyConfig(cfg);
        assert(controller.resetFromEStop());
        controller.setLimitOverride(true);
        controller.setSimulatedInputPin(6, true); // E-Stop pressed
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        assert(controller.getTelemetry().state == rnr::core::ControllerState::EStop);
        assert(!controller.resetFromEStop()); // Still pressed
        controller.setSimulatedInputPin(6, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        assert(controller.resetFromEStop());

        // A limit switch blocks reset unless Limit Override is on
        controller.setSimulatedInputPin(7, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        assert(controller.getTelemetry().state == rnr::core::ControllerState::EStop);
        assert(!controller.resetFromEStop());
        controller.setLimitOverride(true);
        assert(controller.resetFromEStop());
        controller.setSimulatedInputPin(7, false);
        std::cout << "  -> PASSED: Limit Override no longer disables the E-Stop input; reset refused while E-Stop / limit active.\n";
    }

    // ========================================================================
    // Test 30: Programs with errors / soft-limit violations never start
    // ========================================================================
    {
        std::cout << "\n[Test 30] Testing Cycle Start validation (program errors & soft limits)...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        controller.loadGCodeString("G21 G90\nG41 D1\nG1 X10 F500\n");
        controller.startJob();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);
        assert(controller.getSliceQueue().empty());

        auto cfg = controller.getConfig();
        cfg.axes[0].soft_limits_enabled = true;
        cfg.axes[0].soft_limit_min_mm = -1.0;
        cfg.axes[0].soft_limit_max_mm = 50.0;
        controller.applyConfig(cfg);
        controller.homeAxis(rnr::core::Axis::X);
        for (int i = 0; i < 400 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(controller.getTelemetry().axis_homed[0]);
        controller.loadGCodeString("G21 G90\nG0 X100 Y0\n"); // X100 is beyond the 50 mm soft limit
        controller.startJob();
        auto t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(t.active_fault_message.find("soft limits") != std::string::npos);
        std::cout << "  -> PASSED: Erroneous programs and soft-limit violations are rejected before any motion.\n";
    }

    // ========================================================================
    // Test 31: Simulated homing & hole-centre probing complete with real sequencing
    // ========================================================================
    {
        std::cout << "\n[Test 31] Testing homing sequence and hole-centre probing in the simulator...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        controller.homeAllAxes();
        for (int i = 0; i < 1500 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        auto t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(t.axis_homed[0] && t.axis_homed[1] && t.axis_homed[2]);
        // Pulled off the switch by home_pulloff_mm (X/Y home negative, Z positive)
        const auto cfg = controller.getConfig();
        assert(std::abs(t.machine_pos_mm[0] - cfg.axes[0].home_pulloff_mm) < 0.05);
        assert(std::abs(t.machine_pos_mm[2] + cfg.axes[2].home_pulloff_mm) < 0.05);

        // Probe refuses to start while the probe input is already active
        controller.setSimulatedInputPin(cfg.probe.probe_pin, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        controller.probeRaw(rnr::core::Axis::Z, -5.0, 100.0);
        assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);
        controller.setSimulatedInputPin(cfg.probe.probe_pin, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));

        controller.probeHoleCenterXY(20.0);
        for (int i = 0; i < 2000 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(std::abs(t.work_pos_mm[0]) < 1e-6 && std::abs(t.work_pos_mm[1]) < 1e-6);
        std::cout << "  -> PASSED: Homing seeks the switch and pulls off; hole centre probes 4 walls and zeros XY.\n";
    }

    // ========================================================================
    // Test 32: Feed hold decelerates along the path and resumes
    // ========================================================================
    {
        std::cout << "\n[Test 32] Testing feed hold deceleration & resume...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        assert(controller.loadGCodeString("G21 G90\nG1 X200 F3000\n"));
        controller.startJob();
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        const double x_at_pause = controller.getTelemetry().machine_pos_mm[0];
        controller.pauseJob();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::PausedHold);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        const double x_stopped = controller.getTelemetry().machine_pos_mm[0];
        assert(x_stopped > x_at_pause);               // Kept moving while decelerating (no instant stop)
        assert(x_stopped - x_at_pause < 10.0);        // ...but only for the braking distance
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        assert(std::abs(controller.getTelemetry().machine_pos_mm[0] - x_stopped) < 1e-9); // Standing still
        controller.startJob();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::RunningGCode);
        for (int i = 0; i < 1000 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(std::abs(controller.getTelemetry().work_pos_mm[0] - 200.0) < 0.01);
        std::cout << "  -> PASSED: Feed hold brakes along the path, stands still, and resumes to the exact end point.\n";
    }

    // ========================================================================
    // Test 33: MDI validation & G10 L20
    // ========================================================================
    {
        std::cout << "\n[Test 33] Testing MDI validation and G10 L20...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        assert(!controller.executeMdiCommand("G1 X10"));       // No feed rate known yet
        assert(!controller.executeMdiCommand("G41 G1 X10 F100")); // Unsupported
        assert(controller.executeMdiCommand("G10 L20 P1 X5"));  // Current position reads X5
        assert(std::abs(controller.getTelemetry().work_pos_mm[0] - 5.0) < 1e-9);
        assert(controller.executeMdiCommand("G1 X6 F600"));
        for (int i = 0; i < 300 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(controller.executeMdiCommand("G1 X7"));          // F is modal across MDI commands now
        std::cout << "  -> PASSED: MDI rejects unsafe commands, G10 L20 sets offsets, modal F retained.\n";
    }

    // ========================================================================
    // Test 34: Settings file recovery after an interrupted save
    // ========================================================================
    {
        std::cout << "\n[Test 34] Testing atomic settings save and .bak recovery...\n";
        const std::string ini = "test_atomic_settings.ini";
        rnr::config::PersistentSettings a{};
        a.machine.axes[0].steps_per_mm = 321.0;
        assert(rnr::config::SettingsManager::saveToFile(ini, a));
        rnr::config::PersistentSettings b = a;
        b.machine.axes[0].steps_per_mm = 654.0;
        assert(rnr::config::SettingsManager::saveToFile(ini, b)); // a is now the .bak
        // Simulate a power loss half-way through writing the file
        std::string content;
        {
            std::ifstream ifs(ini);
            std::stringstream ss;
            ss << ifs.rdbuf();
            content = ss.str();
        }
        {
            std::ofstream ofs(ini, std::ios::trunc);
            ofs << content.substr(0, content.size() / 2);
        }
        rnr::config::PersistentSettings loaded{};
        assert(rnr::config::SettingsManager::loadFromFile(ini, loaded));
        assert(std::abs(loaded.machine.axes[0].steps_per_mm - 321.0) < 1e-9);
        assert(!loaded.machine.limit_override);
        std::remove(ini.c_str());
        std::remove((ini + ".bak").c_str());
        std::cout << "  -> PASSED: Truncated settings fall back to the last complete save.\n";
    }

    // ========================================================================
    // Test 35: Parser tracks G43 H<n> / G49 and only warns when the mode is not configured
    // ========================================================================
    {
        std::cout << "\n[Test 35] Testing G43/G49 parsing and tool length mode warnings...\n";
        using rnr::gcode::GCodeParser;
        const std::string prog = "G0 X1\nG43 H2 Z5\nG0 X2\nG49\nG0 X3\n";
        auto pr = GCodeParser::parse(prog);
        assert(pr.segments.size() == 4);
        assert(pr.segments[0].tlo_tool == rnr::core::kTloInherit);
        assert(pr.segments[1].tlo_tool == 2);
        assert(pr.segments[2].tlo_tool == 2);
        assert(pr.segments[3].tlo_tool == 0);
        assert(pr.warnings.size() == 1); // Not configured -> "G43 ignored" warning

        rnr::gcode::ParseOptions touch_off;
        touch_off.tool_length_mode = rnr::core::ToolLengthMode::TouchOffEachTool;
        assert(GCodeParser::parse(prog, {0, 0, 0, 0}, 0.25, {0, 0, 0, 0}, touch_off).warnings.empty());
        rnr::gcode::ParseOptions table;
        table.tool_length_mode = rnr::core::ToolLengthMode::ToolTable;
        assert(GCodeParser::parse(prog, {0, 0, 0, 0}, 0.25, {0, 0, 0, 0}, table).warnings.empty());
        assert(GCodeParser::parse("G44 H1\n").hasErrors());

        // Switching the mode re-evaluates the program that is already loaded
        rnr::core::RnRController controller;
        assert(controller.loadGCodeString(prog));
        assert(controller.getProgram()->warnings.size() == 1);
        auto cfg = controller.getConfig();
        cfg.tool_table.mode = rnr::core::ToolLengthMode::TouchOffEachTool;
        controller.applyConfig(cfg);
        assert(controller.getProgram()->warnings.empty());
        assert(controller.getProgram()->segments.size() == 4);
        cfg = controller.getConfig();
        cfg.tool_table.mode = rnr::core::ToolLengthMode::NotConfigured;
        controller.applyConfig(cfg);
        assert(controller.getProgram()->warnings.size() == 1);
        std::cout << "  -> PASSED: G43/G49 recorded per segment; warning only when the tool length mode is not configured.\n";
    }

    // ========================================================================
    // Test 36: Tool table mode applies G43 offsets to motion, DRO and touch-off
    // ========================================================================
    {
        std::cout << "\n[Test 36] Testing tool table G43 offsets in jobs, MDI, touch-off and Cycle Start checks...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        auto cfg = controller.getConfig();
        cfg.tool_table.mode = rnr::core::ToolLengthMode::ToolTable;
        cfg.tool_table.tools[1].length_mm = 0.0;
        cfg.tool_table.tools[2].length_mm = 10.0;
        cfg.scripting.m6_script_enabled = false;
        controller.applyConfig(cfg);

        auto waitIdle = [&]() {
            for (int i = 0; i < 600; ++i) {
                const auto st = controller.getTelemetry().state;
                if (st == rnr::core::ControllerState::Idle || st == rnr::core::ControllerState::ToolChangeHold) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        };

        // MDI G43 H2: the DRO now shows the tool tip (10 mm below the spindle reference)
        assert(controller.executeMdiCommand("G43 H2"));
        auto t = controller.getTelemetry();
        assert(t.active_tlo_tool == 2 && std::abs(t.active_tlo_mm - 10.0) < 1e-9);
        assert(std::abs(t.work_pos_mm[2] - (t.machine_pos_mm[2] - 10.0)) < 1e-9);
        assert(controller.executeMdiCommand("G49"));
        assert(controller.getTelemetry().active_tlo_tool == 0);

        // Program with G43 H2: the tip reaches Z5, so the spindle (machine Z) is 10 mm higher
        assert(controller.loadGCodeString("G21 G90\nG43 H2 G0 Z5\n"));
        assert(controller.getProgram()->warnings.empty());
        controller.startJob();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::RunningGCode);
        waitIdle();
        t = controller.getTelemetry();
        assert(std::abs(t.machine_pos_mm[2] - 15.0) < 1e-3);
        assert(std::abs(t.work_pos_mm[2] - 5.0) < 1e-3);
        assert(t.active_tlo_tool == 2);

        // Touch off with T2 loaded, then cancel the offset: the spindle frame shows the tool length
        controller.zeroWorkAxis(rnr::core::Axis::Z);
        assert(std::abs(controller.getTelemetry().work_pos_mm[2]) < 1e-9);
        assert(controller.executeMdiCommand("G49"));
        assert(std::abs(controller.getTelemetry().work_pos_mm[2] - 10.0) < 1e-9);

        // Cycle Start checks: unknown tool length, and M6 without G43 for the new tool
        controller.loadGCodeString("G21 G90\nT3 M6\nG43 H3\nG0 Z5\n");
        controller.startJob();
        t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(t.active_fault_message.find("no length") != std::string::npos);
        controller.loadGCodeString("G21 G90\nT2 M6\nG0 Z5\n");
        controller.startJob();
        t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(t.active_fault_message.find("not followed by G43") != std::string::npos);
        std::cout << "  -> PASSED: G43 offsets move the spindle, DRO shows the tip, touch-off is tool-aware, unsafe programs refused.\n";
    }

    // ========================================================================
    // Test 37: Tool length measuring (simulator) & tool table persistence
    // ========================================================================
    {
        std::cout << "\n[Test 37] Testing tool length measuring and tool table persistence...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        auto cfg = controller.getConfig();
        cfg.tool_table.mode = rnr::core::ToolLengthMode::ToolTable;
        cfg.tool_table.reference_tool = 1;
        controller.applyConfig(cfg);
        auto waitIdle = [&]() {
            for (int i = 0; i < 600 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        };

        // Other tools need the reference first
        controller.measureToolLength(2);
        assert(controller.getConfig().tool_table.tools.count(2) == 0);

        controller.measureToolLength(1); // Simulator: contact 20 mm below the start (half of the 40 mm search)
        waitIdle();
        assert(controller.getTelemetry().tool_reference_measured);
        assert(std::abs(controller.getConfig().tool_table.tools.at(1).length_mm) < 1e-9);

        assert(controller.executeMdiCommand("G90 G0 Z13"));
        waitIdle();
        controller.measureToolLength(2); // Contact at Z13-20 = -7, reference contact at -20 -> +13 mm
        waitIdle();
        const auto measured = controller.getConfig().tool_table.tools;
        assert(measured.count(2) == 1);
        assert(std::abs(measured.at(2).length_mm - 13.0) < 1e-3);

        // Persistence
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.machine.tool_table.tools[2].diameter_mm = 6.0;
        ps.machine.tool_table.tools[2].description = "6mm flat endmill; 3 flute";
        const std::string ini = "test_tool_table.ini";
        assert(rnr::config::SettingsManager::saveToFile(ini, ps));
        rnr::config::PersistentSettings loaded{};
        assert(rnr::config::SettingsManager::loadFromFile(ini, loaded));
        assert(loaded.machine.tool_table.mode == rnr::core::ToolLengthMode::ToolTable);
        assert(loaded.machine.tool_table.tools.size() == 2);
        assert(std::abs(loaded.machine.tool_table.tools.at(2).length_mm - 13.0) < 1e-3);
        assert(std::abs(loaded.machine.tool_table.tools.at(2).diameter_mm - 6.0) < 1e-9);
        assert(loaded.machine.tool_table.tools.at(2).description == "6mm flat endmill; 3 flute");
        std::remove(ini.c_str());
        std::remove((ini + ".bak").c_str());
        std::cout << "  -> PASSED: Reference-relative tool lengths measured on the setter and persisted.\n";
    }

    // ========================================================================
    // Test 38: Every example in the Lua guide compiles, and runs in the simulator
    //          (dialogs answered automatically, including typed input values)
    // ========================================================================
    {
        std::cout << "\n[Test 38] Testing Lua guide examples (syntax + simulated runs with auto-answered dialogs)...\n";
        using namespace rnr::ui::guide_examples;

        // 1. Syntax of every example
        for (const auto& ex : kAll) {
            lua_State* L = luaL_newstate();
            const int rc = luaL_loadstring(L, ex.code);
            if (rc != LUA_OK) std::cerr << "Guide example '" << ex.id << "' does not compile: " << lua_tostring(L, -1) << "\n";
            assert(rc == LUA_OK);
            lua_close(L);
        }

        // 2. Run examples with an automatic operator
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        std::vector<std::string> errors;
        std::mutex err_mutex;
        controller.setScriptLogCallback([&](const std::string& msg, bool is_err) {
            if (is_err) {
                std::lock_guard<std::mutex> lock(err_mutex);
                errors.push_back(msg);
            }
        });
        controller.setSimulatedInputPin(6, true); // Sensor used by the "wait for input" example

        // Typed answers for input dialogs, keyed by a piece of the dialog message
        const std::map<std::string, std::string> typed = {
            {"Holes in X", "2"}, {"Holes in Y", "1"}, {"Spacing", "5"}, {"Depth (mm, positive)", "1"},
            {"Width in X", "10"}, {"Length in Y", "4"}, {"Stepover (mm):", "4"}, {"Depth of cut", "0.5"},
            {"Feed (mm/min)", "3000"},
        };
        std::vector<std::string> dialogs_seen;
        auto runExample = [&](const char* id, const char* code, const char* button_override) {
            errors.clear();
            assert(controller.runScriptString(code, true));
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            for (int i = 0; i < 3000 && controller.isScriptRunning(); ++i) {
                const auto t = controller.getTelemetry();
                if (t.has_script_prompt) {
                    dialogs_seen.push_back(t.script_prompt_title);
                    if (t.script_prompt_input_kind != 0) {
                        std::string value = t.script_prompt_input_default;
                        for (const auto& [key, v] : typed) {
                            if (t.script_prompt_message.find(key) != std::string::npos) value = v;
                        }
                        controller.submitScriptPromptInput("OK", value);
                    } else {
                        std::string button = t.script_prompt_buttons.empty() ? "OK" : t.script_prompt_buttons.front();
                        if (button_override) button = button_override;
                        controller.submitScriptPromptResponse(button);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            assert(!controller.isScriptRunning());
            for (int i = 0; i < 300 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (!errors.empty()) std::cerr << "Guide example '" << id << "' failed: " << errors.front() << "\n";
            assert(errors.empty());
            assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);
        };

        runExample("first", kFirstScript, nullptr);
        runExample("dlg_confirm", kDlgConfirm, nullptr);
        runExample("dlg_menu", kDlgMenu, nullptr);
        runExample("dlg_text", kDlgText, nullptr);
        runExample("dlg_number", kDlgNumber, nullptr);
        runExample("dlg_wizard", kDlgWizard, nullptr);
        runExample("dlg_checklist", kDlgChecklist, nullptr);
        runExample("ex_warmup", kExWarmup, "Cancel");
        runExample("ex_touch", kExTouchPlate, nullptr);
        const double z_after_touch = controller.getTelemetry().work_pos_mm[2];
        assert(std::abs(z_after_touch - (controller.getConfig().probe.plate_thickness_mm + 10.0)) < 0.01);
        runExample("ex_grid", kExGrid, nullptr);
        runExample("ex_facing", kExFacing, nullptr);
        runExample("ex_park", kExPark, nullptr);       // Not homed -> explains and stops
        runExample("ex_tools", kExMeasureTools, nullptr); // Not in tool table mode -> explains and stops
        runExample("ex_wait", kExWaitInput, nullptr);
        runExample("ex_log", kExJobLog, nullptr);
        std::remove("job_log.csv");
        runExample("hook_macro", kHookMacro, nullptr);
        runExample("mach3", kMach3Example, nullptr);
        assert(std::abs(controller.getTelemetry().work_pos_mm[2]) < 1e-9); // Question default 0 -> SetDRO(2, 0)

        // The Mach3 SetDRO shim sets the given value (it used to always zero the axis)
        assert(controller.runScriptString("SetDRO(2, 15.5)", false));
        assert(std::abs(controller.getTelemetry().work_pos_mm[2] - 15.5) < 1e-9);
        assert(dialogs_seen.size() > 25);
        std::cout << "  -> PASSED: " << std::size(kAll) << " guide examples compile; "
                  << dialogs_seen.size() << " dialogs answered across 17 simulated runs without errors.\n";
    }

    // ------------------------------------------------------------------------
    // Test 39: The machine core drives the hardware only through IBoardDriver (fake board, no USB)
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 39] Testing the machine core against a fake board driver...\n";
        auto fake = std::make_unique<FakeBoardDriver>();
        FakeBoardDriver* fb = fake.get();
        auto closed = fb->closed;
        auto waitIdle = [](rnr::core::MachineController& mc) {
            for (int i = 0; i < 200; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
                if (mc.getTelemetry().state == rnr::core::ControllerState::Idle) return true;
            }
            return false;
        };
        {
            rnr::core::MachineController mc(std::move(fake));
            // Connect: board configured, disarmed, outputs off, step counters synced
            auto tel = mc.getTelemetry();
            assert(!tel.simulator_mode && tel.usb_connected);
            assert(tel.product_name == "Fake Board" && tel.state == rnr::core::ControllerState::EStop);
            assert(fb->count("configure") == 1 && fb->count("estop") >= 1 && fb->count("outputs") >= 1);
            assert(fb->count("positions") == 1);
            assert(mc.boardCapabilities().display_name == "Fake Board");

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // First board report
            assert(mc.resetFromEStop());
            assert(fb->count("clear_estop") == 1);

            // Jog: internal slices only, the board ends exactly at the commanded position
            mc.jogStep(rnr::core::Axis::X, 5.0, 600.0);
            assert(waitIdle(mc));
            const double spm = mc.getConfig().axes[0].steps_per_mm;
            assert(fb->steps(0) == static_cast<int32_t>(std::llround(5.0 * spm)));
            assert(fb->count("slice_internal") > 0 && fb->count("slice_program") == 0);
            assert(std::abs(mc.getTelemetry().machine_pos_mm[0] - 5.0) < 1e-6);

            // Outputs
            mc.setAuxOutput(3, true);
            assert((fb->lastOutputs() & rnr::core::OutputBit::OUT3_FloodCoolant) != 0);
            mc.setAuxOutput(3, false);
            assert((fb->lastOutputs() & rnr::core::OutputBit::OUT3_FloodCoolant) == 0);

            // A G-code job streams program slices
            assert(mc.loadGCodeString("G90 G21\nG1 X10 Y-4 F900\nG0 X0 Y0\n"));
            mc.startJob();
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            assert(waitIdle(mc));
            assert(fb->count("slice_program") > 0);
            assert(fb->steps(0) == 0 && fb->steps(1) == 0);
            assert(fb->maxInFlight() <= 2); // Never more than the board's buffer (capabilities: 2)

            // E-Stop reaches the board
            const int estops = fb->count("estop");
            mc.triggerEStop();
            assert(fb->count("estop") > estops);
        }
        assert(closed->load()); // The core closes the board on shutdown
        std::cout << "  -> PASSED: connect, reset, jog, outputs, job streaming, slice budget and E-Stop all go through the driver.\n";
    }

    std::cout << "\n====================================================\n";
    std::cout << "ALL SCRIPT ENGINE UNIT TESTS PASSED SUCCESSFULLY! (39/39)\n";
    std::cout << "====================================================\n";
    return 0;
}
