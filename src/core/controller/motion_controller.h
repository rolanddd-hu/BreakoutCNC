#pragma once
#include "core/board/board_types.h"
#include "core/board/board_capabilities.h"
#include "core/board/board_profile.h"
#include "core/board/board_registry.h"
#include "core/board/grbl/grbl_settings.h"
#include "core/transport/protocol_log.h"
#include <array>
#include <type_traits>
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <memory>
#include <map>
#include "core/spindle/huanyang_vfd.h"

#include "core/controller/machine_config.h"
#include "core/controller/surface_height_map.h"
#include "core/controller/telemetry.h"
#include "core/controller/toolpath_segment.h"

namespace rnr::gcode {
struct ParseResult;
}

namespace rnr::core {
struct ProgramCheckResult; // core/controller/program_check.h

// Board report (GRBL-family boards): what the board says about itself, for diagnosis
struct BoardReport {
    uint64_t    revision = 0;   // Increases when a report is finished
    bool        running = false;
    std::string text;
};

// ============================================================================
// Abstract Motion Controller Interface (Implemented by RnRController)
// ============================================================================
class IMotionController {
public:
    virtual ~IMotionController() = default;

    // Connection, Simulator & Configuration
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual void setSimulatorMode(bool enabled) = 0;
    virtual void setDryRunMode(bool enabled) = 0;
    // Configuration changes are deferred until motion stops if the machine is moving.
    virtual void applyConfig(const MachineConfig& cfg) = 0;
    // Returns a consistent copy (thread-safe)
    virtual MachineConfig getConfig() const = 0;
    // The configuration as a shared, read-only snapshot: the UI reads it many times per frame. No copy is made unless
    // the configuration changed since the last call. Keep the pointer while using a reference into it.
    virtual std::shared_ptr<const MachineConfig> getConfigShared() const {
        return std::make_shared<const MachineConfig>(getConfig());
    }
    virtual TelemetrySnapshot getTelemetry() const = 0;

    // E-Stop, End-Stop Override & Simulated Inputs
    virtual void triggerEStop() = 0;
    virtual bool resetFromEStop() = 0;
    virtual void setLimitOverride(bool enabled) = 0;
    virtual void setSimulatedInputPin(uint8_t pin_1_to_12, bool active) = 0;

    // Coordinate Zeroing (Work Offsets & Machine Zero)
    virtual void zeroWorkAxis(core::Axis axis) = 0;
    virtual void zeroAllWorkAxes() = 0;
    virtual void goToWorkZeroXY() = 0;

    // Work Coordinate Systems (G54 - G59)
    virtual void selectWorkspace(uint8_t index) = 0;
    virtual void setWorkspaceOffset(uint8_t index, const std::array<double, 4>& offset_mm) = 0;
    virtual void setWorkspaceOffsetAxis(uint8_t index, core::Axis axis, double offset_mm) = 0;
    virtual uint8_t getActiveWorkspace() const = 0;
    virtual std::array<std::array<double, 4>, 6> getWorkspaceOffsets() const = 0;

    // Manual Jogging (Continuous & Incremental Step)
    virtual void jogStartContinuous(core::Axis axis, bool positive_dir, double speed_ratio) = 0;
    virtual void jogStep(core::Axis axis, double delta_mm, double feed_mm_min) = 0;
    virtual void jogStopAxis(core::Axis axis) = 0;
    virtual void jogStop() = 0;

    // Plasma Torch / Mill Spindle & Aux Outputs
    virtual void setTorchOrSpindle(bool enabled) = 0; // Toggles OUT1 immediately (Opcode 0x03)
    virtual void setAuxOutput(uint8_t out_index_1_to_4, bool enabled) = 0; // Output channel 1..4 (logical)
    // Physical output OUT1..OUT4: switches the channel assigned to it, or the custom output
    virtual void setPhysicalOutput(uint8_t out_1_to_4, bool enabled) { setAuxOutput(out_1_to_4, enabled); }
    virtual void setSpindlePwm(uint16_t duty) = 0;    // Mill 0-10V AVI (Opcode 0x0E)
    virtual void setSpindleSpeedRpm(uint32_t rpm) = 0; // Commanded spindle speed (updates PWM + Huanyang VFD)
    virtual void resetSpindleSpeedOverride() = 0;      // Restores G-Code programmatic RPM

    // Laser & Operation Mode Control
    virtual void setOperationMode(OperationMode mode) = 0;
    virtual OperationMode getOperationMode() const = 0;
    virtual void setLaserPowerOverride(float percent) = 0;
    virtual void testFireLaserPulse(float duration_ms, float power_pct) = 0;
    virtual void frameBoundingBox() = 0;
    virtual void setAirAssist(bool enabled) = 0;
    virtual void setRedPointer(bool enabled) = 0;

    // Huanyang VFD RS485 Inverter
    virtual void setHuanyangVfdConfig(const HuanyangVfdConfig& config) = 0;
    virtual HuanyangVfdConfig getHuanyangVfdConfig() const = 0;
    virtual HuanyangVfdTelemetry getHuanyangVfdTelemetry() const = 0;
    virtual void clearVfdAlarms() = 0;
    // Reads PD001/PD002/PD005/PD011/PD144 from the inverter; results arrive in the VFD telemetry params
    virtual void requestVfdParameterRead() = 0;

    // Homing & Probing (G31 Floating Head / Touch-Off / Edge & Center Finding)
    // homeAxis: the built-in cycle for one axis. homeAllAxes: the home script when it is enabled, else the built-in
    // cycle for all axes. The *BuiltIn variants never run the script (cnc.home() inside the script uses them).
    virtual void homeAxis(core::Axis axis) = 0;
    virtual void homeAllAxes() = 0;
    virtual void homeAxisBuiltIn(core::Axis axis) { homeAxis(axis); }
    virtual void homeAllAxesBuiltIn() { homeAllAxes(); }
    virtual void probeAxis(core::Axis axis, double max_travel_mm, double feed_mm_min) = 0;
    virtual void probeZWithToolSetter() = 0;
    virtual void probeWorkAxisEdge(core::Axis axis, bool positive_dir) = 0;
    virtual void probeHoleCenterXY(double approx_diameter_mm) = 0;
    // G38.2-style raw probe: move `signed_travel_mm` along `axis` until contact, then stop.
    // Records last_probed_work_pos_mm + last_probe_hit. Does NOT change offsets and does NOT retract.
    virtual void probeRaw(core::Axis axis, double signed_travel_mm, double feed_mm_min) = 0;

    // Tool table (ToolLengthMode::ToolTable)
    // Measure tool `tool` on the tool setter below the current XY position. The reference tool is
    // measured first after startup / Z homing; other tools get their length relative to it.
    virtual void measureToolLength(uint16_t tool) = 0;
    // The operator put `tool` in the spindle: sets the current tool and (tool table mode) applies its
    // length offset like "M6 T<tool> G43 H<tool>". 0 = no tool / cancel offset.
    virtual void selectToolInSpindle(uint16_t tool) = 0;
    // Apply the length offset of `tool` (G43 H<tool>) without changing the current tool; 0 = G49.
    virtual void activateToolLengthOffset(uint16_t tool) = 0;
    // G92-style: set the current work coordinate of `axis` to `value_mm` (adjusts the work offset).
    virtual void setWorkPosition(core::Axis axis, double value_mm) = 0;

    // Coordinate Persistence: restore saved machine positions and work offsets across application sessions
    virtual void restoreSavedPositions(
        const std::array<double, 4>& machine_pos_mm,
        const std::array<double, 4>& work_offset_mm) = 0;

    // Surface Level Probing & Height Map Warping
    virtual void startSurfaceProbing(double min_x, double max_x, double min_y, double max_y, int points_x, int points_y, double smoothness = 0.5) = 0;
    virtual void cancelSurfaceProbing() = 0;
    virtual void applySurfaceHeightMap(const SurfaceHeightMap& map) = 0;
    virtual void clearSurfaceHeightMap() = 0;
    virtual bool isSurfaceCompensationActive() const = 0;
    virtual SurfaceHeightMap getSurfaceHeightMap() const = 0;
    virtual bool getProgramBounds(std::array<double, 3>& out_min, std::array<double, 3>& out_max) const = 0;

    // G-Code Program, MDI & Job Execution
    virtual bool loadGCodeString(const std::string& gcode_text) = 0;
    virtual bool executeMdiCommand(const std::string& gcode_line) = 0;
    virtual void setNextGCodeLine(size_t line_index) = 0;
    // Immutable snapshot of the loaded program (segments, lines, bounds, diagnostics); never null.
    // Hold the returned pointer for as long as the data is used - a reload never invalidates it.
    virtual std::shared_ptr<const gcode::ParseResult> getProgram() const = 0;
    // Every problem of the loaded program: parse errors and warnings, and the checks against the machine settings
    // and state (homing, tool table, soft limits with the current offsets) that Cycle Start makes. Recomputed only
    // when the program or one of these inputs changes; may be null (no check available).
    virtual std::shared_ptr<const ProgramCheckResult> getProgramCheck() const = 0;
    virtual void startJob() = 0;
    virtual void pauseJob() = 0;
    virtual void stopJob() = 0;
    virtual void setFeedOverride(float percent) = 0;
    virtual void setPierceDelayOverride(float seconds) = 0;

    // Scripting & Automation (Lua 5.4 Engine)
    virtual bool runScriptString(const std::string& code, bool async = true) = 0;
    virtual bool runScriptFile(const std::string& filepath, bool async = true) = 0;
    virtual void cancelScript() = 0;
    virtual bool isScriptRunning() const = 0;
    virtual void submitScriptPromptResponse(const std::string& response) = 0;
    // Input dialogs (cnc.input / cnc.input_number): button pressed + entered text
    virtual void submitScriptPromptInput(const std::string& button, const std::string& input) {
        (void)input;
        submitScriptPromptResponse(button);
    }
    virtual void setScriptLogCallback(std::function<void(const std::string&, bool)> cb) = 0;

    // Board (MULTI_BOARD_PLAN.md): what the selected board can do, its console and its firmware settings
    virtual board::BoardCapabilities getBoardCapabilities() const { return board::rnrEcoMotionCapabilities(); }
    virtual bool unlockBoard() { return false; }                        // Firmware boards: clear an alarm ($X)
    virtual void setRapidOverride(float percent) { (void)percent; }     // 100 / 50 / 25 %
    virtual bool sendBoardConsoleLine(const std::string& line) { (void)line; return false; }
    virtual std::vector<transport::ProtocolLogEntry> getBoardLog(uint64_t after_seq) const {
        (void)after_seq;
        return {};
    }
    virtual bool requestBoardReport() { return false; } // Runs read-only queries ($I, $$, $#, $G ...) on the board
    virtual BoardReport getBoardReport() const { return {}; }
    virtual board::grbl::SettingsSnapshot getFirmwareSettings() const { return {}; }
    virtual void requestFirmwareSettings() {}
    virtual bool writeFirmwareSettings(const std::vector<std::pair<std::string, std::string>>& id_values) {
        (void)id_values;
        return false;
    }
};

} // namespace rnr::core
