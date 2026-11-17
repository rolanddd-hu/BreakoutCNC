#pragma once
#include "core/controller/motion_controller.h"
#include "core/board/board_driver.h"
#include "core/gcode/gcode_parser.h"
#include "core/planner/trajectory_planner.h"
#include "core/controller/program_check.h"
#include "core/motion/firmware_planned_path.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <deque>
#include <unordered_map>
#include <memory>
#include <functional>

namespace rnr::scripting {
class ScriptRunner;
}

namespace rnr::core {

class IMotionBackend;
class SliceBackend;
class FirmwareBackend;

// Board-independent machine core: jobs, planning, work offsets, tool table, homing & probing sequences,
// spindle / VFD, THC, laser rules and scripting. All hardware access goes through an IBoardDriver.
class MachineController : public IMotionController {
public:
    // `board` = the host-planned board driver (nullptr = RNR Eco Motion 2.0). Firmware-planned boards (GRBL family)
    // are selected with MachineConfig::board_id and driven through motion::FirmwarePlannedPath.
    // `connect_now` = false: the application connects itself once it has applied its settings.
    explicit MachineController(std::unique_ptr<board::IBoardDriver> board = nullptr, bool connect_now = true);
    ~MachineController() override;

    MachineController(const MachineController&) = delete;
    MachineController& operator=(const MachineController&) = delete;

    // Description of the selected board (refined by a firmware board once it is connected)
    board::BoardCapabilities boardCapabilities() const { return getBoardCapabilities(); }

    // IMotionController implementation
    bool connect() override;
    void disconnect() override;
    void setSimulatorMode(bool enabled) override;
    void setDryRunMode(bool enabled) override;
    void applyConfig(const MachineConfig& cfg) override;
    MachineConfig getConfig() const override;
    std::shared_ptr<const MachineConfig> getConfigShared() const override;
    TelemetrySnapshot getTelemetry() const override;

    void triggerEStop() override;
    bool resetFromEStop() override;
    void setLimitOverride(bool enabled) override;
    void setSimulatedInputPin(uint8_t pin_1_to_12, bool active) override;

    void zeroWorkAxis(core::Axis axis) override;
    void zeroAllWorkAxes() override;
    void goToWorkZeroXY() override;

    // Work Coordinate Systems (G54 - G59)
    void selectWorkspace(uint8_t index) override;
    void setWorkspaceOffset(uint8_t index, const std::array<double, 4>& offset_mm) override;
    void setWorkspaceOffsetAxis(uint8_t index, core::Axis axis, double offset_mm) override;
    uint8_t getActiveWorkspace() const override;
    std::array<std::array<double, 4>, 6> getWorkspaceOffsets() const override;

    // Continuous jog is a dead-man control: callers must repeat jogStartContinuous() at least every
    // kJogDeadmanMs while the jog input is held, otherwise the axis decelerates to a stop.
    void jogStartContinuous(core::Axis axis, bool positive_dir, double speed_ratio) override;
    void jogStep(core::Axis axis, double delta_mm, double feed_mm_min) override;
    void jogStopAxis(core::Axis axis) override;
    void jogStop() override;

    void setTorchOrSpindle(bool enabled) override;
    void setAuxOutput(uint8_t out_index_1_to_4, bool enabled) override;
    void setPhysicalOutput(uint8_t out_1_to_4, bool enabled) override;
    void setSpindlePwm(uint16_t duty) override;
    void setSpindleSpeedRpm(uint32_t rpm) override;
    void resetSpindleSpeedOverride() override;

    // Laser & Operation Mode Control
    void setOperationMode(OperationMode mode) override;
    OperationMode getOperationMode() const override;
    void setLaserPowerOverride(float percent) override;
    void testFireLaserPulse(float duration_ms, float power_pct) override;
    void frameBoundingBox() override;
    void setAirAssist(bool enabled) override;
    void setRedPointer(bool enabled) override;

    void setHuanyangVfdConfig(const HuanyangVfdConfig& config) override;
    HuanyangVfdConfig getHuanyangVfdConfig() const override;
    HuanyangVfdTelemetry getHuanyangVfdTelemetry() const override;
    void clearVfdAlarms() override;
    void requestVfdParameterRead() override;

    void homeAxis(core::Axis axis) override;
    void homeAllAxes() override;
    void homeAxisBuiltIn(core::Axis axis) override { homeAxis(axis); }
    void homeAllAxesBuiltIn() override;
    void probeAxis(core::Axis axis, double max_travel_mm, double feed_mm_min) override;
    void probeZWithToolSetter() override;
    void probeWorkAxisEdge(core::Axis axis, bool positive_dir) override;
    void probeHoleCenterXY(double approx_diameter_mm) override;
    void probeRaw(core::Axis axis, double signed_travel_mm, double feed_mm_min) override;
    void setWorkPosition(core::Axis axis, double value_mm) override;
    void measureToolLength(uint16_t tool) override;
    void selectToolInSpindle(uint16_t tool) override;
    void activateToolLengthOffset(uint16_t tool) override;
    void restoreSavedPositions(const std::array<double, 4>& machine_pos_mm, const std::array<double, 4>& work_offset_mm) override;

    // Surface Level Probing & Height Map Warping
    void startSurfaceProbing(double min_x, double max_x, double min_y, double max_y, int points_x, int points_y, double smoothness = 0.5) override;
    void cancelSurfaceProbing() override;
    void applySurfaceHeightMap(const SurfaceHeightMap& map) override;
    void clearSurfaceHeightMap() override;
    bool isSurfaceCompensationActive() const override;
    SurfaceHeightMap getSurfaceHeightMap() const override;
    bool getProgramBounds(std::array<double, 3>& out_min, std::array<double, 3>& out_max) const override;

    bool loadGCodeString(const std::string& gcode_text) override;
    bool executeMdiCommand(const std::string& gcode_line) override;
    void setNextGCodeLine(size_t line_index) override;
    std::shared_ptr<const gcode::ParseResult> getProgram() const override;
    std::shared_ptr<const ProgramCheckResult> getProgramCheck() const override;

    void startJob() override;
    void pauseJob() override;
    void stopJob() override;
    void setFeedOverride(float percent) override;
    void setPierceDelayOverride(float seconds) override;
    // Test accessor: copies the whole queue under the lock (never call it from the UI)
    std::deque<planner::PlannedSlice> getSliceQueue() const;

    // Scripting & Automation (Lua 5.4 Engine)
    bool runScriptString(const std::string& code, bool async = true) override;
    bool runScriptFile(const std::string& filepath, bool async = true) override;
    void cancelScript() override;
    bool isScriptRunning() const override;
    void submitScriptPromptResponse(const std::string& response) override;
    void submitScriptPromptInput(const std::string& button, const std::string& input) override;
    void setScriptLogCallback(std::function<void(const std::string&, bool)> cb) override;

    // Board selection, console and firmware settings
    board::BoardCapabilities getBoardCapabilities() const override;
    bool unlockBoard() override;
    void setRapidOverride(float percent) override;
    bool sendBoardConsoleLine(const std::string& line) override;
    std::vector<transport::ProtocolLogEntry> getBoardLog(uint64_t after_seq) const override;
    bool requestBoardReport() override;
    BoardReport getBoardReport() const override;
    board::grbl::SettingsSnapshot getFirmwareSettings() const override;
    void requestFirmwareSettings() override;
    bool writeFirmwareSettings(const std::vector<std::pair<std::string, std::string>>& id_values) override;

    static constexpr int kJogDeadmanMs = 500;

    // Process-wide switch: when disabled, connect() never opens a real board (unit tests must never
    // drive a real machine that happens to be plugged in). Call before constructing controllers.
    static void setHardwareAccessEnabled(bool enabled) noexcept { hardware_access_enabled_.store(enabled); }
    static bool hardwareAccessEnabled() noexcept { return hardware_access_enabled_.load(); }

private:
    // The motion of each kind of board (see motion_backend.h). They use the controller's state directly.
    friend class SliceBackend;
    friend class FirmwareBackend;

    using Clock = std::chrono::steady_clock;

    // The backend that moves the active board: a connected firmware board, else the slice path (RNR or simulator)
    IMotionBackend& motionLocked() noexcept;
    const IMotionBackend& motionLocked() const noexcept;
    // The backend that reads the attached board in the worker tick (a firmware board also while it connects)
    IMotionBackend& boardLinkLocked() noexcept;

    void workerLoop();

    // Firmware-planned boards (GRBL family)
    bool connectLocked();
    board::BoardCapabilities getBoardCapabilitiesLocked() const;
    void fillBoardLimitsLocked(MachineCheckInput::BoardLimits& b) const;
    bool firmwareBoardSelectedLocked() const;
    bool fwActiveLocked() const noexcept { return fw_ && fw_->isOpen() && !telemetry_.simulator_mode; }
    bool fwReadyLocked() const noexcept { return fwActiveLocked() && fw_->ready(); }
    // Outputs go to a board: the RNR board or a connected firmware board (not in the simulator)
    bool outputsLiveLocked() const noexcept { return !telemetry_.simulator_mode && (board_->isOpen() || fwActiveLocked()); }
    void closeFirmwarePathLocked();
    bool probeInputMissingLocked() const noexcept;
    // Safe approach moves before a job starts (shared by the slice planner and firmware boards)
    std::vector<ToolpathSegment> buildJobPrefixLocked(size_t start_from_segment, bool resume_in_place,
                                                      size_t& program_first, std::array<double, 4>& plan_start);
    void updateSpindleLocked(bool gcode_running);
    // Spindle speed: the range without a VFD is 0..kDefaultSpindleMaxRpm
    uint32_t spindleMaxRpmLocked() const noexcept;
    // The commanded speed changed: PWM duty (mill only; BreakoutCut's PWM is the laser power), the board's PWM output
    // (send_pwm) and the VFD follow
    void applyCommandedRpmLocked(uint32_t rpm, bool send_pwm);
    // A program / MDI S word: it becomes the commanded speed unless the operator overrides it. reset_override = the
    // S word cancels the operator's override (MDI, a newly loaded program)
    void setProgramSpindleRpmLocked(uint16_t rpm, bool reset_override, bool send_pwm);
    void updateContinuousJogLocked();

    void faultLocked(const std::string& message, ControllerState state = ControllerState::EStop);
    void stopJobLocked(bool is_estop = false);
    void pauseJobLocked();
    void applyConfigLocked(const MachineConfig& cfg);
    // Board output writes: translate the logical outputs mask to OUT1..OUT4 with the pin assignment (config_.io)
    void setBoardOutputsLocked(uint8_t logical_mask, bool limit_override);
    void setBoardOutputsAfterMotionLocked(uint8_t logical_mask, bool limit_override);
    void setMachinePositionLocked(const std::array<double, 4>& machine_pos_mm);
    void bumpWorkspaceRevisionLocked() noexcept { config_.workspace_revision++; }

    bool motionActiveLocked() const noexcept;
    // A job is running or paused (feed hold, M0). include_tool_change: also while it waits at an M6. Commands that
    // only need the machine at rest (zeroing, offsets) are allowed at an M6; the program, height map and settings
    // must not change under a job waiting there.
    bool jobActiveLocked(bool include_tool_change) const noexcept;
    // State after a move or a probe ends: ToolChangeHold while a tool change waits, else Idle
    ControllerState restStateLocked() const noexcept;
    // Door closed, chiller flowing and no PSU fault (inputs that are not configured count as fine)
    bool laserInterlocksOkLocked() const noexcept;
    // M6: runs the M6 script when it is enabled and found (it continues the job itself); false = not started
    bool runM6ScriptLocked();
    // Description of a tool from the program's comments ("(T2 D=6 flat end mill)"), empty if none
    std::string toolDescriptionLocked(uint16_t tool) const;
    // Retract to the tool change height and the optional park position (within the soft limits)
    std::vector<ToolpathSegment> toolChangeMovesLocked();
    // Starts / stops the VFD driver to match config_.vfd. Takes the controller lock itself: never call it holding it.
    void syncVfd();
    bool readyForCommandLocked(bool allow_tool_change_hold, const char* what);
    bool evaluatePinStateLocked(uint8_t pin_1_to_12, bool invert) const noexcept;
    bool anyLimitActiveLocked(std::string* desc) const;
    bool estopInputActiveLocked() const noexcept;
    void evaluateInputsAndEndStopsLocked();

    // Soft limits: true if the machine-coordinate point is inside the soft limits of homed axes
    bool withinSoftLimitsLocked(const std::array<double, 4>& machine_pt, std::string* err) const;
    bool checkLinearMoveLocked(const std::array<double, 4>& start_work, const std::array<double, 4>& end_work);
    std::array<double, 4> workToMachineLocked(const std::array<double, 4>& work) const;

    // Work frame & tool length offsets
    double activeTloLocked() const noexcept;
    void refreshWorkFrameLocked();
    double wcsOffsetForLocked(size_t axis, double work_value) const;
    void setActiveWcsAxisLocked(size_t axis, double wcs_offset);
    void updateActiveToolLengthLocked();
    bool setToolLengthOffsetLocked(uint16_t tool);
    std::array<double, 4> planFramePosLocked() const;

    // Motion helpers
    bool queueLinearMoveLocked(const std::array<double, 4>& end_work, double feed_mm_min, ControllerState state,
                               bool rapid = false);

    // Job planning
    planner::FrameContext jobFrameLocked() const;
    bool validateProgramForRunLocked(size_t start_seg);
    void setProgramLocked(std::shared_ptr<const gcode::ParseResult> program);
    // Takes the lock itself; parses outside it. wait = false (worker tick): skip if another re-parse is running
    void reparseLoadedProgramIfNeeded(bool wait = true);

    // Homing
    // axis < 0: all axes (Z, Y, X). Never runs the home script.
    void startBuiltInHomingLocked(int axis);

    // Probing
    // `sim_contact_work`: simulator only - work coordinate where the emulated contact happens
    bool beginProbeMoveLocked(core::Axis axis, double signed_travel_mm, double feed_mm_min,
                              const double* sim_contact_work = nullptr);
    void onProbeMoveFinishedLocked(bool hit);
    void continueHoleCenterLocked();

    mutable std::mutex mutex_;
    // Serializes program re-parses so a slower, older one can never replace a newer result; taken before mutex_
    std::mutex reparse_mutex_;
    MachineConfig config_{};
    mutable std::shared_ptr<const MachineConfig> config_snapshot_; // getConfigShared(): replaced when config_ differs
    MachineConfig pending_config_{};
    bool          has_pending_config_ = false;
    std::unique_ptr<board::IBoardDriver> board_;
    HuanyangVfdController vfd_controller_{};
    std::atomic<bool>     vfd_sync_needed_{false};   // config_.vfd changed: syncVfd() after releasing mutex_
    std::mutex            vfd_lifecycle_mutex_;      // Serializes syncVfd() (UI thread and worker)
    TelemetrySnapshot telemetry_{};
    uint16_t hw_inputs_bitmask_ = 0;

    std::shared_ptr<const gcode::ParseResult> program_;          // Active (possibly height-map warped) program
    std::shared_ptr<const gcode::ParseResult> unwarped_program_; // As parsed
    std::shared_ptr<const std::string>        program_text_;     // Source of the loaded program
    bool                                      program_reparse_needed_ = false;
    uint64_t                                  config_revision_ = 0; // Bumped on every settings change (program check cache)
    // Program check cache (getProgramCheck): recomputed only when the program or one of its inputs changes
    struct ProgramCheckKey {
        const void*           program = nullptr;
        planner::FrameContext frame{};
        std::array<bool, 4>   homed{};
        bool                  simulator = false;
        bool                  board_thc = false;
        MachineCheckInput::BoardLimits board{};
        uint64_t              config_revision = 0;
        uint64_t              tool_revision = 0;
        bool operator==(const ProgramCheckKey& o) const;
    };
    mutable std::mutex                                program_check_mutex_; // Taken after mutex_, never before
    mutable std::shared_ptr<const ProgramCheckResult> program_check_;
    mutable ProgramCheckKey                           program_check_key_;
    SurfaceHeightMap   surface_height_map_{};
    std::thread        surface_probing_thread_{};
    std::atomic<bool>  surface_probing_cancel_requested_{false};
    planner::FrameContext job_frame_{};
    double mdi_feed_mm_min_ = 0.0;






    // Continuous jog state (with simultaneous multi-axis smooth accel & decel ramps)
    bool cont_jog_active_ = false;
    std::array<bool, 4>   cont_jog_held_{false, false, false, false};
    std::array<Clock::time_point, 4> cont_jog_refresh_{};
    std::array<double, 4> cont_jog_target_vel_mm_s_{0.0, 0.0, 0.0, 0.0};


    // Active Probing state machine
    enum class ProbeMode : uint8_t {
        None = 0,
        FloatingHeadZ,
        ToolSetterZ,
        EdgeSingleAxis,
        HoleCenterXY,
        Raw,            // G38.2-style: stop on contact, record position, no offsets / retract
        ToolLength,     // Tool setter measurement for the tool table
    };
    uint16_t measuring_tool_       = 0;
    double   tool_ref_contact_z_   = 0.0;   // Machine Z where the reference tool touched the setter
    bool     tool_ref_contact_valid_ = false;
    ProbeMode active_probe_mode_   = ProbeMode::None;
    ControllerState probe_return_state_ = ControllerState::Idle; // State restored after a Raw probe
    core::Axis active_probe_axis_ = core::Axis::Z;
    bool active_probe_pos_dir_     = false;
    bool probe_move_is_search_     = false; // false = repositioning move inside a probe sequence
    bool probe_contact_captured_   = false;
    std::array<double, 4> probe_contact_machine_{};
    // Hole-center sequence
    int    hole_step_ = 0;
    double hole_search_mm_ = 0.0;
    std::array<double, 4> hole_start_work_{};
    std::array<double, 4> hole_contacts_work_{}; // x-, x+, y-, y+

    // Plasma THC tracking & Mill Spindle Ramp state
    double   torch_on_duration_s_      = 0.0;
    double   spindle_ramp_pct_         = 0.0;
    bool     manual_spindle_requested_ = false;
    uint16_t target_spindle_pwm_duty_  = 500;
    int      vfd_last_run_cmd_         = 0; // 0 = stop, 1 = forward, 2 = reverse

    // Laser State & Timing Control
    float    laser_power_override_pct_ = 100.0f;
    bool     pulse_active_             = false;
    double   pulse_timer_s_            = 0.0;
    double   pulse_duration_s_         = 0.05;
    float    pulse_power_pct_          = 10.0f;
    double   air_postflow_timer_s_     = 0.0;
    uint16_t last_sent_laser_pwm_duty_ = 0xFFFF;

    std::atomic<bool> running_thread_{false};
    std::thread worker_thread_;

    std::unique_ptr<scripting::ScriptRunner> script_runner_;

    // Firmware-planned board path (GRBL family); nullptr while an RNR board is selected
    std::unique_ptr<motion::FirmwarePlannedPath> fw_;
    std::mutex fw_ptr_mutex_;              // Guards fw_ replacement against triggerEStop() (taken after mutex_)
    std::string board_id_ = board::kRnrBoardId;
    uint64_t caps_revision_ = 1;

    // Host-planned boards (RNR) and the simulator: slice queue, planner, board feedback, app-driven homing
    std::unique_ptr<SliceBackend> slice_;
    // Firmware-planned boards (GRBL family): drives fw_ while one is connected
    std::unique_ptr<FirmwareBackend> firmware_;

    static inline std::atomic<bool> hardware_access_enabled_{true};
};

} // namespace rnr::core
