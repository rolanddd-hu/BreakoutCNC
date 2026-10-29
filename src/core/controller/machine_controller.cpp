#include "core/controller/machine_controller.h"
#include "core/scripting/script_runner.h"
#include "core/scripting/script_path_helper.h"
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <algorithm>
#include <filesystem>

#ifdef _WIN32
// Raise the Windows scheduler resolution to 1 ms for the 25 ms motion tick (default granularity is 15.6 ms)
extern "C" __declspec(dllimport) unsigned int __stdcall timeBeginPeriod(unsigned int uPeriod);
extern "C" __declspec(dllimport) unsigned int __stdcall timeEndPeriod(unsigned int uPeriod);
#pragma comment(lib, "winmm.lib")
#endif

namespace rnr::core {

namespace {

constexpr auto   kTickInterval        = std::chrono::milliseconds(25); // 40 Hz engine loop
constexpr double kDtSec               = 0.025;
static_assert(kDtSec == board::IBoardDriver::kSlicePeriodSec, "The engine tick must match the slice period");
constexpr size_t kRefillThreshold     = 20;  // Re-plan when fewer than this many slices are queued
constexpr size_t kRefillAmount        = 40;
constexpr size_t kOverrideKeepSlices  = 12;  // Slices kept when a feed override change re-plans the queue
constexpr auto   kAckTimeout          = std::chrono::milliseconds(1500);
constexpr auto   kReportTimeout       = std::chrono::milliseconds(500);
constexpr auto   kFirstReportTimeout  = std::chrono::milliseconds(2000);
constexpr auto   kSettleTimeout       = std::chrono::milliseconds(400);
constexpr double kFloatingHeadPierceHeightMm = 3.8;
const char* const kAxisNames[4] = {"X", "Y", "Z", "A"};

bool finitePositive(double v) {
    return std::isfinite(v) && v > 0.0;
}

// Replace invalid / dangerous values so the motion code can rely on sane numbers
void sanitizeConfig(MachineConfig& c) {
    for (auto& ax : c.axes) {
        if (!finitePositive(ax.steps_per_mm))        ax.steps_per_mm = 200.0;
        if (!finitePositive(ax.max_velocity_mm_s))   ax.max_velocity_mm_s = 10.0;
        if (!finitePositive(ax.acceleration_mm_s2))  ax.acceleration_mm_s2 = 100.0;
        if (!finitePositive(ax.deceleration_mm_s2))  ax.deceleration_mm_s2 = ax.acceleration_mm_s2;
        if (!finitePositive(ax.homing_feed_mm_min))  ax.homing_feed_mm_min = 300.0;
        if (!finitePositive(ax.homing_max_travel_mm)) ax.homing_max_travel_mm = 1500.0;
        if (!std::isfinite(ax.home_pulloff_mm) || ax.home_pulloff_mm < 0.0) ax.home_pulloff_mm = 2.5;
        if (ax.limit_plus_pin > 12)  ax.limit_plus_pin = 0;
        if (ax.limit_minus_pin > 12) ax.limit_minus_pin = 0;
        if (ax.home_pin > 12)        ax.home_pin = 0;
        if (ax.soft_limits_enabled && !(ax.soft_limit_min_mm < ax.soft_limit_max_mm)) {
            ax.soft_limits_enabled = false;
        }
    }
    if (c.pulse_rate_khz == 0 || c.pulse_rate_khz > 100) c.pulse_rate_khz = 100;
    if (c.estop_pin > 12) c.estop_pin = 0;
    if (c.probe.probe_pin > 12) c.probe.probe_pin = 0;
    c.spindle_ramp_up_s   = std::clamp(std::isfinite(c.spindle_ramp_up_s) ? c.spindle_ramp_up_s : 0.0, 0.0, 30.0);
    c.spindle_ramp_down_s = std::clamp(std::isfinite(c.spindle_ramp_down_s) ? c.spindle_ramp_down_s : 0.0, 0.0, 30.0);
    if (c.vfd.max_rpm == 0) c.vfd.max_rpm = 24000;
    if (c.vfd.min_rpm > c.vfd.max_rpm) c.vfd.min_rpm = c.vfd.max_rpm;
    if (!(c.vfd.max_freq_hz > 0.0f)) c.vfd.max_freq_hz = 400.0f;
    c.vfd.speed_margin_pct    = std::clamp<uint8_t>(c.vfd.speed_margin_pct, 1, 50);
    c.vfd.bad_speed_threshold = std::max<uint8_t>(c.vfd.bad_speed_threshold, 1);
    c.vfd.tracking_threshold  = std::max<uint8_t>(c.vfd.tracking_threshold, 1);
    for (float* lim : {&c.vfd.limit_out_freq_hz, &c.vfd.limit_current_a, &c.vfd.limit_rpm,
                       &c.vfd.limit_dc_volts, &c.vfd.limit_ac_volts, &c.vfd.limit_temp_c}) {
        if (!std::isfinite(*lim) || *lim < 0.0f) *lim = 0.0f;
    }
    c.active_workspace = std::min<uint8_t>(c.active_workspace, 5);
    // Tool table: valid tool numbers and plausible lengths only
    for (auto it = c.tool_table.tools.begin(); it != c.tool_table.tools.end();) {
        auto& e = it->second;
        if (it->first == 0 || it->first > 255 || !std::isfinite(e.length_mm) || std::abs(e.length_mm) > 1000.0) {
            it = c.tool_table.tools.erase(it);
            continue;
        }
        if (!std::isfinite(e.diameter_mm) || e.diameter_mm < 0.0) e.diameter_mm = 0.0;
        ++it;
    }
    if (c.tool_table.reference_tool == 0 || c.tool_table.reference_tool > 255) c.tool_table.reference_tool = 1;
}

uint16_t clampRpm(uint32_t rpm) {
    return static_cast<uint16_t>(std::min<uint32_t>(rpm, 65535u));
}

uint16_t rpmToDuty(uint32_t rpm, uint32_t max_rpm) {
    max_rpm = std::max<uint32_t>(1, max_rpm);
    return static_cast<uint16_t>((static_cast<uint64_t>(std::min(rpm, max_rpm)) * 1000u) / max_rpm);
}

double dist3(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

// ============================================================================
// Construction / connection
// ============================================================================
MachineController::MachineController(std::unique_ptr<board::IBoardDriver> board)
    : board_(board ? std::move(board) : board::createBoardDriver(board::BoardType::RnrEcoMotion)) {
    max_slices_in_flight_ = std::max<size_t>(1, board_->capabilities().max_slices_in_flight);
    script_runner_ = std::make_unique<scripting::ScriptRunner>(this);

    for (size_t i = 0; i < 4; ++i) {
        auto& ax = config_.axes[i];
        ax.steps_per_mm        = 200.0;
        ax.max_velocity_mm_s   = 100.0;
        ax.acceleration_mm_s2  = 500.0;
        ax.deceleration_mm_s2  = 600.0;
        ax.home_pulloff_mm     = 2.5;
        ax.homing_feed_mm_min  = 900.0;
        ax.homing_dir_negative = (i != 2); // X/Y home toward -, Z homes toward + (up)
    }
    config_.axes[2].max_velocity_mm_s  = 30.0;  // Z axis max 1800 mm/min
    config_.axes[2].acceleration_mm_s2 = 250.0;
    config_.axes[2].deceleration_mm_s2 = 300.0;
    config_.axes[2].homing_feed_mm_min = 500.0;

    telemetry_.state = ControllerState::Disconnected;
    telemetry_.feed_override_pct = 100.0f;

    program_          = std::make_shared<const gcode::ParseResult>();
    unwarped_program_ = program_;

    if (!connect()) {
        setSimulatorMode(true);
    }

    running_thread_.store(true);
    worker_thread_ = std::thread(&MachineController::workerLoop, this);
}

MachineController::~MachineController() {
    cancelSurfaceProbing();
    if (surface_probing_thread_.joinable()) {
        surface_probing_thread_.join();
    }
    triggerEStop();
    if (script_runner_) {
        script_runner_->cancel();
    }
    running_thread_.store(false);
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    disconnect();
}

bool MachineController::connect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked()) {
        stopJobLocked(true);
    }
    // Unit tests disable hardware access so a board that happens to be plugged in is never driven
    const bool blocked = board_->capabilities().accesses_real_hardware && !hardware_access_enabled_.load();
    if (blocked || !board_->open()) {
        telemetry_.usb_connected = false;
        return false;
    }
    const auto info = board_->info();
    telemetry_.usb_connected  = true;
    telemetry_.simulator_mode = false;
    telemetry_.product_name   = info.product_name;
    telemetry_.serial_number  = info.serial_number;

    board_->configure(config_, false);
    // Engage E-Stop by default on connect/startup so machine is safely disarmed until user clicks RESET E-STOP
    board_->emergencyStop();
    board_->setOutputs(0x00, false);
    telemetry_.outputs_bitmask     = 0;
    telemetry_.torch_or_spindle_on = false;

    // Sync board hardware step accumulators to the current machine coordinates
    syncBoardPositionLocked();

    board_report_seen_    = false;
    feedback_validated_   = false;
    connect_time_         = Clock::now();
    last_report_time_     = connect_time_;
    hw_estop_fault_       = false;
    hw_limit_fault_       = false;
    in_flight_seqs_.clear();

    telemetry_.limit_switch_triggered = false;
    telemetry_.soft_limit_triggered   = false;
    telemetry_.active_fault_message.clear();
    telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    telemetry_.current_feed_mm_min = 0.0;
    telemetry_.state = ControllerState::EStop;
    return true;
}

void MachineController::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked()) {
        stopJobLocked(true);
    }
    if (board_->isOpen()) {
        board_->emergencyStop();
        board_->setOutputs(0x00, false);
        board_->close();
    }
    telemetry_.usb_connected = false;
    if (!telemetry_.simulator_mode) {
        telemetry_.state = ControllerState::Disconnected;
    }
}

void MachineController::setSimulatorMode(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled == telemetry_.simulator_mode && telemetry_.state != ControllerState::Disconnected) {
        return;
    }
    // Switching between real hardware and simulation mid-motion is never safe
    if (motionActiveLocked()) {
        stopJobLocked(true);
        telemetry_.state = ControllerState::EStop;
    }
    telemetry_.simulator_mode = enabled;
    if (enabled) {
        if (board_->isOpen()) {
            // The real board must stay disarmed while the simulator drives the UI
            board_->emergencyStop();
            board_->setOutputs(0x00, false);
        }
        telemetry_.product_name  = "Virtual " + board_->capabilities().display_name + " (Simulator)";
        telemetry_.serial_number = "SIM-200HZ";
        if (telemetry_.state == ControllerState::Disconnected) {
            telemetry_.state = ControllerState::EStop;
        }
    } else if (board_->isOpen()) {
        const auto info = board_->info();
        telemetry_.product_name  = info.product_name;
        telemetry_.serial_number = info.serial_number;
        // Leaving the simulator: the simulated position is meaningless for the real machine
        telemetry_.axis_homed = {false, false, false, false};
        syncBoardPositionLocked();
        feedback_validated_ = false;
        board_report_seen_  = false;
        connect_time_       = Clock::now();
        telemetry_.state    = ControllerState::EStop;
    } else {
        telemetry_.state = ControllerState::Disconnected;
    }
}

void MachineController::setDryRunMode(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.dry_run_mode = enabled;
    if (enabled) {
        telemetry_.torch_or_spindle_on = false;
        telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        manual_spindle_requested_ = false;
        if (board_->isOpen() && !telemetry_.simulator_mode) {
            board_->setOutputs(telemetry_.outputs_bitmask, false);
        }
    }
}

void MachineController::setLimitOverride(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Limit override only lets the operator jog off an end-stop. It never bypasses E-Stop or interlocks.
    config_.limit_override           = enabled;
    telemetry_.limit_override_active = enabled;
    if (enabled) {
        telemetry_.limit_switch_triggered = false;
        telemetry_.soft_limit_triggered   = false;
    }
}

void MachineController::setSimulatedInputPin(uint8_t pin_1_to_12, bool active) {
    if (pin_1_to_12 < 1 || pin_1_to_12 > 12) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const uint16_t bit = static_cast<uint16_t>(1U << (pin_1_to_12 - 1));
    if (active) {
        telemetry_.simulated_inputs_bitmask |= bit;
    } else {
        telemetry_.simulated_inputs_bitmask &= ~bit;
    }
    // Simulated inputs only take effect in simulator mode: on a real machine they could fake a closed
    // door, a flowing chiller or a probe contact.
    telemetry_.inputs_bitmask = hw_inputs_bitmask_ |
        (telemetry_.simulator_mode ? telemetry_.simulated_inputs_bitmask : static_cast<uint16_t>(0));
}

// ============================================================================
// Configuration
// ============================================================================
void MachineController::applyConfig(const MachineConfig& cfg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
            // Changing kinematics / pinout while the machine moves is unsafe: apply once motion has stopped
            pending_config_     = cfg;
            has_pending_config_ = true;
            telemetry_.active_fault_message = "Configuration change will be applied when motion stops";
            return;
        }
        applyConfigLocked(cfg);
    }
    reparseLoadedProgramIfNeeded();
}

void MachineController::reparseLoadedProgramIfNeeded(bool wait) {
    // A caller that waits (applyConfig) only returns once the program matches the new configuration, even
    // when the worker thread had already started the re-parse
    std::unique_lock<std::mutex> reparse_lock(reparse_mutex_, std::defer_lock);
    if (wait) {
        reparse_lock.lock();
    } else if (!reparse_lock.try_lock()) {
        return;
    }

    std::shared_ptr<const std::string> text;
    gcode::ParseOptions opts;
    std::array<double, 4> work_offset{};
    SurfaceHeightMap map;
    bool warp = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto s = telemetry_.state;
        if (!program_reparse_needed_ || motionActiveLocked() ||
            s == ControllerState::PausedHold || s == ControllerState::ToolChangeHold) {
            return; // Retried later (job in progress: the program must not change under it)
        }
        program_reparse_needed_ = false;
        if (!program_text_) return;
        text = program_text_;
        opts.dwell_p_milliseconds = config_.dwell_p_milliseconds;
        opts.tool_length_mode     = config_.tool_table.mode;
        work_offset = telemetry_.work_offset_mm;
        warp = telemetry_.surface_compensation_active && surface_height_map_.is_valid;
        map = surface_height_map_;
    }

    // Same text, same segments: only the diagnostics (and G4 durations) can differ
    auto parsed = std::make_shared<gcode::ParseResult>(
        gcode::GCodeParser::parse(*text, {0.0, 0.0, 0.0, 0.0}, 0.3, work_offset, opts));
    std::shared_ptr<const gcode::ParseResult> active = parsed;
    if (warp) {
        auto w = std::make_shared<gcode::ParseResult>(*parsed);
        w->segments = planner::TrajectoryPlanner::warpToolpathWithHeightMap(parsed->segments, map, 2.0);
        active = w;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto s = telemetry_.state;
    if (program_text_ != text || motionActiveLocked() ||
        s == ControllerState::PausedHold || s == ControllerState::ToolChangeHold) {
        return; // Another program was loaded, or a job started meanwhile
    }
    unwarped_program_ = parsed;
    setProgramLocked(active);
}

void MachineController::applyConfigLocked(const MachineConfig& cfg_in) {
    MachineConfig cfg = cfg_in;
    sanitizeConfig(cfg);

    // Only adopt the caller's work offsets if its copy is current; a stale copy must never overwrite
    // offsets that were changed by zeroing / probing / G10 / G92 in the meantime.
    uint64_t revision = config_.workspace_revision;
    if (cfg.workspace_revision != config_.workspace_revision) {
        cfg.workspace_offsets_mm = config_.workspace_offsets_mm;
        cfg.active_workspace     = config_.active_workspace;
    } else if (cfg.workspace_offsets_mm != config_.workspace_offsets_mm ||
               cfg.active_workspace != config_.active_workspace) {
        ++revision;
    }

    // Same for the tool table (lengths measured by the controller must survive a stale UI copy)
    uint64_t tt_revision = config_.tool_table_revision;
    if (cfg.tool_table_revision != config_.tool_table_revision) {
        cfg.tool_table = config_.tool_table;
    } else {
        ++tt_revision;
    }

    bool steps_changed = false;
    for (size_t a = 0; a < 4; ++a) {
        if (cfg.axes[a].steps_per_mm != config_.axes[a].steps_per_mm) steps_changed = true;
    }
    // Program diagnostics depend on the tool length mode (G43 warning): re-parse the loaded program
    if (cfg.tool_table.mode != config_.tool_table.mode || cfg.dwell_p_milliseconds != config_.dwell_p_milliseconds) {
        program_reparse_needed_ = true;
    }

    config_ = cfg;
    config_.workspace_revision  = revision;
    config_.tool_table_revision = tt_revision;

    telemetry_.operation_mode       = cfg.operation_mode;
    telemetry_.workspace_offsets_mm = cfg.workspace_offsets_mm;
    telemetry_.active_workspace     = cfg.active_workspace;
    // The active tool length offset follows the (possibly edited) table and mode
    updateActiveToolLengthLocked();
    refreshWorkFrameLocked();
    telemetry_.limit_override_active = cfg.limit_override;
    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->configure(cfg, false);
        if (steps_changed) {
            // Board step counters are in steps: re-express the current position with the new scale
            syncBoardPositionLocked();
        }
    }

    if (cfg.vfd.enabled && !vfd_controller_.isRunning()) {
        vfd_controller_.start(cfg.vfd);
    } else if (!cfg.vfd.enabled && vfd_controller_.isRunning()) {
        vfd_controller_.stop();
    } else if (cfg.vfd.enabled) {
        vfd_controller_.updateConfig(cfg.vfd);
    }
}

MachineConfig MachineController::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

void MachineController::setHuanyangVfdConfig(const HuanyangVfdConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.vfd = config;
    if (config_.vfd.min_rpm > config_.vfd.max_rpm) config_.vfd.min_rpm = config_.vfd.max_rpm;
    if (config.enabled && !vfd_controller_.isRunning()) {
        vfd_controller_.start(config_.vfd);
    } else if (!config.enabled && vfd_controller_.isRunning()) {
        vfd_controller_.stop();
    } else if (config.enabled) {
        vfd_controller_.updateConfig(config_.vfd);
    }
}

HuanyangVfdConfig MachineController::getHuanyangVfdConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_.vfd;
}

HuanyangVfdTelemetry MachineController::getHuanyangVfdTelemetry() const {
    return vfd_controller_.getTelemetry();
}

void MachineController::clearVfdAlarms() {
    vfd_controller_.clearAlarms();
}

void MachineController::requestVfdParameterRead() {
    vfd_controller_.requestParameterRead();
}

TelemetrySnapshot MachineController::getTelemetry() const {
    TelemetrySnapshot telem;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        telem = telemetry_;
    }
    // Script state is queried without holding the controller lock (the script thread calls back into us)
    if (script_runner_) {
        telem.script_running = script_runner_->isRunning();
        telem.active_script_name = script_runner_->getActiveScriptName();
        telem.has_script_prompt = script_runner_->hasPendingPrompt();
        if (telem.has_script_prompt) {
            auto req = script_runner_->getPendingPrompt();
            telem.script_prompt_title = req.title;
            telem.script_prompt_message = req.message;
            telem.script_prompt_buttons = req.buttons;
            telem.script_prompt_input_kind    = static_cast<uint8_t>(req.input_kind);
            telem.script_prompt_input_default = req.input_default;
            telem.script_prompt_input_min     = req.input_min;
            telem.script_prompt_input_max     = req.input_max;
        }
    }
    return telem;
}

// ============================================================================
// E-Stop & faults
// ============================================================================
void MachineController::triggerEStop() {
    // Fast path: halt the board before anything else. The USB device serializes its own I/O, so this
    // does not have to wait for the controller lock (which the worker may hold for a full tick).
    if (board_->isOpen()) {
        board_->emergencyStop();
    }
    if (script_runner_) {
        script_runner_->cancel();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    faultLocked("EMERGENCY STOP TRIGGERED");
}

void MachineController::faultLocked(const std::string& message, ControllerState state) {
    const bool was_moving = motionActiveLocked() || hold_decel_active_;
    stopJobLocked(true);
    telemetry_.state = state;
    telemetry_.active_fault_message = message;
    config_.limit_override = false;
    telemetry_.limit_override_active = false;
    if (was_moving && !telemetry_.simulator_mode && !feedback_validated_) {
        // The board was stopped mid-motion and we cannot read back where it actually stopped
        telemetry_.axis_homed = {false, false, false, false};
        telemetry_.active_fault_message += " - position uncertain, re-home the machine";
    }
}

bool MachineController::resetFromEStop() {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool hw = !telemetry_.simulator_mode;
    if (hw && !board_->isOpen()) {
        telemetry_.active_fault_message = "Cannot reset: motion board is not connected";
        return false;
    }
    evaluateInputsAndEndStopsLocked();
    if (estopInputActiveLocked()) {
        telemetry_.active_fault_message = "Cannot reset: E-Stop input is still active";
        return false;
    }
    if (hw && hw_estop_fault_) {
        telemetry_.active_fault_message = "Cannot reset: board reports hardware E-Stop";
        return false;
    }
    std::string limit_desc;
    if (!config_.limit_override && anyLimitActiveLocked(&limit_desc)) {
        telemetry_.active_fault_message = "Cannot reset: " + limit_desc + " is active - enable Limit Override to jog off";
        return false;
    }
    if (config_.operation_mode == OperationMode::Laser && !config_.is_mill) {
        if (config_.laser.chiller_flow_pin >= 1 && !telemetry_.laser_chiller_flow_ok) {
            telemetry_.active_fault_message = "Cannot reset: laser chiller flow interlock not satisfied";
            return false;
        }
        if (config_.laser.laser_fault_pin >= 1 && telemetry_.laser_fault_active) {
            telemetry_.active_fault_message = "Cannot reset: laser PSU fault input is active";
            return false;
        }
    }
    if (config_.vfd.enabled) {
        const auto vt = vfd_controller_.getTelemetry();
        if (vt.anyAlarm()) {
            telemetry_.active_fault_message = "Cannot reset: clear the VFD alarm first (" + vt.last_error_msg + ")";
            return false;
        }
    }

    if (board_->isOpen() && hw) {
        board_->clearEmergencyStop();
    }
    telemetry_.limit_switch_triggered = false;
    telemetry_.soft_limit_triggered   = false;
    telemetry_.active_fault_message.clear();
    telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    telemetry_.current_feed_mm_min = 0.0;
    telemetry_.state = ControllerState::Idle;
    return true;
}

bool MachineController::motionActiveLocked() const noexcept {
    const auto s = telemetry_.state;
    return s == ControllerState::RunningGCode || s == ControllerState::DwellPierce ||
           s == ControllerState::Jogging || s == ControllerState::Homing || s == ControllerState::Probing ||
           cont_jog_active_ || hold_decel_active_;
}

bool MachineController::readyForCommandLocked(bool allow_tool_change_hold, const char* what) {
    const auto s = telemetry_.state;
    const bool ok_state = (s == ControllerState::Idle) ||
                          (allow_tool_change_hold && s == ControllerState::ToolChangeHold);
    if (!ok_state || cont_jog_active_ || hold_decel_active_) {
        return false;
    }
    if (telemetry_.surface_probing_active) {
        telemetry_.active_fault_message = std::string(what) + " rejected: surface probing in progress";
        return false;
    }
    return true;
}

// ============================================================================
// Coordinates & work offsets
// ============================================================================
void MachineController::setMachinePositionLocked(const std::array<double, 4>& machine_pos_mm) {
    telemetry_.machine_pos_mm = machine_pos_mm;
    for (size_t a = 0; a < 4; ++a) {
        telemetry_.work_pos_mm[a] = telemetry_.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
    }
    syncBoardPositionLocked();
    last_sent_target_valid_ = false;
}

void MachineController::syncBoardPositionLocked() {
    if (!board_->isOpen() || telemetry_.simulator_mode) return;
    std::array<int32_t, 4> steps{};
    for (size_t i = 0; i < 4; ++i) {
        steps[i] = static_cast<int32_t>(std::llround(telemetry_.machine_pos_mm[i] * config_.axes[i].steps_per_mm));
    }
    board_->setAxisPositions(steps);
}

std::array<double, 4> MachineController::workToMachineLocked(const std::array<double, 4>& work) const {
    std::array<double, 4> m{};
    for (size_t a = 0; a < 4; ++a) m[a] = work[a] + telemetry_.work_offset_mm[a];
    return m;
}

// ----------------------------------------------------------------------------
// Work frame with tool length offset:
//   work_offset (effective, telemetry) = WCS offset + (0, 0, active tool length, 0)
//   work position = machine position - effective work offset   (= tool tip in work coordinates)
// ----------------------------------------------------------------------------
double MachineController::activeTloLocked() const noexcept {
    return (config_.tool_table.mode == ToolLengthMode::ToolTable) ? telemetry_.active_tlo_mm : 0.0;
}

void MachineController::refreshWorkFrameLocked() {
    telemetry_.work_offset_mm = telemetry_.workspace_offsets_mm[std::min<uint8_t>(telemetry_.active_workspace, 5)];
    telemetry_.work_offset_mm[2] += activeTloLocked();
    for (size_t a = 0; a < 4; ++a) {
        telemetry_.work_pos_mm[a] = telemetry_.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
    }
}

double MachineController::wcsOffsetForLocked(size_t axis, double work_value) const {
    // WCS offset that makes the CURRENT tool tip read `work_value` on this axis
    return telemetry_.machine_pos_mm[axis] - work_value - (axis == 2 ? activeTloLocked() : 0.0);
}

void MachineController::setActiveWcsAxisLocked(size_t axis, double wcs_offset) {
    telemetry_.workspace_offsets_mm[telemetry_.active_workspace][axis] = wcs_offset;
    config_.workspace_offsets_mm = telemetry_.workspace_offsets_mm;
    bumpWorkspaceRevisionLocked();
    refreshWorkFrameLocked();
}

void MachineController::updateActiveToolLengthLocked() {
    if (config_.tool_table.mode != ToolLengthMode::ToolTable) {
        telemetry_.active_tlo_tool = 0;
        telemetry_.active_tlo_mm   = 0.0;
        return;
    }
    const auto it = config_.tool_table.tools.find(telemetry_.active_tlo_tool);
    telemetry_.active_tlo_mm = (telemetry_.active_tlo_tool != 0 && it != config_.tool_table.tools.end())
                                   ? it->second.length_mm : 0.0;
}

bool MachineController::setToolLengthOffsetLocked(uint16_t tool) {
    if (config_.tool_table.mode != ToolLengthMode::ToolTable) {
        telemetry_.active_tlo_tool = 0;
        telemetry_.active_tlo_mm   = 0.0;
        refreshWorkFrameLocked();
        return true; // G43 / G49 have no effect outside tool table mode
    }
    if (tool != 0 && config_.tool_table.tools.find(tool) == config_.tool_table.tools.end()) {
        telemetry_.active_fault_message = "Tool T" + std::to_string(tool) + " has no length in the tool table";
        return false;
    }
    telemetry_.active_tlo_tool = tool;
    updateActiveToolLengthLocked();
    refreshWorkFrameLocked();
    return true;
}

void MachineController::zeroWorkAxis(core::Axis axis) {
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
        telemetry_.active_fault_message = "Cannot zero an axis while a job is active";
        return;
    }
    setActiveWcsAxisLocked(idx, wcsOffsetForLocked(idx, 0.0));
    if (idx == 2) telemetry_.thc_z_offset_mm = 0.0;
}

void MachineController::zeroAllWorkAxes() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
        telemetry_.active_fault_message = "Cannot zero axes while a job is active";
        return;
    }
    for (size_t i = 0; i < 4; ++i) {
        setActiveWcsAxisLocked(i, wcsOffsetForLocked(i, 0.0));
    }
    telemetry_.thc_z_offset_mm = 0.0;
}

void MachineController::goToWorkZeroXY() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Go to zero")) return;

    // Retract Z to a safe height first so the XY travel cannot drag the tool through the work
    const std::array<double, 4> cur = telemetry_.work_pos_mm;
    const double safe_z = std::max(cur[2], config_.tool_change.safe_z_mm);
    std::vector<ToolpathSegment> moves;
    auto addRapid = [&](const std::array<double, 4>& a, const std::array<double, 4>& b) {
        ToolpathSegment s{};
        s.type          = SegmentType::RapidMove;
        s.start_xyza_mm = a;
        s.end_xyza_mm   = b;
        s.outputs_mask  = telemetry_.outputs_bitmask & ~core::OutputBit::OUT1_TorchOrSpindle;
        s.length_mm     = dist3(a, b);
        moves.push_back(s);
    };
    std::array<double, 4> up = {cur[0], cur[1], safe_z, cur[3]};
    std::array<double, 4> xy = {0.0, 0.0, safe_z, cur[3]};
    if (!checkLinearMoveLocked(cur, up) || !checkLinearMoveLocked(up, xy)) return;
    if (safe_z - cur[2] > 1e-3) addRapid(cur, up);
    addRapid(up, xy);
    MachineConfig manual_cfg = config_;
    manual_cfg.is_mill = true; // Manual moves: no laser Z lock / output rules
    slice_queue_ = planner::TrajectoryPlanner::sliceProgram(moves, manual_cfg);
    if (!slice_queue_.empty()) {
        if (board_->isOpen() && !telemetry_.simulator_mode) {
            board_->beginJog(Axis::X, config_.limit_override);
        }
        telemetry_.state = ControllerState::Jogging;
    }
}

void MachineController::selectWorkspace(uint8_t index) {
    if (index >= 6) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
        telemetry_.active_fault_message = "Cannot change the work coordinate system while a job is active";
        return;
    }
    telemetry_.active_workspace = index;
    config_.active_workspace    = index;
    bumpWorkspaceRevisionLocked();
    refreshWorkFrameLocked();
}

void MachineController::setWorkspaceOffset(uint8_t index, const std::array<double, 4>& offset_mm) {
    if (index >= 6) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
        telemetry_.active_fault_message = "Cannot change work offsets while a job is active";
        return;
    }
    telemetry_.workspace_offsets_mm[index] = offset_mm;
    config_.workspace_offsets_mm           = telemetry_.workspace_offsets_mm;
    bumpWorkspaceRevisionLocked();
    refreshWorkFrameLocked();
}

void MachineController::setWorkspaceOffsetAxis(uint8_t index, core::Axis axis, double offset_mm) {
    if (index >= 6) return;
    const size_t a = static_cast<size_t>(axis);
    if (a >= 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
        telemetry_.active_fault_message = "Cannot change work offsets while a job is active";
        return;
    }
    telemetry_.workspace_offsets_mm[index][a] = offset_mm;
    config_.workspace_offsets_mm              = telemetry_.workspace_offsets_mm;
    bumpWorkspaceRevisionLocked();
    refreshWorkFrameLocked();
}

uint8_t MachineController::getActiveWorkspace() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_.active_workspace;
}

std::array<std::array<double, 4>, 6> MachineController::getWorkspaceOffsets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_.workspace_offsets_mm;
}

void MachineController::setWorkPosition(core::Axis axis, double value_mm) {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4) return;
    if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
        telemetry_.active_fault_message = "Cannot set work position while a job is active";
        return;
    }
    setActiveWcsAxisLocked(idx, wcsOffsetForLocked(idx, value_mm));
    if (idx == 2) telemetry_.thc_z_offset_mm = 0.0;
}

void MachineController::restoreSavedPositions(
    const std::array<double, 4>& machine_pos_mm,
    const std::array<double, 4>& work_offset_mm)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked()) return;
    // `work_offset_mm` is the WCS offset (without any tool length offset)
    telemetry_.workspace_offsets_mm[telemetry_.active_workspace] = work_offset_mm;
    config_.workspace_offsets_mm = telemetry_.workspace_offsets_mm;
    bumpWorkspaceRevisionLocked();
    // A restored position is only a best guess (the machine may have moved while powered off):
    // the axes are NOT considered homed until a real homing cycle has run, and tool lengths measured
    // against the old machine Z are not comparable with new measurements.
    telemetry_.axis_homed = {false, false, false, false};
    tool_ref_contact_valid_ = false;
    telemetry_.tool_reference_measured = false;
    refreshWorkFrameLocked();
    setMachinePositionLocked(machine_pos_mm);
}

// ============================================================================
// Tool table
// ============================================================================
void MachineController::selectToolInSpindle(uint16_t tool) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tool > 255) return;
    if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
        telemetry_.active_fault_message = "Cannot change the tool in spindle while a job is active";
        return;
    }
    if (config_.tool_table.mode == ToolLengthMode::ToolTable && !setToolLengthOffsetLocked(tool)) {
        return; // Unknown length: keep the previous tool & offset rather than guessing
    }
    if (tool > 0) telemetry_.current_tool_number = tool;
}

void MachineController::activateToolLengthOffset(uint16_t tool) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tool > 255) return;
    if (motionActiveLocked() || telemetry_.state == ControllerState::PausedHold) {
        telemetry_.active_fault_message = "Cannot change the tool length offset while a job is active";
        return;
    }
    setToolLengthOffsetLocked(tool);
}

void MachineController::measureToolLength(uint16_t tool) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tool == 0 || tool > 255) return;
    if (config_.tool_table.mode != ToolLengthMode::ToolTable) {
        telemetry_.active_fault_message = "Tool measuring needs Tool Length Mode = Tool Table";
        return;
    }
    if (!readyForCommandLocked(true, "Tool measuring")) return;
    if (tool != config_.tool_table.reference_tool && !tool_ref_contact_valid_) {
        telemetry_.active_fault_message = "Measure the reference tool T" + std::to_string(config_.tool_table.reference_tool) +
                                          " first (required after every startup / Z homing)";
        return;
    }
    measuring_tool_ = tool;
    active_probe_mode_ = ProbeMode::ToolLength;
    const double travel = std::max(5.0, config_.probe.max_travel_mm);
    if (!beginProbeMoveLocked(core::Axis::Z, -travel, std::max(30.0, config_.probe.fast_feed_mm_min))) {
        active_probe_mode_ = ProbeMode::None;
    }
}

// ============================================================================
// Soft limits
// ============================================================================
bool MachineController::withinSoftLimitsLocked(const std::array<double, 4>& machine_pt, std::string* err) const {
    for (size_t a = 0; a < 4; ++a) {
        const auto& ax = config_.axes[a];
        // Soft limits are only meaningful relative to a homed machine coordinate system
        if (!ax.soft_limits_enabled || !telemetry_.axis_homed[a]) continue;
        if (machine_pt[a] < ax.soft_limit_min_mm - 1e-6 || machine_pt[a] > ax.soft_limit_max_mm + 1e-6) {
            if (err) {
                char buf[160];
                std::snprintf(buf, sizeof(buf), "%s %.3f mm (machine) is outside the soft limits %.1f..%.1f mm",
                              kAxisNames[a], machine_pt[a], ax.soft_limit_min_mm, ax.soft_limit_max_mm);
                *err = buf;
            }
            return false;
        }
    }
    return true;
}

bool MachineController::checkLinearMoveLocked(const std::array<double, 4>& start_work, const std::array<double, 4>& end_work) {
    (void)start_work; // A straight line between two in-limit points stays inside the (box) limits
    std::string err;
    if (!withinSoftLimitsLocked(workToMachineLocked(end_work), &err)) {
        telemetry_.active_fault_message = "Move rejected: " + err;
        return false;
    }
    return true;
}

bool MachineController::queueLinearMoveLocked(const std::array<double, 4>& end_work, double feed_mm_min,
                                          ControllerState state, bool rapid)
{
    const std::array<double, 4> start_w = telemetry_.work_pos_mm;
    if (!checkLinearMoveLocked(start_w, end_work)) return false;
    if (rapid) {
        ToolpathSegment s{};
        s.type          = SegmentType::RapidMove;
        s.start_xyza_mm = start_w;
        s.end_xyza_mm   = end_work;
        s.length_mm     = dist3(start_w, end_work);
        MachineConfig manual_cfg = config_;
        manual_cfg.is_mill = true; // Manual moves: no laser Z lock / output rules
        slice_queue_ = planner::TrajectoryPlanner::sliceProgram({s}, manual_cfg);
    } else {
        slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(start_w, end_work, feed_mm_min, config_, 0);
    }
    telemetry_.state = state;
    return true;
}

// ============================================================================
// Jogging
// ============================================================================
void MachineController::jogStartContinuous(core::Axis axis, bool positive_dir, double speed_ratio) {
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto s = telemetry_.state;
    const bool state_ok = (s == ControllerState::Idle || s == ControllerState::ToolChangeHold ||
                           (s == ControllerState::Jogging && cont_jog_active_));
    if (!state_ok || hold_decel_active_ || telemetry_.surface_probing_active) {
        return;
    }

    if (!cont_jog_active_) {
        if (!slice_queue_.empty()) return; // A step jog / MDI move is still executing
        for (size_t a = 0; a < 4; ++a) {
            cont_jog_ideal_pos_mm_[a]     = telemetry_.machine_pos_mm[a];
            cont_jog_accum_steps_[a]      = static_cast<int32_t>(std::llround(telemetry_.machine_pos_mm[a] * config_.axes[a].steps_per_mm));
            cont_jog_current_vel_mm_s_[a] = 0.0;
            cont_jog_target_vel_mm_s_[a]  = 0.0;
            cont_jog_held_[a]             = false;
        }
    }

    const double max_vel = planner::TrajectoryPlanner::axisVelocityLimit(config_, idx);
    const double target_vel = std::clamp(speed_ratio, 0.05, 1.0) * max_vel * (positive_dir ? 1.0 : -1.0);

    if (!cont_jog_held_[idx] && std::abs(cont_jog_current_vel_mm_s_[idx]) < 1e-9) {
        if (board_->isOpen() && !telemetry_.simulator_mode) {
            board_->beginJog(static_cast<Axis>(idx), config_.limit_override);
        }
    }
    // Direction changes ramp through zero using the normal accel/decel limits (no instant reversal)
    cont_jog_held_[idx]            = true;
    cont_jog_refresh_[idx]         = Clock::now();
    cont_jog_active_               = true;
    cont_jog_target_vel_mm_s_[idx] = target_vel;
    telemetry_.state               = ControllerState::Jogging;
}

void MachineController::jogStopAxis(core::Axis axis) {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4) return;
    cont_jog_held_[idx]            = false;
    cont_jog_target_vel_mm_s_[idx] = 0.0;
}

void MachineController::jogStep(core::Axis axis, double delta_mm, double feed_mm_min) {
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(true, "Jog")) return;

    std::array<double, 4> start_work = telemetry_.work_pos_mm;
    std::array<double, 4> end_work   = start_work;
    end_work[idx] += delta_mm;

    // Clamp the step to the soft limits of a homed axis (instead of faulting after crossing them)
    const auto& ax = config_.axes[idx];
    if (ax.soft_limits_enabled && telemetry_.axis_homed[idx]) {
        const double off = telemetry_.work_offset_mm[idx];
        end_work[idx] = std::clamp(end_work[idx] + off, ax.soft_limit_min_mm, ax.soft_limit_max_mm) - off;
    }
    if (std::abs(end_work[idx] - start_work[idx]) < 1e-6) return;

    const double feed = std::clamp(feed_mm_min, 1.0, planner::TrajectoryPlanner::axisVelocityLimit(config_, idx) * 60.0);
    slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(start_work, end_work, feed, config_, telemetry_.outputs_bitmask);
    if (!slice_queue_.empty()) {
        if (board_->isOpen() && !telemetry_.simulator_mode) {
            board_->beginJog(static_cast<Axis>(idx), config_.limit_override);
        }
        telemetry_.state = ControllerState::Jogging;
    }
}

void MachineController::jogStop() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t a = 0; a < 4; ++a) {
        cont_jog_held_[a]            = false;
        cont_jog_target_vel_mm_s_[a] = 0.0;
    }
    if (!cont_jog_active_ && telemetry_.state == ControllerState::Jogging && !slice_queue_.empty()) {
        // Step jog / MDI move: decelerate along the path instead of cutting the pulse stream
        beginHoldDecelLocked(true);
    }
}

void MachineController::updateContinuousJogLocked() {
    if (!cont_jog_active_) return;
    const auto now = Clock::now();
    bool any_axis_moving = false;
    StepSlice ms{};
    ms.outputs_mask = telemetry_.outputs_bitmask;

    for (size_t a = 0; a < 4; ++a) {
        // Dead-man: a jog input that is no longer being refreshed is treated as released
        if (cont_jog_held_[a] &&
            now - cont_jog_refresh_[a] > std::chrono::milliseconds(kJogDeadmanMs)) {
            cont_jog_held_[a] = false;
            cont_jog_target_vel_mm_s_[a] = 0.0;
        }

        const auto& ax = config_.axes[a];
        const double accel = std::max(10.0, ax.acceleration_mm_s2);
        const double decel = std::max(10.0, ax.deceleration_mm_s2);
        double& vel = cont_jog_current_vel_mm_s_[a];

        // Soft limits: start braking early enough to stop at the limit of a homed axis
        bool force_stop = false;
        if (ax.soft_limits_enabled && telemetry_.axis_homed[a] && std::abs(vel) > 1e-9) {
            const double pos = cont_jog_ideal_pos_mm_[a];
            const double dist = (vel > 0.0) ? (ax.soft_limit_max_mm - pos) : (pos - ax.soft_limit_min_mm);
            const double stop_dist = (vel * vel) / (2.0 * decel) + std::abs(vel) * kDtSec;
            if (dist <= stop_dist) force_stop = true;
        }
        if (ax.soft_limits_enabled && telemetry_.axis_homed[a] && cont_jog_held_[a]) {
            const double pos = cont_jog_ideal_pos_mm_[a];
            const double tv = cont_jog_target_vel_mm_s_[a];
            if ((tv > 0.0 && pos >= ax.soft_limit_max_mm) || (tv < 0.0 && pos <= ax.soft_limit_min_mm)) {
                force_stop = true;
            }
        }

        if (cont_jog_held_[a] && !force_stop) {
            const double diff = cont_jog_target_vel_mm_s_[a] - vel;
            // Speeding up uses the acceleration limit, slowing down / reversing the deceleration limit
            const bool slowing = (vel > 0.0 && diff < 0.0) || (vel < 0.0 && diff > 0.0);
            const double max_dv = (slowing ? decel : accel) * kDtSec;
            vel = (std::abs(diff) <= max_dv) ? cont_jog_target_vel_mm_s_[a] : vel + (diff > 0.0 ? max_dv : -max_dv);
        } else {
            const double max_dv = decel * kDtSec;
            vel = (std::abs(vel) <= max_dv) ? 0.0 : vel - (vel > 0.0 ? max_dv : -max_dv);
        }

        if (std::abs(vel) > 1e-6) {
            any_axis_moving = true;
            double next = cont_jog_ideal_pos_mm_[a] + vel * kDtSec;
            if (ax.soft_limits_enabled && telemetry_.axis_homed[a]) {
                next = std::clamp(next, ax.soft_limit_min_mm, ax.soft_limit_max_mm);
            }
            cont_jog_ideal_pos_mm_[a] = next;
            const int32_t target_steps = static_cast<int32_t>(std::llround(next * ax.steps_per_mm));
            ms.delta_steps[a] = target_steps - cont_jog_accum_steps_[a];
            cont_jog_accum_steps_[a] = target_steps;
            telemetry_.machine_pos_mm[a]  = static_cast<double>(target_steps) / ax.steps_per_mm;
            telemetry_.work_pos_mm[a]     = telemetry_.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
            telemetry_.axis_vel_mm_min[a] = std::abs(vel) * 60.0;
        } else {
            telemetry_.axis_vel_mm_min[a] = 0.0;
        }
    }

    if (const int master = config_.gantryMasterAxis(); master >= 0) {
        ms.delta_steps[3] = ms.delta_steps[master];
        telemetry_.axis_vel_mm_min[3] = telemetry_.axis_vel_mm_min[master];
    }

    if (any_axis_moving) {
        if (telemetry_.simulator_mode) {
            telemetry_.tick_counter += 25;
        } else if (!board_->sendSlice(board::SliceStream::Internal, ms, config_.limit_override)) {
            faultLocked("USB write failed while jogging");
        }
    } else {
        cont_jog_active_           = false;
        telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        if (telemetry_.state == ControllerState::Jogging) {
            telemetry_.state = (telemetry_.pending_tool_number > 0)
                                   ? ControllerState::ToolChangeHold
                                   : ControllerState::Idle;
        }
        if (board_->isOpen() && !telemetry_.simulator_mode) {
            board_->setOutputsAfterMotion(telemetry_.outputs_bitmask, config_.limit_override);
        }
    }
}

// ============================================================================
// Spindle / torch / outputs
// ============================================================================
void MachineController::setTorchOrSpindle(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled && (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected)) {
        return;
    }
    if (telemetry_.dry_run_mode && enabled) {
        return;
    }
    // SAFETY LOCK: If Huanyang VFD is enabled but not answering, the spindle cannot start
    if (enabled && config_.vfd.enabled && !vfd_controller_.isConnected()) {
        faultLocked("Huanyang VFD is not responding (offline). Spindle cannot start.");
        return;
    }
    manual_spindle_requested_ = enabled;
    if (enabled) {
        telemetry_.torch_or_spindle_on = true;
        telemetry_.outputs_bitmask |= core::OutputBit::OUT1_TorchOrSpindle;
        if (config_.spindle_ramp_up_s <= 0.01) {
            spindle_ramp_pct_ = 100.0;
            telemetry_.spindle_speed_pct = 100.0f;
            telemetry_.spindle_ramping_up = false;
        } else {
            telemetry_.spindle_ramping_up = (spindle_ramp_pct_ < 99.9);
        }
        telemetry_.spindle_ramping_down = false;
    } else {
        telemetry_.spindle_ramping_up = false;
        if (config_.spindle_ramp_down_s <= 0.01 || spindle_ramp_pct_ <= 0.1) {
            spindle_ramp_pct_ = 0.0;
            telemetry_.spindle_speed_pct = 0.0f;
            telemetry_.spindle_ramping_down = false;
            telemetry_.torch_or_spindle_on = false;
            telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        } else {
            telemetry_.spindle_ramping_down = true;
        }
    }
    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
    }
    // The VFD run/stop command follows within one worker tick (see updateSpindleLocked)
}

void MachineController::setAuxOutput(uint8_t out_index_1_to_4, bool enabled) {
    if (out_index_1_to_4 < 1 || out_index_1_to_4 > 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled && (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected)) {
        return;
    }
    if (enabled && out_index_1_to_4 == 1) {
        if (telemetry_.dry_run_mode) return;
        if (config_.vfd.enabled && !vfd_controller_.isConnected()) {
            faultLocked("Huanyang VFD is not responding (offline). Spindle cannot start.");
            return;
        }
    }
    const uint8_t bit = static_cast<uint8_t>(1U << (out_index_1_to_4 - 1));
    if (enabled) {
        telemetry_.outputs_bitmask |= bit;
    } else {
        telemetry_.outputs_bitmask &= ~bit;
    }
    telemetry_.torch_or_spindle_on = (telemetry_.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0;
    if (out_index_1_to_4 == 1) {
        manual_spindle_requested_ = enabled;
    }
    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
    }
}

void MachineController::setSpindlePwm(uint16_t duty) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_spindle_pwm_duty_    = std::min<uint16_t>(duty, 1000);
    telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;

    const uint32_t max_rpm = (config_.vfd.enabled && config_.vfd.max_rpm > 0) ? config_.vfd.max_rpm : 24000;
    telemetry_.commanded_spindle_rpm   = clampRpm((static_cast<uint32_t>(target_spindle_pwm_duty_) * max_rpm) / 1000);
    telemetry_.spindle_override_active = (telemetry_.commanded_spindle_rpm != telemetry_.program_spindle_rpm);

    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setSpindlePwm(telemetry_.spindle_pwm_duty, config_.limit_override);
    }
    if (config_.vfd.enabled) {
        vfd_controller_.setRpm(telemetry_.commanded_spindle_rpm);
    }
}

void MachineController::setSpindleSpeedRpm(uint32_t rpm) {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint32_t max_limit = (config_.vfd.enabled && config_.vfd.max_rpm > 0) ? config_.vfd.max_rpm : 24000;
    const uint32_t min_limit = std::min<uint32_t>(
        (config_.vfd.enabled && config_.vfd.min_rpm > 0) ? config_.vfd.min_rpm : 1000, max_limit);
    const uint32_t clamped_rpm = std::clamp(rpm, min_limit, max_limit);

    telemetry_.commanded_spindle_rpm   = clampRpm(clamped_rpm);
    telemetry_.spindle_override_active = (telemetry_.commanded_spindle_rpm != telemetry_.program_spindle_rpm);

    // 0-1000 duty cycle for the 0-10V AVI output on the RnR USB board
    target_spindle_pwm_duty_    = rpmToDuty(clamped_rpm, max_limit);
    telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;

    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setSpindlePwm(telemetry_.spindle_pwm_duty, config_.limit_override);
    }
    if (config_.vfd.enabled) {
        vfd_controller_.setRpm(clampRpm(clamped_rpm));
    }
}

void MachineController::resetSpindleSpeedOverride() {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.spindle_override_active = false;
    telemetry_.commanded_spindle_rpm   = telemetry_.program_spindle_rpm;

    const uint32_t max_limit = (config_.vfd.enabled && config_.vfd.max_rpm > 0) ? config_.vfd.max_rpm : 24000;
    target_spindle_pwm_duty_    = rpmToDuty(telemetry_.commanded_spindle_rpm, max_limit);
    telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;

    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setSpindlePwm(telemetry_.spindle_pwm_duty, config_.limit_override);
    }
    if (config_.vfd.enabled) {
        vfd_controller_.setRpm(telemetry_.commanded_spindle_rpm);
    }
}

void MachineController::setOperationMode(OperationMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked()) {
        telemetry_.active_fault_message = "Operation mode cannot change while the machine is moving";
        return;
    }
    config_.operation_mode = mode;
    telemetry_.operation_mode = mode;
}

OperationMode MachineController::getOperationMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_.operation_mode;
}

void MachineController::setLaserPowerOverride(float percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    laser_power_override_pct_ = std::clamp(percent, 10.0f, 200.0f);
    telemetry_.laser_power_override_pct = laser_power_override_pct_;
}

void MachineController::setAirAssist(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled && (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected)) {
        return;
    }
    if (enabled) {
        telemetry_.outputs_bitmask |= core::OutputBit::OUT3_FloodCoolant;
    } else {
        telemetry_.outputs_bitmask &= ~core::OutputBit::OUT3_FloodCoolant;
    }
    telemetry_.laser_air_assist_active = enabled;
    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
    }
}

void MachineController::setRedPointer(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled) {
        if (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected) {
            return;
        }
        // OUT2 is the red pointer only on a laser; on a mill/plasma it is the spindle direction output
        if (config_.is_mill || config_.operation_mode != OperationMode::Laser) {
            return;
        }
    }
    if (enabled) {
        telemetry_.outputs_bitmask |= core::OutputBit::OUT2_SpindleDir;
    } else {
        telemetry_.outputs_bitmask &= ~core::OutputBit::OUT2_SpindleDir;
    }
    telemetry_.laser_red_pointer_active = enabled;
    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
    }
}

void MachineController::testFireLaserPulse(float duration_ms, float power_pct) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected ||
        telemetry_.dry_run_mode || motionActiveLocked())
    {
        return;
    }
    if (config_.is_mill || config_.operation_mode != OperationMode::Laser) {
        return;
    }
    if ((config_.laser.door_interlock_pin >= 1 && !telemetry_.laser_door_closed) ||
        (config_.laser.chiller_flow_pin >= 1 && !telemetry_.laser_chiller_flow_ok) ||
        (config_.laser.laser_fault_pin >= 1 && telemetry_.laser_fault_active))
    {
        telemetry_.active_fault_message = "Laser test pulse blocked: safety interlock not satisfied (door / chiller / PSU)";
        return;
    }
    pulse_active_ = true;
    pulse_timer_s_ = 0.0;
    pulse_duration_s_ = std::clamp(static_cast<double>(duration_ms) / 1000.0, 0.01, 2.0);
    pulse_power_pct_ = std::clamp(power_pct, 1.0f, 100.0f);

    float duty_fraction = config_.laser.min_duty_pct + (pulse_power_pct_ / 100.0f) * (config_.laser.max_duty_pct - config_.laser.min_duty_pct);
    uint16_t duty = static_cast<uint16_t>(std::clamp(duty_fraction * 10.0f, 0.0f, 1000.0f));

    telemetry_.outputs_bitmask |= core::OutputBit::OUT1_TorchOrSpindle;
    telemetry_.torch_or_spindle_on = true;
    telemetry_.spindle_pwm_duty = duty;
    telemetry_.live_laser_power_pct = pulse_power_pct_;

    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setSpindlePwm(duty, config_.limit_override);
        board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
    }
}

void MachineController::frameBoundingBox() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Framing")) return;
    if (program_->segments.empty()) return;

    const auto& bmin = program_->bbox_min_mm;
    const auto& bmax = program_->bbox_max_mm;
    if (bmax[0] - bmin[0] < 0.1 && bmax[1] - bmin[1] < 0.1) {
        return;
    }

    double off_x = 0.0;
    double off_y = 0.0;
    uint8_t out_mask = 0;
    uint16_t framing_duty = 0;
    const bool laser = (config_.operation_mode == OperationMode::Laser && !config_.is_mill);

    if (laser && config_.laser.framing_mode == LaserFramingMode::RedDotPointer) {
        out_mask = core::OutputBit::OUT2_SpindleDir;
        off_x = config_.laser.red_dot_offset_x_mm;
        off_y = config_.laser.red_dot_offset_y_mm;
    } else if (laser && config_.laser.framing_mode == LaserFramingMode::LowPowerBeam) {
        out_mask = core::OutputBit::OUT1_TorchOrSpindle;
        framing_duty = static_cast<uint16_t>(std::clamp(config_.laser.framing_power_pct * 10.0f, 1.0f, 100.0f));
    } else if (laser && config_.laser.framing_mode == LaserFramingMode::Both) {
        out_mask = core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir;
        framing_duty = static_cast<uint16_t>(std::clamp(config_.laser.framing_power_pct * 10.0f, 1.0f, 100.0f));
        off_x = config_.laser.red_dot_offset_x_mm;
        off_y = config_.laser.red_dot_offset_y_mm;
    }

    if ((out_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0) {
        if (telemetry_.dry_run_mode) {
            out_mask &= ~core::OutputBit::OUT1_TorchOrSpindle;
            framing_duty = 0;
        } else if ((config_.laser.door_interlock_pin >= 1 && !telemetry_.laser_door_closed) ||
                   (config_.laser.chiller_flow_pin >= 1 && !telemetry_.laser_chiller_flow_ok) ||
                   (config_.laser.laser_fault_pin >= 1 && telemetry_.laser_fault_active)) {
            telemetry_.active_fault_message = "Framing with the beam blocked: laser safety interlock not satisfied";
            return;
        }
    }

    const double z_plane = telemetry_.work_pos_mm[2];
    const double speed = std::max(500.0, config_.laser.framing_speed_mm_min);

    std::vector<std::array<double, 4>> pts = {
        {bmin[0] + off_x, bmin[1] + off_y, z_plane, telemetry_.work_pos_mm[3]},
        {bmax[0] + off_x, bmin[1] + off_y, z_plane, telemetry_.work_pos_mm[3]},
        {bmax[0] + off_x, bmax[1] + off_y, z_plane, telemetry_.work_pos_mm[3]},
        {bmin[0] + off_x, bmax[1] + off_y, z_plane, telemetry_.work_pos_mm[3]},
        {bmin[0] + off_x, bmin[1] + off_y, z_plane, telemetry_.work_pos_mm[3]}
    };
    for (const auto& p : pts) {
        if (!checkLinearMoveLocked(telemetry_.work_pos_mm, p)) return;
    }

    MachineConfig manual_cfg = config_;
    manual_cfg.is_mill = true; // Framing moves: no laser Z lock / output rewriting
    slice_queue_.clear();
    auto rapid_start = planner::TrajectoryPlanner::sliceLinearMove(telemetry_.work_pos_mm, pts[0], speed, manual_cfg, 0);
    slice_queue_.insert(slice_queue_.end(), rapid_start.begin(), rapid_start.end());
    for (size_t i = 0; i < 4; ++i) {
        auto edge_slices = planner::TrajectoryPlanner::sliceLinearMove(pts[i], pts[i + 1], speed, manual_cfg, out_mask);
        slice_queue_.insert(slice_queue_.end(), edge_slices.begin(), edge_slices.end());
    }

    if (framing_duty > 0 && board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setSpindlePwm(framing_duty, config_.limit_override);
    }
    telemetry_.spindle_pwm_duty = framing_duty;
    // Framing runs as a manual move: job-only logic (dynamic laser power, THC) must not touch it
    telemetry_.state = ControllerState::Jogging;
}

// ============================================================================
// Hardware stop / settle helpers
// ============================================================================
void MachineController::beginSettleLocked() {
    settle_active_ = true;
    settle_board_idle_seen_ = false;
    settle_deadline_ = Clock::now() + kSettleTimeout;
}

bool MachineController::settledLocked() const {
    return settle_board_idle_seen_ || Clock::now() >= settle_deadline_;
}

void MachineController::stopMotionForEventLocked() {
    slice_queue_.clear();
    job_planner_.reset();
    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->cancelMotion();
        in_flight_seqs_.clear();
        beginSettleLocked();
    }
    last_sent_target_valid_ = false;
    last_sent_speed_mm_s_ = 0.0;
}

// ============================================================================
// Homing (per axis: back off switch if needed -> seek switch -> set zero -> pull off)
// ============================================================================
bool MachineController::homeSwitchActiveLocked(size_t axis) const {
    const auto& ax = config_.axes[axis];
    if (telemetry_.simulator_mode) {
        // Simulated home switch located at machine zero
        return ax.homing_dir_negative ? (telemetry_.machine_pos_mm[axis] <= 0.0)
                                      : (telemetry_.machine_pos_mm[axis] >= 0.0);
    }
    return evaluatePinStateLocked(ax.home_pin, ax.home_invert);
}

void MachineController::startNextHomingAxisLocked() {
    auto finishHoming = [&](const std::string& message) {
        homing_axis_queue_.clear();
        homing_phase_ = HomingPhase::None;
        slice_queue_.clear();
        if (board_->isOpen() && !telemetry_.simulator_mode) {
            board_->configure(config_, false);
        }
        telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        telemetry_.state = ControllerState::Idle;
        if (!message.empty()) telemetry_.active_fault_message = message;
    };

    if (homing_axis_queue_.empty()) {
        finishHoming("");
        return;
    }

    active_homing_axis_ = homing_axis_queue_.front();
    homing_axis_queue_.pop_front();
    const size_t idx = static_cast<size_t>(active_homing_axis_);
    const auto& ax = config_.axes[idx];

    if (!telemetry_.simulator_mode && ax.home_pin == 0) {
        finishHoming(std::string("Cannot home ") + kAxisNames[idx] + ": no home switch input is configured");
        return;
    }

    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->configure(config_, true);
        board_->armHomeSwitchStop(static_cast<Axis>(idx));
    }

    telemetry_.axis_homed[idx] = false;
    homing_switch_event_ = false;
    const double dir = ax.homing_dir_negative ? -1.0 : 1.0;
    std::array<double, 4> end_w = telemetry_.work_pos_mm;
    double feed = ax.homing_feed_mm_min;
    if (homeSwitchActiveLocked(idx)) {
        // Already on the switch: back off first so the switch edge is found from the correct side
        homing_phase_ = HomingPhase::BackOff;
        end_w[idx] -= dir * std::min(25.0, ax.home_pulloff_mm * 2.0 + 5.0);
        feed = std::max(60.0, ax.homing_feed_mm_min * 0.5);
    } else {
        homing_phase_ = HomingPhase::Seek;
        end_w[idx] += dir * ax.homing_max_travel_mm;
    }
    slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(telemetry_.work_pos_mm, end_w, feed, config_, 0);
    telemetry_.state = ControllerState::Homing;
}

void MachineController::homingTickLocked() {
    const size_t idx = static_cast<size_t>(active_homing_axis_);
    const auto& ax = config_.axes[idx];
    const bool sw = homeSwitchActiveLocked(idx);

    if (!homing_switch_event_) {
        if ((homing_phase_ == HomingPhase::BackOff && !sw) || (homing_phase_ == HomingPhase::Seek && sw)) {
            homing_switch_event_ = true;
            stopMotionForEventLocked();
        }
    }
    if (!slice_queue_.empty()) return;
    if (board_->isOpen() && !telemetry_.simulator_mode) {
        if (settle_active_) {
            if (!settledLocked()) return;
            settle_active_ = false;
            if (feedback_validated_) {
                std::array<double, 4> m{};
                for (size_t a = 0; a < 4; ++a) m[a] = static_cast<double>(hw_live_steps_[a]) / config_.axes[a].steps_per_mm;
                telemetry_.machine_pos_mm = m;
                for (size_t a = 0; a < 4; ++a) telemetry_.work_pos_mm[a] = m[a] - telemetry_.work_offset_mm[a];
            }
        } else if (!in_flight_seqs_.empty()) {
            return;
        }
    }

    auto abortHoming = [&](const std::string& msg) {
        homing_axis_queue_.clear();
        startNextHomingAxisLocked(); // Restores the board pinout and returns to Idle
        telemetry_.active_fault_message = msg;
    };

    const double dir = ax.homing_dir_negative ? -1.0 : 1.0;
    switch (homing_phase_) {
        case HomingPhase::BackOff: {
            if (!homing_switch_event_) {
                abortHoming(std::string("Homing ") + kAxisNames[idx] + " failed: could not move off the home switch");
                return;
            }
            homing_switch_event_ = false;
            homing_phase_ = HomingPhase::Seek;
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[idx] += dir * ax.homing_max_travel_mm;
            slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(telemetry_.work_pos_mm, end_w, ax.homing_feed_mm_min, config_, 0);
            break;
        }
        case HomingPhase::Seek: {
            if (!homing_switch_event_) {
                char buf[160];
                std::snprintf(buf, sizeof(buf), "Homing %s failed: home switch not found within %.0f mm", kAxisNames[idx], ax.homing_max_travel_mm);
                abortHoming(buf);
                return;
            }
            homing_switch_event_ = false;
            if (idx == 2) {
                // New machine Z frame: the tool-setter reference contact must be measured again
                tool_ref_contact_valid_ = false;
                telemetry_.tool_reference_measured = false;
            }
            // The switch position defines machine zero for this axis
            std::array<double, 4> m = telemetry_.machine_pos_mm;
            m[idx] = 0.0;
            setMachinePositionLocked(m);
            homing_phase_ = HomingPhase::PullOff;
            if (ax.home_pulloff_mm > 1e-3) {
                std::array<double, 4> end_w = telemetry_.work_pos_mm;
                end_w[idx] -= dir * ax.home_pulloff_mm;
                slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(
                    telemetry_.work_pos_mm, end_w, std::max(60.0, ax.homing_feed_mm_min * 0.5), config_, 0);
            }
            break;
        }
        case HomingPhase::PullOff:
            telemetry_.axis_homed[idx] = true;
            startNextHomingAxisLocked();
            break;
        case HomingPhase::None:
            startNextHomingAxisLocked();
            break;
    }
}

void MachineController::homeAxis(core::Axis axis) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Homing")) return;
    if (static_cast<size_t>(axis) >= 4) return;

    if (config_.scripting.enabled && config_.scripting.home_script_enabled && script_runner_) {
        std::string resolved = scripting::resolveScriptPath(config_.scripting.home_script_path);
        if (!resolved.empty()) {
            script_runner_->runFileAsync(resolved);
            return;
        }
    }

    homing_axis_queue_.clear();
    homing_axis_queue_.push_back(axis);
    startNextHomingAxisLocked();
}

void MachineController::homeAllAxes() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Homing")) return;

    if (config_.scripting.enabled && config_.scripting.home_script_enabled && script_runner_) {
        std::string resolved = scripting::resolveScriptPath(config_.scripting.home_script_path);
        if (!resolved.empty()) {
            script_runner_->runFileAsync(resolved);
            return;
        }
    }

    homing_axis_queue_.clear();
    // Safe CNC Homing Order: Z Up first, then Y, then X
    homing_axis_queue_.push_back(core::Axis::Z);
    homing_axis_queue_.push_back(core::Axis::Y);
    homing_axis_queue_.push_back(core::Axis::X);
    startNextHomingAxisLocked();
}

// ============================================================================
// Probing (G31)
// ============================================================================
bool MachineController::beginProbeMoveLocked(core::Axis axis, double signed_travel_mm, double feed_mm_min,
                                         const double* sim_contact_work)
{
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4 || std::abs(signed_travel_mm) < 1e-6) return false;
    if (!telemetry_.simulator_mode && config_.probe.probe_pin == 0) {
        telemetry_.active_fault_message = "Probing rejected: no probe input is configured";
        return false;
    }
    if (telemetry_.probe_contact_active) {
        telemetry_.active_fault_message = "Probing rejected: probe is already in contact - move it clear first";
        return false;
    }

    const std::array<double, 4> start_w = telemetry_.work_pos_mm;
    std::array<double, 4> end_w = start_w;
    if (telemetry_.simulator_mode) {
        // Simulator: contact is emulated where the move ends
        end_w[idx] = sim_contact_work ? *sim_contact_work : (start_w[idx] + signed_travel_mm * 0.5);
    } else {
        end_w[idx] += signed_travel_mm;
    }
    if (!checkLinearMoveLocked(start_w, end_w)) return false;

    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->armProbe(static_cast<Axis>(idx));
    }
    probe_latched_at_arm_   = hw_latched_steps_;
    probe_contact_captured_ = false;
    probe_move_is_search_   = true;
    active_probe_axis_      = axis;
    active_probe_pos_dir_   = (signed_travel_mm >= 0.0);
    telemetry_.last_probe_hit = false;
    slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(start_w, end_w, std::max(5.0, feed_mm_min), config_, 0);
    telemetry_.state = ControllerState::Probing;
    return true;
}

void MachineController::probingTickLocked() {
    const bool hw = board_->isOpen() && !telemetry_.simulator_mode;
    if (probe_move_is_search_ && !probe_contact_captured_ && telemetry_.probe_contact_active) {
        probe_contact_captured_ = true;
        std::array<double, 4> m = telemetry_.machine_pos_mm;
        if (hw && feedback_validated_) {
            // Prefer the hardware-latched trigger position; fall back to the live step counters
            const bool latched_new = (hw_latched_steps_ != probe_latched_at_arm_);
            for (size_t a = 0; a < 4; ++a) {
                const int32_t steps = latched_new ? hw_latched_steps_[a] : hw_live_steps_[a];
                m[a] = static_cast<double>(steps) / config_.axes[a].steps_per_mm;
            }
        }
        probe_contact_machine_ = m;
        stopMotionForEventLocked();
    }
    if (!slice_queue_.empty()) return;
    if (hw) {
        if (settle_active_) {
            if (!settledLocked()) return;
            settle_active_ = false;
            if (feedback_validated_) {
                std::array<double, 4> m{};
                for (size_t a = 0; a < 4; ++a) m[a] = static_cast<double>(hw_live_steps_[a]) / config_.axes[a].steps_per_mm;
                telemetry_.machine_pos_mm = m;
                for (size_t a = 0; a < 4; ++a) telemetry_.work_pos_mm[a] = m[a] - telemetry_.work_offset_mm[a];
            }
        } else if (!in_flight_seqs_.empty()) {
            return;
        }
    }
    if (probe_move_is_search_) {
        onProbeMoveFinishedLocked(probe_contact_captured_ || telemetry_.simulator_mode);
    } else {
        continueHoleCenterLocked();
    }
}

void MachineController::onProbeMoveFinishedLocked(bool hit) {
    const std::array<double, 4> contact_m = probe_contact_captured_ ? probe_contact_machine_ : telemetry_.machine_pos_mm;
    std::array<double, 4> contact_w{};
    for (size_t a = 0; a < 4; ++a) contact_w[a] = contact_m[a] - telemetry_.work_offset_mm[a];
    telemetry_.last_probed_work_pos_mm = contact_w;
    telemetry_.last_probe_hit = hit;
    telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    const size_t p_idx = static_cast<size_t>(active_probe_axis_);
    const double retract = std::max(1.0, config_.probe.retract_mm);
    const uint8_t ws = telemetry_.active_workspace;

    (void)ws;
    // `offset` is the effective offset (tip frame); the stored WCS offset excludes the tool length
    auto setAxisOffset = [&](size_t a, double offset) {
        setActiveWcsAxisLocked(a, offset - (a == 2 ? activeTloLocked() : 0.0));
    };

    if (active_probe_mode_ == ProbeMode::Raw) {
        // G38.2-style: just report. Contact is still made at this point (no retract yet).
        active_probe_mode_ = ProbeMode::None;
        telemetry_.state = probe_return_state_;
        return;
    }
    if (!hit) {
        active_probe_mode_ = ProbeMode::None;
        telemetry_.active_fault_message = "Probing failed: no probe contact within the search distance";
        telemetry_.state = ControllerState::Idle;
        return;
    }

    switch (active_probe_mode_) {
        case ProbeMode::FloatingHeadZ: {
            // Zero Work Z at plate contact and retract to pierce height
            setAxisOffset(2, contact_m[2]);
            active_probe_mode_ = ProbeMode::None;
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[2] = kFloatingHeadPierceHeightMm;
            if (!queueLinearMoveLocked(end_w, 600.0, ControllerState::Jogging)) telemetry_.state = ControllerState::Idle;
            break;
        }
        case ProbeMode::ToolSetterZ: {
            // Work Z = plate thickness at the contact point, then retract +Z
            const double plate_h = config_.probe.plate_thickness_mm;
            setAxisOffset(2, contact_m[2] - plate_h);
            active_probe_mode_ = ProbeMode::None;
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[2] = plate_h + retract;
            if (!queueLinearMoveLocked(end_w, 600.0, ControllerState::Jogging)) telemetry_.state = ControllerState::Idle;
            break;
        }
        case ProbeMode::EdgeSingleAxis: {
            // Compensate for probe tip radius: probing +dir, the edge is +radius beyond the tip centre
            const double radius = 0.5 * config_.probe.tool_diameter_mm;
            const double edge_pos = active_probe_pos_dir_ ? -radius : radius; // Tip centre in work coords
            setAxisOffset(p_idx, contact_m[p_idx] - edge_pos);
            active_probe_mode_ = ProbeMode::None;
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[p_idx] += active_probe_pos_dir_ ? -retract : retract;
            if (!queueLinearMoveLocked(end_w, 600.0, ControllerState::Jogging)) telemetry_.state = ControllerState::Idle;
            break;
        }
        case ProbeMode::HoleCenterXY:
            continueHoleCenterLocked();
            break;
        case ProbeMode::ToolLength: {
            // Tool setter contact (machine Z). The reference tool defines length 0; every other tool's
            // length is how much higher (+) or lower (-) it touches the setter than the reference.
            const uint16_t tool = measuring_tool_;
            active_probe_mode_ = ProbeMode::None;
            double length = 0.0;
            if (tool == config_.tool_table.reference_tool) {
                tool_ref_contact_z_     = contact_m[2];
                tool_ref_contact_valid_ = true;
                telemetry_.tool_reference_measured = true;
            } else {
                length = contact_m[2] - tool_ref_contact_z_;
            }
            config_.tool_table.tools[tool].length_mm = length;
            config_.tool_table_revision++;
            if (telemetry_.active_tlo_tool == tool) {
                updateActiveToolLengthLocked();
                refreshWorkFrameLocked();
            }
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[2] += retract;
            if (!queueLinearMoveLocked(end_w, 600.0, ControllerState::Jogging)) telemetry_.state = ControllerState::Idle;
            break;
        }
        default:
            active_probe_mode_ = ProbeMode::None;
            telemetry_.state = ControllerState::Idle;
            break;
    }
}

void MachineController::continueHoleCenterLocked() {
    ++hole_step_;
    const double feed = std::max(50.0, config_.probe.fast_feed_mm_min);
    auto fail = [&]() {
        active_probe_mode_ = ProbeMode::None;
        slice_queue_.clear();
        telemetry_.state = ControllerState::Idle;
        if (telemetry_.active_fault_message.empty()) {
            telemetry_.active_fault_message = "Hole centre probing aborted";
        }
    };
    auto probe = [&](core::Axis axis, double dir) {
        const size_t a = static_cast<size_t>(axis);
        const double sim_contact = hole_start_work_[a] + dir * (hole_search_mm_ * 0.8);
        if (!beginProbeMoveLocked(axis, dir * hole_search_mm_, feed, &sim_contact)) fail();
    };
    auto moveTo = [&](size_t a, double value) {
        std::array<double, 4> end_w = telemetry_.work_pos_mm;
        end_w[a] = value;
        probe_move_is_search_ = false;
        if (!queueLinearMoveLocked(end_w, feed, ControllerState::Probing)) {
            fail();
        } else if (slice_queue_.empty()) {
            continueHoleCenterLocked(); // Already there
        }
    };

    switch (hole_step_) {
        case 0: probe(core::Axis::X, -1.0); break;
        case 1: hole_contacts_work_[0] = telemetry_.last_probed_work_pos_mm[0]; moveTo(0, hole_start_work_[0]); break;
        case 2: probe(core::Axis::X, +1.0); break;
        case 3:
            hole_contacts_work_[1] = telemetry_.last_probed_work_pos_mm[0];
            moveTo(0, 0.5 * (hole_contacts_work_[0] + hole_contacts_work_[1]));
            break;
        case 4: probe(core::Axis::Y, -1.0); break;
        case 5: hole_contacts_work_[2] = telemetry_.last_probed_work_pos_mm[1]; moveTo(1, hole_start_work_[1]); break;
        case 6: probe(core::Axis::Y, +1.0); break;
        case 7:
            hole_contacts_work_[3] = telemetry_.last_probed_work_pos_mm[1];
            moveTo(1, 0.5 * (hole_contacts_work_[2] + hole_contacts_work_[3]));
            break;
        default: {
            // Tool is now at the measured centre: make it work X0 Y0
            for (size_t a = 0; a < 2; ++a) {
                setActiveWcsAxisLocked(a, wcsOffsetForLocked(a, 0.0));
            }
            active_probe_mode_ = ProbeMode::None;
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            telemetry_.state = ControllerState::Idle;
            break;
        }
    }
}

void MachineController::probeAxis(core::Axis axis, double max_travel_mm, double feed_mm_min) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(true, "Probing")) return;
    active_probe_mode_ = ProbeMode::FloatingHeadZ;
    double travel = max_travel_mm;
    if (std::abs(travel) < 1e-6) travel = -std::max(5.0, config_.probe.max_travel_mm);
    if (!beginProbeMoveLocked(axis, travel, feed_mm_min > 0.0 ? feed_mm_min : config_.probe.fast_feed_mm_min)) {
        active_probe_mode_ = ProbeMode::None;
    }
}

void MachineController::probeZWithToolSetter() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(true, "Probing")) return;
    active_probe_mode_ = ProbeMode::ToolSetterZ;
    const double travel = std::max(5.0, config_.probe.max_travel_mm);
    if (!beginProbeMoveLocked(core::Axis::Z, -travel, std::max(30.0, config_.probe.fast_feed_mm_min))) {
        active_probe_mode_ = ProbeMode::None;
    }
}

void MachineController::probeWorkAxisEdge(core::Axis axis, bool positive_dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Probing")) return;
    active_probe_mode_ = ProbeMode::EdgeSingleAxis;
    const double travel = std::max(5.0, config_.probe.max_travel_mm);
    if (!beginProbeMoveLocked(axis, positive_dir ? travel : -travel, std::max(30.0, config_.probe.fast_feed_mm_min))) {
        active_probe_mode_ = ProbeMode::None;
    }
}

void MachineController::probeHoleCenterXY(double approx_diameter_mm) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Probing")) return;
    if (!telemetry_.simulator_mode && config_.probe.probe_pin == 0) {
        telemetry_.active_fault_message = "Probing rejected: no probe input is configured";
        return;
    }
    const double r = std::max(3.0, approx_diameter_mm * 0.5 - config_.probe.tool_diameter_mm * 0.5);
    hole_search_mm_   = r + std::max(2.0, 0.25 * r); // Search a bit beyond the expected wall
    hole_start_work_  = telemetry_.work_pos_mm;
    hole_step_        = -1;
    active_probe_mode_ = ProbeMode::HoleCenterXY;
    continueHoleCenterLocked();
}

void MachineController::probeRaw(core::Axis axis, double signed_travel_mm, double feed_mm_min) {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.last_probe_hit = false; // Reset first so a rejected request reads as "no contact"
    if (!readyForCommandLocked(true, "Probing")) return;
    const ControllerState return_state = telemetry_.state;
    active_probe_mode_ = ProbeMode::Raw;
    if (!beginProbeMoveLocked(axis, signed_travel_mm, feed_mm_min)) {
        active_probe_mode_ = ProbeMode::None;
        return;
    }
    probe_return_state_ = return_state;
}

// ============================================================================
// Surface probing (height map)
// ============================================================================
void MachineController::startSurfaceProbing(double min_x, double max_x, double min_y, double max_y, int points_x, int points_y, double smoothness) {
    if (points_x < 2 || points_y < 2 || points_x > 100 || points_y > 100) return;
    if (!(max_x > min_x) || !(max_y > min_y)) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!readyForCommandLocked(true, "Surface probing")) return;
        if (!telemetry_.simulator_mode && config_.probe.probe_pin == 0) {
            telemetry_.active_fault_message = "Surface probing rejected: no probe input is configured";
            return;
        }
        surface_probing_cancel_requested_.store(false);
        telemetry_.surface_probing_active = true;
        telemetry_.surface_probe_current_point = 0;
        telemetry_.surface_probe_total_points = points_x * points_y;
    }

    if (surface_probing_thread_.joinable()) {
        surface_probing_thread_.join();
    }

    surface_probing_thread_ = std::thread([this, min_x, max_x, min_y, max_y, points_x, points_y, smoothness]() {
        struct GridPt { int i; int j; double x; double y; };
        std::vector<GridPt> grid;
        grid.reserve(static_cast<size_t>(points_x * points_y));

        const double dx = (max_x - min_x) / static_cast<double>(points_x - 1);
        const double dy = (max_y - min_y) / static_cast<double>(points_y - 1);
        for (int j = 0; j < points_y; ++j) {
            const double py = min_y + static_cast<double>(j) * dy;
            if (j % 2 == 0) {
                for (int i = 0; i < points_x; ++i) grid.push_back({i, j, min_x + static_cast<double>(i) * dx, py});
            } else {
                for (int i = points_x - 1; i >= 0; --i) grid.push_back({i, j, min_x + static_cast<double>(i) * dx, py});
            }
        }

        SurfaceHeightMap result_map;
        result_map.min_x = min_x;
        result_map.max_x = max_x;
        result_map.min_y = min_y;
        result_map.max_y = max_y;
        result_map.points_x = points_x;
        result_map.points_y = points_y;
        result_map.smoothness = smoothness;
        result_map.grid_z.assign(static_cast<size_t>(points_x * points_y), 0.0);

        double safe_clearance = 3.0, max_depth = 3.0, search_feed = 300.0, z_feed = 600.0, xy_feed = 1200.0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            safe_clearance = std::max(1.0, config_.probe.surface_clearance_z_mm);
            max_depth   = std::max(0.5, config_.probe.surface_max_depth_below_zero_mm);
            search_feed = std::max(20.0, config_.probe.fast_feed_mm_min);
            z_feed      = planner::TrajectoryPlanner::axisVelocityLimit(config_, 2) * 60.0;
            xy_feed     = std::max(300.0, planner::TrajectoryPlanner::axisVelocityLimit(config_, 0) * 60.0 * 0.5);
        }

        // Issue a step under the controller lock, re-checking cancellation INSIDE the lock so a cancel can
        // never be followed by new motion.
        auto issue = [this](const std::function<bool()>& fn) -> bool {
            std::lock_guard<std::mutex> lock(mutex_);
            if (surface_probing_cancel_requested_.load()) return false;
            if (telemetry_.state != ControllerState::Idle && telemetry_.state != ControllerState::ToolChangeHold) return false;
            return fn();
        };
        auto waitDone = [this]() -> bool {
            while (running_thread_.load()) {
                if (surface_probing_cancel_requested_.load()) return false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected) return false;
                    if (!motionActiveLocked() && slice_queue_.empty() && in_flight_seqs_.empty()) return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        };

        bool ok = true;
        // 1. Initial safe Z lift if currently below safe_clearance
        ok = issue([&]() {
            std::array<double, 4> up = telemetry_.work_pos_mm;
            if (up[2] >= safe_clearance) return true;
            up[2] = safe_clearance;
            return queueLinearMoveLocked(up, z_feed, ControllerState::Jogging);
        }) && waitDone();

        for (size_t pt_idx = 0; ok && pt_idx < grid.size(); ++pt_idx) {
            const auto& pt = grid[pt_idx];

            // 2a. Move XY to probe point at safe clearance
            ok = issue([&]() {
                std::array<double, 4> xy = telemetry_.work_pos_mm;
                xy[0] = pt.x; xy[1] = pt.y; xy[2] = safe_clearance;
                return queueLinearMoveLocked(xy, xy_feed, ControllerState::Jogging);
            }) && waitDone();
            if (!ok) break;

            // 2b. Probe down toward -max_depth
            ok = issue([&]() {
                const double cur_z = telemetry_.work_pos_mm[2];
                double sim_z = 0.0;
                if (telemetry_.simulator_mode) {
                    const double nx = static_cast<double>(pt.i) / (points_x - 1);
                    const double ny = static_cast<double>(pt.j) / (points_y - 1);
                    sim_z = 0.22 * std::sin(nx * 3.14159) - 0.16 * std::cos(ny * 3.14159) + 0.12 * (nx - 0.5);
                    sim_z = std::clamp(sim_z, -max_depth + 0.1, safe_clearance - 0.1);
                }
                active_probe_mode_  = ProbeMode::Raw;
                probe_return_state_ = ControllerState::Idle;
                if (!beginProbeMoveLocked(core::Axis::Z, -max_depth - cur_z, search_feed,
                                          telemetry_.simulator_mode ? &sim_z : nullptr)) {
                    active_probe_mode_ = ProbeMode::None;
                    return false;
                }
                return true;
            }) && waitDone();
            if (!ok) break;

            double measured_z = 0.0;
            bool hit = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                measured_z = telemetry_.last_probed_work_pos_mm[2];
                hit = telemetry_.last_probe_hit;
            }

            // 2c. Retract back to safe clearance
            ok = issue([&]() {
                std::array<double, 4> up = telemetry_.work_pos_mm;
                up[2] = safe_clearance;
                return queueLinearMoveLocked(up, z_feed, ControllerState::Jogging);
            }) && waitDone();
            if (!ok) break;

            if (!hit) {
                std::lock_guard<std::mutex> lock(mutex_);
                telemetry_.active_fault_message = "Surface probing error: no probe contact at grid point (" +
                    std::to_string(pt.i) + "," + std::to_string(pt.j) + "). Aborted for safety.";
                ok = false;
                break;
            }

            result_map.grid_z[static_cast<size_t>(pt.j * points_x + pt.i)] = measured_z;
            std::lock_guard<std::mutex> lock(mutex_);
            telemetry_.surface_probe_current_point = static_cast<int>(pt_idx + 1);
        }

        if (ok && !surface_probing_cancel_requested_.load()) {
            result_map.is_valid = true;
            result_map.recalculateStats();
            applySurfaceHeightMap(result_map);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.surface_probing_active = false;
    });
}

void MachineController::cancelSurfaceProbing() {
    surface_probing_cancel_requested_.store(true);
    std::lock_guard<std::mutex> lock(mutex_);
    if (telemetry_.surface_probing_active &&
        (telemetry_.state == ControllerState::Probing || telemetry_.state == ControllerState::Jogging))
    {
        stopMotionForEventLocked();
        active_probe_mode_ = ProbeMode::None;
        telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        telemetry_.state = ControllerState::Idle;
    }
    // surface_probing_active is cleared by the probing thread itself when it exits
}

void MachineController::applySurfaceHeightMap(const SurfaceHeightMap& map) {
    std::shared_ptr<const gcode::ParseResult> base;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto s = telemetry_.state;
        if (motionActiveLocked() || s == ControllerState::PausedHold || s == ControllerState::ToolChangeHold) {
            telemetry_.active_fault_message = "Height map cannot be changed while a job is active";
            return;
        }
        base = unwarped_program_;
    }

    // Warping can be slow on large programs: do it without holding the controller lock
    std::shared_ptr<gcode::ParseResult> warped;
    if (map.is_valid && !base->segments.empty()) {
        warped = std::make_shared<gcode::ParseResult>(*base);
        warped->segments = planner::TrajectoryPlanner::warpToolpathWithHeightMap(base->segments, map, 2.0);
        bool first = true;
        for (const auto& s : warped->segments) {
            if (s.type == SegmentType::RapidMove && s.machine_axes_mask != 0) continue;
            if (first) {
                warped->bbox_min_mm = {s.start_xyza_mm[0], s.start_xyza_mm[1], s.start_xyza_mm[2]};
                warped->bbox_max_mm = warped->bbox_min_mm;
                first = false;
            }
            for (int i = 0; i < 3; ++i) {
                warped->bbox_min_mm[i] = std::min({warped->bbox_min_mm[i], s.start_xyza_mm[i], s.end_xyza_mm[i]});
                warped->bbox_max_mm[i] = std::max({warped->bbox_max_mm[i], s.start_xyza_mm[i], s.end_xyza_mm[i]});
            }
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (unwarped_program_ != base || motionActiveLocked()) {
        return; // Program reloaded or motion started meanwhile
    }
    surface_height_map_ = map;
    telemetry_.surface_height_map = std::make_shared<const SurfaceHeightMap>(map);
    if (!warped) {
        telemetry_.surface_compensation_active = false;
        return;
    }
    setProgramLocked(warped);
    telemetry_.surface_compensation_active = true;
}

void MachineController::clearSurfaceHeightMap() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto s = telemetry_.state;
    if (motionActiveLocked() || s == ControllerState::PausedHold || s == ControllerState::ToolChangeHold) {
        telemetry_.active_fault_message = "Height map cannot be changed while a job is active";
        return;
    }
    setProgramLocked(unwarped_program_);
    telemetry_.surface_compensation_active = false;
}

bool MachineController::isSurfaceCompensationActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_.surface_compensation_active;
}

SurfaceHeightMap MachineController::getSurfaceHeightMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return surface_height_map_;
}

bool MachineController::getProgramBounds(std::array<double, 3>& out_min, std::array<double, 3>& out_max) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (program_->segments.empty()) return false;
    out_min = program_->bbox_min_mm;
    out_max = program_->bbox_max_mm;
    return true;
}

// ============================================================================
// Program loading & MDI
// ============================================================================
void MachineController::setProgramLocked(std::shared_ptr<const gcode::ParseResult> program) {
    program_ = program ? std::move(program) : std::make_shared<const gcode::ParseResult>();
    telemetry_.total_segments = program_->segments.size();
    job_planner_.reset();
}

std::shared_ptr<const gcode::ParseResult> MachineController::getProgram() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return program_;
}

bool MachineController::loadGCodeString(const std::string& gcode_text) {
    gcode::ParseOptions opts;
    std::array<double, 4> work_offset{};
    SurfaceHeightMap map;
    bool warp = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto s = telemetry_.state;
        if (motionActiveLocked() || s == ControllerState::PausedHold || s == ControllerState::ToolChangeHold) {
            telemetry_.active_fault_message = "Stop the current job before loading a new program";
            return false;
        }
        opts.dwell_p_milliseconds = config_.dwell_p_milliseconds;
        opts.tool_length_mode     = config_.tool_table.mode;
        work_offset = telemetry_.work_offset_mm;
        warp = telemetry_.surface_compensation_active && surface_height_map_.is_valid;
        map = surface_height_map_;
    }

    // Parsing (and warping) large files happens without holding the controller lock
    auto parsed = std::make_shared<gcode::ParseResult>(
        gcode::GCodeParser::parse(gcode_text, {0.0, 0.0, 0.0, 0.0}, 0.3, work_offset, opts));
    std::shared_ptr<const gcode::ParseResult> active = parsed;
    if (warp) {
        auto w = std::make_shared<gcode::ParseResult>(*parsed);
        w->segments = planner::TrajectoryPlanner::warpToolpathWithHeightMap(parsed->segments, map, 2.0);
        active = w;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto s = telemetry_.state;
    if (motionActiveLocked() || s == ControllerState::PausedHold || s == ControllerState::ToolChangeHold) {
        telemetry_.active_fault_message = "Stop the current job before loading a new program";
        return false;
    }
    unwarped_program_ = parsed;
    program_text_ = std::make_shared<const std::string>(gcode_text);
    program_reparse_needed_ = false;
    setProgramLocked(active);
    if (!warp) telemetry_.surface_compensation_active = false;

    telemetry_.active_gcode_line_index = 0;
    telemetry_.active_segment_index    = 0;
    telemetry_.segment_progress_0_to_1 = 0.0f;
    telemetry_.job_elapsed_seconds     = 0.0;
    telemetry_.pending_tool_number     = 0;
    telemetry_.pending_tool_description.clear();
    slice_queue_.clear();

    if (parsed->hasErrors()) {
        telemetry_.active_fault_message = "Program has " + std::to_string(parsed->errors.size()) +
                                          " error(s) and cannot run. " + parsed->firstErrorText();
    } else if (telemetry_.state != ControllerState::EStop && telemetry_.state != ControllerState::Disconnected) {
        telemetry_.active_fault_message.clear();
    }

    // Scan for initial commanded spindle speed (S word or M3 S...)
    for (const auto& seg : parsed->segments) {
        if (seg.spindle_rpm > 0) {
            telemetry_.program_spindle_rpm     = seg.spindle_rpm;
            telemetry_.commanded_spindle_rpm   = seg.spindle_rpm;
            telemetry_.spindle_override_active = false;
            const uint32_t max_rpm = (config_.vfd.enabled && config_.vfd.max_rpm > 0) ? config_.vfd.max_rpm : 24000;
            target_spindle_pwm_duty_ = rpmToDuty(seg.spindle_rpm, max_rpm);
            telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;
            if (config_.vfd.enabled) {
                vfd_controller_.setRpm(seg.spindle_rpm);
            }
            break;
        }
    }

    return !parsed->segments.empty();
}

bool MachineController::executeMdiCommand(const std::string& gcode_line) {
    if (gcode_line.empty()) return false;

    std::vector<gcode::GCodeWord> words;
    std::string tok_err;
    const bool tok_ok = gcode::tokenizeBlock(gcode_line, words, nullptr, &tok_err);

    std::lock_guard<std::mutex> lock(mutex_);
    if (!tok_ok) {
        telemetry_.active_fault_message = "MDI rejected: " + tok_err;
        return false;
    }
    if (!readyForCommandLocked(true, "MDI")) {
        return false;
    }
    telemetry_.last_mdi_command = gcode_line;

    auto hasG = [&](double g) {
        for (const auto& w : words) if (w.letter == 'G' && std::abs(w.value - g) < 1e-3) return true;
        return false;
    };
    auto hasM = [&](int m) {
        for (const auto& w : words) if (w.letter == 'M' && static_cast<int>(std::lround(w.value)) == m) return true;
        return false;
    };
    auto wordValue = [&](char letter, double& out) {
        for (const auto& w : words) if (w.letter == letter) { out = w.value; return true; }
        return false;
    };
    const double unit_scale = hasG(20) || hasG(70) ? 25.4 : 1.0;
    const bool laser = (config_.operation_mode == OperationMode::Laser && !config_.is_mill);

    // Custom M-code macros M100..M999 (e.g. M101.lua)
    for (const auto& w : words) {
        const int m_num = static_cast<int>(std::lround(w.value));
        if (w.letter == 'M' && m_num >= 100 && m_num <= 999) {
            if (!config_.scripting.enabled || !script_runner_) {
                telemetry_.active_fault_message = "MDI rejected: scripting is disabled (M" + std::to_string(m_num) + ")";
                return false;
            }
            const std::string macro_path = config_.scripting.scripts_dir + "/macros/M" + std::to_string(m_num) + ".lua";
            const std::string resolved = scripting::resolveScriptPath(macro_path);
            if (resolved.empty()) {
                telemetry_.active_fault_message = "MDI rejected: macro M" + std::to_string(m_num) + " not found";
                return false;
            }
            return script_runner_->runFileAsync(resolved);
        }
    }

    // Workspace selection G54 .. G59
    for (int w = 0; w < 6; ++w) {
        if (hasG(54.0 + w)) {
            telemetry_.active_workspace = static_cast<uint8_t>(w);
            config_.active_workspace    = static_cast<uint8_t>(w);
            bumpWorkspaceRevisionLocked();
            refreshWorkFrameLocked();
            break;
        }
    }

    // Tool length offset G43 H<n> / G49 (applied before any motion on the same block)
    if (hasG(44) || hasG(43.1)) {
        telemetry_.active_fault_message = "MDI rejected: only G43 H<n> / G49 tool length offsets are supported";
        return false;
    }
    if (hasG(43)) {
        double h_val = static_cast<double>(telemetry_.current_tool_number);
        wordValue('H', h_val);
        const long h = std::lround(h_val);
        if (h < 0 || h > 255 || !setToolLengthOffsetLocked(static_cast<uint16_t>(h))) {
            if (h < 0 || h > 255) telemetry_.active_fault_message = "MDI rejected: G43 H value out of range";
            return false;
        }
    } else if (hasG(49)) {
        setToolLengthOffsetLocked(0);
    }

    // G10 L2 / L20 P1..P6: set work coordinate system offsets
    if (hasG(10)) {
        double l_val = 0.0, p_val = 0.0;
        if (!wordValue('L', l_val) || !wordValue('P', p_val) ||
            (std::lround(l_val) != 2 && std::lround(l_val) != 20) || p_val < 1.0 || p_val > 6.0)
        {
            telemetry_.active_fault_message = "MDI rejected: only G10 L2/L20 P1..P6 is supported";
            return false;
        }
        const bool l20 = (std::lround(l_val) == 20);
        const uint8_t target_wcs = static_cast<uint8_t>(std::lround(p_val) - 1);
        const char letters[4] = {'X', 'Y', 'Z', 'A'};
        for (size_t a = 0; a < 4; ++a) {
            double v = 0.0;
            if (wordValue(letters[a], v)) {
                v *= unit_scale;
                // L2: offset value given directly. L20: offset such that the current tool tip reads `v`.
                telemetry_.workspace_offsets_mm[target_wcs][a] = l20 ? wcsOffsetForLocked(a, v) : v;
            }
        }
        config_.workspace_offsets_mm = telemetry_.workspace_offsets_mm;
        bumpWorkspaceRevisionLocked();
        refreshWorkFrameLocked();
        return true;
    }

    // G92: make the current position read as the given work coordinates (adjusts the active WCS offset)
    if (hasG(92)) {
        const char letters[4] = {'X', 'Y', 'Z', 'A'};
        bool any = false;
        for (size_t a = 0; a < 4; ++a) {
            double v = 0.0;
            if (wordValue(letters[a], v)) {
                any = true;
                v *= unit_scale;
                telemetry_.workspace_offsets_mm[telemetry_.active_workspace][a] = wcsOffsetForLocked(a, v);
                if (a == 2) telemetry_.thc_z_offset_mm = 0.0;
            }
        }
        if (!any) {
            telemetry_.active_fault_message = "MDI rejected: G92 needs at least one axis word";
            return false;
        }
        config_.workspace_offsets_mm = telemetry_.workspace_offsets_mm;
        bumpWorkspaceRevisionLocked();
        refreshWorkFrameLocked();
        return true;
    }

    gcode::ParseOptions opts;
    opts.dwell_p_milliseconds = config_.dwell_p_milliseconds;
    opts.initial_feed_mm_min  = mdi_feed_mm_min_;
    opts.tool_length_mode     = config_.tool_table.mode;
    auto mdi_res = gcode::GCodeParser::parse(gcode_line, telemetry_.work_pos_mm, 0.3, telemetry_.work_offset_mm, opts);
    if (mdi_res.hasErrors()) {
        telemetry_.active_fault_message = "MDI rejected: " + mdi_res.errors.front().text;
        return false;
    }
    double f_val = 0.0;
    if (wordValue('F', f_val) && f_val > 0.0) {
        mdi_feed_mm_min_ = f_val * unit_scale; // Feed rate is modal across MDI commands
    }
    if (!telemetry_.simulator_mode) {
        for (size_t a = 0; a < 4; ++a) {
            if (((mdi_res.machine_axes_used >> a) & 1U) && !telemetry_.axis_homed[a]) {
                telemetry_.active_fault_message = std::string("MDI rejected: machine-coordinate move on unhomed axis ") + kAxisNames[a];
                return false;
            }
        }
    }

    // Soft limits for every move of this command
    for (const auto& seg : mdi_res.segments) {
        if (seg.type == SegmentType::RapidMove || seg.type == SegmentType::CutLinear ||
            seg.type == SegmentType::CutArcCW || seg.type == SegmentType::CutArcCCW) {
            if (!checkLinearMoveLocked(seg.start_xyza_mm, seg.end_xyza_mm)) return false;
        }
    }

    // S word: update commanded spindle speed
    double s_val = 0.0;
    if (wordValue('S', s_val) && s_val > 0.0) {
        const uint16_t rpm = clampRpm(static_cast<uint32_t>(std::lround(s_val)));
        telemetry_.program_spindle_rpm     = rpm;
        telemetry_.commanded_spindle_rpm   = rpm;
        telemetry_.spindle_override_active = false;
        const uint32_t max_rpm = (config_.vfd.enabled && config_.vfd.max_rpm > 0) ? config_.vfd.max_rpm : 24000;
        target_spindle_pwm_duty_ = rpmToDuty(rpm, max_rpm);
        telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;
        if (board_->isOpen() && !telemetry_.simulator_mode) {
            board_->setSpindlePwm(telemetry_.spindle_pwm_duty, config_.limit_override);
        }
        if (config_.vfd.enabled) {
            vfd_controller_.setRpm(rpm);
        }
    }

    // Spindle / torch & coolant M-codes
    bool outputs_changed = false;
    if (hasM(3) || hasM(4)) {
        if (!telemetry_.dry_run_mode) {
            if (config_.vfd.enabled && !vfd_controller_.isConnected()) {
                faultLocked("Huanyang VFD is not responding (offline). Spindle cannot start.");
                return false;
            }
            telemetry_.torch_or_spindle_on = true;
            telemetry_.outputs_bitmask |= core::OutputBit::OUT1_TorchOrSpindle;
            if (config_.is_mill && hasM(4)) {
                telemetry_.outputs_bitmask |= core::OutputBit::OUT2_SpindleDir;
            } else if (config_.is_mill) {
                telemetry_.outputs_bitmask &= ~core::OutputBit::OUT2_SpindleDir;
            }
            manual_spindle_requested_ = true;
            outputs_changed = true;
        }
    }
    if (hasM(5)) {
        telemetry_.torch_or_spindle_on = false;
        telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        if (config_.is_mill) telemetry_.outputs_bitmask &= ~core::OutputBit::OUT2_SpindleDir;
        manual_spindle_requested_ = false;
        outputs_changed = true;
    }
    if (hasM(7)) { telemetry_.outputs_bitmask |= core::OutputBit::OUT4_MistCoolant;  outputs_changed = true; }
    if (hasM(8)) { telemetry_.outputs_bitmask |= core::OutputBit::OUT3_FloodCoolant; outputs_changed = true; }
    if (hasM(9)) {
        telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant);
        outputs_changed = true;
    }
    if (outputs_changed && board_->isOpen() && !telemetry_.simulator_mode) {
        board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
    }

    // Motion
    std::vector<ToolpathSegment> moves;
    moves.reserve(mdi_res.segments.size());
    for (auto& seg : mdi_res.segments) {
        if (seg.type == SegmentType::ProgramStop) continue; // M0/M1 have no meaning in MDI
        seg.outputs_mask    = telemetry_.outputs_bitmask;
        seg.workspace_index = telemetry_.active_workspace;
        moves.push_back(seg);
    }
    if (!moves.empty()) {
        MachineConfig mdi_cfg = config_;
        if (!laser) mdi_cfg.is_mill = true;
        slice_queue_ = planner::TrajectoryPlanner::sliceProgram(
            moves, mdi_cfg, telemetry_.feed_override_pct, 0.0f, telemetry_.dry_run_mode, kDtSec);
        if (!slice_queue_.empty()) {
            if (board_->isOpen() && !telemetry_.simulator_mode) {
                board_->beginJog(Axis::X, config_.limit_override);
            }
            telemetry_.state = ControllerState::Jogging;
        }
    }
    return true;
}

void MachineController::setNextGCodeLine(size_t line_index) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked()) {
        return;
    }
    if (program_->lines.empty()) return;

    const size_t clamped_line = std::min(line_index, program_->lines.size() - 1);
    telemetry_.active_gcode_line_index = clamped_line;
    telemetry_.segment_progress_0_to_1 = 0.0f;
    telemetry_.pending_tool_number     = 0;
    telemetry_.pending_tool_description.clear();
    if (clamped_line == 0) {
        telemetry_.job_elapsed_seconds = 0.0;
        telemetry_.thc_z_offset_mm     = 0.0;
    }

    size_t target_seg = 0;
    if (clamped_line > 0) {
        target_seg = program_->segments.size();
        for (size_t i = 0; i < program_->segments.size(); ++i) {
            if (program_->segments[i].gcode_line >= clamped_line) {
                target_seg = i;
                break;
            }
        }
    }
    telemetry_.active_segment_index = target_seg;
    last_sent_program_segment_valid_ = false;

    if (telemetry_.state == ControllerState::PausedHold ||
        telemetry_.state == ControllerState::ToolChangeHold)
    {
        slice_queue_.clear();
        job_planner_.reset();
        hold_needs_replan_ = false;
        telemetry_.state = ControllerState::Idle;
    }
}

// ============================================================================
// Job planning & execution
// ============================================================================
planner::FrameContext MachineController::jobFrameLocked() const {
    planner::FrameContext f;
    f.enabled           = true;
    f.workspace_offsets = telemetry_.workspace_offsets_mm;
    f.plan_workspace    = telemetry_.active_workspace;
    if (config_.tool_table.mode == ToolLengthMode::ToolTable) {
        f.apply_tool_lengths = true;
        for (const auto& [tool, entry] : config_.tool_table.tools) {
            if (tool < f.tool_lengths.size()) f.tool_lengths[tool] = entry.length_mm;
        }
        f.inherit_tlo_mm = telemetry_.active_tlo_mm;
    }
    return f;
}

std::array<double, 4> MachineController::planFramePosLocked() const {
    // Planning frame = spindle frame of the active WCS (work position without the tool length offset)
    std::array<double, 4> p = telemetry_.work_pos_mm;
    p[2] += activeTloLocked();
    return p;
}

bool MachineController::validateProgramForRunLocked(size_t start_seg) {
    const auto& prog = *program_;
    if (prog.hasErrors()) {
        telemetry_.active_fault_message = "Cannot start: program has errors. " + prog.firstErrorText();
        return false;
    }
    if (!telemetry_.simulator_mode) {
        for (size_t a = 0; a < 4; ++a) {
            if (((prog.machine_axes_used >> a) & 1U) && !telemetry_.axis_homed[a]) {
                telemetry_.active_fault_message = std::string("Cannot start: the program uses machine coordinates (G53/G28) on ") +
                                                  kAxisNames[a] + ", which is not homed. Home the machine first.";
                return false;
            }
        }
    }

    if (config_.tool_table.mode == ToolLengthMode::ToolTable) {
        // Every G43 H<n> must have a measured length, and after every tool change the program must
        // select that tool's offset (otherwise the previous tool's length would still be applied).
        const auto& tools = config_.tool_table.tools;
        bool expect_tlo_after_change = false;
        uint16_t changed_to = 0;
        size_t change_line = 0;
        for (size_t i = start_seg; i < prog.segments.size(); ++i) {
            const auto& seg = prog.segments[i];
            if (seg.tlo_tool != kTloInherit && seg.tlo_tool != 0 && tools.find(seg.tlo_tool) == tools.end()) {
                telemetry_.active_fault_message = "Cannot start: line " + std::to_string(seg.gcode_line + 1) +
                    " uses G43 H" + std::to_string(seg.tlo_tool) + ", but T" + std::to_string(seg.tlo_tool) +
                    " has no length in the tool table";
                return false;
            }
            if (seg.type == SegmentType::ToolChange && config_.tool_change.mode != ToolChangeMode::IgnoreM6) {
                expect_tlo_after_change = true;
                changed_to  = seg.tool_number;
                change_line = seg.gcode_line;
                continue;
            }
            const bool is_motion = seg.type == SegmentType::RapidMove || seg.type == SegmentType::CutLinear ||
                                   seg.type == SegmentType::CutArcCW || seg.type == SegmentType::CutArcCCW;
            if (expect_tlo_after_change && is_motion && seg.machine_axes_mask == 0) {
                if (seg.tlo_tool != changed_to) {
                    telemetry_.active_fault_message = "Cannot start: the tool change to T" + std::to_string(changed_to) +
                        " on line " + std::to_string(change_line + 1) + " is not followed by G43 H" +
                        std::to_string(changed_to) + " (required in tool table mode)";
                    return false;
                }
                expect_tlo_after_change = false;
            }
        }
    }

    bool any_soft = false;
    for (size_t a = 0; a < 4; ++a) {
        if (config_.axes[a].soft_limits_enabled && telemetry_.axis_homed[a]) any_soft = true;
    }
    if (!any_soft) return true;

    // Every programmed point must lie inside the soft limits (checked before any motion starts)
    const auto frame = jobFrameLocked();
    const auto& plan_off = frame.workspace_offsets[frame.plan_workspace];
    for (size_t i = start_seg; i < prog.segments.size(); ++i) {
        const auto& seg = prog.segments[i];
        if (seg.type != SegmentType::RapidMove && seg.type != SegmentType::CutLinear &&
            seg.type != SegmentType::CutArcCW && seg.type != SegmentType::CutArcCCW) {
            continue;
        }
        const auto t = planner::TrajectoryPlanner::transformTarget(seg, frame);
        std::array<double, 4> m{};
        for (size_t a = 0; a < 4; ++a) m[a] = t[a] + plan_off[a];
        std::string err;
        if (!withinSoftLimitsLocked(m, &err)) {
            telemetry_.active_fault_message = "Cannot start: line " + std::to_string(seg.gcode_line + 1) +
                                              " exceeds soft limits (" + err + ")";
            return false;
        }
    }
    return true;
}

void MachineController::rebuildPlannedSlicesLocked(size_t start_from_segment, bool resume_in_place) {
    slice_queue_.clear();
    job_planner_.reset();
    const auto& segs = program_->segments;
    if (segs.empty() || start_from_segment >= segs.size()) {
        return;
    }

    job_frame_ = jobFrameLocked();
    const uint8_t plan_ws = job_frame_.plan_workspace;
    const auto& offsets = job_frame_.workspace_offsets;
    auto toPlan = [&](const std::array<double, 4>& p, uint8_t ws) {
        std::array<double, 4> r{};
        for (size_t a = 0; a < 4; ++a) r[a] = p[a] + offsets[std::min<uint8_t>(ws, 5)][a] - offsets[plan_ws][a];
        return r;
    };
    auto isMotion = [](const ToolpathSegment& s) {
        return s.type == SegmentType::RapidMove || s.type == SegmentType::CutLinear ||
               s.type == SegmentType::CutArcCW || s.type == SegmentType::CutArcCCW;
    };

    std::vector<ToolpathSegment> prefix;
    // Planning frame = spindle frame of the active WCS (tool length offsets come from the segments)
    std::array<double, 4> cur = planFramePosLocked();
    const std::array<double, 4> plan_start = cur;

    // Safe retract height (configured for the tool tip): never above the Z soft limit of a homed machine
    double safe_z = std::max(cur[2], config_.tool_change.safe_z_mm + activeTloLocked());
    {
        const auto& zax = config_.axes[2];
        if (zax.soft_limits_enabled && telemetry_.axis_homed[2]) {
            const double z_max_work = zax.soft_limit_max_mm - offsets[plan_ws][2] - 0.5;
            safe_z = std::max(cur[2], std::min(safe_z, z_max_work));
        }
    }

    // Scan upcoming segments in the current tool block to check if spindle is required.
    // RULE: Never start spindle before M03 if starting from the beginning (start_from_segment == 0).
    bool spindle_needed = false;
    uint16_t target_spindle_rpm = (telemetry_.commanded_spindle_rpm > 0) ? telemetry_.commanded_spindle_rpm : 12000;
    uint8_t approach_mask = 0;

    if (start_from_segment > 0) {
        bool m3_active_before = false;
        for (size_t i = start_from_segment; i > 0; --i) {
            const auto& s = segs[i - 1];
            if (s.type == SegmentType::ToolChange) {
                break; // Tool change stops spindle
            }
            if ((s.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0) {
                m3_active_before = true;
                break;
            }
        }
        if (m3_active_before || telemetry_.torch_or_spindle_on) {
            for (size_t i = start_from_segment; i < segs.size(); ++i) {
                const auto& s = segs[i];
                if (s.type == SegmentType::ToolChange && i > start_from_segment) {
                    break; // Do not scan beyond next tool change
                }
                if (s.spindle_rpm > 0) {
                    target_spindle_rpm = s.spindle_rpm;
                }
                if ((s.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0 ||
                    s.type == SegmentType::CutLinear || s.type == SegmentType::CutArcCW || s.type == SegmentType::CutArcCCW)
                {
                    spindle_needed = true;
                    approach_mask = s.outputs_mask | core::OutputBit::OUT1_TorchOrSpindle;
                    break;
                }
            }
        }
    }

    if (spindle_needed) {
        telemetry_.program_spindle_rpm = target_spindle_rpm;
        if (!telemetry_.spindle_override_active) {
            telemetry_.commanded_spindle_rpm = target_spindle_rpm;
            const uint32_t max_rpm = (config_.vfd.enabled && config_.vfd.max_rpm > 0) ? config_.vfd.max_rpm : 24000;
            target_spindle_pwm_duty_ = rpmToDuty(target_spindle_rpm, max_rpm);
            telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;
            if (config_.vfd.enabled) {
                vfd_controller_.setRpm(target_spindle_rpm);
            }
        }
    }

    const bool is_plasma = (config_.operation_mode == OperationMode::Plasma && !config_.is_mill);
    auto makePrefix = [&](SegmentType type, const ToolpathSegment& meta) {
        ToolpathSegment s{};
        s.type            = type;
        s.segment_index   = meta.segment_index;
        s.gcode_line      = meta.gcode_line;
        s.start_xyza_mm   = cur;
        s.end_xyza_mm     = cur;
        s.workspace_index = plan_ws; // Prefix moves are already in the planning frame
        s.tlo_tool        = 0;       // ...including any tool length offset
        s.tool_number     = meta.tool_number;
        return s;
    };
    auto spinUpDwell = [&](const ToolpathSegment& meta) {
        ToolpathSegment d = makePrefix(SegmentType::Dwell, meta);
        // Spindle needs its ramp time; a plasma torch needs a fresh pierce before it moves
        d.dwell_seconds = is_plasma
            ? std::max(0.1, telemetry_.pierce_delay_override_s > 0.01f ? static_cast<double>(telemetry_.pierce_delay_override_s) : 0.5)
            : std::max(0.1, config_.spindle_ramp_up_s);
        d.outputs_mask  = approach_mask;
        d.spindle_rpm   = target_spindle_rpm;
        d.torch_active  = true;
        return d;
    };

    size_t program_first = start_from_segment;

    if (resume_in_place) {
        // Resuming from Feed Hold: the tool sits on the path; continue the interrupted segment from here
        if (spindle_needed && !telemetry_.dry_run_mode && (!telemetry_.torch_or_spindle_on || is_plasma)) {
            prefix.push_back(spinUpDwell(segs[start_from_segment]));
        }
    } else {
        // Fresh start, restart after Stop, or jump to a line: find the first segment with physical motion
        size_t first_motion = segs.size();
        for (size_t i = start_from_segment; i < segs.size(); ++i) {
            if (isMotion(segs[i])) {
                first_motion = i;
                break;
            }
        }
        // Non-motion segments (tool change, dwell, program stop) before the first motion run in place
        for (size_t i = start_from_segment; i < first_motion; ++i) {
            ToolpathSegment non_motion = segs[i];
            non_motion.start_xyza_mm = cur;
            non_motion.end_xyza_mm   = cur;
            non_motion.workspace_index = plan_ws;
            non_motion.machine_axes_mask = 0;
            non_motion.tlo_tool = 0;
            prefix.push_back(non_motion);
        }
        program_first = first_motion;

        if (first_motion < segs.size()) {
            const auto& fm = segs[first_motion];
            // Planning-frame start point of the first motion = end of the segment before it
            const std::array<double, 4> target_motion_start = (first_motion > 0)
                ? planner::TrajectoryPlanner::transformTarget(segs[first_motion - 1], job_frame_)
                : [&] { auto p = toPlan(fm.start_xyza_mm, fm.workspace_index); p[2] += planner::TrajectoryPlanner::toolLengthFor(fm, job_frame_); return p; }();
            const std::array<double, 4> fm_end = planner::TrajectoryPlanner::transformTarget(fm, job_frame_);

            if (start_from_segment > 0) {
                // MID-PROGRAM START: lift (spindle off) -> spin up -> XY at safe Z -> descend -> continue
                if (safe_z - cur[2] > 0.05) {
                    ToolpathSegment lift = makePrefix(SegmentType::RapidMove, fm);
                    lift.end_xyza_mm = {cur[0], cur[1], safe_z, cur[3]};
                    prefix.push_back(lift);
                    cur = lift.end_xyza_mm;
                }
                if (spindle_needed && !telemetry_.dry_run_mode) {
                    prefix.push_back(spinUpDwell(fm));
                }
                const bool is_cutting_motion = (fm.type == SegmentType::CutLinear ||
                                                fm.type == SegmentType::CutArcCW || fm.type == SegmentType::CutArcCCW);
                if (is_cutting_motion) {
                    if (std::hypot(target_motion_start[0] - cur[0], target_motion_start[1] - cur[1]) > 0.05) {
                        ToolpathSegment xy_move = makePrefix(SegmentType::RapidMove, fm);
                        xy_move.end_xyza_mm  = {target_motion_start[0], target_motion_start[1], cur[2], target_motion_start[3]};
                        xy_move.outputs_mask = spindle_needed ? approach_mask : 0;
                        prefix.push_back(xy_move);
                        cur = xy_move.end_xyza_mm;
                    }
                    if (cur[2] - target_motion_start[2] > 0.05) {
                        ToolpathSegment z_approach = makePrefix(SegmentType::RapidMove, fm);
                        z_approach.end_xyza_mm  = target_motion_start;
                        z_approach.outputs_mask = spindle_needed ? approach_mask : 0;
                        prefix.push_back(z_approach);
                        cur = target_motion_start;
                    }
                }
            } else {
                // START FROM THE BEGINNING: a first RAPID that travels in XY never drags the tool sideways
                // at a low height: go up, across at travel height, then the rapid itself only descends.
                // A first FEED move is executed exactly as programmed.
                const bool moves_xy = std::hypot(fm_end[0] - cur[0], fm_end[1] - cur[1]) > 0.05;
                if (fm.type == SegmentType::RapidMove && moves_xy) {
                    double travel_z = std::max(cur[2], fm_end[2]);
                    if (std::abs(fm_end[2] - cur[2]) <= 0.05) {
                        travel_z = std::max(travel_z, safe_z); // Pure XY rapid at the current (possibly low) Z
                    }
                    if (travel_z - cur[2] > 0.05) {
                        ToolpathSegment lift = makePrefix(SegmentType::RapidMove, fm);
                        lift.end_xyza_mm = {cur[0], cur[1], travel_z, cur[3]};
                        prefix.push_back(lift);
                        cur = lift.end_xyza_mm;
                    }
                    if (travel_z - fm_end[2] > 0.05) {
                        ToolpathSegment across = makePrefix(SegmentType::RapidMove, fm);
                        across.end_xyza_mm = {fm_end[0], fm_end[1], travel_z, fm_end[3]};
                        prefix.push_back(across);
                        cur = across.end_xyza_mm;
                    }
                }
            }
        }
    }

    for (auto& p : prefix) {
        p.length_mm = dist3(p.start_xyza_mm, p.end_xyza_mm);
    }

    planner::PlannerOptions opts;
    opts.feed_override_pct       = telemetry_.feed_override_pct;
    opts.pierce_delay_override_s = telemetry_.pierce_delay_override_s;
    opts.dry_run_no_torch        = telemetry_.dry_run_mode;
    opts.slice_dt_sec            = kDtSec;
    job_planner_ = std::make_unique<planner::StreamingPlanner>(config_, plan_start, opts, job_frame_);
    job_planner_->setSource(std::move(prefix),
                            std::shared_ptr<const std::vector<ToolpathSegment>>(program_, &program_->segments),
                            program_first);
    job_planner_->plan(slice_queue_, kRefillAmount * 2);
}

void MachineController::refillJobQueueLocked() {
    if (!job_planner_ || job_planner_->finished() || hold_decel_active_) return;
    const auto s = telemetry_.state;
    if (s != ControllerState::RunningGCode && s != ControllerState::DwellPierce) return;
    if (slice_queue_.size() >= kRefillThreshold) return;
    job_planner_->plan(slice_queue_, kRefillAmount);
}

void MachineController::replanFromQueueTailLocked() {
    // Keep a short buffer of already-planned slices and re-plan everything after it with the new feed
    if (!job_planner_ || hold_decel_active_ || slice_queue_.size() <= kOverrideKeepSlices) return;
    const auto& last = slice_queue_[kOverrideKeepSlices - 1];
    const auto& prev = slice_queue_[kOverrideKeepSlices - 2];
    if (last.is_dwell || last.is_tool_change || last.is_program_stop || last.is_hold_decel || !last.from_program ||
        prev.is_dwell || prev.is_tool_change || prev.is_program_stop || !prev.from_program) {
        return;
    }
    if (last.segment_index >= program_->segments.size()) return;

    std::array<double, 4> unit{};
    double d2 = 0.0;
    for (size_t a = 0; a < 4; ++a) {
        unit[a] = last.target_pos_mm[a] - prev.target_pos_mm[a];
        d2 += unit[a] * unit[a];
    }
    const double d = std::sqrt(d2);
    if (d > 1e-9) {
        for (auto& u : unit) u /= d;
    }
    const double v = d / kDtSec;

    // Remainder of the current segment becomes the prefix of the new planner
    ToolpathSegment rem = program_->segments[last.segment_index];
    rem.start_xyza_mm = last.target_pos_mm;

    planner::PlannerOptions opts;
    opts.feed_override_pct       = telemetry_.feed_override_pct;
    opts.pierce_delay_override_s = telemetry_.pierce_delay_override_s;
    opts.dry_run_no_torch        = telemetry_.dry_run_mode;
    opts.slice_dt_sec            = kDtSec;
    auto np = std::make_unique<planner::StreamingPlanner>(config_, last.target_pos_mm, opts, job_frame_);
    np->setSource({rem}, std::shared_ptr<const std::vector<ToolpathSegment>>(program_, &program_->segments),
                  last.segment_index + 1, true);
    np->setInitialMotion(v, unit);

    slice_queue_.erase(slice_queue_.begin() + static_cast<std::ptrdiff_t>(kOverrideKeepSlices), slice_queue_.end());
    job_planner_ = std::move(np);
    job_planner_->plan(slice_queue_, kRefillAmount);
}

void MachineController::startJob() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (program_->segments.empty()) return;

    // SAFETY LOCK: Never allow Cycle Start while E-Stop is active or the board is disconnected
    if (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected) {
        return;
    }
    if (telemetry_.surface_probing_active) {
        telemetry_.active_fault_message = "Cycle Start rejected: surface probing in progress";
        return;
    }

    // SAFETY LOCK: If Huanyang VFD is enabled but not answering, the spindle cannot start
    if (config_.vfd.enabled && !vfd_controller_.isConnected()) {
        faultLocked("Huanyang VFD is not responding (offline). Spindle cannot start.");
        return;
    }

    if (telemetry_.state == ControllerState::PausedHold) {
        if (hold_decel_active_) {
            // Still decelerating from the hold: resume as soon as the machine has stopped
            resume_after_hold_ = true;
            stop_after_hold_   = false;
            telemetry_.state   = ControllerState::RunningGCode;
            return;
        }
        if (!hold_needs_replan_ && (!slice_queue_.empty() || (job_planner_ && !job_planner_->finished()))) {
            telemetry_.state = ControllerState::RunningGCode; // M0 / M1 resume: continue the queued motion
            telemetry_.active_fault_message.clear();
            return;
        }
    }

    if (telemetry_.state != ControllerState::Idle &&
        telemetry_.state != ControllerState::PausedHold &&
        telemetry_.state != ControllerState::ToolChangeHold)
    {
        return;
    }
    if (cont_jog_active_ || (!slice_queue_.empty() && telemetry_.state == ControllerState::Idle)) {
        return;
    }

    const bool resume_in_place = (telemetry_.state == ControllerState::PausedHold) && last_sent_from_program_;

    // If resuming from Manual Tool Change Hold (M6), confirm the new tool is installed
    const bool was_tool_hold = (telemetry_.state == ControllerState::ToolChangeHold || telemetry_.pending_tool_number > 0);

    size_t start_seg = telemetry_.active_segment_index;
    if (telemetry_.state == ControllerState::PausedHold && last_sent_program_segment_valid_) {
        // Resume from the segment that was actually sent last (acknowledgements may lag behind)
        start_seg = last_sent_program_segment_;
    }
    if (start_seg >= program_->segments.size()) {
        start_seg = 0;
    }
    // If resuming after confirming an M6 tool change, advance past the ToolChange segment
    if (was_tool_hold && start_seg < program_->segments.size() &&
        program_->segments[start_seg].type == SegmentType::ToolChange)
    {
        start_seg++;
    }

    if (!validateProgramForRunLocked(start_seg)) {
        return;
    }

    if (was_tool_hold) {
        if (telemetry_.pending_tool_number > 0) {
            telemetry_.current_tool_number = telemetry_.pending_tool_number;
        }
        telemetry_.pending_tool_number = 0;
        telemetry_.pending_tool_description.clear();
    }

    telemetry_.limit_switch_triggered = false;
    telemetry_.soft_limit_triggered   = false;
    telemetry_.active_fault_message.clear();

    if (start_seg == 0) {
        telemetry_.active_gcode_line_index = 0;
        telemetry_.job_elapsed_seconds = 0.0;
        telemetry_.thc_z_offset_mm     = 0.0;
    }
    telemetry_.active_segment_index    = start_seg;
    telemetry_.segment_progress_0_to_1 = 0.0f;

    hold_decel_active_ = false;
    hold_needs_replan_ = false;
    resume_after_hold_ = false;
    stop_after_hold_   = false;
    rebuildPlannedSlicesLocked(start_seg, resume_in_place);
    telemetry_.state = slice_queue_.empty() ? ControllerState::Idle : ControllerState::RunningGCode;
}

void MachineController::pauseJob() {
    std::lock_guard<std::mutex> lock(mutex_);
    pauseJobLocked();
}

void MachineController::pauseJobLocked() {
    if (telemetry_.state != ControllerState::RunningGCode && telemetry_.state != ControllerState::DwellPierce) {
        return;
    }
    telemetry_.state = ControllerState::PausedHold;
    telemetry_.current_feed_mm_min = 0.0;
    manual_spindle_requested_ = false;
    if (config_.spindle_ramp_down_s <= 0.01) {
        spindle_ramp_pct_ = 0.0;
        telemetry_.spindle_speed_pct = 0.0f;
        telemetry_.spindle_ramping_up = false;
        telemetry_.spindle_ramping_down = false;
        telemetry_.torch_or_spindle_on = false;
        telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        if (board_->isOpen() && !telemetry_.simulator_mode) {
            board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
        }
    }
    beginHoldDecelLocked(false);
}

void MachineController::beginHoldDecelLocked(bool stop_after) {
    if (hold_decel_active_) {
        if (stop_after) {
            stop_after_hold_   = true;
            resume_after_hold_ = false;
        }
        return;
    }
    job_planner_.reset();
    hold_needs_replan_ = true;
    resume_after_hold_ = false;
    const double v0 = last_sent_target_valid_ ? last_sent_speed_mm_s_ : 0.0;
    const std::array<double, 4> from = last_sent_target_valid_ ? last_sent_target_ : telemetry_.work_pos_mm;
    planner::TrajectoryPlanner::planFeedHoldDeceleration(slice_queue_, from, v0, config_, kDtSec);
    // Planned slices carry outputs; during a hold the torch / laser must not be re-enabled
    for (auto& s : slice_queue_) {
        if (telemetry_.state == ControllerState::PausedHold) {
            s.slice.outputs_mask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        }
    }
    if (slice_queue_.empty()) {
        hold_decel_active_ = false;
        if (stop_after) {
            stopJobLocked(false);
        }
    } else {
        hold_decel_active_ = true;
        stop_after_hold_   = stop_after;
    }
}

void MachineController::stopJobLocked(bool is_estop) {
    surface_probing_cancel_requested_.store(true);
    if (script_runner_) {
        script_runner_->cancel();
    }

    const bool hw = board_->isOpen() && !telemetry_.simulator_mode;
    if (hw && is_estop) {
        // Halt the board first, then the outputs, then drop anything still buffered
        board_->emergencyStop();
        board_->setOutputs(0x00, false);
    }

    slice_queue_.clear();
    job_planner_.reset();
    in_flight_seqs_.clear();
    homing_axis_queue_.clear();
    homing_phase_              = HomingPhase::None;
    active_probe_mode_         = ProbeMode::None;
    cont_jog_active_           = false;
    cont_jog_held_             = {false, false, false, false};
    cont_jog_target_vel_mm_s_  = {0.0, 0.0, 0.0, 0.0};
    cont_jog_current_vel_mm_s_ = {0.0, 0.0, 0.0, 0.0};
    hold_decel_active_         = false;
    hold_needs_replan_         = false;
    resume_after_hold_         = false;
    stop_after_hold_           = false;
    settle_active_             = false;
    torch_on_duration_s_       = 0.0;
    manual_spindle_requested_  = false;
    pulse_active_              = false;
    air_postflow_timer_s_      = 0.0;
    last_sent_laser_pwm_duty_  = 0xFFFF;
    last_sent_target_valid_    = false;
    last_sent_speed_mm_s_      = 0.0;
    telemetry_.live_laser_power_pct = 0.0f;
    telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    telemetry_.current_feed_mm_min  = 0.0;
    telemetry_.segment_progress_0_to_1 = 0.0f;
    telemetry_.pending_tool_number     = 0;
    telemetry_.pending_tool_description.clear();

    if (hw) {
        board_->cancelMotion();
    }

    if (is_estop) {
        // Immediate hard cut-off of every output on E-Stop
        spindle_ramp_pct_               = 0.0;
        telemetry_.spindle_speed_pct    = 0.0f;
        telemetry_.spindle_ramping_up   = false;
        telemetry_.spindle_ramping_down = false;
        telemetry_.torch_or_spindle_on  = false;
        telemetry_.outputs_bitmask      = 0;
        telemetry_.laser_air_assist_active  = false;
        telemetry_.laser_red_pointer_active = false;
        telemetry_.spindle_pwm_duty     = 0;
        config_.limit_override          = false;
        telemetry_.limit_override_active = false;
        if (hw) {
            board_->setSpindlePwm(0, false);
        }
        if (config_.vfd.enabled) {
            vfd_controller_.emergencyStop();
        }
        vfd_last_run_cmd_ = 0;
    } else {
        // Normal Stop: if spindle is active, engage controlled deceleration ramp down
        if ((spindle_ramp_pct_ > 0.05 || telemetry_.torch_or_spindle_on) && config_.spindle_ramp_down_s > 0.01) {
            telemetry_.spindle_ramping_up   = false;
            telemetry_.spindle_ramping_down = true;
            telemetry_.torch_or_spindle_on  = true;
            telemetry_.outputs_bitmask     |= core::OutputBit::OUT1_TorchOrSpindle;
        } else {
            spindle_ramp_pct_               = 0.0;
            telemetry_.spindle_speed_pct    = 0.0f;
            telemetry_.spindle_ramping_up   = false;
            telemetry_.spindle_ramping_down = false;
            telemetry_.torch_or_spindle_on  = false;
            telemetry_.outputs_bitmask     &= ~core::OutputBit::OUT1_TorchOrSpindle;
        }
        if (hw) {
            board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
        }
    }

    if (telemetry_.state != ControllerState::Disconnected && telemetry_.state != ControllerState::EStop) {
        telemetry_.state = ControllerState::Idle;
    }
}

void MachineController::stopJob() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto s = telemetry_.state;
    const bool path_motion = (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce ||
                              (s == ControllerState::Jogging && !cont_jog_active_) ||
                              (s == ControllerState::PausedHold && hold_decel_active_));
    if (path_motion && !slice_queue_.empty() && last_sent_target_valid_ && last_sent_speed_mm_s_ > 0.5) {
        // Controlled stop: decelerate along the path, then stop (no lost steps)
        beginHoldDecelLocked(true);
        return;
    }
    stopJobLocked();
}

void MachineController::setFeedOverride(float percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    float old_pct = telemetry_.feed_override_pct;
    telemetry_.feed_override_pct = std::clamp(percent, 10.0f, 250.0f);
    if (std::abs(old_pct - telemetry_.feed_override_pct) < 0.5f) {
        return;
    }
    // Feed override never raises motion above the configured axis limits (enforced by the planner)
    if (job_planner_) {
        job_planner_->setFeedOverride(telemetry_.feed_override_pct);
        if (telemetry_.state == ControllerState::RunningGCode || telemetry_.state == ControllerState::PausedHold) {
            replanFromQueueTailLocked();
        }
    }
}

void MachineController::setPierceDelayOverride(float seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.pierce_delay_override_s = std::clamp(seconds, 0.0f, 10.0f);
    if (job_planner_) {
        job_planner_->setPierceDelayOverride(telemetry_.pierce_delay_override_s);
    }
}

// ============================================================================
// Inputs, E-Stop input, end-stops and interlocks
// ============================================================================
bool MachineController::evaluatePinStateLocked(uint8_t pin_1_to_12, bool invert) const noexcept {
    if (pin_1_to_12 < 1 || pin_1_to_12 > 12) return false;
    bool raw_high = ((telemetry_.inputs_bitmask >> (pin_1_to_12 - 1)) & 1U) != 0;
    return invert ? !raw_high : raw_high;
}

bool MachineController::estopInputActiveLocked() const noexcept {
    return config_.estop_pin >= 1 && evaluatePinStateLocked(config_.estop_pin, config_.estop_invert);
}

bool MachineController::anyLimitActiveLocked(std::string* desc) const {
    const bool homing = (telemetry_.state == ControllerState::Homing);
    const uint8_t homing_pin = homing ? config_.axes[static_cast<size_t>(active_homing_axis_)].home_pin : 0;
    for (size_t a = 0; a < 4; ++a) {
        const auto& ax = config_.axes[a];
        // During homing only the switch shared with the active axis' home input is ignored
        if (ax.limit_plus_pin != 0 && !(homing && ax.limit_plus_pin == homing_pin) &&
            evaluatePinStateLocked(ax.limit_plus_pin, ax.limit_plus_invert)) {
            if (desc) {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "END-STOP HIT: %s+ Limit (IN%u)", kAxisNames[a], ax.limit_plus_pin);
                *desc = buf;
            }
            return true;
        }
        if (ax.limit_minus_pin != 0 && !(homing && ax.limit_minus_pin == homing_pin) &&
            evaluatePinStateLocked(ax.limit_minus_pin, ax.limit_minus_invert)) {
            if (desc) {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "END-STOP HIT: %s- Limit (IN%u)", kAxisNames[a], ax.limit_minus_pin);
                *desc = buf;
            }
            return true;
        }
    }
    if (!homing && !telemetry_.simulator_mode &&
        hw_limit_fault_) {
        if (desc) *desc = "END-STOP HIT: board hardware limit fault";
        return true;
    }
    return false;
}

void MachineController::evaluateInputsAndEndStopsLocked() {
    telemetry_.inputs_bitmask = hw_inputs_bitmask_ |
        (telemetry_.simulator_mode ? telemetry_.simulated_inputs_bitmask : static_cast<uint16_t>(0));

    // 1. Plasma THC inputs & touch probe
    telemetry_.thc_arc_ok_active    = evaluatePinStateLocked(config_.thc.arc_ok_pin, config_.thc.arc_ok_invert);
    telemetry_.thc_up_active        = evaluatePinStateLocked(config_.thc.thc_up_pin, config_.thc.thc_up_invert);
    telemetry_.thc_down_active      = evaluatePinStateLocked(config_.thc.thc_down_pin, config_.thc.thc_down_invert);
    telemetry_.probe_contact_active = evaluatePinStateLocked(config_.probe.probe_pin, config_.probe.probe_invert);

    if (config_.operation_mode == OperationMode::Laser && !config_.is_mill) {
        telemetry_.laser_chiller_flow_ok = evaluatePinStateLocked(config_.laser.chiller_flow_pin, config_.laser.chiller_flow_invert);
        telemetry_.laser_door_closed     = evaluatePinStateLocked(config_.laser.door_interlock_pin, config_.laser.door_interlock_invert);
        telemetry_.laser_fault_active    = evaluatePinStateLocked(config_.laser.laser_fault_pin, config_.laser.laser_fault_invert);
    }

    // 2. E-Stop: input pin or board hardware E-Stop. NEVER bypassed by Limit Override.
    const bool hw_estop = !telemetry_.simulator_mode && board_->isOpen() &&
                          hw_estop_fault_;
    const bool pin_estop = estopInputActiveLocked();
    telemetry_.estop_input_active = hw_estop || pin_estop;
    if (telemetry_.estop_input_active &&
        telemetry_.state != ControllerState::EStop && telemetry_.state != ControllerState::Disconnected) {
        faultLocked(pin_estop ? "E-STOP INPUT PIN TRIGGERED" : "HARDWARE E-STOP REPORTED BY MOTION BOARD");
        return;
    }

    if (telemetry_.state == ControllerState::Disconnected) return;

    // 3. End-stop switches (Limit Override lets the operator jog off a switch)
    std::string hit_desc;
    const bool any_limit_hit = anyLimitActiveLocked(&hit_desc);
    if (!config_.limit_override && any_limit_hit && telemetry_.state != ControllerState::EStop) {
        faultLocked(hit_desc);
        telemetry_.limit_switch_triggered = true;
        return;
    }

    // 4. Soft limits (backstop: moves are checked before they start)
    const auto s = telemetry_.state;
    if (!config_.limit_override &&
        (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce ||
         s == ControllerState::Jogging || s == ControllerState::Probing)) {
        std::string err;
        if (!withinSoftLimitsLocked(telemetry_.machine_pos_mm, &err)) {
            telemetry_.soft_limit_triggered = true;
            faultLocked("SOFT LIMIT EXCEEDED: " + err);
            return;
        }
    }

    // 5. Laser safety interlocks (never bypassed by Limit Override)
    if (config_.operation_mode == OperationMode::Laser && !config_.is_mill) {
        const bool laser_firing = (telemetry_.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0 || pulse_active_;
        const bool gcode_active = (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce);

        // Water Chiller Flow Sensor check (critical for tube cooling)
        if (config_.laser.chiller_flow_pin >= 1 && !telemetry_.laser_chiller_flow_ok &&
            (laser_firing || gcode_active) && s != ControllerState::EStop) {
            faultLocked("LASER INTERLOCK: WATER CHILLER NO FLOW ALARM (IN" + std::to_string(config_.laser.chiller_flow_pin) + ")");
            return;
        }

        // Laser Power Supply Fault / Overtemp Alarm check
        if (config_.laser.laser_fault_pin >= 1 && telemetry_.laser_fault_active && s != ControllerState::EStop) {
            faultLocked("LASER INTERLOCK: LASER PSU FAULT ALARM (IN" + std::to_string(config_.laser.laser_fault_pin) + ")");
            return;
        }

        // Enclosure Door / Lid Safety Switch check
        if (config_.laser.door_interlock_pin >= 1 && !telemetry_.laser_door_closed && laser_firing) {
            const std::string msg = "LASER INTERLOCK: ENCLOSURE LID OPEN (IN" + std::to_string(config_.laser.door_interlock_pin) + ")";
            if (gcode_active) {
                pauseJobLocked();
            } else {
                stopJobLocked(false);
            }
            // The beam goes off immediately regardless of any ramp settings
            pulse_active_ = false;
            telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
            telemetry_.torch_or_spindle_on = false;
            for (auto& q : slice_queue_) q.slice.outputs_mask &= ~core::OutputBit::OUT1_TorchOrSpindle;
            if (board_->isOpen() && !telemetry_.simulator_mode) {
                board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
            }
            telemetry_.active_fault_message = msg;
        }
    }
}

// ============================================================================
// Worker thread
// ============================================================================
void MachineController::processBoardReportLocked(bool got_report, const board::BoardStatus& rep, bool usb_open_now) {
    if (telemetry_.simulator_mode) return;

    if (!usb_open_now) {
        hw_inputs_bitmask_ = 0;
        if (telemetry_.usb_connected) {
            telemetry_.usb_connected = false;
            faultLocked("USB CONNECTION TO MOTION BOARD LOST", ControllerState::Disconnected);
        }
        return;
    }

    const auto now = Clock::now();
    if (got_report) {
        board_report_seen_   = true;
        last_report_time_    = now;
        telemetry_.last_ack_seq = rep.ack_seq;
        telemetry_.tick_counter = rep.tick_counter;
        hw_inputs_bitmask_   = rep.inputs_bitmask;
        hw_estop_fault_      = rep.estop_fault;
        hw_limit_fault_      = rep.limit_fault;
        hw_motion_active_    = rep.motion_active;
        hw_live_steps_       = rep.live_steps;
        hw_latched_steps_    = rep.latched_steps;

        auto it = in_flight_seqs_.find(rep.ack_seq);
        if (it != in_flight_seqs_.end() && it->second.is_gcode_stream) {
            telemetry_.active_segment_index    = it->second.segment_index;
            telemetry_.active_gcode_line_index = it->second.gcode_line;
            telemetry_.segment_progress_0_to_1 = it->second.segment_progress;
        }
        // Prune acknowledged sequence numbers (handles 16-bit wrap-around)
        for (auto map_it = in_flight_seqs_.begin(); map_it != in_flight_seqs_.end();) {
            const int16_t diff = static_cast<int16_t>(map_it->first - rep.ack_seq);
            if (diff <= 0) {
                map_it = in_flight_seqs_.erase(map_it);
            } else {
                ++map_it;
            }
        }

        if (settle_active_ && !rep.motion_active) {
            settle_board_idle_seen_ = true;
        }

        // Position feedback: when everything is acknowledged and the board is idle, its step counters
        // must equal our commanded position. The first match proves the counters are usable; after that
        // any difference (stopped early, lost packet) is corrected from the board.
        const bool board_idle = !rep.motion_active && in_flight_seqs_.empty() && slice_queue_.empty() &&
                                !cont_jog_active_ && !hold_decel_active_ && !settle_active_ && !motionActiveLocked();
        if (board_idle) {
            double max_diff_mm = 0.0;
            bool step_mismatch = false;
            for (size_t a = 0; a < 4; ++a) {
                const auto spm = config_.axes[a].steps_per_mm;
                const int32_t cmd = static_cast<int32_t>(std::llround(telemetry_.machine_pos_mm[a] * spm));
                if (std::abs(static_cast<int64_t>(rep.live_steps[a]) - cmd) > 1) {
                    step_mismatch = true;
                    max_diff_mm = std::max(max_diff_mm, std::abs(rep.live_steps[a] - cmd) / spm);
                }
            }
            if (!feedback_validated_) {
                if (!step_mismatch) feedback_validated_ = true;
            } else if (step_mismatch) {
                std::array<double, 4> m{};
                for (size_t a = 0; a < 4; ++a) m[a] = static_cast<double>(rep.live_steps[a]) / config_.axes[a].steps_per_mm;
                telemetry_.machine_pos_mm = m;
                for (size_t a = 0; a < 4; ++a) telemetry_.work_pos_mm[a] = m[a] - telemetry_.work_offset_mm[a];
                last_sent_target_valid_ = false;
                if (max_diff_mm > 0.5 && telemetry_.active_fault_message.empty()) {
                    char buf[128];
                    std::snprintf(buf, sizeof(buf), "Position corrected from motion board feedback (%.2f mm)", max_diff_mm);
                    telemetry_.active_fault_message = buf;
                }
            }
        }
    } else {
        const bool timed_out = board_report_seen_ ? (now - last_report_time_ > kReportTimeout)
                                                  : (now - connect_time_ > kFirstReportTimeout);
        if (timed_out && telemetry_.state != ControllerState::EStop) {
            faultLocked("MOTION BOARD STOPPED RESPONDING (no status reports)");
            return;
        }
    }

    for (const auto& kv : in_flight_seqs_) {
        if (now - kv.second.sent_at > kAckTimeout) {
            faultLocked("MOTION BOARD STOPPED ACKNOWLEDGING MOTION DATA");
            return;
        }
    }
}

void MachineController::updateSpindleLocked(bool gcode_running) {
    bool want_spindle_on = false;
    uint8_t source_mask = telemetry_.outputs_bitmask;
    if (telemetry_.state != ControllerState::EStop && telemetry_.state != ControllerState::Disconnected &&
        !telemetry_.dry_run_mode) {
        if (gcode_running && !slice_queue_.empty()) {
            const auto& front = slice_queue_.front();
            source_mask = front.slice.outputs_mask;
            want_spindle_on = ((front.slice.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0) &&
                              !front.is_tool_change;
        } else {
            want_spindle_on = manual_spindle_requested_;
        }
    }

    if (want_spindle_on) {
        telemetry_.torch_or_spindle_on = true;
        telemetry_.outputs_bitmask |= core::OutputBit::OUT1_TorchOrSpindle;
        telemetry_.spindle_ramping_down = false;
        if (config_.spindle_ramp_up_s <= 0.01) {
            spindle_ramp_pct_ = 100.0;
            telemetry_.spindle_ramping_up = false;
        } else if (spindle_ramp_pct_ < 100.0) {
            spindle_ramp_pct_ = std::min(100.0, spindle_ramp_pct_ + (100.0 / config_.spindle_ramp_up_s) * kDtSec);
            telemetry_.spindle_ramping_up = (spindle_ramp_pct_ < 99.95);
            if (!telemetry_.spindle_ramping_up) {
                spindle_ramp_pct_ = 100.0;
            }
        } else {
            telemetry_.spindle_ramping_up = false;
        }
    } else {
        telemetry_.spindle_ramping_up = false;
        if (config_.spindle_ramp_down_s <= 0.01 || telemetry_.state == ControllerState::EStop) {
            spindle_ramp_pct_ = 0.0;
            telemetry_.spindle_ramping_down = false;
            if (!gcode_running && !pulse_active_ && telemetry_.state != ControllerState::Jogging) {
                if ((telemetry_.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0 &&
                    board_->isOpen() && !telemetry_.simulator_mode) {
                    telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
                    board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
                }
                telemetry_.torch_or_spindle_on = false;
                telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
            }
        } else if (spindle_ramp_pct_ > 0.0) {
            spindle_ramp_pct_ = std::max(0.0, spindle_ramp_pct_ - (100.0 / config_.spindle_ramp_down_s) * kDtSec);
            telemetry_.spindle_ramping_down = (spindle_ramp_pct_ > 0.05);
            if (!telemetry_.spindle_ramping_down) {
                spindle_ramp_pct_ = 0.0;
                telemetry_.torch_or_spindle_on = false;
                telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
                if (board_->isOpen() && !telemetry_.simulator_mode) {
                    board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
                }
            } else {
                telemetry_.torch_or_spindle_on = true;
                telemetry_.outputs_bitmask |= core::OutputBit::OUT1_TorchOrSpindle;
            }
        } else {
            telemetry_.spindle_ramping_down = false;
        }
    }
    telemetry_.spindle_speed_pct = static_cast<float>(spindle_ramp_pct_);
    if (config_.spindle_ramp_up_s > 0.01 || config_.spindle_ramp_down_s > 0.01) {
        const uint16_t new_duty = static_cast<uint16_t>(
            std::llround(static_cast<double>(target_spindle_pwm_duty_) * (spindle_ramp_pct_ / 100.0)));
        if (telemetry_.spindle_ramping_down && board_->isOpen() && !telemetry_.simulator_mode) {
            if (std::abs(static_cast<int>(new_duty) - static_cast<int>(telemetry_.spindle_pwm_duty)) >= 5 || new_duty == 0) {
                board_->setSpindlePwm(new_duty, config_.limit_override);
            }
        }
        telemetry_.spindle_pwm_duty = new_duty;
    }

    // Huanyang VFD: send run / stop / direction only when the requested state changes
    if (config_.vfd.enabled) {
        const auto vfd_telem = vfd_controller_.getTelemetry();
        telemetry_.vfd_telemetry = vfd_telem;
        const bool reverse = config_.is_mill && (source_mask & core::OutputBit::OUT2_SpindleDir) != 0;
        const int desired = want_spindle_on ? (reverse ? 2 : 1) : 0;
        if (desired != vfd_last_run_cmd_) {
            if (desired != 0) {
                vfd_controller_.setRpm(telemetry_.commanded_spindle_rpm);
                vfd_controller_.setRun(true, desired == 2);
            } else {
                vfd_controller_.setRun(false, false);
            }
            vfd_last_run_cmd_ = desired;
        }
        if (vfd_telem.anyAlarm() &&
            telemetry_.state != ControllerState::EStop && telemetry_.state != ControllerState::Disconnected) {
            faultLocked(vfd_telem.last_error_msg.empty() ? "Huanyang VFD alarm" : vfd_telem.last_error_msg);
        }
    }

    // Laser test pulse timer
    if (pulse_active_) {
        pulse_timer_s_ += kDtSec;
        if (pulse_timer_s_ >= pulse_duration_s_) {
            pulse_active_ = false;
            telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
            telemetry_.torch_or_spindle_on = false;
            telemetry_.spindle_pwm_duty = 0;
            telemetry_.live_laser_power_pct = 0.0f;
            last_sent_laser_pwm_duty_ = 0;
            if (board_->isOpen() && !telemetry_.simulator_mode) {
                board_->setSpindlePwm(0, config_.limit_override);
                board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
            }
        }
    }

    // Track how long the torch has been continuously firing (for THC pierce delay)
    if (telemetry_.torch_or_spindle_on && !telemetry_.spindle_ramping_up) {
        torch_on_duration_s_ += kDtSec;
    } else {
        torch_on_duration_s_ = 0.0;
        telemetry_.thc_anti_dive_locked = false;
    }
}

bool MachineController::sendQueuedSliceLocked() {
    planner::PlannedSlice next_slice = std::move(slice_queue_.front());
    slice_queue_.pop_front();
    const bool hw = board_->isOpen() && !telemetry_.simulator_mode;
    const bool is_job = (telemetry_.state == ControllerState::RunningGCode || telemetry_.state == ControllerState::DwellPierce);

    // Manual Tool Change (M6): spindle off, retract, wait for the operator
    if (next_slice.is_tool_change) {
        job_planner_.reset();
        manual_spindle_requested_      = false;
        telemetry_.torch_or_spindle_on = false;
        telemetry_.outputs_bitmask    &= ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
        telemetry_.axis_vel_mm_min     = {0.0, 0.0, 0.0, 0.0};
        if (hw) {
            board_->setOutputs(telemetry_.outputs_bitmask, config_.limit_override);
        }
        telemetry_.pending_tool_number      = (next_slice.tool_number > 0) ? next_slice.tool_number : 1;
        telemetry_.pending_tool_description = next_slice.tool_description;
        telemetry_.active_segment_index     = next_slice.segment_index + 1;
        telemetry_.active_gcode_line_index  = next_slice.gcode_line;
        telemetry_.segment_progress_0_to_1  = 0.0f;
        last_sent_program_segment_valid_    = false;
        slice_queue_.clear();

        std::string resolved_m6;
        if (config_.scripting.enabled && config_.scripting.m6_script_enabled && script_runner_) {
            resolved_m6 = scripting::resolveScriptPath(config_.scripting.m6_script_path);
        }
        if (!resolved_m6.empty()) {
            telemetry_.state = ControllerState::ToolChangeHold;
            script_runner_->runFileAsync(resolved_m6, [this](bool ok, const std::string&) {
                if (ok) {
                    // Operator confirmed "Tool Ready": start/resume milling automatically
                    this->startJob();
                }
            });
        } else {
            // Automatic Z retract to safe_z_mm (and optional XY park position), spindle strictly OFF
            std::array<double, 4> cur_w = telemetry_.work_pos_mm;
            const double target_z = std::max(cur_w[2], config_.tool_change.safe_z_mm);
            telemetry_.state = ControllerState::ToolChangeHold;
            std::vector<ToolpathSegment> moves;
            auto addRapid = [&](const std::array<double, 4>& a, const std::array<double, 4>& b) {
                ToolpathSegment s{};
                s.type = SegmentType::RapidMove;
                s.start_xyza_mm = a;
                s.end_xyza_mm = b;
                s.length_mm = dist3(a, b);
                moves.push_back(s);
            };
            std::array<double, 4> z_up = {cur_w[0], cur_w[1], target_z, cur_w[3]};
            bool ok = true;
            if (target_z - cur_w[2] > 0.05) {
                ok = checkLinearMoveLocked(cur_w, z_up);
                if (ok) addRapid(cur_w, z_up);
                cur_w = z_up;
            }
            if (ok && config_.tool_change.move_to_park_xy) {
                std::array<double, 4> park_xy = {config_.tool_change.park_x_mm, config_.tool_change.park_y_mm, cur_w[2], cur_w[3]};
                if (std::hypot(park_xy[0] - cur_w[0], park_xy[1] - cur_w[1]) > 0.05 && checkLinearMoveLocked(cur_w, park_xy)) {
                    addRapid(cur_w, park_xy);
                }
            }
            if (!moves.empty()) {
                MachineConfig manual_cfg = config_;
                manual_cfg.is_mill = true;
                slice_queue_ = planner::TrajectoryPlanner::sliceProgram(moves, manual_cfg);
                if (!slice_queue_.empty()) telemetry_.state = ControllerState::Jogging;
            }
        }
        return false;
    }

    // Program stop (M0 / M1): planner already brought the motion to rest before this marker
    if (next_slice.is_program_stop) {
        telemetry_.state = ControllerState::PausedHold;
        manual_spindle_requested_ = false;
        hold_needs_replan_ = false;
        telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        telemetry_.active_segment_index    = next_slice.segment_index;
        telemetry_.active_gcode_line_index = next_slice.gcode_line;
        telemetry_.active_fault_message = "Program stop (M0/M1) - press Cycle Start to continue";
        return false;
    }

    if (next_slice.tool_number > 0 && is_job) {
        telemetry_.current_tool_number = next_slice.tool_number;
    }
    if (next_slice.spindle_rpm > 0 && next_slice.spindle_rpm != telemetry_.program_spindle_rpm && is_job) {
        telemetry_.program_spindle_rpm = next_slice.spindle_rpm;
        if (!telemetry_.spindle_override_active) {
            telemetry_.commanded_spindle_rpm = next_slice.spindle_rpm;
            const uint32_t max_rpm = (config_.vfd.enabled && config_.vfd.max_rpm > 0) ? config_.vfd.max_rpm : 24000;
            target_spindle_pwm_duty_ = rpmToDuty(next_slice.spindle_rpm, max_rpm);
            telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;
            if (config_.vfd.enabled) {
                vfd_controller_.setRpm(next_slice.spindle_rpm);
            }
        }
    }

    if (is_job) {
        telemetry_.state = next_slice.is_dwell ? ControllerState::DwellPierce : ControllerState::RunningGCode;
        if (!hw) {
            telemetry_.active_segment_index    = next_slice.segment_index;
            telemetry_.active_gcode_line_index = next_slice.gcode_line;
            telemetry_.segment_progress_0_to_1 = next_slice.segment_progress;
        }
    }

    // Path speed of this slice in the planning frame
    double xy_speed_mm_min = 0.0;
    double path_speed_mm_s = 0.0;
    if (last_sent_target_valid_) {
        const double dx = next_slice.target_pos_mm[0] - last_sent_target_[0];
        const double dy = next_slice.target_pos_mm[1] - last_sent_target_[1];
        xy_speed_mm_min = (std::hypot(dx, dy) / kDtSec) * 60.0;
        path_speed_mm_s = dist3(next_slice.target_pos_mm, last_sent_target_) / kDtSec;
    }

    const bool laser_mode = (config_.operation_mode == OperationMode::Laser && !config_.is_mill);
    if (laser_mode && is_job) {
        // CNC Laser Mode: M4 velocity-proportional power scaling & auto air assist (job slices only)
        const bool laser_active = (next_slice.slice.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0;
        if (laser_active && !telemetry_.dry_run_mode) {
            double target_feed = 1000.0;
            if (next_slice.from_program && next_slice.segment_index < program_->segments.size()) {
                target_feed = program_->segments[next_slice.segment_index].feed_mm_min * (telemetry_.feed_override_pct / 100.0);
            }
            double ratio = 1.0;
            if (config_.laser.dynamic_power_m4 && target_feed > 1.0) {
                ratio = std::clamp(xy_speed_mm_min / target_feed,
                                   static_cast<double>(config_.laser.corner_power_floor_pct) / 100.0, 1.0);
            }
            double base_power_pct = 100.0;
            if (next_slice.spindle_rpm > 0 && next_slice.spindle_rpm <= 1000) {
                base_power_pct = static_cast<double>(next_slice.spindle_rpm) / 10.0;
            }
            const double effective_power_pct = std::clamp(base_power_pct * ratio * (laser_power_override_pct_ / 100.0), 0.0, 100.0);
            telemetry_.live_laser_power_pct = static_cast<float>(effective_power_pct);

            const float duty_frac = config_.laser.min_duty_pct +
                                    (static_cast<float>(effective_power_pct) / 100.0f) * (config_.laser.max_duty_pct - config_.laser.min_duty_pct);
            const uint16_t duty = static_cast<uint16_t>(std::clamp(duty_frac * 10.0f, 0.0f, 1000.0f));
            if (std::abs(static_cast<int>(duty) - static_cast<int>(last_sent_laser_pwm_duty_)) >= 15 ||
                (duty == 0 && last_sent_laser_pwm_duty_ != 0)) {
                last_sent_laser_pwm_duty_ = duty;
                telemetry_.spindle_pwm_duty = duty;
                if (hw) {
                    board_->setSpindlePwm(duty, config_.limit_override);
                }
            }
            if (config_.laser.auto_air_assist) {
                next_slice.slice.outputs_mask |= core::OutputBit::OUT3_FloodCoolant;
                air_postflow_timer_s_ = config_.laser.air_postflow_s;
            }
        } else {
            telemetry_.live_laser_power_pct = 0.0f;
            if (last_sent_laser_pwm_duty_ != 0) {
                last_sent_laser_pwm_duty_ = 0;
                telemetry_.spindle_pwm_duty = 0;
                if (hw) {
                    board_->setSpindlePwm(0, config_.limit_override);
                }
            }
            if (config_.laser.auto_air_assist) {
                if (air_postflow_timer_s_ > 0.0) {
                    air_postflow_timer_s_ -= kDtSec;
                    next_slice.slice.outputs_mask |= core::OutputBit::OUT3_FloodCoolant;
                } else {
                    next_slice.slice.outputs_mask &= ~core::OutputBit::OUT3_FloodCoolant;
                }
            }
        }
        telemetry_.laser_air_assist_active  = (next_slice.slice.outputs_mask & core::OutputBit::OUT3_FloodCoolant) != 0;
        telemetry_.laser_red_pointer_active = (next_slice.slice.outputs_mask & core::OutputBit::OUT2_SpindleDir) != 0;
    } else if (is_job && !config_.is_mill && config_.thc.enabled &&
               telemetry_.state == ControllerState::RunningGCode &&
               (next_slice.slice.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0 &&
               torch_on_duration_s_ >= config_.thc.delay_after_pierce_s) {
        // Plasma THC: anti-dive lock below a fraction of the programmed feed, else follow UP/DOWN inputs
        double target_feed = 1500.0;
        if (next_slice.from_program && next_slice.segment_index < program_->segments.size()) {
            target_feed = program_->segments[next_slice.segment_index].feed_mm_min * (telemetry_.feed_override_pct / 100.0);
        }
        const double min_allowed_xy = target_feed * (config_.thc.min_xy_feed_pct / 100.0);
        telemetry_.thc_anti_dive_locked = (xy_speed_mm_min < min_allowed_xy);
        if (!telemetry_.thc_anti_dive_locked) {
            double dz_thc = 0.0;
            if (telemetry_.thc_up_active && !telemetry_.thc_down_active) {
                dz_thc = config_.thc.adjust_speed_mm_s * kDtSec;
            } else if (telemetry_.thc_down_active && !telemetry_.thc_up_active) {
                dz_thc = -config_.thc.adjust_speed_mm_s * kDtSec;
            }
            if (std::abs(dz_thc) > 1e-6) {
                const double new_offset = std::clamp(telemetry_.thc_z_offset_mm + dz_thc, config_.thc.z_min_mm, config_.thc.z_max_mm);
                const double actual_dz = new_offset - telemetry_.thc_z_offset_mm;
                const int32_t dsteps = static_cast<int32_t>(std::llround(actual_dz * config_.axes[2].steps_per_mm));
                telemetry_.thc_z_offset_mm += static_cast<double>(dsteps) / config_.axes[2].steps_per_mm;
                next_slice.slice.delta_steps[2] += dsteps;
            }
        }
    }

    telemetry_.outputs_bitmask     = next_slice.slice.outputs_mask;
    telemetry_.torch_or_spindle_on = (telemetry_.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0;

    // Synchronize the displayed workspace if the program switched G54..G59 (motion is already mapped)
    bool frame_changed = false;
    if (is_job && next_slice.workspace_index != telemetry_.active_workspace && next_slice.workspace_index < 6) {
        telemetry_.active_workspace = next_slice.workspace_index;
        config_.active_workspace    = next_slice.workspace_index;
        bumpWorkspaceRevisionLocked();
        frame_changed = true;
    }
    // ...and the tool length offset the program selected with G43 H<n> / G49
    if (is_job && next_slice.from_program && config_.tool_table.mode == ToolLengthMode::ToolTable && next_slice.tlo_tool != kTloInherit &&
        (next_slice.tlo_tool != telemetry_.active_tlo_tool || std::abs(next_slice.tlo_mm - telemetry_.active_tlo_mm) > 1e-9)) {
        telemetry_.active_tlo_tool = next_slice.tlo_tool;
        telemetry_.active_tlo_mm   = next_slice.tlo_mm;
        frame_changed = true;
    }
    if (frame_changed) {
        refreshWorkFrameLocked();
    }

    // Advance the commanded position (the THC Z correction is part of the step deltas, so it is
    // already contained in the machine position - it is not added again)
    for (size_t a = 0; a < 4; ++a) {
        const double delta_mm = static_cast<double>(next_slice.slice.delta_steps[a]) / config_.axes[a].steps_per_mm;
        telemetry_.machine_pos_mm[a] += delta_mm;
        telemetry_.work_pos_mm[a]     = telemetry_.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
        telemetry_.axis_vel_mm_min[a] = std::abs(delta_mm) / kDtSec * 60.0;
    }

    last_sent_target_       = next_slice.target_pos_mm;
    last_sent_target_valid_ = true;
    last_sent_speed_mm_s_   = next_slice.is_dwell ? 0.0 : path_speed_mm_s;
    if (is_job || next_slice.is_hold_decel) {
        last_sent_from_program_ = next_slice.from_program;
        if (next_slice.from_program) {
            last_sent_program_segment_       = next_slice.segment_index;
            last_sent_program_segment_valid_ = true;
        }
    }

    if (!hw) {
        telemetry_.tick_counter += 25;
        return true;
    }
    uint16_t assigned_seq = 0;
    // A feed-hold deceleration of a job stays on the G-code stream
    const bool gcode_stream = is_job || (next_slice.is_hold_decel && telemetry_.state == ControllerState::PausedHold);
    const auto stream = gcode_stream ? board::SliceStream::Program : board::SliceStream::Internal;
    if (!board_->sendSlice(stream, next_slice.slice, config_.limit_override, &assigned_seq)) {
        faultLocked("USB WRITE FAILED WHILE STREAMING MOTION");
        return false;
    }
    in_flight_seqs_[assigned_seq] = SeqProgressInfo{
        next_slice.segment_index, next_slice.gcode_line, next_slice.segment_progress, gcode_stream, Clock::now()};
    return true;
}

void MachineController::finishQueueLocked() {
    const bool hw = board_->isOpen() && !telemetry_.simulator_mode;

    if (hold_decel_active_) {
        if (stop_after_hold_ && hw && !in_flight_seqs_.empty()) return; // Let the last decel slices execute
        hold_decel_active_ = false;
        telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        last_sent_speed_mm_s_ = 0.0;
        if (stop_after_hold_) {
            stop_after_hold_ = false;
            stopJobLocked(false);
        } else if (resume_after_hold_) {
            resume_after_hold_ = false;
            hold_needs_replan_ = false;
            const size_t seg = last_sent_program_segment_valid_ ? last_sent_program_segment_ : telemetry_.active_segment_index;
            rebuildPlannedSlicesLocked(seg, last_sent_from_program_);
            telemetry_.state = slice_queue_.empty() ? ControllerState::Idle : ControllerState::RunningGCode;
        }
        return;
    }

    if (telemetry_.state == ControllerState::PausedHold) return;

    // In real hardware mode, wait until the board has acknowledged every slice
    if (hw && !in_flight_seqs_.empty()) return;

    telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    last_sent_speed_mm_s_ = 0.0;
    manual_spindle_requested_  = manual_spindle_requested_ && telemetry_.state == ControllerState::Jogging;
    if (config_.spindle_ramp_down_s <= 0.01 || spindle_ramp_pct_ <= 0.05) {
        if (!manual_spindle_requested_) {
            spindle_ramp_pct_               = 0.0;
            telemetry_.spindle_speed_pct    = 0.0f;
            telemetry_.spindle_ramping_down = false;
            telemetry_.torch_or_spindle_on  = false;
            telemetry_.outputs_bitmask     &= ~core::OutputBit::OUT1_TorchOrSpindle;
        }
    } else if (!manual_spindle_requested_) {
        telemetry_.spindle_ramping_down = true;
    }
    if (telemetry_.state == ControllerState::RunningGCode || telemetry_.state == ControllerState::DwellPierce) {
        telemetry_.active_segment_index    = program_->segments.size();
        telemetry_.segment_progress_0_to_1 = 1.0f;
        last_sent_program_segment_valid_   = false;
        job_planner_.reset();
    }
    if (config_.operation_mode == OperationMode::Laser && !config_.is_mill && telemetry_.spindle_pwm_duty != 0 &&
        !manual_spindle_requested_) {
        // End of framing / laser job: make sure the beam PWM is back to zero
        telemetry_.spindle_pwm_duty = 0;
        last_sent_laser_pwm_duty_ = 0;
        if (hw) board_->setSpindlePwm(0, config_.limit_override);
    }
    if (hw) {
        board_->setOutputsAfterMotion(telemetry_.outputs_bitmask, config_.limit_override);
    }
    telemetry_.state = (telemetry_.pending_tool_number > 0) ? ControllerState::ToolChangeHold : ControllerState::Idle;
}

void MachineController::runMotionTickLocked(bool gcode_running) {
    const auto s = telemetry_.state;
    const bool motion_state = (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce ||
                               s == ControllerState::Jogging || s == ControllerState::Homing ||
                               s == ControllerState::Probing) || hold_decel_active_;
    if (cont_jog_active_ || !motion_state) return;

    if (s == ControllerState::Homing) {
        homingTickLocked();
        if (telemetry_.state != ControllerState::Homing || slice_queue_.empty()) return;
    } else if (s == ControllerState::Probing) {
        probingTickLocked();
        if (telemetry_.state != ControllerState::Probing || slice_queue_.empty()) return;
    }

    refillJobQueueLocked();

    const bool hw = board_->isOpen() && !telemetry_.simulator_mode;
    if (slice_queue_.empty()) {
        finishQueueLocked();
        return;
    }

    size_t budget = 1;
    if (hw) {
        budget = (in_flight_seqs_.size() < max_slices_in_flight_) ? (max_slices_in_flight_ - in_flight_seqs_.size()) : 0;
    }

    bool sent_any = false;
    for (size_t i = 0; i < budget && !slice_queue_.empty(); ++i) {
        const auto& front = slice_queue_.front();
        const bool front_requires_spindle = (front.slice.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0;
        const bool job_now = (telemetry_.state == ControllerState::RunningGCode || telemetry_.state == ControllerState::DwellPierce);

        // Plasma: wait for Arc OK before moving once the torch is lit
        if (job_now && !config_.is_mill && config_.operation_mode == OperationMode::Plasma &&
            config_.thc.enabled && config_.thc.wait_for_arc_ok &&
            telemetry_.torch_or_spindle_on && front_requires_spindle && !telemetry_.thc_arc_ok_active) {
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            break;
        }
        // Mill: do not cut until the spindle is at speed (or has spun down before M5/M6)
        if (job_now && front_requires_spindle && !front.is_hold_decel &&
            (telemetry_.spindle_ramping_up || telemetry_.spindle_ramping_down)) {
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            telemetry_.active_segment_index    = front.segment_index;
            telemetry_.active_gcode_line_index = front.gcode_line;
            break;
        }
        const auto state_before = telemetry_.state;
        if (!sendQueuedSliceLocked()) break;
        sent_any = true;
        if (telemetry_.state != state_before &&
            !(state_before == ControllerState::RunningGCode && telemetry_.state == ControllerState::DwellPierce) &&
            !(state_before == ControllerState::DwellPierce && telemetry_.state == ControllerState::RunningGCode)) {
            break;
        }
        refillJobQueueLocked();
    }

    if (gcode_running || sent_any) {
        if (telemetry_.state == ControllerState::RunningGCode || telemetry_.state == ControllerState::DwellPierce) {
            telemetry_.job_elapsed_seconds += kDtSec;
        }
    }
}

void MachineController::workerLoop() {
#ifdef _WIN32
    timeBeginPeriod(1);
#endif
    auto next_wake = Clock::now();

    while (running_thread_.load(std::memory_order_relaxed)) {
        next_wake += kTickInterval;

        // 1. Read the newest board status WITHOUT holding the controller lock (board reads can block for a few ms)
        board::BoardStatus rep{};
        const bool got_any = board_->isOpen() && board_->readStatus(rep, 2);
        const bool usb_open_now = board_->isOpen();

        {
            std::lock_guard<std::mutex> lock(mutex_);

            processBoardReportLocked(got_any, rep, usb_open_now);
            evaluateInputsAndEndStopsLocked();

            if (has_pending_config_ && !motionActiveLocked() && telemetry_.state != ControllerState::PausedHold) {
                has_pending_config_ = false;
                applyConfigLocked(pending_config_);
                if (telemetry_.active_fault_message == "Configuration change will be applied when motion stops") {
                    telemetry_.active_fault_message.clear();
                }
            }

            const bool gcode_running = (telemetry_.state == ControllerState::RunningGCode ||
                                        telemetry_.state == ControllerState::DwellPierce);
            updateSpindleLocked(gcode_running);
            updateContinuousJogLocked();
            runMotionTickLocked(gcode_running);

            // Live 3D feed rate & commanded feed rate for the DRO feed box
            const bool is_motion_state = motionActiveLocked();
            if (!is_motion_state) {
                telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
                telemetry_.current_feed_mm_min = 0.0;
            } else {
                const double vx = telemetry_.axis_vel_mm_min[0];
                const double vy = telemetry_.axis_vel_mm_min[1];
                const double vz = telemetry_.axis_vel_mm_min[2];
                telemetry_.current_feed_mm_min = std::sqrt(vx * vx + vy * vy + vz * vz);
            }
            double base_cmd_feed = 0.0;
            if (!program_->segments.empty()) {
                const size_t seg_i = std::min(telemetry_.active_segment_index, program_->segments.size() - 1);
                base_cmd_feed = program_->segments[seg_i].feed_mm_min;
            }
            telemetry_.commanded_feed_mm_min = base_cmd_feed * (telemetry_.feed_override_pct / 100.0);
        }

        // A deferred configuration change may require re-parsing the loaded program (outside the lock).
        // Never blocks the tick: if a re-parse is already running, the flag is checked again next tick.
        reparseLoadedProgramIfNeeded(false);

        // Never "catch up" with a burst of ticks after a stall
        const auto now = Clock::now();
        if (now > next_wake + 4 * kTickInterval) {
            next_wake = now;
        }
        std::this_thread::sleep_until(next_wake);
    }
#ifdef _WIN32
    timeEndPeriod(1);
#endif
}

// ============================================================================
// Scripting & Automation (Lua 5.4 Engine)
// ============================================================================
bool MachineController::runScriptString(const std::string& code, bool async) {
    if (!script_runner_) return false;
    if (async) {
        return script_runner_->runStringAsync(code);
    }
    return script_runner_->runStringBlocking(code);
}

bool MachineController::runScriptFile(const std::string& filepath, bool async) {
    if (!script_runner_) return false;
    if (async) {
        return script_runner_->runFileAsync(filepath);
    }
    return script_runner_->runFileBlocking(filepath);
}

void MachineController::cancelScript() {
    if (!script_runner_) return;
    const bool was_running = script_runner_->isRunning();
    script_runner_->cancel();
    if (!was_running) return;
    // Also stop the motion the script started (MDI moves, probing, homing)
    std::lock_guard<std::mutex> lock(mutex_);
    const auto s = telemetry_.state;
    if (s == ControllerState::Jogging && !cont_jog_active_ && !slice_queue_.empty()) {
        beginHoldDecelLocked(true);
    } else if (s == ControllerState::Probing || s == ControllerState::Homing) {
        stopJobLocked(false);
    }
}

bool MachineController::isScriptRunning() const {
    return script_runner_ && script_runner_->isRunning();
}

void MachineController::submitScriptPromptResponse(const std::string& response) {
    if (script_runner_) script_runner_->submitPromptResponse(response);
}

void MachineController::submitScriptPromptInput(const std::string& button, const std::string& input) {
    if (script_runner_) script_runner_->submitPromptResponse(button, input);
}

void MachineController::setScriptLogCallback(std::function<void(const std::string&, bool)> cb) {
    if (script_runner_) script_runner_->setLogCallback(std::move(cb));
}

} // namespace rnr::core
