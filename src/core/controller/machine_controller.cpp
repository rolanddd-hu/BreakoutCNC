// MachineController: Construction, connection, configuration, E-stop (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

// ============================================================================
// Construction / connection
// ============================================================================
MachineController::MachineController(std::unique_ptr<board::IBoardDriver> board, bool connect_now)
    : board_(board ? std::move(board) : board::createBoardDriver(board::BoardType::RnrEcoMotion)),
      slice_(std::make_unique<SliceBackend>(*this)),
      firmware_(std::make_unique<FirmwareBackend>(*this)) {
    slice_->setMaxSlicesInFlight(board_->capabilities().max_slices_in_flight);
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

    if (connect_now && !connect()) {
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
    return connectLocked();
}

bool MachineController::firmwareBoardSelectedLocked() const {
    const auto* d = board::BoardRegistry::instance().find(config_.board_id);
    return d && d->family == board::Family::FirmwarePlanned;
}

void MachineController::closeFirmwarePathLocked() {
    if (!fw_) return;
    std::lock_guard<std::mutex> plock(fw_ptr_mutex_);
    fw_->close();
    fw_.reset();
    ++caps_revision_;
    // What the old board reported no longer applies
    telemetry_.board_status_text.clear();
    telemetry_.firmware_state.clear();
    telemetry_.firmware_version.clear();
    telemetry_.board_inputs_mask = 0;
    telemetry_.thc_on_board = telemetry_.thc_board_active = telemetry_.thc_torch_on = false;
    telemetry_.thc_arc_voltage = 0.0;
    telemetry_.thc_board_mode.clear();
    firmware_->onBoardClosedLocked();
}

bool MachineController::connectLocked() {
    if (motionActiveLocked()) {
        stopJobLocked(true);
    }
    board_id_ = config_.board_id;
    telemetry_.board_id = board_id_;
    if (firmwareBoardSelectedLocked()) {
        // GRBL family: the board plans the motion; an RNR board left open is disarmed and released
        if (board_->isOpen()) {
            board_->emergencyStop();
            setBoardOutputsLocked(0x00, false);
            board_->close();
        }
        closeFirmwarePathLocked();
        auto& profile = config_.board_profiles[board_id_];
        board::completeProfile(board_id_, profile);
        auto path = std::make_unique<motion::FirmwarePlannedPath>(board_id_, profile);
        std::string err;
        if (!path->open(hardware_access_enabled_.load(), &err)) {
            telemetry_.usb_connected = false;
            telemetry_.board_connecting = false;
            telemetry_.board_status_text = err;
            telemetry_.active_fault_message = "Cannot connect: " + err;
            if (!telemetry_.simulator_mode) telemetry_.state = ControllerState::Disconnected;
            return false;
        }
        {
            std::lock_guard<std::mutex> plock(fw_ptr_mutex_);
            fw_ = std::move(path);
        }
        ++caps_revision_;
        const auto* desc = board::BoardRegistry::instance().find(board_id_);
        telemetry_.usb_connected   = true;
        telemetry_.board_connecting = true;
        telemetry_.simulator_mode  = false;
        telemetry_.product_name    = desc ? desc->display_name : board_id_;
        telemetry_.serial_number   = fw_->connectionLabel();
        telemetry_.firmware_version.clear();
        telemetry_.board_status_text = fw_->feedback().status_text;
        firmware_->onBoardConnected();
        telemetry_.outputs_bitmask     = 0;
        telemetry_.torch_or_spindle_on = false;
        telemetry_.limit_switch_triggered = false;
        telemetry_.soft_limit_triggered   = false;
        telemetry_.active_fault_message = "Connecting to " + telemetry_.product_name + " on " + fw_->connectionLabel() + "...";
        telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        telemetry_.current_feed_mm_min = 0.0;
        telemetry_.state = ControllerState::EStop;
        return true;
    }
    closeFirmwarePathLocked();
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
    setBoardOutputsLocked(0x00, false);
    telemetry_.outputs_bitmask     = 0;
    telemetry_.torch_or_spindle_on = false;

    // Sync board hardware step accumulators to the current machine coordinates
    slice_->syncBoardPositionLocked();
    slice_->onBoardConnected();

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
    if (fw_) {
        closeFirmwarePathLocked();
        telemetry_.board_connecting = false;
    }
    if (board_->isOpen()) {
        board_->emergencyStop();
        setBoardOutputsLocked(0x00, false);
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
        if (fw_) {
            // The real board stays disarmed while the simulator drives the UI
            fw_->emergencyStop();
            closeFirmwarePathLocked();
            telemetry_.board_connecting = false;
        }
        if (board_->isOpen()) {
            // The real board must stay disarmed while the simulator drives the UI
            board_->emergencyStop();
            setBoardOutputsLocked(0x00, false);
        }
        telemetry_.product_name  = "Virtual " + getBoardCapabilitiesLocked().display_name + " (Simulator)";
        telemetry_.serial_number = "SIM-200HZ";
        if (telemetry_.state == ControllerState::Disconnected) {
            telemetry_.state = ControllerState::EStop;
        }
    } else if (firmwareBoardSelectedLocked()) {
        // Leaving the simulator with a firmware board selected: connect to it
        telemetry_.axis_homed = {false, false, false, false};
        if (!connectLocked()) telemetry_.state = ControllerState::Disconnected;
    } else if (board_->isOpen()) {
        const auto info = board_->info();
        telemetry_.product_name  = info.product_name;
        telemetry_.serial_number = info.serial_number;
        // Leaving the simulator: the simulated position is meaningless for the real machine
        telemetry_.axis_homed = {false, false, false, false};
        slice_->syncBoardPositionLocked();
        slice_->restartFeedbackLocked();
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
        if (outputsLiveLocked()) {
            setBoardOutputsLocked(telemetry_.outputs_bitmask, false);
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
        if (jobActiveLocked(false)) {
            // Changing kinematics / pinout while the machine moves is unsafe: apply once motion has stopped
            pending_config_     = cfg;
            has_pending_config_ = true;
            telemetry_.active_fault_message = "Configuration change will be applied when motion stops";
            return;
        }
        applyConfigLocked(cfg);
    }
    if (vfd_sync_needed_.load()) syncVfd();
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
        if (!program_reparse_needed_ || jobActiveLocked(true)) {
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
    const std::shared_ptr<const gcode::ParseResult> active = warp ? warpProgram(*parsed, map) : parsed;

    std::lock_guard<std::mutex> lock(mutex_);
    if (program_text_ != text || jobActiveLocked(true)) {
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

    // A new output pin assignment: switch everything off once, so no output stays on under its old meaning
    const bool outputs_remapped = cfg.io.output_pin != config_.io.output_pin;
    if (!board::BoardRegistry::instance().find(cfg.board_id)) cfg.board_id = board::kRnrBoardId;
    const bool board_changed = cfg.board_id != config_.board_id;
    if (board_changed) {
        // Another board: release the connected one; the operator connects to the new board
        const bool was_connected = (board_->isOpen() || fw_) && !telemetry_.simulator_mode;
        if (board_->isOpen()) {
            board_->emergencyStop();
            setBoardOutputsLocked(0x00, false);
            board_->close();
        }
        if (fw_) {
            fw_->emergencyStop();
            closeFirmwarePathLocked();
        }
        telemetry_.usb_connected = false;
        telemetry_.board_connecting = false;
        telemetry_.axis_homed = {false, false, false, false};
        if (was_connected) {
            telemetry_.state = ControllerState::Disconnected;
            telemetry_.active_fault_message = "Board changed: press Connect";
        }
        ++caps_revision_;
    }

    config_ = cfg;
    board_id_ = cfg.board_id;
    telemetry_.board_id = board_id_;
    config_.workspace_revision  = revision;
    config_.tool_table_revision = tt_revision;
    ++config_revision_;

    telemetry_.operation_mode       = cfg.operation_mode;
    telemetry_.workspace_offsets_mm = cfg.workspace_offsets_mm;
    telemetry_.active_workspace     = cfg.active_workspace;
    // The active tool length offset follows the (possibly edited) table and mode
    updateActiveToolLengthLocked();
    refreshWorkFrameLocked();
    telemetry_.limit_override_active = cfg.limit_override;
    if (outputs_remapped) {
        telemetry_.outputs_bitmask     = 0;
        telemetry_.torch_or_spindle_on = false;
        manual_spindle_requested_      = false;
    }
    if (board_->isOpen() && !telemetry_.simulator_mode) {
        board_->configure(cfg, false);
        if (outputs_remapped) setBoardOutputsLocked(0, cfg.limit_override);
        if (steps_changed) {
            // Board step counters are in steps: re-express the current position with the new scale
            slice_->syncBoardPositionLocked();
        }
    }

    // Starting / stopping the VFD driver joins its thread: done by syncVfd() once the controller lock is released
    vfd_sync_needed_.store(true);
}

void MachineController::syncVfd() {
    std::lock_guard<std::mutex> lifecycle(vfd_lifecycle_mutex_);
    HuanyangVfdConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_.vfd;
        vfd_sync_needed_.store(false);
    }
    if (cfg.enabled && !vfd_controller_.isRunning() && hardware_access_enabled_.load()) {
        vfd_controller_.start(cfg);
    } else if (!cfg.enabled && vfd_controller_.isRunning()) {
        vfd_controller_.stop(); // Sends a stop command first if the spindle runs
    } else if (cfg.enabled) {
        vfd_controller_.updateConfig(cfg);
    }
}

std::shared_ptr<const MachineConfig> MachineController::getConfigShared() const {
    const auto wait_start = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    diag::controllerReadWait().add(Clock::now() - wait_start);
    // Comparing does not allocate; a copy is made only after a change
    if (!config_snapshot_ || !(*config_snapshot_ == config_)) {
        config_snapshot_ = std::make_shared<const MachineConfig>(config_);
    }
    return config_snapshot_;
}

MachineConfig MachineController::getConfig() const {
    const auto wait_start = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    diag::controllerReadWait().add(Clock::now() - wait_start);
    return config_;
}

void MachineController::setHuanyangVfdConfig(const HuanyangVfdConfig& config) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_.vfd = config;
        if (config_.vfd.min_rpm > config_.vfd.max_rpm) config_.vfd.min_rpm = config_.vfd.max_rpm;
        ++config_revision_;
    }
    syncVfd();
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
        const auto wait_start = Clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        diag::controllerReadWait().add(Clock::now() - wait_start);
        telem = telemetry_;
        telem.physical_outputs_bitmask = io::physicalOutputs(config_.io, telem.outputs_bitmask);
        telem.board_id = config_.board_id;
        telem.board_firmware_planned = firmwareBoardSelectedLocked();
        telem.board_busy = fwActiveLocked() && (fw_->busy() || !fw_->ready());
        telem.capabilities_revision = caps_revision_ + (fw_ ? fw_->capabilitiesRevision() * 1000003ULL : 0);
    }
    // Script state is queried without holding the controller lock (the script thread calls back into us)
    if (script_runner_) {
        telem.script_running = script_runner_->isRunning();
        telem.active_script_name = script_runner_->getActiveScriptName();
        scripting::PromptRequest req;
        telem.has_script_prompt = script_runner_->pendingPrompt(req); // One snapshot: the prompt cannot change in between
        if (telem.has_script_prompt) {
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
    {
        // Firmware board: soft reset at once (written straight to the port, safe from any thread)
        std::lock_guard<std::mutex> plock(fw_ptr_mutex_);
        if (fw_ && fw_->isOpen()) fw_->emergencyStop();
    }
    if (script_runner_) {
        script_runner_->cancel();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    faultLocked("EMERGENCY STOP TRIGGERED");
}

void MachineController::faultLocked(const std::string& message, ControllerState state) {
    const bool was_moving = motionActiveLocked() || motionLocked().decelerating();
    stopJobLocked(true);
    telemetry_.state = state;
    telemetry_.active_fault_message = message;
    config_.limit_override = false;
    telemetry_.limit_override_active = false;
    if (was_moving && !telemetry_.simulator_mode && !motionLocked().positionKnownAfterStop()) {
        // The board was stopped mid-motion and we cannot read back where it actually stopped
        telemetry_.axis_homed = {false, false, false, false};
        telemetry_.active_fault_message += " - position uncertain, re-home the machine";
    }
}

bool MachineController::resetFromEStop() {
    std::lock_guard<std::mutex> lock(mutex_);
    // A firmware board selected (connected or not) answers for itself; the RNR board and the simulator otherwise
    if (!telemetry_.simulator_mode && firmwareBoardSelectedLocked()) return firmware_->resetFromEStop();
    return slice_->resetFromEStop();
}

bool MachineController::jobActiveLocked(bool include_tool_change) const noexcept {
    const auto s = telemetry_.state;
    return motionActiveLocked() || s == ControllerState::PausedHold ||
           (include_tool_change && s == ControllerState::ToolChangeHold);
}

ControllerState MachineController::restStateLocked() const noexcept {
    return telemetry_.pending_tool_number > 0 ? ControllerState::ToolChangeHold : ControllerState::Idle;
}

bool MachineController::laserInterlocksOkLocked() const noexcept {
    const auto& lz = config_.laser;
    return !(lz.door_interlock_pin >= 1 && !telemetry_.laser_door_closed) &&
           !(lz.chiller_flow_pin >= 1 && !telemetry_.laser_chiller_flow_ok) &&
           !(lz.laser_fault_pin >= 1 && telemetry_.laser_fault_active);
}


bool MachineController::motionActiveLocked() const noexcept {
    const auto s = telemetry_.state;
    return s == ControllerState::RunningGCode || s == ControllerState::DwellPierce ||
           s == ControllerState::Jogging || s == ControllerState::Homing || s == ControllerState::Probing ||
           cont_jog_active_ || motionLocked().decelerating();
}

bool MachineController::readyForCommandLocked(bool allow_tool_change_hold, const char* what) {
    const auto s = telemetry_.state;
    const bool ok_state = (s == ControllerState::Idle) ||
                          (allow_tool_change_hold && s == ControllerState::ToolChangeHold);
    if (!ok_state || cont_jog_active_ || motionLocked().decelerating()) {
        return false;
    }
    if (motionLocked().boardBusy()) {
        telemetry_.active_fault_message = std::string(what) + " rejected: the board is busy";
        return false;
    }
    if (telemetry_.surface_probing_active) {
        telemetry_.active_fault_message = std::string(what) + " rejected: surface probing in progress";
        return false;
    }
    return true;
}

} // namespace rnr::core
