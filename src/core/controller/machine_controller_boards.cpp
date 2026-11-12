// MachineController: Scripting, board console and firmware settings, firmware-planned boards (GRBL family) (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

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
    if (fwActiveLocked() && (s == ControllerState::Jogging || s == ControllerState::Probing ||
                             s == ControllerState::Homing)) {
        fw_->stop();
    } else if (s == ControllerState::Jogging && !cont_jog_active_ && !slice_queue_.empty()) {
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

// ============================================================================
// Board selection, console & firmware settings
// ============================================================================
void MachineController::fillBoardLimitsLocked(MachineCheckInput::BoardLimits& b) const {
    b = {};
    if (!fw_ || !fw_->isOpen() || !fw_->ready() || telemetry_.simulator_mode) return;
    const auto& p = fw_->firmware();
    b.known = true;
    b.name = board::grbl::flavourName(p.flavour);
    b.axis_count = p.axis_count;
    b.aux_outputs = (p.flavour == board::grbl::Flavour::Grbl11 || p.flavour == board::grbl::Flavour::GenericGrbl)
                        ? 0 : static_cast<uint8_t>(std::min<int>(4, p.aux_outputs));
    if (p.flavour == board::grbl::Flavour::FluidNC) b.aux_outputs = 4; // user_outputs: not reported, not checked
    b.rates_known = p.settings_read && p.flavour != board::grbl::Flavour::FluidNC;
    for (size_t a = 0; a < 4; ++a) {
        b.max_rate_mm_min[a] = (a < p.axis_count) ? p.max_rate_mm_min[a] : 0.0;
        double lo = 0.0, hi = 0.0;
        b.travel_known[a] = fw_->travelEnvelope(a, lo, hi);
        b.travel_min[a] = lo;
        b.travel_max[a] = hi;
    }
    b.soft_limits = p.soft_limits;
}

board::BoardCapabilities MachineController::getBoardCapabilitiesLocked() const {
    if (fw_ && fw_->isOpen()) return fw_->capabilities();
    if (config_.board_id == board::kRnrBoardId) return board_->capabilities();
    return board::BoardRegistry::instance().capabilitiesFor(config_.board_id);
}

board::BoardCapabilities MachineController::getBoardCapabilities() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return getBoardCapabilitiesLocked();
}

bool MachineController::unlockBoard() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fwReadyLocked()) return false;
    std::string err;
    if (!fw_->unlock(&err)) {
        telemetry_.active_fault_message = "Unlock rejected: " + err;
        return false;
    }
    telemetry_.board_status_text.clear();
    return true;
}

void MachineController::setRapidOverride(float percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.rapid_override_pct = percent >= 75.0f ? 100.0f : (percent >= 37.5f ? 50.0f : 25.0f);
    if (fwActiveLocked()) fw_->setRapidOverride(telemetry_.rapid_override_pct);
}

bool MachineController::sendBoardConsoleLine(const std::string& line) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fwActiveLocked()) {
        telemetry_.active_fault_message = "The console needs a connected GRBL-family board";
        return false;
    }
    const auto s = telemetry_.state;
    const bool realtime = line == "?" || line == "!" || line == "~";
    if (!realtime && s != ControllerState::Idle && s != ControllerState::EStop && s != ControllerState::ToolChangeHold) {
        telemetry_.active_fault_message = "Console commands are accepted only while the machine is idle";
        return false;
    }
    std::string err;
    if (!fw_->sendConsoleLine(line, &err)) {
        telemetry_.active_fault_message = "Console: " + err;
        return false;
    }
    return true;
}

std::vector<transport::ProtocolLogEntry> MachineController::getBoardLog(uint64_t after_seq) const {
    std::shared_ptr<transport::ProtocolLog> log;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fw_) log = fw_->log();
    }
    return log ? log->since(after_seq) : std::vector<transport::ProtocolLogEntry>{};
}

bool MachineController::requestBoardReport() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fwActiveLocked() || !fw_->ready()) {
        telemetry_.active_fault_message = "The board report needs a connected GRBL-family board";
        return false;
    }
    const auto s = telemetry_.state;
    if (s != ControllerState::Idle && s != ControllerState::EStop && s != ControllerState::ToolChangeHold) {
        telemetry_.active_fault_message = "The board report is made only while the machine is idle";
        return false;
    }
    std::string err;
    if (!fw_->startBoardReport(&err)) {
        telemetry_.active_fault_message = "Board report: " + err;
        return false;
    }
    return true;
}

BoardReport MachineController::getBoardReport() const {
    std::lock_guard<std::mutex> lock(mutex_);
    BoardReport r;
    if (!fw_) return r;
    r.revision = fw_->boardReportRevision();
    r.running = fw_->boardReportRunning();
    if (!r.running && r.revision > 0) r.text = fw_->boardReportText();
    return r;
}

board::grbl::SettingsSnapshot MachineController::getFirmwareSettings() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fw_ ? fw_->settings() : board::grbl::SettingsSnapshot{};
}

void MachineController::requestFirmwareSettings() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fwReadyLocked() && !motionActiveLocked()) fw_->requestSettings();
}

bool MachineController::writeFirmwareSettings(const std::vector<std::pair<std::string, std::string>>& id_values) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fwReadyLocked() || motionActiveLocked()) {
        telemetry_.active_fault_message = "Board settings can only be changed while the machine is idle";
        return false;
    }
    std::string err;
    if (!fw_->writeSettings(id_values, &err)) {
        telemetry_.active_fault_message = "Board settings: " + err;
        return false;
    }
    return true;
}

// ============================================================================
// Firmware-planned boards (GRBL family)
// ============================================================================
bool MachineController::probeInputMissingLocked() const noexcept {
    // Firmware boards have a fixed probe terminal
    return !telemetry_.simulator_mode && !firmwareBoardSelectedLocked() && config_.probe.probe_pin == 0;
}

double MachineController::fwAxisMaxVelocityLocked(size_t axis) const {
    if (fw_ && axis < 4) {
        const double r = fw_->firmware().max_rate_mm_min[axis];
        if (r > 0.0) return r / 60.0;
    }
    return planner::TrajectoryPlanner::axisVelocityLimit(config_, std::min<size_t>(axis, 3));
}

bool MachineController::fwStartManualMovesLocked(const std::vector<ToolpathSegment>& moves, ControllerState state,
                                                 const motion::OutputRequest* during) {
    if (!fwReadyLocked()) {
        telemetry_.active_fault_message = "Move rejected: the board is not ready";
        return false;
    }
    std::vector<motion::MoveRequest> reqs;
    for (const auto& seg : moves) {
        motion::MoveRequest r;
        if (seg.type == SegmentType::Dwell) {
            if (seg.dwell_seconds <= 0.0) continue;
            r.dwell_s = seg.dwell_seconds;
            reqs.push_back(r);
            continue;
        }
        if (seg.type != SegmentType::RapidMove && seg.type != SegmentType::CutLinear &&
            seg.type != SegmentType::CutArcCW && seg.type != SegmentType::CutArcCCW) {
            continue;
        }
        // Work coordinates of the current frame (G53 previews included) -> machine
        r.target_mm = workToMachineLocked(seg.end_xyza_mm);
        r.rapid = seg.type == SegmentType::RapidMove;
        r.feed_mm_min = seg.feed_mm_min > 0.0 ? seg.feed_mm_min : 600.0;
        reqs.push_back(r);
    }
    if (reqs.empty()) return true;
    std::string err;
    if (!fw_->startMoves(reqs, during, &err)) {
        telemetry_.active_fault_message = "Move rejected: " + err;
        return false;
    }
    telemetry_.state = state;
    return true;
}

void MachineController::fwApplyOutputsLocked(uint8_t mask) {
    if (!fwReadyLocked() || mask == fw_outputs_applied_) return;
    const uint8_t main_bits = 0x0F;
    const bool main_changed = fw_outputs_applied_ == 0xFF || ((mask ^ fw_outputs_applied_) & main_bits) != 0;
    std::string err;
    if (main_changed) {
        motion::OutputRequest o;
        const bool laser = !config_.is_mill && config_.operation_mode == OperationMode::Laser;
        const bool app_spindle = config_.vfd.enabled && config_.is_mill;
        o.spindle_on = (mask & core::OutputBit::OUT1_TorchOrSpindle) != 0 && !app_spindle;
        o.spindle_ccw = config_.is_mill && (mask & core::OutputBit::OUT2_SpindleDir) != 0;
        if (laser) {
            o.laser = true;
            o.laser_dynamic = config_.laser.dynamic_power_m4;
            o.s_value = telemetry_.spindle_pwm_duty / 1000.0 * fw_->firmware().max_spindle_rpm;
        } else if (!config_.is_mill) {
            o.s_value = fw_->firmware().max_spindle_rpm; // Plasma torch on the spindle output: S0 may leave it off
        } else {
            o.s_value = telemetry_.commanded_spindle_rpm;
        }
        o.flood = (mask & core::OutputBit::OUT3_FloodCoolant) != 0;
        o.mist = (mask & core::OutputBit::OUT4_MistCoolant) != 0;
        if (!fw_->setOutputs(o, &err)) {
            telemetry_.active_fault_message = "Outputs: " + err;
            return;
        }
        fw_outputs_applied_ = static_cast<uint8_t>(((fw_outputs_applied_ == 0xFF ? 0 : fw_outputs_applied_) & ~main_bits) |
                                                   (mask & main_bits));
        return;
    }
    // Custom outputs: the board's aux outputs (grblHAL / FluidNC M64 / M65), one per tick
    for (int i = 0; i < 4; ++i) {
        const uint8_t bit = static_cast<uint8_t>(1U << (kCustomOutputShift + i));
        if ((mask & bit) == (fw_outputs_applied_ & bit)) continue;
        if (!fw_->setAuxOutput(i, (mask & bit) != 0, &err)) {
            telemetry_.active_fault_message = "Custom output: " + err;
            fw_outputs_applied_ = static_cast<uint8_t>((fw_outputs_applied_ & ~bit) | (mask & bit)); // Do not retry
            return;
        }
        fw_outputs_applied_ = static_cast<uint8_t>((fw_outputs_applied_ & ~bit) | (mask & bit));
        return;
    }
}

void MachineController::fwSpindleSpeedChangedLocked() {
    if (!fwReadyLocked()) return;
    if (fw_->jobActive()) {
        // During a job the program's S words run on the board: a manual speed change is a spindle override
        if (telemetry_.program_spindle_rpm > 0) {
            fw_->setSpindleOverride(100.0f * telemetry_.commanded_spindle_rpm / telemetry_.program_spindle_rpm);
        }
        return;
    }
    if ((telemetry_.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0) {
        fw_outputs_applied_ = 0xFF; // Send the spindle line again with the new S
        fwApplyOutputsLocked(telemetry_.outputs_bitmask);
    }
}

bool MachineController::fwStartJobLocked(size_t start_seg, bool resume_in_place) {
    job_frame_ = jobFrameLocked();
    size_t program_first = start_seg;
    std::array<double, 4> plan_start{};
    std::vector<ToolpathSegment> prefix = buildJobPrefixLocked(start_seg, resume_in_place, program_first, plan_start);

    motion::JobRequest jr;
    jr.program = program_;
    jr.prefix = std::move(prefix);
    jr.first_index = program_first;
    jr.frame = job_frame_;
    jr.arcs_allowed = !telemetry_.surface_compensation_active;
    jr.mill = config_.is_mill;
    jr.laser = !config_.is_mill && config_.operation_mode == OperationMode::Laser;
    jr.plasma = !config_.is_mill && config_.operation_mode == OperationMode::Plasma;
    jr.laser_lock_z = config_.laser.ignore_z_motion;
    jr.laser_z_plan = plan_start[2];
    jr.laser_dynamic = config_.laser.dynamic_power_m4;
    jr.laser_air_assist = config_.laser.auto_air_assist;
    jr.laser_min_duty_pct = config_.laser.min_duty_pct;
    jr.laser_max_duty_pct = config_.laser.max_duty_pct;
    jr.laser_s_max = config_.laser.s_max;
    jr.laser_power_override_pct = laser_power_override_pct_;
    jr.dry_run = telemetry_.dry_run_mode;
    jr.app_drives_spindle = config_.vfd.enabled && config_.is_mill;
    jr.spinup_dwell_s = config_.is_mill ? config_.spindle_ramp_up_s : 0.0;
    jr.pierce_delay_override_s = telemetry_.pierce_delay_override_s;
    jr.ends_with_program_end = program_->ends_with_program_end;
    jr.spindle_on_at_start = telemetry_.torch_or_spindle_on;
    jr.spindle_ccw_at_start = (telemetry_.outputs_bitmask & core::OutputBit::OUT2_SpindleDir) != 0;
    jr.rpm_at_start = telemetry_.commanded_spindle_rpm;
    fw_->setFeedOverride(telemetry_.feed_override_pct);
    std::string err;
    if (!fw_->startJob(std::move(jr), &err)) {
        telemetry_.active_fault_message = "Cycle Start rejected: " + err;
        return false;
    }
    fw_job_spindle_on_ = telemetry_.torch_or_spindle_on;
    fw_job_spindle_ccw_ = (telemetry_.outputs_bitmask & core::OutputBit::OUT2_SpindleDir) != 0;
    fw_seen_held_ = false;
    return true;
}

void MachineController::fwToolChangeLocked(const ToolpathSegment& seg) {
    // Same as the slice path (sendQueuedSliceLocked): spindle off, then m6.lua or retract and wait for the operator
    manual_spindle_requested_      = false;
    fw_job_spindle_on_             = false;
    telemetry_.torch_or_spindle_on = false;
    telemetry_.outputs_bitmask    &= ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
    telemetry_.axis_vel_mm_min     = {0.0, 0.0, 0.0, 0.0};
    telemetry_.pending_tool_number      = (seg.tool_number > 0) ? seg.tool_number : 1;
    telemetry_.pending_tool_description = toolDescriptionLocked(telemetry_.pending_tool_number);
    telemetry_.active_segment_index     = seg.segment_index + 1;
    telemetry_.active_gcode_line_index  = seg.gcode_line;
    telemetry_.segment_progress_0_to_1  = 0.0f;
    telemetry_.state = ControllerState::ToolChangeHold;
    fw_outputs_applied_ = 0xFF;
    fwApplyOutputsLocked(telemetry_.outputs_bitmask);

    if (runM6ScriptLocked()) return;
    const auto moves = toolChangeMovesLocked();
    if (!moves.empty()) fwStartManualMovesLocked(moves, ControllerState::Jogging); // MoveDone returns to ToolChangeHold
}

std::string MachineController::toolDescriptionLocked(uint16_t tool) const {
    const auto it = program_->tool_descriptions.find(tool);
    return it != program_->tool_descriptions.end() ? it->second : std::string();
}

bool MachineController::runM6ScriptLocked() {
    if (!config_.scripting.enabled || !config_.scripting.m6_script_enabled || !script_runner_) return false;
    const std::string resolved = scripting::resolveScriptPath(config_.scripting.m6_script_path);
    if (resolved.empty()) return false;
    script_runner_->runFileAsync(resolved, [this](bool ok, const std::string&) {
        // Operator confirmed "Tool Ready": continue the job
        if (ok) this->startJob();
    });
    return true;
}

std::vector<ToolpathSegment> MachineController::toolChangeMovesLocked() {
    std::array<double, 4> cur_w = telemetry_.work_pos_mm;
    const double target_z = std::max(cur_w[2], config_.tool_change.safe_z_mm);
    std::vector<ToolpathSegment> moves;
    auto addRapid = [&](const std::array<double, 4>& a, const std::array<double, 4>& b) {
        ToolpathSegment seg{};
        seg.type = SegmentType::RapidMove;
        seg.start_xyza_mm = a;
        seg.end_xyza_mm = b;
        seg.length_mm = dist3(a, b);
        moves.push_back(seg);
    };
    const std::array<double, 4> z_up = {cur_w[0], cur_w[1], target_z, cur_w[3]};
    if (target_z - cur_w[2] > 0.05) {
        if (!checkLinearMoveLocked(cur_w, z_up)) return moves; // Soft limit: no retract, and no park move either
        addRapid(cur_w, z_up);
        cur_w = z_up;
    }
    if (config_.tool_change.move_to_park_xy) {
        const std::array<double, 4> park = {config_.tool_change.park_x_mm, config_.tool_change.park_y_mm, cur_w[2], cur_w[3]};
        if (std::hypot(park[0] - cur_w[0], park[1] - cur_w[1]) > 0.05 && checkLinearMoveLocked(cur_w, park)) {
            addRapid(cur_w, park);
        }
    }
    return moves;
}

void MachineController::fwHandleEventLocked(const motion::PathEvent& e) {
    using K = motion::PathEvent::Kind;
    const auto s = telemetry_.state;
    switch (e.kind) {
        case K::Connected: {
            telemetry_.board_connecting = false;
            telemetry_.firmware_version = e.text;
            telemetry_.product_name = fw_->capabilities().display_name;
            ++caps_revision_;
            // Machines without home switches keep the machine position saved at the last exit (machine shift)
            const auto& fb = fw_->feedback();
            bool any_homed = false;
            for (bool h : fb.homed) any_homed = any_homed || h;
            std::array<double, 4> shift{};
            if (!config_.require_homing && !any_homed) {
                const auto fwpos = fw_->firmwareMachinePos();
                for (size_t a = 0; a < 4; ++a) shift[a] = telemetry_.machine_pos_mm[a] - fwpos[a];
            }
            fw_->setMachineShift(shift);
            telemetry_.axis_homed = fb.homed_reported ? fb.homed : std::array<bool, 4>{false, false, false, false};
            fw_->setFeedOverride(telemetry_.feed_override_pct);
            fw_outputs_applied_ = 0xFF;
            telemetry_.active_fault_message = "Connected to " + e.text + ". Press RESET E-STOP to enable the machine.";
            if (telemetry_.state != ControllerState::Disconnected) telemetry_.state = ControllerState::EStop;
            break;
        }
        case K::ConnectFailed:
            telemetry_.usb_connected = false;
            telemetry_.board_connecting = false;
            telemetry_.active_fault_message = e.text;
            telemetry_.board_status_text = e.text;
            if (!telemetry_.simulator_mode) telemetry_.state = ControllerState::Disconnected;
            break;
        case K::Fault: {
            if (e.position_lost) telemetry_.axis_homed = {false, false, false, false};
            const bool link_down = !fw_->isOpen();
            suppress_fw_estop_ = true; // The board has already stopped
            faultLocked(e.text, link_down ? ControllerState::Disconnected : ControllerState::EStop);
            suppress_fw_estop_ = false;
            if (link_down) {
                telemetry_.usb_connected = false;
                telemetry_.board_connecting = false;
            }
            break;
        }
        case K::Message:
            if (!e.text.empty()) telemetry_.active_fault_message = e.text;
            break;
        case K::MoveDone:
            if (s == ControllerState::Probing && !probe_move_is_search_ && active_probe_mode_ == ProbeMode::HoleCenterXY) {
                if (e.ok) {
                    continueHoleCenterLocked();
                } else {
                    active_probe_mode_ = ProbeMode::None;
                    telemetry_.state = restStateLocked();
                }
            } else if (s == ControllerState::Jogging || s == ControllerState::Probing) {
                telemetry_.state = restStateLocked();
                if (!e.ok && !e.text.empty()) telemetry_.active_fault_message = e.text;
            }
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            break;
        case K::ProbeDone:
            if (s == ControllerState::Probing) {
                probe_contact_captured_ = e.ok;
                if (e.ok) probe_contact_machine_ = e.position_mm;
                const auto& fb = fw_->feedback();
                telemetry_.machine_pos_mm = fb.machine_pos_mm;
                for (size_t a = 0; a < 4; ++a) telemetry_.work_pos_mm[a] = fb.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
                onProbeMoveFinishedLocked(e.ok);
            }
            break;
        case K::HomingDone:
            if (s == ControllerState::Homing) {
                telemetry_.state = ControllerState::Idle;
                if (e.ok) {
                    const uint8_t n = fw_->capabilities().axis_count;
                    for (size_t a = 0; a < 4; ++a) {
                        if (a < n && (fw_homing_mask_ == 0 || ((fw_homing_mask_ >> a) & 1U))) telemetry_.axis_homed[a] = true;
                    }
                    if (fw_homing_mask_ == 0 || (fw_homing_mask_ & 0x04)) {
                        tool_ref_contact_valid_ = false;
                        telemetry_.tool_reference_measured = false;
                    }
                    const auto& fb = fw_->feedback();
                    telemetry_.machine_pos_mm = fb.machine_pos_mm;
                    refreshWorkFrameLocked();
                    telemetry_.active_fault_message.clear();
                } else if (telemetry_.active_fault_message.empty()) {
                    telemetry_.active_fault_message = "Homing failed";
                }
            }
            break;
        case K::JogStopped:
            break; // updateContinuousJogLocked() ends the jog once the board is idle
        case K::JobSyncPoint: {
            const auto kind = static_cast<motion::SyncKind>(e.code);
            if (kind == motion::SyncKind::ToolChange) {
                if (e.segment_index < program_->segments.size()) {
                    fwToolChangeLocked(program_->segments[e.segment_index]);
                } else {
                    telemetry_.state = ControllerState::Idle;
                }
            } else if (kind == motion::SyncKind::ProgramStop) {
                telemetry_.state = ControllerState::PausedHold;
                manual_spindle_requested_ = false;
                fw_program_stop_ = true;
                fw_resume_segment_ = e.segment_index + 1;
                telemetry_.active_segment_index = e.segment_index;
                if (e.segment_index < program_->segments.size()) {
                    telemetry_.active_gcode_line_index = program_->segments[e.segment_index].gcode_line;
                }
                telemetry_.active_fault_message = "Program stop (M0/M1) - press Cycle Start to continue";
            } else {
                // App-driven spindle (Huanyang VFD): switch it with the machine at rest, wait for the speed
                const bool turning_on = e.spindle_on && !fw_job_spindle_on_;
                fw_job_spindle_on_ = e.spindle_on;
                fw_job_spindle_ccw_ = e.spindle_ccw;
                if (e.spindle_rpm > 0) {
                    telemetry_.program_spindle_rpm = e.spindle_rpm;
                    if (!telemetry_.spindle_override_active) {
                        telemetry_.commanded_spindle_rpm = e.spindle_rpm;
                        vfd_controller_.setRpm(e.spindle_rpm);
                    }
                }
                const double wait = e.spindle_on ? (turning_on ? config_.spindle_ramp_up_s : 0.5) : config_.spindle_ramp_down_s;
                fw_sync_wait_ = true;
                fw_sync_continue_at_ = Clock::now() + std::chrono::milliseconds(static_cast<int>(std::max(0.0, wait) * 1000.0));
            }
            break;
        }
        case K::JobFinished:
            if (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce || s == ControllerState::PausedHold) {
                manual_spindle_requested_ = false;
                fw_job_spindle_on_ = false;
                telemetry_.torch_or_spindle_on = false;
                telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
                if (e.ok) {
                    if (program_->ends_with_program_end) {
                        telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant);
                    }
                    telemetry_.active_segment_index = program_->segments.size();
                    telemetry_.segment_progress_0_to_1 = 1.0f;
                } else {
                    // The stop sequence resets the board, which switches spindle and coolant off
                    telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant);
                    telemetry_.laser_air_assist_active = false;
                    if (telemetry_.active_fault_message.empty() || telemetry_.active_fault_message == "Stopping...") {
                        telemetry_.active_fault_message = "Job stopped";
                    }
                }
                fw_outputs_applied_ = 0xFF;
                telemetry_.state = restStateLocked();
            }
            break;
        case K::SettingsChanged:
            ++caps_revision_;
            break;
    }
}

void MachineController::fwTickLocked() {
    if (!fw_) return;
    const auto now = Clock::now();
    fw_->tick(now);
    motion::PathEvent e;
    while (fw_ && fw_->pollEvent(e)) fwHandleEventLocked(e);
    if (!fw_) return;
    if (!fw_->isOpen()) {
        if (telemetry_.usb_connected) {
            telemetry_.usb_connected = false;
            telemetry_.board_connecting = false;
            if (!telemetry_.simulator_mode && telemetry_.state != ControllerState::Disconnected) {
                telemetry_.state = ControllerState::Disconnected;
            }
        }
        return;
    }
    const auto& fb = fw_->feedback();
    telemetry_.board_status_text = fb.status_text;
    telemetry_.firmware_state = fb.firmware_state;
    telemetry_.rapid_override_pct = static_cast<float>(fb.ov_rapid);
    if (!fw_->ready() || !fb.position_valid) return;

    telemetry_.machine_pos_mm = fb.machine_pos_mm;
    for (size_t a = 0; a < 4; ++a) telemetry_.work_pos_mm[a] = fb.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
    // The board reports only the path feed: per-axis velocities from the position change (every ~100 ms)
    const double vel_dt = std::chrono::duration<double>(now - fw_vel_time_).count();
    if (vel_dt >= 0.09) {
        const bool moving = fb.feed_mm_min > 0.5 && vel_dt < 1.0;
        // Direction from the position change, speed from the board's reported feed
        double len = 0.0;
        for (size_t a = 0; a < 4; ++a) len += (fb.machine_pos_mm[a] - fw_vel_pos_[a]) * (fb.machine_pos_mm[a] - fw_vel_pos_[a]);
        len = std::sqrt(len);
        for (size_t a = 0; a < 4; ++a) {
            telemetry_.axis_vel_mm_min[a] = (moving && len > 1e-6)
                ? std::abs(fb.machine_pos_mm[a] - fw_vel_pos_[a]) / len * fb.feed_mm_min : 0.0;
        }
        fw_vel_pos_ = fb.machine_pos_mm;
        fw_vel_time_ = now;
    }
    if (fb.homed_reported) {
        const uint8_t n = fw_->capabilities().axis_count;
        for (size_t a = 0; a < 4; ++a) telemetry_.axis_homed[a] = a < n && fb.homed[a];
    }
    const auto s = telemetry_.state;
    const bool job = s == ControllerState::RunningGCode || s == ControllerState::DwellPierce || s == ControllerState::PausedHold;
    if (job && fb.job_active) {
        telemetry_.active_segment_index    = fb.active_segment;
        telemetry_.active_gcode_line_index = fb.active_line;
        telemetry_.segment_progress_0_to_1 = fb.segment_progress;
        telemetry_.progress_is_estimate    = fb.progress_is_estimate;
        telemetry_.outputs_bitmask = static_cast<uint8_t>(
            (telemetry_.outputs_bitmask & ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant)) |
            (fb.flood_on ? core::OutputBit::OUT3_FloodCoolant : 0) | (fb.mist_on ? core::OutputBit::OUT4_MistCoolant : 0));
        fw_outputs_applied_ = telemetry_.outputs_bitmask;
    }
    // State changes the board made itself (its own hold / start buttons, the door)
    if (now < fw_state_grace_until_) {
        // Just paused / resumed by the app: the board's next report may still show the old state
    } else if (s == ControllerState::RunningGCode && (fb.state == motion::PathState::Held || fb.state == motion::PathState::Holding)) {
        telemetry_.state = ControllerState::PausedHold;
        fw_seen_held_ = true;
        if (fb.door_open) telemetry_.active_fault_message = "Door open: close it and press Cycle Start";
    } else if (s == ControllerState::PausedHold && fw_->jobActive()) {
        if (fb.state == motion::PathState::Held || fb.state == motion::PathState::Holding) {
            fw_seen_held_ = true;
        } else if (fb.state == motion::PathState::Moving && fw_seen_held_) {
            telemetry_.state = ControllerState::RunningGCode; // Resumed from the board's cycle start button
            fw_seen_held_ = false;
        }
    }
    if (fw_sync_wait_ && now >= fw_sync_continue_at_) {
        fw_sync_wait_ = false;
        fw_->continueJob();
    }
}

} // namespace rnr::core
