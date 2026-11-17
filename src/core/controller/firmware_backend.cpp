// FirmwareBackend: motion on firmware-planned boards (GRBL family). See firmware_backend.h.
#include "core/controller/firmware_backend.h"
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

FirmwareBackend::FirmwareBackend(MachineController& mc) : mc_(mc) {}

double FirmwareBackend::fwAxisMaxVelocityLocked(size_t axis) const {
    if (mc_.fw_ && axis < 4) {
        const double r = mc_.fw_->firmware().max_rate_mm_min[axis];
        if (r > 0.0) return r / 60.0;
    }
    return planner::TrajectoryPlanner::axisVelocityLimit(mc_.config_, std::min<size_t>(axis, 3));
}

bool FirmwareBackend::fwStartManualMovesLocked(const std::vector<ToolpathSegment>& moves, ControllerState state,
                                                 const motion::OutputRequest* during) {
    if (!mc_.fwReadyLocked()) {
        mc_.telemetry_.active_fault_message = "Move rejected: the board is not ready";
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
        r.target_mm = mc_.workToMachineLocked(seg.end_xyza_mm);
        r.rapid = seg.type == SegmentType::RapidMove;
        r.feed_mm_min = seg.feed_mm_min > 0.0 ? seg.feed_mm_min : 600.0;
        reqs.push_back(r);
    }
    if (reqs.empty()) return true;
    std::string err;
    if (!mc_.fw_->startMoves(reqs, during, &err)) {
        mc_.telemetry_.active_fault_message = "Move rejected: " + err;
        return false;
    }
    mc_.telemetry_.state = state;
    return true;
}

void FirmwareBackend::fwApplyOutputsLocked(uint8_t mask) {
    if (!mc_.fwReadyLocked() || mask == fw_outputs_applied_) return;
    const uint8_t main_bits = 0x0F;
    const bool main_changed = fw_outputs_applied_ == 0xFF || ((mask ^ fw_outputs_applied_) & main_bits) != 0;
    std::string err;
    if (main_changed) {
        motion::OutputRequest o;
        const bool laser = !mc_.config_.is_mill && mc_.config_.operation_mode == OperationMode::Laser;
        const bool app_spindle = mc_.config_.vfd.enabled && mc_.config_.is_mill;
        o.spindle_on = (mask & core::OutputBit::OUT1_TorchOrSpindle) != 0 && !app_spindle;
        o.spindle_ccw = mc_.config_.is_mill && (mask & core::OutputBit::OUT2_SpindleDir) != 0;
        if (laser) {
            o.laser = true;
            o.laser_dynamic = mc_.config_.laser.dynamic_power_m4;
            o.s_value = mc_.telemetry_.spindle_pwm_duty / 1000.0 * mc_.fw_->firmware().max_spindle_rpm;
        } else if (!mc_.config_.is_mill) {
            o.s_value = mc_.fw_->firmware().max_spindle_rpm; // Plasma torch on the spindle output: S0 may leave it off
        } else {
            o.s_value = mc_.telemetry_.commanded_spindle_rpm;
        }
        o.flood = (mask & core::OutputBit::OUT3_FloodCoolant) != 0;
        o.mist = (mask & core::OutputBit::OUT4_MistCoolant) != 0;
        if (!mc_.fw_->setOutputs(o, &err)) {
            mc_.telemetry_.active_fault_message = "Outputs: " + err;
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
        if (!mc_.fw_->setAuxOutput(i, (mask & bit) != 0, &err)) {
            mc_.telemetry_.active_fault_message = "Custom output: " + err;
            fw_outputs_applied_ = static_cast<uint8_t>((fw_outputs_applied_ & ~bit) | (mask & bit)); // Do not retry
            return;
        }
        fw_outputs_applied_ = static_cast<uint8_t>((fw_outputs_applied_ & ~bit) | (mask & bit));
        return;
    }
}

void FirmwareBackend::fwSpindleSpeedChangedLocked() {
    if (!mc_.fwReadyLocked()) return;
    if (mc_.fw_->jobActive()) {
        // During a job the program's S words run on the board: a manual speed change is a spindle override
        if (mc_.telemetry_.program_spindle_rpm > 0) {
            mc_.fw_->setSpindleOverride(100.0f * mc_.telemetry_.commanded_spindle_rpm / mc_.telemetry_.program_spindle_rpm);
        }
        return;
    }
    if ((mc_.telemetry_.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0) {
        fw_outputs_applied_ = 0xFF; // Send the spindle line again with the new S
        fwApplyOutputsLocked(mc_.telemetry_.outputs_bitmask);
    }
}

bool FirmwareBackend::fwStartJobLocked(size_t start_seg, bool resume_in_place) {
    mc_.job_frame_ = mc_.jobFrameLocked();
    size_t program_first = start_seg;
    std::array<double, 4> plan_start{};
    std::vector<ToolpathSegment> prefix = mc_.buildJobPrefixLocked(start_seg, resume_in_place, program_first, plan_start);

    motion::JobRequest jr;
    jr.program = mc_.program_;
    jr.prefix = std::move(prefix);
    jr.first_index = program_first;
    jr.frame = mc_.job_frame_;
    jr.arcs_allowed = !mc_.telemetry_.surface_compensation_active;
    jr.mill = mc_.config_.is_mill;
    jr.laser = !mc_.config_.is_mill && mc_.config_.operation_mode == OperationMode::Laser;
    jr.plasma = !mc_.config_.is_mill && mc_.config_.operation_mode == OperationMode::Plasma;
    jr.laser_lock_z = mc_.config_.laser.ignore_z_motion;
    jr.laser_z_plan = plan_start[2];
    jr.laser_dynamic = mc_.config_.laser.dynamic_power_m4;
    jr.laser_air_assist = mc_.config_.laser.auto_air_assist;
    jr.laser_min_duty_pct = mc_.config_.laser.min_duty_pct;
    jr.laser_max_duty_pct = mc_.config_.laser.max_duty_pct;
    jr.laser_s_max = mc_.config_.laser.s_max;
    jr.laser_power_override_pct = mc_.laser_power_override_pct_;
    jr.dry_run = mc_.telemetry_.dry_run_mode;
    jr.app_drives_spindle = mc_.config_.vfd.enabled && mc_.config_.is_mill;
    jr.spinup_dwell_s = mc_.config_.is_mill ? mc_.config_.spindle_ramp_up_s : 0.0;
    jr.pierce_delay_override_s = mc_.telemetry_.pierce_delay_override_s;
    jr.ends_with_program_end = mc_.program_->ends_with_program_end;
    jr.spindle_on_at_start = mc_.telemetry_.torch_or_spindle_on;
    jr.spindle_ccw_at_start = (mc_.telemetry_.outputs_bitmask & core::OutputBit::OUT2_SpindleDir) != 0;
    jr.rpm_at_start = mc_.telemetry_.commanded_spindle_rpm;
    mc_.fw_->setFeedOverride(mc_.telemetry_.feed_override_pct);
    std::string err;
    if (!mc_.fw_->startJob(std::move(jr), &err)) {
        mc_.telemetry_.active_fault_message = "Cycle Start rejected: " + err;
        return false;
    }
    fw_job_spindle_on_ = mc_.telemetry_.torch_or_spindle_on;
    fw_job_spindle_ccw_ = (mc_.telemetry_.outputs_bitmask & core::OutputBit::OUT2_SpindleDir) != 0;
    fw_seen_held_ = false;
    return true;
}

void FirmwareBackend::fwToolChangeLocked(const ToolpathSegment& seg) {
    // Same as the slice path (sendQueuedSliceLocked): spindle off, then m6.lua or retract and wait for the operator
    mc_.manual_spindle_requested_      = false;
    fw_job_spindle_on_             = false;
    mc_.telemetry_.torch_or_spindle_on = false;
    mc_.telemetry_.outputs_bitmask    &= ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
    mc_.telemetry_.axis_vel_mm_min     = {0.0, 0.0, 0.0, 0.0};
    mc_.telemetry_.pending_tool_number      = (seg.tool_number > 0) ? seg.tool_number : 1;
    mc_.telemetry_.pending_tool_description = mc_.toolDescriptionLocked(mc_.telemetry_.pending_tool_number);
    mc_.telemetry_.active_segment_index     = seg.segment_index + 1;
    mc_.telemetry_.active_gcode_line_index  = seg.gcode_line;
    mc_.telemetry_.segment_progress_0_to_1  = 0.0f;
    mc_.telemetry_.state = ControllerState::ToolChangeHold;
    fw_outputs_applied_ = 0xFF;
    fwApplyOutputsLocked(mc_.telemetry_.outputs_bitmask);

    if (mc_.runM6ScriptLocked()) return;
    const auto moves = mc_.toolChangeMovesLocked();
    if (!moves.empty()) fwStartManualMovesLocked(moves, ControllerState::Jogging); // MoveDone returns to ToolChangeHold
}

void FirmwareBackend::fwHandleEventLocked(const motion::PathEvent& e) {
    using K = motion::PathEvent::Kind;
    const auto s = mc_.telemetry_.state;
    switch (e.kind) {
        case K::Connected: {
            mc_.telemetry_.board_connecting = false;
            mc_.telemetry_.firmware_version = e.text;
            mc_.telemetry_.product_name = mc_.fw_->capabilities().display_name;
            ++mc_.caps_revision_;
            // Machines without home switches keep the machine position saved at the last exit (machine shift)
            const auto& fb = mc_.fw_->feedback();
            bool any_homed = false;
            for (bool h : fb.homed) any_homed = any_homed || h;
            std::array<double, 4> shift{};
            if (!mc_.config_.require_homing && !any_homed) {
                const auto fwpos = mc_.fw_->firmwareMachinePos();
                for (size_t a = 0; a < 4; ++a) shift[a] = mc_.telemetry_.machine_pos_mm[a] - fwpos[a];
            }
            mc_.fw_->setMachineShift(shift);
            mc_.telemetry_.axis_homed = fb.homed_reported ? fb.homed : std::array<bool, 4>{false, false, false, false};
            mc_.fw_->setFeedOverride(mc_.telemetry_.feed_override_pct);
            fw_outputs_applied_ = 0xFF;
            mc_.telemetry_.active_fault_message = "Connected to " + e.text + ". Press RESET E-STOP to enable the machine.";
            if (mc_.telemetry_.state != ControllerState::Disconnected) mc_.telemetry_.state = ControllerState::EStop;
            break;
        }
        case K::ConnectFailed:
            mc_.telemetry_.usb_connected = false;
            mc_.telemetry_.board_connecting = false;
            mc_.telemetry_.active_fault_message = e.text;
            mc_.telemetry_.board_status_text = e.text;
            if (!mc_.telemetry_.simulator_mode) mc_.telemetry_.state = ControllerState::Disconnected;
            break;
        case K::Fault: {
            if (e.position_lost) mc_.telemetry_.axis_homed = {false, false, false, false};
            const bool link_down = !mc_.fw_->isOpen();
            suppress_fw_estop_ = true; // The board has already stopped
            mc_.faultLocked(e.text, link_down ? ControllerState::Disconnected : ControllerState::EStop);
            suppress_fw_estop_ = false;
            if (link_down) {
                mc_.telemetry_.usb_connected = false;
                mc_.telemetry_.board_connecting = false;
            }
            break;
        }
        case K::Message:
            if (!e.text.empty()) mc_.telemetry_.active_fault_message = e.text;
            break;
        case K::MoveDone: {
            // The path reports the end after a fresh Idle status: take that position now, before a sequence (hole
            // centre) plans its next move from it. The telemetry is otherwise only updated after the events.
            const auto& fb = mc_.fw_->feedback();
            if (fb.position_valid) {
                mc_.telemetry_.machine_pos_mm = fb.machine_pos_mm;
                for (size_t a = 0; a < 4; ++a) {
                    mc_.telemetry_.work_pos_mm[a] = fb.machine_pos_mm[a] - mc_.telemetry_.work_offset_mm[a];
                }
            }
            if (s == ControllerState::Probing && !mc_.probe_move_is_search_ && mc_.active_probe_mode_ == MachineController::ProbeMode::HoleCenterXY) {
                if (e.ok) {
                    mc_.continueHoleCenterLocked();
                } else {
                    mc_.active_probe_mode_ = MachineController::ProbeMode::None;
                    mc_.telemetry_.state = mc_.restStateLocked();
                }
            } else if (s == ControllerState::Jogging || s == ControllerState::Probing) {
                mc_.telemetry_.state = mc_.restStateLocked();
                if (!e.ok && !e.text.empty()) mc_.telemetry_.active_fault_message = e.text;
            }
            mc_.telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            break;
        }
        case K::ProbeDone:
            if (s == ControllerState::Probing) {
                mc_.probe_contact_captured_ = e.ok;
                if (e.ok) mc_.probe_contact_machine_ = e.position_mm;
                const auto& fb = mc_.fw_->feedback();
                mc_.telemetry_.machine_pos_mm = fb.machine_pos_mm;
                for (size_t a = 0; a < 4; ++a) mc_.telemetry_.work_pos_mm[a] = fb.machine_pos_mm[a] - mc_.telemetry_.work_offset_mm[a];
                mc_.onProbeMoveFinishedLocked(e.ok);
            }
            break;
        case K::HomingDone:
            if (s == ControllerState::Homing) {
                mc_.telemetry_.state = ControllerState::Idle;
                if (e.ok) {
                    const uint8_t n = mc_.fw_->capabilities().axis_count;
                    for (size_t a = 0; a < 4; ++a) {
                        if (a < n && (fw_homing_mask_ == 0 || ((fw_homing_mask_ >> a) & 1U))) mc_.telemetry_.axis_homed[a] = true;
                    }
                    if (fw_homing_mask_ == 0 || (fw_homing_mask_ & 0x04)) {
                        mc_.tool_ref_contact_valid_ = false;
                        mc_.telemetry_.tool_reference_measured = false;
                    }
                    const auto& fb = mc_.fw_->feedback();
                    mc_.telemetry_.machine_pos_mm = fb.machine_pos_mm;
                    mc_.refreshWorkFrameLocked();
                    mc_.telemetry_.active_fault_message.clear();
                } else if (mc_.telemetry_.active_fault_message.empty()) {
                    mc_.telemetry_.active_fault_message = "Homing failed";
                }
            }
            break;
        case K::JogStopped:
            break; // updateContinuousJogLocked() ends the jog once the board is idle
        case K::JobSyncPoint: {
            const auto kind = static_cast<motion::SyncKind>(e.code);
            if (kind == motion::SyncKind::ToolChange) {
                if (e.segment_index < mc_.program_->segments.size()) {
                    fwToolChangeLocked(mc_.program_->segments[e.segment_index]);
                } else {
                    mc_.telemetry_.state = ControllerState::Idle;
                }
            } else if (kind == motion::SyncKind::ProgramStop) {
                mc_.telemetry_.state = ControllerState::PausedHold;
                mc_.manual_spindle_requested_ = false;
                fw_program_stop_ = true;
                fw_resume_segment_ = e.segment_index + 1;
                mc_.telemetry_.active_segment_index = e.segment_index;
                if (e.segment_index < mc_.program_->segments.size()) {
                    mc_.telemetry_.active_gcode_line_index = mc_.program_->segments[e.segment_index].gcode_line;
                }
                mc_.telemetry_.active_fault_message = "Program stop (M0/M1) - press Cycle Start to continue";
            } else {
                // App-driven spindle (Huanyang VFD): switch it with the machine at rest, wait for the speed
                const bool turning_on = e.spindle_on && !fw_job_spindle_on_;
                fw_job_spindle_on_ = e.spindle_on;
                fw_job_spindle_ccw_ = e.spindle_ccw;
                if (e.spindle_rpm > 0) {
                    mc_.telemetry_.program_spindle_rpm = e.spindle_rpm;
                    if (!mc_.telemetry_.spindle_override_active) {
                        mc_.telemetry_.commanded_spindle_rpm = e.spindle_rpm;
                        mc_.vfd_controller_.setRpm(e.spindle_rpm);
                    }
                }
                const double wait = e.spindle_on ? (turning_on ? mc_.config_.spindle_ramp_up_s : 0.5) : mc_.config_.spindle_ramp_down_s;
                fw_sync_wait_ = true;
                fw_sync_continue_at_ = Clock::now() + std::chrono::milliseconds(static_cast<int>(std::max(0.0, wait) * 1000.0));
            }
            break;
        }
        case K::JobFinished:
            if (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce || s == ControllerState::PausedHold) {
                mc_.manual_spindle_requested_ = false;
                fw_job_spindle_on_ = false;
                mc_.telemetry_.torch_or_spindle_on = false;
                mc_.telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
                if (e.ok) {
                    if (mc_.program_->ends_with_program_end) {
                        mc_.telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant);
                    }
                    mc_.telemetry_.active_segment_index = mc_.program_->segments.size();
                    mc_.telemetry_.segment_progress_0_to_1 = 1.0f;
                } else {
                    // The stop sequence resets the board, which switches spindle and coolant off
                    mc_.telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant);
                    mc_.telemetry_.laser_air_assist_active = false;
                    if (mc_.telemetry_.active_fault_message.empty() || mc_.telemetry_.active_fault_message == "Stopping...") {
                        mc_.telemetry_.active_fault_message = "Job stopped";
                    }
                }
                fw_outputs_applied_ = 0xFF;
                mc_.telemetry_.state = mc_.restStateLocked();
            }
            break;
        case K::SettingsChanged:
            ++mc_.caps_revision_;
            break;
    }
}

void FirmwareBackend::fwTickLocked() {
    if (!mc_.fw_) return;
    const auto now = Clock::now();
    mc_.fw_->tick(now);
    motion::PathEvent e;
    while (mc_.fw_ && mc_.fw_->pollEvent(e)) fwHandleEventLocked(e);
    if (!mc_.fw_) return;
    if (!mc_.fw_->isOpen()) {
        if (mc_.telemetry_.usb_connected) {
            mc_.telemetry_.usb_connected = false;
            mc_.telemetry_.board_connecting = false;
            if (!mc_.telemetry_.simulator_mode && mc_.telemetry_.state != ControllerState::Disconnected) {
                mc_.telemetry_.state = ControllerState::Disconnected;
            }
        }
        return;
    }
    const auto& fb = mc_.fw_->feedback();
    mc_.telemetry_.board_status_text = fb.status_text;
    mc_.telemetry_.firmware_state = fb.firmware_state;
    mc_.telemetry_.rapid_override_pct = static_cast<float>(fb.ov_rapid);
    if (!mc_.fw_->ready() || !fb.position_valid) return;

    mc_.telemetry_.machine_pos_mm = fb.machine_pos_mm;
    for (size_t a = 0; a < 4; ++a) mc_.telemetry_.work_pos_mm[a] = fb.machine_pos_mm[a] - mc_.telemetry_.work_offset_mm[a];
    // The board reports only the path feed: per-axis velocities from the position change (every ~100 ms)
    const double vel_dt = std::chrono::duration<double>(now - fw_vel_time_).count();
    if (vel_dt >= 0.09) {
        const bool moving = fb.feed_mm_min > 0.5 && vel_dt < 1.0;
        // Direction from the position change, speed from the board's reported feed
        double len = 0.0;
        for (size_t a = 0; a < 4; ++a) len += (fb.machine_pos_mm[a] - fw_vel_pos_[a]) * (fb.machine_pos_mm[a] - fw_vel_pos_[a]);
        len = std::sqrt(len);
        for (size_t a = 0; a < 4; ++a) {
            mc_.telemetry_.axis_vel_mm_min[a] = (moving && len > 1e-6)
                ? std::abs(fb.machine_pos_mm[a] - fw_vel_pos_[a]) / len * fb.feed_mm_min : 0.0;
        }
        fw_vel_pos_ = fb.machine_pos_mm;
        fw_vel_time_ = now;
    }
    if (fb.homed_reported) {
        const uint8_t n = mc_.fw_->capabilities().axis_count;
        for (size_t a = 0; a < 4; ++a) mc_.telemetry_.axis_homed[a] = a < n && fb.homed[a];
    }
    const auto s = mc_.telemetry_.state;
    const bool job = s == ControllerState::RunningGCode || s == ControllerState::DwellPierce || s == ControllerState::PausedHold;
    if (job && fb.job_active) {
        mc_.telemetry_.active_segment_index    = fb.active_segment;
        mc_.telemetry_.active_gcode_line_index = fb.active_line;
        mc_.telemetry_.segment_progress_0_to_1 = fb.segment_progress;
        mc_.telemetry_.progress_is_estimate    = fb.progress_is_estimate;
        mc_.telemetry_.outputs_bitmask = static_cast<uint8_t>(
            (mc_.telemetry_.outputs_bitmask & ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant)) |
            (fb.flood_on ? core::OutputBit::OUT3_FloodCoolant : 0) | (fb.mist_on ? core::OutputBit::OUT4_MistCoolant : 0));
        fw_outputs_applied_ = mc_.telemetry_.outputs_bitmask;
    }
    // State changes the board made itself (its own hold / start buttons, the door)
    if (now < fw_state_grace_until_) {
        // Just paused / resumed by the app: the board's next report may still show the old state
    } else if (s == ControllerState::RunningGCode && (fb.state == motion::PathState::Held || fb.state == motion::PathState::Holding)) {
        mc_.telemetry_.state = ControllerState::PausedHold;
        fw_seen_held_ = true;
        if (fb.door_open) mc_.telemetry_.active_fault_message = "Door open: close it and press Cycle Start";
    } else if (s == ControllerState::PausedHold && mc_.fw_->jobActive()) {
        if (fb.state == motion::PathState::Held || fb.state == motion::PathState::Holding) {
            fw_seen_held_ = true;
        } else if (fb.state == motion::PathState::Moving && fw_seen_held_) {
            mc_.telemetry_.state = ControllerState::RunningGCode; // Resumed from the board's cycle start button
            fw_seen_held_ = false;
        }
    }
    if (fw_sync_wait_ && now >= fw_sync_continue_at_) {
        fw_sync_wait_ = false;
        mc_.fw_->continueJob();
    }
}

// ============================================================================
// IMotionBackend
// ============================================================================
void FirmwareBackend::readBoard(bool, const board::BoardStatus&, bool) {
    fwTickLocked();
}

void FirmwareBackend::onBoardConnected() {
    fw_outputs_applied_ = 0xFF;
}

bool FirmwareBackend::resetFromEStop() {
    auto& t = mc_.telemetry_;
    const auto& cfg = mc_.config_;
    if (!mc_.fwActiveLocked()) {
        t.active_fault_message = "Cannot reset: the board is not connected";
        return false;
    }
    auto& fw = *mc_.fw_;
    if (!fw.ready()) {
        t.active_fault_message = "Cannot reset: the board is still connecting";
        return false;
    }
    mc_.evaluateInputsAndEndStopsLocked();
    if (fw.feedback().estop_active) {
        t.active_fault_message = "Cannot reset: the board's E-stop input is still active";
        return false;
    }
    if (cfg.vfd.enabled) {
        const auto vt = mc_.vfd_controller_.getTelemetry();
        if (vt.anyAlarm()) {
            t.active_fault_message = "Cannot reset: clear the VFD alarm first (" + vt.last_error_msg + ")";
            return false;
        }
    }
    if (fw.inAlarm()) {
        std::string err;
        if (!fw.unlock(&err)) {
            t.active_fault_message = "Cannot reset: " + err;
            return false;
        }
    }
    fw_outputs_applied_ = 0xFF;
    t.limit_switch_triggered = false;
    t.soft_limit_triggered   = false;
    t.active_fault_message.clear();
    if (cfg.require_homing) {
        for (size_t a = 0; a < 3; ++a) {
            if (!t.axis_homed[a]) {
                t.active_fault_message = "Machine not homed: home it before running programs in machine coordinates";
                break;
            }
        }
    }
    t.state = ControllerState::Idle;
    return true;
}

bool FirmwareBackend::readInputs() {
    // Inputs are the board's fixed terminals (Pn:); limits and the door are handled by the board
    auto& t = mc_.telemetry_;
    const auto& fw = *mc_.fw_;
    const auto& fb = fw.feedback();
    t.inputs_bitmask       = 0;
    t.board_inputs_mask    = fb.inputs_mask;
    t.probe_contact_active = fb.probe_active;
    t.thc_on_board         = fw.boardRunsThc();
    t.thc_arc_ok_active    = fb.arc_ok;
    t.thc_up_active        = fb.thc_up;
    t.thc_down_active      = fb.thc_down;
    t.thc_anti_dive_locked = fb.thc_velocity_lock || fb.thc_void_lock;
    t.thc_board_active     = fb.thc_active;
    t.thc_torch_on         = fb.torch_on;
    t.thc_arc_voltage      = fb.arc_voltage;
    if (!t.thc_on_board) {
        t.thc_board_mode.clear();
    } else if (fw_thc_mode_rev_ != fw.capabilitiesRevision() || t.thc_board_mode.empty()) {
        fw_thc_mode_rev_ = fw.capabilitiesRevision();
        t.thc_board_mode = fw.capabilities().board_thc_mode;
    }
    if (fb.arc_failed_count != fw_arc_failed_seen_) {
        fw_arc_failed_seen_ = fb.arc_failed_count;
        t.active_fault_message = "The torch did not start (no Arc OK): the board paused the job. "
                                 "Check the torch and press Cycle Start, or Stop.";
    }
    t.limit_switch_triggered = fb.limit_active && t.state == ControllerState::EStop;
    if (mc_.config_.operation_mode == OperationMode::Laser && !mc_.config_.is_mill) {
        t.laser_door_closed     = !fb.door_open;
        t.laser_chiller_flow_ok = true;
        t.laser_fault_active    = false;
    }
    t.estop_input_active = fb.estop_active;
    if (fb.estop_active && t.state != ControllerState::EStop && t.state != ControllerState::Disconnected) {
        mc_.faultLocked("E-STOP INPUT ON THE BOARD TRIGGERED");
    }
    return true;
}

void FirmwareBackend::tick(bool gcode_running) {
    // The board runs the motion: only the job clock advances here
    if (gcode_running) mc_.telemetry_.job_elapsed_seconds += kDtSec;
}

void FirmwareBackend::updateFeedTelemetry() {
    mc_.telemetry_.current_feed_mm_min = mc_.fw_->feedback().feed_mm_min;
}

void FirmwareBackend::updateContinuousJog() {
    // The core keeps the dead-man and the target velocity, the jog feeder streams $J= lines
    auto& t = mc_.telemetry_;
    const auto now = Clock::now();
    std::array<double, 4> vel{};
    bool any_held = false;
    for (size_t a = 0; a < 4; ++a) {
        if (mc_.cont_jog_held_[a] && now - mc_.cont_jog_refresh_[a] > std::chrono::milliseconds(MachineController::kJogDeadmanMs)) {
            mc_.cont_jog_held_[a] = false; // Dead-man: no refresh = released
            mc_.cont_jog_target_vel_mm_s_[a] = 0.0;
        }
        if (mc_.cont_jog_held_[a]) {
            vel[a] = mc_.cont_jog_target_vel_mm_s_[a];
            any_held = true;
        }
    }
    mc_.fw_->setJogVelocity(vel);
    if (!any_held && !mc_.fw_->jogActive()) {
        mc_.cont_jog_active_ = false;
        t.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        if (t.state == ControllerState::Jogging) {
            t.state = mc_.restStateLocked();
        }
    }
}

bool FirmwareBackend::boardBusy() const { return mc_.fw_->busyWithMotion() || !mc_.fw_->ready(); }
bool FirmwareBackend::decelerating() const { return false; }
bool FirmwareBackend::motionQueued() const { return false; }
bool FirmwareBackend::motionPending() const { return false; }
bool FirmwareBackend::appSoftLimits() const { return false; } // The board enforces its own soft limits ($20)
bool FirmwareBackend::thcAvailable() const { return mc_.fw_->boardRunsThc(); }
bool FirmwareBackend::boardRunsThc() const { return mc_.fw_->boardRunsThc(); }
bool FirmwareBackend::boardRampsSpindle() const { return true; } // G4 after M3 on the board
bool FirmwareBackend::positionKnownAfterStop() const { return false; }

ResumePoint FirmwareBackend::pausedResumePoint() const {
    // After M0 / M1 the board has finished the program up to the stop: continue after it
    return ResumePoint{fw_program_stop_, fw_program_stop_, fw_resume_segment_};
}

bool FirmwareBackend::jobSpindleFromBoard(bool& on, bool& ccw) const {
    if (!mc_.fw_->jobActive()) return false;
    // The board switches its own spindle in the program (and keeps it on in a feed hold); an app-driven VFD follows
    // the sync points
    on  = mc_.config_.vfd.enabled ? fw_job_spindle_on_ : mc_.fw_->feedback().spindle_on;
    ccw = mc_.config_.vfd.enabled ? fw_job_spindle_ccw_ : mc_.fw_->feedback().spindle_ccw;
    return true;
}

bool FirmwareBackend::queuedJobSpindle(uint8_t&, bool&) const { return false; }

bool FirmwareBackend::startManualMoves(const std::vector<ToolpathSegment>& moves, ControllerState state, ManualMovePlan) {
    if (moves.empty()) return true;
    return fwStartManualMovesLocked(moves, state);
}

void FirmwareBackend::startFraming(const std::vector<std::array<double, 4>>& corners, double speed_mm_min, uint8_t out_mask,
                                   uint16_t beam_duty) {
    std::vector<ToolpathSegment> moves;
    std::array<double, 4> from = mc_.telemetry_.work_pos_mm;
    for (const auto& p : corners) {
        ToolpathSegment seg{};
        seg.type = SegmentType::CutLinear;
        seg.start_xyza_mm = from;
        seg.end_xyza_mm = p;
        seg.feed_mm_min = speed_mm_min;
        moves.push_back(seg);
        from = p;
    }
    motion::OutputRequest beam;
    if ((out_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0 && beam_duty > 0) {
        beam.spindle_on = true;
        beam.laser = true;
        beam.laser_dynamic = false;
        beam.s_value = beam_duty / 1000.0 * mc_.fw_->firmware().max_spindle_rpm;
    }
    fwStartManualMovesLocked(moves, ControllerState::Jogging, beam.spindle_on ? &beam : nullptr);
}

void FirmwareBackend::jogContinuous(size_t idx, bool positive_dir, double speed_ratio) {
    auto& fw = *mc_.fw_;
    if (!fw.ready()) return;
    if (!mc_.cont_jog_active_) {
        if (fw.busyWithMotion()) return;
        mc_.cont_jog_held_ = {false, false, false, false};
        mc_.cont_jog_target_vel_mm_s_ = {0.0, 0.0, 0.0, 0.0};
    }
    const double max_vel = fwAxisMaxVelocityLocked(idx);
    mc_.cont_jog_held_[idx]            = true;
    mc_.cont_jog_refresh_[idx]         = Clock::now();
    mc_.cont_jog_active_               = true;
    mc_.cont_jog_target_vel_mm_s_[idx] = std::clamp(speed_ratio, 0.05, 1.0) * max_vel * (positive_dir ? 1.0 : -1.0);
    mc_.telemetry_.state               = ControllerState::Jogging;
}

void FirmwareBackend::jogStep(size_t idx, double delta_mm, double feed_mm_min) {
    const double feed = std::clamp(feed_mm_min, 1.0, fwAxisMaxVelocityLocked(idx) * 60.0);
    std::string err;
    if (!mc_.fw_->jogStep(idx, delta_mm, feed, &err)) {
        mc_.telemetry_.active_fault_message = "Jog rejected: " + err;
        return;
    }
    mc_.telemetry_.state = ControllerState::Jogging;
}

void FirmwareBackend::stopManualMove() {
    if (mc_.fw_->busy()) mc_.fw_->stop(); // Jog cancel / controlled stop of the move
}

bool FirmwareBackend::stopScriptMotion() {
    const auto s = mc_.telemetry_.state;
    if (s == ControllerState::Jogging || s == ControllerState::Probing || s == ControllerState::Homing) {
        mc_.fw_->stop();
        return true;
    }
    return false;
}

void FirmwareBackend::stopForEvent() {
    // Probe contacts and home switches stop the board itself. A cancelled surface probe must stop the move the board
    // is running (its MoveDone / ProbeDone then arrive in a state that ignores them).
    if (mc_.fw_->busy() && !mc_.fw_->stopping()) mc_.fw_->stop();
}

void FirmwareBackend::clearQueuedMotion() {}

void FirmwareBackend::startHoming(int axis) {
    // The board runs its own homing cycle with its own settings ($22..$27)
    const uint8_t mask = axis < 0 ? 0 : static_cast<uint8_t>(1U << axis);
    std::string err;
    if (!mc_.fw_->startHoming(mask, &err)) {
        mc_.telemetry_.active_fault_message = "Homing rejected: " + err;
        return;
    }
    fw_homing_mask_ = mask;
    mc_.telemetry_.state = ControllerState::Homing;
}

bool FirmwareBackend::startProbeMove(size_t, const std::array<double, 4>&, const std::array<double, 4>& end_work,
                                     double feed_mm_min) {
    motion::ProbeRequest req;
    req.target_mm = mc_.workToMachineLocked(end_work);
    req.feed_mm_min = std::max(5.0, feed_mm_min);
    std::string err;
    if (!mc_.fw_->startProbe(req, &err)) {
        mc_.telemetry_.active_fault_message = "Probing rejected: " + err;
        return false;
    }
    return true;
}

bool FirmwareBackend::startJob(size_t start_seg, bool resume_in_place) {
    fw_program_stop_ = false;
    if (!fwStartJobLocked(start_seg, resume_in_place)) return false;
    mc_.telemetry_.state = ControllerState::RunningGCode;
    return true;
}

bool FirmwareBackend::resumeHeldJob() {
    if (!mc_.fw_->jobActive()) return false;
    // Feed hold on the board: cycle start continues the buffered motion
    mc_.fw_->resume();
    mc_.telemetry_.state = ControllerState::RunningGCode;
    mc_.telemetry_.active_fault_message.clear();
    fw_state_grace_until_ = Clock::now() + std::chrono::milliseconds(400);
    return true;
}

void FirmwareBackend::pauseJob() {
    // Feed hold on the board: it decelerates on the path; spindle and coolant stay on
    mc_.telemetry_.state = ControllerState::PausedHold;
    mc_.telemetry_.current_feed_mm_min = 0.0;
    mc_.fw_->hold();
    fw_seen_held_ = false;
    fw_state_grace_until_ = Clock::now() + std::chrono::milliseconds(400);
}

bool FirmwareBackend::controlledStop() {
    auto& fw = *mc_.fw_;
    if (fw.busy() && !fw.stopping()) {
        // Controlled stop on the board: hold, wait, reset (position kept); JobFinished / MoveDone follow
        fw.stop();
        if (fw.busy()) {
            mc_.telemetry_.active_fault_message = "Stopping...";
            return true;
        }
    }
    return false;
}

void FirmwareBackend::stopMotion(bool is_estop) {
    if (is_estop && !suppress_fw_estop_) {
        mc_.fw_->emergencyStop();
    } else if (!is_estop && mc_.fw_->busy()) {
        mc_.fw_->stop();
    }
    fw_program_stop_ = false;
    fw_sync_wait_ = false;
    fw_outputs_applied_ = 0xFF;
}

void FirmwareBackend::setFeedOverride(float pct) {
    mc_.fw_->setFeedOverride(pct);
}

void FirmwareBackend::setPierceDelayOverride(float) {
    // Pierce delays are planned into the job when it starts
}

void FirmwareBackend::setOutputs(uint8_t logical_mask, bool) {
    fwApplyOutputsLocked(logical_mask);
}

void FirmwareBackend::setOutputsAfterMotion(uint8_t logical_mask, bool) {
    fwApplyOutputsLocked(logical_mask);
}

void FirmwareBackend::spindleSpeedChanged() {
    fwSpindleSpeedChangedLocked();
}

void FirmwareBackend::testFireLaserPulse(float duration_ms, float power_pct) {
    // Constant power (M3) for the pulse time, at standstill
    const auto& cfg = mc_.config_;
    const double secs = std::clamp(static_cast<double>(duration_ms) / 1000.0, 0.01, 2.0);
    const float pct = std::clamp(power_pct, 1.0f, 100.0f);
    const double duty = cfg.laser.min_duty_pct + (pct / 100.0) * (cfg.laser.max_duty_pct - cfg.laser.min_duty_pct);
    const double s_val = std::clamp(duty, 0.0, 100.0) / 100.0 * mc_.fw_->firmware().max_spindle_rpm;
    std::string err;
    if (!mc_.fw_->runLines({"M3S" + motion::GcodeEmitter::num(s_val, 0), "G4P" + motion::GcodeEmitter::num(secs, 3), "M5"}, &err)) {
        mc_.telemetry_.active_fault_message = "Laser test pulse rejected: " + err;
    }
}

} // namespace rnr::core
