// MachineController: Inputs and interlocks, the worker thread (motion tick) (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

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
    if (fwActiveLocked()) {
        // Firmware board: inputs are the board's fixed terminals (Pn:); limits and the door are handled by the board
        const auto& fb = fw_->feedback();
        telemetry_.inputs_bitmask       = 0;
        telemetry_.board_inputs_mask    = fb.inputs_mask;
        telemetry_.probe_contact_active = fb.probe_active;
        telemetry_.thc_on_board         = fw_->boardRunsThc();
        telemetry_.thc_arc_ok_active    = fb.arc_ok;
        telemetry_.thc_up_active        = fb.thc_up;
        telemetry_.thc_down_active      = fb.thc_down;
        telemetry_.thc_anti_dive_locked = fb.thc_velocity_lock || fb.thc_void_lock;
        telemetry_.thc_board_active     = fb.thc_active;
        telemetry_.thc_torch_on         = fb.torch_on;
        telemetry_.thc_arc_voltage      = fb.arc_voltage;
        if (!telemetry_.thc_on_board) {
            telemetry_.thc_board_mode.clear();
        } else if (fw_thc_mode_rev_ != fw_->capabilitiesRevision() || telemetry_.thc_board_mode.empty()) {
            fw_thc_mode_rev_ = fw_->capabilitiesRevision();
            telemetry_.thc_board_mode = fw_->capabilities().board_thc_mode;
        }
        if (fb.arc_failed_count != fw_arc_failed_seen_) {
            fw_arc_failed_seen_ = fb.arc_failed_count;
            telemetry_.active_fault_message = "The torch did not start (no Arc OK): the board paused the job. "
                                              "Check the torch and press Cycle Start, or Stop.";
        }
        telemetry_.limit_switch_triggered = fb.limit_active && telemetry_.state == ControllerState::EStop;
        if (config_.operation_mode == OperationMode::Laser && !config_.is_mill) {
            telemetry_.laser_door_closed     = !fb.door_open;
            telemetry_.laser_chiller_flow_ok = true;
            telemetry_.laser_fault_active    = false;
        }
        telemetry_.estop_input_active = fb.estop_active;
        if (fb.estop_active && telemetry_.state != ControllerState::EStop &&
            telemetry_.state != ControllerState::Disconnected) {
            faultLocked("E-STOP INPUT ON THE BOARD TRIGGERED");
        }
        return;
    }
    telemetry_.inputs_bitmask = hw_inputs_bitmask_ |
        (telemetry_.simulator_mode ? telemetry_.simulated_inputs_bitmask : static_cast<uint16_t>(0));

    // Plasma THC inputs & touch probe
    telemetry_.thc_arc_ok_active    = evaluatePinStateLocked(config_.thc.arc_ok_pin, config_.thc.arc_ok_invert);
    telemetry_.thc_up_active        = evaluatePinStateLocked(config_.thc.thc_up_pin, config_.thc.thc_up_invert);
    telemetry_.thc_down_active      = evaluatePinStateLocked(config_.thc.thc_down_pin, config_.thc.thc_down_invert);
    telemetry_.probe_contact_active = evaluatePinStateLocked(config_.probe.probe_pin, config_.probe.probe_invert);

    if (config_.operation_mode == OperationMode::Laser && !config_.is_mill) {
        telemetry_.laser_chiller_flow_ok = evaluatePinStateLocked(config_.laser.chiller_flow_pin, config_.laser.chiller_flow_invert);
        telemetry_.laser_door_closed     = evaluatePinStateLocked(config_.laser.door_interlock_pin, config_.laser.door_interlock_invert);
        telemetry_.laser_fault_active    = evaluatePinStateLocked(config_.laser.laser_fault_pin, config_.laser.laser_fault_invert);
    }

    // E-Stop: input pin or board hardware E-Stop. NEVER bypassed by Limit Override.
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

    // End-stop switches (Limit Override lets the operator jog off a switch)
    std::string hit_desc;
    const bool any_limit_hit = anyLimitActiveLocked(&hit_desc);
    if (!config_.limit_override && any_limit_hit && telemetry_.state != ControllerState::EStop) {
        faultLocked(hit_desc);
        telemetry_.limit_switch_triggered = true;
        return;
    }

    // Soft limits (backstop: moves are checked before they start)
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

    // Laser safety interlocks (never bypassed by Limit Override)
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
            if (outputsLiveLocked()) {
                setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
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
    // Firmware board running a job: the display follows the board (or the app's VFD state at the sync points)
    const bool fw_job = (gcode_running || telemetry_.state == ControllerState::PausedHold) && fwActiveLocked() &&
                        fw_->jobActive();
    if (telemetry_.state != ControllerState::EStop && telemetry_.state != ControllerState::Disconnected &&
        !telemetry_.dry_run_mode) {
        if (fw_job) {
            // The board switches its own spindle in the program (and keeps it on in a feed hold); an app-driven VFD
            // follows the sync points
            const bool on = config_.vfd.enabled ? fw_job_spindle_on_ : fw_->feedback().spindle_on;
            const bool ccw = config_.vfd.enabled ? fw_job_spindle_ccw_ : fw_->feedback().spindle_ccw;
            want_spindle_on = on;
            source_mask = (on ? core::OutputBit::OUT1_TorchOrSpindle : 0) | (ccw ? core::OutputBit::OUT2_SpindleDir : 0);
        } else if (gcode_running && !slice_queue_.empty()) {
            const auto& front = slice_queue_.front();
            source_mask = front.slice.outputs_mask;
            want_spindle_on = ((front.slice.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0) &&
                              !front.is_tool_change;
        } else {
            want_spindle_on = manual_spindle_requested_;
        }
    }

    // A board-driven spindle on a firmware board: the board waits for spin-up itself (G4 after M3), no app ramp
    const bool board_spindle = fwActiveLocked() && !config_.vfd.enabled;
    if (want_spindle_on) {
        telemetry_.torch_or_spindle_on = true;
        telemetry_.outputs_bitmask |= core::OutputBit::OUT1_TorchOrSpindle;
        telemetry_.spindle_ramping_down = false;
        if (config_.spindle_ramp_up_s <= 0.01 || board_spindle) {
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
        if (config_.spindle_ramp_down_s <= 0.01 || telemetry_.state == ControllerState::EStop || board_spindle) {
            spindle_ramp_pct_ = 0.0;
            telemetry_.spindle_ramping_down = false;
            if ((!gcode_running || fw_job) && !pulse_active_ && telemetry_.state != ControllerState::Jogging) {
                if ((telemetry_.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0 && outputsLiveLocked()) {
                    telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
                    setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
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
                if (outputsLiveLocked()) {
                    setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
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
                setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
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
            setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
        }
        telemetry_.pending_tool_number      = (next_slice.tool_number > 0) ? next_slice.tool_number : 1;
        telemetry_.pending_tool_description = toolDescriptionLocked(telemetry_.pending_tool_number);
        telemetry_.active_segment_index     = next_slice.segment_index + 1;
        telemetry_.active_gcode_line_index  = next_slice.gcode_line;
        telemetry_.segment_progress_0_to_1  = 0.0f;
        last_sent_program_segment_valid_    = false;
        slice_queue_.clear();

        telemetry_.state = ControllerState::ToolChangeHold;
        if (!runM6ScriptLocked()) {
            // Automatic Z retract to safe_z_mm (and optional XY park position), spindle strictly OFF
            const auto moves = toolChangeMovesLocked();
            if (!moves.empty()) {
                slice_queue_ = planManualMovesLocked(moves);
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
        setProgramSpindleRpmLocked(next_slice.spindle_rpm, false, false);
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
            // S0 is beam off (overscan, white pixels); no S word yet is full power
            const double base_power_pct = laserPowerPctForS(config_.laser, next_slice.spindle_s_set, next_slice.spindle_rpm);
            const double effective_power_pct = std::clamp(base_power_pct * ratio * (laser_power_override_pct_ / 100.0), 0.0, 100.0);
            telemetry_.live_laser_power_pct = static_cast<float>(effective_power_pct);

            const double duty_pct = laserDutyPctForPower(config_.laser, effective_power_pct);
            const uint16_t duty = static_cast<uint16_t>(std::clamp(std::lround(duty_pct * 10.0), 0L, 1000L));
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
    // A feed-hold deceleration of a job counts as program motion
    const bool gcode_stream = is_job || (next_slice.is_hold_decel && telemetry_.state == ControllerState::PausedHold);
    // Program slices are sent as internal motion slices, like MDI moves and jogs (runMotionTickLocked prepares
    // the run with prepareProgramRunLocked). The board holds the G-code slice stream (Opcode 0x04) back until
    // several seconds of it are queued: the motors ran 3-4 s behind the app, and a feed hold or override took
    // that long to act. Internal slices run as they arrive, within max_slices_in_flight_ (75 ms) of the app.
    const auto stream = board::SliceStream::Internal;
    if (!sendBoardSliceLocked(stream, next_slice.slice, config_.limit_override, &assigned_seq)) {
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
        // Let the last decel slices run on the board before the stop cancels its buffer
        if (stop_after_hold_ && hw && !in_flight_seqs_.empty()) return;
        if (stop_after_hold_ && hw && Clock::now() < board_busy_until_ + 2 * kTickInterval) return;
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

    // In real hardware mode, wait until the board has acknowledged every slice, and until it has had the time to run
    // them: it acknowledges a slice when it buffers it. A move started before the board has finished the previous one
    // is ignored by the board (seen on the RNR Eco Motion 2.0: an MDI move right after another one was lost), and Idle
    // must mean the machine has stopped.
    if (hw && !in_flight_seqs_.empty()) return;
    if (hw && Clock::now() < board_busy_until_ + 2 * kTickInterval) return;

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
        if (program_->ends_with_program_end) {
            // M2 / M30: coolant and spindle direction off as well (the spindle is switched off above)
            telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT2_SpindleDir | core::OutputBit::OUT3_FloodCoolant |
                                            core::OutputBit::OUT4_MistCoolant);
        }
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
        setBoardOutputsAfterMotionLocked(telemetry_.outputs_bitmask, config_.limit_override);
    }
    program_run_open_     = false; // The End of Motion above closes a program run as well
    program_outputs_sent_ = 0xFFFF;
    telemetry_.state = restStateLocked();
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
        const bool program_slice = job_now || (front.is_hold_decel && telemetry_.state == ControllerState::PausedHold);

        // Plasma: wait for Arc OK before moving once the torch is lit
        if (job_now && !config_.is_mill && config_.operation_mode == OperationMode::Plasma &&
            config_.thc.enabled && config_.thc.wait_for_arc_ok &&
            telemetry_.torch_or_spindle_on && front_requires_spindle && !telemetry_.thc_arc_ok_active) {
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            if (hw) prepareProgramRunLocked(front.slice.outputs_mask, false); // Light the torch
            break;
        }
        // Mill: do not cut until the spindle is at speed (or has spun down before M5/M6)
        if (job_now && front_requires_spindle && !front.is_hold_decel &&
            (telemetry_.spindle_ramping_up || telemetry_.spindle_ramping_down)) {
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            telemetry_.active_segment_index    = front.segment_index;
            telemetry_.active_gcode_line_index = front.gcode_line;
            if (hw) prepareProgramRunLocked(front.slice.outputs_mask, false); // Spindle on while it spins up
            break;
        }
        if (hw && program_slice && !prepareProgramRunLocked(front.slice.outputs_mask, true)) {
            break; // Waiting for the board to come to rest before switching outputs or starting a run
        }
        if (hw && boardBufferFullLocked()) break; // Real time: the board has enough motion queued
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

        // Read the newest board status WITHOUT holding the controller lock (board reads can block for a few ms)
        board::BoardStatus rep{};
        const bool got_any = board_->isOpen() && board_->readStatus(rep, 2);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Read under the lock: connect() may have opened the board since the status read above, and an open state
            // read before it would report the fresh connection as lost
            const bool usb_open_now = board_->isOpen();
            const auto lock_start = Clock::now();
            struct HoldTimer {
                Clock::time_point start;
                ~HoldTimer() { diag::workerTickLockHold().add(Clock::now() - start); }
            } hold_timer{lock_start};

            if (fw_ && !telemetry_.simulator_mode) {
                fwTickLocked();
            } else {
                processBoardReportLocked(got_any, rep, usb_open_now);
            }
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
            if (fwActiveLocked()) {
                if (gcode_running) telemetry_.job_elapsed_seconds += kDtSec;
            } else {
                runMotionTickLocked(gcode_running);
            }

            // Live 3D feed rate & commanded feed rate for the DRO feed box
            const bool is_motion_state = motionActiveLocked();
            if (fwActiveLocked()) {
                telemetry_.current_feed_mm_min = fw_->feedback().feed_mm_min;
            } else if (!is_motion_state) {
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

        // A deferred configuration change may switch the VFD driver or require re-parsing the loaded program
        // (outside the lock). The re-parse never blocks the tick: if one is already running, it is retried next tick.
        if (vfd_sync_needed_.load()) syncVfd();
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

} // namespace rnr::core
