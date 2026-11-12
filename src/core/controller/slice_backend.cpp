// SliceBackend: motion on host-planned boards (RNR Eco Motion 2.0) and in the simulator. See slice_backend.h.
#include "core/controller/slice_backend.h"
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

SliceBackend::SliceBackend(MachineController& mc) : mc_(mc) {}

void SliceBackend::processBoardReportLocked(bool got_report, const board::BoardStatus& rep, bool usb_open_now) {
    if (mc_.telemetry_.simulator_mode) return;

    if (!usb_open_now) {
        mc_.hw_inputs_bitmask_ = 0;
        if (mc_.telemetry_.usb_connected) {
            mc_.telemetry_.usb_connected = false;
            mc_.faultLocked("USB CONNECTION TO MOTION BOARD LOST", ControllerState::Disconnected);
        }
        return;
    }

    const auto now = Clock::now();
    if (got_report) {
        board_report_seen_   = true;
        last_report_time_    = now;
        mc_.telemetry_.last_ack_seq = rep.ack_seq;
        mc_.telemetry_.tick_counter = rep.tick_counter;
        mc_.hw_inputs_bitmask_   = rep.inputs_bitmask;
        hw_estop_fault_      = rep.estop_fault;
        hw_limit_fault_      = rep.limit_fault;
        hw_motion_active_    = rep.motion_active;
        hw_live_steps_       = rep.live_steps;
        hw_latched_steps_    = rep.latched_steps;

        auto it = in_flight_seqs_.find(rep.ack_seq);
        if (it != in_flight_seqs_.end() && it->second.is_gcode_stream) {
            mc_.telemetry_.active_segment_index    = it->second.segment_index;
            mc_.telemetry_.active_gcode_line_index = it->second.gcode_line;
            mc_.telemetry_.segment_progress_0_to_1 = it->second.segment_progress;
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
                                !mc_.cont_jog_active_ && !hold_decel_active_ && !settle_active_ && !mc_.motionActiveLocked();
        if (board_idle) {
            double max_diff_mm = 0.0;
            bool step_mismatch = false;
            for (size_t a = 0; a < 4; ++a) {
                const auto spm = mc_.config_.axes[a].steps_per_mm;
                const int32_t cmd = static_cast<int32_t>(std::llround(mc_.telemetry_.machine_pos_mm[a] * spm));
                if (std::abs(static_cast<int64_t>(rep.live_steps[a]) - cmd) > 1) {
                    step_mismatch = true;
                    max_diff_mm = std::max(max_diff_mm, std::abs(rep.live_steps[a] - cmd) / spm);
                }
            }
            if (!feedback_validated_) {
                if (!step_mismatch) feedback_validated_ = true;
            } else if (step_mismatch) {
                std::array<double, 4> m{};
                for (size_t a = 0; a < 4; ++a) m[a] = static_cast<double>(rep.live_steps[a]) / mc_.config_.axes[a].steps_per_mm;
                mc_.telemetry_.machine_pos_mm = m;
                for (size_t a = 0; a < 4; ++a) mc_.telemetry_.work_pos_mm[a] = m[a] - mc_.telemetry_.work_offset_mm[a];
                last_sent_target_valid_ = false;
                if (max_diff_mm > 0.5 && mc_.telemetry_.active_fault_message.empty()) {
                    char buf[128];
                    std::snprintf(buf, sizeof(buf), "Position corrected from motion board feedback (%.2f mm)", max_diff_mm);
                    mc_.telemetry_.active_fault_message = buf;
                }
            }
        }
    } else {
        const bool timed_out = board_report_seen_ ? (now - last_report_time_ > kReportTimeout)
                                                  : (now - connect_time_ > kFirstReportTimeout);
        if (timed_out && mc_.telemetry_.state != ControllerState::EStop) {
            mc_.faultLocked("MOTION BOARD STOPPED RESPONDING (no status reports)");
            return;
        }
    }

    for (const auto& kv : in_flight_seqs_) {
        if (now - kv.second.sent_at > kAckTimeout) {
            mc_.faultLocked("MOTION BOARD STOPPED ACKNOWLEDGING MOTION DATA");
            return;
        }
    }
}

bool SliceBackend::sendQueuedSliceLocked() {
    planner::PlannedSlice next_slice = std::move(slice_queue_.front());
    slice_queue_.pop_front();
    const bool hw = mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode;
    const bool is_job = (mc_.telemetry_.state == ControllerState::RunningGCode || mc_.telemetry_.state == ControllerState::DwellPierce);

    // Manual Tool Change (M6): spindle off, retract, wait for the operator
    if (next_slice.is_tool_change) {
        job_planner_.reset();
        mc_.manual_spindle_requested_      = false;
        mc_.telemetry_.torch_or_spindle_on = false;
        mc_.telemetry_.outputs_bitmask    &= ~(core::OutputBit::OUT1_TorchOrSpindle | core::OutputBit::OUT2_SpindleDir);
        mc_.telemetry_.axis_vel_mm_min     = {0.0, 0.0, 0.0, 0.0};
        if (hw) {
            mc_.setBoardOutputsLocked(mc_.telemetry_.outputs_bitmask, mc_.config_.limit_override);
        }
        mc_.telemetry_.pending_tool_number      = (next_slice.tool_number > 0) ? next_slice.tool_number : 1;
        mc_.telemetry_.pending_tool_description = mc_.toolDescriptionLocked(mc_.telemetry_.pending_tool_number);
        mc_.telemetry_.active_segment_index     = next_slice.segment_index + 1;
        mc_.telemetry_.active_gcode_line_index  = next_slice.gcode_line;
        mc_.telemetry_.segment_progress_0_to_1  = 0.0f;
        last_sent_program_segment_valid_    = false;
        slice_queue_.clear();

        mc_.telemetry_.state = ControllerState::ToolChangeHold;
        if (!mc_.runM6ScriptLocked()) {
            // Automatic Z retract to safe_z_mm (and optional XY park position), spindle strictly OFF
            const auto moves = mc_.toolChangeMovesLocked();
            if (!moves.empty()) {
                slice_queue_ = planManualMovesLocked(moves);
                if (!slice_queue_.empty()) mc_.telemetry_.state = ControllerState::Jogging;
            }
        }
        return false;
    }

    // Program stop (M0 / M1): planner already brought the motion to rest before this marker
    if (next_slice.is_program_stop) {
        mc_.telemetry_.state = ControllerState::PausedHold;
        mc_.manual_spindle_requested_ = false;
        hold_needs_replan_ = false;
        mc_.telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        mc_.telemetry_.active_segment_index    = next_slice.segment_index;
        mc_.telemetry_.active_gcode_line_index = next_slice.gcode_line;
        mc_.telemetry_.active_fault_message = "Program stop (M0/M1) - press Cycle Start to continue";
        return false;
    }

    if (next_slice.tool_number > 0 && is_job) {
        mc_.telemetry_.current_tool_number = next_slice.tool_number;
    }
    if (next_slice.spindle_rpm > 0 && next_slice.spindle_rpm != mc_.telemetry_.program_spindle_rpm && is_job) {
        mc_.setProgramSpindleRpmLocked(next_slice.spindle_rpm, false, false);
    }

    if (is_job) {
        mc_.telemetry_.state = next_slice.is_dwell ? ControllerState::DwellPierce : ControllerState::RunningGCode;
        if (!hw) {
            mc_.telemetry_.active_segment_index    = next_slice.segment_index;
            mc_.telemetry_.active_gcode_line_index = next_slice.gcode_line;
            mc_.telemetry_.segment_progress_0_to_1 = next_slice.segment_progress;
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

    const bool laser_mode = (mc_.config_.operation_mode == OperationMode::Laser && !mc_.config_.is_mill);
    if (laser_mode && is_job) {
        // CNC Laser Mode: M4 velocity-proportional power scaling & auto air assist (job slices only)
        const bool laser_active = (next_slice.slice.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0;
        if (laser_active && !mc_.telemetry_.dry_run_mode) {
            double target_feed = 1000.0;
            if (next_slice.from_program && next_slice.segment_index < mc_.program_->segments.size()) {
                target_feed = mc_.program_->segments[next_slice.segment_index].feed_mm_min * (mc_.telemetry_.feed_override_pct / 100.0);
            }
            double ratio = 1.0;
            if (mc_.config_.laser.dynamic_power_m4 && target_feed > 1.0) {
                ratio = std::clamp(xy_speed_mm_min / target_feed,
                                   static_cast<double>(mc_.config_.laser.corner_power_floor_pct) / 100.0, 1.0);
            }
            // S0 is beam off (overscan, white pixels); no S word yet is full power
            const double base_power_pct = laserPowerPctForS(mc_.config_.laser, next_slice.spindle_s_set, next_slice.spindle_rpm);
            const double effective_power_pct = std::clamp(base_power_pct * ratio * (mc_.laser_power_override_pct_ / 100.0), 0.0, 100.0);
            mc_.telemetry_.live_laser_power_pct = static_cast<float>(effective_power_pct);

            const double duty_pct = laserDutyPctForPower(mc_.config_.laser, effective_power_pct);
            const uint16_t duty = static_cast<uint16_t>(std::clamp(std::lround(duty_pct * 10.0), 0L, 1000L));
            if (std::abs(static_cast<int>(duty) - static_cast<int>(mc_.last_sent_laser_pwm_duty_)) >= 15 ||
                (duty == 0 && mc_.last_sent_laser_pwm_duty_ != 0)) {
                mc_.last_sent_laser_pwm_duty_ = duty;
                mc_.telemetry_.spindle_pwm_duty = duty;
                if (hw) {
                    mc_.board_->setSpindlePwm(duty, mc_.config_.limit_override);
                }
            }
            if (mc_.config_.laser.auto_air_assist) {
                next_slice.slice.outputs_mask |= core::OutputBit::OUT3_FloodCoolant;
                mc_.air_postflow_timer_s_ = mc_.config_.laser.air_postflow_s;
            }
        } else {
            mc_.telemetry_.live_laser_power_pct = 0.0f;
            if (mc_.last_sent_laser_pwm_duty_ != 0) {
                mc_.last_sent_laser_pwm_duty_ = 0;
                mc_.telemetry_.spindle_pwm_duty = 0;
                if (hw) {
                    mc_.board_->setSpindlePwm(0, mc_.config_.limit_override);
                }
            }
            if (mc_.config_.laser.auto_air_assist) {
                if (mc_.air_postflow_timer_s_ > 0.0) {
                    mc_.air_postflow_timer_s_ -= kDtSec;
                    next_slice.slice.outputs_mask |= core::OutputBit::OUT3_FloodCoolant;
                } else {
                    next_slice.slice.outputs_mask &= ~core::OutputBit::OUT3_FloodCoolant;
                }
            }
        }
        mc_.telemetry_.laser_air_assist_active  = (next_slice.slice.outputs_mask & core::OutputBit::OUT3_FloodCoolant) != 0;
        mc_.telemetry_.laser_red_pointer_active = (next_slice.slice.outputs_mask & core::OutputBit::OUT2_SpindleDir) != 0;
    } else if (is_job && !mc_.config_.is_mill && mc_.config_.thc.enabled &&
               mc_.telemetry_.state == ControllerState::RunningGCode &&
               (next_slice.slice.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0 &&
               mc_.torch_on_duration_s_ >= mc_.config_.thc.delay_after_pierce_s) {
        // Plasma THC: anti-dive lock below a fraction of the programmed feed, else follow UP/DOWN inputs
        double target_feed = 1500.0;
        if (next_slice.from_program && next_slice.segment_index < mc_.program_->segments.size()) {
            target_feed = mc_.program_->segments[next_slice.segment_index].feed_mm_min * (mc_.telemetry_.feed_override_pct / 100.0);
        }
        const double min_allowed_xy = target_feed * (mc_.config_.thc.min_xy_feed_pct / 100.0);
        mc_.telemetry_.thc_anti_dive_locked = (xy_speed_mm_min < min_allowed_xy);
        if (!mc_.telemetry_.thc_anti_dive_locked) {
            double dz_thc = 0.0;
            if (mc_.telemetry_.thc_up_active && !mc_.telemetry_.thc_down_active) {
                dz_thc = mc_.config_.thc.adjust_speed_mm_s * kDtSec;
            } else if (mc_.telemetry_.thc_down_active && !mc_.telemetry_.thc_up_active) {
                dz_thc = -mc_.config_.thc.adjust_speed_mm_s * kDtSec;
            }
            if (std::abs(dz_thc) > 1e-6) {
                const double new_offset = std::clamp(mc_.telemetry_.thc_z_offset_mm + dz_thc, mc_.config_.thc.z_min_mm, mc_.config_.thc.z_max_mm);
                const double actual_dz = new_offset - mc_.telemetry_.thc_z_offset_mm;
                const int32_t dsteps = static_cast<int32_t>(std::llround(actual_dz * mc_.config_.axes[2].steps_per_mm));
                mc_.telemetry_.thc_z_offset_mm += static_cast<double>(dsteps) / mc_.config_.axes[2].steps_per_mm;
                next_slice.slice.delta_steps[2] += dsteps;
            }
        }
    }

    mc_.telemetry_.outputs_bitmask     = next_slice.slice.outputs_mask;
    mc_.telemetry_.torch_or_spindle_on = (mc_.telemetry_.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0;

    // Synchronize the displayed workspace if the program switched G54..G59 (motion is already mapped)
    bool frame_changed = false;
    if (is_job && next_slice.workspace_index != mc_.telemetry_.active_workspace && next_slice.workspace_index < 6) {
        mc_.telemetry_.active_workspace = next_slice.workspace_index;
        mc_.config_.active_workspace    = next_slice.workspace_index;
        mc_.bumpWorkspaceRevisionLocked();
        frame_changed = true;
    }
    // ...and the tool length offset the program selected with G43 H<n> / G49
    if (is_job && next_slice.from_program && mc_.config_.tool_table.mode == ToolLengthMode::ToolTable && next_slice.tlo_tool != kTloInherit &&
        (next_slice.tlo_tool != mc_.telemetry_.active_tlo_tool || std::abs(next_slice.tlo_mm - mc_.telemetry_.active_tlo_mm) > 1e-9)) {
        mc_.telemetry_.active_tlo_tool = next_slice.tlo_tool;
        mc_.telemetry_.active_tlo_mm   = next_slice.tlo_mm;
        frame_changed = true;
    }
    if (frame_changed) {
        mc_.refreshWorkFrameLocked();
    }

    // Advance the commanded position (the THC Z correction is part of the step deltas, so it is
    // already contained in the machine position - it is not added again)
    for (size_t a = 0; a < 4; ++a) {
        const double delta_mm = static_cast<double>(next_slice.slice.delta_steps[a]) / mc_.config_.axes[a].steps_per_mm;
        mc_.telemetry_.machine_pos_mm[a] += delta_mm;
        mc_.telemetry_.work_pos_mm[a]     = mc_.telemetry_.machine_pos_mm[a] - mc_.telemetry_.work_offset_mm[a];
        mc_.telemetry_.axis_vel_mm_min[a] = std::abs(delta_mm) / kDtSec * 60.0;
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
        mc_.telemetry_.tick_counter += 25;
        return true;
    }
    uint16_t assigned_seq = 0;
    // A feed-hold deceleration of a job counts as program motion
    const bool gcode_stream = is_job || (next_slice.is_hold_decel && mc_.telemetry_.state == ControllerState::PausedHold);
    // Program slices are sent as internal motion slices, like MDI moves and jogs (runMotionTickLocked prepares
    // the run with prepareProgramRunLocked). The board holds the G-code slice stream (Opcode 0x04) back until
    // several seconds of it are queued: the motors ran 3-4 s behind the app, and a feed hold or override took
    // that long to act. Internal slices run as they arrive, within max_slices_in_flight_ (75 ms) of the app.
    const auto stream = board::SliceStream::Internal;
    if (!sendBoardSliceLocked(stream, next_slice.slice, mc_.config_.limit_override, &assigned_seq)) {
        mc_.faultLocked("USB WRITE FAILED WHILE STREAMING MOTION");
        return false;
    }
    in_flight_seqs_[assigned_seq] = SeqProgressInfo{
        next_slice.segment_index, next_slice.gcode_line, next_slice.segment_progress, gcode_stream, Clock::now()};
    return true;
}

void SliceBackend::finishQueueLocked() {
    const bool hw = mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode;

    if (hold_decel_active_) {
        // Let the last decel slices run on the board before the stop cancels its buffer
        if (stop_after_hold_ && hw && !in_flight_seqs_.empty()) return;
        if (stop_after_hold_ && hw && Clock::now() < board_busy_until_ + 2 * kTickInterval) return;
        hold_decel_active_ = false;
        mc_.telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        last_sent_speed_mm_s_ = 0.0;
        if (stop_after_hold_) {
            stop_after_hold_ = false;
            mc_.stopJobLocked(false);
        } else if (resume_after_hold_) {
            resume_after_hold_ = false;
            hold_needs_replan_ = false;
            const size_t seg = last_sent_program_segment_valid_ ? last_sent_program_segment_ : mc_.telemetry_.active_segment_index;
            rebuildPlannedSlicesLocked(seg, last_sent_from_program_);
            mc_.telemetry_.state = slice_queue_.empty() ? ControllerState::Idle : ControllerState::RunningGCode;
        }
        return;
    }

    if (mc_.telemetry_.state == ControllerState::PausedHold) return;

    // In real hardware mode, wait until the board has acknowledged every slice, and until it has had the time to run
    // them: it acknowledges a slice when it buffers it. A move started before the board has finished the previous one
    // is ignored by the board (seen on the RNR Eco Motion 2.0: an MDI move right after another one was lost), and Idle
    // must mean the machine has stopped.
    if (hw && !in_flight_seqs_.empty()) return;
    if (hw && Clock::now() < board_busy_until_ + 2 * kTickInterval) return;

    mc_.telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    last_sent_speed_mm_s_ = 0.0;
    mc_.manual_spindle_requested_  = mc_.manual_spindle_requested_ && mc_.telemetry_.state == ControllerState::Jogging;
    if (mc_.config_.spindle_ramp_down_s <= 0.01 || mc_.spindle_ramp_pct_ <= 0.05) {
        if (!mc_.manual_spindle_requested_) {
            mc_.spindle_ramp_pct_               = 0.0;
            mc_.telemetry_.spindle_speed_pct    = 0.0f;
            mc_.telemetry_.spindle_ramping_down = false;
            mc_.telemetry_.torch_or_spindle_on  = false;
            mc_.telemetry_.outputs_bitmask     &= ~core::OutputBit::OUT1_TorchOrSpindle;
        }
    } else if (!mc_.manual_spindle_requested_) {
        mc_.telemetry_.spindle_ramping_down = true;
    }
    if (mc_.telemetry_.state == ControllerState::RunningGCode || mc_.telemetry_.state == ControllerState::DwellPierce) {
        if (mc_.program_->ends_with_program_end) {
            // M2 / M30: coolant and spindle direction off as well (the spindle is switched off above)
            mc_.telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT2_SpindleDir | core::OutputBit::OUT3_FloodCoolant |
                                            core::OutputBit::OUT4_MistCoolant);
        }
        mc_.telemetry_.active_segment_index    = mc_.program_->segments.size();
        mc_.telemetry_.segment_progress_0_to_1 = 1.0f;
        last_sent_program_segment_valid_   = false;
        job_planner_.reset();
    }
    if (mc_.config_.operation_mode == OperationMode::Laser && !mc_.config_.is_mill && mc_.telemetry_.spindle_pwm_duty != 0 &&
        !mc_.manual_spindle_requested_) {
        // End of framing / laser job: make sure the beam PWM is back to zero
        mc_.telemetry_.spindle_pwm_duty = 0;
        mc_.last_sent_laser_pwm_duty_ = 0;
        if (hw) mc_.board_->setSpindlePwm(0, mc_.config_.limit_override);
    }
    if (hw) {
        mc_.setBoardOutputsAfterMotionLocked(mc_.telemetry_.outputs_bitmask, mc_.config_.limit_override);
    }
    program_run_open_     = false; // The End of Motion above closes a program run as well
    program_outputs_sent_ = 0xFFFF;
    mc_.telemetry_.state = mc_.restStateLocked();
}

void SliceBackend::runMotionTickLocked(bool gcode_running) {
    const auto s = mc_.telemetry_.state;
    const bool motion_state = (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce ||
                               s == ControllerState::Jogging || s == ControllerState::Homing ||
                               s == ControllerState::Probing) || hold_decel_active_;
    if (mc_.cont_jog_active_ || !motion_state) return;

    if (s == ControllerState::Homing) {
        homingTickLocked();
        if (mc_.telemetry_.state != ControllerState::Homing || slice_queue_.empty()) return;
    } else if (s == ControllerState::Probing) {
        probingTickLocked();
        if (mc_.telemetry_.state != ControllerState::Probing || slice_queue_.empty()) return;
    }

    refillJobQueueLocked();

    const bool hw = mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode;
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
        const bool job_now = (mc_.telemetry_.state == ControllerState::RunningGCode || mc_.telemetry_.state == ControllerState::DwellPierce);
        const bool program_slice = job_now || (front.is_hold_decel && mc_.telemetry_.state == ControllerState::PausedHold);

        // Plasma: wait for Arc OK before moving once the torch is lit
        if (job_now && !mc_.config_.is_mill && mc_.config_.operation_mode == OperationMode::Plasma &&
            mc_.config_.thc.enabled && mc_.config_.thc.wait_for_arc_ok &&
            mc_.telemetry_.torch_or_spindle_on && front_requires_spindle && !mc_.telemetry_.thc_arc_ok_active) {
            mc_.telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            if (hw) prepareProgramRunLocked(front.slice.outputs_mask, false); // Light the torch
            break;
        }
        // Mill: do not cut until the spindle is at speed (or has spun down before M5/M6)
        if (job_now && front_requires_spindle && !front.is_hold_decel &&
            (mc_.telemetry_.spindle_ramping_up || mc_.telemetry_.spindle_ramping_down)) {
            mc_.telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            mc_.telemetry_.active_segment_index    = front.segment_index;
            mc_.telemetry_.active_gcode_line_index = front.gcode_line;
            if (hw) prepareProgramRunLocked(front.slice.outputs_mask, false); // Spindle on while it spins up
            break;
        }
        if (hw && program_slice && !prepareProgramRunLocked(front.slice.outputs_mask, true)) {
            break; // Waiting for the board to come to rest before switching outputs or starting a run
        }
        if (hw && boardBufferFullLocked()) break; // Real time: the board has enough motion queued
        const auto state_before = mc_.telemetry_.state;
        if (!sendQueuedSliceLocked()) break;
        sent_any = true;
        if (mc_.telemetry_.state != state_before &&
            !(state_before == ControllerState::RunningGCode && mc_.telemetry_.state == ControllerState::DwellPierce) &&
            !(state_before == ControllerState::DwellPierce && mc_.telemetry_.state == ControllerState::RunningGCode)) {
            break;
        }
        refillJobQueueLocked();
    }

    if (gcode_running || sent_any) {
        if (mc_.telemetry_.state == ControllerState::RunningGCode || mc_.telemetry_.state == ControllerState::DwellPierce) {
            mc_.telemetry_.job_elapsed_seconds += kDtSec;
        }
    }
}

bool SliceBackend::sendBoardSliceLocked(board::SliceStream stream, const core::StepSlice& slice,
                                             bool limit_override, uint16_t* out_seq) {
    core::StepSlice phys = slice;
    phys.outputs_mask = io::physicalOutputs(mc_.config_.io, slice.outputs_mask);
    if (!mc_.board_->sendSlice(stream, phys, limit_override, out_seq)) return false;
    const auto now = Clock::now();
    board_busy_until_ = std::max(now, board_busy_until_) + kTickInterval;
    return true;
}

bool SliceBackend::boardBufferFullLocked() const {
    // Keep at most max_slices_in_flight_ slices (75 ms) of motion ahead of the board. An acknowledgement only
    // means the board has buffered a slice: pacing by acknowledgements alone let the app run up to 3x real time
    // ahead until the board's buffer was full, so programs and feed holds ran 3-4 s behind the app.
    const auto lead = kTickInterval * static_cast<int>(max_slices_in_flight_);
    return board_busy_until_ > Clock::now() + lead - kTickInterval / 2;
}

bool SliceBackend::prepareProgramRunLocked(uint8_t outputs_mask, bool arm) {
    // The board is at rest: every slice acknowledged, and the board has had time to run every slice sent
    // (plus 50 ms). The motion flag is not used: it may stay set while a run is open.
    const bool at_rest = in_flight_seqs_.empty() && (Clock::now() > board_busy_until_ + 2 * kTickInterval);
    // Ran out of slices (spindle wait, pause, tool change): end that run like a finished MDI move
    if (program_run_open_ && at_rest) {
        closeProgramRunLocked();
    }
    if (program_outputs_sent_ != outputs_mask) {
        // The planner stops at every outputs change, so this only waits for the last slices to run
        if (!at_rest) return false;
        mc_.setBoardOutputsLocked(outputs_mask, mc_.config_.limit_override);
        program_outputs_sent_ = outputs_mask;
    }
    if (!arm) return false;
    if (!program_run_open_) {
        if (!at_rest) return false;
        mc_.board_->beginJog(Axis::X, mc_.config_.limit_override);
        program_run_open_ = true;
    }
    return true;
}

void SliceBackend::closeProgramRunLocked() {
    if (!program_run_open_) return;
    program_run_open_ = false;
    if (mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode) {
        const uint8_t mask = (program_outputs_sent_ <= 0xFF) ? static_cast<uint8_t>(program_outputs_sent_)
                                                            : mc_.telemetry_.outputs_bitmask;
        mc_.setBoardOutputsAfterMotionLocked(mask, mc_.config_.limit_override);
    }
}

void SliceBackend::rebuildPlannedSlicesLocked(size_t start_from_segment, bool resume_in_place) {
    slice_queue_.clear();
    job_planner_.reset();
    const auto& segs = mc_.program_->segments;
    if (segs.empty() || start_from_segment >= segs.size()) {
        return;
    }

    mc_.job_frame_ = mc_.jobFrameLocked();
    size_t program_first = start_from_segment;
    std::array<double, 4> plan_start{};
    std::vector<ToolpathSegment> prefix = mc_.buildJobPrefixLocked(start_from_segment, resume_in_place, program_first, plan_start);

    planner::PlannerOptions opts;
    opts.feed_override_pct       = mc_.telemetry_.feed_override_pct;
    opts.pierce_delay_override_s = mc_.telemetry_.pierce_delay_override_s;
    opts.dry_run_no_torch        = mc_.telemetry_.dry_run_mode;
    opts.slice_dt_sec            = kDtSec;
    job_planner_ = std::make_unique<planner::StreamingPlanner>(mc_.config_, plan_start, opts, mc_.job_frame_);
    queued_feed_override_pct_ = opts.feed_override_pct;
    job_planner_->setSource(std::move(prefix),
                            std::shared_ptr<const std::vector<ToolpathSegment>>(mc_.program_, &mc_.program_->segments),
                            program_first);
    job_planner_->plan(slice_queue_, kRefillAmount * 2);
}

void SliceBackend::refillJobQueueLocked() {
    if (!job_planner_ || job_planner_->finished() || hold_decel_active_) return;
    const auto s = mc_.telemetry_.state;
    if (s != ControllerState::RunningGCode && s != ControllerState::DwellPierce) return;
    if (slice_queue_.size() >= kRefillThreshold) return;
    job_planner_->plan(slice_queue_, kRefillAmount);
}

void SliceBackend::replanFromQueueTailLocked() {
    // Keep a short buffer of already-planned slices and re-plan everything after it with the new feed
    if (!job_planner_ || hold_decel_active_ || slice_queue_.size() <= kOverrideKeepSlices) return;
    const auto& last = slice_queue_[kOverrideKeepSlices - 1];
    const auto& prev = slice_queue_[kOverrideKeepSlices - 2];
    if (last.is_dwell || last.is_tool_change || last.is_program_stop || last.is_hold_decel || !last.from_program ||
        prev.is_dwell || prev.is_tool_change || prev.is_program_stop || !prev.from_program) {
        return;
    }
    if (last.segment_index >= mc_.program_->segments.size()) return;

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
    ToolpathSegment rem = mc_.program_->segments[last.segment_index];
    rem.start_xyza_mm = last.target_pos_mm;

    planner::PlannerOptions opts;
    opts.feed_override_pct       = mc_.telemetry_.feed_override_pct;
    opts.pierce_delay_override_s = mc_.telemetry_.pierce_delay_override_s;
    opts.dry_run_no_torch        = mc_.telemetry_.dry_run_mode;
    opts.slice_dt_sec            = kDtSec;
    auto np = std::make_unique<planner::StreamingPlanner>(mc_.config_, last.target_pos_mm, opts, mc_.job_frame_);
    np->setSource({rem}, std::shared_ptr<const std::vector<ToolpathSegment>>(mc_.program_, &mc_.program_->segments),
                  last.segment_index + 1, true);
    np->setInitialMotion(v, unit);

    slice_queue_.erase(slice_queue_.begin() + static_cast<std::ptrdiff_t>(kOverrideKeepSlices), slice_queue_.end());
    job_planner_ = std::move(np);
    queued_feed_override_pct_ = opts.feed_override_pct;
    job_planner_->plan(slice_queue_, kRefillAmount);
}

void SliceBackend::beginHoldDecelLocked(bool stop_after) {
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
    const std::array<double, 4> from = last_sent_target_valid_ ? last_sent_target_ : mc_.telemetry_.work_pos_mm;
    planner::TrajectoryPlanner::planFeedHoldDeceleration(slice_queue_, from, v0, mc_.config_, kDtSec);
    // Planned slices carry outputs; during a hold the torch / laser must not be re-enabled
    for (auto& s : slice_queue_) {
        if (mc_.telemetry_.state == ControllerState::PausedHold) {
            s.slice.outputs_mask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        }
    }
    if (slice_queue_.empty()) {
        hold_decel_active_ = false;
        if (stop_after) {
            mc_.stopJobLocked(false);
        }
    } else {
        hold_decel_active_ = true;
        stop_after_hold_   = stop_after;
    }
}

void SliceBackend::beginSettleLocked() {
    settle_active_ = true;
    settle_board_idle_seen_ = false;
    settle_deadline_ = Clock::now() + kSettleTimeout;
}

bool SliceBackend::settledLocked() const {
    return settle_board_idle_seen_ || Clock::now() >= settle_deadline_;
}

void SliceBackend::stopMotionForEventLocked() {
    slice_queue_.clear();
    job_planner_.reset();
    if (mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode) {
        mc_.board_->cancelMotion();
        in_flight_seqs_.clear();
        beginSettleLocked();
    }
    last_sent_target_valid_ = false;
    last_sent_speed_mm_s_ = 0.0;
}

bool SliceBackend::homeSwitchActiveLocked(size_t axis) const {
    const auto& ax = mc_.config_.axes[axis];
    if (mc_.telemetry_.simulator_mode) {
        // Simulated home switch located at machine zero
        return ax.homing_dir_negative ? (mc_.telemetry_.machine_pos_mm[axis] <= 0.0)
                                      : (mc_.telemetry_.machine_pos_mm[axis] >= 0.0);
    }
    return mc_.evaluatePinStateLocked(ax.home_pin, ax.home_invert);
}

void SliceBackend::startNextHomingAxisLocked() {
    auto finishHoming = [&](const std::string& message) {
        homing_axis_queue_.clear();
        homing_phase_ = HomingPhase::None;
        slice_queue_.clear();
        if (mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode) {
            mc_.board_->configure(mc_.config_, false);
        }
        mc_.telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        mc_.telemetry_.state = ControllerState::Idle;
        if (!message.empty()) mc_.telemetry_.active_fault_message = message;
    };

    if (homing_axis_queue_.empty()) {
        finishHoming("");
        return;
    }

    active_homing_axis_ = homing_axis_queue_.front();
    homing_axis_queue_.pop_front();
    const size_t idx = static_cast<size_t>(active_homing_axis_);
    const auto& ax = mc_.config_.axes[idx];

    if (!mc_.telemetry_.simulator_mode && ax.home_pin == 0) {
        finishHoming(std::string("Cannot home ") + kAxisNames[idx] + ": no home switch input is configured");
        return;
    }

    if (mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode) {
        mc_.board_->configure(mc_.config_, true);
        mc_.board_->armHomeSwitchStop(static_cast<Axis>(idx));
    }

    mc_.telemetry_.axis_homed[idx] = false;
    homing_switch_event_ = false;
    const double dir = ax.homing_dir_negative ? -1.0 : 1.0;
    std::array<double, 4> end_w = mc_.telemetry_.work_pos_mm;
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
    slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(mc_.telemetry_.work_pos_mm, end_w, feed, mc_.config_, 0);
    mc_.telemetry_.state = ControllerState::Homing;
}

void SliceBackend::homingTickLocked() {
    const size_t idx = static_cast<size_t>(active_homing_axis_);
    const auto& ax = mc_.config_.axes[idx];
    const bool sw = homeSwitchActiveLocked(idx);

    if (!homing_switch_event_) {
        if ((homing_phase_ == HomingPhase::BackOff && !sw) || (homing_phase_ == HomingPhase::Seek && sw)) {
            homing_switch_event_ = true;
            stopMotionForEventLocked();
        }
    }
    if (!slice_queue_.empty()) return;
    if (mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode) {
        if (settle_active_) {
            if (!settledLocked()) return;
            settle_active_ = false;
            if (feedback_validated_) {
                std::array<double, 4> m{};
                for (size_t a = 0; a < 4; ++a) m[a] = static_cast<double>(hw_live_steps_[a]) / mc_.config_.axes[a].steps_per_mm;
                mc_.telemetry_.machine_pos_mm = m;
                for (size_t a = 0; a < 4; ++a) mc_.telemetry_.work_pos_mm[a] = m[a] - mc_.telemetry_.work_offset_mm[a];
            }
        } else if (!in_flight_seqs_.empty()) {
            return;
        }
    }

    auto abortHoming = [&](const std::string& msg) {
        homing_axis_queue_.clear();
        startNextHomingAxisLocked(); // Restores the board pinout and returns to Idle
        mc_.telemetry_.active_fault_message = msg;
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
            std::array<double, 4> end_w = mc_.telemetry_.work_pos_mm;
            end_w[idx] += dir * ax.homing_max_travel_mm;
            slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(mc_.telemetry_.work_pos_mm, end_w, ax.homing_feed_mm_min, mc_.config_, 0);
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
                mc_.tool_ref_contact_valid_ = false;
                mc_.telemetry_.tool_reference_measured = false;
            }
            // The switch position defines machine zero for this axis
            std::array<double, 4> m = mc_.telemetry_.machine_pos_mm;
            m[idx] = 0.0;
            mc_.setMachinePositionLocked(m);
            homing_phase_ = HomingPhase::PullOff;
            if (ax.home_pulloff_mm > 1e-3) {
                std::array<double, 4> end_w = mc_.telemetry_.work_pos_mm;
                end_w[idx] -= dir * ax.home_pulloff_mm;
                slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(
                    mc_.telemetry_.work_pos_mm, end_w, std::max(60.0, ax.homing_feed_mm_min * 0.5), mc_.config_, 0);
            }
            break;
        }
        case HomingPhase::PullOff:
            mc_.telemetry_.axis_homed[idx] = true;
            startNextHomingAxisLocked();
            break;
        case HomingPhase::None:
            startNextHomingAxisLocked();
            break;
    }
}

void SliceBackend::probingTickLocked() {
    const bool hw = mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode;
    if (mc_.probe_move_is_search_ && !mc_.probe_contact_captured_ && mc_.telemetry_.probe_contact_active) {
        mc_.probe_contact_captured_ = true;
        std::array<double, 4> m = mc_.telemetry_.machine_pos_mm;
        if (hw && feedback_validated_) {
            // Prefer the hardware-latched trigger position; fall back to the live step counters
            const bool latched_new = (hw_latched_steps_ != probe_latched_at_arm_);
            for (size_t a = 0; a < 4; ++a) {
                const int32_t steps = latched_new ? hw_latched_steps_[a] : hw_live_steps_[a];
                m[a] = static_cast<double>(steps) / mc_.config_.axes[a].steps_per_mm;
            }
        }
        mc_.probe_contact_machine_ = m;
        stopMotionForEventLocked();
    }
    if (!slice_queue_.empty()) return;
    if (hw) {
        if (settle_active_) {
            if (!settledLocked()) return;
            settle_active_ = false;
            if (feedback_validated_) {
                std::array<double, 4> m{};
                for (size_t a = 0; a < 4; ++a) m[a] = static_cast<double>(hw_live_steps_[a]) / mc_.config_.axes[a].steps_per_mm;
                mc_.telemetry_.machine_pos_mm = m;
                for (size_t a = 0; a < 4; ++a) mc_.telemetry_.work_pos_mm[a] = m[a] - mc_.telemetry_.work_offset_mm[a];
            }
        } else if (!in_flight_seqs_.empty()) {
            return;
        }
    }
    if (mc_.probe_move_is_search_) {
        mc_.onProbeMoveFinishedLocked(mc_.probe_contact_captured_ || mc_.telemetry_.simulator_mode);
    } else {
        mc_.continueHoleCenterLocked();
    }
}

std::deque<planner::PlannedSlice> SliceBackend::planManualMovesLocked(const std::vector<ToolpathSegment>& moves) const {
    MachineConfig manual_cfg = mc_.config_;
    manual_cfg.is_mill = true; // Manual moves: no laser Z lock / output rules
    return planner::TrajectoryPlanner::sliceProgram(moves, manual_cfg);
}

void SliceBackend::restartFeedbackLocked() {
    feedback_validated_ = false;
    board_report_seen_  = false;
    connect_time_       = Clock::now();
}

void SliceBackend::syncBoardPositionLocked() {
    if (!mc_.board_->isOpen() || mc_.telemetry_.simulator_mode) return;
    // The machine position was redefined (connect, homing, restored position, new steps/mm). The RNR Eco Motion 2.0
    // ignores Set Axis Positions, so its counters keep their own zero: they are no longer proven to be machine
    // coordinates until they match the commanded position again. Adopting them anyway moved the machine position
    // (and every work offset) by the homing distance at the next E-stop.
    feedback_validated_ = false;
    std::array<int32_t, 4> steps{};
    for (size_t i = 0; i < 4; ++i) {
        steps[i] = static_cast<int32_t>(std::llround(mc_.telemetry_.machine_pos_mm[i] * mc_.config_.axes[i].steps_per_mm));
    }
    mc_.board_->setAxisPositions(steps);
}

void SliceBackend::positionRedefinedLocked() {
    syncBoardPositionLocked();
    last_sent_target_valid_ = false;
}

void SliceBackend::removeOutputsFromQueueLocked(uint8_t mask) {
    for (auto& q : slice_queue_) q.slice.outputs_mask &= static_cast<uint8_t>(~mask);
}

void SliceBackend::forgetJobPositionLocked(bool discard_held_job) {
    last_sent_program_segment_valid_ = false;
    if (discard_held_job) {
        slice_queue_.clear();
        job_planner_.reset();
        hold_needs_replan_ = false;
    }
}

// ============================================================================
// IMotionBackend
// ============================================================================
void SliceBackend::readBoard(bool got_report, const board::BoardStatus& rep, bool usb_open_now) {
    processBoardReportLocked(got_report, rep, usb_open_now);
}

void SliceBackend::onBoardConnected() {
    restartFeedbackLocked();
    last_report_time_     = connect_time_;
    hw_estop_fault_       = false;
    hw_limit_fault_       = false;
    in_flight_seqs_.clear();
    program_run_open_     = false;
    program_outputs_sent_ = 0xFFFF;
}

bool SliceBackend::resetFromEStop() {
    auto& t = mc_.telemetry_;
    const auto& cfg = mc_.config_;
    const bool hw = !t.simulator_mode;
    if (hw && !mc_.board_->isOpen()) {
        t.active_fault_message = "Cannot reset: motion board is not connected";
        return false;
    }
    mc_.evaluateInputsAndEndStopsLocked();
    if (mc_.estopInputActiveLocked()) {
        t.active_fault_message = "Cannot reset: E-Stop input is still active";
        return false;
    }
    if (hw && hw_estop_fault_) {
        t.active_fault_message = "Cannot reset: board reports hardware E-Stop";
        return false;
    }
    std::string limit_desc;
    if (!cfg.limit_override && mc_.anyLimitActiveLocked(&limit_desc)) {
        t.active_fault_message = "Cannot reset: " + limit_desc + " is active - enable Limit Override to jog off";
        return false;
    }
    if (cfg.operation_mode == OperationMode::Laser && !cfg.is_mill) {
        if (cfg.laser.chiller_flow_pin >= 1 && !t.laser_chiller_flow_ok) {
            t.active_fault_message = "Cannot reset: laser chiller flow interlock not satisfied";
            return false;
        }
        if (cfg.laser.laser_fault_pin >= 1 && t.laser_fault_active) {
            t.active_fault_message = "Cannot reset: laser PSU fault input is active";
            return false;
        }
    }
    if (cfg.vfd.enabled) {
        const auto vt = mc_.vfd_controller_.getTelemetry();
        if (vt.anyAlarm()) {
            t.active_fault_message = "Cannot reset: clear the VFD alarm first (" + vt.last_error_msg + ")";
            return false;
        }
    }

    if (mc_.board_->isOpen() && hw) {
        mc_.board_->clearEmergencyStop();
        // The Pre-Reset in clearEmergencyStop() puts the board back on its default timing: without a new
        // configuration it ran our 25 ms slices jerkily until any setting (such as the pulse rate) was changed.
        // The Mach3 plugin also re-sends its configuration after the clear.
        mc_.board_->configure(cfg, false);
        // The reset clears the board's step counters (measured on an RNR Eco Motion 2.0, which also ignores Set Axis
        // Positions): from now on they count from zero, not from the machine position
        feedback_validated_     = false;
        last_sent_target_valid_ = false;
    }
    program_run_open_     = false;
    program_outputs_sent_ = 0xFFFF;
    t.limit_switch_triggered = false;
    t.soft_limit_triggered   = false;
    t.active_fault_message.clear();
    t.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    t.current_feed_mm_min = 0.0;
    t.state = ControllerState::Idle;
    return true;
}

bool SliceBackend::readInputs() {
    // The board's input pins (IN1..IN12) as one mask: the controller evaluates limits, probe, THC and E-stop
    auto& t = mc_.telemetry_;
    t.inputs_bitmask = mc_.hw_inputs_bitmask_ | (t.simulator_mode ? t.simulated_inputs_bitmask : static_cast<uint16_t>(0));
    return false;
}

void SliceBackend::tick(bool gcode_running) {
    runMotionTickLocked(gcode_running);
}

void SliceBackend::updateFeedTelemetry() {
    auto& t = mc_.telemetry_;
    if (!mc_.motionActiveLocked()) {
        t.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        t.current_feed_mm_min = 0.0;
    } else {
        const double vx = t.axis_vel_mm_min[0];
        const double vy = t.axis_vel_mm_min[1];
        const double vz = t.axis_vel_mm_min[2];
        t.current_feed_mm_min = std::sqrt(vx * vx + vy * vy + vz * vz);
    }
}

void SliceBackend::updateContinuousJog() {
    // Dead-man and target velocities are the controller's; the ramps and step positions are this backend's
    auto& t = mc_.telemetry_;
    const auto& cfg = mc_.config_;
    auto& held = mc_.cont_jog_held_;
    auto& target_vel = mc_.cont_jog_target_vel_mm_s_;
    const auto now = Clock::now();
    bool any_axis_moving = false;
    StepSlice ms{};
    ms.outputs_mask = t.outputs_bitmask;

    for (size_t a = 0; a < 4; ++a) {
        // Dead-man: a jog input that is no longer being refreshed is treated as released
        if (held[a] &&
            now - mc_.cont_jog_refresh_[a] > std::chrono::milliseconds(MachineController::kJogDeadmanMs)) {
            held[a] = false;
            target_vel[a] = 0.0;
        }

        const auto& ax = cfg.axes[a];
        const double accel = std::max(10.0, ax.acceleration_mm_s2);
        const double decel = std::max(10.0, ax.deceleration_mm_s2);
        double& vel = cont_jog_current_vel_mm_s_[a];

        // Soft limits: start braking early enough to stop at the limit of a homed axis
        bool force_stop = false;
        if (ax.soft_limits_enabled && t.axis_homed[a] && std::abs(vel) > 1e-9) {
            const double pos = cont_jog_ideal_pos_mm_[a];
            const double dist = (vel > 0.0) ? (ax.soft_limit_max_mm - pos) : (pos - ax.soft_limit_min_mm);
            const double stop_dist = (vel * vel) / (2.0 * decel) + std::abs(vel) * kDtSec;
            if (dist <= stop_dist) force_stop = true;
        }
        if (ax.soft_limits_enabled && t.axis_homed[a] && held[a]) {
            const double pos = cont_jog_ideal_pos_mm_[a];
            const double tv = target_vel[a];
            if ((tv > 0.0 && pos >= ax.soft_limit_max_mm) || (tv < 0.0 && pos <= ax.soft_limit_min_mm)) {
                force_stop = true;
            }
        }

        if (held[a] && !force_stop) {
            const double diff = target_vel[a] - vel;
            // Speeding up uses the acceleration limit, slowing down / reversing the deceleration limit
            const bool slowing = (vel > 0.0 && diff < 0.0) || (vel < 0.0 && diff > 0.0);
            const double max_dv = (slowing ? decel : accel) * kDtSec;
            vel = (std::abs(diff) <= max_dv) ? target_vel[a] : vel + (diff > 0.0 ? max_dv : -max_dv);
        } else {
            const double max_dv = decel * kDtSec;
            vel = (std::abs(vel) <= max_dv) ? 0.0 : vel - (vel > 0.0 ? max_dv : -max_dv);
        }

        if (std::abs(vel) > 1e-6) {
            any_axis_moving = true;
            double next = cont_jog_ideal_pos_mm_[a] + vel * kDtSec;
            if (ax.soft_limits_enabled && t.axis_homed[a]) {
                next = std::clamp(next, ax.soft_limit_min_mm, ax.soft_limit_max_mm);
            }
            cont_jog_ideal_pos_mm_[a] = next;
            const int32_t target_steps = static_cast<int32_t>(std::llround(next * ax.steps_per_mm));
            ms.delta_steps[a] = target_steps - cont_jog_accum_steps_[a];
            cont_jog_accum_steps_[a] = target_steps;
            t.machine_pos_mm[a]  = static_cast<double>(target_steps) / ax.steps_per_mm;
            t.work_pos_mm[a]     = t.machine_pos_mm[a] - t.work_offset_mm[a];
            t.axis_vel_mm_min[a] = std::abs(vel) * 60.0;
        } else {
            t.axis_vel_mm_min[a] = 0.0;
        }
    }

    if (const int master = cfg.gantryMasterAxis(); master >= 0) {
        ms.delta_steps[3] = ms.delta_steps[master];
        t.axis_vel_mm_min[3] = t.axis_vel_mm_min[master];
    }

    if (any_axis_moving) {
        if (t.simulator_mode) {
            t.tick_counter += 25;
        } else if (!sendBoardSliceLocked(board::SliceStream::Internal, ms, cfg.limit_override)) {
            mc_.faultLocked("USB write failed while jogging");
        }
    } else {
        // The board still runs the last jog slices: stay in Jogging until it has had the time (see finishQueueLocked)
        if (mc_.board_->isOpen() && !t.simulator_mode && now < board_busy_until_ + 2 * kTickInterval) {
            t.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            return;
        }
        mc_.cont_jog_active_       = false;
        t.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        if (t.state == ControllerState::Jogging) {
            t.state = mc_.restStateLocked();
        }
        if (mc_.outputsLiveLocked()) {
            mc_.setBoardOutputsAfterMotionLocked(t.outputs_bitmask, cfg.limit_override);
        }
    }
}

bool SliceBackend::boardBusy() const { return false; }
bool SliceBackend::decelerating() const { return hold_decel_active_; }
bool SliceBackend::motionQueued() const { return !slice_queue_.empty(); }
bool SliceBackend::motionPending() const { return !slice_queue_.empty() || !in_flight_seqs_.empty(); }
bool SliceBackend::appSoftLimits() const { return true; }
bool SliceBackend::thcAvailable() const { return true; }
bool SliceBackend::boardRunsThc() const { return false; }
bool SliceBackend::boardRampsSpindle() const { return false; }
bool SliceBackend::positionKnownAfterStop() const { return feedback_validated_; }

ResumePoint SliceBackend::pausedResumePoint() const {
    // Resume from the segment that was actually sent last (acknowledgements may lag behind)
    return ResumePoint{last_sent_from_program_, last_sent_program_segment_valid_, last_sent_program_segment_};
}

bool SliceBackend::jobSpindleFromBoard(bool&, bool&) const { return false; }

bool SliceBackend::queuedJobSpindle(uint8_t& outputs_mask, bool& spindle_on) const {
    if (slice_queue_.empty()) return false;
    const auto& front = slice_queue_.front();
    outputs_mask = front.slice.outputs_mask;
    spindle_on = ((front.slice.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0) && !front.is_tool_change;
    return true;
}

bool SliceBackend::startManualMoves(const std::vector<ToolpathSegment>& moves, ControllerState state,
                                    ManualMovePlan plan) {
    auto& t = mc_.telemetry_;
    if (moves.empty()) return true;
    switch (plan) {
        case ManualMovePlan::Sequence: {
            // One move of a sequence: no Start Continuous Jog (the sequence's moves run like before the seam)
            const auto& s = moves.front();
            if (s.type == SegmentType::RapidMove) {
                slice_queue_ = planManualMovesLocked({s});
            } else {
                slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(s.start_xyza_mm, s.end_xyza_mm, s.feed_mm_min,
                                                                         mc_.config_, 0);
            }
            t.state = state;
            return true;
        }
        case ManualMovePlan::Mdi: {
            const bool laser = (mc_.config_.operation_mode == OperationMode::Laser && !mc_.config_.is_mill);
            MachineConfig mdi_cfg = mc_.config_;
            if (!laser) mdi_cfg.is_mill = true;
            slice_queue_ = planner::TrajectoryPlanner::sliceProgram(moves, mdi_cfg, t.feed_override_pct, 0.0f,
                                                                    t.dry_run_mode, kDtSec);
            break;
        }
        case ManualMovePlan::Machine:
            slice_queue_ = planManualMovesLocked(moves);
            break;
    }
    if (!slice_queue_.empty()) {
        if (mc_.board_->isOpen() && !t.simulator_mode) {
            mc_.board_->beginJog(Axis::X, mc_.config_.limit_override);
        }
        t.state = state;
    }
    return true;
}

void SliceBackend::startFraming(const std::vector<std::array<double, 4>>& corners, double speed_mm_min, uint8_t out_mask,
                                uint16_t beam_duty) {
    auto& t = mc_.telemetry_;
    MachineConfig manual_cfg = mc_.config_;
    manual_cfg.is_mill = true; // Framing moves: no laser Z lock / output rewriting
    slice_queue_.clear();
    auto rapid_start = planner::TrajectoryPlanner::sliceLinearMove(t.work_pos_mm, corners[0], speed_mm_min, manual_cfg, 0);
    slice_queue_.insert(slice_queue_.end(), rapid_start.begin(), rapid_start.end());
    for (size_t i = 0; i + 1 < corners.size(); ++i) {
        auto edge_slices = planner::TrajectoryPlanner::sliceLinearMove(corners[i], corners[i + 1], speed_mm_min, manual_cfg, out_mask);
        slice_queue_.insert(slice_queue_.end(), edge_slices.begin(), edge_slices.end());
    }

    if (beam_duty > 0 && mc_.board_->isOpen() && !t.simulator_mode) {
        mc_.board_->setSpindlePwm(beam_duty, mc_.config_.limit_override);
    }
    t.spindle_pwm_duty = beam_duty;
    // Framing runs as a manual move: job-only logic (dynamic laser power, THC) must not touch it
    t.state = ControllerState::Jogging;
}

void SliceBackend::jogContinuous(size_t idx, bool positive_dir, double speed_ratio) {
    auto& t = mc_.telemetry_;
    const auto& cfg = mc_.config_;
    if (!mc_.cont_jog_active_) {
        if (!slice_queue_.empty()) return; // A step jog / MDI move is still executing
        for (size_t a = 0; a < 4; ++a) {
            cont_jog_ideal_pos_mm_[a]     = t.machine_pos_mm[a];
            cont_jog_accum_steps_[a]      = static_cast<int32_t>(std::llround(t.machine_pos_mm[a] * cfg.axes[a].steps_per_mm));
            cont_jog_current_vel_mm_s_[a] = 0.0;
            mc_.cont_jog_target_vel_mm_s_[a] = 0.0;
            mc_.cont_jog_held_[a]            = false;
        }
    }

    const double max_vel = planner::TrajectoryPlanner::axisVelocityLimit(cfg, idx);
    const double target_vel = std::clamp(speed_ratio, 0.05, 1.0) * max_vel * (positive_dir ? 1.0 : -1.0);

    if (!mc_.cont_jog_held_[idx] && std::abs(cont_jog_current_vel_mm_s_[idx]) < 1e-9) {
        if (mc_.board_->isOpen() && !t.simulator_mode) {
            mc_.board_->beginJog(static_cast<Axis>(idx), cfg.limit_override);
        }
    }
    // Direction changes ramp through zero using the normal accel/decel limits (no instant reversal)
    mc_.cont_jog_held_[idx]            = true;
    mc_.cont_jog_refresh_[idx]         = Clock::now();
    mc_.cont_jog_active_               = true;
    mc_.cont_jog_target_vel_mm_s_[idx] = target_vel;
    t.state                            = ControllerState::Jogging;
}

void SliceBackend::jogStep(size_t idx, double delta_mm, double feed_mm_min) {
    auto& t = mc_.telemetry_;
    const auto& cfg = mc_.config_;
    std::array<double, 4> start_work = t.work_pos_mm;
    std::array<double, 4> end_work   = start_work;
    end_work[idx] += delta_mm;

    // Clamp the step to the soft limits of a homed axis (instead of faulting after crossing them)
    const auto& ax = cfg.axes[idx];
    if (ax.soft_limits_enabled && t.axis_homed[idx]) {
        const double off = t.work_offset_mm[idx];
        end_work[idx] = std::clamp(end_work[idx] + off, ax.soft_limit_min_mm, ax.soft_limit_max_mm) - off;
    }
    if (std::abs(end_work[idx] - start_work[idx]) < 1e-6) return;

    const double feed = std::clamp(feed_mm_min, 1.0, planner::TrajectoryPlanner::axisVelocityLimit(cfg, idx) * 60.0);
    slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(start_work, end_work, feed, cfg, t.outputs_bitmask);
    if (!slice_queue_.empty()) {
        if (mc_.board_->isOpen() && !t.simulator_mode) {
            mc_.board_->beginJog(static_cast<Axis>(idx), cfg.limit_override);
        }
        t.state = ControllerState::Jogging;
    }
}

void SliceBackend::stopManualMove() {
    // Step jog / MDI move: decelerate along the path instead of cutting the pulse stream
    if (!slice_queue_.empty()) beginHoldDecelLocked(true);
}

bool SliceBackend::stopScriptMotion() {
    if (mc_.telemetry_.state == ControllerState::Jogging && !mc_.cont_jog_active_ && !slice_queue_.empty()) {
        beginHoldDecelLocked(true);
        return true;
    }
    return false;
}

void SliceBackend::stopForEvent() {
    stopMotionForEventLocked();
}

void SliceBackend::clearQueuedMotion() {
    slice_queue_.clear();
}

void SliceBackend::startHoming(int axis) {
    homing_axis_queue_.clear();
    if (axis >= 0) {
        homing_axis_queue_.push_back(static_cast<core::Axis>(axis));
    } else {
        // Safe CNC Homing Order: Z Up first, then Y, then X
        homing_axis_queue_.push_back(core::Axis::Z);
        homing_axis_queue_.push_back(core::Axis::Y);
        homing_axis_queue_.push_back(core::Axis::X);
    }
    startNextHomingAxisLocked();
}

bool SliceBackend::startProbeMove(size_t axis, const std::array<double, 4>& start_work,
                                  const std::array<double, 4>& end_work, double feed_mm_min) {
    if (mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode) {
        mc_.board_->armProbe(static_cast<Axis>(axis));
    }
    probe_latched_at_arm_ = hw_latched_steps_;
    slice_queue_ = planner::TrajectoryPlanner::sliceLinearMove(start_work, end_work, std::max(5.0, feed_mm_min),
                                                               mc_.config_, 0);
    return true;
}

bool SliceBackend::startJob(size_t start_seg, bool resume_in_place) {
    hold_decel_active_ = false;
    hold_needs_replan_ = false;
    resume_after_hold_ = false;
    stop_after_hold_   = false;
    rebuildPlannedSlicesLocked(start_seg, resume_in_place);
    mc_.telemetry_.state = slice_queue_.empty() ? ControllerState::Idle : ControllerState::RunningGCode;
    return true;
}

bool SliceBackend::resumeHeldJob() {
    auto& t = mc_.telemetry_;
    if (hold_decel_active_) {
        // Still decelerating from the hold: resume as soon as the machine has stopped
        resume_after_hold_ = true;
        stop_after_hold_   = false;
        t.state            = ControllerState::RunningGCode;
        return true;
    }
    if (!hold_needs_replan_ && (!slice_queue_.empty() || (job_planner_ && !job_planner_->finished()))) {
        t.state = ControllerState::RunningGCode; // M0 / M1 resume: continue the queued motion
        t.active_fault_message.clear();
        return true;
    }
    return false;
}

void SliceBackend::pauseJob() {
    auto& t = mc_.telemetry_;
    t.state = ControllerState::PausedHold;
    t.current_feed_mm_min = 0.0;
    mc_.manual_spindle_requested_ = false;
    if (mc_.config_.spindle_ramp_down_s <= 0.01) {
        mc_.spindle_ramp_pct_ = 0.0;
        t.spindle_speed_pct = 0.0f;
        t.spindle_ramping_up = false;
        t.spindle_ramping_down = false;
        t.torch_or_spindle_on = false;
        t.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        if (mc_.outputsLiveLocked()) {
            mc_.setBoardOutputsLocked(t.outputs_bitmask, mc_.config_.limit_override);
        }
    }
    beginHoldDecelLocked(false);
}

bool SliceBackend::controlledStop() {
    const auto s = mc_.telemetry_.state;
    const bool path_motion = (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce ||
                              (s == ControllerState::Jogging && !mc_.cont_jog_active_) ||
                              (s == ControllerState::PausedHold && hold_decel_active_));
    if (path_motion && !slice_queue_.empty() && last_sent_target_valid_ && last_sent_speed_mm_s_ > 0.5) {
        // Controlled stop: decelerate along the path, then stop (no lost steps)
        beginHoldDecelLocked(true);
        return true;
    }
    return false;
}

void SliceBackend::stopMotion(bool is_estop) {
    const bool hw = mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode;
    if (hw && is_estop) {
        // Halt the board first, then the outputs, then drop anything still buffered
        mc_.board_->emergencyStop();
        mc_.setBoardOutputsLocked(0x00, false);
    }
    slice_queue_.clear();
    job_planner_.reset();
    in_flight_seqs_.clear();
    homing_axis_queue_.clear();
    homing_phase_              = HomingPhase::None;
    cont_jog_current_vel_mm_s_ = {0.0, 0.0, 0.0, 0.0};
    hold_decel_active_         = false;
    hold_needs_replan_         = false;
    resume_after_hold_         = false;
    stop_after_hold_           = false;
    settle_active_             = false;
    last_sent_target_valid_    = false;
    last_sent_speed_mm_s_      = 0.0;
    if (hw) {
        mc_.board_->cancelMotion();
    }
    program_run_open_     = false; // Cancel ends any open run
    program_outputs_sent_ = 0xFFFF;
}

void SliceBackend::setFeedOverride(float pct) {
    // Feed override never raises motion above the configured axis limits (enforced by the planner). The planner gets
    // every change; the slices already queued are re-planned once the override differs by 0.5 % or more from the
    // one they were planned with, so many small steps add up instead of being lost.
    if (job_planner_) {
        job_planner_->setFeedOverride(pct);
        const auto s = mc_.telemetry_.state;
        if (std::abs(queued_feed_override_pct_ - pct) >= 0.5f &&
            (s == ControllerState::RunningGCode || s == ControllerState::PausedHold)) {
            replanFromQueueTailLocked();
        }
    }
}

void SliceBackend::setPierceDelayOverride(float seconds) {
    if (job_planner_) {
        job_planner_->setPierceDelayOverride(seconds);
    }
}

void SliceBackend::setOutputs(uint8_t logical_mask, bool limit_override) {
    mc_.board_->setOutputs(io::physicalOutputs(mc_.config_.io, logical_mask), limit_override);
}

void SliceBackend::setOutputsAfterMotion(uint8_t logical_mask, bool limit_override) {
    mc_.board_->setOutputsAfterMotion(io::physicalOutputs(mc_.config_.io, logical_mask), limit_override);
}

void SliceBackend::spindleSpeedChanged() {
    if (mc_.board_->isOpen() && !mc_.telemetry_.simulator_mode) {
        mc_.board_->setSpindlePwm(mc_.telemetry_.spindle_pwm_duty, mc_.config_.limit_override);
    }
}

void SliceBackend::testFireLaserPulse(float duration_ms, float power_pct) {
    auto& t = mc_.telemetry_;
    const auto& cfg = mc_.config_;
    mc_.pulse_active_ = true;
    mc_.pulse_timer_s_ = 0.0;
    mc_.pulse_duration_s_ = std::clamp(static_cast<double>(duration_ms) / 1000.0, 0.01, 2.0);
    mc_.pulse_power_pct_ = std::clamp(power_pct, 1.0f, 100.0f);

    float duty_fraction = cfg.laser.min_duty_pct + (mc_.pulse_power_pct_ / 100.0f) * (cfg.laser.max_duty_pct - cfg.laser.min_duty_pct);
    uint16_t duty = static_cast<uint16_t>(std::clamp(duty_fraction * 10.0f, 0.0f, 1000.0f));

    t.outputs_bitmask |= core::OutputBit::OUT1_TorchOrSpindle;
    t.torch_or_spindle_on = true;
    t.spindle_pwm_duty = duty;
    t.live_laser_power_pct = mc_.pulse_power_pct_;

    if (mc_.board_->isOpen() && !t.simulator_mode) {
        mc_.board_->setSpindlePwm(duty, cfg.limit_override);
        mc_.setBoardOutputsLocked(t.outputs_bitmask, cfg.limit_override);
    }
}

} // namespace rnr::core
