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
    const uint8_t homing_pin = homing ? config_.axes[static_cast<size_t>(slice_->homingAxis())].home_pin : 0;
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
        slice_->boardLimitFault()) {
        if (desc) *desc = "END-STOP HIT: board hardware limit fault";
        return true;
    }
    return false;
}

void MachineController::evaluateInputsAndEndStopsLocked() {
    if (motionLocked().readInputs()) return; // The board handles its limits, door and E-stop itself

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
                          slice_->boardEStopFault();
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
            slice_->removeOutputsFromQueueLocked(core::OutputBit::OUT1_TorchOrSpindle);
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

void MachineController::updateSpindleLocked(bool gcode_running) {
    bool want_spindle_on = false;
    uint8_t source_mask = telemetry_.outputs_bitmask;
    auto& motion = motionLocked();
    // Firmware board running a job: the display follows the board (or the app's VFD state at the sync points)
    bool board_on = false, board_ccw = false;
    const bool fw_job = (gcode_running || telemetry_.state == ControllerState::PausedHold) &&
                        motion.jobSpindleFromBoard(board_on, board_ccw);
    if (telemetry_.state != ControllerState::EStop && telemetry_.state != ControllerState::Disconnected &&
        !telemetry_.dry_run_mode) {
        if (fw_job) {
            want_spindle_on = board_on;
            source_mask = (board_on ? core::OutputBit::OUT1_TorchOrSpindle : 0) |
                          (board_ccw ? core::OutputBit::OUT2_SpindleDir : 0);
        } else if (gcode_running && motion.queuedJobSpindle(source_mask, want_spindle_on)) {
            // The next queued job slice switches the spindle
        } else {
            want_spindle_on = manual_spindle_requested_;
        }
    }

    // A board-driven spindle on a firmware board: the board waits for spin-up itself (G4 after M3), no app ramp
    const bool board_spindle = motion.boardRampsSpindle() && !config_.vfd.enabled;
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




IMotionBackend& MachineController::motionLocked() noexcept {
    if (fwActiveLocked()) return *firmware_;
    return *slice_;
}

const IMotionBackend& MachineController::motionLocked() const noexcept {
    if (fwActiveLocked()) return *firmware_;
    return *slice_;
}

IMotionBackend& MachineController::boardLinkLocked() noexcept {
    if (fw_ && !telemetry_.simulator_mode) return *firmware_;
    return *slice_;
}

std::deque<planner::PlannedSlice> MachineController::getSliceQueue() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return slice_->queuedSlices();
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

            boardLinkLocked().readBoard(got_any, rep, usb_open_now);
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
            motionLocked().tick(gcode_running);

            // Live 3D feed rate & commanded feed rate for the DRO feed box
            motionLocked().updateFeedTelemetry();
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
