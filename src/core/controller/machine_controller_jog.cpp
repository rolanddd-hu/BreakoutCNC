// MachineController: Jogging (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

// ============================================================================
// Jogging
// ============================================================================
void MachineController::jogStartContinuous(core::Axis axis, bool positive_dir, double speed_ratio) {
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    // A slaved motor only moves with its master (RNR; firmware boards gang motors in the firmware)
    if (idx == 3 && config_.gantryMasterAxis() >= 0 && config_.board_id == board::kRnrBoardId) return;
    const auto s = telemetry_.state;
    const bool state_ok = (s == ControllerState::Idle || s == ControllerState::ToolChangeHold ||
                           (s == ControllerState::Jogging && cont_jog_active_));
    if (!state_ok || hold_decel_active_ || telemetry_.surface_probing_active) {
        return;
    }

    if (fwActiveLocked()) {
        // Firmware board: the core keeps the dead-man and the target velocity, the jog feeder streams $J= lines
        if (!fw_->ready()) return;
        if (!cont_jog_active_) {
            if (fw_->busyWithMotion()) return;
            cont_jog_held_ = {false, false, false, false};
            cont_jog_target_vel_mm_s_ = {0.0, 0.0, 0.0, 0.0};
        }
        const double max_vel = fwAxisMaxVelocityLocked(idx);
        cont_jog_held_[idx]            = true;
        cont_jog_refresh_[idx]         = Clock::now();
        cont_jog_active_               = true;
        cont_jog_target_vel_mm_s_[idx] = std::clamp(speed_ratio, 0.05, 1.0) * max_vel * (positive_dir ? 1.0 : -1.0);
        telemetry_.state               = ControllerState::Jogging;
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
    // A slaved motor only moves with its master (RNR; firmware boards gang motors in the firmware)
    if (idx == 3 && config_.gantryMasterAxis() >= 0 && config_.board_id == board::kRnrBoardId) return;
    if (!readyForCommandLocked(true, "Jog")) return;

    if (fwActiveLocked()) {
        const double feed = std::clamp(feed_mm_min, 1.0, fwAxisMaxVelocityLocked(idx) * 60.0);
        std::string err;
        if (!fw_->jogStep(idx, delta_mm, feed, &err)) {
            telemetry_.active_fault_message = "Jog rejected: " + err;
            return;
        }
        telemetry_.state = ControllerState::Jogging;
        return;
    }

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
    if (fwActiveLocked() && !cont_jog_active_ && telemetry_.state == ControllerState::Jogging && fw_->busy()) {
        fw_->stop(); // Jog cancel / controlled stop of the move
    }
}

void MachineController::updateContinuousJogLocked() {
    if (!cont_jog_active_) return;
    const auto now = Clock::now();
    if (fwActiveLocked()) {
        std::array<double, 4> vel{};
        bool any_held = false;
        for (size_t a = 0; a < 4; ++a) {
            if (cont_jog_held_[a] && now - cont_jog_refresh_[a] > std::chrono::milliseconds(kJogDeadmanMs)) {
                cont_jog_held_[a] = false; // Dead-man: no refresh = released
                cont_jog_target_vel_mm_s_[a] = 0.0;
            }
            if (cont_jog_held_[a]) {
                vel[a] = cont_jog_target_vel_mm_s_[a];
                any_held = true;
            }
        }
        fw_->setJogVelocity(vel);
        if (!any_held && !fw_->jogActive()) {
            cont_jog_active_ = false;
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            if (telemetry_.state == ControllerState::Jogging) {
                telemetry_.state = restStateLocked();
            }
        }
        return;
    }
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
        } else if (!sendBoardSliceLocked(board::SliceStream::Internal, ms, config_.limit_override)) {
            faultLocked("USB write failed while jogging");
        }
    } else {
        // The board still runs the last jog slices: stay in Jogging until it has had the time (see finishQueueLocked)
        if (board_->isOpen() && !telemetry_.simulator_mode && now < board_busy_until_ + 2 * kTickInterval) {
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            return;
        }
        cont_jog_active_           = false;
        telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        if (telemetry_.state == ControllerState::Jogging) {
            telemetry_.state = restStateLocked();
        }
        if (outputsLiveLocked()) {
            setBoardOutputsAfterMotionLocked(telemetry_.outputs_bitmask, config_.limit_override);
        }
    }
}

} // namespace rnr::core
