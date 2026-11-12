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
    if (!state_ok || motionLocked().decelerating() || telemetry_.surface_probing_active) {
        return;
    }

    motionLocked().jogContinuous(idx, positive_dir, speed_ratio);
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
    motionLocked().jogStep(idx, delta_mm, feed_mm_min);
}

void MachineController::jogStop() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t a = 0; a < 4; ++a) {
        cont_jog_held_[a]            = false;
        cont_jog_target_vel_mm_s_[a] = 0.0;
    }
    if (!cont_jog_active_ && telemetry_.state == ControllerState::Jogging) {
        motionLocked().stopManualMove(); // Step jog / MDI move: controlled stop
    }
}

void MachineController::updateContinuousJogLocked() {
    if (!cont_jog_active_) return;
    motionLocked().updateContinuousJog();
}

} // namespace rnr::core
