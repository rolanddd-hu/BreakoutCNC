// MachineController: Spindle, torch, laser and outputs (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

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
    if (outputsLiveLocked()) {
        setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
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
    if (outputsLiveLocked()) {
        setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
    }
}

void MachineController::setPhysicalOutput(uint8_t out_1_to_4, bool enabled) {
    if (out_1_to_4 < 1 || out_1_to_4 > kPhysicalOutputCount) return;
    int channel = -1;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        channel = io::channelForPin(config_.io, out_1_to_4);
        if (channel < 0) {
            // Custom output: no machine function, just the pin
            if (enabled && (telemetry_.state == ControllerState::EStop ||
                            telemetry_.state == ControllerState::Disconnected)) {
                return;
            }
            const uint8_t bit = io::logicalBitForPin(config_.io, out_1_to_4);
            if (enabled) {
                telemetry_.outputs_bitmask |= bit;
            } else {
                telemetry_.outputs_bitmask &= ~bit;
            }
            if (outputsLiveLocked()) {
                setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
            }
            return;
        }
    }
    setAuxOutput(static_cast<uint8_t>(channel + 1), enabled); // Same checks as the function itself (VFD, dry run)
}

void MachineController::setBoardOutputsLocked(uint8_t logical_mask, bool limit_override) {
    motionLocked().setOutputs(logical_mask, limit_override);
}

void MachineController::setBoardOutputsAfterMotionLocked(uint8_t logical_mask, bool limit_override) {
    motionLocked().setOutputsAfterMotion(logical_mask, limit_override);
}





void MachineController::setSpindlePwm(uint16_t duty) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_spindle_pwm_duty_    = std::min<uint16_t>(duty, 1000);
    telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;

    telemetry_.commanded_spindle_rpm   = clampRpm((static_cast<uint32_t>(target_spindle_pwm_duty_) * spindleMaxRpmLocked()) / 1000);
    telemetry_.spindle_override_active = (telemetry_.commanded_spindle_rpm != telemetry_.program_spindle_rpm);

    motionLocked().spindleSpeedChanged();
    if (config_.vfd.enabled) {
        vfd_controller_.setRpm(telemetry_.commanded_spindle_rpm);
    }
}

uint32_t MachineController::spindleMaxRpmLocked() const noexcept {
    return (config_.vfd.enabled && config_.vfd.max_rpm > 0) ? config_.vfd.max_rpm : kDefaultSpindleMaxRpm;
}

void MachineController::applyCommandedRpmLocked(uint32_t rpm, bool send_pwm) {
    telemetry_.commanded_spindle_rpm = clampRpm(rpm);
    // 0-1000 duty cycle for the 0-10V AVI output on the RnR USB board
    target_spindle_pwm_duty_ = rpmToDuty(rpm, spindleMaxRpmLocked());
    // BreakoutCut: the PWM output is the laser power, set only by the laser code (never from an RPM)
    if (config_.is_mill) {
        telemetry_.spindle_pwm_duty = target_spindle_pwm_duty_;
        if (send_pwm) {
            motionLocked().spindleSpeedChanged();
        }
    }
    if (config_.vfd.enabled) {
        vfd_controller_.setRpm(telemetry_.commanded_spindle_rpm);
    }
}

void MachineController::setProgramSpindleRpmLocked(uint16_t rpm, bool reset_override, bool send_pwm) {
    telemetry_.program_spindle_rpm = rpm;
    if (reset_override) telemetry_.spindle_override_active = false;
    if (!telemetry_.spindle_override_active) applyCommandedRpmLocked(rpm, send_pwm);
}

void MachineController::setSpindleSpeedRpm(uint32_t rpm) {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint32_t max_limit = spindleMaxRpmLocked();
    const uint32_t min_limit = std::min<uint32_t>(
        (config_.vfd.enabled && config_.vfd.min_rpm > 0) ? config_.vfd.min_rpm : kDefaultSpindleMinRpm, max_limit);
    applyCommandedRpmLocked(std::clamp(rpm, min_limit, max_limit), true);
    telemetry_.spindle_override_active = (telemetry_.commanded_spindle_rpm != telemetry_.program_spindle_rpm);
}

void MachineController::resetSpindleSpeedOverride() {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.spindle_override_active = false;
    applyCommandedRpmLocked(telemetry_.program_spindle_rpm, true);
}

void MachineController::setOperationMode(OperationMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked()) {
        telemetry_.active_fault_message = "Operation mode cannot change while the machine is moving";
        return;
    }
    config_.operation_mode = mode;
    ++config_revision_;
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
    if (outputsLiveLocked()) {
        setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
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
    if (outputsLiveLocked()) {
        setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
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
    if (!laserInterlocksOkLocked()) {
        telemetry_.active_fault_message = "Laser test pulse blocked: safety interlock not satisfied (door / chiller / PSU)";
        return;
    }
    motionLocked().testFireLaserPulse(duration_ms, power_pct);
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
        } else if (!laserInterlocksOkLocked()) {
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

    motionLocked().startFraming(pts, speed, out_mask, framing_duty);
}

} // namespace rnr::core
