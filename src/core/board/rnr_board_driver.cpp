#include "core/board/rnr_board_driver.h"
#include "core/controller/motion_controller.h"

namespace rnr::board {

BoardCapabilities RnrBoardDriver::capabilities() const {
    BoardCapabilities caps;
    caps.display_name         = "RNR Eco Motion 2.0";
    caps.input_count          = 12;
    caps.output_count         = 4;
    caps.min_pulse_khz        = 10;
    caps.max_pulse_khz        = 100;
    caps.max_slices_in_flight = 3; // 75 ms ahead of the acknowledgement
    caps.hardware_probe_latch = true;
    caps.hardware_home_latch  = true;
    caps.spindle_pwm          = true;
    caps.accesses_real_hardware = true;
    return caps;
}

bool RnrBoardDriver::open() {
    return usb_.open();
}

void RnrBoardDriver::close() {
    usb_.close();
}

bool RnrBoardDriver::isOpen() const {
    return usb_.isOpen();
}

BoardInfo RnrBoardDriver::info() const {
    const auto dev = usb_.deviceInfo();
    return {dev.product_name, dev.serial_number};
}

bool RnrBoardDriver::readStatus(BoardStatus& out, uint32_t timeout_ms) {
    if (!usb_.isOpen()) return false;
    // The board reports at its own rate: keep only the newest report
    protocol::BoardStatusReport rep{};
    bool got_any = false;
    for (int i = 0; i < 64; ++i) {
        protocol::BoardStatusReport tmp{};
        if (!usb_.readStatusReport(tmp, got_any ? 0 : timeout_ms)) break;
        rep = tmp;
        got_any = true;
    }
    if (!got_any) return false;

    out.motion_active  = rep.motion_state != 0;
    out.estop_fault    = rep.isInHardwareEStop();
    out.limit_fault    = rep.isLimitTriggered();
    out.ack_seq        = rep.ack_seq;
    out.live_steps     = rep.live_steps;
    out.latched_steps  = rep.latched_steps;
    out.inputs_bitmask = rep.inputs_bitmask;
    out.tick_counter   = rep.tick_counter;
    return true;
}

protocol::BoardPinoutConfig RnrBoardDriver::buildPinout(const core::MachineConfig& cfg, bool homing) noexcept {
    protocol::BoardPinoutConfig hw{};
    for (size_t a = 0; a < 4; ++a) {
        hw.axes[a].limit_plus_pin     = cfg.axes[a].limit_plus_pin;
        hw.axes[a].limit_minus_pin    = cfg.axes[a].limit_minus_pin;
        hw.axes[a].home_pin           = cfg.axes[a].home_pin;
        hw.axes[a].limit_plus_invert  = cfg.axes[a].limit_plus_invert;
        hw.axes[a].limit_minus_invert = cfg.axes[a].limit_minus_invert;
        hw.axes[a].home_invert        = cfg.axes[a].home_invert;
        hw.axes[a].step_active_low    = cfg.axes[a].step_active_low;
        hw.axes[a].dir_active_low     = cfg.axes[a].dir_active_low;
    }
    hw.probe_pin               = cfg.probe.probe_pin;
    hw.probe_invert            = cfg.probe.probe_invert;
    hw.estop_pin               = cfg.estop_pin;
    hw.estop_invert            = cfg.estop_invert;
    hw.pulse_rate_khz          = cfg.pulse_rate_khz;
    hw.slice_interval_ms       = static_cast<uint8_t>(IBoardDriver::kSlicePeriodSec * 1000.0 + 0.5);
    hw.mask_shared_home_limits = homing;
    return hw;
}

void RnrBoardDriver::configure(const core::MachineConfig& cfg, bool homing) {
    usb_.sendFullConfigureBoard(buildPinout(cfg, homing));
}

void RnrBoardDriver::emergencyStop() {
    // Ahead of any queued slices / commands: the controller drops all pending motion on an E-stop anyway
    usb_.sendUrgentShortCommand(protocol::Opcode::EStopAbort);
}

void RnrBoardDriver::clearEmergencyStop() {
    usb_.sendShortCommand(protocol::Opcode::PreResetClear, true);
    usb_.sendShortCommand(protocol::Opcode::ClearEStopEnable, true);
}

void RnrBoardDriver::cancelMotion() {
    usb_.sendShortCommand(protocol::Opcode::CancelSequence, true);
}

void RnrBoardDriver::setOutputs(uint8_t outputs_mask, bool limit_override) {
    usb_.sendShortCommand(protocol::Opcode::SetOutputsIdle, limit_override, outputs_mask);
}

void RnrBoardDriver::setOutputsAfterMotion(uint8_t outputs_mask, bool limit_override) {
    usb_.sendShortCommand(protocol::Opcode::EndOfMotionOutputs, limit_override, outputs_mask);
}

void RnrBoardDriver::setSpindlePwm(uint16_t duty_per_mille, bool limit_override) {
    // arg5 = 0x01 (PWM channel), arg7_8 = duty, arg9_12 = full scale (1000)
    usb_.sendShortCommand(protocol::Opcode::SetSpindlePwm, limit_override, 0x01, 0x00, duty_per_mille, 1000);
}

void RnrBoardDriver::setAxisPositions(const std::array<int32_t, 4>& steps) {
    usb_.sendSetAxisPositions(steps, true);
}

void RnrBoardDriver::beginJog(core::Axis axis, bool limit_override) {
    usb_.sendShortCommand(protocol::Opcode::StartContJog, limit_override, static_cast<uint8_t>(axis));
}

void RnrBoardDriver::armHomeSwitchStop(core::Axis axis) {
    usb_.sendShortCommand(protocol::Opcode::StartAxisHome, true, static_cast<uint8_t>(axis), 0x01);
}

void RnrBoardDriver::armProbe(core::Axis axis) {
    usb_.sendShortCommand(protocol::Opcode::StartProbeG31, true, static_cast<uint8_t>(axis), 0x01);
}

bool RnrBoardDriver::sendSlice(SliceStream stream, const core::StepSlice& slice, bool limit_override, uint16_t* out_seq) {
    protocol::MotionSlice ms{};
    ms.delta_steps  = slice.delta_steps;
    ms.outputs_mask = slice.outputs_mask;
    const auto op = (stream == SliceStream::Program) ? protocol::Opcode::StreamGcodeSlice : protocol::Opcode::StreamJogSlice;
    return usb_.sendMotionSlice(op, ms, limit_override, out_seq);
}

std::unique_ptr<IBoardDriver> createBoardDriver(BoardType type) {
    switch (type) {
        case BoardType::RnrEcoMotion:
        default:
            return std::make_unique<RnrBoardDriver>();
    }
}

} // namespace rnr::board
