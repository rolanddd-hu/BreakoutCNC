#pragma once
#include "core/board/board_driver.h"
#include "core/hid/rnr_usb_device.h"

namespace rnr::core {
struct MachineConfig;
}

namespace rnr::board {

// RNR Eco Motion 2.0 USB card (VID 0xA720 / PID 0xF803), wire protocol from RnRMotion.dll.
// See docs/protocol/PROTOCOL_SPEC.md.
class RnrBoardDriver final : public IBoardDriver {
public:
    BoardCapabilities capabilities() const override;

    bool open() override;
    void close() override;
    bool isOpen() const override;
    BoardInfo info() const override;

    bool readStatus(BoardStatus& out, uint32_t timeout_ms) override;
    void configure(const core::MachineConfig& cfg, bool homing) override;

    void emergencyStop() override;
    void clearEmergencyStop() override;
    void cancelMotion() override;

    void setOutputs(uint8_t outputs_mask, bool limit_override) override;
    void setOutputsAfterMotion(uint8_t outputs_mask, bool limit_override) override;
    void setSpindlePwm(uint16_t duty_per_mille, bool limit_override) override;

    void setAxisPositions(const std::array<int32_t, 4>& steps) override;

    void beginJog(core::Axis axis, bool limit_override) override;
    void armHomeSwitchStop(core::Axis axis) override;
    void armProbe(core::Axis axis) override;
    bool sendSlice(SliceStream stream, const core::StepSlice& slice, bool limit_override, uint16_t* out_seq) override;

    // Board pin/polarity/step-rate packet for a machine configuration (exposed for tests)
    static protocol::BoardPinoutConfig buildPinout(const core::MachineConfig& cfg, bool homing) noexcept;

private:
    hid::RnrUsbDevice usb_;
};

} // namespace rnr::board
