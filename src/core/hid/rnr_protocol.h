#pragma once
#include "core/board/board_types.h"
#include <cstddef>
#include <cstdint>
#include <array>

namespace rnr::protocol {

// ============================================================================
// USB HID Hardware Identifiers (RNR ECO MOTION 2.0)
// ============================================================================
constexpr uint16_t kVendorId  = 0xA720;
constexpr uint16_t kProductId = 0xF803;
constexpr uint16_t kUsagePage = 0xFF00;
constexpr uint16_t kUsage     = 0x0001;

constexpr size_t kOutputReportBytes = 27; // 0x1B bytes (PC -> Board)
constexpr size_t kInputReportBytes  = 49; // 0x31 bytes (Board -> PC)

// ============================================================================
// Frame Types (buf[0])
// ============================================================================
enum class FrameType : uint8_t {
    ShortControl   = 0x02, // Short commands (E-Stop, outputs, jog/home/probe arming, ...)
    ExtendedPacket = 0x03, // Motion slices, axis positions, board configuration
};

// ============================================================================
// Command Opcodes (buf[1])
// ============================================================================
enum class Opcode : uint8_t {
    EStopAbort        = 0x01, // FrameType::ShortControl - Immediate halt
    ClearEStopEnable  = 0x02, // FrameType::ShortControl - Enable after 0x07
    SetOutputsIdle    = 0x03, // FrameType::ShortControl - buf[5] = OUT1..OUT4 mask
    StreamGcodeSlice  = 0x04, // FrameType::ExtendedPacket - 4-axis motion slice + outputs
    StreamJogSlice    = 0x05, // FrameType::ExtendedPacket - Internal jog/home/probe slice
    EndOfMotionOutputs= 0x06, // FrameType::ShortControl - buf[5] = OUT1..OUT4 mask
    PreResetClear     = 0x07, // FrameType::ShortControl - Sent before 0x02 on Reset
    CancelSequence    = 0x08, // FrameType::ShortControl - Abort active sequence
    SetAxisPositions  = 0x09, // FrameType::ExtendedPacket - Sync X/Y/Z/A step accumulators
    StartContJog      = 0x0A, // FrameType::ShortControl - Continuous jog / Home pull-off
    StartStepMpgJog   = 0x0B, // FrameType::ShortControl - Step / MPG jog
    StartProbeG31     = 0x0C, // FrameType::ShortControl - Arm hardware G31 probe latch
    StartAxisHome     = 0x0D, // FrameType::ShortControl - Arm hardware home switch stop
    SetSpindlePwm     = 0x0E, // FrameType::ShortControl - Set 0-10V AVI PWM duty & divider
    ConfigureBoard    = 0x0F, // FrameType::ExtendedPacket - Pinout, inversions, kHz pulse rate
    StopJogDecel      = 0x10, // FrameType::ShortControl - Begin jog deceleration
};

// ============================================================================
// Axis & I/O Bitmasks
// ============================================================================
// Board-neutral types live in core/board/board_types.h; the RNR wire format uses the same values
using Axis = ::rnr::core::Axis;
namespace OutputBit = ::rnr::core::OutputBit;

namespace FaultFlag {
    constexpr uint8_t HardwareLimitTriggered = 0x10; // rx[2] bit 4
    constexpr uint8_t HardwareEStopTriggered = 0x20; // rx[2] bit 5
}

// ============================================================================
// Decoded 49-Byte Input Status Report (Board -> PC)
// ============================================================================
struct BoardStatusReport {
    uint8_t  report_id      = 0;     // rx[0]
    uint8_t  motion_state   = 0;     // rx[1] (0 = Idle/Done, 1 = Sequence active)
    uint8_t  fault_flags    = 0;     // rx[2] (0x10 = Limit hit, 0x20 = E-Stop hit)
    uint16_t ack_seq        = 0;     // rx[3..4] Last processed packet sequence number
    uint32_t status_word    = 0;     // rx[5..8] Buffer / spindle counter

    // Live 32-bit step accumulators (rx[9, 17, 25, 33])
    std::array<int32_t, 4> live_steps    = {0, 0, 0, 0};

    // Hardware-latched 32-bit step positions on Probe (G31) / Home trigger (rx[13, 21, 29, 37])
    std::array<int32_t, 4> latched_steps = {0, 0, 0, 0};

    uint8_t  aux_status     = 0;     // rx[41]
    uint16_t inputs_bitmask = 0;     // rx[42..43] Bits 0..3 = IN1..IN4, Bits 4..11 = IDC IN5..IN12
    uint16_t tick_counter   = 0;     // rx[44..45] 200Hz hardware timer tick for velocity calculation
    uint16_t mpg_raw_count  = 0;     // rx[46..47] Quadrature handwheel counter (>> 2)

    bool isInHardwareEStop() const noexcept { return (fault_flags & FaultFlag::HardwareEStopTriggered) != 0; }
    bool isLimitTriggered()  const noexcept { return (fault_flags & FaultFlag::HardwareLimitTriggered) != 0; }
    bool getInputPin(uint8_t pin_1_to_12) const noexcept {
        if (pin_1_to_12 < 1 || pin_1_to_12 > 12) return false;
        return ((inputs_bitmask >> (pin_1_to_12 - 1)) & 1U) != 0;
    }
};

// ============================================================================
// Motion Slice Payload (Opcode 0x04 & 0x05, 27-byte Output Report)
// ============================================================================
struct MotionSlice {
    uint32_t header_flags = 0;              // buf[5..8]
    std::array<int32_t, 4> delta_steps{};   // buf[9..24] dX, dY, dZ, dA fixed-point/step deltas
    uint8_t  outputs_mask = 0;              // buf[25] Synchronized OUT1..OUT4 state during slice
};

using OutputReportBuffer = std::array<uint8_t, kOutputReportBytes>;
using InputReportBuffer  = std::array<uint8_t, kInputReportBytes>;

// ============================================================================
// Little-Endian Serialization Helpers
// ============================================================================
inline uint16_t readLE16(const uint8_t* p) noexcept {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t readLE32(const uint8_t* p) noexcept {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline int32_t readLE32Signed(const uint8_t* p) noexcept {
    return static_cast<int32_t>(readLE32(p));
}

inline void writeLE16(uint8_t* p, uint16_t val) noexcept {
    p[0] = static_cast<uint8_t>(val & 0xFF);
    p[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
}

inline void writeLE32(uint8_t* p, uint32_t val) noexcept {
    p[0] = static_cast<uint8_t>(val & 0xFF);
    p[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((val >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((val >> 24) & 0xFF);
}

// ============================================================================
// Decode 49-Byte Input Report
// ============================================================================
inline BoardStatusReport parseStatusReport(const uint8_t* rx, size_t len) noexcept {
    BoardStatusReport r{};
    if (!rx || len < kInputReportBytes) {
        return r;
    }
    r.report_id      = rx[0];
    r.motion_state   = rx[1];
    r.fault_flags    = rx[2];
    r.ack_seq        = readLE16(&rx[3]);
    r.status_word    = readLE32(&rx[5]);

    for (size_t axis = 0; axis < 4; ++axis) {
        const size_t base = 9 + axis * 8;
        r.live_steps[axis]    = readLE32Signed(&rx[base]);
        r.latched_steps[axis] = readLE32Signed(&rx[base + 4]);
    }

    r.aux_status     = rx[41];
    // Digital inputs bitmask IN1..IN12 (rx[42] = low byte, rx[43] = high byte)
    r.inputs_bitmask = readLE16(&rx[42]);
    r.tick_counter   = readLE16(&rx[44]);
    r.mpg_raw_count  = readLE16(&rx[46]) >> 2;
    return r;
}

// ============================================================================
// Build 27-Byte Output Reports
// ============================================================================
inline OutputReportBuffer buildShortControlPacket(
    Opcode opcode,
    uint16_t seq,
    bool limit_override = false,
    uint8_t arg5 = 0,
    uint8_t arg6 = 0,
    uint16_t arg7_8 = 0,
    uint32_t arg9_12 = 0) noexcept
{
    OutputReportBuffer buf{};
    buf[0] = static_cast<uint8_t>(FrameType::ShortControl);
    buf[1] = static_cast<uint8_t>(opcode);
    buf[2] = limit_override ? 0x40 : 0x00;
    writeLE16(&buf[3], seq);
    buf[5] = arg5;
    buf[6] = arg6;
    writeLE16(&buf[7], arg7_8);
    writeLE32(&buf[9], arg9_12);
    return buf;
}

inline OutputReportBuffer buildMotionSlicePacket(
    Opcode opcode, // StreamGcodeSlice (0x04) or StreamJogSlice (0x05)
    uint16_t seq,
    const MotionSlice& slice,
    bool limit_override = false) noexcept
{
    OutputReportBuffer buf{};
    buf[0] = static_cast<uint8_t>(FrameType::ExtendedPacket);
    buf[1] = static_cast<uint8_t>(opcode);
    buf[2] = limit_override ? 0x40 : 0x00;
    writeLE16(&buf[3], seq);
    writeLE32(&buf[5], slice.header_flags);
    for (size_t i = 0; i < 4; ++i) {
        writeLE32(&buf[9 + i * 4], static_cast<uint32_t>(slice.delta_steps[i]));
    }
    buf[25] = slice.outputs_mask;
    buf[26] = 0x00;
    return buf;
}

inline OutputReportBuffer buildSetAxisPositionsPacket(
    uint16_t seq,
    const std::array<int32_t, 4>& axis_steps,
    bool limit_override = false) noexcept
{
    OutputReportBuffer buf{};
    buf[0] = static_cast<uint8_t>(FrameType::ExtendedPacket);
    buf[1] = static_cast<uint8_t>(Opcode::SetAxisPositions);
    buf[2] = limit_override ? 0x40 : 0x00;
    writeLE16(&buf[3], seq);
    for (size_t i = 0; i < 4; ++i) {
        writeLE32(&buf[5 + i * 4], static_cast<uint32_t>(axis_steps[i]));
    }
    return buf;
}

// ============================================================================
// Exact STM32 Hardware Pinout, Polarity & Timing Configuration (Opcode 0x0F)
// Normal layout, and the homing variant that masks limits sharing a pin with a home switch
// ============================================================================
struct AxisHardwarePinConfig {
    uint8_t limit_plus_pin     = 0; // 0 = disabled, 1..12 = IN1..IN12
    uint8_t limit_minus_pin    = 0;
    uint8_t home_pin           = 0;
    bool    limit_plus_invert  = false;
    bool    limit_minus_invert = false;
    bool    home_invert        = false;
    bool    step_active_low    = false;
    bool    dir_active_low     = false;
};

struct BoardPinoutConfig {
    std::array<AxisHardwarePinConfig, 4> axes{};
    uint8_t probe_pin               = 0;
    bool    probe_invert            = false;
    uint8_t mpg_pin                 = 0;
    bool    mpg_invert              = false;
    uint8_t estop_pin               = 0;
    bool    estop_invert            = false;
    uint8_t pulse_rate_khz          = 100;
    uint8_t slice_interval_ms       = 25;
    bool    mask_shared_home_limits = false; // True during axis homing
};

inline OutputReportBuffer buildFullConfigureBoardPacket(
    uint16_t seq,
    const BoardPinoutConfig& cfg) noexcept
{
    OutputReportBuffer buf{};
    buf[0] = static_cast<uint8_t>(FrameType::ExtendedPacket);
    buf[1] = static_cast<uint8_t>(Opcode::ConfigureBoard);
    buf[2] = 0x00;
    writeLE16(&buf[3], seq);

    // buf[5..12]: 2 bytes per axis (X=0, Y=1, Z=2, A=3)
    for (size_t a = 0; a < 4; ++a) {
        uint8_t lp = cfg.axes[a].limit_plus_pin & 0x0F;
        uint8_t lm = cfg.axes[a].limit_minus_pin & 0x0F;
        uint8_t hm = cfg.axes[a].home_pin & 0x0F;

        // Homing: if an axis shares the same physical input pin for Home and Limit,
        // mask out the Limit assignment during Homing so hardware latches Home instead of Hard Limit fault.
        if (cfg.mask_shared_home_limits && hm != 0) {
            if (lp == hm) lp = 0;
            if (lm == hm) lm = 0;
        }

        uint8_t inv_mask = (cfg.axes[a].limit_plus_invert  ? 0x01 : 0x00) |
                           (cfg.axes[a].limit_minus_invert ? 0x02 : 0x00) |
                           (cfg.axes[a].home_invert        ? 0x04 : 0x00);

        buf[5 + a * 2] = static_cast<uint8_t>((lm << 4) | lp);
        buf[6 + a * 2] = static_cast<uint8_t>((inv_mask << 4) | hm);
    }

    // buf[13]: Low nibble = Probe pin, High nibble = MPG/Trig pin
    buf[13] = static_cast<uint8_t>(((cfg.mpg_pin & 0x0F) << 4) | (cfg.probe_pin & 0x0F));

    // buf[14]: Low nibble = E-Stop pin, High nibble = invert mask (bit0=Probe, bit1=MPG, bit2=EStop)
    uint8_t aux_inv = (cfg.probe_invert ? 0x01 : 0x00) |
                      (cfg.mpg_invert   ? 0x02 : 0x00) |
                      (cfg.estop_invert ? 0x04 : 0x00);
    buf[14] = static_cast<uint8_t>((aux_inv << 4) | (cfg.estop_pin & 0x0F));

    // buf[15]: Low nibble = Step LowActive (bits 0..3 = X,Y,Z,A), High nibble = Dir LowActive (bits 0..3 = X,Y,Z,A)
    uint8_t step_inv = 0;
    uint8_t dir_inv  = 0;
    for (size_t a = 0; a < 4; ++a) {
        if (cfg.axes[a].step_active_low) step_inv |= static_cast<uint8_t>(1U << a);
        if (cfg.axes[a].dir_active_low)  dir_inv  |= static_cast<uint8_t>(1U << a);
    }
    buf[15] = static_cast<uint8_t>((dir_inv << 4) | step_inv);

    // buf[16]: Max Pulse Rate in kHz (1..100)
    buf[16] = cfg.pulse_rate_khz;

    // buf[17]: Motion slice interval in ms (default 25)
    buf[17] = cfg.slice_interval_ms;

    return buf;
}

inline OutputReportBuffer buildConfigureBoardPacket(
    uint16_t seq,
    uint8_t max_pulse_freq_khz = 100,
    uint8_t step_dir_invert_mask = 0x00,
    uint8_t input_invert_low = 0x00,
    uint8_t input_invert_high = 0x00) noexcept
{
    OutputReportBuffer buf{};
    buf[0] = static_cast<uint8_t>(FrameType::ExtendedPacket);
    buf[1] = static_cast<uint8_t>(Opcode::ConfigureBoard);
    buf[2] = 0x00;
    writeLE16(&buf[3], seq);
    buf[7]  = input_invert_low;
    buf[8]  = input_invert_high;
    buf[15] = step_dir_invert_mask; // Low nibble = Step active-low (X..A), High nibble = Dir active-low (X..A)
    buf[16] = max_pulse_freq_khz;   // PulseRate kHz (10..100)
    buf[17] = 25;                   // 25ms (40Hz) motion slice interval
    return buf;
}

} // namespace rnr::protocol

