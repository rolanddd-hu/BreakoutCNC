#pragma once
#include "core/controller/motion_controller.h"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

// Input / output pin assignment helpers (IoConfig): translation of the logical output mask to the board's OUT1..OUT4,
// names of the output channels, and the list of functions configured on each input. Pure functions, no state.
namespace rnr::core::io {

// Logical outputs mask (channel bits 0..3 + custom bits 4..7) -> physical OUT1..OUT4 bits.
// A custom bit only drives its pin while no channel is assigned to that pin.
uint8_t physicalOutputs(const IoConfig& io, uint8_t logical_mask) noexcept;

// Output channel (0..3) assigned to a physical output (1..4), or -1 for a custom output
int channelForPin(const IoConfig& io, int pin_1_to_4) noexcept;

// Logical bit that switches physical output pin_1_to_4: its channel's bit, or its custom bit
uint8_t logicalBitForPin(const IoConfig& io, int pin_1_to_4) noexcept;

// Channel names for the current application / mode: full ("Mist coolant (M7)") and short ("Mist")
const char* channelName(int channel, bool is_mill, OperationMode mode) noexcept;
const char* channelShortName(int channel, bool is_mill, OperationMode mode) noexcept;

// Pins out of range become 0, a pin used by two channels stays with the lower channel, and an unconnected enable
// channel gets the first free output. Returns true when something had to be changed.
bool sanitize(IoConfig& io) noexcept;

// One function configured on an input pin
struct InputUse {
    std::string label;   // "X home", "Probe", "Arc OK"
    std::string section; // Settings section that configures it ("Limits & Homing")
    bool        invert = false;
};

// Functions configured on IN1..IN12 (index 0 = IN1) for the current application / mode. An input without any
// use is free for a custom input.
std::array<std::vector<InputUse>, kInputCount> describeInputs(const MachineConfig& cfg);

// Short label for the I/O strip: the custom name, the single function, "X home+lim" style for an axis, or the
// number of functions
std::string inputShortLabel(const std::vector<InputUse>& uses, const std::string& custom_name);

// Custom input pin (1..12) with this name (case-insensitive), or 0
int inputPinByName(const IoConfig& io, const std::string& name) noexcept;

} // namespace rnr::core::io
