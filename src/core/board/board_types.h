#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

// Board-neutral motion & I/O types shared by the machine core, the planner, the UI and every board driver.
namespace rnr::core {

enum class Axis : uint8_t {
    X = 0,
    Y = 1,
    Z = 2,
    A = 3,
};

// Logical machine outputs (OUT1..OUT4). A board driver maps them to its own terminals.
namespace OutputBit {
    constexpr uint8_t OUT1_TorchOrSpindle = (1 << 0); // Primary Plasma Torch / Mill Spindle Relay
    constexpr uint8_t OUT2_SpindleDir     = (1 << 1);
    constexpr uint8_t OUT3_FloodCoolant   = (1 << 2);
    constexpr uint8_t OUT4_MistCoolant    = (1 << 3);
}

// One fixed-period chunk of host-planned motion: signed step counts per axis and the outputs held during it
struct StepSlice {
    std::array<int32_t, 4> delta_steps{};
    uint8_t                outputs_mask = 0;
};

} // namespace rnr::core
