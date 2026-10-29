#pragma once
#include "core/board/board_types.h"
#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace rnr::core {
struct MachineConfig;
}

// ============================================================================
// Board driver interface: everything the machine core needs from a motion controller board.
//
// The machine core (MachineController) owns the machine logic - jobs, planning, work offsets, tool table,
// homing & probing sequences, spindle/VFD, THC, scripting - and talks to the hardware only through this
// interface. A driver translates these calls into its board's wire protocol.
//
// Motion model: the core plans on the PC and streams fixed-period StepSlices (kSlicePeriodSec each), keeping
// at most maxSlicesInFlight() unacknowledged slices on the board. Boards that plan motion themselves
// (e.g. GRBL-family firmware taking G-code) need an additional motion path in the core; see
// docs/architecture/BOARD_DRIVERS.md.
//
// Threading: readStatus() is called by the core's worker thread without the core lock held; every other
// call is made with the core lock held. Drivers must therefore be safe against those two callers running at
// the same time.
// ============================================================================
namespace rnr::board {

enum class BoardType : uint8_t {
    RnrEcoMotion = 0, // RNR Eco Motion 2.0 USB card (RnRMotion.dll compatible)
};

// Which slice stream a motion chunk belongs to
enum class SliceStream : uint8_t {
    Program,  // G-code job slices
    Internal, // Jog, homing, probing and other machine-generated moves
};

struct BoardInfo {
    std::string product_name;
    std::string serial_number;
};

struct BoardCapabilities {
    std::string display_name;            // Shown in the UI, e.g. "RNR Eco Motion 2.0"
    uint8_t     input_count      = 12;   // Inputs IN1..INn usable for switches, probe and E-Stop
    uint8_t     output_count     = 4;    // Outputs OUT1..OUTn
    uint8_t     min_pulse_khz    = 10;
    uint8_t     max_pulse_khz    = 100;
    size_t      max_slices_in_flight = 3; // Unacknowledged slices the board can buffer
    bool        hardware_probe_latch = true;  // Board stops and latches positions on probe contact
    bool        hardware_home_latch  = true;  // Board stops and latches positions on a home switch
    bool        spindle_pwm          = true;  // 0-10 V / PWM spindle speed output
    bool        accesses_real_hardware = true; // False for simulated / test drivers
};

// Latest board feedback, in board-neutral form
struct BoardStatus {
    bool     motion_active   = false; // Board is executing a sequence / buffered slices
    bool     estop_fault     = false; // Hardware E-Stop input tripped
    bool     limit_fault     = false; // Hardware end-stop tripped
    uint16_t ack_seq         = 0;     // Sequence number of the last slice the board processed
    std::array<int32_t, 4> live_steps{};    // Current step position per axis
    std::array<int32_t, 4> latched_steps{}; // Position captured on probe contact / home switch
    uint16_t inputs_bitmask  = 0;     // Bit n = IN(n+1), raw electrical state
    uint16_t tick_counter    = 0;     // Board timer, for diagnostics
};

class IBoardDriver {
public:
    // Duration of one motion slice the core streams
    static constexpr double kSlicePeriodSec = 0.025;

    virtual ~IBoardDriver() = default;

    virtual BoardCapabilities capabilities() const = 0;

    // Connection
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual BoardInfo info() const = 0;

    // Feedback: drains pending reports into `out`. Returns false when no report arrived within timeout_ms.
    virtual bool readStatus(BoardStatus& out, uint32_t timeout_ms) = 0;

    // Sends pin assignments, polarities and step rate from the machine configuration.
    // `homing` = true while homing (home and limit switches that share a pin must not trip a limit fault).
    virtual void configure(const core::MachineConfig& cfg, bool homing) = 0;

    // Safety & control
    virtual void emergencyStop() = 0;      // Halt all motion immediately and disarm
    virtual void clearEmergencyStop() = 0; // Re-arm after the operator reset the E-Stop
    virtual void cancelMotion() = 0;       // Abort the active sequence and drop buffered slices

    // Outputs (OUT1..OUT4 bitmask, see core::OutputBit)
    virtual void setOutputs(uint8_t outputs_mask, bool limit_override) = 0;
    virtual void setOutputsAfterMotion(uint8_t outputs_mask, bool limit_override) = 0; // Applied when buffered motion ends
    virtual void setSpindlePwm(uint16_t duty_per_mille, bool limit_override) = 0;     // 0..1000

    // Position
    virtual void setAxisPositions(const std::array<int32_t, 4>& steps) = 0;

    // Motion
    virtual void beginJog(core::Axis axis, bool limit_override) = 0; // Start of a jog / pull-off slice stream
    virtual void armHomeSwitchStop(core::Axis axis) = 0;              // Next slices stop on that axis' home switch
    virtual void armProbe(core::Axis axis) = 0;                       // Next slices stop on probe contact
    // Queues one slice. `out_seq` receives the sequence number the board will acknowledge. False = not sent.
    virtual bool sendSlice(SliceStream stream, const core::StepSlice& slice, bool limit_override,
                           uint16_t* out_seq = nullptr) = 0;
};

// Creates the driver for a board type
std::unique_ptr<IBoardDriver> createBoardDriver(BoardType type);

} // namespace rnr::board
