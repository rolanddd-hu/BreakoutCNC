#pragma once
// The seam between MachineController and the two ways boards move.
//
//  * SliceBackend (slice_backend.h): host-planned boards (RNR Eco Motion 2.0) and the simulator. The app plans
//    the path, streams 25 ms step slices, follows the board's step counters and runs homing itself.
//  * FirmwareBackend (firmware_backend.h): firmware-planned boards (GRBL family). The board plans and runs the
//    path, homes and probes itself; the app sends G-code and follows the board's reports.
//
// The controller keeps what both share: the state machine and telemetry, the work frame and tool table, spindle /
// VFD, laser and THC rules, the checks before a command, the probing sequences and scripting. It asks the backend of
// the active board (MachineController::motionLocked()) to move. Every call is made with the controller lock held,
// after the controller's own checks (state, soft limits, interlocks).
#include "core/board/board_driver.h"
#include "core/controller/motion_controller.h"
#include "core/controller/toolpath_segment.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rnr::core {

// How the slice planner plans manual moves (firmware boards run every kind the same way)
enum class ManualMovePlan : uint8_t {
    Machine,  // Plain machine moves, no laser rules; the board's jog run is started (go to zero, park moves)
    Mdi,      // MDI: the laser rules, the feed override and dry run apply; the board's jog run is started
    Sequence, // One move of a probing / measuring sequence: a rapid is a machine move, a feed move runs at its feed
};

// Where Cycle Start continues a paused job
struct ResumePoint {
    bool   in_place    = false; // Continue the interrupted segment from where the tool is
    bool   has_segment = false; // `segment` is set (else the controller's active segment)
    size_t segment     = 0;
};

class IMotionBackend {
public:
    virtual ~IMotionBackend() = default;

    // ---- Board link ----
    // Worker tick: read what the board reported (the RNR status report is read outside the lock and passed in)
    virtual void readBoard(bool got_report, const board::BoardStatus& rep, bool usb_open_now) = 0;
    virtual void onBoardConnected() = 0;
    // RESET E-STOP: checks and arms the board. false = refused (telemetry_.active_fault_message says why)
    virtual bool resetFromEStop() = 0;
    // The board's inputs into the telemetry. true: the board handles limits, the door and the E-stop itself, so the
    // controller's pin checks do not apply
    virtual bool readInputs() = 0;

    // ---- Every tick ----
    virtual void tick(bool gcode_running) = 0;
    virtual void updateFeedTelemetry() = 0;
    virtual void updateContinuousJog() = 0; // Called while the controller's continuous jog is active

    // ---- Questions before and while commanding motion ----
    virtual bool boardBusy() const = 0;         // The board cannot take a command now
    virtual bool decelerating() const = 0;      // A feed hold or controlled stop is still decelerating
    virtual bool motionQueued() const = 0;      // Planned motion not yet sent to the board
    virtual bool motionPending() const = 0;     // Queued, or sent and not yet acknowledged
    virtual bool appSoftLimits() const = 0;     // false: the board enforces its own soft limits
    virtual bool thcAvailable() const = 0;      // Torch height control (and Wait for Arc OK) can run
    virtual bool boardRunsThc() const = 0;
    virtual bool boardRampsSpindle() const = 0; // The board waits for spin-up itself (no app ramp)
    virtual bool positionKnownAfterStop() const = 0; // The position is read back after an E-stop mid-motion
    virtual ResumePoint pausedResumePoint() const = 0;
    // The spindle of a running job as the board reports it (false: the app drives it)
    virtual bool jobSpindleFromBoard(bool& on, bool& ccw) const = 0;
    // The outputs of the next queued job slice (false: none queued)
    virtual bool queuedJobSpindle(uint8_t& outputs_mask, bool& spindle_on) const = 0;

    // ---- Moves (work coordinates) ----
    virtual bool startManualMoves(const std::vector<ToolpathSegment>& moves, ControllerState state,
                                  ManualMovePlan plan) = 0;
    // Laser framing: around the corners, out_mask on the edges, beam_duty (0.1 %) for a low-power beam
    virtual void startFraming(const std::vector<std::array<double, 4>>& corners, double speed_mm_min, uint8_t out_mask,
                              uint16_t beam_duty) = 0;
    virtual void jogContinuous(size_t axis, bool positive_dir, double speed_ratio) = 0;
    virtual void jogStep(size_t axis, double delta_mm, double feed_mm_min) = 0;
    virtual void stopManualMove() = 0;          // Step jog / MDI move: controlled stop
    virtual bool stopScriptMotion() = 0;        // A cancelled script's move; false = not handled here
    virtual void stopForEvent() = 0;            // Stop the running move at once (cancelled surface probing)
    virtual void clearQueuedMotion() = 0;
    virtual void startHoming(int axis) = 0;     // axis < 0: all axes
    virtual bool startProbeMove(size_t axis, const std::array<double, 4>& start_work,
                                const std::array<double, 4>& end_work, double feed_mm_min) = 0;

    // ---- Jobs ----
    virtual bool startJob(size_t start_seg, bool resume_in_place) = 0;
    virtual bool resumeHeldJob() = 0;           // Cycle Start in a hold that continues the motion already planned
    virtual void pauseJob() = 0;
    virtual bool controlledStop() = 0;          // true: decelerating / stopping, the stop completes later
    virtual void stopMotion(bool is_estop) = 0; // Cancel everything at once
    virtual void setFeedOverride(float pct) = 0;
    virtual void setPierceDelayOverride(float seconds) = 0;

    // ---- Outputs ----
    virtual void setOutputs(uint8_t logical_mask, bool limit_override) = 0;
    virtual void setOutputsAfterMotion(uint8_t logical_mask, bool limit_override) = 0;
    virtual void spindleSpeedChanged() = 0;
    virtual void testFireLaserPulse(float duration_ms, float power_pct) = 0;
};

} // namespace rnr::core
