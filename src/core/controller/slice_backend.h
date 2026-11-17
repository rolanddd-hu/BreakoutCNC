#pragma once
// Motion on host-planned boards (RNR Eco Motion 2.0) and in the simulator: the app plans the path and streams
// 25 ms step slices. Part of MachineController (see motion_backend.h); all members are used with the controller
// lock held.
#include "core/controller/motion_backend.h"
#include "core/board/board_driver.h"
#include "core/planner/trajectory_planner.h"
#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

namespace rnr::core {

class MachineController;

class SliceBackend final : public IMotionBackend {
public:
    explicit SliceBackend(MachineController& mc);

    SliceBackend(const SliceBackend&) = delete;
    SliceBackend& operator=(const SliceBackend&) = delete;

    // ---- IMotionBackend ----
    void readBoard(bool got_report, const board::BoardStatus& rep, bool usb_open_now) override;
    void onBoardConnected() override;
    bool resetFromEStop() override;
    bool readInputs() override;
    void tick(bool gcode_running) override;
    void updateFeedTelemetry() override;
    void updateContinuousJog() override;
    bool boardBusy() const override;
    bool decelerating() const override;
    bool motionQueued() const override;
    bool motionPending() const override;
    bool appSoftLimits() const override;
    bool thcAvailable() const override;
    bool boardRunsThc() const override;
    bool boardRampsSpindle() const override;
    bool positionKnownAfterStop() const override;
    ResumePoint pausedResumePoint() const override;
    bool jobSpindleFromBoard(bool& on, bool& ccw) const override;
    bool queuedJobSpindle(uint8_t& outputs_mask, bool& spindle_on) const override;
    bool startManualMoves(const std::vector<ToolpathSegment>& moves, ControllerState state,
                          ManualMovePlan plan) override;
    void startFraming(const std::vector<std::array<double, 4>>& corners, double speed_mm_min, uint8_t out_mask,
                      uint16_t beam_duty) override;
    void jogContinuous(size_t axis, bool positive_dir, double speed_ratio) override;
    void jogStep(size_t axis, double delta_mm, double feed_mm_min) override;
    void stopManualMove() override;
    bool stopScriptMotion() override;
    void stopForEvent() override;
    void clearQueuedMotion() override;
    void startHoming(int axis) override;
    bool startProbeMove(size_t axis, const std::array<double, 4>& start_work, const std::array<double, 4>& end_work,
                        double feed_mm_min) override;
    bool startJob(size_t start_seg, bool resume_in_place) override;
    bool resumeHeldJob() override;
    void pauseJob() override;
    bool controlledStop() override;
    void stopMotion(bool is_estop) override;
    void setFeedOverride(float pct) override;
    void setPierceDelayOverride(float seconds) override;
    void setOutputs(uint8_t logical_mask, bool limit_override) override;
    void setOutputsAfterMotion(uint8_t logical_mask, bool limit_override) override;
    void spindleSpeedChanged() override;
    void testFireLaserPulse(float duration_ms, float power_pct) override;

    // ---- Host-planned boards only: called by MachineController whichever backend is active ----
    void setMaxSlicesInFlight(size_t n) { max_slices_in_flight_ = n < 1 ? 1 : n; }
    // Position feedback starts over: after connecting, leaving the simulator or a new machine position
    void restartFeedbackLocked();
    // The machine position was redefined (homing, restored position, steps/mm): the board's counters and the last
    // sent target no longer describe it
    void syncBoardPositionLocked();
    void positionRedefinedLocked();
    // A new start point for the job (Run From Line): forget where the last one stopped; discard_held_job drops the
    // motion planned for a held job as well
    void forgetJobPositionLocked(bool discard_held_job);
    void dropJobPlanLocked() { job_planner_.reset(); }
    // Takes outputs off every queued slice (the laser lid opened)
    void removeOutputsFromQueueLocked(uint8_t mask);
    // The RNR board's own report: the axis being homed (its home switch is not a limit then) and fault flags
    core::Axis homingAxis() const noexcept { return active_homing_axis_; }
    bool boardLimitFault() const noexcept { return hw_limit_fault_; }
    bool boardEStopFault() const noexcept { return hw_estop_fault_; }
    const std::deque<planner::PlannedSlice>& queuedSlices() const noexcept { return slice_queue_; }

private:
    using Clock = std::chrono::steady_clock;

    // ---- Board feedback ----
    void processBoardReportLocked(bool got_report, const board::BoardStatus& rep, bool usb_open_now);

    // ---- Slice streaming ----
    void runMotionTickLocked(bool gcode_running);
    bool sendQueuedSliceLocked();
    // Program motion runs on the board like an MDI move: outputs set while the board is at rest, Start
    // Continuous Jog, slices, and End of Motion when the board runs out of slices. Returns true when a program
    // slice with these outputs may be sent now; false = wait (the board must come to rest first). With
    // arm = false it only sets the outputs if possible (spindle spin-up wait).
    bool prepareProgramRunLocked(uint8_t outputs_mask, bool arm);
    bool boardBufferFullLocked() const;
    void closeProgramRunLocked();
    void finishQueueLocked();
    bool sendBoardSliceLocked(board::SliceStream stream, const core::StepSlice& slice, bool limit_override,
                              uint16_t* out_seq = nullptr);
    // Manual moves (go to zero, retracts, tool change park): planned as plain machine moves, no laser rules
    std::deque<planner::PlannedSlice> planManualMovesLocked(const std::vector<ToolpathSegment>& moves) const;

    // ---- Jobs ----
    void rebuildPlannedSlicesLocked(size_t start_from_segment, bool resume_in_place);
    void refillJobQueueLocked();
    void replanFromQueueTailLocked();
    void beginHoldDecelLocked(bool stop_after);

    // ---- Stops on an event (probe contact, home switch) ----
    void beginSettleLocked();
    bool settledLocked() const;
    void stopMotionForEventLocked();

    // ---- App-driven homing (per axis: back off the switch -> seek it -> machine zero -> pull off) ----
    void startNextHomingAxisLocked();
    void homingTickLocked();
    bool homeSwitchActiveLocked(size_t axis) const;

    // ---- Probe moves ----
    void probingTickLocked();

    MachineController& mc_;

    size_t max_slices_in_flight_ = 3; // Unacknowledged slices kept on the board (from its capabilities)
    float  queued_feed_override_pct_ = 100.0f; // Override the queued slices were planned with
    std::deque<planner::PlannedSlice> slice_queue_{};
    std::unique_ptr<planner::StreamingPlanner> job_planner_;

    struct SeqProgressInfo {
        size_t segment_index    = 0;
        size_t gcode_line       = 0;
        float  segment_progress = 0.0f;
        bool   is_gcode_stream  = false;
        Clock::time_point sent_at{};
    };
    std::unordered_map<uint16_t, SeqProgressInfo> in_flight_seqs_{};
    // Program slices go to the board as internal motion slices in MDI-style runs (prepareProgramRunLocked)
    bool     program_run_open_     = false;  // Start Continuous Jog sent, End of Motion not yet
    uint16_t program_outputs_sent_ = 0xFFFF; // Logical outputs last set for the program, 0xFFFF = unknown
    // When the board will have run every slice sent so far (each slice is 25 ms of motion). Slices are paced by
    // this, not only by acknowledgements: the board acknowledges a slice when it is buffered, not when it has run.
    Clock::time_point board_busy_until_{};

    // Board feedback & watchdogs
    bool                   board_report_seen_   = false;
    Clock::time_point      connect_time_{};
    Clock::time_point      last_report_time_{};
    bool                   hw_estop_fault_      = false;
    bool                   hw_limit_fault_      = false;
    bool                   hw_motion_active_    = false;
    std::array<int32_t, 4> hw_live_steps_{};
    std::array<int32_t, 4> hw_latched_steps_{};
    bool                   feedback_validated_  = false; // Board step counters proven to track commanded position

    // Settle wait after a hardware stop (probe contact, home switch, cancel)
    bool              settle_active_ = false;
    bool              settle_board_idle_seen_ = false;
    Clock::time_point settle_deadline_{};

    // Last slice handed to the hardware (planning frame) - used for feed hold deceleration
    std::array<double, 4> last_sent_target_{};
    bool                  last_sent_target_valid_ = false;
    double                last_sent_speed_mm_s_   = 0.0;
    // Program segment of the last slice SENT (acknowledgements lag behind) - resume point after a hold
    size_t                last_sent_program_segment_       = 0;
    bool                  last_sent_program_segment_valid_ = false;
    bool                  last_sent_from_program_          = false; // false = inside a safe-approach prefix

    // Feed hold / stop deceleration
    bool hold_decel_active_   = false;
    bool hold_needs_replan_   = false;
    bool resume_after_hold_   = false;
    bool stop_after_hold_     = false;

    // Continuous jog ramps (the dead-man and the target velocities are the controller's)
    std::array<double, 4>  cont_jog_current_vel_mm_s_{0.0, 0.0, 0.0, 0.0};
    std::array<int32_t, 4> cont_jog_accum_steps_{0, 0, 0, 0};
    std::array<double, 4>  cont_jog_ideal_pos_mm_{0.0, 0.0, 0.0, 0.0};

    // Multi-axis homing state machine
    enum class HomingPhase : uint8_t { None, BackOff, Seek, PullOff };
    std::deque<core::Axis> homing_axis_queue_{};
    core::Axis  active_homing_axis_  = core::Axis::Z;
    HomingPhase homing_phase_        = HomingPhase::None;
    bool        homing_switch_event_ = false; // Switch transition seen; waiting for the stop to settle

    std::array<int32_t, 4> probe_latched_at_arm_{}; // The board's latched probe position when the probe was armed
};

} // namespace rnr::core
