#pragma once
// Motion on firmware-planned boards (GRBL 1.1, grblHAL, FluidNC): the board plans and runs the path; the app sends
// G-code through motion::FirmwarePlannedPath (MachineController::fw_) and follows the board's reports. Part of
// MachineController (see motion_backend.h); all members are used with the controller lock held.
#include "core/controller/motion_backend.h"
#include "core/motion/firmware_planned_path.h"
#include <array>
#include <chrono>
#include <vector>

namespace rnr::core {

class MachineController;

class FirmwareBackend final : public IMotionBackend {
public:
    explicit FirmwareBackend(MachineController& mc);

    FirmwareBackend(const FirmwareBackend&) = delete;
    FirmwareBackend& operator=(const FirmwareBackend&) = delete;

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

    // The board was closed: what it reported no longer applies
    void onBoardClosedLocked() { fw_arc_failed_seen_ = 0; }

private:
    using Clock = std::chrono::steady_clock;

    // Board reports and path events (worker tick)
    void fwTickLocked();
    void fwHandleEventLocked(const motion::PathEvent& e);
    // Manual moves (work coordinates, like the slice planner's input) on a firmware board
    bool fwStartManualMovesLocked(const std::vector<ToolpathSegment>& moves, ControllerState state,
                                  const motion::OutputRequest* during = nullptr);
    void fwApplyOutputsLocked(uint8_t logical_mask);
    void fwSpindleSpeedChangedLocked();
    void fwToolChangeLocked(const ToolpathSegment& seg);
    bool fwStartJobLocked(size_t start_seg, bool resume_in_place);
    double fwAxisMaxVelocityLocked(size_t axis) const;

    MachineController& mc_;

    uint8_t  fw_outputs_applied_ = 0xFF;   // Logical outputs last applied to the firmware board
    bool     fw_job_spindle_on_ = false, fw_job_spindle_ccw_ = false; // App-driven spindle during a firmware job
    bool     fw_sync_wait_ = false;        // Waiting for the spindle before continuing the job
    Clock::time_point fw_sync_continue_at_{};
    size_t   fw_resume_segment_ = 0;       // After M0 / M1 on a firmware board
    bool     fw_program_stop_ = false;
    uint8_t  fw_homing_mask_ = 0;
    uint32_t fw_arc_failed_seen_ = 0;      // PathFeedback::arc_failed_count already reported
    uint64_t fw_thc_mode_rev_ = ~0ULL;     // Capabilities revision the board THC mode was read at
    bool     fw_seen_held_ = false;        // The board reported its feed hold (resume from its own button)
    bool     suppress_fw_estop_ = false;   // Fault from the board itself: no extra reset
    Clock::time_point fw_state_grace_until_{};
    std::array<double, 4> fw_vel_pos_{};   // Position at the last velocity sample (per-axis velocity readout)
    Clock::time_point fw_vel_time_{};
};

} // namespace rnr::core
