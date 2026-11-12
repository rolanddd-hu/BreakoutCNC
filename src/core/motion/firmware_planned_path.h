#pragma once
#include "core/board/board_capabilities.h"
#include "core/board/board_profile.h"
#include "core/board/grbl/firmware_profile.h"
#include "core/board/grbl/grbl_protocol.h"
#include "core/board/grbl/grbl_settings.h"
#include "core/gcode/gcode_parser.h"
#include "core/motion/gcode_emitter.h"
#include "core/motion/motion_types.h"
#include "core/planner/trajectory_planner.h"
#include "core/transport/protocol_log.h"
#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

// Motion path for firmware-planned boards (GRBL 1.1, grblHAL, FluidNC): the board plans the motion, the app sends
// G-code lines. See docs/architecture/GRBL_FAMILY_PLAN.md.
//
// Coordinates: the app resolves work offsets, tool lengths, G53 / G28, G68 and height maps itself and sends
// firmware coordinates = app machine position - machine shift S - firmware frame offset F (section 6). Single
// moves use G53 (no F), probes and jogs are incremental (no F), jobs read F after their preamble.
//
// Threading: every method runs on the core thread with the core lock held, except emergencyStop() (any thread).
namespace rnr::core::motion {

struct JobRequest {
    std::shared_ptr<const gcode::ParseResult> program;
    std::vector<ToolpathSegment> prefix;   // Approach moves (planning frame), before program[first_index..]
    size_t first_index = 0;
    planner::FrameContext frame;           // Resolves segments to the planning frame (+ WCS offset = machine)
    bool   arcs_allowed = true;            // false: height map active (warped chords are not on a circle)
    bool   mill = true;
    bool   laser = false;                  // BreakoutCut laser mode
    bool   laser_lock_z = false;
    double laser_z_plan = 0.0;
    bool   laser_dynamic = true;           // M4
    bool   laser_air_assist = false;
    float  laser_min_duty_pct = 0.0f, laser_max_duty_pct = 100.0f;
    uint16_t laser_s_max = 1000;           // S value for 100 % power
    float  laser_power_override_pct = 100.0f;
    bool   dry_run = false;
    bool   app_drives_spindle = false;     // Huanyang VFD from the PC: spindle changes are sync points
    double spinup_dwell_s = 0.0;           // Dwell after the board switches the spindle on
    float  pierce_delay_override_s = 0.0f;
    bool   plasma = false;
    bool   ends_with_program_end = false;  // M2 / M30: coolant off at the end as well
    // Spindle state when the job starts (app-driven spindle: a change is a sync point)
    bool     spindle_on_at_start = false;
    bool     spindle_ccw_at_start = false;
    uint16_t rpm_at_start = 0;
};

enum class SyncKind : uint8_t { ToolChange, ProgramStop, SpindleChange };

class FirmwarePlannedPath {
public:
    FirmwarePlannedPath(std::string board_id, board::BoardProfile profile);
    ~FirmwarePlannedPath();
    FirmwarePlannedPath(const FirmwarePlannedPath&) = delete;
    FirmwarePlannedPath& operator=(const FirmwarePlannedPath&) = delete;

    // ---- Connection ----
    // Opens the transport and starts identifying the board (non-blocking; Connected / ConnectFailed follow)
    bool open(bool hardware_access_allowed, std::string* error);
    void close();
    bool isOpen() const { return protocol_.isOpen(); }
    bool ready() const { return phase_ == Phase::Ready; }
    // grblHAL plasma plugin with a THC mode on: the board runs Arc OK and torch height control itself
    bool boardRunsThc() const { return fw_.board_thc && fw_.flavour == board::grbl::Flavour::GrblHal; }
    bool busy() const { return op_ != Op::None || static_cast<bool>(deferred_); }
    // Busy with something a new command cannot be queued behind (motion, a job, a stop). Short command sequences
    // (outputs, console, settings, unlock) are not: commands sent meanwhile wait for them.
    bool busyWithMotion() const { return (op_ != Op::None && op_ != Op::Script) || static_cast<bool>(deferred_); }
    const std::string& boardId() const { return board_id_; }
    const board::BoardProfile& profile() const { return profile_; }
    std::string connectionLabel() const;

    void tick(Clock::time_point now);
    const PathFeedback& feedback() const { return fb_; }
    bool pollEvent(PathEvent& out);

    board::BoardCapabilities capabilities() const;
    uint64_t capabilitiesRevision() const { return caps_revision_; }
    const board::grbl::FirmwareProfile& firmware() const { return fw_; }
    std::shared_ptr<transport::ProtocolLog> log() const { return log_; }

    // Machine shift S: app machine position = firmware machine position + S (machines without home switches)
    void setMachineShift(const std::array<double, 4>& shift) { shift_ = shift; }
    const std::array<double, 4>& machineShift() const { return shift_; }
    std::array<double, 4> firmwareMachinePos() const { return fw_mpos_; }

    // ---- Commands (false + error when the board cannot take it now) ----
    bool startMoves(const std::vector<MoveRequest>& moves, const OutputRequest* during, std::string* error);
    bool startProbe(const ProbeRequest& p, std::string* error);
    bool startHoming(uint8_t axis_mask, std::string* error); // 0 = all ($H)
    bool jogStep(size_t axis, double delta_mm, double feed_mm_min, std::string* error);
    void setJogVelocity(const std::array<double, 4>& mm_per_s); // All zero = stop (dead-man handled by the core)
    bool jogActive() const { return op_ == Op::JogContinuous || op_ == Op::JogStep; }
    bool startJob(JobRequest job, std::string* error);
    bool jobActive() const { return op_ == Op::Job; }
    void continueJob();            // After a SpindleChange sync point
    void hold();
    void resume();
    void stop();                   // Controlled: hold, wait, reset, resync (position kept)
    bool stopping() const { return op_ == Op::Stopping; }
    void emergencyStop();          // Any thread: soft reset at once
    bool unlock(std::string* error);
    bool inAlarm() const { return fw_state_ == "Alarm"; }
    void setFeedOverride(float pct);
    void setRapidOverride(float pct);
    void setSpindleOverride(float pct);
    bool setOutputs(const OutputRequest& o, std::string* error);
    bool setAuxOutput(int index, bool on, std::string* error);
    bool sendConsoleLine(const std::string& line, std::string* error);
    // Plain send-response lines (laser test pulse); answers only, no motion tracking
    bool runLines(const std::vector<std::string>& lines, std::string* error);

    // ---- Board report: identification and settings answers for diagnosis (read-only commands) ----
    bool startBoardReport(std::string* error);
    bool boardReportRunning() const { return report_running_; }
    uint64_t boardReportRevision() const { return report_revision_; }
    std::string boardReportText() const; // The last report: profile, capabilities, answers, recent log

    // ---- Alarm and error texts: the board's own for codes beyond GRBL 1.1 (grblHAL $EA / $EE, FluidNC $A / $E) ----
    std::string alarmText(int code) const;
    std::string errorText(int code) const;
    size_t boardCodeTextCount() const { return board_alarm_texts_.size() + board_error_texts_.size(); }

    // Travel of axis a in the app machine frame, from $130-$133, the homing direction and build option Z.
    // False when it is not known (settings not read, homing off, FluidNC, no such axis).
    bool travelEnvelope(size_t a, double& lo, double& hi) const;

    // ---- Firmware settings ----
    const board::grbl::SettingsSnapshot& settings() const { return settings_; }
    void requestSettings();        // $$ (and $CD on FluidNC)
    bool writeSettings(const std::vector<std::pair<std::string, std::string>>& id_values, std::string* error);

    // Line of the job the tracker is on (testing / diagnostics)
    size_t trackedMoveCount() const { return sent_moves_.size(); }

private:
    enum class Phase : uint8_t { Closed, WaitWelcome, Identify, ReadSettings, ReadEnums, ReadCodes, ReadParams, Ready };
    enum class Op : uint8_t { None, Script, Moves, Probe, Homing, JogStep, JogContinuous, Job, Stopping };

    // A send-response line of a command script
    struct ScriptLine {
        std::string text;
        bool        quiet = false;  // Board answers late ($H): watchdog suspended
        bool        collect = false; // Collect text lines until the answer ($CD)
        bool        params = false; // $# answer: take F from G54
        bool        settings = false; // $$ answer
        bool        report = false;   // Board report: every answer line is kept, an error does not stop the script
    };

    struct SentMove {
        uint32_t ticket = 0;
        size_t   segment_index = 0;
        size_t   gcode_line = 0;
        bool     from_program = false;
        long     line_number = 0;
        std::array<double, 4> start{}, end{}; // App machine frame
        // G2 / G3 sent for a run of chords: the chord vertices (start, then the end of each chord) and how far the
        // true arc may lie from them. segment_index is then the first chord's segment.
        std::vector<std::array<double, 4>> arc_points;
        double   arc_sag_mm = 0.0;
        size_t lastSegment() const { return arc_points.size() > 1 ? segment_index + arc_points.size() - 2 : segment_index; }
    };

    struct PendingLine {
        std::string text;
        bool        is_move = false;
        SentMove    move;
    };

    void emit(PathEvent e) { events_.push_back(std::move(e)); }
    void fault(const std::string& text, int code, bool position_lost);
    void setStatusText(const std::string& s) { fb_.status_text = s; }
    void refreshDialect();

    // Connection phases
    void beginIdentify(Clock::time_point now);
    void handleMessage(const board::grbl::FirmwareMessage& m, Clock::time_point now);
    void onStatus(const board::grbl::StatusReport& s, Clock::time_point now);
    void onAck(const board::grbl::FirmwareMessage& m, Clock::time_point now);
    void onNack(const board::grbl::FirmwareMessage& m, Clock::time_point now);
    void onAlarm(int code);
    void onWelcome(const std::string& text, Clock::time_point now);
    void sendHandshakeLine(const std::string& line);

    // Scripts (send-response command sequences)
    bool canStart(std::string* error) const;
    void startScript(Op op, std::vector<ScriptLine> lines);
    void pumpScript(Clock::time_point now);
    void finishOp(bool ok);
    void abortOp(const std::string& why);

    // Jobs
    void pumpJob(Clock::time_point now);
    bool produceJobLines();       // false = nothing more to produce now (sync point or end)
    std::array<double, 4> resolveSegment(const ToolpathSegment& s) const;   // App machine frame
    std::array<double, 4> toFirmware(const std::array<double, 4>& app_machine, bool with_frame) const;
    const ToolpathSegment& jobSegment(size_t virtual_index) const;
    size_t jobSourceSize() const;
    bool   jobIsProgram(size_t virtual_index) const { return virtual_index >= job_.prefix.size(); }
    size_t jobProgramIndex(size_t virtual_index) const { return job_.first_index + (virtual_index - job_.prefix.size()); }
    bool   emitOutputsFor(const ToolpathSegment& s, std::vector<std::string>& lines, bool* sync_needed);
    void   updateTracker();

    // Jog feeder
    void pumpJog(Clock::time_point now);

    // Overrides
    void pumpOverrides();

    // State
    std::string board_id_;
    board::BoardProfile profile_;
    std::shared_ptr<transport::ProtocolLog> log_;
    board::grbl::GrblProtocol protocol_;
    board::grbl::FirmwareProfile fw_;
    board::grbl::Flavour selected_flavour_;
    uint64_t caps_revision_ = 1;
    GcodeEmitter emitter_;

    Phase phase_ = Phase::Closed;
    Clock::time_point phase_deadline_{};
    bool  welcome_seen_ = false;
    bool  expect_reset_ = false;         // Our own soft reset: the next welcome line is expected
    Clock::time_point next_poll_{};
    Clock::time_point last_status_{};
    bool  status_since_ack_ = false;

    PathFeedback fb_;
    std::string  fw_state_;
    int          fw_substate_ = -1;
    std::array<double, 4> fw_mpos_{};    // Firmware machine position (mm)
    std::array<double, 4> wco_{};        // Firmware work coordinate offset (mm)
    std::array<double, 4> frame_f_{};    // Frame offset F used for job coordinates
    std::array<double, 4> shift_{};      // Machine shift S
    std::deque<PathEvent> events_;

    // Current operation
    Op op_ = Op::None;
    std::deque<ScriptLine> script_;
    bool        script_waiting_ = false; // A script line is unanswered
    ScriptLine  script_current_{};
    uint32_t    script_ticket_ = 0;
    bool        script_wait_idle_ = false; // All lines answered: waiting for an Idle status
    Clock::time_point script_done_at_{};
    std::vector<std::string> collected_;
    // Board report
    bool        report_running_ = false;
    uint64_t    report_revision_ = 0;
    std::vector<std::string> report_lines_;
    // Board code texts and the handshake requests that read them
    std::map<int, std::string> board_alarm_texts_, board_error_texts_;
    std::deque<std::string> codes_queue_;
    char        codes_target_ = 0;       // 'A' (alarms) or 'E' (errors) while a code list is read
    void        readCodesOrParams(Clock::time_point now);
    bool        probe_result_seen_ = false;
    bool        probe_hit_ = false;
    std::array<double, 4> probe_pos_{};
    uint8_t     homing_mask_ = 0;
    std::string op_error_;
    std::map<std::string, std::array<double, 4>> params_; // $# answers
    std::vector<std::pair<std::string, std::string>> pending_writes_;
    bool        settings_after_op_ = false;

    // Job
    JobRequest  job_;
    size_t      job_cursor_ = 0;          // Next virtual source index to produce
    std::deque<PendingLine> job_lines_;   // Produced, not yet sent
    enum class JobPhase : uint8_t { Preamble, Streaming, Draining, Finishing, SyncDrain, SyncWait, Done } job_phase_ = JobPhase::Done;
    std::vector<ScriptLine> job_preamble_;
    size_t      job_preamble_pos_ = 0;
    bool        job_produce_blocked_ = false; // Reached a sync point or the end
    SyncKind    job_sync_kind_ = SyncKind::ToolChange;
    size_t      job_sync_segment_ = 0;
    uint32_t    job_drain_ticket_ = 0;
    bool        job_drain_acked_ = false;
    bool        job_held_ = false;
    std::array<double, 4> job_pos_{};     // App machine position after the last produced move
    uint8_t     job_outputs_ = 0xFF;      // Logical outputs emitted (0xFF = unknown)
    uint16_t    job_rpm_ = 0;
    bool        job_laser_s_set_ = false; // The job has programmed an S word (laser power from S, S0 = off)
    std::map<uint32_t, bool> acked_tickets_;
    std::deque<SentMove> sent_moves_;
    size_t      tracker_index_ = 0;
    size_t      tracker_sub_ = 0;          // Chord of the tracked arc the machine is on (only moves forward)
    bool        track_xy_only_ = false;
    size_t      acked_moves_ = 0;

    // Stop sequence
    enum class StopPhase : uint8_t { HoldWait, ResetWait, Resync } stop_phase_ = StopPhase::HoldWait;
    Clock::time_point stop_deadline_{};
    bool        stop_from_job_ = false;

    // Jog
    std::array<double, 4> jog_vel_{};       // Requested (mm/s)
    std::array<double, 4> jog_dir_{};       // Unit direction of the active jog
    double      jog_speed_ = 0.0;
    double      jog_sent_ = 0.0;            // Distance sent along jog_dir_
    std::array<double, 4> jog_start_{};
    bool        jog_cancelling_ = false;
    Clock::time_point jog_report_time_{};
    uint32_t    jog_step_ticket_ = 0;
    bool        jog_step_acked_ = false;

    // Overrides
    float ov_feed_target_ = 100.0f, ov_spindle_target_ = 100.0f;
    int   ov_feed_expected_ = 100, ov_spindle_expected_ = 100;
    int   ov_rapid_target_ = 100;
    bool  ov_rapid_dirty_ = false;
    Clock::time_point ov_last_send_{};

    // Pending manual outputs (sent when the board is idle)
    bool          outputs_pending_ = false;
    OutputRequest outputs_req_{};

    std::atomic<bool> estop_requested_{false};
    std::function<void()> deferred_;          // A job / step jog waiting for the running command sequence
    std::atomic<bool> ready_flag_{false};   // phase_ == Ready, readable from any thread (emergencyStop)
    std::atomic<int64_t> last_reset_ns_{0}; // When we last sent a soft reset (steady clock)

    // Settings
    board::grbl::SettingsSnapshot settings_;
    std::map<int, std::string> numbered_;
    std::map<std::string, board::grbl::EnumeratedSetting> enums_;
    std::map<std::string, std::string> enum_groups_;
    bool hardware_access_allowed_ = true;
    std::vector<board::IoPoint> io_cache_;
    uint64_t io_cache_rev_ = 0;
    uint32_t last_acked_ticket_ = 0;
    uint32_t job_preamble_ticket_ = 0;
    bool     job_preamble_waiting_ = false;
    std::deque<std::string> job_finish_lines_;
    bool     job_vfd_on_ = false, job_vfd_ccw_ = false;
    uint16_t job_vfd_rpm_ = 0;
    Op       stop_prev_op_ = Op::None;
    Clock::time_point jog_cancel_at_{};
    Clock::time_point script_line_sent_at_{};
};

} // namespace rnr::core::motion
