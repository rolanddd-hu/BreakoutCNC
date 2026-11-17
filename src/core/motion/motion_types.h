#pragma once
#include "core/controller/motion_controller.h"
#include <array>
#include <chrono>
#include <cstdint>
#include <string>

// Board-neutral types of the motion path seam (MULTI_BOARD_PLAN.md section 4.8). The machine core sends intents
// (moves in the machine frame, probes, homing, jog velocity, jobs) and receives feedback and events.
namespace rnr::core::motion {

using Clock = std::chrono::steady_clock;

enum class PathState : uint8_t {
    Offline,    // Not connected
    Connecting, // Port open, identifying the board
    Fault,      // Alarm / error: needs Unlock or a reset
    Idle,
    Moving,     // Running a job, a move, a probe or a jog
    Holding,    // Feed hold, still decelerating
    Held,       // Feed hold, stopped (also: door open)
    Homing,
};

struct PathFeedback {
    PathState   state = PathState::Offline;
    std::string firmware_state;            // As the board reports it ("Idle", "Hold:0", "Alarm")
    std::string status_text;               // "Connecting to COM5...", alarm text; empty when nothing to say
    bool        position_valid = false;    // A status report has been received
    std::array<double, 4> machine_pos_mm{}; // App machine frame (firmware position + machine shift)
    double      feed_mm_min = 0.0;
    double      spindle_rpm = 0.0;
    std::string pins;                       // Active input letters (Pn:)
    uint64_t    inputs_mask = 0;            // Bit n = BoardCapabilities::io[n] active
    bool        probe_active = false;
    bool        door_open = false;
    bool        estop_active = false;
    bool        limit_active = false;
    bool        spindle_on = false, spindle_ccw = false, flood_on = false, mist_on = false;
    int         ov_feed = 100, ov_rapid = 100, ov_spindle = 100;
    std::array<bool, 4> homed{};
    bool        homed_reported = false;     // The board reports its homed state (grblHAL)
    // Job progress
    bool        job_active = false;
    size_t      active_segment = 0;
    size_t      active_line = 0;
    float       segment_progress = 0.0f;
    bool        progress_is_estimate = true;
    int         planner_free = -1, rx_free = -1; // Bf: (diagnostics)
    // Board-side plasma THC (grblHAL plasma plugin)
    bool        thc_reported = false;
    double      arc_voltage = 0.0;
    bool        arc_ok = false, torch_on = false, thc_enabled = false, thc_active = false;
    bool        thc_velocity_lock = false, thc_void_lock = false, thc_up = false, thc_down = false;
    uint32_t    arc_failed_count = 0;          // [MSG:arc failed] received (the plugin then holds the job)
};

struct PathEvent {
    enum class Kind : uint8_t {
        Connected,      // Identified; capabilities are final
        ConnectFailed,  // No GRBL-compatible answer (text = reason)
        MoveDone,       // startMoves() finished (ok = false: interrupted)
        ProbeDone,      // ok = contact made; position_mm = contact (app machine frame)
        HomingDone,     // ok = success
        JobSyncPoint,   // The job reached a segment the core has to handle (tool change, program stop, spindle)
        JobFinished,    // ok = ran to the end; false = stopped
        JogStopped,
        Fault,          // text = message, code = firmware alarm number (0 = other), position_lost
        Message,        // [MSG:] feedback worth showing
        SettingsChanged,
    } kind = Kind::Message;
    bool        ok = true;
    bool        position_lost = false;
    std::array<double, 4> position_mm{};
    size_t      segment_index = 0;
    int         code = 0;           // Fault: firmware alarm number; JobSyncPoint: SyncKind
    std::string text;
    // JobSyncPoint of kind SpindleChange: the spindle state the program asks for
    bool        spindle_on = false, spindle_ccw = false;
    uint16_t    spindle_rpm = 0;
};

struct MoveRequest {
    std::array<double, 4> target_mm{};   // App machine frame
    double feed_mm_min = 0.0;
    bool   rapid = false;
    double dwell_s = 0.0;                // > 0: a dwell (G4) instead of a move
};

struct ProbeRequest {
    std::array<double, 4> target_mm{};   // App machine frame: the end of the search move
    double feed_mm_min = 100.0;
};

// Spindle / laser and coolant outside a job
struct OutputRequest {
    bool   spindle_on = false;
    bool   spindle_ccw = false;
    double s_value = 0.0;     // Spindle RPM, or laser power in the board's S range
    bool   flood = false;
    bool   mist = false;
    bool   laser = false;     // M4 dynamic power / M3 constant (BreakoutCut laser)
    bool   laser_dynamic = true;
};

} // namespace rnr::core::motion
