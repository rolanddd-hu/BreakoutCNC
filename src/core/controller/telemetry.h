#pragma once
// Controller state and the telemetry snapshot the UI reads every frame.
#include "core/controller/machine_config.h"
#include "core/controller/surface_height_map.h"
#include <memory>
#include <string>
#include <vector>

namespace rnr::core {

// ============================================================================
// Real-Time Telemetry Snapshot (Published from 40Hz HID Thread -> 60Hz UI Thread)
// ============================================================================
enum class ControllerState : uint8_t {
    Disconnected   = 0,
    EStop          = 1,
    Idle           = 2,
    RunningGCode   = 3,
    PausedHold     = 4,
    Jogging        = 5,
    Homing         = 6,
    Probing        = 7,
    DwellPierce    = 8, // Plasma pierce delay / Mill G4 dwell
    ToolChangeHold = 9, // Waiting on M6 Manual Tool Change (Jog & Z Probe allowed, Cycle Start resumes)
};

struct TelemetrySnapshot {
    bool            usb_connected     = false; // The selected board is connected (USB, serial or network)
    // Board
    std::string     board_id;                  // Selected board (BoardRegistry id)
    bool            board_firmware_planned = false; // GRBL-family: the board plans the motion
    bool            board_connecting  = false; // Port open, board being identified
    bool            board_busy        = false; // Firmware board: a command is still being answered
    std::string     firmware_version;          // "grblHAL 1.1f.20230610"
    std::string     firmware_state;            // As the board reports it ("Idle", "Hold:0", "Alarm")
    std::string     board_status_text;         // Board-level status: alarm text, "Door open", connection errors
    uint64_t        capabilities_revision = 0; // Changes when getBoardCapabilities() may return something new
    uint64_t        board_inputs_mask = 0;     // Firmware boards: bit n = BoardCapabilities::io[n] active
    bool            progress_is_estimate = false; // The active line is estimated from the position
    float           rapid_override_pct = 100.0f;
    bool            simulator_mode    = false;
    bool            dry_run_mode      = false;
    ControllerState state             = ControllerState::Disconnected;
    std::string     product_name;
    std::string     serial_number;

    // Machine & Work Coordinates (in mm)
    std::array<double, 4> machine_pos_mm  = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> work_offset_mm  = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> work_pos_mm     = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    std::array<bool, 4>   axis_homed      = {false, false, false, false};

    // Work Coordinate Systems (G54 - G59)
    uint8_t                              active_workspace = 0; // 0=G54, 1=G55, ..., 5=G59
    std::array<std::array<double, 4>, 6> workspace_offsets_mm{};

    // Active Toolpath Tracking (for Cut vs. Uncut rendering in UI)
    size_t      active_gcode_line_index = 0;
    size_t      active_segment_index    = 0;
    size_t      total_segments          = 0;
    float       segment_progress_0_to_1 = 0.0f;
    double      job_elapsed_seconds     = 0.0;
    float       feed_override_pct       = 100.0f;
    double      current_feed_mm_min     = 0.0;   // Live 3D tool velocity (mm/min)
    double      commanded_feed_mm_min   = 0.0;   // Programmed segment feed * override (mm/min)
    float       pierce_delay_override_s = 0.0f;
    std::string last_mdi_command;

    // Mach3 Tool Change (M6 / T# / S#) & Spindle Ramp Live Telemetry
    uint16_t    current_tool_number     = 1;
    // Tool length offset (tool table mode): work Z is the tool tip, machine Z = work Z + WCS + offset
    uint16_t    active_tlo_tool         = 0;     // 0 = no offset (G49)
    double      active_tlo_mm           = 0.0;
    bool        tool_reference_measured = false; // Reference tool measured since startup / Z homing
    uint16_t    pending_tool_number     = 0;
    std::string pending_tool_description;
    uint16_t    commanded_spindle_rpm   = 12000;
    uint16_t    program_spindle_rpm     = 12000; // Programmatic speed from G-Code (S word)
    bool        spindle_override_active = false; // True when operator manually overrides spindle speed
    float       spindle_speed_pct       = 0.0f;  // 0..100% current spindle spin-up/down ramp
    bool        spindle_ramping_up      = false; // True while waiting for spindle to reach 100% before cutting
    bool        spindle_ramping_down    = false; // True while spindle is ramping down to 0%

    // End-Stop / Limit Fault Telemetry
    bool        limit_switch_triggered  = false;
    bool        soft_limit_triggered    = false;
    bool        limit_override_active   = false;
    bool        estop_input_active      = false; // E-Stop input pin or board hardware E-Stop currently asserted
    std::string active_fault_message;

    // Plasma Torch Height Control (THC) Live Telemetry
    bool        thc_arc_ok_active       = false;
    bool        thc_up_active           = false;
    bool        thc_down_active         = false;
    // Board-side THC (grblHAL plasma plugin): the board waits for Arc OK and corrects the torch height itself
    bool        thc_on_board            = false;
    bool        thc_board_active        = false; // Height control is correcting Z right now
    bool        thc_torch_on            = false;
    double      thc_arc_voltage         = 0.0;
    std::string thc_board_mode;                  // "Voltage", "Up/down", "Arc ok"

    // Laser Live Telemetry & Process States
    OperationMode operation_mode           = OperationMode::Plasma;
    bool          laser_chiller_flow_ok    = false; // IN2 (or configured pin)
    bool          laser_door_closed        = false; // IN3 (or configured pin)
    bool          laser_fault_active       = false; // IN4 (or configured pin)
    bool          laser_air_assist_active  = false; // OUT3
    bool          laser_red_pointer_active = false; // OUT2
    float         live_laser_power_pct     = 0.0f;  // Real-time modulated dynamic power %
    float         laser_power_override_pct = 100.0f;

    // Huanyang VFD Live RS485 Telemetry
    HuanyangVfdTelemetry vfd_telemetry{};
    bool        thc_anti_dive_locked    = false;
    double      thc_z_offset_mm         = 0.0;

    // Mill / Plasma Touch Probe (G31) Live Telemetry
    bool                  probe_contact_active    = false;
    std::array<double, 4> last_probed_work_pos_mm = {0.0, 0.0, 0.0, 0.0};
    bool                  last_probe_hit          = false; // Result of the last probeRaw() move (true = contact)

    // Surface Leveling & Auto-Bed Height Map Telemetry
    bool             surface_compensation_active = false;
    bool             surface_probing_active      = false;
    int              surface_probe_current_point = 0;
    int              surface_probe_total_points  = 0;
    // Immutable shared snapshot (cheap to copy every UI frame); never null
    std::shared_ptr<const SurfaceHeightMap> surface_height_map = std::make_shared<const SurfaceHeightMap>();

    // Hardware & Simulated I/O States
    bool     torch_or_spindle_on      = false; // OUT1
    uint8_t  outputs_bitmask          = 0;     // Logical: output channels (bits 0..3) + custom outputs (bits 4..7)
    uint8_t  physical_outputs_bitmask = 0;     // What the board drives on OUT1..OUT4 (after the pin assignment)
    uint16_t inputs_bitmask           = 0;     // Effective IN1..IN12 (Hardware OR Simulated)
    uint16_t simulated_inputs_bitmask = 0;     // Virtual IN1..IN12 toggles for testing
    uint16_t spindle_pwm_duty         = 0;     // Mill PWM
    uint16_t last_ack_seq             = 0;
    uint16_t tick_counter             = 0;

    // Script Engine Live Telemetry
    bool                     script_running        = false;
    std::string              active_script_name;
    bool                     has_script_prompt     = false;
    std::string              script_prompt_title;
    std::string              script_prompt_message;
    std::vector<std::string> script_prompt_buttons;
    uint8_t                  script_prompt_input_kind = 0; // 0 = buttons only, 1 = text (cnc.input), 2 = number
    std::string              script_prompt_input_default;
    double                   script_prompt_input_min  = -1e300;
    double                   script_prompt_input_max  = 1e300;
};

} // namespace rnr::core
