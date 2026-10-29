#pragma once
#include "core/board/board_types.h"
#include <array>
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <memory>
#include <map>
#include "core/spindle/huanyang_vfd.h"

namespace rnr::gcode {
struct ParseResult;
}

namespace rnr::core {

// ============================================================================
// System Units of Measurement (Metric vs Imperial)
// ============================================================================
enum class LinearUnits : uint8_t {
    Metric   = 0, // Millimeters (mm, mm/min, mm/s)
    Imperial = 1, // Inches (in, in/min [IPM], in/s)
};

// ============================================================================
// Operation Mode (Sheet Metal Suite: Plasma Cutting vs Laser Cutting / Engraving)
// ============================================================================
enum class OperationMode : uint8_t {
    Plasma = 0,
    Laser  = 1,
};

enum class LaserSourceType : uint8_t {
    Diode    = 0, // 450nm Blue / Infrared Diode (0-100% linear, 1-2 kHz PWM)
    Co2Glass = 1, // DC-excited CO2 glass tube (8-12% min striking threshold, 85-90% max safe duty, water chiller required)
    RfCo2    = 2, // RF metal tube CO2 (0-100% linear, 5 kHz PWM)
    Fiber    = 3, // 1064nm Fiber laser (fast gating, red-dot framing essential)
    Custom   = 4, // Fully user-customized duty and frequency limits
};

enum class LaserFramingMode : uint8_t {
    LowPowerBeam  = 0, // Trace perimeter using actual laser beam at non-marking 0.5% - 1.0% power
    RedDotPointer = 1, // Trace perimeter using visible Red Dot Crosshair (OUT2) with optical X/Y offset
    Both          = 2, // Activate both Red Dot (OUT2) and low-power beam simultaneously
    DryRun        = 3, // Trace perimeter with laser and red dot completely OFF
};

// ============================================================================
// CNC Laser Configuration (PWM, Dynamic Power M4, Air Assist & Interlocks)
// ============================================================================
struct LaserConfig {
    bool             enabled                = false;
    LaserSourceType  source_type            = LaserSourceType::Diode;
    LaserFramingMode framing_mode           = LaserFramingMode::LowPowerBeam;
    bool             dynamic_power_m4       = true;  // M4 velocity-proportional laser power scaling
    float            pwm_frequency_khz      = 1.0f;  // Base PWM frequency (1 to 5 kHz)
    float            min_duty_pct           = 0.0f;  // Striking threshold (e.g. 8-12% for CO2, 0% for Diode)
    float            max_duty_pct           = 100.0f; // Safe power ceiling (e.g. 85-90% for CO2, 100% for Diode)
    float            corner_power_floor_pct = 10.0f; // Minimum duty floor during corner deceleration
    bool             ignore_z_motion        = true;  // Lock focal plane (suppress G-code Z moves during cutting)
    bool             auto_air_assist        = true;  // Automatically energize OUT3 air solenoid on laser fire
    float            air_preflow_s          = 0.20f; // Delay before laser fires to pressurize air nozzle
    float            air_postflow_s         = 0.50f; // Delay after laser extinguishes to clear vapor and cool lens
    float            framing_power_pct      = 0.5f;  // Low-power beam percentage during framing (0.1% to 5.0%)
    double           framing_speed_mm_min   = 3000.0;
    float            pulse_duration_ms      = 50.0f; // Test pulse duration (10 to 500 ms)
    float            pulse_power_pct        = 10.0f; // Test pulse power percentage (1 to 50%)

    // 4 Physical Optocoupled Screw Terminal Inputs (IN1..IN4 on RNR Eco Motion 2.0)
    uint8_t          chiller_flow_pin       = 2;     // Physical IN2 (0 = disabled, 1..4 = IN1..IN4)
    bool             chiller_flow_invert    = false; // Invert: checked = contact to GND indicates normal flow
    uint8_t          door_interlock_pin     = 3;     // Physical IN3 (0 = disabled, 1..4 = IN1..IN4)
    bool             door_interlock_invert  = false; // Invert: checked = contact to GND indicates door closed
    uint8_t          laser_fault_pin        = 4;     // Physical IN4 (0 = disabled, 1..4 = IN1..IN4)
    bool             laser_fault_invert     = false;

    // Optical Red-Dot Aiming Pointer Calibration Offset
    double           red_dot_offset_x_mm    = 0.0;
    double           red_dot_offset_y_mm    = 0.0;
};

// ============================================================================
// Plasma Torch Height Control (THC) Configuration
// ============================================================================
struct ThcConfig {
    bool    enabled              = true;
    bool    wait_for_arc_ok      = false; // Wait for Arc OK (Transfer) before XY motion starts
    uint8_t arc_ok_pin           = 2;     // Default IN2 (0 = disabled, 1..12 = IN1..IN12)
    uint8_t thc_up_pin           = 3;     // Default IN3
    uint8_t thc_down_pin         = 4;     // Default IN4
    bool    arc_ok_invert        = false;
    bool    thc_up_invert        = false;
    bool    thc_down_invert      = false;
    double  adjust_speed_mm_s    = 4.0;   // Z correction speed (mm/s) when THC UP/DOWN is active
    float   min_xy_feed_pct      = 80.0f; // Anti-dive lock when XY velocity < 80% of target feed
    double  delay_after_pierce_s = 0.30;  // THC activation delay after pierce
    double  z_min_mm             = -10.0; // Minimum allowed Z height during THC tracking
    double  z_max_mm             = 30.0;  // Maximum allowed Z height during THC tracking
};

// ============================================================================
// Mill & Plasma Touch Probe Configuration (G31)
// ============================================================================
struct ProbeConfig {
    uint8_t probe_pin            = 5;     // Default IN5 (0 = disabled, 1..12 = IN1..IN12)
    bool    probe_invert         = false;
    double  plate_thickness_mm   = 15.0;  // Z Touch-Plate / Tool-Setter height (mm)
    double  tool_diameter_mm     = 6.0;   // Probe tip / endmill diameter for XY edge compensation
    double  fast_feed_mm_min     = 300.0; // Approach feedrate (mm/min)
    double  slow_feed_mm_min     = 80.0;  // Fine latch feedrate (mm/min)
    double  max_travel_mm        = 40.0;  // Max search distance before probe alarm
    double  retract_mm           = 3.0;   // Back-off retract distance after contact
    double  approx_hole_diam_mm  = 25.0;  // Default diameter for XY Inside Hole Center probing

    // Probing macro geometry (used by scripts/mill/probing/*.lua via cnc.get_probe_config())
    double  xy_clearance_mm                 = 10.0;  // Distance the probe moves past an outside edge before descending
    double  probe_depth_mm                  = 6.0;   // How far Z drops below the start height for outside side-probing
    double  approx_size_x_mm                = 50.0;  // Approx. X width of web/boss/slot for CENTER routines
    double  approx_size_y_mm                = 50.0;  // Approx. Y width of web/boss/slot for CENTER routines

    // Surface Level Probing & Height Map Warping settings
    double  surface_max_depth_below_zero_mm = 3.0;   // Max safe plunge depth below Z 0.0 during grid probing
    double  surface_clearance_z_mm          = 3.0;   // Safe transit clearance height in Work Z between probe points
    double  surface_smoothness              = 0.5;   // Default smoothness factor (0.0 = Bilinear, 1.0 = Bicubic Spline)
    int     surface_max_points              = 30;    // Maximum selectable points on grid resolution sliders
};

// ============================================================================
// Probed Workpiece Surface Height Map (Bilinear & Bicubic Spline Warping)
// ============================================================================
struct SurfaceHeightMap {
    bool                is_valid = false;
    double              min_x    = 0.0;
    double              max_x    = 0.0;
    double              min_y    = 0.0;
    double              max_y    = 0.0;
    int                 points_x = 0;
    int                 points_y = 0;
    std::vector<double> grid_z{}; // Row-major: row j (Y), col i (X) -> j * points_x + i
    double              smoothness = 0.5; // 0.0 = Pure Bilinear (Rigid), 1.0 = Pure Bicubic Spline (Smooth)

    // Statistics
    double min_z          = 0.0;
    double max_z          = 0.0;
    double mean_z         = 0.0;
    double peak_to_peak_z = 0.0;

    void clear() {
        is_valid = false;
        min_x = max_x = min_y = max_y = 0.0;
        points_x = points_y = 0;
        grid_z.clear();
        smoothness = 0.5;
        min_z = max_z = mean_z = peak_to_peak_z = 0.0;
    }

    void recalculateStats() {
        if (grid_z.empty()) {
            min_z = max_z = mean_z = peak_to_peak_z = 0.0;
            return;
        }
        min_z = grid_z.front();
        max_z = grid_z.front();
        double sum = 0.0;
        for (double z : grid_z) {
            if (z < min_z) min_z = z;
            if (z > max_z) max_z = z;
            sum += z;
        }
        mean_z = sum / static_cast<double>(grid_z.size());
        peak_to_peak_z = max_z - min_z;
    }

    double getInterpolatedZ(double x, double y) const {
        return getInterpolatedZWithSmoothness(x, y, smoothness);
    }

    double getInterpolatedZWithSmoothness(double x, double y, double user_smoothness) const {
        if (!is_valid || points_x < 2 || points_y < 2 ||
            grid_z.size() != static_cast<size_t>(points_x * points_y))
        {
            return 0.0;
        }
        double span_x = max_x - min_x;
        double span_y = max_y - min_y;
        if (span_x <= 1e-6 || span_y <= 1e-6) return 0.0;

        // Clamp x, y to bounding box for smooth edge extrapolation
        double cx = std::clamp(x, min_x, max_x);
        double cy = std::clamp(y, min_y, max_y);

        double u = ((cx - min_x) / span_x) * static_cast<double>(points_x - 1);
        double v = ((cy - min_y) / span_y) * static_cast<double>(points_y - 1);

        int i = std::clamp(static_cast<int>(std::floor(u)), 0, points_x - 2);
        int j = std::clamp(static_cast<int>(std::floor(v)), 0, points_y - 2);

        double t = u - static_cast<double>(i); // [0.0, 1.0]
        double s = v - static_cast<double>(j); // [0.0, 1.0]

        double z00 = grid_z[static_cast<size_t>(j * points_x + i)];
        double z10 = grid_z[static_cast<size_t>(j * points_x + (i + 1))];
        double z01 = grid_z[static_cast<size_t>((j + 1) * points_x + i)];
        double z11 = grid_z[static_cast<size_t>((j + 1) * points_x + (i + 1))];

        double z_bilinear = (1.0 - t) * (1.0 - s) * z00 +
                            t * (1.0 - s) * z10 +
                            (1.0 - t) * s * z01 +
                            t * s * z11;

        double alpha = std::clamp(user_smoothness, 0.0, 1.0);
        if (alpha <= 1e-4 || points_x < 3 || points_y < 3) {
            return z_bilinear;
        }

        // Bicubic Catmull-Rom Spline Interpolation:
        // C1 continuous tangents matching adjacent cell boundaries
        auto sample_node = [&](int col, int row) -> double {
            col = std::clamp(col, 0, points_x - 1);
            row = std::clamp(row, 0, points_y - 1);
            return grid_z[static_cast<size_t>(row * points_x + col)];
        };

        // 1D Catmull-Rom polynomial evaluator across p_{-1}, p_0, p_1, p_2
        auto catmull_rom_1d = [](double p_minus, double p0, double p1, double p_plus, double frac) -> double {
            double a = -0.5 * p_minus + 1.5 * p0 - 1.5 * p1 + 0.5 * p_plus;
            double b = p_minus - 2.5 * p0 + 2.0 * p1 - 0.5 * p_plus;
            double c = -0.5 * p_minus + 0.5 * p1;
            double d = p0;
            return ((a * frac + b) * frac + c) * frac + d;
        };

        // Evaluate 4 rows in X at parameter t
        double row_vals[4];
        for (int r = 0; r < 4; ++r) {
            int row_idx = j - 1 + r;
            double p_m = sample_node(i - 1, row_idx);
            double p_0 = sample_node(i,     row_idx);
            double p_1 = sample_node(i + 1, row_idx);
            double p_p = sample_node(i + 2, row_idx);
            row_vals[r] = catmull_rom_1d(p_m, p_0, p_1, p_p, t);
        }

        // Evaluate across the 4 row values in Y at parameter s
        double z_bicubic = catmull_rom_1d(row_vals[0], row_vals[1], row_vals[2], row_vals[3], s);

        // Blend bilinear stiffness and bicubic curvature based on smoothness alpha
        return (1.0 - alpha) * z_bilinear + alpha * z_bicubic;
    }
};

// ============================================================================
// Mach3 ATC (Auto Tool Changer) & Manual Tool Change Configuration (M6 / T# / G43)
// ============================================================================
enum class ToolChangeMode : uint8_t {
    ManualPrompt = 0, // Stop spindle, retract Z, pause in ToolChangeHold for manual swap + Z touch-off, wait for Cycle Start
    AutoAtc      = 1, // Automatic Tool Changer sequence (retract, actuate ATC dwell, apply offset, auto-continue)
    IgnoreM6     = 2, // Ignore M6 commands (e.g. single-tool or Plasma profile)
};

// ============================================================================
// Tool Length Offsets (G43 H<n> / G49) & Tool Table
// ============================================================================
enum class ToolLengthMode : uint8_t {
    NotConfigured    = 0, // G43 is ignored and the program shows a warning (default until the user chooses)
    TouchOffEachTool = 1, // G43 is ignored silently: Z is touched off (plate / setter) after every tool change
    ToolTable        = 2, // G43 H<n> applies the measured length of tool n from the tool table
};

// Tool lengths are RELATIVE to the reference tool (length 0): a tool that touches the tool setter
// 12.5 mm higher than the reference tool has length +12.5 mm.
struct ToolTableEntry {
    double      length_mm   = 0.0;
    double      diameter_mm = 0.0;
    std::string description;
};

struct ToolTableConfig {
    ToolLengthMode                     mode           = ToolLengthMode::NotConfigured;
    uint16_t                           reference_tool = 1; // Measured first; defines length 0
    std::map<uint16_t, ToolTableEntry> tools;              // Tool number 1..255 -> entry
};

// ToolpathSegment::tlo_tool value before the program issued any G43 / G49: the offset that is active
// when the job starts stays in effect.
constexpr uint16_t kTloInherit = 0xFFFF;

struct ToolChangeConfig {
    ToolChangeMode mode              = ToolChangeMode::ManualPrompt;
    double         safe_z_mm         = 25.0;  // Z retract height for tool change
    bool           move_to_park_xy   = false; // Move to fixed XY tool-change position on M6
    double         park_x_mm         = 0.0;
    double         park_y_mm         = 0.0;
    double         atc_dwell_seconds = 2.5;   // Swap duration in AutoAtc mode
};

// ============================================================================
// Machine & Per-Axis Configuration (Shared by Plasma & Mill Profiles)
// ============================================================================
struct AxisConfig {
    double  steps_per_mm        = 200.0;
    double  max_velocity_mm_s   = 100.0;  // 6000 mm/min
    double  acceleration_mm_s2  = 500.0;  // Motor acceleration ramp (mm/s^2)
    double  deceleration_mm_s2  = 600.0;  // Motor deceleration ramp (mm/s^2)
    double  home_pulloff_mm     = 2.5;    // Back-off distance after hitting home switch
    double  homing_feed_mm_min  = 900.0;  // Homing seek speed (mm/min)
    bool    homing_dir_negative = true;   // True = seek toward - (X/Y), False = seek toward + (Z)
    double  homing_max_travel_mm = 1500.0; // Max seek distance before "home switch not found" alarm
    bool    step_active_low     = false;
    bool    dir_active_low      = false;

    // End-Stop & Home Switch Pins (0 = disabled, 1..12 = IN1..IN12)
    uint8_t limit_plus_pin      = 0;
    uint8_t limit_minus_pin     = 0;
    uint8_t home_pin            = 0;
    bool    limit_plus_invert   = false;
    bool    limit_minus_invert  = false;
    bool    home_invert         = false;

    // Software Travel Limits (checked after homing or when enabled)
    bool    soft_limits_enabled = false;
    double  soft_limit_min_mm   = -5.0;
    double  soft_limit_max_mm   = 1200.0;
};

// ============================================================================
// Scripting & Automation Configuration (Lua 5.4 Engine)
// ============================================================================
struct MacroButtonConfig {
    std::string label;
    std::string path;
    bool        show_on_control_panel = false;
};

struct ScriptingConfig {
    bool        enabled             = true;
    std::string scripts_dir         = "scripts";
    bool        m6_script_enabled   = true;
    std::string m6_script_path      = "scripts/mill/m6.lua";
    bool        home_script_enabled = false;
    std::string home_script_path    = "scripts/mill/home_all.lua";
    bool        ihs_script_enabled  = false;
    std::string ihs_script_path     = "scripts/cut/m3_touch.lua";

    // Dynamic Customizable UI Macro Buttons
    std::vector<MacroButtonConfig> macro_buttons = {
        {"Tool Touch",   "scripts/mill/probe_z.lua"},
        {"Warmup",       "scripts/mill/spindle_warmup.lua"},
        {"Park",         "scripts/mill/park.lua"},
        {"Laser Offset", "scripts/cut/laser_offset.lua"}
    };
};

// ============================================================================
// Optical Camera Alignment ("Position by Camera") & Remote Monitoring Config
// ============================================================================
struct CameraConfig {
    // Role Assignments (-1 = Disabled / None, 0..N-1 = Camera Device Index)
    int         top_down_camera_index   = -1;
    std::string top_down_camera_name;
    int         monitoring_camera_index = -1;
    std::string monitoring_camera_name;

    // Top-Down Camera XY Calibration (BreakoutMill)
    bool        is_calibrated           = false;
    double      offset_x_mm             = 45.0;  // Work X coord when camera crosshair is over tool (0,0) dent
    double      offset_y_mm             = 30.0;  // Work Y coord when camera crosshair is over tool (0,0) dent
    double      fov_x_mm                = 120.0; // Physical horizontal field of view on workpiece (mm)
    double      fov_y_mm                = 67.5;  // Physical vertical field of view on workpiece (mm)
    double      cal_move_x_mm           = 15.0;  // Known X move distance (mm) during pixel-distance FOV calibration
    float       overlay_opacity         = 0.85f; // Opacity of camera video under G-code toolpath in 2D view
    bool        mirror_x                = false; // Flip image horizontally if camera is mounted inverted
    bool        mirror_y                = false; // Flip image vertically if camera is mounted inverted

    // Built-in Remote Webcam Monitoring Server (HTTP + MJPEG + E-Stop)
    bool        stream_enabled          = false;
    uint16_t    stream_port             = 8080;
    int         stream_fps              = 15;
    // Anyone on the network can reach the server (no authentication): remote G-code upload is opt-in
    bool        stream_allow_upload     = false;
};

// Dual-motor gantry: the A motor output drives the second motor of X or Y
enum class GantrySlave : uint8_t {
    None      = 0,
    AFollowsX = 1,
    AFollowsY = 2,
};

struct MachineConfig {
    LinearUnits     linear_units      = LinearUnits::Metric;
    std::array<AxisConfig, 4> axes{};
    uint8_t          pulse_rate_khz    = 100;   // 10, 25, 50, or 100 kHz
    uint8_t          slice_interval_ms = 4;     // Default 4ms per motion slice
    GantrySlave      gantry_slave      = GantrySlave::None; // Dual-motor gantry: A motor follows X or Y
    bool             limit_override    = false; // Allow jogging off an active end-stop switch (buf[2]=0x40)
    uint8_t          estop_pin           = 0;     // 0 = disabled by default, 1..12 = IN1..IN12
    bool             estop_invert        = false;
    float            spindle_pwm_ratio   = 1.0f;  // Mill 0-10V AVI scaling
    double           spindle_ramp_up_s   = 0.0;   // Mill spindle spin-up ramp time (0..30s) before cutting starts
    double           spindle_ramp_down_s = 0.0;   // Mill spindle spin-down ramp time (0..30s)
    ThcConfig        thc{};
    LaserConfig      laser{};
    OperationMode    operation_mode    = OperationMode::Plasma;
    ProbeConfig      probe{};
    ToolChangeConfig tool_change{};
    ToolTableConfig  tool_table{};
    HuanyangVfdConfig vfd{};
    ScriptingConfig  scripting{};
    CameraConfig     camera{};

    // G-code interpretation: G4 P value unit (Mach3 "Dwell in ms" option). false = seconds.
    bool             dwell_p_milliseconds = false;

    // Runtime-only (not persisted): true for the BreakoutMill application. Disables plasma/laser-only
    // behaviour (THC, pierce delay override, laser output rules) and enables spindle direction (M4).
    bool             is_mill = false;

    // Work Coordinate Systems (G54 - G59, in mm)
    std::array<std::array<double, 4>, 6> workspace_offsets_mm{};
    uint8_t                              active_workspace = 0; // 0=G54..5=G59

    // Runtime-only: incremented by the controller whenever it changes work offsets itself (zeroing,
    // probing, G10/G92). applyConfig() only adopts a config's offsets when its revision is current, so a
    // stale getConfig() copy can never overwrite newer offsets.
    uint64_t                             workspace_revision = 0;
    // Runtime-only: same protection for the tool table (tool lengths measured by the controller)
    uint64_t                             tool_table_revision = 0;

    // Axis index (0 = X, 1 = Y) the A motor copies step-for-step, or -1 when A is an independent axis
    int gantryMasterAxis() const noexcept {
        switch (gantry_slave) {
            case GantrySlave::AFollowsX: return 0;
            case GantrySlave::AFollowsY: return 1;
            default:                     return -1;
        }
    }
};

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
    bool            usb_connected     = false;
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
    uint8_t  outputs_bitmask          = 0;     // OUT1..OUT4
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

// ============================================================================
// Toolpath Segment (Used by 2D/3D OpenGL Viewport to draw Cut vs. Uncut paths)
// ============================================================================
enum class SegmentType : uint8_t {
    RapidMove,  // G0 - Dashed travel line (Torch/Spindle OFF)
    CutLinear,  // G1 - Solid cut line (or tessellated G2/G3 chord)
    CutArcCW,   // G2 - Clockwise arc
    CutArcCCW,  // G3 - Counter-clockwise arc
    Dwell,      // G4 - Pierce delay / dwell
    ToolChange, // M6 / M06 - Mach3 Tool Change (Auto ATC or Manual Tool Change Hold)
    ProgramStop, // M0 / M1 - Program stop: motion halts in Feed Hold until Cycle Start
};

struct ToolpathSegment {
    SegmentType           type            = SegmentType::RapidMove;
    size_t                segment_index   = 0;
    size_t                gcode_line      = 0;
    std::array<double, 4> start_xyza_mm   = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> end_xyza_mm     = {0.0, 0.0, 0.0, 0.0};
    double                feed_mm_min     = 1500.0;
    double                dwell_seconds   = 0.0;
    double                length_mm       = 0.0;
    bool                  torch_active    = false; // True when M3 is active on this segment
    uint8_t               outputs_mask    = 0;     // Synchronized OUT1..OUT4 bitmask
    uint16_t              tool_number     = 1;     // Active or target tool number (T1..T99)
    uint16_t              spindle_rpm     = 0;     // Commanded spindle speed (S word)
    std::string           tool_description;        // Extracted comment from T/M6 line if present
    uint8_t               workspace_index = 0;     // 0=G54 .. 5=G59
    // Machine-coordinate targets (G53 / G28 / G30): bit a set => end axis a is the absolute machine
    // coordinate machine_target_mm[a]. end_xyza_mm holds the work-coordinate preview at parse time.
    uint8_t               machine_axes_mask = 0;
    std::array<double, 4> machine_target_mm = {0.0, 0.0, 0.0, 0.0};
    // Tool length offset in effect (G43 H<n>): 0 = none (G49), kTloInherit = not set by the program yet
    uint16_t              tlo_tool = kTloInherit;
};

// ============================================================================
// Abstract Motion Controller Interface (Implemented by RnRController)
// ============================================================================
class IMotionController {
public:
    virtual ~IMotionController() = default;

    // Connection, Simulator & Configuration
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual void setSimulatorMode(bool enabled) = 0;
    virtual void setDryRunMode(bool enabled) = 0;
    // Configuration changes are deferred until motion stops if the machine is moving.
    virtual void applyConfig(const MachineConfig& cfg) = 0;
    // Returns a consistent copy (thread-safe)
    virtual MachineConfig getConfig() const = 0;
    virtual TelemetrySnapshot getTelemetry() const = 0;

    // E-Stop, End-Stop Override & Simulated Inputs
    virtual void triggerEStop() = 0;
    virtual bool resetFromEStop() = 0;
    virtual void setLimitOverride(bool enabled) = 0;
    virtual void setSimulatedInputPin(uint8_t pin_1_to_12, bool active) = 0;

    // Coordinate Zeroing (Work Offsets & Machine Zero)
    virtual void zeroWorkAxis(core::Axis axis) = 0;
    virtual void zeroAllWorkAxes() = 0;
    virtual void goToWorkZeroXY() = 0;

    // Work Coordinate Systems (G54 - G59)
    virtual void selectWorkspace(uint8_t index) = 0;
    virtual void setWorkspaceOffset(uint8_t index, const std::array<double, 4>& offset_mm) = 0;
    virtual void setWorkspaceOffsetAxis(uint8_t index, core::Axis axis, double offset_mm) = 0;
    virtual uint8_t getActiveWorkspace() const = 0;
    virtual std::array<std::array<double, 4>, 6> getWorkspaceOffsets() const = 0;

    // Manual Jogging (Continuous & Incremental Step)
    virtual void jogStartContinuous(core::Axis axis, bool positive_dir, double speed_ratio) = 0;
    virtual void jogStep(core::Axis axis, double delta_mm, double feed_mm_min) = 0;
    virtual void jogStopAxis(core::Axis axis) = 0;
    virtual void jogStop() = 0;

    // Plasma Torch / Mill Spindle & Aux Outputs
    virtual void setTorchOrSpindle(bool enabled) = 0; // Toggles OUT1 immediately (Opcode 0x03)
    virtual void setAuxOutput(uint8_t out_index_1_to_4, bool enabled) = 0;
    virtual void setSpindlePwm(uint16_t duty) = 0;    // Mill 0-10V AVI (Opcode 0x0E)
    virtual void setSpindleSpeedRpm(uint32_t rpm) = 0; // Commanded spindle speed (updates PWM + Huanyang VFD)
    virtual void resetSpindleSpeedOverride() = 0;      // Restores G-Code programmatic RPM

    // Laser & Operation Mode Control
    virtual void setOperationMode(OperationMode mode) = 0;
    virtual OperationMode getOperationMode() const = 0;
    virtual void setLaserPowerOverride(float percent) = 0;
    virtual void testFireLaserPulse(float duration_ms, float power_pct) = 0;
    virtual void frameBoundingBox() = 0;
    virtual void setAirAssist(bool enabled) = 0;
    virtual void setRedPointer(bool enabled) = 0;

    // Huanyang VFD RS485 Inverter
    virtual void setHuanyangVfdConfig(const HuanyangVfdConfig& config) = 0;
    virtual HuanyangVfdConfig getHuanyangVfdConfig() const = 0;
    virtual HuanyangVfdTelemetry getHuanyangVfdTelemetry() const = 0;
    virtual void clearVfdAlarms() = 0;
    // Reads PD001/PD002/PD005/PD011/PD144 from the inverter; results arrive in the VFD telemetry params
    virtual void requestVfdParameterRead() = 0;

    // Homing & Probing (G31 Floating Head / Touch-Off / Edge & Center Finding)
    virtual void homeAxis(core::Axis axis) = 0;
    virtual void homeAllAxes() = 0;
    virtual void probeAxis(core::Axis axis, double max_travel_mm, double feed_mm_min) = 0;
    virtual void probeZWithToolSetter() = 0;
    virtual void probeWorkAxisEdge(core::Axis axis, bool positive_dir) = 0;
    virtual void probeHoleCenterXY(double approx_diameter_mm) = 0;
    // G38.2-style raw probe: move `signed_travel_mm` along `axis` until contact, then stop.
    // Records last_probed_work_pos_mm + last_probe_hit. Does NOT change offsets and does NOT retract.
    virtual void probeRaw(core::Axis axis, double signed_travel_mm, double feed_mm_min) = 0;

    // Tool table (ToolLengthMode::ToolTable)
    // Measure tool `tool` on the tool setter below the current XY position. The reference tool is
    // measured first after startup / Z homing; other tools get their length relative to it.
    virtual void measureToolLength(uint16_t tool) = 0;
    // The operator put `tool` in the spindle: sets the current tool and (tool table mode) applies its
    // length offset like "M6 T<tool> G43 H<tool>". 0 = no tool / cancel offset.
    virtual void selectToolInSpindle(uint16_t tool) = 0;
    // Apply the length offset of `tool` (G43 H<tool>) without changing the current tool; 0 = G49.
    virtual void activateToolLengthOffset(uint16_t tool) = 0;
    // G92-style: set the current work coordinate of `axis` to `value_mm` (adjusts the work offset).
    virtual void setWorkPosition(core::Axis axis, double value_mm) = 0;

    // Coordinate Persistence: restore saved machine positions and work offsets across application sessions
    virtual void restoreSavedPositions(
        const std::array<double, 4>& machine_pos_mm,
        const std::array<double, 4>& work_offset_mm) = 0;

    // Surface Level Probing & Height Map Warping
    virtual void startSurfaceProbing(double min_x, double max_x, double min_y, double max_y, int points_x, int points_y, double smoothness = 0.5) = 0;
    virtual void cancelSurfaceProbing() = 0;
    virtual void applySurfaceHeightMap(const SurfaceHeightMap& map) = 0;
    virtual void clearSurfaceHeightMap() = 0;
    virtual bool isSurfaceCompensationActive() const = 0;
    virtual SurfaceHeightMap getSurfaceHeightMap() const = 0;
    virtual bool getProgramBounds(std::array<double, 3>& out_min, std::array<double, 3>& out_max) const = 0;

    // G-Code Program, MDI & Job Execution
    virtual bool loadGCodeString(const std::string& gcode_text) = 0;
    virtual bool executeMdiCommand(const std::string& gcode_line) = 0;
    virtual void setNextGCodeLine(size_t line_index) = 0;
    // Immutable snapshot of the loaded program (segments, lines, bounds, diagnostics); never null.
    // Hold the returned pointer for as long as the data is used - a reload never invalidates it.
    virtual std::shared_ptr<const gcode::ParseResult> getProgram() const = 0;
    virtual void startJob() = 0;
    virtual void pauseJob() = 0;
    virtual void stopJob() = 0;
    virtual void setFeedOverride(float percent) = 0;
    virtual void setPierceDelayOverride(float seconds) = 0;

    // Scripting & Automation (Lua 5.4 Engine)
    virtual bool runScriptString(const std::string& code, bool async = true) = 0;
    virtual bool runScriptFile(const std::string& filepath, bool async = true) = 0;
    virtual void cancelScript() = 0;
    virtual bool isScriptRunning() const = 0;
    virtual void submitScriptPromptResponse(const std::string& response) = 0;
    // Input dialogs (cnc.input / cnc.input_number): button pressed + entered text
    virtual void submitScriptPromptInput(const std::string& button, const std::string& input) {
        (void)input;
        submitScriptPromptResponse(button);
    }
    virtual void setScriptLogCallback(std::function<void(const std::string&, bool)> cb) = 0;
};

} // namespace rnr::core
