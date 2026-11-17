#pragma once
// Machine settings (MachineConfig and its parts). Part of motion_controller.h, which includes it.
#include "core/board/board_types.h"
#include "core/board/board_profile.h"
#include "core/board/board_registry.h"
#include "core/spindle/huanyang_vfd.h"
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

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
    uint16_t         s_max                  = 1000;  // S value for 100 % power (1000 = Mach3 / LightBurn default, 255 = some GRBL setups)
    float            corner_power_floor_pct = 10.0f; // Minimum duty floor during corner deceleration
    bool             ignore_z_motion        = true;  // Lock focal plane (suppress G-code Z moves during cutting)
    bool             auto_air_assist        = true;  // Automatically energize OUT3 air solenoid on laser fire
    float            air_preflow_s          = 0.20f; // Delay before laser fires to pressurize air nozzle
    float            air_postflow_s         = 0.50f; // Delay after laser extinguishes to clear vapor and cool lens
    float            framing_power_pct      = 0.5f;  // Low-power beam percentage during framing (0.1% to 5.0%)
    double           framing_speed_mm_min   = 3000.0;
    float            pulse_duration_ms      = 50.0f; // Test pulse duration (10 to 500 ms)
    float            pulse_power_pct        = 10.0f; // Test pulse power percentage (1 to 50%)

    // Interlock inputs (0 = disabled, 1..12 = IN1..IN12; IN1..IN4 are the screw terminals on the RNR Eco Motion 2.0)
    uint8_t          chiller_flow_pin       = 2;     // Physical IN2 (0 = disabled, 1..12 = IN1..IN12)
    bool             chiller_flow_invert    = false; // Invert: checked = contact to GND indicates normal flow
    uint8_t          door_interlock_pin     = 3;     // Physical IN3 (0 = disabled, 1..12 = IN1..IN12)
    bool             door_interlock_invert  = false; // Invert: checked = contact to GND indicates door closed
    uint8_t          laser_fault_pin        = 4;     // Physical IN4 (0 = disabled, 1..12 = IN1..IN12)
    bool             laser_fault_invert     = false;

    // Optical Red-Dot Aiming Pointer Calibration Offset
    double           red_dot_offset_x_mm    = 0.0;
    double           red_dot_offset_y_mm    = 0.0;

    bool operator==(const LaserConfig&) const = default;
};

// Laser power of a program S word in percent (0..100). `s_given` = false: the program has no S word yet, the beam
// runs at full power (Mach3 behaviour). S0 is 0 % (beam off): CAM programs use it for overscan and white pixels.
inline double laserPowerPctForS(const LaserConfig& lz, bool s_given, uint16_t s) noexcept {
    if (!s_given) return 100.0;
    const double s_max = lz.s_max > 0 ? static_cast<double>(lz.s_max) : 1000.0;
    const double pct = static_cast<double>(s) * 100.0 / s_max;
    return pct < 0.0 ? 0.0 : (pct > 100.0 ? 100.0 : pct);
}
// PWM duty (0..100 %) for a laser power in percent: 0 % power is 0 duty (beam off), anything above starts at the
// striking threshold min_duty_pct and reaches max_duty_pct at 100 %
inline double laserDutyPctForPower(const LaserConfig& lz, double power_pct) noexcept {
    if (!(power_pct > 0.0)) return 0.0;
    const double p = power_pct > 100.0 ? 100.0 : power_pct;
    const double d = lz.min_duty_pct + (p / 100.0) * (lz.max_duty_pct - lz.min_duty_pct);
    return d < 0.0 ? 0.0 : (d > 100.0 ? 100.0 : d);
}

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

    bool operator==(const ThcConfig&) const = default;
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

    bool operator==(const ProbeConfig&) const = default;
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

    bool operator==(const ToolTableEntry&) const = default;
};

struct ToolTableConfig {
    ToolLengthMode                     mode           = ToolLengthMode::NotConfigured;
    uint16_t                           reference_tool = 1; // Measured first; defines length 0
    std::map<uint16_t, ToolTableEntry> tools;              // Tool number 1..255 -> entry

    bool operator==(const ToolTableConfig&) const = default;
};


struct ToolChangeConfig {
    ToolChangeMode mode              = ToolChangeMode::ManualPrompt;
    double         safe_z_mm         = 25.0;  // Z retract height for tool change
    bool           move_to_park_xy   = false; // Move to fixed XY tool-change position on M6
    double         park_x_mm         = 0.0;
    double         park_y_mm         = 0.0;
    double         atc_dwell_seconds = 2.5;   // Swap duration in AutoAtc mode

    bool operator==(const ToolChangeConfig&) const = default;
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

    bool operator==(const AxisConfig&) const = default;
};

// ============================================================================
// Scripting & Automation Configuration (Lua 5.4 Engine)
// ============================================================================
struct MacroButtonConfig {
    std::string label;
    std::string path;
    bool        show_on_control_panel = false;
    bool        has_color             = false; // false = the theme's default button color
    uint32_t    color_rgb             = 0;     // 0xRRGGBB, used when has_color

    bool operator==(const MacroButtonConfig&) const = default;
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

    bool operator==(const ScriptingConfig&) const = default;
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
    // Remote G-code upload is opt-in
    bool        stream_allow_upload     = false;
    // Access code the page asks for ("" = none: anyone who can reach the server may use it)
    std::string stream_access_code;
    // Listen address: "" = all networks, "127.0.0.1" = this PC only, or one of this PC's IPv4 addresses
    std::string stream_bind_address;

    bool operator==(const CameraConfig&) const = default;
};

// ============================================================================
// Input / Output Pin Assignment
// ============================================================================
// The machine logic drives four output channels (OutputBit bits 0..3: spindle / torch / laser enable, spindle
// direction or red pointer, flood coolant or air assist, mist coolant or exhaust fan). Each channel can be
// connected to any physical output OUT1..OUT4; a physical output without a channel is a custom output, switched
// by its own bit (kCustomOutputBit) from the I/O strip, the settings card or Lua cnc.set_output().
constexpr int     kOutputChannelCount = 4;
constexpr int     kPhysicalOutputCount = 4;
constexpr int     kInputCount = 12;
constexpr uint8_t kCustomOutputShift = 4; // Logical mask bits 4..7 = custom outputs on OUT1..OUT4

struct IoConfig {
    // Physical output (1..4 = OUT1..OUT4, 0 = not connected) of each output channel; the defaults are the
    // original fixed wiring. The spindle / torch / laser enable channel (0) is always connected.
    std::array<uint8_t, kOutputChannelCount> output_pin{1, 2, 3, 4};
    std::array<std::string, kPhysicalOutputCount> output_name{}; // Names of custom outputs
    std::array<std::string, kInputCount> input_name{};           // Names of custom (otherwise unused) inputs
    std::array<bool, kInputCount> input_invert{};                // Custom inputs: active when the input is off

    bool operator==(const IoConfig&) const = default;
};

// Dual-motor axis: the A motor output drives the second motor of X, Y or Z
enum class GantrySlave : uint8_t {
    None      = 0,
    AFollowsX = 1,
    AFollowsY = 2,
    AFollowsZ = 3,
};

struct MachineConfig {
    // Selected motion board (BoardRegistry id) and the settings of every board used so far ([Board.<id>])
    std::string      board_id = board::kRnrBoardId;
    std::map<std::string, board::BoardProfile> board_profiles;

    LinearUnits     linear_units      = LinearUnits::Metric;
    std::array<AxisConfig, 4> axes{};
    uint8_t          pulse_rate_khz    = 100;   // 25, 50 or 100 kHz, the choices of the RNR Mach3 plugin
    uint8_t          slice_interval_ms = 4;     // Default 4ms per motion slice
    GantrySlave      gantry_slave      = GantrySlave::None; // Dual-motor axis: A motor follows X, Y or Z
    bool             limit_override    = false; // Allow jogging off an active end-stop switch (buf[2]=0x40)
    uint8_t          estop_pin           = 0;     // 0 = disabled by default, 1..12 = IN1..IN12
    bool             estop_invert        = false;
    // Programs and MDI commands that move in machine coordinates (G53 / G28 / G30) need the axis homed first.
    // Off for machines without home switches: machine coordinates then continue from the position saved at the last exit.
    bool             require_homing      = true;
    // Program start: a first G0 that moves in XY lifts Z first (to start_safe_z_mm when it does not change Z
    // itself), so the tool is not dragged sideways at a low height. Off = the first rapid runs as written.
    // start_safe_z_mm (work Z of the tool tip) is also the lift height of Run From Line.
    bool             start_lift_to_safe_z = true;
    double           start_safe_z_mm      = 25.0;
    // Constant velocity motion. The tolerance sets the speed through corners; a larger value is faster and less
    // precise. G64 P overrides it inside a program. limit_curve_speed also slows down on curves made of many short
    // moves, to keep the sideways acceleration near the axis limits and the 25 ms slices close to the path.
    static constexpr double kMinCornerToleranceMm = 0.001;
    static constexpr double kMaxCornerToleranceMm = 5.0;
    double           corner_tolerance_mm  = 0.02;
    bool             limit_curve_speed    = true;
    // Sideways acceleration allowed on curves, in percent of the axis acceleration
    static constexpr int kMinCurveAccelPct = 25;
    static constexpr int kMaxCurveAccelPct = 200;
    int              curve_accel_pct      = 100;
    float            spindle_pwm_ratio  = 1.0f;  // Mill 0-10V AVI scaling
    double           spindle_ramp_up_s   = 0.0;   // Mill spindle spin-up ramp time (0..30s) before cutting starts
    double           spindle_ramp_down_s = 0.0;   // Mill spindle spin-down ramp time (0..30s)
    ThcConfig        thc{};
    LaserConfig      laser{};
    IoConfig         io{};
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

    // Step pulse rates the RNR board offers (its Mach3 plugin has the same three)
    static constexpr std::array<uint8_t, 3> kPulseRatesKhz = {25, 50, 100};

    // Rounds a pulse rate up to the next rate the board offers (anything above 50 or invalid gives 100)
    static constexpr uint8_t snapPulseRateKhz(int khz) noexcept {
        for (uint8_t r : kPulseRatesKhz) {
            if (khz > 0 && khz <= r) return r;
        }
        return 100;
    }

    // Axis index (0 = X, 1 = Y, 2 = Z) the A motor copies step-for-step, or -1 when A is an independent axis
    int gantryMasterAxis() const noexcept {
        switch (gantry_slave) {
            case GantrySlave::AFollowsX: return 0;
            case GantrySlave::AFollowsY: return 1;
            case GantrySlave::AFollowsZ: return 2;
            default:                     return -1;
        }
    }

    bool operator==(const MachineConfig&) const = default;
};

} // namespace rnr::core
