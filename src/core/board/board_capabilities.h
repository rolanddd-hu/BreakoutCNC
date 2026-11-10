#pragma once
#include "core/board/io_model.h"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// What a board can do. The core, the UI, the program check and Lua read this; outside board code nothing asks
// which board is connected, only what it can do. See docs/architecture/MULTI_BOARD_PLAN.md section 4.4.
namespace rnr::board {

enum class Family : uint8_t {
    HostPlanned,     // The PC plans the motion and streams timed chunks (RNR Eco Motion)
    FirmwarePlanned, // The PC sends G-code lines, the board plans (GRBL, grblHAL, FluidNC)
};

enum class HomingKind : uint8_t {
    None,
    AppDriven,  // The app runs the homing moves and watches the switches
    BoardCycle, // The board runs its own homing cycle ($H)
};

enum class SettingsModel : uint8_t {
    AppFields,      // Settings live in the app (RNR)
    NumberedDollar, // GRBL 1.1 $$ list
    Enumerated,     // grblHAL: the controller describes its settings ($ES / $EG)
    YamlConfig,     // FluidNC config.yaml ($CD, $/path=value)
};

struct BoardCapabilities {
    std::string display_name;                // Shown in the UI, e.g. "RNR Eco Motion 2.0"
    Family      family = Family::HostPlanned;
    bool        accesses_real_hardware = true; // False for simulated / test drivers

    // ---- Host-planned boards (RNR terminal model) ----
    uint8_t     input_count      = 12;      // Inputs IN1..INn usable for switches, probe and E-Stop
    uint8_t     output_count     = 4;       // Outputs OUT1..OUTn
    uint8_t     min_pulse_khz    = 10;
    uint8_t     max_pulse_khz    = 100;
    size_t      max_slices_in_flight = 3;   // Unacknowledged slices the board can buffer
    bool        hardware_probe_latch = true;  // Board stops and latches positions on probe contact
    bool        hardware_home_latch  = true;  // Board stops and latches positions on a home switch

    // ---- Axes ----
    uint8_t     axis_count = 4;
    std::string axis_letters = "XYZA";
    bool        slaved_motor_option = true; // RNR: the A motor can follow X / Y / Z

    // ---- Motion ----
    bool   app_owns_motor_tuning = true;    // false: steps/mm, speeds, accelerations live in the firmware
    bool   realtime_process_control = true; // THC Z tracking, per-chunk laser power, Wait for Arc OK (done by the app)
    bool   board_thc = false;               // The board runs plasma torch height control and Arc OK itself
    std::string board_thc_mode;             // How it does it ("Voltage", "Up/down", "Arc ok")
    bool   native_arcs = false;
    bool   exact_stop_mode = true;          // G61 / G64 honoured
    bool   precise_line_tracking = true;    // false: the highlighted line is estimated from the position
    bool   can_set_machine_position = true;
    bool   jog_while_held = false;          // Jogging during a feed hold

    // ---- Homing, probing, limits ----
    HomingKind homing = HomingKind::AppDriven;
    bool   home_single_axis = true;
    bool   probe = true;
    bool   limit_override = true;           // Jog off an active end-stop
    bool   board_soft_limits = false;
    bool   needs_unlock = false;            // An alarm state that needs Unlock ($X)

    // ---- Overrides ----
    float  feed_override_min_pct = 10.0f, feed_override_max_pct = 250.0f;
    bool   rapid_override = false;
    bool   spindle_override = false;

    // ---- Spindle / outputs ----
    bool   spindle_pwm = true;              // 0-10 V / PWM spindle speed output
    bool   spindle_direction = true;
    bool   estop_input = true;
    bool   estop_keeps_position = true;     // false: a software E-stop while moving loses the position
    bool   mist_output = true;
    uint8_t aux_outputs = 0;                // Extra outputs (M62-M65 on firmware boards)

    // ---- Settings and console ----
    SettingsModel settings_model = SettingsModel::AppFields;
    bool   text_console = false;            // The board speaks a text protocol that can be shown / typed into

    std::vector<IoPoint> io;                // The board's terminals
};

} // namespace rnr::board
