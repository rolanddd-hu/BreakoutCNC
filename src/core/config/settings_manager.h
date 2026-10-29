#pragma once
#include "core/controller/motion_controller.h"
#include <string>

namespace rnr::config {

struct AppUiSettings {
    int         window_width            = 1480;
    int         window_height           = 920;
    int         window_pos_x            = 80;
    int         window_pos_y            = 60;
    bool        window_maximized        = false;
    bool        window_fullscreen       = false;

    bool        simulator_mode          = false;
    bool        dry_run_mode            = false;
    bool        isometric_3d            = false;
    bool        show_rapids             = true;
    bool        show_pierces            = true;
    bool        follow_torch            = false;
    bool        minimal_layout          = false; // false = Full layout, true = Minimal (big viewport) layout

    bool        step_jog_mode           = false;
    float       step_size_mm            = 10.0f;
    float       jog_speed_pct           = 50.0f;
    float       dro_jog_height          = 0.0f; // 0.0f = auto / max height
    float       feed_override_pct       = 100.0f;
    float       pierce_delay_override_s = 0.0f;
    int         spindle_rpm             = 12000;
    std::string last_gcode_file_path;

    // Speed / Velocity configuration unit sub-index (0 = mm/m or IPM, 1 = mm/s or in/s)
    int         speed_unit_idx          = 0;

    // Operation mode (0=Plasma, 1=Laser)
    uint8_t     operation_mode           = 0;
    float       laser_power_override_pct = 100.0f;

    // Collapsible header states (persisted across sessions)
    bool        header_spindle_open        = true;
    bool        header_usb_open            = true;
    bool        header_vfd_open            = false;
    bool        header_toolchanger_open    = false;
    bool        header_tool_table_open     = false;

    // Tool in the spindle and active G43 tool when the application was closed (tool table mode)
    int         last_tool_in_spindle       = 0;
    int         last_tlo_tool              = 0;
    bool        header_probe_macro_open    = true;
    bool        header_probe_settings_open = false;
    bool        header_limits_open         = false;
    bool        header_motor_open          = false;
    bool        header_lua_open            = false;
    bool        header_workspaces_open     = true;
    bool        header_camera_open         = false;

    // Laser mode collapsible headers
    bool        header_laser_power_open      = true;
    bool        header_laser_alignment_open  = true;
    bool        header_laser_air_open        = true;
    bool        header_laser_interlocks_open = true;

    // Persisted machine and work coordinates across application sessions
    bool        has_saved_coordinates       = false;
    double      last_machine_pos_mm[4]      = {0.0, 0.0, 0.0, 0.0};
    double      last_work_offset_mm[4]      = {0.0, 0.0, 0.0, 0.0};
    double      last_work_pos_mm[4]         = {0.0, 0.0, 0.0, 0.0};
};

struct PersistentSettings {
    core::MachineConfig machine{};
    AppUiSettings       ui{};
};

class SettingsManager {
public:
    // Load settings from INI file (returns true if file existed and was loaded). A truncated file
    // (missing integrity marker) falls back to the "<file>.bak" copy of the previous save.
    static bool loadFromFile(const std::string& file_path, PersistentSettings& inout_settings);

    // Save all settings to INI file atomically (temp file + rename, previous file kept as .bak)
    static bool saveToFile(const std::string& file_path, const PersistentSettings& settings);

private:
    static bool loadFromFileImpl(const std::string& file_path, PersistentSettings& inout_settings, bool& complete);
};

} // namespace rnr::config
