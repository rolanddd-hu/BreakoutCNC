#include "core/config/settings_manager.h"
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <iomanip>
#include <algorithm>
#include <filesystem>
#include <cstring>
#include <functional>
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rnr::config {

namespace {
std::string trim(const std::string& s) {
    size_t a = 0;
    while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    size_t b = s.size();
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

bool toBool(const std::string& v) {
    return (v == "1" || v == "true" || v == "TRUE" || v == "yes");
}

// The BreakoutCut scripts folder was renamed from scripts/plasma to scripts/cut
std::string migrateScriptPath(const std::string& path) {
    for (const char* old_dir : {"scripts/plasma/", "scripts\\plasma\\"}) {
        const std::string prefix(old_dir);
        if (path.rfind(prefix, 0) == 0) return "scripts/cut/" + path.substr(prefix.size());
    }
    return path;
}

// Write to a temporary file, keep the previous file as .bak, then replace: a crash or power loss while
// saving can never leave a truncated settings file behind.
bool writeFileAtomically(const std::string& file_path, const std::string& content) {
    const std::string tmp_path = file_path + ".tmp";
    {
        std::ofstream ofs(tmp_path, std::ios::binary | std::ios::trunc);
        if (!ofs.is_open()) return false;
        ofs.write(content.data(), static_cast<std::streamsize>(content.size()));
        ofs.flush();
        if (!ofs.good()) return false;
    }
    std::error_code ec;
    if (std::filesystem::exists(file_path, ec)) {
        // A backup copied from a read-only file is read-only too: make it writable, or it is never updated again
        std::filesystem::permissions(file_path + ".bak", std::filesystem::perms::owner_write,
                                     std::filesystem::perm_options::add, ec);
        std::filesystem::copy_file(file_path, file_path + ".bak", std::filesystem::copy_options::overwrite_existing, ec);
    }
    std::filesystem::rename(tmp_path, file_path, ec);
    if (ec) {
        std::filesystem::remove(tmp_path, ec);
        return false;
    }
    return true;
}
} // namespace

bool SettingsManager::loadFromFile(const std::string& file_path, PersistentSettings& s) {
    PersistentSettings primary = s;
    bool complete = false;
    if (loadFromFileImpl(file_path, primary, complete) && complete) {
        s = primary;
        return true;
    }
    // Missing integrity marker (truncated save) or unreadable: fall back to the last good backup
    PersistentSettings backup = s;
    bool backup_complete = false;
    if (loadFromFileImpl(file_path + ".bak", backup, backup_complete) && backup_complete) {
        s = backup;
        return true;
    }
    std::error_code ec;
    if (std::filesystem::exists(file_path, ec)) {
        // Legacy file written before the integrity marker existed
        s = primary;
        return true;
    }
    return false;
}

// ============================================================================
// One table for the plain "key = value" settings: it drives both loading and saving, so a setting can never be
// written without being read back (or the other way round). Sections with lists (axes, I/O, tool table, macro
// buttons, work offsets, board profiles) and old key names are handled by code below the table.
// ============================================================================
namespace {

using Loader = std::function<void(PersistentSettings&, const std::string&)>;
using Saver  = std::function<void(std::ostream&, const PersistentSettings&)>;

struct Field {
    const char* section;
    const char* key;
    Loader      load;
    Saver       save;
};

constexpr double kNoLimit = std::numeric_limits<double>::infinity();

// A field bound through an accessor: [](auto& s) -> auto& { return s.ui.window_width; }. Numbers are limited to
// [lo, hi] when loaded (enums too); text can be transformed when loaded (old script folder names).
template <typename Acc>
Field field(const char* section, const char* key, Acc acc, double lo = -kNoLimit, double hi = kNoLimit,
            std::string (*transform)(const std::string&) = nullptr) {
    using T = std::remove_reference_t<decltype(acc(std::declval<PersistentSettings&>()))>;
    Field f{section, key, {}, {}};
    f.load = [acc, lo, hi, transform](PersistentSettings& s, const std::string& val) {
        auto& target = acc(s);
        if constexpr (std::is_same_v<T, bool>) {
            target = toBool(val);
        } else if constexpr (std::is_same_v<T, std::string>) {
            target = transform ? transform(val) : val;
        } else if constexpr (std::is_enum_v<T>) {
            target = static_cast<T>(static_cast<int>(std::clamp<double>(std::atoi(val.c_str()), lo, hi)));
        } else if constexpr (std::is_floating_point_v<T>) {
            target = static_cast<T>(std::clamp(std::atof(val.c_str()), lo, hi));
        } else {
            // Integers: limited when a range is given, otherwise converted as before (std::atol, then the type)
            const long long v = std::atoll(val.c_str());
            target = (lo == -kNoLimit && hi == kNoLimit)
                         ? static_cast<T>(v)
                         : static_cast<T>(static_cast<long long>(std::clamp<double>(static_cast<double>(v), lo, hi)));
        }
    };
    f.save = [acc](std::ostream& os, const PersistentSettings& s) {
        const auto& v = acc(s);
        if constexpr (std::is_same_v<T, bool>) {
            os << (v ? 1 : 0);
        } else if constexpr (std::is_enum_v<T>) {
            os << static_cast<int>(v);
        } else if constexpr (std::is_same_v<T, uint8_t>) {
            os << static_cast<int>(v);
        } else {
            os << v;
        }
    };
    return f;
}

// The same for one axis ([Axis_0] .. [Axis_3])
using AxisLoader = std::function<void(core::AxisConfig&, const std::string&)>;
using AxisSaver  = std::function<void(std::ostream&, const core::AxisConfig&)>;
struct AxisField {
    const char* key;
    AxisLoader  load;
    AxisSaver   save;
};

template <typename Acc>
AxisField axisField(const char* key, Acc acc) {
    using T = std::remove_reference_t<decltype(acc(std::declval<core::AxisConfig&>()))>;
    AxisField f{key, {}, {}};
    f.load = [acc](core::AxisConfig& ax, const std::string& val) {
        auto& target = acc(ax);
        if constexpr (std::is_same_v<T, bool>) {
            target = toBool(val);
        } else if constexpr (std::is_floating_point_v<T>) {
            target = static_cast<T>(std::atof(val.c_str()));
        } else {
            target = static_cast<T>(std::atoi(val.c_str()));
        }
    };
    f.save = [acc](std::ostream& os, const core::AxisConfig& ax) {
        const auto& v = acc(ax);
        if constexpr (std::is_same_v<T, bool>) {
            os << (v ? 1 : 0);
        } else if constexpr (std::is_same_v<T, uint8_t>) {
            os << static_cast<int>(v);
        } else {
            os << v;
        }
    };
    return f;
}

#define UI_(name) [](auto& s) -> auto& { return s.ui.name; }
#define MC_(name) [](auto& s) -> auto& { return s.machine.name; }

const std::vector<Field>& fields() {
    static const std::vector<Field> table = {
        // [Window]
        field("Window", "width", UI_(window_width), 800),
        field("Window", "height", UI_(window_height), 600),
        field("Window", "pos_x", UI_(window_pos_x)),
        field("Window", "pos_y", UI_(window_pos_y)),
        field("Window", "maximized", UI_(window_maximized)),
        field("Window", "fullscreen", UI_(window_fullscreen)),
        // [UI]
        field("UI", "simulator_mode", UI_(simulator_mode)),
        field("UI", "dry_run_mode", UI_(dry_run_mode)),
        field("UI", "isometric_3d", UI_(isometric_3d)),
        field("UI", "show_rapids", UI_(show_rapids)),
        field("UI", "toolpath_simplify_px", UI_(toolpath_simplify_px)),
        field("UI", "show_pierces", UI_(show_pierces)),
        field("UI", "follow_torch", UI_(follow_torch)),
        field("UI", "minimal_layout", UI_(minimal_layout)),
        field("UI", "light_theme", UI_(light_theme)),
        field("UI", "dark_theme_id", UI_(dark_theme_id)),
        field("UI", "light_theme_id", UI_(light_theme_id)),
        field("UI", "step_jog_mode", UI_(step_jog_mode)),
        field("UI", "step_size_mm", UI_(step_size_mm)),
        field("UI", "jog_speed_pct", UI_(jog_speed_pct)),
        field("UI", "dro_jog_height", UI_(dro_jog_height)),
        field("UI", "feed_override_pct", UI_(feed_override_pct)),
        field("UI", "pierce_delay_override_s", UI_(pierce_delay_override_s)),
        field("UI", "spindle_rpm", UI_(spindle_rpm)),
        field("UI", "last_gcode_file_path", UI_(last_gcode_file_path)),
        field("UI", "header_spindle_open", UI_(header_spindle_open)),
        field("UI", "header_usb_open", UI_(header_usb_open)),
        field("UI", "header_vfd_open", UI_(header_vfd_open)),
        field("UI", "header_toolchanger_open", UI_(header_toolchanger_open)),
        field("UI", "header_tool_table_open", UI_(header_tool_table_open)),
        field("UI", "last_tool_in_spindle", UI_(last_tool_in_spindle), 0, 255),
        field("UI", "last_tlo_tool", UI_(last_tlo_tool), 0, 255),
        field("UI", "header_probe_macro_open", UI_(header_probe_macro_open)),
        field("UI", "header_probe_settings_open", UI_(header_probe_settings_open)),
        field("UI", "header_limits_open", UI_(header_limits_open)),
        field("UI", "header_io_open", UI_(header_io_open)),
        field("UI", "header_motor_open", UI_(header_motor_open)),
        field("UI", "header_lua_open", UI_(header_lua_open)),
        field("UI", "header_workspaces_open", UI_(header_workspaces_open)),
        field("UI", "header_camera_open", UI_(header_camera_open)),
        field("UI", "operation_mode", UI_(operation_mode), 0, 1), // Also sets the machine's mode (see load)
        field("UI", "laser_power_override_pct", UI_(laser_power_override_pct)),
        field("UI", "header_laser_power_open", UI_(header_laser_power_open)),
        field("UI", "header_laser_alignment_open", UI_(header_laser_alignment_open)),
        field("UI", "header_laser_air_open", UI_(header_laser_air_open)),
        field("UI", "header_laser_interlocks_open", UI_(header_laser_interlocks_open)),
        field("UI", "speed_unit_idx", UI_(speed_unit_idx), 0, 1),
        field("UI", "has_saved_coordinates", UI_(has_saved_coordinates)),
        field("UI", "last_machine_x", UI_(last_machine_pos_mm[0])),
        field("UI", "last_machine_y", UI_(last_machine_pos_mm[1])),
        field("UI", "last_machine_z", UI_(last_machine_pos_mm[2])),
        field("UI", "last_machine_a", UI_(last_machine_pos_mm[3])),
        field("UI", "last_work_offset_x", UI_(last_work_offset_mm[0])),
        field("UI", "last_work_offset_y", UI_(last_work_offset_mm[1])),
        field("UI", "last_work_offset_z", UI_(last_work_offset_mm[2])),
        field("UI", "last_work_offset_a", UI_(last_work_offset_mm[3])),
        field("UI", "last_work_x", UI_(last_work_pos_mm[0])),
        field("UI", "last_work_y", UI_(last_work_pos_mm[1])),
        field("UI", "last_work_z", UI_(last_work_pos_mm[2])),
        field("UI", "last_work_a", UI_(last_work_pos_mm[3])),
        // [Machine] (limit_override is deliberately not saved: end-stops are always active after a restart)
        field("Machine", "board", MC_(board_id)),
        field("Machine", "linear_units", MC_(linear_units), 0, 1),
        field("Machine", "pulse_rate_khz", MC_(pulse_rate_khz)),
        field("Machine", "gantry_slave", MC_(gantry_slave), 0, 3),
        field("Machine", "dwell_p_milliseconds", MC_(dwell_p_milliseconds)),
        field("Machine", "estop_pin", MC_(estop_pin)),
        field("Machine", "estop_invert", MC_(estop_invert)),
        field("Machine", "require_homing", MC_(require_homing)),
        field("Machine", "start_lift_to_safe_z", MC_(start_lift_to_safe_z)),
        field("Machine", "start_safe_z_mm", MC_(start_safe_z_mm)),
        field("Machine", "corner_tolerance_mm", MC_(corner_tolerance_mm)),
        field("Machine", "limit_curve_speed", MC_(limit_curve_speed)),
        field("Machine", "curve_accel_pct", MC_(curve_accel_pct)),
        field("Machine", "spindle_pwm_ratio", MC_(spindle_pwm_ratio)),
        field("Machine", "spindle_ramp_up_s", MC_(spindle_ramp_up_s), 0.0),
        field("Machine", "spindle_ramp_down_s", MC_(spindle_ramp_down_s), 0.0),
        // [THC]
        field("THC", "enabled", MC_(thc.enabled)),
        field("THC", "wait_for_arc_ok", MC_(thc.wait_for_arc_ok)),
        field("THC", "arc_ok_pin", MC_(thc.arc_ok_pin)),
        field("THC", "thc_up_pin", MC_(thc.thc_up_pin)),
        field("THC", "thc_down_pin", MC_(thc.thc_down_pin)),
        field("THC", "arc_ok_invert", MC_(thc.arc_ok_invert)),
        field("THC", "thc_up_invert", MC_(thc.thc_up_invert)),
        field("THC", "thc_down_invert", MC_(thc.thc_down_invert)),
        field("THC", "adjust_speed_mm_s", MC_(thc.adjust_speed_mm_s)),
        field("THC", "min_xy_feed_pct", MC_(thc.min_xy_feed_pct)),
        field("THC", "delay_after_pierce_s", MC_(thc.delay_after_pierce_s)),
        field("THC", "z_min_mm", MC_(thc.z_min_mm)),
        field("THC", "z_max_mm", MC_(thc.z_max_mm)),
        // [Laser]
        field("Laser", "enabled", MC_(laser.enabled)),
        field("Laser", "source_type", MC_(laser.source_type), 0, 4),
        field("Laser", "framing_mode", MC_(laser.framing_mode), 0, 3),
        field("Laser", "dynamic_power_m4", MC_(laser.dynamic_power_m4)),
        field("Laser", "pwm_frequency_khz", MC_(laser.pwm_frequency_khz)),
        field("Laser", "min_duty_pct", MC_(laser.min_duty_pct)),
        field("Laser", "max_duty_pct", MC_(laser.max_duty_pct)),
        field("Laser", "s_max", MC_(laser.s_max), 1, 60000),
        field("Laser", "corner_power_floor_pct", MC_(laser.corner_power_floor_pct)),
        field("Laser", "ignore_z_motion", MC_(laser.ignore_z_motion)),
        field("Laser", "auto_air_assist", MC_(laser.auto_air_assist)),
        field("Laser", "air_preflow_s", MC_(laser.air_preflow_s)),
        field("Laser", "air_postflow_s", MC_(laser.air_postflow_s)),
        field("Laser", "framing_power_pct", MC_(laser.framing_power_pct)),
        field("Laser", "framing_speed_mm_min", MC_(laser.framing_speed_mm_min)),
        field("Laser", "pulse_duration_ms", MC_(laser.pulse_duration_ms)),
        field("Laser", "pulse_power_pct", MC_(laser.pulse_power_pct)),
        field("Laser", "chiller_flow_pin", MC_(laser.chiller_flow_pin)),
        field("Laser", "chiller_flow_invert", MC_(laser.chiller_flow_invert)),
        field("Laser", "door_interlock_pin", MC_(laser.door_interlock_pin)),
        field("Laser", "door_interlock_invert", MC_(laser.door_interlock_invert)),
        field("Laser", "laser_fault_pin", MC_(laser.laser_fault_pin)),
        field("Laser", "laser_fault_invert", MC_(laser.laser_fault_invert)),
        field("Laser", "red_dot_offset_x_mm", MC_(laser.red_dot_offset_x_mm)),
        field("Laser", "red_dot_offset_y_mm", MC_(laser.red_dot_offset_y_mm)),
        // [Probe]
        field("Probe", "probe_pin", MC_(probe.probe_pin)),
        field("Probe", "probe_invert", MC_(probe.probe_invert)),
        field("Probe", "plate_thickness_mm", MC_(probe.plate_thickness_mm)),
        field("Probe", "tool_diameter_mm", MC_(probe.tool_diameter_mm)),
        field("Probe", "fast_feed_mm_min", MC_(probe.fast_feed_mm_min)),
        field("Probe", "slow_feed_mm_min", MC_(probe.slow_feed_mm_min)),
        field("Probe", "max_travel_mm", MC_(probe.max_travel_mm)),
        field("Probe", "retract_mm", MC_(probe.retract_mm)),
        field("Probe", "approx_hole_diam_mm", MC_(probe.approx_hole_diam_mm)),
        field("Probe", "xy_clearance_mm", MC_(probe.xy_clearance_mm)),
        field("Probe", "probe_depth_mm", MC_(probe.probe_depth_mm)),
        field("Probe", "approx_size_x_mm", MC_(probe.approx_size_x_mm)),
        field("Probe", "approx_size_y_mm", MC_(probe.approx_size_y_mm)),
        field("Probe", "surface_max_depth_below_zero_mm", MC_(probe.surface_max_depth_below_zero_mm)),
        field("Probe", "surface_clearance_z_mm", MC_(probe.surface_clearance_z_mm)),
        field("Probe", "surface_smoothness", MC_(probe.surface_smoothness), 0.0, 1.0),
        field("Probe", "surface_max_points", MC_(probe.surface_max_points), 5, 500),
        // [ToolChange]
        field("ToolChange", "mode", MC_(tool_change.mode), 0, 2),
        field("ToolChange", "safe_z_mm", MC_(tool_change.safe_z_mm)),
        field("ToolChange", "move_to_park_xy", MC_(tool_change.move_to_park_xy)),
        field("ToolChange", "park_x_mm", MC_(tool_change.park_x_mm)),
        field("ToolChange", "park_y_mm", MC_(tool_change.park_y_mm)),
        field("ToolChange", "atc_dwell_seconds", MC_(tool_change.atc_dwell_seconds)),
        // [ToolTable] (the tools themselves: see below)
        field("ToolTable", "mode", MC_(tool_table.mode), 0, 2),
        field("ToolTable", "reference_tool", MC_(tool_table.reference_tool), 1, 255),
        // [HuanyangVFD]
        field("HuanyangVFD", "enabled", MC_(vfd.enabled)),
        field("HuanyangVFD", "port_name", MC_(vfd.port_name)),
        field("HuanyangVFD", "baud_rate", MC_(vfd.baud_rate)),
        field("HuanyangVFD", "slave_address", MC_(vfd.slave_address)),
        field("HuanyangVFD", "max_rpm", MC_(vfd.max_rpm)),
        field("HuanyangVFD", "max_freq_hz", MC_(vfd.max_freq_hz)),
        field("HuanyangVFD", "min_rpm", MC_(vfd.min_rpm)),
        field("HuanyangVFD", "estop_on_comm_loss", MC_(vfd.estop_on_comm_loss)),
        field("HuanyangVFD", "estop_on_stall", MC_(vfd.estop_on_stall)),
        field("HuanyangVFD", "speed_monitor_enabled", MC_(vfd.speed_monitor_enabled)),
        field("HuanyangVFD", "speed_margin_pct", MC_(vfd.speed_margin_pct), 1, 50),
        field("HuanyangVFD", "bad_speed_threshold", MC_(vfd.bad_speed_threshold), 1, 255),
        field("HuanyangVFD", "tracking_threshold", MC_(vfd.tracking_threshold), 1, 255),
        field("HuanyangVFD", "limit_out_freq_hz", MC_(vfd.limit_out_freq_hz)),
        field("HuanyangVFD", "limit_current_a", MC_(vfd.limit_current_a)),
        field("HuanyangVFD", "limit_rpm", MC_(vfd.limit_rpm)),
        field("HuanyangVFD", "limit_dc_volts", MC_(vfd.limit_dc_volts)),
        field("HuanyangVFD", "limit_ac_volts", MC_(vfd.limit_ac_volts)),
        field("HuanyangVFD", "limit_temp_c", MC_(vfd.limit_temp_c)),
        // [Scripting] (the macro buttons: see below)
        field("Scripting", "enabled", MC_(scripting.enabled)),
        field("Scripting", "scripts_dir", MC_(scripting.scripts_dir)),
        field("Scripting", "m6_script_enabled", MC_(scripting.m6_script_enabled)),
        field("Scripting", "m6_script_path", MC_(scripting.m6_script_path), -kNoLimit, kNoLimit, migrateScriptPath),
        field("Scripting", "home_script_enabled", MC_(scripting.home_script_enabled)),
        field("Scripting", "home_script_path", MC_(scripting.home_script_path), -kNoLimit, kNoLimit, migrateScriptPath),
        field("Scripting", "ihs_script_enabled", MC_(scripting.ihs_script_enabled)),
        field("Scripting", "ihs_script_path", MC_(scripting.ihs_script_path), -kNoLimit, kNoLimit, migrateScriptPath),
        // [Workspaces] (the offsets: see below)
        field("Workspaces", "active_workspace", MC_(active_workspace), 0, 5),
        // [Camera]
        field("Camera", "top_down_camera_index", MC_(camera.top_down_camera_index)),
        field("Camera", "top_down_camera_name", MC_(camera.top_down_camera_name)),
        field("Camera", "monitoring_camera_index", MC_(camera.monitoring_camera_index)),
        field("Camera", "monitoring_camera_name", MC_(camera.monitoring_camera_name)),
        field("Camera", "is_calibrated", MC_(camera.is_calibrated)),
        field("Camera", "offset_x_mm", MC_(camera.offset_x_mm)),
        field("Camera", "offset_y_mm", MC_(camera.offset_y_mm)),
        field("Camera", "fov_x_mm", MC_(camera.fov_x_mm), 1.0),
        field("Camera", "fov_y_mm", MC_(camera.fov_y_mm), 1.0),
        field("Camera", "cal_move_x_mm", MC_(camera.cal_move_x_mm), 0.5),
        field("Camera", "overlay_opacity", MC_(camera.overlay_opacity), 0.1, 1.0),
        field("Camera", "mirror_x", MC_(camera.mirror_x)),
        field("Camera", "mirror_y", MC_(camera.mirror_y)),
        field("Camera", "stream_enabled", MC_(camera.stream_enabled)),
        field("Camera", "stream_port", MC_(camera.stream_port), 1024, 65535),
        field("Camera", "stream_fps", MC_(camera.stream_fps), 1, 60),
        field("Camera", "stream_allow_upload", MC_(camera.stream_allow_upload)),
        field("Camera", "stream_access_code", MC_(camera.stream_access_code)),
        field("Camera", "stream_bind_address", MC_(camera.stream_bind_address)),
    };
    return table;
}

#define AX_(name) [](auto& ax) -> auto& { return ax.name; }
const std::vector<AxisField>& axisFields() {
    static const std::vector<AxisField> table = {
        axisField("steps_per_mm", AX_(steps_per_mm)),
        axisField("max_velocity_mm_s", AX_(max_velocity_mm_s)),
        axisField("acceleration_mm_s2", AX_(acceleration_mm_s2)),
        axisField("deceleration_mm_s2", AX_(deceleration_mm_s2)),
        axisField("home_pulloff_mm", AX_(home_pulloff_mm)),
        axisField("homing_feed_mm_min", AX_(homing_feed_mm_min)),
        axisField("homing_max_travel_mm", AX_(homing_max_travel_mm)),
        axisField("homing_dir_negative", AX_(homing_dir_negative)),
        axisField("step_active_low", AX_(step_active_low)),
        axisField("dir_active_low", AX_(dir_active_low)),
        axisField("limit_plus_pin", AX_(limit_plus_pin)),
        axisField("limit_minus_pin", AX_(limit_minus_pin)),
        axisField("home_pin", AX_(home_pin)),
        axisField("limit_plus_invert", AX_(limit_plus_invert)),
        axisField("limit_minus_invert", AX_(limit_minus_invert)),
        axisField("home_invert", AX_(home_invert)),
        axisField("soft_limits_enabled", AX_(soft_limits_enabled)),
        axisField("soft_limit_min_mm", AX_(soft_limit_min_mm)),
        axisField("soft_limit_max_mm", AX_(soft_limit_max_mm)),
    };
    return table;
}
#undef UI_
#undef MC_
#undef AX_

// Section order in the file (sections with list entries add them after their table fields)
constexpr const char* kSectionOrder[] = {"Window", "UI", "Machine", "IO", "THC", "Laser", "Probe", "ToolChange",
                                         "ToolTable", "HuanyangVFD", "Scripting", "Workspaces", "Camera"};

// ---- List entries and old key names (load) ----
void loadIoEntry(core::IoConfig& io, const std::string& key, const std::string& val) {
    // Output channel pins (out_channel_0_pin ..), custom output / input names, custom input invert
    auto indexed = [&](const char* prefix, const char* suffix, int count) {
        const std::string p = prefix, sfx = suffix;
        if (key.size() <= p.size() + sfx.size() || key.compare(0, p.size(), p) != 0 ||
            key.compare(key.size() - sfx.size(), sfx.size(), sfx) != 0) {
            return -1;
        }
        const int i = std::atoi(key.substr(p.size(), key.size() - p.size() - sfx.size()).c_str());
        return (i >= 0 && i < count) ? i : -1;
    };
    int i = -1;
    if ((i = indexed("out_channel_", "_pin", core::kOutputChannelCount)) >= 0) {
        io.output_pin[i] = static_cast<uint8_t>(std::clamp(std::atoi(val.c_str()), 0, core::kPhysicalOutputCount));
    } else if ((i = indexed("out_", "_name", core::kPhysicalOutputCount)) >= 0) {
        io.output_name[i] = val;
    } else if ((i = indexed("in_", "_name", core::kInputCount)) >= 0) {
        io.input_name[i] = val;
    } else if ((i = indexed("in_", "_invert", core::kInputCount)) >= 0) {
        io.input_invert[i] = toBool(val);
    }
}

void loadToolEntry(core::ToolTableConfig& tt, const std::string& key, const std::string& val) {
    if (key.rfind("tool_", 0) != 0) return;
    // tool_<n> = <length_mm>;<diameter_mm>;<description>
    const int num = std::atoi(key.c_str() + 5);
    if (num < 1 || num > 255) return;
    core::ToolTableEntry e;
    const size_t p1 = val.find(';');
    const size_t p2 = (p1 == std::string::npos) ? std::string::npos : val.find(';', p1 + 1);
    e.length_mm = std::atof(val.substr(0, p1).c_str());
    if (p1 != std::string::npos) e.diameter_mm = std::atof(val.substr(p1 + 1, p2 - p1 - 1).c_str());
    if (p2 != std::string::npos) e.description = trim(val.substr(p2 + 1));
    tt.tools[static_cast<uint16_t>(num)] = e;
}

void loadMacroEntry(core::ScriptingConfig& sc, const std::string& key, const std::string& val, bool& has_count) {
    if (key == "macro_button_count") {
        has_count = true;
        const int count = std::clamp(std::atoi(val.c_str()), 0, 50);
        sc.macro_buttons.assign(static_cast<size_t>(count), core::MacroButtonConfig{});
        return;
    }
    if (key.rfind("macro_btn_", 0) != 0) return;
    const size_t next_us = key.find('_', 10);
    if (next_us == std::string::npos) return;
    const int raw_idx = std::atoi(key.substr(10, next_us - 10).c_str());
    const int idx = has_count ? raw_idx : (raw_idx - 1); // Old files numbered the buttons from 1
    const std::string suffix = key.substr(next_us + 1);
    if (idx < 0 || idx >= 50) return;
    if (static_cast<size_t>(idx) >= sc.macro_buttons.size()) sc.macro_buttons.resize(idx + 1);
    auto& b = sc.macro_buttons[idx];
    if (suffix == "label") {
        b.label = val;
    } else if (suffix == "path") {
        b.path = migrateScriptPath(val);
    } else if (suffix == "control_panel") {
        b.show_on_control_panel = toBool(val);
    } else if (suffix == "color") {
        // "#RRGGBB"; anything else (empty) keeps the default color
        if (val.size() == 7 && val[0] == '#' && val.find_first_not_of("0123456789abcdefABCDEF", 1) == std::string::npos) {
            b.has_color = true;
            b.color_rgb = static_cast<uint32_t>(std::strtoul(val.c_str() + 1, nullptr, 16));
        }
    }
}

void loadWorkspaceEntry(core::MachineConfig& m, const std::string& key, const std::string& val) {
    for (int w = 0; w < 6; ++w) {
        const std::string prefix = "g" + std::to_string(54 + w) + "_";
        if (key.rfind(prefix, 0) != 0 || key.size() != prefix.size() + 1) continue;
        const char ax = key[prefix.size()];
        const double d = std::atof(val.c_str());
        if (ax == 'x') m.workspace_offsets_mm[w][0] = d;
        else if (ax == 'y') m.workspace_offsets_mm[w][1] = d;
        else if (ax == 'z') m.workspace_offsets_mm[w][2] = d;
        else if (ax == 'a') m.workspace_offsets_mm[w][3] = d;
    }
}

// ---- List entries (save) ----
void saveSectionExtras(std::ostream& ofs, const std::string& section, const PersistentSettings& s) {
    if (section == "IO") {
        const auto& io = s.machine.io;
        for (int i = 0; i < core::kOutputChannelCount; ++i) {
            ofs << "out_channel_" << i << "_pin = " << static_cast<int>(io.output_pin[i]) << "\n";
        }
        for (int i = 0; i < core::kPhysicalOutputCount; ++i) {
            if (!io.output_name[i].empty()) ofs << "out_" << i << "_name = " << io.output_name[i] << "\n";
        }
        for (int i = 0; i < core::kInputCount; ++i) {
            if (!io.input_name[i].empty()) ofs << "in_" << i << "_name = " << io.input_name[i] << "\n";
            if (io.input_invert[i]) ofs << "in_" << i << "_invert = 1\n";
        }
    } else if (section == "ToolTable") {
        ofs << "; mode: 0 = not configured (G43 ignored, warning), 1 = touch off every tool, 2 = tool table\n";
        ofs << "; tool_<n> = <length_mm relative to reference tool>;<diameter_mm>;<description>\n";
        std::ostringstream lens;
        lens << std::fixed << std::setprecision(4);
        for (const auto& [num, e] : s.machine.tool_table.tools) {
            lens.str("");
            lens << e.length_mm << ";" << e.diameter_mm << ";";
            ofs << "tool_" << num << " = " << lens.str() << e.description << "\n";
        }
    } else if (section == "Scripting") {
        const auto& sc = s.machine.scripting;
        ofs << "macro_button_count = " << sc.macro_buttons.size() << "\n";
        for (size_t i = 0; i < sc.macro_buttons.size(); ++i) {
            ofs << "macro_btn_" << i << "_label = " << sc.macro_buttons[i].label << "\n";
            ofs << "macro_btn_" << i << "_path = " << sc.macro_buttons[i].path << "\n";
            ofs << "macro_btn_" << i << "_control_panel = " << (sc.macro_buttons[i].show_on_control_panel ? 1 : 0) << "\n";
            if (sc.macro_buttons[i].has_color) {
                char hex[8];
                std::snprintf(hex, sizeof(hex), "#%06X", static_cast<unsigned>(sc.macro_buttons[i].color_rgb & 0xFFFFFFu));
                ofs << "macro_btn_" << i << "_color = " << hex << "\n";
            }
        }
    } else if (section == "Workspaces") {
        for (int w = 0; w < 6; ++w) {
            const int g_num = 54 + w;
            ofs << "g" << g_num << "_x = " << s.machine.workspace_offsets_mm[w][0] << "\n";
            ofs << "g" << g_num << "_y = " << s.machine.workspace_offsets_mm[w][1] << "\n";
            ofs << "g" << g_num << "_z = " << s.machine.workspace_offsets_mm[w][2] << "\n";
            ofs << "g" << g_num << "_a = " << s.machine.workspace_offsets_mm[w][3] << "\n";
        }
    }
}

} // namespace

bool SettingsManager::loadFromFileImpl(const std::string& file_path, PersistentSettings& s, bool& complete) {
    complete = false;
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        return false;
    }

    // Lookup: "Section\nkey" -> field
    static const std::unordered_map<std::string, const Field*> index = [] {
        std::unordered_map<std::string, const Field*> m;
        for (const auto& f : fields()) m[std::string(f.section) + "\n" + f.key] = &f;
        return m;
    }();
    static const std::unordered_map<std::string, const AxisField*> axis_index = [] {
        std::unordered_map<std::string, const AxisField*> m;
        for (const auto& f : axisFields()) m[f.key] = &f;
        return m;
    }();

    std::string section;
    std::string line;
    bool has_macro_button_count = false;
    bool has_start_safe_z = false;
    while (std::getline(ifs, line)) {
        const std::string t = trim(line);
        if (t.empty() || t[0] == ';' || t[0] == '#') continue;
        if (t.front() == '[' && t.back() == ']') {
            section = trim(t.substr(1, t.size() - 2));
            continue;
        }
        const size_t eq = t.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(t.substr(0, eq));
        const std::string val = trim(t.substr(eq + 1));

        if (section == "Integrity") {
            if (key == "complete" && toBool(val)) complete = true;
            continue;
        }
        if (section.rfind("Board.", 0) == 0 && section.size() > 6) {
            // [Board.<id>]: kept even for a board this build does not know
            s.machine.board_profiles[section.substr(6)].set(key, val);
            continue;
        }
        if (section.rfind("Axis_", 0) == 0 && section.size() == 6) {
            const int idx = section[5] - '0';
            const auto it = axis_index.find(key);
            if (idx >= 0 && idx < 4 && it != axis_index.end()) it->second->load(s.machine.axes[static_cast<size_t>(idx)], val);
            continue;
        }
        if (const auto it = index.find(section + "\n" + key); it != index.end()) {
            it->second->load(s, val);
            // Keys with an effect beyond their own field
            if (section == "UI" && key == "operation_mode") {
                s.machine.operation_mode = static_cast<core::OperationMode>(s.ui.operation_mode);
            } else if (section == "Machine" && key == "board" && s.machine.board_id.empty()) {
                s.machine.board_id = board::kRnrBoardId;
            } else if (section == "Machine" && key == "start_safe_z_mm") {
                has_start_safe_z = true;
            }
            continue;
        }
        // List entries and old key names
        if (section == "Machine" && key == "slave_a_to_x") {
            if (toBool(val)) s.machine.gantry_slave = core::GantrySlave::AFollowsX; // Older files only knew A-follows-X
        } else if (section == "IO") {
            loadIoEntry(s.machine.io, key, val);
        } else if (section == "ToolTable") {
            loadToolEntry(s.machine.tool_table, key, val);
        } else if (section == "Scripting") {
            loadMacroEntry(s.machine.scripting, key, val, has_macro_button_count);
        } else if (section == "Workspaces") {
            loadWorkspaceEntry(s.machine, key, val);
        }
    }

    // Older settings files lifted to the M6 safe Z at program start: keep that height
    if (!has_start_safe_z) s.machine.start_safe_z_mm = s.machine.tool_change.safe_z_mm;

    return true;
}

bool SettingsManager::saveToFile(const std::string& file_path, const PersistentSettings& s) {
    std::ostringstream ofs;

    ofs << std::fixed << std::setprecision(3);
    ofs << "; ====================================================================\n";
    ofs << "; BreakoutCNC Persistent Configuration File\n";
    ofs << "; Automatically loaded on startup and saved on exit.\n";
    ofs << "; ====================================================================\n\n";

    for (const char* section : kSectionOrder) {
        ofs << "[" << section << "]\n";
        for (const auto& f : fields()) {
            if (std::strcmp(f.section, section) != 0) continue;
            ofs << f.key << " = ";
            f.save(ofs, s);
            ofs << "\n";
        }
        saveSectionExtras(ofs, section, s);
        ofs << "\n";
        if (std::strcmp(section, "Machine") == 0) {
            for (size_t i = 0; i < 4; ++i) {
                ofs << "[Axis_" << i << "]\n";
                for (const auto& f : axisFields()) {
                    ofs << f.key << " = ";
                    f.save(ofs, s.machine.axes[i]);
                    ofs << "\n";
                }
                ofs << "\n";
            }
        }
    }

    // One section per board that has settings (connection, firmware board options)
    for (const auto& [id, profile] : s.machine.board_profiles) {
        if (profile.values.empty()) continue;
        ofs << "[Board." << id << "]\n";
        for (const auto& [k, v] : profile.values) ofs << k << " = " << v << "\n";
        ofs << "\n";
    }

    // Integrity marker: a file without it was cut short (e.g. power loss while saving)
    ofs << "[Integrity]\ncomplete = 1\n";

    return writeFileAtomically(file_path, ofs.str());
}

} // namespace rnr::config
