#include "core/config/settings_manager.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cctype>
#include <iomanip>
#include <algorithm>
#include <filesystem>

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
    if (std::filesystem::exists(file_path)) {
        // Legacy file written before the integrity marker existed
        s = primary;
        return true;
    }
    return false;
}

bool SettingsManager::loadFromFileImpl(const std::string& file_path, PersistentSettings& s, bool& complete) {
    complete = false;
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        return false;
    }

    std::string section;
    std::string line;
    bool has_macro_button_count = false;
    while (std::getline(ifs, line)) {
        std::string t = trim(line);
        if (t.empty() || t[0] == ';' || t[0] == '#') continue;

        if (t.front() == '[' && t.back() == ']') {
            section = trim(t.substr(1, t.size() - 2));
            continue;
        }

        size_t eq = t.find('=');
        if (eq == std::string::npos) continue;

        std::string key = trim(t.substr(0, eq));
        std::string val = trim(t.substr(eq + 1));

        if (section == "Integrity") {
            if (key == "complete" && toBool(val)) complete = true;
            continue;
        }

        if (section == "Window") {
            if (key == "width")      s.ui.window_width      = std::max(800, std::atoi(val.c_str()));
            if (key == "height")     s.ui.window_height     = std::max(600, std::atoi(val.c_str()));
            if (key == "pos_x")      s.ui.window_pos_x      = std::atoi(val.c_str());
            if (key == "pos_y")      s.ui.window_pos_y      = std::atoi(val.c_str());
            if (key == "maximized")  s.ui.window_maximized  = toBool(val);
            if (key == "fullscreen") s.ui.window_fullscreen = toBool(val);
        } else if (section == "UI") {
            if (key == "simulator_mode")          s.ui.simulator_mode          = toBool(val);
            if (key == "dry_run_mode")            s.ui.dry_run_mode            = toBool(val);
            if (key == "isometric_3d")            s.ui.isometric_3d            = toBool(val);
            if (key == "show_rapids")             s.ui.show_rapids             = toBool(val);
            if (key == "show_pierces")            s.ui.show_pierces            = toBool(val);
            if (key == "follow_torch")            s.ui.follow_torch            = toBool(val);
            if (key == "minimal_layout")          s.ui.minimal_layout          = toBool(val);
            if (key == "step_jog_mode")           s.ui.step_jog_mode           = toBool(val);
            if (key == "step_size_mm")            s.ui.step_size_mm            = static_cast<float>(std::atof(val.c_str()));
            if (key == "jog_speed_pct")           s.ui.jog_speed_pct           = static_cast<float>(std::atof(val.c_str()));
            if (key == "dro_jog_height")          s.ui.dro_jog_height          = static_cast<float>(std::atof(val.c_str()));
            if (key == "feed_override_pct")       s.ui.feed_override_pct       = static_cast<float>(std::atof(val.c_str()));
            if (key == "pierce_delay_override_s") s.ui.pierce_delay_override_s = static_cast<float>(std::atof(val.c_str()));
            if (key == "spindle_rpm")             s.ui.spindle_rpm             = std::atoi(val.c_str());
            if (key == "last_gcode_file_path")    s.ui.last_gcode_file_path    = val;
            if (key == "header_spindle_open")        s.ui.header_spindle_open        = toBool(val);
            if (key == "header_usb_open")            s.ui.header_usb_open            = toBool(val);
            if (key == "header_vfd_open")            s.ui.header_vfd_open            = toBool(val);
            if (key == "header_toolchanger_open")    s.ui.header_toolchanger_open    = toBool(val);
            if (key == "header_tool_table_open")     s.ui.header_tool_table_open     = toBool(val);
            if (key == "last_tool_in_spindle")       s.ui.last_tool_in_spindle       = std::clamp(std::atoi(val.c_str()), 0, 255);
            if (key == "last_tlo_tool")              s.ui.last_tlo_tool              = std::clamp(std::atoi(val.c_str()), 0, 255);
            if (key == "header_probe_macro_open")    s.ui.header_probe_macro_open    = toBool(val);
            if (key == "header_probe_settings_open") s.ui.header_probe_settings_open = toBool(val);
            if (key == "header_limits_open")         s.ui.header_limits_open         = toBool(val);
            if (key == "header_motor_open")          s.ui.header_motor_open          = toBool(val);
            if (key == "header_lua_open")            s.ui.header_lua_open            = toBool(val);
            if (key == "header_workspaces_open")     s.ui.header_workspaces_open     = toBool(val);
            if (key == "header_camera_open")         s.ui.header_camera_open         = toBool(val);
            if (key == "operation_mode") {
                s.ui.operation_mode = static_cast<uint8_t>(std::clamp(std::atoi(val.c_str()), 0, 1));
                s.machine.operation_mode = static_cast<core::OperationMode>(s.ui.operation_mode);
            }
            if (key == "laser_power_override_pct")   s.ui.laser_power_override_pct   = static_cast<float>(std::atof(val.c_str()));
            if (key == "header_laser_power_open")      s.ui.header_laser_power_open      = toBool(val);
            if (key == "header_laser_alignment_open")  s.ui.header_laser_alignment_open  = toBool(val);
            if (key == "header_laser_air_open")        s.ui.header_laser_air_open        = toBool(val);
            if (key == "speed_unit_idx")          s.ui.speed_unit_idx          = std::clamp(std::atoi(val.c_str()), 0, 1);
            if (key == "header_laser_interlocks_open") s.ui.header_laser_interlocks_open = toBool(val);
            if (key == "has_saved_coordinates") s.ui.has_saved_coordinates = toBool(val);
            if (key == "last_machine_x") s.ui.last_machine_pos_mm[0] = std::atof(val.c_str());
            if (key == "last_machine_y") s.ui.last_machine_pos_mm[1] = std::atof(val.c_str());
            if (key == "last_machine_z") s.ui.last_machine_pos_mm[2] = std::atof(val.c_str());
            if (key == "last_machine_a") s.ui.last_machine_pos_mm[3] = std::atof(val.c_str());
            if (key == "last_work_offset_x") s.ui.last_work_offset_mm[0] = std::atof(val.c_str());
            if (key == "last_work_offset_y") s.ui.last_work_offset_mm[1] = std::atof(val.c_str());
            if (key == "last_work_offset_z") s.ui.last_work_offset_mm[2] = std::atof(val.c_str());
            if (key == "last_work_offset_a") s.ui.last_work_offset_mm[3] = std::atof(val.c_str());
            if (key == "last_work_x") s.ui.last_work_pos_mm[0] = std::atof(val.c_str());
            if (key == "last_work_y") s.ui.last_work_pos_mm[1] = std::atof(val.c_str());
            if (key == "last_work_z") s.ui.last_work_pos_mm[2] = std::atof(val.c_str());
            if (key == "last_work_a") s.ui.last_work_pos_mm[3] = std::atof(val.c_str());
        } else if (section == "Machine") {
            if (key == "linear_units")        s.machine.linear_units        = static_cast<core::LinearUnits>(std::clamp(std::atoi(val.c_str()), 0, 1));
            if (key == "pulse_rate_khz")      s.machine.pulse_rate_khz      = static_cast<uint8_t>(std::atoi(val.c_str()));
            // Older settings files only knew A-follows-X
            if (key == "slave_a_to_x" && toBool(val)) s.machine.gantry_slave = core::GantrySlave::AFollowsX;
            if (key == "gantry_slave")        s.machine.gantry_slave        = static_cast<core::GantrySlave>(std::clamp(std::atoi(val.c_str()), 0, 2));
            // limit_override is deliberately NOT restored: end-stops are always active after a restart
            if (key == "dwell_p_milliseconds") s.machine.dwell_p_milliseconds = toBool(val);
            if (key == "estop_pin")           s.machine.estop_pin           = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "estop_invert")        s.machine.estop_invert        = toBool(val);
            if (key == "spindle_pwm_ratio")   s.machine.spindle_pwm_ratio   = static_cast<float>(std::atof(val.c_str()));
            if (key == "spindle_ramp_up_s")   s.machine.spindle_ramp_up_s   = std::max(0.0, std::atof(val.c_str()));
            if (key == "spindle_ramp_down_s") s.machine.spindle_ramp_down_s = std::max(0.0, std::atof(val.c_str()));
        } else if (section.rfind("Axis_", 0) == 0 && section.size() == 6) {
            int idx = section[5] - '0';
            if (idx >= 0 && idx < 4) {
                auto& ax = s.machine.axes[static_cast<size_t>(idx)];
                if (key == "steps_per_mm")        ax.steps_per_mm        = std::atof(val.c_str());
                if (key == "max_velocity_mm_s")   ax.max_velocity_mm_s   = std::atof(val.c_str());
                if (key == "acceleration_mm_s2")  ax.acceleration_mm_s2  = std::atof(val.c_str());
                if (key == "deceleration_mm_s2")  ax.deceleration_mm_s2  = std::atof(val.c_str());
                if (key == "home_pulloff_mm")     ax.home_pulloff_mm     = std::atof(val.c_str());
                if (key == "homing_feed_mm_min")  ax.homing_feed_mm_min  = std::atof(val.c_str());
                if (key == "homing_max_travel_mm") ax.homing_max_travel_mm = std::atof(val.c_str());
                if (key == "homing_dir_negative") ax.homing_dir_negative = toBool(val);
                if (key == "step_active_low")     ax.step_active_low     = toBool(val);
                if (key == "dir_active_low")      ax.dir_active_low      = toBool(val);
                if (key == "limit_plus_pin")      ax.limit_plus_pin      = static_cast<uint8_t>(std::atoi(val.c_str()));
                if (key == "limit_minus_pin")     ax.limit_minus_pin     = static_cast<uint8_t>(std::atoi(val.c_str()));
                if (key == "home_pin")            ax.home_pin            = static_cast<uint8_t>(std::atoi(val.c_str()));
                if (key == "limit_plus_invert")   ax.limit_plus_invert   = toBool(val);
                if (key == "limit_minus_invert")  ax.limit_minus_invert  = toBool(val);
                if (key == "home_invert")         ax.home_invert         = toBool(val);
                if (key == "soft_limits_enabled") ax.soft_limits_enabled = toBool(val);
                if (key == "soft_limit_min_mm")   ax.soft_limit_min_mm   = std::atof(val.c_str());
                if (key == "soft_limit_max_mm")   ax.soft_limit_max_mm   = std::atof(val.c_str());
            }
        } else if (section == "THC") {
            auto& thc = s.machine.thc;
            if (key == "enabled")              thc.enabled              = toBool(val);
            if (key == "wait_for_arc_ok")      thc.wait_for_arc_ok      = toBool(val);
            if (key == "arc_ok_pin")           thc.arc_ok_pin           = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "thc_up_pin")           thc.thc_up_pin           = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "thc_down_pin")         thc.thc_down_pin         = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "arc_ok_invert")        thc.arc_ok_invert        = toBool(val);
            if (key == "thc_up_invert")        thc.thc_up_invert        = toBool(val);
            if (key == "thc_down_invert")      thc.thc_down_invert      = toBool(val);
            if (key == "adjust_speed_mm_s")    thc.adjust_speed_mm_s    = std::atof(val.c_str());
            if (key == "min_xy_feed_pct")      thc.min_xy_feed_pct      = static_cast<float>(std::atof(val.c_str()));
            if (key == "delay_after_pierce_s") thc.delay_after_pierce_s = std::atof(val.c_str());
            if (key == "z_min_mm")             thc.z_min_mm             = std::atof(val.c_str());
            if (key == "z_max_mm")             thc.z_max_mm             = std::atof(val.c_str());
        } else if (section == "Laser") {
            auto& lz = s.machine.laser;
            if (key == "enabled")                lz.enabled                = toBool(val);
            if (key == "source_type")            lz.source_type            = static_cast<core::LaserSourceType>(std::clamp(std::atoi(val.c_str()), 0, 4));
            if (key == "framing_mode")           lz.framing_mode           = static_cast<core::LaserFramingMode>(std::clamp(std::atoi(val.c_str()), 0, 3));
            if (key == "dynamic_power_m4")       lz.dynamic_power_m4       = toBool(val);
            if (key == "pwm_frequency_khz")      lz.pwm_frequency_khz      = static_cast<float>(std::atof(val.c_str()));
            if (key == "min_duty_pct")           lz.min_duty_pct           = static_cast<float>(std::atof(val.c_str()));
            if (key == "max_duty_pct")           lz.max_duty_pct           = static_cast<float>(std::atof(val.c_str()));
            if (key == "corner_power_floor_pct") lz.corner_power_floor_pct = static_cast<float>(std::atof(val.c_str()));
            if (key == "ignore_z_motion")        lz.ignore_z_motion        = toBool(val);
            if (key == "auto_air_assist")        lz.auto_air_assist        = toBool(val);
            if (key == "air_preflow_s")          lz.air_preflow_s          = static_cast<float>(std::atof(val.c_str()));
            if (key == "air_postflow_s")         lz.air_postflow_s         = static_cast<float>(std::atof(val.c_str()));
            if (key == "framing_power_pct")      lz.framing_power_pct      = static_cast<float>(std::atof(val.c_str()));
            if (key == "framing_speed_mm_min")   lz.framing_speed_mm_min   = std::atof(val.c_str());
            if (key == "pulse_duration_ms")      lz.pulse_duration_ms      = static_cast<float>(std::atof(val.c_str()));
            if (key == "pulse_power_pct")        lz.pulse_power_pct        = static_cast<float>(std::atof(val.c_str()));
            if (key == "chiller_flow_pin")       lz.chiller_flow_pin       = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "chiller_flow_invert")    lz.chiller_flow_invert    = toBool(val);
            if (key == "door_interlock_pin")     lz.door_interlock_pin     = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "door_interlock_invert")  lz.door_interlock_invert  = toBool(val);
            if (key == "laser_fault_pin")        lz.laser_fault_pin        = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "laser_fault_invert")     lz.laser_fault_invert     = toBool(val);
            if (key == "red_dot_offset_x_mm")    lz.red_dot_offset_x_mm    = std::atof(val.c_str());
            if (key == "red_dot_offset_y_mm")    lz.red_dot_offset_y_mm    = std::atof(val.c_str());
        } else if (section == "Probe") {
            auto& pr = s.machine.probe;
            if (key == "probe_pin")           pr.probe_pin           = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "probe_invert")        pr.probe_invert        = toBool(val);
            if (key == "plate_thickness_mm")  pr.plate_thickness_mm  = std::atof(val.c_str());
            if (key == "tool_diameter_mm")    pr.tool_diameter_mm    = std::atof(val.c_str());
            if (key == "fast_feed_mm_min")    pr.fast_feed_mm_min    = std::atof(val.c_str());
            if (key == "slow_feed_mm_min")    pr.slow_feed_mm_min    = std::atof(val.c_str());
            if (key == "max_travel_mm")       pr.max_travel_mm       = std::atof(val.c_str());
            if (key == "retract_mm")          pr.retract_mm          = std::atof(val.c_str());
            if (key == "approx_hole_diam_mm") pr.approx_hole_diam_mm = std::atof(val.c_str());
            if (key == "xy_clearance_mm")     pr.xy_clearance_mm     = std::atof(val.c_str());
            if (key == "probe_depth_mm")      pr.probe_depth_mm      = std::atof(val.c_str());
            if (key == "approx_size_x_mm")    pr.approx_size_x_mm    = std::atof(val.c_str());
            if (key == "approx_size_y_mm")    pr.approx_size_y_mm    = std::atof(val.c_str());
            if (key == "surface_max_depth_below_zero_mm") pr.surface_max_depth_below_zero_mm = std::atof(val.c_str());
            if (key == "surface_clearance_z_mm")          pr.surface_clearance_z_mm          = std::atof(val.c_str());
            if (key == "surface_smoothness")              pr.surface_smoothness              = std::clamp(std::atof(val.c_str()), 0.0, 1.0);
            if (key == "surface_max_points")              pr.surface_max_points              = std::clamp(std::atoi(val.c_str()), 5, 500);
        } else if (section == "ToolChange") {
            auto& tc = s.machine.tool_change;
            if (key == "mode") {
                int m = std::clamp(std::atoi(val.c_str()), 0, 2);
                tc.mode = static_cast<core::ToolChangeMode>(m);
            }
            if (key == "safe_z_mm")         tc.safe_z_mm         = std::atof(val.c_str());
            if (key == "move_to_park_xy")   tc.move_to_park_xy   = toBool(val);
            if (key == "park_x_mm")         tc.park_x_mm         = std::atof(val.c_str());
            if (key == "park_y_mm")         tc.park_y_mm         = std::atof(val.c_str());
            if (key == "atc_dwell_seconds") tc.atc_dwell_seconds = std::atof(val.c_str());
        } else if (section == "ToolTable") {
            auto& tt = s.machine.tool_table;
            if (key == "mode") {
                tt.mode = static_cast<core::ToolLengthMode>(std::clamp(std::atoi(val.c_str()), 0, 2));
            } else if (key == "reference_tool") {
                tt.reference_tool = static_cast<uint16_t>(std::clamp(std::atoi(val.c_str()), 1, 255));
            } else if (key.rfind("tool_", 0) == 0) {
                // tool_<n> = <length_mm>;<diameter_mm>;<description>
                const int num = std::atoi(key.c_str() + 5);
                if (num >= 1 && num <= 255) {
                    core::ToolTableEntry e;
                    const size_t p1 = val.find(';');
                    const size_t p2 = (p1 == std::string::npos) ? std::string::npos : val.find(';', p1 + 1);
                    e.length_mm = std::atof(val.substr(0, p1).c_str());
                    if (p1 != std::string::npos) e.diameter_mm = std::atof(val.substr(p1 + 1, p2 - p1 - 1).c_str());
                    if (p2 != std::string::npos) e.description = trim(val.substr(p2 + 1));
                    tt.tools[static_cast<uint16_t>(num)] = e;
                }
            }
        } else if (section == "HuanyangVFD") {
            auto& vfd = s.machine.vfd;
            if (key == "enabled")            vfd.enabled = toBool(val);
            if (key == "port_name")          vfd.port_name = val;
            if (key == "baud_rate")          vfd.baud_rate = static_cast<uint32_t>(std::atol(val.c_str()));
            if (key == "slave_address")      vfd.slave_address = static_cast<uint8_t>(std::atoi(val.c_str()));
            if (key == "max_rpm")            vfd.max_rpm = static_cast<uint16_t>(std::atoi(val.c_str()));
            if (key == "max_freq_hz")        vfd.max_freq_hz = static_cast<float>(std::atof(val.c_str()));
            if (key == "min_rpm")            vfd.min_rpm = static_cast<uint16_t>(std::atoi(val.c_str()));
            if (key == "estop_on_comm_loss") vfd.estop_on_comm_loss = toBool(val);
            if (key == "estop_on_stall")     vfd.estop_on_stall = toBool(val);
            if (key == "speed_monitor_enabled") vfd.speed_monitor_enabled = toBool(val);
            if (key == "speed_margin_pct")   vfd.speed_margin_pct = static_cast<uint8_t>(std::clamp(std::atoi(val.c_str()), 1, 50));
            if (key == "bad_speed_threshold") vfd.bad_speed_threshold = static_cast<uint8_t>(std::clamp(std::atoi(val.c_str()), 1, 255));
            if (key == "tracking_threshold") vfd.tracking_threshold = static_cast<uint8_t>(std::clamp(std::atoi(val.c_str()), 1, 255));
            if (key == "limit_out_freq_hz")  vfd.limit_out_freq_hz = static_cast<float>(std::atof(val.c_str()));
            if (key == "limit_current_a")    vfd.limit_current_a = static_cast<float>(std::atof(val.c_str()));
            if (key == "limit_rpm")          vfd.limit_rpm = static_cast<float>(std::atof(val.c_str()));
            if (key == "limit_dc_volts")     vfd.limit_dc_volts = static_cast<float>(std::atof(val.c_str()));
            if (key == "limit_ac_volts")     vfd.limit_ac_volts = static_cast<float>(std::atof(val.c_str()));
            if (key == "limit_temp_c")       vfd.limit_temp_c = static_cast<float>(std::atof(val.c_str()));
        } else if (section == "Scripting") {
            auto& sc = s.machine.scripting;
            if (key == "enabled")             sc.enabled             = toBool(val);
            if (key == "scripts_dir")         sc.scripts_dir         = val;
            if (key == "m6_script_enabled")   sc.m6_script_enabled   = toBool(val);
            if (key == "m6_script_path")      sc.m6_script_path      = migrateScriptPath(val);
            if (key == "home_script_enabled") sc.home_script_enabled = toBool(val);
            if (key == "home_script_path")    sc.home_script_path    = migrateScriptPath(val);
            if (key == "ihs_script_enabled")  sc.ihs_script_enabled  = toBool(val);
            if (key == "ihs_script_path")     sc.ihs_script_path     = migrateScriptPath(val);
            if (key == "macro_button_count") {
                has_macro_button_count = true;
                int count = std::clamp(std::atoi(val.c_str()), 0, 50);
                sc.macro_buttons.assign(static_cast<size_t>(count), core::MacroButtonConfig{});
            }
            if (key.rfind("macro_btn_", 0) == 0) {
                size_t next_us = key.find('_', 10);
                if (next_us != std::string::npos) {
                    int raw_idx = std::atoi(key.substr(10, next_us - 10).c_str());
                    int idx = has_macro_button_count ? raw_idx : (raw_idx - 1);
                    std::string suffix = key.substr(next_us + 1);
                    if (idx >= 0 && idx < 50) {
                        if (static_cast<size_t>(idx) >= sc.macro_buttons.size()) {
                            sc.macro_buttons.resize(idx + 1);
                        }
                        if (suffix == "label") {
                            sc.macro_buttons[idx].label = val;
                        } else if (suffix == "path") {
                            sc.macro_buttons[idx].path = migrateScriptPath(val);
                        } else if (suffix == "control_panel") {
                            sc.macro_buttons[idx].show_on_control_panel = toBool(val);
                        }
                    }
                }
            }
        } else if (section == "Workspaces") {
            if (key == "active_workspace") {
                s.machine.active_workspace = static_cast<uint8_t>(std::clamp(std::atoi(val.c_str()), 0, 5));
            }
            for (int w = 0; w < 6; ++w) {
                std::string prefix = "g" + std::to_string(54 + w) + "_";
                if (key.rfind(prefix, 0) == 0 && key.size() == prefix.size() + 1) {
                    char ax = key[prefix.size()];
                    double d = std::atof(val.c_str());
                    if (ax == 'x') s.machine.workspace_offsets_mm[w][0] = d;
                    else if (ax == 'y') s.machine.workspace_offsets_mm[w][1] = d;
                    else if (ax == 'z') s.machine.workspace_offsets_mm[w][2] = d;
                    else if (ax == 'a') s.machine.workspace_offsets_mm[w][3] = d;
                }
            }
        } else if (section == "Camera") {
            auto& cam = s.machine.camera;
            if (key == "top_down_camera_index")   cam.top_down_camera_index   = std::atoi(val.c_str());
            if (key == "top_down_camera_name")    cam.top_down_camera_name    = val;
            if (key == "monitoring_camera_index") cam.monitoring_camera_index = std::atoi(val.c_str());
            if (key == "monitoring_camera_name")  cam.monitoring_camera_name  = val;
            if (key == "is_calibrated")           cam.is_calibrated           = toBool(val);
            if (key == "offset_x_mm")             cam.offset_x_mm             = std::atof(val.c_str());
            if (key == "offset_y_mm")             cam.offset_y_mm             = std::atof(val.c_str());
            if (key == "fov_x_mm")                cam.fov_x_mm                = std::max(1.0, std::atof(val.c_str()));
            if (key == "fov_y_mm")                cam.fov_y_mm                = std::max(1.0, std::atof(val.c_str()));
            if (key == "cal_move_x_mm")           cam.cal_move_x_mm           = std::max(0.5, std::atof(val.c_str()));
            if (key == "overlay_opacity")         cam.overlay_opacity         = std::clamp(static_cast<float>(std::atof(val.c_str())), 0.1f, 1.0f);
            if (key == "mirror_x")                cam.mirror_x                = toBool(val);
            if (key == "mirror_y")                cam.mirror_y                = toBool(val);
            if (key == "stream_enabled")          cam.stream_enabled          = toBool(val);
            if (key == "stream_port")             cam.stream_port             = static_cast<uint16_t>(std::clamp(std::atoi(val.c_str()), 1024, 65535));
            if (key == "stream_fps")              cam.stream_fps              = std::clamp(std::atoi(val.c_str()), 1, 60);
            if (key == "stream_allow_upload")     cam.stream_allow_upload     = toBool(val);
        }
    }

    return true;
}

bool SettingsManager::saveToFile(const std::string& file_path, const PersistentSettings& s) {
    std::ostringstream ofs;

    ofs << std::fixed << std::setprecision(3);
    ofs << "; ====================================================================\n";
    ofs << "; BreakoutCNC Persistent Configuration File\n";
    ofs << "; Automatically loaded on startup and saved on exit.\n";
    ofs << "; ====================================================================\n\n";

    ofs << "[Window]\n";
    ofs << "width = " << s.ui.window_width << "\n";
    ofs << "height = " << s.ui.window_height << "\n";
    ofs << "pos_x = " << s.ui.window_pos_x << "\n";
    ofs << "pos_y = " << s.ui.window_pos_y << "\n";
    ofs << "maximized = " << (s.ui.window_maximized ? 1 : 0) << "\n";
    ofs << "fullscreen = " << (s.ui.window_fullscreen ? 1 : 0) << "\n\n";

    ofs << "[UI]\n";
    ofs << "simulator_mode = " << (s.ui.simulator_mode ? 1 : 0) << "\n";
    ofs << "dry_run_mode = " << (s.ui.dry_run_mode ? 1 : 0) << "\n";
    ofs << "isometric_3d = " << (s.ui.isometric_3d ? 1 : 0) << "\n";
    ofs << "show_rapids = " << (s.ui.show_rapids ? 1 : 0) << "\n";
    ofs << "show_pierces = " << (s.ui.show_pierces ? 1 : 0) << "\n";
    ofs << "follow_torch = " << (s.ui.follow_torch ? 1 : 0) << "\n";
    ofs << "minimal_layout = " << (s.ui.minimal_layout ? 1 : 0) << "\n";
    ofs << "step_jog_mode = " << (s.ui.step_jog_mode ? 1 : 0) << "\n";
    ofs << "step_size_mm = " << s.ui.step_size_mm << "\n";
    ofs << "jog_speed_pct = " << s.ui.jog_speed_pct << "\n";
    ofs << "dro_jog_height = " << s.ui.dro_jog_height << "\n";
    ofs << "feed_override_pct = " << s.ui.feed_override_pct << "\n";
    ofs << "pierce_delay_override_s = " << s.ui.pierce_delay_override_s << "\n";
    ofs << "spindle_rpm = " << s.ui.spindle_rpm << "\n";
    ofs << "last_gcode_file_path = " << s.ui.last_gcode_file_path << "\n";
    ofs << "header_spindle_open = " << (s.ui.header_spindle_open ? 1 : 0) << "\n";
    ofs << "header_usb_open = " << (s.ui.header_usb_open ? 1 : 0) << "\n";
    ofs << "header_vfd_open = " << (s.ui.header_vfd_open ? 1 : 0) << "\n";
    ofs << "header_toolchanger_open = " << (s.ui.header_toolchanger_open ? 1 : 0) << "\n";
    ofs << "header_tool_table_open = " << (s.ui.header_tool_table_open ? 1 : 0) << "\n";
    ofs << "last_tool_in_spindle = " << s.ui.last_tool_in_spindle << "\n";
    ofs << "last_tlo_tool = " << s.ui.last_tlo_tool << "\n";
    ofs << "header_probe_macro_open = " << (s.ui.header_probe_macro_open ? 1 : 0) << "\n";
    ofs << "header_probe_settings_open = " << (s.ui.header_probe_settings_open ? 1 : 0) << "\n";
    ofs << "header_limits_open = " << (s.ui.header_limits_open ? 1 : 0) << "\n";
    ofs << "header_motor_open = " << (s.ui.header_motor_open ? 1 : 0) << "\n";
    ofs << "header_lua_open = " << (s.ui.header_lua_open ? 1 : 0) << "\n";
    ofs << "header_workspaces_open = " << (s.ui.header_workspaces_open ? 1 : 0) << "\n";
    ofs << "header_camera_open = " << (s.ui.header_camera_open ? 1 : 0) << "\n";
    ofs << "operation_mode = " << static_cast<int>(s.ui.operation_mode) << "\n";
    ofs << "laser_power_override_pct = " << s.ui.laser_power_override_pct << "\n";
    ofs << "header_laser_power_open = " << (s.ui.header_laser_power_open ? 1 : 0) << "\n";
    ofs << "header_laser_alignment_open = " << (s.ui.header_laser_alignment_open ? 1 : 0) << "\n";
    ofs << "header_laser_air_open = " << (s.ui.header_laser_air_open ? 1 : 0) << "\n";
    ofs << "header_laser_interlocks_open = " << (s.ui.header_laser_interlocks_open ? 1 : 0) << "\n";
    ofs << "speed_unit_idx = " << s.ui.speed_unit_idx << "\n";
    ofs << "has_saved_coordinates = " << (s.ui.has_saved_coordinates ? 1 : 0) << "\n";
    ofs << "last_machine_x = " << s.ui.last_machine_pos_mm[0] << "\n";
    ofs << "last_machine_y = " << s.ui.last_machine_pos_mm[1] << "\n";
    ofs << "last_machine_z = " << s.ui.last_machine_pos_mm[2] << "\n";
    ofs << "last_machine_a = " << s.ui.last_machine_pos_mm[3] << "\n";
    ofs << "last_work_offset_x = " << s.ui.last_work_offset_mm[0] << "\n";
    ofs << "last_work_offset_y = " << s.ui.last_work_offset_mm[1] << "\n";
    ofs << "last_work_offset_z = " << s.ui.last_work_offset_mm[2] << "\n";
    ofs << "last_work_offset_a = " << s.ui.last_work_offset_mm[3] << "\n";
    ofs << "last_work_x = " << s.ui.last_work_pos_mm[0] << "\n";
    ofs << "last_work_y = " << s.ui.last_work_pos_mm[1] << "\n";
    ofs << "last_work_z = " << s.ui.last_work_pos_mm[2] << "\n";
    ofs << "last_work_a = " << s.ui.last_work_pos_mm[3] << "\n\n";

    ofs << "[Machine]\n";
    ofs << "linear_units = " << static_cast<int>(s.machine.linear_units) << "\n";
    ofs << "pulse_rate_khz = " << static_cast<int>(s.machine.pulse_rate_khz) << "\n";
    ofs << "gantry_slave = " << static_cast<int>(s.machine.gantry_slave) << "\n";
    ofs << "dwell_p_milliseconds = " << (s.machine.dwell_p_milliseconds ? 1 : 0) << "\n";
    ofs << "estop_pin = " << static_cast<int>(s.machine.estop_pin) << "\n";
    ofs << "estop_invert = " << (s.machine.estop_invert ? 1 : 0) << "\n";
    ofs << "spindle_pwm_ratio = " << s.machine.spindle_pwm_ratio << "\n";
    ofs << "spindle_ramp_up_s = " << s.machine.spindle_ramp_up_s << "\n";
    ofs << "spindle_ramp_down_s = " << s.machine.spindle_ramp_down_s << "\n\n";

    for (size_t i = 0; i < 4; ++i) {
        const auto& ax = s.machine.axes[i];
        ofs << "[Axis_" << i << "]\n";
        ofs << "steps_per_mm = " << ax.steps_per_mm << "\n";
        ofs << "max_velocity_mm_s = " << ax.max_velocity_mm_s << "\n";
        ofs << "acceleration_mm_s2 = " << ax.acceleration_mm_s2 << "\n";
        ofs << "deceleration_mm_s2 = " << ax.deceleration_mm_s2 << "\n";
        ofs << "home_pulloff_mm = " << ax.home_pulloff_mm << "\n";
        ofs << "homing_feed_mm_min = " << ax.homing_feed_mm_min << "\n";
        ofs << "homing_max_travel_mm = " << ax.homing_max_travel_mm << "\n";
        ofs << "homing_dir_negative = " << (ax.homing_dir_negative ? 1 : 0) << "\n";
        ofs << "step_active_low = " << (ax.step_active_low ? 1 : 0) << "\n";
        ofs << "dir_active_low = " << (ax.dir_active_low ? 1 : 0) << "\n";
        ofs << "limit_plus_pin = " << static_cast<int>(ax.limit_plus_pin) << "\n";
        ofs << "limit_minus_pin = " << static_cast<int>(ax.limit_minus_pin) << "\n";
        ofs << "home_pin = " << static_cast<int>(ax.home_pin) << "\n";
        ofs << "limit_plus_invert = " << (ax.limit_plus_invert ? 1 : 0) << "\n";
        ofs << "limit_minus_invert = " << (ax.limit_minus_invert ? 1 : 0) << "\n";
        ofs << "home_invert = " << (ax.home_invert ? 1 : 0) << "\n";
        ofs << "soft_limits_enabled = " << (ax.soft_limits_enabled ? 1 : 0) << "\n";
        ofs << "soft_limit_min_mm = " << ax.soft_limit_min_mm << "\n";
        ofs << "soft_limit_max_mm = " << ax.soft_limit_max_mm << "\n\n";
    }

    const auto& thc = s.machine.thc;
    ofs << "[THC]\n";
    ofs << "enabled = " << (thc.enabled ? 1 : 0) << "\n";
    ofs << "wait_for_arc_ok = " << (thc.wait_for_arc_ok ? 1 : 0) << "\n";
    ofs << "arc_ok_pin = " << static_cast<int>(thc.arc_ok_pin) << "\n";
    ofs << "thc_up_pin = " << static_cast<int>(thc.thc_up_pin) << "\n";
    ofs << "thc_down_pin = " << static_cast<int>(thc.thc_down_pin) << "\n";
    ofs << "arc_ok_invert = " << (thc.arc_ok_invert ? 1 : 0) << "\n";
    ofs << "thc_up_invert = " << (thc.thc_up_invert ? 1 : 0) << "\n";
    ofs << "thc_down_invert = " << (thc.thc_down_invert ? 1 : 0) << "\n";
    ofs << "adjust_speed_mm_s = " << thc.adjust_speed_mm_s << "\n";
    ofs << "min_xy_feed_pct = " << thc.min_xy_feed_pct << "\n";
    ofs << "delay_after_pierce_s = " << thc.delay_after_pierce_s << "\n";
    ofs << "z_min_mm = " << thc.z_min_mm << "\n";
    ofs << "z_max_mm = " << thc.z_max_mm << "\n\n";

    const auto& lz = s.machine.laser;
    ofs << "[Laser]\n";
    ofs << "enabled = " << (lz.enabled ? 1 : 0) << "\n";
    ofs << "source_type = " << static_cast<int>(lz.source_type) << "\n";
    ofs << "framing_mode = " << static_cast<int>(lz.framing_mode) << "\n";
    ofs << "dynamic_power_m4 = " << (lz.dynamic_power_m4 ? 1 : 0) << "\n";
    ofs << "pwm_frequency_khz = " << lz.pwm_frequency_khz << "\n";
    ofs << "min_duty_pct = " << lz.min_duty_pct << "\n";
    ofs << "max_duty_pct = " << lz.max_duty_pct << "\n";
    ofs << "corner_power_floor_pct = " << lz.corner_power_floor_pct << "\n";
    ofs << "ignore_z_motion = " << (lz.ignore_z_motion ? 1 : 0) << "\n";
    ofs << "auto_air_assist = " << (lz.auto_air_assist ? 1 : 0) << "\n";
    ofs << "air_preflow_s = " << lz.air_preflow_s << "\n";
    ofs << "air_postflow_s = " << lz.air_postflow_s << "\n";
    ofs << "framing_power_pct = " << lz.framing_power_pct << "\n";
    ofs << "framing_speed_mm_min = " << lz.framing_speed_mm_min << "\n";
    ofs << "pulse_duration_ms = " << lz.pulse_duration_ms << "\n";
    ofs << "pulse_power_pct = " << lz.pulse_power_pct << "\n";
    ofs << "chiller_flow_pin = " << static_cast<int>(lz.chiller_flow_pin) << "\n";
    ofs << "chiller_flow_invert = " << (lz.chiller_flow_invert ? 1 : 0) << "\n";
    ofs << "door_interlock_pin = " << static_cast<int>(lz.door_interlock_pin) << "\n";
    ofs << "door_interlock_invert = " << (lz.door_interlock_invert ? 1 : 0) << "\n";
    ofs << "laser_fault_pin = " << static_cast<int>(lz.laser_fault_pin) << "\n";
    ofs << "laser_fault_invert = " << (lz.laser_fault_invert ? 1 : 0) << "\n";
    ofs << "red_dot_offset_x_mm = " << lz.red_dot_offset_x_mm << "\n";
    ofs << "red_dot_offset_y_mm = " << lz.red_dot_offset_y_mm << "\n\n";

    const auto& pr = s.machine.probe;
    ofs << "[Probe]\n";
    ofs << "probe_pin = " << static_cast<int>(pr.probe_pin) << "\n";
    ofs << "probe_invert = " << (pr.probe_invert ? 1 : 0) << "\n";
    ofs << "plate_thickness_mm = " << pr.plate_thickness_mm << "\n";
    ofs << "tool_diameter_mm = " << pr.tool_diameter_mm << "\n";
    ofs << "fast_feed_mm_min = " << pr.fast_feed_mm_min << "\n";
    ofs << "slow_feed_mm_min = " << pr.slow_feed_mm_min << "\n";
    ofs << "max_travel_mm = " << pr.max_travel_mm << "\n";
    ofs << "retract_mm = " << pr.retract_mm << "\n";
    ofs << "approx_hole_diam_mm = " << pr.approx_hole_diam_mm << "\n";
    ofs << "xy_clearance_mm = " << pr.xy_clearance_mm << "\n";
    ofs << "probe_depth_mm = " << pr.probe_depth_mm << "\n";
    ofs << "approx_size_x_mm = " << pr.approx_size_x_mm << "\n";
    ofs << "approx_size_y_mm = " << pr.approx_size_y_mm << "\n";
    ofs << "surface_max_depth_below_zero_mm = " << pr.surface_max_depth_below_zero_mm << "\n";
    ofs << "surface_clearance_z_mm = " << pr.surface_clearance_z_mm << "\n";
    ofs << "surface_smoothness = " << pr.surface_smoothness << "\n";
    ofs << "surface_max_points = " << pr.surface_max_points << "\n\n";

    const auto& tc = s.machine.tool_change;
    ofs << "[ToolChange]\n";
    ofs << "mode = " << static_cast<int>(tc.mode) << "\n";
    ofs << "safe_z_mm = " << tc.safe_z_mm << "\n";
    ofs << "move_to_park_xy = " << (tc.move_to_park_xy ? 1 : 0) << "\n";
    ofs << "park_x_mm = " << tc.park_x_mm << "\n";
    ofs << "park_y_mm = " << tc.park_y_mm << "\n";
    ofs << "atc_dwell_seconds = " << tc.atc_dwell_seconds << "\n\n";

    const auto& tt = s.machine.tool_table;
    ofs << "[ToolTable]\n";
    ofs << "; mode: 0 = not configured (G43 ignored, warning), 1 = touch off every tool, 2 = tool table\n";
    ofs << "mode = " << static_cast<int>(tt.mode) << "\n";
    ofs << "reference_tool = " << tt.reference_tool << "\n";
    ofs << "; tool_<n> = <length_mm relative to reference tool>;<diameter_mm>;<description>\n";
    {
        std::ostringstream lens;
        lens << std::fixed << std::setprecision(4);
        for (const auto& [num, e] : tt.tools) {
            lens.str("");
            lens << e.length_mm << ";" << e.diameter_mm << ";";
            ofs << "tool_" << num << " = " << lens.str() << e.description << "\n";
        }
    }
    ofs << "\n";

    const auto& vfd = s.machine.vfd;
    ofs << "[HuanyangVFD]\n";
    ofs << "enabled = " << (vfd.enabled ? 1 : 0) << "\n";
    ofs << "port_name = " << vfd.port_name << "\n";
    ofs << "baud_rate = " << vfd.baud_rate << "\n";
    ofs << "slave_address = " << static_cast<int>(vfd.slave_address) << "\n";
    ofs << "max_rpm = " << vfd.max_rpm << "\n";
    ofs << "max_freq_hz = " << vfd.max_freq_hz << "\n";
    ofs << "min_rpm = " << vfd.min_rpm << "\n";
    ofs << "estop_on_comm_loss = " << (vfd.estop_on_comm_loss ? 1 : 0) << "\n";
    ofs << "estop_on_stall = " << (vfd.estop_on_stall ? 1 : 0) << "\n";
    ofs << "speed_monitor_enabled = " << (vfd.speed_monitor_enabled ? 1 : 0) << "\n";
    ofs << "speed_margin_pct = " << static_cast<int>(vfd.speed_margin_pct) << "\n";
    ofs << "bad_speed_threshold = " << static_cast<int>(vfd.bad_speed_threshold) << "\n";
    ofs << "tracking_threshold = " << static_cast<int>(vfd.tracking_threshold) << "\n";
    ofs << "limit_out_freq_hz = " << vfd.limit_out_freq_hz << "\n";
    ofs << "limit_current_a = " << vfd.limit_current_a << "\n";
    ofs << "limit_rpm = " << vfd.limit_rpm << "\n";
    ofs << "limit_dc_volts = " << vfd.limit_dc_volts << "\n";
    ofs << "limit_ac_volts = " << vfd.limit_ac_volts << "\n";
    ofs << "limit_temp_c = " << vfd.limit_temp_c << "\n\n";

    const auto& sc = s.machine.scripting;
    ofs << "[Scripting]\n";
    ofs << "enabled = " << (sc.enabled ? 1 : 0) << "\n";
    ofs << "scripts_dir = " << sc.scripts_dir << "\n";
    ofs << "m6_script_enabled = " << (sc.m6_script_enabled ? 1 : 0) << "\n";
    ofs << "m6_script_path = " << sc.m6_script_path << "\n";
    ofs << "home_script_enabled = " << (sc.home_script_enabled ? 1 : 0) << "\n";
    ofs << "home_script_path = " << sc.home_script_path << "\n";
    ofs << "ihs_script_enabled = " << (sc.ihs_script_enabled ? 1 : 0) << "\n";
    ofs << "ihs_script_path = " << sc.ihs_script_path << "\n";
    ofs << "macro_button_count = " << sc.macro_buttons.size() << "\n";
    for (size_t i = 0; i < sc.macro_buttons.size(); ++i) {
        ofs << "macro_btn_" << i << "_label = " << sc.macro_buttons[i].label << "\n";
        ofs << "macro_btn_" << i << "_path = " << sc.macro_buttons[i].path << "\n";
        ofs << "macro_btn_" << i << "_control_panel = " << (sc.macro_buttons[i].show_on_control_panel ? 1 : 0) << "\n";
    }
    ofs << "\n";

    ofs << "[Workspaces]\n";
    ofs << "active_workspace = " << static_cast<int>(s.machine.active_workspace) << "\n";
    for (int w = 0; w < 6; ++w) {
        int g_num = 54 + w;
        ofs << "g" << g_num << "_x = " << s.machine.workspace_offsets_mm[w][0] << "\n";
        ofs << "g" << g_num << "_y = " << s.machine.workspace_offsets_mm[w][1] << "\n";
        ofs << "g" << g_num << "_z = " << s.machine.workspace_offsets_mm[w][2] << "\n";
        ofs << "g" << g_num << "_a = " << s.machine.workspace_offsets_mm[w][3] << "\n";
    }
    ofs << "\n";

    const auto& cam = s.machine.camera;
    ofs << "[Camera]\n";
    ofs << "top_down_camera_index = " << cam.top_down_camera_index << "\n";
    ofs << "top_down_camera_name = " << cam.top_down_camera_name << "\n";
    ofs << "monitoring_camera_index = " << cam.monitoring_camera_index << "\n";
    ofs << "monitoring_camera_name = " << cam.monitoring_camera_name << "\n";
    ofs << "is_calibrated = " << (cam.is_calibrated ? 1 : 0) << "\n";
    ofs << "offset_x_mm = " << cam.offset_x_mm << "\n";
    ofs << "offset_y_mm = " << cam.offset_y_mm << "\n";
    ofs << "fov_x_mm = " << cam.fov_x_mm << "\n";
    ofs << "fov_y_mm = " << cam.fov_y_mm << "\n";
    ofs << "cal_move_x_mm = " << cam.cal_move_x_mm << "\n";
    ofs << "overlay_opacity = " << cam.overlay_opacity << "\n";
    ofs << "mirror_x = " << (cam.mirror_x ? 1 : 0) << "\n";
    ofs << "mirror_y = " << (cam.mirror_y ? 1 : 0) << "\n";
    ofs << "stream_enabled = " << (cam.stream_enabled ? 1 : 0) << "\n";
    ofs << "stream_port = " << cam.stream_port << "\n";
    ofs << "stream_fps = " << cam.stream_fps << "\n";
    ofs << "stream_allow_upload = " << (cam.stream_allow_upload ? 1 : 0) << "\n";

    // Integrity marker: a file without it was cut short (e.g. power loss while saving)
    ofs << "\n[Integrity]\ncomplete = 1\n";

    return writeFileAtomically(file_path, ofs.str());
}

} // namespace rnr::config
