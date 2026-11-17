#pragma once
// Shared by the machine_controller*.cpp files (one class, split by topic). Not for other code.
#include "core/controller/machine_controller.h"
#include "core/controller/firmware_backend.h"
#include "core/controller/slice_backend.h"
#include "core/diagnostics.h"
#include "core/io/io_map.h"
#include "core/scripting/script_runner.h"
#include "core/scripting/script_path_helper.h"
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <algorithm>
#include <filesystem>

#ifdef _WIN32
// Raise the Windows scheduler resolution to 1 ms for the 25 ms motion tick (default granularity is 15.6 ms)
extern "C" __declspec(dllimport) unsigned int __stdcall timeBeginPeriod(unsigned int uPeriod);
extern "C" __declspec(dllimport) unsigned int __stdcall timeEndPeriod(unsigned int uPeriod);
#endif

namespace rnr::core::mc_detail {


// The program with Z warped to the probed surface; the bounding box follows the warped moves
inline std::shared_ptr<const gcode::ParseResult> warpProgram(const gcode::ParseResult& base, const SurfaceHeightMap& map) {
    auto warped = std::make_shared<gcode::ParseResult>(base);
    warped->segments = planner::TrajectoryPlanner::warpToolpathWithHeightMap(base.segments, map, 2.0);
    bool first = true;
    for (const auto& seg : warped->segments) {
        if (seg.type == SegmentType::RapidMove && seg.hasMachineTarget()) continue;
        if (first) {
            warped->bbox_min_mm = {seg.start_xyza_mm[0], seg.start_xyza_mm[1], seg.start_xyza_mm[2]};
            warped->bbox_max_mm = warped->bbox_min_mm;
            first = false;
        }
        for (int i = 0; i < 3; ++i) {
            warped->bbox_min_mm[i] = std::min({warped->bbox_min_mm[i], seg.start_xyza_mm[i], seg.end_xyza_mm[i]});
            warped->bbox_max_mm[i] = std::max({warped->bbox_max_mm[i], seg.start_xyza_mm[i], seg.end_xyza_mm[i]});
        }
    }
    return warped;
}

constexpr auto   kTickInterval        = std::chrono::milliseconds(25); // 40 Hz engine loop
constexpr double kDtSec               = 0.025;
static_assert(kDtSec == board::IBoardDriver::kSlicePeriodSec, "The engine tick must match the slice period");
constexpr size_t kRefillThreshold     = 20;  // Re-plan when fewer than this many slices are queued
constexpr size_t kRefillAmount        = 40;
constexpr size_t kOverrideKeepSlices  = 12;  // Slices kept when a feed override change re-plans the queue
constexpr auto   kAckTimeout          = std::chrono::milliseconds(1500);
constexpr auto   kReportTimeout       = std::chrono::milliseconds(500);
constexpr auto   kFirstReportTimeout  = std::chrono::milliseconds(2000);
constexpr auto   kSettleTimeout       = std::chrono::milliseconds(400);
constexpr double kPi = 3.14159265358979323846;
constexpr double kFloatingHeadPierceHeightMm = 3.8;
// Spindle speed range without a VFD (the 0-10 V output scales to this), and the speed a resumed job spins up to
// when no speed was ever programmed
constexpr uint32_t kDefaultSpindleMaxRpm = 24000;
constexpr uint32_t kDefaultSpindleMinRpm = 1000;
constexpr uint16_t kFallbackSpindleRpm   = 12000;
inline constexpr const char* kAxisNames[4] = {"X", "Y", "Z", "A"};

inline bool finitePositive(double v) {
    return std::isfinite(v) && v > 0.0;
}

// Replace invalid / dangerous values so the motion code can rely on sane numbers
inline void sanitizeConfig(MachineConfig& c) {
    io::sanitize(c.io);
    for (auto& ax : c.axes) {
        if (!finitePositive(ax.steps_per_mm))        ax.steps_per_mm = 200.0;
        if (!finitePositive(ax.max_velocity_mm_s))   ax.max_velocity_mm_s = 10.0;
        if (!finitePositive(ax.acceleration_mm_s2))  ax.acceleration_mm_s2 = 100.0;
        if (!finitePositive(ax.deceleration_mm_s2))  ax.deceleration_mm_s2 = ax.acceleration_mm_s2;
        if (!finitePositive(ax.homing_feed_mm_min))  ax.homing_feed_mm_min = 300.0;
        if (!finitePositive(ax.homing_max_travel_mm)) ax.homing_max_travel_mm = 1500.0;
        if (!std::isfinite(ax.home_pulloff_mm) || ax.home_pulloff_mm < 0.0) ax.home_pulloff_mm = 2.5;
        if (ax.limit_plus_pin > 12)  ax.limit_plus_pin = 0;
        if (ax.limit_minus_pin > 12) ax.limit_minus_pin = 0;
        if (ax.home_pin > 12)        ax.home_pin = 0;
        if (ax.soft_limits_enabled && !(ax.soft_limit_min_mm < ax.soft_limit_max_mm)) {
            ax.soft_limits_enabled = false;
        }
    }
    c.pulse_rate_khz = MachineConfig::snapPulseRateKhz(c.pulse_rate_khz);
    if (c.estop_pin > 12) c.estop_pin = 0;
    c.start_safe_z_mm = std::isfinite(c.start_safe_z_mm) ? std::clamp(c.start_safe_z_mm, 0.0, 1000.0) : 25.0;
    c.corner_tolerance_mm = std::isfinite(c.corner_tolerance_mm)
        ? std::clamp(c.corner_tolerance_mm, MachineConfig::kMinCornerToleranceMm, MachineConfig::kMaxCornerToleranceMm)
        : 0.02;
    c.curve_accel_pct = std::clamp(c.curve_accel_pct, MachineConfig::kMinCurveAccelPct, MachineConfig::kMaxCurveAccelPct);
    if (c.probe.probe_pin > 12) c.probe.probe_pin = 0;
    c.spindle_ramp_up_s   = std::clamp(std::isfinite(c.spindle_ramp_up_s) ? c.spindle_ramp_up_s : 0.0, 0.0, 30.0);
    c.spindle_ramp_down_s = std::clamp(std::isfinite(c.spindle_ramp_down_s) ? c.spindle_ramp_down_s : 0.0, 0.0, 30.0);
    if (!c.is_mill) {
        // A torch / laser switches instantly: a spin-down delay (e.g. imported from a Mach3 profile) would keep
        // OUT1 on after M5 and ramp the PWM output with spindle RPM values
        c.spindle_ramp_up_s   = 0.0;
        c.spindle_ramp_down_s = 0.0;
    }
    if (c.vfd.max_rpm == 0) c.vfd.max_rpm = 24000;
    if (c.vfd.min_rpm > c.vfd.max_rpm) c.vfd.min_rpm = c.vfd.max_rpm;
    if (!(c.vfd.max_freq_hz > 0.0f)) c.vfd.max_freq_hz = 400.0f;
    c.vfd.speed_margin_pct    = std::clamp<uint8_t>(c.vfd.speed_margin_pct, 1, 50);
    c.vfd.bad_speed_threshold = std::max<uint8_t>(c.vfd.bad_speed_threshold, 1);
    c.vfd.tracking_threshold  = std::max<uint8_t>(c.vfd.tracking_threshold, 1);
    for (float* lim : {&c.vfd.limit_out_freq_hz, &c.vfd.limit_current_a, &c.vfd.limit_rpm,
                       &c.vfd.limit_dc_volts, &c.vfd.limit_ac_volts, &c.vfd.limit_temp_c}) {
        if (!std::isfinite(*lim) || *lim < 0.0f) *lim = 0.0f;
    }
    c.active_workspace = std::min<uint8_t>(c.active_workspace, 5);
    // Tool table: valid tool numbers and plausible lengths only
    for (auto it = c.tool_table.tools.begin(); it != c.tool_table.tools.end();) {
        auto& e = it->second;
        if (it->first == 0 || it->first > 255 || !std::isfinite(e.length_mm) || std::abs(e.length_mm) > 1000.0) {
            it = c.tool_table.tools.erase(it);
            continue;
        }
        if (!std::isfinite(e.diameter_mm) || e.diameter_mm < 0.0) e.diameter_mm = 0.0;
        ++it;
    }
    if (c.tool_table.reference_tool == 0 || c.tool_table.reference_tool > 255) c.tool_table.reference_tool = 1;
}

inline uint16_t clampRpm(uint32_t rpm) {
    return static_cast<uint16_t>(std::min<uint32_t>(rpm, 65535u));
}

inline uint16_t rpmToDuty(uint32_t rpm, uint32_t max_rpm) {
    max_rpm = std::max<uint32_t>(1, max_rpm);
    return static_cast<uint16_t>((static_cast<uint64_t>(std::min(rpm, max_rpm)) * 1000u) / max_rpm);
}

inline double dist3(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}


} // namespace rnr::core::mc_detail
