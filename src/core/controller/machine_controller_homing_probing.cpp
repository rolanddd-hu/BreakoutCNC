// MachineController: Homing, probing and surface probing (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

// ============================================================================
// Hardware stop / settle helpers
// ============================================================================



// ============================================================================
// Homing (per axis: back off switch if needed -> seek switch -> set zero -> pull off)
// ============================================================================



void MachineController::homeAxis(core::Axis axis) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (static_cast<size_t>(axis) >= 4) return;
    startBuiltInHomingLocked(static_cast<int>(axis)); // One axis: always the built-in cycle
}

void MachineController::homeAllAxes() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_.scripting.enabled && config_.scripting.home_script_enabled && script_runner_) {
        if (!readyForCommandLocked(false, "Homing")) return;
        const std::string resolved = scripting::resolveScriptPath(config_.scripting.home_script_path);
        if (resolved.empty()) {
            telemetry_.active_fault_message = "Homing rejected: home script " + config_.scripting.home_script_path + " not found";
            return;
        }
        // The script homes with cnc.home(), which runs the built-in cycle (no recursion into the script)
        script_runner_->runFileAsync(resolved);
        return;
    }
    startBuiltInHomingLocked(-1);
}

void MachineController::homeAllAxesBuiltIn() {
    std::lock_guard<std::mutex> lock(mutex_);
    startBuiltInHomingLocked(-1);
}

void MachineController::startBuiltInHomingLocked(int axis) {
    if (!readyForCommandLocked(false, "Homing")) return;
    motionLocked().startHoming(axis);
}

// ============================================================================
// Probing (G31)
// ============================================================================
bool MachineController::beginProbeMoveLocked(core::Axis axis, double signed_travel_mm, double feed_mm_min,
                                         const double* sim_contact_work)
{
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4 || std::abs(signed_travel_mm) < 1e-6) return false;
    if (probeInputMissingLocked()) {
        telemetry_.active_fault_message = "Probing rejected: no probe input is configured";
        return false;
    }
    if (telemetry_.probe_contact_active) {
        telemetry_.active_fault_message = "Probing rejected: probe is already in contact - move it clear first";
        return false;
    }

    const std::array<double, 4> start_w = telemetry_.work_pos_mm;
    std::array<double, 4> end_w = start_w;
    if (telemetry_.simulator_mode) {
        // Simulator: contact is emulated where the move ends
        end_w[idx] = sim_contact_work ? *sim_contact_work : (start_w[idx] + signed_travel_mm * 0.5);
    } else {
        end_w[idx] += signed_travel_mm;
    }
    if (!checkLinearMoveLocked(start_w, end_w)) return false;

    if (!motionLocked().startProbeMove(idx, start_w, end_w, feed_mm_min)) return false;
    probe_contact_captured_ = false;
    probe_move_is_search_   = true;
    active_probe_axis_      = axis;
    active_probe_pos_dir_   = (signed_travel_mm >= 0.0);
    telemetry_.last_probe_hit = false;
    telemetry_.state = ControllerState::Probing;
    return true;
}

void MachineController::onProbeMoveFinishedLocked(bool hit) {
    const std::array<double, 4> contact_m = probe_contact_captured_ ? probe_contact_machine_ : telemetry_.machine_pos_mm;
    std::array<double, 4> contact_w{};
    for (size_t a = 0; a < 4; ++a) contact_w[a] = contact_m[a] - telemetry_.work_offset_mm[a];
    telemetry_.last_probed_work_pos_mm = contact_w;
    telemetry_.last_probe_hit = hit;
    telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    const size_t p_idx = static_cast<size_t>(active_probe_axis_);
    const double retract = std::max(1.0, config_.probe.retract_mm);
    // `offset` is the effective offset (tip frame); the stored WCS offset excludes the tool length
    auto setAxisOffset = [&](size_t a, double offset) {
        setActiveWcsAxisLocked(a, offset - (a == 2 ? activeTloLocked() : 0.0));
    };

    if (active_probe_mode_ == ProbeMode::Raw) {
        // G38.2-style: just report. Contact is still made at this point (no retract yet).
        active_probe_mode_ = ProbeMode::None;
        telemetry_.state = probe_return_state_;
        return;
    }
    if (!hit) {
        active_probe_mode_ = ProbeMode::None;
        telemetry_.active_fault_message = "Probing failed: no probe contact within the search distance";
        telemetry_.state = restStateLocked();
        return;
    }

    switch (active_probe_mode_) {
        case ProbeMode::FloatingHeadZ: {
            // Zero Work Z at plate contact and retract to pierce height
            setAxisOffset(2, contact_m[2]);
            active_probe_mode_ = ProbeMode::None;
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[2] = kFloatingHeadPierceHeightMm;
            if (!queueLinearMoveLocked(end_w, 600.0, ControllerState::Jogging)) telemetry_.state = restStateLocked();
            break;
        }
        case ProbeMode::ToolSetterZ: {
            // Work Z = plate thickness at the contact point, then retract +Z
            const double plate_h = config_.probe.plate_thickness_mm;
            setAxisOffset(2, contact_m[2] - plate_h);
            active_probe_mode_ = ProbeMode::None;
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[2] = plate_h + retract;
            if (!queueLinearMoveLocked(end_w, 600.0, ControllerState::Jogging)) telemetry_.state = restStateLocked();
            break;
        }
        case ProbeMode::EdgeSingleAxis: {
            // Compensate for probe tip radius: probing +dir, the edge is +radius beyond the tip centre
            const double radius = 0.5 * config_.probe.tool_diameter_mm;
            const double edge_pos = active_probe_pos_dir_ ? -radius : radius; // Tip centre in work coords
            setAxisOffset(p_idx, contact_m[p_idx] - edge_pos);
            active_probe_mode_ = ProbeMode::None;
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[p_idx] += active_probe_pos_dir_ ? -retract : retract;
            if (!queueLinearMoveLocked(end_w, 600.0, ControllerState::Jogging)) telemetry_.state = restStateLocked();
            break;
        }
        case ProbeMode::HoleCenterXY:
            continueHoleCenterLocked();
            break;
        case ProbeMode::ToolLength: {
            // Tool setter contact (machine Z). The reference tool defines length 0; every other tool's
            // length is how much higher (+) or lower (-) it touches the setter than the reference.
            const uint16_t tool = measuring_tool_;
            active_probe_mode_ = ProbeMode::None;
            double length = 0.0;
            if (tool == config_.tool_table.reference_tool) {
                tool_ref_contact_z_     = contact_m[2];
                tool_ref_contact_valid_ = true;
                telemetry_.tool_reference_measured = true;
            } else {
                length = contact_m[2] - tool_ref_contact_z_;
            }
            config_.tool_table.tools[tool].length_mm = length;
            config_.tool_table_revision++;
            if (telemetry_.active_tlo_tool == tool) {
                updateActiveToolLengthLocked();
                refreshWorkFrameLocked();
            }
            std::array<double, 4> end_w = telemetry_.work_pos_mm;
            end_w[2] += retract;
            if (!queueLinearMoveLocked(end_w, 600.0, ControllerState::Jogging)) telemetry_.state = restStateLocked();
            break;
        }
        default:
            active_probe_mode_ = ProbeMode::None;
            telemetry_.state = restStateLocked();
            break;
    }
}

void MachineController::continueHoleCenterLocked() {
    ++hole_step_;
    const double feed = std::max(50.0, config_.probe.fast_feed_mm_min);
    auto fail = [&]() {
        active_probe_mode_ = ProbeMode::None;
        motionLocked().clearQueuedMotion();
        telemetry_.state = restStateLocked();
        if (telemetry_.active_fault_message.empty()) {
            telemetry_.active_fault_message = "Hole centre probing aborted";
        }
    };
    auto probe = [&](core::Axis axis, double dir) {
        const size_t a = static_cast<size_t>(axis);
        const double sim_contact = hole_start_work_[a] + dir * (hole_search_mm_ * 0.8);
        if (!beginProbeMoveLocked(axis, dir * hole_search_mm_, feed, &sim_contact)) fail();
    };
    auto moveTo = [&](size_t a, double value) {
        std::array<double, 4> end_w = telemetry_.work_pos_mm;
        end_w[a] = value;
        probe_move_is_search_ = false;
        if (!queueLinearMoveLocked(end_w, feed, ControllerState::Probing)) {
            fail();
        } else if (!motionLocked().motionQueued() && !motionLocked().boardBusy()) {
            // Already there (no slices planned). A firmware board always runs the move and reports MoveDone, which
            // continues the sequence: continuing here as well started the next probe while it still moved.
            continueHoleCenterLocked();
        }
    };

    switch (hole_step_) {
        case 0: probe(core::Axis::X, -1.0); break;
        case 1: hole_contacts_work_[0] = telemetry_.last_probed_work_pos_mm[0]; moveTo(0, hole_start_work_[0]); break;
        case 2: probe(core::Axis::X, +1.0); break;
        case 3:
            hole_contacts_work_[1] = telemetry_.last_probed_work_pos_mm[0];
            moveTo(0, 0.5 * (hole_contacts_work_[0] + hole_contacts_work_[1]));
            break;
        case 4: probe(core::Axis::Y, -1.0); break;
        case 5: hole_contacts_work_[2] = telemetry_.last_probed_work_pos_mm[1]; moveTo(1, hole_start_work_[1]); break;
        case 6: probe(core::Axis::Y, +1.0); break;
        case 7:
            hole_contacts_work_[3] = telemetry_.last_probed_work_pos_mm[1];
            moveTo(1, 0.5 * (hole_contacts_work_[2] + hole_contacts_work_[3]));
            break;
        default: {
            // Tool is now at the measured centre: make it work X0 Y0
            for (size_t a = 0; a < 2; ++a) {
                setActiveWcsAxisLocked(a, wcsOffsetForLocked(a, 0.0));
            }
            active_probe_mode_ = ProbeMode::None;
            telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
            telemetry_.state = restStateLocked();
            break;
        }
    }
}

void MachineController::probeAxis(core::Axis axis, double max_travel_mm, double feed_mm_min) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(true, "Probing")) return;
    active_probe_mode_ = ProbeMode::FloatingHeadZ;
    double travel = max_travel_mm;
    if (std::abs(travel) < 1e-6) travel = -std::max(5.0, config_.probe.max_travel_mm);
    if (!beginProbeMoveLocked(axis, travel, feed_mm_min > 0.0 ? feed_mm_min : config_.probe.fast_feed_mm_min)) {
        active_probe_mode_ = ProbeMode::None;
    }
}

void MachineController::probeZWithToolSetter() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(true, "Probing")) return;
    active_probe_mode_ = ProbeMode::ToolSetterZ;
    const double travel = std::max(5.0, config_.probe.max_travel_mm);
    if (!beginProbeMoveLocked(core::Axis::Z, -travel, std::max(30.0, config_.probe.fast_feed_mm_min))) {
        active_probe_mode_ = ProbeMode::None;
    }
}

void MachineController::probeWorkAxisEdge(core::Axis axis, bool positive_dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Probing")) return;
    active_probe_mode_ = ProbeMode::EdgeSingleAxis;
    const double travel = std::max(5.0, config_.probe.max_travel_mm);
    if (!beginProbeMoveLocked(axis, positive_dir ? travel : -travel, std::max(30.0, config_.probe.fast_feed_mm_min))) {
        active_probe_mode_ = ProbeMode::None;
    }
}

void MachineController::probeHoleCenterXY(double approx_diameter_mm) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Probing")) return;
    if (probeInputMissingLocked()) {
        telemetry_.active_fault_message = "Probing rejected: no probe input is configured";
        return;
    }
    const double r = std::max(3.0, approx_diameter_mm * 0.5 - config_.probe.tool_diameter_mm * 0.5);
    hole_search_mm_   = r + std::max(2.0, 0.25 * r); // Search a bit beyond the expected wall
    hole_start_work_  = telemetry_.work_pos_mm;
    hole_step_        = -1;
    active_probe_mode_ = ProbeMode::HoleCenterXY;
    continueHoleCenterLocked();
}

void MachineController::probeRaw(core::Axis axis, double signed_travel_mm, double feed_mm_min) {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.last_probe_hit = false; // Reset first so a rejected request reads as "no contact"
    if (!readyForCommandLocked(true, "Probing")) return;
    const ControllerState return_state = telemetry_.state;
    active_probe_mode_ = ProbeMode::Raw;
    if (!beginProbeMoveLocked(axis, signed_travel_mm, feed_mm_min)) {
        active_probe_mode_ = ProbeMode::None;
        return;
    }
    probe_return_state_ = return_state;
}

// ============================================================================
// Surface probing (height map)
// ============================================================================
void MachineController::startSurfaceProbing(double min_x, double max_x, double min_y, double max_y, int points_x, int points_y, double smoothness) {
    if (points_x < 2 || points_y < 2 || points_x > 100 || points_y > 100) return;
    if (!(max_x > min_x) || !(max_y > min_y)) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!readyForCommandLocked(true, "Surface probing")) return;
        if (probeInputMissingLocked()) {
            telemetry_.active_fault_message = "Surface probing rejected: no probe input is configured";
            return;
        }
        surface_probing_cancel_requested_.store(false);
        telemetry_.surface_probing_active = true;
        telemetry_.surface_probe_current_point = 0;
        telemetry_.surface_probe_total_points = points_x * points_y;
    }

    if (surface_probing_thread_.joinable()) {
        surface_probing_thread_.join();
    }

    surface_probing_thread_ = std::thread([this, min_x, max_x, min_y, max_y, points_x, points_y, smoothness]() {
        struct GridPt { int i; int j; double x; double y; };
        std::vector<GridPt> grid;
        grid.reserve(static_cast<size_t>(points_x * points_y));

        const double dx = (max_x - min_x) / static_cast<double>(points_x - 1);
        const double dy = (max_y - min_y) / static_cast<double>(points_y - 1);
        for (int j = 0; j < points_y; ++j) {
            const double py = min_y + static_cast<double>(j) * dy;
            if (j % 2 == 0) {
                for (int i = 0; i < points_x; ++i) grid.push_back({i, j, min_x + static_cast<double>(i) * dx, py});
            } else {
                for (int i = points_x - 1; i >= 0; --i) grid.push_back({i, j, min_x + static_cast<double>(i) * dx, py});
            }
        }

        SurfaceHeightMap result_map;
        result_map.min_x = min_x;
        result_map.max_x = max_x;
        result_map.min_y = min_y;
        result_map.max_y = max_y;
        result_map.points_x = points_x;
        result_map.points_y = points_y;
        result_map.smoothness = smoothness;
        result_map.grid_z.assign(static_cast<size_t>(points_x * points_y), 0.0);

        double safe_clearance = 3.0, max_depth = 3.0, search_feed = 300.0, z_feed = 600.0, xy_feed = 1200.0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            safe_clearance = std::max(1.0, config_.probe.surface_clearance_z_mm);
            max_depth   = std::max(0.5, config_.probe.surface_max_depth_below_zero_mm);
            search_feed = std::max(20.0, config_.probe.fast_feed_mm_min);
            z_feed      = planner::TrajectoryPlanner::axisVelocityLimit(config_, 2) * 60.0;
            xy_feed     = std::max(300.0, planner::TrajectoryPlanner::axisVelocityLimit(config_, 0) * 60.0 * 0.5);
        }

        // Issue a step under the controller lock, re-checking cancellation INSIDE the lock so a cancel can
        // never be followed by new motion.
        auto issue = [this](const std::function<bool()>& fn) -> bool {
            std::lock_guard<std::mutex> lock(mutex_);
            if (surface_probing_cancel_requested_.load()) return false;
            if (telemetry_.state != ControllerState::Idle && telemetry_.state != ControllerState::ToolChangeHold) return false;
            return fn();
        };
        auto waitDone = [this]() -> bool {
            while (running_thread_.load()) {
                if (surface_probing_cancel_requested_.load()) return false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected) return false;
                    if (!motionActiveLocked() && !motionLocked().motionPending()) return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        };

        bool ok = true;
        // Initial safe Z lift if currently below safe_clearance
        ok = issue([&]() {
            std::array<double, 4> up = telemetry_.work_pos_mm;
            if (up[2] >= safe_clearance) return true;
            up[2] = safe_clearance;
            return queueLinearMoveLocked(up, z_feed, ControllerState::Jogging);
        }) && waitDone();

        for (size_t pt_idx = 0; ok && pt_idx < grid.size(); ++pt_idx) {
            const auto& pt = grid[pt_idx];

            // Move XY to probe point at safe clearance
            ok = issue([&]() {
                std::array<double, 4> xy = telemetry_.work_pos_mm;
                xy[0] = pt.x; xy[1] = pt.y; xy[2] = safe_clearance;
                return queueLinearMoveLocked(xy, xy_feed, ControllerState::Jogging);
            }) && waitDone();
            if (!ok) break;

            // Probe down toward -max_depth
            ok = issue([&]() {
                const double cur_z = telemetry_.work_pos_mm[2];
                double sim_z = 0.0;
                if (telemetry_.simulator_mode) {
                    const double nx = static_cast<double>(pt.i) / (points_x - 1);
                    const double ny = static_cast<double>(pt.j) / (points_y - 1);
                    sim_z = 0.22 * std::sin(nx * kPi) - 0.16 * std::cos(ny * kPi) + 0.12 * (nx - 0.5);
                    sim_z = std::clamp(sim_z, -max_depth + 0.1, safe_clearance - 0.1);
                }
                active_probe_mode_  = ProbeMode::Raw;
                probe_return_state_ = restStateLocked();
                if (!beginProbeMoveLocked(core::Axis::Z, -max_depth - cur_z, search_feed,
                                          telemetry_.simulator_mode ? &sim_z : nullptr)) {
                    active_probe_mode_ = ProbeMode::None;
                    return false;
                }
                return true;
            }) && waitDone();
            if (!ok) break;

            double measured_z = 0.0;
            bool hit = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                measured_z = telemetry_.last_probed_work_pos_mm[2];
                hit = telemetry_.last_probe_hit;
            }

            // Retract back to safe clearance
            ok = issue([&]() {
                std::array<double, 4> up = telemetry_.work_pos_mm;
                up[2] = safe_clearance;
                return queueLinearMoveLocked(up, z_feed, ControllerState::Jogging);
            }) && waitDone();
            if (!ok) break;

            if (!hit) {
                std::lock_guard<std::mutex> lock(mutex_);
                telemetry_.active_fault_message = "Surface probing error: no probe contact at grid point (" +
                    std::to_string(pt.i) + "," + std::to_string(pt.j) + "). Aborted for safety.";
                ok = false;
                break;
            }

            result_map.grid_z[static_cast<size_t>(pt.j * points_x + pt.i)] = measured_z;
            std::lock_guard<std::mutex> lock(mutex_);
            telemetry_.surface_probe_current_point = static_cast<int>(pt_idx + 1);
        }

        if (ok && !surface_probing_cancel_requested_.load()) {
            result_map.is_valid = true;
            result_map.recalculateStats();
            applySurfaceHeightMap(result_map);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.surface_probing_active = false;
    });
}

void MachineController::cancelSurfaceProbing() {
    surface_probing_cancel_requested_.store(true);
    std::lock_guard<std::mutex> lock(mutex_);
    if (telemetry_.surface_probing_active &&
        (telemetry_.state == ControllerState::Probing || telemetry_.state == ControllerState::Jogging))
    {
        motionLocked().stopForEvent();
        active_probe_mode_ = ProbeMode::None;
        telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
        telemetry_.state = restStateLocked();
    }
    // surface_probing_active is cleared by the probing thread itself when it exits
}

void MachineController::applySurfaceHeightMap(const SurfaceHeightMap& map) {
    std::shared_ptr<const gcode::ParseResult> base;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (jobActiveLocked(true)) {
            telemetry_.active_fault_message = "Height map cannot be changed while a job is active";
            return;
        }
        base = unwarped_program_;
    }

    // Warping can be slow on large programs: do it without holding the controller lock
    std::shared_ptr<const gcode::ParseResult> warped;
    if (map.is_valid && !base->segments.empty()) warped = warpProgram(*base, map);

    std::lock_guard<std::mutex> lock(mutex_);
    if (unwarped_program_ != base || motionActiveLocked()) {
        return; // Program reloaded or motion started meanwhile
    }
    surface_height_map_ = map;
    telemetry_.surface_height_map = std::make_shared<const SurfaceHeightMap>(map);
    if (!warped) {
        telemetry_.surface_compensation_active = false;
        return;
    }
    setProgramLocked(warped);
    telemetry_.surface_compensation_active = true;
}

void MachineController::clearSurfaceHeightMap() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (jobActiveLocked(true)) {
        telemetry_.active_fault_message = "Height map cannot be changed while a job is active";
        return;
    }
    setProgramLocked(unwarped_program_);
    telemetry_.surface_compensation_active = false;
}

bool MachineController::isSurfaceCompensationActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_.surface_compensation_active;
}

SurfaceHeightMap MachineController::getSurfaceHeightMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return surface_height_map_;
}

bool MachineController::getProgramBounds(std::array<double, 3>& out_min, std::array<double, 3>& out_max) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (program_->segments.empty()) return false;
    out_min = program_->bbox_min_mm;
    out_max = program_->bbox_max_mm;
    return true;
}

} // namespace rnr::core
