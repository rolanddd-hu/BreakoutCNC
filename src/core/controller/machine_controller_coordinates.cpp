// MachineController: Work coordinates and offsets, tool table, soft limits (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

// ============================================================================
// Coordinates & work offsets
// ============================================================================
void MachineController::setMachinePositionLocked(const std::array<double, 4>& machine_pos_mm) {
    telemetry_.machine_pos_mm = machine_pos_mm;
    for (size_t a = 0; a < 4; ++a) {
        telemetry_.work_pos_mm[a] = telemetry_.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
    }
    slice_->positionRedefinedLocked();
}

std::array<double, 4> MachineController::workToMachineLocked(const std::array<double, 4>& work) const {
    std::array<double, 4> m{};
    for (size_t a = 0; a < 4; ++a) m[a] = work[a] + telemetry_.work_offset_mm[a];
    return m;
}

// ----------------------------------------------------------------------------
// Work frame with tool length offset:
//   work_offset (effective, telemetry) = WCS offset + (0, 0, active tool length, 0)
//   work position = machine position - effective work offset   (= tool tip in work coordinates)
// ----------------------------------------------------------------------------
double MachineController::activeTloLocked() const noexcept {
    return (config_.tool_table.mode == ToolLengthMode::ToolTable) ? telemetry_.active_tlo_mm : 0.0;
}

void MachineController::refreshWorkFrameLocked() {
    telemetry_.work_offset_mm = telemetry_.workspace_offsets_mm[std::min<uint8_t>(telemetry_.active_workspace, 5)];
    telemetry_.work_offset_mm[2] += activeTloLocked();
    for (size_t a = 0; a < 4; ++a) {
        telemetry_.work_pos_mm[a] = telemetry_.machine_pos_mm[a] - telemetry_.work_offset_mm[a];
    }
}

double MachineController::wcsOffsetForLocked(size_t axis, double work_value) const {
    // WCS offset that makes the CURRENT tool tip read `work_value` on this axis
    return telemetry_.machine_pos_mm[axis] - work_value - (axis == 2 ? activeTloLocked() : 0.0);
}

void MachineController::setActiveWcsAxisLocked(size_t axis, double wcs_offset) {
    telemetry_.workspace_offsets_mm[telemetry_.active_workspace][axis] = wcs_offset;
    config_.workspace_offsets_mm = telemetry_.workspace_offsets_mm;
    bumpWorkspaceRevisionLocked();
    refreshWorkFrameLocked();
}

void MachineController::updateActiveToolLengthLocked() {
    if (config_.tool_table.mode != ToolLengthMode::ToolTable) {
        telemetry_.active_tlo_tool = 0;
        telemetry_.active_tlo_mm   = 0.0;
        return;
    }
    const auto it = config_.tool_table.tools.find(telemetry_.active_tlo_tool);
    telemetry_.active_tlo_mm = (telemetry_.active_tlo_tool != 0 && it != config_.tool_table.tools.end())
                                   ? it->second.length_mm : 0.0;
}

bool MachineController::setToolLengthOffsetLocked(uint16_t tool) {
    if (config_.tool_table.mode != ToolLengthMode::ToolTable) {
        telemetry_.active_tlo_tool = 0;
        telemetry_.active_tlo_mm   = 0.0;
        refreshWorkFrameLocked();
        return true; // G43 / G49 have no effect outside tool table mode
    }
    if (tool != 0 && config_.tool_table.tools.find(tool) == config_.tool_table.tools.end()) {
        telemetry_.active_fault_message = "Tool T" + std::to_string(tool) + " has no length in the tool table";
        return false;
    }
    telemetry_.active_tlo_tool = tool;
    updateActiveToolLengthLocked();
    refreshWorkFrameLocked();
    return true;
}

void MachineController::zeroWorkAxis(core::Axis axis) {
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (jobActiveLocked(false)) {
        telemetry_.active_fault_message = "Cannot zero an axis while a job is active";
        return;
    }
    setActiveWcsAxisLocked(idx, wcsOffsetForLocked(idx, 0.0));
    if (idx == 2) telemetry_.thc_z_offset_mm = 0.0;
}

void MachineController::zeroAllWorkAxes() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (jobActiveLocked(false)) {
        telemetry_.active_fault_message = "Cannot zero axes while a job is active";
        return;
    }
    for (size_t i = 0; i < 4; ++i) {
        setActiveWcsAxisLocked(i, wcsOffsetForLocked(i, 0.0));
    }
    telemetry_.thc_z_offset_mm = 0.0;
}

void MachineController::goToWorkZeroXY() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!readyForCommandLocked(false, "Go to zero")) return;

    // Retract Z to a safe height first so the XY travel cannot drag the tool through the work
    const std::array<double, 4> cur = telemetry_.work_pos_mm;
    const double safe_z = std::max(cur[2], config_.tool_change.safe_z_mm);
    std::vector<ToolpathSegment> moves;
    auto addRapid = [&](const std::array<double, 4>& a, const std::array<double, 4>& b) {
        ToolpathSegment s{};
        s.type          = SegmentType::RapidMove;
        s.start_xyza_mm = a;
        s.end_xyza_mm   = b;
        s.outputs_mask  = telemetry_.outputs_bitmask & ~core::OutputBit::OUT1_TorchOrSpindle;
        s.length_mm     = dist3(a, b);
        moves.push_back(s);
    };
    std::array<double, 4> up = {cur[0], cur[1], safe_z, cur[3]};
    std::array<double, 4> xy = {0.0, 0.0, safe_z, cur[3]};
    if (!checkLinearMoveLocked(cur, up) || !checkLinearMoveLocked(up, xy)) return;
    if (safe_z - cur[2] > 1e-3) addRapid(cur, up);
    addRapid(up, xy);
    motionLocked().startManualMoves(moves, ControllerState::Jogging, ManualMovePlan::Machine);
}

void MachineController::selectWorkspace(uint8_t index) {
    if (index >= 6) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (jobActiveLocked(false)) {
        telemetry_.active_fault_message = "Cannot change the work coordinate system while a job is active";
        return;
    }
    telemetry_.active_workspace = index;
    config_.active_workspace    = index;
    bumpWorkspaceRevisionLocked();
    refreshWorkFrameLocked();
}

void MachineController::setWorkspaceOffset(uint8_t index, const std::array<double, 4>& offset_mm) {
    if (index >= 6) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (jobActiveLocked(false)) {
        telemetry_.active_fault_message = "Cannot change work offsets while a job is active";
        return;
    }
    telemetry_.workspace_offsets_mm[index] = offset_mm;
    config_.workspace_offsets_mm           = telemetry_.workspace_offsets_mm;
    bumpWorkspaceRevisionLocked();
    refreshWorkFrameLocked();
}

void MachineController::setWorkspaceOffsetAxis(uint8_t index, core::Axis axis, double offset_mm) {
    if (index >= 6) return;
    const size_t a = static_cast<size_t>(axis);
    if (a >= 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (jobActiveLocked(false)) {
        telemetry_.active_fault_message = "Cannot change work offsets while a job is active";
        return;
    }
    telemetry_.workspace_offsets_mm[index][a] = offset_mm;
    config_.workspace_offsets_mm              = telemetry_.workspace_offsets_mm;
    bumpWorkspaceRevisionLocked();
    refreshWorkFrameLocked();
}

uint8_t MachineController::getActiveWorkspace() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_.active_workspace;
}

std::array<std::array<double, 4>, 6> MachineController::getWorkspaceOffsets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_.workspace_offsets_mm;
}

void MachineController::setWorkPosition(core::Axis axis, double value_mm) {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t idx = static_cast<size_t>(axis);
    if (idx >= 4) return;
    if (jobActiveLocked(false)) {
        telemetry_.active_fault_message = "Cannot set work position while a job is active";
        return;
    }
    setActiveWcsAxisLocked(idx, wcsOffsetForLocked(idx, value_mm));
    if (idx == 2) telemetry_.thc_z_offset_mm = 0.0;
}

void MachineController::restoreSavedPositions(
    const std::array<double, 4>& machine_pos_mm,
    const std::array<double, 4>& work_offset_mm)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked()) return;
    // `work_offset_mm` is the WCS offset (without any tool length offset)
    telemetry_.workspace_offsets_mm[telemetry_.active_workspace] = work_offset_mm;
    config_.workspace_offsets_mm = telemetry_.workspace_offsets_mm;
    bumpWorkspaceRevisionLocked();
    // A restored position is only a best guess (the machine may have moved while powered off):
    // the axes are NOT considered homed until a real homing cycle has run, and tool lengths measured
    // against the old machine Z are not comparable with new measurements.
    telemetry_.axis_homed = {false, false, false, false};
    tool_ref_contact_valid_ = false;
    telemetry_.tool_reference_measured = false;
    refreshWorkFrameLocked();
    setMachinePositionLocked(machine_pos_mm);
}

// ============================================================================
// Tool table
// ============================================================================
void MachineController::selectToolInSpindle(uint16_t tool) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tool > 255) return;
    if (jobActiveLocked(false)) {
        telemetry_.active_fault_message = "Cannot change the tool in spindle while a job is active";
        return;
    }
    if (config_.tool_table.mode == ToolLengthMode::ToolTable && !setToolLengthOffsetLocked(tool)) {
        return; // Unknown length: keep the previous tool & offset rather than guessing
    }
    if (tool > 0) telemetry_.current_tool_number = tool;
}

void MachineController::activateToolLengthOffset(uint16_t tool) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tool > 255) return;
    if (jobActiveLocked(false)) {
        telemetry_.active_fault_message = "Cannot change the tool length offset while a job is active";
        return;
    }
    setToolLengthOffsetLocked(tool);
}

void MachineController::measureToolLength(uint16_t tool) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tool == 0 || tool > 255) return;
    if (config_.tool_table.mode != ToolLengthMode::ToolTable) {
        telemetry_.active_fault_message = "Tool measuring needs Tool Length Mode = Tool Table";
        return;
    }
    if (!readyForCommandLocked(true, "Tool measuring")) return;
    if (tool != config_.tool_table.reference_tool && !tool_ref_contact_valid_) {
        telemetry_.active_fault_message = "Measure the reference tool T" + std::to_string(config_.tool_table.reference_tool) +
                                          " first (required after every startup / Z homing)";
        return;
    }
    measuring_tool_ = tool;
    active_probe_mode_ = ProbeMode::ToolLength;
    const double travel = std::max(5.0, config_.probe.max_travel_mm);
    if (!beginProbeMoveLocked(core::Axis::Z, -travel, std::max(30.0, config_.probe.fast_feed_mm_min))) {
        active_probe_mode_ = ProbeMode::None;
    }
}

// ============================================================================
// Soft limits
// ============================================================================
bool MachineController::withinSoftLimitsLocked(const std::array<double, 4>& machine_pt, std::string* err) const {
    // Firmware boards enforce their own soft limits ($20); the app's limits belong to its own motor settings
    if (!motionLocked().appSoftLimits()) return true;
    for (size_t a = 0; a < 4; ++a) {
        const auto& ax = config_.axes[a];
        // Soft limits are only meaningful relative to a homed machine coordinate system
        if (!ax.soft_limits_enabled || !telemetry_.axis_homed[a]) continue;
        if (machine_pt[a] < ax.soft_limit_min_mm - 1e-6 || machine_pt[a] > ax.soft_limit_max_mm + 1e-6) {
            if (err) {
                char buf[160];
                std::snprintf(buf, sizeof(buf), "%s %.3f mm (machine) is outside the soft limits %.1f..%.1f mm",
                              kAxisNames[a], machine_pt[a], ax.soft_limit_min_mm, ax.soft_limit_max_mm);
                *err = buf;
            }
            return false;
        }
    }
    return true;
}

bool MachineController::checkLinearMoveLocked(const std::array<double, 4>& start_work, const std::array<double, 4>& end_work) {
    (void)start_work; // A straight line between two in-limit points stays inside the (box) limits
    std::string err;
    if (!withinSoftLimitsLocked(workToMachineLocked(end_work), &err)) {
        telemetry_.active_fault_message = "Move rejected: " + err;
        return false;
    }
    return true;
}

bool MachineController::queueLinearMoveLocked(const std::array<double, 4>& end_work, double feed_mm_min,
                                          ControllerState state, bool rapid)
{
    const std::array<double, 4> start_w = telemetry_.work_pos_mm;
    if (!checkLinearMoveLocked(start_w, end_work)) return false;
    ToolpathSegment s{};
    s.type          = rapid ? SegmentType::RapidMove : SegmentType::CutLinear;
    s.start_xyza_mm = start_w;
    s.end_xyza_mm   = end_work;
    s.feed_mm_min   = feed_mm_min;
    s.length_mm     = dist3(start_w, end_work);
    return motionLocked().startManualMoves({s}, state, ManualMovePlan::Sequence);
}

} // namespace rnr::core
