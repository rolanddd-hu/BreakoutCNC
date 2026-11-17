// MachineController: Scripting, board console and firmware settings, firmware-planned boards (GRBL family) (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

// ============================================================================
// Scripting & Automation (Lua 5.4 Engine)
// ============================================================================
bool MachineController::runScriptString(const std::string& code, bool async) {
    if (!script_runner_) return false;
    if (async) {
        return script_runner_->runStringAsync(code);
    }
    return script_runner_->runStringBlocking(code);
}

bool MachineController::runScriptFile(const std::string& filepath, bool async) {
    if (!script_runner_) return false;
    if (async) {
        return script_runner_->runFileAsync(filepath);
    }
    return script_runner_->runFileBlocking(filepath);
}

void MachineController::cancelScript() {
    if (!script_runner_) return;
    const bool was_running = script_runner_->isRunning();
    script_runner_->cancel();
    if (!was_running) return;
    // Also stop the motion the script started (MDI moves, probing, homing)
    std::lock_guard<std::mutex> lock(mutex_);
    const auto s = telemetry_.state;
    if (!motionLocked().stopScriptMotion() && (s == ControllerState::Probing || s == ControllerState::Homing)) {
        stopJobLocked(false);
    }
}

bool MachineController::isScriptRunning() const {
    return script_runner_ && script_runner_->isRunning();
}

void MachineController::submitScriptPromptResponse(const std::string& response) {
    if (script_runner_) script_runner_->submitPromptResponse(response);
}

void MachineController::submitScriptPromptInput(const std::string& button, const std::string& input) {
    if (script_runner_) script_runner_->submitPromptResponse(button, input);
}

void MachineController::setScriptLogCallback(std::function<void(const std::string&, bool)> cb) {
    if (script_runner_) script_runner_->setLogCallback(std::move(cb));
}

// ============================================================================
// Board selection, console & firmware settings
// ============================================================================
void MachineController::fillBoardLimitsLocked(MachineCheckInput::BoardLimits& b) const {
    b = {};
    if (!fw_ || !fw_->isOpen() || !fw_->ready() || telemetry_.simulator_mode) return;
    const auto& p = fw_->firmware();
    b.known = true;
    b.name = board::grbl::flavourName(p.flavour);
    b.axis_count = p.axis_count;
    b.aux_outputs = (p.flavour == board::grbl::Flavour::Grbl11 || p.flavour == board::grbl::Flavour::GenericGrbl)
                        ? 0 : static_cast<uint8_t>(std::min<int>(4, p.aux_outputs));
    if (p.flavour == board::grbl::Flavour::FluidNC) b.aux_outputs = 4; // user_outputs: not reported, not checked
    b.rates_known = p.settings_read && p.flavour != board::grbl::Flavour::FluidNC;
    for (size_t a = 0; a < 4; ++a) {
        b.max_rate_mm_min[a] = (a < p.axis_count) ? p.max_rate_mm_min[a] : 0.0;
        double lo = 0.0, hi = 0.0;
        b.travel_known[a] = fw_->travelEnvelope(a, lo, hi);
        b.travel_min[a] = lo;
        b.travel_max[a] = hi;
    }
    b.soft_limits = p.soft_limits;
}

board::BoardCapabilities MachineController::getBoardCapabilitiesLocked() const {
    if (fw_ && fw_->isOpen()) return fw_->capabilities();
    if (config_.board_id == board::kRnrBoardId) return board_->capabilities();
    return board::BoardRegistry::instance().capabilitiesFor(config_.board_id);
}

board::BoardCapabilities MachineController::getBoardCapabilities() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return getBoardCapabilitiesLocked();
}

bool MachineController::unlockBoard() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fwReadyLocked()) return false;
    std::string err;
    if (!fw_->unlock(&err)) {
        telemetry_.active_fault_message = "Unlock rejected: " + err;
        return false;
    }
    telemetry_.board_status_text.clear();
    return true;
}

void MachineController::setRapidOverride(float percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.rapid_override_pct = percent >= 75.0f ? 100.0f : (percent >= 37.5f ? 50.0f : 25.0f);
    if (fwActiveLocked()) fw_->setRapidOverride(telemetry_.rapid_override_pct);
}

bool MachineController::sendBoardConsoleLine(const std::string& line) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fwActiveLocked()) {
        telemetry_.active_fault_message = "The console needs a connected GRBL-family board";
        return false;
    }
    const auto s = telemetry_.state;
    const bool realtime = line == "?" || line == "!" || line == "~";
    if (!realtime && s != ControllerState::Idle && s != ControllerState::EStop && s != ControllerState::ToolChangeHold) {
        telemetry_.active_fault_message = "Console commands are accepted only while the machine is idle";
        return false;
    }
    std::string err;
    if (!fw_->sendConsoleLine(line, &err)) {
        telemetry_.active_fault_message = "Console: " + err;
        return false;
    }
    return true;
}

std::vector<transport::ProtocolLogEntry> MachineController::getBoardLog(uint64_t after_seq) const {
    std::shared_ptr<transport::ProtocolLog> log;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fw_) log = fw_->log();
    }
    return log ? log->since(after_seq) : std::vector<transport::ProtocolLogEntry>{};
}

bool MachineController::requestBoardReport() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fwActiveLocked() || !fw_->ready()) {
        telemetry_.active_fault_message = "The board report needs a connected GRBL-family board";
        return false;
    }
    const auto s = telemetry_.state;
    if (s != ControllerState::Idle && s != ControllerState::EStop && s != ControllerState::ToolChangeHold) {
        telemetry_.active_fault_message = "The board report is made only while the machine is idle";
        return false;
    }
    std::string err;
    if (!fw_->startBoardReport(&err)) {
        telemetry_.active_fault_message = "Board report: " + err;
        return false;
    }
    return true;
}

BoardReport MachineController::getBoardReport() const {
    std::lock_guard<std::mutex> lock(mutex_);
    BoardReport r;
    if (!fw_) return r;
    r.revision = fw_->boardReportRevision();
    r.running = fw_->boardReportRunning();
    if (!r.running && r.revision > 0) r.text = fw_->boardReportText();
    return r;
}

board::grbl::SettingsSnapshot MachineController::getFirmwareSettings() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fw_ ? fw_->settings() : board::grbl::SettingsSnapshot{};
}

void MachineController::requestFirmwareSettings() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fwReadyLocked() && !motionActiveLocked()) fw_->requestSettings();
}

bool MachineController::writeFirmwareSettings(const std::vector<std::pair<std::string, std::string>>& id_values) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fwReadyLocked() || motionActiveLocked()) {
        telemetry_.active_fault_message = "Board settings can only be changed while the machine is idle";
        return false;
    }
    std::string err;
    if (!fw_->writeSettings(id_values, &err)) {
        telemetry_.active_fault_message = "Board settings: " + err;
        return false;
    }
    return true;
}

// ============================================================================
// Firmware-planned boards (GRBL family)
// ============================================================================
bool MachineController::probeInputMissingLocked() const noexcept {
    // Firmware boards have a fixed probe terminal
    return !telemetry_.simulator_mode && !firmwareBoardSelectedLocked() && config_.probe.probe_pin == 0;
}







std::string MachineController::toolDescriptionLocked(uint16_t tool) const {
    const auto it = program_->tool_descriptions.find(tool);
    return it != program_->tool_descriptions.end() ? it->second : std::string();
}

bool MachineController::runM6ScriptLocked() {
    if (!config_.scripting.enabled || !config_.scripting.m6_script_enabled || !script_runner_) return false;
    const std::string resolved = scripting::resolveScriptPath(config_.scripting.m6_script_path);
    if (resolved.empty()) return false;
    script_runner_->runFileAsync(resolved, [this](bool ok, const std::string&) {
        // Operator confirmed "Tool Ready": continue the job
        if (ok) this->startJob();
    });
    return true;
}

std::vector<ToolpathSegment> MachineController::toolChangeMovesLocked() {
    std::array<double, 4> cur_w = telemetry_.work_pos_mm;
    const double target_z = std::max(cur_w[2], config_.tool_change.safe_z_mm);
    std::vector<ToolpathSegment> moves;
    auto addRapid = [&](const std::array<double, 4>& a, const std::array<double, 4>& b) {
        ToolpathSegment seg{};
        seg.type = SegmentType::RapidMove;
        seg.start_xyza_mm = a;
        seg.end_xyza_mm = b;
        seg.length_mm = dist3(a, b);
        moves.push_back(seg);
    };
    const std::array<double, 4> z_up = {cur_w[0], cur_w[1], target_z, cur_w[3]};
    if (target_z - cur_w[2] > 0.05) {
        if (!checkLinearMoveLocked(cur_w, z_up)) return moves; // Soft limit: no retract, and no park move either
        addRapid(cur_w, z_up);
        cur_w = z_up;
    }
    if (config_.tool_change.move_to_park_xy) {
        const std::array<double, 4> park = {config_.tool_change.park_x_mm, config_.tool_change.park_y_mm, cur_w[2], cur_w[3]};
        if (std::hypot(park[0] - cur_w[0], park[1] - cur_w[1]) > 0.05 && checkLinearMoveLocked(cur_w, park)) {
            addRapid(cur_w, park);
        }
    }
    return moves;
}



} // namespace rnr::core
