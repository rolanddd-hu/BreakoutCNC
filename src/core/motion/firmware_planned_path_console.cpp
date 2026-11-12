// FirmwarePlannedPath: Console, unlock, alarm / error texts, board report and settings (the class is split by topic; see firmware_planned_path.h)
#include "core/motion/firmware_planned_path_internal.h"

namespace rnr::core::motion {

using namespace fpp_detail;

bool FirmwarePlannedPath::sendConsoleLine(const std::string& line_in, std::string* error) {
    std::string line = line_in;
    while (!line.empty() && (line.back() == ' ' || line.back() == '\r' || line.back() == '\n')) line.pop_back();
    if (line.empty()) return true;
    if (line == "?" || line == "!" || line == "~") {
        const Realtime r = (line == "?") ? Realtime::StatusReport : (line == "!") ? Realtime::FeedHold : Realtime::CycleStart;
        return protocol_.realtime(r, fw_.top_bit_realtime);
    }
    if (!canStart(error)) return false;
    if (fw_state_ != "Idle" && fw_state_ != "Alarm") {
        if (error) *error = "Console commands are accepted only while the board is idle";
        return false;
    }
    ScriptLine l;
    l.text = line;
    l.quiet = line.rfind("$H", 0) == 0;
    l.collect = line == "$CD" || line == "$S";
    std::vector<ScriptLine> lines{l};
    if (line.rfind("$H", 0) != 0 && line.find('=') == std::string::npos) {
        // The command may have changed offsets: re-read them (F is taken from G54 again by every job)
        ScriptLine p;
        p.text = "$#";
        p.params = true;
        lines.push_back(p);
    }
    settings_after_op_ = line.size() > 1 && line[0] == '$' && line.find('=') != std::string::npos;
    emitter_.reset();
    startScript(Op::Script, lines);
    return true;
}

bool FirmwarePlannedPath::runLines(const std::vector<std::string>& lines, std::string* error) {
    if (!canStart(error)) return false;
    std::vector<ScriptLine> sl;
    for (const auto& l : lines) {
        ScriptLine x;
        x.text = l;
        sl.push_back(x);
    }
    emitter_.reset();
    startScript(Op::Script, sl);
    return true;
}

bool FirmwarePlannedPath::unlock(std::string* error) {
    if (!canStart(error)) return false;
    ScriptLine l;
    l.text = "$X";
    startScript(Op::Script, {l});
    fb_.status_text.clear();
    return true;
}

void FirmwarePlannedPath::readCodesOrParams(Clock::time_point now) {
    if (!codes_queue_.empty()) {
        const std::string line = codes_queue_.front();
        codes_queue_.pop_front();
        codes_target_ = line.back() == 'A' ? 'A' : 'E';
        phase_ = Phase::ReadCodes;
        phase_deadline_ = now + kSettingsTimeout;
        sendHandshakeLine(line);
        return;
    }
    codes_target_ = 0;
    phase_ = Phase::ReadParams;
    phase_deadline_ = now + kIdentifyTimeout;
    sendHandshakeLine("$#");
}

std::string FirmwarePlannedPath::alarmText(int code) const {
    // GRBL 1.1's alarms 1-9 mean the same on all three: BreakoutCNC's wording. Beyond that the codes differ
    // between grblHAL and FluidNC, so the board's own text wins.
    if (code >= 10) {
        const auto it = board_alarm_texts_.find(code);
        if (it != board_alarm_texts_.end()) return it->second;
    }
    return board::grbl::alarmInfo(code).text;
}

std::string FirmwarePlannedPath::errorText(int code) const {
    if (code > 38) {
        const auto it = board_error_texts_.find(code);
        if (it != board_error_texts_.end()) return it->second;
    }
    return board::grbl::errorText(code);
}

bool FirmwarePlannedPath::travelEnvelope(size_t a, double& lo, double& hi) const {
    if (a >= 4 || a >= fw_.axis_count || !fw_.settings_read || !fw_.homing_enabled) return false;
    if (fw_.flavour == board::grbl::Flavour::FluidNC) return false; // Its own rules (homing/mpos_mm): to confirm
    const double travel = std::abs(fw_.max_travel_mm[a]);
    if (travel < 1e-6) return false;
    // GRBL: machine zero at the home switch and the travel on the negative side; with build option Z (force
    // origin) an axis that homes toward its minimum has its travel on the positive side.
    const bool homes_negative = ((fw_.homing_dir_invert_mask >> a) & 1U) != 0;
    if (fw_.origin_at_home && homes_negative) {
        lo = 0.0;
        hi = travel;
    } else {
        lo = -travel;
        hi = 0.0;
    }
    lo += shift_[a];
    hi += shift_[a];
    return true;
}

bool FirmwarePlannedPath::startBoardReport(std::string* error) {
    if (!canStart(error)) return false;
    if (fw_state_ != "Idle" && fw_state_ != "Alarm") {
        if (error) *error = "The report is made only while the board is idle";
        return false;
    }
    std::vector<std::string> cmds;
    switch (fw_.flavour) {
        case board::grbl::Flavour::GrblHal: cmds = {"$I", "$I+", "$$", "$#", "$G", "$N"}; break;
        case board::grbl::Flavour::FluidNC: cmds = {"$I", "$$", "$#", "$G", "$CD", "$S"}; break;
        default:                            cmds = {"$I", "$$", "$#", "$G", "$N"}; break;
    }
    std::vector<ScriptLine> lines;
    for (const auto& c : cmds) {
        ScriptLine l;
        l.text = c;
        l.report = true;
        lines.push_back(l);
    }
    report_lines_.clear();
    report_running_ = true;
    emitter_.reset();
    startScript(Op::Script, lines);
    return true;
}

std::string FirmwarePlannedPath::boardReportText() const {
    std::string r;
    auto line = [&](const std::string& s) { r += s + "\n"; };
    auto yn = [](bool b) { return b ? std::string("yes") : std::string("no"); };
    const auto caps = capabilities();
    line("Board: " + caps.display_name + " (" + board_id_ + "), " + connectionLabel());
    line("Firmware: " + fw_.describe() + " | detected as " + board::grbl::flavourName(fw_.flavour) +
         (fw_.detected ? "" : " (not confirmed)"));
    line("Welcome: " + fw_.welcome);
    line("VER: " + fw_.version_text + " | OPT: " + fw_.option_codes + " | NEWOPT: " + fw_.new_options);
    line("FIRMWARE: " + fw_.firmware_name + " | BOARD: " + fw_.board_name);
    line("Axes: " + std::to_string(fw_.axis_count) + " (" + fw_.axis_letters + ") | planner blocks " +
         std::to_string(fw_.planner_blocks) + " | receive buffer " + std::to_string(fw_.rx_buffer_bytes) +
         " bytes | longest line " + std::to_string(fw_.max_line_chars));
    line("Line numbers: " + yn(fw_.reports_line_numbers) + " | homed state: " + yn(fw_.reports_homed) +
         " | single-axis homing: " + yn(fw_.home_single_axis) + " | origin at home (Z): " + yn(fw_.origin_at_home) +
         " | mist: " + yn(fw_.has_mist) + " | settings enumeration: " + yn(fw_.settings_enumeration) +
         " | top-bit real-time: " + yn(fw_.top_bit_realtime));
    line("Aux outputs: " + std::to_string(fw_.aux_outputs) + " | aux inputs: " + std::to_string(fw_.aux_inputs) +
         " | plasma plugin: " + yn(fw_.plasma_plugin) + (fw_.plasma_plugin ? " (" + fw_.plasma_mode + ")" : ""));
    char buf[256];
    std::snprintf(buf, sizeof(buf), "Settings read: %s | inches: %s | homing: %s | soft limits: %s | laser mode: %s | "
                  "spindle %.0f..%.0f RPM | homing dir mask %u",
                  yn(fw_.settings_read).c_str(), yn(fw_.reports_inches).c_str(), yn(fw_.homing_enabled).c_str(),
                  yn(fw_.soft_limits).c_str(), yn(fw_.laser_mode).c_str(), fw_.min_spindle_rpm, fw_.max_spindle_rpm,
                  static_cast<unsigned>(fw_.homing_dir_invert_mask));
    line(buf);
    for (size_t a = 0; a < std::min<size_t>(4, fw_.axis_count); ++a) {
        double lo = 0.0, hi = 0.0;
        const bool env = travelEnvelope(a, lo, hi);
        std::snprintf(buf, sizeof(buf), "%c: %.3f steps/mm, %.0f mm/min, %.1f mm/s2, travel %.1f mm%s", kAxis[a],
                      fw_.steps_per_mm[a], fw_.max_rate_mm_min[a], fw_.accel_mm_s2[a], fw_.max_travel_mm[a],
                      env ? "" : " (envelope not known)");
        std::string s = buf;
        if (env) {
            std::snprintf(buf, sizeof(buf), " -> app machine %.1f..%.1f", lo, hi);
            s += buf;
        }
        line(s);
    }
    std::snprintf(buf, sizeof(buf), "Machine shift: %.3f %.3f %.3f %.3f | board code texts: %zu alarms, %zu errors",
                  shift_[0], shift_[1], shift_[2], shift_[3], board_alarm_texts_.size(), board_error_texts_.size());
    line(buf);
    line("Capabilities: homing " + std::string(caps.homing == board::HomingKind::BoardCycle ? "board cycle" : "app") +
         " | single axis " + yn(caps.home_single_axis) + " | line tracking " + yn(caps.precise_line_tracking) +
         " | rapid override " + yn(caps.rapid_override) + " | E-stop input " + yn(caps.estop_input) +
         " | board THC " + yn(caps.board_thc) + " | aux outputs " + std::to_string(caps.aux_outputs));
    std::string io;
    for (const auto& p : caps.io) io += (io.empty() ? "" : ", ") + p.label;
    line("Inputs and outputs: " + io);
    line("");
    line("---- Answers ----");
    for (const auto& l : report_lines_) line(l);
    line("");
    line("---- Recent status reports ----");
    const auto entries = log_->since(0, 3000);
    std::vector<const transport::ProtocolLogEntry*> statuses, others;
    for (const auto& e : entries) (e.is_status ? statuses : others).push_back(&e);
    auto put = [&](const transport::ProtocolLogEntry& e) {
        using D = transport::ProtocolLogEntry::Dir;
        const char* p = (e.dir == D::Sent) ? "> " : (e.dir == D::Received) ? "  " : "# ";
        std::snprintf(buf, sizeof(buf), "%9.3f ", e.time_s);
        line(buf + std::string(p) + e.text);
    };
    for (size_t i = statuses.size() > 10 ? statuses.size() - 10 : 0; i < statuses.size(); ++i) put(*statuses[i]);
    line("");
    line("---- Recent exchange (without status reports) ----");
    for (size_t i = others.size() > 300 ? others.size() - 300 : 0; i < others.size(); ++i) put(*others[i]);
    return r;
}

void FirmwarePlannedPath::requestSettings() {
    if (!canStart(nullptr)) return;
    settings_.loading = true;
    ++settings_.revision;
    std::vector<ScriptLine> lines;
    ScriptLine all;
    all.text = "$$";
    all.settings = true;
    lines.push_back(all);
    if (fw_.flavour == board::grbl::Flavour::FluidNC) {
        ScriptLine cd;
        cd.text = "$CD";
        cd.collect = true;
        lines.push_back(cd);
    }
    startScript(Op::Script, lines);
}

bool FirmwarePlannedPath::writeSettings(const std::vector<std::pair<std::string, std::string>>& id_values,
                                        std::string* error) {
    if (!canStart(error)) return false;
    if (fw_state_ != "Idle" && fw_state_ != "Alarm") {
        if (error) *error = "Settings can only be changed while the board is idle";
        return false;
    }
    std::vector<ScriptLine> lines;
    for (const auto& [id, value] : id_values) {
        for (auto& it : settings_.items) {
            if (it.id == id) it.error.clear();
        }
        ScriptLine l;
        l.text = "$" + id + "=" + value;
        lines.push_back(l);
    }
    if (lines.empty()) return true;
    settings_after_op_ = true;
    startScript(Op::Script, lines);
    return true;
}

} // namespace rnr::core::motion
