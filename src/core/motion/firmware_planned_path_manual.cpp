// FirmwarePlannedPath: Manual moves, probing, homing, jogging, hold / stop, overrides and outputs (the class is split by topic; see firmware_planned_path.h)
#include "core/motion/firmware_planned_path_internal.h"

namespace rnr::core::motion {

using namespace fpp_detail;

// ============================================================================
// Single moves, probing, homing
// ============================================================================
std::array<double, 4> FirmwarePlannedPath::toFirmware(const std::array<double, 4>& m, bool with_frame) const {
    std::array<double, 4> r{};
    for (size_t a = 0; a < 4; ++a) r[a] = m[a] - shift_[a] - (with_frame ? frame_f_[a] : 0.0);
    return r;
}

bool FirmwarePlannedPath::startMoves(const std::vector<MoveRequest>& moves, const OutputRequest* during,
                                     std::string* error) {
    // Moves may follow a running command sequence (outputs, console); they are sent after it
    const bool append = op_ == Op::Script && isOpen() && phase_ == Phase::Ready && fw_state_ != "Alarm";
    if (!append) {
        if (!canStart(error)) return false;
        if (fw_state_ != "Idle") {
            if (error) *error = "The board is not idle (" + fb_.firmware_state + ")";
            return false;
        }
    }
    std::vector<ScriptLine> lines;
    const int n_axes = std::clamp<int>(fw_.axis_count, 3, 4);
    if (during && during->spindle_on) {
        ScriptLine l;
        l.text = std::string(during->laser && during->laser_dynamic ? "M4" : "M3") + "S" + GcodeEmitter::num(during->s_value, 0);
        lines.push_back(l);
    }
    for (const auto& mv : moves) {
        if (mv.dwell_s > 0.0) {
            ScriptLine d;
            d.text = "G4P" + GcodeEmitter::num(mv.dwell_s, 3);
            lines.push_back(d);
            continue;
        }
        const auto fw = toFirmware(mv.target_mm, false);
        std::string t = "G21G90G53";
        t += mv.rapid ? "G0" : "G1";
        for (int a = 0; a < n_axes; ++a) {
            t += kAxis[a];
            t += fmt(fw[static_cast<size_t>(a)]);
        }
        if (!mv.rapid) t += "F" + GcodeEmitter::num(std::max(1.0, mv.feed_mm_min), 0);
        ScriptLine l;
        l.text = t;
        lines.push_back(l);
    }
    if (during && during->spindle_on) {
        ScriptLine l;
        l.text = "M5";
        lines.push_back(l);
    }
    ScriptLine wait;
    wait.text = "G4P0";
    lines.push_back(wait);
    emitter_.reset();
    if (append) {
        // Continue the running command sequence (e.g. spindle off, then the tool change retract)
        for (auto& l : lines) script_.push_back(l);
        op_ = Op::Moves;
        return true;
    }
    startScript(Op::Moves, lines);
    return true;
}

bool FirmwarePlannedPath::startProbe(const ProbeRequest& p, std::string* error) {
    if (!canStart(error)) return false;
    if (fw_state_ != "Idle") {
        if (error) *error = "The board is not idle (" + fb_.firmware_state + ")";
        return false;
    }
    if (fb_.probe_active) {
        if (error) *error = "The probe is already in contact - move it clear first";
        return false;
    }
    const int n_axes = std::clamp<int>(fw_.axis_count, 3, 4);
    std::string t = "G21G91G38.3";
    bool any = false;
    for (int a = 0; a < n_axes; ++a) {
        const double d = p.target_mm[static_cast<size_t>(a)] - fb_.machine_pos_mm[static_cast<size_t>(a)];
        if (std::abs(d) >= 0.0005) {
            t += kAxis[a];
            t += fmt(d);
            any = true;
        }
    }
    if (!any) {
        if (error) *error = "The probe move has no length";
        return false;
    }
    t += "F" + GcodeEmitter::num(std::max(1.0, p.feed_mm_min), 0);
    ScriptLine probe;
    probe.text = t;
    ScriptLine abs;
    abs.text = "G90";
    emitter_.reset();
    startScript(Op::Probe, {probe, abs});
    return true;
}

bool FirmwarePlannedPath::startHoming(uint8_t axis_mask, std::string* error) {
    if (!canStart(error)) return false;
    if (!fw_.homing_enabled && fw_.settings_read && fw_.flavour != board::grbl::Flavour::FluidNC) {
        if (error) *error = "Homing is disabled on the board ($22 = 0)";
        return false;
    }
    ScriptLine l;
    l.text = "$H";
    l.quiet = true;
    if (axis_mask != 0) {
        if (!fw_.home_single_axis) {
            if (error) *error = "This board can only home all axes together";
            return false;
        }
        for (size_t a = 0; a < 4; ++a) {
            if ((axis_mask >> a) & 1U) l.text += kAxis[a];
        }
    }
    homing_mask_ = axis_mask;
    for (size_t a = 0; a < 4; ++a) {
        if (axis_mask == 0 || ((axis_mask >> a) & 1U)) fb_.homed[a] = false;
    }
    emitter_.reset();
    startScript(Op::Homing, {l});
    return true;
}

// ============================================================================
// Jogging
// ============================================================================
bool FirmwarePlannedPath::jogStep(size_t axis, double delta_mm, double feed, std::string* error) {
    if (!canStart(error)) return false;
    if (op_ == Op::Script) {
        // Started once the running command sequence is answered
        deferred_ = [this, axis, delta_mm, feed] {
            std::string e;
            if (!jogStep(axis, delta_mm, feed, &e)) {
                PathEvent ev;
                ev.kind = PathEvent::Kind::MoveDone;
                ev.ok = false;
                ev.text = "Jog rejected: " + e;
                emit(ev);
            }
        };
        return true;
    }
    if (fw_state_ != "Idle") {
        if (error) *error = "The board is not idle (" + fb_.firmware_state + ")";
        return false;
    }
    if (axis >= 4) return false;
    std::array<double, 4> d{};
    d[axis] = delta_mm;
    const std::string line = GcodeEmitter::jogLine(d, feed, std::clamp<int>(fw_.axis_count, 3, 4));
    jog_step_ticket_ = protocol_.sendLine(line);
    if (jog_step_ticket_ == 0) {
        if (error) *error = "Write to the board failed";
        return false;
    }
    jog_step_acked_ = false;
    status_since_ack_ = false;
    script_line_sent_at_ = Clock::now();
    op_ = Op::JogStep;
    return true;
}

void FirmwarePlannedPath::setJogVelocity(const std::array<double, 4>& v) {
    jog_vel_ = v;
}

void FirmwarePlannedPath::pumpJog(Clock::time_point now) {
    double speed = 0.0;
    for (double x : jog_vel_) speed += x * x;
    speed = std::sqrt(speed);

    if (op_ == Op::None) {
        if (speed < 1e-6 || phase_ != Phase::Ready || fw_state_ != "Idle") return;
        op_ = Op::JogContinuous;
        for (size_t a = 0; a < 4; ++a) jog_dir_[a] = jog_vel_[a] / speed;
        jog_speed_ = speed;
        jog_sent_ = 0.0;
        jog_start_ = fb_.machine_pos_mm;
        jog_cancelling_ = false;
    }
    if (op_ != Op::JogContinuous) return;

    if (jog_cancelling_) {
        if (fw_state_ == "Idle" && now - jog_cancel_at_ > std::chrono::milliseconds(120) && status_since_ack_ &&
            last_status_ > jog_cancel_at_) {
            op_ = Op::None;
            jog_cancelling_ = false;
            PathEvent e;
            e.kind = PathEvent::Kind::JogStopped;
            emit(e);
        }
        return;
    }
    if (speed < 1e-6) {
        protocol_.realtime(Realtime::JogCancel);
        jog_cancelling_ = true;
        jog_cancel_at_ = now;
        return;
    }
    double dot = 0.0;
    for (size_t a = 0; a < 4; ++a) dot += (jog_vel_[a] / speed) * jog_dir_[a];
    if (dot < 0.999) {
        // New direction: stop, then start again with the new vector
        protocol_.realtime(Realtime::JogCancel);
        jog_cancelling_ = true;
        jog_cancel_at_ = now;
        return;
    }
    jog_speed_ = speed;

    // Lead = distance sent minus distance travelled (extrapolated between status reports)
    double travelled = 0.0;
    for (size_t a = 0; a < 4; ++a) travelled += (fb_.machine_pos_mm[a] - jog_start_[a]) * jog_dir_[a];
    if (fw_state_ == "Jog") {
        const double dt = std::chrono::duration<double>(now - jog_report_time_).count();
        travelled += std::min(0.2, std::max(0.0, dt)) * fb_.feed_mm_min / 60.0;
    }
    double accel = 1e9;
    for (size_t a = 0; a < 4; ++a) {
        const double u = std::abs(jog_dir_[a]);
        if (u > 1e-9) accel = std::min(accel, std::max(1.0, fw_.accel_mm_s2[a]) / u);
    }
    if (accel > 1e8) accel = 100.0;
    const double lead_target = speed * speed / (2.0 * accel) + speed * kJogLeadTime_s;
    const double chunk = std::max(0.01, speed * kJogChunkTime_s);
    int sent_now = 0;
    while (jog_sent_ - travelled < lead_target && sent_now < 4 && protocol_.unacknowledged() < 6) {
        std::array<double, 4> d{};
        for (size_t a = 0; a < 4; ++a) d[a] = jog_dir_[a] * chunk;
        const std::string line = GcodeEmitter::jogLine(d, speed * 60.0, std::clamp<int>(fw_.axis_count, 3, 4));
        if (!protocol_.canSend(line.size())) break;
        if (protocol_.sendLine(line) == 0) break;
        jog_sent_ += chunk;
        ++sent_now;
    }
}

// ============================================================================
// Feed hold, resume, stop, E-stop, overrides, outputs
// ============================================================================
void FirmwarePlannedPath::hold() {
    if (!isOpen()) return;
    if (op_ == Op::JogContinuous || op_ == Op::JogStep) {
        protocol_.realtime(Realtime::JogCancel);
        jog_vel_ = {};
        return;
    }
    protocol_.realtime(Realtime::FeedHold, fw_.top_bit_realtime);
    job_held_ = true;
}

void FirmwarePlannedPath::resume() {
    if (!isOpen()) return;
    protocol_.realtime(Realtime::CycleStart, fw_.top_bit_realtime);
    job_held_ = false;
}

void FirmwarePlannedPath::stop() {
    if (!isOpen() || op_ == Op::None || op_ == Op::Stopping || op_ == Op::Script) return;
    const auto now = Clock::now();
    if (op_ == Op::JogContinuous || op_ == Op::JogStep) {
        protocol_.realtime(Realtime::JogCancel);
        jog_vel_ = {};
        jog_cancelling_ = true;
        jog_cancel_at_ = now;
        return;
    }
    if (op_ == Op::Job && job_phase_ == JobPhase::SyncWait) {
        // At rest at a sync point: nothing is moving, just forget the job
        op_ = Op::None;
        job_phase_ = JobPhase::Done;
        fb_.job_active = false;
        PathEvent e;
        e.kind = PathEvent::Kind::JobFinished;
        e.ok = false;
        emit(e);
        return;
    }
    stop_prev_op_ = op_;
    stop_from_job_ = op_ == Op::Job;
    protocol_.realtime(Realtime::FeedHold, fw_.top_bit_realtime);
    op_ = Op::Stopping;
    stop_phase_ = StopPhase::HoldWait;
    stop_deadline_ = now + kHoldTimeout;
    script_.clear();
    script_waiting_ = false;
    job_lines_.clear();
}

void FirmwarePlannedPath::emergencyStop() {
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    // Several callers may ask in the same moment (button, fault handling): one reset is enough. Also while the board
    // is being identified: it may be moving on its own (startup blocks, a pendant); tick() then starts over.
    if (now_ns - last_reset_ns_.load() < 300'000'000LL) return;
    if (protocol_.realtime(Realtime::SoftReset)) {
        last_reset_ns_ = now_ns;
        estop_requested_ = true;
    }
}

void FirmwarePlannedPath::setFeedOverride(float pct) {
    ov_feed_target_ = std::clamp(pct, 10.0f, 200.0f);
}

void FirmwarePlannedPath::setSpindleOverride(float pct) {
    ov_spindle_target_ = std::clamp(pct, 10.0f, 200.0f);
}

void FirmwarePlannedPath::setRapidOverride(float pct) {
    const int r = pct >= 75.0f ? 100 : (pct >= 37.5f ? 50 : 25);
    if (r != ov_rapid_target_) {
        ov_rapid_target_ = r;
        ov_rapid_dirty_ = true;
    }
}

void FirmwarePlannedPath::pumpOverrides() {
    if (phase_ != Phase::Ready) return;
    auto step = [&](float target_f, int& expected, Realtime r100, Realtime p10, Realtime m10, Realtime p1, Realtime m1) {
        const int target = static_cast<int>(std::lround(target_f));
        for (int i = 0; i < 3 && expected != target; ++i) {
            if (target == 100 && std::abs(expected - 100) > 1) {
                protocol_.realtime(r100);
                expected = 100;
            } else {
                const int d = target - expected;
                if (d >= 10) { protocol_.realtime(p10); expected += 10; }
                else if (d <= -10) { protocol_.realtime(m10); expected -= 10; }
                else if (d > 0) { protocol_.realtime(p1); expected += 1; }
                else { protocol_.realtime(m1); expected -= 1; }
            }
            ov_last_send_ = Clock::now();
        }
    };
    step(ov_feed_target_, ov_feed_expected_, Realtime::FeedOv100, Realtime::FeedOvPlus10, Realtime::FeedOvMinus10,
         Realtime::FeedOvPlus1, Realtime::FeedOvMinus1);
    step(ov_spindle_target_, ov_spindle_expected_, Realtime::SpindleOv100, Realtime::SpindleOvPlus10,
         Realtime::SpindleOvMinus10, Realtime::SpindleOvPlus1, Realtime::SpindleOvMinus1);
    if (ov_rapid_dirty_) {
        ov_rapid_dirty_ = false;
        protocol_.realtime(ov_rapid_target_ == 100 ? Realtime::RapidOv100
                           : ov_rapid_target_ == 50 ? Realtime::RapidOv50 : Realtime::RapidOv25);
    }
}

bool FirmwarePlannedPath::setOutputs(const OutputRequest& o, std::string* error) {
    if (!isOpen() || phase_ != Phase::Ready) {
        if (error) *error = "The board is not connected";
        return false;
    }
    if (op_ == Op::Job) {
        // The program controls the spindle; coolant can be toggled while it runs
        if (o.flood != fb_.flood_on) protocol_.realtime(Realtime::FloodToggle);
        if (o.mist != fb_.mist_on && fw_.has_mist) protocol_.realtime(Realtime::MistToggle);
        return true;
    }
    if (op_ != Op::None || (fw_state_ != "Idle" && fw_state_ != "Alarm")) {
        outputs_pending_ = true;
        outputs_req_ = o;
        return true;
    }
    if (fw_state_ == "Alarm") {
        if (error) *error = "The board is in alarm";
        return false;
    }
    ScriptLine sp;
    if (o.spindle_on) {
        sp.text = std::string((o.spindle_ccw || (o.laser && o.laser_dynamic)) ? "M4" : "M3") + "S" +
                  GcodeEmitter::num(std::max(0.0, o.s_value), 0);
    } else {
        sp.text = "M5";
    }
    // M7, M8 and M9 are one modal group: one per line
    std::vector<ScriptLine> lines{sp};
    for (const char* w : {"M9", o.flood ? "M8" : "", (o.mist && fw_.has_mist) ? "M7" : ""}) {
        if (*w == '\0') continue;
        ScriptLine c;
        c.text = w;
        lines.push_back(c);
    }
    emitter_.reset();
    startScript(Op::Script, lines);
    return true;
}

bool FirmwarePlannedPath::setAuxOutput(int index, bool on, std::string* error) {
    if (index < 0 || index >= emitter_.dialect().aux_outputs) {
        if (error) *error = "The board has no aux output " + std::to_string(index);
        return false;
    }
    if (!canStart(error)) return false;
    ScriptLine l;
    l.text = "M" + std::to_string(on ? 64 : 65) + "P" + std::to_string(index);
    startScript(Op::Script, {l});
    return true;
}

} // namespace rnr::core::motion
