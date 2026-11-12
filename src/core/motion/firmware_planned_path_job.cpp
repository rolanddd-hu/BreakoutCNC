// FirmwarePlannedPath: Job streaming and progress tracking (the class is split by topic; see firmware_planned_path.h)
#include "core/motion/firmware_planned_path_internal.h"

namespace rnr::core::motion {

using namespace fpp_detail;

// ============================================================================
// Jobs
// ============================================================================
size_t FirmwarePlannedPath::jobSourceSize() const {
    const size_t prog = job_.program ? job_.program->segments.size() : 0;
    return job_.prefix.size() + (prog > job_.first_index ? prog - job_.first_index : 0);
}

const ToolpathSegment& FirmwarePlannedPath::jobSegment(size_t v) const {
    if (v < job_.prefix.size()) return job_.prefix[v];
    return job_.program->segments[jobProgramIndex(v)];
}

std::array<double, 4> FirmwarePlannedPath::resolveSegment(const ToolpathSegment& s) const {
    auto t = planner::TrajectoryPlanner::transformTarget(s, job_.frame);
    if (job_.laser && job_.laser_lock_z) t[2] = job_.laser_z_plan;
    if (job_.frame.enabled) {
        const auto& off = job_.frame.workspace_offsets[std::min<uint8_t>(job_.frame.plan_workspace, 5)];
        for (size_t a = 0; a < 4; ++a) t[a] += off[a];
    }
    return t;
}

bool FirmwarePlannedPath::startJob(JobRequest job, std::string* error) {
    if (!canStart(error)) return false;
    if (op_ == Op::Script) {
        auto shared = std::make_shared<JobRequest>(std::move(job));
        deferred_ = [this, shared] {
            std::string e;
            if (!startJob(std::move(*shared), &e)) {
                PathEvent ev;
                ev.kind = PathEvent::Kind::JobFinished;
                ev.ok = false;
                ev.text = "Cycle Start rejected: " + e;
                emit(ev);
            }
        };
        return true;
    }
    if (fw_state_ != "Idle") {
        if (error) *error = (fw_state_ == "Alarm") ? "The board is in alarm: unlock or home it first"
                                                   : "The board is not idle (" + fb_.firmware_state + ")";
        return false;
    }
    if (!job.program) {
        if (error) *error = "No program";
        return false;
    }
    if (job.laser && !fw_.laser_mode && fw_.flavour != board::grbl::Flavour::FluidNC) {
        if (error) *error = "Laser jobs need the board's laser mode: set $32 = 1 in the board settings";
        return false;
    }
    job_ = std::move(job);
    // Board THC moves Z on its own: follow the job by the torch's XY position
    track_xy_only_ = job_.plasma && boardRunsThc();
    // Until the first status report the job is at its first segment, not where the previous job ended
    fb_.active_segment = job_.first_index;
    if (job_.first_index < job_.program->segments.size()) fb_.active_line = job_.program->segments[job_.first_index].gcode_line;
    fb_.segment_progress = 0.0f;
    job_cursor_ = 0;
    job_lines_.clear();
    sent_moves_.clear();
    tracker_index_ = 0;
    tracker_sub_ = 0;
    acked_moves_ = 0;
    job_produce_blocked_ = false;
    job_drain_ticket_ = 0;
    job_drain_acked_ = false;
    job_held_ = false;
    job_pos_ = fb_.machine_pos_mm;
    job_outputs_ = 0;
    job_rpm_ = job_.rpm_at_start;
    job_laser_s_set_ = false;
    job_vfd_on_ = job_.spindle_on_at_start;
    job_vfd_ccw_ = job_.spindle_ccw_at_start;
    job_vfd_rpm_ = job_.rpm_at_start;
    job_finish_lines_.clear();
    emitter_.reset();
    {
        // The preamble switches spindle and coolant off: the emitter knows that state
        std::vector<std::string> dummy;
        emitter_.outputs(false, false, -1.0, false, false, false, dummy);
    }
    job_preamble_.clear();
    for (const char* l : {"G21G90G94G17", "G54", "G49", "G92.1", "M5M9"}) {
        ScriptLine s;
        s.text = l;
        job_preamble_.push_back(s);
    }
    ScriptLine p;
    p.text = "$#";
    p.params = true;
    job_preamble_.push_back(p);
    job_preamble_pos_ = 0;
    job_preamble_waiting_ = false;
    job_phase_ = JobPhase::Preamble;
    fb_.job_active = true;
    fb_.progress_is_estimate = !fw_.reports_line_numbers;
    op_ = Op::Job;
    pumpJob(Clock::now());
    return true;
}

void FirmwarePlannedPath::continueJob() {
    if (op_ == Op::Job && job_phase_ == JobPhase::SyncWait) {
        job_produce_blocked_ = false;
        job_drain_ticket_ = 0;
        job_drain_acked_ = false;
        job_phase_ = JobPhase::Streaming;
    }
}

bool FirmwarePlannedPath::emitOutputsFor(const ToolpathSegment& s, std::vector<std::string>& lines, bool* sync_needed) {
    uint8_t mask = s.outputs_mask;
    if (job_.dry_run) mask &= ~OutputBit::OUT1_TorchOrSpindle;
    if (job_.laser) {
        // In laser mode the board switches the beam off during G0 by itself: switching it off and on around every
        // rapid would only add lines
        if ((mask & OutputBit::OUT1_TorchOrSpindle) && job_.laser_air_assist) mask |= OutputBit::OUT3_FloodCoolant;
    }
    if (!job_.mill) mask &= ~OutputBit::OUT2_SpindleDir; // Red pointer: no such output on these boards
    // Laser: a programmed S0 is a real value (beam off). Mill: S0 keeps the operator's speed, as on the RNR board.
    const bool s_given = job_.laser ? s.spindle_s_set : s.spindle_rpm > 0;
    const uint16_t rpm = s_given ? s.spindle_rpm : job_rpm_;
    const bool spindle_on = (mask & OutputBit::OUT1_TorchOrSpindle) != 0;
    const bool ccw = job_.mill && (mask & OutputBit::OUT2_SpindleDir) != 0;

    if (job_.app_drives_spindle) {
        if (spindle_on != job_vfd_on_ || (spindle_on && (ccw != job_vfd_ccw_ || rpm != job_vfd_rpm_))) {
            *sync_needed = true;
            job_vfd_on_ = spindle_on;
            job_vfd_ccw_ = ccw;
            job_vfd_rpm_ = rpm;
            return false;
        }
    }
    if (mask == job_outputs_ && rpm == job_rpm_) return true;

    double s_value = -1.0;
    if (job_.mill && !job_.app_drives_spindle) {
        s_value = rpm;
    } else if (job_.plasma) {
        // The torch is the spindle output: with S0 a variable-spindle board keeps PWM (and, set that way, enable) off
        s_value = fw_.max_spindle_rpm;
    } else if (job_.laser) {
        core::LaserConfig lz;
        lz.min_duty_pct = job_.laser_min_duty_pct;
        lz.max_duty_pct = job_.laser_max_duty_pct;
        lz.s_max = job_.laser_s_max;
        double pct = core::laserPowerPctForS(lz, s.spindle_s_set || job_laser_s_set_, rpm);
        pct = std::clamp(pct * job_.laser_power_override_pct / 100.0, 0.0, 100.0);
        s_value = core::laserDutyPctForPower(lz, pct) / 100.0 * fw_.max_spindle_rpm;
    }
    const bool was_on = (job_outputs_ & OutputBit::OUT1_TorchOrSpindle) != 0;
    const bool board_spindle = spindle_on && !job_.app_drives_spindle;
    emitter_.outputs(board_spindle, job_.laser ? job_.laser_dynamic : ccw, s_value, job_.laser_dynamic,
                     (mask & OutputBit::OUT3_FloodCoolant) != 0, (mask & OutputBit::OUT4_MistCoolant) != 0, lines);
    if (board_spindle && !was_on && job_.mill && job_.spinup_dwell_s > 0.01) {
        emitter_.dwell(job_.spinup_dwell_s, lines);
    }
    for (int i = 0; i < 4; ++i) {
        const uint8_t bit = static_cast<uint8_t>(1U << (kCustomOutputShift + i));
        if ((mask & bit) != (job_outputs_ & bit)) emitter_.auxOutput(i, (mask & bit) != 0, true, lines);
    }
    job_outputs_ = mask;
    job_rpm_ = rpm;
    job_laser_s_set_ = job_laser_s_set_ || s.spindle_s_set;
    return true;
}

bool FirmwarePlannedPath::produceJobLines() {
    if (job_cursor_ >= jobSourceSize()) {
        job_produce_blocked_ = true;
        job_sync_segment_ = SIZE_MAX; // The end
        return false;
    }
    const ToolpathSegment& seg = jobSegment(job_cursor_);
    const bool from_program = jobIsProgram(job_cursor_);
    const size_t seg_index = from_program ? jobProgramIndex(job_cursor_) : seg.segment_index;

    if (seg.type == SegmentType::ToolChange || seg.type == SegmentType::ProgramStop) {
        job_produce_blocked_ = true;
        job_sync_kind_ = (seg.type == SegmentType::ToolChange) ? SyncKind::ToolChange : SyncKind::ProgramStop;
        job_sync_segment_ = seg_index;
        ++job_cursor_;
        return false;
    }

    std::vector<std::string> lines;
    bool sync = false;
    if (!emitOutputsFor(seg, lines, &sync)) {
        if (sync) {
            job_produce_blocked_ = true;
            job_sync_kind_ = SyncKind::SpindleChange;
            job_sync_segment_ = seg_index;
            return false;
        }
    }
    for (auto& l : lines) job_lines_.push_back({l, false, {}});
    lines.clear();

    std::vector<std::array<double, 4>> arc_points; // Set for a G2 / G3 line
    double arc_sag = 0.0;
    auto pushMotion = [&](const std::vector<std::string>& ls, const std::array<double, 4>& end, size_t last_index,
                          size_t gline) {
        bool marked = false;
        for (const auto& l : ls) {
            PendingLine pl{l, false, {}};
            if (!marked && l != "G17" && l != "G18" && l != "G19") {
                pl.is_move = true;
                pl.move.segment_index = last_index;
                pl.move.gcode_line = gline;
                pl.move.from_program = from_program;
                pl.move.start = job_pos_;
                pl.move.end = end;
                pl.move.line_number = emitter_.lastLineNumber();
                if (arc_points.size() > 1) {
                    pl.move.arc_points = arc_points;
                    pl.move.arc_sag_mm = arc_sag;
                    pl.move.segment_index = last_index + 2 - arc_points.size(); // First chord
                }
                marked = true;
            }
            job_lines_.push_back(pl);
        }
    };

    if (seg.type == SegmentType::Dwell) {
        double secs = seg.dwell_seconds;
        if (job_.plasma && seg.torch_active && job_.pierce_delay_override_s > 0.01f) secs = job_.pierce_delay_override_s;
        emitter_.dwell(secs, lines);
        for (auto& l : lines) job_lines_.push_back({l, false, {}});
        ++job_cursor_;
        return true;
    }
    if (!isMotion(seg.type)) {
        ++job_cursor_;
        return true;
    }

    const double s_laser = -1.0;
    // Arc: a complete run of chords becomes one G2 / G3
    const auto& prog = *job_.program;
    if (seg.arc_index != kNoArc && from_program && job_.arcs_allowed && emitter_.dialect().arcs &&
        seg.arc_index < prog.arcs.size()) {
        const auto& arc = prog.arcs[seg.arc_index];
        const size_t pi = jobProgramIndex(job_cursor_);
        const size_t last = static_cast<size_t>(arc.first_segment) + arc.segment_count - 1;
        bool whole = pi == arc.first_segment && arc.segment_count > 0 && last < prog.segments.size() &&
                     !(job_.laser && job_.laser_lock_z && arc.plane != 17);
        for (size_t i = pi; whole && i <= last; ++i) {
            if (prog.segments[i].arc_index != seg.arc_index || prog.segments[i].machine_axes_mask != 0) whole = false;
        }
        if (whole) {
            const auto end = resolveSegment(prog.segments[last]);
            std::array<double, 3> ijk{};
            for (size_t a = 0; a < 3; ++a) ijk[a] = arc.center_mm[a] - seg.start_xyza_mm[a];
            // GRBL rejects arcs whose end radius differs (error:33): check it with the rounded numbers sent
            const auto sfw = toFirmware(job_pos_, true);
            const auto efw = toFirmware(end, true);
            const size_t u = (arc.plane == 19) ? 1 : 0;
            const size_t v = (arc.plane == 17) ? 1 : 2;
            auto rnd = [](double x, int dec) { return std::atof(GcodeEmitter::num(x, dec).c_str()); };
            const double su = rnd(sfw[u], 3), sv = rnd(sfw[v], 3);
            const double cu = su + rnd(ijk[u], 4), cv = sv + rnd(ijk[v], 4);
            const double r0 = std::hypot(su - cu, sv - cv);
            const double r1 = std::hypot(rnd(efw[u], 3) - cu, rnd(efw[v], 3) - cv);
            const double tol = std::max(emitter_.dialect().arc_radius_tolerance_mm, 0.0005 * r0);
            if (std::abs(r0 - r1) <= tol && r0 > 0.001) {
                emitter_.arc(arc.clockwise, arc.plane, efw, ijk, prog.segments[last].feed_mm_min, lines, s_laser);
                // Chord vertices for the tracker; the arc bulges at most L^2 / (8 r) from a chord of length L
                arc_points.clear();
                arc_points.push_back(job_pos_);
                double max_l2 = 0.0;
                for (size_t i = pi; i <= last; ++i) {
                    const auto p = (i == last) ? end : resolveSegment(prog.segments[i]);
                    double l2 = 0.0;
                    for (size_t a = 0; a < 4; ++a) l2 += (p[a] - arc_points.back()[a]) * (p[a] - arc_points.back()[a]);
                    max_l2 = std::max(max_l2, l2);
                    arc_points.push_back(p);
                }
                arc_sag = max_l2 / (8.0 * r0);
                pushMotion(lines, end, last, prog.segments[last].gcode_line);
                job_pos_ = end;
                job_cursor_ = jobSourceSize() - (prog.segments.size() - (last + 1));
                return true;
            }
        }
    }

    const auto target = resolveSegment(seg);
    const auto fw = toFirmware(target, true);
    if (seg.type == SegmentType::RapidMove) {
        emitter_.rapid(fw, lines);
    } else {
        emitter_.linear(fw, seg.feed_mm_min, lines, s_laser);
    }
    if (!lines.empty()) pushMotion(lines, target, seg_index, seg.gcode_line);
    job_pos_ = target;
    ++job_cursor_;
    return true;
}

void FirmwarePlannedPath::pumpJob(Clock::time_point now) {
    switch (job_phase_) {
        case JobPhase::Preamble: {
            if (job_preamble_waiting_) return;
            if (job_preamble_pos_ < job_preamble_.size()) {
                if (protocol_.unacknowledged() != 0) return;
                const auto& l = job_preamble_[job_preamble_pos_++];
                job_preamble_ticket_ = protocol_.sendLine(l.text);
                job_preamble_waiting_ = job_preamble_ticket_ != 0;
                return;
            }
            job_phase_ = JobPhase::Streaming;
            [[fallthrough]];
        }
        case JobPhase::Streaming: {
            if (job_held_) return;
            for (int guard = 0; guard < 400; ++guard) {
                if (job_lines_.empty()) {
                    if (job_produce_blocked_) break;
                    if (!produceJobLines() && job_lines_.empty()) {
                        if (job_produce_blocked_) break;
                        continue;
                    }
                    continue;
                }
                auto& front = job_lines_.front();
                if (front.text.size() > emitter_.dialect().max_line_chars) {
                    // The emitter already left out the line number; the board would refuse it (error 11 / 14)
                    log_->add(LogDir::Error, "Line too long for the board: " + front.text);
                    PathEvent e;
                    e.kind = PathEvent::Kind::Message;
                    e.ok = false;
                    std::string where;
                    if (front.is_move) where = " (program line " + std::to_string(front.move.gcode_line + 1) + ")";
                    e.text = "Job stopped: a line is longer than the board accepts (" +
                             std::to_string(front.text.size()) + " of " +
                             std::to_string(emitter_.dialect().max_line_chars) + " characters)" + where;
                    emit(e);
                    job_lines_.clear();
                    stop();
                    return;
                }
                if (!protocol_.canSend(front.text.size())) break;
                const uint32_t ticket = protocol_.sendLine(front.text);
                if (ticket == 0) return; // Link failure is reported by tick()
                if (front.is_move) {
                    front.move.ticket = ticket;
                    sent_moves_.push_back(front.move);
                }
                job_lines_.pop_front();
            }
            if (job_lines_.empty() && job_produce_blocked_) {
                if (!protocol_.canSend(emitter_.dialect().wait_idle.size())) return;
                job_drain_ticket_ = protocol_.sendLine(emitter_.dialect().wait_idle);
                job_drain_acked_ = false;
                status_since_ack_ = false;
                job_phase_ = (job_sync_segment_ == SIZE_MAX) ? JobPhase::Draining : JobPhase::SyncDrain;
            }
            return;
        }
        case JobPhase::Draining:
        case JobPhase::SyncDrain: {
            if (!job_drain_acked_ || !status_since_ack_ || fw_state_ != "Idle") return;
            if (!sent_moves_.empty()) {
                tracker_index_ = sent_moves_.size() - 1;
                const auto& m = sent_moves_.back();
                fb_.active_segment = m.lastSegment();
                fb_.active_line = m.gcode_line;
                fb_.segment_progress = 1.0f;
            }
            if (job_phase_ == JobPhase::Draining) {
                job_finish_lines_.clear();
                job_finish_lines_.push_back(job_.ends_with_program_end ? "M5M9" : "M5");
                job_phase_ = JobPhase::Finishing;
                return;
            }
            PathEvent e;
            e.kind = PathEvent::Kind::JobSyncPoint;
            e.segment_index = job_sync_segment_;
            e.code = static_cast<int>(job_sync_kind_);
            e.spindle_on = job_vfd_on_;
            e.spindle_ccw = job_vfd_ccw_;
            e.spindle_rpm = job_vfd_rpm_;
            emit(e);
            if (job_sync_kind_ == SyncKind::SpindleChange) {
                job_phase_ = JobPhase::SyncWait;
            } else {
                job_phase_ = JobPhase::Done;
                fb_.job_active = false;
                op_ = Op::None;
            }
            return;
        }
        case JobPhase::Finishing: {
            if (protocol_.unacknowledged() != 0) return;
            if (!job_finish_lines_.empty()) {
                protocol_.sendLine(job_finish_lines_.front());
                job_finish_lines_.pop_front();
                return;
            }
            job_phase_ = JobPhase::Done;
            fb_.job_active = false;
            op_ = Op::None;
            if (job_.program) fb_.active_segment = job_.program->segments.size();
            fb_.segment_progress = 1.0f;
            PathEvent e;
            e.kind = PathEvent::Kind::JobFinished;
            e.ok = true;
            emit(e);
            return;
        }
        default:
            (void)now;
            return;
    }
}

void FirmwarePlannedPath::updateTracker() {
    if (sent_moves_.empty()) return;
    while (acked_moves_ < sent_moves_.size() && sent_moves_[acked_moves_].ticket <= last_acked_ticket_) ++acked_moves_;
    const size_t blocks = std::max<size_t>(1, fw_.planner_blocks);
    // Every acknowledged line still holds at most one planner block: anything older has finished
    const size_t lower = acked_moves_ > blocks ? acked_moves_ - blocks : 0;
    const auto& pos = fb_.machine_pos_mm;
    // How far the machine position is off a sent move (beyond its tolerance), where along the move it projects
    // (t, 0..1) and, for an arc, on which chord (sub) and where along that chord (t_sub)
    const size_t axes = track_xy_only_ ? 2 : 4;
    auto onLine = [&](const std::array<double, 4>& s, const std::array<double, 4>& e, double& t) {
        // Following by XY only (board THC moves Z): a move without XY travel (pierce height, retract) is still
        // measured along Z for its progress, and the position is on it while XY stays at its point
        size_t proj_axes = axes;
        if (axes == 2 && std::hypot(e[0] - s[0], e[1] - s[1]) < 1e-6) proj_axes = 4;
        double len2 = 0.0, proj = 0.0;
        for (size_t a = 0; a < proj_axes; ++a) {
            const double d = e[a] - s[a];
            len2 += d * d;
            proj += (pos[a] - s[a]) * d;
        }
        t = len2 > 1e-12 ? std::clamp(proj / len2, 0.0, 1.0) : 1.0;
        double dist2 = 0.0;
        for (size_t a = 0; a < axes; ++a) {
            const double c = s[a] + (e[a] - s[a]) * t - pos[a];
            dist2 += c * c;
        }
        return std::sqrt(dist2);
    };
    auto onMove = [&](const SentMove& mv, double& t, size_t& sub, double& t_sub, size_t from_sub) {
        sub = 0;
        if (mv.arc_points.size() < 2) {
            const double d = onLine(mv.start, mv.end, t);
            t_sub = t;
            return d;
        }
        // The nearest chord at or after the one the machine was on (the machine only goes forward). A full circle
        // starts and ends at the same point: this keeps a finished circle finished and a starting one at its start.
        const size_t n = mv.arc_points.size() - 1;
        double best = 1e300;
        for (size_t k = std::min(from_sub, n - 1); k < n; ++k) {
            double tk = 0.0;
            const double d = onLine(mv.arc_points[k], mv.arc_points[k + 1], tk);
            if (d < best - 1e-9) {
                best = d;
                sub = k;
                t_sub = tk;
            }
        }
        t = (static_cast<double>(sub) + t_sub) / static_cast<double>(n);
        return std::max(0.0, best - mv.arc_sag_mm);
    };
    constexpr double kOnPathMm = 0.05;
    // Moves below the lower bound have certainly finished
    while (tracker_index_ + 1 < sent_moves_.size() && tracker_index_ + 1 < lower) {
        ++tracker_index_;
        tracker_sub_ = 0;
    }
    double t = 0.0, t_sub = 0.0;
    size_t sub = 0;
    const double d_cur = onMove(sent_moves_[tracker_index_], t, sub, t_sub, tracker_sub_);
    if (!(d_cur < kOnPathMm && t < 0.999)) {
        // Not (or no longer) inside the current move: the machine is on the first later move whose path it is on.
        // Status reports come every 100 ms, so short moves can be passed without ever being seen at their end.
        const size_t limit = std::min(sent_moves_.size(), tracker_index_ + 2 * blocks + 2);
        for (size_t i = tracker_index_ + 1; i < limit; ++i) {
            double ti = 0.0, ti_sub = 0.0;
            size_t si = 0;
            if (onMove(sent_moves_[i], ti, si, ti_sub, 0) < kOnPathMm) {
                // At the very end of one move and the start of the next: count it as the next one
                tracker_index_ = i;
                t = ti;
                sub = si;
                t_sub = ti_sub;
                break;
            }
        }
    }
    const auto& m = sent_moves_[tracker_index_];
    tracker_sub_ = sub;
    fb_.active_segment = m.segment_index + sub;
    fb_.active_line = m.gcode_line;
    fb_.segment_progress = static_cast<float>(t_sub);
    fb_.progress_is_estimate = !fw_.reports_line_numbers;
    // Keep the ring short
    while (tracker_index_ > 256) {
        sent_moves_.pop_front();
        --tracker_index_;
        if (acked_moves_ > 0) --acked_moves_;
    }
}

} // namespace rnr::core::motion
