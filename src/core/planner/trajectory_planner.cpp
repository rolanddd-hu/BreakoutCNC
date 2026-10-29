#include "core/planner/trajectory_planner.h"
#include <cmath>
#include <algorithm>

namespace rnr::planner {

namespace {

// Corner tolerance used by the junction-deviation cornering model (mm). Larger = faster corners.
constexpr double kJunctionDeviationMm = 0.02;
// Upper bound on segments examined per planning window
constexpr size_t kMaxWindowSegments = 4000;

bool isCutType(core::SegmentType t) noexcept {
    return t == core::SegmentType::CutLinear || t == core::SegmentType::CutArcCW || t == core::SegmentType::CutArcCCW;
}

struct Phase {
    double duration = 0.0;
    double v0       = 0.0;
    double acc      = 0.0; // signed
};

// Velocity profile for a segment of length L entering at v_in, cruising at <= v_c and leaving at v_out.
std::array<Phase, 3> buildProfile(double L, double v_in, double v_c, double v_out, double a, double d, int& n_phases) {
    std::array<Phase, 3> ph{};
    n_phases = 0;
    v_c = std::max(v_c, 1e-6);
    v_out = std::min(v_out, std::max(v_c, v_in));

    if (v_in <= v_c) {
        const double s_acc = (v_c * v_c - v_in * v_in) / (2.0 * a);
        const double s_dec = (v_c * v_c - v_out * v_out) / (2.0 * d);
        double vp = v_c;
        if (s_acc + s_dec > L) {
            vp = std::sqrt(std::max(0.0, (2.0 * a * d * L + d * v_in * v_in + a * v_out * v_out) / (a + d)));
        }
        if (vp + 1e-9 < std::max(v_in, v_out)) {
            // Infeasible (should not happen after the planner passes): uniform ramp between the two speeds
            const double vm = std::max(1e-6, 0.5 * (v_in + v_out));
            ph[n_phases++] = {L / vm, v_in, (v_out - v_in) / (L / vm)};
            return ph;
        }
        const double t1 = (vp - v_in) / a;
        const double t3 = (vp - v_out) / d;
        const double s1 = 0.5 * (v_in + vp) * t1;
        const double s3 = 0.5 * (vp + v_out) * t3;
        const double s2 = std::max(0.0, L - s1 - s3);
        if (t1 > 1e-12) ph[n_phases++] = {t1, v_in, a};
        if (s2 > 1e-12) ph[n_phases++] = {s2 / vp, vp, 0.0};
        if (t3 > 1e-12) ph[n_phases++] = {t3, vp, -d};
    } else {
        // Entering faster than the cruise limit (e.g. feed override was lowered): slow down first
        const double s1 = (v_in * v_in - v_c * v_c) / (2.0 * d);
        const double s3 = (v_c * v_c - v_out * v_out) / (2.0 * d);
        if (s1 + s3 <= L) {
            const double t1 = (v_in - v_c) / d;
            const double t3 = (v_c - v_out) / d;
            const double s2 = L - s1 - s3;
            if (t1 > 1e-12) ph[n_phases++] = {t1, v_in, -d};
            if (s2 > 1e-12) ph[n_phases++] = {s2 / v_c, v_c, 0.0};
            if (t3 > 1e-12) ph[n_phases++] = {t3, v_c, -d};
        } else {
            const double vm = std::max(1e-6, 0.5 * (v_in + v_out));
            ph[n_phases++] = {L / vm, v_in, (v_out - v_in) / (L / vm)};
        }
    }
    if (n_phases == 0) {
        ph[n_phases++] = {L / v_c, v_c, 0.0};
    }
    return ph;
}

double profileDistance(const std::array<Phase, 3>& ph, int n, double t) {
    double s = 0.0;
    for (int i = 0; i < n; ++i) {
        if (t <= ph[i].duration) {
            return s + ph[i].v0 * t + 0.5 * ph[i].acc * t * t;
        }
        s += ph[i].v0 * ph[i].duration + 0.5 * ph[i].acc * ph[i].duration * ph[i].duration;
        t -= ph[i].duration;
    }
    return s;
}

double profileDuration(const std::array<Phase, 3>& ph, int n) {
    double t = 0.0;
    for (int i = 0; i < n; ++i) t += ph[i].duration;
    return t;
}

} // namespace

// ============================================================================
// StreamingPlanner
// ============================================================================
StreamingPlanner::StreamingPlanner(const core::MachineConfig& cfg,
                                   const std::array<double, 4>& start_pos_mm,
                                   const PlannerOptions& opts,
                                   const FrameContext& frame)
    : cfg_(cfg), opts_(opts), frame_(frame)
{
    pos_ = start_pos_mm;
    laser_locked_z_ = start_pos_mm[2];
    emitted_pos_ = pos_;
    for (size_t a = 0; a < 4; ++a) {
        accum_steps_[a] = static_cast<int32_t>(std::llround(pos_[a] * cfg_.axes[a].steps_per_mm));
        axis_vmax_[a]   = TrajectoryPlanner::axisVelocityLimit(cfg_, a);
    }
    if (const int master = cfg_.gantryMasterAxis(); master >= 0) {
        // The slaved A motor follows its master step-for-step, so the master must respect both motors' limits
        axis_vmax_[master] = std::min(axis_vmax_[master], axis_vmax_[3]);
    }
    t_next_ = opts_.slice_dt_sec;
}

void StreamingPlanner::setSource(std::vector<core::ToolpathSegment> prefix,
                                 std::shared_ptr<const std::vector<core::ToolpathSegment>> program,
                                 size_t first_index,
                                 bool prefix_from_program)
{
    prefix_        = std::move(prefix);
    program_       = std::move(program);
    program_first_ = first_index;
    prefix_from_program_ = prefix_from_program;
    next_          = 0;
}

void StreamingPlanner::setInitialMotion(double v_mm_s, const std::array<double, 4>& unit_dir) {
    v_entry_       = std::max(0.0, v_mm_s);
    prev_unit_     = unit_dir;
    have_prev_dir_ = v_entry_ > 1e-9;
}

bool StreamingPlanner::isProgramSegment(const core::ToolpathSegment* s) const noexcept {
    if (prefix_from_program_) return true;
    if (prefix_.empty()) return true;
    return !(s >= prefix_.data() && s < prefix_.data() + prefix_.size());
}

size_t StreamingPlanner::sourceSize() const noexcept {
    size_t n = prefix_.size();
    if (program_ && program_first_ < program_->size()) {
        n += program_->size() - program_first_;
    }
    return n;
}

const core::ToolpathSegment& StreamingPlanner::segAt(size_t i) const {
    if (i < prefix_.size()) return prefix_[i];
    return (*program_)[program_first_ + (i - prefix_.size())];
}

bool StreamingPlanner::finished() const noexcept {
    return next_ >= sourceSize();
}

size_t StreamingPlanner::nextProgramIndex() const noexcept {
    if (next_ < prefix_.size()) return program_first_;
    return program_first_ + (next_ - prefix_.size());
}

bool StreamingPlanner::isMotion(const core::ToolpathSegment& s) const noexcept {
    return s.type == core::SegmentType::RapidMove || isCutType(s.type);
}

uint8_t StreamingPlanner::outputsFor(const core::ToolpathSegment& s) const noexcept {
    uint8_t out_mask = s.outputs_mask;
    if (opts_.dry_run_no_torch) {
        out_mask &= ~core::OutputBit::OUT1_TorchOrSpindle;
    }
    if (cfg_.operation_mode == core::OperationMode::Laser && !cfg_.is_mill) {
        if (s.type == core::SegmentType::RapidMove) {
            // Laser is strictly forced OFF during G0 rapid positioning moves
            out_mask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        } else if ((out_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0 && cfg_.laser.auto_air_assist) {
            // Auto Air Assist: energize OUT3 air solenoid whenever laser is cutting
            out_mask |= core::OutputBit::OUT3_FloodCoolant;
        }
    }
    return out_mask;
}

std::array<double, 4> StreamingPlanner::targetFor(const core::ToolpathSegment& s) const {
    auto t = TrajectoryPlanner::transformTarget(s, frame_);
    if (cfg_.operation_mode == core::OperationMode::Laser && !cfg_.is_mill && cfg_.laser.ignore_z_motion) {
        t[2] = laser_locked_z_; // Lock focal distance across entire program, suppress Z motion
    }
    return t;
}

bool StreamingPlanner::buildMotionItem(const core::ToolpathSegment& s, const std::array<double, 4>& from, MotionItem& m) const {
    m.seg   = &s;
    m.start = from;
    m.end   = targetFor(s);
    double L2 = 0.0;
    std::array<double, 4> d{};
    for (size_t a = 0; a < 4; ++a) {
        d[a] = m.end[a] - m.start[a];
        L2 += d[a] * d[a];
    }
    m.length = std::sqrt(L2);
    if (m.length <= 1e-6) return false;
    for (size_t a = 0; a < 4; ++a) m.unit[a] = d[a] / m.length;

    // Path velocity / acceleration limits imposed by every moving motor: v_a = v * |u_a|
    double v_lim = 1e9, a_lim = 1e9, d_lim = 1e9;
    for (size_t a = 0; a < 4; ++a) {
        const double ua = std::abs(m.unit[a]);
        if (ua > 1e-9) {
            v_lim = std::min(v_lim, axis_vmax_[a] / ua);
            a_lim = std::min(a_lim, std::max(1.0, cfg_.axes[a].acceleration_mm_s2) / ua);
            d_lim = std::min(d_lim, std::max(1.0, cfg_.axes[a].deceleration_mm_s2) / ua);
        }
    }
    const double feed_scale = std::clamp(static_cast<double>(opts_.feed_override_pct) / 100.0, 0.1, 3.0);
    m.rapid = (s.type == core::SegmentType::RapidMove);
    if (m.rapid) {
        // Rapids run at the motor limit; override can only slow them down, never exceed the limit
        m.v_max = v_lim * std::min(1.0, feed_scale);
    } else {
        // Programmed feed * override, never above what the motors are configured for
        m.v_max = std::clamp((s.feed_mm_min / 60.0) * feed_scale, 0.05, v_lim);
    }
    m.accel    = a_lim;
    m.decel    = d_lim;
    m.out_mask = outputsFor(s);
    return true;
}

PlannedSlice StreamingPlanner::makeSliceFromTarget(const std::array<double, 4>& target,
                                                   const core::ToolpathSegment& meta,
                                                   uint8_t out_mask, float progress)
{
    PlannedSlice ps{};
    ps.segment_index      = meta.segment_index;
    ps.gcode_line         = meta.gcode_line;
    ps.segment_progress   = progress;
    ps.slice.outputs_mask = out_mask;
    ps.tool_number        = meta.tool_number;
    ps.spindle_rpm        = meta.spindle_rpm;
    ps.workspace_index    = meta.workspace_index;
    ps.target_pos_mm      = target;
    ps.from_program       = isProgramSegment(&meta);
    ps.tlo_tool           = meta.tlo_tool;
    ps.tlo_mm             = TrajectoryPlanner::toolLengthFor(meta, frame_);
    for (size_t a = 0; a < 4; ++a) {
        const int32_t target_steps = static_cast<int32_t>(std::llround(target[a] * cfg_.axes[a].steps_per_mm));
        ps.slice.delta_steps[a] = target_steps - accum_steps_[a];
        accum_steps_[a] = target_steps;
    }
    if (const int master = cfg_.gantryMasterAxis(); master >= 0) {
        ps.slice.delta_steps[3] = ps.slice.delta_steps[master];
    }
    emitted_pos_ = target;
    return ps;
}

void StreamingPlanner::emitSliceAt(const MotionItem& m, double s_mm, std::deque<PlannedSlice>& out, size_t& count) {
    const double frac = std::clamp(s_mm / m.length, 0.0, 1.0);
    std::array<double, 4> p{};
    for (size_t a = 0; a < 4; ++a) {
        p[a] = m.start[a] + (m.end[a] - m.start[a]) * frac;
    }
    out.push_back(makeSliceFromTarget(p, *m.seg, m.out_mask, static_cast<float>(frac)));
    ++count;
}

void StreamingPlanner::flushToCommittedPosition(std::deque<PlannedSlice>& out, size_t& count) {
    // Land exactly on the committed position before a hard stop (a slightly shorter final slice).
    bool differs = false;
    for (size_t a = 0; a < 4; ++a) {
        if (static_cast<int32_t>(std::llround(pos_[a] * cfg_.axes[a].steps_per_mm)) != accum_steps_[a]) {
            differs = true;
        }
    }
    if (differs && last_motion_meta_) {
        out.push_back(makeSliceFromTarget(pos_, *last_motion_meta_, last_motion_mask_, 1.0f));
        ++count;
    }
    emitted_pos_ = pos_;
    t_next_  = opts_.slice_dt_sec;
    v_entry_ = 0.0;
    have_prev_dir_ = false;
}

void StreamingPlanner::emitSegmentProfile(const MotionItem& m, double v_in, double v_out,
                                          std::deque<PlannedSlice>& out, size_t& count)
{
    int n = 0;
    const auto ph = buildProfile(m.length, v_in, m.v_max, v_out, m.accel, m.decel, n);
    const double T = profileDuration(ph, n);
    const double dt = opts_.slice_dt_sec;
    while (t_next_ <= T + 1e-9) {
        const double s = (t_next_ >= T - 1e-9) ? m.length : profileDistance(ph, n, t_next_);
        emitSliceAt(m, s, out, count);
        t_next_ += dt;
    }
    t_next_ -= T; // Carry the remaining slice time into the next segment (continuous motion)
    pos_ = m.end;
    last_motion_meta_ = m.seg;
    last_motion_mask_ = m.out_mask;
}

void StreamingPlanner::emitNonMotion(const core::ToolpathSegment& seg, std::deque<PlannedSlice>& out, size_t& count) {
    const double dt = opts_.slice_dt_sec;
    uint8_t out_mask = outputsFor(seg);
    const bool is_laser = (cfg_.operation_mode == core::OperationMode::Laser && !cfg_.is_mill);
    const bool is_plasma = (cfg_.operation_mode == core::OperationMode::Plasma && !cfg_.is_mill);

    auto marker = [&](float progress) {
        PlannedSlice ps{};
        ps.slice.delta_steps  = {0, 0, 0, 0};
        ps.slice.outputs_mask = out_mask;
        ps.segment_index      = seg.segment_index;
        ps.gcode_line         = seg.gcode_line;
        ps.segment_progress   = progress;
        ps.target_pos_mm      = pos_;
        ps.tool_number        = seg.tool_number;
        ps.spindle_rpm        = seg.spindle_rpm;
        ps.workspace_index    = seg.workspace_index;
        ps.from_program       = isProgramSegment(&seg);
        ps.tlo_tool           = seg.tlo_tool;
        ps.tlo_mm             = TrajectoryPlanner::toolLengthFor(seg, frame_);
        return ps;
    };

    if (seg.type == core::SegmentType::ToolChange) {
        if (cfg_.tool_change.mode == core::ToolChangeMode::IgnoreM6) {
            return;
        }
        out_mask = 0; // Spindle off for any tool change
        if (cfg_.tool_change.mode == core::ToolChangeMode::ManualPrompt) {
            PlannedSlice ps = marker(1.0f);
            ps.is_tool_change   = true;
            ps.tool_description = seg.tool_description;
            out.push_back(std::move(ps));
            ++count;
        } else if (cfg_.tool_change.mode == core::ToolChangeMode::AutoAtc) {
            // Auto Toolchanger sequence: dwell for ATC swap duration then auto-continue
            const double swap_sec = std::max(0.2, cfg_.tool_change.atc_dwell_seconds);
            const int num_slices = std::max(1, static_cast<int>(std::ceil(swap_sec / dt)));
            for (int i = 1; i <= num_slices; ++i) {
                PlannedSlice ps = marker(static_cast<float>(i) / static_cast<float>(num_slices));
                ps.is_dwell         = true;
                ps.tool_description = seg.tool_description;
                out.push_back(std::move(ps));
                ++count;
            }
        }
        return;
    }

    if (seg.type == core::SegmentType::ProgramStop) {
        PlannedSlice ps = marker(1.0f);
        ps.is_program_stop = true;
        out.push_back(std::move(ps));
        ++count;
        return;
    }

    if (seg.type == core::SegmentType::Dwell) {
        double dwell_sec = seg.dwell_seconds;
        // The operator pierce-delay override only applies to plasma torch pierce dwells
        if (is_plasma && opts_.pierce_delay_override_s > 0.01f && seg.torch_active) {
            dwell_sec = static_cast<double>(opts_.pierce_delay_override_s);
        }
        if (is_laser && dwell_sec <= 0.05) {
            return; // Laser: pierce delays are bypassed (instant cut initiation)
        }
        if (dwell_sec <= 1e-6) {
            return;
        }
        const int num_slices = std::max(1, static_cast<int>(std::ceil(dwell_sec / dt)));
        for (int i = 1; i <= num_slices; ++i) {
            PlannedSlice ps = marker(static_cast<float>(i) / static_cast<float>(num_slices));
            ps.is_dwell = true;
            out.push_back(std::move(ps));
            ++count;
        }
    }
}

size_t StreamingPlanner::plan(std::deque<PlannedSlice>& out, size_t max_slices) {
    size_t count = 0;
    const size_t total = sourceSize();
    std::vector<MotionItem> win;
    std::vector<double> junction;

    while (count < max_slices && next_ < total) {
        const auto& seg = segAt(next_);
        if (!isMotion(seg)) {
            flushToCommittedPosition(out, count);
            emitNonMotion(seg, out, count);
            ++next_;
            continue;
        }

        // ---- Collect a look-ahead window of smoothly connected motion segments ----
        win.clear();
        junction.clear();
        bool hard_end = false;
        double window_len = 0.0;
        double v_peak = 0.0;
        double min_decel = 1e9;
        std::array<double, 4> from = pos_;
        size_t idx = next_;
        while (idx < total && win.size() < kMaxWindowSegments) {
            const auto& s = segAt(idx);
            if (!isMotion(s)) {
                hard_end = true;
                break;
            }
            MotionItem m{};
            if (!buildMotionItem(s, from, m)) {
                // Zero-length move: skip it (the cursor advance below steps over it)
                if (win.empty()) {
                    ++next_;
                }
                ++idx;
                continue;
            }
            if (!win.empty()) {
                const auto& p = win.back();
                if (p.out_mask != m.out_mask || p.rapid != m.rapid) {
                    hard_end = true; // Outputs change or rapid<->feed: come to a stop between them
                    break;
                }
            }
            win.push_back(m);
            from = m.end;
            window_len += m.length;
            v_peak    = std::max(v_peak, m.v_max);
            min_decel = std::min(min_decel, m.decel);
            ++idx;
            const double brake = (v_peak * v_peak) / (2.0 * min_decel);
            if (window_len >= 2.0 * brake + 1.0 && win.size() >= 2) {
                break;
            }
        }
        if (idx >= total) hard_end = true;
        if (win.empty()) {
            continue; // Only zero-length moves were skipped
        }

        // If the previous committed run ended at a boundary that requires a stop, the new run starts from rest
        if (!have_prev_dir_) {
            v_entry_ = 0.0;
        } else {
            // Junction between the last committed segment and the first window segment
            double c = 0.0;
            for (size_t a = 0; a < 4; ++a) c += prev_unit_[a] * win[0].unit[a];
            if (c < -0.999999) v_entry_ = 0.0;
        }

        // Junction speed limits inside the window (junction-deviation cornering model)
        const size_t n = win.size();
        junction.assign(n, 0.0);
        junction[0] = v_entry_;
        for (size_t i = 1; i < n; ++i) {
            double c = 0.0;
            for (size_t a = 0; a < 4; ++a) c += win[i - 1].unit[a] * win[i].unit[a];
            double vj = 0.0;
            const double vcap = std::min(win[i - 1].v_max, win[i].v_max);
            if (c > 0.999999) {
                vj = vcap;
            } else if (c > -0.999999) {
                const double sin_half = std::sqrt(std::max(0.0, 0.5 * (1.0 + c)));
                const double a_j = std::min(win[i - 1].accel, win[i].accel);
                vj = std::sqrt(std::max(0.0, a_j * kJunctionDeviationMm * sin_half / std::max(1e-9, 1.0 - sin_half)));
                vj = std::min(vj, vcap);
            }
            junction[i] = vj;
        }

        // Backward pass: from standstill at the end of the window
        std::vector<double> v_exit(n, 0.0);
        std::vector<double> v_entry_max(n, 0.0);
        double next_entry = 0.0;
        for (size_t k = n; k-- > 0;) {
            v_exit[k] = next_entry;
            const double reach = std::sqrt(v_exit[k] * v_exit[k] + 2.0 * win[k].decel * win[k].length);
            v_entry_max[k] = std::min({junction[k], win[k].v_max, reach});
            if (k == 0) v_entry_max[k] = std::min(v_entry_max[k], std::max(v_entry_, 0.0));
            next_entry = v_entry_max[k];
        }

        // Commit segments while enough distance remains in the window to stop (unless the window ends hard)
        size_t commit = n;
        if (!hard_end) {
            const double brake = (v_peak * v_peak) / (2.0 * min_decel);
            double remaining = window_len;
            commit = 0;
            while (commit < n && (remaining - win[commit].length) >= brake) {
                remaining -= win[commit].length;
                ++commit;
            }
            commit = std::max<size_t>(commit, 1);
        }

        // Forward pass + emission
        double v_in = v_entry_;
        for (size_t k = 0; k < commit; ++k) {
            const double reach = std::sqrt(v_in * v_in + 2.0 * win[k].accel * win[k].length);
            const double v_out = std::min(v_exit[k], reach);
            emitSegmentProfile(win[k], v_in, v_out, out, count);
            v_in = v_out;
            prev_unit_ = win[k].unit;
            have_prev_dir_ = true;
        }
        v_entry_ = v_in;

        // Advance the source cursor past the committed segments (and any zero-length moves between them)
        const core::ToolpathSegment* last_committed = win[commit - 1].seg;
        while (next_ < total && &segAt(next_) != last_committed) ++next_;
        ++next_;

        if (hard_end && commit == n) {
            flushToCommittedPosition(out, count);
        }
    }

    if (next_ >= total) {
        flushToCommittedPosition(out, count);
    }
    return count;
}

// ============================================================================
// TrajectoryPlanner (one-shot helpers)
// ============================================================================
double TrajectoryPlanner::axisVelocityLimit(const core::MachineConfig& cfg, size_t axis) {
    const auto& ax = cfg.axes[axis];
    double v = std::max(0.1, ax.max_velocity_mm_s);
    const double spm = std::max(1e-6, ax.steps_per_mm);
    const double pulse_hz = std::max<double>(1.0, cfg.pulse_rate_khz) * 1000.0;
    // Stay 5% under the maximum step pulse rate of the board
    v = std::min(v, 0.95 * pulse_hz / spm);
    return v;
}

std::array<double, 4> TrajectoryPlanner::transformTarget(const core::ToolpathSegment& seg, const FrameContext& frame) {
    std::array<double, 4> t = seg.end_xyza_mm;
    if (!frame.enabled) {
        return t;
    }
    const uint8_t pw = std::min<uint8_t>(frame.plan_workspace, 5);
    const uint8_t sw = std::min<uint8_t>(seg.workspace_index, 5);
    for (size_t a = 0; a < 4; ++a) {
        if ((seg.machine_axes_mask >> a) & 1U) {
            // Machine coordinates (G53 / G28) address the spindle: no tool length offset
            t[a] = seg.machine_target_mm[a] - frame.workspace_offsets[pw][a];
        } else {
            t[a] = seg.end_xyza_mm[a] + frame.workspace_offsets[sw][a] - frame.workspace_offsets[pw][a];
            if (a == 2) t[a] += toolLengthFor(seg, frame);
        }
    }
    return t;
}

double TrajectoryPlanner::toolLengthFor(const core::ToolpathSegment& seg, const FrameContext& frame) {
    if (!frame.enabled || !frame.apply_tool_lengths) return 0.0;
    if (seg.tlo_tool == core::kTloInherit) return frame.inherit_tlo_mm;
    return (seg.tlo_tool < frame.tool_lengths.size()) ? frame.tool_lengths[seg.tlo_tool] : 0.0;
}

std::deque<PlannedSlice> TrajectoryPlanner::sliceProgram(
    const std::vector<core::ToolpathSegment>& segments,
    const core::MachineConfig& cfg,
    float feed_override_pct,
    float pierce_delay_override_s,
    bool dry_run_no_torch,
    double slice_dt_sec)
{
    std::deque<PlannedSlice> out;
    if (segments.empty()) return out;

    PlannerOptions opts;
    opts.feed_override_pct       = feed_override_pct;
    opts.pierce_delay_override_s = pierce_delay_override_s;
    opts.dry_run_no_torch        = dry_run_no_torch;
    opts.slice_dt_sec            = slice_dt_sec;

    StreamingPlanner sp(cfg, segments.front().start_xyza_mm, opts);
    sp.setSource(segments, nullptr, 0);
    while (!sp.finished()) {
        sp.plan(out, 4096);
    }
    return out;
}

std::deque<PlannedSlice> TrajectoryPlanner::sliceLinearMove(
    const std::array<double, 4>& start_mm,
    const std::array<double, 4>& end_mm,
    double feed_mm_min,
    const core::MachineConfig& cfg,
    uint8_t outputs_mask,
    double slice_dt_sec)
{
    core::ToolpathSegment seg{};
    seg.type          = core::SegmentType::CutLinear;
    seg.segment_index = 0;
    seg.gcode_line    = 0;
    seg.start_xyza_mm = start_mm;
    seg.end_xyza_mm   = end_mm;
    seg.feed_mm_min   = feed_mm_min;
    seg.outputs_mask  = outputs_mask;

    double dx = end_mm[0] - start_mm[0];
    double dy = end_mm[1] - start_mm[1];
    double dz = end_mm[2] - start_mm[2];
    double da = end_mm[3] - start_mm[3];
    seg.length_mm = std::sqrt(dx * dx + dy * dy + dz * dz + da * da);

    // Manual moves are never subject to laser Z locking: plan them as a plain machine move
    core::MachineConfig plain = cfg;
    plain.operation_mode = core::OperationMode::Plasma;
    plain.is_mill = true;

    std::vector<core::ToolpathSegment> vec{seg};
    return sliceProgram(vec, plain, 100.0f, 0.0f, false, slice_dt_sec);
}

void TrajectoryPlanner::planFeedHoldDeceleration(
    std::deque<PlannedSlice>& queue,
    const std::array<double, 4>& from_pos,
    double v0_mm_s,
    const core::MachineConfig& cfg,
    double slice_dt_sec)
{
    // Path made of the queued slice targets, up to the first non-motion marker
    std::vector<size_t> path_idx;
    for (size_t i = 0; i < queue.size(); ++i) {
        const auto& q = queue[i];
        if (q.is_dwell || q.is_tool_change || q.is_program_stop) break;
        path_idx.push_back(i);
    }

    double decel = 1e9;
    for (size_t a = 0; a < 3; ++a) {
        decel = std::min(decel, std::max(1.0, cfg.axes[a].deceleration_mm_s2));
    }

    std::deque<PlannedSlice> result;
    if (v0_mm_s <= 1e-6 || path_idx.empty()) {
        queue.clear();
        return;
    }

    // Cumulative path length at each queued target
    std::vector<double> cum(path_idx.size(), 0.0);
    std::array<double, 4> prev = from_pos;
    double total = 0.0;
    for (size_t k = 0; k < path_idx.size(); ++k) {
        const auto& p = queue[path_idx[k]].target_pos_mm;
        double d2 = 0.0;
        for (size_t a = 0; a < 3; ++a) d2 += (p[a] - prev[a]) * (p[a] - prev[a]);
        total += std::sqrt(d2);
        cum[k] = total;
        prev = p;
    }

    const double s_stop = (v0_mm_s * v0_mm_s) / (2.0 * decel);
    if (s_stop >= total) {
        // The queued motion already comes to rest before the stop distance: keep it as-is
        std::deque<PlannedSlice> keep;
        for (size_t k = 0; k < path_idx.size(); ++k) {
            auto ps = queue[path_idx[k]];
            ps.is_hold_decel = true;
            keep.push_back(std::move(ps));
        }
        queue.swap(keep);
        return;
    }

    std::array<int32_t, 4> accum{};
    for (size_t a = 0; a < 4; ++a) {
        accum[a] = static_cast<int32_t>(std::llround(from_pos[a] * cfg.axes[a].steps_per_mm));
    }

    auto pointAt = [&](double s, size_t& seg_k, double& frac) {
        size_t k = 0;
        while (k + 1 < cum.size() && cum[k] < s) ++k;
        const double s0 = (k == 0) ? 0.0 : cum[k - 1];
        const double len = cum[k] - s0;
        frac = (len > 1e-12) ? std::clamp((s - s0) / len, 0.0, 1.0) : 1.0;
        seg_k = k;
        const auto& a_pos = (k == 0) ? from_pos : queue[path_idx[k - 1]].target_pos_mm;
        const auto& b_pos = queue[path_idx[k]].target_pos_mm;
        std::array<double, 4> p{};
        for (size_t a = 0; a < 4; ++a) p[a] = a_pos[a] + (b_pos[a] - a_pos[a]) * frac;
        return p;
    };

    double t = slice_dt_sec;
    bool done = false;
    while (!done) {
        double v = v0_mm_s - decel * t;
        double s = v0_mm_s * t - 0.5 * decel * t * t;
        if (v <= 0.0 || s >= s_stop) {
            s = s_stop;
            done = true;
        }
        size_t k = 0;
        double frac = 0.0;
        const auto p = pointAt(s, k, frac);
        PlannedSlice ps = queue[path_idx[k]];
        ps.is_hold_decel = true;
        ps.target_pos_mm = p;
        for (size_t a = 0; a < 4; ++a) {
            const int32_t steps = static_cast<int32_t>(std::llround(p[a] * cfg.axes[a].steps_per_mm));
            ps.slice.delta_steps[a] = steps - accum[a];
            accum[a] = steps;
        }
        if (const int master = cfg.gantryMasterAxis(); master >= 0) {
            ps.slice.delta_steps[3] = ps.slice.delta_steps[master];
        }
        result.push_back(std::move(ps));
        t += slice_dt_sec;
    }
    queue.swap(result);
}

std::vector<core::ToolpathSegment> TrajectoryPlanner::warpToolpathWithHeightMap(
    const std::vector<core::ToolpathSegment>& original_segments,
    const core::SurfaceHeightMap& map,
    double max_subdivision_len)
{
    std::vector<core::ToolpathSegment> out;
    if (original_segments.empty() || !map.is_valid) {
        return original_segments;
    }

    if (max_subdivision_len < 0.2) max_subdivision_len = 0.2;
    out.reserve(original_segments.size() * 2);

    for (const auto& seg : original_segments) {
        if (seg.type == core::SegmentType::ToolChange || seg.type == core::SegmentType::Dwell ||
            seg.type == core::SegmentType::ProgramStop || seg.machine_axes_mask != 0)
        {
            // Non-motion and machine-coordinate (G53/G28) moves are never warped
            core::ToolpathSegment copy = seg;
            copy.segment_index = out.size();
            out.push_back(copy);
            continue;
        }

        double dx = seg.end_xyza_mm[0] - seg.start_xyza_mm[0];
        double dy = seg.end_xyza_mm[1] - seg.start_xyza_mm[1];
        double dz = seg.end_xyza_mm[2] - seg.start_xyza_mm[2];
        double da = seg.end_xyza_mm[3] - seg.start_xyza_mm[3];
        double len_xy = std::hypot(dx, dy);

        int num_sub = 1;
        if (len_xy > max_subdivision_len) {
            num_sub = static_cast<int>(std::ceil(len_xy / max_subdivision_len));
        }

        if (num_sub <= 1) {
            core::ToolpathSegment wseg = seg;
            wseg.segment_index = out.size();
            wseg.start_xyza_mm[2] += map.getInterpolatedZ(wseg.start_xyza_mm[0], wseg.start_xyza_mm[1]);
            wseg.end_xyza_mm[2]   += map.getInterpolatedZ(wseg.end_xyza_mm[0], wseg.end_xyza_mm[1]);
            double d_x = wseg.end_xyza_mm[0] - wseg.start_xyza_mm[0];
            double d_y = wseg.end_xyza_mm[1] - wseg.start_xyza_mm[1];
            double d_z = wseg.end_xyza_mm[2] - wseg.start_xyza_mm[2];
            double d_a = wseg.end_xyza_mm[3] - wseg.start_xyza_mm[3];
            wseg.length_mm = std::sqrt(d_x * d_x + d_y * d_y + d_z * d_z + d_a * d_a);
            out.push_back(wseg);
        } else {
            for (int k = 0; k < num_sub; ++k) {
                double u0 = static_cast<double>(k) / static_cast<double>(num_sub);
                double u1 = static_cast<double>(k + 1) / static_cast<double>(num_sub);

                core::ToolpathSegment sub = seg;
                sub.segment_index = out.size();
                sub.start_xyza_mm = {
                    seg.start_xyza_mm[0] + dx * u0,
                    seg.start_xyza_mm[1] + dy * u0,
                    seg.start_xyza_mm[2] + dz * u0,
                    seg.start_xyza_mm[3] + da * u0
                };
                sub.end_xyza_mm = {
                    seg.start_xyza_mm[0] + dx * u1,
                    seg.start_xyza_mm[1] + dy * u1,
                    seg.start_xyza_mm[2] + dz * u1,
                    seg.start_xyza_mm[3] + da * u1
                };

                sub.start_xyza_mm[2] += map.getInterpolatedZ(sub.start_xyza_mm[0], sub.start_xyza_mm[1]);
                sub.end_xyza_mm[2]   += map.getInterpolatedZ(sub.end_xyza_mm[0], sub.end_xyza_mm[1]);

                double sx = sub.end_xyza_mm[0] - sub.start_xyza_mm[0];
                double sy = sub.end_xyza_mm[1] - sub.start_xyza_mm[1];
                double sz = sub.end_xyza_mm[2] - sub.start_xyza_mm[2];
                double sa = sub.end_xyza_mm[3] - sub.start_xyza_mm[3];
                sub.length_mm = std::sqrt(sx * sx + sy * sy + sz * sz + sa * sa);

                out.push_back(sub);
            }
        }
    }
    return out;
}

} // namespace rnr::planner
