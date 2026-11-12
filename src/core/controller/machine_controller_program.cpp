// MachineController: Program loading, MDI, job planning and execution (the class is split by topic; see machine_controller.h)
#include "core/controller/machine_controller_internal.h"

namespace rnr::core {

using namespace mc_detail;

// ============================================================================
// Program loading & MDI
// ============================================================================
void MachineController::setProgramLocked(std::shared_ptr<const gcode::ParseResult> program) {
    program_ = program ? std::move(program) : std::make_shared<const gcode::ParseResult>();
    telemetry_.total_segments = program_->segments.size();
    job_planner_.reset();
}

std::shared_ptr<const gcode::ParseResult> MachineController::getProgram() const {
    const auto wait_start = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    diag::controllerReadWait().add(Clock::now() - wait_start);
    return program_;
}

bool MachineController::loadGCodeString(const std::string& gcode_text) {
    gcode::ParseOptions opts;
    std::array<double, 4> work_offset{};
    SurfaceHeightMap map;
    bool warp = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (jobActiveLocked(true)) {
            telemetry_.active_fault_message = "Stop the current job before loading a new program";
            return false;
        }
        opts.dwell_p_milliseconds = config_.dwell_p_milliseconds;
        opts.tool_length_mode     = config_.tool_table.mode;
        work_offset = telemetry_.work_offset_mm;
        warp = telemetry_.surface_compensation_active && surface_height_map_.is_valid;
        map = surface_height_map_;
    }

    // Parsing (and warping) large files happens without holding the controller lock
    auto parsed = std::make_shared<gcode::ParseResult>(
        gcode::GCodeParser::parse(gcode_text, {0.0, 0.0, 0.0, 0.0}, 0.3, work_offset, opts));
    if (parsed->segments.empty() && !parsed->hasErrors()) {
        // Nothing to run and nothing to show: keep the loaded program
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.active_fault_message = "The program has no moves: nothing was loaded";
        return false;
    }
    const std::shared_ptr<const gcode::ParseResult> active = warp ? warpProgram(*parsed, map) : parsed;

    std::lock_guard<std::mutex> lock(mutex_);
    if (jobActiveLocked(true)) {
        telemetry_.active_fault_message = "Stop the current job before loading a new program";
        return false;
    }
    unwarped_program_ = parsed;
    program_text_ = std::make_shared<const std::string>(gcode_text);
    program_reparse_needed_ = false;
    setProgramLocked(active);
    if (!warp) telemetry_.surface_compensation_active = false;

    telemetry_.active_gcode_line_index = 0;
    telemetry_.active_segment_index    = 0;
    telemetry_.segment_progress_0_to_1 = 0.0f;
    telemetry_.job_elapsed_seconds     = 0.0;
    telemetry_.pending_tool_number     = 0;
    telemetry_.pending_tool_description.clear();
    slice_queue_.clear();

    if (parsed->hasErrors()) {
        telemetry_.active_fault_message = "Program has " + std::to_string(parsed->errors.size()) +
                                          " error(s) and cannot run. " + parsed->firstErrorText();
    } else if (telemetry_.state != ControllerState::EStop && telemetry_.state != ControllerState::Disconnected) {
        telemetry_.active_fault_message.clear();
    }

    // Scan for initial commanded spindle speed (S word or M3 S...)
    for (const auto& seg : parsed->segments) {
        if (seg.spindle_rpm > 0) {
            setProgramSpindleRpmLocked(seg.spindle_rpm, true, false);
            break;
        }
    }

    return true; // Loaded (a program with errors is shown, but Cycle Start refuses it)
}

bool MachineController::executeMdiCommand(const std::string& gcode_line) {
    if (gcode_line.empty()) return false;

    std::vector<gcode::GCodeWord> words;
    std::string tok_err;
    const bool tok_ok = gcode::tokenizeBlock(gcode_line, words, nullptr, &tok_err);

    std::lock_guard<std::mutex> lock(mutex_);
    if (!tok_ok) {
        telemetry_.active_fault_message = "MDI rejected: " + tok_err;
        return false;
    }
    if (!readyForCommandLocked(true, "MDI")) {
        return false;
    }
    telemetry_.last_mdi_command = gcode_line;

    auto hasG = [&](double g) {
        for (const auto& w : words) if (w.letter == 'G' && std::abs(w.value - g) < 1e-3) return true;
        return false;
    };
    auto hasM = [&](int m) {
        for (const auto& w : words) if (w.letter == 'M' && static_cast<int>(std::lround(w.value)) == m) return true;
        return false;
    };
    auto wordValue = [&](char letter, double& out) {
        for (const auto& w : words) if (w.letter == letter) { out = w.value; return true; }
        return false;
    };
    // Each MDI command is parsed on its own, so a rotation would not reach the next command
    if (hasG(68)) {
        telemetry_.active_fault_message = "MDI rejected: G68 rotation works only inside a program";
        return false;
    }
    const double unit_scale = hasG(20) || hasG(70) ? 25.4 : 1.0;
    const bool laser = (config_.operation_mode == OperationMode::Laser && !config_.is_mill);

    // Custom M-code macros M100..M999 (e.g. M101.lua)
    for (const auto& w : words) {
        const int m_num = static_cast<int>(std::lround(w.value));
        if (w.letter == 'M' && m_num >= 100 && m_num <= 999) {
            if (!config_.scripting.enabled || !script_runner_) {
                telemetry_.active_fault_message = "MDI rejected: scripting is disabled (M" + std::to_string(m_num) + ")";
                return false;
            }
            const std::string macro_path = config_.scripting.scripts_dir + "/macros/M" + std::to_string(m_num) + ".lua";
            const std::string resolved = scripting::resolveScriptPath(macro_path);
            if (resolved.empty()) {
                telemetry_.active_fault_message = "MDI rejected: macro M" + std::to_string(m_num) + " not found";
                return false;
            }
            return script_runner_->runFileAsync(resolved);
        }
    }

    // G54..G59 and G43 / G49 take effect before the motion of the same block is parsed. If the block is then
    // refused (bad word, soft limit, unhomed axis), they are undone: a refused command changes nothing.
    const uint8_t  wcs_before      = telemetry_.active_workspace;
    const uint16_t tlo_tool_before = telemetry_.active_tlo_tool;
    const double   tlo_mm_before   = telemetry_.active_tlo_mm;
    bool mdi_accepted = false;
    struct MdiRollback {
        std::function<void()> undo;
        ~MdiRollback() { undo(); }
    } rollback{[&] {
        if (mdi_accepted) return;
        if (telemetry_.active_workspace == wcs_before && telemetry_.active_tlo_tool == tlo_tool_before &&
            telemetry_.active_tlo_mm == tlo_mm_before) return;
        telemetry_.active_workspace = wcs_before;
        config_.active_workspace    = wcs_before;
        telemetry_.active_tlo_tool  = tlo_tool_before;
        telemetry_.active_tlo_mm    = tlo_mm_before;
        bumpWorkspaceRevisionLocked();
        refreshWorkFrameLocked();
    }};

    // Workspace selection G54 .. G59
    for (int w = 0; w < 6; ++w) {
        if (hasG(54.0 + w)) {
            telemetry_.active_workspace = static_cast<uint8_t>(w);
            config_.active_workspace    = static_cast<uint8_t>(w);
            bumpWorkspaceRevisionLocked();
            refreshWorkFrameLocked();
            break;
        }
    }

    // Tool length offset G43 H<n> / G49 (applied before any motion on the same block)
    if (hasG(44) || hasG(43.1)) {
        telemetry_.active_fault_message = "MDI rejected: only G43 H<n> / G49 tool length offsets are supported";
        return false;
    }
    if (hasG(43)) {
        double h_val = static_cast<double>(telemetry_.current_tool_number);
        wordValue('H', h_val);
        const long h = std::lround(h_val);
        if (h < 0 || h > 255 || !setToolLengthOffsetLocked(static_cast<uint16_t>(h))) {
            if (h < 0 || h > 255) telemetry_.active_fault_message = "MDI rejected: G43 H value out of range";
            return false;
        }
    } else if (hasG(49)) {
        setToolLengthOffsetLocked(0);
    }

    // G10 L2 / L20 P1..P6: set work coordinate system offsets
    if (hasG(10)) {
        double l_val = 0.0, p_val = 0.0;
        if (!wordValue('L', l_val) || !wordValue('P', p_val) ||
            (std::lround(l_val) != 2 && std::lround(l_val) != 20) || p_val < 1.0 || p_val > 6.0)
        {
            telemetry_.active_fault_message = "MDI rejected: only G10 L2/L20 P1..P6 is supported";
            return false;
        }
        const bool l20 = (std::lround(l_val) == 20);
        const uint8_t target_wcs = static_cast<uint8_t>(std::lround(p_val) - 1);
        const char letters[4] = {'X', 'Y', 'Z', 'A'};
        for (size_t a = 0; a < 4; ++a) {
            double v = 0.0;
            if (wordValue(letters[a], v)) {
                v *= unit_scale;
                // L2: offset value given directly. L20: offset such that the current tool tip reads `v`.
                telemetry_.workspace_offsets_mm[target_wcs][a] = l20 ? wcsOffsetForLocked(a, v) : v;
            }
        }
        config_.workspace_offsets_mm = telemetry_.workspace_offsets_mm;
        bumpWorkspaceRevisionLocked();
        refreshWorkFrameLocked();
        mdi_accepted = true;
        return true;
    }

    // G92: make the current position read as the given work coordinates (adjusts the active WCS offset)
    if (hasG(92)) {
        const char letters[4] = {'X', 'Y', 'Z', 'A'};
        bool any = false;
        for (size_t a = 0; a < 4; ++a) {
            double v = 0.0;
            if (wordValue(letters[a], v)) {
                any = true;
                v *= unit_scale;
                telemetry_.workspace_offsets_mm[telemetry_.active_workspace][a] = wcsOffsetForLocked(a, v);
                if (a == 2) telemetry_.thc_z_offset_mm = 0.0;
            }
        }
        if (!any) {
            telemetry_.active_fault_message = "MDI rejected: G92 needs at least one axis word";
            return false;
        }
        config_.workspace_offsets_mm = telemetry_.workspace_offsets_mm;
        bumpWorkspaceRevisionLocked();
        refreshWorkFrameLocked();
        mdi_accepted = true;
        return true;
    }

    gcode::ParseOptions opts;
    opts.dwell_p_milliseconds = config_.dwell_p_milliseconds;
    opts.initial_feed_mm_min  = mdi_feed_mm_min_;
    opts.tool_length_mode     = config_.tool_table.mode;
    auto mdi_res = gcode::GCodeParser::parse(gcode_line, telemetry_.work_pos_mm, 0.3, telemetry_.work_offset_mm, opts);
    if (mdi_res.hasErrors()) {
        telemetry_.active_fault_message = "MDI rejected: " + mdi_res.errors.front().text;
        return false;
    }
    double f_val = 0.0;
    if (wordValue('F', f_val) && f_val > 0.0) {
        mdi_feed_mm_min_ = f_val * unit_scale; // Feed rate is modal across MDI commands
    }
    if (!telemetry_.simulator_mode && config_.require_homing) {
        for (size_t a = 0; a < 4; ++a) {
            if (((mdi_res.machine_axes_used >> a) & 1U) && !telemetry_.axis_homed[a]) {
                telemetry_.active_fault_message = std::string("MDI rejected: machine-coordinate move on unhomed axis ") + kAxisNames[a];
                return false;
            }
        }
    }

    // Soft limits for every move of this command
    for (const auto& seg : mdi_res.segments) {
        if (seg.type == SegmentType::RapidMove || seg.type == SegmentType::CutLinear ||
            seg.type == SegmentType::CutArcCW || seg.type == SegmentType::CutArcCCW) {
            if (!checkLinearMoveLocked(seg.start_xyza_mm, seg.end_xyza_mm)) return false;
        }
    }

    // S word: update commanded spindle speed
    double s_val = 0.0;
    if (wordValue('S', s_val) && s_val > 0.0) {
        setProgramSpindleRpmLocked(clampRpm(static_cast<uint32_t>(std::lround(s_val))), true, true);
    }

    // Spindle / torch & coolant M-codes
    bool outputs_changed = false;
    if (hasM(3) || hasM(4)) {
        if (!telemetry_.dry_run_mode) {
            if (config_.vfd.enabled && !vfd_controller_.isConnected()) {
                faultLocked("Huanyang VFD is not responding (offline). Spindle cannot start.");
                return false;
            }
            telemetry_.torch_or_spindle_on = true;
            telemetry_.outputs_bitmask |= core::OutputBit::OUT1_TorchOrSpindle;
            if (config_.is_mill && hasM(4)) {
                telemetry_.outputs_bitmask |= core::OutputBit::OUT2_SpindleDir;
            } else if (config_.is_mill) {
                telemetry_.outputs_bitmask &= ~core::OutputBit::OUT2_SpindleDir;
            }
            manual_spindle_requested_ = true;
            outputs_changed = true;
        }
    }
    if (hasM(5)) {
        telemetry_.torch_or_spindle_on = false;
        telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        if (config_.is_mill) telemetry_.outputs_bitmask &= ~core::OutputBit::OUT2_SpindleDir;
        manual_spindle_requested_ = false;
        outputs_changed = true;
    }
    if (hasM(7)) { telemetry_.outputs_bitmask |= core::OutputBit::OUT4_MistCoolant;  outputs_changed = true; }
    if (hasM(8)) { telemetry_.outputs_bitmask |= core::OutputBit::OUT3_FloodCoolant; outputs_changed = true; }
    if (hasM(9)) {
        telemetry_.outputs_bitmask &= ~(core::OutputBit::OUT3_FloodCoolant | core::OutputBit::OUT4_MistCoolant);
        outputs_changed = true;
    }
    if (outputs_changed && outputsLiveLocked()) {
        setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
    }

    // Motion
    std::vector<ToolpathSegment> moves;
    moves.reserve(mdi_res.segments.size());
    for (auto& seg : mdi_res.segments) {
        if (seg.type == SegmentType::ProgramStop) continue; // M0/M1 have no meaning in MDI
        seg.outputs_mask    = telemetry_.outputs_bitmask;
        seg.workspace_index = telemetry_.active_workspace;
        moves.push_back(seg);
    }
    if (!moves.empty() && fwActiveLocked()) {
        mdi_accepted = fwStartManualMovesLocked(moves, ControllerState::Jogging);
        return mdi_accepted;
    }
    if (!moves.empty()) {
        MachineConfig mdi_cfg = config_;
        if (!laser) mdi_cfg.is_mill = true;
        slice_queue_ = planner::TrajectoryPlanner::sliceProgram(
            moves, mdi_cfg, telemetry_.feed_override_pct, 0.0f, telemetry_.dry_run_mode, kDtSec);
        if (!slice_queue_.empty()) {
            if (board_->isOpen() && !telemetry_.simulator_mode) {
                board_->beginJog(Axis::X, config_.limit_override);
            }
            telemetry_.state = ControllerState::Jogging;
        }
    }
    mdi_accepted = true;
    return true;
}

void MachineController::setNextGCodeLine(size_t line_index) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motionActiveLocked()) {
        return;
    }
    if (program_->lines.empty()) return;

    const size_t clamped_line = std::min(line_index, program_->lines.size() - 1);
    telemetry_.active_gcode_line_index = clamped_line;
    telemetry_.segment_progress_0_to_1 = 0.0f;
    telemetry_.pending_tool_number     = 0;
    telemetry_.pending_tool_description.clear();
    if (clamped_line == 0) {
        telemetry_.job_elapsed_seconds = 0.0;
        telemetry_.thc_z_offset_mm     = 0.0;
    }

    size_t target_seg = 0;
    if (clamped_line > 0) {
        target_seg = program_->segments.size();
        for (size_t i = 0; i < program_->segments.size(); ++i) {
            if (program_->segments[i].gcode_line >= clamped_line) {
                target_seg = i;
                break;
            }
        }
    }
    telemetry_.active_segment_index = target_seg;
    last_sent_program_segment_valid_ = false;

    if (telemetry_.state == ControllerState::PausedHold ||
        telemetry_.state == ControllerState::ToolChangeHold)
    {
        slice_queue_.clear();
        job_planner_.reset();
        hold_needs_replan_ = false;
        telemetry_.state = ControllerState::Idle;
    }
}

// ============================================================================
// Job planning & execution
// ============================================================================
planner::FrameContext MachineController::jobFrameLocked() const {
    planner::FrameContext f;
    f.enabled           = true;
    f.workspace_offsets = telemetry_.workspace_offsets_mm;
    f.plan_workspace    = telemetry_.active_workspace;
    if (config_.tool_table.mode == ToolLengthMode::ToolTable) {
        f.apply_tool_lengths = true;
        for (const auto& [tool, entry] : config_.tool_table.tools) {
            if (tool < f.tool_lengths.size()) f.tool_lengths[tool] = entry.length_mm;
        }
        f.inherit_tlo_mm = telemetry_.active_tlo_mm;
    }
    return f;
}

std::array<double, 4> MachineController::planFramePosLocked() const {
    // Planning frame = spindle frame of the active WCS (work position without the tool length offset)
    std::array<double, 4> p = telemetry_.work_pos_mm;
    p[2] += activeTloLocked();
    return p;
}

bool MachineController::validateProgramForRunLocked(size_t start_seg) {
    const auto& prog = *program_;
    if (prog.hasErrors()) {
        telemetry_.active_fault_message = "Cannot start: program has errors. " + prog.firstErrorText();
        return false;
    }
    // Homing, tool table and soft limits: the same checks the program panel lists after loading
    MachineCheckInput in;
    in.frame            = jobFrameLocked();
    in.homed            = telemetry_.axis_homed;
    in.simulator        = telemetry_.simulator_mode;
    in.start_segment    = start_seg;
    in.include_warnings = false;
    in.board_thc        = fwActiveLocked() && fw_->boardRunsThc();
    fillBoardLimitsLocked(in.board);
    const auto check = checkProgramAgainstMachine(prog, config_, in);
    if (!check.start_refusal.empty()) {
        telemetry_.active_fault_message = check.start_refusal;
        return false;
    }
    return true;
}

std::shared_ptr<const ProgramCheckResult> MachineController::getProgramCheck() const {
    std::shared_ptr<const gcode::ParseResult> prog;
    MachineCheckInput in;
    ProgramCheckKey key;
    MachineConfig cfg;
    {
        const auto wait_start = Clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        diag::controllerReadWait().add(Clock::now() - wait_start);
        prog         = program_;
        in.frame     = jobFrameLocked();
        in.homed     = telemetry_.axis_homed;
        in.simulator = telemetry_.simulator_mode;
        in.board_thc = fw_ && fw_->isOpen() && fw_->boardRunsThc();
        fillBoardLimitsLocked(in.board);
        key.program          = prog.get();
        key.board_thc        = in.board_thc;
        key.board            = in.board;
        key.frame            = in.frame;
        key.homed            = in.homed;
        key.simulator        = in.simulator;
        key.config_revision  = config_revision_;
        key.tool_revision    = config_.tool_table_revision;
        {
            std::lock_guard<std::mutex> check_lock(program_check_mutex_);
            if (program_check_ && key == program_check_key_) return program_check_;
        }
        cfg = config_;
    }
    // Only when the program or one of its inputs changed: a pass over the segments, outside the controller lock
    auto result = std::make_shared<const ProgramCheckResult>(buildProgramCheck(prog, &cfg, in));
    std::lock_guard<std::mutex> check_lock(program_check_mutex_);
    program_check_     = result;
    program_check_key_ = key;
    return result;
}

bool MachineController::ProgramCheckKey::operator==(const ProgramCheckKey& o) const {
    return program == o.program && homed == o.homed && simulator == o.simulator && board_thc == o.board_thc &&
           board == o.board &&
           config_revision == o.config_revision && tool_revision == o.tool_revision &&
           frame.enabled == o.frame.enabled && frame.workspace_offsets == o.frame.workspace_offsets &&
           frame.plan_workspace == o.frame.plan_workspace && frame.apply_tool_lengths == o.frame.apply_tool_lengths &&
           frame.tool_lengths == o.frame.tool_lengths && frame.inherit_tlo_mm == o.frame.inherit_tlo_mm;
}

void MachineController::rebuildPlannedSlicesLocked(size_t start_from_segment, bool resume_in_place) {
    slice_queue_.clear();
    job_planner_.reset();
    const auto& segs = program_->segments;
    if (segs.empty() || start_from_segment >= segs.size()) {
        return;
    }

    job_frame_ = jobFrameLocked();
    size_t program_first = start_from_segment;
    std::array<double, 4> plan_start{};
    std::vector<ToolpathSegment> prefix = buildJobPrefixLocked(start_from_segment, resume_in_place, program_first, plan_start);

    planner::PlannerOptions opts;
    opts.feed_override_pct       = telemetry_.feed_override_pct;
    opts.pierce_delay_override_s = telemetry_.pierce_delay_override_s;
    opts.dry_run_no_torch        = telemetry_.dry_run_mode;
    opts.slice_dt_sec            = kDtSec;
    job_planner_ = std::make_unique<planner::StreamingPlanner>(config_, plan_start, opts, job_frame_);
    queued_feed_override_pct_ = opts.feed_override_pct;
    job_planner_->setSource(std::move(prefix),
                            std::shared_ptr<const std::vector<ToolpathSegment>>(program_, &program_->segments),
                            program_first);
    job_planner_->plan(slice_queue_, kRefillAmount * 2);
}


// Approach moves before a job (start lift to safe Z, Run From Line, resume in place), in the planning frame of
// job_frame_ (set by the caller). Shared by the slice planner and firmware-planned boards.
std::vector<ToolpathSegment> MachineController::buildJobPrefixLocked(size_t start_from_segment, bool resume_in_place,
                                                                     size_t& program_first, std::array<double, 4>& plan_start) {
    const auto& segs = program_->segments;
    const uint8_t plan_ws = job_frame_.plan_workspace;
    const auto& offsets = job_frame_.workspace_offsets;
    auto toPlan = [&](const std::array<double, 4>& p, uint8_t ws) {
        std::array<double, 4> r{};
        for (size_t a = 0; a < 4; ++a) r[a] = p[a] + offsets[std::min<uint8_t>(ws, 5)][a] - offsets[plan_ws][a];
        return r;
    };
    auto isMotion = [](const ToolpathSegment& s) {
        return s.type == SegmentType::RapidMove || s.type == SegmentType::CutLinear ||
               s.type == SegmentType::CutArcCW || s.type == SegmentType::CutArcCCW;
    };

    std::vector<ToolpathSegment> prefix;
    // Planning frame = spindle frame of the active WCS (tool length offsets come from the segments)
    std::array<double, 4> cur = planFramePosLocked();
    plan_start = cur;

    // Safe retract height (configured for the tool tip): never above the Z soft limit of a homed machine
    double safe_z = std::max(cur[2], config_.start_safe_z_mm + activeTloLocked());
    {
        const auto& zax = config_.axes[2];
        if (zax.soft_limits_enabled && telemetry_.axis_homed[2]) {
            const double z_max_work = zax.soft_limit_max_mm - offsets[plan_ws][2] - 0.5;
            safe_z = std::max(cur[2], std::min(safe_z, z_max_work));
        }
    }

    // Scan upcoming segments in the current tool block to check if spindle is required.
    // RULE: Never start spindle before M03 if starting from the beginning (start_from_segment == 0).
    bool spindle_needed = false;
    uint16_t target_spindle_rpm = (telemetry_.commanded_spindle_rpm > 0) ? telemetry_.commanded_spindle_rpm : kFallbackSpindleRpm;
    uint8_t approach_mask = 0;

    if (start_from_segment > 0) {
        bool m3_active_before = false;
        for (size_t i = start_from_segment; i > 0; --i) {
            const auto& s = segs[i - 1];
            if (s.type == SegmentType::ToolChange) {
                break; // Tool change stops spindle
            }
            if ((s.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0) {
                m3_active_before = true;
                break;
            }
        }
        if (m3_active_before || telemetry_.torch_or_spindle_on) {
            for (size_t i = start_from_segment; i < segs.size(); ++i) {
                const auto& s = segs[i];
                if (s.type == SegmentType::ToolChange && i > start_from_segment) {
                    break; // Do not scan beyond next tool change
                }
                if (s.spindle_rpm > 0) {
                    target_spindle_rpm = s.spindle_rpm;
                }
                if ((s.outputs_mask & core::OutputBit::OUT1_TorchOrSpindle) != 0 ||
                    s.type == SegmentType::CutLinear || s.type == SegmentType::CutArcCW || s.type == SegmentType::CutArcCCW)
                {
                    spindle_needed = true;
                    approach_mask = s.outputs_mask | core::OutputBit::OUT1_TorchOrSpindle;
                    break;
                }
            }
        }
    }

    if (spindle_needed) {
        setProgramSpindleRpmLocked(target_spindle_rpm, false, false);
    }

    const bool is_plasma = (config_.operation_mode == OperationMode::Plasma && !config_.is_mill);
    auto makePrefix = [&](SegmentType type, const ToolpathSegment& meta) {
        ToolpathSegment s{};
        s.type            = type;
        s.segment_index   = meta.segment_index;
        s.gcode_line      = meta.gcode_line;
        s.start_xyza_mm   = cur;
        s.end_xyza_mm     = cur;
        s.workspace_index = plan_ws; // Prefix moves are already in the planning frame
        s.tlo_tool        = 0;       // ...including any tool length offset
        s.tool_number     = meta.tool_number;
        return s;
    };
    auto spinUpDwell = [&](const ToolpathSegment& meta) {
        ToolpathSegment d = makePrefix(SegmentType::Dwell, meta);
        // Spindle needs its ramp time; a plasma torch needs a fresh pierce before it moves
        d.dwell_seconds = is_plasma
            ? std::max(0.1, telemetry_.pierce_delay_override_s > 0.01f ? static_cast<double>(telemetry_.pierce_delay_override_s) : 0.5)
            : std::max(0.1, config_.spindle_ramp_up_s);
        d.outputs_mask  = approach_mask;
        d.spindle_rpm   = target_spindle_rpm;
        d.torch_active  = true;
        return d;
    };

    program_first = start_from_segment;

    if (resume_in_place) {
        // Resuming from Feed Hold: the tool sits on the path; continue the interrupted segment from here
        if (spindle_needed && !telemetry_.dry_run_mode && (!telemetry_.torch_or_spindle_on || is_plasma)) {
            prefix.push_back(spinUpDwell(segs[start_from_segment]));
        }
    } else {
        // Fresh start, restart after Stop, or jump to a line: find the first segment with physical motion
        size_t first_motion = segs.size();
        for (size_t i = start_from_segment; i < segs.size(); ++i) {
            if (isMotion(segs[i])) {
                first_motion = i;
                break;
            }
        }
        // Non-motion segments (tool change, dwell, program stop) before the first motion run in place
        for (size_t i = start_from_segment; i < first_motion; ++i) {
            ToolpathSegment non_motion = segs[i];
            non_motion.start_xyza_mm = cur;
            non_motion.end_xyza_mm   = cur;
            non_motion.workspace_index = plan_ws;
            non_motion.machine_axes_mask = 0;
            non_motion.tlo_tool = 0;
            prefix.push_back(non_motion);
        }
        program_first = first_motion;

        if (first_motion < segs.size()) {
            const auto& fm = segs[first_motion];
            // Planning-frame start point of the first motion = end of the segment before it
            const std::array<double, 4> target_motion_start = (first_motion > 0)
                ? planner::TrajectoryPlanner::transformTarget(segs[first_motion - 1], job_frame_)
                : [&] { auto p = toPlan(fm.start_xyza_mm, fm.workspace_index); p[2] += planner::TrajectoryPlanner::toolLengthFor(fm, job_frame_); return p; }();
            const std::array<double, 4> fm_end = planner::TrajectoryPlanner::transformTarget(fm, job_frame_);

            if (start_from_segment > 0) {
                // MID-PROGRAM START: lift (spindle off) -> spin up -> XY at safe Z -> descend -> continue
                if (safe_z - cur[2] > 0.05) {
                    ToolpathSegment lift = makePrefix(SegmentType::RapidMove, fm);
                    lift.end_xyza_mm = {cur[0], cur[1], safe_z, cur[3]};
                    prefix.push_back(lift);
                    cur = lift.end_xyza_mm;
                }
                if (spindle_needed && !telemetry_.dry_run_mode) {
                    prefix.push_back(spinUpDwell(fm));
                }
                const bool is_cutting_motion = (fm.type == SegmentType::CutLinear ||
                                                fm.type == SegmentType::CutArcCW || fm.type == SegmentType::CutArcCCW);
                if (is_cutting_motion) {
                    if (std::hypot(target_motion_start[0] - cur[0], target_motion_start[1] - cur[1]) > 0.05) {
                        ToolpathSegment xy_move = makePrefix(SegmentType::RapidMove, fm);
                        xy_move.end_xyza_mm  = {target_motion_start[0], target_motion_start[1], cur[2], target_motion_start[3]};
                        xy_move.outputs_mask = spindle_needed ? approach_mask : 0;
                        prefix.push_back(xy_move);
                        cur = xy_move.end_xyza_mm;
                    }
                    if (cur[2] - target_motion_start[2] > 0.05) {
                        ToolpathSegment z_approach = makePrefix(SegmentType::RapidMove, fm);
                        z_approach.end_xyza_mm  = target_motion_start;
                        z_approach.outputs_mask = spindle_needed ? approach_mask : 0;
                        prefix.push_back(z_approach);
                        cur = target_motion_start;
                    }
                }
            } else {
                // START FROM THE BEGINNING: a first RAPID that travels in XY never drags the tool sideways
                // at a low height: go up, across at travel height, then the rapid itself only descends.
                // A first FEED move is executed exactly as programmed, and so is everything when the lift is off.
                const bool moves_xy = std::hypot(fm_end[0] - cur[0], fm_end[1] - cur[1]) > 0.05;
                if (config_.start_lift_to_safe_z && fm.type == SegmentType::RapidMove && moves_xy) {
                    double travel_z = std::max(cur[2], fm_end[2]);
                    if (std::abs(fm_end[2] - cur[2]) <= 0.05) {
                        travel_z = std::max(travel_z, safe_z); // Pure XY rapid at the current (possibly low) Z
                    }
                    if (travel_z - cur[2] > 0.05) {
                        ToolpathSegment lift = makePrefix(SegmentType::RapidMove, fm);
                        lift.end_xyza_mm = {cur[0], cur[1], travel_z, cur[3]};
                        prefix.push_back(lift);
                        cur = lift.end_xyza_mm;
                    }
                    if (travel_z - fm_end[2] > 0.05) {
                        ToolpathSegment across = makePrefix(SegmentType::RapidMove, fm);
                        across.end_xyza_mm = {fm_end[0], fm_end[1], travel_z, fm_end[3]};
                        prefix.push_back(across);
                        cur = across.end_xyza_mm;
                    }
                }
            }
        }
    }

    for (auto& p : prefix) {
        p.length_mm = dist3(p.start_xyza_mm, p.end_xyza_mm);
    }

    return prefix;
}

void MachineController::refillJobQueueLocked() {
    if (!job_planner_ || job_planner_->finished() || hold_decel_active_) return;
    const auto s = telemetry_.state;
    if (s != ControllerState::RunningGCode && s != ControllerState::DwellPierce) return;
    if (slice_queue_.size() >= kRefillThreshold) return;
    job_planner_->plan(slice_queue_, kRefillAmount);
}

void MachineController::replanFromQueueTailLocked() {
    // Keep a short buffer of already-planned slices and re-plan everything after it with the new feed
    if (!job_planner_ || hold_decel_active_ || slice_queue_.size() <= kOverrideKeepSlices) return;
    const auto& last = slice_queue_[kOverrideKeepSlices - 1];
    const auto& prev = slice_queue_[kOverrideKeepSlices - 2];
    if (last.is_dwell || last.is_tool_change || last.is_program_stop || last.is_hold_decel || !last.from_program ||
        prev.is_dwell || prev.is_tool_change || prev.is_program_stop || !prev.from_program) {
        return;
    }
    if (last.segment_index >= program_->segments.size()) return;

    std::array<double, 4> unit{};
    double d2 = 0.0;
    for (size_t a = 0; a < 4; ++a) {
        unit[a] = last.target_pos_mm[a] - prev.target_pos_mm[a];
        d2 += unit[a] * unit[a];
    }
    const double d = std::sqrt(d2);
    if (d > 1e-9) {
        for (auto& u : unit) u /= d;
    }
    const double v = d / kDtSec;

    // Remainder of the current segment becomes the prefix of the new planner
    ToolpathSegment rem = program_->segments[last.segment_index];
    rem.start_xyza_mm = last.target_pos_mm;

    planner::PlannerOptions opts;
    opts.feed_override_pct       = telemetry_.feed_override_pct;
    opts.pierce_delay_override_s = telemetry_.pierce_delay_override_s;
    opts.dry_run_no_torch        = telemetry_.dry_run_mode;
    opts.slice_dt_sec            = kDtSec;
    auto np = std::make_unique<planner::StreamingPlanner>(config_, last.target_pos_mm, opts, job_frame_);
    np->setSource({rem}, std::shared_ptr<const std::vector<ToolpathSegment>>(program_, &program_->segments),
                  last.segment_index + 1, true);
    np->setInitialMotion(v, unit);

    slice_queue_.erase(slice_queue_.begin() + static_cast<std::ptrdiff_t>(kOverrideKeepSlices), slice_queue_.end());
    job_planner_ = std::move(np);
    queued_feed_override_pct_ = opts.feed_override_pct;
    job_planner_->plan(slice_queue_, kRefillAmount);
}

void MachineController::startJob() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (program_->segments.empty()) return;

    // SAFETY LOCK: Never allow Cycle Start while E-Stop is active or the board is disconnected
    if (telemetry_.state == ControllerState::EStop || telemetry_.state == ControllerState::Disconnected) {
        return;
    }
    if (telemetry_.surface_probing_active) {
        telemetry_.active_fault_message = "Cycle Start rejected: surface probing in progress";
        return;
    }

    // SAFETY LOCK: If Huanyang VFD is enabled but not answering, the spindle cannot start
    if (config_.vfd.enabled && !vfd_controller_.isConnected()) {
        faultLocked("Huanyang VFD is not responding (offline). Spindle cannot start.");
        return;
    }

    if (telemetry_.state == ControllerState::PausedHold) {
        if (hold_decel_active_) {
            // Still decelerating from the hold: resume as soon as the machine has stopped
            resume_after_hold_ = true;
            stop_after_hold_   = false;
            telemetry_.state   = ControllerState::RunningGCode;
            return;
        }
        if (!hold_needs_replan_ && (!slice_queue_.empty() || (job_planner_ && !job_planner_->finished()))) {
            telemetry_.state = ControllerState::RunningGCode; // M0 / M1 resume: continue the queued motion
            telemetry_.active_fault_message.clear();
            return;
        }
    }

    bool fw_m0_resume = false;
    if (fwActiveLocked() && telemetry_.state == ControllerState::PausedHold) {
        if (fw_->jobActive()) {
            // Feed hold on the board: cycle start continues the buffered motion
            fw_->resume();
            telemetry_.state = ControllerState::RunningGCode;
            telemetry_.active_fault_message.clear();
            fw_state_grace_until_ = Clock::now() + std::chrono::milliseconds(400);
            return;
        }
        fw_m0_resume = fw_program_stop_;
    }

    if (telemetry_.state != ControllerState::Idle &&
        telemetry_.state != ControllerState::PausedHold &&
        telemetry_.state != ControllerState::ToolChangeHold)
    {
        return;
    }
    if (cont_jog_active_ || (!slice_queue_.empty() && telemetry_.state == ControllerState::Idle)) {
        return;
    }
    if (fwActiveLocked() && (!fw_->ready() || fw_->busyWithMotion())) {
        telemetry_.active_fault_message = "Cycle Start rejected: the board is busy";
        return;
    }
    if (fwActiveLocked() && !config_.is_mill && config_.operation_mode == OperationMode::Plasma &&
        (config_.thc.enabled || config_.thc.wait_for_arc_ok) && !fw_->boardRunsThc()) {
        telemetry_.active_fault_message = "Cycle Start rejected: torch height control and Wait for Arc OK need a board "
                                          "with real-time control; switch THC off for this board";
        return;
    }

    const bool resume_in_place = fw_m0_resume ||
        ((telemetry_.state == ControllerState::PausedHold) && last_sent_from_program_ && !fwActiveLocked());

    // If resuming from Manual Tool Change Hold (M6), confirm the new tool is installed
    const bool was_tool_hold = (telemetry_.state == ControllerState::ToolChangeHold || telemetry_.pending_tool_number > 0);

    size_t start_seg = telemetry_.active_segment_index;
    if (fw_m0_resume) {
        start_seg = fw_resume_segment_;
    } else if (telemetry_.state == ControllerState::PausedHold && last_sent_program_segment_valid_ && !fwActiveLocked()) {
        // Resume from the segment that was actually sent last (acknowledgements may lag behind)
        start_seg = last_sent_program_segment_;
    }
    if (start_seg >= program_->segments.size()) {
        start_seg = 0;
    }
    // If resuming after confirming an M6 tool change, advance past the ToolChange segment
    if (was_tool_hold && start_seg < program_->segments.size() &&
        program_->segments[start_seg].type == SegmentType::ToolChange)
    {
        start_seg++;
    }

    if (!validateProgramForRunLocked(start_seg)) {
        return;
    }

    if (was_tool_hold) {
        if (telemetry_.pending_tool_number > 0) {
            telemetry_.current_tool_number = telemetry_.pending_tool_number;
        }
        telemetry_.pending_tool_number = 0;
        telemetry_.pending_tool_description.clear();
    }

    telemetry_.limit_switch_triggered = false;
    telemetry_.soft_limit_triggered   = false;
    telemetry_.active_fault_message.clear();

    if (start_seg == 0) {
        telemetry_.active_gcode_line_index = 0;
        telemetry_.job_elapsed_seconds = 0.0;
        telemetry_.thc_z_offset_mm     = 0.0;
    }
    telemetry_.active_segment_index    = start_seg;
    telemetry_.segment_progress_0_to_1 = 0.0f;

    hold_decel_active_ = false;
    hold_needs_replan_ = false;
    resume_after_hold_ = false;
    stop_after_hold_   = false;
    if (fwActiveLocked()) {
        fw_program_stop_ = false;
        if (fwStartJobLocked(start_seg, resume_in_place)) {
            telemetry_.state = ControllerState::RunningGCode;
        }
        return;
    }
    rebuildPlannedSlicesLocked(start_seg, resume_in_place);
    telemetry_.state = slice_queue_.empty() ? ControllerState::Idle : ControllerState::RunningGCode;
}

void MachineController::pauseJob() {
    std::lock_guard<std::mutex> lock(mutex_);
    pauseJobLocked();
}

void MachineController::pauseJobLocked() {
    if (telemetry_.state != ControllerState::RunningGCode && telemetry_.state != ControllerState::DwellPierce) {
        return;
    }
    if (fwActiveLocked()) {
        // Feed hold on the board: it decelerates on the path; spindle and coolant stay on
        telemetry_.state = ControllerState::PausedHold;
        telemetry_.current_feed_mm_min = 0.0;
        fw_->hold();
        fw_seen_held_ = false;
        fw_state_grace_until_ = Clock::now() + std::chrono::milliseconds(400);
        return;
    }
    telemetry_.state = ControllerState::PausedHold;
    telemetry_.current_feed_mm_min = 0.0;
    manual_spindle_requested_ = false;
    if (config_.spindle_ramp_down_s <= 0.01) {
        spindle_ramp_pct_ = 0.0;
        telemetry_.spindle_speed_pct = 0.0f;
        telemetry_.spindle_ramping_up = false;
        telemetry_.spindle_ramping_down = false;
        telemetry_.torch_or_spindle_on = false;
        telemetry_.outputs_bitmask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        if (outputsLiveLocked()) {
            setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
        }
    }
    beginHoldDecelLocked(false);
}

void MachineController::beginHoldDecelLocked(bool stop_after) {
    if (hold_decel_active_) {
        if (stop_after) {
            stop_after_hold_   = true;
            resume_after_hold_ = false;
        }
        return;
    }
    job_planner_.reset();
    hold_needs_replan_ = true;
    resume_after_hold_ = false;
    const double v0 = last_sent_target_valid_ ? last_sent_speed_mm_s_ : 0.0;
    const std::array<double, 4> from = last_sent_target_valid_ ? last_sent_target_ : telemetry_.work_pos_mm;
    planner::TrajectoryPlanner::planFeedHoldDeceleration(slice_queue_, from, v0, config_, kDtSec);
    // Planned slices carry outputs; during a hold the torch / laser must not be re-enabled
    for (auto& s : slice_queue_) {
        if (telemetry_.state == ControllerState::PausedHold) {
            s.slice.outputs_mask &= ~core::OutputBit::OUT1_TorchOrSpindle;
        }
    }
    if (slice_queue_.empty()) {
        hold_decel_active_ = false;
        if (stop_after) {
            stopJobLocked(false);
        }
    } else {
        hold_decel_active_ = true;
        stop_after_hold_   = stop_after;
    }
}

void MachineController::stopJobLocked(bool is_estop) {
    surface_probing_cancel_requested_.store(true);
    if (script_runner_) {
        script_runner_->cancel();
    }

    if (fwActiveLocked()) {
        if (is_estop && !suppress_fw_estop_) {
            fw_->emergencyStop();
        } else if (!is_estop && fw_->busy()) {
            fw_->stop();
        }
        fw_program_stop_ = false;
        fw_sync_wait_ = false;
        fw_outputs_applied_ = 0xFF;
    }
    const bool hw = board_->isOpen() && !telemetry_.simulator_mode;
    if (hw && is_estop) {
        // Halt the board first, then the outputs, then drop anything still buffered
        board_->emergencyStop();
        setBoardOutputsLocked(0x00, false);
    }

    slice_queue_.clear();
    job_planner_.reset();
    in_flight_seqs_.clear();
    homing_axis_queue_.clear();
    homing_phase_              = HomingPhase::None;
    active_probe_mode_         = ProbeMode::None;
    cont_jog_active_           = false;
    cont_jog_held_             = {false, false, false, false};
    cont_jog_target_vel_mm_s_  = {0.0, 0.0, 0.0, 0.0};
    cont_jog_current_vel_mm_s_ = {0.0, 0.0, 0.0, 0.0};
    hold_decel_active_         = false;
    hold_needs_replan_         = false;
    resume_after_hold_         = false;
    stop_after_hold_           = false;
    settle_active_             = false;
    torch_on_duration_s_       = 0.0;
    manual_spindle_requested_  = false;
    pulse_active_              = false;
    air_postflow_timer_s_      = 0.0;
    last_sent_laser_pwm_duty_  = 0xFFFF;
    last_sent_target_valid_    = false;
    last_sent_speed_mm_s_      = 0.0;
    telemetry_.live_laser_power_pct = 0.0f;
    telemetry_.axis_vel_mm_min = {0.0, 0.0, 0.0, 0.0};
    telemetry_.current_feed_mm_min  = 0.0;
    telemetry_.segment_progress_0_to_1 = 0.0f;
    telemetry_.pending_tool_number     = 0;
    telemetry_.pending_tool_description.clear();

    if (hw) {
        board_->cancelMotion();
    }
    program_run_open_     = false; // Cancel ends any open run
    program_outputs_sent_ = 0xFFFF;

    if (is_estop) {
        // Immediate hard cut-off of every output on E-Stop
        spindle_ramp_pct_               = 0.0;
        telemetry_.spindle_speed_pct    = 0.0f;
        telemetry_.spindle_ramping_up   = false;
        telemetry_.spindle_ramping_down = false;
        telemetry_.torch_or_spindle_on  = false;
        telemetry_.outputs_bitmask      = 0;
        telemetry_.laser_air_assist_active  = false;
        telemetry_.laser_red_pointer_active = false;
        telemetry_.spindle_pwm_duty     = 0;
        config_.limit_override          = false;
        telemetry_.limit_override_active = false;
        if (hw) {
            board_->setSpindlePwm(0, false);
        }
        if (config_.vfd.enabled) {
            vfd_controller_.emergencyStop();
        }
        vfd_last_run_cmd_ = 0;
    } else {
        // Normal Stop: if spindle is active, engage controlled deceleration ramp down
        if ((spindle_ramp_pct_ > 0.05 || telemetry_.torch_or_spindle_on) && config_.spindle_ramp_down_s > 0.01) {
            telemetry_.spindle_ramping_up   = false;
            telemetry_.spindle_ramping_down = true;
            telemetry_.torch_or_spindle_on  = true;
            telemetry_.outputs_bitmask     |= core::OutputBit::OUT1_TorchOrSpindle;
        } else {
            spindle_ramp_pct_               = 0.0;
            telemetry_.spindle_speed_pct    = 0.0f;
            telemetry_.spindle_ramping_up   = false;
            telemetry_.spindle_ramping_down = false;
            telemetry_.torch_or_spindle_on  = false;
            telemetry_.outputs_bitmask     &= ~core::OutputBit::OUT1_TorchOrSpindle;
        }
        if (hw) {
            setBoardOutputsLocked(telemetry_.outputs_bitmask, config_.limit_override);
        }
    }

    if (telemetry_.state != ControllerState::Disconnected && telemetry_.state != ControllerState::EStop) {
        telemetry_.state = ControllerState::Idle;
    }
}

void MachineController::stopJob() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fwActiveLocked() && fw_->busy() && !fw_->stopping()) {
        // Controlled stop on the board: hold, wait, reset (position kept); JobFinished / MoveDone follow
        fw_->stop();
        if (fw_->busy()) {
            telemetry_.active_fault_message = "Stopping...";
            return;
        }
    }
    const auto s = telemetry_.state;
    const bool path_motion = (s == ControllerState::RunningGCode || s == ControllerState::DwellPierce ||
                              (s == ControllerState::Jogging && !cont_jog_active_) ||
                              (s == ControllerState::PausedHold && hold_decel_active_));
    if (path_motion && !slice_queue_.empty() && last_sent_target_valid_ && last_sent_speed_mm_s_ > 0.5) {
        // Controlled stop: decelerate along the path, then stop (no lost steps)
        beginHoldDecelLocked(true);
        return;
    }
    stopJobLocked();
}

void MachineController::setFeedOverride(float percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    float old_pct = telemetry_.feed_override_pct;
    const float max_pct = firmwareBoardSelectedLocked() ? 200.0f : 250.0f;
    telemetry_.feed_override_pct = std::clamp(percent, 10.0f, max_pct);
    if (fwActiveLocked()) fw_->setFeedOverride(telemetry_.feed_override_pct);
    (void)old_pct;
    // Feed override never raises motion above the configured axis limits (enforced by the planner). The planner gets
    // every change; the slices already queued are re-planned once the override differs by 0.5 % or more from the
    // one they were planned with, so many small steps add up instead of being lost.
    if (job_planner_) {
        job_planner_->setFeedOverride(telemetry_.feed_override_pct);
        if (std::abs(queued_feed_override_pct_ - telemetry_.feed_override_pct) >= 0.5f &&
            (telemetry_.state == ControllerState::RunningGCode || telemetry_.state == ControllerState::PausedHold)) {
            replanFromQueueTailLocked();
        }
    }
}

void MachineController::setPierceDelayOverride(float seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.pierce_delay_override_s = std::clamp(seconds, 0.0f, 10.0f);
    if (job_planner_) {
        job_planner_->setPierceDelayOverride(telemetry_.pierce_delay_override_s);
    }
}

} // namespace rnr::core
