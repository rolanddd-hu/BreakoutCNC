// G-code parser, planner and program check tests
#include "test_support.h"

void test11() {
    // ------------------------------------------------------------------------
    // Test 11: Machine Type Detection & G-Code Target Compatibility Checking
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 11] Testing G-Code Machine Type Detection & Compatibility Mismatch Alerts...\n";

        // 1. Mill program detection
        std::string mill_gcode = rnr::gcode::GCodeParser::sampleMillAtcGCode();
        int mill_score = 0;
        std::vector<std::string> mill_ind;
        rnr::gcode::MachineType detected_mill = rnr::gcode::GCodeParser::detectTargetMachine(mill_gcode, &mill_score, &mill_ind);
        assert(detected_mill == rnr::gcode::MachineType::Mill);
        assert(mill_score >= 50);
        assert(!mill_ind.empty());

        // 2. Plasma program detection
        std::string plasma_gcode = rnr::gcode::GCodeParser::samplePlasmaBracketGCode();
        int plasma_score = 0;
        std::vector<std::string> plasma_ind;
        rnr::gcode::MachineType detected_plasma = rnr::gcode::GCodeParser::detectTargetMachine(plasma_gcode, &plasma_score, &plasma_ind);
        assert(detected_plasma == rnr::gcode::MachineType::Plasma);
        assert(plasma_score >= 50);
        assert(!plasma_ind.empty());

        // 2b. Built-in laser demo: detected as laser and parses without warnings
        std::string laser_demo = rnr::gcode::GCodeParser::sampleLaserCoasterGCode();
        assert(rnr::gcode::GCodeParser::detectTargetMachine(laser_demo, nullptr, nullptr) == rnr::gcode::MachineType::Laser);
        {
            rnr::core::RnRController laser_ctrl;
            assert(laser_ctrl.loadGCodeString(laser_demo));
            const auto prog = laser_ctrl.getProgram();
            assert(prog->segments.size() > 1000);
            assert(prog->warnings.empty());
            assert(std::abs(prog->bbox_max_mm[0] - prog->bbox_min_mm[0] - 90.0) < 0.01);
            assert(std::abs(prog->bbox_max_mm[1] - prog->bbox_min_mm[1] - 90.0) < 0.01);

            // BreakoutCut: S is laser power, never an RPM-based PWM duty while the laser is off
            assert(laser_ctrl.getTelemetry().spindle_pwm_duty == 0);
            assert(laser_ctrl.resetFromEStop());
            assert(laser_ctrl.executeMdiCommand("S500"));
            assert(laser_ctrl.getTelemetry().spindle_pwm_duty == 0);
            // A spindle ramp (e.g. from a Mach3 import) would keep the torch / laser on after M5
            auto cut_cfg = laser_ctrl.getConfig();
            cut_cfg.spindle_ramp_up_s   = 2.5;
            cut_cfg.spindle_ramp_down_s = 2.0;
            laser_ctrl.applyConfig(cut_cfg);
            assert(laser_ctrl.getConfig().spindle_ramp_up_s == 0.0 && laser_ctrl.getConfig().spindle_ramp_down_s == 0.0);
        }
        {
            rnr::core::RnRController mill_ctrl;
            auto mill_cfg = mill_ctrl.getConfig();
            mill_cfg.is_mill = true;
            mill_cfg.spindle_ramp_down_s = 2.0;
            mill_ctrl.applyConfig(mill_cfg);
            assert(mill_ctrl.getConfig().spindle_ramp_down_s == 2.0);
            assert(mill_ctrl.resetFromEStop());
            // With a ramp the worker recomputes the duty every tick (0 while the spindle is off), so run the spindle
            assert(mill_ctrl.executeMdiCommand("M3 S12000"));
            assert(waitUntil([&] { return mill_ctrl.getTelemetry().spindle_pwm_duty == 500; })); // 12000 of 24000 RPM
        }

        // 3. Laser program detection
        std::string laser_gcode =
            "; LightBurn 1.4.03\n"
            "; GRBL-M3/M4 Laser Device Profile\n"
            "G21 G90\n"
            "M4 S0\n"
            "G0 X10.0 Y10.0\n"
            "G1 X50.0 Y10.0 S800 F3000\n"
            "G1 X50.0 Y50.0 S800\n"
            "G1 X10.0 Y50.0 S800\n"
            "G1 X10.0 Y10.0 S800\n"
            "M5\n"
            "M30\n";
        int laser_score = 0;
        std::vector<std::string> laser_ind;
        rnr::gcode::MachineType detected_laser = rnr::gcode::GCodeParser::detectTargetMachine(laser_gcode, &laser_score, &laser_ind);
        assert(detected_laser == rnr::gcode::MachineType::Laser);
        assert(laser_score >= 50);
        assert(!laser_ind.empty());

        // 4. Generic neutral program detection
        std::string neutral_gcode =
            "G21 G90\n"
            "G0 X0 Y0\n"
            "G1 X100.0 Y50.0 F1000\n"
            "G1 X0 Y0\n"
            "M30\n";
        int neutral_score = 0;
        rnr::gcode::MachineType detected_neutral = rnr::gcode::GCodeParser::detectTargetMachine(neutral_gcode, &neutral_score);
        assert(detected_neutral == rnr::gcode::MachineType::Unknown);

        // 5. Compatibility matrix verification
        // Case A: Mill program in Mill app -> MATCH (no popup)
        auto res_mill_in_mill = rnr::gcode::GCodeParser::checkMachineCompatibility(mill_gcode, rnr::gcode::MachineType::Mill);
        assert(!res_mill_in_mill.has_mismatch);

        // Case B: Plasma program in Mill app -> MISMATCH (triggers warning)
        auto res_plasma_in_mill = rnr::gcode::GCodeParser::checkMachineCompatibility(plasma_gcode, rnr::gcode::MachineType::Mill);
        assert(res_plasma_in_mill.has_mismatch);
        assert(res_plasma_in_mill.detected_type == rnr::gcode::MachineType::Plasma);
        assert(res_plasma_in_mill.detected_name == "CNC Plasma Cutter");
        assert(!res_plasma_in_mill.indicators.empty());

        // Case C: Laser program in Mill app -> MISMATCH
        auto res_laser_in_mill = rnr::gcode::GCodeParser::checkMachineCompatibility(laser_gcode, rnr::gcode::MachineType::Mill);
        assert(res_laser_in_mill.has_mismatch);
        assert(res_laser_in_mill.detected_type == rnr::gcode::MachineType::Laser);

        // Case D: Mill program in Plasma app -> MISMATCH
        auto res_mill_in_plasma = rnr::gcode::GCodeParser::checkMachineCompatibility(mill_gcode, rnr::gcode::MachineType::Plasma);
        assert(res_mill_in_plasma.has_mismatch);
        assert(res_mill_in_plasma.detected_type == rnr::gcode::MachineType::Mill);
        assert(res_mill_in_plasma.detected_name == "CNC Milling Machine");

        // Case E: Plasma program in Plasma app -> MATCH
        auto res_plasma_in_plasma = rnr::gcode::GCodeParser::checkMachineCompatibility(plasma_gcode, rnr::gcode::MachineType::Plasma);
        assert(!res_plasma_in_plasma.has_mismatch);

        // Case F: Laser program in Plasma app -> MISMATCH
        auto res_laser_in_plasma = rnr::gcode::GCodeParser::checkMachineCompatibility(laser_gcode, rnr::gcode::MachineType::Plasma);
        assert(res_laser_in_plasma.has_mismatch);
        assert(res_laser_in_plasma.detected_type == rnr::gcode::MachineType::Laser);

        // Case G: Plasma program in Laser app -> MISMATCH
        auto res_plasma_in_laser = rnr::gcode::GCodeParser::checkMachineCompatibility(plasma_gcode, rnr::gcode::MachineType::Laser);
        assert(res_plasma_in_laser.has_mismatch);
        assert(res_plasma_in_laser.detected_type == rnr::gcode::MachineType::Plasma);

        // Case H: Neutral program in any app -> MATCH (no nuisance warnings)
        auto res_neutral_in_mill = rnr::gcode::GCodeParser::checkMachineCompatibility(neutral_gcode, rnr::gcode::MachineType::Mill);
        auto res_neutral_in_plasma = rnr::gcode::GCodeParser::checkMachineCompatibility(neutral_gcode, rnr::gcode::MachineType::Plasma);
        auto res_neutral_in_laser = rnr::gcode::GCodeParser::checkMachineCompatibility(neutral_gcode, rnr::gcode::MachineType::Laser);
        assert(!res_neutral_in_mill.has_mismatch);
        assert(!res_neutral_in_plasma.has_mismatch);
        assert(!res_neutral_in_laser.has_mismatch);

        std::cout << "  -> PASSED: G-Code machine detection, indicators & cross-profile compatibility verified.\n";
    }
}

void test20() {
    // ----------------------------------------------------
    // Test 20: Laser Mode ignore_z_motion Zero Z Spikes (Issue A5)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 20] Testing Laser Mode ignore_z_motion step continuity...\n";
        std::vector<rnr::core::ToolpathSegment> segs;

        rnr::core::ToolpathSegment s1{};
        s1.type = rnr::core::SegmentType::RapidMove;
        s1.start_xyza_mm = {0.0, 0.0, 10.0, 0.0};
        s1.end_xyza_mm   = {50.0, 0.0, 10.0, 0.0};
        s1.length_mm     = 50.0;
        segs.push_back(s1);

        rnr::core::ToolpathSegment s2{};
        s2.type = rnr::core::SegmentType::CutLinear;
        s2.start_xyza_mm = {50.0, 0.0, 2.0, 0.0}; // Different Z in file
        s2.end_xyza_mm   = {50.0, 50.0, 2.0, 0.0};
        s2.length_mm     = 50.0;
        s2.feed_mm_min   = 1200.0;
        segs.push_back(s2);

        rnr::core::MachineConfig cfg{};
        cfg.operation_mode = rnr::core::OperationMode::Laser;
        cfg.laser.ignore_z_motion = true;

        auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(segs, cfg);
        assert(!slices.empty());

        // Verify Z delta steps are 0 across all slices and segment transitions
        for (const auto& sl : slices) {
            assert(sl.slice.delta_steps[2] == 0);
        }
        std::cout << "  -> PASSED: Laser mode locks Z focal height with zero inter-segment step spikes.\n";
    }
}

void test26() {
    // ========================================================================
    // Test 26: Surface Height Map Bicubic Spline Interpolation & Smoothness Slider
    // ========================================================================
    {
        std::cout << "\n[Test 26] Testing Surface Height Map Bicubic Spline, Smoothness Blending & Apply Workflow...\n";

        // Create a 4x4 height map across a 300x300mm surface with a curved dome topology
        rnr::core::SurfaceHeightMap shm;
        shm.is_valid = true;
        shm.min_x = 0.0;
        shm.max_x = 300.0;
        shm.min_y = 0.0;
        shm.max_y = 300.0;
        shm.points_x = 4;
        shm.points_y = 4;
        shm.grid_z = {
            0.0, 0.2, 0.2, 0.0,
            0.3, 0.8, 0.8, 0.3,
            0.3, 0.8, 0.8, 0.3,
            0.0, 0.2, 0.2, 0.0
        };
        shm.recalculateStats();
        assert(shm.grid_z.size() == 16);
        assert(std::abs(shm.min_z - 0.0) < 1e-4);
        assert(std::abs(shm.max_z - 0.8) < 1e-4);

        // 1. Verify exact grid vertex preservation at all 16 nodes for both Bilinear (0.0) and Bicubic (1.0)
        const double dx = 100.0; // 300 / 3
        const double dy = 100.0;
        for (int j = 0; j < 4; ++j) {
            double y = static_cast<double>(j) * dy;
            for (int i = 0; i < 4; ++i) {
                double x = static_cast<double>(i) * dx;
                double expected_z = shm.grid_z[static_cast<size_t>(j * 4 + i)];

                double z_bilinear = shm.getInterpolatedZWithSmoothness(x, y, 0.0);
                double z_bicubic  = shm.getInterpolatedZWithSmoothness(x, y, 1.0);
                double z_blend    = shm.getInterpolatedZWithSmoothness(x, y, 0.5);

                assert(std::abs(z_bilinear - expected_z) < 1e-5);
                assert(std::abs(z_bicubic - expected_z) < 1e-5);
                assert(std::abs(z_blend - expected_z) < 1e-5);
            }
        }

        // 2. Verify intermediate curvature: at (50, 50) mm (midpoint of first cell)
        // Bilinear is flat hyperbolic: average of (0.0, 0.2, 0.3, 0.8) = 0.325
        double z_mid_bilinear = shm.getInterpolatedZWithSmoothness(50.0, 50.0, 0.0);
        assert(std::abs(z_mid_bilinear - 0.325) < 1e-4);

        // Bicubic produces smooth curved ascent
        double z_mid_bicubic = shm.getInterpolatedZWithSmoothness(50.0, 50.0, 1.0);
        assert(z_mid_bicubic > 0.1 && z_mid_bicubic < 0.8);

        // 50% Smoothness blend is exactly arithmetic average
        double z_mid_half = shm.getInterpolatedZWithSmoothness(50.0, 50.0, 0.5);
        assert(std::abs(z_mid_half - (0.5 * z_mid_bilinear + 0.5 * z_mid_bicubic)) < 1e-5);

        // 3. Test Controller Apply Height Map workflow with Smoothness
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        const std::string test_gcode =
            "G21 G90\n"
            "G00 X0 Y0 Z5.0\n"
            "G01 X300.0 Y0 Z0.0 F1200\n"
            "G01 X300.0 Y300.0 Z0.0\n"
            "G01 X0 Y300.0 Z0.0\n"
            "G01 X0 Y0 Z0.0\n";
        bool loaded = controller.loadGCodeString(test_gcode);
        assert(loaded);

        // Apply with smoothness = 0.0 (Bilinear)
        shm.smoothness = 0.0;
        controller.applySurfaceHeightMap(shm);
        assert(controller.isSurfaceCompensationActive());
        auto segs_bilinear = controller.getProgram()->segments;
        assert(!segs_bilinear.empty());

        // Apply with smoothness = 1.0 (Bicubic Spline)
        shm.smoothness = 1.0;
        controller.applySurfaceHeightMap(shm);
        assert(controller.isSurfaceCompensationActive());
        auto segs_bicubic = controller.getProgram()->segments;
        assert(!segs_bicubic.empty());

        // Confirm both produce warped segments while respecting boundary node zero
        assert(std::abs(segs_bilinear.front().end_xyza_mm[2] - 5.0) < 1e-3); // G00 X0 Y0 Z5 ends on a zero map node
        assert(std::abs(segs_bicubic.front().end_xyza_mm[2] - 5.0) < 1e-3);

        controller.disconnect();
        std::cout << "  -> PASSED: 4x4 Bicubic Spline nodes preserved, intermediate curvature evaluated, and Apply smoothness workflow verified.\n";
    }
}

void test27() {
    // ========================================================================
    // Test 27: Parser rejects / reports what it cannot execute correctly
    // ========================================================================
    {
        std::cout << "\n[Test 27] Testing G-code parser errors, G92 continuity, G28/G53 machine moves & M0...\n";
        using rnr::gcode::GCodeParser;
        assert(GCodeParser::parse("G1 X10\n").hasErrors());                       // Feed move without F
        assert(!GCodeParser::parse("G1 X10 F500\n").hasErrors());
        assert(GCodeParser::parse("G41 D1\nG1 X10 F500\n").hasErrors());          // Cutter compensation
        assert(GCodeParser::parse("G95 F0.1\n").hasErrors());                     // Feed per revolution
        assert(GCodeParser::parse("G0 X#1\n").hasErrors());                       // Expressions
        assert(GCodeParser::parse("G2 X10 Y0 I3 J0 F500\n").hasErrors());         // End point not on arc
        assert(GCodeParser::parse("G2 X10 Y0 R2 F500\n").hasErrors());            // R smaller than half chord
        assert(!GCodeParser::parse("G2 X10 Y0 I5 J0 F500\n").hasErrors());
        auto g43 = GCodeParser::parse("G43 H1 Z5\n");
        assert(!g43.hasErrors() && !g43.warnings.empty());                        // G43 ignored -> warning

        // Words written without spaces: "0X1" must not be read as a hexadecimal number, nor "1E2" as 100
        {
            std::vector<rnr::gcode::GCodeWord> words;
            assert(rnr::gcode::tokenizeBlock("g0x1y-2.5z+.5", words, nullptr, nullptr));
            assert(words.size() == 4);
            assert(words[0].letter == 'G' && words[0].value == 0.0);
            assert(words[1].letter == 'X' && words[1].value == 1.0);
            assert(words[2].letter == 'Y' && words[2].value == -2.5);
            assert(words[3].letter == 'Z' && words[3].value == 0.5);
            assert(rnr::gcode::tokenizeBlock("G1X1E2F500", words, nullptr, nullptr));
            assert(words.size() == 4 && words[1].value == 1.0 && words[2].letter == 'E' && words[2].value == 2.0);
            assert(!rnr::gcode::tokenizeBlock("G0X", words, nullptr, nullptr));   // Letter without a number
            assert(!rnr::gcode::tokenizeBlock("G0X.", words, nullptr, nullptr));

            auto packed = GCodeParser::parse("G0X10Y5\nG1X20F500\n");
            auto spaced = GCodeParser::parse("G0 X10 Y5\nG1 X20 F500\n");
            assert(!packed.hasErrors() && packed.segments.size() == spaced.segments.size());
            for (size_t i = 0; i < packed.segments.size(); ++i) {
                for (size_t a = 0; a < 4; ++a) {
                    assert(packed.segments[i].end_xyza_mm[a] == spaced.segments[i].end_xyza_mm[a]);
                }
            }
            assert(std::abs(packed.segments.back().end_xyza_mm[0] - 20.0) < 1e-9);
        }

        // G92 inside a program shifts coordinates without creating a discontinuous jump
        auto g92 = GCodeParser::parse("G0 X10\nG92 X0\nG1 X5 F600\n");
        assert(!g92.hasErrors());
        for (size_t i = 1; i < g92.segments.size(); ++i) {
            for (size_t a = 0; a < 4; ++a) {
                assert(std::abs(g92.segments[i].start_xyza_mm[a] - g92.segments[i - 1].end_xyza_mm[a]) < 1e-9);
            }
        }
        assert(std::abs(g92.segments.back().end_xyza_mm[0] - 15.0) < 1e-9); // Programmed X5 = physical X15

        // G28 Z0 is a machine-coordinate move to machine Z0 even when the parse-time offset is unknown
        auto g28 = GCodeParser::parse("G28 G91 Z0\nG90\n");
        assert(g28.machine_axes_used == 0x04);
        bool found_machine_move = false;
        for (const auto& s : g28.segments) {
            if (!s.hasMachineTarget()) continue;
            const auto& mt = g28.machine_targets.at(s.machine_target);
            if (mt.axes_mask == 0x04 && mt.mm[2] == 0.0) found_machine_move = true;
        }
        assert(found_machine_move);

        // M0 produces a program stop marker
        auto m0 = GCodeParser::parse("G0 X5\nM0\nG0 X0\n");
        bool has_stop = false;
        for (const auto& s : m0.segments) if (s.type == rnr::core::SegmentType::ProgramStop) has_stop = true;
        assert(has_stop);

        // Blocks after M30 are ignored
        auto m30 = GCodeParser::parse("G0 X5\nM30\nG0 X100\n");
        for (const auto& s : m30.segments) assert(s.end_xyza_mm[0] < 50.0);
        std::cout << "  -> PASSED: Unsupported codes reported as errors; G92 continuous; G28 machine move; M0 stop; M30 end.\n";
    }
}

void test28() {
    // ========================================================================
    // Test 28: Planner acceleration, junction speed and hard velocity limits
    // ========================================================================
    {
        std::cout << "\n[Test 28] Testing planner acceleration from rest, feed override limit & pulse-rate cap...\n";
        rnr::core::MachineConfig cfg{};
        // Short (5 mm) move: previously executed at full feed from the first slice
        auto slices = rnr::planner::TrajectoryPlanner::sliceLinearMove({0, 0, 0, 0}, {5, 0, 0, 0}, 3000.0, cfg);
        const double spm = cfg.axes[0].steps_per_mm;
        const double cruise_steps = (3000.0 / 60.0) * 0.025 * spm;
        assert(slices.front().slice.delta_steps[0] < cruise_steps * 0.6); // Ramps up
        assert(slices.back().slice.delta_steps[0] < cruise_steps * 0.6);  // Ramps down
        // Consecutive slices never change speed faster than the acceleration limit allows (+1 step rounding)
        const double max_dv_steps = cfg.axes[0].deceleration_mm_s2 * 0.025 * 0.025 * spm + 2.0;
        for (size_t i = 1; i < slices.size(); ++i) {
            assert(std::abs(slices[i].slice.delta_steps[0] - slices[i - 1].slice.delta_steps[0]) <= max_dv_steps);
        }

        // 250% feed override must never exceed the configured axis velocity
        rnr::core::ToolpathSegment fast{};
        fast.type = rnr::core::SegmentType::CutLinear;
        fast.end_xyza_mm = {500.0, 0.0, 0.0, 0.0};
        fast.length_mm = 500.0;
        fast.feed_mm_min = 6000.0; // = axis max (100 mm/s)
        auto ov = rnr::planner::TrajectoryPlanner::sliceProgram({fast}, cfg, 250.0f);
        const double max_steps = cfg.axes[0].max_velocity_mm_s * 0.025 * spm + 1.0;
        for (const auto& s : ov) assert(std::abs(s.slice.delta_steps[0]) <= max_steps);

        // Pulse-rate cap: 25 kHz board with 1000 steps/mm -> at most 23.75 mm/s
        rnr::core::MachineConfig slow = cfg;
        slow.pulse_rate_khz = 25;
        slow.axes[0].steps_per_mm = 1000.0;
        auto capped = rnr::planner::TrajectoryPlanner::sliceProgram({fast}, slow);
        for (const auto& s : capped) assert(std::abs(s.slice.delta_steps[0]) <= 25000 * 0.025);

        // Square corner: speed drops at the corner instead of turning at full feed
        rnr::core::ToolpathSegment a{}, b{};
        a.type = b.type = rnr::core::SegmentType::CutLinear;
        a.end_xyza_mm = {50, 0, 0, 0};  a.length_mm = 50; a.feed_mm_min = 3000;
        b.start_xyza_mm = {50, 0, 0, 0}; b.end_xyza_mm = {50, 50, 0, 0}; b.length_mm = 50; b.feed_mm_min = 3000;
        auto corner = rnr::planner::TrajectoryPlanner::sliceProgram({a, b}, cfg);
        double min_speed_near_corner = 1e9;
        for (const auto& s : corner) {
            if (std::abs(s.target_pos_mm[0] - 50.0) < 1.0 && s.target_pos_mm[1] < 1.0) {
                const double d = std::hypot(double(s.slice.delta_steps[0]), double(s.slice.delta_steps[1]));
                min_speed_near_corner = std::min(min_speed_near_corner, d);
            }
        }
        assert(min_speed_near_corner < cruise_steps * 0.5);
        std::cout << "  -> PASSED: Moves ramp within accel limits; override & pulse rate never exceed hardware limits; corners slow down.\n";
    }
}

void test35() {
    // ========================================================================
    // Test 35: Parser tracks G43 H<n> / G49 and only warns when the mode is not configured
    // ========================================================================
    {
        std::cout << "\n[Test 35] Testing G43/G49 parsing and tool length mode warnings...\n";
        using rnr::gcode::GCodeParser;
        const std::string prog = "G0 X1\nG43 H2 Z5\nG0 X2\nG49\nG0 X3\n";
        auto pr = GCodeParser::parse(prog);
        assert(pr.segments.size() == 4);
        assert(pr.segments[0].tlo_tool == rnr::core::kTloInherit);
        assert(pr.segments[1].tlo_tool == 2);
        assert(pr.segments[2].tlo_tool == 2);
        assert(pr.segments[3].tlo_tool == 0);
        assert(pr.warnings.size() == 1); // Not configured -> "G43 ignored" warning

        rnr::gcode::ParseOptions touch_off;
        touch_off.tool_length_mode = rnr::core::ToolLengthMode::TouchOffEachTool;
        assert(GCodeParser::parse(prog, {0, 0, 0, 0}, 0.25, {0, 0, 0, 0}, touch_off).warnings.empty());
        rnr::gcode::ParseOptions table;
        table.tool_length_mode = rnr::core::ToolLengthMode::ToolTable;
        assert(GCodeParser::parse(prog, {0, 0, 0, 0}, 0.25, {0, 0, 0, 0}, table).warnings.empty());
        assert(GCodeParser::parse("G44 H1\n").hasErrors());

        // Switching the mode re-evaluates the program that is already loaded
        rnr::core::RnRController controller;
        assert(controller.loadGCodeString(prog));
        assert(controller.getProgram()->warnings.size() == 1);
        auto cfg = controller.getConfig();
        cfg.tool_table.mode = rnr::core::ToolLengthMode::TouchOffEachTool;
        controller.applyConfig(cfg);
        assert(controller.getProgram()->warnings.empty());
        assert(controller.getProgram()->segments.size() == 4);
        cfg = controller.getConfig();
        cfg.tool_table.mode = rnr::core::ToolLengthMode::NotConfigured;
        controller.applyConfig(cfg);
        assert(controller.getProgram()->warnings.size() == 1);
        std::cout << "  -> PASSED: G43/G49 recorded per segment; warning only when the tool length mode is not configured.\n";
    }
}

void test44() {
    // ========================================================================
    // Test 44: G61 / G64 P, corner tolerance and the curve speed limit
    // ========================================================================
    {
        std::cout << "\n[Test 44] Testing exact stop, corner tolerance and curve speed...\n";
        using rnr::core::SegmentType;
        using rnr::core::ToolpathSegment;
        using rnr::gcode::GCodeParser;

        // Modal state per move; the P word is not a dwell and the Q word not a peck depth
        const auto parsed = GCodeParser::parse(
            "G21 G90 F600\nG1 X1\nG61\nG1 X2\nG64 P0.05 Q0.01\nG1 X3\nG64\nG1 X4\nG20\nG64 P0.001\nG1 X5\nG64 P0\nG1 X6\n");
        assert(parsed.errors.empty());
        std::vector<ToolpathSegment> moves;
        for (const auto& s : parsed.segments) {
            assert(s.type != SegmentType::Dwell);
            if (s.type == SegmentType::CutLinear) moves.push_back(s);
        }
        assert(moves.size() == 6);
        assert(!moves[0].exact_stop && moves[1].exact_stop && !moves[2].exact_stop);
        assert(moves[0].corner_tolerance_mm == 0.0 && std::abs(moves[2].corner_tolerance_mm - 0.05) < 1e-9);
        assert(moves[3].corner_tolerance_mm == 0.0);
        assert(std::abs(moves[4].corner_tolerance_mm - 0.0254) < 1e-9);
        assert(moves[5].corner_tolerance_mm == 0.0);
        assert(GCodeParser::parse("G64 P99\n").warnings.size() == 1);

        rnr::core::MachineConfig cfg;
        for (auto& ax : cfg.axes) {
            ax.steps_per_mm       = 200.0;
            ax.max_velocity_mm_s  = 500.0;
            ax.acceleration_mm_s2 = 200.0;
            ax.deceleration_mm_s2 = 200.0;
        }
        const auto lineSeg = [](std::array<double, 4> a, std::array<double, 4> b, double feed) {
            ToolpathSegment s{};
            s.type = SegmentType::CutLinear;
            s.start_xyza_mm = a;
            s.end_xyza_mm = b;
            s.length_mm = std::hypot(b[0] - a[0], b[1] - a[1]);
            s.feed_mm_min = feed;
            return s;
        };
        const auto slice = [](const std::vector<ToolpathSegment>& segs, const rnr::core::MachineConfig& c) {
            return rnr::planner::TrajectoryPlanner::sliceProgram(segs, c);
        };

        // G61: a slice lands exactly on the joint of two moves in a straight line and the speed drops to zero
        ToolpathSegment first = lineSeg({0, 0, 0, 0}, {50, 0, 0, 0}, 3000.0);
        ToolpathSegment second = lineSeg({50, 0, 0, 0}, {100, 0, 0, 0}, 3000.0);
        const auto through = slice({first, second}, cfg);
        first.exact_stop = true;
        const auto stopped = slice({first, second}, cfg);
        assert(stopped.size() > through.size());
        bool on_joint = false;
        for (const auto& s : stopped) {
            if (s.target_pos_mm[0] == 50.0) on_joint = true;
        }
        assert(on_joint);

        // A bigger tolerance takes a square corner faster; G64 P overrides the setting
        const auto cornerSlices = [&](double setting, double g64p) {
            auto c = cfg;
            c.corner_tolerance_mm = setting;
            c.limit_curve_speed = false;
            ToolpathSegment a = lineSeg({0, 0, 0, 0}, {20, 0, 0, 0}, 6000.0);
            ToolpathSegment b = lineSeg({20, 0, 0, 0}, {20, 20, 0, 0}, 6000.0);
            a.corner_tolerance_mm = b.corner_tolerance_mm = g64p;
            return slice({a, b}, c).size();
        };
        assert(cornerSlices(1.0, 0.0) < cornerSlices(0.02, 0.0));
        assert(cornerSlices(0.02, 1.0) == cornerSlices(1.0, 0.0));

        // 5 mm radius circle in 0.2 mm moves at 6350 mm/min: measure the speed and how far the slices leave the path
        std::vector<ToolpathSegment> circle;
        constexpr double kR = 5.0;
        const int kN = static_cast<int>(2.0 * 3.14159265358979 * kR * 3.0 / 0.2);
        std::array<double, 4> prev = {kR, 0, 0, 0};
        for (int i = 1; i <= kN; ++i) {
            const double t = i * 0.2 / kR;
            const std::array<double, 4> p = {kR * std::cos(t), kR * std::sin(t), 0, 0};
            circle.push_back(lineSeg(prev, p, 6350.0));
            prev = p;
        }
        const auto measure = [&](bool limit, double& max_speed, double& max_error) {
            auto c = cfg;
            c.limit_curve_speed = limit;
            const auto slices = slice(circle, c);
            max_speed = 0.0;
            max_error = 0.0;
            for (size_t i = 1; i < slices.size(); ++i) {
                const auto& p0 = slices[i - 1].target_pos_mm;
                const auto& p1 = slices[i].target_pos_mm;
                max_speed = std::max(max_speed, std::hypot(p1[0] - p0[0], p1[1] - p0[1]) / 0.025);
                max_error = std::max(max_error, kR - std::hypot(0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])));
            }
        };
        double speed_off = 0.0, error_off = 0.0, speed_on = 0.0, error_on = 0.0;
        measure(false, speed_off, error_off);
        measure(true, speed_on, error_on);
        std::cout << "  limit off: " << speed_off << " mm/s, " << error_off << " mm; limit on: " << speed_on
                  << " mm/s, " << error_on << " mm\n";
        assert(speed_off > 100.0);                      // Full feed on a 5 mm radius without the limit
        assert(speed_on < std::sqrt(8.0 * kR * 0.02) / 0.025 * 1.05); // Slice chord within the 0.02 mm tolerance
        assert(error_on < 0.022);                       // 0.02 mm tolerance + chord of the 0.2 mm moves

        // Curve acceleration: the speed follows the square root of the percentage (tolerance large enough not to limit)
        const auto peakAtAccel = [&](int pct) {
            auto c = cfg;
            c.corner_tolerance_mm = 1.0;
            c.curve_accel_pct = pct;
            const auto slices = slice(circle, c);
            double peak = 0.0;
            for (size_t i = 1; i < slices.size(); ++i) {
                const auto& p0 = slices[i - 1].target_pos_mm;
                const auto& p1 = slices[i].target_pos_mm;
                peak = std::max(peak, std::hypot(p1[0] - p0[0], p1[1] - p0[1]) / 0.025);
            }
            return peak;
        };
        const double peak100 = peakAtAccel(100), peak200 = peakAtAccel(200), peak25 = peakAtAccel(25);
        std::cout << "  curve acceleration 25% / 100% / 200%: " << peak25 << " / " << peak100 << " / " << peak200 << " mm/s\n";
        assert(peak200 / peak100 > 1.3 && peak200 / peak100 < 1.5);
        assert(peak25 / peak100 > 0.45 && peak25 / peak100 < 0.55);

        // No sudden speed changes: the limit of a junction must not depend on where a planning window starts.
        // 30 mm radius in 0.05 mm moves with 3-decimal coordinates used to step down by 28 mm/s in one slice.
        const auto maxPathAccel = [&](const std::vector<ToolpathSegment>& segs) {
            const auto slices = slice(segs, cfg);
            double prev_speed = 0.0, worst = 0.0;
            for (size_t i = 1; i + 1 < slices.size(); ++i) {
                const auto& p0 = slices[i - 1].target_pos_mm;
                const auto& p1 = slices[i].target_pos_mm;
                const double speed = std::hypot(p1[0] - p0[0], p1[1] - p0[1]) / 0.025;
                if (i > 1) worst = std::max(worst, std::abs(speed - prev_speed) / 0.025);
                prev_speed = speed;
            }
            return worst;
        };
        const auto round3 = [](double x) { return std::round(x * 1000.0) / 1000.0; };
        std::vector<ToolpathSegment> rounded;
        std::array<double, 4> rp = {30, 0, 0, 0};
        for (int i = 1; i <= 60000; ++i) {
            const double t = i * 0.05 / 30.0;
            const std::array<double, 4> p = {round3(30.0 * std::cos(t)), round3(30.0 * std::sin(t)), 0, 0};
            rounded.push_back(lineSeg(rp, p, 6350.0));
            rp = p;
        }
        const double rounded_accel = maxPathAccel(rounded);
        std::cout << "  rounded arc max acceleration: " << rounded_accel << " mm/s2\n";
        assert(rounded_accel < 350.0);

        // G61: an arc stops once at its end, not at every chord; a subdivided move stops once too
        const auto arcSegs = [](const char* mode) {
            return GCodeParser::parse(std::string("G21 G90 F3000\nG0 X20 Y0\n") + mode + "\nG3 X-20 Y0 I-20 J0\nG64\n",
                                      {0, 0, 0, 0}, 0.3).segments;
        };
        const auto arc_g61 = arcSegs("G61");
        size_t arc_stops = 0;
        for (const auto& s : arc_g61) {
            if (s.exact_stop) ++arc_stops;
        }
        assert(arc_stops == 1 && arc_g61.back().exact_stop);
        assert(slice(arc_g61, cfg).size() < slice(arcSegs("G64"), cfg).size() + 40);

        rnr::core::SurfaceHeightMap flat_map;
        flat_map.is_valid = true;
        flat_map.points_x = 2;
        flat_map.points_y = 2;
        flat_map.min_x = 0.0;
        flat_map.max_x = 100.0;
        flat_map.min_y = 0.0;
        flat_map.max_y = 100.0;
        flat_map.grid_z = {0.0, 0.0, 0.0, 0.0};
        ToolpathSegment warp_in = lineSeg({0, 0, 0, 0}, {20, 0, 0, 0}, 3000.0);
        warp_in.exact_stop = true;
        const auto warped = rnr::planner::TrajectoryPlanner::warpToolpathWithHeightMap({warp_in}, flat_map, 2.0);
        assert(warped.size() == 10);
        for (size_t i = 0; i < warped.size(); ++i) assert(warped[i].exact_stop == (i + 1 == warped.size()));

        // Long straight moves with a small kink are not slowed down by the curve limit
        ToolpathSegment k1 = lineSeg({0, 0, 0, 0}, {100, 0, 0, 0}, 3000.0);
        ToolpathSegment k2 = lineSeg({100, 0, 0, 0}, {200, 0.1, 0, 0}, 3000.0);
        auto kc = cfg;
        const size_t kink_on = slice({k1, k2}, kc).size();
        kc.limit_curve_speed = false;
        assert(kink_on == slice({k1, k2}, kc).size());

        // Settings: range is enforced and both values are saved
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        auto ccfg = controller.getConfig();
        ccfg.corner_tolerance_mm = 100.0;
        controller.applyConfig(ccfg);
        assert(controller.getConfig().corner_tolerance_mm == rnr::core::MachineConfig::kMaxCornerToleranceMm);
        ccfg = controller.getConfig();
        ccfg.curve_accel_pct = 1000;
        controller.applyConfig(ccfg);
        assert(controller.getConfig().curve_accel_pct == rnr::core::MachineConfig::kMaxCurveAccelPct);

        const std::string ini = (std::filesystem::temp_directory_path() / "breakout_corner_settings_test.ini").string();
        rnr::config::PersistentSettings ps;
        ps.machine.corner_tolerance_mm = 0.05;
        ps.machine.limit_curve_speed   = false;
        ps.machine.curve_accel_pct     = 150;
        assert(rnr::config::SettingsManager::saveToFile(ini, ps));
        rnr::config::PersistentSettings back;
        assert(rnr::config::SettingsManager::loadFromFile(ini, back));
        assert(back.machine.corner_tolerance_mm == 0.05 && !back.machine.limit_curve_speed);
        assert(back.machine.curve_accel_pct == 150);
        std::filesystem::remove(ini);
        std::filesystem::remove(ini + ".bak");
        std::cout << "  -> PASSED: G61 stops, G64 P and the setting set the corner speed, curves stay within limits.\n";
    }
}

void test46() {
    // ========================================================================
    // Test 46: Canned cycle words: G82 P for all holes, warning for pecks without Q
    // ========================================================================
    {
        std::cout << "\n[Test 46] Testing G82 P and G73 / G83 without Q...\n";
        using rnr::core::SegmentType;
        using rnr::gcode::GCodeParser;
        const auto dwells = [](const std::string& gcode) {
            std::vector<double> out;
            for (const auto& s : GCodeParser::parse(gcode).segments) {
                if (s.type == SegmentType::Dwell) out.push_back(s.dwell_seconds);
            }
            return out;
        };
        const std::string head = "G21 G90 F300\nG0 Z10\n";
        // P on the first hole applies to all of them; a new P replaces it
        auto d = dwells(head + "G82 X5 Z-2 R2 P0.5\nX15\nX25 P0.2\nX35\nG80\n");
        assert(d.size() == 4 && d[0] == 0.5 && d[1] == 0.5 && d[2] == 0.2 && d[3] == 0.2);
        // G80 ends the cycle: the next G82 does not inherit P; a G4 inside the cycle does not change it
        assert(dwells(head + "G82 X5 Z-2 R2 P0.5\nG80\nG82 X15 Z-2 R2\nG80\n").size() == 1);
        d = dwells(head + "G82 X5 Z-2 R2 P0.5\nG4 P3\nX15\nG80\n");
        assert(d.size() == 3 && d[0] == 0.5 && d[1] == 3.0 && d[2] == 0.5);

        const auto warnings = [](const std::string& gcode) { return GCodeParser::parse(gcode).warnings.size(); };
        assert(warnings(head + "G83 X5 Z-3 R2 Q1\nX15\nG80\n") == 0);
        assert(warnings(head + "G73 X5 Z-3 R2 Q1\nX15\nG80\n") == 0);
        assert(warnings(head + "G83 X5 Z-3 R2\nX15\nG80\n") == 1); // Once per cycle, not per hole
        assert(warnings(head + "G73 X5 Z-3 R2\nG80\nG83 X15 Z-3 R2\nG80\n") == 2);
        assert(warnings(head + "G83 X5 Z-3 R2 Q1\nG80\nG83 X15 Z-3 R2\nG80\n") == 1); // Q does not outlive the cycle
        assert(warnings(head + "G81 X5 Z-3 R2\nG80\n") == 0);
        std::cout << "  -> PASSED: G82 P lasts for the cycle, pecks without Q warn.\n";
    }
}

void test47() {
    // ========================================================================
    // Test 47: Only G61 / G61.1 select exact stop: common CAM header codes must not
    // ========================================================================
    {
        std::cout << "\n[Test 47] Testing that CAM header codes do not select exact stop...\n";
        using rnr::gcode::GCodeParser;
        const auto stopsOf = [](const std::string& gcode) {
            std::vector<bool> out;
            for (const auto& s : GCodeParser::parse(gcode).segments) out.push_back(s.exact_stop);
            return out;
        };
        const std::string moves = "G1 X10 F1000\nG1 X20 Y1\n";
        for (const char* code : {"G15", "G17", "G21", "G40", "G49", "G50", "G54", "G69", "G80", "G90", "G91.1", "G94"}) {
            assert(stopsOf(std::string(code) + "\nG21 G90\n" + moves) == std::vector<bool>({false, false}));
        }
        // The header of the Vectric post that exposed this
        assert(stopsOf("G00 G21 G17 G90 G40 G49 G80\nG71 G91.1\nG94\n" + moves) == std::vector<bool>({false, false}));
        // G40 does not cancel an exact stop that is in effect, G64 does
        assert(stopsOf("G21 G90 G61\nG40\n" + moves) == std::vector<bool>({true, true}));
        assert(stopsOf("G21 G90 G61.1\n" + moves + "G64\nG1 X30\n") == std::vector<bool>({true, true, false}));

        // The planner carries speed through a small corner after such a header
        rnr::core::MachineConfig cfg;
        cfg.is_mill = true;
        for (auto& axis : cfg.axes) {
            axis.steps_per_mm = 200.0;
            axis.max_velocity_mm_s = 100.0;
            axis.acceleration_mm_s2 = 300.0;
            axis.deceleration_mm_s2 = 300.0;
        }
        const auto segs = GCodeParser::parse("G00 G21 G17 G90 G40 G49 G80\nG1 X10 F1000\nG1 X20 Y0.5\n").segments;
        const auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(segs, cfg);
        double corner_speed = 0.0;
        for (size_t i = 1; i < slices.size(); ++i) {
            if (slices[i].target_pos_mm[0] >= 10.0) {
                corner_speed = (slices[i].target_pos_mm[0] - slices[i - 1].target_pos_mm[0]) / 0.025;
                break;
            }
        }
        std::cout << "  speed at the corner: " << corner_speed << " mm/s\n";
        assert(corner_speed > 10.0);
        std::cout << "  -> PASSED: only G61 / G61.1 select exact stop.\n";
    }
}

void test48() {
    // ========================================================================
    // Test 48: Go to line in the program panel, by line number or N number
    // ========================================================================
    {
        std::cout << "\n[Test 48] Testing go to line...\n";
        using rnr::ui::findGCodeLine;
        const std::vector<std::string> numbered = {"(header)", "N120 G00 G21", "N130 G71", "  n140 T3 M06", "G1 X1", "N0150 G1 X2", "N3 G1 X3"};
        assert(findGCodeLine(numbered, "N130") == 2u);
        assert(findGCodeLine(numbered, "n130") == 2u);
        assert(findGCodeLine(numbered, " 130 ") == 2u); // Plain number: the N number shown in the listing
        assert(findGCodeLine(numbered, "140") == 3u);   // Lower case, indented
        assert(findGCodeLine(numbered, "150") == 5u);   // Leading zero in the file
        assert(findGCodeLine(numbered, "3") == 6u);     // N3 wins over the third line of the file
        assert(findGCodeLine(numbered, "5") == 4u);     // No N5: the fifth line of the file
        assert(!findGCodeLine(numbered, "N5"));         // An N word is only ever an N number
        assert(!findGCodeLine(numbered, "99"));
        assert(!findGCodeLine(numbered, "0"));
        assert(!findGCodeLine(numbered, ""));
        assert(!findGCodeLine(numbered, "  "));
        assert(!findGCodeLine(numbered, "N"));
        assert(!findGCodeLine(numbered, "12a"));
        assert(!findGCodeLine(numbered, "-3"));
        assert(!findGCodeLine(numbered, "99999999999999999999999"));

        const std::vector<std::string> plain = {"G21", "G90", "G1 X1 F100"};
        assert(findGCodeLine(plain, "1") == 0u);
        assert(findGCodeLine(plain, "3") == 2u);
        assert(!findGCodeLine(plain, "4"));
        assert(!findGCodeLine(plain, "N1"));
        assert(!findGCodeLine({}, "1"));
        std::cout << "  -> PASSED: line numbers and N numbers are found.\n";
    }
}

void test50() {
    // ========================================================================
    // Test 50: Program check after loading: parse problems plus the Cycle Start checks against the machine
    // ========================================================================
    {
        std::cout << "\n[Test 50] Testing the program check (homing, tool table, soft limits, feed, spindle)...\n";
        using namespace rnr::core;
        using rnr::gcode::GCodeParser;
        const auto run = [](const std::string& gcode, const MachineConfig* cfg, MachineCheckInput in) {
            auto p = std::make_shared<const rnr::gcode::ParseResult>(GCodeParser::parse(gcode));
            in.frame.machine_targets = std::shared_ptr<const std::vector<MachineTarget>>(p, &p->machine_targets);
            return buildProgramCheck(p, cfg, in);
        };
        const auto find = [](const ProgramCheckResult& r, ProgramIssueKind kind,
                             const std::string& part) -> const ProgramIssue* {
            for (const auto& i : r.issues) {
                if (i.kind == kind && i.text.find(part) != std::string::npos) return &i;
            }
            return nullptr;
        };
        MachineCheckInput in;
        in.frame.enabled = true;
        in.homed = {true, true, true, true};
        const MachineConfig base;

        // A clean program has no issues, also without M30; an M3 / M4 without a later M5 gets a warning on its line
        assert(run("G21 G90\nG0 X10 Y10\nG1 X20 F500\nM30\n", &base, in).issues.empty());
        assert(run("G21 G90\nG0 X10 Y10\nG1 X20 F500\n", &base, in).issues.empty());
        assert(run("G21 G90\nM3\nG1 X20 F500\nM5\nM30\n", &base, in).issues.empty());
        // M2 / M30 switch the spindle off as well
        assert(run("G21 G90\nM3\nG1 X20 F500\nM5\nM4\nG1 X0\nM30\n", &base, in).issues.empty());
        assert(run("G21 G90\nM3\nG1 X20 F500\nM2\n", &base, in).issues.empty());
        {
            const auto r = run("G21 G90\nM3\nG1 X20 F500\nM5\nM4\nG1 X0\n", &base, in);
            assert(r.warning_count == 1 && r.issues.size() == 1);
            const auto* w = find(r, ProgramIssueKind::Warning, "M3 / M4 is not followed by M5");
            assert(w && w->line == 4 && w->text.find("torch") != std::string::npos);
        }

        // Parse-only (no machine settings): parse errors and warnings, sorted by line
        {
            const auto r = run("G21 G90\nM100\nG41 D1\nG1 X10 F500\n", nullptr, in);
            assert(r.error_count == 1 && r.warning_count == 1 && r.blocker_count == 0);
            assert(r.issues[0].line == 1 && r.issues[0].kind == ProgramIssueKind::Warning);
            assert(r.issues[1].line == 2 && r.issues[1].kind == ProgramIssueKind::Error);
        }

        // Soft limits: every line out of range is counted, the Cycle Start message keeps its old wording
        {
            MachineConfig cfg = base;
            cfg.axes[0].soft_limits_enabled = true;
            cfg.axes[0].soft_limit_min_mm = -1.0;
            cfg.axes[0].soft_limit_max_mm = 50.0;
            const std::string prog = "G21 G90\nG0 X100\nG1 X120 F500\nG0 X0\nM30\n";
            const auto r = run(prog, &cfg, in);
            assert(r.blocker_count == 1);
            const auto* b = find(r, ProgramIssueKind::StartBlocker, "X goes to 120.000 mm");
            assert(b && b->line == 1 && b->text.find("(1 more line)") != std::string::npos);
            const auto m = checkProgramAgainstMachine(GCodeParser::parse(prog), cfg, in);
            assert(m.start_refusal.find("line 2 exceeds soft limits") != std::string::npos);

            MachineCheckInput shifted = in; // G54 X-60: X100 is inside, X120 is still 10 mm above
            MachineConfig wide = cfg;         // (and G0 X0 goes to -60, inside a wider lower limit)
            wide.axes[0].soft_limit_min_mm = -100.0;
            shifted.frame.workspace_offsets[0][0] = -60.0;
            const auto r2 = run(prog, &wide, shifted);
            const auto* b2 = find(r2, ProgramIssueKind::StartBlocker, "X goes to 60.000 mm");
            assert(r2.blocker_count == 1 && b2 && b2->line == 2 && b2->text.find("more line") == std::string::npos);

            MachineCheckInput not_homed = in; // Soft limits only apply to homed axes
            not_homed.homed = {false, true, true, true};
            assert(run(prog, &cfg, not_homed).blocker_count == 0);
        }

        // Homing: machine coordinates on an axis that is not homed (not in the simulator)
        {
            MachineCheckInput not_homed = in;
            not_homed.homed = {false, false, false, false};
            const std::string prog = "G21 G90\nG0 X5\nG53 G0 Z0\nM30\n";
            const auto r = run(prog, &base, not_homed);
            const auto* b = find(r, ProgramIssueKind::StartBlocker, "machine coordinates (G53 / G28) on Z");
            assert(r.blocker_count == 1 && b && b->line == 2);
            not_homed.simulator = true;
            assert(run(prog, &base, not_homed).blocker_count == 0);
        }

        // Tool table: every missing length and every tool change without its G43 is listed
        {
            MachineConfig cfg = base;
            cfg.tool_table.mode = ToolLengthMode::ToolTable;
            const std::string prog = "G21 G90\nT1 M6\nG43 H1\nG0 X0\nT2 M6\nG0 X5\nM30\n";
            const auto r = run(prog, &cfg, in);
            assert(find(r, ProgramIssueKind::StartBlocker, "T1 has no length"));
            const auto* b = find(r, ProgramIssueKind::StartBlocker, "tool change to T2 is not followed by G43 H2");
            assert(b && b->line == 4);
            const auto m = checkProgramAgainstMachine(GCodeParser::parse(prog), cfg, in);
            assert(m.start_refusal.find("no length") != std::string::npos);
        }

        // Feed faster than the axes: one warning on the first line, with the number of other lines
        {
            MachineConfig cfg = base;
            cfg.axes[0].max_velocity_mm_s = 10.0; // 600 mm/min
            const auto r = run("G21 G90\nG1 X10 F2000\nG1 X20\nG1 Y10\nM30\n", &cfg, in);
            const auto* w = find(r, ProgramIssueKind::Warning, "F2000 mm/min is faster than the axes");
            assert(r.warning_count == 1 && w && w->line == 1);
            assert(w->text.find("at most 600 mm/min") != std::string::npos);
            assert(w->text.find("(1 more line)") != std::string::npos);
        }

        // BreakoutMill: spindle speeds outside the VFD range, and cutting with the spindle off
        {
            MachineConfig cfg = base;
            cfg.is_mill = true;
            cfg.vfd.enabled = true;
            cfg.vfd.max_rpm = 24000;
            cfg.vfd.min_rpm = 6000;
            const auto r = run("G21 G90\nM3 S30000\nG1 X10 F500\nS1000\nG1 X20\nM5\nM30\n", &cfg, in);
            assert(r.warning_count == 2);
            assert(find(r, ProgramIssueKind::Warning, "S30000 is above the VFD's maximum speed (24000 RPM)"));
            assert(find(r, ProgramIssueKind::Warning, "S1000 is below the VFD's minimum speed (6000 RPM)"));

            cfg.vfd.enabled = false;
            // Feed moves at or above work Z0 with the spindle off (clearance height, positioning) are fine
            const auto r2 = run("G21 G90\nG1 Z5 F2400\nG1 X10 Z0\nM3 S10000\nG1 Z-1\nG1 X20\nM5\nM30\n", &cfg, in);
            assert(!find(r2, ProgramIssueKind::Warning, "spindle is off"));

            // A plunge below Z0 before M3 warns once, on the plunge
            const auto r3 = run("G21 G90\nG1 Z5 F500\nG1 Z-1\nG1 X10\nM3 S10000\nG1 X20\nM5\nM30\n", &cfg, in);
            const auto* w = find(r3, ProgramIssueKind::Warning, "spindle is off");
            assert(r3.warning_count == 1 && w && w->line == 2);

            // A sideways feed that starts below Z0 (after a rapid down) warns too
            const auto r4 = run("G21 G90\nG0 Z-1\nG1 X10 F500\nM3 S10000\nM5\nM30\n", &cfg, in);
            const auto* w4 = find(r4, ProgramIssueKind::Warning, "spindle is off");
            assert(w4 && w4->line == 2);
        }

        // A slaved A motor: programmed A moves are not made
        {
            MachineConfig cfg = base;
            cfg.gantry_slave = GantrySlave::AFollowsX;
            const auto r = run("G21 G90\nG1 A10 F500\nM30\n", &cfg, in);
            const auto* w = find(r, ProgramIssueKind::Warning, "A moves are not made");
            assert(w && w->line == 1);
        }

        // The controller keeps the check of the loaded program and recomputes it only when an input changes
        {
            RnRController controller;
            assert(controller.resetFromEStop());
            controller.loadGCodeString("G21 G90\nM3\nG0 X10\nG1 X20 F500\n");
            const auto c1 = controller.getProgramCheck();
            assert(c1 && c1->program == controller.getProgram());
            assert(find(*c1, ProgramIssueKind::Warning, "not followed by M5"));
            assert(controller.getProgramCheck() == c1); // Cached

            auto cfg = controller.getConfig();
            cfg.axes[0].soft_limits_enabled = true;
            cfg.axes[0].soft_limit_min_mm = -1.0;
            cfg.axes[0].soft_limit_max_mm = 15.0;
            controller.applyConfig(cfg);
            controller.homeAxis(Axis::X);
            for (int i = 0; i < 400 && controller.getTelemetry().state != ControllerState::Idle; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            assert(controller.getTelemetry().axis_homed[0]);
            const auto c2 = controller.getProgramCheck();
            assert(c2 != c1 && c2->blocker_count == 1);
            controller.startJob(); // Refused with the same problem the panel lists
            assert(controller.getTelemetry().active_fault_message.find("exceeds soft limits") != std::string::npos);

            controller.loadGCodeString("G21 G90\nG41 D1\nG1 X10 F500\nM30\n");
            const auto c3 = controller.getProgramCheck();
            assert(c3->program == controller.getProgram() && c3->error_count == 1);
        }
        std::cout << "  -> PASSED: every problem is listed after loading, and Cycle Start refuses the same ones.\n";
    }
}

void test51() {
    // ========================================================================
    // Test 51: G68 / G69 coordinate rotation (Mach3: G68 A<x> B<y> R<degrees> [I])
    // ========================================================================
    {
        std::cout << "\n[Test 51] Testing G68 / G69 coordinate rotation...\n";
        using rnr::gcode::GCodeParser;
        const auto endOf = [](const std::string& gcode) {
            const auto r = GCodeParser::parse(gcode);
            assert(!r.hasErrors());
            assert(!r.segments.empty());
            return r.segments.back().end_xyza_mm;
        };
        const auto near = [](const std::array<double, 4>& p, double x, double y) {
            return std::abs(p[0] - x) < 1e-6 && std::abs(p[1] - y) < 1e-6;
        };
        const auto hasError = [](const std::string& gcode, const std::string& part) {
            for (const auto& e : GCodeParser::parse(gcode).errors) {
                if (e.text.find(part) != std::string::npos) return true;
            }
            return false;
        };

        assert(near(endOf("G21 G90\nG68 A0 B0 R90\nG1 X10 Y0 F500\n"), 0.0, 10.0));        // Counterclockwise
        assert(near(endOf("G21 G90\nG68 A10 B10 R180\nG0 X20 Y10\n"), 0.0, 10.0));          // About (A, B)
        assert(near(endOf("G21 G90\nG68 A0 B0 R90\nG0 X10\nG91\nG0 X5\n"), 0.0, 15.0));    // G91 moves turn too
        assert(near(endOf("G21 G90\nG68 A0 B0 R90\nG0 X10\nG69\nG0 X10\n"), 10.0, 10.0));  // G69 cancels
        assert(near(endOf("G21 G90\nG68 A0 B0 R45\nG68 I0 R45\nG0 X10\n"), 0.0, 10.0));    // I: relative angle
        assert(near(endOf("G21 G90\nG68 A10 B0 R90\nG68 R180\nG0 X30 Y0\n"), -10.0, 0.0)); // Centre kept
        assert(near(endOf("G20 G90\nG68 A1 B0 R180\nG0 X3 Y0\n"), -25.4, 0.0));            // Inches
        assert(near(endOf("G21 G90\nG0 Z5\nG68 A0 B0 R90\nG81 X10 Y0 Z-1 R2 F100\nG80\n"), 0.0, 10.0)); // Drill hole

        // Arcs: I/J offsets turn with the program, the end lands on the rotated point
        {
            const auto r = GCodeParser::parse("G21 G90\nG68 A0 B0 R90\nG0 X10 Y0\nG3 X0 Y10 I-10 J0 F500\n");
            assert(!r.hasErrors());
            assert(near(r.segments.back().end_xyza_mm, -10.0, 0.0));
            for (const auto& s : r.segments) {
                if (s.type == rnr::core::SegmentType::CutArcCCW) {
                    assert(std::abs(std::hypot(s.end_xyza_mm[0], s.end_xyza_mm[1]) - 10.0) < 0.01);
                }
            }
        }

        // A Z-only move keeps X and Y exactly
        {
            const auto r = GCodeParser::parse("G21 G90\nG68 A0 B0 R30\nG0 X10 Y3\nG1 Z-1 F100\n");
            const auto& a = r.segments[r.segments.size() - 2].end_xyza_mm;
            const auto& b = r.segments.back().end_xyza_mm;
            assert(a[0] == b[0] && a[1] == b[1] && b[2] == -1.0);
        }

        // G68's A, B and I words are not the A axis, an unsupported word or an arc offset
        {
            const auto r = GCodeParser::parse("G21 G90\nG68 A5 B5 I0 R30\nG1 X10 F500\nM30\n");
            assert(!r.hasErrors() && r.warnings.empty());
            for (const auto& s : r.segments) assert(s.end_xyza_mm[3] == 0.0);
            rnr::core::MachineConfig cfg;
            cfg.gantry_slave = rnr::core::GantrySlave::AFollowsX;
            rnr::core::MachineCheckInput in;
            in.frame.enabled = true;
            const auto check = rnr::core::buildProgramCheck(std::make_shared<const rnr::gcode::ParseResult>(r), &cfg, in);
            for (const auto& i : check.issues) assert(i.text.find("A moves") == std::string::npos);
        }
        assert(hasError("G21 G90\nG1 X10 B5 F500\n", "Unsupported word 'B'")); // B outside G68 is still an error

        // Not supported: axis words on the G68 line, G92 while rotated, G18 / G19 arcs while rotated
        assert(hasError("G21 G90\nG68 A0 B0 R10 X5\n", "G68 cannot be combined"));
        assert(hasError("G21 G90\nG68 R10\nG92 X0\n", "G92 is not supported while G68"));
        assert(hasError("G21 G90\nG18\nG68 R10\nG2 X10 Z0 I5 K0 F500\n", "G18/G19 is not supported while G68"));

        // MDI: each command is parsed alone, so G68 is refused there
        {
            rnr::core::RnRController controller;
            assert(controller.resetFromEStop());
            assert(!controller.executeMdiCommand("G68 A0 B0 R30"));
            assert(controller.getTelemetry().active_fault_message.find("G68") != std::string::npos);
        }
        std::cout << "  -> PASSED: G68 turns the programmed XY points about (A, B); G69 cancels.\n";
    }
}
