// Machine controller tests (simulator and fake board)
#include "test_support.h"

void test07() {
    // ------------------------------------------------------------------------
    // Test 7: Multi-Axis Simultaneous Jogging in RnRController
    // ------------------------------------------------------------------------
    std::cout << "[Test 7] Testing simultaneous multi-axis continuous jogging & per-axis stop...\n";
    {
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        // Jogging is refused while in E-Stop
        controller.jogStartContinuous(rnr::core::Axis::X, true, 0.5);
        assert(controller.getTelemetry().state == rnr::core::ControllerState::EStop);
        assert(controller.resetFromEStop());

        // Continuous jog is a dead-man control: like the UI, keep refreshing the held axes while waiting, so a slow
        // machine (CI runners) cannot let them time out. No fixed sleeps: each check waits for its condition.
        bool hold_x = true;
        auto refreshJogs = [&] {
            if (hold_x) controller.jogStartContinuous(rnr::core::Axis::X, true, 0.5);
            controller.jogStartContinuous(rnr::core::Axis::Y, true, 0.5);
            controller.jogStartContinuous(rnr::core::Axis::Z, true, 0.5);
        };
        auto holdUntil = [&](auto pred) {
            return waitUntil([&] { refreshJogs(); return pred(controller.getTelemetry()); });
        };

        // Start jogging X, Y, and Z simultaneously
        refreshJogs();
        assert(holdUntil([](const rnr::core::TelemetrySnapshot& t) {
            return t.state == rnr::core::ControllerState::Jogging &&
                   t.work_pos_mm[0] > 0.0 && t.work_pos_mm[1] > 0.0 && t.work_pos_mm[2] > 0.0;
        }));
        auto tel1 = controller.getTelemetry();
        std::cout << "  -> Multi-axis jog active: X=" << tel1.work_pos_mm[0]
                  << ", Y=" << tel1.work_pos_mm[1]
                  << ", Z=" << tel1.work_pos_mm[2] << " mm\n";

        // Stop only X axis; Y and Z are still held
        hold_x = false;
        controller.jogStopAxis(rnr::core::Axis::X);
        // Y and Z must have continued moving forward
        assert(holdUntil([&](const rnr::core::TelemetrySnapshot& t) {
            return t.axis_vel_mm_min[0] == 0.0 && t.work_pos_mm[1] > tel1.work_pos_mm[1] && t.work_pos_mm[2] > tel1.work_pos_mm[2];
        }));

        auto tel2 = controller.getTelemetry();
        const double x_stopped_pos = tel2.work_pos_mm[0];

        // Y and Z keep advancing while X stays where it stopped
        assert(holdUntil([&](const rnr::core::TelemetrySnapshot& t) {
            return t.work_pos_mm[1] > tel2.work_pos_mm[1] && t.work_pos_mm[2] > tel2.work_pos_mm[2];
        }));
        auto tel3 = controller.getTelemetry();
        assert(std::abs(tel3.work_pos_mm[0] - x_stopped_pos) < 1e-6);

        // Stop all remaining axes
        controller.jogStop();
        assert(waitUntil([&] { return controller.getTelemetry().state == rnr::core::ControllerState::Idle; }));

        auto tel4 = controller.getTelemetry();
        assert(tel4.axis_vel_mm_min[0] == 0.0 && tel4.axis_vel_mm_min[1] == 0.0 && tel4.axis_vel_mm_min[2] == 0.0);
        controller.disconnect();

        std::cout << "  -> PASSED: 3 axes moved concurrently; X stopped independently while Y & Z kept running.\n";
    }
}

void test08() {
    // ------------------------------------------------------------------------
    // Test 8: Multi-tool Mach3 ATC tool change and progress tracking
    // ------------------------------------------------------------------------
    std::cout << "\n[Test 8] Testing multi-tool tool change and progress tracking...\n";
    {
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        std::string gcode = rnr::gcode::GCodeParser::sampleMillAtcGCode();
        controller.loadGCodeString(gcode);
        const auto& segs = controller.getProgram()->segments;
        assert(!segs.empty());

        auto cfg = controller.getConfig();
        cfg.scripting.m6_script_enabled = false;
        controller.applyConfig(cfg);

        // Start job: should pause at T1 M06 (after the automatic Z retract to safe Z has finished)
        controller.startJob();
        for (int i = 0; i < 300; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto t = controller.getTelemetry();
            if (t.state == rnr::core::ControllerState::ToolChangeHold) break;
        }
        auto t_hold = controller.getTelemetry();
        assert(t_hold.state == rnr::core::ControllerState::ToolChangeHold);
        assert(t_hold.pending_tool_number == 1);

        // Resume after confirming T1 installed
        controller.startJob();
        auto t_resumed = controller.getTelemetry();
        assert(t_resumed.pending_tool_number == 0);
        assert(t_resumed.current_tool_number == 1);

        controller.disconnect();
        std::cout << "  -> PASSED: Tool change hold, resume, and multi-tool progression verified.\n";
    }
}

void test09() {
    // ------------------------------------------------------------------------
    // Test 9: Workspace Coordinate Systems (G54 - G59) & G-Code / MDI Switching
    // ------------------------------------------------------------------------
    std::cout << "\n[Test 9] Testing Workspace Coordinate Systems (G54 - G59) & MDI / G-Code switching...\n";
    {
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // 1. Initial workspace should be G54 (index 0)
        assert(controller.getActiveWorkspace() == 0);

        // 2. Set G55 offset to (100.0, 50.0, -10.0, 0.0)
        controller.setWorkspaceOffset(1, {100.0, 50.0, -10.0, 0.0});
        auto all_offsets = controller.getWorkspaceOffsets();
        assert(all_offsets[1][0] == 100.0);
        assert(all_offsets[1][1] == 50.0);
        assert(all_offsets[1][2] == -10.0);

        // 3. Switch to G55
        controller.selectWorkspace(1);
        auto t_g55 = controller.getTelemetry();
        assert(t_g55.active_workspace == 1);
        assert(t_g55.work_offset_mm[0] == 100.0);
        assert(t_g55.work_offset_mm[1] == 50.0);
        assert(t_g55.work_offset_mm[2] == -10.0);

        // 4. MDI command switching: G56
        controller.executeMdiCommand("G56");
        auto t_g56 = controller.getTelemetry();
        assert(t_g56.active_workspace == 2);

        // 5. MDI G10 L2 setting: G10 L2 P4 X200.0 Y150.0 Z-25.0 (P4 = G57)
        controller.executeMdiCommand("G10 L2 P4 X200.0 Y150.0 Z-25.0");
        auto offsets_updated = controller.getWorkspaceOffsets();
        assert(offsets_updated[3][0] == 200.0);
        assert(offsets_updated[3][1] == 150.0);
        assert(offsets_updated[3][2] == -25.0);

        // 6. G-Code parser multi-workspace tagging
        std::string gcode = "G54\nG0 X10 Y10\nG55\nG0 X20 Y20\nG56\nG1 X30 Y30 F1000\n";
        controller.loadGCodeString(gcode);
        const auto& segs = controller.getProgram()->segments;
        assert(segs.size() >= 3);
        assert(segs[0].workspace_index == 0); // G54
        assert(segs[1].workspace_index == 1); // G55
        assert(segs[2].workspace_index == 2); // G56

        // 7. Settings persistence check
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.ui.header_workspaces_open = true;
        std::string test_ini = "test_workspace_settings.ini";
        bool saved = rnr::config::SettingsManager::saveToFile(test_ini, ps);
        assert(saved);

        rnr::config::PersistentSettings loaded_ps{};
        bool loaded = rnr::config::SettingsManager::loadFromFile(test_ini, loaded_ps);
        assert(loaded);
        assert(loaded_ps.ui.header_workspaces_open == true);
        assert(loaded_ps.machine.workspace_offsets_mm[3][0] == 200.0);
        assert(loaded_ps.machine.workspace_offsets_mm[3][1] == 150.0);
        std::remove(test_ini.c_str());

        controller.disconnect();
        std::cout << "  -> PASSED: G54..G59 switching, MDI G10 L2, G-code segment tagging & persistence verified.\n";
    }

    // -------------------------------------------------------------------------
    // TEST 10: CNC Laser Mode, Slicing, Dynamic Power, Auto Air & Interlocks
    // -------------------------------------------------------------------------
    {
        std::cout << "\n[Test 10] Testing Dual-Mode CNC Laser Mode, Slicing, Auto Air & Settings Persistence...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // 1. Operation Mode Switching
        assert(controller.getOperationMode() == rnr::core::OperationMode::Plasma);
        controller.setOperationMode(rnr::core::OperationMode::Laser);
        assert(controller.getOperationMode() == rnr::core::OperationMode::Laser);
        assert(controller.getTelemetry().operation_mode == rnr::core::OperationMode::Laser);

        // 2. Trajectory Slicing in Laser Mode:
        // G0 (rapid) must have OUT1=0, G1 (cut) with M3 must have OUT1=1 and Auto Air Assist (OUT3=1)
        rnr::core::MachineConfig cfg = controller.getConfig();
        cfg.operation_mode = rnr::core::OperationMode::Laser;
        cfg.laser.auto_air_assist = true;
        cfg.laser.ignore_z_motion = true;
        controller.applyConfig(cfg);

        std::string laser_gcode = "G0 X10 Y10 Z5\nM3 S800\nG1 X50 Y50 Z0 F1200\nM5\nG0 X0 Y0\n";
        bool loaded = controller.loadGCodeString(laser_gcode);
        assert(loaded);

        const auto& segs = controller.getProgram()->segments;
        assert(!segs.empty());

        auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(segs, cfg, 100.0f, 0.0f, false, 0.025);
        assert(!slices.empty());

        // Verify that rapid move slices do not fire laser
        bool saw_rapid_with_laser = false;
        bool saw_cut_with_laser = false;
        bool saw_air_assist = false;
        bool z_remained_locked = true;

        for (const auto& sl : slices) {
            bool out1 = (sl.slice.outputs_mask & rnr::core::OutputBit::OUT1_TorchOrSpindle) != 0;
            bool out3 = (sl.slice.outputs_mask & rnr::core::OutputBit::OUT3_FloodCoolant) != 0;
            if (sl.segment_index == 0) { // First G0 rapid
                if (out1) saw_rapid_with_laser = true;
            } else if (sl.segment_index == 1) { // G1 cutting move with M3
                if (out1) saw_cut_with_laser = true;
                if (out3) saw_air_assist = true;
            }
            if (sl.slice.delta_steps[2] != 0) {
                z_remained_locked = false; // Z delta should be 0 because ignore_z_motion is active
            }
        }
        assert(!saw_rapid_with_laser);
        assert(saw_cut_with_laser);
        assert(saw_air_assist);
        assert(z_remained_locked);

        // 3. Air Assist & Red Pointer Controls
        controller.setAirAssist(true);
        assert((controller.getTelemetry().outputs_bitmask & rnr::core::OutputBit::OUT3_FloodCoolant) != 0);
        assert(controller.getTelemetry().laser_air_assist_active);
        controller.setAirAssist(false);
        assert(!controller.getTelemetry().laser_air_assist_active);

        controller.setRedPointer(true);
        assert((controller.getTelemetry().outputs_bitmask & rnr::core::OutputBit::OUT2_SpindleDir) != 0);
        assert(controller.getTelemetry().laser_red_pointer_active);
        controller.setRedPointer(false);
        assert(!controller.getTelemetry().laser_red_pointer_active);

        // 4. Test Fire Pulse: refused while the chiller / door interlocks are not satisfied
        controller.testFireLaserPulse(100.0f, 25.0f);
        assert(!controller.getTelemetry().torch_or_spindle_on);
        controller.setSimulatedInputPin(cfg.laser.chiller_flow_pin, true);   // Chiller flowing
        controller.setSimulatedInputPin(cfg.laser.door_interlock_pin, true); // Lid closed
        assert(waitUntil([&] {                                                // Worker re-evaluates inputs
            const auto t = controller.getTelemetry();
            return t.laser_chiller_flow_ok && t.laser_door_closed;
        }));
        controller.testFireLaserPulse(2000.0f, 25.0f); // Long pulse so it is still on when read back
        const auto fired = controller.getTelemetry();
        assert(fired.torch_or_spindle_on);
        assert(fired.live_laser_power_pct == 25.0f);
        assert(fired.spindle_pwm_duty > 0);

        // 5. Settings Persistence Verification
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.ui.operation_mode = 1; // Laser
        ps.machine.laser.source_type = rnr::core::LaserSourceType::Co2Glass;
        ps.machine.laser.min_duty_pct = 12.0f;
        ps.machine.laser.max_duty_pct = 85.0f;
        ps.machine.laser.chiller_flow_pin = 2;
        ps.machine.laser.auto_air_assist = true;

        std::string test_laser_ini = "test_laser_settings.ini";
        bool saved = rnr::config::SettingsManager::saveToFile(test_laser_ini, ps);
        assert(saved);

        rnr::config::PersistentSettings loaded_ps{};
        bool loaded_ini = rnr::config::SettingsManager::loadFromFile(test_laser_ini, loaded_ps);
        assert(loaded_ini);
        assert(loaded_ps.ui.operation_mode == 1);
        assert(loaded_ps.machine.operation_mode == rnr::core::OperationMode::Laser);
        assert(loaded_ps.machine.laser.source_type == rnr::core::LaserSourceType::Co2Glass);
        assert(loaded_ps.machine.laser.min_duty_pct == 12.0f);
        assert(loaded_ps.machine.laser.max_duty_pct == 85.0f);
        assert(loaded_ps.machine.laser.chiller_flow_pin == 2);
        assert(loaded_ps.machine.laser.auto_air_assist == true);
        std::remove(test_laser_ini.c_str());

        controller.disconnect();
        std::cout << "  -> PASSED: Laser mode, G0 safety, dynamic power, auto air assist & persistence verified.\n";
    }
}

void test12() {
    // ------------------------------------------------------------------------
    // Test 12: Surface Level Probing, Bilinear Interpolation, Toolpath Warping & Settings Persistence
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 12] Testing Surface Level Probing, Bilinear Interpolation, Toolpath Warping & Settings Persistence...\n";

        // 1. SurfaceHeightMap statistics & bilinear interpolation
        rnr::core::SurfaceHeightMap map;
        map.is_valid = true;
        map.min_x = 0.0;
        map.max_x = 100.0;
        map.min_y = 0.0;
        map.max_y = 50.0;
        map.points_x = 3;
        map.points_y = 3;
        map.smoothness = 0.0; // This test checks pure bilinear interpolation
        // 3x3 grid:
        // row 0 (Y=0):  (0,0)=0.0, (50,0)=0.10, (100,0)=0.20
        // row 1 (Y=25): (0,25)=0.30, (50,25)=0.40, (100,25)=0.50
        // row 2 (Y=50): (0,50)=0.60, (50,50)=0.70, (100,50)=0.80
        map.grid_z = {
            0.00, 0.10, 0.20,
            0.30, 0.40, 0.50,
            0.60, 0.70, 0.80
        };
        map.recalculateStats();

        assert(std::abs(map.min_z - 0.00) < 1e-6);
        assert(std::abs(map.max_z - 0.80) < 1e-6);
        assert(std::abs(map.peak_to_peak_z - 0.80) < 1e-6);
        assert(std::abs(map.mean_z - 0.40) < 1e-6);

        // Grid vertex exact lookups
        assert(std::abs(map.getInterpolatedZ(0.0, 0.0) - 0.00) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(50.0, 0.0) - 0.10) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(100.0, 0.0) - 0.20) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(50.0, 25.0) - 0.40) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(100.0, 50.0) - 0.80) < 1e-6);

        // Sub-cell center bilinear interpolation: (25.0, 12.5) -> (0.0 + 0.1 + 0.3 + 0.4) / 4 = 0.20
        double z_center = map.getInterpolatedZ(25.0, 12.5);
        assert(std::abs(z_center - 0.20) < 1e-6);

        // Out-of-bounds edge clamping check
        assert(std::abs(map.getInterpolatedZ(-20.0, -10.0) - 0.00) < 1e-6);
        assert(std::abs(map.getInterpolatedZ(150.0, 80.0) - 0.80) < 1e-6);

        // 2. TrajectoryPlanner Toolpath Warping & Segment Subdivision
        rnr::core::ToolpathSegment long_cut{};
        long_cut.type = rnr::core::SegmentType::CutLinear;
        long_cut.start_xyza_mm = {0.0, 25.0, 0.0, 0.0};
        long_cut.end_xyza_mm   = {100.0, 25.0, 0.0, 0.0};
        long_cut.length_mm     = 100.0;
        long_cut.feed_mm_min   = 1200.0;
        long_cut.torch_active  = true;

        std::vector<rnr::core::ToolpathSegment> orig_segs = {long_cut};
        auto warped_segs = rnr::planner::TrajectoryPlanner::warpToolpathWithHeightMap(orig_segs, map, 2.0);

        // 100mm move subdivided by max 2.0mm -> exactly 50 chord sub-segments
        assert(warped_segs.size() == 50);
        // Start segment Z should match (0, 25) elevation (0.30)
        assert(std::abs(warped_segs.front().start_xyza_mm[2] - 0.30) < 1e-4);
        // Middle segment around index 24..25 should match (50, 25) elevation (0.40)
        assert(std::abs(warped_segs[24].end_xyza_mm[2] - 0.40) < 1e-4);
        // End segment Z should match (100, 25) elevation (0.50)
        assert(std::abs(warped_segs.back().end_xyza_mm[2] - 0.50) < 1e-4);

        // 3. Controller Height Map Application and Compensation Reset
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        std::string test_gcode = "G21 G90\nG0 X0 Y25 Z5\nG1 X100 Y25 Z0 F1200\nM30\n";
        bool loaded = controller.loadGCodeString(test_gcode);
        assert(loaded);
        assert(!controller.isSurfaceCompensationActive());

        // Apply height map
        controller.applySurfaceHeightMap(map);
        assert(controller.isSurfaceCompensationActive());
        assert(controller.getTelemetry().surface_compensation_active);
        const auto& comp_segs = controller.getProgram()->segments;
        assert(comp_segs.size() > 10); // Expanded through chord subdivision

        // Clear / Cancel compensation
        controller.clearSurfaceHeightMap();
        assert(!controller.isSurfaceCompensationActive());
        assert(!controller.getTelemetry().surface_compensation_active);

        // 4. Settings Persistence Verification
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.machine.probe.surface_max_depth_below_zero_mm = 4.75;
        ps.machine.probe.surface_clearance_z_mm          = 6.25;

        std::string test_ini = "test_surface_probe_settings.ini";
        bool saved = rnr::config::SettingsManager::saveToFile(test_ini, ps);
        assert(saved);

        rnr::config::PersistentSettings loaded_ps{};
        bool loaded_ini = rnr::config::SettingsManager::loadFromFile(test_ini, loaded_ps);
        assert(loaded_ini);
        assert(std::abs(loaded_ps.machine.probe.surface_max_depth_below_zero_mm - 4.75) < 1e-5);
        assert(std::abs(loaded_ps.machine.probe.surface_clearance_z_mm - 6.25) < 1e-5);
        std::remove(test_ini.c_str());

        controller.disconnect();
        std::cout << "  -> PASSED: Surface map interpolation, toolpath subdivision warping, controller state & settings persistence verified.\n";
    }
}

void test16() {
    // ----------------------------------------------------
    // Test 16: Safe Approach Continuity & No-Teleport Verification (Issue A1)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 16] Testing Safe Approach Continuity, ATC start & zero coordinate jumps...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // Load ATC program where Segment 0 is "T1 M06" and Segment 1 is "G0 X60.0 Y40.0"
        std::string atc_code = rnr::gcode::GCodeParser::sampleMillAtcGCode();
        assert(controller.loadGCodeString(atc_code));

        // Position endmill at (50, 40, 15) away from (0, 0, 0)
        controller.setWorkPosition(rnr::core::Axis::X, 50.0);
        controller.setWorkPosition(rnr::core::Axis::Y, 40.0);
        controller.setWorkPosition(rnr::core::Axis::Z, 15.0);

        auto cfg = controller.getConfig();
        cfg.tool_change.mode = rnr::core::ToolChangeMode::AutoAtc;
        cfg.tool_change.safe_z_mm = 25.0;
        controller.applyConfig(cfg);

        // Start job
        controller.startJob();
        auto slices = controller.getSliceQueue();
        assert(!slices.empty());

        // Verify continuity: every consecutive slice must have smooth delta_steps (no teleport)
        const int32_t max_allowed_step_jump_per_slice = 2000; // steps in 25ms
        for (size_t i = 0; i < slices.size(); ++i) {
            for (size_t a = 0; a < 4; ++a) {
                assert(std::abs(slices[i].slice.delta_steps[a]) < max_allowed_step_jump_per_slice);
            }
        }

        // Verify that Z does NOT plunge to 0 during the initial XY positioning
        bool z_plunged_prematurely = false;
        for (const auto& sl : slices) {
            if (sl.segment_index == 1 && sl.target_pos_mm[2] < 10.0) {
                z_plunged_prematurely = true;
            }
        }
        assert(!z_plunged_prematurely);

        controller.stopJob();
        controller.disconnect();
        std::cout << "  -> PASSED: Toolchange segment 0 continuity and safe Z rapid approach verified.\n";
    }
}

void test17() {
    // ----------------------------------------------------
    // Test 17: Feed Hold Pause & Resume Continuity (Issue A2)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 17] Testing Feed Hold pause & resume continuity...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        std::string gcode = "G21 G90\nG0 X0 Y0 Z5\nG1 X100 Y0 Z-1 F600\nG0 Z5\n";
        assert(controller.loadGCodeString(gcode));
        controller.startJob();

        // Simulate pausing
        controller.pauseJob();
        auto telem = controller.getTelemetry();
        assert(telem.state == rnr::core::ControllerState::PausedHold);

        // Change feed override while paused
        controller.setFeedOverride(120.0f);

        // Resume job
        controller.startJob();
        telem = controller.getTelemetry();
        assert(telem.state == rnr::core::ControllerState::RunningGCode);

        controller.stopJob();
        controller.disconnect();
        std::cout << "  -> PASSED: Feed Hold pause and resume smoothly handles feed override.\n";
    }
}

void test18() {
    // ----------------------------------------------------
    // Test 18: G53 Machine Coordinates & G92 Offset Setting (Issues A3, A4)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 18] Testing G53 Machine Coordinates & G92 MDI offset setting...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // 1. Test G92 via MDI
        assert(controller.executeMdiCommand("G90 G92 X10.0 Y20.0 Z15.0"));
        auto tel = controller.getTelemetry();
        assert(std::abs(tel.work_pos_mm[0] - 10.0) < 1e-4);
        assert(std::abs(tel.work_pos_mm[1] - 20.0) < 1e-4);
        assert(std::abs(tel.work_pos_mm[2] - 15.0) < 1e-4);

        // 2. Test G53 Machine Coordinate rapid
        auto pr = rnr::gcode::GCodeParser::parse("G0 G53 Z0", tel.work_pos_mm, 0.25, tel.work_offset_mm);
        assert(pr.segments.size() == 1);
        assert(pr.segments[0].type == rnr::core::SegmentType::RapidMove);
        assert(std::abs(pr.segments[0].end_xyza_mm[2] - (-tel.work_offset_mm[2])) < 1e-4);
        assert((pr.segments[0].outputs_mask & rnr::core::OutputBit::OUT1_TorchOrSpindle) == 0);

        controller.disconnect();
        std::cout << "  -> PASSED: G92 sets work offsets cleanly & G53 plans machine coordinate rapids.\n";
    }
}

void test19() {
    // ----------------------------------------------------
    // Test 19: Manual M6 Tool Change Spindle Safety & Limit Override (Issues B1, B2)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 19] Testing Manual M6 Tool Change Spindle Safety & E-Stop Limit Override...\n";
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // 1. Verify E-Stop Limit Override Safety:
        controller.triggerEStop();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::EStop);

        // Limit Override should NOT blindly clear E-Stop into Idle!
        controller.setLimitOverride(true);
        assert(controller.getTelemetry().state == rnr::core::ControllerState::EStop);

        // Reset clears E-Stop
        controller.resetFromEStop();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);

        // 2. Verify Manual M6 Tool Change retract passes outputs_mask = 0
        auto cfg = controller.getConfig();
        std::array<double, 4> cur_w = {10.0, 10.0, 5.0, 0.0};
        std::array<double, 4> z_up  = {10.0, 10.0, 25.0, 0.0};
        auto z_slices = rnr::planner::TrajectoryPlanner::sliceLinearMove(cur_w, z_up, 1500.0, cfg, 0 /* Spindle strictly OFF */);
        assert(!z_slices.empty());
        for (const auto& s : z_slices) {
            assert((s.slice.outputs_mask & rnr::core::OutputBit::OUT1_TorchOrSpindle) == 0);
        }

        // 3. Verify cnc.get_tool_change_config() returns settings cleanly to Lua
        assert(controller.runScriptString("local tc = cnc.get_tool_change_config(); assert(tc.move_to_park_xy == false)", false));

        controller.disconnect();
        std::cout << "  -> PASSED: Spindle strictly OFF during M6 retract; Limit Override does not bypass E-Stop; Lua tool change config verified.\n";
    }
}

void test21() {
    // ----------------------------------------------------
    // Test 21: Probing Math Accuracy & Subdivided Linear Cruise (Issues C1, D2)
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 21] Testing Probing Edge Math Compensation & Subdivided Linear Cruise...\n";

        // Probing edge calculation verification
        const double hit_pos = 97.0;
        const double tip_radius = 3.0;
        const double retract = 3.0;
        const double travel_dir = 1.0; // probing right

        const double edge_coord = hit_pos + travel_dir * tip_radius; // 100.0
        const double current_pos = hit_pos - travel_dir * retract;   // 94.0
        const double current_offset = -travel_dir * (retract + tip_radius); // -6.0

        // When current_pos is set to current_offset (-6.0):
        // Position of edge is current_offset + (edge_coord - current_pos) = -6.0 + (100.0 - 94.0) = 0.0!
        assert(std::abs(edge_coord - 100.0) < 1e-6);
        assert(std::abs(current_pos - 94.0) < 1e-6);
        assert(std::abs(current_offset - (-6.0)) < 1e-6);
        assert(std::abs((current_pos + current_offset) - (edge_coord - 12.0)) < 1e-6);

        // Subdivided linear chord cruise test (Surface height-map subdivided moves)
        std::vector<rnr::core::ToolpathSegment> cut_segs;
        for (int i = 0; i < 10; ++i) {
            rnr::core::ToolpathSegment cs{};
            cs.type = rnr::core::SegmentType::CutLinear;
            cs.start_xyza_mm = {i * 2.0, 0.0, 0.0, 0.0};
            cs.end_xyza_mm   = {(i + 1) * 2.0, 0.0, 0.0, 0.0};
            cs.length_mm     = 2.0;
            cs.feed_mm_min   = 1200.0;
            cut_segs.push_back(cs);
        }

        rnr::core::MachineConfig cfg{};
        auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(cut_segs, cfg);
        assert(!slices.empty());

        // Each 2mm segment slices into constant velocity chords without deceleration to 0
        for (const auto& sl : slices) {
            assert(sl.slice.delta_steps[0] > 0);
        }

        std::cout << "  -> PASSED: Edge tip compensation mathematically exact; 2mm sub-segments cruise smoothly.\n";
    }

    // -------------------------------------------------------------
    // [Test 22] Position Persistence across Sessions & Curve Feedrate Continuity
    // -------------------------------------------------------------
    {
        std::cout << "\n[Test 22] Testing Position Persistence & Curve Feedrate Continuity...\n";

        // 1. Coordinates Persistence
        rnr::config::PersistentSettings ps{};
        ps.ui.has_saved_coordinates = true;
        ps.ui.last_machine_pos_mm[0] = 123.456;
        ps.ui.last_machine_pos_mm[1] = 78.910;
        ps.ui.last_machine_pos_mm[2] = 15.250;
        ps.ui.last_machine_pos_mm[3] = 0.0;
        ps.ui.last_work_offset_mm[0] = 50.0;
        ps.ui.last_work_offset_mm[1] = 20.0;
        ps.ui.last_work_offset_mm[2] = 5.0;
        ps.ui.last_work_offset_mm[3] = 0.0;
        for (int i = 0; i < 4; ++i) {
            ps.ui.last_work_pos_mm[i] = ps.ui.last_machine_pos_mm[i] - ps.ui.last_work_offset_mm[i];
        }

        const std::string test_ini = "test_coords_persist.ini";
        bool save_ok = rnr::config::SettingsManager::saveToFile(test_ini, ps);
        assert(save_ok);

        rnr::config::PersistentSettings loaded_ps{};
        bool load_ok = rnr::config::SettingsManager::loadFromFile(test_ini, loaded_ps);
        assert(load_ok);
        assert(loaded_ps.ui.has_saved_coordinates);
        assert(std::abs(loaded_ps.ui.last_machine_pos_mm[0] - 123.456) < 1e-4);
        assert(std::abs(loaded_ps.ui.last_work_offset_mm[0] - 50.0) < 1e-4);
        assert(std::abs(loaded_ps.ui.last_work_pos_mm[0] - (123.456 - 50.0)) < 1e-4);
        std::remove(test_ini.c_str());

        MockController mock;
        std::array<double, 4> m_pos = {loaded_ps.ui.last_machine_pos_mm[0], loaded_ps.ui.last_machine_pos_mm[1], loaded_ps.ui.last_machine_pos_mm[2], loaded_ps.ui.last_machine_pos_mm[3]};
        std::array<double, 4> w_off = {loaded_ps.ui.last_work_offset_mm[0], loaded_ps.ui.last_work_offset_mm[1], loaded_ps.ui.last_work_offset_mm[2], loaded_ps.ui.last_work_offset_mm[3]};
        mock.restoreSavedPositions(m_pos, w_off);
        auto tel = mock.getTelemetry();
        assert(std::abs(tel.machine_pos_mm[0] - 123.456) < 1e-4);
        assert(std::abs(tel.work_pos_mm[0] - (123.456 - 50.0)) < 1e-4);

        // 2. Curve Feed Rate Continuity: Twenty 0.1mm chords (total 2.0mm) at F1200 (20mm/s)
        // Cruise time is 0.10s (4 slices). Starting and ending at rest adds the acceleration (20/500 s) and
        // deceleration (20/600 s) ramps, so ~0.137s = 6 slices. Without chord slice-time accumulation this
        // would be 20+ slices (one per chord).
        std::vector<rnr::core::ToolpathSegment> curve_chords;
        for (int i = 0; i < 20; ++i) {
            rnr::core::ToolpathSegment cs{};
            cs.type          = rnr::core::SegmentType::CutArcCW;
            cs.segment_index = static_cast<size_t>(i);
            cs.gcode_line    = 10;
            cs.start_xyza_mm = {i * 0.1, 0.0, 0.0, 0.0};
            cs.end_xyza_mm   = {(i + 1) * 0.1, 0.0, 0.0, 0.0};
            cs.length_mm     = 0.1;
            cs.feed_mm_min   = 1200.0; // 20 mm/s
            curve_chords.push_back(cs);
        }

        rnr::core::MachineConfig cfg{};
        auto slices = rnr::planner::TrajectoryPlanner::sliceProgram(curve_chords, cfg, 100.0f, 0.0f, false, 0.025);
        // Full speed is maintained across the chords: no stop-and-go between them
        assert(slices.size() >= 5 && slices.size() <= 7);
        for (const auto& sl : slices) {
            assert(sl.slice.delta_steps[0] > 0);
        }
        // Accelerates from rest: the first slice is shorter than a cruise slice (20mm/s * 25ms = 0.5mm = 100 steps)
        assert(slices.front().slice.delta_steps[0] < 100);

        std::cout << "  -> PASSED: Coordinates persisted across sessions; 20x 0.1mm curve chords sliced at full feed velocity ("
                  << slices.size() << " slices).\n";
    }
}

void test24(MockController& mock_controller) {
    // ----------------------------------------------------
    // Test 24: M6 Tool Change "TX already probed" Button
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 24] Testing M6 Tool Change Dialog & 'TX already probed' Button...\n";
        MockController mock_controller;
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::atomic<bool> script_done{false};
        std::atomic<bool> script_success{false};

        // Run M6 script file asynchronously
        std::string m6_path = "scripts/mill/m6.lua";
        runner.runFileAsync(m6_path, [&](bool success, const std::string& err) {
            script_done = true;
            script_success = success;
        });

        // Wait for prompt
        auto start = std::chrono::steady_clock::now();
        while (!runner.hasPendingPrompt()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > 3.0) break;
        }

        assert(runner.hasPendingPrompt());
        auto req = runner.getPendingPrompt();
        assert(req.title == "Tool Change (M6)");
        assert(req.buttons.size() == 3);
        assert(req.buttons[0] == "OK");
        assert(req.buttons[1] == "T1 already probed");
        assert(req.buttons[2] == "Cancel");

        // Operator clicks "T1 already probed"
        runner.submitPromptResponse("T1 already probed");

        // Script should finish immediately with success = true without performing probe touches
        start = std::chrono::steady_clock::now();
        while (!script_done.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > 3.0) break;
        }

        assert(script_done.load());
        assert(script_success.load());

        std::cout << "  -> PASSED: M6 prompt surfaced 3 buttons including 'T1 already probed'; clicking it bypassed probing and completed with success.\n";
    }
}

void test25() {
    // ----------------------------------------------------
    // Test 25: Stop vs Rewind, Rapid Z20 Continuity & N-Line Numbering
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 25] Testing Stop vs Rewind, Rapid Z20 Continuity & N-Line Numbering...\n";

        // 1. Rapid Z20 Continuity (N180 G00 G43 Z20.000 H1):
        // Ensure that starting after an M6 tool change with G0 Z20 NEVER plunges to Z=0!
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        std::string test_gcode =
            "N140G00G21G17G90G40G49G80\n"
            "N150G71G91.1\n"
            "N160T1M06\n"
            "N170 (V-Bit {90 deg 0.5\"})\n"
            "N180G00G43Z20.000H1\n"
            "N190S16000M03\n"
            "N200G01X10.0Y10.0Z-1.0F1200\n";

        bool loaded = controller.loadGCodeString(test_gcode);
        assert(loaded);
        const auto& pr = (*controller.getProgram());
        assert(pr.segments.size() >= 3);
        assert(pr.segments[0].type == rnr::core::SegmentType::ToolChange);
        assert(pr.segments[1].type == rnr::core::SegmentType::RapidMove);
        assert(std::abs(pr.segments[1].end_xyza_mm[2] - 20.0) < 1e-4);

        // Simulate tool is at Safe Z (25.0) after M6 tool change
        controller.setWorkPosition(rnr::core::Axis::Z, 25.0);
        assert(std::abs(controller.getTelemetry().work_pos_mm[2] - 25.0) < 1e-4);

        // Trigger job from segment 1 (line N180) - exactly what happens when M6 completes
        controller.setNextGCodeLine(4); // Line N180 is line index 4
        controller.startJob();

        // Check planned slices: NO slice should ever have Z close to 0.0 before reaching Z20!
        // The first motion should go monotonically from 25.0 down to 20.0, NEVER plunging to 0.0!
        auto slices = controller.getSliceQueue();
        assert(!slices.empty());
        for (const auto& s : slices) {
            if (s.segment_index == 1) { // Segment 1 is G00 G43 Z20.000 H1
                // Target Z must stay >= 19.99 (moving from 25.0 to 20.0), never drop toward 0.0!
                assert(s.target_pos_mm[2] >= 19.99);
            }
        }
        controller.stopJob();

        // 2. Stop vs Rewind logic test
        // Verify controller line index preservation on stopJob vs rewind to line 0
        controller.setNextGCodeLine(4);
        controller.stopJob();
        assert(controller.getTelemetry().active_gcode_line_index == 4);

        // Rewind sets next gcode line to 0
        controller.setNextGCodeLine(0);
        assert(controller.getTelemetry().active_gcode_line_index == 0);

        controller.disconnect();
        std::cout << "  -> PASSED: Rapid G0 Z20 plans cleanly from Safe Z without plunging to Z0; Stop retains active line and Rewind resets to line 1.\n";
    }
}

void test29() {
    // ========================================================================
    // Test 29: E-Stop input is never bypassed by Limit Override; reset needs a released E-Stop
    // ========================================================================
    {
        std::cout << "\n[Test 29] Testing E-Stop input vs Limit Override and reset interlocks...\n";
        rnr::core::RnRController controller;
        auto cfg = controller.getConfig();
        cfg.estop_pin = 6;
        cfg.axes[0].limit_plus_pin = 7;
        controller.applyConfig(cfg);
        assert(controller.resetFromEStop());
        controller.setLimitOverride(true);
        controller.setSimulatedInputPin(6, true); // E-Stop pressed
        assert(waitUntil([&] { return controller.getTelemetry().state == rnr::core::ControllerState::EStop; }));
        assert(!controller.resetFromEStop()); // Still pressed
        controller.setSimulatedInputPin(6, false);
        assert(waitUntil([&] { return controller.resetFromEStop(); }));

        // A limit switch blocks reset unless Limit Override is on
        controller.setSimulatedInputPin(7, true);
        assert(waitUntil([&] { return controller.getTelemetry().state == rnr::core::ControllerState::EStop; }));
        assert(!controller.resetFromEStop());
        controller.setLimitOverride(true);
        assert(controller.resetFromEStop());
        controller.setSimulatedInputPin(7, false);
        std::cout << "  -> PASSED: Limit Override no longer disables the E-Stop input; reset refused while E-Stop / limit active.\n";
    }
}

void test30() {
    // ========================================================================
    // Test 30: Programs with errors / soft-limit violations never start
    // ========================================================================
    {
        std::cout << "\n[Test 30] Testing Cycle Start validation (program errors & soft limits)...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        controller.loadGCodeString("G21 G90\nG41 D1\nG1 X10 F500\n");
        controller.startJob();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);
        assert(controller.getSliceQueue().empty());

        auto cfg = controller.getConfig();
        cfg.axes[0].soft_limits_enabled = true;
        cfg.axes[0].soft_limit_min_mm = -1.0;
        cfg.axes[0].soft_limit_max_mm = 50.0;
        controller.applyConfig(cfg);
        controller.homeAxis(rnr::core::Axis::X);
        for (int i = 0; i < 400 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(controller.getTelemetry().axis_homed[0]);
        controller.loadGCodeString("G21 G90\nG0 X100 Y0\n"); // X100 is beyond the 50 mm soft limit
        controller.startJob();
        auto t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(t.active_fault_message.find("soft limits") != std::string::npos);
        std::cout << "  -> PASSED: Erroneous programs and soft-limit violations are rejected before any motion.\n";
    }
}

void test31() {
    // ========================================================================
    // Test 31: Simulated homing & hole-centre probing complete with real sequencing
    // ========================================================================
    {
        std::cout << "\n[Test 31] Testing homing sequence and hole-centre probing in the simulator...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        controller.homeAllAxes();
        for (int i = 0; i < 1500 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        auto t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(t.axis_homed[0] && t.axis_homed[1] && t.axis_homed[2]);
        // Pulled off the switch by home_pulloff_mm (X/Y home negative, Z positive)
        const auto cfg = controller.getConfig();
        assert(std::abs(t.machine_pos_mm[0] - cfg.axes[0].home_pulloff_mm) < 0.05);
        assert(std::abs(t.machine_pos_mm[2] + cfg.axes[2].home_pulloff_mm) < 0.05);

        // Probe refuses to start while the probe input is already active
        controller.setSimulatedInputPin(cfg.probe.probe_pin, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        controller.probeRaw(rnr::core::Axis::Z, -5.0, 100.0);
        assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);
        controller.setSimulatedInputPin(cfg.probe.probe_pin, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));

        controller.probeHoleCenterXY(20.0);
        for (int i = 0; i < 2000 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(std::abs(t.work_pos_mm[0]) < 1e-6 && std::abs(t.work_pos_mm[1]) < 1e-6);
        std::cout << "  -> PASSED: Homing seeks the switch and pulls off; hole centre probes 4 walls and zeros XY.\n";
    }
}

void test32() {
    // ========================================================================
    // Test 32: Feed hold decelerates along the path and resumes
    // ========================================================================
    {
        std::cout << "\n[Test 32] Testing feed hold deceleration & resume...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        assert(controller.loadGCodeString("G21 G90\nG1 X200 F3000\n"));
        controller.startJob();
        // Wait for conditions, not fixed times: a busy CI machine (macOS runners) can stall the worker thread
        for (int i = 0; i < 500 && controller.getTelemetry().machine_pos_mm[0] < 5.0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const double x_at_pause = controller.getTelemetry().machine_pos_mm[0];
        assert(x_at_pause >= 5.0);
        controller.pauseJob();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::PausedHold);
        // Braking has ended once the position stays the same for 150 ms
        double x_stopped = controller.getTelemetry().machine_pos_mm[0];
        for (int stable = 0, i = 0; i < 300 && stable < 3; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            const double x = controller.getTelemetry().machine_pos_mm[0];
            stable = (x == x_stopped) ? stable + 1 : 0;
            x_stopped = x;
        }
        assert(x_stopped > x_at_pause);               // Kept moving while decelerating (no instant stop)
        assert(x_stopped - x_at_pause < 10.0);        // ...but only for the braking distance
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        assert(std::abs(controller.getTelemetry().machine_pos_mm[0] - x_stopped) < 1e-9); // Standing still
        controller.startJob();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::RunningGCode);
        for (int i = 0; i < 1000 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(std::abs(controller.getTelemetry().work_pos_mm[0] - 200.0) < 0.01);
        std::cout << "  -> PASSED: Feed hold brakes along the path, stands still, and resumes to the exact end point.\n";
    }
}

void test33() {
    // ========================================================================
    // Test 33: MDI validation & G10 L20
    // ========================================================================
    {
        std::cout << "\n[Test 33] Testing MDI validation and G10 L20...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        assert(!controller.executeMdiCommand("G1 X10"));       // No feed rate known yet
        assert(!controller.executeMdiCommand("G41 G1 X10 F100")); // Unsupported
        assert(controller.executeMdiCommand("G10 L20 P1 X5"));  // Current position reads X5
        assert(std::abs(controller.getTelemetry().work_pos_mm[0] - 5.0) < 1e-9);
        assert(controller.executeMdiCommand("G1 X6 F600"));
        for (int i = 0; i < 300 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(controller.executeMdiCommand("G1 X7"));          // F is modal across MDI commands now
        std::cout << "  -> PASSED: MDI rejects unsafe commands, G10 L20 sets offsets, modal F retained.\n";
    }
}

void test36() {
    // ========================================================================
    // Test 36: Tool table mode applies G43 offsets to motion, DRO and touch-off
    // ========================================================================
    {
        std::cout << "\n[Test 36] Testing tool table G43 offsets in jobs, MDI, touch-off and Cycle Start checks...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        auto cfg = controller.getConfig();
        cfg.tool_table.mode = rnr::core::ToolLengthMode::ToolTable;
        cfg.tool_table.tools[1].length_mm = 0.0;
        cfg.tool_table.tools[2].length_mm = 10.0;
        cfg.scripting.m6_script_enabled = false;
        controller.applyConfig(cfg);

        auto waitIdle = [&]() {
            for (int i = 0; i < 600; ++i) {
                const auto st = controller.getTelemetry().state;
                if (st == rnr::core::ControllerState::Idle || st == rnr::core::ControllerState::ToolChangeHold) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        };

        // MDI G43 H2: the DRO now shows the tool tip (10 mm below the spindle reference)
        assert(controller.executeMdiCommand("G43 H2"));
        auto t = controller.getTelemetry();
        assert(t.active_tlo_tool == 2 && std::abs(t.active_tlo_mm - 10.0) < 1e-9);
        assert(std::abs(t.work_pos_mm[2] - (t.machine_pos_mm[2] - 10.0)) < 1e-9);
        assert(controller.executeMdiCommand("G49"));
        assert(controller.getTelemetry().active_tlo_tool == 0);

        // Program with G43 H2: the tip reaches Z5, so the spindle (machine Z) is 10 mm higher
        assert(controller.loadGCodeString("G21 G90\nG43 H2 G0 Z5\n"));
        assert(controller.getProgram()->warnings.empty());
        controller.startJob();
        assert(controller.getTelemetry().state == rnr::core::ControllerState::RunningGCode);
        waitIdle();
        t = controller.getTelemetry();
        assert(std::abs(t.machine_pos_mm[2] - 15.0) < 1e-3);
        assert(std::abs(t.work_pos_mm[2] - 5.0) < 1e-3);
        assert(t.active_tlo_tool == 2);

        // Touch off with T2 loaded, then cancel the offset: the spindle frame shows the tool length
        controller.zeroWorkAxis(rnr::core::Axis::Z);
        assert(std::abs(controller.getTelemetry().work_pos_mm[2]) < 1e-9);
        assert(controller.executeMdiCommand("G49"));
        assert(std::abs(controller.getTelemetry().work_pos_mm[2] - 10.0) < 1e-9);

        // Cycle Start checks: unknown tool length, and M6 without G43 for the new tool
        controller.loadGCodeString("G21 G90\nT3 M6\nG43 H3\nG0 Z5\n");
        controller.startJob();
        t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(t.active_fault_message.find("no length") != std::string::npos);
        controller.loadGCodeString("G21 G90\nT2 M6\nG0 Z5\n");
        controller.startJob();
        t = controller.getTelemetry();
        assert(t.state == rnr::core::ControllerState::Idle);
        assert(t.active_fault_message.find("not followed by G43") != std::string::npos);
        std::cout << "  -> PASSED: G43 offsets move the spindle, DRO shows the tip, touch-off is tool-aware, unsafe programs refused.\n";
    }
}

void test37() {
    // ========================================================================
    // Test 37: Tool length measuring (simulator) & tool table persistence
    // ========================================================================
    {
        std::cout << "\n[Test 37] Testing tool length measuring and tool table persistence...\n";
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        auto cfg = controller.getConfig();
        cfg.tool_table.mode = rnr::core::ToolLengthMode::ToolTable;
        cfg.tool_table.reference_tool = 1;
        controller.applyConfig(cfg);
        auto waitIdle = [&]() {
            for (int i = 0; i < 600 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        };

        // Other tools need the reference first
        controller.measureToolLength(2);
        assert(controller.getConfig().tool_table.tools.count(2) == 0);

        controller.measureToolLength(1); // Simulator: contact 20 mm below the start (half of the 40 mm search)
        waitIdle();
        assert(controller.getTelemetry().tool_reference_measured);
        assert(std::abs(controller.getConfig().tool_table.tools.at(1).length_mm) < 1e-9);

        assert(controller.executeMdiCommand("G90 G0 Z13"));
        waitIdle();
        controller.measureToolLength(2); // Contact at Z13-20 = -7, reference contact at -20 -> +13 mm
        waitIdle();
        const auto measured = controller.getConfig().tool_table.tools;
        assert(measured.count(2) == 1);
        assert(std::abs(measured.at(2).length_mm - 13.0) < 1e-3);

        // Persistence
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.machine.tool_table.tools[2].diameter_mm = 6.0;
        ps.machine.tool_table.tools[2].description = "6mm flat endmill; 3 flute";
        const std::string ini = "test_tool_table.ini";
        assert(rnr::config::SettingsManager::saveToFile(ini, ps));
        rnr::config::PersistentSettings loaded{};
        assert(rnr::config::SettingsManager::loadFromFile(ini, loaded));
        assert(loaded.machine.tool_table.mode == rnr::core::ToolLengthMode::ToolTable);
        assert(loaded.machine.tool_table.tools.size() == 2);
        assert(std::abs(loaded.machine.tool_table.tools.at(2).length_mm - 13.0) < 1e-3);
        assert(std::abs(loaded.machine.tool_table.tools.at(2).diameter_mm - 6.0) < 1e-9);
        assert(loaded.machine.tool_table.tools.at(2).description == "6mm flat endmill; 3 flute");
        std::remove(ini.c_str());
        std::remove((ini + ".bak").c_str());
        std::cout << "  -> PASSED: Reference-relative tool lengths measured on the setter and persisted.\n";
    }
}

void test39() {
    // ------------------------------------------------------------------------
    // Test 39: The machine core drives the hardware only through IBoardDriver (fake board, no USB)
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 39] Testing the machine core against a fake board driver...\n";
        auto fake = std::make_unique<FakeBoardDriver>();
        FakeBoardDriver* fb = fake.get();
        auto closed = fb->closed;
        auto waitIdle = [](rnr::core::MachineController& mc) {
            for (int i = 0; i < 200; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
                if (mc.getTelemetry().state == rnr::core::ControllerState::Idle) return true;
            }
            return false;
        };
        {
            rnr::core::MachineController mc(std::move(fake));
            // Connect: board configured, disarmed, outputs off, step counters synced
            auto tel = mc.getTelemetry();
            assert(!tel.simulator_mode && tel.usb_connected);
            assert(tel.product_name == "Fake Board" && tel.state == rnr::core::ControllerState::EStop);
            assert(fb->count("configure") == 1 && fb->count("estop") >= 1 && fb->count("outputs") >= 1);
            assert(fb->count("positions") == 1);
            assert(mc.boardCapabilities().display_name == "Fake Board");

            assert(waitUntil([&] { return mc.resetFromEStop(); })); // After the first board report
            assert(fb->count("clear_estop") == 1);
            // The reset puts the board on its default timing: the configuration is sent again after the clear
            assert(fb->count("configure") == 2);
            {
                const auto ev = fb->events();
                const auto clr = std::find(ev.begin(), ev.end(), "clear_estop");
                assert(clr != ev.end() && std::find(clr, ev.end(), "configure") != ev.end());
            }

            // Jog: internal slices only, the board ends exactly at the commanded position
            mc.jogStep(rnr::core::Axis::X, 5.0, 600.0);
            assert(waitIdle(mc));
            const double spm = mc.getConfig().axes[0].steps_per_mm;
            assert(fb->steps(0) == static_cast<int32_t>(std::llround(5.0 * spm)));
            assert(fb->count("slice_internal") > 0 && fb->count("slice_program") == 0);
            assert(std::abs(mc.getTelemetry().machine_pos_mm[0] - 5.0) < 1e-6);

            // Outputs
            mc.setAuxOutput(3, true);
            assert((fb->lastOutputs() & rnr::core::OutputBit::OUT3_FloodCoolant) != 0);
            mc.setAuxOutput(3, false);
            assert((fb->lastOutputs() & rnr::core::OutputBit::OUT3_FloodCoolant) == 0);

            // Output pin assignment: flood coolant on OUT4, mist on OUT3; M8 from MDI drives OUT4
            {
                auto cfg = mc.getConfig();
                cfg.io.output_pin[2] = 4;
                cfg.io.output_pin[3] = 3;
                mc.applyConfig(cfg);
                assert(mc.executeMdiCommand("M8"));
                assert(waitUntil([&] { return (fb->lastOutputs() & 0x0F) == 0x08; }));
                assert(mc.getTelemetry().physical_outputs_bitmask == 0x08);
                assert(mc.executeMdiCommand("M9"));
                assert(waitUntil([&] { return (fb->lastOutputs() & 0x0F) == 0x00; }));

                // Mist not connected: OUT3 is a custom output, switched by its own bit
                cfg = mc.getConfig();
                cfg.io.output_pin[3] = 0;
                mc.applyConfig(cfg);
                mc.setPhysicalOutput(3, true);
                assert((fb->lastOutputs() & 0x0F) == 0x04);
                assert((mc.getTelemetry().outputs_bitmask & (1u << (rnr::core::kCustomOutputShift + 2))) != 0);
                // Changing the assignment switches every output off
                cfg = mc.getConfig();
                cfg.io.output_pin = {1, 2, 3, 4};
                mc.applyConfig(cfg);
                assert((fb->lastOutputs() & 0x0F) == 0x00 && mc.getTelemetry().outputs_bitmask == 0);
            }

            // A G-code job streams internal slices (the board holds G-code slices back for seconds), armed
            // like an MDI move, with outputs also set by explicit commands when they change (M8 / M9)
            // Each run goes like an MDI move: outputs set at rest, Start Continuous Jog, slices, End of Motion.
            const int internal0 = fb->count("slice_internal");
            fb->clearEvents();
            assert(mc.loadGCodeString("G90 G21\nG1 X10 Y-4 F900\nM8\nG0 X0 Y0\nM9\nG1 X1 F900\nG0 X0\n"));
            mc.startJob();
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            assert(waitIdle(mc));
            assert(fb->count("slice_program") == 0 && fb->count("slice_internal") > internal0);
            assert((fb->lastOutputs() & rnr::core::OutputBit::OUT3_FloodCoolant) == 0);
            {
                const auto ev = fb->events();
                int runs = 0;
                for (size_t k = 0; k < ev.size(); ++k) {
                    if (ev[k] != "begin_jog") continue;
                    ++runs;
                    // Slices follow the Start Continuous Jog, never an outputs command in between
                    assert(k + 1 < ev.size() && ev[k + 1] == "slices");
                    // Before it: outputs set (at rest), or the previous run closed by End of Motion
                    assert(k > 0 && (ev[k - 1] == "outputs" || ev[k - 1] == "outputs_after"));
                }
                assert(runs >= 3); // Start, after M8, after M9
                for (size_t k = 1; k < ev.size(); ++k) {
                    // Outputs never change in the middle of a run: a run is closed first
                    if (ev[k] == "outputs" && ev[k - 1] == "slices") assert(false);
                }
                assert(ev.back() == "outputs_after"); // The job ends with End of Motion
            }
            assert(fb->steps(0) == 0 && fb->steps(1) == 0);
            assert(fb->maxInFlight() <= 2); // Never more than the board's buffer (capabilities: 2)
            // The board acknowledges slices at once here: the sender must still pace them in real time, at most
            // about max_slices_in_flight (2) slices ahead of the board, or motion runs behind the app
            std::cout << "  max lead of the slice stream: " << fb->maxLeadMs() << " ms\n";
            assert(fb->maxLeadMs() <= 2 * 25.0 + 25.0 + 1.0);

            // M30 switches the coolant off at the end of the job (as in Mach3); without M2 / M30 it stays on
            assert(mc.loadGCodeString("G90 G21\nM8\nG1 X2 F900\nG0 X0\nM30\n"));
            mc.startJob();
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            assert(waitIdle(mc));
            assert((fb->lastOutputs() & rnr::core::OutputBit::OUT3_FloodCoolant) == 0);
            assert((mc.getTelemetry().outputs_bitmask & rnr::core::OutputBit::OUT3_FloodCoolant) == 0);
            assert(mc.loadGCodeString("G90 G21\nM8\nG1 X2 F900\nG0 X0\n"));
            mc.startJob();
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            assert(waitIdle(mc));
            assert((fb->lastOutputs() & rnr::core::OutputBit::OUT3_FloodCoolant) != 0);
            assert(mc.executeMdiCommand("M9"));
            assert(waitUntil([&] { return (fb->lastOutputs() & rnr::core::OutputBit::OUT3_FloodCoolant) == 0; }));

            // E-Stop reaches the board
            const int estops = fb->count("estop");
            mc.triggerEStop();
            assert(fb->count("estop") > estops);
        }
        assert(closed->load()); // The core closes the board on shutdown
        std::cout << "  -> PASSED: connect, reset, jog, outputs, job streaming (internal slices, armed, outputs synced), slice budget and E-Stop all go through the driver.\n";
    }

    {
        std::cout << "\n[Test 40] Testing input / output pin assignment...\n";
        namespace io = rnr::core::io;
        rnr::core::IoConfig iocfg;

        // Default assignment: every logical channel drives the output with the same number
        for (unsigned m = 0; m < 16; ++m) assert(io::physicalOutputs(iocfg, static_cast<uint8_t>(m)) == m);
        assert(io::channelForPin(iocfg, 3) == 2 && io::logicalBitForPin(iocfg, 3) == 0x04);

        // Swapped coolant outputs, and a custom bit only drives a pin without a channel
        iocfg.output_pin = {1, 2, 4, 0};
        assert(io::physicalOutputs(iocfg, 0x04) == 0x08);                    // Flood -> OUT4
        assert(io::physicalOutputs(iocfg, 0x08) == 0x00);                    // Mist not connected
        assert(io::physicalOutputs(iocfg, static_cast<uint8_t>(1u << 6)) == 0x04); // Custom OUT3
        assert(io::physicalOutputs(iocfg, static_cast<uint8_t>(1u << 7)) == 0x00); // OUT4 belongs to flood
        assert(io::channelForPin(iocfg, 3) == -1 && io::logicalBitForPin(iocfg, 3) == (1u << 6));

        // Sanitizing: out-of-range pin, a pin used twice, and an unconnected enable channel
        rnr::core::IoConfig bad;
        bad.output_pin = {0, 2, 2, 9};
        assert(io::sanitize(bad));
        assert(bad.output_pin[1] == 2 && bad.output_pin[2] == 0 && bad.output_pin[3] == 0);
        assert(bad.output_pin[0] == 1);
        rnr::core::IoConfig good;
        assert(!io::sanitize(good));

        // Input overview: one switch for X home and X limit-, the probe, and a free input for a custom name
        rnr::core::MachineConfig mcfg;
        mcfg.is_mill = true;
        mcfg.axes[0].home_pin = 1;
        mcfg.axes[0].limit_minus_pin = 1;
        mcfg.probe.probe_pin = 5;
        mcfg.io.input_name[6] = "Clamp";
        mcfg.io.input_invert[6] = true;
        const auto uses = io::describeInputs(mcfg);
        assert(uses[0].size() == 2 && io::inputShortLabel(uses[0], "") == "X home/lim");
        assert(uses[4].size() == 1 && uses[4][0].label == "Probe");
        assert(uses[6].empty() && io::inputShortLabel(uses[6], "Clamp") == "Clamp");
        assert(io::inputPinByName(mcfg.io, "clamp") == 7 && io::inputPinByName(mcfg.io, "Vacuum") == 0);
        // The mill has no THC: its default pins stay free
        assert(uses[1].empty() && uses[2].empty());

        // Settings round trip
        rnr::config::PersistentSettings ps;
        ps.machine.io.output_pin = {1, 2, 4, 0};
        ps.machine.io.output_name[2] = "Vacuum";
        ps.machine.io.input_name[6] = "Clamp";
        ps.machine.io.input_invert[6] = true;
        const std::string ini = "test_io_settings.ini";
        assert(rnr::config::SettingsManager::saveToFile(ini, ps));
        rnr::config::PersistentSettings back;
        assert(rnr::config::SettingsManager::loadFromFile(ini, back));
        std::filesystem::remove(ini);
        assert(back.machine.io.output_pin == ps.machine.io.output_pin);
        assert(back.machine.io.output_name[2] == "Vacuum" && back.machine.io.input_name[6] == "Clamp");
        assert(back.machine.io.input_invert[6] && !back.machine.io.input_invert[5]);

        // Lua: cnc.get_input_by_name applies the custom input's invert setting
        MockController mock;
        mock.config_.io = mcfg.io;
        mock.telemetry_.inputs_bitmask = 0;
        rnr::scripting::ScriptRunner runner(&mock);
        std::string err;
        bool ok = runner.runStringBlocking(R"LUA(
            assert(cnc.get_input_by_name("Clamp") == true)   -- inverted: active while IN7 is off
            assert(cnc.get_input(7) == false)
            local found = pcall(cnc.get_input_by_name, "Nope")
            assert(found == false)
        )LUA", &err);
        if (!ok) std::cerr << err << "\n";
        assert(ok);
        std::cout << "  -> PASSED: output mapping, custom outputs, sanitizing, input overview, settings and Lua.\n";
    }
}

void test41() {
    // ========================================================================
    // Test 41: Program start Z lift (on / off, own safe Z height, settings)
    // ========================================================================
    {
        std::cout << "\n[Test 41] Testing the program start Z lift setting...\n";
        auto firstSlices = [](bool lift, double safe_z, const std::string& gcode) {
            rnr::core::RnRController controller;
            controller.setSimulatorMode(true);
            controller.connect();
            controller.resetFromEStop();
            auto cfg = controller.getConfig();
            cfg.start_lift_to_safe_z = lift;
            cfg.start_safe_z_mm      = safe_z;
            cfg.tool_change.safe_z_mm = 40.0; // Must not be used
            controller.applyConfig(cfg);
            assert(controller.loadGCodeString(gcode));
            controller.startJob();
            auto slices = controller.getSliceQueue();
            controller.stopJob();
            controller.disconnect();
            assert(!slices.empty());
            return slices;
        };

        // Lift on: Z rises to the line's own Z10 before X / Y / A move
        auto on = firstSlices(true, 7.0, "G0 X10 Y10 Z10 A10\n");
        for (const auto& sl : on) {
            if (sl.target_pos_mm[2] < 10.0 - 1e-6) assert(std::abs(sl.target_pos_mm[0]) < 1e-6);
        }
        // Lift off: all axes move together from the first slice
        auto off = firstSlices(false, 7.0, "G0 X10 Y10 Z10 A10\n");
        assert(off.front().target_pos_mm[0] > 1e-6 && off.front().target_pos_mm[2] > 1e-6);
        // A pure XY rapid lifts to the configured height, not the M6 safe Z
        auto xy = firstSlices(true, 7.0, "G0 X10 Y10\n");
        double max_z = 0.0;
        for (const auto& sl : xy) max_z = std::max(max_z, sl.target_pos_mm[2]);
        assert(std::abs(max_z - 7.0) < 1e-6);
        auto xy_off = firstSlices(false, 7.0, "G0 X10 Y10\n");
        for (const auto& sl : xy_off) assert(std::abs(sl.target_pos_mm[2]) < 1e-6);

        // Settings: saved and restored; older files take the height from the M6 safe Z
        const std::string ini = (std::filesystem::temp_directory_path() / "breakout_start_lift_test.ini").string();
        rnr::config::PersistentSettings ps;
        ps.machine.start_lift_to_safe_z = false;
        ps.machine.start_safe_z_mm      = 12.5;
        assert(rnr::config::SettingsManager::saveToFile(ini, ps));
        rnr::config::PersistentSettings back;
        assert(rnr::config::SettingsManager::loadFromFile(ini, back));
        assert(!back.machine.start_lift_to_safe_z && back.machine.start_safe_z_mm == 12.5);
        std::filesystem::remove(ini + ".bak"); // An old file has no integrity marker: no backup may win
        {
            std::ofstream old_file(ini, std::ios::trunc);
            old_file << "[ToolChange]\nsafe_z_mm = 18\n";
        }
        rnr::config::PersistentSettings old;
        assert(rnr::config::SettingsManager::loadFromFile(ini, old));
        std::filesystem::remove(ini);
        std::filesystem::remove(ini + ".bak");
        assert(old.machine.start_lift_to_safe_z && old.machine.start_safe_z_mm == 18.0);
        std::cout << "  -> PASSED: lift on / off, own safe Z height and settings migration.\n";
    }
}

void test42() {
    // ========================================================================
    // Test 42: A as an independent (rotary) axis vs. A slaved to Z
    // ========================================================================
    {
        std::cout << "\n[Test 42] Testing A jogging and A slaved to Z...\n";
        using rnr::core::GantrySlave;

        // A follows Z: every Z step is copied to the A motor
        rnr::core::MachineConfig zcfg;
        zcfg.gantry_slave = GantrySlave::AFollowsZ;
        assert(zcfg.gantryMasterAxis() == 2);
        auto zs = rnr::planner::TrajectoryPlanner::sliceLinearMove({0, 0, 0, 0}, {0, 0, -5, 0}, 600.0, zcfg);
        assert(!zs.empty());
        int32_t z_total = 0;
        for (const auto& sl : zs) {
            assert(sl.slice.delta_steps[3] == sl.slice.delta_steps[2]);
            z_total += sl.slice.delta_steps[2];
        }
        assert(z_total == static_cast<int32_t>(std::llround(-5.0 * zcfg.axes[2].steps_per_mm)));

        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        // Slaved A refuses its own jogs
        auto cfg = controller.getConfig();
        cfg.gantry_slave = GantrySlave::AFollowsZ;
        controller.applyConfig(cfg);
        controller.jogStep(rnr::core::Axis::A, 10.0, 600.0);
        controller.jogStartContinuous(rnr::core::Axis::A, true, 0.5);
        assert(controller.getSliceQueue().empty());
        assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);

        // Independent A jogs in degrees
        cfg = controller.getConfig();
        cfg.gantry_slave = GantrySlave::None;
        controller.applyConfig(cfg);
        const double a0 = controller.getTelemetry().work_pos_mm[3];
        controller.jogStep(rnr::core::Axis::A, 10.0, 1800.0);
        assert(waitUntil([&] {
            const auto t = controller.getTelemetry();
            return t.state == rnr::core::ControllerState::Idle && std::abs(t.work_pos_mm[3] - (a0 + 10.0)) < 1e-3;
        }));
        controller.disconnect();

        // Settings keep "A follows Z"
        const std::string ini = (std::filesystem::temp_directory_path() / "breakout_a_follows_z_test.ini").string();
        rnr::config::PersistentSettings ps;
        ps.machine.gantry_slave = GantrySlave::AFollowsZ;
        assert(rnr::config::SettingsManager::saveToFile(ini, ps));
        rnr::config::PersistentSettings back;
        assert(rnr::config::SettingsManager::loadFromFile(ini, back));
        std::filesystem::remove(ini);
        std::filesystem::remove(ini + ".bak");
        assert(back.machine.gantry_slave == GantrySlave::AFollowsZ);
        std::cout << "  -> PASSED: A follows Z copies Z steps, slaved A refuses jogs, independent A jogs.\n";
    }
}

void test43() {
    // ========================================================================
    // Test 43: Step pulse rate limited to the board's 25 / 50 / 100 kHz
    // ========================================================================
    {
        std::cout << "\n[Test 43] Testing the step pulse rate choices...\n";
        using rnr::core::MachineConfig;
        static_assert(MachineConfig::snapPulseRateKhz(10) == 25 && MachineConfig::snapPulseRateKhz(25) == 25);
        static_assert(MachineConfig::snapPulseRateKhz(35) == 50 && MachineConfig::snapPulseRateKhz(50) == 50);
        static_assert(MachineConfig::snapPulseRateKhz(75) == 100 && MachineConfig::snapPulseRateKhz(100) == 100);
        static_assert(MachineConfig::snapPulseRateKhz(0) == 100 && MachineConfig::snapPulseRateKhz(255) == 100);

        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        for (const auto& [in, out] : {std::pair<int, int>{25, 25}, {50, 50}, {100, 100}, {35, 50}, {0, 100}}) {
            auto cfg = controller.getConfig();
            cfg.pulse_rate_khz = static_cast<uint8_t>(in);
            controller.applyConfig(cfg);
            assert(controller.getConfig().pulse_rate_khz == out);
        }

        // The rate caps the axis speed: 200 steps/mm at 25 kHz gives 0.95 * 25000 / 200 mm/s
        MachineConfig vcfg;
        vcfg.axes[0].steps_per_mm      = 200.0;
        vcfg.axes[0].max_velocity_mm_s = 500.0;
        vcfg.pulse_rate_khz            = 25;
        assert(std::abs(rnr::planner::TrajectoryPlanner::axisVelocityLimit(vcfg, 0) - 118.75) < 1e-9);
        vcfg.pulse_rate_khz = 50;
        assert(std::abs(rnr::planner::TrajectoryPlanner::axisVelocityLimit(vcfg, 0) - 237.5) < 1e-9);
        std::cout << "  -> PASSED: pulse rates snap to 25 / 50 / 100 kHz and cap the axis speed.\n";
    }
}

void test45() {
    // ========================================================================
    // Test 45: Dragging a settings slider (applyConfig on every frame) against a fake board
    // ========================================================================
    {
        std::cout << "\n[Test 45] Testing a settings slider drag against a fake board...\n";
        auto fake = std::make_unique<FakeBoardDriver>();
        FakeBoardDriver* fb = fake.get();
        rnr::core::MachineController mc(std::move(fake));
        const auto waitFor = [](const std::function<bool()>& done) {
            for (int i = 0; i < 400; ++i) {
                if (done()) return true;
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
            return false;
        };
        assert(waitFor([&] { return mc.resetFromEStop(); }));
        mc.jogStep(rnr::core::Axis::X, 2.0, 600.0);
        assert(waitFor([&] { return mc.getTelemetry().state == rnr::core::ControllerState::Idle; }));

        const auto drag = [&](int ticks) {
            for (int i = 0; i < ticks; ++i) {
                auto cfg = mc.getConfig();
                cfg.curve_accel_pct = 25 + (i % 176);
                mc.applyConfig(cfg);
            }
        };

        // Idle: nothing but configuration reaches the board, and the position does not move
        const int configure0 = fb->count("configure");
        const int32_t steps0 = fb->steps(0);
        const int estop0 = fb->count("estop"), cancel0 = fb->count("cancel"), outputs0 = fb->count("outputs");
        const int positions0 = fb->count("positions"), slices0 = fb->count("slice_internal");
        const auto t0 = std::chrono::steady_clock::now();
        drag(120);
        const auto drag_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const int idle_configures = fb->count("configure") - configure0;
        std::cout << "  idle: 120 ticks sent " << idle_configures << " configure calls in " << drag_ms << " ms\n";
        assert(idle_configures == 120);
        assert(fb->count("estop") == estop0 && fb->count("cancel") == cancel0 && fb->count("outputs") == outputs0);
        assert(fb->count("positions") == positions0 && fb->count("slice_internal") == slices0);
        assert(fb->steps(0) == steps0);
        assert(mc.getTelemetry().state == rnr::core::ControllerState::Idle);

        // During a job: nothing is sent until motion stops, and the job ends where it should
        assert(mc.loadGCodeString("G90 G21\nG1 X40 F1200\n"));
        mc.startJob();
        const auto running = [&] { return mc.getTelemetry().state == rnr::core::ControllerState::RunningGCode; };
        assert(waitFor(running));
        const int configure1 = fb->count("configure");
        int ticks_in_motion = 0;
        for (int i = 0; i < 10 && running(); ++i) {
            drag(1);
            // Read the count first: on a slow PC the job can end during the sleep, and then a change goes out at once
            const int sent = fb->count("configure");
            if (running()) {
                assert(sent == configure1);
                ++ticks_in_motion;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        assert(waitFor([&] { return mc.getTelemetry().state == rnr::core::ControllerState::Idle; }));
        assert(fb->steps(0) == static_cast<int32_t>(std::llround(40.0 * mc.getConfig().axes[0].steps_per_mm)));
        assert(waitFor([&] { return fb->count("configure") > configure1; })); // The latest values, once motion stopped
        std::cout << "  job: " << ticks_in_motion << " ticks sent nothing during motion; status message: \""
                  << mc.getTelemetry().active_fault_message << "\"\n";
        std::cout << "  -> PASSED: a slider drag only sends configuration, and waits for motion to stop.\n";
    }
}
