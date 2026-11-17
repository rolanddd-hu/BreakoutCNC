// Settings, Mach3 import, camera, web server and version check tests
#include "test_support.h"

void test13() {
    // ------------------------------------------------------------------------
    // Test 13: Global System of Measurement (Metric vs Imperial) & Persistence
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 13] Testing Global System of Measurement (Metric vs Imperial) & Persistence...\n";

        rnr::core::RnRController controller;
        auto cfg = controller.getConfig();
        assert(cfg.linear_units == rnr::core::LinearUnits::Metric);

        // 1. Switch to Imperial
        cfg.linear_units = rnr::core::LinearUnits::Imperial;
        cfg.axes[0].steps_per_mm = 200.0;
        cfg.axes[0].max_velocity_mm_s = 100.0;
        cfg.axes[0].acceleration_mm_s2 = 500.0;
        controller.applyConfig(cfg);

        const auto& read_cfg = controller.getConfig();
        assert(read_cfg.linear_units == rnr::core::LinearUnits::Imperial);

        // 2. Mathematical unit conversions:
        // Steps per inch = steps_per_mm * 25.4
        double steps_per_inch = read_cfg.axes[0].steps_per_mm * 25.4;
        assert(std::abs(steps_per_inch - 5080.0) < 1e-4);

        // Max velocity in IPM = max_velocity_mm_s * (60.0 / 25.4)
        double vel_ipm = read_cfg.axes[0].max_velocity_mm_s * (60.0 / 25.4);
        assert(std::abs(vel_ipm - (6000.0 / 25.4)) < 1e-4);

        // Acceleration in in/s^2 = acceleration_mm_s2 / 25.4
        double accel_in_s2 = read_cfg.axes[0].acceleration_mm_s2 / 25.4;
        assert(std::abs(accel_in_s2 - (500.0 / 25.4)) < 1e-4);

        // 3. Settings persistence (save and load)
        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.machine.linear_units = rnr::core::LinearUnits::Imperial;
        ps.ui.speed_unit_idx = 1; // in/s
        assert(ps.machine.require_homing); // Default: G53 / G28 / G30 need homed axes
        ps.machine.require_homing = false;
        ps.ui.toolpath_simplify_px = 2.5f;

        std::string test_units_ini = "test_units_settings.ini";
        bool saved = rnr::config::SettingsManager::saveToFile(test_units_ini, ps);
        assert(saved);

        rnr::config::PersistentSettings loaded_ps{};
        bool loaded = rnr::config::SettingsManager::loadFromFile(test_units_ini, loaded_ps);
        assert(loaded);
        assert(loaded_ps.machine.linear_units == rnr::core::LinearUnits::Imperial);
        assert(!loaded_ps.machine.require_homing);
        assert(std::abs(loaded_ps.ui.toolpath_simplify_px - 2.5f) < 1e-5f);
        assert(loaded_ps.ui.speed_unit_idx == 1);
        std::remove(test_units_ini.c_str());

        std::cout << "  -> PASSED: Global Metric/Imperial switching, mathematical scaling & settings persistence verified.\n";
    }
}

void test14() {
    // ------------------------------------------------------------------------
    // Test 14: Mach3 XML Profile Migration, Safety Validations & Full Opcode 0x0F Packing
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 14] Testing Mach3 XML Profile Migration, Safety Validations & Opcode 0x0F Hardware Packing...\n";

        // 1. Real Mach3 profile (trimmed from a parallel-port mill with the Huanyang VFD plugin v2):
        //    metric (<Units>0), A slaved to Y, inputs on LPT port 1, plugin settings in <VfdDriver>
        rnr::core::MachineConfig base_cfg{};
        const char* lpt_mill_xml = R"XML(<profile><Preferences><PulseSpeed>1</PulseSpeed><Profile>Mach3Mill</Profile><Units>0</Units>
<ComPort>1</ComPort><BaudRate>9600</BaudRate>
<Motor0Active>1</Motor0Active><Motor0StepNeg>1</Motor0StepNeg><Motor0DirNeg>0</Motor0DirNeg>
<Motor1Active>1</Motor1Active><Motor1StepNeg>1</Motor1StepNeg><Motor1DirNeg>0</Motor1DirNeg>
<Motor2Active>1</Motor2Active><Motor2StepNeg>1</Motor2StepNeg><Motor2DirNeg>0</Motor2DirNeg>
<Motor3Active>1</Motor3Active><Motor3StepNeg>1</Motor3StepNeg><Motor3DirNeg>0</Motor3DirNeg>
<Steps0>640.</Steps0><Vel0>24.17</Vel0><Acc0>202.5</Acc0>
<Steps1>640.</Steps1><Vel1>24.17</Vel1><Acc1>200.39063</Acc1>
<Steps2>640.</Steps2><Vel2>14.48</Vel2><Acc2>167.69531</Acc2><Steps6>1.</Steps6>
<M0Max>77.66</M0Max><M0Min>0.</M0Min><M0Rev>0</M0Rev><M0Neg>1</M0Neg><RefSpeed0>50</RefSpeed0>
<M1Max>97.5</M1Max><M1Min>0.</M1Min><M1Rev>1</M1Rev><M1Neg>1</M1Neg><RefSpeed1>50</RefSpeed1>
<M2Max>0.</M2Max><M2Min>-118.5</M2Min><M2Rev>1</M2Rev><M2Neg>0</M2Neg><RefSpeed2>30</RefSpeed2>
<M3Max>97.5</M3Max><M3Min>0.</M3Min><M3Rev>1</M3Rev><M3Neg>1</M3Neg><RefSpeed3>20</RefSpeed3>
<SoftLimit>0</SoftLimit><Slave0>3</Slave0><Slave1>0</Slave1><Slave2>3</Slave2><Slave>-1</Slave>
<Input0Active>0</Input0Active><Input0Port>1</Input0Port><Input0Pin>15</Input0Pin><Input0Neg>0</Input0Neg>
<Input2Active>1</Input2Active><Input2Port>1</Input2Port><Input2Pin>15</Input2Pin><Input2Neg>1</Input2Neg><Input2Emulated>0</Input2Emulated>
<Input5Active>0</Input5Active><Input5Port>1</Input5Port><Input5Pin>0</Input5Pin><Input5Neg>0</Input5Neg>
<Input8Active>1</Input8Active><Input8Port>1</Input8Port><Input8Pin>13</Input8Pin><Input8Neg>1</Input8Neg>
<Input22Active>1</Input22Active><Input22Port>1</Input22Port><Input22Pin>12</Input22Pin><Input22Neg>1</Input22Neg>
<Input25Active>1</Input25Active><Input25Port>1</Input25Port><Input25Pin>10</Input25Pin><Input25Neg>1</Input25Neg>
<Input26Active>0</Input26Active><THCOn>0</THCOn><PlasmaMode>0</PlasmaMode>
<SpinDelayCW>1.</SpinDelayCW><SpinDownCW>1.</SpinDownCW><PULLEY>3</PULLEY>
<SPEED3>4000</SPEED3><LSPEED3>0</LSPEED3><SPEED4>24000</SPEED4><LSPEED4>5000</LSPEED4>
<IGNORETOOL>1</IGNORETOOL><AutoTool>0</AutoTool><UseSafeZ>0</UseSafeZ><m_SafeZ>0.</m_SafeZ><DwellinMilli>0</DwellinMilli>
</Preferences><GCodeVar><GVar501>0.</GVar501></GCodeVar>
<PlugIns><VFD-Driver-by-royaumedeole.fr>1</VFD-Driver-by-royaumedeole.fr><SystemTest>1</SystemTest></PlugIns>
<VfdDriver><ComPort>7</ComPort><BaudRate>9600</BaudRate><Address>1</Address><PulleyOption>1</PulleyOption><EStopOption>0</EStopOption><Status>62</Status><Margin>3</Margin><BadSpeedThreshold>3</BadSpeedThreshold><BadtrackingThreshold>3</BadtrackingThreshold><EStopCauseOption>0</EStopCauseOption><Status1_dro>0</Status1_dro><Status1_lim>0.00</Status1_lim><Status2_dro>1002</Status2_dro><Status2_lim>9.3</Status2_lim><Status3_dro>0</Status3_dro><Status3_lim>0</Status3_lim></VfdDriver></profile>)XML";

        rnr::config::Mach3ImportOptions opts{};
        auto r = rnr::config::Mach3XmlImporter::importFromString(lpt_mill_xml, base_cfg, opts, "Mach3Mill.xml");
        const auto& c = r.migrated_config;
        assert(r.success);
        assert(!r.detected_imperial_units && c.linear_units == rnr::core::LinearUnits::Metric);
        assert(r.detected_mill_profile && !r.detected_plasma_profile && !r.detected_lathe_profile);
        assert(!r.detected_rnr_profile && r.detected_lpt_parallel_port);
        // Motor tuning in mm (Mach3 stores units/sec)
        assert(std::abs(c.axes[0].steps_per_mm - 640.0) < 1e-9);
        assert(std::abs(c.axes[0].max_velocity_mm_s - 24.17) < 1e-9);
        assert(std::abs(c.axes[0].acceleration_mm_s2 - 202.5) < 1e-9);
        assert(std::abs(c.axes[2].max_velocity_mm_s - 14.48) < 1e-9);
        // Polarity: Step LowActive on all motors, Dir = DirNeg XOR Reversed
        for (int a = 0; a < 4; ++a) assert(c.axes[a].step_active_low);
        assert(!c.axes[0].dir_active_low && c.axes[1].dir_active_low && c.axes[2].dir_active_low && c.axes[3].dir_active_low);
        // Homing: X/Y toward -, Z up; speed % of max velocity
        assert(c.axes[0].homing_dir_negative && !c.axes[2].homing_dir_negative);
        assert(std::abs(c.axes[0].homing_feed_mm_min - 0.5 * 24.17 * 60.0) < 1e-6);
        // Soft limits imported but off (SoftLimit = 0)
        assert(std::abs(c.axes[0].soft_limit_max_mm - 77.66) < 1e-9 && !c.axes[0].soft_limits_enabled);
        assert(std::abs(c.axes[2].soft_limit_min_mm + 118.5) < 1e-9);
        // A slaved to Y takes Y's tuning
        assert(c.gantry_slave == rnr::core::GantrySlave::AFollowsY && c.gantryMasterAxis() == 1);
        assert(std::abs(c.axes[3].steps_per_mm - 640.0) < 1e-9);
        // Step rate: Mach3 kernel 35 kHz (PulseSpeed list position 1) covers 15.5 kHz needed, rounded up to 50
        assert(c.pulse_rate_khz == 50);
        // Parallel-port inputs: suggested wiring plan
        assert(c.estop_pin == 1 && c.estop_invert);
        assert(c.probe.probe_pin == 3 && c.probe.probe_invert);
        assert(c.axes[2].home_pin == 4 && c.axes[0].home_pin == 5);
        assert(c.axes[1].home_pin == 0 && c.axes[0].limit_plus_pin == 0);
        assert(r.lpt_remaps.size() == 4);
        assert(!c.limit_override);
        // Huanyang plugin: <VfdDriver> port, not Mach3's own <ComPort>
        assert(r.detected_vfd_plugin && c.vfd.enabled);
#ifdef _WIN32
        assert(c.vfd.port_name == "COM7");
#else
        // COM numbers mean nothing off Windows: the configured device path is kept
        assert(c.vfd.port_name == base_cfg.vfd.port_name);
#endif
        assert(c.vfd.baud_rate == 9600 && c.vfd.slave_address == 1);
        assert(!c.vfd.speed_monitor_enabled && c.vfd.speed_margin_pct == 3);
        assert(c.vfd.bad_speed_threshold == 3 && c.vfd.tracking_threshold == 3);
        assert(std::abs(c.vfd.limit_current_a - 9.3f) < 1e-4f && c.vfd.limit_out_freq_hz == 0.0f);
        // Active pulley (PULLEY counts from 0): SPEED4 / LSPEED4
        assert(c.vfd.max_rpm == 24000 && c.vfd.min_rpm == 5000);
        // Spindle delays, M6 and dwell
        assert(std::abs(c.spindle_ramp_up_s - 1.0) < 1e-9 && std::abs(c.spindle_ramp_down_s - 1.0) < 1e-9);
        assert(c.tool_change.mode == rnr::core::ToolChangeMode::IgnoreM6);
        assert(!c.dwell_p_milliseconds);

        // Wiring plan choice by the operator
        opts.lpt_pin_overrides["E-Stop"] = 6;
        opts.lpt_pin_overrides["Probe (G31)"] = 0;
        auto r2 = rnr::config::Mach3XmlImporter::importFromString(lpt_mill_xml, base_cfg, opts, "Mach3Mill.xml");
        assert(r2.migrated_config.estop_pin == 6 && r2.migrated_config.probe.probe_pin == 0);

        // Forcing inches converts with 25.4 and flags the implausible result
        rnr::config::Mach3ImportOptions inch_opts{};
        inch_opts.units_override = 1;
        auto r3 = rnr::config::Mach3XmlImporter::importFromString(lpt_mill_xml, base_cfg, inch_opts, "Mach3Mill.xml");
        assert(r3.detected_imperial_units);
        assert(std::abs(r3.migrated_config.axes[0].steps_per_mm - 640.0 / 25.4) < 1e-9);
        bool implausible = false;
        for (const auto& it : r3.items) implausible |= (it.parameter_name == "Units Plausibility");
        assert(implausible);

        // 2. Profile used with the RNR plugin: only Port 3 inputs reached the board, inch units, A slaved to X
        const char* rnr_inch_xml = R"XML(<?xml version="1.0"?>
< profile ><Preferences><Profile>Plasma Table</Profile><Units>1</Units><PlasmaMode>1</PlasmaMode>
<Steps0>10160.</Steps0><Vel0>2.54</Vel0><Acc0>10.</Acc0><Motor0DirNeg>1</Motor0DirNeg><Motor0StepNeg>1</Motor0StepNeg>
<M0Min>0.</M0Min><M0Max>24.</M0Max><SoftLimit>1</SoftLimit><Slave0>0</Slave0><Slave1>3</Slave1><Slave2>3</Slave2>
<Input22Active>1</Input22Active><Input22Port>3</Input22Port><Input22Pin>4</Input22Pin><Input22Neg>1</Input22Neg>
<Input25Active>1</Input25Active><Input25Port>1</Input25Port><Input25Pin>11</Input25Pin><Input25Neg>1</Input25Neg>
<LimitOverRide>1</LimitOverRide></Preferences>
<RnRMotion><PulseRates>75</PulseRates><XPullOff>0.1</XPullOff></RnRMotion></ profile >)XML";
        auto ri = rnr::config::Mach3XmlImporter::importFromString(rnr_inch_xml, base_cfg, {}, "Plasma.xml");
        const auto& ci = ri.migrated_config;
        assert(ri.success && ri.detected_imperial_units && ri.detected_rnr_profile && ri.detected_plasma_profile);
        assert(std::abs(ci.axes[0].steps_per_mm - 400.0) < 1e-9);
        assert(std::abs(ci.axes[0].max_velocity_mm_s - 2.54 * 25.4) < 1e-9);
        assert(std::abs(ci.axes[0].acceleration_mm_s2 - 254.0) < 1e-9);
        assert(std::abs(ci.axes[0].soft_limit_max_mm - 24.0 * 25.4) < 1e-9 && ci.axes[0].soft_limits_enabled);
        assert(std::abs(ci.axes[0].home_pulloff_mm - 2.54) < 1e-9); // Pull-off in native units
        assert(ci.axes[0].dir_active_low && ci.axes[0].step_active_low);
        assert(ci.pulse_rate_khz == 100); // 75 is not a board rate: rounded up
        assert(ci.gantry_slave == rnr::core::GantrySlave::AFollowsX);
        assert(ci.probe.probe_pin == 4 && ci.probe.probe_invert);
        assert(ci.estop_pin == 0);   // Port 1 input was ignored by the RNR plugin
        assert(!ci.limit_override);  // Never imported
        assert(ri.lpt_remaps.empty());

        // 3. Lathe profiles are flagged; non-profile XML is rejected
        auto rl = rnr::config::Mach3XmlImporter::importFromString(
            "<profile><Preferences><Profile>cnc lathe</Profile><Units>1</Units><Steps0>16000.</Steps0><Vel0>1.333</Vel0></Preferences></profile>",
            base_cfg, {}, "lathe.xml");
        assert(rl.success && rl.detected_lathe_profile && rl.detected_imperial_units);
        auto rx = rnr::config::Mach3XmlImporter::importFromString("<config><Units>0</Units></config>", base_cfg, {}, "x.xml");
        assert(!rx.success);
        assert(rnr::config::Mach3XmlImporter::kernelSpeedKhz(0) == 25 && rnr::config::Mach3XmlImporter::kernelSpeedKhz(6) == 100);

        // 3. Test Full Opcode 0x0F Hardware Packet Packing (normal and homing layout)
        rnr::protocol::BoardPinoutConfig pinout{};
        pinout.pulse_rate_khz = 75;
        pinout.slice_interval_ms = 25;
        pinout.axes[0].step_active_low = true;
        pinout.axes[2].step_active_low = true; // X & Z step active-low (0x05)
        pinout.axes[1].dir_active_low  = true;
        pinout.axes[3].dir_active_low  = true; // Y & A dir active-low (0x0A)
        pinout.probe_pin = 4;
        pinout.probe_invert = true;
        pinout.estop_pin = 1;
        pinout.estop_invert = true;
        pinout.axes[0].home_pin = 2;
        pinout.axes[0].limit_minus_pin = 2; // Shared with home_pin!
        pinout.axes[0].limit_plus_pin = 3;
        pinout.axes[0].home_invert = true;

        // Idle mode (mask_shared_home_limits = false)
        pinout.mask_shared_home_limits = false;
        auto pkt_idle = rnr::protocol::buildFullConfigureBoardPacket(0x0042, pinout);
        assert(pkt_idle[0] == static_cast<uint8_t>(rnr::protocol::FrameType::ExtendedPacket));
        assert(pkt_idle[1] == static_cast<uint8_t>(rnr::protocol::Opcode::ConfigureBoard));
        assert(pkt_idle[3] == 0x42);
        // Axis 0 byte 0: (limit_minus_pin << 4) | limit_plus_pin = (2 << 4) | 3 = 0x23
        assert(pkt_idle[5] == 0x23);
        // Axis 0 byte 1: ((home_inv<<2) << 4) | home_pin = (4 << 4) | 2 = 0x42
        assert(pkt_idle[6] == 0x42);
        // buf[13]: (mpg_pin << 4) | probe_pin = 0x04
        assert(pkt_idle[13] == 0x04);
        // buf[14]: ((estop_inv<<2 | probe_inv) << 4) | estop_pin = (5 << 4) | 1 = 0x51
        assert(pkt_idle[14] == 0x51);
        // buf[15]: (dir_mask << 4) | step_mask = (0x0A << 4) | 0x05 = 0xA5
        assert(pkt_idle[15] == 0xA5);
        assert(pkt_idle[16] == 75);
        assert(pkt_idle[17] == 25);

        // Homing mode (mask_shared_home_limits = true)
        // Since limit_minus_pin (2) == home_pin (2), limit_minus_pin is zeroed in buf[5] high nibble!
        pinout.mask_shared_home_limits = true;
        auto pkt_homing = rnr::protocol::buildFullConfigureBoardPacket(0x0043, pinout);
        assert(pkt_homing[5] == 0x03); // limit_minus zeroed, limit_plus=3 remains

        std::cout << "  -> PASSED: Mach3 XML Metric/Imperial migration, safety swaps, LPT remapping & Opcode 0x0F packing verified.\n";
    }
}

void test15() {
    // ------------------------------------------------------------------------
    // Test 15: Camera Support — Dual Role Assignment, Calibration, Zeroing, JPEG Stream & Settings
    // ------------------------------------------------------------------------
    {
        std::cout << "\n[Test 15] Testing Camera Manager, Exclusive 1/2-Cam Role Assignment, Calibration, Zeroing & JPEG Encoding...\n";

        rnr::core::CameraManager cam_mgr;
        auto devs = cam_mgr.enumerateDevices(true);
        assert(devs.size() >= 2); // At least the 2 virtual simulator cameras are present

        int sim_top_idx = static_cast<int>(devs.size()) - 2;
        int sim_mon_idx = static_cast<int>(devs.size()) - 1;

        // 1. Assign 2 distinct cameras in BreakoutMill mode (allow_top_down_role = true)
        rnr::core::CameraConfig cam_cfg{};
        cam_cfg.top_down_camera_index   = sim_top_idx;
        cam_cfg.monitoring_camera_index = sim_mon_idx;
        cam_mgr.applyConfig(cam_cfg, true, true);
        assert(cam_cfg.top_down_camera_index == sim_top_idx);
        assert(cam_cfg.monitoring_camera_index == sim_mon_idx);
        assert(cam_mgr.isRoleActive(rnr::core::CameraRole::TopDown));
        assert(cam_mgr.isRoleActive(rnr::core::CameraRole::Monitoring));

        // 2. Single-camera exclusive switching: if monitoring selects the same camera index as top-down,
        // top-down is automatically unassigned so hardware handles never conflict
        cam_cfg.monitoring_camera_index = sim_top_idx;
        cam_mgr.applyConfig(cam_cfg, true, true);
        assert(cam_cfg.monitoring_camera_index == sim_top_idx);
        assert(cam_cfg.top_down_camera_index == -1);
        assert(!cam_mgr.isRoleActive(rnr::core::CameraRole::TopDown));
        assert(cam_mgr.isRoleActive(rnr::core::CameraRole::Monitoring));

        // 3. BreakoutCut mode (allow_top_down_role = false) forces Top-Down off and keeps Monitoring only
        cam_cfg.top_down_camera_index   = sim_top_idx;
        cam_cfg.monitoring_camera_index = sim_mon_idx;
        cam_mgr.applyConfig(cam_cfg, true, false);
        assert(cam_cfg.top_down_camera_index == -1);
        assert(cam_cfg.monitoring_camera_index == sim_mon_idx);
        assert(!cam_mgr.isRoleActive(rnr::core::CameraRole::TopDown));
        assert(cam_mgr.isRoleActive(rnr::core::CameraRole::Monitoring));

        // 4. Verify live frame generation & Baseline JPEG encoder (SOI 0xFFD8 .. EOI 0xFFD9)
        cam_cfg.top_down_camera_index   = sim_top_idx;
        cam_cfg.monitoring_camera_index = sim_mon_idx;
        cam_mgr.applyConfig(cam_cfg, true, true);

        rnr::core::CameraFrame frame{};
        uint64_t seq = 0;
        // The simulated camera's first frame arrives from its own thread
        assert(waitUntil([&] { return cam_mgr.getLatestFrame(rnr::core::CameraRole::TopDown, frame, seq); }));
        assert(frame.width == 640 && frame.height == 360);
        assert(frame.rgba.size() == static_cast<size_t>(640 * 360 * 4));

        std::vector<uint8_t> jpeg_bytes;
        rnr::core::CameraStreamServer::encodeRgbaToJpeg(frame.rgba.data(), frame.width, frame.height, jpeg_bytes);
        assert(jpeg_bytes.size() > 512);
        assert(jpeg_bytes[0] == 0xFF && jpeg_bytes[1] == 0xD8); // SOI
        assert(jpeg_bytes[jpeg_bytes.size() - 2] == 0xFF && jpeg_bytes[jpeg_bytes.size() - 1] == 0xD9); // EOI

        cam_mgr.stopAll();

        // 5. Verify Camera Offset Zeroing & Settings Persistence
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        auto mcfg = controller.getConfig();
        mcfg.camera.top_down_camera_index   = 0;
        mcfg.camera.monitoring_camera_index = 1;
        mcfg.camera.is_calibrated           = true;
        mcfg.camera.offset_x_mm             = 45.0;
        mcfg.camera.offset_y_mm             = 30.0;
        mcfg.camera.fov_x_mm                = 120.0;
        mcfg.camera.fov_y_mm                = 67.5;
        mcfg.camera.stream_enabled          = false;
        mcfg.camera.stream_port             = 8090;
        controller.applyConfig(mcfg);

        // Zero Work XY at Camera Crosshair: sets current tool Work XY to (offset_x_mm, offset_y_mm)
        // so that (work_pos - offset) = (0.0, 0.0) under the camera crosshair!
        controller.setWorkPosition(rnr::core::Axis::X, mcfg.camera.offset_x_mm);
        controller.setWorkPosition(rnr::core::Axis::Y, mcfg.camera.offset_y_mm);
        auto tel = controller.getTelemetry();
        assert(std::abs((tel.work_pos_mm[0] - mcfg.camera.offset_x_mm) - 0.0) < 1e-5);
        assert(std::abs((tel.work_pos_mm[1] - mcfg.camera.offset_y_mm) - 0.0) < 1e-5);

        rnr::config::PersistentSettings ps{};
        ps.machine = controller.getConfig();
        ps.ui.header_camera_open = true;
        std::string test_cam_ini = "test_camera_settings.ini";
        assert(rnr::config::SettingsManager::saveToFile(test_cam_ini, ps));

        rnr::config::PersistentSettings loaded_ps{};
        assert(rnr::config::SettingsManager::loadFromFile(test_cam_ini, loaded_ps));
        assert(loaded_ps.ui.header_camera_open == true);
        assert(loaded_ps.machine.camera.top_down_camera_index == 0);
        assert(loaded_ps.machine.camera.monitoring_camera_index == 1);
        assert(loaded_ps.machine.camera.is_calibrated == true);
        assert(std::abs(loaded_ps.machine.camera.offset_x_mm - 45.0) < 1e-4);
        assert(std::abs(loaded_ps.machine.camera.offset_y_mm - 30.0) < 1e-4);
        assert(std::abs(loaded_ps.machine.camera.fov_x_mm - 120.0) < 1e-4);
        assert(std::abs(loaded_ps.machine.camera.fov_y_mm - 67.5) < 1e-4);
        assert(loaded_ps.machine.camera.stream_port == 8090);
        std::remove(test_cam_ini.c_str());

        controller.disconnect();
        std::cout << "  -> PASSED: Camera enumeration, exclusive 1/2-cam assignment, calibration, zeroing, JPEG stream & INI persistence verified.\n";
    }
}

void test23() {
    // ----------------------------------------------------
    // Test 23: Remote Web G-Code Upload & Toolpath Curve Grouping
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 23] Testing Remote Web G-Code Upload Callback & Curve Highlighting Grouping...\n";

        // 1. Curve span grouping: 10 arc chords of same gcode_line
        std::vector<rnr::core::ToolpathSegment> segments;
        for (int i = 0; i < 10; ++i) {
            rnr::core::ToolpathSegment s{};
            s.type          = rnr::core::SegmentType::CutArcCW;
            s.segment_index = static_cast<size_t>(i);
            s.gcode_line    = 42;
            s.start_xyza_mm = {i * 1.0, 0.0, 0.0, 0.0};
            s.end_xyza_mm   = {(i + 1) * 1.0, 0.0, 0.0, 0.0};
            s.length_mm     = 1.0;
            segments.push_back(s);
        }
        // Linear move afterwards
        rnr::core::ToolpathSegment s_lin{};
        s_lin.type          = rnr::core::SegmentType::CutLinear;
        s_lin.segment_index = 10;
        s_lin.gcode_line    = 43;
        s_lin.length_mm     = 25.0;
        segments.push_back(s_lin);

        // Grouping from active_idx = 2
        size_t active_idx = 2;
        size_t active_end_idx = active_idx;
        const auto& active_seg = segments[active_idx];
        if (active_seg.type == rnr::core::SegmentType::CutArcCW || active_seg.type == rnr::core::SegmentType::CutArcCCW) {
            while (active_end_idx + 1 < segments.size()) {
                const auto& nxt = segments[active_end_idx + 1];
                if (nxt.gcode_line == active_seg.gcode_line &&
                    (nxt.type == rnr::core::SegmentType::CutArcCW || nxt.type == rnr::core::SegmentType::CutArcCCW))
                {
                    active_end_idx++;
                } else {
                    break;
                }
            }
        }
        assert(active_end_idx == 9); // All remaining 7 arc chords are grouped together!

        // 2. CameraStreamServer callback testing
        rnr::core::CameraManager cam_mgr;
        rnr::core::CameraStreamServer stream_server(cam_mgr);
        rnr::core::RnRController controller;
        controller.setSimulatorMode(true);
        controller.connect();
        controller.resetFromEStop();

        stream_server.setLoadGCodeCallback([&](const std::string& filename, const std::string& content) -> rnr::core::LoadGCodeResult {
            const auto tel = controller.getTelemetry();
            if (tel.state == rnr::core::ControllerState::RunningGCode) {
                return {false, "Machine is currently running", 0};
            }
            if (!controller.loadGCodeString(content)) {
                return {false, "Failed to parse G-code", 0};
            }
            return {true, "", (*controller.getProgram()).segments.size()};
        });

        // Test uploading sample G-code
        std::string sample_gcode = "G90\nG0 X0 Y0 Z5\nG1 X50 Y50 F1200\n";
        auto upload_res = controller.loadGCodeString(sample_gcode);
        assert(upload_res);

        // Update telemetry with file name and verify
        auto tel = controller.getTelemetry();
        stream_server.updateTelemetry(tel, rnr::core::LinearUnits::Metric, "BreakoutMill", "remotely_uploaded_bracket.nc");

        controller.disconnect();
        std::cout << "  -> PASSED: Arc curve grouping correctly captures full arc span (chords 2..9); remote upload callback verified.\n";
    }
}

void test23b() {
    // ----------------------------------------------------
    // Test 23b: Remote monitoring server protections
    // ----------------------------------------------------
    {
        std::cout << "\n[Test 23b] Remote monitoring server: host / origin checks, access code, uploads, limits...\n";
        using rnr::core::CameraStreamServer;

        // Upload file names: base name, safe characters, no Windows device names
        assert(CameraStreamServer::sanitizeUploadFileName("COM3.nc") == "_COM3.nc");
        assert(CameraStreamServer::sanitizeUploadFileName("nul") == "_nul");
        assert(CameraStreamServer::sanitizeUploadFileName("lpt1 .tap") == "_lpt1 .tap");
        assert(CameraStreamServer::sanitizeUploadFileName("..\\..\\Windows\\evil.nc") == "evil.nc");
        assert(CameraStreamServer::sanitizeUploadFileName("../../etc/x.nc") == "x.nc");
        assert(CameraStreamServer::sanitizeUploadFileName("a:b<c>.nc") == "a_b_c_.nc");
        assert(CameraStreamServer::sanitizeUploadFileName("...") == "remote_program.nc");
        assert(CameraStreamServer::sanitizeUploadFileName("") == "remote_program.nc");
        assert(CameraStreamServer::sanitizeUploadFileName("bracket v2.nc") == "bracket v2.nc");
        assert(CameraStreamServer::sanitizeUploadFileName("console.nc") == "console.nc");
        assert(CameraStreamServer::sanitizeUploadFileName(std::string(300, 'a') + ".nc").size() == 100);

        // Host header: IP address, localhost or this PC's name; a web site's name (DNS rebinding) is refused
        assert(CameraStreamServer::isAllowedHost("192.168.1.20:8080", "shop-pc"));
        assert(CameraStreamServer::isAllowedHost("localhost:8080", "shop-pc"));
        assert(CameraStreamServer::isAllowedHost("[::1]:8080", "shop-pc"));
        assert(CameraStreamServer::isAllowedHost("SHOP-PC:8080", "shop-pc"));
        assert(CameraStreamServer::isAllowedHost("shop-pc.local", "shop-pc"));
        assert(!CameraStreamServer::isAllowedHost("evil.example:8080", "shop-pc"));
        assert(!CameraStreamServer::isAllowedHost("shop-pcx", "shop-pc"));
        assert(!CameraStreamServer::isAllowedHost("", "shop-pc"));
        assert(!CameraStreamServer::isAllowedHost("192.168.1.20:80x", "shop-pc"));
        assert(!CameraStreamServer::isAllowedHost("[2001:db8::1]:8080", "shop-pc"));

        rnr::core::CameraManager cam_mgr;
        CameraStreamServer srv(cam_mgr);
        std::atomic<int> estops{0};
        std::mutex upload_mtx;
        std::string uploaded_name;
        srv.setEStopCallback([&]() { ++estops; });
        srv.setLoadGCodeCallback([&](const std::string& fn, const std::string&) -> rnr::core::LoadGCodeResult {
            std::lock_guard<std::mutex> lock(upload_mtx);
            uploaded_name = fn;
            return {true, "", 3};
        });
        const uint16_t port = 18573;
        rnr::core::CameraConfig cfg;
        cfg.stream_enabled = true;
        cfg.stream_port = port;
        cfg.stream_bind_address = "127.0.0.1";
        cfg.stream_allow_upload = true;
        srv.applyConfig(cfg);
        assert(srv.isRunning());
        assert(srv.getServerUrl() == "http://127.0.0.1:18573");
        const std::string H = "127.0.0.1:18573";
        const std::string kDash = "X-BreakoutCNC: 1\r\n";

        // Page (with anti-framing headers), wrong host, unknown path, cross-site read
        std::string r = httpRaw(port, httpReq("GET", "/", H, "", ""));
        assert(httpStatus(r) == 200 && r.find("frame-ancestors 'none'") != std::string::npos);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/", "evil.example:18573", "", ""))) == 403);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/nothing", H, "", ""))) == 404);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/api/telemetry", H, "Sec-Fetch-Site: cross-site\r\n", ""))) == 403);
        r = httpRaw(port, httpReq("GET", "/api/telemetry", H, "", ""));
        assert(httpStatus(r) == 200 && r.find("\"state\"") != std::string::npos);

        // E-stop: only with the dashboard's header and from this server's own page
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/estop", H, "", ""))) == 403);
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/estop", H, kDash + "Origin: http://evil.example\r\n", ""))) == 403);
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/estop", H, kDash + "Sec-Fetch-Site: same-site\r\n", ""))) == 403);
        assert(estops.load() == 0);
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/estop", H,
                                                kDash + "Origin: http://127.0.0.1:18573\r\nSec-Fetch-Site: same-origin\r\n", ""))) == 200);
        assert(estops.load() == 1);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/api/estop", H, "", ""))) == 405);
        assert(httpStatus(httpRaw(port, httpReq("OPTIONS", "/api/estop", H, "Origin: http://evil.example\r\n", ""))) == 405);

        // Upload: a device name is defused; without the dashboard header nothing is loaded
        r = httpRaw(port, httpReq("POST", "/api/upload_gcode?filename=COM3.nc", H, kDash + "Content-Type: text/plain\r\n", "G0 X1\n"));
        assert(httpStatus(r) == 200);
        {
            std::lock_guard<std::mutex> lock(upload_mtx);
            assert(uploaded_name == "_COM3.nc");
            uploaded_name.clear();
        }
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/upload_gcode", H, "Content-Type: text/plain\r\n", "G0 X1\n"))) == 403);
        {
            std::lock_guard<std::mutex> lock(upload_mtx);
            assert(uploaded_name.empty());
        }
        // Upload switched off: refused at once, nothing of the announced body is read
        cfg.stream_allow_upload = false;
        srv.applyConfig(cfg);
        assert(srv.isRunning());
        r = httpRaw(port, "POST /api/upload_gcode HTTP/1.1\r\nHost: " + H + "\r\n" + kDash + "Content-Length: 30000000\r\n\r\n");
        assert(httpStatus(r) == 403);
        cfg.stream_allow_upload = true;
        srv.applyConfig(cfg);
        r = httpRaw(port, "POST /api/upload_gcode HTTP/1.1\r\nHost: " + H + "\r\n" + kDash + "Content-Length: 40000000\r\n\r\n");
        assert(httpStatus(r) == 413);
        r = httpRaw(port, "POST /api/upload_gcode HTTP/1.1\r\nHost: " + H + "\r\n" + kDash + "Transfer-Encoding: chunked\r\n\r\n");
        assert(httpStatus(r) == 411);

        // Access code: everything but the page needs a session from /api/login
        cfg.stream_access_code = "4711";
        srv.applyConfig(cfg);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/", H, "", ""))) == 200);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/api/telemetry", H, "", ""))) == 401);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/stream.mjpg", H, "", ""))) == 401);
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/estop", H, kDash, ""))) == 401);
        assert(estops.load() == 1);
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/login", H, "", "4711"))) == 403); // Dashboard header
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/login", H, kDash, "1234"))) == 401);
        r = httpRaw(port, httpReq("POST", "/api/login", H, kDash, "4711"));
        assert(httpStatus(r) == 200);
        const size_t cpos = r.find("bcnc_session=");
        assert(cpos != std::string::npos);
        assert(r.find("HttpOnly; SameSite=Strict") != std::string::npos);
        const std::string token = r.substr(cpos + 13, 32);
        const std::string cookie = "Cookie: other=1; bcnc_session=" + token + "\r\n";
        assert(httpStatus(httpRaw(port, httpReq("GET", "/api/telemetry", H, cookie, ""))) == 200);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/api/telemetry", H,
                                                "Cookie: bcnc_session=00000000000000000000000000000000\r\n", ""))) == 401);
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/estop", H, kDash + cookie, ""))) == 200);
        assert(estops.load() == 2);
        // Wrong codes lock this address out for a minute, even for the right code
        for (int i = 0; i < CameraStreamServer::kMaxLoginFailures; ++i) {
            assert(httpStatus(httpRaw(port, httpReq("POST", "/api/login", H, kDash, "0000"))) == 401);
        }
        assert(httpStatus(httpRaw(port, httpReq("POST", "/api/login", H, kDash, "4711"))) == 429);
        // A new code signs everybody out
        cfg.stream_access_code = "9999";
        srv.applyConfig(cfg);
        assert(httpStatus(httpRaw(port, httpReq("GET", "/api/telemetry", H, cookie, ""))) == 401);
        cfg.stream_access_code.clear();
        srv.applyConfig(cfg);

        // A camera viewer that leaves frees its place, also while no camera sends frames
        {
            rnr::transport::TcpStream s;
            std::string err;
            assert(s.open(rnr::transport::ConnectionTarget{"tcp", "127.0.0.1:18573", ""}, rnr::transport::StreamOptions{}, &err));
            const std::string get = httpReq("GET", "/stream.mjpg", H, "", "");
            assert(s.write(reinterpret_cast<const uint8_t*>(get.data()), get.size()));
            auto waitStreams = [&](int n) {
                const auto t0 = std::chrono::steady_clock::now();
                while (srv.getActiveStreamCount() != n && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(3)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                return srv.getActiveStreamCount() == n;
            };
            assert(waitStreams(1));
            s.close();
            assert(waitStreams(0));
        }

        // Limits: connections from one address are capped, and stop() returns promptly with idle clients connected
        {
            std::vector<std::unique_ptr<rnr::transport::TcpStream>> idle;
            for (int i = 0; i < 10; ++i) {
                auto s = std::make_unique<rnr::transport::TcpStream>();
                std::string err;
                rnr::transport::ConnectionTarget t{"tcp", "127.0.0.1:18573", ""};
                if (s->open(t, rnr::transport::StreamOptions{}, &err)) idle.push_back(std::move(s));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            assert(srv.getActiveClientCount() <= CameraStreamServer::kMaxClientsPerAddr);
            const auto t0 = std::chrono::steady_clock::now();
            srv.stop();
            assert(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(3));
            assert(!srv.isRunning() && srv.getActiveClientCount() == 0);
        }
        // A client that sends its request a byte at a time is dropped after the head deadline
        srv.applyConfig(cfg);
        assert(srv.isRunning());
        {
            rnr::transport::TcpStream s;
            std::string err;
            assert(s.open(rnr::transport::ConnectionTarget{"tcp", "127.0.0.1:18573", ""}, rnr::transport::StreamOptions{}, &err));
            const auto t0 = std::chrono::steady_clock::now();
            bool closed = false;
            while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(9)) {
                const uint8_t b = 'G';
                if (!s.write(&b, 1)) { closed = true; break; }
                uint8_t buf[64];
                if (s.read(buf, sizeof(buf), 1500) < 0) { closed = true; break; }
            }
            assert(closed);
            assert(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(8));
        }
        // Restarting on the same port at once (the panel restarts the server when the port or address changes)
        for (int i = 0; i < 5; ++i) {
            srv.stop();
            srv.applyConfig(cfg);
            assert(srv.isRunning());
            assert(httpStatus(httpRaw(port, httpReq("GET", "/api/telemetry", H, "", ""))) == 200);
        }
        // The port is taken: a second server says so instead of silently sharing it
        {
            CameraStreamServer srv2(cam_mgr);
            srv2.applyConfig(cfg);
            assert(!srv2.isRunning() && !srv2.lastError().empty());
        }
        assert(httpStatus(httpRaw(port, httpReq("GET", "/api/telemetry", H, "", ""))) == 200);
        srv.stop();
        std::cout << "  -> PASSED: device names defused, foreign hosts / origins / cross-site actions refused, access code "
                     "and lockout, upload limits, connection caps, head deadline, restart and exclusive port.\n";
    }
}

void test34() {
    // ========================================================================
    // Test 34: Settings file recovery after an interrupted save
    // ========================================================================
    {
        std::cout << "\n[Test 34] Testing atomic settings save and .bak recovery...\n";
        const std::string ini = "test_atomic_settings.ini";
        rnr::config::PersistentSettings a{};
        a.machine.axes[0].steps_per_mm = 321.0;
        assert(rnr::config::SettingsManager::saveToFile(ini, a));
        rnr::config::PersistentSettings b = a;
        b.machine.axes[0].steps_per_mm = 654.0;
        assert(rnr::config::SettingsManager::saveToFile(ini, b)); // a is now the .bak
        // Simulate a power loss half-way through writing the file
        std::string content;
        {
            std::ifstream ifs(ini);
            std::stringstream ss;
            ss << ifs.rdbuf();
            content = ss.str();
        }
        {
            std::ofstream ofs(ini, std::ios::trunc);
            ofs << content.substr(0, content.size() / 2);
        }
        rnr::config::PersistentSettings loaded{};
        assert(rnr::config::SettingsManager::loadFromFile(ini, loaded));
        assert(std::abs(loaded.machine.axes[0].steps_per_mm - 321.0) < 1e-9);
        assert(!loaded.machine.limit_override);
        std::remove(ini.c_str());
        std::remove((ini + ".bak").c_str());
        std::cout << "  -> PASSED: Truncated settings fall back to the last complete save.\n";
    }
}

void test49() {
    // ========================================================================
    // Test 49: New version check: version comparison and the GitHub release response (no network)
    // ========================================================================
    {
        std::cout << "\n[Test 49] Testing the new version check...\n";
        using rnr::core::newerVersionInReleaseJson;
        const auto release = [](const std::string& tag) {
            return "{\"url\":\"x\",\"author\":{\"login\":\"a\"},\"tag_name\": \"" + tag + "\",\"body\":\"text\"}";
        };

        assert(newerVersionInReleaseJson(release("v0.3.2"), "0.3.1") == "0.3.2");
        assert(newerVersionInReleaseJson(release("v0.4"), "0.3.1") == "0.4");
        assert(newerVersionInReleaseJson(release("v1.0"), "0.9.9") == "1.0");
        assert(newerVersionInReleaseJson(release("0.10"), "0.9") == "0.10"); // Numbers, not text: 10 > 9
        assert(newerVersionInReleaseJson(release("v0.3.0.1"), "0.3") == "0.3.0.1");

        assert(newerVersionInReleaseJson(release("v0.3.1"), "0.3.1").empty()); // The same version
        assert(newerVersionInReleaseJson(release("v0.3"), "0.3.0").empty());   // Missing numbers count as 0
        assert(newerVersionInReleaseJson(release("v0.3"), "0.3.1").empty());   // This build is newer

        // Anything unexpected shows no label
        assert(newerVersionInReleaseJson(release("v0.4-beta"), "0.3.1").empty());
        assert(newerVersionInReleaseJson(release("latest"), "0.3.1").empty());
        assert(newerVersionInReleaseJson(release(""), "0.3.1").empty());
        assert(newerVersionInReleaseJson(release("v"), "0.3.1").empty());
        assert(newerVersionInReleaseJson(release("v1..2"), "0.3.1").empty());
        assert(newerVersionInReleaseJson(release("v1."), "0.3.1").empty());
        assert(newerVersionInReleaseJson(release("v99999999999"), "0.3.1").empty());
        assert(newerVersionInReleaseJson(release("v1.2.3.4.5"), "0.3.1").empty());
        assert(newerVersionInReleaseJson(release("v0.4"), "unknown").empty());
        assert(newerVersionInReleaseJson("", "0.3.1").empty());                                  // No internet connection
        assert(newerVersionInReleaseJson("{\"message\":\"Not Found\"}", "0.3.1").empty());
        assert(newerVersionInReleaseJson("{\"tag_name\":", "0.3.1").empty());                    // Cut off
        assert(newerVersionInReleaseJson("{\"tag_name\":\"v0.4", "0.3.1").empty());
        assert(newerVersionInReleaseJson("{\"tag_name\":4}", "0.3.1").empty());

        rnr::core::UpdateChecker not_started;
        assert(not_started.availableVersion().empty());
        std::cout << "  -> PASSED: only a newer, well-formed release version is reported.\n";
    }
}
