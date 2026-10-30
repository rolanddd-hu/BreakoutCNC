#include "breakout_version.h"
#include "core/controller/rnr_controller.h"
#include "core/config/settings_manager.h"
#include "ui_common/app_window.h"
#include "ui_common/toolpath_viewport.h"
#include "ui_common/dro_panel.h"
#include "ui_common/io_status_bar.h"
#include "ui_common/gcode_panel.h"
#include "ui_common/app_dir.h"
#include "ui_common/scripting_panel.h"
#include "ui_common/workspace_panel.h"
#include "ui_common/file_dialog.h"
#include "ui_common/mach3_migration_dialog.h"
#include "ui_common/gantry_slave_combo.h"
#include "ui_common/camera_panel.h"
#include "imgui.h"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

using namespace rnr;

int main(int argc, char** argv) {
    // Before anything reads the settings file, scripts/ or resources/ (all next to the executable)
    const std::string arg_file = ui::enterExecutableDirectory(argc, argv);
    // --smoke-test (CI): never touch USB devices (simulator), run the demo job briefly, exit without saving
    const bool smoke_test = ui::hasCommandLineFlag(argc, argv, "--smoke-test");
    if (smoke_test) core::RnRController::setHardwareAccessEnabled(false);
    const std::string settings_file = "breakout_cut_settings.ini";
    config::PersistentSettings saved_settings{};

    core::RnRController controller;
    saved_settings.machine = controller.getConfig();
    bool had_saved_settings = config::SettingsManager::loadFromFile(settings_file, saved_settings);
    if (!had_saved_settings) {
        // Fallback to legacy settings file if present
        had_saved_settings = config::SettingsManager::loadFromFile("rnr_plasma_settings.ini", saved_settings);
    }

    ui::AppWindow window(
        "BreakoutCut - Dual-Mode CNC Plasma & Laser Controller (BreakoutCNC v" BREAKOUTCNC_VERSION_STRING ")",
        saved_settings.ui.window_width,
        saved_settings.ui.window_height);
    if (!window.isValid()) {
        return 1;
    }

    if (had_saved_settings) {
        window.setWindowGeometry(
            saved_settings.ui.window_pos_x,
            saved_settings.ui.window_pos_y,
            saved_settings.ui.window_width,
            saved_settings.ui.window_height,
            saved_settings.ui.window_maximized);
        if (saved_settings.ui.window_fullscreen) {
            window.setFullscreen(true);
        }
    }

    ui::ToolpathViewport  viewport;
    viewport.setFollowToolLabel("Follow Torch");
    ui::DroAndJogPanel    dro_panel;
    ui::GCodeProgramPanel gcode_panel;
    ui::ScriptingPanel    scripting_panel;
    ui::WorkspacePanel    workspace_panel;
    ui::Mach3MigrationDialog mach3_wizard;

    core::CameraManager      camera_mgr;
    core::CameraStreamServer stream_server(camera_mgr);
    stream_server.setEStopCallback([&controller]() { controller.triggerEStop(); });

    std::mutex  remote_upload_mtx;
    std::string remote_loaded_path;
    bool        remote_viewport_fit_needed = false;

    stream_server.setLoadGCodeCallback([&](const std::string& filename, const std::string& content) -> core::LoadGCodeResult {
        const auto tel = controller.getTelemetry();
        if (tel.state == core::ControllerState::RunningGCode ||
            tel.state == core::ControllerState::DwellPierce ||
            tel.state == core::ControllerState::Probing ||
            tel.state == core::ControllerState::Homing)
        {
            return {false, "Machine is currently executing a motion job. Stop or wait before loading.", 0};
        }

        std::error_code ec;
        std::filesystem::path upload_dir = std::filesystem::temp_directory_path() / "BreakoutCNC" / "uploads";
        std::filesystem::create_directories(upload_dir, ec);
        std::filesystem::path disk_path = upload_dir / filename;
        {
            std::ofstream ofs(disk_path, std::ios::binary | std::ios::trunc);
            if (ofs) {
                ofs.write(content.data(), content.size());
            }
        }

        if (!controller.loadGCodeString(content)) {
            return {false, "Failed to parse G-code: no valid toolpath moves found", 0};
        }

        const size_t seg_count = controller.getProgram()->segments.size();
        {
            std::lock_guard<std::mutex> lock(remote_upload_mtx);
            remote_loaded_path = disk_path.string();
            remote_viewport_fit_needed = true;
        }
        return {true, "", seg_count};
    });

    ui::CameraPanel          camera_panel(camera_mgr, stream_server);

    controller.setScriptLogCallback([&scripting_panel](const std::string& msg, bool is_err) {
        scripting_panel.appendLog(msg, is_err);
    });

    saved_settings.machine.is_mill = false;

    // Apply loaded machine, camera & UI settings (BreakoutCut uses Monitoring role only)
    camera_mgr.applyConfig(saved_settings.machine.camera, saved_settings.ui.simulator_mode, false);
    stream_server.applyConfig(saved_settings.machine.camera);
    controller.applyConfig(saved_settings.machine);

    if (had_saved_settings) {
        if (saved_settings.ui.simulator_mode) {
            controller.setSimulatorMode(true);
        }
        controller.setDryRunMode(saved_settings.ui.dry_run_mode);
        controller.setFeedOverride(saved_settings.ui.feed_override_pct);
        controller.setPierceDelayOverride(saved_settings.ui.pierce_delay_override_s);

        viewport.setIsometric3D(saved_settings.ui.isometric_3d);
        viewport.setShowRapids(saved_settings.ui.show_rapids);
        viewport.setSimplifyPx(saved_settings.ui.toolpath_simplify_px);
        viewport.setShowPierces(saved_settings.ui.show_pierces);
        viewport.setFollowTorch(saved_settings.ui.follow_torch);

        dro_panel.setStepJogMode(saved_settings.ui.step_jog_mode);
        dro_panel.setStepSizeMm(saved_settings.ui.step_size_mm);
        dro_panel.setJogSpeedPct(saved_settings.ui.jog_speed_pct);

        // Restore persisted machine and work coordinates across sessions
        if (saved_settings.ui.has_saved_coordinates) {
            std::array<double, 4> m_pos = {
                saved_settings.ui.last_machine_pos_mm[0],
                saved_settings.ui.last_machine_pos_mm[1],
                saved_settings.ui.last_machine_pos_mm[2],
                saved_settings.ui.last_machine_pos_mm[3]
            };
            std::array<double, 4> w_off = {
                saved_settings.ui.last_work_offset_mm[0],
                saved_settings.ui.last_work_offset_mm[1],
                saved_settings.ui.last_work_offset_mm[2],
                saved_settings.ui.last_work_offset_mm[3]
            };
            controller.restoreSavedPositions(m_pos, w_off);
        }
    }

    // Always engage E-Stop by default on program start so machine is disarmed until operator clicks RESET E-STOP
    controller.triggerEStop();

    // Load the built-in demo program for the current mode (or the user's last loaded file below)
    static const std::string kPlasmaDemoName = "Built-in Demo: Gusset Bracket (140x90mm)";
    static const std::string kLaserDemoName  = "Built-in Demo: Compass Rose Coaster (90x90mm)";
    auto loadDemoProgram = [&](core::OperationMode mode, std::string& name) {
        const bool laser = (mode == core::OperationMode::Laser);
        if (controller.loadGCodeString(laser ? gcode::GCodeParser::sampleLaserCoasterGCode()
                                             : gcode::GCodeParser::samplePlasmaBracketGCode()))
        {
            name = laser ? kLaserDemoName : kPlasmaDemoName;
            return true;
        }
        return false;
    };
    std::string loaded_filename;
    loadDemoProgram(saved_settings.machine.operation_mode, loaded_filename);
    std::string persisted_file_path;
    if (!saved_settings.ui.last_gcode_file_path.empty() &&
        std::filesystem::exists(saved_settings.ui.last_gcode_file_path))
    {
        // Parsed in the background on the first frame, so the window appears right away; remembered even if
        // the app is closed before the parse finishes
        persisted_file_path = saved_settings.ui.last_gcode_file_path;
        gcode_panel.queueFileToLoad(persisted_file_path, false);
    }

    const auto initial_program = controller.getProgram();
    const auto& initial_parse = *initial_program;
    viewport.fitToBounds(initial_parse.bbox_min_mm, initial_parse.bbox_max_mm);

    float s_dro_user_h = saved_settings.ui.dro_jog_height;
    core::OperationMode current_op_mode = saved_settings.machine.operation_mode;
    controller.setOperationMode(current_op_mode);

    auto saveCurrentSettings = [&]() {
        if (smoke_test) return; // A smoke test never changes the settings next to the executable
        const auto tel = controller.getTelemetry();
        saved_settings.machine = controller.getConfig();
        window.getWindowGeometry(
            saved_settings.ui.window_pos_x,
            saved_settings.ui.window_pos_y,
            saved_settings.ui.window_width,
            saved_settings.ui.window_height,
            saved_settings.ui.window_maximized);
        saved_settings.ui.window_fullscreen = window.isFullscreen();

        saved_settings.ui.simulator_mode          = tel.simulator_mode;
        saved_settings.ui.dry_run_mode            = tel.dry_run_mode;
        saved_settings.ui.feed_override_pct       = tel.feed_override_pct;
        saved_settings.ui.pierce_delay_override_s = tel.pierce_delay_override_s;

        saved_settings.ui.isometric_3d            = viewport.isIsometric3D();
        saved_settings.ui.show_rapids             = viewport.getShowRapids();
        saved_settings.ui.toolpath_simplify_px = viewport.getSimplifyPx();
        saved_settings.ui.show_pierces            = viewport.getShowPierces();
        saved_settings.ui.follow_torch            = viewport.getFollowTorch();
        saved_settings.ui.minimal_layout          = saved_settings.ui.minimal_layout;

        saved_settings.ui.step_jog_mode           = dro_panel.isStepJogMode();
        saved_settings.ui.step_size_mm            = dro_panel.getStepSizeMm();
        saved_settings.ui.jog_speed_pct           = dro_panel.getJogSpeedPct();
        saved_settings.ui.dro_jog_height          = s_dro_user_h;
        saved_settings.ui.last_gcode_file_path    = persisted_file_path;
        saved_settings.ui.operation_mode          = static_cast<uint8_t>(current_op_mode);
        saved_settings.machine.operation_mode     = current_op_mode;
        saved_settings.ui.laser_power_override_pct = tel.laser_power_override_pct;

        saved_settings.ui.has_saved_coordinates = true;
        for (size_t i = 0; i < 4; ++i) {
            saved_settings.ui.last_machine_pos_mm[i] = tel.machine_pos_mm[i];
            // WCS offset without the tool length offset (that is restored separately)
            saved_settings.ui.last_work_offset_mm[i] = tel.workspace_offsets_mm[tel.active_workspace][i];
            saved_settings.ui.last_work_pos_mm[i]    = tel.work_pos_mm[i];
        }

        config::SettingsManager::saveToFile(settings_file, saved_settings);
    };

    bool minimal_layout = saved_settings.ui.minimal_layout;

    bool settings_dirty = false;
    double last_dirty_time = 0.0;
    auto markSettingsDirty = [&]() {
        settings_dirty = true;
        last_dirty_time = ImGui::GetTime();
    };

    dro_panel.setOnConfigChanged([&]() {
        saved_settings.machine = controller.getConfig();
        markSettingsDirty();
    });

    // Operation Mode (Plasma / Laser) selector for title bar
    auto updateWindowCaptionForMode = [&](core::OperationMode mode) {
        if (mode == core::OperationMode::Laser) {
            window.setTitle("BreakoutCut [Laser] - CNC Laser Cutter & Engraver (BreakoutCNC v" BREAKOUTCNC_VERSION_STRING ")");
            viewport.setFollowToolLabel("Follow Laser");
            gcode_panel.setMachineType(gcode::MachineType::Laser);
        } else {
            window.setTitle("BreakoutCut [Plasma] - CNC Plasma Cutter Controller (BreakoutCNC v" BREAKOUTCNC_VERSION_STRING ")");
            viewport.setFollowToolLabel("Follow Torch");
            gcode_panel.setMachineType(gcode::MachineType::Plasma);
        }
    };
    updateWindowCaptionForMode(current_op_mode);

    window.setOperationMode(&current_op_mode, [&](core::OperationMode new_mode) {
        current_op_mode = new_mode;
        saved_settings.ui.operation_mode = static_cast<uint8_t>(new_mode);
        saved_settings.machine.operation_mode = new_mode;
        controller.setOperationMode(new_mode);
        updateWindowCaptionForMode(new_mode);
        // Swap the built-in demo to the one for the new mode; a user's own program stays loaded (also one that
        // is still being parsed in the background)
        if (!gcode_panel.isLoading() && persisted_file_path.empty() &&
            (loaded_filename == kPlasmaDemoName || loaded_filename == kLaserDemoName) &&
            loadDemoProgram(new_mode, loaded_filename))
        {
            const auto prog = controller.getProgram();
            viewport.fitToBounds(prog->bbox_min_mm, prog->bbox_max_mm);
        }
        markSettingsDirty();
    });

    // Connect FULL / MINIMAL layout switcher to the custom title bar
    window.setMinimalLayout(&minimal_layout, [&](bool is_min) {
        saved_settings.ui.minimal_layout = is_min;
        markSettingsDirty();
    });

    // Auto-save viewport settings when toggled
    viewport.setOnViewChanged([&]() {
        saved_settings.ui.isometric_3d = viewport.isIsometric3D();
        saved_settings.ui.show_rapids  = viewport.getShowRapids();
        saved_settings.ui.toolpath_simplify_px = viewport.getSimplifyPx();
        saved_settings.ui.show_pierces = viewport.getShowPierces();
        saved_settings.ui.follow_torch = viewport.getFollowTorch();
        markSettingsDirty();
    });

    // A G-code file given on the command line ("Open with" / double-click) or dropped onto the window
    if (!arg_file.empty()) {
        gcode_panel.queueFileToLoad(arg_file);
    }
    window.setOnFileDropped([&](const std::string& path) { gcode_panel.queueFileToLoad(path); });

    bool show_exit_confirm_modal = false;
    bool force_close = false;
    window.setCloseInterceptor(
        [&]() {
            if (force_close) return true;
            const auto tel = controller.getTelemetry();
            bool is_running = (tel.state == core::ControllerState::RunningGCode ||
                               tel.state == core::ControllerState::DwellPierce ||
                               tel.state == core::ControllerState::Jogging);
            return !is_running;
        },
        [&]() {
            show_exit_confirm_modal = true;
        }
    );

    window.setOnCloseCallback([&]() {
        controller.triggerEStop();
        saveCurrentSettings();
    });

    if (smoke_test) ImGui::GetIO().IniFilename = nullptr; // No imgui.ini either
    int  smoke_frames = 0;
    bool smoke_failed = false;

    int exit_code = window.runLoop([&]() {
        if (smoke_test) {
            ++smoke_frames;
            if (smoke_frames == 10 && controller.resetFromEStop()) controller.startJob();
            if (smoke_frames == 240) { // ~4 s at 60 FPS
                const auto t = controller.getTelemetry();
                smoke_failed = (t.active_segment_index == 0);
                std::printf("Smoke test %s: %d frames, demo job at segment %zu\n", smoke_failed ? "FAILED" : "passed",
                            smoke_frames, t.active_segment_index);
                std::fflush(stdout);
                window.closeWindow();
            }
        }
        // Global keyboard E-Stop: ESC always halts the machine, whatever has keyboard focus
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            controller.triggerEStop();
        }

        // Process any pending remote G-Code upload from the web server
        {
            std::lock_guard<std::mutex> lock(remote_upload_mtx);
            if (!remote_loaded_path.empty()) {
                loaded_filename     = remote_loaded_path;
                persisted_file_path = remote_loaded_path;
                saved_settings.ui.last_gcode_file_path = persisted_file_path;
                markSettingsDirty();
                if (remote_viewport_fit_needed) {
                    const auto pr_ptr = controller.getProgram();
                    const auto& pr = *pr_ptr;
                    viewport.fitToBounds(pr.bbox_min_mm, pr.bbox_max_mm);
                    remote_viewport_fit_needed = false;
                }
                remote_loaded_path.clear();
            }
        }

        const auto telemetry   = controller.getTelemetry();
        const auto program_snapshot = controller.getProgram(); // Held for the whole frame
        const auto& parse_res  = *program_snapshot;
        const auto& segments   = program_snapshot->segments;

        camera_mgr.updateLiveTelemetry(telemetry, controller.getConfig());
        stream_server.updateTelemetry(telemetry, controller.getConfig().linear_units,
                                      current_op_mode == core::OperationMode::Laser ? "BreakoutCut [Laser]" : "BreakoutCut [Plasma]",
                                      loaded_filename);

        // Automatic debounced settings persistence on change (within 500ms of any setting modification)
        if (settings_dirty && (ImGui::GetTime() - last_dirty_time > 0.5)) {
            saveCurrentSettings();
            settings_dirty = false;
        }

        saved_settings.ui.minimal_layout = minimal_layout;

        // Note: Top header rows eliminated! Viewport and right column go directly up against the title bar.
        const float spacing_x = ImGui::GetStyle().ItemSpacing.x;
        const float spacing_y = ImGui::GetStyle().ItemSpacing.y;
        const float workspace_h = ImGui::GetContentRegionAvail().y - 40.0f - spacing_y;

        const float right_col_w = minimal_layout ? 0.0f   : 450.0f;
        const float left_col_w  = minimal_layout ? 325.0f : 450.0f;

        // 1. Left Column: Sticky DRO/Jog Panel & Bottom Controller State Badge / E-Stop Card
        // Sized to exactly workspace_h with NoScrollbar & NoScrollWithMouse so it NEVER scrolls
        ImGui::BeginChild("##PlasmaLeftCol", ImVec2(left_col_w, workspace_h), false,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        {
            // State badge header and CYCLE START/E-STOP row (110px)
            const float estop_card_h = 110.0f;
            const float dro_h = std::max(100.0f, workspace_h - estop_card_h - spacing_y - 2.0f);

            // DRO and Manual Jog Panel (top of left panel)
            ImGui::BeginChild("##PlasmaDroJogFixed", ImVec2(0.0f, dro_h), true,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            dro_panel.render(controller, telemetry, false, dro_h); // 3 Axes for Plasma (X/Y/Z)
            ImGui::EndChild();

            // Pin Controller State Badge, Cycle Control, and Emergency Stop / Reset Card to the bottom
            ImGui::SetCursorPosY(workspace_h - estop_card_h);
            ui::IoStatusBar::renderStatusAndEStopCard(controller, telemetry, estop_card_h, &gcode_panel);
        }
        ImGui::EndChild(); // ##PlasmaLeftCol

        ImGui::SameLine();

        const float center_col_w = minimal_layout
                                       ? ImGui::GetContentRegionAvail().x
                                       : std::max(200.0f, ImGui::GetContentRegionAvail().x - right_col_w - spacing_x);

        static bool s_plasma_lua_open = saved_settings.ui.header_lua_open;

        // 2. Center Column: Toolpath Viewport + G-Code Panel + Lua Scripting Deck (Full View Only)
        ImGui::BeginChild("##PlasmaCenterCol", ImVec2(center_col_w, workspace_h), false);
        {
            const float lua_reserved_h = minimal_layout
                                             ? 0.0f
                                             : (s_plasma_lua_open ? 260.0f : (ImGui::GetFrameHeightWithSpacing() + 6.0f));

            // Compute exact snug height for G-Code & MDI card so there is zero unused space below MDI
            // In Minimal view, user requested exactly 8 lines of G-Code visible
            float gcode_h;
            if (minimal_layout) {
                gcode_h = gcode_panel.calculateDesiredHeight(0.0f, false, 8);
            } else {
                const float target_gcode_ratio = s_plasma_lua_open ? 0.28f : 0.38f;
                const float max_gcode_h = std::clamp(workspace_h * target_gcode_ratio, 160.0f, workspace_h - 220.0f);
                gcode_h = gcode_panel.calculateDesiredHeight(max_gcode_h, false);
            }

            // Viewport takes all remaining vertical space up to the top title bar
            const float viewport_h = minimal_layout
                                         ? std::max(120.0f, workspace_h - gcode_h - spacing_y)
                                         : std::max(120.0f, workspace_h - gcode_h - lua_reserved_h - spacing_y * 2.0f);

            ImGui::BeginChild("##PlasmaViewport", ImVec2(0.0f, viewport_h), true);
            viewport.render(segments, telemetry, parse_res.bbox_min_mm, parse_res.bbox_max_mm, &controller);
            ImGui::EndChild();

            // G-Code Program, Load File... & MDI Card
            ImGui::BeginChild("##PlasmaGCodeCard", ImVec2(0.0f, gcode_h), true);
            gcode_panel.render(
                controller,
                telemetry,
                loaded_filename,
                parse_res.pierce_count,
                parse_res.total_cut_length_mm,
                [&](const std::string& path, const std::string& /*content*/) {
                    // Called once the panel has loaded the program (parsed on a background thread)
                    loaded_filename     = path;
                    persisted_file_path = path;
                    const auto pr_ptr = controller.getProgram();
                    const auto& pr = *pr_ptr;
                    viewport.fitToBounds(pr.bbox_min_mm, pr.bbox_max_mm);
                    markSettingsDirty();
                });
            ImGui::EndChild();

            // In MINIMAL view, Lua Scripting is completely removed
            if (!minimal_layout) {
                const float lua_h = s_plasma_lua_open ? 0.0f : lua_reserved_h;
                ImGui::BeginChild("##PlasmaLuaCard", ImVec2(0.0f, lua_h), true, s_plasma_lua_open ? 0 : ImGuiWindowFlags_NoScrollbar);
                ImGui::SetNextItemAllowOverlap();
                ImGui::SetNextItemOpen(saved_settings.ui.header_lua_open, ImGuiCond_Once);
                bool lua_hdr_open = ImGui::CollapsingHeader("Lua Scripting & User Macros", ImGuiTreeNodeFlags_AllowOverlap);

                // Information sign icon button on the right side of the header bar
                const ImVec2 hdr_min = ImGui::GetItemRectMin();
                const ImVec2 hdr_max = ImGui::GetItemRectMax();
                const float icon_sz = 20.0f;
                const ImVec2 icon_pos(hdr_max.x - icon_sz - 12.0f, hdr_min.y + (hdr_max.y - hdr_min.y - icon_sz) * 0.5f);
                if (ui::ScriptingPanel::renderHelpIconButton("##PlasmaLuaHelpBtn", icon_pos, icon_sz)) {
                    scripting_panel.openHelpModal();
                    lua_hdr_open = s_plasma_lua_open; // Prevent header from collapsing when info icon is clicked
                }

                if (lua_hdr_open != saved_settings.ui.header_lua_open) {
                    saved_settings.ui.header_lua_open = lua_hdr_open;
                    markSettingsDirty();
                }
                if (lua_hdr_open) {
                    s_plasma_lua_open = true;
                    core::MachineConfig cur_cfg = controller.getConfig();
                    scripting_panel.render(controller, cur_cfg, [&]() {
                        saved_settings.machine = cur_cfg;
                        markSettingsDirty();
                    });
                } else {
                    s_plasma_lua_open = false;
                    ImGui::Dummy(ImVec2(0.0f, 0.0f));
                }
                ImGui::EndChild();
            }
        }
        ImGui::EndChild(); // ##PlasmaCenterCol

        // 3. Right Column: Settings & Configuration Deck (Completely hidden in MINIMAL view)
        if (!minimal_layout) {
            ImGui::SameLine();
            ImGui::BeginChild("##PlasmaRightCol", ImVec2(right_col_w, workspace_h), false, ImGuiWindowFlags_NoScrollbar);
            {
                ImGui::BeginChild("##PlasmaSettingsScroll", ImVec2(0.0f, 0.0f), true);
                {
                    // -----------------------------------------------------------------
                    // 1. USB MOTION CONTROLLER & CONNECTION (RNR ECO MOTION 2.0)
                    // -----------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_usb_open, ImGuiCond_Once);
                    bool usb_hdr_open = ImGui::CollapsingHeader("USB Motion Controller & Connection (RNR Eco Motion 2.0)");
                    if (usb_hdr_open != saved_settings.ui.header_usb_open) {
                        saved_settings.ui.header_usb_open = usb_hdr_open;
                        markSettingsDirty();
                    }
                    if (usb_hdr_open) {
                        ui::IoStatusBar::renderUsbConnectionSection(controller, telemetry, [&]() {
                            markSettingsDirty();
                        });
                    }
                    ImGui::Spacing();

                    if (current_op_mode == core::OperationMode::Plasma) {
                    // -------------------------------------------------------------
                    // 2. PLASMA TORCH & PROCESS CONTROL
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_spindle_open, ImGuiCond_Once);
                    bool torch_hdr_open = ImGui::CollapsingHeader("Plasma Torch & Process Control");
                    if (torch_hdr_open != saved_settings.ui.header_spindle_open) {
                        saved_settings.ui.header_spindle_open = torch_hdr_open;
                        markSettingsDirty();
                    }
                    if (torch_hdr_open) {
                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                        // Card: Torch Firing & Interlocks
                        ImGui::BeginChild("##TorchFireCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "TORCH FIRING & PROCESS INTERLOCKS");
                            ImGui::Spacing();

                            const bool torch_on = telemetry.torch_or_spindle_on;
                            ImVec4 torch_col = torch_on ? ImVec4(1.0f, 0.38f, 0.05f, 1.0f) : ImVec4(0.20f, 0.25f, 0.32f, 1.0f);
                            ImGui::PushStyleColor(ImGuiCol_Button, torch_col);
                            const char* torch_lbl = torch_on ? "TORCH FIRING (M3 ON) - CLICK TO EXTINGUISH"
                                                             : "FIRE TORCH MANUALLY (M3 / OUT1)";
                            if (ImGui::Button(torch_lbl, ImVec2(-1.0f, 38.0f))) {
                                controller.setTorchOrSpindle(!torch_on);
                            }
                            ImGui::PopStyleColor();

                            bool dry_run = telemetry.dry_run_mode;
                            if (ImGui::Checkbox("Dry-Run Mode (Trace Path with Torch Disabled)", &dry_run)) {
                                controller.setDryRunMode(dry_run);
                                markSettingsDirty();
                            }

                            if (ImGui::Button("Floating-Head Z Touch-Off (G31 Probe)", ImVec2(-1.0f, 30.0f))) {
                                controller.probeAxis(core::Axis::Z, -30.0, 400.0);
                            }
                        }
                        ImGui::EndChild();

                        ImGui::Spacing();

                        // Card: Feed Rate & Pierce Delay
                        ImGui::BeginChild("##TorchFeedCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "FEED OVERRIDE & SPEED GAUGES");
                            ImGui::Spacing();

                            static double s_display_act_feed = 0.0;
                            static double s_last_feed_sample_time = 0.0;
                            double cur_time = ImGui::GetTime();
                            const bool is_motion_state = (telemetry.state == core::ControllerState::RunningGCode ||
                                                          telemetry.state == core::ControllerState::Jogging ||
                                                          telemetry.state == core::ControllerState::Homing ||
                                                          telemetry.state == core::ControllerState::Probing);
                            bool is_moving = is_motion_state && (telemetry.current_feed_mm_min > 0.5);
                            if (!is_moving) {
                                s_display_act_feed = 0.0;
                                s_last_feed_sample_time = cur_time;
                            } else if (cur_time - s_last_feed_sample_time >= 0.5 || s_display_act_feed == 0.0) {
                                s_display_act_feed = telemetry.current_feed_mm_min;
                                s_last_feed_sample_time = cur_time;
                            }
                            double act_feed_mm_min = is_moving ? s_display_act_feed : 0.0;

                            const float feed_inner_w = ImGui::GetContentRegionAvail().x;
                            const float left_dro_w   = std::max(160.0f, std::min(220.0f, feed_inner_w * 0.44f));
                            const float right_btn_w  = feed_inner_w - left_dro_w - spacing_x;
                            const float dro_box_h    = 58.0f;

                            // Left DRO Box
                            ImVec2 box_p0 = ImGui::GetCursorScreenPos();
                            ImVec2 box_p1(box_p0.x + left_dro_w, box_p0.y + dro_box_h);
                            ImDrawList* dl = ImGui::GetWindowDrawList();
                            dl->AddRectFilled(box_p0, box_p1, IM_COL32(8, 11, 15, 255), 4.0f);
                            dl->AddRect(box_p0, box_p1, is_moving ? IM_COL32(0, 200, 120, 220) : IM_COL32(45, 62, 84, 220), 4.0f, 0, 1.2f);

                            ImFont* gfont = ui::AppWindow::getGCodeFont();
                            float fsz_lbl = 11.0f;
                            float fsz_val = 14.0f;

                            const bool is_imperial_feed = (controller.getConfig().linear_units == core::LinearUnits::Imperial);
                            const double feed_scale = is_imperial_feed ? (1.0 / 25.4) : 1.0;
                            const char* feed_unit_lbl = is_imperial_feed ? "IPM" : "mm/m";
                            const char* feed_sec_lbl = is_imperial_feed ? "in/s" : "mm/s";

                            float line1_y = box_p0.y + 7.0f;
                            dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line1_y + 2.0f),
                                        is_moving ? IM_COL32(140, 245, 180, 255) : IM_COL32(110, 135, 160, 230), "ACT FEED");
                            char act_feed_str[32];
                            std::snprintf(act_feed_str, sizeof(act_feed_str), "%4.0f %s", act_feed_mm_min * feed_scale, feed_unit_lbl);
                            ImVec2 act_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, act_feed_str) : ImGui::CalcTextSize(act_feed_str);
                            dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - act_sz.x - 7.0f, line1_y),
                                        is_moving ? IM_COL32(0, 240, 150, 255) : IM_COL32(115, 155, 195, 230), act_feed_str);

                            float line2_y = box_p0.y + 31.0f;
                            dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line2_y + 2.0f), IM_COL32(100, 120, 145, 200), "CMD (F)");
                            char cmd_feed_str[32];
                            std::snprintf(cmd_feed_str, sizeof(cmd_feed_str), "%4.0f %s", telemetry.commanded_feed_mm_min * feed_scale, feed_unit_lbl);
                            ImVec2 cmd_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, cmd_feed_str) : ImGui::CalcTextSize(cmd_feed_str);
                            dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - cmd_sz.x - 7.0f, line2_y), IM_COL32(80, 175, 245, 220), cmd_feed_str);

                            ImGui::Dummy(ImVec2(left_dro_w, dro_box_h));
                            if (ImGui::IsItemHovered()) {
                                ImGui::BeginTooltip();
                                ImGui::Text("Actual Tool Speed: %.0f %s (%.1f %s)",
                                            act_feed_mm_min * feed_scale, feed_unit_lbl,
                                            (act_feed_mm_min / 60.0) * feed_scale, feed_sec_lbl);
                                ImGui::Text("Commanded Feed (F): %.0f %s (%.1f %s)",
                                            telemetry.commanded_feed_mm_min * feed_scale, feed_unit_lbl,
                                            (telemetry.commanded_feed_mm_min / 60.0) * feed_scale, feed_sec_lbl);
                                ImGui::EndTooltip();
                            }

                            ImGui::SameLine();

                            // Right controls: Feed Override Slider & Reset
                            ImGui::BeginGroup();
                            {
                                static float s_drag_feed_ov = 100.0f;
                                static bool s_is_dragging_feed = false;
                                if (!s_is_dragging_feed) s_drag_feed_ov = telemetry.feed_override_pct;

                                const bool feed_ov = (std::abs(telemetry.feed_override_pct - 100.0f) >= 0.5f);
                                if (feed_ov) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.40f, 0.22f, 0.05f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.70f, 0.20f, 1.0f));
                                }
                                if (ImGui::Button("Reset 100%##Feed", ImVec2(right_btn_w, 24.0f))) {
                                    controller.setFeedOverride(100.0f);
                                    s_drag_feed_ov = 100.0f;
                                    markSettingsDirty();
                                }
                                if (feed_ov) ImGui::PopStyleColor(2);

                                ImGui::SetNextItemWidth(right_btn_w);
                                if (ImGui::SliderFloat("##FeedSlider", &s_drag_feed_ov, 20.0f, 200.0f, "Feed: %.0f%%")) {
                                    controller.setFeedOverride(s_drag_feed_ov);
                                    markSettingsDirty();
                                }
                            }
                            ImGui::EndGroup();

                            ImGui::Spacing();

                            float pierce_ov = telemetry.pierce_delay_override_s;
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderFloat("Pierce Delay (s)", &pierce_ov, 0.0f, 3.0f, "%.2f s")) {
                                controller.setPierceDelayOverride(pierce_ov);
                                markSettingsDirty();
                            }
                        }
                        ImGui::EndChild();

                        ImGui::PopStyleVar(4);
                        ImGui::PopStyleColor(2);
                    }

                    ImGui::Spacing();

                    // -------------------------------------------------------------
                    // 3. TORCH HEIGHT CONTROL (THC)
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_vfd_open, ImGuiCond_Once);
                    bool thc_hdr_open = ImGui::CollapsingHeader("Torch Height Control (THC)");
                    if (thc_hdr_open != saved_settings.ui.header_vfd_open) {
                        saved_settings.ui.header_vfd_open = thc_hdr_open;
                        markSettingsDirty();
                    }
                    if (thc_hdr_open) {
                        core::MachineConfig cfg = controller.getConfig();
                        bool thc_changed = false;

                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                        // Card: THC Status & LEDs
                        ImGui::BeginChild("##ThcStatusCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "THC REAL-TIME STATUS & INTERLOCKS");
                            ImGui::Spacing();

                            if (ImGui::Checkbox("Enable Real-Time THC Z Tracking", &cfg.thc.enabled)) thc_changed = true;
                            ImGui::SameLine();
                            if (ImGui::Checkbox("Wait Arc OK", &cfg.thc.wait_for_arc_ok)) thc_changed = true;

                            ImGui::BeginChild("##ThcStatusBox", ImVec2(0.0f, telemetry.simulator_mode ? 64.0f : 36.0f), true, ImGuiWindowFlags_NoScrollbar);
                            ImGui::TextColored(telemetry.thc_arc_ok_active ? ImVec4(0.15f, 0.95f, 0.45f, 1.0f) : ImVec4(0.45f, 0.50f, 0.56f, 1.0f), "[ARC OK]");
                            ImGui::SameLine();
                            ImGui::TextColored(telemetry.thc_up_active ? ImVec4(0.00f, 0.88f, 1.00f, 1.0f) : ImVec4(0.45f, 0.50f, 0.56f, 1.0f), "[THC UP]");
                            ImGui::SameLine();
                            ImGui::TextColored(telemetry.thc_down_active ? ImVec4(1.00f, 0.65f, 0.10f, 1.0f) : ImVec4(0.45f, 0.50f, 0.56f, 1.0f), "[THC DN]");
                            ImGui::SameLine();
                            ImGui::TextColored(telemetry.thc_anti_dive_locked ? ImVec4(1.00f, 0.82f, 0.20f, 1.0f) : ImVec4(0.45f, 0.50f, 0.56f, 1.0f), "[ANTI-DIVE]");
                            ImGui::SameLine();
                            ImGui::TextColored(ImVec4(0.75f, 0.88f, 1.0f, 1.0f), "dZ:%+5.2fmm", telemetry.thc_z_offset_mm);

                            if (telemetry.simulator_mode) {
                                ImGui::TextDisabled("Sim Test:");
                                ImGui::SameLine();
                                if (cfg.thc.arc_ok_pin >= 1) {
                                    bool arc_sim = ((telemetry.simulated_inputs_bitmask >> (cfg.thc.arc_ok_pin - 1)) & 1U) != 0;
                                    if (ImGui::SmallButton(arc_sim ? "ArcOK:ON" : "ArcOK:OFF")) {
                                        controller.setSimulatedInputPin(cfg.thc.arc_ok_pin, !arc_sim);
                                    }
                                    ImGui::SameLine();
                                }
                                if (cfg.thc.thc_up_pin >= 1) {
                                    ImGui::SmallButton("Hold THC UP");
                                    if (ImGui::IsItemActivated()) controller.setSimulatedInputPin(cfg.thc.thc_up_pin, true);
                                    if (ImGui::IsItemDeactivated()) controller.setSimulatedInputPin(cfg.thc.thc_up_pin, false);
                                    ImGui::SameLine();
                                }
                                if (cfg.thc.thc_down_pin >= 1) {
                                    ImGui::SmallButton("Hold THC DN");
                                    if (ImGui::IsItemActivated()) controller.setSimulatedInputPin(cfg.thc.thc_down_pin, true);
                                    if (ImGui::IsItemDeactivated()) controller.setSimulatedInputPin(cfg.thc.thc_down_pin, false);
                                }
                            }
                            ImGui::EndChild();
                        }
                        ImGui::EndChild();

                        ImGui::Spacing();

                        // Card: THC Parameters
                        ImGui::BeginChild("##ThcParamsCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "THC SPEEDS & VOLTAGE TRACKING PARAMETERS");
                            ImGui::Spacing();

                            float spd = static_cast<float>(cfg.thc.adjust_speed_mm_s);
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderFloat("THC Z Speed (mm/s)", &spd, 0.5f, 20.0f, "%.1f mm/s")) {
                                cfg.thc.adjust_speed_mm_s = spd;
                                thc_changed = true;
                            }

                            float delay_s = static_cast<float>(cfg.thc.delay_after_pierce_s);
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderFloat("Pierce-to-THC Delay (s)", &delay_s, 0.0f, 3.0f, "%.2f s")) {
                                cfg.thc.delay_after_pierce_s = delay_s;
                                thc_changed = true;
                            }

                            float anti_dive = cfg.thc.min_xy_feed_pct;
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderFloat("Anti-Dive Corner Lock %", &anti_dive, 10.0f, 95.0f, "%.0f%%")) {
                                cfg.thc.min_xy_feed_pct = anti_dive;
                                thc_changed = true;
                            }

                            float z_min = static_cast<float>(cfg.thc.z_min_mm);
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderFloat("Min THC Z (mm)", &z_min, -30.0f, 5.0f, "%.1f mm")) {
                                cfg.thc.z_min_mm = z_min;
                                thc_changed = true;
                            }

                            float z_max = static_cast<float>(cfg.thc.z_max_mm);
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderFloat("Max THC Z (mm)", &z_max, 2.0f, 50.0f, "%.1f mm")) {
                                cfg.thc.z_max_mm = z_max;
                                thc_changed = true;
                            }
                        }
                        ImGui::EndChild();

                        ImGui::Spacing();

                        // Card: THC Hardware Input Pins
                        ImGui::BeginChild("##ThcPinsCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "THC HARDWARE INPUT PINS & POLARITY");
                            ImGui::Spacing();

                            int arc_pin = cfg.thc.arc_ok_pin;
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderInt("Arc OK Pin (IN#)", &arc_pin, 0, 12)) {
                                cfg.thc.arc_ok_pin = static_cast<uint8_t>(arc_pin);
                                thc_changed = true;
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("Active Low##Arc", &cfg.thc.arc_ok_invert)) thc_changed = true;
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Active Low: Checked = Arc OK / Transfer signal pulls to COM/GND when arc establishes.\nUnchecked = Active High.");
                            }

                            int up_pin = cfg.thc.thc_up_pin;
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderInt("THC UP Pin (IN#)", &up_pin, 0, 12)) {
                                cfg.thc.thc_up_pin = static_cast<uint8_t>(up_pin);
                                thc_changed = true;
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("Active Low##Up", &cfg.thc.thc_up_invert)) thc_changed = true;
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Active Low: Checked = THC UP signal pulls to COM/GND.\nUnchecked = Active High.");
                            }

                            int dn_pin = cfg.thc.thc_down_pin;
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderInt("THC DN Pin (IN#)", &dn_pin, 0, 12)) {
                                cfg.thc.thc_down_pin = static_cast<uint8_t>(dn_pin);
                                thc_changed = true;
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("Active Low##Dn", &cfg.thc.thc_down_invert)) thc_changed = true;
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Active Low: Checked = THC DN signal pulls to COM/GND.\nUnchecked = Active High.");
                            }
                        }
                        ImGui::EndChild();

                        ImGui::PopStyleVar(4);
                        ImGui::PopStyleColor(2);

                        if (thc_changed) {
                            controller.applyConfig(cfg);
                            markSettingsDirty();
                        }
                    }

                    ImGui::Spacing();

                    // -------------------------------------------------------------
                    // 4. OHMIC SENSE & FLOATING HEAD (IHS / PROBE)
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_probe_settings_open, ImGuiCond_Once);
                    bool probe_hdr_open = ImGui::CollapsingHeader("Ohmic Sense & Floating Head (IHS / Probe)");
                    if (probe_hdr_open != saved_settings.ui.header_probe_settings_open) {
                        saved_settings.ui.header_probe_settings_open = probe_hdr_open;
                        markSettingsDirty();
                    }
                    if (probe_hdr_open) {
                        core::MachineConfig cfg = controller.getConfig();
                        bool probe_changed = false;

                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                        // Card: Probe Pin & Status
                        ImGui::BeginChild("##IhsPinCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "IHS HARDWARE INPUT PIN & POLARITY");
                            ImGui::Spacing();

                            ImGui::BeginChild("##PlasmaProbeStatusBox", ImVec2(0.0f, 32.0f), true, ImGuiWindowFlags_NoScrollbar);
                            ImGui::TextColored(
                                telemetry.probe_contact_active ? ImVec4(0.15f, 0.95f, 0.45f, 1.0f) : ImVec4(0.55f, 0.62f, 0.70f, 1.0f),
                                telemetry.probe_contact_active ? "[IHS PROBE: CONTACT TRIGGERED]" : "[IHS PROBE: OPEN / READY]");
                            if (telemetry.simulator_mode && cfg.probe.probe_pin >= 1) {
                                ImGui::SameLine(ImGui::GetWindowWidth() - 125.0f);
                                bool sim_p = ((telemetry.simulated_inputs_bitmask >> (cfg.probe.probe_pin - 1)) & 1U) != 0;
                                if (ImGui::SmallButton(sim_p ? "Release Probe" : "Sim Touch")) {
                                    controller.setSimulatedInputPin(cfg.probe.probe_pin, !sim_p);
                                }
                            }
                            ImGui::EndChild();

                            int p_pin = cfg.probe.probe_pin;
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderInt("IHS Probe Pin (0=Off, IN1..12)", &p_pin, 0, 12)) {
                                cfg.probe.probe_pin = static_cast<uint8_t>(p_pin);
                                probe_changed = true;
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("Active Low (Grounded to COM)##PlasmaProbe", &cfg.probe.probe_invert)) {
                                probe_changed = true;
                            }
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Active Low: Checked = Ohmic shield or Floating Head microswitch pulls to COM/GND when nozzle touches plate.\nUnchecked = Active High.");
                            }
                        }
                        ImGui::EndChild();

                        ImGui::Spacing();

                        // Card: Probe Dimensions & Speeds
                        ImGui::BeginChild("##IhsParamsCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "MECHANICAL SWITCH & SEARCH PARAMETERS");
                            ImGui::Spacing();

                            float switch_offset = static_cast<float>(cfg.probe.plate_thickness_mm);
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::InputFloat("Floating Head Switch Offset (mm)", &switch_offset, 0.1f, 1.0f, "%.3f mm")) {
                                cfg.probe.plate_thickness_mm = std::max(0.0f, switch_offset);
                                probe_changed = true;
                            }
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Mechanical switch travel distance before switch trips (subtracted during IHS zeroing). Set to 0.0 for Ohmic contact.");
                            }

                            float search_feed = static_cast<float>(cfg.probe.fast_feed_mm_min);
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::InputFloat("IHS Search Speed (mm/min)", &search_feed, 50.0f, 200.0f, "%.0f")) {
                                cfg.probe.fast_feed_mm_min = std::max(20.0f, search_feed);
                                probe_changed = true;
                            }
                        }
                        ImGui::EndChild();

                        ImGui::PopStyleVar(4);
                        ImGui::PopStyleColor(2);

                        if (probe_changed) {
                            controller.applyConfig(cfg);
                            markSettingsDirty();
                        }
                    }
                    } else {
                        // -------------------------------------------------------------
                        // 2. LASER SOURCE & POWER CONTROL (PWM / M4)
                        // -------------------------------------------------------------
                        ImGui::SetNextItemOpen(saved_settings.ui.header_laser_power_open, ImGuiCond_Once);
                        bool laser_power_hdr_open = ImGui::CollapsingHeader("Laser Source & Power Control (PWM / M4)");
                        if (laser_power_hdr_open != saved_settings.ui.header_laser_power_open) {
                            saved_settings.ui.header_laser_power_open = laser_power_hdr_open;
                            markSettingsDirty();
                        }
                        if (laser_power_hdr_open) {
                            core::MachineConfig cfg = controller.getConfig();
                            bool changed = false;

                            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                            ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                            // Card: Laser Source Type Dropdown & Mode
                            ImGui::BeginChild("##LaserSourceCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(1.00f, 0.30f, 0.50f, 1.0f), "LASER SOURCE TYPE & PRESETS");
                                ImGui::Spacing();

                                const char* source_names[] = {
                                    "Diode Laser (450nm Blue / IR)",
                                    "CO2 Glass Tube (DC 10.6um)",
                                    "RF CO2 Laser (Metal Tube)",
                                    "Fiber Laser (1064nm)",
                                    "Custom / Manual"
                                };
                                int cur_src = static_cast<int>(cfg.laser.source_type);
                                ImGui::SetNextItemWidth(-1.0f);
                                if (ImGui::Combo("##LaserSourceCombo", &cur_src, source_names, IM_ARRAYSIZE(source_names))) {
                                    cfg.laser.source_type = static_cast<core::LaserSourceType>(cur_src);
                                    if (cfg.laser.source_type == core::LaserSourceType::Diode) {
                                        cfg.laser.min_duty_pct = 0.0f;
                                        cfg.laser.max_duty_pct = 100.0f;
                                        cfg.laser.pwm_frequency_khz = 1.0f;
                                        cfg.laser.framing_mode = core::LaserFramingMode::LowPowerBeam;
                                        cfg.laser.chiller_flow_pin = 0;
                                    } else if (cfg.laser.source_type == core::LaserSourceType::Co2Glass) {
                                        cfg.laser.min_duty_pct = 10.0f;
                                        cfg.laser.max_duty_pct = 85.0f;
                                        cfg.laser.pwm_frequency_khz = 2.0f;
                                        cfg.laser.framing_mode = core::LaserFramingMode::RedDotPointer;
                                        cfg.laser.chiller_flow_pin = 2;
                                    } else if (cfg.laser.source_type == core::LaserSourceType::RfCo2) {
                                        cfg.laser.min_duty_pct = 0.0f;
                                        cfg.laser.max_duty_pct = 100.0f;
                                        cfg.laser.pwm_frequency_khz = 5.0f;
                                        cfg.laser.framing_mode = core::LaserFramingMode::RedDotPointer;
                                        cfg.laser.chiller_flow_pin = 2;
                                    } else if (cfg.laser.source_type == core::LaserSourceType::Fiber) {
                                        cfg.laser.min_duty_pct = 0.0f;
                                        cfg.laser.max_duty_pct = 100.0f;
                                        cfg.laser.pwm_frequency_khz = 5.0f;
                                        cfg.laser.framing_mode = core::LaserFramingMode::RedDotPointer;
                                        cfg.laser.chiller_flow_pin = 2;
                                    }
                                    changed = true;
                                }

                                ImGui::Spacing();
                                if (ImGui::Checkbox("Dynamic Laser Power (M4) - Velocity Proportional", &cfg.laser.dynamic_power_m4)) changed = true;
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("When active, scales laser PWM duty proportionally with corner deceleration (v / v_target) to prevent burning corners.");
                                }

                                if (ImGui::Checkbox("Ignore G-Code Z Moves (Lock Focal Plane)", &cfg.laser.ignore_z_motion)) changed = true;
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Locks Z-axis during cutting moves. Prevents unnecessary retracts or crashes from plasma G-code files.");
                                }
                            }
                            ImGui::EndChild();

                            ImGui::Spacing();

                            // Card: Laser Power Gauge & Override Slider
                            ImGui::BeginChild("##LaserPowerOverrideCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(1.00f, 0.30f, 0.50f, 1.0f), "LIVE LASER POWER & OVERRIDE");
                                ImGui::Spacing();

                                const bool laser_firing = (telemetry.outputs_bitmask & core::OutputBit::OUT1_TorchOrSpindle) != 0;
                                const float live_pwr = telemetry.live_laser_power_pct;
                                const uint16_t pwm_duty = telemetry.spindle_pwm_duty;

                                const float inner_w = ImGui::GetContentRegionAvail().x;
                                const float dro_w   = std::max(160.0f, std::min(220.0f, inner_w * 0.44f));
                                const float right_w = inner_w - dro_w - spacing_x;
                                const float dro_h   = 58.0f;

                                ImVec2 box_p0 = ImGui::GetCursorScreenPos();
                                ImVec2 box_p1(box_p0.x + dro_w, box_p0.y + dro_h);
                                ImDrawList* dl = ImGui::GetWindowDrawList();
                                dl->AddRectFilled(box_p0, box_p1, IM_COL32(10, 10, 14, 255), 4.0f);
                                dl->AddRect(box_p0, box_p1, laser_firing ? IM_COL32(240, 40, 80, 240) : IM_COL32(45, 55, 75, 220), 4.0f, 0, 1.2f);

                                ImFont* gfont = ui::AppWindow::getGCodeFont();
                                float fsz_lbl = 11.0f;
                                float fsz_val = 14.0f;

                                float line1_y = box_p0.y + 7.0f;
                                dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line1_y + 2.0f),
                                            laser_firing ? IM_COL32(255, 120, 150, 255) : IM_COL32(110, 135, 160, 230), "LASER POWER");
                                char pwr_str[32];
                                std::snprintf(pwr_str, sizeof(pwr_str), "%5.1f%%", live_pwr);
                                ImVec2 pwr_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, pwr_str) : ImGui::CalcTextSize(pwr_str);
                                dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - pwr_sz.x - 7.0f, line1_y),
                                            laser_firing ? IM_COL32(255, 40, 80, 255) : IM_COL32(140, 165, 195, 230), pwr_str);

                                float line2_y = box_p0.y + 31.0f;
                                dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line2_y + 2.0f), IM_COL32(100, 120, 145, 200), "PWM DUTY");
                                char duty_str[32];
                                std::snprintf(duty_str, sizeof(duty_str), "%4u / 1000", pwm_duty);
                                ImVec2 duty_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, duty_str) : ImGui::CalcTextSize(duty_str);
                                dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - duty_sz.x - 7.0f, line2_y), IM_COL32(240, 180, 50, 230), duty_str);

                                ImGui::Dummy(ImVec2(dro_w, dro_h));
                                ImGui::SameLine();

                                ImGui::BeginGroup();
                                {
                                    static float s_drag_laser_ov = 100.0f;
                                    static bool s_is_dragging_laser = false;
                                    if (!s_is_dragging_laser) s_drag_laser_ov = telemetry.laser_power_override_pct;

                                    const bool pwr_ov = (std::abs(telemetry.laser_power_override_pct - 100.0f) >= 0.5f);
                                    if (pwr_ov) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.40f, 0.15f, 0.22f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.50f, 0.65f, 1.0f));
                                    }
                                    if (ImGui::Button("Reset 100%##LaserPwr", ImVec2(right_w, 24.0f))) {
                                        controller.setLaserPowerOverride(100.0f);
                                        s_drag_laser_ov = 100.0f;
                                        markSettingsDirty();
                                    }
                                    if (pwr_ov) ImGui::PopStyleColor(2);

                                    ImGui::SetNextItemWidth(right_w);
                                    if (ImGui::SliderFloat("##LaserPwrSlider", &s_drag_laser_ov, 10.0f, 150.0f, "Power: %.0f%%")) {
                                        controller.setLaserPowerOverride(s_drag_laser_ov);
                                        markSettingsDirty();
                                    }
                                }
                                ImGui::EndGroup();

                                ImGui::Spacing();
                                ImGui::Separator();
                                ImGui::Spacing();

                                float min_d = cfg.laser.min_duty_pct;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderFloat("Min Striking Duty %", &min_d, 0.0f, 25.0f, "%.1f%%")) {
                                    cfg.laser.min_duty_pct = min_d;
                                    changed = true;
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Minimum threshold needed to ignite laser (typically 8-12% for CO2 glass tubes, 0% for diodes).");
                                }

                                float max_d = cfg.laser.max_duty_pct;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderFloat("Max Safe Duty %", &max_d, 50.0f, 100.0f, "%.1f%%")) {
                                    cfg.laser.max_duty_pct = max_d;
                                    changed = true;
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Ceiling power cap (typically 85-90% to extend CO2 glass tube longevity, 100% for diodes).");
                                }

                                float corner_d = cfg.laser.corner_power_floor_pct;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderFloat("Corner Power Floor %", &corner_d, 0.0f, 30.0f, "%.0f%%")) {
                                    cfg.laser.corner_power_floor_pct = corner_d;
                                    changed = true;
                                }
                            }
                            ImGui::EndChild();

                            ImGui::PopStyleVar(4);
                            ImGui::PopStyleColor(2);

                            if (changed) {
                                controller.applyConfig(cfg);
                                markSettingsDirty();
                            }
                        }

                        ImGui::Spacing();

                        // -------------------------------------------------------------
                        // 3. LASER ALIGNMENT, PULSE & FRAMING
                        // -------------------------------------------------------------
                        ImGui::SetNextItemOpen(saved_settings.ui.header_laser_alignment_open, ImGuiCond_Once);
                        bool laser_align_hdr_open = ImGui::CollapsingHeader("Laser Alignment, Pulse & Framing");
                        if (laser_align_hdr_open != saved_settings.ui.header_laser_alignment_open) {
                            saved_settings.ui.header_laser_alignment_open = laser_align_hdr_open;
                            markSettingsDirty();
                        }
                        if (laser_align_hdr_open) {
                            core::MachineConfig cfg = controller.getConfig();
                            bool changed = false;

                            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                            ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                            // Card: Framing Mode & Execution
                            ImGui::BeginChild("##LaserFramingCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "BOUNDING BOX MATERIAL FRAMING");
                                ImGui::Spacing();

                                const char* framing_modes[] = {
                                    "Low-Power Beam (PWM %)",
                                    "Red-Dot Pointer (OUT2)",
                                    "Both (Red Dot + Low Power)",
                                    "Dry-Run Motion Only"
                                };
                                int cur_frame = static_cast<int>(cfg.laser.framing_mode);
                                ImGui::SetNextItemWidth(-1.0f);
                                if (ImGui::Combo("##FramingModeCombo", &cur_frame, framing_modes, IM_ARRAYSIZE(framing_modes))) {
                                    cfg.laser.framing_mode = static_cast<core::LaserFramingMode>(cur_frame);
                                    changed = true;
                                }

                                if (cfg.laser.framing_mode == core::LaserFramingMode::LowPowerBeam || cfg.laser.framing_mode == core::LaserFramingMode::Both) {
                                    float fp = cfg.laser.framing_power_pct;
                                    ImGui::SetNextItemWidth(140.0f);
                                    if (ImGui::SliderFloat("Framing Beam %", &fp, 0.1f, 3.0f, "%.1f%%")) {
                                        cfg.laser.framing_power_pct = fp;
                                        changed = true;
                                    }
                                }

                                float fs = static_cast<float>(cfg.laser.framing_speed_mm_min);
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat("Framing Speed (mm/min)", &fs, 200.0f, 1000.0f, "%.0f")) {
                                    cfg.laser.framing_speed_mm_min = std::max(100.0f, fs);
                                    changed = true;
                                }

                                ImGui::Spacing();
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.45f, 0.70f, 1.0f));
                                if (ImGui::Button("FRAME JOB BOUNDING BOX", ImVec2(-1.0f, 36.0f))) {
                                    controller.frameBoundingBox();
                                }
                                ImGui::PopStyleColor();
                            }
                            ImGui::EndChild();

                            ImGui::Spacing();

                            // Card: Test Fire / Laser Pulse
                            ImGui::BeginChild("##LaserPulseCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "TEST FIRE / FOCUS PULSE");
                                ImGui::Spacing();

                                float p_dur = cfg.laser.pulse_duration_ms;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderFloat("Pulse Duration (ms)", &p_dur, 10.0f, 500.0f, "%.0f ms")) {
                                    cfg.laser.pulse_duration_ms = p_dur;
                                    changed = true;
                                }

                                float p_pwr = cfg.laser.pulse_power_pct;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderFloat("Pulse Power %", &p_pwr, 1.0f, 50.0f, "%.0f%%")) {
                                    cfg.laser.pulse_power_pct = p_pwr;
                                    changed = true;
                                }

                                ImGui::Spacing();
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.20f, 0.15f, 1.0f));
                                if (ImGui::Button("FIRE TEST PULSE", ImVec2(-1.0f, 32.0f))) {
                                    controller.testFireLaserPulse(cfg.laser.pulse_duration_ms, cfg.laser.pulse_power_pct);
                                }
                                ImGui::PopStyleColor();
                            }
                            ImGui::EndChild();

                            ImGui::Spacing();

                            // Card: Red Dot Crosshair & Offset
                            ImGui::BeginChild("##LaserRedDotCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "RED-DOT CROSSHAIR POINTER (OUT2)");
                                ImGui::Spacing();

                                const bool red_on = telemetry.laser_red_pointer_active;
                                ImVec4 red_btn_col = red_on ? ImVec4(0.85f, 0.15f, 0.20f, 1.0f) : ImVec4(0.20f, 0.25f, 0.32f, 1.0f);
                                ImGui::PushStyleColor(ImGuiCol_Button, red_btn_col);
                                const char* red_lbl = red_on ? "RED-DOT POINTER: ON (CLICK TO DISABLE)"
                                                             : "RED-DOT POINTER: OFF (CLICK TO ENABLE)";
                                if (ImGui::Button(red_lbl, ImVec2(-1.0f, 30.0f))) {
                                    controller.setRedPointer(!red_on);
                                }
                                ImGui::PopStyleColor();

                                float off_x = static_cast<float>(cfg.laser.red_dot_offset_x_mm);
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat("Offset X (mm)", &off_x, 0.5f, 5.0f, "%.2f mm")) {
                                    cfg.laser.red_dot_offset_x_mm = off_x;
                                    changed = true;
                                }

                                float off_y = static_cast<float>(cfg.laser.red_dot_offset_y_mm);
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat("Offset Y (mm)", &off_y, 0.5f, 5.0f, "%.2f mm")) {
                                    cfg.laser.red_dot_offset_y_mm = off_y;
                                    changed = true;
                                }

                                if (ImGui::Button("Zero Work Coordinates to Red Dot", ImVec2(-1.0f, 26.0f))) {
                                    controller.setWorkPosition(core::Axis::X, telemetry.work_pos_mm[0] - cfg.laser.red_dot_offset_x_mm);
                                    controller.setWorkPosition(core::Axis::Y, telemetry.work_pos_mm[1] - cfg.laser.red_dot_offset_y_mm);
                                }
                            }
                            ImGui::EndChild();

                            ImGui::PopStyleVar(4);
                            ImGui::PopStyleColor(2);

                            if (changed) {
                                controller.applyConfig(cfg);
                                markSettingsDirty();
                            }
                        }

                        ImGui::Spacing();

                        // -------------------------------------------------------------
                        // 4. AIR ASSIST & AUXILIARY OUTPUTS
                        // -------------------------------------------------------------
                        ImGui::SetNextItemOpen(saved_settings.ui.header_laser_air_open, ImGuiCond_Once);
                        bool laser_air_hdr_open = ImGui::CollapsingHeader("Air Assist & Auxiliary Outputs (OUT3 / OUT4)");
                        if (laser_air_hdr_open != saved_settings.ui.header_laser_air_open) {
                            saved_settings.ui.header_laser_air_open = laser_air_hdr_open;
                            markSettingsDirty();
                        }
                        if (laser_air_hdr_open) {
                            core::MachineConfig cfg = controller.getConfig();
                            bool changed = false;

                            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                            ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                            // Card: Air Assist Solenoid
                            ImGui::BeginChild("##LaserAirAssistCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "AIR ASSIST SOLENOID (OUT3 / M8 / M9)");
                                ImGui::Spacing();

                                if (ImGui::Checkbox("Auto air assist on laser start", &cfg.laser.auto_air_assist)) {
                                    changed = true;
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("When checked, automatically energizes OUT3 air solenoid whenever the laser fires. When unchecked, strictly follows M8/M9 G-code.");
                                }

                                const bool air_on = telemetry.laser_air_assist_active;
                                ImVec4 air_col = air_on ? ImVec4(0.00f, 0.65f, 0.85f, 1.0f) : ImVec4(0.20f, 0.25f, 0.32f, 1.0f);
                                ImGui::PushStyleColor(ImGuiCol_Button, air_col);
                                const char* air_lbl = air_on ? "AIR ASSIST [OUT3]: ACTIVE (CLICK TO STOP)"
                                                             : "MANUAL AIR ASSIST [OUT3]: OFF (CLICK TO OPEN)";
                                if (ImGui::Button(air_lbl, ImVec2(-1.0f, 30.0f))) {
                                    controller.setAirAssist(!air_on);
                                }
                                ImGui::PopStyleColor();

                                float preflow = cfg.laser.air_preflow_s;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderFloat("Air Pre-Flow Delay (s)", &preflow, 0.0f, 2.0f, "%.2f s")) {
                                    cfg.laser.air_preflow_s = preflow;
                                    changed = true;
                                }

                                float postflow = cfg.laser.air_postflow_s;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderFloat("Air Post-Flow Delay (s)", &postflow, 0.0f, 5.0f, "%.2f s")) {
                                    cfg.laser.air_postflow_s = postflow;
                                    changed = true;
                                }
                            }
                            ImGui::EndChild();

                            ImGui::Spacing();

                            // Card: Exhaust Fan Relay
                            ImGui::BeginChild("##LaserExhaustCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "EXHAUST BLOWER / FUME EXTRACTOR (OUT4)");
                                ImGui::Spacing();

                                const bool exhaust_on = (telemetry.outputs_bitmask & core::OutputBit::OUT4_MistCoolant) != 0;
                                ImVec4 ex_col = exhaust_on ? ImVec4(0.20f, 0.75f, 0.40f, 1.0f) : ImVec4(0.20f, 0.25f, 0.32f, 1.0f);
                                ImGui::PushStyleColor(ImGuiCol_Button, ex_col);
                                const char* ex_lbl = exhaust_on ? "EXHAUST FAN [OUT4]: RUNNING"
                                                                : "TOGGLE EXHAUST FAN [OUT4]: OFF";
                                if (ImGui::Button(ex_lbl, ImVec2(-1.0f, 30.0f))) {
                                    controller.setAuxOutput(4, !exhaust_on);
                                }
                                ImGui::PopStyleColor();
                            }
                            ImGui::EndChild();

                            ImGui::PopStyleVar(4);
                            ImGui::PopStyleColor(2);

                            if (changed) {
                                controller.applyConfig(cfg);
                                markSettingsDirty();
                            }
                        }

                        ImGui::Spacing();

                        // -------------------------------------------------------------
                        // 5. LASER SAFETY INTERLOCKS (4-INPUT TERMINALS)
                        // -------------------------------------------------------------
                        ImGui::SetNextItemOpen(saved_settings.ui.header_laser_interlocks_open, ImGuiCond_Once);
                        bool laser_safe_hdr_open = ImGui::CollapsingHeader("Laser Safety Interlocks (IN1 - IN4)");
                        if (laser_safe_hdr_open != saved_settings.ui.header_laser_interlocks_open) {
                            saved_settings.ui.header_laser_interlocks_open = laser_safe_hdr_open;
                            markSettingsDirty();
                        }
                        if (laser_safe_hdr_open) {
                            core::MachineConfig cfg = controller.getConfig();
                            bool changed = false;

                            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                            ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                            ImGui::BeginChild("##LaserInterlocksCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "OPTICAL & COOLING SAFETY INTERLOCKS");
                                ImGui::Spacing();

                                ImGui::BeginChild("##LaserSafeStatusBox", ImVec2(0.0f, telemetry.simulator_mode ? 68.0f : 36.0f), true, ImGuiWindowFlags_NoScrollbar);
                                {
                                    ImGui::TextColored(telemetry.laser_chiller_flow_ok ? ImVec4(0.15f, 0.95f, 0.45f, 1.0f) : ImVec4(0.95f, 0.20f, 0.20f, 1.0f),
                                                       telemetry.laser_chiller_flow_ok ? "[CHILLER: OK]" : "[CHILLER: NO FLOW]");
                                    ImGui::SameLine();
                                    ImGui::TextColored(telemetry.laser_door_closed ? ImVec4(0.15f, 0.95f, 0.45f, 1.0f) : ImVec4(1.00f, 0.75f, 0.10f, 1.0f),
                                                       telemetry.laser_door_closed ? "[DOOR: CLOSED]" : "[LID OPEN]");
                                    ImGui::SameLine();
                                    ImGui::TextColored(!telemetry.laser_fault_active ? ImVec4(0.15f, 0.95f, 0.45f, 1.0f) : ImVec4(0.95f, 0.20f, 0.20f, 1.0f),
                                                       !telemetry.laser_fault_active ? "[PSU: READY]" : "[PSU FAULT]");

                                    if (telemetry.simulator_mode) {
                                        ImGui::TextDisabled("Sim Test:");
                                        ImGui::SameLine();
                                        if (cfg.laser.chiller_flow_pin >= 1) {
                                            bool c_sim = ((telemetry.simulated_inputs_bitmask >> (cfg.laser.chiller_flow_pin - 1)) & 1U) != 0;
                                            if (ImGui::SmallButton(c_sim ? "Trip Chiller" : "Restore Chiller")) {
                                                controller.setSimulatedInputPin(cfg.laser.chiller_flow_pin, !c_sim);
                                            }
                                            ImGui::SameLine();
                                        }
                                        if (cfg.laser.door_interlock_pin >= 1) {
                                            bool d_sim = ((telemetry.simulated_inputs_bitmask >> (cfg.laser.door_interlock_pin - 1)) & 1U) != 0;
                                            if (ImGui::SmallButton(d_sim ? "Close Lid" : "Open Lid")) {
                                                controller.setSimulatedInputPin(cfg.laser.door_interlock_pin, !d_sim);
                                            }
                                        }
                                    }
                                }
                                ImGui::EndChild();

                                ImGui::Spacing();

                                int chill_pin = cfg.laser.chiller_flow_pin;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderInt("Water Chiller Pin (0=Off, IN1..4)", &chill_pin, 0, 4)) {
                                    cfg.laser.chiller_flow_pin = static_cast<uint8_t>(chill_pin);
                                    changed = true;
                                }
                                ImGui::SameLine();
                                if (ImGui::Checkbox("Active Low##Chill", &cfg.laser.chiller_flow_invert)) changed = true;
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Active Low: Checked = flow sensor closes to COM/GND when cooling water is flowing normally.");
                                }

                                int door_pin = cfg.laser.door_interlock_pin;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderInt("Door / Lid Interlock Pin", &door_pin, 0, 4)) {
                                    cfg.laser.door_interlock_pin = static_cast<uint8_t>(door_pin);
                                    changed = true;
                                }
                                ImGui::SameLine();
                                if (ImGui::Checkbox("Active Low##Door", &cfg.laser.door_interlock_invert)) changed = true;
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Active Low: Checked = switch closes to COM/GND when enclosure door is closed.");
                                }

                                int fault_pin = cfg.laser.laser_fault_pin;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderInt("Laser PSU Alarm Pin", &fault_pin, 0, 4)) {
                                    cfg.laser.laser_fault_pin = static_cast<uint8_t>(fault_pin);
                                    changed = true;
                                }
                                ImGui::SameLine();
                                if (ImGui::Checkbox("Active Low##Fault", &cfg.laser.laser_fault_invert)) changed = true;
                            }
                            ImGui::EndChild();

                            ImGui::PopStyleVar(4);
                            ImGui::PopStyleColor(2);

                            if (changed) {
                                controller.applyConfig(cfg);
                                markSettingsDirty();
                            }
                        }
                    }

                    ImGui::Spacing();

                    // -----------------------------------------------------------------
                    // WORKSPACES / COORDINATE SYSTEMS (G54 - G59)
                    // -----------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_workspaces_open, ImGuiCond_Once);
                    bool ws_hdr_open = ImGui::CollapsingHeader("Workspaces / Coordinate Systems (G54 - G59)");
                    if (ws_hdr_open != saved_settings.ui.header_workspaces_open) {
                        saved_settings.ui.header_workspaces_open = ws_hdr_open;
                        markSettingsDirty();
                    }
                    if (ws_hdr_open) {
                        workspace_panel.render(controller, telemetry, false, [&]() {
                            saved_settings.machine.workspace_offsets_mm = controller.getWorkspaceOffsets();
                            saved_settings.machine.active_workspace     = controller.getActiveWorkspace();
                            markSettingsDirty();
                        });
                    }

                    ImGui::Spacing();

                    // -----------------------------------------------------------------
                    // REMOTE WEBCAM MONITORING (BREAKOUTCUT — MONITORING ONLY)
                    // -----------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_camera_open, ImGuiCond_Once);
                    bool cam_hdr_open = ImGui::CollapsingHeader("Remote Webcam Monitoring");
                    if (cam_hdr_open != saved_settings.ui.header_camera_open) {
                        saved_settings.ui.header_camera_open = cam_hdr_open;
                        markSettingsDirty();
                    }
                    if (cam_hdr_open) {
                        if (camera_panel.render(controller, telemetry, nullptr, false)) {
                            saved_settings.machine = controller.getConfig();
                            markSettingsDirty();
                        }
                    }

                    ImGui::Spacing();

                    // -------------------------------------------------------------
                    // 5. END-STOPS, HOMING & SOFT LIMITS
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_limits_open, ImGuiCond_Once);
                    bool limits_hdr_open = ImGui::CollapsingHeader("End-Stops, Homing & Soft Limits");
                    if (limits_hdr_open != saved_settings.ui.header_limits_open) {
                        saved_settings.ui.header_limits_open = limits_hdr_open;
                        markSettingsDirty();
                    }
                    if (limits_hdr_open) {
                        core::MachineConfig cfg = controller.getConfig();
                        bool changed = false;

                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                        // 1. Hardware E-Stop Input Pin
                        ImGui::BeginChild("##PlasmaEStopLimitCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "HARDWARE EMERGENCY STOP (E-STOP PIN)");
                            ImGui::Spacing();

                            int estop_pin = cfg.estop_pin;
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::SliderInt("E-Stop Pin (0=Off, IN1..12)", &estop_pin, 0, 12)) {
                                cfg.estop_pin = static_cast<uint8_t>(estop_pin);
                                changed = true;
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("Active Low##EStop", &cfg.estop_invert)) changed = true;
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Active Low: Checked = Normally-Closed (NC) switch connected to COM/GND.\nTriggers E-Stop when opened or pulled low. Unchecked = Active High (NO switch).");
                            }
                        }
                        ImGui::EndChild();

                        ImGui::Spacing();

                        // 2. Homing requirement
                        ImGui::BeginChild("##PlasmaHomingRequiredCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "HOMING");
                            ImGui::Spacing();
                            if (ImGui::Checkbox("Require homing for G53 / G28 / G30##ReqHome", &cfg.require_homing)) changed = true;
                            ImGui::PushTextWrapPos(0.0f);
                            if (cfg.require_homing) {
                                ImGui::TextDisabled("Programs and MDI commands that move in machine coordinates (G53, G28, G30) "
                                                    "are refused until those axes are homed.");
                            } else {
                                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.30f, 1.0f),
                                                   "Machine coordinates are then not tied to switches: they continue from the position saved when the app last closed. "
                                                   "If the axes were moved by hand or lost steps, G53 / G28 / G30 moves go to the wrong place. "
                                                   "Use this only on a machine without home switches.");
                            }
                            ImGui::PopTextWrapPos();
                        }
                        ImGui::EndChild();

                        ImGui::Spacing();

                        const char* ax_names[3] = {"X AXIS LIMITS & HOMING", "Y AXIS LIMITS & HOMING", "Z AXIS LIMITS & HOMING"};
                        const char* card_ids[3] = {"##LimitsXCard", "##LimitsYCard", "##LimitsZCard"};
                        for (int a = 0; a < 3; ++a) {
                            ImGui::PushID(100 + a);
                            ImGui::BeginChild(card_ids[a], ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "%s", ax_names[a]);
                                ImGui::Spacing();

                                int home_pin = cfg.axes[a].home_pin;
                                int lim_m    = cfg.axes[a].limit_minus_pin;
                                int lim_p    = cfg.axes[a].limit_plus_pin;

                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderInt("Home Pin (0=Off, IN1..12)", &home_pin, 0, 12)) {
                                    cfg.axes[a].home_pin = static_cast<uint8_t>(home_pin);
                                    changed = true;
                                }
                                ImGui::SameLine();
                                if (ImGui::Checkbox("Active Low##H", &cfg.axes[a].home_invert)) changed = true;
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Active Low: Checked = Switch connects to COM/GND when triggered.\nUnchecked = Active High.");
                                }

                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderInt("Limit- Pin (0=Off, IN1..12)", &lim_m, 0, 12)) {
                                    cfg.axes[a].limit_minus_pin = static_cast<uint8_t>(lim_m);
                                    changed = true;
                                }
                                ImGui::SameLine();
                                if (ImGui::Checkbox("Active Low##LM", &cfg.axes[a].limit_minus_invert)) changed = true;
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Active Low: Checked = Switch connects to COM/GND when triggered.\nUnchecked = Active High.");
                                }

                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::SliderInt("Limit+ Pin (0=Off, IN1..12)", &lim_p, 0, 12)) {
                                    cfg.axes[a].limit_plus_pin = static_cast<uint8_t>(lim_p);
                                    changed = true;
                                }
                                ImGui::SameLine();
                                if (ImGui::Checkbox("Active Low##LP", &cfg.axes[a].limit_plus_invert)) changed = true;
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Active Low: Checked = Switch connects to COM/GND when triggered.\nUnchecked = Active High.");
                                }

                                ImGui::Spacing();
                                ImGui::Separator();
                                ImGui::Spacing();

                                if (ImGui::Checkbox("Home Dir (-)", &cfg.axes[a].homing_dir_negative)) {
                                    changed = true;
                                }

                                bool is_imp = (cfg.linear_units == core::LinearUnits::Imperial);
                                double dist_scale = is_imp ? (1.0 / 25.4) : 1.0;

                                float pulloff = static_cast<float>(cfg.axes[a].home_pulloff_mm * dist_scale);
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat(is_imp ? "Home Pull-Off (in)" : "Home Pull-Off (mm)", &pulloff, is_imp ? 0.02f : 0.5f, is_imp ? 0.1f : 2.0f, is_imp ? "%.3f" : "%.2f")) {
                                    cfg.axes[a].home_pulloff_mm = std::max(0.01, static_cast<double>(pulloff) / dist_scale);
                                    changed = true;
                                }

                                float h_feed = static_cast<float>(cfg.axes[a].homing_feed_mm_min * dist_scale);
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat(is_imp ? "Homing Speed (IPM)" : "Homing Speed (mm/min)", &h_feed, is_imp ? 2.0f : 50.0f, is_imp ? 10.0f : 200.0f, is_imp ? "%.1f" : "%.0f")) {
                                    cfg.axes[a].homing_feed_mm_min = std::max(1.0, static_cast<double>(h_feed) / dist_scale);
                                    changed = true;
                                }

                                ImGui::Spacing();
                                ImGui::Separator();
                                ImGui::Spacing();

                                if (ImGui::Checkbox("Enable Soft Limits", &cfg.axes[a].soft_limits_enabled)) {
                                    changed = true;
                                }
                                if (cfg.axes[a].soft_limits_enabled) {
                                    float smin = static_cast<float>(cfg.axes[a].soft_limit_min_mm * dist_scale);
                                    float smax = static_cast<float>(cfg.axes[a].soft_limit_max_mm * dist_scale);
                                    ImGui::SetNextItemWidth(140.0f);
                                    if (ImGui::InputFloat(is_imp ? "Soft Limit Min (in)" : "Soft Limit Min (mm)", &smin, is_imp ? 0.2f : 5.0f, is_imp ? 1.0f : 25.0f, is_imp ? "%.2f" : "%.1f")) {
                                        cfg.axes[a].soft_limit_min_mm = static_cast<double>(smin) / dist_scale;
                                        changed = true;
                                    }
                                    ImGui::SetNextItemWidth(140.0f);
                                    if (ImGui::InputFloat(is_imp ? "Soft Limit Max (in)" : "Soft Limit Max (mm)", &smax, is_imp ? 0.2f : 5.0f, is_imp ? 1.0f : 25.0f, is_imp ? "%.2f" : "%.1f")) {
                                        cfg.axes[a].soft_limit_max_mm = static_cast<double>(smax) / dist_scale;
                                        changed = true;
                                    }
                                }
                            }
                            ImGui::EndChild();
                            ImGui::PopID();
                            ImGui::Spacing();
                        }

                        ImGui::PopStyleVar(4);
                        ImGui::PopStyleColor(2);

                        if (changed) {
                            controller.applyConfig(cfg);
                            markSettingsDirty();
                        }
                    }

                    ImGui::Spacing();

                    // -------------------------------------------------------------
                    // 6. MACHINE MOTOR TUNING
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_motor_open, ImGuiCond_Once);
                    bool motor_hdr_open = ImGui::CollapsingHeader("Machine Motor Tuning (Steps, Speed, Accel/Decel)");
                    if (motor_hdr_open != saved_settings.ui.header_motor_open) {
                        saved_settings.ui.header_motor_open = motor_hdr_open;
                        markSettingsDirty();
                    }
                    if (motor_hdr_open) {
                        core::MachineConfig cfg = controller.getConfig();
                        bool changed = false;

                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                        // Card: Global System Units & Velocity Configuration
                        ImGui::BeginChild("##PlasmaMotorUnitCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "UNITS, VELOCITY & MACH3 PROFILE MIGRATION");
                            ImGui::Spacing();

                            // Mach3 Migration Action Button
                            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.10f, 0.44f, 0.72f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.15f, 0.56f, 0.88f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.08f, 0.35f, 0.58f, 1.0f));
                            if (ImGui::Button("MIGRATE FROM MACH3 XML...", ImVec2(-1.0f, 30.0f))) {
                                mach3_wizard.openMigrationWizard(controller.getConfig());
                            }
                            ImGui::PopStyleColor(3);
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Import motor tuning (steps/unit, velocity, acceleration), pin assignments,\ninversions, homing speeds, soft limits, and torch/output relays directly from a Mach3 XML profile\nfollowed by the interactive 5-Stage Post-Migration Machine Verification Routine.");
                            }

                            ImGui::Spacing();
                            ImGui::Separator();
                            ImGui::Spacing();

                            // Unit Selector
                            ImGui::Text("Unit of Measurement:");
                            ImGui::SameLine();
                            const bool is_metric = (cfg.linear_units == core::LinearUnits::Metric);
                            const bool is_imperial = (cfg.linear_units == core::LinearUnits::Imperial);

                            if (is_metric) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.85f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.55f, 0.95f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.26f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.30f, 0.36f, 1.0f));
                            }
                            if (ImGui::Button("  METRIC (mm)  ")) {
                                if (!is_metric) {
                                    cfg.linear_units = core::LinearUnits::Metric;
                                    saved_settings.ui.speed_unit_idx = 0;
                                    changed = true;
                                }
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Set system-wide machine units to Metric (Millimeters, mm/min, mm/s).");
                            }

                            ImGui::SameLine();

                            if (is_imperial) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.85f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.55f, 0.95f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.26f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.28f, 0.30f, 0.36f, 1.0f));
                            }
                            if (ImGui::Button("  IMPERIAL (Inches)  ")) {
                                if (!is_imperial) {
                                    cfg.linear_units = core::LinearUnits::Imperial;
                                    saved_settings.ui.speed_unit_idx = 0;
                                    changed = true;
                                }
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Set system-wide machine units to Imperial (Inches, in/min [IPM], in/s).");
                            }

                            ImGui::Spacing();
                            ImGui::Separator();
                            ImGui::Spacing();

                            // Filtered velocity configuration unit based on global unit
                            const char* metric_speed_units[] = {
                                "mm/min (Metric mm/m)",
                                "mm/sec (Metric mm/s)"
                            };
                            const char* imperial_speed_units[] = {
                                "in/min (Imperial IPM)",
                                "in/sec (Imperial in/s)"
                            };

                            int current_speed_idx = std::clamp(saved_settings.ui.speed_unit_idx, 0, 1);

                            ImGui::SetNextItemWidth(220.0f);
                            if (is_metric) {
                                if (ImGui::Combo("Velocity Unit", &current_speed_idx, metric_speed_units, IM_ARRAYSIZE(metric_speed_units))) {
                                    saved_settings.ui.speed_unit_idx = current_speed_idx;
                                    markSettingsDirty();
                                }
                            } else {
                                if (ImGui::Combo("Velocity Unit", &current_speed_idx, imperial_speed_units, IM_ARRAYSIZE(imperial_speed_units))) {
                                    saved_settings.ui.speed_unit_idx = current_speed_idx;
                                    markSettingsDirty();
                                }
                            }
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Configure the units used for max velocity and feedrate tuning for all axes.");
                            }
                        }
                        ImGui::EndChild();

                        ImGui::Spacing();

                        const bool is_imperial = (cfg.linear_units == core::LinearUnits::Imperial);
                        const int speed_sub_idx = std::clamp(saved_settings.ui.speed_unit_idx, 0, 1);

                        double to_disp_factor = 60.0;
                        const char* speed_label = "Max Speed (mm/m)";
                        const char* speed_fmt = "%.0f";
                        float speed_step = 100.0f;
                        float speed_step_fast = 500.0f;

                        const char* steps_label = "Steps/mm";
                        const char* accel_label = "Acceleration (mm/s^2)";
                        const char* decel_label = "Deceleration (mm/s^2)";
                        double steps_mult = 1.0;
                        double accel_mult = 1.0;

                        if (!is_imperial) {
                            if (speed_sub_idx == 1) {
                                to_disp_factor = 1.0;
                                speed_label = "Max Speed (mm/s)";
                                speed_fmt = "%.1f";
                                speed_step = 5.0f;
                                speed_step_fast = 25.0f;
                            } else {
                                to_disp_factor = 60.0;
                                speed_label = "Max Speed (mm/m)";
                                speed_fmt = "%.0f";
                                speed_step = 100.0f;
                                speed_step_fast = 500.0f;
                            }
                            steps_label = "Steps/mm";
                            accel_label = "Acceleration (mm/s^2)";
                            decel_label = "Deceleration (mm/s^2)";
                            steps_mult = 1.0;
                            accel_mult = 1.0;
                        } else {
                            if (speed_sub_idx == 1) {
                                to_disp_factor = 1.0 / 25.4;
                                speed_label = "Max Speed (in/s)";
                                speed_fmt = "%.2f";
                                speed_step = 0.5f;
                                speed_step_fast = 2.0f;
                            } else {
                                to_disp_factor = 60.0 / 25.4;
                                speed_label = "Max Speed (IPM)";
                                speed_fmt = "%.1f";
                                speed_step = 10.0f;
                                speed_step_fast = 50.0f;
                            }
                            steps_label = "Steps/inch";
                            accel_label = "Acceleration (in/s^2)";
                            decel_label = "Deceleration (in/s^2)";
                            steps_mult = 25.4;
                            accel_mult = 1.0 / 25.4;
                        }

                        const char* ax_names[3] = {"X AXIS MOTOR TUNING", "Y AXIS MOTOR TUNING", "Z AXIS MOTOR TUNING"};
                        const char* motor_card_ids[3] = {"##MotorXCard", "##MotorYCard", "##MotorZCard"};
                        for (int a = 0; a < 3; ++a) {
                            ImGui::PushID(200 + a);
                            ImGui::BeginChild(motor_card_ids[a], ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "%s", ax_names[a]);
                                ImGui::Spacing();

                                float steps = static_cast<float>(cfg.axes[a].steps_per_mm * steps_mult);
                                float disp_v = static_cast<float>(cfg.axes[a].max_velocity_mm_s * to_disp_factor);
                                float acc   = static_cast<float>(cfg.axes[a].acceleration_mm_s2 * accel_mult);
                                float dec   = static_cast<float>(cfg.axes[a].deceleration_mm_s2 * accel_mult);

                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat(steps_label, &steps, 1.0f, 10.0f, "%.2f")) {
                                    cfg.axes[a].steps_per_mm = std::max(0.1, static_cast<double>(steps) / steps_mult);
                                    changed = true;
                                }
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat(speed_label, &disp_v, speed_step, speed_step_fast, speed_fmt)) {
                                    cfg.axes[a].max_velocity_mm_s = std::max(0.01, static_cast<double>(disp_v) / to_disp_factor);
                                    changed = true;
                                }
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat(accel_label, &acc, is_imperial ? 1.0f : 25.0f, is_imperial ? 5.0f : 100.0f, "%.1f")) {
                                    cfg.axes[a].acceleration_mm_s2 = std::max(0.5, static_cast<double>(acc) / accel_mult);
                                    changed = true;
                                }
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputFloat(decel_label, &dec, is_imperial ? 1.0f : 25.0f, is_imperial ? 5.0f : 100.0f, "%.1f")) {
                                    cfg.axes[a].deceleration_mm_s2 = std::max(0.5, static_cast<double>(dec) / accel_mult);
                                    changed = true;
                                }

                                ImGui::Spacing();
                                ImGui::Separator();
                                ImGui::Spacing();

                                if (ImGui::Checkbox("Step Active Low", &cfg.axes[a].step_active_low)) {
                                    changed = true;
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Invert step pulse polarity (Active Low vs Active High) for stepper driver optocoupler.");
                                }
                                ImGui::SameLine();
                                if (ImGui::Checkbox("Dir Invert (Active Low)", &cfg.axes[a].dir_active_low)) {
                                    changed = true;
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Invert direction signal polarity to reverse motor rotation without swapping motor wires.");
                                }
                            }
                            ImGui::EndChild();
                            ImGui::PopID();
                            ImGui::Spacing();
                        }

                        if (ui::renderGantrySlaveCombo(cfg)) {
                            changed = true;
                        }

                        ImGui::PopStyleVar(4);
                        ImGui::PopStyleColor(2);

                        if (changed) {
                            controller.applyConfig(cfg);
                            markSettingsDirty();
                        }
                    }
                }
                ImGui::EndChild(); // ##PlasmaSettingsScroll
            }
            ImGui::EndChild(); // ##PlasmaRightCol
        } // if (!minimal_layout)

        // Exit Confirmation Modal when cycle is active
        if (show_exit_confirm_modal) {
            ImGui::OpenPopup("Confirm Exit While Running##ExitModal");
            show_exit_confirm_modal = false;
        }

        ImVec2 modal_center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(modal_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f));
        if (ImGui::BeginPopupModal("Confirm Exit While Running##ExitModal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
            ImGui::TextUnformatted("WARNING: CNC Cycle Is Currently Running!");
            ImGui::PopStyleColor();
            ImGui::Spacing();
            ImGui::TextWrapped("A cycle or motion command is currently running. Exiting the software while the machine is running can cause loss of position or leave torch outputs energized.");
            ImGui::Spacing();
            ImGui::TextWrapped("Clicking 'E-STOP & EXIT' will immediately engage E-Stop, halt all pulse generation and torch control, and safely close the application.");
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.30f, 0.30f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.70f, 0.15f, 0.15f, 1.0f));
            if (ImGui::Button("E-STOP & EXIT PROGRAM", ImVec2(220.0f, 36.0f))) {
                force_close = true;
                controller.triggerEStop();
                window.closeWindow();
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(3);

            ImGui::SameLine();
            if (ImGui::Button("CANCEL (KEEP RUNNING)", ImVec2(220.0f, 36.0f))) {
                ImGui::CloseCurrentPopup();
            }

            ImGui::EndPopup();
        }

        // Interactive Script Operator Prompt Modal
        scripting_panel.renderPromptModal(controller);
        scripting_panel.renderHelpModal(controller);

        // Mach3 XML Profile Migration & 5-Stage Post-Migration Machine Verification Wizard
        if (mach3_wizard.render(controller, true)) {
            saved_settings.machine = controller.getConfig();
            markSettingsDirty();
        }

        // 4. Bottom Hardware I/O Status Bar
        ui::IoStatusBar::renderBottomIoStrip(controller, telemetry);
    });

    // Stop the web server first: its client threads use callbacks that capture locals of main()
    stream_server.stop();

    // Ensure E-Stop is engaged and automatically persist all settings on exit
    controller.triggerEStop();
    saveCurrentSettings();
    return smoke_failed ? 2 : exit_code;
}
