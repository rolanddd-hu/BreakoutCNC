#include "breakout_version.h"
#include "core/controller/rnr_controller.h"
#include "core/config/settings_manager.h"
#include "ui_common/app_window.h"
#include "ui_common/toolpath_viewport.h"
#include "ui_common/dro_panel.h"
#include "ui_common/io_status_bar.h"
#include "ui_common/gcode_panel.h"
#include "ui_common/scripting_panel.h"
#include "ui_common/probe_panel.h"
#include "ui_common/workspace_panel.h"
#include "ui_common/tool_table_panel.h"
#include "ui_common/gantry_slave_combo.h"
#include "ui_common/file_dialog.h"
#include "ui_common/mach3_migration_dialog.h"
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

int main(int /*argc*/, char** /*argv*/) {
    const std::string settings_file = "breakout_mill_settings.ini";
    config::PersistentSettings saved_settings{};

    core::RnRController controller;
    saved_settings.machine = controller.getConfig();
    saved_settings.machine.spindle_ramp_up_s   = 2.5; // Default 2.5s spindle spin-up before cutting starts
    saved_settings.machine.spindle_ramp_down_s = 2.0; // Default 2.0s spindle spin-down
    saved_settings.ui.isometric_3d = true;            // Default 3D Isometric view for Mill
    bool had_saved_settings = config::SettingsManager::loadFromFile(settings_file, saved_settings);
    if (!had_saved_settings) {
        // Fallback to legacy settings file if present
        had_saved_settings = config::SettingsManager::loadFromFile("rnr_mill_settings.ini", saved_settings);
    }

    ui::AppWindow window(
        "BreakoutMill - 4-Axis CNC Mill Controller (BreakoutCNC v" BREAKOUTCNC_VERSION_STRING ")",
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
    viewport.setFollowToolLabel("Follow Endmill");
    viewport.setIsometric3D(saved_settings.ui.isometric_3d);
    ui::DroAndJogPanel    dro_panel;
    ui::GCodeProgramPanel gcode_panel;
    gcode_panel.setMachineType(gcode::MachineType::Mill);
    ui::ScriptingPanel    scripting_panel;
    ui::ProbePanel        probe_panel;
    ui::WorkspacePanel    workspace_panel;
    ui::ToolTablePanel    tool_table_panel;
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
    viewport.setCameraManager(&camera_mgr);

    controller.setScriptLogCallback([&scripting_panel](const std::string& msg, bool is_err) {
        scripting_panel.appendLog(msg, is_err);
    });

    // Mill-specific controller behaviour (spindle direction, no THC / plasma pierce rules)
    saved_settings.machine.is_mill = true;

    // Apply loaded machine, camera & UI settings
    camera_mgr.applyConfig(saved_settings.machine.camera, saved_settings.ui.simulator_mode, true);
    stream_server.applyConfig(saved_settings.machine.camera);
    controller.applyConfig(saved_settings.machine);

    int spindle_rpm = saved_settings.ui.spindle_rpm;
    float hole_bore_diameter_hint_mm = 25.0f;

    if (had_saved_settings) {
        if (saved_settings.ui.simulator_mode) {
            controller.setSimulatorMode(true);
        }
        controller.setDryRunMode(saved_settings.ui.dry_run_mode);
        controller.setFeedOverride(saved_settings.ui.feed_override_pct);

        viewport.setShowRapids(saved_settings.ui.show_rapids);
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

        // Restore the tool in the spindle and its length offset (tool table mode)
        const int saved_tool = saved_settings.ui.last_tool_in_spindle;
        const int saved_tlo  = saved_settings.ui.last_tlo_tool;
        if (saved_tool > 0) {
            controller.selectToolInSpindle(static_cast<uint16_t>(saved_tool));
        }
        if (saved_tlo != saved_tool) {
            controller.activateToolLengthOffset(static_cast<uint16_t>(saved_tlo));
        }
    }

    // Always engage E-Stop by default on program start so machine is disarmed until operator clicks RESET E-STOP
    controller.triggerEStop();

    // Load Mach3 ATC multi-tool demo program by default for Mill (or user's last loaded file)
    controller.loadGCodeString(gcode::GCodeParser::sampleMillAtcGCode());
    std::string loaded_filename = "Built-in Mill ATC Demo (T1 6mm EndMill + T2 3mm Drill)";
    std::string persisted_file_path;
    if (!saved_settings.ui.last_gcode_file_path.empty() &&
        std::filesystem::exists(saved_settings.ui.last_gcode_file_path))
    {
        std::ifstream ifs(saved_settings.ui.last_gcode_file_path);
        if (ifs) {
            std::ostringstream ss;
            ss << ifs.rdbuf();
            if (controller.loadGCodeString(ss.str())) {
                loaded_filename     = saved_settings.ui.last_gcode_file_path;
                persisted_file_path = saved_settings.ui.last_gcode_file_path;
            }
        }
    }

    const auto initial_program = controller.getProgram();
    const auto& initial_parse = *initial_program;
    viewport.fitToBounds(initial_parse.bbox_min_mm, initial_parse.bbox_max_mm);

    float s_dro_user_h   = saved_settings.ui.dro_jog_height;
    bool  minimal_layout = saved_settings.ui.minimal_layout;
    bool  s_settings_dirty = false;
    double s_last_dirty_time = 0.0;
    auto markSettingsDirty = [&]() {
        s_settings_dirty = true;
        s_last_dirty_time = ImGui::GetTime();
    };

    dro_panel.setOnConfigChanged([&]() {
        saved_settings.machine = controller.getConfig();
        markSettingsDirty();
    });

    auto saveCurrentSettings = [&]() {
        const auto tel = controller.getTelemetry();
        saved_settings.machine = controller.getConfig();
        window.getWindowGeometry(
            saved_settings.ui.window_pos_x,
            saved_settings.ui.window_pos_y,
            saved_settings.ui.window_width,
            saved_settings.ui.window_height,
            saved_settings.ui.window_maximized);
        saved_settings.ui.window_fullscreen = window.isFullscreen();

        saved_settings.ui.simulator_mode       = tel.simulator_mode;
        saved_settings.ui.dry_run_mode         = tel.dry_run_mode;
        saved_settings.ui.feed_override_pct    = tel.feed_override_pct;
        saved_settings.ui.spindle_rpm          = spindle_rpm;

        saved_settings.ui.isometric_3d         = viewport.isIsometric3D();
        saved_settings.ui.show_rapids          = viewport.getShowRapids();
        saved_settings.ui.show_pierces         = viewport.getShowPierces();
        saved_settings.ui.follow_torch         = viewport.getFollowTorch();
        saved_settings.ui.minimal_layout       = minimal_layout;

        saved_settings.ui.step_jog_mode        = dro_panel.isStepJogMode();
        saved_settings.ui.step_size_mm         = dro_panel.getStepSizeMm();
        saved_settings.ui.jog_speed_pct        = dro_panel.getJogSpeedPct();
        saved_settings.ui.dro_jog_height       = s_dro_user_h;
        saved_settings.ui.last_gcode_file_path = persisted_file_path;

        saved_settings.ui.has_saved_coordinates = true;
        for (size_t i = 0; i < 4; ++i) {
            saved_settings.ui.last_machine_pos_mm[i] = tel.machine_pos_mm[i];
            // WCS offset without the tool length offset (that is restored separately)
            saved_settings.ui.last_work_offset_mm[i] = tel.workspace_offsets_mm[tel.active_workspace][i];
            saved_settings.ui.last_work_pos_mm[i]    = tel.work_pos_mm[i];
        }
        saved_settings.ui.last_tool_in_spindle = tel.current_tool_number;
        saved_settings.ui.last_tlo_tool        = tel.active_tlo_tool;

        config::SettingsManager::saveToFile(settings_file, saved_settings);
    };

    // Connect FULL / MINIMAL layout switcher to the custom title bar
    window.setMinimalLayout(&minimal_layout, [&](bool is_min) {
        saved_settings.ui.minimal_layout = is_min;
        markSettingsDirty();
    });

    // Auto-save viewport settings when toggled
    viewport.setOnViewChanged([&]() {
        saved_settings.ui.isometric_3d = viewport.isIsometric3D();
        saved_settings.ui.show_rapids  = viewport.getShowRapids();
        saved_settings.ui.show_pierces = viewport.getShowPierces();
        saved_settings.ui.follow_torch = viewport.getFollowTorch();
        markSettingsDirty();
    });

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

    int exit_code = window.runLoop([&]() {
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

        const auto telemetry    = controller.getTelemetry();
        const auto program_snapshot = controller.getProgram(); // Held for the whole frame
        const auto& parse_res   = *program_snapshot;
        const auto& segments    = program_snapshot->segments;

        camera_mgr.updateLiveTelemetry(telemetry, controller.getConfig());
        stream_server.updateTelemetry(telemetry, controller.getConfig().linear_units, "BreakoutMill", loaded_filename);

        saved_settings.ui.minimal_layout = minimal_layout;

        // Note: Top header rows eliminated! Viewport and right column go directly up against the title bar.
        const float spacing_x = ImGui::GetStyle().ItemSpacing.x;
        const float spacing_y = ImGui::GetStyle().ItemSpacing.y;
        const float workspace_h = ImGui::GetContentRegionAvail().y - 40.0f - spacing_y;

        const float right_col_w = minimal_layout ? 0.0f   : 450.0f;
        const float left_col_w  = minimal_layout ? 325.0f : 450.0f;

        // 1. Left Column: Sticky DRO/Jog Panel & Bottom Controller State Badge / E-Stop Card
        // Sized to exactly workspace_h with NoScrollbar & NoScrollWithMouse so it NEVER scrolls
        ImGui::BeginChild("##MillLeftCol", ImVec2(left_col_w, workspace_h), false,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        {
            // State badge header and CYCLE START/E-STOP row (110px)
            const float estop_card_h = 110.0f;
            const float dro_h = std::max(100.0f, workspace_h - estop_card_h - spacing_y - 2.0f);

            // DRO and Manual Jog Control Panel (top of left panel)
            ImGui::BeginChild("##MillDroJogFixed", ImVec2(0.0f, dro_h), true,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            dro_panel.render(controller, telemetry, true, dro_h); // Show 4th Axis (A) for Mill
            ImGui::EndChild();

            // Pin Controller State Badge, Cycle Control, and Emergency Stop / Reset Card to the bottom
            ImGui::SetCursorPosY(workspace_h - estop_card_h);
            ui::IoStatusBar::renderStatusAndEStopCard(controller, telemetry, estop_card_h, &gcode_panel);
        }
        ImGui::EndChild(); // ##MillLeftCol

        ImGui::SameLine();

        const float center_col_w = minimal_layout
                                       ? ImGui::GetContentRegionAvail().x
                                       : std::max(200.0f, ImGui::GetContentRegionAvail().x - right_col_w - spacing_x);

        static bool s_mill_lua_open = saved_settings.ui.header_lua_open;

        // 2. Center Column: Toolpath Viewport + G-Code Panel + Lua Scripting Deck (Full View Only)
        ImGui::BeginChild("##MillCenterCol", ImVec2(center_col_w, workspace_h), false);
        {
            const bool is_tool_change = (telemetry.state == core::ControllerState::ToolChangeHold ||
                                         telemetry.pending_tool_number > 0);
            const float lua_reserved_h = minimal_layout
                                             ? 0.0f
                                             : (s_mill_lua_open ? 260.0f : (ImGui::GetFrameHeightWithSpacing() + 6.0f));

            // Compute exact snug height for G-Code & MDI card so there is zero unused space below MDI
            // In Minimal view, user requested exactly 8 lines of G-Code visible
            float gcode_h;
            if (minimal_layout) {
                gcode_h = gcode_panel.calculateDesiredHeight(0.0f, is_tool_change, 8);
            } else {
                const float target_gcode_ratio = s_mill_lua_open ? 0.28f : 0.38f;
                const float max_gcode_h = std::clamp(workspace_h * target_gcode_ratio, 160.0f, workspace_h - 220.0f);
                gcode_h = gcode_panel.calculateDesiredHeight(max_gcode_h, is_tool_change);
            }

            // Viewport takes all remaining vertical space up to the top title bar
            const float viewport_h = minimal_layout
                                         ? std::max(120.0f, workspace_h - gcode_h - spacing_y)
                                         : std::max(120.0f, workspace_h - gcode_h - lua_reserved_h - spacing_y * 2.0f);

            ImGui::BeginChild("##MillViewport", ImVec2(0.0f, viewport_h), true);
            viewport.render(segments, telemetry, parse_res.bbox_min_mm, parse_res.bbox_max_mm, &controller);
            ImGui::EndChild();

            ImGui::BeginChild("##MillGCodeCard", ImVec2(0.0f, gcode_h), true);
            gcode_panel.render(
                controller,
                telemetry,
                loaded_filename,
                parse_res.pierce_count,
                parse_res.total_cut_length_mm,
                [&](const std::string& path, const std::string& content) {
                    if (controller.loadGCodeString(content)) {
                        loaded_filename     = path;
                        persisted_file_path = path;
                        const auto pr_ptr = controller.getProgram();
                        const auto& pr = *pr_ptr;
                        viewport.fitToBounds(pr.bbox_min_mm, pr.bbox_max_mm);
                        markSettingsDirty();
                    }
                });
            ImGui::EndChild();

            // In MINIMAL view, Lua Scripting is completely removed
            if (!minimal_layout) {
                const float lua_card_h = s_mill_lua_open ? 0.0f : lua_reserved_h;
                ImGui::BeginChild("##MillLuaCard", ImVec2(0.0f, lua_card_h), true, s_mill_lua_open ? 0 : ImGuiWindowFlags_NoScrollbar);
                ImGui::SetNextItemAllowOverlap();
                ImGui::SetNextItemOpen(saved_settings.ui.header_lua_open, ImGuiCond_Once);
                bool lua_open = ImGui::CollapsingHeader("Lua Scripting & User Macros", ImGuiTreeNodeFlags_AllowOverlap);

                // Information sign icon button on the right side of the header bar
                const ImVec2 hdr_min = ImGui::GetItemRectMin();
                const ImVec2 hdr_max = ImGui::GetItemRectMax();
                const float icon_sz = 20.0f;
                const ImVec2 icon_pos(hdr_max.x - icon_sz - 12.0f, hdr_min.y + (hdr_max.y - hdr_min.y - icon_sz) * 0.5f);
                if (ui::ScriptingPanel::renderHelpIconButton("##MillLuaHelpBtn", icon_pos, icon_sz)) {
                    scripting_panel.openHelpModal();
                    lua_open = s_mill_lua_open; // Prevent header from collapsing when info icon is clicked
                }

                if (lua_open != saved_settings.ui.header_lua_open) {
                    saved_settings.ui.header_lua_open = lua_open;
                    markSettingsDirty();
                }
                s_mill_lua_open = lua_open;
                if (lua_open) {
                    core::MachineConfig cur_cfg = controller.getConfig();
                    scripting_panel.render(controller, cur_cfg, [&]() {
                        saved_settings.machine = cur_cfg;
                        markSettingsDirty();
                    });
                } else {
                    ImGui::Dummy(ImVec2(0.0f, 0.0f));
                }
                ImGui::EndChild();
            }
        }
        ImGui::EndChild(); // ##MillCenterCol

        // 3. Right Column: Scrollable Settings Area (Full View Only - Completely Hidden in Minimal View)
        if (!minimal_layout) {
            ImGui::SameLine();

            ImGui::BeginChild("##MillRightCol", ImVec2(right_col_w, workspace_h), false, ImGuiWindowFlags_NoScrollbar);
            {
                ImGui::BeginChild("##MillSettingsScroll", ImVec2(0.0f, 0.0f), true);
                {
                    ImGui::SetNextItemOpen(saved_settings.ui.header_spindle_open, ImGuiCond_Once);
                    bool sp_hdr_open = ImGui::CollapsingHeader("Spindle (0-10V PWM) & Feed Control");
                    if (sp_hdr_open != saved_settings.ui.header_spindle_open) {
                        saved_settings.ui.header_spindle_open = sp_hdr_open;
                        markSettingsDirty();
                    }
                    if (sp_hdr_open) {
                        core::MachineConfig sp_cfg = controller.getConfig();
                        bool sp_cfg_changed = false;
                        const float spacing = ImGui::GetStyle().ItemSpacing.x;

                        // =================================================================
                        // 1. SPINDLE SUBSYSTEM CARD
                        // =================================================================
                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));

                        ImGui::BeginChild("##SpindleSubCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            // Card Header Strip
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "SPINDLE CONTROL");
                            ImGui::SameLine();
                            if (sp_cfg.vfd.enabled) {
                                if (telemetry.vfd_telemetry.connected) {
                                    ImGui::TextColored(ImVec4(0.20f, 0.90f, 0.40f, 1.0f), "[RS485 VFD ONLINE]");
                                } else {
                                    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "[VFD OFFLINE]");
                                }
                            } else {
                                ImGui::TextColored(ImVec4(0.65f, 0.70f, 0.78f, 0.8f), "[0-10V ANALOG]");
                            }
                            ImGui::Spacing();

                            // Main Action Button (38px height)
                            bool spindle_on = telemetry.torch_or_spindle_on;
                            ImVec4 sp_col;
                            ImVec4 sp_col_hov;
                            ImVec4 sp_col_act;
                            char sp_btn_lbl[96];

                            if (telemetry.spindle_ramping_up) {
                                sp_col     = ImVec4(0.85f, 0.50f, 0.08f, 1.0f);
                                sp_col_hov = ImVec4(0.95f, 0.60f, 0.15f, 1.0f);
                                sp_col_act = ImVec4(0.70f, 0.40f, 0.05f, 1.0f);
                                std::snprintf(sp_btn_lbl, sizeof(sp_btn_lbl),
                                              "SPINDLE RAMPING UP (%3.0f%%) - HOLDING CUT UNTIL 100%%",
                                              telemetry.spindle_speed_pct);
                            } else if (telemetry.spindle_ramping_down) {
                                sp_col     = ImVec4(0.75f, 0.35f, 0.10f, 1.0f);
                                sp_col_hov = ImVec4(0.85f, 0.45f, 0.15f, 1.0f);
                                sp_col_act = ImVec4(0.60f, 0.25f, 0.05f, 1.0f);
                                std::snprintf(sp_btn_lbl, sizeof(sp_btn_lbl),
                                              "SPINDLE RAMPING DOWN (%3.0f%%)",
                                              telemetry.spindle_speed_pct);
                            } else if (spindle_on) {
                                sp_col     = ImVec4(0.12f, 0.65f, 0.35f, 1.0f);
                                sp_col_hov = ImVec4(0.16f, 0.75f, 0.42f, 1.0f);
                                sp_col_act = ImVec4(0.08f, 0.50f, 0.25f, 1.0f);
                                std::snprintf(sp_btn_lbl, sizeof(sp_btn_lbl),
                                              "SPINDLE RUNNING (%d RPM) - CLICK TO STOP",
                                              spindle_rpm);
                            } else {
                                sp_col     = ImVec4(0.18f, 0.23f, 0.30f, 1.0f);
                                sp_col_hov = ImVec4(0.24f, 0.31f, 0.40f, 1.0f);
                                sp_col_act = ImVec4(0.14f, 0.18f, 0.24f, 1.0f);
                                std::snprintf(sp_btn_lbl, sizeof(sp_btn_lbl), "START SPINDLE (M3)");
                            }

                            ImGui::PushStyleColor(ImGuiCol_Button, sp_col);
                            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, sp_col_hov);
                            ImGui::PushStyleColor(ImGuiCol_ButtonActive, sp_col_act);
                            if (ImGui::Button(sp_btn_lbl, ImVec2(-1.0f, 38.0f))) {
                                controller.setTorchOrSpindle(!spindle_on);
                            }
                            ImGui::PopStyleColor(3);

                            if (sp_cfg.vfd.enabled && !telemetry.vfd_telemetry.connected) {
                                ImGui::TextColored(ImVec4(1.0f, 0.40f, 0.40f, 1.0f),
                                                   " [!] VFD OFFLINE: Spindle disabled until RS485 connects.");
                            }

                            // Target Speed Row with Nudge buttons & Slider
                            static bool s_is_dragging_spindle_rpm = false;
                            if (!s_is_dragging_spindle_rpm && !telemetry.spindle_override_active && telemetry.commanded_spindle_rpm > 0) {
                                spindle_rpm = static_cast<int>(telemetry.commanded_spindle_rpm);
                            }

                            const int min_slider_rpm = (sp_cfg.vfd.enabled && sp_cfg.vfd.min_rpm > 0) ? static_cast<int>(sp_cfg.vfd.min_rpm) : 1000;
                            const int max_slider_rpm = (sp_cfg.vfd.enabled && sp_cfg.vfd.max_rpm > 0) ? static_cast<int>(sp_cfg.vfd.max_rpm) : 24000;
                            const bool sp_overridden = telemetry.spindle_override_active;

                            ImGui::Text("COMMANDED SPEED:");
                            ImGui::SameLine();
                            if (sp_overridden) {
                                ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.15f, 1.0f), "%d RPM (Override Active)", spindle_rpm);
                            } else {
                                ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.00f, 1.0f), "%d RPM", spindle_rpm);
                            }

                            const float btn_w = 48.0f;
                            const float card_inner_w = ImGui::GetContentRegionAvail().x;
                            const float slider_w = std::max(60.0f, card_inner_w - (btn_w * 3.0f + spacing * 3.0f));

                            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5f, 0.5f));

                            if (sp_overridden) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.40f, 0.22f, 0.05f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.55f, 0.30f, 0.08f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.70f, 0.20f, 1.0f));
                            }
                            if (ImGui::Button("Reset##Spindle", ImVec2(btn_w, 28.0f))) {
                                controller.resetSpindleSpeedOverride();
                                spindle_rpm = static_cast<int>(telemetry.program_spindle_rpm);
                                markSettingsDirty();
                            }
                            if (sp_overridden) {
                                ImGui::PopStyleColor(3);
                            }
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Reset spindle speed to programmed G-code RPM (%d RPM)",
                                                  static_cast<int>(telemetry.program_spindle_rpm));
                            }

                            ImGui::SameLine();
                            if (ImGui::Button("-500##Spindle", ImVec2(btn_w, 28.0f))) {
                                spindle_rpm = std::clamp(spindle_rpm - 500, min_slider_rpm, max_slider_rpm);
                                controller.setSpindleSpeedRpm(static_cast<uint32_t>(spindle_rpm));
                                markSettingsDirty();
                            }

                            ImGui::PopStyleVar(2);

                            ImGui::SameLine();
                            if (sp_overridden) {
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.60f, 0.15f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(1.0f, 0.60f, 0.15f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(1.0f, 0.75f, 0.25f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.30f, 0.18f, 0.05f, 1.0f));
                            }
                            ImGui::SetNextItemWidth(slider_w);
                            if (ImGui::SliderInt("##SpindleRpmSlider", &spindle_rpm, min_slider_rpm, max_slider_rpm, "%d RPM")) {
                                controller.setSpindleSpeedRpm(static_cast<uint32_t>(spindle_rpm));
                                markSettingsDirty();
                            }
                            if (ImGui::IsItemActive()) s_is_dragging_spindle_rpm = true;
                            if (ImGui::IsItemDeactivated()) s_is_dragging_spindle_rpm = false;
                            if (sp_overridden) ImGui::PopStyleColor(4);

                            ImGui::SameLine();
                            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5f, 0.5f));
                            if (ImGui::Button("+500##Spindle", ImVec2(btn_w, 28.0f))) {
                                spindle_rpm = std::clamp(spindle_rpm + 500, min_slider_rpm, max_slider_rpm);
                                controller.setSpindleSpeedRpm(static_cast<uint32_t>(spindle_rpm));
                                markSettingsDirty();
                            }
                            ImGui::PopStyleVar(2);

                            ImGui::Spacing();

                            // Collapsible Spindle Acceleration & Ramp Tuning
                            char ramp_header[96];
                            std::snprintf(ramp_header, sizeof(ramp_header),
                                          "Acceleration & Ramp Times (%.1fs Up / %.1fs Down)###RampTuning",
                                          sp_cfg.spindle_ramp_up_s, sp_cfg.spindle_ramp_down_s);
                            if (ImGui::TreeNodeEx(ramp_header, ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_NoTreePushOnOpen)) {
                                float ramp_up   = static_cast<float>(sp_cfg.spindle_ramp_up_s);
                                float ramp_down = static_cast<float>(sp_cfg.spindle_ramp_down_s);
                                const float avail_w = ImGui::GetContentRegionAvail().x;
                                const float col_w   = (avail_w - spacing) * 0.5f;

                                // 1. Ramp-Up (Hold Cut)
                                const float up_lbl_w = ImGui::CalcTextSize("Ramp-Up:").x;
                                const float up_slider_w = std::max(50.0f, col_w - up_lbl_w - spacing);
                                ImGui::AlignTextToFramePadding();
                                ImGui::TextDisabled("Ramp-Up:");
                                ImGui::SameLine();
                                ImGui::SetNextItemWidth(up_slider_w);
                                if (ImGui::SliderFloat("##RampUp", &ramp_up, 0.0f, 30.0f, "%.1f s")) {
                                    sp_cfg.spindle_ramp_up_s = ramp_up;
                                    sp_cfg_changed = true;
                                }

                                ImGui::SameLine();

                                // 2. Ramp-Down (Decel)
                                const float down_lbl_w = ImGui::CalcTextSize("Ramp-Down:").x;
                                const float down_slider_w = std::max(50.0f, avail_w - (col_w + spacing) - down_lbl_w - spacing);
                                ImGui::AlignTextToFramePadding();
                                ImGui::TextDisabled("Ramp-Down:");
                                ImGui::SameLine();
                                ImGui::SetNextItemWidth(down_slider_w);
                                if (ImGui::SliderFloat("##RampDown", &ramp_down, 0.0f, 30.0f, "%.1f s")) {
                                    sp_cfg.spindle_ramp_down_s = ramp_down;
                                    sp_cfg_changed = true;
                                }
                            }

                            if (sp_cfg_changed) {
                                controller.applyConfig(sp_cfg);
                                markSettingsDirty();
                            }
                        }
                        ImGui::EndChild();
                        ImGui::PopStyleVar(3);
                        ImGui::PopStyleColor(2);

                        ImGui::Spacing();

                        // =================================================================
                        // 2. FEED RATE & CUTTING OVERRIDE CARD
                        // =================================================================
                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                        ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));

                        ImGui::BeginChild("##FeedSubCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "FEED RATE & CUTTING OVERRIDE");
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
                            const float right_btn_w  = feed_inner_w - left_dro_w - spacing;
                            const float dro_box_h    = 58.0f;

                            // 1. Left DRO Box
                            ImVec2 box_p0 = ImGui::GetCursorScreenPos();
                            ImVec2 box_p1(box_p0.x + left_dro_w, box_p0.y + dro_box_h);
                            ImDrawList* dl = ImGui::GetWindowDrawList();

                            dl->AddRectFilled(box_p0, box_p1, IM_COL32(8, 11, 15, 255), 4.0f);
                            dl->AddRect(box_p0, box_p1,
                                        is_moving ? IM_COL32(0, 200, 120, 220) : IM_COL32(45, 62, 84, 220),
                                        4.0f, 0, 1.2f);

                            ImFont* gfont = ui::AppWindow::getGCodeFont();
                            float fsz_lbl = 11.0f;
                            float fsz_val = 14.0f;

                            const bool is_imperial_feed = (controller.getConfig().linear_units == core::LinearUnits::Imperial);
                            const double feed_scale = is_imperial_feed ? (1.0 / 25.4) : 1.0;
                            const char* feed_unit_lbl = is_imperial_feed ? "IPM" : "mm/m";
                            const char* feed_sec_lbl = is_imperial_feed ? "in/s" : "mm/s";

                            // Line 1: ACT FEED
                            float line1_y = box_p0.y + 7.0f;
                            dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line1_y + 2.0f),
                                        is_moving ? IM_COL32(140, 245, 180, 255) : IM_COL32(110, 135, 160, 230),
                                        "ACT FEED");

                            char act_feed_str[32];
                            std::snprintf(act_feed_str, sizeof(act_feed_str), "%4.0f %s", act_feed_mm_min * feed_scale, feed_unit_lbl);
                            ImVec2 act_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, act_feed_str) : ImGui::CalcTextSize(act_feed_str);
                            dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - act_sz.x - 7.0f, line1_y),
                                        is_moving ? IM_COL32(0, 240, 150, 255) : IM_COL32(115, 155, 195, 230),
                                        act_feed_str);

                            // Line 2: CMD (F)
                            float line2_y = box_p0.y + 31.0f;
                            dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line2_y + 2.0f),
                                        IM_COL32(100, 120, 145, 200),
                                        "CMD (F)");

                            char cmd_feed_str[32];
                            std::snprintf(cmd_feed_str, sizeof(cmd_feed_str), "%4.0f %s", telemetry.commanded_feed_mm_min * feed_scale, feed_unit_lbl);
                            ImVec2 cmd_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, cmd_feed_str) : ImGui::CalcTextSize(cmd_feed_str);
                            dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - cmd_sz.x - 7.0f, line2_y),
                                        IM_COL32(80, 175, 245, 220),
                                        cmd_feed_str);

                            ImGui::Dummy(ImVec2(left_dro_w, dro_box_h));
                            if (ImGui::IsItemHovered()) {
                                ImGui::BeginTooltip();
                                ImGui::Text("Actual Tool Speed: %.0f %s (%.1f %s)",
                                            act_feed_mm_min * feed_scale, feed_unit_lbl,
                                            (act_feed_mm_min / 60.0) * feed_scale, feed_sec_lbl);
                                ImGui::Text("Commanded Feed (F): %.0f %s (%.1f %s)",
                                            telemetry.commanded_feed_mm_min * feed_scale, feed_unit_lbl,
                                            (telemetry.commanded_feed_mm_min / 60.0) * feed_scale, feed_sec_lbl);
                                ImGui::Separator();
                                ImGui::Text("Axis Speeds (%s): X: %.0f | Y: %.0f | Z: %.0f | A: %.0f",
                                            feed_unit_lbl,
                                            telemetry.axis_vel_mm_min[0] * feed_scale,
                                            telemetry.axis_vel_mm_min[1] * feed_scale,
                                            telemetry.axis_vel_mm_min[2] * feed_scale,
                                            telemetry.axis_vel_mm_min[3] * feed_scale);
                                core::MachineConfig mcfg = controller.getConfig();
                                ImGui::Text("Motor Max Limits (%s): X: %.0f | Y: %.0f | Z: %.0f",
                                            feed_unit_lbl,
                                            mcfg.axes[0].max_velocity_mm_s * 60.0 * feed_scale,
                                            mcfg.axes[1].max_velocity_mm_s * 60.0 * feed_scale,
                                            mcfg.axes[2].max_velocity_mm_s * 60.0 * feed_scale);
                                ImGui::Separator();
                                ImGui::TextDisabled("Calculation: 3D vector magnitude sqrt(vx^2 + vy^2 + vz^2)");
                                ImGui::EndTooltip();
                            }

                            ImGui::SameLine();

                            // 2. Right Presets Grid (2 rows of 3 buttons)
                            static float s_drag_feed_ov = 100.0f;
                            static bool s_is_dragging_feed = false;
                            if (!s_is_dragging_feed) {
                                s_drag_feed_ov = telemetry.feed_override_pct;
                            }

                            ImGui::BeginGroup();
                            {
                                const float pbtn_w = (right_btn_w - spacing * 2.0f) / 3.0f;
                                const float pbtn_h = std::floor((dro_box_h - ImGui::GetStyle().ItemSpacing.y) * 0.5f);

                                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
                                ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5f, 0.5f));

                                // Row 1: -10%, 100% Reset, +10%
                                if (ImGui::Button("-10%##Feed", ImVec2(pbtn_w, pbtn_h))) {
                                    float new_ov = std::clamp(std::round((telemetry.feed_override_pct - 10.0f) / 5.0f) * 5.0f, 20.0f, 200.0f);
                                    controller.setFeedOverride(new_ov);
                                    s_drag_feed_ov = new_ov;
                                    markSettingsDirty();
                                }
                                ImGui::SameLine();

                                const bool feed_overridden = (std::abs(telemetry.feed_override_pct - 100.0f) >= 0.5f);
                                if (feed_overridden) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.40f, 0.22f, 0.05f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.55f, 0.30f, 0.08f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.70f, 0.20f, 1.0f));
                                }
                                if (ImGui::Button("100% (1:1)##Feed", ImVec2(pbtn_w, pbtn_h))) {
                                    controller.setFeedOverride(100.0f);
                                    s_drag_feed_ov = 100.0f;
                                    markSettingsDirty();
                                }
                                if (feed_overridden) {
                                    ImGui::PopStyleColor(3);
                                }
                                ImGui::SameLine();

                                if (ImGui::Button("+10%##Feed", ImVec2(pbtn_w, pbtn_h))) {
                                    float new_ov = std::clamp(std::round((telemetry.feed_override_pct + 10.0f) / 5.0f) * 5.0f, 20.0f, 200.0f);
                                    controller.setFeedOverride(new_ov);
                                    s_drag_feed_ov = new_ov;
                                    markSettingsDirty();
                                }

                                // Row 2: 50%, 150%, 200%
                                const float preset_vals[3] = {50.0f, 150.0f, 200.0f};
                                const char* preset_lbls[3] = {"50%##Feed", "150%##Feed", "200%##Feed"};
                                for (int i = 0; i < 3; ++i) {
                                    if (i > 0) ImGui::SameLine();
                                    bool is_active_preset = (std::abs(telemetry.feed_override_pct - preset_vals[i]) < 1.0f);
                                    if (is_active_preset) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.52f, 0.72f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    }
                                    if (ImGui::Button(preset_lbls[i], ImVec2(pbtn_w, pbtn_h))) {
                                        controller.setFeedOverride(preset_vals[i]);
                                        s_drag_feed_ov = preset_vals[i];
                                        markSettingsDirty();
                                    }
                                    if (is_active_preset) {
                                        ImGui::PopStyleColor(2);
                                    }
                                }

                                ImGui::PopStyleVar(2);
                            }
                            ImGui::EndGroup();

                            ImGui::Spacing();

                            // 3. Continuous Slider
                            const bool feed_overridden = (std::abs(telemetry.feed_override_pct - 100.0f) >= 0.5f);
                            ImGui::Text("FEED OVERRIDE:");
                            ImGui::SameLine();
                            if (feed_overridden) {
                                ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.15f, 1.0f), "%.0f%% (Modified)", telemetry.feed_override_pct);
                            } else {
                                ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.00f, 1.0f), "100%% (Standard 1:1)");
                            }

                            if (feed_overridden) {
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.60f, 0.15f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(1.0f, 0.60f, 0.15f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(1.0f, 0.75f, 0.25f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.30f, 0.18f, 0.05f, 1.0f));
                            }

                            ImGui::SetNextItemWidth(-1.0f);
                            if (ImGui::SliderFloat("##FeedOverrideSlider", &s_drag_feed_ov, 20.0f, 200.0f, "%.0f%%")) {
                                if (std::abs(s_drag_feed_ov - telemetry.feed_override_pct) >= 1.0f) {
                                    controller.setFeedOverride(s_drag_feed_ov);
                                    markSettingsDirty();
                                }
                            }
                            if (ImGui::IsItemActive()) s_is_dragging_feed = true;
                            if (ImGui::IsItemDeactivated()) {
                                s_is_dragging_feed = false;
                                controller.setFeedOverride(s_drag_feed_ov);
                                markSettingsDirty();
                            }

                            if (feed_overridden) {
                                ImGui::PopStyleColor(4);
                            }
                        }
                        ImGui::EndChild();
                        ImGui::PopStyleVar(3);
                        ImGui::PopStyleColor(2);
                    } // if CollapsingHeader Spindle (0-10V PWM) & Feed Control

            // -----------------------------------------------------------------
            // USB MOTION CONTROLLER & CONNECTION (RNR ECO MOTION 2.0)
            // Positioned directly above Huanyang VFD settings
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

            // -----------------------------------------------------------------
            // HUANYANG VFD (RS485 SPINDLE INVERTER) - Positioned directly below USB Connection
            // -----------------------------------------------------------------
            ImGui::SetNextItemOpen(saved_settings.ui.header_vfd_open, ImGuiCond_Once);
            bool vfd_hdr_open = ImGui::CollapsingHeader("Huanyang VFD (RS485 Spindle Inverter)");
            if (vfd_hdr_open != saved_settings.ui.header_vfd_open) {
                saved_settings.ui.header_vfd_open = vfd_hdr_open;
                markSettingsDirty();
            }
            if (vfd_hdr_open) {
                core::MachineConfig cfg = controller.getConfig();
                bool vfd_changed = false;
                auto& vfd = cfg.vfd;
                const auto& vt = telemetry.vfd_telemetry;
                const float spacing = ImGui::GetStyle().ItemSpacing.x;

                if (ImGui::Checkbox("Enable Huanyang RS485 VFD Control", &vfd.enabled)) {
                    vfd_changed = true;
                }

                ImGui::SameLine();
                if (!vfd.enabled) {
                    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "[DISABLED (PWM ACTIVE)]");
                } else if (vt.connected) {
                    ImGui::TextColored(ImVec4(0.15f, 0.95f, 0.35f, 1.0f), "[RS485 CONNECTED]");
                } else {
                    ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.15f, 1.0f), "[OFFLINE / CONNECTING...]");
                }

                if (vfd.enabled) {
                    ImGui::Spacing();

                    // Push common sub-card styling
                    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                    ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                    // =========================================================
                    // 1. SERIAL COMMUNICATION CARD (Port & Baud Rate)
                    // =========================================================
                    ImGui::BeginChild("##VfdSerialCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                    {
                        ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "RS485 SERIAL CONNECTION");
                        ImGui::Spacing();

                        static std::vector<std::string> s_com_ports;
                        static float s_last_port_scan_time = -10.0f;
                        float cur_port_time = static_cast<float>(ImGui::GetTime());
                        if (s_com_ports.empty() || cur_port_time - s_last_port_scan_time > 5.0f) {
                            s_com_ports = HuanyangVfdController::enumeratePorts();
                            s_last_port_scan_time = cur_port_time;
                        }

                        int cur_port_idx = -1;
                        std::vector<const char*> port_items;
                        for (size_t i = 0; i < s_com_ports.size(); ++i) {
                            port_items.push_back(s_com_ports[i].c_str());
                            if (s_com_ports[i] == vfd.port_name) {
                                cur_port_idx = static_cast<int>(i);
                            }
                        }

                        // Row 1: COM Port selector + Refresh button
                        const float refresh_btn_w = 75.0f;
                        const float port_w = std::max(100.0f, ImGui::GetContentRegionAvail().x - refresh_btn_w - spacing);
                        ImGui::SetNextItemWidth(port_w);
                        if (cur_port_idx >= 0) {
                            if (ImGui::Combo("##ComPortCombo", &cur_port_idx, port_items.data(), static_cast<int>(port_items.size()))) {
                                vfd.port_name = s_com_ports[cur_port_idx];
                                vfd_changed = true;
                            }
                        } else {
                            char port_buf[128];
                            std::strncpy(port_buf, vfd.port_name.c_str(), sizeof(port_buf) - 1);
                            port_buf[sizeof(port_buf) - 1] = '\0';
                            if (ImGui::InputText("##ComPortInput", port_buf, sizeof(port_buf))) {
                                vfd.port_name = port_buf;
                                vfd_changed = true;
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Refresh", ImVec2(refresh_btn_w, 0.0f))) {
                            s_com_ports = HuanyangVfdController::enumeratePorts();
                            s_last_port_scan_time = cur_port_time;
                        }

                        // Row 2: Baud Rate selector (PD164)
                        const char* baud_rates[] = { "9600 Baud (PD164=1)", "19200 Baud (PD164=2)", "38400 Baud (PD164=3)" };
                        int baud_idx = (vfd.baud_rate == 38400) ? 2 : ((vfd.baud_rate == 19200) ? 1 : 0);
                        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                        if (ImGui::Combo("##BaudRateCombo", &baud_idx, baud_rates, 3)) {
                            vfd.baud_rate = (baud_idx == 2) ? 38400 : ((baud_idx == 1) ? 19200 : 9600);
                            vfd_changed = true;
                        }
                    }
                    ImGui::EndChild();

                    ImGui::Spacing();

                    // =========================================================
                    // 2. INVERTER PARAMETERS CARD (Slave ID, Freq, RPM Limits)
                    // =========================================================
                    ImGui::BeginChild("##VfdParamsCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                    {
                        ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "INVERTER PARAMETERS & SPEED LIMITS");
                        ImGui::Spacing();

                        int slave = static_cast<int>(vfd.slave_address);
                        ImGui::SetNextItemWidth(140.0f);
                        if (ImGui::InputInt("Slave ID (PD163)", &slave, 1, 1)) {
                            vfd.slave_address = static_cast<uint8_t>(std::clamp(slave, 1, 32));
                            vfd_changed = true;
                        }

                        float max_freq = vfd.max_freq_hz;
                        ImGui::SetNextItemWidth(140.0f);
                        if (ImGui::InputFloat("Max Freq (Hz, PD005)", &max_freq, 10.0f, 50.0f, "%.1f Hz")) {
                            vfd.max_freq_hz = std::clamp(max_freq, 50.0f, 1000.0f);
                            vfd_changed = true;
                        }

                        int min_rpm_val = static_cast<int>(vfd.min_rpm);
                        ImGui::SetNextItemWidth(140.0f);
                        if (ImGui::InputInt("Min Spindle RPM", &min_rpm_val, 500, 2000)) {
                            vfd.min_rpm = static_cast<uint16_t>(std::clamp(min_rpm_val, 0, static_cast<int>(vfd.max_rpm)));
                            vfd_changed = true;
                        }

                        int max_rpm_val = static_cast<int>(vfd.max_rpm);
                        ImGui::SetNextItemWidth(140.0f);
                        if (ImGui::InputInt("Max Spindle RPM", &max_rpm_val, 1000, 5000)) {
                            vfd.max_rpm = static_cast<uint16_t>(std::clamp(max_rpm_val, 1000, 60000));
                            vfd_changed = true;
                        }

                        // Read PD001/PD002/PD005/PD011/PD144 from the inverter and offer them as the speed scaling
                        ImGui::Spacing();
                        const auto& vp = vt.params;
                        const char* read_lbl = vp.busy ? "Reading from VFD..." : "Read Parameters from VFD";
                        ImGui::BeginDisabled(vp.busy || !vt.connected);
                        if (ImGui::Button(read_lbl, ImVec2(ImGui::CalcTextSize("Read Parameters from VFD").x + 32.0f, 28.0f))) {
                            controller.requestVfdParameterRead();
                        }
                        ImGui::EndDisabled();
                        ImGui::PushTextWrapPos(0.0f);
                        if (!vp.error.empty()) {
                            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "%s", vp.error.c_str());
                        } else if (vp.valid) {
                            ImGui::Text("PD005 max %.2f Hz, PD011 min %.2f Hz, PD144 %u RPM at 50 Hz", vp.pd005_max_hz,
                                        vp.pd011_min_hz, static_cast<unsigned>(vp.pd144_rpm_at_50hz));
                            ImGui::Text("Spindle speed range: %u .. %u RPM", static_cast<unsigned>(vp.minRpm()),
                                        static_cast<unsigned>(vp.maxRpm()));
                            if (!vp.rs485Controlled()) {
                                ImGui::TextColored(ImVec4(1.0f, 0.70f, 0.20f, 1.0f),
                                                   "PD001 = %d and PD002 = %d: set both to 2 so the VFD takes run "
                                                   "and frequency commands over RS485.",
                                                   vp.pd001_run_source, vp.pd002_freq_source);
                            }
                            const bool usable = vp.pd005_max_hz > 0.0f && vp.maxRpm() >= 1000;
                            ImGui::BeginDisabled(!usable);
                            if (ImGui::Button("Use These Values", ImVec2(ImGui::CalcTextSize("Use These Values").x + 32.0f, 28.0f))) {
                                vfd.max_freq_hz = vp.pd005_max_hz;
                                vfd.max_rpm     = vp.maxRpm();
                                vfd.min_rpm     = std::min(vp.minRpm(), vfd.max_rpm);
                                vfd_changed = true;
                            }
                            ImGui::EndDisabled();
                        } else if (!vt.connected) {
                            ImGui::TextDisabled("Connect the VFD to read PD005 (max frequency) and PD144 (rated speed).");
                        }
                        ImGui::PopTextWrapPos();
                    }
                    ImGui::EndChild();

                    ImGui::Spacing();

                    // =========================================================
                    // 3. SAFETY & PROTECTION INTERLOCKS CARD
                    // =========================================================
                    ImGui::BeginChild("##VfdSafetyCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                    {
                        ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "SAFETY INTERLOCKS & PROTECTION");
                        ImGui::Spacing();

                        if (ImGui::Checkbox("E-Stop on Spindle Stall (0 RPM detected while running)", &vfd.estop_on_stall)) {
                            vfd_changed = true;
                        }
                        if (ImGui::IsItemHovered()) {
                            ImGui::SetTooltip("If the VFD reports 0 RPM while the spindle is commanded to spin, automatically triggers Emergency Stop.");
                        }

                        if (ImGui::Checkbox("E-Stop on RS485 Communication Loss", &vfd.estop_on_comm_loss)) {
                            vfd_changed = true;
                        }
                        if (ImGui::IsItemHovered()) {
                            ImGui::SetTooltip("If the VFD stops answering for more than 3 seconds while the spindle runs, automatically triggers Emergency Stop.");
                        }

                        // Speed monitoring (same rules as the Mach3 vfdPlugin "Automatic EStop")
                        ImGui::Spacing();
                        ImGui::Separator();
                        ImGui::Spacing();
                        if (ImGui::Checkbox("Speed Monitoring E-Stop", &vfd.speed_monitor_enabled)) {
                            vfd_changed = true;
                        }
                        if (vfd.speed_monitor_enabled) {
                            ImGui::PushTextWrapPos(0.0f);
                            ImGui::TextDisabled("While the speed changes it must get closer to the target on every RPM "
                                                "reading; once at speed it must stay inside the tolerance. The VFD's RPM "
                                                "reading depends on PD144, so check that value first.");
                            ImGui::PopTextWrapPos();
                            int margin = vfd.speed_margin_pct;
                            ImGui::SetNextItemWidth(160.0f);
                            if (ImGui::SliderInt("Speed tolerance", &margin, 1, 20, "+/- %d%%")) {
                                vfd.speed_margin_pct = static_cast<uint8_t>(margin);
                                vfd_changed = true;
                            }
                            int bad = vfd.bad_speed_threshold;
                            ImGui::SetNextItemWidth(160.0f);
                            if (ImGui::SliderInt("Readings outside tolerance", &bad, 1, 20, "%d")) {
                                vfd.bad_speed_threshold = static_cast<uint8_t>(bad);
                                vfd_changed = true;
                            }
                            int track = vfd.tracking_threshold;
                            ImGui::SetNextItemWidth(160.0f);
                            if (ImGui::SliderInt("Readings without progress", &track, 1, 20, "%d")) {
                                vfd.tracking_threshold = static_cast<uint8_t>(track);
                                vfd_changed = true;
                            }
                        }

                        // Reading limits (Mach3 vfdPlugin status limits), 0 = off
                        ImGui::Spacing();
                        ImGui::Separator();
                        ImGui::Spacing();
                        ImGui::TextWrapped("E-Stop when a reading goes above its limit (0 = off):");
                        auto limitInput = [&](const char* label, float* value, const char* fmt) {
                            ImGui::SetNextItemWidth(160.0f);
                            if (ImGui::InputFloat(label, value, 0.0f, 0.0f, fmt)) {
                                *value = std::max(0.0f, *value);
                                vfd_changed = true;
                            }
                        };
                        limitInput("Current limit##VfdLimA", &vfd.limit_current_a, "%.1f A");
                        limitInput("Output frequency limit##VfdLimHz", &vfd.limit_out_freq_hz, "%.1f Hz");
                        limitInput("Speed limit##VfdLimRpm", &vfd.limit_rpm, "%.0f RPM");
                        limitInput("DC bus voltage limit##VfdLimDc", &vfd.limit_dc_volts, "%.1f V");
                        limitInput("Output voltage limit##VfdLimAc", &vfd.limit_ac_volts, "%.1f V");
                        limitInput("Temperature limit##VfdLimT", &vfd.limit_temp_c, "%.1f C");
                    }
                    ImGui::EndChild();

                    ImGui::Spacing();

                    // =========================================================
                    // 4. LIVE TELEMETRY & DIAGNOSTICS CARD
                    // =========================================================
                    ImGui::BeginChild("##VfdTelemetryCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                    {
                        ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "LIVE TELEMETRY & DIAGNOSTICS");
                        ImGui::Spacing();

                        // Alarm Banner if tripped
                        if (vt.anyAlarm()) {
                            ImGui::PushTextWrapPos(0.0f);
                            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.30f, 1.0f), "VFD ALARM: %s", vt.last_error_msg.c_str());
                            ImGui::PopTextWrapPos();
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                            if (ImGui::Button("Clear VFD Alarm", ImVec2(ImGui::CalcTextSize("Clear VFD Alarm").x + 32.0f, 28.0f))) {
                                controller.clearVfdAlarms();
                            }
                            ImGui::PopStyleColor();
                            ImGui::Spacing();
                        }

                        if (vt.connected) {
                            ImGui::TextColored(ImVec4(0.20f, 0.85f, 1.0f, 1.0f), "Speed: %5u RPM", vt.actual_rpm);
                            ImGui::SameLine(180.0f);
                            ImGui::TextColored(ImVec4(0.20f, 0.85f, 1.0f, 1.0f), "Freq: %5.1f Hz", vt.actual_freq_hz);

                            ImGui::TextColored(ImVec4(0.30f, 0.90f, 0.50f, 1.0f), "Current: %4.1f A", vt.actual_amps);
                            ImGui::SameLine(180.0f);
                            ImGui::TextColored(ImVec4(0.30f, 0.90f, 0.50f, 1.0f), "Bus: %5.1f VDC", vt.dc_bus_volts);

                            ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.20f, 1.0f), "Inverter Temp: %4.1f C", vt.temperature_c);
                            ImGui::SameLine(180.0f);
                            ImGui::TextColored(ImVec4(0.30f, 0.90f, 0.50f, 1.0f), "Output: %5.1f VAC", vt.ac_out_volts);
                        } else {
                            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.15f, 1.0f),
                                               "VFD Telemetry Offline - Check RS485 USB Adapter & Inverter Power");
                        }

                        ImGui::Spacing();
                        ImGui::Separator();
                        ImGui::Spacing();
                        ImGui::TextDisabled("Packets: %u TX | %u RX | %u Errors",
                                            vt.packets_sent, vt.packets_received, vt.comm_errors);
                    }
                    ImGui::EndChild();

                    ImGui::PopStyleVar(4);
                    ImGui::PopStyleColor(2);
                }

                if (vfd_changed) {
                    controller.applyConfig(cfg);
                    markSettingsDirty();
                }
            }

            // -----------------------------------------------------------------
            // MACH3 TOOL CHANGER (M6 AUTO ATC / MANUAL TOOLCHANGE FALLBACK)
            // -----------------------------------------------------------------
            ImGui::SetNextItemOpen(saved_settings.ui.header_toolchanger_open, ImGuiCond_Once);
            bool tc_hdr_open = ImGui::CollapsingHeader("Tool Changer (M6 Auto ATC / Manual)");
            if (tc_hdr_open != saved_settings.ui.header_toolchanger_open) {
                saved_settings.ui.header_toolchanger_open = tc_hdr_open;
                markSettingsDirty();
            }
            if (tc_hdr_open) {
                core::MachineConfig cfg = controller.getConfig();
                bool tc_changed = false;
                const float spacing = ImGui::GetStyle().ItemSpacing.x;

                ImGui::BeginChild("##ToolStatusBox", ImVec2(0.0f, 38.0f), true, ImGuiWindowFlags_NoScrollbar);
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(ImVec4(0.0f, 0.88f, 1.0f, 1.0f),
                                   "ACTIVE TOOL: T%d",
                                   telemetry.current_tool_number);
                if (telemetry.pending_tool_number > 0) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.15f, 1.0f),
                                       "  |  WAITING FOR M6 -> INSERT T%d",
                                       telemetry.pending_tool_number);
                    ImGui::SameLine(ImGui::GetWindowWidth() - 125.0f);
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f, 0.65f, 0.32f, 1.0f));
                    if (ImGui::SmallButton("Resume M6")) {
                        controller.startJob();
                    }
                    ImGui::PopStyleColor();
                } else {
                    ImGui::SameLine();
                    const char* mode_badge = (cfg.tool_change.mode == core::ToolChangeMode::ManualPrompt)
                                                 ? "[Mode: Manual M6 Stop & Wait]"
                                                 : (cfg.tool_change.mode == core::ToolChangeMode::AutoAtc)
                                                       ? "[Mode: Auto ATC]"
                                                       : "[Mode: Ignore M6]";
                    ImGui::TextDisabled("  %s", mode_badge);
                }
                ImGui::EndChild();

                const char* tc_modes[] = {
                    "Manual Toolchange (Stop Spindle, Retract Z & Wait)",
                    "Auto Toolchanger (ATC - Automatic M6 Change)",
                    "Ignore M6 (Single-Tool Continuous)"
                };
                int cur_mode = static_cast<int>(cfg.tool_change.mode);
                const float m6_lbl_w = ImGui::CalcTextSize("M6 Mode").x;
                ImGui::SetNextItemWidth(std::max(100.0f, ImGui::GetContentRegionAvail().x - m6_lbl_w - spacing));
                if (ImGui::Combo("M6 Mode", &cur_mode, tc_modes, 3)) {
                    cfg.tool_change.mode = static_cast<core::ToolChangeMode>(cur_mode);
                    tc_changed = true;
                }

                if (cfg.tool_change.mode == core::ToolChangeMode::ManualPrompt) {
                    const bool is_imperial_m6 = (cfg.linear_units == core::LinearUnits::Imperial);
                    const double m6_scale = is_imperial_m6 ? (1.0 / 25.4) : 1.0;
                    const double m6_unscale = is_imperial_m6 ? 25.4 : 1.0;
                    const char* m6_unit_str = is_imperial_m6 ? "in" : "mm";
                    const float m6_step_slow = is_imperial_m6 ? 0.1f : 2.0f;
                    const float m6_step_fast = is_imperial_m6 ? 0.5f : 10.0f;
                    const char* m6_fmt = is_imperial_m6 ? "%.2f" : "%.1f";

                    float safe_z = static_cast<float>(cfg.tool_change.safe_z_mm * m6_scale);
                    char safe_z_lbl[48];
                    std::snprintf(safe_z_lbl, sizeof(safe_z_lbl), "M6 Safe Z Retract (%s)", m6_unit_str);
                    const float z_lbl_w = ImGui::CalcTextSize(safe_z_lbl).x;
                    ImGui::SetNextItemWidth(std::max(80.0f, ImGui::GetContentRegionAvail().x - z_lbl_w - spacing));
                    if (ImGui::InputFloat(safe_z_lbl, &safe_z, m6_step_slow, m6_step_fast, m6_fmt)) {
                        cfg.tool_change.safe_z_mm = std::max(0.0f, static_cast<float>(safe_z * m6_unscale));
                        tc_changed = true;
                    }
                    if (ImGui::Checkbox("Move to XY Tool Change Park Position on M6", &cfg.tool_change.move_to_park_xy)) {
                        tc_changed = true;
                    }
                    if (cfg.tool_change.move_to_park_xy) {
                        float px = static_cast<float>(cfg.tool_change.park_x_mm * m6_scale);
                        float py = static_cast<float>(cfg.tool_change.park_y_mm * m6_scale);
                        const float park_avail = ImGui::GetContentRegionAvail().x;
                        const float park_half  = (park_avail - spacing) * 0.5f;

                        char px_lbl[32], py_lbl[32];
                        std::snprintf(px_lbl, sizeof(px_lbl), "Park X (%s)", m6_unit_str);
                        std::snprintf(py_lbl, sizeof(py_lbl), "Park Y (%s)", m6_unit_str);

                        const float px_lbl_w = ImGui::CalcTextSize(px_lbl).x;
                        ImGui::SetNextItemWidth(std::max(50.0f, park_half - px_lbl_w - spacing));
                        if (ImGui::InputFloat(px_lbl, &px, is_imperial_m6 ? 0.25f : 5.0f, is_imperial_m6 ? 1.0f : 25.0f, m6_fmt)) {
                            cfg.tool_change.park_x_mm = px * m6_unscale;
                            tc_changed = true;
                        }
                        ImGui::SameLine();
                        const float py_lbl_w = ImGui::CalcTextSize(py_lbl).x;
                        ImGui::SetNextItemWidth(std::max(50.0f, ImGui::GetContentRegionAvail().x - py_lbl_w - spacing));
                        if (ImGui::InputFloat(py_lbl, &py, is_imperial_m6 ? 0.25f : 5.0f, is_imperial_m6 ? 1.0f : 25.0f, m6_fmt)) {
                            cfg.tool_change.park_y_mm = py * m6_unscale;
                            tc_changed = true;
                        }
                    }
                } else if (cfg.tool_change.mode == core::ToolChangeMode::AutoAtc) {
                    float dwell = static_cast<float>(cfg.tool_change.atc_dwell_seconds);
                    const float dwell_lbl_w = ImGui::CalcTextSize("ATC Change Time (s)").x;
                    ImGui::SetNextItemWidth(std::max(60.0f, ImGui::GetContentRegionAvail().x - dwell_lbl_w - spacing));
                    if (ImGui::InputFloat("ATC Change Time (s)", &dwell, 0.5f, 2.0f, "%.1f")) {
                        cfg.tool_change.atc_dwell_seconds = std::clamp(dwell, 0.2f, 30.0f);
                        tc_changed = true;
                    }
                }

                if (tc_changed) {
                    controller.applyConfig(cfg);
                    markSettingsDirty();
                }
            }

            // -----------------------------------------------------------------
            // TOOL TABLE & TOOL LENGTH OFFSETS (G43 H<n>)
            // -----------------------------------------------------------------
            ImGui::SetNextItemOpen(saved_settings.ui.header_tool_table_open, ImGuiCond_Once);
            bool tt_hdr_open = ImGui::CollapsingHeader("Tool Table & Tool Length Offsets (G43)");
            if (tt_hdr_open != saved_settings.ui.header_tool_table_open) {
                saved_settings.ui.header_tool_table_open = tt_hdr_open;
                markSettingsDirty();
            }
            if (tt_hdr_open) {
                tool_table_panel.render(controller, telemetry, [&]() {
                    saved_settings.machine = controller.getConfig();
                    markSettingsDirty();
                });
            }

            // -----------------------------------------------------------------
            // TOUCH PROBE & TOOL SETTER DECK (MILL)
            // -----------------------------------------------------------------
            ImGui::SetNextItemOpen(saved_settings.ui.header_probe_macro_open, ImGuiCond_Once);
            bool probe_macro_hdr_open = ImGui::CollapsingHeader("Touch Probe & Tool Setter (Macro Routines)");
            if (probe_macro_hdr_open != saved_settings.ui.header_probe_macro_open) {
                saved_settings.ui.header_probe_macro_open = probe_macro_hdr_open;
                markSettingsDirty();
            }
            if (probe_macro_hdr_open) {
                core::MachineConfig probe_cfg = controller.getConfig();
                probe_panel.render(controller, telemetry, probe_cfg, &viewport);
            }

            // -----------------------------------------------------------------
            // TOUCH PROBE & TOOL PLATE SETTINGS
            // -----------------------------------------------------------------
            ImGui::SetNextItemOpen(saved_settings.ui.header_probe_settings_open, ImGuiCond_Once);
            bool probe_set_hdr_open = ImGui::CollapsingHeader("Touch Probe & Tool Plate Settings");
            if (probe_set_hdr_open != saved_settings.ui.header_probe_settings_open) {
                saved_settings.ui.header_probe_settings_open = probe_set_hdr_open;
                markSettingsDirty();
            }
            if (probe_set_hdr_open) {
                core::MachineConfig cfg = controller.getConfig();
                bool probe_changed = false;

                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
                ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
                ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                // 1. HARDWARE INPUT PIN & POLARITY
                ImGui::BeginChild("##ProbePinCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                {
                    ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "PROBE HARDWARE INPUT & POLARITY");
                    ImGui::Spacing();

                    int p_pin = cfg.probe.probe_pin;
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::SliderInt("Probe Pin (IN#)", &p_pin, 1, 12)) {
                        cfg.probe.probe_pin = static_cast<uint8_t>(p_pin);
                        probe_changed = true;
                    }
                    ImGui::SameLine();
                    if (ImGui::Checkbox("Active Low##Probe", &cfg.probe.probe_invert)) {
                        probe_changed = true;
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Active Low: Checked = Normally-Open touch plate / switch that connects to COM/GND when touched.\nUnchecked = Active High (Normally-Closed NC or PNP sensor).");
                    }
                }
                ImGui::EndChild();

                ImGui::Spacing();

                // 2. TOUCH PLATE & TOOL SETTER DIMENSIONS
                ImGui::BeginChild("##ProbeDimsCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                {
                    ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "TOUCH PLATE & TOOL SETTER DIMENSIONS");
                    ImGui::Spacing();

                    float plate_t = static_cast<float>(cfg.probe.plate_thickness_mm);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat("Z Plate Thickness (mm)", &plate_t, 0.1f, 1.0f, "%.3f")) {
                        cfg.probe.plate_thickness_mm = std::max(0.0f, plate_t);
                        probe_changed = true;
                    }

                    float tip_d = static_cast<float>(cfg.probe.tool_diameter_mm);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat("Probe Tip Diameter (mm)", &tip_d, 0.1f, 1.0f, "%.3f")) {
                        cfg.probe.tool_diameter_mm = std::max(0.0f, tip_d);
                        probe_changed = true;
                    }
                }
                ImGui::EndChild();

                ImGui::Spacing();

                // 3. PROBING SPEEDS & SEARCH LIMITS
                ImGui::BeginChild("##ProbeSpeedsCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                {
                    ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "PROBING SPEEDS & TRAVEL LIMITS");
                    ImGui::Spacing();

                    const bool is_imp_prb = (cfg.linear_units == core::LinearUnits::Imperial);
                    const double prb_f_scale = is_imp_prb ? (1.0 / 25.4) : 1.0;
                    const double prb_f_unscale = is_imp_prb ? 25.4 : 1.0;
                    const char* prb_f_lbl = is_imp_prb ? "IPM" : "mm/min";
                    const char* prb_d_lbl = is_imp_prb ? "in" : "mm";

                    float fast_f = static_cast<float>(cfg.probe.fast_feed_mm_min * prb_f_scale);
                    char search_lbl[48];
                    std::snprintf(search_lbl, sizeof(search_lbl), "Search Feed (%s)", prb_f_lbl);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat(search_lbl, &fast_f, is_imp_prb ? 1.0f : 20.0f, is_imp_prb ? 5.0f : 100.0f, is_imp_prb ? "%.1f" : "%.0f")) {
                        cfg.probe.fast_feed_mm_min = std::max(is_imp_prb ? 1.0f : 10.0f, static_cast<float>(fast_f * prb_f_unscale));
                        probe_changed = true;
                    }

                    float slow_f = static_cast<float>(cfg.probe.slow_feed_mm_min * prb_f_scale);
                    char latch_lbl[48];
                    std::snprintf(latch_lbl, sizeof(latch_lbl), "Latch Feed (%s)", prb_f_lbl);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat(latch_lbl, &slow_f, is_imp_prb ? 0.2f : 5.0f, is_imp_prb ? 1.0f : 20.0f, is_imp_prb ? "%.1f" : "%.0f")) {
                        cfg.probe.slow_feed_mm_min = std::max(is_imp_prb ? 0.1f : 2.0f, static_cast<float>(slow_f * prb_f_unscale));
                        probe_changed = true;
                    }

                    float max_d = static_cast<float>(cfg.probe.max_travel_mm * prb_f_scale);
                    char max_lbl[48];
                    std::snprintf(max_lbl, sizeof(max_lbl), "Max Search Travel (%s)", prb_d_lbl);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat(max_lbl, &max_d, is_imp_prb ? 0.2f : 5.0f, is_imp_prb ? 1.0f : 20.0f, is_imp_prb ? "%.2f" : "%.1f")) {
                        cfg.probe.max_travel_mm = std::max(is_imp_prb ? 0.1f : 2.0f, static_cast<float>(max_d * prb_f_unscale));
                        probe_changed = true;
                    }

                    float ret_d = static_cast<float>(cfg.probe.retract_mm * prb_f_scale);
                    char ret_lbl[48];
                    std::snprintf(ret_lbl, sizeof(ret_lbl), "Retract Distance (%s)", prb_d_lbl);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat(ret_lbl, &ret_d, is_imp_prb ? 0.02f : 0.5f, is_imp_prb ? 0.1f : 2.0f, is_imp_prb ? "%.3f" : "%.2f")) {
                        cfg.probe.retract_mm = std::max(is_imp_prb ? 0.01f : 0.2f, static_cast<float>(ret_d * prb_f_unscale));
                        probe_changed = true;
                    }

                    float srf_depth = static_cast<float>(cfg.probe.surface_max_depth_below_zero_mm * prb_f_scale);
                    char srf_depth_lbl[64];
                    std::snprintf(srf_depth_lbl, sizeof(srf_depth_lbl), "Surface Max Depth Below Z 0.0 (%s)", prb_d_lbl);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat(srf_depth_lbl, &srf_depth, is_imp_prb ? 0.02f : 0.5f, is_imp_prb ? 0.1f : 2.0f, is_imp_prb ? "%.3f" : "%.2f")) {
                        cfg.probe.surface_max_depth_below_zero_mm = std::max(is_imp_prb ? 0.01f : 0.2f, static_cast<float>(srf_depth * prb_f_unscale));
                        probe_changed = true;
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Maximum plunge depth below Work Z 0.0 during surface grid probing to prevent probe crashes if traversing over holes, pockets, or workpiece edges.");
                    }

                    float srf_clear = static_cast<float>(cfg.probe.surface_clearance_z_mm * prb_f_scale);
                    char srf_clear_lbl[64];
                    std::snprintf(srf_clear_lbl, sizeof(srf_clear_lbl), "Surface Grid Clearance Z (%s)", prb_d_lbl);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat(srf_clear_lbl, &srf_clear, is_imp_prb ? 0.05f : 0.5f, is_imp_prb ? 0.2f : 2.0f, is_imp_prb ? "%.3f" : "%.2f")) {
                        cfg.probe.surface_clearance_z_mm = std::max(is_imp_prb ? 0.05f : 1.0f, static_cast<float>(srf_clear * prb_f_unscale));
                        probe_changed = true;
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Safe transit clearance height in Work Z above Z 0.0 when traveling between surface probe points.");
                    }
                }
                ImGui::EndChild();

                ImGui::Spacing();

                // 4. WORKPIECE & FEATURE CLEARANCES
                ImGui::BeginChild("##ProbeClearanceCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                {
                    ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "FEATURE SEARCH CLEARANCES & GEOMETRY");
                    ImGui::Spacing();

                    float xy_clear = static_cast<float>(cfg.probe.xy_clearance_mm);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat("XY Clearance (mm)", &xy_clear, 1.0f, 5.0f, "%.1f")) {
                        cfg.probe.xy_clearance_mm = std::max(0.5f, xy_clear);
                        probe_changed = true;
                    }

                    float p_depth = static_cast<float>(cfg.probe.probe_depth_mm);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat("Z Probe Depth (mm)", &p_depth, 1.0f, 5.0f, "%.1f")) {
                        cfg.probe.probe_depth_mm = std::max(0.5f, p_depth);
                        probe_changed = true;
                    }

                    float bore_d = static_cast<float>(cfg.probe.approx_hole_diam_mm);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat("Hole Bore Hint (mm)", &bore_d, 1.0f, 10.0f, "%.1f")) {
                        cfg.probe.approx_hole_diam_mm = std::max(4.0f, bore_d);
                        probe_changed = true;
                    }

                    float size_x = static_cast<float>(cfg.probe.approx_size_x_mm);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat("Approx Workpiece X (mm)", &size_x, 5.0f, 25.0f, "%.1f")) {
                        cfg.probe.approx_size_x_mm = std::max(1.0f, size_x);
                        probe_changed = true;
                    }

                    float size_y = static_cast<float>(cfg.probe.approx_size_y_mm);
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::InputFloat("Approx Workpiece Y (mm)", &size_y, 5.0f, 25.0f, "%.1f")) {
                        cfg.probe.approx_size_y_mm = std::max(1.0f, size_y);
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
                workspace_panel.render(controller, telemetry, true, [&]() {
                    saved_settings.machine.workspace_offsets_mm = controller.getWorkspaceOffsets();
                    saved_settings.machine.active_workspace     = controller.getActiveWorkspace();
                    markSettingsDirty();
                });
            }

            // -----------------------------------------------------------------
            // OPTICAL CAMERA POSITIONING & REMOTE WEBCAM MONITORING (MILL)
            // -----------------------------------------------------------------
            ImGui::SetNextItemOpen(saved_settings.ui.header_camera_open, ImGuiCond_Once);
            bool cam_hdr_open = ImGui::CollapsingHeader("Camera Positioning & Remote Monitoring");
            if (cam_hdr_open != saved_settings.ui.header_camera_open) {
                saved_settings.ui.header_camera_open = cam_hdr_open;
                markSettingsDirty();
            }
            if (cam_hdr_open) {
                if (camera_panel.render(controller, telemetry, &viewport, true)) {
                    saved_settings.machine = controller.getConfig();
                    markSettingsDirty();
                }
            }

            // -----------------------------------------------------------------
            // END-STOPS, HOMING & SOFT LIMITS CONFIGURATION
            // -----------------------------------------------------------------
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

                // 1. HARDWARE E-STOP INPUT PIN
                ImGui::BeginChild("##EStopLimitCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
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

                const char* ax_names[4] = {"X AXIS LIMITS & HOMING", "Y AXIS LIMITS & HOMING", "Z AXIS LIMITS & HOMING", "A AXIS LIMITS & HOMING"};
                const char* card_ids[4] = {"##LimitsXCard", "##LimitsYCard", "##LimitsZCard", "##LimitsACard"};
                for (int a = 0; a < 4; ++a) {
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

            // -----------------------------------------------------------------
            // MACHINE MOTOR TUNING
            // -----------------------------------------------------------------
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

                // 1. GLOBAL SYSTEM OF MEASUREMENT & VELOCITY CONFIGURATION
                ImGui::BeginChild("##MotorUnitCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
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
                        ImGui::SetTooltip("Import motor tuning (steps/unit, velocity, acceleration), pin assignments,\ninversions, homing speeds, soft limits, and spindle pulleys directly from a Mach3 XML profile\nfollowed by the interactive 5-Stage Post-Migration Machine Verification Routine.");
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

                const char* ax_names[4] = {"X AXIS MOTOR TUNING", "Y AXIS MOTOR TUNING", "Z AXIS MOTOR TUNING", "A AXIS MOTOR TUNING"};
                const char* motor_card_ids[4] = {"##MotorXCard", "##MotorYCard", "##MotorZCard", "##MotorACard"};
                for (int a = 0; a < 4; ++a) {
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
                ImGui::EndChild(); // ##MillSettingsScroll
            }
            ImGui::EndChild(); // ##MillRightCol
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
            ImGui::TextWrapped("A cycle or motion command is currently running. Exiting the software while the machine is running can cause loss of position or leave spindle outputs energized.");
            ImGui::Spacing();
            ImGui::TextWrapped("Clicking 'E-STOP & EXIT' will immediately engage E-Stop, halt all pulse generation and spindle control, and safely close the application.");
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

        // Interactive Script Operator Prompt Modal (e.g. M6 Tool Change prompt)
        scripting_panel.renderPromptModal(controller);
        scripting_panel.renderHelpModal(controller);

        // Mach3 XML Profile Migration & 5-Stage Post-Migration Machine Verification Wizard
        if (mach3_wizard.render(controller, false)) {
            saved_settings.machine = controller.getConfig();
            markSettingsDirty();
        }

        // Sync DRO jog settings if modified in panel
        if (dro_panel.isStepJogMode() != saved_settings.ui.step_jog_mode ||
            std::abs(dro_panel.getStepSizeMm() - saved_settings.ui.step_size_mm) > 0.001f ||
            std::abs(dro_panel.getJogSpeedPct() - saved_settings.ui.jog_speed_pct) > 0.01f) {
            saved_settings.ui.step_jog_mode = dro_panel.isStepJogMode();
            saved_settings.ui.step_size_mm  = dro_panel.getStepSizeMm();
            saved_settings.ui.jog_speed_pct = dro_panel.getJogSpeedPct();
            markSettingsDirty();
        }

        ui::IoStatusBar::renderBottomIoStrip(controller, telemetry);

        // Debounced automatic background saving when any setting, panel, or layout changes
        if (s_settings_dirty && (ImGui::GetTime() - s_last_dirty_time >= 0.25 || !ImGui::IsAnyItemActive())) {
            saveCurrentSettings();
            s_settings_dirty = false;
        }
    });

    // Stop the web server first: its client threads use callbacks that capture locals of main()
    stream_server.stop();

    // Ensure E-Stop is engaged and automatically persist all settings on exit
    controller.triggerEStop();
    saveCurrentSettings();
    return exit_code;
}
