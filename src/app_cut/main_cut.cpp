#include "breakout_version.h"
#include "core/controller/rnr_controller.h"
#include "core/config/settings_manager.h"
#include "ui_common/app_window.h"
#include "ui_common/theme.h"
#include "ui_common/io_settings_panel.h"
#include "ui_common/toolpath_viewport.h"
#include "ui_common/dro_panel.h"
#include "ui_common/io_status_bar.h"
#include "ui_common/gcode_panel.h"
#include "ui_common/app_dir.h"
#include "ui_common/remote_upload.h"
#include "ui_common/app_shell.h"
#include "ui_common/machine_settings_sections.h"
#include "core/util/utf8_path.h"
#include "ui_common/scripting_panel.h"
#include "ui_common/workspace_panel.h"
#include "ui_common/file_dialog.h"
#include "ui_common/mach3_migration_dialog.h"
#include "ui_common/gantry_slave_combo.h"
#include "ui_common/corner_settings.h"
#include "ui_common/program_start_settings.h"
#include "ui_common/camera_panel.h"
#include "ui_common/board_panels.h"
#include "ui_common/fake_board_mode.h"
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
    // --fake-board: drive the built-in GRBL-family emulator; no hardware, separate settings file
    const bool fake_board = ui::hasCommandLineFlag(argc, argv, ui::kFakeBoardFlag);
    if (fake_board) core::RnRController::setHardwareAccessEnabled(false);
    const std::string settings_file = fake_board ? ui::fakeBoardSettingsFile("breakout_cut_settings.ini") : std::string("breakout_cut_settings.ini");
    config::PersistentSettings saved_settings{};

    core::RnRController controller(false); // Connects below, once the settings (and the board) are applied
    saved_settings.machine = *controller.getConfigShared();
    // Off unless the settings file says otherwise: slowing down on curves changes the cut and locks the THC anti-dive
    saved_settings.machine.limit_curve_speed = false;
    saved_settings.machine.scripting.home_script_path = "scripts/cut/home_all.lua";
    bool had_saved_settings = config::SettingsManager::loadFromFile(settings_file, saved_settings);
    if (!had_saved_settings && !fake_board) {
        // Fallback to legacy settings file if present
        had_saved_settings = config::SettingsManager::loadFromFile("rnr_plasma_settings.ini", saved_settings);
    }

    if (fake_board) ui::enterFakeBoardMode(saved_settings, false);

    ui::AppWindow window(
        fake_board ? "BreakoutCut " BREAKOUTCNC_VERSION_STRING " [fake GRBL board]" : "BreakoutCut " BREAKOUTCNC_VERSION_STRING,
        saved_settings.ui.window_width,
        saved_settings.ui.window_height);
    if (!window.isValid()) {
        return 1;
    }
    // Test mode only: operate the UI from ui_remote_commands.txt (never with real hardware)
    if (fake_board && ui::hasCommandLineFlag(argc, argv, "--ui-remote")) window.enableUiRemote(".");

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

    ui::RemoteUploadInbox remote_upload; // Filled by the web server thread, read by the frame loop

    stream_server.setLoadGCodeCallback([&](const std::string& filename, const std::string& content) -> core::LoadGCodeResult {
        std::string path;
        auto result = ui::loadRemoteUpload(controller, filename, content, path);
        if (result.success) remote_upload.post(path);
        return result;
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

    // Connect to the selected board (the simulator when none answers), then start disarmed: the operator clicks RESET E-STOP
    if (!(had_saved_settings && saved_settings.ui.simulator_mode) && !controller.connect()) {
        controller.setSimulatorMode(true);
    }
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
    // Older versions stored the name in the ANSI code page: names are UTF-8 now
    saved_settings.ui.last_gcode_file_path = util::utf8FromLegacy(saved_settings.ui.last_gcode_file_path);
    ui::cleanOldRemoteUploads(saved_settings.ui.last_gcode_file_path);
    std::error_code last_file_ec;
    if (!saved_settings.ui.last_gcode_file_path.empty() &&
        std::filesystem::exists(util::pathFromUtf8(saved_settings.ui.last_gcode_file_path), last_file_ec))
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

    std::string settings_save_error; // Shown until dismissed or a save succeeds
    auto saveCurrentSettings = [&]() {
        if (smoke_test) return; // A smoke test never changes the settings next to the executable
        const auto tel = controller.getTelemetry();
        saved_settings.machine = *controller.getConfigShared();
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

        if (config::SettingsManager::saveToFile(settings_file, saved_settings)) {
            settings_save_error.clear();
        } else if (settings_save_error.empty()) {
            settings_save_error = "The settings could not be saved to " + settings_file +
                                  " next to the application (read-only folder?). Work offsets, the tool table and "
                                  "the last position are lost when the application closes.";
        }
    };

    bool minimal_layout = saved_settings.ui.minimal_layout;

    bool settings_dirty = false;
    double last_dirty_time = 0.0;
    auto markSettingsDirty = [&]() {
        settings_dirty = true;
        last_dirty_time = ImGui::GetTime();
    };

    dro_panel.setOnConfigChanged([&]() {
        saved_settings.machine = *controller.getConfigShared();
        markSettingsDirty();
    });

    // Operation Mode (Plasma / Laser) selector for title bar
    auto updateWindowCaptionForMode = [&](core::OperationMode mode) {
        if (mode == core::OperationMode::Laser) {
            window.setTitle(std::string("BreakoutCut " BREAKOUTCNC_VERSION_STRING " - Laser") + (fake_board ? " [fake GRBL board]" : ""));
            viewport.setFollowToolLabel("Follow Laser");
            gcode_panel.setMachineType(gcode::MachineType::Laser);
        } else {
            window.setTitle(std::string("BreakoutCut " BREAKOUTCNC_VERSION_STRING " - Plasma") + (fake_board ? " [fake GRBL board]" : ""));
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

    ui::connectWindowSettings(window, viewport, saved_settings.ui, markSettingsDirty, &minimal_layout);

    // A G-code file given on the command line ("Open with" / double-click) or dropped onto the window
    if (!arg_file.empty()) {
        gcode_panel.queueFileToLoad(arg_file);
    }
    window.setOnFileDropped([&](const std::string& path) { gcode_panel.queueFileToLoad(path); });
    window.setIdlePredicate([&] {
        return !gcode_panel.isLoading() && ui::screenIsStatic(controller.getTelemetry(), *controller.getConfigShared());
    });

    bool show_exit_confirm_modal = false;
    bool force_close = false;
    ui::installCloseHandling(window, controller, force_close, show_exit_confirm_modal, saveCurrentSettings);

    if (smoke_test) ImGui::GetIO().IniFilename = nullptr; // No imgui.ini either
    ui::SmokeTest smoke{smoke_test};

    int exit_code = window.runLoop([&]() {
        smoke.frame(window, controller);
        // Global keyboard E-Stop: ESC halts a moving machine whatever has keyboard focus (at rest it closes a popup)
        if (ui::escapeTriggersEStop(controller.getTelemetry())) {
            controller.triggerEStop();
        }
        ui::renderSettingsSaveWarning(settings_save_error);

        // Process any pending remote G-Code upload from the web server
        {
            std::string upload_path;
            bool fit = false;
            if (remote_upload.take(upload_path, fit)) {
                loaded_filename     = upload_path;
                persisted_file_path = upload_path;
                saved_settings.ui.last_gcode_file_path = persisted_file_path;
                markSettingsDirty();
                if (fit) {
                    const auto pr_ptr = controller.getProgram();
                    viewport.fitToBounds(pr_ptr->bbox_min_mm, pr_ptr->bbox_max_mm);
                }
            }
        }

        const auto telemetry   = controller.getTelemetry();
        const board::BoardCapabilities& board_caps = ui::boardCaps(controller, telemetry);
        const ui::SettingsSectionContext settings_ctx{controller, board_caps, saved_settings.ui, markSettingsDirty,
                                                      &mach3_wizard, 3};
        const bool host_board = board_caps.family == board::Family::HostPlanned;
        const auto program_snapshot = controller.getProgram(); // Held for the whole frame
        const auto& parse_res  = *program_snapshot;
        const auto& segments   = program_snapshot->segments;

        camera_mgr.updateLiveTelemetry(telemetry, *controller.getConfigShared());
        stream_server.updateTelemetry(telemetry, controller.getConfigShared()->linear_units,
                                      current_op_mode == core::OperationMode::Laser ? "BreakoutCut - Laser" : "BreakoutCut - Plasma",
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

        // Left Column: Sticky DRO/Jog Panel & Bottom Controller State Badge / E-Stop Card
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

        // Center Column: Toolpath Viewport + G-Code Panel + Lua Scripting Deck (Full View Only)
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
                bool lua_hdr_open = ImGui::CollapsingHeader("Scripts & Macros", ImGuiTreeNodeFlags_AllowOverlap);

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
                    core::MachineConfig cur_cfg = *controller.getConfigShared();
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

        // Right Column: Settings & Configuration Deck (Completely hidden in MINIMAL view)
        if (!minimal_layout) {
            ImGui::SameLine();
            ImGui::BeginChild("##PlasmaRightCol", ImVec2(right_col_w, workspace_h), false, ImGuiWindowFlags_NoScrollbar);
            {
                ImGui::BeginChild("##PlasmaSettingsScroll", ImVec2(0.0f, 0.0f), true);
                {
                    // -----------------------------------------------------------------
                    // USB MOTION CONTROLLER & CONNECTION (RNR ECO MOTION 2.0)
                    // -----------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_usb_open, ImGuiCond_Once);
                    bool usb_hdr_open = ImGui::CollapsingHeader("Connection");
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
                    // BOARD SETTINGS & CONSOLE (firmware-planned boards: GRBL, grblHAL, FluidNC)
                    // -----------------------------------------------------------------
                    if (board_caps.family == board::Family::FirmwarePlanned) {
                        static bool board_settings_open = false;
                        ImGui::SetNextItemOpen(board_settings_open, ImGuiCond_Once);
                        board_settings_open = ImGui::CollapsingHeader("Board Settings");
                        if (board_settings_open) ui::renderFirmwareSettings(controller, telemetry);
                        static bool board_console_open = false;
                        ImGui::SetNextItemOpen(board_console_open, ImGuiCond_Once);
                        board_console_open = ImGui::CollapsingHeader("Board Console");
                        if (board_console_open) ui::renderBoardConsole(controller, telemetry);
                    }
                    ImGui::Spacing();

                    if (current_op_mode == core::OperationMode::Plasma) {
                    // -------------------------------------------------------------
                    // PLASMA TORCH & PROCESS CONTROL
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_spindle_open, ImGuiCond_Once);
                    bool torch_hdr_open = ImGui::CollapsingHeader("Torch");
                    if (torch_hdr_open != saved_settings.ui.header_spindle_open) {
                        saved_settings.ui.header_spindle_open = torch_hdr_open;
                        markSettingsDirty();
                    }
                    if (torch_hdr_open) {
                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
                        ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                        // Card: Torch Firing & Interlocks
                        ImGui::BeginChild("##TorchFireCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ui::theme::pal().heading, "TORCH");
                            ImGui::Spacing();

                            const bool torch_on = telemetry.torch_or_spindle_on;
                            ImVec4 torch_col = torch_on ? ui::theme::pal().btn_warn : ui::theme::pal().btn_neutral;
                            ImGui::PushStyleColor(ImGuiCol_Button, torch_col);
                            const char* torch_lbl = torch_on ? "Torch on - click to stop"
                                                             : "Fire torch (M3)";
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
                            ImGui::TextColored(ui::theme::pal().heading, "FEED OVERRIDE");
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
                            dl->AddRectFilled(box_p0, box_p1, ui::theme::pal().readout_bg, 4.0f);
                            dl->AddRect(box_p0, box_p1, is_moving ? ui::theme::pal().readout_border_active : ui::theme::pal().readout_border, 4.0f, 0, 1.2f);

                            ImFont* gfont = ui::AppWindow::getGCodeFont();
                            float fsz_lbl = 11.0f;
                            float fsz_val = 14.0f;

                            const bool is_imperial_feed = (controller.getConfigShared()->linear_units == core::LinearUnits::Imperial);
                            const double feed_scale = is_imperial_feed ? (1.0 / 25.4) : 1.0;
                            const char* feed_unit_lbl = is_imperial_feed ? "IPM" : "mm/m";
                            const char* feed_sec_lbl = is_imperial_feed ? "in/s" : "mm/s";

                            float line1_y = box_p0.y + 7.0f;
                            dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line1_y + 2.0f),
                                        is_moving ? ui::theme::pal().readout_label_active : ui::theme::pal().readout_label, "ACT FEED");
                            char act_feed_str[32];
                            std::snprintf(act_feed_str, sizeof(act_feed_str), "%4.0f %s", act_feed_mm_min * feed_scale, feed_unit_lbl);
                            ImVec2 act_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, act_feed_str) : ImGui::CalcTextSize(act_feed_str);
                            dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - act_sz.x - 7.0f, line1_y),
                                        is_moving ? ui::theme::pal().readout_value_active : ui::theme::pal().readout_value, act_feed_str);

                            float line2_y = box_p0.y + 31.0f;
                            dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line2_y + 2.0f), ui::theme::pal().readout_label_dim, "CMD (F)");
                            char cmd_feed_str[32];
                            std::snprintf(cmd_feed_str, sizeof(cmd_feed_str), "%4.0f %s", telemetry.commanded_feed_mm_min * feed_scale, feed_unit_lbl);
                            ImVec2 cmd_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, cmd_feed_str) : ImGui::CalcTextSize(cmd_feed_str);
                            dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - cmd_sz.x - 7.0f, line2_y), ui::theme::pal().readout_value_cmd, cmd_feed_str);

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
                                    ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_warn_subtle);
                                    ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::pal().text_warn);
                                }
                                if (ImGui::Button("Reset 100%##Feed", ImVec2(right_btn_w, 28.0f))) {
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
                    // TORCH HEIGHT CONTROL (THC)
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_vfd_open, ImGuiCond_Once);
                    bool thc_hdr_open = ImGui::CollapsingHeader("Torch Height Control");
                    if (thc_hdr_open != saved_settings.ui.header_vfd_open) {
                        saved_settings.ui.header_vfd_open = thc_hdr_open;
                        markSettingsDirty();
                    }
                    if (thc_hdr_open && board_caps.board_thc) {
                        // grblHAL plasma plugin: the board waits for Arc OK and corrects the height itself
                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
                        ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));
                        ImGui::BeginChild("##BoardThcCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ui::theme::pal().heading, "THC ON THE BOARD");
                            ImGui::PushTextWrapPos(0.0f);
                            const std::string mode = telemetry.thc_board_mode.empty() ? board_caps.board_thc_mode : telemetry.thc_board_mode;
                            ImGui::TextDisabled("%s runs torch height control itself (plasma plugin, %s mode). After M3 it waits "
                                                "for Arc OK, and during the cut it corrects the torch height. Its settings are "
                                                "in the Plasma group under Board Settings.",
                                                board_caps.display_name.c_str(), mode.empty() ? "unknown" : mode.c_str());
                            ImGui::PopTextWrapPos();
                            ImGui::Spacing();

                            // Status lamps, wrapped onto the next line when the panel is narrow
                            bool first_lamp = true;
                            auto lamp = [&](const char* label, bool on, const ImVec4& on_col) {
                                const float w = ImGui::CalcTextSize(label).x;
                                if (!first_lamp) {
                                    const float right = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
                                    if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + w <= right) ImGui::SameLine();
                                }
                                first_lamp = false;
                                ImGui::TextColored(on ? on_col : ui::theme::pal().text_muted, "%s", label);
                            };
                            lamp("Torch", telemetry.thc_torch_on, ui::theme::pal().text_warn);
                            lamp("Arc OK", telemetry.thc_arc_ok_active, ui::theme::pal().text_ok);
                            lamp("THC active", telemetry.thc_board_active, ui::theme::pal().text_ok);
                            lamp("Up", telemetry.thc_up_active, ui::theme::pal().text_info);
                            lamp("Down", telemetry.thc_down_active, ui::theme::pal().text_warn);
                            lamp("Anti-dive", telemetry.thc_anti_dive_locked, ui::theme::pal().text_warn);
                            ImGui::Text("Arc voltage: %.1f V", telemetry.thc_arc_voltage);

                            core::MachineConfig thc_cfg = *controller.getConfigShared();
                            if (thc_cfg.thc.enabled || thc_cfg.thc.wait_for_arc_ok) {
                                ImGui::PushTextWrapPos(0.0f);
                                ImGui::TextDisabled("The app's own THC settings (inputs, speeds, limits) are not used with this board.");
                                ImGui::PopTextWrapPos();
                            }
                        }
                        ImGui::EndChild();
                        ImGui::PopStyleVar(4);
                        ImGui::PopStyleColor(2);
                    } else if (thc_hdr_open && !board_caps.realtime_process_control) {
                        ImGui::PushTextWrapPos(0.0f);
                        ImGui::TextColored(ui::theme::pal().text_warn,
                                           "Torch height control and Wait for Arc OK need a board with real-time control. "
                                           "%s plans the motion itself: switch THC off to cut plasma without height control.",
                                           board_caps.display_name.c_str());
                        ImGui::PopTextWrapPos();
                        core::MachineConfig thc_cfg = *controller.getConfigShared();
                        if (thc_cfg.thc.enabled || thc_cfg.thc.wait_for_arc_ok) {
                            if (ImGui::Button("Switch THC off", ImVec2(ImGui::CalcTextSize("Switch THC off").x + 32.0f, 28.0f))) {
                                thc_cfg.thc.enabled = false;
                                thc_cfg.thc.wait_for_arc_ok = false;
                                controller.applyConfig(thc_cfg);
                                markSettingsDirty();
                            }
                        }
                    } else if (thc_hdr_open) {
                        core::MachineConfig cfg = *controller.getConfigShared();
                        bool thc_changed = false;

                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
                        ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                        // Card: THC Status & LEDs
                        ImGui::BeginChild("##ThcStatusCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ui::theme::pal().heading, "THC STATUS");
                            ImGui::Spacing();

                            if (ImGui::Checkbox("Enable Real-Time THC Z Tracking", &cfg.thc.enabled)) thc_changed = true;
                            ImGui::SameLine();
                            if (ImGui::Checkbox("Wait Arc OK", &cfg.thc.wait_for_arc_ok)) thc_changed = true;

                            ImGui::BeginChild("##ThcStatusBox", ImVec2(0.0f, telemetry.simulator_mode ? 64.0f : 36.0f), true, ImGuiWindowFlags_NoScrollbar);
                            ImGui::TextColored(telemetry.thc_arc_ok_active ? ui::theme::pal().text_ok : ui::theme::pal().text_muted, "Arc OK");
                            ImGui::SameLine();
                            ImGui::TextColored(telemetry.thc_up_active ? ui::theme::pal().text_info : ui::theme::pal().text_muted, "THC up");
                            ImGui::SameLine();
                            ImGui::TextColored(telemetry.thc_down_active ? ui::theme::pal().text_warn : ui::theme::pal().text_muted, "THC down");
                            ImGui::SameLine();
                            ImGui::TextColored(telemetry.thc_anti_dive_locked ? ui::theme::pal().text_warn : ui::theme::pal().text_muted, "Anti-dive");
                            ImGui::SameLine();
                            ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_Text), "dZ:%+5.2fmm", telemetry.thc_z_offset_mm);

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
                            ImGui::TextColored(ui::theme::pal().heading, "THC SPEEDS");
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
                            ImGui::TextColored(ui::theme::pal().heading, "THC INPUTS");
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
                    // OHMIC SENSE & FLOATING HEAD (IHS / PROBE)
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_probe_settings_open, ImGuiCond_Once);
                    bool probe_hdr_open = ImGui::CollapsingHeader("Initial Height Sensing");
                    if (probe_hdr_open != saved_settings.ui.header_probe_settings_open) {
                        saved_settings.ui.header_probe_settings_open = probe_hdr_open;
                        markSettingsDirty();
                    }
                    if (probe_hdr_open) {
                        core::MachineConfig cfg = *controller.getConfigShared();
                        bool probe_changed = false;

                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
                        ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                        // Card: Probe Pin & Status
                        ImGui::BeginChild("##IhsPinCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                        {
                            ImGui::TextColored(ui::theme::pal().heading, "PROBE INPUT");
                            ImGui::Spacing();

                            ImGui::BeginChild("##PlasmaProbeStatusBox", ImVec2(0.0f, 32.0f), true, ImGuiWindowFlags_NoScrollbar);
                            ImGui::TextColored(
                                telemetry.probe_contact_active ? ui::theme::pal().text_ok : ui::theme::pal().text_muted,
                                telemetry.probe_contact_active ? "Probe: contact" : "Probe: ready (open)");
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
                            ImGui::TextColored(ui::theme::pal().heading, "SWITCH & SEARCH");
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
                        // LASER SOURCE & POWER CONTROL (PWM / M4)
                        // -------------------------------------------------------------
                        ImGui::SetNextItemOpen(saved_settings.ui.header_laser_power_open, ImGuiCond_Once);
                        bool laser_power_hdr_open = ImGui::CollapsingHeader("Laser Power");
                        if (laser_power_hdr_open != saved_settings.ui.header_laser_power_open) {
                            saved_settings.ui.header_laser_power_open = laser_power_hdr_open;
                            markSettingsDirty();
                        }
                        if (laser_power_hdr_open) {
                            core::MachineConfig cfg = *controller.getConfigShared();
                            bool changed = false;

                            ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
                            ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                            // Card: Laser Source Type Dropdown & Mode
                            ImGui::BeginChild("##LaserSourceCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ui::theme::pal().text_danger, "LASER SOURCE TYPE & PRESETS");
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
                                ImGui::TextColored(ui::theme::pal().text_danger, "LIVE LASER POWER & OVERRIDE");
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
                                dl->AddRectFilled(box_p0, box_p1, ui::theme::pal().readout_bg, 4.0f);
                                dl->AddRect(box_p0, box_p1, laser_firing ? ui::theme::pal().readout_laser_border : ui::theme::pal().readout_border, 4.0f, 0, 1.2f);

                                ImFont* gfont = ui::AppWindow::getGCodeFont();
                                float fsz_lbl = 11.0f;
                                float fsz_val = 14.0f;

                                float line1_y = box_p0.y + 7.0f;
                                dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line1_y + 2.0f),
                                            laser_firing ? ui::theme::pal().readout_laser_label : ui::theme::pal().readout_label, "LASER POWER");
                                char pwr_str[32];
                                std::snprintf(pwr_str, sizeof(pwr_str), "%5.1f%%", live_pwr);
                                ImVec2 pwr_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, pwr_str) : ImGui::CalcTextSize(pwr_str);
                                dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - pwr_sz.x - 7.0f, line1_y),
                                            laser_firing ? ui::theme::pal().readout_laser_value : ui::theme::pal().readout_laser_value_idle, pwr_str);

                                float line2_y = box_p0.y + 31.0f;
                                dl->AddText(gfont, fsz_lbl, ImVec2(box_p0.x + 8.0f, line2_y + 2.0f), ui::theme::pal().readout_label_dim, "PWM DUTY");
                                char duty_str[32];
                                std::snprintf(duty_str, sizeof(duty_str), "%4u / 1000", pwm_duty);
                                ImVec2 duty_sz = gfont ? gfont->CalcTextSizeA(fsz_val, 300.0f, 0.0f, duty_str) : ImGui::CalcTextSize(duty_str);
                                dl->AddText(gfont, fsz_val, ImVec2(box_p1.x - duty_sz.x - 7.0f, line2_y), ui::theme::pal().readout_duty, duty_str);

                                ImGui::Dummy(ImVec2(dro_w, dro_h));
                                ImGui::SameLine();

                                ImGui::BeginGroup();
                                {
                                    static float s_drag_laser_ov = 100.0f;
                                    static bool s_is_dragging_laser = false;
                                    if (!s_is_dragging_laser) s_drag_laser_ov = telemetry.laser_power_override_pct;

                                    const bool pwr_ov = (std::abs(telemetry.laser_power_override_pct - 100.0f) >= 0.5f);
                                    if (pwr_ov) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_danger_subtle);
                                        ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::pal().text_danger);
                                    }
                                    if (ImGui::Button("Reset 100%##LaserPwr", ImVec2(right_w, 28.0f))) {
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

                                int s_max = cfg.laser.s_max;
                                ImGui::SetNextItemWidth(140.0f);
                                if (ImGui::InputInt("S for 100% Power", &s_max, 0, 0)) {
                                    cfg.laser.s_max = static_cast<uint16_t>(std::clamp(s_max, 1, 60000));
                                    changed = true;
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("The S value your CAM writes for full power: 1000 (LightBurn, Mach3) or 255.\n"
                                                      "S0 switches the beam off; a program without any S word cuts at full power.");
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
                        // LASER ALIGNMENT, PULSE & FRAMING
                        // -------------------------------------------------------------
                        ImGui::SetNextItemOpen(saved_settings.ui.header_laser_alignment_open, ImGuiCond_Once);
                        bool laser_align_hdr_open = ImGui::CollapsingHeader("Laser Alignment & Framing");
                        if (laser_align_hdr_open != saved_settings.ui.header_laser_alignment_open) {
                            saved_settings.ui.header_laser_alignment_open = laser_align_hdr_open;
                            markSettingsDirty();
                        }
                        if (laser_align_hdr_open) {
                            core::MachineConfig cfg = *controller.getConfigShared();
                            bool changed = false;

                            ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
                            ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                            // Card: Framing Mode & Execution
                            ImGui::BeginChild("##LaserFramingCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ui::theme::pal().heading, "FRAMING");
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
                                ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_primary);
                                ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::contrastText(ui::theme::pal().btn_primary));
                                if (ImGui::Button("FRAME JOB BOUNDING BOX", ImVec2(-1.0f, 36.0f))) {
                                    controller.frameBoundingBox();
                                }
                                ImGui::PopStyleColor(2);
                            }
                            ImGui::EndChild();

                            ImGui::Spacing();

                            // Card: Test Fire / Laser Pulse
                            ImGui::BeginChild("##LaserPulseCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ui::theme::pal().heading, "TEST PULSE");
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
                                ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_danger);
                                ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::contrastText(ui::theme::pal().btn_danger));
                                if (ImGui::Button("FIRE TEST PULSE", ImVec2(-1.0f, 32.0f))) {
                                    controller.testFireLaserPulse(cfg.laser.pulse_duration_ms, cfg.laser.pulse_power_pct);
                                }
                                ImGui::PopStyleColor(2);
                            }
                            ImGui::EndChild();

                            ImGui::Spacing();

                            // Card: Red Dot Crosshair & Offset
                            ImGui::BeginChild("##LaserRedDotCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ui::theme::pal().heading, "RED-DOT POINTER");
                                ImGui::Spacing();

                                const bool red_on = telemetry.laser_red_pointer_active;
                                ImVec4 red_btn_col = red_on ? ui::theme::pal().btn_danger : ui::theme::pal().btn_neutral;
                                ImGui::PushStyleColor(ImGuiCol_Button, red_btn_col);
                                const char* red_lbl = red_on ? "Red-dot pointer on - click to turn off"
                                                             : "Red-dot pointer off - click to turn on";
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

                                if (ImGui::Button("Zero Work Coordinates to Red Dot", ImVec2(-1.0f, 28.0f))) {
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
                        // AIR ASSIST & AUXILIARY OUTPUTS
                        // -------------------------------------------------------------
                        ImGui::SetNextItemOpen(saved_settings.ui.header_laser_air_open, ImGuiCond_Once);
                        bool laser_air_hdr_open = ImGui::CollapsingHeader("Air Assist & Exhaust");
                        if (laser_air_hdr_open != saved_settings.ui.header_laser_air_open) {
                            saved_settings.ui.header_laser_air_open = laser_air_hdr_open;
                            markSettingsDirty();
                        }
                        if (laser_air_hdr_open) {
                            core::MachineConfig cfg = *controller.getConfigShared();
                            bool changed = false;

                            ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
                            ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                            // Card: Air Assist Solenoid
                            ImGui::BeginChild("##LaserAirAssistCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ui::theme::pal().heading, "AIR ASSIST");
                                ImGui::Spacing();

                                if (ImGui::Checkbox("Auto air assist on laser start", &cfg.laser.auto_air_assist)) {
                                    changed = true;
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("When checked, automatically energizes OUT3 air solenoid whenever the laser fires. When unchecked, strictly follows M8/M9 G-code.");
                                }

                                const bool air_on = telemetry.laser_air_assist_active;
                                ImVec4 air_col = air_on ? ui::theme::pal().btn_primary : ui::theme::pal().btn_neutral;
                                ImGui::PushStyleColor(ImGuiCol_Button, air_col);
                                const char* air_lbl = air_on ? "Air assist on - click to stop"
                                                             : "Air assist off - click to start";
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
                                ImGui::TextColored(ui::theme::pal().heading, "EXHAUST FAN");
                                ImGui::Spacing();

                                const bool exhaust_on = (telemetry.outputs_bitmask & core::OutputBit::OUT4_MistCoolant) != 0;
                                ImVec4 ex_col = exhaust_on ? ui::theme::pal().btn_ok : ui::theme::pal().btn_neutral;
                                ImGui::PushStyleColor(ImGuiCol_Button, ex_col);
                                const char* ex_lbl = exhaust_on ? "Exhaust fan on - click to stop"
                                                                : "Exhaust fan off - click to start";
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
                        // LASER SAFETY INTERLOCKS (4-INPUT TERMINALS)
                        // -------------------------------------------------------------
                        ImGui::SetNextItemOpen(saved_settings.ui.header_laser_interlocks_open, ImGuiCond_Once);
                        bool laser_safe_hdr_open = ImGui::CollapsingHeader("Laser Interlocks");
                        if (laser_safe_hdr_open != saved_settings.ui.header_laser_interlocks_open) {
                            saved_settings.ui.header_laser_interlocks_open = laser_safe_hdr_open;
                            markSettingsDirty();
                        }
                        if (laser_safe_hdr_open) {
                            core::MachineConfig cfg = *controller.getConfigShared();
                            bool changed = false;

                            ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::theme::pal().card_bg);
                            ImGui::PushStyleColor(ImGuiCol_Border,  ui::theme::pal().card_border);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
                            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
                            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

                            ImGui::BeginChild("##LaserInterlocksCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                            {
                                ImGui::TextColored(ui::theme::pal().heading, "INTERLOCKS");
                                ImGui::Spacing();

                                ImGui::BeginChild("##LaserSafeStatusBox", ImVec2(0.0f, telemetry.simulator_mode ? 68.0f : 36.0f), true, ImGuiWindowFlags_NoScrollbar);
                                {
                                    ImGui::TextColored(telemetry.laser_chiller_flow_ok ? ui::theme::pal().text_ok : ui::theme::pal().text_danger,
                                                       telemetry.laser_chiller_flow_ok ? "Chiller OK" : "Chiller: no flow");
                                    ImGui::SameLine();
                                    ImGui::TextColored(telemetry.laser_door_closed ? ui::theme::pal().text_ok : ui::theme::pal().text_warn,
                                                       telemetry.laser_door_closed ? "Door closed" : "Lid open");
                                    ImGui::SameLine();
                                    ImGui::TextColored(!telemetry.laser_fault_active ? ui::theme::pal().text_ok : ui::theme::pal().text_danger,
                                                       !telemetry.laser_fault_active ? "PSU ready" : "PSU fault");

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
                                if (ImGui::SliderInt("Water Chiller Pin (0=Off, IN1..12)", &chill_pin, 0, 12)) {
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
                                if (ImGui::SliderInt("Door / Lid Interlock Pin", &door_pin, 0, 12)) {
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
                                if (ImGui::SliderInt("Laser PSU Alarm Pin", &fault_pin, 0, 12)) {
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
                    bool ws_hdr_open = ImGui::CollapsingHeader("Work Offsets");
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
                    bool cam_hdr_open = ImGui::CollapsingHeader("Remote View");
                    if (cam_hdr_open != saved_settings.ui.header_camera_open) {
                        saved_settings.ui.header_camera_open = cam_hdr_open;
                        markSettingsDirty();
                    }
                    if (cam_hdr_open) {
                        if (camera_panel.render(controller, telemetry, nullptr, false)) {
                            saved_settings.machine = *controller.getConfigShared();
                            markSettingsDirty();
                        }
                    }

                    ImGui::Spacing();

                    // -----------------------------------------------------------------
                    // INPUTS & OUTPUTS (PIN ASSIGNMENT)
                    // -----------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_io_open, ImGuiCond_Once);
                    bool io_hdr_open = ImGui::CollapsingHeader("Inputs & Outputs");
                    if (io_hdr_open != saved_settings.ui.header_io_open) {
                        saved_settings.ui.header_io_open = io_hdr_open;
                        markSettingsDirty();
                    }
                    if (io_hdr_open && !host_board) {
                        ui::renderNotUsedByBoardNote("The functions and polarity of the inputs and outputs", board_caps);
                    } else if (io_hdr_open) {
                        core::MachineConfig cfg = *controller.getConfigShared();
                        if (ui::renderIoSettings(controller, telemetry, cfg)) {
                            controller.applyConfig(cfg);
                            markSettingsDirty();
                        }
                    }

                    // -------------------------------------------------------------
                    // END-STOPS, HOMING & SOFT LIMITS
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_limits_open, ImGuiCond_Once);
                    bool limits_hdr_open = ImGui::CollapsingHeader("Limits & Homing");
                    if (limits_hdr_open != saved_settings.ui.header_limits_open) {
                        saved_settings.ui.header_limits_open = limits_hdr_open;
                        markSettingsDirty();
                    }
                    if (limits_hdr_open) {
                ui::renderLimitsHomingSection(settings_ctx);
            }

                    ImGui::Spacing();

                    // -------------------------------------------------------------
                    // MACHINE MOTOR TUNING
                    // -------------------------------------------------------------
                    ImGui::SetNextItemOpen(saved_settings.ui.header_motor_open, ImGuiCond_Once);
                    bool motor_hdr_open = ImGui::CollapsingHeader("Motors");
                    if (motor_hdr_open != saved_settings.ui.header_motor_open) {
                        saved_settings.ui.header_motor_open = motor_hdr_open;
                        markSettingsDirty();
                    }
                    if (motor_hdr_open && !board_caps.app_owns_motor_tuning) {
                        ui::renderNotUsedByBoardNote("Steps per mm, speeds and accelerations", board_caps);
                    } else if (motor_hdr_open) {
                ui::renderMotorsSection(settings_ctx);
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
            ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::pal().text_danger);
            ImGui::TextUnformatted("WARNING: CNC Cycle Is Currently Running!");
            ImGui::PopStyleColor();
            ImGui::Spacing();
            ImGui::TextWrapped("A cycle or motion command is currently running. Exiting the software while the machine is running can cause loss of position or leave torch outputs energized.");
            ImGui::Spacing();
            ImGui::TextWrapped("Clicking 'E-STOP & EXIT' will immediately engage E-Stop, halt all pulse generation and torch control, and safely close the application.");
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::PushStyleColor(ImGuiCol_Button, ui::theme::pal().btn_danger);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ui::theme::pal().btn_danger_hov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ui::theme::pal().btn_danger_act);
            ImGui::PushStyleColor(ImGuiCol_Text, ui::theme::contrastText(ui::theme::pal().btn_danger));
            if (ImGui::Button("E-STOP & EXIT PROGRAM", ImVec2(220.0f, 36.0f))) {
                force_close = true;
                controller.triggerEStop();
                window.closeWindow();
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(4);

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
            saved_settings.machine = *controller.getConfigShared();
            markSettingsDirty();
        }

        // Bottom Hardware I/O Status Bar
        ui::IoStatusBar::renderBottomIoStrip(controller, telemetry);
    });

    // Stop the web server first: its client threads use callbacks that capture locals of main()
    stream_server.stop();

    // Ensure E-Stop is engaged and automatically persist all settings on exit
    controller.triggerEStop();
    saveCurrentSettings();
    return smoke.failed ? 2 : exit_code;
}
