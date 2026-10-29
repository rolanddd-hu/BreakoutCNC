#include "ui_common/camera_panel.h"
#include "imgui.h"
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace rnr::ui {

CameraPanel::CameraPanel(core::CameraManager& cam_mgr, core::CameraStreamServer& stream_srv)
    : cam_mgr_(cam_mgr), stream_srv_(stream_srv)
{
}

bool CameraPanel::render(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    ToolpathViewport* viewport,
    bool enable_top_down_positioning)
{
    if (!devices_initialized_) {
        devices_ = cam_mgr_.enumerateDevices(true);
        devices_initialized_ = true;
    }

    core::MachineConfig cfg = controller.getConfig();
    core::CameraConfig& cam = cfg.camera;
    bool changed = false;

    const bool is_imp = (cfg.linear_units == core::LinearUnits::Imperial);
    const double u_scale   = is_imp ? (1.0 / 25.4) : 1.0;
    const double u_unscale = is_imp ? 25.4 : 1.0;
    const char*  u_lbl     = is_imp ? "in" : "mm";
    const char*  u_fmt     = is_imp ? "%.3f" : "%.2f";

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.14f, 0.18f, 0.70f));
    ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.24f, 0.30f, 0.38f, 0.70f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));

    auto renderFullWidthCameraCombo = [&](const char* combo_id, int& role_idx) -> bool {
        const char* preview = "Disabled / None";
        if (role_idx >= 0 && role_idx < static_cast<int>(devices_.size())) {
            preview = devices_[static_cast<size_t>(role_idx)].name.c_str();
        }
        bool role_changed = false;
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo(combo_id, preview)) {
            if (ImGui::Selectable("Disabled / None", role_idx < 0)) {
                role_idx = -1;
                role_changed = true;
            }
            for (const auto& d : devices_) {
                bool is_sel = (role_idx == d.index);
                if (ImGui::Selectable(d.name.c_str(), is_sel)) {
                    role_idx = d.index;
                    role_changed = true;
                }
            }
            ImGui::EndCombo();
        }
        return role_changed;
    };

    // =========================================================================
    // BREAKOUTMILL: TOP-DOWN POSITION BY CAMERA SECTION
    // =========================================================================
    if (enable_top_down_positioning) {
        const bool top_active  = cam_mgr_.isRoleActive(core::CameraRole::TopDown);
        const bool in_cal_view = (viewport != nullptr && viewport->isCameraCalibrating());
        const bool in_ov_view  = (viewport != nullptr && viewport->isCameraOverlayEnabled());

        // ---------------------------------------------------------------------
        // 1. TOP-DOWN CAMERA SELECTION CARD
        // ---------------------------------------------------------------------
        ImGui::BeginChild("##TopDownCamSelectCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
        {
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "TOP-DOWN ALIGNMENT CAMERA");
            const float refresh_w = 72.0f;
            ImGui::SameLine(std::max(180.0f, ImGui::GetWindowWidth() - refresh_w - 10.0f));
            if (ImGui::SmallButton("Refresh##TopCam")) {
                devices_ = cam_mgr_.enumerateDevices(true);
                cam_mgr_.applyConfig(cam, telemetry.simulator_mode, true);
                changed = true;
            }

            if (renderFullWidthCameraCombo("##TopDownCamCombo", cam.top_down_camera_index)) {
                if (cam.top_down_camera_index >= 0 && cam.top_down_camera_index == cam.monitoring_camera_index) {
                    cam.monitoring_camera_index = -1; // Exclusive role assignment when 1 camera is shared
                }
                if (cam.top_down_camera_index < 0 && viewport != nullptr) {
                    viewport->finishCameraCalibration(false);
                }
                changed = true;
            }

            if (top_active) {
                ImGui::Spacing();
                if (ImGui::Checkbox("Mirror X", &cam.mirror_x)) changed = true;
                ImGui::SameLine(0.0f, 16.0f);
                if (ImGui::Checkbox("Mirror Y", &cam.mirror_y)) changed = true;
                if (in_ov_view) {
                    ImGui::Spacing();
                    float op_pct = cam.overlay_opacity * 100.0f;
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::SliderFloat("Overlay Opacity", &op_pct, 20.0f, 100.0f, "%.0f%%")) {
                        cam.overlay_opacity = std::clamp(op_pct / 100.0f, 0.2f, 1.0f);
                        changed = true;
                    }
                }
            } else {
                ImGui::TextDisabled("Select a top-down camera above to enable optical XY positioning.");
            }
        }
        ImGui::EndChild();

        // ---------------------------------------------------------------------
        // 2. POSITION BY CAMERA OR STEP-BY-STEP CALIBRATION WIZARD
        //    (Only shown when a Top-Down Camera is selected!)
        // ---------------------------------------------------------------------
        if (top_active) {
            ImGui::Spacing();

            if (!in_cal_view) {
                // =============================================================
                // NORMAL OPERATION MODE (Calibration steps hidden!)
                // =============================================================
                ImGui::BeginChild("##CamNormalOpCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                {
                    ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "POSITION BY CAMERA (XY ZERO)");
                    const char* status_txt = cam.is_calibrated ? "[CALIBRATED]" : "[UNCALIBRATED]";
                    const float badge_w = ImGui::CalcTextSize(status_txt).x;
                    ImGui::SameLine(std::max(180.0f, ImGui::GetWindowWidth() - badge_w - 12.0f));
                    if (cam.is_calibrated) {
                        ImGui::TextColored(ImVec4(0.15f, 0.92f, 0.50f, 1.0f), "%s", status_txt);
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "%s", status_txt);
                    }
                    ImGui::Spacing();

                    if (cam.is_calibrated) {
                        double cam_wx = (telemetry.work_pos_mm[0] - cam.offset_x_mm) * u_scale;
                        double cam_wy = (telemetry.work_pos_mm[1] - cam.offset_y_mm) * u_scale;
                        ImGui::Text("Crosshair Target:  X: %+.3f %s   Y: %+.3f %s",
                                    cam_wx, u_lbl, cam_wy, u_lbl);
                        ImGui::Spacing();

                        ImGui::BeginDisabled(telemetry.state == core::ControllerState::RunningGCode);
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.58f, 0.34f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.14f, 0.70f, 0.42f, 1.0f));
                        if (ImGui::Button("ZERO WORK XY AT CAMERA CROSSHAIR", ImVec2(-1.0f, 34.0f))) {
                            controller.setWorkPosition(core::Axis::X, cam.offset_x_mm);
                            controller.setWorkPosition(core::Axis::Y, cam.offset_y_mm);
                        }
                        ImGui::PopStyleColor(2);
                        ImGui::EndDisabled();

                        if (viewport != nullptr) {
                            if (in_ov_view) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.08f, 0.52f, 0.70f, 1.0f));
                            }
                            if (ImGui::Button(in_ov_view
                                                  ? "2D G-CODE OVERLAY: ACTIVE (CLICK TO EXIT)"
                                                  : "SHOW 2D G-CODE CAMERA OVERLAY",
                                              ImVec2(-1.0f, 30.0f)))
                            {
                                if (in_ov_view) {
                                    viewport->setCameraOverlayEnabled(false);
                                } else {
                                    viewport->finishCameraCalibration(true);
                                }
                            }
                            if (in_ov_view) {
                                ImGui::PopStyleColor();
                            }
                        }

                        ImGui::Spacing();
                        ImGui::Separator();
                        ImGui::Spacing();
                        ImGui::TextDisabled("Offset: X %+.2f, Y %+.2f %s  |  FOV: %.1f x %.1f %s",
                                            cam.offset_x_mm * u_scale, cam.offset_y_mm * u_scale, u_lbl,
                                            cam.fov_x_mm * u_scale, cam.fov_y_mm * u_scale, u_lbl);
                        if (ImGui::Button("Recalibrate Camera...", ImVec2(-1.0f, 26.0f)) && viewport != nullptr) {
                            cal_step_ = 0;
                            viewport->startCameraCalibration();
                        }
                    } else {
                        ImGui::TextWrapped("Calibrate the tool-to-camera offset and Field of View once to unlock optical XY zeroing and 2D G-code overlay.");
                        ImGui::Spacing();
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.48f, 0.68f, 1.0f));
                        if (ImGui::Button("START CAMERA CALIBRATION WIZARD", ImVec2(-1.0f, 34.0f)) && viewport != nullptr) {
                            cal_step_ = 0;
                            viewport->startCameraCalibration();
                        }
                        ImGui::PopStyleColor();
                    }
                }
                ImGui::EndChild();
            } else {
                // =============================================================
                // ACTIVE 2-STEP CALIBRATION WIZARD (Shows 1 Step at a Time!)
                // =============================================================
                ImGui::BeginChild("##CamWizardCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.80f, 0.25f, 1.0f),
                                       cal_step_ == 0 ? "CAMERA CALIBRATION — STEP 1 OF 2" : "CAMERA CALIBRATION — STEP 2 OF 2");
                    ImGui::Spacing();

                    // Step selector pills
                    const float half_w = (ImGui::GetContentRegionAvail().x - 8.0f) * 0.5f;
                    if (cal_step_ == 0) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.48f, 0.68f, 1.0f));
                    if (ImGui::Button("1. XY Offset", ImVec2(half_w, 26.0f))) {
                        cal_step_ = 0;
                        if (viewport) viewport->cancelWaitingForDentClick();
                    }
                    if (cal_step_ == 0) ImGui::PopStyleColor();

                    ImGui::SameLine();
                    if (cal_step_ == 1) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.48f, 0.68f, 1.0f));
                    if (ImGui::Button("2. Field of View", ImVec2(half_w, 26.0f))) {
                        cal_step_ = 1;
                    }
                    if (cal_step_ == 1) ImGui::PopStyleColor();

                    ImGui::Spacing();
                    ImGui::Separator();
                    ImGui::Spacing();

                    if (cal_step_ == 0) {
                        // -----------------------------------------------------
                        // STEP 1 ONLY: Tool-to-Camera XY Offset
                        // -----------------------------------------------------
                        ImGui::TextWrapped("1. Move endmill to a test point, click 'Zero XY & Mark Dent', and touch the surface.\n"
                                           "2. Jog XY until the viewport crosshair centers on the dent, then click 'Store Offset'.");
                        ImGui::Spacing();

                        if (ImGui::Button("1A. Zero XY & Mark Dent", ImVec2(half_w, 30.0f))) {
                            controller.zeroWorkAxis(core::Axis::X);
                            controller.zeroWorkAxis(core::Axis::Y);
                            cam_mgr_.makeCalibrationDentAtCurrentToolPos(controller.getTelemetry());
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("1B. Store Offset", ImVec2(half_w, 30.0f))) {
                            cam.offset_x_mm = telemetry.work_pos_mm[0];
                            cam.offset_y_mm = telemetry.work_pos_mm[1];
                            changed = true;
                        }

                        ImGui::Spacing();
                        // Stacked 1-per-row inputs so labels never clip!
                        float off_x = static_cast<float>(cam.offset_x_mm * u_scale);
                        float off_y = static_cast<float>(cam.offset_y_mm * u_scale);
                        char ox_lbl[48], oy_lbl[48];
                        std::snprintf(ox_lbl, sizeof(ox_lbl), "Camera Offset X (%s)", u_lbl);
                        std::snprintf(oy_lbl, sizeof(oy_lbl), "Camera Offset Y (%s)", u_lbl);

                        ImGui::SetNextItemWidth(140.0f);
                        if (ImGui::InputFloat(ox_lbl, &off_x, is_imp ? 0.01f : 0.25f, is_imp ? 0.1f : 2.0f, u_fmt)) {
                            cam.offset_x_mm = off_x * u_unscale;
                            changed = true;
                        }
                        ImGui::SetNextItemWidth(140.0f);
                        if (ImGui::InputFloat(oy_lbl, &off_y, is_imp ? 0.01f : 0.25f, is_imp ? 0.1f : 2.0f, u_fmt)) {
                            cam.offset_y_mm = off_y * u_unscale;
                            changed = true;
                        }

                        ImGui::Spacing();
                        ImGui::Separator();
                        ImGui::Spacing();
                        if (ImGui::Button("Cancel", ImVec2(half_w, 30.0f)) && viewport != nullptr) {
                            viewport->finishCameraCalibration(cam.is_calibrated);
                        }
                        ImGui::SameLine();
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.48f, 0.68f, 1.0f));
                        if (ImGui::Button("Next: Field of View ->", ImVec2(half_w, 30.0f))) {
                            cal_step_ = 1;
                        }
                        ImGui::PopStyleColor();
                    } else {
                        // -----------------------------------------------------
                        // STEP 2 ONLY: Field of View (Move & Click OR Tape Measure)
                        // -----------------------------------------------------
                        ImGui::TextDisabled("Select how you want to measure the camera Field of View:");
                        if (fov_method_ == 0) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.42f, 0.58f, 1.0f));
                        if (ImGui::Button("Method A: Move +X & Click", ImVec2(half_w, 26.0f))) {
                            fov_method_ = 0;
                        }
                        if (fov_method_ == 0) ImGui::PopStyleColor();

                        ImGui::SameLine();
                        if (fov_method_ == 1) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.42f, 0.58f, 1.0f));
                        if (ImGui::Button("Method B: Tape Measure", ImVec2(half_w, 26.0f))) {
                            fov_method_ = 1;
                            if (viewport) viewport->cancelWaitingForDentClick();
                        }
                        if (fov_method_ == 1) ImGui::PopStyleColor();

                        ImGui::Spacing();

                        if (fov_method_ == 0) {
                            ImGui::TextWrapped("With the crosshair on the dent, click below to jog +X by a known distance, then click the displaced dent in the viewport.");
                            ImGui::Spacing();

                            float cal_mv = static_cast<float>(cam.cal_move_x_mm * u_scale);
                            char mv_lbl[48];
                            std::snprintf(mv_lbl, sizeof(mv_lbl), "Test Move +X (%s)", u_lbl);
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::InputFloat(mv_lbl, &cal_mv, is_imp ? 0.05f : 1.0f, is_imp ? 0.25f : 5.0f, u_fmt)) {
                                cam.cal_move_x_mm = std::max(0.5, static_cast<double>(cal_mv) * u_unscale);
                                changed = true;
                            }

                            bool waiting_click = (viewport != nullptr && viewport->isWaitingForDentClick());
                            if (waiting_click) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.82f, 0.55f, 0.10f, 1.0f));
                            }
                            if (ImGui::Button(waiting_click
                                                  ? "WAITING: CLICK DENT IN VIEWPORT..."
                                                  : "JOG +X & CLICK DENT IN VIEWPORT",
                                              ImVec2(-1.0f, 32.0f)))
                            {
                                if (viewport != nullptr) {
                                    controller.jogStep(core::Axis::X, cam.cal_move_x_mm, 1200.0);
                                    const double moved_mm = cam.cal_move_x_mm;
                                    viewport->startWaitingForDentClick([&controller, viewport, moved_mm](double norm_dx, double norm_dy, int img_w, int img_h) {
                                        double norm_dist = std::max(std::abs(norm_dx), std::hypot(norm_dx, norm_dy));
                                        if (norm_dist > 0.01 && img_w > 0 && img_h > 0) {
                                            core::MachineConfig updated = controller.getConfig();
                                            updated.camera.fov_x_mm      = std::clamp(std::abs(moved_mm) / norm_dist, 5.0, 2000.0);
                                            updated.camera.fov_y_mm      = updated.camera.fov_x_mm * (static_cast<double>(img_h) / static_cast<double>(img_w));
                                            updated.camera.is_calibrated = true;
                                            controller.applyConfig(updated);
                                            viewport->finishCameraCalibration(true);
                                        }
                                    });
                                }
                            }
                            if (waiting_click) {
                                ImGui::PopStyleColor();
                            }
                        } else {
                            ImGui::TextWrapped("Place a tape measure/ruler on the workpiece and enter the visible width (X) and height (Y) of the camera view:");
                            ImGui::Spacing();

                            float fov_x = static_cast<float>(cam.fov_x_mm * u_scale);
                            float fov_y = static_cast<float>(cam.fov_y_mm * u_scale);
                            char fx_lbl[48], fy_lbl[48];
                            std::snprintf(fx_lbl, sizeof(fx_lbl), "Visible FOV Width X (%s)", u_lbl);
                            std::snprintf(fy_lbl, sizeof(fy_lbl), "Visible FOV Height Y (%s)", u_lbl);

                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::InputFloat(fx_lbl, &fov_x, is_imp ? 0.1f : 2.0f, is_imp ? 0.5f : 10.0f, u_fmt)) {
                                cam.fov_x_mm = std::clamp(static_cast<double>(fov_x) * u_unscale, 2.0, 2000.0);
                                changed = true;
                            }
                            ImGui::SetNextItemWidth(140.0f);
                            if (ImGui::InputFloat(fy_lbl, &fov_y, is_imp ? 0.1f : 2.0f, is_imp ? 0.5f : 10.0f, u_fmt)) {
                                cam.fov_y_mm = std::clamp(static_cast<double>(fov_y) * u_unscale, 2.0, 2000.0);
                                changed = true;
                            }
                        }

                        ImGui::Spacing();
                        ImGui::Separator();
                        ImGui::Spacing();
                        if (ImGui::Button("<- Back to Step 1", ImVec2(half_w, 30.0f))) {
                            cal_step_ = 0;
                            if (viewport) viewport->cancelWaitingForDentClick();
                        }
                        ImGui::SameLine();
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.58f, 0.34f, 1.0f));
                        if (ImGui::Button("Finish & Switch to Overlay", ImVec2(half_w, 30.0f))) {
                            cam.is_calibrated = true;
                            if (viewport != nullptr) {
                                viewport->finishCameraCalibration(true);
                            }
                            changed = true;
                        }
                        ImGui::PopStyleColor();
                    }
                }
                ImGui::EndChild();
            }
        }

        ImGui::Spacing();
    }

    // =========================================================================
    // 3. OPTIONAL REMOTE WEBCAM STREAMING SECTION (Collapsed by Default in Mill)
    // =========================================================================
    bool remote_open = true;
    if (enable_top_down_positioning) {
        // In BreakoutMill, keep the optional Remote Webcam Streaming section collapsed by default
        // Use CollapsingHeader (or NoTreePushOnOpen) to prevent tree indentation / left padding
        ImGuiTreeNodeFlags flags = cam.stream_enabled ? ImGuiTreeNodeFlags_DefaultOpen : ImGuiTreeNodeFlags_None;
        const char* hdr_label = stream_srv_.isRunning()
                                    ? "Remote Webcam Streaming [LIVE]###RemoteCamSection"
                                    : "Remote Webcam Streaming (Optional)###RemoteCamSection";
        remote_open = ImGui::CollapsingHeader(hdr_label, flags);
    }

    if (remote_open) {
        ImGui::BeginChild("##CamStreamCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
        {
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "MONITORING CAMERA & WEB STREAM");
            const float refresh_w = 72.0f;
            ImGui::SameLine(std::max(180.0f, ImGui::GetWindowWidth() - refresh_w - 10.0f));
            if (ImGui::SmallButton("Refresh##MonCam")) {
                devices_ = cam_mgr_.enumerateDevices(true);
                cam_mgr_.applyConfig(cam, telemetry.simulator_mode, enable_top_down_positioning);
                changed = true;
            }

            if (renderFullWidthCameraCombo("##MonCamCombo", cam.monitoring_camera_index)) {
                if (cam.monitoring_camera_index >= 0 && cam.monitoring_camera_index == cam.top_down_camera_index) {
                    cam.top_down_camera_index = -1; // Exclusive role assignment when 1 camera is shared
                }
                changed = true;
            }

            ImGui::Spacing();
            if (ImGui::Checkbox("Enable HTTP/MJPEG Remote Monitoring Server", &cam.stream_enabled)) {
                changed = true;
            }

            // Only show server port/FPS/URL details when streaming is enabled!
            if (cam.stream_enabled) {
                ImGui::Spacing();
                int port_val = static_cast<int>(cam.stream_port);
                ImGui::SetNextItemWidth(140.0f);
                if (ImGui::InputInt("HTTP Server Port", &port_val, 1, 10)) {
                    cam.stream_port = static_cast<uint16_t>(std::clamp(port_val, 1024, 65535));
                    changed = true;
                }

                ImGui::SetNextItemWidth(140.0f);
                if (ImGui::SliderInt("Stream Frame Rate", &cam.stream_fps, 5, 30, "%d FPS")) {
                    changed = true;
                }

                if (ImGui::Checkbox("Allow remote G-code upload", &cam.stream_allow_upload)) {
                    changed = true;
                }
                ImGui::PushTextWrapPos(0.0f);
                ImGui::TextDisabled("The server has no password: anyone on your network can open it. "
                                    "Uploads are refused while a job is active.");
                ImGui::PopTextWrapPos();

                if (stream_srv_.isRunning()) {
                    std::string url = stream_srv_.getServerUrl();
                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.0f, 0.90f, 1.0f, 1.0f), "URL: %s", url.c_str());
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Copy URL")) {
                        ImGui::SetClipboardText(url.c_str());
                    }
                    ImGui::TextDisabled("Active Viewers: %d  |  Includes live DRO & Remote E-STOP",
                                        stream_srv_.getActiveClientCount());
                }
            }
        }
        ImGui::EndChild();
    }

    ImGui::PopStyleVar(4);
    ImGui::PopStyleColor(2);

    if (changed) {
        cam_mgr_.applyConfig(cam, telemetry.simulator_mode, enable_top_down_positioning);
        stream_srv_.applyConfig(cam);
        controller.applyConfig(cfg);
    }

    return changed;
}

} // namespace rnr::ui
