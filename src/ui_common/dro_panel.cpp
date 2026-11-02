#include "ui_common/dro_panel.h"
#include "ui_common/app_window.h"
#include "ui_common/frame_stats.h"
#include "ui_common/theme.h"
#include "ui_common/gcode_panel.h"
#include "core/scripting/script_path_helper.h"
#include "imgui.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>

namespace rnr::ui {
namespace {

void addArcPoints(std::vector<ImVec2>& pts, ImVec2 center, float radius, float a_min, float a_max, int num_segments) {
    for (int i = 0; i <= num_segments; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(num_segments);
        float a = a_min + (a_max - a_min) * t;
        pts.push_back(ImVec2(center.x + std::cos(a) * radius, center.y + std::sin(a) * radius));
    }
}

// Builds a closed polygon for the symmetrical X/Y Cross with rounded caps and concave fillets
void buildCrossPolygon(std::vector<ImVec2>& pts, ImVec2 center, float cross_sz, float arm_w, float cap_r, float fillet_r, int arc_steps = 8) {
    pts.clear();
    const float cx = center.x;
    const float cy = center.y;
    const float hw = arm_w * 0.5f;
    const float hr = cross_sz * 0.5f;
    const float rc = std::clamp(cap_r, 2.0f, hw * 0.9f);
    const float rf = std::max(2.0f, fillet_r);

    // Top Arm Cap (top-left rounded corner -> top-right rounded corner)
    addArcPoints(pts, ImVec2(cx - hw + rc, cy - hr + rc), rc, 3.14159265f, 4.71238898f, arc_steps);
    addArcPoints(pts, ImVec2(cx + hw - rc, cy - hr + rc), rc, 4.71238898f, 6.28318531f, arc_steps);

    // Top-Right Inner Fillet (concave arc from top arm to right arm)
    addArcPoints(pts, ImVec2(cx + hw + rf, cy - hw - rf), rf, 3.14159265f, 1.57079633f, arc_steps);

    // Right Arm Cap (top-right rounded corner -> bottom-right rounded corner)
    addArcPoints(pts, ImVec2(cx + hr - rc, cy - hw + rc), rc, 4.71238898f, 6.28318531f, arc_steps);
    addArcPoints(pts, ImVec2(cx + hr - rc, cy + hw - rc), rc, 0.0f, 1.57079633f, arc_steps);

    // Bottom-Right Inner Fillet (concave arc from right arm to bottom arm)
    addArcPoints(pts, ImVec2(cx + hw + rf, cy + hw + rf), rf, 4.71238898f, 3.14159265f, arc_steps);

    // Bottom Arm Cap (bottom-right rounded corner -> bottom-left rounded corner)
    addArcPoints(pts, ImVec2(cx + hw - rc, cy + hr - rc), rc, 0.0f, 1.57079633f, arc_steps);
    addArcPoints(pts, ImVec2(cx - hw + rc, cy + hr - rc), rc, 1.57079633f, 3.14159265f, arc_steps);

    // Bottom-Left Inner Fillet (concave arc from bottom arm to left arm)
    addArcPoints(pts, ImVec2(cx - hw - rf, cy + hw + rf), rf, 6.28318531f, 4.71238898f, arc_steps);

    // Left Arm Cap (bottom-left rounded corner -> top-left rounded corner)
    addArcPoints(pts, ImVec2(cx - hr + rc, cy + hw - rc), rc, 1.57079633f, 3.14159265f, arc_steps);
    addArcPoints(pts, ImVec2(cx - hr + rc, cy - hw + rc), rc, 3.14159265f, 4.71238898f, arc_steps);

    // Top-Left Inner Fillet (concave arc from left arm to top arm)
    addArcPoints(pts, ImVec2(cx - hw - rf, cy - hw - rf), rf, 1.57079633f, 0.0f, arc_steps);
}

// Builds a closed polygon for the vertical Stadium / Pill shape
void buildPillPolygon(std::vector<ImVec2>& pts, ImVec2 center, float width, float height, int arc_steps = 16) {
    pts.clear();
    const float cx = center.x;
    const float cy = center.y;
    const float r = width * 0.5f;
    const float dy = std::max(0.0f, (height - width) * 0.5f);

    // Top hemisphere (left to right)
    addArcPoints(pts, ImVec2(cx, cy - dy), r, 3.14159265f, 6.28318531f, arc_steps);
    // Bottom hemisphere (right to left)
    addArcPoints(pts, ImVec2(cx, cy + dy), r, 0.0f, 3.14159265f, arc_steps);
}

// Renders the multi-pass drop shadow, 3D molded directional bevel, raised matte face, and specular ridge
void renderMoldedPadWithShadow(
    ImDrawList* dl,
    const std::vector<ImVec2>& outer_pts,
    const std::vector<ImVec2>& inner_pts,
    float scale)
{
    const size_t n = outer_pts.size();
    if (n < 3 || inner_pts.size() != n) return;
    const theme::Palette& pal = theme::pal();
    auto shadow = [&](int a) { return IM_COL32(0, 0, 0, static_cast<int>(a * pal.jog_shadow)); };

    const float sh_dx = 1.0f * scale;
    const float sh_dy = 3.5f * scale;

    std::vector<ImVec2> sh_pts(n);
    auto shiftPts = [&](float dx, float dy) {
        for (size_t i = 0; i < n; ++i) {
            sh_pts[i] = ImVec2(outer_pts[i].x + dx, outer_pts[i].y + dy);
        }
    };

    // Soft multi-layer drop shadow: ambient, medium, contact
    shiftPts(sh_dx * 2.0f, sh_dy * 2.0f);
    dl->AddPolyline(sh_pts.data(), static_cast<int>(n), shadow(22), 12.0f * scale, ImDrawFlags_Closed);
    dl->AddConcavePolyFilled(sh_pts.data(), static_cast<int>(n), shadow(25));

    shiftPts(sh_dx * 1.3f, sh_dy * 1.3f);
    dl->AddPolyline(sh_pts.data(), static_cast<int>(n), shadow(45), 6.5f * scale, ImDrawFlags_Closed);
    dl->AddConcavePolyFilled(sh_pts.data(), static_cast<int>(n), shadow(50));

    shiftPts(sh_dx * 0.8f, sh_dy * 0.8f);
    dl->AddPolyline(sh_pts.data(), static_cast<int>(n), shadow(80), 3.0f * scale, ImDrawFlags_Closed);
    dl->AddConcavePolyFilled(sh_pts.data(), static_cast<int>(n), shadow(110));

    // Pad body
    dl->AddConcavePolyFilled(outer_pts.data(), static_cast<int>(n), pal.jog_body);

    // Bevel rim, shaded by how much each edge faces the light
    for (size_t i = 0; i < n; ++i) {
        size_t next = (i + 1 == n) ? 0 : i + 1;
        ImVec2 o0 = outer_pts[i];
        ImVec2 o1 = outer_pts[next];
        ImVec2 i0 = inner_pts[i];
        ImVec2 i1 = inner_pts[next];

        float vx = o1.x - o0.x;
        float vy = o1.y - o0.y;
        float len = std::sqrt(vx * vx + vy * vy);
        if (len < 0.0001f) continue;
        float nx = vy / len;
        float ny = -vx / len;

        // Light direction from top-left: (-0.55, -0.835)
        float intensity = -(0.55f * nx + 0.835f * ny);

        const ImU32 col_bevel = (intensity > 0.0f)
                                    ? theme::lerp(pal.jog_bevel_mid, pal.jog_bevel_lit, std::min(1.0f, intensity))
                                    : theme::lerp(pal.jog_bevel_mid, pal.jog_bevel_shade, std::min(1.0f, -intensity));
        dl->AddQuadFilled(o0, o1, i1, i0, col_bevel);
    }

    // Raised matte face
    dl->AddConcavePolyFilled(inner_pts.data(), static_cast<int>(inner_pts.size()), pal.jog_face);

    // Highlight along the top-left ridge
    for (size_t i = 0; i < n; ++i) {
        size_t next = (i + 1 == n) ? 0 : i + 1;
        ImVec2 i0 = inner_pts[i];
        ImVec2 i1 = inner_pts[next];
        float vx = i1.x - i0.x;
        float vy = i1.y - i0.y;
        float len = std::sqrt(vx * vx + vy * vy);
        if (len < 0.0001f) continue;
        float nx = vy / len;
        float ny = -vx / len;
        float intensity = -(0.55f * nx + 0.835f * ny);
        if (intensity > 0.20f) {
            int alpha = static_cast<int>(160.0f * intensity);
            dl->AddLine(i0, i1, theme::withAlpha(pal.jog_ridge, alpha), 1.0f);
        }
    }
}

// Renders the 3D circular dial knob matching the target design
void renderCenterKnob(
    ImDrawList* dl,
    ImVec2 center,
    float knob_r,
    const char* label,
    ImFont* font,
    float font_sz,
    bool is_active,
    bool is_hovered,
    ImVec2 drag_offset)
{
    const ImVec2 knob_pos = ImVec2(center.x + drag_offset.x, center.y + drag_offset.y);
    const theme::Palette& pal = theme::pal();
    auto shadow = [&](int a) { return IM_COL32(0, 0, 0, static_cast<int>(a * pal.jog_shadow)); };

    // Recessed groove in the pad face
    dl->AddCircle(center, knob_r + 2.5f, pal.jog_groove, 32, 2.0f);

    // Drop shadow beneath the knob
    dl->AddCircleFilled(ImVec2(knob_pos.x + 1.2f, knob_pos.y + 4.0f), knob_r + 1.0f, shadow(75), 32);
    dl->AddCircleFilled(ImVec2(knob_pos.x + 0.8f, knob_pos.y + 2.5f), knob_r, shadow(170), 32);

    // Knob body
    const ImU32 col_knob_base = is_active ? pal.knob_base_active : (is_hovered ? pal.knob_base_hover : pal.knob_base);
    const ImU32 col_knob_face = is_active ? pal.knob_face_active : (is_hovered ? pal.knob_face_hover : pal.knob_face);
    dl->AddCircleFilled(knob_pos, knob_r, col_knob_base, 32);
    dl->AddCircleFilled(knob_pos, knob_r - 1.2f, col_knob_face, 32);

    // Top-left highlight and bottom-right rim shadow
    dl->PathArcTo(knob_pos, knob_r - 1.0f, -2.85f, -0.65f, 20);
    dl->PathStroke(pal.knob_highlight, 0, 1.8f);
    dl->PathArcTo(knob_pos, knob_r - 1.0f, 0.35f, 2.35f, 20);
    dl->PathStroke(pal.knob_rim_shadow, 0, 1.8f);

    // Concave dimple in the center
    const float dimple_r = knob_r * 0.60f;
    dl->AddCircleFilled(knob_pos, dimple_r, pal.knob_dimple, 32);
    dl->PathArcTo(knob_pos, dimple_r - 0.8f, -2.8f, -0.3f, 16);
    dl->PathStroke(pal.knob_dimple_shadow, 0, 1.2f);

    if (is_active) {
        dl->AddCircle(knob_pos, knob_r, pal.jog_accent, 32, 2.0f);
    }

    // Centered Bold White Text
    const ImU32 text_col = is_active ? theme::withAlpha(pal.knob_text, 255) : pal.knob_text;
    if (font) {
        const ImVec2 sz = font->CalcTextSizeA(font_sz, 500.0f, 0.0f, label);
        dl->AddText(font, font_sz, ImVec2(knob_pos.x - sz.x * 0.5f, knob_pos.y - sz.y * 0.5f), text_col, label);
    } else {
        const ImVec2 sz = ImGui::CalcTextSize(label);
        dl->AddText(ImVec2(knob_pos.x - sz.x * 0.5f, knob_pos.y - sz.y * 0.5f), text_col, label);
    }
}

} // namespace

void DroAndJogPanel::render(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry,
    bool show_fourth_axis_a,
    float allocated_h,
    GCodeProgramPanel* gcode_panel)
{
    perf::ScopedTimer perf_timer("DRO + jog panel");
    float start_cursor_y = ImGui::GetCursorPosY();
    // A gets its own jog controls only when it is a rotary / independent axis, not a gantry slave
    a_jog_enabled_ = show_fourth_axis_a && controller.getConfig().gantryMasterAxis() < 0;
    handleKeyboardJog(controller, telemetry);

    // Compact mode for narrow side panels (e.g. 325px Minimal layout): hide MPos/Vel column,
    // shorten action labels and shrink the jog pad so everything fits without clipping.
    // In Full layout (450px left panel), compact is false so full MPos & Vel readouts are shown.
    const bool compact = ImGui::GetContentRegionAvail().x < 400.0f;

    ImGui::SeparatorText(compact ? "POSITION" : "POSITION (WORK / MACHINE)");

    const char* axis_names[4] = {"X", "Y", "Z", "A"};
    const theme::Palette& pal = theme::pal();
    const ImVec4* axis_cols = pal.dro_axis; // X red-coral, Y emerald, Z sky cyan, A amber

    const int num_axes = show_fourth_axis_a ? 4 : 3;
    ImFont* dro_font   = AppWindow::getDroFont();
    ImFont* mono_font  = AppWindow::getGCodeFont();
    const auto& active_cfg = controller.getConfig();
    const bool is_imperial = (active_cfg.linear_units == core::LinearUnits::Imperial);
    const double pos_scale = is_imperial ? (1.0 / 25.4) : 1.0;

    for (int i = 0; i < num_axes; ++i) {
        ImGui::PushID(i);
        ImGui::BeginChild("##AxisDroRow", ImVec2(0.0f, 54.0f), true, ImGuiWindowFlags_NoScrollbar);

        // Recessed digital readout screen background for axis coordinate
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 row_p0  = ImGui::GetWindowPos();
        dl->AddRectFilled(
            ImVec2(row_p0.x + 6.0f, row_p0.y + 5.0f),
            ImVec2(row_p0.x + 198.0f, row_p0.y + 49.0f),
            theme::pal().dro_bg,
            4.0f);
        dl->AddRect(
            ImVec2(row_p0.x + 6.0f, row_p0.y + 5.0f),
            ImVec2(row_p0.x + 198.0f, row_p0.y + 49.0f),
            telemetry.axis_homed[i] ? theme::pal().dro_border_homed : theme::pal().dro_border,
            4.0f);

        if (dro_font) ImGui::PushFont(dro_font);
        // Axis letter in the axis color, digits neutral
        ImGui::PushStyleColor(ImGuiCol_Text, axis_cols[i]);
        ImGui::SetCursorPos(ImVec2(12.0f, 15.0f));
        ImGui::Text("%s", axis_names[i]);
        ImGui::PopStyleColor();
        ImGui::PushStyleColor(ImGuiCol_Text, pal.dro_digits);
        ImGui::SameLine(38.0f);
        ImGui::SetCursorPosY(15.0f);
        if (is_imperial) {
            ImGui::Text("%+9.4f\"", telemetry.work_pos_mm[i] * pos_scale);
        } else {
            ImGui::Text("%+9.3f", telemetry.work_pos_mm[i]);
        }
        ImGui::PopStyleColor();
        if (dro_font) ImGui::PopFont();

        if (!compact) {
            ImGui::SameLine(206.0f);
            ImGui::SetCursorPosY(7.0f);
            if (mono_font) ImGui::PushFont(mono_font);
            ImGui::BeginGroup();
            if (is_imperial) {
                ImGui::TextColored(pal.dro_secondary, "MPos:%+8.3f\"", telemetry.machine_pos_mm[i] * pos_scale);
                double axis_vel_ipm = telemetry.axis_vel_mm_min[i] * pos_scale;
                if (telemetry.axis_homed[i]) {
                    ImGui::TextColored(pal.dro_homed, "Vel:%4.0f IPM", axis_vel_ipm);
                } else {
                    ImGui::TextColored(pal.dro_secondary, "Vel:%4.0f IPM", axis_vel_ipm);
                }
                ImGui::EndGroup();
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s Axis: Machine Pos: %+9.4f in\nLive Velocity: %.1f IPM",
                                      axis_names[i], telemetry.machine_pos_mm[i] * pos_scale, axis_vel_ipm);
                }
            } else {
                ImGui::TextColored(pal.dro_secondary, "MPos:%+8.2f", telemetry.machine_pos_mm[i]);
                double axis_vel_mm_s = telemetry.axis_vel_mm_min[i] / 60.0;
                if (telemetry.axis_homed[i]) {
                    ImGui::TextColored(pal.dro_homed, "Vel:%4.0f mm/m", telemetry.axis_vel_mm_min[i]);
                } else {
                    ImGui::TextColored(pal.dro_secondary, "Vel:%4.0f mm/m", telemetry.axis_vel_mm_min[i]);
                }
                ImGui::EndGroup();
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s Axis: Machine Pos: %+9.3f mm\nLive Velocity: %.0f mm/m (%.1f mm/s)",
                                      axis_names[i], telemetry.machine_pos_mm[i], telemetry.axis_vel_mm_min[i], axis_vel_mm_s);
                }
            }
            if (mono_font) ImGui::PopFont();
        }

        // Per-axis Home button + Zero button on the right
        ImGui::SameLine(ImGui::GetWindowWidth() - 104.0f);
        ImGui::SetCursorPosY(9.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, telemetry.axis_homed[i]
            ? theme::pal().btn_ok
            : theme::pal().btn_neutral);
        if (ImGui::Button("H", ImVec2(28.0f, 36.0f))) {
            controller.homeAxis(static_cast<core::Axis>(i));
        }
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Home %s Axis to Home/Limit Switch", axis_names[i]);
        }

        ImGui::SameLine(ImGui::GetWindowWidth() - 70.0f);
        ImGui::SetCursorPosY(9.0f);
        char zero_btn[32];
        std::snprintf(zero_btn, sizeof(zero_btn), "Zero %s", axis_names[i]);
        if (ImGui::Button(zero_btn, ImVec2(62.0f, 36.0f))) {
            controller.zeroWorkAxis(static_cast<core::Axis>(i));
        }

        ImGui::EndChild();
        ImGui::PopID();
    }

    // Action row: ZERO ALL | HOME ALL | GO TO ZERO | PROBE Z
    const float avail_w = ImGui::GetContentRegionAvail().x;
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float btn_w   = std::floor((avail_w - spacing * 3.0f) / 4.0f);

    if (ImGui::Button("ZERO ALL", ImVec2(btn_w, 34.0f))) {
        controller.zeroAllWorkAxes();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Zero Work Coordinate for All Axes (X, Y, Z, A)");
    }

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_neutral);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_neutral_hov);
    if (ImGui::Button(compact ? "HOME" : "HOME ALL", ImVec2(btn_w, 34.0f))) {
        controller.homeAllAxes();
    }
    ImGui::PopStyleColor(2);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Run Multi-Axis Homing Sequence (Z Up -> Y -> X) with Pull-Off & Zeroing");
    }

    ImGui::SameLine();
    if (ImGui::Button(compact ? "GO TO 0" : "GO TO ZERO", ImVec2(btn_w, 34.0f))) {
        controller.goToWorkZeroXY();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Rapid Traverse to Work Zero (XY) at Safe Z Height");
    }

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_neutral);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_neutral_hov);
    if (ImGui::Button("PROBE Z", ImVec2(-1.0f, 34.0f))) {
        show_probe_z_modal_ = true;
    }
    ImGui::PopStyleColor(2);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Open Z Probe Touch-Off Dialog (Z Plate or Direct Workpiece Surface)");
    }

    // Live Spindle Speed / Progress Bar Indicator (mill only: the cutter's torch / laser has its own panel)
    if (active_cfg.is_mill) {
        ImGui::Spacing();
        char ramp_overlay[96];
        uint32_t cmd_rpm = telemetry.commanded_spindle_rpm > 0 ? telemetry.commanded_spindle_rpm : 12000;
        int live_rpm = (active_cfg.vfd.enabled && telemetry.vfd_telemetry.connected)
                           ? static_cast<int>(telemetry.vfd_telemetry.actual_rpm)
                           : static_cast<int>(std::lround(cmd_rpm * (telemetry.spindle_speed_pct / 100.0f)));

        if (active_cfg.vfd.enabled && telemetry.vfd_telemetry.connected) {
            std::snprintf(ramp_overlay, sizeof(ramp_overlay),
                          "VFD: %d RPM | %3.1f Hz | %2.1f A | %2.0f C",
                          live_rpm, telemetry.vfd_telemetry.actual_freq_hz,
                          telemetry.vfd_telemetry.actual_amps,
                          telemetry.vfd_telemetry.temperature_c);
        } else {
            std::snprintf(ramp_overlay, sizeof(ramp_overlay),
                          "Spindle Speed: %3.0f%% (%d / %d RPM)",
                          telemetry.spindle_speed_pct, live_rpm, cmd_rpm);
        }

        ImGui::PushStyleColor(
            ImGuiCol_PlotHistogram,
            telemetry.spindle_ramping_up
                ? theme::pal().btn_warn
                : (telemetry.torch_or_spindle_on ? theme::pal().btn_ok
                                                 : theme::pal().btn_neutral));
        ImGui::ProgressBar(telemetry.spindle_speed_pct / 100.0f, ImVec2(-1.0f, 20.0f), ramp_overlay);
        ImGui::PopStyleColor();
    }

    ImGui::Spacing();
    ImGui::SeparatorText("JOG");

    const float pad_avail_w  = ImGui::GetContentRegionAvail().x;
    const bool  is_wide      = (pad_avail_w >= 500.0f);

    // Uniform base geometry
    const float base_cross_sz = is_wide ? 260.0f : 200.0f;
    const float base_arm_w    = is_wide ? 90.0f  : 70.0f;
    const float base_gap      = is_wide ? 26.0f  : 20.0f;
    const float base_hub_r    = is_wide ? 38.0f  : 30.0f;
    const float base_rounding = is_wide ? 22.0f  : 18.0f;
    const float base_fillet   = is_wide ? 14.0f  : 11.0f;
    const float base_th       = is_wide ? 30.0f  : 24.0f;
    const float base_tb       = is_wide ? 46.0f  : 36.0f;
    const float base_font_sz  = is_wide ? 24.0f  : 20.0f;

    // Fixed base width so other controls (toggle, step buttons, slider, STOP JOG) stay the same size:
    // the X/Y cross plus the Z pill, and the A pill when A is jogged on its own
    const int   pill_count    = a_jog_enabled_ ? 2 : 1;
    const float base_total_w  = base_cross_sz + pill_count * (base_gap + base_arm_w); // 376 / 290 without A
    const float target_w      = std::min(base_total_w, pad_avail_w);
    const float pad_offset_x  = std::max(0.0f, (pad_avail_w - target_w) * 0.5f);

    if (allocated_h > 0.0f) {
        allocated_height_ = allocated_h;
    }

    // When the user drags the bottom of the panel up (allocated_height_ < max_height_),
    // the jog-pads shrink in size while all other controls maintain their size.
    float cross_sz = base_cross_sz;
    if (allocated_height_ > 0.0f && max_height_ > 0.0f && allocated_height_ < max_height_) {
        float deficit = max_height_ - allocated_height_;
        cross_sz = std::max(70.0f, base_cross_sz - deficit);
    }
    // Narrow panel: shrink the pad so the cross and every pill fit the width
    if (base_total_w > pad_avail_w) {
        cross_sz = std::min(cross_sz, std::max(70.0f, base_cross_sz * pad_avail_w / base_total_w));
    }

    const float scale        = cross_sz / base_cross_sz;
    const float arm_w        = std::round(base_arm_w * scale);
    const float gap          = std::round(base_gap * scale);
    const float z_w          = arm_w;
    const float z_h          = cross_sz;
    const float total_pad_w  = cross_sz + pill_count * (gap + z_w);
    const float hub_r        = std::round(base_hub_r * scale);
    const float arm_rounding = std::round(base_rounding * scale);
    const float fillet_r     = std::round(base_fillet * scale);
    const float arm_inset    = (cross_sz - arm_w) * 0.5f;

    // Horizontally center the (potentially shrunk) jog pad within the panel
    const float jog_pad_offset_x = pad_offset_x + (target_w - total_pad_w) * 0.5f;

    // Centered Segmented Tab Toggle (CONTINUOUS HOLD / STEP INCREMENT) matching exact target_w
    const float mode_tab_w0  = std::floor(target_w * 0.5f);
    const float mode_tab_w1  = target_w - mode_tab_w0;
    const float mode_tab_h   = 34.0f;

    ImGui::SetCursorPosX(pad_offset_x);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);

    const char* mode_labels[2] = {"CONTINUOUS HOLD", "STEP INCREMENT"};
    for (int t = 0; t < 2; ++t) {
        const bool selected = ((t == 1) == step_jog_mode_);
        ImGui::PushStyleColor(ImGuiCol_Button, selected ? theme::pal().btn_primary
                                                        : theme::pal().btn_neutral);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, selected ? theme::pal().btn_primary_hov
                                                               : theme::pal().btn_neutral_hov);
        ImGui::PushStyleColor(ImGuiCol_Text, selected ? theme::pal().text_on_fill
                                                      : theme::pal().text_muted);
        if (t > 0) ImGui::SameLine();
        const float this_tab_w = (t == 0) ? mode_tab_w0 : mode_tab_w1;
        if (ImGui::Button(mode_labels[t], ImVec2(this_tab_w, mode_tab_h))) {
            step_jog_mode_ = (t == 1);
        }
        ImGui::PopStyleColor(3);
    }
    ImGui::PopStyleVar(2);

    // Large Centered Incremental Step Buttons spanning exact target_w (when in Step Mode)
    if (step_jog_mode_) {
        ImGui::Spacing();
        const float step_gap    = 8.0f;
        const float step_btn_w  = std::floor((target_w - 3.0f * step_gap) / 4.0f);
        const float last_btn_w  = target_w - (3.0f * (step_btn_w + step_gap));
        const float step_btn_h  = 34.0f;

        ImGui::SetCursorPosX(pad_offset_x);

        const float steps_metric[4]       = {0.01f, 0.1f, 1.0f, 10.0f};
        const char* step_labels_metric[4] = {"0.01 mm", "0.1 mm", "1.0 mm", "10 mm"};

        const float steps_imperial[4]       = {0.001f * 25.4f, 0.01f * 25.4f, 0.1f * 25.4f, 1.0f * 25.4f};
        const char* step_labels_imperial[4] = {"0.001 in", "0.01 in", "0.1 in", "1.0 in"};

        const float* steps_ptr       = is_imperial ? steps_imperial : steps_metric;
        const char* const* labels_ptr = is_imperial ? step_labels_imperial : step_labels_metric;

        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
        for (int s = 0; s < 4; ++s) {
            if (s > 0) ImGui::SameLine(0.0f, step_gap);
            const bool selected = (std::abs(step_size_mm_ - steps_ptr[s]) < 0.005f);

            ImGui::PushStyleColor(ImGuiCol_Button, selected ? theme::pal().btn_primary
                                                            : theme::pal().btn_neutral);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, selected ? theme::pal().btn_primary_hov
                                                                   : theme::pal().btn_neutral_hov);
            ImGui::PushStyleColor(ImGuiCol_Text, selected ? theme::pal().text_on_fill
                                                          : theme::pal().text_muted);

            const float cur_w = (s == 3) ? last_btn_w : step_btn_w;
            if (ImGui::Button(labels_ptr[s], ImVec2(cur_w, step_btn_h))) {
                step_size_mm_ = steps_ptr[s];
            }
            ImGui::PopStyleColor(3);
        }
        ImGui::PopStyleVar();
    }

    ImGui::Spacing();

    // Centered Jog Speed % Slider (Symmetrically matching exact target_w)
    ImGui::SetCursorPosX(pad_offset_x);
    ImGui::SetNextItemWidth(target_w);
    ImGui::SliderFloat("##JogSpeed", &jog_speed_pct_, 5.0f, 100.0f, "Jog Speed: %.0f%%");
    ImGui::Spacing();

    // Custom Vector Graphics Jog Pad with Unified Rubber Depth & Molded Shadow
    ImGui::SetCursorPosX(jog_pad_offset_x);
    const ImVec2 screen_pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const ImVec2 P_xy = screen_pos;
    const ImVec2 P_z  = ImVec2(P_xy.x + cross_sz + gap, P_xy.y);
    const ImVec2 C_xy = ImVec2(P_xy.x + cross_sz * 0.5f, P_xy.y + cross_sz * 0.5f);
    const ImVec2 C_z  = ImVec2(P_z.x + z_w * 0.5f, P_z.y + z_h * 0.5f);
    const ImVec2 P_a  = ImVec2(P_z.x + z_w + gap, P_xy.y);
    const ImVec2 C_a  = ImVec2(P_a.x + z_w * 0.5f, P_a.y + z_h * 0.5f);

    const float bevel_w = std::max(3.0f, std::round(4.5f * scale));

    // Build outer and inner polygons for X/Y Cross
    std::vector<ImVec2> pts_cross_outer, pts_cross_inner;
    buildCrossPolygon(pts_cross_outer, C_xy, cross_sz, arm_w, arm_rounding, fillet_r, 8);
    buildCrossPolygon(pts_cross_inner, C_xy, cross_sz - bevel_w * 2.0f, arm_w - bevel_w * 2.0f,
                      std::max(2.0f, arm_rounding - bevel_w), fillet_r + bevel_w, 8);

    // Build outer and inner polygons for Z Pill
    std::vector<ImVec2> pts_pill_outer, pts_pill_inner;
    buildPillPolygon(pts_pill_outer, C_z, z_w, z_h, 16);
    buildPillPolygon(pts_pill_inner, C_z, z_w - bevel_w * 2.0f, z_h - bevel_w * 2.0f, 16);

    renderMoldedPadWithShadow(dl, pts_cross_outer, pts_cross_inner, scale);
    renderMoldedPadWithShadow(dl, pts_pill_outer, pts_pill_inner, scale);
    if (a_jog_enabled_) {
        std::vector<ImVec2> pts_a_outer, pts_a_inner;
        buildPillPolygon(pts_a_outer, C_a, z_w, z_h, 16);
        buildPillPolygon(pts_a_inner, C_a, z_w - bevel_w * 2.0f, z_h - bevel_w * 2.0f, 16);
        renderMoldedPadWithShadow(dl, pts_a_outer, pts_a_inner, scale);
    }

    // Virtual Analog Joystick Interaction & State Updates (Continuous Mode Only)
    // Safety check: if EStop occurred, immediately cancel any active joystick drag
    if (telemetry.state == core::ControllerState::EStop) {
        js_xy_active_ = false;
        js_xy_drag_   = ImVec2(0.0f, 0.0f);
        js_pill_active_ = {false, false};
        js_pill_drag_   = {0.0f, 0.0f};
    }
    if (!a_jog_enabled_ && js_pill_active_[1]) {
        js_pill_active_[1] = false;
        js_pill_drag_[1]   = 0.0f;
        controller.jogStopAxis(core::Axis::A);
    }

    const float max_deflect = hub_r * 0.85f;
    const float deadzone    = 0.08f;
    const float dz_px       = max_deflect * deadzone;

    // --- X/Y Center Hub Joystick Interaction ---
    ImGui::SetCursorScreenPos(ImVec2(C_xy.x - hub_r, C_xy.y - hub_r));
    ImGui::InvisibleButton("##JogCenterXY", ImVec2(hub_r * 2.0f, hub_r * 2.0f));
    const bool xy_btn_hovered = ImGui::IsItemHovered();

    if (!step_jog_mode_) {
        if (ImGui::IsItemActive()) {
            js_xy_active_ = true;
            const ImVec2 mouse = ImGui::GetIO().MousePos;
            const float raw_dx = mouse.x - C_xy.x;
            const float raw_dy = mouse.y - C_xy.y;
            const float raw_dist = std::sqrt(raw_dx * raw_dx + raw_dy * raw_dy);

            if (raw_dist > max_deflect && raw_dist > 0.0001f) {
                const float scale = max_deflect / raw_dist;
                js_xy_drag_ = ImVec2(raw_dx * scale, raw_dy * scale);
            } else {
                js_xy_drag_ = ImVec2(raw_dx, raw_dy);
            }

            // Proportional speeds (-1.0 .. +1.0)
            const float norm_x = js_xy_drag_.x / max_deflect;
            const float norm_y = -js_xy_drag_.y / max_deflect; // Screen Y is inverted, so up is +Y
            const float max_spd = std::clamp(jog_speed_pct_ / 100.0f, 0.05f, 1.0f);

            // X axis proportional drive
            const float abs_x = std::abs(norm_x);
            if (abs_x > deadzone) {
                const float ratio_x = (abs_x - deadzone) / (1.0f - deadzone);
                const float speed_x = std::clamp(ratio_x * max_spd, 0.05f, 1.0f);
                controller.jogStartContinuous(core::Axis::X, norm_x > 0.0f, static_cast<double>(speed_x));
            } else {
                controller.jogStopAxis(core::Axis::X);
            }

            // Y axis proportional drive
            const float abs_y = std::abs(norm_y);
            if (abs_y > deadzone) {
                const float ratio_y = (abs_y - deadzone) / (1.0f - deadzone);
                const float speed_y = std::clamp(ratio_y * max_spd, 0.05f, 1.0f);
                controller.jogStartContinuous(core::Axis::Y, norm_y > 0.0f, static_cast<double>(speed_y));
            } else {
                controller.jogStopAxis(core::Axis::Y);
            }

            ImGui::SetTooltip("X: %+.0f%%  Y: %+.0f%%", norm_x * 100.0f, norm_y * 100.0f);
        }

        if (js_xy_active_ && (ImGui::IsItemDeactivated() || !ImGui::IsMouseDown(ImGuiMouseButton_Left))) {
            js_xy_active_ = false;
            js_xy_drag_   = ImVec2(0.0f, 0.0f);
            controller.jogStopAxis(core::Axis::X);
            controller.jogStopAxis(core::Axis::Y);
        }

        if (xy_btn_hovered && !js_xy_active_) {
            ImGui::SetTooltip("Analog Joystick: Drag X/Y (Deflection controls speed)");
        }
    } else {
        // Step increment mode: virtual joystick is inactive
        if (js_xy_active_) {
            js_xy_active_ = false;
            js_xy_drag_   = ImVec2(0.0f, 0.0f);
        }
        if (xy_btn_hovered) {
            ImGui::SetTooltip("Virtual Joystick (Continuous Jog Mode Only)");
        }
    }

    // --- Z and A Pill Hub Joystick Interaction (vertical drag) ---
    struct PillHub {
        const char* id;
        core::Axis  axis;
        char        name;
        ImVec2      center;
    };
    const PillHub pill_hubs[2] = {
        {"##JogCenterZ", core::Axis::Z, 'Z', C_z},
        {"##JogCenterA", core::Axis::A, 'A', C_a},
    };
    bool pill_hovered[2] = {false, false};
    for (int p = 0; p < pill_count; ++p) {
        const PillHub& hub = pill_hubs[p];
        ImGui::SetCursorScreenPos(ImVec2(hub.center.x - hub_r, hub.center.y - hub_r));
        ImGui::InvisibleButton(hub.id, ImVec2(hub_r * 2.0f, hub_r * 2.0f));
        pill_hovered[p] = ImGui::IsItemHovered();
        bool&  active = js_pill_active_[p];
        float& drag   = js_pill_drag_[p];

        if (!step_jog_mode_) {
            if (ImGui::IsItemActive()) {
                active = true;
                const float raw_dy = ImGui::GetIO().MousePos.y - hub.center.y;
                drag = std::clamp(raw_dy, -max_deflect, max_deflect);

                const float norm    = -drag / max_deflect; // Up is positive
                const float abs_n   = std::abs(norm);
                const float max_spd = std::clamp(jog_speed_pct_ / 100.0f, 0.05f, 1.0f);

                if (abs_n > deadzone) {
                    const float ratio = (abs_n - deadzone) / (1.0f - deadzone);
                    const float speed = std::clamp(ratio * max_spd, 0.05f, 1.0f);
                    controller.jogStartContinuous(hub.axis, norm > 0.0f, static_cast<double>(speed));
                } else {
                    controller.jogStopAxis(hub.axis);
                }

                ImGui::SetTooltip("%c: %+.0f%%", hub.name, norm * 100.0f);
            }

            if (active && (ImGui::IsItemDeactivated() || !ImGui::IsMouseDown(ImGuiMouseButton_Left))) {
                active = false;
                drag   = 0.0f;
                controller.jogStopAxis(hub.axis);
            }

            if (pill_hovered[p] && !active) {
                ImGui::SetTooltip("Analog Joystick: Drag %c (Deflection controls speed)", hub.name);
            }
        } else {
            if (active) {
                active = false;
                drag   = 0.0f;
            }
            if (pill_hovered[p]) {
                ImGui::SetTooltip("Virtual Joystick (Continuous Jog Mode Only)");
            }
        }
    }

    // Hit Testing & Embossed Molded Rubber Directional Arrows
    const float th = base_th * scale; // triangle height
    const float tb = base_tb * scale; // triangle base

    // A rotates in degrees: it takes the step value as shown (10 mm -> 10 deg, 0.1 in -> 0.1 deg)
    auto stepFor = [&](core::Axis axis) {
        const double step = static_cast<double>(step_size_mm_);
        return (axis == core::Axis::A && is_imperial) ? step / 25.4 : step;
    };

    auto doArrowButton = [&](const char* id,
                             ImVec2 btn_pos,
                             ImVec2 btn_size,
                             core::Axis axis,
                             bool pos_dir,
                             const char* tooltip,
                             int arrow_type, // 0=Up, 1=Down, 2=Left, 3=Right
                             ImVec2 arrow_center)
    {
        ImGui::SetCursorScreenPos(btn_pos);
        ImGui::InvisibleButton(id, btn_size);

        const size_t ax_idx = static_cast<size_t>(axis);
        bool mouse_active = ImGui::IsItemActive();
        bool hovered      = ImGui::IsItemHovered();
        bool kb_active    = (kb_axis_active_[ax_idx] == (pos_dir ? 1 : -1));

        bool js_active = false;
        if (js_xy_active_) {
            if (axis == core::Axis::X) {
                js_active = pos_dir ? (js_xy_drag_.x > dz_px) : (js_xy_drag_.x < -dz_px);
            } else if (axis == core::Axis::Y) {
                js_active = pos_dir ? (-js_xy_drag_.y > dz_px) : (-js_xy_drag_.y < -dz_px);
            }
        }
        const int pill = (axis == core::Axis::Z) ? 0 : (axis == core::Axis::A ? 1 : -1);
        if (pill >= 0 && js_pill_active_[pill]) {
            const float d = -js_pill_drag_[pill];
            js_active = pos_dir ? (d > dz_px) : (d < -dz_px);
        }

        bool active = mouse_active || kb_active || js_active;

        if (step_jog_mode_) {
            if (ImGui::IsItemClicked()) {
                const double step = stepFor(axis);
                double delta = pos_dir ? step : -step;
                double feed  = 3000.0 * (jog_speed_pct_ / 100.0f);
                controller.jogStep(axis, delta, feed);
            }
        } else {
            if (ImGui::IsItemActive()) {
                // Re-sent every frame while held (controller-side jog dead-man)
                controller.jogStartContinuous(axis, pos_dir, static_cast<double>(jog_speed_pct_ / 100.0f));
            }
            if (ImGui::IsItemDeactivated()) {
                controller.jogStopAxis(axis);
            }
        }

        if (hovered && !js_xy_active_ && !js_pill_active_[0] && !js_pill_active_[1]) {
            ImGui::SetTooltip("%s", tooltip);
        }

        // --- Translucent Rubber Membrane Glow / Backlighting on Press / Hover ---
        if (active) {
            dl->AddCircleFilled(arrow_center, th * 1.15f, theme::withAlpha(theme::pal().jog_accent, 50), 24);
            dl->AddCircleFilled(arrow_center, th * 0.75f, theme::withAlpha(theme::pal().jog_accent, 90), 24);
            arrow_center.y += 1.0f;
        } else if (hovered) {
            dl->AddCircleFilled(arrow_center, th * 0.95f, theme::pal().jog_hover_glow, 24);
        }

        // --- Embossed Rubber Directional Arrow ---
        ImVec2 v1, v2, v3;
        if (arrow_type == 0) { // UP
            v1 = ImVec2(arrow_center.x, arrow_center.y - th * 0.55f);
            v2 = ImVec2(arrow_center.x - tb * 0.5f, arrow_center.y + th * 0.45f);
            v3 = ImVec2(arrow_center.x + tb * 0.5f, arrow_center.y + th * 0.45f);
        } else if (arrow_type == 1) { // DOWN
            v1 = ImVec2(arrow_center.x, arrow_center.y + th * 0.55f);
            v2 = ImVec2(arrow_center.x - tb * 0.5f, arrow_center.y - th * 0.45f);
            v3 = ImVec2(arrow_center.x + tb * 0.5f, arrow_center.y - th * 0.45f);
        } else if (arrow_type == 2) { // LEFT
            v1 = ImVec2(arrow_center.x - th * 0.55f, arrow_center.y);
            v2 = ImVec2(arrow_center.x + th * 0.45f, arrow_center.y - tb * 0.5f);
            v3 = ImVec2(arrow_center.x + th * 0.45f, arrow_center.y + tb * 0.5f);
        } else { // RIGHT
            v1 = ImVec2(arrow_center.x + th * 0.55f, arrow_center.y);
            v2 = ImVec2(arrow_center.x - th * 0.45f, arrow_center.y - tb * 0.5f);
            v3 = ImVec2(arrow_center.x - th * 0.45f, arrow_center.y + tb * 0.5f);
        }

        // Contact shadow beneath the triangle
        const float sh_tri_y = (arrow_type == 0) ? 2.5f : ((arrow_type == 1) ? 2.0f : 2.2f);
        dl->AddTriangleFilled(ImVec2(v1.x, v1.y + sh_tri_y),
                              ImVec2(v2.x, v2.y + sh_tri_y),
                              ImVec2(v3.x, v3.y + sh_tri_y),
                              theme::pal().jog_arrow_shadow);

        // Molded Silvery Platinum Arrow Face
        const ImU32 tri_col = active ? theme::pal().jog_accent
                                     : (hovered ? theme::pal().jog_arrow_hover : theme::pal().jog_arrow);
        dl->AddTriangleFilled(v1, v2, v3, tri_col);

        if (active) {
            dl->AddTriangle(v1, v2, v3, IM_COL32(255, 255, 255, 240), 1.8f);
        }
    };

    const float arm_reach = (arm_w * 0.5f + arm_inset * 0.5f);

    // Y+ (Up)
    doArrowButton("##JogYPlus",
                  ImVec2(C_xy.x - arm_w * 0.5f, P_xy.y),
                  ImVec2(arm_w, arm_inset),
                  core::Axis::Y, true,
                  "Jog Y+ (Up Arrow)",
                  0,
                  ImVec2(C_xy.x, C_xy.y - arm_reach));

    // Y- (Down)
    doArrowButton("##JogYMinus",
                  ImVec2(C_xy.x - arm_w * 0.5f, C_xy.y + arm_w * 0.5f),
                  ImVec2(arm_w, arm_inset),
                  core::Axis::Y, false,
                  "Jog Y- (Down Arrow)",
                  1,
                  ImVec2(C_xy.x, C_xy.y + arm_reach));

    // X- (Left)
    doArrowButton("##JogXMinus",
                  ImVec2(P_xy.x, C_xy.y - arm_w * 0.5f),
                  ImVec2(arm_inset, arm_w),
                  core::Axis::X, false,
                  "Jog X- (Left Arrow)",
                  2,
                  ImVec2(C_xy.x - arm_reach, C_xy.y));

    // X+ (Right)
    doArrowButton("##JogXPlus",
                  ImVec2(C_xy.x + arm_w * 0.5f, C_xy.y - arm_w * 0.5f),
                  ImVec2(arm_inset, arm_w),
                  core::Axis::X, true,
                  "Jog X+ (Right Arrow)",
                  3,
                  ImVec2(C_xy.x + arm_reach, C_xy.y));

    // Z+ (Up)
    const float z_reach = (z_h * 0.5f - arm_inset * 0.5f);
    doArrowButton("##JogZPlus",
                  P_z,
                  ImVec2(z_w, arm_inset),
                  core::Axis::Z, true,
                  "Jog Z+ (Page Up)",
                  0,
                  ImVec2(C_z.x, C_z.y - z_reach));

    // Z- (Down)
    doArrowButton("##JogZMinus",
                  ImVec2(P_z.x, P_z.y + z_h - arm_inset),
                  ImVec2(z_w, arm_inset),
                  core::Axis::Z, false,
                  "Jog Z- (Page Down)",
                  1,
                  ImVec2(C_z.x, C_z.y + z_reach));

    if (a_jog_enabled_) {
        doArrowButton("##JogAPlus",
                      P_a,
                      ImVec2(z_w, arm_inset),
                      core::Axis::A, true,
                      "Jog A+ (Home)",
                      0,
                      ImVec2(C_a.x, C_a.y - z_reach));
        doArrowButton("##JogAMinus",
                      ImVec2(P_a.x, P_a.y + z_h - arm_inset),
                      ImVec2(z_w, arm_inset),
                      core::Axis::A, false,
                      "Jog A- (End)",
                      1,
                      ImVec2(C_a.x, C_a.y + z_reach));
    }

    // Draw Hub Sockets & Analog Thumbstick Knobs
    const float font_sz = std::max(11.0f, base_font_sz * scale);
    const float knob_r  = hub_r * 0.78f;

    // Stem line connecting base center to displaced thumbstick knob
    if (js_xy_active_ && (js_xy_drag_.x * js_xy_drag_.x + js_xy_drag_.y * js_xy_drag_.y > 4.0f)) {
        const ImVec2 knob_xy = ImVec2(C_xy.x + js_xy_drag_.x, C_xy.y + js_xy_drag_.y);
        dl->AddLine(C_xy, knob_xy, theme::pal().stick_shadow, 6.0f);
        dl->AddLine(C_xy, knob_xy, theme::withAlpha(theme::pal().jog_accent, 220), 3.5f);
    }

    renderCenterKnob(dl, C_xy, knob_r, "X/Y", dro_font, font_sz, js_xy_active_, xy_btn_hovered && !step_jog_mode_, js_xy_drag_);

    for (int p = 0; p < pill_count; ++p) {
        const ImVec2 c = pill_hubs[p].center;
        const float  d = js_pill_drag_[p];
        if (js_pill_active_[p] && std::abs(d) > 2.0f) {
            const ImVec2 knob = ImVec2(c.x, c.y + d);
            dl->AddLine(c, knob, theme::pal().stick_shadow, 6.0f);
            dl->AddLine(c, knob, theme::withAlpha(theme::pal().jog_accent, 220), 3.5f);
        }
        const char label[2] = {pill_hubs[p].name, '\0'};
        renderCenterKnob(dl, c, knob_r, label, dro_font, font_sz, js_pill_active_[p],
                         pill_hovered[p] && !step_jog_mode_, ImVec2(0.0f, d));
    }

    // Advance cursor cleanly below the jog pad
    ImGui::SetCursorScreenPos(ImVec2(screen_pos.x, screen_pos.y + cross_sz + 14.0f));

    // Centered STOP JOG Button (placed directly below the jog pads, spanning target_w)
    ImGui::SetCursorPosX(pad_offset_x);
    ImGui::PushStyleColor(ImGuiCol_Button, theme::pal().btn_danger);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_danger_hov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, theme::pal().btn_danger_act);
    ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_danger));
    if (ImGui::Button("STOP JOG", ImVec2(target_w, 38.0f))) {
        controller.jogStop();
    }
    ImGui::PopStyleColor(4);

    ImGui::Spacing();
#ifdef __APPLE__
    const char* one_step = ", Cmd: one step";
#else
    const char* one_step = ", Ctrl: one step";
#endif
    char hint_text[128];
    std::snprintf(hint_text, sizeof(hint_text), "Keys: arrows X / Y, Page Up / Down Z%s%s",
                  a_jog_enabled_ ? ", Home / End A" : "", step_jog_mode_ ? "" : one_step);
    const ImVec2 hint_sz  = ImGui::CalcTextSize(hint_text);
    if (hint_sz.x <= pad_avail_w) {
        ImGui::SetCursorPosX((pad_avail_w - hint_sz.x) * 0.5f);
        ImGui::TextDisabled("%s", hint_text);
    } else {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%s", hint_text);
        ImGui::PopTextWrapPos();
    }

    // Macro Buttons Pinned to Control Panel
    core::MachineConfig cfg = controller.getConfig();
    auto& sc = cfg.scripting;
    std::vector<size_t> cp_indices;
    for (size_t i = 0; i < sc.macro_buttons.size(); ++i) {
        if (sc.macro_buttons[i].show_on_control_panel) {
            cp_indices.push_back(i);
        }
    }

    if (!cp_indices.empty()) {
        ImGui::Spacing();
        ImGui::Dummy(ImVec2(0.0f, 2.0f));

        const float cp_avail_w = ImGui::GetContentRegionAvail().x;
        const float cp_spacing_x = ImGui::GetStyle().ItemSpacing.x;
        const size_t cp_count = cp_indices.size();
        float cp_btn_w = 110.0f;
        if (cp_count <= 4 && cp_count > 0) {
            float fit = (cp_avail_w - static_cast<float>(cp_count - 1) * cp_spacing_x) / static_cast<float>(cp_count);
            if (fit >= 75.0f && fit <= 160.0f) {
                cp_btn_w = fit;
            }
        }
        const float cp_btn_h = 32.0f;

        int button_to_unpin = -1;
        int button_to_remove = -1;
        bool color_changed = false;

        cp_color_menu_.beginFrame();
        for (size_t k = 0; k < cp_count; ++k) {
            size_t idx = cp_indices[k];
            const auto& btn_cfg = sc.macro_buttons[idx];

            if (k > 0) {
                float next_x = ImGui::GetItemRectMax().x + cp_spacing_x + cp_btn_w;
                if (next_x < ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x) {
                    ImGui::SameLine();
                }
            }

            ImGui::PushID(static_cast<int>(1000 + idx));
            std::string resolved = scripting::resolveScriptPath(btn_cfg.path);
            bool has_target = !resolved.empty();

            if (!has_target) ImGui::BeginDisabled();
            std::string btn_text = btn_cfg.label.empty() ? ("Macro " + std::to_string(idx + 1)) : btn_cfg.label;
            if (macroButton(btn_text.c_str(), btn_cfg, ImVec2(cp_btn_w, cp_btn_h), &theme::pal().macro_btn,
                            cp_color_menu_.previewFor(idx))) {
                controller.runScriptFile(resolved, true);
            }
            if (!has_target) ImGui::EndDisabled();

            if (ImGui::IsItemHovered()) {
                if (has_target) {
                    ImGui::SetTooltip("Macro: %s\n(Right-click to remove from control panel)", resolved.c_str());
                } else {
                    ImGui::SetTooltip("File not found: %s\n(Right-click to remove from control panel)", btn_cfg.path.c_str());
                }
            }

            // Right-click context popup to remove from control panel
            if (ImGui::BeginPopupContextItem("##CpMacroBtnCtx")) {
                if (ImGui::MenuItem("Remove from control panel")) {
                    button_to_unpin = static_cast<int>(idx);
                }
                if (ImGui::MenuItem("Delete Macro Button")) {
                    button_to_remove = static_cast<int>(idx);
                }
                color_changed |= cp_color_menu_.menu(idx, sc.macro_buttons[idx]);
                ImGui::EndPopup();
            }

            ImGui::PopID();
        }
        cp_color_menu_.endFrame();

        if (color_changed) {
            controller.applyConfig(cfg);
            if (on_config_changed_) on_config_changed_();
        }

        if (button_to_unpin >= 0 && button_to_unpin < static_cast<int>(sc.macro_buttons.size())) {
            sc.macro_buttons[button_to_unpin].show_on_control_panel = false;
            controller.applyConfig(cfg);
            if (on_config_changed_) on_config_changed_();
        }
        if (button_to_remove >= 0 && button_to_remove < static_cast<int>(sc.macro_buttons.size())) {
            sc.macro_buttons.erase(sc.macro_buttons.begin() + button_to_remove);
            controller.applyConfig(cfg);
            if (on_config_changed_) on_config_changed_();
        }
    }

    // Limit Triggered / Override Alarm Banner at the bottom of the Manual Jog Control section
    if (telemetry.limit_switch_triggered ||
        telemetry.soft_limit_triggered ||
        telemetry.limit_override_active ||
        telemetry.state == core::ControllerState::EStop ||
        !telemetry.active_fault_message.empty())
    {
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_ChildBg, theme::pal().surface_danger);
        ImGui::PushStyleColor(ImGuiCol_Border,  theme::pal().border_danger);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 6.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 3.0f));

        const bool is_alarm_estop = (telemetry.state == core::ControllerState::EStop ||
                                     telemetry.limit_switch_triggered ||
                                     telemetry.soft_limit_triggered ||
                                     telemetry.limit_override_active);

        ImGui::BeginChild("##LimitAlertBanner", ImVec2(0.0f, 0.0f),
                          ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY,
                          ImGuiWindowFlags_NoScrollbar);

        ImGui::PushTextWrapPos(0.0f);
        if (is_alarm_estop) {
            ImGui::TextColored(theme::pal().text_danger,
                               "ALARM / E-STOP: %s",
                               telemetry.active_fault_message.empty()
                                   ? "Emergency Stop / Hardware Limit Active!"
                                   : telemetry.active_fault_message.c_str());
        } else if (!telemetry.active_fault_message.empty()) {
            ImGui::TextColored(theme::pal().text_warn,
                               "STATUS: %s", telemetry.active_fault_message.c_str());
        } else {
            ImGui::TextColored(theme::pal().text_warn,
                               "Limit Override Active - Jog carefully off switch");
        }
        ImGui::PopTextWrapPos();

        // The Limit Override checkbox should ONLY appear if this is an ALARM / E-STOP condition
        if (is_alarm_estop) {
            bool ov = telemetry.limit_override_active;
            const float banner_w = ImGui::GetWindowWidth();
            const char* cb_label = (banner_w < 430.0f) ? "Limit Override" : "Limit Override (Allow Jog Off Switch)";

            // Compact, proportional checkbox styling so it fits neatly without scrolling
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2.0f, 1.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f);
            if (ImGui::Checkbox(cb_label, &ov)) {
                controller.setLimitOverride(ov);
            }
            ImGui::PopStyleVar(2);
        }

        ImGui::EndChild();
        ImGui::PopStyleVar(4);
        ImGui::PopStyleColor(2);
    }

    rendered_height_ = ImGui::GetCursorPosY() - start_cursor_y + ImGui::GetStyle().WindowPadding.y * 2.0f;
    max_height_      = rendered_height_ + (base_cross_sz - cross_sz);
    min_height_      = max_height_ - (base_cross_sz - 70.0f);

    // -------------------------------------------------------------------------
    // Interactive Probe Z Modal Dialog
    // -------------------------------------------------------------------------
    if (show_probe_z_modal_) {
        ImGui::OpenPopup("Probe Z - Tool Touch-Off##ProbeZModal");
        show_probe_z_modal_ = false;
    }

    ImVec2 modal_center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(modal_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(540.0f, 0.0f));
    if (ImGui::BeginPopupModal("Probe Z - Tool Touch-Off##ProbeZModal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, theme::pal().text_info);
        ImGui::TextUnformatted("Z-AXIS PROBE & TOOL TOUCH-OFF");
        ImGui::PopStyleColor();
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        core::MachineConfig cfg = controller.getConfig();
        float plate_t = static_cast<float>(cfg.probe.plate_thickness_mm);

        ImGui::AlignTextToFramePadding();
        ImGui::Text("Configured Z Plate Thickness:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(270.0f);
        if (ImGui::InputFloat("##PlateThicknessModal", &plate_t, 0.1f, 1.0f, "%.3f mm")) {
            cfg.probe.plate_thickness_mm = std::max(0.0f, plate_t);
            controller.applyConfig(cfg);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Thickness of your conductive touch plate / tool setter block.");
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Select probing touch-off method:");
        ImGui::Spacing();

        // Probe with Z Plate
        ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_primary);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_primary_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_primary_act);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_primary));
        if (ImGui::Button("PROBE WITH Z PLATE", ImVec2(-1.0f, 38.0f))) {
            controller.runScriptString("local p = dofile(cnc.resolve_path('scripts/mill/probing/probe_lib.lua')); p.probe_z(false)", true);
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(4);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Probes down to touch plate. Upon contact, zeroes stock surface by setting Work Z to +%.3f mm (Plate Thickness).", cfg.probe.plate_thickness_mm);
        }

        ImGui::Spacing();

        // Probe on top of workpiece (if metal)
        ImGui::PushStyleColor(ImGuiCol_Button,        theme::pal().btn_ok);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::pal().btn_ok_hov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  theme::pal().btn_ok_act);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::contrastText(theme::pal().btn_ok));
        if (ImGui::Button("PROBE ON TOP OF WORKPIECE (IF METAL)", ImVec2(-1.0f, 38.0f))) {
            controller.runScriptString("local p = dofile(cnc.resolve_path('scripts/mill/probing/probe_lib.lua')); p.probe_z(true)", true);
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(4);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Probes down directly onto a conductive metal workpiece. Upon contact, sets Work Z to 0.000 mm.");
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Cancel
        if (ImGui::Button("CANCEL", ImVec2(-1.0f, 32.0f))) {
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

void DroAndJogPanel::handleKeyboardJog(
    core::IMotionController& controller,
    const core::TelemetrySnapshot& telemetry)
{
    ImGuiIO& io = ImGui::GetIO();

    // Never steal keys from text fields (MDI input, numeric inputs) or while E-Stop is active
    if (io.WantTextInput || telemetry.state == core::ControllerState::EStop) {
        if (kb_axis_active_ != std::array<int, 4>{0, 0, 0, 0}) {
            controller.jogStop();
            kb_axis_active_ = {0, 0, 0, 0};
        }
        return;
    }

    // Step mode, or Ctrl (Cmd on macOS) held in continuous mode: each key press moves one step increment
    if (step_jog_mode_ || io.KeyCtrl) {
        // Stop any residual continuous jogging if user switched to step mode while keys were held
        if (kb_axis_active_ != std::array<int, 4>{0, 0, 0, 0}) {
            controller.jogStop();
            kb_axis_active_ = {0, 0, 0, 0};
        }

        const double step = static_cast<double>(step_size_mm_);
        const double feed = 3000.0 * (jog_speed_pct_ / 100.0f);

        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, false)) controller.jogStep(core::Axis::X,  step, feed);
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow,  false)) controller.jogStep(core::Axis::X, -step, feed);
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow,    false)) controller.jogStep(core::Axis::Y,  step, feed);
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow,  false)) controller.jogStep(core::Axis::Y, -step, feed);
        if (ImGui::IsKeyPressed(ImGuiKey_PageUp,     false)) controller.jogStep(core::Axis::Z,  step, feed);
        if (ImGui::IsKeyPressed(ImGuiKey_PageDown,   false)) controller.jogStep(core::Axis::Z, -step, feed);
        if (a_jog_enabled_) {
            // A takes the step value as shown, in degrees
            const bool imperial = controller.getConfig().linear_units == core::LinearUnits::Imperial;
            const double a_step = imperial ? step / 25.4 : step;
            if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) controller.jogStep(core::Axis::A,  a_step, feed);
            if (ImGui::IsKeyPressed(ImGuiKey_End,  false)) controller.jogStep(core::Axis::A, -a_step, feed);
        }
        return;
    }

    // Continuous Mode: Support simultaneous multi-axis motion!
    // When X, Y, and Z arrow/page keys are pressed together, all 3 axes move concurrently.
    const double speed_ratio = static_cast<double>(jog_speed_pct_ / 100.0f);

    struct AxisKeyBinding {
        core::Axis axis;
        size_t         index;
        ImGuiKey       pos_key;
        ImGuiKey       neg_key;
    };

    static const AxisKeyBinding kBindings[4] = {
        {core::Axis::X, 0, ImGuiKey_RightArrow, ImGuiKey_LeftArrow},
        {core::Axis::Y, 1, ImGuiKey_UpArrow,    ImGuiKey_DownArrow},
        {core::Axis::Z, 2, ImGuiKey_PageUp,     ImGuiKey_PageDown},
        {core::Axis::A, 3, ImGuiKey_Home,       ImGuiKey_End},
    };

    for (const auto& b : kBindings) {
        if (b.axis == core::Axis::A && !a_jog_enabled_) {
            if (kb_axis_active_[3] != 0) controller.jogStopAxis(core::Axis::A);
            kb_axis_active_[3] = 0;
            continue;
        }
        bool pos_down = ImGui::IsKeyDown(b.pos_key);
        bool neg_down = ImGui::IsKeyDown(b.neg_key);

        int desired = 0;
        if (pos_down && !neg_down) desired = 1;
        else if (neg_down && !pos_down) desired = -1;

        if (desired != 0) {
            // Re-sent every frame while held: the controller stops the axis if the refresh stops (dead-man)
            controller.jogStartContinuous(b.axis, desired > 0, speed_ratio);
        } else if (kb_axis_active_[b.index] != 0) {
            controller.jogStopAxis(b.axis);
        }
        kb_axis_active_[b.index] = desired;
    }
}

} // namespace rnr::ui
