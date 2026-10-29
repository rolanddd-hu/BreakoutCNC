#include "ui_common/probe_panel.h"
#include "ui_common/toolpath_viewport.h"
#include "core/controller/motion_controller.h"
#include "core/scripting/script_path_helper.h"
#include "imgui.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace rnr::ui {

namespace {

// Palette matching DroAndJogPanel industrial vector theme & reference screenshot
constexpr ImU32 col_card_bg       = IM_COL32(18, 20, 25, 255);
constexpr ImU32 col_card_border   = IM_COL32(38, 44, 54, 255);

// Realistic metallic aluminum tiles with sheen
constexpr ImU32 col_metal_top     = IM_COL32(146, 154, 166, 255);
constexpr ImU32 col_metal_bot     = IM_COL32(96, 104, 116, 255);
constexpr ImU32 col_metal_bevel   = IM_COL32(180, 190, 205, 255);
constexpr ImU32 col_metal_border  = IM_COL32(40, 44, 52, 255);

// Dark graphite pocket / center tile
constexpr ImU32 col_center_tile   = IM_COL32(32, 35, 42, 255);
constexpr ImU32 col_pocket_bg     = IM_COL32(22, 25, 31, 255);
constexpr ImU32 col_pocket_shadow = IM_COL32(12, 14, 18, 240);

// Crisp chunky solid white arrows with drop shadow
constexpr ImU32 col_arrow         = IM_COL32(245, 248, 255, 255);
constexpr ImU32 col_arrow_shadow  = IM_COL32(8, 10, 14, 220);

// Safety Orange Hotspots (matching user request: dot size stays the same)
constexpr ImU32 col_dot_idle      = IM_COL32(255, 140, 18, 255);
constexpr ImU32 col_dot_border    = IM_COL32(255, 190, 60, 230);
constexpr ImU32 col_dot_hover     = IM_COL32(255, 205, 65, 255);
constexpr ImU32 col_dot_active    = IM_COL32(255, 245, 210, 255);
constexpr ImU32 col_dot_glow      = IM_COL32(255, 140, 18, 95);

// Purple/Blue guideline colors matching reference screenshot
constexpr ImU32 col_dash_line     = IM_COL32(150, 95, 215, 220);
constexpr ImU32 col_blue_border   = IM_COL32(42, 102, 162, 210);

// Draws a bold, chunky, solid arrow with drop shadow
void drawChunkyArrow(ImDrawList* dl, ImVec2 from, ImVec2 to, float stem_width, float head_width, float head_length, ImU32 color = col_arrow) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float len = std::hypot(dx, dy);
    if (len < 1e-4f) return;
    float ux = dx / len;
    float uy = dy / len;
    float px = -uy;
    float py = ux;

    float hl = std::min(head_length, len * 0.75f);
    ImVec2 base(to.x - ux * hl, to.y - uy * hl);

    float half_head = head_width * 0.5f;
    float half_stem = stem_width * 0.5f;

    ImVec2 p_tip(to.x, to.y);
    ImVec2 p_l(base.x + px * half_head, base.y + py * half_head);
    ImVec2 p_r(base.x - px * half_head, base.y - py * half_head);

    ImVec2 s_tl(from.x + px * half_stem, from.y + py * half_stem);
    ImVec2 s_tr(from.x - px * half_stem, from.y - py * half_stem);
    ImVec2 s_bl(base.x + px * half_stem, base.y + py * half_stem);
    ImVec2 s_br(base.x - px * half_stem, base.y - py * half_stem);

    // Drop shadow
    constexpr float so = 1.5f;
    dl->AddQuadFilled(ImVec2(s_tl.x + so, s_tl.y + so), ImVec2(s_bl.x + so, s_bl.y + so),
                      ImVec2(s_br.x + so, s_br.y + so), ImVec2(s_tr.x + so, s_tr.y + so), col_arrow_shadow);
    dl->AddTriangleFilled(ImVec2(p_tip.x + so, p_tip.y + so), ImVec2(p_l.x + so, p_l.y + so),
                          ImVec2(p_r.x + so, p_r.y + so), col_arrow_shadow);

    // Solid arrow body
    dl->AddQuadFilled(s_tl, s_bl, s_br, s_tr, color);
    dl->AddTriangleFilled(p_tip, p_l, p_r, color);
}

// Draws a realistic metallic beveled square
void drawMetalTile(ImDrawList* dl, ImVec2 p0, ImVec2 p1, float rounding = 4.0f) {
    dl->AddRectFilledMultiColor(p0, p1, col_metal_top, col_metal_top, col_metal_bot, col_metal_bot);
    dl->AddRect(p0, p1, col_metal_border, rounding, 0, 1.2f);
    // Subtle top-left highlight bevel
    dl->AddLine(ImVec2(p0.x + 1.0f, p0.y + 1.0f), ImVec2(p1.x - 2.0f, p0.y + 1.0f), col_metal_bevel, 1.0f);
    dl->AddLine(ImVec2(p0.x + 1.0f, p0.y + 1.0f), ImVec2(p0.x + 1.0f, p1.y - 2.0f), col_metal_bevel, 1.0f);
}

// Draws a dashed line segment between two points
void drawDashedLine(ImDrawList* dl, ImVec2 p1, ImVec2 p2, ImU32 col, float thickness = 1.2f, float dash_len = 4.0f, float gap_len = 3.0f) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float len = std::hypot(dx, dy);
    if (len < 1e-4f) return;
    float ux = dx / len;
    float uy = dy / len;

    float cur = 0.0f;
    while (cur < len) {
        float next_dash = std::min(cur + dash_len, len);
        dl->AddLine(ImVec2(p1.x + ux * cur, p1.y + uy * cur),
                    ImVec2(p1.x + ux * next_dash, p1.y + uy * next_dash), col, thickness);
        cur += dash_len + gap_len;
    }
}

} // namespace

bool ProbePanel::renderHotspot(const char* id,
                               const ImVec2& center,
                               float radius,
                               const char* /*label*/,
                               const char* tooltip,
                               const std::string& script_rel_path,
                               core::IMotionController& controller)
{
    ImDrawList* dl = ImGui::GetWindowDrawList();

    ImGui::SetCursorScreenPos(ImVec2(center.x - radius * 1.3f, center.y - radius * 1.3f));
    bool clicked = ImGui::InvisibleButton(id, ImVec2(radius * 2.6f, radius * 2.6f));
    bool hovered = ImGui::IsItemHovered();
    bool active  = ImGui::IsItemActive();

    // Outer shadow
    dl->AddCircleFilled(ImVec2(center.x + 1.0f, center.y + 1.5f), radius + 1.0f, IM_COL32(10, 12, 16, 170), 24);

    // Glow ring when hovered or active
    if (hovered || active) {
        dl->AddCircleFilled(center, radius + 4.5f, col_dot_glow, 24);
    }

    ImU32 fill_col   = active ? col_dot_active : (hovered ? col_dot_hover : col_dot_idle);
    ImU32 border_col = hovered ? IM_COL32(255, 255, 255, 240) : col_dot_border;
    float cur_r      = hovered ? (radius + 1.5f) : radius;

    dl->AddCircleFilled(center, cur_r, fill_col, 24);
    dl->AddCircle(center, cur_r, border_col, 24, hovered ? 2.0f : 1.5f);

    // Subtle highlight pip
    dl->AddCircleFilled(ImVec2(center.x - cur_r * 0.35f, center.y - cur_r * 0.35f), cur_r * 0.28f, IM_COL32(255, 255, 255, 200), 12);

    if (hovered && tooltip) {
        ImGui::BeginTooltip();
        ImGui::TextColored(ImVec4(1.0f, 0.70f, 0.15f, 1.0f), "[PROBE MACRO]");
        ImGui::Separator();
        ImGui::TextUnformatted(tooltip);
        ImGui::Spacing();
        ImGui::TextDisabled("Script: %s", script_rel_path.c_str());
        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "Click to execute probe routine.");
        ImGui::EndTooltip();
    }

    if (clicked) {
        std::string full_path = scripting::resolveScriptPath(script_rel_path);
        if (!full_path.empty()) {
            controller.runScriptFile(full_path, true);
        } else {
            controller.runScriptFile(script_rel_path, true);
        }
    }

    return clicked;
}

void ProbePanel::render(core::IMotionController& controller,
                        const core::TelemetrySnapshot& telemetry,
                        core::MachineConfig& cfg,
                        ToolpathViewport* viewport)
{
    if (viewport) {
        viewport_ = viewport;
    }
    ToolpathViewport* eff_viewport = viewport ? viewport : viewport_;
    const float avail_w = ImGui::GetContentRegionAvail().x;

    // 1. Live Touch Probe Contact Status Header (Auto-resized with matching top/bottom padding, zero vertical scrolling)
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 6.0f));
    ImGui::BeginChild("##ProbeContactStatusCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    {
        bool hit = telemetry.probe_contact_active;
        ImVec4 status_col = hit ? ImVec4(0.15f, 0.95f, 0.45f, 1.0f) : ImVec4(0.55f, 0.65f, 0.75f, 1.0f);
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(status_col, hit ? "[PROBE: CONTACT ACTIVE (G31 HIT)]" : "[PROBE: READY (OPEN CIRCUIT)]");

        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("| Pin: IN%d (%s)",
                            cfg.probe.probe_pin,
                            cfg.probe.probe_invert ? "Active Low" : "Active High");

        if (telemetry.simulator_mode && cfg.probe.probe_pin >= 1) {
            ImGui::SameLine(std::max(ImGui::GetCursorPosX() + 10.0f, ImGui::GetWindowWidth() - 130.0f));
            bool sim_p = ((telemetry.simulated_inputs_bitmask >> (cfg.probe.probe_pin - 1)) & 1U) != 0;
            if (ImGui::Button(sim_p ? "Release Probe" : "Simulate Touch", ImVec2(120.0f, 22.0f))) {
                controller.setSimulatedInputPin(cfg.probe.probe_pin, !sim_p);
            }
        }
    }
    ImGui::EndChild();
    ImGui::PopStyleVar();

    ImGui::Spacing();

    // 2. Segmented Mode Toggle Bar (SURFACE | OUTSIDE | INSIDE | CENTER | PROBE Z)
    {
        const char* deck_labels[5] = {"SURFACE", "OUTSIDE", "INSIDE", "CENTER", "PROBE Z"};
        const float tab_h = 32.0f;

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f);

        auto drawTabBtn = [&](int idx, float w) {
            bool is_selected = (selected_deck_ == idx);
            if (is_selected) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.52f, 0.72f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.00f, 0.60f, 0.82f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.13f, 0.16f, 0.21f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.26f, 0.34f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.62f, 0.68f, 0.76f, 1.0f));
            }
            if (ImGui::Button(deck_labels[idx], ImVec2(w, tab_h))) {
                selected_deck_ = idx;
            }
            ImGui::PopStyleColor(3);
        };

        if (avail_w >= 320.0f) {
            // 1 Row of 5 tabs: SURFACE | OUTSIDE | INSIDE | CENTER | PROBE Z
            const float tab_w = std::floor(avail_w / 5.0f);
            for (int i = 0; i < 5; ++i) {
                if (i > 0) ImGui::SameLine();
                float cur_w = (i == 4) ? (avail_w - tab_w * 4.0f) : tab_w;
                drawTabBtn(i, cur_w);
            }
        } else {
            // 2 Rows if narrow:
            // Row 1: SURFACE (50%) | OUTSIDE (50%)
            float r1_w = std::floor(avail_w / 2.0f);
            drawTabBtn(0, r1_w);
            ImGui::SameLine();
            drawTabBtn(1, avail_w - r1_w);

            // Row 2: INSIDE (33.3%) | CENTER (33.3%) | PROBE Z (rem)
            float r2_w = std::floor(avail_w / 3.0f);
            drawTabBtn(2, r2_w);
            ImGui::SameLine();
            drawTabBtn(3, r2_w);
            ImGui::SameLine();
            drawTabBtn(4, avail_w - r2_w * 2.0f);
        }
        ImGui::PopStyleVar(2);
    }

    ImGui::Spacing();

    // 3. Render ONLY the selected deck
    if (selected_deck_ == 0) {
        renderSurfaceDeck(controller, telemetry, cfg, avail_w, eff_viewport);
    } else if (selected_deck_ == 1) {
        renderOutsideDeck(controller, avail_w);
    } else if (selected_deck_ == 2) {
        renderInsideDeck(controller, avail_w);
    } else if (selected_deck_ == 3) {
        renderCenterDeck(controller, avail_w);
    } else {
        renderProbeZDeck(controller, avail_w);
    }
}


void ProbePanel::renderSurfaceDeck(core::IMotionController& controller,
                                   const core::TelemetrySnapshot& telemetry,
                                   core::MachineConfig& cfg,
                                   float avail_w,
                                   ToolpathViewport* viewport)
{
    if (!smoothness_initialized_) {
        float init_s = static_cast<float>(cfg.probe.surface_smoothness);
        if (init_s <= 0.001f && (!telemetry.surface_height_map->is_valid || telemetry.surface_height_map->smoothness <= 0.001)) {
            init_s = 0.5f;
            cfg.probe.surface_smoothness = 0.5;
        } else if (telemetry.surface_height_map->is_valid && telemetry.surface_height_map->smoothness > 0.001) {
            init_s = static_cast<float>(telemetry.surface_height_map->smoothness);
        }
        init_s = std::clamp(init_s, 0.0f, 1.0f);
        surface_smoothness_ = init_s;
        applied_smoothness_ = telemetry.surface_height_map->is_valid ? static_cast<float>(telemetry.surface_height_map->smoothness) : init_s;
        surface_max_points_ = std::max(5, cfg.probe.surface_max_points);
        smoothness_initialized_ = true;
    } else if (telemetry.surface_height_map->is_valid && !telemetry.surface_probing_active) {
        float map_s = static_cast<float>(telemetry.surface_height_map->smoothness);
        if (std::abs(map_s - applied_smoothness_) > 0.005f &&
            std::abs(surface_smoothness_ - applied_smoothness_) < 0.005f)
        {
            applied_smoothness_ = map_s;
            surface_smoothness_ = map_s;
        }
    }

    if (viewport) {
        const bool has_pending_change = telemetry.surface_height_map->is_valid &&
                                       (std::abs(surface_smoothness_ - applied_smoothness_) > 0.005f);
        viewport->setProposedSurfaceSmoothness(surface_smoothness_, has_pending_change);
    }

    if (!surface_bounds_initialized_) {
        std::array<double, 3> bmin{}, bmax{};
        if (controller.getProgramBounds(bmin, bmax)) {
            surface_min_x_ = bmin[0];
            surface_max_x_ = bmax[0];
            surface_min_y_ = bmin[1];
            surface_max_y_ = bmax[1];
        } else {
            surface_min_x_ = 0.0;
            surface_max_x_ = 100.0;
            surface_min_y_ = 0.0;
            surface_max_y_ = 80.0;
        }
        surface_bounds_initialized_ = true;
    }

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.10f, 0.13f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.22f, 0.28f, 0.36f, 0.85f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 8.0f));

    // 1. Status & Active Probing Progress Card
    ImGui::BeginChild("##SurfaceStatusCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        if (telemetry.surface_probing_active) {
            int cur_pt = telemetry.surface_probe_current_point;
            int tot_pts = std::max(1, telemetry.surface_probe_total_points);
            float frac = std::clamp(static_cast<float>(cur_pt) / static_cast<float>(tot_pts), 0.0f, 1.0f);

            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "[PROBING SURFACE GRID IN PROGRESS]");
            ImGui::Spacing();

            char prog_txt[64];
            std::snprintf(prog_txt, sizeof(prog_txt), "Point %d of %d (%.0f%%)", cur_pt, tot_pts, frac * 100.0f);
            ImGui::ProgressBar(frac, ImVec2(-1.0f, 24.0f), prog_txt);
            ImGui::Spacing();

            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.18f, 0.18f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.25f, 0.25f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.65f, 0.12f, 0.12f, 1.0f));
            if (ImGui::Button("ABORT SURFACE PROBING", ImVec2(-1.0f, 32.0f))) {
                controller.cancelSurfaceProbing();
            }
            ImGui::PopStyleColor(3);
        } else if (telemetry.surface_compensation_active) {
            ImGui::TextColored(ImVec4(0.15f, 0.95f, 0.45f, 1.0f), "[SURFACE LEVELING COMPENSATION: ACTIVE]");
            ImGui::TextWrapped("Toolpath Z-coordinates are actively warped to workpiece surface topology based on probed height map.");
        } else if (telemetry.surface_height_map->is_valid) {
            ImGui::TextColored(ImVec4(0.35f, 0.75f, 1.0f, 1.0f), "[HEIGHT MAP STORED - COMPENSATION OFF]");
            ImGui::TextWrapped("Probed height map is ready. Click 'Apply Compensation' below to warp the active toolpath.");
        } else {
            ImGui::TextColored(ImVec4(0.65f, 0.70f, 0.78f, 1.0f), "[SURFACE PROBING: READY]");
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.60f, 0.68f, 1.0f));
            ImGui::TextWrapped("Define grid rectangle & resolution below to probe workpiece surface contour.");
            ImGui::PopStyleColor();
        }
    }
    ImGui::EndChild();

    ImGui::Spacing();

    // 2. Probing Area Grid Rectangle Card
    ImGui::BeginChild("##SurfaceAreaCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "SURFACE GRID BOUNDARIES (WORK COORDINATES)");
        ImGui::Spacing();

        const bool is_imperial = (controller.getConfig().linear_units == core::LinearUnits::Imperial);
        const double u_scale = is_imperial ? (1.0 / 25.4) : 1.0;
        const double u_unscale = is_imperial ? 25.4 : 1.0;
        const char* u_str = is_imperial ? "in" : "mm";

        const float half_w = (ImGui::GetContentRegionAvail().x - 10.0f) * 0.5f;

        float f_xmin = static_cast<float>(surface_min_x_ * u_scale);
        float f_xmax = static_cast<float>(surface_max_x_ * u_scale);
        float f_ymin = static_cast<float>(surface_min_y_ * u_scale);
        float f_ymax = static_cast<float>(surface_max_y_ * u_scale);

        const float step_fast = is_imperial ? 0.25f : 10.0f;
        const float step_slow = is_imperial ? 0.05f : 1.0f;
        const char* val_fmt = is_imperial ? "%.3f" : "%.2f";

        char lbl_xmin[32], lbl_xmax[32], lbl_ymin[32], lbl_ymax[32];
        std::snprintf(lbl_xmin, sizeof(lbl_xmin), "X Min (%s)", u_str);
        std::snprintf(lbl_xmax, sizeof(lbl_xmax), "X Max (%s)", u_str);
        std::snprintf(lbl_ymin, sizeof(lbl_ymin), "Y Min (%s)", u_str);
        std::snprintf(lbl_ymax, sizeof(lbl_ymax), "Y Max (%s)", u_str);

        ImGui::BeginDisabled(telemetry.surface_probing_active);

        ImGui::SetNextItemWidth(half_w);
        if (ImGui::InputFloat(lbl_xmin, &f_xmin, step_slow, step_fast, val_fmt)) {
            surface_min_x_ = f_xmin * u_unscale;
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(half_w);
        if (ImGui::InputFloat(lbl_xmax, &f_xmax, step_slow, step_fast, val_fmt)) {
            surface_max_x_ = f_xmax * u_unscale;
        }

        ImGui::SetNextItemWidth(half_w);
        if (ImGui::InputFloat(lbl_ymin, &f_ymin, step_slow, step_fast, val_fmt)) {
            surface_min_y_ = f_ymin * u_unscale;
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(half_w);
        if (ImGui::InputFloat(lbl_ymax, &f_ymax, step_slow, step_fast, val_fmt)) {
            surface_max_y_ = f_ymax * u_unscale;
        }

        ImGui::EndDisabled();

        double span_w = std::max(0.0, surface_max_x_ - surface_min_x_);
        double span_h = std::max(0.0, surface_max_y_ - surface_min_y_);
        ImGui::Spacing();
        if (is_imperial) {
            double span_w_in = span_w * u_scale;
            double span_h_in = span_h * u_scale;
            ImGui::TextDisabled("Area: %.2f in (W) x %.2f in (H)  |  %.2f sq in",
                                span_w_in, span_h_in, span_w_in * span_h_in);
        } else {
            ImGui::TextDisabled("Area: %.1f mm (W) x %.1f mm (H)  |  %.1f cm2",
                                span_w, span_h, span_w * span_h / 100.0);
        }

        ImGui::Spacing();

        // Viewport selection and G-Code bounds buttons
        bool is_selecting = (viewport && viewport->isSelectingSurfaceRect());
        if (is_selecting) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.55f, 0.15f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.18f, 0.24f, 0.32f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.32f, 0.44f, 1.0f));
        }

        ImGui::BeginDisabled(telemetry.surface_probing_active);

        if (ImGui::Button(is_selecting ? "Cancel Viewport Selection##Srf" : "Select Area on Viewport##Srf", ImVec2(half_w, 28.0f))) {
            if (viewport) {
                if (is_selecting) {
                    viewport->cancelSelectingSurfaceRect();
                } else {
                    viewport->startSelectingSurfaceRect([this](double x0, double x1, double y0, double y1) {
                        surface_min_x_ = x0;
                        surface_max_x_ = x1;
                        surface_min_y_ = y0;
                        surface_max_y_ = y1;
                    });
                }
            }
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Switch to 2D toolpath viewport and drag a rectangle over the desired workpiece area.");
        }

        ImGui::SameLine();

        if (ImGui::Button("Fit to G-Code Bounds##Srf", ImVec2(half_w, 28.0f))) {
            std::array<double, 3> bmin{}, bmax{};
            if (controller.getProgramBounds(bmin, bmax)) {
                surface_min_x_ = bmin[0];
                surface_max_x_ = bmax[0];
                surface_min_y_ = bmin[1];
                surface_max_y_ = bmax[1];
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Automatically match boundaries to currently loaded G-Code program footprint.");
        }

        ImGui::EndDisabled();
    }
    ImGui::EndChild();

    ImGui::Spacing();

    // 3. Grid Resolution & Density Card
    ImGui::BeginChild("##SurfaceGridCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "PROBE GRID DENSITY & RESOLUTION");
        ImGui::Spacing();

        const float avail_grid_w = ImGui::GetContentRegionAvail().x;
        const float col_w = (avail_grid_w - 12.0f) * 0.5f;
        const float lbl_x_w = ImGui::CalcTextSize("X Points").x;
        const float lbl_y_w = ImGui::CalcTextSize("Y Points").x;
        const float slider_x_w = std::max(45.0f, col_w - lbl_x_w - 12.0f);
        const float slider_y_w = std::max(45.0f, col_w - lbl_y_w - 12.0f);

        ImGui::BeginDisabled(telemetry.surface_probing_active);

        surface_points_x_ = std::clamp(surface_points_x_, 2, surface_max_points_);
        surface_points_y_ = std::clamp(surface_points_y_, 2, surface_max_points_);

        ImGui::SetNextItemWidth(slider_x_w);
        ImGui::SliderInt("X Points##Srf", &surface_points_x_, 2, surface_max_points_);
        if (ImGui::BeginPopupContextItem("##XPointsCtx")) {
            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "Grid Points Limit");
            ImGui::Separator();
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Max:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70.0f);
            if (ImGui::InputInt("##MaxPointsX", &surface_max_points_, 1, 5)) {
                surface_max_points_ = std::clamp(surface_max_points_, 5, 200);
                cfg.probe.surface_max_points = surface_max_points_;
            }
            ImGui::EndPopup();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Drag to select X grid points (2..%d).\nRight-click to adjust Max points limit.", surface_max_points_);
        }

        ImGui::SameLine(col_w + 12.0f);
        ImGui::SetNextItemWidth(slider_y_w);
        ImGui::SliderInt("Y Points##Srf", &surface_points_y_, 2, surface_max_points_);
        if (ImGui::BeginPopupContextItem("##YPointsCtx")) {
            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "Grid Points Limit");
            ImGui::Separator();
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Max:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70.0f);
            if (ImGui::InputInt("##MaxPointsY", &surface_max_points_, 1, 5)) {
                surface_max_points_ = std::clamp(surface_max_points_, 5, 200);
                cfg.probe.surface_max_points = surface_max_points_;
            }
            ImGui::EndPopup();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Drag to select Y grid points (2..%d).\nRight-click to adjust Max points limit.", surface_max_points_);
        }

        ImGui::EndDisabled();

        int total_pts = surface_points_x_ * surface_points_y_;
        double dx = (surface_points_x_ > 1) ? (surface_max_x_ - surface_min_x_) / (surface_points_x_ - 1) : 0.0;
        double dy = (surface_points_y_ > 1) ? (surface_max_y_ - surface_min_y_) / (surface_points_y_ - 1) : 0.0;

        const bool is_imperial = (controller.getConfig().linear_units == core::LinearUnits::Imperial);
        const double u_scale = is_imperial ? (1.0 / 25.4) : 1.0;

        ImGui::Spacing();
        if (is_imperial) {
            ImGui::TextDisabled("Total Points: %d (%d x %d)  |  dX: %.3f in, dY: %.3f in",
                                total_pts, surface_points_x_, surface_points_y_, dx * u_scale, dy * u_scale);
        } else {
            ImGui::TextDisabled("Total Points: %d (%d x %d)  |  dX: %.1f mm, dY: %.1f mm",
                                total_pts, surface_points_x_, surface_points_y_, dx, dy);
        }
    }
    ImGui::EndChild();

    ImGui::Spacing();

    // 3b. Surface Interpolation & Smoothness Slider Card with Apply Action
    ImGui::BeginChild("##SurfaceSmoothnessCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "SURFACE INTERPOLATION & SMOOTHNESS");
        ImGui::Spacing();

        const bool has_map = telemetry.surface_height_map->is_valid;
        const bool has_pending_change = has_map && (std::abs(surface_smoothness_ - applied_smoothness_) > 0.005f);

        const float avail = ImGui::GetContentRegionAvail().x;
        const float btn_w = 76.0f;
        const float slider_w = std::max(120.0f, avail - btn_w - ImGui::GetStyle().ItemSpacing.x);

        ImGui::BeginDisabled(telemetry.surface_probing_active);

        int smoothness_pct = std::clamp(static_cast<int>(std::round(surface_smoothness_ * 100.0f)), 0, 100);

        ImGui::SetNextItemWidth(slider_w);
        if (ImGui::SliderInt("##SmoothnessSlider", &smoothness_pct, 0, 100, "%d%%")) {
            surface_smoothness_ = static_cast<float>(smoothness_pct) / 100.0f;
            cfg.probe.surface_smoothness = surface_smoothness_;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("0%% = Rigid Bilinear (flat facets, 0%% overshoot risk).\n"
                              "50%% = Balanced Spline (smooth C1 transitions, minimal overshoot).\n"
                              "100%% = Pure Bicubic Spline (maximum organic curvature).");
        }

        ImGui::SameLine();

        // Apply Button right next to the slider
        if (has_pending_change) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.12f, 0.65f, 0.35f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.16f, 0.78f, 0.42f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text,         ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.18f, 0.22f, 0.28f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.30f, 0.38f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text,         ImVec4(0.65f, 0.70f, 0.78f, 1.0f));
        }

        ImGui::BeginDisabled(!has_pending_change || telemetry.surface_probing_active);
        if (ImGui::Button("Apply##SrfSmooth", ImVec2(btn_w, 0.0f))) {
            core::SurfaceHeightMap updated_map = *telemetry.surface_height_map;
            updated_map.smoothness = surface_smoothness_;
            controller.applySurfaceHeightMap(updated_map);
            applied_smoothness_ = surface_smoothness_;
            cfg.probe.surface_smoothness = surface_smoothness_;
        }
        ImGui::EndDisabled();
        ImGui::PopStyleColor(3);

        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            if (has_pending_change) {
                ImGui::SetTooltip("Click to recalculate and re-warp active G-Code toolpath using proposed %d%% smoothness.", smoothness_pct);
            } else if (!has_map) {
                ImGui::SetTooltip("Probed height map required. Slider value will be applied automatically upon probe completion.");
            } else {
                ImGui::SetTooltip("Currently applied to toolpath. Adjust slider to preview another smoothness level.");
            }
        }

        // Quick Preset Buttons
        ImGui::Spacing();
        const float preset_w = (avail - ImGui::GetStyle().ItemSpacing.x * 2.0f) / 3.0f;
        auto drawPresetBtn = [&](const char* label, float val) {
            bool is_curr = (std::abs(surface_smoothness_ - val) < 0.01f);
            if (is_curr) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.52f, 0.72f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f, 0.18f, 0.24f, 1.0f));
            }
            if (ImGui::Button(label, ImVec2(preset_w, 28.0f))) {
                surface_smoothness_ = val;
                cfg.probe.surface_smoothness = surface_smoothness_;
            }
            ImGui::PopStyleColor();
        };

        drawPresetBtn("0% Rigid", 0.0f);
        ImGui::SameLine();
        drawPresetBtn("50% Balanced", 0.5f);
        ImGui::SameLine();
        drawPresetBtn("100% Spline", 1.0f);

        ImGui::EndDisabled();

        // Descriptive profile label and explanation
        const char* profile_desc = "Balanced Spline Blend (Recommended)";
        if (smoothness_pct == 0) {
            profile_desc = "Rigid Bilinear (Flat facets, 0% overshoot)";
        } else if (smoothness_pct < 40) {
            profile_desc = "Subtle Spline Blend";
        } else if (smoothness_pct <= 60) {
            profile_desc = "Balanced Spline Blend (Recommended)";
        } else if (smoothness_pct < 100) {
            profile_desc = "High Curvature Spline";
        } else {
            profile_desc = "Pure Bicubic Spline (Max organic curvature)";
        }

        ImGui::Spacing();
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(ImVec4(0.35f, 0.75f, 1.0f, 1.0f), "Profile: %s", profile_desc);

        // Helper explanation / status text below slider (properly wrapped to prevent truncation)
        if (has_pending_change) {
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f),
                               "* Previewing proposed %d%% smoothness in viewport. Click 'Apply' to re-warp G-Code toolpath.",
                               smoothness_pct);
        } else if (has_map && telemetry.surface_compensation_active) {
            ImGui::TextDisabled("Toolpath is active and warped at %d%% smoothness.", static_cast<int>(std::round(applied_smoothness_ * 100.0f)));
        } else {
            ImGui::TextDisabled("Selected smoothness (%d%%) will be automatically applied upon probe completion.", smoothness_pct);
        }
        ImGui::PopTextWrapPos();
    }
    ImGui::EndChild();

    ImGui::Spacing();

    // 4. Action Buttons (Start Probing / Cancel / Apply)
    {
        bool area_valid = (surface_max_x_ > surface_min_x_ + 0.5) &&
                          (surface_max_y_ > surface_min_y_ + 0.5) &&
                          (surface_points_x_ >= 2) && (surface_points_y_ >= 2);
        bool can_probe = area_valid &&
                         !telemetry.surface_probing_active &&
                         (telemetry.state == core::ControllerState::Idle ||
                          telemetry.state == core::ControllerState::ToolChangeHold);

        if (!telemetry.surface_probing_active) {
            ImGui::BeginDisabled(!can_probe);
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.10f, 0.62f, 0.30f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.14f, 0.76f, 0.38f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.08f, 0.52f, 0.24f, 1.0f));

            if (ImGui::Button("START SURFACE PROBING##Srf", ImVec2(-1.0f, 38.0f))) {
                applied_smoothness_ = surface_smoothness_;
                controller.startSurfaceProbing(surface_min_x_, surface_max_x_,
                                              surface_min_y_, surface_max_y_,
                                              surface_points_x_, surface_points_y_,
                                              surface_smoothness_);
            }
            ImGui::PopStyleColor(3);
            ImGui::EndDisabled();

            if (!area_valid && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("X Max must be > X Min and Y Max must be > Y Min with at least 2x2 grid points.");
            } else if (!can_probe && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("Machine must be in Idle or Tool Change Hold state to begin probing.");
            } else if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Executes boustrophedon zigzag surface probe sequence across defined grid.");
            }
        }

        // Compensation control buttons
        if (telemetry.surface_height_map->is_valid && !telemetry.surface_probing_active) {
            ImGui::Spacing();
            const float half_btn_w = (ImGui::GetContentRegionAvail().x - 10.0f) * 0.5f;

            if (telemetry.surface_compensation_active) {
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.45f, 0.10f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.55f, 0.14f, 1.0f));
                if (ImGui::Button("Cancel / Disable Compensation##Srf", ImVec2(half_btn_w, 32.0f))) {
                    controller.clearSurfaceHeightMap();
                }
                ImGui::PopStyleColor(2);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Disables surface compensation and restores nominal flat toolpath.");
                }
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.12f, 0.55f, 0.35f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.16f, 0.68f, 0.42f, 1.0f));
                if (ImGui::Button("Apply Compensation##Srf", ImVec2(half_btn_w, 32.0f))) {
                    core::SurfaceHeightMap updated_map = *telemetry.surface_height_map;
                    updated_map.smoothness = surface_smoothness_;
                    controller.applySurfaceHeightMap(updated_map);
                    applied_smoothness_ = surface_smoothness_;
                }
                ImGui::PopStyleColor(2);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Applies stored height map to loaded G-Code toolpath with chord subdivision.");
                }
            }

            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.24f, 0.28f, 0.34f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.32f, 0.38f, 0.46f, 1.0f));
            if (ImGui::Button("Clear Height Map##Srf", ImVec2(half_btn_w, 32.0f))) {
                controller.clearSurfaceHeightMap();
                core::SurfaceHeightMap empty_map{};
                controller.applySurfaceHeightMap(empty_map);
            }
            ImGui::PopStyleColor(2);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Erases probed height map and resets surface compensation.");
            }
        }
    }

    // 5. Surface Statistics Card
    if (telemetry.surface_height_map->is_valid) {
        ImGui::Spacing();
        ImGui::BeginChild("##SurfaceStatsCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
        {
            const auto& shm = *telemetry.surface_height_map;
            ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "PROBED SURFACE TOPOLOGY & STATISTICS");
            ImGui::Spacing();

            ImGui::Columns(2, "##shm_cols", false);
            ImGui::TextDisabled("Grid Size:");
            ImGui::NextColumn();
            ImGui::Text("%d x %d (%zu pts)", shm.points_x, shm.points_y, shm.grid_z.size());
            ImGui::NextColumn();

            const bool is_imperial_shm = (controller.getConfig().linear_units == core::LinearUnits::Imperial);
            const double shm_scale = is_imperial_shm ? (1.0 / 25.4) : 1.0;
            const char* z_fmt = is_imperial_shm ? "%+.4f in" : "%+.3f mm";
            const char* p2p_fmt = is_imperial_shm ? "%.4f in" : "%.3f mm";

            ImGui::TextDisabled("Min dZ:");
            ImGui::NextColumn();
            ImGui::TextColored(shm.min_z < 0.0 ? ImVec4(0.4f, 0.7f, 1.0f, 1.0f) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f),
                               z_fmt, shm.min_z * shm_scale);
            ImGui::NextColumn();

            ImGui::TextDisabled("Max dZ:");
            ImGui::NextColumn();
            ImGui::TextColored(shm.max_z > 0.0 ? ImVec4(1.0f, 0.6f, 0.3f, 1.0f) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f),
                               z_fmt, shm.max_z * shm_scale);
            ImGui::NextColumn();

            ImGui::TextDisabled("Peak-to-Peak Warp:");
            ImGui::NextColumn();
            ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.25f, 1.0f), p2p_fmt, shm.peak_to_peak_z * shm_scale);
            ImGui::NextColumn();

            ImGui::TextDisabled("Mean Offset:");
            ImGui::NextColumn();
            ImGui::Text(z_fmt, shm.mean_z * shm_scale);
            ImGui::Columns(1);
        }
        ImGui::EndChild();
    }

    ImGui::PopStyleVar(4);
    ImGui::PopStyleColor(2);
}


void ProbePanel::renderOutsideDeck(core::IMotionController& controller, float avail_w) {
    const float card_w = avail_w;
    const float card_h = 340.0f;
    ImVec2 card_pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Card background
    dl->AddRectFilled(card_pos, ImVec2(card_pos.x + card_w, card_pos.y + card_h), col_card_bg, 6.0f);
    dl->AddRect(card_pos, ImVec2(card_pos.x + card_w, card_pos.y + card_h), col_card_border, 6.0f);

    const float cx = card_pos.x + card_w * 0.5f;
    const float cy = card_pos.y + card_h * 0.5f;

    // Workpiece block sized proportionally to fill space evenly
    const float box_size = std::clamp(card_w - 110.0f, 180.0f, 212.0f);
    const float half_b   = box_size * 0.5f;
    const float tile_sz  = box_size / 3.0f;
    const float x0       = cx - half_b;
    const float y0       = cy - half_b;
    const float x1       = x0 + box_size;
    const float y1       = y0 + box_size;

    // Draw 3x3 segmented metallic workpiece
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            ImVec2 p_a(x0 + c * tile_sz + 1.0f, y0 + r * tile_sz + 1.0f);
            ImVec2 p_b(p_a.x + tile_sz - 2.0f, p_a.y + tile_sz - 2.0f);

            if (r == 1 && c == 1) {
                // Dark graphite center square
                dl->AddRectFilled(p_a, p_b, col_center_tile, 4.0f);
                dl->AddRect(p_a, p_b, col_metal_border, 4.0f, 0, 1.2f);
            } else {
                drawMetalTile(dl, p_a, p_b, 4.0f);
            }
        }
    }

    // Workpiece label in center tile
    const char* lbl1 = "WORK";
    const char* lbl2 = "PIECE";
    ImVec2 sz1 = ImGui::CalcTextSize(lbl1);
    ImVec2 sz2 = ImGui::CalcTextSize(lbl2);
    dl->AddText(ImVec2(cx - sz1.x * 0.5f, cy - sz1.y - 1.0f), IM_COL32(230, 238, 250, 250), lbl1);
    dl->AddText(ImVec2(cx - sz2.x * 0.5f, cy + 1.0f), IM_COL32(230, 238, 250, 250), lbl2);

    // Cyan/Blue thin rounded bounding frame outside the workpiece (enlarged frame)
    const float frame_pad = 25.0f;
    ImVec2 f_min(x0 - frame_pad, y0 - frame_pad);
    ImVec2 f_max(x1 + frame_pad, y1 + frame_pad);
    dl->AddRect(f_min, f_max, col_blue_border, 18.0f, 0, 1.2f);

    // Arrow parameters
    const float dot_r = 8.5f;

    // Main center arrows
    const float main_sw = 9.0f;
    const float main_hw = 20.0f;
    const float main_hl = 13.0f;
    const float main_len = 22.0f;
    const float main_gap = 12.0f; // Gap between arrow tip and workpiece edge

    // Corner pair arrows - placed closer to corners so diagonal line touches workpiece corner
    const float c_sw   = 6.0f;
    const float c_hw   = 14.0f;
    const float c_hl   = 9.0f;
    const float c_len  = 16.0f;
    const float c_dist = 13.0f; // Offset along workpiece edge
    const float c_gap  = 13.0f; // Perpendicular gap to arrow tip

    // Arrow tip positions for main center arrows
    const float top_main_tip_y  = y0 - main_gap;
    const float bot_main_tip_y  = y1 + main_gap;
    const float left_main_tip_x = x0 - main_gap;
    const float right_main_tip_x = x1 + main_gap;

    // Corner arrow tip coordinates:
    // With c_dist == c_gap, the midpoint between the two corner arrow tips is exactly the workpiece corner (x0, y0), etc.
    const ImVec2 tl_top_tip(x0 + c_dist, y0 - c_gap);
    const ImVec2 tl_left_tip(x0 - c_gap, y0 + c_dist);

    const ImVec2 tr_top_tip(x1 - c_dist, y0 - c_gap);
    const ImVec2 tr_right_tip(x1 + c_gap, y0 + c_dist);

    const ImVec2 bl_bot_tip(x0 + c_dist, y1 + c_gap);
    const ImVec2 bl_left_tip(x0 - c_gap, y1 - c_dist);

    const ImVec2 br_bot_tip(x1 - c_dist, y1 + c_gap);
    const ImVec2 br_right_tip(x1 + c_gap, y1 - c_dist);

    // -------------------------------------------------------------------------
    // DIAGONAL PURPLE DASHED LINES IN CORNERS
    // Line passes directly behind the orange corner dot and touches the workpiece corner
    drawDashedLine(dl, tl_top_tip, tl_left_tip, col_dash_line, 1.4f, 4.0f, 3.0f);
    drawDashedLine(dl, tr_top_tip, tr_right_tip, col_dash_line, 1.4f, 4.0f, 3.0f);
    drawDashedLine(dl, bl_bot_tip, bl_left_tip, col_dash_line, 1.4f, 4.0f, 3.0f);
    drawDashedLine(dl, br_bot_tip, br_right_tip, col_dash_line, 1.4f, 4.0f, 3.0f);

    // Dashed purple ticks connecting center arrows to the center edge dots
    drawDashedLine(dl, ImVec2(cx, top_main_tip_y), ImVec2(cx, y0), col_dash_line, 1.2f, 3.0f, 2.0f);
    drawDashedLine(dl, ImVec2(cx, bot_main_tip_y), ImVec2(cx, y1), col_dash_line, 1.2f, 3.0f, 2.0f);
    drawDashedLine(dl, ImVec2(left_main_tip_x, cy), ImVec2(x0, cy), col_dash_line, 1.2f, 3.0f, 2.0f);
    drawDashedLine(dl, ImVec2(right_main_tip_x, cy), ImVec2(x1, cy), col_dash_line, 1.2f, 3.0f, 2.0f);

    // -------------------------------------------------------------------------
    // CHUNKY WHITE ARROWS
    // Top Edge:
    drawChunkyArrow(dl, ImVec2(tl_top_tip.x, tl_top_tip.y - c_len), tl_top_tip, c_sw, c_hw, c_hl);
    drawChunkyArrow(dl, ImVec2(cx, top_main_tip_y - main_len), ImVec2(cx, top_main_tip_y), main_sw, main_hw, main_hl);
    drawChunkyArrow(dl, ImVec2(tr_top_tip.x, tr_top_tip.y - c_len), tr_top_tip, c_sw, c_hw, c_hl);

    // Bottom Edge:
    drawChunkyArrow(dl, ImVec2(bl_bot_tip.x, bl_bot_tip.y + c_len), bl_bot_tip, c_sw, c_hw, c_hl);
    drawChunkyArrow(dl, ImVec2(cx, bot_main_tip_y + main_len), ImVec2(cx, bot_main_tip_y), main_sw, main_hw, main_hl);
    drawChunkyArrow(dl, ImVec2(br_bot_tip.x, br_bot_tip.y + c_len), br_bot_tip, c_sw, c_hw, c_hl);

    // Left Edge:
    drawChunkyArrow(dl, ImVec2(tl_left_tip.x - c_len, tl_left_tip.y), tl_left_tip, c_sw, c_hw, c_hl);
    drawChunkyArrow(dl, ImVec2(left_main_tip_x - main_len, cy), ImVec2(left_main_tip_x, cy), main_sw, main_hw, main_hl);
    drawChunkyArrow(dl, ImVec2(bl_left_tip.x - c_len, bl_left_tip.y), bl_left_tip, c_sw, c_hw, c_hl);

    // Right Edge:
    drawChunkyArrow(dl, ImVec2(tr_right_tip.x + c_len, tr_right_tip.y), tr_right_tip, c_sw, c_hw, c_hl);
    drawChunkyArrow(dl, ImVec2(right_main_tip_x + main_len, cy), ImVec2(right_main_tip_x, cy), main_sw, main_hw, main_hl);
    drawChunkyArrow(dl, ImVec2(br_right_tip.x + c_len, br_right_tip.y), br_right_tip, c_sw, c_hw, c_hl);

    // -------------------------------------------------------------------------
    // 8 ORANGE DOTS ON THE GREY WORKPIECE BLOCKS
    renderHotspot("##OutTL", ImVec2(x0, y0), dot_r, "TL",
                  "Outside Top-Left Corner\nProbes X+ and Y- edges sequentially to set Work X0 Y0",
                  "scripts/mill/probing/outside_corner_top_left.lua", controller);

    renderHotspot("##OutTop", ImVec2(cx, y0), dot_r, "T",
                  "Outside Top Edge (Y+)\nProbes toward Y- to set Work Y0",
                  "scripts/mill/probing/outside_edge_top.lua", controller);

    renderHotspot("##OutTR", ImVec2(x1, y0), dot_r, "TR",
                  "Outside Top-Right Corner\nProbes X- and Y- edges sequentially to set Work X0 Y0",
                  "scripts/mill/probing/outside_corner_top_right.lua", controller);

    renderHotspot("##OutLeft", ImVec2(x0, cy), dot_r, "L",
                  "Outside Left Edge (X-)\nProbes toward X+ to set Work X0",
                  "scripts/mill/probing/outside_edge_left.lua", controller);

    renderHotspot("##OutRight", ImVec2(x1, cy), dot_r, "R",
                  "Outside Right Edge (X+)\nProbes toward X- to set Work X0",
                  "scripts/mill/probing/outside_edge_right.lua", controller);

    renderHotspot("##OutBL", ImVec2(x0, y1), dot_r, "BL",
                  "Outside Bottom-Left Corner\nProbes X+ and Y+ edges sequentially to set Work X0 Y0",
                  "scripts/mill/probing/outside_corner_bottom_left.lua", controller);

    renderHotspot("##OutBottom", ImVec2(cx, y1), dot_r, "B",
                  "Outside Bottom Edge (Y-)\nProbes toward Y+ to set Work Y0",
                  "scripts/mill/probing/outside_edge_bottom.lua", controller);

    renderHotspot("##OutBR", ImVec2(x1, y1), dot_r, "BR",
                  "Outside Bottom-Right Corner\nProbes X- and Y+ edges sequentially to set Work X0 Y0",
                  "scripts/mill/probing/outside_corner_bottom_right.lua", controller);

    ImGui::SetCursorScreenPos(ImVec2(card_pos.x, card_pos.y + card_h + 8.0f));
}

void ProbePanel::renderInsideDeck(core::IMotionController& controller, float avail_w) {
    const float card_w = avail_w;
    const float card_h = 340.0f;
    ImVec2 card_pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Card background
    dl->AddRectFilled(card_pos, ImVec2(card_pos.x + card_w, card_pos.y + card_h), col_card_bg, 6.0f);
    dl->AddRect(card_pos, ImVec2(card_pos.x + card_w, card_pos.y + card_h), col_card_border, 6.0f);

    const float cx = card_pos.x + card_w * 0.5f;
    const float cy = card_pos.y + card_h * 0.5f;

    // Workpiece sized to match outside boundary rectangle (~262px)
    // 5px thinner wall all around (wall_w = 43px instead of 48px, expanding inner pocket to ~176px)
    const float outside_box = std::clamp(card_w - 110.0f, 180.0f, 212.0f);
    const float outer_sz    = outside_box + 2.0f * 25.0f; // Exactly matches outside boundary rectangle
    const float half_out    = outer_sz * 0.5f;
    const float wall_w      = std::round(outer_sz * 0.182f) - 5.0f; // 5px thinner all around (~43px)
    const float inner_sz    = outer_sz - 2.0f * wall_w;     // ~176px spacious pocket cavity
    const float half_in     = inner_sz * 0.5f;
    const float tile_sz     = outer_sz / 3.0f;              // 3x3 segmented workpiece tile size

    const float x0 = cx - half_out;
    const float y0 = cy - half_out;
    const float x1 = cx + half_out;
    const float y1 = cy + half_out;

    const float px0 = cx - half_in;
    const float py0 = cy - half_in;
    const float px1 = cx + half_in;
    const float py1 = cy + half_in;

    // 1. Draw outer metallic workpiece block with smooth vertical grey gradient and 18px rounded corners
    // (exact same 18px rounding as the OUTSIDE bounding rectangle)
    const float corner_r = 18.0f;
    auto lerpCol = [](ImU32 c1, ImU32 c2, float t) -> ImU32 {
        int r1 = (c1 >> 0) & 0xFF, g1 = (c1 >> 8) & 0xFF, b1 = (c1 >> 16) & 0xFF, a1 = (c1 >> 24) & 0xFF;
        int r2 = (c2 >> 0) & 0xFF, g2 = (c2 >> 8) & 0xFF, b2 = (c2 >> 16) & 0xFF, a2 = (c2 >> 24) & 0xFF;
        return IM_COL32((int)(r1 + (r2 - r1) * t),
                        (int)(g1 + (g2 - g1) * t),
                        (int)(b1 + (b2 - b1) * t),
                        (int)(a1 + (a2 - a1) * t));
    };

    const float t_top = corner_r / outer_sz;
    const float t_bot = (outer_sz - corner_r) / outer_sz;
    const ImU32 col_top_mid = lerpCol(col_metal_top, col_metal_bot, t_top);
    const ImU32 col_bot_mid = lerpCol(col_metal_top, col_metal_bot, t_bot);

    // Top cap with top rounded corners (18px)
    dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y0 + corner_r), col_metal_top, corner_r, ImDrawFlags_RoundCornersTop);
    // Middle body with smooth vertical multicolor gradient
    dl->AddRectFilledMultiColor(ImVec2(x0, y0 + corner_r), ImVec2(x1, y1 - corner_r), col_top_mid, col_top_mid, col_bot_mid, col_bot_mid);
    // Bottom cap with bottom rounded corners (18px)
    dl->AddRectFilled(ImVec2(x0, y1 - corner_r), ImVec2(x1, y1), col_metal_bot, corner_r, ImDrawFlags_RoundCornersBottom);

    // Outer border with 18px rounding (matching OUTSIDE graphic boundary)
    dl->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), col_metal_border, corner_r, 0, 1.2f);
    // Top bevel highlight between the rounded corners
    dl->AddLine(ImVec2(x0 + corner_r, y0 + 1.0f), ImVec2(x1 - corner_r, y0 + 1.0f), col_metal_bevel, 1.0f);

    // 2. Draw pocket cavity
    dl->AddRectFilled(ImVec2(px0, py0), ImVec2(px1, py1), col_pocket_bg, 3.0f);
    dl->AddRect(ImVec2(px0, py0), ImVec2(px1, py1), col_pocket_shadow, 3.0f, 0, 1.8f);

    // 3. Workpiece seam grooves (dividing outer stock into 8 tiles with 3x3 layout, preserving vertical middle sections!)
    const float x_s1 = x0 + tile_sz;
    const float x_s2 = x0 + 2.0f * tile_sz;
    const float y_s1 = y0 + tile_sz;
    const float y_s2 = y0 + 2.0f * tile_sz;
    const ImU32 col_seam = IM_COL32(22, 26, 32, 255);

    // Top vertical seams (at 1/3 and 2/3)
    dl->AddLine(ImVec2(x_s1, y0), ImVec2(x_s1, py0), col_seam, 1.8f);
    dl->AddLine(ImVec2(x_s1 + 1.0f, y0), ImVec2(x_s1 + 1.0f, py0), col_metal_bevel, 1.0f);
    dl->AddLine(ImVec2(x_s2, y0), ImVec2(x_s2, py0), col_seam, 1.8f);
    dl->AddLine(ImVec2(x_s2 + 1.0f, y0), ImVec2(x_s2 + 1.0f, py0), col_metal_bevel, 1.0f);

    // Bottom vertical seams (at 1/3 and 2/3)
    dl->AddLine(ImVec2(x_s1, py1), ImVec2(x_s1, y1), col_seam, 1.8f);
    dl->AddLine(ImVec2(x_s1 + 1.0f, py1), ImVec2(x_s1 + 1.0f, y1), col_metal_bevel, 1.0f);
    dl->AddLine(ImVec2(x_s2, py1), ImVec2(x_s2, y1), col_seam, 1.8f);
    dl->AddLine(ImVec2(x_s2 + 1.0f, py1), ImVec2(x_s2 + 1.0f, y1), col_metal_bevel, 1.0f);

    // Left horizontal seams (at 1/3 and 2/3 - framing the vertical middle section!)
    dl->AddLine(ImVec2(x0, y_s1), ImVec2(px0, y_s1), col_seam, 1.8f);
    dl->AddLine(ImVec2(x0, y_s1 + 1.0f), ImVec2(px0, y_s1 + 1.0f), col_metal_bevel, 1.0f);
    dl->AddLine(ImVec2(x0, y_s2), ImVec2(px0, y_s2), col_seam, 1.8f);
    dl->AddLine(ImVec2(x0, y_s2 + 1.0f), ImVec2(px0, y_s2 + 1.0f), col_metal_bevel, 1.0f);

    // Right horizontal seams (at 1/3 and 2/3 - framing the vertical middle section!)
    dl->AddLine(ImVec2(px1, y_s1), ImVec2(x1, y_s1), col_seam, 1.8f);
    dl->AddLine(ImVec2(px1, y_s1 + 1.0f), ImVec2(x1, y_s1 + 1.0f), col_metal_bevel, 1.0f);
    dl->AddLine(ImVec2(px1, y_s2), ImVec2(x1, y_s2), col_seam, 1.8f);
    dl->AddLine(ImVec2(px1, y_s2 + 1.0f), ImVec2(x1, y_s2 + 1.0f), col_metal_bevel, 1.0f);

    // 4. Cyan/Blue thin rounded bounding frame inside pocket
    const float p_inset = 12.0f;
    dl->AddRect(ImVec2(px0 + p_inset, py0 + p_inset),
                ImVec2(px1 - p_inset, py1 - p_inset),
                col_blue_border, 16.0f, 0, 1.2f);

    // 5. Center pocket label
    const char* lbl1 = "WORK";
    const char* lbl2 = "PIECE";
    ImVec2 sz1 = ImGui::CalcTextSize(lbl1);
    ImVec2 sz2 = ImGui::CalcTextSize(lbl2);
    dl->AddText(ImVec2(cx - sz1.x * 0.5f, cy - sz1.y - 1.0f), IM_COL32(190, 205, 225, 230), lbl1);
    dl->AddText(ImVec2(cx - sz2.x * 0.5f, cy + 1.0f), IM_COL32(190, 205, 225, 230), lbl2);

    const float dot_r = 8.5f;

    // Arrow parameters - short, compact, chunky matching original reference
    const float main_sw  = 12.0f; // Broad chunky stem
    const float main_hw  = 22.0f; // Wide arrowhead
    const float main_hl  = 13.0f; // Arrowhead length
    const float main_gap = 16.0f; // Gap between arrow tip and pocket wall
    const float main_len = 28.0f; // Shortened length for compact, non-dominating arrows

    // Corner arrows made smaller with separated bases so they never overlap
    const float c_sw      = 6.0f;  // Slender, clean corner stem
    const float c_hw      = 13.0f; // Crisp, non-overlapping corner arrowhead
    const float c_hl      = 9.0f;  // Corner arrowhead length
    const float c_tip_gap = 14.0f; // Gap between corner arrow tip and pocket wall
    const float c_axis    = 33.0f; // Coordinate offset along wall edge for arrow axis
    const float c_tail    = 31.0f; // Coordinate offset for arrow tail start (leaves clear gap)

    // Main arrow tip positions
    const float in_top_tip_y   = py0 + main_gap;
    const float in_bot_tip_y   = py1 - main_gap;
    const float in_left_tip_x  = px0 + main_gap;
    const float in_right_tip_x = px1 - main_gap;

    // Corner arrow vertex coordinates (for diagonal guidelines through corner dots)
    const ImVec2 v_tl(px0 + c_axis, py0 + c_axis);
    const ImVec2 v_tr(px1 - c_axis, py0 + c_axis);
    const ImVec2 v_bl(px0 + c_axis, py1 - c_axis);
    const ImVec2 v_br(px1 - c_axis, py1 - c_axis);

    // Corner arrow tail start positions (separated so stems and heads never touch)
    const ImVec2 tl_tail_up(px0 + c_axis, py0 + c_tail);
    const ImVec2 tl_tail_left(px0 + c_tail, py0 + c_axis);

    const ImVec2 tr_tail_up(px1 - c_axis, py0 + c_tail);
    const ImVec2 tr_tail_right(px1 - c_tail, py0 + c_axis);

    const ImVec2 bl_tail_down(px0 + c_axis, py1 - c_tail);
    const ImVec2 bl_tail_left(px0 + c_tail, py1 - c_axis);

    const ImVec2 br_tail_down(px1 - c_axis, py1 - c_tail);
    const ImVec2 br_tail_right(px1 - c_tail, py1 - c_axis);

    // Corner arrow tips (pointing outward toward pocket walls)
    const ImVec2 tl_tip_up(px0 + c_axis, py0 + c_tip_gap);
    const ImVec2 tl_tip_left(px0 + c_tip_gap, py0 + c_axis);

    const ImVec2 tr_tip_up(px1 - c_axis, py0 + c_tip_gap);
    const ImVec2 tr_tip_right(px1 - c_tip_gap, py0 + c_axis);

    const ImVec2 bl_tip_down(px0 + c_axis, py1 - c_tip_gap);
    const ImVec2 bl_tip_left(px0 + c_tip_gap, py1 - c_axis);

    const ImVec2 br_tip_down(px1 - c_axis, py1 - c_tip_gap);
    const ImVec2 br_tip_right(px1 - c_tip_gap, py1 - c_axis);

    // -------------------------------------------------------------------------
    // PURPLE DASHED LINES (drawn behind arrows and dots as in reference image):
    // 1. Center arrows dashed ticks leading to wall dots:
    drawDashedLine(dl, ImVec2(cx, in_top_tip_y), ImVec2(cx, py0), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, ImVec2(cx, in_bot_tip_y), ImVec2(cx, py1), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, ImVec2(in_left_tip_x, cy), ImVec2(px0, cy), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, ImVec2(in_right_tip_x, cy), ImVec2(px1, cy), col_dash_line, 1.3f, 3.5f, 2.5f);

    // 2. Corner arrow purple dashed guidelines:
    // Vertical dashed lines project through vertical arrows to top/bottom walls
    // Horizontal dashed lines project through horizontal arrows to left/right walls
    // Diagonal dashed lines radiate from vertex V through the clear gap between arrows straight to corner dots
    // Top-Left:
    drawDashedLine(dl, ImVec2(v_tl.x, v_tl.y + 5.0f), ImVec2(v_tl.x, py0), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, ImVec2(v_tl.x + 5.0f, v_tl.y), ImVec2(px0, v_tl.y), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, v_tl, ImVec2(px0, py0), col_dash_line, 1.4f, 4.0f, 3.0f);

    // Top-Right:
    drawDashedLine(dl, ImVec2(v_tr.x, v_tr.y + 5.0f), ImVec2(v_tr.x, py0), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, ImVec2(v_tr.x - 5.0f, v_tr.y), ImVec2(px1, v_tr.y), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, v_tr, ImVec2(px1, py0), col_dash_line, 1.4f, 4.0f, 3.0f);

    // Bottom-Left:
    drawDashedLine(dl, ImVec2(v_bl.x, v_bl.y - 5.0f), ImVec2(v_bl.x, py1), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, ImVec2(v_bl.x + 5.0f, v_bl.y), ImVec2(px0, v_bl.y), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, v_bl, ImVec2(px0, py1), col_dash_line, 1.4f, 4.0f, 3.0f);

    // Bottom-Right:
    drawDashedLine(dl, ImVec2(v_br.x, v_br.y - 5.0f), ImVec2(v_br.x, py1), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, ImVec2(v_br.x - 5.0f, v_br.y), ImVec2(px1, v_br.y), col_dash_line, 1.3f, 3.5f, 2.5f);
    drawDashedLine(dl, v_br, ImVec2(px1, py1), col_dash_line, 1.4f, 4.0f, 3.0f);

    // -------------------------------------------------------------------------
    // CHUNKY WHITE ARROWS (short & compact matching original reference)
    // 4 Main Wall Arrows (pointing OUTWARD from center):
    drawChunkyArrow(dl, ImVec2(cx, in_top_tip_y + main_len), ImVec2(cx, in_top_tip_y), main_sw, main_hw, main_hl);
    drawChunkyArrow(dl, ImVec2(cx, in_bot_tip_y - main_len), ImVec2(cx, in_bot_tip_y), main_sw, main_hw, main_hl);
    drawChunkyArrow(dl, ImVec2(in_left_tip_x + main_len, cy), ImVec2(in_left_tip_x, cy), main_sw, main_hw, main_hl);
    drawChunkyArrow(dl, ImVec2(in_right_tip_x - main_len, cy), ImVec2(in_right_tip_x, cy), main_sw, main_hw, main_hl);

    // 4 Corner Arrow Pairs (pointing OUTWARD towards adjacent walls, non-overlapping):
    // Top-Left:
    drawChunkyArrow(dl, tl_tail_up, tl_tip_up, c_sw, c_hw, c_hl);
    drawChunkyArrow(dl, tl_tail_left, tl_tip_left, c_sw, c_hw, c_hl);

    // Top-Right:
    drawChunkyArrow(dl, tr_tail_up, tr_tip_up, c_sw, c_hw, c_hl);
    drawChunkyArrow(dl, tr_tail_right, tr_tip_right, c_sw, c_hw, c_hl);

    // Bottom-Left:
    drawChunkyArrow(dl, bl_tail_down, bl_tip_down, c_sw, c_hw, c_hl);
    drawChunkyArrow(dl, bl_tail_left, bl_tip_left, c_sw, c_hw, c_hl);

    // Bottom-Right:
    drawChunkyArrow(dl, br_tail_down, br_tip_down, c_sw, c_hw, c_hl);
    drawChunkyArrow(dl, br_tail_right, br_tip_right, c_sw, c_hw, c_hl);

    // -------------------------------------------------------------------------
    // 8 ORANGE DOTS ON THE WORKPIECE EDGES / POCKET BOUNDARIES
    // Top Row:
    renderHotspot("##InTL", ImVec2(px0, py0), dot_r, "InTL",
                  "Inside Top-Left Corner\nProbes inside walls toward X- and Y+ sequentially to set Work X0 Y0",
                  "scripts/mill/probing/inside_corner_top_left.lua", controller);

    renderHotspot("##InTop", ImVec2(cx, py0), dot_r, "InT",
                  "Inside Top Wall (Y+)\nProbes toward Y+ wall to set Work Y0",
                  "scripts/mill/probing/inside_edge_top.lua", controller);

    renderHotspot("##InTR", ImVec2(px1, py0), dot_r, "InTR",
                  "Inside Top-Right Corner\nProbes inside walls toward X+ and Y+ sequentially to set Work X0 Y0",
                  "scripts/mill/probing/inside_corner_top_right.lua", controller);

    // Middle Row:
    renderHotspot("##InLeft", ImVec2(px0, cy), dot_r, "InL",
                  "Inside Left Wall (X-)\nProbes toward X- wall to set Work X0",
                  "scripts/mill/probing/inside_edge_left.lua", controller);

    renderHotspot("##InRight", ImVec2(px1, cy), dot_r, "InR",
                  "Inside Right Wall (X+)\nProbes toward X- wall to set Work X0",
                  "scripts/mill/probing/inside_edge_right.lua", controller);

    // Bottom Row:
    renderHotspot("##InBL", ImVec2(px0, py1), dot_r, "InBL",
                  "Inside Bottom-Left Corner\nProbes inside walls toward X- and Y- sequentially to set Work X0 Y0",
                  "scripts/mill/probing/inside_corner_bottom_left.lua", controller);

    renderHotspot("##InBottom", ImVec2(cx, py1), dot_r, "InB",
                  "Inside Bottom Wall (Y-)\nProbes toward Y- wall to set Work Y0",
                  "scripts/mill/probing/inside_edge_bottom.lua", controller);

    renderHotspot("##InBR", ImVec2(px1, py1), dot_r, "InBR",
                  "Inside Bottom-Right Corner\nProbes inside walls toward X+ and Y- sequentially to set Work X0 Y0",
                  "scripts/mill/probing/inside_corner_bottom_right.lua", controller);

    // Accurately reset layout cursor to end of card plus tight spacing
    ImGui::SetCursorScreenPos(ImVec2(card_pos.x, card_pos.y + card_h + 8.0f));
}

void ProbePanel::renderCenterDeck(core::IMotionController& controller, float avail_w) {
    const float spacing = 8.0f;
    const float tile_w  = (avail_w - spacing) * 0.5f;
    const float tile_h  = 108.0f;
    const float dot_r   = 8.5f;

    ImVec2 origin = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    auto renderTileFrame = [&](ImVec2 pos, ImVec2 sz) {
        dl->AddRectFilled(pos, ImVec2(pos.x + sz.x, pos.y + sz.y), col_card_bg, 6.0f);
        dl->AddRect(pos, ImVec2(pos.x + sz.x, pos.y + sz.y), col_card_border, 6.0f);
    };

    // -------------------------------------------------------------------------
    // ROW 1: Horizontal Outside Web (Y) & Vertical Outside Web (X)
    // -------------------------------------------------------------------------
    ImVec2 p0 = origin;
    renderTileFrame(p0, ImVec2(tile_w, tile_h));
    {
        // Tile 1: Outside Horizontal Web (Y Center)
        float cx = p0.x + tile_w * 0.5f;
        float cy = p0.y + tile_h * 0.5f;
        const float w_w = std::min(tile_w - 46.0f, 78.0f);
        const float w_h = 32.0f;

        // Metallic horizontal web workpiece
        drawMetalTile(dl, ImVec2(cx - w_w * 0.5f, cy - w_h * 0.5f), ImVec2(cx + w_w * 0.5f, cy + w_h * 0.5f), 4.0f);

        // Rounded cyan/blue outer rectangle
        const float b_w = w_w + 18.0f;
        const float b_h = w_h + 22.0f;
        const ImVec2 b_min(cx - b_w * 0.5f, cy - b_h * 0.5f);
        const ImVec2 b_max(cx + b_w * 0.5f, cy + b_h * 0.5f);
        dl->AddRect(b_min, b_max, col_blue_border, 12.0f, 0, 1.4f);

        // Purple dashed midline rotated 90 deg -> runs VERTICALLY through center
        drawDashedLine(dl, ImVec2(cx, b_min.y - 1.0f), ImVec2(cx, b_max.y + 1.0f), col_dash_line, 1.2f, 3.5f, 2.5f);

        // Inward pointing arrows from top and bottom pointing directly to the edge of the blue rectangle
        drawChunkyArrow(dl, ImVec2(cx, b_min.y - 18.0f), ImVec2(cx, b_min.y), 6.5f, 15.0f, 10.0f);
        drawChunkyArrow(dl, ImVec2(cx, b_max.y + 18.0f), ImVec2(cx, b_max.y), 6.5f, 15.0f, 10.0f);

        renderHotspot("##WebY", ImVec2(cx, cy), dot_r, "WebY",
                      "Outside Horizontal Web (Y Center)\nProbes top & bottom outside edges to find and zero Y midpoint",
                      "scripts/mill/probing/center_web_y.lua", controller);
    }

    ImVec2 p1(p0.x + tile_w + spacing, p0.y);
    renderTileFrame(p1, ImVec2(tile_w, tile_h));
    {
        // Tile 2: Outside Vertical Web (X Center) - exact 90 deg rotation of horizontal web
        float cx = p1.x + tile_w * 0.5f;
        float cy = p1.y + tile_h * 0.5f;
        const float w_w = 32.0f;
        const float w_h = std::min(tile_h - 30.0f, 78.0f);

        // Metallic vertical web workpiece
        drawMetalTile(dl, ImVec2(cx - w_w * 0.5f, cy - w_h * 0.5f), ImVec2(cx + w_w * 0.5f, cy + w_h * 0.5f), 4.0f);

        // Rounded cyan/blue outer rectangle
        const float b_w = w_w + 22.0f;
        const float b_h = w_h + 18.0f;
        const ImVec2 b_min(cx - b_w * 0.5f, cy - b_h * 0.5f);
        const ImVec2 b_max(cx + b_w * 0.5f, cy + b_h * 0.5f);
        dl->AddRect(b_min, b_max, col_blue_border, 12.0f, 0, 1.4f);

        // Purple dashed midline rotated 90 deg -> runs HORIZONTALLY through center
        drawDashedLine(dl, ImVec2(b_min.x - 1.0f, cy), ImVec2(b_max.x + 1.0f, cy), col_dash_line, 1.2f, 3.5f, 2.5f);

        // Inward pointing arrows from left and right pointing directly to the edge of the blue rectangle
        drawChunkyArrow(dl, ImVec2(b_min.x - 18.0f, cy), ImVec2(b_min.x, cy), 6.5f, 15.0f, 10.0f);
        drawChunkyArrow(dl, ImVec2(b_max.x + 18.0f, cy), ImVec2(b_max.x, cy), 6.5f, 15.0f, 10.0f);

        renderHotspot("##WebX", ImVec2(cx, cy), dot_r, "WebX",
                      "Outside Vertical Web (X Center)\nProbes left & right outside edges to find and zero X midpoint",
                      "scripts/mill/probing/center_web_x.lua", controller);
    }

    // -------------------------------------------------------------------------
    // ROW 2: Inside Horizontal Slot (Y) & Inside Vertical Slot (X)
    // -------------------------------------------------------------------------
    p0 = ImVec2(origin.x, origin.y + tile_h + spacing);
    renderTileFrame(p0, ImVec2(tile_w, tile_h));
    {
        // Tile 3: Inside Horizontal Slot (Y Center) - taller to fill available space, blue rectangle
        float cx = p0.x + tile_w * 0.5f;
        float cy = p0.y + tile_h * 0.5f;
        const float s_w = std::min(tile_w - 40.0f, 86.0f);
        const float slot_h = 54.0f;
        const float bar_h  = 18.0f;

        // Two metallic boundary bars (top & bottom walls)
        drawMetalTile(dl, ImVec2(cx - s_w * 0.5f, cy - slot_h * 0.5f - bar_h), ImVec2(cx + s_w * 0.5f, cy - slot_h * 0.5f), 3.0f);
        drawMetalTile(dl, ImVec2(cx - s_w * 0.5f, cy + slot_h * 0.5f), ImVec2(cx + s_w * 0.5f, cy + slot_h * 0.5f + bar_h), 3.0f);

        // Dark slot pocket channel with blue border
        const ImVec2 b_min(cx - s_w * 0.5f, cy - slot_h * 0.5f);
        const ImVec2 b_max(cx + s_w * 0.5f, cy + slot_h * 0.5f);
        dl->AddRectFilled(b_min, b_max, col_pocket_bg, 8.0f);
        dl->AddRect(b_min, b_max, col_blue_border, 8.0f, 0, 1.4f);

        // Vertical purple dashed line through center
        drawDashedLine(dl, ImVec2(cx, cy - slot_h * 0.5f), ImVec2(cx, cy + slot_h * 0.5f), col_dash_line, 1.2f, 3.5f, 2.5f);

        // Outward pointing arrows from center dot toward top & bottom walls
        drawChunkyArrow(dl, ImVec2(cx, cy - dot_r - 1.0f), ImVec2(cx, cy - slot_h * 0.5f + 2.0f), 5.5f, 13.0f, 9.0f);
        drawChunkyArrow(dl, ImVec2(cx, cy + dot_r + 1.0f), ImVec2(cx, cy + slot_h * 0.5f - 2.0f), 5.5f, 13.0f, 9.0f);

        renderHotspot("##SlotY", ImVec2(cx, cy), dot_r, "SlotY",
                      "Inside Horizontal Slot (Y Center)\nProbes inside top & bottom walls to zero Y center",
                      "scripts/mill/probing/center_slot_y.lua", controller);
    }

    p1 = ImVec2(p0.x + tile_w + spacing, p0.y);
    renderTileFrame(p1, ImVec2(tile_w, tile_h));
    {
        // Tile 4: Inside Vertical Slot (X Center) - exact 90 deg rotation of horizontal slot
        float cx = p1.x + tile_w * 0.5f;
        float cy = p1.y + tile_h * 0.5f;
        const float slot_w = 54.0f;
        const float s_h    = std::min(tile_h - 22.0f, 86.0f);
        const float bar_w  = 18.0f;

        // Two metallic boundary bars (left & right walls)
        drawMetalTile(dl, ImVec2(cx - slot_w * 0.5f - bar_w, cy - s_h * 0.5f), ImVec2(cx - slot_w * 0.5f, cy + s_h * 0.5f), 3.0f);
        drawMetalTile(dl, ImVec2(cx + slot_w * 0.5f, cy - s_h * 0.5f), ImVec2(cx + slot_w * 0.5f + bar_w, cy + s_h * 0.5f), 3.0f);

        // Dark slot pocket channel with blue border
        const ImVec2 b_min(cx - slot_w * 0.5f, cy - s_h * 0.5f);
        const ImVec2 b_max(cx + slot_w * 0.5f, cy + s_h * 0.5f);
        dl->AddRectFilled(b_min, b_max, col_pocket_bg, 8.0f);
        dl->AddRect(b_min, b_max, col_blue_border, 8.0f, 0, 1.4f);

        // Horizontal purple dashed line through center
        drawDashedLine(dl, ImVec2(cx - slot_w * 0.5f, cy), ImVec2(cx + slot_w * 0.5f, cy), col_dash_line, 1.2f, 3.5f, 2.5f);

        // Outward pointing arrows from center dot toward left & right walls
        drawChunkyArrow(dl, ImVec2(cx - dot_r - 1.0f, cy), ImVec2(cx - slot_w * 0.5f + 2.0f, cy), 5.5f, 13.0f, 9.0f);
        drawChunkyArrow(dl, ImVec2(cx + dot_r + 1.0f, cy), ImVec2(cx + slot_w * 0.5f - 2.0f, cy), 5.5f, 13.0f, 9.0f);

        renderHotspot("##SlotX", ImVec2(cx, cy), dot_r, "SlotX",
                      "Inside Vertical Slot (X Center)\nProbes inside left & right walls to zero X center",
                      "scripts/mill/probing/center_slot_x.lua", controller);
    }

    // -------------------------------------------------------------------------
    // ROW 3: Outside Circular Boss Center & Inside Circular Bore Center (Arrows Switched)
    // -------------------------------------------------------------------------
    p0 = ImVec2(origin.x, origin.y + (tile_h + spacing) * 2.0f);
    renderTileFrame(p0, ImVec2(tile_w, tile_h));
    {
        // Tile 5: Outside Circular Boss Center (Row 3 Left) - raised metallic disc, outer blue circle, inward arrows
        float cx = p0.x + tile_w * 0.5f;
        float cy = p0.y + tile_h * 0.5f;
        const float r_boss = 28.0f;
        const float r_blue = 38.0f;

        // Metallic cylinder boss in center
        dl->AddCircleFilled(ImVec2(cx, cy), r_boss, col_metal_top, 40);
        dl->AddCircle(ImVec2(cx, cy), r_boss, col_metal_bevel, 40, 1.5f);
        dl->AddCircle(ImVec2(cx, cy), r_boss, col_metal_border, 40, 1.0f);

        // Outer concentric blue circle
        dl->AddCircle(ImVec2(cx, cy), r_blue, col_blue_border, 48, 1.4f);

        // Purple dashed crosshairs across blue circle
        drawDashedLine(dl, ImVec2(cx - r_blue - 2.0f, cy), ImVec2(cx + r_blue + 2.0f, cy), col_dash_line, 1.2f, 3.5f, 2.5f);
        drawDashedLine(dl, ImVec2(cx, cy - r_blue - 2.0f), ImVec2(cx, cy + r_blue + 2.0f), col_dash_line, 1.2f, 3.5f, 2.5f);

        // 4 Inward pointing chunky arrows from outside toward blue circle
        drawChunkyArrow(dl, ImVec2(cx - r_blue - 18.0f, cy), ImVec2(cx - r_blue, cy), 6.0f, 14.0f, 9.0f);
        drawChunkyArrow(dl, ImVec2(cx + r_blue + 18.0f, cy), ImVec2(cx + r_blue, cy), 6.0f, 14.0f, 9.0f);
        drawChunkyArrow(dl, ImVec2(cx, cy - r_blue - 18.0f), ImVec2(cx, cy - r_blue), 6.0f, 14.0f, 9.0f);
        drawChunkyArrow(dl, ImVec2(cx, cy + r_blue + 18.0f), ImVec2(cx, cy + r_blue), 6.0f, 14.0f, 9.0f);

        renderHotspot("##BossXY", ImVec2(cx, cy), dot_r, "BossXY",
                      "Outside Boss / Cylinder Center\nFinds and zeroes X0 Y0 center",
                      "scripts/mill/probing/center_boss_xy.lua", controller);
    }

    p1 = ImVec2(p0.x + tile_w + spacing, p0.y);
    renderTileFrame(p1, ImVec2(tile_w, tile_h));
    {
        // Tile 6: Inside Circular Bore Center (Row 3 Right) - outer metallic donut, inner hole with blue circle, outward arrows
        float cx = p1.x + tile_w * 0.5f;
        float cy = p1.y + tile_h * 0.5f;
        const float r_outer = 43.0f;
        const float r_inner = 29.0f;
        const float r_blue  = 27.0f;

        // Outer metallic donut ring
        dl->AddCircleFilled(ImVec2(cx, cy), r_outer, col_metal_bot, 48);
        dl->AddCircle(ImVec2(cx, cy), r_outer, col_metal_bevel, 48, 1.5f);

        // Recessed circular bore pocket
        dl->AddCircleFilled(ImVec2(cx, cy), r_inner, col_pocket_bg, 40);
        dl->AddCircle(ImVec2(cx, cy), r_inner, col_pocket_shadow, 40, 1.8f);

        // Concentric blue circle inside bore
        dl->AddCircle(ImVec2(cx, cy), r_blue, col_blue_border, 40, 1.4f);

        // Purple dashed crosshairs across bore
        drawDashedLine(dl, ImVec2(cx - r_inner + 2.0f, cy), ImVec2(cx + r_inner - 2.0f, cy), col_dash_line, 1.2f, 3.5f, 2.5f);
        drawDashedLine(dl, ImVec2(cx, cy - r_inner + 2.0f), ImVec2(cx, cy + r_inner - 2.0f), col_dash_line, 1.2f, 3.5f, 2.5f);

        // 4 Outward pointing chunky arrows from center dot toward bore walls
        drawChunkyArrow(dl, ImVec2(cx, cy - dot_r - 1.0f), ImVec2(cx, cy - r_blue + 2.0f), 5.5f, 13.0f, 8.5f);
        drawChunkyArrow(dl, ImVec2(cx, cy + dot_r + 1.0f), ImVec2(cx, cy + r_blue - 2.0f), 5.5f, 13.0f, 8.5f);
        drawChunkyArrow(dl, ImVec2(cx - dot_r - 1.0f, cy), ImVec2(cx - r_blue + 2.0f, cy), 5.5f, 13.0f, 8.5f);
        drawChunkyArrow(dl, ImVec2(cx + dot_r + 1.0f, cy), ImVec2(cx + r_blue - 2.0f, cy), 5.5f, 13.0f, 8.5f);

        renderHotspot("##BoreXY", ImVec2(cx, cy), dot_r, "BoreXY",
                      "Inside Circular Bore Center\nProbes 4 points inside hole (-X, +X, -Y, +Y) to find and zero X0 Y0 center",
                      "scripts/mill/probing/center_bore_xy.lua", controller);
    }

    // Accurately reset layout cursor to end of all 3 rows plus tight spacing
    const float total_center_h = 3.0f * tile_h + 2.0f * spacing;
    ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + total_center_h + 8.0f));
}

void ProbePanel::renderProbeZDeck(core::IMotionController& controller, float avail_w) {
    const float card_w = avail_w;
    const float card_h = 340.0f;
    ImVec2 card_pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Card background
    dl->AddRectFilled(card_pos, ImVec2(card_pos.x + card_w, card_pos.y + card_h), col_card_bg, 6.0f);
    dl->AddRect(card_pos, ImVec2(card_pos.x + card_w, card_pos.y + card_h), col_card_border, 6.0f);

    const float cx = card_pos.x + card_w * 0.5f;
    const float dot_r     = 8.5f;
    const float shank_w   = 11.0f; // Thinner probe shaft matching reference

    // Vertically centered visual cluster above the bottom action button
    // Available visual height above button: (card_h - 52.0f) = 288.0f, center is at 144.0f
    const float shank_top = card_pos.y + 68.0f;
    const float dot_y     = card_pos.y + 144.0f;
    const float shank_bot = dot_y; // Connected directly to the probe tip

    // 1. Spindle tool shank (vertical metallic cylinder)
    drawMetalTile(dl, ImVec2(cx - shank_w * 0.5f, shank_top), ImVec2(cx + shank_w * 0.5f, shank_bot), 2.5f);

    // 2. Touch Plate / Tool Setter Block below
    const float block_w   = std::min(card_w - 60.0f, 140.0f);
    const float block_top = card_pos.y + 196.0f;
    const float block_bot = card_pos.y + 228.0f;
    drawMetalTile(dl, ImVec2(cx - block_w * 0.5f, block_top), ImVec2(cx + block_w * 0.5f, block_bot), 4.0f);

    // Label on touch block
    const char* p_txt = "TOOL SETTER";
    ImVec2 txt_sz = ImGui::CalcTextSize(p_txt);
    dl->AddText(ImVec2(cx - txt_sz.x * 0.5f, block_top + (block_bot - block_top - txt_sz.y) * 0.5f), IM_COL32(235, 242, 252, 255), p_txt);

    // Purple dashed probe motion path downward from probe tip to block
    drawDashedLine(dl, ImVec2(cx, dot_y + dot_r + 1.0f), ImVec2(cx, block_top - 1.0f), col_dash_line, 1.3f, 3.5f, 2.5f);

    // 3. Orange Hotspot on Probe Tip (contact position)
    renderHotspot("##ProbeZDot", ImVec2(cx, dot_y), dot_r, "Z",
                  "Probe Z Tool Setter\nLowers tool down to touch plate, zeroes Work Z to plate thickness, and retracts",
                  "scripts/mill/probing/probe_z.lua", controller);

    // Prominent clickable button at the bottom of the card
    ImGui::SetCursorScreenPos(ImVec2(card_pos.x + 20.0f, card_pos.y + card_h - 44.0f));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f, 0.42f, 0.62f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.54f, 0.78f, 1.0f));
    if (ImGui::Button("RUN Z TOOL SETTER PROBE", ImVec2(card_w - 40.0f, 32.0f))) {
        controller.runScriptFile("scripts/mill/probing/probe_z.lua", true);
    }
    ImGui::PopStyleColor(2);

    // Accurately reset layout cursor to end of card plus tight spacing
    ImGui::SetCursorScreenPos(ImVec2(card_pos.x, card_pos.y + card_h + 8.0f));
}

} // namespace rnr::ui
