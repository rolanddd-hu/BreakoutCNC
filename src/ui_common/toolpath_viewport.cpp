#include "ui_common/toolpath_viewport.h"
#include "ui_common/app_window.h"
#include "ui_common/io_status_bar.h"
#include "ui_common/frame_stats.h"
#include "imgui.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <cstdio>
#include <algorithm>

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

namespace rnr::ui {

ToolpathViewport::~ToolpathViewport() {
    if (cam_gl_tex_ != 0) {
        GLuint tex = static_cast<GLuint>(cam_gl_tex_);
        glDeleteTextures(1, &tex);
        cam_gl_tex_ = 0;
    }
}

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// Background opacity of every overlay on the toolpath view (toolbar, readouts, legend, banners): 85 %, so the
// toolpath stays faintly visible behind them
constexpr float kOverlayBgAlpha    = 0.85f;
constexpr int   kOverlayBgAlpha255 = static_cast<int>(kOverlayBgAlpha * 255.0f + 0.5f);

// Clips the line a->b to rectangle r (x0, y0, x1, y1) in parametric form (Liang-Barsky).
// Returns false if no part of the line is inside.
bool clipLineParams(ImVec2 a, ImVec2 b, const ImVec4& r, float& t0, float& t1) {
    t0 = 0.0f;
    t1 = 1.0f;
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float p[4] = {-dx, dx, -dy, dy};
    const float q[4] = {a.x - r.x, r.z - a.x, a.y - r.y, r.w - a.y};
    for (int i = 0; i < 4; ++i) {
        if (std::abs(p[i]) < 1e-9f) {
            if (q[i] < 0.0f) return false;
            continue;
        }
        const float t = q[i] / p[i];
        if (p[i] < 0.0f) {
            t0 = std::max(t0, t);
        } else {
            t1 = std::min(t1, t);
        }
        if (t0 > t1) return false;
    }
    return true;
}

// Only the visible part is dashed (a long rapid seen up close would otherwise produce thousands of dashes);
// the dash pattern stays anchored to point a so dashes do not crawl while panning
void drawDashedLine(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float thickness, float dash_len, const ImVec4& clip) {
    float t0 = 0.0f, t1 = 1.0f;
    if (!clipLineParams(a, b, clip, t0, t1)) return;
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dist = std::hypot(dx, dy);
    if (dist < 1.0f) return;
    float ux = dx / dist;
    float uy = dy / dist;
    const float period = dash_len * 2.0f;
    float pos = std::floor(t0 * dist / period) * period;
    dist *= t1;
    while (pos < dist) {
        float end_p = std::min(pos + dash_len, dist);
        ImVec2 p0(a.x + ux * pos, a.y + uy * pos);
        ImVec2 p1(a.x + ux * end_p, a.y + uy * end_p);
        dl->AddLine(p0, p1, col, thickness);
        pos += period;
    }
}

// Grows a clip rectangle (x0, y0, x1, y1) by the line thickness
ImVec4 cullRect(const ImVec4& clip) {
    constexpr float kMargin = 4.0f;
    return ImVec4(clip.x - kMargin, clip.y - kMargin, clip.z + kMargin, clip.w + kMargin);
}

bool lineOutside(ImVec2 a, ImVec2 b, const ImVec4& r) {
    return (a.x < r.x && b.x < r.x) || (a.x > r.z && b.x > r.z) || (a.y < r.y && b.y < r.y) || (a.y > r.w && b.y > r.w);
}

// Merges toolpath segments into lines. Dense CAM programs have 100k+ segments, most of them tiny steps
// along nearly straight or gently curved paths: a chain of connected segments becomes one line for as long
// as every point of the chain stays within tolerance_px of that line. Each finished line goes to
// sink(a, b, first_segment, last_segment).
//
// The check is constant work per point (sleeve / angular-wedge simplification): each point P at distance d
// from the chain start allows the final line direction to deviate from P's direction by asin(tol / d). The
// wedge of directions allowed by all points so far is kept as two unit vectors; a new point fits if its
// direction lies inside the wedge and it is not closer to the start than the farthest point (no reversal).
template <class Sink>
class ToolpathLineBatch {
public:
    ToolpathLineBatch(Sink sink, float tolerance_px) : sink_(std::move(sink)), tol_(tolerance_px) {}
    ~ToolpathLineBatch() { flush(); }

    void add(ImVec2 p0, ImVec2 p1, uint32_t seg) { add(p0, p1, seg, seg); }

    // A line standing for the segments first..last (a chord over several segments while the view moves)
    void add(ImVec2 p0, ImVec2 p1, uint32_t first, uint32_t last) {
        if (!pending_ || distSq(p0, end_) > kJoinPx * kJoinPx) {
            flush();
            startChain(p0, first);
        }
        if (!extend(p1, last)) {
            emit();
            startChain(end_, first);
            extend(p1, last); // always fits a fresh chain
        }
    }

    void flush() {
        if (pending_ && distSq(start_, end_) > 0.01f) emit();
        pending_ = false;
    }

private:
    static constexpr float kJoinPx = 0.25f; // segment starts where the previous one ended

    static float distSq(ImVec2 a, ImVec2 b) {
        const float dx = b.x - a.x;
        const float dy = b.y - a.y;
        return dx * dx + dy * dy;
    }
    static float cross(ImVec2 a, ImVec2 b) { return a.x * b.y - a.y * b.x; }

    void startChain(ImVec2 p, uint32_t seg) {
        start_      = p;
        end_        = p;
        max_dist_   = 0.0f;
        has_wedge_  = false;
        pending_    = true;
        first_seg_  = seg;
        last_seg_   = seg;
    }

    // Moves the chain end to p if the whole chain still fits one line; false (unchanged) otherwise
    bool extend(ImVec2 p, uint32_t seg) {
        const ImVec2 v(p.x - start_.x, p.y - start_.y);
        const float d = std::sqrt(v.x * v.x + v.y * v.y);
        if (d + 0.25f * tol_ < max_dist_) return false; // path turned back towards the start
        if (d > tol_) {
            const ImVec2 u(v.x / d, v.y / d);
            const float s = tol_ / d;                // sin of the allowed deviation
            const float c = std::sqrt(1.0f - s * s); // cos of the allowed deviation
            const ImVec2 lo(u.x * c + u.y * s, u.y * c - u.x * s); // u rotated clockwise
            const ImVec2 hi(u.x * c - u.y * s, u.y * c + u.x * s); // u rotated counter-clockwise
            if (!has_wedge_) {
                wedge_lo_  = lo;
                wedge_hi_  = hi;
                has_wedge_ = true;
            } else {
                if (cross(wedge_lo_, u) < 0.0f || cross(u, wedge_hi_) < 0.0f) return false;
                if (cross(wedge_lo_, lo) > 0.0f) wedge_lo_ = lo; // keep the tighter bound on each side
                if (cross(hi, wedge_hi_) > 0.0f) wedge_hi_ = hi;
            }
        }
        // Points within tol of the start constrain nothing
        end_      = p;
        max_dist_ = std::max(max_dist_, d);
        last_seg_ = seg;
        return true;
    }

    void emit() { sink_(start_, end_, first_seg_, last_seg_); }

    Sink sink_;
    float tol_;
    bool pending_   = false;
    bool has_wedge_ = false;
    float max_dist_ = 0.0f;
    ImVec2 start_{}, end_{};
    ImVec2 wedge_lo_{}, wedge_hi_{}; // allowed directions of the merged line (unit vectors)
    uint32_t first_seg_ = 0, last_seg_ = 0;
};

// Draws merged lines of one color into a draw list, skipping lines outside the clip rectangle
struct DrawLineSink {
    ImDrawList* dl;
    ImVec4 cull;
    ImU32 col;
    void operator()(ImVec2 a, ImVec2 b, uint32_t, uint32_t) const {
        if (!lineOutside(a, b, cull)) dl->AddLine(a, b, col, 1.0f);
    }
};

bool isCutSegment(const core::ToolpathSegment& seg) {
    return seg.type == core::SegmentType::CutLinear || seg.type == core::SegmentType::CutArcCW ||
           seg.type == core::SegmentType::CutArcCCW;
}

void drawHudTextbox(
    ImDrawList* dl,
    ImFont* font,
    float font_size,
    ImVec2 box_p0,
    float box_w,
    float box_h,
    const char* label,
    const char* value,
    ImU32 val_color = IM_COL32(225, 238, 252, 255))
{
    ImVec2 box_p1(box_p0.x + box_w, box_p0.y + box_h);
    dl->AddRectFilled(box_p0, box_p1, IM_COL32(10, 13, 18, kOverlayBgAlpha255), 4.0f);
    dl->AddRect(box_p0, box_p1, IM_COL32(46, 62, 82, 215), 4.0f, 0, 1.0f);

    float text_y = box_p0.y + (box_h - font_size) * 0.5f - 1.0f;
    dl->AddText(font, font_size, ImVec2(box_p0.x + 8.0f, text_y), IM_COL32(130, 152, 178, 245), label);

    ImVec2 val_sz = font ? font->CalcTextSizeA(font_size, 1000.0f, 0.0f, value)
                         : ImGui::CalcTextSize(value);
    float val_x = std::max(box_p0.x + 8.0f, box_p1.x - val_sz.x - 8.0f);
    dl->AddText(font, font_size, ImVec2(val_x, text_y), val_color, value);
}
} // namespace

void ToolpathViewport::fitToBounds(const std::array<double, 3>& bbox_min_mm, const std::array<double, 3>& bbox_max_mm) {
    double cx = 0.5 * (bbox_min_mm[0] + bbox_max_mm[0]);
    double cy = 0.5 * (bbox_min_mm[1] + bbox_max_mm[1]);
    pan_x_mm_ = static_cast<float>(cx);
    pan_y_mm_ = static_cast<float>(cy);
    initial_fit_done_ = false; // Will compute exact pixel scale on next frame
}

// The view projection without pan and canvas center: projectXYZ(p) == center + projectNoPan(p) - projectNoPan(pan)
ImVec2 ToolpathViewport::projectNoPan(double x_mm, double y_mm, double z_mm) const {
    if (!isometric_3d_) {
        return ImVec2(static_cast<float>(x_mm) * zoom_px_per_mm_, -static_cast<float>(y_mm) * zoom_px_per_mm_);
    }
    const double yaw_rad   = static_cast<double>(view_yaw_deg_) * kDegToRad;
    const double pitch_rad = static_cast<double>(view_pitch_deg_) * kDegToRad;
    const double rx = x_mm * std::cos(yaw_rad) - y_mm * std::sin(yaw_rad);
    const double ry = x_mm * std::sin(yaw_rad) + y_mm * std::cos(yaw_rad);
    const double sy = ry * std::cos(pitch_rad) + z_mm * std::sin(pitch_rad);
    return ImVec2(static_cast<float>(rx * zoom_px_per_mm_), -static_cast<float>(sy * zoom_px_per_mm_));
}

void ToolpathViewport::updatePathCache(const std::vector<core::ToolpathSegment>& segments) {
    // A cheap fingerprint guards against a new program allocated at the same address with the same size
    double fingerprint = 0.0;
    if (!segments.empty()) {
        for (size_t i : {size_t{0}, segments.size() / 2, segments.size() - 1}) {
            const auto& s = segments[i];
            fingerprint += s.end_xyza_mm[0] * 1.3 + s.end_xyza_mm[1] * 1.7 + s.end_xyza_mm[2] * 1.9 +
                           static_cast<double>(s.gcode_line) + static_cast<double>(s.type) * 0.1;
        }
    }
    // Rotating / zooming a very large program rebuilds the cache every frame: build it from chords over a few
    // segments until the view has been still for kInteractHoldS, then once more at full detail
    const bool coarse = segments.size() >= kCoarseMinSegments && ImGui::GetTime() < interact_until_s_;
    auto& c = path_cache_;
    const bool same_program = c.geo_valid && c.seg_data == segments.data() && c.seg_count == segments.size() &&
                              c.fingerprint == fingerprint;
    if (same_program && c.valid && c.iso == isometric_3d_ && c.yaw == view_yaw_deg_ && c.pitch == view_pitch_deg_ &&
        c.zoom == zoom_px_per_mm_ && c.tol == simplify_px_ && c.tool_filter == selected_tool_filter_ && c.coarse == coarse) {
        return;
    }
    if (!same_program) {
        c.geo.resize(segments.size());
        for (size_t i = 0; i < segments.size(); ++i) {
            const auto& s = segments[i];
            auto& g = c.geo[i];
            for (int a = 0; a < 3; ++a) {
                g.start[a] = static_cast<float>(s.start_xyza_mm[a]);
                g.end[a]   = static_cast<float>(s.end_xyza_mm[a]);
            }
            g.tool = s.tool_number;
            g.type = static_cast<uint8_t>(s.type);
        }
        c.geo_valid   = true;
        c.seg_data    = segments.data();
        c.seg_count   = segments.size();
        c.fingerprint = fingerprint;
    }
    c.valid       = true;
    c.coarse      = coarse;
    c.iso         = isometric_3d_;
    c.yaw         = view_yaw_deg_;
    c.pitch       = view_pitch_deg_;
    c.zoom        = zoom_px_per_mm_;
    c.tol         = simplify_px_;
    c.tool_filter = selected_tool_filter_;
    c.cuts.clear();
    c.rapids.clear();
    c.pierces.clear();
    c.tools_in_file.clear();

    // Same math as projectNoPan() with the trigonometry hoisted out of the loop
    const bool   iso  = isometric_3d_;
    const float  zoom = zoom_px_per_mm_;
    const double cy   = std::cos(static_cast<double>(view_yaw_deg_) * kDegToRad);
    const double sy   = std::sin(static_cast<double>(view_yaw_deg_) * kDegToRad);
    const double cp   = std::cos(static_cast<double>(view_pitch_deg_) * kDegToRad);
    const double sp   = std::sin(static_cast<double>(view_pitch_deg_) * kDegToRad);
    auto project = [&](const float* p) -> ImVec2 {
        if (!iso) return ImVec2(p[0] * zoom, -p[1] * zoom);
        const double rx = p[0] * cy - p[1] * sy;
        const double ry = p[0] * sy + p[1] * cy;
        return ImVec2(static_cast<float>(rx * zoom), -static_cast<float>((ry * cp + p[2] * sp) * zoom));
    };
    constexpr auto kRapid = static_cast<uint8_t>(core::SegmentType::RapidMove);
    constexpr auto kDwell = static_cast<uint8_t>(core::SegmentType::Dwell);
    auto isCut = [](uint8_t t) {
        return t == static_cast<uint8_t>(core::SegmentType::CutLinear) ||
               t == static_cast<uint8_t>(core::SegmentType::CutArcCW) ||
               t == static_cast<uint8_t>(core::SegmentType::CutArcCCW);
    };

    uint16_t batch_tool = 0;
    auto sink = [&c, &batch_tool](ImVec2 a, ImVec2 b, uint32_t first, uint32_t last) {
        c.cuts.push_back(CachedCutLine{a, b, first, last, batch_tool});
    };
    ToolpathLineBatch<decltype(sink)> batch(sink, simplify_px_);
    const size_t stride = coarse ? std::max<size_t>(2, segments.size() / kCoarseTargetSegments) : 1;
    size_t chord_first = 0, chord_len = 0;
    ImVec2 chord_p0{};
    uint16_t last_tool_seen = 0;
    const auto& geo = c.geo;
    for (size_t i = 0; i < geo.size(); ++i) {
        const auto& seg = geo[i];
        if (seg.tool > 0 && seg.tool != last_tool_seen) {
            last_tool_seen = seg.tool;
            if (std::find(c.tools_in_file.begin(), c.tools_in_file.end(), seg.tool) == c.tools_in_file.end()) {
                c.tools_in_file.push_back(seg.tool);
            }
        }
        if (selected_tool_filter_ != 0 && seg.tool != selected_tool_filter_) continue;
        if (seg.type == kRapid) {
            c.rapids.push_back(project(seg.start));
            c.rapids.push_back(project(seg.end));
        } else if (seg.type == kDwell) {
            c.pierces.push_back(CachedPoint{project(seg.start), static_cast<uint32_t>(i)});
        } else if (isCut(seg.type)) {
            if (seg.tool != batch_tool) {
                batch.flush(); // one tool (one color) per merged line
                batch_tool = seg.tool;
            }
            if (stride == 1) {
                batch.add(project(seg.start), project(seg.end), static_cast<uint32_t>(i));
                continue;
            }
            // Chord over up to `stride` consecutive cut segments of one tool; interior points are not projected
            if (chord_len == 0) {
                chord_first = i;
                chord_p0    = project(seg.start);
            }
            ++chord_len;
            const bool run_continues = (i + 1 < geo.size()) && isCut(geo[i + 1].type) && geo[i + 1].tool == seg.tool;
            if (chord_len >= stride || !run_continues) {
                batch.add(chord_p0, project(seg.end), static_cast<uint32_t>(chord_first), static_cast<uint32_t>(i));
                chord_len = 0;
            }
        }
    }
    batch.flush();
}

void ToolpathViewport::render(
    const std::vector<core::ToolpathSegment>& segments,
    const core::TelemetrySnapshot& telemetry,
    const std::array<double, 3>& bbox_min_mm,
    const std::array<double, 3>& bbox_max_mm,
    core::IMotionController* controller)
{
    perf::ScopedTimer perf_timer("Toolpath view");
    ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
    ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
    if (canvas_sz.x < 60.0f) canvas_sz.x = 60.0f;
    if (canvas_sz.y < 60.0f) canvas_sz.y = 60.0f;
    ImVec2 canvas_p1(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

    const ImVec2 tb_p0(canvas_p0.x + 8.0f, canvas_p0.y + 8.0f);
    const float  tb_w  = std::max(100.0f, canvas_sz.x - 16.0f);
    const ImVec2 tb_frame_padding(7.0f, 2.5f);
    const float  tb_comp_h = std::round(ImGui::GetFontSize() + tb_frame_padding.y * 2.0f);
    const float  tb_h      = tb_comp_h + 6.0f; // 1px border + 2px top clearance + item + 2px bottom clearance + 1px border
    const ImVec2 tb_p1(tb_p0.x + tb_w, tb_p0.y + tb_h);

    // Check active Top-Down Camera state (BreakoutMill)
    const bool has_top_down_cam = (camera_mgr_ != nullptr && camera_mgr_->isRoleActive(core::CameraRole::TopDown));
    const core::CameraConfig cam_cfg = controller ? controller->getConfig().camera : core::CameraConfig{};

    if (!has_top_down_cam) {
        camera_calibrating_     = false;
        waiting_for_dent_click_ = false;
        camera_overlay_enabled_ = false;
    } else if (isometric_3d_) {
        camera_overlay_enabled_ = false;
        camera_calibrating_     = false;
        waiting_for_dent_click_ = false;
    } else if (camera_overlay_enabled_ && !cam_cfg.is_calibrated) {
        camera_overlay_enabled_ = false;
    }

    const bool in_cam_calibration = has_top_down_cam && camera_calibrating_ && !isometric_3d_;
    const bool in_cam_overlay     = has_top_down_cam && !in_cam_calibration && camera_overlay_enabled_ &&
                                    cam_cfg.is_calibrated && !isometric_3d_;

    // Upload latest camera frame to OpenGL texture when calibration or overlay is active
    if (in_cam_calibration || in_cam_overlay) {
        core::CameraFrame frame;
        if (camera_mgr_->getLatestFrame(core::CameraRole::TopDown, frame, cam_last_frame_seq_)) {
            if (cam_gl_tex_ == 0) {
                GLuint tex = 0;
                glGenTextures(1, &tex);
                cam_gl_tex_ = static_cast<uint32_t>(tex);
            }
            glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(cam_gl_tex_));
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            if (cam_tex_w_ != frame.width || cam_tex_h_ != frame.height) {
                cam_tex_w_ = frame.width;
                cam_tex_h_ = frame.height;
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, cam_tex_w_, cam_tex_h_, 0,
                             GL_RGBA, GL_UNSIGNED_BYTE, frame.rgba.data());
            } else {
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, cam_tex_w_, cam_tex_h_,
                                GL_RGBA, GL_UNSIGNED_BYTE, frame.rgba.data());
            }
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }

    if (in_cam_overlay) {
        // Lock pan and zoom strictly to the calibrated camera's live position and Field of View
        double cam_world_x = telemetry.work_pos_mm[0] - cam_cfg.offset_x_mm;
        double cam_world_y = telemetry.work_pos_mm[1] - cam_cfg.offset_y_mm;
        pan_x_mm_ = static_cast<float>(cam_world_x);
        pan_y_mm_ = static_cast<float>(cam_world_y);
        float zx  = canvas_sz.x / static_cast<float>(std::max(1.0, cam_cfg.fov_x_mm));
        float zy  = canvas_sz.y / static_cast<float>(std::max(1.0, cam_cfg.fov_y_mm));
        zoom_px_per_mm_   = std::clamp(std::min(zx, zy), 0.1f, 200.0f);
        initial_fit_done_ = true;
    } else if (!initial_fit_done_) {
        double span_x = std::max(20.0, (bbox_max_mm[0] - bbox_min_mm[0]) + 25.0);
        double span_y = std::max(20.0, (bbox_max_mm[1] - bbox_min_mm[1]) + 25.0);
        pan_x_mm_ = static_cast<float>(0.5 * (bbox_min_mm[0] + bbox_max_mm[0]));
        pan_y_mm_ = static_cast<float>(0.5 * (bbox_min_mm[1] + bbox_max_mm[1]));
        float zx = canvas_sz.x / static_cast<float>(span_x);
        float zy = canvas_sz.y / static_cast<float>(span_y);
        zoom_px_per_mm_ = std::clamp(std::min(zx, zy), 0.25f, 40.0f);
        initial_fit_done_ = true;
    }

    if (!in_cam_overlay && !in_cam_calibration && follow_torch_ &&
        (telemetry.state == core::ControllerState::RunningGCode ||
         telemetry.state == core::ControllerState::DwellPierce ||
         telemetry.state == core::ControllerState::Jogging))
    {
        pan_x_mm_ = static_cast<float>(telemetry.work_pos_mm[0]);
        pan_y_mm_ = static_cast<float>(telemetry.work_pos_mm[1]);
    }

    ImGui::SetNextItemAllowOverlap();
    ImGui::InvisibleButton("##ToolpathCanvas", canvas_sz,
                           ImGuiButtonFlags_MouseButtonLeft |
                           ImGuiButtonFlags_MouseButtonRight |
                           ImGuiButtonFlags_MouseButtonMiddle);
    const bool is_hovered = ImGui::IsItemHovered();
    const bool is_active  = ImGui::IsItemActive();

    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 mpos = io.MousePos;
    const bool over_toolbar = (mpos.x >= tb_p0.x && mpos.x <= tb_p1.x && mpos.y >= tb_p0.y && mpos.y <= tb_p1.y);
    const bool over_bottom_hud = (mpos.y >= canvas_p1.y - 36.0f);
    const bool allow_canvas_input = is_hovered && !over_toolbar && !over_bottom_hud;

    // Disable zoom & pan while in Camera Overlay or Camera Calibration mode to never lose sync
    if (!in_cam_overlay && !in_cam_calibration && allow_canvas_input && std::abs(io.MouseWheel) > 0.001f) {
        float factor = (io.MouseWheel > 0.0f) ? 1.15f : (1.0f / 1.15f);
        zoom_px_per_mm_ = std::clamp(zoom_px_per_mm_ * factor, 0.2f, 80.0f);
        interact_until_s_ = ImGui::GetTime() + kInteractHoldS;
    }

    // Precompute 3D camera trig for projection and screen-aligned panning
    const double yaw_rad   = static_cast<double>(view_yaw_deg_) * kDegToRad;
    const double pitch_rad = static_cast<double>(view_pitch_deg_) * kDegToRad;
    const double cos_yaw   = std::cos(yaw_rad);
    const double sin_yaw   = std::sin(yaw_rad);
    const double cos_pitch = std::cos(pitch_rad);
    const double sin_pitch = std::sin(pitch_rad);

    if (!in_cam_overlay && !in_cam_calibration && allow_canvas_input && is_active && !selecting_surface_rect_) {
        if (isometric_3d_ && ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
            // In 3D view, holding & dragging Right Mouse Button continuously rotates the 3D camera
            view_yaw_deg_   += io.MouseDelta.x * 0.45f;
            if (view_yaw_deg_ > 180.0f)  view_yaw_deg_ -= 360.0f;
            if (view_yaw_deg_ < -180.0f) view_yaw_deg_ += 360.0f;
            view_pitch_deg_ += io.MouseDelta.y * 0.45f;
            while (view_pitch_deg_ > 90.0f)  view_pitch_deg_ -= 180.0f;
            while (view_pitch_deg_ < -90.0f) view_pitch_deg_ += 180.0f;
            if (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f) {
                interact_until_s_ = ImGui::GetTime() + kInteractHoldS;
            }
        } else if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) ||
                   ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
                   (!isometric_3d_ && ImGui::IsMouseDragging(ImGuiMouseButton_Right)))
        {
            // Pan in 2D or 3D
            double d_screen_x = static_cast<double>(io.MouseDelta.x) / zoom_px_per_mm_;
            double d_screen_y = static_cast<double>(-io.MouseDelta.y) / zoom_px_per_mm_;
            if (!isometric_3d_) {
                pan_x_mm_ -= static_cast<float>(d_screen_x);
                pan_y_mm_ -= static_cast<float>(d_screen_y);
            } else {
                double eff_cos_p = (std::abs(cos_pitch) < 0.25) ? (cos_pitch >= 0.0 ? 0.25 : -0.25) : cos_pitch;
                double dy_rot = d_screen_y / eff_cos_p;
                pan_x_mm_ -= static_cast<float>(d_screen_x * cos_yaw + dy_rot * sin_yaw);
                pan_y_mm_ -= static_cast<float>(-d_screen_x * sin_yaw + dy_rot * cos_yaw);
            }
        }
    }

    const ImVec2 center_px(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);

    // Coordinate conversion helpers
    auto screenToWorkXY = [&](ImVec2 screen_pt) -> std::pair<double, double> {
        double x = static_cast<double>(pan_x_mm_) + static_cast<double>(screen_pt.x - center_px.x) / zoom_px_per_mm_;
        double y = static_cast<double>(pan_y_mm_) - static_cast<double>(screen_pt.y - center_px.y) / zoom_px_per_mm_;
        return {x, y};
    };

    if (selecting_surface_rect_) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_None);
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            cancelSelectingSurfaceRect();
        }
        if (allow_canvas_input && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            surface_rect_drag_active_ = true;
            surface_rect_start_px_ = mpos;
        }
        if (surface_rect_drag_active_) {
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                surface_rect_drag_active_ = false;
                selecting_surface_rect_   = false;
                auto p0 = screenToWorkXY(surface_rect_start_px_);
                auto p1 = screenToWorkXY(mpos);
                double xmin = std::min(p0.first, p1.first);
                double xmax = std::max(p0.first, p1.first);
                double ymin = std::min(p0.second, p1.second);
                double ymax = std::max(p0.second, p1.second);
                if (xmax - xmin >= 1.0 && ymax - ymin >= 1.0 && on_surface_rect_selected_) {
                    on_surface_rect_selected_(xmin, xmax, ymin, ymax);
                }
            }
        }
    }

    // Coordinate projection: Work (X, Y, Z) in mm -> Screen (px, py)
    auto projectXYZ = [&](double x_mm, double y_mm, double z_mm) -> ImVec2 {
        double dx = x_mm - static_cast<double>(pan_x_mm_);
        double dy = y_mm - static_cast<double>(pan_y_mm_);
        if (!isometric_3d_) {
            return ImVec2(
                center_px.x + static_cast<float>(dx) * zoom_px_per_mm_,
                center_px.y - static_cast<float>(dy) * zoom_px_per_mm_);
        } else {
            // 3D turntable rotation (Yaw around Z axis, Pitch tilt around X axis)
            double rx = dx * cos_yaw - dy * sin_yaw;
            double ry = dx * sin_yaw + dy * cos_yaw;
            double screen_x = rx;
            double screen_y = ry * cos_pitch + z_mm * sin_pitch;
            return ImVec2(
                center_px.x + static_cast<float>(screen_x) * zoom_px_per_mm_,
                center_px.y - static_cast<float>(screen_y) * zoom_px_per_mm_);
        }
    };

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(canvas_p0, canvas_p1, true);

    // 1. Background fill
    dl->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(14, 17, 22, 255), 6.0f);

    // 1.5 Camera Calibration Full-Screen Feed OR Calibrated 2D Camera Overlay
    if (in_cam_calibration && cam_gl_tex_ != 0 && cam_tex_w_ > 0 && cam_tex_h_ > 0) {
        // During calibration, display the camera feed instead of the toolpath viewport
        float img_aspect = static_cast<float>(cam_tex_w_) / static_cast<float>(cam_tex_h_);
        float vp_aspect  = canvas_sz.x / canvas_sz.y;
        float draw_w     = canvas_sz.x;
        float draw_h     = canvas_sz.y;
        if (vp_aspect > img_aspect) {
            draw_h = canvas_sz.y;
            draw_w = draw_h * img_aspect;
        } else {
            draw_w = canvas_sz.x;
            draw_h = draw_w / img_aspect;
        }
        ImVec2 img_p0(center_px.x - draw_w * 0.5f, center_px.y - draw_h * 0.5f);
        ImVec2 img_p1(center_px.x + draw_w * 0.5f, center_px.y + draw_h * 0.5f);

        dl->AddImage(static_cast<ImTextureID>(static_cast<uintptr_t>(cam_gl_tex_)),
                     img_p0, img_p1, ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f),
                     IM_COL32(255, 255, 255, 255));
        dl->AddRect(img_p0, img_p1, IM_COL32(0, 210, 255, 180), 2.0f, 0, 1.5f);

        // Center calibration crosshair & target rings
        dl->AddLine(ImVec2(img_p0.x, center_px.y), ImVec2(img_p1.x, center_px.y), IM_COL32(0, 0, 0, 180), 2.6f);
        dl->AddLine(ImVec2(center_px.x, img_p0.y), ImVec2(center_px.x, img_p1.y), IM_COL32(0, 0, 0, 180), 2.6f);
        dl->AddLine(ImVec2(img_p0.x, center_px.y), ImVec2(img_p1.x, center_px.y), IM_COL32(0, 245, 255, 235), 1.2f);
        dl->AddLine(ImVec2(center_px.x, img_p0.y), ImVec2(center_px.x, img_p1.y), IM_COL32(0, 245, 255, 235), 1.2f);
        dl->AddCircle(center_px, 14.0f, IM_COL32(0, 245, 255, 230), 28, 1.5f);
        dl->AddCircle(center_px, 38.0f, IM_COL32(0, 245, 255, 140), 36, 1.0f);
        dl->AddCircleFilled(center_px, 2.2f, IM_COL32(255, 70, 70, 255), 12);

        // Calibration status banner at top
        ImVec2 banner_p0(canvas_p0.x + 8.0f, tb_p1.y + 34.0f);
        ImVec2 banner_p1(canvas_p1.x - 8.0f, banner_p0.y + 32.0f);
        dl->AddRectFilled(banner_p0, banner_p1, IM_COL32(10, 18, 28, kOverlayBgAlpha255), 4.0f);
        dl->AddRect(banner_p0, banner_p1, IM_COL32(0, 215, 255, 230), 4.0f, 0, 1.5f);
        if (waiting_for_dent_click_) {
            dl->AddText(ImVec2(banner_p0.x + 12.0f, banner_p0.y + 7.0f),
                        IM_COL32(255, 220, 70, 255),
                        "CAMERA FOV CALIBRATION: Click directly on the displaced dent in the camera image to calculate FOV  |  [Esc] Cancel");
            if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                waiting_for_dent_click_ = false;
            }
            if (allow_canvas_input && mpos.x >= img_p0.x && mpos.x <= img_p1.x &&
                mpos.y >= img_p0.y && mpos.y <= img_p1.y)
            {
                ImGui::SetMouseCursor(ImGuiMouseCursor_None);
                dl->AddLine(ImVec2(mpos.x - 12.0f, mpos.y), ImVec2(mpos.x + 12.0f, mpos.y), IM_COL32(255, 220, 50, 255), 1.6f);
                dl->AddLine(ImVec2(mpos.x, mpos.y - 12.0f), ImVec2(mpos.x, mpos.y + 12.0f), IM_COL32(255, 220, 50, 255), 1.6f);
                dl->AddCircle(mpos, 6.0f, IM_COL32(255, 220, 50, 255), 16, 1.5f);

                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    double norm_dx = static_cast<double>(mpos.x - center_px.x) / static_cast<double>(draw_w);
                    double norm_dy = static_cast<double>(center_px.y - mpos.y) / static_cast<double>(draw_h);
                    if (std::abs(norm_dx) > 0.01 || std::abs(norm_dy) > 0.01) {
                        waiting_for_dent_click_ = false;
                        if (on_dent_clicked_) {
                            on_dent_clicked_(norm_dx, norm_dy, cam_tex_w_, cam_tex_h_);
                        }
                    }
                }
            }
        } else {
            dl->AddText(ImVec2(banner_p0.x + 12.0f, banner_p0.y + 7.0f),
                        IM_COL32(0, 235, 255, 255),
                        "CAMERA CALIBRATION VIEW: Align center crosshair with the endmill dent or measure FOV in the Camera tab");
        }
    } else if (in_cam_overlay && cam_gl_tex_ != 0 && cam_tex_w_ > 0 && cam_tex_h_ > 0) {
        // Draw calibrated camera feed locked underneath the 2D G-code toolpath
        double cam_world_x = static_cast<double>(pan_x_mm_);
        double cam_world_y = static_cast<double>(pan_y_mm_);
        ImVec2 img_p0 = projectXYZ(cam_world_x - cam_cfg.fov_x_mm * 0.5, cam_world_y + cam_cfg.fov_y_mm * 0.5, 0.0);
        ImVec2 img_p1 = projectXYZ(cam_world_x + cam_cfg.fov_x_mm * 0.5, cam_world_y - cam_cfg.fov_y_mm * 0.5, 0.0);
        uint8_t alpha = static_cast<uint8_t>(std::clamp(cam_cfg.overlay_opacity * 255.0f, 30.0f, 255.0f));
        dl->AddImage(static_cast<ImTextureID>(static_cast<uintptr_t>(cam_gl_tex_)),
                     img_p0, img_p1, ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f),
                     IM_COL32(255, 255, 255, alpha));
        dl->AddRect(img_p0, img_p1, IM_COL32(0, 215, 255, 190), 0.0f, 0, 1.5f);
    }

    const size_t active_idx = telemetry.active_segment_index;
    const float  active_t   = std::clamp(telemetry.segment_progress_0_to_1, 0.0f, 1.0f);

    if (!in_cam_calibration) {
    // 2. Coordinate Grid (10mm minor, 50mm major)
    const double grid_step_mm = (zoom_px_per_mm_ > 2.5f) ? 10.0 : 50.0;
    const int min_gx = static_cast<int>(std::floor((pan_x_mm_ - canvas_sz.x / zoom_px_per_mm_) / grid_step_mm));
    const int max_gx = static_cast<int>(std::ceil((pan_x_mm_ + canvas_sz.x / zoom_px_per_mm_) / grid_step_mm));
    const int min_gy = static_cast<int>(std::floor((pan_y_mm_ - canvas_sz.y / zoom_px_per_mm_) / grid_step_mm));
    const int max_gy = static_cast<int>(std::ceil((pan_y_mm_ + canvas_sz.y / zoom_px_per_mm_) / grid_step_mm));

    for (int gx = min_gx; gx <= max_gx; ++gx) {
        double x = gx * grid_step_mm;
        bool is_major = (std::abs(gx) % 5 == 0);
        ImU32 col = (gx == 0) ? IM_COL32(180, 60, 60, 140)
                              : (is_major ? IM_COL32(42, 50, 62, 160) : IM_COL32(26, 32, 40, 120));
        ImVec2 p0 = projectXYZ(x, min_gy * grid_step_mm, 0.0);
        ImVec2 p1 = projectXYZ(x, max_gy * grid_step_mm, 0.0);
        dl->AddLine(p0, p1, col, (gx == 0) ? 1.6f : 1.0f);
    }
    for (int gy = min_gy; gy <= max_gy; ++gy) {
        double y = gy * grid_step_mm;
        bool is_major = (std::abs(gy) % 5 == 0);
        ImU32 col = (gy == 0) ? IM_COL32(60, 180, 80, 140)
                              : (is_major ? IM_COL32(42, 50, 62, 160) : IM_COL32(26, 32, 40, 120));
        ImVec2 p0 = projectXYZ(min_gx * grid_step_mm, y, 0.0);
        ImVec2 p1 = projectXYZ(max_gx * grid_step_mm, y, 0.0);
        dl->AddLine(p0, p1, col, (gy == 0) ? 1.6f : 1.0f);
    }

    // Work Origin (0,0) Marker & 3D Axis Triad
    ImVec2 origin_px = projectXYZ(0.0, 0.0, 0.0);
    if (isometric_3d_) {
        double axis_len_mm = std::clamp(28.0 / static_cast<double>(zoom_px_per_mm_), 4.0, 60.0);
        ImVec2 ax_x = projectXYZ(axis_len_mm, 0.0, 0.0);
        ImVec2 ax_y = projectXYZ(0.0, axis_len_mm, 0.0);
        ImVec2 ax_z = projectXYZ(0.0, 0.0, axis_len_mm);
        dl->AddLine(origin_px, ax_x, IM_COL32(255, 85, 85, 230), 2.2f);
        dl->AddLine(origin_px, ax_y, IM_COL32(85, 240, 110, 230), 2.2f);
        dl->AddLine(origin_px, ax_z, IM_COL32(85, 175, 255, 230), 2.2f);
        dl->AddText(ImVec2(ax_x.x + 3.0f, ax_x.y - 6.0f), IM_COL32(255, 110, 110, 240), "X");
        dl->AddText(ImVec2(ax_y.x + 3.0f, ax_y.y - 6.0f), IM_COL32(110, 255, 130, 240), "Y");
        dl->AddText(ImVec2(ax_z.x + 3.0f, ax_z.y - 6.0f), IM_COL32(110, 195, 255, 240), "Z");
    }
    dl->AddCircle(origin_px, 5.0f, IM_COL32(220, 220, 230, 180), 16, 1.5f);
    dl->AddText(ImVec2(origin_px.x + 7.0f, origin_px.y + 4.0f), IM_COL32(180, 190, 205, 190), "(0,0)");

    // 2.5 Probed Surface Height Map Mesh & Heat Dots
    if (telemetry.surface_height_map->is_valid &&
        telemetry.surface_height_map->points_x >= 2 &&
        telemetry.surface_height_map->points_y >= 2)
    {
        const auto& shm = *telemetry.surface_height_map;
        const int px = shm.points_x;
        const int py = shm.points_y;
        const double dx = (shm.max_x - shm.min_x) / static_cast<double>(px - 1);
        const double dy = (shm.max_y - shm.min_y) / static_cast<double>(py - 1);

        const double active_smoothness = has_pending_surface_change_ ? proposed_surface_smoothness_ : shm.smoothness;
        const int sub_divisions = (active_smoothness > 0.05 && px >= 3 && py >= 3) ? 4 : 1;

        // If pending change, first draw subtle applied baseline mesh
        if (has_pending_surface_change_) {
            for (int j = 0; j < py; ++j) {
                double y = shm.min_y + static_cast<double>(j) * dy;
                for (int i = 0; i < px; ++i) {
                    double x0 = shm.min_x + static_cast<double>(i) * dx;
                    double z0 = shm.getInterpolatedZWithSmoothness(x0, y, shm.smoothness);
                    ImVec2 p0 = projectXYZ(x0, y, z0);
                    if (i + 1 < px) {
                        double x1 = shm.min_x + static_cast<double>(i + 1) * dx;
                        double z1 = shm.getInterpolatedZWithSmoothness(x1, y, shm.smoothness);
                        dl->AddLine(p0, projectXYZ(x1, y, z1), IM_COL32(0, 160, 200, 50), 1.0f);
                    }
                    if (j + 1 < py) {
                        double y1 = shm.min_y + static_cast<double>(j + 1) * dy;
                        double z1 = shm.getInterpolatedZWithSmoothness(x0, y1, shm.smoothness);
                        dl->AddLine(p0, projectXYZ(x0, y1, z1), IM_COL32(0, 160, 200, 50), 1.0f);
                    }
                }
            }
        }

        const ImU32 col_mesh = has_pending_surface_change_ ? IM_COL32(255, 190, 45, 190) : IM_COL32(0, 180, 230, 90);
        const float line_thick = has_pending_surface_change_ ? 1.8f : 1.2f;

        // Draw mesh grid lines connecting probed nodes with curvature subdivision
        for (int j = 0; j < py; ++j) {
            double y = shm.min_y + static_cast<double>(j) * dy;
            for (int i = 0; i < px; ++i) {
                double x0 = shm.min_x + static_cast<double>(i) * dx;
                if (i + 1 < px) {
                    double x1 = shm.min_x + static_cast<double>(i + 1) * dx;
                    for (int s = 0; s < sub_divisions; ++s) {
                        double xa = x0 + (x1 - x0) * (static_cast<double>(s) / sub_divisions);
                        double xb = x0 + (x1 - x0) * (static_cast<double>(s + 1) / sub_divisions);
                        double za = shm.getInterpolatedZWithSmoothness(xa, y, active_smoothness);
                        double zb = shm.getInterpolatedZWithSmoothness(xb, y, active_smoothness);
                        dl->AddLine(projectXYZ(xa, y, za), projectXYZ(xb, y, zb), col_mesh, line_thick);
                    }
                }
                if (j + 1 < py) {
                    double y1 = shm.min_y + static_cast<double>(j + 1) * dy;
                    for (int s = 0; s < sub_divisions; ++s) {
                        double ya = y + (y1 - y) * (static_cast<double>(s) / sub_divisions);
                        double yb = y + (y1 - y) * (static_cast<double>(s + 1) / sub_divisions);
                        double za = shm.getInterpolatedZWithSmoothness(x0, ya, active_smoothness);
                        double zb = shm.getInterpolatedZWithSmoothness(x0, yb, active_smoothness);
                        dl->AddLine(projectXYZ(x0, ya, za), projectXYZ(x0, yb, zb), col_mesh, line_thick);
                    }
                }
            }
        }

        // Draw elevation heat dots at each grid vertex
        for (int j = 0; j < py; ++j) {
            double y = shm.min_y + static_cast<double>(j) * dy;
            for (int i = 0; i < px; ++i) {
                double x = shm.min_x + static_cast<double>(i) * dx;
                double z = shm.grid_z[static_cast<size_t>(j * px + i)];
                ImVec2 p0 = projectXYZ(x, y, z);

                ImU32 dot_col;
                if (std::abs(z) < 0.05) {
                    dot_col = IM_COL32(50, 230, 120, 230); // Nominal green
                } else if (z > 0.0) {
                    float t = std::clamp(static_cast<float>(z / 0.5), 0.0f, 1.0f);
                    dot_col = IM_COL32(255, static_cast<int>(210 - 110 * t), 40, 240); // Warm Amber/Red
                } else {
                    float t = std::clamp(static_cast<float>(-z / 0.5), 0.0f, 1.0f);
                    dot_col = IM_COL32(40, static_cast<int>(180 + 75 * t), 255, 240); // Cool Cyan/Blue
                }

                dl->AddCircleFilled(p0, 3.5f, dot_col);
                dl->AddCircle(p0, 4.5f, IM_COL32(15, 20, 30, 220), 12, 1.0f);
            }
        }
    }

    // 3. Draw Toolpath Segments: Stratified Layering (Rapids -> Pierces -> Uncut -> Completed -> Active)
    // Completed cuts (neon green) are ALWAYS rendered on top of uncut lines so toolpath progress is never obscured!
    const ImU32 col_rapid            = IM_COL32(120, 135, 155, 150);
    const ImU32 col_uncut            = IM_COL32(0, 225, 255, 235);   // Bright Cyan = Active Tool To Be Cut
    const ImU32 col_uncut_other_tool = IM_COL32(28, 110, 148, 150);  // Blueprint Teal = Other Tool To Be Cut
    const ImU32 col_cut_done         = IM_COL32(0, 235, 118, 255);   // Neon Emerald Green = Already Cut
    const ImU32 col_active_cut       = IM_COL32(255, 155, 0, 255);   // Glowing Amber = Active Segment

    uint16_t current_active_tool = telemetry.current_tool_number;
    if (current_active_tool == 0 && active_idx < segments.size()) {
        current_active_tool = segments[active_idx].tool_number;
    }
    if (current_active_tool == 0 && !segments.empty()) {
        current_active_tool = segments[0].tool_number;
    }

    // Whole-pixel line widths: ImGui draws them from its line texture with half the vertices of fractional widths
    const ImVec4 canvas_clip(canvas_p0.x, canvas_p0.y, canvas_p1.x, canvas_p1.y);
    const ImVec4 cull = cullRect(canvas_clip);

    // Cached geometry + this frame's pan offset
    updatePathCache(segments);
    const ImVec2 pan_px = projectNoPan(pan_x_mm_, pan_y_mm_, 0.0);
    const ImVec2 off(center_px.x - pan_px.x, center_px.y - pan_px.y);
    auto shifted = [&off](ImVec2 p) { return ImVec2(p.x + off.x, p.y + off.y); };

    // Layer 1: Rapids (G0 travel moves)
    if (show_rapids_) {
        const auto& r = path_cache_.rapids;
        for (size_t i = 0; i + 1 < r.size(); i += 2) {
            drawDashedLine(dl, shifted(r[i]), shifted(r[i + 1]), col_rapid, 1.0f, 5.0f, canvas_clip);
        }
    }

    // Layer 2: Pierce / Dwell markers (G4)
    if (show_pierces_) {
        for (const auto& pc : path_cache_.pierces) {
            const ImVec2 pt = shifted(pc.p);
            if (pt.x < cull.x - 8.0f || pt.x > cull.z + 8.0f || pt.y < cull.y - 8.0f || pt.y > cull.w + 8.0f) continue;
            bool done = (pc.seg < active_idx);
            ImU32 pcol = done ? col_cut_done : IM_COL32(255, 190, 40, 220);
            dl->AddCircleFilled(pt, 4.0f, pcol, 12);
            dl->AddCircle(pt, 7.0f, pcol, 12, 1.2f);
        }
    }

    // Compute active cut segment span [active_idx, active_end_idx]
    // For G2/G3 arcs or CAM-approximated curve chords, this groups the entire upcoming curve
    // so it gets rendered with the prominent 3.5f orange line ahead of the tool
    size_t active_end_idx = active_idx;
    if (active_idx < segments.size()) {
        const auto& active_seg = segments[active_idx];
        if (active_seg.type == core::SegmentType::CutArcCW || active_seg.type == core::SegmentType::CutArcCCW) {
            // G2/G3 arc composed of multiple tessellated chords sharing the same gcode_line
            while (active_end_idx + 1 < segments.size()) {
                const auto& nxt = segments[active_end_idx + 1];
                if (nxt.gcode_line == active_seg.gcode_line &&
                    (nxt.type == core::SegmentType::CutArcCW || nxt.type == core::SegmentType::CutArcCCW))
                {
                    active_end_idx++;
                } else {
                    break;
                }
            }
        } else if (active_seg.type == core::SegmentType::CutLinear && active_seg.length_mm < 3.0) {
            // CAM-approximated curve (sequence of short linear chords)
            double accum_len = active_seg.length_mm * (1.0 - active_t);
            while (active_end_idx + 1 < segments.size() && accum_len < 20.0) {
                const auto& nxt = segments[active_end_idx + 1];
                if (nxt.type == core::SegmentType::CutLinear && nxt.length_mm < 3.0 &&
                    nxt.tool_number == active_seg.tool_number)
                {
                    accum_len += nxt.length_mm;
                    active_end_idx++;
                } else {
                    break;
                }
            }
        }
    }

    // A cached line that covers segments on both sides of the active span is redrawn from its own segments,
    // so progress is shown segment-exact instead of whole merged lines changing color at once
    auto drawSegmentRange = [&](uint32_t first, uint32_t last, ImU32 col) {
        ToolpathLineBatch<DrawLineSink> batch(DrawLineSink{dl, cull, col}, simplify_px_);
        for (uint32_t i = first; i <= last && i < segments.size(); ++i) {
            const auto& seg = segments[i];
            if (!isCutSegment(seg)) continue;
            if (selected_tool_filter_ != 0 && seg.tool_number != selected_tool_filter_) continue;
            batch.add(projectXYZ(seg.start_xyza_mm[0], seg.start_xyza_mm[1], seg.start_xyza_mm[2]),
                      projectXYZ(seg.end_xyza_mm[0], seg.end_xyza_mm[1], seg.end_xyza_mm[2]), i);
        }
    };

    // Layer 3: Uncut Cut Segments (seg.segment_index > active_end_idx)
    // Drawn FIRST so future operations never paint over completed cuts
    for (const auto& ln : path_cache_.cuts) {
        if (ln.last_seg <= active_end_idx) continue;
        const ImU32 col = (ln.tool == current_active_tool) ? col_uncut : col_uncut_other_tool;
        if (ln.first_seg > active_end_idx) {
            const ImVec2 a = shifted(ln.a), b = shifted(ln.b);
            if (!lineOutside(a, b, cull)) dl->AddLine(a, b, col, 1.0f);
        } else {
            drawSegmentRange(static_cast<uint32_t>(active_end_idx + 1), ln.last_seg, col);
        }
    }

    // Layer 4: Completed Cut Segments (seg.segment_index < active_idx)
    // Drawn ON TOP of uncut lines with vivid neon green
    for (const auto& ln : path_cache_.cuts) {
        if (ln.first_seg >= active_idx) continue;
        if (ln.last_seg < active_idx) {
            const ImVec2 a = shifted(ln.a), b = shifted(ln.b);
            if (!lineOutside(a, b, cull)) dl->AddLine(a, b, col_cut_done, 1.0f);
        } else {
            drawSegmentRange(ln.first_seg, static_cast<uint32_t>(active_idx - 1), col_cut_done);
        }
    }

    // Layer 5: Currently Active Cut Segment & Curve Continuation (active_idx .. active_end_idx)
    if (active_idx < segments.size()) {
        const auto& seg = segments[active_idx];
        if (seg.type == core::SegmentType::CutLinear ||
            seg.type == core::SegmentType::CutArcCW ||
            seg.type == core::SegmentType::CutArcCCW)
        {
            if (selected_tool_filter_ == 0 || seg.tool_number == selected_tool_filter_) {
                ImVec2 p0 = projectXYZ(seg.start_xyza_mm[0], seg.start_xyza_mm[1], seg.start_xyza_mm[2]);
                ImVec2 p1 = projectXYZ(seg.end_xyza_mm[0], seg.end_xyza_mm[1], seg.end_xyza_mm[2]);
                ImVec2 pm(p0.x + (p1.x - p0.x) * active_t, p0.y + (p1.y - p0.y) * active_t);
                dl->AddLine(p0, pm, col_cut_done, 1.2f);
                dl->AddLine(pm, p1, col_active_cut, 3.5f);

                // Highlight remainder of active curve / arc with the same prominent 3.5f orange line
                for (size_t k = active_idx + 1; k <= active_end_idx; ++k) {
                    const auto& k_seg = segments[k];
                    ImVec2 kp0 = projectXYZ(k_seg.start_xyza_mm[0], k_seg.start_xyza_mm[1], k_seg.start_xyza_mm[2]);
                    ImVec2 kp1 = projectXYZ(k_seg.end_xyza_mm[0], k_seg.end_xyza_mm[1], k_seg.end_xyza_mm[2]);
                    dl->AddLine(kp0, kp1, col_active_cut, 3.5f);
                }
            }
        }
    }

    // 4. Draw Live Torch / Tool Position Reticle
    ImVec2 torch_px = projectXYZ(telemetry.work_pos_mm[0], telemetry.work_pos_mm[1], telemetry.work_pos_mm[2]);
    if (telemetry.torch_or_spindle_on) {
        // Plasma arc / Active Spindle halo!
        dl->AddCircleFilled(torch_px, 14.0f, IM_COL32(255, 120, 0, 70), 24);
        dl->AddCircleFilled(torch_px, 7.0f, IM_COL32(255, 230, 80, 240), 20);
        dl->AddCircle(torch_px, 11.0f, IM_COL32(255, 80, 20, 255), 24, 2.0f);
    } else {
        dl->AddCircle(torch_px, 8.0f, IM_COL32(255, 255, 255, 230), 20, 1.8f);
        dl->AddCircleFilled(torch_px, 2.5f, IM_COL32(0, 225, 255, 255), 12);
    }
    dl->AddLine(ImVec2(torch_px.x - 14.0f, torch_px.y), ImVec2(torch_px.x + 14.0f, torch_px.y),
                IM_COL32(255, 255, 255, 190), 1.2f);
    dl->AddLine(ImVec2(torch_px.x, torch_px.y - 14.0f), ImVec2(torch_px.x, torch_px.y + 14.0f),
                IM_COL32(255, 255, 255, 190), 1.2f);

    // 4.5 Draw Camera Center Crosshair when 2D Camera Overlay Mode is active
    if (in_cam_overlay) {
        dl->AddLine(ImVec2(center_px.x - 28.0f, center_px.y), ImVec2(center_px.x + 28.0f, center_px.y),
                    IM_COL32(0, 0, 0, 190), 2.5f);
        dl->AddLine(ImVec2(center_px.x, center_px.y - 28.0f), ImVec2(center_px.x, center_px.y + 28.0f),
                    IM_COL32(0, 0, 0, 190), 2.5f);
        dl->AddLine(ImVec2(center_px.x - 28.0f, center_px.y), ImVec2(center_px.x + 28.0f, center_px.y),
                    IM_COL32(255, 220, 60, 240), 1.3f);
        dl->AddLine(ImVec2(center_px.x, center_px.y - 28.0f), ImVec2(center_px.x, center_px.y + 28.0f),
                    IM_COL32(255, 220, 60, 240), 1.3f);
        dl->AddCircle(center_px, 12.0f, IM_COL32(255, 220, 60, 230), 24, 1.5f);
        dl->AddText(ImVec2(center_px.x + 14.0f, center_px.y - 16.0f), IM_COL32(255, 225, 80, 245), "CAM");
    }
    } // end if (!in_cam_calibration)

    // 5. Bottom-Left Viewport HUD Overlay: individual digital textboxes, each as wide as its label and value
    size_t cut_done_count = std::min(active_idx, segments.size());
    float pct_done = segments.empty() ? 0.0f : (100.0f * static_cast<float>(cut_done_count) / static_cast<float>(segments.size()));
    int elapsed_min = static_cast<int>(telemetry.job_elapsed_seconds) / 60;
    double elapsed_sec = std::fmod(telemetry.job_elapsed_seconds, 60.0);

    ImFont* hud_font = AppWindow::getGCodeFont();
    const float hud_font_sz = 15.0f;
    const float box_h       = 26.0f;
    const float box_gap     = 6.0f;
    const float hud_y       = canvas_p1.y - box_h - 8.0f;
    float cur_x             = canvas_p0.x + 10.0f;

    // Label + value + 8 px padding each side + 12 px between them (matches drawHudTextbox)
    auto hudBoxWidth = [hud_font](float font_sz, const char* label, const char* value) {
        auto text_w = [&](const char* s) {
            return hud_font ? hud_font->CalcTextSizeA(font_sz, 1000.0f, 0.0f, s).x : ImGui::CalcTextSize(s).x;
        };
        return std::ceil(text_w(label) + text_w(value) + 28.0f);
    };

    const bool is_imp = (controller && controller->getConfig().linear_units == core::LinearUnits::Imperial);
    const double b_scale = is_imp ? (1.0 / 25.4) : 1.0;
    char bounds_val[64];
    if (is_imp) {
        std::snprintf(bounds_val, sizeof(bounds_val), "%.2f x %.2f in",
                      (bbox_max_mm[0] - bbox_min_mm[0]) * b_scale,
                      (bbox_max_mm[1] - bbox_min_mm[1]) * b_scale);
    } else {
        std::snprintf(bounds_val, sizeof(bounds_val), "%.1f x %.1f mm",
                      bbox_max_mm[0] - bbox_min_mm[0],
                      bbox_max_mm[1] - bbox_min_mm[1]);
    }
    const float bounds_w = hudBoxWidth(hud_font_sz, "Part Bounds:", bounds_val);
    drawHudTextbox(dl, hud_font, hud_font_sz, ImVec2(cur_x, hud_y), bounds_w, box_h,
                   "Part Bounds:", bounds_val, IM_COL32(215, 228, 245, 255));
    cur_x += bounds_w + box_gap;

    char prog_val[64];
    std::snprintf(prog_val, sizeof(prog_val), "%zu / %zu (%3.0f%%)",
                  cut_done_count, segments.size(), pct_done);
    const float prog_w = hudBoxWidth(hud_font_sz, "Progress:", prog_val);
    drawHudTextbox(dl, hud_font, hud_font_sz, ImVec2(cur_x, hud_y), prog_w, box_h,
                   "Progress:", prog_val, IM_COL32(0, 230, 140, 255));
    cur_x += prog_w + box_gap;

    char elapsed_val[64];
    std::snprintf(elapsed_val, sizeof(elapsed_val), "%02d:%04.1f s",
                  elapsed_min, elapsed_sec);
    const float elapsed_w = hudBoxWidth(hud_font_sz, "Elapsed:", elapsed_val);
    drawHudTextbox(dl, hud_font, hud_font_sz, ImVec2(cur_x, hud_y), elapsed_w, box_h,
                   "Elapsed:", elapsed_val, IM_COL32(255, 210, 75, 255));
    cur_x += elapsed_w + box_gap;

    // 6. Top-Left Viewport HUD Overlay: Zoom & 3D RMB readouts right below the top-left view buttons
    const float top_hud_y   = tb_p1.y + 4.0f;
    float top_cur_x         = canvas_p0.x + 8.0f;
    const float top_box_h   = 24.0f;
    const float top_font_sz = 14.0f;

    char zoom_val[64];
    std::snprintf(zoom_val, sizeof(zoom_val), "%.1f px/mm", zoom_px_per_mm_);
    const float zoom_w = hudBoxWidth(top_font_sz, "Zoom:", zoom_val);
    drawHudTextbox(dl, hud_font, top_font_sz, ImVec2(top_cur_x, top_hud_y), zoom_w, top_box_h,
                   "Zoom:", zoom_val, IM_COL32(120, 215, 255, 255));
    top_cur_x += zoom_w + box_gap;

    if (isometric_3d_) {
        char rot_val[64];
        std::snprintf(rot_val, sizeof(rot_val), "Y:%+.0f P:%+.0f", view_yaw_deg_, view_pitch_deg_);
        const float rot_w = hudBoxWidth(top_font_sz, "3D RMB:", rot_val);
        drawHudTextbox(dl, hud_font, top_font_sz, ImVec2(top_cur_x, top_hud_y), rot_w, top_box_h,
                       "3D RMB:", rot_val, IM_COL32(190, 175, 255, 255));
        top_cur_x += rot_w + box_gap;
    } else if (in_cam_overlay) {
        const float sync_w = hudBoxWidth(top_font_sz, "Cam Sync:", "PAN/ZOOM LOCKED");
        drawHudTextbox(dl, hud_font, top_font_sz, ImVec2(top_cur_x, top_hud_y), sync_w, top_box_h,
                       "Cam Sync:", "PAN/ZOOM LOCKED", IM_COL32(255, 215, 70, 255));
        top_cur_x += sync_w + box_gap;
    } else if (in_cam_calibration) {
        const float calib_w = hudBoxWidth(top_font_sz, "Cam Mode:", "CALIBRATING");
        drawHudTextbox(dl, hud_font, top_font_sz, ImVec2(top_cur_x, top_hud_y), calib_w, top_box_h,
                       "Cam Mode:", "CALIBRATING", IM_COL32(0, 235, 255, 255));
        top_cur_x += calib_w + box_gap;
    }

    // Color legend on the same row, aligned to the right edge (skipped if the view is too narrow for it)
    {
        struct LegendItem { const char* label; ImU32 col; };
        const LegendItem items[] = {
            {"Cut",       IM_COL32(0, 230, 117, 255)},
            {"Active",    IM_COL32(255, 145, 0, 255)},
            {"To Be Cut", IM_COL32(0, 230, 255, 255)},
            {"Rapid",     IM_COL32(140, 153, 173, 255)},
        };
        const float pad = 8.0f, swatch_w = 12.0f, swatch_gap = 5.0f, item_gap = 12.0f;
        auto textWidth = [&](const char* s) {
            return hud_font ? hud_font->CalcTextSizeA(top_font_sz, 1000.0f, 0.0f, s).x : ImGui::CalcTextSize(s).x;
        };
        float legend_w = pad * 2.0f - item_gap;
        for (const auto& it : items) legend_w += swatch_w + swatch_gap + textWidth(it.label) + item_gap;

        const float legend_x = canvas_p1.x - 8.0f - legend_w;
        if (legend_x >= top_cur_x) {
            const ImVec2 box_p0(legend_x, top_hud_y);
            const ImVec2 box_p1(legend_x + legend_w, top_hud_y + top_box_h);
            dl->AddRectFilled(box_p0, box_p1, IM_COL32(10, 13, 18, kOverlayBgAlpha255), 4.0f);
            dl->AddRect(box_p0, box_p1, IM_COL32(46, 62, 82, 215), 4.0f, 0, 1.0f);
            const float text_y = top_hud_y + (top_box_h - top_font_sz) * 0.5f - 1.0f;
            const float mid_y  = top_hud_y + top_box_h * 0.5f;
            float x = legend_x + pad;
            for (const auto& it : items) {
                dl->AddLine(ImVec2(x, mid_y), ImVec2(x + swatch_w, mid_y), it.col, 3.0f);
                x += swatch_w + swatch_gap;
                dl->AddText(hud_font, top_font_sz, ImVec2(x, text_y), it.col, it.label);
                x += textWidth(it.label) + item_gap;
            }
        }
    }

    // 5.5 Interactive Surface Rectangle Selection Overlay
    if (selecting_surface_rect_) {
        ImVec2 banner_p0(canvas_p0.x + 8.0f, canvas_p0.y + tb_h + 10.0f);
        ImVec2 banner_p1(canvas_p1.x - 8.0f, banner_p0.y + 32.0f);
        dl->AddRectFilled(banner_p0, banner_p1, IM_COL32(12, 20, 32, kOverlayBgAlpha255), 4.0f);
        dl->AddRect(banner_p0, banner_p1, IM_COL32(0, 200, 255, 220), 4.0f, 0, 1.5f);
        dl->AddText(ImVec2(banner_p0.x + 12.0f, banner_p0.y + 7.0f),
                    IM_COL32(0, 235, 255, 255),
                    "DRAW SURFACE PROBING RECTANGLE: Drag mouse on 2D canvas to define area  |  [Esc] Cancel");

        if (surface_rect_drag_active_) {
            ImVec2 rmin(std::min(surface_rect_start_px_.x, mpos.x), std::min(surface_rect_start_px_.y, mpos.y));
            ImVec2 rmax(std::max(surface_rect_start_px_.x, mpos.x), std::max(surface_rect_start_px_.y, mpos.y));
            dl->AddRectFilled(rmin, rmax, IM_COL32(0, 180, 255, 45));
            dl->AddRect(rmin, rmax, IM_COL32(0, 230, 255, 240), 0.0f, 0, 2.0f);

            auto p0 = screenToWorkXY(surface_rect_start_px_);
            auto p1 = screenToWorkXY(mpos);
            double w = std::abs(p1.first - p0.first);
            double h = std::abs(p1.second - p0.second);
            double rx_min = std::min(p0.first, p1.first);
            double rx_max = std::max(p0.first, p1.first);
            double ry_min = std::min(p0.second, p1.second);
            double ry_max = std::max(p0.second, p1.second);

            const double tip_scale = is_imp ? (1.0 / 25.4) : 1.0;
            const char* tip_unit = is_imp ? "in" : "mm";
            char dim_buf[160];
            if (is_imp) {
                std::snprintf(dim_buf, sizeof(dim_buf), "%.2f x %.2f %s\nX:[%.2f..%.2f] Y:[%.2f..%.2f]",
                              w * tip_scale, h * tip_scale, tip_unit,
                              rx_min * tip_scale, rx_max * tip_scale,
                              ry_min * tip_scale, ry_max * tip_scale);
            } else {
                std::snprintf(dim_buf, sizeof(dim_buf), "%.1f x %.1f %s\nX:[%.1f..%.1f] Y:[%.1f..%.1f]",
                              w, h, tip_unit, rx_min, rx_max, ry_min, ry_max);
            }

            ImVec2 text_sz = ImGui::CalcTextSize(dim_buf);
            const float pad_x = 10.0f;
            const float pad_y = 6.0f;
            const float tip_w = text_sz.x + pad_x * 2.0f;
            const float tip_h = text_sz.y + pad_y * 2.0f;

            float tip_x0 = rmax.x + 10.0f;
            float tip_y0 = rmax.y - tip_h;
            if (tip_x0 + tip_w > canvas_p1.x - 8.0f) {
                tip_x0 = rmin.x - tip_w - 10.0f;
            }
            if (tip_y0 < canvas_p0.y + tb_h + 8.0f) {
                tip_y0 = canvas_p0.y + tb_h + 8.0f;
            }

            dl->AddRectFilled(ImVec2(tip_x0, tip_y0), ImVec2(tip_x0 + tip_w, tip_y0 + tip_h), IM_COL32(10, 15, 24, kOverlayBgAlpha255), 4.0f);
            dl->AddRect(ImVec2(tip_x0, tip_y0), ImVec2(tip_x0 + tip_w, tip_y0 + tip_h), IM_COL32(0, 200, 255, 210), 4.0f, 0, 1.2f);
            dl->AddText(ImVec2(tip_x0 + pad_x, tip_y0 + pad_y), IM_COL32(230, 245, 255, 255), dim_buf);
        }

        // Sleek, high-contrast crosshair cursor at mouse location
        if (mpos.x >= canvas_p0.x && mpos.x <= canvas_p1.x &&
            mpos.y >= canvas_p0.y && mpos.y <= canvas_p1.y) {
            const float ch_len = 11.0f;
            const float ch_gap = 3.0f;
            // Shadow outline
            dl->AddLine(ImVec2(mpos.x - ch_len, mpos.y), ImVec2(mpos.x - ch_gap, mpos.y), IM_COL32(0, 0, 0, 255), 2.5f);
            dl->AddLine(ImVec2(mpos.x + ch_gap, mpos.y), ImVec2(mpos.x + ch_len, mpos.y), IM_COL32(0, 0, 0, 255), 2.5f);
            dl->AddLine(ImVec2(mpos.x, mpos.y - ch_len), ImVec2(mpos.x, mpos.y - ch_gap), IM_COL32(0, 0, 0, 255), 2.5f);
            dl->AddLine(ImVec2(mpos.x, mpos.y + ch_gap), ImVec2(mpos.x, mpos.y + ch_len), IM_COL32(0, 0, 0, 255), 2.5f);
            // Crisp cyan crosshair
            dl->AddLine(ImVec2(mpos.x - ch_len, mpos.y), ImVec2(mpos.x - ch_gap, mpos.y), IM_COL32(0, 245, 255, 255), 1.2f);
            dl->AddLine(ImVec2(mpos.x + ch_gap, mpos.y), ImVec2(mpos.x + ch_len, mpos.y), IM_COL32(0, 245, 255, 255), 1.2f);
            dl->AddLine(ImVec2(mpos.x, mpos.y - ch_len), ImVec2(mpos.x, mpos.y - ch_gap), IM_COL32(0, 245, 255, 255), 1.2f);
            dl->AddLine(ImVec2(mpos.x, mpos.y + ch_gap), ImVec2(mpos.x, mpos.y + ch_len), IM_COL32(0, 245, 255, 255), 1.2f);
            // Center indicator dot
            dl->AddCircleFilled(mpos, 1.2f, IM_COL32(0, 245, 255, 255));
        }
    }

    dl->PopClipRect();

    // 6. Floating Top Toolbar HUD hovering directly over the top of the canvas
    ImGui::SetCursorScreenPos(tb_p0);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.07f, 0.10f, kOverlayBgAlpha));
    ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.16f, 0.20f, 0.27f, 0.95f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, tb_frame_padding);

    if (ImGui::BeginChild("##ViewportTopHud", ImVec2(tb_w, tb_h), true,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
    {
        // 2px clearance above and below the action components
        ImGui::SetCursorPosY(2.0f);

        ImGui::BeginDisabled(in_cam_overlay || in_cam_calibration);
        if (ImGui::Button("Fit View")) {
            initial_fit_done_ = false;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button(isometric_3d_ ? "View: 3D" : "View: 2D")) {
            setIsometric3D(!isometric_3d_);
            if (on_view_changed_) on_view_changed_();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Switch between 3D Isometric (RMB drag to rotate) and 2D Top (XY) view");
        }
        if (has_top_down_cam && !isometric_3d_) {
            ImGui::SameLine();
            if (in_cam_calibration) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.45f, 0.08f, 0.95f));
                if (ImGui::Button("Exit Cam Calib")) {
                    finishCameraCalibration(cam_cfg.is_calibrated);
                }
                ImGui::PopStyleColor();
            } else {
                ImGui::BeginDisabled(!cam_cfg.is_calibrated);
                bool active_ov = camera_overlay_enabled_;
                if (active_ov) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.05f, 0.55f, 0.72f, 0.95f));
                }
                if (ImGui::Button(active_ov ? "Cam Overlay: ON" : "Cam Overlay: OFF")) {
                    setCameraOverlayEnabled(!active_ov);
                    if (active_ov) {
                        initial_fit_done_ = false; // Re-fit toolpath when leaving camera lock
                    }
                }
                if (active_ov) {
                    ImGui::PopStyleColor();
                }
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                    if (!cam_cfg.is_calibrated) {
                        ImGui::SetTooltip("Calibrate the Top-Down Camera first in the CAMERA tab to unlock 2D G-Code Overlay.");
                    } else {
                        ImGui::SetTooltip("Overlay 2D G-Code toolpath on live Top-Down Camera feed (locks pan & zoom to camera FOV)");
                    }
                }
            }
        }
        if (isometric_3d_) {
            ImGui::SameLine();
            if (ImGui::Button("Reset View")) {
                resetView();
                if (on_view_changed_) on_view_changed_();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Fit View + 3D angle Y:+15 P:+40");
            }
        }

        // Distinct spacing between the View buttons and the checkboxes
        ImGui::SameLine(0.0f, 16.0f);
        if (ImGui::Checkbox("Rapids (G0)", &show_rapids_)) {
            if (on_view_changed_) on_view_changed_();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Pierce Points", &show_pierces_)) {
            if (on_view_changed_) on_view_changed_();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox(follow_tool_label_.c_str(), &follow_torch_)) {
            if (on_view_changed_) on_view_changed_();
        }

        // Detail + tool selector are aligned to the right edge; the surface status goes between, when it fits
        const auto& tools_in_file = path_cache_.tools_in_file;
        const bool  show_tool_combo = tools_in_file.size() > 1;
        const float combo_w   = 125.0f;
        const float group_gap = 16.0f;
        const float detail_w  = ImGui::CalcTextSize("Detail").x + ImGui::GetStyle().FramePadding.x * 2.0f;
        const float group_w   = detail_w + (show_tool_combo ? group_gap + combo_w : 0.0f);
        const float after_checkboxes_x = ImGui::GetItemRectMax().x - ImGui::GetWindowPos().x + group_gap;
        const float right_group_x = std::max(after_checkboxes_x, ImGui::GetWindowWidth() - ImGui::GetStyle().WindowPadding.x - group_w);

        char surface_txt[64] = "";
        ImVec4 surface_col(0.0f, 0.90f, 0.46f, 1.0f);
        if (has_pending_surface_change_) {
            std::snprintf(surface_txt, sizeof(surface_txt), "[SURFACE PREVIEW: %.0f%% (UNAPPLIED)]", proposed_surface_smoothness_ * 100.0f);
            surface_col = ImVec4(1.0f, 0.75f, 0.20f, 1.0f);
        } else if (telemetry.surface_compensation_active) {
            std::snprintf(surface_txt, sizeof(surface_txt), "[SURFACE COMPENSATED: %.0f%%]", telemetry.surface_height_map->smoothness * 100.0f);
        }
        if (surface_txt[0] != '\0' && after_checkboxes_x + ImGui::CalcTextSize(surface_txt).x + group_gap <= right_group_x) {
            ImGui::SameLine(0.0f, group_gap);
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(surface_col, "%s", surface_txt);
        }

        ImGui::SameLine(right_group_x);
        if (ImGui::Button("Detail")) {
            ImGui::OpenPopup("##ToolpathDetailPopup");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Toolpath drawing accuracy (lower it if the view is slow on large programs)");
        }
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));
        if (ImGui::BeginPopup("##ToolpathDetailPopup")) {
            const float popup_w = 280.0f;
            ImGui::TextUnformatted("Toolpath drawing tolerance");
            ImGui::SetNextItemWidth(popup_w);
            float px = simplify_px_;
            if (ImGui::SliderFloat("##SimplifyPx", &px, kMinSimplifyPx, kMaxSimplifyPx, "%.1f px")) {
                setSimplifyPx(px);
                if (on_view_changed_) on_view_changed_();
            }
            ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + popup_w);
            ImGui::TextDisabled("Short connected moves are drawn as one line while they stay within this many pixels "
                                "of it. Higher values draw large programs faster on slow PCs; curves look slightly "
                                "more angular. Only the display changes, never the machine motion.");
            ImGui::PopTextWrapPos();
            ImGui::EndPopup();
        }
        ImGui::PopStyleVar(2);

        // Multi-tool filter selector (if program contains more than 1 tool)
        if (show_tool_combo) {
            ImGui::SameLine(0.0f, group_gap);
            ImGui::SetNextItemWidth(combo_w);
            char combo_preview[64];
            if (selected_tool_filter_ == 0) {
                std::snprintf(combo_preview, sizeof(combo_preview), "Tool: All (%zu)", tools_in_file.size());
            } else {
                std::snprintf(combo_preview, sizeof(combo_preview), "Tool: T%u", selected_tool_filter_);
            }
            if (ImGui::BeginCombo("##ToolFilter", combo_preview)) {
                if (ImGui::Selectable("Tool: All", selected_tool_filter_ == 0)) {
                    selected_tool_filter_ = 0;
                }
                for (uint16_t tn : tools_in_file) {
                    char label[64];
                    std::snprintf(label, sizeof(label), "Tool: T%u%s", tn, (tn == telemetry.current_tool_number) ? " (Active)" : "");
                    if (ImGui::Selectable(label, selected_tool_filter_ == tn)) {
                        selected_tool_filter_ = tn;
                    }
                }
                ImGui::EndCombo();
            }
        }
    }
    ImGui::EndChild();
    ImGui::PopStyleVar(5);
    ImGui::PopStyleColor(2);

    // Floating Alarm / Emergency Stop overlay HUD hovering directly over the top of the canvas
    if (telemetry.state == core::ControllerState::EStop && controller != nullptr) {
        const ImVec2 banner_pos(canvas_p0.x + 8.0f, canvas_p0.y + tb_h + 34.0f);
        const float  banner_w  = canvas_sz.x - 16.0f;
        IoStatusBar::renderFloatingAlarmBanner(*controller, telemetry, banner_pos, banner_w);
    }
}

} // namespace rnr::ui
