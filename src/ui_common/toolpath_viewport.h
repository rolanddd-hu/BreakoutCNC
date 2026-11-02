#pragma once
#include "imgui.h"
#include "core/controller/motion_controller.h"
#include "core/camera/camera_manager.h"
#include <algorithm>
#include <vector>
#include <array>
#include <functional>

namespace rnr::ui {

class ToolpathViewport {
public:
    ToolpathViewport() = default;
    ~ToolpathViewport();

    // Fit camera to given bounding box in mm
    void fitToBounds(const std::array<double, 3>& bbox_min_mm, const std::array<double, 3>& bbox_max_mm);

    // Render the interactive toolpath viewport inside the current ImGui window/child
    void render(
        const std::vector<core::ToolpathSegment>& segments,
        const core::TelemetrySnapshot& telemetry,
        const std::array<double, 3>& bbox_min_mm,
        const std::array<double, 3>& bbox_max_mm,
        core::IMotionController* controller = nullptr);

    void setIsometric3D(bool iso) noexcept {
        isometric_3d_ = iso;
        if (iso) {
            camera_overlay_enabled_ = false;
            camera_calibrating_     = false;
            waiting_for_dent_click_ = false;
        }
    }
    bool isIsometric3D() const noexcept { return isometric_3d_; }

    // Top-Down Position-by-Camera Integration (2D View Only)
    void setCameraManager(core::CameraManager* mgr) noexcept { camera_mgr_ = mgr; }
    core::CameraManager* getCameraManager() const noexcept { return camera_mgr_; }

    void setCameraOverlayEnabled(bool enabled) noexcept {
        camera_overlay_enabled_ = enabled;
        if (enabled) {
            isometric_3d_ = false; // Camera overlay strictly works in 2D view
        }
    }
    bool isCameraOverlayEnabled() const noexcept { return camera_overlay_enabled_; }

    void startCameraCalibration() noexcept {
        camera_calibrating_     = true;
        waiting_for_dent_click_ = false;
        isometric_3d_           = false;
    }
    void finishCameraCalibration(bool switch_to_overlay = true) noexcept {
        camera_calibrating_     = false;
        waiting_for_dent_click_ = false;
        if (switch_to_overlay) {
            isometric_3d_           = false;
            camera_overlay_enabled_ = true;
        }
    }
    bool isCameraCalibrating() const noexcept { return camera_calibrating_; }

    void startWaitingForDentClick(std::function<void(double norm_dx, double norm_dy, int img_w, int img_h)> cb) {
        camera_calibrating_     = true;
        isometric_3d_           = false;
        waiting_for_dent_click_ = true;
        on_dent_clicked_        = std::move(cb);
    }
    void cancelWaitingForDentClick() noexcept { waiting_for_dent_click_ = false; }
    bool isWaitingForDentClick() const noexcept { return waiting_for_dent_click_; }

    void setShowRapids(bool v) noexcept { show_rapids_ = v; }
    bool getShowRapids() const noexcept { return show_rapids_; }

    void setShowPierces(bool v) noexcept { show_pierces_ = v; }
    bool getShowPierces() const noexcept { return show_pierces_; }

    void setFollowTorch(bool v) noexcept { follow_torch_ = v; }
    bool getFollowTorch() const noexcept { return follow_torch_; }

    void setFollowToolLabel(const std::string& label) { follow_tool_label_ = label; }

    // Toolpath drawing tolerance in screen pixels: chains of short segments are drawn as one line while they
    // stay within this distance of it. Larger = fewer lines, faster drawing, less exact curves.
    static constexpr float kMinSimplifyPx = 0.5f;
    static constexpr float kMaxSimplifyPx = 4.0f;
    void setSimplifyPx(float px) noexcept { simplify_px_ = std::clamp(px, kMinSimplifyPx, kMaxSimplifyPx); }
    float getSimplifyPx() const noexcept { return simplify_px_; }

    void setOnViewChanged(std::function<void()> cb) { on_view_changed_ = std::move(cb); }

    void setSelectedToolFilter(uint16_t tool) noexcept { selected_tool_filter_ = tool; }
    uint16_t getSelectedToolFilter() const noexcept { return selected_tool_filter_; }

    void reset3DAngle() noexcept {
        view_yaw_deg_   = 15.0f;
        view_pitch_deg_ = 40.0f;
    }

    // Predefined "home" view for the 3D viewport: Fit View + Yaw +15 deg / Pitch +40 deg
    void resetView() noexcept {
        view_yaw_deg_     = 15.0f;
        view_pitch_deg_   = 40.0f;
        initial_fit_done_ = false; // re-fit to part bounds on next frame
    }

    // Interactive Surface Probing Rectangle Selection in 2D Viewport
    void startSelectingSurfaceRect(std::function<void(double min_x, double max_x, double min_y, double max_y)> callback) {
        selecting_surface_rect_   = true;
        surface_rect_drag_active_ = false;
        isometric_3d_             = false; // Switch to 2D for top-down rectangle draw
        on_surface_rect_selected_ = std::move(callback);
    }
    void cancelSelectingSurfaceRect() {
        selecting_surface_rect_   = false;
        surface_rect_drag_active_ = false;
    }
    bool isSelectingSurfaceRect() const noexcept { return selecting_surface_rect_; }

    // Surface Height Map Proposed Smoothness & Pending Change Highlights
    void setProposedSurfaceSmoothness(float smoothness, bool has_pending_change) noexcept {
        proposed_surface_smoothness_ = smoothness;
        has_pending_surface_change_  = has_pending_change;
    }

private:
    // Toolpath geometry projected and simplified for the current view, without the pan offset (screen position =
    // cached position + offset). Rebuilt only when the program, zoom, 3D angle, tolerance or tool filter changes,
    // so panning, following the tool and job progress do not re-process every segment each frame.
    struct CachedCutLine {
        ImVec2   a, b;
        uint32_t first_seg = 0, last_seg = 0; // segment index range the merged line covers
        uint16_t tool      = 0;
    };
    struct CachedPoint {
        ImVec2   p;
        uint32_t seg = 0;
    };
    struct PathCache {
        bool        valid       = false;
        const void* seg_data    = nullptr;
        size_t      seg_count   = 0;
        double      fingerprint = 0.0;
        bool        iso         = false;
        float       yaw = 0.0f, pitch = 0.0f, zoom = 0.0f, tol = 0.0f;
        uint16_t    tool_filter = 0;
        bool        coarse      = false; // built from chords while the view was rotating / zooming
        // Compact copy of the program geometry, built once per program: a view change re-projects ~28 bytes
        // per segment instead of reading the ~200-byte segments (the rebuild is memory-bound)
        struct Geo {
            float    start[3];
            float    end[3];
            uint16_t tool;
            uint8_t  type;
        };
        std::vector<Geo> geo;
        bool             geo_valid = false;
        std::vector<CachedCutLine> cuts;
        std::vector<ImVec2>        rapids;  // start / end pairs
        std::vector<CachedPoint>   pierces;
        std::vector<uint16_t>      tools_in_file; // in order of first use, regardless of the filter
    };
    void updatePathCache(const std::vector<core::ToolpathSegment>& segments);
    ImVec2 projectNoPan(double x_mm, double y_mm, double z_mm) const;
    PathCache path_cache_;
    static constexpr size_t kCoarseMinSegments    = 150000; // smaller programs always rebuild at full detail
    static constexpr size_t kCoarseTargetSegments = 100000; // chords per rebuild while the view moves
    static constexpr double kInteractHoldS        = 0.2;    // full detail returns this long after the last move
    double interact_until_s_ = -1.0;

    float zoom_px_per_mm_   = 4.2f;
    float pan_x_mm_         = 65.0f;
    float pan_y_mm_         = 45.0f;
    float view_yaw_deg_     = 15.0f; // 3D turntable azimuth (degrees, rotated by Right Mouse drag)
    float view_pitch_deg_   = 40.0f; // 3D tilt elevation (degrees, rotated by Right Mouse drag)
    bool  isometric_3d_     = false;
    bool  show_rapids_      = true;
    bool  show_pierces_     = true;
    bool  follow_torch_     = false;
    float simplify_px_      = 1.0f;
    bool  initial_fit_done_ = false;
    std::string follow_tool_label_ = "Follow Torch";
    float proposed_surface_smoothness_ = 0.5f;
    bool  has_pending_surface_change_  = false;
    uint16_t    selected_tool_filter_ = 0; // 0 = All tools
    std::function<void()> on_view_changed_;

    // Surface rectangle selection mode
    bool   selecting_surface_rect_   = false;
    bool   surface_rect_drag_active_ = false;
    ImVec2 surface_rect_start_px_{0.0f, 0.0f};
    std::function<void(double, double, double, double)> on_surface_rect_selected_;

    // Top-Down Camera state & OpenGL texture
    core::CameraManager* camera_mgr_             = nullptr;
    bool                 camera_overlay_enabled_ = false;
    bool                 camera_calibrating_     = false;
    bool                 waiting_for_dent_click_ = false;
    uint32_t             cam_gl_tex_             = 0;
    int                  cam_tex_w_              = 0;
    int                  cam_tex_h_              = 0;
    uint64_t             cam_last_frame_seq_     = 0;
    std::function<void(double, double, int, int)> on_dent_clicked_;
};

} // namespace rnr::ui
