#pragma once
#include <cstdint>
#include <string>
#include <functional>
#include "core/diagnostics.h"

struct GLFWwindow;
struct ImFont;

namespace rnr::core {
enum class OperationMode : uint8_t;
}

namespace rnr::ui {

class AppWindow {
public:
    AppWindow(const std::string& title, int width = 1440, int height = 900);
    ~AppWindow();

    AppWindow(const AppWindow&) = delete;
    AppWindow& operator=(const AppWindow&) = delete;

    bool isValid() const noexcept { return window_ != nullptr; }

    // Runs the 60Hz main UI render loop until the window is closed
    int runLoop(const std::function<void()>& render_callback);

    // Get or restore window geometry for persistent settings
    void setWindowGeometry(int x, int y, int width, int height, bool maximized);
    void getWindowGeometry(int& out_x, int& out_y, int& out_width, int& out_height, bool& out_maximized) const;

    // Fullscreen / Windowed mode toggling
    bool isFullscreen() const noexcept { return is_fullscreen_; }
    void setFullscreen(bool fullscreen);
    void toggleFullscreen();

    // Layout mode (Full / Minimal) selector for title bar
    void setMinimalLayout(bool* minimal_layout, std::function<void(bool)> on_changed = nullptr) {
        minimal_layout_ptr_ = minimal_layout;
        on_minimal_layout_changed_ = std::move(on_changed);
    }

    // Operation mode (Plasma / Laser) selector for title bar
    void setOperationMode(core::OperationMode* operation_mode, std::function<void(core::OperationMode)> on_changed = nullptr) {
        operation_mode_ptr_ = operation_mode;
        on_operation_mode_changed_ = std::move(on_changed);
    }

    // Window close interception (e.g. prompt if cycle is currently running)
    void setCloseInterceptor(std::function<bool()> can_close_predicate,
                             std::function<void()> on_close_blocked);
    void setOnCloseCallback(std::function<void()> on_close) { on_close_callback_ = std::move(on_close); }
    void closeWindow() noexcept;

    const std::string& getTitle() const noexcept { return title_; }
    void setTitle(const std::string& title);

    static void applyIndustrialDarkTheme();

    // Digital / industrial monospace fonts for G-Code and DRO readouts
    static ImFont* getGCodeFont() noexcept { return s_gcode_font_; }
    static ImFont* getDroFont() noexcept { return s_dro_font_; }

private:
    void renderCustomTitleBar();
    void renderPerfOverlay(); // F12: frame time breakdown

    // F12 performance overlay statistics
    int    perf_frames_in_second_ = 0;
    double perf_second_start_s_   = 0.0;
    int    perf_fps_              = 0;
    bool   perf_overlay_visible_  = false;
    core::diag::Counter::Snapshot perf_read_wait_{}, perf_tick_hold_{}, perf_usb_write_{}; // last completed second

    GLFWwindow* window_ = nullptr;
    std::string title_;
    std::function<bool()> can_close_predicate_;
    std::function<void()> on_close_blocked_;
    std::function<void()> on_close_callback_;
    static ImFont* s_gcode_font_;
    static ImFont* s_dro_font_;
    static ImFont* s_badge_font_top_;
    static ImFont* s_badge_font_sub_;

    // Layout mode (Full / Minimal) selector in title bar
    bool* minimal_layout_ptr_ = nullptr;
    std::function<void(bool)> on_minimal_layout_changed_;

    // Operation mode (Plasma / Laser) selector in title bar
    core::OperationMode* operation_mode_ptr_ = nullptr;
    std::function<void(core::OperationMode)> on_operation_mode_changed_;

    // Fullscreen and window geometry tracking
    bool is_fullscreen_ = false;
    int saved_window_x_ = 100;
    int saved_window_y_ = 100;
    int saved_window_w_ = 1440;
    int saved_window_h_ = 900;
    bool saved_was_maximized_ = false;
};

} // namespace rnr::ui
