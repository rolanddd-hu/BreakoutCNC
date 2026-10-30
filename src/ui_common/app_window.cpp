#include "ui_common/app_window.h"
#include "ui_common/frame_stats.h"
#include "core/controller/motion_controller.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <dwmapi.h>
#include <windowsx.h>
#include <timeapi.h>
#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "dwmapi.lib")

namespace {
static WNDPROC s_original_wndproc = nullptr;
static HWND s_overlay_hwnd = nullptr;
static bool s_is_fullscreen = false;

static LRESULT CALLBACK OverlayWindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        RECT rc;
        GetClientRect(hwnd, &rc);

        // 1. Black background (treated as 100% transparent by LWA_COLORKEY)
        HBRUSH bg_brush = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
        FillRect(hdc, &rc, bg_brush);

        // 2. Crisp 2px cyan outline preview
        HBRUSH border_brush = CreateSolidBrush(RGB(0, 210, 255));
        FrameRect(hdc, &rc, border_brush);
        InflateRect(&rc, -1, -1);
        FrameRect(hdc, &rc, border_brush);
        DeleteObject(border_brush);

        // 3. Live dimensions pill badge in bottom-right corner
        int w = rc.right + 1;
        int h = rc.bottom + 1;
        if (w > 200 && h > 100) {
            wchar_t dim_text[32];
            swprintf_s(dim_text, L"%d x %d", w, h);

            SetBkMode(hdc, TRANSPARENT);
            SetTextColor(hdc, RGB(0, 230, 255));
            HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
            HFONT old_font = static_cast<HFONT>(SelectObject(hdc, font));

            SIZE text_sz;
            GetTextExtentPoint32W(hdc, dim_text, static_cast<int>(wcslen(dim_text)), &text_sz);

            const int pad_x = 8;
            const int pad_y = 4;
            RECT badge_rc = {
                rc.right - text_sz.cx - pad_x * 2 - 12,
                rc.bottom - text_sz.cy - pad_y * 2 - 12,
                rc.right - 12,
                rc.bottom - 12
            };

            HBRUSH badge_bg = CreateSolidBrush(RGB(18, 22, 28));
            HBRUSH badge_border = CreateSolidBrush(RGB(0, 180, 220));
            FillRect(hdc, &badge_rc, badge_bg);
            FrameRect(hdc, &badge_rc, badge_border);
            DeleteObject(badge_bg);
            DeleteObject(badge_border);

            DrawTextW(hdc, dim_text, -1, &badge_rc, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            SelectObject(hdc, old_font);
        }

        EndPaint(hwnd, &ps);
        return 0;
    }
    default:
        return DefWindowProcW(hwnd, uMsg, wParam, lParam);
    }
}

static void EnsureOverlayWindow(HWND parent_hwnd) {
    if (s_overlay_hwnd && IsWindow(s_overlay_hwnd)) {
        return;
    }

    HINSTANCE hinst = GetModuleHandle(nullptr);
    WNDCLASSEXW wc = { sizeof(WNDCLASSEXW) };
    wc.lpfnWndProc = OverlayWindowProc;
    wc.hInstance = hinst;
    wc.lpszClassName = L"BreakoutResizeOverlay";
    RegisterClassExW(&wc);

    s_overlay_hwnd = CreateWindowExW(
        WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST | WS_EX_TOOLWINDOW | WS_EX_NOACTIVATE,
        L"BreakoutResizeOverlay",
        L"",
        WS_POPUP,
        0, 0, 0, 0,
        parent_hwnd,
        nullptr,
        hinst,
        nullptr
    );

    if (s_overlay_hwnd) {
        SetLayeredWindowAttributes(s_overlay_hwnd, RGB(0, 0, 0), 0, LWA_COLORKEY);
    }
}

static LRESULT CALLBACK CustomWindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    if (uMsg == WM_NCCALCSIZE && wParam == TRUE) {
        if (IsZoomed(hwnd)) {
            // When maximized, adjust the client rect so it doesn't spill over by the 8px resize frame
            auto* params = reinterpret_cast<NCCALCSIZE_PARAMS*>(lParam);
            int frame_x = GetSystemMetrics(SM_CXSIZEFRAME) + GetSystemMetrics(SM_CXPADDEDBORDER);
            int frame_y = GetSystemMetrics(SM_CYSIZEFRAME) + GetSystemMetrics(SM_CXPADDEDBORDER);
            params->rgrc[0].left   += frame_x;
            params->rgrc[0].top    += frame_y;
            params->rgrc[0].right  -= frame_x;
            params->rgrc[0].bottom -= frame_y;
            return WVR_REDRAW;
        }
        // Returning 0 for un-maximized windows expands the client area over the entire window,
        // completely eliminating the Windows 10/11 1px white non-client top border line!
        return 0;
    }

    if (uMsg == WM_NCHITTEST) {
        if (!IsZoomed(hwnd) && !s_is_fullscreen) {
            POINT pt = { GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam) };
            RECT rc;
            GetWindowRect(hwnd, &rc);

            int border_x = GetSystemMetrics(SM_CXSIZEFRAME) + GetSystemMetrics(SM_CXPADDEDBORDER);
            int border_y = GetSystemMetrics(SM_CYSIZEFRAME) + GetSystemMetrics(SM_CXPADDEDBORDER);
            if (border_x < 8) border_x = 8;
            if (border_y < 8) border_y = 8;

            bool left   = (pt.x >= rc.left && pt.x < rc.left + border_x);
            bool right  = (pt.x < rc.right && pt.x >= rc.right - border_x);
            bool top    = (pt.y >= rc.top && pt.y < rc.top + border_y);
            bool bottom = (pt.y < rc.bottom && pt.y >= rc.bottom - border_y);

            if (top && left)     return HTTOPLEFT;
            if (top && right)    return HTTOPRIGHT;
            if (bottom && left)  return HTBOTTOMLEFT;
            if (bottom && right) return HTBOTTOMRIGHT;
            if (left)            return HTLEFT;
            if (right)           return HTRIGHT;
            if (bottom)          return HTBOTTOM;
            if (top) {
                // Keep caption buttons (fullscreen/min/max/close in right 190px) clickable without top resize hijacking
                if (pt.x < rc.right - 190) {
                    return HTTOP;
                }
            }
        }
    }

    if (uMsg == WM_SIZING) {
        LRESULT res = CallWindowProc(s_original_wndproc, hwnd, uMsg, wParam, lParam);
        RECT* pRect = reinterpret_cast<RECT*>(lParam);
        if (pRect) {
            EnsureOverlayWindow(hwnd);
            if (s_overlay_hwnd) {
                int w = pRect->right - pRect->left;
                int h = pRect->bottom - pRect->top;
                SetWindowPos(s_overlay_hwnd, HWND_TOPMOST,
                             pRect->left, pRect->top, w, h,
                             SWP_NOACTIVATE | SWP_SHOWWINDOW);
                InvalidateRect(s_overlay_hwnd, nullptr, TRUE);
                UpdateWindow(s_overlay_hwnd);
            }
        }
        return res ? res : TRUE;
    }

    if (uMsg == WM_EXITSIZEMOVE || uMsg == WM_CAPTURECHANGED) {
        if (s_overlay_hwnd && IsWindow(s_overlay_hwnd)) {
            ShowWindow(s_overlay_hwnd, SW_HIDE);
        }
    }

    if (uMsg == WM_GETMINMAXINFO) {
        LRESULT res = CallWindowProc(s_original_wndproc, hwnd, uMsg, wParam, lParam);
        auto* mmi = reinterpret_cast<MINMAXINFO*>(lParam);
        mmi->ptMinTrackSize.x = 960;
        mmi->ptMinTrackSize.y = 600;
        return res;
    }

    return CallWindowProc(s_original_wndproc, hwnd, uMsg, wParam, lParam);
}
} // namespace
#endif

namespace rnr::ui {

ImFont* AppWindow::s_gcode_font_     = nullptr;
ImFont* AppWindow::s_dro_font_       = nullptr;
ImFont* AppWindow::s_badge_font_top_ = nullptr;
ImFont* AppWindow::s_badge_font_sub_ = nullptr;

static void fitWindowToWorkArea(GLFWwindow* window);

AppWindow::AppWindow(const std::string& title, int width, int height)
    : title_(title), saved_window_w_(width), saved_window_h_(height)
{
    // Visible in a terminal (Linux / macOS) or the CI log when the window or OpenGL context cannot be created
    glfwSetErrorCallback([](int code, const char* description) {
        std::fprintf(stderr, "GLFW error %d: %s\n", code, description ? description : "");
    });
    if (!glfwInit()) {
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE); // macOS only creates 3.2+ core contexts forward-compatible
#endif
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_FALSE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    // Custom borderless window: replace default white OS title bar with sleek industrial header
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    // Created hidden: runLoop() shows it after the saved geometry is applied and the first frame is drawn
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

    window_ = glfwCreateWindow(width, height, title_.c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        return;
    }

    glfwSetWindowSizeLimits(window_, 960, 600, GLFW_DONT_CARE, GLFW_DONT_CARE);

#ifdef _WIN32
    HWND hwnd = glfwGetWin32Window(window_);
    if (hwnd) {
        // Subclass WndProc to eliminate the 1px white top border via WM_NCCALCSIZE
        s_original_wndproc = reinterpret_cast<WNDPROC>(
            SetWindowLongPtr(hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(CustomWindowProc)));

        // Apply dark mode and border attributes to DWM
        BOOL dark_mode = TRUE;
        DwmSetWindowAttribute(hwnd, 20 /* DWMWA_USE_IMMERSIVE_DARK_MODE */, &dark_mode, sizeof(dark_mode));
        DwmSetWindowAttribute(hwnd, 19 /* DWMWA_USE_IMMERSIVE_DARK_MODE_BEFORE_20H1 */, &dark_mode, sizeof(dark_mode));

        COLORREF border_none = 0xFFFFFFFE; // DWMWA_COLOR_NONE (turns off 1px DWM border)
        DwmSetWindowAttribute(hwnd, 34 /* DWMWA_BORDER_COLOR */, &border_none, sizeof(border_none));

        COLORREF dark_bg = RGB(16, 18, 24);
        DwmSetWindowAttribute(hwnd, 35 /* DWMWA_CAPTION_COLOR */, &dark_bg, sizeof(dark_bg));

        MARGINS margins = { 0, 0, 0, 0 };
        DwmExtendFrameIntoClientArea(hwnd, &margins);

        LONG_PTR style = GetWindowLongPtr(hwnd, GWL_STYLE);
        style |= WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX;
        SetWindowLongPtr(hwnd, GWL_STYLE, style);
        SetWindowPos(hwnd, nullptr, 0, 0, 0, 0,
                     SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_FRAMECHANGED);

        // Load application icon (embedded resource or disk)
        std::string ico_name = "breakout_cnc.ico";
        if (title_.find("Mill") != std::string::npos) {
            ico_name = "breakout_mill.ico";
        } else if (title_.find("Cut") != std::string::npos || title_.find("Plasma") != std::string::npos) {
            ico_name = "breakout_cut.ico";
        }

        // Query system metrics for DPI-accurate big and small icon resolutions
        const int big_w = GetSystemMetrics(SM_CXICON);
        const int big_h = GetSystemMetrics(SM_CYICON);
        const int sm_w  = GetSystemMetrics(SM_CXSMICON);
        const int sm_h  = GetSystemMetrics(SM_CYSMICON);

        HICON hIconBig = (HICON)LoadImageA(GetModuleHandleA(nullptr), MAKEINTRESOURCEA(1), IMAGE_ICON, big_w, big_h, LR_SHARED);
        HICON hIconSmall = (HICON)LoadImageA(GetModuleHandleA(nullptr), MAKEINTRESOURCEA(1), IMAGE_ICON, sm_w, sm_h, LR_SHARED);
        if (!hIconBig || !hIconSmall) {
            const std::filesystem::path search_paths[] = {
                std::filesystem::path("resources/icons") / ico_name,
                std::filesystem::path("../resources/icons") / ico_name,
                std::filesystem::current_path() / "resources/icons" / ico_name
            };
            for (const auto& p : search_paths) {
                if (std::filesystem::exists(p)) {
                    if (!hIconBig) {
                        hIconBig = (HICON)LoadImageA(nullptr, p.string().c_str(), IMAGE_ICON, big_w, big_h, LR_LOADFROMFILE);
                    }
                    if (!hIconSmall) {
                        hIconSmall = (HICON)LoadImageA(nullptr, p.string().c_str(), IMAGE_ICON, sm_w, sm_h, LR_LOADFROMFILE);
                    }
                    if (hIconBig && hIconSmall) break;
                }
            }
        }
        if (hIconBig) {
            SendMessageA(hwnd, WM_SETICON, ICON_BIG, reinterpret_cast<LPARAM>(hIconBig));
        }
        if (hIconSmall) {
            SendMessageA(hwnd, WM_SETICON, ICON_SMALL, reinterpret_cast<LPARAM>(hIconSmall));
        }
    }
#endif

    glfwMakeContextCurrent(window_);
    // No driver vsync wait: many OpenGL drivers busy-wait for the display refresh inside SwapBuffers, which keeps
    // a whole CPU core at 100% and starves the UI on 2-core laptops. runLoop() paces frames by sleeping instead
    // (the Windows compositor still presents without tearing in windowed and borderless mode).
    glfwSwapInterval(0);
#ifdef _WIN32
    timeBeginPeriod(1); // 1 ms sleep granularity for the frame limiter
#endif

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    // NOTE: ImGuiConfigFlags_NavEnableKeyboard intentionally NOT enabled: Arrow keys and PgUp/PgDn
    // are reserved for keyboard machine jogging (see DroAndJogPanel::handleKeyboardJog).
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // 1. Default UI Font (Segoe UI Semibold / Regular)
    const char* ui_font_path = "C:/Windows/Fonts/seguisb.ttf";
    if (!std::filesystem::exists(ui_font_path)) {
        ui_font_path = "C:/Windows/Fonts/segoeui.ttf";
    }
    if (std::filesystem::exists(ui_font_path)) {
        io.Fonts->AddFontFromFileTTF(ui_font_path, 16.5f);
    } else {
        io.Fonts->AddFontDefault();
    }

    // 2. High-DPI Sharp Badge Fonts (Segoe UI Bold baked at exact native target pixel sizes)
    const char* bold_font_path = "C:/Windows/Fonts/segoeuib.ttf";
    if (!std::filesystem::exists(bold_font_path)) {
        bold_font_path = ui_font_path;
    }
    if (std::filesystem::exists(bold_font_path)) {
        s_badge_font_top_ = io.Fonts->AddFontFromFileTTF(bold_font_path, 10.5f);
        s_badge_font_sub_ = io.Fonts->AddFontFromFileTTF(bold_font_path, 12.5f);
    }

    // 3. Industrial / Digital Monospace Font for G-Code & DRO (Consolas Bold / OCR-A / CascadiaMono)
    const char* digital_font_path = "C:/Windows/Fonts/consolab.ttf";
    if (!std::filesystem::exists(digital_font_path)) {
        digital_font_path = "C:/Windows/Fonts/CascadiaMono.ttf";
    }
    if (!std::filesystem::exists(digital_font_path)) {
        digital_font_path = "C:/Windows/Fonts/consola.ttf";
    }
    if (std::filesystem::exists(digital_font_path)) {
        s_gcode_font_ = io.Fonts->AddFontFromFileTTF(digital_font_path, 16.0f);
        s_dro_font_   = io.Fonts->AddFontFromFileTTF(digital_font_path, 26.0f);
    }

    applyIndustrialDarkTheme();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    glfwSetWindowUserPointer(window_, this);
    glfwSetWindowCloseCallback(window_, [](GLFWwindow* win) {
        auto* self = static_cast<AppWindow*>(glfwGetWindowUserPointer(win));
        if (self && self->can_close_predicate_) {
            if (!self->can_close_predicate_()) {
                // Cancel closing window and trigger confirmation popup
                glfwSetWindowShouldClose(win, GLFW_FALSE);
                if (self->on_close_blocked_) {
                    self->on_close_blocked_();
                }
            }
        }
    });
    glfwSetDropCallback(window_, [](GLFWwindow* win, int count, const char** paths) {
        auto* self = static_cast<AppWindow*>(glfwGetWindowUserPointer(win));
        if (!self || !self->on_file_dropped_ || count < 1 || !paths[0]) return;
        std::string path = paths[0]; // Only one program can be loaded: the first file wins
#ifdef _WIN32
        // GLFW gives UTF-8; std::ifstream(std::string) and the file dialog use the ANSI code page
        const int wlen = MultiByteToWideChar(CP_UTF8, 0, paths[0], -1, nullptr, 0);
        if (wlen > 0) {
            std::wstring wide(static_cast<size_t>(wlen), L'\0');
            MultiByteToWideChar(CP_UTF8, 0, paths[0], -1, wide.data(), wlen);
            const int alen = WideCharToMultiByte(CP_ACP, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
            if (alen > 0) {
                std::string ansi(static_cast<size_t>(alen), '\0');
                WideCharToMultiByte(CP_ACP, 0, wide.c_str(), -1, ansi.data(), alen, nullptr, nullptr);
                ansi.resize(static_cast<size_t>(alen) - 1);
                path = std::move(ansi);
            }
        }
#endif
        self->on_file_dropped_(path);
    });
}

AppWindow::~AppWindow() {
    if (window_) {
#ifdef _WIN32
        timeEndPeriod(1);
        if (s_overlay_hwnd && IsWindow(s_overlay_hwnd)) {
            DestroyWindow(s_overlay_hwnd);
            s_overlay_hwnd = nullptr;
        }
        HWND hwnd = glfwGetWin32Window(window_);
        if (hwnd && s_original_wndproc) {
            SetWindowLongPtr(hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(s_original_wndproc));
            s_original_wndproc = nullptr;
        }
#endif
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window_);
        glfwTerminate();
        window_ = nullptr;
    }
}

void AppWindow::renderPerfOverlay() {
    const auto& reg = perf::registry();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + vp->Size.x - 12.0f, vp->Pos.y + 44.0f), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(400.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.92f);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                                   ImGuiWindowFlags_AlwaysAutoResize;
    if (ImGui::Begin("##PerfOverlay", nullptr, flags)) {
        const ImGuiIO& io = ImGui::GetIO();
        ImGui::TextColored(ImVec4(0.40f, 0.75f, 1.0f, 1.0f), "PERFORMANCE (F12 to close)");
        ImGui::Text("%d frames in the last second", perf_fps_);
        ImGui::Text("Vertices: %d   Indices: %d   Windows: %d", io.MetricsRenderVertices, io.MetricsRenderIndices,
                    io.MetricsRenderWindows);
        ImGui::Spacing();
        if (ImGui::BeginTable("##PerfTable", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
            ImGui::TableSetupColumn("Section", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("avg ms", ImGuiTableColumnFlags_WidthFixed, 64.0f);
            ImGui::TableSetupColumn("worst ms", ImGuiTableColumnFlags_WidthFixed, 72.0f);
            ImGui::TableHeadersRow();
            for (size_t i = 0; i < reg.count; ++i) {
                const auto& s = reg.sections[i];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(s.name);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", s.avg_ms);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.2f", s.max_ms);
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        if (ImGui::BeginTable("##PerfCoreTable", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
            ImGui::TableSetupColumn("Controller / USB", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("per s", ImGuiTableColumnFlags_WidthFixed, 48.0f);
            ImGui::TableSetupColumn("avg ms", ImGuiTableColumnFlags_WidthFixed, 56.0f);
            ImGui::TableSetupColumn("worst ms", ImGuiTableColumnFlags_WidthFixed, 64.0f);
            ImGui::TableHeadersRow();
            const struct { const char* name; const core::diag::Counter::Snapshot* s; } rows[] = {
                {"Waiting for controller (reads)", &perf_read_wait_},
                {"Controller tick holds lock", &perf_tick_hold_},
                {"USB writes to board", &perf_usb_write_},
            };
            for (const auto& r : rows) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(r.name);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%llu", static_cast<unsigned long long>(r.s->count));
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.2f", r.s->avg_ms);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.2f", r.s->max_ms);
            }
            ImGui::EndTable();
        }
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Averages and the worst frame over the last second. \"Whole frame (busy)\" is the work per "
                            "frame; at 60 FPS it must stay below 16.7 ms. \"Idle\" is the time the app sleeps until "
                            "the next frame. \"GPU draw\" waits for the GPU while this overlay is open.");
        ImGui::PopTextWrapPos();
    }
    ImGui::End();
}

int AppWindow::runLoop(const std::function<void()>& render_callback) {
    if (!window_) return 1;

    using PerfClock = std::chrono::steady_clock;
    auto msSince = [](PerfClock::time_point t) {
        return std::chrono::duration<double, std::milli>(PerfClock::now() - t).count();
    };
    auto& perf_reg = perf::registry();
    // BREAKOUTCNC_PERF_LOG=<file>: also write the overlay's numbers once per second (for remote diagnosis)
    std::FILE* perf_log = nullptr;
    if (const char* log_path = std::getenv("BREAKOUTCNC_PERF_LOG"); log_path && *log_path) {
        perf_log = std::fopen(log_path, "w");
    }

    // Frame limiter: 60 FPS, 10 FPS while minimized
    constexpr auto kFramePeriod     = std::chrono::microseconds(16667);
    constexpr auto kIconifiedPeriod = std::chrono::milliseconds(100);
    auto next_frame = PerfClock::now();

    // The default or saved size may not fit this screen (e.g. settings copied from a larger monitor)
    if (!is_fullscreen_ && glfwGetWindowAttrib(window_, GLFW_MAXIMIZED) != GLFW_TRUE) {
        fitWindowToWorkArea(window_);
    }
    bool window_shown = false;

    while (!glfwWindowShouldClose(window_)) {
        const auto t_frame = PerfClock::now();
        glfwPollEvents();

        // Track normal window geometry when not fullscreen and not maximized or minimized
        if (!is_fullscreen_ &&
            glfwGetWindowAttrib(window_, GLFW_MAXIMIZED) != GLFW_TRUE &&
            glfwGetWindowAttrib(window_, GLFW_ICONIFIED) != GLFW_TRUE) {
            int cur_x = 0, cur_y = 0, cur_w = 0, cur_h = 0;
            glfwGetWindowPos(window_, &cur_x, &cur_y);
            glfwGetWindowSize(window_, &cur_w, &cur_h);
            if (cur_x > -10000 && cur_y > -10000 && cur_w >= 640 && cur_h >= 480) {
                saved_window_x_ = cur_x;
                saved_window_y_ = cur_y;
                saved_window_w_ = cur_w;
                saved_window_h_ = cur_h;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // F11 shortcut to toggle fullscreen
        if (ImGui::IsKeyPressed(ImGuiKey_F11, false)) {
            toggleFullscreen();
        }
        // F12: performance overlay
        if (ImGui::IsKeyPressed(ImGuiKey_F12, false)) {
            perf_overlay_visible_ = !perf_overlay_visible_;
        }
        perf_reg.enabled = perf_overlay_visible_ || perf_log != nullptr;
        perf::add("Events + new frame", msSince(t_frame));
        const auto t_ui = PerfClock::now();

        // 1. Sleek Modern Custom Title Bar Header (edge-to-edge, height = 32px)
        renderCustomTitleBar();

        // 2. Fullscreen dockspace host (occupies remaining height from y = 32px to bottom)
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        const float title_bar_height = 32.0f;
        ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x, viewport->Pos.y + title_bar_height));
        ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, std::max(100.0f, viewport->Size.y - title_bar_height)));
        ImGui::SetNextWindowViewport(viewport->ID);

        ImGuiWindowFlags host_flags =
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoBackground |
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse; // whole app never scrolls, only panels

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.0f, 6.0f));
        ImGui::Begin("##MainDockHost", nullptr, host_flags);
        ImGui::PopStyleVar(3);

        if (render_callback) {
            render_callback();
        }

        ImGui::End();

        if (perf_overlay_visible_) {
            renderPerfOverlay();
        }
        perf::add("UI build (all panels)", msSince(t_ui));

        auto t_phase = PerfClock::now();
        ImGui::Render();
        perf::add("ImGui render (CPU)", msSince(t_phase));

        t_phase = PerfClock::now();
        int display_w = 0, display_h = 0;
        glfwGetFramebufferSize(window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.06f, 0.07f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (perf_reg.enabled) {
            glFinish(); // Only while measuring: separates the GPU's work from the vsync wait below
        }
        perf::add("GPU draw", msSince(t_phase));

        t_phase = PerfClock::now();
        glfwSwapBuffers(window_);
        perf::add("Swap", msSince(t_phase));
        if (!window_shown) {
            window_shown = true;
            glfwShowWindow(window_);
            glfwFocusWindow(window_);
        }

        perf::add("Whole frame (busy)", msSince(t_frame));

        // Sleep until the next frame slot; never try to catch up after a slow frame
        t_phase = PerfClock::now();
        next_frame += (glfwGetWindowAttrib(window_, GLFW_ICONIFIED) == GLFW_TRUE) ? kIconifiedPeriod : kFramePeriod;
        if (t_phase > next_frame + kFramePeriod) {
            next_frame = t_phase;
        }
        std::this_thread::sleep_until(next_frame);
        perf::add("Idle (frame limiter)", msSince(t_phase));
        if (perf_reg.enabled &&
            perf::endFrame(perf_frames_in_second_, perf_second_start_s_, glfwGetTime(), perf_fps_)) {
            perf_read_wait_ = core::diag::controllerReadWait().takeAndReset();
            perf_tick_hold_ = core::diag::workerTickLockHold().takeAndReset();
            perf_usb_write_ = core::diag::usbWrite().takeAndReset();
            if (perf_log) {
                std::fprintf(perf_log, "fps=%d vertices=%d", perf_fps_, ImGui::GetIO().MetricsRenderVertices);
                for (size_t i = 0; i < perf_reg.count; ++i) {
                    const auto& s = perf_reg.sections[i];
                    std::fprintf(perf_log, " | %s avg=%.2f max=%.2f", s.name, s.avg_ms, s.max_ms);
                }
                const struct { const char* name; const core::diag::Counter::Snapshot* s; } rows[] = {
                    {"read wait", &perf_read_wait_}, {"tick lock hold", &perf_tick_hold_}, {"usb write", &perf_usb_write_}};
                for (const auto& r : rows) {
                    std::fprintf(perf_log, " | %s n=%llu avg=%.2f max=%.2f", r.name,
                                 static_cast<unsigned long long>(r.s->count), r.s->avg_ms, r.s->max_ms);
                }
                std::fprintf(perf_log, "\n");
                std::fflush(perf_log);
            }
        }
    }
    if (perf_log) {
        std::fclose(perf_log);
    }

    if (on_close_callback_) {
        on_close_callback_();
    }

    return 0;
}

void AppWindow::setTitle(const std::string& title) {
    title_ = title;
    if (window_) {
        glfwSetWindowTitle(window_, title_.c_str());
    }
}

void AppWindow::renderCustomTitleBar() {
    if (!window_) return;

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float title_bar_h = 32.0f;

    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, title_bar_h));
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollWithMouse |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.065f, 0.075f, 0.095f, 1.0f));

    if (ImGui::Begin("##CustomTitleBar", nullptr, flags)) {
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        const ImVec2 pos = ImGui::GetWindowPos();
        const ImVec2 size = ImGui::GetWindowSize();
        const float right_x = pos.x + size.x;
        const float bottom_y = pos.y + size.y;

        // Bottom separator border (subtle industrial line)
        draw_list->AddLine(
            ImVec2(pos.x, bottom_y - 1.0f),
            ImVec2(right_x, bottom_y - 1.0f),
            IM_COL32(38, 44, 56, 255),
            1.0f);

        // Control buttons geometry (fullscreen: 46px, minimize: 46px, maximize: 46px, close: 48px)
        const float btn_w = 46.0f;
        const float close_w = 48.0f;
        const float buttons_total_w = btn_w * 3.0f + close_w;

        // Left side: Icon badge and Title text
        const float text_start_x = pos.x + 10.0f;
        const float text_y = pos.y + 7.5f;

        // Breakout Logo Badge matching high-resolution industrial icon design
        const float badge_w = 76.0f;
        const float badge_h = 24.0f;
        const ImVec2 badge_p1(text_start_x, pos.y + 4.0f);
        const ImVec2 badge_p2(text_start_x + badge_w, pos.y + 4.0f + badge_h);
        draw_list->AddRectFilled(badge_p1, badge_p2, IM_COL32(4, 10, 18, 255), 4.0f);
        draw_list->AddRect(badge_p1, badge_p2, IM_COL32(0, 245, 255, 255), 4.0f, 0, 1.2f);

        std::string badge_sub = "CNC";
        if (title_.find("Mill") != std::string::npos) {
            badge_sub = "MILL";
        } else if (title_.find("Cut") != std::string::npos || title_.find("Plasma") != std::string::npos) {
            badge_sub = "CUT";
        }

        const char* top_txt = "BREAKOUT";
        ImFont* top_font = s_badge_font_top_ ? s_badge_font_top_ : ImGui::GetFont();
        ImFont* sub_font = s_badge_font_sub_ ? s_badge_font_sub_ : ImGui::GetFont();

        const float sz_top = 10.5f;
        const float sz_sub = 12.5f;

        ImVec2 top_sz = top_font->CalcTextSizeA(sz_top, 200.0f, 0.0f, top_txt);
        ImVec2 sub_sz = sub_font->CalcTextSizeA(sz_sub, 200.0f, 0.0f, badge_sub.c_str());

        float top_x = badge_p1.x + (badge_w - top_sz.x) * 0.5f;
        float sub_x = badge_p1.x + (badge_w - sub_sz.x) * 0.5f;

        draw_list->AddText(top_font, sz_top, ImVec2(top_x, badge_p1.y + 1.0f), IM_COL32(245, 250, 255, 245), top_txt);
        draw_list->AddText(sub_font, sz_sub, ImVec2(sub_x, badge_p1.y + 11.5f), IM_COL32(0, 245, 255, 255), badge_sub.c_str());

        // Title string
        const float title_x = text_start_x + badge_w + 10.0f;
        draw_list->AddText(ImVec2(title_x, text_y), IM_COL32(220, 228, 238, 255), title_.c_str());
        const float title_end_x = title_x + ImGui::CalcTextSize(title_.c_str()).x;

        // Custom Title Bar Segmented Toggles: [ PLASMA | LASER ] and [ FULL | MINIMAL ]
        const float buttons_start_x = right_x - buttons_total_w;
        const float tab_w    = 76.0f;
        const float tab_h    = 22.0f;
        const float tab_gap  = 12.0f;

        const float mode_w   = (operation_mode_ptr_ != nullptr) ? (tab_w * 2.0f) : 0.0f;
        const float layout_w = (minimal_layout_ptr_ != nullptr) ? (tab_w * 2.0f) : 0.0f;
        const float total_groups_w = mode_w + ((mode_w > 0.0f && layout_w > 0.0f) ? tab_gap : 0.0f) + layout_w;

        const float group_y  = pos.y + (title_bar_h - tab_h) * 0.5f;
        float start_x        = pos.x + (size.x - total_groups_w) * 0.5f;

        if (start_x < title_end_x + 16.0f) {
            start_x = title_end_x + 16.0f;
        }
        if (start_x + total_groups_w > buttons_start_x - 16.0f) {
            start_x = buttons_start_x - total_groups_w - 16.0f;
        }

        // Draggable zones helper lambda
        auto handleDragZone = [&](const char* id, float x1, float x2) {
            float w = x2 - x1;
            if (w < 8.0f) return;
            ImGui::SetCursorScreenPos(ImVec2(x1, pos.y));
            ImGui::InvisibleButton(id, ImVec2(w, title_bar_h));

#ifdef _WIN32
            if (!is_fullscreen_) {
                if (ImGui::IsItemActive() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    HWND hwnd = glfwGetWin32Window(window_);
                    if (hwnd) {
                        ReleaseCapture();
                        SendMessage(hwnd, WM_NCLBUTTONDOWN, HTCAPTION, 0);
                    }
                } else if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    if (glfwGetWindowAttrib(window_, GLFW_MAXIMIZED)) {
                        glfwRestoreWindow(window_);
                    } else {
                        glfwMaximizeWindow(window_);
                    }
                }
            }
#else
            if (!is_fullscreen_) {
                static double s_drag_start_x = 0.0, s_drag_start_y = 0.0;
                static int s_win_start_x = 0, s_win_start_y = 0;
                if (ImGui::IsItemActive()) {
                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                        glfwGetCursorPos(window_, &s_drag_start_x, &s_drag_start_y);
                        glfwGetWindowPos(window_, &s_win_start_x, &s_win_start_y);
                    } else if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 1.0f)) {
                        double cur_x = 0.0, cur_y = 0.0;
                        glfwGetCursorPos(window_, &cur_x, &cur_y);
                        int new_x = s_win_start_x + static_cast<int>(cur_x - s_drag_start_x);
                        int new_y = s_win_start_y + static_cast<int>(cur_y - s_drag_start_y);
                        glfwSetWindowPos(window_, new_x, new_y);
                    }
                } else if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    if (glfwGetWindowAttrib(window_, GLFW_MAXIMIZED)) {
                        glfwRestoreWindow(window_);
                    } else {
                        glfwMaximizeWindow(window_);
                    }
                }
            }
#endif
        };

        if (total_groups_w > 0.0f) {
            handleDragZone("##TitleBarDragZone1", title_end_x + 8.0f, start_x - 6.0f);
            float cur_group_x = start_x;

            // 1. Render Segmented [ PLASMA | LASER ] Mode Toggle
            if (operation_mode_ptr_ != nullptr) {
                draw_list->AddRectFilled(ImVec2(cur_group_x, group_y), ImVec2(cur_group_x + mode_w, group_y + tab_h), IM_COL32(18, 22, 28, 255), 4.0f);
                draw_list->AddRect(ImVec2(cur_group_x, group_y), ImVec2(cur_group_x + mode_w, group_y + tab_h), IM_COL32(45, 55, 70, 255), 4.0f, 0, 1.0f);

                const char* mode_names[2] = { "PLASMA", "LASER" };
                for (int m = 0; m < 2; ++m) {
                    const float tab_x = cur_group_x + m * tab_w;
                    const bool is_selected = (m == 0 && *operation_mode_ptr_ == core::OperationMode::Plasma) ||
                                            (m == 1 && *operation_mode_ptr_ == core::OperationMode::Laser);

                    ImGui::SetCursorScreenPos(ImVec2(tab_x, group_y));
                    char btn_id[32];
                    std::snprintf(btn_id, sizeof(btn_id), "##TitleModeTab_%d", m);
                    bool clicked = ImGui::InvisibleButton(btn_id, ImVec2(tab_w, tab_h));
                    bool hovered = ImGui::IsItemHovered();

                    if (hovered) {
                        ImGui::SetTooltip(m == 0 ? "Switch to CNC Plasma Mode (THC, Arc OK, Ohmic/Floating Head IHS)"
                                                 : "Switch to CNC Laser Mode (PWM Modulation, Dynamic M4, Air Assist)");
                    }

                    if (clicked && !is_selected) {
                        *operation_mode_ptr_ = (m == 0) ? core::OperationMode::Plasma : core::OperationMode::Laser;
                        if (on_operation_mode_changed_) {
                            on_operation_mode_changed_(*operation_mode_ptr_);
                        }
                    }

                    if (is_selected) {
                        // Plasma = Electric Blue; Laser = Ruby Crimson
                        ImU32 fill_col = (m == 0) ? IM_COL32(0, 140, 200, 255) : IM_COL32(230, 30, 80, 255);
                        draw_list->AddRectFilled(
                            ImVec2(tab_x + 1.0f, group_y + 1.0f),
                            ImVec2(tab_x + tab_w - 1.0f, group_y + tab_h - 1.0f),
                            fill_col, 3.0f);
                    } else if (hovered) {
                        draw_list->AddRectFilled(
                            ImVec2(tab_x + 1.0f, group_y + 1.0f),
                            ImVec2(tab_x + tab_w - 1.0f, group_y + tab_h - 1.0f),
                            IM_COL32(36, 44, 56, 255), 3.0f);
                    }

                    ImVec2 tsz = ImGui::CalcTextSize(mode_names[m]);
                    float tx = tab_x + (tab_w - tsz.x) * 0.5f;
                    float ty = group_y + (tab_h - tsz.y) * 0.5f;
                    draw_list->AddText(
                        ImVec2(tx, ty),
                        is_selected ? IM_COL32(255, 255, 255, 255) : (hovered ? IM_COL32(210, 225, 240, 255) : IM_COL32(150, 165, 185, 255)),
                        mode_names[m]);
                }

                cur_group_x += mode_w;
                if (layout_w > 0.0f) {
                    handleDragZone("##TitleBarDragZoneMid", cur_group_x + 1.0f, cur_group_x + tab_gap - 1.0f);
                    cur_group_x += tab_gap;
                }
            }

            // 2. Render Segmented [ FULL | MINIMAL ] Layout Toggle
            if (minimal_layout_ptr_ != nullptr) {
                draw_list->AddRectFilled(ImVec2(cur_group_x, group_y), ImVec2(cur_group_x + layout_w, group_y + tab_h), IM_COL32(18, 22, 28, 255), 4.0f);
                draw_list->AddRect(ImVec2(cur_group_x, group_y), ImVec2(cur_group_x + layout_w, group_y + tab_h), IM_COL32(45, 55, 70, 255), 4.0f, 0, 1.0f);

                const char* tab_names[2] = { "FULL", "MINIMAL" };
                for (int t = 0; t < 2; ++t) {
                    const float tab_x = cur_group_x + t * tab_w;
                    const bool is_selected = ((t == 1) == *minimal_layout_ptr_);

                    ImGui::SetCursorScreenPos(ImVec2(tab_x, group_y));
                    char btn_id[32];
                    std::snprintf(btn_id, sizeof(btn_id), "##TitleTab_%d", t);
                    bool clicked = ImGui::InvisibleButton(btn_id, ImVec2(tab_w, tab_h));
                    bool hovered = ImGui::IsItemHovered();

                    if (hovered) {
                        ImGui::SetTooltip(t == 0 ? "Switch to Full Layout (All Controls & Diagnostics)"
                                                 : "Switch to Minimal Layout (Maximized Toolpath Viewport)");
                    }

                    if (clicked && !is_selected) {
                        *minimal_layout_ptr_ = (t == 1);
                        if (on_minimal_layout_changed_) {
                            on_minimal_layout_changed_(*minimal_layout_ptr_);
                        }
                    }

                    if (is_selected) {
                        draw_list->AddRectFilled(
                            ImVec2(tab_x + 1.0f, group_y + 1.0f),
                            ImVec2(tab_x + tab_w - 1.0f, group_y + tab_h - 1.0f),
                            IM_COL32(0, 130, 180, 255), 3.0f);
                    } else if (hovered) {
                        draw_list->AddRectFilled(
                            ImVec2(tab_x + 1.0f, group_y + 1.0f),
                            ImVec2(tab_x + tab_w - 1.0f, group_y + tab_h - 1.0f),
                            IM_COL32(36, 44, 56, 255), 3.0f);
                    }

                    ImVec2 tsz = ImGui::CalcTextSize(tab_names[t]);
                    float tx = tab_x + (tab_w - tsz.x) * 0.5f;
                    float ty = group_y + (tab_h - tsz.y) * 0.5f;
                    draw_list->AddText(
                        ImVec2(tx, ty),
                        is_selected ? IM_COL32(255, 255, 255, 255) : (hovered ? IM_COL32(210, 225, 240, 255) : IM_COL32(150, 165, 185, 255)),
                        tab_names[t]);
                }
                cur_group_x += layout_w;
            }

            handleDragZone("##TitleBarDragZone2", cur_group_x + 6.0f, buttons_start_x);
        } else {
            handleDragZone("##TitleBarDragZone", title_end_x + 8.0f, buttons_start_x);
        }

        // Helper lambda for modern title bar buttons
        auto renderButton = [&](const char* id, float x, float w, auto draw_icon, bool is_close, const char* tooltip = nullptr) -> bool {
            ImGui::SetCursorScreenPos(ImVec2(x, pos.y));
            bool pressed = ImGui::InvisibleButton(id, ImVec2(w, title_bar_h));
            bool hovered = ImGui::IsItemHovered();
            bool active = ImGui::IsItemActive();

            if (hovered && tooltip && tooltip[0] != '\0') {
                ImGui::SetTooltip("%s", tooltip);
            }

            ImU32 bg_col = 0;
            ImU32 icon_col = IM_COL32(200, 210, 220, 255);

            if (is_close) {
                if (active) {
                    bg_col = IM_COL32(196, 16, 30, 255);
                    icon_col = IM_COL32(255, 255, 255, 255);
                } else if (hovered) {
                    bg_col = IM_COL32(232, 17, 35, 255);
                    icon_col = IM_COL32(255, 255, 255, 255);
                }
            } else {
                if (active) {
                    bg_col = IM_COL32(255, 255, 255, 45);
                    icon_col = IM_COL32(255, 255, 255, 255);
                } else if (hovered) {
                    bg_col = IM_COL32(255, 255, 255, 25);
                    icon_col = IM_COL32(255, 255, 255, 255);
                }
            }

            if (bg_col != 0) {
                draw_list->AddRectFilled(ImVec2(x, pos.y), ImVec2(x + w, pos.y + title_bar_h), bg_col);
            }

            const float center_x = x + w * 0.5f;
            const float center_y = pos.y + title_bar_h * 0.5f;
            draw_icon(draw_list, center_x, center_y, icon_col);

            return pressed;
        };

        const float fs_x  = right_x - buttons_total_w;
        const float min_x = right_x - (btn_w * 2.0f + close_w);
        const float max_x = right_x - (btn_w + close_w);
        const float cls_x = right_x - close_w;

        // 1. Full Screen / Window Mode Button (positioned next to the minimize button)
        const char* fs_tooltip = is_fullscreen_ ? "Window Mode (F11)" : "Full Screen (F11)";
        if (renderButton("##TitleFullscreenBtn", fs_x, btn_w, [this](ImDrawList* dl, float cx, float cy, ImU32 col) {
            if (is_fullscreen_) {
                // Window mode icon: 4 inward-pointing corners
                const float s = 5.0f;
                // Top-Left (inward toward center)
                dl->AddLine(ImVec2(cx - 1.8f, cy - s), ImVec2(cx - 1.8f, cy - 1.8f), col, 1.4f);
                dl->AddLine(ImVec2(cx - s, cy - 1.8f), ImVec2(cx - 1.8f, cy - 1.8f), col, 1.4f);
                // Top-Right (inward toward center)
                dl->AddLine(ImVec2(cx + 1.8f, cy - s), ImVec2(cx + 1.8f, cy - 1.8f), col, 1.4f);
                dl->AddLine(ImVec2(cx + s, cy - 1.8f), ImVec2(cx + 1.8f, cy - 1.8f), col, 1.4f);
                // Bottom-Left (inward toward center)
                dl->AddLine(ImVec2(cx - 1.8f, cy + s), ImVec2(cx - 1.8f, cy + 1.8f), col, 1.4f);
                dl->AddLine(ImVec2(cx - s, cy + 1.8f), ImVec2(cx - 1.8f, cy + 1.8f), col, 1.4f);
                // Bottom-Right (inward toward center)
                dl->AddLine(ImVec2(cx + 1.8f, cy + s), ImVec2(cx + 1.8f, cy + 1.8f), col, 1.4f);
                dl->AddLine(ImVec2(cx + s, cy + 1.8f), ImVec2(cx + 1.8f, cy + 1.8f), col, 1.4f);
            } else {
                // Full screen icon: 4 outward-pointing corners
                const float s = 5.0f;
                const float a = 3.2f;
                // Top-Left corner
                dl->AddLine(ImVec2(cx - s, cy - s), ImVec2(cx - s + a, cy - s), col, 1.4f);
                dl->AddLine(ImVec2(cx - s, cy - s), ImVec2(cx - s, cy - s + a), col, 1.4f);
                // Top-Right corner
                dl->AddLine(ImVec2(cx + s, cy - s), ImVec2(cx + s - a, cy - s), col, 1.4f);
                dl->AddLine(ImVec2(cx + s, cy - s), ImVec2(cx + s, cy - s + a), col, 1.4f);
                // Bottom-Left corner
                dl->AddLine(ImVec2(cx - s, cy + s), ImVec2(cx - s + a, cy + s), col, 1.4f);
                dl->AddLine(ImVec2(cx - s, cy + s), ImVec2(cx - s, cy + s - a), col, 1.4f);
                // Bottom-Right corner
                dl->AddLine(ImVec2(cx + s, cy + s), ImVec2(cx + s - a, cy + s), col, 1.4f);
                dl->AddLine(ImVec2(cx + s, cy + s), ImVec2(cx + s, cy + s - a), col, 1.4f);
            }
        }, false, fs_tooltip)) {
            toggleFullscreen();
        }

        // 2. Minimize Button
        if (renderButton("##TitleMinBtn", min_x, btn_w, [](ImDrawList* dl, float cx, float cy, ImU32 col) {
            dl->AddLine(ImVec2(cx - 5.0f, cy), ImVec2(cx + 5.0f, cy), col, 1.4f);
        }, false, "Minimize")) {
            glfwIconifyWindow(window_);
        }

        // 3. Maximize / Restore Button
        const bool is_maximized = (glfwGetWindowAttrib(window_, GLFW_MAXIMIZED) == GLFW_TRUE);
        const char* max_tooltip = is_maximized ? "Restore Down" : "Maximize";
        if (renderButton("##TitleMaxBtn", max_x, btn_w, [is_maximized](ImDrawList* dl, float cx, float cy, ImU32 col) {
            if (is_maximized) {
                // Restore icon (overlapping squares)
                dl->AddRect(ImVec2(cx - 3.0f, cy - 5.0f), ImVec2(cx + 5.0f, cy + 3.0f), col, 0.0f, 0, 1.2f);
                dl->AddRectFilled(ImVec2(cx - 5.0f, cy - 3.0f), ImVec2(cx + 3.0f, cy + 5.0f), IM_COL32(16, 19, 24, 255));
                dl->AddRect(ImVec2(cx - 5.0f, cy - 3.0f), ImVec2(cx + 3.0f, cy + 5.0f), col, 0.0f, 0, 1.2f);
            } else {
                // Maximize icon (single square)
                dl->AddRect(ImVec2(cx - 5.0f, cy - 5.0f), ImVec2(cx + 5.0f, cy + 5.0f), col, 0.0f, 0, 1.4f);
            }
        }, false, max_tooltip)) {
            if (is_fullscreen_) {
                setFullscreen(false);
                if (!is_maximized) {
                    glfwMaximizeWindow(window_);
                }
            } else {
                if (is_maximized) {
                    glfwRestoreWindow(window_);
                } else {
                    glfwMaximizeWindow(window_);
                }
            }
        }

        // 4. Close Button
        if (renderButton("##TitleCloseBtn", cls_x, close_w, [](ImDrawList* dl, float cx, float cy, ImU32 col) {
            dl->AddLine(ImVec2(cx - 4.5f, cy - 4.5f), ImVec2(cx + 4.5f, cy + 4.5f), col, 1.5f);
            dl->AddLine(ImVec2(cx + 4.5f, cy - 4.5f), ImVec2(cx - 4.5f, cy + 4.5f), col, 1.5f);
        }, true, "Close")) {
            if (can_close_predicate_) {
                if (!can_close_predicate_()) {
                    if (on_close_blocked_) {
                        on_close_blocked_();
                    }
                    goto end_title_bar;
                }
            }
            if (on_close_callback_) {
                on_close_callback_();
            }
            glfwSetWindowShouldClose(window_, GLFW_TRUE);
        }

    end_title_bar:;
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
}

static GLFWmonitor* getCurrentMonitor(GLFWwindow* window) {
    if (!window) return glfwGetPrimaryMonitor();

    int win_x = 0, win_y = 0, win_w = 0, win_h = 0;
    glfwGetWindowPos(window, &win_x, &win_y);
    glfwGetWindowSize(window, &win_w, &win_h);

    int count = 0;
    GLFWmonitor** monitors = glfwGetMonitors(&count);
    if (!monitors || count == 0) {
        return glfwGetPrimaryMonitor();
    }

    GLFWmonitor* best_monitor = monitors[0];
    int best_overlap = -1;

    const int win_x2 = win_x + win_w;
    const int win_y2 = win_y + win_h;

    for (int i = 0; i < count; ++i) {
        int mx = 0, my = 0;
        glfwGetMonitorPos(monitors[i], &mx, &my);
        const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);
        if (!mode) continue;

        int mx2 = mx + mode->width;
        int my2 = my + mode->height;

        int overlap_x = std::max(0, std::min(win_x2, mx2) - std::max(win_x, mx));
        int overlap_y = std::max(0, std::min(win_y2, my2) - std::max(win_y, my));
        int overlap_area = overlap_x * overlap_y;

        if (overlap_area > best_overlap) {
            best_overlap = overlap_area;
            best_monitor = monitors[i];
        }
    }
    return best_monitor ? best_monitor : glfwGetPrimaryMonitor();
}

// Shrinks and moves a normal (not maximized) window so it lies inside the work area of the monitor it
// overlaps most; a window that overlaps no monitor goes to the first one
static void fitWindowToWorkArea(GLFWwindow* window) {
    GLFWmonitor* monitor = getCurrentMonitor(window);
    if (!monitor) return;
    int wx = 0, wy = 0, ww = 0, wh = 0;
    glfwGetMonitorWorkarea(monitor, &wx, &wy, &ww, &wh);
    if (ww <= 0 || wh <= 0) return;

    int x = 0, y = 0, w = 0, h = 0;
    glfwGetWindowPos(window, &x, &y);
    glfwGetWindowSize(window, &w, &h);
    const int new_w = std::min(w, ww);
    const int new_h = std::min(h, wh);
    const int new_x = std::clamp(x, wx, wx + ww - new_w);
    const int new_y = std::clamp(y, wy, wy + wh - new_h);
    if (new_w != w || new_h != h) {
        glfwSetWindowSize(window, new_w, new_h);
    }
    if (new_x != x || new_y != y) {
        glfwSetWindowPos(window, new_x, new_y);
    }
}

void AppWindow::setWindowGeometry(int x, int y, int width, int height, bool maximized) {
    if (!window_) return;
    if (width >= 640 && height >= 480) {
        saved_window_w_ = width;
        saved_window_h_ = height;
        glfwSetWindowSize(window_, width, height);
    }
    // Monitors left of or above the primary one have negative coordinates; -32000 marks a minimized window
    if (x > -10000 && y > -10000) {
        saved_window_x_ = x;
        saved_window_y_ = y;
        glfwSetWindowPos(window_, x, y);
    }
    // The settings may come from another PC or a monitor that is no longer connected
    fitWindowToWorkArea(window_);
    if (maximized) {
        saved_was_maximized_ = true;
        glfwMaximizeWindow(window_);
    }
}

void AppWindow::getWindowGeometry(int& out_x, int& out_y, int& out_width, int& out_height, bool& out_maximized) const {
    if (!window_) return;
    if (is_fullscreen_) {
        out_x = saved_window_x_;
        out_y = saved_window_y_;
        out_width = saved_window_w_;
        out_height = saved_window_h_;
        out_maximized = saved_was_maximized_;
        return;
    }
    out_maximized = (glfwGetWindowAttrib(window_, GLFW_MAXIMIZED) == GLFW_TRUE);
    if (!out_maximized) {
        glfwGetWindowPos(window_, &out_x, &out_y);
        glfwGetWindowSize(window_, &out_width, &out_height);
    }
}

void AppWindow::setFullscreen(bool fullscreen) {
    if (!window_) return;
    if (is_fullscreen_ == fullscreen) return;

    if (fullscreen) {
        saved_was_maximized_ = (glfwGetWindowAttrib(window_, GLFW_MAXIMIZED) == GLFW_TRUE);
        if (!saved_was_maximized_) {
            glfwGetWindowPos(window_, &saved_window_x_, &saved_window_y_);
            glfwGetWindowSize(window_, &saved_window_w_, &saved_window_h_);
        }

        GLFWmonitor* monitor = getCurrentMonitor(window_);
        if (!monitor) {
            monitor = glfwGetPrimaryMonitor();
        }
        if (monitor) {
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);
            if (mode) {
                is_fullscreen_ = true;
#ifdef _WIN32
                s_is_fullscreen = true;
#endif
                glfwSetWindowMonitor(window_, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            }
        }
    } else {
        is_fullscreen_ = false;
#ifdef _WIN32
        s_is_fullscreen = false;
#endif
        // Restore to saved window position and size
        glfwSetWindowMonitor(window_, nullptr, saved_window_x_, saved_window_y_, saved_window_w_, saved_window_h_, 0);

#ifdef _WIN32
        HWND hwnd = glfwGetWin32Window(window_);
        if (hwnd) {
            LONG_PTR style = GetWindowLongPtr(hwnd, GWL_STYLE);
            style |= WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX;
            SetWindowLongPtr(hwnd, GWL_STYLE, style);
            SetWindowPos(hwnd, nullptr, 0, 0, 0, 0,
                         SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_FRAMECHANGED);
        }
#endif

        if (saved_was_maximized_) {
            glfwMaximizeWindow(window_);
        }
    }
}

void AppWindow::toggleFullscreen() {
    setFullscreen(!is_fullscreen_);
}

void AppWindow::setCloseInterceptor(std::function<bool()> can_close_predicate,
                                    std::function<void()> on_close_blocked)
{
    can_close_predicate_ = std::move(can_close_predicate);
    on_close_blocked_    = std::move(on_close_blocked);
}

void AppWindow::closeWindow() noexcept {
    can_close_predicate_ = nullptr;
    if (on_close_callback_) {
        on_close_callback_();
    }
    if (window_) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

void AppWindow::applyIndustrialDarkTheme() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding    = 6.0f;
    style.ChildRounding     = 6.0f;
    style.FrameRounding     = 5.0f;
    style.GrabRounding      = 4.0f;
    style.PopupRounding     = 5.0f;
    style.ScrollbarRounding = 6.0f;
    style.FramePadding      = ImVec2(10.0f, 6.0f);
    style.ItemSpacing       = ImVec2(8.0f, 7.0f);
    style.WindowBorderSize  = 1.0f;
    style.ChildBorderSize   = 1.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]             = ImVec4(0.09f, 0.10f, 0.13f, 1.00f);
    colors[ImGuiCol_ChildBg]              = ImVec4(0.11f, 0.13f, 0.16f, 1.00f);
    colors[ImGuiCol_PopupBg]              = ImVec4(0.10f, 0.12f, 0.15f, 0.98f);
    colors[ImGuiCol_Border]               = ImVec4(0.22f, 0.25f, 0.31f, 1.00f);
    colors[ImGuiCol_FrameBg]              = ImVec4(0.15f, 0.18f, 0.22f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.20f, 0.24f, 0.30f, 1.00f);
    colors[ImGuiCol_FrameBgActive]        = ImVec4(0.24f, 0.30f, 0.38f, 1.00f);
    colors[ImGuiCol_TitleBg]              = ImVec4(0.07f, 0.08f, 0.10f, 1.00f);
    colors[ImGuiCol_TitleBgActive]        = ImVec4(0.10f, 0.13f, 0.18f, 1.00f);
    colors[ImGuiCol_Header]               = ImVec4(0.18f, 0.24f, 0.32f, 1.00f);
    colors[ImGuiCol_HeaderHovered]        = ImVec4(0.22f, 0.32f, 0.44f, 1.00f);
    colors[ImGuiCol_HeaderActive]         = ImVec4(0.26f, 0.38f, 0.52f, 1.00f);
    colors[ImGuiCol_Button]               = ImVec4(0.17f, 0.22f, 0.29f, 1.00f);
    colors[ImGuiCol_ButtonHovered]        = ImVec4(0.23f, 0.31f, 0.42f, 1.00f);
    colors[ImGuiCol_ButtonActive]         = ImVec4(0.14f, 0.46f, 0.70f, 1.00f);
    colors[ImGuiCol_CheckMark]            = ImVec4(0.00f, 0.85f, 0.95f, 1.00f);
    colors[ImGuiCol_SliderGrab]           = ImVec4(0.00f, 0.75f, 0.90f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.00f, 0.90f, 1.00f, 1.00f);
    colors[ImGuiCol_Tab]                  = ImVec4(0.12f, 0.15f, 0.20f, 1.00f);
    colors[ImGuiCol_TabHovered]           = ImVec4(0.22f, 0.32f, 0.45f, 1.00f);
    colors[ImGuiCol_TabSelected]          = ImVec4(0.16f, 0.26f, 0.38f, 1.00f);
}

} // namespace rnr::ui
