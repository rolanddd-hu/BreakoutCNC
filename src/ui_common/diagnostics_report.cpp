#include "ui_common/diagnostics_report.h"
#include "breakout_version.h"
#include "imgui.h"
#include <cstdio>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h> // Before the GL header (APIENTRY / WINGDIAPI)
#else
#include <sys/utsname.h>
#endif
#ifdef __APPLE__
#include <sys/sysctl.h>
#define GL_SILENCE_DEPRECATION
#endif
#include <GLFW/glfw3.h> // Pulls in the system GL header for glGetString()
#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <dwmapi.h>
#endif

namespace rnr::ui {

namespace {
#ifdef _WIN32
std::string rectText(const RECT& r) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%ld,%ld - %ld,%ld", r.left, r.top, r.right, r.bottom);
    return buf;
}

// Window placement as Windows sees it, and the mouse as each layer reports it: tells an app-side mismatch from a
// graphics driver that shows the picture shifted (seen with an old Intel HD 4400 driver)
std::string windowGeometry() {
    GLFWwindow* win = glfwGetCurrentContext();
    HWND hwnd = win ? glfwGetWin32Window(win) : nullptr;
    if (!hwnd) return "Window geometry: (no window)\n";
    RECT wr{}, cr{}, er{};
    GetWindowRect(hwnd, &wr);
    GetClientRect(hwnd, &cr);
    const bool have_ext = SUCCEEDED(DwmGetWindowAttribute(hwnd, DWMWA_EXTENDED_FRAME_BOUNDS, &er, sizeof(er)));
    POINT origin{0, 0};
    ClientToScreen(hwnd, &origin);
    MONITORINFO mi{};
    mi.cbSize = sizeof(mi);
    GetMonitorInfoW(MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST), &mi);
    POINT cursor{};
    GetCursorPos(&cursor);
    POINT cursor_client = cursor;
    ScreenToClient(hwnd, &cursor_client);
    int fb_w = 0, fb_h = 0, ww = 0, wh = 0;
    glfwGetFramebufferSize(win, &fb_w, &fb_h);
    glfwGetWindowSize(win, &ww, &wh);
    double gx = 0.0, gy = 0.0;
    glfwGetCursorPos(win, &gx, &gy);
    const ImGuiIO& io = ImGui::GetIO();

    std::ostringstream r;
    r << "Window geometry: " << (IsZoomed(hwnd) ? "maximized" : "not maximized") << ", dpi " << GetDpiForWindow(hwnd)
      << ", system dpi " << GetDpiForSystem() << ", frame metrics " << GetSystemMetrics(SM_CXSIZEFRAME) << "+"
      << GetSystemMetrics(SM_CXPADDEDBORDER) << "\n"
      << "  monitor " << rectText(mi.rcMonitor) << ", work area " << rectText(mi.rcWork) << "\n"
      << "  window " << rectText(wr) << ", visible frame " << (have_ext ? rectText(er) : std::string("?")) << "\n"
      << "  client origin " << origin.x << "," << origin.y << " size " << cr.right << "x" << cr.bottom
      << ", GLFW window " << ww << "x" << wh << ", framebuffer " << fb_w << "x" << fb_h << "\n"
      << "  mouse: screen " << cursor.x << "," << cursor.y << ", client " << cursor_client.x << "," << cursor_client.y
      << ", GLFW " << static_cast<int>(gx) << "," << static_cast<int>(gy) << ", ImGui "
      << static_cast<int>(io.MousePos.x) << "," << static_cast<int>(io.MousePos.y) << "\n";
    return r.str();
}
#endif

std::string osDescription() {
#ifdef _WIN32
    // GetVersionEx reports Windows 8 for unmanifested apps; RtlGetVersion gives the real build
    using RtlGetVersionFn = LONG(WINAPI*)(PRTL_OSVERSIONINFOW);
    OSVERSIONINFOW vi{};
    vi.dwOSVersionInfoSize = sizeof(vi);
    if (HMODULE ntdll = GetModuleHandleW(L"ntdll.dll")) {
        if (auto fn = reinterpret_cast<RtlGetVersionFn>(GetProcAddress(ntdll, "RtlGetVersion"))) fn(&vi);
    }
    char buf[96];
    // Windows 11 still reports major version 10; build 22000+ is Windows 11
    std::snprintf(buf, sizeof(buf), "Windows %s (build %lu)",
                  (vi.dwMajorVersion == 10 && vi.dwBuildNumber >= 22000) ? "11" : std::to_string(vi.dwMajorVersion).c_str(),
                  vi.dwBuildNumber);
    return buf;
#else
    std::string name;
#ifdef __APPLE__
    char ver[64] = {};
    size_t len = sizeof(ver);
    if (sysctlbyname("kern.osproductversion", ver, &len, nullptr, 0) == 0) name = std::string("macOS ") + ver;
#else
    std::ifstream os_release("/etc/os-release");
    std::string line;
    while (std::getline(os_release, line)) {
        if (line.rfind("PRETTY_NAME=", 0) == 0) {
            name = line.substr(12);
            if (name.size() >= 2 && name.front() == '"' && name.back() == '"') name = name.substr(1, name.size() - 2);
            break;
        }
    }
#endif
    utsname u{};
    if (uname(&u) == 0) {
        name += (name.empty() ? "" : ", ") + std::string(u.sysname) + " " + u.release + " " + u.machine;
    }
    return name.empty() ? "unknown" : name;
#endif
}

const char* stateName(core::ControllerState s) {
    switch (s) {
        case core::ControllerState::Disconnected:   return "Disconnected";
        case core::ControllerState::EStop:          return "E-stop";
        case core::ControllerState::Idle:           return "Idle";
        case core::ControllerState::RunningGCode:   return "Running G-code";
        case core::ControllerState::PausedHold:     return "Feed hold";
        case core::ControllerState::Jogging:        return "Jogging";
        case core::ControllerState::Homing:         return "Homing";
        case core::ControllerState::Probing:        return "Probing";
        case core::ControllerState::DwellPierce:    return "Dwell / pierce";
        case core::ControllerState::ToolChangeHold: return "Tool change hold";
    }
    return "?";
}

const char* glString(GLenum name) {
    const auto* s = reinterpret_cast<const char*>(glGetString(name));
    return s ? s : "?";
}
} // namespace

std::string buildDiagnosticsReport(const core::MachineConfig& cfg,
                                   const core::TelemetrySnapshot& t,
                                   size_t program_lines,
                                   size_t program_segments)
{
    const char* app = cfg.is_mill ? "BreakoutMill"
                    : (cfg.operation_mode == core::OperationMode::Laser ? "BreakoutCut (laser)" : "BreakoutCut (plasma)");
#if defined(__aarch64__) || defined(_M_ARM64)
    const char* arch = "arm64";
#elif defined(__x86_64__) || defined(_M_X64)
    const char* arch = "x86_64";
#else
    const char* arch = "other";
#endif
    const ImGuiIO& io = ImGui::GetIO();
    const char* tool_mode = cfg.tool_table.mode == core::ToolLengthMode::ToolTable        ? "tool table"
                          : cfg.tool_table.mode == core::ToolLengthMode::TouchOffEachTool ? "touch off every tool"
                                                                                          : "not configured";

    std::ostringstream r;
    r << "BreakoutCNC " << BREAKOUTCNC_VERSION_STRING << " - " << app << " (" << arch << " build)\n"
      << "OS: " << osDescription() << "\n"
      << "OpenGL: " << glString(GL_RENDERER) << " | " << glString(GL_VENDOR) << " | " << glString(GL_VERSION) << "\n"
      << "Window: " << static_cast<int>(io.DisplaySize.x) << " x " << static_cast<int>(io.DisplaySize.y)
      << " (framebuffer scale " << io.DisplayFramebufferScale.x << "), " << static_cast<int>(io.Framerate + 0.5f) << " FPS\n";
#ifdef _WIN32
    r << windowGeometry();
#endif

    r << "Board: ";
    if (t.simulator_mode)      r << "simulator";
    else if (t.usb_connected)  r << (t.product_name.empty() ? "connected" : t.product_name);
    else                       r << "not connected";
    r << " | state: " << stateName(t.state) << " | last ack seq " << t.last_ack_seq << "\n";
    r << "Board id: " << (t.board_id.empty() ? "?" : t.board_id);
    if (t.board_firmware_planned) {
        r << " | firmware: " << (t.firmware_version.empty() ? "unknown" : t.firmware_version)
          << " | firmware state: " << (t.firmware_state.empty() ? "-" : t.firmware_state);
        if (!t.board_status_text.empty()) r << " | " << t.board_status_text;
    }
    r << "\n";

    r << "Settings: " << (cfg.linear_units == core::LinearUnits::Imperial ? "inch" : "mm")
      << ", require homing " << (cfg.require_homing ? "on" : "off")
      << ", start lift " << (cfg.start_lift_to_safe_z ? "on" : "off") << " (Z " << cfg.start_safe_z_mm << ")"
      << ", tool length: " << tool_mode
      << ", VFD " << (cfg.vfd.enabled ? "on" : "off")
      << ", pulse rate " << static_cast<int>(cfg.pulse_rate_khz) << " kHz\n";
    r << "Program: " << program_lines << " lines, " << program_segments << " segments\n";
    r << "Last fault: " << (t.active_fault_message.empty() ? "(none)" : t.active_fault_message) << "\n";
    return r.str();
}

} // namespace rnr::ui
