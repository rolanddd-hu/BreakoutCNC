#pragma once
#include "core/camera/camera_manager.h"
#include "core/controller/motion_controller.h"
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <cstdint>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace rnr::core {

struct LoadGCodeResult {
    bool        success       = false;
    std::string error_message;
    size_t      line_count    = 0;
};

using LoadGCodeCallback = std::function<LoadGCodeResult(const std::string& filename, const std::string& content)>;

// The remote monitoring web server: dashboard page, MJPEG camera stream, telemetry, remote E-stop and (opt-in)
// G-code upload. Protections:
//  - Host header must be an IP address, localhost or this PC's name (blocks DNS rebinding from web sites);
//  - requests a browser marks cross-site are refused; actions (POST) need the dashboard's own header and a matching
//    Origin, so another web page cannot upload or press E-stop through the operator's browser;
//  - optional access code: the page asks for it once, then a session cookie (HttpOnly, SameSite=Strict) is used;
//  - optional listen address (all networks, one network, or this PC only);
//  - bounded connections (total, per address, camera viewers), request deadlines, one upload at a time, and a body is
//    only read for an allowed upload.
class CameraStreamServer {
public:
    CameraStreamServer(CameraManager& camera_mgr);
    ~CameraStreamServer();

    CameraStreamServer(const CameraStreamServer&) = delete;
    CameraStreamServer& operator=(const CameraStreamServer&) = delete;

    // Set callback invoked when remote user clicks EMERGENCY STOP on the web dashboard
    void setEStopCallback(std::function<void()> cb) { on_estop_ = std::move(cb); }

    // Set callback invoked when remote user uploads and loads a G-Code file from the web dashboard. The file name
    // passed is already reduced to a safe base name (see sanitizeUploadFileName).
    void setLoadGCodeCallback(LoadGCodeCallback cb) { on_load_gcode_ = std::move(cb); }

    // Start or stop server according to config (port, listen address, access code, fps, enabled). Binding happens
    // here, on the caller's thread: when it fails, isRunning() is false and lastError() says why.
    void applyConfig(const CameraConfig& cfg);

    // Update live telemetry snapshot served at /api/telemetry (including loaded program name)
    void updateTelemetry(const TelemetrySnapshot& telemetry, LinearUnits units, const std::string& app_title, const std::string& loaded_filename = "");

    bool isRunning() const noexcept { return running_.load(); }
    uint16_t getPort() const noexcept { return port_.load(); }
    int getActiveClientCount() const noexcept { return active_clients_.load(); }
    int getActiveStreamCount() const noexcept { return active_streams_.load(); }
    std::string getServerUrl() const;
    std::string lastError() const;

    void stop();

    // This PC's IPv4 addresses (without loopback and link-local), for the listen address choice
    static std::vector<std::string> localIpv4Addresses();

    // Exposed for unit testing: encode RGBA image buffer to baseline JPEG bytes
    static void encodeRgbaToJpeg(const uint8_t* rgba, int width, int height, std::vector<uint8_t>& out_jpeg);

    // Exposed for unit testing: a file name an upload may be saved under (base name only, safe characters, no
    // Windows device names such as CON, NUL, COM3 or LPT1)
    static std::string sanitizeUploadFileName(const std::string& name);

    // Exposed for unit testing: whether a Host header names this server (IP address, localhost, machine name)
    static bool isAllowedHost(const std::string& host_header, const std::string& machine_name);

    // Limits
    static constexpr int    kMaxClients        = 16;
    static constexpr int    kMaxClientsPerAddr = 6;
    static constexpr int    kMaxStreams        = 4;
    static constexpr size_t kMaxUploadBytes    = 32u * 1024u * 1024u;
    static constexpr int    kMaxLoginFailures  = 5;

private:
    struct ClientThread {
        std::thread                        thread;
        std::shared_ptr<std::atomic<bool>> done;
    };
    struct LoginFailures {
        int                                   count = 0;
        std::chrono::steady_clock::time_point locked_until{};
    };

    CameraManager&        camera_mgr_;
    std::function<void()> on_estop_;
    LoadGCodeCallback     on_load_gcode_;

    std::atomic<bool>     running_{false};
    std::atomic<uint16_t> port_{8080};
    std::atomic<int>      target_fps_{15};
    std::atomic<int>      active_clients_{0};
    std::atomic<int>      active_streams_{0};
    std::atomic<bool>     allow_upload_{false};
    std::atomic<bool>     upload_busy_{false};
    std::string           bind_address_;              // "" = all networks (caller's thread only)
    std::string           machine_name_;              // gethostname(), for the Host check
    uintptr_t             listen_sock_ = ~static_cast<uintptr_t>(0); // Caller's thread only
    std::thread           accept_thread_;

    std::mutex                      clients_mutex_;
    std::set<uintptr_t>             client_socks_;    // Open client connections (shut down on stop())
    std::map<uint32_t, int>         clients_per_addr_;
    std::vector<ClientThread>       client_threads_;  // Joined on stop(); finished ones reaped on accept

    mutable std::mutex              auth_mutex_;
    std::string                     access_code_;     // "" = no code
    std::deque<std::string>         sessions_;        // Session tokens handed out since the code was set
    std::map<uint32_t, LoginFailures> login_failures_;

    mutable std::mutex    error_mutex_;
    std::string           last_error_;

    mutable std::mutex    telemetry_mutex_;
    TelemetrySnapshot     latest_telemetry_{};
    LinearUnits           latest_units_ = LinearUnits::Metric;
    std::string           app_title_    = "BreakoutCNC";
    std::string           loaded_filename_;

    void acceptLoop(uintptr_t listen_sock);
    void handleClient(uintptr_t client_sock, uint32_t peer_addr);
    void serveRequest(uintptr_t client_sock, uint32_t peer_addr);
    void reapFinishedClientsLocked();
    void setLastError(const std::string& e);
    void setAccessCode(const std::string& code);
    bool sessionValid(const std::string& cookie_header) const;
    std::string buildTelemetryJson() const;
    std::string buildDashboardHtml() const;
};

} // namespace rnr::core
