#pragma once
#include "core/camera/camera_manager.h"
#include "core/controller/motion_controller.h"
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <cstdint>
#include <set>

namespace rnr::core {

struct LoadGCodeResult {
    bool        success       = false;
    std::string error_message;
    size_t      line_count    = 0;
};

using LoadGCodeCallback = std::function<LoadGCodeResult(const std::string& filename, const std::string& content)>;

class CameraStreamServer {
public:
    CameraStreamServer(CameraManager& camera_mgr);
    ~CameraStreamServer();

    CameraStreamServer(const CameraStreamServer&) = delete;
    CameraStreamServer& operator=(const CameraStreamServer&) = delete;

    // Set callback invoked when remote user clicks EMERGENCY STOP on the web dashboard
    void setEStopCallback(std::function<void()> cb) { on_estop_ = std::move(cb); }

    // Set callback invoked when remote user uploads and loads a G-Code file from the web dashboard
    void setLoadGCodeCallback(LoadGCodeCallback cb) { on_load_gcode_ = std::move(cb); }

    // Start or stop server according to config (port, fps, enabled)
    void applyConfig(const CameraConfig& cfg);

    // Update live telemetry snapshot served at /api/telemetry (including loaded program name)
    void updateTelemetry(const TelemetrySnapshot& telemetry, LinearUnits units, const std::string& app_title, const std::string& loaded_filename = "");

    bool isRunning() const noexcept { return running_.load(); }
    uint16_t getPort() const noexcept { return port_.load(); }
    int getActiveClientCount() const noexcept { return active_clients_.load(); }
    std::string getServerUrl() const;

    void stop();

    // Exposed for unit testing: encode RGBA image buffer to baseline JPEG bytes
    static void encodeRgbaToJpeg(const uint8_t* rgba, int width, int height, std::vector<uint8_t>& out_jpeg);

private:
    CameraManager&        camera_mgr_;
    std::function<void()> on_estop_;
    LoadGCodeCallback     on_load_gcode_;

    std::atomic<bool>     running_{false};
    std::atomic<uint16_t> port_{8080};
    std::atomic<int>      target_fps_{15};
    std::atomic<int>      active_clients_{0};
    std::atomic<bool>     allow_upload_{false};
    uintptr_t             listen_sock_ = ~static_cast<uintptr_t>(0);
    std::thread           accept_thread_;
    std::mutex            clients_mutex_;
    std::set<uintptr_t>   client_socks_; // Open client connections (shut down on stop())

    mutable std::mutex    telemetry_mutex_;
    TelemetrySnapshot     latest_telemetry_{};
    LinearUnits           latest_units_ = LinearUnits::Metric;
    std::string           app_title_    = "BreakoutCNC";
    std::string           loaded_filename_;

    void acceptLoop(uint16_t port);
    void handleClient(uintptr_t client_sock);
    std::string buildTelemetryJson() const;
    std::string buildDashboardHtml() const;
};

} // namespace rnr::core
