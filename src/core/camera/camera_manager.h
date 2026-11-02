#pragma once
#include "core/controller/motion_controller.h"
#include <vector>
#include <string>
#include <cstdint>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <array>

namespace rnr::core {

enum class CameraRole : uint8_t {
    TopDown    = 0, // Top-down optical XY alignment camera (BreakoutMill)
    Monitoring = 1, // Remote monitoring / enclosure webcam (BreakoutMill & BreakoutCut)
};

struct CameraDeviceInfo {
    int         index        = -1;
    std::string name;
    std::string unique_id;
    bool        is_simulator = false;
};

struct CameraFrame {
    int                  width     = 0;
    int                  height    = 0;
    uint64_t             frame_seq = 0;
    std::vector<uint8_t> rgba; // 4 bytes per pixel (R, G, B, A)
};

class CameraManager {
public:
    CameraManager();
    ~CameraManager();

    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

    // Enumerate available USB / UVC cameras (plus virtual simulator cameras when include_simulator is true)
    std::vector<CameraDeviceInfo> enumerateDevices(bool include_simulator = true);

    // Apply CameraConfig and synchronize active capture streams for TopDown and Monitoring roles.
    // Enforces exclusive assignment if a single camera is selected for one of the two roles.
    void applyConfig(CameraConfig& inout_cfg, bool simulator_mode, bool allow_top_down_role = true);

    // Update live machine telemetry for the procedural simulator camera feeds (and dent detection)
    void updateLiveTelemetry(const TelemetrySnapshot& telemetry, const MachineConfig& cfg);

    // Explicitly stamp a calibration dent at the current tool XY position (for simulator or visual reference)
    void makeCalibrationDentAtCurrentToolPos(const TelemetrySnapshot& telemetry);

    // Query whether a role currently has an active camera stream
    bool isRoleActive(CameraRole role) const;

    // Copy the latest RGBA frame for a role if frame_seq > inout_last_seq (returns true if updated)
    bool getLatestFrame(CameraRole role, CameraFrame& out_frame, uint64_t& inout_last_seq) const;

    // Always copy the latest frame for a role (returns true if valid frame exists)
    bool copyCurrentFrame(CameraRole role, CameraFrame& out_frame) const;

    // Stop all capture threads
    void stopAll();

private:
    struct StreamSlot;
    std::unique_ptr<StreamSlot> slots_[2];

    mutable std::mutex                devices_mutex_;
    std::vector<CameraDeviceInfo>     cached_devices_;
    bool                              last_simulator_mode_ = false;

    mutable std::mutex                sim_state_mutex_;
    std::array<double, 4>             sim_machine_pos_mm_{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4>             sim_work_pos_mm_{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4>             sim_work_offset_mm_{0.0, 0.0, 0.0, 0.0};
    bool                              sim_spindle_on_  = false;
    ControllerState                   sim_state_       = ControllerState::Idle;
    OperationMode                     sim_op_mode_     = OperationMode::Plasma;
    double                            sim_progress_    = 0.0;
    double                            sim_elapsed_s_   = 0.0;
    bool                              sim_has_dent_    = true;
    double                            sim_dent_work_x_ = 0.0;
    double                            sim_dent_work_y_ = 0.0;

    void startSlot(CameraRole role, int device_index, bool mirror_x, bool mirror_y);
    void stopSlot(CameraRole role);
    void captureThreadLoop(CameraRole role, int device_index);
    void renderSimulatedTopDownFrame(CameraFrame& frame, bool mirror_x, bool mirror_y);
    void renderSimulatedMonitorFrame(CameraFrame& frame, bool mirror_x, bool mirror_y);
};

} // namespace rnr::core
