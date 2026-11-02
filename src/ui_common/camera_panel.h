#pragma once
#include "core/controller/motion_controller.h"
#include "core/camera/camera_manager.h"
#include "core/camera/camera_stream_server.h"
#include "ui_common/toolpath_viewport.h"
#include <vector>

namespace rnr::ui {

class CameraPanel {
public:
    CameraPanel(core::CameraManager& cam_mgr, core::CameraStreamServer& stream_srv);
    ~CameraPanel() = default;

    // Renders the Camera Alignment & Remote Monitoring sub-cards with progressive disclosure.
    // Returns true if MachineConfig::camera was modified and should be persisted.
    bool render(core::IMotionController& controller,
                const core::TelemetrySnapshot& telemetry,
                ToolpathViewport* viewport,
                bool enable_top_down_positioning);

private:
    core::CameraManager&                  cam_mgr_;
    core::CameraStreamServer&             stream_srv_;
    std::vector<core::CameraDeviceInfo>   devices_;
    bool                                  devices_initialized_ = false;
    int                                   cal_step_            = 0; // 0 = Step 1 (XY Offset), 1 = Step 2 (FOV)
    int                                   fov_method_          = 0; // 0 = Method A (Move & Click), 1 = Method B (Tape Measure)
};

} // namespace rnr::ui
