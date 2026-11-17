#pragma once
// Pieces both applications (BreakoutMill, BreakoutCut) share around their main loop
#include "core/config/settings_manager.h"
#include "core/controller/motion_controller.h"
#include <functional>
#include <mutex>
#include <string>

namespace rnr::ui {

class AppWindow;
class ToolpathViewport;

// ESC is the keyboard E-stop. While the machine moves (or a job is paused) it always stops it. At rest, ESC in an open
// popup or a text field only closes / leaves it, as everywhere else in the UI.
bool escapeTriggersEStop(const core::TelemetrySnapshot& telemetry);

// A settings file that cannot be written (read-only folder): a small warning at the bottom of the window until it is
// dismissed or a later save succeeds. `message` is cleared by the Dismiss button.
void renderSettingsSaveWarning(std::string& message);

// Nothing on screen changes by itself: the machine is at rest, no script runs and no camera is shown (the window may
// then draw fewer frames, see AppWindow::setIdlePredicate)
bool screenIsStatic(const core::TelemetrySnapshot& telemetry, const core::MachineConfig& cfg);

// Title bar layout switch and theme buttons, viewport view options: restored from the settings, saved when changed
void connectWindowSettings(AppWindow& window, ToolpathViewport& viewport, config::AppUiSettings& ui_settings,
                           const std::function<void()>& mark_dirty, bool* minimal_layout);

// Closing the window while a job, a dwell or a jog runs asks first (`show_confirm`, answered by the app's modal,
// which sets `force_close`); closing always E-stops and saves the settings
void installCloseHandling(AppWindow& window, core::IMotionController& controller, bool& force_close, bool& show_confirm,
                          std::function<void()> save_settings);

// --smoke-test (CI): reset, start the demo job, check after ~4 s that it advanced, close. Exit code 2 when it failed.
struct SmokeTest {
    bool enabled = false;
    int  frames  = 0;
    bool failed  = false;
    void frame(AppWindow& window, core::IMotionController& controller);
};

// A program uploaded on the remote page: loaded on the web server's thread, shown by the UI on its next frame
class RemoteUploadInbox {
public:
    void post(const std::string& path) {
        std::lock_guard<std::mutex> lock(m_);
        path_ = path;
        fit_ = true;
    }
    // The newest upload (empty if none) and whether the view should fit it
    bool take(std::string& path, bool& fit) {
        std::lock_guard<std::mutex> lock(m_);
        if (path_.empty()) return false;
        path = path_;
        fit = fit_;
        path_.clear();
        fit_ = false;
        return true;
    }
private:
    std::mutex  m_;
    std::string path_;
    bool        fit_ = false;
};

} // namespace rnr::ui
