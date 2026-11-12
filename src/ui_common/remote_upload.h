#pragma once
#include "core/camera/camera_stream_server.h"
#include "core/controller/motion_controller.h"
#include <string>

namespace rnr::ui {

// Loads a G-code file uploaded from the remote monitoring page (BreakoutMill and BreakoutCut). The program is loaded
// first; only an accepted program is kept as a file in the temp folder (BreakoutCNC/uploads), so it can be reloaded
// and edited like a file from disk. `out_path` = that file (UTF-8), or the plain name if it could not be saved.
core::LoadGCodeResult loadRemoteUpload(core::IMotionController& controller, const std::string& filename,
                                       const std::string& content, std::string& out_path);

// Deletes old uploads at startup, except `keep_path` (the last loaded program, reloaded at startup)
void cleanOldRemoteUploads(const std::string& keep_path);

} // namespace rnr::ui
