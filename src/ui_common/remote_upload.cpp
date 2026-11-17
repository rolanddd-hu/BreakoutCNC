#include "ui_common/remote_upload.h"
#include "core/gcode/gcode_parser.h"
#include "core/util/utf8_path.h"
#include <filesystem>
#include <fstream>

namespace rnr::ui {

namespace {
std::filesystem::path uploadDir() {
    std::error_code ec;
    const auto tmp = std::filesystem::temp_directory_path(ec);
    return ec ? std::filesystem::path{} : tmp / "BreakoutCNC" / "uploads";
}
} // namespace

core::LoadGCodeResult loadRemoteUpload(core::IMotionController& controller, const std::string& filename,
                                       const std::string& content, std::string& out_path) {
    if (!controller.loadGCodeString(content)) {
        // The controller says why (a job is active, the file has no moves)
        const auto msg = controller.getTelemetry().active_fault_message;
        return {false, msg.empty() ? std::string("The program could not be loaded") : msg, 0};
    }
    out_path = filename;
    const auto dir = uploadDir();
    if (!dir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        const auto disk_path = dir / util::pathFromUtf8(filename); // The name is sanitized by the server
        std::ofstream ofs(disk_path, std::ios::binary | std::ios::trunc);
        ofs.write(content.data(), static_cast<std::streamsize>(content.size()));
        ofs.close();
        if (ofs.good()) out_path = util::pathToUtf8(disk_path);
    }
    return {true, "", controller.getProgram()->segments.size()};
}

void cleanOldRemoteUploads(const std::string& keep_path) {
    const auto dir = uploadDir();
    std::error_code ec;
    if (dir.empty() || !std::filesystem::is_directory(dir, ec)) return;
    const auto keep = util::pathFromUtf8(keep_path);
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
        std::error_code ec2;
        if (!e.is_regular_file(ec2)) continue;
        if (!keep_path.empty() && std::filesystem::equivalent(e.path(), keep, ec2)) continue;
        std::filesystem::remove(e.path(), ec2);
    }
}

} // namespace rnr::ui
