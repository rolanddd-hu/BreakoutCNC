#include "ui_common/app_dir.h"
#include <cstring>
#include <filesystem>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#include <cstdint>
#endif

namespace rnr::ui {

namespace {
std::filesystem::path executablePath() {
#ifdef _WIN32
    std::vector<wchar_t> buf(MAX_PATH);
    while (true) {
        const DWORD n = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
        if (n == 0) return {};
        if (n < buf.size()) return std::filesystem::path(std::wstring(buf.data(), n));
        buf.resize(buf.size() * 2);
    }
#elif defined(__APPLE__)
    uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::vector<char> buf(size + 1, '\0');
    if (_NSGetExecutablePath(buf.data(), &size) != 0) return {};
    std::error_code ec;
    const auto p = std::filesystem::canonical(buf.data(), ec);
    return ec ? std::filesystem::path(buf.data()) : p;
#else
    std::error_code ec;
    const auto p = std::filesystem::read_symlink("/proc/self/exe", ec);
    return ec ? std::filesystem::path{} : p;
#endif
}
} // namespace

bool hasCommandLineFlag(int argc, char** argv, const char* flag) {
    for (int i = 1; i < argc; ++i) {
        if (argv[i] && std::strcmp(argv[i], flag) == 0) return true;
    }
    return false;
}

std::string enterExecutableDirectory(int argc, char** argv) {
    std::error_code ec;
    std::string arg_file;
    for (int i = 1; i < argc; ++i) {
        if (!argv[i] || argv[i][0] == '\0' || std::strncmp(argv[i], "--", 2) == 0) continue;
        const auto abs = std::filesystem::absolute(argv[i], ec);
        arg_file = ec ? std::string(argv[i]) : abs.string();
        break;
    }
    const auto exe = executablePath();
    if (!exe.empty()) {
        std::filesystem::current_path(exe.parent_path(), ec); // On failure keep the current folder
    }
    return arg_file;
}

} // namespace rnr::ui
