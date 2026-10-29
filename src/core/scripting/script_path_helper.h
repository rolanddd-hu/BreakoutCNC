#pragma once
#include <string>
#include <filesystem>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace rnr::scripting {

inline std::string resolveScriptPath(const std::string& path_str) {
    if (path_str.empty()) return {};

    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path p(path_str);

    // 1. Direct check (working directory or absolute path)
    if (fs::exists(p, ec) && !fs::is_directory(p, ec)) {
        return fs::absolute(p, ec).string();
    }

#ifdef _WIN32
    // 2. Search relative to executable directory
    char exe_buf[MAX_PATH] = {0};
    if (GetModuleFileNameA(NULL, exe_buf, MAX_PATH)) {
        fs::path exe_dir = fs::path(exe_buf).parent_path();

        // Candidate A: <exe_dir>/<path>
        fs::path c1 = exe_dir / p;
        if (fs::exists(c1, ec) && !fs::is_directory(c1, ec)) {
            return fs::absolute(c1, ec).string();
        }

        // Candidate B: <exe_dir>/../<path>
        fs::path c2 = exe_dir.parent_path() / p;
        if (fs::exists(c2, ec) && !fs::is_directory(c2, ec)) {
            return fs::absolute(c2, ec).string();
        }

        // Candidate C: <exe_dir>/../../<path> (repo root when run from build/Release)
        fs::path c3 = exe_dir.parent_path().parent_path() / p;
        if (fs::exists(c3, ec) && !fs::is_directory(c3, ec)) {
            return fs::absolute(c3, ec).string();
        }
    }
#endif

    return {};
}

} // namespace rnr::scripting
