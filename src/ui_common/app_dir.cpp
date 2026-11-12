#include "ui_common/app_dir.h"
#include "core/util/app_paths.h"
#include "core/util/utf8_path.h"
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
#include <shellapi.h>
#endif

namespace rnr::ui {

bool hasCommandLineFlag(int argc, char** argv, const char* flag) {
    for (int i = 1; i < argc; ++i) {
        if (argv[i] && std::strcmp(argv[i], flag) == 0) return true;
    }
    return false;
}

std::string enterExecutableDirectory(int argc, char** argv) {
    std::error_code ec;
    std::string arg_file; // UTF-8
#ifdef _WIN32
    // argv is in the ANSI code page, which cannot hold every file name: read the wide command line instead
    (void)argc;
    (void)argv;
    int wargc = 0;
    if (LPWSTR* wargv = CommandLineToArgvW(GetCommandLineW(), &wargc)) {
        for (int i = 1; i < wargc; ++i) {
            const std::wstring a = wargv[i];
            if (a.empty() || a.rfind(L"--", 0) == 0) continue;
            const auto abs = std::filesystem::absolute(std::filesystem::path(a), ec);
            arg_file = util::pathToUtf8(ec ? std::filesystem::path(a) : abs);
            break;
        }
        LocalFree(wargv);
    }
#else
    for (int i = 1; i < argc; ++i) {
        if (!argv[i] || argv[i][0] == '\0' || std::strncmp(argv[i], "--", 2) == 0) continue;
        const auto abs = std::filesystem::absolute(argv[i], ec);
        arg_file = ec ? std::string(argv[i]) : abs.string();
        break;
    }
#endif
    const auto exe = util::executablePath();
    if (!exe.empty()) {
        std::filesystem::current_path(exe.parent_path(), ec); // On failure keep the current folder
    }
    return arg_file;
}

} // namespace rnr::ui
