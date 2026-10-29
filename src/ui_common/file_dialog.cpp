#include "ui_common/file_dialog.h"
#include <cstdio>
#include <cstdlib>
#include <array>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>
#endif

namespace rnr::ui {

std::string openFileDialog(
    const std::string& title,
    const std::string& win_filter,
    const std::vector<std::string>& patterns)
{
#ifdef _WIN32
    char file_buf[MAX_PATH] = {0};
    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner   = nullptr;

    // Build a properly double-null-terminated filter buffer so embedded \0 is never truncated
    std::vector<char> filter_buf;
    auto append_filter_part = [&](const std::string& str) {
        filter_buf.insert(filter_buf.end(), str.begin(), str.end());
        filter_buf.push_back('\0');
    };

    if (!patterns.empty()) {
        std::string pattern_join;
        for (size_t i = 0; i < patterns.size(); ++i) {
            pattern_join += patterns[i];
            if (i + 1 < patterns.size()) pattern_join += ";";
        }
        std::string desc = "Supported Files (" + pattern_join + ")";
        append_filter_part(desc);
        append_filter_part(pattern_join);
    } else if (!win_filter.empty()) {
        append_filter_part(win_filter);
        append_filter_part("*.*");
    }

    // Always include "All Files (*.*)" option
    append_filter_part("All Files (*.*)");
    append_filter_part("*.*");
    filter_buf.push_back('\0'); // Double null terminator

    ofn.lpstrFilter = filter_buf.data();
    ofn.lpstrFile   = file_buf;
    ofn.nMaxFile    = sizeof(file_buf);
    ofn.lpstrTitle  = title.c_str();
    ofn.nFilterIndex = 1;
    ofn.Flags       = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameA(&ofn)) {
        return std::string(file_buf);
    }
    return {};

#elif defined(__APPLE__)
    // Native macOS AppleScript Cocoa Open Panel
    std::string extensions;
    for (size_t i = 0; i < patterns.size(); ++i) {
        std::string p = patterns[i];
        if (p.rfind("*.", 0) == 0) p = p.substr(2);
        extensions += "\"" + p + "\"" + (i + 1 < patterns.size() ? ", " : "");
    }
    std::string cmd = "osascript -e 'POSIX path of (choose file with prompt \"" + title + "\" of type {" + extensions + "})' 2>/dev/null";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return {};
    std::array<char, 1024> buffer{};
    std::string result;
    if (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        result = buffer.data();
        while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
            result.pop_back();
        }
    }
    pclose(pipe);
    return result;

#elif defined(__linux__) || defined(__unix__)
    // 1. Try zenity (GNOME / Ubuntu / Fedora / Debian standard)
    std::string pattern_list;
    for (const auto& pat : patterns) {
        pattern_list += " " + pat;
    }
    std::string zenity_cmd = "zenity --file-selection --title=\"" + title + "\" --file-filter=\"" + pattern_list + "\" 2>/dev/null";
    FILE* pipe = popen(zenity_cmd.c_str(), "r");
    if (pipe) {
        std::array<char, 1024> buffer{};
        std::string result;
        if (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
            result = buffer.data();
        }
        int status = pclose(pipe);
        if (status == 0 && !result.empty()) {
            while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
                result.pop_back();
            }
            return result;
        }
    }

    // 2. Try kdialog (KDE Plasma standard)
    std::string kdialog_cmd = "kdialog --getopenfilename . \"" + pattern_list + "\" --title \"" + title + "\" 2>/dev/null";
    pipe = popen(kdialog_cmd.c_str(), "r");
    if (pipe) {
        std::array<char, 1024> buffer{};
        std::string result;
        if (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
            result = buffer.data();
        }
        int status = pclose(pipe);
        if (status == 0 && !result.empty()) {
            while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
                result.pop_back();
            }
            return result;
        }
    }

    return {};
#else
    (void)title;
    (void)win_filter;
    (void)patterns;
    return {};
#endif
}

std::string openGCodeFileDialog() {
    return openFileDialog(
        "Open G-Code Program",
        "G-Code Files (*.nc;*.tap;*.gcode;*.ngc;*.cnc;*.txt)",
        {"*.nc", "*.tap", "*.gcode", "*.ngc", "*.cnc", "*.txt"});
}

std::string openLuaScriptDialog() {
    return openFileDialog(
        "Select Lua Script to Execute",
        "Lua Scripts (*.lua)",
        {"*.lua"});
}

bool openFileInDefaultEditor(const std::string& path) {
    if (path.empty()) return false;
#ifdef _WIN32
    HINSTANCE h = ShellExecuteA(nullptr, "open", path.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
    if (reinterpret_cast<INT_PTR>(h) <= 32) {
        h = ShellExecuteA(nullptr, "open", "notepad.exe", path.c_str(), nullptr, SW_SHOWNORMAL);
    }
    return reinterpret_cast<INT_PTR>(h) > 32;
#elif defined(__APPLE__)
    std::string cmd = "open \"" + path + "\"";
    return system(cmd.c_str()) == 0;
#else
    std::string cmd = "xdg-open \"" + path + "\"";
    return system(cmd.c_str()) == 0;
#endif
}

} // namespace rnr::ui
