#pragma once
// File names inside BreakoutCNC are UTF-8 strings (Dear ImGui draws UTF-8, settings files store UTF-8). These helpers
// convert at the file system edge: std::filesystem::path from a std::string would use the Windows ANSI code page,
// which cannot hold every name (and mangles accented ones).
#include <filesystem>
#include <string>

namespace rnr::util {

// UTF-8 string -> path (any character, on every platform)
inline std::filesystem::path pathFromUtf8(const std::string& utf8) {
    return std::filesystem::path(std::u8string(utf8.begin(), utf8.end()));
}

// path -> UTF-8 string
inline std::string pathToUtf8(const std::filesystem::path& p) {
    const std::u8string u = p.u8string();
    return std::string(u.begin(), u.end());
}

// True when the bytes are well-formed UTF-8 (plain ASCII included)
bool isValidUtf8(const std::string& s);

// A file name stored by an older version: ANSI code page on Windows. Valid UTF-8 is returned as it is.
std::string utf8FromLegacy(const std::string& s);

#ifdef _WIN32
std::string utf8FromWide(const std::wstring& w);
std::wstring wideFromUtf8(const std::string& s);
#endif

} // namespace rnr::util
