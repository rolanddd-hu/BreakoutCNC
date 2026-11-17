// Built into lua_lib (Windows): Lua's fopen / freopen are redirected here by core/util/utf8_fopen.h, so script file
// names are UTF-8 like every other file name in BreakoutCNC (the C library would read them in the ANSI code page).
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <cstdio>
#include <string>

namespace {
std::wstring wide(const char* s) {
    if (!s || !*s) return {};
    const int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, s, -1, nullptr, 0);
    if (n <= 0) return {};
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, s, -1, w.data(), n);
    w.resize(static_cast<size_t>(n - 1));
    return w;
}
} // namespace

FILE* rnr_utf8_fopen(const char* name, const char* mode) {
    const std::wstring wname = wide(name), wmode = wide(mode);
    if (wname.empty() || wmode.empty()) return std::fopen(name, mode); // Not UTF-8: as the C library would
    return _wfopen(wname.c_str(), wmode.c_str());
}

FILE* rnr_utf8_freopen(const char* name, const char* mode, FILE* stream) {
    const std::wstring wname = wide(name), wmode = wide(mode);
    if (wname.empty() || wmode.empty()) return std::freopen(name, mode, stream);
    return _wfreopen(wname.c_str(), wmode.c_str(), stream);
}
#endif
