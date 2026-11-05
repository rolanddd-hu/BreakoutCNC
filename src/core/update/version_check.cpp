#include "core/update/version_check.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <mutex>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winhttp.h>
#endif

namespace rnr::core {

namespace {
constexpr size_t kMaxResponseBytes = 1024 * 1024;

#ifdef _WIN32
std::string fetchLatestRelease() {
    struct Handle {
        HINTERNET h = nullptr;
        ~Handle() { if (h) WinHttpCloseHandle(h); }
    };

#ifdef WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY
    const DWORD access = WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY;
#else
    const DWORD access = WINHTTP_ACCESS_TYPE_DEFAULT_PROXY;
#endif
    Handle session{WinHttpOpen(L"BreakoutCNC-update-check", access, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0)};
    if (!session.h) return {};
    WinHttpSetTimeouts(session.h, 5000, 5000, 5000, 5000);

    Handle connection{WinHttpConnect(session.h, L"api.github.com", INTERNET_DEFAULT_HTTPS_PORT, 0)};
    if (!connection.h) return {};
    Handle request{WinHttpOpenRequest(connection.h, L"GET", L"/repos/rolanddd-hu/BreakoutCNC/releases/latest", nullptr,
                                      WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE)};
    if (!request.h) return {};

    const wchar_t* headers = L"Accept: application/vnd.github+json\r\n";
    if (!WinHttpSendRequest(request.h, headers, static_cast<DWORD>(-1L), WINHTTP_NO_REQUEST_DATA, 0, 0, 0) ||
        !WinHttpReceiveResponse(request.h, nullptr)) {
        return {};
    }

    DWORD status = 0;
    DWORD status_size = sizeof(status);
    if (!WinHttpQueryHeaders(request.h, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX,
                             &status, &status_size, WINHTTP_NO_HEADER_INDEX) ||
        status != 200) {
        return {};
    }

    std::string body;
    char buffer[8192];
    DWORD bytes_read = 0;
    while (body.size() < kMaxResponseBytes && WinHttpReadData(request.h, buffer, sizeof(buffer), &bytes_read) && bytes_read > 0) {
        body.append(buffer, bytes_read);
    }
    return body;
}
#else
// Linux / macOS: uses the system's curl, so the app does not need a TLS library. Without curl there is no check.
std::string fetchLatestRelease() {
    FILE* pipe = popen("curl -fsSL --max-time 15 -A BreakoutCNC-update-check -H 'Accept: application/vnd.github+json' "
                       "https://api.github.com/repos/rolanddd-hu/BreakoutCNC/releases/latest 2>/dev/null", "r");
    if (!pipe) return {};
    std::string body;
    char buffer[8192];
    size_t bytes_read = 0;
    while (body.size() < kMaxResponseBytes && (bytes_read = std::fread(buffer, 1, sizeof(buffer), pipe)) > 0) {
        body.append(buffer, bytes_read);
    }
    const int status = pclose(pipe);
    return status == 0 ? body : std::string{};
}
#endif
} // namespace

bool parseVersion(const std::string& text, std::vector<int>& out) {
    out.clear();
    size_t pos = (!text.empty() && (text[0] == 'v' || text[0] == 'V')) ? 1 : 0;
    while (true) {
        const size_t start = pos;
        int value = 0;
        while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos]))) {
            if (pos - start >= 6) return false; // Not a version number, and would not fit an int for long
            value = value * 10 + (text[pos] - '0');
            ++pos;
        }
        if (pos == start) return false;
        out.push_back(value);
        if (pos == text.size()) return out.size() <= 4;
        if (text[pos] != '.') return false;
        ++pos;
    }
}

int compareVersions(const std::vector<int>& a, const std::vector<int>& b) {
    const size_t count = std::max(a.size(), b.size());
    for (size_t i = 0; i < count; ++i) {
        const int x = (i < a.size()) ? a[i] : 0;
        const int y = (i < b.size()) ? b[i] : 0;
        if (x != y) return (x < y) ? -1 : 1;
    }
    return 0;
}

std::string latestTagFromReleaseJson(const std::string& json) {
    const std::string key = "\"tag_name\"";
    size_t pos = json.find(key);
    if (pos == std::string::npos) return {};
    pos += key.size();
    const auto skipSpace = [&] {
        while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) ++pos;
    };
    skipSpace();
    if (pos >= json.size() || json[pos] != ':') return {};
    ++pos;
    skipSpace();
    if (pos >= json.size() || json[pos] != '"') return {};
    ++pos;
    const size_t end = json.find('"', pos);
    if (end == std::string::npos) return {};
    return json.substr(pos, end - pos);
}

std::string newerVersionInReleaseJson(const std::string& json, const std::string& current) {
    const std::string tag = latestTagFromReleaseJson(json);
    std::vector<int> latest_numbers, current_numbers;
    if (!parseVersion(tag, latest_numbers) || !parseVersion(current, current_numbers)) return {};
    if (compareVersions(latest_numbers, current_numbers) <= 0) return {};
    return (tag[0] == 'v' || tag[0] == 'V') ? tag.substr(1) : tag;
}

struct UpdateChecker::State {
    mutable std::mutex mutex;
    std::string available_version;
};

void UpdateChecker::start(const std::string& current_version) {
    if (state_) return;
    state_ = std::make_shared<State>();
    // An exception must never leave this thread or the thread creation: it would end the app, and a job with it
    try {
        std::thread([state = state_, current_version] {
            try {
                std::string version = newerVersionInReleaseJson(fetchLatestRelease(), current_version);
                std::lock_guard<std::mutex> lock(state->mutex);
                state->available_version = std::move(version);
            } catch (...) {
            }
        }).detach();
    } catch (...) {
    }
}

std::string UpdateChecker::availableVersion() const {
    if (!state_) return {};
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->available_version;
}

} // namespace rnr::core
