#include "core/camera/camera_stream_server.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <map>
#include <random>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSock = INVALID_SOCKET;
inline void closeSocketCrossPlatform(SocketHandle s) { ::closesocket(s); }
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
using SocketHandle = int;
constexpr SocketHandle kInvalidSock = -1;
inline void closeSocketCrossPlatform(SocketHandle s) { ::close(s); }
#endif

namespace rnr::core {

namespace {

// ============================================================================
// Compact Self-Contained Baseline JPEG Encoder (Standard JFIF 8x8 DCT + Huffman)
// ============================================================================
static const uint8_t kZigZag[64] = {
     0,  1,  5,  6, 14, 15, 27, 28,
     2,  4,  7, 13, 16, 26, 29, 42,
     3,  8, 12, 17, 25, 30, 41, 43,
     9, 11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54,
    20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61,
    35, 36, 48, 49, 57, 58, 62, 63
};

static const uint8_t kStdQuantY[64] = {
    16, 11, 10, 16,  24,  40,  51,  61,
    12, 12, 14, 19,  26,  58,  60,  55,
    14, 13, 16, 24,  40,  57,  69,  56,
    14, 17, 22, 29,  51,  87,  80,  62,
    18, 22, 37, 56,  68, 109, 103,  77,
    24, 35, 55, 64,  81, 104, 113,  92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103,  99
};

static const uint8_t kStdQuantC[64] = {
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99
};

static const uint8_t kDcLumBits[16] = {0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
static const uint8_t kDcLumVal[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static const uint8_t kDcChrBits[16] = {0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
static const uint8_t kDcChrVal[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};

static const uint8_t kAcLumBits[16] = {0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d};
static const uint8_t kAcLumVal[162] = {
    0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
    0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,
    0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
    0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
    0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
    0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
    0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,
    0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
    0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa
};

static const uint8_t kAcChrBits[16] = {0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77};
static const uint8_t kAcChrVal[162] = {
    0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
    0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,
    0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
    0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,
    0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
    0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,
    0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
    0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa
};

struct HuffTable {
    uint16_t code[256]{};
    uint8_t  len[256]{};
    void build(const uint8_t bits[16], const uint8_t* vals) {
        std::memset(code, 0, sizeof(code));
        std::memset(len, 0, sizeof(len));
        uint16_t hcode = 0;
        int k = 0;
        for (int i = 0; i < 16; ++i) {
            for (int j = 0; j < bits[i]; ++j) {
                uint8_t sym = vals[k++];
                code[sym] = hcode++;
                len[sym]  = static_cast<uint8_t>(i + 1);
            }
            hcode <<= 1;
        }
    }
};

struct BitWriter {
    std::vector<uint8_t>& out;
    uint32_t bit_buf = 0;
    int      bit_cnt = 0;

    void putBits(uint16_t bits, int n) {
        bit_buf = (bit_buf << n) | (bits & ((1u << n) - 1u));
        bit_cnt += n;
        while (bit_cnt >= 8) {
            bit_cnt -= 8;
            uint8_t b = static_cast<uint8_t>((bit_buf >> bit_cnt) & 0xFF);
            out.push_back(b);
            if (b == 0xFF) out.push_back(0x00); // Byte stuffing
        }
    }

    void flush() {
        if (bit_cnt > 0) {
            putBits(0x7F, 8 - bit_cnt);
            bit_cnt = 0;
        }
    }
};

void fdct8x8(float block[64]) {
    static const float c1 = 0.98078528f, c2 = 0.92387953f, c3 = 0.83146961f;
    static const float c4 = 0.70710678f, c5 = 0.55557023f, c6 = 0.38268343f, c7 = 0.19509032f;
    auto dct1d = [](float* v, int stride) {
        float s07 = v[0*stride] + v[7*stride], d07 = v[0*stride] - v[7*stride];
        float s16 = v[1*stride] + v[6*stride], d16 = v[1*stride] - v[6*stride];
        float s25 = v[2*stride] + v[5*stride], d25 = v[2*stride] - v[5*stride];
        float s34 = v[3*stride] + v[4*stride], d34 = v[3*stride] - v[4*stride];

        float e0 = s07 + s34, e3 = s07 - s34;
        float e1 = s16 + s25, e2 = s16 - s25;

        v[0*stride] = (e0 + e1) * 0.35355339f;
        v[4*stride] = (e0 - e1) * 0.35355339f;
        v[2*stride] = (e3 * c2 + e2 * c6) * 0.5f;
        v[6*stride] = (e3 * c6 - e2 * c2) * 0.5f;

        v[1*stride] = (d07 * c1 + d16 * c3 + d25 * c5 + d34 * c7) * 0.5f;
        v[3*stride] = (d07 * c3 - d16 * c7 - d25 * c1 - d34 * c5) * 0.5f;
        v[5*stride] = (d07 * c5 - d16 * c1 + d25 * c7 + d34 * c3) * 0.5f;
        v[7*stride] = (d07 * c7 - d16 * c5 + d25 * c3 - d34 * c1) * 0.5f;
    };
    for (int i = 0; i < 8; ++i) dct1d(block + i * 8, 1);
    for (int i = 0; i < 8; ++i) dct1d(block + i, 8);
}

int encodeBlock(BitWriter& bw, float block[64], const uint8_t quant[64], int prev_dc,
                const HuffTable& dc_ht, const HuffTable& ac_ht)
{
    fdct8x8(block);
    int zz[64];
    for (int i = 0; i < 64; ++i) {
        float q = block[i] / static_cast<float>(quant[i]);
        zz[kZigZag[i]] = static_cast<int>(q >= 0.0f ? (q + 0.5f) : (q - 0.5f));
    }

    auto bitCat = [](int v, int& mag, int& nbits) {
        int a = v < 0 ? -v : v;
        nbits = 0;
        while (a > 0) { a >>= 1; ++nbits; }
        mag = v < 0 ? (v + (1 << nbits) - 1) : v;
    };

    int diff = zz[0] - prev_dc;
    int mag = 0, nbits = 0;
    bitCat(diff, mag, nbits);
    bw.putBits(dc_ht.code[nbits], dc_ht.len[nbits]);
    if (nbits > 0) bw.putBits(static_cast<uint16_t>(mag), nbits);

    int zero_run = 0;
    for (int i = 1; i < 64; ++i) {
        if (zz[i] == 0) {
            ++zero_run;
        } else {
            while (zero_run >= 16) {
                bw.putBits(ac_ht.code[0xF0], ac_ht.len[0xF0]);
                zero_run -= 16;
            }
            bitCat(zz[i], mag, nbits);
            uint8_t sym = static_cast<uint8_t>((zero_run << 4) | nbits);
            bw.putBits(ac_ht.code[sym], ac_ht.len[sym]);
            bw.putBits(static_cast<uint16_t>(mag), nbits);
            zero_run = 0;
        }
    }
    if (zero_run > 0) {
        bw.putBits(ac_ht.code[0x00], ac_ht.len[0x00]); // EOB
    }
    return zz[0];
}

const char* stateToString(ControllerState st) {
    switch (st) {
    case ControllerState::Disconnected:   return "DISCONNECTED";
    case ControllerState::EStop:          return "E-STOP ACTIVE";
    case ControllerState::Idle:           return "READY / IDLE";
    case ControllerState::RunningGCode:   return "RUNNING G-CODE";
    case ControllerState::PausedHold:     return "FEED HOLD (PAUSED)";
    case ControllerState::Jogging:        return "MANUAL JOGGING";
    case ControllerState::Homing:         return "HOMING AXES";
    case ControllerState::Probing:        return "PROBING";
    case ControllerState::DwellPierce:    return "PIERCE / DWELL";
    case ControllerState::ToolChangeHold: return "TOOL CHANGE (M6)";
    default:                              return "IDLE";
    }
}

using Clock = std::chrono::steady_clock;

bool sendAllBytes(SocketHandle s, const void* data, size_t len) {
    const char* ptr = static_cast<const char*>(data);
    size_t sent = 0;
    while (sent < len) {
        int n = ::send(s, ptr + sent, static_cast<int>(len - sent), 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

std::string toLower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string trimWs(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string urlDecode(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '%' && i + 2 < in.size()) {
            char hex[3] = {in[i+1], in[i+2], 0};
            char* end = nullptr;
            long val = std::strtol(hex, &end, 16);
            if (end == hex + 2) {
                out += static_cast<char>(val);
                i += 2;
                continue;
            }
        } else if (in[i] == '+') {
            out += ' ';
            continue;
        }
        out += in[i];
    }
    return out;
}

std::string escapeJsonString(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\r') out += "\\r";
        else if (c == '\n') out += "\\n";
        else if (c == '\t') out += "\\t";
        else if (static_cast<unsigned char>(c) < 0x20) {
            char u[8];
            std::snprintf(u, sizeof(u), "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
            out += u;
        } else out += c;
    }
    return out;
}

std::string jsonError(const std::string& msg) {
    return "{\"ok\":false,\"error\":\"" + escapeJsonString(msg) + "\"}";
}

// A multipart/form-data upload (curl -F): the first part's content and file name; anything else is the file itself
std::string extractFileContent(const std::string& content_type, const std::string& body, std::string& io_filename) {
    if (toLower(content_type).find("multipart/form-data") == std::string::npos) return body;
    auto b_pos = content_type.find("boundary=");
    if (b_pos == std::string::npos) return body;
    b_pos += 9;
    size_t b_end = content_type.find_first_of(";\r\n", b_pos);
    std::string boundary = "--" + trimWs(content_type.substr(b_pos, (b_end == std::string::npos) ? std::string::npos : (b_end - b_pos)));
    if (boundary.size() > 2 && boundary[2] == '"' && boundary.back() == '"') boundary = "--" + boundary.substr(3, boundary.size() - 4);
    size_t p0 = body.find(boundary);
    if (p0 == std::string::npos) return body;
    size_t p_hdr_end = body.find("\r\n\r\n", p0);
    if (p_hdr_end == std::string::npos) return body;
    std::string part_hdr = body.substr(p0, p_hdr_end - p0);
    auto fn_pos = part_hdr.find("filename=\"");
    if (fn_pos != std::string::npos) {
        fn_pos += 10;
        size_t fn_end = part_hdr.find('"', fn_pos);
        if (fn_end != std::string::npos) io_filename = part_hdr.substr(fn_pos, fn_end - fn_pos);
    }
    size_t data_start = p_hdr_end + 4;
    size_t p_next = body.find(boundary, data_start);
    if (p_next == std::string::npos) return body.substr(data_start);
    size_t data_len = (p_next >= data_start + 2 && body[p_next - 2] == '\r' && body[p_next - 1] == '\n')
                          ? (p_next - 2 - data_start)
                          : (p_next - data_start);
    return body.substr(data_start, data_len);
}

struct HttpRequest {
    std::string method, path, query;
    std::map<std::string, std::string> headers; // Lower-case names, first occurrence
    std::string header(const char* name) const {
        auto it = headers.find(name);
        return it == headers.end() ? std::string() : it->second;
    }
};

bool parseRequestHead(const std::string& head, HttpRequest& rq) {
    std::istringstream in(head);
    std::string line;
    if (!std::getline(in, line)) return false;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const size_t sp1 = line.find(' ');
    if (sp1 == std::string::npos) return false;
    const size_t sp2 = line.find(' ', sp1 + 1);
    rq.method = line.substr(0, sp1);
    std::string target = line.substr(sp1 + 1, sp2 == std::string::npos ? std::string::npos : sp2 - sp1 - 1);
    if (rq.method.empty() || target.empty() || target[0] != '/') return false;
    const size_t q = target.find('?');
    rq.path = target.substr(0, q);
    if (q != std::string::npos) rq.query = target.substr(q + 1);
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const size_t colon = line.find(':');
        if (colon == std::string::npos || colon == 0) continue;
        rq.headers.emplace(toLower(trimWs(line.substr(0, colon))), trimWs(line.substr(colon + 1)));
    }
    return true;
}

// Raw (still URL-encoded) value of a query parameter
std::string queryParam(const std::string& query, const std::string& name) {
    size_t pos = 0;
    while (pos <= query.size()) {
        size_t amp = query.find('&', pos);
        if (amp == std::string::npos) amp = query.size();
        const std::string pair = query.substr(pos, amp - pos);
        const size_t eq = pair.find('=');
        if (urlDecode(pair.substr(0, eq)) == name) return eq == std::string::npos ? "" : pair.substr(eq + 1);
        pos = amp + 1;
    }
    return "";
}

std::string cookieValue(const std::string& cookie_header, const std::string& name) {
    size_t pos = 0;
    while (pos < cookie_header.size()) {
        size_t semi = cookie_header.find(';', pos);
        if (semi == std::string::npos) semi = cookie_header.size();
        const std::string pair = trimWs(cookie_header.substr(pos, semi - pos));
        const size_t eq = pair.find('=');
        if (eq != std::string::npos && pair.substr(0, eq) == name) return pair.substr(eq + 1);
        pos = semi + 1;
    }
    return "";
}

const char* statusText(int code) {
    switch (code) {
    case 200: return "OK";
    case 400: return "Bad Request";
    case 401: return "Unauthorized";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 411: return "Length Required";
    case 413: return "Payload Too Large";
    case 429: return "Too Many Requests";
    case 431: return "Request Header Fields Too Large";
    case 503: return "Service Unavailable";
    default:  return "Error";
    }
}

void sendResponse(SocketHandle s, int code, const char* content_type, const std::string& body,
                  const std::string& extra_headers = "") {
    std::string r = "HTTP/1.1 " + std::to_string(code) + " " + statusText(code) + "\r\n";
    r += "Content-Type: ";
    r += content_type;
    r += "\r\nCache-Control: no-cache, no-store\r\nX-Content-Type-Options: nosniff\r\n";
    r += extra_headers;
    r += "Content-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n";
    r += body;
    sendAllBytes(s, r.data(), r.size());
}

void sendJson(SocketHandle s, int code, const std::string& json, const std::string& extra_headers = "") {
    sendResponse(s, code, "application/json", json, extra_headers);
}

// 128 random bits as hex (std::random_device is the OS's cryptographic generator on MSVC and glibc)
std::string randomToken() {
    std::random_device rd;
    std::string t;
    char buf[9];
    for (int i = 0; i < 4; ++i) {
        std::snprintf(buf, sizeof(buf), "%08x", static_cast<unsigned>(rd()));
        t += buf;
    }
    return t;
}

bool constantTimeEquals(const std::string& a, const std::string& b) {
    unsigned diff = static_cast<unsigned>(a.size() ^ b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        diff |= static_cast<unsigned char>(a[i]) ^ static_cast<unsigned char>(i < b.size() ? b[i] : 0);
    }
    return diff == 0;
}

void closeAndShutdown(SocketHandle s) {
#ifdef _WIN32
    ::shutdown(s, SD_BOTH);
#else
    ::shutdown(s, SHUT_RDWR);
#endif
    closeSocketCrossPlatform(s);
}

} // namespace

// ============================================================================

CameraStreamServer::CameraStreamServer(CameraManager& camera_mgr)
    : camera_mgr_(camera_mgr)
{
#ifdef _WIN32
    WSADATA wsa{};
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    char host_buf[256] = {};
    if (::gethostname(host_buf, sizeof(host_buf) - 1) == 0) machine_name_ = host_buf;
}

CameraStreamServer::~CameraStreamServer() {
    stop();
}

void CameraStreamServer::setLastError(const std::string& e) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = e;
}

std::string CameraStreamServer::lastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void CameraStreamServer::setAccessCode(const std::string& code) {
    const std::string c = trimWs(code);
    std::lock_guard<std::mutex> lock(auth_mutex_);
    if (c == access_code_) return;
    // A new code signs everybody out
    access_code_ = c;
    sessions_.clear();
    login_failures_.clear();
}

bool CameraStreamServer::sessionValid(const std::string& cookie_header) const {
    const std::string token = cookieValue(cookie_header, "bcnc_session");
    if (token.empty()) return false;
    std::lock_guard<std::mutex> lock(auth_mutex_);
    bool ok = false;
    for (const auto& s : sessions_) ok = constantTimeEquals(token, s) || ok;
    return ok;
}

void CameraStreamServer::applyConfig(const CameraConfig& cfg) {
    target_fps_.store(std::clamp(cfg.stream_fps, 1, 60));
    allow_upload_.store(cfg.stream_allow_upload);
    setAccessCode(cfg.stream_access_code);
    if (!cfg.stream_enabled) {
        stop();
        setLastError("");
        return;
    }
    const std::string addr = trimWs(cfg.stream_bind_address);
    if (running_.load() && port_.load() == cfg.stream_port && bind_address_ == addr) {
        return;
    }
    stop();

    // Bind here, on the caller's thread: stop() can then always close the socket the accept thread waits on
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(cfg.stream_port);
    if (addr.empty()) {
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, addr.c_str(), &sa.sin_addr) != 1) {
        setLastError("Invalid listen address: " + addr);
        return;
    }
    SocketHandle s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == kInvalidSock) {
        setLastError("Could not create the server socket");
        return;
    }
#ifdef _WIN32
    // No other program may bind the same port and take over the connections (SO_REUSEADDR would allow that)
    BOOL exclusive = TRUE;
    ::setsockopt(s, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&exclusive), sizeof(exclusive));
#else
    int opt = 1;
    ::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
#endif
    if (::bind(s, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) != 0 || ::listen(s, 16) != 0) {
        closeSocketCrossPlatform(s);
        const std::string where = (addr.empty() ? std::string("port ") : addr + ":") + std::to_string(cfg.stream_port);
        setLastError("Could not listen on " + where +
                     (addr.empty() ? ": the port is in use by another program"
                                   : ": the port is in use, or the address is no longer this PC's"));
        return;
    }
    listen_sock_  = static_cast<uintptr_t>(s);
    bind_address_ = addr;
    port_.store(cfg.stream_port);
    setLastError("");
    running_.store(true);
    accept_thread_ = std::thread(&CameraStreamServer::acceptLoop, this, listen_sock_);
}

void CameraStreamServer::updateTelemetry(const TelemetrySnapshot& telemetry, LinearUnits units, const std::string& app_title, const std::string& loaded_filename) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    latest_telemetry_ = telemetry;
    latest_units_     = units;
    app_title_        = app_title;
    loaded_filename_  = loaded_filename;
}

std::vector<std::string> CameraStreamServer::localIpv4Addresses() {
    std::vector<std::string> out;
    char host_buf[256] = {};
    if (::gethostname(host_buf, sizeof(host_buf) - 1) != 0) return out;
    struct addrinfo hints{}, *res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (::getaddrinfo(host_buf, nullptr, &hints, &res) != 0 || !res) return out;
    for (struct addrinfo* p = res; p != nullptr; p = p->ai_next) {
        auto* ipv4 = reinterpret_cast<struct sockaddr_in*>(p->ai_addr);
        char ip_buf[INET_ADDRSTRLEN] = {};
        if (!::inet_ntop(AF_INET, &(ipv4->sin_addr), ip_buf, sizeof(ip_buf))) continue;
        const std::string cand(ip_buf);
        if (cand.rfind("127.", 0) == 0 || cand.rfind("169.254.", 0) == 0) continue;
        if (std::find(out.begin(), out.end(), cand) == out.end()) out.push_back(cand);
    }
    ::freeaddrinfo(res);
    return out;
}

std::string CameraStreamServer::getServerUrl() const {
    std::string ip_str = bind_address_;
    if (ip_str.empty()) {
        const auto ips = localIpv4Addresses();
        ip_str = ips.empty() ? "127.0.0.1" : ips.front();
    }
    return "http://" + ip_str + ":" + std::to_string(port_.load());
}

bool CameraStreamServer::isAllowedHost(const std::string& host_header, const std::string& machine_name) {
    std::string h = toLower(trimWs(host_header));
    if (h.empty()) return false;
    if (h.front() == '[') {
        // IPv6 literal: the server only listens on IPv4, so only loopback can be meant
        const size_t e = h.find(']');
        return e != std::string::npos && h.substr(1, e - 1) == "::1";
    }
    const size_t colon = h.rfind(':');
    if (colon != std::string::npos) {
        const std::string port = h.substr(colon + 1);
        if (port.empty() || port.find_first_not_of("0123456789") != std::string::npos) return false;
        h = h.substr(0, colon);
    }
    if (!h.empty() && h.back() == '.') h.pop_back();
    if (h.empty()) return false;
    in_addr a{};
    if (::inet_pton(AF_INET, h.c_str(), &a) == 1) return true;
    if (h == "localhost") return true;
    // This PC's name, also with a local domain after it (mypc, mypc.local, mypc.fritz.box): a web site's own name
    // (DNS rebinding) is refused
    const std::string m = toLower(trimWs(machine_name));
    if (m.empty()) return false;
    return h == m || (h.size() > m.size() && h.compare(0, m.size(), m) == 0 && h[m.size()] == '.');
}

std::string CameraStreamServer::sanitizeUploadFileName(const std::string& name) {
    std::string base = name;
    const size_t slash = base.find_last_of("/\\");
    if (slash != std::string::npos) base = base.substr(slash + 1);
    std::string out;
    for (unsigned char c : base) {
        const bool safe = (c < 0x80 && std::isalnum(c)) || c == '.' || c == '-' || c == '_' || c == ' ' || c == '(' || c == ')';
        out += safe ? static_cast<char>(c) : '_';
    }
    // No leading or trailing dots and spaces (Windows drops trailing ones; ".." stays out)
    const size_t b = out.find_first_not_of(". ");
    if (b == std::string::npos) return "remote_program.nc";
    out = out.substr(b, out.find_last_not_of(". ") - b + 1);
    if (out.size() > 100) {
        const size_t dot = out.rfind('.');
        const std::string ext = (dot != std::string::npos && out.size() - dot <= 10) ? out.substr(dot) : "";
        out = out.substr(0, 100 - ext.size()) + ext;
    }
    // Windows opens a device instead of a file for these names, in any folder and with any extension
    std::string stem = toLower(out.substr(0, out.find('.')));
    while (!stem.empty() && stem.back() == ' ') stem.pop_back();
    const bool device = stem == "con" || stem == "prn" || stem == "aux" || stem == "nul" ||
                        (stem.size() == 4 && (stem.rfind("com", 0) == 0 || stem.rfind("lpt", 0) == 0) &&
                         std::isdigit(static_cast<unsigned char>(stem[3])));
    if (device) out = "_" + out;
    return out;
}

void CameraStreamServer::stop() {
    running_.store(false);
    if (listen_sock_ != ~static_cast<uintptr_t>(0)) {
        closeAndShutdown(static_cast<SocketHandle>(listen_sock_)); // Wakes the accept thread
        listen_sock_ = ~static_cast<uintptr_t>(0);
    }
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    // No new clients now. Unblock the open ones and wait for every client thread: they use this object and the
    // app's callbacks (an upload may still be loading)
    std::vector<ClientThread> threads;
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (uintptr_t s : client_socks_) {
#ifdef _WIN32
            ::shutdown(static_cast<SocketHandle>(s), SD_BOTH);
#else
            ::shutdown(static_cast<SocketHandle>(s), SHUT_RDWR);
#endif
        }
        threads.swap(client_threads_);
    }
    for (auto& ct : threads) {
        if (ct.thread.joinable()) ct.thread.join();
    }
}

void CameraStreamServer::reapFinishedClientsLocked() {
    for (auto it = client_threads_.begin(); it != client_threads_.end();) {
        if (it->done->load()) {
            if (it->thread.joinable()) it->thread.join();
            it = client_threads_.erase(it);
        } else {
            ++it;
        }
    }
}

void CameraStreamServer::acceptLoop(uintptr_t listen_sock) {
    const SocketHandle s = static_cast<SocketHandle>(listen_sock);
    while (running_.load()) {
        sockaddr_in client_addr{};
#ifdef _WIN32
        int addr_len = sizeof(client_addr);
#else
        socklen_t addr_len = sizeof(client_addr);
#endif
        SocketHandle client = ::accept(s, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client == kInvalidSock) {
            if (!running_.load()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        const uint32_t peer = client_addr.sin_addr.s_addr;

        std::lock_guard<std::mutex> lock(clients_mutex_);
        reapFinishedClientsLocked();
        // Bound resource use: limited connections in all and per address, so one client cannot take every slot
        const int from_peer = clients_per_addr_.count(peer) ? clients_per_addr_[peer] : 0;
        if (!running_.load() || active_clients_.load() >= kMaxClients || from_peer >= kMaxClientsPerAddr) {
            closeSocketCrossPlatform(client);
            continue;
        }
        // Each receive waits at most 2 s (requests also have an overall deadline); a stalled viewer is dropped
#ifdef _WIN32
        DWORD rcv_ms = 2000, snd_ms = 10000;
        ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&rcv_ms), sizeof(rcv_ms));
        ::setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&snd_ms), sizeof(snd_ms));
#else
        timeval rcv{2, 0}, snd{10, 0};
        ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &rcv, sizeof(rcv));
        ::setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
#endif
        clients_per_addr_[peer] = from_peer + 1;
        active_clients_.fetch_add(1);
        client_socks_.insert(static_cast<uintptr_t>(client));
        auto done = std::make_shared<std::atomic<bool>>(false);
        const uintptr_t raw = static_cast<uintptr_t>(client);
        client_threads_.push_back({std::thread([this, raw, peer, done] {
                                       handleClient(raw, peer);
                                       done->store(true);
                                   }),
                                   done});
    }
}

void CameraStreamServer::handleClient(uintptr_t raw_sock, uint32_t peer_addr) {
    serveRequest(raw_sock, peer_addr);
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_socks_.erase(raw_sock);
        auto it = clients_per_addr_.find(peer_addr);
        if (it != clients_per_addr_.end() && --it->second <= 0) clients_per_addr_.erase(it);
    }
    closeSocketCrossPlatform(static_cast<SocketHandle>(raw_sock));
    active_clients_.fetch_sub(1);
}

void CameraStreamServer::serveRequest(uintptr_t raw_sock, uint32_t peer_addr) {
    const SocketHandle client = static_cast<SocketHandle>(raw_sock);

    // Request head: at most 16 KB within 5 s in all (a client sending a byte now and then cannot hold a slot)
    constexpr size_t kMaxHead = 16384;
    const auto head_deadline = Clock::now() + std::chrono::seconds(5);
    std::string req;
    char chunk[4096];
    size_t head_end = std::string::npos;
    size_t delimiter_len = 4;
    while (req.size() < kMaxHead && Clock::now() < head_deadline && running_.load()) {
        const int n = ::recv(client, chunk, sizeof(chunk), 0);
        if (n <= 0) break;
        req.append(chunk, static_cast<size_t>(n));
        head_end = req.find("\r\n\r\n");
        if (head_end != std::string::npos) { delimiter_len = 4; break; }
        head_end = req.find("\n\n");
        if (head_end != std::string::npos) { delimiter_len = 2; break; }
    }
    if (head_end == std::string::npos) {
        if (req.size() >= kMaxHead) sendJson(client, 431, jsonError("Request header too large"));
        return;
    }
    HttpRequest rq;
    if (!parseRequestHead(req.substr(0, head_end), rq)) {
        sendJson(client, 400, jsonError("Bad request"));
        return;
    }
    const std::string leftover = req.substr(head_end + delimiter_len);

    // Reads a request body of known length (part of it may have come with the head), within a deadline
    auto readBody = [&](size_t len, std::chrono::seconds limit, std::string& body) -> bool {
        body = leftover.substr(0, std::min(leftover.size(), len));
        body.reserve(len);
        const auto deadline = Clock::now() + limit;
        while (body.size() < len && running_.load() && Clock::now() < deadline) {
            const size_t want = std::min(sizeof(chunk), len - body.size());
            const int n = ::recv(client, chunk, static_cast<int>(want), 0);
            if (n <= 0) break;
            body.append(chunk, static_cast<size_t>(n));
        }
        return body.size() == len;
    };
    auto contentLength = [&](size_t& out) -> bool {
        const std::string v = rq.header("content-length");
        if (v.empty() || v.find_first_not_of("0123456789") != std::string::npos || v.size() > 12) return false;
        out = static_cast<size_t>(std::strtoull(v.c_str(), nullptr, 10));
        return true;
    };

    // 1. The Host must name this server: a web site that points its own name at this PC (DNS rebinding) is refused
    if (!isAllowedHost(rq.header("host"), machine_name_)) {
        sendResponse(client, 403, "text/plain; charset=utf-8",
                     "Unknown host name. Open the address shown in BreakoutCNC (Camera settings).");
        return;
    }
    // 2. A request the browser itself marks as coming from another site (an image, a script or a form there)
    const std::string fetch_site = toLower(rq.header("sec-fetch-site"));
    if (fetch_site == "cross-site") {
        sendJson(client, 403, jsonError("Cross-site request refused"));
        return;
    }
    const bool is_get  = rq.method == "GET";
    const bool is_post = rq.method == "POST";
    if (is_post) {
        // 3. Actions only from the dashboard page itself. Its own header makes a browser ask first (CORS preflight)
        //    before another site could send it, and this server never allows that; the Origin must be this server.
        if (rq.header("x-breakoutcnc").empty()) {
            sendJson(client, 403, jsonError("Request refused: not sent by the BreakoutCNC page"));
            return;
        }
        const std::string origin = toLower(trimWs(rq.header("origin")));
        if (!origin.empty() && origin != "http://" + toLower(trimWs(rq.header("host")))) {
            sendJson(client, 403, jsonError("Request refused: sent from another site"));
            return;
        }
        if (!fetch_site.empty() && fetch_site != "same-origin" && fetch_site != "none") {
            sendJson(client, 403, jsonError("Request refused: sent from another site"));
            return;
        }
    }

    const std::string& path = rq.path;

    if (path == "/" || path == "/index.html") {
        if (!is_get) { sendJson(client, 405, jsonError("Use GET")); return; }
        const std::string html = buildDashboardHtml();
        sendResponse(client, 200, "text/html; charset=utf-8", html,
                     "Content-Security-Policy: default-src 'none'; script-src 'unsafe-inline'; "
                     "style-src 'unsafe-inline'; img-src 'self'; connect-src 'self'; frame-ancestors 'none'; "
                     "form-action 'none'; base-uri 'none'\r\n"
                     "X-Frame-Options: DENY\r\nReferrer-Policy: no-referrer\r\n");
        return;
    }

    if (path == "/api/login") {
        if (!is_post) { sendJson(client, 405, jsonError("Use POST")); return; }
        size_t len = 0;
        if (!contentLength(len)) { sendJson(client, 411, jsonError("Content-Length required")); return; }
        if (len > 256) { sendJson(client, 413, jsonError("Too long")); return; }
        std::string body;
        if (!readBody(len, std::chrono::seconds(5), body)) { sendJson(client, 400, jsonError("Incomplete request")); return; }
        const std::string code = trimWs(body);
        const auto now = Clock::now();
        int status = 200;
        std::string json, extra;
        {
            std::lock_guard<std::mutex> lock(auth_mutex_);
            if (login_failures_.size() > 1024) login_failures_.clear();
            if (access_code_.empty()) {
                json = "{\"ok\":true}";
            } else {
                LoginFailures& f = login_failures_[peer_addr];
                if (now < f.locked_until) {
                    const auto wait_s = std::chrono::duration_cast<std::chrono::seconds>(f.locked_until - now).count() + 1;
                    status = 429;
                    json = jsonError("Too many wrong codes: wait " + std::to_string(wait_s) + " s");
                } else if (constantTimeEquals(code, access_code_)) {
                    login_failures_.erase(peer_addr);
                    const std::string token = randomToken();
                    sessions_.push_back(token);
                    while (sessions_.size() > 64) sessions_.pop_front();
                    json = "{\"ok\":true}";
                    extra = "Set-Cookie: bcnc_session=" + token + "; Path=/; HttpOnly; SameSite=Strict\r\n";
                } else {
                    if (++f.count >= kMaxLoginFailures) {
                        f.count = 0;
                        f.locked_until = now + std::chrono::seconds(60);
                    }
                    status = 401;
                    json = jsonError("Wrong access code");
                }
            }
        }
        sendJson(client, status, json, extra);
        return;
    }

    bool code_set = false;
    {
        std::lock_guard<std::mutex> lock(auth_mutex_);
        code_set = !access_code_.empty();
    }
    const bool authed = !code_set || sessionValid(rq.header("cookie"));
    const std::string need_login = "{\"ok\":false,\"login\":true,\"error\":\"Enter the access code\"}";

    if (path == "/api/telemetry") {
        if (!is_get) { sendJson(client, 405, jsonError("Use GET")); return; }
        if (!authed) { sendJson(client, 401, need_login); return; }
        sendJson(client, 200, buildTelemetryJson());
        return;
    }

    if (path == "/api/estop") {
        // State-changing endpoints are POST-only (a link preview / prefetch must not act)
        if (!is_post) { sendJson(client, 405, jsonError("Use POST")); return; }
        if (!authed) { sendJson(client, 401, need_login); return; }
        if (on_estop_) on_estop_();
        sendJson(client, 200, "{\"ok\":true,\"state\":\"E-STOP ACTIVE\"}");
        return;
    }

    if (path == "/api/upload_gcode") {
        if (!is_post) { sendJson(client, 405, jsonError("Use POST")); return; }
        if (!authed) { sendJson(client, 401, need_login); return; }
        // Nothing of the body is read unless an upload is allowed
        if (!allow_upload_.load()) {
            sendJson(client, 403, jsonError("Remote G-Code upload is disabled in BreakoutCNC (Camera settings)"));
            return;
        }
        if (!on_load_gcode_) {
            sendJson(client, 503, jsonError("Remote G-Code loading is not enabled in BreakoutCNC"));
            return;
        }
        size_t len = 0;
        if (!rq.header("transfer-encoding").empty() || !contentLength(len)) {
            sendJson(client, 411, jsonError("Send the file with a Content-Length"));
            return;
        }
        if (len > kMaxUploadBytes) { sendJson(client, 413, jsonError("Upload too large")); return; }
        if (len == 0) { sendJson(client, 400, jsonError("Uploaded file is empty")); return; }
        // One upload at a time
        if (upload_busy_.exchange(true)) {
            sendJson(client, 429, jsonError("Another upload is in progress"));
            return;
        }
        struct BusyRelease {
            std::atomic<bool>& flag;
            ~BusyRelease() { flag.store(false); }
        } release{upload_busy_};

        std::string body;
        if (!readBody(len, std::chrono::seconds(120), body)) {
            sendJson(client, 400, jsonError("Upload incomplete"));
            return;
        }
        std::string filename = queryParam(rq.query, "filename");
        if (filename.empty()) filename = rq.header("x-filename");
        filename = urlDecode(filename);
        const std::string content = extractFileContent(rq.header("content-type"), body, filename);
        filename = sanitizeUploadFileName(filename);
        if (content.empty()) { sendJson(client, 400, jsonError("Uploaded file is empty")); return; }

        const LoadGCodeResult res = on_load_gcode_(filename, content);
        if (res.success) {
            sendJson(client, 200, "{\"ok\":true,\"filename\":\"" + escapeJsonString(filename) +
                                      "\",\"lines\":" + std::to_string(res.line_count) + "}");
        } else {
            sendJson(client, 400, jsonError(res.error_message));
        }
        return;
    }

    if (path == "/stream.mjpg") {
        if (!is_get) { sendJson(client, 405, jsonError("Use GET")); return; }
        if (!authed) { sendJson(client, 401, need_login); return; }
        // Camera viewers hold their connection: a few at most, so the page and E-stop always find a free one
        if (active_streams_.fetch_add(1) >= kMaxStreams) {
            active_streams_.fetch_sub(1);
            sendJson(client, 503, jsonError("Too many camera viewers (" + std::to_string(kMaxStreams) + ")"));
            return;
        }
        struct StreamRelease {
            std::atomic<int>& n;
            ~StreamRelease() { n.fetch_sub(1); }
        } release{active_streams_};

        const std::string hdr =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "Cache-Control: no-cache, no-store, must-revalidate\r\n"
            "Pragma: no-cache\r\n"
            "X-Content-Type-Options: nosniff\r\n"
            "Connection: close\r\n\r\n";
        if (!sendAllBytes(client, hdr.data(), hdr.size())) return;
        CameraFrame frame;
        std::vector<uint8_t> jpeg;
        while (running_.load()) {
            // A viewer that left is noticed even while no frame is sent (no camera selected): its socket turns
            // readable and reads as closed. Anything a viewer sends after the request is not expected.
            {
                fd_set rd;
                FD_ZERO(&rd);
                FD_SET(client, &rd);
                timeval tv{0, 0};
                if (::select(static_cast<int>(client) + 1, &rd, nullptr, nullptr, &tv) != 0) {
                    char probe[256];
                    if (::recv(client, probe, sizeof(probe), 0) <= 0) break;
                }
            }
            bool got = camera_mgr_.copyCurrentFrame(CameraRole::Monitoring, frame);
            if (!got) got = camera_mgr_.copyCurrentFrame(CameraRole::TopDown, frame);
            if (got && !frame.rgba.empty()) {
                encodeRgbaToJpeg(frame.rgba.data(), frame.width, frame.height, jpeg);
                char part_hdr[128];
                const int plen = std::snprintf(part_hdr, sizeof(part_hdr),
                                               "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                                               jpeg.size());
                if (!sendAllBytes(client, part_hdr, static_cast<size_t>(plen)) ||
                    !sendAllBytes(client, jpeg.data(), jpeg.size()) ||
                    !sendAllBytes(client, "\r\n", 2)) {
                    break;
                }
            }
            const int fps = std::clamp(target_fps_.load(), 1, 60);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));
        }
        return;
    }

    sendJson(client, 404, jsonError("Not found"));
}

std::string CameraStreamServer::buildTelemetryJson() const {
    TelemetrySnapshot t;
    LinearUnits units = LinearUnits::Metric;
    std::string title;
    std::string file_str;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        t        = latest_telemetry_;
        units    = latest_units_;
        title    = app_title_;
        file_str = loaded_filename_;
    }

    const bool imp = (units == LinearUnits::Imperial);
    const double scale = imp ? (1.0 / 25.4) : 1.0;
    const char* unit_lbl = imp ? "in" : "mm";

    double prog_pct = 0.0;
    if (t.total_segments > 0) {
        prog_pct = std::clamp(
            ((static_cast<double>(t.active_segment_index) + t.segment_progress_0_to_1) /
             static_cast<double>(t.total_segments)) * 100.0,
            0.0, 100.0);
    }

    std::string base_file = "None";
    if (!file_str.empty()) {
        try {
            base_file = std::filesystem::path(file_str).filename().string();
        } catch (...) {
            base_file = file_str;
        }
    }

    auto finite = [](double v) { return std::isfinite(v) ? v : 0.0; };
    char nums[512];
    std::snprintf(
        nums, sizeof(nums),
        "\"state\":\"%s\",\"estop\":%s,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,"
        "\"unit\":\"%s\",\"progress\":%.1f,\"elapsed\":%.1f,\"spindle\":%s,\"segments\":%zu",
        stateToString(t.state),
        (t.state == ControllerState::EStop) ? "true" : "false",
        finite(t.work_pos_mm[0] * scale),
        finite(t.work_pos_mm[1] * scale),
        finite(t.work_pos_mm[2] * scale),
        unit_lbl,
        finite(prog_pct),
        finite(t.job_elapsed_seconds),
        t.torch_or_spindle_on ? "true" : "false",
        t.total_segments);
    return "{\"title\":\"" + escapeJsonString(title) + "\",\"file\":\"" + escapeJsonString(base_file) + "\"," + nums + "}";
}

void CameraStreamServer::encodeRgbaToJpeg(const uint8_t* rgba, int width, int height, std::vector<uint8_t>& out) {
    out.clear();
    if (!rgba || width <= 0 || height <= 0) return;
    out.reserve(static_cast<size_t>(width * height / 2));

    auto put16 = [&](uint16_t v) {
        out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
        out.push_back(static_cast<uint8_t>(v & 0xFF));
    };

    // SOI + APP0 JFIF
    out.push_back(0xFF); out.push_back(0xD8);
    static const uint8_t kApp0[] = {
        0xFF,0xE0,0x00,0x10,'J','F','I','F',0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x01,0x00,0x00
    };
    out.insert(out.end(), std::begin(kApp0), std::end(kApp0));

    // DQT (Y=0, C=1)
    auto writeDqt = [&](uint8_t id, const uint8_t q[64]) {
        out.push_back(0xFF); out.push_back(0xDB);
        put16(67);
        out.push_back(id);
        for (int i = 0; i < 64; ++i) out.push_back(q[kZigZag[i]]); // ZigZag order
    };
    // Note: JPEG DQT expects table entries in zigzag scan order:
    uint8_t dqt_y[64], dqt_c[64];
    for (int i = 0; i < 64; ++i) {
        dqt_y[kZigZag[i]] = kStdQuantY[i];
        dqt_c[kZigZag[i]] = kStdQuantC[i];
    }
    out.push_back(0xFF); out.push_back(0xDB); put16(67); out.push_back(0);
    out.insert(out.end(), std::begin(dqt_y), std::end(dqt_y));
    out.push_back(0xFF); out.push_back(0xDB); put16(67); out.push_back(1);
    out.insert(out.end(), std::begin(dqt_c), std::end(dqt_c));

    // SOF0 (Baseline DCT, YCbCr 4:4:4)
    out.push_back(0xFF); out.push_back(0xC0);
    put16(17);
    out.push_back(8);
    put16(static_cast<uint16_t>(height));
    put16(static_cast<uint16_t>(width));
    out.push_back(3);
    out.push_back(1); out.push_back(0x11); out.push_back(0);
    out.push_back(2); out.push_back(0x11); out.push_back(1);
    out.push_back(3); out.push_back(0x11); out.push_back(1);

    // DHT (4 tables)
    auto writeDht = [&](uint8_t tc_th, const uint8_t bits[16], const uint8_t* vals, int nvals) {
        out.push_back(0xFF); out.push_back(0xC4);
        put16(static_cast<uint16_t>(3 + 16 + nvals));
        out.push_back(tc_th);
        out.insert(out.end(), bits, bits + 16);
        out.insert(out.end(), vals, vals + nvals);
    };
    writeDht(0x00, kDcLumBits, kDcLumVal, 12);
    writeDht(0x10, kAcLumBits, kAcLumVal, 162);
    writeDht(0x01, kDcChrBits, kDcChrVal, 12);
    writeDht(0x11, kAcChrBits, kAcChrVal, 162);

    // SOS
    static const uint8_t kSos[] = {
        0xFF,0xDA,0x00,0x0C,0x03,0x01,0x00,0x02,0x11,0x03,0x11,0x00,0x3F,0x00
    };
    out.insert(out.end(), std::begin(kSos), std::end(kSos));

    HuffTable dc_y, ac_y, dc_c, ac_c;
    dc_y.build(kDcLumBits, kDcLumVal);
    ac_y.build(kAcLumBits, kAcLumVal);
    dc_c.build(kDcChrBits, kDcChrVal);
    ac_c.build(kAcChrBits, kAcChrVal);

    BitWriter bw{out};
    int prev_dc_y = 0, prev_dc_cb = 0, prev_dc_cr = 0;
    float blk_y[64], blk_cb[64], blk_cr[64];

    for (int by = 0; by < height; by += 8) {
        for (int bx = 0; bx < width; bx += 8) {
            for (int dy = 0; dy < 8; ++dy) {
                int py = std::min(by + dy, height - 1);
                const uint8_t* row = rgba + static_cast<size_t>(py * width) * 4;
                for (int dx = 0; dx < 8; ++dx) {
                    int px = std::min(bx + dx, width - 1);
                    float r = row[px * 4 + 0];
                    float g = row[px * 4 + 1];
                    float b = row[px * 4 + 2];
                    int idx = dy * 8 + dx;
                    blk_y[idx]  =  0.299f * r + 0.587f * g + 0.114f * b - 128.0f;
                    blk_cb[idx] = -0.1687f * r - 0.3313f * g + 0.5f * b;
                    blk_cr[idx] =  0.5f * r - 0.4187f * g - 0.0813f * b;
                }
            }
            prev_dc_y  = encodeBlock(bw, blk_y,  kStdQuantY, prev_dc_y,  dc_y, ac_y);
            prev_dc_cb = encodeBlock(bw, blk_cb, kStdQuantC, prev_dc_cb, dc_c, ac_c);
            prev_dc_cr = encodeBlock(bw, blk_cr, kStdQuantC, prev_dc_cr, dc_c, ac_c);
        }
    }
    bw.flush();
    out.push_back(0xFF); out.push_back(0xD9); // EOI
}

std::string CameraStreamServer::buildDashboardHtml() const {
    return R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>BreakoutCNC Remote Monitor</title>
<style>
  body { margin:0; background:#0d1016; color:#e6edf3; font-family:Consolas,monospace,sans-serif; }
  .wrap { max-width:860px; margin:0 auto; padding:14px; }
  .hdr { display:flex; justify-content:space-between; align-items:center; background:#151a23; border:1px solid #283446; border-radius:8px; padding:10px 14px; margin-bottom:12px; }
  .brand { font-size:18px; font-weight:bold; color:#78c0ff; letter-spacing:1px; }
  .badge { padding:5px 12px; border-radius:5px; font-weight:bold; font-size:14px; background:#1f6f43; color:#fff; }
  .badge.estop { background:#c92a2a; }
  .cam-box { background:#06080b; border:1px solid #283446; border-radius:8px; overflow:hidden; text-align:center; margin-bottom:12px; }
  .cam-box img { width:100%; max-height:480px; object-fit:contain; display:block; }
  .grid { display:grid; grid-template-columns:repeat(3, 1fr); gap:10px; margin-bottom:12px; }
  .dro { background:#131820; border:1px solid #263244; border-radius:6px; padding:10px; text-align:center; }
  .dro .ax { color:#78c0ff; font-size:13px; margin-bottom:4px; }
  .dro .val { font-size:22px; font-weight:bold; }
  .meta { display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-bottom:14px; }
  .card { background:#131820; border:1px solid #283446; border-radius:8px; padding:14px; margin-bottom:14px; }
  .card-hdr { display:flex; justify-content:space-between; align-items:center; font-size:13px; font-weight:bold; color:#ffb703; letter-spacing:1px; margin-bottom:10px; }
  .prog-badge { font-size:11px; padding:3px 8px; border-radius:4px; background:#1b2432; color:#8b9bb4; }
  .file-row { display:flex; gap:10px; align-items:center; flex-wrap:wrap; margin-bottom:10px; }
  .btn { padding:9px 16px; font-size:14px; font-weight:bold; font-family:inherit; border-radius:6px; cursor:pointer; transition:all 0.15s ease; border:1px solid transparent; }
  .btn-browse { background:#16304f; color:#78c0ff; border-color:#78c0ff; }
  .btn-browse:hover { background:#1d3f66; }
  .btn-upload { background:#1b5e3a; color:#fff; border-color:#2ecc71; }
  .btn-upload:hover:not(:disabled) { background:#227a4b; }
  .btn-upload:disabled { opacity:0.4; cursor:not-allowed; }
  .fname-disp { flex:1; min-width:180px; background:#090c10; border:1px solid #212b3b; border-radius:5px; padding:8px 12px; font-size:13px; color:#c9d1d9; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .status-line { font-size:13px; min-height:18px; line-height:18px; margin-top:4px; }
  .prog-summary { display:flex; justify-content:space-between; font-size:12px; color:#8b9bb4; margin-top:8px; padding-top:8px; border-top:1px solid #1c2430; }
  .estop-btn { width:100%; padding:18px; font-size:20px; font-weight:bold; color:#fff; background:linear-gradient(180deg,#e03131,#a61e1e); border:2px solid #ff6b6b; border-radius:8px; cursor:pointer; letter-spacing:1px; }
  .estop-btn:active { background:#7d1414; }
  .note { font-size:12px; color:#8b9bb4; margin-top:8px; line-height:16px; }
  .code-in { flex:1; min-width:160px; background:#090c10; border:1px solid #212b3b; border-radius:5px; padding:8px 12px; font-size:15px; color:#e6edf3; font-family:inherit; }
  #login { display:none; }
</style>
</head>
<body>
<div class="wrap">
  <div class="hdr">
    <div class="brand" id="title">BREAKOUT CNC MONITOR</div>
    <div class="badge" id="state">CONNECTING...</div>
  </div>

  <div class="card" id="login">
    <div class="card-hdr"><span>ACCESS CODE</span></div>
    <form class="file-row" onsubmit="login(); return false;">
      <input class="code-in" type="password" id="code" autocomplete="current-password" placeholder="Access code set in BreakoutCNC"/>
      <button class="btn btn-upload" type="submit">Open</button>
    </form>
    <div class="status-line" id="login-status"></div>
  </div>

  <div id="main">
  <div class="cam-box">
    <img id="cam" src="/stream.mjpg" alt="Live CNC Camera Stream"/>
  </div>
  <div class="grid">
    <div class="dro"><div class="ax">X AXIS</div><div class="val" id="vx">0.000</div></div>
    <div class="dro"><div class="ax">Y AXIS</div><div class="val" id="vy">0.000</div></div>
    <div class="dro"><div class="ax">Z AXIS</div><div class="val" id="vz">0.000</div></div>
  </div>
  <div class="meta">
    <div class="dro"><div class="ax">PROGRESS</div><div class="val" id="prog">0.0%</div></div>
    <div class="dro"><div class="ax">ELAPSED</div><div class="val" id="elap">0.0s</div></div>
  </div>

  <div class="card">
    <div class="card-hdr">
      <span>REMOTE G-CODE PROGRAM</span>
      <span class="prog-badge">FILE TRANSFER</span>
    </div>
    <div class="file-row">
      <input type="file" id="gcode-file" accept=".nc,.tap,.gcode,.ngc,.cnc,.txt" style="display:none" onchange="onFileSelected(this)"/>
      <button class="btn btn-browse" onclick="document.getElementById('gcode-file').click()">Browse G-Code...</button>
      <div class="fname-disp" id="file-label">No file selected</div>
      <button class="btn btn-upload" id="upload-btn" onclick="uploadGCode()" disabled>Load File</button>
    </div>
    <div class="status-line" id="upload-status"></div>
    <div class="prog-summary">
      <div>Active Program: <strong id="cur-prog" style="color:#78c0ff">None</strong></div>
      <div>Toolpath Segments: <strong id="cur-segs" style="color:#78c0ff">0</strong></div>
    </div>
  </div>

  <button class="estop-btn" onclick="triggerEStop()">EMERGENCY STOP (E-STOP)</button>
  <div class="note">The remote E-stop depends on the network and the browser: it is not a safety device. Keep the
  machine's own E-stop within reach.</div>
  </div>
</div>
<script>
// Every action carries this header: the server refuses actions without it (other web sites cannot add it)
const API_HDR = {'X-BreakoutCNC': '1'};
let selectedFile = null;
let loginShown = false;

function showLogin(on) {
  if (on === loginShown) return;
  loginShown = on;
  document.getElementById('login').style.display = on ? 'block' : 'none';
  document.getElementById('main').style.display = on ? 'none' : 'block';
  if (on) {
    document.getElementById('state').textContent = 'LOCKED';
    document.getElementById('code').focus();
  } else {
    document.getElementById('cam').src = '/stream.mjpg?t=' + Date.now();
  }
}

async function login() {
  const st = document.getElementById('login-status');
  st.style.color = '#ffb703';
  st.textContent = 'Checking...';
  try {
    const r = await fetch('/api/login', {
      method: 'POST',
      headers: Object.assign({'Content-Type': 'text/plain; charset=utf-8'}, API_HDR),
      body: document.getElementById('code').value
    });
    const d = await r.json();
    if (d.ok) {
      document.getElementById('code').value = '';
      st.textContent = '';
      poll();
    } else {
      st.textContent = d.error || 'Refused';
      st.style.color = '#ff6b6b';
    }
  } catch (e) {
    st.textContent = 'Network error: ' + e.message;
    st.style.color = '#ff6b6b';
  }
}

function onFileSelected(input) {
  if (input.files && input.files.length > 0) {
    selectedFile = input.files[0];
    const szKb = (selectedFile.size / 1024).toFixed(1);
    document.getElementById('file-label').textContent = selectedFile.name + ' (' + szKb + ' KB)';
    document.getElementById('upload-btn').disabled = false;
    const st = document.getElementById('upload-status');
    st.textContent = 'Ready to load. Click "Load File" to transfer to BreakoutCNC.';
    st.style.color = '#8b9bb4';
  }
}

async function uploadGCode() {
  if (!selectedFile) return;
  const btn = document.getElementById('upload-btn');
  const st = document.getElementById('upload-status');
  btn.disabled = true;
  st.textContent = 'Transferring ' + selectedFile.name + ' to BreakoutCNC...';
  st.style.color = '#ffb703';

  try {
    const text = await selectedFile.text();
    const resp = await fetch('/api/upload_gcode?filename=' + encodeURIComponent(selectedFile.name), {
      method: 'POST',
      headers: Object.assign({'Content-Type': 'text/plain; charset=utf-8'}, API_HDR),
      body: text
    });
    if (resp.status === 401) { showLogin(true); btn.disabled = false; return; }
    const result = await resp.json();
    if (result.ok) {
      st.textContent = 'Successfully loaded into BreakoutCNC: ' + result.filename + ' (' + result.lines + ' segments)';
      st.style.color = '#00eb76';
      poll();
    } else {
      st.textContent = 'Load rejected: ' + (result.error || 'Server error');
      st.style.color = '#ff6b6b';
      btn.disabled = false;
    }
  } catch (err) {
    st.textContent = 'Upload network error: ' + err.message;
    st.style.color = '#ff6b6b';
    btn.disabled = false;
  }
}

async function poll() {
  try {
    const r = await fetch('/api/telemetry', {cache: 'no-store'});
    if (r.status === 401) { showLogin(true); return; }
    if (!r.ok) return;
    showLogin(false);
    const d = await r.json();
    document.getElementById('title').textContent = d.title;
    const st = document.getElementById('state');
    st.textContent = d.state;
    st.className = d.estop ? 'badge estop' : 'badge';
    document.getElementById('vx').textContent = d.x.toFixed(3) + ' ' + d.unit;
    document.getElementById('vy').textContent = d.y.toFixed(3) + ' ' + d.unit;
    document.getElementById('vz').textContent = d.z.toFixed(3) + ' ' + d.unit;
    document.getElementById('prog').textContent = d.progress.toFixed(1) + '%';
    document.getElementById('elap').textContent = d.elapsed.toFixed(1) + 's';
    if (d.file) document.getElementById('cur-prog').textContent = d.file;
    if (d.segments !== undefined) document.getElementById('cur-segs').textContent = d.segments;
  } catch (e) {}
}
async function triggerEStop() {
  try {
    const r = await fetch('/api/estop', {method: 'POST', headers: API_HDR});
    if (r.status === 401) showLogin(true);
  } catch (e) {}
  poll();
}
setInterval(poll, 250);
poll();
</script>
</body>
</html>)HTML";
}

} // namespace rnr::core
