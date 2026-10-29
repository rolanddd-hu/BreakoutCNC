#include "core/camera/camera_stream_server.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <filesystem>

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
        else out += c;
    }
    return out;
}

std::string extractFileContent(const std::string& headers, const std::string& body, std::string& io_filename) {
    auto ct_pos = headers.find("multipart/form-data");
    if (ct_pos == std::string::npos) {
        ct_pos = headers.find("Multipart/Form-Data");
    }
    if (ct_pos == std::string::npos) {
        return body;
    }
    auto b_pos = headers.find("boundary=", ct_pos);
    if (b_pos == std::string::npos) return body;
    b_pos += 9;
    size_t b_end = headers.find_first_of(";\r\n", b_pos);
    std::string boundary = "--" + headers.substr(b_pos, (b_end == std::string::npos) ? std::string::npos : (b_end - b_pos));
    while (!boundary.empty() && (boundary.back() == ' ' || boundary.back() == '\t' || boundary.back() == '\r')) {
        boundary.pop_back();
    }
    size_t p0 = body.find(boundary);
    if (p0 == std::string::npos) return body;
    size_t p_hdr_end = body.find("\r\n\r\n", p0);
    if (p_hdr_end == std::string::npos) return body;
    std::string part_hdr = body.substr(p0, p_hdr_end - p0);
    auto fn_pos = part_hdr.find("filename=\"");
    if (fn_pos != std::string::npos) {
        fn_pos += 10;
        size_t fn_end = part_hdr.find("\"", fn_pos);
        if (fn_end != std::string::npos) {
            io_filename = part_hdr.substr(fn_pos, fn_end - fn_pos);
        }
    }
    size_t data_start = p_hdr_end + 4;
    size_t p_next = body.find(boundary, data_start);
    if (p_next == std::string::npos) {
        return body.substr(data_start);
    }
    size_t data_len = (p_next >= data_start + 2 && body[p_next - 2] == '\r' && body[p_next - 1] == '\n')
                          ? (p_next - 2 - data_start)
                          : (p_next - data_start);
    return body.substr(data_start, data_len);
}

} // namespace

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

CameraStreamServer::CameraStreamServer(CameraManager& camera_mgr)
    : camera_mgr_(camera_mgr)
{
#ifdef _WIN32
    WSADATA wsa{};
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
}

CameraStreamServer::~CameraStreamServer() {
    stop();
}

void CameraStreamServer::applyConfig(const CameraConfig& cfg) {
    target_fps_.store(std::clamp(cfg.stream_fps, 1, 60));
    allow_upload_.store(cfg.stream_allow_upload);
    if (!cfg.stream_enabled) {
        stop();
        return;
    }
    if (running_.load() && port_.load() == cfg.stream_port) {
        return;
    }
    stop();
    port_.store(cfg.stream_port);
    running_.store(true);
    accept_thread_ = std::thread(&CameraStreamServer::acceptLoop, this, cfg.stream_port);
}

void CameraStreamServer::updateTelemetry(const TelemetrySnapshot& telemetry, LinearUnits units, const std::string& app_title, const std::string& loaded_filename) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    latest_telemetry_ = telemetry;
    latest_units_     = units;
    app_title_        = app_title;
    loaded_filename_  = loaded_filename;
}

std::string CameraStreamServer::getServerUrl() const {
    char host_buf[256] = {};
    std::string ip_str = "127.0.0.1";
    if (::gethostname(host_buf, sizeof(host_buf)) == 0) {
        struct addrinfo hints{}, *res = nullptr;
        hints.ai_family   = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        if (::getaddrinfo(host_buf, nullptr, &hints, &res) == 0 && res) {
            for (struct addrinfo* p = res; p != nullptr; p = p->ai_next) {
                auto* ipv4 = reinterpret_cast<struct sockaddr_in*>(p->ai_addr);
                char ip_buf[INET_ADDRSTRLEN] = {};
                if (::inet_ntop(AF_INET, &(ipv4->sin_addr), ip_buf, sizeof(ip_buf))) {
                    std::string cand(ip_buf);
                    if (cand != "127.0.0.1" && cand.rfind("169.254.", 0) != 0) {
                        ip_str = cand;
                        break;
                    }
                }
            }
            ::freeaddrinfo(res);
        }
    }
    return "http://" + ip_str + ":" + std::to_string(port_.load());
}

void CameraStreamServer::stop() {
    running_.store(false);
    if (listen_sock_ != ~static_cast<uintptr_t>(0)) {
        closeSocketCrossPlatform(static_cast<SocketHandle>(listen_sock_));
        listen_sock_ = ~static_cast<uintptr_t>(0);
    }
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    // Unblock and wait for client threads: they reference this object (and the app's callbacks)
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (uintptr_t s : client_socks_) {
#ifdef _WIN32
            ::shutdown(static_cast<SocketHandle>(s), SD_BOTH);
#else
            ::shutdown(static_cast<SocketHandle>(s), SHUT_RDWR);
#endif
        }
    }
    for (int i = 0; i < 500 && active_clients_.load() > 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void CameraStreamServer::acceptLoop(uint16_t port) {
    SocketHandle s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == kInvalidSock) {
        running_.store(false);
        return;
    }

    int opt = 1;
    ::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);

    if (::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(s, 8) != 0) {
        closeSocketCrossPlatform(s);
        running_.store(false);
        return;
    }

    listen_sock_ = static_cast<uintptr_t>(s);

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
        // Bound resource use: limited concurrent clients, and no connection may stall a thread forever
        constexpr int kMaxClients = 16;
        if (active_clients_.load() >= kMaxClients || !running_.load()) {
            closeSocketCrossPlatform(client);
            continue;
        }
#ifdef _WIN32
        DWORD tmo_ms = 10000;
        ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tmo_ms), sizeof(tmo_ms));
        ::setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&tmo_ms), sizeof(tmo_ms));
#else
        timeval tmo{10, 0};
        ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tmo, sizeof(tmo));
        ::setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &tmo, sizeof(tmo));
#endif
        active_clients_.fetch_add(1);
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            client_socks_.insert(static_cast<uintptr_t>(client));
        }
        std::thread(&CameraStreamServer::handleClient, this, static_cast<uintptr_t>(client)).detach();
    }
}

void CameraStreamServer::handleClient(uintptr_t raw_sock) {
    SocketHandle client = static_cast<SocketHandle>(raw_sock);
    // (active_clients_ was already incremented by the accept loop)

    // Read full HTTP request headers
    std::string req;
    char chunk[4096];
    size_t header_end_pos = std::string::npos;
    size_t delimiter_len = 4;

    while (req.size() < 65536) { // max 64KB for HTTP headers
        int recvd = ::recv(client, chunk, sizeof(chunk), 0);
        if (recvd <= 0) break;
        req.append(chunk, static_cast<size_t>(recvd));
        header_end_pos = req.find("\r\n\r\n");
        if (header_end_pos != std::string::npos) {
            delimiter_len = 4;
            break;
        }
        header_end_pos = req.find("\n\n");
        if (header_end_pos != std::string::npos) {
            delimiter_len = 2;
            break;
        }
    }

    if (header_end_pos != std::string::npos) {
        size_t body_start = header_end_pos + delimiter_len;
        std::string headers = req.substr(0, header_end_pos);

        // Read remaining body based on Content-Length header if present
        size_t content_len = 0;
        auto pos_cl = headers.find("Content-Length:");
        if (pos_cl == std::string::npos) pos_cl = headers.find("content-length:");
        if (pos_cl != std::string::npos) {
            size_t val_start = headers.find_first_not_of(" \t", pos_cl + 15);
            if (val_start != std::string::npos) {
                content_len = static_cast<size_t>(std::strtoull(&headers[val_start], nullptr, 10));
            }
        }

        auto sendSimple = [&](const char* status_line, const std::string& json) {
            std::string resp = std::string(status_line) +
                "Content-Type: application/json\r\n"
                "Cache-Control: no-cache\r\n"
                "Content-Length: " + std::to_string(json.size()) + "\r\n"
                "Connection: close\r\n\r\n" + json;
            sendAllBytes(client, resp.data(), resp.size());
        };

        // Never trust Content-Length for an allocation: G-code uploads are capped
        constexpr size_t kMaxBodyBytes = 32u * 1024u * 1024u;
        std::string body;
        if (body_start < req.size()) {
            body = req.substr(body_start);
        }
        if (content_len > kMaxBodyBytes) {
            sendSimple("HTTP/1.1 413 Payload Too Large\r\n", "{\"ok\":false,\"error\":\"Upload too large\"}");
            headers.clear();
        } else if (content_len > 0) {
            body.reserve(content_len);
            while (body.size() < content_len && running_.load()) {
                size_t want = std::min(sizeof(chunk), content_len - body.size());
                int n = ::recv(client, chunk, static_cast<int>(want), 0);
                if (n <= 0) break;
                body.append(chunk, static_cast<size_t>(n));
            }
        }

        if (headers.empty()) {
            // Request already answered
        } else if (headers.rfind("GET /api/estop", 0) == 0) {
            // State-changing endpoints are POST-only (a link preview / prefetch must not act)
            sendSimple("HTTP/1.1 405 Method Not Allowed\r\n", "{\"ok\":false,\"error\":\"Use POST\"}");
        } else if (headers.rfind("POST /api/estop", 0) == 0) {
            if (on_estop_) {
                on_estop_();
            }
            std::string resp_body = "{\"ok\":true,\"state\":\"E-STOP ACTIVE\"}";
            std::string resp =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json\r\n"
                "Cache-Control: no-cache\r\n"
                "Content-Length: " + std::to_string(resp_body.size()) + "\r\n"
                "Connection: close\r\n\r\n" + resp_body;
            sendAllBytes(client, resp.data(), resp.size());
        } else if (headers.rfind("GET /api/telemetry", 0) == 0) {
            std::string resp_body = buildTelemetryJson();
            std::string resp =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json\r\n"
                "Cache-Control: no-cache\r\n"
                "Content-Length: " + std::to_string(resp_body.size()) + "\r\n"
                "Connection: close\r\n\r\n" + resp_body;
            sendAllBytes(client, resp.data(), resp.size());
        } else if (headers.rfind("POST /api/upload_gcode", 0) == 0) {
            // Remote G-Code file upload handler
            std::string filename = "remote_program.nc";
            auto q_pos = headers.find("POST /api/upload_gcode?filename=");
            if (q_pos != std::string::npos) {
                size_t fn_start = q_pos + 32;
                size_t fn_end = headers.find_first_of(" \r\n", fn_start);
                filename = headers.substr(fn_start, fn_end - fn_start);
            } else {
                auto xf_pos = headers.find("X-Filename:");
                if (xf_pos == std::string::npos) xf_pos = headers.find("x-filename:");
                if (xf_pos != std::string::npos) {
                    size_t fn_start = headers.find_first_not_of(" \t", xf_pos + 11);
                    size_t fn_end = headers.find_first_of("\r\n", fn_start);
                    filename = headers.substr(fn_start, fn_end - fn_start);
                }
            }
            filename = urlDecode(filename);
            std::string content = extractFileContent(headers, body, filename);
            filename = urlDecode(filename);
            try {
                filename = std::filesystem::path(filename).filename().string();
            } catch (...) {}
            if (filename.empty()) filename = "remote_program.nc";

            std::string resp_json;
            int status_code = 200;
            if (!allow_upload_.load()) {
                status_code = 403;
                resp_json = "{\"ok\":false,\"error\":\"Remote G-Code upload is disabled in BreakoutCNC (Camera settings)\"}";
            } else if (!on_load_gcode_) {
                status_code = 503;
                resp_json = "{\"ok\":false,\"error\":\"Remote G-Code loading is not enabled in BreakoutCNC\"}";
            } else if (content.empty()) {
                status_code = 400;
                resp_json = "{\"ok\":false,\"error\":\"Uploaded file is empty\"}";
            } else {
                LoadGCodeResult res = on_load_gcode_(filename, content);
                if (res.success) {
                    status_code = 200;
                    resp_json = "{\"ok\":true,\"filename\":\"" + escapeJsonString(filename) +
                                "\",\"lines\":" + std::to_string(res.line_count) + "}";
                } else {
                    status_code = 400;
                    resp_json = "{\"ok\":false,\"error\":\"" + escapeJsonString(res.error_message) + "\"}";
                }
            }

            std::string status_hdr = (status_code == 200) ? "HTTP/1.1 200 OK\r\n"
                                   : (status_code == 403) ? "HTTP/1.1 403 Forbidden\r\n"
                                   : "HTTP/1.1 400 Bad Request\r\n";
            std::string resp =
                status_hdr +
                "Content-Type: application/json\r\n"
                "Cache-Control: no-cache\r\n"
                "Content-Length: " + std::to_string(resp_json.size()) + "\r\n"
                "Connection: close\r\n\r\n" + resp_json;
            sendAllBytes(client, resp.data(), resp.size());
        } else if (headers.rfind("GET /stream.mjpg", 0) == 0) {
            std::string hdr =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                "Pragma: no-cache\r\n"
                "Connection: close\r\n\r\n";
            if (sendAllBytes(client, hdr.data(), hdr.size())) {
                CameraFrame frame;
                std::vector<uint8_t> jpeg;
                while (running_.load()) {
                    bool got = camera_mgr_.copyCurrentFrame(CameraRole::Monitoring, frame);
                    if (!got) {
                        got = camera_mgr_.copyCurrentFrame(CameraRole::TopDown, frame);
                    }
                    if (got && !frame.rgba.empty()) {
                        encodeRgbaToJpeg(frame.rgba.data(), frame.width, frame.height, jpeg);
                        char part_hdr[128];
                        int plen = std::snprintf(
                            part_hdr, sizeof(part_hdr),
                            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                            jpeg.size());
                        if (!sendAllBytes(client, part_hdr, static_cast<size_t>(plen)) ||
                            !sendAllBytes(client, jpeg.data(), jpeg.size()) ||
                            !sendAllBytes(client, "\r\n", 2))
                        {
                            break;
                        }
                    }
                    int fps = std::clamp(target_fps_.load(), 1, 60);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));
                }
            }
        } else {
            std::string html = buildDashboardHtml();
            std::string resp =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n"
                "Content-Length: " + std::to_string(html.size()) + "\r\n"
                "Connection: close\r\n\r\n" + html;
            sendAllBytes(client, resp.data(), resp.size());
        }
    }

    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_socks_.erase(raw_sock);
    }
    closeSocketCrossPlatform(client);
    active_clients_.fetch_sub(1);
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

    char buf[1024];
    std::snprintf(
        buf, sizeof(buf),
        "{\"title\":\"%s\",\"state\":\"%s\",\"estop\":%s,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,"
        "\"unit\":\"%s\",\"progress\":%.1f,\"elapsed\":%.1f,\"spindle\":%s,"
        "\"file\":\"%s\",\"segments\":%zu}",
        escapeJsonString(title).c_str(),
        stateToString(t.state),
        (t.state == ControllerState::EStop) ? "true" : "false",
        t.work_pos_mm[0] * scale,
        t.work_pos_mm[1] * scale,
        t.work_pos_mm[2] * scale,
        unit_lbl,
        prog_pct,
        t.job_elapsed_seconds,
        t.torch_or_spindle_on ? "true" : "false",
        escapeJsonString(base_file).c_str(),
        t.total_segments);
    return std::string(buf);
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
  .brand { font-size:18px; font-weight:bold; color:#00e5ff; letter-spacing:1px; }
  .badge { padding:5px 12px; border-radius:5px; font-weight:bold; font-size:14px; background:#1f6f43; color:#fff; }
  .badge.estop { background:#c92a2a; }
  .cam-box { background:#06080b; border:1px solid #283446; border-radius:8px; overflow:hidden; text-align:center; margin-bottom:12px; }
  .cam-box img { width:100%; max-height:480px; object-fit:contain; display:block; }
  .grid { display:grid; grid-template-columns:repeat(3, 1fr); gap:10px; margin-bottom:12px; }
  .dro { background:#131820; border:1px solid #263244; border-radius:6px; padding:10px; text-align:center; }
  .dro .ax { color:#00e5ff; font-size:13px; margin-bottom:4px; }
  .dro .val { font-size:22px; font-weight:bold; }
  .meta { display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-bottom:14px; }
  .card { background:#131820; border:1px solid #283446; border-radius:8px; padding:14px; margin-bottom:14px; }
  .card-hdr { display:flex; justify-content:space-between; align-items:center; font-size:13px; font-weight:bold; color:#ffb703; letter-spacing:1px; margin-bottom:10px; }
  .prog-badge { font-size:11px; padding:3px 8px; border-radius:4px; background:#1b2432; color:#8b9bb4; }
  .file-row { display:flex; gap:10px; align-items:center; flex-wrap:wrap; margin-bottom:10px; }
  .btn { padding:9px 16px; font-size:14px; font-weight:bold; font-family:inherit; border-radius:6px; cursor:pointer; transition:all 0.15s ease; border:1px solid transparent; }
  .btn-browse { background:#0d3b53; color:#00e5ff; border-color:#00e5ff; }
  .btn-browse:hover { background:#124e6e; }
  .btn-upload { background:#1b5e3a; color:#fff; border-color:#2ecc71; }
  .btn-upload:hover:not(:disabled) { background:#227a4b; }
  .btn-upload:disabled { opacity:0.4; cursor:not-allowed; }
  .fname-disp { flex:1; min-width:180px; background:#090c10; border:1px solid #212b3b; border-radius:5px; padding:8px 12px; font-size:13px; color:#c9d1d9; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .status-line { font-size:13px; min-height:18px; line-height:18px; margin-top:4px; }
  .prog-summary { display:flex; justify-content:space-between; font-size:12px; color:#8b9bb4; margin-top:8px; padding-top:8px; border-top:1px solid #1c2430; }
  .estop-btn { width:100%; padding:18px; font-size:20px; font-weight:bold; color:#fff; background:linear-gradient(180deg,#e03131,#a61e1e); border:2px solid #ff6b6b; border-radius:8px; cursor:pointer; letter-spacing:1px; }
  .estop-btn:active { background:#7d1414; }
</style>
</head>
<body>
<div class="wrap">
  <div class="hdr">
    <div class="brand" id="title">BREAKOUT CNC MONITOR</div>
    <div class="badge" id="state">CONNECTING...</div>
  </div>
  <div class="cam-box">
    <img src="/stream.mjpg" alt="Live CNC Camera Stream"/>
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
      <div>Active Program: <strong id="cur-prog" style="color:#00e5ff">None</strong></div>
      <div>Toolpath Segments: <strong id="cur-segs" style="color:#00e5ff">0</strong></div>
    </div>
  </div>

  <button class="estop-btn" onclick="triggerEStop()">EMERGENCY STOP (E-STOP)</button>
</div>
<script>
let selectedFile = null;

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
      headers: {
        'Content-Type': 'text/plain; charset=utf-8',
        'X-Filename': encodeURIComponent(selectedFile.name)
      },
      body: text
    });
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
    const r = await fetch('/api/telemetry');
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
  await fetch('/api/estop', {method:'POST'});
  poll();
}
setInterval(poll, 250);
poll();
</script>
</body>
</html>)HTML";
}

} // namespace rnr::core
