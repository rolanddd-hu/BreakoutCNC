#include "core/camera/camera_manager.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdio>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")
#elif defined(__linux__)
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#endif

namespace rnr::core {

struct CameraManager::StreamSlot {
    std::atomic<bool> running{false};
    int               device_index = -1;
    std::atomic<bool> mirror_x{false};
    std::atomic<bool> mirror_y{false};
    std::thread       worker;

    mutable std::mutex frame_mutex;
    CameraFrame        latest_frame;
};

namespace {

#ifdef _WIN32
std::string wideToUtf8(const wchar_t* wstr) {
    if (!wstr || !*wstr) return {};
    int len = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return {};
    std::string out(static_cast<size_t>(len - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, out.data(), len, nullptr, nullptr);
    return out;
}
#endif

inline uint8_t clampByte(int v) {
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

void yuy2ToRgba(const uint8_t* src, int width, int height, int stride, uint8_t* dst_rgba) {
    for (int y = 0; y < height; ++y) {
        const uint8_t* row = src + y * stride;
        uint8_t* out = dst_rgba + y * width * 4;
        for (int x = 0; x < width; x += 2) {
            int y0 = row[0];
            int u  = row[1] - 128;
            int y1 = row[2];
            int v  = row[3] - 128;
            row += 4;

            auto yuv2rgb = [](int y_val, int u_val, int v_val, uint8_t* p) {
                int c = y_val - 16;
                if (c < 0) c = 0;
                int r = (298 * c + 409 * v_val + 128) >> 8;
                int g = (298 * c - 100 * u_val - 208 * v_val + 128) >> 8;
                int b = (298 * c + 516 * u_val + 128) >> 8;
                p[0] = clampByte(r);
                p[1] = clampByte(g);
                p[2] = clampByte(b);
                p[3] = 255;
            };

            yuv2rgb(y0, u, v, out);
            if (x + 1 < width) {
                yuv2rgb(y1, u, v, out + 4);
            }
            out += 8;
        }
    }
}

void nv12ToRgba(const uint8_t* src, int width, int height, int stride, uint8_t* dst_rgba) {
    const uint8_t* y_plane  = src;
    const uint8_t* uv_plane = src + stride * height;
    for (int y = 0; y < height; ++y) {
        const uint8_t* y_row  = y_plane + y * stride;
        const uint8_t* uv_row = uv_plane + (y / 2) * stride;
        uint8_t* out = dst_rgba + y * width * 4;
        for (int x = 0; x < width; ++x) {
            int y_val = y_row[x];
            int u_val = uv_row[(x & ~1)] - 128;
            int v_val = uv_row[(x & ~1) + 1] - 128;
            int c = std::max(0, y_val - 16);
            out[0] = clampByte((298 * c + 409 * v_val + 128) >> 8);
            out[1] = clampByte((298 * c - 100 * u_val - 208 * v_val + 128) >> 8);
            out[2] = clampByte((298 * c + 516 * u_val + 128) >> 8);
            out[3] = 255;
            out += 4;
        }
    }
}

void applyMirrorInPlace(std::vector<uint8_t>& rgba, int width, int height, bool mirror_x, bool mirror_y) {
    if (width <= 0 || height <= 0 || (!mirror_x && !mirror_y)) return;

    if (mirror_y) {
        const size_t row_bytes = static_cast<size_t>(width) * 4;
        std::vector<uint8_t> tmp(row_bytes);
        for (int y = 0; y < height / 2; ++y) {
            uint8_t* r1 = rgba.data() + static_cast<size_t>(y) * row_bytes;
            uint8_t* r2 = rgba.data() + static_cast<size_t>(height - 1 - y) * row_bytes;
            std::memcpy(tmp.data(), r1, row_bytes);
            std::memcpy(r1, r2, row_bytes);
            std::memcpy(r2, tmp.data(), row_bytes);
        }
    }

    if (mirror_x) {
        for (int y = 0; y < height; ++y) {
            uint8_t* row = rgba.data() + static_cast<size_t>(y * width) * 4;
            for (int x = 0; x < width / 2; ++x) {
                uint8_t* p1 = row + x * 4;
                uint8_t* p2 = row + (width - 1 - x) * 4;
                for (int c = 0; c < 4; ++c) {
                    std::swap(p1[c], p2[c]);
                }
            }
        }
    }
}

// Simple 5x7 bitmap font for simulator ruler & OSD numbers
void drawTinyChar(std::vector<uint8_t>& rgba, int w, int h, int px, int py, char c, uint8_t r, uint8_t g, uint8_t b) {
    static const uint8_t kDigits[14][7] = {
        {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, // 0
        {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // 1
        {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F}, // 2
        {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}, // 3
        {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, // 4
        {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, // 5
        {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, // 6
        {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, // 7
        {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, // 8
        {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, // 9
        {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, // - (10)
        {0x00,0x00,0x1B,0x15,0x11,0x11,0x00}, // m (11)
        {0x11,0x0A,0x04,0x04,0x04,0x0A,0x11}, // X (12)
        {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, // Y (13)
    };
    int glyph_idx = -1;
    if (c >= '0' && c <= '9') glyph_idx = c - '0';
    else if (c == '-') glyph_idx = 10;
    else if (c == 'm') glyph_idx = 11;
    else if (c == 'X') glyph_idx = 12;
    else if (c == 'Y') glyph_idx = 13;
    if (glyph_idx < 0) return;

    for (int row = 0; row < 7; ++row) {
        uint8_t bits = kDigits[glyph_idx][row];
        int yy = py + row;
        if (yy < 0 || yy >= h) continue;
        for (int col = 0; col < 5; ++col) {
            if (bits & (1 << (4 - col))) {
                int xx = px + col;
                if (xx >= 0 && xx < w) {
                    size_t idx = static_cast<size_t>(yy * w + xx) * 4;
                    rgba[idx + 0] = r;
                    rgba[idx + 1] = g;
                    rgba[idx + 2] = b;
                    rgba[idx + 3] = 255;
                }
            }
        }
    }
}

void drawTinyString(std::vector<uint8_t>& rgba, int w, int h, int px, int py, const char* str, uint8_t r, uint8_t g, uint8_t b) {
    int cur_x = px;
    while (str && *str) {
        drawTinyChar(rgba, w, h, cur_x, py, *str, r, g, b);
        cur_x += 6;
        ++str;
    }
}

} // namespace

CameraManager::CameraManager() {
    slots_[0] = std::make_unique<StreamSlot>();
    slots_[1] = std::make_unique<StreamSlot>();
}

CameraManager::~CameraManager() {
    stopAll();
}

std::vector<CameraDeviceInfo> CameraManager::enumerateDevices(bool include_simulator) {
    std::vector<CameraDeviceInfo> list;

#ifdef _WIN32
    HRESULT hr_init = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    bool co_uninit = SUCCEEDED(hr_init);
    if (SUCCEEDED(MFStartup(MF_VERSION))) {
        IMFAttributes* attr = nullptr;
        if (SUCCEEDED(MFCreateAttributes(&attr, 1))) {
            if (SUCCEEDED(attr->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID))) {
                IMFActivate** devices = nullptr;
                UINT32 count = 0;
                if (SUCCEEDED(MFEnumDeviceSources(attr, &devices, &count))) {
                    for (UINT32 i = 0; i < count; ++i) {
                        wchar_t* friendly = nullptr;
                        UINT32 friendly_len = 0;
                        wchar_t* sym_link = nullptr;
                        UINT32 sym_len = 0;

                        devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &friendly, &friendly_len);
                        devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &sym_link, &sym_len);

                        CameraDeviceInfo info;
                        info.index        = static_cast<int>(list.size());
                        info.name         = friendly ? wideToUtf8(friendly) : ("USB Camera #" + std::to_string(i + 1));
                        info.unique_id    = sym_link ? wideToUtf8(sym_link) : info.name;
                        info.is_simulator = false;
                        list.push_back(std::move(info));

                        if (friendly) CoTaskMemFree(friendly);
                        if (sym_link) CoTaskMemFree(sym_link);
                        devices[i]->Release();
                    }
                    CoTaskMemFree(devices);
                }
            }
            attr->Release();
        }
        MFShutdown();
    }
    if (co_uninit) CoUninitialize();
#elif defined(__linux__)
    for (int i = 0; i < 8; ++i) {
        std::string dev_path = "/dev/video" + std::to_string(i);
        int fd = ::open(dev_path.c_str(), O_RDWR | O_NONBLOCK);
        if (fd >= 0) {
            struct v4l2_capability cap{};
            if (::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 && (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                CameraDeviceInfo info;
                info.index        = static_cast<int>(list.size());
                info.name         = reinterpret_cast<const char*>(cap.card);
                info.unique_id    = dev_path;
                info.is_simulator = false;
                list.push_back(std::move(info));
            }
            ::close(fd);
        }
    }
#endif

    if (include_simulator) {
        CameraDeviceInfo sim_top;
        sim_top.index        = static_cast<int>(list.size());
        sim_top.name         = "Simulated Top-Down CNC Alignment Camera";
        sim_top.unique_id    = "SIM_TOPDOWN";
        sim_top.is_simulator = true;
        list.push_back(std::move(sim_top));

        CameraDeviceInfo sim_mon;
        sim_mon.index        = static_cast<int>(list.size());
        sim_mon.name         = "Simulated Enclosure Monitoring Webcam";
        sim_mon.unique_id    = "SIM_MONITOR";
        sim_mon.is_simulator = true;
        list.push_back(std::move(sim_mon));
    }

    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        cached_devices_ = list;
    }
    return list;
}

void CameraManager::applyConfig(CameraConfig& cfg, bool simulator_mode, bool allow_top_down_role) {
    bool need_enum = false;
    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        if (cached_devices_.empty() || last_simulator_mode_ != simulator_mode) {
            need_enum = true;
            last_simulator_mode_ = simulator_mode;
        }
    }
    if (need_enum) {
        enumerateDevices(true);
    }

    int dev_count = 0;
    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        dev_count = static_cast<int>(cached_devices_.size());
    }

    if (!allow_top_down_role) {
        cfg.top_down_camera_index = -1;
        cfg.top_down_camera_name.clear();
    }

    if (cfg.top_down_camera_index >= dev_count) {
        cfg.top_down_camera_index = -1;
    }
    if (cfg.monitoring_camera_index >= dev_count) {
        cfg.monitoring_camera_index = -1;
    }

    // Exclusive assignment: if only 1 camera is selected and both roles point to the same index,
    // keep the active role and unassign the conflicting one.
    if (cfg.top_down_camera_index >= 0 && cfg.top_down_camera_index == cfg.monitoring_camera_index) {
        // Check which slot was already on this index; the newly changed one wins
        int prev_top = slots_[0]->device_index;
        int prev_mon = slots_[1]->device_index;
        if (prev_top == cfg.top_down_camera_index && prev_mon != cfg.monitoring_camera_index) {
            // User just selected this camera for Monitoring -> unassign TopDown
            cfg.top_down_camera_index = -1;
            cfg.top_down_camera_name.clear();
        } else {
            // User selected this camera for TopDown -> unassign Monitoring
            cfg.monitoring_camera_index = -1;
            cfg.monitoring_camera_name.clear();
        }
    }

    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        if (cfg.top_down_camera_index >= 0 && cfg.top_down_camera_index < static_cast<int>(cached_devices_.size())) {
            cfg.top_down_camera_name = cached_devices_[static_cast<size_t>(cfg.top_down_camera_index)].name;
        } else {
            cfg.top_down_camera_name.clear();
        }
        if (cfg.monitoring_camera_index >= 0 && cfg.monitoring_camera_index < static_cast<int>(cached_devices_.size())) {
            cfg.monitoring_camera_name = cached_devices_[static_cast<size_t>(cfg.monitoring_camera_index)].name;
        } else {
            cfg.monitoring_camera_name.clear();
        }
    }

    startSlot(CameraRole::TopDown, cfg.top_down_camera_index, cfg.mirror_x, cfg.mirror_y);
    startSlot(CameraRole::Monitoring, cfg.monitoring_camera_index, false, false);
}

void CameraManager::updateLiveTelemetry(const TelemetrySnapshot& telemetry, const MachineConfig& /*cfg*/) {
    std::lock_guard<std::mutex> lock(sim_state_mutex_);
    sim_machine_pos_mm_ = telemetry.machine_pos_mm;
    sim_work_pos_mm_    = telemetry.work_pos_mm;
    sim_work_offset_mm_ = telemetry.work_offset_mm;
    sim_spindle_on_     = telemetry.torch_or_spindle_on;
    sim_state_          = telemetry.state;
    sim_op_mode_        = telemetry.operation_mode;
    sim_elapsed_s_      = telemetry.job_elapsed_seconds;
    if (telemetry.total_segments > 0) {
        sim_progress_ = (static_cast<double>(telemetry.active_segment_index) + telemetry.segment_progress_0_to_1) /
                        static_cast<double>(telemetry.total_segments);
    } else {
        sim_progress_ = 0.0;
    }

    // Automatically record calibration dent if spindle is running and Z touches <= 0.05 mm
    if (telemetry.torch_or_spindle_on && telemetry.work_pos_mm[2] <= 0.05) {
        sim_has_dent_    = true;
        sim_dent_work_x_ = telemetry.work_pos_mm[0];
        sim_dent_work_y_ = telemetry.work_pos_mm[1];
    }
}

void CameraManager::makeCalibrationDentAtCurrentToolPos(const TelemetrySnapshot& telemetry) {
    std::lock_guard<std::mutex> lock(sim_state_mutex_);
    sim_has_dent_    = true;
    sim_dent_work_x_ = telemetry.work_pos_mm[0];
    sim_dent_work_y_ = telemetry.work_pos_mm[1];
}

bool CameraManager::isRoleActive(CameraRole role) const {
    const auto& slot = slots_[static_cast<size_t>(role)];
    return slot && slot->running.load() && slot->device_index >= 0;
}

bool CameraManager::getLatestFrame(CameraRole role, CameraFrame& out_frame, uint64_t& inout_last_seq) const {
    const auto& slot = slots_[static_cast<size_t>(role)];
    if (!slot || !slot->running.load()) return false;
    std::lock_guard<std::mutex> lock(slot->frame_mutex);
    if (slot->latest_frame.frame_seq == 0 || slot->latest_frame.frame_seq == inout_last_seq ||
        slot->latest_frame.rgba.empty())
    {
        return false;
    }
    out_frame      = slot->latest_frame;
    inout_last_seq = slot->latest_frame.frame_seq;
    return true;
}

bool CameraManager::copyCurrentFrame(CameraRole role, CameraFrame& out_frame) const {
    const auto& slot = slots_[static_cast<size_t>(role)];
    if (!slot || !slot->running.load()) return false;
    std::lock_guard<std::mutex> lock(slot->frame_mutex);
    if (slot->latest_frame.frame_seq == 0 || slot->latest_frame.rgba.empty()) {
        return false;
    }
    out_frame = slot->latest_frame;
    return true;
}

void CameraManager::stopAll() {
    stopSlot(CameraRole::TopDown);
    stopSlot(CameraRole::Monitoring);
}

void CameraManager::startSlot(CameraRole role, int device_index, bool mirror_x, bool mirror_y) {
    auto& slot = slots_[static_cast<size_t>(role)];
    slot->mirror_x.store(mirror_x);
    slot->mirror_y.store(mirror_y);

    if (device_index < 0) {
        stopSlot(role);
        return;
    }

    if (slot->running.load() && slot->device_index == device_index) {
        return; // Already running on the requested camera
    }

    stopSlot(role);
    slot->device_index = device_index;
    slot->running.store(true);
    slot->worker = std::thread(&CameraManager::captureThreadLoop, this, role, device_index);
}

void CameraManager::stopSlot(CameraRole role) {
    auto& slot = slots_[static_cast<size_t>(role)];
    slot->running.store(false);
    if (slot->worker.joinable()) {
        slot->worker.join();
    }
    slot->device_index = -1;
    std::lock_guard<std::mutex> lock(slot->frame_mutex);
    slot->latest_frame = CameraFrame{};
}

void CameraManager::captureThreadLoop(CameraRole role, int device_index) {
    auto& slot = slots_[static_cast<size_t>(role)];

    CameraDeviceInfo dev_info{};
    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        if (device_index >= 0 && device_index < static_cast<int>(cached_devices_.size())) {
            dev_info = cached_devices_[static_cast<size_t>(device_index)];
        } else {
            dev_info.is_simulator = true;
        }
    }

    if (dev_info.is_simulator) {
        CameraFrame local_frame;
        local_frame.width  = 640;
        local_frame.height = 360;
        local_frame.rgba.resize(static_cast<size_t>(local_frame.width * local_frame.height * 4), 255);

        while (slot->running.load()) {
            bool mx = slot->mirror_x.load();
            bool my = slot->mirror_y.load();
            if (role == CameraRole::TopDown && dev_info.unique_id != "SIM_MONITOR") {
                renderSimulatedTopDownFrame(local_frame, mx, my);
            } else {
                renderSimulatedMonitorFrame(local_frame, mx, my);
            }
            local_frame.frame_seq++;
            {
                std::lock_guard<std::mutex> lock(slot->frame_mutex);
                slot->latest_frame = local_frame;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        }
        return;
    }

#ifdef _WIN32
    HRESULT hr_co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    bool co_uninit = SUCCEEDED(hr_co);
    bool mf_started = SUCCEEDED(MFStartup(MF_VERSION));

    IMFSourceReader* reader = nullptr;
    GUID subtype = MFVideoFormat_RGB32;
    UINT32 vid_w = 640;
    UINT32 vid_h = 480;
    LONG stride  = 640 * 4;

    if (mf_started) {
        IMFAttributes* enum_attr = nullptr;
        if (SUCCEEDED(MFCreateAttributes(&enum_attr, 1))) {
            enum_attr->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
            IMFActivate** devices = nullptr;
            UINT32 count = 0;
            if (SUCCEEDED(MFEnumDeviceSources(enum_attr, &devices, &count))) {
                if (static_cast<UINT32>(device_index) < count) {
                    IMFMediaSource* source = nullptr;
                    if (SUCCEEDED(devices[device_index]->ActivateObject(IID_PPV_ARGS(&source)))) {
                        IMFAttributes* reader_attr = nullptr;
                        MFCreateAttributes(&reader_attr, 2);
                        if (reader_attr) {
                            reader_attr->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE);
                        }
                        MFCreateSourceReaderFromMediaSource(source, reader_attr, &reader);
                        if (reader_attr) reader_attr->Release();
                        source->Release();
                    }
                }
                for (UINT32 i = 0; i < count; ++i) devices[i]->Release();
                CoTaskMemFree(devices);
            }
            enum_attr->Release();
        }
    }

    if (reader) {
        // Attempt to configure RGB32 output via Media Foundation video processor
        IMFMediaType* type = nullptr;
        if (SUCCEEDED(MFCreateMediaType(&type))) {
            type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
            type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
            if (FAILED(reader->SetCurrentMediaType(static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, type))) {
                type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_YUY2);
                if (FAILED(reader->SetCurrentMediaType(static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, type))) {
                    type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_NV12);
                    reader->SetCurrentMediaType(static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, type);
                }
            }
            type->Release();
        }

        IMFMediaType* cur_type = nullptr;
        if (SUCCEEDED(reader->GetCurrentMediaType(static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM), &cur_type))) {
            cur_type->GetGUID(MF_MT_SUBTYPE, &subtype);
            MFGetAttributeSize(cur_type, MF_MT_FRAME_SIZE, &vid_w, &vid_h);
            UINT32 default_stride = 0;
            if (SUCCEEDED(cur_type->GetUINT32(MF_MT_DEFAULT_STRIDE, &default_stride))) {
                stride = static_cast<LONG>(default_stride);
            } else {
                stride = static_cast<LONG>(subtype == MFVideoFormat_RGB32 ? vid_w * 4 : (subtype == MFVideoFormat_YUY2 ? vid_w * 2 : vid_w));
            }
            cur_type->Release();
        }

        CameraFrame local_frame;
        local_frame.width  = static_cast<int>(vid_w);
        local_frame.height = static_cast<int>(vid_h);
        local_frame.rgba.resize(static_cast<size_t>(vid_w * vid_h * 4), 255);

        while (slot->running.load()) {
            DWORD stream_idx = 0;
            DWORD flags = 0;
            LONGLONG timestamp = 0;
            IMFSample* sample = nullptr;
            HRESULT hr = reader->ReadSample(
                static_cast<DWORD>(MF_SOURCE_READER_FIRST_VIDEO_STREAM),
                0, &stream_idx, &flags, &timestamp, &sample);

            if (SUCCEEDED(hr) && sample) {
                IMFMediaBuffer* buf = nullptr;
                if (SUCCEEDED(sample->ConvertToContiguousBuffer(&buf)) && buf) {
                    BYTE* raw = nullptr;
                    DWORD max_len = 0, cur_len = 0;
                    if (SUCCEEDED(buf->Lock(&raw, &max_len, &cur_len)) && raw) {
                        int w = static_cast<int>(vid_w);
                        int h = static_cast<int>(vid_h);
                        if (subtype == MFVideoFormat_RGB32) {
                            bool bottom_up = (stride < 0);
                            int abs_stride = std::abs( static_cast<int>(stride) );
                            if (abs_stride < w * 4) abs_stride = w * 4;
                            for (int y = 0; y < h; ++y) {
                                int src_y = bottom_up ? (h - 1 - y) : y;
                                if (static_cast<DWORD>((src_y + 1) * abs_stride) <= cur_len) {
                                    const uint8_t* src_row = raw + src_y * abs_stride;
                                    uint8_t* dst_row = local_frame.rgba.data() + y * w * 4;
                                    for (int x = 0; x < w; ++x) {
                                        dst_row[x * 4 + 0] = src_row[x * 4 + 2]; // R
                                        dst_row[x * 4 + 1] = src_row[x * 4 + 1]; // G
                                        dst_row[x * 4 + 2] = src_row[x * 4 + 0]; // B
                                        dst_row[x * 4 + 3] = 255;
                                    }
                                }
                            }
                        } else if (subtype == MFVideoFormat_YUY2) {
                            int abs_stride = std::max(w * 2, std::abs(static_cast<int>(stride)));
                            if (static_cast<DWORD>(h * abs_stride) <= cur_len) {
                                yuy2ToRgba(raw, w, h, abs_stride, local_frame.rgba.data());
                            }
                        } else if (subtype == MFVideoFormat_NV12) {
                            int abs_stride = std::max(w, std::abs(static_cast<int>(stride)));
                            if (static_cast<DWORD>(h * abs_stride * 3 / 2) <= cur_len) {
                                nv12ToRgba(raw, w, h, abs_stride, local_frame.rgba.data());
                            }
                        }

                        applyMirrorInPlace(local_frame.rgba, w, h, slot->mirror_x.load(), slot->mirror_y.load());
                        local_frame.frame_seq++;
                        {
                            std::lock_guard<std::mutex> lock(slot->frame_mutex);
                            slot->latest_frame = local_frame;
                        }
                        buf->Unlock();
                    }
                    buf->Release();
                }
                sample->Release();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        reader->Release();
    } else {
        // Fallback to simulator generator if hardware open failed
        CameraFrame local_frame;
        local_frame.width  = 640;
        local_frame.height = 360;
        local_frame.rgba.resize(static_cast<size_t>(640 * 360 * 4), 255);
        while (slot->running.load()) {
            if (role == CameraRole::TopDown) {
                renderSimulatedTopDownFrame(local_frame, slot->mirror_x.load(), slot->mirror_y.load());
            } else {
                renderSimulatedMonitorFrame(local_frame, slot->mirror_x.load(), slot->mirror_y.load());
            }
            local_frame.frame_seq++;
            {
                std::lock_guard<std::mutex> lock(slot->frame_mutex);
                slot->latest_frame = local_frame;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    }

    if (mf_started) MFShutdown();
    if (co_uninit) CoUninitialize();

#elif defined(__linux__)
    int fd = ::open(dev_info.unique_id.c_str(), O_RDWR);
    if (fd >= 0) {
        struct v4l2_format fmt{};
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 640;
        fmt.fmt.pix.height      = 480;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
        ::ioctl(fd, VIDIOC_S_FMT, &fmt);

        int w = static_cast<int>(fmt.fmt.pix.width);
        int h = static_cast<int>(fmt.fmt.pix.height);
        std::vector<uint8_t> raw_buf(static_cast<size_t>(w * h * 2));
        CameraFrame local_frame;
        local_frame.width  = w;
        local_frame.height = h;
        local_frame.rgba.resize(static_cast<size_t>(w * h * 4), 255);

        while (slot->running.load()) {
            ssize_t n = ::read(fd, raw_buf.data(), raw_buf.size());
            if (n > 0) {
                yuy2ToRgba(raw_buf.data(), w, h, w * 2, local_frame.rgba.data());
                applyMirrorInPlace(local_frame.rgba, w, h, slot->mirror_x.load(), slot->mirror_y.load());
                local_frame.frame_seq++;
                std::lock_guard<std::mutex> lock(slot->frame_mutex);
                slot->latest_frame = local_frame;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(15));
            }
        }
        ::close(fd);
    }
#endif
}

void CameraManager::renderSimulatedTopDownFrame(CameraFrame& frame, bool mirror_x, bool mirror_y) {
    const int w = frame.width;
    const int h = frame.height;

    double work_x = 0.0, work_y = 0.0;
    bool has_dent = true;
    double dent_x = 0.0, dent_y = 0.0;
    {
        std::lock_guard<std::mutex> lock(sim_state_mutex_);
        work_x   = sim_work_pos_mm_[0];
        work_y   = sim_work_pos_mm_[1];
        has_dent = sim_has_dent_;
        dent_x   = sim_dent_work_x_;
        dent_y   = sim_dent_work_y_;
    }

    // Physical simulated camera geometry:
    // Mounted at +45.0 mm X, +30.0 mm Y offset from tool, with 120.0 mm x 67.5 mm FOV
    const double kSimOffsetX = 45.0;
    const double kSimOffsetY = 30.0;
    const double kSimFovX    = 120.0;
    const double kSimFovY    = 67.5;

    const double cam_world_x = work_x - kSimOffsetX;
    const double cam_world_y = work_y - kSimOffsetY;

    for (int py = 0; py < h; ++py) {
        double wy = cam_world_y + (0.5 - static_cast<double>(py) / static_cast<double>(h)) * kSimFovY;
        uint8_t* row = frame.rgba.data() + static_cast<size_t>(py * w) * 4;

        for (int px = 0; px < w; ++px) {
            double wx = cam_world_x + (static_cast<double>(px) / static_cast<double>(w) - 0.5) * kSimFovX;

            // Background: CNC T-slot table (dark anodized slate with horizontal T-slots every 40mm)
            int r = 34, g = 38, b = 44;
            double slot_mod = std::fmod( std::abs(wy + 200.0), 40.0 );
            if (slot_mod < 6.0) {
                r = 20; g = 23; b = 27;
            }

            // Workpiece Stock: Brushed Aluminum Block from X in [0..140], Y in [0..90]
            if (wx >= 0.0 && wx <= 140.0 && wy >= 0.0 && wy <= 90.0) {
                int grain = static_cast<int>(std::sin(wy * 9.5 + wx * 0.3) * 6.0);
                r = 108 + grain;
                g = 116 + grain;
                b = 126 + grain;

                // Chamfered metallic stock edge highlight (within 1.0 mm of stock perimeter)
                if (wx < 1.0 || wx > 139.0 || wy < 1.0 || wy > 89.0) {
                    r = 165; g = 178; b = 192;
                }

                // Faint 10mm layout grid scribed on stock
                double gx = std::fmod(wx, 10.0);
                double gy = std::fmod(wy, 10.0);
                if (gx < 0.28 || gy < 0.28) {
                    r -= 14; g -= 14; b -= 14;
                }
            }

            // Virtual Steel Tape Measure / Ruler along bottom edge (Y in [-12 .. -2] mm)
            if (wy >= -12.0 && wy <= -2.0 && wx >= -20.0 && wx <= 160.0) {
                r = 215; g = 185; b = 55; // High-contrast yellow/brass machinist scale
                double rx = std::fmod(wx + 200.0, 1.0);
                double rx5 = std::fmod(wx + 200.0, 5.0);
                double rx10 = std::fmod(wx + 200.0, 10.0);
                if (rx10 < 0.35 && wy >= -9.5) {
                    r = 20; g = 20; b = 20;
                } else if (rx5 < 0.30 && wy >= -7.0) {
                    r = 35; g = 35; b = 35;
                } else if (rx < 0.25 && wy >= -4.8) {
                    r = 50; g = 50; b = 50;
                }
            }

            // Calibration Dent (Endmill spot mark at (dent_x, dent_y))
            if (has_dent) {
                double dx = wx - dent_x;
                double dy = wy - dent_y;
                double dist_mm = std::sqrt(dx * dx + dy * dy);
                if (dist_mm < 1.8) {
                    if (dist_mm < 0.45) {
                        // Deep center pit of endmill dent
                        r = 25; g = 28; b = 32;
                    } else if (dist_mm < 1.35) {
                        // Shiny conical machined bevel ring
                        double angle = std::atan2(dy, dx);
                        int spec = static_cast<int>(std::cos(angle * 2.0) * 35.0);
                        r = clampByte(195 + spec);
                        g = clampByte(205 + spec);
                        b = clampByte(220 + spec);
                    } else {
                        // Outer burr rim
                        r = 65; g = 72; b = 80;
                    }
                }
            }

            row[px * 4 + 0] = clampByte(r);
            row[px * 4 + 1] = clampByte(g);
            row[px * 4 + 2] = clampByte(b);
            row[px * 4 + 3] = 255;
        }
    }

    // Draw millimeter numbers on the virtual ruler every 10mm
    for (int mm = -20; mm <= 160; mm += 10) {
        int px = static_cast<int>(((static_cast<double>(mm) - cam_world_x) / kSimFovX + 0.5) * w);
        int py = static_cast<int>((0.5 - (-10.8 - cam_world_y) / kSimFovY) * h);
        if (px >= 10 && px < w - 24 && py >= 4 && py < h - 10) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%d", mm);
            drawTinyString(frame.rgba, w, h, px + 2, py, buf, 20, 20, 20);
        }
    }

    applyMirrorInPlace(frame.rgba, w, h, mirror_x, mirror_y);
}

void CameraManager::renderSimulatedMonitorFrame(CameraFrame& frame, bool mirror_x, bool mirror_y) {
    const int w = frame.width;
    const int h = frame.height;

    double work_x = 0.0, work_y = 0.0, work_z = 0.0;
    bool spindle_on = false;
    ControllerState state = ControllerState::Idle;
    {
        std::lock_guard<std::mutex> lock(sim_state_mutex_);
        work_x     = sim_work_pos_mm_[0];
        work_y     = sim_work_pos_mm_[1];
        work_z     = sim_work_pos_mm_[2];
        spindle_on = sim_spindle_on_;
        state      = sim_state_;
    }

    // Wide-angle enclosure view
    for (int py = 0; py < h; ++py) {
        uint8_t* row = frame.rgba.data() + static_cast<size_t>(py * w) * 4;
        for (int px = 0; px < w; ++px) {
            int r = 24, g = 28, b = 35;
            // Machine bed in lower half
            if (py > h * 0.48) {
                r = 42; g = 47; b = 55;
                if ((px + py / 2) % 32 < 3) {
                    r = 30; g = 34; b = 40;
                }
            }
            row[px * 4 + 0] = static_cast<uint8_t>(r);
            row[px * 4 + 1] = static_cast<uint8_t>(g);
            row[px * 4 + 2] = static_cast<uint8_t>(b);
            row[px * 4 + 3] = 255;
        }
    }

    // Draw gantry & spindle head at simulated position
    int head_x = std::clamp(static_cast<int>(w * 0.25 + (work_x / 160.0) * (w * 0.5)), 60, w - 60);
    int head_y = std::clamp(static_cast<int>(h * 0.42 - (work_z / 50.0) * 45.0 - (work_y / 120.0) * 25.0), 40, h - 80);

    // Gantry beam
    for (int py = head_y - 18; py <= head_y - 4; ++py) {
        if (py < 0 || py >= h) continue;
        for (int px = 30; px < w - 30; ++px) {
            size_t idx = static_cast<size_t>(py * w + px) * 4;
            frame.rgba[idx + 0] = 68;
            frame.rgba[idx + 1] = 76;
            frame.rgba[idx + 2] = 88;
        }
    }

    // Spindle / Torch carriage
    for (int py = head_y - 30; py <= head_y + 38; ++py) {
        if (py < 0 || py >= h) continue;
        for (int px = head_x - 22; px <= head_x + 22; ++px) {
            if (px < 0 || px >= w) continue;
            size_t idx = static_cast<size_t>(py * w + px) * 4;
            frame.rgba[idx + 0] = 95;
            frame.rgba[idx + 1] = 105;
            frame.rgba[idx + 2] = 118;
        }
    }

    // Active cutting glow / indicator LED on head
    uint8_t led_r = (state == ControllerState::EStop) ? 255 : (spindle_on ? 0 : 40);
    uint8_t led_g = (state == ControllerState::EStop) ? 50  : (spindle_on ? 240 : 180);
    uint8_t led_b = (state == ControllerState::EStop) ? 50  : (spindle_on ? 255 : 90);
    for (int dy = -6; dy <= 6; ++dy) {
        for (int dx = -6; dx <= 6; ++dx) {
            if (dx * dx + dy * dy <= 36) {
                int px = head_x + dx;
                int py = head_y + 42 + dy;
                if (px >= 0 && px < w && py >= 0 && py < h) {
                    size_t idx = static_cast<size_t>(py * w + px) * 4;
                    frame.rgba[idx + 0] = led_r;
                    frame.rgba[idx + 1] = led_g;
                    frame.rgba[idx + 2] = led_b;
                }
            }
        }
    }

    applyMirrorInPlace(frame.rgba, w, h, mirror_x, mirror_y);
}

} // namespace rnr::core
