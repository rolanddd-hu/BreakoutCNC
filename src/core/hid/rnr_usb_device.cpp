#include "core/hid/rnr_usb_device.h"
#include <algorithm>
#include <cstring>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>

#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "hid.lib")
#else
#include <hidapi.h> // hidapi::hidapi adds the hidapi/ header folder to the include path
#include <codecvt>
#include <locale>
#endif

namespace rnr::hid {

namespace {
#ifdef _WIN32
std::string wideToUtf8(const wchar_t* wstr) {
    if (!wstr || wstr[0] == L'\0') return {};
    int len = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return {};
    std::string out(static_cast<size_t>(len - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, out.data(), len, nullptr, nullptr);
    return out;
}

bool isDisconnectError(DWORD err) {
    return err == ERROR_DEVICE_NOT_CONNECTED || err == ERROR_GEN_FAILURE ||
           err == ERROR_INVALID_HANDLE || err == ERROR_BAD_COMMAND;
}
#endif
} // namespace

RnrUsbDevice::RnrUsbDevice() {
#ifdef _WIN32
    static_assert(sizeof(OVERLAPPED) <= sizeof(read_overlapped_storage_), "OVERLAPPED storage too small");
    handle_ = INVALID_HANDLE_VALUE;
    read_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    write_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    std::memset(read_overlapped_storage_, 0, sizeof(read_overlapped_storage_));
#else
    handle_ = nullptr;
#endif
    rx_buf_.assign(protocol::kInputReportBytes, 0);
    tx_buf_.assign(protocol::kOutputReportBytes, 0);
}

RnrUsbDevice::~RnrUsbDevice() {
    close();
#ifdef _WIN32
    if (read_event_) {
        CloseHandle(static_cast<HANDLE>(read_event_));
        read_event_ = nullptr;
    }
    if (write_event_) {
        CloseHandle(static_cast<HANDLE>(write_event_));
        write_event_ = nullptr;
    }
#endif
}

std::vector<UsbDeviceInfo> RnrUsbDevice::enumerateDevices() {
    std::vector<UsbDeviceInfo> result;
#ifdef _WIN32
    GUID hid_guid{};
    HidD_GetHidGuid(&hid_guid);

    HDEVINFO dev_info = SetupDiGetClassDevsA(
        &hid_guid,
        nullptr,
        nullptr,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (dev_info == INVALID_HANDLE_VALUE) {
        return result;
    }

    SP_DEVICE_INTERFACE_DATA iface_data{};
    iface_data.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    for (DWORD idx = 0; SetupDiEnumDeviceInterfaces(dev_info, nullptr, &hid_guid, idx, &iface_data); ++idx) {
        DWORD required_size = 0;
        SetupDiGetDeviceInterfaceDetailA(dev_info, &iface_data, nullptr, 0, &required_size, nullptr);
        if (required_size == 0) continue;

        std::vector<uint8_t> detail_buf(required_size, 0);
        auto* detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_A*>(detail_buf.data());
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);

        if (!SetupDiGetDeviceInterfaceDetailA(dev_info, &iface_data, detail, required_size, nullptr, nullptr)) {
            continue;
        }

        // Open with 0 access first to query attributes without locking
        HANDLE h = CreateFileA(
            detail->DevicePath,
            0,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            nullptr,
            OPEN_EXISTING,
            FILE_FLAG_OVERLAPPED,
            nullptr);

        if (h == INVALID_HANDLE_VALUE) {
            continue;
        }

        HIDD_ATTRIBUTES attrib{};
        attrib.Size = sizeof(HIDD_ATTRIBUTES);
        if (HidD_GetAttributes(h, &attrib)) {
            if (attrib.VendorID == protocol::kVendorId && attrib.ProductID == protocol::kProductId) {
                UsbDeviceInfo info{};
                info.device_path = detail->DevicePath;
                info.vendor_id   = attrib.VendorID;
                info.product_id  = attrib.ProductID;
                info.version     = attrib.VersionNumber;

                wchar_t wstr[256]{};
                if (HidD_GetProductString(h, wstr, sizeof(wstr))) {
                    info.product_name = wideToUtf8(wstr);
                }
                std::memset(wstr, 0, sizeof(wstr));
                if (HidD_GetSerialNumberString(h, wstr, sizeof(wstr))) {
                    info.serial_number = wideToUtf8(wstr);
                }

                PHIDP_PREPARSED_DATA preparsed = nullptr;
                if (HidD_GetPreparsedData(h, &preparsed) && preparsed) {
                    HIDP_CAPS caps{};
                    if (HidP_GetCaps(preparsed, &caps) == HIDP_STATUS_SUCCESS) {
                        info.input_report_len  = caps.InputReportByteLength;
                        info.output_report_len = caps.OutputReportByteLength;
                    }
                    HidD_FreePreparsedData(preparsed);
                }
                result.push_back(std::move(info));
            }
        }
        CloseHandle(h);
    }

    SetupDiDestroyDeviceInfoList(dev_info);
#else
    if (hid_init() != 0) return result;
    struct hid_device_info* devs = hid_enumerate(protocol::kVendorId, protocol::kProductId);
    for (auto* cur = devs; cur; cur = cur->next) {
        UsbDeviceInfo info;
        info.device_path = cur->path ? cur->path : "";
        info.vendor_id   = cur->vendor_id;
        info.product_id  = cur->product_id;
        info.version     = cur->release_number;
        if (cur->product_string) {
            std::wstring_convert<std::codecvt_utf8<wchar_t>> cvt;
            try { info.product_name = cvt.to_bytes(cur->product_string); } catch (...) {}
        }
        if (cur->serial_number) {
            std::wstring_convert<std::codecvt_utf8<wchar_t>> cvt;
            try { info.serial_number = cvt.to_bytes(cur->serial_number); } catch (...) {}
        }
        info.input_report_len  = protocol::kInputReportBytes;
        info.output_report_len = protocol::kOutputReportBytes;
        result.push_back(std::move(info));
    }
    hid_free_enumeration(devs);
#endif
    return result;
}

bool RnrUsbDevice::open(const std::string& device_path) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    closeLocked();

    std::string target_path = device_path;
    auto devices = enumerateDevices();
    if (target_path.empty()) {
        if (devices.empty()) {
            return false;
        }
        info_ = devices.front();
        target_path = info_.device_path;
    } else {
        info_ = UsbDeviceInfo{};
        for (const auto& d : devices) {
            if (d.device_path == target_path) {
                info_ = d;
                break;
            }
        }
        info_.device_path = target_path;
    }

    // Size transfer buffers to the device's reported HID report lengths: Windows requires the read
    // buffer to be at least InputReportByteLength and writes to be exactly OutputReportByteLength.
    rx_buf_.assign(std::max<size_t>(protocol::kInputReportBytes, info_.input_report_len), 0);
    tx_buf_.assign(std::max<size_t>(protocol::kOutputReportBytes, info_.output_report_len), 0);

#ifdef _WIN32
    HANDLE h = CreateFileA(
        target_path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        nullptr);

    if (h == INVALID_HANDLE_VALUE) {
        return false;
    }

    // Set 64 input report ring buffer inside Windows HID class driver (matches hidapi default)
    HidD_SetNumInputBuffers(h, 64);

    handle_ = h;
    read_pending_ = false;
    ResetEvent(static_cast<HANDLE>(read_event_));
#else
    if (hid_init() != 0) return false;
    hid_device* dev = hid_open_path(target_path.c_str());
    if (!dev) return false;
    handle_ = dev;
    read_pending_ = false;
#endif
    open_.store(true);
    return true;
}

void RnrUsbDevice::close() {
    std::lock_guard<std::mutex> lock(io_mutex_);
    closeLocked();
}

void RnrUsbDevice::closeLocked() {
    open_.store(false);
#ifdef _WIN32
    if (handle_ != nullptr && handle_ != INVALID_HANDLE_VALUE) {
        HANDLE h = static_cast<HANDLE>(handle_);
        // CancelIoEx cancels I/O issued by ANY thread on this handle (CancelIo only cancels the caller's).
        CancelIoEx(h, nullptr);
        if (read_pending_) {
            // Wait for the cancelled read so the kernel no longer references the OVERLAPPED / buffer.
            DWORD ignored = 0;
            GetOverlappedResult(h, reinterpret_cast<OVERLAPPED*>(read_overlapped_storage_), &ignored, TRUE);
        }
        CloseHandle(h);
    }
    handle_ = INVALID_HANDLE_VALUE;
#else
    if (handle_) {
        hid_close(static_cast<hid_device*>(handle_));
    }
    handle_ = nullptr;
#endif
    read_pending_ = false;
}

bool RnrUsbDevice::isOpen() const noexcept {
    return open_.load();
}

UsbDeviceInfo RnrUsbDevice::deviceInfo() const {
    std::lock_guard<std::mutex> lock(io_mutex_);
    return info_;
}

bool RnrUsbDevice::readStatusReport(protocol::BoardStatusReport& out_report, uint32_t timeout_ms) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    return readStatusReportLocked(out_report, timeout_ms);
}

bool RnrUsbDevice::readStatusReportLocked(protocol::BoardStatusReport& out_report, uint32_t timeout_ms) {
    if (!open_.load()) return false;
#ifdef _WIN32
    HANDLE h = static_cast<HANDLE>(handle_);
    auto* ol = reinterpret_cast<OVERLAPPED*>(read_overlapped_storage_);
    DWORD bytes_read = 0;

    if (!read_pending_) {
        std::memset(ol, 0, sizeof(OVERLAPPED));
        ol->hEvent = static_cast<HANDLE>(read_event_);
        ResetEvent(ol->hEvent);
        std::fill(rx_buf_.begin(), rx_buf_.end(), 0);

        BOOL ok = ReadFile(h, rx_buf_.data(), static_cast<DWORD>(rx_buf_.size()), &bytes_read, ol);
        if (!ok) {
            DWORD err = GetLastError();
            if (err != ERROR_IO_PENDING) {
                // Device disconnected or hardware fault
                closeLocked();
                return false;
            }
            read_pending_ = true;
        } else {
            // Completed synchronously
            if (bytes_read >= protocol::kInputReportBytes) {
                out_report = protocol::parseStatusReport(rx_buf_.data(), bytes_read);
                return true;
            }
            return false;
        }
    }

    DWORD wait_res = WaitForSingleObject(ol->hEvent, timeout_ms);
    if (wait_res != WAIT_OBJECT_0) {
        return false; // Read stays pending and is collected on the next call
    }
    read_pending_ = false;
    if (GetOverlappedResult(h, ol, &bytes_read, FALSE)) {
        if (bytes_read >= protocol::kInputReportBytes) {
            out_report = protocol::parseStatusReport(rx_buf_.data(), bytes_read);
            return true;
        }
        return false;
    }
    if (isDisconnectError(GetLastError())) {
        closeLocked();
    }
    return false;
#else
    int res = hid_read_timeout(
        static_cast<hid_device*>(handle_),
        rx_buf_.data(),
        rx_buf_.size(),
        static_cast<int>(timeout_ms));
    if (res < 0) {
        closeLocked();
        return false;
    }
    if (res >= static_cast<int>(protocol::kInputReportBytes)) {
        out_report = protocol::parseStatusReport(rx_buf_.data(), static_cast<size_t>(res));
        return true;
    }
    return false;
#endif
}

bool RnrUsbDevice::sendPacket(protocol::OutputReportBuffer& packet, uint16_t* out_assigned_seq, uint32_t timeout_ms) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!open_.load()) return false;

    // Assign and increment 16-bit sequence number at buf[3..4] (acknowledged by the board in its status report)
    const uint16_t seq = static_cast<uint16_t>(tx_seq_.fetch_add(1, std::memory_order_relaxed) + 1);
    protocol::writeLE16(&packet[3], seq);
    if (out_assigned_seq) {
        *out_assigned_seq = seq;
    }

    std::fill(tx_buf_.begin(), tx_buf_.end(), 0);
    std::memcpy(tx_buf_.data(), packet.data(), packet.size());

#ifdef _WIN32
    HANDLE h = static_cast<HANDLE>(handle_);
    OVERLAPPED ol{};
    ol.hEvent = static_cast<HANDLE>(write_event_);
    ResetEvent(ol.hEvent);

    DWORD bytes_written = 0;
    BOOL ok = WriteFile(h, tx_buf_.data(), static_cast<DWORD>(tx_buf_.size()), &bytes_written, &ol);
    if (!ok) {
        const DWORD err = GetLastError();
        if (err != ERROR_IO_PENDING) {
            if (isDisconnectError(err)) {
                closeLocked();
            }
            return false;
        }
        DWORD wait_res = WaitForSingleObject(ol.hEvent, timeout_ms);
        if (wait_res != WAIT_OBJECT_0) {
            // Cancel the write and WAIT for the cancellation to complete: the OVERLAPPED lives on this
            // stack frame, so the kernel must be finished with it before we return.
            CancelIoEx(h, &ol);
            GetOverlappedResult(h, &ol, &bytes_written, TRUE);
            return false;
        }
        if (!GetOverlappedResult(h, &ol, &bytes_written, FALSE)) {
            if (isDisconnectError(GetLastError())) {
                closeLocked();
            }
            return false;
        }
    }
    return bytes_written == tx_buf_.size();
#else
    (void)timeout_ms;
    int bytes_written = hid_write(
        static_cast<hid_device*>(handle_),
        tx_buf_.data(),
        tx_buf_.size());
    if (bytes_written < 0) {
        closeLocked();
        return false;
    }
    return bytes_written == static_cast<int>(tx_buf_.size());
#endif
}

bool RnrUsbDevice::sendShortCommand(
    protocol::Opcode opcode,
    bool limit_override,
    uint8_t arg5,
    uint8_t arg6,
    uint16_t arg7_8,
    uint32_t arg9_12,
    uint16_t* out_seq)
{
    auto pkt = protocol::buildShortControlPacket(opcode, 0, limit_override, arg5, arg6, arg7_8, arg9_12);
    return sendPacket(pkt, out_seq);
}

bool RnrUsbDevice::sendMotionSlice(
    protocol::Opcode opcode,
    const protocol::MotionSlice& slice,
    bool limit_override,
    uint16_t* out_seq)
{
    auto pkt = protocol::buildMotionSlicePacket(opcode, 0, slice, limit_override);
    return sendPacket(pkt, out_seq);
}

bool RnrUsbDevice::sendSetAxisPositions(
    const std::array<int32_t, 4>& steps,
    bool limit_override,
    uint16_t* out_seq)
{
    auto pkt = protocol::buildSetAxisPositionsPacket(0, steps, limit_override);
    return sendPacket(pkt, out_seq);
}

bool RnrUsbDevice::sendConfigureBoard(
    uint8_t max_pulse_freq_khz,
    uint8_t step_dir_invert_mask,
    uint8_t input_invert_low,
    uint8_t input_invert_high,
    uint16_t* out_seq)
{
    auto pkt = protocol::buildConfigureBoardPacket(
        0, max_pulse_freq_khz, step_dir_invert_mask, input_invert_low, input_invert_high);
    return sendPacket(pkt, out_seq);
}

bool RnrUsbDevice::sendFullConfigureBoard(
    const protocol::BoardPinoutConfig& cfg,
    uint16_t* out_seq)
{
    auto pkt = protocol::buildFullConfigureBoardPacket(0, cfg);
    return sendPacket(pkt, out_seq);
}

} // namespace rnr::hid
