#pragma once
#include "core/hid/rnr_protocol.h"
#include <string>
#include <vector>
#include <atomic>
#include <mutex>

namespace rnr::hid {

struct UsbDeviceInfo {
    std::string device_path;
    std::string product_name;
    std::string serial_number;
    uint16_t vendor_id  = 0;
    uint16_t product_id = 0;
    uint16_t version    = 0;
    uint16_t input_report_len  = 0;
    uint16_t output_report_len = 0;
};

// Thread-safety: all public methods may be called from any thread. A single I/O mutex serializes
// reads, writes, open and close so the device handle can never be closed underneath a pending
// overlapped operation.
class RnrUsbDevice {
public:
    RnrUsbDevice();
    ~RnrUsbDevice();

    RnrUsbDevice(const RnrUsbDevice&) = delete;
    RnrUsbDevice& operator=(const RnrUsbDevice&) = delete;

    // Enumerate all connected RNR Eco Motion 2.0 boards (VID=0xA720, PID=0xF803)
    static std::vector<UsbDeviceInfo> enumerateDevices();

    // Open the first available RNR board (or a specific device path)
    bool open(const std::string& device_path = "");
    void close();
    bool isOpen() const noexcept;

    UsbDeviceInfo deviceInfo() const;

    // Timeout-based 49-byte status report read (returns true if a report was read)
    bool readStatusReport(protocol::BoardStatusReport& out_report, uint32_t timeout_ms = 10);

    // Raw 27-byte output report write (automatically assigns & increments 16-bit sequence number at buf[3..4])
    bool sendPacket(protocol::OutputReportBuffer& packet, uint16_t* out_assigned_seq = nullptr, uint32_t timeout_ms = 50);

    // High-level convenience wrappers around protocol packet builders
    bool sendShortCommand(
        protocol::Opcode opcode,
        bool limit_override = false,
        uint8_t arg5 = 0,
        uint8_t arg6 = 0,
        uint16_t arg7_8 = 0,
        uint32_t arg9_12 = 0,
        uint16_t* out_seq = nullptr);

    bool sendMotionSlice(
        protocol::Opcode opcode,
        const protocol::MotionSlice& slice,
        bool limit_override = false,
        uint16_t* out_seq = nullptr);

    bool sendSetAxisPositions(
        const std::array<int32_t, 4>& steps,
        bool limit_override = false,
        uint16_t* out_seq = nullptr);

    bool sendConfigureBoard(
        uint8_t max_pulse_freq_khz = 100,
        uint8_t step_dir_invert_mask = 0x00,
        uint8_t input_invert_low = 0x00,
        uint8_t input_invert_high = 0x00,
        uint16_t* out_seq = nullptr);

    bool sendFullConfigureBoard(
        const protocol::BoardPinoutConfig& cfg,
        uint16_t* out_seq = nullptr);

    uint16_t lastSentSequence() const noexcept { return tx_seq_.load(std::memory_order_relaxed); }

private:
    void closeLocked();
    bool readStatusReportLocked(protocol::BoardStatusReport& out_report, uint32_t timeout_ms);

    mutable std::mutex io_mutex_;               // Guards handle_, info_, read state and all I/O
    void* handle_ = nullptr;                    // Windows HANDLE / hid_device*
    std::atomic<bool> open_{false};             // Lock-free mirror of "handle_ is valid" for isOpen()
    void* read_event_ = nullptr;                // Overlapped Read Event HANDLE
    void* write_event_ = nullptr;               // Overlapped Write Event HANDLE
    UsbDeviceInfo info_{};
    std::atomic<uint16_t> tx_seq_{0};
    bool read_pending_ = false;
    std::vector<uint8_t> rx_buf_;               // Sized to the device's input report length (>= 49)
    std::vector<uint8_t> tx_buf_;               // Sized to the device's output report length (>= 27)
    // Internal storage for persistent read OVERLAPPED structure
    alignas(8) uint8_t read_overlapped_storage_[64]{};
};

} // namespace rnr::hid
