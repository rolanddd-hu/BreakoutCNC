#pragma once
#include "core/hid/rnr_protocol.h"
#include <string>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

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

// Thread-safety: all public methods may be called from any thread. The I/O mutex serializes reads,
// open and close; writes go through a queue served by a writer thread (the board can take ~25 ms to
// accept a report, which must not stall the caller). close() waits for queued writes and holds both
// mutexes, so the handle can never be closed underneath a pending overlapped operation.
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

    // Queues a raw 27-byte output report (automatically assigns & increments 16-bit sequence number at buf[3..4])
    // and returns without waiting for the board. Reports are written in order. Returns false if the device is
    // closed, the queue is full, or an earlier queued write failed.
    bool sendPacket(protocol::OutputReportBuffer& packet, uint16_t* out_assigned_seq = nullptr);

    // E-stop path: discards the motion slices not yet written, so the report only waits for the write in
    // progress and any queued short commands. Never refused for a full queue or an earlier failed write.
    bool sendPacketUrgent(protocol::OutputReportBuffer& packet, uint16_t* out_assigned_seq = nullptr);

    // Waits until every queued report has been written (or the timeout expires)
    bool flush(uint32_t timeout_ms);

    // High-level convenience wrappers around protocol packet builders
    bool sendShortCommand(
        protocol::Opcode opcode,
        bool limit_override = false,
        uint8_t arg5 = 0,
        uint8_t arg6 = 0,
        uint16_t arg7_8 = 0,
        uint32_t arg9_12 = 0,
        uint16_t* out_seq = nullptr);

    // sendShortCommand() through sendPacketUrgent()
    bool sendUrgentShortCommand(protocol::Opcode opcode, bool limit_override = false);

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
    enum class WriteResult { Ok, Failed, Disconnected };

    void closeLocked();
    bool readStatusReportLocked(protocol::BoardStatusReport& out_report, uint32_t timeout_ms);
    void writerLoop();
    WriteResult writePacket(const protocol::OutputReportBuffer& packet);

    static constexpr size_t   kMaxQueuedWrites = 64;
    static constexpr uint32_t kWriteTimeoutMs  = 50;

    mutable std::mutex io_mutex_;               // Guards handle_, info_ and read state; held by open/close/read
    std::mutex write_mutex_;                    // Guards tx_buf_ / write_event_; handle_ changes hold both mutexes
    std::mutex queue_mutex_;                    // Guards tx_queue_, writer_busy_, writer_stop_
    std::condition_variable queue_cv_;
    std::deque<protocol::OutputReportBuffer> tx_queue_;
    bool writer_busy_ = false;
    bool writer_stop_ = false;
    std::atomic<bool> write_failed_{false};     // A queued write failed; reported by the next sendPacket()
    std::thread writer_thread_;                 // Started by the first successful open()
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
