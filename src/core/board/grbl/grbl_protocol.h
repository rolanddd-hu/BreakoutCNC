#pragma once
#include "core/board/grbl/grbl_messages.h"
#include "core/transport/byte_stream.h"
#include "core/transport/protocol_log.h"
#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// GRBL-family wire protocol (GRBL_FAMILY_PLAN.md section 5): a reader thread that splits the stream into lines,
// character-counting flow control for streamed lines, and real-time command bytes.
//
// Threading: open / close / poll / sendLine / forgetUnacknowledged run on the core thread (core lock held).
// realtime() may be called from any thread. The reader thread only touches its own line queue.
namespace rnr::board::grbl {

enum class Realtime : uint8_t {
    StatusReport, CycleStart, FeedHold, SoftReset, SafetyDoor, JogCancel,
    FeedOv100, FeedOvPlus10, FeedOvMinus10, FeedOvPlus1, FeedOvMinus1,
    RapidOv100, RapidOv50, RapidOv25,
    SpindleOv100, SpindleOvPlus10, SpindleOvMinus10, SpindleOvPlus1, SpindleOvMinus1,
    SpindleStop, FloodToggle, MistToggle,
};

uint8_t realtimeByte(Realtime c, bool top_bit) noexcept;

class GrblProtocol {
public:
    using Clock = std::chrono::steady_clock;

    explicit GrblProtocol(std::shared_ptr<transport::ProtocolLog> log);
    ~GrblProtocol();
    GrblProtocol(const GrblProtocol&) = delete;
    GrblProtocol& operator=(const GrblProtocol&) = delete;

    bool open(std::unique_ptr<transport::IByteStream> stream, const transport::ConnectionTarget& target,
              const transport::StreamOptions& options, std::string* error);
    void close();
    bool isOpen() const { return stream_ != nullptr; }
    bool linkFailed() const { return link_failed_.load(); }
    bool accessesRealHardware() const { return stream_ && stream_->accessesRealHardware(); }
    void setControlLines(bool dtr, bool rts);

    // Received messages since the last call, oldest first. Ack / Nack carry the ticket of the line they answer.
    void poll(std::vector<FirmwareMessage>& out);
    Clock::time_point lastReceive() const;

    // Flow control: the firmware's receive buffer (character counting)
    void setRxBufferBytes(size_t bytes) { rx_buffer_bytes_ = bytes; }
    bool canSend(size_t line_chars) const;      // Without the line end
    size_t unacknowledged() const { return pending_.size(); }
    size_t unacknowledgedBytes() const { return pending_bytes_; }
    // Sends one line (a '\n' is added). Returns its ticket, 0 when the write failed.
    uint32_t sendLine(const std::string& line);
    // After a reset the board forgets every line it had not answered yet
    void forgetUnacknowledged();

    // Real-time command byte, written at once (between lines). Any thread.
    bool realtime(Realtime c, bool top_bit = false);

private:
    void readerLoop();

    std::shared_ptr<transport::ProtocolLog> log_;
    std::unique_ptr<transport::IByteStream> stream_;
    std::mutex stream_mutex_; // realtime() from other threads vs close()
    std::thread reader_;
    std::atomic<bool> reader_run_{false};
    std::atomic<bool> link_failed_{false};

    mutable std::mutex queue_mutex_;
    std::deque<std::string> queue_;
    Clock::time_point last_receive_{};

    struct Pending {
        uint32_t ticket = 0;
        size_t   bytes = 0;
    };
    std::deque<Pending> pending_;
    size_t   pending_bytes_ = 0;
    size_t   rx_buffer_bytes_ = 128;
    uint32_t next_ticket_ = 1;
};

} // namespace rnr::board::grbl
