#include "core/board/grbl/grbl_protocol.h"
#include <cstdio>

namespace rnr::board::grbl {

using Dir = transport::ProtocolLogEntry::Dir;

uint8_t realtimeByte(Realtime c, bool top_bit) noexcept {
    switch (c) {
        case Realtime::StatusReport:     return top_bit ? 0x80 : '?';
        case Realtime::CycleStart:       return top_bit ? 0x81 : '~';
        case Realtime::FeedHold:         return top_bit ? 0x82 : '!';
        case Realtime::SoftReset:        return 0x18;
        case Realtime::SafetyDoor:       return 0x84;
        case Realtime::JogCancel:        return 0x85;
        case Realtime::FeedOv100:        return 0x90;
        case Realtime::FeedOvPlus10:     return 0x91;
        case Realtime::FeedOvMinus10:    return 0x92;
        case Realtime::FeedOvPlus1:      return 0x93;
        case Realtime::FeedOvMinus1:     return 0x94;
        case Realtime::RapidOv100:       return 0x95;
        case Realtime::RapidOv50:        return 0x96;
        case Realtime::RapidOv25:        return 0x97;
        case Realtime::SpindleOv100:     return 0x99;
        case Realtime::SpindleOvPlus10:  return 0x9A;
        case Realtime::SpindleOvMinus10: return 0x9B;
        case Realtime::SpindleOvPlus1:   return 0x9C;
        case Realtime::SpindleOvMinus1:  return 0x9D;
        case Realtime::SpindleStop:      return 0x9E;
        case Realtime::FloodToggle:      return 0xA0;
        case Realtime::MistToggle:       return 0xA1;
    }
    return '?';
}

GrblProtocol::GrblProtocol(std::shared_ptr<transport::ProtocolLog> log) : log_(std::move(log)) {
    if (!log_) log_ = std::make_shared<transport::ProtocolLog>();
}

GrblProtocol::~GrblProtocol() {
    close();
}

bool GrblProtocol::open(std::unique_ptr<transport::IByteStream> stream, const transport::ConnectionTarget& target,
                        const transport::StreamOptions& options, std::string* error) {
    close();
    if (!stream) {
        if (error) *error = "Unknown connection type '" + target.transport + "'";
        return false;
    }
    stream_ = std::move(stream);
    target_ = target;
    options_ = options;
    link_failed_ = false;
    connecting_.store(true, std::memory_order_release);
    pending_.clear();
    pending_bytes_ = 0;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.clear();
        open_error_.clear();
        last_receive_ = Clock::now();
    }
    reader_run_ = true;
    reader_ = std::thread(&GrblProtocol::readerLoop, this);
    return true;
}

void GrblProtocol::close() {
    reader_run_ = false;
    if (reader_.joinable()) reader_.join();
    if (stream_) {
        std::lock_guard<std::mutex> lock(stream_mutex_);
        stream_->close();
        stream_.reset();
        log_->add(Dir::Info, "Connection closed");
    }
    pending_.clear();
    pending_bytes_ = 0;
}

void GrblProtocol::setControlLines(bool dtr, bool rts) {
    if (stream_ && !connecting()) stream_->setControlLines(dtr, rts);
}

std::string GrblProtocol::openError() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return open_error_;
}

void GrblProtocol::readerLoop() {
    {
        std::string err;
        if (!stream_->open(target_, options_, &err)) {
            if (err.empty()) err = "Cannot open " + target_.address;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                open_error_ = err;
            }
            log_->add(Dir::Error, err);
            link_failed_ = true;
            connecting_.store(false, std::memory_order_release);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            last_receive_ = Clock::now();
        }
        log_->add(Dir::Info, "Opened " + (target_.label.empty() ? target_.address : target_.label));
        connecting_.store(false, std::memory_order_release);
    }
    std::string partial;
    uint8_t buf[512];
    while (reader_run_.load()) {
        const long n = stream_->read(buf, sizeof(buf), 50);
        if (n < 0) {
            if (!link_failed_.exchange(true)) log_->add(Dir::Error, "Connection lost (read failed)");
            break;
        }
        if (n == 0) continue;
        std::vector<std::string> lines;
        for (long i = 0; i < n; ++i) {
            const char c = static_cast<char>(buf[i]);
            if (c == '\n') {
                if (!partial.empty() && partial.back() == '\r') partial.pop_back();
                lines.push_back(std::move(partial));
                partial.clear();
            } else if (partial.size() < 4096) {
                partial.push_back(c);
            }
        }
        const auto now = Clock::now();
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            last_receive_ = now;
            for (auto& l : lines) queue_.push_back(l);
        }
        for (auto& l : lines) {
            if (l.empty()) continue;
            log_->add(Dir::Received, l, l[0] == '<');
        }
    }
}

void GrblProtocol::poll(std::vector<FirmwareMessage>& out) {
    std::deque<std::string> lines;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        lines.swap(queue_);
    }
    for (const auto& l : lines) {
        FirmwareMessage m = parseLine(l);
        if (m.kind == MessageKind::Empty) continue;
        if (m.kind == MessageKind::Ack || m.kind == MessageKind::Nack) {
            if (!pending_.empty()) {
                m.ticket = pending_.front().ticket;
                pending_bytes_ -= pending_.front().bytes;
                pending_.pop_front();
            }
        } else if (m.kind == MessageKind::Welcome) {
            // A restarted board has forgotten everything it was sent
            forgetUnacknowledged();
        }
        out.push_back(std::move(m));
    }
}

GrblProtocol::Clock::time_point GrblProtocol::lastReceive() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return last_receive_;
}

bool GrblProtocol::canSend(size_t line_chars) const {
    // Leave one byte free: GRBL's serial buffer holds rx_buffer_bytes - 1 characters
    return pending_bytes_ + line_chars + 1 <= rx_buffer_bytes_ - 1 || pending_.empty();
}

uint32_t GrblProtocol::sendLine(const std::string& line) {
    if (!stream_ || link_failed_.load() || connecting()) return 0;
    const std::string data = line + "\n";
    if (!stream_->write(reinterpret_cast<const uint8_t*>(data.data()), data.size())) {
        if (!link_failed_.exchange(true)) log_->add(Dir::Error, "Connection lost (write failed)");
        return 0;
    }
    const uint32_t ticket = next_ticket_++;
    if (next_ticket_ == 0) next_ticket_ = 1;
    pending_.push_back({ticket, data.size()});
    pending_bytes_ += data.size();
    log_->add(Dir::Sent, line);
    return ticket;
}

void GrblProtocol::forgetUnacknowledged() {
    pending_.clear();
    pending_bytes_ = 0;
}

bool GrblProtocol::realtime(Realtime c, bool top_bit) {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    if (!stream_ || link_failed_.load() || connecting()) return false;
    const uint8_t b = realtimeByte(c, top_bit);
    if (!stream_->write(&b, 1)) {
        link_failed_ = true;
        return false;
    }
    if (c != Realtime::StatusReport) {
        char hex[16];
        std::snprintf(hex, sizeof(hex), "<0x%02X>", b);
        log_->add(Dir::Sent, (b >= 0x20 && b < 0x7F) ? std::string(1, static_cast<char>(b)) : std::string(hex));
    }
    return true;
}

} // namespace rnr::board::grbl
