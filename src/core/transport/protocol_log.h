#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

// Ring buffer of the data exchanged with a board (sent / received lines, notes), for the console panel and the
// diagnostics report (MULTI_BOARD_PLAN.md section 4.18). Thread-safe.
namespace rnr::transport {

struct ProtocolLogEntry {
    enum class Dir : uint8_t { Sent, Received, Info, Error } dir = Dir::Info;
    uint64_t    seq = 0;        // Increasing number, for incremental reads
    double      time_s = 0.0;   // Seconds since the log was created
    bool        is_status = false; // Status poll / report: hidden by the console's default filter
    std::string text;
};

class ProtocolLog {
public:
    explicit ProtocolLog(size_t capacity = 3000) : capacity_(capacity), start_(std::chrono::steady_clock::now()) {}

    void add(ProtocolLogEntry::Dir dir, std::string text, bool is_status = false) {
        std::lock_guard<std::mutex> lock(mutex_);
        ProtocolLogEntry e;
        e.dir = dir;
        e.seq = ++seq_;
        e.time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
        e.is_status = is_status;
        e.text = std::move(text);
        entries_.push_back(std::move(e));
        while (entries_.size() > capacity_) entries_.pop_front();
    }

    // Entries with seq > after_seq (at most max_count, newest kept)
    std::vector<ProtocolLogEntry> since(uint64_t after_seq, size_t max_count = 3000) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<ProtocolLogEntry> out;
        for (const auto& e : entries_) {
            if (e.seq > after_seq) out.push_back(e);
        }
        if (out.size() > max_count) out.erase(out.begin(), out.end() - static_cast<std::ptrdiff_t>(max_count));
        return out;
    }

    uint64_t lastSeq() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return seq_;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.clear();
    }

private:
    mutable std::mutex mutex_;
    std::deque<ProtocolLogEntry> entries_;
    size_t capacity_;
    uint64_t seq_ = 0;
    std::chrono::steady_clock::time_point start_;
};

} // namespace rnr::transport
