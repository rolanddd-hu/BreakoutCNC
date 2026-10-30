#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>

// Lock-free timing counters for the F12 performance overlay. Any thread may add samples.
namespace rnr::core::diag {

struct Counter {
    std::atomic<uint64_t> count{0};
    std::atomic<uint64_t> total_ns{0};
    std::atomic<uint64_t> max_ns{0};

    void add(std::chrono::steady_clock::duration d) noexcept {
        const auto ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(d).count());
        count.fetch_add(1, std::memory_order_relaxed);
        total_ns.fetch_add(ns, std::memory_order_relaxed);
        uint64_t m = max_ns.load(std::memory_order_relaxed);
        while (ns > m && !max_ns.compare_exchange_weak(m, ns, std::memory_order_relaxed)) {
        }
    }

    struct Snapshot {
        uint64_t count = 0;
        double   avg_ms = 0.0;
        double   max_ms = 0.0;
    };

    // Values since the previous call, then starts a new period
    Snapshot takeAndReset() noexcept {
        Snapshot s;
        s.count = count.exchange(0, std::memory_order_relaxed);
        const uint64_t total = total_ns.exchange(0, std::memory_order_relaxed);
        s.max_ms = max_ns.exchange(0, std::memory_order_relaxed) * 1e-6;
        s.avg_ms = s.count ? (total * 1e-6) / static_cast<double>(s.count) : 0.0;
        return s;
    }
};

// Time a caller waited for the controller lock in getTelemetry() / getConfig() / getProgram()
inline Counter& controllerReadWait() { static Counter c; return c; }
// Time the controller's worker thread holds the lock per 25 ms tick
inline Counter& workerTickLockHold() { static Counter c; return c; }
// Duration of one USB HID write to the motion board (write + wait for completion)
inline Counter& usbWrite() { static Counter c; return c; }

} // namespace rnr::core::diag
