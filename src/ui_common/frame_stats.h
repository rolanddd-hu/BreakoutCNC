#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstring>

// Per-frame timing of UI sections for the F12 performance overlay. UI thread only.
namespace rnr::ui::perf {

struct Section {
    const char* name = nullptr;
    double frame_ms  = 0.0; // accumulated in the current frame
    double avg_ms    = 0.0; // average over the last completed second
    double max_ms    = 0.0; // worst frame in the last completed second
    double sum_ms    = 0.0; // running sums for the current second
    double peak_ms   = 0.0;
};

struct Registry {
    static constexpr size_t kMaxSections = 24;
    std::array<Section, kMaxSections> sections{};
    size_t count = 0;
    bool   enabled = false;

    Section* find(const char* name) {
        for (size_t i = 0; i < count; ++i) {
            if (sections[i].name == name || std::strcmp(sections[i].name, name) == 0) return &sections[i];
        }
        if (count == kMaxSections) return nullptr;
        sections[count].name = name;
        return &sections[count++];
    }
};

inline Registry& registry() {
    static Registry r;
    return r;
}

inline void add(const char* name, double ms) {
    auto& r = registry();
    if (!r.enabled) return;
    if (Section* s = r.find(name)) s->frame_ms += ms;
}

// Times the enclosing scope into the named section (name must be a string literal)
class ScopedTimer {
public:
    explicit ScopedTimer(const char* name) : name_(name), start_(std::chrono::steady_clock::now()) {}
    ~ScopedTimer() {
        add(name_, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_).count());
    }
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    const char* name_;
    std::chrono::steady_clock::time_point start_;
};

// Called once per frame by the window loop: folds this frame into the per-second statistics.
// Returns true when a new second was completed (statistics refreshed).
inline bool endFrame(int& frames_in_second, double& second_start_s, double now_s, int& last_fps) {
    auto& r = registry();
    ++frames_in_second;
    for (size_t i = 0; i < r.count; ++i) {
        auto& s = r.sections[i];
        s.sum_ms += s.frame_ms;
        s.peak_ms = std::max(s.peak_ms, s.frame_ms);
        s.frame_ms = 0.0;
    }
    if (now_s - second_start_s < 1.0) return false;
    for (size_t i = 0; i < r.count; ++i) {
        auto& s = r.sections[i];
        s.avg_ms  = s.sum_ms / frames_in_second;
        s.max_ms  = s.peak_ms;
        s.sum_ms  = 0.0;
        s.peak_ms = 0.0;
    }
    last_fps = frames_in_second;
    frames_in_second = 0;
    second_start_s = now_s;
    return true;
}

} // namespace rnr::ui::perf
