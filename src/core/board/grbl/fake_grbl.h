#pragma once
// FakeGrbl: an in-process GRBL-family board behind transport::IByteStream, for unit tests and the apps' --fake-board mode
// (GRBL_FAMILY_PLAN.md section 16). It simulates the receive buffer (character counting is checked), a planner
// queue, motion at constant speed on a sped-up clock, status reports, real-time commands, homing, probing, jogging,
// alarms, settings and the three flavours' identification. Not a complete GRBL: enough to exercise the app.
#include "core/transport/byte_stream.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <cctype>
#include <thread>

namespace rnr::test {

class FakeGrbl {
public:
    enum class Flavour { Grbl11, GrblHal, FluidNC };
    using Clock = std::chrono::steady_clock;

    explicit FakeGrbl(Flavour f = Flavour::Grbl11) : flavour_(f) {
        settings_ = {{0, "10"}, {1, "25"}, {2, "0"}, {3, "0"}, {4, "0"}, {5, "0"}, {6, "0"}, {10, "1"}, {11, "0.010"},
                     {12, "0.002"}, {13, "0"}, {20, "0"}, {21, "0"}, {22, "1"}, {23, "0"}, {24, "25.000"},
                     {25, "500.000"}, {26, "250"}, {27, "1.000"}, {30, "1000"}, {31, "0"}, {32, "0"},
                     {100, "250.000"}, {101, "250.000"}, {102, "250.000"}, {110, "3000.000"}, {111, "3000.000"},
                     {112, "1500.000"}, {120, "500.000"}, {121, "500.000"}, {122, "300.000"}, {130, "300.000"},
                     {131, "300.000"}, {132, "100.000"}};
    }
    // Call before connecting: adds the A axis settings when axes == 4
    void useAxes(int n) {
        std::lock_guard<std::mutex> l(m_);
        axes = n;
        if (n >= 4) {
            settings_[103] = "40.000";
            settings_[113] = "7200.000";
            settings_[123] = "300.000";
            settings_[133] = "360.000";
        }
    }

    // ---- Test controls ----
    double time_scale = 20.0;          // Simulated seconds per real second
    size_t rx_size = 128;              // Receive buffer (bytes)
    size_t planner_blocks = 15;
    double probe_contact_z = NAN;      // Machine Z where a downward probe touches (NaN = never)
    std::string reject_substring;      // A line containing this is answered with reject_code
    int    reject_code = 20;
    bool   silent = false;             // Stop answering entirely
    // grblHAL plasma plugin (GrblHal flavour only): Arc OK after M3, |THC: in the status, $350-$369
    bool   plasma_plugin = false;
    int    plasma_mode = 2;            // $350: 0 off, 1 voltage, 2 up/down, 3 arc ok
    int    arc_failures = 0;           // The next N torch starts fail ([MSG:arc failed] and a feed hold)
    double arc_voltage = 118.5;
    double thc_z_drift_mm = 0.0;       // How far height control moves Z during a cut (reported MPos only)
    std::array<double, 4> g54{0.0, 0.0, 0.0, 0.0};
    int    axes = 3;                   // 4: an A axis (grblHAL and FluidNC; GRBL 1.1 stays at 3)

    // ---- Observations ----
    bool   overflowed = false;         // The sender put more than rx_size bytes in the receive buffer
    size_t max_rx_used = 0;
    std::vector<std::string> received; // Every complete line received
    std::array<double, 4> pos{0.0, 0.0, 0.0, 0.0};
    std::string state = "Idle";
    bool   spindle_on = false, spindle_ccw = false, flood = false, mist = false;
    bool   arc_ok = false;
    int    torch_starts = 0;
    double spindle_s = 0.0;
    int    feed_ov = 100, rapid_ov = 100, spindle_ov = 100;
    int    resets = 0;

    std::array<double, 4> position() {
        std::lock_guard<std::mutex> l(m_);
        return pos;
    }
    std::string currentState() {
        std::lock_guard<std::mutex> l(m_);
        return state;
    }
    std::vector<std::string> receivedLines() {
        std::lock_guard<std::mutex> l(m_);
        return received;
    }
    bool sawLineContaining(const std::string& s) {
        std::lock_guard<std::mutex> l(m_);
        for (const auto& r : received) {
            if (r.find(s) != std::string::npos) return true;
        }
        return false;
    }
    void raiseAlarm(int code) {
        std::lock_guard<std::mutex> l(m_);
        planner_.clear();
        input_.clear();
        rx_used_ = 0;
        state = "Alarm";
        out("ALARM:" + std::to_string(code));
    }
    void restart() {
        std::lock_guard<std::mutex> l(m_);
        doReset(false);
        pos = {0.0, 0.0, 0.0};
    }
    void setSetting(int n, const std::string& v) {
        std::lock_guard<std::mutex> l(m_);
        settings_[n] = v;
    }
    std::string setting(int n) {
        std::lock_guard<std::mutex> l(m_);
        return settings_[n];
    }

    // ---- Stream side ----
    void onOpen() {
        std::lock_guard<std::mutex> l(m_);
        last_ = Clock::now();
        out(welcome());
        if (flavour_ == Flavour::Grbl11 && settings_[22] == "1") {
            state = "Alarm";
            out("[MSG:'$H'|'$X' to unlock]");
        }
    }

    void write(const uint8_t* d, size_t n) {
        std::lock_guard<std::mutex> l(m_);
        advance();
        for (size_t i = 0; i < n; ++i) {
            const uint8_t c = d[i];
            if (handleRealtime(c)) continue;
            if (c == '\n') {
                if (!partial_.empty() && partial_.back() == '\r') partial_.pop_back();
                received.push_back(partial_);
                input_.push_back(partial_);
                partial_.clear();
                continue;
            }
            partial_.push_back(static_cast<char>(c));
            ++rx_used_;
            ++rx_used_line_;
        }
        // Each complete line also occupies its line end
        rx_used_ += 0;
        recountRx();
        process();
    }

    long read(uint8_t* buf, size_t max, uint32_t timeout_ms) {
        const auto until = Clock::now() + std::chrono::milliseconds(timeout_ms);
        while (true) {
            {
                std::lock_guard<std::mutex> l(m_);
                advance();
                process();
                if (!outbuf_.empty() && !silent) {
                    const size_t n = std::min(max, outbuf_.size());
                    std::memcpy(buf, outbuf_.data(), n);
                    outbuf_.erase(0, n);
                    return static_cast<long>(n);
                }
                if (silent) outbuf_.clear();
            }
            if (Clock::now() >= until) return 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

private:
    struct Block {
        std::array<double, 4> target{};
        double feed = 0.0;   // mm/min
        bool   rapid = false;
        bool   jog = false;
        bool   probe = false;
        double dwell = 0.0;  // seconds (> 0: a dwell)
        // G2 / G3: traced in the plane (u, w) around (cu, cw); the third axis moves linearly (helix)
        bool   arc = false;
        bool   clockwise = true;
        size_t u = 0, w = 1, n = 2;
        double cu = 0.0, cv = 0.0, radius = 0.0, a0 = 0.0, sweep = 0.0;
        std::array<double, 4> from{};
        double length = 0.0, done = 0.0;
    };
    struct Pending {
        std::string line;
    };

    std::string welcome() const {
        switch (flavour_) {
            case Flavour::GrblHal: return "GrblHAL 1.1f ['$' or '$HELP' for help]";
            case Flavour::FluidNC: return "Grbl 3.7 [FluidNC v3.7.8 (fake) '$' for help]";
            default:               return "Grbl 1.1h ['$' for help]";
        }
    }

    void out(const std::string& line) { outbuf_ += line + "\r\n"; }

    void recountRx() {
        size_t used = partial_.size();
        for (const auto& l : input_) used += l.size() + 1;
        rx_used_ = used;
        max_rx_used = std::max(max_rx_used, used);
        if (used > rx_size - 1) overflowed = true;
    }

    bool handleRealtime(uint8_t c) {
        const bool hal = flavour_ == Flavour::GrblHal;
        if (c == '?' || (hal && c == 0x80)) {
            statusReport();
            return true;
        }
        if (c == '!' || (hal && c == 0x82)) {
            if (state == "Run") state = "Hold:0";
            else if (state == "Jog") { cancelJog(); }
            return true;
        }
        if (c == '~' || (hal && c == 0x81)) {
            if (state.rfind("Hold", 0) == 0) state = planner_.empty() ? "Idle" : "Run";
            if (plasmaActive() && spindle_on && !arc_ok) arc_ok = true; // Retried by the operator
            return true;
        }
        if (c == 0x18) {
            doReset(true);
            return true;
        }
        if (c == 0x85) {
            if (state == "Jog") cancelJog();
            return true;
        }
        if (c >= 0x90 && c <= 0x94) {
            if (c == 0x90) feed_ov = 100;
            if (c == 0x91) feed_ov += 10;
            if (c == 0x92) feed_ov -= 10;
            if (c == 0x93) feed_ov += 1;
            if (c == 0x94) feed_ov -= 1;
            feed_ov = std::clamp(feed_ov, 10, 200);
            return true;
        }
        if (c >= 0x95 && c <= 0x97) {
            rapid_ov = (c == 0x95) ? 100 : (c == 0x96 ? 50 : 25);
            return true;
        }
        if (c >= 0x99 && c <= 0x9D) {
            if (c == 0x99) spindle_ov = 100;
            if (c == 0x9A) spindle_ov += 10;
            if (c == 0x9B) spindle_ov -= 10;
            if (c == 0x9C) spindle_ov += 1;
            if (c == 0x9D) spindle_ov -= 1;
            spindle_ov = std::clamp(spindle_ov, 10, 200);
            return true;
        }
        if (c == 0xA0) { flood = !flood; return true; }
        if (c == 0xA1) { mist = !mist; return true; }
        if (c >= 0x80) return true; // Other real-time bytes: ignored
        return false;
    }

    void cancelJog() {
        std::deque<Block> keep;
        for (const auto& b : planner_) {
            if (!b.jog) keep.push_back(b);
        }
        planner_ = keep;
        state = "Idle";
    }

    void doReset(bool from_command) {
        const bool moving = (state == "Run" || state == "Jog" || state == "Home");
        planner_.clear();
        input_.clear();
        partial_.clear();
        rx_used_ = 0;
        spindle_on = false;
        arc_ok = false;
        thc_offset_ = 0.0;
        flood = false;
        mist = false;
        dwell_left_ = 0.0;
        waiting_completion_ = false;
        homing_left_ = 0.0;
        ++resets;
        if (moving && from_command) {
            out("ALARM:3");
            state = "Alarm";
            out(welcome());
            out("[MSG:'$H'|'$X' to unlock]");
            return;
        }
        if (state != "Alarm") state = "Idle";
        out(welcome());
    }

    void statusReport() {
        char buf[256];
        std::string st = state;
        std::array<double, 4> shown = pos;
        shown[2] += thc_offset_;
        std::string r = "<" + st + "|MPos:" + values(shown);
        std::snprintf(buf, sizeof(buf), "|FS:%.0f,%.0f", current_feed_, spindle_on ? spindle_s : 0.0);
        r += buf;
        std::string pins;
        if (probeTouching()) pins += 'P';
        if (!pins.empty()) r += "|Pn:" + pins;
        std::snprintf(buf, sizeof(buf), "|Ov:%d,%d,%d", feed_ov, rapid_ov, spindle_ov);
        r += buf;
        std::string acc;
        if (spindle_on) acc += spindle_ccw ? 'C' : 'S';
        if (flood) acc += 'F';
        if (mist) acc += 'M';
        if (!acc.empty()) r += "|A:" + acc;
        if (flavour_ == Flavour::GrblHal && homed_) r += "|H:1," + std::to_string(homed_mask_);
        if (plasmaActive()) {
            std::string f;
            if (arc_ok) f += 'A';
            if (spindle_on) f += 'T';
            if (plasma_mode != 0) f += 'E';
            if (thcRunning()) f += 'R';
            if (thc_locked_) f += 'V';
            std::snprintf(buf, sizeof(buf), "|THC:%.1f", arc_ok ? arc_voltage : 0.0);
            r += buf;
            if (!f.empty()) r += "," + f;
        }
        r += ">";
        out(r);
    }

    size_t axisCount() const { return (flavour_ != Flavour::Grbl11 && axes >= 4) ? 4 : 3; }
    std::string values(const std::array<double, 4>& v) const {
        std::string r;
        char b[32];
        for (size_t a = 0; a < axisCount(); ++a) {
            std::snprintf(b, sizeof(b), a ? ",%.3f" : "%.3f", v[a]);
            r += b;
        }
        return r;
    }

    bool plasmaActive() const { return plasma_plugin && flavour_ == Flavour::GrblHal; }
    bool thcRunning() const { return plasmaActive() && plasma_mode != 0 && arc_ok && cutting_; }

    bool probeTouching() const { return !std::isnan(probe_contact_z) && pos[2] <= probe_contact_z + 1e-9; }

    // Simulated time
    void advance() {
        const auto now = Clock::now();
        double dt = std::chrono::duration<double>(now - last_).count() * time_scale;
        last_ = now;
        if (silent) return;
        if (homing_left_ > 0.0) {
            homing_left_ -= dt;
            if (homing_left_ <= 0.0) {
                homing_left_ = 0.0;
                for (size_t a = 0; a < axisCount(); ++a) {
                    if (homing_axes_ & (1U << a)) pos[a] = 0.0;
                }
                homed_ = true;
                homed_mask_ |= homing_axes_;
                state = "Idle";
                finishCompletion();
            }
            return;
        }
        if (state.rfind("Hold", 0) == 0 || state == "Alarm") {
            current_feed_ = 0.0;
            return;
        }
        while (dt > 1e-12) {
            if (dwell_left_ > 0.0) {
                const double use = std::min(dt, dwell_left_);
                dwell_left_ -= use;
                dt -= use;
                if (dwell_left_ <= 1e-12) {
                    dwell_left_ = 0.0;
                    finishCompletion();
                }
                continue;
            }
            if (planner_.empty()) {
                current_feed_ = 0.0;
                if (state == "Run" || state == "Jog") state = "Idle";
                if (waiting_completion_ && completion_on_empty_) finishCompletion();
                break;
            }
            Block& b = planner_.front();
            state = b.jog ? "Jog" : "Run";
            const double feed = b.rapid ? 3000.0 * rapid_ov / 100.0 : (b.jog ? b.feed : b.feed * feed_ov / 100.0);
            current_feed_ = feed;
            cutting_ = !b.rapid && !b.jog && !b.probe && spindle_on;
            thc_locked_ = false;
            if (thcRunning() && thc_z_drift_mm != 0.0) {
                const double rate = std::abs(thc_z_drift_mm) * 2.0; // reaches the drift in 0.5 s
                thc_offset_ = std::clamp(thc_offset_ + (thc_z_drift_mm > 0 ? 1.0 : -1.0) * rate * dt, -std::abs(thc_z_drift_mm),
                                         std::abs(thc_z_drift_mm));
            }
            const double speed = std::max(1.0, feed) / 60.0;
            if (b.arc) {
                const double step = speed * dt;
                if (b.done + step >= b.length) {
                    dt -= (b.length - b.done) / speed;
                    pos = b.target;
                    planner_.pop_front();
                    continue;
                }
                b.done += step;
                const double f = b.done / b.length;
                const double ang = b.a0 + (b.clockwise ? -1.0 : 1.0) * b.sweep * f;
                pos[b.u] = b.cu + b.radius * std::cos(ang);
                pos[b.w] = b.cv + b.radius * std::sin(ang);
                pos[b.n] = b.from[b.n] + (b.target[b.n] - b.from[b.n]) * f;
                pos[3] = b.from[3] + (b.target[3] - b.from[3]) * f;
                dt = 0.0;
                break;
            }
            std::array<double, 4> d{};
            double len = 0.0;
            for (size_t a = 0; a < 4; ++a) {
                d[a] = b.target[a] - pos[a];
                len += d[a] * d[a];
            }
            len = std::sqrt(len);
            const double step = speed * dt;
            if (len <= step) {
                dt -= len / speed;
                pos = b.target;
                const bool was_probe = b.probe;
                planner_.pop_front();
                if (was_probe) {
                    char buf[96];
                    out("[PRB:" + values(pos) + ":0]");
                    finishCompletion();
                }
                continue;
            }
            for (size_t a = 0; a < 4; ++a) pos[a] += d[a] / len * step;
            dt = 0.0;
            if (b.probe && probeTouching()) {
                pos[2] = probe_contact_z;
                planner_.clear();
                out("[PRB:" + values(pos) + ":1]");
                state = "Idle";
                finishCompletion();
            }
        }
    }

    void finishCompletion() {
        if (!waiting_completion_) return;
        waiting_completion_ = false;
        completion_on_empty_ = false;
        out("ok");
        if (!input_.empty()) input_.pop_front();
        recountRx();
    }

    static bool word(const std::string& line, char letter, double& v) {
        for (size_t i = 0; i < line.size(); ++i) {
            if (std::toupper(static_cast<unsigned char>(line[i])) == letter) {
                // Skip letters inside "$J=" etc. are handled by callers
                char* end = nullptr;
                v = std::strtod(line.c_str() + i + 1, &end);
                if (end != line.c_str() + i + 1) return true;
            }
        }
        return false;
    }

    static std::vector<double> allG(const std::string& line) {
        std::vector<double> gs;
        for (size_t i = 0; i < line.size(); ++i) {
            if (line[i] == 'G') {
                char* end = nullptr;
                const double v = std::strtod(line.c_str() + i + 1, &end);
                if (end != line.c_str() + i + 1) gs.push_back(v);
            }
        }
        return gs;
    }
    static bool hasG(const std::vector<double>& gs, double g) {
        for (double x : gs) {
            if (std::abs(x - g) < 1e-3) return true;
        }
        return false;
    }

    void process() {
        if (silent) return;
        while (!input_.empty() && !waiting_completion_ && homing_left_ <= 0.0) {
            const std::string line = input_.front();
            if (!handleLine(line)) break; // Waits (planner full / sync)
        }
    }

    void ok() {
        out("ok");
        input_.pop_front();
        recountRx();
    }
    void error(int code) {
        out("error:" + std::to_string(code));
        input_.pop_front();
        recountRx();
    }

    // false = cannot be processed yet
    bool handleLine(const std::string& raw) {
        std::string line;
        for (char c : raw) {
            if (c != ' ') line.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
        }
        if (line.empty()) { ok(); return true; }
        if (!reject_substring.empty() && raw.find(reject_substring) != std::string::npos) { error(reject_code); return true; }
        if (line.size() > 255) { error(11); return true; }
        if (line[0] == '$') return handleDollar(raw, line);
        if (state == "Alarm") { error(9); return true; }
        if (state.rfind("Hold", 0) == 0) {
            // Lines are queued while held (planner permitting)
        }
        // Strip a line number
        if (line[0] == 'N') {
            size_t i = 1;
            while (i < line.size() && std::isdigit(static_cast<unsigned char>(line[i]))) ++i;
            line = line.substr(i);
        }
        const auto gs = allG(line);
        double v = 0.0;
        if (hasG(gs, 90)) absolute_ = true;
        if (hasG(gs, 91)) absolute_ = false;
        if (hasG(gs, 17)) plane_ = 17;
        if (hasG(gs, 18)) plane_ = 18;
        if (hasG(gs, 19)) plane_ = 19;
        if (hasG(gs, 4)) {
            if (!planner_.empty()) return false;
            double p = 0.0;
            word(line, 'P', p);
            dwell_left_ = std::max(0.0, p);
            waiting_completion_ = true;
            if (dwell_left_ <= 0.0) finishCompletion();
            return true;
        }
        // Spindle / coolant: synchronised with motion
        const bool m_word = line.find('M') != std::string::npos;
        if (m_word) {
            // GRBL: one word per modal group in a block (M3 / M4 / M5, M7 / M8 / M9), else error 21
            int spindle_words = 0, coolant_words = 0;
            for (size_t i = 0; i < line.size(); ++i) {
                if (line[i] != 'M') continue;
                const int m = std::atoi(line.c_str() + i + 1);
                if (m == 3 || m == 4 || m == 5) ++spindle_words;
                if (m == 7 || m == 8 || m == 9) ++coolant_words;
            }
            if (spindle_words > 1 || coolant_words > 1) { error(21); return true; }
            if (!planner_.empty()) return false;
            for (size_t i = 0; i < line.size(); ++i) {
                if (line[i] != 'M') continue;
                const int m = std::atoi(line.c_str() + i + 1);
                if (m == 3 || m == 4) {
                    spindle_on = true;
                    spindle_ccw = (m == 4);
                    if (plasmaActive() && plasma_mode != 0 && !arc_ok) {
                        ++torch_starts;
                        if (arc_failures > 0) {
                            --arc_failures;
                            out("[MSG:arc failed]");
                            state = "Hold:0"; // The plugin holds the program until Cycle Start
                        } else {
                            arc_ok = true;
                        }
                    }
                }
                if (m == 5) {
                    spindle_on = false;
                    arc_ok = false;
                    cutting_ = false;
                    thc_offset_ = 0.0;
                }
                if (m == 7) mist = true;
                if (m == 8) flood = true;
                if (m == 9) { flood = false; mist = false; }
            }
        }
        if (word(line, 'S', v)) spindle_s = v;
        if (word(line, 'F', v)) feed_ = v;
        for (double g : gs) {
            if (std::abs(g - 0) < 1e-3) motion_ = 0;
            if (std::abs(g - 1) < 1e-3) motion_ = 1;
            if (std::abs(g - 2) < 1e-3) motion_ = 2;
            if (std::abs(g - 3) < 1e-3) motion_ = 3;
        }
        const bool probe = hasG(gs, 38.3) || hasG(gs, 38.2);
        const bool machine = hasG(gs, 53);
        std::array<double, 4> target = planner_.empty() ? pos : planner_.back().target;
        const std::array<double, 4> start = target;
        bool axis = false;
        const char letters[4] = {'X', 'Y', 'Z', 'A'};
        for (size_t a = 0; a < axisCount(); ++a) {
            if (word(line, letters[a], v)) {
                axis = true;
                if (machine) target[a] = v;
                else if (absolute_) target[a] = v + g54[a];
                else target[a] = start[a] + v;
            }
        }
        if (!axis && !probe) { ok(); return true; }
        if (probe) {
            if (!planner_.empty()) return false;
            if (probeTouching()) {
                out("ALARM:4");
                state = "Alarm";
                input_.pop_front();
                recountRx();
                return true;
            }
            Block b;
            b.target = target;
            b.feed = feed_;
            b.probe = true;
            planner_.push_back(b);
            waiting_completion_ = true;
            return true;
        }
        Block arc;
        if (motion_ == 2 || motion_ == 3) {
            double i = 0, j = 0, k = 0;
            word(line, 'I', i);
            word(line, 'J', j);
            word(line, 'K', k);
            // GRBL's planes: G17 (X, Y), G18 (Z, X), G19 (Y, Z); G2 is clockwise looking down the third axis
            size_t u = 0, w = 1, n = 2;
            double cu = start[0] + i, cv = start[1] + j;
            if (plane_ == 18) { u = 2; w = 0; n = 1; cu = start[2] + k; cv = start[0] + i; }
            if (plane_ == 19) { u = 1; w = 2; n = 0; cu = start[1] + j; cv = start[2] + k; }
            const double r0 = std::hypot(start[u] - cu, start[w] - cv);
            const double r1 = std::hypot(target[u] - cu, target[w] - cv);
            if (std::abs(r0 - r1) > 0.005 && std::abs(r0 - r1) > 0.001 * r0) { error(33); return true; }
            arc.arc = true;
            arc.clockwise = motion_ == 2;
            arc.u = u; arc.w = w; arc.n = n;
            arc.cu = cu; arc.cv = cv; arc.radius = r0;
            arc.a0 = std::atan2(start[w] - cv, start[u] - cu);
            const double a1 = std::atan2(target[w] - cv, target[u] - cu);
            constexpr double kTwoPi = 6.283185307179586;
            double sweep = arc.clockwise ? arc.a0 - a1 : a1 - arc.a0;
            while (sweep < 0.0) sweep += kTwoPi;
            while (sweep >= kTwoPi) sweep -= kTwoPi;
            if (sweep < 1e-6) sweep = kTwoPi; // Same start and end: a full circle
            arc.sweep = sweep;
            arc.from = start;
            const double planar = sweep * r0;
            arc.length = std::max(1e-6, std::hypot(planar, target[n] - start[n]));
        }
        if (motion_ != 0 && feed_ <= 0.0) { error(22); return true; }
        if (planner_.size() >= planner_blocks) return false;
        Block b = arc;
        b.target = target;
        b.feed = feed_;
        b.rapid = motion_ == 0;
        planner_.push_back(b);
        if (state == "Idle") state = "Run";
        ok();
        return true;
    }

    bool handleDollar(const std::string& raw, const std::string& line) {
        if (line.rfind("$J=", 0) == 0) {
            if (state != "Idle" && state != "Jog") { error(8); return true; }
            std::string j = line.substr(3);
            double f = 0.0;
            if (!word(j, 'F', f)) { error(16); return true; }
            const bool inc = j.find("G91") != std::string::npos;
            std::array<double, 4> target = planner_.empty() ? pos : planner_.back().target;
            const std::array<double, 4> start = target;
            const char letters[4] = {'X', 'Y', 'Z', 'A'};
            double v = 0.0;
            for (size_t a = 0; a < axisCount(); ++a) {
                if (word(j, letters[a], v)) target[a] = inc ? start[a] + v : v + g54[a];
            }
            if (planner_.size() >= planner_blocks) return false;
            Block b;
            b.target = target;
            b.feed = f;
            b.jog = true;
            planner_.push_back(b);
            state = "Jog";
            ok();
            return true;
        }
        if (!planner_.empty() || (state != "Idle" && state != "Alarm")) return false;
        if (line == "$I") {
            switch (flavour_) {
                case Flavour::GrblHal:
                    out("[VER:1.1f.20230610:]");
                    out("[OPT:VNMHSL,35,1024," + std::to_string(axisCount()) + ",0]");
                    out(axisCount() == 4 ? "[AXS:4:XYZA]" : "[AXS:3:XYZ]");
                    out(plasmaActive() && plasma_mode != 0 ? "[NEWOPT:ENUMS,RT+,HOME,TC,SED,THC]"
                                                           : "[NEWOPT:ENUMS,RT+,HOME,TC,SED]");
                    out("[FIRMWARE:grblHAL]");
                    if (plasmaActive()) {
                        static const char* modes[] = {"Off", "Voltage", "Up/down", "Arc ok"};
                        out(std::string("[PLUGIN:PLASMA (") + modes[std::clamp(plasma_mode, 0, 3)] + ") v0.27]");
                    }
                    out("[AUX IO:0,4,0,0]");
                    break;
                case Flavour::FluidNC:
                    out("[VER:3.7 FluidNC v3.7.8:]");
                    out("[OPT:PHS]");
                    break;
                default:
                    out("[VER:1.1h.20190825:]");
                    out("[OPT:V,15,128]");
                    break;
            }
            ok();
            return true;
        }
        if (line == "$I+" && flavour_ == Flavour::GrblHal) {
            out("[VER:1.1f.20230610:]");
            out("[OPT:VNMHSL,35,1024," + std::to_string(axisCount()) + ",0]");
            out("[NEWOPT:ENUMS,RT+,HOME,TC,SED]");
            out("[FIRMWARE:grblHAL]");
            out("[DRIVER:emulator]");
            ok();
            return true;
        }
        if (line == "$N" && flavour_ != Flavour::FluidNC) {
            out("$N0=");
            out("$N1=");
            ok();
            return true;
        }
        // The board's own alarm / error texts (grblHAL $EA / $EE, FluidNC $A / $E)
        if (line == "$EA" && flavour_ == Flavour::GrblHal) {
            out("[ALARMCODE:1||Hard limit has been triggered. Machine position is likely lost due to sudden halt.]");
            out("[ALARMCODE:10||EStop asserted. Clear and reset]");
            out("[ALARMCODE:14||Spindle at speed timeout. Clear and reset]");
            ok();
            return true;
        }
        if (line == "$EE" && flavour_ == Flavour::GrblHal) {
            out("[ERRORCODE:1||G-code words consist of a letter and a value. Letter was not found.]");
            out("[ERRORCODE:79||Emergency stop active.]");
            ok();
            return true;
        }
        if (line == "$A" && flavour_ == Flavour::FluidNC) {
            out("[MSG:1: Hard Limit]");
            out("[MSG:10: Spindle Control]");
            ok();
            return true;
        }
        if (line == "$E" && flavour_ == Flavour::FluidNC) {
            out("[MSG:1: Expected command letter]");
            out("[MSG:61: SD card failed to open file]");
            ok();
            return true;
        }
        if (line == "$$") {
            if (plasmaActive() && !settings_.count(350)) {
                settings_[350] = std::to_string(plasma_mode);
                settings_[351] = "3.0";
                settings_[352] = "1.0";
                settings_[356] = "90";
                settings_[358] = "3.0";
                settings_[360] = "3";
            }
            for (const auto& [n, v] : settings_) out("$" + std::to_string(n) + "=" + v);
            ok();
            return true;
        }
        if (line == "$EG") {
            out("[SETTINGGROUP:0|0|Root]");
            out("[SETTINGGROUP:1|0|General]");
            out("[SETTINGGROUP:10|0|X-axis]");
            ok();
            return true;
        }
        if (line == "$ES") {
            out("[SETTING:0|1|Step pulse time|microseconds|6|#0.0|2.0||0|0]");
            out("[SETTING:100|10|X-axis travel resolution|step/mm|6|###0.000|||0|0]");
            ok();
            return true;
        }
        if (line == "$CD") {
            out("name: fake");
            out("axes:");
            out("  x:");
            out("    steps_per_mm: " + settings_[100]);
            out("    max_rate_mm_per_min: 3000.000");
            out("  y:");
            out("    steps_per_mm: 250.000");
            ok();
            return true;
        }
        if (line == "$#") {
            char buf[96];
            out("[G54:" + values(g54) + "]");
            out("[G55:" + values({}) + "]");
            out("[G28:0.000,0.000,0.000]");
            out("[G92:0.000,0.000,0.000]");
            out("[TLO:0.000]");
            out("[PRB:0.000,0.000,0.000:0]");
            ok();
            return true;
        }
        if (line == "$G") {
            out("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]");
            ok();
            return true;
        }
        if (line == "$X") {
            if (state == "Alarm") state = "Idle";
            out("[MSG:Caution: Unlocked]");
            ok();
            return true;
        }
        if (line.rfind("$H", 0) == 0) {
            if (settings_[22] != "1") { error(5); return true; }
            homing_axes_ = 0;
            if (line.size() == 2) homing_axes_ = axisCount() == 4 ? 15 : 7;
            for (char c : line.substr(2)) {
                if (c == 'X') homing_axes_ |= 1;
                if (c == 'Y') homing_axes_ |= 2;
                if (c == 'Z') homing_axes_ |= 4;
                if (c == 'A' && axisCount() == 4) homing_axes_ |= 8;
            }
            state = "Home";
            homing_left_ = 1.0;
            waiting_completion_ = true;
            return true;
        }
        if (line.size() > 2 && line[1] == '/' && flavour_ == Flavour::FluidNC) {
            ok();
            return true;
        }
        const size_t eq = line.find('=');
        if (eq != std::string::npos) {
            const int n = std::atoi(line.c_str() + 1);
            if (n == 10 && std::atoi(line.c_str() + eq + 1) > 3) { error(3); return true; }
            settings_[n] = raw.substr(raw.find('=') + 1);
            ok();
            return true;
        }
        error(3);
        return true;
    }

    std::mutex m_;
    Flavour flavour_;
    Clock::time_point last_{Clock::now()};
    std::string outbuf_;
    std::string partial_;
    std::deque<std::string> input_;
    size_t rx_used_ = 0, rx_used_line_ = 0;
    std::deque<Block> planner_;
    std::map<int, std::string> settings_;
    bool absolute_ = true;
    int  plane_ = 17;
    int  motion_ = 0;
    double feed_ = 0.0;
    double current_feed_ = 0.0;
    double dwell_left_ = 0.0;
    bool waiting_completion_ = false;
    bool completion_on_empty_ = false;
    double homing_left_ = 0.0;
    uint8_t homing_axes_ = 0;
    bool homed_ = false;
    unsigned homed_mask_ = 0;
    bool cutting_ = false;      // A feed move with the torch on
    bool thc_locked_ = false;
    double thc_offset_ = 0.0;   // Z moved by height control
};

// The stream the protocol opens; forwards to a shared FakeGrbl
class FakeGrblStream final : public transport::IByteStream {
public:
    explicit FakeGrblStream(std::shared_ptr<FakeGrbl> g) : g_(std::move(g)) {}
    bool open(const transport::ConnectionTarget&, const transport::StreamOptions&, std::string*) override {
        open_ = true;
        g_->onOpen();
        return true;
    }
    void close() override { open_ = false; }
    bool isOpen() const override { return open_; }
    long read(uint8_t* buf, size_t max, uint32_t timeout_ms) override {
        if (!open_) return -1;
        return g_->read(buf, max, timeout_ms);
    }
    bool write(const uint8_t* data, size_t n) override {
        if (!open_) return false;
        g_->write(data, n);
        return true;
    }
    bool accessesRealHardware() const override { return false; }

private:
    std::shared_ptr<FakeGrbl> g_;
    bool open_ = false;
};

} // namespace rnr::test
