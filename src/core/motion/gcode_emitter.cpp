#include "core/motion/gcode_emitter.h"
#include <cmath>
#include <cstdio>

namespace rnr::core::motion {

namespace {
const char kAxisLetters[4] = {'X', 'Y', 'Z', 'A'};
constexpr long kMaxLineNumber = 9999999; // GRBL's limit
} // namespace

void GcodeEmitter::reset() {
    motion_mode_ = -1;
    plane_known_ = false;
    feed_ = -1.0;
    axis_sent_ = {};
    axis_known_ = {};
    spindle_ = -1;
    s_ = -1.0;
    flood_ = -1;
    mist_ = -1;
}

std::string GcodeEmitter::num(double v, int decimals) {
    if (!std::isfinite(v)) v = 0.0;
    const double scale = std::pow(10.0, decimals);
    double r = std::round(v * scale) / scale;
    if (std::abs(r) < 0.5 / scale) r = 0.0;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, r);
    std::string s = buf;
    if (s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.') s.pop_back();
    }
    if (s == "-0") s = "0";
    return s;
}

std::string GcodeEmitter::prefix() {
    if (!d_.line_numbers) return {};
    line_no_ = (line_no_ >= kMaxLineNumber) ? 1 : line_no_ + 1;
    return "N" + std::to_string(line_no_);
}

void GcodeEmitter::push(std::string line, std::vector<std::string>& out) {
    if (line.empty()) return;
    if (line.size() > d_.max_line_chars && line[0] == 'N') {
        // Too long for the board with its line number: send it without (the position tracker covers it)
        size_t i = 1;
        while (i < line.size() && line[i] >= '0' && line[i] <= '9') ++i;
        line.erase(0, i);
    }
    out.push_back(std::move(line));
}

std::string GcodeEmitter::axisWords(const std::array<double, 4>& target, bool force_all) {
    std::string w;
    const int n = std::max(3, std::min(4, d_.axis_count));
    for (int a = 0; a < n; ++a) {
        const std::string v = num(target[static_cast<size_t>(a)], d_.xyz_decimals);
        if (force_all || !axis_known_[static_cast<size_t>(a)] || axis_sent_[static_cast<size_t>(a)] != v) {
            w += kAxisLetters[a];
            w += v;
            axis_sent_[static_cast<size_t>(a)] = v;
            axis_known_[static_cast<size_t>(a)] = true;
        }
    }
    return w;
}

void GcodeEmitter::rapid(const std::array<double, 4>& target, std::vector<std::string>& out) {
    const std::string axes = axisWords(target, false);
    if (axes.empty()) return;
    std::string line = prefix();
    if (motion_mode_ != 0) line += "G0";
    motion_mode_ = 0;
    line += axes;
    push(line, out);
}

void GcodeEmitter::linear(const std::array<double, 4>& target, double feed, std::vector<std::string>& out,
                          double s_value) {
    const std::string axes = axisWords(target, false);
    if (axes.empty()) return;
    std::string line = prefix();
    if (motion_mode_ != 1) line += "G1";
    motion_mode_ = 1;
    line += axes;
    const std::string f = num(std::max(1.0, feed), d_.feed_decimals);
    if (feed_ < 0.0 || num(feed_, d_.feed_decimals) != f) {
        line += "F" + f;
        feed_ = std::max(1.0, feed);
    }
    if (s_value >= 0.0) {
        const std::string s = num(s_value, 0);
        if (s_ < 0.0 || num(s_, 0) != s) {
            line += "S" + s;
            s_ = s_value;
        }
    }
    push(line, out);
}

void GcodeEmitter::arc(bool clockwise, uint8_t plane, const std::array<double, 4>& target,
                       const std::array<double, 3>& ijk, double feed, std::vector<std::string>& out,
                       double s_value) {
    if (!plane_known_ || plane_ != plane) {
        out.push_back("G" + std::to_string(plane));
        plane_ = plane;
        plane_known_ = true;
    }
    std::string line = prefix();
    const int mode = clockwise ? 2 : 3;
    line += (mode == 2) ? "G2" : "G3"; // Always stated: an arc line without its G word is easy to misread
    motion_mode_ = mode;
    line += axisWords(target, true); // An arc needs its axis words even when an axis does not change
    const char* off_letters = (plane == 18) ? "IK" : (plane == 19) ? "JK" : "IJ";
    const size_t u = (plane == 19) ? 1 : 0;
    const size_t v = (plane == 17) ? 1 : 2;
    line += off_letters[0] + num(ijk[u], d_.ijk_decimals);
    line += off_letters[1] + num(ijk[v], d_.ijk_decimals);
    const std::string f = num(std::max(1.0, feed), d_.feed_decimals);
    if (feed_ < 0.0 || num(feed_, d_.feed_decimals) != f) {
        line += "F" + f;
        feed_ = std::max(1.0, feed);
    }
    if (s_value >= 0.0) {
        const std::string s = num(s_value, 0);
        if (s_ < 0.0 || num(s_, 0) != s) {
            line += "S" + s;
            s_ = s_value;
        }
    }
    push(line, out);
    if (plane != 17) {
        out.push_back("G17");
        plane_ = 17;
    }
}

void GcodeEmitter::dwell(double seconds, std::vector<std::string>& out) {
    if (!(seconds > 0.0)) return;
    out.push_back("G4P" + num(seconds, 3));
}

void GcodeEmitter::outputs(bool spindle_on, bool ccw, double s_value, bool laser_dynamic, bool flood, bool mist,
                           std::vector<std::string>& out) {
    (void)laser_dynamic;
    const int want = spindle_on ? (ccw ? 4 : 3) : 0;
    std::string line;
    if (want != spindle_) {
        line += (want == 0) ? "M5" : (want == 4 ? "M4" : "M3");
        spindle_ = want;
    }
    if (spindle_on && s_value >= 0.0) {
        const std::string s = num(s_value, 0);
        if (s_ < 0.0 || num(s_, 0) != s || !line.empty()) {
            line += "S" + s;
            s_ = s_value;
        }
    }
    push(line, out);

    const int f = flood ? 1 : 0;
    const int m = (mist && d_.mist) ? 1 : 0;
    if (f == flood_ && m == mist_) return;
    // M7, M8 and M9 are one modal group: GRBL refuses two of them in a block (error 21), so one per line
    const bool unknown = flood_ < 0 || mist_ < 0;
    if (unknown || (flood_ == 1 && f == 0) || (mist_ == 1 && m == 0)) {
        // M9 switches both off; the one that stays on is sent again
        out.push_back("M9");
        if (f) out.push_back("M8");
        if (m) out.push_back("M7");
    } else {
        if (f && flood_ != 1) out.push_back("M8");
        if (m && mist_ != 1) out.push_back("M7");
    }
    flood_ = f;
    mist_ = m;
}

void GcodeEmitter::auxOutput(int index, bool on, bool synchronized, std::vector<std::string>& out) {
    if (index < 0 || index >= d_.aux_outputs) return;
    const int m = synchronized ? (on ? 62 : 63) : (on ? 64 : 65);
    out.push_back("M" + std::to_string(m) + "P" + std::to_string(index));
}

std::string GcodeEmitter::jogLine(const std::array<double, 4>& delta, double feed, int axis_count) {
    std::string line = "$J=G91G21";
    const int n = std::max(3, std::min(4, axis_count));
    for (int a = 0; a < n; ++a) {
        const std::string v = num(delta[static_cast<size_t>(a)], 3);
        if (v != "0") {
            line += kAxisLetters[a];
            line += v;
        }
    }
    line += "F" + num(std::max(1.0, feed), 0);
    return line;
}

} // namespace rnr::core::motion
