#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// G-code for firmware-planned boards (GRBL_FAMILY_PLAN.md section 7). The dialect is a data table of what the
// firmware accepts; the emitter turns resolved moves (already in the firmware's coordinate frame) into lines and
// only writes the words that change.
namespace rnr::core::motion {

struct GcodeDialect {
    size_t      max_line_chars = 80;
    int         xyz_decimals = 3;
    int         ijk_decimals = 4;
    int         feed_decimals = 0;
    int         axis_count = 3;            // XYZ or XYZA
    bool        line_numbers = false;      // N words (the firmware reports Ln:)
    bool        arcs = true;
    bool        mist = false;              // M7
    int         aux_outputs = 0;           // M62-M65 P0..P(n-1)
    std::string wait_idle = "G4P0";        // Answered only after all queued motion has finished
    double      arc_radius_tolerance_mm = 0.002;
};

class GcodeEmitter {
public:
    explicit GcodeEmitter(GcodeDialect d = {}) : d_(std::move(d)) {}

    const GcodeDialect& dialect() const { return d_; }
    void setDialect(const GcodeDialect& d) { d_ = d; reset(); }

    // Forget everything sent: the next lines state every word again (after a sync point, a reset, a console command)
    void reset();

    // Number formatting: fixed decimals, trailing zeros removed, no exponent, never "-0"
    static std::string num(double v, int decimals);

    void rapid(const std::array<double, 4>& target, std::vector<std::string>& out);
    // s_value >= 0: an S word on the same line (laser power in M4 mode)
    void linear(const std::array<double, 4>& target, double feed_mm_min, std::vector<std::string>& out,
                double s_value = -1.0);
    // ijk: centre minus start, in the plane's two axes (the third is ignored)
    void arc(bool clockwise, uint8_t plane, const std::array<double, 4>& target, const std::array<double, 3>& ijk,
             double feed_mm_min, std::vector<std::string>& out, double s_value = -1.0);
    void dwell(double seconds, std::vector<std::string>& out);

    // Spindle / laser and coolant. Only the changes are written; M9 switches both coolants off, so the one that
    // stays on is sent again.
    void outputs(bool spindle_on, bool ccw, double s_value, bool laser_dynamic, bool flood, bool mist,
                 std::vector<std::string>& out);
    void auxOutput(int index, bool on, bool synchronized, std::vector<std::string>& out);

    // Line number of the last motion line (0 = none)
    long lastLineNumber() const { return line_no_; }

    // "$J=G91G21X1.5Y-2F600" (incremental jog in mm)
    static std::string jogLine(const std::array<double, 4>& delta, double feed_mm_min, int axis_count);

private:
    std::string axisWords(const std::array<double, 4>& target, bool force_all);
    std::string prefix();
    void push(std::string line, std::vector<std::string>& out);

    GcodeDialect d_;
    int    motion_mode_ = -1;      // 0..3, -1 unknown
    int    plane_ = 17;
    bool   plane_known_ = false;
    double feed_ = -1.0;
    std::array<std::string, 4> axis_sent_{};
    std::array<bool, 4> axis_known_{};
    int    spindle_ = -1;          // -1 unknown, 0 off, 3 M3, 4 M4
    double s_ = -1.0;
    int    flood_ = -1, mist_ = -1;
    long   line_no_ = 0;
};

} // namespace rnr::core::motion
