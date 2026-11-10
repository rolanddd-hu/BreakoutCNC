#include "core/board/grbl/grbl_messages.h"
#include <cstdlib>
#include <cstring>

namespace rnr::board::grbl {

namespace {

bool startsWith(const std::string& s, const char* prefix) {
    return s.compare(0, std::strlen(prefix), prefix) == 0;
}

// Content between "[TAG:" and the closing ']' (tolerates a missing ']')
std::string bracketBody(const std::string& line, size_t tag_len) {
    std::string body = line.substr(tag_len);
    if (!body.empty() && body.back() == ']') body.pop_back();
    return body;
}

} // namespace

int parseNumberList(const std::string& text, std::array<double, 4>& out) {
    int n = 0;
    const char* p = text.c_str();
    while (*p && n < 4) {
        char* end = nullptr;
        const double v = std::strtod(p, &end);
        if (end == p) break;
        out[static_cast<size_t>(n++)] = v;
        p = end;
        if (*p != ',') break;
        ++p;
    }
    return n;
}

bool parseStatusReport(const std::string& line_in, StatusReport& out) {
    if (line_in.empty() || line_in[0] != '<') return false;
    std::string line = line_in.substr(1);
    if (!line.empty() && line.back() == '>') line.pop_back();

    out = StatusReport{};
    size_t pos = 0;
    bool first = true;
    while (pos <= line.size()) {
        size_t bar = line.find('|', pos);
        if (bar == std::string::npos) bar = line.size();
        const std::string field = line.substr(pos, bar - pos);
        pos = bar + 1;
        if (first) {
            first = false;
            const size_t colon = field.find(':');
            out.state = field.substr(0, colon);
            if (colon != std::string::npos) out.substate = std::atoi(field.c_str() + colon + 1);
            if (bar >= line.size()) break;
            continue;
        }
        const size_t colon = field.find(':');
        if (colon == std::string::npos) {
            if (bar >= line.size()) break;
            continue;
        }
        const std::string key = field.substr(0, colon);
        const std::string val = field.substr(colon + 1);
        if (key == "MPos") {
            out.axes_reported = parseNumberList(val, out.mpos);
            out.has_mpos = out.axes_reported > 0;
        } else if (key == "WPos") {
            out.axes_reported = parseNumberList(val, out.wpos);
            out.has_wpos = out.axes_reported > 0;
        } else if (key == "WCO") {
            out.has_wco = parseNumberList(val, out.wco) > 0;
        } else if (key == "FS" || key == "F") {
            std::array<double, 4> v{};
            const int n = parseNumberList(val, v);
            out.has_feed = n > 0;
            out.feed = v[0];
            out.spindle = (n > 1) ? v[1] : 0.0;
        } else if (key == "Ov") {
            std::array<double, 4> v{};
            if (parseNumberList(val, v) >= 3) {
                out.has_ov = true;
                out.ov_feed = static_cast<int>(v[0]);
                out.ov_rapid = static_cast<int>(v[1]);
                out.ov_spindle = static_cast<int>(v[2]);
            }
        } else if (key == "Pn") {
            out.has_pins = true;
            out.pins = val;
        } else if (key == "A") {
            out.has_accessories = true;
            out.accessories = val;
        } else if (key == "Bf") {
            std::array<double, 4> v{};
            if (parseNumberList(val, v) >= 2) {
                out.has_buffer = true;
                out.buffer_blocks = static_cast<int>(v[0]);
                out.buffer_bytes = static_cast<int>(v[1]);
            }
        } else if (key == "Ln") {
            out.has_line = true;
            out.line = std::atol(val.c_str());
        } else if (key == "H") {
            std::array<double, 4> v{};
            const int n = parseNumberList(val, v);
            if (n > 0) {
                out.has_homed = true;
                out.homed = v[0] != 0.0;
                out.homed_mask = (n > 1) ? static_cast<uint8_t>(v[1]) : (out.homed ? 0xFF : 0);
            }
        } else if (key == "MPG") {
            out.mpg = std::atoi(val.c_str()) != 0;
        } else if (key == "THC") {
            out.has_thc = true;
            out.thc_voltage = std::atof(val.c_str());
            const size_t comma = val.find(',');
            out.thc_flags = (comma == std::string::npos) ? std::string() : val.substr(comma + 1);
        }
        if (bar >= line.size()) break;
    }
    return !out.state.empty();
}

FirmwareMessage parseLine(const std::string& line_in) {
    FirmwareMessage m;
    std::string line = line_in;
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' ')) line.pop_back();
    m.raw = line;
    if (line.empty()) return m;

    if (line == "ok") {
        m.kind = MessageKind::Ack;
        return m;
    }
    if (startsWith(line, "error:")) {
        m.kind = MessageKind::Nack;
        m.code = std::atoi(line.c_str() + 6);
        return m;
    }
    if (line[0] == '<') {
        if (parseStatusReport(line, m.status)) {
            m.kind = MessageKind::Status;
        } else {
            m.kind = MessageKind::Text;
        }
        return m;
    }
    if (startsWith(line, "ALARM:")) {
        m.kind = MessageKind::Alarm;
        m.code = std::atoi(line.c_str() + 6);
        return m;
    }
    if (startsWith(line, "Grbl ") || startsWith(line, "GrblHAL ")) {
        m.kind = MessageKind::Welcome;
        m.text = line;
        return m;
    }
    if (line[0] == '$' && line.size() > 2) {
        // $n=value (numbered settings; grblHAL may use larger numbers)
        const size_t eq = line.find('=');
        if (eq != std::string::npos && eq > 1) {
            bool digits = true;
            for (size_t i = 1; i < eq; ++i) {
                if (line[i] < '0' || line[i] > '9') { digits = false; break; }
            }
            if (digits) {
                m.kind = MessageKind::Setting;
                m.name = line.substr(1, eq - 1);
                m.text = line.substr(eq + 1);
                return m;
            }
        }
        m.kind = MessageKind::Text;
        return m;
    }
    if (line[0] == '[') {
        const size_t colon = line.find(':');
        const std::string tag = (colon == std::string::npos) ? std::string() : line.substr(1, colon - 1);
        if (tag.empty()) {
            m.kind = MessageKind::Text;
            return m;
        }
        const std::string body = bracketBody(line, colon + 1);
        m.name = tag;
        if (tag == "PRB") {
            // [PRB:x,y,z:1]
            m.kind = MessageKind::Probe;
            const size_t last = body.rfind(':');
            m.value_count = parseNumberList(body.substr(0, last), m.position);
            m.flag = (last != std::string::npos) && std::atoi(body.c_str() + last + 1) != 0;
            return m;
        }
        if (tag == "GC") {
            m.kind = MessageKind::ParserState;
            m.text = body;
            return m;
        }
        if (tag == "MSG") {
            m.kind = MessageKind::Feedback;
            m.text = body;
            return m;
        }
        if (tag == "VER" || tag == "OPT" || tag == "NEWOPT" || tag == "AXS" || tag == "AUX IO" ||
            tag == "FIRMWARE" || tag == "BOARD" || tag == "DRIVER" || tag == "DRIVER VERSION" || tag == "PLUGIN") {
            m.kind = MessageKind::BuildInfo;
            m.text = body;
            return m;
        }
        if (tag == "SETTING" || tag == "SETTINGGROUP" || tag == "ALARMCODE" || tag == "ERRORCODE") {
            m.kind = MessageKind::Enumeration;
            m.text = body;
            return m;
        }
        if ((tag.size() >= 3 && tag[0] == 'G') || tag == "TLO" || tag == "HOME" || tag == "TLR") {
            // [G54:x,y,z], [G28:...], [G92:...], [TLO:z], grblHAL [G59.1:...], [HOME:...], [TLR:...]
            m.kind = MessageKind::Parameter;
            const size_t extra = body.find(':'); // [HOME:x,y,z:mask]
            m.value_count = parseNumberList(body.substr(0, extra), m.position);
            m.text = body;
            return m;
        }
        m.kind = MessageKind::Text;
        m.text = body;
        return m;
    }
    m.kind = MessageKind::Text;
    return m;
}

} // namespace rnr::board::grbl
