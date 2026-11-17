#pragma once
#include <array>
#include <cstdint>
#include <string>

// GRBL-family wire messages: one received line -> one typed message (GRBL_FAMILY_PLAN.md section 5.1).
// Unknown fields, tags and lines are ignored, never treated as errors.
namespace rnr::board::grbl {

struct StatusReport {
    std::string state;          // "Idle", "Run", "Hold", "Jog", "Home", "Alarm", "Door", "Check", "Sleep", "Tool"
    int         substate = -1;  // "Hold:0" -> 0, "Door:1" -> 1

    int  axes_reported = 0;
    bool has_mpos = false, has_wpos = false;
    std::array<double, 4> mpos{}, wpos{};
    bool has_wco = false;
    std::array<double, 4> wco{};

    bool   has_feed = false;
    double feed = 0.0, spindle = 0.0;

    bool has_ov = false;
    int  ov_feed = 100, ov_rapid = 100, ov_spindle = 100;

    bool        has_pins = false;  // Pn: present (absent = no input active)
    std::string pins;              // Pn: letters
    bool        has_accessories = false;
    std::string accessories;       // A: letters (S, C, F, M)

    bool has_buffer = false;
    int  buffer_blocks = 0, buffer_bytes = 0;

    bool has_line = false;
    long line = 0;

    bool    has_homed = false;     // grblHAL H:<homed>[,<axis mask>]
    bool    homed = false;
    uint8_t homed_mask = 0;
    bool    mpg = false;           // grblHAL MPG:1 (a pendant has control)

    // grblHAL plasma plugin: |THC:<arc voltage>[,<flags>] (A arc ok, T torch on, E THC enabled, R THC active,
    // V velocity lock (anti-dive), H void lock, U / D correcting up / down, O ohmic probe, F float switch, B breakaway)
    bool        has_thc = false;
    double      thc_voltage = 0.0;
    std::string thc_flags;
};

enum class MessageKind : uint8_t {
    Empty,
    Ack,         // ok
    Nack,        // error:N
    Status,      // <...>
    Alarm,       // ALARM:N
    Probe,       // [PRB:x,y,z:s]
    ParserState, // [GC:...]
    Parameter,   // [G54:...], [G92:...], [TLO:...], [HOME:...]
    Setting,     // $n=value
    BuildInfo,   // [VER:...], [OPT:...], [NEWOPT:...], [AXS:...], [AUX IO:...], [FIRMWARE:...], [BOARD:...]
    Welcome,     // Grbl 1.1h ['$' for help], GrblHAL ..., Grbl 3.x [FluidNC ...]
    Feedback,    // [MSG:...]
    Enumeration, // [SETTING:...], [SETTINGGROUP:...], [ALARMCODE:...], [ERRORCODE:...]
    Text,        // Anything else (console only), e.g. FluidNC $CD YAML lines
};

struct FirmwareMessage {
    MessageKind  kind = MessageKind::Empty;
    int          code = 0;            // Nack / Alarm number
    StatusReport status;              // Status
    std::array<double, 4> position{}; // Probe, Parameter (up to 4 values)
    int          value_count = 0;     // Parameter / Probe: number of values read
    bool         flag = false;        // Probe: contact made
    std::string  name;                // Parameter "G54", BuildInfo tag "VER", Setting number "100", Enumeration tag
    std::string  text;                // Setting value, Feedback text, BuildInfo / Enumeration content, Welcome line
    std::string  raw;                 // The line as received (without line end)
    uint32_t     ticket = 0;          // Ack / Nack: the sent line it answers (filled by the protocol)
};

FirmwareMessage parseLine(const std::string& line);

// Parses "<Idle|MPos:...|...>"; false when the line is not a status report
bool parseStatusReport(const std::string& line, StatusReport& out);

// Parses up to 4 comma-separated numbers; returns how many were read
int parseNumberList(const std::string& text, std::array<double, 4>& out);

} // namespace rnr::board::grbl
