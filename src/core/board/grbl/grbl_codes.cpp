#include "core/board/grbl/grbl_codes.h"
#include <cstdlib>

namespace rnr::board::grbl {

AlarmInfo alarmInfo(int code) {
    switch (code) {
        case 1:  return {"Hard limit triggered: a limit switch was hit while moving", true, true};
        case 2:  return {"The move would leave the machine travel (board soft limit)", false, false};
        case 3:  return {"The board was reset while moving", true, true};
        case 4:  return {"The probe was already in contact before probing started", false, false};
        case 5:  return {"The probe did not touch anything", false, false};
        case 6:  return {"Homing was interrupted by a reset", true, true};
        case 7:  return {"The door was opened during homing", true, true};
        case 8:  return {"Homing could not move off the switch (check the pull-off distance and wiring)", true, true};
        case 9:  return {"Homing did not find the switch (check the travel settings and wiring)", true, true};
        case 10: return {"E-stop input is active on the board", true, false};
        case 11: return {"The board requires homing first", false, true};
        case 12: return {"A limit switch is active", false, false};
        case 13: return {"Spindle at speed timeout on the board", false, false};
        case 14: return {"Homing is required and limits are disabled", false, true};
        case 15: return {"The board's motor driver reported a fault", true, true};
        case 16: return {"Homing could not reach the required position", true, true};
        case 17: return {"Motor fault reported by the board", true, true};
        default: return {"Board alarm " + std::to_string(code), true, false};
    }
}

AlarmInfo fluidNcAlarmInfo(int code) {
    // FluidNC's ExecAlarm: 1-9 as GRBL 1.1, then its own (texts normally come from the board's $A list)
    switch (code) {
        case 10: return {"Spindle control fault", false, false};
        case 11: return {"A control input was active at startup", false, false};
        case 12: return {"Homing found an ambiguous switch state", false, true};
        case 13: return {"Hard stop", true, true};
        case 14: return {"The machine is not homed", false, true};
        case 15: return {"Initialisation failed (check the configuration)", false, false};
        default: return alarmInfo(code);
    }
}

bool parseEnumeratedCode(const std::string& body, int& code, std::string& text) {
    const size_t bar = body.find('|');
    if (bar == 0 || bar == std::string::npos) return false;
    for (size_t i = 0; i < bar; ++i) {
        if (body[i] < '0' || body[i] > '9') return false;
    }
    code = std::atoi(body.c_str());
    // The description is the last non-empty field
    text.clear();
    size_t start = bar + 1;
    while (start <= body.size()) {
        size_t end = body.find('|', start);
        if (end == std::string::npos) end = body.size();
        if (end > start) text = body.substr(start, end - start);
        start = end + 1;
    }
    return !text.empty();
}

bool parseCodeListLine(const std::string& line_in, int& code, std::string& text) {
    std::string line = line_in;
    if (line.rfind("[MSG:", 0) == 0) {
        line = line.substr(5);
        if (!line.empty() && line.back() == ']') line.pop_back();
        if (line.rfind("INFO: ", 0) == 0) line = line.substr(6);
    }
    size_t i = 0;
    while (i < line.size() && line[i] == ' ') ++i;
    const size_t digits = i;
    while (i < line.size() && line[i] >= '0' && line[i] <= '9') ++i;
    if (i == digits || i >= line.size() || line[i] != ':') return false;
    code = std::atoi(line.c_str() + digits);
    text = line.substr(i + 1);
    while (!text.empty() && text.front() == ' ') text.erase(text.begin());
    while (!text.empty() && (text.back() == ' ' || text.back() == '\r')) text.pop_back();
    return !text.empty();
}

std::string errorText(int code) {
    switch (code) {
        case 1:  return "the board could not read a word (expected a letter)";
        case 2:  return "a number is missing or invalid";
        case 3:  return "unknown $ command";
        case 4:  return "a negative value where a positive one is required";
        case 5:  return "homing is disabled on the board";
        case 6:  return "the step pulse must be at least 3 microseconds";
        case 7:  return "the board's settings could not be read (EEPROM); defaults restored";
        case 8:  return "the board is busy (only possible while idle)";
        case 9:  return "the board is locked: unlock or home first";
        case 10: return "soft limits need homing to be enabled";
        case 11: return "the line is too long";
        case 12: return "the step rate would be too high";
        case 13: return "the safety door is open";
        case 14: return "the build information line is too long";
        case 15: return "the jog move would leave the machine travel";
        case 16: return "invalid jog command";
        case 17: return "laser mode needs a PWM spindle output";
        case 20: return "unsupported G-code command";
        case 21: return "two commands of the same group on one line";
        case 22: return "no feed rate set";
        case 23: return "a value must be a whole number";
        case 24: return "two commands on one line need the axis words";
        case 25: return "a word is repeated on the line";
        case 26: return "an axis word is missing";
        case 27: return "invalid line number";
        case 28: return "a required value is missing";
        case 29: return "G59.x work coordinate systems are not supported";
        case 30: return "G53 needs G0 or G1";
        case 31: return "axis words that are not used";
        case 32: return "the arc needs axis words in its plane";
        case 33: return "the arc's end point is not on the arc";
        case 34: return "invalid arc radius";
        case 35: return "G2 / G3 needs an offset in its plane";
        case 36: return "unused words on the line";
        case 37: return "the tool length offset axis is not valid";
        case 38: return "the tool number is too large";
        default: return "error " + std::to_string(code);
    }
}

} // namespace rnr::board::grbl
