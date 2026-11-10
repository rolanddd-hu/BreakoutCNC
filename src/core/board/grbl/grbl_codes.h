#pragma once
#include <string>

// GRBL-family alarm and error codes: texts in BreakoutCNC's own wording and their consequences
// (GRBL_FAMILY_PLAN.md section 15). grblHAL and FluidNC can describe their codes themselves; those texts are
// preferred when the board supplied them.
namespace rnr::board::grbl {

struct AlarmInfo {
    std::string text;
    bool        position_lost = false; // The machine position can no longer be trusted
    bool        needs_homing  = false; // Homing is the suggested way out
};

AlarmInfo alarmInfo(int code);          // GRBL 1.1 and grblHAL numbering
AlarmInfo fluidNcAlarmInfo(int code);   // FluidNC numbering (codes 10 and above differ)
std::string errorText(int code);

// One entry of the board's own code list: grblHAL "[ALARMCODE:1||Hard limit ...]" / "[ERRORCODE:...]" bodies
// ("1||text"), FluidNC "$A" / "$E" lines ("[MSG:1: Hard Limit]" or "1: Hard Limit"). False when the line is not one.
bool parseEnumeratedCode(const std::string& body, int& code, std::string& text);
bool parseCodeListLine(const std::string& line, int& code, std::string& text);

// Error codes the job runner treats specially
inline constexpr int kErrorJogTravelExceeded = 15;
inline constexpr int kErrorNotIdle           = 8;
inline constexpr int kErrorLocked            = 9;

} // namespace rnr::board::grbl
