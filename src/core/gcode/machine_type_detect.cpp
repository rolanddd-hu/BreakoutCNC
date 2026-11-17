// Machine type detection: does a program look like it was made for a mill, a plasma or a laser?
#include "core/gcode/gcode_parser.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace rnr::gcode {

static bool containsCi(const std::string& str, const std::string& sub) {
    if (sub.empty() || str.size() < sub.size()) return false;
    auto it = std::search(
        str.begin(), str.end(),
        sub.begin(), sub.end(),
        [](char ch1, char ch2) {
            return std::tolower(static_cast<unsigned char>(ch1)) == std::tolower(static_cast<unsigned char>(ch2));
        }
    );
    return it != str.end();
}

static void addUniqueIndicator(std::vector<std::string>& vec, const std::string& item) {
    for (const auto& existing : vec) {
        if (existing == item) return;
    }
    vec.push_back(item);
}

MachineType GCodeParser::detectTargetMachine(
    const std::string& gcode_text,
    int* out_score,
    std::vector<std::string>* out_indicators)
{
    if (gcode_text.empty()) {
        if (out_score) *out_score = 0;
        return MachineType::Unknown;
    }

    int mill_score   = 0;
    int plasma_score = 0;
    int laser_score  = 0;

    std::vector<std::string> mill_reasons;
    std::vector<std::string> plasma_reasons;
    std::vector<std::string> laser_reasons;

    std::istringstream stream(gcode_text);
    std::string line;
    int line_num = 0;
    int last_m3_line = -100;
    int g1_inline_s_count = 0;

    // Scan up to the first 4000 lines to maintain ultra-fast UI loading
    while (std::getline(stream, line) && line_num < 4000) {
        ++line_num;

        // Check comments in the line
        std::string comment;
        auto paren_open = line.find('(');
        if (paren_open != std::string::npos) {
            auto paren_close = line.find(')', paren_open + 1);
            if (paren_close != std::string::npos) {
                comment += " " + line.substr(paren_open + 1, paren_close - paren_open - 1);
            }
        }
        auto semi = line.find(';');
        if (semi != std::string::npos) {
            comment += " " + line.substr(semi + 1);
        }

        if (!comment.empty()) {
            // Plasma CAM signatures
            if (containsCi(comment, "sheetcam")) {
                plasma_score += 45;
                addUniqueIndicator(plasma_reasons, "Header specifies SheetCam Plasma CAM");
            }
            if (containsCi(comment, "hypertherm") || containsCi(comment, "powermax")) {
                plasma_score += 40;
                addUniqueIndicator(plasma_reasons, "Comment references Hypertherm Plasma Torch");
            }
            if (containsCi(comment, "pierce height") || containsCi(comment, "cut height") || containsCi(comment, "pierce delay")) {
                plasma_score += 35;
                addUniqueIndicator(plasma_reasons, "Plasma process parameters (Pierce / Cut Height)");
            }
            if (containsCi(comment, "ohmic") || containsCi(comment, "floating head") || containsCi(comment, "torch on")) {
                plasma_score += 30;
                addUniqueIndicator(plasma_reasons, "Plasma torch sensing & ignition comments");
            }
            if (containsCi(comment, "plasma")) {
                plasma_score += 25;
                addUniqueIndicator(plasma_reasons, "Header/comment explicitly references Plasma");
            }

            // Laser CAM signatures
            if (containsCi(comment, "lightburn")) {
                laser_score += 50;
                addUniqueIndicator(laser_reasons, "Header specifies LightBurn Laser Software");
            }
            if (containsCi(comment, "lasergrbl") || containsCi(comment, "laserweb") || containsCi(comment, "rdworks")) {
                laser_score += 45;
                addUniqueIndicator(laser_reasons, "Header specifies Laser engraving CAM software");
            }
            if (containsCi(comment, "air assist") || containsCi(comment, "laser power")) {
                laser_score += 30;
                addUniqueIndicator(laser_reasons, "Laser optics & air assist references");
            }
            if (containsCi(comment, "laser")) {
                laser_score += 20;
                addUniqueIndicator(laser_reasons, "Header/comment explicitly references Laser");
            }

            // Mill CAM signatures
            if (containsCi(comment, "fusion 360") || containsCi(comment, "autodesk") ||
                containsCi(comment, "vcarve") || containsCi(comment, "aspire") || containsCi(comment, "vectric") ||
                containsCi(comment, "mastercam") || containsCi(comment, "solidcam") || containsCi(comment, "carbide create")) {
                mill_score += 45;
                addUniqueIndicator(mill_reasons, "Header specifies Milling CAM software (Fusion 360 / Vectric / Mastercam)");
            }
            if (containsCi(comment, "endmill") || containsCi(comment, "end mill") || containsCi(comment, "ball nose") ||
                containsCi(comment, "v-bit") || containsCi(comment, "face mill") || containsCi(comment, "twist drill") ||
                containsCi(comment, "chamfer mill")) {
                mill_score += 40;
                addUniqueIndicator(mill_reasons, "Tool definitions specify Milling Cutters (Endmill / Drill)");
            }
            if (containsCi(comment, "spindle speed") || containsCi(comment, "spindle on")) {
                mill_score += 20;
                addUniqueIndicator(mill_reasons, "Header specifies Milling Spindle Parameters");
            }
        }

        // Strip comments to inspect raw executable G-code
        std::string code = line;
        auto semi_pos = code.find(';');
        if (semi_pos != std::string::npos) code = code.substr(0, semi_pos);
        while (true) {
            auto p1 = code.find('(');
            if (p1 == std::string::npos) break;
            auto p2 = code.find(')', p1 + 1);
            if (p2 == std::string::npos) { code = code.substr(0, p1); break; }
            code.erase(p1, p2 - p1 + 1);
        }

        if (code.empty()) continue;

        // Convert to uppercase
        for (char& c : code) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

        // Tool change (M6 / M06 / Tx M6) -> Strong Mill
        if (code.find("M6") != std::string::npos || code.find("M06") != std::string::npos) {
            mill_score += 50;
            addUniqueIndicator(mill_reasons, "Tool change command (M6)");
        }

        // Tool length offset (G43 / G49) -> Strong Mill
        if (code.find("G43") != std::string::npos) {
            mill_score += 35;
            addUniqueIndicator(mill_reasons, "Tool length offset compensation (G43)");
        }

        // Canned drilling cycles (G81, G82, G83, G73, G84, G85) -> Exclusive to Mill
        if (code.find("G81") != std::string::npos || code.find("G82") != std::string::npos ||
            code.find("G83") != std::string::npos || code.find("G73") != std::string::npos ||
            code.find("G84") != std::string::npos || code.find("G85") != std::string::npos) {
            mill_score += 50;
            addUniqueIndicator(mill_reasons, "Canned drilling / pecking cycles (G81-G83)");
        }

        // Cutter radius compensation (G41 / G42)
        if (code.find("G41") != std::string::npos || code.find("G42") != std::string::npos) {
            mill_score += 25;
            addUniqueIndicator(mill_reasons, "Cutter radius compensation (G41/G42)");
        }

        // Spindle speed checks (S...)
        auto s_pos = code.find('S');
        if (s_pos != std::string::npos && s_pos + 1 < code.size() && std::isdigit(static_cast<unsigned char>(code[s_pos + 1]))) {
            try {
                size_t num_len = 0;
                long s_val = std::stol(code.substr(s_pos + 1), &num_len);
                if (s_val >= 2000) {
                    mill_score += 30;
                    addUniqueIndicator(mill_reasons, "High spindle RPM commanded (S" + std::to_string(s_val) + ")");
                } else if (s_val > 0 && s_val <= 1000) {
                    if (code.find("M4") != std::string::npos || code.find("G1") != std::string::npos) {
                        laser_score += 25;
                        addUniqueIndicator(laser_reasons, "PWM laser power level (S" + std::to_string(s_val) + ")");
                    }
                }
            } catch (...) {}
        }

        // Dynamic laser mode (M4 / M04)
        if (code.find("M4") != std::string::npos || code.find("M04") != std::string::npos) {
            laser_score += 45;
            addUniqueIndicator(laser_reasons, "Dynamic laser power mode (M4)");
        }

        // Inline power adjustments on cutting moves (e.g. G1 X... Y... S...)
        if (code.find("G1") != std::string::npos && code.find('S') != std::string::npos && code.find('Z') == std::string::npos) {
            if (++g1_inline_s_count >= 5) {
                laser_score += 35;
                addUniqueIndicator(laser_reasons, "Continuous inline laser power modulation on cut vectors");
            }
        }

        // Plasma Torch Fire (M3 / M03) and Pierce Dwell (G4 / G04) pattern
        if (code.find("M3") != std::string::npos || code.find("M03") != std::string::npos) {
            // If line does NOT command a high spindle RPM (e.g. S12000), record M3 line for pierce sequence check
            if (code.find("S1") == std::string::npos && code.find("S2") == std::string::npos) {
                last_m3_line = line_num;
            }
        }

        if (code.find("G4") != std::string::npos || code.find("G04") != std::string::npos) {
            if (line_num - last_m3_line <= 2 && (line_num - last_m3_line) >= 0) {
                plasma_score += 55;
                addUniqueIndicator(plasma_reasons, "Plasma torch pierce dwell sequence (M3 followed by G4 dwell)");
            }
        }

        // Deep negative Z plunge (Z-1.0 or deeper) typical for milling operations
        auto z_pos = code.find("Z-");
        if (z_pos != std::string::npos && z_pos + 2 < code.size() && std::isdigit(static_cast<unsigned char>(code[z_pos + 2]))) {
            mill_score += 15;
            addUniqueIndicator(mill_reasons, "Multi-depth material cutting (Z negative plunge)");
        }
    }

    // Determine target machine type with confidence threshold
    const int kConfidenceThreshold = 35;
    MachineType detected = MachineType::Unknown;
    int detected_score = 0;
    std::vector<std::string> chosen_reasons;

    if (mill_score >= kConfidenceThreshold && mill_score > plasma_score + 15 && mill_score > laser_score + 15) {
        detected = MachineType::Mill;
        detected_score = mill_score;
        chosen_reasons = mill_reasons;
    } else if (plasma_score >= kConfidenceThreshold && plasma_score > mill_score + 15 && plasma_score > laser_score + 15) {
        detected = MachineType::Plasma;
        detected_score = plasma_score;
        chosen_reasons = plasma_reasons;
    } else if (laser_score >= kConfidenceThreshold && laser_score > mill_score + 15 && laser_score > plasma_score + 15) {
        detected = MachineType::Laser;
        detected_score = laser_score;
        chosen_reasons = laser_reasons;
    }

    if (out_score) *out_score = detected_score;
    if (out_indicators) *out_indicators = chosen_reasons;

    return detected;
}

MachineTypeCheckResult GCodeParser::checkMachineCompatibility(
    const std::string& gcode_text,
    MachineType current_machine)
{
    int score = 0;
    std::vector<std::string> indicators;
    MachineType detected = detectTargetMachine(gcode_text, &score, &indicators);

    MachineTypeCheckResult result;
    result.detected_type = detected;
    result.confidence_score = score;
    result.indicators = indicators;

    auto typeToName = [](MachineType t) -> std::string {
        switch (t) {
            case MachineType::Mill:   return "CNC Milling Machine";
            case MachineType::Plasma: return "CNC Plasma Cutter";
            case MachineType::Laser:  return "CNC Laser Cutter";
            default:                  return "Standard CNC Machine";
        }
    };

    result.detected_name = typeToName(detected);
    result.current_name  = typeToName(current_machine);

    // If ambiguous or detected matches current machine environment, there is no mismatch
    if (detected == MachineType::Unknown || detected == current_machine) {
        result.has_mismatch = false;
        return result;
    }

    // Mismatch detected!
    result.has_mismatch = true;
    return result;
}

} // namespace rnr::gcode
