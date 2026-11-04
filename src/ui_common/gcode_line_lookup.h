#pragma once
#include <cctype>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rnr::ui {

// Position of the digits of a leading "N<digits>" word (e.g. N100), or npos
inline size_t fileLineNumberDigits(const std::string& line, size_t& end_digit) {
    const size_t idx = line.find_first_not_of(" \t");
    if (idx == std::string::npos || (line[idx] != 'N' && line[idx] != 'n')) return std::string::npos;
    size_t d_idx = idx + 1;
    while (d_idx < line.size() && (line[d_idx] == ' ' || line[d_idx] == '\t')) d_idx++;
    if (d_idx >= line.size() || !std::isdigit(static_cast<unsigned char>(line[d_idx]))) return std::string::npos;
    end_digit = d_idx;
    while (end_digit < line.size() && std::isdigit(static_cast<unsigned char>(line[end_digit]))) end_digit++;
    return d_idx;
}

// Index in `lines` for what was typed in the go-to box:
//  "N64000" is an N number. "64000" is an N number if a line has it (the listing shows those numbers), and the
//  line number in the file otherwise. Empty when there is no such line or the text is not a number.
inline std::optional<size_t> findGCodeLine(const std::vector<std::string>& lines, const std::string& typed) {
    const size_t first = typed.find_first_not_of(" \t");
    if (first == std::string::npos) return std::nullopt;
    const size_t last = typed.find_last_not_of(" \t");
    std::string text = typed.substr(first, last - first + 1);

    const bool n_word = (text[0] == 'N' || text[0] == 'n');
    if (n_word) text.erase(0, 1);
    if (text.empty() || text.size() > 18) return std::nullopt;
    uint64_t wanted = 0;
    for (const char c : text) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return std::nullopt;
        wanted = wanted * 10 + static_cast<uint64_t>(c - '0');
    }

    for (size_t i = 0; i < lines.size(); ++i) {
        size_t end_digit = 0;
        const size_t d_idx = fileLineNumberDigits(lines[i], end_digit);
        if (d_idx == std::string::npos || end_digit - d_idx > 18) continue;
        uint64_t number = 0;
        for (size_t k = d_idx; k < end_digit; ++k) number = number * 10 + static_cast<uint64_t>(lines[i][k] - '0');
        if (number == wanted) return i;
    }
    if (!n_word && wanted >= 1 && wanted <= lines.size()) return static_cast<size_t>(wanted - 1);
    return std::nullopt;
}

} // namespace rnr::ui
