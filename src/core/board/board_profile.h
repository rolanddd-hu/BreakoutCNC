#pragma once
#include <cstdlib>
#include <map>
#include <string>

// Settings of one board, saved as [Board.<id>] in the settings file. A typed key / value store, so the settings
// file can keep the section of a board this build does not know (MULTI_BOARD_PLAN.md section 4.15).
namespace rnr::board {

struct BoardProfile {
    std::map<std::string, std::string> values;

    bool has(const std::string& key) const { return values.count(key) != 0; }

    std::string get(const std::string& key, const std::string& def = {}) const {
        const auto it = values.find(key);
        return it == values.end() ? def : it->second;
    }
    long getInt(const std::string& key, long def) const {
        const auto it = values.find(key);
        if (it == values.end() || it->second.empty()) return def;
        char* end = nullptr;
        const long v = std::strtol(it->second.c_str(), &end, 10);
        return (end && *end == '\0') ? v : def;
    }
    double getDouble(const std::string& key, double def) const {
        const auto it = values.find(key);
        if (it == values.end() || it->second.empty()) return def;
        char* end = nullptr;
        const double v = std::strtod(it->second.c_str(), &end);
        return (end && *end == '\0') ? v : def;
    }
    bool getBool(const std::string& key, bool def) const {
        const auto it = values.find(key);
        if (it == values.end()) return def;
        const auto& v = it->second;
        if (v == "1" || v == "true" || v == "yes") return true;
        if (v == "0" || v == "false" || v == "no") return false;
        return def;
    }

    void set(const std::string& key, const std::string& value) { values[key] = value; }
    void setInt(const std::string& key, long value) { values[key] = std::to_string(value); }
    void setBool(const std::string& key, bool value) { values[key] = value ? "1" : "0"; }

    bool operator==(const BoardProfile& o) const { return values == o.values; }
    bool operator!=(const BoardProfile& o) const { return values != o.values; }
};

} // namespace rnr::board
