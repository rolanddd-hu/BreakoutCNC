#pragma once
#include "core/util/app_paths.h"
#include "core/util/utf8_path.h"
#include <filesystem>
#include <string>

namespace rnr::scripting {

// Finds a script file: as given (absolute, or relative to the working directory), then relative to the executable's
// folder, its parent and grandparent (the repository root when run from build/Release). UTF-8 in and out.
inline std::string resolveScriptPath(const std::string& path_str) {
    if (path_str.empty()) return {};

    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path p = util::pathFromUtf8(path_str);
    auto found = [&](const fs::path& c) { return fs::exists(c, ec) && !fs::is_directory(c, ec); };

    if (found(p)) return util::pathToUtf8(fs::absolute(p, ec));
    if (p.is_absolute()) return {};

    const fs::path exe = util::executablePath();
    if (exe.empty()) return {};
    fs::path dir = exe.parent_path();
    for (int up = 0; up < 3 && !dir.empty(); ++up) {
        const fs::path c = dir / p;
        if (found(c)) return util::pathToUtf8(fs::absolute(c, ec));
        dir = dir.parent_path();
    }
    return {};
}

} // namespace rnr::scripting
