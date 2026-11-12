#pragma once
#include <filesystem>

namespace rnr::util {

// Full path of the running executable (empty if it cannot be found). Windows, Linux and macOS.
std::filesystem::path executablePath();

} // namespace rnr::util
