#pragma once
#include <string>

namespace rnr::ui {

// The settings file, scripts/ and resources/ live next to the executable and are opened by relative paths.
// A file manager (Linux / macOS) or "Open with" (Windows) can start the app in another folder, so switch to
// the executable's folder first. Returns the first argument that is not a "--flag", made absolute against the
// original folder ("" if none).
std::string enterExecutableDirectory(int argc, char** argv);

// True if `flag` (e.g. "--smoke-test") is among the command-line arguments
bool hasCommandLineFlag(int argc, char** argv, const char* flag);

} // namespace rnr::ui
