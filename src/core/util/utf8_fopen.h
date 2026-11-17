#pragma once
// Forced into the Lua sources on Windows (CMakeLists.txt): Lua's fopen / freopen take UTF-8 file names, like the
// rest of BreakoutCNC (see utf8_path.h). Implemented in utf8_path.cpp.
#include <stdio.h>
#ifdef _WIN32
FILE* rnr_utf8_fopen(const char* name, const char* mode);
FILE* rnr_utf8_freopen(const char* name, const char* mode, FILE* stream);
#define fopen rnr_utf8_fopen
#define freopen rnr_utf8_freopen
#endif
