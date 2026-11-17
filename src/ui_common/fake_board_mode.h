#pragma once
#include "core/config/settings_manager.h"
#include <string>

// --fake-board: the application drives the built-in GRBL-family emulator (FakeGrbl) instead of real hardware, to try
// the GRBL / grblHAL / FluidNC user interface without a board. Hardware access is switched off for the whole process,
// the Huanyang VFD link is off, and a separate settings file is used, so neither a connected board nor the normal
// settings are touched.
namespace rnr::ui {

inline constexpr const char* kFakeBoardFlag = "--fake-board";

// Settings file of the fake-board mode for a normal settings file name ("breakout_mill_settings.ini" ->
// "breakout_mill_settings_fakeboard.ini")
std::string fakeBoardSettingsFile(const std::string& normal_file);

// Call before the controller connects: disables hardware access, registers the emulator transports
// ("fake-grbl", "fake-grblhal", "fake-fluidnc", one emulated board each) and points every GRBL-family board profile at
// its emulator. The GRBL 1.1 emulator is selected when the settings select the RNR board. mill: BreakoutMill (the
// grblHAL and FluidNC emulators get an A axis), otherwise BreakoutCut (the grblHAL emulator gets the plasma plugin).
void enterFakeBoardMode(config::PersistentSettings& settings, bool mill);

} // namespace rnr::ui
