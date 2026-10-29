#pragma once
#include <string>
#include <vector>

namespace rnr::ui {

// Cross-platform native file open dialog
// - Windows: Win32 GetOpenFileNameA (comdlg32)
// - macOS:   Native AppleScript Cocoa choose-file dialog (osascript)
// - Linux:   Native desktop dialog (zenity / kdialog)
std::string openFileDialog(
    const std::string& title,
    const std::string& win_filter,
    const std::vector<std::string>& patterns);

// Specialized helpers for G-Code programs and Lua scripts
std::string openGCodeFileDialog();
std::string openLuaScriptDialog();

// Open file in default system editor (VS Code, Notepad, etc.)
bool openFileInDefaultEditor(const std::string& path);

} // namespace rnr::ui
