#pragma once
#include "core/controller/motion_controller.h"
#include "ui_common/lua_guide.h"
#include <imgui.h>
#include <string>
#include <vector>
#include <mutex>
#include <functional>

namespace rnr::ui {

class ScriptingPanel {
public:
    ScriptingPanel();
    ~ScriptingPanel();

    // Render the complete Scripting & Macros Deck (Console, REPL, Quick Macro buttons, dynamic + add/edit)
    void render(
        core::IMotionController& controller,
        core::MachineConfig& cfg,
        const std::function<void()>& on_config_changed = nullptr);

    // Modal popup handler for interactive cnc.prompt() dialogs
    void renderPromptModal(core::IMotionController& controller);

    // Renders the circular information icon button (white circle and 'i', transparent background).
    // Returns true when clicked.
    static bool renderHelpIconButton(const char* str_id, ImVec2 pos, float size = 18.0f);

    // Open/show the comprehensive Lua scripting help popup modal
    void openHelpModal() { show_help_modal_ = true; }

    // Renders the comprehensive Lua help popup modal window
    void renderHelpModal(core::IMotionController& controller);

    // Log append method (connected to ScriptRunner callback)
    void appendLog(const std::string& message, bool is_error = false);
    void clearLogs();

private:
    void renderPromptModalImpl(core::IMotionController& controller);
    struct LogEntry {
        std::string timestamp;
        std::string text;
        bool        is_error = false;
    };

    std::vector<LogEntry> logs_;
    mutable std::mutex    log_mutex_;
    char                  repl_input_[512] = {0};
    char                  prompt_input_buf_[256] = {0}; // Text of the active cnc.input dialog
    bool                  auto_scroll_     = true;
    size_t                max_logs_        = 500;

    // Help guide modal state
    bool show_help_modal_ = false;
    LuaGuide guide_;
    bool prompt_was_active_ = false; // A script dialog was showing last frame
    int  prompt_popup_level_ = 0;     // Popup stack level the script dialog was opened at
    bool guide_open_ = false;         // Guide window open: script dialogs nest inside it

    // Add / Edit macro button modal state
    bool show_add_modal_  = false;
    bool show_edit_modal_ = false;
    int  edit_btn_index_  = -1;
    char modal_label_[64] = {0};
    char modal_path_[260] = {0};
};

} // namespace rnr::ui
