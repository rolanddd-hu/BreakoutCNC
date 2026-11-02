#pragma once
#include "core/controller/motion_controller.h"
#include <string>

namespace rnr::ui {

// Contents of the "Lua Scripting & Macro Guide" window: tutorials, dialog cookbook, examples, hooks,
// searchable API reference, Mach3 compatibility and safety notes, with drawn diagrams and runnable
// dialog demos.
class LuaGuide {
public:
    // Renders the tab bar; call inside the guide window/popup.
    void render(core::IMotionController& controller);

private:
    void renderStartTab();
    void renderDialogsTab(core::IMotionController& controller);
    void renderExamplesTab();
    void renderHooksTab();
    void renderApiTab();
    void renderMach3Tab();
    void renderSafetyTab();

    // Code block with syntax highlighting, a Copy button and (optionally) a "Run demo" button
    void codeBlock(const char* id, const char* code, core::IMotionController* run_with = nullptr);

    char        api_filter_[64] = {0};
    std::string flash_id_;      // Code block whose button was just used
    std::string flash_text_;    // "Copied" / "Started" / error text
    double      flash_until_ = 0.0;
};

} // namespace rnr::ui
