#pragma once
#include "core/controller/motion_controller.h"
#include <imgui.h>

namespace rnr::ui {

// Macro (Lua script) button in its custom color. Without a custom color it uses default_bg, or the style's normal
// button color when default_bg is null. preview_rgb (if set) overrides both while the user picks a color.
bool macroButton(const char* label, const core::MacroButtonConfig& btn, ImVec2 size,
                 const ImVec4* default_bg = nullptr, const float* preview_rgb = nullptr);

// "Button Color" submenu for a macro button's right-click menu: a color wheel (the button shows the color live
// while dragging) and "Default Color". The color is only stored on release, so the settings are not re-applied on
// every drag frame. Call beginFrame() before the buttons are drawn and endFrame() after them.
class MacroColorMenu {
public:
    void beginFrame() noexcept { seen_ = false; }
    void endFrame() noexcept {
        if (!seen_) idx_ = -1;
    }

    // Live color for button `index` while its picker is open, else nullptr
    const float* previewFor(size_t index) const noexcept {
        return (idx_ >= 0 && static_cast<size_t>(idx_) == index) ? rgb_ : nullptr;
    }

    // Draws the submenu inside an open popup; returns true when btn was changed and must be saved
    bool menu(size_t index, core::MacroButtonConfig& btn);

private:
    int   idx_    = -1;
    float rgb_[3] = {0.0f, 0.0f, 0.0f};
    bool  seen_   = false;
};

} // namespace rnr::ui
