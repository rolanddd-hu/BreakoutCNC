#include "ui_common/macro_button.h"
#include "ui_common/theme.h"

namespace rnr::ui {

bool macroButton(const char* label, const core::MacroButtonConfig& btn, ImVec2 size,
                 const ImVec4* default_bg, const float* preview_rgb) {
    const ImVec4* bg = default_bg;
    ImVec4 custom;
    if (preview_rgb) {
        custom = ImVec4(preview_rgb[0], preview_rgb[1], preview_rgb[2], 1.0f);
        bg = &custom;
    } else if (btn.has_color) {
        custom = theme::fromRgb(btn.color_rgb);
        bg = &custom;
    }
    if (!bg) return ImGui::Button(label, size);

    theme::pushButtonColors(*bg);
    const bool pressed = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    return pressed;
}

bool MacroColorMenu::menu(size_t index, core::MacroButtonConfig& btn) {
    if (!ImGui::BeginMenu("Button Color")) return false;
    seen_ = true;
    bool changed = false;
    if (idx_ < 0 || static_cast<size_t>(idx_) != index) {
        // Opened for this button: start from its current color (or the default button color)
        idx_ = static_cast<int>(index);
        const ImVec4 c = btn.has_color ? theme::fromRgb(btn.color_rgb) : theme::pal().macro_btn;
        rgb_[0] = c.x;
        rgb_[1] = c.y;
        rgb_[2] = c.z;
    }
    ImGui::SetNextItemWidth(200.0f);
    ImGui::ColorPicker3("##MacroBtnColor", rgb_,
                        ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoSidePreview |
                            ImGuiColorEditFlags_DisplayHex);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        btn.has_color = true;
        btn.color_rgb = theme::toRgb(ImVec4(rgb_[0], rgb_[1], rgb_[2], 1.0f));
        changed = true;
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Default Color", nullptr, false, btn.has_color)) {
        btn.has_color = false;
        btn.color_rgb = 0;
        idx_ = -1;
        changed = true;
    }
    ImGui::EndMenu();
    return changed;
}

} // namespace rnr::ui
