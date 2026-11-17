#pragma once
#include <imgui.h>
#include <cstdarg>

namespace rnr::ui {

// ImGui::BulletText() never wraps (it ignores PushTextWrapPos). This draws the bullet and wraps the text at the
// right edge of the window/child, continuation lines indented under the first.
inline void BulletTextWrapped(const char* fmt, ...) IM_FMTARGS(1);
inline void BulletTextWrapped(const char* fmt, ...) {
    ImGui::Bullet(); // Leaves the cursor on the same line, after the bullet
    va_list args;
    va_start(args, fmt);
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextV(fmt, args);
    ImGui::PopTextWrapPos();
    va_end(args);
}

// Width of a check box with this label (the part after "##" is not shown)
inline float checkboxWidth(const char* label) {
    return ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(label, nullptr, true).x;
}

// SameLine() when an item next_w wide still fits to the right of the last one, else the next item starts a new line
// (a check box after a slider with a long label was cut off at the right edge)
inline void sameLineIfFits(float next_w) {
    // After an item the cursor is at the start of the next line: from there the region reaches the right edge
    const float right_edge = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
    if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + next_w <= right_edge) ImGui::SameLine();
}

} // namespace rnr::ui
