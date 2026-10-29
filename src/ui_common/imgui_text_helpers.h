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

} // namespace rnr::ui
