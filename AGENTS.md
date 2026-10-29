# BreakoutCNC — Agent Guidelines & UI Design Rules

This document establishes mandatory design rules and technical constraints for all AI coding agents working on the BreakoutCNC codebase. Adhering to these rules prevents common Dear ImGui layout bugs, text clipping, and UI responsiveness issues.

---

## 1. Text Wrapping & Clipping Prevention (MANDATORY)

### The Problem
In Dear ImGui, standard text rendering functions (`ImGui::Text()`, `ImGui::TextDisabled()`, `ImGui::TextColored()`, etc.) **do not wrap text automatically**. They calculate layout on a single line and will truncate / clip text against the right window/child border if the string exceeds the available width.

### Mandatory Rules
1. **Never use single-line text calls for explanations or dynamic status messages:**
   Any sentence, description, or status message longer than a couple of short words must be wrapped.
2. **How to wrap properly:**
   - **Option A (Block Wrapping):** Wrap the text block with `PushTextWrapPos(0.0f)` / `PopTextWrapPos()`:
     ```cpp
     ImGui::PushTextWrapPos(0.0f); // 0.0f wraps at the right boundary of current window/child
     ImGui::TextColored(col, "Status: %s", long_message);
     ImGui::TextDisabled("Selected smoothness (%d%%) will be automatically applied upon probe completion.", pct);
     ImGui::PopTextWrapPos();
     ```
   - **Option B (TextWrapped):** Use `ImGui::TextWrapped(...)` for standard-colored paragraphs:
     ```cpp
     ImGui::TextWrapped("Toolpath Z-coordinates are actively warped to workpiece surface topology based on probed height map.");
     ```
3. **Bullets never wrap:** `ImGui::BulletText()` ignores `PushTextWrapPos()` and always clips. Use `rnr::ui::BulletTextWrapped(...)` from [`imgui_text_helpers.h`](src/ui_common/imgui_text_helpers.h) for any bullet longer than a few words. Checkbox, button and combo labels do not wrap either: keep them short and put explanations in wrapped text or a tooltip.
4. **Child Windows:** When rendering status alerts or dynamic cards, configure child windows with `ImGuiChildFlags_AutoResizeY` so that multi-line wrapped text expands the vertical space automatically rather than clipping at a fixed height.

---

## 2. Button Sizing & Font Vertical Padding

### The Problem
Setting button heights too small (e.g. `<= 22.0f`) causes text to touch or overlap the top/bottom button bevels, clipping font descenders (like `p`, `y`, `g`) and looking squished.

### Mandatory Rules
- **Preset & Secondary Buttons:** Minimum height **`28.0f`** (e.g. `ImVec2(w, 28.0f)`).
- **Primary & Action Buttons:** Minimum height **`32.0f` to `38.0f`**.
- **Large Cycle/Safety Buttons:** Minimum height **`42.0f` to `50.0f`**.
- Always ensure sufficient horizontal padding: if calculating dynamic button widths, use `ImGui::CalcTextSize(label).x + 32.0f` padding.

---

## 3. Side-by-Side Sliders & Inputs (Label Width Budgeting)

### The Problem
When placing two sliders or inputs side-by-side using `ImGui::SameLine()`, developers frequently write:
```cpp
// INCORRECT - CAUSES RIGHT LABEL CLIPPING:
const float half_w = (avail - 10.0f) * 0.5f;
ImGui::SetNextItemWidth(half_w);
ImGui::SliderInt("X Points##Srf", &px, 2, 30);
ImGui::SameLine();
ImGui::SetNextItemWidth(half_w);
ImGui::SliderInt("Y Points##Srf", &py, 2, 30); // "Y Points" label is pushed completely off-screen!
```
`ImGui::SetNextItemWidth()` sets the width of **only the slider drag-bar widget**, *excluding* the text label to its right. Sizing the drag-bar to `half_w` means the drag-bar plus label takes up `half_w + label_width`, pushing the second column's label off the screen.

### Mandatory Rules
Always budget the column width and subtract the label width:
```cpp
// CORRECT - BOTH SLIDERS AND LABELS FIT PERFECTLY:
const float avail_w = ImGui::GetContentRegionAvail().x;
const float col_w = (avail_w - 12.0f) * 0.5f;
const float lbl_x_w = ImGui::CalcTextSize("X Points").x;
const float lbl_y_w = ImGui::CalcTextSize("Y Points").x;
const float slider_x_w = std::max(45.0f, col_w - lbl_x_w - 12.0f);
const float slider_y_w = std::max(45.0f, col_w - lbl_y_w - 12.0f);

ImGui::SetNextItemWidth(slider_x_w);
ImGui::SliderInt("X Points##Srf", &surface_points_x_, 2, 30);

ImGui::SameLine(col_w + 12.0f);
ImGui::SetNextItemWidth(slider_y_w);
ImGui::SliderInt("Y Points##Srf", &surface_points_y_, 2, 30);
```

---

## 4. Slider Format Strings & Rounding Traps

### The Problem
`ImGui::SliderFloat` parses the format string to deduce decimal rounding precision via `RoundScalarWithFormatT`. If you pass `SliderFloat(0.0f, 1.0f)` with a format string lacking decimal digits (e.g. `"%d%%"` or dynamic text without `.Xf`), ImGui sets `decimal_precision = 0` (integer rounding). Any float value $< 0.5$ rounds to `0.0f` on every drag tick, locking the slider at 0.

### Mandatory Rules
- **For Percentages (0% to 100%):** Always use integer sliders `SliderInt("##ID", &pct, 0, 100, "%d%%")`, and divide by `100.0f` for underlying math.
- **For Floats:** If using `SliderFloat`, always provide an explicit decimal precision specifier (e.g. `"%.2f"`, `"%.3f mm"`).
- Never place dynamic string labels inside the format string of a slider. Keep the slider format clean (e.g. `"%d%%"` or `"%.1f mm"`), and render descriptive captions on a separate line or adjacent text widget.

---

## 5. Workflow Continuity & Verification

- **Always update `WORKFLOW_STATUS.md`:** On every single turn/step, append a completed step section with detailed bullet points and file links so subsequent sessions can pick up seamlessly. This is a local session log and is git-ignored: create it if it does not exist, and use repository-relative links (no absolute paths, user names or hardware serial numbers).
- **User-visible changes go in `CHANGELOG.md`** under the next version. The version number lives only in `project(VERSION ...)` in `CMakeLists.txt`.
- **Never add to the repository:** vendor binaries (DLLs, installers), decompiled vendor code, personal machine profiles or settings files.
- **Always build and run test suites:** Verify with `cmake --build build --config Release` and `script_engine_test.exe`. All unit tests must pass.
