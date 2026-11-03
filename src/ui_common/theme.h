#pragma once
#include <imgui.h>
#include <cstdint>

// Dark / light color themes. Colors are looked up by role from the active Palette: all palettes are built once at
// startup and pal() is a single pointer read, so a lookup per widget costs the same as the literal it replaces.
// Switching the theme swaps the pointer and re-applies the ImGui style colors; nothing is recomputed per frame.
namespace rnr::ui::theme {

enum class Mode : uint8_t { Dark = 0, Light = 1 };

struct Palette {
    // Window frame
    ImVec4 clear_bg;          // Behind all windows (GL clear color)
    ImU32  title_bg;
    ImU32  title_separator;
    ImU32  title_text;
    ImU32  title_icon;        // Caption button icons (fullscreen, minimize, ...)
    ImU32  title_icon_hover;
    ImU32  title_btn_hover;   // Caption button background while hovered / pressed
    ImU32  title_btn_active;

    // Segmented toggles in the title bar ([PLASMA | LASER], [FULL | MINIMAL]); the selected segment keeps its
    // saturated fill with white text in both themes
    ImU32  segment_bg;
    ImU32  segment_border;
    ImU32  segment_hover;
    ImU32  segment_text;
    ImU32  segment_text_hover;

    // Panels
    ImVec4 card_bg;           // Grouped settings cards (ChildBg / Border pushed around a BeginChild)
    ImVec4 card_border;
    ImVec4 heading;           // Section headings ("SPINDLE CONTROL", ...)

    // Script console
    ImVec4 console_bg;
    ImVec4 console_time;
    ImVec4 console_ok;
    ImVec4 console_error;

    // DRO: the readout screen stays dark in both themes; the text next to it sits on the panel background
    ImVec4 dro_axis[4];       // X, Y, Z, A axis letters
    ImVec4 dro_digits;        // Position digits
    ImVec4 dro_secondary;     // MPos / Vel text beside the readout
    ImVec4 dro_homed;         // "[H]" velocity text of a homed axis

    // Macro buttons without a custom color (control panel row)
    ImVec4 macro_btn;

    // Status text
    ImVec4 text_ok;           // Connected, homed, passed
    ImVec4 text_warn;         // Overrides, simulator, cautions
    ImVec4 text_danger;       // Alarms, errors, offline
    ImVec4 text_info;         // Values and highlighted information
    ImVec4 text_muted;        // Secondary labels
    ImVec4 text_on_fill;      // Text pushed explicitly on top of a filled button

    // Filled buttons by meaning (normal / hovered / pressed). The dark theme uses saturated fills, the light theme
    // pale fills, so the default text color stays readable on them in both themes.
    ImVec4 btn_primary, btn_primary_hov, btn_primary_act;    // Main actions, selected toggles
    ImVec4 btn_ok, btn_ok_hov, btn_ok_act;                   // Start, apply, confirm
    ImVec4 btn_danger, btn_danger_hov, btn_danger_act;       // Stop, delete, E-stop
    ImVec4 btn_warn, btn_warn_hov, btn_warn_act;             // Feed hold, reset, caution
    ImVec4 btn_neutral, btn_neutral_hov, btn_neutral_act;    // Secondary actions, unselected toggles
    ImVec4 btn_warn_subtle, btn_warn_subtle_hov, btn_warn_subtle_act; // Muted caution (override active)
    ImVec4 btn_danger_subtle, btn_danger_subtle_hov, btn_danger_subtle_act;

    // Tinted surfaces (ChildBg / FrameBg / PopupBg) and their borders
    ImVec4 surface_inset;     // Recessed areas inside a panel (consoles, lists, popups)
    ImVec4 surface_ok;
    ImVec4 surface_warn;
    ImVec4 surface_danger;    // Alarm banners
    ImVec4 border_ok;
    ImVec4 border_warn;
    ImVec4 border_danger;
    ImVec4 border_accent;     // Highlighted / active card
    ImVec4 card_active_bg;    // Selected card (active work offset)

    // E-stop / alarm banner (above the program, over the toolpath view) and its RESET E-STOP button
    ImVec4 alarm_bg, alarm_border, alarm_text;
    ImVec4 alarm_btn, alarm_btn_hov, alarm_btn_act, alarm_btn_text;
    ImVec4 modal_dim;         // Behind modal dialogs

    // Instrument readouts: feed / laser power gauges (light gray in the light theme, like the DRO) and the status
    // bar display (readout_bezel*, readout_prefix: dark in both themes)
    ImU32 readout_bg, readout_border, readout_border_ok, readout_border_active;
    ImU32 readout_label, readout_label_active, readout_label_dim;
    ImU32 readout_value, readout_value_active, readout_value_cmd;
    ImU32 readout_laser_border, readout_laser_label, readout_laser_value, readout_laser_value_idle, readout_duty;
    ImU32 readout_bezel, readout_bezel_border, readout_prefix;

    // G-code program list
    ImU32 gc_row_even, gc_row_odd;
    ImU32 gc_row_run, gc_row_current, gc_row_selected;      // Executing line (job running / idle), selected line
    ImU32 gc_accent_run, gc_accent_current, gc_selected_border;
    ImU32 gc_num, gc_num_run, gc_num_current, gc_separator;
    ImU32 gc_text, gc_text_current, gc_comment, gc_comment_current;

    // DRO rows: readout screen behind the axis digits (dro_axis)
    ImU32 dro_bg, dro_border, dro_border_homed;

    // Jog pad (cross and Z rocker with thumbstick knobs)
    float jog_shadow;                                        // Drop shadow strength (1 = dark theme)
    ImU32 jog_body, jog_face, jog_groove, jog_ridge;         // jog_ridge: RGB only, alpha set per edge
    ImU32 jog_bevel_mid, jog_bevel_lit, jog_bevel_shade;     // Rim shading towards / away from the light
    ImU32 jog_arrow, jog_arrow_hover, jog_arrow_shadow, jog_hover_glow;
    ImU32 jog_accent;                                        // Pressed arrow / active knob (RGB, alpha varies)
    ImU32 knob_base, knob_base_hover, knob_base_active;
    ImU32 knob_face, knob_face_hover, knob_face_active;
    ImU32 knob_highlight, knob_rim_shadow, knob_dimple, knob_dimple_shadow, knob_text;
    ImU32 stick_shadow;

    // Card behind the probing pad graphics and the pad itself (the orange touch points keep fixed colors)
    ImU32 probe_card_bg, probe_card_border;
    ImU32 pp_tile, pp_tile_border;                           // Flat tiles around the probed feature
    ImU32 pp_center_tile, pp_pocket_bg, pp_pocket_shadow, pp_text, pp_text_dim;
    ImU32 pp_arrow, pp_arrow_border;                         // pp_arrow_border alpha 0 = no outline
    ImU32 pp_dash_line, pp_blue_border;

    // Small round icon buttons (help "i")
    ImU32 icon_idle, icon_hover, icon_active;

    // Lua syntax highlighting in the scripting guide
    ImVec4 syn_default, syn_keyword, syn_cnc, syn_builtin, syn_mach3, syn_string, syn_number, syn_comment;
};

namespace detail {
extern const Palette* g_active;
}

// Active palette (valid before setMode() is called: starts as the dark palette)
inline const Palette& pal() noexcept { return *detail::g_active; }

Mode mode() noexcept;
inline bool isLight() noexcept { return mode() == Mode::Light; }

// Color themes: the default dark and light themes plus variants inspired by editor color schemes. Each belongs to
// the dark or the light family; the title bar has one button per family that cycles through its themes.
struct ThemeInfo {
    const char* id;   // Stored in the settings
    const char* name; // Shown in the title bar tooltip
    Mode        mode;
};
int              themeCount() noexcept;
const ThemeInfo& themeInfo(int index) noexcept;
int              activeTheme() noexcept;
// Theme of a family used when switching to it (the last one chosen there)
int              familyTheme(Mode m) noexcept;
// Next theme of the family after `index`, wrapping around
int              nextTheme(int index) noexcept;
// Index of the theme with this id in the family, or the family's default theme
int              findTheme(Mode m, const char* id) noexcept;

// Switches the palette and applies the matching ImGui style (needs a current ImGui context)
void setTheme(int index);
// Switches to the family's remembered theme
void setMode(Mode m);

// Readable text color (near-white or near-black) for a filled background
ImVec4 contrastText(const ImVec4& bg) noexcept;

// Hovered / pressed variants of a button color
ImVec4 hoveredOf(const ImVec4& c) noexcept;
ImVec4 activeOf(const ImVec4& c) noexcept;

// Same color with another alpha (0-255)
inline ImU32 withAlpha(ImU32 c, int a) noexcept {
    return (c & ~IM_COL32_A_MASK) | (static_cast<ImU32>(a & 0xFF) << IM_COL32_A_SHIFT);
}

// Linear blend from a (t = 0) to b (t = 1), per channel including alpha
inline ImU32 lerp(ImU32 a, ImU32 b, float t) noexcept {
    auto ch = [&](int shift) {
        const float x = static_cast<float>((a >> shift) & 0xFF), y = static_cast<float>((b >> shift) & 0xFF);
        return static_cast<ImU32>(x + (y - x) * t + 0.5f) << shift;
    };
    return ch(IM_COL32_R_SHIFT) | ch(IM_COL32_G_SHIFT) | ch(IM_COL32_B_SHIFT) | ch(IM_COL32_A_SHIFT);
}

// 0xRRGGBB <-> opaque ImVec4
ImVec4   fromRgb(uint32_t rgb) noexcept;
uint32_t toRgb(const ImVec4& c) noexcept;

// Pushes Button / ButtonHovered / ButtonActive / Text for a filled button (pop with ImGui::PopStyleColor(4))
void pushButtonColors(const ImVec4& bg);

// Widgets drawn on a surface that stays dark in both themes (toolpath view) use the dark theme's widget colors
// while this is in scope. In the dark theme it pushes nothing.
class DarkSurfaceScope {
public:
    DarkSurfaceScope();
    ~DarkSurfaceScope();
    DarkSurfaceScope(const DarkSurfaceScope&) = delete;
    DarkSurfaceScope& operator=(const DarkSurfaceScope&) = delete;

private:
    int pushed_ = 0;
};

} // namespace rnr::ui::theme
