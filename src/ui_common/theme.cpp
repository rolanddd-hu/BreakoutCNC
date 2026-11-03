#include "ui_common/theme.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <initializer_list>

namespace rnr::ui::theme {

namespace {

constexpr ImVec4 rgba(float r, float g, float b, float a = 1.0f) { return ImVec4(r, g, b, a); }

// Same values the dark UI used before the light theme existed
const Palette kDark = [] {
    Palette p{};
    p.clear_bg           = rgba(0.06f, 0.07f, 0.09f);
    p.title_bg           = IM_COL32(17, 19, 24, 255);
    p.title_separator    = IM_COL32(38, 44, 56, 255);
    p.title_text         = IM_COL32(220, 228, 238, 255);
    p.title_icon         = IM_COL32(200, 210, 220, 255);
    p.title_icon_hover   = IM_COL32(255, 255, 255, 255);
    p.title_btn_hover    = IM_COL32(255, 255, 255, 25);
    p.title_btn_active   = IM_COL32(255, 255, 255, 45);

    p.segment_bg         = IM_COL32(18, 22, 28, 255);
    p.segment_border     = IM_COL32(45, 55, 70, 255);
    p.segment_hover      = IM_COL32(36, 44, 56, 255);
    p.segment_text       = IM_COL32(150, 165, 185, 255);
    p.segment_text_hover = IM_COL32(210, 225, 240, 255);

    p.card_bg            = rgba(0.12f, 0.14f, 0.18f, 0.70f);
    p.card_border        = rgba(0.24f, 0.30f, 0.38f, 0.70f);
    p.heading            = rgba(0.64f, 0.70f, 0.78f); // Card titles: muted, color is kept for meaning

    p.console_bg         = rgba(0.06f, 0.08f, 0.11f);
    p.console_time       = rgba(0.50f, 0.55f, 0.60f);
    p.console_ok         = rgba(0.40f, 0.90f, 0.55f);
    p.console_error      = rgba(1.00f, 0.35f, 0.35f);

    p.dro_axis[0]        = rgba(1.00f, 0.38f, 0.38f); // X red-coral
    p.dro_axis[1]        = rgba(0.30f, 0.92f, 0.50f); // Y emerald
    p.dro_axis[2]        = rgba(0.47f, 0.75f, 1.00f); // Z blue
    p.dro_axis[3]        = rgba(0.95f, 0.75f, 0.30f); // A amber
    p.dro_digits         = rgba(0.90f, 0.93f, 0.97f);
    p.dro_secondary      = rgba(0.58f, 0.66f, 0.76f);
    p.dro_homed          = rgba(0.20f, 0.92f, 0.52f);

    p.macro_btn          = rgba(0.14f, 0.38f, 0.58f);

    p.text_ok            = rgba(0.20f, 0.95f, 0.50f);
    p.text_warn          = rgba(1.00f, 0.75f, 0.20f);
    p.text_danger        = rgba(1.00f, 0.35f, 0.35f);
    p.text_info          = rgba(0.47f, 0.75f, 1.00f);
    p.text_muted         = rgba(0.65f, 0.70f, 0.78f);
    p.text_on_fill       = rgba(1.00f, 1.00f, 1.00f);

    p.btn_primary        = rgba(0.12f, 0.45f, 0.72f);
    p.btn_primary_hov    = rgba(0.18f, 0.55f, 0.88f);
    p.btn_primary_act    = rgba(0.08f, 0.35f, 0.58f);
    p.btn_ok             = rgba(0.10f, 0.58f, 0.32f);
    p.btn_ok_hov         = rgba(0.14f, 0.72f, 0.40f);
    p.btn_ok_act         = rgba(0.08f, 0.48f, 0.24f);
    p.btn_danger         = rgba(0.82f, 0.18f, 0.18f);
    p.btn_danger_hov     = rgba(0.95f, 0.28f, 0.28f);
    p.btn_danger_act     = rgba(0.65f, 0.12f, 0.12f);
    p.btn_warn           = rgba(0.85f, 0.48f, 0.10f);
    p.btn_warn_hov       = rgba(0.95f, 0.58f, 0.15f);
    p.btn_warn_act       = rgba(0.68f, 0.38f, 0.08f);
    p.btn_neutral        = rgba(0.20f, 0.22f, 0.27f);
    p.btn_neutral_hov    = rgba(0.28f, 0.30f, 0.36f);
    p.btn_neutral_act    = rgba(0.14f, 0.16f, 0.20f);
    p.btn_warn_subtle    = rgba(0.40f, 0.22f, 0.05f);
    p.btn_warn_subtle_hov = rgba(0.55f, 0.30f, 0.08f);
    p.btn_warn_subtle_act = rgba(0.32f, 0.18f, 0.04f);
    p.btn_danger_subtle  = rgba(0.40f, 0.15f, 0.18f);
    p.btn_danger_subtle_hov = rgba(0.55f, 0.20f, 0.24f);
    p.btn_danger_subtle_act = rgba(0.32f, 0.12f, 0.14f);

    p.surface_inset      = rgba(0.08f, 0.10f, 0.13f, 0.98f);
    p.surface_ok         = rgba(0.06f, 0.22f, 0.12f, 0.95f);
    p.surface_warn       = rgba(0.30f, 0.17f, 0.05f, 0.95f);
    p.surface_danger     = rgba(0.42f, 0.08f, 0.08f, 0.95f);
    p.border_ok          = rgba(0.00f, 0.95f, 0.55f, 0.95f);
    p.border_warn        = rgba(0.93f, 0.60f, 0.15f, 1.00f);
    p.border_danger      = rgba(0.85f, 0.25f, 0.20f, 1.00f);
    p.border_accent      = rgba(0.31f, 0.61f, 0.94f, 0.85f);
    p.card_active_bg     = rgba(0.07f, 0.16f, 0.22f, 0.90f);
    p.alarm_bg           = p.surface_danger;
    p.alarm_border       = p.border_danger;
    p.alarm_text         = p.text_warn;
    p.alarm_btn          = p.btn_warn;
    p.alarm_btn_hov      = p.btn_warn_hov;
    p.alarm_btn_act      = p.btn_warn_act;
    p.alarm_btn_text     = p.text_on_fill;
    p.modal_dim          = rgba(0.00f, 0.00f, 0.00f, 0.70f);

    p.readout_bg               = IM_COL32(8, 11, 15, 255);
    p.readout_border           = IM_COL32(45, 58, 74, 220);
    p.readout_border_ok        = IM_COL32(32, 165, 95, 220);
    p.readout_border_active    = IM_COL32(0, 200, 120, 220);
    p.readout_label            = IM_COL32(110, 135, 160, 230);
    p.readout_label_active     = IM_COL32(140, 245, 180, 255);
    p.readout_label_dim        = IM_COL32(100, 120, 145, 200);
    p.readout_value            = IM_COL32(115, 155, 195, 230);
    p.readout_value_active     = IM_COL32(0, 240, 150, 255);
    p.readout_value_cmd        = IM_COL32(120, 192, 255, 220);
    p.readout_laser_border     = IM_COL32(240, 40, 80, 240);
    p.readout_laser_label      = IM_COL32(255, 120, 150, 255);
    p.readout_laser_value      = IM_COL32(255, 40, 80, 255);
    p.readout_laser_value_idle = IM_COL32(140, 165, 195, 230);
    p.readout_duty             = IM_COL32(240, 180, 50, 230);
    p.readout_bezel            = IM_COL32(11, 15, 22, 255);
    p.readout_bezel_border     = IM_COL32(36, 46, 62, 255);
    p.readout_prefix           = IM_COL32(130, 152, 178, 230);

    p.gc_row_even        = IM_COL32(20, 24, 30, 255);
    p.gc_row_odd         = IM_COL32(25, 30, 37, 255);
    p.gc_row_run         = IM_COL32(88, 62, 10, 240);
    p.gc_row_current     = IM_COL32(24, 50, 84, 230);
    p.gc_row_selected    = IM_COL32(36, 54, 78, 240);
    p.gc_accent_run      = IM_COL32(255, 195, 0, 255);
    p.gc_accent_current  = IM_COL32(90, 170, 255, 255);
    p.gc_selected_border = IM_COL32(85, 145, 220, 160);
    p.gc_num             = IM_COL32(100, 140, 190, 255);
    p.gc_num_run         = IM_COL32(255, 215, 60, 255);
    p.gc_num_current     = IM_COL32(110, 180, 255, 255);
    p.gc_separator       = IM_COL32(42, 50, 62, 180);
    p.gc_text            = IM_COL32(230, 238, 248, 255);
    p.gc_text_current    = IM_COL32(255, 245, 190, 255);
    p.gc_comment         = IM_COL32(115, 138, 126, 240);
    p.gc_comment_current = IM_COL32(205, 190, 120, 240);

    p.probe_card_bg      = IM_COL32(18, 20, 25, 255);

    p.dro_bg             = IM_COL32(8, 11, 15, 255);
    p.dro_border         = IM_COL32(45, 58, 74, 220);
    p.dro_border_homed   = IM_COL32(32, 165, 95, 220);

    p.jog_shadow         = 1.0f;
    p.jog_body           = IM_COL32(24, 27, 34, 255);
    p.jog_face           = IM_COL32(48, 52, 60, 255);
    p.jog_groove         = IM_COL32(14, 16, 20, 255);
    p.jog_ridge          = IM_COL32(140, 155, 175, 255);
    p.jog_bevel_mid      = IM_COL32(46, 50, 60, 255);
    p.jog_bevel_lit      = IM_COL32(96, 105, 125, 255);
    p.jog_bevel_shade    = IM_COL32(22, 24, 28, 255);
    p.jog_arrow          = IM_COL32(215, 222, 232, 255);
    p.jog_arrow_hover    = IM_COL32(255, 255, 255, 255);
    p.jog_arrow_shadow   = IM_COL32(10, 12, 16, 230);
    p.jog_hover_glow     = IM_COL32(255, 255, 255, 22);
    p.jog_accent         = IM_COL32(120, 192, 255, 255);
    p.knob_base          = IM_COL32(22, 25, 31, 255);
    p.knob_base_hover    = IM_COL32(36, 40, 48, 255);
    p.knob_base_active   = IM_COL32(28, 42, 58, 255);
    p.knob_face          = IM_COL32(34, 38, 46, 255);
    p.knob_face_hover    = IM_COL32(46, 52, 62, 255);
    p.knob_face_active   = IM_COL32(34, 52, 72, 255);
    p.knob_highlight     = IM_COL32(145, 160, 180, 230);
    p.knob_rim_shadow    = IM_COL32(10, 12, 16, 240);
    p.knob_dimple        = IM_COL32(24, 27, 33, 255);
    p.knob_dimple_shadow = IM_COL32(12, 14, 18, 200);
    p.knob_text          = IM_COL32(255, 255, 255, 245);
    p.stick_shadow       = IM_COL32(10, 12, 16, 200);

    p.pp_tile            = IM_COL32(118, 126, 138, 255);
    p.pp_tile_border     = IM_COL32(40, 44, 52, 255);
    p.pp_center_tile     = IM_COL32(32, 35, 42, 255);
    p.pp_pocket_bg       = IM_COL32(22, 25, 31, 255);
    p.pp_pocket_shadow   = IM_COL32(12, 14, 18, 240);
    p.pp_text            = IM_COL32(230, 238, 250, 250);
    p.pp_text_dim        = IM_COL32(190, 205, 225, 230);
    p.pp_arrow           = IM_COL32(245, 248, 255, 255);
    p.pp_arrow_border    = IM_COL32(0, 0, 0, 0);
    p.pp_dash_line       = IM_COL32(95, 145, 220, 210);
    p.pp_blue_border     = IM_COL32(42, 102, 162, 210);
    p.probe_card_border  = IM_COL32(38, 44, 54, 255);

    p.icon_idle          = IM_COL32(235, 240, 250, 235);
    p.icon_hover         = IM_COL32(120, 192, 255, 255);
    p.icon_active        = IM_COL32(80, 150, 235, 255);

    p.syn_default        = rgba(0.84f, 0.88f, 0.94f);
    p.syn_keyword        = rgba(0.80f, 0.58f, 0.98f);
    p.syn_cnc            = rgba(0.47f, 0.75f, 1.00f);
    p.syn_builtin        = rgba(0.95f, 0.86f, 0.55f);
    p.syn_mach3          = rgba(1.00f, 0.65f, 0.35f);
    p.syn_string         = rgba(0.62f, 0.90f, 0.52f);
    p.syn_number         = rgba(0.98f, 0.72f, 0.48f);
    p.syn_comment        = rgba(0.45f, 0.58f, 0.52f);
    return p;
}();

const Palette kLight = [] {
    Palette p = kDark; // Roles that sit on dark surfaces in both themes (DRO digits) keep the dark values
    p.clear_bg           = rgba(0.86f, 0.88f, 0.91f);
    p.title_bg           = IM_COL32(222, 226, 232, 255);
    p.title_separator    = IM_COL32(184, 191, 202, 255);
    p.title_text         = IM_COL32(30, 36, 46, 255);
    p.title_icon         = IM_COL32(60, 68, 80, 255);
    p.title_icon_hover   = IM_COL32(10, 14, 20, 255);
    p.title_btn_hover    = IM_COL32(0, 0, 0, 22);
    p.title_btn_active   = IM_COL32(0, 0, 0, 40);

    p.segment_bg         = IM_COL32(244, 246, 249, 255);
    p.segment_border     = IM_COL32(176, 184, 196, 255);
    p.segment_hover      = IM_COL32(212, 222, 234, 255);
    p.segment_text       = IM_COL32(84, 94, 110, 255);
    p.segment_text_hover = IM_COL32(20, 26, 36, 255);

    p.card_bg            = rgba(0.94f, 0.95f, 0.96f, 1.00f); // Very light gray: white inputs and sliders stand out
    p.card_border        = rgba(0.74f, 0.78f, 0.84f, 1.00f);
    p.heading            = rgba(0.36f, 0.41f, 0.48f);

    p.console_bg         = rgba(1.00f, 1.00f, 1.00f);
    p.console_time       = rgba(0.45f, 0.48f, 0.54f);
    p.console_ok         = rgba(0.05f, 0.48f, 0.22f);
    p.console_error      = rgba(0.78f, 0.10f, 0.10f);

    p.dro_secondary      = rgba(0.30f, 0.36f, 0.44f);
    p.dro_homed          = rgba(0.05f, 0.52f, 0.26f);

    p.text_ok            = rgba(0.04f, 0.50f, 0.22f);
    p.text_warn          = rgba(0.66f, 0.40f, 0.00f);
    p.text_danger        = rgba(0.78f, 0.12f, 0.12f);
    p.text_info          = rgba(0.10f, 0.40f, 0.75f);
    p.text_muted         = rgba(0.38f, 0.42f, 0.48f);
    p.text_on_fill       = rgba(1.00f, 1.00f, 1.00f); // Solid fills, as in the dark theme

    p.btn_primary        = rgba(0.12f, 0.45f, 0.72f); // White text
    p.btn_primary_hov    = rgba(0.18f, 0.53f, 0.84f);
    p.btn_primary_act    = rgba(0.09f, 0.37f, 0.60f);
    p.btn_ok             = rgba(0.12f, 0.52f, 0.29f); // White text
    p.btn_ok_hov         = rgba(0.15f, 0.60f, 0.34f);
    p.btn_ok_act         = rgba(0.09f, 0.43f, 0.23f);
    p.btn_danger         = rgba(0.78f, 0.16f, 0.16f); // White text
    p.btn_danger_hov     = rgba(0.87f, 0.23f, 0.23f);
    p.btn_danger_act     = rgba(0.65f, 0.12f, 0.12f);
    p.btn_warn           = rgba(0.95f, 0.65f, 0.12f); // Amber, dark text
    p.btn_warn_hov       = rgba(0.98f, 0.71f, 0.22f);
    p.btn_warn_act       = rgba(0.86f, 0.56f, 0.07f);
    p.btn_neutral        = rgba(0.81f, 0.84f, 0.89f);
    p.btn_neutral_hov    = rgba(0.74f, 0.78f, 0.84f);
    p.btn_neutral_act    = rgba(0.67f, 0.71f, 0.78f);
    p.btn_warn_subtle    = rgba(0.99f, 0.85f, 0.64f);
    p.btn_warn_subtle_hov = rgba(0.98f, 0.79f, 0.53f);
    p.btn_warn_subtle_act = rgba(0.96f, 0.73f, 0.43f);
    p.btn_danger_subtle  = rgba(0.97f, 0.76f, 0.76f);
    p.btn_danger_subtle_hov = rgba(0.95f, 0.68f, 0.68f);
    p.btn_danger_subtle_act = rgba(0.92f, 0.60f, 0.60f);

    p.surface_inset      = rgba(0.94f, 0.95f, 0.96f, 1.00f);
    p.surface_ok         = rgba(0.78f, 0.93f, 0.82f, 1.00f);
    p.surface_warn       = rgba(1.00f, 0.87f, 0.66f, 1.00f);
    p.surface_danger     = rgba(0.98f, 0.79f, 0.79f, 1.00f);
    p.border_ok          = rgba(0.10f, 0.60f, 0.30f, 1.00f);
    p.border_warn        = rgba(0.82f, 0.48f, 0.02f, 1.00f);
    p.border_danger      = rgba(0.80f, 0.20f, 0.16f, 1.00f);
    p.border_accent      = rgba(0.12f, 0.45f, 0.80f, 1.00f);
    p.card_active_bg     = rgba(0.86f, 0.93f, 0.98f, 1.00f);
    // Solid red bar with white text and a white button: an E-stop must stand out on the light UI
    p.alarm_bg           = rgba(0.80f, 0.13f, 0.13f, 1.00f);
    p.alarm_border       = rgba(0.62f, 0.06f, 0.06f, 1.00f);
    p.alarm_text         = rgba(1.00f, 1.00f, 1.00f, 1.00f);
    p.alarm_btn          = rgba(1.00f, 1.00f, 1.00f, 1.00f);
    p.alarm_btn_hov      = rgba(1.00f, 0.88f, 0.88f, 1.00f);
    p.alarm_btn_act      = rgba(0.98f, 0.76f, 0.76f, 1.00f);
    p.alarm_btn_text     = rgba(0.70f, 0.08f, 0.08f, 1.00f);
    p.modal_dim          = rgba(0.20f, 0.22f, 0.26f, 0.35f);

    p.gc_row_even        = IM_COL32(255, 255, 255, 255);
    p.gc_row_odd         = IM_COL32(245, 247, 250, 255);
    p.gc_row_run         = IM_COL32(255, 236, 180, 255);
    p.gc_row_current     = IM_COL32(205, 236, 250, 255);
    p.gc_row_selected    = IM_COL32(218, 230, 247, 255);
    p.gc_accent_run      = IM_COL32(220, 150, 0, 255);
    p.gc_accent_current  = IM_COL32(40, 110, 200, 255);
    p.gc_selected_border = IM_COL32(90, 140, 210, 170);
    p.gc_num             = IM_COL32(90, 112, 135, 255);
    p.gc_num_run         = IM_COL32(150, 95, 0, 255);
    p.gc_num_current     = IM_COL32(25, 100, 185, 255);
    p.gc_separator       = IM_COL32(205, 211, 220, 255);
    p.gc_text            = IM_COL32(25, 30, 38, 255);
    p.gc_text_current    = IM_COL32(20, 22, 26, 255);
    p.gc_comment         = IM_COL32(95, 120, 105, 255);
    p.gc_comment_current = IM_COL32(110, 92, 30, 255);

    p.probe_card_bg      = IM_COL32(240, 242, 245, 255);

    p.dro_axis[0]        = rgba(0.80f, 0.12f, 0.12f); // Digits on the light gray readout
    p.dro_axis[1]        = rgba(0.04f, 0.52f, 0.22f);
    p.dro_axis[2]        = rgba(0.00f, 0.40f, 0.72f);
    p.dro_axis[3]        = rgba(0.68f, 0.42f, 0.00f);
    p.dro_bg             = IM_COL32(220, 224, 230, 255);
    p.dro_digits         = rgba(0.10f, 0.12f, 0.16f);

    p.readout_bg               = IM_COL32(220, 224, 230, 255); // Feed / laser power gauges, like the DRO
    p.readout_border           = IM_COL32(168, 175, 187, 255);
    p.readout_border_ok        = IM_COL32(30, 150, 85, 255);
    p.readout_border_active    = IM_COL32(20, 150, 90, 255);
    p.readout_label            = IM_COL32(90, 100, 116, 255);
    p.readout_label_active     = IM_COL32(15, 115, 65, 255);
    p.readout_label_dim        = IM_COL32(105, 114, 128, 255);
    p.readout_value            = IM_COL32(55, 70, 92, 255);
    p.readout_value_active     = IM_COL32(10, 120, 65, 255);
    p.readout_value_cmd        = IM_COL32(25, 95, 180, 255);
    p.readout_laser_border     = IM_COL32(200, 30, 70, 255);
    p.readout_laser_label      = IM_COL32(185, 40, 80, 255);
    p.readout_laser_value      = IM_COL32(190, 20, 60, 255);
    p.readout_laser_value_idle = IM_COL32(55, 70, 92, 255);
    p.readout_duty             = IM_COL32(155, 100, 0, 255);
    p.dro_border         = IM_COL32(168, 175, 187, 255);
    p.dro_border_homed   = IM_COL32(30, 150, 85, 255);

    p.jog_shadow         = 0.35f;
    p.jog_body           = IM_COL32(160, 166, 176, 255);
    p.jog_face           = IM_COL32(214, 218, 225, 255);
    p.jog_groove         = IM_COL32(150, 156, 166, 255);
    p.jog_ridge          = IM_COL32(255, 255, 255, 255);
    p.jog_bevel_mid      = IM_COL32(196, 200, 208, 255);
    p.jog_bevel_lit      = IM_COL32(242, 244, 247, 255);
    p.jog_bevel_shade    = IM_COL32(150, 156, 166, 255);
    p.jog_arrow          = IM_COL32(70, 78, 92, 255);
    p.jog_arrow_hover    = IM_COL32(30, 36, 46, 255);
    p.jog_arrow_shadow   = IM_COL32(255, 255, 255, 170);
    p.jog_hover_glow     = IM_COL32(0, 0, 0, 16);
    p.knob_base          = IM_COL32(66, 72, 84, 255);
    p.knob_base_hover    = IM_COL32(80, 87, 100, 255);
    p.knob_base_active   = IM_COL32(36, 70, 104, 255);
    p.knob_face          = IM_COL32(88, 95, 108, 255);
    p.knob_face_hover    = IM_COL32(102, 110, 124, 255);
    p.knob_face_active   = IM_COL32(48, 90, 130, 255);
    p.knob_highlight     = IM_COL32(170, 178, 192, 230);
    p.knob_rim_shadow    = IM_COL32(40, 44, 52, 220);
    p.knob_dimple        = IM_COL32(64, 70, 82, 255);
    p.knob_dimple_shadow = IM_COL32(40, 44, 52, 180);
    p.stick_shadow       = IM_COL32(40, 44, 52, 150);

    p.pp_tile            = IM_COL32(178, 185, 195, 255);
    p.pp_tile_border     = IM_COL32(120, 128, 140, 255);
    p.pp_center_tile     = IM_COL32(208, 213, 221, 255);
    p.pp_pocket_bg       = IM_COL32(214, 218, 226, 255);
    p.pp_pocket_shadow   = IM_COL32(140, 147, 158, 240);
    p.pp_text            = IM_COL32(25, 30, 40, 255);
    p.pp_text_dim        = IM_COL32(55, 62, 75, 240);
    p.pp_arrow           = IM_COL32(250, 251, 253, 255);
    p.pp_arrow_border    = IM_COL32(35, 40, 50, 255);
    p.pp_dash_line       = IM_COL32(60, 110, 185, 230);
    p.pp_blue_border     = IM_COL32(35, 95, 160, 230);
    p.probe_card_border  = IM_COL32(188, 196, 208, 255);

    p.icon_idle          = IM_COL32(70, 78, 92, 235);
    p.icon_hover         = IM_COL32(30, 115, 205, 255);
    p.icon_active        = IM_COL32(25, 95, 180, 255);

    p.syn_default        = rgba(0.15f, 0.17f, 0.20f);
    p.syn_keyword        = rgba(0.50f, 0.20f, 0.75f);
    p.syn_cnc            = rgba(0.08f, 0.40f, 0.75f);
    p.syn_builtin        = rgba(0.55f, 0.42f, 0.00f);
    p.syn_mach3          = rgba(0.75f, 0.38f, 0.05f);
    p.syn_string         = rgba(0.15f, 0.50f, 0.10f);
    p.syn_number         = rgba(0.70f, 0.35f, 0.05f);
    p.syn_comment        = rgba(0.40f, 0.48f, 0.44f);
    return p;
}();

// Variant themes: a few key colors (0xRRGGBB) from which the palette and the ImGui style are derived. Status
// colors (ok / warn / danger, E-stop) and the DRO readout keep the values of the dark or light base theme.
struct ThemeSpec {
    uint32_t window, panel, frame, border, title; // Backgrounds from outermost to innermost, borders, title bar
    uint32_t text, text_dim, heading;
    uint32_t accent;                              // Headers, highlighted borders, current line
    uint32_t grab;                                // Check marks and slider handles
    uint32_t primary;                             // Primary / selected button fill
    // Lua syntax highlighting: default, keyword, cnc.*, built-ins, Mach3 calls, strings, numbers, comments
    uint32_t syn[8];
};

constexpr ImVec4 hex(uint32_t rgb, float a = 1.0f) {
    return ImVec4(((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f, a);
}

// Inspired by Sublime Text's Monokai: warm charcoal, purple selection
constexpr ThemeSpec kMonokai = {
    0x1E1F1C, 0x272822, 0x34352D, 0x49483E, 0x191A17,
    0xF8F8F2, 0xA59F85, 0xA59F85,
    0xAE81FF, 0xAE81FF, 0x7A56C8,
    {0xF8F8F2, 0xF92672, 0x66D9EF, 0xA6E22E, 0xFD971F, 0xE6DB74, 0xAE81FF, 0x75715E}};
// Inspired by Mariana: slate blue-gray
constexpr ThemeSpec kMariana = {
    0x283038, 0x303841, 0x3B4550, 0x4F5B66, 0x232A31,
    0xD8DEE9, 0xA6ACB9, 0xA6ACB9,
    0x6699CC, 0x6699CC, 0x4F7FB8,
    {0xD8DEE9, 0xC695C6, 0x6699CC, 0x5FB4B4, 0xF99157, 0x99C794, 0xF9AE58, 0xA6ACB9}};
// Inspired by Solarized (dark)
constexpr ThemeSpec kSolarizedDark = {
    0x002B36, 0x073642, 0x0D3F4C, 0x1F5361, 0x00222B,
    0xA9B7B7, 0x7A8F95, 0x839496,
    0x268BD2, 0x268BD2, 0x1F78B8,
    {0x93A1A1, 0x859900, 0x268BD2, 0xB58900, 0xCB4B16, 0x2AA198, 0xD33682, 0x586E75}};
// Inspired by Ayu Light: near-white, blue accents, orange controls
constexpr ThemeSpec kAyuLight = {
    0xF3F4F5, 0xFCFCFC, 0xFFFFFF, 0xD9DCE0, 0xECEEF0,
    0x4A5056, 0x828A92, 0x828A92,
    0x399EE6, 0xFF9940, 0x399EE6,
    {0x5C6166, 0xFA8D3E, 0x399EE6, 0xD9952B, 0xF07171, 0x86B300, 0xA37ACC, 0x9DA2A6}};
// Inspired by Solarized (light)
constexpr ThemeSpec kSolarizedLight = {
    0xEEE8D5, 0xFDF6E3, 0xFFFBF0, 0xD3CBB7, 0xE4DDC8,
    0x485A62, 0x7B8C91, 0x7B8C91,
    0x268BD2, 0x268BD2, 0x268BD2,
    {0x586E75, 0x859900, 0x268BD2, 0xB58900, 0xCB4B16, 0x2AA198, 0xD33682, 0x93A1A1}};

struct ThemeEntry {
    ThemeInfo        info;
    const ThemeSpec* spec; // nullptr: the base dark / light theme as is
};

constexpr ThemeEntry kThemes[] = {
    {{"dark", "Dark", Mode::Dark}, nullptr},
    {{"monokai", "Monokai", Mode::Dark}, &kMonokai},
    {{"mariana", "Mariana", Mode::Dark}, &kMariana},
    {{"solarized_dark", "Solarized Dark", Mode::Dark}, &kSolarizedDark},
    {{"light", "Light", Mode::Light}, nullptr},
    {{"ayu_light", "Ayu Light", Mode::Light}, &kAyuLight},
    {{"solarized_light", "Solarized Light", Mode::Light}, &kSolarizedLight},
};
constexpr int kThemeCount = static_cast<int>(sizeof(kThemes) / sizeof(kThemes[0]));

float channel(float c) { return std::clamp(c, 0.0f, 1.0f); }

ImVec4 mix(const ImVec4& a, const ImVec4& b, float t) {
    return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}

ImVec4 toVec(ImU32 c) {
    return ImVec4(((c >> IM_COL32_R_SHIFT) & 0xFF) / 255.0f, ((c >> IM_COL32_G_SHIFT) & 0xFF) / 255.0f,
                  ((c >> IM_COL32_B_SHIFT) & 0xFF) / 255.0f, ((c >> IM_COL32_A_SHIFT) & 0xFF) / 255.0f);
}

ImU32 toU32(const ImVec4& c) { return ImGui::ColorConvertFloat4ToU32(c); }

ImVec4 withA(const ImVec4& c, float a) { return ImVec4(c.x, c.y, c.z, a); }

float luminance(const ImVec4& c) { return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z; }

// Gives a neutral color the hue of `hue` while keeping its brightness (k = how much)
ImVec4 colorize(const ImVec4& c, const ImVec4& hue, float k) {
    const float scale = luminance(c) / std::max(0.02f, luminance(hue));
    const ImVec4 tinted(channel(hue.x * scale), channel(hue.y * scale), channel(hue.z * scale), c.w);
    return mix(c, tinted, k);
}

void colorizeAll(std::initializer_list<ImU32*> colors, const ImVec4& hue, float k) {
    for (ImU32* c : colors) *c = toU32(colorize(toVec(*c), hue, k));
}

constexpr ImVec4 kBlack = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);

Palette buildPalette(const ThemeEntry& e) {
    const bool light = e.info.mode == Mode::Light;
    Palette p = light ? kLight : kDark;
    if (!e.spec) return p;
    const ThemeSpec& t = *e.spec;
    const ImVec4 window = hex(t.window), panel = hex(t.panel), frame = hex(t.frame), border = hex(t.border);
    const ImVec4 text = hex(t.text), text_dim = hex(t.text_dim), accent = hex(t.accent), primary = hex(t.primary);

    // Drawn surfaces (jog pad, probing pad, gauge bezels) take the theme's hue
    colorizeAll({&p.segment_bg, &p.segment_border, &p.segment_hover, &p.gc_row_selected,
                 &p.probe_card_bg, &p.probe_card_border, &p.jog_body, &p.jog_face, &p.jog_groove, &p.jog_bevel_mid,
                 &p.jog_bevel_lit, &p.jog_bevel_shade, &p.knob_base, &p.knob_base_hover, &p.knob_face,
                 &p.knob_face_hover, &p.knob_dimple, &p.knob_dimple_shadow, &p.knob_rim_shadow, &p.stick_shadow,
                 &p.pp_tile, &p.pp_tile_border, &p.pp_center_tile, &p.pp_pocket_bg, &p.pp_pocket_shadow,
                 &p.readout_bezel, &p.readout_bezel_border},
                panel, 0.75f);

    p.clear_bg           = mix(window, kBlack, light ? 0.05f : 0.25f);
    p.title_bg           = toU32(hex(t.title));
    p.title_separator    = toU32(border);
    p.title_text         = toU32(text);
    p.title_icon         = toU32(mix(text, hex(t.title), 0.15f));
    p.title_icon_hover   = toU32(text);
    p.segment_text       = toU32(text_dim);
    p.segment_text_hover = toU32(text);

    p.card_bg            = panel;
    p.card_border        = border;
    p.heading            = hex(t.heading);
    p.console_bg         = light ? frame : mix(window, kBlack, 0.2f);
    p.console_time       = text_dim;
    p.dro_secondary      = text_dim;
    p.text_muted         = text_dim;
    p.text_info          = light ? mix(accent, kBlack, 0.25f) : accent;
    p.macro_btn          = primary;

    p.btn_primary        = primary;
    p.btn_primary_hov    = hoveredOf(primary);
    p.btn_primary_act    = activeOf(primary);
    p.btn_neutral        = mix(frame, text, light ? 0.12f : 0.10f);
    p.btn_neutral_hov    = mix(frame, text, light ? 0.20f : 0.18f);
    p.btn_neutral_act    = mix(frame, text, light ? 0.28f : 0.04f);
    p.surface_inset      = withA(light ? frame : mix(window, kBlack, 0.1f), 0.98f);
    p.border_accent      = withA(accent, light ? 1.0f : 0.85f);
    p.card_active_bg     = mix(panel, accent, 0.16f);

    p.gc_row_even        = toU32(light ? frame : mix(frame, window, 0.6f));
    p.gc_row_odd         = toU32(light ? mix(frame, panel, 0.7f) : mix(frame, window, 0.3f));
    p.gc_separator       = toU32(withA(border, 0.8f));
    p.gc_text            = toU32(text);
    p.gc_num             = toU32(text_dim);
    p.gc_accent_current  = toU32(accent);
    p.gc_num_current     = toU32(light ? mix(accent, kBlack, 0.2f) : accent);
    p.gc_selected_border = toU32(withA(accent, 0.65f));
    p.gc_row_current     = toU32(mix(light ? frame : panel, accent, light ? 0.22f : 0.28f));

    p.jog_accent         = toU32(accent);
    p.icon_hover         = toU32(accent);
    p.icon_active        = toU32(activeOf(accent));
    p.pp_dash_line       = toU32(withA(accent, 0.85f));

    ImVec4* syn[] = {&p.syn_default, &p.syn_keyword, &p.syn_cnc, &p.syn_builtin,
                     &p.syn_mach3, &p.syn_string, &p.syn_number, &p.syn_comment};
    for (int i = 0; i < 8; ++i) *syn[i] = hex(t.syn[i]);
    return p;
}

// Palettes of all themes, built on first use (after kDark / kLight above)
const Palette& themePalette(int index) {
    static const auto palettes = [] {
        std::array<Palette, kThemeCount> all{};
        for (int i = 0; i < kThemeCount; ++i) all[i] = buildPalette(kThemes[i]);
        return all;
    }();
    return palettes[index];
}

Mode s_mode = Mode::Dark;
int  s_theme = 0;
int  s_family_theme[2] = {0, 4}; // Last dark / light theme

void applyStyle(ImGuiStyle& style, Mode m) {
    style.WindowRounding    = 6.0f;
    style.ChildRounding     = 6.0f;
    style.FrameRounding     = 5.0f;
    style.GrabRounding      = 4.0f;
    style.PopupRounding     = 5.0f;
    style.ScrollbarRounding = 6.0f;
    style.FramePadding      = ImVec2(10.0f, 6.0f);
    style.ItemSpacing       = ImVec2(8.0f, 7.0f);
    style.WindowBorderSize  = 1.0f;
    style.ChildBorderSize   = 1.0f;

    ImVec4* colors = style.Colors;
    if (m == Mode::Dark) {
        ImGui::StyleColorsDark(&style);
        colors[ImGuiCol_WindowBg]             = rgba(0.09f, 0.10f, 0.13f);
        colors[ImGuiCol_ChildBg]              = rgba(0.11f, 0.13f, 0.16f);
        colors[ImGuiCol_PopupBg]              = rgba(0.10f, 0.12f, 0.15f, 0.98f);
        colors[ImGuiCol_Border]               = rgba(0.22f, 0.25f, 0.31f);
        colors[ImGuiCol_FrameBg]              = rgba(0.15f, 0.18f, 0.22f);
        colors[ImGuiCol_FrameBgHovered]       = rgba(0.20f, 0.24f, 0.30f);
        colors[ImGuiCol_FrameBgActive]        = rgba(0.24f, 0.30f, 0.38f);
        colors[ImGuiCol_TitleBg]              = rgba(0.07f, 0.08f, 0.10f);
        colors[ImGuiCol_TitleBgActive]        = rgba(0.10f, 0.13f, 0.18f);
        colors[ImGuiCol_Header]               = rgba(0.18f, 0.24f, 0.32f);
        colors[ImGuiCol_HeaderHovered]        = rgba(0.22f, 0.32f, 0.44f);
        colors[ImGuiCol_HeaderActive]         = rgba(0.26f, 0.38f, 0.52f);
        colors[ImGuiCol_Button]               = rgba(0.17f, 0.22f, 0.29f);
        colors[ImGuiCol_ButtonHovered]        = rgba(0.23f, 0.31f, 0.42f);
        colors[ImGuiCol_ButtonActive]         = rgba(0.14f, 0.46f, 0.70f);
        colors[ImGuiCol_CheckMark]            = rgba(0.47f, 0.75f, 1.00f);
        colors[ImGuiCol_SliderGrab]           = rgba(0.38f, 0.62f, 0.95f);
        colors[ImGuiCol_SliderGrabActive]     = rgba(0.50f, 0.75f, 1.00f);
        colors[ImGuiCol_Tab]                  = rgba(0.12f, 0.15f, 0.20f);
        colors[ImGuiCol_TabHovered]           = rgba(0.22f, 0.32f, 0.45f);
        colors[ImGuiCol_TabSelected]          = rgba(0.16f, 0.26f, 0.38f);
        colors[ImGuiCol_ModalWindowDimBg]     = rgba(0.00f, 0.00f, 0.00f, 0.55f); // Darken behind dialogs
    } else {
        ImGui::StyleColorsLight(&style);
        colors[ImGuiCol_Text]                 = rgba(0.10f, 0.12f, 0.16f);
        colors[ImGuiCol_TextDisabled]         = rgba(0.44f, 0.48f, 0.54f);
        colors[ImGuiCol_WindowBg]             = rgba(0.91f, 0.92f, 0.94f);
        colors[ImGuiCol_ChildBg]              = rgba(0.96f, 0.97f, 0.98f);
        colors[ImGuiCol_PopupBg]              = rgba(0.98f, 0.98f, 0.99f, 0.98f);
        colors[ImGuiCol_Border]               = rgba(0.72f, 0.75f, 0.80f);
        colors[ImGuiCol_FrameBg]              = rgba(1.00f, 1.00f, 1.00f);
        colors[ImGuiCol_FrameBgHovered]       = rgba(0.88f, 0.92f, 0.97f);
        colors[ImGuiCol_FrameBgActive]        = rgba(0.80f, 0.87f, 0.95f);
        colors[ImGuiCol_TitleBg]              = rgba(0.85f, 0.87f, 0.90f);
        colors[ImGuiCol_TitleBgActive]        = rgba(0.80f, 0.84f, 0.90f);
        colors[ImGuiCol_Header]               = rgba(0.80f, 0.86f, 0.93f);
        colors[ImGuiCol_HeaderHovered]        = rgba(0.72f, 0.81f, 0.92f);
        colors[ImGuiCol_HeaderActive]         = rgba(0.64f, 0.76f, 0.90f);
        colors[ImGuiCol_Button]               = rgba(0.82f, 0.86f, 0.91f);
        colors[ImGuiCol_ButtonHovered]        = rgba(0.74f, 0.80f, 0.88f);
        colors[ImGuiCol_ButtonActive]         = rgba(0.62f, 0.72f, 0.85f);
        colors[ImGuiCol_CheckMark]            = rgba(0.12f, 0.45f, 0.80f);
        colors[ImGuiCol_SliderGrab]           = rgba(0.15f, 0.50f, 0.85f);
        colors[ImGuiCol_SliderGrabActive]     = rgba(0.10f, 0.40f, 0.72f);
        colors[ImGuiCol_Separator]            = rgba(0.72f, 0.75f, 0.80f);
        colors[ImGuiCol_Tab]                  = rgba(0.84f, 0.87f, 0.91f);
        colors[ImGuiCol_TabHovered]           = rgba(0.72f, 0.80f, 0.90f);
        colors[ImGuiCol_TabSelected]          = rgba(0.96f, 0.97f, 0.98f);
        colors[ImGuiCol_ScrollbarBg]          = rgba(0.88f, 0.89f, 0.91f);
        colors[ImGuiCol_ModalWindowDimBg]     = rgba(0.20f, 0.22f, 0.26f, 0.35f);
    }
}

// ImGui style colors of a variant theme, on top of its base dark / light style
void applySpecStyle(ImGuiStyle& style, const ThemeSpec& t, Mode m) {
    const bool light = m == Mode::Light;
    const ImVec4 window = hex(t.window), panel = hex(t.panel), frame = hex(t.frame), border = hex(t.border);
    const ImVec4 text = hex(t.text), accent = hex(t.accent), primary = hex(t.primary);
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text]                 = text;
    colors[ImGuiCol_TextDisabled]         = hex(t.text_dim);
    colors[ImGuiCol_WindowBg]             = window;
    colors[ImGuiCol_ChildBg]              = panel;
    colors[ImGuiCol_PopupBg]              = withA(panel, 0.98f);
    colors[ImGuiCol_Border]               = border;
    colors[ImGuiCol_FrameBg]              = frame;
    colors[ImGuiCol_FrameBgHovered]       = mix(frame, accent, 0.15f);
    colors[ImGuiCol_FrameBgActive]        = mix(frame, accent, 0.28f);
    colors[ImGuiCol_TitleBg]              = hex(t.title);
    colors[ImGuiCol_TitleBgActive]        = hex(t.title);
    colors[ImGuiCol_Header]               = mix(frame, accent, light ? 0.18f : 0.20f);
    colors[ImGuiCol_HeaderHovered]        = mix(frame, accent, light ? 0.30f : 0.32f);
    colors[ImGuiCol_HeaderActive]         = mix(frame, accent, light ? 0.42f : 0.45f);
    colors[ImGuiCol_Button]               = mix(frame, light ? text : accent, light ? 0.12f : 0.14f);
    colors[ImGuiCol_ButtonHovered]        = mix(frame, light ? text : accent, light ? 0.20f : 0.26f);
    colors[ImGuiCol_ButtonActive]         = primary;
    colors[ImGuiCol_CheckMark]            = hex(t.grab);
    colors[ImGuiCol_SliderGrab]           = hex(t.grab);
    colors[ImGuiCol_SliderGrabActive]     = hoveredOf(hex(t.grab));
    colors[ImGuiCol_Separator]            = border;
    colors[ImGuiCol_SeparatorHovered]     = accent;
    colors[ImGuiCol_SeparatorActive]      = accent;
    colors[ImGuiCol_ResizeGrip]           = withA(accent, 0.25f);
    colors[ImGuiCol_ResizeGripHovered]    = withA(accent, 0.65f);
    colors[ImGuiCol_ResizeGripActive]     = withA(accent, 0.95f);
    colors[ImGuiCol_Tab]                  = mix(panel, window, 0.5f);
    colors[ImGuiCol_TabHovered]           = mix(panel, accent, 0.30f);
    colors[ImGuiCol_TabSelected]          = light ? panel : mix(panel, accent, 0.18f);
    colors[ImGuiCol_ScrollbarBg]          = window;
    colors[ImGuiCol_ScrollbarGrab]        = mix(window, text, 0.22f);
    colors[ImGuiCol_ScrollbarGrabHovered] = mix(window, text, 0.32f);
    colors[ImGuiCol_ScrollbarGrabActive]  = mix(window, text, 0.42f);
    colors[ImGuiCol_TextSelectedBg]       = withA(accent, 0.35f);
}

} // namespace

namespace detail {
const Palette* g_active = &kDark;
}

Mode mode() noexcept { return s_mode; }

int themeCount() noexcept { return kThemeCount; }

const ThemeInfo& themeInfo(int index) noexcept { return kThemes[std::clamp(index, 0, kThemeCount - 1)].info; }

int activeTheme() noexcept { return s_theme; }

int familyTheme(Mode m) noexcept { return s_family_theme[m == Mode::Light ? 1 : 0]; }

int nextTheme(int index) noexcept {
    index = std::clamp(index, 0, kThemeCount - 1);
    const Mode m = kThemes[index].info.mode;
    for (int step = 1; step < kThemeCount; ++step) {
        const int i = (index + step) % kThemeCount;
        if (kThemes[i].info.mode == m) return i;
    }
    return index;
}

int findTheme(Mode m, const char* id) noexcept {
    int fallback = -1;
    for (int i = 0; i < kThemeCount; ++i) {
        if (kThemes[i].info.mode != m) continue;
        if (fallback < 0) fallback = i;
        if (id && std::strcmp(kThemes[i].info.id, id) == 0) return i;
    }
    return fallback;
}

void setTheme(int index) {
    index = std::clamp(index, 0, kThemeCount - 1);
    const ThemeEntry& e = kThemes[index];
    s_theme = index;
    s_mode  = e.info.mode;
    s_family_theme[s_mode == Mode::Light ? 1 : 0] = index;
    detail::g_active = &themePalette(index);
    applyStyle(ImGui::GetStyle(), s_mode);
    if (e.spec) applySpecStyle(ImGui::GetStyle(), *e.spec, s_mode);
}

void setMode(Mode m) { setTheme(familyTheme(m)); }

ImVec4 contrastText(const ImVec4& bg) noexcept {
    // Relative luminance (sRGB weights); the threshold leans to white text on mid-tone fills
    const float lum = 0.2126f * bg.x + 0.7152f * bg.y + 0.0722f * bg.z;
    return lum > 0.55f ? rgba(0.06f, 0.07f, 0.09f) : rgba(1.0f, 1.0f, 1.0f);
}

ImVec4 hoveredOf(const ImVec4& c) noexcept {
    return rgba(channel(c.x + (1.0f - c.x) * 0.15f), channel(c.y + (1.0f - c.y) * 0.15f),
                channel(c.z + (1.0f - c.z) * 0.15f), c.w);
}

ImVec4 activeOf(const ImVec4& c) noexcept {
    return rgba(c.x * 0.78f, c.y * 0.78f, c.z * 0.78f, c.w);
}

ImVec4 fromRgb(uint32_t rgb) noexcept {
    return rgba(((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f);
}

uint32_t toRgb(const ImVec4& c) noexcept {
    auto byte = [](float v) { return static_cast<uint32_t>(std::lround(channel(v) * 255.0f)); };
    return (byte(c.x) << 16) | (byte(c.y) << 8) | byte(c.z);
}

void pushButtonColors(const ImVec4& bg) {
    ImGui::PushStyleColor(ImGuiCol_Button, bg);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hoveredOf(bg));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, activeOf(bg));
    ImGui::PushStyleColor(ImGuiCol_Text, contrastText(bg));
}

DarkSurfaceScope::DarkSurfaceScope() {
    if (s_mode == Mode::Dark) return;
    // Dark style colors, built once on first use
    static const ImGuiStyle kDarkStyle = [] {
        ImGuiStyle st;
        applyStyle(st, Mode::Dark);
        return st;
    }();
    static constexpr ImGuiCol kRoles[] = {
        ImGuiCol_Text, ImGuiCol_TextDisabled, ImGuiCol_ChildBg, ImGuiCol_PopupBg, ImGuiCol_Border,
        ImGuiCol_FrameBg, ImGuiCol_FrameBgHovered, ImGuiCol_FrameBgActive, ImGuiCol_Button, ImGuiCol_ButtonHovered,
        ImGuiCol_ButtonActive, ImGuiCol_CheckMark, ImGuiCol_CheckboxSelectedBg, ImGuiCol_Header,
        ImGuiCol_HeaderHovered, ImGuiCol_HeaderActive, ImGuiCol_SliderGrab, ImGuiCol_SliderGrabActive, ImGuiCol_Separator, ImGuiCol_ScrollbarBg,
        ImGuiCol_ScrollbarGrab};
    for (ImGuiCol c : kRoles) ImGui::PushStyleColor(c, kDarkStyle.Colors[c]);
    pushed_ = static_cast<int>(sizeof(kRoles) / sizeof(kRoles[0]));
}

DarkSurfaceScope::~DarkSurfaceScope() {
    if (pushed_ > 0) ImGui::PopStyleColor(pushed_);
}

} // namespace rnr::ui::theme
