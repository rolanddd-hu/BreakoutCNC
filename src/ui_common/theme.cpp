#include "ui_common/theme.h"
#include <algorithm>
#include <cmath>

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

Mode s_mode = Mode::Dark;

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

float channel(float c) { return std::clamp(c, 0.0f, 1.0f); }

} // namespace

namespace detail {
const Palette* g_active = &kDark;
}

Mode mode() noexcept { return s_mode; }

void setMode(Mode m) {
    s_mode = m;
    detail::g_active = (m == Mode::Light) ? &kLight : &kDark;
    applyStyle(ImGui::GetStyle(), m);
}

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
