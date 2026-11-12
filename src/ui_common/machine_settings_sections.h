#pragma once
// Settings sections BreakoutMill and BreakoutCut share: Limits & Homing, Motors
#include "core/board/board_capabilities.h"
#include "core/config/settings_manager.h"
#include "core/controller/motion_controller.h"
#include <functional>

namespace rnr::ui {

class Mach3MigrationDialog;

struct SettingsSectionContext {
    core::IMotionController&          controller;
    const board::BoardCapabilities&   board_caps;
    config::AppUiSettings&            ui_settings;  // Remembered choices (speed unit)
    std::function<void()>             mark_dirty;   // Save the settings file soon
    Mach3MigrationDialog*             mach3_wizard; // "Import from Mach3" button
    int                               axis_count;   // 4 (BreakoutMill: X Y Z A) or 3 (BreakoutCut)
};

// The contents of the open section (the caller draws the collapsing header)
void renderLimitsHomingSection(const SettingsSectionContext& ctx);
void renderMotorsSection(const SettingsSectionContext& ctx);

} // namespace rnr::ui
