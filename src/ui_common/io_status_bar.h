#pragma once
#include "core/controller/motion_controller.h"
#include "imgui.h"
#include <functional>

namespace rnr::ui {

class GCodeProgramPanel;

class IoStatusBar {
public:
    static void renderTopBar(
        const char* app_title,
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        const std::function<void()>& on_save_settings = nullptr,
        bool* minimal_layout = nullptr); // optional: renders FULL / MINIMAL layout selector tabs in the middle

    // High-visibility card containing State Badge and Emergency Stop / Reset button
    static void renderStatusAndEStopCard(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        float card_height = 150.0f,
        GCodeProgramPanel* gcode_panel = nullptr);

    // Dedicated USB controller connection & hardware diagnostics section
    static void renderUsbConnectionSection(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        const std::function<void()>& on_settings_changed = nullptr);

    static void renderGlobalAlarmBanner(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry);

    static void renderFloatingAlarmBanner(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry,
        ImVec2 screen_pos,
        float width);

    static void renderBottomIoStrip(
        core::IMotionController& controller,
        const core::TelemetrySnapshot& telemetry);
};

} // namespace rnr::ui
