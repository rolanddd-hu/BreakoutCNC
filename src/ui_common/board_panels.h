#pragma once
#include "core/controller/motion_controller.h"
#include <functional>

// Board selection, connection, console and firmware settings panels (MULTI_BOARD_PLAN.md section 4.16).
// Which panels show follows the board's capabilities, never its name.
namespace rnr::ui {

// Capabilities of the selected board, cached until telemetry.capabilities_revision changes (cheap every frame)
const board::BoardCapabilities& boardCaps(const core::IMotionController& controller,
                                          const core::TelemetrySnapshot& telemetry);

// Board selector and the connection settings of the selected board (serial port / network address, baud rate)
void renderBoardSelector(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry,
                         const std::function<void()>& on_settings_changed);

// Text console of a firmware board: protocol log and a command line
void renderBoardConsole(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry);

// The firmware's own settings ($$, grblHAL enumeration, FluidNC configuration)
void renderFirmwareSettings(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry);

// Short note shown in place of a settings section the selected board does not use
void renderNotUsedByBoardNote(const char* what, const board::BoardCapabilities& caps);

} // namespace rnr::ui
