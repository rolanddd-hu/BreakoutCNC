#pragma once
#include "core/controller/machine_controller.h"
#include "core/board/rnr_board_driver.h"

namespace rnr::core {

// The machine core driving an RNR Eco Motion 2.0 USB card (the board the applications use today).
// Other boards: construct a MachineController with their IBoardDriver instead.
class RnRController final : public MachineController {
public:
    RnRController() : MachineController(std::make_unique<board::RnrBoardDriver>()) {}
};

} // namespace rnr::core
