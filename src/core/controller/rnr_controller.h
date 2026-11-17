#pragma once
#include "core/controller/machine_controller.h"
#include "core/board/rnr_board_driver.h"

namespace rnr::core {

// The machine core with the RNR Eco Motion 2.0 as its host-planned board. Firmware-planned boards (GRBL family) are
// selected through MachineConfig::board_id; the applications use this class for every board.
// connect_now = false: the application connects once it has applied its settings (and so knows the board).
class RnRController final : public MachineController {
public:
    explicit RnRController(bool connect_now = true)
        : MachineController(std::make_unique<board::RnrBoardDriver>(), connect_now) {}
};

} // namespace rnr::core
