#include "ui_common/fake_board_mode.h"
#include "core/board/board_registry.h"
#include "core/board/grbl/fake_grbl.h"
#include "core/controller/machine_controller.h"
#include "core/transport/byte_stream.h"
#include <memory>

namespace rnr::ui {

std::string fakeBoardSettingsFile(const std::string& normal_file) {
    const size_t dot = normal_file.rfind('.');
    if (dot == std::string::npos) return normal_file + "_fakeboard";
    return normal_file.substr(0, dot) + "_fakeboard" + normal_file.substr(dot);
}

void enterFakeBoardMode(config::PersistentSettings& settings, bool mill) {
    core::MachineController::setHardwareAccessEnabled(false);

    struct Emulated {
        const char* board_id;
        const char* transport;
        test::FakeGrbl::Flavour flavour;
    };
    static const Emulated kBoards[] = {
        {board::kGrblBoardId,    "fake-grbl",    test::FakeGrbl::Flavour::Grbl11},
        {board::kGrblHalBoardId, "fake-grblhal", test::FakeGrbl::Flavour::GrblHal},
        {board::kFluidNcBoardId, "fake-fluidnc", test::FakeGrbl::Flavour::FluidNC},
    };
    // One emulated board per flavour, alive for the whole run (reconnecting keeps its position, like a powered board)
    static std::shared_ptr<test::FakeGrbl> boards[3];
    for (size_t i = 0; i < 3; ++i) {
        if (!boards[i]) {
            boards[i] = std::make_shared<test::FakeGrbl>(kBoards[i].flavour);
            boards[i]->time_scale = 1.0;       // Real time
            boards[i]->probe_contact_z = -5.0; // A probe touches 5 mm below machine Z0
            if (mill && kBoards[i].flavour != test::FakeGrbl::Flavour::Grbl11) {
                boards[i]->useAxes(4); // BreakoutMill: grblHAL and FluidNC with a 4th axis (A)
            }
            if (kBoards[i].flavour == test::FakeGrbl::Flavour::GrblHal && !mill) {
                // BreakoutCut: the grblHAL emulator has the plasma plugin (board-side Arc OK and height control)
                boards[i]->plasma_plugin = true;
                boards[i]->thc_z_drift_mm = 0.4;
            }
        }
        auto fake = boards[i];
        transport::registerStreamFactory(kBoards[i].transport,
                                         [fake] { return std::make_unique<test::FakeGrblStream>(fake); });
        auto& profile = settings.machine.board_profiles[kBoards[i].board_id];
        board::completeProfile(kBoards[i].board_id, profile);
        profile.set("transport", kBoards[i].transport);
        profile.set("address", "emulator");
        profile.setInt("connect_wait_ms", 300);
    }
    if (settings.machine.board_id == board::kRnrBoardId) settings.machine.board_id = board::kGrblBoardId;
    settings.machine.vfd.enabled = false;  // Never open the VFD's serial port in this mode
    settings.ui.simulator_mode = false;
}

} // namespace rnr::ui
