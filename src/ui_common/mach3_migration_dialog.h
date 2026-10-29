#pragma once
#include "core/controller/motion_controller.h"
#include "core/config/mach3_xml_importer.h"
#include <string>
#include <vector>
#include <array>

namespace rnr::ui {

class Mach3MigrationDialog {
public:
    Mach3MigrationDialog() = default;

    // Open Phase 1: Mach3 XML Profile Import & Audit Wizard
    void openMigrationWizard(const core::MachineConfig& current_cfg);

    // Open Phase 2 directly: Interactive 5-Stage Machine Verification & Safety Test Routine
    void openMachineTestRoutine();

    // Render modal windows (call once per frame from main UI loop)
    // Returns true if MachineConfig was modified and should be saved to disk
    bool render(core::IMotionController& controller, bool is_plasma_app);

private:
    enum class WizardPhase : uint8_t {
        Closed        = 0,
        ImportAudit   = 1, // Phase 1: XML Selection, Audit Table & Safety Review
        MachineTest   = 2, // Phase 2: 5-Stage Post-Migration Machine Test Routine
    };

    WizardPhase phase_        = WizardPhase::Closed;
    bool        open_popup_   = false;

    // Phase 1 State
    std::string                     selected_xml_path_;
    std::vector<std::string>        discovered_profiles_;
    config::Mach3ImportOptions      import_options_{};
    config::Mach3MigrationReport    report_{};
    bool                            has_loaded_report_ = false;
    bool                            show_warnings_only_ = false;
    int                             category_filter_idx_ = 0;

    // Phase 2 (5-Stage Machine Test Routine) State
    int                   test_stage_ = 0; // 0..4 (Stages 1..5)
    std::array<bool, 5>   stage_completed_ = {false, false, false, false, false};
    std::array<bool, 4>   axis_dir_verified_ = {false, false, false, false};
    std::array<bool, 4>   axis_steps_verified_ = {false, false, false, false};
    std::array<bool, 12>  input_latch_seen_{}; // Latches true when IN1..IN12 is triggered during Stage 4

    // Stage 2, 3 & 5 interactive controls
    float                 nudge_dist_mm_      = 2.0f;
    float                 nudge_feed_mm_min_  = 300.0f;
    int                   calib_axis_idx_     = 0;
    float                 calib_cmd_dist_mm_  = 10.0f;
    float                 calib_meas_dist_mm_ = 10.0f;
    int                   test_spindle_rpm_   = 3000;
    std::string           test_status_msg_;

    // Configuration from before the import was applied: Cancel restores it
    core::MachineConfig   backup_config_{};
    bool                  has_backup_ = false;
    bool                  demo_loaded_ = false;
    bool                  demo_is_plasma_ = false;

    void reloadReport(const core::MachineConfig& base_cfg);
    void loadDemoMach3Profile(const core::MachineConfig& base_cfg, bool is_plasma_app);
    void resetTestRoutineState();
    void applyImport(core::IMotionController& controller);
    // Puts the pre-import configuration back. Returns true if there was one to restore.
    bool restoreBackup(core::IMotionController& controller);

    bool renderImportAuditPhase(
        core::IMotionController& controller,
        bool is_plasma_app);

    bool renderMachineTestPhase(
        core::IMotionController& controller,
        bool is_plasma_app);
};

} // namespace rnr::ui
