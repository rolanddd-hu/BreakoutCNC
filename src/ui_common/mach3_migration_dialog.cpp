#include "ui_common/mach3_migration_dialog.h"
#include "ui_common/file_dialog.h"
#include "ui_common/app_window.h"
#include "ui_common/imgui_text_helpers.h"
#include <imgui.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <filesystem>

namespace rnr::ui {

namespace {

bool isPinTriggered(uint16_t inputs_bitmask, uint8_t pin_1_to_12, bool invert) noexcept {
    if (pin_1_to_12 < 1 || pin_1_to_12 > 12) return false;
    bool raw_high = (inputs_bitmask & (1U << (pin_1_to_12 - 1))) != 0;
    return invert ? !raw_high : raw_high;
}

// Width of a button sized to its label (AGENTS.md: text width + 32 px padding)
float buttonWidth(const char* label) {
    return ImGui::CalcTextSize(label, nullptr, true).x + 32.0f;
}

// Sample profiles in the real Mach3 format: a metric router that used the RNR plugin and the Huanyang
// VFD plugin, and a plasma table with a dual-motor X gantry.
constexpr const char* kDemoMillXml = R"(<?xml version="1.0" encoding="UTF-8"?>
<profile><Preferences>
<Profile>Mach3Mill_6040Z_Router</Profile><Units>0</Units><PulseSpeed>6</PulseSpeed>
<Motor0Active>1</Motor0Active><Motor0StepNeg>1</Motor0StepNeg><Motor0DirNeg>0</Motor0DirNeg>
<Motor1Active>1</Motor1Active><Motor1StepNeg>1</Motor1StepNeg><Motor1DirNeg>1</Motor1DirNeg>
<Motor2Active>1</Motor2Active><Motor2StepNeg>1</Motor2StepNeg><Motor2DirNeg>0</Motor2DirNeg>
<Motor3Active>0</Motor3Active>
<Steps0>320.</Steps0><Vel0>75.</Vel0><Acc0>600.</Acc0>
<Steps1>320.</Steps1><Vel1>75.</Vel1><Acc1>600.</Acc1>
<Steps2>400.</Steps2><Vel2>35.</Vel2><Acc2>400.</Acc2>
<M0Rev>0</M0Rev><M0Neg>1</M0Neg><M0Min>-2.</M0Min><M0Max>400.</M0Max><RefSpeed0>20</RefSpeed0>
<M1Rev>1</M1Rev><M1Neg>1</M1Neg><M1Min>-2.</M1Min><M1Max>600.</M1Max><RefSpeed1>20</RefSpeed1>
<M2Rev>0</M2Rev><M2Neg>0</M2Neg><M2Min>-120.</M2Min><M2Max>0.</M2Max><RefSpeed2>15</RefSpeed2>
<SoftLimit>1</SoftLimit><Slave0>3</Slave0><Slave1>3</Slave1><Slave2>3</Slave2>
<Input2Active>1</Input2Active><Input2Port>3</Input2Port><Input2Pin>1</Input2Pin><Input2Neg>1</Input2Neg>
<Input5Active>1</Input5Active><Input5Port>3</Input5Port><Input5Pin>2</Input5Pin><Input5Neg>1</Input5Neg>
<Input8Active>1</Input8Active><Input8Port>3</Input8Port><Input8Pin>3</Input8Pin><Input8Neg>1</Input8Neg>
<Input22Active>1</Input22Active><Input22Port>3</Input22Port><Input22Pin>5</Input22Pin><Input22Neg>1</Input22Neg>
<Input25Active>1</Input25Active><Input25Port>3</Input25Port><Input25Pin>4</Input25Pin><Input25Neg>0</Input25Neg>
<SpinDelayCW>2.5</SpinDelayCW><SpinDownCW>1.5</SpinDownCW><PULLEY>0</PULLEY><SPEED1>24000</SPEED1><LSPEED1>6000</LSPEED1>
<IGNORETOOL>0</IGNORETOOL><AutoTool>0</AutoTool><UseSafeZ>1</UseSafeZ><m_SafeZ>30.</m_SafeZ><DwellinMilli>0</DwellinMilli>
</Preferences>
<RnRMotion><PulseRates>100</PulseRates><XPullOff>3.0</XPullOff><YPullOff>3.0</YPullOff><ZPullOff>2.5</ZPullOff><SpindleRatio>0.95</SpindleRatio></RnRMotion>
<PlugIns><VFD-Driver-by-royaumedeole.fr>1</VFD-Driver-by-royaumedeole.fr></PlugIns>
<VfdDriver><ComPort>4</ComPort><BaudRate>9600</BaudRate><Address>1</Address><PulleyOption>1</PulleyOption><EStopOption>1</EStopOption><Status>62</Status><Margin>3</Margin><BadSpeedThreshold>3</BadSpeedThreshold><BadtrackingThreshold>3</BadtrackingThreshold><EStopCauseOption>1</EStopCauseOption><Status2_dro>1002</Status2_dro><Status2_lim>9.3</Status2_lim></VfdDriver>
</profile>)";

constexpr const char* kDemoPlasmaXml = R"(<?xml version="1.0" encoding="UTF-8"?>
<profile><Preferences>
<Profile>Mach3Plasma_4x8_Gantry</Profile><Units>0</Units><PulseSpeed>6</PulseSpeed><PlasmaMode>1</PlasmaMode>
<Motor0Active>1</Motor0Active><Motor0StepNeg>0</Motor0StepNeg><Motor0DirNeg>0</Motor0DirNeg>
<Motor1Active>1</Motor1Active><Motor1StepNeg>0</Motor1StepNeg><Motor1DirNeg>1</Motor1DirNeg>
<Motor2Active>1</Motor2Active><Motor2StepNeg>0</Motor2StepNeg><Motor2DirNeg>0</Motor2DirNeg>
<Motor3Active>1</Motor3Active><Motor3StepNeg>0</Motor3StepNeg><Motor3DirNeg>1</Motor3DirNeg>
<Steps0>160.</Steps0><Vel0>150.</Vel0><Acc0>950.</Acc0>
<Steps1>160.</Steps1><Vel1>150.</Vel1><Acc1>950.</Acc1>
<Steps2>400.</Steps2><Vel2>40.</Vel2><Acc2>500.</Acc2>
<M0Neg>1</M0Neg><M0Min>0.</M0Min><M0Max>1250.</M0Max><RefSpeed0>20</RefSpeed0>
<M1Neg>1</M1Neg><M1Min>0.</M1Min><M1Max>2500.</M1Max><RefSpeed1>20</RefSpeed1>
<M2Neg>0</M2Neg><M2Min>-150.</M2Min><M2Max>0.</M2Max><RefSpeed2>20</RefSpeed2>
<M3Rev>1</M3Rev><SoftLimit>1</SoftLimit><Slave0>0</Slave0><Slave1>3</Slave1><Slave2>3</Slave2>
<Input22Active>1</Input22Active><Input22Port>3</Input22Port><Input22Pin>1</Input22Pin><Input22Neg>1</Input22Neg>
<Input25Active>1</Input25Active><Input25Port>3</Input25Port><Input25Pin>5</Input25Pin><Input25Neg>0</Input25Neg>
<Input26Active>1</Input26Active><Input26Port>3</Input26Port><Input26Pin>2</Input26Pin><Input26Neg>1</Input26Neg>
<Input27Active>1</Input27Active><Input27Port>3</Input27Port><Input27Pin>3</Input27Pin><Input27Neg>1</Input27Neg>
<Input28Active>1</Input28Active><Input28Port>3</Input28Port><Input28Pin>4</Input28Pin><Input28Neg>1</Input28Neg>
</Preferences>
<RnRMotion><PulseRates>100</PulseRates><XPullOff>4.0</XPullOff><YPullOff>4.0</YPullOff><ZPullOff>3.0</ZPullOff></RnRMotion>
</profile>)";

} // namespace

void Mach3MigrationDialog::openMigrationWizard(const core::MachineConfig& current_cfg) {
    (void)current_cfg;
    discovered_profiles_ = config::Mach3XmlImporter::discoverLocalMach3Profiles("C:/Mach3");
    phase_ = WizardPhase::ImportAudit;
    open_popup_ = true;
}

void Mach3MigrationDialog::openMachineTestRoutine() {
    resetTestRoutineState();
    phase_ = WizardPhase::MachineTest;
    open_popup_ = true;
}

void Mach3MigrationDialog::resetTestRoutineState() {
    test_stage_ = 0;
    stage_completed_    = {false, false, false, false, false};
    axis_dir_verified_  = {false, false, false, false};
    axis_steps_verified_= {false, false, false, false};
    input_latch_seen_.fill(false);
    test_status_msg_.clear();
}

void Mach3MigrationDialog::reloadReport(const core::MachineConfig& base_cfg) {
    if (selected_xml_path_.empty()) return;
    demo_loaded_ = false;
    report_ = config::Mach3XmlImporter::importFromFile(selected_xml_path_, base_cfg, import_options_);
    has_loaded_report_ = true;
}

void Mach3MigrationDialog::loadDemoMach3Profile(const core::MachineConfig& base_cfg, bool is_plasma_app) {
    selected_xml_path_.clear();
    demo_loaded_ = true;
    demo_is_plasma_ = is_plasma_app;
    report_ = config::Mach3XmlImporter::importFromString(
        is_plasma_app ? kDemoPlasmaXml : kDemoMillXml,
        base_cfg,
        import_options_,
        is_plasma_app ? "Mach3Plasma_4x8_Demo.xml" : "Mach3Mill_6040Z_Demo.xml");
    has_loaded_report_ = true;
}

void Mach3MigrationDialog::applyImport(core::IMotionController& controller) {
    if (!has_backup_) {
        backup_config_ = controller.getConfig();
        has_backup_ = true;
    }
    controller.applyConfig(report_.migrated_config);
}

bool Mach3MigrationDialog::restoreBackup(core::IMotionController& controller) {
    if (!has_backup_) return false;
    controller.setTorchOrSpindle(false);
    controller.applyConfig(backup_config_);
    has_backup_ = false;
    return true;
}

bool Mach3MigrationDialog::render(core::IMotionController& controller, bool is_plasma_app) {
    if (phase_ == WizardPhase::Closed) {
        return false;
    }

    const char* modal_title = (phase_ == WizardPhase::ImportAudit)
        ? "Migrate Machine Profile from Mach3 XML##Mach3Wizard"
        : "Post-Migration Machine Verification & Safety Test Routine##Mach3Wizard";

    if (open_popup_) {
        ImGui::OpenPopup(modal_title);
        open_popup_ = false;
    }

    ImVec2 vp_center = ImGui::GetMainViewport()->GetCenter();
    ImVec2 vp_size   = ImGui::GetMainViewport()->WorkSize;
    float target_w = std::min(1120.0f, vp_size.x - 30.0f);
    float target_h = std::min(840.0f,  vp_size.y - 30.0f);
    ImGui::SetNextWindowPos(vp_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(target_w, target_h), ImGuiCond_Appearing);

    bool config_modified = false;
    bool is_open = true;

    ImGui::PushStyleColor(ImGuiCol_PopupBg, ImVec4(0.08f, 0.10f, 0.13f, 0.98f));
    ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.0f, 0.78f, 0.95f, 0.85f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 14.0f));

    if (ImGui::BeginPopupModal(modal_title, &is_open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar)) {
        if (phase_ == WizardPhase::ImportAudit) {
            config_modified = renderImportAuditPhase(controller, is_plasma_app);
        } else if (phase_ == WizardPhase::MachineTest) {
            config_modified = renderMachineTestPhase(controller, is_plasma_app);
        }
        ImGui::EndPopup();
    } else {
        phase_ = WizardPhase::Closed;
    }

    // Closing the window with [X] counts as Cancel: an applied but unconfirmed import is undone
    if (!is_open) {
        if (restoreBackup(controller)) config_modified = true;
        phase_ = WizardPhase::Closed;
    }

    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
    return config_modified;
}

bool Mach3MigrationDialog::renderImportAuditPhase(core::IMotionController& controller, bool is_plasma_app) {
    bool modified = false;
    // Imports start from the configuration as it was before any import of this session
    const core::MachineConfig current_cfg = has_backup_ ? backup_config_ : controller.getConfig();

    auto reimport = [&]() {
        if (!selected_xml_path_.empty()) {
            reloadReport(current_cfg);
        } else if (demo_loaded_) {
            loadDemoMach3Profile(current_cfg, demo_is_plasma_);
        }
    };

    // 1. Top File Selection & Discovery Card
    ImGui::BeginChild("##Mach3SourceCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(ImVec4(0.0f, 0.90f, 1.0f, 1.0f), "1. SELECT MACH3 XML PROFILE (e.g. C:\\Mach3\\Mach3Mill.xml)");
        ImGui::PopTextWrapPos();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.12f, 0.45f, 0.82f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.55f, 0.95f, 1.0f));
        const char* browse_lbl = "Browse Mach3 XML File...";
        if (ImGui::Button(browse_lbl, ImVec2(buttonWidth(browse_lbl), 32.0f))) {
            std::string picked = openFileDialog(
                "Select Mach3 XML Configuration Profile",
                "Mach3 XML Profiles (*.xml)",
                {"*.xml"});
            if (!picked.empty()) {
                selected_xml_path_ = picked;
                import_options_.lpt_pin_overrides.clear();
                reloadReport(current_cfg);
            }
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();
        const char* demo_lbl = "Load Sample Profile";
        if (ImGui::Button(demo_lbl, ImVec2(buttonWidth(demo_lbl), 32.0f))) {
            import_options_.lpt_pin_overrides.clear();
            loadDemoMach3Profile(current_cfg, is_plasma_app);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Loads a sample Mach3 profile to preview the migration and the test routine.");
        }

        ImGui::SameLine();
        const char* skip_lbl = "Skip Import: Run Machine Test";
        if (ImGui::Button(skip_lbl, ImVec2(buttonWidth(skip_lbl), 32.0f))) {
            resetTestRoutineState();
            phase_ = WizardPhase::MachineTest;
            open_popup_ = true;
            ImGui::CloseCurrentPopup();
        }

        if (!discovered_profiles_.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.65f, 0.85f, 1.0f, 1.0f), "Found in C:\\Mach3:");
            for (const auto& p : discovered_profiles_) {
                std::string fname = std::filesystem::path(p).filename().string();
                if (ImGui::GetContentRegionAvail().x > buttonWidth(fname.c_str()) + 8.0f) ImGui::SameLine();
                if (ImGui::Button(fname.c_str(), ImVec2(buttonWidth(fname.c_str()), 28.0f))) {
                    selected_xml_path_ = p;
                    import_options_.lpt_pin_overrides.clear();
                    reloadReport(current_cfg);
                }
            }
        }

        // Conversion options
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        bool opt_changed = false;
        const char* units_items[] = {"From the profile (<Units>: 0 = mm, 1 = inch)", "Force millimeters", "Force inches"};
        int units_idx = import_options_.units_override + 1;
        ImGui::TextWrapped("Native units (Mach3 Config -> Select Native Units)");
        ImGui::SetNextItemWidth(std::min(360.0f, ImGui::GetContentRegionAvail().x));
        if (ImGui::Combo("##UnitsMode", &units_idx, units_items, IM_ARRAYSIZE(units_items))) {
            import_options_.units_override = units_idx - 1;
            opt_changed = true;
        }

        if (ImGui::Checkbox("Fold Homing/Limits 'Reversed' into Dir Invert", &import_options_.combine_reversed_with_dir)) opt_changed = true;
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Mach3 reverses an axis either with Dir LowActive or with 'Reversed'; both become Dir Invert here.");
        }
        if (ImGui::Checkbox("Swap soft limits when Min > Max", &import_options_.auto_fix_inverted_soft_limits)) opt_changed = true;

        if (opt_changed && has_loaded_report_) {
            reimport();
        }
    }
    ImGui::EndChild();

    ImGui::Spacing();

    const float bottom_bar_h = 34.0f + ImGui::GetStyle().ItemSpacing.y * 2.0f + 8.0f;

    if (!has_loaded_report_) {
        ImGui::BeginChild("##Mach3EmptyCard", ImVec2(0.0f, -bottom_bar_h), ImGuiChildFlags_Borders);
        ImGui::Spacing();
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(ImVec4(0.75f, 0.82f, 0.90f, 1.0f),
                           "Select a Mach3 profile above (usually C:\\Mach3\\Mach3Mill.xml or Mach3Plasma.xml). Nothing "
                           "changes until you press Apply.");
        ImGui::Spacing();
        BulletTextWrapped("Motor tuning: steps, max velocity and acceleration, converted from inches when needed.");
        BulletTextWrapped("Directions: Step/Dir Low Active and the Homing/Limits 'Reversed' flag.");
        BulletTextWrapped("Homing direction and speed, soft limits, and a dual-motor gantry (A slaved to X or Y).");
        BulletTextWrapped("Inputs: limits, home switches, probe and E-Stop. Parallel-port profiles get a wiring plan for the RNR terminals.");
        BulletTextWrapped("Spindle delays, the RNR plugin settings and the Huanyang VFD plugin (port, speed monitoring, limits).");
        BulletTextWrapped("Afterwards: a 5-stage test routine to check inputs, directions, steps and the spindle.");
        ImGui::PopTextWrapPos();
        ImGui::EndChild();
    } else if (!report_.success) {
        ImGui::BeginChild("##Mach3ErrCard", ImVec2(0.0f, -bottom_bar_h), ImGuiChildFlags_Borders);
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "MIGRATION ERROR: %s", report_.error_message.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndChild();
    } else {
        const size_t n_ok   = report_.countBySeverity(config::MigrationSeverity::Ok);
        const size_t n_auto = report_.countBySeverity(config::MigrationSeverity::AutoCorrected);
        const size_t n_warn = report_.countBySeverity(config::MigrationSeverity::Warning);

        ImGui::BeginChild("##Mach3AuditContent", ImVec2(0.0f, -bottom_bar_h), ImGuiChildFlags_None);
        {
            // Profile type mismatch
            const bool mismatch = report_.detected_lathe_profile ||
                                  (is_plasma_app && !report_.detected_plasma_profile) ||
                                  (!is_plasma_app && report_.detected_plasma_profile);
            if (mismatch) {
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.35f, 0.15f, 0.05f, 0.95f));
                ImGui::BeginChild("##ProfMismatchBanner", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
                ImGui::PushTextWrapPos(0.0f);
                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f),
                                   "[!] PROFILE TYPE: '%s' looks like a %s profile, and this is %s. Check the spindle, "
                                   "torch and axis settings before running.",
                                   report_.profile_name.c_str(),
                                   report_.detected_lathe_profile ? "lathe" : (report_.detected_plasma_profile ? "plasma" : "mill/router"),
                                   is_plasma_app ? "BreakoutCut (plasma)" : "BreakoutMill");
                ImGui::PopTextWrapPos();
                ImGui::EndChild();
                ImGui::PopStyleColor();
                ImGui::Spacing();
            }

            // Safety warnings
            if (!report_.safety_warnings.empty()) {
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.28f, 0.12f, 0.06f, 0.90f));
                ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.95f, 0.55f, 0.15f, 0.90f));
                ImGui::BeginChild("##SafetyWarnCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
                ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.22f, 1.0f),
                                   "SAFETY NOTES (%zu):", report_.safety_warnings.size());
                ImGui::PushTextWrapPos(0.0f);
                for (const auto& w : report_.safety_warnings) {
                    BulletTextWrapped("%s", w.c_str());
                }
                ImGui::PopTextWrapPos();
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
            }

            // Wiring plan for parallel-port inputs (editable)
            if (!report_.lpt_remaps.empty()) {
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.18f, 0.26f, 0.90f));
                ImGui::PushStyleColor(ImGuiCol_Border,  ImVec4(0.0f, 0.75f, 0.95f, 0.85f));
                ImGui::BeginChild("##LptWiringCard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
                ImGui::PushTextWrapPos(0.0f);
                ImGui::TextColored(ImVec4(0.0f, 0.92f, 1.0f, 1.0f), "WIRING PLAN: PARALLEL PORT -> RNR INPUT TERMINALS");
                ImGui::PopTextWrapPos();
                ImGui::PushTextWrapPos(0.0f);
                ImGui::TextWrapped("These switches were wired to the parallel-port breakout board. Choose the RNR terminal "
                                   "each one will be wired to. The RNR inputs may have the opposite polarity to your old "
                                   "breakout board: Stage 1 of the test checks that.");
                ImGui::PopTextWrapPos();
                ImGui::Spacing();
                static const char* kTerminals[] = {"Off", "IN1", "IN2", "IN3", "IN4", "IN5", "IN6",
                                                   "IN7", "IN8", "IN9", "IN10", "IN11", "IN12"};
                bool plan_changed = false;
                if (ImGui::BeginTable("##LptPlanTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                    ImGui::TableSetupColumn("Signal", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                    ImGui::TableSetupColumn("Mach3 (parallel port)", ImGuiTableColumnFlags_WidthStretch, 1.2f);
                    ImGui::TableSetupColumn("RNR terminal", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                    ImGui::TableHeadersRow();
                    for (const auto& rm : report_.lpt_remaps) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(rm.signal_name.c_str());
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("Port %d Pin %d%s", rm.lpt_port, rm.lpt_pin, rm.active_low ? ", Active Low" : "");
                        ImGui::TableSetColumnIndex(2);
                        int term = rm.rnr_in_pin;
                        ImGui::SetNextItemWidth(-1.0f);
                        if (ImGui::Combo(("##Term" + rm.signal_name).c_str(), &term, kTerminals, IM_ARRAYSIZE(kTerminals))) {
                            import_options_.lpt_pin_overrides[rm.signal_name] = static_cast<uint8_t>(term);
                            plan_changed = true;
                        }
                    }
                    ImGui::EndTable();
                }
                ImGui::EndChild();
                ImGui::PopStyleColor(2);
                ImGui::Spacing();
                if (plan_changed) {
                    reimport();
                }
            }

            // Summary & filter
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(ImVec4(0.45f, 0.85f, 1.0f, 1.0f),
                               "Profile: %s  |  %zu OK  |  %zu Converted  |  %zu Need Attention",
                               report_.profile_name.c_str(), n_ok, n_auto, n_warn);
            ImGui::PopTextWrapPos();
            ImGui::Checkbox("Show only converted values and warnings", &show_warnings_only_);
            ImGui::Spacing();

            // Audit table: the notes column takes the remaining width and wraps
            if (ImGui::BeginTable("##Mach3AuditTable", 5,
                                  ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable))
            {
                ImGui::TableSetupColumn("Category",        ImGuiTableColumnFlags_WidthFixed, 110.0f);
                ImGui::TableSetupColumn("Parameter",       ImGuiTableColumnFlags_WidthFixed, 190.0f);
                ImGui::TableSetupColumn("Mach3 Value",     ImGuiTableColumnFlags_WidthFixed, 190.0f);
                ImGui::TableSetupColumn("BreakoutCNC",     ImGuiTableColumnFlags_WidthFixed, 200.0f);
                ImGui::TableSetupColumn("Notes",           ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableHeadersRow();

                for (const auto& item : report_.items) {
                    if (show_warnings_only_ && item.severity == config::MigrationSeverity::Ok) {
                        continue;
                    }
                    ImGui::TableNextRow();
                    ImGui::PushTextWrapPos(0.0f); // Wraps at the right edge of each cell

                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextDisabled("%s", item.category.c_str());

                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(item.parameter_name.c_str());

                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextColored(ImVec4(0.75f, 0.80f, 0.88f, 1.0f), "%s", item.mach3_raw_value.c_str());

                    ImGui::TableSetColumnIndex(3);
                    ImVec4 val_col(0.20f, 0.95f, 0.55f, 1.0f);
                    if (item.severity == config::MigrationSeverity::AutoCorrected) {
                        val_col = ImVec4(0.30f, 0.88f, 1.0f, 1.0f);
                    } else if (item.severity == config::MigrationSeverity::Warning) {
                        val_col = ImVec4(1.0f, 0.72f, 0.20f, 1.0f);
                    }
                    ImGui::TextColored(val_col, "%s", item.migrated_value.c_str());

                    ImGui::TableSetColumnIndex(4);
                    if (item.severity == config::MigrationSeverity::Warning) {
                        ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.20f, 1.0f), "[!] %s", item.note.c_str());
                    } else if (item.severity == config::MigrationSeverity::AutoCorrected) {
                        ImGui::TextColored(ImVec4(0.45f, 0.85f, 1.0f, 1.0f), "[Converted] %s", item.note.c_str());
                    } else {
                        ImGui::TextDisabled("%s", item.note.c_str());
                    }
                    ImGui::PopTextWrapPos();
                }
                ImGui::EndTable();
            }
        }
        ImGui::EndChild();
    }

    // Bottom Action Bar
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::BeginDisabled(!has_loaded_report_ || !report_.success);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.10f, 0.62f, 0.30f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.14f, 0.75f, 0.38f, 1.0f));
    const char* apply_test_lbl = "Apply & Start Machine Test (Recommended)";
    if (ImGui::Button(apply_test_lbl, ImVec2(buttonWidth(apply_test_lbl), 34.0f))) {
        applyImport(controller);
        modified = true;
        resetTestRoutineState();
        phase_ = WizardPhase::MachineTest;
        open_popup_ = true;
        ImGui::CloseCurrentPopup();
    }
    ImGui::PopStyleColor(2);

    ImGui::SameLine();
    const char* apply_lbl = "Apply Settings Only";
    if (ImGui::Button(apply_lbl, ImVec2(buttonWidth(apply_lbl), 34.0f))) {
        applyImport(controller);
        has_backup_ = false; // Applied for good
        modified = true;
        phase_ = WizardPhase::Closed;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    const char* cancel_lbl = has_backup_ ? "Cancel (Restore Previous Settings)" : "Cancel";
    if (ImGui::Button(cancel_lbl, ImVec2(-1.0f, 34.0f))) {
        if (restoreBackup(controller)) modified = true;
        phase_ = WizardPhase::Closed;
        ImGui::CloseCurrentPopup();
    }

    return modified;
}

bool Mach3MigrationDialog::renderMachineTestPhase(
    core::IMotionController& controller,
    bool is_plasma_app)
{
    bool modified = false;
    core::MachineConfig cfg = controller.getConfig();
    const core::TelemetrySnapshot tel = controller.getTelemetry();

    const bool is_imp = (cfg.linear_units == core::LinearUnits::Imperial);
    const double u_scale   = is_imp ? (1.0 / 25.4) : 1.0;
    const double u_unscale = is_imp ? 25.4 : 1.0;
    const char*  u_str     = is_imp ? "in" : "mm";
    const char*  f_str     = is_imp ? "IPM" : "mm/min";

    // Latch any active inputs for Stage 4 switch verification
    for (int i = 0; i < 12; ++i) {
        if ((tel.inputs_bitmask & (1U << i)) != 0) {
            input_latch_seen_[i] = true;
        }
    }

    // ------------------------------------------------------------------------
    // Top Safety Bar: E-Stop / Arming, Live Coordinates & Input LEDs
    // ------------------------------------------------------------------------
    ImGui::BeginChild("##TestSafetyTopBar", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoScrollbar);
    {
        const bool in_estop = (tel.state == core::ControllerState::EStop);
        if (in_estop) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.85f, 0.20f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.28f, 0.22f, 1.0f));
            if (ImGui::Button("  [!] MACHINE IN E-STOP - CLICK TO RESET & ARM FOR TESTING  ", ImVec2(430.0f, 32.0f))) {
                controller.resetFromEStop();
            }
            ImGui::PopStyleColor(2);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.12f, 0.12f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.92f, 0.18f, 0.18f, 1.0f));
            if (ImGui::Button("  EMERGENCY STOP / HALT MOTION IMMEDIATELY (ESC)  ", ImVec2(430.0f, 32.0f))) {
                controller.triggerEStop();
            }
            ImGui::PopStyleColor(2);
        }

        ImGui::SameLine(0.0f, 16.0f);
        ImFont* gfont = AppWindow::getGCodeFont();
        if (gfont) ImGui::PushFont(gfont);
        if (is_imp) {
            ImGui::TextColored(ImVec4(0.0f, 0.92f, 1.0f, 1.0f),
                               "POS: X:%+7.3f\"  Y:%+7.3f\"  Z:%+7.3f\"  A:%+7.3f\"",
                               tel.work_pos_mm[0] * u_scale,
                               tel.work_pos_mm[1] * u_scale,
                               tel.work_pos_mm[2] * u_scale,
                               tel.work_pos_mm[3] * u_scale);
        } else {
            ImGui::TextColored(ImVec4(0.0f, 0.92f, 1.0f, 1.0f),
                               "POS: X:%+7.2f   Y:%+7.2f   Z:%+7.2f   A:%+7.2f mm",
                               tel.work_pos_mm[0],
                               tel.work_pos_mm[1],
                               tel.work_pos_mm[2],
                               tel.work_pos_mm[3]);
        }
        if (gfont) ImGui::PopFont();
    }
    ImGui::EndChild();

    ImGui::Spacing();

    // ------------------------------------------------------------------------
    // 5-Stage Stepper Header Tabs
    // ------------------------------------------------------------------------
    const char* stage_titles[5] = {
        "1. Pin Polarity Sanity",
        "2. Axis Directions",
        "3. Step Calibration",
        "4. Switch & Probe Test",
        "5. Spindle / Outputs"
    };

    const float tab_w = (ImGui::GetContentRegionAvail().x - 32.0f) / 5.0f;
    for (int s = 0; s < 5; ++s) {
        if (s > 0) ImGui::SameLine();
        bool active = (test_stage_ == s);
        bool done   = stage_completed_[s];

        if (active) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.08f, 0.48f, 0.82f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.12f, 0.58f, 0.92f, 1.0f));
        } else if (done) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.10f, 0.42f, 0.24f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.14f, 0.52f, 0.30f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f, 0.19f, 0.24f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.26f, 0.32f, 1.0f));
        }

        char btn_lbl[64];
        std::snprintf(btn_lbl, sizeof(btn_lbl), "%s%s##StgTab%d",
                      done ? "[OK] " : "", stage_titles[s], s);
        if (ImGui::Button(btn_lbl, ImVec2(tab_w, 30.0f))) {
            test_stage_ = s;
        }
        ImGui::PopStyleColor(2);
    }

    ImGui::Spacing();

    const float bottom_bar_h = 34.0f + ImGui::GetStyle().ItemSpacing.y * 2.0f + 8.0f;

    // ------------------------------------------------------------------------
    // Stage Body Content
    // ------------------------------------------------------------------------
    ImGui::BeginChild("##TestStageBody", ImVec2(0.0f, -bottom_bar_h), ImGuiChildFlags_Borders, ImGuiWindowFlags_NoScrollbar);
    {
        if (test_stage_ == 0) {
            // ================================================================
            // STAGE 1: IDLE PIN POLARITY SANITY CHECK
            // ================================================================
            ImGui::TextColored(ImVec4(0.0f, 0.90f, 1.0f, 1.0f),
                               "STAGE 1: IDLE INPUT PIN POLARITY SANITY CHECK");
            ImGui::TextWrapped(
                "Ensure the machine is NOT currently pressing any limit/home switch, E-Stop is released, and the Touch Probe is not touching. "
                "All configured inputs below should read [CLEAR] while idle. If any pin reads [TRIGGERED WHILE IDLE], click [Invert Polarity] to fix it immediately.");
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            int active_idle_faults = 0;

            auto renderPinSanityRow = [&](const char* name, uint8_t pin, bool& invert_flag, int row_id) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(name);

                ImGui::TableSetColumnIndex(1);
                if (pin == 0) {
                    ImGui::TextDisabled("Disabled");
                } else {
                    ImGui::Text("IN%u", pin);
                }

                ImGui::TableSetColumnIndex(2);
                if (pin == 0) {
                    ImGui::TextDisabled("-");
                } else {
                    ImGui::TextUnformatted(invert_flag ? "Active-Low (Inverted)" : "Active-High (Normal)");
                }

                ImGui::TableSetColumnIndex(3);
                if (pin == 0) {
                    ImGui::TextDisabled("Not Configured");
                } else {
                    bool trig = isPinTriggered(tel.inputs_bitmask, pin, invert_flag);
                    if (trig) {
                        ++active_idle_faults;
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.30f, 1.0f), "[!] TRIGGERED WHILE IDLE");
                    } else {
                        ImGui::TextColored(ImVec4(0.20f, 0.95f, 0.50f, 1.0f), "[OK] CLEAR (OPEN)");
                    }
                }

                ImGui::TableSetColumnIndex(4);
                if (pin > 0) {
                    char btn_id[48];
                    std::snprintf(btn_id, sizeof(btn_id), "Invert Polarity##PinSan%d", row_id);
                    if (ImGui::Button(btn_id, ImVec2(buttonWidth(btn_id), 28.0f))) {
                        invert_flag = !invert_flag;
                        controller.applyConfig(cfg);
                        // Never re-arm automatically after changing pin polarity (this may be the E-Stop
                        // input itself): the operator has to press RESET explicitly.
                        modified = true;
                    }
                }
            };

            if (ImGui::BeginTable("##Stage1PinTable", 5,
                                  ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(0.0f, 0.0f)))
            {
                ImGui::TableSetupScrollFreeze(1, 1);
                ImGui::TableSetupColumn("Signal Name",     ImGuiTableColumnFlags_WidthFixed, 230.0f);
                ImGui::TableSetupColumn("Terminal",        ImGuiTableColumnFlags_WidthFixed, 95.0f);
                ImGui::TableSetupColumn("Polarity",        ImGuiTableColumnFlags_WidthFixed, 180.0f);
                ImGui::TableSetupColumn("Live Idle State", ImGuiTableColumnFlags_WidthFixed, 240.0f);
                ImGui::TableSetupColumn("Quick Fix",       ImGuiTableColumnFlags_WidthFixed, 160.0f);
                ImGui::TableHeadersRow();

                renderPinSanityRow("External E-Stop",       cfg.estop_pin,               cfg.estop_invert,               0);
                renderPinSanityRow("Touch Probe (G31)",     cfg.probe.probe_pin,         cfg.probe.probe_invert,         1);
                renderPinSanityRow("X+ Limit Switch",       cfg.axes[0].limit_plus_pin,  cfg.axes[0].limit_plus_invert,  2);
                renderPinSanityRow("X- Limit Switch",       cfg.axes[0].limit_minus_pin, cfg.axes[0].limit_minus_invert, 3);
                renderPinSanityRow("X Home Switch",         cfg.axes[0].home_pin,        cfg.axes[0].home_invert,        4);
                renderPinSanityRow("Y+ Limit Switch",       cfg.axes[1].limit_plus_pin,  cfg.axes[1].limit_plus_invert,  5);
                renderPinSanityRow("Y- Limit Switch",       cfg.axes[1].limit_minus_pin, cfg.axes[1].limit_minus_invert, 6);
                renderPinSanityRow("Y Home Switch",         cfg.axes[1].home_pin,        cfg.axes[1].home_invert,        7);
                renderPinSanityRow("Z+ Limit Switch",       cfg.axes[2].limit_plus_pin,  cfg.axes[2].limit_plus_invert,  8);
                renderPinSanityRow("Z- Limit Switch",       cfg.axes[2].limit_minus_pin, cfg.axes[2].limit_minus_invert, 9);
                renderPinSanityRow("Z Home Switch",         cfg.axes[2].home_pin,        cfg.axes[2].home_invert,        10);
                if (is_plasma_app) {
                    renderPinSanityRow("Plasma THC Arc OK", cfg.thc.arc_ok_pin,          cfg.thc.arc_ok_invert,          11);
                    renderPinSanityRow("Plasma THC UP",     cfg.thc.thc_up_pin,          cfg.thc.thc_up_invert,          12);
                    renderPinSanityRow("Plasma THC DOWN",   cfg.thc.thc_down_pin,        cfg.thc.thc_down_invert,        13);
                }
                ImGui::EndTable();
            }

            ImGui::Spacing();
            if (active_idle_faults == 0) {
                stage_completed_[0] = true;
                ImGui::TextColored(ImVec4(0.20f, 0.95f, 0.50f, 1.0f),
                                   "[STAGE 1 PASSED] All configured inputs are CLEAR in idle state. Ready for Stage 2 (Axis Direction Check)!");
            } else {
                stage_completed_[0] = false;
                ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.20f, 1.0f),
                                   "[!] %d input(s) are currently reading TRIGGERED while idle. Click 'Invert Polarity' above or release the physical switch.",
                                   active_idle_faults);
            }
        }
        else if (test_stage_ == 1) {
            // ================================================================
            // STAGE 2: SAFE LOW-SPEED AXIS DIRECTION VERIFICATION
            // ================================================================
            ImGui::TextColored(ImVec4(0.0f, 0.90f, 1.0f, 1.0f),
                               "STAGE 2: SAFE LOW-SPEED AXIS DIRECTION VERIFICATION (+Z, +X, +Y, +A)");
            ImGui::TextWrapped(
                "Before homing or running G-Code, verify that each axis moves in the correct physical direction using a short, low-speed nudge. "
                "Always verify Z (+Z = UP away from table) first! If an axis moves opposite to the button pressed, click [Flip Direction & Re-Test].");
            ImGui::Spacing();

            float disp_nudge = static_cast<float>(nudge_dist_mm_ * u_scale);
            float disp_feed  = static_cast<float>(nudge_feed_mm_min_ * u_scale);
            ImGui::SetNextItemWidth(110.0f);
            if (ImGui::InputFloat((std::string("Test Nudge Distance (") + u_str + ")").c_str(),
                                  &disp_nudge, is_imp ? 0.05f : 1.0f, is_imp ? 0.2f : 5.0f, is_imp ? "%.3f" : "%.1f"))
            {
                nudge_dist_mm_ = std::clamp(static_cast<float>(disp_nudge * u_unscale), 0.5f, 25.0f);
            }
            ImGui::SameLine(0.0f, 20.0f);
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputFloat((std::string("Test Speed (") + f_str + ")").c_str(),
                                  &disp_feed, is_imp ? 2.0f : 50.0f, is_imp ? 10.0f : 200.0f, is_imp ? "%.1f" : "%.0f"))
            {
                nudge_feed_mm_min_ = std::clamp(static_cast<float>(disp_feed * u_unscale), 50.0f, 1500.0f);
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Test order: Z (index 2) first for safety, then X (0), Y (1), A (3)
            const int axis_order[4] = {2, 0, 1, 3};
            const char* axis_labels[4] = {
                "X Axis (Left / Right)",
                "Y Axis (Front / Back)",
                "Z Axis (Spindle / Torch Vertical - TEST FIRST!)",
                "A Axis (Rotary / Gantry Slave)"
            };
            const char* pos_expected[4] = {
                "+X should move Tool RIGHT (or Table Left)",
                "+Y should move Tool AWAY/BACK (or Table Front)",
                "+Z MUST lift Spindle/Torch UP away from workpiece",
                "+A positive rotation / slaved gantry motion"
            };

            const bool can_jog = (tel.state == core::ControllerState::Idle);

            if (ImGui::BeginTable("##Stage2DirTable", 5,
                                  ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(0.0f, 0.0f)))
            {
                ImGui::TableSetupScrollFreeze(1, 1);
                ImGui::TableSetupColumn("Axis",                ImGuiTableColumnFlags_WidthFixed, 240.0f);
                ImGui::TableSetupColumn("Expected + Motion",   ImGuiTableColumnFlags_WidthFixed, 320.0f);
                ImGui::TableSetupColumn("Safe Nudge Controls", ImGuiTableColumnFlags_WidthFixed, 185.0f);
                ImGui::TableSetupColumn("Direction Polarity",  ImGuiTableColumnFlags_WidthFixed, 165.0f);
                ImGui::TableSetupColumn("Verification Status", ImGuiTableColumnFlags_WidthFixed, 190.0f);
                ImGui::TableHeadersRow();

                for (int k = 0; k < 4; ++k) {
                    int a = axis_order[k];
                    const char ax_ch = "XYZA"[a];
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextColored(a == 2 ? ImVec4(1.0f, 0.85f, 0.30f, 1.0f) : ImVec4(0.85f, 0.92f, 1.0f, 1.0f),
                                       "%s", axis_labels[a]);

                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextWrapped("%s", pos_expected[a]);

                    ImGui::TableSetColumnIndex(2);
                    if (a == 3 && cfg.gantryMasterAxis() >= 0) {
                        // The slaved A motor only moves with its master: nudge the master and watch both motors
                        ImGui::TextWrapped("Moves with %c: nudge %c and check that both gantry motors turn the same way.",
                                           "XY"[cfg.gantryMasterAxis()], "XY"[cfg.gantryMasterAxis()]);
                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("DirInv: %s", cfg.axes[a].dir_active_low ? "ON" : "OFF");
                        char flip_a[48];
                        std::snprintf(flip_a, sizeof(flip_a), "Flip A Direction##FlpSlave");
                        if (ImGui::Button(flip_a, ImVec2(buttonWidth(flip_a), 28.0f))) {
                            cfg.axes[a].dir_active_low = !cfg.axes[a].dir_active_low;
                            controller.applyConfig(cfg);
                            modified = true;
                        }
                        ImGui::TableSetColumnIndex(4);
                        ImGui::TextDisabled("Checked with %c", "XY"[cfg.gantryMasterAxis()]);
                        continue;
                    }
                    ImGui::BeginDisabled(!can_jog);
                    char btn_pos[32], btn_neg[32];
                    std::snprintf(btn_pos, sizeof(btn_pos), "Nudge +%c##Dir%d", ax_ch, a);
                    std::snprintf(btn_neg, sizeof(btn_neg), "Nudge -%c##Dir%d", ax_ch, a);

                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.78f, 1.0f));
                    if (ImGui::Button(btn_pos, ImVec2(84.0f, 28.0f))) {
                        controller.jogStep(static_cast<core::Axis>(a), nudge_dist_mm_, nudge_feed_mm_min_);
                    }
                    ImGui::PopStyleColor();
                    ImGui::SameLine();
                    if (ImGui::Button(btn_neg, ImVec2(84.0f, 28.0f))) {
                        controller.jogStep(static_cast<core::Axis>(a), -nudge_dist_mm_, nudge_feed_mm_min_);
                    }
                    ImGui::EndDisabled();

                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("DirInv: %s", cfg.axes[a].dir_active_low ? "ON" : "OFF");
                    char flip_btn[48];
                    std::snprintf(flip_btn, sizeof(flip_btn), "Flip %c Direction##Flp%d", ax_ch, a);
                    if (ImGui::Button(flip_btn, ImVec2(buttonWidth(flip_btn), 28.0f))) {
                        cfg.axes[a].dir_active_low = !cfg.axes[a].dir_active_low;
                        controller.applyConfig(cfg);
                        axis_dir_verified_[a] = false;
                        modified = true;
                    }

                    ImGui::TableSetColumnIndex(4);
                    if (axis_dir_verified_[a]) {
                        ImGui::TextColored(ImVec4(0.20f, 0.95f, 0.50f, 1.0f), "[VERIFIED OK]");
                    } else {
                        char ok_btn[48];
                        std::snprintf(ok_btn, sizeof(ok_btn), "Confirm +%c Correct##OkDir%d", ax_ch, a);
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.55f, 0.28f, 1.0f));
                        if (ImGui::Button(ok_btn, ImVec2(buttonWidth(ok_btn), 28.0f))) {
                            axis_dir_verified_[a] = true;
                            stage_completed_[1] = (axis_dir_verified_[0] && axis_dir_verified_[1] && axis_dir_verified_[2]);
                        }
                        ImGui::PopStyleColor();
                    }
                }
                ImGui::EndTable();
            }
        }
        else if (test_stage_ == 2) {
            // ================================================================
            // STAGE 3: STEP CALIBRATION & TRAVEL DISTANCE ACCURACY CHECK
            // ================================================================
            ImGui::TextColored(ImVec4(0.0f, 0.90f, 1.0f, 1.0f),
                               "STAGE 3: STEP CALIBRATION & TRAVEL DISTANCE CHECK (Steps/mm | Steps/in)");
            ImGui::TextWrapped(
                "Verify that the migrated step resolution matches your physical leadscrew/belt pitch and stepper driver microstepping switches. "
                "Zero an axis against a ruler, dial indicator, or mark, command a test move, and measure the actual travel.");
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            const char* ax_items[] = {"X Axis", "Y Axis", "Z Axis", "A Axis"};
            ImGui::SetNextItemWidth(140.0f);
            ImGui::Combo("Select Axis to Calibrate", &calib_axis_idx_, ax_items, 4);
            calib_axis_idx_ = std::clamp(calib_axis_idx_, 0, 3);

            auto& cal_ax = cfg.axes[calib_axis_idx_];
            const double cur_steps_disp = is_imp ? (cal_ax.steps_per_mm * 25.4) : cal_ax.steps_per_mm;

            ImGui::SameLine(0.0f, 24.0f);
            ImGui::TextColored(ImVec4(0.35f, 0.88f, 1.0f, 1.0f),
                               "Current %s Resolution: %.3f steps/%s (%.3f steps/mm)",
                               ax_items[calib_axis_idx_], cur_steps_disp, u_str, cal_ax.steps_per_mm);

            ImGui::Spacing();

            float cmd_disp = static_cast<float>(calib_cmd_dist_mm_ * u_scale);
            ImGui::SetNextItemWidth(130.0f);
            if (ImGui::InputFloat((std::string("1. Commanded Test Distance (") + u_str + ")").c_str(),
                                  &cmd_disp, is_imp ? 0.1f : 5.0f, is_imp ? 0.5f : 10.0f, is_imp ? "%.3f" : "%.2f"))
            {
                calib_cmd_dist_mm_ = std::clamp(static_cast<float>(cmd_disp * u_unscale), 0.5f, 200.0f);
                calib_meas_dist_mm_ = calib_cmd_dist_mm_;
            }

            const bool can_jog = (tel.state == core::ControllerState::Idle);
            ImGui::BeginDisabled(!can_jog);
            if (ImGui::Button("Zero Axis DRO Here", ImVec2(160.0f, 30.0f))) {
                controller.zeroWorkAxis(static_cast<core::Axis>(calib_axis_idx_));
            }
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.48f, 0.82f, 1.0f));
            if (ImGui::Button("Move +Test Distance", ImVec2(175.0f, 30.0f))) {
                controller.jogStep(static_cast<core::Axis>(calib_axis_idx_), calib_cmd_dist_mm_, nudge_feed_mm_min_);
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if (ImGui::Button("Move Back (-Test Distance)", ImVec2(195.0f, 30.0f))) {
                controller.jogStep(static_cast<core::Axis>(calib_axis_idx_), -calib_cmd_dist_mm_, nudge_feed_mm_min_);
            }
            ImGui::EndDisabled();

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            float meas_disp = static_cast<float>(calib_meas_dist_mm_ * u_scale);
            ImGui::SetNextItemWidth(130.0f);
            if (ImGui::InputFloat((std::string("2. Actual Measured Travel on Ruler/Indicator (") + u_str + ")").c_str(),
                                  &meas_disp, is_imp ? 0.01f : 0.5f, is_imp ? 0.1f : 5.0f, is_imp ? "%.4f" : "%.2f"))
            {
                calib_meas_dist_mm_ = std::max(0.01f, static_cast<float>(meas_disp * u_unscale));
            }

            double new_steps_mm = cal_ax.steps_per_mm * (static_cast<double>(calib_cmd_dist_mm_) / std::max(0.01, static_cast<double>(calib_meas_dist_mm_)));
            double new_steps_disp = is_imp ? (new_steps_mm * 25.4) : new_steps_mm;

            ImGui::Text("Calculated New Resolution: %.3f steps/%s", new_steps_disp, u_str);
            ImGui::SameLine(0.0f, 18.0f);
            if (ImGui::Button("Apply Calibrated Steps", ImVec2(190.0f, 28.0f))) {
                cal_ax.steps_per_mm = std::clamp(new_steps_mm, 1.0, 50000.0);
                controller.applyConfig(cfg);
                calib_meas_dist_mm_ = calib_cmd_dist_mm_;
                axis_steps_verified_[calib_axis_idx_] = true;
                modified = true;
            }
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.55f, 0.28f, 1.0f));
            if (ImGui::Button("Distance Matches Exactly (Mark Verified)", ImVec2(280.0f, 28.0f))) {
                axis_steps_verified_[calib_axis_idx_] = true;
                stage_completed_[2] = true;
            }
            ImGui::PopStyleColor();
        }
        else if (test_stage_ == 3) {
            // ================================================================
            // STAGE 4: LIVE END-STOP, HOME SWITCH & TOUCH PROBE TRIGGER TEST
            // ================================================================
            ImGui::TextColored(ImVec4(0.0f, 0.90f, 1.0f, 1.0f),
                               "STAGE 4: LIVE END-STOP, HOME SWITCH & TOUCH PROBE TRIGGER TEST");
            ImGui::TextWrapped(
                "Manually press each physical Home/Limit switch or touch the Z Probe Plate to the tool tip to verify that BreakoutCNC receives the signal. "
                "When a switch is triggered even briefly, its [LATCHED VERIFIED] badge turns green!");
            ImGui::Spacing();

            // Live IN1..IN12 strip (with clickable simulation toggles if Simulator Mode is active)
            ImGui::Text("Physical Input Terminals (IN1..IN12):");
            for (int i = 1; i <= 12; ++i) {
                if (i > 1) ImGui::SameLine();
                bool is_high = (tel.inputs_bitmask & (1U << (i - 1))) != 0;
                bool seen    = input_latch_seen_[i - 1];
                if (is_high) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.80f, 0.35f, 1.0f));
                } else if (seen) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.42f, 0.28f, 1.0f));
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.20f, 0.25f, 1.0f));
                }
                char in_lbl[24];
                std::snprintf(in_lbl, sizeof(in_lbl), "IN%d", i);
                if (ImGui::Button(in_lbl, ImVec2(56.0f, 28.0f))) {
                    if (tel.simulator_mode) {
                        bool cur_sim = (tel.simulated_inputs_bitmask & (1U << (i - 1))) != 0;
                        controller.setSimulatedInputPin(static_cast<uint8_t>(i), !cur_sim);
                        input_latch_seen_[i - 1] = true;
                    }
                }
                ImGui::PopStyleColor();
            }
            if (tel.simulator_mode) {
                ImGui::TextDisabled("(Simulator Mode active: click any IN1..IN12 button above to simulate a switch press)");
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.55f, 0.28f, 1.0f));
            if (ImGui::Button("Mark Switch & Probe Trigger Test Complete", ImVec2(320.0f, 32.0f))) {
                stage_completed_[3] = true;
            }
            ImGui::PopStyleColor();
        }
        else if (test_stage_ == 4) {
            // ================================================================
            // STAGE 5: AUX OUTPUTS & SPINDLE / TORCH DRY-RUN CHECK
            // ================================================================
            ImGui::TextColored(ImVec4(0.0f, 0.90f, 1.0f, 1.0f),
                               "STAGE 5: AUXILIARY RELAY OUTPUTS (OUT1..OUT4) & SPINDLE DRY-RUN CHECK");
            ImGui::TextWrapped(
                "Test auxiliary relay outputs (Coolant / Air Assist / Dust Collector) and verify low-RPM spindle speed control. "
                "Ensure hands and tools are clear of the spindle before clicking Test Spin.");
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::Text("Relay Outputs (OUT1..OUT4):");
            for (uint8_t o = 1; o <= 4; ++o) {
                if (o > 1) ImGui::SameLine(0.0f, 12.0f);
                bool out_on = (tel.outputs_bitmask & (1U << (o - 1))) != 0;
                if (out_on) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.50f, 0.10f, 1.0f));
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.22f, 0.28f, 1.0f));
                }
                char out_lbl[48];
                std::snprintf(out_lbl, sizeof(out_lbl), "Toggle OUT%u (%s)", o, out_on ? "ON" : "OFF");
                if (ImGui::Button(out_lbl, ImVec2(170.0f, 32.0f))) {
                    controller.setAuxOutput(o, !out_on);
                }
                ImGui::PopStyleColor();
            }

            if (!is_plasma_app) {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.35f, 0.88f, 1.0f, 1.0f), "Spindle Dry-Run Speed Check:");
                ImGui::TextDisabled("Select or type the safe test speed for your spindle (minimum RPM depends on spindle type/cooling):");

                ImGui::SetNextItemWidth(130.0f);
                if (ImGui::InputInt("Test RPM##SpindleTestInput", &test_spindle_rpm_, 500, 2000)) {
                    test_spindle_rpm_ = std::clamp(test_spindle_rpm_, 300, 60000);
                }
                ImGui::SameLine(0.0f, 16.0f);
                ImGui::TextDisabled("Presets:");
                ImGui::SameLine();
                auto renderPreset = [&](int rpm) {
                    char plbl[24];
                    std::snprintf(plbl, sizeof(plbl), "%d##SpPreset", rpm);
                    if (test_spindle_rpm_ == rpm) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.80f, 1.0f));
                    } else {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.26f, 1.0f));
                    }
                    if (ImGui::Button(plbl, ImVec2(buttonWidth(plbl), 28.0f))) {
                        test_spindle_rpm_ = rpm;
                    }
                    ImGui::PopStyleColor();
                    ImGui::SameLine(0.0f, 6.0f);
                };
                renderPreset(1500);
                renderPreset(3000);
                renderPreset(6000);
                renderPreset(10000);
                renderPreset(12000);

                ImGui::Spacing();

                char spin_btn_label[64];
                std::snprintf(spin_btn_label, sizeof(spin_btn_label), "Test Spindle ON @ %d RPM", test_spindle_rpm_);

                bool spindle_is_on = tel.torch_or_spindle_on;
                if (!spindle_is_on) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.50f, 0.82f, 1.0f));
                    if (ImGui::Button(spin_btn_label, ImVec2(240.0f, 32.0f))) {
                        controller.setSpindleSpeedRpm(test_spindle_rpm_);
                        controller.setTorchOrSpindle(true);
                    }
                    ImGui::PopStyleColor();
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.75f, 0.35f, 1.0f));
                    ImGui::Button("SPINDLE RUNNING...", ImVec2(240.0f, 32.0f));
                    ImGui::PopStyleColor();
                }

                ImGui::SameLine(0.0f, 12.0f);
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.80f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.92f, 0.28f, 0.28f, 1.0f));
                if (ImGui::Button("STOP SPINDLE (M5)", ImVec2(180.0f, 32.0f))) {
                    controller.setTorchOrSpindle(false);
                }
                ImGui::PopStyleColor(2);
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (stage_completed_[4]) {
                ImGui::TextColored(ImVec4(0.20f, 0.95f, 0.50f, 1.0f),
                                   "[STAGE 5 VERIFIED] Relays and spindle checks complete. Click 'Save Verified Settings & Close' below to finish!");
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.55f, 0.28f, 1.0f));
                if (ImGui::Button("Mark Stage 5 Complete (Relays & Spindle Verified)", ImVec2(380.0f, 32.0f))) {
                    controller.setTorchOrSpindle(false);
                    stage_completed_[4] = true;
                }
                ImGui::PopStyleColor();
            }
        }
    }
    ImGui::EndChild();

    // ------------------------------------------------------------------------
    // Bottom Navigation Bar
    // ------------------------------------------------------------------------
    ImGui::Separator();
    ImGui::Spacing();

    const char* back_lbl = "< Back to Import";
    if (ImGui::Button(back_lbl, ImVec2(buttonWidth(back_lbl), 34.0f))) {
        phase_ = WizardPhase::ImportAudit;
        open_popup_ = true;
        ImGui::CloseCurrentPopup();
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(test_stage_ <= 0);
    const char* prev_lbl = "< Previous Stage";
    if (ImGui::Button(prev_lbl, ImVec2(buttonWidth(prev_lbl), 34.0f))) {
        --test_stage_;
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::BeginDisabled(test_stage_ >= 4);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.48f, 0.82f, 1.0f));
    const char* next_lbl = "Next Stage >";
    if (ImGui::Button(next_lbl, ImVec2(buttonWidth(next_lbl), 34.0f))) {
        stage_completed_[test_stage_] = true;
        ++test_stage_;
    }
    ImGui::PopStyleColor();
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.10f, 0.62f, 0.30f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.14f, 0.75f, 0.38f, 1.0f));
    const char* save_lbl = "Save Settings & Close";
    if (ImGui::Button(save_lbl, ImVec2(buttonWidth(save_lbl), 34.0f))) {
        controller.setTorchOrSpindle(false);
        has_backup_ = false; // The imported settings are kept
        modified = true;
        phase_ = WizardPhase::Closed;
        ImGui::CloseCurrentPopup();
    }
    ImGui::PopStyleColor(2);

    ImGui::SameLine();
    const char* cancel_lbl = has_backup_ ? "Cancel (Restore Previous Settings)" : "Cancel Without Saving";
    if (ImGui::Button(cancel_lbl, ImVec2(-1.0f, 34.0f))) {
        controller.setTorchOrSpindle(false);
        if (restoreBackup(controller)) modified = true;
        phase_ = WizardPhase::Closed;
        ImGui::CloseCurrentPopup();
    }

    return modified;
}

} // namespace rnr::ui
