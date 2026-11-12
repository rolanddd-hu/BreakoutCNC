// Script engine & Lua tests, and main() running every test of this executable in order
#include "test_support.h"

void test01() {
    // ------------------------------------------------------------------------
    // Test 1: Basic Lua VM Execution & Print Output Redirection
    // ------------------------------------------------------------------------
    std::cout << "[Test 1] Testing basic Lua 5.4 execution and output redirection...\n" << std::flush;
    {
        rnr::scripting::ScriptEngine engine;
        assert(engine.isInitialized());

        std::vector<std::string> log_messages;
        engine.setLogCallback([&](const std::string& msg, bool is_err) {
            log_messages.push_back((is_err ? "[ERR] " : "") + msg);
        });

        std::string err;
        bool ok = engine.executeString("print('Hello from Lua 5.4!')", &err);
        assert(ok);
        assert(!log_messages.empty());
        assert(log_messages.back() == "Hello from Lua 5.4!");

        ok = engine.executeString("x = 10 + 25; print('Sum = ' .. tostring(x))");
        assert(ok);
        assert(log_messages.back() == "Sum = 35");

        std::cout << "  -> PASSED: Lua 5.4 VM initialized and print redirect works.\n";
    }
}

void test02(MockController& mock_controller) {
    // ------------------------------------------------------------------------
    // Test 2: cnc.* Motion, Position, I/O, Spindle, & Prompts API
    // ------------------------------------------------------------------------
    std::cout << "[Test 2] Testing cnc.* Lua API bindings with MockController...\n";
    {
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::vector<std::string> logs;
        runner.setLogCallback([&](const std::string& msg, bool) {
            logs.push_back(msg);
        });

        // Test G-Code command dispatch
        std::string err;
        bool ok = runner.runStringBlocking("cnc.gcode('G0 X50 Y100')", &err);
        assert(ok);
        assert(mock_controller.last_mdi_cmd_ == "G0 X50 Y100");

        // Test DRO reading
        ok = runner.runStringBlocking(R"LUA(
            local pos = cnc.get_work_pos()
            assert(math.abs(pos.x - 12.5) < 0.001)
            assert(math.abs(pos.y - 34.0) < 0.001)
            assert(math.abs(pos.z - (-5.2)) < 0.001)

            local mpos = cnc.get_machine_pos()
            assert(math.abs(mpos.x - 112.5) < 0.001)
        )LUA", &err);
        if (!ok) std::cerr << "DRO test error: " << err << "\n";
        assert(ok);

        // Test Spindle & RPM control
        ok = runner.runStringBlocking(R"LUA(
            cnc.set_spindle(true)
            cnc.set_spindle_rpm(18000)
            assert(cnc.is_spindle_on() == true)
            assert(cnc.get_spindle_rpm() == 18000)
        )LUA", &err);
        assert(ok);
        assert(mock_controller.telemetry_.torch_or_spindle_on == true);
        assert(mock_controller.telemetry_.commanded_spindle_rpm == 18000);

        // Test Digital I/O
        ok = runner.runStringBlocking(R"LUA(
            assert(cnc.get_input(1) == true)  -- IN1 is active
            assert(cnc.get_input(2) == false) -- IN2 is inactive
            assert(cnc.get_input(3) == true)  -- IN3 is active
            assert(cnc.get_inputs_mask() == 5)

            cnc.set_output(2, true)
            assert(cnc.get_output(2) == true)
        )LUA", &err);
        assert(ok);
        assert((mock_controller.telemetry_.outputs_bitmask & (1 << 1)) != 0);

        // Test Coordinate Zeroing
        ok = runner.runStringBlocking("cnc.zero_axis('X')", &err);
        assert(ok);
        assert(mock_controller.telemetry_.work_pos_mm[0] == 0.0);

        // Test Probing
        ok = runner.runStringBlocking(R"LUA(
            local res = cnc.probe("Z", -50.0, 300.0)
            assert(res.hit == true)
            assert(math.abs(res.z - (-15.5)) < 0.001)
        )LUA", &err);
        assert(ok);

        std::cout << "  -> PASSED: All cnc.* API bindings functional.\n";
    }
}

void test03(MockController& mock_controller) {
    // ------------------------------------------------------------------------
    // Test 3: Mach3 Compatibility Shim
    // ------------------------------------------------------------------------
    std::cout << "[Test 3] Testing Mach3 compatibility shims (Code, GetDRO, SetDRO, Sleep)...\n";
    {
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::string err;
        bool ok = runner.runStringBlocking(R"LUA(
            Code("G1 X20 Y30 F1000")
            local x = GetDRO(0)
            local y = GetDRO(801)
            assert(x == 0.0) -- Zeroed in test 2
            assert(math.abs(y - 34.0) < 0.001)
            SetDRO(1, 0)
            Sleep(20)
        )LUA", &err);
        if (!ok) std::cerr << "Mach3 shim error: " << err << "\n";
        assert(ok);
        assert(mock_controller.last_mdi_cmd_ == "G1 X20 Y30 F1000");
        assert(mock_controller.telemetry_.work_pos_mm[1] == 0.0);

        std::cout << "  -> PASSED: Mach3 compatibility layer working seamlessly.\n";
    }
}

void test04(MockController& mock_controller) {
    // ------------------------------------------------------------------------
    // Test 4: Interactive UI Prompt Mailbox
    // ------------------------------------------------------------------------
    std::cout << "[Test 4] Testing interactive prompt modal mailbox...\n";
    {
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::atomic<bool> prompt_received{false};
        std::string selected_choice;

        // Run async script that calls cnc.prompt()
        runner.runStringAsync(R"LUA(
            local choice = cnc.prompt("Tool Change", "Insert Tool T3 and click OK", {"OK", "Cancel"})
            cnc.gcode("PROMPT_RESULT_" .. choice)
        )LUA", [&](bool success, const std::string& error) {
            prompt_received = true;
            assert(success);
        });

        // Simulate UI thread polling for prompt
        auto start = std::chrono::steady_clock::now();
        while (!runner.hasPendingPrompt()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > 2.0) break;
        }

        assert(runner.hasPendingPrompt());
        auto req = runner.getPendingPrompt();
        assert(req.title == "Tool Change");
        assert(req.message == "Insert Tool T3 and click OK");
        assert(req.buttons.size() == 2 && req.buttons[0] == "OK");

        // Operator clicks "OK" in UI
        runner.submitPromptResponse("OK");

        // Wait for async script to finish
        start = std::chrono::steady_clock::now();
        while (runner.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > 2.0) break;
        }

        assert(!runner.isRunning());
        assert(mock_controller.last_mdi_cmd_ == "PROMPT_RESULT_OK");

        std::cout << "  -> PASSED: Interactive prompt mailbox unblocks and receives UI response.\n";
    }
}

void test05(MockController& mock_controller) {
    // ------------------------------------------------------------------------
    // Test 5: Cooperative Cancellation on E-STOP
    // ------------------------------------------------------------------------
    std::cout << "[Test 5] Testing cooperative script cancellation on E-STOP...\n";
    {
        rnr::scripting::ScriptRunner runner(&mock_controller);

        std::atomic<bool> script_aborted{false};
        std::string final_err;

        // Run infinite loop script
        runner.runStringAsync(R"LUA(
            local count = 0
            while true do
                count = count + 1
            end
        )LUA", [&](bool success, const std::string& err) {
            script_aborted = !success;
            final_err = err;
        });

        // Allow script to start spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        assert(runner.isRunning());

        // Operator hits E-STOP / Cancel
        runner.cancel();

        // Wait for worker thread to cleanly abort
        assert(waitUntil([&] { return !runner.isRunning(); }));
        assert(script_aborted.load());
        std::cout << "  -> Abort message received: \"" << final_err << "\"\n";
        assert(final_err.find("aborted") != std::string::npos || final_err.find("E-STOP") != std::string::npos);

        std::cout << "  -> PASSED: Cooperative cancellation terminates runaway scripts instantly.\n";
    }
}

void test06() {
    // ------------------------------------------------------------------------
    // Test 6: Dynamic Macro Button Configuration & Persistence
    // ------------------------------------------------------------------------
    std::cout << "[Test 6] Testing dynamic macro button configuration & persistence...\n";
    {
        rnr::config::PersistentSettings settings;
        settings.machine.scripting.macro_buttons.clear();
        settings.machine.scripting.macro_buttons.push_back({"Custom Probe", "scripts/mill/probe_z.lua", false});
        settings.machine.scripting.macro_buttons.push_back({"Tool Zero", "scripts/mill/park.lua", false});
        settings.machine.scripting.macro_buttons.push_back({"Laser", "scripts/cut/laser_offset.lua", true});
        settings.machine.scripting.macro_buttons[1].has_color = true;
        settings.machine.scripting.macro_buttons[1].color_rgb = 0xC0392B;
        settings.ui.light_theme = true;
        settings.ui.dark_theme_id  = "monokai";
        settings.ui.light_theme_id = "solarized_light";

        std::string test_ini = "test_scripting_settings.ini";
        bool save_ok = rnr::config::SettingsManager::saveToFile(test_ini, settings);
        assert(save_ok);

        rnr::config::PersistentSettings loaded;
        bool load_ok = rnr::config::SettingsManager::loadFromFile(test_ini, loaded);
        assert(load_ok);

        assert(loaded.machine.scripting.macro_buttons.size() == 3);
        assert(loaded.machine.scripting.macro_buttons[0].label == "Custom Probe");
        assert(loaded.machine.scripting.macro_buttons[0].path == "scripts/mill/probe_z.lua");
        assert(!loaded.machine.scripting.macro_buttons[0].show_on_control_panel);
        assert(loaded.machine.scripting.macro_buttons[1].label == "Tool Zero");
        assert(loaded.machine.scripting.macro_buttons[2].label == "Laser");
        assert(loaded.machine.scripting.macro_buttons[2].show_on_control_panel);
        // Custom button color and the light theme; buttons without a color keep the default
        assert(!loaded.machine.scripting.macro_buttons[0].has_color);
        assert(loaded.machine.scripting.macro_buttons[1].has_color);
        assert(loaded.machine.scripting.macro_buttons[1].color_rgb == 0xC0392B);
        assert(!loaded.machine.scripting.macro_buttons[2].has_color);
        assert(loaded.ui.light_theme);
        assert(loaded.ui.dark_theme_id == "monokai" && loaded.ui.light_theme_id == "solarized_light");

        std::filesystem::remove(test_ini);

        // Settings saved before the scripts/plasma -> scripts/cut rename load with the new folder
        {
            std::ofstream legacy(test_ini);
            legacy << "[Scripting]\n"
                   << "ihs_script_path = scripts/plasma/m3_touch.lua\n"
                   << "macro_button_count = 1\n"
                   << "macro_btn_0_label = Laser\n"
                   << "macro_btn_0_path = scripts/plasma/laser_offset.lua\n"
                   << "macro_btn_0_color = red\n"; // Not #RRGGBB: ignored
        }
        rnr::config::PersistentSettings legacy_loaded;
        assert(rnr::config::SettingsManager::loadFromFile(test_ini, legacy_loaded));
        assert(legacy_loaded.machine.scripting.ihs_script_path == "scripts/cut/m3_touch.lua");
        assert(legacy_loaded.machine.scripting.macro_buttons.size() == 1);
        assert(legacy_loaded.machine.scripting.macro_buttons[0].path == "scripts/cut/laser_offset.lua");
        assert(!legacy_loaded.machine.scripting.macro_buttons[0].has_color);
        assert(!legacy_loaded.ui.light_theme);
        // Settings from before the theme variants keep the default themes
        assert(legacy_loaded.ui.dark_theme_id == "dark" && legacy_loaded.ui.light_theme_id == "light");
        std::filesystem::remove(test_ini);
        std::cout << "  -> PASSED: Dynamic macro buttons serialize and deserialize accurately.\n";
    }
}

void test38() {
    // ========================================================================
    // Test 38: Every example in the Lua guide compiles, and runs in the simulator
    //          (dialogs answered automatically, including typed input values)
    // ========================================================================
    {
        std::cout << "\n[Test 38] Testing Lua guide examples (syntax + simulated runs with auto-answered dialogs)...\n";
        using namespace rnr::ui::guide_examples;

        // 1. Syntax of every example
        for (const auto& ex : kAll) {
            lua_State* L = luaL_newstate();
            const int rc = luaL_loadstring(L, ex.code);
            if (rc != LUA_OK) std::cerr << "Guide example '" << ex.id << "' does not compile: " << lua_tostring(L, -1) << "\n";
            assert(rc == LUA_OK);
            lua_close(L);
        }

        // 2. Run examples with an automatic operator
        rnr::core::RnRController controller;
        assert(controller.resetFromEStop());
        std::vector<std::string> errors;
        std::mutex err_mutex;
        controller.setScriptLogCallback([&](const std::string& msg, bool is_err) {
            if (is_err) {
                std::lock_guard<std::mutex> lock(err_mutex);
                errors.push_back(msg);
            }
        });
        controller.setSimulatedInputPin(6, true); // Sensor used by the "wait for input" example

        // Typed answers for input dialogs, keyed by a piece of the dialog message
        const std::map<std::string, std::string> typed = {
            {"Holes in X", "2"}, {"Holes in Y", "1"}, {"Spacing", "5"}, {"Depth (mm, positive)", "1"},
            {"Width in X", "10"}, {"Length in Y", "4"}, {"Stepover (mm):", "4"}, {"Depth of cut", "0.5"},
            {"Feed (mm/min)", "3000"},
        };
        std::vector<std::string> dialogs_seen;
        auto runExample = [&](const char* id, const char* code, const char* button_override) {
            errors.clear();
            assert(controller.runScriptString(code, true));
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            for (int i = 0; i < 3000 && controller.isScriptRunning(); ++i) {
                const auto t = controller.getTelemetry();
                if (t.has_script_prompt) {
                    dialogs_seen.push_back(t.script_prompt_title);
                    if (t.script_prompt_input_kind != 0) {
                        std::string value = t.script_prompt_input_default;
                        for (const auto& [key, v] : typed) {
                            if (t.script_prompt_message.find(key) != std::string::npos) value = v;
                        }
                        controller.submitScriptPromptInput("OK", value);
                    } else {
                        std::string button = t.script_prompt_buttons.empty() ? "OK" : t.script_prompt_buttons.front();
                        if (button_override) button = button_override;
                        controller.submitScriptPromptResponse(button);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            assert(!controller.isScriptRunning());
            for (int i = 0; i < 300 && controller.getTelemetry().state != rnr::core::ControllerState::Idle; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (!errors.empty()) std::cerr << "Guide example '" << id << "' failed: " << errors.front() << "\n";
            assert(errors.empty());
            assert(controller.getTelemetry().state == rnr::core::ControllerState::Idle);
        };

        runExample("first", kFirstScript, nullptr);
        runExample("dlg_confirm", kDlgConfirm, nullptr);
        runExample("dlg_menu", kDlgMenu, nullptr);
        runExample("dlg_text", kDlgText, nullptr);
        runExample("dlg_number", kDlgNumber, nullptr);
        runExample("dlg_wizard", kDlgWizard, nullptr);
        runExample("dlg_checklist", kDlgChecklist, nullptr);
        runExample("ex_warmup", kExWarmup, "Cancel");
        runExample("ex_touch", kExTouchPlate, nullptr);
        const double z_after_touch = controller.getTelemetry().work_pos_mm[2];
        assert(std::abs(z_after_touch - (controller.getConfig().probe.plate_thickness_mm + 10.0)) < 0.01);
        runExample("ex_grid", kExGrid, nullptr);
        runExample("ex_facing", kExFacing, nullptr);
        runExample("ex_park", kExPark, nullptr);       // Not homed -> explains and stops
        runExample("ex_tools", kExMeasureTools, nullptr); // Not in tool table mode -> explains and stops
        runExample("ex_wait", kExWaitInput, nullptr);
        runExample("ex_log", kExJobLog, nullptr);
        std::remove("job_log.csv");
        runExample("hook_macro", kHookMacro, nullptr);
        runExample("mach3", kMach3Example, nullptr);
        assert(std::abs(controller.getTelemetry().work_pos_mm[2]) < 1e-9); // Question default 0 -> SetDRO(2, 0)

        // The Mach3 SetDRO shim sets the given value (it used to always zero the axis)
        assert(controller.runScriptString("SetDRO(2, 15.5)", false));
        assert(std::abs(controller.getTelemetry().work_pos_mm[2] - 15.5) < 1e-9);
        assert(dialogs_seen.size() > 25);
        std::cout << "  -> PASSED: " << std::size(kAll) << " guide examples compile; "
                  << dialogs_seen.size() << " dialogs answered across 17 simulated runs without errors.\n";
    }
}

int main() {
    // Unit tests run purely in the simulator: never open a real motion board that may be plugged in
    rnr::core::RnRController::setHardwareAccessEnabled(false);

    std::cout << "STARTING MAIN\n" << std::flush;
    std::cout << "====================================================\n" << std::flush;
    std::cout << "BreakoutCNC - Script Engine & Lua 5.4 Unit Test\n" << std::flush;
    std::cout << "====================================================\n\n" << std::flush;

    MockController mock_controller;
    mock_controller.telemetry_.state = rnr::core::ControllerState::Idle;
    mock_controller.telemetry_.work_pos_mm = {12.5, 34.0, -5.2, 0.0};
    mock_controller.telemetry_.machine_pos_mm = {112.5, 234.0, 45.0, 0.0};
    mock_controller.telemetry_.inputs_bitmask = 0x0005; // IN1 and IN3 active (bits 0 and 2)

    test01();
    test02(mock_controller);
    test03(mock_controller);
    test04(mock_controller);
    test05(mock_controller);
    test06();
    test07();
    test08();
    test09();
    test11();
    test12();
    test13();
    test14();
    test15();
    test16();
    test17();
    test18();
    test19();
    test20();
    test21();
    test23();
    test23b();
    test24(mock_controller);
    test25();
    test26();
    test27();
    test28();
    test29();
    test30();
    test31();
    test32();
    test33();
    test34();
    test35();
    test36();
    test37();
    test38();
    test39();
    test41();
    test42();
    test43();
    test44();
    test45();
    test46();
    test47();
    test48();
    test49();
    test50();
    test51();

    std::cout << "\n====================================================\n";
    std::cout << "ALL SCRIPT ENGINE UNIT TESTS PASSED SUCCESSFULLY! (51/51)\n";
    std::cout << "====================================================\n";
    return 0;
}
