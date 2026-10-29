#include "script_api.h"
#include "script_engine.h"
#include "script_path_helper.h"
#include <chrono>
#include <thread>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

// Lua is compiled as C++ (see CMakeLists.txt), so its headers are included without extern "C"
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

namespace rnr::scripting {

namespace {

const char* const kContextRegistryKey = "__rnr_cnc_api_context";

struct ApiContext {
    core::IMotionController* controller = nullptr;
    ScriptEngine*            engine     = nullptr;
    PromptHandler            prompt_handler;
    CancellationCheck        cancellation_check;
    InputHandler             input_handler;
};

ApiContext* getContext(lua_State* L) {
    lua_pushstring(L, kContextRegistryKey);
    lua_gettable(L, LUA_REGISTRYINDEX);
    auto* ctx = static_cast<ApiContext*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    return ctx;
}

int parseAxis(lua_State* L, int arg_idx) {
    if (lua_isinteger(L, arg_idx)) {
        int idx = static_cast<int>(lua_tointeger(L, arg_idx));
        return std::clamp(idx, 0, 3);
    }
    if (lua_isstring(L, arg_idx)) {
        const char* str = lua_tostring(L, arg_idx);
        char ch = static_cast<char>(std::toupper(str[0]));
        switch (ch) {
            case 'X': return 0;
            case 'Y': return 1;
            case 'Z': return 2;
            case 'A': return 3;
            default: break;
        }
    }
    return 0;
}

// ----------------------------------------------------------------------------
// Motion & G-Code Functions
// ----------------------------------------------------------------------------

int l_cnc_gcode(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    const char* cmd = luaL_checkstring(L, 1);
    bool ok = ctx->controller->executeMdiCommand(cmd);
    if (!ok) {
        // A rejected command must stop the script: continuing would run the rest of the macro
        // (probing, zeroing, ...) from the wrong place.
        const auto telem = ctx->controller->getTelemetry();
        std::string msg = std::string("cnc.gcode rejected: \"") + cmd + "\"";
        if (!telem.active_fault_message.empty()) {
            msg += " (" + telem.active_fault_message + ")";
        }
        return luaL_error(L, "%s", msg.c_str());
    }
    lua_pushboolean(L, ok);
    return 1;
}

int l_cnc_wait_idle(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;

    double timeout_sec = 0.0;
    if (lua_gettop(L) >= 1 && !lua_isnil(L, 1)) {
        timeout_sec = luaL_checknumber(L, 1);
    }

    // Brief sleep to allow motion thread to pick up queued slices (e.g. from cnc.gcode MDI)
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    auto start = std::chrono::steady_clock::now();
    while (true) {
        if (ctx->cancellation_check && ctx->cancellation_check()) {
            return luaL_error(L, "Script execution cancelled by operator / E-STOP");
        }

        auto telem = ctx->controller->getTelemetry();
        if (telem.state == core::ControllerState::EStop) {
            return luaL_error(L, "Script aborted: Emergency Stop is active");
        }

        bool is_moving = (telem.current_feed_mm_min > 0.1) ||
                         (telem.state == core::ControllerState::RunningGCode) ||
                         (telem.state == core::ControllerState::Jogging) ||
                         (telem.state == core::ControllerState::Homing) ||
                         (telem.state == core::ControllerState::Probing) ||
                         (telem.state == core::ControllerState::DwellPierce);
        if (!is_moving) {
            break;
        }

        if (timeout_sec > 0.0) {
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > timeout_sec) {
                return luaL_error(L, "Motion wait timed out after %.2f seconds", timeout_sec);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    lua_pushboolean(L, true);
    return 1;
}

int l_cnc_is_moving(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, false);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    bool moving = (telem.current_feed_mm_min > 0.1) ||
                  (telem.state == core::ControllerState::RunningGCode) ||
                  (telem.state == core::ControllerState::Jogging) ||
                  (telem.state == core::ControllerState::Homing) ||
                  (telem.state == core::ControllerState::Probing);
    lua_pushboolean(L, moving);
    return 1;
}

int l_cnc_sleep_ms(lua_State* L) {
    ApiContext* ctx = getContext(L);
    int ms = static_cast<int>(luaL_checkinteger(L, 1));
    if (ms <= 0) return 0;

    const int kStep = 10;
    for (int passed = 0; passed < ms; passed += kStep) {
        if (ctx && ctx->cancellation_check && ctx->cancellation_check()) {
            return luaL_error(L, "Script execution cancelled by operator / E-STOP");
        }
        int cur_sleep = std::min(kStep, ms - passed);
        std::this_thread::sleep_for(std::chrono::milliseconds(cur_sleep));
    }
    return 0;
}

int l_cnc_stop(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (ctx && ctx->controller) ctx->controller->stopJob();
    return 0;
}

int l_cnc_pause(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (ctx && ctx->controller) ctx->controller->pauseJob();
    return 0;
}

int l_cnc_resume(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (ctx && ctx->controller) ctx->controller->startJob();
    return 0;
}

// ----------------------------------------------------------------------------
// Position, Coordinates & Offsets
// ----------------------------------------------------------------------------

int l_cnc_get_work_pos(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_newtable(L);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_newtable(L);
    lua_pushnumber(L, telem.work_pos_mm[0]); lua_setfield(L, -2, "x");
    lua_pushnumber(L, telem.work_pos_mm[1]); lua_setfield(L, -2, "y");
    lua_pushnumber(L, telem.work_pos_mm[2]); lua_setfield(L, -2, "z");
    lua_pushnumber(L, telem.work_pos_mm[3]); lua_setfield(L, -2, "a");
    return 1;
}

int l_cnc_get_machine_pos(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_newtable(L);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_newtable(L);
    lua_pushnumber(L, telem.machine_pos_mm[0]); lua_setfield(L, -2, "x");
    lua_pushnumber(L, telem.machine_pos_mm[1]); lua_setfield(L, -2, "y");
    lua_pushnumber(L, telem.machine_pos_mm[2]); lua_setfield(L, -2, "z");
    lua_pushnumber(L, telem.machine_pos_mm[3]); lua_setfield(L, -2, "a");
    return 1;
}

int l_cnc_zero_axis(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    int axis = parseAxis(L, 1);
    ctx->controller->zeroWorkAxis(static_cast<core::Axis>(axis));
    return 0;
}

int l_cnc_zero_all(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (ctx && ctx->controller) ctx->controller->zeroAllWorkAxes();
    return 0;
}

int l_cnc_is_homed(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, false);
        return 1;
    }
    int axis = parseAxis(L, 1);
    auto telem = ctx->controller->getTelemetry();
    lua_pushboolean(L, telem.axis_homed[axis]);
    return 1;
}

// ----------------------------------------------------------------------------
// Spindle, Torch & Digital I/O
// ----------------------------------------------------------------------------

int l_cnc_set_spindle(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    bool enable = lua_toboolean(L, 1) != 0;
    ctx->controller->setTorchOrSpindle(enable);
    return 0;
}

int l_cnc_is_spindle_on(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, false);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushboolean(L, telem.torch_or_spindle_on);
    return 1;
}

int l_cnc_set_spindle_rpm(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    uint32_t rpm = static_cast<uint32_t>(luaL_checkinteger(L, 1));
    ctx->controller->setSpindleSpeedRpm(rpm);
    return 0;
}

int l_cnc_get_spindle_rpm(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushinteger(L, 0);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushinteger(L, telem.commanded_spindle_rpm);
    return 1;
}

int l_cnc_set_torch(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    bool enable = lua_toboolean(L, 1) != 0;
    ctx->controller->setTorchOrSpindle(enable);
    return 0;
}

int l_cnc_is_torch_on(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, false);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushboolean(L, telem.torch_or_spindle_on);
    return 1;
}

int l_cnc_get_input(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, false);
        return 1;
    }
    int pin = static_cast<int>(luaL_checkinteger(L, 1)); // 1..12
    if (pin < 1 || pin > 12) {
        lua_pushboolean(L, false);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    bool active = (telem.inputs_bitmask & (1 << (pin - 1))) != 0;
    lua_pushboolean(L, active);
    return 1;
}

int l_cnc_get_inputs_mask(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushinteger(L, 0);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushinteger(L, telem.inputs_bitmask);
    return 1;
}

int l_cnc_set_output(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    int out_idx = static_cast<int>(luaL_checkinteger(L, 1)); // 1..4
    bool enable = lua_toboolean(L, 2) != 0;
    if (out_idx >= 1 && out_idx <= 4) {
        ctx->controller->setAuxOutput(static_cast<uint8_t>(out_idx), enable);
    }
    return 0;
}

int l_cnc_get_output(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, false);
        return 1;
    }
    int out_idx = static_cast<int>(luaL_checkinteger(L, 1)); // 1..4
    auto telem = ctx->controller->getTelemetry();
    bool active = (out_idx >= 1 && out_idx <= 4) && ((telem.outputs_bitmask & (1 << (out_idx - 1))) != 0);
    lua_pushboolean(L, active);
    return 1;
}

// ----------------------------------------------------------------------------
// Probing (G31)
// ----------------------------------------------------------------------------

int l_cnc_probe(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;

    int axis = parseAxis(L, 1);
    double max_travel_mm = luaL_checknumber(L, 2); // Signed search distance (e.g. -20 = probe toward -)
    double feed_mm_min = luaL_checknumber(L, 3);

    // G38.2-style raw probe: stops on contact, no offset changes and no automatic retract
    ctx->controller->probeRaw(static_cast<core::Axis>(axis), max_travel_mm, feed_mm_min);

    // Wait for probing move to complete (controller leaves the Probing state)
    while (true) {
        if (ctx->cancellation_check && ctx->cancellation_check()) {
            return luaL_error(L, "Probe operation cancelled by operator / E-STOP");
        }
        auto telem = ctx->controller->getTelemetry();
        if (telem.state == core::ControllerState::EStop) {
            return luaL_error(L, "Probe aborted: Emergency Stop active");
        }
        if (telem.state != core::ControllerState::Probing) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto telem = ctx->controller->getTelemetry();
    lua_newtable(L);
    lua_pushboolean(L, telem.last_probe_hit); lua_setfield(L, -2, "hit");
    lua_pushnumber(L, telem.last_probed_work_pos_mm[0]); lua_setfield(L, -2, "x");
    lua_pushnumber(L, telem.last_probed_work_pos_mm[1]); lua_setfield(L, -2, "y");
    lua_pushnumber(L, telem.last_probed_work_pos_mm[2]); lua_setfield(L, -2, "z");
    lua_pushnumber(L, telem.last_probed_work_pos_mm[3]); lua_setfield(L, -2, "a");
    return 1;
}

// cnc.set_work_pos(axis, value): G92-style - current position of `axis` becomes `value` in work coordinates
int l_cnc_set_work_pos(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    int axis = parseAxis(L, 1);
    double value = luaL_checknumber(L, 2);
    ctx->controller->setWorkPosition(static_cast<core::Axis>(axis), value);
    return 0;
}

// cnc.get_probe_config(): returns the operator's touch-probe settings from the UI
int l_cnc_get_probe_config(lua_State* L) {
    ApiContext* ctx = getContext(L);
    lua_newtable(L);
    if (!ctx || !ctx->controller) return 1;
    const auto cfg = ctx->controller->getConfig(); // Copy: getConfig() returns by value
    const auto& p = cfg.probe;
    lua_pushnumber(L, p.tool_diameter_mm);    lua_setfield(L, -2, "tip_diameter");
    lua_pushnumber(L, p.plate_thickness_mm);  lua_setfield(L, -2, "plate_thickness");
    lua_pushnumber(L, p.fast_feed_mm_min);    lua_setfield(L, -2, "fast_feed");
    lua_pushnumber(L, p.slow_feed_mm_min);    lua_setfield(L, -2, "slow_feed");
    lua_pushnumber(L, p.max_travel_mm);       lua_setfield(L, -2, "max_travel");
    lua_pushnumber(L, p.retract_mm);          lua_setfield(L, -2, "retract");
    lua_pushnumber(L, p.approx_hole_diam_mm); lua_setfield(L, -2, "diameter");
    lua_pushnumber(L, p.xy_clearance_mm);     lua_setfield(L, -2, "xy_clearance");
    lua_pushnumber(L, p.probe_depth_mm);      lua_setfield(L, -2, "probe_depth");
    lua_pushnumber(L, p.approx_size_x_mm);    lua_setfield(L, -2, "size_x");
    lua_pushnumber(L, p.approx_size_y_mm);    lua_setfield(L, -2, "size_y");
    return 1;
}

// cnc.get_tool_change_config(): returns M6 tool change settings (move_to_park_xy, park_x, park_y, safe_z, mode)
int l_cnc_get_tool_change_config(lua_State* L) {
    ApiContext* ctx = getContext(L);
    lua_newtable(L);
    if (!ctx || !ctx->controller) return 1;
    const auto cfg = ctx->controller->getConfig(); // Copy: getConfig() returns by value
    const auto& tc = cfg.tool_change;
    lua_pushboolean(L, tc.move_to_park_xy); lua_setfield(L, -2, "move_to_park_xy");
    lua_pushnumber(L, tc.park_x_mm);        lua_setfield(L, -2, "park_x");
    lua_pushnumber(L, tc.park_y_mm);        lua_setfield(L, -2, "park_y");
    lua_pushnumber(L, tc.safe_z_mm);        lua_setfield(L, -2, "safe_z");
    lua_pushinteger(L, static_cast<int>(tc.mode)); lua_setfield(L, -2, "mode");
    return 1;
}

// cnc.get_tool_length_mode(): "not_configured" | "touch_off" | "tool_table"
int l_cnc_get_tool_length_mode(lua_State* L) {
    ApiContext* ctx = getContext(L);
    const char* name = "not_configured";
    if (ctx && ctx->controller) {
        switch (ctx->controller->getConfig().tool_table.mode) {
            case core::ToolLengthMode::TouchOffEachTool: name = "touch_off"; break;
            case core::ToolLengthMode::ToolTable:        name = "tool_table"; break;
            default: break;
        }
    }
    lua_pushstring(L, name);
    return 1;
}

// cnc.get_tool_length(tool): length (mm, relative to the reference tool) or nil if not in the tool table
int l_cnc_get_tool_length(lua_State* L) {
    ApiContext* ctx = getContext(L);
    const auto tool = static_cast<uint16_t>(luaL_checkinteger(L, 1));
    if (!ctx || !ctx->controller) {
        lua_pushnil(L);
        return 1;
    }
    const auto cfg = ctx->controller->getConfig();
    const auto it = cfg.tool_table.tools.find(tool);
    if (it == cfg.tool_table.tools.end()) {
        lua_pushnil(L);
    } else {
        lua_pushnumber(L, it->second.length_mm);
    }
    return 1;
}

// cnc.measure_tool_length(tool): measures the tool on the tool setter below the current position and
// stores its length in the tool table. Returns the length, or raises an error if it failed.
int l_cnc_measure_tool_length(lua_State* L) {
    ApiContext* ctx = getContext(L);
    const auto tool = static_cast<uint16_t>(luaL_checkinteger(L, 1));
    if (!ctx || !ctx->controller) return 0;

    const auto before = ctx->controller->getConfig();
    ctx->controller->measureToolLength(tool);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    while (true) {
        if (ctx->cancellation_check && ctx->cancellation_check()) {
            return luaL_error(L, "Tool measuring cancelled by operator / E-STOP");
        }
        const auto telem = ctx->controller->getTelemetry();
        if (telem.state == core::ControllerState::EStop) {
            return luaL_error(L, "Tool measuring aborted: Emergency Stop active");
        }
        if (telem.state != core::ControllerState::Probing && telem.state != core::ControllerState::Jogging) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto telem = ctx->controller->getTelemetry();
    const auto after = ctx->controller->getConfig();
    const auto it = after.tool_table.tools.find(tool);
    if (!telem.last_probe_hit || it == after.tool_table.tools.end() ||
        after.tool_table_revision == before.tool_table_revision) {
        const std::string why = telem.active_fault_message.empty() ? "no probe contact" : telem.active_fault_message;
        return luaL_error(L, "Tool T%d measuring failed: %s", static_cast<int>(tool), why.c_str());
    }
    lua_pushnumber(L, it->second.length_mm);
    return 1;
}

// cnc.resolve_path(relative_path): absolute path of a script/data file (or nil if not found)
int l_cnc_resolve_path(lua_State* L) {
    const char* rel = luaL_checkstring(L, 1);
    std::string resolved = resolveScriptPath(rel);
    if (resolved.empty()) {
        lua_pushnil(L);
    } else {
        lua_pushstring(L, resolved.c_str());
    }
    return 1;
}

int l_cnc_is_probe_tripped(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, false);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushboolean(L, telem.probe_contact_active);
    return 1;
}

int l_cnc_get_last_probe_pos(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_newtable(L);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_newtable(L);
    lua_pushnumber(L, telem.last_probed_work_pos_mm[0]); lua_setfield(L, -2, "x");
    lua_pushnumber(L, telem.last_probed_work_pos_mm[1]); lua_setfield(L, -2, "y");
    lua_pushnumber(L, telem.last_probed_work_pos_mm[2]); lua_setfield(L, -2, "z");
    lua_pushnumber(L, telem.last_probed_work_pos_mm[3]); lua_setfield(L, -2, "a");
    return 1;
}

// ----------------------------------------------------------------------------
// Tooling & Machine Modals
// ----------------------------------------------------------------------------

int l_cnc_get_current_tool(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushinteger(L, 1);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushinteger(L, telem.current_tool_number);
    return 1;
}

int l_cnc_get_pending_tool(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushinteger(L, 0);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushinteger(L, telem.pending_tool_number);
    return 1;
}

int l_cnc_get_feed_override(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushnumber(L, 100.0);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushnumber(L, telem.feed_override_pct);
    return 1;
}

int l_cnc_set_feed_override(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    float pct = static_cast<float>(luaL_checknumber(L, 1));
    ctx->controller->setFeedOverride(pct);
    return 0;
}

// ----------------------------------------------------------------------------
// Safety, E-Stop & Prompts
// ----------------------------------------------------------------------------

int l_cnc_estop(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) return 0;
    ctx->controller->triggerEStop();
    if (lua_gettop(L) >= 1 && lua_isstring(L, 1)) {
        const char* reason = lua_tostring(L, 1);
        if (ctx->engine) ctx->engine->log(std::string("E-STOP: ") + reason, true);
    }
    return 0;
}

int l_cnc_is_estop(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, true);
        return 1;
    }
    auto telem = ctx->controller->getTelemetry();
    lua_pushboolean(L, telem.state == core::ControllerState::EStop);
    return 1;
}

int l_cnc_reset_estop(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->controller) {
        lua_pushboolean(L, false);
        return 1;
    }
    bool ok = ctx->controller->resetFromEStop();
    lua_pushboolean(L, ok);
    return 1;
}

int l_cnc_log(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->engine) return 0;
    const char* msg = luaL_checkstring(L, 1);
    ctx->engine->log(msg, false);
    return 0;
}

int l_cnc_prompt(lua_State* L) {
    ApiContext* ctx = getContext(L);
    if (!ctx || !ctx->prompt_handler) {
        lua_pushstring(L, "OK");
        return 1;
    }

    std::string title = luaL_checkstring(L, 1);
    std::string message = luaL_checkstring(L, 2);
    std::vector<std::string> buttons;

    if (lua_gettop(L) >= 3 && lua_istable(L, 3)) {
        int len = static_cast<int>(lua_rawlen(L, 3));
        for (int i = 1; i <= len; ++i) {
            lua_rawgeti(L, 3, i);
            if (lua_isstring(L, -1)) {
                buttons.push_back(lua_tostring(L, -1));
            }
            lua_pop(L, 1);
        }
    }
    if (buttons.empty()) {
        buttons = {"OK", "Cancel"};
    }

    std::string res = ctx->prompt_handler(title, message, buttons);
    lua_pushstring(L, res.c_str());
    return 1;
}

// cnc.input(title, message [, default]) -> entered text, or nil if the operator pressed Cancel
int l_cnc_input(lua_State* L) {
    ApiContext* ctx = getContext(L);
    InputRequest req;
    req.title   = luaL_checkstring(L, 1);
    req.message = luaL_checkstring(L, 2);
    if (lua_gettop(L) >= 3 && !lua_isnil(L, 3)) {
        req.default_value = luaL_checkstring(L, 3);
    }
    if (!ctx || !ctx->input_handler) {
        lua_pushstring(L, req.default_value.c_str()); // Headless: accept the default
        return 1;
    }
    bool ok = false;
    const std::string text = ctx->input_handler(req, ok);
    if (!ok) {
        lua_pushnil(L);
    } else {
        lua_pushstring(L, text.c_str());
    }
    return 1;
}

// cnc.input_number(title, message [, default [, min [, max]]]) -> number, or nil if cancelled.
// The dialog only accepts a value inside [min, max].
int l_cnc_input_number(lua_State* L) {
    ApiContext* ctx = getContext(L);
    InputRequest req;
    req.numeric = true;
    req.title   = luaL_checkstring(L, 1);
    req.message = luaL_checkstring(L, 2);
    double def = 0.0;
    if (lua_gettop(L) >= 3 && !lua_isnil(L, 3)) def = luaL_checknumber(L, 3);
    if (lua_gettop(L) >= 4 && !lua_isnil(L, 4)) req.min_value = luaL_checknumber(L, 4);
    if (lua_gettop(L) >= 5 && !lua_isnil(L, 5)) req.max_value = luaL_checknumber(L, 5);
    if (req.min_value > req.max_value) {
        return luaL_error(L, "cnc.input_number: min (%f) is greater than max (%f)", req.min_value, req.max_value);
    }
    def = std::clamp(def, req.min_value, req.max_value);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%g", def);
    req.default_value = buf;
    if (!ctx || !ctx->input_handler) {
        lua_pushnumber(L, def);
        return 1;
    }
    bool ok = false;
    const std::string text = ctx->input_handler(req, ok);
    if (!ok) {
        lua_pushnil(L);
        return 1;
    }
    char* end = nullptr;
    const double v = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || !std::isfinite(v)) {
        lua_pushnil(L);
        return 1;
    }
    lua_pushnumber(L, std::clamp(v, req.min_value, req.max_value));
    return 1;
}

} // namespace

void ScriptApi::registerApi(
    lua_State* L,
    core::IMotionController* controller,
    ScriptEngine* engine,
    PromptHandler prompt_handler,
    CancellationCheck cancellation_check,
    InputHandler input_handler)
{
    if (!L) return;

    // Allocate ApiContext and store in Lua registry
    auto* ctx = new ApiContext{
        controller,
        engine,
        std::move(prompt_handler),
        std::move(cancellation_check),
        std::move(input_handler)
    };

    lua_pushstring(L, kContextRegistryKey);
    lua_pushlightuserdata(L, ctx);
    lua_settable(L, LUA_REGISTRYINDEX);

    // Build the 'cnc' table
    lua_newtable(L);

    // Motion & G-Code
    lua_pushcfunction(L, l_cnc_gcode);           lua_setfield(L, -2, "gcode");
    lua_pushcfunction(L, l_cnc_wait_idle);       lua_setfield(L, -2, "wait_idle");
    lua_pushcfunction(L, l_cnc_is_moving);       lua_setfield(L, -2, "is_moving");
    lua_pushcfunction(L, l_cnc_sleep_ms);        lua_setfield(L, -2, "sleep_ms");
    lua_pushcfunction(L, l_cnc_stop);            lua_setfield(L, -2, "stop");
    lua_pushcfunction(L, l_cnc_pause);           lua_setfield(L, -2, "pause");
    lua_pushcfunction(L, l_cnc_resume);          lua_setfield(L, -2, "resume");

    // Positions & DROs
    lua_pushcfunction(L, l_cnc_get_work_pos);    lua_setfield(L, -2, "get_work_pos");
    lua_pushcfunction(L, l_cnc_get_machine_pos); lua_setfield(L, -2, "get_machine_pos");
    lua_pushcfunction(L, l_cnc_zero_axis);       lua_setfield(L, -2, "zero_axis");
    lua_pushcfunction(L, l_cnc_zero_all);        lua_setfield(L, -2, "zero_all");
    lua_pushcfunction(L, l_cnc_is_homed);        lua_setfield(L, -2, "is_homed");

    // Spindle & Torch
    lua_pushcfunction(L, l_cnc_set_spindle);     lua_setfield(L, -2, "set_spindle");
    lua_pushcfunction(L, l_cnc_is_spindle_on);   lua_setfield(L, -2, "is_spindle_on");
    lua_pushcfunction(L, l_cnc_set_spindle_rpm); lua_setfield(L, -2, "set_spindle_rpm");
    lua_pushcfunction(L, l_cnc_get_spindle_rpm); lua_setfield(L, -2, "get_spindle_rpm");
    lua_pushcfunction(L, l_cnc_set_torch);       lua_setfield(L, -2, "set_torch");
    lua_pushcfunction(L, l_cnc_is_torch_on);     lua_setfield(L, -2, "is_torch_on");

    // Digital I/O
    lua_pushcfunction(L, l_cnc_get_input);       lua_setfield(L, -2, "get_input");
    lua_pushcfunction(L, l_cnc_get_inputs_mask); lua_setfield(L, -2, "get_inputs_mask");
    lua_pushcfunction(L, l_cnc_set_output);      lua_setfield(L, -2, "set_output");
    lua_pushcfunction(L, l_cnc_get_output);      lua_setfield(L, -2, "get_output");

    // Probing
    lua_pushcfunction(L, l_cnc_probe);           lua_setfield(L, -2, "probe");
    lua_pushcfunction(L, l_cnc_is_probe_tripped);lua_setfield(L, -2, "is_probe_tripped");
    lua_pushcfunction(L, l_cnc_get_last_probe_pos); lua_setfield(L, -2, "get_last_probe_pos");
    lua_pushcfunction(L, l_cnc_set_work_pos);    lua_setfield(L, -2, "set_work_pos");
    lua_pushcfunction(L, l_cnc_get_probe_config);lua_setfield(L, -2, "get_probe_config");
    lua_pushcfunction(L, l_cnc_get_tool_change_config); lua_setfield(L, -2, "get_tool_change_config");
    lua_pushcfunction(L, l_cnc_get_tool_length_mode); lua_setfield(L, -2, "get_tool_length_mode");
    lua_pushcfunction(L, l_cnc_get_tool_length);     lua_setfield(L, -2, "get_tool_length");
    lua_pushcfunction(L, l_cnc_measure_tool_length); lua_setfield(L, -2, "measure_tool_length");
    lua_pushcfunction(L, l_cnc_resolve_path);    lua_setfield(L, -2, "resolve_path");

    // Tooling & Overrides
    lua_pushcfunction(L, l_cnc_get_current_tool);lua_setfield(L, -2, "get_current_tool");
    lua_pushcfunction(L, l_cnc_get_pending_tool);lua_setfield(L, -2, "get_pending_tool");
    lua_pushcfunction(L, l_cnc_get_feed_override);lua_setfield(L, -2, "get_feed_override");
    lua_pushcfunction(L, l_cnc_set_feed_override);lua_setfield(L, -2, "set_feed_override");

    // Safety & Prompts
    lua_pushcfunction(L, l_cnc_estop);           lua_setfield(L, -2, "estop");
    lua_pushcfunction(L, l_cnc_is_estop);        lua_setfield(L, -2, "is_estop");
    lua_pushcfunction(L, l_cnc_reset_estop);     lua_setfield(L, -2, "reset_estop");
    lua_pushcfunction(L, l_cnc_log);             lua_setfield(L, -2, "log");
    lua_pushcfunction(L, l_cnc_prompt);          lua_setfield(L, -2, "prompt");
    lua_pushcfunction(L, l_cnc_input);           lua_setfield(L, -2, "input");
    lua_pushcfunction(L, l_cnc_input_number);    lua_setfield(L, -2, "input_number");

    lua_setglobal(L, "cnc");

    // Register Mach3 Compatibility global functions
    registerMach3Compat(L);
}

void ScriptApi::registerMach3Compat(lua_State* L) {
    if (!L) return;

    // Direct Mach3 Shims: Code(), WhileIsMoving(), Sleep(), Message(), etc.
    const char* kMach3ShimScript = R"LUA(
        function Code(str)
            return cnc.gcode(str)
        end

        function WhileIsMoving()
            cnc.wait_idle()
        end

        function Sleep(ms)
            cnc.sleep_ms(ms)
        end

        function IsMoving()
            return cnc.is_moving()
        end

        function Message(str)
            cnc.log(str)
        end

        function GetDRO(axis_id)
            local pos = cnc.get_work_pos()
            if axis_id == 0 or axis_id == 800 then return pos.x
            elseif axis_id == 1 or axis_id == 801 then return pos.y
            elseif axis_id == 2 or axis_id == 802 then return pos.z
            elseif axis_id == 3 or axis_id == 803 then return pos.a
            end
            return 0.0
        end

        -- Sets the work DRO of an axis to `val` (current position reads val), like Mach3 SetDRO
        function SetDRO(axis_id, val)
            local v = tonumber(val) or 0.0
            if axis_id == 0 or axis_id == 800 then cnc.set_work_pos("X", v)
            elseif axis_id == 1 or axis_id == 801 then cnc.set_work_pos("Y", v)
            elseif axis_id == 2 or axis_id == 802 then cnc.set_work_pos("Z", v)
            elseif axis_id == 3 or axis_id == 803 then cnc.set_work_pos("A", v)
            end
        end

        -- Mach3 dialogs
        function MsgBox(str)
            cnc.prompt("Message", tostring(str), {"OK"})
        end

        function Question(str)
            return cnc.input_number("Question", tostring(str), 0)
        end

        function GetUserText(str)
            return cnc.input("Input", tostring(str), "")
        end

        function ActivateSignal(out_pin)
            cnc.set_output(out_pin, true)
        end

        function DeActivateSignal(out_pin)
            cnc.set_output(out_pin, false)
        end

        function IsProbing()
            return cnc.is_probe_tripped()
        end
    )LUA";

    luaL_dostring(L, kMach3ShimScript);
}

} // namespace rnr::scripting
