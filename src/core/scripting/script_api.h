#pragma once
#include <string>
#include <vector>
#include <functional>
#include "core/controller/motion_controller.h"

struct lua_State;

namespace rnr::scripting {

class ScriptEngine;

using PromptHandler = std::function<std::string(
    const std::string& title,
    const std::string& message,
    const std::vector<std::string>& buttons)>;

using CancellationCheck = std::function<bool()>;

// Input dialog request (cnc.input / cnc.input_number)
struct InputRequest {
    std::string title;
    std::string message;
    bool        numeric       = false;
    std::string default_value;
    double      min_value     = -1e300;
    double      max_value     = 1e300;
};

// Returns the entered text, or false in `ok` when the operator cancelled
using InputHandler = std::function<std::string(const InputRequest& request, bool& ok)>;

class ScriptApi {
public:
    static void registerApi(
        lua_State* L,
        core::IMotionController* controller,
        ScriptEngine* engine,
        PromptHandler prompt_handler = nullptr,
        CancellationCheck cancellation_check = nullptr,
        InputHandler input_handler = nullptr);

    static void registerMach3Compat(lua_State* L);
};

} // namespace rnr::scripting
