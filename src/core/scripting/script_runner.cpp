#include "script_runner.h"
#include <iostream>

// Lua is compiled as C++ (see CMakeLists.txt), so its headers are included without extern "C"
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

namespace rnr::scripting {

namespace {

const char* const kRunnerRegistryKey = "__rnr_script_runner";

void luaCancellationHook(lua_State* L, lua_Debug* /*ar*/) {
    lua_pushstring(L, kRunnerRegistryKey);
    lua_gettable(L, LUA_REGISTRYINDEX);
    auto* runner = static_cast<ScriptRunner*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    if (runner && runner->isCancellationRequested()) {
        luaL_error(L, "Script execution aborted by operator / E-STOP");
    }
}

} // namespace

ScriptRunner::ScriptRunner(core::IMotionController* controller)
    : controller_(controller)
{
    initLuaEnvironment();
}

ScriptRunner::~ScriptRunner() {
    cancel();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    engine_.shutdown();
}

void ScriptRunner::setController(core::IMotionController* controller) {
    std::lock_guard<std::mutex> lock(runner_mutex_);
    controller_ = controller;
    initLuaEnvironment();
}

void ScriptRunner::initLuaEnvironment() {
    engine_.initialize();
    lua_State* L = engine_.getLuaState();
    if (!L) return;

    // Store runner pointer for cancellation hook
    lua_pushstring(L, kRunnerRegistryKey);
    lua_pushlightuserdata(L, this);
    lua_settable(L, LUA_REGISTRYINDEX);

    // Register cnc.* API and Mach3 shims
    ScriptApi::registerApi(
        L,
        controller_,
        &engine_,
        [this](const std::string& title, const std::string& message, const std::vector<std::string>& buttons) {
            return handlePromptRequest(title, message, buttons);
        },
        [this]() {
            return isCancellationRequested();
        },
        [this](const InputRequest& req, bool& ok) {
            return handleInputRequest(req, ok);
        }
    );
}

void ScriptRunner::cancel() {
    cancel_requested_ = true;

    // Unblock any pending prompt modal wait
    {
        std::lock_guard<std::mutex> lock(prompt_mutex_);
        if (has_pending_prompt_) {
            prompt_response_ = "Cancel";
            has_pending_prompt_ = false;
            prompt_cv_.notify_all();
        }
    }
}

std::string ScriptRunner::getActiveScriptName() const {
    std::lock_guard<std::mutex> lock(name_mutex_);
    return active_script_name_;
}

bool ScriptRunner::executeClaimed(bool is_file, const std::string& source, std::string* out_error) {
    // Precondition: is_running_ was set to true by the caller (atomically claimed)
    cancel_requested_ = false;
    {
        std::lock_guard<std::mutex> lock(name_mutex_);
        active_script_name_ = is_file ? source : std::string("<inline_script>");
    }

    lua_State* L = engine_.getLuaState();
    if (L) {
        lua_sethook(L, luaCancellationHook, LUA_MASKCOUNT, 100);
    }

    const bool ok = is_file ? engine_.executeFile(source, out_error) : engine_.executeString(source, out_error);

    if (L) {
        lua_sethook(L, nullptr, 0, 0);
    }

    is_running_ = false;
    return ok;
}

bool ScriptRunner::runStringBlocking(const std::string& code, std::string* out_error) {
    if (is_running_.exchange(true)) {
        if (out_error) *out_error = "Another script is already running";
        return false;
    }
    return executeClaimed(false, code, out_error);
}

bool ScriptRunner::runFileBlocking(const std::string& filepath, std::string* out_error) {
    if (is_running_.exchange(true)) {
        if (out_error) *out_error = "Another script is already running";
        return false;
    }
    return executeClaimed(true, filepath, out_error);
}

bool ScriptRunner::startAsync(bool is_file, const std::string& source, ScriptCompletionCallback on_complete) {
    std::string reject;
    {
        std::lock_guard<std::mutex> lock(runner_mutex_);
        // Claim the runner atomically so two quick requests can never both start a script
        if (is_running_.exchange(true)) {
            reject = "Another script is already running";
        } else if (worker_thread_.joinable()) {
            if (thread_done_.load()) {
                worker_thread_.join(); // Finished thread: returns immediately
            } else {
                // The previous script is still inside its completion callback. Never block here: callers
                // may hold the motion controller lock that the callback is waiting for.
                is_running_ = false;
                reject = "Previous script is still finishing";
            }
        }
        if (reject.empty()) {
            thread_done_ = false;
            worker_thread_ = std::thread([this, is_file, source, on_complete]() {
                std::string err;
                const bool ok = executeClaimed(is_file, source, &err);
                if (on_complete) {
                    on_complete(ok, err);
                }
                thread_done_ = true;
            });
            return true;
        }
    }
    if (on_complete) on_complete(false, reject);
    return false;
}

bool ScriptRunner::runStringAsync(const std::string& code, ScriptCompletionCallback on_complete) {
    return startAsync(false, code, std::move(on_complete));
}

bool ScriptRunner::runFileAsync(const std::string& filepath, ScriptCompletionCallback on_complete) {
    return startAsync(true, filepath, std::move(on_complete));
}

std::string ScriptRunner::handlePromptRequest(
    const std::string& title,
    const std::string& message,
    const std::vector<std::string>& buttons)
{
    std::unique_lock<std::mutex> lock(prompt_mutex_);
    pending_prompt_ = PromptRequest{title, message, buttons};
    has_pending_prompt_ = true;
    prompt_response_.clear();

    prompt_cv_.wait(lock, [this]() {
        return !has_pending_prompt_ || cancel_requested_.load();
    });

    if (cancel_requested_.load()) {
        return "Cancel";
    }

    return prompt_response_.empty() ? "Cancel" : prompt_response_;
}

std::string ScriptRunner::handleInputRequest(const InputRequest& req, bool& ok) {
    std::unique_lock<std::mutex> lock(prompt_mutex_);
    PromptRequest p;
    p.title         = req.title;
    p.message       = req.message;
    p.buttons       = {"OK", "Cancel"};
    p.input_kind    = req.numeric ? PromptInputKind::Number : PromptInputKind::Text;
    p.input_default = req.default_value;
    p.input_min     = req.min_value;
    p.input_max     = req.max_value;
    pending_prompt_ = p;
    has_pending_prompt_ = true;
    prompt_response_.clear();
    prompt_input_.clear();

    prompt_cv_.wait(lock, [this]() {
        return !has_pending_prompt_ || cancel_requested_.load();
    });

    ok = !cancel_requested_.load() && prompt_response_ == "OK";
    return ok ? prompt_input_ : std::string();
}

bool ScriptRunner::hasPendingPrompt() const {
    std::lock_guard<std::mutex> lock(prompt_mutex_);
    return has_pending_prompt_;
}

PromptRequest ScriptRunner::getPendingPrompt() const {
    std::lock_guard<std::mutex> lock(prompt_mutex_);
    return pending_prompt_;
}

void ScriptRunner::submitPromptResponse(const std::string& response, const std::string& input) {
    std::lock_guard<std::mutex> lock(prompt_mutex_);
    if (has_pending_prompt_) {
        prompt_response_ = response;
        prompt_input_    = input;
        has_pending_prompt_ = false;
        prompt_cv_.notify_all();
    }
}

} // namespace rnr::scripting
