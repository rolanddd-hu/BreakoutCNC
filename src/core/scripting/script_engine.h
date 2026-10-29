#pragma once
#include <string>
#include <functional>
#include <mutex>

struct lua_State;

namespace rnr::scripting {

using LogCallback = std::function<void(const std::string& message, bool is_error)>;

class ScriptEngine {
public:
    ScriptEngine();
    ~ScriptEngine();

    // Non-copyable
    ScriptEngine(const ScriptEngine&) = delete;
    ScriptEngine& operator=(const ScriptEngine&) = delete;

    // Initialize or reset Lua VM state
    bool initialize();
    void shutdown();
    bool isInitialized() const { return L_ != nullptr; }

    // Execute script code or file directly on current thread
    bool executeString(const std::string& code, std::string* out_error = nullptr);
    bool executeFile(const std::string& filepath, std::string* out_error = nullptr);

    // Call a specific global Lua function by name
    bool callFunction(const std::string& func_name, std::string* out_error = nullptr);

    // Logging & Output redirect (Lua 'print' and 'cnc.log')
    void setLogCallback(LogCallback callback);
    void log(const std::string& message, bool is_error = false);

    // Raw Lua State access (for registering C functions & modules)
    lua_State* getLuaState() const { return L_; }

    // Instruction count hook (used by ScriptRunner for cooperative cancellation)
    void clearHook();

private:
    void registerBaseOverrides();

    lua_State*            L_ = nullptr;
    LogCallback           log_callback_;
    std::mutex            log_mutex_;
    mutable std::recursive_mutex mutex_;
};

} // namespace rnr::scripting
