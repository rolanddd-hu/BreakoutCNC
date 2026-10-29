#include "script_engine.h"
#include <iostream>
#include <sstream>

// Lua is compiled as C++ (see CMakeLists.txt), so its headers are included without extern "C"
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

namespace rnr::scripting {

namespace {

// Helper: Custom Lua 'print' replacement that directs output to ScriptEngine::log()
int luaCustomPrint(lua_State* L) {
    auto* engine = static_cast<ScriptEngine*>(lua_touserdata(L, lua_upvalueindex(1)));
    if (!engine) return 0;

    int nargs = lua_gettop(L);
    std::ostringstream ss;
    for (int i = 1; i <= nargs; ++i) {
        if (i > 1) ss << "\t";
        if (lua_isstring(L, i)) {
            ss << lua_tostring(L, i);
        } else if (lua_isboolean(L, i)) {
            ss << (lua_toboolean(L, i) ? "true" : "false");
        } else if (lua_isnil(L, i)) {
            ss << "nil";
        } else {
            ss << luaL_typename(L, i) << ": " << lua_topointer(L, i);
        }
    }

    engine->log(ss.str(), false);
    return 0;
}

} // namespace

ScriptEngine::ScriptEngine() {
    initialize();
}

ScriptEngine::~ScriptEngine() {
    shutdown();
}

bool ScriptEngine::initialize() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (L_) {
        lua_close(L_);
        L_ = nullptr;
    }

    L_ = luaL_newstate();
    if (!L_) return false;

    luaL_openlibs(L_);
    // Sandbox: machine scripts must not spawn processes, delete/rename files, exit the controller
    // process or load native code.
    static const char* const kSandbox =
        "os.execute = nil; os.exit = nil; os.remove = nil; os.rename = nil; os.tmpname = nil\n"
        "io.popen = nil\n"
        "package.loadlib = nil; package.cpath = ''\n"
        "if package.searchers then package.searchers[3] = nil; package.searchers[4] = nil end\n";
    if (luaL_dostring(L_, kSandbox) != LUA_OK) {
        lua_pop(L_, 1);
    }

    // Register custom print() override
    registerBaseOverrides();

    return true;
}

void ScriptEngine::shutdown() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (L_) {
        lua_close(L_);
        L_ = nullptr;
    }
}

void ScriptEngine::registerBaseOverrides() {
    if (!L_) return;

    // Push ScriptEngine pointer as upvalue to luaCustomPrint
    lua_pushlightuserdata(L_, this);
    lua_pushcclosure(L_, luaCustomPrint, 1);
    lua_setglobal(L_, "print");
}

void ScriptEngine::setLogCallback(LogCallback callback) {
    // Separate lock: the VM lock is held for the whole duration of a running script
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_callback_ = std::move(callback);
}

void ScriptEngine::log(const std::string& message, bool is_error) {
    LogCallback cb;
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        cb = log_callback_;
    }
    if (cb) {
        cb(message, is_error);
    } else {
        if (is_error) {
            std::cerr << "[Script Error] " << message << std::endl;
        } else {
            std::cout << "[Script] " << message << std::endl;
        }
    }
}

bool ScriptEngine::executeString(const std::string& code, std::string* out_error) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!L_) {
        if (out_error) *out_error = "Lua VM not initialized";
        return false;
    }

    int load_res = luaL_loadbuffer(L_, code.c_str(), code.size(), "string_chunk");
    if (load_res != LUA_OK) {
        const char* err_str = lua_tostring(L_, -1);
        std::string err = err_str ? err_str : "Unknown compilation error";
        lua_pop(L_, 1);
        log(err, true);
        if (out_error) *out_error = err;
        return false;
    }

    int call_res = lua_pcall(L_, 0, LUA_MULTRET, 0);
    if (call_res != LUA_OK) {
        const char* err_str = lua_tostring(L_, -1);
        std::string err = err_str ? err_str : "Unknown runtime error";
        lua_pop(L_, 1);
        log(err, true);
        if (out_error) *out_error = err;
        return false;
    }

    return true;
}

bool ScriptEngine::executeFile(const std::string& filepath, std::string* out_error) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!L_) {
        if (out_error) *out_error = "Lua VM not initialized";
        return false;
    }

    int load_res = luaL_loadfile(L_, filepath.c_str());
    if (load_res != LUA_OK) {
        const char* err_str = lua_tostring(L_, -1);
        std::string err = err_str ? err_str : "Failed to load script file";
        lua_pop(L_, 1);
        log(err, true);
        if (out_error) *out_error = err;
        return false;
    }

    int call_res = lua_pcall(L_, 0, LUA_MULTRET, 0);
    if (call_res != LUA_OK) {
        const char* err_str = lua_tostring(L_, -1);
        std::string err = err_str ? err_str : "Runtime error in script file";
        lua_pop(L_, 1);
        log(err, true);
        if (out_error) *out_error = err;
        return false;
    }

    return true;
}

bool ScriptEngine::callFunction(const std::string& func_name, std::string* out_error) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!L_) {
        if (out_error) *out_error = "Lua VM not initialized";
        return false;
    }

    lua_getglobal(L_, func_name.c_str());
    if (!lua_isfunction(L_, -1)) {
        lua_pop(L_, 1);
        if (out_error) *out_error = "Global symbol '" + func_name + "' is not a function";
        return false;
    }

    int call_res = lua_pcall(L_, 0, 0, 0);
    if (call_res != LUA_OK) {
        const char* err_str = lua_tostring(L_, -1);
        std::string err = err_str ? err_str : "Error executing function " + func_name;
        lua_pop(L_, 1);
        log(err, true);
        if (out_error) *out_error = err;
        return false;
    }

    return true;
}

void ScriptEngine::clearHook() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (L_) {
        lua_sethook(L_, nullptr, 0, 0);
    }
}

} // namespace rnr::scripting
