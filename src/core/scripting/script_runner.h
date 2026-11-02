#pragma once
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include "script_engine.h"
#include "script_api.h"
#include "core/controller/motion_controller.h"

namespace rnr::scripting {

enum class PromptInputKind : uint8_t {
    None   = 0, // Buttons only (cnc.prompt)
    Text   = 1, // cnc.input
    Number = 2, // cnc.input_number
};

struct PromptRequest {
    std::string              title;
    std::string              message;
    std::vector<std::string> buttons;
    PromptInputKind          input_kind    = PromptInputKind::None;
    std::string              input_default;
    double                   input_min     = -1e300;
    double                   input_max     = 1e300;
};

using ScriptCompletionCallback = std::function<void(bool success, const std::string& error_message)>;

class ScriptRunner {
public:
    ScriptRunner(core::IMotionController* controller = nullptr);
    ~ScriptRunner();

    // Attach or change motion controller
    void setController(core::IMotionController* controller);

    // Asynchronous non-blocking execution (runs on dedicated Tier-2 script worker thread)
    bool runStringAsync(const std::string& code, ScriptCompletionCallback on_complete = nullptr);
    bool runFileAsync(const std::string& filepath, ScriptCompletionCallback on_complete = nullptr);

    // Synchronous blocking execution (caller blocks until script finishes)
    bool runStringBlocking(const std::string& code, std::string* out_error = nullptr);
    bool runFileBlocking(const std::string& filepath, std::string* out_error = nullptr);

    // Cooperative emergency cancellation (safe to call from UI thread or E-Stop handler)
    void cancel();

    // Status queries
    bool isRunning() const { return is_running_.load(); }
    bool isCancellationRequested() const { return cancel_requested_.load(); }
    std::string getActiveScriptName() const;

    // Interactive UI Prompt Mailbox (called by Dear ImGui render loop)
    bool hasPendingPrompt() const;
    PromptRequest getPendingPrompt() const;
    // `input` carries the entered text for cnc.input / cnc.input_number dialogs
    void submitPromptResponse(const std::string& response, const std::string& input = {});

    // Script Logging
    void setLogCallback(LogCallback cb) { engine_.setLogCallback(std::move(cb)); }
    ScriptEngine& getEngine() { return engine_; }

private:
    void initLuaEnvironment();
    bool executeClaimed(bool is_file, const std::string& source, std::string* out_error);
    bool startAsync(bool is_file, const std::string& source, ScriptCompletionCallback on_complete);
    std::string handlePromptRequest(
        const std::string& title,
        const std::string& message,
        const std::vector<std::string>& buttons);
    std::string handleInputRequest(const InputRequest& req, bool& ok);

    core::IMotionController* controller_ = nullptr;
    ScriptEngine             engine_;

    std::thread              worker_thread_;
    std::atomic<bool>        is_running_{false};
    std::atomic<bool>        cancel_requested_{false};
    std::atomic<bool>        thread_done_{true};
    std::string              active_script_name_;
    mutable std::mutex       name_mutex_;
    mutable std::mutex       runner_mutex_;

    // Interactive Prompt Mailbox
    mutable std::mutex       prompt_mutex_;
    std::condition_variable  prompt_cv_;
    bool                     has_pending_prompt_ = false;
    PromptRequest            pending_prompt_;
    std::string              prompt_response_;
    std::string              prompt_input_;
};

} // namespace rnr::scripting
