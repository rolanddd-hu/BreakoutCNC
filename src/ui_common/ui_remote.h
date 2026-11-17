#pragma once
#include <cstdint>
#include <string>
#include <vector>

// Test-only UI remote control (enabled with --fake-board --ui-remote): the application reads commands from
// ui_remote_commands.txt next to the executable and feeds them to Dear ImGui as its own input events, so a test can
// operate the UI without touching the computer's mouse or keyboard. Screenshots of the frame go to
// ui_remote_shots/<name>.png; ui_remote_status.txt reports how many commands have been executed.
//
// Commands (one per line, coordinates in window pixels):
//   move X Y         put the virtual mouse at X Y
//   click X Y        left click at X Y          dclick X Y   double click
//   press X Y        hold the left button at X Y (until release)
//   scroll X Y N     mouse wheel N notches at X Y (positive = up)
//   type TEXT        type the text
//   key NAME         enter, tab, escape, backspace, delete, up, down, left, right, f12, ctrl+a
//   wait N           wait N frames
//   shot NAME        save a screenshot
//   quit             close the window
namespace rnr::ui {

class UiRemote {
public:
    explicit UiRemote(std::string dir);

    // After the platform backend's NewFrame, before ImGui::NewFrame: inject this frame's events
    void injectEvents(bool* request_close);
    // After the frame is drawn, before swapping buffers: save a pending screenshot
    void afterRender(int framebuffer_w, int framebuffer_h);

private:
    void loadCommands();
    void writeStatus();

    std::string dir_;
    std::vector<std::string> commands_;
    size_t next_ = 0;
    uintmax_t file_size_ = 0;
    float mx_ = -1.0f, my_ = -1.0f;
    int   wait_frames_ = 0;
    int   step_ = 0;          // Sub-step of a multi-frame command (click: move, down, up)
    std::string shot_name_;
    int   frame_ = 0;
};

} // namespace rnr::ui
