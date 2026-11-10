#include "ui_common/board_panels.h"
#include "breakout_version.h"
#include "core/board/board_registry.h"
#include "core/transport/serial_port.h"
#include "ui_common/file_dialog.h"
#include "ui_common/theme.h"
#include "imgui.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace rnr::ui {

namespace {

bool machineAtRest(const core::TelemetrySnapshot& t) {
    return t.state == core::ControllerState::Idle || t.state == core::ControllerState::EStop ||
           t.state == core::ControllerState::Disconnected;
}

std::vector<transport::SerialPortInfo>& portCache() {
    static std::vector<transport::SerialPortInfo> ports;
    return ports;
}
bool& portsScanned() {
    static bool scanned = false;
    return scanned;
}

} // namespace

const board::BoardCapabilities& boardCaps(const core::IMotionController& controller,
                                          const core::TelemetrySnapshot& telemetry) {
    static board::BoardCapabilities caps = board::rnrEcoMotionCapabilities();
    static uint64_t rev = ~0ULL;
    static std::string id;
    static bool connected = false;
    if (rev != telemetry.capabilities_revision || id != telemetry.board_id || connected != telemetry.usb_connected) {
        caps = controller.getBoardCapabilities();
        rev = telemetry.capabilities_revision;
        id = telemetry.board_id;
        connected = telemetry.usb_connected;
    }
    return caps;
}

void renderNotUsedByBoardNote(const char* what, const board::BoardCapabilities& caps) {
    ImGui::PushTextWrapPos(0.0f);
    const bool plural = std::strchr(what, ',') || std::strstr(what, " and ");
    ImGui::TextDisabled("%s %s set in the board itself on %s. Use Board Settings to see and change them.", what,
                        plural ? "are" : "is", caps.display_name.c_str());
    ImGui::PopTextWrapPos();
}

void renderBoardSelector(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry,
                         const std::function<void()>& on_settings_changed) {
    const auto& reg = board::BoardRegistry::instance();
    core::MachineConfig cfg = controller.getConfig();
    const auto* current = reg.find(cfg.board_id);
    const bool can_change = machineAtRest(telemetry);

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Board");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(std::max(160.0f, ImGui::GetContentRegionAvail().x));
    ImGui::BeginDisabled(!can_change);
    if (ImGui::BeginCombo("##BoardSelect", current ? current->display_name.c_str() : cfg.board_id.c_str())) {
        for (const auto& d : reg.all()) {
            const bool sel = d.id == cfg.board_id;
            if (ImGui::Selectable(d.display_name.c_str(), sel) && !sel) {
                cfg.board_id = d.id;
                board::completeProfile(d.id, cfg.board_profiles[d.id]);
                controller.applyConfig(cfg);
                if (on_settings_changed) on_settings_changed();
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", d.description.c_str());
            if (sel) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    if (!can_change && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Stop the machine to change the board.");
    }
    if (current) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%s", current->description.c_str());
        ImGui::PopTextWrapPos();
    }
    if (!current || current->family != board::Family::FirmwarePlanned) return;

    // ---- Connection settings of a firmware board ----
    auto& profile = cfg.board_profiles[cfg.board_id];
    board::completeProfile(cfg.board_id, profile);
    bool changed = false;
    std::string transport = profile.get("transport", "serial");
    if (transport.rfind("fake-", 0) == 0) {
        // --fake-board mode: the board is the built-in emulator
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(theme::pal().text_info, "Test mode: this board is the built-in emulator (%s), not hardware.",
                           transport.c_str());
        ImGui::PopTextWrapPos();
    }
    const bool emulator = transport.rfind("fake-", 0) == 0;
    ImGui::BeginDisabled(!can_change || emulator);
    if (transport != "serial" && transport != "tcp") transport = "serial";
    if (!emulator) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Connect by");
    ImGui::SameLine();
    if (ImGui::RadioButton("USB / serial", transport == "serial")) {
        profile.set("transport", "serial");
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Network", transport == "tcp")) {
        profile.set("transport", "tcp");
        changed = true;
    }

    if (profile.get("transport", "serial") == "serial") {
        if (!portsScanned()) {
            portCache() = transport::SerialPort::enumerate();
            portsScanned() = true;
        }
        const std::string address = profile.get("address");
        std::string preview = address.empty() ? "Choose a port" : address;
        for (const auto& p : portCache()) {
            if (p.name == address && !p.description.empty()) preview = p.name + " - " + p.description;
        }
        const float refresh_w = ImGui::CalcTextSize("Refresh").x + 32.0f;
        const float lbl_w = ImGui::CalcTextSize("Port").x + ImGui::GetStyle().ItemSpacing.x;
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Port");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(std::max(120.0f, ImGui::GetContentRegionAvail().x - refresh_w - lbl_w));
        if (ImGui::BeginCombo("##FwPort", preview.c_str())) {
            if (portCache().empty()) ImGui::TextDisabled("No serial ports found");
            for (const auto& p : portCache()) {
                const std::string label = p.description.empty() ? p.name : p.name + " - " + p.description;
                if (ImGui::Selectable(label.c_str(), p.name == address)) {
                    profile.set("address", p.name);
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Refresh##FwPorts", ImVec2(refresh_w, 28.0f))) {
            portCache() = transport::SerialPort::enumerate();
        }

        static const int kBauds[] = {115200, 250000, 230400, 921600, 57600, 38400, 19200, 9600};
        const int baud = static_cast<int>(profile.getInt("baud", 115200));
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Baud rate");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140.0f);
        char baud_txt[16];
        std::snprintf(baud_txt, sizeof(baud_txt), "%d", baud);
        if (ImGui::BeginCombo("##FwBaud", baud_txt)) {
            for (int b : kBauds) {
                char t[16];
                std::snprintf(t, sizeof(t), "%d", b);
                if (ImGui::Selectable(t, b == baud)) {
                    profile.setInt("baud", b);
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        bool dtr = profile.getBool("dtr_on_connect", true);
        if (ImGui::Checkbox("Reset the board when connecting (DTR)", &dtr)) {
            profile.setBool("dtr_on_connect", dtr);
            changed = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Arduino-based GRBL boards restart when the port opens with DTR on, which is normal.\n"
                              "ESP32 boards (FluidNC) should connect with it off so they do not enter their bootloader.");
        }
    } else {
        char host[128];
        std::snprintf(host, sizeof(host), "%s", profile.get("address").c_str());
        const float port_field_w = 80.0f;
        const float lbl_w = ImGui::CalcTextSize("Address").x + ImGui::GetStyle().ItemSpacing.x;
        const float port_lbl_w = ImGui::CalcTextSize("Port").x + ImGui::GetStyle().ItemSpacing.x * 2.0f;
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Address");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(std::max(120.0f, ImGui::GetContentRegionAvail().x - lbl_w - port_field_w - port_lbl_w));
        if (ImGui::InputTextWithHint("##FwHost", "192.168.1.20", host, sizeof(host))) {
            profile.set("address", host);
            changed = true;
        }
        ImGui::SameLine();
        ImGui::TextUnformatted("Port");
        ImGui::SameLine();
        int port = static_cast<int>(profile.getInt("tcp_port", 23));
        ImGui::SetNextItemWidth(port_field_w);
        if (ImGui::InputInt("##FwTcpPort", &port, 0, 0)) {
            profile.setInt("tcp_port", std::clamp(port, 1, 65535));
            changed = true;
        }
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Telnet-style connection, port 23 on grblHAL and FluidNC by default.");
        ImGui::PopTextWrapPos();
    }
    } // !emulator
    ImGui::EndDisabled();
    if (changed) {
        controller.applyConfig(cfg);
        if (on_settings_changed) on_settings_changed();
    }

    // ---- State of the firmware board ----
    if (telemetry.board_firmware_planned && !telemetry.simulator_mode) {
        ImGui::PushTextWrapPos(0.0f);
        if (telemetry.board_connecting) {
            ImGui::TextColored(theme::pal().text_warn, "Identifying the board...");
        } else if (telemetry.usb_connected && !telemetry.firmware_version.empty()) {
            ImGui::TextColored(theme::pal().text_ok, "Firmware: %s", telemetry.firmware_version.c_str());
            if (!telemetry.firmware_state.empty()) {
                ImGui::TextDisabled("Board state: %s", telemetry.firmware_state.c_str());
            }
        }
        if (!telemetry.board_status_text.empty()) {
            ImGui::TextColored(theme::pal().text_warn, "%s", telemetry.board_status_text.c_str());
        }
        ImGui::PopTextWrapPos();
        if (telemetry.usb_connected && telemetry.firmware_state == "Alarm") {
            if (ImGui::Button("Unlock ($X)", ImVec2(ImGui::CalcTextSize("Unlock ($X)").x + 32.0f, 28.0f))) {
                controller.unlockBoard();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Clears the board's alarm without homing. If the position was lost, home the machine instead.");
            }
        }
    }
}

namespace {
// "<prefix>_<board>_<yyyymmdd>_<hhmmss>.txt" for files saved next to the application; when: "yyyy-mm-dd hh:mm:ss"
std::string timestampedFileName(const char* prefix, const std::string& board_id, std::string* when = nullptr) {
    const std::time_t now = std::time(nullptr);
    std::tm tmv{};
#ifdef _WIN32
    localtime_s(&tmv, &now);
#else
    localtime_r(&now, &tmv);
#endif
    char name[160];
    std::snprintf(name, sizeof(name), "%s_%s_%04d%02d%02d_%02d%02d%02d.txt", prefix, board_id.c_str(), tmv.tm_year + 1900,
                  tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    if (when) {
        char w[32];
        std::snprintf(w, sizeof(w), "%04d-%02d-%02d %02d:%02d:%02d", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                      tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
        *when = w;
    }
    return name;
}
} // namespace

void renderBoardConsole(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry) {
    struct Line {
        transport::ProtocolLogEntry::Dir dir;
        bool is_status;
        std::string text;
    };
    static std::vector<Line> lines;
    static uint64_t last_seq = 0;
    static uint64_t rev = ~0ULL;
    static bool show_status = false;
    static bool autoscroll = true;
    static char input[256] = "";

    if (rev != telemetry.capabilities_revision) {
        // Another connection: its log starts again
        rev = telemetry.capabilities_revision;
        lines.clear();
        last_seq = 0;
    }
    for (auto& e : controller.getBoardLog(last_seq)) {
        last_seq = std::max(last_seq, e.seq);
        lines.push_back({e.dir, e.is_status, std::move(e.text)});
    }
    if (lines.size() > 2000) lines.erase(lines.begin(), lines.begin() + static_cast<std::ptrdiff_t>(lines.size() - 2000));

    const bool live = telemetry.board_firmware_planned && telemetry.usb_connected && !telemetry.simulator_mode;
    if (!telemetry.board_firmware_planned) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("The console is available for GRBL-family boards (they speak a text protocol).");
        ImGui::PopTextWrapPos();
        return;
    }
    ImGui::Checkbox("Show status reports", &show_status);
    ImGui::SameLine();
    ImGui::Checkbox("Follow", &autoscroll);
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear view")) lines.clear();

    ImGui::BeginChild("##BoardConsoleLog", ImVec2(0.0f, 200.0f), ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
    for (const auto& l : lines) {
        if (l.is_status && !show_status) continue;
        using D = transport::ProtocolLogEntry::Dir;
        const ImVec4 col = (l.dir == D::Sent) ? theme::pal().text_info
                         : (l.dir == D::Error) ? theme::pal().text_danger
                         : (l.dir == D::Info) ? theme::pal().text_muted
                         : ImGui::GetStyleColorVec4(ImGuiCol_Text);
        const char* prefix = (l.dir == D::Sent) ? "> " : (l.dir == D::Received) ? "  " : "# ";
        ImGui::TextColored(col, "%s%s", prefix, l.text.c_str());
    }
    if (autoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20.0f) ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();

    const bool at_rest = telemetry.state == core::ControllerState::Idle || telemetry.state == core::ControllerState::EStop ||
                         telemetry.state == core::ControllerState::ToolChangeHold;
    ImGui::BeginDisabled(!live || !at_rest);
    const float send_w = ImGui::CalcTextSize("Send").x + 32.0f;
    ImGui::SetNextItemWidth(std::max(100.0f, ImGui::GetContentRegionAvail().x - send_w - ImGui::GetStyle().ItemSpacing.x));
    bool send = ImGui::InputTextWithHint("##BoardConsoleInput", "$$, $#, $G, $I ...", input, sizeof(input),
                                         ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    send = ImGui::Button("Send##BoardConsole", ImVec2(send_w, 28.0f)) || send;
    if (send && input[0] != '\0') {
        controller.sendBoardConsoleLine(input);
        input[0] = '\0';
        ImGui::SetKeyboardFocusHere(-1);
    }
    for (const char* q : {"$$", "$#", "$G", "$I"}) {
        if (ImGui::Button(q, ImVec2(ImGui::CalcTextSize(q).x + 32.0f, 28.0f))) controller.sendBoardConsoleLine(q);
        ImGui::SameLine();
    }
    ImGui::NewLine();
    // Board report: the board's answers to read-only queries plus the recent exchange, saved to a text file
    static bool report_waiting = false;
    static uint64_t report_seen = 0;
    static std::string report_note;
    if (ImGui::Button("Save board report", ImVec2(ImGui::CalcTextSize("Save board report").x + 32.0f, 28.0f))) {
        report_seen = controller.getBoardReport().revision;
        report_waiting = controller.requestBoardReport();
        report_note = report_waiting ? "Asking the board..." : "";
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && ImGui::BeginTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
        ImGui::TextUnformatted("Asks the board for its build info, settings, offsets and modes ($I, $$, $#, $G ...; "
                               "nothing is changed) and saves them with the recent exchange to a text file next to the "
                               "application. Useful when something behaves unexpectedly on a new board.");
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
    ImGui::EndDisabled();
    if (report_waiting) {
        const auto rep = controller.getBoardReport();
        if (!rep.running && rep.revision != report_seen) {
            report_waiting = false;
            std::string when;
            const std::string name = timestampedFileName("board_report", telemetry.board_id, &when);
            std::ofstream ofs(name);
            ofs << "BreakoutCNC " << BREAKOUTCNC_VERSION_STRING << " board report, " << when << "\n" << rep.text;
            report_note = ofs.good() ? std::string("Saved ") + name + " next to the application"
                                     : std::string("Could not write ") + name;
        }
    }
    if (!report_note.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(report_waiting ? theme::pal().text_muted : theme::pal().text_ok, "%s", report_note.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("Commands are sent only while the machine is idle. BreakoutCNC keeps its own work offsets, so "
                        "G10 / G92 typed here do not change the app's coordinates; the next job sets the board's modes again.");
    ImGui::PopTextWrapPos();
}

void renderFirmwareSettings(core::IMotionController& controller, const core::TelemetrySnapshot& telemetry) {
    static std::map<std::string, std::string> edits;
    static uint64_t seen_revision = ~0ULL;
    static std::string note;
    const auto st = controller.getFirmwareSettings();
    if (st.revision != seen_revision) {
        seen_revision = st.revision;
        // Keep edits the board has not taken yet; drop the ones that now match
        for (auto it = edits.begin(); it != edits.end();) {
            bool same = false;
            for (const auto& item : st.items) {
                if (item.id == it->first && item.value == it->second) same = true;
            }
            it = same ? edits.erase(it) : std::next(it);
        }
    }
    const bool live = telemetry.board_firmware_planned && telemetry.usb_connected && !telemetry.simulator_mode &&
                      !telemetry.board_connecting;
    if (!telemetry.board_firmware_planned) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("The selected board keeps its settings in BreakoutCNC (Motors, Limits & Homing, Inputs & Outputs).");
        ImGui::PopTextWrapPos();
        return;
    }
    if (!live) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Connect to the board to read its settings.");
        ImGui::PopTextWrapPos();
        return;
    }
    const bool at_rest = (telemetry.state == core::ControllerState::Idle || telemetry.state == core::ControllerState::EStop) &&
                         !telemetry.board_busy;
    ImGui::BeginDisabled(!at_rest);
    if (ImGui::Button("Read from board", ImVec2(ImGui::CalcTextSize("Read from board").x + 32.0f, 28.0f))) {
        controller.requestFirmwareSettings();
    }
    ImGui::SameLine();
    if (ImGui::Button("Backup", ImVec2(ImGui::CalcTextSize("Backup").x + 32.0f, 28.0f))) {
        const std::string name = timestampedFileName("board_settings", telemetry.board_id);
        std::ofstream ofs(name);
        for (const auto& it : st.items) ofs << "$" << it.id << "=" << it.value << "\n";
        note = ofs.good() ? std::string("Saved ") + name + " next to the application"
                          : std::string("Could not write ") + name;
    }
    ImGui::SameLine();
    if (ImGui::Button("Restore...", ImVec2(ImGui::CalcTextSize("Restore...").x + 32.0f, 28.0f))) {
        const std::string file = openFileDialog("Restore board settings", "Text files (*.txt)\0*.txt\0All files\0*.*\0",
                                                {"*.txt"});
        if (!file.empty()) {
            std::ifstream ifs(file);
            std::string line;
            size_t n = 0;
            while (std::getline(ifs, line)) {
                while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
                if (line.size() < 3 || line[0] != '$') continue;
                const size_t eq = line.find('=');
                if (eq == std::string::npos) continue;
                const std::string id = line.substr(1, eq - 1);
                const std::string v = line.substr(eq + 1);
                for (const auto& it : st.items) {
                    if (it.id == id && it.value != v) {
                        edits[id] = v;
                        ++n;
                    }
                }
            }
            note = n == 0 ? "The file matches the board: nothing to restore"
                          : std::to_string(n) + " differences loaded: check them below, then Write all changes";
        }
    }
    ImGui::EndDisabled();
    ImGui::PushTextWrapPos(0.0f);
    if (!st.status.empty()) ImGui::TextDisabled("%s", st.status.c_str());
    if (!note.empty()) ImGui::TextColored(theme::pal().text_info, "%s", note.c_str());
    if (!at_rest) ImGui::TextColored(theme::pal().text_warn, "Settings can be changed only while the machine is idle.");
    ImGui::PopTextWrapPos();

    if (!edits.empty()) {
        ImGui::BeginDisabled(!at_rest);
        char lbl[64];
        std::snprintf(lbl, sizeof(lbl), "Write all changes (%zu)", edits.size());
        if (ImGui::Button(lbl, ImVec2(ImGui::CalcTextSize(lbl).x + 32.0f, 32.0f))) {
            std::vector<std::pair<std::string, std::string>> w(edits.begin(), edits.end());
            controller.writeFirmwareSettings(w);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Discard", ImVec2(ImGui::CalcTextSize("Discard").x + 32.0f, 32.0f))) edits.clear();
    }

    std::string group;
    bool group_open = false;
    for (const auto& it : st.items) {
        if (it.group != group) {
            if (group_open) ImGui::TreePop();
            group = it.group;
            group_open = ImGui::TreeNodeEx(group.c_str(), ImGuiTreeNodeFlags_SpanAvailWidth);
        }
        if (!group_open) continue;
        ImGui::PushID(it.id.c_str());
        const auto e = edits.find(it.id);
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s", (e != edits.end() ? e->second : it.value).c_str());
        const float value_w = 120.0f;
        const std::string label = st.yaml ? it.name : ("$" + it.id + "  " + it.name);
        ImGui::AlignTextToFramePadding();
        if (e != edits.end()) ImGui::TextColored(theme::pal().text_warn, "%s", label.c_str());
        else ImGui::TextUnformatted(label.c_str());
        if (ImGui::IsItemHovered() && (!it.description.empty() || it.needs_restart)) {
            ImGui::SetTooltip("%s%s%s", it.description.c_str(), it.description.empty() ? "" : "\n",
                              it.needs_restart ? "The board must be restarted for this change." : "");
        }
        ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetContentRegionMax().x - value_w - 110.0f));
        ImGui::SetNextItemWidth(value_w);
        ImGui::BeginDisabled(!at_rest);
        if (ImGui::InputText("##v", buf, sizeof(buf))) {
            if (it.value == buf) edits.erase(it.id);
            else edits[it.id] = buf;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::TextDisabled("%s", it.unit.c_str());
        if (!it.error.empty()) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(theme::pal().text_danger, "Not accepted: %s", it.error.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::PopID();
    }
    if (group_open) ImGui::TreePop();
}

} // namespace rnr::ui
