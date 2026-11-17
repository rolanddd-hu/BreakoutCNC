#include "ui_common/board_panels.h"
#include "core/util/utf8_path.h"
#include "breakout_version.h"
#include "core/board/board_registry.h"
#include "core/transport/serial_port.h"
#include "ui_common/file_dialog.h"
#include "ui_common/theme.h"
#include "imgui.h"
#include "imgui_internal.h" // TableGetHoveredRow()
#include <algorithm>
#include <chrono>
#include <cmath>
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
    core::MachineConfig cfg = *controller.getConfigShared();
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

namespace {
// Short form of a unit the board reports, for the value field's suffix ("milliseconds" -> "ms")
std::string shortUnit(const std::string& u) {
    if (u == "microseconds") return "us";
    if (u == "milliseconds") return "ms";
    if (u == "seconds") return "s";
    if (u == "mm/sec^2") return "mm/s^2";
    if (u == "deg/sec^2") return "deg/s^2";
    if (u == "percent") return "%";
    return u;
}

// Board Settings, one card per group: a framed header with the number of settings, then a table without borders. Columns: the setting number (dimmed, so the names start in one column), the name (wraps), and the
// value field with its unit attached on the right. The row under the mouse is highlighted; a changed value that is not
// written yet is shown in the warning colour (name and field border).
void renderSettingGroups(const board::grbl::SettingsSnapshot& st, std::map<std::string, std::string>& edits,
                         bool at_rest) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const float frame_h = ImGui::GetFrameHeight();
    float unit_w = 0.0f;
    float id_w = 0.0f;
    std::map<std::string, size_t> group_counts;
    for (const auto& it : st.items) {
        unit_w = std::max(unit_w, ImGui::CalcTextSize(shortUnit(it.unit).c_str()).x);
        if (!st.yaml) id_w = std::max(id_w, ImGui::CalcTextSize(("$" + it.id).c_str()).x);
        ++group_counts[it.group];
    }
    if (unit_w > 0.0f) unit_w += style.FramePadding.x * 2.0f;
    const float field_w = std::max(70.0f, ImGui::CalcTextSize("-00000.000").x + style.FramePadding.x * 2.0f);
    const float value_w = field_w + unit_w;
    const ImU32 hover_bg = ImGui::GetColorU32(ImGuiCol_HeaderHovered, 0.35f);
    const ImU32 unit_bg = ImGui::GetColorU32(ImGuiCol_FrameBgHovered);
    const ImU32 unit_line = ImGui::GetColorU32(ImGuiCol_Border);
    const ImU32 unit_text = ImGui::GetColorU32(ImGuiCol_TextDisabled);

    constexpr float kGroupIndent = 8.0f;
    ImGui::Indent(kGroupIndent);
    std::string group;
    bool first_group = true, table_open = false;
    // The group's ID stays pushed while its table is open (BeginTable pushes an ID of its own on top of it)
    auto closeGroup = [&] {
        if (table_open) {
            ImGui::EndTable();
            ImGui::PopID();
            ImGui::Spacing();
        }
        table_open = false;
    };
    for (const auto& it : st.items) {
        if (first_group || it.group != group) {
            closeGroup();
            first_group = false;
            group = it.group;
            const char* title = group.empty() ? "Settings" : group.c_str();
            ImGui::PushID(title);
            // Quieter than the settings window's own section headers: these are sub-sections of Board Settings
            ImGui::PushStyleColor(ImGuiCol_Header, ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
            ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImGui::GetStyleColorVec4(ImGuiCol_FrameBgHovered));
            ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImGui::GetStyleColorVec4(ImGuiCol_FrameBgActive));
            const bool open = ImGui::TreeNodeEx("##Group", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_SpanAvailWidth |
                                                               ImGuiTreeNodeFlags_AllowOverlap |
                                                               ImGuiTreeNodeFlags_NoTreePushOnOpen,
                                                "%s", title);
            ImGui::PopStyleColor(3);
            // Number of settings, at the right end of the header
            char count[32];
            const size_t n = group_counts[group];
            std::snprintf(count, sizeof(count), n == 1 ? "%zu setting" : "%zu settings", n);
            const ImVec2 count_sz = ImGui::CalcTextSize(count);
            const ImVec2 hmin = ImGui::GetItemRectMin(), hmax = ImGui::GetItemRectMax();
            ImGui::GetWindowDrawList()->AddText(
                ImVec2(hmax.x - count_sz.x - style.FramePadding.x * 2.0f, std::floor((hmin.y + hmax.y - count_sz.y) * 0.5f)),
                ImGui::GetColorU32(ImGuiCol_TextDisabled), count);
            ImGui::PopID();
            if (!open) continue;
            ImGui::PushID(title);
            table_open = ImGui::BeginTable("##Rows", id_w > 0.0f ? 3 : 2,
                                           ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX);
            if (!table_open) ImGui::PopID();
            if (table_open) {
                if (id_w > 0.0f) ImGui::TableSetupColumn("Number", ImGuiTableColumnFlags_WidthFixed, id_w);
                ImGui::TableSetupColumn("Setting", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed, value_w);
            }
        }
        if (!table_open) continue;
        ImGui::PushID(it.id.c_str());
        const auto e = edits.find(it.id);
        const bool changed = e != edits.end();
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s", (changed ? e->second : it.value).c_str());
        ImGui::TableNextRow(ImGuiTableRowFlags_None, frame_h + style.CellPadding.y * 2.0f);
        if (ImGui::TableGetHoveredRow() == ImGui::TableGetRowIndex()) {
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, hover_bg);
        }
        int col = 0;
        if (id_w > 0.0f) {
            ImGui::TableSetColumnIndex(col++);
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("$%s", it.id.c_str());
        }
        ImGui::TableSetColumnIndex(col++);
        ImGui::AlignTextToFramePadding();
        ImGui::PushTextWrapPos(0.0f);
        if (changed) ImGui::TextColored(theme::pal().text_warn, "%s", it.name.c_str());
        else ImGui::TextUnformatted(it.name.c_str());
        ImGui::PopTextWrapPos();
        if (ImGui::IsItemHovered() && (!it.description.empty() || it.needs_restart)) {
            ImGui::SetTooltip("%s%s%s", it.description.c_str(), it.description.empty() ? "" : "\n",
                              it.needs_restart ? "The board must be restarted for this change." : "");
        }

        // Value field with the unit as a suffix inside the same frame
        ImGui::TableSetColumnIndex(col++);
        const std::string unit = shortUnit(it.unit);
        const ImVec2 field_pos = ImGui::GetCursorScreenPos();
        ImGui::SetNextItemWidth(unit.empty() ? value_w : field_w);
        if (changed) {
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Border, theme::pal().text_warn);
        }
        ImGui::BeginDisabled(!at_rest);
        if (ImGui::InputText("##v", buf, sizeof(buf))) {
            if (it.value == buf) edits.erase(it.id);
            else edits[it.id] = buf;
        }
        ImGui::EndDisabled();
        if (changed) {
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        }
        if (!unit.empty()) {
            ImDrawList* dl = ImGui::GetWindowDrawList();
            const ImVec2 u0(field_pos.x + field_w, field_pos.y);
            const ImVec2 u1(field_pos.x + value_w, field_pos.y + frame_h);
            dl->AddRectFilled(u0, u1, unit_bg, style.FrameRounding, ImDrawFlags_RoundCornersRight);
            dl->AddLine(ImVec2(u0.x, u0.y + 3.0f), ImVec2(u0.x, u1.y - 3.0f), unit_line);
            const ImVec2 ts = ImGui::CalcTextSize(unit.c_str());
            dl->AddText(ImVec2(u0.x + (unit_w - ts.x) * 0.5f, u0.y + (frame_h - ts.y) * 0.5f), unit_text, unit.c_str());
        }
        if (!it.error.empty()) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(id_w > 0.0f ? 1 : 0);
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(theme::pal().text_danger, "Not accepted: %s", it.error.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::PopID();
    }
    closeGroup();
    ImGui::Unindent(kGroupIndent);
}
} // namespace

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
            std::ifstream ifs(util::pathFromUtf8(file));
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

    // The board's group names ("Probing", "Spindle", ...) are also section headers of the settings window: without
    // an ID scope of their own their tree nodes had the same ID as those headers (ImGui "conflicting ID" error)
    ImGui::PushID("##BoardSettingsList");
    renderSettingGroups(st, edits, at_rest);
    ImGui::PopID();
}

} // namespace rnr::ui
