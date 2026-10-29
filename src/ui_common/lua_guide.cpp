#include "ui_common/lua_guide.h"
#include "ui_common/lua_guide_examples.h"
#include "ui_common/app_window.h"
#include <imgui.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace rnr::ui {

namespace {

// ============================================================================
// Palette
// ============================================================================
const ImVec4 kHeading   = ImVec4(0.00f, 0.85f, 1.00f, 1.0f);
const ImVec4 kSubHead   = ImVec4(1.00f, 0.82f, 0.28f, 1.0f);
const ImVec4 kGood      = ImVec4(0.30f, 0.90f, 0.50f, 1.0f);
const ImVec4 kWarn      = ImVec4(1.00f, 0.62f, 0.25f, 1.0f);
const ImVec4 kDanger    = ImVec4(1.00f, 0.42f, 0.38f, 1.0f);

// Syntax colours
const ImVec4 kSynDefault = ImVec4(0.84f, 0.88f, 0.94f, 1.0f);
const ImVec4 kSynKeyword = ImVec4(0.80f, 0.58f, 0.98f, 1.0f);
const ImVec4 kSynCnc     = ImVec4(0.30f, 0.82f, 1.00f, 1.0f);
const ImVec4 kSynBuiltin = ImVec4(0.95f, 0.86f, 0.55f, 1.0f);
const ImVec4 kSynMach3   = ImVec4(1.00f, 0.65f, 0.35f, 1.0f);
const ImVec4 kSynString  = ImVec4(0.62f, 0.90f, 0.52f, 1.0f);
const ImVec4 kSynNumber  = ImVec4(0.98f, 0.72f, 0.48f, 1.0f);
const ImVec4 kSynComment = ImVec4(0.45f, 0.58f, 0.52f, 1.0f);

void heading(const char* text) {
    ImGui::Spacing();
    ImGui::TextColored(kHeading, "%s", text);
    ImGui::Separator();
    ImGui::Spacing();
}

void subHeading(const char* text) {
    ImGui::Spacing();
    ImGui::TextColored(kSubHead, "%s", text);
}

void paragraph(const char* text) {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
}

void note(const ImVec4& col, const char* label, const char* text) {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextColored(col, "%s", label);
    ImGui::SameLine();
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
}

void bullet(const char* text) {
    ImGui::Bullet();
    ImGui::SameLine();
    paragraph(text);
}

// ============================================================================
// Lua syntax highlighting (token based)
// ============================================================================
bool isKeyword(const std::string& w) {
    static const char* const kw[] = {"local", "function", "return", "if", "then", "else", "elseif", "end",
                                     "for", "while", "do", "in", "and", "or", "not", "nil", "true", "false",
                                     "break", "repeat", "until", "goto"};
    for (const char* k : kw) if (w == k) return true;
    return false;
}

bool isBuiltin(const std::string& w) {
    static const char* const prefixes[] = {"string.", "math.", "table.", "io.", "os."};
    for (const char* p : prefixes) if (w.rfind(p, 0) == 0) return true;
    static const char* const names[] = {"print", "tonumber", "tostring", "ipairs", "pairs", "pcall",
                                        "error", "type", "assert", "select", "require"};
    for (const char* n : names) if (w == n) return true;
    return false;
}

bool isMach3(const std::string& w) {
    static const char* const names[] = {"Code", "WhileIsMoving", "Sleep", "IsMoving", "Message", "GetDRO",
                                        "SetDRO", "ActivateSignal", "DeActivateSignal", "IsProbing",
                                        "MsgBox", "Question", "GetUserText"};
    for (const char* n : names) if (w == n) return true;
    return false;
}

void renderHighlightedLine(const std::string& line) {
    struct Seg { size_t b, e; ImVec4 col; };
    std::vector<Seg> segs;
    size_t i = 0;
    const size_t n = line.size();
    auto push = [&](size_t b, size_t e, const ImVec4& c) { if (e > b) segs.push_back({b, e, c}); };
    while (i < n) {
        const char c = line[i];
        if (c == '-' && i + 1 < n && line[i + 1] == '-') {
            push(i, n, kSynComment);
            break;
        }
        if (c == '"' || c == '\'') {
            size_t j = i + 1;
            while (j < n && line[j] != c) {
                if (line[j] == '\\') ++j;
                ++j;
            }
            j = std::min(n, j + 1);
            push(i, j, kSynString);
            i = j;
            continue;
        }
        if (std::isdigit(static_cast<unsigned char>(c)) ||
            (c == '.' && i + 1 < n && std::isdigit(static_cast<unsigned char>(line[i + 1])))) {
            size_t j = i;
            while (j < n && (std::isalnum(static_cast<unsigned char>(line[j])) || line[j] == '.')) ++j;
            push(i, j, kSynNumber);
            i = j;
            continue;
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            size_t j = i;
            while (j < n && (std::isalnum(static_cast<unsigned char>(line[j])) || line[j] == '_' || line[j] == '.')) ++j;
            const std::string w = line.substr(i, j - i);
            ImVec4 col = kSynDefault;
            if (isKeyword(w))                 col = kSynKeyword;
            else if (w.rfind("cnc.", 0) == 0) col = kSynCnc;
            else if (isMach3(w))              col = kSynMach3;
            else if (isBuiltin(w))            col = kSynBuiltin;
            push(i, j, col);
            i = j;
            continue;
        }
        size_t j = i + 1;
        while (j < n && !std::isalnum(static_cast<unsigned char>(line[j])) && line[j] != '_' &&
               line[j] != '"' && line[j] != '\'' && !(line[j] == '-' && j + 1 < n && line[j + 1] == '-')) ++j;
        push(i, j, kSynDefault);
        i = j;
    }

    if (segs.empty()) {
        ImGui::TextUnformatted(" ");
        return;
    }
    for (size_t k = 0; k < segs.size(); ++k) {
        if (k > 0) ImGui::SameLine(0.0f, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, segs[k].col);
        ImGui::TextUnformatted(line.data() + segs[k].b, line.data() + segs[k].e);
        ImGui::PopStyleColor();
    }
}

// ============================================================================
// Drawing helpers for diagrams
// ============================================================================
void arrow(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float thickness = 2.0f) {
    dl->AddLine(a, b, col, thickness);
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float len = std::max(1e-3f, std::sqrt(dx * dx + dy * dy));
    const float ux = dx / len, uy = dy / len;
    const float hs = 8.0f;
    const ImVec2 p1(b.x - ux * hs - uy * hs * 0.55f, b.y - uy * hs + ux * hs * 0.55f);
    const ImVec2 p2(b.x - ux * hs + uy * hs * 0.55f, b.y - uy * hs - ux * hs * 0.55f);
    dl->AddTriangleFilled(b, p1, p2, col);
}

// Box with a title and up to a few lines of text, centred
void box(ImDrawList* dl, ImVec2 p0, ImVec2 p1, ImU32 fill, ImU32 border, const char* title, const char* body) {
    dl->AddRectFilled(p0, p1, fill, 6.0f);
    dl->AddRect(p0, p1, border, 6.0f, 0, 1.5f);
    const float cx = (p0.x + p1.x) * 0.5f;
    float y = p0.y + 8.0f;
    const ImVec2 ts = ImGui::CalcTextSize(title);
    dl->AddText(ImVec2(cx - ts.x * 0.5f, y), IM_COL32(255, 255, 255, 255), title);
    y += ts.y + 4.0f;
    if (body) {
        std::string s(body);
        size_t start = 0;
        while (start <= s.size()) {
            size_t nl = s.find('\n', start);
            const std::string ln = s.substr(start, nl == std::string::npos ? std::string::npos : nl - start);
            const ImVec2 ls = ImGui::CalcTextSize(ln.c_str());
            dl->AddText(ImVec2(cx - ls.x * 0.5f, y), IM_COL32(190, 205, 222, 255), ln.c_str());
            y += ls.y + 1.0f;
            if (nl == std::string::npos) break;
            start = nl + 1;
        }
    }
}

void labelAt(ImDrawList* dl, ImVec2 centre, ImU32 col, const char* text) {
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const ImVec2 p(centre.x - ts.x * 0.5f, centre.y - ts.y * 0.5f);
    dl->AddRectFilled(ImVec2(p.x - 4, p.y - 1), ImVec2(p.x + ts.x + 4, p.y + ts.y + 1), IM_COL32(16, 20, 28, 230), 3.0f);
    dl->AddText(p, col, text);
}

// "Who talks to whom" while a script runs
void drawThreadDiagram() {
    const float w = std::max(560.0f, ImGui::GetContentRegionAvail().x);
    const float h = 230.0f;
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(o, ImVec2(o.x + w, o.y + h), IM_COL32(12, 16, 23, 255), 6.0f);

    const float bw = w * 0.24f, bh = 92.0f;
    const ImVec2 ui0(o.x + 16, o.y + 26), ui1(ui0.x + bw, ui0.y + bh);
    const ImVec2 sc0(o.x + w * 0.5f - bw * 0.5f, o.y + 26), sc1(sc0.x + bw, sc0.y + bh);
    const ImVec2 mw0(o.x + w - 16 - bw, o.y + 26), mw1(mw0.x + bw, mw0.y + bh);

    box(dl, ui0, ui1, IM_COL32(24, 52, 78, 255), IM_COL32(0, 170, 230, 255), "UI thread",
        "draws the screen\nshows your dialogs\nE-STOP button / ESC");
    box(dl, sc0, sc1, IM_COL32(44, 34, 70, 255), IM_COL32(170, 120, 250, 255), "Your Lua script",
        "own thread\nwaits in wait_idle,\nprobe, prompt, sleep");
    box(dl, mw0, mw1, IM_COL32(22, 58, 40, 255), IM_COL32(60, 200, 110, 255), "Motion worker",
        "40 Hz loop\nplans & streams motion\nlimits, interlocks");

    const ImU32 c_cmd = IM_COL32(120, 200, 255, 255);
    const ImU32 c_ret = IM_COL32(150, 220, 160, 255);
    // script <-> UI (dialogs)
    arrow(dl, ImVec2(sc0.x, sc0.y + 30), ImVec2(ui1.x, ui0.y + 30), c_cmd);
    labelAt(dl, ImVec2((sc0.x + ui1.x) * 0.5f, ui0.y + 22), c_cmd, "cnc.prompt / cnc.input");
    arrow(dl, ImVec2(ui1.x, ui0.y + 66), ImVec2(sc0.x, sc0.y + 66), c_ret);
    labelAt(dl, ImVec2((sc0.x + ui1.x) * 0.5f, ui0.y + 76), c_ret, "button / value");
    // script <-> worker (motion)
    arrow(dl, ImVec2(sc1.x, sc0.y + 30), ImVec2(mw0.x, mw0.y + 30), c_cmd);
    labelAt(dl, ImVec2((sc1.x + mw0.x) * 0.5f, mw0.y + 22), c_cmd, "cnc.gcode / probe");
    arrow(dl, ImVec2(mw0.x, mw0.y + 66), ImVec2(sc1.x, sc0.y + 66), c_ret);
    labelAt(dl, ImVec2((sc1.x + mw0.x) * 0.5f, mw0.y + 76), c_ret, "state, positions");
    // E-stop path
    const ImU32 c_stop = IM_COL32(255, 90, 80, 255);
    const float y_stop = o.y + h - 40.0f;
    dl->AddLine(ImVec2((ui0.x + ui1.x) * 0.5f, ui1.y), ImVec2((ui0.x + ui1.x) * 0.5f, y_stop), c_stop, 2.0f);
    dl->AddLine(ImVec2((ui0.x + ui1.x) * 0.5f, y_stop), ImVec2((mw0.x + mw1.x) * 0.5f, y_stop), c_stop, 2.0f);
    arrow(dl, ImVec2((sc0.x + sc1.x) * 0.5f, y_stop), ImVec2((sc0.x + sc1.x) * 0.5f, sc1.y + 4), c_stop);
    arrow(dl, ImVec2((mw0.x + mw1.x) * 0.5f, y_stop), ImVec2((mw0.x + mw1.x) * 0.5f, mw1.y + 4), c_stop);
    labelAt(dl, ImVec2(o.x + w * 0.5f, y_stop + 16), c_stop, "E-STOP / Cancel: script ends at its next cnc.* call, motion stops");

    ImGui::Dummy(ImVec2(w, h));
}

// Horizontal flow chart of an M6 tool change with the m6.lua hook
void drawM6Flow() {
    const char* titles[] = {"Program: M6 T2", "Hold", "m6.lua runs", "Operator dialog", "Script ends"};
    const char* bodies[] = {"spindle stops,\nZ retract", "ToolChangeHold\nstate", "cnc.get_pending_tool()\n= 2",
                            "cnc.prompt(...)\ninsert T2", "no error:\njob resumes"};
    const float w = std::max(600.0f, ImGui::GetContentRegionAvail().x);
    const float h = 112.0f;
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(o, ImVec2(o.x + w, o.y + h), IM_COL32(12, 16, 23, 255), 6.0f);
    const int n = 5;
    const float gap = 26.0f;
    const float bw = (w - 24.0f - gap * (n - 1)) / n;
    const ImU32 fills[] = {IM_COL32(40, 40, 52, 255), IM_COL32(70, 54, 20, 255), IM_COL32(44, 34, 70, 255),
                           IM_COL32(24, 52, 78, 255), IM_COL32(22, 58, 40, 255)};
    const ImU32 borders[] = {IM_COL32(140, 140, 160, 255), IM_COL32(240, 180, 60, 255), IM_COL32(170, 120, 250, 255),
                             IM_COL32(0, 170, 230, 255), IM_COL32(60, 200, 110, 255)};
    for (int i = 0; i < n; ++i) {
        const ImVec2 p0(o.x + 12 + i * (bw + gap), o.y + 16);
        const ImVec2 p1(p0.x + bw, p0.y + 80);
        box(dl, p0, p1, fills[i], borders[i], titles[i], bodies[i]);
        if (i + 1 < n) arrow(dl, ImVec2(p1.x + 2, p0.y + 40), ImVec2(p1.x + gap - 2, p0.y + 40), IM_COL32(200, 210, 225, 255));
    }
    ImGui::Dummy(ImVec2(w, h));
}

// Side view of a two-touch Z probe on a touch plate
void drawProbeDiagram() {
    const float w = std::max(520.0f, ImGui::GetContentRegionAvail().x);
    const float h = 210.0f;
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(o, ImVec2(o.x + w, o.y + h), IM_COL32(12, 16, 23, 255), 6.0f);

    const float cx = o.x + w * 0.30f;
    const float work_top = o.y + h - 40.0f;
    // Workpiece & plate
    dl->AddRectFilled(ImVec2(o.x + 20, work_top), ImVec2(o.x + w * 0.62f, o.y + h - 12), IM_COL32(120, 90, 55, 255), 2.0f);
    dl->AddText(ImVec2(o.x + 28, work_top + 8), IM_COL32(255, 240, 220, 255), "workpiece (Z0 = top)");
    const float plate_top = work_top - 18.0f;
    dl->AddRectFilled(ImVec2(cx - 40, plate_top), ImVec2(cx + 40, work_top), IM_COL32(190, 190, 200, 255), 2.0f);
    // Tool
    const float tool_tip = plate_top - 46.0f;
    dl->AddRectFilled(ImVec2(cx - 9, tool_tip - 70), ImVec2(cx + 9, tool_tip), IM_COL32(170, 200, 230, 255), 2.0f);
    dl->AddTriangleFilled(ImVec2(cx - 9, tool_tip), ImVec2(cx + 9, tool_tip), ImVec2(cx, tool_tip + 10), IM_COL32(170, 200, 230, 255));
    // Plate thickness dimension
    const float dx = cx + 58.0f;
    dl->AddLine(ImVec2(dx, plate_top), ImVec2(dx, work_top), IM_COL32(255, 210, 90, 255), 1.5f);
    dl->AddLine(ImVec2(dx - 5, plate_top), ImVec2(dx + 5, plate_top), IM_COL32(255, 210, 90, 255), 1.5f);
    dl->AddLine(ImVec2(dx - 5, work_top), ImVec2(dx + 5, work_top), IM_COL32(255, 210, 90, 255), 1.5f);
    dl->AddText(ImVec2(dx + 8, plate_top), IM_COL32(255, 210, 90, 255), "plate thickness");

    // Steps
    const float sx = o.x + w * 0.66f;
    const ImU32 c1 = IM_COL32(120, 200, 255, 255), c2 = IM_COL32(150, 220, 160, 255), c3 = IM_COL32(255, 160, 90, 255);
    arrow(dl, ImVec2(cx - 26, tool_tip - 60), ImVec2(cx - 26, plate_top - 4), c1, 2.5f);
    arrow(dl, ImVec2(cx - 40, plate_top - 6), ImVec2(cx - 40, plate_top - 30), c2, 2.0f);
    arrow(dl, ImVec2(cx - 54, plate_top - 30), ImVec2(cx - 54, plate_top - 4), c3, 1.5f);
    dl->AddText(ImVec2(sx, o.y + 24), c1, "1  cnc.probe(\"Z\", -travel, fast_feed)");
    dl->AddText(ImVec2(sx, o.y + 48), c2, "2  back off: G91 G0 Z<retract>");
    dl->AddText(ImVec2(sx, o.y + 72), c3, "3  cnc.probe(\"Z\", -2*retract, slow_feed)");
    dl->AddText(ImVec2(sx, o.y + 104), IM_COL32(255, 210, 90, 255), "4  cnc.set_work_pos(\"Z\", plate)");
    dl->AddText(ImVec2(sx, o.y + 128), IM_COL32(200, 210, 225, 255), "   tool tip now reads Z = plate thickness,");
    dl->AddText(ImVec2(sx, o.y + 146), IM_COL32(200, 210, 225, 255), "   so the work top is Z0");
    ImGui::Dummy(ImVec2(w, h));
}

// Static picture of the dialog a script call produces (not interactive)
void drawDialogPreview(const char* title, const char* message, const char* input_text, const char* hint,
                       std::vector<const char*> buttons, bool hint_is_error = false) {
    const float w = std::min(460.0f, std::max(320.0f, ImGui::GetContentRegionAvail().x));
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float lh = ImGui::GetTextLineHeight();

    std::vector<std::string> lines;
    {
        std::string s(message);
        size_t start = 0;
        while (true) {
            size_t nl = s.find('\n', start);
            lines.push_back(s.substr(start, nl == std::string::npos ? std::string::npos : nl - start));
            if (nl == std::string::npos) break;
            start = nl + 1;
        }
    }
    float h = 30.0f + 26.0f + lines.size() * (lh + 2.0f) + 14.0f + 40.0f + 14.0f;
    if (input_text) h += 34.0f;
    if (hint) h += lh + 4.0f;

    // Window
    dl->AddRectFilled(ImVec2(o.x + 4, o.y + 4), ImVec2(o.x + w + 4, o.y + h + 4), IM_COL32(0, 0, 0, 90), 6.0f);
    dl->AddRectFilled(o, ImVec2(o.x + w, o.y + h), IM_COL32(26, 30, 38, 255), 6.0f);
    dl->AddRectFilled(o, ImVec2(o.x + w, o.y + 26), IM_COL32(32, 70, 112, 255), 6.0f, ImDrawFlags_RoundCornersTop);
    dl->AddText(ImVec2(o.x + 10, o.y + 5), IM_COL32(255, 255, 255, 255), title);
    float y = o.y + 34.0f;
    dl->AddText(ImVec2(o.x + 12, y), IM_COL32(255, 215, 64, 255), "[!] SCRIPT OPERATOR PROMPT");
    y += lh + 6.0f;
    dl->AddLine(ImVec2(o.x + 10, y), ImVec2(o.x + w - 10, y), IM_COL32(70, 80, 95, 255));
    y += 8.0f;
    for (const auto& ln : lines) {
        dl->AddText(ImVec2(o.x + 12, y), IM_COL32(225, 230, 238, 255), ln.c_str());
        y += lh + 2.0f;
    }
    y += 6.0f;
    if (input_text) {
        dl->AddRectFilled(ImVec2(o.x + 12, y), ImVec2(o.x + w - 12, y + 26), IM_COL32(40, 48, 60, 255), 3.0f);
        dl->AddRect(ImVec2(o.x + 12, y), ImVec2(o.x + w - 12, y + 26), IM_COL32(0, 150, 220, 255), 3.0f);
        dl->AddText(ImVec2(o.x + 18, y + 5), IM_COL32(240, 240, 240, 255), input_text);
        const float tx = o.x + 18 + ImGui::CalcTextSize(input_text).x + 1;
        dl->AddLine(ImVec2(tx, y + 5), ImVec2(tx, y + 21), IM_COL32(240, 240, 240, 255));
        y += 32.0f;
    }
    if (hint) {
        dl->AddText(ImVec2(o.x + 12, y), hint_is_error ? IM_COL32(255, 115, 102, 255) : IM_COL32(150, 160, 175, 255), hint);
        y += lh + 4.0f;
    }
    dl->AddLine(ImVec2(o.x + 10, y), ImVec2(o.x + w - 10, y), IM_COL32(70, 80, 95, 255));
    y += 10.0f;
    // Buttons, centred; first one is the primary (green)
    float total = 0.0f;
    std::vector<float> bws;
    for (const char* b : buttons) {
        bws.push_back(std::max(84.0f, ImGui::CalcTextSize(b).x + 28.0f));
        total += bws.back();
    }
    total += (buttons.size() - 1) * 10.0f;
    float bx = o.x + (w - total) * 0.5f;
    for (size_t i = 0; i < buttons.size(); ++i) {
        const ImU32 fill = (i == 0) ? IM_COL32(31, 133, 92, 255) : IM_COL32(89, 97, 115, 255);
        dl->AddRectFilled(ImVec2(bx, y), ImVec2(bx + bws[i], y + 30), fill, 4.0f);
        const ImVec2 ts = ImGui::CalcTextSize(buttons[i]);
        dl->AddText(ImVec2(bx + (bws[i] - ts.x) * 0.5f, y + (30 - ts.y) * 0.5f), IM_COL32(255, 255, 255, 255), buttons[i]);
        bx += bws[i] + 10.0f;
    }
    ImGui::Dummy(ImVec2(w + 6, h + 6));
}

using namespace guide_examples;

// ============================================================================
// API reference data
// ============================================================================
struct ApiEntry {
    const char* category;
    const char* sig;
    const char* desc;
};

const ApiEntry kApi[] = {
    {"Motion", "cnc.gcode(cmd)", "Runs one MDI block (e.g. \"G90 G0 Z10\"). Motion is queued: call cnc.wait_idle() before the next motion command. Raises an error when the controller rejects the command (machine busy, invalid / unsupported G-code, soft limits, G53/G28 on an unhomed axis)."},
    {"Motion", "cnc.wait_idle([timeout_s])", "Waits until the machine is idle. Raises an error on E-STOP, cancellation or timeout."},
    {"Motion", "cnc.is_moving()", "true while a job, jog, homing or probing move is active."},
    {"Motion", "cnc.sleep_ms(ms)", "Sleeps; ends the script immediately on E-STOP / Cancel."},
    {"Motion", "cnc.stop()", "Controlled stop of the running job / move. Also ends the calling script."},
    {"Motion", "cnc.pause()  /  cnc.resume()", "Feed hold (decelerates along the path) / Cycle Start."},
    {"Position", "cnc.get_work_pos()", "{x, y, z, a} work coordinates in mm (Z = tool tip when a tool length offset is active)."},
    {"Position", "cnc.get_machine_pos()", "{x, y, z, a} machine coordinates in mm."},
    {"Position", "cnc.zero_axis(axis)  /  cnc.zero_all()", "Sets the work coordinate of \"X\", \"Y\", \"Z\" or \"A\" (or all) to 0 at the current position. Refused while a job is active."},
    {"Position", "cnc.set_work_pos(axis, value)", "The current position of the axis reads `value` (G92-style, adjusts the active work offset)."},
    {"Position", "cnc.is_homed(axis)", "true when the axis was homed in this session."},
    {"Spindle & I/O", "cnc.set_spindle(on)  /  cnc.is_spindle_on()", "Spindle (OUT1) on/off. Refused in E-STOP and dry run; a configured but silent VFD trips E-STOP."},
    {"Spindle & I/O", "cnc.set_spindle_rpm(rpm)  /  cnc.get_spindle_rpm()", "Commanded speed; clamped to the VFD min/max RPM."},
    {"Spindle & I/O", "cnc.set_torch(on)  /  cnc.is_torch_on()", "Plasma torch (same output as the spindle, OUT1)."},
    {"Spindle & I/O", "cnc.get_input(pin)  /  cnc.get_inputs_mask()", "Raw level of input IN1..IN12 (true = high) / all 12 inputs as a bitmask."},
    {"Spindle & I/O", "cnc.set_output(n, on)  /  cnc.get_output(n)", "Outputs OUT1..OUT4. Switching on is refused in E-STOP."},
    {"Probing", "cnc.probe(axis, distance, feed)", "Moves `distance` mm (signed) until the probe touches. Returns {hit, x, y, z, a} (work coordinates). No retract, no offset change. hit = false if refused (probe already touching / no probe input) or no contact."},
    {"Probing", "cnc.is_probe_tripped()  /  cnc.get_last_probe_pos()", "Probe input state / result of the last probe."},
    {"Probing", "cnc.get_probe_config()", "Probe settings: plate_thickness, tip_diameter, fast_feed, slow_feed, max_travel, retract, diameter, xy_clearance, probe_depth, size_x, size_y."},
    {"Tools", "cnc.get_current_tool()  /  cnc.get_pending_tool()", "Tool in the spindle / tool requested by the M6 being processed."},
    {"Tools", "cnc.get_tool_change_config()", "{move_to_park_xy, park_x, park_y, safe_z, mode} (mode: 0 manual, 1 ATC, 2 ignore M6)."},
    {"Tools", "cnc.get_tool_length_mode()", "\"not_configured\", \"touch_off\" or \"tool_table\"."},
    {"Tools", "cnc.get_tool_length(t)", "Length of tool t from the tool table (mm, relative to the reference tool) or nil."},
    {"Tools", "cnc.measure_tool_length(t)", "Probes tool t on the tool setter below the current position, stores and returns its length. Raises an error on failure. Measure the reference tool first after startup / Z homing."},
    {"Overrides", "cnc.get_feed_override()  /  cnc.set_feed_override(pct)", "Feed override in percent (10 .. 250)."},
    {"Dialogs", "cnc.prompt(title, message [, buttons])", "Modal dialog with buttons (default {\"OK\", \"Cancel\"}). Blocks the script and returns the clicked label (\"Cancel\" if the script is cancelled). The first button is highlighted. Use \\n for line breaks."},
    {"Dialogs", "cnc.input(title, message [, default])", "Text input dialog. Returns the text, or nil on Cancel. Enter confirms."},
    {"Dialogs", "cnc.input_number(title, message [, default [, min [, max]]])", "Number input dialog. OK is only enabled for a valid number inside [min, max]. Returns the number, or nil on Cancel."},
    {"Dialogs", "cnc.log(msg)  /  print(...)", "Writes to the script console."},
    {"Safety", "cnc.estop([reason])", "Triggers E-STOP (logs the reason) and ends the script."},
    {"Safety", "cnc.is_estop()  /  cnc.reset_estop()", "E-STOP state / try to reset (returns false while the E-STOP input, a limit or an interlock is still active)."},
    {"Files", "cnc.resolve_path(rel_path)", "Absolute path of a file in the working dir, the executable dir or its parents (nil if not found)."},
};

} // namespace

// ============================================================================
// LuaGuide
// ============================================================================
void LuaGuide::codeBlock(const char* id, const char* code, core::IMotionController* run_with) {
    ImGui::PushID(id);
    // Toolbar: Copy (+ Run demo)
    const float copy_w = ImGui::CalcTextSize("Copy").x + 32.0f;
    const float run_w  = ImGui::CalcTextSize("Run demo").x + 32.0f;
    const float bar_w  = copy_w + (run_with ? run_w + ImGui::GetStyle().ItemSpacing.x : 0.0f);
    const bool flashing = (flash_id_ == id) && (ImGui::GetTime() < flash_until_);
    if (flashing) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(kGood, "%s", flash_text_.c_str());
        ImGui::SameLine();
    }
    ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), ImGui::GetContentRegionMax().x - bar_w));
    if (run_with) {
        if (ImGui::Button("Run demo", ImVec2(run_w, 28.0f))) {
            const bool ok = run_with->runScriptString(code, true);
            flash_id_ = id;
            flash_text_ = ok ? "Started - answer the dialog" : "Another script is running";
            flash_until_ = ImGui::GetTime() + 3.0;
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Runs this demo now (dialogs and log output only, no motion)");
        ImGui::SameLine();
    }
    if (ImGui::Button("Copy", ImVec2(copy_w, 28.0f))) {
        ImGui::SetClipboardText(code);
        flash_id_ = id;
        flash_text_ = "Copied to clipboard";
        flash_until_ = ImGui::GetTime() + 2.0;
    }

    // Code
    ImFont* mono = AppWindow::getGCodeFont();
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.07f, 0.10f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.22f, 0.30f, 0.42f, 0.70f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
    ImGui::BeginChild("##code", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY,
                      ImGuiWindowFlags_HorizontalScrollbar);
    if (mono) ImGui::PushFont(mono);
    const char* p = code;
    while (true) {
        const char* nl = std::strchr(p, '\n');
        const std::string line(p, nl ? static_cast<size_t>(nl - p) : std::strlen(p));
        renderHighlightedLine(line);
        if (!nl) break;
        p = nl + 1;
    }
    if (mono) ImGui::PopFont();
    ImGui::EndChild();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
    ImGui::PopID();
    ImGui::Spacing();
}

void LuaGuide::render(core::IMotionController& controller) {
    if (ImGui::BeginTabBar("##LuaGuideTabs")) {
        if (ImGui::BeginTabItem("Start Here"))      { renderStartTab();               ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Dialogs"))         { renderDialogsTab(controller);   ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Machine Examples")){ renderExamplesTab();            ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Hooks & Macros"))  { renderHooksTab();               ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("API Reference"))   { renderApiTab();                 ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Mach3 Scripts"))   { renderMach3Tab();               ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Safety"))          { renderSafetyTab();              ImGui::EndTabItem(); }
        ImGui::EndTabBar();
    }
}

void LuaGuide::renderStartTab() {
    ImGui::BeginChild("##StartTab", ImVec2(0.0f, 0.0f));
    heading("WHAT LUA SCRIPTS CAN DO");
    paragraph("Scripts (Lua 5.4) automate anything you would otherwise do by hand: touch-off routines, tool "
              "changes, warm-ups, drilling patterns, checklists, or small \"wizards\" that ask the operator "
              "for values and then run the machine. Everything goes through the cnc.* functions.");

    subHeading("Where scripts come from");
    bullet("REPL line in the Lua Scripting deck - type a line and press Enter to run it immediately.");
    bullet("Macro buttons - '+ Add Macro' in the Lua Scripting deck binds a .lua file to a button.");
    bullet("M-code macros - scripts/macros/M100.lua .. M999.lua run when you type that M-code in MDI.");
    bullet("Hooks - m6.lua runs on every tool change, home_all.lua can replace the built-in homing.");

    heading("HOW A SCRIPT RUNS");
    paragraph("Your script runs in its own thread, next to the screen (UI thread) and the motion worker. "
              "Calls like cnc.wait_idle(), cnc.probe() or cnc.prompt() simply wait; the screen stays "
              "responsive and E-STOP always works.");
    drawThreadDiagram();

    heading("YOUR FIRST SCRIPT");
    codeBlock("first", kFirstScript);
    subHeading("The three rules");
    bullet("One motion command at a time: after every motion cnc.gcode(...) call cnc.wait_idle(). A second "
           "command while the machine still moves is rejected (and raises an error).");
    bullet("Errors stop the script. A rejected command, a failed probe wait or E-STOP raises a Lua error; "
           "use pcall(...) when you want to handle it yourself (see the Dialogs tab).");
    bullet("Each cnc.gcode() call is parsed on its own with G90 as default: write \"G91 G0 Z5\" on one line "
           "instead of sending G91 and G0 Z5 separately.");

    subHeading("Try it");
    paragraph("Open the Dialogs tab and press 'Run demo' on any example: the dialog appears just like it "
              "would from your own macro. The demos only show dialogs and write to the log.");
    ImGui::EndChild();
}

void LuaGuide::renderDialogsTab(core::IMotionController& controller) {
    ImGui::BeginChild("##DialogsTab", ImVec2(0.0f, 0.0f));
    heading("THREE KINDS OF DIALOGS");
    paragraph("All dialogs are modal: the script waits until the operator answers. While a dialog is open "
              "the machine can still be stopped with E-STOP, which also closes the dialog and ends the script.");

    if (ImGui::BeginTable("##DlgKinds", 3, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_PadOuterX)) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextColored(kSubHead, "cnc.prompt  - buttons");
        drawDialogPreview("Coolant", "Turn the flood coolant on?", nullptr, nullptr, {"Yes", "No"});
        ImGui::TableSetColumnIndex(1);
        ImGui::TextColored(kSubHead, "cnc.input  - text");
        drawDialogPreview("Job Name", "Name of this job:", "bracket-v2", nullptr, {"OK", "Cancel"});
        ImGui::TableSetColumnIndex(2);
        ImGui::TextColored(kSubHead, "cnc.input_number  - number");
        drawDialogPreview("Pocket Depth", "Total depth of cut (mm):", "40", "Enter a number between 0.1 and 25",
                          {"OK", "Cancel"}, true);
        ImGui::EndTable();
    }
    note(kGood, "Return values:", "prompt -> the clicked label;  input -> text or nil;  input_number -> number or nil. "
         "nil always means the operator pressed Cancel.");
    note(kWarn, "Tip:", "the first button is drawn green - put the safe or most common answer first. "
         "Use \\n in the message for line breaks.");

    heading("1. YES / NO QUESTION");
    codeBlock("dlg_confirm", kDlgConfirm, &controller);
    heading("2. MENU - CHOOSE ONE OF SEVERAL OPTIONS");
    codeBlock("dlg_menu", kDlgMenu, &controller);
    heading("3. TEXT INPUT");
    codeBlock("dlg_text", kDlgText, &controller);
    heading("4. NUMBER INPUT WITH LIMITS");
    paragraph("The dialog itself refuses values outside the limits (the OK button stays disabled and the hint turns "
              "red), so the script never has to re-check the range.");
    codeBlock("dlg_number", kDlgNumber, &controller);
    heading("5. WIZARD - SEVERAL STEPS, SUMMARY, EDIT LOOP");
    drawDialogPreview("Summary", "Width: 100.0 mm\nLength: 60.0 mm\nStepover: 5.0 mm\n\n13 passes", nullptr, nullptr,
                      {"Start", "Edit", "Cancel"});
    codeBlock("dlg_wizard", kDlgWizard, &controller);
    heading("6. CHECKLIST BEFORE A JOB");
    codeBlock("dlg_checklist", kDlgChecklist, &controller);
    heading("7. SHOWING ERRORS TO THE OPERATOR");
    paragraph("pcall(f, ...) runs f and returns false plus the error message instead of stopping the script. "
              "This example moves the machine, so it has no 'Run demo' button.");
    codeBlock("dlg_error", kDlgError);
    ImGui::EndChild();
}

void LuaGuide::renderExamplesTab() {
    ImGui::BeginChild("##ExamplesTab", ImVec2(0.0f, 0.0f));
    note(kWarn, "These scripts move the machine.", "Try them in Simulator Mode first, then on the machine with the "
         "hand near E-STOP. They are templates: adapt feeds, depths and outputs to your machine.");

    heading("SPINDLE WARM-UP WITH A CHOICE DIALOG");
    codeBlock("ex_warmup", kExWarmup);

    heading("Z TOUCH PLATE (TWO TOUCHES)");
    drawProbeDiagram();
    codeBlock("ex_touch", kExTouchPlate);

    heading("HOLE GRID GENERATOR (FORM -> G-CODE)");
    paragraph("Asks for the grid size, shows a summary and drills each hole with a G83 peck cycle.");
    codeBlock("ex_grid", kExGrid);

    heading("FACING / SURFACING WIZARD");
    codeBlock("ex_facing", kExFacing);

    heading("PARK POSITION (MACHINE COORDINATES)");
    codeBlock("ex_park", kExPark);

    heading("MEASURE SEVERAL TOOLS FOR THE TOOL TABLE");
    codeBlock("ex_tools", kExMeasureTools);

    heading("WAIT FOR A SENSOR WITH TIMEOUT");
    codeBlock("ex_wait", kExWaitInput);

    heading("WRITE A JOB LOG FILE");
    codeBlock("ex_log", kExJobLog);
    ImGui::EndChild();
}

void LuaGuide::renderHooksTab() {
    ImGui::BeginChild("##HooksTab", ImVec2(0.0f, 0.0f));
    heading("TOOL CHANGE HOOK (M6)");
    paragraph("With 'M6 script' enabled, every M6 in a program stops the spindle, holds the job and runs "
              "scripts/mill/m6.lua. When the script finishes without an error, the job continues automatically.");
    drawM6Flow();
    codeBlock("hook_m6", kHookM6);
    note(kWarn, "Important:", "returning false is not enough to keep the job waiting - raise an error or call "
         "cnc.estop(). The shipped m6.lua already handles both tool length modes.");

    heading("HOMING HOOK");
    paragraph("With 'Home script' enabled, the HOME buttons run scripts/mill/home_all.lua instead of the built-in "
              "homing cycle. The built-in cycle (switch search, zero, pull-off) is recommended unless you need "
              "a special order or squaring routine.");

    heading("M-CODE MACROS (M100 - M999)");
    paragraph("A file scripts/macros/M<number>.lua runs when that M-code is entered in MDI (or bound to a macro "
              "button). Inside G-code programs custom M-codes are not executed - the program shows a warning "
              "for them - so a running job can never start an unexpected script.");
    codeBlock("hook_macro", kHookMacro);

    heading("MACRO BUTTONS");
    paragraph("In the Lua Scripting deck, '+ Add Macro' binds a script file to a button; right-click a button "
              "to rename it, change its file, or show it on the main control panel.");

    heading("PLASMA SCRIPTS");
    paragraph("scripts/cut/ contains m3_touch.lua (initial height sensing) and laser_offset.lua. They are "
              "run as macros (buttons or MDI); they are not triggered automatically by M3.");

    heading("WHERE FILES ARE FOUND");
    paragraph("Relative paths are looked up in the working directory, then next to the executable and up to "
              "two folders above it. cnc.resolve_path(\"scripts/mill/m6.lua\") shows which file is used.");
    ImGui::EndChild();
}

void LuaGuide::renderApiTab() {
    ImGui::TextUnformatted("Filter");
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputTextWithHint("##ApiFilter", "type to filter, e.g. probe, dialog, tool ...", api_filter_, sizeof(api_filter_));

    std::string filter(api_filter_);
    std::transform(filter.begin(), filter.end(), filter.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    auto matches = [&](const ApiEntry& e) {
        if (filter.empty()) return true;
        std::string hay = std::string(e.category) + " " + e.sig + " " + e.desc;
        std::transform(hay.begin(), hay.end(), hay.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return hay.find(filter) != std::string::npos;
    };

    ImGui::BeginChild("##ApiTab", ImVec2(0.0f, 0.0f));
    const char* current = nullptr;
    bool table_open = false;
    int shown = 0;
    for (const auto& e : kApi) {
        if (!matches(e)) continue;
        ++shown;
        if (!current || std::strcmp(current, e.category) != 0) {
            if (table_open) ImGui::EndTable();
            current = e.category;
            subHeading(current);
            table_open = ImGui::BeginTable(e.category, 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
            if (table_open) {
                ImGui::TableSetupColumn("Function", ImGuiTableColumnFlags_WidthStretch, 0.9f);
                ImGui::TableSetupColumn("What it does", ImGuiTableColumnFlags_WidthStretch, 1.6f);
            }
        }
        if (!table_open) continue;
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(kSynCnc, "%s", e.sig);
        ImGui::PopTextWrapPos();
        ImGui::TableSetColumnIndex(1);
        ImGui::TextWrapped("%s", e.desc);
    }
    if (table_open) ImGui::EndTable();
    if (shown == 0) ImGui::TextDisabled("No function matches \"%s\".", api_filter_);

    subHeading("Standard Lua");
    paragraph("string, math, table, os.date / os.time / os.clock and io.open (files) are available. For safety "
              "os.execute, os.remove, os.rename, os.exit, io.popen and native C modules are disabled.");
    ImGui::EndChild();
}

void LuaGuide::renderMach3Tab() {
    ImGui::BeginChild("##Mach3Tab", ImVec2(0.0f, 0.0f));
    heading("MACH3 FUNCTIONS THAT WORK AS-IS");
    paragraph("Common Mach3 (VB / Cypress Enable) macro calls are available as Lua functions, so most simple "
              "macros only need Lua syntax (if ... then ... end, local variables) to run.");
    if (ImGui::BeginTable("##Mach3", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Mach3", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("BreakoutCNC", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Notes", ImGuiTableColumnFlags_WidthStretch, 1.3f);
        ImGui::TableHeadersRow();
        auto row = [](const char* a, const char* b, const char* c) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextColored(kSynMach3, "%s", a);
            ImGui::TableSetColumnIndex(1); ImGui::TextColored(kSynCnc, "%s", b);
            ImGui::TableSetColumnIndex(2); ImGui::TextWrapped("%s", c);
        };
        row("Code(\"G0 X10\")", "cnc.gcode", "Raises an error when the command is rejected");
        row("WhileIsMoving()", "cnc.wait_idle", "");
        row("IsMoving()", "cnc.is_moving", "");
        row("Sleep(ms)", "cnc.sleep_ms", "Cancellable");
        row("Message(text)", "cnc.log", "Script console");
        row("MsgBox(text)", "cnc.prompt", "Dialog with an OK button");
        row("Question(text)", "cnc.input_number", "Returns nil on Cancel");
        row("GetUserText(text)", "cnc.input", "Returns nil on Cancel");
        row("GetDRO(n)", "cnc.get_work_pos", "n = 0..3 or 800..803 (X, Y, Z, A)");
        row("SetDRO(n, value)", "cnc.set_work_pos", "Current position reads `value`");
        row("ActivateSignal(n) / DeActivateSignal(n)", "cnc.set_output", "Outputs OUT1..OUT4");
        row("IsProbing()", "cnc.is_probe_tripped", "Probe input state");
        ImGui::EndTable();
    }
    heading("EXAMPLE");
    codeBlock("mach3", kMach3Example);
    note(kWarn, "Not supported:", "OEM codes (DoOEMButton, GetOEMDRO, ...), Mach3 user DROs/LEDs and VB-only syntax.");
    ImGui::EndChild();
}

void LuaGuide::renderSafetyTab() {
    ImGui::BeginChild("##SafetyTab", ImVec2(0.0f, 0.0f));
    heading("WHAT PROTECTS YOU WHILE A SCRIPT RUNS");
    bullet("E-STOP (button, ESC key, E-STOP input) stops the machine and ends the script at its next cnc.* call; "
           "open dialogs close and return \"Cancel\".");
    bullet("Every motion command goes through the same checks as MDI: soft limits, homed axes for machine "
           "coordinates, interlocks. A refused command raises an error instead of being skipped silently.");
    bullet("Scripts cannot start programs, delete files or load native code (sandbox).");
    bullet("Only one script runs at a time; a second one is refused while the first is running.");

    heading("GOOD HABITS");
    bullet("Test in Simulator Mode first - dialogs and the log behave exactly the same.");
    bullet("Check preconditions at the top: cnc.is_homed(), cnc.is_probe_tripped(), tool length mode ...");
    bullet("Confirm destructive steps with a dialog showing the values that will be used.");
    bullet("Always cnc.wait_idle() after motion; never assume a move has finished.");
    bullet("Turn the spindle off at the end and on every early return.");
    bullet("Use cnc.estop(\"reason\") when continuing would be unsafe - the reason appears in the log.");

    heading("COMMON ERRORS");
    if (ImGui::BeginTable("##Errors", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Message", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Cause / fix", ImGuiTableColumnFlags_WidthStretch, 1.2f);
        ImGui::TableHeadersRow();
        auto row = [](const char* a, const char* b) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextColored(kDanger, "%s", a);
            ImGui::TableSetColumnIndex(1); ImGui::TextWrapped("%s", b);
        };
        row("cnc.gcode rejected: \"...\"", "Machine still moving (add cnc.wait_idle()), invalid or unsupported G-code, soft limit, or G53/G28 on an unhomed axis. The reason is shown in brackets.");
        row("Feed move without a feed rate (F word)", "The first G1/G2/G3 needs an F word; afterwards F is remembered between MDI commands.");
        row("Script execution aborted by operator / E-STOP", "E-STOP or Cancel while the script was running.");
        row("Motion wait timed out", "cnc.wait_idle(timeout) expired - the move took longer than expected.");
        row("Another script is already running", "Wait for the running script or cancel it in the Lua deck.");
        ImGui::EndTable();
    }
    ImGui::EndChild();
}

} // namespace rnr::ui
