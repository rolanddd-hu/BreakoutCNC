#include "ui_common/ui_remote.h"
#include "imgui.h"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace rnr::ui {

namespace {

uint32_t crc32(const uint8_t* d, size_t n, uint32_t crc = 0) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        init = true;
    }
    crc = ~crc;
    for (size_t i = 0; i < n; ++i) crc = table[(crc ^ d[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

void put32(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(static_cast<uint8_t>(x >> 24));
    v.push_back(static_cast<uint8_t>(x >> 16));
    v.push_back(static_cast<uint8_t>(x >> 8));
    v.push_back(static_cast<uint8_t>(x));
}

void chunk(std::vector<uint8_t>& png, const char* type, const std::vector<uint8_t>& data) {
    put32(png, static_cast<uint32_t>(data.size()));
    std::vector<uint8_t> td(type, type + 4);
    td.insert(td.end(), data.begin(), data.end());
    png.insert(png.end(), td.begin(), td.end());
    put32(png, crc32(td.data(), td.size()));
}

// Uncompressed (stored deflate) RGB PNG; rows are given bottom-up as glReadPixels returns them
bool writePng(const std::string& path, int w, int h, const std::vector<uint8_t>& rgba) {
    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>(h) * (static_cast<size_t>(w) * 3 + 1));
    for (int y = h - 1; y >= 0; --y) {
        raw.push_back(0);
        const uint8_t* row = rgba.data() + static_cast<size_t>(y) * static_cast<size_t>(w) * 4;
        for (int x = 0; x < w; ++x) {
            raw.push_back(row[x * 4]);
            raw.push_back(row[x * 4 + 1]);
            raw.push_back(row[x * 4 + 2]);
        }
    }
    std::vector<uint8_t> z = {0x78, 0x01};
    uint32_t a = 1, b = 0;
    for (uint8_t c : raw) {
        a = (a + c) % 65521;
        b = (b + a) % 65521;
    }
    size_t pos = 0;
    while (pos < raw.size() || raw.empty()) {
        const size_t len = std::min<size_t>(65535, raw.size() - pos);
        const bool last = pos + len >= raw.size();
        z.push_back(last ? 1 : 0);
        z.push_back(static_cast<uint8_t>(len));
        z.push_back(static_cast<uint8_t>(len >> 8));
        z.push_back(static_cast<uint8_t>(~len));
        z.push_back(static_cast<uint8_t>(~len >> 8));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos), raw.begin() + static_cast<std::ptrdiff_t>(pos + len));
        pos += len;
        if (raw.empty()) break;
    }
    put32(z, (b << 16) | a);

    std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<uint8_t> ihdr;
    put32(ihdr, static_cast<uint32_t>(w));
    put32(ihdr, static_cast<uint32_t>(h));
    ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});
    chunk(png, "IHDR", ihdr);
    chunk(png, "IDAT", z);
    chunk(png, "IEND", {});
    std::ofstream ofs(path, std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
    return ofs.good();
}

ImGuiKey keyFor(const std::string& n) {
    if (n == "enter") return ImGuiKey_Enter;
    if (n == "tab") return ImGuiKey_Tab;
    if (n == "escape") return ImGuiKey_Escape;
    if (n == "backspace") return ImGuiKey_Backspace;
    if (n == "delete") return ImGuiKey_Delete;
    if (n == "up") return ImGuiKey_UpArrow;
    if (n == "down") return ImGuiKey_DownArrow;
    if (n == "left") return ImGuiKey_LeftArrow;
    if (n == "right") return ImGuiKey_RightArrow;
    if (n == "f12") return ImGuiKey_F12;
    if (n == "a") return ImGuiKey_A;
    return ImGuiKey_None;
}

} // namespace

UiRemote::UiRemote(std::string dir) : dir_(std::move(dir)) {
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(dir_) / "ui_remote_shots", ec);
    std::ofstream(std::filesystem::path(dir_) / "ui_remote_commands.txt", std::ios::trunc);
    writeStatus();
}

void UiRemote::loadCommands() {
    const auto path = std::filesystem::path(dir_) / "ui_remote_commands.txt";
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    if (ec || size == file_size_) return;
    file_size_ = size;
    std::ifstream ifs(path);
    std::vector<std::string> lines;
    std::string l;
    while (std::getline(ifs, l)) {
        while (!l.empty() && (l.back() == '\r' || l.back() == ' ')) l.pop_back();
        if (!l.empty()) lines.push_back(l);
    }
    commands_ = lines;
}

void UiRemote::writeStatus() {
    std::ofstream ofs(std::filesystem::path(dir_) / "ui_remote_status.txt", std::ios::trunc);
    ofs << "executed " << next_ << "\n";
}

void UiRemote::injectEvents(bool* request_close) {
    ImGuiIO& io = ImGui::GetIO();
    ++frame_;
    if (frame_ % 10 == 0) loadCommands();
    // The virtual mouse wins over the real one (the platform backend posted the real position first)
    if (mx_ >= 0.0f) io.AddMousePosEvent(mx_, my_);
    if (wait_frames_ > 0) {
        --wait_frames_;
        return;
    }
    if (!shot_name_.empty()) return; // Screenshot pending
    if (next_ >= commands_.size()) return;

    std::istringstream is(commands_[next_]);
    std::string cmd;
    is >> cmd;
    auto done = [&](int wait = 2) {
        ++next_;
        step_ = 0;
        wait_frames_ = wait;
        writeStatus();
    };
    if (cmd == "move" || cmd == "click" || cmd == "dclick" || cmd == "scroll") {
        float x = 0, y = 0;
        is >> x >> y;
        if (step_ == 0) {
            mx_ = x;
            my_ = y;
            io.AddMousePosEvent(x, y);
            if (cmd == "move") { done(3); return; }
            step_ = 1;
            return;
        }
        if (cmd == "scroll") {
            float n = 0;
            is >> n;
            io.AddMouseWheelEvent(0.0f, n);
            done(3);
            return;
        }
        const int clicks = (cmd == "dclick") ? 2 : 1;
        // step 1: down, 2: up, (3: down, 4: up)
        if (step_ <= clicks * 2) {
            io.AddMouseButtonEvent(0, (step_ % 2) == 1);
            ++step_;
            if (step_ > clicks * 2) done(4);
            return;
        }
        done();
        return;
    }
    if (cmd == "press" || cmd == "release") {
        // press X Y: move there and hold the left button; release: let go (for buttons that act while held)
        if (cmd == "press") {
            is >> mx_ >> my_;
            io.AddMousePosEvent(mx_, my_);
            if (step_ == 0) { step_ = 1; return; }
            io.AddMouseButtonEvent(0, true);
        } else {
            io.AddMouseButtonEvent(0, false);
        }
        done(2);
        return;
    }
    if (cmd == "type") {
        std::string rest;
        std::getline(is, rest);
        if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
        io.AddInputCharactersUTF8(rest.c_str());
        done(2);
        return;
    }
    if (cmd == "key") {
        std::string name;
        is >> name;
        const bool ctrl = name.rfind("ctrl+", 0) == 0;
        const ImGuiKey k = keyFor(ctrl ? name.substr(5) : name);
        if (step_ == 0) {
            if (ctrl) io.AddKeyEvent(ImGuiMod_Ctrl, true);
            io.AddKeyEvent(k, true);
            step_ = 1;
            return;
        }
        io.AddKeyEvent(k, false);
        if (ctrl) io.AddKeyEvent(ImGuiMod_Ctrl, false);
        done(3);
        return;
    }
    if (cmd == "wait") {
        int n = 0;
        is >> n;
        done(std::max(0, n));
        return;
    }
    if (cmd == "shot") {
        is >> shot_name_;
        if (shot_name_.empty()) shot_name_ = "shot";
        return; // Taken in afterRender, then the command completes
    }
    if (cmd == "quit") {
        if (request_close) *request_close = true;
        done();
        return;
    }
    done(0); // Unknown: skip
}

void UiRemote::afterRender(int w, int h) {
    if (shot_name_.empty() || w <= 0 || h <= 0) return;
    std::vector<uint8_t> px(static_cast<size_t>(w) * static_cast<size_t>(h) * 4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    writePng((std::filesystem::path(dir_) / "ui_remote_shots" / (shot_name_ + ".png")).string(), w, h, px);
    shot_name_.clear();
    ++next_;
    step_ = 0;
    wait_frames_ = 1;
    writeStatus();
}

} // namespace rnr::ui
