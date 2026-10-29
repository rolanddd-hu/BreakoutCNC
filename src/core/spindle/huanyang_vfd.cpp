#include "core/spindle/huanyang_vfd.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#endif

namespace rnr {

namespace {
// The VFD counts as connected while it answered within this window
constexpr auto kConnectedWindow = std::chrono::seconds(3);

// Status codes polled round-robin (function 0x04): Out Freq, Out Amps, RPM, DC bus V, AC out V, Temp
constexpr uint8_t kStatusPollCodes[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x07};
} // namespace

// ============================================================================
// Inverter Parameters & Speed Monitor
// ============================================================================
uint16_t HuanyangVfdParams::maxRpm() const noexcept {
    return static_cast<uint16_t>(std::clamp(std::lround(pd144_rpm_at_50hz * pd005_max_hz / 50.0f), 0L, 65535L));
}

uint16_t HuanyangVfdParams::minRpm() const noexcept {
    return static_cast<uint16_t>(std::clamp(std::lround(pd144_rpm_at_50hz * pd011_min_hz / 50.0f), 0L, 65535L));
}

void VfdSpeedMonitor::reset() noexcept {
    target_      = 0;
    ramping_     = false;
    last_dist_   = -1;
    no_progress_ = 0;
    bad_speed_   = 0;
}

std::string VfdSpeedMonitor::onRpmReading(bool running, uint16_t target_rpm, uint16_t actual_rpm,
                                          const HuanyangVfdConfig& cfg) {
    if (!cfg.speed_monitor_enabled || !running || target_rpm == 0) {
        reset();
        return {};
    }
    const double margin = cfg.speed_margin_pct / 100.0;
    const bool in_band = actual_rpm >= target_rpm * (1.0 - margin) && actual_rpm <= target_rpm * (1.0 + margin);
    const int dist = std::abs(static_cast<int>(actual_rpm) - static_cast<int>(target_rpm));

    if (target_rpm != target_) {
        // New target: the speed has to move towards it from here on
        target_      = target_rpm;
        ramping_     = !in_band;
        last_dist_   = dist;
        no_progress_ = 0;
        bad_speed_   = 0;
        return {};
    }

    char msg[128];
    if (ramping_) {
        if (in_band) {
            ramping_ = false;
            return {};
        }
        no_progress_ = (dist >= last_dist_) ? no_progress_ + 1 : 0;
        last_dist_   = dist;
        if (no_progress_ > cfg.tracking_threshold) {
            std::snprintf(msg, sizeof(msg), "Spindle speed not tracking: %u RPM, commanded %u RPM",
                          actual_rpm, target_rpm);
            reset();
            return msg;
        }
        return {};
    }

    if (in_band) {
        bad_speed_ = 0;
        return {};
    }
    if (++bad_speed_ > cfg.bad_speed_threshold) {
        std::snprintf(msg, sizeof(msg), "Bad spindle speed: %u RPM, commanded %u RPM (+/-%u%%)",
                      actual_rpm, target_rpm, static_cast<unsigned>(cfg.speed_margin_pct));
        reset();
        return msg;
    }
    return {};
}

// ============================================================================
// Frame Helpers
// ============================================================================
void HuanyangVfdController::buildParameterRead(uint8_t slave, uint8_t pd, uint8_t out[8]) {
    out[0] = slave;
    out[1] = 0x01;
    out[2] = 0x03;
    out[3] = pd;
    out[4] = 0x00;
    out[5] = 0x00;
    const uint16_t crc = calculateCrc16(out, 6);
    out[6] = static_cast<uint8_t>(crc & 0xFF);
    out[7] = static_cast<uint8_t>(crc >> 8);
}

bool HuanyangVfdController::decodeParameterReply(const uint8_t* rx, size_t len, uint8_t slave, uint8_t pd,
                                                 uint16_t& value) {
    if (len < 8 || rx[0] != slave || rx[1] != 0x01 || rx[3] != pd) return false;
    const uint16_t crc = calculateCrc16(rx, 6);
    if (rx[6] != static_cast<uint8_t>(crc & 0xFF) || rx[7] != static_cast<uint8_t>(crc >> 8)) return false;
    value = static_cast<uint16_t>((static_cast<uint16_t>(rx[4]) << 8) | rx[5]);
    return true;
}

float HuanyangVfdController::scaleStatusValue(uint8_t status_code, uint16_t raw) {
    // Decimal places per status code, as in the Mach3 vfdPlugin v2 table {2, 2, 1, 0, 1, 1, 0, 1}
    switch (status_code) {
        case 0x00:
        case 0x01: return raw / 100.0f;          // Set / output frequency (Hz)
        case 0x02:
        case 0x04:
        case 0x05:
        case 0x07: return raw / 10.0f;           // Current (A), DC / AC voltage (V), temperature (C)
        default:   return static_cast<float>(raw); // RPM, counter
    }
}

std::string HuanyangVfdController::checkReadingLimit(uint8_t status_code, float value, const HuanyangVfdConfig& cfg) {
    float limit = 0.0f;
    const char* what = "";
    const char* unit = "";
    switch (status_code) {
        case 0x01: limit = cfg.limit_out_freq_hz; what = "output frequency"; unit = "Hz";  break;
        case 0x02: limit = cfg.limit_current_a;   what = "current";          unit = "A";   break;
        case 0x03: limit = cfg.limit_rpm;         what = "speed";            unit = "RPM"; break;
        case 0x04: limit = cfg.limit_dc_volts;    what = "DC bus voltage";   unit = "V";   break;
        case 0x05: limit = cfg.limit_ac_volts;    what = "output voltage";   unit = "V";   break;
        case 0x07: limit = cfg.limit_temp_c;      what = "temperature";      unit = "C";   break;
        default: return {};
    }
    if (!(limit > 0.0f) || !(value > limit)) return {};
    char msg[128];
    std::snprintf(msg, sizeof(msg), "VFD %s %.1f %s is above the %.1f %s limit", what, value, unit, limit, unit);
    return msg;
}

// ============================================================================
// Constructor & Destructor
// ============================================================================
HuanyangVfdController::HuanyangVfdController() {
    last_tx_time_ = std::chrono::steady_clock::now();
    last_successful_rx_time_ = std::chrono::steady_clock::now();
}

HuanyangVfdController::~HuanyangVfdController() {
    stop();
}

// ============================================================================
// Modbus CRC-16 Calculation (Polynomial 0xA001)
// ============================================================================
uint16_t HuanyangVfdController::calculateCrc16(const uint8_t* buffer, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= buffer[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Enumerate Available Serial Ports
// ============================================================================
std::vector<std::string> HuanyangVfdController::enumeratePorts() {
    std::vector<std::string> ports;
#ifdef _WIN32
    char target_path[512];
    for (int i = 1; i <= 64; ++i) {
        std::string port_name = "COM" + std::to_string(i);
        if (QueryDosDeviceA(port_name.c_str(), target_path, sizeof(target_path)) != 0) {
            ports.push_back(port_name);
        }
    }
#else
    // USB RS-485 adapters: ttyUSB* / ttyACM* on Linux, cu.* on macOS (cu = call-out, no carrier wait)
#ifdef __APPLE__
    const char* prefixes[] = {"cu."};
#else
    const char* prefixes[] = {"ttyUSB", "ttyACM"};
#endif
    if (DIR* dir = opendir("/dev")) {
        while (const dirent* entry = readdir(dir)) {
            const std::string name = entry->d_name;
            for (const char* prefix : prefixes) {
                if (name.rfind(prefix, 0) == 0 && name != "cu.Bluetooth-Incoming-Port") {
                    ports.push_back("/dev/" + name);
                }
            }
        }
        closedir(dir);
    }
    std::sort(ports.begin(), ports.end());
#endif
    return ports;
}

// ============================================================================
// Start / Stop Controller Thread
// ============================================================================
bool HuanyangVfdController::start(const HuanyangVfdConfig& config) {
    stop();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        telemetry_ = HuanyangVfdTelemetry{};
        commanded_run_ = false;
        run_control_byte_ = 0x08;
        run_dirty_ = true; // Make sure the spindle is commanded OFF on connect
        freq_dirty_ = false;
        params_requested_ = false;
        speed_monitor_.reset();
    }

    if (!config.enabled) {
        return false;
    }

    reopen_requested_ = false;
    running_ = true;
    worker_thread_ = std::thread(&HuanyangVfdController::workerLoop, this);
    return true;
}

void HuanyangVfdController::stop() {
    if (running_.exchange(false)) {
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }
    closeSerialPort(); // Worker has exited: safe to touch the handle here

    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.connected = false;
    telemetry_.port_open = false;
    telemetry_.spindle_running = false;
}

bool HuanyangVfdController::isRunning() const {
    return running_.load();
}

bool HuanyangVfdController::isConnected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_.connected;
}

void HuanyangVfdController::updateConfig(const HuanyangVfdConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool restart_needed = (config_.port_name != config.port_name ||
                                 config_.baud_rate != config.baud_rate ||
                                 config_.enabled != config.enabled);
    config_ = config;
    if (restart_needed && running_) {
        // The worker owns the serial handle: ask it to reopen instead of closing it from this thread
        reopen_requested_ = true;
    }
}

HuanyangVfdConfig HuanyangVfdController::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

HuanyangVfdTelemetry HuanyangVfdController::getTelemetry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_;
}

void HuanyangVfdController::clearAlarms() {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.alarm_stall_tripped = false;
    telemetry_.alarm_comm_loss_tripped = false;
    telemetry_.alarm_speed_tripped = false;
    telemetry_.alarm_limit_tripped = false;
    speed_monitor_.reset();
}

void HuanyangVfdController::requestParameterRead() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        telemetry_.params.busy  = false;
        telemetry_.params.error = "Huanyang VFD control is not enabled";
        return;
    }
    params_requested_ = true;
    telemetry_.params.busy = true;
    telemetry_.params.error.clear();
}

void HuanyangVfdController::readParameters(uint8_t slave) {
    struct Item { uint8_t pd; uint16_t value; };
    Item items[] = {{1, 0}, {2, 0}, {5, 0}, {11, 0}, {144, 0}};
    std::string error;
    for (auto& it : items) {
        uint8_t tx[8];
        buildParameterRead(slave, it.pd, tx);
        uint8_t rx[16];
        size_t rx_len = 0;
        if (!sendPacketAndReceive(tx, 8, rx, rx_len, 8) || !decodeParameterReply(rx, rx_len, slave, it.pd, it.value)) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "No valid reply reading PD%03u", static_cast<unsigned>(it.pd));
            error = buf;
            break;
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto& p = telemetry_.params;
    p.busy  = false;
    p.valid = error.empty();
    p.error = error;
    if (p.valid) {
        p.pd001_run_source  = items[0].value;
        p.pd002_freq_source = items[1].value;
        p.pd005_max_hz      = items[2].value / 100.0f;
        p.pd011_min_hz      = items[3].value / 100.0f;
        p.pd144_rpm_at_50hz = items[4].value;
    }
}

// ============================================================================
// Spindle Commands (latest-state, retried until acknowledged)
// ============================================================================
void HuanyangVfdController::setRun(bool run, bool reverse) {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint8_t byte = !run ? 0x08 : (reverse ? 0x11 : 0x01); // Stop / Run Rev / Run Fwd
    if (byte != run_control_byte_ || !run_dirty_) {
        run_control_byte_ = byte;
        run_dirty_ = true;
    }
    if (run && !commanded_run_) {
        run_command_time_ = std::chrono::steady_clock::now();
    }
    commanded_run_ = run;
}

void HuanyangVfdController::setRpm(uint16_t rpm) {
    std::lock_guard<std::mutex> lock(mutex_);
    uint16_t clamped_rpm = rpm;
    if (clamped_rpm > config_.max_rpm) clamped_rpm = config_.max_rpm;
    if (clamped_rpm > 0 && clamped_rpm < config_.min_rpm) clamped_rpm = config_.min_rpm;

    telemetry_.target_rpm = clamped_rpm;

    // Convert RPM to hundredths of Hz: (rpm / max_rpm) * (max_freq_hz * 100)
    const float ratio = (config_.max_rpm > 0) ? (static_cast<float>(clamped_rpm) / config_.max_rpm) : 0.0f;
    const float target_hz = ratio * config_.max_freq_hz;
    telemetry_.target_freq_hz = target_hz;

    const uint16_t freq_raw = static_cast<uint16_t>(std::clamp(target_hz * 100.0f + 0.5f, 0.0f, 65535.0f));
    if (freq_raw != freq_raw_ || !freq_dirty_) {
        freq_raw_ = freq_raw;
        freq_dirty_ = true;
    }
}

void HuanyangVfdController::emergencyStop() {
    std::lock_guard<std::mutex> lock(mutex_);
    run_control_byte_ = 0x08;
    run_dirty_ = true;
    commanded_run_ = false;
}

// ============================================================================
// Serial Port Operations (platform independent part)
// ============================================================================
bool HuanyangVfdController::openSerialPort() {
    closeSerialPort();
    std::string port_name;
    uint32_t baud = 9600;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        port_name = config_.port_name;
        baud = config_.baud_rate;
    }

    std::string error;
    const bool ok = openPlatformPort(port_name, baud, error);

    std::lock_guard<std::mutex> lock(mutex_);
    if (!ok) {
        telemetry_.connected = false;
        telemetry_.port_open = false;
        telemetry_.last_error_msg = error;
        return false;
    }
    last_tx_time_ = std::chrono::steady_clock::now();
    telemetry_.port_open = true;
    // `connected` only becomes true once the VFD actually answers a request
    telemetry_.last_error_msg.clear();
    return true;
}

void HuanyangVfdController::closeSerialPort() {
    if (platformPortOpen()) {
        flushPlatformPort();
        closePlatformPort();
    }
}

// ============================================================================
// Packet Transmission & Reception
// ============================================================================
bool HuanyangVfdController::sendPacketAndReceive(const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t& rx_len, size_t expected_len) {
    rx_len = 0;
    if (!platformPortOpen()) return false;

    // Enforce a 60ms quiet time between frames (same pacing as the Mach3 plugin)
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tx_time_).count();
    if (elapsed_ms < 60) {
        std::this_thread::sleep_for(std::chrono::milliseconds(60 - elapsed_ms));
    }

    flushPlatformPort();

    if (!writePlatformPort(tx, tx_len)) {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.comm_errors++;
        telemetry_.last_error_msg = "Serial write failure";
        return false;
    }

    last_tx_time_ = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.packets_sent++;
    }

    uint8_t buffer[64];
    expected_len = std::min(expected_len, sizeof(buffer));
    size_t total_read = 0;
    const auto start_read = std::chrono::steady_clock::now();
    while (total_read < expected_len) {
        const size_t bytes_read = readPlatformPort(buffer + total_read, expected_len - total_read);
        if (bytes_read > 0) {
            total_read += bytes_read;
        } else if (std::chrono::steady_clock::now() - start_read > std::chrono::milliseconds(250)) {
            break;
        }
    }

    if (total_read < expected_len) {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.comm_errors++;
        telemetry_.last_error_msg = "RX timeout: read " + std::to_string(total_read) + "/" + std::to_string(expected_len);
        return false;
    }

    const uint16_t expected_crc = calculateCrc16(buffer, total_read - 2);
    const uint16_t received_crc = buffer[total_read - 2] | (static_cast<uint16_t>(buffer[total_read - 1]) << 8);
    // The reply must come from the addressed slave and echo the function code
    if (expected_crc != received_crc || buffer[0] != tx[0] || buffer[1] != tx[1]) {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.comm_errors++;
        telemetry_.last_error_msg = (expected_crc != received_crc) ? "CRC mismatch" : "Unexpected reply";
        return false;
    }

    std::memcpy(rx, buffer, total_read);
    rx_len = total_read;
    last_successful_rx_time_ = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_.packets_received++;
        telemetry_.connected = true;
    }
    return true;
}

#ifdef _WIN32
// ============================================================================
// Raw Port I/O (Win32 API)
// ============================================================================
bool HuanyangVfdController::openPlatformPort(const std::string& port_name, uint32_t baud, std::string& error) {
    const std::string full_name = "\\\\.\\" + port_name;
    HANDLE hComm = CreateFileA(full_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hComm == INVALID_HANDLE_VALUE) {
        error = "Cannot open " + port_name;
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hComm, &dcb)) {
        CloseHandle(hComm);
        error = "Cannot read settings of " + port_name;
        return false;
    }
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fAbortOnError = FALSE;
    if (!SetCommState(hComm, &dcb)) {
        CloseHandle(hComm);
        error = "Cannot configure " + port_name + " (8N1, " + std::to_string(baud) + " baud)";
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = 20;
    timeouts.ReadTotalTimeoutMultiplier = 5;
    timeouts.ReadTotalTimeoutConstant = 150;
    timeouts.WriteTotalTimeoutMultiplier = 5;
    timeouts.WriteTotalTimeoutConstant = 100;
    if (!SetCommTimeouts(hComm, &timeouts)) {
        CloseHandle(hComm);
        error = "Cannot set timeouts on " + port_name;
        return false;
    }

    PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
    serial_handle_ = static_cast<void*>(hComm);
    return true;
}

void HuanyangVfdController::closePlatformPort() {
    if (serial_handle_) CloseHandle(static_cast<HANDLE>(serial_handle_));
    serial_handle_ = nullptr;
}

bool HuanyangVfdController::platformPortOpen() const {
    return serial_handle_ != nullptr;
}

void HuanyangVfdController::flushPlatformPort() {
    PurgeComm(static_cast<HANDLE>(serial_handle_), PURGE_RXCLEAR | PURGE_TXCLEAR);
}

bool HuanyangVfdController::writePlatformPort(const uint8_t* data, size_t len) {
    DWORD bytes_written = 0;
    return WriteFile(static_cast<HANDLE>(serial_handle_), data, static_cast<DWORD>(len), &bytes_written, NULL) &&
           bytes_written == len;
}

size_t HuanyangVfdController::readPlatformPort(uint8_t* data, size_t max_len) {
    DWORD bytes_read = 0;
    if (!ReadFile(static_cast<HANDLE>(serial_handle_), data, static_cast<DWORD>(max_len), &bytes_read, NULL)) {
        bytes_read = 0;
    }
    if (bytes_read == 0) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return bytes_read;
}

#else
// ============================================================================
// Raw Port I/O (POSIX termios: Linux, macOS)
// ============================================================================
namespace {
bool toTermiosSpeed(uint32_t baud, speed_t& speed) {
    switch (baud) {
        case 1200:   speed = B1200;   return true;
        case 2400:   speed = B2400;   return true;
        case 4800:   speed = B4800;   return true;
        case 9600:   speed = B9600;   return true;
        case 19200:  speed = B19200;  return true;
        case 38400:  speed = B38400;  return true;
        case 57600:  speed = B57600;  return true;
        case 115200: speed = B115200; return true;
        default:     return false;
    }
}
} // namespace

bool HuanyangVfdController::openPlatformPort(const std::string& port_name, uint32_t baud, std::string& error) {
    speed_t speed{};
    if (!toTermiosSpeed(baud, speed)) {
        error = "Unsupported baud rate " + std::to_string(baud);
        return false;
    }

    // O_NONBLOCK so the open does not wait for a carrier signal; blocking mode is restored below
    const int fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        const int err = errno;
        error = "Cannot open " + port_name + ": " + std::strerror(err) +
                (err == EACCES ? " (add your user to the 'dialout' or 'uucp' group)" : "");
        return false;
    }
    // Exclusive access, like the Win32 share mode 0
    ioctl(fd, TIOCEXCL);

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        error = "Cannot read settings of " + port_name + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }
    // Raw 8N1, no flow control
    cfmakeraw(&tio);
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    // read() returns what has arrived, or 0 after 100 ms of silence
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        error = "Cannot configure " + port_name + " (8N1, " + std::to_string(baud) + " baud): " + std::strerror(errno);
        ::close(fd);
        return false;
    }

    const int flags = fcntl(fd, F_GETFL);
    if (flags >= 0) fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    // DTR and RTS on, as on Windows (some adapters power their RS-485 transceiver from them)
    int lines = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd, TIOCMBIS, &lines);

    tcflush(fd, TCIOFLUSH);
    serial_fd_ = fd;
    return true;
}

void HuanyangVfdController::closePlatformPort() {
    if (serial_fd_ >= 0) ::close(serial_fd_);
    serial_fd_ = -1;
}

bool HuanyangVfdController::platformPortOpen() const {
    return serial_fd_ >= 0;
}

void HuanyangVfdController::flushPlatformPort() {
    tcflush(serial_fd_, TCIOFLUSH);
}

bool HuanyangVfdController::writePlatformPort(const uint8_t* data, size_t len) {
    size_t written = 0;
    while (written < len) {
        const ssize_t n = ::write(serial_fd_, data + written, len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        written += static_cast<size_t>(n);
    }
    // Wait until the frame has left the UART so the reply timeout starts at the end of the request
    tcdrain(serial_fd_);
    return true;
}

size_t HuanyangVfdController::readPlatformPort(uint8_t* data, size_t max_len) {
    const ssize_t n = ::read(serial_fd_, data, max_len); // VTIME: returns 0 after 100 ms without data
    return n > 0 ? static_cast<size_t>(n) : 0;
}
#endif

// ============================================================================
// Worker Thread Loop
// ============================================================================
void HuanyangVfdController::workerLoop() {
    uint8_t poll_step = 0;
    last_successful_rx_time_ = std::chrono::steady_clock::now();

    while (running_) {
        if (reopen_requested_.exchange(false)) {
            closeSerialPort();
        }
        if (!platformPortOpen()) {
            if (!openSerialPort()) {
                for (int i = 0; i < 20 && running_; ++i) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                continue;
            }
        }

        uint8_t slave = 1;
        bool send_run = false, send_freq = false, read_params = false;
        uint8_t control_byte = 0x08;
        uint16_t freq_raw = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            slave = config_.slave_address;
            send_run = run_dirty_;
            control_byte = run_control_byte_;
            send_freq = freq_dirty_;
            freq_raw = freq_raw_;
            read_params = params_requested_;
            params_requested_ = false;
        }

        if (send_run) {
            // Run/Stop has priority (a stop must never wait behind frequency updates).
            // Function 0x03: [Slave] [0x03] [0x01] [Control] [CRC_Lo] [CRC_Hi]
            uint8_t tx[6];
            tx[0] = slave;
            tx[1] = 0x03;
            tx[2] = 0x01;
            tx[3] = control_byte;
            const uint16_t crc = calculateCrc16(tx, 4);
            tx[4] = static_cast<uint8_t>(crc & 0xFF);
            tx[5] = static_cast<uint8_t>(crc >> 8);

            uint8_t rx[16];
            size_t rx_len = 0;
            if (sendPacketAndReceive(tx, 6, rx, rx_len, 6)) {
                std::lock_guard<std::mutex> lock(mutex_);
                // Only clear the request if nothing newer arrived meanwhile; otherwise retry is automatic
                if (run_control_byte_ == control_byte) run_dirty_ = false;
                telemetry_.spindle_running = (control_byte != 0x08);
                telemetry_.spindle_direction_rev = (control_byte == 0x11);
            }
        } else if (send_freq) {
            // Function 0x05: [Slave] [0x05] [0x02] [Freq_Hi] [Freq_Lo] [CRC_Lo] [CRC_Hi]
            uint8_t tx[7];
            tx[0] = slave;
            tx[1] = 0x05;
            tx[2] = 0x02;
            tx[3] = static_cast<uint8_t>(freq_raw >> 8);
            tx[4] = static_cast<uint8_t>(freq_raw & 0xFF);
            const uint16_t crc = calculateCrc16(tx, 5);
            tx[5] = static_cast<uint8_t>(crc & 0xFF);
            tx[6] = static_cast<uint8_t>(crc >> 8);

            uint8_t rx[16];
            size_t rx_len = 0;
            if (sendPacketAndReceive(tx, 7, rx, rx_len, 7)) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (freq_raw_ == freq_raw) freq_dirty_ = false;
            }
        } else if (read_params) {
            readParameters(slave);
        } else {
            // Round-Robin Status Query (Function 0x04)
            constexpr size_t kPollCount = sizeof(kStatusPollCodes) / sizeof(kStatusPollCodes[0]);
            const uint8_t param_code = kStatusPollCodes[poll_step % kPollCount];
            poll_step++;

            uint8_t tx[6];
            tx[0] = slave;
            tx[1] = 0x04;
            tx[2] = 0x01;
            tx[3] = param_code;
            const uint16_t crc = calculateCrc16(tx, 4);
            tx[4] = static_cast<uint8_t>(crc & 0xFF);
            tx[5] = static_cast<uint8_t>(crc >> 8);

            uint8_t rx[16];
            size_t rx_len = 0;
            // Expected response: [Slave] [0x04] [0x02] [Val_Hi] [Val_Lo] [CRC_Lo] [CRC_Hi]
            if (sendPacketAndReceive(tx, 6, rx, rx_len, 7) && rx_len >= 5) {
                const uint16_t raw = (static_cast<uint16_t>(rx[3]) << 8) | rx[4];
                const float value = scaleStatusValue(param_code, raw);
                std::lock_guard<std::mutex> lock(mutex_);
                switch (param_code) {
                    case 0x01: telemetry_.actual_freq_hz = value; break;
                    case 0x02: telemetry_.actual_amps    = value; break;
                    case 0x03: telemetry_.actual_rpm     = raw;   break;
                    case 0x04: telemetry_.dc_bus_volts   = value; break;
                    case 0x05: telemetry_.ac_out_volts   = value; break;
                    case 0x07: telemetry_.temperature_c  = value; break;
                }
                const std::string limit_msg = checkReadingLimit(param_code, value, config_);
                if (!limit_msg.empty() && !telemetry_.alarm_limit_tripped) {
                    telemetry_.alarm_limit_tripped = true;
                    telemetry_.last_error_msg = limit_msg;
                }
                if (param_code == 0x03) {
                    const std::string speed_msg = speed_monitor_.onRpmReading(
                        commanded_run_ && telemetry_.spindle_running, telemetry_.target_rpm, raw, config_);
                    if (!speed_msg.empty() && !telemetry_.alarm_speed_tripped) {
                        telemetry_.alarm_speed_tripped = true;
                        telemetry_.last_error_msg = speed_msg;
                    }
                }
            }
        }

        // Safety supervision (stall & communication loss)
        const auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (now - last_successful_rx_time_ > kConnectedWindow) {
                telemetry_.connected = false;
            }
            if (config_.estop_on_stall && commanded_run_) {
                const auto run_dur_s = std::chrono::duration_cast<std::chrono::seconds>(now - run_command_time_).count();
                if (run_dur_s >= 3 && telemetry_.actual_rpm == 0 && telemetry_.target_rpm > 0) {
                    telemetry_.alarm_stall_tripped = true;
                    telemetry_.last_error_msg = "Spindle stall detected (0 RPM)";
                }
            }
            if (config_.estop_on_comm_loss && commanded_run_ && !telemetry_.connected) {
                telemetry_.alarm_comm_loss_tripped = true;
                telemetry_.last_error_msg = "VFD RS485 communication lost while running";
            }
        }

        // Idle delay between polling loops (commands are retried promptly)
        std::this_thread::sleep_for(std::chrono::milliseconds((send_run || send_freq) ? 5 : 50));
    }
    closeSerialPort();
}

} // namespace rnr
