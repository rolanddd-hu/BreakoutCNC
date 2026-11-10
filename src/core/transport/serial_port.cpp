#include "core/transport/serial_port.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <setupapi.h>
#pragma comment(lib, "setupapi.lib")
#else
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#endif

namespace rnr::transport {

SerialPort::~SerialPort() {
    close();
}

#ifdef _WIN32
// ============================================================================
// Windows: overlapped I/O, so a blocking read on the reader thread never delays a write (E-stop, real-time bytes)
// ============================================================================
bool SerialPort::open(const ConnectionTarget& target, const StreamOptions& options, std::string* error) {
    close();
    options_ = options;
    const std::string& port = target.address;
    const std::string full_name = "\\\\.\\" + port;
    HANDLE h = CreateFileA(full_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                           FILE_FLAG_OVERLAPPED, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        const DWORD err = GetLastError();
        if (error) {
            *error = "Cannot open " + port +
                     (err == ERROR_ACCESS_DENIED ? " (in use by another program)"
                      : err == ERROR_FILE_NOT_FOUND ? " (port not found)" : "");
        }
        return false;
    }
    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        if (error) *error = "Cannot read settings of " + port;
        return false;
    }
    dcb.BaudRate = options.baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = options.dtr ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fRtsControl = options.rts ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;
    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        if (error) *error = "Cannot configure " + port + " (8N1, " + std::to_string(options.baud) + " baud)";
        return false;
    }
    // A read returns as soon as at least one byte has arrived, or after read_timeout_ms with nothing
    COMMTIMEOUTS t{};
    t.ReadIntervalTimeout = MAXDWORD;
    t.ReadTotalTimeoutMultiplier = MAXDWORD;
    t.ReadTotalTimeoutConstant = std::max<DWORD>(1, options.read_timeout_ms);
    t.WriteTotalTimeoutMultiplier = 2;
    t.WriteTotalTimeoutConstant = 500;
    if (!SetCommTimeouts(h, &t)) {
        CloseHandle(h);
        if (error) *error = "Cannot set timeouts on " + port;
        return false;
    }
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    handle_ = h;
    return true;
}

void SerialPort::close() {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (handle_) {
        CancelIoEx(static_cast<HANDLE>(handle_), nullptr);
        CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = nullptr;
    }
}

bool SerialPort::isOpen() const {
    return handle_ != nullptr;
}

long SerialPort::read(uint8_t* buf, size_t max, uint32_t timeout_ms) {
    HANDLE h = static_cast<HANDLE>(handle_);
    if (!h) return -1;
    OVERLAPPED ov{};
    ov.hEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    if (!ov.hEvent) return -1;
    DWORD got = 0;
    long result = 0;
    if (ReadFile(h, buf, static_cast<DWORD>(max), &got, &ov)) {
        result = static_cast<long>(got);
    } else if (GetLastError() == ERROR_IO_PENDING) {
        const DWORD wait = WaitForSingleObject(ov.hEvent, timeout_ms + options_.read_timeout_ms + 50);
        if (wait != WAIT_OBJECT_0) {
            CancelIoEx(h, &ov);
        }
        if (GetOverlappedResult(h, &ov, &got, TRUE)) {
            result = static_cast<long>(got);
        } else {
            const DWORD err = GetLastError();
            result = (err == ERROR_OPERATION_ABORTED) ? 0 : -1;
        }
    } else {
        result = -1;
    }
    CloseHandle(ov.hEvent);
    if (result == 0 && got == 0) {
        // The device may have gone away: a removed USB adapter keeps returning 0 bytes on some drivers
        DWORD errors = 0;
        COMSTAT stat{};
        if (!ClearCommError(h, &errors, &stat)) return -1;
    }
    return result;
}

bool SerialPort::write(const uint8_t* data, size_t n) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    HANDLE h = static_cast<HANDLE>(handle_);
    if (!h) return false;
    OVERLAPPED ov{};
    ov.hEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    if (!ov.hEvent) return false;
    DWORD written = 0;
    bool ok = false;
    if (WriteFile(h, data, static_cast<DWORD>(n), &written, &ov)) {
        ok = written == n;
    } else if (GetLastError() == ERROR_IO_PENDING) {
        ok = GetOverlappedResult(h, &ov, &written, TRUE) && written == n;
    }
    CloseHandle(ov.hEvent);
    if (ok && options_.drain_writes) FlushFileBuffers(h);
    return ok;
}

void SerialPort::flushInput() {
    if (handle_) PurgeComm(static_cast<HANDLE>(handle_), PURGE_RXCLEAR | PURGE_TXCLEAR);
}

void SerialPort::setControlLines(bool dtr, bool rts) {
    HANDLE h = static_cast<HANDLE>(handle_);
    if (!h) return;
    EscapeCommFunction(h, dtr ? SETDTR : CLRDTR);
    EscapeCommFunction(h, rts ? SETRTS : CLRRTS);
}

std::vector<SerialPortInfo> SerialPort::enumerate() {
    // Ports device setup class {4D36E978-E325-11CE-BFC1-08002BE10318}
    static const GUID kPortsClass = {0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};
    std::vector<SerialPortInfo> ports;
    char target_path[512];
    for (int i = 1; i <= 128; ++i) {
        const std::string name = "COM" + std::to_string(i);
        if (QueryDosDeviceA(name.c_str(), target_path, sizeof(target_path)) != 0) {
            ports.push_back({name, ""});
        }
    }
    // Friendly names ("USB-SERIAL CH340 (COM5)") from the Ports device class
    HDEVINFO devs = SetupDiGetClassDevsA(&kPortsClass, nullptr, nullptr, DIGCF_PRESENT);
    if (devs != INVALID_HANDLE_VALUE) {
        SP_DEVINFO_DATA info{};
        info.cbSize = sizeof(info);
        for (DWORD i = 0; SetupDiEnumDeviceInfo(devs, i, &info); ++i) {
            char friendly[256] = {};
            if (!SetupDiGetDeviceRegistryPropertyA(devs, &info, SPDRP_FRIENDLYNAME, nullptr,
                                                   reinterpret_cast<PBYTE>(friendly), sizeof(friendly) - 1, nullptr)) {
                continue;
            }
            const std::string f = friendly;
            const size_t open_paren = f.rfind("(COM");
            if (open_paren == std::string::npos) continue;
            const size_t close_paren = f.find(')', open_paren);
            const std::string com = f.substr(open_paren + 1, close_paren - open_paren - 1);
            std::string desc = f.substr(0, open_paren);
            while (!desc.empty() && desc.back() == ' ') desc.pop_back();
            for (auto& p : ports) {
                if (p.name == com) p.description = desc;
            }
        }
        SetupDiDestroyDeviceInfoList(devs);
    }
    return ports;
}

#else
// ============================================================================
// POSIX (Linux, macOS): termios, poll() for read timeouts
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
#ifdef B230400
        case 230400: speed = B230400; return true;
#endif
#ifdef B250000
        case 250000: speed = B250000; return true;
#endif
#ifdef B460800
        case 460800: speed = B460800; return true;
#endif
#ifdef B921600
        case 921600: speed = B921600; return true;
#endif
        default:     return false;
    }
}
} // namespace

bool SerialPort::open(const ConnectionTarget& target, const StreamOptions& options, std::string* error) {
    close();
    options_ = options;
    const std::string& port = target.address;
    speed_t speed{};
    if (!toTermiosSpeed(options.baud, speed)) {
        if (error) *error = "Unsupported baud rate " + std::to_string(options.baud);
        return false;
    }
    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        const int err = errno;
        if (error) {
            *error = "Cannot open " + port + ": " + std::strerror(err) +
                     (err == EACCES ? " (add your user to the 'dialout' or 'uucp' group)" : "");
        }
        return false;
    }
    ioctl(fd, TIOCEXCL);
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        if (error) *error = "Cannot read settings of " + port + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_cflag &= ~HUPCL; // Do not drop DTR on close (that resets an Arduino)
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        if (error) *error = "Cannot configure " + port + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }
    const int flags = fcntl(fd, F_GETFL);
    if (flags >= 0) fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    int set = 0, clr = 0;
    (options.dtr ? set : clr) |= TIOCM_DTR;
    (options.rts ? set : clr) |= TIOCM_RTS;
    if (set) ioctl(fd, TIOCMBIS, &set);
    if (clr) ioctl(fd, TIOCMBIC, &clr);
    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    return true;
}

void SerialPort::close() {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

bool SerialPort::isOpen() const {
    return fd_ >= 0;
}

long SerialPort::read(uint8_t* buf, size_t max, uint32_t timeout_ms) {
    const int fd = fd_;
    if (fd < 0) return -1;
    pollfd p{fd, POLLIN, 0};
    const int r = ::poll(&p, 1, static_cast<int>(timeout_ms));
    if (r < 0) return errno == EINTR ? 0 : -1;
    if (r == 0) return 0;
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
    const ssize_t n = ::read(fd, buf, max);
    if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    if (n == 0) return -1; // Readable with no data: the device is gone
    return static_cast<long>(n);
}

bool SerialPort::write(const uint8_t* data, size_t n) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (fd_ < 0) return false;
    size_t written = 0;
    while (written < n) {
        const ssize_t w = ::write(fd_, data + written, n - written);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        written += static_cast<size_t>(w);
    }
    if (options_.drain_writes) tcdrain(fd_);
    return true;
}

void SerialPort::flushInput() {
    if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
}

void SerialPort::setControlLines(bool dtr, bool rts) {
    if (fd_ < 0) return;
    int bit = TIOCM_DTR;
    ioctl(fd_, dtr ? TIOCMBIS : TIOCMBIC, &bit);
    bit = TIOCM_RTS;
    ioctl(fd_, rts ? TIOCMBIS : TIOCMBIC, &bit);
}

std::vector<SerialPortInfo> SerialPort::enumerate() {
    std::vector<SerialPortInfo> ports;
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
                    ports.push_back({"/dev/" + name, ""});
                }
            }
        }
        closedir(dir);
    }
    std::sort(ports.begin(), ports.end(), [](const SerialPortInfo& a, const SerialPortInfo& b) { return a.name < b.name; });
    return ports;
}
#endif

} // namespace rnr::transport
