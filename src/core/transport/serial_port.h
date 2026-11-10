#pragma once
#include "core/transport/byte_stream.h"
#include <mutex>
#include <string>
#include <vector>

namespace rnr::transport {

struct SerialPortInfo {
    std::string name;        // "COM5" / "/dev/ttyUSB0"
    std::string description; // "USB-SERIAL CH340" when the system knows it
};

// One serial port implementation for Windows, Linux and macOS (the Huanyang VFD link and GRBL-family boards).
// Reads and writes may run on different threads; writes are serialized.
class SerialPort final : public IByteStream {
public:
    SerialPort() = default;
    ~SerialPort() override;
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const ConnectionTarget& target, const StreamOptions& options, std::string* error) override;
    bool open(const std::string& port, const StreamOptions& options, std::string* error) {
        return open(ConnectionTarget{"serial", port, port}, options, error);
    }
    void close() override;
    bool isOpen() const override;
    long read(uint8_t* buf, size_t max, uint32_t timeout_ms) override;
    bool write(const uint8_t* data, size_t n) override;
    void flushInput() override;
    void setControlLines(bool dtr, bool rts) override;

    // Ports present on this computer, sorted, with descriptions where available
    static std::vector<SerialPortInfo> enumerate();

private:
#ifdef _WIN32
    void* handle_ = nullptr;
#else
    int fd_ = -1;
#endif
    StreamOptions options_{};
    std::mutex write_mutex_;
};

} // namespace rnr::transport
