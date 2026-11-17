#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Byte-stream transports for text protocols (serial ports, TCP). See MULTI_BOARD_PLAN.md section 4.11.
namespace rnr::transport {

struct ConnectionTarget {
    std::string transport; // "serial", "tcp", or a registered test transport
    std::string address;   // "COM5", "/dev/ttyUSB0", "192.168.1.20:23"
    std::string label;     // Shown in the UI: "COM5 - USB-SERIAL CH340"
};

struct StreamOptions {
    uint32_t baud = 115200;
    bool     dtr = true;            // Serial control lines after opening
    bool     rts = true;
    uint32_t read_timeout_ms = 50;  // Upper bound for one read() call
    bool     drain_writes = false;  // Wait until written bytes have left the UART (RS-485 turnaround)
    uint16_t default_tcp_port = 23;
};

class IByteStream {
public:
    virtual ~IByteStream() = default;
    virtual bool open(const ConnectionTarget& target, const StreamOptions& options, std::string* error) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    // > 0: bytes read; 0: nothing within timeout_ms; < 0: the link failed (cable unplugged, connection closed)
    virtual long read(uint8_t* buf, size_t max, uint32_t timeout_ms) = 0;
    // All or nothing; false = the link failed
    virtual bool write(const uint8_t* data, size_t n) = 0;
    virtual void flushInput() {}
    virtual void setControlLines(bool dtr, bool rts) { (void)dtr; (void)rts; }
    // True when this stream reaches real hardware (unit tests never open those)
    virtual bool accessesRealHardware() const { return true; }
};

using StreamFactory = std::function<std::unique_ptr<IByteStream>()>;

// Creates a stream for a transport name ("serial", "tcp", or one registered with registerStreamFactory)
std::unique_ptr<IByteStream> createByteStream(const std::string& transport);

// Test hook: an in-process stream (e.g. the FakeGrbl emulator) under its own transport name
void registerStreamFactory(const std::string& transport, StreamFactory factory);
void unregisterStreamFactory(const std::string& transport);

} // namespace rnr::transport
