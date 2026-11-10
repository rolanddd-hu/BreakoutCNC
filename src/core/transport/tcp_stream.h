#pragma once
#include "core/transport/byte_stream.h"
#include <atomic>
#include <cstdint>
#include <mutex>

namespace rnr::transport {

// TCP client stream (telnet-style raw socket, e.g. grblHAL / FluidNC on port 23). Address "host[:port]".
class TcpStream final : public IByteStream {
public:
    TcpStream();
    ~TcpStream() override;
    TcpStream(const TcpStream&) = delete;
    TcpStream& operator=(const TcpStream&) = delete;

    bool open(const ConnectionTarget& target, const StreamOptions& options, std::string* error) override;
    void close() override;
    bool isOpen() const override;
    long read(uint8_t* buf, size_t max, uint32_t timeout_ms) override;
    bool write(const uint8_t* data, size_t n) override;

private:
    std::atomic<intptr_t> sock_;
    std::mutex write_mutex_;
};

} // namespace rnr::transport
