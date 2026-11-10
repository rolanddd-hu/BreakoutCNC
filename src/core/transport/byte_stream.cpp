#include "core/transport/byte_stream.h"
#include "core/transport/serial_port.h"
#include "core/transport/tcp_stream.h"
#include <map>
#include <mutex>

namespace rnr::transport {

namespace {
std::mutex& factoryMutex() {
    static std::mutex m;
    return m;
}
std::map<std::string, StreamFactory>& factories() {
    static std::map<std::string, StreamFactory> f;
    return f;
}
} // namespace

std::unique_ptr<IByteStream> createByteStream(const std::string& transport) {
    {
        std::lock_guard<std::mutex> lock(factoryMutex());
        const auto it = factories().find(transport);
        if (it != factories().end()) return it->second();
    }
    if (transport == "tcp") return std::make_unique<TcpStream>();
    if (transport == "serial" || transport.empty()) return std::make_unique<SerialPort>();
    return nullptr;
}

void registerStreamFactory(const std::string& transport, StreamFactory factory) {
    std::lock_guard<std::mutex> lock(factoryMutex());
    factories()[transport] = std::move(factory);
}

void unregisterStreamFactory(const std::string& transport) {
    std::lock_guard<std::mutex> lock(factoryMutex());
    factories().erase(transport);
}

} // namespace rnr::transport
