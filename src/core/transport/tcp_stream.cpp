#include "core/transport/tcp_stream.h"
#include <cstring>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using SocketHandle = SOCKET;
constexpr intptr_t kNoSocket = static_cast<intptr_t>(INVALID_SOCKET);
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
using SocketHandle = int;
constexpr intptr_t kNoSocket = -1;
#endif

namespace rnr::transport {

namespace {
#ifdef _WIN32
struct WinsockInit {
    WinsockInit() {
        WSADATA d;
        WSAStartup(MAKEWORD(2, 2), &d);
    }
};
void ensureWinsock() {
    static WinsockInit init;
}
void closeSocket(SocketHandle s) { closesocket(s); }
#else
void ensureWinsock() {}
void closeSocket(SocketHandle s) { ::close(s); }
#endif

// Waits until the socket is readable (or writable). poll() on POSIX: select() with FD_SET is undefined for
// descriptors >= FD_SETSIZE (1024). Returns > 0 ready, 0 timeout, < 0 error.
int waitSocket(SocketHandle s, bool for_write, uint32_t timeout_ms) {
#ifdef _WIN32
    fd_set set;
    FD_ZERO(&set);
    FD_SET(s, &set);
    timeval tv{static_cast<long>(timeout_ms / 1000), static_cast<long>((timeout_ms % 1000) * 1000)};
    return ::select(0, for_write ? nullptr : &set, for_write ? &set : nullptr, nullptr, &tv);
#else
    pollfd p{s, static_cast<short>(for_write ? POLLOUT : POLLIN), 0};
    int r;
    do {
        r = ::poll(&p, 1, static_cast<int>(timeout_ms));
    } while (r < 0 && errno == EINTR);
    if (r > 0 && !for_write && (p.revents & (POLLERR | POLLNVAL))) return -1;
    return r;
#endif
}

// "host", "host:port", "[v6]", "[v6]:port" or a bare IPv6 address (no port)
void splitHostPort(const std::string& address, std::string& host, std::string& port) {
    host = address;
    if (!address.empty() && address[0] == '[') {
        const size_t close = address.find(']');
        if (close != std::string::npos) {
            host = address.substr(1, close - 1);
            if (close + 1 < address.size() && address[close + 1] == ':') port = address.substr(close + 2);
        }
        return;
    }
    const size_t colon = address.rfind(':');
    if (colon != std::string::npos && address.find(':') == colon) {
        port = address.substr(colon + 1);
        host = address.substr(0, colon);
    }
}
} // namespace

TcpStream::TcpStream() : sock_(kNoSocket) {}

TcpStream::~TcpStream() {
    close();
}

bool TcpStream::open(const ConnectionTarget& target, const StreamOptions& options, std::string* error) {
    close();
    ensureWinsock();
    std::string host;
    std::string port = std::to_string(options.default_tcp_port);
    splitHostPort(target.address, host, port);
    if (host.empty()) {
        if (error) *error = "No network address entered";
        return false;
    }
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
        if (error) *error = "Cannot resolve " + host;
        return false;
    }
    SocketHandle s = static_cast<SocketHandle>(kNoSocket);
    for (addrinfo* ai = res; ai; ai = ai->ai_next) {
        s = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (static_cast<intptr_t>(s) == kNoSocket) continue;
        // Connect with a 3 s timeout
#ifdef _WIN32
        u_long nb = 1;
        ioctlsocket(s, FIONBIO, &nb);
#else
        fcntl(s, F_SETFL, fcntl(s, F_GETFL) | O_NONBLOCK);
#endif
        ::connect(s, ai->ai_addr, static_cast<int>(ai->ai_addrlen));
        bool ok = waitSocket(s, true, 3000) > 0;
        if (ok) {
            int err = 0;
            socklen_t len = sizeof(err);
            getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &len);
            ok = err == 0;
        }
        if (ok) {
#ifdef _WIN32
            nb = 0;
            ioctlsocket(s, FIONBIO, &nb);
#else
            fcntl(s, F_SETFL, fcntl(s, F_GETFL) & ~O_NONBLOCK);
#endif
            int one = 1;
            setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));
            // A stalled network must not block the sender (and the E-stop byte behind it) for ever: a send that
            // cannot complete within 1 s fails, and the connection is reported lost
#ifdef _WIN32
            const DWORD send_timeout_ms = 1000;
            setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&send_timeout_ms), sizeof(send_timeout_ms));
#else
            timeval send_timeout{1, 0};
            setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
#endif
            break;
        }
        closeSocket(s);
        s = static_cast<SocketHandle>(kNoSocket);
    }
    freeaddrinfo(res);
    if (static_cast<intptr_t>(s) == kNoSocket) {
        if (error) *error = "Cannot connect to " + host + ":" + port;
        return false;
    }
    sock_.store(static_cast<intptr_t>(s));
    return true;
}

void TcpStream::close() {
    std::lock_guard<std::mutex> lock(write_mutex_);
    const intptr_t s = sock_.exchange(kNoSocket);
    if (s != kNoSocket) closeSocket(static_cast<SocketHandle>(s));
}

bool TcpStream::isOpen() const {
    return sock_.load() != kNoSocket;
}

long TcpStream::read(uint8_t* buf, size_t max, uint32_t timeout_ms) {
    const intptr_t sv = sock_.load();
    if (sv == kNoSocket) return -1;
    const SocketHandle s = static_cast<SocketHandle>(sv);
    const int r = waitSocket(s, false, timeout_ms);
    if (r < 0) return -1;
    if (r == 0) return 0;
    const int n = ::recv(s, reinterpret_cast<char*>(buf), static_cast<int>(max), 0);
    if (n <= 0) return -1; // 0 = closed by the board
    return n;
}

bool TcpStream::write(const uint8_t* data, size_t n) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    const intptr_t sv = sock_.load();
    if (sv == kNoSocket) return false;
    size_t sent = 0;
    while (sent < n) {
        const int w = ::send(static_cast<SocketHandle>(sv), reinterpret_cast<const char*>(data + sent),
                             static_cast<int>(n - sent), 0);
        if (w <= 0) return false;
        sent += static_cast<size_t>(w);
    }
    return true;
}

} // namespace rnr::transport
