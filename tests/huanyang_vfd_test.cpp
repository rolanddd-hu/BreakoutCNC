#include "core/spindle/huanyang_vfd.h"
#include <iostream>
// Release builds define NDEBUG: keep the asserts active, they are the test
#undef NDEBUG
#include <cassert>
#include <iomanip>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace rnr;

// In-memory RS-485 link: every request is answered with its own bytes (a valid frame with the same CRC), and every
// request is recorded so a test can see what the driver sent
struct FakeVfdLink {
    std::mutex m;
    std::vector<std::vector<uint8_t>> frames;
    std::deque<uint8_t> rx;
};

class FakeVfdStream : public transport::IByteStream {
public:
    explicit FakeVfdStream(std::shared_ptr<FakeVfdLink> link) : link_(std::move(link)) {}
    bool open(const transport::ConnectionTarget&, const transport::StreamOptions&, std::string*) override { open_ = true; return true; }
    void close() override { open_ = false; }
    bool isOpen() const override { return open_; }
    long read(uint8_t* buf, size_t max, uint32_t timeout_ms) override {
        std::lock_guard<std::mutex> lock(link_->m);
        if (link_->rx.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(std::min<uint32_t>(timeout_ms, 2)));
            return 0;
        }
        size_t n = 0;
        while (n < max && !link_->rx.empty()) { buf[n++] = link_->rx.front(); link_->rx.pop_front(); }
        return static_cast<long>(n);
    }
    bool write(const uint8_t* data, size_t n) override {
        std::lock_guard<std::mutex> lock(link_->m);
        link_->frames.emplace_back(data, data + n);
        link_->rx.insert(link_->rx.end(), data, data + n);
        return true;
    }
    void flushInput() override { std::lock_guard<std::mutex> lock(link_->m); link_->rx.clear(); }
    bool accessesRealHardware() const override { return false; }
private:
    std::shared_ptr<FakeVfdLink> link_;
    bool open_ = false;
};

// Last run / stop control byte written (function 0x03), or -1
int lastRunControl(FakeVfdLink& link) {
    std::lock_guard<std::mutex> lock(link.m);
    for (auto it = link.frames.rbegin(); it != link.frames.rend(); ++it) {
        if (it->size() >= 4 && (*it)[1] == 0x03 && (*it)[2] == 0x01) return (*it)[3];
    }
    return -1;
}

// A2: switching the VFD off (or closing the app) while the spindle runs sends a stop command first
void testStopSendsStopFrame() {
    std::cout << "[TEST] Driver stop while the spindle runs sends a stop frame..." << std::endl;
    auto link = std::make_shared<FakeVfdLink>();
    HuanyangVfdController vfd;
    vfd.setStreamFactory([link] { return std::make_unique<FakeVfdStream>(link); });
    HuanyangVfdConfig cfg;
    cfg.enabled = true;
    cfg.port_name = "FAKE";
    cfg.max_rpm = 24000;
    cfg.min_rpm = 6000;
    assert(vfd.start(cfg));
    vfd.setRpm(12000);
    vfd.setRun(true, false);
    for (int i = 0; i < 200 && lastRunControl(*link) != 0x01; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    assert(lastRunControl(*link) == 0x01);
    vfd.stop();
    assert(!vfd.isRunning());
    assert(lastRunControl(*link) == 0x08);
    std::cout << "  -> Stop frame sent before the driver ended." << std::endl;
}

void testCrcCalculation() {
    std::cout << "[TEST] Testing Modbus CRC-16 Calculation..." << std::endl;

    // Test vector 1: Run Forward command: 01 03 01 01
    // Expected CRC: 0x8831 -> Little-endian: 0x31 0x88
    uint8_t run_fwd[4] = { 0x01, 0x03, 0x01, 0x01 };
    uint16_t crc1 = HuanyangVfdController::calculateCrc16(run_fwd, 4);
    assert((crc1 & 0xFF) == 0x31);
    assert((crc1 >> 8) == 0x88);

    // Test vector 2: Stop command: 01 03 01 08
    // Expected CRC: 0x8EF1 -> Little-endian: 0xF1 0x8E
    uint8_t stop_cmd[4] = { 0x01, 0x03, 0x01, 0x08 };
    uint16_t crc2 = HuanyangVfdController::calculateCrc16(stop_cmd, 4);
    assert((crc2 & 0xFF) == 0xF1);
    assert((crc2 >> 8) == 0x8E);

    // Test vector 3: Set Frequency 400.00 Hz (40000 = 0x9C40): 01 05 02 9C 40
    // Expected CRC: 0x3CD0 -> Little-endian: 0xD0 0x3C
    uint8_t freq_cmd[5] = { 0x01, 0x05, 0x02, 0x9C, 0x40 };
    uint16_t crc3 = HuanyangVfdController::calculateCrc16(freq_cmd, 5);
    assert((crc3 & 0xFF) == 0xD0);
    assert((crc3 >> 8) == 0x3C);

    std::cout << "  -> CRC-16 test passed successfully!" << std::endl;
}

void testPortEnumeration() {
    std::cout << "[TEST] Testing Serial Port Enumeration..." << std::endl;
    auto ports = HuanyangVfdController::enumeratePorts();
    std::cout << "  Found " << ports.size() << " available COM port(s):" << std::endl;
    for (const auto& p : ports) {
        std::cout << "    - " << p << std::endl;
    }
    std::cout << "  -> Port enumeration test completed." << std::endl;
}

void testControllerConfiguration() {
    std::cout << "[TEST] Testing HuanyangVfdController Configuration & Lifecycle..." << std::endl;
    HuanyangVfdController vfd;

    HuanyangVfdConfig cfg;
    cfg.enabled = true;
    cfg.port_name = "COM99"; // Non-existent port for offline testing
    cfg.baud_rate = 9600;
    cfg.slave_address = 1;
    cfg.max_rpm = 24000;
    cfg.max_freq_hz = 400.0f;
    cfg.min_rpm = 6000;

    vfd.start(cfg);
    assert(vfd.getConfig().max_rpm == 24000);

    // Set RPM and Run
    vfd.setRpm(18000);
    vfd.setRun(true);

    auto telem = vfd.getTelemetry();
    assert(telem.target_rpm == 18000);
    assert(std::abs(telem.target_freq_hz - 300.0f) < 0.1f);

    // Stop and Disconnect
    vfd.emergencyStop();
    vfd.stop();
    assert(!vfd.isConnected());

    std::cout << "  -> Controller Lifecycle test passed successfully!" << std::endl;
}

void testSpeedMonitor() {
    std::cout << "[TEST] Testing speed monitoring (Mach3 vfdPlugin v2 rules)..." << std::endl;
    HuanyangVfdConfig cfg;
    cfg.speed_monitor_enabled = true;
    cfg.speed_margin_pct = 3;
    cfg.bad_speed_threshold = 3;
    cfg.tracking_threshold = 3;

    // Normal spin-up to 12000 RPM, then holding speed: no fault
    VfdSpeedMonitor m;
    for (uint16_t rpm : {0, 2000, 5000, 9000, 11800, 12100, 11900, 12000}) {
        assert(m.onRpmReading(true, 12000, rpm, cfg).empty());
    }
    // Speed drops out of the +/-3% band: trips on the 4th consecutive bad reading
    assert(m.onRpmReading(true, 12000, 11000, cfg).empty());
    assert(m.onRpmReading(true, 12000, 11000, cfg).empty());
    assert(m.onRpmReading(true, 12000, 11000, cfg).empty());
    assert(!m.onRpmReading(true, 12000, 11000, cfg).empty());

    // A single bad reading is forgiven
    m.reset();
    assert(m.onRpmReading(true, 12000, 12000, cfg).empty());
    assert(m.onRpmReading(true, 12000, 10000, cfg).empty());
    assert(m.onRpmReading(true, 12000, 12000, cfg).empty());
    for (int i = 0; i < 3; ++i) assert(m.onRpmReading(true, 12000, 10000, cfg).empty());

    // Spin-up that stops making progress: tracking fault
    m.reset();
    assert(m.onRpmReading(true, 18000, 0, cfg).empty());
    assert(m.onRpmReading(true, 18000, 4000, cfg).empty());
    assert(m.onRpmReading(true, 18000, 4000, cfg).empty());
    assert(m.onRpmReading(true, 18000, 4000, cfg).empty());
    assert(m.onRpmReading(true, 18000, 3900, cfg).empty());
    assert(!m.onRpmReading(true, 18000, 3900, cfg).empty());

    // Off, stopped or disabled: never trips
    m.reset();
    for (int i = 0; i < 10; ++i) assert(m.onRpmReading(false, 12000, 0, cfg).empty());
    cfg.speed_monitor_enabled = false;
    for (int i = 0; i < 10; ++i) assert(m.onRpmReading(true, 12000, 0, cfg).empty());
    std::cout << "  -> Speed monitor test passed successfully!" << std::endl;
}

void testReadingLimitsAndParameters() {
    std::cout << "[TEST] Testing reading limits, status scaling and parameter frames..." << std::endl;
    HuanyangVfdConfig cfg;
    cfg.limit_current_a = 9.3f;
    assert(HuanyangVfdController::checkReadingLimit(0x02, 9.2f, cfg).empty());
    assert(!HuanyangVfdController::checkReadingLimit(0x02, 9.4f, cfg).empty());
    assert(HuanyangVfdController::checkReadingLimit(0x07, 200.0f, cfg).empty()); // No temperature limit set

    // Decimal places per status code as in the plugin: Hz /100, A, V and C /10, RPM as is
    assert(std::abs(HuanyangVfdController::scaleStatusValue(0x01, 40000) - 400.0f) < 1e-3f);
    assert(std::abs(HuanyangVfdController::scaleStatusValue(0x02, 93) - 9.3f) < 1e-3f);
    assert(HuanyangVfdController::scaleStatusValue(0x03, 24000) == 24000.0f);
    assert(std::abs(HuanyangVfdController::scaleStatusValue(0x04, 3110) - 311.0f) < 1e-3f);

    // Function 0x01 read of PD005: 01 01 03 05 00 00 CRC; reply 01 01 03 05 9C 40 CRC (400.00 Hz)
    uint8_t tx[8];
    HuanyangVfdController::buildParameterRead(1, 5, tx);
    assert(tx[0] == 1 && tx[1] == 0x01 && tx[2] == 0x03 && tx[3] == 5 && tx[4] == 0 && tx[5] == 0);
    const uint16_t tx_crc = HuanyangVfdController::calculateCrc16(tx, 6);
    assert(tx[6] == (tx_crc & 0xFF) && tx[7] == (tx_crc >> 8));

    uint8_t rx[8] = {0x01, 0x01, 0x03, 0x05, 0x9C, 0x40, 0, 0};
    const uint16_t rx_crc = HuanyangVfdController::calculateCrc16(rx, 6);
    rx[6] = static_cast<uint8_t>(rx_crc & 0xFF);
    rx[7] = static_cast<uint8_t>(rx_crc >> 8);
    uint16_t value = 0;
    assert(HuanyangVfdController::decodeParameterReply(rx, 8, 1, 5, value) && value == 40000);
    assert(!HuanyangVfdController::decodeParameterReply(rx, 8, 1, 11, value)); // Different PD echoed
    rx[5] ^= 0xFF;
    assert(!HuanyangVfdController::decodeParameterReply(rx, 8, 1, 5, value));  // CRC mismatch

    // Speed range as the plugin computes it: PD144 * f / 50
    HuanyangVfdParams p;
    p.pd005_max_hz = 400.0f;
    p.pd011_min_hz = 100.0f;
    p.pd144_rpm_at_50hz = 3000;
    assert(p.maxRpm() == 24000 && p.minRpm() == 6000);
    std::cout << "  -> Limits & parameter frame test passed successfully!" << std::endl;
}

int main() {
    std::cout << "=================================================" << std::endl;
    std::cout << "Huanyang VFD RS485 Protocol & Controller Tests" << std::endl;
    std::cout << "=================================================" << std::endl;

    testCrcCalculation();
    testPortEnumeration();
    testControllerConfiguration();
    testSpeedMonitor();
    testReadingLimitsAndParameters();
    testStopSendsStopFrame();

    std::cout << "\nALL HUANYANG VFD TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
