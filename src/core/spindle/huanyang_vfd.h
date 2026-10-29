#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

namespace rnr {

// ============================================================================
// Huanyang VFD Configuration Parameters
// ============================================================================
struct HuanyangVfdConfig {
    bool        enabled            = false;      // True if RS485 Huanyang VFD is enabled
#ifdef _WIN32
    std::string port_name          = "COM3";     // Serial port: COM1..COM64 on Windows
#else
    std::string port_name          = "/dev/ttyUSB0"; // Serial port: /dev/ttyUSB0, /dev/cu.usbserial-..., ...
#endif
    uint32_t    baud_rate          = 9600;       // Baud rate (9600, 19200, 38400)
    uint8_t     slave_address      = 1;          // VFD Slave Address (PD163)
    uint16_t    max_rpm            = 24000;      // Max spindle RPM at max frequency
    float       max_freq_hz        = 400.0f;     // Max operating frequency in Hz (PD005)
    uint16_t    min_rpm            = 6000;       // Minimum operating RPM to protect spindle cooling
    bool        estop_on_comm_loss = true;       // Trigger CNC E-Stop if VFD communication fails
    bool        estop_on_stall     = true;       // Trigger CNC E-Stop if spindle commanded ON but RPM == 0

    // Speed monitoring (Mach3 vfdPlugin "Automatic EStop"): E-Stop when the spindle does not reach or
    // hold the commanded speed. The thresholds count consecutive RPM readings.
    bool        speed_monitor_enabled = false;   // <EStopOption>
    uint8_t     speed_margin_pct      = 3;       // <Margin>: tolerance band around the commanded RPM
    uint8_t     bad_speed_threshold   = 3;       // <BadSpeedThreshold>: out-of-band readings once at speed
    uint8_t     tracking_threshold    = 3;       // <BadtrackingThreshold>: readings without getting closer while ramping

    // Reading limits (Mach3 vfdPlugin <Status<n>_lim>): E-Stop when a reading exceeds its limit, 0 = off.
    // Checked whenever the VFD is polled, also while the spindle is stopped.
    float       limit_out_freq_hz  = 0.0f;
    float       limit_current_a    = 0.0f;
    float       limit_rpm          = 0.0f;
    float       limit_dc_volts     = 0.0f;
    float       limit_ac_volts     = 0.0f;
    float       limit_temp_c       = 0.0f;
};

// ============================================================================
// Inverter parameters read back over RS485 (function 0x01)
// ============================================================================
struct HuanyangVfdParams {
    bool        busy          = false;  // A read is queued or running
    bool        valid         = false;  // All parameters below were read successfully
    std::string error;                  // Why the last read failed
    int         pd001_run_source  = -1; // 0 = keypad, 1 = terminal, 2 = RS485
    int         pd002_freq_source = -1; // 0 = keypad knob, 1 = analog, 2 = RS485
    float       pd005_max_hz      = 0.0f;
    float       pd011_min_hz      = 0.0f;
    uint16_t    pd144_rpm_at_50hz = 0;  // Rated motor speed at 50 Hz

    // Spindle speeds at PD005 / PD011, as the Mach3 plugin computes them (PD144 * f / 50)
    uint16_t maxRpm() const noexcept;
    uint16_t minRpm() const noexcept;
    bool     rs485Controlled() const noexcept { return pd001_run_source == 2 && pd002_freq_source == 2; }
};

// ============================================================================
// Huanyang VFD Real-Time Telemetry Readout
// ============================================================================
struct HuanyangVfdTelemetry {
    bool        connected              = false;  // VFD answered within the last few seconds
    bool        port_open              = false;  // Serial port is open (says nothing about the VFD itself)
    bool        spindle_running        = false;
    bool        spindle_direction_rev  = false;
    uint16_t    target_rpm             = 0;
    float       target_freq_hz         = 0.0f;
    uint16_t    actual_rpm             = 0;
    float       actual_freq_hz         = 0.0f;
    float       actual_amps            = 0.0f;
    float       dc_bus_volts           = 0.0f;
    float       ac_out_volts           = 0.0f;
    float       temperature_c          = 0.0f;
    uint32_t    packets_sent           = 0;
    uint32_t    packets_received       = 0;
    uint32_t    comm_errors            = 0;
    std::string last_error_msg;
    bool        alarm_stall_tripped    = false;
    bool        alarm_comm_loss_tripped = false;
    bool        alarm_speed_tripped    = false; // Speed monitoring: bad speed or speed not tracking
    bool        alarm_limit_tripped    = false; // A reading exceeded its configured limit
    HuanyangVfdParams params{};

    bool anyAlarm() const noexcept {
        return alarm_stall_tripped || alarm_comm_loss_tripped || alarm_speed_tripped || alarm_limit_tripped;
    }
};

// ============================================================================
// Speed supervision as done by the Mach3 vfdPlugin (v2): while the speed changes it must get closer
// to the target on every reading; once inside the +/- margin band it has to stay there.
// ============================================================================
class VfdSpeedMonitor {
public:
    void reset() noexcept;
    // Feed one RPM reading. Returns a fault message when the spindle fails to reach or hold the target.
    std::string onRpmReading(bool running, uint16_t target_rpm, uint16_t actual_rpm, const HuanyangVfdConfig& cfg);

private:
    uint16_t target_      = 0;
    bool     ramping_     = false;
    int      last_dist_   = -1;
    unsigned no_progress_ = 0;
    unsigned bad_speed_   = 0;
};

// ============================================================================
// Huanyang VFD Controller Class (Win32 RS485 Serial Driver)
//
// Run/stop and frequency are kept as the latest requested state (not a queue): the worker sends a
// pending request, and keeps retrying it until the VFD acknowledges it. A stop request therefore can
// never be lost on a noisy RS485 line, and repeated requests can never pile up.
// ============================================================================
class HuanyangVfdController {
public:
    HuanyangVfdController();
    ~HuanyangVfdController();

    // Port enumeration (COMn on Windows, /dev/ttyUSB* / ttyACM* on Linux, /dev/cu.* on macOS)
    static std::vector<std::string> enumeratePorts();

    // Connection lifecycle
    bool start(const HuanyangVfdConfig& config);
    void stop();
    bool isRunning() const;   // Driver thread active (enabled)
    bool isConnected() const; // VFD is answering

    // Spindle control commands
    void setRun(bool run, bool reverse = false);
    void setRpm(uint16_t rpm);
    void emergencyStop();

    // Configuration updates
    void updateConfig(const HuanyangVfdConfig& config);
    HuanyangVfdConfig getConfig() const;

    // Telemetry
    HuanyangVfdTelemetry getTelemetry() const;
    void clearAlarms();

    // Read PD001, PD002, PD005, PD011 and PD144 from the inverter; the result arrives in telemetry.params
    void requestParameterRead();

    // Static CRC-16 utility (Modbus polynomial 0xA001)
    static uint16_t calculateCrc16(const uint8_t* buffer, size_t length);

    // Function 0x01 frames: [Slave] [0x01] [0x03] [PD] [0x00] [0x00] [CRC_Lo] [CRC_Hi], the reply echoes
    // the PD number followed by the 16-bit value.
    static void buildParameterRead(uint8_t slave, uint8_t pd, uint8_t out[8]);
    static bool decodeParameterReply(const uint8_t* rx, size_t len, uint8_t slave, uint8_t pd, uint16_t& value);

    // Fault message when a status reading (function 0x04 code) exceeds its configured limit, else empty
    static std::string checkReadingLimit(uint8_t status_code, float value, const HuanyangVfdConfig& cfg);
    // Scales a raw status reading (function 0x04 code) to its unit
    static float scaleStatusValue(uint8_t status_code, uint16_t raw);

private:
    void readParameters(uint8_t slave);
    void workerLoop();
    bool openSerialPort();
    void closeSerialPort();
    bool sendPacketAndReceive(const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t& rx_len, size_t expected_len);

    // Raw port I/O, one implementation per platform (Win32 / POSIX termios)
    bool openPlatformPort(const std::string& port_name, uint32_t baud, std::string& error);
    void closePlatformPort();
    bool platformPortOpen() const;
    void flushPlatformPort();
    bool writePlatformPort(const uint8_t* data, size_t len);
    size_t readPlatformPort(uint8_t* data, size_t max_len); // waits briefly; 0 = nothing arrived

    mutable std::mutex       mutex_;
    HuanyangVfdConfig        config_;
    HuanyangVfdTelemetry     telemetry_;

    std::atomic<bool>        running_{false};
    std::atomic<bool>        reopen_requested_{false};
    std::thread              worker_thread_;

    // Latest requested state (guarded by mutex_)
    bool                     run_dirty_       = false;
    uint8_t                  run_control_byte_ = 0x08; // 0x08 = Stop, 0x01 = Fwd, 0x11 = Rev
    bool                     freq_dirty_      = false;
    uint16_t                 freq_raw_        = 0;

    // Owned by the worker thread only
#ifdef _WIN32
    void*                    serial_handle_ = nullptr; // HANDLE (void* to avoid Windows.h in header)
#else
    int                      serial_fd_ = -1;
#endif
    std::chrono::steady_clock::time_point last_tx_time_;
    std::chrono::steady_clock::time_point last_successful_rx_time_;

    std::chrono::steady_clock::time_point run_command_time_;
    bool                     commanded_run_{false};
    bool                     params_requested_ = false; // guarded by mutex_
    VfdSpeedMonitor          speed_monitor_;            // guarded by mutex_
};

} // namespace rnr
