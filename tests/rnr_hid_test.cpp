#include "core/hid/rnr_usb_device.h"
#include <cstdio>
#include <chrono>
#include <thread>
#include <string>

using namespace rnr;

static void printBinaryInputs(uint16_t mask) {
    for (int i = 1; i <= 12; ++i) {
        bool on = ((mask >> (i - 1)) & 1U) != 0;
        std::printf("IN%d:%d ", i, on ? 1 : 0);
    }
}

int main(int argc, char** argv) {
    std::printf("================================================================\n");
    std::printf("  BreakoutCNC - Live Hardware & Protocol Diagnostic Tool\n");
    std::printf("  Target: RNR ECO MOTION 2.0 (VID=0x%04X, PID=0x%04X)\n",
                protocol::kVendorId, protocol::kProductId);
    std::printf("================================================================\n\n");

    auto devices = hid::RnrUsbDevice::enumerateDevices();
    if (devices.empty()) {
        std::printf("[WARN] No RNR ECO MOTION 2.0 USB board detected.\n");
        std::printf("       Running offline protocol self-test only.\n");
        auto pkt = protocol::buildShortControlPacket(protocol::Opcode::SetOutputsIdle, 42, false, 0x01);
        if (pkt[0] == 0x02 && pkt[1] == 0x03 && pkt[3] == 42 && pkt[5] == 0x01) {
            std::printf("[PASS] 27-byte Output Report serialization verified.\n");
            return 0;
        }
        return 1;
    }

    std::printf("[OK] Found %zu RNR Eco Motion device(s):\n", devices.size());
    for (size_t i = 0; i < devices.size(); ++i) {
        const auto& d = devices[i];
        std::printf("  [%zu] Product : %s\n", i, d.product_name.c_str());
        std::printf("      Serial  : %s\n", d.serial_number.c_str());
        std::printf("      VID/PID : 0x%04X / 0x%04X (Rev 0x%04X)\n", d.vendor_id, d.product_id, d.version);
        std::printf("      Reports : Input=%u bytes, Output=%u bytes\n", d.input_report_len, d.output_report_len);
        std::printf("      Path    : %s\n\n", d.device_path.c_str());
    }

    hid::RnrUsbDevice dev;
    if (!dev.open(devices.front().device_path)) {
        std::printf("[ERROR] Failed to open USB HID device handle!\n");
        return 1;
    }

    std::printf("[OK] Device opened! Performing handshake (Opcode 0x0F Configure + 0x07/0x02 Reset Clear)...\n");
    uint16_t seq = 0;
    dev.sendConfigureBoard(100, 0x00, 0x00, 0x00, &seq);
    dev.sendShortCommand(protocol::Opcode::PreResetClear, false, 0, 0, 0, 0, &seq);
    dev.sendShortCommand(protocol::Opcode::ClearEStopEnable, false, 0, 0, 0, 0, &seq);
    std::printf("[OK] Sent handshake packets up to seq=%u.\n\n", seq);

    int duration_ms = 1500;
    bool toggle_out1 = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--pulse-out1") {
            toggle_out1 = true;
        } else if (arg == "--short") {
            duration_ms = 600;
        }
    }

    if (toggle_out1) {
        std::printf("[TEST] Pulsing OUT1 (Torch/Spindle relay) ON for 300ms then OFF...\n");
        dev.sendShortCommand(protocol::Opcode::SetOutputsIdle, false, protocol::OutputBit::OUT1_TorchOrSpindle);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        dev.sendShortCommand(protocol::Opcode::SetOutputsIdle, false, 0x00);
    }

    std::printf("[STREAM] Reading live 49-byte status reports for %d ms...\n", duration_ms);
    auto t_start = std::chrono::steady_clock::now();
    size_t report_count = 0;
    protocol::BoardStatusReport last_report{};

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - t_start).count() < duration_ms)
    {
        protocol::BoardStatusReport rep{};
        if (dev.readStatusReport(rep, 20)) {
            ++report_count;
            last_report = rep;
            if (report_count == 1 || report_count % 25 == 0) {
                std::printf("  [#%03zu] State=%u Fault=0x%02X AckSeq=%u Tick=%u | Steps X=%d Y=%d Z=%d A=%d | ",
                            report_count,
                            rep.motion_state,
                            rep.fault_flags,
                            rep.ack_seq,
                            rep.tick_counter,
                            rep.live_steps[0],
                            rep.live_steps[1],
                            rep.live_steps[2],
                            rep.live_steps[3]);
                printBinaryInputs(rep.inputs_bitmask);
                std::printf("\n");
            }
        }
    }

    double elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    double rate_hz = (elapsed_sec > 0.0) ? (static_cast<double>(report_count) / elapsed_sec) : 0.0;

    std::printf("\n================================================================\n");
    std::printf("  Telemetry Summary:\n");
    std::printf("  Total Reports Read : %zu in %.2f sec (%.1f Hz)\n", report_count, elapsed_sec, rate_hz);
    std::printf("  Last Ack Sequence  : %u (Sent: %u)\n", last_report.ack_seq, dev.lastSentSequence());
    std::printf("  Hardware E-Stop    : %s (fault_flags=0x%02X)\n",
                last_report.isInHardwareEStop() ? "TRIGGERED" : "CLEAR", last_report.fault_flags);
    std::printf("  Hardware Limit     : %s\n",
                last_report.isLimitTriggered() ? "TRIGGERED" : "CLEAR");
    std::printf("================================================================\n");

    dev.close();
    return (report_count > 0) ? 0 : 2;
}
