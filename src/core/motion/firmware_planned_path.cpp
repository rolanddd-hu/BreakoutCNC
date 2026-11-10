#include "core/motion/firmware_planned_path.h"
#include "core/board/board_registry.h"
#include "core/board/grbl/grbl_codes.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rnr::core::motion {

using board::grbl::FirmwareMessage;
using board::grbl::MessageKind;
using board::grbl::Realtime;
using LogDir = transport::ProtocolLogEntry::Dir;

namespace {

constexpr auto kSilenceTimeout   = std::chrono::milliseconds(2500);
constexpr auto kHomingTimeout    = std::chrono::seconds(180);
constexpr auto kIdentifyTimeout  = std::chrono::milliseconds(2500);
constexpr auto kSettingsTimeout  = std::chrono::seconds(6);
constexpr auto kHoldTimeout      = std::chrono::seconds(10);
constexpr auto kResetTimeout     = std::chrono::seconds(3);
constexpr double kJogLeadTime_s  = 0.10; // Starting values, to be tuned on the bench (GRBL_FAMILY_PLAN 9.2)
constexpr double kJogChunkTime_s = 0.05;
const char kAxis[4] = {'X', 'Y', 'Z', 'A'};

bool isMotion(SegmentType t) {
    return t == SegmentType::RapidMove || t == SegmentType::CutLinear || t == SegmentType::CutArcCW ||
           t == SegmentType::CutArcCCW;
}

std::string fmt(double v) {
    return GcodeEmitter::num(v, 3);
}

} // namespace

FirmwarePlannedPath::FirmwarePlannedPath(std::string board_id, board::BoardProfile profile)
    : board_id_(std::move(board_id)),
      profile_(std::move(profile)),
      log_(std::make_shared<transport::ProtocolLog>()),
      protocol_(log_),
      selected_flavour_(board::grbl::flavourForBoardId(board_id_)) {
    board::completeProfile(board_id_, profile_);
    fw_ = board::grbl::defaultProfile(selected_flavour_);
    refreshDialect();
}

FirmwarePlannedPath::~FirmwarePlannedPath() {
    close();
}

std::string FirmwarePlannedPath::connectionLabel() const {
    const std::string t = profile_.get("transport", "serial");
    const std::string a = profile_.get("address");
    return (t == "tcp") ? ("network " + a) : a;
}

board::BoardCapabilities FirmwarePlannedPath::capabilities() const {
    const auto f = fw_.detected && fw_.flavour != board::grbl::Flavour::GenericGrbl ? fw_.flavour : selected_flavour_;
    auto caps = board::grbl::firmwareCapabilities(f, &fw_);
    caps.accesses_real_hardware = !isOpen() || protocol_.accessesRealHardware();
    return caps;
}

void FirmwarePlannedPath::refreshDialect() {
    GcodeDialect d;
    d.max_line_chars = fw_.max_line_chars;
    d.axis_count = std::clamp<int>(fw_.axis_count, 3, 4);
    d.line_numbers = fw_.reports_line_numbers;
    d.mist = fw_.has_mist;
    d.aux_outputs = (fw_.flavour == board::grbl::Flavour::Grbl11) ? 0 : fw_.aux_outputs;
    emitter_.setDialect(d);
    ++caps_revision_;
}

// ============================================================================
// Connection
// ============================================================================
bool FirmwarePlannedPath::open(bool hardware_access_allowed, std::string* error) {
    close();
    hardware_access_allowed_ = hardware_access_allowed;
    const std::string transport_name = profile_.get("transport", "serial");
    const std::string address = profile_.get("address");
    if (address.empty()) {
        if (error) *error = (transport_name == "tcp") ? "Enter the board's network address first"
                                                      : "Choose the board's serial port first";
        return false;
    }
    auto stream = transport::createByteStream(transport_name);
    if (!stream) {
        if (error) *error = "Unknown connection type '" + transport_name + "'";
        return false;
    }
    if (stream->accessesRealHardware() && !hardware_access_allowed) {
        if (error) *error = "Hardware access is disabled";
        return false;
    }
    transport::StreamOptions opts;
    opts.baud = static_cast<uint32_t>(profile_.getInt("baud", 115200));
    opts.dtr = profile_.getBool("dtr_on_connect", true);
    opts.rts = profile_.getBool("rts_on_connect", false);
    opts.read_timeout_ms = 50;
    opts.default_tcp_port = static_cast<uint16_t>(profile_.getInt("tcp_port", 23));
    transport::ConnectionTarget target{transport_name, address, connectionLabel()};

    fw_ = board::grbl::defaultProfile(selected_flavour_);
    refreshDialect();
    protocol_.setRxBufferBytes(fw_.rx_buffer_bytes);
    if (!protocol_.open(std::move(stream), target, opts, error)) return false;

    const auto now = Clock::now();
    phase_ = Phase::WaitWelcome;
    phase_deadline_ = now + std::chrono::milliseconds(std::clamp<long>(profile_.getInt("connect_wait_ms", 800), 100, 10000));
    welcome_seen_ = false;
    expect_reset_ = false;
    fb_ = PathFeedback{};
    fb_.state = PathState::Connecting;
    fb_.status_text = "Connecting to " + connectionLabel() + "...";
    fw_state_.clear();
    op_ = Op::None;
    script_.clear();
    script_waiting_ = false;
    events_.clear();
    numbered_.clear();
    enums_.clear();
    enum_groups_.clear();
    settings_ = board::grbl::SettingsSnapshot{};
    params_.clear();
    frame_f_ = {};
    wco_ = {};
    next_poll_ = now + std::chrono::milliseconds(300);
    estop_requested_ = false;
    return true;
}

void FirmwarePlannedPath::close() {
    ready_flag_ = false;
    deferred_ = nullptr;
    protocol_.close();
    phase_ = Phase::Closed;
    op_ = Op::None;
    script_.clear();
    script_waiting_ = false;
    job_phase_ = JobPhase::Done;
    job_lines_.clear();
    fb_.state = PathState::Offline;
    fb_.job_active = false;
    if (report_running_) {
        report_running_ = false;
        report_lines_.push_back("# The report was interrupted: the connection closed");
        ++report_revision_;
    }
}

bool FirmwarePlannedPath::pollEvent(PathEvent& out) {
    if (events_.empty()) return false;
    out = std::move(events_.front());
    events_.pop_front();
    return true;
}

void FirmwarePlannedPath::fault(const std::string& text, int code, bool position_lost) {
    PathEvent e;
    e.kind = PathEvent::Kind::Fault;
    e.ok = false;
    e.text = text;
    e.code = code;
    e.position_lost = position_lost;
    emit(e);
    fb_.status_text = text;
    if (position_lost) fb_.homed = {};
}

void FirmwarePlannedPath::sendHandshakeLine(const std::string& line) {
    protocol_.sendLine(line);
}

void FirmwarePlannedPath::beginIdentify(Clock::time_point now) {
    ready_flag_ = false;
    phase_ = Phase::Identify;
    phase_deadline_ = now + kIdentifyTimeout;
    protocol_.forgetUnacknowledged();
    sendHandshakeLine("$I");
}

void FirmwarePlannedPath::tick(Clock::time_point now) {
    if (estop_requested_.exchange(false)) {
        // The soft reset was written by emergencyStop(); the board answers with its welcome line
        expect_reset_ = true;
        if (phase_ != Phase::Ready && phase_ != Phase::Closed) {
            // Reset during the handshake: identify the board again once it has restarted
            expect_reset_ = false;
            phase_ = Phase::WaitWelcome;
            welcome_seen_ = false;
            phase_deadline_ = now + std::chrono::milliseconds(std::clamp<long>(profile_.getInt("connect_wait_ms", 800), 100, 10000));
        }
        protocol_.forgetUnacknowledged();
        script_.clear();
        script_waiting_ = false;
        job_lines_.clear();
        job_phase_ = JobPhase::Done;
        fb_.job_active = false;
        jog_vel_ = {};
        outputs_pending_ = false;
        deferred_ = nullptr;
        op_ = Op::None;
    }
    if (!isOpen()) return;
    if (protocol_.linkFailed()) {
        const bool was_ready = phase_ == Phase::Ready;
        close();
        if (was_ready) {
            fault("Connection to the board lost (cable unplugged or board switched off)", 0, true);
        } else {
            PathEvent e;
            e.kind = PathEvent::Kind::ConnectFailed;
            e.ok = false;
            e.text = "Connection to " + connectionLabel() + " failed";
            emit(e);
        }
        fb_.status_text = "Connection lost";
        return;
    }

    std::vector<FirmwareMessage> msgs;
    protocol_.poll(msgs);
    for (const auto& m : msgs) {
        handleMessage(m, now);
        if (!isOpen()) return;
    }

    switch (phase_) {
        case Phase::WaitWelcome:
            if (welcome_seen_ || now >= phase_deadline_) beginIdentify(now);
            break;
        case Phase::Identify:
        case Phase::ReadSettings:
        case Phase::ReadEnums:
        case Phase::ReadCodes:
        case Phase::ReadParams:
            if (now >= phase_deadline_) {
                if (phase_ == Phase::Identify && !fw_.detected && !welcome_seen_) {
                    PathEvent e;
                    e.kind = PathEvent::Kind::ConnectFailed;
                    e.ok = false;
                    e.text = "No GRBL-compatible board answered on " + connectionLabel() +
                             " (check the port, the baud rate and that no other program uses it)";
                    close();
                    fb_.status_text = e.text;
                    emit(e);
                    return;
                }
                // A board that does not answer one of the optional requests: carry on with what we have
                FirmwareMessage fake;
                fake.kind = MessageKind::Ack;
                onAck(fake, now);
            }
            break;
        case Phase::Ready: {
            const auto silence = now - protocol_.lastReceive();
            const bool quiet = (op_ == Op::Homing) || (op_ == Op::Script && script_waiting_ && script_current_.quiet);
            if (quiet ? (silence > kHomingTimeout) : (silence > kSilenceTimeout)) {
                fault("The board stopped responding", 0, true);
                close();
                fb_.status_text = "The board stopped responding";
                return;
            }
            break;
        }
        default:
            break;
    }

    // Status polling: 100 ms while anything moves, otherwise the profile interval
    if (phase_ != Phase::WaitWelcome && now >= next_poll_) {
        protocol_.realtime(Realtime::StatusReport, fw_.top_bit_realtime);
        const bool moving = fb_.state == PathState::Moving || fb_.state == PathState::Homing ||
                            fb_.state == PathState::Holding || op_ != Op::None;
        const long interval = moving ? 100 : std::clamp<long>(profile_.getInt("status_interval_ms", 200), 50, 1000);
        next_poll_ = now + std::chrono::milliseconds(interval);
    }

    if (phase_ != Phase::Ready) return;

    switch (op_) {
        case Op::Script:
        case Op::Moves:
        case Op::Probe:
        case Op::Homing:
            pumpScript(now);
            break;
        case Op::Job:
            pumpJob(now);
            break;
        case Op::JogStep:
            if (jog_step_acked_ && status_since_ack_ && fw_state_ == "Idle" &&
                now - script_line_sent_at_ > std::chrono::milliseconds(150)) {
                finishOp(true);
            }
            break;
        case Op::Stopping: {
            if (stop_phase_ == StopPhase::HoldWait) {
                const bool stopped = (fw_state_ == "Hold" && fw_substate_ == 0) || fw_state_ == "Idle" ||
                                     fw_state_ == "Door";
                if (stopped || now >= stop_deadline_) {
                    if (!stopped) {
                        log_->add(LogDir::Info, "Feed hold did not complete in time: resetting anyway");
                    }
                    protocol_.realtime(Realtime::SoftReset);
                    last_reset_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
                    expect_reset_ = true;
                    stop_phase_ = StopPhase::ResetWait;
                    stop_deadline_ = now + kResetTimeout;
                    if (!stopped) fb_.homed = {};
                }
            } else if (stop_phase_ == StopPhase::ResetWait) {
                if (!expect_reset_ || now >= stop_deadline_) {
                    expect_reset_ = false;
                    protocol_.forgetUnacknowledged();
                    job_lines_.clear();
                    job_phase_ = JobPhase::Done;
                    fb_.job_active = false;
                    emitter_.reset();
                    const Op prev = stop_prev_op_;
                    op_ = Op::None;
                    PathEvent e;
                    e.ok = false;
                    if (prev == Op::Job) {
                        e.kind = PathEvent::Kind::JobFinished;
                        e.segment_index = fb_.active_segment;
                        emit(e);
                    } else if (prev == Op::Probe) {
                        e.kind = PathEvent::Kind::ProbeDone;
                        emit(e);
                    } else if (prev == Op::Homing) {
                        e.kind = PathEvent::Kind::HomingDone;
                        emit(e);
                    } else if (prev == Op::JogContinuous) {
                        e.kind = PathEvent::Kind::JogStopped;
                        emit(e);
                    } else {
                        e.kind = PathEvent::Kind::MoveDone;
                        emit(e);
                    }
                }
            }
            break;
        }
        default:
            break;
    }

    if (op_ == Op::None && deferred_) {
        auto start = std::move(deferred_);
        deferred_ = nullptr;
        start();
    }
    pumpJog(now);
    pumpOverrides();

    if (op_ == Op::None && outputs_pending_ && fw_state_ == "Idle") {
        outputs_pending_ = false;
        setOutputs(outputs_req_, nullptr);
    }
}

void FirmwarePlannedPath::handleMessage(const FirmwareMessage& m, Clock::time_point now) {
    if (report_running_ && script_waiting_ && script_current_.report && m.kind != MessageKind::Status &&
        m.kind != MessageKind::Empty) {
        report_lines_.push_back(m.raw);
    }
    if (phase_ == Phase::ReadCodes && (m.kind == MessageKind::Feedback || m.kind == MessageKind::Text)) {
        int code = 0;
        std::string text;
        if (board::grbl::parseCodeListLine(m.raw, code, text)) {
            (codes_target_ == 'A' ? board_alarm_texts_ : board_error_texts_)[code] = text;
        }
    }
    switch (m.kind) {
        case MessageKind::Status:
            onStatus(m.status, now);
            break;
        case MessageKind::Ack:
            onAck(m, now);
            break;
        case MessageKind::Nack:
            onNack(m, now);
            break;
        case MessageKind::Alarm:
            onAlarm(m.code);
            break;
        case MessageKind::Welcome:
            onWelcome(m.text, now);
            break;
        case MessageKind::BuildInfo:
            board::grbl::applyBuildInfo(fw_, m.name, m.text);
            protocol_.setRxBufferBytes(fw_.rx_buffer_bytes);
            refreshDialect();
            break;
        case MessageKind::Setting: {
            const int n = std::atoi(m.name.c_str());
            numbered_[n] = m.text;
            const bool thc_before = fw_.board_thc;
            board::grbl::applyNumberedSetting(fw_, n, std::atof(m.text.c_str()));
            if (fw_.board_thc != thc_before) ++caps_revision_; // Plasma mode ($350) switched THC on or off
            break;
        }
        case MessageKind::Parameter: {
            std::array<double, 4> v = m.position;
            const double unit = fw_.reports_inches ? 25.4 : 1.0;
            for (size_t a = 0; a < 3; ++a) v[a] *= unit; // A is in degrees
            params_[m.name] = v;
            break;
        }
        case MessageKind::Probe: {
            const double unit = fw_.reports_inches ? 25.4 : 1.0;
            probe_result_seen_ = true;
            probe_hit_ = m.flag;
            for (size_t a = 0; a < 4; ++a) {
                probe_pos_[a] = (static_cast<int>(a) < m.value_count) ? m.position[a] * (a < 3 ? unit : 1.0) : fw_mpos_[a];
            }
            break;
        }
        case MessageKind::Enumeration:
            if (m.name == "SETTING") {
                board::grbl::EnumeratedSetting e;
                if (board::grbl::parseEnumeratedSetting(m.text, e)) enums_[e.id] = e;
            } else if (m.name == "SETTINGGROUP") {
                std::string id, name;
                if (board::grbl::parseSettingGroup(m.text, id, name)) enum_groups_[id] = name;
            } else if (m.name == "ALARMCODE" || m.name == "ERRORCODE") {
                int code = 0;
                std::string text;
                if (board::grbl::parseEnumeratedCode(m.text, code, text)) {
                    (m.name == "ALARMCODE" ? board_alarm_texts_ : board_error_texts_)[code] = text;
                }
            }
            break;
        case MessageKind::Text:
            if (script_waiting_ && script_current_.collect) collected_.push_back(m.raw);
            break;
        case MessageKind::Feedback:
            if (script_waiting_ && script_current_.collect) collected_.push_back(m.raw);
            if (m.text.find("arc failed") != std::string::npos || m.text.find("Arc failed") != std::string::npos) {
                ++fb_.arc_failed_count; // grblHAL plasma plugin: no Arc OK after its retries; it holds the job
                log_->add(LogDir::Error, "The torch did not start: no Arc OK (" + m.text + ")");
            }
            break;
        default:
            break;
    }
}

void FirmwarePlannedPath::onWelcome(const std::string& text, Clock::time_point now) {
    fw_.welcome = text;
    const auto d = board::grbl::detectFromWelcome(text);
    if (d != board::grbl::Flavour::GenericGrbl) fw_.flavour = d;
    fw_.detected = true;
    protocol_.forgetUnacknowledged();
    if (phase_ == Phase::WaitWelcome) {
        welcome_seen_ = true;
        return;
    }
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    if (expect_reset_ || now_ns - last_reset_ns_.load() < 3'000'000'000LL) {
        expect_reset_ = false; // Our own reset (stop sequence or E-stop)
        return;
    }
    // Restarted on its own: power, reset button, brown-out
    log_->add(LogDir::Error, "The board restarted");
    if (op_ == Op::Job) {
        PathEvent e;
        e.kind = PathEvent::Kind::JobFinished;
        e.ok = false;
        emit(e);
    }
    op_ = Op::None;
    script_.clear();
    script_waiting_ = false;
    job_lines_.clear();
    job_phase_ = JobPhase::Done;
    fb_.job_active = false;
    fault("The board restarted (power, reset button or brown-out): position lost", 0, true);
    if (phase_ == Phase::Ready) beginIdentify(now);
}

void FirmwarePlannedPath::onStatus(const board::grbl::StatusReport& s, Clock::time_point now) {
    last_status_ = now;
    status_since_ack_ = true;
    const double unit = fw_.reports_inches ? 25.4 : 1.0;
    auto scale = [&](size_t a) { return a < 3 ? unit : 1.0; }; // A is in degrees
    if (s.has_wco) {
        for (size_t a = 0; a < 4; ++a) wco_[a] = s.wco[a] * scale(a);
    }
    if (s.has_mpos) {
        for (size_t a = 0; a < 4; ++a) {
            if (static_cast<int>(a) < s.axes_reported) fw_mpos_[a] = s.mpos[a] * scale(a);
        }
    } else if (s.has_wpos) {
        for (size_t a = 0; a < 4; ++a) {
            if (static_cast<int>(a) < s.axes_reported) fw_mpos_[a] = s.wpos[a] * scale(a) + wco_[a];
        }
    }
    // FluidNC does not list its axes in $I: the status report has one value per configured axis
    if (fw_.flavour == board::grbl::Flavour::FluidNC && s.axes_reported >= 3 && s.axes_reported <= 8 &&
        s.axes_reported != fw_.axis_count) {
        fw_.axis_count = static_cast<uint8_t>(s.axes_reported);
        fw_.axis_letters = std::string("XYZABC").substr(0, std::min<size_t>(6, fw_.axis_count));
        refreshDialect();
    }
    fw_state_ = s.state;
    fw_substate_ = s.substate;
    fb_.firmware_state = s.state + (s.substate >= 0 ? ":" + std::to_string(s.substate) : std::string());
    for (size_t a = 0; a < 4; ++a) fb_.machine_pos_mm[a] = fw_mpos_[a] + shift_[a];
    fb_.position_valid = true;
    jog_report_time_ = now;
    if (s.has_feed) {
        fb_.feed_mm_min = s.feed * unit;
        fb_.spindle_rpm = s.spindle;
    }
    if (s.has_ov) {
        fb_.ov_feed = s.ov_feed;
        fb_.ov_rapid = s.ov_rapid;
        fb_.ov_spindle = s.ov_spindle;
        if (now - ov_last_send_ > std::chrono::milliseconds(400)) {
            ov_feed_expected_ = s.ov_feed;
            ov_spindle_expected_ = s.ov_spindle;
        }
        // A: is part of the override block: absent there = nothing on
        const std::string acc = s.has_accessories ? s.accessories : std::string();
        fb_.spindle_on = acc.find('S') != std::string::npos || acc.find('C') != std::string::npos;
        fb_.spindle_ccw = acc.find('C') != std::string::npos;
        fb_.flood_on = acc.find('F') != std::string::npos;
        fb_.mist_on = acc.find('M') != std::string::npos;
    } else if (s.has_accessories) {
        const std::string& acc = s.accessories;
        fb_.spindle_on = acc.find('S') != std::string::npos || acc.find('C') != std::string::npos;
        fb_.spindle_ccw = acc.find('C') != std::string::npos;
        fb_.flood_on = acc.find('F') != std::string::npos;
        fb_.mist_on = acc.find('M') != std::string::npos;
    }
    fb_.pins = s.has_pins ? s.pins : std::string();
    if (io_cache_rev_ != caps_revision_) {
        io_cache_ = capabilities().io;
        io_cache_rev_ = caps_revision_;
    }
    fb_.inputs_mask = 0;
    for (size_t i = 0; i < io_cache_.size() && i < 64; ++i) {
        const char letter = io_cache_[i].status_letter;
        if (letter && fb_.pins.find(letter) != std::string::npos) fb_.inputs_mask |= (1ULL << i);
    }
    fb_.probe_active = fb_.pins.find('P') != std::string::npos;
    fb_.door_open = fb_.pins.find('D') != std::string::npos || s.state == "Door";
    fb_.estop_active = fb_.pins.find('E') != std::string::npos;
    fb_.limit_active = false;
    for (char c : {'X', 'Y', 'Z', 'A'}) {
        if (fb_.pins.find(c) != std::string::npos) fb_.limit_active = true;
    }
    if (s.has_homed) {
        fb_.homed_reported = true;
        for (size_t a = 0; a < 4; ++a) fb_.homed[a] = s.homed && ((s.homed_mask >> a) & 1U);
    }
    if (s.has_buffer) {
        fb_.planner_free = s.buffer_blocks;
        fb_.rx_free = s.buffer_bytes;
    }
    fb_.thc_reported = s.has_thc;
    if (s.has_thc) {
        const auto& f = s.thc_flags;
        auto has = [&](char c) { return f.find(c) != std::string::npos; };
        fb_.arc_voltage = s.thc_voltage;
        fb_.arc_ok = has('A');
        fb_.torch_on = has('T');
        fb_.thc_enabled = has('E');
        fb_.thc_active = has('R');
        fb_.thc_velocity_lock = has('V');
        fb_.thc_void_lock = has('H');
        fb_.thc_up = has('U');
        fb_.thc_down = has('D');
        for (size_t i = 0; i < io_cache_.size() && i < 64; ++i) {
            if (io_cache_[i].fixed_role == board::IoRole::ArcOk && fb_.arc_ok) fb_.inputs_mask |= (1ULL << i);
        }
    } else {
        fb_.arc_ok = fb_.torch_on = fb_.thc_enabled = fb_.thc_active = false;
        fb_.thc_velocity_lock = fb_.thc_void_lock = fb_.thc_up = fb_.thc_down = false;
    }

    PathState st = fb_.state;
    if (s.state == "Idle") st = PathState::Idle;
    else if (s.state == "Run" || s.state == "Jog") st = PathState::Moving;
    else if (s.state == "Hold") st = (s.substate == 0) ? PathState::Held : PathState::Holding;
    else if (s.state == "Home") st = PathState::Homing;
    else if (s.state == "Alarm" || s.state == "Check" || s.state == "Sleep") st = PathState::Fault;
    else if (s.state == "Door" || s.state == "Tool") st = PathState::Held;
    fb_.state = (phase_ == Phase::Ready) ? st : PathState::Connecting;
    if (s.state == "Alarm" && fb_.status_text.empty()) {
        fb_.status_text = "The board is in alarm: unlock or home it";
    } else if (s.state == "Door") {
        fb_.status_text = (s.substate == 0) ? "Door closed: press Cycle Start to continue" : "Door open";
    } else if (s.state == "Check") {
        fb_.status_text = "The board is in check mode ($C)";
    } else if (s.state == "Sleep") {
        fb_.status_text = "The board is asleep: a reset is needed";
    } else if (s.state == "Idle" || s.state == "Run" || s.state == "Jog") {
        if (fb_.status_text.rfind("Door", 0) == 0 || fb_.status_text.rfind("The board is in", 0) == 0) {
            fb_.status_text.clear();
        }
    }
    if (op_ == Op::Job) {
        if (s.has_line) {
            // Ln: selects the move directly
            for (size_t i = tracker_index_; i < sent_moves_.size(); ++i) {
                if (sent_moves_[i].line_number == s.line) {
                    if (i != tracker_index_) tracker_sub_ = 0;
                    tracker_index_ = i;
                    break;
                }
            }
        }
        updateTracker();
    }
}

void FirmwarePlannedPath::onAck(const FirmwareMessage& m, Clock::time_point now) {
    if (m.ticket != 0) last_acked_ticket_ = std::max(last_acked_ticket_, m.ticket);
    status_since_ack_ = false;
    next_poll_ = now; // A fresh status report shows whether the machine is idle

    switch (phase_) {
        case Phase::Identify: {
            if (fw_.flavour != selected_flavour_ && fw_.flavour != board::grbl::Flavour::GenericGrbl) {
                PathEvent e;
                e.kind = PathEvent::Kind::Message;
                e.text = std::string("The board answered as ") + board::grbl::flavourName(fw_.flavour) + ", but " +
                         board::grbl::flavourName(selected_flavour_) +
                         " is selected. Select the matching board for its settings panel.";
                emit(e);
            }
            protocol_.setRxBufferBytes(fw_.rx_buffer_bytes);
            refreshDialect();
            phase_ = Phase::ReadSettings;
            phase_deadline_ = now + kSettingsTimeout;
            sendHandshakeLine("$$");
            return;
        }
        case Phase::ReadSettings:
            if (fw_.flavour == board::grbl::Flavour::GrblHal && fw_.settings_enumeration && enum_groups_.empty() &&
                enums_.empty()) {
                phase_ = Phase::ReadEnums;
                phase_deadline_ = now + kSettingsTimeout;
                sendHandshakeLine("$EG");
                return;
            }
            codes_queue_.clear();
            if (board_alarm_texts_.empty() && board_error_texts_.empty()) {
                if (fw_.flavour == board::grbl::Flavour::GrblHal) codes_queue_ = {"$EA", "$EE"};
                if (fw_.flavour == board::grbl::Flavour::FluidNC) codes_queue_ = {"$A", "$E"};
            }
            readCodesOrParams(now);
            return;
        case Phase::ReadEnums:
            if (enums_.empty()) {
                phase_deadline_ = now + kSettingsTimeout;
                enums_[""] = {}; // Marker: $ES requested
                sendHandshakeLine("$ES");
                return;
            }
            enums_.erase("");
            codes_queue_.clear();
            if (board_alarm_texts_.empty() && board_error_texts_.empty()) codes_queue_ = {"$EA", "$EE"};
            readCodesOrParams(now);
            return;
        case Phase::ReadCodes:
            readCodesOrParams(now);
            return;
        case Phase::ReadParams: {
            const auto it = params_.find("G54");
            if (it != params_.end()) frame_f_ = it->second;
            settings_.items = board::grbl::buildNumberedItems(numbered_, enums_, enum_groups_);
            settings_.status = "Read " + std::to_string(settings_.items.size()) + " settings";
            ++settings_.revision;
            phase_ = Phase::Ready;
            ready_flag_ = true;
            refreshDialect();
            fb_.state = (fw_state_ == "Alarm") ? PathState::Fault : PathState::Idle;
            if (fb_.status_text.rfind("Connecting", 0) == 0) fb_.status_text.clear();
            if (fw_state_ == "Alarm") fb_.status_text = "The board is in alarm: unlock or home it";
            PathEvent e;
            e.kind = PathEvent::Kind::Connected;
            e.text = fw_.describe();
            emit(e);
            return;
        }
        default:
            break;
    }

    if (script_waiting_ && m.ticket == script_ticket_) {
        script_waiting_ = false;
        if (script_current_.params) {
            const auto it = params_.find("G54");
            if (it != params_.end()) frame_f_ = it->second;
        }
        if (script_current_.settings) {
            settings_.items = board::grbl::buildNumberedItems(numbered_, enums_, enum_groups_);
            settings_.yaml = false;
            settings_.loading = false;
            settings_.status = "Read " + std::to_string(settings_.items.size()) + " settings";
            ++settings_.revision;
        }
        if (script_current_.collect) {
            settings_.items = board::grbl::parseFluidNcConfig(collected_);
            settings_.yaml = true;
            settings_.loading = false;
            settings_.status = "Read " + std::to_string(settings_.items.size()) + " configuration values";
            ++settings_.revision;
            collected_.clear();
        }
        script_done_at_ = now;
        return;
    }
    if (op_ == Op::Job) {
        if (job_preamble_waiting_ && m.ticket == job_preamble_ticket_) {
            job_preamble_waiting_ = false;
            const auto it = params_.find("G54");
            if (it != params_.end()) frame_f_ = it->second;
        }
        if (m.ticket == job_drain_ticket_ && job_drain_ticket_ != 0) job_drain_acked_ = true;
        updateTracker();
    }
    if (op_ == Op::JogStep && m.ticket == jog_step_ticket_) jog_step_acked_ = true;
}

void FirmwarePlannedPath::onNack(const FirmwareMessage& m, Clock::time_point now) {
    const std::string why = "error " + std::to_string(m.code) + ": " + errorText(m.code);
    if (phase_ != Phase::Ready && phase_ != Phase::Closed) {
        // An optional request the board does not support
        FirmwareMessage ack = m;
        ack.kind = MessageKind::Ack;
        onAck(ack, now);
        return;
    }
    if (m.ticket != 0) last_acked_ticket_ = std::max(last_acked_ticket_, m.ticket);
    status_since_ack_ = false;
    next_poll_ = now;

    if (script_waiting_ && m.ticket == script_ticket_ && script_current_.report) {
        script_waiting_ = false; // Kept in the report; carry on with the next command
        return;
    }
    if (script_waiting_ && m.ticket == script_ticket_) {
        script_waiting_ = false;
        const std::string sent = script_current_.text;
        if (sent.size() > 1 && sent[0] == '$' && sent.find('=') != std::string::npos) {
            // A setting write: show the reason next to the setting and continue with the others
            const std::string id = sent.substr(1, sent.find('=') - 1);
            for (auto& it : settings_.items) {
                if (it.id == id || ("/" + it.id) == id || it.id == "/" + id) it.error = errorText(m.code);
            }
            settings_.status = "$" + id + " was not accepted: " + errorText(m.code);
            ++settings_.revision;
            return;
        }
        op_error_ = "The board rejected '" + sent + "' (" + why + ")";
        log_->add(LogDir::Error, op_error_);
        script_.clear();
        if (op_ == Op::Script) {
            PathEvent e;
            e.kind = PathEvent::Kind::Message;
            e.text = op_error_;
            emit(e);
            op_ = Op::None;
        } else {
            finishOp(false);
        }
        return;
    }
    if (op_ == Op::Job) {
        if (job_preamble_waiting_ && m.ticket == job_preamble_ticket_) job_preamble_waiting_ = false;
        // A streamed line was rejected: hold at once, report the line, then the stop sequence
        protocol_.realtime(Realtime::FeedHold, fw_.top_bit_realtime);
        std::string where;
        for (const auto& mv : sent_moves_) {
            if (mv.ticket == m.ticket) {
                where = "line " + std::to_string(mv.gcode_line + 1) + ": ";
                break;
            }
        }
        PathEvent e;
        e.kind = PathEvent::Kind::Message;
        e.ok = false;
        e.text = "Job stopped: the board rejected a line (" + where + why + ")";
        emit(e);
        stop();
        return;
    }
    if (op_ == Op::JogContinuous || op_ == Op::JogStep) {
        protocol_.realtime(Realtime::JogCancel);
        jog_cancelling_ = true;
        jog_cancel_at_ = now;
        jog_vel_ = {};
        PathEvent e;
        e.kind = PathEvent::Kind::Message;
        e.ok = false;
        e.text = (m.code == board::grbl::kErrorJogTravelExceeded) ? "Jog stopped at the board's soft limit"
                                                                 : "Jog rejected by the board (" + why + ")";
        emit(e);
        if (op_ == Op::JogStep) finishOp(false);
        return;
    }
}

void FirmwarePlannedPath::onAlarm(int code) {
    auto info = (fw_.flavour == board::grbl::Flavour::FluidNC) ? board::grbl::fluidNcAlarmInfo(code)
                                                               : board::grbl::alarmInfo(code);
    info.text = alarmText(code);
    if (report_running_) {
        report_running_ = false;
        report_lines_.push_back("# The report was interrupted by ALARM:" + std::to_string(code));
        ++report_revision_;
    }
    fw_state_ = "Alarm";
    // An alarm resets the board's input buffer: lines it had not answered are gone
    protocol_.forgetUnacknowledged();
    const Op prev = op_;
    op_ = Op::None;
    deferred_ = nullptr;
    script_.clear();
    script_waiting_ = false;
    job_lines_.clear();
    job_phase_ = JobPhase::Done;
    fb_.job_active = false;
    jog_vel_ = {};
    PathEvent done;
    done.ok = false;
    switch (prev) {
        case Op::Moves:
        case Op::JogStep:       done.kind = PathEvent::Kind::MoveDone; emit(done); break;
        case Op::Probe:         done.kind = PathEvent::Kind::ProbeDone; emit(done); break;
        case Op::Homing:        done.kind = PathEvent::Kind::HomingDone; emit(done); break;
        case Op::Job:           done.kind = PathEvent::Kind::JobFinished; emit(done); break;
        case Op::JogContinuous: done.kind = PathEvent::Kind::JogStopped; emit(done); break;
        case Op::Stopping:      expect_reset_ = false; break;
        default: break;
    }
    if (expect_reset_ && code == 3) {
        // Our own E-stop while moving: reported once by the core
        fb_.homed = {};
        fb_.status_text = "Board reset while moving: position lost, home the machine";
        return;
    }
    fault("Board alarm " + std::to_string(code) + ": " + info.text, code, info.position_lost);
}

// ============================================================================
// Scripts: send-response command sequences
// ============================================================================
bool FirmwarePlannedPath::canStart(std::string* error) const {
    if (!isOpen() || phase_ != Phase::Ready) {
        if (error) *error = "The board is not connected";
        return false;
    }
    // A running command sequence (outputs, console, settings) is not in the way: new commands queue behind it
    if ((op_ != Op::None && op_ != Op::Script) || deferred_) {
        if (error) *error = "The board is busy";
        return false;
    }
    return true;
}

void FirmwarePlannedPath::startScript(Op op, std::vector<ScriptLine> lines) {
    if (op_ == Op::Script) {
        // Queue behind the running sequence; a motion command takes over its completion (MoveDone, ProbeDone, ...)
        for (auto& l : lines) script_.push_back(l);
        if (op != Op::Script) {
            op_ = op;
            probe_result_seen_ = false;
            probe_hit_ = false;
            op_error_.clear();
        }
        return;
    }
    op_ = op;
    script_.assign(lines.begin(), lines.end());
    script_waiting_ = false;
    op_error_.clear();
    collected_.clear();
    probe_result_seen_ = false;
    probe_hit_ = false;
    script_done_at_ = Clock::now();
    pumpScript(Clock::now());
}

void FirmwarePlannedPath::pumpScript(Clock::time_point now) {
    if (script_waiting_) return;
    if (!script_.empty()) {
        if (protocol_.unacknowledged() != 0) return;
        script_current_ = script_.front();
        script_.pop_front();
        if (script_current_.report) report_lines_.push_back("> " + script_current_.text);
        script_ticket_ = protocol_.sendLine(script_current_.text);
        script_waiting_ = script_ticket_ != 0;
        script_line_sent_at_ = now;
        return;
    }
    // Every line answered. A fresh status report follows (the state may have changed, e.g. after $X); motion
    // commands also wait until it shows Idle.
    if (!status_since_ack_) return;
    if ((op_ == Op::Moves || op_ == Op::Probe || op_ == Op::Homing) && fw_state_ != "Idle") return;
    finishOp(true);
}

void FirmwarePlannedPath::finishOp(bool ok) {
    const Op o = op_;
    op_ = Op::None;
    if (report_running_) {
        report_running_ = false;
        if (!ok) report_lines_.push_back("# The report was interrupted");
        ++report_revision_;
    }
    script_.clear();
    script_waiting_ = false;
    PathEvent e;
    e.ok = ok;
    e.text = op_error_;
    switch (o) {
        case Op::Moves:
        case Op::JogStep:
            e.kind = PathEvent::Kind::MoveDone;
            emit(e);
            break;
        case Op::Probe:
            e.kind = PathEvent::Kind::ProbeDone;
            e.ok = ok && probe_result_seen_ && probe_hit_;
            for (size_t a = 0; a < 4; ++a) e.position_mm[a] = probe_pos_[a] + shift_[a];
            emit(e);
            break;
        case Op::Homing:
            if (ok) {
                for (size_t a = 0; a < 4; ++a) {
                    if (homing_mask_ == 0 || ((homing_mask_ >> a) & 1U)) {
                        if (a < fw_.axis_count || homing_mask_ == 0) fb_.homed[a] = (a < fw_.axis_count);
                        shift_[a] = 0.0; // The homed board defines machine zero
                    }
                }
                for (size_t a = 0; a < 4; ++a) fb_.machine_pos_mm[a] = fw_mpos_[a] + shift_[a];
            }
            e.kind = PathEvent::Kind::HomingDone;
            emit(e);
            break;
        case Op::Script:
            if (settings_after_op_) {
                settings_after_op_ = false;
                e.kind = PathEvent::Kind::SettingsChanged;
                emit(e);
                requestSettings();
            }
            break;
        default:
            break;
    }
}

bool FirmwarePlannedPath::sendConsoleLine(const std::string& line_in, std::string* error) {
    std::string line = line_in;
    while (!line.empty() && (line.back() == ' ' || line.back() == '\r' || line.back() == '\n')) line.pop_back();
    if (line.empty()) return true;
    if (line == "?" || line == "!" || line == "~") {
        const Realtime r = (line == "?") ? Realtime::StatusReport : (line == "!") ? Realtime::FeedHold : Realtime::CycleStart;
        return protocol_.realtime(r, fw_.top_bit_realtime);
    }
    if (!canStart(error)) return false;
    if (fw_state_ != "Idle" && fw_state_ != "Alarm") {
        if (error) *error = "Console commands are accepted only while the board is idle";
        return false;
    }
    ScriptLine l;
    l.text = line;
    l.quiet = line.rfind("$H", 0) == 0;
    l.collect = line == "$CD" || line == "$S";
    std::vector<ScriptLine> lines{l};
    if (line.rfind("$H", 0) != 0 && line.find('=') == std::string::npos) {
        // The command may have changed offsets: re-read them (F is taken from G54 again by every job)
        ScriptLine p;
        p.text = "$#";
        p.params = true;
        lines.push_back(p);
    }
    settings_after_op_ = line.size() > 1 && line[0] == '$' && line.find('=') != std::string::npos;
    emitter_.reset();
    startScript(Op::Script, lines);
    return true;
}

bool FirmwarePlannedPath::runLines(const std::vector<std::string>& lines, std::string* error) {
    if (!canStart(error)) return false;
    std::vector<ScriptLine> sl;
    for (const auto& l : lines) {
        ScriptLine x;
        x.text = l;
        sl.push_back(x);
    }
    emitter_.reset();
    startScript(Op::Script, sl);
    return true;
}

bool FirmwarePlannedPath::unlock(std::string* error) {
    if (!canStart(error)) return false;
    ScriptLine l;
    l.text = "$X";
    startScript(Op::Script, {l});
    fb_.status_text.clear();
    return true;
}

void FirmwarePlannedPath::readCodesOrParams(Clock::time_point now) {
    if (!codes_queue_.empty()) {
        const std::string line = codes_queue_.front();
        codes_queue_.pop_front();
        codes_target_ = line.back() == 'A' ? 'A' : 'E';
        phase_ = Phase::ReadCodes;
        phase_deadline_ = now + kSettingsTimeout;
        sendHandshakeLine(line);
        return;
    }
    codes_target_ = 0;
    phase_ = Phase::ReadParams;
    phase_deadline_ = now + kIdentifyTimeout;
    sendHandshakeLine("$#");
}

std::string FirmwarePlannedPath::alarmText(int code) const {
    // GRBL 1.1's alarms 1-9 mean the same on all three: BreakoutCNC's wording. Beyond that the codes differ
    // between grblHAL and FluidNC, so the board's own text wins.
    if (code >= 10) {
        const auto it = board_alarm_texts_.find(code);
        if (it != board_alarm_texts_.end()) return it->second;
    }
    return board::grbl::alarmInfo(code).text;
}

std::string FirmwarePlannedPath::errorText(int code) const {
    if (code > 38) {
        const auto it = board_error_texts_.find(code);
        if (it != board_error_texts_.end()) return it->second;
    }
    return board::grbl::errorText(code);
}

bool FirmwarePlannedPath::travelEnvelope(size_t a, double& lo, double& hi) const {
    if (a >= 4 || a >= fw_.axis_count || !fw_.settings_read || !fw_.homing_enabled) return false;
    if (fw_.flavour == board::grbl::Flavour::FluidNC) return false; // Its own rules (homing/mpos_mm): to confirm
    const double travel = std::abs(fw_.max_travel_mm[a]);
    if (travel < 1e-6) return false;
    // GRBL: machine zero at the home switch and the travel on the negative side; with build option Z (force
    // origin) an axis that homes toward its minimum has its travel on the positive side.
    const bool homes_negative = ((fw_.homing_dir_invert_mask >> a) & 1U) != 0;
    if (fw_.origin_at_home && homes_negative) {
        lo = 0.0;
        hi = travel;
    } else {
        lo = -travel;
        hi = 0.0;
    }
    lo += shift_[a];
    hi += shift_[a];
    return true;
}

bool FirmwarePlannedPath::startBoardReport(std::string* error) {
    if (!canStart(error)) return false;
    if (fw_state_ != "Idle" && fw_state_ != "Alarm") {
        if (error) *error = "The report is made only while the board is idle";
        return false;
    }
    std::vector<std::string> cmds;
    switch (fw_.flavour) {
        case board::grbl::Flavour::GrblHal: cmds = {"$I", "$I+", "$$", "$#", "$G", "$N"}; break;
        case board::grbl::Flavour::FluidNC: cmds = {"$I", "$$", "$#", "$G", "$CD", "$S"}; break;
        default:                            cmds = {"$I", "$$", "$#", "$G", "$N"}; break;
    }
    std::vector<ScriptLine> lines;
    for (const auto& c : cmds) {
        ScriptLine l;
        l.text = c;
        l.report = true;
        lines.push_back(l);
    }
    report_lines_.clear();
    report_running_ = true;
    emitter_.reset();
    startScript(Op::Script, lines);
    return true;
}

std::string FirmwarePlannedPath::boardReportText() const {
    std::string r;
    auto line = [&](const std::string& s) { r += s + "\n"; };
    auto yn = [](bool b) { return b ? std::string("yes") : std::string("no"); };
    const auto caps = capabilities();
    line("Board: " + caps.display_name + " (" + board_id_ + "), " + connectionLabel());
    line("Firmware: " + fw_.describe() + " | detected as " + board::grbl::flavourName(fw_.flavour) +
         (fw_.detected ? "" : " (not confirmed)"));
    line("Welcome: " + fw_.welcome);
    line("VER: " + fw_.version_text + " | OPT: " + fw_.option_codes + " | NEWOPT: " + fw_.new_options);
    line("FIRMWARE: " + fw_.firmware_name + " | BOARD: " + fw_.board_name);
    line("Axes: " + std::to_string(fw_.axis_count) + " (" + fw_.axis_letters + ") | planner blocks " +
         std::to_string(fw_.planner_blocks) + " | receive buffer " + std::to_string(fw_.rx_buffer_bytes) +
         " bytes | longest line " + std::to_string(fw_.max_line_chars));
    line("Line numbers: " + yn(fw_.reports_line_numbers) + " | homed state: " + yn(fw_.reports_homed) +
         " | single-axis homing: " + yn(fw_.home_single_axis) + " | origin at home (Z): " + yn(fw_.origin_at_home) +
         " | mist: " + yn(fw_.has_mist) + " | settings enumeration: " + yn(fw_.settings_enumeration) +
         " | top-bit real-time: " + yn(fw_.top_bit_realtime));
    line("Aux outputs: " + std::to_string(fw_.aux_outputs) + " | aux inputs: " + std::to_string(fw_.aux_inputs) +
         " | plasma plugin: " + yn(fw_.plasma_plugin) + (fw_.plasma_plugin ? " (" + fw_.plasma_mode + ")" : ""));
    char buf[256];
    std::snprintf(buf, sizeof(buf), "Settings read: %s | inches: %s | homing: %s | soft limits: %s | laser mode: %s | "
                  "spindle %.0f..%.0f RPM | homing dir mask %u",
                  yn(fw_.settings_read).c_str(), yn(fw_.reports_inches).c_str(), yn(fw_.homing_enabled).c_str(),
                  yn(fw_.soft_limits).c_str(), yn(fw_.laser_mode).c_str(), fw_.min_spindle_rpm, fw_.max_spindle_rpm,
                  static_cast<unsigned>(fw_.homing_dir_invert_mask));
    line(buf);
    for (size_t a = 0; a < std::min<size_t>(4, fw_.axis_count); ++a) {
        double lo = 0.0, hi = 0.0;
        const bool env = travelEnvelope(a, lo, hi);
        std::snprintf(buf, sizeof(buf), "%c: %.3f steps/mm, %.0f mm/min, %.1f mm/s2, travel %.1f mm%s", kAxis[a],
                      fw_.steps_per_mm[a], fw_.max_rate_mm_min[a], fw_.accel_mm_s2[a], fw_.max_travel_mm[a],
                      env ? "" : " (envelope not known)");
        std::string s = buf;
        if (env) {
            std::snprintf(buf, sizeof(buf), " -> app machine %.1f..%.1f", lo, hi);
            s += buf;
        }
        line(s);
    }
    std::snprintf(buf, sizeof(buf), "Machine shift: %.3f %.3f %.3f %.3f | board code texts: %zu alarms, %zu errors",
                  shift_[0], shift_[1], shift_[2], shift_[3], board_alarm_texts_.size(), board_error_texts_.size());
    line(buf);
    line("Capabilities: homing " + std::string(caps.homing == board::HomingKind::BoardCycle ? "board cycle" : "app") +
         " | single axis " + yn(caps.home_single_axis) + " | line tracking " + yn(caps.precise_line_tracking) +
         " | rapid override " + yn(caps.rapid_override) + " | E-stop input " + yn(caps.estop_input) +
         " | board THC " + yn(caps.board_thc) + " | aux outputs " + std::to_string(caps.aux_outputs));
    std::string io;
    for (const auto& p : caps.io) io += (io.empty() ? "" : ", ") + p.label;
    line("Inputs and outputs: " + io);
    line("");
    line("---- Answers ----");
    for (const auto& l : report_lines_) line(l);
    line("");
    line("---- Recent status reports ----");
    const auto entries = log_->since(0, 3000);
    std::vector<const transport::ProtocolLogEntry*> statuses, others;
    for (const auto& e : entries) (e.is_status ? statuses : others).push_back(&e);
    auto put = [&](const transport::ProtocolLogEntry& e) {
        using D = transport::ProtocolLogEntry::Dir;
        const char* p = (e.dir == D::Sent) ? "> " : (e.dir == D::Received) ? "  " : "# ";
        std::snprintf(buf, sizeof(buf), "%9.3f ", e.time_s);
        line(buf + std::string(p) + e.text);
    };
    for (size_t i = statuses.size() > 10 ? statuses.size() - 10 : 0; i < statuses.size(); ++i) put(*statuses[i]);
    line("");
    line("---- Recent exchange (without status reports) ----");
    for (size_t i = others.size() > 300 ? others.size() - 300 : 0; i < others.size(); ++i) put(*others[i]);
    return r;
}

void FirmwarePlannedPath::requestSettings() {
    if (!canStart(nullptr)) return;
    settings_.loading = true;
    ++settings_.revision;
    std::vector<ScriptLine> lines;
    ScriptLine all;
    all.text = "$$";
    all.settings = true;
    lines.push_back(all);
    if (fw_.flavour == board::grbl::Flavour::FluidNC) {
        ScriptLine cd;
        cd.text = "$CD";
        cd.collect = true;
        lines.push_back(cd);
    }
    startScript(Op::Script, lines);
}

bool FirmwarePlannedPath::writeSettings(const std::vector<std::pair<std::string, std::string>>& id_values,
                                        std::string* error) {
    if (!canStart(error)) return false;
    if (fw_state_ != "Idle" && fw_state_ != "Alarm") {
        if (error) *error = "Settings can only be changed while the board is idle";
        return false;
    }
    std::vector<ScriptLine> lines;
    for (const auto& [id, value] : id_values) {
        for (auto& it : settings_.items) {
            if (it.id == id) it.error.clear();
        }
        ScriptLine l;
        l.text = "$" + id + "=" + value;
        lines.push_back(l);
    }
    if (lines.empty()) return true;
    settings_after_op_ = true;
    startScript(Op::Script, lines);
    return true;
}

// ============================================================================
// Single moves, probing, homing
// ============================================================================
std::array<double, 4> FirmwarePlannedPath::toFirmware(const std::array<double, 4>& m, bool with_frame) const {
    std::array<double, 4> r{};
    for (size_t a = 0; a < 4; ++a) r[a] = m[a] - shift_[a] - (with_frame ? frame_f_[a] : 0.0);
    return r;
}

bool FirmwarePlannedPath::startMoves(const std::vector<MoveRequest>& moves, const OutputRequest* during,
                                     std::string* error) {
    // Moves may follow a running command sequence (outputs, console); they are sent after it
    const bool append = op_ == Op::Script && isOpen() && phase_ == Phase::Ready && fw_state_ != "Alarm";
    if (!append) {
        if (!canStart(error)) return false;
        if (fw_state_ != "Idle") {
            if (error) *error = "The board is not idle (" + fb_.firmware_state + ")";
            return false;
        }
    }
    std::vector<ScriptLine> lines;
    const int n_axes = std::clamp<int>(fw_.axis_count, 3, 4);
    if (during && during->spindle_on) {
        ScriptLine l;
        l.text = std::string(during->laser && during->laser_dynamic ? "M4" : "M3") + "S" + GcodeEmitter::num(during->s_value, 0);
        lines.push_back(l);
    }
    for (const auto& mv : moves) {
        if (mv.dwell_s > 0.0) {
            ScriptLine d;
            d.text = "G4P" + GcodeEmitter::num(mv.dwell_s, 3);
            lines.push_back(d);
            continue;
        }
        const auto fw = toFirmware(mv.target_mm, false);
        std::string t = "G21G90G53";
        t += mv.rapid ? "G0" : "G1";
        for (int a = 0; a < n_axes; ++a) {
            t += kAxis[a];
            t += fmt(fw[static_cast<size_t>(a)]);
        }
        if (!mv.rapid) t += "F" + GcodeEmitter::num(std::max(1.0, mv.feed_mm_min), 0);
        ScriptLine l;
        l.text = t;
        lines.push_back(l);
    }
    if (during && during->spindle_on) {
        ScriptLine l;
        l.text = "M5";
        lines.push_back(l);
    }
    ScriptLine wait;
    wait.text = "G4P0";
    lines.push_back(wait);
    emitter_.reset();
    if (append) {
        // Continue the running command sequence (e.g. spindle off, then the tool change retract)
        for (auto& l : lines) script_.push_back(l);
        op_ = Op::Moves;
        return true;
    }
    startScript(Op::Moves, lines);
    return true;
}

bool FirmwarePlannedPath::startProbe(const ProbeRequest& p, std::string* error) {
    if (!canStart(error)) return false;
    if (fw_state_ != "Idle") {
        if (error) *error = "The board is not idle (" + fb_.firmware_state + ")";
        return false;
    }
    if (fb_.probe_active) {
        if (error) *error = "The probe is already in contact - move it clear first";
        return false;
    }
    const int n_axes = std::clamp<int>(fw_.axis_count, 3, 4);
    std::string t = "G21G91G38.3";
    bool any = false;
    for (int a = 0; a < n_axes; ++a) {
        const double d = p.target_mm[static_cast<size_t>(a)] - fb_.machine_pos_mm[static_cast<size_t>(a)];
        if (std::abs(d) >= 0.0005) {
            t += kAxis[a];
            t += fmt(d);
            any = true;
        }
    }
    if (!any) {
        if (error) *error = "The probe move has no length";
        return false;
    }
    t += "F" + GcodeEmitter::num(std::max(1.0, p.feed_mm_min), 0);
    ScriptLine probe;
    probe.text = t;
    ScriptLine abs;
    abs.text = "G90";
    emitter_.reset();
    startScript(Op::Probe, {probe, abs});
    return true;
}

bool FirmwarePlannedPath::startHoming(uint8_t axis_mask, std::string* error) {
    if (!canStart(error)) return false;
    if (!fw_.homing_enabled && fw_.settings_read && fw_.flavour != board::grbl::Flavour::FluidNC) {
        if (error) *error = "Homing is disabled on the board ($22 = 0)";
        return false;
    }
    ScriptLine l;
    l.text = "$H";
    l.quiet = true;
    if (axis_mask != 0) {
        if (!fw_.home_single_axis) {
            if (error) *error = "This board can only home all axes together";
            return false;
        }
        for (size_t a = 0; a < 4; ++a) {
            if ((axis_mask >> a) & 1U) l.text += kAxis[a];
        }
    }
    homing_mask_ = axis_mask;
    for (size_t a = 0; a < 4; ++a) {
        if (axis_mask == 0 || ((axis_mask >> a) & 1U)) fb_.homed[a] = false;
    }
    emitter_.reset();
    startScript(Op::Homing, {l});
    return true;
}

// ============================================================================
// Jogging
// ============================================================================
bool FirmwarePlannedPath::jogStep(size_t axis, double delta_mm, double feed, std::string* error) {
    if (!canStart(error)) return false;
    if (op_ == Op::Script) {
        // Started once the running command sequence is answered
        deferred_ = [this, axis, delta_mm, feed] {
            std::string e;
            if (!jogStep(axis, delta_mm, feed, &e)) {
                PathEvent ev;
                ev.kind = PathEvent::Kind::MoveDone;
                ev.ok = false;
                ev.text = "Jog rejected: " + e;
                emit(ev);
            }
        };
        return true;
    }
    if (fw_state_ != "Idle") {
        if (error) *error = "The board is not idle (" + fb_.firmware_state + ")";
        return false;
    }
    if (axis >= 4) return false;
    std::array<double, 4> d{};
    d[axis] = delta_mm;
    const std::string line = GcodeEmitter::jogLine(d, feed, std::clamp<int>(fw_.axis_count, 3, 4));
    jog_step_ticket_ = protocol_.sendLine(line);
    if (jog_step_ticket_ == 0) {
        if (error) *error = "Write to the board failed";
        return false;
    }
    jog_step_acked_ = false;
    status_since_ack_ = false;
    script_line_sent_at_ = Clock::now();
    op_ = Op::JogStep;
    return true;
}

void FirmwarePlannedPath::setJogVelocity(const std::array<double, 4>& v) {
    jog_vel_ = v;
}

void FirmwarePlannedPath::pumpJog(Clock::time_point now) {
    double speed = 0.0;
    for (double x : jog_vel_) speed += x * x;
    speed = std::sqrt(speed);

    if (op_ == Op::None) {
        if (speed < 1e-6 || phase_ != Phase::Ready || fw_state_ != "Idle") return;
        op_ = Op::JogContinuous;
        for (size_t a = 0; a < 4; ++a) jog_dir_[a] = jog_vel_[a] / speed;
        jog_speed_ = speed;
        jog_sent_ = 0.0;
        jog_start_ = fb_.machine_pos_mm;
        jog_cancelling_ = false;
    }
    if (op_ != Op::JogContinuous) return;

    if (jog_cancelling_) {
        if (fw_state_ == "Idle" && now - jog_cancel_at_ > std::chrono::milliseconds(120) && status_since_ack_ &&
            last_status_ > jog_cancel_at_) {
            op_ = Op::None;
            jog_cancelling_ = false;
            PathEvent e;
            e.kind = PathEvent::Kind::JogStopped;
            emit(e);
        }
        return;
    }
    if (speed < 1e-6) {
        protocol_.realtime(Realtime::JogCancel);
        jog_cancelling_ = true;
        jog_cancel_at_ = now;
        return;
    }
    double dot = 0.0;
    for (size_t a = 0; a < 4; ++a) dot += (jog_vel_[a] / speed) * jog_dir_[a];
    if (dot < 0.999) {
        // New direction: stop, then start again with the new vector
        protocol_.realtime(Realtime::JogCancel);
        jog_cancelling_ = true;
        jog_cancel_at_ = now;
        return;
    }
    jog_speed_ = speed;

    // Lead = distance sent minus distance travelled (extrapolated between status reports)
    double travelled = 0.0;
    for (size_t a = 0; a < 4; ++a) travelled += (fb_.machine_pos_mm[a] - jog_start_[a]) * jog_dir_[a];
    if (fw_state_ == "Jog") {
        const double dt = std::chrono::duration<double>(now - jog_report_time_).count();
        travelled += std::min(0.2, std::max(0.0, dt)) * fb_.feed_mm_min / 60.0;
    }
    double accel = 1e9;
    for (size_t a = 0; a < 4; ++a) {
        const double u = std::abs(jog_dir_[a]);
        if (u > 1e-9) accel = std::min(accel, std::max(1.0, fw_.accel_mm_s2[a]) / u);
    }
    if (accel > 1e8) accel = 100.0;
    const double lead_target = speed * speed / (2.0 * accel) + speed * kJogLeadTime_s;
    const double chunk = std::max(0.01, speed * kJogChunkTime_s);
    int sent_now = 0;
    while (jog_sent_ - travelled < lead_target && sent_now < 4 && protocol_.unacknowledged() < 6) {
        std::array<double, 4> d{};
        for (size_t a = 0; a < 4; ++a) d[a] = jog_dir_[a] * chunk;
        const std::string line = GcodeEmitter::jogLine(d, speed * 60.0, std::clamp<int>(fw_.axis_count, 3, 4));
        if (!protocol_.canSend(line.size())) break;
        if (protocol_.sendLine(line) == 0) break;
        jog_sent_ += chunk;
        ++sent_now;
    }
}

// ============================================================================
// Feed hold, resume, stop, E-stop, overrides, outputs
// ============================================================================
void FirmwarePlannedPath::hold() {
    if (!isOpen()) return;
    if (op_ == Op::JogContinuous || op_ == Op::JogStep) {
        protocol_.realtime(Realtime::JogCancel);
        jog_vel_ = {};
        return;
    }
    protocol_.realtime(Realtime::FeedHold, fw_.top_bit_realtime);
    job_held_ = true;
}

void FirmwarePlannedPath::resume() {
    if (!isOpen()) return;
    protocol_.realtime(Realtime::CycleStart, fw_.top_bit_realtime);
    job_held_ = false;
}

void FirmwarePlannedPath::stop() {
    if (!isOpen() || op_ == Op::None || op_ == Op::Stopping || op_ == Op::Script) return;
    const auto now = Clock::now();
    if (op_ == Op::JogContinuous || op_ == Op::JogStep) {
        protocol_.realtime(Realtime::JogCancel);
        jog_vel_ = {};
        jog_cancelling_ = true;
        jog_cancel_at_ = now;
        return;
    }
    if (op_ == Op::Job && job_phase_ == JobPhase::SyncWait) {
        // At rest at a sync point: nothing is moving, just forget the job
        op_ = Op::None;
        job_phase_ = JobPhase::Done;
        fb_.job_active = false;
        PathEvent e;
        e.kind = PathEvent::Kind::JobFinished;
        e.ok = false;
        emit(e);
        return;
    }
    stop_prev_op_ = op_;
    stop_from_job_ = op_ == Op::Job;
    protocol_.realtime(Realtime::FeedHold, fw_.top_bit_realtime);
    op_ = Op::Stopping;
    stop_phase_ = StopPhase::HoldWait;
    stop_deadline_ = now + kHoldTimeout;
    script_.clear();
    script_waiting_ = false;
    job_lines_.clear();
}

void FirmwarePlannedPath::emergencyStop() {
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    // Several callers may ask in the same moment (button, fault handling): one reset is enough. Also while the board
    // is being identified: it may be moving on its own (startup blocks, a pendant); tick() then starts over.
    if (now_ns - last_reset_ns_.load() < 300'000'000LL) return;
    if (protocol_.realtime(Realtime::SoftReset)) {
        last_reset_ns_ = now_ns;
        estop_requested_ = true;
    }
}

void FirmwarePlannedPath::setFeedOverride(float pct) {
    ov_feed_target_ = std::clamp(pct, 10.0f, 200.0f);
}

void FirmwarePlannedPath::setSpindleOverride(float pct) {
    ov_spindle_target_ = std::clamp(pct, 10.0f, 200.0f);
}

void FirmwarePlannedPath::setRapidOverride(float pct) {
    const int r = pct >= 75.0f ? 100 : (pct >= 37.5f ? 50 : 25);
    if (r != ov_rapid_target_) {
        ov_rapid_target_ = r;
        ov_rapid_dirty_ = true;
    }
}

void FirmwarePlannedPath::pumpOverrides() {
    if (phase_ != Phase::Ready) return;
    auto step = [&](float target_f, int& expected, Realtime r100, Realtime p10, Realtime m10, Realtime p1, Realtime m1) {
        const int target = static_cast<int>(std::lround(target_f));
        for (int i = 0; i < 3 && expected != target; ++i) {
            if (target == 100 && std::abs(expected - 100) > 1) {
                protocol_.realtime(r100);
                expected = 100;
            } else {
                const int d = target - expected;
                if (d >= 10) { protocol_.realtime(p10); expected += 10; }
                else if (d <= -10) { protocol_.realtime(m10); expected -= 10; }
                else if (d > 0) { protocol_.realtime(p1); expected += 1; }
                else { protocol_.realtime(m1); expected -= 1; }
            }
            ov_last_send_ = Clock::now();
        }
    };
    step(ov_feed_target_, ov_feed_expected_, Realtime::FeedOv100, Realtime::FeedOvPlus10, Realtime::FeedOvMinus10,
         Realtime::FeedOvPlus1, Realtime::FeedOvMinus1);
    step(ov_spindle_target_, ov_spindle_expected_, Realtime::SpindleOv100, Realtime::SpindleOvPlus10,
         Realtime::SpindleOvMinus10, Realtime::SpindleOvPlus1, Realtime::SpindleOvMinus1);
    if (ov_rapid_dirty_) {
        ov_rapid_dirty_ = false;
        protocol_.realtime(ov_rapid_target_ == 100 ? Realtime::RapidOv100
                           : ov_rapid_target_ == 50 ? Realtime::RapidOv50 : Realtime::RapidOv25);
    }
}

bool FirmwarePlannedPath::setOutputs(const OutputRequest& o, std::string* error) {
    if (!isOpen() || phase_ != Phase::Ready) {
        if (error) *error = "The board is not connected";
        return false;
    }
    if (op_ == Op::Job) {
        // The program controls the spindle; coolant can be toggled while it runs
        if (o.flood != fb_.flood_on) protocol_.realtime(Realtime::FloodToggle);
        if (o.mist != fb_.mist_on && fw_.has_mist) protocol_.realtime(Realtime::MistToggle);
        return true;
    }
    if (op_ != Op::None || (fw_state_ != "Idle" && fw_state_ != "Alarm")) {
        outputs_pending_ = true;
        outputs_req_ = o;
        return true;
    }
    if (fw_state_ == "Alarm") {
        if (error) *error = "The board is in alarm";
        return false;
    }
    ScriptLine sp;
    if (o.spindle_on) {
        sp.text = std::string((o.spindle_ccw || (o.laser && o.laser_dynamic)) ? "M4" : "M3") + "S" +
                  GcodeEmitter::num(std::max(0.0, o.s_value), 0);
    } else {
        sp.text = "M5";
    }
    // M7, M8 and M9 are one modal group: one per line
    std::vector<ScriptLine> lines{sp};
    for (const char* w : {"M9", o.flood ? "M8" : "", (o.mist && fw_.has_mist) ? "M7" : ""}) {
        if (*w == '\0') continue;
        ScriptLine c;
        c.text = w;
        lines.push_back(c);
    }
    emitter_.reset();
    startScript(Op::Script, lines);
    return true;
}

bool FirmwarePlannedPath::setAuxOutput(int index, bool on, std::string* error) {
    if (index < 0 || index >= emitter_.dialect().aux_outputs) {
        if (error) *error = "The board has no aux output " + std::to_string(index);
        return false;
    }
    if (!canStart(error)) return false;
    ScriptLine l;
    l.text = "M" + std::to_string(on ? 64 : 65) + "P" + std::to_string(index);
    startScript(Op::Script, {l});
    return true;
}

// ============================================================================
// Jobs
// ============================================================================
size_t FirmwarePlannedPath::jobSourceSize() const {
    const size_t prog = job_.program ? job_.program->segments.size() : 0;
    return job_.prefix.size() + (prog > job_.first_index ? prog - job_.first_index : 0);
}

const ToolpathSegment& FirmwarePlannedPath::jobSegment(size_t v) const {
    if (v < job_.prefix.size()) return job_.prefix[v];
    return job_.program->segments[jobProgramIndex(v)];
}

std::array<double, 4> FirmwarePlannedPath::resolveSegment(const ToolpathSegment& s) const {
    auto t = planner::TrajectoryPlanner::transformTarget(s, job_.frame);
    if (job_.laser && job_.laser_lock_z) t[2] = job_.laser_z_plan;
    if (job_.frame.enabled) {
        const auto& off = job_.frame.workspace_offsets[std::min<uint8_t>(job_.frame.plan_workspace, 5)];
        for (size_t a = 0; a < 4; ++a) t[a] += off[a];
    }
    return t;
}

bool FirmwarePlannedPath::startJob(JobRequest job, std::string* error) {
    if (!canStart(error)) return false;
    if (op_ == Op::Script) {
        auto shared = std::make_shared<JobRequest>(std::move(job));
        deferred_ = [this, shared] {
            std::string e;
            if (!startJob(std::move(*shared), &e)) {
                PathEvent ev;
                ev.kind = PathEvent::Kind::JobFinished;
                ev.ok = false;
                ev.text = "Cycle Start rejected: " + e;
                emit(ev);
            }
        };
        return true;
    }
    if (fw_state_ != "Idle") {
        if (error) *error = (fw_state_ == "Alarm") ? "The board is in alarm: unlock or home it first"
                                                   : "The board is not idle (" + fb_.firmware_state + ")";
        return false;
    }
    if (!job.program) {
        if (error) *error = "No program";
        return false;
    }
    if (job.laser && !fw_.laser_mode && fw_.flavour != board::grbl::Flavour::FluidNC) {
        if (error) *error = "Laser jobs need the board's laser mode: set $32 = 1 in the board settings";
        return false;
    }
    job_ = std::move(job);
    // Board THC moves Z on its own: follow the job by the torch's XY position
    track_xy_only_ = job_.plasma && boardRunsThc();
    // Until the first status report the job is at its first segment, not where the previous job ended
    fb_.active_segment = job_.first_index;
    if (job_.first_index < job_.program->segments.size()) fb_.active_line = job_.program->segments[job_.first_index].gcode_line;
    fb_.segment_progress = 0.0f;
    job_cursor_ = 0;
    job_lines_.clear();
    sent_moves_.clear();
    tracker_index_ = 0;
    tracker_sub_ = 0;
    acked_moves_ = 0;
    job_produce_blocked_ = false;
    job_drain_ticket_ = 0;
    job_drain_acked_ = false;
    job_held_ = false;
    job_pos_ = fb_.machine_pos_mm;
    job_outputs_ = 0;
    job_rpm_ = job_.rpm_at_start;
    job_vfd_on_ = job_.spindle_on_at_start;
    job_vfd_ccw_ = job_.spindle_ccw_at_start;
    job_vfd_rpm_ = job_.rpm_at_start;
    job_finish_lines_.clear();
    emitter_.reset();
    {
        // The preamble switches spindle and coolant off: the emitter knows that state
        std::vector<std::string> dummy;
        emitter_.outputs(false, false, -1.0, false, false, false, dummy);
    }
    job_preamble_.clear();
    for (const char* l : {"G21G90G94G17", "G54", "G49", "G92.1", "M5M9"}) {
        ScriptLine s;
        s.text = l;
        job_preamble_.push_back(s);
    }
    ScriptLine p;
    p.text = "$#";
    p.params = true;
    job_preamble_.push_back(p);
    job_preamble_pos_ = 0;
    job_preamble_waiting_ = false;
    job_phase_ = JobPhase::Preamble;
    fb_.job_active = true;
    fb_.progress_is_estimate = !fw_.reports_line_numbers;
    op_ = Op::Job;
    pumpJob(Clock::now());
    return true;
}

void FirmwarePlannedPath::continueJob() {
    if (op_ == Op::Job && job_phase_ == JobPhase::SyncWait) {
        job_produce_blocked_ = false;
        job_drain_ticket_ = 0;
        job_drain_acked_ = false;
        job_phase_ = JobPhase::Streaming;
    }
}

bool FirmwarePlannedPath::emitOutputsFor(const ToolpathSegment& s, std::vector<std::string>& lines, bool* sync_needed) {
    uint8_t mask = s.outputs_mask;
    if (job_.dry_run) mask &= ~OutputBit::OUT1_TorchOrSpindle;
    if (job_.laser) {
        // In laser mode the board switches the beam off during G0 by itself: switching it off and on around every
        // rapid would only add lines
        if ((mask & OutputBit::OUT1_TorchOrSpindle) && job_.laser_air_assist) mask |= OutputBit::OUT3_FloodCoolant;
    }
    if (!job_.mill) mask &= ~OutputBit::OUT2_SpindleDir; // Red pointer: no such output on these boards
    const uint16_t rpm = s.spindle_rpm > 0 ? s.spindle_rpm : job_rpm_;
    const bool spindle_on = (mask & OutputBit::OUT1_TorchOrSpindle) != 0;
    const bool ccw = job_.mill && (mask & OutputBit::OUT2_SpindleDir) != 0;

    if (job_.app_drives_spindle) {
        if (spindle_on != job_vfd_on_ || (spindle_on && (ccw != job_vfd_ccw_ || rpm != job_vfd_rpm_))) {
            *sync_needed = true;
            job_vfd_on_ = spindle_on;
            job_vfd_ccw_ = ccw;
            job_vfd_rpm_ = rpm;
            return false;
        }
    }
    if (mask == job_outputs_ && rpm == job_rpm_) return true;

    double s_value = -1.0;
    if (job_.mill && !job_.app_drives_spindle) {
        s_value = rpm;
    } else if (job_.plasma) {
        // The torch is the spindle output: with S0 a variable-spindle board keeps PWM (and, set that way, enable) off
        s_value = fw_.max_spindle_rpm;
    } else if (job_.laser) {
        double pct = (rpm > 0 && rpm <= 1000) ? rpm / 10.0 : 100.0;
        pct *= job_.laser_power_override_pct / 100.0;
        pct = std::clamp(pct, 0.0, 100.0);
        const double duty = job_.laser_min_duty_pct + pct / 100.0 * (job_.laser_max_duty_pct - job_.laser_min_duty_pct);
        s_value = std::clamp(duty, 0.0, 100.0) / 100.0 * fw_.max_spindle_rpm;
    }
    const bool was_on = (job_outputs_ & OutputBit::OUT1_TorchOrSpindle) != 0;
    const bool board_spindle = spindle_on && !job_.app_drives_spindle;
    emitter_.outputs(board_spindle, job_.laser ? job_.laser_dynamic : ccw, s_value, job_.laser_dynamic,
                     (mask & OutputBit::OUT3_FloodCoolant) != 0, (mask & OutputBit::OUT4_MistCoolant) != 0, lines);
    if (board_spindle && !was_on && job_.mill && job_.spinup_dwell_s > 0.01) {
        emitter_.dwell(job_.spinup_dwell_s, lines);
    }
    for (int i = 0; i < 4; ++i) {
        const uint8_t bit = static_cast<uint8_t>(1U << (kCustomOutputShift + i));
        if ((mask & bit) != (job_outputs_ & bit)) emitter_.auxOutput(i, (mask & bit) != 0, true, lines);
    }
    job_outputs_ = mask;
    job_rpm_ = rpm;
    return true;
}

bool FirmwarePlannedPath::produceJobLines() {
    if (job_cursor_ >= jobSourceSize()) {
        job_produce_blocked_ = true;
        job_sync_segment_ = SIZE_MAX; // The end
        return false;
    }
    const ToolpathSegment& seg = jobSegment(job_cursor_);
    const bool from_program = jobIsProgram(job_cursor_);
    const size_t seg_index = from_program ? jobProgramIndex(job_cursor_) : seg.segment_index;

    if (seg.type == SegmentType::ToolChange || seg.type == SegmentType::ProgramStop) {
        job_produce_blocked_ = true;
        job_sync_kind_ = (seg.type == SegmentType::ToolChange) ? SyncKind::ToolChange : SyncKind::ProgramStop;
        job_sync_segment_ = seg_index;
        ++job_cursor_;
        return false;
    }

    std::vector<std::string> lines;
    bool sync = false;
    if (!emitOutputsFor(seg, lines, &sync)) {
        if (sync) {
            job_produce_blocked_ = true;
            job_sync_kind_ = SyncKind::SpindleChange;
            job_sync_segment_ = seg_index;
            return false;
        }
    }
    for (auto& l : lines) job_lines_.push_back({l, false, {}});
    lines.clear();

    std::vector<std::array<double, 4>> arc_points; // Set for a G2 / G3 line
    double arc_sag = 0.0;
    auto pushMotion = [&](const std::vector<std::string>& ls, const std::array<double, 4>& end, size_t last_index,
                          size_t gline) {
        bool marked = false;
        for (const auto& l : ls) {
            PendingLine pl{l, false, {}};
            if (!marked && l != "G17" && l != "G18" && l != "G19") {
                pl.is_move = true;
                pl.move.segment_index = last_index;
                pl.move.gcode_line = gline;
                pl.move.from_program = from_program;
                pl.move.start = job_pos_;
                pl.move.end = end;
                pl.move.line_number = emitter_.lastLineNumber();
                if (arc_points.size() > 1) {
                    pl.move.arc_points = arc_points;
                    pl.move.arc_sag_mm = arc_sag;
                    pl.move.segment_index = last_index + 2 - arc_points.size(); // First chord
                }
                marked = true;
            }
            job_lines_.push_back(pl);
        }
    };

    if (seg.type == SegmentType::Dwell) {
        double secs = seg.dwell_seconds;
        if (job_.plasma && seg.torch_active && job_.pierce_delay_override_s > 0.01f) secs = job_.pierce_delay_override_s;
        emitter_.dwell(secs, lines);
        for (auto& l : lines) job_lines_.push_back({l, false, {}});
        ++job_cursor_;
        return true;
    }
    if (!isMotion(seg.type)) {
        ++job_cursor_;
        return true;
    }

    const double s_laser = -1.0;
    // Arc: a complete run of chords becomes one G2 / G3
    const auto& prog = *job_.program;
    if (seg.arc_index != kNoArc && from_program && job_.arcs_allowed && emitter_.dialect().arcs &&
        seg.arc_index < prog.arcs.size()) {
        const auto& arc = prog.arcs[seg.arc_index];
        const size_t pi = jobProgramIndex(job_cursor_);
        const size_t last = static_cast<size_t>(arc.first_segment) + arc.segment_count - 1;
        bool whole = pi == arc.first_segment && arc.segment_count > 0 && last < prog.segments.size() &&
                     !(job_.laser && job_.laser_lock_z && arc.plane != 17);
        for (size_t i = pi; whole && i <= last; ++i) {
            if (prog.segments[i].arc_index != seg.arc_index || prog.segments[i].machine_axes_mask != 0) whole = false;
        }
        if (whole) {
            const auto end = resolveSegment(prog.segments[last]);
            std::array<double, 3> ijk{};
            for (size_t a = 0; a < 3; ++a) ijk[a] = arc.center_mm[a] - seg.start_xyza_mm[a];
            // GRBL rejects arcs whose end radius differs (error:33): check it with the rounded numbers sent
            const auto sfw = toFirmware(job_pos_, true);
            const auto efw = toFirmware(end, true);
            const size_t u = (arc.plane == 19) ? 1 : 0;
            const size_t v = (arc.plane == 17) ? 1 : 2;
            auto rnd = [](double x, int dec) { return std::atof(GcodeEmitter::num(x, dec).c_str()); };
            const double su = rnd(sfw[u], 3), sv = rnd(sfw[v], 3);
            const double cu = su + rnd(ijk[u], 4), cv = sv + rnd(ijk[v], 4);
            const double r0 = std::hypot(su - cu, sv - cv);
            const double r1 = std::hypot(rnd(efw[u], 3) - cu, rnd(efw[v], 3) - cv);
            const double tol = std::max(emitter_.dialect().arc_radius_tolerance_mm, 0.0005 * r0);
            if (std::abs(r0 - r1) <= tol && r0 > 0.001) {
                emitter_.arc(arc.clockwise, arc.plane, efw, ijk, prog.segments[last].feed_mm_min, lines, s_laser);
                // Chord vertices for the tracker; the arc bulges at most L^2 / (8 r) from a chord of length L
                arc_points.clear();
                arc_points.push_back(job_pos_);
                double max_l2 = 0.0;
                for (size_t i = pi; i <= last; ++i) {
                    const auto p = (i == last) ? end : resolveSegment(prog.segments[i]);
                    double l2 = 0.0;
                    for (size_t a = 0; a < 4; ++a) l2 += (p[a] - arc_points.back()[a]) * (p[a] - arc_points.back()[a]);
                    max_l2 = std::max(max_l2, l2);
                    arc_points.push_back(p);
                }
                arc_sag = max_l2 / (8.0 * r0);
                pushMotion(lines, end, last, prog.segments[last].gcode_line);
                job_pos_ = end;
                job_cursor_ = jobSourceSize() - (prog.segments.size() - (last + 1));
                return true;
            }
        }
    }

    const auto target = resolveSegment(seg);
    const auto fw = toFirmware(target, true);
    if (seg.type == SegmentType::RapidMove) {
        emitter_.rapid(fw, lines);
    } else {
        emitter_.linear(fw, seg.feed_mm_min, lines, s_laser);
    }
    if (!lines.empty()) pushMotion(lines, target, seg_index, seg.gcode_line);
    job_pos_ = target;
    ++job_cursor_;
    return true;
}

void FirmwarePlannedPath::pumpJob(Clock::time_point now) {
    switch (job_phase_) {
        case JobPhase::Preamble: {
            if (job_preamble_waiting_) return;
            if (job_preamble_pos_ < job_preamble_.size()) {
                if (protocol_.unacknowledged() != 0) return;
                const auto& l = job_preamble_[job_preamble_pos_++];
                job_preamble_ticket_ = protocol_.sendLine(l.text);
                job_preamble_waiting_ = job_preamble_ticket_ != 0;
                return;
            }
            job_phase_ = JobPhase::Streaming;
            [[fallthrough]];
        }
        case JobPhase::Streaming: {
            if (job_held_) return;
            for (int guard = 0; guard < 400; ++guard) {
                if (job_lines_.empty()) {
                    if (job_produce_blocked_) break;
                    if (!produceJobLines() && job_lines_.empty()) {
                        if (job_produce_blocked_) break;
                        continue;
                    }
                    continue;
                }
                auto& front = job_lines_.front();
                if (front.text.size() > emitter_.dialect().max_line_chars) {
                    // The emitter already left out the line number; the board would refuse it (error 11 / 14)
                    log_->add(LogDir::Error, "Line too long for the board: " + front.text);
                    PathEvent e;
                    e.kind = PathEvent::Kind::Message;
                    e.ok = false;
                    std::string where;
                    if (front.is_move) where = " (program line " + std::to_string(front.move.gcode_line + 1) + ")";
                    e.text = "Job stopped: a line is longer than the board accepts (" +
                             std::to_string(front.text.size()) + " of " +
                             std::to_string(emitter_.dialect().max_line_chars) + " characters)" + where;
                    emit(e);
                    job_lines_.clear();
                    stop();
                    return;
                }
                if (!protocol_.canSend(front.text.size())) break;
                const uint32_t ticket = protocol_.sendLine(front.text);
                if (ticket == 0) return; // Link failure is reported by tick()
                if (front.is_move) {
                    front.move.ticket = ticket;
                    sent_moves_.push_back(front.move);
                }
                job_lines_.pop_front();
            }
            if (job_lines_.empty() && job_produce_blocked_) {
                if (!protocol_.canSend(emitter_.dialect().wait_idle.size())) return;
                job_drain_ticket_ = protocol_.sendLine(emitter_.dialect().wait_idle);
                job_drain_acked_ = false;
                status_since_ack_ = false;
                job_phase_ = (job_sync_segment_ == SIZE_MAX) ? JobPhase::Draining : JobPhase::SyncDrain;
            }
            return;
        }
        case JobPhase::Draining:
        case JobPhase::SyncDrain: {
            if (!job_drain_acked_ || !status_since_ack_ || fw_state_ != "Idle") return;
            if (!sent_moves_.empty()) {
                tracker_index_ = sent_moves_.size() - 1;
                const auto& m = sent_moves_.back();
                fb_.active_segment = m.lastSegment();
                fb_.active_line = m.gcode_line;
                fb_.segment_progress = 1.0f;
            }
            if (job_phase_ == JobPhase::Draining) {
                job_finish_lines_.clear();
                job_finish_lines_.push_back(job_.ends_with_program_end ? "M5M9" : "M5");
                job_phase_ = JobPhase::Finishing;
                return;
            }
            PathEvent e;
            e.kind = PathEvent::Kind::JobSyncPoint;
            e.segment_index = job_sync_segment_;
            e.code = static_cast<int>(job_sync_kind_);
            e.spindle_on = job_vfd_on_;
            e.spindle_ccw = job_vfd_ccw_;
            e.spindle_rpm = job_vfd_rpm_;
            emit(e);
            if (job_sync_kind_ == SyncKind::SpindleChange) {
                job_phase_ = JobPhase::SyncWait;
            } else {
                job_phase_ = JobPhase::Done;
                fb_.job_active = false;
                op_ = Op::None;
            }
            return;
        }
        case JobPhase::Finishing: {
            if (protocol_.unacknowledged() != 0) return;
            if (!job_finish_lines_.empty()) {
                protocol_.sendLine(job_finish_lines_.front());
                job_finish_lines_.pop_front();
                return;
            }
            job_phase_ = JobPhase::Done;
            fb_.job_active = false;
            op_ = Op::None;
            if (job_.program) fb_.active_segment = job_.program->segments.size();
            fb_.segment_progress = 1.0f;
            PathEvent e;
            e.kind = PathEvent::Kind::JobFinished;
            e.ok = true;
            emit(e);
            return;
        }
        default:
            (void)now;
            return;
    }
}

void FirmwarePlannedPath::updateTracker() {
    if (sent_moves_.empty()) return;
    while (acked_moves_ < sent_moves_.size() && sent_moves_[acked_moves_].ticket <= last_acked_ticket_) ++acked_moves_;
    const size_t blocks = std::max<size_t>(1, fw_.planner_blocks);
    // Every acknowledged line still holds at most one planner block: anything older has finished
    const size_t lower = acked_moves_ > blocks ? acked_moves_ - blocks : 0;
    const auto& pos = fb_.machine_pos_mm;
    // How far the machine position is off a sent move (beyond its tolerance), where along the move it projects
    // (t, 0..1) and, for an arc, on which chord (sub) and where along that chord (t_sub)
    const size_t axes = track_xy_only_ ? 2 : 4;
    auto onLine = [&](const std::array<double, 4>& s, const std::array<double, 4>& e, double& t) {
        // Following by XY only (board THC moves Z): a move without XY travel (pierce height, retract) is still
        // measured along Z for its progress, and the position is on it while XY stays at its point
        size_t proj_axes = axes;
        if (axes == 2 && std::hypot(e[0] - s[0], e[1] - s[1]) < 1e-6) proj_axes = 4;
        double len2 = 0.0, proj = 0.0;
        for (size_t a = 0; a < proj_axes; ++a) {
            const double d = e[a] - s[a];
            len2 += d * d;
            proj += (pos[a] - s[a]) * d;
        }
        t = len2 > 1e-12 ? std::clamp(proj / len2, 0.0, 1.0) : 1.0;
        double dist2 = 0.0;
        for (size_t a = 0; a < axes; ++a) {
            const double c = s[a] + (e[a] - s[a]) * t - pos[a];
            dist2 += c * c;
        }
        return std::sqrt(dist2);
    };
    auto onMove = [&](const SentMove& mv, double& t, size_t& sub, double& t_sub, size_t from_sub) {
        sub = 0;
        if (mv.arc_points.size() < 2) {
            const double d = onLine(mv.start, mv.end, t);
            t_sub = t;
            return d;
        }
        // The nearest chord at or after the one the machine was on (the machine only goes forward). A full circle
        // starts and ends at the same point: this keeps a finished circle finished and a starting one at its start.
        const size_t n = mv.arc_points.size() - 1;
        double best = 1e300;
        for (size_t k = std::min(from_sub, n - 1); k < n; ++k) {
            double tk = 0.0;
            const double d = onLine(mv.arc_points[k], mv.arc_points[k + 1], tk);
            if (d < best - 1e-9) {
                best = d;
                sub = k;
                t_sub = tk;
            }
        }
        t = (static_cast<double>(sub) + t_sub) / static_cast<double>(n);
        return std::max(0.0, best - mv.arc_sag_mm);
    };
    constexpr double kOnPathMm = 0.05;
    // Moves below the lower bound have certainly finished
    while (tracker_index_ + 1 < sent_moves_.size() && tracker_index_ + 1 < lower) {
        ++tracker_index_;
        tracker_sub_ = 0;
    }
    double t = 0.0, t_sub = 0.0;
    size_t sub = 0;
    const double d_cur = onMove(sent_moves_[tracker_index_], t, sub, t_sub, tracker_sub_);
    if (!(d_cur < kOnPathMm && t < 0.999)) {
        // Not (or no longer) inside the current move: the machine is on the first later move whose path it is on.
        // Status reports come every 100 ms, so short moves can be passed without ever being seen at their end.
        const size_t limit = std::min(sent_moves_.size(), tracker_index_ + 2 * blocks + 2);
        for (size_t i = tracker_index_ + 1; i < limit; ++i) {
            double ti = 0.0, ti_sub = 0.0;
            size_t si = 0;
            if (onMove(sent_moves_[i], ti, si, ti_sub, 0) < kOnPathMm) {
                // At the very end of one move and the start of the next: count it as the next one
                tracker_index_ = i;
                t = ti;
                sub = si;
                t_sub = ti_sub;
                break;
            }
        }
    }
    const auto& m = sent_moves_[tracker_index_];
    tracker_sub_ = sub;
    fb_.active_segment = m.segment_index + sub;
    fb_.active_line = m.gcode_line;
    fb_.segment_progress = static_cast<float>(t_sub);
    fb_.progress_is_estimate = !fw_.reports_line_numbers;
    // Keep the ring short
    while (tracker_index_ > 256) {
        sent_moves_.pop_front();
        --tracker_index_;
        if (acked_moves_ > 0) --acked_moves_;
    }
}

} // namespace rnr::core::motion
