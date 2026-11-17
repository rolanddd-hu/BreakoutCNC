// FirmwarePlannedPath: Connection, handshake, status and acknowledgements (the class is split by topic; see firmware_planned_path.h)
#include "core/motion/firmware_planned_path_internal.h"

namespace rnr::core::motion {

using namespace fpp_detail;

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
        const std::string open_error = protocol_.openError();
        close();
        if (was_ready) {
            fault("Connection to the board lost (cable unplugged or board switched off)", 0, true);
            fb_.status_text = "Connection lost";
        } else {
            PathEvent e;
            e.kind = PathEvent::Kind::ConnectFailed;
            e.ok = false;
            e.text = open_error.empty() ? "Connection to " + connectionLabel() + " failed" : "Cannot connect: " + open_error;
            fb_.status_text = e.text;
            emit(e);
        }
        return;
    }
    if (protocol_.connecting()) {
        // The port / socket is still being opened: the welcome wait starts once it is open
        phase_deadline_ = now + std::chrono::milliseconds(std::clamp<long>(profile_.getInt("connect_wait_ms", 800), 100, 10000));
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

} // namespace rnr::core::motion
