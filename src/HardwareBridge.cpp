#include "HardwareBridge.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "conversion.hpp"
#include "pact_controller.hpp"

using namespace unitree::common;
using namespace unitree::robot;

namespace
{
constexpr const char *kLowCmdTopic = "rt/lowcmd";
constexpr const char *kLowStateTopic = "rt/lowstate";

int64_t SteadyTimeNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

template <size_t N>
void AppendArrayHeaders(std::ostream &stream, const std::string &name)
{
    for (size_t i = 0; i < N; ++i) stream << ',' << name << '_' << i;
}

template <size_t N>
void AppendArrayValues(std::ostream &stream, const std::array<float, N> &values)
{
    for (float value : values) stream << ',' << value;
}

const char *StateName(unsigned int state)
{
    switch (static_cast<STATES>(state))
    {
    case STATES::DAMPING: return "DAMPING";
    case STATES::SIT: return "SIT";
    case STATES::STAND: return "STAND";
    case STATES::CTRL: return "CTRL";
    }
    return "UNKNOWN";
}
}

HardwareBridge::HardwareBridge(const std::string &config_file_path,
                               const std::filesystem::path &log_file_path,
                               bool deactivate_service)
    : ctrl(new PACTController(config_file_path)),
      log_file_path(log_file_path),
      config_file_path(config_file_path),
      deactivate_motion_service(deactivate_service)
{
    const PACTRLCfg cfg(config_file_path);
    if (cfg.command_limit_max_abs.size() != 3)
        throw std::runtime_error("command_limit_max_abs must contain [linear_x, linear_y, angular_z]");
    if (std::any_of(cfg.command_limit_max_abs.begin(), cfg.command_limit_max_abs.end(),
                    [](float limit) { return limit < 0.0f; }))
        throw std::runtime_error("command_limit_max_abs values must be non-negative");
    gamepad.setCommandLimits({cfg.command_limit_max_abs[0],
                              cfg.command_limit_max_abs[1],
                              cfg.command_limit_max_abs[2]});
    if (cfg.lowstate_timeout_ms <= 0 || cfg.log_flush_count <= 0 || cfg.log_loop_dt <= 0.0f)
        throw std::runtime_error("watchdog and logging intervals/counts must be positive");
    lowstate_timeout_ns = static_cast<int64_t>(cfg.lowstate_timeout_ms) * 1'000'000;
    log_flush_count = static_cast<size_t>(cfg.log_flush_count);
    log_loop_period_us = static_cast<uint64_t>(cfg.log_loop_dt * 1'000'000.0f);
    log_buffer.reserve(log_flush_count);
    if (cfg.use_kalman_filter)
        torso_estimator = std::make_unique<LinearKFObserver>(config_file_path, cfg.dt);
}

HardwareBridge::~HardwareBridge()
{
    delete ctrl;
}

void HardwareBridge::run()
{
    ctrl->loadParam();
    ctrl->loadPolicy();
    InitDdsModel();
    logging_thread = CreateRecurrentThreadEx(
        "logging", UT_CPU_ID_NONE, log_loop_period_us, &HardwareBridge::LoggingLoop, this);

    if (deactivate_motion_service)
    {
        ReleaseMotionMode();
    }

    StartControl();
}

void HardwareBridge::ReleaseMotionMode()
{
    motion_switcher_client.SetTimeout(10.0f);
    motion_switcher_client.Init();

    constexpr int kMaximumReleaseAttempts = 10;
    for (int attempt = 0; attempt <= kMaximumReleaseAttempts; ++attempt)
    {
        std::string robot_form;
        std::string motion_name;
        const int32_t check_result =
            motion_switcher_client.CheckMode(robot_form, motion_name);
        if (check_result != 0)
        {
            throw std::runtime_error(
                "MotionSwitcher CheckMode failed with error " +
                std::to_string(check_result) +
                "; refusing to start low-level command output");
        }

        if (motion_name.empty())
        {
            std::cout << "Low-level control handoff complete: no high-level motion mode is active."
                      << std::endl;
            return;
        }

        if (attempt == kMaximumReleaseAttempts)
        {
            throw std::runtime_error(
                "High-level motion mode '" + motion_name +
                "' remained active after " + std::to_string(kMaximumReleaseAttempts) +
                " release attempts; refusing to publish motor commands");
        }

        std::cout << "Releasing high-level motion mode: form='" << robot_form
                  << "' mode='" << motion_name << "' (attempt " << (attempt + 1)
                  << '/' << kMaximumReleaseAttempts << ")" << std::endl;
        const int32_t release_result = motion_switcher_client.ReleaseMode();
        if (release_result != 0)
        {
            throw std::runtime_error(
                "MotionSwitcher ReleaseMode failed with error " +
                std::to_string(release_result) +
                "; refusing to publish motor commands");
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void HardwareBridge::InitDdsModel()
{
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(kLowCmdTopic));
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(kLowStateTopic));
    lowcmd_publisher->InitChannel();
    lowstate_subscriber->InitChannel(
        std::bind(&HardwareBridge::LowStateMessageHandler, this, std::placeholders::_1), 1);
}

void HardwareBridge::StartControl()
{
    InitLowCmd();
    std::cout << "Press START button to start!" << std::endl;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        IntegrateGamepad();
        if (gamepad.start.on_press) break;
    }

    std::cout << "Start!" << std::endl;
    Damping();
    const auto control_period_us = static_cast<uint64_t>(ctrl->dt * 1'000'000.0f);
    control_thread = CreateRecurrentThreadEx(
        "ctrl", UT_CPU_ID_NONE, control_period_us, &HardwareBridge::RobotControl, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    StartSendCmd();
    std::cout << "Start Send Cmd!" << std::endl;
    
    status_thread = CreateRecurrentThreadEx(
        "status", UT_CPU_ID_NONE, 1'000'000, &HardwareBridge::PrintStatus, this);

    while (true) std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void HardwareBridge::LowStateMessageHandler(const void *message)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    state = *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
    robot_interface.SetState(state);
    last_lowstate_time_ns.store(SteadyTimeNs(), std::memory_order_release);
    motor0_position.store(state.motor_state()[0].q(), std::memory_order_relaxed);
    motor0_mode.store(state.motor_state()[0].mode(), std::memory_order_relaxed);
    lowstate_count.fetch_add(1, std::memory_order_relaxed);
}

void HardwareBridge::IntegrateGamepad()
{
    std::lock_guard<std::mutex> lock(state_mutex);
    std::memcpy(remote.buff, state.wireless_remote().data(), sizeof(remote.buff));
    gamepad.update(remote.RF_RX);
}

void HardwareBridge::LowCmdWriteHandler()
{
    std::lock_guard<std::mutex> lock(cmd_mutex);
    if (!IsLowStateFresh())
    {
        ApplyDampingCommand();
        watchdog_active.store(true, std::memory_order_relaxed);
    }
    const bool succeeded = lowcmd_publisher->Write(cmd);
    lowcmd_attempt_count.fetch_add(1, std::memory_order_relaxed);
    if (succeeded) lowcmd_success_count.fetch_add(1, std::memory_order_relaxed);
}

void HardwareBridge::PrintStatus()
{
    const uint64_t states = lowstate_count.load(std::memory_order_relaxed);
    const uint64_t attempts = lowcmd_attempt_count.load(std::memory_order_relaxed);
    const uint64_t successes = lowcmd_success_count.load(std::memory_order_relaxed);
    const uint64_t state_rate = states - previous_lowstate_count;
    const uint64_t attempt_rate = attempts - previous_lowcmd_attempt_count;
    const uint64_t success_rate = successes - previous_lowcmd_success_count;
    previous_lowstate_count = states;
    previous_lowcmd_attempt_count = attempts;
    previous_lowcmd_success_count = successes;

    const unsigned int buttons = reported_buttons.exchange(0, std::memory_order_relaxed);
    std::ostringstream button_text;
    if (buttons & (1U << 0)) button_text << " L1";
    if (buttons & (1U << 1)) button_text << " R1";
    if (buttons & (1U << 2)) button_text << " R2";
    if (buttons & (1U << 3)) button_text << " A";
    if (buttons & (1U << 4)) button_text << " B";
    if (button_text.str().empty()) button_text << " none";

    std::cout << "[status] state="
              << StateName(reported_state.load(std::memory_order_relaxed))
              << " lowstate_rx=" << state_rate << "/s"
              << " lowcmd_tx_ok=" << success_rate << '/' << attempt_rate << "/s"
              << " tx_fail_total=" << (attempts - successes)
              << " motor0_mode=" << motor0_mode.load(std::memory_order_relaxed)
              << " motor0_q=" << std::fixed << std::setprecision(4)
              << motor0_position.load(std::memory_order_relaxed)
              << " motor0_q_des="
              << desired_motor0_position.load(std::memory_order_relaxed)
              << " motor0_kp=" << desired_motor0_kp.load(std::memory_order_relaxed)
              << " watchdog=" << (watchdog_active.load(std::memory_order_relaxed) ? "ACTIVE" : "ok")
              << " button_events:" << button_text.str()
              << std::endl;
}

void HardwareBridge::StartSendCmd()
{
    low_cmd_write_thread = CreateRecurrentThreadEx(
        "writebasiccmd", UT_CPU_ID_NONE, 2000, &HardwareBridge::LowCmdWriteHandler, this);
}

void HardwareBridge::InitLowCmd()
{
    cmd.head()[0] = 0xFE;
    cmd.head()[1] = 0xEF;
    cmd.level_flag() = 0xFF;
    cmd.gpio() = 0;
    for (auto &motor : cmd.motor_cmd())
    {
        motor.mode() = 0x01;
        motor.q() = PosStopF;
        motor.kp() = 0.0f;
        motor.dq() = VelStopF;
        motor.kd() = 0.0f;
        motor.tau() = 0.0f;
    }
}

void HardwareBridge::SetCmd()
{
    for (size_t i = 0; i < 12; ++i)
    {
        auto &motor = cmd.motor_cmd()[i];
        motor.q() = robot_interface.jpos_des[i];
        motor.dq() = robot_interface.jvel_des[i];
        motor.kp() = robot_interface.kp[i];
        motor.kd() = robot_interface.kd[i];
        motor.tau() = robot_interface.tau_ff[i];
    }
    desired_motor0_position.store(cmd.motor_cmd()[0].q(), std::memory_order_relaxed);
    desired_motor0_kp.store(cmd.motor_cmd()[0].kp(), std::memory_order_relaxed);
    cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&cmd),
                           (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
}

void HardwareBridge::UpdateStateMachine()
{
    if (gamepad.R1.on_press && gamepad.L1.pressed && state_machine.Sit()) SitCallback();
    if (gamepad.R2.on_press && gamepad.L1.pressed && state_machine.Stand()) StandCallback();
    if (gamepad.A.on_press && gamepad.L1.pressed && state_machine.Ctrl()) CtrlCallback();
    if (gamepad.B.on_press && gamepad.L1.pressed) state_machine.Stop();
}

void HardwareBridge::RobotControl()
{
    if (!IsLowStateFresh())
    {
        watchdog_active.store(true, std::memory_order_relaxed);
        state_machine.Stop();
        Damping();
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            SetCmd();
        }
        LogCurrentRobotInterface();
        ++control_iteration;
        return;
    }
    watchdog_active.store(false, std::memory_order_relaxed);
    IntegrateGamepad();
    HandleLoggingButtons();
    UpdateStateMachine();
    reported_state.store(static_cast<unsigned int>(state_machine.state),
                         std::memory_order_relaxed);
    unsigned int buttons = 0;
    if (gamepad.L1.on_press) buttons |= 1U << 0;
    if (gamepad.R1.on_press) buttons |= 1U << 1;
    if (gamepad.R2.on_press) buttons |= 1U << 2;
    if (gamepad.A.on_press) buttons |= 1U << 3;
    if (gamepad.B.on_press) buttons |= 1U << 4;
    if (buttons != 0) reported_buttons.fetch_or(buttons, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        if (torso_estimator)
        {
            torso_estimator->Update(state);
            robot_interface.SetTorsoEstimate(torso_estimator->GetEstimatedPosition(),
                                             torso_estimator->GetEstimatedVelocityWorld(),
                                             torso_estimator->GetEstimatedVelocityBody());
        }
        ctrl->GetInput(robot_interface, gamepad);
    }

    switch (state_machine.state)
    {
    case STATES::SIT: Sitting(); break;
    case STATES::STAND: Standing(); break;
    case STATES::DAMPING: Damping(); break;
    case STATES::CTRL: UserControlStep(true); break;
    }

    {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        SetCmd();
    }
    LogCurrentRobotInterface();
    ++control_iteration;
}

void HardwareBridge::SitCallback()
{
    std::lock_guard<std::mutex> lock(state_mutex);
    ctrl->save_jpos(robot_interface);
    robot_interface.jpos_des = ctrl->start_pos;
    robot_interface.jvel_des.fill(0.0f);
    robot_interface.kp.fill(ctrl->stand_kp);
    robot_interface.kd.fill(ctrl->stand_kd);
    robot_interface.tau_ff.fill(0.0f);
}

void HardwareBridge::StandCallback()
{
    std::lock_guard<std::mutex> lock(state_mutex);
    ctrl->save_jpos(robot_interface);
    robot_interface.jpos_des = ctrl->start_pos;
    robot_interface.jvel_des.fill(0.0f);
    robot_interface.kp.fill(ctrl->stand_kp);
    robot_interface.kd.fill(ctrl->stand_kd);
    robot_interface.tau_ff.fill(0.0f);
}

void HardwareBridge::CtrlCallback()
{
    ctrl->reset(robot_interface, gamepad);
    robot_interface.jpos_des = ctrl->stand_pos;
    robot_interface.jvel_des.fill(0.0f);
    robot_interface.kp.fill(ctrl->ctrl_kp);
    robot_interface.kd.fill(ctrl->ctrl_kd);
    robot_interface.tau_ff.fill(0.0f);
}

void HardwareBridge::Damping(float kd)
{
    robot_interface.jpos_des.fill(0.0f);
    robot_interface.jvel_des.fill(0.0f);
    robot_interface.kp.fill(0.0f);
    robot_interface.kd.fill(kd);
    robot_interface.tau_ff.fill(0.0f);
}

void HardwareBridge::Sitting()
{
    state_machine.Sitting(*ctrl);
    robot_interface.jpos_des = ctrl->jpos_des;
}

void HardwareBridge::Standing()
{
    ctrl->DummyCalculate();
    state_machine.Standing(*ctrl);
    robot_interface.jpos_des = ctrl->jpos_des;
}

void HardwareBridge::UserControlStep(bool send_cmd)
{
    ctrl->Calculate();
    if (send_cmd)
    {
        robot_interface.jpos_des = ctrl->jpos_des;
        robot_interface.jvel_des = ctrl->jvel_des;
        robot_interface.tau_ff = ctrl->jtau_des;
    }
}

bool HardwareBridge::IsLowStateFresh() const
{
    const int64_t last = last_lowstate_time_ns.load(std::memory_order_acquire);
    return last != 0 && SteadyTimeNs() - last <= lowstate_timeout_ns;
}

void HardwareBridge::ApplyDampingCommand(float kd)
{
    for (size_t i = 0; i < 12; ++i)
    {
        auto &motor = cmd.motor_cmd()[i];
        motor.q() = 0.0f;
        motor.dq() = 0.0f;
        motor.kp() = 0.0f;
        motor.kd() = kd;
        motor.tau() = 0.0f;
    }
    desired_motor0_position.store(0.0f, std::memory_order_relaxed);
    desired_motor0_kp.store(0.0f, std::memory_order_relaxed);
    cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&cmd),
                           (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
}

void HardwareBridge::HandleLoggingButtons()
{
    if (gamepad.left.on_press) StartLogging();
    if (gamepad.right.on_press) StopLogging();
}

void HardwareBridge::StartLogging()
{
    if (logging_enabled.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> lock(log_mutex);
    log_buffer.clear();
    if (log_file.is_open()) log_file.close();
    std::filesystem::create_directories(log_file_path.parent_path());
    log_file.open(log_file_path);
    if (!log_file)
    {
        std::cerr << "Failed to open log file: " << log_file_path << std::endl;
        return;
    }
    WriteLogHeader();
    log_time_zero = std::chrono::steady_clock::now();
    logging_enabled.store(true, std::memory_order_release);
    std::cout << "Started logging: " << log_file_path << std::endl;
}

void HardwareBridge::StopLogging()
{
    if (!logging_enabled.exchange(false, std::memory_order_acq_rel)) return;
    std::lock_guard<std::mutex> lock(log_mutex);
    WriteBufferedLogEntries(log_buffer);
    log_buffer.clear();
    log_file.flush();
    log_file.close();
    std::cout << "Stopped logging: " << log_file_path << std::endl;
}

void HardwareBridge::LogCurrentRobotInterface()
{
    if (!logging_enabled.load(std::memory_order_acquire)) return;
    RobotLogEntry entry;
    entry.iteration = control_iteration;
    entry.time_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - log_time_zero).count();
    entry.state = state_machine.state;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        entry.robot = robot_interface;
    }
    std::lock_guard<std::mutex> lock(log_mutex);
    if (logging_enabled.load(std::memory_order_relaxed)) log_buffer.push_back(entry);
}

void HardwareBridge::LoggingLoop()
{
    std::lock_guard<std::mutex> lock(log_mutex);
    if (!log_file.is_open() || log_buffer.size() < log_flush_count) return;
    std::vector<RobotLogEntry> entries;
    entries.swap(log_buffer);
    WriteBufferedLogEntries(entries);
    log_file.flush();
}

void HardwareBridge::WriteLogHeader()
{
    log_file << "iteration,time_seconds,state";
    AppendArrayHeaders<12>(log_file, "jpos");
    AppendArrayHeaders<12>(log_file, "jvel");
    AppendArrayHeaders<12>(log_file, "tau");
    AppendArrayHeaders<4>(log_file, "quat");
    AppendArrayHeaders<3>(log_file, "rpy");
    AppendArrayHeaders<3>(log_file, "gyro");
    AppendArrayHeaders<3>(log_file, "projected_gravity");
    AppendArrayHeaders<3>(log_file, "acc");
    AppendArrayHeaders<3>(log_file, "torso_pos_est");
    AppendArrayHeaders<3>(log_file, "torso_vel_world_est");
    AppendArrayHeaders<3>(log_file, "torso_vel_body_est");
    AppendArrayHeaders<3>(log_file, "cmd");
    AppendArrayHeaders<12>(log_file, "jpos_des");
    AppendArrayHeaders<12>(log_file, "jvel_des");
    AppendArrayHeaders<12>(log_file, "kp");
    AppendArrayHeaders<12>(log_file, "kd");
    AppendArrayHeaders<12>(log_file, "tau_ff");
    log_file << '\n';
}

void HardwareBridge::WriteBufferedLogEntries(const std::vector<RobotLogEntry> &entries)
{
    if (!log_file.is_open()) return;
    for (const auto &entry : entries)
    {
        const auto &robot = entry.robot;
        log_file << entry.iteration << ',' << entry.time_seconds << ',' << static_cast<int>(entry.state);
        AppendArrayValues(log_file, robot.jpos); AppendArrayValues(log_file, robot.jvel);
        AppendArrayValues(log_file, robot.tau); AppendArrayValues(log_file, robot.quat);
        AppendArrayValues(log_file, robot.rpy); AppendArrayValues(log_file, robot.gyro);
        AppendArrayValues(log_file, robot.projected_gravity); AppendArrayValues(log_file, robot.acc);
        AppendArrayValues(log_file, robot.torso_pos_est); AppendArrayValues(log_file, robot.torso_vel_world_est);
        AppendArrayValues(log_file, robot.torso_vel_body_est); AppendArrayValues(log_file, robot.cmd);
        AppendArrayValues(log_file, robot.jpos_des); AppendArrayValues(log_file, robot.jvel_des);
        AppendArrayValues(log_file, robot.kp); AppendArrayValues(log_file, robot.kd);
        AppendArrayValues(log_file, robot.tau_ff); log_file << '\n';
    }
}
