#include "HardwareBridge.hpp"

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
      log_file(log_file_path),
      deactivate_motion_service(deactivate_service)
{
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
    IntegrateGamepad();
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
    WriteLog();
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

void HardwareBridge::WriteLog()
{
    if (!log_file.is_open()) return;
    log_file << static_cast<size_t>(state_machine.state);
    for (float value : ctrl->GetLog()) log_file << ',' << value;
    log_file << '\n';
}
