#include "HardwareBridge.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include "conversion.hpp"
#include "pact_controller.hpp"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

namespace
{
constexpr const char *kLowCmdTopic = "rt/lowcmd";
constexpr const char *kLowStateTopic = "rt/lowstate";
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

    if (deactivate_motion_service)
    {
        InitRobotStateClient();
        int service_status = 0;
        std::cout << "Try to deactivate the service: mcf" << std::endl;
        while (!service_status)
        {
            robot_state_client.ServiceSwitch("mcf", 0, service_status);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Deactivate the service: mcf" << std::endl;
    }

    InitDdsModel();
    StartControl();
}

void HardwareBridge::InitRobotStateClient()
{
    robot_state_client.SetTimeout(10.0f);
    robot_state_client.Init();
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

    while (true) std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void HardwareBridge::LowStateMessageHandler(const void *message)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    state = *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
    robot_interface.SetState(state);
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
    lowcmd_publisher->Write(cmd);
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
