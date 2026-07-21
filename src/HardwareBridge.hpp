#pragma once

#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include "unitree/common/thread/thread.hpp"
#include "unitree/idl/go2/LowCmd_.hpp"
#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/go2/robot_state/robot_state_client.hpp"

#include "basic_controller.hpp"
#include "gamepad.hpp"
#include "robot_interface.hpp"
#include "state_machine.hpp"

class HardwareBridge
{
public:
    HardwareBridge(const std::string &config_file_path,
                   const std::filesystem::path &log_file_path,
                   bool deactivate_motion_service);
    ~HardwareBridge();

    void run();

private:
    void InitDdsModel();
    void InitRobotStateClient();
    void StartControl();
    void StartSendCmd();
    void LowStateMessageHandler(const void *message);
    void LowCmdWriteHandler();
    void RobotControl();
    void IntegrateGamepad();
    void UpdateStateMachine();
    void InitLowCmd();
    void SetCmd();
    void SitCallback();
    void StandCallback();
    void CtrlCallback();
    void Sitting();
    void Standing();
    void Damping(float kd = 3.0f);
    void UserControlStep(bool send_cmd = false);
    void WriteLog();

    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    unitree::common::ThreadPtr low_cmd_write_thread, control_thread;
    unitree_go::msg::dds_::LowCmd_ cmd;
    unitree_go::msg::dds_::LowState_ state;

    unitree::common::Gamepad gamepad;
    unitree::common::REMOTE_DATA_RX remote{};
    unitree::common::PACTStateMachine state_machine;
    unitree::common::BasicUserController *ctrl = nullptr;
    unitree::common::BasicRobotInterface robot_interface;
    unitree::robot::go2::RobotStateClient robot_state_client;

    std::mutex state_mutex, cmd_mutex;
    std::ofstream log_file;
    bool deactivate_motion_service;
};
