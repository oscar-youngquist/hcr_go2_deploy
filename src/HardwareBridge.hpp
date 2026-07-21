#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include "unitree/common/thread/thread.hpp"
#include "unitree/idl/go2/LowCmd_.hpp"
#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/robot/b2/motion_switcher/motion_switcher_client.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

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
    void ReleaseMotionMode();
    void StartControl();
    void StartSendCmd();
    void LowStateMessageHandler(const void *message);
    void LowCmdWriteHandler();
    void PrintStatus();
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
    unitree::common::ThreadPtr low_cmd_write_thread, control_thread, status_thread;
    // LowCmd CRC is calculated over the object's raw storage, including
    // padding. Value-initialize it so those bytes are deterministic, matching
    // the Unitree SDK2 low-level examples and the robot's CRC calculation.
    unitree_go::msg::dds_::LowCmd_ cmd{};
    unitree_go::msg::dds_::LowState_ state{};

    unitree::common::Gamepad gamepad;
    unitree::common::REMOTE_DATA_RX remote{};
    unitree::common::PACTStateMachine state_machine;
    unitree::common::BasicUserController *ctrl = nullptr;
    unitree::common::BasicRobotInterface robot_interface;
    unitree::robot::b2::MotionSwitcherClient motion_switcher_client;

    std::mutex state_mutex, cmd_mutex;
    std::ofstream log_file;
    bool deactivate_motion_service;

    std::atomic<uint64_t> lowstate_count{0};
    std::atomic<uint64_t> lowcmd_attempt_count{0};
    std::atomic<uint64_t> lowcmd_success_count{0};
    std::atomic<float> motor0_position{0.0f};
    std::atomic<float> desired_motor0_position{0.0f};
    std::atomic<float> desired_motor0_kp{0.0f};
    std::atomic<unsigned int> motor0_mode{0};
    std::atomic<unsigned int> reported_state{0};
    std::atomic<unsigned int> reported_buttons{0};
    uint64_t previous_lowstate_count = 0;
    uint64_t previous_lowcmd_attempt_count = 0;
    uint64_t previous_lowcmd_success_count = 0;
};
