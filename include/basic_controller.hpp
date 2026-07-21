#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "gamepad.hpp"
#include "robot_interface.hpp"

namespace unitree::common
{
class BasicUserController
{
public:
    virtual ~BasicUserController() = default;
    virtual void loadParam() = 0;
    virtual void loadPolicy() = 0;
    virtual void reset(BasicRobotInterface &, Gamepad &) = 0;
    virtual void GetInput(BasicRobotInterface &, Gamepad &) = 0;
    virtual void DummyCalculate() = 0;
    virtual void Calculate() = 0;
    virtual std::vector<float> GetLog() = 0;

    void save_jpos(BasicRobotInterface &robot)
    {
        std::copy(robot.jpos.begin(), robot.jpos.end(), start_pos.begin());
    }

    float dt = 0.0f;
    float stand_kp = 0.0f, stand_kd = 0.0f;
    float ctrl_kp = 0.0f, ctrl_kd = 0.0f;
    std::array<float, 12> start_pos{}, stand_pos{}, sit_pos{};
    std::array<float, 12> jpos_des{}, jvel_des{}, jtau_des{};
    std::array<float, 12> jpos_lim_min{}, jpos_lim_max{}, jtau_lim{};
};
} // namespace unitree::common
