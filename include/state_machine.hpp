#pragma once

#include "basic_controller.hpp"
#include <algorithm>

namespace unitree::common
{
    enum class STATES
    {
        DAMPING = 0,
        SIT = 1,
        STAND = 2,
        CTRL = 3
    };

    class SimpleStateMachine
    {
    public:
        SimpleStateMachine() : state(STATES::DAMPING) {}

        bool Stop()
        {
            state = STATES::DAMPING;
            return true;
        }

        bool Stand()
        {
            state = STATES::STAND;
            standing_count = 0;
            return true;
        }

        bool Ctrl()
        {
            state = STATES::CTRL;
            return true;
        }

        /**
         * @brief 站立状态计算函数
         * @param None
         * @note  线性插值计算关节期望位置, 由阻尼状态逐渐变为站立状态. 目前站立需要2s
        */
        void Standing(BasicUserController& ctrl)
        {
            // 计算插值百分比
            standing_percent = (float)standing_count / standing_duration;
            // 计算期望位置
            for (int i = 0; i < 12; i++)
            {
                ctrl.jpos_des.at(i) = (1 - standing_percent)*ctrl.start_pos.at(i) + standing_percent*ctrl.stand_pos.at(i);
            }
            // 计数器加1
            standing_count++;
            standing_count = standing_count > standing_duration? standing_duration : standing_count;
        }

        STATES state;
    protected:
        int standing_count = 0; // 进入站立状态后开始计数, 用于线性插值
        float standing_percent = 0.0; // 站立百分比, 用于线性插值
        int standing_duration = 100; // 站立消耗timestep数

    };

    class PACTStateMachine: public SimpleStateMachine
    {
    public:
        bool Sit()
        {
            if(state == STATES::DAMPING || state == STATES::STAND)
            {
                state = STATES::SIT;
                sitting_count = 0; // 进入站立状态后重置计数
                return true;
            }
            else
            {
                return false;
            }
        }

        bool Stand()
        {
            state = STATES::STAND;
            standing_count = 0;
            return true;
        }

        /**
         * @brief SIT状态计算函数
         * @param None
         * @note  线性插值计算关节期望位置, 由阻尼状态逐渐变为SIT状态. 目前SIT需要2s
        */
        void Sitting(BasicUserController& ctrl)
        {
            // 计算插值百分比
            sitting_percent = (float)sitting_count / sitting_duration;
            // 计算期望位置
            for (int i = 0; i < 12; i++)
            {
                ctrl.jpos_des.at(i) = (1 - sitting_percent)*ctrl.start_pos.at(i) + sitting_percent*ctrl.sit_pos.at(i);
            }
            // 计数器加1
            sitting_count++;
            sitting_count = sitting_count > sitting_duration? sitting_duration : sitting_count;
        }
    protected:
        int sitting_count = 0;
        float sitting_percent = 0.0;
        int sitting_duration = 200;
    };

    using RLStateMachine = PACTStateMachine;
} // namespace unitree
