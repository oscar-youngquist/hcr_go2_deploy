#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace unitree::common
{
    inline float remapSymmetricAxis(float raw_command, float deadband_region, float max_abs_value)
    {
        const float abs_command = std::fabs(raw_command);
        if (abs_command < deadband_region)
        {
            return 0.0f;
        }

        const float normalized =
            std::clamp((abs_command - deadband_region) / (1.0f - deadband_region), 0.0f, 1.0f);
        return std::copysign(normalized * max_abs_value, raw_command);
    }

    // bytecode mapping for raw joystick data
    // 16b
    typedef union
    {
        struct
        {
            uint8_t R1 : 1;
            uint8_t L1 : 1;
            uint8_t start : 1;
            uint8_t select : 1;
            uint8_t R2 : 1;
            uint8_t L2 : 1;
            uint8_t F1 : 1;
            uint8_t F2 : 1;
            uint8_t A : 1;
            uint8_t B : 1;
            uint8_t X : 1;
            uint8_t Y : 1;
            uint8_t up : 1;
            uint8_t right : 1;
            uint8_t down : 1;
            uint8_t left : 1;
        } components;
        uint16_t value;
    } xKeySwitchUnion;

    // 40 Byte (now used 24B)
    typedef struct
    {
        uint8_t head[2];
        xKeySwitchUnion btn;
        float lx;
        float rx;
        float ry;
        float L2;
        float ly;

        uint8_t idle[16];
    } xRockerBtnDataStruct;

    typedef union
    {
        xRockerBtnDataStruct RF_RX;
        uint8_t buff[40];
    } REMOTE_DATA_RX;

    class Button
    {
    public:
        Button() {}

        void update(bool state)
        {
            on_press = state ? state != pressed : false;
            on_release = state ? false : state != pressed;
            pressed = state;
        }

        bool pressed = false;    // pressed会持续
        bool on_press = false;   // on_press和on_release都是一瞬间
        bool on_release = false;
    };

    class Gamepad
    {
    public:
        Gamepad() {}

        void setCommandLimits(const std::array<float, 3> &limits)
        {
            command_limits = limits;
        }

        void update(xRockerBtnDataStruct &key_data)
        {
            lx = lx * (1 - smooth) + remapSymmetricAxis(key_data.lx, dead_zone, command_limits[1]) * smooth;
            rx = rx * (1 - smooth) + remapSymmetricAxis(key_data.rx, dead_zone, command_limits[2]) * smooth;
            ry = ry * (1 - smooth) + (std::fabs(key_data.ry) < dead_zone ? 0.0 : key_data.ry) * smooth;
            l2 = l2 * (1 - smooth) + (std::fabs(key_data.L2) < dead_zone ? 0.0 : key_data.L2) * smooth;
            ly = ly * (1 - smooth) + remapSymmetricAxis(key_data.ly, dead_zone, command_limits[0]) * smooth;

            if (std::fabs(lx) < dead_zone) lx = 0.0f;
            if (std::fabs(ly) < dead_zone) ly = 0.0f;
            if (std::fabs(rx) < dead_zone) rx = 0.0f;

            R1.update(key_data.btn.components.R1);
            L1.update(key_data.btn.components.L1);
            start.update(key_data.btn.components.start);
            select.update(key_data.btn.components.select);
            R2.update(key_data.btn.components.R2);
            L2.update(key_data.btn.components.L2);
            F1.update(key_data.btn.components.F1);
            F2.update(key_data.btn.components.F2);
            A.update(key_data.btn.components.A);
            B.update(key_data.btn.components.B);
            X.update(key_data.btn.components.X);
            Y.update(key_data.btn.components.Y);
            up.update(key_data.btn.components.up);
            right.update(key_data.btn.components.right);
            down.update(key_data.btn.components.down);
            left.update(key_data.btn.components.left);
        }

        float lx = 0.;
        float rx = 0.;
        float ry = 0.;
        float l2 = 0.;
        float ly = 0.;

        float smooth = 0.03;
        float dead_zone = 0.01;
        std::array<float, 3> command_limits = {1.0f, 0.5f, 1.0f};

        Button R1;
        Button L1;
        Button start;
        Button select;
        Button R2;
        Button L2;
        Button F1;
        Button F2;
        Button A;
        Button B;
        Button X;
        Button Y;
        Button up;
        Button right;
        Button down;
        Button left;
    };
} // namespace unitree::common
