#pragma once

#include <algorithm>
#include <array>
#include <deque>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <torch/script.h>
#include <torch/torch.h>

#include "basic_controller.hpp"
#include "param/cfg.hpp"

namespace unitree::common
{
class PACTController : public BasicUserController
{
public:
    explicit PACTController(const std::string &cfg_file) : config_file_name(cfg_file)
    {
        projected_gravity[2] = -1.0f;
    }

    void loadParam() override
    {
        const PACTRLCfg cfg(config_file_name);
        if (cfg.stand_pos.size() != 12 || cfg.sit_pos.size() != 12 ||
            cfg.pos_limit_min.size() != 12 || cfg.pos_limit_max.size() != 12 ||
            cfg.tau_limit.size() != 12)
        {
            throw std::runtime_error("PACT configuration arrays must each contain 12 values");
        }

        dt = cfg.dt;
        stand_kp = cfg.stand_kp;
        stand_kd = cfg.stand_kd;
        ctrl_kp = cfg.ctrl_kp;
        ctrl_kd = cfg.ctrl_kd;
        pos_action_scale = cfg.pos_action_scale;
        tau_action_scale = cfg.tau_action_scale;
        lin_vel_scale = cfg.lin_vel_scale;
        ang_vel_scale = cfg.ang_vel_scale;
        dof_pos_scale = cfg.dof_pos_scale;
        dof_vel_scale = cfg.dof_vel_scale;
        obs_clip = cfg.clip_obs;
        act_clip = cfg.clip_actions;
        policy_name = cfg.policy_name;
        frame_stack = cfg.frame_stack;
        num_single_obs = cfg.num_single_obs;

        std::copy_n(cfg.stand_pos.begin(), 12, stand_pos.begin());
        std::copy_n(cfg.sit_pos.begin(), 12, sit_pos.begin());
        std::copy_n(cfg.pos_limit_min.begin(), 12, jpos_lim_min.begin());
        std::copy_n(cfg.pos_limit_max.begin(), 12, jpos_lim_max.begin());
        std::copy_n(cfg.tau_limit.begin(), 12, jtau_lim.begin());

        single_step_obs.assign(num_single_obs, 0.0f);
        last_obs.assign(num_single_obs, 0.0f);
        history_obs.assign(frame_stack, single_step_obs);
    }

    void loadPolicy() override
    {
        const auto package_root = std::filesystem::absolute(config_file_name).parent_path().parent_path();
        const auto model_path = package_root / "models" / policy_name;
        policy = torch::jit::load(model_path.string());
        policy.eval();
        std::cout << "Load policy from: " << model_path << std::endl;
    }

    void reset(BasicRobotInterface &, Gamepad &) override
    {
        std::fill(single_step_obs.begin(), single_step_obs.end(), 0.0f);
        std::fill(last_obs.begin(), last_obs.end(), 0.0f);
        history_obs.assign(frame_stack, single_step_obs);
        pos_actions.fill(0.0f);
        tau_actions.fill(0.0f);
    }

    void GetInput(BasicRobotInterface &robot, Gamepad &gamepad) override
    {
        base_ang_vel = robot.gyro;
        projected_gravity = robot.projected_gravity;
        cmd = {gamepad.ly, -gamepad.lx, -gamepad.rx};
        for (size_t i = 0; i < 12; ++i)
        {
            jpos_processed[i] = robot.jpos[i] - stand_pos[i];
            jvel[i] = robot.jvel[i];
        }
    }

    void DummyCalculate() override
    {
        torch::InferenceMode guard;
        const auto zero_obs = torch::zeros({1, num_single_obs});
        const auto zero_history = torch::zeros({1, frame_stack * num_single_obs});
        std::vector<torch::jit::IValue> input{zero_obs, zero_history};
        policy.get_method("act_inference")(input);
    }

    void Calculate() override
    {
        torch::InferenceMode guard;
        fill_single_step_obs();
        history_obs.pop_front();
        history_obs.push_back(single_step_obs);

        auto obs = torch::from_blob(single_step_obs.data(), {1, num_single_obs}, torch::kFloat32).clone();
        auto history = torch::zeros({1, frame_stack * num_single_obs});
        auto history_accessor = history.accessor<float, 2>();
        for (int i = 0; i < frame_stack; ++i)
            for (int j = 0; j < num_single_obs; ++j)
                history_accessor[0][i * num_single_obs + j] = history_obs[i][j];

        std::vector<torch::jit::IValue> input{obs, history};
        const auto output = policy.get_method("act_inference")(input).toTensor();
        if (output.numel() < 24)
            throw std::runtime_error("PACT policy output must contain at least 24 actions");

        for (size_t i = 0; i < 12; ++i)
        {
            pos_actions[i] = clip(output[0][i].item<float>(), -act_clip, act_clip);
            tau_actions[i] = clip(output[0][i + 12].item<float>(), -act_clip, act_clip);
            jpos_des[i] = clip(pos_actions[i] * pos_action_scale + stand_pos[i],
                               jpos_lim_min[i], jpos_lim_max[i]);
            jtau_des[i] = clip(tau_actions[i] * tau_action_scale, -jtau_lim[i], jtau_lim[i]);
        }
        jvel_des.fill(0.0f);
    }

    std::vector<float> GetLog() override { return {cmd[0], cmd[1], cmd[2]}; }

private:
    static float clip(float value, float low, float high)
    {
        return std::clamp(value, low, high);
    }

    void fill_single_step_obs()
    {
        if (num_single_obs < 57)
            throw std::runtime_error("PACT num_single_obs must be at least 57");
        last_obs = single_step_obs;
        single_step_obs[0] = clip(cmd[0] * lin_vel_scale, -obs_clip, obs_clip);
        single_step_obs[1] = clip(cmd[1] * lin_vel_scale, -obs_clip, obs_clip);
        single_step_obs[2] = clip(cmd[2] * ang_vel_scale, -obs_clip, obs_clip);
        for (size_t i = 0; i < 3; ++i)
        {
            single_step_obs[i + 3] = clip(projected_gravity[i], -obs_clip, obs_clip);
            single_step_obs[i + 6] = clip(base_ang_vel[i] * ang_vel_scale, -obs_clip, obs_clip);
        }
        for (size_t i = 0; i < 12; ++i)
        {
            single_step_obs[i + 9] = clip(jpos_processed[i] * dof_pos_scale, -obs_clip, obs_clip);
            single_step_obs[i + 21] = clip(jvel[i] * dof_vel_scale, -obs_clip, obs_clip);
            single_step_obs[i + 33] = clip(pos_actions[i], -obs_clip, obs_clip);
            single_step_obs[i + 45] = clip(tau_actions[i], -obs_clip, obs_clip);
        }
    }

    std::string config_file_name, policy_name;
    std::array<float, 3> base_ang_vel{}, projected_gravity{}, cmd{};
    std::array<float, 12> jpos_processed{}, jvel{}, pos_actions{}, tau_actions{};
    int frame_stack = 0, num_single_obs = 0;
    std::vector<float> single_step_obs, last_obs;
    std::deque<std::vector<float>> history_obs;
    float lin_vel_scale = 0.0f, ang_vel_scale = 0.0f;
    float dof_pos_scale = 0.0f, dof_vel_scale = 0.0f;
    float pos_action_scale = 0.0f, tau_action_scale = 0.0f;
    float obs_clip = 0.0f, act_clip = 0.0f;
    torch::jit::script::Module policy;
};
} // namespace unitree::common
