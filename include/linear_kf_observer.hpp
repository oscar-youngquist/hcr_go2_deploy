#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "unitree/idl/go2/LowState_.hpp"

namespace unitree::common
{
class LinearKFObserver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LinearKFObserver(const std::string &config_file_path, float dt) : dt_(dt)
    {
        LoadParams(config_file_path);
        Setup();
    }

    void Update(const unitree_go::msg::dds_::LowState_ &state)
    {
        Eigen::Matrix<float, 18, 18> Q = Eigen::Matrix<float, 18, 18>::Identity();
        Q.block<3, 3>(0, 0) = Q0_.block<3, 3>(0, 0) * process_noise_pimu_;
        Q.block<3, 3>(3, 3) = Q0_.block<3, 3>(3, 3) * process_noise_vimu_;
        Q.block<12, 12>(6, 6) = Q0_.block<12, 12>(6, 6) * process_noise_pfoot_;

        Eigen::Matrix<float, 28, 28> R = Eigen::Matrix<float, 28, 28>::Identity();
        R.block<12, 12>(0, 0) = R0_.block<12, 12>(0, 0) * sensor_noise_pimu_rel_foot_;
        R.block<12, 12>(12, 12) = R0_.block<12, 12>(12, 12) * sensor_noise_vimu_rel_foot_;
        R.block<4, 4>(24, 24) = R0_.block<4, 4>(24, 24) * sensor_noise_zfoot_;

        const auto &imu = state.imu_state();
        const Eigen::Vector3f acc(imu.accelerometer()[0], imu.accelerometer()[1], imu.accelerometer()[2]);
        const Eigen::Vector3f gyro(imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2]);
        Eigen::Quaternionf body_quat(imu.quaternion()[0], imu.quaternion()[1],
                                     imu.quaternion()[2], imu.quaternion()[3]);
        body_quat = body_quat.norm() <= 1e-6f ? Eigen::Quaternionf::Identity() : body_quat.normalized();
        Rbw_ = body_quat.toRotationMatrix();
        const Eigen::Matrix3f Rwb = Rbw_.transpose();
        const Eigen::Vector3f a_world = Rwb * acc + Eigen::Vector3f(0.0f, 0.0f, -9.81f);

        Eigen::Matrix<float, 4, 1> foot_heights = Eigen::Matrix<float, 4, 1>::Zero();
        const Eigen::Vector3f base_pos = xhat_.block<3, 1>(0, 0);
        const Eigen::Vector3f base_vel = xhat_.block<3, 1>(3, 0);
        for (int leg = 0; leg < 4; ++leg)
        {
            const int i1 = 3 * leg;
            const int q_index = 6 + i1;
            const Eigen::Vector3f joints(state.motor_state()[i1].q(),
                                         state.motor_state()[i1 + 1].q(),
                                         state.motor_state()[i1 + 2].q());
            const Eigen::Vector3f qd(state.motor_state()[i1].dq(),
                                     -state.motor_state()[i1 + 1].dq(),
                                     -state.motor_state()[i1 + 2].dq());
            Eigen::Vector3f p_rel = Eigen::Vector3f::Zero();
            Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
            ComputeLegPositionAndJacobian(joints, leg, p_rel, J);
            const Eigen::Vector3f p_f = Rwb * p_rel;
            const Eigen::Vector3f dp_f = Rwb * (gyro.cross(p_rel) + J * qd);
            const float trust = std::clamp(
                static_cast<float>(state.foot_force()[leg]) / sensor_airbag_contact_lower_, 0.0f, 1.0f);
            constexpr float high_suspect = 100.0f;

            Q.block<3, 3>(q_index, q_index) *= 1.0f + (1.0f - trust) * high_suspect;
            R.block<3, 3>(12 + i1, 12 + i1) *= 1.0f + (1.0f - trust) * high_suspect;
            R(24 + leg, 24 + leg) *= 1.0f + (1.0f - trust) * high_suspect;
            ps_.segment<3>(i1) = -p_f;
            vs_.segment<3>(i1) = (1.0f - trust) * base_vel + trust * (-dp_f);
            foot_heights(leg) = (1.0f - trust) * (base_pos.z() + p_f.z());
        }

        Eigen::Matrix<float, 28, 1> y;
        y << ps_, vs_, foot_heights;
        xhat_ = A_ * xhat_ + B_ * a_world;
        const auto predicted_cov = A_ * P_ * A_.transpose() + Q;
        const auto innovation_cov = C_ * predicted_cov * C_.transpose() + R;
        const auto gain_rhs = predicted_cov * C_.transpose();
        xhat_ += gain_rhs * innovation_cov.ldlt().solve(y - C_ * xhat_);
        P_ = (Eigen::Matrix<float, 18, 18>::Identity() -
              gain_rhs * innovation_cov.ldlt().solve(C_)) * predicted_cov;
        P_ = (P_ + P_.transpose()) * 0.5f;
    }

    std::array<float, 3> GetEstimatedPosition() const { return ToArray(xhat_.block<3, 1>(0, 0)); }
    std::array<float, 3> GetEstimatedVelocityWorld() const { return ToArray(xhat_.block<3, 1>(3, 0)); }
    std::array<float, 3> GetEstimatedVelocityBody() const { return ToArray(Rbw_ * xhat_.block<3, 1>(3, 0)); }

private:
    template <typename T>
    static void LoadOptional(const YAML::Node &cfg, const char *key, T &value)
    {
        if (cfg[key]) value = cfg[key].as<T>();
    }

    void LoadParams(const std::string &path)
    {
        const YAML::Node cfg = YAML::LoadFile(path);
        LoadOptional(cfg, "estimator_dt", dt_);
        LoadOptional(cfg, "imu_process_noise_position", process_noise_pimu_);
        LoadOptional(cfg, "imu_process_noise_velocity", process_noise_vimu_);
        LoadOptional(cfg, "foot_process_noise_position", process_noise_pfoot_);
        LoadOptional(cfg, "foot_sensor_noise_position", sensor_noise_pimu_rel_foot_);
        LoadOptional(cfg, "foot_sensor_noise_velocity", sensor_noise_vimu_rel_foot_);
        LoadOptional(cfg, "foot_height_sensor_noise", sensor_noise_zfoot_);
        LoadOptional(cfg, "foot_force_contact_threshold", sensor_airbag_contact_lower_);
        LoadOptional(cfg, "go2_body_length", body_length_);
        LoadOptional(cfg, "go2_body_width", body_width_);
        LoadOptional(cfg, "go2_abad_link_length", abad_link_length_);
        LoadOptional(cfg, "go2_hip_link_length", hip_link_length_);
        LoadOptional(cfg, "go2_knee_link_length", knee_link_length_);
    }

    void Setup()
    {
        A_.setIdentity();
        A_.block<3, 3>(0, 3) = dt_ * Eigen::Matrix3f::Identity();
        B_.setZero();
        B_.block<3, 3>(3, 0) = dt_ * Eigen::Matrix3f::Identity();
        C_.setZero();
        for (int leg = 0; leg < 4; ++leg)
        {
            C_.block<3, 3>(3 * leg, 0).setIdentity();
            C_.block<3, 3>(3 * leg, 6 + 3 * leg) = -Eigen::Matrix3f::Identity();
            C_.block<3, 3>(12 + 3 * leg, 3).setIdentity();
            C_(24 + leg, 8 + 3 * leg) = 1.0f;
        }
        P_.setIdentity();
        P_ *= 100.0f;
        Q0_.setIdentity();
        Q0_.block<3, 3>(0, 0) *= dt_ / 20.0f;
        Q0_.block<3, 3>(3, 3) *= dt_ * 9.8f / 20.0f;
        Q0_.block<12, 12>(6, 6) *= dt_;
        R0_.setIdentity();
    }

    void ComputeLegPositionAndJacobian(const Eigen::Vector3f &q, int leg,
                                       Eigen::Vector3f &p, Eigen::Matrix3f &J) const
    {
        const float side = (leg == 0 || leg == 2) ? -1.0f : 1.0f;
        const float s1 = std::sin(q[0]), c1 = std::cos(q[0]);
        const float s2 = std::sin(-q[1]), c2 = std::cos(-q[1]);
        const float s3 = std::sin(-q[2]), c3 = std::cos(-q[2]);
        const float c23 = c2 * c3 - s2 * s3, s23 = s2 * c3 + c2 * s3;
        p = {knee_link_length_ * s23 + hip_link_length_ * s2,
             abad_link_length_ * side * c1 + knee_link_length_ * s1 * c23 + hip_link_length_ * c2 * s1,
             abad_link_length_ * side * s1 - knee_link_length_ * c1 * c23 - hip_link_length_ * c1 * c2};
        p += GetHipLocation(leg);
        J << 0.0f, knee_link_length_ * c23 + hip_link_length_ * c2, knee_link_length_ * c23,
             knee_link_length_ * c1 * c23 + hip_link_length_ * c1 * c2 - abad_link_length_ * side * s1,
             -knee_link_length_ * s1 * s23 - hip_link_length_ * s1 * s2, -knee_link_length_ * s1 * s23,
             knee_link_length_ * s1 * c23 + hip_link_length_ * c2 * s1 + abad_link_length_ * side * c1,
             knee_link_length_ * c1 * s23 + hip_link_length_ * c1 * s2, knee_link_length_ * c1 * s23;
    }

    Eigen::Vector3f GetHipLocation(int leg) const
    {
        return {(leg < 2) ? 0.5f * body_length_ : -0.5f * body_length_,
                (leg == 1 || leg == 3) ? 0.5f * body_width_ : -0.5f * body_width_, 0.0f};
    }

    static std::array<float, 3> ToArray(const Eigen::Vector3f &v) { return {v[0], v[1], v[2]}; }

    Eigen::Matrix<float, 18, 1> xhat_ = Eigen::Matrix<float, 18, 1>::Zero();
    Eigen::Matrix<float, 12, 1> ps_ = Eigen::Matrix<float, 12, 1>::Zero(), vs_ = ps_;
    Eigen::Matrix<float, 18, 18> A_, Q0_, P_;
    Eigen::Matrix<float, 28, 28> R0_;
    Eigen::Matrix<float, 18, 3> B_;
    Eigen::Matrix<float, 28, 18> C_;
    Eigen::Matrix3f Rbw_ = Eigen::Matrix3f::Identity();
    float dt_ = 0.002f;
    float process_noise_pimu_ = 0.02f, process_noise_vimu_ = 0.02f, process_noise_pfoot_ = 0.002f;
    float sensor_noise_pimu_rel_foot_ = 0.001f, sensor_noise_vimu_rel_foot_ = 0.1f;
    float sensor_noise_zfoot_ = 0.001f, sensor_airbag_contact_lower_ = 15.0f;
    float body_length_ = 0.3868f, body_width_ = 0.093f;
    float abad_link_length_ = 0.0955f, hip_link_length_ = 0.213f, knee_link_length_ = 0.213f;
};
} // namespace unitree::common
