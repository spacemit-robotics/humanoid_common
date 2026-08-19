/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file policy_adapter.cpp
 * @brief MJLab 和 ProtoMotions 策略输入适配
 */

#include "policy_adapter.h"

#include <cnpy.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace behavior_manager {
namespace policy_adapter {
namespace {

namespace fs = std::filesystem;

struct PlaybackSample {
    int frame0 = 0;
    int frame1 = 0;
    float alpha = 0.0F;
    bool hold_last_frame = false;
};

fs::path ResolvePath(const std::string &robot_dir, const std::string &path) {
    fs::path resolved(path);
    if (!resolved.is_absolute()) {
        resolved = fs::path(robot_dir) / resolved;
    }
    return fs::weakly_canonical(resolved);
}

PlaybackSample SamplePlayback(double elapsed_s,
    int num_frames,
    const Config &config) {
    if (num_frames <= 0) {
        throw std::runtime_error("[policy_adapter] 参考动作没有帧");
    }
    if (config.motion_fps <= 0.0 || config.playback_speed <= 0.0) {
        throw std::runtime_error("[policy_adapter] fps 和 playback_speed 必须大于 0");
    }
    if (config.loop_pause < 0.0) {
        throw std::runtime_error("[policy_adapter] loop_pause 不能小于 0");
    }

    const double frame_dt = 1.0 / config.motion_fps;
    const double duration = num_frames * frame_dt;
    const double active_wall_duration = duration / config.playback_speed;
    double cycle_elapsed = std::max(elapsed_s, 0.0);
    bool hold_last_frame = false;
    if (config.loop) {
        const double cycle_duration = active_wall_duration + config.loop_pause;
        cycle_elapsed = std::fmod(cycle_elapsed, cycle_duration);
        hold_last_frame = cycle_elapsed >= active_wall_duration;
    } else {
        hold_last_frame = cycle_elapsed >= active_wall_duration;
    }

    const double motion_time = hold_last_frame
        ? duration
        : std::clamp(cycle_elapsed * config.playback_speed, 0.0, duration);
    const double frame_position =
        std::min(motion_time / frame_dt, num_frames - 1.0);
    PlaybackSample sample;
    sample.frame0 = static_cast<int>(std::floor(frame_position));
    sample.frame1 = std::min(sample.frame0 + 1, num_frames - 1);
    sample.alpha = static_cast<float>(frame_position - sample.frame0);
    sample.hold_last_frame = hold_last_frame;
    return sample;
}

Eigen::Quaternionf RobotQuaternion(const robot_base::RobotData &robot) {
    Eigen::Quaternionf quat(static_cast<float>(robot.base_quat[0]),
                            static_cast<float>(robot.base_quat[1]),
                            static_cast<float>(robot.base_quat[2]),
                            static_cast<float>(robot.base_quat[3]));
    if (quat.norm() < 1.0e-6F) {
        return Eigen::Quaternionf::Identity();
    }
    return quat.normalized();
}

float QuaternionYaw(const Eigen::Quaternionf &quat) {
    const float w = quat.w();
    const float x = quat.x();
    const float y = quat.y();
    const float z = quat.z();
    return std::atan2(2.0F * (w * z + x * y),
        1.0F - 2.0F * (y * y + z * z));
}

Eigen::Quaternionf YawQuaternion(float yaw) {
    return Eigen::Quaternionf(
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
}

Eigen::Quaternionf AnchorQuaternion(
    const robot_base::RobotData &robot,
    const std::vector<int> &waist_joint_indices) {
    Eigen::Quaternionf anchor = RobotQuaternion(robot);
    if (waist_joint_indices.size() != 3) {
        return anchor;
    }
    auto joint_at = [&](int index) -> float {
        if (index < 0 ||
            static_cast<std::size_t>(index) >= robot.joint_pos.size()) {
            return 0.0F;
        }
        return static_cast<float>(robot.joint_pos[index]);
    };
    anchor *= Eigen::Quaternionf(Eigen::AngleAxisf(
        joint_at(waist_joint_indices[0]), Eigen::Vector3f::UnitZ()));
    anchor *= Eigen::Quaternionf(Eigen::AngleAxisf(
        joint_at(waist_joint_indices[1]), Eigen::Vector3f::UnitX()));
    anchor *= Eigen::Quaternionf(Eigen::AngleAxisf(
        joint_at(waist_joint_indices[2]), Eigen::Vector3f::UnitY()));
    return anchor.normalized();
}

void AppendQuaternionXyzw(
    const Eigen::Quaternionf &quat, std::vector<float> &output) {
    output.push_back(quat.x());
    output.push_back(quat.y());
    output.push_back(quat.z());
    output.push_back(quat.w());
}

class MjlabPolicyAdapter final : public PolicyAdapter {
public:
    explicit MjlabPolicyAdapter(const Config &config) : config_(config) {
        LoadReference();
    }

    const char *Type() const override { return "mjlab"; }

    void Reset(const robot_base::RobotData &robot) override {
        if (!config_.anchor_yaw_align) {
            heading_offset_ = Eigen::Quaternionf::Identity();
            return;
        }
        const Eigen::Quaternionf robot_anchor =
            AnchorQuaternion(robot, config_.anchor_waist_joint_indices);
        heading_offset_ = YawQuaternion(QuaternionYaw(robot_anchor)) *
            YawQuaternion(QuaternionYaw(reference_quat_[0])).conjugate();
        heading_offset_.normalize();
    }

    void PrepareInputs(const robot_base::RobotData &robot,
        double elapsed_s,
        rl_policy::PolicyExecutor &policy) override {
        const PlaybackSample sample =
            SamplePlayback(elapsed_s, num_frames_, config_);
        const Eigen::VectorXf &pos0 = reference_joint_pos_[sample.frame0];
        const Eigen::VectorXf &pos1 = reference_joint_pos_[sample.frame1];
        const Eigen::VectorXf &vel0 = reference_joint_vel_[sample.frame0];
        const Eigen::VectorXf &vel1 = reference_joint_vel_[sample.frame1];

        for (int joint = 0; joint < joint_dim_; ++joint) {
            motion_command_[joint] =
                (1.0F - sample.alpha) * pos0[joint] +
                sample.alpha * pos1[joint];
            motion_command_[joint_dim_ + joint] = sample.hold_last_frame
                ? 0.0F
                : static_cast<float>(config_.playback_speed) *
                    ((1.0F - sample.alpha) * vel0[joint] +
                    sample.alpha * vel1[joint]);
        }

        const Eigen::Quaternionf robot_anchor =
            AnchorQuaternion(robot, config_.anchor_waist_joint_indices);
        const Eigen::Vector3f robot_anchor_pos(
            static_cast<float>(robot.base_pos[0]),
            static_cast<float>(robot.base_pos[1]),
            static_cast<float>(robot.base_pos[2]));
        Eigen::Vector3f reference_pos =
            (1.0F - sample.alpha) * reference_pos_[sample.frame0] +
            sample.alpha * reference_pos_[sample.frame1];
        Eigen::Quaternionf reference_quat = reference_quat_[sample.frame0]
            .slerp(sample.alpha, reference_quat_[sample.frame1])
            .normalized();
        reference_pos = heading_offset_ * reference_pos;
        reference_quat = (heading_offset_ * reference_quat).normalized();

        const Eigen::Vector3f position_body = robot_anchor.conjugate() *
            (reference_pos - robot_anchor_pos);
        anchor_pos_body_ = {
            position_body.x(), position_body.y(), position_body.z()};

        const Eigen::Matrix3f rotation =
            (robot_anchor.conjugate() * reference_quat)
                .normalized()
                .toRotationMatrix();
        anchor_ori_body_ = {
            rotation(0, 0), rotation(0, 1),
            rotation(1, 0), rotation(1, 1),
            rotation(2, 0), rotation(2, 1)};

        policy.SetCustomArray("motion_command", motion_command_.data(),
            static_cast<int>(motion_command_.size()));
        policy.SetCustomArray("motion_anchor_pos_b", anchor_pos_body_.data(), 3);
        policy.SetCustomArray("motion_anchor_ori_b", anchor_ori_body_.data(), 6);
    }

private:
    void LoadReference() {
        std::cout << "[policy_adapter] 加载 MJLab NPZ: "
            << config_.reference_file << std::endl;
        const auto joint_pos =
            cnpy::npz_load(config_.reference_file, "joint_pos");
        const auto joint_vel =
            cnpy::npz_load(config_.reference_file, "joint_vel");
        const auto body_pos =
            cnpy::npz_load(config_.reference_file, "body_pos_w");
        const auto body_quat =
            cnpy::npz_load(config_.reference_file, "body_quat_w");

        auto require = [](bool condition, const std::string &message) {
            if (!condition) {
                throw std::runtime_error(
                    "[policy_adapter] MJLab NPZ 非法: " + message);
            }
        };
        require(joint_pos.shape.size() == 2,
                "joint_pos 应为 [T,N]");
        require(joint_vel.shape == joint_pos.shape,
                "joint_vel 与 joint_pos 维度不一致");
        require(body_pos.shape.size() == 3 && body_pos.shape[2] == 3,
                "body_pos_w 应为 [T,B,3]");
        require(body_quat.shape.size() == 3 && body_quat.shape[2] == 4,
                "body_quat_w 应为 [T,B,4]");
        require(body_pos.shape[0] == joint_pos.shape[0] &&
                    body_quat.shape[0] == joint_pos.shape[0],
                "body 与 joint 帧数不一致");
        require(joint_pos.word_size == sizeof(float) &&
                    joint_vel.word_size == sizeof(float) &&
                    body_pos.word_size == sizeof(float) &&
                    body_quat.word_size == sizeof(float),
                "运行时数组必须为 float32");

        num_frames_ = static_cast<int>(joint_pos.shape[0]);
        joint_dim_ = static_cast<int>(joint_pos.shape[1]);
        const int body_count = static_cast<int>(body_pos.shape[1]);
        require(num_frames_ > 0 && joint_dim_ > 0,
                "帧数和关节数必须大于 0");
        require(config_.anchor_body_index >= 0 &&
                    config_.anchor_body_index < body_count &&
                    config_.anchor_body_index <
                        static_cast<int>(body_quat.shape[1]),
                "anchor_body_index 越界");

        const std::size_t body_pos_stride =
            body_pos.shape[1] * body_pos.shape[2];
        const std::size_t body_quat_stride =
            body_quat.shape[1] * body_quat.shape[2];
        reference_joint_pos_.reserve(num_frames_);
        reference_joint_vel_.reserve(num_frames_);
        reference_pos_.reserve(num_frames_);
        reference_quat_.reserve(num_frames_);
        for (int frame = 0; frame < num_frames_; ++frame) {
            Eigen::VectorXf position(joint_dim_);
            Eigen::VectorXf velocity(joint_dim_);
            for (int joint = 0; joint < joint_dim_; ++joint) {
                position[joint] =
                    joint_pos.data<float>()[frame * joint_dim_ + joint];
                velocity[joint] =
                    joint_vel.data<float>()[frame * joint_dim_ + joint];
            }
            reference_joint_pos_.push_back(std::move(position));
            reference_joint_vel_.push_back(std::move(velocity));

            const float *position_data = body_pos.data<float>() +
                frame * body_pos_stride + config_.anchor_body_index * 3;
            reference_pos_.emplace_back(
                position_data[0], position_data[1], position_data[2]);

            const float *quat_data = body_quat.data<float>() +
                frame * body_quat_stride + config_.anchor_body_index * 4;
            reference_quat_.emplace_back(
                Eigen::Quaternionf(quat_data[0], quat_data[1],
                    quat_data[2], quat_data[3])
                    .normalized());
        }
        motion_command_.assign(joint_dim_ * 2, 0.0F);
        std::cout << "[policy_adapter] MJLab: " << num_frames_
            << " 帧, " << joint_dim_ << " 关节, speed="
            << config_.playback_speed << "x" << std::endl;
    }

    Config config_;
    int num_frames_ = 0;
    int joint_dim_ = 0;
    std::vector<Eigen::VectorXf> reference_joint_pos_;
    std::vector<Eigen::VectorXf> reference_joint_vel_;
    std::vector<Eigen::Vector3f> reference_pos_;
    std::vector<Eigen::Quaternionf> reference_quat_;
    Eigen::Quaternionf heading_offset_ = Eigen::Quaternionf::Identity();
    std::vector<float> motion_command_;
    std::array<float, 3> anchor_pos_body_ = {};
    std::array<float, 6> anchor_ori_body_ = {};
};

struct ProtomotionsFrame {
    Eigen::Quaternionf anchor_quat = Eigen::Quaternionf::Identity();
    std::vector<float> dof_pos;
    std::vector<float> dof_vel;
};

std::vector<double> ParseCsvRow(const std::string &line) {
    std::vector<double> values;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        const double value = std::stod(field);
        if (!std::isfinite(value)) {
            throw std::runtime_error(
                "[policy_adapter] 参考 CSV 包含非有限值");
        }
        values.push_back(value);
    }
    return values;
}

Eigen::Quaternionf MakeXyzwQuaternion(
    const std::vector<double> &values, int offset) {
    Eigen::Quaternionf quat(static_cast<float>(values[offset + 3]),
                            static_cast<float>(values[offset]),
                            static_cast<float>(values[offset + 1]),
                            static_cast<float>(values[offset + 2]));
    if (quat.norm() < 1.0e-6F) {
        throw std::runtime_error("[policy_adapter] 参考 CSV 包含零四元数");
    }
    quat.normalize();
    if (quat.w() < 0.0F) {
        quat.coeffs() *= -1.0F;
    }
    return quat;
}

class ProtomotionsPolicyAdapter final : public PolicyAdapter {
public:
    ProtomotionsPolicyAdapter(
        const Config &config,
        const rl_policy::PolicyExecutorConfig &policy_config)
        : config_(config), policy_config_(policy_config) {
        ValidateConfig();
        LoadReference();
    }

    const char *Type() const override { return "protomotions"; }

    void Reset(const robot_base::RobotData &robot) override {
        const Eigen::Quaternionf robot_heading_source =
            ProtomotionsAnchorQuaternion(robot);
        heading_offset_ = YawQuaternion(
            QuaternionYaw(robot_heading_source) -
            QuaternionYaw(motion_[0].anchor_quat));
        heading_offset_.normalize();
    }

    void PrepareInputs(const robot_base::RobotData &robot,
        double elapsed_s,
        rl_policy::PolicyExecutor &policy) override {
        const PlaybackSample sample = SamplePlayback(
            elapsed_s, static_cast<int>(motion_.size()), config_);
        frame_index_ = sample.frame0;
        SetProtomotionsInputs(robot, policy);
    }

private:
    void ValidateConfig() {
        robot_dof_count_ =
            static_cast<int>(policy_config_.rl_default_pos.size());
        if (robot_dof_count_ <= 0) {
            throw std::runtime_error(
                "[policy_adapter] tracker 需要非空 rl_default_pos");
        }
        model_dof_count_ = policy_config_.action_joint_index.empty()
            ? robot_dof_count_
            : static_cast<int>(policy_config_.action_joint_index.size());
        for (const int joint : policy_config_.action_joint_index) {
            if (joint < 0 || joint >= robot_dof_count_) {
                throw std::runtime_error(
                    "[policy_adapter] action_joint_index 越界: " +
                    std::to_string(joint));
            }
        }

        int previous_step = 0;
        for (const int step : config_.future_steps) {
            if (step <= previous_step) {
                throw std::runtime_error(
                    "[policy_adapter] ProtoMotions future_steps "
                    "必须为严格递增的正整数");
            }
            previous_step = step;
        }
        if (config_.future_steps.empty()) {
            throw std::runtime_error(
                "[policy_adapter] ProtoMotions future_steps 不能为空");
        }
        if (config_.anchor_waist_joint_indices.size() != 3) {
            throw std::runtime_error(
                "[policy_adapter] ProtoMotions 需要 3 个 anchor 腰关节索引");
        }
    }

    void LoadReference() {
        std::ifstream input(config_.reference_file);
        if (!input) {
            throw std::runtime_error(
                "[policy_adapter] 无法打开参考 CSV: " +
                config_.reference_file);
        }
        const int expected_columns = 9 + 2 * robot_dof_count_;
        std::string line;
        int line_number = 0;
        while (std::getline(input, line)) {
            ++line_number;
            if (line.empty() || line[0] == '#') {
                continue;
            }
            const std::vector<double> values = ParseCsvRow(line);
            if (static_cast<int>(values.size()) != expected_columns) {
                throw std::runtime_error(
                    "[policy_adapter] 参考 CSV 第 " +
                    std::to_string(line_number) + " 行列数错误: 实际=" +
                    std::to_string(values.size()) + ", 期望=" +
                    std::to_string(expected_columns));
            }

            ProtomotionsFrame frame;
            frame.dof_pos.assign(robot_dof_count_, 0.0F);
            frame.dof_vel.assign(robot_dof_count_, 0.0F);
            frame.anchor_quat = MakeXyzwQuaternion(values, 5);
            for (int joint = 0; joint < robot_dof_count_; ++joint) {
                frame.dof_pos[joint] =
                    static_cast<float>(values[9 + joint]);
                frame.dof_vel[joint] = static_cast<float>(
                    values[9 + robot_dof_count_ + joint]);
            }
            motion_.push_back(frame);
        }
        if (motion_.empty()) {
            throw std::runtime_error("[policy_adapter] 参考 CSV 没有数据帧");
        }
        std::cout << "[policy_adapter] " << Type() << ": "
            << motion_.size() << " 帧, robot_dof=" << robot_dof_count_
            << ", model_dof=" << model_dof_count_ << ", reference="
            << config_.reference_file << std::endl;
    }

    const ProtomotionsFrame &FrameAt(int index) const {
        return motion_[std::clamp(
            index, 0, static_cast<int>(motion_.size()) - 1)];
    }

    int ModelJointAt(int model_index) const {
        return policy_config_.action_joint_index.empty()
            ? model_index
            : policy_config_.action_joint_index[model_index];
    }

    void AppendDofInModelOrder(const std::vector<float> &dof,
        std::vector<float> &output) const {
        for (int model_index = 0;
            model_index < model_dof_count_;
            ++model_index) {
            output.push_back(dof[ModelJointAt(model_index)]);
        }
    }

    void AppendRobotDofInModelOrder(
        const std::vector<double> &dof,
        std::vector<float> &output) const {
        for (int model_index = 0;
            model_index < model_dof_count_;
            ++model_index) {
            output.push_back(static_cast<float>(
                dof[ModelJointAt(model_index)]));
        }
    }

    Eigen::Quaternionf ProtomotionsAnchorQuaternion(
        const robot_base::RobotData &robot) const {
        return AnchorQuaternion(robot, config_.anchor_waist_joint_indices);
    }

    void SetProtomotionsInputs(
        const robot_base::RobotData &robot,
        rl_policy::PolicyExecutor &policy) const {
        if (robot.joint_pos.size() <
                static_cast<std::size_t>(robot_dof_count_) ||
            robot.joint_vel.size() <
                static_cast<std::size_t>(robot_dof_count_)) {
            throw std::runtime_error(
                "[policy_adapter] ProtoMotions 机器人状态维度不足，期望=" +
                std::to_string(robot_dof_count_));
        }
        const Eigen::Quaternionf anchor =
            ProtomotionsAnchorQuaternion(robot);
        const std::array<float, 4> anchor_xyzw = {
            anchor.x(), anchor.y(), anchor.z(), anchor.w()};
        policy.SetModelInput(
            "current_anchor_rot", anchor_xyzw.data(), anchor_xyzw.size());

        std::vector<float> dof_velocity;
        dof_velocity.reserve(model_dof_count_);
        AppendRobotDofInModelOrder(robot.joint_vel, dof_velocity);
        policy.SetModelInput("current_dof_vel", dof_velocity.data(),
            dof_velocity.size());

        const std::array<float, 3> root_local_angular_velocity = {
            static_cast<float>(robot.gyro[0]),
            static_cast<float>(robot.gyro[1]),
            static_cast<float>(robot.gyro[2])};
        policy.SetModelInput("current_root_local_ang_vel",
            root_local_angular_velocity.data(),
            root_local_angular_velocity.size());

        std::vector<float> future_anchor;
        std::vector<float> future_dof_pos;
        std::vector<float> future_dof_vel;
        future_anchor.reserve(config_.future_steps.size() * 4);
        future_dof_pos.reserve(
            config_.future_steps.size() * model_dof_count_);
        future_dof_vel.reserve(
            config_.future_steps.size() * model_dof_count_);
        for (const int offset : config_.future_steps) {
            const ProtomotionsFrame &future = FrameAt(frame_index_ + offset);
            Eigen::Quaternionf aligned =
                (heading_offset_ * future.anchor_quat).normalized();
            AppendQuaternionXyzw(aligned, future_anchor);
            AppendDofInModelOrder(future.dof_pos, future_dof_pos);
            AppendDofInModelOrder(future.dof_vel, future_dof_vel);
        }
        policy.SetModelInput("mimic_future_anchor_rot", future_anchor.data(),
            future_anchor.size());
        policy.SetModelInput("mimic_future_dof_pos", future_dof_pos.data(),
            future_dof_pos.size());
        policy.SetModelInput("mimic_future_dof_vel", future_dof_vel.data(),
            future_dof_vel.size());
    }

    Config config_;
    rl_policy::PolicyExecutorConfig policy_config_;
    int robot_dof_count_ = 0;
    int model_dof_count_ = 0;
    std::vector<ProtomotionsFrame> motion_;
    int frame_index_ = 0;
    Eigen::Quaternionf heading_offset_ = Eigen::Quaternionf::Identity();
};

std::string ReadReferenceField(const robot_base::YamlFile &yaml,
    const std::string &base,
    const std::string &robot_dir) {
    const auto reference = yaml.Read<std::string>(base + ".reference_file");
    if (!reference || reference->empty()) {
        throw std::runtime_error(
            "[policy_adapter] 缺少 " + base + ".reference_file");
    }
    return ResolvePath(robot_dir, *reference).string();
}

}  // namespace

Config LoadConfig(const std::string &yaml_path,
    const std::string &policy_name,
    const std::string &robot_dir) {
    const robot_base::YamlFile yaml = robot_base::YamlFile::Load(yaml_path);
    const std::string policy_base =
        "rl_policy.onnx_infer.policies." + policy_name;
    std::string adapter_base = policy_base + ".policy_adapter";

    Config config;
    auto type = yaml.Read<std::string>(adapter_base + ".type");
    if (!type) {
        // 兼容迁移期间已有的 tracker 节点。
        adapter_base = policy_base + ".tracker";
        type = yaml.Read<std::string>(adapter_base + ".type");
    }
    if (type) {
        config.type = *type;
        config.reference_file =
            ReadReferenceField(yaml, adapter_base, robot_dir);
        config.motion_fps =
            yaml.Read<double>(adapter_base + ".motion_fps").value_or(50.0);
        config.playback_speed = yaml.Read<double>(
            adapter_base + ".playback_speed").value_or(1.0);
        config.loop =
            yaml.Read<bool>(adapter_base + ".loop").value_or(false);
        config.loop_pause = yaml.Read<double>(
            adapter_base + ".loop_pause").value_or(0.0);
        config.anchor_body_index = yaml.Read<int>(
            adapter_base + ".anchor_body_index").value_or(-1);
        config.anchor_waist_joint_indices = yaml.Read<std::vector<int>>(
            adapter_base + ".anchor_waist_joint_indices")
                .value_or(std::vector<int>{});
        config.anchor_yaw_align = yaml.Read<bool>(
            adapter_base + ".anchor_yaw_align").value_or(true);
        config.future_steps = yaml.Read<std::vector<int>>(
            adapter_base + ".future_steps").value_or(std::vector<int>{});
        return config;
    }

    // 兼容原有 unitree_rl_mjlab 平铺字段；新配置统一使用 policy_adapter。
    const auto motion_file =
        yaml.Read<std::string>(policy_base + ".motion_file");
    if (!motion_file || motion_file->empty()) {
        return config;
    }
    config.type = "mjlab";
    config.reference_file = ResolvePath(robot_dir, *motion_file).string();
    config.motion_fps =
        yaml.Read<double>(policy_base + ".motion_fps").value_or(50.0);
    config.playback_speed = yaml.Read<double>(
        policy_base + ".motion_playback_speed").value_or(1.0);
    config.loop =
        yaml.Read<bool>(policy_base + ".motion_loop").value_or(false);
    config.loop_pause = yaml.Read<double>(
        policy_base + ".motion_loop_pause").value_or(0.0);
    config.anchor_body_index = yaml.Read<int>(
        policy_base + ".anchor_body_index").value_or(-1);
    config.anchor_waist_joint_indices = yaml.Read<std::vector<int>>(
        policy_base + ".anchor_waist_joint_indices")
            .value_or(std::vector<int>{});
    config.anchor_yaw_align = yaml.Read<bool>(
        policy_base + ".anchor_yaw_align").value_or(true);
    return config;
}

std::unique_ptr<PolicyAdapter> Create(
    const Config &config,
    const rl_policy::PolicyExecutorConfig &policy_config) {
    if (!config.Enabled()) {
        return nullptr;
    }
    if (config.type == "mjlab" || config.type == "mjlab_tracking") {
        return std::make_unique<MjlabPolicyAdapter>(config);
    }
    if (config.type == "protomotions") {
        return std::make_unique<ProtomotionsPolicyAdapter>(
            config, policy_config);
    }
    throw std::runtime_error(
        "[policy_adapter] 不支持的 type: " + config.type);
}

}  // namespace policy_adapter
}  // namespace behavior_manager
