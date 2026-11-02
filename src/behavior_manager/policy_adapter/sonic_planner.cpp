/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file sonic_planner.cpp
 * @brief SONIC Kinematic Planner 异步参考轨迹生成器实现
 */

#include "sonic_planner.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace behavior_manager {
namespace policy_adapter {
namespace {

constexpr int kPlannerDofCount = 29;
constexpr int kPlannerQposDim = 36;
constexpr int kPlannerContextFrames = 4;
constexpr int kPlannerMaxFrames = 64;
constexpr int kPlannerAllowedTokenCount = 11;
constexpr int kPlannerBlendFrames = 8;
constexpr int kPlannerOutputSize =
    kPlannerMaxFrames * kPlannerQposDim;
constexpr double kPlannerFps = 30.0;

Eigen::Quaternionf NormalizeQuaternion(
    const std::array<float, 4> &quat_wxyz) {
    Eigen::Quaternionf quat(
        quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]);
    if (quat.norm() < 1.0e-6F) {
        return Eigen::Quaternionf::Identity();
    }
    return quat.normalized();
}

Eigen::Quaternionf RobotQuaternion(const robot_base::RobotData &robot) {
    Eigen::Quaternionf quat(
        static_cast<float>(robot.base_quat[0]),
        static_cast<float>(robot.base_quat[1]),
        static_cast<float>(robot.base_quat[2]),
        static_cast<float>(robot.base_quat[3]));
    if (quat.norm() < 1.0e-6F) {
        return Eigen::Quaternionf::Identity();
    }
    return quat.normalized();
}

float QuaternionYaw(const Eigen::Quaternionf &quat) {
    return std::atan2(
        2.0F * (quat.w() * quat.z() + quat.x() * quat.y()),
        1.0F - 2.0F * (quat.y() * quat.y() + quat.z() * quat.z()));
}

bool CommandChanged(const robot_base::Command &lhs,
    const robot_base::Command &rhs,
    double epsilon) {
    return std::abs(lhs.vx - rhs.vx) > epsilon ||
        std::abs(lhs.vy - rhs.vy) > epsilon ||
        std::abs(lhs.wz - rhs.wz) > epsilon;
}

bool IsStaticMode(int mode) {
    return mode == 0 || mode == 4 || mode == 5 || mode == 6 ||
        mode == 7 || mode == 9;
}

rl_policy::PolicyExecutorConfig BuildPlannerPolicyConfig(
    const SonicPlannerConfig &config) {
    rl_policy::PolicyExecutorConfig policy;
    policy.model_path = config.model_path;
    policy.runtime = config.runtime;
    policy.action_scale = {1.0};
    policy.rl_default_pos.assign(kPlannerOutputSize, 0.0);
    policy.strict_obs_dim_check = true;
    policy.custom_array_dims["planner_context"] =
        kPlannerContextFrames * kPlannerQposDim;

    rl_policy::ObsSegmentConfig segment;
    segment.terms = {"planner_context"};
    policy.obs_segments.push_back(std::move(segment));

    rl_policy::ModelInputBindingConfig input;
    input.name = "context_mujoco_qpos";
    input.source = rl_policy::ModelInputSource::OBSERVATION;
    policy.model_io.inputs.push_back(input);

    const std::array<const char *, 10> external_inputs = {
        "target_vel",
        "mode",
        "movement_direction",
        "facing_direction",
        "random_seed",
        "has_specific_target",
        "specific_target_positions",
        "specific_target_headings",
        "allowed_pred_num_tokens",
        "height",
    };
    for (const char *name : external_inputs) {
        input = {};
        input.name = name;
        input.source = rl_policy::ModelInputSource::EXTERNAL;
        policy.model_io.inputs.push_back(input);
    }

    rl_policy::ModelOutputBindingConfig output;
    output.name = "mujoco_qpos";
    output.target = rl_policy::ModelOutputTarget::ACTION;
    policy.model_io.outputs.push_back(output);

    output = {};
    output.name = "num_pred_frames";
    output.target = rl_policy::ModelOutputTarget::EXPOSE;
    policy.model_io.outputs.push_back(output);
    return policy;
}

}  // namespace

class SonicPlanner::Impl {
public:
    Impl(const SonicPlannerConfig &config,
        const rl_policy::PolicyExecutorConfig &tracker_policy_config)
        : config_(config),
            planner_policy_config_(BuildPlannerPolicyConfig(config)) {
        const int model_dof_count = tracker_policy_config.action_joint_index.empty()
            ? static_cast<int>(tracker_policy_config.rl_default_pos.size())
            : static_cast<int>(tracker_policy_config.action_joint_index.size());
        if (model_dof_count != kPlannerDofCount) {
            throw std::runtime_error(
                "[sonic_planner] model requires a 29-joint tracker");
        }

        model_to_mujoco_.resize(kPlannerDofCount);
        for (int model_joint = 0; model_joint < kPlannerDofCount;
            ++model_joint) {
            model_to_mujoco_[model_joint] =
                tracker_policy_config.action_joint_index.empty()
                ? model_joint
                : tracker_policy_config.action_joint_index[model_joint];
        }
        ValidateConfig();
        ValidateJointMapping();
    }

    ~Impl() { Stop(); }

    void Reset(const robot_base::RobotData &robot) {
        Stop();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_ = BuildSeedMotion(robot);
            latest_robot_ = robot;
            latest_command_ = {};
            latest_elapsed_s_ = 0.0;
            has_snapshot_ = true;
            request_generation_ = 1;
            plan_count_ = 0;
            published_generation_ = 0;
            idle_ = false;
            idle_ready_at_s_ = 0.0;
            error_.clear();
            running_ = true;
        }
        worker_ = std::thread(&Impl::PlannerLoop, this);
    }

    void Update(const robot_base::RobotData &robot,
        const robot_base::Command &command,
        double elapsed_s) {
        bool command_changed = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            command_changed = CommandChanged(
                latest_command_, command, config_.command_epsilon);
            latest_robot_ = robot;
            latest_command_ = command;
            latest_elapsed_s_ = std::max(0.0, elapsed_s);
            has_snapshot_ = true;
            if (command_changed) {
                ++request_generation_;
            }
        }
        if (command_changed) {
            condition_.notify_one();
        }
    }

    SonicPlannerMotion Motion() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!error_.empty()) throw std::runtime_error(error_);
        return motion_;
    }

    void RequestStop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
        }
        condition_.notify_one();
        planner_policy_.RequestInferenceTermination();
    }

    bool Idle() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return idle_ && published_generation_ == request_generation_ &&
            latest_elapsed_s_ >= idle_ready_at_s_;
    }

private:
    void ValidateConfig() const {
        if (!config_.Enabled()) {
            throw std::runtime_error(
                "[sonic_planner] enabled=true 时必须配置 model_path");
        }
        if (config_.output_fps <= 0.0 ||
            config_.replan_interval <= 0.0 || config_.lookahead < 0.0) {
            throw std::runtime_error(
                "[sonic_planner] fps/replan_interval/lookahead 配置无效");
        }
        if (config_.mode < -1 || config_.mode > 26) {
            throw std::runtime_error(
                "[sonic_planner] mode 必须为 -1(auto) 或 [0, 26]");
        }
        if (config_.allowed_pred_num_tokens.size() !=
            kPlannerAllowedTokenCount ||
            std::none_of(config_.allowed_pred_num_tokens.begin(),
                config_.allowed_pred_num_tokens.end(),
                [](int value) { return value != 0; })) {
            throw std::runtime_error(
                "[sonic_planner] allowed_pred_num_tokens 必须为 11 维且至少启用一个长度");
        }
        if (config_.slow_walk_max_speed <= 0.0 ||
            config_.walk_max_speed <= config_.slow_walk_max_speed) {
            throw std::runtime_error(
                "[sonic_planner] 自动 mode 速度阈值配置无效");
        }
    }

    void ValidateJointMapping() const {
        std::array<bool, kPlannerDofCount> seen = {};
        for (const int mujoco_joint : model_to_mujoco_) {
            if (mujoco_joint < 0 || mujoco_joint >= kPlannerDofCount ||
                seen[mujoco_joint]) {
                throw std::runtime_error(
                    "[sonic_planner] action_joint_index must permute all model joints");
            }
            seen[mujoco_joint] = true;
        }
    }

    SonicPlannerMotion BuildSeedMotion(
        const robot_base::RobotData &robot) const {
        if (robot.joint_pos.size() < kPlannerDofCount) {
            throw std::runtime_error(
                "[sonic_planner] 机器人关节状态不足 29 维");
        }

        SonicMotionFrame frame;
        frame.dof_pos.resize(kPlannerDofCount);
        frame.dof_vel.assign(kPlannerDofCount, 0.0F);
        for (int model_joint = 0; model_joint < kPlannerDofCount;
            ++model_joint) {
            frame.dof_pos[model_joint] = static_cast<float>(
                robot.joint_pos[model_to_mujoco_[model_joint]]);
        }
        for (int axis = 0; axis < 3; ++axis) {
            frame.root_pos[axis] = static_cast<float>(robot.base_pos[axis]);
        }
        if (!std::isfinite(frame.root_pos[2]) ||
            std::abs(frame.root_pos[2]) < 1.0e-4F) {
            frame.root_pos[2] = static_cast<float>(config_.default_height);
        }
        const Eigen::Quaternionf quat = RobotQuaternion(robot);
        frame.root_quat = {quat.w(), quat.x(), quat.y(), quat.z()};

        SonicPlannerMotion seed;
        seed.frames.assign(100, frame);
        seed.start_time_s = 0.0;
        return seed;
    }

    SonicMotionFrame SampleMotion(const SonicPlannerMotion &motion,
        double absolute_time_s) const {
        if (motion.frames.empty()) {
            throw std::runtime_error(
                "[sonic_planner] 没有可用于 planner context 的参考帧");
        }
        const double frame_position = std::clamp(
            (absolute_time_s - motion.start_time_s) * config_.output_fps,
            0.0,
            static_cast<double>(motion.frames.size() - 1));
        const int frame0 = static_cast<int>(std::floor(frame_position));
        const int frame1 = std::min(
            frame0 + 1, static_cast<int>(motion.frames.size()) - 1);
        const float alpha = static_cast<float>(frame_position - frame0);
        const auto &source0 = motion.frames[frame0];
        const auto &source1 = motion.frames[frame1];

        SonicMotionFrame sampled;
        sampled.dof_pos.resize(kPlannerDofCount);
        sampled.dof_vel.resize(kPlannerDofCount);
        for (int joint = 0; joint < kPlannerDofCount; ++joint) {
            sampled.dof_pos[joint] =
                (1.0F - alpha) * source0.dof_pos[joint] +
                alpha * source1.dof_pos[joint];
            sampled.dof_vel[joint] =
                (1.0F - alpha) * source0.dof_vel[joint] +
                alpha * source1.dof_vel[joint];
        }
        for (int axis = 0; axis < 3; ++axis) {
            sampled.root_pos[axis] =
                (1.0F - alpha) * source0.root_pos[axis] +
                alpha * source1.root_pos[axis];
        }
        const Eigen::Quaternionf quat0 =
            NormalizeQuaternion(source0.root_quat);
        const Eigen::Quaternionf quat1 =
            NormalizeQuaternion(source1.root_quat);
        const Eigen::Quaternionf quat = quat0.slerp(alpha, quat1).normalized();
        sampled.root_quat = {quat.w(), quat.x(), quat.y(), quat.z()};
        return sampled;
    }

    int SelectMode(const robot_base::Command &command) const {
        if (config_.mode >= 0) {
            return config_.mode;
        }
        const double speed = std::hypot(command.vx, command.vy);
        if (speed <= config_.command_epsilon &&
            std::abs(command.wz) <= config_.command_epsilon) {
            return 0;
        }
        if (speed < config_.slow_walk_max_speed) {
            return 1;
        }
        if (speed < config_.walk_max_speed) {
            return 2;
        }
        return 3;
    }

    bool NeedsPeriodicReplan(const robot_base::Command &command) const {
        const double speed = std::hypot(command.vx, command.vy);
        return !IsStaticMode(SelectMode(command)) &&
            (speed > config_.command_epsilon ||
                std::abs(command.wz) > config_.command_epsilon);
    }

    Eigen::VectorXf BuildContext(const SonicPlannerMotion &source,
        double context_start_s) const {
        Eigen::VectorXf context(
            kPlannerContextFrames * kPlannerQposDim);
        context.setZero();
        for (int context_frame = 0;
            context_frame < kPlannerContextFrames; ++context_frame) {
            const SonicMotionFrame frame = SampleMotion(source,
                context_start_s + context_frame / kPlannerFps);
            const int offset = context_frame * kPlannerQposDim;
            for (int axis = 0; axis < 3; ++axis) {
                context[offset + axis] = frame.root_pos[axis];
            }
            for (int component = 0; component < 4; ++component) {
                context[offset + 3 + component] = frame.root_quat[component];
            }
            for (int model_joint = 0; model_joint < kPlannerDofCount;
                ++model_joint) {
                context[offset + 7 + model_to_mujoco_[model_joint]] =
                    frame.dof_pos[model_joint];
            }
        }
        return context;
    }

    void SetPlannerInputs(const robot_base::RobotData &robot,
        const robot_base::Command &command) {
        const float speed = static_cast<float>(
            std::hypot(command.vx, command.vy));
        const std::array<float, 1> target_vel = {
            speed > config_.command_epsilon ? speed : -1.0F};
        const std::array<std::int64_t, 1> mode = {
            static_cast<std::int64_t>(SelectMode(command))};

        const float yaw = QuaternionYaw(RobotQuaternion(robot));
        std::array<float, 3> movement_direction = {};
        if (speed > config_.command_epsilon) {
            const float local_x = command.vx / speed;
            const float local_y = command.vy / speed;
            movement_direction[0] =
                std::cos(yaw) * local_x - std::sin(yaw) * local_y;
            movement_direction[1] =
                std::sin(yaw) * local_x + std::cos(yaw) * local_y;
        }
        const float target_yaw = yaw + static_cast<float>(
            command.wz * config_.facing_horizon);
        const std::array<float, 3> facing_direction = {
            std::cos(target_yaw), std::sin(target_yaw), 0.0F};
        const std::array<std::int64_t, 1> random_seed = {
            static_cast<std::int64_t>(config_.random_seed)};
        const std::array<std::int64_t, 1> has_specific_target = {0};
        const std::array<float, 12> target_positions = {};
        const std::array<float, 4> target_headings = {};
        std::array<std::int64_t, kPlannerAllowedTokenCount> allowed = {};
        for (int index = 0; index < kPlannerAllowedTokenCount; ++index) {
            allowed[index] = config_.allowed_pred_num_tokens[index];
        }
        const std::array<float, 1> height = {-1.0F};

        planner_policy_.SetModelInput(
            "target_vel", rl_policy::MakeTensorView(target_vel.data(), 1));
        planner_policy_.SetModelInput(
            "mode", rl_policy::MakeTensorView(mode.data(), 1));
        planner_policy_.SetModelInput("movement_direction",
            rl_policy::MakeTensorView(movement_direction.data(), 3));
        planner_policy_.SetModelInput("facing_direction",
            rl_policy::MakeTensorView(facing_direction.data(), 3));
        planner_policy_.SetModelInput("random_seed",
            rl_policy::MakeTensorView(random_seed.data(), 1));
        planner_policy_.SetModelInput("has_specific_target",
            rl_policy::MakeTensorView(has_specific_target.data(), 1));
        planner_policy_.SetModelInput("specific_target_positions",
            rl_policy::MakeTensorView(target_positions.data(), 12));
        planner_policy_.SetModelInput("specific_target_headings",
            rl_policy::MakeTensorView(target_headings.data(), 4));
        planner_policy_.SetModelInput("allowed_pred_num_tokens",
            rl_policy::MakeTensorView(allowed.data(), allowed.size()));
        planner_policy_.SetModelInput(
            "height", rl_policy::MakeTensorView(height.data(), 1));
    }

    SonicPlannerMotion ConvertPlannerOutput(
        const std::vector<double> &qpos,
        int valid_frames,
        double start_time_s) const {
        if (valid_frames < 2 || valid_frames > kPlannerMaxFrames ||
            qpos.size() != kPlannerOutputSize) {
            throw std::runtime_error(
                "[sonic_planner] planner 输出帧数或 qpos 维度错误");
        }
        const int output_frames = static_cast<int>(std::floor(
            valid_frames * config_.output_fps / kPlannerFps));
        if (output_frames < 2) {
            throw std::runtime_error(
                "[sonic_planner] 重采样后的参考轨迹帧数不足");
        }

        SonicPlannerMotion output;
        output.frames.resize(output_frames);
        output.start_time_s = start_time_s;
        for (int output_frame = 0; output_frame < output_frames;
            ++output_frame) {
            const double source_position =
                output_frame * kPlannerFps / config_.output_fps;
            const int source0 = static_cast<int>(std::floor(source_position));
            const int source1 = std::min(source0 + 1, valid_frames - 1);
            const float alpha = static_cast<float>(source_position - source0);
            const int offset0 = source0 * kPlannerQposDim;
            const int offset1 = source1 * kPlannerQposDim;
            auto &frame = output.frames[output_frame];
            frame.dof_pos.resize(kPlannerDofCount);
            frame.dof_vel.assign(kPlannerDofCount, 0.0F);

            for (int axis = 0; axis < 3; ++axis) {
                frame.root_pos[axis] = static_cast<float>(
                    (1.0F - alpha) * qpos[offset0 + axis] +
                    alpha * qpos[offset1 + axis]);
            }
            const std::array<float, 4> quat0 = {
                static_cast<float>(qpos[offset0 + 3]),
                static_cast<float>(qpos[offset0 + 4]),
                static_cast<float>(qpos[offset0 + 5]),
                static_cast<float>(qpos[offset0 + 6]),
            };
            const std::array<float, 4> quat1 = {
                static_cast<float>(qpos[offset1 + 3]),
                static_cast<float>(qpos[offset1 + 4]),
                static_cast<float>(qpos[offset1 + 5]),
                static_cast<float>(qpos[offset1 + 6]),
            };
            const Eigen::Quaternionf quat = NormalizeQuaternion(quat0)
                .slerp(alpha, NormalizeQuaternion(quat1)).normalized();
            frame.root_quat = {quat.w(), quat.x(), quat.y(), quat.z()};

            for (int model_joint = 0; model_joint < kPlannerDofCount;
                ++model_joint) {
                const int qpos_index = 7 + model_to_mujoco_[model_joint];
                const double position =
                    (1.0F - alpha) * qpos[offset0 + qpos_index] +
                    alpha * qpos[offset1 + qpos_index];
                if (!std::isfinite(position)) {
                    throw std::runtime_error(
                        "[sonic_planner] planner qpos 包含非有限值");
                }
                frame.dof_pos[model_joint] = static_cast<float>(position);
            }
        }

        for (int frame = 0; frame + 1 < output_frames; ++frame) {
            for (int joint = 0; joint < kPlannerDofCount; ++joint) {
                output.frames[frame].dof_vel[joint] = static_cast<float>(
                    (output.frames[frame + 1].dof_pos[joint] -
                        output.frames[frame].dof_pos[joint]) *
                    config_.output_fps);
            }
        }
        output.frames.back().dof_vel =
            output.frames[output_frames - 2].dof_vel;
        return output;
    }

    SonicPlannerMotion Generate(const robot_base::RobotData &robot,
        const robot_base::Command &command,
        double elapsed_s,
        const SonicPlannerMotion &source) {
        const double context_start_s = elapsed_s + config_.lookahead;
        Eigen::VectorXf context = BuildContext(source, context_start_s);
        SetPlannerInputs(robot, command);

        std::vector<double> qpos;
        planner_policy_.Infer(context, qpos);
        const rl_policy::TensorView frame_count =
            planner_policy_.GetModelOutputTensor("num_pred_frames");
        if (frame_count.element_type != rl_policy::TensorElementType::INT32 ||
            frame_count.element_count != 1 || !frame_count.data) {
            throw std::runtime_error(
                "[sonic_planner] num_pred_frames 不是 int32[1]");
        }
        const int valid_frames =
            *static_cast<const std::int32_t *>(frame_count.data);
        return ConvertPlannerOutput(qpos, valid_frames, context_start_s);
    }

    SonicPlannerMotion BlendWithCurrentMotion(
        const SonicPlannerMotion &current,
        const SonicPlannerMotion &generated,
        double publish_time_s) const {
        if (generated.frames.empty()) {
            throw std::runtime_error(
                "[sonic_planner] planner 生成了空参考轨迹");
        }
        const double generated_end_s = generated.start_time_s +
            static_cast<double>(generated.frames.size() - 1) /
                config_.output_fps;
        if (generated_end_s < publish_time_s) {
            return generated;
        }

        const int output_frames = std::max(2,
            static_cast<int>(std::floor(
                (generated_end_s - publish_time_s) * config_.output_fps)) +
                1);
        SonicPlannerMotion blended;
        blended.frames.reserve(output_frames);
        blended.start_time_s = publish_time_s;
        for (int frame_index = 0; frame_index < output_frames;
            ++frame_index) {
            const double sample_time_s = publish_time_s +
                static_cast<double>(frame_index) / config_.output_fps;
            SonicMotionFrame old_frame = SampleMotion(current, sample_time_s);
            const SonicMotionFrame new_frame =
                SampleMotion(generated, sample_time_s);
            const float new_weight = static_cast<float>(std::clamp(
                (sample_time_s - generated.start_time_s) *
                    config_.output_fps / kPlannerBlendFrames,
                0.0,
                1.0));
            const float old_weight = 1.0F - new_weight;
            for (int joint = 0; joint < kPlannerDofCount; ++joint) {
                old_frame.dof_pos[joint] =
                    old_weight * old_frame.dof_pos[joint] +
                    new_weight * new_frame.dof_pos[joint];
                old_frame.dof_vel[joint] =
                    old_weight * old_frame.dof_vel[joint] +
                    new_weight * new_frame.dof_vel[joint];
            }
            for (int axis = 0; axis < 3; ++axis) {
                old_frame.root_pos[axis] =
                    old_weight * old_frame.root_pos[axis] +
                    new_weight * new_frame.root_pos[axis];
            }
            const Eigen::Quaternionf blended_quat = NormalizeQuaternion(
                old_frame.root_quat).slerp(
                    new_weight,
                    NormalizeQuaternion(new_frame.root_quat)).normalized();
            old_frame.root_quat = {
                blended_quat.w(), blended_quat.x(),
                blended_quat.y(), blended_quat.z()};
            blended.frames.push_back(std::move(old_frame));
        }
        return blended;
    }

    void PlannerLoop() {
        try {
            const auto load_start = std::chrono::steady_clock::now();
            planner_policy_.Init(planner_policy_config_);
            const double load_seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - load_start).count();
            const auto runtime = planner_policy_.GetRuntimeInfo();
            std::cout << "[sonic_planner] 模型就绪: provider="
                << runtime.initialized_provider << ", load_s="
                << load_seconds << std::endl;
        } catch (const std::exception &error) {
            std::cerr << "[sonic_planner] 模型初始化失败: "
                << error.what() << std::endl;
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = std::string("[sonic_planner] initialization failed: ") + error.what();
            running_ = false;
            return;
        }

        std::uint64_t handled_generation = 0;
        auto next_periodic = std::chrono::steady_clock::now();
        while (true) {
            robot_base::RobotData robot;
            robot_base::Command command;
            double elapsed_s = 0.0;
            SonicPlannerMotion source;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                while (running_) {
                    const bool command_request =
                        request_generation_ != handled_generation;
                    const bool periodic_request = has_snapshot_ &&
                        NeedsPeriodicReplan(latest_command_) &&
                        std::chrono::steady_clock::now() >= next_periodic;
                    if (has_snapshot_ &&
                        (command_request || periodic_request)) {
                        break;
                    }
                    if (has_snapshot_ &&
                        NeedsPeriodicReplan(latest_command_)) {
                        condition_.wait_until(lock, next_periodic);
                    } else {
                        condition_.wait(lock);
                    }
                }
                if (!running_) {
                    return;
                }
                robot = latest_robot_;
                command = latest_command_;
                elapsed_s = latest_elapsed_s_;
                source = motion_;
                handled_generation = request_generation_;
            }

            const auto infer_start = std::chrono::steady_clock::now();
            try {
                SonicPlannerMotion generated =
                    Generate(robot, command, elapsed_s, source);
                const double infer_ms = 1000.0 * std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - infer_start).count();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!running_) return;
                    motion_ = BlendWithCurrentMotion(
                        motion_, generated, latest_elapsed_s_);
                    idle_ = SelectMode(command) == 0;
                    idle_ready_at_s_ = generated.start_time_s +
                        static_cast<double>(generated.frames.size() - 1) /
                        config_.output_fps;
                    published_generation_ = handled_generation;
                }
                ++plan_count_;
                if (plan_count_ <= 3 || plan_count_ % 10 == 0) {
                    std::cout << "[sonic_planner] plan=" << plan_count_
                        << ", mode=" << SelectMode(command)
                        << ", speed=" << std::hypot(command.vx, command.vy)
                        << ", infer_ms=" << infer_ms
                        << ", frames=" << generated.frames.size()
                        << std::endl;
                }
            } catch (const std::exception &error) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_) return;
                error_ = std::string("[sonic_planner] inference failed: ") + error.what();
                running_ = false;
                return;
            }
            // The configured interval is start-to-start, including inference.
            next_periodic = infer_start +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(config_.replan_interval));
        }
    }

    void Stop() {
        RequestStop();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    SonicPlannerConfig config_;
    rl_policy::PolicyExecutorConfig planner_policy_config_;
    std::vector<int> model_to_mujoco_;
    rl_policy::PolicyExecutor planner_policy_;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool running_ = false;
    bool has_snapshot_ = false;
    robot_base::RobotData latest_robot_;
    robot_base::Command latest_command_;
    double latest_elapsed_s_ = 0.0;
    std::uint64_t request_generation_ = 0;
    std::uint64_t plan_count_ = 0;
    std::uint64_t published_generation_ = 0;
    bool idle_ = false;
    double idle_ready_at_s_ = 0.0;
    std::string error_;
    SonicPlannerMotion motion_;
};

SonicPlanner::SonicPlanner(const SonicPlannerConfig &config,
    const rl_policy::PolicyExecutorConfig &tracker_policy_config)
    : impl_(std::make_unique<Impl>(config, tracker_policy_config)) {}

SonicPlanner::~SonicPlanner() = default;

void SonicPlanner::Reset(const robot_base::RobotData &robot) {
    impl_->Reset(robot);
}

void SonicPlanner::Update(const robot_base::RobotData &robot,
    const robot_base::Command &command,
    double elapsed_s) {
    impl_->Update(robot, command, elapsed_s);
}

SonicPlannerMotion SonicPlanner::Motion() const {
    return impl_->Motion();
}

void SonicPlanner::RequestStop() {
    impl_->RequestStop();
}

bool SonicPlanner::Idle() const {
    return impl_->Idle();
}

}  // namespace policy_adapter
}  // namespace behavior_manager
