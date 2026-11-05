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
#include <exception>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "runtime_logger.h"

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

double MotionEndTime(const SonicPlannerMotion &motion, double fps) {
    return motion.start_time_s +
        static_cast<double>(motion.frames.size() - 1) / fps;
}

}  // namespace

SonicMotionFrame SampleSonicPlannerMotion(const SonicPlannerMotion &motion,
    double time_s, double fps) {
    if (motion.frames.empty()) {
        throw std::runtime_error(
            "[sonic_planner] 没有可用于 planner context 的参考帧");
    }
    const double frame_position = std::clamp(
        (time_s - motion.start_time_s) * fps,
        0.0,
        static_cast<double>(motion.frames.size() - 1));
    const int frame0 = static_cast<int>(std::floor(frame_position));
    const int frame1 = std::min(
        frame0 + 1, static_cast<int>(motion.frames.size()) - 1);
    const float alpha = static_cast<float>(frame_position - frame0);
    const auto &source0 = motion.frames[frame0];
    const auto &source1 = motion.frames[frame1];

    SonicMotionFrame sampled;
    sampled.dof_pos.resize(source0.dof_pos.size());
    sampled.dof_vel.resize(source0.dof_vel.size());
    for (std::size_t joint = 0; joint < sampled.dof_pos.size(); ++joint) {
        sampled.dof_pos[joint] =
            (1.0F - alpha) * source0.dof_pos[joint] +
            alpha * source1.dof_pos[joint];
    }
    for (std::size_t joint = 0; joint < sampled.dof_vel.size(); ++joint) {
        sampled.dof_vel[joint] =
            (1.0F - alpha) * source0.dof_vel[joint] +
            alpha * source1.dof_vel[joint];
    }
    for (int axis = 0; axis < 3; ++axis) {
        sampled.root_pos[axis] =
            (1.0F - alpha) * source0.root_pos[axis] +
            alpha * source1.root_pos[axis];
    }
    const Eigen::Quaternionf quat0 = NormalizeQuaternion(source0.root_quat);
    const Eigen::Quaternionf quat1 = NormalizeQuaternion(source1.root_quat);
    const Eigen::Quaternionf quat = quat0.slerp(alpha, quat1).normalized();
    sampled.root_quat = {quat.w(), quat.x(), quat.y(), quat.z()};
    return sampled;
}

SonicPlannerBlend BlendSonicPlannerMotion(const SonicPlannerMotion &current,
    const SonicPlannerMotion &generated, double publish_time_s, double fps,
    int blend_frames) {
    if (generated.frames.empty() || current.frames.empty()) {
        throw std::runtime_error(
            "[sonic_planner] planner 生成了空参考轨迹");
    }
    if (!(fps > 0.0) || blend_frames <= 0) {
        throw std::runtime_error("[sonic_planner] 过渡参数无效");
    }
    SonicPlannerBlend result;
    result.late_s = std::max(0.0, publish_time_s - generated.start_time_s);
    result.stale_s = std::max(0.0, publish_time_s - MotionEndTime(current, fps));
    // A plan published after its start must still crossfade from the
    // reference being tracked, or the tracker sees a step at publication.
    result.blend_start_s = std::max(generated.start_time_s, publish_time_s);
    const double generated_end_s = MotionEndTime(generated, fps);
    if (generated_end_s < publish_time_s) {
        result.expired = true;
        result.motion = generated;
        return result;
    }

    const int output_frames = std::max(2,
        static_cast<int>(std::floor(
            (generated_end_s - publish_time_s) * fps)) + 1);
    result.motion.frames.reserve(output_frames);
    result.motion.start_time_s = publish_time_s;
    for (int frame_index = 0; frame_index < output_frames; ++frame_index) {
        const double sample_time_s = publish_time_s +
            static_cast<double>(frame_index) / fps;
        SonicMotionFrame old_frame =
            SampleSonicPlannerMotion(current, sample_time_s, fps);
        const SonicMotionFrame new_frame =
            SampleSonicPlannerMotion(generated, sample_time_s, fps);
        const float new_weight = static_cast<float>(std::clamp(
            (sample_time_s - result.blend_start_s) * fps / blend_frames,
            0.0, 1.0));
        if (sample_time_s >= result.blend_start_s && new_weight < 1.0F) {
            ++result.blend_frames;
        }
        const float old_weight = 1.0F - new_weight;
        for (std::size_t joint = 0; joint < old_frame.dof_pos.size(); ++joint) {
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
        result.motion.frames.push_back(std::move(old_frame));
    }
    return result;
}

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
        LoadModel();
    }

    ~Impl() { Stop(); }

    void Reset(const robot_base::RobotData &robot) {
        Stop();
        // The model is loaded once; a previous Stop() must not cancel new plans.
        planner_policy_.ClearInferenceTermination();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_ = std::make_shared<const SonicPlannerMotion>(BuildSeedMotion(robot));
            latest_robot_ = robot;
            latest_command_ = {};
            latest_elapsed_s_ = 0.0;
            has_snapshot_ = true;
            request_generation_ = 1;
            plan_count_ = 0;
            late_plans_ = 0;
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

    std::shared_ptr<const SonicPlannerMotion> Motion() const {
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
    // Runs during policy preparation, before the policy can be selected, on a
    // thread with the planner's CPU set and scheduling so the inference workers
    // created with the session inherit them instead of the preparing thread's.
    void LoadModel() {
        std::exception_ptr failure;
        double load_s = 0.0;
        std::array<double, 2> warmup_ms = {};
        std::thread loader([&] {
            try {
                config_.thread.Apply();
                const auto start = std::chrono::steady_clock::now();
                planner_policy_.Init(planner_policy_config_);
                load_s = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                warmup_ms = Warmup();
            } catch (...) {
                failure = std::current_exception();
            }
        });
        loader.join();
        if (failure) {
            try {
                std::rethrow_exception(failure);
            } catch (const std::exception &error) {
                throw std::runtime_error(
                    std::string("[sonic_planner] initialization failed: ") + error.what());
            }
        }
        const auto runtime = planner_policy_.GetRuntimeInfo();
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(1)
            << "SONIC planner model ready: load_s=" << load_s
            << ", warmup_idle_ms=" << warmup_ms[0] << ", warmup_walk_ms=" << warmup_ms[1]
            << ", provider=" << runtime.initialized_provider
            << ", intra=" << runtime.ort_intra_threads
            << ", spinning=" << (runtime.ort_spinning ? "on" : "off")
            << ", affinity=" << (runtime.affinity.empty() ? "inherit" : runtime.affinity)
            << ", workers=" << (runtime.worker_threads.empty() ? "none" : runtime.worker_threads);
        runtime_logging::Log(runtime_logging::Level::kInfo, detail.str());
    }

    // One standing and one walking plan, so the first real plan does not pay
    // for lazy allocation inside the inference runtime.
    std::array<double, 2> Warmup() {
        auto robot = robot_base::RobotData::Create(kPlannerDofCount);
        robot.base_quat = {1.0, 0.0, 0.0, 0.0};
        const SonicPlannerMotion seed = BuildSeedMotion(robot);
        robot_base::Command walk;
        walk.vx = static_cast<float>(0.5 * config_.slow_walk_max_speed);
        std::array<double, 2> elapsed_ms = {};
        const std::array<robot_base::Command, 2> commands = {robot_base::Command{}, walk};
        for (std::size_t i = 0; i < commands.size(); ++i) {
            const auto start = std::chrono::steady_clock::now();
            static_cast<void>(Generate(robot, commands[i], 0.0, seed));
            elapsed_ms[i] = 1000.0 * std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
        }
        return elapsed_ms;
    }

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
        return SampleSonicPlannerMotion(motion, absolute_time_s, config_.output_fps);
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

    void PlannerLoop() {
        // The calling thread joins the inference work; keep it on the planner CPUs.
        config_.thread.Apply();
        std::uint64_t handled_generation = 0;
        auto next_periodic = std::chrono::steady_clock::now();
        while (true) {
            robot_base::RobotData robot;
            robot_base::Command command;
            double elapsed_s = 0.0;
            std::shared_ptr<const SonicPlannerMotion> source;
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
                    Generate(robot, command, elapsed_s, *source);
                const double infer_ms = 1000.0 * std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - infer_start).count();
                double publish_time_s = 0.0;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!running_) return;
                    publish_time_s = latest_elapsed_s_;
                }
                // Build the immutable trajectory outside the tracker-facing lock.
                // Only this worker publishes motion between Reset and Stop.
                SonicPlannerBlend blend = BlendSonicPlannerMotion(*source,
                    generated, publish_time_s, config_.output_fps, kPlannerBlendFrames);
                auto next_motion = std::make_shared<const SonicPlannerMotion>(
                    std::move(blend.motion));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!running_) return;
                    motion_.swap(next_motion);
                    idle_ = SelectMode(command) == 0;
                    idle_ready_at_s_ = MotionEndTime(generated, config_.output_fps);
                    published_generation_ = handled_generation;
                }
                ++plan_count_;
                RecordPlan(command, elapsed_s, publish_time_s, infer_ms, generated, blend);
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

    // Planner worker only: one CSV row per published plan, plus a rate-limited
    // warning when inference outlasted the lookahead budget.
    void RecordPlan(const robot_base::Command &command, double request_time_s,
        double publish_time_s, double infer_ms, const SonicPlannerMotion &generated,
        const SonicPlannerBlend &blend) {
        const int mode = SelectMode(command);
        std::ostringstream row;
        row << std::fixed << std::setprecision(6)
            << std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count()
            << ',' << plan_count_ << ',' << mode << ',' << command.vx << ','
            << command.vy << ',' << command.wz << ',' << request_time_s << ','
            << publish_time_s << ',' << generated.start_time_s << ','
            << MotionEndTime(generated, config_.output_fps) << ',' << infer_ms << ','
            << generated.frames.size() << ',' << blend.late_s << ',' << blend.stale_s
            << ',' << blend.blend_frames << ',' << (blend.expired ? 1 : 0);
        runtime_logging::RecordCsv("control_sonic_planner",
            "wall_time_s,plan,mode,vx,vy,wz,request_time_s,publish_time_s,"
            "plan_start_s,plan_end_s,infer_ms,frames,late_s,stale_s,blend_frames,expired",
            row.str());
        if (plan_count_ == 1) {
            std::ostringstream detail;
            detail << std::fixed << std::setprecision(1)
                << "SONIC planner first plan published: mode=" << mode
                << ", infer_ms=" << infer_ms;
            runtime_logging::Log(runtime_logging::Level::kInfo, detail.str(), false);
        }
        if (blend.late_s <= 0.0 && blend.stale_s <= 0.0 && !blend.expired) return;
        const auto now = std::chrono::steady_clock::now();
        ++late_plans_;
        if (late_plans_ > 1 && now - last_late_report_ < std::chrono::seconds(10)) return;
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(1)
            << "SONIC planner plan published late: plan=" << plan_count_
            << ", infer_ms=" << infer_ms << ", lookahead_ms=" << config_.lookahead * 1000.0
            << ", late_ms=" << blend.late_s * 1000.0
            << ", reference_stalled_ms=" << blend.stale_s * 1000.0
            << ", expired=" << (blend.expired ? 1 : 0) << ", late_plans=" << late_plans_;
        runtime_logging::Log(runtime_logging::Level::kWarning, detail.str(), false);
        last_late_report_ = now;
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
    std::uint64_t late_plans_ = 0;
    std::chrono::steady_clock::time_point last_late_report_;
    std::uint64_t published_generation_ = 0;
    bool idle_ = false;
    double idle_ready_at_s_ = 0.0;
    std::string error_;
    std::shared_ptr<const SonicPlannerMotion> motion_;
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

std::shared_ptr<const SonicPlannerMotion> SonicPlanner::Motion() const {
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
