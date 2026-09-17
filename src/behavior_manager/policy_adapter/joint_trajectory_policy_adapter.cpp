/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file joint_trajectory_policy_adapter.cpp
 * @brief RL 内选择性关节轨迹接管策略适配器
 */

#include "joint_trajectory_policy_adapter.h"

#include <cnpy.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_manager {
namespace policy_adapter {
namespace {

double SmoothStep(double value) {
    const double clamped = std::clamp(value, 0.0, 1.0);
    return clamped * clamped * (3.0 - 2.0 * clamped);
}

struct JointTrajectoryBinding {
    int action_index = -1;
    int joint_index = -1;
    double action_scale = 1.0;
    double default_position = 0.0;
};

struct JointTrajectoryClip {
    JointTrajectoryActionConfig config;
    int frame_count = 0;
    int joint_count = 0;
    std::vector<float> joint_pos;
    std::vector<JointTrajectoryBinding> bindings;
};

class JointTrajectoryPolicyAdapter final : public PolicyAdapter {
public:
    JointTrajectoryPolicyAdapter(const Config &config,
        const rl_policy::PolicyExecutorConfig &policy_config)
        : config_(config), policy_config_(policy_config) {
        ConfigurePolicy();
        LoadClips();
    }

    const char *Type() const override { return "joint_trajectory"; }

    void Reset(const robot_base::RobotData &) override {
        applied_action_.assign(action_dim_, 0.0F);
        previous_applied_action_.assign(action_dim_, 0.0);
        blend_source_.clear();
        active_clip_ = nullptr;
        phase_start_elapsed_ = 0.0;
        playback_start_elapsed_ = 0.0;
        last_elapsed_s_ = 0.0;
        last_request_sequence_ = 0;
        status_ = {};
    }

    void PrepareInputs(const robot_base::RobotData &,
        double elapsed_s,
        rl_policy::PolicyExecutor &policy) override {
        if (!std::isfinite(elapsed_s) || elapsed_s < 0.0) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory elapsed_s 非法");
        }
        last_elapsed_s_ = elapsed_s;
        policy.SetCustomArray(config_.joint_trajectory.applied_action_term,
            applied_action_.data(), static_cast<int>(applied_action_.size()));
    }

    void HandleInteractionRequest(
        const robot_base::InteractionRequest &request,
        double elapsed_s) override {
        if (request.operation ==
                robot_base::InteractionRequest::Operation::NONE ||
            request.sequence == 0 ||
            request.sequence <= last_request_sequence_) {
            return;
        }

        if (request.operation ==
                robot_base::InteractionRequest::Operation::CANCEL) {
            last_request_sequence_ = request.sequence;
            status_.request_accepted = true;
            BeginBlendOut(elapsed_s, request.sequence);
            return;
        }

        const Phase phase = status_.phase;
        if (active_clip_ && (phase == Phase::BLEND_IN ||
            phase == Phase::PLAYING || phase == Phase::HOLDING ||
            phase == Phase::BLEND_OUT)) {
            last_request_sequence_ = request.sequence;
            status_.sequence = request.sequence;
            status_.request_accepted = false;
            return;
        }
        last_request_sequence_ = request.sequence;

        const auto clip = clips_by_name_.find(request.action);
        if (clip == clips_by_name_.end()) {
            status_.sequence = request.sequence;
            status_.request_accepted = false;
            status_.phase = robot_base::InteractionStatus::Phase::REJECTED;
            status_.progress = 0.0F;
            status_.action = request.action;
            return;
        }

        active_clip_ = &clips_[clip->second];
        blend_source_.clear();
        phase_start_elapsed_ = elapsed_s;
        playback_start_elapsed_ = elapsed_s +
            config_.joint_trajectory.blend_in_duration;
        status_.sequence = request.sequence;
        status_.request_accepted = true;
        status_.phase = robot_base::InteractionStatus::Phase::BLEND_IN;
        status_.progress = 0.0F;
        status_.action = request.action;
    }

    void OnAction(std::vector<double> &action) override {
        if (action.size() != static_cast<std::size_t>(action_dim_)) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory action 维度不匹配");
        }

        if (active_clip_) {
            ComposeAction(action);
        }
        previous_applied_action_ = action;
        for (int index = 0; index < action_dim_; ++index) {
            applied_action_[index] = static_cast<float>(action[index]);
        }
    }

    robot_base::InteractionStatus GetInteractionStatus() const override {
        return status_;
    }

private:
    using Phase = robot_base::InteractionStatus::Phase;

    void ConfigurePolicy() {
        const auto &trajectory = config_.joint_trajectory;
        if (trajectory.applied_action_term.empty()) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory 缺少 applied_action_term");
        }
        if (!std::isfinite(trajectory.blend_in_duration) ||
            trajectory.blend_in_duration < 0.0 ||
            !std::isfinite(trajectory.blend_out_duration) ||
            trajectory.blend_out_duration < 0.0) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory blend 时长非法");
        }
        if (policy_config_.rl_default_pos.empty()) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory 缺少 rl_default_pos");
        }
        action_dim_ = policy_config_.action_joint_index.empty()
            ? static_cast<int>(policy_config_.rl_default_pos.size())
            : static_cast<int>(policy_config_.action_joint_index.size());
        if (policy_config_.action_scale.size() != 1 &&
            policy_config_.action_scale.size() !=
                static_cast<std::size_t>(action_dim_)) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory action_scale 维度非法");
        }
        applied_action_.assign(action_dim_, 0.0F);
        previous_applied_action_.assign(action_dim_, 0.0);
    }

    JointTrajectoryBinding MakeBinding(int joint_index) const {
        if (joint_index < 0 || joint_index >=
                static_cast<int>(policy_config_.rl_default_pos.size())) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory 关节索引越界: " +
                std::to_string(joint_index));
        }
        int action_index = joint_index;
        if (!policy_config_.action_joint_index.empty()) {
            const auto position = std::find(
                policy_config_.action_joint_index.begin(),
                policy_config_.action_joint_index.end(), joint_index);
            if (position == policy_config_.action_joint_index.end()) {
                throw std::runtime_error(
                    "[policy_adapter] joint_trajectory 关节未被策略控制: " +
                    std::to_string(joint_index));
            }
            action_index = static_cast<int>(std::distance(
                policy_config_.action_joint_index.begin(), position));
        }
        const double scale = policy_config_.action_scale.size() == 1
            ? policy_config_.action_scale[0]
            : policy_config_.action_scale[action_index];
        const double default_position =
            policy_config_.rl_default_pos[joint_index];
        if (!std::isfinite(scale) || std::abs(scale) <= 1.0e-12 ||
            !std::isfinite(default_position)) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory 动作缩放或默认位置非法");
        }
        return {action_index, joint_index, scale, default_position};
    }

    void LoadClips() {
        if (!config_.joint_trajectory.Enabled()) {
            throw std::runtime_error(
                "[policy_adapter] joint_trajectory 动作列表为空");
        }
        for (const auto &action_config :
                config_.joint_trajectory.actions) {
            if (action_config.name.empty() || action_config.file.empty() ||
                action_config.joint_indices.empty()) {
                throw std::runtime_error(
                    "[policy_adapter] joint_trajectory 动作配置不完整");
            }
            if (clips_by_name_.count(action_config.name) != 0) {
                throw std::runtime_error(
                    "[policy_adapter] joint_trajectory 动作名重复: " +
                    action_config.name);
            }
            if (!std::isfinite(action_config.motion_fps) ||
                action_config.motion_fps <= 0.0 ||
                !std::isfinite(action_config.playback_speed) ||
                action_config.playback_speed <= 0.0) {
                throw std::runtime_error(
                    "[policy_adapter] joint_trajectory fps/speed 非法: " +
                    action_config.name);
            }

            const cnpy::NpyArray positions =
                cnpy::npz_load(action_config.file, "joint_pos");
            if (positions.shape.size() != 2 || positions.shape[0] == 0 ||
                positions.shape[1] != action_config.joint_indices.size() ||
                (positions.word_size != sizeof(float) &&
                    positions.word_size != sizeof(double))) {
                throw std::runtime_error(
                    "[policy_adapter] joint_trajectory NPZ joint_pos 应为 "
                    "float32/float64 [T,K]: " + action_config.file);
            }

            JointTrajectoryClip clip;
            clip.config = action_config;
            clip.frame_count = static_cast<int>(positions.shape[0]);
            clip.joint_count = static_cast<int>(positions.shape[1]);
            clip.joint_pos.resize(
                static_cast<std::size_t>(clip.frame_count * clip.joint_count));
            std::vector<bool> used_actions(action_dim_, false);
            for (int column = 0; column < clip.joint_count; ++column) {
                const auto binding = MakeBinding(
                    action_config.joint_indices[column]);
                if (used_actions[binding.action_index]) {
                    throw std::runtime_error(
                        "[policy_adapter] joint_trajectory 关节重复: " +
                        action_config.name);
                }
                used_actions[binding.action_index] = true;
                clip.bindings.push_back(binding);
            }
            for (std::size_t index = 0;
                    index < clip.joint_pos.size(); ++index) {
                const std::size_t source_index = positions.fortran_order
                    ? (index % positions.shape[1]) * positions.shape[0] +
                        index / positions.shape[1]
                    : index;
                const double value = positions.word_size == sizeof(float)
                    ? static_cast<double>(positions.data<float>()[source_index])
                    : positions.data<double>()[source_index];
                if (!std::isfinite(value)) {
                    throw std::runtime_error(
                        "[policy_adapter] joint_trajectory NPZ 包含非有限值: " +
                        action_config.file);
                }
                clip.joint_pos[index] = static_cast<float>(value);
            }
            clips_by_name_[action_config.name] = clips_.size();
            clips_.push_back(std::move(clip));
        }
        std::cout << "[policy_adapter] joint_trajectory: " << clips_.size()
            << " actions, applied_action="
            << config_.joint_trajectory.applied_action_term << std::endl;
    }

    double ClipPosition(const JointTrajectoryClip &clip,
        int binding_index,
        double playback_elapsed) const {
        const double frame_position = std::clamp(
            playback_elapsed * clip.config.playback_speed *
                clip.config.motion_fps,
            0.0, static_cast<double>(clip.frame_count - 1));
        const int frame0 = static_cast<int>(std::floor(frame_position));
        const int frame1 = std::min(frame0 + 1, clip.frame_count - 1);
        const double alpha = frame_position - frame0;
        const std::size_t index0 = static_cast<std::size_t>(
            frame0 * clip.joint_count + binding_index);
        const std::size_t index1 = static_cast<std::size_t>(
            frame1 * clip.joint_count + binding_index);
        return (1.0 - alpha) * clip.joint_pos[index0] +
            alpha * clip.joint_pos[index1];
    }

    double ToNormalized(const JointTrajectoryBinding &binding,
        double position) const {
        return (position - binding.default_position) /
            binding.action_scale;
    }

    void CaptureBlendSource() {
        blend_source_.clear();
        blend_source_.reserve(active_clip_->bindings.size());
        for (const auto &binding : active_clip_->bindings) {
            blend_source_.push_back(
                previous_applied_action_[binding.action_index]);
        }
    }

    void BeginBlendOut(double elapsed_s, uint64_t sequence) {
        if (!active_clip_) {
            status_.sequence = sequence;
            status_.phase = Phase::IDLE;
            status_.progress = 0.0F;
            status_.action.clear();
            return;
        }
        CaptureBlendSource();
        phase_start_elapsed_ = elapsed_s;
        status_.sequence = sequence;
        status_.phase = Phase::BLEND_OUT;
        status_.progress = 1.0F;
    }

    void ComposeAction(std::vector<double> &action) {
        if (blend_source_.empty()) CaptureBlendSource();

        Phase phase = status_.phase;
        double blend = 1.0;
        double playback_elapsed = 0.0;
        if (phase == Phase::BLEND_IN) {
            const double duration = config_.joint_trajectory.blend_in_duration;
            blend = duration <= 0.0 ? 1.0 : SmoothStep(
                (last_elapsed_s_ - phase_start_elapsed_) / duration);
            if (blend >= 1.0) {
                phase = Phase::PLAYING;
                status_.phase = phase;
            }
        }

        if (phase == Phase::PLAYING || phase == Phase::HOLDING) {
            playback_elapsed = std::max(
                0.0, last_elapsed_s_ - playback_start_elapsed_);
            const double duration = active_clip_->frame_count <= 1
                ? 0.0
                : (active_clip_->frame_count - 1) /
                    (active_clip_->config.motion_fps *
                        active_clip_->config.playback_speed);
            status_.progress = duration <= 0.0
                ? 1.0F
                : static_cast<float>(std::clamp(
                    playback_elapsed / duration, 0.0, 1.0));
            if (playback_elapsed >= duration) {
                playback_elapsed = duration;
                if (active_clip_->config.hold_last_frame) {
                    phase = Phase::HOLDING;
                    status_.phase = phase;
                } else {
                    phase = Phase::BLEND_OUT;
                    status_.phase = phase;
                    phase_start_elapsed_ = last_elapsed_s_;
                    blend_source_.clear();
                }
            }
        }

        if (phase == Phase::BLEND_OUT) {
            if (blend_source_.empty()) {
                blend_source_.reserve(active_clip_->bindings.size());
                for (int index = 0;
                    index < static_cast<int>(active_clip_->bindings.size());
                    ++index) {
                    const auto &binding = active_clip_->bindings[index];
                    const double position = ClipPosition(
                        *active_clip_, index, playback_elapsed);
                    blend_source_.push_back(
                        ToNormalized(binding, position));
                }
            }
            const double duration = config_.joint_trajectory.blend_out_duration;
            const double alpha = duration <= 0.0 ? 1.0 : SmoothStep(
                (last_elapsed_s_ - phase_start_elapsed_) / duration);
            for (int index = 0;
                index < static_cast<int>(active_clip_->bindings.size());
                ++index) {
                const auto &binding = active_clip_->bindings[index];
                action[binding.action_index] =
                    (1.0 - alpha) * blend_source_[index] +
                    alpha * action[binding.action_index];
            }
            if (alpha >= 1.0) {
                active_clip_ = nullptr;
                blend_source_.clear();
                status_.phase = Phase::FINISHED;
                status_.progress = 1.0F;
            }
            return;
        }

        for (int index = 0;
            index < static_cast<int>(active_clip_->bindings.size());
            ++index) {
            const auto &binding = active_clip_->bindings[index];
            const double position = ClipPosition(
                *active_clip_, index, playback_elapsed);
            const double target = ToNormalized(binding, position);
            action[binding.action_index] = phase == Phase::BLEND_IN
                ? (1.0 - blend) * blend_source_[index] + blend * target
                : target;
        }
    }

    Config config_;
    rl_policy::PolicyExecutorConfig policy_config_;
    int action_dim_ = 0;
    std::vector<JointTrajectoryClip> clips_;
    std::unordered_map<std::string, std::size_t> clips_by_name_;
    const JointTrajectoryClip *active_clip_ = nullptr;
    std::vector<float> applied_action_;
    std::vector<double> previous_applied_action_;
    std::vector<double> blend_source_;
    double phase_start_elapsed_ = 0.0;
    double playback_start_elapsed_ = 0.0;
    double last_elapsed_s_ = 0.0;
    uint64_t last_request_sequence_ = 0;
    robot_base::InteractionStatus status_;
};

}  // namespace

std::unique_ptr<PolicyAdapter> CreateJointTrajectoryPolicyAdapter(
    const Config &config,
    const rl_policy::PolicyExecutorConfig &policy_config) {
    return std::make_unique<JointTrajectoryPolicyAdapter>(
        config, policy_config);
}

}  // namespace policy_adapter
}  // namespace behavior_manager
