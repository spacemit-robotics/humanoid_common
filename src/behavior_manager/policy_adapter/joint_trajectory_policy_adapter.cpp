/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file joint_trajectory_policy_adapter.cpp
 * @brief Joint trajectory takeover with policy-space action feedback
 */

#include "joint_trajectory_policy_adapter.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace behavior_manager {
namespace policy_adapter {
namespace {

class JointTrajectoryPolicyAdapter final : public PolicyAdapter {
public:
    JointTrajectoryPolicyAdapter(const Config &config,
        const rl_policy::PolicyExecutorConfig &policy)
        : config_(config.joint_trajectory), policy_(policy),
            player_(config_, static_cast<int>(policy.rl_default_pos.size())) {
        action_dim_ = policy.action_joint_index.empty()
            ? static_cast<int>(policy.rl_default_pos.size()) : static_cast<int>(policy.action_joint_index.size());
        if (config_.applied_action_term.empty() ||
            (policy.action_scale.size() != 1 && policy.action_scale.size() != static_cast<size_t>(action_dim_)))
            throw std::runtime_error("joint_trajectory: missing applied_action_term or invalid action scale");
        std::vector<bool> selected(policy.rl_default_pos.size(), false);
        for (const auto &clip : config_.actions) {
            for (int joint : clip.joint_indices) selected[joint] = true;
        }
        for (size_t joint = 0; joint < selected.size(); ++joint) {
            if (!selected[joint]) continue;
            int action = static_cast<int>(joint);
            if (!policy.action_joint_index.empty()) {
                const auto found = std::find(policy.action_joint_index.begin(), policy.action_joint_index.end(), joint);
                if (found == policy.action_joint_index.end())
                    throw std::runtime_error("joint_trajectory: joint is not controlled by policy");
                action = static_cast<int>(std::distance(policy.action_joint_index.begin(), found));
            }
            const double scale = Scale(action);
            if (!std::isfinite(scale) || std::abs(scale) <= 1e-12 || !std::isfinite(policy.rl_default_pos[joint]))
                throw std::runtime_error("joint_trajectory: invalid action mapping");
            bindings_.push_back({action, static_cast<int>(joint)});
        }
        Reset({});
    }

    const char *Type() const override { return "joint_trajectory"; }
    void Reset(const robot_base::RobotData &) override {
        applied_.assign(action_dim_, 0.0F);
        player_.Reset(policy_.rl_default_pos);
        elapsed_s_ = 0.0;
    }
    void PrepareInputs(const robot_base::RobotData &, double elapsed_s,
        rl_policy::PolicyExecutor &policy) override {
        if (!std::isfinite(elapsed_s) || elapsed_s < 0.0)
            throw std::runtime_error("joint_trajectory: invalid elapsed time");
        elapsed_s_ = elapsed_s;
        policy.SetCustomArray(config_.applied_action_term, applied_.data(), static_cast<int>(applied_.size()));
    }
    void HandleInteractionRequest(const robot_base::InteractionRequest &request, double elapsed_s) override {
        player_.Request(request, elapsed_s);
    }
    void OnAction(std::vector<double> &action) override {
        if (action.size() != static_cast<size_t>(action_dim_))
            throw std::runtime_error("joint_trajectory: action dimension mismatch");
        auto position = policy_.rl_default_pos;
        for (const auto &binding : bindings_)
            position[binding.joint] += action[binding.action] * Scale(binding.action);
        player_.Apply(elapsed_s_, position);
        for (const auto &binding : bindings_)
            action[binding.action] = (position[binding.joint] - policy_.rl_default_pos[binding.joint]) / Scale(binding.action);
        for (int i = 0; i < action_dim_; ++i) applied_[i] = static_cast<float>(action[i]);
    }
    robot_base::InteractionStatus GetInteractionStatus() const override { return player_.Status(); }

private:
    struct Binding { int action; int joint; };
    double Scale(int action) const {
        return policy_.action_scale.size() == 1 ? policy_.action_scale[0] : policy_.action_scale[action];
    }
    JointTrajectoryConfig config_;
    rl_policy::PolicyExecutorConfig policy_;
    joint_trajectory::Player player_;
    int action_dim_ = 0;
    double elapsed_s_ = 0.0;
    std::vector<Binding> bindings_;
    std::vector<float> applied_;
};

}  // namespace

std::unique_ptr<PolicyAdapter> CreateJointTrajectoryPolicyAdapter(
    const Config &config, const rl_policy::PolicyExecutorConfig &policy_config) {
    return std::make_unique<JointTrajectoryPolicyAdapter>(config, policy_config);
}

}  // namespace policy_adapter
}  // namespace behavior_manager
