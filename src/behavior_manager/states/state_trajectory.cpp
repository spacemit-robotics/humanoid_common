/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_trajectory.cpp
 * @brief Fixed-base joint trajectory behavior without policy inference
 */

#include <memory>
#include <vector>

#include "state_factory.h"

namespace behavior_manager {
namespace {

class StateTrajectory final : public State {
public:
    StateTrajectory(const joint_trajectory::Config &config,
        const std::vector<double> &ready_position,
        const std::vector<double> &kp, const std::vector<double> &kd)
        : player_(config, static_cast<int>(ready_position.size())),
            ready_position_(ready_position), kp_(kp), kd_(kd) {}

    void OnEnter() override {
        elapsed_s_ = 0.0;
        player_.Reset(ready_position_);
        if (command_) command_->interaction = {};
        output_->actuation_mode = robot_base::ActuationMode::HYBRID;
        output_->kp = kp_;
        output_->kd = kd_;
        output_->target_pos = ready_position_;
        output_->target_vel.assign(ready_position_.size(), 0.0);
        output_->target_torque.assign(ready_position_.size(), 0.0);
        output_->enable = true;
    }

    void Run(float control_dt, float rl_dt) override {
        (void)rl_dt;
        if (command_) player_.Request(command_->interaction, elapsed_s_);
        output_->target_pos = ready_position_;
        player_.Apply(elapsed_s_, output_->target_pos);
        elapsed_s_ += control_dt;
    }

    StateName CheckTransition() override {
        if (command_ && (command_->key == 1 || command_->key == -1)) {
            const auto next = command_->key == 1 ? StateName::DAMP : StateName::POWER_OFF;
            command_->key = 0;
            return next;
        }
        return StateName::TRAJECTORY;
    }

    void OnExit() override { player_.Reset(ready_position_); }
    robot_base::InteractionStatus CurrentInteractionStatus() const override {
        return player_.Status();
    }

private:
    joint_trajectory::Player player_;
    std::vector<double> ready_position_, kp_, kd_;
    double elapsed_s_ = 0.0;
};

}  // namespace

std::unique_ptr<State> CreateStateTrajectory(const joint_trajectory::Config &config,
    const std::vector<double> &ready_position,
    const std::vector<double> &kp, const std::vector<double> &kd) {
    return std::make_unique<StateTrajectory>(config, ready_position, kp, kd);
}

}  // namespace behavior_manager
