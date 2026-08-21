/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_home.cpp
 * @brief 机型默认姿态复位状态
 */

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

#include "behavior_state.h"

namespace behavior_manager {
namespace {

double SmoothStep(double progress) {
    const double s = std::clamp(progress, 0.0, 1.0);
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

}  // namespace

class StateHome : public State {
public:
    void SetConfig(const std::vector<double> &default_pos,
            double gain_ramp_duration,
            double move_duration,
            const std::vector<double> &kp,
            const std::vector<double> &kd) {
        default_pos_ = default_pos;
        gain_ramp_duration_ = gain_ramp_duration;
        move_duration_ = move_duration;
        kp_ = kp;
        kd_ = kd;
    }

    void OnEnter() override {
        std::cout << "[StateHome] 进入机型默认姿态复位状态" << std::endl;
        timer_ = 0.0;
        finished_ = false;
        init_pos_ = sensor_ ? sensor_->joint_pos : std::vector<double>{};
        if (!output_ || init_pos_.size() != default_pos_.size()) return;

        const std::size_t num_dof = default_pos_.size();
        output_->enable = true;
        output_->actuation_mode = robot_base::ActuationMode::HYBRID;
        output_->target_pos = init_pos_;
        output_->target_vel.assign(num_dof, 0.0);
        output_->target_torque.assign(num_dof, 0.0);
        output_->kp.assign(num_dof, 0.0);
        output_->kd = kd_;
    }

    void Run(float control_dt, float rl_dt) override {
        (void)rl_dt;
        if (!output_ || init_pos_.size() != default_pos_.size()) return;

        const std::size_t num_dof = default_pos_.size();
        output_->enable = true;
        output_->actuation_mode = robot_base::ActuationMode::HYBRID;
        output_->target_vel.assign(num_dof, 0.0);
        output_->target_torque.assign(num_dof, 0.0);
        output_->kd = kd_;

        if (timer_ < gain_ramp_duration_) {
            const double alpha = SmoothStep(timer_ / gain_ramp_duration_);
            output_->target_pos = init_pos_;
            output_->kp.resize(num_dof);
            for (std::size_t i = 0; i < num_dof; ++i) output_->kp[i] = alpha * kp_[i];
        } else if (!finished_) {
            const double elapsed = timer_ - gain_ramp_duration_;
            const double alpha = SmoothStep(elapsed / move_duration_);
            output_->target_pos.resize(num_dof);
            output_->kp = kp_;
            for (std::size_t i = 0; i < num_dof; ++i) {
                output_->target_pos[i] = init_pos_[i] + alpha * (default_pos_[i] - init_pos_[i]);
            }
            if (elapsed >= move_duration_) finished_ = true;
        } else {
            output_->target_pos = default_pos_;
            output_->kp = kp_;
        }
        timer_ += control_dt;
    }

    StateName CheckTransition() override {
        if (command_ && command_->key == 2 && finished_) {
            command_->key = 0;
            return StateName::ZERO;
        }
        if (command_ && command_->key == 1) {
            command_->key = 0;
            return StateName::DAMP;
        }
        if (command_ && command_->key == -1) {
            command_->key = 0;
            return StateName::POWER_OFF;
        }
        return StateName::HOME;
    }

    void OnExit() override {
        std::cout << "[StateHome] 退出机型默认姿态复位状态" << std::endl;
    }

private:
    std::vector<double> default_pos_;
    std::vector<double> init_pos_;
    std::vector<double> kp_;
    std::vector<double> kd_;
    double gain_ramp_duration_ = 1.0;
    double move_duration_ = 5.0;
    double timer_ = 0.0;
    bool finished_ = false;
};

std::unique_ptr<State> CreateStateHome(const std::vector<double> &default_pos,
        double gain_ramp_duration,
        double move_duration,
        const std::vector<double> &kp,
        const std::vector<double> &kd) {
    auto state = std::make_unique<StateHome>();
    state->SetConfig(default_pos, gain_ramp_duration, move_duration, kp, kd);
    return state;
}

}  // namespace behavior_manager
