/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_zero.cpp
 * @brief 回零位状态 — 五次多项式平滑到 RL 训练初始位置
 */

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "behavior_state.h"
namespace behavior_manager {

class StateZero : public State {
public:
    void SetDefaultPos(const std::vector<double> &pos) { default_pos_ = pos; }

    void SetDuration(double duration) { total_time_ = duration; }

    void SetKpKd(const std::vector<double> &kp, const std::vector<double> &kd) {
        kp_ = kp;
        kd_ = kd;
    }

    void OnEnter() override {
        std::cout << "[StateZero] 进入回零位状态" << std::endl;
        timer_ = 0;
        finished_ = false;

        // 恢复 kp/kd（从 POWER_OFF 清零状态恢复）
        if (output_) {
            output_->kp = kp_;
            output_->kd = kd_;
        }

        // 从当前实际关节位置开始插值，确保第一帧目标与实际位置一致
        if (sensor_) {
            init_pos_ = sensor_->joint_pos;
        }
    }

    void Run(float control_dt, float rl_dt) override {
        (void)rl_dt;
        if (!output_ || init_pos_.empty() || default_pos_.empty())
            return;

        int ndof = static_cast<int>(default_pos_.size());
        output_->target_pos.resize(ndof);
        output_->target_vel.resize(ndof);
        output_->enable = true;

        if (timer_ < total_time_) {
            // 五次多项式插值: s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5
            double s = timer_ / total_time_;
            double alpha = 10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s;
            double alpha_dot = (30 * s * s - 60 * s * s * s + 30 * s * s * s * s) / total_time_;

            for (int i = 0; i < ndof; i++) {
                double p0 = (i < static_cast<int>(init_pos_.size())) ? init_pos_[i] : 0.0;
                double p1 = default_pos_[i];
                output_->target_pos[i] = p0 + alpha * (p1 - p0);
                output_->target_vel[i] = alpha_dot * (p1 - p0);
            }
        } else {
            // 到位
            finished_ = true;
            for (int i = 0; i < ndof; i++) {
                output_->target_pos[i] = default_pos_[i];
                output_->target_vel[i] = 0;
            }
        }

        timer_ += control_dt;
    }

    StateName CheckTransition() override {
        // key=3 且已到位 → RL 控制
        if (command_ && command_->key == 3 && finished_) {
            command_->key = 0;
            return StateName::RL;
        }
        // key=1 → 退回阻尼状态
        if (command_ && command_->key == 1) {
            command_->key = 0;
            return StateName::DAMP;
        }
        // key=-1 → 完全失力
        if (command_ && command_->key == -1) {
            command_->key = 0;
            return StateName::POWER_OFF;
        }
        return StateName::ZERO;
    }

    void OnExit() override {
        std::cout << "[StateZero] 退出回零位状态 (用时: " << timer_ << "s)" << std::endl;
    }

private:
    std::vector<double> default_pos_;  // RL 训练初始位置
    std::vector<double> init_pos_;     // 运动起始位置
    std::vector<double> kp_;           // PD 增益（OnEnter 时恢复）
    std::vector<double> kd_;
    double total_time_ = 2.0;  // 插值总时间 (s)
    double timer_ = 0;
    bool finished_ = false;
};

// 工厂函数（带默认位置参数）
std::unique_ptr<State> CreateStateZero(const std::vector<double> &default_pos,
                                        double duration,
                                        const std::vector<double> &kp,
                                        const std::vector<double> &kd) {
    auto s = std::make_unique<StateZero>();
    s->SetDefaultPos(default_pos);
    s->SetDuration(duration);
    s->SetKpKd(kp, kd);
    return s;
}

}  // namespace behavior_manager
