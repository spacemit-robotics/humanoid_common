/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_safety.cpp
 * @brief 安全保护状态 — IMU 倾角/关节限位触发后的紧急阻尼保护
 *
 * 进入后缓慢卸力，过渡到 POWER_OFF。
 */

#include <iostream>
#include <memory>
#include <vector>

#include "behavior_state.h"
namespace behavior_manager {

class StateSafety : public State {
public:
    void OnEnter() override {
        std::cerr << "[StateSafety] ⚠ 进入安全保护状态！" << std::endl;
        timer_ = 0;
        finished_ = false;

        // 记录进入时的关节位置
        if (sensor_) {
            hold_pos_ = sensor_->joint_pos;
        }

        // 保存初始 kp/kd 副本（用于线性衰减）
        if (output_) {
            kp_init_ = output_->kp;
            kd_init_ = output_->kd;
        }
    }

    void Run(float control_dt, float rl_dt) override {
        (void)rl_dt;
        if (!output_)
            return;

        timer_ += control_dt;

        if (timer_ < damp_duration_) {
            // 阻尼阶段：逐渐减小 kp/kd，保持当前位置
            double ratio = 1.0 - timer_ / damp_duration_;
            output_->enable = true;
            output_->target_pos = hold_pos_;
            output_->target_vel.assign(hold_pos_.size(), 0.0);

            // 用初始值 × ratio（线性衰减，而非累积乘法）
            output_->kp.resize(kp_init_.size());
            output_->kd.resize(kd_init_.size());
            for (size_t i = 0; i < kp_init_.size(); i++) {
                output_->kp[i] = kp_init_[i] * ratio;
            }
            for (size_t i = 0; i < kd_init_.size(); i++) {
                output_->kd[i] = kd_init_[i] * ratio;
            }
        } else {
            // 卸力完成
            output_->enable = false;
            finished_ = true;
        }
    }

    StateName CheckTransition() override {
        // 卸力完成 → POWER_OFF
        if (finished_) {
            return StateName::POWER_OFF;
        }
        return StateName::SAFETY;
    }

    void OnExit() override {
        std::cout << "[StateSafety] 退出安全保护状态 (阻尼时间: " << timer_ << "s)" << std::endl;
    }

private:
    double damp_duration_ = 1.0;  // 阻尼过渡时间 (s)
    double timer_ = 0;
    bool finished_ = false;
    std::vector<double> hold_pos_;  // 保持位置
    std::vector<double> kp_init_;   // 初始 kp 副本
    std::vector<double> kd_init_;   // 初始 kd 副本
};

// 工厂函数
std::unique_ptr<State> CreateStateSafety() {
    return std::make_unique<StateSafety>();
}

}  // namespace behavior_manager
