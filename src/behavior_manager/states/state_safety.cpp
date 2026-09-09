/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_safety.cpp
 * @brief 安全保护状态 — IMU 倾角/关节限位触发后的紧急阻尼保护
 *
 * 进入后缓慢卸力，过渡到 POWER_OFF。
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "behavior_state.h"
namespace behavior_manager {
namespace {

bool IsFiniteNonnegative(const std::vector<double> &values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value) && value >= 0.0;
    });
}

bool IsFinite(const std::vector<double> &values) {
    return std::all_of(values.begin(), values.end(),
        [](double value) { return std::isfinite(value); });
}

bool AllowsControlledRelease(const robot_base::FaultStatus *fault) {
    if (!fault || !fault->active) return false;
    if (fault->source == robot_base::FaultSource::POLICY) return true;
    return fault->source == robot_base::FaultSource::SAFETY_MONITOR &&
        fault->code == robot_base::FaultCode::LIMIT_EXCEEDED;
}

}  // namespace

class StateSafety : public State {
public:
    StateSafety(double release_duration, const robot_base::FaultStatus *fault)
        : release_duration_(release_duration), fault_(fault) {}

    void OnEnter() override {
        std::cerr << "[StateSafety] ⚠ 进入安全保护状态！" << std::endl;
        timer_ = 0;
        finished_ = false;

        const size_t num_dof = sensor_ ? sensor_->joint_pos.size() : 0;
        can_ramp_ = output_ && output_->enable && AllowsControlledRelease(fault_) &&
            num_dof > 0 && IsFinite(sensor_->joint_pos) &&
            output_->kp.size() == num_dof && output_->kd.size() == num_dof &&
            IsFiniteNonnegative(output_->kp) && IsFiniteNonnegative(output_->kd);
        hold_pos_ = can_ramp_ ? sensor_->joint_pos : std::vector<double>(num_dof, 0.0);
        kp_init_ = can_ramp_ ? output_->kp : std::vector<double>(num_dof, 0.0);
        kd_init_ = can_ramp_ ? output_->kd : std::vector<double>(num_dof, 0.0);

        if (!output_) return;
        output_->actuation_mode = robot_base::ActuationMode::HYBRID;
        output_->target_pos = hold_pos_;
        output_->target_vel.assign(num_dof, 0.0);
        output_->target_torque.assign(num_dof, 0.0);
        if (!can_ramp_) {
            output_->kp.assign(num_dof, 0.0);
            output_->kd.assign(num_dof, 0.0);
            output_->enable = false;
        }
    }

    void Run(float control_dt, float rl_dt) override {
        (void)rl_dt;
        if (!output_)
            return;

        timer_ += control_dt;

        if (can_ramp_ && timer_ < release_duration_) {
            // 阻尼阶段：逐渐减小 kp/kd，保持当前位置
            double ratio = 1.0 - timer_ / release_duration_;
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
            output_->target_pos = hold_pos_;
            output_->target_vel.assign(hold_pos_.size(), 0.0);
            output_->target_torque.assign(hold_pos_.size(), 0.0);
            output_->kp.assign(hold_pos_.size(), 0.0);
            output_->kd.assign(hold_pos_.size(), 0.0);
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
    double release_duration_ = 1.0;
    double timer_ = 0;
    bool finished_ = false;
    bool can_ramp_ = false;
    const robot_base::FaultStatus *fault_ = nullptr;
    std::vector<double> hold_pos_;  // 保持位置
    std::vector<double> kp_init_;   // 初始 kp 副本
    std::vector<double> kd_init_;   // 初始 kd 副本
};

// 工厂函数
std::unique_ptr<State> CreateStateSafety(double release_duration,
        const robot_base::FaultStatus *fault) {
    return std::make_unique<StateSafety>(release_duration, fault);
}

}  // namespace behavior_manager
