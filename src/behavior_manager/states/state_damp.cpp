/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_damp.cpp
 * @brief 阻尼状态 — kp=0, kd=配置值，软性支撑过渡
 *
 * 作为上电后的默认安全态：关节有阻尼力（抵抗速度），但无位置刚度。
 * 机器人处于可手动推动的"软支撑"状态，不会像 POWER_OFF 那样完全瘫软。
 */

#include <iostream>
#include <memory>
#include <vector>

#include "behavior_state.h"
namespace behavior_manager {

class StateDamp : public State {
public:
    void SetDampKd(const std::vector<double> &damp_kd) { damp_kd_ = damp_kd; }

    void OnEnter() override {
        std::cout << "[StateDamp] 进入阻尼状态 (kp=0, kd=damp_kd)" << std::endl;
        if (output_ && sensor_) {
            int ndof = static_cast<int>(sensor_->joint_pos.size());
            // kp=0：无位置刚度，可手动推动
            output_->kp.assign(ndof, 0.0);
            // kd=damp_kd：阻尼力，抵抗速度，防止完全瘫软
            if (!damp_kd_.empty()) {
                output_->kd = damp_kd_;
            } else {
                output_->kd.assign(ndof, 2.0);
            }
            output_->target_pos = sensor_->joint_pos;
            output_->target_vel.assign(ndof, 0.0);
            output_->enable = true;
        }
    }

    void Run(float control_dt, float rl_dt) override {
        (void)control_dt;
        (void)rl_dt;
        if (output_) {
            output_->enable = true;
        }
    }

    StateName CheckTransition() override {
        // key=2 → 回零位
        if (command_ && command_->key == 2) {
            command_->key = 0;
            return StateName::ZERO;
        }
        // key=-1 → 完全失力
        if (command_ && command_->key == -1) {
            command_->key = 0;
            return StateName::POWER_OFF;
        }
        return StateName::DAMP;
    }

    void OnExit() override { std::cout << "[StateDamp] 退出阻尼状态" << std::endl; }

private:
    std::vector<double> damp_kd_;
};

// 工厂函数
std::unique_ptr<State> CreateStateDamp(const std::vector<double> &damp_kd) {
    auto s = std::make_unique<StateDamp>();
    s->SetDampKd(damp_kd);
    return s;
}

}  // namespace behavior_manager
