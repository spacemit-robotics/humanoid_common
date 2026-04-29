/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_power_off.cpp
 * @brief 不上电状态 — 关节零力矩，自由态
 */

#include <iostream>
#include <memory>

#include "behavior_state.h"
namespace behavior_manager {

class StatePowerOff : public State {
public:
    void OnEnter() override {
        std::cout << "[StatePowerOff] 进入不上电状态" << std::endl;
        // 输出零力矩
        if (output_ && sensor_) {
            output_->enable = false;
            std::size_t ndof = sensor_->joint_pos.size();
            output_->target_pos.assign(ndof, 0.0);
            output_->target_vel.assign(ndof, 0.0);
            output_->kp.assign(ndof, 0.0);
            output_->kd.assign(ndof, 0.0);
        }
    }

    void Run(float control_dt, float rl_dt) override {
        (void)control_dt;
        (void)rl_dt;
        // 不上电：保持零力矩输出
        if (output_) {
            output_->enable = false;
        }
    }

    StateName CheckTransition() override {
        // key=1 → 进入阻尼状态
        if (command_ && command_->key == 1) {
            command_->key = 0;
            return StateName::DAMP;
        }
        return StateName::POWER_OFF;
    }

    void OnExit() override { std::cout << "[StatePowerOff] 退出不上电状态" << std::endl; }
};

// 工厂函数
std::unique_ptr<State> CreateStatePowerOff() {
    return std::make_unique<StatePowerOff>();
}

}  // namespace behavior_manager
