/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file behavior_fsm.cpp
 * @brief FSM 状态机引擎实现
 *
 * 管理状态注册、step 驱动、自动切换和强制切换。
 */

#include "behavior_fsm.h"

#include <iostream>
#include <stdexcept>
#include <memory>
#include <string>
#include <utility>

namespace behavior_manager {

FSM::FSM() : current_name_(StateName::POWER_OFF) {}

FSM::~FSM() = default;

void FSM::AddState(StateName name, std::unique_ptr<State> state) {
    state->name = name;
    states_[name] = std::move(state);
}

void FSM::ReplaceState(StateName name, std::unique_ptr<State> state) {
    state->name = name;
    state->sensor_ = sensor_;
    state->command_ = command_;
    state->output_ = output_;
    if (name == current_name_ && current_) {
        current_->OnExit();
        states_[name] = std::move(state);
        current_ = states_[name].get();
        current_->OnEnter();
    } else {
        states_[name] = std::move(state);
    }
}

void FSM::SetDataPointers(robot_base::RobotData *sensor,
                        robot_base::Command *cmd,
                        ControlOutput *output) {
    sensor_ = sensor;
    command_ = cmd;
    output_ = output;

    // 注入到所有已注册状态
    for (auto &[name, state] : states_) {
        state->sensor_ = sensor;
        state->command_ = cmd;
        state->output_ = output;
    }
}

void FSM::Init() {
    if (states_.empty()) {
        throw std::runtime_error("[FSM] 未注册任何状态");
    }

    if (states_.find(StateName::POWER_OFF) == states_.end()) {
        throw std::runtime_error("[FSM] 必须注册 POWER_OFF 状态");
    }

    current_name_ = StateName::POWER_OFF;
    current_ = states_[current_name_].get();
    current_->OnEnter();

    std::cout << "[FSM] 初始化完成，当前状态: " << StateNameStr(current_name_) << std::endl;
}

void FSM::Step(float control_dt, float rl_dt) {
    if (!current_)
        return;

    // 检查状态切换
    StateName next = current_->CheckTransition();
    if (next != current_name_) {
        SwitchTo(next);
    }

    // 执行当前状态
    current_->Run(control_dt, rl_dt);
}

void FSM::ForceSwitch(StateName target, const std::string &reason) {
    if (target == current_name_)
        return;

    std::cout << "[FSM] 强制切换: " << StateNameStr(current_name_) << " → " << StateNameStr(target)
            << " (原因: " << reason << ")" << std::endl;

    SwitchTo(target);
}

StateName FSM::CurrentState() const {
    return current_name_;
}

void FSM::SwitchTo(StateName target) {
    auto it = states_.find(target);
    if (it == states_.end()) {
        std::cerr << "[FSM] 目标状态未注册: " << StateNameStr(target) << std::endl;
        return;
    }

    std::cout << "[FSM] 切换: " << StateNameStr(current_name_) << " → " << StateNameStr(target)
            << std::endl;

    if (current_) {
        current_->OnExit();
    }

    current_name_ = target;
    current_ = it->second.get();
    current_->OnEnter();
}

}  // namespace behavior_manager
