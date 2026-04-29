/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file behavior_state.h
 * @brief 行为状态基类与接口定义
 *
 * 定义行为状态的抽象基类 State，规定所有具体状态（POWER_OFF、DAMP、ZERO、RL、SAFETY）
 * 的生命周期接口（OnEnter、Run、CheckTransition、OnExit）和数据指针注入机制。
 */

#ifndef BEHAVIOR_STATE_H
#define BEHAVIOR_STATE_H

#include "../include/behavior_manager.h"

namespace behavior_manager {

// ==================== 状态基类 ====================

class State {
public:
    virtual ~State() = default;

    // 进入状态
    virtual void OnEnter() = 0;
    // 状态执行（每控制周期调用）
    virtual void Run(float control_dt, float rl_dt) = 0;
    // 检查状态切换条件
    virtual StateName CheckTransition() = 0;
    // 退出状态
    virtual void OnExit() = 0;

    StateName name;

protected:
    robot_base::RobotData *sensor_ = nullptr;  // 传感器数据指针（由 FSM 注入）
    robot_base::Command *command_ = nullptr;   // 命令数据指针
    ControlOutput *output_ = nullptr;          // 控制输出指针

    friend class FSM;  // FSM 负责注入数据指针
};

}  // namespace behavior_manager

#endif  // BEHAVIOR_STATE_H
