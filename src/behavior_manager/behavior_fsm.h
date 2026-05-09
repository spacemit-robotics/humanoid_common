/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file behavior_fsm.h
 * @brief 有限状态机（FSM）核心引擎
 *
 * 管理行为状态的生命周期、驱动循环、自动状态切换和强制切换逻辑。
 * 支持数据指针注入机制，将传感器、命令、输出数据传递给各状态。
 */

#ifndef BEHAVIOR_FSM_H
#define BEHAVIOR_FSM_H

#include <map>
#include <memory>
#include <string>

#include "behavior_state.h"  // 内部实现，位于 src/
#include "robot_base.h"      // robot_base::RobotData
namespace behavior_manager {

/**
 * @brief 有限状态机引擎
 *
 * 管理状态注册、step 驱动、自动切换逻辑。
 * 支持 ForceSwitch 强制切换（安全保护场景）。
 */
class FSM {
public:
    FSM();
    ~FSM();

    /**
     * @brief 注册状态
     * @param name 状态名称
     * @param state 状态实例（转移所有权）
     */
    void AddState(StateName name, std::unique_ptr<State> state);

    /**
     * @brief 替换状态（安全版本：处理"替换正在运行的当前状态"场景）
     *
     * 与 AddState 区别：若 name == 当前状态，会调用旧状态 OnExit、
     * 替换 unique_ptr、调用新状态 OnEnter，并更新 current_ 指针避免悬挂。
     * 同时直接注入数据指针（无需追加 SetDataPointers）。
     */
    void ReplaceState(StateName name, std::unique_ptr<State> state);

    /**
     * @brief 初始化 FSM，设置初始状态为 POWER_OFF
     */
    void Init();

    /**
     * @brief 执行一步
     *
     * 流程：CheckTransition → 如需切换则 OnExit/OnEnter → Run
     */
    void Step(float control_dt, float rl_dt);

    /**
     * @brief 强制切换状态（可从任意状态切入目标状态）
     * @param target 目标状态
     * @param reason 切换原因（日志输出）
     */
    void ForceSwitch(StateName target, const std::string &reason);

    /**
     * @brief 获取当前状态名
     */
    StateName CurrentState() const;

    /**
     * @brief 设置共享数据指针（所有状态共享）
     */
    void SetDataPointers(robot_base::RobotData *sensor,
                        robot_base::Command *cmd,
                        ControlOutput *output);

private:
    std::map<StateName, std::unique_ptr<State>> states_;
    State *current_ = nullptr;
    StateName current_name_;

    // 共享数据
    robot_base::RobotData *sensor_ = nullptr;
    robot_base::Command *command_ = nullptr;
    ControlOutput *output_ = nullptr;

    // 执行状态切换
    void SwitchTo(StateName target);
};

}  // namespace behavior_manager

#endif  // BEHAVIOR_FSM_H
