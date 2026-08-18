/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_factory.h
 * @brief 状态工厂函数声明
 *
 * 各状态的创建函数，供 BehaviorManager 注册使用。
 */

#ifndef STATE_FACTORY_H
#define STATE_FACTORY_H

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "behavior_state.h"  // 内部实现，位于 src/
#include "policy_adapter/policy_adapter.h"
#include "robot_base.h"      // robot_base::ThreadLoop
#include "rl_service.h"      // rl_policy::PolicyExecutorConfig
namespace behavior_manager {

// StateRL 的配置结构（需要在工厂函数中传入）
struct RLConfig {
    // 完整透传底层执行器配置，避免新增 RL YAML 字段时在 common 层漏传。
    rl_policy::PolicyExecutorConfig policy;
    int infer_decimation = 4;                  // 推理降频
    robot_base::ThreadLoop infer_thread_cfg;  // 推理线程配置（robot_base.threads.rl_infer）
    double max_roll = 0.7;                    // 最大翻滚角 (rad)
    double max_pitch = 0.7;                   // 最大俯仰角 (rad)

    // ---- 策略输入协议适配（可选；type 为空表示普通 RL 策略）----
    policy_adapter::Config policy_adapter;
    std::vector<double> zero_target_pos;  // ZERO 阶段目标位姿（可选，空则用 rl_default_pos）

    // ---- PD 控制增益（从 behavior_manager 传入，OnEnter 时恢复）----
    std::vector<double> kp;
    std::vector<double> kd;

    // ---- 运行时统计（由 behavior_manager 注入）----
    std::atomic<double> *rl_freq_hz = nullptr;
};

// 工厂函数
std::unique_ptr<State> CreateStatePowerOff();
std::unique_ptr<State> CreateStateDamp(const std::vector<double> &damp_kd);
std::unique_ptr<State> CreateStateZero(const std::vector<double> &default_pos,
                                        double duration,
                                        const std::vector<double> &kp,
                                        const std::vector<double> &kd);
std::unique_ptr<State> CreateStateRl(const RLConfig &cfg);
std::unique_ptr<State> CreateStateSafety();

}  // namespace behavior_manager

#endif  // STATE_FACTORY_H
