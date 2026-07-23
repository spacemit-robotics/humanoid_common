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

    // ---- motion tracking 参数（可选；motion_file 为空表示该策略不启用 tracking）----
    std::string motion_file;                    // npz 路径（绝对或相对 robot_dir）
    double motion_fps = 50.0;                   // mjlab 训练默认 50 Hz
    int anchor_body_index = -1;  // anchor body 在 npz body 顺序中的索引（由机型 yaml 提供；<0 表示未配置）
    std::vector<int> anchor_waist_joint_indices;  // pelvis→anchor 的关节索引 [yaw, roll, pitch]（机型 yaml 提供，用于 yaw 对齐）
    bool anchor_yaw_align = true;
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
