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
#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behavior_state.h"  // 内部实现，位于 src/
#include "robot_base.h"      // robot_base::ThreadLoop
#include "rl_service.h"      // rl_policy::ObsSegmentConfig
namespace behavior_manager {

// StateRL 的配置结构（需要在工厂函数中传入）
struct RLConfig {
    std::string model_path;                    // ONNX 模型路径
    std::vector<double> action_scale = {1.0};  // 动作缩放（标量或逐关节）
    double action_blend_ratio = 1.0;           // 动作混合比例（1.0=不混合）
    std::vector<double> rl_default_pos;        // RL 默认关节位置
    std::vector<int> action_joint_index;       // 动作到关节的索引映射
    int infer_decimation = 4;                  // 推理降频
    robot_base::ThreadLoop infer_thread_cfg;  // 推理线程配置（robot_base.threads.rl_infer）
    double max_roll = 0.7;                    // 最大翻滚角 (rad)
    double max_pitch = 0.7;                   // 最大俯仰角 (rad)

    // ---- 段式观测配置（必须） ----
    std::vector<rl_policy::ObsSegmentConfig> obs_segments;

    // ---- 观测归一化参数 ----
    double ang_vel_scale = 1.0;
    double dof_pos_scale = 1.0;
    double dof_vel_scale = 1.0;
    double euler_angle_scale = 1.0;
    std::array<double, 3> command_scale = {1.0, 1.0, 1.0};
    bool dof_pos_subtract_default = true;

    // ---- phase / gait_phase 参数 ----
    double phase_period = 0.8;
    double gait_cycle = 0.85;
    double gait_left_offset = 0.0;
    double gait_right_offset = 0.5;
    double gait_left_ratio = 0.5;
    double gait_right_ratio = 0.5;

    // ---- ref_motion_phase 参数 ----
    double motion_length = 0.0;

    // ---- 维度校验 ----
    bool strict_obs_dim_check = false;

    // ---- 自定义标量默认值（如 "z": 0.42, "stand_flag": 0.5）----
    std::unordered_map<std::string, float> custom_scalar_defaults;

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
