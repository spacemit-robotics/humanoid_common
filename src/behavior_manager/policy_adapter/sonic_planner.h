/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file sonic_planner.h
 * @brief SONIC Kinematic Planner 异步参考轨迹生成器
 */

#ifndef SONIC_PLANNER_H
#define SONIC_PLANNER_H

#include <array>
#include <memory>
#include <vector>

#include "policy_adapter.h"
#include "rl_service.h"
#include "robot_base.h"

namespace behavior_manager {
namespace policy_adapter {

/** @brief SONIC tracker 使用的一帧关节参考 */
struct SonicMotionFrame {
    std::vector<float> dof_pos;
    std::vector<float> dof_vel;
    std::array<float, 3> root_pos = {};
    std::array<float, 4> root_quat = {1.0F, 0.0F, 0.0F, 0.0F};
};

/** @brief 可按绝对 RL 时间采样的 planner 输出 */
struct SonicPlannerMotion {
    std::vector<SonicMotionFrame> frames;
    double start_time_s = 0.0;
};

/**
 * @brief 在独立线程中运行 planner，避免大模型推理阻塞 50 Hz tracker
 */
class SonicPlanner {
public:
    SonicPlanner(const SonicPlannerConfig &config,
        const rl_policy::PolicyExecutorConfig &tracker_policy_config);
    ~SonicPlanner();

    SonicPlanner(const SonicPlanner &) = delete;
    SonicPlanner &operator=(const SonicPlanner &) = delete;

    /** @brief 用当前机器人状态建立静态参考并启动 planner 线程 */
    void Reset(const robot_base::RobotData &robot);

    /** @brief 更新最新机器人状态、速度命令和 RL 时间 */
    void Update(const robot_base::RobotData &robot,
        const robot_base::Command &command,
        double elapsed_s);

    /** @brief 获取当前已发布参考轨迹的线程安全副本 */
    SonicPlannerMotion Motion() const;

    /** @brief Request cancellation without blocking the caller. */
    void RequestStop();

    /** @brief Last published plan is stationary and no new command is pending. */
    bool Idle() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace policy_adapter
}  // namespace behavior_manager

#endif  // SONIC_PLANNER_H
