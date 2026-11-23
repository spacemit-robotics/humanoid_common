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
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "motion_planner/motion_planner.h"
#include "rl_service.h"
#include "robot_base.h"

namespace behavior_manager {
namespace policy_adapter {

/**
 * @brief SONIC kinematic planner 的模型与规划参数
 *
 * 机型 YAML 在 motion_planners.<name> 下以 type: sonic_planner 登记，参数由
 * LoadSonicPlannerConfig 读取。
 */
struct SonicPlannerConfig {
    bool enabled = false;
    std::string model_path;
    rl_policy::InferenceRuntimeConfig runtime;
    robot_base::ThreadLoop thread = robot_base::ThreadLoop::Background("motion_planner");
    double output_fps = 50.0;
    double check_interval = 0.1;
    double replan_interval = 1.0;
    double running_replan_interval = 0.1;
    double crawling_replan_interval = 0.2;
    double lookahead = 0.04;
    double default_height = 0.788740;
    double command_epsilon = 1.0e-3;
    double slow_walk_max_speed = 0.8;
    double walk_max_speed = 2.5;
    int mode = -1;
    int random_seed = 1234;
    std::vector<int> allowed_pred_num_tokens = {
        0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0};
    std::function<void()> on_warmup;  ///< 可选：模型加载完成、开始预热时在加载线程调用

    bool Enabled() const { return enabled && !model_path.empty(); }
};

/**
 * @brief 从 YAML 节点 base 读取 planner 参数
 * @param robot_dir 机型资源根目录，用于解析相对的 model_path
 * @param default_fps 未配置 output_fps 时使用的参考帧率
 */
SonicPlannerConfig LoadSonicPlannerConfig(const robot_base::YamlFile &yaml,
    const std::string &base,
    const std::string &robot_dir,
    double default_fps);

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

/** @brief 新规划拼接到已发布参考的结果及时序诊断 */
struct SonicPlannerBlend {
    SonicPlannerMotion motion;
    double blend_start_s = 0.0;  ///< 新旧参考开始过渡的 RL 时间
    double late_s = 0.0;         ///< 发布晚于新轨迹起点的时长
    double stale_s = 0.0;        ///< 发布前旧参考已经播完、停在末帧的时长
    int blend_frames = 0;        ///< 新旧参考实际过渡的帧数
    bool expired = false;        ///< 新轨迹已过期，motion 为空，调用方保留旧参考
};

/** @brief 按绝对 RL 时间采样参考；超出两端时取端点帧 */
SonicMotionFrame SampleSonicPlannerMotion(const SonicPlannerMotion &motion,
    double time_s, double fps);

/**
 * @brief 把新规划拼接到当前参考
 *
 * 过渡从新轨迹起点和发布时刻中较晚的一个开始，推理超过 lookahead 时
 * 仍保留完整的 blend_frames 帧过渡；剩余帧不足时延长末帧，已过期时丢弃。
 */
SonicPlannerBlend BlendSonicPlannerMotion(const SonicPlannerMotion &current,
    const SonicPlannerMotion &generated, double publish_time_s, double fps,
    int blend_frames);

/**
 * @brief 在独立线程中运行 planner，避免大模型推理阻塞 50 Hz tracker
 */
class SonicPlanner {
public:
    /** @brief Motion() 的关节按 tracker 模型顺序排列（tracker 的 action_joint_index）。 */
    SonicPlanner(const SonicPlannerConfig &config,
        const rl_policy::PolicyExecutorConfig &tracker_policy_config);

    /** @brief Motion() 的关节按机器人关节顺序排列。 */
    explicit SonicPlanner(const SonicPlannerConfig &config);
    ~SonicPlanner();

    SonicPlanner(const SonicPlanner &) = delete;
    SonicPlanner &operator=(const SonicPlanner &) = delete;

    /**
     * @brief 用当前机器人状态建立静态参考并启动 planner 线程
     * @param start_time_s 静态参考首帧对应的 RL 时间；之后的 Update 沿用同一时间轴
     */
    void Reset(const robot_base::RobotData &robot, double start_time_s = 0.0);

    /** @brief 更新最新机器人状态、速度命令和 RL 时间 */
    void Update(const robot_base::RobotData &robot,
        const robot_base::Command &command,
        double elapsed_s, int mode = -1, double height = -1);

    /** @brief 单一 tracker 线程读取只读快照，发布繁忙时沿用上次快照；Reset 不得并发 */
    std::shared_ptr<const SonicPlannerMotion> Motion() const;

    /**
     * @brief Motion() 对应的通用参考轨迹，关节按机器人关节顺序排列
     *
     * 与 Motion() 同步发布、读取规则相同；Reset 之前为空。
     */
    std::shared_ptr<const motion_planner::PlannerOutput> Reference() const;

    /** @brief Request cancellation without blocking the caller. */
    void RequestStop();

    /** @brief Stop planning and join the planner thread; Reset starts it again. */
    void Stop();

    /** @brief Last published plan is stationary and no new command is pending. */
    bool Idle() const;

    /** @brief Plans completed since Reset and the latest inference time; never blocks. */
    motion_planner::PlannerStats Stats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace policy_adapter
}  // namespace behavior_manager

#endif  // SONIC_PLANNER_H
