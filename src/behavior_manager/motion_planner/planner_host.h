/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file planner_host.h
 * @brief 跟踪策略内嵌的规划器宿主：按请求加载、启动、停止机型登记的规划器
 */

#ifndef PLANNER_HOST_H
#define PLANNER_HOST_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "motion_planner.h"
#include "robot_base.h"

namespace behavior_manager {
namespace motion_planner {

/** @brief 机型 YAML 在 motion_planners 下登记的一个规划器 */
struct PlannerConfig {
    std::string name;
    std::string type;
    Category category = Category::REALTIME;
    robot_base::EnvironmentSet environments;
    bool preload = false;          ///< true：构造宿主时完成加载；false：首次启动时加载
    bool keep_loaded = true;       ///< 停止后保留已加载的规划器，再次启动不重新加载
    double stop_timeout_s = 10.0;  ///< 停止时等待规划器回到静止的上限 (s)；0 表示一直等待
    /// 加载并完成首次启动的预计耗时 (s)，只用于估计上报的准备进度；0 表示未知。
    /// 同一进程内完成过一次加载后改用实测值。
    double prepare_time_s = 0.0;
    MotionCommand minimum_command;
    MotionCommand maximum_command;
    PlannerContext context;
};

using InstallReference = std::function<void()>;

/** @brief 一个跟踪策略可选用的规划器 */
struct HostConfig {
    // Worker converts the trajectory; the returned operation only swaps ready buffers on the tracker.
    std::function<InstallReference(const ReferenceTrajectory &)> prepare_trajectory;
    std::vector<PlannerConfig> planners;
    std::string auto_start;  ///< 进入策略后自动启动的实时规划器；为空时等待操作端请求
    std::map<std::string, robot_base::EnvironmentSet> action_environments;

    bool Enabled() const { return !planners.empty(); }
};

/**
 * @brief 读取策略的 policy_adapter.planners / auto_planner 及对应的 motion_planners 登记
 *
 * 校验 type 已注册、category 与注册信息一致。策略未配置规划器时返回空配置。
 */
HostConfig LoadHostConfig(const std::string &yaml_path,
    const std::string &policy_name,
    const std::string &robot_dir,
    double reference_fps);

/** @brief 跟踪策略每步提供的自身状态 */
struct TrackerState {
    bool busy = false;       ///< 正在执行交互动作或准备切换策略，此时不接受启动请求
    PlaybackState playback;  ///< 对规划器最近一次发布的播放情况
};

/**
 * @brief 规划器宿主
 *
 * 加载、启动、生成和停止在宿主的后台线程执行，期间跟踪策略继续使用自身的
 * 待命参考。Step() 等接口在跟踪策略的推理线程调用且不阻塞；构造、Reset()
 * 和 Suspend() 只在推理线程未运行时调用。
 */
class PlannerHost {
public:
    /** @brief 启动后台线程；配置了 preload 的规划器在返回前完成加载，失败时抛出异常。 */
    explicit PlannerHost(HostConfig config);
    ~PlannerHost();

    PlannerHost(const PlannerHost &) = delete;
    PlannerHost &operator=(const PlannerHost &) = delete;

    /** @brief 进入 RL 状态：清空请求序号和运行状态，已加载的规划器保留。 */
    void Reset();

    /** @brief 离开 RL 状态且推理线程已停止：交还参考并卸载规划器。 */
    void Suspend();

    /**
     * @brief 处理请求并推进规划器
     * @param command 按当前规划器 command.limits 限幅后的运动指令
     * @param time_s RL 时间 (s)
     * @param request 最近一次启停请求；序号不大于已处理序号的请求被忽略
     */
    void Step(const robot_base::RobotData &robot,
        const MotionCommand &command,
        double time_s,
        const robot_base::PlannerRequest &request,
        const TrackerState &tracker);

    /** @brief 跟踪策略应跟随的发布；为空时使用待命参考。对象不变表示没有新的发布。 */
    const std::shared_ptr<const PlannerOutput> &Output() const;

    /** @brief 当前发布来自离线规划器：只播放一遍，起播时刻由跟踪策略决定。 */
    bool OutputFinite() const;
    const InstallReference &PreparedReference() const;

    /** @brief 规划器正在准备、运行或交还参考；期间跟踪策略不接受交互动作。 */
    bool Engaged() const;

    /** @brief 请求交还参考（如策略切换前）；规划器回到静止后 Output() 变为空。 */
    void RequestRelease();

    /** @brief 跟踪策略无法使用当前发布时调用：交还参考并上报 FAILED。 */
    void Fail(const std::string &detail);

    const robot_base::PlannerStatus &Status() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace motion_planner
}  // namespace behavior_manager

#endif  // PLANNER_HOST_H
