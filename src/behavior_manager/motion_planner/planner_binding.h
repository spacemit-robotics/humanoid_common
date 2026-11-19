/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file planner_binding.h
 * @brief 跟踪策略接入规划器的通用衔接：把规划器的发布转换成参考切换
 */

#ifndef PLANNER_BINDING_H
#define PLANNER_BINDING_H

#include <memory>
#include <string>

#include "motion_planner.h"
#include "planner_host.h"
#include "robot_base.h"

namespace behavior_manager {
namespace motion_planner {

/**
 * @brief 跟踪策略提供的参考切换操作
 *
 * 跟踪策略自己持有待命参考和动作目录，并决定过渡的时长与插值方式。各函数
 * 都在跟踪策略的推理线程调用。交互动作的状态只描述操作端发起的目录动作，
 * 这里的切换不应改动它。
 *
 * FollowAction / FollowTrajectory 成功时返回空串；跟踪策略无法采用这次发布时
 * 返回原因并保持原有参考不变，规划器随即进入 FAILED，参考回到待命。
 */
class ReferenceSink {
public:
    virtual ~ReferenceSink() = default;

    /** @brief 从当前参考过渡到目录动作，并从其首帧起播。 */
    virtual std::string FollowAction(const std::string &action, double time_s) = 0;

    /**
     * @brief 跟随一段显式轨迹
     * @param finite true：过渡结束后从首帧起播，只播放一遍；false：按
     *     trajectory.start_time_s 所在的 RL 时间轴取帧，超出末帧后保持末帧
     * @param replace 正在跟随同一规划器的上一段连续轨迹：原位替换，不做过渡
     */
    virtual std::string FollowTrajectory(const ReferenceTrajectory &trajectory,
        const InstallReference &install,
        bool finite,
        bool replace,
        double time_s) = 0;

    /** @brief 从当前参考过渡回待命参考。 */
    virtual void FollowStandby(double time_s) = 0;

    /** @brief 目录动作或有限轨迹的播放进度；跟随连续轨迹时不会调用。 */
    virtual PlaybackState Playback(double time_s) const = 0;
};

/**
 * @brief 规划器宿主与一个跟踪策略的衔接
 *
 * 持有规划器宿主，每个推理步推进它，并在发布变化时调用 ReferenceSink。
 * Step() / SetRequest() / RequestRelease() 在推理线程调用且不阻塞；构造、
 * Reset() 和 Suspend() 只在推理线程未运行时调用。
 */
class PlannerBinding {
public:
    /** @brief sink 须在本对象的整个生命周期内有效。 */
    PlannerBinding(HostConfig config, ReferenceSink &sink);
    ~PlannerBinding();

    PlannerBinding(const PlannerBinding &) = delete;
    PlannerBinding &operator=(const PlannerBinding &) = delete;

    /** @brief 进入 RL 状态：跟踪策略已回到待命参考，清空请求和跟随状态。 */
    void Reset();

    /** @brief 推理线程已停止：交还参考并卸载规划器。 */
    void Suspend();

    /** @brief 记录最近一次启停请求，在下一次 Step() 中处理。 */
    void SetRequest(const robot_base::PlannerRequest &request);

    /** @brief 请求规划器交还参考（如策略切换前）。 */
    void RequestRelease();

    /**
     * @brief 推进规划器，并把新的发布交给 ReferenceSink
     * @param command 按当前规划器 command.limits 限幅后的运动指令
     * @param time_s RL 时间 (s)
     * @param tracker_busy 跟踪策略正在执行交互动作或准备切换策略
     */
    void Step(const robot_base::RobotData &robot,
        const MotionCommand &command,
        double time_s,
        bool tracker_busy);

    /** @brief 规划器占用参考：正在准备、运行或交还。期间不接受交互动作，也不切换策略。 */
    bool Engaged() const;

    /** @brief 跟踪策略正在跟随规划器的发布。 */
    bool Following() const { return following_; }

    const robot_base::PlannerStatus &Status() const;

private:
    void Release(double time_s);

    ReferenceSink &sink_;
    PlannerHost host_;
    robot_base::PlannerRequest request_;
    std::shared_ptr<const PlannerOutput> output_;  // Publication handed to the sink.
    bool following_ = false;
    bool continuous_ = false;  // Following a realtime trajectory on the RL time axis.
};

}  // namespace motion_planner
}  // namespace behavior_manager

#endif  // PLANNER_BINDING_H
