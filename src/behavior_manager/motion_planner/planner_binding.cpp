/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file planner_binding.cpp
 * @brief 规划器宿主与跟踪策略的衔接实现
 */

#include "planner_binding.h"

#include <utility>

#include "planner_catalog.h"

namespace behavior_manager {
namespace motion_planner {

PlannerBinding::PlannerBinding(HostConfig config, ReferenceSink &sink)
    : sink_(sink), host_(std::move(config)) {
    // Sized once, so copying a request on the inference thread does not allocate.
    request_.planner.reserve(runtime_config::kPlannerNameMaxLength);
    request_.argument.reserve(robot_base::kPlannerArgumentMaxLength);
}

PlannerBinding::~PlannerBinding() = default;

void PlannerBinding::Reset() {
    following_ = false;
    continuous_ = false;
    output_.reset();
    request_.sequence = 0;
    request_.operation = robot_base::PlannerRequest::Operation::NONE;
    request_.planner.clear();
    request_.argument.clear();
    host_.Reset();
}

void PlannerBinding::Suspend() { host_.Suspend(); }

void PlannerBinding::SetRequest(const robot_base::PlannerRequest &request) { request_ = request; }

void PlannerBinding::RequestRelease() { host_.RequestRelease(); }

bool PlannerBinding::Engaged() const { return following_ || host_.Engaged(); }

const robot_base::PlannerStatus &PlannerBinding::Status() const { return host_.Status(); }

void PlannerBinding::Step(const robot_base::RobotData &robot,
        const MotionCommand &command,
        double time_s,
        bool tracker_busy) {
    TrackerState tracker;
    tracker.busy = tracker_busy;
    if (following_) {
        if (!continuous_) tracker.playback = sink_.Playback(time_s);
        tracker.playback.active = true;
    }
    host_.Step(robot, command, time_s, request_, tracker);

    const std::shared_ptr<const PlannerOutput> output = host_.Output();
    if (!output) {
        output_.reset();
        if (following_) Release(time_s);
        return;
    }
    if (output == output_) return;
    output_ = output;

    const bool named = !output->action.empty();
    const bool finite = host_.OutputFinite();
    // Consecutive plans of a realtime planner replace each other in place; the
    // planner blends them itself.
    const bool replace = !named && !finite && following_ && continuous_;
    const std::string error = named ? sink_.FollowAction(output->action, time_s)
        : sink_.FollowTrajectory(output->trajectory, host_.PreparedReference(), finite, replace, time_s);
    if (!error.empty()) {
        host_.Fail("the tracker cannot follow the published reference: " + error);
        output_.reset();
        if (following_) Release(time_s);
        return;
    }
    following_ = true;
    continuous_ = !named && !finite;
}

void PlannerBinding::Release(double time_s) {
    following_ = false;
    continuous_ = false;
    sink_.FollowStandby(time_s);
}

}  // namespace motion_planner
}  // namespace behavior_manager
