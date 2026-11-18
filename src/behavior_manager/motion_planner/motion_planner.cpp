/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motion_planner.cpp
 * @brief 参考动作规划器类型注册表与输出校验
 */

#include "motion_planner.h"

#include <cmath>
#include <map>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace behavior_manager {
namespace motion_planner {
namespace {

struct Registry {
    std::mutex mutex;
    std::map<std::string, PlannerType> types;
};

// Function-local so that registrations from other translation units, which run
// during static initialization, always find a constructed registry.
Registry &GetRegistry() {
    static Registry registry;
    return registry;
}

bool AllFinite(const std::vector<float> &values) {
    for (const float value : values) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

}  // namespace

const char *CategoryName(Category category) {
    return category == Category::OFFLINE ? "offline" : "realtime";
}

const char *KindName(Kind kind) {
    return kind == Kind::BEHAVIOR_TREE ? "behavior_tree" : "model";
}

std::string ValidateOutput(const PlannerOutput &output, int num_dof) {
    const auto &trajectory = output.trajectory;
    const std::size_t frames = trajectory.FrameCount();
    if (!output.action.empty()) {
        return trajectory.joint_pos.empty() && trajectory.joint_vel.empty() && trajectory.root_pos.empty() &&
            trajectory.root_quat.empty() ? "" : "output names an action and also carries a trajectory";
    }
    if (trajectory.num_dof != num_dof) {
        return "trajectory has " + std::to_string(trajectory.num_dof) + " joints, robot has " +
            std::to_string(num_dof);
    }
    if (frames < 2) return "trajectory needs at least two frames";
    if (!std::isfinite(trajectory.fps) || trajectory.fps <= 0.0 ||
        !std::isfinite(trajectory.start_time_s)) {
        return "trajectory fps or start time is invalid";
    }
    const std::size_t dof = static_cast<std::size_t>(num_dof);
    if (trajectory.joint_pos.size() != frames * dof || trajectory.joint_vel.size() != frames * dof ||
        trajectory.root_pos.size() != frames * 3 || trajectory.root_quat.size() != frames * 4) {
        return "trajectory arrays do not describe the same number of frames";
    }
    if (!AllFinite(trajectory.joint_pos) || !AllFinite(trajectory.joint_vel) ||
        !AllFinite(trajectory.root_pos) || !AllFinite(trajectory.root_quat)) {
        return "trajectory contains a non-finite value";
    }
    for (std::size_t frame = 0; frame < frames; ++frame) {
        const float *quat = &trajectory.root_quat[frame * 4];
        const float norm_squared =
            quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3];
        if (!std::isfinite(norm_squared)) return "trajectory root quaternion norm is not finite";
        if (norm_squared < 1.0e-6F) return "trajectory contains a zero root quaternion";
    }
    return {};
}

void RegisterPlannerType(const std::string &type, PlannerType entry) {
    if (type.empty() || !entry.factory) {
        throw std::invalid_argument("[motion_planner] 规划器类型需要非空的 type 和 factory");
    }
    auto &registry = GetRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    if (!registry.types.emplace(type, std::move(entry)).second) {
        throw std::invalid_argument("[motion_planner] 规划器类型重复注册: " + type);
    }
}

const PlannerType *FindPlannerType(const std::string &type) {
    auto &registry = GetRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    const auto entry = registry.types.find(type);
    // Entries are never removed and std::map keeps node addresses stable.
    return entry == registry.types.end() ? nullptr : &entry->second;
}

std::vector<std::string> RegisteredPlannerTypes() {
    auto &registry = GetRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    std::vector<std::string> names;
    names.reserve(registry.types.size());
    for (const auto &entry : registry.types) names.push_back(entry.first);
    return names;
}

}  // namespace motion_planner
}  // namespace behavior_manager
