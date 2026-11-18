/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file planner_catalog.h
 * @brief Load the reference-motion planners that tracker policies accept
 *
 * Planners are registered once under the top-level motion_planners node and
 * referenced by name from policy_adapter.planners of each tracker policy.
 * Control and the operator service read the same nodes through this header.
 */

#ifndef PLANNER_CATALOG_H
#define PLANNER_CATALOG_H

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "policy_availability.h"
#include "policy_command_limits.h"
#include "robot_base.h"

namespace runtime_config {

/** @brief Longest planner name the transport packets can carry. */
constexpr std::size_t kPlannerNameMaxLength = 63;

/** @brief Fields of one registered planner that are independent of its type. */
struct PlannerEntry {
    std::string name;
    std::string display_name;
    std::string type;
    std::string category;  ///< "realtime" or "offline"
    robot_base::EnvironmentSet environments;
    PolicyCommandLimits limits;  ///< Velocity command range while the planner runs
    std::string base;            ///< YAML node of this planner
};

inline std::string PlannerBase(const std::string &name) {
    return "motion_planners." + name;
}

inline std::string PolicyPlannerList(const std::string &policy) {
    return PolicyBase(policy) + ".policy_adapter.planners";
}

/** @brief Planners of one tracker policy, in the order the policy lists them. */
inline std::vector<PlannerEntry> LoadPolicyPlanners(
        const robot_base::YamlFile &yaml, const std::string &policy) {
    std::vector<PlannerEntry> result;
    const std::string list = PolicyPlannerList(policy);
    const auto names = yaml.Read<std::vector<std::string>>(list);
    if (!names) {
        if (yaml.Read<std::string>(list)) {
            throw std::runtime_error("[runtime] " + list + " 必须是规划器名组成的列表");
        }
        return result;
    }
    for (const auto &name : *names) {
        PlannerEntry entry;
        entry.name = name;
        entry.base = PlannerBase(name);
        const auto type = yaml.Read<std::string>(entry.base + ".type");
        if (name.empty() || name.size() > kPlannerNameMaxLength || !type || type->empty()) {
            throw std::runtime_error("[runtime] " + list + " 引用的规划器未在 motion_planners 下登记 type: "
                + name);
        }
        if (std::any_of(result.begin(), result.end(),
                [&](const PlannerEntry &other) { return other.name == name; })) {
            throw std::runtime_error("[runtime] " + list + " 重复引用规划器: " + name);
        }
        entry.type = *type;
        entry.display_name = yaml.Read<std::string>(entry.base + ".display_name").value_or(name);
        entry.category = yaml.Read<std::string>(entry.base + ".category").value_or("realtime");
        if (entry.category != "realtime" && entry.category != "offline") {
            throw std::runtime_error("[runtime] " + entry.base + ".category 只支持 realtime 和 offline");
        }
        entry.environments = robot_base::EnvironmentSet::FromYaml(yaml, entry.base + ".enabled_in");
        entry.limits = LoadCommandLimits(yaml, entry.base, "规划器 " + name);
        result.push_back(std::move(entry));
    }
    return result;
}

/** @brief Velocity command limits of every planner referenced by a selectable policy. */
inline PolicyCommandLimitMap LoadPlannerCommandLimits(const robot_base::YamlFile &yaml) {
    PolicyCommandLimitMap result;
    for (const auto &policy : yaml.Read<std::vector<std::string>>(
            "rl_policy.onnx_infer.policy_names").value_or(std::vector<std::string>{})) {
        for (const auto &planner : LoadPolicyPlanners(yaml, policy)) {
            if (planner.limits.Enabled()) result.emplace(planner.name, planner.limits);
        }
    }
    return result;
}

}  // namespace runtime_config

#endif  // PLANNER_CATALOG_H
