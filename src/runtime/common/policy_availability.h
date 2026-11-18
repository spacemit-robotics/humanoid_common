/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file policy_availability.h
 * @brief Load the runtime environments in which policies and their actions are enabled
 *
 * Control enforces these sets and the operator service filters its catalog with
 * them, so both read the same YAML nodes through this header.
 */

#ifndef POLICY_AVAILABILITY_H
#define POLICY_AVAILABILITY_H

#include <map>
#include <string>
#include <vector>

#include "robot_base.h"

namespace runtime_config {

using ActionEnvironmentMap = std::map<std::string, robot_base::EnvironmentSet>;

inline bool ActionEnabled(const ActionEnvironmentMap &actions, const std::string &name,
        robot_base::RuntimeEnvironment environment) {
    const auto entry = actions.find(name);
    return entry == actions.end() || entry->second.Allows(environment);
}

inline std::string PolicyBase(const std::string &policy) {
    return "rl_policy.onnx_infer.policies." + policy;
}

/** @brief enabled_in of one policy entry; absent means every environment. */
inline robot_base::EnvironmentSet LoadPolicyEnvironments(
        const robot_base::YamlFile &yaml, const std::string &policy) {
    return robot_base::EnvironmentSet::FromYaml(yaml, PolicyBase(policy) + ".enabled_in");
}

/**
 * @brief YAML node holding the interactive actions of a policy
 *
 * policy_actions owned by the policy entry take precedence over the adapter
 * catalog. Returns an empty string when the policy has no action catalog.
 */
inline std::string PolicyActionCatalog(
        const robot_base::YamlFile &yaml, const std::string &policy) {
    const std::string base = PolicyBase(policy);
    if (yaml.Read<std::vector<std::string>>(base + ".policy_actions.action_names")) {
        return base + ".policy_actions";
    }
    const std::string adapter = base + ".policy_adapter";
    const auto type = yaml.Read<std::string>(adapter + ".type").value_or("");
    const auto configured = yaml.Read<std::string>(adapter +
        (type == "stance_motion" ? ".joint_trajectory.catalog" : ".catalog"));
    if (type == "joint_trajectory") return configured.value_or(adapter);
    if ((type == "sonic" || type == "supermimic" || type == "stance_motion" ||
            type == "history_actor") && configured && !configured->empty()) {
        return *configured;
    }
    return {};
}

/** @brief enabled_in of every action listed by <catalog>.action_names. */
inline ActionEnvironmentMap LoadActionEnvironments(
        const robot_base::YamlFile &yaml, const std::string &catalog) {
    ActionEnvironmentMap result;
    if (catalog.empty()) return result;
    for (const auto &name : yaml.Read<std::vector<std::string>>(catalog + ".action_names")
            .value_or(std::vector<std::string>{})) {
        result[name] = robot_base::EnvironmentSet::FromYaml(
            yaml, catalog + ".actions." + name + ".enabled_in");
    }
    return result;
}

/** @brief enabled_in of the actions of the standalone TRAJECTORY state. */
inline ActionEnvironmentMap LoadTrajectoryActionEnvironments(const robot_base::YamlFile &yaml) {
    const auto file = yaml.Read<std::string>("behavior_manager.trajectory.catalog_file");
    const auto catalog = yaml.Read<std::string>("behavior_manager.trajectory.catalog")
        .value_or("behavior_manager.trajectory");
    return LoadActionEnvironments(
        file ? robot_base::YamlFile::Load(yaml.ToAbsPath(*file)) : yaml, catalog);
}

}  // namespace runtime_config

#endif  // POLICY_AVAILABILITY_H
