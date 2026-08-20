/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file policy_command_limits.h
 * @brief Load and apply per-policy velocity command limits from application YAML
 */

#ifndef POLICY_COMMAND_LIMITS_H
#define POLICY_COMMAND_LIMITS_H

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot_base.h"

namespace runtime_config {

struct PolicyCommandLimits {
    float max_vx = 0.0f;
    float max_vy = 0.0f;
    float max_wz = 0.0f;

    bool Enabled() const {
        return max_vx > 0.0f || max_vy > 0.0f || max_wz > 0.0f;
    }
};

using PolicyCommandLimitMap =
    std::unordered_map<std::string, PolicyCommandLimits>;

inline PolicyCommandLimits LoadOnePolicyCommandLimits(
        const robot_base::YamlFile &yaml, const std::string &policy_name) {
    const std::string base = "rl_policy.onnx_infer.policies." + policy_name
        + ".command.limits";
    const auto max_vx = yaml.Read<double>(base + ".max_vx");
    const auto max_vy = yaml.Read<double>(base + ".max_vy");
    const auto max_wz = yaml.Read<double>(base + ".max_wz");

    const bool configured = max_vx.has_value() || max_vy.has_value()
        || max_wz.has_value();
    if (!configured) return {};
    if (!max_vx || !max_vy || !max_wz) {
        throw std::runtime_error("[runtime] 策略 " + policy_name
            + " 的 command.limits 必须同时配置 max_vx/max_vy/max_wz");
    }

    const auto validate = [&](double value, const char *name) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::runtime_error("[runtime] 策略 " + policy_name
                + " 的 command.limits." + name + " 必须为非负有限数");
        }
        return static_cast<float>(value);
    };

    return {
        validate(*max_vx, "max_vx"),
        validate(*max_vy, "max_vy"),
        validate(*max_wz, "max_wz"),
    };
}

inline PolicyCommandLimitMap LoadPolicyCommandLimits(
        const robot_base::YamlFile &yaml) {
    PolicyCommandLimitMap result;
    const auto policy_names = yaml.Read<std::vector<std::string>>(
        "rl_policy.onnx_infer.policy_names").value_or(
            std::vector<std::string>{});
    for (const auto &policy_name : policy_names) {
        const auto limits = LoadOnePolicyCommandLimits(yaml, policy_name);
        if (limits.Enabled()) result.emplace(policy_name, limits);
    }
    return result;
}

inline const PolicyCommandLimits *FindPolicyCommandLimits(
        const PolicyCommandLimitMap &limits_by_policy,
        const std::string &policy_name) {
    const auto it = limits_by_policy.find(policy_name);
    return it == limits_by_policy.end() ? nullptr : &it->second;
}

inline float ClampVelocity(float value, float limit) {
    if (!std::isfinite(value)) return 0.0f;
    return std::clamp(value, -limit, limit);
}

inline void ApplyPolicyCommandLimits(const PolicyCommandLimits *limits,
        robot_base::Command *command) {
    if (!command) return;
    if (!limits) {
        command->vx = 0.0f;
        command->vy = 0.0f;
        command->wz = 0.0f;
        return;
    }
    command->vx = ClampVelocity(command->vx, limits->max_vx);
    command->vy = ClampVelocity(command->vy, limits->max_vy);
    command->wz = ClampVelocity(command->wz, limits->max_wz);
}

}  // namespace runtime_config

#endif  // POLICY_COMMAND_LIMITS_H
