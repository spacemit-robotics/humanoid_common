/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file rl_state_config.cpp
 * @brief StateRL 配置装配
 */

#include "state_factory.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace behavior_manager {

void ValidateRLTargetPositionLimits(const RLConfig &config) {
    const std::string prefix = "[BehaviorManager] policy '" +
        config.policy_name + "' target position limits: ";
    const double margin = config.target_limit_margin;
    if (!std::isfinite(margin) || margin < 0.0) {
        throw std::runtime_error(prefix + "target_limit_margin must be finite and nonnegative");
    }
    const auto &lower = config.target_position_lower;
    const auto &upper = config.target_position_upper;
    if (lower.empty() && upper.empty() && margin == 0.0) return;
    const std::size_t num_dof = config.policy.rl_default_pos.size();
    if (num_dof == 0 || lower.size() != num_dof || upper.size() != num_dof) {
        throw std::runtime_error(prefix +
            "target_position_lower/upper must both match rl_default_pos dimensions");
    }
    for (std::size_t i = 0; i < num_dof; ++i) {
        const double safe_lower = lower[i] + margin;
        const double safe_upper = upper[i] - margin;
        if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
            !std::isfinite(safe_lower) || !std::isfinite(safe_upper) ||
            safe_lower >= safe_upper) {
            throw std::runtime_error(prefix + "invalid range or margin at joint " +
                std::to_string(i));
        }
    }
}

RLConfig LoadRLStateConfig(const std::string &yaml_path,
    const std::string &policy_name,
    const std::string &robot_dir) {
    const rl_policy::LoadedPolicyConfig loaded =
        rl_policy::LoadPolicyConfigFromYaml(yaml_path, policy_name, robot_dir);

    RLConfig config;
    config.policy = loaded.exec_cfg;
    config.policy_name = policy_name;
    config.command_init = loaded.command_init;
    config.rl_dt = loaded.rl_dt;
    config.infer_decimation =
        loaded.infer_decimation > 0 ? loaded.infer_decimation : 1;
    config.max_roll = loaded.max_roll;
    config.max_pitch = loaded.max_pitch;
    config.kp = loaded.kp;
    config.kd = loaded.kd;

    const auto yaml = robot_base::YamlFile::Load(yaml_path);
    const std::string base =
        "rl_policy.onnx_infer.policies." + policy_name;
    const std::string safety_base = base + ".runtime_safety";
    config.first_action_timeout_s = yaml.Read<double>(
        safety_base + ".first_action_timeout_s").value_or(
            yaml.Read<double>(
                "behavior_manager.rl_safety.first_action_timeout_s").value_or(0.0));
    config.max_action_age_s = yaml.Read<double>(
        safety_base + ".max_action_age_s").value_or(
            yaml.Read<double>(
                "behavior_manager.rl_safety.max_action_age_s").value_or(0.0));
    config.inference_deadline_s = yaml.Read<double>(
        safety_base + ".inference_deadline_s").value_or(
            yaml.Read<double>(
                "behavior_manager.rl_safety.inference_deadline_s").value_or(0.0));
    const std::array<double, 3> runtime_limits = {
        config.first_action_timeout_s, config.max_action_age_s,
        config.inference_deadline_s};
    for (double limit : runtime_limits) {
        if (!std::isfinite(limit) || limit < 0.0) {
            throw std::runtime_error(
                "[BehaviorManager] " + safety_base + " 时限配置无效");
        }
    }
    config.policy_adapter =
        policy_adapter::LoadConfig(yaml_path, policy_name, robot_dir);
    config.zero_target_pos =
        yaml.Read<std::vector<double>>(base + ".zero_target_pos")
            .value_or(std::vector<double>{});
    config.entry_target_transition_duration =
        yaml.Read<double>(base + ".entry_target_transition_duration")
            .value_or(0.0);
    if (!std::isfinite(config.entry_target_transition_duration) ||
        config.entry_target_transition_duration < 0.0) {
        throw std::runtime_error(
            "[BehaviorManager] " + base +
            ".entry_target_transition_duration 配置无效");
    }

    // YamlFile::Read treats conversion errors as missing fields. Safety limits
    // require strict parsing so malformed values cannot disable protection.
    try {
        const auto policy = YAML::LoadFile(yaml_path)["rl_policy"]
            ["onnx_infer"]["policies"][policy_name];
        const auto lower = policy["target_position_lower"];
        const auto upper = policy["target_position_upper"];
        if (static_cast<bool>(lower) != static_cast<bool>(upper)) {
            throw std::runtime_error("[BehaviorManager] " + base +
                ": target_position_lower and target_position_upper must be configured together");
        }
        if (lower) {
            config.target_position_lower = lower.as<std::vector<double>>();
            config.target_position_upper = upper.as<std::vector<double>>();
        }
        if (policy["target_limit_margin"]) {
            config.target_limit_margin = policy["target_limit_margin"].as<double>();
        }
    } catch (const YAML::Exception &error) {
        throw std::runtime_error("[BehaviorManager] " + base +
            ": invalid target position limits: " + error.what());
    }
    ValidateRLTargetPositionLimits(config);
    return config;
}

}  // namespace behavior_manager
