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

namespace behavior_manager {

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
    return config;
}

}  // namespace behavior_manager
