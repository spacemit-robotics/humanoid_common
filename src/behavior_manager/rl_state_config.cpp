/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file rl_state_config.cpp
 * @brief StateRL 配置装配
 */

#include "state_factory.h"

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
