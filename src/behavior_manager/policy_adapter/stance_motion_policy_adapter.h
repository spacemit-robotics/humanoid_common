/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file stance_motion_policy_adapter.h
 * @brief Paired stance/motion inference with shared proprioceptive history.
 */

#ifndef STANCE_MOTION_POLICY_ADAPTER_H
#define STANCE_MOTION_POLICY_ADAPTER_H

#include <memory>
#include <string>

#include "policy_adapter.h"

namespace behavior_manager {
namespace policy_adapter {

Config::StanceMotionConfig LoadStanceMotionConfig(const std::string &yaml_path,
    const std::string &base, const std::string &robot_dir);

std::unique_ptr<PolicyAdapter> CreateStanceMotionPolicyAdapter(
    const Config &config, const rl_policy::PolicyExecutorConfig &policy_config);

}  // namespace policy_adapter
}  // namespace behavior_manager

#endif  // STANCE_MOTION_POLICY_ADAPTER_H
