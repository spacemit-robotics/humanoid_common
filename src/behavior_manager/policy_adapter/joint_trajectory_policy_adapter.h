/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file joint_trajectory_policy_adapter.h
 * @brief RL 内选择性关节轨迹接管策略适配器工厂
 */

#ifndef JOINT_TRAJECTORY_POLICY_ADAPTER_H
#define JOINT_TRAJECTORY_POLICY_ADAPTER_H

#include <memory>

#include "policy_adapter.h"

namespace behavior_manager {
namespace policy_adapter {

std::unique_ptr<PolicyAdapter> CreateJointTrajectoryPolicyAdapter(
    const Config &config,
    const rl_policy::PolicyExecutorConfig &policy_config);

}  // namespace policy_adapter
}  // namespace behavior_manager

#endif  // JOINT_TRAJECTORY_POLICY_ADAPTER_H
