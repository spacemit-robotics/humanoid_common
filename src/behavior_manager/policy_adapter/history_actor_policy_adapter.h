/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file history_actor_policy_adapter.h
 * @brief History actor configuration and factory.
 */
#ifndef HISTORY_ACTOR_POLICY_ADAPTER_H
#define HISTORY_ACTOR_POLICY_ADAPTER_H

#include "policy_adapter.h"

namespace behavior_manager {
namespace policy_adapter {
Config::HistoryActorConfig LoadHistoryActorConfig(const std::string &yaml_path,
    const std::string &base, const std::string &robot_dir);
std::unique_ptr<PolicyAdapter> CreateHistoryActorPolicyAdapter(const Config &config,
    const rl_policy::PolicyExecutorConfig &policy_config);
}  // namespace policy_adapter
}  // namespace behavior_manager
#endif  // HISTORY_ACTOR_POLICY_ADAPTER_H
