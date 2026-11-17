// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0

#ifndef HIKING_POLICY_ADAPTER_H
#define HIKING_POLICY_ADAPTER_H

#include "policy_adapter.h"
namespace behavior_manager::policy_adapter {
std::unique_ptr<PolicyAdapter> CreateHikingPolicyAdapter(const Config &,
    const rl_policy::PolicyExecutorConfig &);
}

#endif // HIKING_POLICY_ADAPTER_H
