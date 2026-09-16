/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file interaction.h
 * @brief HMI 交互动作目录与状态辅助接口
 */

#ifndef INTERACTION_H
#define INTERACTION_H

#include <string>
#include <unordered_map>
#include <vector>

#include "robot_base.h"

namespace hmi_runtime {

struct InteractionAction {
    std::string key;
    std::string display_name;
};

using InteractionActionMap =
    std::unordered_map<std::string, std::vector<InteractionAction>>;

InteractionActionMap LoadInteractionActions(
    const robot_base::YamlFile &yaml,
    const std::vector<std::string> &policies);

const std::vector<InteractionAction> *FindInteractionActions(
    const InteractionActionMap &actions_by_policy,
    const std::string &policy);

const char *InteractionPhaseName(
    robot_base::InteractionStatus::Phase phase);

bool InteractionIsBusy(robot_base::InteractionStatus::Phase phase);

}  // namespace hmi_runtime

#endif  // INTERACTION_H
