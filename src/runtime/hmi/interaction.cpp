/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file interaction.cpp
 * @brief HMI 交互动作目录与状态辅助实现
 */

#include "interaction.h"

#include <string>
#include <vector>

namespace hmi_runtime {

InteractionActionMap LoadInteractionActions(
    const robot_base::YamlFile &yaml,
    const std::vector<std::string> &policies) {
    InteractionActionMap actions_by_policy;
    for (const auto &policy : policies) {
        const std::string adapter_base =
            "rl_policy.onnx_infer.policies." + policy + ".policy_adapter";
        const std::string type = yaml.Read<std::string>(
            adapter_base + ".type").value_or("");
        if (type != "joint_trajectory" && type != "sonic") {
            continue;
        }
        const auto configured_catalog = yaml.Read<std::string>(
            adapter_base + ".catalog");
        if (type == "sonic" &&
            (!configured_catalog || configured_catalog->empty())) {
            continue;
        }
        const std::string catalog =
            configured_catalog.value_or(adapter_base);
        const auto action_names = yaml.Read<std::vector<std::string>>(
            catalog + ".action_names").value_or(std::vector<std::string>{});
        auto &actions = actions_by_policy[policy];
        actions.reserve(action_names.size());
        for (const auto &name : action_names) {
            actions.push_back({name,
                yaml.Read<std::string>(catalog + ".actions." + name +
                    ".display_name").value_or(name)});
        }
    }
    return actions_by_policy;
}

const std::vector<InteractionAction> *FindInteractionActions(
    const InteractionActionMap &actions_by_policy,
    const std::string &policy) {
    const auto actions = actions_by_policy.find(policy);
    return actions == actions_by_policy.end() ? nullptr : &actions->second;
}

const char *InteractionPhaseName(
        robot_base::InteractionStatus::Phase phase) {
    using Phase = robot_base::InteractionStatus::Phase;
    switch (phase) {
    case Phase::IDLE:
        return "就绪";
    case Phase::BLEND_IN:
        return "进入";
    case Phase::PLAYING:
        return "播放";
    case Phase::HOLDING:
        return "保持";
    case Phase::BLEND_OUT:
        return "收回";
    case Phase::FINISHED:
        return "完成";
    case Phase::REJECTED:
        return "拒绝";
    }
    return "未知";
}

bool InteractionIsBusy(robot_base::InteractionStatus::Phase phase) {
    using Phase = robot_base::InteractionStatus::Phase;
    return phase == Phase::BLEND_IN || phase == Phase::PLAYING ||
        phase == Phase::HOLDING || phase == Phase::BLEND_OUT;
}

}  // namespace hmi_runtime
