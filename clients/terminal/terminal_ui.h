/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file terminal_ui.h
 * @brief Original HMI presentation, backed by the public operator status
 */
#ifndef TERMINAL_UI_H
#define TERMINAL_UI_H
#include "operator_types.h"
#include <algorithm>
#include <string>
#include <vector>

namespace operator_terminal {
enum class Page { MAIN, POLICY_SELECT, INTERACTION_SELECT, PLANNER_SELECT, VELOCITY };
struct View {
    Page page = Page::MAIN;
    operator_service::Status status;
    std::vector<operator_service::Policy> policies;
    std::vector<operator_service::Action> actions;
    operator_service::Velocity target_command;
    int active_policy_idx = 0;
    int policy_cursor_idx = 0;
    int interaction_cursor_idx = 0;
    int planner_cursor_idx = 0;
    int highlighted_key = -1;
    bool reference_start_requested = false;
    std::string transition_target;
    std::string last_action = "按 L 申请控制权";
};
// Catalog entry of the policy control reports as active; nullptr when it is not listed.
inline const operator_service::Policy *ActivePolicy(const View &view) {
    const auto it = std::find_if(view.policies.begin(), view.policies.end(),
        [&](const auto &policy) { return policy.name == view.status.policy; });
    return it == view.policies.end() ? nullptr : &*it;
}
// Catalog entry of the planner that currently receives velocity commands; nullptr when none runs.
inline const operator_service::Planner *RunningPlanner(const View &view) {
    const auto *policy = ActivePolicy(view);
    if (!policy || view.status.planner.phase != "RUNNING") return nullptr;
    const auto it = std::find_if(policy->planners.begin(), policy->planners.end(),
        [&](const auto &planner) { return planner.name == view.status.planner.name; });
    return it == policy->planners.end() ? nullptr : &*it;
}
// Whether the receiver of velocity commands (running planner, else active policy) has two speed presets.
inline bool HasSpeedModes(const View &view) {
    if (const auto *planner = RunningPlanner(view)) return planner->HasSpeedModes();
    const auto *policy = ActivePolicy(view);
    return policy && policy->HasSpeedModes();
}
void Render(const View &state);
} // namespace operator_terminal
#endif // TERMINAL_UI_H
