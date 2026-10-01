/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file terminal_ui.h
 * @brief Original HMI presentation, backed by the public operator status
 */
#ifndef TERMINAL_UI_H
#define TERMINAL_UI_H
#include "operator_types.h"
#include <string>
#include <vector>

namespace operator_terminal {
enum class Page { MAIN, POLICY_SELECT, INTERACTION_SELECT, VELOCITY };
struct View {
    Page page = Page::MAIN;
    operator_service::Status status;
    std::vector<operator_service::Policy> policies;
    operator_service::Velocity target_command;
    int active_policy_idx = 0;
    int policy_cursor_idx = 0;
    int interaction_cursor_idx = 0;
    int highlighted_key = -1;
    bool reference_start_requested = false;
    std::string transition_target;
    std::string last_action = "按 L 申请控制权";
};
void Render(const View &state);
} // namespace operator_terminal
#endif // TERMINAL_UI_H
