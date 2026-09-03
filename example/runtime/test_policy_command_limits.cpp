/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_policy_command_limits.cpp
 * @brief Tests application-owned per-policy velocity command limits
 */

#include <cassert>
#include <cmath>
#include <limits>

#include "policy_command_limits.h"
#include "robot_base.h"

int main(int argc, char **argv) {
    if (argc != 1 && argc != 3) return 3;
    const char *config_path = argc == 3
        ? argv[1] : POLICY_COMMAND_LIMITS_TEST_CONFIG;
    const auto yaml = robot_base::YamlFile::Load(config_path);
    const auto limits_by_policy = runtime_config::LoadPolicyCommandLimits(yaml);

    if (argc == 3) {
        return runtime_config::FindPolicyCommandLimits(
            limits_by_policy, argv[2]) ? 0 : 2;
    }

    const auto *walk = runtime_config::FindPolicyCommandLimits(
        limits_by_policy, "walk");
    assert(walk != nullptr);
    assert(std::abs(walk->min_vx + 1.5f) < 1.0e-6f);
    assert(std::abs(walk->max_vx - 1.5f) < 1.0e-6f);
    assert(std::abs(walk->min_vy + 0.5f) < 1.0e-6f);
    assert(std::abs(walk->max_vy - 0.5f) < 1.0e-6f);
    assert(std::abs(walk->min_wz + 2.2f) < 1.0e-6f);
    assert(std::abs(walk->max_wz - 2.2f) < 1.0e-6f);

    const auto *asymmetric = runtime_config::FindPolicyCommandLimits(
        limits_by_policy, "walk_asymmetric");
    assert(asymmetric != nullptr);
    assert(std::abs(asymmetric->min_vx + 0.3f) < 1.0e-6f);
    assert(std::abs(asymmetric->max_vx - 1.0f) < 1.0e-6f);
    assert(std::abs(asymmetric->min_vy + 0.25f) < 1.0e-6f);
    assert(std::abs(asymmetric->max_vy - 0.25f) < 1.0e-6f);
    assert(std::abs(asymmetric->min_wz + 0.5f) < 1.0e-6f);
    assert(std::abs(asymmetric->max_wz - 0.6f) < 1.0e-6f);
    assert(runtime_config::FindPolicyCommandLimits(
        limits_by_policy, "wave") == nullptr);

    robot_base::Command command;
    command.vx = 3.0f;
    command.vy = -2.0f;
    command.wz = std::numeric_limits<float>::quiet_NaN();
    runtime_config::ApplyPolicyCommandLimits(walk, &command);
    assert(std::abs(command.vx - 1.5f) < 1.0e-6f);
    assert(std::abs(command.vy + 0.5f) < 1.0e-6f);
    assert(command.wz == 0.0f);

    command.vx = -1.0f;
    command.vy = 0.5f;
    command.wz = -0.8f;
    runtime_config::ApplyPolicyCommandLimits(asymmetric, &command);
    assert(std::abs(command.vx + 0.3f) < 1.0e-6f);
    assert(std::abs(command.vy - 0.25f) < 1.0e-6f);
    assert(std::abs(command.wz + 0.5f) < 1.0e-6f);

    command.vx = 0.3f;
    command.vy = 0.2f;
    command.wz = -0.1f;
    runtime_config::ApplyPolicyCommandLimits(nullptr, &command);
    assert(command.vx == 0.0f && command.vy == 0.0f && command.wz == 0.0f);
    return 0;
}
