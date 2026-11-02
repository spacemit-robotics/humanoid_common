/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_home_limits.cpp
 * @brief Offline HOME entry regression for freely hanging out-of-range joints.
 */

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "behavior_fsm.h"
#include "state_factory.h"

namespace {

void Require(bool value) {
    if (!value) throw std::runtime_error("HOME limits regression failed");
}

void TestEntry(bool bounded) {
    auto sensor = robot_base::RobotData::Create(2);
    sensor.joint_pos = {0.269196, -0.266339};
    const auto original = sensor.joint_pos;
    robot_base::Command request;
    behavior_manager::ControlOutput output;
    behavior_manager::FSM fsm;
    fsm.AddState(behavior_manager::StateName::POWER_OFF, behavior_manager::CreateStatePowerOff());
    fsm.AddState(behavior_manager::StateName::HOME, behavior_manager::CreateStateHome(
        {0.0, 0.0}, 1.0, 3.0, {20.0, 20.0}, {1.0, 1.0},
        bounded ? std::vector<double>{-0.2618, -0.2618} : std::vector<double>{},
        bounded ? std::vector<double>{0.2618, 0.2618} : std::vector<double>{}));
    fsm.SetDataPointers(&sensor, &request, &output);
    fsm.Init();
    fsm.ForceSwitch(behavior_manager::StateName::HOME, "offline test");
    Require(output.kp == std::vector<double>({0.0, 0.0}));
    Require(output.target_pos == (bounded ? std::vector<double>{0.2618, -0.2618} : original));
    for (int step = 0; step < 2100; ++step) {
        fsm.Step(0.002f, 0.02f);
        Require(sensor.joint_pos == original);
        for (std::size_t i = 0; i < 2; ++i) {
            Require(std::isfinite(output.target_pos[i]));
            if (bounded) Require(std::abs(output.target_pos[i]) <= 0.2618);
        }
    }
    Require(output.target_pos == std::vector<double>({0.0, 0.0}));
}

}  // namespace

int main() {
    TestEntry(true);
    TestEntry(false);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const auto &lower : std::vector<std::vector<double>>{{-1.0}, {nan, -1.0}, {0.1, -1.0}, {2.0, -1.0}}) {
        bool rejected = false;
        try {
            behavior_manager::CreateStateHome({0.0, 0.0}, 1.0, 3.0, {20.0, 20.0}, {1.0, 1.0}, lower, {1.0, 1.0});
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        Require(rejected);
    }
}
