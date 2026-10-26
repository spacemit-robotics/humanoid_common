/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_trajectory.cpp
 * @brief Offline fixed-base FSM and trajectory playback regression
 */

#include <cnpy.h>

#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "behavior_manager.h"
#include "joint_trajectory.h"

namespace {

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

void CheckProfile(const std::string &path) {
    namespace bm = behavior_manager;
    using Phase = robot_base::InteractionStatus::Phase;
    const auto yaml = robot_base::YamlFile::Load(path);
    const auto directory = yaml.ToAbsPath(yaml.Read<std::string>("robot_base.robot_dir").value());
    const auto catalog = bm::joint_trajectory::LoadConfig(yaml, "behavior_manager.trajectory", directory);
    bm::BehaviorManagerClass manager(path);
    manager.Init();
    auto sensor = robot_base::RobotData::FromYaml(path);
    sensor.time = 1.0;
    robot_base::Command command;
    const auto step = [&]() {
        sensor.time += 0.002;
        manager.SetSensorData(sensor);
        manager.SetCommand(command);
        manager.Step(0.002F, 0.02F);
        const auto &output = manager.GetOutput();
        Require(!manager.CurrentFault().latched, "unexpected safety fault");
        Require(manager.CurrentPolicyName().empty() && manager.GetRlFreq() == 0.0, "RL unexpectedly active");
        Require(output.target_pos.size() == static_cast<size_t>(sensor.num_dof), "incorrect active joint count");
        for (double q : output.target_pos) Require(std::isfinite(q), "invalid command");
        // Synthetic perfect tracking; this validates software, not motor dynamics.
        if (output.enable) sensor.joint_pos = output.target_pos;
        command.key = 0;
    };
    if (yaml.Read<bool>("behavior_manager.trajectory.prepare_on_start").value_or(false)) {
        manager.Step(0.002F, 0.02F);
        Require(!manager.GetOutput().enable, "automatic preparation started without feedback");
        bool seen_home = false, seen_zero = false;
        for (int i = 0; i < 10000 && manager.CurrentState() != bm::StateName::TRAJECTORY; ++i) {
            step();
            seen_home |= manager.CurrentState() == bm::StateName::HOME;
            seen_zero |= manager.CurrentState() == bm::StateName::ZERO;
        }
        Require(seen_home && seen_zero, "automatic preparation skipped HOME or ZERO");
    } else {
        step();
        Require(!manager.GetOutput().enable, "startup must not enable motors");
        command.key = 1;
        step();
        Require(manager.CurrentState() == bm::StateName::DAMP, "DAMP unavailable");
        command.key = 4;
        step();
        for (int i = 0; i < 2500; ++i) step();
        command.key = 2;
        step();
        command.key = 3;
        step();
        Require(manager.CurrentState() == bm::StateName::ZERO, "ZERO gate was skipped");
        for (int i = 0; i < 4000 && !manager.IsZeroReady(); ++i) step();
        Require(manager.IsZeroReady(), "ZERO did not settle");
        command.key = 3;
        step();
    }
    Require(manager.CurrentState() == bm::StateName::TRAJECTORY, "trajectory unavailable");
    for (int i = 0; i < 100; ++i) step();
    Require(manager.CurrentInteractionStatus().phase == Phase::IDLE, "action started automatically");
    uint64_t sequence = 0;
    for (const auto &action : catalog.actions) {
        command.interaction = {++sequence, robot_base::InteractionRequest::Operation::START, action.name};
        step();
        Require(manager.CurrentInteractionStatus().request_accepted, "action rejected: " + action.name);
        command.interaction.operation = robot_base::InteractionRequest::Operation::NONE;
        for (int i = 0; i < 60000; ++i) {
            step();
            const auto phase = manager.CurrentInteractionStatus().phase;
            if (phase == Phase::FINISHED || phase == Phase::HOLDING) break;
        }
        const auto phase = manager.CurrentInteractionStatus().phase;
        Require(phase == Phase::FINISHED || phase == Phase::HOLDING, "action did not finish: " + action.name);
        command.interaction = {++sequence, robot_base::InteractionRequest::Operation::CANCEL, ""};
        step();
        command.interaction.operation = robot_base::InteractionRequest::Operation::NONE;
        const auto kp = manager.GetOutput().kp, kd = manager.GetOutput().kd;
        for (int i = 0; i < 20000; ++i) {
            step();
            Require(manager.GetOutput().enable && manager.GetOutput().kp == kp && manager.GetOutput().kd == kd,
                "cancel changed motor enable or gains");
            if (manager.CurrentInteractionStatus().phase != Phase::BLEND_OUT) break;
        }
        const auto ready = yaml.Read<std::vector<double>>("behavior_manager.zero_pos").value();
        for (size_t i = 0; i < ready.size(); ++i)
            Require(std::abs(manager.GetOutput().target_pos[i] - ready[i]) < 1e-6, "did not return to ready pose");
    }
    command.interaction = {++sequence, robot_base::InteractionRequest::Operation::START, catalog.actions.front().name};
    step();
    for (int i = 0; i < 25; ++i) step();
    const auto before_cancel = manager.GetOutput().target_pos;
    command.interaction = {++sequence, robot_base::InteractionRequest::Operation::CANCEL, ""};
    step();
    Require(manager.GetOutput().target_pos == before_cancel, "cancel command discontinuity");
    command.key = 1;
    step();
    Require(manager.CurrentState() == bm::StateName::DAMP, "cannot leave trajectory");
    command.key = -1;
    step();
    step();
    Require(!manager.GetOutput().enable, "POWER_OFF left motors enabled");
    std::cout << "PASS: " << catalog.actions.size() << " trajectories; no ONNX or hardware initialized\n";
}

void CheckReturnSpeed(const std::string &file) {
    namespace jt = behavior_manager::joint_trajectory;
    using Phase = robot_base::InteractionStatus::Phase;
    using Operation = robot_base::InteractionRequest::Operation;
    jt::Config config;
    config.blend_in_duration = 0;
    config.blend_out_duration = 2;
    config.return_max_velocity = 0.6;
    config.actions.push_back({"test", file, {0, 1}, 50, 1, true});
    jt::Player player(config, 2);
    const std::vector<double> ready{-2, 0};
    auto position = ready;
    player.Reset(position);
    player.Request({1, Operation::START, "test"}, 0);
    player.Apply(1, position);
    Require(player.Status().phase == Phase::HOLDING, "expected held last frame");
    const auto initial = position;
    player.Request({2, Operation::CANCEL, ""}, 1);
    position = ready;
    player.Apply(1, position);
    Require(position == initial, "cancel moved the target immediately");
    int ticks = 0;
    for (; ticks < 5000 && player.Status().phase == Phase::BLEND_OUT; ++ticks) {
        const auto previous = position;
        const double time = 1 + (ticks + 1) * 0.002;
        if (ticks % 100 == 0) player.Request({static_cast<uint64_t>(ticks + 3), Operation::CANCEL, ""}, time);
        position = ready;
        player.Apply(time, position);
        for (size_t j = 0; j < position.size(); ++j)
            Require(std::abs(position[j] - previous[j]) / 0.002 <= 0.600001, "return exceeded configured speed");
    }
    Require(player.Status().phase == Phase::FINISHED && position == ready, "repeated cancel prevented return");
    Require(ticks >= 3750 && ticks <= 3752, "unexpected return duration");
}

void CheckAutomaticStop(const std::string &path, bool inject_fault) {
    namespace bm = behavior_manager;
    bm::BehaviorManagerClass manager(path);
    manager.Init();
    auto sensor = robot_base::RobotData::FromYaml(path);
    sensor.time = 1;
    manager.SetSensorData(sensor);
    manager.Step(0.002F, 0.02F);
    Require(manager.CurrentState() == bm::StateName::HOME, "automatic HOME not entered");
    robot_base::Command command;
    if (inject_fault) {
        robot_base::FaultStatus fault;
        fault.active = fault.latched = true;
        fault.source = robot_base::FaultSource::MOTOR;
        fault.code = robot_base::FaultCode::DEVICE_ERROR;
        fault.sequence = 1;
        manager.SetSensorFault(fault);
        manager.Step(0.002F, 0.02F);
    }
    command.key = -1;
    manager.SetCommand(command);
    manager.Step(0.002F, 0.02F);
    command.key = 0;
    manager.SetCommand(command);
    manager.SetSensorFault({});
    manager.Step(0.002F, 0.02F);
    if (inject_fault) manager.AcknowledgeFault(manager.CurrentFault().sequence);
    for (int i = 0; i < 6000; ++i) {
        sensor.time += 0.002;
        manager.SetSensorData(sensor);
        manager.Step(0.002F, 0.02F);
        Require(manager.CurrentState() == bm::StateName::POWER_OFF && !manager.GetOutput().enable,
            "automatic preparation re-enabled after stop or fault");
    }
}

}  // namespace

int main(int argc, char **argv) {
    std::filesystem::path temporary;
    bool owned = false;
    try {
        if (argc == 2) {
            CheckProfile(argv[1]);
        } else {
            temporary = std::filesystem::temp_directory_path() /
                ("fixed_trajectory_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
            owned = std::filesystem::create_directory(temporary);
            Require(owned, "cannot create test directory");
            const std::vector<double> positions{0.2, -0.1, 0.4, 0.1};
            cnpy::npz_save((temporary / "action.npz").string(), "joint_pos", positions.data(), {2, 2}, "w");
            std::ofstream yaml(temporary / "config.yaml");
            yaml << "robot_base:\n  name: test\n  num_dof: 2\n  fixed_base: true\n"
                "  robot_dir: .\n  joint_names: [a, b]\n  default_joint_pos: [0, 0]\n"
                "  kp: [10, 10]\n  kd: [1, 1]\n"
                "behavior_manager:\n  damp_kd: [1, 1]\n  zero_pos: [0, 0]\n"
                "  trajectory:\n    enabled: true\n    action_names: [test]\n"
                "    actions:\n      test:\n        file: action.npz\n        joint_indices: [0, 1]\n";
            yaml.close();
            CheckProfile((temporary / "config.yaml").string());
            CheckReturnSpeed((temporary / "action.npz").string());
            std::ofstream(temporary / "config.yaml", std::ios::app)
                << "    prepare_on_start: true\n    blend_out_duration: 2.0\n    return_max_velocity: 0.6\n";
            CheckProfile((temporary / "config.yaml").string());
            CheckAutomaticStop((temporary / "config.yaml").string(), false);
            CheckAutomaticStop((temporary / "config.yaml").string(), true);
            std::filesystem::remove_all(temporary);
        }
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        if (owned) std::filesystem::remove_all(temporary);
        return 1;
    }
}
