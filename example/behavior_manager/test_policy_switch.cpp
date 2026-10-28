/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_policy_switch.cpp
 * @brief Verify asynchronous policy results, rollback and FSM availability
 */

#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

#include "behavior_manager.h"
#include "rl_test_model.h"

namespace {
namespace fs = std::filesystem;
using Phase = robot_base::PolicySwitchStatus::Phase;
using State = behavior_manager::StateName;

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

class Fixture {
public:
    Fixture() {
        const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
        directory_ = fs::temp_directory_path() / ("policy_switch_" + std::to_string(nonce));
        Require(fs::create_directory(directory_), "cannot create test directory");
        std::ofstream model(directory_ / "identity.onnx", std::ios::binary);
        const auto &bytes = behavior_manager::testing::kIdentityModel;
        model.write(reinterpret_cast<const char *>(bytes), sizeof(bytes));
        Require(model.good(), "cannot write test model");
        std::ofstream yaml(Path());
        yaml << "robot_base:\n"
            "  name: policy_switch_test\n"
            "  robot_dir: .\n"
            "  num_dof: 3\n"
            "  default_joint_pos: [0, 0, 0]\n"
            "  kp: [1, 1, 1]\n"
            "  kd: [0.1, 0.1, 0.1]\n"
            "behavior_manager:\n"
            "  damp_kd: [0.1, 0.1, 0.1]\n"
            "  home: {gain_ramp_duration: 0.01, move_duration: 0.01}\n"
            "  zero: {move_duration: 0.01, settle_duration: 0.01}\n"
            "rl_policy:\n"
            "  type: onnx_infer\n"
            "  rl_dt: 0.02\n"
            "  onnx_infer:\n"
            "    default_policy: good\n"
            "    policy_names: [good, missing, other, chain, finite]\n"
            "    policies:\n";
        for (const std::string name : {"good", "missing", "other", "chain", "finite"}) {
            yaml << "      " << name << ":\n"
                << "        model_path: " << (name == "missing"
                    ? "missing.onnx" : "identity.onnx") << "\n"
                << "        runtime: {provider: cpu}\n"
                "        model_io:\n"
                "          inputs: [{name: obs, source: observation}]\n"
                "          outputs: [{name: action, target: action}]\n"
                "        action_scale: [1.0]\n"
                "        rl_default_pos: [0, 0, 0]\n"
                "        switch_group: locomotion\n"
                "        policy_switch: {transition_duration: 0.1}\n"
                << "        kp: " << (name == "other" ? "[10, 10, 10]" : "[1, 1, 1]") << "\n"
                << "        kd: " << (name == "other" ? "[2, 2, 2]" : "[0.1, 0.1, 0.1]") << "\n"
                "        observation:\n"
                << "          segment_0_terms: " << (name == "finite"
                    ? "[phase, ref_motion_phase]" : "[ang_vel]") << "\n";
            if (name == "chain") {
                yaml << "        prerequisite: {policy: good, duration: 0.1}\n";
            }
            if (name == "finite") {
                yaml << "        motion_length: 0.1\n        return_policy: good\n";
            }
        }
        Require(yaml.good(), "cannot write test config");
    }

    ~Fixture() {
        std::error_code error;
        fs::remove_all(directory_, error);
    }

    std::string Path() const { return (directory_ / "config.yaml").string(); }

private:
    fs::path directory_;
};

class Probe {
public:
    explicit Probe(const std::string &config) : manager(config) {
        manager.Init();
        sensor = robot_base::RobotData::FromYaml(config);
    }

    void Step() {
        sensor.time += 0.001;
        manager.SetSensorData(sensor);
        manager.SetCommand(command);
        manager.Step(0.001F, 0.02F);
        Require(!manager.CurrentFault().latched, "unexpected robot fault");
    }

    void Request(const std::string &policy, uint64_t sequence) {
        command = {};
        command.switch_policy = policy;
        command.policy_request_sequence = sequence;
        Step();
    }

    void Wait(Phase phase) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (manager.CurrentPolicySwitchStatus().phase == Phase::PREPARING &&
            std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            Step();
        }
        Require(manager.CurrentPolicySwitchStatus().phase == phase, "unexpected switch result");
        Require(manager.CurrentPolicySwitchStatus().sequence == command.policy_request_sequence,
            "switch sequence mismatch");
        Require(manager.CurrentPolicySwitchStatus().policy == command.switch_policy,
            "switch target mismatch");
    }

    void Key(int key, State expected) {
        command = {};
        command.key = key;
        Step();
        Require(manager.CurrentState() == expected, "FSM request was blocked");
    }

    void Tick(int count) {
        for (int i = 0; i < count; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            Step();
        }
    }

    void EnterRl() {
        Key(1, State::DAMP);
        Key(4, State::HOME);
        command = {};
        Tick(30);
        Key(2, State::ZERO);
        command = {};
        Tick(30);
        Key(3, State::RL);
        command = {};
        Tick(50);
    }

    behavior_manager::BehaviorManagerClass manager;
    robot_base::Command command;
    robot_base::RobotData sensor;
};

void Run() {
    Fixture fixture;
    Probe probe(fixture.Path());
    probe.Request("good", 1);
    probe.Wait(Phase::READY);
    probe.Key(1, State::DAMP);

    probe.Request("missing", 2);
    probe.Wait(Phase::FAILED);
    Require(!probe.manager.CurrentPolicySwitchStatus().detail.empty(), "missing error detail");
    Require(probe.manager.CurrentPolicySwitchStatus().detail.find("missing.onnx") !=
        std::string::npos, "failure did not exercise the missing-model path");
    Require(probe.manager.CurrentPolicyName() == "good", "failure replaced the active policy");
    probe.Key(4, State::HOME);
    probe.Request("other", 3);
    probe.Wait(Phase::REJECTED);
    probe.Key(1, State::DAMP);

    probe.Request("missing", 4);
    probe.Wait(Phase::FAILED);
    // Same policy, new sequence: retry without needing an intervening empty heartbeat.
    probe.Request("missing", 5);
    probe.Wait(Phase::FAILED);
    probe.Request("chain", 6);
    probe.Wait(Phase::READY);
    Require(probe.manager.CurrentPolicyName() == "good", "prerequisite was not preserved");

    // Supersede a failing load before its future is polled.
    probe.Request("missing", 7);
    probe.Request("other", 8);
    probe.Wait(Phase::READY);
    Require(probe.manager.CurrentPolicyName() == "other", "new request was lost");

    probe.Request("missing", 9);
    probe.Key(-1, State::POWER_OFF);
    Require(!probe.manager.GetOutput().enable, "power-off retained actuation");
    // Also reject combined switch + power-off commands.
    probe.command.switch_policy = "good";
    probe.command.policy_request_sequence = 10;
    probe.Step();
    probe.Wait(Phase::REJECTED);
    Require(!probe.manager.GetOutput().enable, "rejected request enabled actuation");
    probe.Request("not_in_operator_catalog", 11);
    probe.Wait(Phase::REJECTED);
    Require(probe.manager.CurrentPolicyName() == "other", "hidden policy request changed active policy");
}

void TestLiveSwitch() {
    Fixture fixture;
    Probe probe(fixture.Path());
    probe.Request("good", 1);
    probe.Wait(Phase::READY);
    probe.EnterRl();
    probe.Request("other", 2);
    probe.Wait(Phase::READY);
    Require(probe.manager.CurrentState() == State::RL, "switch left RL");
    Require(probe.manager.CurrentPolicyName() == "other", "plain policy cannot hand off");
    Require(probe.manager.GetOutput().kp[0] == 1.0, "handoff jumped to new KP before first action");
    probe.Tick(50);
    Require(probe.manager.GetOutput().kp[0] > 1.0 && probe.manager.GetOutput().kp[0] < 10.0,
        "KP did not interpolate during handoff");
    Require(probe.manager.GetOutput().kd[0] > 0.1 && probe.manager.GetOutput().kd[0] < 2.0,
        "KD did not interpolate during handoff");
    probe.Tick(100);
    Require(std::abs(probe.manager.GetOutput().kp[0] - 10.0) < 1e-8,
        "handoff did not reach target gains");

    probe.Request("missing", 3);
    probe.Wait(Phase::FAILED);
    Require(probe.manager.CurrentState() == State::RL && probe.manager.GetOutput().enable &&
        probe.manager.CurrentPolicyName() == "other", "load failure interrupted active control");

    probe.Request("missing", 30);
    probe.Request("other", 31);
    probe.Wait(Phase::READY);
    probe.Tick(50);
    Require(probe.manager.CurrentPolicyName() == "other" &&
        probe.manager.CurrentPolicySwitchStatus().phase == Phase::READY,
        "returning to the active policy did not cancel an in-flight load");

    probe.Request("chain", 4);
    int prerequisite_ticks = 0;
    for (int i = 0; i < 2000 && probe.manager.CurrentPolicyName() != "chain"; ++i) {
        if (probe.manager.CurrentPolicyName() == "good") ++prerequisite_ticks;
        probe.Tick(1);
    }
    Require(probe.manager.CurrentPolicyName() == "chain" && prerequisite_ticks >= 99,
        "prerequisite duration included preparation or previous-policy time");
    Require(probe.manager.CurrentState() == State::RL, "prerequisite handoff exited RL");
    probe.Request("finite", 5);
    probe.Wait(Phase::READY);
    Require(probe.manager.CurrentPolicyName() == "finite", "finite policy did not enter");
    for (int i = 0; i < 1000 && probe.manager.CurrentPolicyName() != "good"; ++i) probe.Tick(1);
    Require(probe.manager.CurrentPolicyName() == "good" && probe.manager.CurrentState() == State::RL,
        "finite motion did not return to the configured policy without leaving RL");
    probe.Key(-1, State::POWER_OFF);
}
}  // namespace

int main() {
    try {
        Run();
        TestLiveSwitch();
        std::cout << "policy switch regression: PASS\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
