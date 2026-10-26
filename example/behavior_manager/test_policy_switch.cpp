/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_policy_switch.cpp
 * @brief Verify asynchronous policy results, rollback and FSM availability
 */

#include <chrono>
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
            "rl_policy:\n"
            "  type: onnx_infer\n"
            "  rl_dt: 0.02\n"
            "  onnx_infer:\n"
            "    default_policy: good\n"
            "    policy_names: [good, missing, other, chain]\n"
            "    policies:\n";
        for (const std::string name : {"good", "missing", "other", "chain"}) {
            yaml << "      " << name << ":\n"
                << "        model_path: " << (name == "missing"
                    ? "missing.onnx" : "identity.onnx") << "\n"
                << "        runtime: {provider: cpu}\n"
                "        model_io:\n"
                "          inputs: [{name: obs, source: observation}]\n"
                "          outputs: [{name: action, target: action}]\n"
                "        action_scale: [1.0]\n"
                "        rl_default_pos: [0, 0, 0]\n"
                "        kp: [1, 1, 1]\n"
                "        kd: [0.1, 0.1, 0.1]\n"
                "        observation:\n"
                "          segment_0_terms: [ang_vel]\n";
            if (name == "chain") {
                yaml << "        prerequisite: {policy: good, duration: 1.0}\n";
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
}  // namespace

int main() {
    try {
        Run();
        std::cout << "policy switch regression: PASS\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
