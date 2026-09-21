/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_rl_target_limits.cpp
 * @brief Offline StateRL target limit configuration and inference regression
 */

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "behavior_fsm.h"
#include "rl_test_model.h"
#include "state_factory.h"

namespace {

namespace fs = std::filesystem;
using behavior_manager::RLConfig;
using behavior_manager::StateName;

using behavior_manager::testing::kIdentityModel;

const char kLimits[] =
    "        target_position_lower: [-0.1, -0.8, -0.5, -0.5]\n"
    "        target_position_upper: [3.05, 1.57, 0.5, 0.5]\n"
    "        target_limit_margin: 0.01\n";

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

void RequirePositions(const std::vector<double> &actual,
    const std::vector<double> &expected) {
    Require(actual.size() == expected.size(), "target dimensions changed");
    for (std::size_t i = 0; i < actual.size(); ++i) {
        Require(std::isfinite(actual[i]) && std::abs(actual[i] - expected[i]) < 1.0e-6,
            "unexpected target at joint " + std::to_string(i));
    }
}

class Fixture {
public:
    Fixture() {
        const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
        directory_ = fs::temp_directory_path() / ("rl_target_limits_" + std::to_string(nonce));
        Require(fs::create_directory(directory_), "cannot create temporary test directory");
        std::ofstream model(directory_ / "identity.onnx", std::ios::binary);
        model.write(reinterpret_cast<const char *>(kIdentityModel), sizeof(kIdentityModel));
        Require(model.good(), "cannot write test model");
    }

    ~Fixture() {
        std::error_code error;
        fs::remove_all(directory_, error);
    }

    RLConfig Load(const std::string &limits) const {
        const auto path = directory_ / "config.yaml";
        {
            std::ofstream yaml(path);
            yaml << "rl_policy:\n"
                "  type: onnx_infer\n"
                "  rl_dt: 0.02\n"
                "  onnx_infer:\n"
                "    policies:\n"
                "      probe:\n"
                "        model_path: identity.onnx\n"
                "        runtime: {provider: cpu}\n"
                "        model_io:\n"
                "          inputs: [{name: obs, source: observation}]\n"
                "          outputs: [{name: action, target: action}]\n"
                "        action_joint_index: [1, 0, 2]\n"
                "        action_scale: [0.4, 0.5, 0.0]\n"
                "        rl_default_pos: [0.2, 1.15, 0.1, -0.3]\n"
                "        kp: [1.0, 1.0, 1.0, 1.0]\n"
                "        kd: [0.1, 0.1, 0.1, 0.1]\n"
                "        observation:\n"
                "          segment_0_terms: [ang_vel]\n" << limits;
            Require(yaml.good(), "cannot write test configuration");
        }
        return behavior_manager::LoadRLStateConfig(path.string(), "probe", directory_.string());
    }

private:
    fs::path directory_;
};

void TestInvalidConfig(const Fixture &fixture) {
    const std::vector<std::string> invalid = {
        "        target_position_lower: [-1, -1, -1, -1]\n",
        "        target_position_upper: [1, 1, 1, 1]\n",
        "        target_position_lower: []\n",
        "        target_position_lower: [-1, -1]\n"
        "        target_position_upper: [1, 1]\n",
        "        target_position_lower: []\n"
        "        target_position_upper: [1, 1, 1, 1]\n",
        "        target_position_lower: invalid\n"
        "        target_position_upper: invalid\n",
        "        target_position_lower: null\n"
        "        target_position_upper: null\n",
        "        target_position_lower: [.nan, -1, -1, -1]\n"
        "        target_position_upper: [1, 1, 1, 1]\n",
        "        target_position_lower: [-1, -1, -1, -1]\n"
        "        target_position_upper: [.inf, 1, 1, 1]\n",
        "        target_position_lower: [2, -1, -1, -1]\n"
        "        target_position_upper: [1, 1, 1, 1]\n",
        "        target_position_lower: [1, -1, -1, -1]\n"
        "        target_position_upper: [1, 1, 1, 1]\n",
        "        target_position_lower: [-1, -1, -1, -1]\n"
        "        target_position_upper: [1, 1, 1, 1]\n"
        "        target_limit_margin: 1\n",
        "        target_limit_margin: -0.01\n",
        "        target_limit_margin: .nan\n",
        "        target_limit_margin: invalid\n",
        "        target_limit_margin: 0.01\n",
    };
    for (std::size_t i = 0; i < invalid.size(); ++i) {
        bool rejected = false;
        try {
            fixture.Load(invalid[i]);
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        Require(rejected, "invalid target limits accepted, case " + std::to_string(i));
    }
    auto config = fixture.Load(kLimits);
    config.target_position_upper.pop_back();
    bool rejected = false;
    try {
        behavior_manager::CreateStateRl(config);
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    Require(rejected, "direct StateRL construction accepted invalid dimensions");
}

class Session {
public:
    Session(const RLConfig &config, const std::array<double, 3> &gyro,
        const std::vector<double> &initial_target) {
        sensor_.num_dof = 4;
        sensor_.joint_pos = config.policy.rl_default_pos;
        sensor_.joint_vel.assign(4, 0.0);
        sensor_.gyro = gyro;
        sensor_.base_quat = {1.0, 0.0, 0.0, 0.0};
        fsm_.AddState(StateName::POWER_OFF, behavior_manager::CreateStatePowerOff());
        fsm_.AddState(StateName::RL, behavior_manager::CreateStateRl(config));
        fsm_.SetDataPointers(&sensor_, &command_, &output_);
        fsm_.Init();
        output_.target_pos = initial_target;
        fsm_.ForceSwitch(StateName::RL, "target limit regression");
    }

    ~Session() { fsm_.ForceSwitch(StateName::POWER_OFF, "target limit regression complete"); }

    void WaitForAction(bool expect_fault = false) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (std::chrono::steady_clock::now() < deadline) {
            fsm_.Step(0.002F, 0.02F);
            const auto fault = fsm_.CurrentFault();
            if (fault.active || fault.latched) {
                Require(expect_fault && fault.code == robot_base::FaultCode::INVALID_DATA,
                    "unexpected StateRL fault: " + fault.detail);
                return;
            }
            if (output_.enable) {
                Require(!expect_fault, "invalid target was published");
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        throw std::runtime_error("StateRL did not publish an action before the test timeout");
    }

    const std::vector<double> &Positions() const { return output_.target_pos; }

private:
    robot_base::RobotData sensor_;
    robot_base::Command command_;
    behavior_manager::ControlOutput output_;
    behavior_manager::FSM fsm_;
};

void TestMappedTargets(const RLConfig &config, const std::array<double, 3> &action,
    const std::vector<double> &expected) {
    Session session(config, action, config.policy.rl_default_pos);
    session.WaitForAction();
    RequirePositions(session.Positions(), expected);
}

}  // namespace

int main() {
    try {
        const Fixture fixture;
        TestInvalidConfig(fixture);
        const auto limited = fixture.Load(kLimits);
        Require(limited.target_limit_margin == 0.01, "YAML margin was not loaded");
        // Valid asymmetric motion, reordered output, zero scale and unmapped joint.
        TestMappedTargets(limited, {-3.0, 4.0, 7.0}, {2.2, -0.05, 0.1, -0.3});
        TestMappedTargets(limited, {-10.0, 10.0, 7.0}, {3.04, -0.79, 0.1, -0.3});
        TestMappedTargets(limited, {10.0, -10.0, -7.0}, {-0.09, 1.56, 0.1, -0.3});

        // Omitted or empty limits preserve the previous target mapping.
        TestMappedTargets(fixture.Load(""), {10.0, -10.0, 7.0}, {-4.8, 5.15, 0.1, -0.3});
        TestMappedTargets(fixture.Load(
            "        target_position_lower: []\n        target_position_upper: []\n"),
            {10.0, -10.0, 7.0}, {-4.8, 5.15, 0.1, -0.3});

        // The final bound also applies to interpolation from an out-of-range prior target.
        auto transition = limited;
        transition.entry_target_transition_duration = 1.0;
        {
            Session session(transition, {0.0, 0.0, 0.0}, {4.0, 2.0, 1.0, 1.0});
            session.WaitForAction();
            RequirePositions(session.Positions(), {3.04, 1.56, 0.49, 0.49});
        }

        // Reject arithmetic overflow without publishing a partially clamped target.
        auto overflow = limited;
        overflow.policy.action_scale[0] = std::numeric_limits<double>::max();
        {
            Session session(overflow, {10.0, 0.0, 0.0}, overflow.policy.rl_default_pos);
            session.WaitForAction(true);
            RequirePositions(session.Positions(), overflow.policy.rl_default_pos);
        }
        std::cout << "StateRL target limit regression passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
