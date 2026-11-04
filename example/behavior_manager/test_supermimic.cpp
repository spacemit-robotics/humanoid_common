/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_supermimic.cpp
 * @brief Exercise the static SuperMimic contract through the real executor.
 */

#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "policy_adapter/policy_adapter.h"

namespace {

using behavior_manager::policy_adapter::Create;
using behavior_manager::policy_adapter::LoadConfig;

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

void Near(double actual, double expected, const std::string &message) {
    Require(std::isfinite(actual) && std::abs(actual - expected) < 1e-6, message);
}

void Reject(const std::function<void()> &operation, const std::string &message) {
    bool rejected = false;
    try {
        operation();
    } catch (const std::exception &) {
        rejected = true;
    }
    Require(rejected, message);
}

// Opset 13 / IR 8. Gather the newest q-offset frame as actions and expose
// both input tensors, so tests inspect what ONNX actually receives.
const char kProbe[] =
    "CAg6mgQKMQoIb2JzX2hpc3QKB2luZGljZXMSB2FjdGlvbnMiBkdhdGhlcioLCgRheGlzGAGg"
    "AQIKGwoEZ29hbBIJc2Vlbl9nb2FsIghJZGVudGl0eQomCghvYnNfaGlzdBIQc2Vlbl9vYnNl"
    "cnZhdGlvbiIISWRlbnRpdHkSGXN1cGVybWltaWNfY29udHJhY3RfcHJvYmUq8AEIHBAHQgdp"
    "bmRpY2VzSuAB/AAAAAAAAAD9AAAAAAAAAP4AAAAAAAAA/wAAAAAAAAAAAQAAAAAAAAEBAAAA"
    "AAAAAgEAAAAAAAADAQAAAAAAAAQBAAAAAAAABQEAAAAAAAAGAQAAAAAAAAcBAAAAAAAACAEA"
    "AAAAAAAJAQAAAAAAAAoBAAAAAAAACwEAAAAAAAAMAQAAAAAAAA0BAAAAAAAADgEAAAAAAAAP"
    "AQAAAAAAABABAAAAAAAAEQEAAAAAAAASAQAAAAAAABMBAAAAAAAAFAEAAAAAAAAVAQAAAAAA"
    "ABYBAAAAAAAAFwEAAAAAAABaGwoIb2JzX2hpc3QSDwoNCAESCQoCCAEKAwiEB1oXCgRnb2Fs"
    "Eg8KDQgBEgkKAggBCgMIlAViGQoHYWN0aW9ucxIOCgwIARIICgIIAQoCCBxiHAoJc2Vlbl9n"
    "b2FsEg8KDQgBEgkKAggBCgMIlAViIwoQc2Vlbl9vYnNlcnZhdGlvbhIPCg0IARIJCgIIAQoD"
    "CIQHQgQKABAN";

class Fixture {
public:
    Fixture() {
        char path[] = "/tmp/supermimic_adapter_XXXXXX";
        const char *directory = mkdtemp(path);
        Require(directory != nullptr, "cannot create fixture directory");
        directory_ = directory;
        model = (directory_ / "probe.onnx").string();
        yaml = (directory_ / "config.yaml").string();
        std::ofstream output(model, std::ios::binary);
        const std::string alphabet =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        unsigned int value = 0;
        int bits = -8;
        for (char character : std::string(kProbe)) {
            const auto code = alphabet.find(character);
            Require(code != std::string::npos, "invalid embedded probe");
            value = (value << 6) | static_cast<unsigned int>(code);
            bits += 6;
            if (bits >= 0) {
                output.put(static_cast<char>((value >> bits) & 255));
                bits -= 8;
            }
        }
        Require(output.good(), "cannot write probe");
    }

    ~Fixture() {
        std::error_code error;
        std::filesystem::remove_all(directory_, error);
    }

    void WriteConfig(const std::string &height = "0.95",
        const std::string &period = "0.02", int decimation = 1) const {
        std::ofstream output(yaml);
        output << "rl_policy:\n  rl_dt: " << period <<
            "\n  onnx_infer:\n    policies:\n      probe:\n"
            "        infer_decimation: " << decimation <<
            "\n        policy_adapter:\n          type: supermimic\n";
        if (!height.empty()) output << "          reference_height: " << height << '\n';
        Require(output.good(), "cannot write configuration");
    }

    std::string model;
    std::string yaml;

private:
    std::filesystem::path directory_;
};

rl_policy::PolicyExecutorConfig Policy(const std::string &model) {
    rl_policy::PolicyExecutorConfig policy;
    policy.model_path = model;
    policy.runtime.provider = "cpu";
    policy.runtime.threads = 1;
    policy.rl_default_pos.assign(28, 0.1);
    policy.action_joint_index.resize(28);
    std::iota(policy.action_joint_index.begin(), policy.action_joint_index.end(), 0);
    policy.action_scale.assign(28, 0.25);
    policy.custom_array_dims["supermimic_obs"] = 900;
    rl_policy::ObsSegmentConfig segment;
    segment.terms = {"supermimic_obs"};
    policy.obs_segments = {segment};
    policy.strict_obs_dim_check = true;
    using Source = rl_policy::ModelInputSource;
    using Target = rl_policy::ModelOutputTarget;
    rl_policy::ModelInputBindingConfig observation;
    observation.name = "obs_hist";
    observation.source = Source::OBSERVATION;
    rl_policy::ModelInputBindingConfig goal;
    goal.name = "goal";
    goal.source = Source::EXTERNAL;
    goal.key = "supermimic_goal";
    policy.model_io.inputs = {observation, goal};
    for (const auto &entry : std::vector<std::pair<std::string, Target>>{
            {"actions", Target::ACTION}, {"seen_goal", Target::EXPOSE},
            {"seen_observation", Target::EXPOSE}}) {
        rl_policy::ModelOutputBindingConfig output;
        output.name = entry.first;
        output.target = entry.second;
        policy.model_io.outputs.push_back(output);
    }
    return policy;
}

void TestContract(const Fixture &fixture) {
    fixture.WriteConfig();
    auto config = LoadConfig(fixture.yaml, "probe", ".");
    auto policy = Policy(fixture.model);
    auto adapter = Create(config, policy);
    Require(std::string(adapter->Type()) == "supermimic", "factory type");
    rl_policy::PolicyExecutor executor;
    executor.Init(policy);
    auto robot = robot_base::RobotData::Create(28);
    robot.base_quat = {1, 0, 0, 0};
    robot.joint_pos.assign(28, 0.1);
    robot.joint_pos[0] = 0.6;
    robot.joint_pos[27] = -0.2;
    robot.joint_vel[0] = 2;
    robot.joint_vel[27] = -3;
    robot.gyro = {0.2, -0.4, 0.6};
    Eigen::VectorXf observation;
    std::vector<double> action;
    auto tick = [&] {
        adapter->PrepareInputs(robot, 0.02, executor);
        executor.AssembleObs(robot.gyro, robot.rpy, 0, 0, 0,
            robot.joint_pos, robot.joint_vel, robot.base_quat, {}, 0.02, observation);
        executor.Infer(observation, action);
    };
    adapter->Reset(robot);
    tick();
    Require(observation.size() == 900 && action.size() == 28, "tensor dimensions");
    const auto initial = executor.GetModelOutput("seen_observation");
    for (int frame = 0; frame < 10; ++frame) {
        Near(initial[frame * 28], 0.5, "seed q-offset, term-major");
        Near(initial[frame * 28 + 27], -0.3, "seed last joint");
        Near(initial[280 + frame * 28], 2, "velocity is not rescaled");
        Near(initial[280 + frame * 28 + 27], -3, "last joint velocity");
        Near(initial[560 + frame * 3], 0.2, "gyro is not rescaled");
        Near(initial[560 + frame * 3 + 1], -0.4, "second gyro axis");
        Near(initial[590 + frame * 3 + 2], -1, "upright projected gravity");
        for (int joint = 0; joint < 28; ++joint)
            Near(initial[620 + frame * 28 + joint], 0, "initial previous action");
    }
    const auto &goals = executor.GetModelOutput("seen_goal");
    Require(goals.size() == 660, "goal dimensions");
    for (int frame = 0; frame < 10; ++frame) {
        for (int feature = 0; feature < 66; ++feature) {
            const double expected = feature == 58 ? -1 : feature == 65 ? 0.95 : 0;
            Near(goals[frame * 66 + feature], expected, "stationary frame-major goal");
        }
    }
    Near(action[0], 0.5, "raw probe action");
    const auto raw = action;
    adapter->OnAction(action);
    Require(action == raw, "adapter must not rewrite raw policy output");
    std::vector<double> targets;
    executor.MapActionToTargetPos(action, targets);
    Near(targets[0], 0.225, "target equals default plus scaled raw action");

    robot.joint_pos[0] = 0.8;
    const double half_sqrt = std::sqrt(0.5);
    robot.base_quat = {half_sqrt, half_sqrt, 0, 0};
    tick();
    const auto second = executor.GetModelOutput("seen_observation");
    Near(second[0], 0.5, "oldest q survives rolling history");
    Near(second[252], 0.7, "newest q replaces only last frame");
    Near(second[617], 0, "gravity x after roll");
    Near(second[618], -1, "body-frame gravity after positive 90-degree roll");
    Near(second[619], 0, "gravity z after roll");
    Near(second[872], 0.5, "feedback uses previous RAW action, not scaled target");
    Near(second[899], -0.3, "last raw feedback component");

    const auto good = robot;
    robot.joint_pos[0] = std::numeric_limits<double>::quiet_NaN();
    Reject(tick, "nonfinite position must fail");
    robot = good;
    robot.base_quat = {0, 0, 0, 0};
    Reject(tick, "zero quaternion must fail");
    robot = good;
    robot.base_quat = {2, 0, 0, 0};
    Reject(tick, "non-unit quaternion must fail");
    robot = good;
    robot.joint_vel.pop_back();
    Reject(tick, "short velocity vector must fail");
    robot = good;
    robot.gyro[0] = std::numeric_limits<double>::max();
    Reject(tick, "float-overflowing sensor input must fail");
    robot = good;
    auto bad_action = raw;
    bad_action[27] = std::numeric_limits<double>::quiet_NaN();
    Reject([&] { adapter->OnAction(bad_action); }, "nonfinite action must fail");
    bad_action.resize(27);
    Reject([&] { adapter->OnAction(bad_action); }, "short action must fail");
    tick();
    const auto third = executor.GetModelOutput("seen_observation");
    Near(third[0], 0.5, "rejected samples must not advance history");
    Near(third[224], 0.7, "valid second sample remains penultimate");
    Near(third[872], 0.5, "rejected action must not poison feedback");
    Reject([&] { adapter->PrepareInputs(robot, -1, executor); }, "negative time must fail");
    Reject([&] { adapter->PrepareInputs(robot,
        std::numeric_limits<double>::quiet_NaN(), executor); }, "nonfinite time must fail");

    for (int i = 0; i < 12; ++i) {
        robot.joint_pos[0] = 1.1 + i;
        tick();
    }
    const auto rolled = executor.GetModelOutput("seen_observation");
    Near(rolled[0], 3, "history evicts frames beyond ten samples");
    Near(rolled[252], 12, "newest sample after wrap");
    adapter->Reset(robot);
    tick();
    const auto &reset = executor.GetModelOutput("seen_observation");
    for (int frame = 0; frame < 10; ++frame) {
        Near(reset[frame * 28], 12, "reset must reseed current position");
        Near(reset[620 + frame * 28], 0, "reset clears prior action");
    }
    Require(adapter->ReadyForPolicySwitch(), "static adapter allows a policy handoff");
}

void TestInvalidConfiguration(const Fixture &fixture) {
    for (const auto &height : {"", "0", "-1", ".nan", ".inf", "1e100", "1e-100", "bad"}) {
        fixture.WriteConfig(height);
        Reject([&] { Create(LoadConfig(fixture.yaml, "probe", "."), Policy(fixture.model)); },
            "invalid/missing reference height must fail");
    }
    for (const auto &period : {"0", "0.01", ".nan", "bad"}) {
        fixture.WriteConfig("0.95", period);
        Reject([&] { LoadConfig(fixture.yaml, "probe", "."); }, "non-50-Hz config must fail");
    }
    fixture.WriteConfig("0.95", "0.02", 2);
    Reject([&] { LoadConfig(fixture.yaml, "probe", "."); }, "extra decimation must fail");
    // Missing timing must not silently pick executor defaults (decimation defaults to four).
    for (const auto &missing : {"rl_dt: 0.02", "infer_decimation: 1"}) {
        fixture.WriteConfig();
        std::ifstream input(fixture.yaml);
        std::string contents((std::istreambuf_iterator<char>(input)), {});
        const auto position = contents.find(missing);
        Require(position != std::string::npos, "missing test configuration token");
        contents.erase(position, std::string(missing).size());
        std::ofstream output(fixture.yaml);
        output << contents;
        output.close();
        Reject([&] { LoadConfig(fixture.yaml, "probe", "."); }, "missing timing must fail");
    }
    fixture.WriteConfig();
    const auto config = LoadConfig(fixture.yaml, "probe", ".");
    const auto valid = Policy(fixture.model);
    auto bad = valid;
    std::swap(bad.action_joint_index[0], bad.action_joint_index[1]);
    Reject([&] { Create(config, bad); }, "wrong canonical joint order must fail");
    bad = valid;
    bad.rl_default_pos.pop_back();
    Reject([&] { Create(config, bad); }, "wrong default position dimensions must fail");
    bad = valid;
    bad.action_scale = {0.25};
    Reject([&] { Create(config, bad); }, "checkpoint requires per-joint action scaling");
    bad = valid;
    bad.action_scale[0] = std::numeric_limits<double>::quiet_NaN();
    Reject([&] { Create(config, bad); }, "nonfinite action scale must fail");
    bad = valid;
    bad.action_blend_ratio = 0.5;
    Reject([&] { Create(config, bad); }, "extra output smoothing must fail");
    bad = valid;
    bad.clip_actions = {1};
    Reject([&] { Create(config, bad); }, "action clipping changes training contract");
    bad = valid;
    bad.clip_observations = {1};
    Reject([&] { Create(config, bad); }, "observation clipping changes training contract");
    bad = valid;
    bad.custom_array_dims["supermimic_obs"] = 90;
    Reject([&] { Create(config, bad); }, "wrong history tensor dimensions must fail");
    bad = valid;
    bad.obs_segments[0].mode = "flat_history";
    bad.obs_segments[0].length = 10;
    Reject([&] { Create(config, bad); }, "double history assembly must fail");
    bad = valid;
    bad.model_io.inputs[1].key = "wrong_goal";
    Reject([&] { Create(config, bad); }, "wrong external goal key must fail");
}

}  // namespace

int main() {
    try {
        const Fixture fixture;
        TestContract(fixture);
        TestInvalidConfiguration(fixture);
        std::cout << "SuperMimic static contract, reset, mapping and error paths passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "SuperMimic test failed: " << error.what() << '\n';
        return 1;
    }
}
