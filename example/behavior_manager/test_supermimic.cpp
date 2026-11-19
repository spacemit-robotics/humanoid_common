/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_supermimic.cpp
 * @brief Exercise SuperMimic observation and reference inputs through the real executor.
 */

#include <unistd.h>

#include <cnpy.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_planners.h"
#include "motion_planner/motion_planner.h"
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
        const std::string &period = "0.02", int decimation = 1,
        const std::string &reference_options = "") const {
        std::ofstream output(yaml);
        output << "rl_policy:\n  rl_dt: " << period <<
            "\n  onnx_infer:\n    policies:\n      probe:\n"
            "        infer_decimation: " << decimation <<
            "\n        policy_adapter:\n          type: supermimic\n";
        if (!height.empty()) output << "          reference_height: " << height << '\n';
        output << reference_options;
        Require(output.good(), "cannot write configuration");
    }

    std::string WriteMotion(std::size_t joint_count = 28, std::size_t frames = 4) const {
        const std::string path = (directory_ / "motion.npz").string();
        std::vector<float> joints(frames * joint_count, 0.1F), velocities(frames * joint_count, 1.0F);
        std::vector<float> positions(frames * 2 * 3, 0.0F), quaternions(frames * 2 * 4, 0.0F);
        std::vector<float> linear(positions.size(), 0.0F), angular(positions.size(), 0.0F);
        for (std::size_t frame = 0; frame < frames; ++frame) {
            joints[frame * joint_count] += 0.04F * frame;
            const std::size_t body = frame * 2 + 1;
            positions[body * 3 + 2] = 1.0F + 0.02F * frame;
            const Eigen::Quaternionf q = Eigen::Quaternionf(
                Eigen::AngleAxisf(std::acos(-1.0F) / 2, Eigen::Vector3f::UnitZ())) *
                Eigen::Quaternionf(Eigen::AngleAxisf(frame == 0 ? std::acos(-1.0F) / 2 : 0.0F,
                    Eigen::Vector3f::UnitX()));
            quaternions[body * 4] = q.w();
            quaternions[body * 4 + 1] = q.x();
            quaternions[body * 4 + 2] = q.y();
            quaternions[body * 4 + 3] = q.z();
            linear[body * 3] = 1;
            angular[body * 3 + 1] = 1;
        }
        cnpy::npz_save(path, "joint_pos", joints.data(), {frames, joint_count}, "w");
        cnpy::npz_save(path, "joint_vel", velocities.data(), {frames, joint_count}, "a");
        cnpy::npz_save(path, "body_pos_w", positions.data(), {frames, 2, 3}, "a");
        cnpy::npz_save(path, "body_quat_w", quaternions.data(), {frames, 2, 4}, "a");
        cnpy::npz_save(path, "body_lin_vel_w", linear.data(), {frames, 2, 3}, "a");
        cnpy::npz_save(path, "body_ang_vel_w", angular.data(), {frames, 2, 3}, "a");
        return path;
    }

    std::string WriteStill(const std::string &name, float joint, float height) const {
        const std::string path = (directory_ / name).string();
        std::vector<float> joints(2 * 28, joint), velocities(2 * 28, 0.0F);
        std::vector<float> positions(2 * 2 * 3, 0.0F), quaternions(2 * 2 * 4, 0.0F);
        std::vector<float> rates(positions.size(), 0.0F);
        for (int body = 0; body < 4; ++body) {
            positions[body * 3 + 2] = height;
            quaternions[body * 4] = 1.0F;
        }
        cnpy::npz_save(path, "joint_pos", joints.data(), {2, 28}, "w");
        cnpy::npz_save(path, "joint_vel", velocities.data(), {2, 28}, "a");
        cnpy::npz_save(path, "body_pos_w", positions.data(), {2, 2, 3}, "a");
        cnpy::npz_save(path, "body_quat_w", quaternions.data(), {2, 2, 4}, "a");
        cnpy::npz_save(path, "body_lin_vel_w", rates.data(), {2, 2, 3}, "a");
        cnpy::npz_save(path, "body_ang_vel_w", rates.data(), {2, 2, 3}, "a");
        return path;
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
    Reject([&] { Create(config, bad); }, "requires per-joint action scaling");
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

void TestReferencePlayback(const Fixture &fixture) {
    const auto path = fixture.WriteMotion();
    const std::string options =
        "          reference_file: motion.npz\n"
        "          anchor_body_index: 1\n"
        "          motion_fps: 25\n"
        "          start_mode: manual\n";
    fixture.WriteConfig("", "0.02", 1, options);
    const auto config = LoadConfig(fixture.yaml, "probe", std::filesystem::path(path).parent_path());
    Require(config.reference_file == path && config.manual_start && config.motion_fps == 25,
        "reference settings and relative resource path");
    auto policy = Policy(fixture.model);
    auto adapter = Create(config, policy);
    rl_policy::PolicyExecutor executor;
    executor.Init(policy);
    auto robot = robot_base::RobotData::Create(28);
    robot.base_quat = {1, 0, 0, 0};
    robot.joint_pos.assign(28, 0.1);
    auto tick = [&](double time) {
        adapter->PrepareInputs(robot, time, executor);
        Eigen::VectorXf observation;
        std::vector<double> action;
        executor.AssembleObs(robot.gyro, robot.rpy, 0, 0, 0,
            robot.joint_pos, robot.joint_vel, robot.base_quat, {}, 0.02, observation);
        executor.Infer(observation, action);
        adapter->OnAction(action);
        return executor.GetModelOutput("seen_goal");
    };
    adapter->Reset(robot);
    auto goal = tick(45.0);
    Require(adapter->ReadyForPolicySwitch() && !adapter->PlaybackFinished(), "manual wait state");
    for (int frame = 0; frame < 10; ++frame) {
        Near(goal[frame * 66], 0, "wait holds first position throughout the horizon");
        Near(goal[frame * 66 + 28], 0, "waiting joint velocity");
        Near(goal[frame * 66 + 57], -1, "reference gravity, not sensor gravity");
        Near(goal[frame * 66 + 59], 0, "waiting root angular velocity");
        Near(goal[frame * 66 + 63], 0, "waiting root linear velocity");
    }
    Require(adapter->StartPlayback(45.0) && !adapter->StartPlayback(45.1), "start once");
    goal = tick(45.0);
    Require(!adapter->ReadyForPolicySwitch(), "active reference blocks handoff");
    Near(goal[28], 1, "reference joint velocity");
    Near(goal[59], 1, "world angular velocity rotated by full root orientation");
    Near(goal[63], -1, "world linear velocity rotated by root heading only");
    Near(goal[0], 0.02, "first goal is 20 ms ahead at 25 fps");
    Near(goal[57], -std::sqrt(0.5), "quaternion interpolation gravity y");
    Near(goal[58], -std::sqrt(0.5), "quaternion interpolation gravity z");
    Near(goal[65], 1.01, "interpolated reference height");
    Near(goal[66], 0.04, "second goal is 40 ms ahead");
    Near(goal[9 * 66], 0.12, "future window repeats final position");
    Near(goal[9 * 66 + 28], 1, "future padding retains the final reference velocity");
    goal = tick(45.04);
    Near(goal[0], 0.06, "current reference advances from manual start plus lookahead");
    goal = tick(45.2);
    Require(adapter->PlaybackFinished() && adapter->ReadyForPolicySwitch(), "finite clip ends");
    Near(goal[0], 0.12, "end holds last position");
    Near(goal[28], 0, "end stops joint velocity");
    Near(goal[59], 0, "end stops root rotation");
    Near(goal[63], 0, "end stops root translation");
    adapter->Reset(robot);
    goal = tick(46.0);
    Near(goal[0], 0, "reset restores manual wait and reference clock");
    Require(!adapter->PlaybackFinished(), "reset clears completion");

    auto loop = config;
    loop.manual_start = false;
    loop.loop = true;
    loop.playback_speed = 0.5;
    loop.loop_pause = 0.1;
    adapter = Create(loop, policy);
    adapter->Reset(robot);
    goal = tick(0);
    Near(goal[0], 0.01, "slow playback changes first future sampling");
    Near(goal[28], 0.5, "slow playback scales joint velocity");
    Near(goal[59], 0.5, "slow playback scales root angular velocity");
    Near(goal[63], -0.5, "slow playback scales root linear velocity");
    goal = tick(0.35);
    Near(goal[0], 0.12, "loop pause holds last position");
    Near(goal[28], 0, "loop pause stops velocity");
    Require(!adapter->PlaybackFinished(), "loop is not finite");
    goal = tick(0.44);
    Near(goal[0], 0.02, "next cycle begins after pause with lookahead");
    adapter->PreparePolicySwitch(true, 0.44);
    goal = tick(0.85);
    Near(goal[0], 0.12, "handoff holds final reference across loop boundary");
    Require(adapter->ReadyForPolicySwitch(), "handoff becomes ready at clip end");
    adapter->PreparePolicySwitch(false, 0.85);
    goal = tick(0.9);
    Near(goal[0], 0.12, "cancelled handoff does not jump back to a running clock");

    // At 50 Hz, a robot initialized on frame 0 receives frames 1..10.
    fixture.WriteMotion(28, 12);
    auto fifty_hz = config;
    fifty_hz.motion_fps = 50;
    fifty_hz.manual_start = false;
    adapter = Create(fifty_hz, policy);
    adapter->Reset(robot);
    goal = tick(0);
    for (int future = 0; future < 10; ++future) {
        Near(goal[future * 66], 0.04 * (future + 1), "reset goal uses frames 1 through 10");
        Near(goal[future * 66 + 65], 1.0 + 0.02 * (future + 1), "future frame height");
    }
    auto bad = config;
    bad.anchor_body_index = 2;
    Reject([&] { Create(bad, policy); }, "reference body index must be valid");
    bad = config;
    bad.playback_speed = 0;
    Reject([&] { Create(bad, policy); }, "reference speed must be positive");
    fixture.WriteMotion(27);
    Reject([&] { Create(config, policy); }, "reference joint shapes must match policy");
}

void TestCatalog(const Fixture &fixture) {
    fixture.WriteMotion();
    fixture.WriteStill("standby.npz", 0.3F, 0.8F);
    auto write = [&](const std::string &extra, const std::string &actions,
        const std::string &cancel_duration = "") {
        std::ofstream output(fixture.yaml);
        output << "rl_policy:\n  rl_dt: 0.02\n  onnx_infer:\n    policies:\n      probe:\n"
            "        infer_decimation: 1\n        policy_adapter:\n          type: supermimic\n"
            "          catalog: probe_actions\n          anchor_body_index: 1\n" << extra <<
            "probe_actions:\n  default_action: standby\n  transition_duration: 0.1\n"
            "  motion_fps: 25\n  action_names: " << actions << "\n  actions:\n"
            "    standby: {file: standby.npz}\n    move: {file: motion.npz";
        if (!cancel_duration.empty()) output << ", cancel_transition_duration: " << cancel_duration;
        output << "}\n";
        Require(output.good(), "cannot write catalog configuration");
    };
    const std::string directory = std::filesystem::path(fixture.yaml).parent_path().string();
    write("", "[standby, move]");
    const auto config = LoadConfig(fixture.yaml, "probe", directory);
    const auto &catalog = config.reference_motion_catalog;
    Require(catalog.Enabled() && catalog.actions.size() == 2 && catalog.default_action == "standby" &&
        catalog.transition_duration == 0.1 && catalog.actions[1].motion_fps == 25 &&
        catalog.actions[1].file ==
            std::filesystem::weakly_canonical(directory + "/motion.npz").string(),
        "catalog settings and resource paths");
    auto policy = Policy(fixture.model);
    auto adapter = Create(config, policy);
    rl_policy::PolicyExecutor executor;
    executor.Init(policy);
    auto robot = robot_base::RobotData::Create(28);
    robot.base_quat = {1, 0, 0, 0};
    robot.joint_pos.assign(28, 0.1);
    auto tick = [&](double time) {
        robot.joint_pos[0] = time;
        adapter->PrepareInputs(robot, time, executor);
        Eigen::VectorXf observation;
        std::vector<double> action;
        executor.AssembleObs(robot.gyro, robot.rpy, 0, 0, 0,
            robot.joint_pos, robot.joint_vel, robot.base_quat, {}, 0.02, observation);
        executor.Infer(observation, action);
        adapter->OnAction(action);
        return executor.GetModelOutput("seen_goal");
    };
    uint64_t sequence = 0;
    auto request = [&](robot_base::InteractionRequest::Operation operation,
        const std::string &name, double time) {
        adapter->HandleInteractionRequest({++sequence, operation, name}, time);
        return adapter->GetInteractionStatus();
    };
    using Operation = robot_base::InteractionRequest::Operation;
    using Phase = robot_base::InteractionStatus::Phase;
    adapter->Reset(robot);
    auto goal = tick(0.98);
    Require(adapter->ReadyForPolicySwitch() && !adapter->PlaybackFinished() &&
        adapter->GetInteractionStatus().phase == Phase::IDLE, "catalog starts on the default action");
    for (int frame = 0; frame < 10; ++frame) {
        Near(goal[frame * 66], 0.2, "default action holds its first frame");
        Near(goal[frame * 66 + 28], 0, "held default joint velocity");
        Near(goal[frame * 66 + 58], -1, "default reference gravity");
        Near(goal[frame * 66 + 65], 0.8, "default reference height");
    }

    auto status = request(Operation::START, "move", 1.0);
    Require(status.request_accepted && status.phase == Phase::BLEND_IN && status.action == "move" &&
        !adapter->ReadyForPolicySwitch(), "start blends towards the action");
    goal = tick(1.0);
    const auto &history = executor.GetModelOutput("seen_observation");
    Near(history[224], 0.88, "action start keeps observation history");
    Near(history[252], 0.9, "newest observation after the action start");
    Near(history[872], 0.88, "action start keeps the previous raw action");
    const double pi = std::acos(-1.0);
    for (int frame = 0; frame < 4; ++frame) {
        const double phase = 0.2 * (frame + 1);
        const double weight = 0.5 - 0.5 * std::cos(pi * phase);
        const double rate = 0.5 * pi * std::sin(pi * phase) / 0.1;
        Near(goal[frame * 66], 0.2 - 0.2 * weight, "future blend position");
        Near(goal[frame * 66 + 28], -0.2 * rate + weight, "blend derivative plus target velocity");
        const double norm = std::hypot(weight, 1.0 - weight);
        Near(goal[frame * 66 + 57], -weight / norm, "normalized gravity blend y");
        Near(goal[frame * 66 + 58], -(1.0 - weight) / norm, "normalized gravity blend z");
        Near(goal[frame * 66 + 59], weight, "root angular velocity fades in");
        Near(goal[frame * 66 + 63], -weight, "root linear velocity fades in");
        Near(goal[frame * 66 + 65], 0.8 + 0.2 * weight, "height blend");
    }
    Near(goal[4 * 66], 0, "playback follows the blend inside the window");
    Near(goal[4 * 66 + 28], 1, "playback velocity after the blend");
    Near(goal[5 * 66], 0.02, "20 ms playback after the blend at 25 fps");

    status = request(Operation::START, "standby", 1.05);
    Require(!status.request_accepted && status.phase == Phase::BLEND_IN, "busy catalog rejects a start");
    goal = tick(1.1);
    status = adapter->GetInteractionStatus();
    Require(status.phase == Phase::PLAYING && status.progress == 0.0F, "blend completes into playback");
    Near(goal[0], 0.02, "action goal begins one policy step ahead");
    Near(goal[66], 0.04, "playback horizon");
    goal = tick(1.18);
    Near(goal[0], 0.10, "action time follows the wall clock plus lookahead");
    Near(goal[9 * 66], 0.12, "catalog horizon clamps to the last frame");
    Near(goal[9 * 66 + 28], 1, "catalog padding keeps reference velocity until playback ends");
    Near(adapter->GetInteractionStatus().progress, 0.5, "action progress");
    goal = tick(1.28);
    Require(adapter->GetInteractionStatus().phase == Phase::FINISHED && adapter->PlaybackFinished() &&
        adapter->ReadyForPolicySwitch(), "finite action finishes");
    Near(goal[0], 0.12, "finished action holds its last frame");
    Near(goal[28], 0, "held last frame stops joint velocity");
    Near(goal[59], 0, "held last frame stops root rotation");

    status = request(Operation::START, "missing", 1.3);
    Require(!status.request_accepted && status.phase == Phase::REJECTED, "unknown action is rejected");
    status = request(Operation::CANCEL, "", 1.3);
    Require(!status.request_accepted, "cancel needs an active action");
    status = request(Operation::START, "move", 1.3);
    Require(status.request_accepted && status.phase == Phase::BLEND_IN, "next action after finishing");
    goal = tick(1.3);
    Near(goal[0], 0.12 - 0.12 * (0.5 - 0.5 * std::cos(0.2 * pi)), "blend back to the first frame");
    goal = tick(1.44);
    status = request(Operation::CANCEL, "", 1.44);
    Require(status.request_accepted && status.phase == Phase::BLEND_OUT &&
        !adapter->ReadyForPolicySwitch(), "cancel blends to the default action");
    goal = tick(1.44);
    const double cancel_weight = 0.5 - 0.5 * std::cos(0.2 * pi);
    Near(goal[0], 0.06 + 0.14 * cancel_weight, "cancel blends from the continuing reference");
    Near(goal[28], 0.14 * 0.5 * pi * std::sin(0.2 * pi) / 0.1 + 1 - cancel_weight,
        "cancel velocity includes the moving reference and the blend derivative");
    Near(goal[59], 1 - cancel_weight, "cancel fades root rotation");
    Near(goal[5 * 66], 0.2, "cancel ends on the default action");
    Near(goal[5 * 66 + 59], 0, "default action is held still");
    goal = tick(1.56);
    status = adapter->GetInteractionStatus();
    Require(status.phase == Phase::FINISHED && adapter->ReadyForPolicySwitch() &&
        !adapter->PlaybackFinished(), "cancel finishes on the default action");
    Near(goal[0], 0.2, "default action after cancel");

    request(Operation::START, "move", 2.0);
    tick(2.2);
    Require(!adapter->ReadyForPolicySwitch(), "playing action blocks a policy switch");
    adapter->PreparePolicySwitch(true, 2.2);
    Require(adapter->GetInteractionStatus().phase == Phase::BLEND_OUT, "policy switch returns to default");
    adapter->PreparePolicySwitch(true, 2.26);
    tick(2.26);
    Require(!adapter->ReadyForPolicySwitch(), "policy switch waits for the blend");
    adapter->PreparePolicySwitch(true, 2.32);
    goal = tick(2.32);
    Require(adapter->ReadyForPolicySwitch(), "default action allows the policy switch");
    Near(goal[0], 0.2, "policy switch leaves the default action held");

    request(Operation::START, "move", 3.0);
    adapter->Reset(robot);
    goal = tick(4.0);
    Require(adapter->GetInteractionStatus().phase == Phase::IDLE && adapter->ReadyForPolicySwitch(),
        "reset returns to the default action");
    Near(goal[0], 0.2, "reset holds the default action");
    status = request(Operation::START, "move", 4.0);
    Require(status.request_accepted && status.sequence == sequence, "reset accepts new request sequences");

    write("", "[standby, move]", "0.2");
    const auto custom = LoadConfig(fixture.yaml, "probe", directory);
    Require(!custom.reference_motion_catalog.actions[0].cancel_transition_duration &&
        custom.reference_motion_catalog.actions[1].cancel_transition_duration == 0.2,
        "cancel duration belongs to its action, not the whole catalog");
    adapter = Create(custom, policy);
    adapter->Reset(robot);
    request(Operation::START, "move", 5.0);
    tick(5.1);
    Require(adapter->GetInteractionStatus().phase == Phase::PLAYING,
        "cancel duration does not change entry timing");
    request(Operation::CANCEL, "", 5.14);
    tick(5.24);
    Require(!adapter->ReadyForPolicySwitch(), "per-action cancellation uses its longer duration");
    request(Operation::CANCEL, "", 5.25);
    adapter->PreparePolicySwitch(true, 5.26);
    tick(5.35);
    Require(adapter->ReadyForPolicySwitch() &&
        adapter->GetInteractionStatus().phase == Phase::FINISHED,
        "repeated cancellation and handoff do not restart the return transition");
    request(Operation::START, "move", 6.0);
    tick(6.14);
    adapter->PreparePolicySwitch(true, 6.14);
    tick(6.25);
    Require(!adapter->ReadyForPolicySwitch(), "policy switch uses the active action return duration");
    tick(6.35);
    Require(adapter->ReadyForPolicySwitch(), "policy switch completes after the custom duration");

    write("", "[standby, move]", "-1");
    Reject([&] { Create(LoadConfig(fixture.yaml, "probe", directory), policy); },
        "negative action cancel duration must fail");
    write("", "[standby, move]", ".nan");
    Reject([&] { Create(LoadConfig(fixture.yaml, "probe", directory), policy); },
        "non-finite action cancel duration must fail");
    write("", "[move]");
    Reject([&] { LoadConfig(fixture.yaml, "probe", directory); }, "default action must be in the catalog");
    write("          start_mode: manual\n", "[standby, move]");
    Reject([&] { Create(LoadConfig(fixture.yaml, "probe", directory), policy); },
        "catalog actions do not use manual start");
    write("          transition_duration: -1\n", "[standby, move]");
    Reject([&] { Create(LoadConfig(fixture.yaml, "probe", directory), policy); },
        "negative transition must fail");
    write("", "[standby, move, move]");
    Reject([&] { Create(LoadConfig(fixture.yaml, "probe", directory), policy); },
        "duplicate catalog action must fail");
}

// A realtime planner with a moving root: it advances along its heading at 0.5 m/s while
// yawing at 1 rad/s, 0.9 m high; joint 0 ramps at 0.5 rad/s from 0.6 and the others hold
// 0.6. The planner named "overflow" publishes root positions whose differences overflow.
class RootMotionPlanner final : public behavior_manager::motion_planner::RealtimePlanner {
public:
    explicit RootMotionPlanner(const behavior_manager::motion_planner::PlannerContext &context)
        : num_dof_(context.num_dof), overflow_(context.name == "overflow") {}

    void Start(const robot_base::RobotData &, double time_s) override {
        constexpr int kFrames = 100;
        auto output = std::make_shared<behavior_manager::motion_planner::PlannerOutput>();
        auto &trajectory = output->trajectory;
        trajectory.num_dof = num_dof_;
        trajectory.fps = 50.0;
        trajectory.start_time_s = time_s;
        trajectory.joint_pos.assign(static_cast<std::size_t>(kFrames) * num_dof_, 0.6F);
        trajectory.joint_vel.assign(static_cast<std::size_t>(kFrames) * num_dof_, 0.0F);
        trajectory.root_pos.resize(kFrames * 3);
        trajectory.root_quat.resize(kFrames * 4);
        for (int frame = 0; frame < kFrames; ++frame) {
            const float time = 0.02F * frame;
            trajectory.joint_pos[static_cast<std::size_t>(frame) * num_dof_] = 0.6F + 0.5F * time;
            trajectory.joint_vel[static_cast<std::size_t>(frame) * num_dof_] = 0.5F;
            trajectory.root_pos[frame * 3] = 0.5F * std::sin(time);
            trajectory.root_pos[frame * 3 + 1] = 0.5F * (1.0F - std::cos(time));
            trajectory.root_pos[frame * 3 + 2] = 0.9F;
            if (overflow_) trajectory.root_pos[frame * 3] = frame % 2 == 0 ? 3.0e38F : -3.0e38F;
            trajectory.root_quat[frame * 4] = std::cos(0.5F * time);
            trajectory.root_quat[frame * 4 + 3] = std::sin(0.5F * time);
        }
        output_ = std::move(output);
    }

    void Update(const behavior_manager::motion_planner::PlannerInput &) override {}
    std::shared_ptr<const behavior_manager::motion_planner::PlannerOutput> Output() override { return output_; }
    void Stop() override { output_.reset(); }

private:
    int num_dof_;
    bool overflow_;
    std::shared_ptr<const behavior_manager::motion_planner::PlannerOutput> output_;
};

// Steps the adapter like the inference thread: one 20 ms policy step per call.
class PlannerProbe {
public:
    PlannerProbe(const behavior_manager::policy_adapter::Config &config, const Fixture &fixture)
        : policy_(Policy(fixture.model)) {
        adapter = Create(config, policy_);
        executor_.Init(policy_);
        robot_ = robot_base::RobotData::Create(28);
        robot_.base_quat = {1, 0, 0, 0};
        robot_.joint_pos.assign(28, 0.1);
        robot_.environment = robot_base::RuntimeEnvironment::SIMULATION;
        adapter->Reset(robot_);
    }

    const std::vector<float> &Tick() {
        static_cast<void>(adapter->SelectExecutor(command, executor_));
        if (!switching) adapter->HandleInteractionRequest(interaction, time);
        adapter->HandlePlannerRequest(request);
        adapter->PreparePolicySwitch(switching, time);
        adapter->PrepareInputs(robot_, time, executor_);
        Eigen::VectorXf observation;
        std::vector<double> action;
        executor_.AssembleObs(robot_.gyro, robot_.rpy, 0, 0, 0,
            robot_.joint_pos, robot_.joint_vel, robot_.base_quat, {}, 0.02, observation);
        executor_.Infer(observation, action);
        adapter->OnAction(action);
        goal = executor_.GetModelOutput("seen_goal");
        last_time = time;
        time += 0.02;
        return goal;
    }

    bool Until(const std::function<bool()> &done, int timeout_ms = 5000) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (true) {
            Tick();
            if (done()) return true;
            if (std::chrono::steady_clock::now() >= deadline) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    bool UntilPhase(robot_base::PlannerStatus::Phase phase) {
        return Until([&] { return planner().phase == phase; });
    }

    // Ticks through a reference change: joint 0 of the first goal frame must move in small
    // steps and end on the target.
    bool Settles(double target, double max_step = 0.12, int ticks = 12) {
        double previous = goal[0];
        for (int index = 0; index < ticks; ++index) {
            const double value = Tick()[0];
            if (std::abs(value - previous) > max_step) return false;
            previous = value;
        }
        return std::abs(previous - target) < 1e-5;
    }

    void Start(const std::string &name, const std::string &argument = "") {
        request = {request.sequence + 1, robot_base::PlannerRequest::Operation::START, name, argument};
    }

    void Stop() { request = {request.sequence + 1, robot_base::PlannerRequest::Operation::STOP, "", ""}; }

    void Action(const std::string &name) {
        interaction = {interaction.sequence + 1, robot_base::InteractionRequest::Operation::START, name};
    }

    void Restart() {
        adapter->OnInferenceStopped();
        adapter->Reset(robot_);
        request = {};
        interaction = {};
        command = {};
        switching = false;
        time = 0.0;
    }

    const robot_base::PlannerStatus &planner() const { return *adapter->PlannerStatus(); }
    robot_base::InteractionStatus action() const { return adapter->GetInteractionStatus(); }

    std::unique_ptr<behavior_manager::policy_adapter::PolicyAdapter> adapter;
    std::array<double, 3> command{};
    robot_base::PlannerRequest request;
    robot_base::InteractionRequest interaction;
    bool switching = false;
    double time = 0.0;
    double last_time = 0.0;
    std::vector<float> goal;

private:
    rl_policy::PolicyExecutorConfig policy_;
    rl_policy::PolicyExecutor executor_;
    robot_base::RobotData robot_;
};

// Planners registered for a SuperMimic policy: trajectories, catalog actions selected by a
// decision planner and generated motions all replace the goal without touching the
// interaction status, and return to the standby reference.
void TestPlanners(const Fixture &fixture) {
    using behavior_manager::motion_planner::Category;
    using behavior_manager::motion_planner::Kind;
    using behavior_manager::motion_planner::PlannerContext;
    using PlannerPhase = robot_base::PlannerStatus::Phase;
    using ActionPhase = robot_base::InteractionStatus::Phase;
    fake_planners::Register();
    behavior_manager::motion_planner::RegisterPlannerType("supermimic_root_motion",
        {Category::REALTIME, Kind::MODEL,
            [](const PlannerContext &context) -> std::unique_ptr<behavior_manager::motion_planner::Planner> {
                return std::make_unique<RootMotionPlanner>(context);
            }});
    auto &script = fake_planners::GetScript();
    script.Reset();
    fixture.WriteMotion();
    fixture.WriteStill("standby.npz", 0.3F, 0.8F);
    const auto write = [&](const std::string &robot_dof, const std::string &adapter) {
        std::ofstream output(fixture.yaml);
        output << "robot_base: {num_dof: " << robot_dof << "}\n"
            "motion_planners:\n"
            "  walker: {type: fake_trajectory, command: {limits: {max_vx: 1, max_vy: 1, max_wz: 1}}}\n"
            "  mover: {type: supermimic_root_motion}\n"
            "  overflow: {type: supermimic_root_motion}\n"
            "  chooser: {type: fake_choice, command: {limits: {max_vx: 1, max_vy: 1, max_wz: 1}}}\n"
            "  writer: {type: fake_generator, category: offline}\n"
            "rl_policy:\n  rl_dt: 0.02\n  onnx_infer:\n    policies:\n      probe:\n"
            "        infer_decimation: 1\n        policy_adapter:\n          type: supermimic\n"
            "          anchor_body_index: 1\n" << adapter <<
            "probe_actions:\n  default_action: idle\n  transition_duration: 0.1\n"
            "  motion_fps: 25\n  action_names: [idle, gesture]\n  actions:\n"
            "    idle: {file: standby.npz}\n    gesture: {file: motion.npz}\n";
        Require(output.good(), "cannot write planner configuration");
    };
    const std::string directory = std::filesystem::path(fixture.yaml).parent_path().string();
    const std::string catalog = "          catalog: probe_actions\n";
    const std::string planners = "          planners: [walker, mover, overflow, chooser, writer]\n";
    write("28", "          reference_file: motion.npz\n" + planners);
    Reject([&] { LoadConfig(fixture.yaml, "probe", directory); },
        "planners need a catalog that provides the standby reference");
    write("29", catalog + planners);
    Reject([&] { Create(LoadConfig(fixture.yaml, "probe", directory), Policy(fixture.model)); },
        "planner trajectories must match the policy joints");
    write("28", catalog + planners);
    const auto config = LoadConfig(fixture.yaml, "probe", directory);
    Require(config.planners.planners.size() == 5 && config.planners.planners[4].category == Category::OFFLINE,
        "planners of a SuperMimic policy come from YAML");
    PlannerProbe probe(config, fixture);
    const auto idle = [&] { return probe.planner().phase == PlannerPhase::READY ||
        probe.planner().phase == PlannerPhase::IDLE; };

    // Model trajectory: crossfade in, in-place replacement of consecutive plans, crossfade out.
    probe.Tick();
    Near(probe.goal[0], 0.2, "the standby reference before any planner");
    Near(probe.goal[65], 0.8, "the standby height");
    Require(probe.planner().phase == PlannerPhase::IDLE && probe.adapter->ReadyForPolicySwitch(),
        "no planner is loaded until one is requested");
    probe.Start("walker");
    Require(probe.UntilPhase(PlannerPhase::RUNNING), "the planner did not start: " + probe.planner().detail);
    Require(!probe.adapter->ReadyForPolicySwitch(), "a running planner blocks a policy switch");
    Require(probe.Settles(0.4), "the planner trajectory must be blended in");
    Near(probe.goal[1], 0.401, "trajectory joints use the policy joint order");
    Near(probe.goal[28], 0, "a constant trajectory has no joint velocity");
    Near(probe.goal[58], -1, "an upright trajectory root");
    Near(probe.goal[65], 0, "the root height comes from the trajectory");
    Require(probe.action().phase == ActionPhase::IDLE && probe.action().action.empty(),
        "a planner reference is not an operator action");
    probe.Action("gesture");
    probe.Tick();
    Require(!probe.action().request_accepted && probe.action().phase == ActionPhase::REJECTED &&
        probe.action().sequence == 1, "actions wait until the planner stops");
    Near(probe.goal[0], 0.4, "a rejected action leaves the planner reference");
    probe.command = {0.3, 0.0, 0.0};
    Require(probe.Until([&] { return script.last_vx == 0.3F; }), "the command did not reach the planner");
    Require(probe.Until([&] { return std::abs(probe.goal[0] - 0.7) < 1e-5; }),
        "consecutive plans replace each other without a new blend");
    probe.command = {0.0, 0.0, 0.0};
    Require(probe.Until([&] { return script.last_vx == 0.0F && std::abs(probe.goal[0] - 0.4) < 1e-5; }),
        "the zero command did not produce the stationary reference");
    probe.Stop();
    Require(probe.Until(idle), "the planner did not stop");
    Require(probe.Settles(0.2), "stopping must blend back to the standby reference");
    Near(probe.goal[65], 0.8, "the standby height after the planner");
    Require(probe.adapter->ReadyForPolicySwitch() && probe.action().phase == ActionPhase::REJECTED,
        "returning to standby is not an action result");
    probe.Action("gesture");
    probe.Tick();
    Require(probe.action().request_accepted && probe.action().phase == ActionPhase::BLEND_IN,
        "actions are available again once the planner has stopped");
    probe.Start("walker");
    probe.Tick();
    Require(!probe.planner().request_accepted && probe.planner().phase == PlannerPhase::READY,
        "a planner cannot start while an action plays");

    // Root pose: velocities are derived from the trajectory and sampled on its time axis.
    probe.Restart();
    probe.Start("mover");
    Require(probe.UntilPhase(PlannerPhase::RUNNING), "the root-motion planner did not start");
    const double start = probe.last_time + probe.planner().reference_remaining_s - 1.98;
    for (int index = 0; index < 8; ++index) probe.Tick();
    // The first goal frame is one policy step ahead of the tick.
    const auto expected = [&] { return 0.5 + 0.5 * (probe.last_time + 0.02 - start); };
    Require(std::abs(probe.goal[0] - expected()) < 0.012,
        "a continuous trajectory keeps the time axis it was planned on");
    Require(std::abs(probe.goal[28] - 0.5) < 1e-4, "the joint velocity of the trajectory");
    Near(probe.goal[1], 0.5, "the joints that hold still");
    Require(std::abs(probe.goal[56]) < 1e-5 && std::abs(probe.goal[58] + 1) < 1e-5, "yaw does not tilt gravity");
    Require(std::abs(probe.goal[59]) < 1e-3 && std::abs(probe.goal[61] - 1) < 1e-3,
        "the root angular velocity is derived in the root frame");
    Require(std::abs(probe.goal[62] - 0.5) < 1e-2 && std::abs(probe.goal[63]) < 1e-2 &&
        std::abs(probe.goal[64]) < 1e-3, "the root linear velocity is derived in the heading frame");
    Require(std::abs(probe.goal[65] - 0.9) < 1e-5, "the root height of the trajectory");
    Require(std::abs(probe.goal[9 * 66] - (expected() + 0.5 * 0.18)) < 0.012,
        "the goal horizon looks ahead along the trajectory");
    // Past its last frame the trajectory is held still.
    Require(probe.Until([&] { return probe.last_time - start > 2.2; }), "the trajectory did not run out");
    Near(probe.goal[28], 0, "a held trajectory has no joint velocity");
    Near(probe.goal[62], 0, "a held trajectory has no root velocity");
    Require(std::abs(probe.goal[0] - (0.5 + 0.5 * 1.98)) < 1e-4 &&
        probe.planner().phase == PlannerPhase::RUNNING, "the last frame is held while the planner runs");
    // The policy leaves through the standby reference.
    probe.switching = true;
    Require(probe.Until([&] { return probe.adapter->ReadyForPolicySwitch(); }),
        "a policy switch must release the planner reference");
    Near(probe.goal[0], 0.2, "the policy switch ends on the standby reference");
    Require(probe.action().phase == ActionPhase::IDLE, "a planner release is not an action result");

    // A trajectory the tracker cannot convert fails the planner and keeps the standby.
    probe.Restart();
    probe.Start("overflow");
    Require(probe.UntilPhase(PlannerPhase::FAILED) &&
        probe.planner().detail.find("not finite") != std::string::npos,
        "an unusable trajectory must fail the planner: " + probe.planner().detail);
    probe.Tick();
    Near(probe.goal[0], 0.2, "the standby reference is kept after a planner failure");
    Require(probe.adapter->ReadyForPolicySwitch(), "a failed planner does not hold the reference");

    // Decision planner: selects catalog actions; the tracker loads, blends and plays them.
    probe.Restart();
    probe.Start("chooser");
    Require(probe.UntilPhase(PlannerPhase::RUNNING), "the decision planner did not start");
    probe.command = {0.4, 0.0, 0.0};
    Require(probe.Until([&] { return probe.planner().progress > 0.3F; }),
        "the selected action did not play");
    Require(probe.goal[0] > -1e-6 && probe.goal[0] < 0.2 && probe.goal[65] > 0.99,
        "the goal follows the selected catalog action");
    Require(probe.action().phase == ActionPhase::IDLE, "a selected action is not an operator action");
    // The action is replayed while the command holds, then the planner returns to idle.
    bool replayed = false;
    float previous = probe.planner().progress;
    Require(probe.Until([&] {
        replayed = replayed || probe.planner().progress < previous;
        previous = probe.planner().progress;
        return replayed;
    }), "the decision planner did not replay its action");
    probe.command = {0.0, 0.0, 0.0};
    probe.Stop();
    Require(probe.Until(idle) && probe.Settles(0.2), "the decision planner did not return to standby");
    probe.Start("chooser", "missing");
    Require(probe.UntilPhase(PlannerPhase::FAILED) &&
        probe.planner().detail.find("not in the catalog") != std::string::npos,
        "an action outside the catalog must fail the planner: " + probe.planner().detail);

    // Offline planner: one generated motion, played once, then back to standby.
    probe.Restart();
    probe.Start("writer", "wave");
    Require(probe.UntilPhase(PlannerPhase::RUNNING), "the generated motion did not start");
    // A 0.4 rad change over the 0.1 s cosine blend moves at most 0.124 rad per step.
    Require(probe.Settles(0.6, 0.13, 8), "the generated motion must be blended in");
    Require(probe.Until([&] { return probe.planner().progress > 0.5F; }), "playback progress was not reported");
    Require(probe.Until(idle), "the generated motion did not finish");
    Require(probe.Settles(0.2, 0.13), "a finished motion returns to the standby reference");
    Require(probe.action().phase == ActionPhase::IDLE && script.generated == 1,
        "a generated motion is not an operator action");

    // Leaving RL unloads the planner; the next session starts on the standby reference.
    probe.Start("walker");
    Require(probe.UntilPhase(PlannerPhase::RUNNING), "the planner did not start again");
    probe.Restart();
    probe.Tick();
    Near(probe.goal[0], 0.2, "a new RL session starts on the standby reference");
    Require(probe.planner().sequence == 0 && !probe.adapter->PlannerStatus()->request_accepted,
        "a new RL session starts without planner requests");
    script.Reset();
}

}  // namespace

int main() {
    try {
        const Fixture fixture;
        TestContract(fixture);
        TestInvalidConfiguration(fixture);
        TestReferencePlayback(fixture);
        TestCatalog(fixture);
        TestPlanners(fixture);
        std::cout << "SuperMimic history, reference playback, catalog, planner and static goal tests passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "SuperMimic test failed: " << error.what() << '\n';
        return 1;
    }
}
