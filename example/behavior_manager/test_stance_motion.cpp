/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_stance_motion.cpp
 * @brief Verify routed inference, command ramps and shared history with a probe model.
 */

#include <unistd.h>
#include <cnpy.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "policy_adapter/policy_adapter.h"

namespace {

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

void Near(double actual, double expected, const std::string &message) {
    Require(std::abs(actual - expected) < 1e-6, message);
}

// Two-joint probe: q as actions, external command/history as exposed outputs.
const char kProbe[] =
    "CAg6xAIKLAoDb2JzCgdpbmRpY2VzEgdhY3Rpb25zIgZHYXRoZXIqCwoEYXhpcxgBoAECCiIK"
    "CGNvbW1hbmRzEgxzZWVuX2NvbW1hbmQiCElkZW50aXR5CiAKBm1lbW9yeRIMc2Vlbl9oaXN0"
    "b3J5IghJZGVudGl0eRITc3RhbmNlX21vdGlvbl9wcm9iZSoRCAIQBzoCCQpCB2luZGljZXNa"
    "FQoDb2JzEg4KDAgBEggKAggBCgIID1oaCghjb21tYW5kcxIOCgwIARIICgIIAQoCCANaGAoG"
    "bWVtb3J5Eg4KDAgBEggKAggBCgIIHmIZCgdhY3Rpb25zEg4KDAgBEggKAggBCgIIAmIeCgxz"
    "ZWVuX2NvbW1hbmQSDgoMCAESCAoCCAEKAggDYh4KDHNlZW5faGlzdG9yeRIOCgwIARIICgII"
    "AQoCCB5CBAoAEBE=";

class ModelFixture {
public:
    ModelFixture() {
        char path[] = "/tmp/stance_motion_XXXXXX";
        const int fd = mkstemp(path);
        Require(fd >= 0, "cannot create fixture");
        close(fd);
        path_ = path;
        std::ofstream output(path_, std::ios::binary);
        const std::string alphabet =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        unsigned int value = 0;
        int bits = -8;
        for (char c : std::string(kProbe)) {
            const auto code = alphabet.find(c);
            if (code == std::string::npos) break;
            value = (value << 6) | static_cast<unsigned int>(code);
            bits += 6;
            if (bits >= 0) {
                output.put(static_cast<char>((value >> bits) & 255));
                bits -= 8;
            }
        }
    }
    ~ModelFixture() {
        std::filesystem::remove(path_);
        std::filesystem::remove(path_ + ".npz");
    }
    std::string path_;
};

void Test() {
    using behavior_manager::policy_adapter::Config;
    using behavior_manager::policy_adapter::Create;
    const ModelFixture model;
    rl_policy::PolicyExecutorConfig policy_config;
    policy_config.model_path = model.path_;
    policy_config.rl_default_pos = {0.1, 0.2};
    policy_config.action_joint_index = {0, 1};
    policy_config.action_scale = {0.5, 0.25};
    policy_config.custom_array_dims["proprio"] = 15;
    rl_policy::ObsSegmentConfig segment;
    segment.terms = {"proprio"};
    policy_config.obs_segments = {segment};
    using Source = rl_policy::ModelInputSource;
    using Target = rl_policy::ModelOutputTarget;
    for (const auto &entry : std::vector<std::pair<std::string, Source>>{
            {"obs", Source::OBSERVATION}, {"commands", Source::EXTERNAL},
            {"memory", Source::EXTERNAL}}) {
        rl_policy::ModelInputBindingConfig input;
        input.name = entry.first;
        input.source = entry.second;
        policy_config.model_io.inputs.push_back(input);
    }
    for (const auto &entry : std::vector<std::pair<std::string, Target>>{
            {"actions", Target::ACTION}, {"seen_command", Target::EXPOSE},
            {"seen_history", Target::EXPOSE}}) {
        rl_policy::ModelOutputBindingConfig output;
        output.name = entry.first;
        output.target = entry.second;
        policy_config.model_io.outputs.push_back(output);
    }
    Config config;
    config.type = "stance_motion";
    auto &settings = config.stance_motion;
    settings.motion_model_path = model.path_;
    settings.observation_term = "proprio";
    settings.command_input = "commands";
    settings.history_input = "memory";
    settings.history_frames = 2;
    settings.command_up = {0.1, 0.1, 0.1};
    settings.command_down = {0.05, 0.05, 0.05};
    settings.stand_threshold = {0.2, 0.2, 0.3};
    settings.forward_threshold = {0.6, 1.5};
    settings.turn_scale = 2.0;
    auto adapter = Create(config, policy_config);
    rl_policy::PolicyExecutor primary;
    primary.Init(policy_config);
    auto robot = robot_base::RobotData::Create(2);
    robot.joint_pos = {0.4, 0.8};
    robot.joint_vel = {0.3, -0.2};
    robot.gyro = {0.1, 0.2, 0.3};
    adapter->Reset(robot);
    Eigen::VectorXf obs;
    std::vector<double> action;
    double elapsed = 0.0;
    auto tick = [&](const std::array<double, 3> &command) -> rl_policy::PolicyExecutor & {
        auto &executor = adapter->SelectExecutor(command, primary);
        adapter->PrepareInputs(robot, elapsed, executor);
        executor.AssembleObs(robot.gyro, robot.rpy, command[0], command[1], command[2],
            robot.joint_pos, robot.joint_vel, robot.base_quat, {}, 0.02, obs);
        executor.Infer(obs, action);
        adapter->OnAction(action);
        return executor;
    };
    auto &first = tick({0, 0, 0});
    Require(&first == &primary, "initial stance executor");
    Require(adapter->ReadyForPolicySwitch(), "stationary adapter should allow handoff");
    Near(obs[9], 0.4, "positions must be absolute");
    Near(obs[11], 0.3, "unscaled velocity");
    const auto first_history = first.GetModelOutput("seen_history");
    for (int i = 0; i < 15; ++i) {
        Near(first_history[i], obs[i], "seed oldest frame");
        Near(first_history[15 + i], obs[i], "seed newest frame");
    }
    Require(&tick({0.5, 0, 0}) == &primary, "ramp below threshold");
    Require(&tick({0.5, 0, 0}) == &primary, "one-tick transition delay");
    auto &moving = tick({0.5, 0, 0});
    Require(&moving != &primary, "motion executor selected");
    Require(!adapter->ReadyForPolicySwitch(), "moving adapter must delay handoff");
    Near(moving.GetModelOutput("seen_command")[0], 0.3, "command ramp");
    Near(obs[13], 0.2, "previous scaled action survives model switch");
    Near(obs[14], 0.2, "per-joint action scales");
    Near(moving.GetModelOutput("seen_history")[13], 0.2, "old history survives switch");
    robot.joint_pos[0] = 0.9;
    auto &same = tick({0.5, 0, 0});
    Require(&same == &moving, "motion executor must be reused");
    Near(same.GetModelOutput("seen_history")[9], 0.4, "oldest history frame");
    Near(same.GetModelOutput("seen_history")[24], 0.9, "current history frame");
    for (int i = 0; i < 15; ++i) tick({0, 0, 0});
    Require(adapter->ActiveModelName() == "stance", "stop returns to stance");
    Require(adapter->ReadyForPolicySwitch(), "stopping should allow handoff");
    for (int i = 0; i < 8; ++i) tick({0, 0, 0.5});
    Require(adapter->ActiveModelName() == "turn", "pure-turn stance route");
    Require(!adapter->ReadyForPolicySwitch(), "turning stance model is not stationary");
    Near(primary.GetModelOutput("seen_command")[2], 1.0, "turn scale");
    adapter->Reset(robot);
    tick({0, 0, 0});
    Near(obs[13], 0.0, "RL re-entry resets previous action");
    robot.base_quat = {std::cos(0.2), 0, std::sin(0.2), 0};
    tick({0, 0, 0});
    Near(obs[3], std::cos(0.4), "rotation first column x");
    Near(obs[7], -std::sin(0.4), "rotation first column z");
    Near(obs[6], 1.0, "rotation second column y");
    settings.command_up = {2.0, 2.0, 2.0};
    settings.command_down = {2.0, 2.0, 2.0};
    adapter = Create(config, policy_config);
    adapter->Reset(robot);
    for (int i = 0; i < 4; ++i) tick({1.8, 0, 0});
    for (int i = 0; i < 3; ++i) {
        Require(&tick({0, 0, 0}) != &primary, "run/fast/slow deceleration sequence");
    }
    Require(&tick({0, 0, 0}) == &primary, "deceleration returns to stance");
    Require(adapter->GetInteractionStatus().action.empty(),
        "automatic model routing must not masquerade as an interaction request");

    const float positions[] = {0.6F, 0.8F, 0.7F};
    cnpy::npz_save(model.path_ + ".npz", "joint_pos", positions, {3, 1}, "w");
    auto &trajectory = config.joint_trajectory;
    trajectory.blend_in_duration = 0.2;
    trajectory.blend_out_duration = 0.2;
    behavior_manager::policy_adapter::JointTrajectoryActionConfig clip;
    clip.name = "wave";
    clip.file = model.path_ + ".npz";
    clip.joint_indices = {1};
    clip.motion_fps = 5;
    clip.hold_last_frame = true;
    clip.max_hold_duration = 0.3;
    clip.kp = {20};
    clip.kd = {2};
    trajectory.actions = {clip};
    adapter = Create(config, policy_config);
    adapter->Reset(robot);
    tick({0.5, 0, 0});
    using Operation = robot_base::InteractionRequest::Operation;
    using Phase = robot_base::InteractionStatus::Phase;
    robot_base::InteractionRequest request;
    request.sequence = 1;
    request.operation = Operation::START;
    request.action = "wave";
    adapter->HandleInteractionRequest(request, elapsed);
    Require(!adapter->GetInteractionStatus().request_accepted,
        "must stop commanded walking before starting an arm action");
    tick({0, 0, 0});
    tick({0, 0, 0});
    adapter->HandleInteractionRequest(request, elapsed);
    Require(!adapter->GetInteractionStatus().request_accepted,
        "rejected request must not replay automatically after stopping");
    request.sequence = 2;
    elapsed = 0.1;
    adapter->HandleInteractionRequest(request, elapsed);
    tick({0, 0, 0});
    Near(action[1], 0.8, "entry must begin at last applied policy action");
    elapsed = 0.2;
    tick({0, 0, 0});
    Near(action[1], 1.2, "arm target blends in normalized action space");
    elapsed = 0.3;
    tick({0, 0, 0});
    Near(action[1], 1.6, "first arm keyframe");
    Near(action[0], robot.joint_pos[0], "non-arm action unchanged");
    elapsed = 0.32;
    Require(&tick({1, 0, 0}) == &primary, "gesture must pin stance despite motion requests");
    Near(obs[14], 0.2, "history must contain scaled network output before trajectory override");
    Near(primary.GetModelOutput("seen_history")[29], 0.2,
        "newest memory frame must preserve network action history during playback");
    Require(std::abs(action[1] * 0.25 - obs[14]) > 0.1,
        "composed gesture must differ from model action history");
    std::vector<double> kp = {4, 6}, kd = {0.2, 0.3};
    adapter->ApplyJointGains(kp, kd);
    Near(kp[0], 4, "leg gain untouched");
    Near(kp[1], 20, "configured arm stiffness");
    Near(kd[1], 2, "configured arm damping");
    request.sequence = 3;
    request.operation = Operation::CANCEL;
    adapter->HandleInteractionRequest(request, elapsed);
    tick({1, 0, 0});
    elapsed = 0.42;
    tick({1, 0, 0});
    Near(obs[14], 0.2, "cancel must not feed composed trajectory back into model history");
    Near(primary.GetModelOutput("seen_history")[14], 0.2,
        "oldest memory frame retains network action history during release");
    kp = {4, 6};
    kd = {0.2, 0.3};
    adapter->ApplyJointGains(kp, kd);
    Near(kp[1], 13, "cancel blends arm gain back to policy");
    elapsed = 0.53;
    tick({1, 0, 0});
    Require(adapter->GetInteractionStatus().phase == Phase::FINISHED, "cancel must finish");
    Require(&tick({1, 0, 0}) == &primary, "held motion input must not restart walking");
    tick({0, 0, 0});
    tick({1, 0, 0});
    Require(&tick({1, 0, 0}) != &primary, "fresh input resumes walking after release");
    adapter->Reset(robot);
    elapsed = 0;
    tick({0, 0, 0});
    request.sequence = 4;
    request.operation = Operation::START;
    adapter->HandleInteractionRequest(request, elapsed);
    elapsed = 0.8;
    tick({0, 0, 0});
    Require(adapter->GetInteractionStatus().phase == Phase::HOLDING, "handshake holds last frame");
    elapsed = 1.0;
    tick({0, 0, 0});
    Require(adapter->GetInteractionStatus().phase == Phase::BLEND_OUT, "finite hold auto-releases");
    elapsed = 1.21;
    tick({0, 0, 0});
    Require(adapter->GetInteractionStatus().phase == Phase::FINISHED, "natural release finishes");
    kp = {4, 6};
    kd = {0.2, 0.3};
    adapter->ApplyJointGains(kp, kd);
    Near(kp[1], 6, "policy gains restored after release");
    adapter->Reset(robot);
    Require(adapter->GetInteractionStatus().phase == Phase::IDLE, "state exit clears gesture");

    // Configured takeover contract: only q is substituted. Real dq and scaled
    // network action history remain intact, including the zero-weight release tick.
    settings.interaction_joint_indices = {1};
    settings.interaction_neutral_positions = {0.5};
    settings.interaction_release_frames = 10;
    trajectory.measured_entry_max_velocity = 2.0;
    trajectory.linear_blend = true;
    adapter = Create(config, policy_config);
    adapter->Reset(robot);
    elapsed = 0;
    tick({0, 0, 0});
    request.sequence = 1;
    request.operation = Operation::START;
    adapter->HandleInteractionRequest(request, elapsed);
    tick({0, 0, 0});
    Near(obs[10], 0.5, "takeover substitutes configured absolute upper-body q");
    Near(obs[12], -0.2, "takeover preserves measured upper-body dq");
    Near(obs[9], robot.joint_pos[0], "leg observation untouched");
    Near(action[1], 2.4, "measured entry starts at actual q, not previous policy target");
    kp = {4, 6};
    kd = {0.2, 0.3};
    adapter->ApplyJointGains(kp, kd);
    Near(kp[1], 20, "measured entry immediately uses configured trajectory gains");
    elapsed = 0.05;
    tick({0, 0, 0});
    Near(action[1], 2.0, "entry moves linearly at configured 2 rad/s");
    Near(obs[14], 0.125, "history stores scaled network output, not neutral target");
    elapsed = 0.1;
    tick({0, 0, 0});
    Near(action[1], 1.6, "distance-based entry reaches first frame in 0.1 s");
    request.sequence = 2;
    request.operation = Operation::CANCEL;
    adapter->HandleInteractionRequest(request, elapsed);
    elapsed = 0.15;
    tick({0, 0, 0});
    Near(action[1], 1.5, "release is linear toward neutral target, not raw model output");
    kp = {4, 6};
    kd = {0.2, 0.3};
    adapter->ApplyJointGains(kp, kd);
    Near(kp[1], 16.5, "release gain uses same linear weight as position");
    elapsed = 0.301;
    tick({0, 0, 0});
    Near(obs[10], 0.8, "zero takeover weight immediately restores real q observation");
    Near(action[1], 1.2, "upper-body output stays neutral on first release tick");
    Near(primary.GetModelOutput("seen_history")[25], 0.8,
        "newest history frame uses real q after zero weight");
    for (int i = 2; i <= 9; ++i) {
        elapsed += 0.02;
        tick({0, 0, 0});
        Near(action[1], 1.2, "neutral output persists through ninth release tick");
        Near(obs[14], 0.2, "post-release history continues raw model output");
    }
    elapsed += 0.02;
    tick({0, 0, 0});
    Near(action[1], 0.8, "tenth zero-weight tick releases output back to model");
    Near(action[0], robot.joint_pos[0], "takeover never overrides legs");
    adapter->Reset(robot);
    tick({0, 0, 0});
    Near(obs[10], 0.8, "reset clears observation takeover");
    Near(action[1], 0.8, "reset clears release hold");
}

}  // namespace

int main() {
    try {
        Test();
        std::cout << "stance_motion contract passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
