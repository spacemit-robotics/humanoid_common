/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file stance_motion_policy_adapter.cpp
 * @brief Command-routed stance/motion models sharing observation and action history.
 */

#include "stance_motion_policy_adapter.h"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "runtime_logger.h"
#include "joint_trajectory_policy_adapter.h"

namespace behavior_manager {
namespace policy_adapter {
namespace {

using Settings = Config::StanceMotionConfig;

template <std::size_t N>
std::array<double, N> ReadArray(const YAML::Node &node, const char *key) {
    const auto values = node[key].as<std::vector<double>>();
    if (values.size() != N || !std::all_of(values.begin(), values.end(),
            [](double value) { return std::isfinite(value); })) {
        throw std::runtime_error(std::string("[stance_motion] invalid ") + key);
    }
    std::array<double, N> result;
    std::copy(values.begin(), values.end(), result.begin());
    return result;
}

class StanceMotionPolicyAdapter final : public PolicyAdapter {
public:
    StanceMotionPolicyAdapter(const Config &config,
        const rl_policy::PolicyExecutorConfig &policy_config)
        : settings_(config.stance_motion), policy_config_(policy_config) {
        const auto &indices = policy_config_.action_joint_index;
        if (indices.empty() || settings_.history_frames <= 0 ||
            settings_.history_frames > 1000 || policy_config_.action_blend_ratio != 1.0) {
            throw std::runtime_error("[stance_motion] invalid history, indices or action blend");
        }
        for (int index : indices) {
            if (index < 0 || index >= static_cast<int>(policy_config_.rl_default_pos.size())) {
                throw std::runtime_error("[stance_motion] joint index out of range");
            }
        }
        if (settings_.interaction_joint_indices.size() !=
                settings_.interaction_neutral_positions.size() ||
            settings_.interaction_release_frames < 0 || settings_.interaction_release_frames > 1000) {
            throw std::runtime_error("[stance_motion] invalid interaction observation settings");
        }
        for (std::size_t i = 0; i < settings_.interaction_joint_indices.size(); ++i) {
            const int joint = settings_.interaction_joint_indices[i];
            const auto found = std::find(indices.begin(), indices.end(), joint);
            const auto &joints = settings_.interaction_joint_indices;
            if (found == indices.end() || std::find(joints.begin(), joints.begin() + i, joint) !=
                    joints.begin() + i || !std::isfinite(settings_.interaction_neutral_positions[i])) {
                throw std::runtime_error("[stance_motion] invalid interaction joint/position");
            }
            const auto action_index = static_cast<std::size_t>(found - indices.begin());
            const auto &scales = policy_config_.action_scale;
            const double scale = scales.at(scales.size() == 1 ? 0 : action_index);
            if (!std::isfinite(scale) || scale == 0.0)
                throw std::runtime_error("[stance_motion] invalid interaction action scale");
            interaction_action_indices_.push_back(action_index);
            interaction_neutral_actions_.push_back((settings_.interaction_neutral_positions[i] -
                policy_config_.rl_default_pos[joint]) / scale);
        }
        if (settings_.orientation_frame != "world" &&
            settings_.orientation_frame != "heading" &&
            settings_.orientation_frame != "initial_heading") {
            throw std::runtime_error("[stance_motion] unknown orientation_frame");
        }
        for (int axis = 0; axis < 3; ++axis) {
            if (settings_.command_up[axis] <= 0 || settings_.command_down[axis] <= 0 ||
                settings_.stand_threshold[axis] <= 0) {
                throw std::runtime_error("[stance_motion] command steps/thresholds must be positive");
            }
        }
        if (settings_.forward_threshold[0] <= settings_.stand_threshold[0] ||
            settings_.forward_threshold[1] <= settings_.forward_threshold[0] ||
            !std::isfinite(settings_.turn_scale) || settings_.turn_scale <= 0) {
            throw std::runtime_error("[stance_motion] invalid mode thresholds or turn scale");
        }
        const std::size_t size = (settings_.gravity_observation ? 6 : 9) + 3 * indices.size();
        observation_.resize(size);
        memory_.resize(size * settings_.history_frames);
        previous_action_.assign(indices.size(), 0.0);
        auto motion_config = policy_config_;
        motion_config.model_path = settings_.motion_model_path;
        motion_.Init(motion_config);
        if (!settings_.run_model_path.empty()) {
            motion_config.model_path = settings_.run_model_path;
            run_.Init(motion_config);
            runtime_logging::RecordArtifact("policy_aux_model", "run", settings_.run_model_path);
        }
        runtime_logging::RecordArtifact("policy_aux_model", "motion", settings_.motion_model_path);
        if (motion_.ObsDim() != static_cast<int>(size) ||
            motion_.ActionDim() != static_cast<int>(indices.size())) {
            throw std::runtime_error("[stance_motion] incompatible motion model dimensions");
        }
        if (config.joint_trajectory.Enabled()) {
            trajectory_ = CreateJointTrajectoryPolicyAdapter(config, policy_config);
        }
    }

    const char *Type() const override { return "stance_motion"; }

    void Reset(const robot_base::RobotData &robot) override {
        filtered_command_ = {};
        model_command_ = settings_.stance_command;
        mode_ = 0;
        active_name_.clear();
        requested_command_ = {};
        last_request_sequence_ = 0;
        rejected_request_ = {};
        wait_for_zero_ = false;
        interaction_observation_active_ = false;
        interaction_release_remaining_ = 0;
        if (trajectory_) trajectory_->Reset(robot);
        initial_yaw_ = robot.rpy[2];
        std::fill(previous_action_.begin(), previous_action_.end(), 0.0);
        BuildObservation(robot);
        for (int frame = 0; frame < settings_.history_frames; ++frame) {
            std::copy(observation_.begin(), observation_.end(),
                memory_.begin() + frame * observation_.size());
        }
    }

    rl_policy::PolicyExecutor &SelectExecutor(const std::array<double, 3> &command,
        rl_policy::PolicyExecutor &primary) override {
        requested_command_ = command;
        if (trajectory_) {
            const bool zero = std::all_of(command.begin(), command.end(),
                [](double value) { return std::isfinite(value) && std::abs(value) < 1e-9; });
            if (InteractionBusy()) wait_for_zero_ = true;
            else if (zero) wait_for_zero_ = false;
            if (InteractionBusy() || wait_for_zero_) {
                filtered_command_ = {};
                model_command_ = settings_.stance_command;
                mode_ = 0;
                active_name_ = "stance";
                return primary;
            }
        }
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(command[axis])) {
                throw std::runtime_error("[stance_motion] nonfinite command");
            }
            double &current = filtered_command_[axis];
            const double delta = command[axis] - current;
            const bool accelerating = current == 0.0 || current * delta >= 0.0;
            const double step = accelerating ? settings_.command_up[axis]
                                            : settings_.command_down[axis];
            current += std::clamp(delta, -step, step);
        }
        const auto &threshold = settings_.stand_threshold;
        const bool linear_zero = std::abs(filtered_command_[0]) < threshold[0] &&
            std::abs(filtered_command_[1]) < threshold[1];
        const bool near_zero = linear_zero && std::abs(filtered_command_[2]) < threshold[2];
        // Select using the old state; transitions take effect on the next tick.
        const int selected_mode = mode_;
        bool stance = selected_mode == 0;
        model_command_ = filtered_command_;
        if (selected_mode == 0) {
            model_command_ = settings_.stance_command;
            if (!near_zero) mode_ = 1;
        } else if (selected_mode == 1) {
            if (filtered_command_[0] > settings_.forward_threshold[0]) mode_ = 2;
            else if (near_zero) mode_ = 0;
            if (linear_zero && std::abs(filtered_command_[2]) > threshold[2]) {
                stance = true;
                model_command_[2] *= settings_.turn_scale;
            }
        } else if (selected_mode == 2) {
            if (filtered_command_[0] < settings_.forward_threshold[0]) mode_ = 1;
            else if (filtered_command_[0] > settings_.forward_threshold[1]) mode_ = 3;
        } else if (filtered_command_[0] < settings_.forward_threshold[1]) {
            mode_ = 2;
        }
        const bool run = !stance && selected_mode == 3 && !settings_.run_model_path.empty();
        const std::string name = stance ? (selected_mode == 0 ? "stance" : "turn") : run ? "run" : "motion";
        if (name != active_name_) {
            active_name_ = name;
            runtime_logging::Log(runtime_logging::Level::kInfo,
                "[stance_motion] " + name + " command=" + std::to_string(model_command_[0]) +
                "," + std::to_string(model_command_[1]) + "," + std::to_string(model_command_[2]));
        }
        return stance ? primary : run ? run_ : motion_;
    }

    void PrepareInputs(const robot_base::RobotData &robot, double elapsed_s,
        rl_policy::PolicyExecutor &policy) override {
        if (trajectory_) trajectory_->PrepareInputs(robot, elapsed_s, policy);
        interaction_observation_active_ = trajectory_ && trajectory_->InteractionWeight() > 0.0;
        if (interaction_observation_active_) {
            interaction_release_remaining_ = settings_.interaction_release_frames;
        } else if (interaction_release_remaining_ > 0) {
            --interaction_release_remaining_;
        }
        BuildObservation(robot);
        std::move(memory_.begin() + observation_.size(), memory_.end(), memory_.begin());
        std::copy(observation_.begin(), observation_.end(), memory_.end() - observation_.size());
        const std::array<float, 3> commands = {static_cast<float>(model_command_[0]),
            static_cast<float>(model_command_[1]), static_cast<float>(model_command_[2])};
        policy.SetModelInput(settings_.command_input, commands.data(), commands.size());
        policy.SetModelInput(settings_.history_input, memory_.data(), memory_.size());
        policy.SetCustomArray(settings_.observation_term, observation_.data(), observation_.size());
    }

    void OnAction(std::vector<double> &action) override {
        if (action.size() != previous_action_.size()) {
            throw std::runtime_error("[stance_motion] action size mismatch");
        }
        // Model history contains the network output, before trajectory composition.
        for (std::size_t i = 0; i < action.size(); ++i) {
            const auto &scale = policy_config_.action_scale;
            previous_action_[i] = action[i] * (settings_.scaled_action_history
                ? scale.at(scale.size() == 1 ? 0 : i) : 1.0);
        }
        // The model's own upper-body targets are neutral while an external
        // trajectory owns these joints, then held briefly after its weight is zero.
        // Never put this override (or the trajectory) into network action history.
        if (interaction_observation_active_ || interaction_release_remaining_ > 0) {
            for (std::size_t i = 0; i < interaction_action_indices_.size(); ++i)
                action[interaction_action_indices_[i]] = interaction_neutral_actions_[i];
        }
        if (trajectory_) trajectory_->OnAction(action);
    }

    void RequestInferenceTermination() override {
        motion_.RequestInferenceTermination();
        if (!settings_.run_model_path.empty()) run_.RequestInferenceTermination();
    }

    bool ReadyForPolicySwitch() const override {
        return active_name_ == "stance" && mode_ == 0 && !InteractionBusy() && !wait_for_zero_;
    }

    std::string ActiveModelName() const override { return active_name_; }

    void HandleInteractionRequest(const robot_base::InteractionRequest &request,
        double elapsed_s) override {
        using Operation = robot_base::InteractionRequest::Operation;
        if (!trajectory_ || request.operation == Operation::NONE ||
            request.sequence == 0 || request.sequence <= last_request_sequence_) return;
        last_request_sequence_ = request.sequence;
        rejected_request_ = {};
        const bool zero = std::all_of(requested_command_.begin(), requested_command_.end(),
            [](double value) { return std::isfinite(value) && std::abs(value) < 1e-9; });
        // Do not freeze walking legs to begin a gesture. The operator must stop first.
        if (request.operation == Operation::START && !InteractionBusy() &&
            (active_name_ != "stance" || !zero || wait_for_zero_)) {
            rejected_request_.sequence = request.sequence;
            rejected_request_.action = request.action;
            rejected_request_.phase = robot_base::InteractionStatus::Phase::REJECTED;
            rejected_request_.request_accepted = false;
            return;
        }
        trajectory_->HandleInteractionRequest(request, elapsed_s);
    }

    robot_base::InteractionStatus GetInteractionStatus() const override {
        if (rejected_request_.sequence != 0) return rejected_request_;
        return trajectory_ ? trajectory_->GetInteractionStatus() : robot_base::InteractionStatus{};
    }

    void ApplyJointGains(std::vector<double> &kp, std::vector<double> &kd) const override {
        if (trajectory_) trajectory_->ApplyJointGains(kp, kd);
    }

private:
    bool InteractionBusy() const {
        if (!trajectory_) return false;
        using Phase = robot_base::InteractionStatus::Phase;
        const auto phase = trajectory_->GetInteractionStatus().phase;
        return phase == Phase::BLEND_IN || phase == Phase::PLAYING ||
            phase == Phase::HOLDING || phase == Phase::BLEND_OUT;
    }

    void BuildObservation(const robot_base::RobotData &robot) {
        Eigen::Quaterniond quaternion(robot.base_quat[0], robot.base_quat[1],
            robot.base_quat[2], robot.base_quat[3]);
        if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-8) {
            throw std::runtime_error("[stance_motion] invalid orientation");
        }
        Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();
        if (settings_.orientation_frame != "world") {
            const double yaw = settings_.orientation_frame == "heading" ? robot.rpy[2] : initial_yaw_;
            rotation = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * rotation;
        }
        if (settings_.transpose_rotation) rotation.transposeInPlace();
        for (int i = 0; i < 3; ++i) observation_[i] = robot.gyro[i];
        if (settings_.gravity_observation) {
            for (int col = 0; col < 3; ++col) observation_[3 + col] = -rotation(2, col);
        } else {
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 2; ++col) observation_[3 + row * 2 + col] = rotation(row, col);
            }
        }
        const std::size_t prefix = settings_.gravity_observation ? 6 : 9;
        const auto &indices = policy_config_.action_joint_index;
        for (std::size_t i = 0; i < indices.size(); ++i) {
            const std::size_t joint = indices[i];
            observation_[prefix + i] = robot.joint_pos.at(joint);
            observation_[prefix + indices.size() + i] = robot.joint_vel.at(joint);
            observation_[prefix + 2 * indices.size() + i] = previous_action_[i];
        }
        if (interaction_observation_active_) {
            for (std::size_t i = 0; i < interaction_action_indices_.size(); ++i)
                observation_[prefix + interaction_action_indices_[i]] =
                    settings_.interaction_neutral_positions[i];
        }
        if (!std::all_of(observation_.begin(), observation_.end(),
                [](float value) { return std::isfinite(value); })) {
            throw std::runtime_error("[stance_motion] nonfinite observation");
        }
    }

    Settings settings_;
    rl_policy::PolicyExecutorConfig policy_config_;
    rl_policy::PolicyExecutor motion_;
    rl_policy::PolicyExecutor run_;
    std::vector<float> observation_;
    std::vector<float> memory_;
    std::vector<double> previous_action_;
    std::array<double, 3> filtered_command_ = {};
    std::array<double, 3> model_command_ = {};
    int mode_ = 0;
    double initial_yaw_ = 0.0;
    std::string active_name_;
    std::unique_ptr<PolicyAdapter> trajectory_;
    std::array<double, 3> requested_command_ = {};
    uint64_t last_request_sequence_ = 0;
    robot_base::InteractionStatus rejected_request_;
    bool wait_for_zero_ = false;
    std::vector<std::size_t> interaction_action_indices_;
    std::vector<double> interaction_neutral_actions_;
    bool interaction_observation_active_ = false;
    int interaction_release_remaining_ = 0;
};

}  // namespace

Config::StanceMotionConfig LoadStanceMotionConfig(const std::string &yaml_path,
    const std::string &base, const std::string &robot_dir) {
    YAML::Node node = YAML::LoadFile(yaml_path);
    std::size_t begin = 0;
    while (begin < base.size()) {
        const auto end = base.find('.', begin);
        node.reset(node[base.substr(begin, end - begin)]);
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    Settings config;
    config.motion_model_path = std::filesystem::weakly_canonical(
        std::filesystem::path(robot_dir) / node["motion_model_path"].as<std::string>()).string();
    if (node["run_model_path"]) config.run_model_path = std::filesystem::weakly_canonical(
        std::filesystem::path(robot_dir) / node["run_model_path"].as<std::string>()).string();
    config.gravity_observation = node["gravity_observation"].as<bool>(false);
    config.scaled_action_history = node["scaled_action_history"].as<bool>(true);
    config.observation_term = node["observation_term"].as<std::string>();
    config.command_input = node["command_input"].as<std::string>();
    config.history_input = node["history_input"].as<std::string>();
    config.history_frames = node["history_frames"].as<int>();
    config.command_up = ReadArray<3>(node, "command_up_delta");
    config.command_down = ReadArray<3>(node, "command_down_delta");
    config.stand_threshold = ReadArray<3>(node, "stand_threshold");
    config.stance_command = ReadArray<3>(node, "stance_command");
    config.forward_threshold = ReadArray<2>(node, "forward_mode_thresholds");
    config.turn_scale = node["turn_scale"].as<double>();
    config.orientation_frame = node["orientation_frame"].as<std::string>();
    config.transpose_rotation = node["transpose_rotation"].as<bool>();
    if (node["interaction_observation"]) {
        const auto interaction = node["interaction_observation"];
        config.interaction_joint_indices = interaction["joint_indices"].as<std::vector<int>>();
        config.interaction_neutral_positions = interaction["neutral_positions"].as<std::vector<double>>();
        config.interaction_release_frames = interaction["release_frames"].as<int>();
    }
    return config;
}

std::unique_ptr<PolicyAdapter> CreateStanceMotionPolicyAdapter(const Config &config,
    const rl_policy::PolicyExecutorConfig &policy_config) {
    return std::make_unique<StanceMotionPolicyAdapter>(config, policy_config);
}

}  // namespace policy_adapter
}  // namespace behavior_manager
