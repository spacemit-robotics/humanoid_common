/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file history_actor_policy_adapter.cpp
 * @brief Proprioceptive history with phase commands or a selectable latent motion decoder.
 */

#include "history_actor_policy_adapter.h"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "runtime_logger.h"

namespace behavior_manager {
namespace policy_adapter {
namespace {
using Settings = Config::HistoryActorConfig;
using Phase = robot_base::InteractionStatus::Phase;

std::vector<std::vector<float>> ReadLatents(const std::string &path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot read latent motion: " + path);
    std::vector<std::vector<float>> frames;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream values(line);
        std::vector<float> frame;
        float value;
        while (values >> value) {
            if (!std::isfinite(value)) throw std::runtime_error("Nonfinite latent: " + path);
            frame.push_back(value);
        }
        if (!values.eof() || frame.empty() || (!frames.empty() && frame.size() != frames[0].size()))
            throw std::runtime_error("Invalid latent CSV: " + path);
        frames.push_back(std::move(frame));
    }
    if (frames.empty()) throw std::runtime_error("Empty latent motion: " + path);
    return frames;
}

class HistoryActorPolicyAdapter final : public PolicyAdapter {
public:
    HistoryActorPolicyAdapter(const Config &config, const rl_policy::PolicyExecutorConfig &policy)
        : settings_(config.history_actor), policy_(policy) {
        const auto &s = settings_;
        for (double value : {s.gyro_scale, s.velocity_scale, s.frequency_center,
                s.command_scale, s.stand_threshold}) {
            if (!std::isfinite(value)) throw std::runtime_error("Nonfinite history_actor parameter");
        }
        if ((s.command_type != "gait" && s.command_type != "latent") ||
            s.history_frames < 1 || s.history_frames > 100 ||
            !std::isfinite(s.dt) || s.dt <= 0 || !std::isfinite(s.observation_clip) ||
            s.observation_clip <= 0 || !std::isfinite(s.gait_frequency) || s.gait_frequency <= 0 ||
            !std::isfinite(s.stance_ratio) || s.stance_ratio <= 0 || s.stance_ratio >= 1 ||
            !std::isfinite(s.transition_duration) || s.transition_duration <= 0 ||
            !std::isfinite(s.root_position_error_clip) || s.root_position_error_clip <= 0 ||
            !std::isfinite(s.command_filter_alpha) || s.command_filter_alpha <= 0 ||
            s.command_filter_alpha > 1 || policy.action_blend_ratio != 1.0)
            throw std::runtime_error("Invalid history_actor configuration");
        const auto count = policy.action_joint_index.size();
        if (!count || (policy.action_scale.size() != 1 && policy.action_scale.size() != count))
            throw std::runtime_error("Invalid history_actor action mapping");
        for (int index : policy.action_joint_index) {
            if (index < 0 || static_cast<std::size_t>(index) >= policy.rl_default_pos.size())
                throw std::runtime_error("Invalid history_actor joint index");
        }
        previous_.resize(count);
        obs_.resize(6 + 3 * count);
        history_.resize(obs_.size() * s.history_frames);
        if (s.command_type == "latent") {
            if (s.motions.empty()) throw std::runtime_error("Missing latent motion catalog");
            settings_.motions[0].model_path = policy.model_path;
            std::map<std::string, std::shared_ptr<rl_policy::PolicyExecutor>> models;
            for (std::size_t i = 0; i < s.motions.size(); ++i) {
                const auto &motion = s.motions[i];
                auto frames = ReadLatents(motion.file);
                const double duration = frames.size() * s.dt;
                const double end_time = motion.end_time == -1.0 ? duration : motion.end_time;
                if (!std::isfinite(motion.start_time) || !std::isfinite(end_time) ||
                    motion.start_time < 0 || end_time <= motion.start_time ||
                    end_time > duration + 1e-8)
                    throw std::runtime_error("Invalid latent motion playback interval: " + motion.name);
                const auto first = static_cast<std::size_t>(std::llround(motion.start_time / s.dt));
                const auto last = static_cast<std::size_t>(std::llround(end_time / s.dt));
                if (first >= last || last > frames.size())
                    throw std::runtime_error("Empty or out-of-range latent motion interval: " + motion.name);
                latents_.emplace_back(frames.begin() + first, frames.begin() + last);
                runtime_logging::RecordArtifact("policy_latent", s.motions[i].name, s.motions[i].file);
                if (s.motions[i].root_pose && latents_[i][0].size() < 8)
                    throw std::runtime_error("Root pose motion requires latent and seven pose columns");
                standby_.push_back(s.motions[i].standby_file.empty() ? latents_[0][0] :
                    ReadLatents(s.motions[i].standby_file)[0]);
                if (standby_.back().size() != latents_[i][0].size())
                    throw std::runtime_error("Motion/standby latent widths differ");
                auto &executor = models[s.motions[i].model_path];
                if (i > 0 && !executor && s.motions[i].model_path != policy.model_path) {
                    executor = std::make_shared<rl_policy::PolicyExecutor>();
                    auto cfg = policy;
                    cfg.model_path = s.motions[i].model_path;
                    executor->Init(cfg);
                    runtime_logging::RecordArtifact("policy_aux_model", s.motions[i].name, cfg.model_path);
                }
                executors_.push_back(executor);
            }
            command_ = latents_[0][0];
        } else {
            command_.resize(8);
        }
        blend_from_ = command_;
        latent_ = command_;
        output_before_switch_.resize(count);
        last_output_.resize(count);
    }

    const char *Type() const override { return "history_actor"; }
    void Reset(const robot_base::RobotData &robot) override {
        phase_ = 0;
        filtered_ = {};
        stationary_ = true;
        current_motion_ = pending_motion_ = 0;
        switch_pending_ = false;
        motion_time_ = 0;
        status_ = {};
        last_sequence_ = 0;
        std::fill(previous_.begin(), previous_.end(), 0.0);
        std::fill(last_output_.begin(), last_output_.end(), 0.0);
        if (!latents_.empty()) command_ = latents_[0][0];
        latent_ = command_;
        cross_decoder_blend_ = false;
        blend_from_ = latent_;
        reset_anchor_ = true;
        BuildObservation(robot);
        for (int frame = 0; frame < settings_.history_frames; ++frame)
            std::copy(obs_.begin(), obs_.end(), history_.begin() + frame * obs_.size());
    }

    rl_policy::PolicyExecutor &SelectExecutor(const std::array<double, 3> &velocity,
        rl_policy::PolicyExecutor &primary) override {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(velocity[axis])) throw std::runtime_error("Invalid velocity");
            filtered_[axis] += settings_.command_filter_alpha * (velocity[axis] - filtered_[axis]);
        }
        if (settings_.command_type == "gait") {
            stationary_ = std::all_of(filtered_.begin(), filtered_.end(),
                [this](double value) { return std::abs(value) < settings_.stand_threshold; });
            return primary;
        }
        // Requests are received after SelectExecutor by StateRL. Apply them on
        // the following sample so the selected decoder and its input always agree.
        if (switch_pending_) {
            const auto old_motion = current_motion_;
            current_motion_ = pending_motion_;
            switch_pending_ = false;
            motion_time_ = 0;
            cross_decoder_blend_ = latents_[old_motion][0].size() != latents_[current_motion_][0].size() ||
                settings_.motions[old_motion].model_path != settings_.motions[current_motion_].model_path;
            blend_from_ = cross_decoder_blend_ ? standby_[current_motion_] : latent_;
            latent_ = blend_from_;
            reset_anchor_ = true;
            output_before_switch_ = last_output_;
            status_.phase = current_motion_ ? Phase::BLEND_IN : Phase::BLEND_OUT;
        }
        return executors_[current_motion_] ? *executors_[current_motion_] : primary;
    }

    void PrepareInputs(const robot_base::RobotData &robot, double,
        rl_policy::PolicyExecutor &executor) override {
        BuildObservation(robot);
        std::move(history_.begin() + obs_.size(), history_.end(), history_.begin());
        std::copy(obs_.begin(), obs_.end(), history_.end() - obs_.size());
        if (settings_.command_type == "gait") {
            const double frequency = stationary_ ? 0.0 : settings_.gait_frequency;
            phase_ = stationary_ ? 0.0 : std::fmod(phase_ + settings_.dt * frequency, 1.0);
            for (int leg = 0; leg < 2; ++leg) {
                double p = std::fmod(phase_ + leg * 0.5, 1.0);
                p = p < settings_.stance_ratio ? 0.5 * p / settings_.stance_ratio
                    : 0.5 + 0.5 * (p - settings_.stance_ratio) / (1.0 - settings_.stance_ratio);
                if (stationary_) p = 0.25;
                command_[leg] = std::sin(2 * 3.14159265358979323846 * p);
                command_[2 + leg] = std::cos(2 * 3.14159265358979323846 * p);
            }
            command_[4] = 0.5 * (frequency - settings_.frequency_center);
            for (int axis = 0; axis < 3; ++axis)
                command_[5 + axis] = stationary_ ? 0.0 : filtered_[axis] * settings_.command_scale;
        } else {
            const auto &frames = latents_[current_motion_];
            const double playback = std::max(0.0, motion_time_ - settings_.transition_duration);
            const auto frame = std::min(frames.size() - 1, static_cast<std::size_t>(playback / settings_.dt));
            const double weight = std::min(1.0, motion_time_ / settings_.transition_duration);
            for (std::size_t i = 0; i < latent_.size(); ++i)
                latent_[i] = (1.0 - weight) * blend_from_[i] + weight * frames[frame][i];
            command_ = latent_;
            if (settings_.motions[current_motion_].root_pose) PrepareRootCommand(robot, frames[0]);
            if (weight >= 1.0) {
                if (current_motion_) {
                    status_.phase = Phase::PLAYING;
                    status_.progress = static_cast<double>(frame) / frames.size();
                    if (frame + 1 == frames.size()) {
                        pending_motion_ = 0;
                        switch_pending_ = true;
                    }
                } else if (status_.phase == Phase::BLEND_OUT) {
                    status_.phase = Phase::FINISHED;
                    status_.progress = 1.0;
                }
            }
            motion_time_ += settings_.dt;
        }
        executor.SetModelInput(settings_.command_input, command_.data(), command_.size());
        executor.SetCustomArray(settings_.observation_term, history_.data(), history_.size());
    }

    void OnAction(std::vector<double> &action) override {
        if (action.size() != previous_.size()) throw std::runtime_error("History actor action size mismatch");
        for (std::size_t i = 0; i < action.size(); ++i)
            previous_[i] = action[i] * policy_.action_scale.at(policy_.action_scale.size() == 1 ? 0 : i);
        if (cross_decoder_blend_) {
            const double weight = std::clamp((motion_time_ - settings_.dt) /
                settings_.transition_duration, 0.0, 1.0);
            for (std::size_t i = 0; i < action.size(); ++i)
                action[i] = (1 - weight) * output_before_switch_[i] + weight * action[i];
        }
        last_output_ = action;
    }

    void HandleInteractionRequest(const robot_base::InteractionRequest &request, double) override {
        using Op = robot_base::InteractionRequest::Operation;
        if (settings_.command_type != "latent" || request.sequence == 0 ||
            request.sequence <= last_sequence_ || request.operation == Op::NONE) return;
        last_sequence_ = request.sequence;
        status_.sequence = request.sequence;
        status_.request_accepted = true;
        if (request.operation == Op::CANCEL) {
            if (current_motion_ == 0) {
                // Repeated cancellation must not restart a return already in
                // progress. Cancelling standby (or a not-yet-selected start)
                // is an acknowledged no-op, never an unnamed BLEND_OUT.
                if (status_.phase == Phase::BLEND_OUT) return;
                pending_motion_ = 0;
                switch_pending_ = false;
                status_ = {};
                status_.sequence = request.sequence;
                status_.request_accepted = true;
                return;
            }
            pending_motion_ = 0;
            switch_pending_ = true;
            return;
        }
        const auto it = std::find_if(settings_.motions.begin() + 1, settings_.motions.end(),
            [&](const auto &motion) { return motion.name == request.action; });
        if (!ReadyForPolicySwitch() || it == settings_.motions.end()) {
            status_.request_accepted = false;
            return;
        }
        status_.action = request.action;
        status_.progress = 0;
        pending_motion_ = it - settings_.motions.begin();
        switch_pending_ = true;
        status_.phase = Phase::BLEND_IN;
    }
    robot_base::InteractionStatus GetInteractionStatus() const override { return status_; }
    bool ReadyForPolicySwitch() const override {
        return settings_.command_type == "gait" ? stationary_ : current_motion_ == 0 &&
            !switch_pending_ && status_.phase != Phase::BLEND_OUT;
    }
    void PreparePolicySwitch(bool requested, double elapsed_s) override {
        (void)elapsed_s;
        if (requested && settings_.command_type != "gait" && current_motion_ != 0) {
            pending_motion_ = 0;
            switch_pending_ = true;
        }
    }
    std::string ActiveModelName() const override {
        return settings_.command_type == "gait" ? (stationary_ ? "stand" : "gait")
            : settings_.motions[current_motion_].name;
    }
    void RequestInferenceTermination() override {
        for (auto &executor : executors_) if (executor) executor->RequestInferenceTermination();
    }

private:
    static double Yaw(const Eigen::Quaterniond &q) {
        const auto r = q.toRotationMatrix();
        return std::atan2(r(1, 0), r(0, 0));
    }

    void PrepareRootCommand(const robot_base::RobotData &robot, const std::vector<float> &first) {
        const auto offset = latent_.size() - 7;
        const Eigen::Quaterniond body = Eigen::Quaterniond(robot.base_quat[0], robot.base_quat[1],
            robot.base_quat[2], robot.base_quat[3]).normalized();
        const auto pose = [offset](const std::vector<float> &frame) {
            Eigen::Quaterniond q(frame[offset + 3], frame[offset + 4], frame[offset + 5], frame[offset + 6]);
            if (!q.coeffs().allFinite() || q.norm() < 1e-8)
                throw std::runtime_error("Invalid motion root quaternion");
            return q.normalized();
        };
        const Eigen::Vector3d position(robot.base_pos[0], robot.base_pos[1], robot.base_pos[2]);
        if (!position.allFinite()) throw std::runtime_error("Invalid robot root position");
        if (reset_anchor_) {
            anchor_position_ = position;
            anchor_heading_ = Eigen::AngleAxisd(Yaw(body) - Yaw(pose(first)), Eigen::Vector3d::UnitZ());
            reset_anchor_ = false;
        }
        const Eigen::Quaterniond heading_inverse(Eigen::AngleAxisd(-Yaw(body), Eigen::Vector3d::UnitZ()));
        Eigen::Vector3d displacement;
        for (int i = 0; i < 3; ++i) displacement[i] = latent_[offset + i] - first[offset + i];
        // The command uses heading-relative XY error; vertical error is zero.
        Eigen::Vector3d error = heading_inverse *
            (anchor_position_ + anchor_heading_ * displacement - position);
        if (settings_.motions[current_motion_].mask_root_position) error.setZero();
        const Eigen::Quaterniond target = heading_inverse * anchor_heading_ * pose(latent_);
        const Eigen::Matrix3d rotation = (target * (heading_inverse * body).conjugate()).toRotationMatrix();
        command_.resize(offset + 9);
        command_[offset] = std::clamp(error.x(), -settings_.root_position_error_clip, settings_.root_position_error_clip);
        command_[offset + 1] = std::clamp(error.y(), -settings_.root_position_error_clip, settings_.root_position_error_clip);
        command_[offset + 2] = 0;
        // Row-major first two columns, matching the decoder's 6D rotation input.
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 2; ++col)
                command_[offset + 3 + row * 2 + col] = rotation(row, col);
    }

    void BuildObservation(const robot_base::RobotData &robot) {
        Eigen::Quaterniond rotation(robot.base_quat[0], robot.base_quat[1],
            robot.base_quat[2], robot.base_quat[3]);
        if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-8)
            throw std::runtime_error("Invalid history actor orientation");
        const auto gravity = rotation.normalized().conjugate() * Eigen::Vector3d(0, 0, -1);
        for (int axis = 0; axis < 3; ++axis) {
            obs_[axis] = robot.gyro[axis] * settings_.gyro_scale;
            obs_[3 + axis] = gravity[axis];
        }
        const auto count = previous_.size();
        for (std::size_t i = 0; i < count; ++i) {
            const auto joint = policy_.action_joint_index[i];
            obs_[6 + i] = robot.joint_pos.at(joint) - policy_.rl_default_pos.at(joint);
            obs_[6 + count + i] = robot.joint_vel.at(joint) * settings_.velocity_scale;
            obs_[6 + 2 * count + i] = previous_[i];
        }
        for (float &value : obs_) {
            if (!std::isfinite(value)) throw std::runtime_error("Nonfinite history actor observation");
            value = std::clamp(value, -static_cast<float>(settings_.observation_clip),
                static_cast<float>(settings_.observation_clip));
        }
    }

    Settings settings_;
    rl_policy::PolicyExecutorConfig policy_;
    std::vector<float> previous_, obs_, history_, command_, blend_from_, latent_;
    std::vector<double> output_before_switch_, last_output_;
    std::vector<std::vector<float>> standby_;
    std::vector<std::vector<std::vector<float>>> latents_;
    std::vector<std::shared_ptr<rl_policy::PolicyExecutor>> executors_;
    Eigen::Quaterniond anchor_heading_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d anchor_position_ = Eigen::Vector3d::Zero();
    bool reset_anchor_ = true, cross_decoder_blend_ = false;
    std::array<double, 3> filtered_{};
    double phase_ = 0, motion_time_ = 0;
    std::size_t current_motion_ = 0, pending_motion_ = 0;
    uint64_t last_sequence_ = 0;
    bool stationary_ = true, switch_pending_ = false;
    robot_base::InteractionStatus status_;
};
}  // namespace

Settings LoadHistoryActorConfig(const std::string &yaml_path,
    const std::string &base, const std::string &robot_dir) {
    const auto yaml = robot_base::YamlFile::Load(yaml_path);
    Settings s;
    s.command_type = yaml.Read<std::string>(base + ".command_type").value();
    s.observation_term = yaml.Read<std::string>(base + ".observation_term").value();
    s.command_input = yaml.Read<std::string>(base + ".command_input").value();
    s.history_frames = yaml.Read<int>(base + ".history_frames").value();
    s.dt = yaml.Read<double>("rl_policy.rl_dt").value();
    s.gyro_scale = yaml.Read<double>(base + ".gyro_scale").value_or(s.gyro_scale);
    s.velocity_scale = yaml.Read<double>(base + ".velocity_scale").value_or(s.velocity_scale);
    s.observation_clip = yaml.Read<double>(base + ".observation_clip").value_or(s.observation_clip);
    s.gait_frequency = yaml.Read<double>(base + ".gait_frequency").value_or(s.gait_frequency);
    s.frequency_center = yaml.Read<double>(base + ".frequency_center").value_or(s.frequency_center);
    s.stance_ratio = yaml.Read<double>(base + ".stance_ratio").value_or(s.stance_ratio);
    s.command_scale = yaml.Read<double>(base + ".command_scale").value_or(s.command_scale);
    s.command_filter_alpha = yaml.Read<double>(base + ".command_filter_alpha").value_or(s.command_filter_alpha);
    s.stand_threshold = yaml.Read<double>(base + ".stand_threshold").value_or(s.stand_threshold);
    s.transition_duration = yaml.Read<double>(base + ".transition_duration").value_or(s.transition_duration);
    s.root_position_error_clip = yaml.Read<double>(base + ".root_position_error_clip").value_or(s.root_position_error_clip);
    const auto path = [&](const std::string &key) {
        return std::filesystem::weakly_canonical(std::filesystem::path(robot_dir) /
            yaml.Read<std::string>(key).value()).string();
    };
    if (s.command_type == "latent") {
        s.motions.push_back({"standby", "", path(base + ".reference_file")});
        s.motions[0].root_pose = yaml.Read<bool>(base + ".reference_root_pose").value_or(false);
        s.motions[0].mask_root_position =
            yaml.Read<bool>(base + ".reference_mask_root_position").value_or(false);
        const auto catalog = yaml.Read<std::string>(base + ".catalog").value();
        const auto names = yaml.Read<std::vector<std::string>>(catalog + ".action_names").value();
        for (const auto &name : names) {
            const auto action = catalog + ".actions." + name;
            s.motions.push_back({name, path(action + ".model_path"), path(action + ".file")});
            s.motions.back().root_pose = yaml.Read<bool>(action + ".root_pose").value_or(false);
            s.motions.back().start_time = yaml.Read<double>(action + ".start_time").value_or(0.0);
            s.motions.back().end_time = yaml.Read<double>(action + ".end_time").value_or(-1.0);
            s.motions.back().mask_root_position =
                yaml.Read<bool>(action + ".mask_root_position").value_or(false);
            if (yaml.Read<std::string>(action + ".standby_file"))
                s.motions.back().standby_file = path(action + ".standby_file");
        }
    }
    return s;
}

std::unique_ptr<PolicyAdapter> CreateHistoryActorPolicyAdapter(const Config &config,
    const rl_policy::PolicyExecutorConfig &policy_config) {
    return std::make_unique<HistoryActorPolicyAdapter>(config, policy_config);
}
}  // namespace policy_adapter
}  // namespace behavior_manager
