/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file joint_trajectory.cpp
 * @brief Joint-space trajectory loading, blending and playback
 */

#include "joint_trajectory.h"

#include <cnpy.h>

#include <algorithm>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace behavior_manager {
namespace joint_trajectory {
namespace {

double SmoothStep(double value) {
    const double x = std::clamp(value, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

void CheckTime(double value) {
    if (!std::isfinite(value) || value < 0.0)
        throw std::runtime_error("joint_trajectory: invalid elapsed time");
}

}  // namespace

Config LoadConfig(const robot_base::YamlFile &yaml, const std::string &base,
    const std::string &robot_dir) {
    const auto file = yaml.Read<std::string>(base + ".catalog_file");
    const auto catalog = file ? robot_base::YamlFile::Load(yaml.ToAbsPath(*file)) : yaml;
    const std::string key = yaml.Read<std::string>(base + ".catalog").value_or(base);
    const auto source_names = catalog.Read<std::vector<std::string>>("robot_base.joint_names")
        .value_or(std::vector<std::string>{});
    const auto target_names = yaml.Read<std::vector<std::string>>("robot_base.joint_names")
        .value_or(std::vector<std::string>{});
    const auto source_dir = file
        ? catalog.ToAbsPath(catalog.Read<std::string>("robot_base.robot_dir").value_or("."))
        : robot_dir;
    Config result;
    result.blend_in_duration = yaml.Read<double>(base + ".blend_in_duration")
        .value_or(catalog.Read<double>(key + ".blend_in_duration").value_or(0.25));
    result.blend_out_duration = yaml.Read<double>(base + ".blend_out_duration")
        .value_or(catalog.Read<double>(key + ".blend_out_duration").value_or(0.25));
    const double fps = catalog.Read<double>(key + ".motion_fps").value_or(50.0);
    const double speed = catalog.Read<double>(key + ".playback_speed").value_or(1.0);
    const bool hold = catalog.Read<bool>(key + ".hold_last_frame").value_or(false);
    for (const auto &name : catalog.Read<std::vector<std::string>>(key + ".action_names")
            .value_or(std::vector<std::string>{})) {
        const std::string action_key = key + ".actions." + name;
        ActionConfig action;
        action.name = name;
        const auto path = catalog.Read<std::string>(action_key + ".file");
        if (!path || path->empty()) throw std::runtime_error("missing " + action_key + ".file");
        action.file = (std::filesystem::path(source_dir) / *path).lexically_normal().string();
        action.joint_indices = catalog.Read<std::vector<int>>(action_key + ".joint_indices")
            .value_or(std::vector<int>{});
        if (file) {
            // Catalog indices belong to its source robot, not a reduced hardware profile.
            for (auto &index : action.joint_indices) {
                if (index < 0 || index >= static_cast<int>(source_names.size()))
                    throw std::runtime_error(action_key + ": invalid source joint index");
                const auto target = std::find(target_names.begin(), target_names.end(), source_names[index]);
                if (target == target_names.end())
                    throw std::runtime_error(action_key + ": joint unavailable: " + source_names[index]);
                index = static_cast<int>(std::distance(target_names.begin(), target));
            }
        }
        action.motion_fps = catalog.Read<double>(action_key + ".motion_fps").value_or(fps);
        action.playback_speed = catalog.Read<double>(action_key + ".playback_speed").value_or(speed);
        action.hold_last_frame = catalog.Read<bool>(action_key + ".hold_last_frame").value_or(hold);
        result.actions.push_back(std::move(action));
    }
    return result;
}

struct Player::Impl {
    using Phase = robot_base::InteractionStatus::Phase;
    struct Clip {
        ActionConfig config;
        size_t frames = 0;
        std::vector<double> positions;

        double Duration() const {
            return (frames - 1) / (config.motion_fps * config.playback_speed);
        }
        double Position(size_t column, double elapsed) const {
            const double sample = std::clamp(elapsed * config.motion_fps * config.playback_speed,
                0.0, static_cast<double>(frames - 1));
            const size_t first = static_cast<size_t>(std::floor(sample));
            const size_t second = std::min(first + 1, frames - 1);
            const size_t width = config.joint_indices.size();
            const double alpha = sample - first;
            return (1.0 - alpha) * positions[first * width + column] +
                alpha * positions[second * width + column];
        }
    };
    Config config;
    int num_dof;
    std::vector<Clip> clips;
    std::unordered_map<std::string, size_t> names;
    const Clip *active = nullptr;
    std::vector<double> previous;
    std::vector<double> source;
    double phase_start = 0.0;
    double playback_start = 0.0;
    uint64_t sequence = 0;
    robot_base::InteractionStatus status;

    Impl(const Config &input, int count) : config(input), num_dof(count) {
        if (count <= 0 || !config.Enabled() || !std::isfinite(config.blend_in_duration) ||
            config.blend_in_duration < 0.0 || !std::isfinite(config.blend_out_duration) ||
            config.blend_out_duration < 0.0) throw std::runtime_error("joint_trajectory: invalid config");
        for (const auto &action : config.actions) {
            if (action.name.empty() || action.file.empty() || action.joint_indices.empty() ||
                names.count(action.name) || !std::isfinite(action.motion_fps) || action.motion_fps <= 0.0 ||
                !std::isfinite(action.playback_speed) || action.playback_speed <= 0.0)
                throw std::runtime_error("joint_trajectory: invalid action " + action.name);
            std::vector<bool> used(count, false);
            for (int index : action.joint_indices) {
                if (index < 0 || index >= count || used[index])
                    throw std::runtime_error("joint_trajectory: invalid joint binding " + action.name);
                used[index] = true;
            }
            const auto data = cnpy::npz_load(action.file, "joint_pos");
            if (data.shape.size() != 2 || data.shape[0] == 0 || data.shape[1] != action.joint_indices.size() ||
                (data.word_size != sizeof(float) && data.word_size != sizeof(double)))
                throw std::runtime_error("joint_trajectory: expected float32/64 [T,K]: " + action.file);
            Clip clip{action, data.shape[0], {}};
            clip.positions.reserve(data.num_vals);
            for (size_t i = 0; i < data.num_vals; ++i) {
                const size_t j = data.fortran_order ? (i % data.shape[1]) * data.shape[0] + i / data.shape[1] : i;
                const double value = data.word_size == sizeof(float) ? data.data<float>()[j] : data.data<double>()[j];
                if (!std::isfinite(value)) throw std::runtime_error("joint_trajectory: non-finite position");
                clip.positions.push_back(value);
            }
            names.emplace(action.name, clips.size());
            clips.push_back(std::move(clip));
        }
        previous.assign(count, 0.0);
    }

    void CaptureSource() {
        source.clear();
        for (int index : active->config.joint_indices) source.push_back(previous[index]);
    }
};

Player::Player(const Config &config, int num_dof) : impl_(std::make_unique<Impl>(config, num_dof)) {}
Player::~Player() = default;

void Player::Reset(const std::vector<double> &position) {
    if (position.size() != static_cast<size_t>(impl_->num_dof))
        throw std::runtime_error("joint_trajectory: reset dimension mismatch");
    impl_->previous = position;
    impl_->active = nullptr;
    impl_->source.clear();
    impl_->sequence = 0;
    impl_->status = {};
}

void Player::Request(const robot_base::InteractionRequest &request, double elapsed_s) {
    using Operation = robot_base::InteractionRequest::Operation;
    using Phase = robot_base::InteractionStatus::Phase;
    auto &p = *impl_;
    CheckTime(elapsed_s);
    if (request.operation == Operation::NONE || request.sequence == 0 || request.sequence <= p.sequence) return;
    p.sequence = request.sequence;
    p.status.sequence = request.sequence;
    if (request.operation == Operation::CANCEL) {
        p.status.request_accepted = true;
        if (p.active) {
            p.CaptureSource();
            p.phase_start = elapsed_s;
            p.status.phase = Phase::BLEND_OUT;
            p.status.progress = 1.0F;
        } else {
            p.status.phase = Phase::IDLE;
            p.status.action.clear();
            p.status.progress = 0.0F;
        }
        return;
    }
    if (p.active) {
        p.status.request_accepted = false;
        return;
    }
    const auto found = p.names.find(request.action);
    if (found == p.names.end() || request.operation != Operation::START) {
        p.status.request_accepted = false;
        p.status.phase = Phase::REJECTED;
        p.status.action = request.action;
        p.status.progress = 0.0F;
        return;
    }
    p.active = &p.clips[found->second];
    p.CaptureSource();
    p.phase_start = elapsed_s;
    p.playback_start = elapsed_s + p.config.blend_in_duration;
    p.status.request_accepted = true;
    p.status.phase = Phase::BLEND_IN;
    p.status.action = request.action;
    p.status.progress = 0.0F;
}

void Player::Apply(double elapsed_s, std::vector<double> &position) {
    using Phase = robot_base::InteractionStatus::Phase;
    auto &p = *impl_;
    CheckTime(elapsed_s);
    if (position.size() != static_cast<size_t>(p.num_dof))
        throw std::runtime_error("joint_trajectory: target dimension mismatch");
    if (p.active) {
        double blend = 1.0;
        if (p.status.phase == Phase::BLEND_IN) {
            blend = p.config.blend_in_duration == 0.0 ? 1.0 :
                SmoothStep((elapsed_s - p.phase_start) / p.config.blend_in_duration);
            if (blend >= 1.0) p.status.phase = Phase::PLAYING;
        }
        double playback = 0.0;
        if (p.status.phase == Phase::PLAYING || p.status.phase == Phase::HOLDING) {
            playback = std::max(0.0, elapsed_s - p.playback_start);
            const double duration = p.active->Duration();
            p.status.progress = duration == 0.0 ? 1.0F : static_cast<float>(std::clamp(playback / duration, 0.0, 1.0));
            if (playback >= duration) {
                playback = duration;
                if (p.active->config.hold_last_frame) {
                    p.status.phase = Phase::HOLDING;
                } else {
                    p.status.phase = Phase::BLEND_OUT;
                    p.phase_start = elapsed_s;
                    for (size_t i = 0; i < p.source.size(); ++i) p.source[i] = p.active->Position(i, duration);
                }
            }
        }
        if (p.status.phase == Phase::BLEND_OUT) {
            const double alpha = p.config.blend_out_duration == 0.0 ? 1.0 :
                SmoothStep((elapsed_s - p.phase_start) / p.config.blend_out_duration);
            for (size_t i = 0; i < p.source.size(); ++i) {
                const int joint = p.active->config.joint_indices[i];
                position[joint] = (1.0 - alpha) * p.source[i] + alpha * position[joint];
            }
            if (alpha >= 1.0) {
                p.active = nullptr;
                p.status.phase = Phase::FINISHED;
                p.status.progress = 1.0F;
            }
        } else {
            for (size_t i = 0; i < p.source.size(); ++i) {
                const int joint = p.active->config.joint_indices[i];
                const double target = p.active->Position(i, playback);
                position[joint] = p.status.phase == Phase::BLEND_IN
                    ? (1.0 - blend) * p.source[i] + blend * target : target;
            }
        }
    }
    p.previous = position;
}

robot_base::InteractionStatus Player::Status() const { return impl_->status; }

}  // namespace joint_trajectory
}  // namespace behavior_manager
