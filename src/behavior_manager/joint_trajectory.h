/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file joint_trajectory.h
 * @brief Joint-space trajectory playback shared by behavior states and policy adapters
 */

#ifndef JOINT_TRAJECTORY_H
#define JOINT_TRAJECTORY_H

#include <memory>
#include <string>
#include <vector>

#include "robot_base.h"

namespace behavior_manager {
namespace joint_trajectory {

struct ActionConfig {
    std::string name;
    std::string file;
    std::vector<int> joint_indices;
    double motion_fps = 50.0;
    double playback_speed = 1.0;
    bool hold_last_frame = false;
};

struct Config {
    double blend_in_duration = 0.25;
    double blend_out_duration = 0.25;
    std::vector<ActionConfig> actions;

    bool Enabled() const { return !actions.empty(); }
};

Config LoadConfig(const robot_base::YamlFile &yaml, const std::string &base,
    const std::string &robot_dir);

class Player {
public:
    Player(const Config &config, int num_dof);
    ~Player();
    void Reset(const std::vector<double> &position);
    void Request(const robot_base::InteractionRequest &request, double elapsed_s);
    // Unselected joints retain their input target; blend-out returns to that target.
    void Apply(double elapsed_s, std::vector<double> &position);
    robot_base::InteractionStatus Status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace joint_trajectory
}  // namespace behavior_manager

#endif  // JOINT_TRAJECTORY_H
