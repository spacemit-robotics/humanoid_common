/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_backend.cpp
 * @brief MuJoCo adapter for the generic driver runtime
 */

#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "driver_backend.h"
#include "mujoco_sim.h"
#include "robot_base.h"

namespace driver_runtime {
namespace {

class MujocoBackend final : public DriverBackend {
public:
    explicit MujocoBackend(const std::string &yaml_path) {
        const auto yaml_file = robot_base::YamlFile::Load(yaml_path);
        const auto robot_name = yaml_file.Read<std::string>("robot_base.name").value();
        const int num_dof = yaml_file.Read<int>("robot_base.num_dof").value();
        const std::string robot_dir =
            yaml_file.ToAbsPath(yaml_file.Read<std::string>("robot_base.robot_dir").value());
        const std::string scene =
            yaml_file.Read<std::string>("simulation.mujoco.scene_xml").value_or("scene.xml");
        const auto default_joint_pos =
            yaml_file.Read<std::vector<double>>("robot_base.default_joint_pos").value();
        const auto kp =
            yaml_file.Read<std::vector<double>>("robot_base.kp").value_or(std::vector<double>{});
        const auto kd =
            yaml_file.Read<std::vector<double>>("robot_base.kd").value_or(std::vector<double>{});
        simulator_ = std::make_unique<mujoco_sim::Simulator>(yaml_path, robot_name, num_dof,
            robot_dir + "/resources/xml/" + scene, default_joint_pos, kp, kd, true);
    }

    int Run(const ExchangeCallback &exchange, const ContinueCallback &should_continue) override {
        auto last_mode = robot_base::ControlMode::POWER_OFF;
        simulator_->Run(
            [&](const mujoco_sim::SimState &sim_state) -> std::optional<mujoco_sim::SimControl> {
                robot_base::RobotData state;
                state.num_dof = sim_state.num_dof;
                state.joint_pos = sim_state.joint_pos;
                state.joint_vel = sim_state.joint_vel;
                state.base_pos = sim_state.base_pos;
                state.base_quat = sim_state.base_quat;
                state.base_vel = sim_state.base_vel;
                state.gyro = sim_state.gyro;
                state.rpy = sim_state.rpy;
                state.time = sim_state.time;
                const auto command = exchange(state);
                if (!command) return std::nullopt;

                if (command->actuation_mode != robot_base::ActuationMode::HYBRID &&
                    command->actuation_mode != robot_base::ActuationMode::POSITION) {
                    throw std::runtime_error(
                        "mujoco backend supports only HYBRID and POSITION actuation modes");
                }
                if (command->actuation_mode == robot_base::ActuationMode::HYBRID) {
                    for (double torque : command->target_torque) {
                        if (torque != 0.0) {
                            throw std::runtime_error(
                                "mujoco backend does not support feed-forward torque");
                        }
                    }
                }

                if (command->mode != last_mode) {
                    if (command->mode == robot_base::ControlMode::RL)
                        simulator_->SetAssistEnabled(false);
                    else if (command->mode == robot_base::ControlMode::POWER_OFF)
                        simulator_->SetAssistEnabled(true);
                    last_mode = command->mode;
                }

                mujoco_sim::SimControl control;
                control.enable = command->enable;
                control.target_pos = command->target_pos;
                control.target_vel = command->target_vel;
                control.kp = command->kp;
                control.kd = command->kd;
                return control;
            },
            should_continue);
        return 0;
    }

private:
    std::unique_ptr<mujoco_sim::Simulator> simulator_;
};

}  // namespace

std::unique_ptr<DriverBackend> CreateMujocoBackend(const std::string &yaml_path) {
    return std::make_unique<MujocoBackend>(yaml_path);
}

}  // namespace driver_runtime
