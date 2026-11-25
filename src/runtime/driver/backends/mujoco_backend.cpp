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
#ifdef HAS_HIKING_SIM_DEPTH
#include "mujoco_depth_publisher.h"
#endif
#include "mujoco_sim.h"

#include <algorithm>
#include "robot_base.h"
#include "runtime_logger.h"

namespace driver_runtime {
namespace {

class MujocoBackend final : public DriverBackend {
public:
    MujocoBackend(const std::string &yaml_path, const std::string &scene_override) {
        const auto yaml_file = robot_base::YamlFile::Load(yaml_path);
        const auto robot_name = yaml_file.Read<std::string>("robot_base.name").value();
        const int num_dof = yaml_file.Read<int>("robot_base.num_dof").value();
        const std::string robot_dir =
            yaml_file.ToAbsPath(yaml_file.Read<std::string>("robot_base.robot_dir").value());
        const std::string scene = scene_override.empty() ?
            yaml_file.Read<std::string>("simulation.mujoco.scene_xml").value_or("scene.xml") : scene_override;
        const std::string scene_path = robot_dir + "/resources/xml/" + scene;
        runtime_logging::RecordArtifact("simulation_model", "scene", scene_path);
        const auto default_joint_pos =
            yaml_file.Read<std::vector<double>>("robot_base.default_joint_pos").value();
        const auto kp =
            yaml_file.Read<std::vector<double>>("robot_base.kp").value_or(std::vector<double>{});
        const auto kd =
            yaml_file.Read<std::vector<double>>("robot_base.kd").value_or(std::vector<double>{});
        simulator_ = std::make_unique<mujoco_sim::Simulator>(yaml_path, robot_name, num_dof,
            scene_path, default_joint_pos, kp, kd, true);
        const auto sim_depth = yaml_file.Read<std::string>("simulation.mujoco.depth_endpoint");
        if (sim_depth || yaml_file.Read<bool>("perception.depth.publish_sim").value_or(false)) {
#ifdef HAS_HIKING_SIM_DEPTH
            perception::SimDepthConfig config;
            config.preview_endpoint = yaml_file.Read<std::string>("simulation.mujoco.preview_endpoint").value_or("");
            const std::string base = "simulation.mujoco.depth.";
            config.fps = yaml_file.Read<double>(base + "fps").value_or(config.fps);
            config.latency_s = yaml_file.Read<double>(base + "latency_s").value_or(config.latency_s);
            const std::string defects = base + "degradation.";
            config.enabled = yaml_file.Read<bool>(defects + "enabled").value_or(config.enabled);
            const int seed = yaml_file.Read<int>(defects + "seed").value_or(42);
            if (seed < 0) throw std::runtime_error("simulation depth seed must be nonnegative");
            config.seed = static_cast<uint32_t>(seed);
            config.random_hole_probability = yaml_file.Read<double>(defects + "random_hole_probability")
                .value_or(config.random_hole_probability);
            config.patch_count = yaml_file.Read<int>(defects + "patch_count").value_or(config.patch_count);
            config.patch_radius_px = yaml_file.Read<int>(defects + "patch_radius_px").value_or(config.patch_radius_px);
            config.edge_threshold_m = yaml_file.Read<double>(defects + "edge_threshold_m").value_or(config.edge_threshold_m);
            config.edge_hole_probability = yaml_file.Read<double>(defects + "edge_hole_probability")
                .value_or(config.edge_hole_probability);
            config.edge_width_px = yaml_file.Read<int>(defects + "edge_width_px").value_or(config.edge_width_px);
            config.hold_frames = yaml_file.Read<int>(defects + "hold_frames").value_or(config.hold_frames);
            config.noise_std_m = yaml_file.Read<double>(defects + "noise_std_m").value_or(config.noise_std_m);
            config.Validate();
            depth_ = std::make_unique<perception::MujocoDepthPublisher>(
                sim_depth.value_or(yaml_file.Read<std::string>("perception.depth.endpoint")
                    .value_or("tcp://127.0.0.1:5560")), config);
            simulator_->SetObserveCallback([this](const auto &state, const auto &frame) {
                depth_->Observe(state.time, frame.model, frame.data);
            });
#else
            runtime_logging::Log(runtime_logging::Level::kWarning,
                "Simulation depth output unavailable in this build; continuing without depth", false);
#endif
        }
    }

    int Run(const PublishStateCallback &publish_state,
            const ReceiveCommandCallback &receive_command,
            const ContinueCallback &should_continue) override {
        auto last_mode = robot_base::ControlMode::POWER_OFF;
        simulator_->Run(
            [&](const mujoco_sim::SimState &sim_state) -> std::optional<mujoco_sim::SimControl> {
                robot_base::RobotData state;
                state.num_dof = sim_state.num_dof;
                state.joint_pos = sim_state.joint_pos;
                state.joint_vel = sim_state.joint_vel;
                state.has_joint_feedback = true;
                state.base_pos = sim_state.base_pos;
                // SimState leaves its quaternion zero when the model has no floating base.
                state.has_base_imu = std::any_of(sim_state.base_quat.begin(),
                    sim_state.base_quat.end(), [](double v) { return v != 0.0; });
                if (state.has_base_imu) state.base_quat = sim_state.base_quat;
                state.base_vel = sim_state.base_vel;
                state.gyro = sim_state.gyro;
                state.rpy = sim_state.rpy;
                state.time = sim_state.time;
                state.environment = robot_base::RuntimeEnvironment::SIMULATION;
                if (!publish_state(state, {})) {
                    throw std::runtime_error("failed to publish MuJoCo state");
                }
                const auto command = receive_command();
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
                    else if (command->mode == robot_base::ControlMode::POWER_OFF ||
                        command->mode == robot_base::ControlMode::DAMP)
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
#ifdef HAS_HIKING_SIM_DEPTH
    std::unique_ptr<perception::MujocoDepthPublisher> depth_;
#endif
};

}  // namespace

std::unique_ptr<DriverBackend> CreateMujocoBackend(const std::string &yaml_path, const std::string &scene) {
    return std::make_unique<MujocoBackend>(yaml_path, scene);
}

}  // namespace driver_runtime
