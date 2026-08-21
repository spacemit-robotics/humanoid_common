/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_adapter.cpp
 * @brief Internal data conversion between humanoid common and whole_body
 */

#include "backends/whole_body_adapter.h"

#include <cstddef>
#include <utility>

namespace driver_runtime {
namespace {

whole_body_mode ToWholeBodyMode(robot_base::ControlMode mode) {
    switch (mode) {
        case robot_base::ControlMode::POWER_OFF:
            return WHOLE_BODY_MODE_POWER_OFF;
        case robot_base::ControlMode::DAMP:
            return WHOLE_BODY_MODE_DAMP;
        case robot_base::ControlMode::HOME:
            return WHOLE_BODY_MODE_HOME;
        case robot_base::ControlMode::ZERO:
            return WHOLE_BODY_MODE_ZERO;
        case robot_base::ControlMode::RL:
            return WHOLE_BODY_MODE_RL;
        case robot_base::ControlMode::SAFETY:
            return WHOLE_BODY_MODE_SAFETY;
    }
    return WHOLE_BODY_MODE_SAFETY;
}

whole_body_actuation_mode ToWholeBodyActuationMode(robot_base::ActuationMode mode) {
    switch (mode) {
        case robot_base::ActuationMode::HYBRID:
            return WHOLE_BODY_ACTUATION_HYBRID;
        case robot_base::ActuationMode::POSITION:
            return WHOLE_BODY_ACTUATION_POSITION;
        case robot_base::ActuationMode::VELOCITY:
            return WHOLE_BODY_ACTUATION_VELOCITY;
        case robot_base::ActuationMode::TORQUE:
            return WHOLE_BODY_ACTUATION_TORQUE;
    }
    return WHOLE_BODY_ACTUATION_HYBRID;
}

bool HasValidCommandShape(const robot_base::ControlCmd &command, uint32_t num_dof) {
    const size_t size = num_dof;
    const bool valid_positions = command.target_pos.size() == size;
    const bool valid_velocities = command.target_vel.size() == size;
    const bool valid_torques = command.target_torque.empty() ||
        command.target_torque.size() == size;
    const bool valid_gains = command.kp.size() == size && command.kd.size() == size;
    return num_dof > 0 && num_dof <= WHOLE_BODY_MAX_DOF &&
        robot_base::IsValidActuationMode(command.actuation_mode) && valid_positions &&
        valid_velocities && valid_torques && valid_gains;
}

}  // namespace

bool ConvertWholeBodyState(
    const whole_body_state &source, robot_base::RobotData *destination) {
    if (!destination || source.num_dof == 0 || source.num_dof > WHOLE_BODY_MAX_DOF)
        return false;

    auto result = robot_base::RobotData::Create(source.num_dof);
    for (size_t i = 0; i < source.num_dof; ++i) {
        result.joint_pos[i] = source.position[i];
        result.joint_vel[i] = source.velocity[i];
        result.joint_torque[i] = source.torque[i];
        result.joint_temperature[i] = source.temperature[i];
        result.joint_error[i] = source.motor_error[i];
    }
    for (size_t i = 0; i < result.base_quat.size(); ++i)
        result.base_quat[i] = source.base_quat[i];
    for (size_t i = 0; i < result.gyro.size(); ++i) {
        result.gyro[i] = source.gyro[i];
        result.acceleration[i] = source.acceleration[i];
        result.base_vel[i + 3] = source.gyro[i];
    }
    robot_base::QuatToRpy(result.base_quat, result.rpy);
    result.time = source.timestamp_s;
    *destination = std::move(result);
    return true;
}

bool ConvertControlCommand(const robot_base::ControlCmd &source, uint32_t num_dof,
    whole_body_joint_command *destination) {
    if (!destination || !HasValidCommandShape(source, num_dof)) return false;

    whole_body_joint_command result{};
    result.num_dof = num_dof;
    result.enable = source.enable;
    result.mode = ToWholeBodyMode(source.mode);
    result.actuation_mode = ToWholeBodyActuationMode(source.actuation_mode);
    for (size_t i = 0; i < num_dof; ++i) {
        result.position[i] = source.target_pos[i];
        result.velocity[i] = source.target_vel[i];
        result.torque[i] = source.target_torque.empty() ? 0.0 : source.target_torque[i];
        result.kp[i] = source.kp[i];
        result.kd[i] = source.kd[i];
    }
    *destination = result;
    return true;
}

}  // namespace driver_runtime
