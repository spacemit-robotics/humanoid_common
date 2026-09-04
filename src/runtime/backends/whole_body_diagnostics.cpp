/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_diagnostics.cpp
 * @brief Whole-body live monitor and telemetry serialization
 */

#include "backends/whole_body_diagnostics.h"

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>

#include "robot_base.h"
#include "runtime_logger.h"

namespace driver_runtime {
namespace {

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

const char *HealthName(whole_body_health_state state) {
    switch (state) {
        case WHOLE_BODY_HEALTH_CREATED:
            return "CREATED";
        case WHOLE_BODY_HEALTH_READY:
            return "READY";
        case WHOLE_BODY_HEALTH_READ_ONLY:
            return "READ_ONLY";
        case WHOLE_BODY_HEALTH_WATCHDOG:
            return "WATCHDOG";
        case WHOLE_BODY_HEALTH_ERROR:
            return "ERROR";
    }
    return "UNKNOWN";
}

const char *FeedbackName(const whole_body_motor_diagnostic &motor) {
    if (!motor.feedback_received) return "WAIT";
    return motor.feedback_fresh ? "OK" : "STALE";
}

double WallTimeSeconds() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::array<double, 3> ImuRpy(const whole_body_imu_diagnostic &imu) {
    std::array<double, 4> quaternion = {
        imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]};
    std::array<double, 3> rpy{};
    robot_base::QuatToRpy(quaternion, rpy);
    return rpy;
}

std::string MotorCell(const whole_body_motor_diagnostic &motor) {
    std::string label = motor.joint_names;
    if (label.find('+') != std::string::npos) label = std::string(motor.name) + "*";
    if (label.size() > 18) label.resize(18);

    std::ostringstream output;
    output << std::left << std::setw(18) << label
        << " " << std::setw(4) << motor.bus
        << ":" << std::hex << std::setw(2) << std::setfill('0') << motor.command_id
        << std::dec << std::setfill(' ')
        << " " << std::setw(5) << FeedbackName(motor)
        << std::right << std::fixed << std::setprecision(3)
        << " raw=" << std::setw(7) << motor.raw_position
        << " map=" << std::setw(7) << motor.calibrated_position
        << " e=" << std::hex << motor.error << std::dec
        << " age=" << std::setprecision(0) << motor.feedback_age_s * 1000.0;
    std::string cell = output.str();
    if (cell.size() < 64) cell.append(64 - cell.size(), ' ');
    return cell;
}

std::set<std::string> CoupledJointNames(const whole_body_diagnostics &diagnostics) {
    std::set<std::string> names;
    for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
        std::string labels = diagnostics.motors[i].joint_names;
        if (labels.find('+') == std::string::npos) continue;
        size_t begin = 0;
        for (;;) {
            const size_t end = labels.find('+', begin);
            names.insert(labels.substr(begin, end - begin));
            if (end == std::string::npos) break;
            begin = end + 1;
        }
    }
    return names;
}

}  // namespace

void RenderWholeBodyDiagnostics(
    const whole_body_diagnostics &diagnostics, double cycle_s) {
    const auto rpy = ImuRpy(diagnostics.imu);
    std::ostringstream output;
    output << "\033[2J\033[H"
        << "[driver] whole_body " << std::fixed << std::setprecision(1)
        << (cycle_s > 0.0 ? 1.0 / cycle_s : 0.0) << " Hz"
        << "  health=" << HealthName(diagnostics.health.state)
        << "  reads=" << diagnostics.health.read_cycles
        << "  writes=" << diagnostics.health.write_cycles
        << "  watchdog=" << diagnostics.health.watchdog_events << "\n";
    output << "IMU " << (diagnostics.imu.feedback_fresh ? "OK" : "STALE")
        << " age=" << std::setprecision(1)
        << diagnostics.imu.feedback_age_s * 1000.0 << " ms"
        << "  RPY[deg]=(" << std::setprecision(2)
        << rpy[0] * kRadiansToDegrees << ", "
        << rpy[1] * kRadiansToDegrees << ", "
        << rpy[2] * kRadiansToDegrees << ")"
        << "  gyro=(" << diagnostics.imu.gyro[0] << ", "
        << diagnostics.imu.gyro[1] << ", "
        << diagnostics.imu.gyro[2] << ")\n\n";
    output << "joint/motor        bus:id state  raw(rad) mapped(rad) error age(ms)\n";
    const uint32_t rows = (diagnostics.motor_count + 1) / 2;
    for (uint32_t row = 0; row < rows; ++row) {
        output << MotorCell(diagnostics.motors[row]);
        const uint32_t right = row + rows;
        if (right < diagnostics.motor_count)
            output << " | " << MotorCell(diagnostics.motors[right]);
        output << "\n";
    }

    const auto coupled_names = CoupledJointNames(diagnostics);
    if (!coupled_names.empty()) {
        output << "\nCoupled virtual joints:\n";
        for (uint32_t i = 0; i < diagnostics.joint_count; ++i) {
            const auto &joint = diagnostics.joints[i];
            if (coupled_names.count(joint.name) == 0) continue;
            output << "  " << std::left << std::setw(28) << joint.name
                << std::right << std::fixed << std::setprecision(4)
                << " pos=" << std::setw(9) << joint.position
                << " vel=" << std::setw(9) << joint.velocity
                << " torque=" << std::setw(9) << joint.torque << "\n";
        }
    }
    std::cout << output.str() << std::flush;
}

void RecordWholeBodyDiagnostics(const whole_body_diagnostics &diagnostics,
    const whole_body_motor_command_diagnostics &command_diagnostics) {
    const double wall_time = WallTimeSeconds();

    std::ostringstream motors;
    motors << std::fixed << std::setprecision(9);
    for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
        const auto &motor = diagnostics.motors[i];
        const bool has_command = i < command_diagnostics.motor_count;
        const auto *command = has_command ? &command_diagnostics.motors[i] : nullptr;
        motors << wall_time << "," << diagnostics.timestamp_s << ","
            << motor.name << "," << motor.joint_names << ","
            << motor.driver << "," << motor.model << "," << motor.bus << ","
            << motor.device << "," << motor.command_id << "," << motor.feedback_id << ","
            << motor.feedback_received << "," << motor.feedback_fresh << ","
            << motor.feedback_age_s << "," << motor.raw_position << ","
            << motor.calibrated_position << "," << motor.raw_velocity << ","
            << motor.calibrated_velocity << "," << motor.raw_torque << ","
            << motor.calibrated_torque << "," << motor.temperature << ","
            << motor.error << "," << (command && command->valid) << ","
            << (command ? command->age_s : 0.0) << ","
            << (command ? command->mode : 0) << ","
            << (command ? command->position : 0.0) << ","
            << (command ? command->velocity : 0.0) << ","
            << (command ? command->torque : 0.0) << ","
            << (command ? command->kp : 0.0) << ","
            << (command ? command->kd : 0.0);
        if (i + 1 < diagnostics.motor_count) motors << "\n";
    }
    runtime_logging::RecordCsv(
        "driver_motor",
        "wall_time_s,device_time_s,motor,joints,driver,model,bus,device,"
        "command_id,feedback_id,received,fresh,age_s,raw_pos,calibrated_pos,"
        "raw_vel,calibrated_vel,raw_torque,calibrated_torque,temp_c,error,"
        "command_valid,command_age_s,command_mode,command_pos,command_vel,"
        "command_torque,command_kp,command_kd",
        motors.str());

    std::ostringstream joints;
    joints << std::fixed << std::setprecision(9);
    for (uint32_t i = 0; i < diagnostics.joint_count; ++i) {
        const auto &joint = diagnostics.joints[i];
        joints << wall_time << "," << diagnostics.timestamp_s << "," << joint.name << ","
            << joint.feedback_valid << "," << joint.position << "," << joint.velocity
            << "," << joint.torque << "," << joint.temperature << ","
            << joint.motor_error;
        if (i + 1 < diagnostics.joint_count) joints << "\n";
    }
    runtime_logging::RecordCsv(
        "driver_joint",
        "wall_time_s,device_time_s,joint,valid,position,velocity,torque,temp_c,error",
        joints.str());

    const auto rpy = ImuRpy(diagnostics.imu);
    std::ostringstream imu;
    imu << std::fixed << std::setprecision(9)
        << wall_time << "," << diagnostics.timestamp_s << ","
        << diagnostics.imu.feedback_received << "," << diagnostics.imu.feedback_fresh << ","
        << diagnostics.imu.feedback_age_s << ","
        << diagnostics.imu.quaternion[0] << "," << diagnostics.imu.quaternion[1] << ","
        << diagnostics.imu.quaternion[2] << "," << diagnostics.imu.quaternion[3] << ","
        << rpy[0] << "," << rpy[1] << "," << rpy[2] << ","
        << diagnostics.imu.gyro[0] << "," << diagnostics.imu.gyro[1] << ","
        << diagnostics.imu.gyro[2] << "," << diagnostics.imu.acceleration[0] << ","
        << diagnostics.imu.acceleration[1] << "," << diagnostics.imu.acceleration[2];
    runtime_logging::RecordCsv(
        "driver_imu",
        "wall_time_s,device_time_s,received,fresh,age_s,qw,qx,qy,qz,"
        "roll,pitch,yaw,gx,gy,gz,ax,ay,az",
        imu.str());
}

}  // namespace driver_runtime
