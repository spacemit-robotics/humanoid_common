/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_driver_runtime.cpp
 * @brief Driver runtime backend selection example and offline test
 */

#include <cassert>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "driver_backend.h"
#include "robot_base.h"
#ifdef HAS_WHOLE_BODY_BACKEND
#include "backends/whole_body_adapter.h"
#endif

namespace {

class StubBackend final : public driver_runtime::DriverBackend {
public:
    int Run(const driver_runtime::ExchangeCallback &,
        const driver_runtime::ContinueCallback &) override {
        return 0;
    }
};

bool ExpectedCompiled(driver_runtime::BackendKind kind) {
    switch (kind) {
        case driver_runtime::BackendKind::MUJOCO:
#ifdef HAS_MUJOCO_BACKEND
            return true;
#else
            return false;
#endif
        case driver_runtime::BackendKind::WHOLE_BODY:
#ifdef HAS_WHOLE_BODY_BACKEND
            return true;
#else
            return false;
#endif
    }
    return false;
}

void VerifyFactory(driver_runtime::BackendKind kind) {
    const bool expected = ExpectedCompiled(kind);
    assert(driver_runtime::BackendIsCompiled(kind) == expected);
    bool created = false;
    try {
        created = driver_runtime::CreateBackend(kind, "unused.yaml") != nullptr;
    } catch (const std::runtime_error &) {
        created = false;
    }
    assert(created == expected);
}

#ifdef HAS_WHOLE_BODY_BACKEND
bool Near(double left, double right) { return std::abs(left - right) < 1.0e-9; }

void VerifyWholeBodyStateConversion() {
    whole_body_state source{};
    source.num_dof = 2;
    source.timestamp_s = 12.5;
    source.position[0] = 0.1;
    source.position[1] = -0.2;
    source.velocity[0] = 0.3;
    source.velocity[1] = -0.4;
    source.torque[0] = 1.5;
    source.torque[1] = -1.6;
    source.temperature[0] = 31.0;
    source.temperature[1] = 32.0;
    source.motor_error[0] = 0;
    source.motor_error[1] = 7;
    const std::array<double, 3> expected_rpy = {0.2, -0.1, 0.3};
    std::array<double, 4> quaternion{};
    robot_base::RpyToQuat(expected_rpy, quaternion);
    for (size_t i = 0; i < quaternion.size(); ++i)
        source.base_quat[i] = quaternion[i];
    source.gyro[0] = 0.4;
    source.gyro[1] = -0.5;
    source.gyro[2] = 0.6;
    source.acceleration[0] = 1.0;
    source.acceleration[1] = 2.0;
    source.acceleration[2] = 9.7;

    robot_base::RobotData destination;
    assert(driver_runtime::ConvertWholeBodyState(source, &destination));
    assert(destination.IsValid());
    assert(destination.num_dof == 2);
    assert(Near(destination.time, source.timestamp_s));
    for (size_t i = 0; i < source.num_dof; ++i) {
        assert(Near(destination.joint_pos[i], source.position[i]));
        assert(Near(destination.joint_vel[i], source.velocity[i]));
        assert(Near(destination.joint_torque[i], source.torque[i]));
        assert(Near(destination.joint_temperature[i], source.temperature[i]));
        assert(destination.joint_error[i] == source.motor_error[i]);
    }
    for (size_t i = 0; i < expected_rpy.size(); ++i) {
        assert(Near(destination.base_quat[i + 1], quaternion[i + 1]));
        assert(Near(destination.rpy[i], expected_rpy[i]));
        assert(Near(destination.gyro[i], source.gyro[i]));
        assert(Near(destination.base_vel[i + 3], source.gyro[i]));
        assert(Near(destination.acceleration[i], source.acceleration[i]));
    }
    assert(Near(destination.base_quat[0], quaternion[0]));

    source.num_dof = 0;
    assert(!driver_runtime::ConvertWholeBodyState(source, &destination));
    source.num_dof = WHOLE_BODY_MAX_DOF + 1;
    assert(!driver_runtime::ConvertWholeBodyState(source, &destination));
    assert(!driver_runtime::ConvertWholeBodyState(source, nullptr));
}

void VerifyWholeBodyCommandConversion() {
    const std::array<robot_base::ControlMode, 6> source_modes = {
        robot_base::ControlMode::POWER_OFF,
        robot_base::ControlMode::DAMP,
        robot_base::ControlMode::HOME,
        robot_base::ControlMode::ZERO,
        robot_base::ControlMode::RL,
        robot_base::ControlMode::SAFETY,
    };
    const std::array<whole_body_mode, 6> expected_modes = {
        WHOLE_BODY_MODE_POWER_OFF,
        WHOLE_BODY_MODE_DAMP,
        WHOLE_BODY_MODE_HOME,
        WHOLE_BODY_MODE_ZERO,
        WHOLE_BODY_MODE_RL,
        WHOLE_BODY_MODE_SAFETY,
    };
    const std::array<robot_base::ActuationMode, 4> source_actuation_modes = {
        robot_base::ActuationMode::HYBRID,
        robot_base::ActuationMode::POSITION,
        robot_base::ActuationMode::VELOCITY,
        robot_base::ActuationMode::TORQUE,
    };
    const std::array<whole_body_actuation_mode, 4> expected_actuation_modes = {
        WHOLE_BODY_ACTUATION_HYBRID,
        WHOLE_BODY_ACTUATION_POSITION,
        WHOLE_BODY_ACTUATION_VELOCITY,
        WHOLE_BODY_ACTUATION_TORQUE,
    };

    robot_base::ControlCmd source;
    source.enable = true;
    source.target_pos = {0.1, -0.2};
    source.target_vel = {0.3, -0.4};
    source.target_torque = {1.5, -1.6};
    source.kp = {20.0, 21.0};
    source.kd = {2.0, 2.1};
    whole_body_joint_command destination{};
    for (size_t mode_index = 0; mode_index < source_modes.size(); ++mode_index) {
        source.mode = source_modes[mode_index];
        for (size_t actuation_index = 0;
            actuation_index < source_actuation_modes.size(); ++actuation_index) {
            source.actuation_mode = source_actuation_modes[actuation_index];
            assert(driver_runtime::ConvertControlCommand(source, 2, &destination));
            assert(destination.mode == expected_modes[mode_index]);
            assert(destination.actuation_mode == expected_actuation_modes[actuation_index]);
        }
    }
    assert(destination.num_dof == 2);
    assert(destination.enable);
    for (size_t i = 0; i < destination.num_dof; ++i) {
        assert(Near(destination.position[i], source.target_pos[i]));
        assert(Near(destination.velocity[i], source.target_vel[i]));
        assert(Near(destination.torque[i], source.target_torque[i]));
        assert(Near(destination.kp[i], source.kp[i]));
        assert(Near(destination.kd[i], source.kd[i]));
    }

    source.target_torque.clear();
    assert(driver_runtime::ConvertControlCommand(source, 2, &destination));
    assert(Near(destination.torque[0], 0.0));
    assert(Near(destination.torque[1], 0.0));
    source.actuation_mode = static_cast<robot_base::ActuationMode>(-1);
    assert(!driver_runtime::ConvertControlCommand(source, 2, &destination));
    source.actuation_mode = robot_base::ActuationMode::HYBRID;
    source.target_pos.pop_back();
    assert(!driver_runtime::ConvertControlCommand(source, 2, &destination));
    assert(!driver_runtime::ConvertControlCommand(source, 0, &destination));
    assert(!driver_runtime::ConvertControlCommand(source, WHOLE_BODY_MAX_DOF + 1, &destination));
    assert(!driver_runtime::ConvertControlCommand(source, 2, nullptr));
}
#endif

}  // namespace

namespace driver_runtime {

#ifdef HAS_MUJOCO_BACKEND
std::unique_ptr<DriverBackend> CreateMujocoBackend(const std::string &) {
    return std::make_unique<StubBackend>();
}
#endif

#ifdef HAS_WHOLE_BODY_BACKEND
std::unique_ptr<DriverBackend> CreateWholeBodyBackend(const std::string &) {
    return std::make_unique<StubBackend>();
}
#endif

}  // namespace driver_runtime

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "用法: " << argv[0] << " <yaml配置文件路径>\n";
        return 1;
    }

    try {
        const auto yaml_file = robot_base::YamlFile::Load(argv[1]);
        assert(driver_runtime::ParseBackendKind(yaml_file) ==
            driver_runtime::BackendKind::MUJOCO);
        assert(driver_runtime::ParseBackendKind(std::nullopt) ==
            driver_runtime::BackendKind::MUJOCO);
        assert(driver_runtime::ParseBackendKind(std::optional<std::string>("mujoco")) ==
            driver_runtime::BackendKind::MUJOCO);
        assert(driver_runtime::ParseBackendKind(std::optional<std::string>("whole_body")) ==
            driver_runtime::BackendKind::WHOLE_BODY);

        bool rejected = false;
        try {
            driver_runtime::ParseBackendKind(std::optional<std::string>("invalid"));
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        assert(rejected);

        VerifyFactory(driver_runtime::BackendKind::MUJOCO);
        VerifyFactory(driver_runtime::BackendKind::WHOLE_BODY);
#ifdef HAS_WHOLE_BODY_BACKEND
        VerifyWholeBodyStateConversion();
        VerifyWholeBodyCommandConversion();
#endif
    } catch (const std::exception &error) {
        std::cerr << "driver_runtime 测试失败: " << error.what() << '\n';
        return 1;
    }

    std::cout << "driver_runtime 测试完成\n";
    return 0;
}
