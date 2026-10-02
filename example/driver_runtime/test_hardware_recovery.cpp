/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_hardware_recovery.cpp
 * @brief Run the real hardware backend against disconnected peripheral fixtures
 */

#include <unistd.h>

#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "backends/whole_body_diagnostics.h"
#include "driver_backend.h"

struct whole_body_dev {
    bool safety_latched = false;
};

namespace {

struct Scenario {
    int has_imu = 1;
    bool saw_motor_feedback_fault = false;
    bool saw_motor_device_fault = false;
    bool initial_feedback_wait = true;
    int destroys = 0;
    int init_error = WHOLE_BODY_OK;
    int init_calls = 0;
    int offline_reads = 0;
    int reads = 0;
    int enabled_writes = 0;
    int power_off_writes = 0;
    int safety_requests = 0;
    int fault_events = 0;
    int recoveries = 0;
    int fresh_reports = 0;
    int publications = 0;
    int command_polls = 0;
    bool offline = false;
    bool last_fault_active = false;
    bool needs_rearm = false;
    bool repeat_outages = false;
    bool reject_publication = false;
    bool needs_query_reply = false;
    double device_time = 0.0;
    robot_base::FaultStatus last_fault;
    std::chrono::steady_clock::time_point last_offline_read;
    std::chrono::steady_clock::time_point last_query;
};

Scenario scenario;

bool Publish(const robot_base::RobotData &state, const robot_base::FaultStatus &fault) {
    assert(state.IsValid());
    if (fault.active && fault.code == robot_base::FaultCode::FEEDBACK_TIMEOUT) {
        assert(fault.source == robot_base::FaultSource::MOTOR);
        scenario.saw_motor_feedback_fault = true;
    }
    if (fault.active && fault.code == robot_base::FaultCode::DEVICE_ERROR &&
        fault.source == robot_base::FaultSource::MOTOR) {
        scenario.saw_motor_device_fault = true;
    }
    ++scenario.publications;
    if (fault.active && !scenario.last_fault_active) {
        ++scenario.fault_events;
        scenario.needs_rearm = true;
        scenario.fresh_reports = 0;
    }
    if (!fault.active && scenario.last_fault_active) {
        assert(fault.latched);
        ++scenario.recoveries;
    }
    if (!fault.active && state.time > 0.0) ++scenario.fresh_reports;
    scenario.last_fault_active = fault.active;
    scenario.last_fault = fault;
    return !scenario.reject_publication;
}

std::optional<robot_base::ControlCmd> Receive() {
    ++scenario.command_polls;
    robot_base::ControlCmd command;
    command.target_pos.assign(2, 0.0);
    command.target_vel.assign(2, 0.0);
    command.target_torque.assign(2, 0.0);
    command.kp.assign(2, 0.0);
    command.kd.assign(2, 0.0);
    command.enable = true;
    command.mode = robot_base::ControlMode::RL;
    if (scenario.needs_rearm && scenario.fresh_reports >= 8) {
        if (scenario.fresh_reports < 12) {
            command.enable = false;
            command.mode = robot_base::ControlMode::POWER_OFF;
        } else {
            assert(scenario.power_off_writes > 0);
            assert(!scenario.last_fault.active);
            scenario.needs_rearm = false;
        }
    }
    return command;
}

}  // namespace

// C API fixtures: no CAN, IMU, SHM, or real motor commands are used.
extern "C" {
int whole_body_create(const char *, whole_body_dev **device) {
    *device = new whole_body_dev;
    return WHOLE_BODY_OK;
}

void whole_body_destroy(whole_body_dev *device) {
    if (device) ++scenario.destroys;
    delete device;
}

int whole_body_init(whole_body_dev *) {
    return ++scenario.init_calls == 1 ? scenario.init_error : WHOLE_BODY_OK;
}

int whole_body_get_cycle_s(const whole_body_dev *, double *cycle_s) {
    *cycle_s = 0.002;
    return WHOLE_BODY_OK;
}

int whole_body_has_imu(whole_body_dev *) { return scenario.has_imu; }

int whole_body_read(whole_body_dev *device, whole_body_state *state) {
    ++scenario.reads;
    scenario.offline = scenario.offline_reads > 0;
    if (scenario.offline) {
        const auto now = std::chrono::steady_clock::now();
        if (scenario.last_offline_read.time_since_epoch().count() != 0) {
            assert(now - scenario.last_offline_read >= std::chrono::milliseconds(90));
        }
        scenario.last_offline_read = now;
        scenario.needs_query_reply = true;
        --scenario.offline_reads;
        if (scenario.reads > 1) device->safety_latched = true;
        return scenario.reads == 1 && scenario.initial_feedback_wait
            ? WHOLE_BODY_ERR_STATE : WHOLE_BODY_ERR_TIMEOUT;
    }
    if (scenario.needs_query_reply) {
        const auto age = std::chrono::steady_clock::now() - scenario.last_query;
        if (age > std::chrono::milliseconds(40)) return WHOLE_BODY_ERR_TIMEOUT;
        scenario.needs_query_reply = false;
    }
    *state = {};
    state->num_dof = 2;
    state->base_quat[0] = 1.0;
    state->timestamp_s = scenario.device_time += 0.002;
    return WHOLE_BODY_OK;
}

int whole_body_write(whole_body_dev *device, const whole_body_joint_command *command) {
    assert(!scenario.offline);
    if (command->enable) {
        assert(!scenario.needs_rearm);
        assert(!device->safety_latched);
        ++scenario.enabled_writes;
        if (scenario.repeat_outages &&
            (scenario.enabled_writes == 2 || scenario.enabled_writes == 4)) {
            scenario.offline_reads = 3;
        }
    } else if (command->mode == WHOLE_BODY_MODE_POWER_OFF) {
        ++scenario.power_off_writes;
        device->safety_latched = false;
    }
    return WHOLE_BODY_OK;
}

int whole_body_tick(whole_body_dev *, double) {
    if (!scenario.offline) scenario.last_query = std::chrono::steady_clock::now();
    return scenario.offline ? WHOLE_BODY_ERR_DEVICE : WHOLE_BODY_OK;
}

int whole_body_set_mode(whole_body_dev *device, whole_body_mode mode) {
    assert(mode == WHOLE_BODY_MODE_SAFETY);
    device->safety_latched = true;
    ++scenario.safety_requests;
    return scenario.offline ? WHOLE_BODY_ERR_DEVICE : WHOLE_BODY_OK;
}

int whole_body_get_diagnostics_v2(const whole_body_dev *, whole_body_diagnostics_v2 *diag) {
    *diag = {};
    diag->motor_count = 1;
    diag->imu.feedback_received = scenario.has_imu == 1;
    diag->imu.feedback_fresh = scenario.has_imu == 1;
    return WHOLE_BODY_OK;
}

int whole_body_get_motor_command_diagnostics_v2(
    const whole_body_dev *, whole_body_motor_command_diagnostics_v2 *) {
    return WHOLE_BODY_OK;
}

const char *whole_body_last_error(const whole_body_dev *) {
    return scenario.reads == 1 && scenario.initial_feedback_wait
        ? "waiting for initial feedback" : "motor power unavailable";
}
}  // extern "C"

namespace driver_runtime {
std::unique_ptr<DriverBackend> CreateWholeBodyBackend(const std::string &yaml_path);
void RenderWholeBodyDiagnostics(const whole_body_diagnostics_v2 &, double, bool) {}
void RecordWholeBodyDiagnostics(const whole_body_diagnostics_v2 &,
    const whole_body_motor_command_diagnostics_v2 &, bool) {}
}  // namespace driver_runtime

int main(int argc, char **argv) {
    assert(argc == 2);
    char fixed_config[] = "/tmp/hardware-fixed-base-XXXXXX";
    const int fd = mkstemp(fixed_config);
    assert(fd >= 0);
    close(fd);
    auto yaml = YAML::LoadFile(argv[1]);
    for (int variant = 0; variant < 3; ++variant) {
        if (variant == 0) yaml["robot_base"].remove("fixed_base");
        else yaml["robot_base"]["fixed_base"] = variant == 2;
        std::ofstream(fixed_config) << yaml;
        scenario = {};
        scenario.has_imu = 0;
        bool rejected = false;
        try {
            (void)driver_runtime::CreateWholeBodyBackend(fixed_config);
        } catch (const std::runtime_error &error) {
            rejected = true;
            assert(std::string(error.what()).find("robot_base.fixed_base: true") != std::string::npos);
        }
        assert(rejected == (variant != 2));
        assert(scenario.destroys == 1 && scenario.init_calls == 0);
    }
    for (int variant = 0; variant < 4; ++variant) {
        const bool fail_init = (variant & 1) != 0;
        scenario = {};
        scenario.has_imu = (variant & 2) == 0 ? 1 : 0;
        scenario.init_error = fail_init ? WHOLE_BODY_ERR_DEVICE : WHOLE_BODY_OK;
        scenario.offline_reads = 3;
        auto backend = driver_runtime::CreateWholeBodyBackend(scenario.has_imu ? argv[1] : fixed_config);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
        assert(backend->Run(Publish, Receive, [&] {
            return scenario.enabled_writes < 2 && std::chrono::steady_clock::now() < deadline;
        }) == 0);
        assert(scenario.enabled_writes == 2);
        assert(scenario.fault_events == 1 && scenario.recoveries == 1);
        assert(scenario.safety_requests >= 1);
        assert(scenario.init_calls == (fail_init ? 2 : 1));
        assert(scenario.publications > scenario.reads * 5);
        assert(scenario.command_polls >= scenario.reads);
        // Tick's first DEVICE_ERROR is retained instead of a later read timeout.
        assert(scenario.saw_motor_feedback_fault || scenario.saw_motor_device_fault || fail_init);
        if (!fail_init) assert(scenario.saw_motor_device_fault && !scenario.saw_motor_feedback_fault);
    }

    for (int has_imu : {0, 1}) {
        scenario = {};
        scenario.has_imu = has_imu;
        scenario.initial_feedback_wait = false;
        scenario.offline_reads = 1;
        scenario.reject_publication = true;
        auto backend = driver_runtime::CreateWholeBodyBackend(has_imu ? argv[1] : fixed_config);
        assert(backend->Run(Publish, Receive, [] { return true; }) == 1);
        assert(scenario.reads == 1 && scenario.saw_motor_feedback_fault);
        assert(scenario.last_fault.source == robot_base::FaultSource::MOTOR);
        assert(scenario.last_fault.code == robot_base::FaultCode::FEEDBACK_TIMEOUT);
        assert(!scenario.saw_motor_device_fault);
    }

    scenario = {};
    scenario.has_imu = WHOLE_BODY_ERR_CONFIG;
    bool rejected = false;
    try {
        (void)driver_runtime::CreateWholeBodyBackend(argv[1]);
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    assert(rejected && scenario.init_calls == 0 && scenario.destroys == 1);

    scenario = {};
    scenario.repeat_outages = true;
    auto backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    assert(backend->Run(Publish, Receive, [&] {
        return scenario.enabled_writes < 6 && std::chrono::steady_clock::now() < deadline;
    }) == 0);
    assert(scenario.enabled_writes == 6);
    assert(scenario.fault_events == 2 && scenario.recoveries == 2);

    scenario = {};
    scenario.init_error = WHOLE_BODY_ERR_CONFIG;
    backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
    assert(backend->Run(Publish, Receive, [] { return true; }) == 1);
    assert(scenario.init_calls == 1);

    scenario = {};
    scenario.reject_publication = true;
    backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
    assert(backend->Run(Publish, Receive, [] { return true; }) == 1);
    assert(scenario.enabled_writes == 0);
    std::remove(fixed_config);
    std::cout << "Hardware outage, recovery, and command rearm tests passed\n";
}
