/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_hardware_recovery.cpp
 * @brief Exercise runtime recovery without accessing hardware or robot transport
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
    int init_error = WHOLE_BODY_OK;
    int offline_reads = 0;
    int reads = 0;
    int enabled_writes = 0;
    int power_off_writes = 0;
    int fault_events = 0;
    int recoveries = 0;
    int fresh_reports = 0;
    int publications = 0;
    int command_polls = 0;
    int tick_error = WHOLE_BODY_OK;
    int retry_reads = 0;
    int retry_writes = 0;
    int retry_ticks = 0;
    int held_reports = 0;
    bool offline = false;
    bool last_fault_active = false;
    bool needs_rearm = false;
    bool repeat_outages = false;
    bool fail_command = false;
    bool reject_publication = false;
    bool needs_query_reply = false;
    bool withhold_power_off = false;
    double device_time = 0.0;
    robot_base::FaultStatus last_fault;
    std::chrono::steady_clock::time_point last_offline_read;
    std::chrono::steady_clock::time_point last_query;
};

Scenario scenario;

bool Publish(const robot_base::RobotData &state, const robot_base::FaultStatus &fault) {
    assert(state.IsValid());
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
    if (scenario.retry_reads > 0 && scenario.reads >= 3) {
        assert(!fault.active && !fault.latched);
        ++scenario.held_reports;
    }
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
    if (scenario.needs_rearm && scenario.fresh_reports >= 8 && !scenario.withhold_power_off) {
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

// The real backend runs against these C API fixtures, never real peripherals.
extern "C" {
int whole_body_create(const char *, whole_body_dev **device) {
    *device = new whole_body_dev;
    return WHOLE_BODY_OK;
}

void whole_body_destroy(whole_body_dev *device) { delete device; }

int whole_body_has_imu(whole_body_dev *) { return scenario.has_imu; }

int whole_body_init(whole_body_dev *) { return scenario.init_error; }

int whole_body_get_cycle_s(const whole_body_dev *, double *cycle_s) {
    *cycle_s = 0.002;
    return WHOLE_BODY_OK;
}

int whole_body_read(whole_body_dev *device, whole_body_state *state) {
    ++scenario.reads;
    scenario.offline = scenario.offline_reads > 0;
    if (scenario.offline) {
        const auto now = std::chrono::steady_clock::now();
        scenario.last_offline_read = now;
        scenario.needs_query_reply = true;
        --scenario.offline_reads;
        device->safety_latched = true;
        return WHOLE_BODY_ERR_TIMEOUT;
    }
    if (scenario.needs_query_reply) {
        const auto age = std::chrono::steady_clock::now() - scenario.last_query;
        if (age > std::chrono::milliseconds(40)) return WHOLE_BODY_ERR_TIMEOUT;
        scenario.needs_query_reply = false;
    }
    *state = {};
    state->num_dof = 2;
    state->base_quat[0] = 1.0;
    if (scenario.retry_reads > 0 && scenario.reads >= 3) {
        --scenario.retry_reads;
        state->timestamp_s = scenario.device_time;
        return WHOLE_BODY_RETRY;
    }
    state->timestamp_s = scenario.device_time += 0.002;
    return WHOLE_BODY_OK;
}

int whole_body_write(whole_body_dev *device, const whole_body_joint_command *command) {
    assert(!scenario.offline);
    if (command->enable) {
        assert(!scenario.needs_rearm && !device->safety_latched);
        if (scenario.retry_writes > 0 && scenario.enabled_writes >= 2) {
            --scenario.retry_writes;
            return WHOLE_BODY_RETRY;
        }
        if (scenario.fail_command) {
            scenario.fail_command = false;
            device->safety_latched = true;
            // The first disabled write succeeds, then Tick loses a reply.
            scenario.tick_error = WHOLE_BODY_ERR_DEVICE;
            return WHOLE_BODY_ERR_DEVICE;
        }
        ++scenario.enabled_writes;
        if (scenario.repeat_outages &&
            (scenario.enabled_writes == 2 || scenario.enabled_writes == 4)) {
            scenario.last_offline_read = {};
            scenario.offline_reads = 3;
        }
    } else if (command->mode == WHOLE_BODY_MODE_POWER_OFF) {
        ++scenario.power_off_writes;
        device->safety_latched = false;
    }
    return WHOLE_BODY_OK;
}

int whole_body_tick(whole_body_dev *, double) {
    if (scenario.retry_ticks > 0 && scenario.enabled_writes >= 2) {
        --scenario.retry_ticks;
        return WHOLE_BODY_RETRY;
    }
    if (scenario.tick_error != WHOLE_BODY_OK) {
        const int result = scenario.tick_error;
        scenario.tick_error = WHOLE_BODY_OK;
        return result;
    }
    if (!scenario.offline) scenario.last_query = std::chrono::steady_clock::now();
    return scenario.offline ? WHOLE_BODY_ERR_DEVICE : WHOLE_BODY_OK;
}

int whole_body_set_mode(whole_body_dev *device, whole_body_mode mode) {
    assert(mode == WHOLE_BODY_MODE_SAFETY);
    device->safety_latched = true;
    return scenario.offline ? WHOLE_BODY_ERR_DEVICE : WHOLE_BODY_OK;
}

int whole_body_get_diagnostics_v2(const whole_body_dev *, whole_body_diagnostics_v2 *diag) {
    *diag = {};
    diag->motor_count = 1;
    diag->imu.feedback_received = true;
    diag->imu.feedback_fresh = true;
    return WHOLE_BODY_OK;
}

int whole_body_get_motor_command_diagnostics_v2(
    const whole_body_dev *, whole_body_motor_command_diagnostics_v2 *) {
    return WHOLE_BODY_OK;
}

int whole_body_get_transport_diagnostics(
    const whole_body_dev *, whole_body_transport_diagnostics *diagnostics) {
    *diagnostics = {};
    return WHOLE_BODY_OK;
}

const char *whole_body_last_error(const whole_body_dev *) { return "motor communication timeout"; }
}  // extern "C"

namespace driver_runtime {
std::unique_ptr<DriverBackend> CreateWholeBodyBackend(const std::string &yaml_path);
void RenderWholeBodyDiagnostics(const whole_body_diagnostics_v2 &, double, bool) {}
void RecordWholeBodyDiagnostics(const whole_body_diagnostics_v2 &,
    const whole_body_motor_command_diagnostics_v2 &, double, bool) {}
void RecordTransportDiagnostics(const whole_body_transport_diagnostics &, double) {}
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
    }
    std::remove(fixed_config);
    // Bounded read/write/tick delays do not latch a fault or skip feedback reads.
    for (int variant = 0; variant < 3; ++variant) {
        scenario = {};
        scenario.retry_reads = variant == 0 ? 12 : 0;
        scenario.retry_writes = variant == 1 ? 12 : 0;
        scenario.retry_ticks = variant == 2 ? 12 : 0;
        auto backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
        assert(backend->Run(Publish, Receive, [] { return scenario.command_polls < 30; }) == 0);
        assert(scenario.retry_reads == 0 && scenario.retry_writes == 0 && scenario.retry_ticks == 0);
        assert(scenario.fault_events == 0 && scenario.power_off_writes == 0);
        assert(scenario.enabled_writes >= 18 && !scenario.last_fault.latched);
        assert(scenario.command_polls == scenario.reads);  // No retry/backoff cycles skipped reads.
        if (variant == 0) assert(scenario.held_reports > 0);
    }
    // Read outage, command+disabled-write timeout, and repeated outages.
    for (int variant = 0; variant < 3; ++variant) {
        scenario = {};
        scenario.offline_reads = variant == 0 ? 3 : 0;
        scenario.fail_command = variant == 1;
        scenario.repeat_outages = variant == 2;
        const int expected_writes = variant == 2 ? 6 : 2;
        auto backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        assert(backend->Run(Publish, Receive, [&] {
            return scenario.enabled_writes < expected_writes &&
                std::chrono::steady_clock::now() < deadline;
        }) == 0);
        assert(scenario.enabled_writes == expected_writes);
        assert(scenario.fault_events == (variant == 2 ? 2 : 1));
        assert(scenario.recoveries == scenario.fault_events);
        assert(scenario.command_polls == scenario.reads);
        assert(scenario.publications >= scenario.reads);
    }
    // Communication returns, but stale RL commands cannot re-enable motors.
    scenario = {};
    scenario.fail_command = true;
    scenario.withhold_power_off = true;
    auto backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    assert(backend->Run(Publish, Receive, [&] {
        return std::chrono::steady_clock::now() < deadline;
    }) == 0);
    assert(scenario.recoveries == 1 && scenario.enabled_writes == 0);
    assert(scenario.last_fault.latched && !scenario.last_fault.active);

    // Permanent loss keeps publishing a fault until normal shutdown.
    scenario = {};
    scenario.offline_reads = 100;
    backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
    assert(backend->Run(Publish, Receive, [] { return scenario.command_polls < 80; }) == 0);
    assert(scenario.last_fault.active && scenario.enabled_writes == 0);
    assert(scenario.publications >= 80 && scenario.reads == 80);

    for (int error : {WHOLE_BODY_ERR_CONFIG, WHOLE_BODY_ERR_DEVICE}) {
        scenario = {};
        scenario.init_error = error;
        backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
        assert(backend->Run(Publish, Receive, [] { return true; }) ==
            (error == WHOLE_BODY_ERR_DEVICE ? driver_runtime::kHardwareInitUnavailable : 1));
    }
    scenario = {};
    scenario.reject_publication = true;
    backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
    assert(backend->Run(Publish, Receive, [] { return true; }) == 1);
    assert(scenario.enabled_writes == 0);

    scenario = {};
    scenario.tick_error = WHOLE_BODY_ERR_CONFIG;
    backend = driver_runtime::CreateWholeBodyBackend(argv[1]);
    assert(backend->Run(Publish, Receive, [] { return true; }) == 1);
    std::cout << "Hardware runtime recovery and command rearm tests passed\n";
}
